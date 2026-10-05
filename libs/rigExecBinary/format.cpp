// The .rigexec FlatBuffer: Open refuses, bounds, verifies, unpacks and
// validates; Write validates and packs; the validator holds every rule a
// file must satisfy that the file alone can decide.
#include "rigExecBinary/format.h"

#include "rigExecBinary/stepGraph.h"

#include <algorithm>
#include <cstring>
#include <new>
#include <string_view>
#include <tuple>
#include <unordered_set>
#include <utility>

namespace rigExec {
namespace {

using fb::InputTag;
using fb::PathKind;
using fb::ReadMode;
using fb::RigExecWireFile;
using fb::RigExecWireInput;

/// The buffer Open verifies from: 16-byte aligned whatever the caller's
/// bytes were, so the verifier's alignment checks and the nested buffers'
/// force_align hold.
struct alignas(16) _AlignedBlock {
    uint8_t bytes[16];
};

/// Table nesting the verifier and the bounding walk admit.
constexpr unsigned _MaxDepth = 64;

/// The verifier limits for a buffer of \p size bytes. A large rig holds a
/// few hundred thousand tables (one per read); an unshared table costs at
/// least 8 bytes (its vtable offset and the offset naming it), so a table
/// count past size / 8 means shared tables. The nested presentation is
/// verified by the validator, after the bounding walk.
flatbuffers::Verifier::Options
_VerifierOptions(size_t size)
{
    flatbuffers::Verifier::Options options;
    options.max_depth = _MaxDepth;
    options.max_tables = flatbuffers::uoffset_t(
        std::min<size_t>(size / 8 + 1, size_t(1) << 26));
    options.check_alignment = true;
    options.check_nested_flatbuffers = false;
    options.max_size = FLATBUFFERS_MAX_BUFFER_SIZE;
    return options;
}

/// Bounds the work the verifier and the object API do on a buffer before
/// either reads it. Every table, vector and string a field reaches is
/// charged its bytes (a table its vtable's object size, at least 4), once
/// per reference, and the charges may not exceed the buffer: Write never
/// shares or overlaps objects, so its buffers always fit, while offsets
/// that share or overlap objects would make UnPack copy them once per
/// reference, without bound. Field types come from the generated type
/// tables; every read is bounds-checked first, so the walk is safe on
/// unverified bytes and stops after at most size / 4 objects.
class _Budget {
public:
    _Budget(const uint8_t *buffer, size_t size)
        : _buffer(buffer), _size(size), _left(size)
    {
    }

    /// Walks the root table of type \p type.
    bool Root(const flatbuffers::TypeTable *type)
    {
        size_t root = 0;
        return _Target(0, &root) && _Table(root, type, 1);
    }

private:
    template <class T>
    bool _Load(size_t at, T *value) const
    {
        if (at > _size || _size - at < sizeof(T)) {
            return false;
        }
        std::memcpy(value, _buffer + at, sizeof(T));
        *value = flatbuffers::EndianScalar(*value);
        return true;
    }

    bool _Charge(size_t bytes)
    {
        if (bytes > _left) {
            return false;
        }
        _left -= bytes;
        return true;
    }

    /// The object the offset stored at \p at names, under the verifier's
    /// offset rules (non-zero, below 2^31, inside the buffer).
    bool _Target(size_t at, size_t *target) const
    {
        uint32_t offset = 0;
        if (!_Load(at, &offset) || offset == 0 || offset > 0x7fffffffu) {
            return false;
        }
        *target = at + offset;
        return *target < _size;
    }

    bool _String(size_t at)
    {
        uint32_t length = 0;
        if (!_Load(at, &length) || length >= _size - at - 4) {
            return false;
        }
        return _Charge(4 + size_t(length) + 1);
    }

    bool _Vector(size_t at, size_t elementSize, uint32_t *count)
    {
        if (!_Load(at, count) || *count > (_size - at - 4) / elementSize) {
            return false;
        }
        return _Charge(4 + size_t(*count) * elementSize);
    }

    static size_t _ElementSize(flatbuffers::ElementaryType type,
                               const flatbuffers::TypeTable *ref)
    {
        switch (type) {
        case flatbuffers::ET_SHORT:
        case flatbuffers::ET_USHORT:
            return 2;
        case flatbuffers::ET_INT:
        case flatbuffers::ET_UINT:
        case flatbuffers::ET_FLOAT:
        case flatbuffers::ET_STRING:
            return 4;
        case flatbuffers::ET_LONG:
        case flatbuffers::ET_ULONG:
        case flatbuffers::ET_DOUBLE:
            return 8;
        case flatbuffers::ET_SEQUENCE:
            // A struct's type table ends its offsets with its size; tables
            // are stored by offset.
            return ref->st == flatbuffers::ST_STRUCT
                       ? size_t(ref->values[ref->num_elems])
                       : 4;
        default:
            return 1;
        }
    }

    bool _Table(size_t at, const flatbuffers::TypeTable *type, unsigned depth)
    {
        int32_t back = 0;
        uint16_t vtableSize = 0, objectSize = 0;
        if (depth > _MaxDepth || !_Load(at, &back)) {
            return false;
        }
        const int64_t vtable = int64_t(at) - int64_t(back);
        if (vtable < 0 || !_Load(size_t(vtable), &vtableSize) ||
            !_Load(size_t(vtable) + 2, &objectSize) ||
            !_Charge(std::max<size_t>(objectSize, 4))) {
            return false;
        }
        for (size_t i = 0; i < type->num_elems; ++i) {
            const flatbuffers::TypeCode code = type->type_codes[i];
            const auto base = flatbuffers::ElementaryType(code.base_type);
            const flatbuffers::TypeTable *ref =
                code.sequence_ref >= 0 ? type->type_refs[code.sequence_ref]()
                                       : nullptr;
            const bool isString = base == flatbuffers::ET_STRING;
            const bool isTable = base == flatbuffers::ET_SEQUENCE &&
                                 ref->st == flatbuffers::ST_TABLE;
            if (base == flatbuffers::ET_SEQUENCE &&
                ref->st == flatbuffers::ST_UNION) {
                return false;  // the schema has none
            }
            if (!code.is_repeating && !isString && !isTable) {
                continue;  // an inline scalar or struct
            }
            // The field's vtable entry; past the vtable or 0 means absent.
            const size_t entry = 4 + 2 * i;
            uint16_t field = 0;
            if (entry + 2 > vtableSize) {
                continue;
            }
            if (!_Load(size_t(vtable) + entry, &field)) {
                return false;
            }
            if (field == 0) {
                continue;
            }
            size_t target = 0;
            if (!_Target(at + field, &target)) {
                return false;
            }
            if (!code.is_repeating) {
                if (isString ? !_String(target)
                             : !_Table(target, ref, depth + 1)) {
                    return false;
                }
                continue;
            }
            uint32_t count = 0;
            if (!_Vector(target, _ElementSize(base, ref), &count)) {
                return false;
            }
            if (!isString && !isTable) {
                continue;
            }
            for (uint32_t k = 0; k < count; ++k) {
                size_t element = 0;
                if (!_Target(target + 4 + 4 * size_t(k), &element) ||
                    (isString ? !_String(element)
                              : !_Table(element, ref, depth + 1))) {
                    return false;
                }
            }
        }
        return true;
    }

    const uint8_t *_buffer;
    size_t _size;
    size_t _left;
};

bool
_Fail(std::string *error, std::string what)
{
    if (error) {
        *error = std::move(what);
    }
    return false;
}

std::string
_N(size_t value)
{
    return std::to_string(value);
}

std::string
_At(const std::string &row, const char *field, long index = -1)
{
    std::string out = row;
    if (field && *field) {
        out += '.';
        out += field;
    }
    if (index >= 0) {
        out += '[' + std::to_string(index) + ']';
    }
    return out;
}

// Path-id roles: which kinds of node a field may name.
constexpr unsigned _Zero = 1u << 0;  // 0, "none"
constexpr unsigned _Prim = 1u << 1;
constexpr unsigned _Property = 1u << 2;
constexpr unsigned _Token = 1u << 3;
constexpr unsigned _AnyPath = _Zero | _Prim | _Property | _Token;
constexpr unsigned _TokenOrZero = _Zero | _Token;
constexpr unsigned _PrimOrZero = _Zero | _Prim;

// Read modes a site admits.
constexpr unsigned _Baked = 1u << unsigned(ReadMode::Baked);
constexpr unsigned _Resolved = 1u << unsigned(ReadMode::Resolved);
constexpr unsigned _Pinned = 1u << unsigned(ReadMode::Pinned);
constexpr unsigned _AnyMode = 0xfu;

/// Any tag, for a read whose type the site does not fix.
constexpr int _AnyTag = -1;

uint8_t
_Flags(fb::InputReadFlags flag)
{
    return uint8_t(flag);
}

/// The tag of a property chain revision's value, min and max reads: a
/// double chain is computed in float.
InputTag
_ChainReadTag(fb::PropertyValueType type)
{
    switch (type) {
    case fb::PropertyValueType::Matrix4d:
        return InputTag::Matrix4d;
    case fb::PropertyValueType::Vec3f:
        return InputTag::Vec3f;
    default:
        return InputTag::Float;
    }
}

bool
_IsScalar(fb::PropertyValueType type)
{
    return type == fb::PropertyValueType::Float ||
           type == fb::PropertyValueType::Double;
}

class _Validator {
public:
    explicit _Validator(const RigExecWireFile &file) : _f(file) {}

    bool Run(std::string *error)
    {
        if (!_Run()) {
            return _Fail(error, _error);
        }
        return true;
    }

private:
    bool _Run()
    {
        return _Root() && _Paths() && _Values() && _Pools() && _Slots() &&
               _SlotMeta() && _Constants() && _Steps() && _Cones() &&
               _Pose() && _Geometry() && _PropertyChains() && _External() &&
               _Overrides() && _Presentation();
    }

    bool _Bad(std::string what)
    {
        _error = std::move(what);
        return false;
    }

    // ----------------------------------------------------------- helpers

    bool _PathId(uint32_t id, unsigned roles, const std::string &row,
                 const char *field, long index = -1)
    {
        if (id >= _f.paths.size()) {
            return _Bad(_At(row, field, index) + ": path id " + _N(id) +
                        " out of range (" + _N(_f.paths.size()) +
                        " paths)");
        }
        unsigned role = _Zero;
        if (id != 0) {
            switch (_f.paths[id].kind()) {
            case PathKind::Prim:
                role = _Prim;
                break;
            case PathKind::Property:
                role = _Property;
                break;
            case PathKind::Token:
                role = _Token;
                break;
            default:
                role = 0;
                break;
            }
        }
        if (!(roles & role)) {
            return _Bad(_At(row, field, index) + ": path id " + _N(id) +
                        " is not a " + _RoleNames(roles));
        }
        return true;
    }

    template <class Ids>
    bool _PathIds(const Ids &ids, unsigned roles, const std::string &row,
                  const char *field)
    {
        for (size_t k = 0; k < ids.size(); ++k) {
            if (!_PathId(ids[k], roles, row, field, long(k))) {
                return false;
            }
        }
        return true;
    }

    static std::string _RoleNames(unsigned roles)
    {
        std::string out;
        const auto add = [&](unsigned bit, const char *name) {
            if (roles & bit) {
                out += out.empty() ? "" : " or ";
                out += name;
            }
        };
        add(_Zero, "0");
        add(_Prim, "prim");
        add(_Property, "property");
        add(_Token, "token");
        return out;
    }

    /// An index into a table of \p count entries, or -1 when \p none.
    bool _Index(int64_t value, size_t count, bool none, const std::string &row,
                const char *field, long index = -1)
    {
        if ((none && value == -1) ||
            (value >= 0 && uint64_t(value) < uint64_t(count))) {
            return true;
        }
        return _Bad(_At(row, field, index) + ": " + std::to_string(value) +
                    " out of range (" + _N(count) + ")");
    }

    template <class Values>
    bool _Indices(const Values &values, size_t count, bool none,
                  const std::string &row, const char *field)
    {
        for (size_t k = 0; k < values.size(); ++k) {
            if (!_Index(int64_t(values[k]), count, none, row, field,
                        long(k))) {
                return false;
            }
        }
        return true;
    }

    bool _Bools(const std::vector<uint8_t> &values, const std::string &row,
                const char *field)
    {
        for (size_t k = 0; k < values.size(); ++k) {
            if (values[k] > 1) {
                return _Bad(_At(row, field, long(k)) + ": " +
                            _N(values[k]) + " is not 0 or 1");
            }
        }
        return true;
    }

    bool _Size(size_t size, size_t want, const std::string &row,
               const char *field)
    {
        if (size != want) {
            return _Bad(_At(row, field) + ": " + _N(size) +
                        " entries, expected " + _N(want));
        }
        return true;
    }

    bool _Pool(uint32_t id, size_t poolSize, const std::string &row,
               const char *field)
    {
        if (id >= poolSize) {
            return _Bad(_At(row, field) + ": pool id " + _N(id) +
                        " out of range (" + _N(poolSize) + ")");
        }
        return true;
    }

    /// Every rule of one bound read: tag, mode and flags in range, the
    /// constant a value of the read's tag, the walk inside the slots, the
    /// pinned hop inside the walk and only where a Baked varying read
    /// takes it, override numbers only on Baked reads, a Raw read over one
    /// slot. \p tag and \p modes are what the site admits.
    bool _Read(const RigExecWireInput *input, int tag, unsigned modes,
               const std::string &row, const char *field, long index = -1)
    {
        const auto where = [&] { return _At(row, field, index); };
        if (!input) {
            return _Bad(where() + ": missing");
        }
        if (input->tag > InputTag::MAX || input->mode > ReadMode::MAX ||
            (input->flags & ~_Flags(fb::InputReadFlags::ANY)) != 0) {
            return _Bad(where() + ": tag, mode or flags out of range");
        }
        if (tag != _AnyTag && input->tag != InputTag(tag)) {
            return _Bad(where() + ": tag " + _N(size_t(input->tag)) +
                        ", expected " + _N(size_t(tag)));
        }
        if (!(modes & (1u << unsigned(input->mode)))) {
            return _Bad(where() + ": read mode " +
                        _N(size_t(input->mode)) + " is not admitted here");
        }
        if (input->constant >= _f.values.size() ||
            _f.values[input->constant].tag != input->tag) {
            return _Bad(where() + ": constant " + _N(input->constant) +
                        " is not a value of the read's tag");
        }
        if (input->walk.size() > 32767) {
            return _Bad(where() + ": walk too long");
        }
        for (size_t k = 0; k < input->walk.size(); ++k) {
            if (input->walk[k] >= _f.inputs.size()) {
                return _Bad(where() + ".walk[" + _N(k) + "]: slot " +
                            _N(input->walk[k]) + " out of range (" +
                            _N(_f.inputs.size()) + " slots)");
            }
        }
        const bool baked = input->mode == ReadMode::Baked;
        const bool varying =
            (input->flags & _Flags(fb::InputReadFlags::Varying)) != 0;
        const bool longWay =
            (input->flags & _Flags(fb::InputReadFlags::LongWay)) != 0;
        if (input->selected < -1 ||
            (input->selected >= 0 &&
             (size_t(input->selected) >= input->walk.size() || !baked ||
              !varying || longWay))) {
            return _Bad(where() + ": selected " +
                        std::to_string(input->selected) +
                        " is not a pinned hop of a Baked varying read");
        }
        if (!baked && longWay) {
            return _Bad(where() + ": LongWay on a read that is not Baked");
        }
        if (input->mode == ReadMode::Raw && input->walk.size() != 1) {
            return _Bad(where() + ": a Raw read walks exactly one slot");
        }
        if (input->overrideIndex != -1) {
            if (!baked || input->overrideIndex < 0 ||
                uint32_t(input->overrideIndex) >= _overrideCount ||
                input->walk.empty()) {
                return _Bad(where() + ": override number " +
                            std::to_string(input->overrideIndex) +
                            " on a read that cannot hold it (" +
                            _N(_overrideCount) + " override numbers)");
            }
            _overrideUses.push_back(uint32_t(input->overrideIndex));
        }
        return true;
    }

    bool _ReadPtr(const std::unique_ptr<RigExecWireInput> &input, int tag,
                  unsigned modes, const std::string &row, const char *field)
    {
        return _Read(input.get(), tag, modes, row, field);
    }

    bool _Phase(const RigExecWireReadPhase &phase, const std::string &row,
                const char *field, long index = -1)
    {
        if (phase.kind > uint8_t(fb::ReadPhaseKind::MAX)) {
            return _Bad(_At(row, field, index) + ": read phase kind " +
                        _N(phase.kind) + " out of range");
        }
        return _PathId(phase.prim, _AnyPath, row, field, index);
    }

    /// A space switch's read version: present, its anchor -1 or a slot,
    /// and each recompose entry a slot the compose has avars for.
    bool _FrameVersion(const fb::RigExecWireFrameVersion *read,
                       const std::string &where)
    {
        if (!read) {
            return _Bad(where + ": missing");
        }
        if (!_Index(read->anchor, _slots, true, where, "anchor") ||
            !_Indices(read->recompose, _slots, false, where, "recompose")) {
            return false;
        }
        for (size_t k = 0; k < read->recompose.size(); ++k) {
            if (_f.slotMeta->slotKind[size_t(read->recompose[k])] !=
                fb::SlotKind::FirstFramePose) {
                return _Bad(_At(where, "recompose", long(k)) +
                            ": not a FirstFramePose slot");
            }
        }
        return true;
    }

    /// The version of a world source or of a missing space: the compose
    /// never reads it, and the program leaves it at {-1, []}.
    bool _UnreadVersion(const fb::RigExecWireFrameVersion &read,
                        const std::string &where)
    {
        if (read.anchor != -1 || !read.recompose.empty()) {
            return _Bad(where + ": reads no slot, so its version is "
                                "{-1, []}");
        }
        return true;
    }

    bool _ClusterSet(const fb::RigExecWireClusterSet *set,
                     const std::string &where)
    {
        if (!set) {
            return _Bad(where + ": missing");
        }
        if (set->clusters != _clusters ||
            set->words.size() != (_clusters + 63) / 64) {
            return _Bad(where + ": covers " + _N(set->clusters) +
                        " clusters in " + _N(set->words.size()) +
                        " words; the program has " + _N(_clusters));
        }
        if (_clusters % 64 != 0 &&
            (set->words.back() >> (_clusters % 64)) != 0) {
            return _Bad(where + ": bits set past the last cluster");
        }
        return true;
    }

    // -------------------------------------------------------- the rules

    bool _Root()
    {
        if (_f.formatVersion != RigExecFormatVersion) {
            return _Bad("format_version is " + _N(_f.formatVersion) +
                        "; this reader reads " + _N(RigExecFormatVersion));
        }
        if (!_f.slotMeta || !_f.constants || !_f.clustering || !_f.cones ||
            !_f.pose || !_f.geometry) {
            return _Bad("a required root table is missing");
        }
        _slots = _f.slotMeta->paths.size();
        _clusters = _f.clustering->clusters.size();
        _steps = _f.steps.size();
        _chains = _f.geometry->chains.size();
        for (const auto &chain : _f.geometry->chains) {
            _revisions += chain.revisions.size();
            _derived += chain.derived.size();
        }
        _weights = _f.geometry->weightObjects.size();
        while (_stepBacked < _weights &&
               !_f.geometry->weightObjects[_stepBacked].envelopeOnly) {
            ++_stepBacked;
        }
        _overrideCount = _f.pose->overrideCount;
        return true;
    }

    /// Whether text(a) < text(b) byte-wise, for prim and property nodes of a
    /// valid tree, without composing either text: both walk up to their
    /// common ancestor, then the first differing byte decides. Over a
    /// sorted sequence the walks visit each node at most twice in all.
    bool _TextLess(uint32_t a, uint32_t b) const
    {
        constexpr uint32_t none = UINT32_MAX;
        // x and y climb from a and b; the two nodes below each, on its
        // path, are kept for the bytes that follow the common prefix.
        uint32_t x = a, xChild = none, xGrandchild = none;
        uint32_t y = b, yChild = none, yGrandchild = none;
        const auto up = [&](uint32_t &at, uint32_t &child,
                            uint32_t &grandchild) {
            grandchild = child;
            child = at;
            at = _f.paths[at].parent();
        };
        while (_depth[x] > _depth[y]) {
            up(x, xChild, xGrandchild);
        }
        while (_depth[y] > _depth[x]) {
            up(y, yChild, yGrandchild);
        }
        if (x == y) {
            // Equal, or one is an ancestor and its text a proper prefix.
            return x == a && a != b;
        }
        while (x != y) {
            up(x, xChild, xGrandchild);
            up(y, yChild, yGrandchild);
        }
        // xChild and yChild are distinct children of the common ancestor:
        // the texts agree up to their separators.
        const auto separator = [&](uint32_t node) {
            return _f.paths[node].kind() == PathKind::Prim ? '/' : '.';
        };
        if (separator(xChild) != separator(yChild)) {
            return separator(xChild) < separator(yChild);
        }
        const std::string &nameA = _f.names[_f.paths[xChild].name()];
        const std::string &nameB = _f.names[_f.paths[yChild].name()];
        const size_t common = std::min(nameA.size(), nameB.size());
        const int order = std::memcmp(nameA.data(), nameB.data(), common);
        if (order != 0) {
            return order < 0;
        }
        // Names are unique, so one is a proper prefix of the other; the
        // shorter text continues with a separator, or ends (-1).
        const auto next = [&](uint32_t grandchild) {
            return grandchild == none ? -1 : int(separator(grandchild));
        };
        if (nameA.size() < nameB.size()) {
            return next(xGrandchild) < int((unsigned char)nameB[common]);
        }
        if (nameB.size() < nameA.size()) {
            return int((unsigned char)nameA[common]) < next(yGrandchild);
        }
        return false;
    }

    /// The path tree: names and nodes, every node unique and canonical so
    /// equal ids mean equal text; then every node's depth.
    bool _Paths()
    {
        if (_f.names.empty() || !_f.names[0].empty()) {
            return _Bad("names[0] is not the empty string");
        }
        std::unordered_set<std::string_view> names;
        names.reserve(_f.names.size());
        for (size_t k = 1; k < _f.names.size(); ++k) {
            if (_f.names[k].empty() ||
                !names.insert(std::string_view(_f.names[k])).second) {
                return _Bad("names[" + _N(k) + "] is empty or repeated");
            }
        }
        if (_f.paths.empty() || _f.paths[0].parent() != 0 ||
            _f.paths[0].name() != 0 || _f.paths[0].kind() != PathKind::None) {
            return _Bad("paths[0] is not {0, 0, None}");
        }
        std::vector<std::tuple<uint32_t, uint32_t, uint8_t>> keys;
        keys.reserve(_f.paths.size());
        for (size_t i = 1; i < _f.paths.size(); ++i) {
            const fb::PathNode &node = _f.paths[i];
            const std::string where = "paths[" + _N(i) + "]";
            if (node.name() == 0 || node.name() >= _f.names.size()) {
                return _Bad(where + ": name " + _N(node.name()) +
                            " out of range");
            }
            const std::string &name = _f.names[node.name()];
            const bool component =
                name.find_first_of("/.") == std::string::npos;
            switch (node.kind()) {
            case PathKind::Prim:
                if (node.parent() >= i ||
                    (node.parent() != 0 &&
                     _f.paths[node.parent()].kind() != PathKind::Prim) ||
                    !component) {
                    return _Bad(where + ": malformed prim node");
                }
                break;
            case PathKind::Property:
                if (node.parent() == 0 || node.parent() >= i ||
                    _f.paths[node.parent()].kind() != PathKind::Prim ||
                    !component) {
                    return _Bad(where + ": malformed property node");
                }
                break;
            case PathKind::Token:
                if (node.parent() != 0) {
                    return _Bad(where + ": a token node has a parent");
                }
                break;
            default:
                return _Bad(where + ": kind " +
                            _N(size_t(node.kind())) + " out of range");
            }
            keys.emplace_back(node.parent(), node.name(),
                              uint8_t(node.kind()));
        }
        std::sort(keys.begin(), keys.end());
        if (std::adjacent_find(keys.begin(), keys.end()) != keys.end()) {
            return _Bad("paths: two nodes name the same path");
        }
        // Depths, not texts: composing every text costs the square of the
        // tree's depth.
        _depth.assign(_f.paths.size(), 0);
        for (size_t i = 1; i < _f.paths.size(); ++i) {
            _depth[i] = _depth[_f.paths[i].parent()] + 1;
        }
        return _PathId(_f.rig, _Prim, "file", "rig");
    }

    bool _Values()
    {
        if (_f.values.empty() || _f.values[0].tag != InputTag::Double ||
            _f.values[0].bits != 0) {
            return _Bad("values[0] is not Double +0.0");
        }
        for (size_t i = 0; i < _f.values.size(); ++i) {
            const fb::RigExecWireValue &value = _f.values[i];
            const std::string where = "values[" + _N(i) + "]";
            if (value.tag > InputTag::MAX) {
                return _Bad(where + ": tag out of range");
            }
            if (bool(value.matrix) != (value.tag == InputTag::Matrix4d) ||
                bool(value.vec3d) != (value.tag == InputTag::Vec3d) ||
                bool(value.vec3f) != (value.tag == InputTag::Vec3f)) {
                return _Bad(where + ": its members do not match its tag");
            }
            bool ok = true;
            switch (value.tag) {
            case InputTag::Bool:
                ok = value.bits <= 1;
                break;
            case InputTag::Float:
            case InputTag::Int:
                ok = (value.bits >> 32) == 0;
                break;
            case InputTag::Token:
                ok = (value.bits >> 32) == 0;
                if (ok && !_PathId(uint32_t(value.bits), _TokenOrZero, where,
                                   "bits")) {
                    return false;
                }
                break;
            case InputTag::Matrix4d:
            case InputTag::Vec3d:
            case InputTag::Vec3f:
                ok = value.bits == 0;
                break;
            default:
                break;
            }
            if (!ok) {
                return _Bad(where + ": bits do not fit the tag");
            }
        }
        return true;
    }

    bool _Pools()
    {
        const auto first = [&](const auto &pool, const char *name) {
            if (pool.empty() || !pool[0].v.empty()) {
                return _Bad(std::string(name) +
                            "[0] is not the empty array");
            }
            return true;
        };
        return first(_f.intArrays, "int_arrays") &&
               first(_f.floatArrays, "float_arrays") &&
               first(_f.doubleArrays, "double_arrays") &&
               first(_f.vec2fArrays, "vec2f_arrays") &&
               first(_f.vec3fArrays, "vec3f_arrays");
    }

    /// The input list: typed defaults, back references to the chains and
    /// phased consumers, the listed prefix sorted by path text, unique
    /// names.
    bool _Slots()
    {
        if (_f.listedInputs > _f.inputs.size()) {
            return _Bad("listed_inputs exceeds the input count");
        }
        std::vector<uint8_t> named(_f.paths.size(), 0);
        for (size_t i = 0; i < _f.inputs.size(); ++i) {
            const fb::InputSlot &slot = _f.inputs[i];
            const std::string where = "inputs[" + _N(i) + "]";
            if (!_PathId(slot.name(), _Property, where, "name")) {
                return false;
            }
            if (named[slot.name()]) {
                return _Bad(where + ": a second slot for " +
                            RigExecFormatPathText(_f, slot.name()));
            }
            named[slot.name()] = 1;
            const bool listed =
                (slot.flags() & uint8_t(fb::InputSlotFlags::Listed)) != 0;
            if (slot.type() > InputTag::MAX ||
                (slot.flags() & ~uint8_t(fb::InputSlotFlags::ANY)) != 0 ||
                slot.value() >= _f.values.size() ||
                _f.values[slot.value()].tag != slot.type() ||
                listed != (i < _f.listedInputs)) {
                return _Bad(where + ": malformed type, flags or default");
            }
            if (slot.chain() != -1 &&
                (slot.chain() < 0 ||
                 size_t(slot.chain()) >= _f.propertyChains.size() ||
                 _f.propertyChains[size_t(slot.chain())].target != i)) {
                return _Bad(where + ": names a chain that does not "
                                    "target it");
            }
            if (slot.phased() != -1 &&
                (slot.phased() < 0 ||
                 size_t(slot.phased()) >= _f.phasedConsumers.size() ||
                 _f.phasedConsumers[size_t(slot.phased())].consumer != i)) {
                return _Bad(where + ": names a phased consumer that does "
                                    "not publish at it");
            }
            if (i > 0 && i < _f.listedInputs &&
                !_TextLess(_f.inputs[i - 1].name(), slot.name())) {
                return _Bad(where + ": listed inputs are not sorted by "
                                    "path text");
            }
        }
        return true;
    }

    bool _SlotMeta()
    {
        const fb::RigExecWireSlotMeta &m = *_f.slotMeta;
        const std::string row = "slot_meta";
        if (!_PathIds(m.paths, _Prim, row, "paths") ||
            !_Size(m.slotKind.size(), _slots, row, "slot_kind") ||
            !_Size(m.parent.size(), _slots, row, "parent") ||
            !_Size(m.propParent.size(), _slots, row, "prop_parent") ||
            !_Size(m.needFinal.size(), _slots, row, "need_final") ||
            !_Size(m.needBase.size(), _slots, row, "need_base") ||
            !_Indices(m.parent, _slots, true, row, "parent") ||
            !_Indices(m.propParent, _slots, true, row, "prop_parent") ||
            !_Bools(m.needFinal, row, "need_final") ||
            !_Bools(m.needBase, row, "need_base")) {
            return false;
        }
        for (size_t k = 0; k < m.slotKind.size(); ++k) {
            if (m.slotKind[k] > fb::SlotKind::MAX) {
                return _Bad(_At(row, "slot_kind", long(k)) +
                            ": out of range");
            }
        }
        const auto pairs = [&](const std::vector<int32_t> &slots,
                               const std::vector<uint32_t> &paths,
                               const char *slotField,
                               const char *pathField) {
            return _Size(paths.size(), slots.size(), row, pathField) &&
                   _Indices(slots, _slots, false, row, slotField) &&
                   _PathIds(paths, _Prim, row, pathField);
        };
        if (!pairs(m.xformSlots, m.xformPaths, "xform_slots",
                   "xform_paths") ||
            !pairs(m.jointSlots, m.jointPaths, "joint_slots",
                   "joint_paths") ||
            !pairs(m.controlSlots, m.controlPaths, "control_slots",
                   "control_paths") ||
            !_Size(m.solverArrayElements.size(), m.solverArrayPaths.size(),
                   row, "solver_array_elements") ||
            !_PathIds(m.solverArrayPaths, _AnyPath, row,
                      "solver_array_paths")) {
            return false;
        }
        const auto permutation = [&](const std::vector<int32_t> &order,
                                     size_t count, const char *field) {
            if (!_Size(order.size(), count, row, field)) {
                return false;
            }
            std::vector<uint8_t> seen(count, 0);
            for (size_t k = 0; k < order.size(); ++k) {
                if (order[k] < 0 || size_t(order[k]) >= count ||
                    seen[size_t(order[k])]) {
                    return _Bad(_At(row, field, long(k)) +
                                ": not a permutation");
                }
                seen[size_t(order[k])] = 1;
            }
            return true;
        };
        return permutation(m.jointPublishOrder, m.jointSlots.size(),
                           "joint_publish_order") &&
               permutation(m.controlPublishOrder, m.controlSlots.size(),
                           "control_publish_order") &&
               permutation(m.solverPublishOrder, m.solverArrayPaths.size(),
                           "solver_publish_order");
    }

    bool _Constants()
    {
        const fb::RigExecWireConstants &c = *_f.constants;
        const std::string row = "constants";
        if (!_Size(c.restM.size(), _slots, row, "rest_m") ||
            !_Size(c.restPts.size(), _slots, row, "rest_pts") ||
            !_Size(c.restFrames.size(), _slots, row, "rest_frames") ||
            !_Size(c.selfD.size(), _slots, row, "self_d") ||
            !_Size(c.parentDinv.size(), _slots, row, "parent_dinv") ||
            !_Size(c.rotOrder.size(), _slots, row, "rot_order") ||
            !_Size(c.restRoundTrip.size(), _slots, row, "rest_round_trip") ||
            !_Size(c.defaultRoundTrip.size(), _slots, row,
                   "default_round_trip") ||
            !_Size(c.posedAuthored.size(), _slots, row, "posed_authored") ||
            !_Size(c.posedAuthoredM.size(), _slots, row,
                   "posed_authored_m") ||
            !_Size(c.noScaleAvars.size(), _slots, row, "no_scale_avars") ||
            !_Size(c.avarConstants.size(), _slots * 11, row,
                   "avar_constants") ||
            !_Size(c.rotationSign.size(), _slots, row, "rotation_sign") ||
            !_PathIds(c.rotOrder, _TokenOrZero, row, "rot_order") ||
            !_Bools(c.posedAuthored, row, "posed_authored") ||
            !_Bools(c.noScaleAvars, row, "no_scale_avars")) {
            return false;
        }
        for (size_t k = 0; k < c.rotationSign.size(); ++k) {
            if (c.rotationSign[k] > 7) {
                return _Bad(_At(row, "rotation_sign", long(k)) +
                            ": more than three bits");
            }
        }
        return true;
    }

    bool _Steps()
    {
        for (size_t i = 0; i < _f.steps.size(); ++i) {
            const fb::RigExecWireStep &step = _f.steps[i];
            const std::string row = "steps[" + _N(i) + "]";
            if (step.kind > fb::StepKind::MAX) {
                return _Bad(row + ": kind out of range");
            }
            if (!_Indices(step.overrideInputs, _overrideCount, false, row,
                          "override_inputs")) {
                return false;
            }
            for (const auto *ranges : {&step.reads, &step.writes}) {
                for (const fb::SlotRange &range : *ranges) {
                    if (range.domain() > fb::SlotDomain::MAX ||
                        range.begin() > range.end()) {
                        return _Bad(row + ": malformed slot range");
                    }
                }
            }
            if (step.kind == fb::StepKind::WeightPacket &&
                (step.object < 0 || size_t(step.object) >= _stepBacked)) {
                return _Bad(row + ": a WeightPacket step names no "
                                  "step-backed weight object");
            }
        }
        // Every step and cluster index, edge order, producers, membership,
        // the partition and the acyclic cluster graph: the rules the
        // runtime's reader applies, in its words.
        const std::string why = RigExecStepGraphError(
            _f.steps, *_f.clustering, [](const fb::SlotRange &range) {
                return RigExecStepGraphRange{uint8_t(range.domain()),
                                             range.begin(), range.end()};
            });
        return why.empty() || _Bad(why);
    }

    bool _Cones()
    {
        const fb::RigExecWireCones &c = *_f.cones;
        const std::string row = "cones";
        if (!_Size(c.cone.size(), _clusters, row, "cone") ||
            !_ClusterSet(c.always.get(), "cones.always") ||
            !_ClusterSet(c.poseClusters.get(), "cones.pose_clusters")) {
            return false;
        }
        for (size_t k = 0; k < c.cone.size(); ++k) {
            if (!_ClusterSet(&c.cone[k], "cones.cone[" + _N(k) + "]")) {
                return false;
            }
        }
        if (!_Size(c.avarCluster.size(), _slots, row, "avar_cluster") ||
            !_Indices(c.avarCluster, _clusters, true, row, "avar_cluster") ||
            !_Size(c.chainBaseClusters.size(), _chains, row,
                   "chain_base_clusters") ||
            !_Size(c.revisionClusters.size(), _revisions, row,
                   "revision_clusters") ||
            !_Size(c.revisionStaticCluster.size(), _revisions, row,
                   "revision_static_cluster") ||
            !_Indices(c.revisionStaticCluster, _clusters, true, row,
                      "revision_static_cluster") ||
            !_Indices(c.varyingSteps, _steps, false, row, "varying_steps") ||
            !_Indices(c.overrideSteps, _steps, false, row,
                      "override_steps")) {
            return false;
        }
        for (const auto *lists : {&c.chainBaseClusters, &c.revisionClusters}) {
            for (size_t k = 0; k < lists->size(); ++k) {
                if (!_Indices((*lists)[k].v, _clusters, false,
                              row + (lists == &c.chainBaseClusters
                                         ? ".chain_base_clusters["
                                         : ".revision_clusters[") +
                                  _N(k) + "]",
                              "v")) {
                    return false;
                }
            }
        }
        return true;
    }

    // ------------------------------------------------------- pose domain

    bool _Pose()
    {
        const fb::RigExecWireDomainPose &p = *_f.pose;
        if (!_Size(p.ladders.size(), _slots, "pose", "ladders") ||
            !_Bools(p.restChainVaries, "pose", "rest_chain_varies") ||
            !_Indices(p.ladderOverrides, _overrideCount, false, "pose",
                      "ladder_overrides") ||
            !_PathIds(p.poseWeightPaths, _AnyPath, "pose",
                      "pose_weight_paths") ||
            !_Indices(p.guideSolvers, p.solvers.size(), false, "pose",
                      "guide_solvers")) {
            return false;
        }
        for (size_t i = 0; i < p.ladders.size(); ++i) {
            if (!_Ladder(p.ladders[i], "pose.ladders[" + _N(i) + "]")) {
                return false;
            }
        }
        for (size_t i = 0; i < p.poseInterpolators.size(); ++i) {
            if (!_Interpolator(p.poseInterpolators[i],
                               "pose.pose_interpolators[" + _N(i) + "]")) {
                return false;
            }
        }
        for (size_t i = 0; i < p.solvers.size(); ++i) {
            if (!_Solver(p.solvers[i], "pose.solvers[" + _N(i) + "]")) {
                return false;
            }
        }
        for (size_t i = 0; i < p.constraints.size(); ++i) {
            if (!_Constraint(p.constraints[i],
                             "pose.constraints[" + _N(i) + "]")) {
                return false;
            }
        }
        for (size_t i = 0; i < p.constraintArrays.size(); ++i) {
            if (!_ConstraintArrays(p.constraintArrays[i],
                                   "pose.constraint_arrays[" + _N(i) + "]")) {
                return false;
            }
        }
        for (size_t i = 0; i < p.nativeSources.size(); ++i) {
            const fb::RigExecWireNativeSource &source = p.nativeSources[i];
            const std::string row = "pose.native_sources[" + _N(i) + "]";
            if (!_PathId(source.path, _AnyPath, row, "path") ||
                !_Indices(source.ancestorSlots, _slots, false, row,
                          "ancestor_slots")) {
                return false;
            }
        }
        for (size_t i = 0; i < p.walkSteps.size(); ++i) {
            const fb::RigExecWireWalkStep &walk = p.walkSteps[i];
            const std::string row = "pose.walk_steps[" + _N(i) + "]";
            if (!_Indices(walk.batchSolvers, p.solvers.size(), false, row,
                          "batch_solvers")) {
                return false;
            }
        }
        for (size_t i = 0; i < p.composeGroups.size(); ++i) {
            const fb::RigExecWireComposeGroup &group = p.composeGroups[i];
            const std::string row = "pose.compose_groups[" + _N(i) + "]";
            if (group.begin < 0 || group.begin > group.end ||
                size_t(group.end) > _slots) {
                return _Bad(row + ": slot range out of range");
            }
            if (!_Indices(group.parentSlots, _slots, true, row,
                          "parent_slots")) {
                return false;
            }
        }
        if (!_Commits(p)) {
            return false;
        }
        if (!_Size(p.jointBindingSolvers.size(), p.jointBindingJoints.size(),
                   "pose", "joint_binding_solvers") ||
            !_Size(p.jointBindingElements.size(),
                   p.jointBindingJoints.size(), "pose",
                   "joint_binding_elements") ||
            !_PathIds(p.jointBindingJoints, _AnyPath, "pose",
                      "joint_binding_joints")) {
            return false;
        }
        for (size_t k = 0; k < p.jointBindingSolvers.size(); ++k) {
            const std::string row =
                "pose.joint_binding_solvers[" + _N(k) + "]";
            if (!_Size(p.jointBindingElements[k].v.size(),
                       p.jointBindingSolvers[k].v.size(), row,
                       "elements") ||
                !_PathIds(p.jointBindingSolvers[k].v, _AnyPath, row, "v")) {
                return false;
            }
        }
        if (p.hasPropertyChains != !_f.propertyChains.empty()) {
            return _Bad("pose.has_property_chains disagrees with "
                        "property_chains");
        }
        int32_t lastSwitch = -1;
        for (size_t i = 0; i < p.spaceSwitches.size(); ++i) {
            const fb::RigExecWireSpaceSwitch &sw = p.spaceSwitches[i];
            const std::string row = "pose.space_switches[" + _N(i) + "]";
            if (!_Index(sw.slot, _slots, false, row, "slot") ||
                !_Indices(sw.sourceSlots, _slots, true, row,
                          "source_slots") ||
                !_Index(sw.spaceSlot, _slots, true, row, "space_slot") ||
                !_ReadPtr(sw.active, int(InputTag::Double), _Baked, row,
                          "active")) {
                return false;
            }
            if (sw.slot <= lastSwitch) {
                return _Bad(row + ": switches are not in ascending slot "
                                  "order");
            }
            lastSwitch = sw.slot;
            if (!sw.filters.empty() &&
                !_Size(sw.filters.size(), sw.sourceSlots.size(), row,
                       "filters")) {
                return false;
            }
            for (size_t k = 0; k < sw.filters.size(); ++k) {
                if (sw.filters[k] > uint8_t(fb::RotationFilter::MAX)) {
                    return _Bad(_At(row, "filters", long(k)) +
                                ": out of range");
                }
            }
            if (!_FrameVersion(sw.parentRead.get(), row + ".parent_read") ||
                !_FrameVersion(sw.spaceRead.get(), row + ".space_read") ||
                (sw.spaceSlot == -1 &&
                 !_UnreadVersion(*sw.spaceRead, row + ".space_read")) ||
                !_Size(sw.sourceReads.size(), sw.sourceSlots.size(), row,
                       "source_reads")) {
                return false;
            }
            for (size_t k = 0; k < sw.sourceReads.size(); ++k) {
                const std::string at = _At(row, "source_reads", long(k));
                if (!_FrameVersion(&sw.sourceReads[k], at) ||
                    (sw.sourceSlots[k] == -1 &&
                     !_UnreadVersion(sw.sourceReads[k], at))) {
                    return false;
                }
            }
        }
        std::vector<uint8_t> avarSeen(_slots * 11, 0);
        for (size_t i = 0; i < p.avarBindings.size(); ++i) {
            const fb::RigExecWireAvarBinding &binding = p.avarBindings[i];
            const std::string row = "pose.avar_bindings[" + _N(i) + "]";
            if (!_Index(binding.flat, _slots * 11, false, row, "flat") ||
                !_ReadPtr(binding.read, int(InputTag::Double), _Baked, row,
                          "read")) {
                return false;
            }
            if (avarSeen[binding.flat] ||
                _f.slotMeta->slotKind[binding.flat / 11] !=
                    fb::SlotKind::FirstFramePose) {
                return _Bad(row + ": a second binding of the avar, or an "
                                  "avar of a slot without avars");
            }
            avarSeen[binding.flat] = 1;
        }
        return _Size(p.xformBase.size(), _f.slotMeta->xformSlots.size(),
                     "pose", "xform_base");
    }

    bool _Ladder(const fb::RigExecWireLadder &ladder, const std::string &row)
    {
        const int matrix = int(InputTag::Matrix4d);
        if (!_ReadPtr(ladder.restSpace, matrix, _Baked, row, "rest_space") ||
            !_ReadPtr(ladder.defaultSpace, matrix, _Baked, row,
                      "default_space") ||
            !_ReadPtr(ladder.posedSpace, matrix, _Baked, row,
                      "posed_space") ||
            !_ReadPtr(ladder.rotationOrder, int(InputTag::Token), _Baked, row,
                      "rotation_order") ||
            !_Size(ladder.restAvars.size(), 6, row, "rest_avars") ||
            !_Size(ladder.defaultAvars.size(), 6, row, "default_avars")) {
            return false;
        }
        for (size_t k = 0; k < 6; ++k) {
            if (!_Read(&ladder.restAvars[k], int(InputTag::Double), _Baked,
                       row, "rest_avars", long(k)) ||
                !_Read(&ladder.defaultAvars[k], int(InputTag::Double), _Baked,
                       row, "default_avars", long(k))) {
                return false;
            }
        }
        return true;
    }

    bool _Interpolator(const fb::RigExecWirePoseInterpolator &interp,
                       const std::string &row)
    {
        if (!_PathId(interp.path, _AnyPath, row, "path") ||
            !_Index(interp.driverSlot, _slots, true, row, "driver_slot") ||
            !_Index(interp.parentSlot, _slots, true, row, "parent_slot") ||
            !_ReadPtr(interp.enabled, int(InputTag::Bool), _Baked, row,
                      "enabled")) {
            return false;
        }
        if (interp.valueInputs.size() > 3 ||
            (!interp.valueInputs.empty() && interp.driverSlot >= 0)) {
            return _Bad(row + ": value_inputs hold at most three dials of "
                              "a numeric driver");
        }
        for (size_t k = 0; k < interp.valueInputs.size(); ++k) {
            if (!_Read(&interp.valueInputs[k], int(InputTag::Double), _Baked,
                       row, "value_inputs", long(k))) {
                return false;
            }
        }
        if (!interp.solver) {
            return _Bad(row + ".solver: missing");
        }
        const fb::RigExecWireRbf &rbf = *interp.solver;
        if (rbf.kernel > uint8_t(fb::RbfKernel::MAX) ||
            (!rbf.poseTypes.empty() &&
             rbf.poseTypes.size() != rbf.poses.size())) {
            return _Bad(row + ".solver: kernel or pose_types malformed");
        }
        for (size_t k = 0; k < rbf.poseTypes.size(); ++k) {
            if (rbf.poseTypes[k] > uint8_t(fb::RbfPoseType::MAX)) {
                return _Bad(_At(row + ".solver", "pose_types", long(k)) +
                            ": out of range");
            }
        }
        for (size_t k = 0; k < rbf.weights.size(); ++k) {
            if (rbf.weights[k].v.size() != rbf.poses.size()) {
                return _Bad(_At(row + ".solver", "weights", long(k)) +
                            ": one weight per pose expected");
            }
        }
        return true;
    }

    bool _Solver(const fb::RigExecWireSolver &s, const std::string &row)
    {
        const int dbl = int(InputTag::Double);
        const int flt = int(InputTag::Float);
        if (!_PathId(s.path, _AnyPath, row, "path") ||
            !_PathId(s.type, _TokenOrZero, row, "type") ||
            !_Bools(s.restIsLive, row, "rest_is_live") ||
            !_Indices(s.restOverrides, _overrideCount, false, row,
                      "rest_overrides") ||
            !_Index(s.start, _slots, true, row, "start") ||
            !_Index(s.root, _slots, true, row, "root") ||
            !_Index(s.mid, _slots, true, row, "mid") ||
            !_Index(s.end, _slots, true, row, "end") ||
            !_Index(s.pole, _slots, true, row, "pole") ||
            !_Index(s.spaceSlot, _slots, true, row, "space_slot") ||
            !_Index(s.inA, _slots, true, row, "in_a") ||
            !_Index(s.inB, _slots, true, row, "in_b") ||
            !_Size(s.ikRests.size(), 3, row, "ik_rests") ||
            !_PathId(s.ribbonPointsPath, _AnyPath, row,
                     "ribbon_points_path")) {
            return false;
        }
        if (s.splineRestMode > uint8_t(fb::SplineIkRestLength::MAX) ||
            s.scaleMode > uint8_t(fb::ScaleBlend::MAX)) {
            return _Bad(row + ": spline_rest_mode or scale_mode out of "
                              "range");
        }
        for (size_t k = 0; k < s.outputs.size(); ++k) {
            if (!_Index(s.outputs[k].first, _slots, false, row, "outputs",
                        long(k))) {
                return false;
            }
        }
        if (!s.splineRest) {
            return _Bad(row + ".spline_rest: missing");
        }
        return _ReadPtr(s.bend, dbl, _Baked, row, "bend") &&
               _ReadPtr(s.upperOffset, dbl, _Baked, row, "upper_offset") &&
               _ReadPtr(s.lowerOffset, dbl, _Baked, row, "lower_offset") &&
               _ReadPtr(s.stretch, flt, _Baked, row, "stretch") &&
               _ReadPtr(s.softness, flt, _Baked, row, "softness") &&
               _ReadPtr(s.ikSpace, int(InputTag::Matrix4d), _Baked, row,
                        "ik_space") &&
               _ReadPtr(s.blendWeight, flt, _Baked, row, "blend_weight") &&
               _ReadPtr(s.preserveVolume, dbl, _Baked, row,
                        "preserve_volume") &&
               _ReadPtr(s.midFollowWeight, dbl, _Baked, row,
                        "mid_follow_weight") &&
               _ReadPtr(s.roll, dbl, _Baked, row, "roll") &&
               _ReadPtr(s.twist, dbl, _Baked, row, "twist") &&
               _ReadPtr(s.minLengthRatio, dbl, _Baked, row,
                        "min_length_ratio") &&
               _ReadPtr(s.twistTurns, dbl, _Baked, row, "twist_turns") &&
               _ReadPtr(s.ribbonSampleCount, int(InputTag::Int), _Baked, row,
                        "ribbon_sample_count");
    }

    bool _Constraint(const fb::RigExecWireConstraint &c,
                     const std::string &row)
    {
        if (!_PathId(c.path, _AnyPath, row, "path") ||
            !_PathId(c.type, _TokenOrZero, row, "type") ||
            !_PathId(c.weightObject, _PrimOrZero, row, "weight_object") ||
            !_PathId(c.worldUpType, _TokenOrZero, row, "world_up_type") ||
            !_PathId(c.pointsTarget, _AnyPath, row, "points_target") ||
            !_PathId(c.deltaBasePath, _AnyPath, row, "delta_base_path") ||
            !_PathId(c.worldUpPath, _AnyPath, row, "world_up_path") ||
            !_PathId(c.effectorPath, _AnyPath, row, "effector_path") ||
            !_PathIds(c.sourcePaths, _AnyPath, row, "source_paths") ||
            !_Index(c.target, _slots, true, row, "target") ||
            !_Indices(c.targetSlots, _slots, true, row, "target_slots") ||
            !_Index(c.spaceSlot, _slots, true, row, "space_slot") ||
            !_Index(c.arrays, _f.pose->constraintArrays.size(), true, row,
                    "arrays") ||
            !_Index(c.weightObjectIndex, _weights, true, row,
                    "weight_object_index") ||
            !_Bools(c.snapshotTargets, row, "snapshot_targets") ||
            !_Bools(c.ikRestLive, row, "ik_rest_live")) {
            return false;
        }
        if (c.order > uint8_t(fb::EulerOrder::MAX) ||
            c.ikMode > uint8_t(fb::SingleChainIkMode::MAX) ||
            (c.flags & ~uint8_t(fb::ConstraintFlags::ANY)) != 0) {
            return _Bad(row + ": order, ik_mode or flags out of range");
        }
        const bool envelope = c.weightObject != 0 && c.pointsTarget == 0;
        if ((c.weightObjectIndex >= 0) != envelope ||
            (envelope &&
             _f.geometry->weightObjects[size_t(c.weightObjectIndex)].path !=
                 c.weightObject)) {
            return _Bad(row + ": weight_object_index does not name the "
                              "envelope of its weight object");
        }
        const int b = int(InputTag::Bool);
        const int v = int(InputTag::Vec3d);
        return _ReadPtr(c.enabled, b, _Baked, row, "enabled") &&
               _ReadPtr(c.defaultWeight, int(InputTag::Float), _Baked, row,
                        "default_weight") &&
               _ReadPtr(c.offset, v, _Baked, row, "offset") &&
               _ReadPtr(c.affectX, b, _Baked, row, "affect_x") &&
               _ReadPtr(c.affectY, b, _Baked, row, "affect_y") &&
               _ReadPtr(c.affectZ, b, _Baked, row, "affect_z") &&
               _ReadPtr(c.tX, b, _Baked, row, "t_x") &&
               _ReadPtr(c.tY, b, _Baked, row, "t_y") &&
               _ReadPtr(c.tZ, b, _Baked, row, "t_z") &&
               _ReadPtr(c.rX, b, _Baked, row, "r_x") &&
               _ReadPtr(c.rY, b, _Baked, row, "r_y") &&
               _ReadPtr(c.rZ, b, _Baked, row, "r_z") &&
               _ReadPtr(c.sX, b, _Baked, row, "s_x") &&
               _ReadPtr(c.sY, b, _Baked, row, "s_y") &&
               _ReadPtr(c.sZ, b, _Baked, row, "s_z") &&
               _ReadPtr(c.aimVector, v, _Baked, row, "aim_vector") &&
               _ReadPtr(c.upVector, v, _Baked, row, "up_vector") &&
               _ReadPtr(c.rotationOffset, v, _Baked, row,
                        "rotation_offset") &&
               _ReadPtr(c.worldUpVector, v, _Baked, row,
                        "world_up_vector") &&
               _ReadPtr(c.poleVector, v, _Baked, row, "pole_vector") &&
               _ReadPtr(c.twistDegrees, int(InputTag::Double), _Baked, row,
                        "twist_degrees");
    }

    bool _ConstraintArrays(const fb::RigExecWireConstraintArrays &a,
                           const std::string &row)
    {
        if (!_PathId(a.prim, _PrimOrZero, row, "prim")) {
            return false;
        }
        if (a.ok && !_Size(a.weights.size(), a.sourceCount, row, "weights")) {
            return false;
        }
        if (a.ok && a.parentOffsets &&
            (!_Size(a.translationOffsets.size(), a.sourceCount, row,
                    "translation_offsets") ||
             !_Size(a.rotationOffsets.size(), a.sourceCount, row,
                    "rotation_offsets"))) {
            return false;
        }
        if (a.readPole && a.poleOk &&
            !_Size(a.poleWeights.size(), a.poleCount, row, "pole_weights")) {
            return false;
        }
        return true;
    }

    /// Commit writes and propagation inside the slots, and every fin and
    /// base version below the pools the runtime sizes from them: the seeds,
    /// the last writes, then the arena of the writes before them.
    bool _Commits(const fb::RigExecWireDomainPose &p)
    {
        std::vector<uint32_t> finWrites(_slots, 0);
        std::vector<uint32_t> baseWrites(_slots, 0);
        uint64_t finMax = 0, baseMax = 0;
        bool haveFin = false, haveBase = false;
        const auto fin = [&](uint32_t v) {
            haveFin = true;
            finMax = std::max<uint64_t>(finMax, v);
        };
        const auto base = [&](uint32_t v) {
            haveBase = true;
            baseMax = std::max<uint64_t>(baseMax, v);
        };
        const auto ancestors =
            [&](const std::vector<RigExecWireAncestorRead> &reads,
                const std::string &row, const char *field) {
                for (size_t k = 0; k < reads.size(); ++k) {
                    if (!_Index(reads[k].slot, _slots, true, row, field,
                                long(k))) {
                        return false;
                    }
                    fin(reads[k].fin);
                    base(reads[k].base);
                }
                return true;
            };
        for (size_t i = 0; i < p.commits.size(); ++i) {
            const fb::RigExecWireCommit &commit = p.commits[i];
            const std::string row = "pose.commits[" + _N(i) + "]";
            if (!_PathId(commit.moverPath, _AnyPath, row, "mover_path")) {
                return false;
            }
            for (size_t k = 0; k < commit.slots.size(); ++k) {
                if (commit.slots[k] < 0 || size_t(commit.slots[k]) >= _slots ||
                    k >= commit.slotWrites.size()) {
                    return _Bad(_At(row, "slots", long(k)) +
                                ": a commit write names no slot");
                }
                const size_t slot = size_t(commit.slots[k]);
                ++finWrites[slot];
                fin(commit.slotWrites[k]);
                if (k < commit.slotReads.size()) {
                    fin(commit.slotReads[k]);
                }
                if (k < commit.slotCarry.size()) {
                    fin(commit.slotCarry[k]);
                }
                if (commit.solverOutput && k < commit.slotBaseWrites.size()) {
                    ++baseWrites[slot];
                    base(commit.slotBaseWrites[k]);
                }
                if (commit.solverOutput && k < commit.slotBaseCarry.size()) {
                    base(commit.slotBaseCarry[k]);
                }
            }
            for (size_t k = 0; k < commit.propagate.size(); ++k) {
                const int32_t slotIndex = commit.propagate[k].first;
                if (slotIndex < 0 || size_t(slotIndex) >= _slots ||
                    k >= commit.descendantWrites.size()) {
                    return _Bad(_At(row, "propagate", long(k)) +
                                ": a propagation write names no slot");
                }
                const size_t slot = size_t(slotIndex);
                ++finWrites[slot];
                fin(commit.descendantWrites[k]);
                if (k < commit.descendantReads.size()) {
                    fin(commit.descendantReads[k]);
                }
                if (k < commit.closestReads.size()) {
                    fin(commit.closestReads[k]);
                }
                if (k < commit.descendantCarry.size()) {
                    fin(commit.descendantCarry[k]);
                }
                if (commit.solverOutput &&
                    k < commit.descendantBaseWrites.size()) {
                    ++baseWrites[slot];
                    base(commit.descendantBaseWrites[k]);
                }
                if (commit.solverOutput &&
                    k < commit.descendantBaseCarry.size()) {
                    base(commit.descendantBaseCarry[k]);
                }
            }
            for (uint32_t v : commit.sourceReads) {
                fin(v);
            }
            fin(commit.worldUpRead);
            fin(commit.targetRead);
            for (uint32_t v : commit.targetReads) {
                fin(v);
            }
            fin(commit.effectorRead);
            for (uint32_t v : commit.poleReads) {
                fin(v);
            }
            if (!ancestors(commit.effectorAncestors, row,
                           "effector_ancestors") ||
                !ancestors(commit.worldUpAncestors, row,
                           "world_up_ancestors")) {
                return false;
            }
            for (const auto *lists :
                 {&commit.poleAncestors, &commit.sourceAncestors}) {
                for (const fb::RigExecWireAncestorReadList &list : *lists) {
                    if (!ancestors(list.v, row, "ancestors")) {
                        return false;
                    }
                }
            }
        }
        for (const fb::RigExecWireSolver &solver : p.solvers) {
            for (uint32_t v : solver.restReads) {
                fin(v);
            }
            for (uint32_t v : solver.controlReads) {
                fin(v);
            }
            fin(solver.rootRead);
            fin(solver.midRead);
            fin(solver.endRead);
            fin(solver.poleRead);
            if (solver.start >= 0) {
                fin(solver.startRead);
            }
            if (solver.spaceSlot >= 0) {
                fin(solver.spaceRead);
            }
        }
        uint64_t finArena = 0, baseArena = 0;
        for (size_t s = 0; s < _slots; ++s) {
            finArena += finWrites[s] > 0 ? finWrites[s] - 1 : 0;
            baseArena += baseWrites[s] > 0 ? baseWrites[s] - 1 : 0;
        }
        const uint64_t finPool = 2 * uint64_t(_slots) + finArena;
        const uint64_t basePool = 2 * uint64_t(_slots) + baseArena;
        if ((haveFin && finMax >= finPool) ||
            (haveBase && baseMax >= basePool)) {
            return _Bad("pose: a version reference exceeds the version "
                        "pools");
        }
        return true;
    }

    // --------------------------------------------------- geometry domain

    bool _Geometry()
    {
        const fb::RigExecWireDomainGeometry &g = *_f.geometry;
        const std::string row = "geometry";
        // The dense indices enumerate the chains' revisions and derived
        // targets chain-major, which is how the runtime addresses them.
        if (!_Size(g.revisionIndex.size(), _revisions, row,
                   "revision_index") ||
            !_Size(g.derivedIndex.size(), _derived, row, "derived_index") ||
            !_Size(g.chainRevisionBegin.size(), _chains, row,
                   "chain_revision_begin") ||
            !_Size(g.chainRevisionEnd.size(), _chains, row,
                   "chain_revision_end") ||
            !_Size(g.chainChunkBegin.size(), _chains, row,
                   "chain_chunk_begin") ||
            !_Size(g.chainChunkEnd.size(), _chains, row,
                   "chain_chunk_end") ||
            !_Size(g.revisionChunkBase.size(), _revisions, row,
                   "revision_chunk_base") ||
            !_Size(g.revisionChunkCount.size(), _revisions, row,
                   "revision_chunk_count")) {
            return false;
        }
        size_t id = 0, derivedId = 0;
        for (size_t c = 0; c < _chains; ++c) {
            const fb::RigExecWireChain &chain = g.chains[c];
            if (g.chainRevisionBegin[c] != int32_t(id) ||
                g.chainRevisionEnd[c] !=
                    int32_t(id + chain.revisions.size())) {
                return _Bad(row + ": chain_revision range of chain " +
                            _N(c) + " does not match its revisions");
            }
            for (size_t r = 0; r < chain.revisions.size(); ++r, ++id) {
                if (g.revisionIndex[id].first != int32_t(c) ||
                    g.revisionIndex[id].second != int32_t(r) ||
                    g.revisionChunkCount[id] !=
                        int32_t(chain.revisions[r].chunks.size()) ||
                    g.revisionChunkBase[id] != chain.revisions[r].chunkBase) {
                    return _Bad(row + ": revision_index or chunk tables "
                                      "disagree at revision " +
                                _N(id));
                }
            }
            for (size_t d = 0; d < chain.derived.size(); ++d, ++derivedId) {
                if (g.derivedIndex[derivedId].first != int32_t(c) ||
                    g.derivedIndex[derivedId].second != int32_t(d)) {
                    return _Bad(row + ": derived_index disagrees at " +
                                _N(derivedId));
                }
            }
        }
        for (size_t c = 0; c < _chains; ++c) {
            if (!_Chain(g.chains[c], "geometry.chains[" + _N(c) + "]")) {
                return false;
            }
        }
        if (!_WeightObjects(g) ||
            !_Size(g.falloffLuts.size(), g.falloffPaths.size(), row,
                   "falloff_luts") ||
            !_PathIds(g.falloffPaths, _AnyPath, row, "falloff_paths") ||
            !_PathIds(g.currentPhaseWeights, _AnyPath, row,
                      "current_phase_weights") ||
            !_PathIds(g.deltaBasePaths, _AnyPath, row, "delta_base_paths") ||
            !_Size(g.deltaBaseMatrix.size(), g.deltaBasePaths.size(), row,
                   "delta_base_matrix") ||
            !_Size(g.deltaBaseOk.size(), g.deltaBasePaths.size(), row,
                   "delta_base_ok") ||
            !_Bools(g.deltaBaseOk, row, "delta_base_ok")) {
            return false;
        }
        return _PathReads(g);
    }

    bool _Chain(const fb::RigExecWireChain &chain, const std::string &row)
    {
        if (!_PathId(chain.target, _AnyPath, row, "target") ||
            !_Pool(chain.base, _f.vec3fArrays.size(), row, "base")) {
            return false;
        }
        if (!chain.haveBase && chain.base != 0) {
            return _Bad(row + ": a base without have_base");
        }
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            if (!_Revision(chain.revisions[r], false,
                           row + ".revisions[" + _N(r) + "]")) {
                return false;
            }
        }
        for (size_t d = 0; d < chain.derived.size(); ++d) {
            const fb::RigExecWireDerived &derived = chain.derived[d];
            const std::string at = row + ".derived[" + _N(d) + "]";
            if (!_PathId(derived.target, _AnyPath, at, "target") ||
                !_Pool(derived.base, _f.vec3fArrays.size(), at, "base")) {
                return false;
            }
            if (!derived.haveBase && derived.base != 0) {
                return _Bad(at + ": a base without have_base");
            }
            if (!derived.revision) {
                return _Bad(at + ".revision: missing");
            }
            if (!_Revision(*derived.revision, true, at + ".revision")) {
                return false;
            }
        }
        return true;
    }

    bool _Revision(const fb::RigExecWireRevision &r, bool derived,
                   const std::string &row)
    {
        if (r.op > uint8_t(fb::RevisionOp::MAX) ||
            r.op == uint8_t(fb::RevisionOp::Reserved10) ||
            r.op == uint8_t(fb::RevisionOp::Reserved11)) {
            return _Bad(row + ": revision op " + _N(r.op) +
                        " is reserved or unknown");
        }
        if (!_PathId(r.moverPath, _AnyPath, row, "mover_path") ||
            !_PathId(r.target, _AnyPath, row, "target") ||
            !_PathId(r.moverPrim, _PrimOrZero, row, "mover_prim") ||
            !_PathId(r.weightFieldTarget, _AnyPath, row,
                     "weight_field_target") ||
            !_PathIds(r.shaderDials, _AnyPath, row, "shader_dials") ||
            !_Indices(r.influenceSlots, _slots, false, row,
                      "influence_slots") ||
            !_Index(r.transformSlot, _slots, true, row, "transform_slot") ||
            !_Index(r.transformSpaceSlot, _slots, true, row,
                    "transform_space_slot") ||
            !_Index(r.carrySpaceSlot, _slots, true, row,
                    "carry_space_slot") ||
            !_Index(r.weightObject, _weights, true, row, "weight_object")) {
            return false;
        }
        if (derived ? bool(r.defaultWeight) : !r.defaultWeight) {
            return _Bad(row + ".default_weight: present on a derived "
                              "revision or missing on a main one");
        }
        if (r.defaultWeight &&
            !_ReadPtr(r.defaultWeight, int(InputTag::Float), _Resolved, row,
                      "default_weight")) {
            return false;
        }
        if (!r.binding) {
            return _Bad(row + ".binding: missing");
        }
        if (!_Binding(*r.binding, row + ".binding")) {
            return false;
        }
        for (size_t ch = 0; ch < r.blendChannels.size(); ++ch) {
            if (!_BlendChannel(r.blendChannels[ch],
                               row + ".blend_channels[" + _N(ch) + "]")) {
                return false;
            }
        }
        if (r.partitionSameAsTopology &&
            (r.partitionTopology || !r.topology)) {
            return _Bad(row + ": partition_same_as_topology needs a "
                              "topology and no partition_topology");
        }
        if ((r.topology &&
             !_Topology(*r.topology, row + ".topology")) ||
            (r.partitionTopology &&
             !_Topology(*r.partitionTopology, row + ".partition_topology"))) {
            return false;
        }
        return true;
    }

    bool _Binding(const fb::RigExecWireRevisionBinding &b,
                  const std::string &row)
    {
        for (const uint32_t *id :
             {&b.moverPath, &b.target, &b.transform, &b.transformSpace,
              &b.weightObject, &b.base, &b.topologyCounts,
              &b.topologyIndices, &b.cagePoints, &b.surfacePoints,
              &b.bindCoords, &b.driverCurvePoints, &b.driverCurveOrder,
              &b.driverCurveKnots, &b.driverFrames, &b.widths}) {
            if (!_PathId(*id, _AnyPath, row, "path role")) {
                return false;
            }
        }
        if (!_PathIds(b.influences, _AnyPath, row, "influences") ||
            !_PathIds(b.blendInputs, _AnyPath, row, "blend_inputs") ||
            !_PathIds(b.blendSampleInputs, _AnyPath, row,
                      "blend_sample_inputs") ||
            !_PathIds(b.phaseInputs, _AnyPath, row, "phase_inputs") ||
            !_Size(b.blendSamples.size(), b.blendSampleInputs.size(), row,
                   "blend_samples") ||
            !_Size(b.phases.size(), b.phaseInputs.size(), row, "phases") ||
            !_Phase(b.transformPhase, row, "transform_phase")) {
            return false;
        }
        for (size_t k = 0; k < b.phases.size(); ++k) {
            if (!_Phase(b.phases[k], row, "phases", long(k))) {
                return false;
            }
        }
        for (size_t k = 0; k < b.blendSamples.size(); ++k) {
            const std::string at = row + ".blend_samples[" + _N(k) + "]";
            for (size_t s = 0; s < b.blendSamples[k].v.size(); ++s) {
                const fb::RigExecWireBlendSampleBinding &sample =
                    b.blendSamples[k].v[s];
                if (!_PathId(sample.sample, _AnyPath, at, "sample",
                             long(s)) ||
                    !_PathId(sample.points, _AnyPath, at, "points",
                             long(s)) ||
                    !_PathId(sample.blendShape, _AnyPath, at, "blend_shape",
                             long(s)) ||
                    !_Phase(sample.phase, at, "phase", long(s))) {
                    return false;
                }
            }
        }
        return true;
    }

    bool _BlendChannel(const fb::RigExecWireBlendChannel &channel,
                       const std::string &row)
    {
        if (!_PathId(channel.weight, channel.weightValid
                                         ? _Property
                                         : _Zero,
                     row, "weight") ||
            !_PathId(channel.weightPath, _AnyPath, row, "weight_path") ||
            !_ReadPtr(channel.weightRead, int(InputTag::Float), _Resolved,
                      row, "weight_read")) {
            return false;
        }
        for (size_t s = 0; s < channel.samples.size(); ++s) {
            const fb::RigExecWireBlendSample &sample = channel.samples[s];
            const std::string at = row + ".samples[" + _N(s) + "]";
            if (!_PathId(sample.samplePath, _AnyPath, at, "sample_path") ||
                !_PathId(sample.activation,
                         sample.activationValid ? _Property : _Zero, at,
                         "activation") ||
                !_PathId(sample.points,
                         sample.pointsValid ? _Property : _Zero, at,
                         "points") ||
                !_PathId(sample.pointsPath, _AnyPath, at, "points_path") ||
                !_PathId(sample.blendShape, _AnyPath, at, "blend_shape") ||
                !_Phase(sample.phase, at, "phase") ||
                !_Pool(sample.pointsValue, _f.vec3fArrays.size(), at,
                       "points_value") ||
                !_Size(sample.offsets.size(), sample.indices.size(), at,
                       "offsets") ||
                !_ReadPtr(sample.activationRead, int(InputTag::Float),
                          _Resolved, at, "activation_read")) {
                return false;
            }
        }
        return true;
    }

    /// The sparse skin layout: one index width and its vector, one counts
    /// vector, every count within element_size, and the counts summing to
    /// the kept entries.
    bool _Topology(const fb::RigExecWireSkinTopology &t,
                   const std::string &row)
    {
        if (t.elementSize < 0 || t.elementSize > 65535) {
            return _Bad(row + ": element_size out of range");
        }
        const bool narrow = t.elementSize <= 255;
        const size_t counted =
            narrow ? t.counts8.size() : t.counts16.size();
        if ((narrow ? !t.counts16.empty() : !t.counts8.empty()) ||
            uint64_t(counted) != t.pointCount) {
            return _Bad(row + ": per-point counts are not one per point in "
                              "the vector element_size selects");
        }
        uint64_t kept = 0;
        for (size_t k = 0; k < counted; ++k) {
            const uint32_t count =
                narrow ? uint32_t(t.counts8[k]) : uint32_t(t.counts16[k]);
            if (count > uint32_t(t.elementSize)) {
                return _Bad(row + ": point " + _N(k) +
                            " keeps more entries than element_size");
            }
            kept += count;
        }
        size_t indices = 0;
        switch (t.indexWidth) {
        case 1:
            indices = t.indices8.size();
            if (!t.indices16.empty() || !t.indices32.empty()) {
                indices = SIZE_MAX;
            }
            break;
        case 2:
            indices = t.indices16.size();
            if (!t.indices8.empty() || !t.indices32.empty()) {
                indices = SIZE_MAX;
            }
            break;
        case 4:
            indices = t.indices32.size();
            if (!t.indices8.empty() || !t.indices16.empty()) {
                indices = SIZE_MAX;
            }
            break;
        default:
            return _Bad(row + ": index_width " + _N(t.indexWidth) +
                        " is not 1, 2 or 4");
        }
        if (uint64_t(indices) != kept || uint64_t(t.weights.size()) != kept) {
            return _Bad(row + ": " + _N(size_t(kept)) +
                        " kept entries, but the indices or weights hold a "
                        "different count");
        }
        return true;
    }

    bool _WeightObjects(const fb::RigExecWireDomainGeometry &g)
    {
        bool envelopeSeen = false;
        for (size_t i = 0; i < g.weightObjects.size(); ++i) {
            const fb::RigExecWireWeightObject &w = g.weightObjects[i];
            const std::string row = "geometry.weight_objects[" + _N(i) + "]";
            if (envelopeSeen && !w.envelopeOnly) {
                return _Bad(row + ": step-backed after an envelope-only "
                                  "entry");
            }
            envelopeSeen = envelopeSeen || w.envelopeOnly;
            if (!_PathId(w.path, _Prim, row, "path") ||
                !_PathId(w.type, _TokenOrZero, row, "type") ||
                !_PathId(w.representation, _TokenOrZero, row,
                         "representation") ||
                !_PathId(w.rangePolicy, _TokenOrZero, row, "range_policy") ||
                !_PathId(w.combineMode, _TokenOrZero, row, "combine_mode") ||
                !_PathId(w.planeAxis, _TokenOrZero, row, "plane_axis") ||
                !_PathId(w.planeBounds, _TokenOrZero, row, "plane_bounds") ||
                !_PathId(w.oraclePlaneAxis, _TokenOrZero, row,
                         "oracle_plane_axis") ||
                !_PathId(w.oraclePlaneBounds, _TokenOrZero, row,
                         "oracle_plane_bounds") ||
                !_PathIds(w.combineTargetPoints, _AnyPath, row,
                          "combine_target_points") ||
                !_PathIds(w.targetPoints, _AnyPath, row, "target_points") ||
                !_PathIds(w.samplePoints, _AnyPath, row, "sample_points") ||
                !_PathIds(w.curvePoints, _AnyPath, row, "curve_points") ||
                !_Size(w.combineTargetValid.size(),
                       w.combineTargetPoints.size(), row,
                       "combine_target_valid") ||
                !_Size(w.targetValid.size(), w.targetPoints.size(), row,
                       "target_valid") ||
                !_Size(w.sampleValid.size(), w.samplePoints.size(), row,
                       "sample_valid") ||
                !_Size(w.curveValid.size(), w.curvePoints.size(), row,
                       "curve_valid") ||
                !_Bools(w.combineTargetValid, row, "combine_target_valid") ||
                !_Bools(w.targetValid, row, "target_valid") ||
                !_Bools(w.sampleValid, row, "sample_valid") ||
                !_Bools(w.curveValid, row, "curve_valid") ||
                !_Index(w.providerSlot, _slots, true, row, "provider_slot") ||
                !_Index(w.oracleSamples, _f.vec3fArrays.size(), true, row,
                        "oracle_samples") ||
                !_Index(w.oracleCurve, _f.vec3fArrays.size(), true, row,
                        "oracle_curve")) {
                return false;
            }
            // Dependency order bounds the oracle's recursion.
            if (!_Index(w.base, i, true, row, "base") ||
                !_Indices(w.inputs, i, false, row, "inputs")) {
                return false;
            }
            const unsigned mode = w.envelopeOnly ? _Resolved : _Baked;
            const int f = int(InputTag::Float);
            const std::pair<const std::unique_ptr<RigExecWireInput> *,
                            const char *>
                reads[] = {
                    {&w.defaultWeight, "default_weight"},
                    {&w.driver, "driver"},
                    {&w.scale, "scale"},
                    {&w.bias, "bias"},
                    {&w.strength, "strength"},
                    {&w.invert, "invert"},
                    {&w.falloffMin, "falloff_min"},
                    {&w.falloffMax, "falloff_max"},
                    {&w.scaleXPos, "scale_x_pos"},
                    {&w.scaleYPos, "scale_y_pos"},
                    {&w.scaleZPos, "scale_z_pos"},
                    {&w.scaleXNeg, "scale_x_neg"},
                    {&w.scaleYNeg, "scale_y_neg"},
                    {&w.scaleZNeg, "scale_z_neg"},
                    {&w.scaleX, "scale_x"},
                    {&w.scaleY, "scale_y"},
                    {&w.scaleZ, "scale_z"},
                    {&w.extentU, "extent_u"},
                    {&w.extentV, "extent_v"},
                };
            for (const auto &read : reads) {
                if (!_ReadPtr(*read.first, f, mode, row, read.second)) {
                    return false;
                }
            }
        }
        return true;
    }

    /// Path reads sorted strictly by (path, rest), each a static value or
    /// a connection-following read headed by its own attribute.
    bool _PathReads(const fb::RigExecWireDomainGeometry &g)
    {
        for (size_t i = 0; i < g.pathReads.size(); ++i) {
            const fb::RigExecWirePathRead &read = g.pathReads[i];
            const std::string row = "geometry.path_reads[" + _N(i) + "]";
            if (!_PathId(read.path, _Property, row, "path")) {
                return false;
            }
            if (i > 0) {
                const fb::RigExecWirePathRead &prev = g.pathReads[i - 1];
                if (std::make_pair(prev.path, prev.rest) >=
                    std::make_pair(read.path, read.rest)) {
                    return _Bad(row + ": not sorted strictly by (path, "
                                      "rest)");
                }
            }
            if (bool(read.value) == bool(read.read)) {
                return _Bad(row + ": holds both or neither of value and "
                                  "read");
            }
            if (read.read) {
                if (!_ReadPtr(read.read, _AnyTag, _Resolved, row, "read")) {
                    return false;
                }
                if (read.rest || read.read->walk.empty() ||
                    _f.inputs[read.read->walk[0]].name() != read.path) {
                    return _Bad(row + ".read: a live read headed by its "
                                      "own attribute expected");
                }
                continue;
            }
            if (read.headFallback) {
                return _Bad(row + ": head_fallback on a static value");
            }
            if (!_PathValue(*read.value, row + ".value")) {
                return false;
            }
        }
        return true;
    }

    bool _PathValue(const fb::RigExecWirePathValue &value,
                    const std::string &row)
    {
        using Tag = fb::PathTag;
        if (value.tag > Tag::MAX) {
            return _Bad(row + ": tag out of range");
        }
        if (bool(value.matrix) != (value.tag == Tag::Matrix4d) ||
            bool(value.vec3d) != (value.tag == Tag::Vec3d) ||
            bool(value.vec3i) != (value.tag == Tag::Vec3i)) {
            return _Bad(row + ": its members do not match its tag");
        }
        size_t pool = 0;
        switch (value.tag) {
        case Tag::IntArray:
            pool = _f.intArrays.size();
            break;
        case Tag::FloatArray:
            pool = _f.floatArrays.size();
            break;
        case Tag::Vec2fArray:
            pool = _f.vec2fArrays.size();
            break;
        case Tag::Vec3fArray:
            pool = _f.vec3fArrays.size();
            break;
        case Tag::DoubleArray:
            pool = _f.doubleArrays.size();
            break;
        default:
            break;
        }
        if (pool != 0) {
            if (value.bits != 0) {
                return _Bad(row + ": bits on an array value");
            }
            return _Pool(value.array, pool, row, "array");
        }
        if (value.array != 0) {
            return _Bad(row + ": an array id on a scalar value");
        }
        switch (value.tag) {
        case Tag::Bool:
            return value.bits <= 1 ? true : _Bad(row + ": bool bits");
        case Tag::Int:
        case Tag::Float:
            return (value.bits >> 32) == 0 ? true
                                           : _Bad(row + ": 32-bit bits");
        case Tag::Token:
            return (value.bits >> 32) == 0
                       ? _PathId(uint32_t(value.bits), _TokenOrZero, row,
                                 "bits")
                       : _Bad(row + ": token bits");
        case Tag::Double:
            return true;
        default:
            return value.bits == 0 ? true
                                   : _Bad(row + ": bits its tag does not "
                                                "use");
        }
    }

    // --------------------------------------------------- property chains

    bool _PropertyChains()
    {
        for (size_t c = 0; c < _f.propertyChains.size(); ++c) {
            const fb::RigExecWirePropertyChain &chain = _f.propertyChains[c];
            const std::string row = "property_chains[" + _N(c) + "]";
            if (chain.valueType > fb::PropertyValueType::MAX ||
                chain.target >= _f.inputs.size() ||
                _f.inputs[chain.target].chain() != int32_t(c)) {
                return _Bad(row + ": malformed value type or target");
            }
            const InputTag tag = _ChainReadTag(chain.valueType);
            const bool scalar = _IsScalar(chain.valueType);
            for (size_t r = 0; r < chain.revisions.size(); ++r) {
                const fb::RigExecWirePropertyRevision &revision =
                    chain.revisions[r];
                const std::string at = row + ".revisions[" + _N(r) + "]";
                if (revision.op > fb::PropertyOp::MAX) {
                    return _Bad(at + ": op out of range");
                }
                if (!_PathId(revision.mover, _PrimOrZero, at, "mover") ||
                    !_Index(revision.envelope, _weights, true, at,
                            "envelope")) {
                    return false;
                }
                // _PinnedRead: Pinned over at most the attribute itself, or
                // Resolved over a connection's walk.
                const unsigned modes = _Pinned | _Resolved;
                if (!_ReadPtr(revision.enabled, int(InputTag::Bool), modes,
                              at, "enabled") ||
                    !_ReadPtr(revision.defaultWeight, int(InputTag::Float),
                              modes, at, "default_weight") ||
                    !_ReadPtr(revision.value, int(tag), modes, at,
                              "value") ||
                    !_ReadPtr(revision.min, int(tag), modes, at, "min") ||
                    !_ReadPtr(revision.max, int(tag), modes, at, "max")) {
                    return false;
                }
                for (const auto *input :
                     {revision.enabled.get(), revision.defaultWeight.get(),
                      revision.value.get(), revision.min.get(),
                      revision.max.get()}) {
                    if (input->mode == ReadMode::Pinned &&
                        input->walk.size() > 1) {
                        return _Bad(at + ": a Pinned read walks more than "
                                         "its own attribute");
                    }
                }
                if (chain.valueType == fb::PropertyValueType::Matrix4d &&
                    (!revision.min->walk.empty() ||
                     !revision.max->walk.empty())) {
                    return _Bad(at + ": a matrix chain's min and max read "
                                     "nothing");
                }
                if (!(scalar && revision.op == fb::PropertyOp::Curve) &&
                    (!revision.keys.empty() || !revision.tangents.empty())) {
                    return _Bad(at + ": keys or tangents on a revision "
                                     "that is not a scalar curve");
                }
            }
        }
        for (size_t k = 0; k < _f.phasedConsumers.size(); ++k) {
            const fb::RigExecWirePhasedConsumer &phased =
                _f.phasedConsumers[k];
            const std::string row = "phased_consumers[" + _N(k) + "]";
            if (phased.chain >= _f.propertyChains.size() ||
                (k > 0 && phased.chain < _f.phasedConsumers[k - 1].chain) ||
                phased.consumer >= _f.inputs.size() ||
                _f.inputs[phased.consumer].phased() != int32_t(k) ||
                phased.consumerType > fb::PropertyValueType::MAX) {
                return _Bad(row + ": malformed");
            }
            const fb::RigExecWirePropertyChain &chain =
                _f.propertyChains[phased.chain];
            if ((phased.consumerType != chain.valueType &&
                 !(_IsScalar(phased.consumerType) &&
                   _IsScalar(chain.valueType))) ||
                phased.applied > chain.revisions.size()) {
                return _Bad(row + ": does not fit its chain");
            }
            // The consumer first, then the walk's attributes short of the
            // target.
            bool hops = !phased.hops.empty() &&
                        phased.hops[0] == phased.consumer;
            for (uint32_t hop : phased.hops) {
                hops = hops && hop < _f.inputs.size() && hop != chain.target;
            }
            if (!hops) {
                return _Bad(row + ": malformed hops");
            }
        }
        return true;
    }

    // --------------------------------------------------- external movers

    bool _External()
    {
        const auto &chains = _f.geometry->chains;
        std::vector<std::vector<uint8_t>> claimed(chains.size());
        for (size_t c = 0; c < chains.size(); ++c) {
            claimed[c].assign(chains[c].revisions.size(), 0);
        }
        for (size_t i = 0; i < _f.externalMovers.size(); ++i) {
            const fb::RigExecWireExternalMover &mover = _f.externalMovers[i];
            const std::string row = "external_movers[" + _N(i) + "]";
            if (mover.chain >= chains.size() ||
                mover.revision >= chains[mover.chain].revisions.size()) {
                return _Bad(row + ": names no revision");
            }
            const fb::RigExecWireRevision &revision =
                chains[mover.chain].revisions[mover.revision];
            if (revision.op != uint8_t(fb::RevisionOp::External) ||
                claimed[mover.chain][mover.revision]) {
                return _Bad(row + ": names a revision that is not a plugin "
                                  "mover, or one another entry names");
            }
            claimed[mover.chain][mover.revision] = 1;
            if (!_PathId(mover.type, _Token, row, "type")) {
                return false;
            }
            if (!_Size(mover.phasedFallback.size(),
                       revision.binding->phaseInputs.size(), row,
                       "phased_fallback")) {
                return false;
            }
            for (uint32_t id : mover.phasedFallback) {
                if (!_Pool(id, _f.vec3fArrays.size(), row,
                           "phased_fallback")) {
                    return false;
                }
            }
            for (size_t k = 0; k < mover.inputs.size(); ++k) {
                if (!_Read(&mover.inputs[k], _AnyTag, _AnyMode, row,
                           "inputs", long(k))) {
                    return false;
                }
            }
        }
        for (size_t c = 0; c < chains.size(); ++c) {
            for (size_t r = 0; r < chains[c].revisions.size(); ++r) {
                if (chains[c].revisions[r].op ==
                        uint8_t(fb::RevisionOp::External) &&
                    !claimed[c][r]) {
                    return _Bad("geometry.chains[" + _N(c) + "].revisions[" +
                                _N(r) + "]: a plugin mover without its "
                                        "external_movers entry");
                }
            }
        }
        return true;
    }

    /// Baked override numbers: each used by exactly one read, together
    /// covering [0, override_count).
    bool _Overrides()
    {
        std::sort(_overrideUses.begin(), _overrideUses.end());
        if (_overrideUses.size() != _overrideCount) {
            return _Bad("pose.override_count is " + _N(_overrideCount) +
                        " but " + _N(_overrideUses.size()) +
                        " reads carry override numbers");
        }
        for (size_t k = 0; k < _overrideUses.size(); ++k) {
            if (_overrideUses[k] != k) {
                return _Bad("override number " + _N(k) +
                            " is held by no read or by two");
            }
        }
        return true;
    }

    bool _Presentation()
    {
        if (_f.presentation.empty()) {
            return true;
        }
        const size_t size = _f.presentation.size();
        if (size >= FLATBUFFERS_MAX_BUFFER_SIZE) {
            return _Bad("presentation is past the FlatBuffers size limit");
        }
        if (!_Budget(_f.presentation.data(), size)
                 .Root(fb::PresentationTypeTable())) {
            return _Bad("presentation's objects do not fit its bytes "
                        "(shared, overlapping or out of range)");
        }
        flatbuffers::Verifier verifier(_f.presentation.data(), size,
                                       _VerifierOptions(size));
        if (!fb::VerifyPresentationBuffer(verifier)) {
            return _Bad("presentation is not a valid REXP buffer");
        }
        return true;
    }

    const RigExecWireFile &_f;
    std::string _error;
    size_t _slots = 0;
    size_t _clusters = 0;
    size_t _steps = 0;
    size_t _chains = 0;
    size_t _revisions = 0;
    size_t _derived = 0;
    size_t _weights = 0;
    size_t _stepBacked = 0;
    uint32_t _overrideCount = 0;
    std::vector<uint32_t> _overrideUses;
    /// Per path node: 0 for paths[0], else its parent's depth plus one.
    std::vector<uint32_t> _depth;
};

}  // namespace

bool
RigExecFormatValidate(const fb::RigExecWireFile &file, std::string *error)
{
    return _Validator(file).Run(error);
}

namespace {

/// Open after the identifier checks: copy, version probe, bounding walk,
/// verifier, unpack, validate.
bool
_OpenAligned(const uint8_t *bytes, size_t size,
             std::unique_ptr<fb::RigExecWireFile> *file, std::string *error)
{
    std::vector<_AlignedBlock> storage((size + 15) / 16);
    std::memcpy(storage.data(), bytes, size);
    const uint8_t *data = storage.data()->bytes;
    const flatbuffers::Verifier::Options options = _VerifierOptions(size);

    // The version first, from the root table alone, so a file of another
    // format version is named as such rather than as malformed.
    {
        flatbuffers::Verifier probe(data, size, options);
        const size_t root = probe.VerifyOffset<flatbuffers::uoffset_t>(0);
        const auto *table =
            reinterpret_cast<const flatbuffers::Table *>(data + root);
        if (root != 0 && table->VerifyTableStart(probe) &&
            table->VerifyField<uint32_t>(probe, fb::File::VT_FORMATVERSION,
                                         sizeof(uint32_t))) {
            const uint32_t version =
                table->GetField<uint32_t>(fb::File::VT_FORMATVERSION, 0);
            if (version != RigExecFormatVersion) {
                return _Fail(error, "unsupported .rigexec format version " +
                                        _N(version) + " (this reader reads " +
                                        _N(RigExecFormatVersion) +
                                        "); rebake");
            }
        }
    }
    if (!_Budget(data, size).Root(fb::FileTypeTable())) {
        return _Fail(error, "malformed .rigexec: its objects do not fit its "
                            "bytes (shared, overlapping or out of range)");
    }
    flatbuffers::Verifier verifier(data, size, options);
    if (!fb::VerifyFileBuffer(verifier)) {
        return _Fail(error, "malformed .rigexec: the FlatBuffers verifier "
                            "refused it");
    }
    const fb::File *root = fb::GetFile(data);
    if (root->formatVersion() != RigExecFormatVersion) {
        return _Fail(error, "unsupported .rigexec format version " +
                                _N(root->formatVersion()) + "; rebake");
    }
    std::unique_ptr<fb::RigExecWireFile> unpacked(root->UnPack());
    std::string why;
    if (!RigExecFormatValidate(*unpacked, &why)) {
        return _Fail(error, "invalid .rigexec: " + why);
    }
    *file = std::move(unpacked);
    return true;
}

}  // namespace

bool
RigExecFormatOpen(const uint8_t *bytes, size_t size,
                  std::unique_ptr<fb::RigExecWireFile> *file,
                  std::string *error)
{
    if (file) {
        file->reset();
    }
    if (!file) {
        return _Fail(error, "no file to open into");
    }
    if (!bytes || size < 8) {
        return _Fail(error, "not a .rigexec file (" + _N(size) + " bytes)");
    }
    if (std::memcmp(bytes, RigExecFormatIdentifier, 4) == 0) {
        return _Fail(error,
                     "not a v4 .rigexec (old REXB container); rebake");
    }
    if (std::memcmp(bytes + 4, RigExecFormatIdentifier, 4) != 0) {
        return _Fail(error, "not a .rigexec file (file identifier is not "
                            "REXB)");
    }
    if (size >= FLATBUFFERS_MAX_BUFFER_SIZE) {
        return _Fail(error, "malformed .rigexec: " + _N(size) +
                                " bytes is past the FlatBuffers limit");
    }
    // Allocation failures (the aligned copy, UnPack, the validator) refuse
    // the file rather than escape.
    try {
        return _OpenAligned(bytes, size, file, error);
    } catch (const std::bad_alloc &) {
        file->reset();
        return _Fail(error, "cannot open .rigexec: out of memory (" +
                                _N(size) + " bytes)");
    }
}

bool
RigExecFormatWrite(const fb::RigExecWireFile &file,
                   std::vector<uint8_t> *bytes, std::string *error)
{
    if (!bytes) {
        return _Fail(error, "no buffer to write into");
    }
    bytes->clear();
    std::string why;
    if (!RigExecFormatValidate(file, &why)) {
        return _Fail(error, "invalid .rigexec: " + why);
    }
    flatbuffers::FlatBufferBuilder builder(1u << 16);
    fb::FinishFileBuffer(builder, fb::File::Pack(builder, &file));
    bytes->assign(builder.GetBufferPointer(),
                  builder.GetBufferPointer() + builder.GetSize());
    return true;
}

std::string
RigExecFormatPathText(const fb::RigExecWireFile &file, uint32_t id)
{
    // Ancestors first: parent < id along every chain, so the walk ends.
    std::vector<uint32_t> chain;
    uint32_t at = id;
    while (at != 0 && at < file.paths.size() &&
           file.paths[at].kind() != PathKind::None) {
        chain.push_back(at);
        const fb::PathNode &node = file.paths[at];
        if (node.kind() == PathKind::Token || node.parent() >= at) {
            break;
        }
        at = node.parent();
    }
    std::string text;
    for (auto it = chain.rbegin(); it != chain.rend(); ++it) {
        const fb::PathNode &node = file.paths[*it];
        const std::string &name = node.name() < file.names.size()
                                      ? file.names[node.name()]
                                      : std::string();
        switch (node.kind()) {
        case PathKind::Prim:
            text += "/" + name;
            break;
        case PathKind::Property:
            text += "." + name;
            break;
        default:
            text += name;
            break;
        }
    }
    return text;
}

}  // namespace rigExec
