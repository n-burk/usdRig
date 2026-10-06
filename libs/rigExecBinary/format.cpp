// The .rigexec FlatBuffer: Open refuses, bounds, verifies, unpacks and
// validates; Write validates and packs; the validator holds every rule a
// file must satisfy that the file alone can decide.
#include "rigExecBinary/format.h"

#include "rigExecBinary/stepGraph.h"

#include <algorithm>
#include <cmath>
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
constexpr unsigned _Raw = 1u << unsigned(ReadMode::Raw);
constexpr unsigned _AnyMode = 0xfu;

/// Any scalar tag, for a read whose type the site does not fix.
constexpr int _AnyTag = -1;
/// Any array tag, for an array read whose element type the site does not
/// fix.
constexpr int _AnyArrayTag = -2;

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

/// The tag of a property chain's target slot: the value type itself.
InputTag
_ChainTargetTag(fb::PropertyValueType type)
{
    switch (type) {
    case fb::PropertyValueType::Double:
        return InputTag::Double;
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

/// Kept index \p k of \p topology, from the vector its index_width selects
/// (indices32 for any width but 1 and 2). \p k must be inside that vector.
int32_t
_TopologyIndex(const fb::RigExecWireSkinTopology &topology, size_t k)
{
    switch (topology.indexWidth) {
    case 1:
        return int32_t(topology.indices8[k]);
    case 2:
        return int32_t(topology.indices16[k]);
    default:
        return topology.indices32[k];
    }
}

/// How many kept indices the vector index_width selects holds.
size_t
_TopologyIndexCount(const fb::RigExecWireSkinTopology &topology)
{
    switch (topology.indexWidth) {
    case 1:
        return topology.indices8.size();
    case 2:
        return topology.indices16.size();
    default:
        return topology.indices32.size();
    }
}

/// Whether the sparse form holds a skin layout of \p indices indices and
/// \p weights weights at \p elementSize: rows of an element size in
/// [0, 65535], with no entries at element size 0.
bool
_SparseCanHold(size_t indices, size_t weights, int32_t elementSize)
{
    if (elementSize < 0 || elementSize > 65535 || indices != weights) {
        return false;
    }
    return elementSize == 0 ? indices == 0
                            : indices % size_t(elementSize) == 0;
}

/// Whether a skin layout of \p indices indices and \p weights weights is
/// rows of \p elementSize, at least 1: the only layouts the evaluators
/// give points and may validate.
bool
_Rows(size_t indices, size_t weights, int32_t elementSize)
{
    return elementSize >= 1 && indices == weights &&
           indices % size_t(elementSize) == 0;
}

/// The point count the evaluators give such a layout: its rows, else 0.
uint64_t
_RowCount(size_t indices, size_t weights, int32_t elementSize)
{
    return _Rows(indices, weights, elementSize)
               ? uint64_t(indices / size_t(elementSize))
               : 0;
}

/// How skin layout \p t breaks the evaluator's layout rules
/// (RigExecFormatSkinShapeValidates, then RigExecFormatSkinEntryValidates
/// on every entry), as the text that follows "validated", or empty when it
/// passes them; the kernels rely on them to index the influence table
/// unchecked. The sparse form is rows by its shape, and an entry it drops
/// is (0, +0), which passes whenever there is an influence. \p t must have
/// passed its form's shape checks, so its index and weight vectors are the
/// same length.
std::string
_RuleBreak(const fb::RigExecWireSkinTopology &t)
{
    if (t.raw &&
        !_Rows(t.rawIndices.size(), t.rawWeights.size(), t.elementSize)) {
        return ", but its raw layout is not rows of element_size";
    }
    const uint64_t dense =
        t.raw ? uint64_t(t.rawIndices.size())
              : t.pointCount * uint64_t(std::max(t.elementSize, 0));
    if (!RigExecFormatSkinShapeValidates(size_t(dense), size_t(dense),
                                         t.elementSize, t.influenceCount)) {
        return " with element_size " + std::to_string(t.elementSize) +
               " and " + std::to_string(t.influenceCount) + " influence(s)";
    }
    const size_t entries = t.raw ? t.rawWeights.size() : t.weights.size();
    for (size_t k = 0; k < entries; ++k) {
        const int32_t index = t.raw ? t.rawIndices[k] : _TopologyIndex(t, k);
        const float weight = t.raw ? t.rawWeights[k] : t.weights[k];
        if (RigExecFormatSkinEntryValidates(index, weight, t.influenceCount)) {
            continue;
        }
        const char *entry = t.raw ? ", but raw entry " : ", but kept entry ";
        if (index < 0 || uint64_t(index) >= t.influenceCount) {
            return entry + _N(k) + " indexes influence " +
                   std::to_string(index) + " of " +
                   std::to_string(t.influenceCount);
        }
        return entry + _N(k) + " has a negative or non-finite weight";
    }
    return {};
}

/// Whether \p weight is +0.0f, bit for bit: the one weight the sparse form
/// drops, at index 0, from the end of a row.
bool
_IsPositiveZero(float weight)
{
    uint32_t bits = 0;
    std::memcpy(&bits, &weight, sizeof bits);
    return bits == 0;
}

/// An input tag's name in the refusals: the scalar's type, or the array's
/// element type and "[]".
const char *
_TagName(fb::InputTag tag)
{
    switch (tag) {
    case fb::InputTag::Double:
        return "double";
    case fb::InputTag::Float:
        return "float";
    case fb::InputTag::Bool:
        return "bool";
    case fb::InputTag::Int:
        return "int";
    case fb::InputTag::Matrix4d:
        return "matrix4d";
    case fb::InputTag::Token:
        return "token";
    case fb::InputTag::Vec3d:
        return "vec3d";
    case fb::InputTag::Vec3f:
        return "vec3f";
    case fb::InputTag::IntArray:
        return "int[]";
    case fb::InputTag::FloatArray:
        return "float[]";
    case fb::InputTag::DoubleArray:
        return "double[]";
    case fb::InputTag::Vec2fArray:
        return "float2[]";
    case fb::InputTag::Vec3fArray:
        return "float3[]";
    }
    return "unknown";
}

/// The PathValue tag of a static value of array input tag \p tag.
fb::PathTag
_PathArrayTag(fb::InputTag tag)
{
    switch (tag) {
    case fb::InputTag::IntArray:
        return fb::PathTag::IntArray;
    case fb::InputTag::FloatArray:
        return fb::PathTag::FloatArray;
    case fb::InputTag::DoubleArray:
        return fb::PathTag::DoubleArray;
    case fb::InputTag::Vec2fArray:
        return fb::PathTag::Vec2fArray;
    case fb::InputTag::Vec3fArray:
        return fb::PathTag::Vec3fArray;
    default:
        return fb::PathTag::Absent;
    }
}

/// Whether \p index names an entry of \p table.
template <class Table>
bool
_Has(const Table &table, int64_t index)
{
    return index >= 0 && uint64_t(index) < uint64_t(table.size());
}

// The step rules, the step labels and the step graph's domain names cover
// every kind and domain up to these; an appended one needs its own.
static_assert(fb::StepKind::MAX == fb::StepKind::SkinTopology &&
                  fb::SlotDomain::MAX == fb::SlotDomain::SkinTopology,
              "a step kind or slot domain was appended: give it its rules");
// The array tags and sources are frozen; the runtime mirrors the numbers.
static_assert(uint8_t(fb::InputTag::IntArray) == 8 &&
                  uint8_t(fb::InputTag::FloatArray) == 9 &&
                  uint8_t(fb::InputTag::DoubleArray) == 10 &&
                  uint8_t(fb::InputTag::Vec2fArray) == 11 &&
                  uint8_t(fb::InputTag::Vec3fArray) == 12 &&
                  fb::InputTag::MAX == fb::InputTag::Vec3fArray,
              "the array input tags are frozen");
static_assert(uint8_t(fb::ArraySource::Pool) == 0 &&
                  uint8_t(fb::ArraySource::SkinIndices) == 1 &&
                  uint8_t(fb::ArraySource::SkinWeights) == 2 &&
                  fb::ArraySource::MAX == fb::ArraySource::SkinWeights,
              "the array sources are frozen");

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
               _Pose() && _Geometry() && _LayoutValues() &&
               _PhaseBindings() && _PropertyChains() && _HeadChecks() && _StepGraph() &&
               _External() && _Overrides() &&
               _Presentation();
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
    /// constant a value of the read's tag, the walk inside the slots and
    /// typed for the read (a scalar read never walks an array slot, an
    /// array read walks slots of its own tag), the pinned hop inside the
    /// walk and only where a Baked varying read takes it, override numbers
    /// only on Baked reads, a Raw read over one slot, and an array read Raw
    /// or Resolved, crossing no chain, with a pool constant. \p tag and
    /// \p modes are what the site admits.
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
        const bool array = RigExecFormatIsArrayTag(input->tag);
        if (tag == _AnyTag && array) {
            return _Bad(where() + ": tag " + _N(size_t(input->tag)) +
                        " is an array tag, which this site does not read");
        }
        if (tag == _AnyArrayTag && !array) {
            return _Bad(where() + ": tag " + _N(size_t(input->tag)) +
                        " is not an array tag");
        }
        if (tag >= 0 && input->tag != InputTag(tag)) {
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
            const InputTag hop = _f.inputs[input->walk[k]].type();
            if (!array && RigExecFormatIsArrayTag(hop)) {
                return _Bad(where() + ".walk[" + _N(k) + "]: slot " +
                            _N(input->walk[k]) +
                            " is an array input, which a scalar read never "
                            "walks");
            }
            if (array && hop != input->tag) {
                return _Bad(where() + ".walk[" + _N(k) + "]: slot " +
                            _N(input->walk[k]) + " holds " + _TagName(hop) +
                            ", not the read's " + _TagName(input->tag));
            }
        }
        const auto candidates = [&](const auto &list) {
            for (size_t k = 0; k < list.size(); ++k) {
                const auto &candidate = list[k];
                const auto kind = fb::PropertyCandidateKind(candidate.kind);
                if (candidate.slot >= _f.inputs.size() ||
                    candidate.kind > uint8_t(fb::PropertyCandidateKind::PhasedRecord) ||
                    (kind == fb::PropertyCandidateKind::SlotOnly ? candidate.version != -1 : candidate.version < 0))
                    return _Bad(where() + ": malformed property candidate");
                const auto &slot = _f.inputs[candidate.slot];
                if (kind == fb::PropertyCandidateKind::ChainFinal) {
                    if (slot.chain() < 0 || size_t(slot.chain()) >= _f.propertyChains.size())
                        return _Bad(where() + ": candidate names no chain hop");
                    const auto &chain = _f.propertyChains[size_t(slot.chain())];
                    if (candidate.version != int32_t(chain.versionBase + chain.revisions.size()))
                        return _Bad(where() + ": candidate version is not its chain final");
                } else if (kind == fb::PropertyCandidateKind::PhasedRecord) {
                    if (slot.phased() < 0 || size_t(slot.phased()) >= _f.phasedConsumers.size() ||
                        candidate.version != int32_t(_f.phasedConsumers[size_t(slot.phased())].version))
                        return _Bad(where() + ": candidate version is not its phased record");
                }
            }
            return true;
        };
        if (!candidates(input->propertyCandidates) || !candidates(input->doubleCandidates)) return false;
        if (input->rawFallbackSlot < -1 ||
            (input->rawFallbackSlot >= 0 && size_t(input->rawFallbackSlot) >= _f.inputs.size()) ||
            (!input->doubleCandidates.empty() && input->tag != InputTag::Float) ||
            (input->mode == ReadMode::Raw && (!input->propertyCandidates.empty() ||
                                             !input->doubleCandidates.empty())))
            return _Bad(where() + ": invalid property binding fallback or double tail");
        _readUses.emplace_back(input, where());
        if (array) {
            if (input->mode != ReadMode::Raw &&
                input->mode != ReadMode::Resolved) {
                return _Bad(where() + ": an array read is Raw or Resolved");
            }
            if ((input->flags & (_Flags(fb::InputReadFlags::ViaChain) |
                                 _Flags(fb::InputReadFlags::LongWay))) != 0) {
                return _Bad(where() + ": an array read crosses no chain "
                                      "and reads no long way");
            }
            if (_f.values[input->constant].arraySource !=
                fb::ArraySource::Pool) {
                return _Bad(where() + ": constant " + _N(input->constant) +
                            " is not a pool array");
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

    /// Switches are stored in resolution order: an anchor that is itself
    /// a switched slot reads the version its switch produces, so that
    /// switch is stored before the reading one (\p reader).
    bool _ResolvedAnchor(const fb::RigExecWireFrameVersion &read,
                         const std::vector<int32_t> &switchOf, size_t reader,
                         const std::string &where)
    {
        if (read.anchor < 0) {
            return true;
        }
        const int32_t producer = switchOf[size_t(read.anchor)];
        if (producer != -1 && size_t(producer) >= reader) {
            return _Bad(where + ".anchor: slot " +
                        _N(size_t(read.anchor)) +
                        " is switched by pose.space_switches[" +
                        _N(size_t(producer)) +
                        "], which is not stored before this switch");
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
            if (value.arraySource > fb::ArraySource::MAX) {
                return _Bad(where + ": array source out of range");
            }
            if (RigExecFormatIsArrayTag(value.tag)) {
                if (!_ArrayValue(value, where)) {
                    return false;
                }
                continue;
            }
            if (value.arraySource != fb::ArraySource::Pool ||
                value.array != 0) {
                return _Bad(where + ": an array source on a scalar value");
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

    /// The pool an array tag's elements live in, by size.
    size_t _PoolSize(InputTag tag) const
    {
        switch (tag) {
        case InputTag::IntArray:
            return _f.intArrays.size();
        case InputTag::FloatArray:
            return _f.floatArrays.size();
        case InputTag::DoubleArray:
            return _f.doubleArrays.size();
        case InputTag::Vec2fArray:
            return _f.vec2fArrays.size();
        case InputTag::Vec3fArray:
            return _f.vec3fArrays.size();
        default:
            return 0;
        }
    }

    /// An array value: no bits, and its elements in the pool of its tag, or
    /// in a revision's stored layout, indices as int[] and weights as
    /// float[] (_LayoutValues checks the revision once the chains are).
    bool _ArrayValue(const fb::RigExecWireValue &value,
                     const std::string &where)
    {
        if (value.bits != 0) {
            return _Bad(where + ": bits or members on an array value");
        }
        switch (value.arraySource) {
        case fb::ArraySource::Pool:
            return _Pool(value.array, _PoolSize(value.tag), where, "array");
        case fb::ArraySource::SkinIndices:
        case fb::ArraySource::SkinWeights: {
            const InputTag want =
                value.arraySource == fb::ArraySource::SkinIndices
                    ? InputTag::IntArray
                    : InputTag::FloatArray;
            if (value.tag != want) {
                return _Bad(where + ": a skin layout source on a " +
                            _TagName(value.tag) + " value");
            }
            return _Index(int64_t(value.array), _revisions, false, where,
                          "array");
        }
        }
        return _Bad(where + ": array source out of range");
    }

    /// Each array value a revision's layout holds names a main revision
    /// whose layout the epoch fixes and the file stores.
    bool _LayoutValues()
    {
        const fb::RigExecWireDomainGeometry &g = *_f.geometry;
        for (size_t i = 0; i < _f.values.size(); ++i) {
            const fb::RigExecWireValue &value = _f.values[i];
            if (!RigExecFormatIsArrayTag(value.tag) ||
                value.arraySource == fb::ArraySource::Pool) {
                continue;
            }
            const RigExecWireIntPair &at = g.revisionIndex[value.array];
            const fb::RigExecWireRevision &r =
                g.chains[size_t(at.first)].revisions[size_t(at.second)];
            if (!r.skinTopologyFixed || !r.topologyResolved || !r.topology) {
                return _Bad("values[" + _N(i) + "]: revision " +
                            _N(value.array) + " stores no fixed layout");
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
            if (RigExecFormatIsArrayTag(slot.type()) &&
                (slot.chain() != -1 || slot.phased() != -1)) {
                return _Bad(where + ": an array slot names a chain or a "
                                    "phased consumer");
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

    /// "step N (<label>)", the program's name for a step in its refusals.
    std::string _StepName(size_t step) const
    {
        return "step " + _N(step) + " (" + RigExecFormatStepLabel(_f, step) +
               ")";
    }

    /// One VolumePlacements step places one volume: part 1, its object a
    /// volume slot (no_scale_avars), its one write that slot of
    /// WeightFrames, and no other step placing the same slot. \p placer
    /// maps each slot to the step that places it, or -1.
    bool _VolumePlacement(size_t i, std::vector<int64_t> *placer)
    {
        const fb::RigExecWireStep &step = _f.steps[i];
        if (step.part != 1) {
            return _Bad(_StepName(i) +
                        (step.part == -1
                             ? " is the retired whole-map placement (part -1)"
                             : " has part " + std::to_string(step.part) +
                                   ", which is no VolumePlacements form"));
        }
        const std::vector<uint8_t> &volumes = _f.constants->noScaleAvars;
        if (!_Has(volumes, step.object) || !volumes[size_t(step.object)]) {
            return _Bad(_StepName(i) + " places slot " +
                        std::to_string(step.object) +
                        ", which is no volume slot");
        }
        const uint32_t slot = uint32_t(step.object);
        if (step.writes.size() != 1 ||
            step.writes[0].domain() != fb::SlotDomain::WeightFrames ||
            step.writes[0].begin() != slot ||
            step.writes[0].end() != slot + 1) {
            return _Bad(_StepName(i) + " writes other than WeightFrames[" +
                        _N(slot) + "]");
        }
        const int64_t first = (*placer)[slot];
        if (first >= 0) {
            return _Bad(_StepName(i) + " places volume slot " + _N(slot) +
                        " again; " + _StepName(size_t(first)) +
                        " already does");
        }
        (*placer)[slot] = int64_t(i);
        return true;
    }

    bool _Steps()
    {
        // The VolumePlacements step of each volume slot. With one step per
        // volume and no other writer of WeightFrames, the step graph's
        // producer check puts each volume's step before every reader of its
        // slot.
        std::vector<int64_t> placer(_slots, -1);
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
        }
        // The reserved values ahead of every rule on what a step of a kind
        // reads or writes, so a retired step is named as one rather than
        // by the rule its kind breaks.
        const std::string reserved =
            RigExecStepGraphReservedError(_f.steps, _GraphRange);
        if (!reserved.empty()) {
            return _Bad(reserved);
        }
        for (size_t i = 0; i < _f.steps.size(); ++i) {
            const fb::RigExecWireStep &step = _f.steps[i];
            if (step.kind == fb::StepKind::WeightPacket &&
                (step.object < 0 || size_t(step.object) >= _stepBacked)) {
                return _Bad("steps[" + _N(i) +
                            "]: a WeightPacket step names no step-backed "
                            "weight object");
            }
            if (step.kind == fb::StepKind::VolumePlacements) {
                if (!_VolumePlacement(i, &placer)) {
                    return false;
                }
                continue;
            }
            for (const fb::SlotRange &range : step.writes) {
                if (range.domain() == fb::SlotDomain::WeightFrames) {
                    return _Bad(_StepName(i) +
                                " writes WeightFrames, which only a "
                                "VolumePlacements step writes");
                }
            }
        }
        const std::vector<uint8_t> &volumes = _f.constants->noScaleAvars;
        for (size_t slot = 0; slot < volumes.size(); ++slot) {
            if (volumes[slot] && placer[slot] < 0) {
                return _Bad("volume slot " + _N(slot) + " (" +
                            RigExecFormatPathText(
                                _f, _f.slotMeta->paths[slot]) +
                            ") has no VolumePlacements step");
            }
        }
        return true;
    }

    /// Every step and cluster index, edge order, producers, membership, the
    /// partition and the acyclic cluster graph: the rules the runtime's
    /// reader applies, in its words. It runs after the phase bindings, so a
    /// reader ordered before what it is bound to is named by the binding's
    /// own rule rather than as an unproduced read.
    bool _StepGraph()
    {
        const std::string why =
            RigExecStepGraphError(_f.steps, *_f.clustering, _GraphRange);
        return why.empty() || _Bad(why);
    }

    bool _HeadChecks()
    {
        const size_t slots = _f.slotMeta->paths.size();
        const size_t layouts = _f.geometry->revisionIndex.size() + _f.geometry->derivedIndex.size();
        size_t versions = 0;
        for (size_t c = 0; c < _f.propertyChains.size(); ++c) {
            const auto &chain = _f.propertyChains[c];
            if (chain.versionBase != versions)
                return _Bad("PropertyRevision chain " + _N(c) + ": noncanonical version_base");
            versions += chain.revisions.size() + 1;
        }
        for (size_t r = 0; r < _f.phasedConsumers.size(); ++r)
            if (_f.phasedConsumers[r].version != versions++)
                return _Bad("PropertyRevision record " + _N(r) + ": noncanonical version");
        std::vector<int64_t> writers(versions, -1);
        std::vector<int64_t> rest(slots, -1), ladder(slots, -1), topology(layouts, -1);
        bool region = false;
        const auto producerDomain = [](fb::SlotDomain domain) {
            return domain == fb::SlotDomain::PropertyResult || domain == fb::SlotDomain::Rest ||
                   domain == fb::SlotDomain::Ladder || domain == fb::SlotDomain::SkinTopology;
        };
        for (size_t i = 0; i < _f.steps.size(); ++i) {
            const auto &step = _f.steps[i];
            const bool kindHead = step.kind >= fb::StepKind::PropertyRevision;
            if (step.isHead != kindHead || (step.isHead && region))
                return _Bad(_StepName(i) + ": heads must form the contiguous prefix with head kinds");
            region = region || !step.isHead;
            if (step.isHead && (step.isSource || step.externalReads))
                return _Bad(_StepName(i) + ": a head is never a source or always region step");
            for (const auto pred : step.preds)
                if (step.isHead && (pred < 0 || size_t(pred) >= i || !_f.steps[size_t(pred)].isHead))
                    return _Bad(_StepName(i) + ": head depends on a non-head or later producer");
            for (const auto &range : step.writes) {
                if (!producerDomain(range.domain())) continue;
                if (!step.isHead) return _Bad(_StepName(i) + ": region writes a head domain");
                auto *table = range.domain() == fb::SlotDomain::PropertyResult ? &writers
                    : range.domain() == fb::SlotDomain::Rest ? &rest
                    : range.domain() == fb::SlotDomain::Ladder ? &ladder : &topology;
                if (range.end() > table->size()) return _Bad(_StepName(i) + ": head write out of range");
                const auto required = range.domain() == fb::SlotDomain::PropertyResult ? fb::StepKind::PropertyRevision
                    : range.domain() == fb::SlotDomain::Rest ? fb::StepKind::RestCompose
                    : range.domain() == fb::SlotDomain::Ladder ? fb::StepKind::LadderCompose : fb::StepKind::SkinTopology;
                if (step.kind != required) return _Bad(_StepName(i) + ": wrong head domain writer kind");
                for (uint32_t v = range.begin(); v < range.end(); ++v) {
                    if ((*table)[v] >= 0) return _Bad(_StepName(i) + ": duplicate head domain writer");
                    (*table)[v] = int64_t(i);
                }
            }
            if (step.kind == fb::StepKind::PropertyRevision) {
                if (!_Has(_f.propertyChains, step.object)) return _Bad(_StepName(i) + ": invalid chain object");
                const auto &chain = _f.propertyChains[size_t(step.object)];
                if (step.part < 0 || size_t(step.part) > chain.revisions.size())
                    return _Bad(_StepName(i) + ": invalid property part");
                std::set<uint32_t> expected{chain.versionBase + uint32_t(step.part)};
                for (const auto &record : _f.phasedConsumers)
                    if (record.chain == uint32_t(step.object) && record.applied == uint32_t(step.part))
                        expected.insert(record.version);
                std::set<uint32_t> actual;
                for (const auto &range : step.writes) {
                    if (range.domain() != fb::SlotDomain::PropertyResult)
                        return _Bad(_StepName(i) + ": property part writes a foreign domain");
                    for (uint32_t v = range.begin(); v < range.end(); ++v) actual.insert(v);
                }
                if (actual != expected) return _Bad(_StepName(i) + ": property version/record ownership differs");
                if (step.part && !_Declares(i, fb::SlotDomain::PropertyResult,
                                           chain.versionBase + uint32_t(step.part) - 1))
                    return _Bad(_StepName(i) + ": part does not declare predecessor version");
            } else if (step.kind == fb::StepKind::RestCompose || step.kind == fb::StepKind::LadderCompose) {
                if (!_Has(_f.pose->composeGroups, step.object)) return _Bad(_StepName(i) + ": invalid compose group");
                const auto &group = _f.pose->composeGroups[size_t(step.object)];
                const auto domain = step.kind == fb::StepKind::RestCompose ? fb::SlotDomain::Rest : fb::SlotDomain::Ladder;
                if (step.part != (step.kind == fb::StepKind::RestCompose ? 0 : 1) ||
                    step.writes.size() != 1 || step.writes[0].domain() != domain ||
                    step.writes[0].begin() != uint32_t(group.begin) || step.writes[0].end() != uint32_t(group.end))
                    return _Bad(_StepName(i) + ": compose output is not its group range");
            } else if (step.kind == fb::StepKind::SkinTopology) {
                if (step.object < 0 || size_t(step.object) >= layouts || step.part != 0 ||
                    step.writes.size() != 1 || step.writes[0].domain() != fb::SlotDomain::SkinTopology ||
                    step.writes[0].begin() != uint32_t(step.object) || step.writes[0].end() != uint32_t(step.object + 1))
                    return _Bad(_StepName(i) + ": invalid topology output ownership");
            }
            for (const auto slot : step.headInputSlots)
                if (slot >= _f.inputs.size()) return _Bad(_StepName(i) + ": memo slot out of range");
            if (!std::is_sorted(step.headInputSlots.begin(), step.headInputSlots.end()) ||
                std::adjacent_find(step.headInputSlots.begin(), step.headInputSlots.end()) != step.headInputSlots.end())
                return _Bad(_StepName(i) + ": memo slots are not sorted unique");
            if (!step.isHead && (!step.headInputSlots.empty() || !step.headInputReads.empty() ||
                                 step.headVaryingLeaves || step.headAlwaysRuns))
                return _Bad(_StepName(i) + ": region carries head memo metadata");
            if (step.kind == fb::StepKind::PropertyRevision) {
                const auto &chain = _f.propertyChains[size_t(step.object)];
                std::set<uint32_t> expectedSlots;
                bool volatileBody = false;
                if (step.part == 0) expectedSlots.insert(chain.target);
                else {
                    const auto &revision = chain.revisions[size_t(step.part - 1)];
                    volatileBody = revision.envelope >= 0;
                    if (volatileBody) {
                        std::vector<int32_t> pending{revision.envelope};
                        std::unordered_set<int32_t> visited;
                        while (!pending.empty()) {
                            const int32_t object = pending.back();
                            pending.pop_back();
                            if (!visited.insert(object).second) continue;
                            if (!_Has(_f.geometry->weightObjects, object))
                                return _Bad(_StepName(i) + ": property envelope object out of range");
                            const auto &weight = _f.geometry->weightObjects[size_t(object)];
                            const std::string type = RigExecFormatPathText(_f, weight.type);
                            if (type == "RigExecSphereWeight" || type == "RigExecPlaneWeight" ||
                                type == "RigExecCurveWeight")
                                return _Bad(_StepName(i) + ": property envelope contains a volume");
                            if (weight.base >= 0) pending.push_back(weight.base);
                            pending.insert(pending.end(), weight.inputs.begin(), weight.inputs.end());
                        }
                    }
                    for (const auto *input : {revision.enabled.get(), revision.defaultWeight.get(),
                                             revision.value.get(), revision.min.get(), revision.max.get()}) {
                        if (!input) continue;
                        for (const auto *list : {&input->propertyCandidates, &input->doubleCandidates})
                            for (const auto &candidate : *list)
                                if (!(_f.inputs[candidate.slot].type() >= InputTag::IntArray))
                                    expectedSlots.insert(candidate.slot);
                    }
                }
                if (std::set<uint32_t>(step.headInputSlots.begin(), step.headInputSlots.end()) != expectedSlots ||
                    !step.headInputReads.empty())
                    return _Bad(_StepName(i) + ": property memo does not cover exactly its body inputs");
                if (step.headAlwaysRuns != volatileBody)
                    return _Bad(_StepName(i) + ": volatile property memo flag differs from its envelope");
            } else if (step.headAlwaysRuns)
                return _Bad(_StepName(i) + ": non-property head is volatile");
            if (step.kind == fb::StepKind::SkinTopology) {
                const size_t layout = size_t(step.object);
                const bool derived = layout >= _f.geometry->revisionIndex.size();
                const auto &index = derived ? _f.geometry->derivedIndex[layout - _f.geometry->revisionIndex.size()]
                                            : _f.geometry->revisionIndex[layout];
                const auto &revision = derived ? *_f.geometry->chains[size_t(index.first)].derived[size_t(index.second)].revision
                                              : _f.geometry->chains[size_t(index.first)].revisions[size_t(index.second)];
                if (!revision.skinTopologyFixed)
                    return _Bad(_StepName(i) + ": topology head has no fixed layout body");
                std::set<uint32_t> expected;
                for (size_t slot = 0; slot < _f.inputs.size(); ++slot)
                    if (_SlotIs(int32_t(slot), revision.moverPath, "rigExec:jointIndices") ||
                        _SlotIs(int32_t(slot), revision.moverPath, "rigExec:jointWeights") ||
                        _SlotIs(int32_t(slot), revision.moverPath, "rigExec:elementSize")) expected.insert(uint32_t(slot));
                if (std::set<uint32_t>(step.headInputSlots.begin(), step.headInputSlots.end()) != expected ||
                    !step.headInputReads.empty())
                    return _Bad(_StepName(i) + ": topology memo does not cover exactly its layout leaves");
            }
            if (step.kind == fb::StepKind::PropertyRevision || step.kind == fb::StepKind::SkinTopology) {
                if (step.headVaryingLeaves)
                    return _Bad(_StepName(i) + ": raw head carries binding-varying flag");
            }
            if (step.kind == fb::StepKind::RestCompose || step.kind == fb::StepKind::LadderCompose) {
                const auto &group = _f.pose->composeGroups[size_t(step.object)];
                std::vector<const RigExecWireInput *> expected;
                const bool restBody = step.kind == fb::StepKind::RestCompose;
                for (int slot = group.begin; slot < group.end; ++slot) {
                    if (_f.slotMeta->slotKind[size_t(slot)] != fb::SlotKind::FirstFramePose) continue;
                    const auto &ladderBody = _f.pose->ladders[size_t(slot)];
                    if (restBody) {
                        for (const auto &read : ladderBody.restAvars) expected.push_back(&read);
                        expected.push_back(ladderBody.restSpace.get());
                    } else {
                        expected.push_back(ladderBody.posedSpace.get());
                        expected.push_back(ladderBody.defaultSpace.get());
                        for (const auto &read : ladderBody.defaultAvars) expected.push_back(&read);
                        expected.push_back(ladderBody.rotationOrder.get());
                    }
                }
                const auto same = [](const auto &a, const auto &b) {
                    return a.tag == b.tag && a.mode == b.mode && a.flags == b.flags &&
                        a.constant == b.constant && a.selected == b.selected &&
                        a.overrideIndex == b.overrideIndex && a.walk == b.walk &&
                        a.propertyCandidates == b.propertyCandidates &&
                        a.doubleCandidates == b.doubleCandidates && a.rawFallbackSlot == b.rawFallbackSlot;
                };
                bool varying = false;
                if (!step.headInputSlots.empty() || step.headInputReads.size() != expected.size())
                    return _Bad(_StepName(i) + ": compose memo does not cover exactly its body bindings");
                for (size_t r = 0; r < expected.size(); ++r) {
                    if (!expected[r] || !same(*expected[r], step.headInputReads[r]))
                        return _Bad(_StepName(i) + ": compose memo binding order or value differs from its body");
                    varying = varying || (expected[r]->flags & uint16_t(fb::InputReadFlags::Varying));
                }
                if (step.headVaryingLeaves != varying)
                    return _Bad(_StepName(i) + ": compose binding-varying flag differs from its body");
            }
            for (const auto &pair : step.shadowedReads)
                if (pair.first < 0 || size_t(pair.first) >= versions || pair.second < 0 ||
                    size_t(pair.second) >= _f.phasedConsumers.size() ||
                    !_Declares(i, fb::SlotDomain::PropertyResult, uint32_t(pair.first)))
                    return _Bad(_StepName(i) + ": invalid shadowed property read");
            const size_t registered = _overrideUses.size();
            for (size_t r = 0; r < step.headInputReads.size(); ++r)
                if (!_Read(&step.headInputReads[r], -1, _Baked,
                           _StepName(i), "head_input_reads", long(r))) return false;
            _overrideUses.resize(registered);
        }
        std::map<const RigExecWireInput *, size_t> owners;
        for (size_t i = 0; i < _f.steps.size(); ++i) {
            const auto &step = _f.steps[i];
            for (const auto &read : step.headInputReads) owners.emplace(&read, i);
            if (step.kind != fb::StepKind::PropertyRevision || step.part == 0) continue;
            const auto &revision = _f.propertyChains[size_t(step.object)].revisions[size_t(step.part - 1)];
            for (const auto *read : {revision.enabled.get(), revision.defaultWeight.get(), revision.value.get(),
                                     revision.min.get(), revision.max.get()}) owners.emplace(read, i);
        }
        for (const auto &[input, label] : _readUses) {
            if (input->mode == ReadMode::Raw) continue;
            const auto owner = owners.find(input);
            const int chainLimit = owner != owners.end() &&
                _f.steps[owner->second].kind == fb::StepKind::PropertyRevision
                ? _f.steps[owner->second].object : int(_f.propertyChains.size());
            for (const uint32_t slotId : input->walk) {
                const auto &slot = _f.inputs[slotId];
                const bool chain = slot.chain() >= 0 && slot.chain() < chainLimit;
                const bool record = slot.phased() >= 0 &&
                    _f.phasedConsumers[size_t(slot.phased())].chain < uint32_t(chainLimit);
                if (!chain && !record) continue;
                const auto expected = chain ? fb::PropertyCandidateKind::ChainFinal : fb::PropertyCandidateKind::PhasedRecord;
                bool found = false;
                for (const auto *list : {&input->propertyCandidates, &input->doubleCandidates})
                    for (const auto &candidate : *list)
                        found = found || (candidate.slot == slotId && candidate.kind == uint8_t(expected));
                if (!found) return _Bad((owner == owners.end() ? label : _StepName(owner->second)) +
                                        ": chain crossing lacks its property candidate");
            }
            size_t doubleStart = 0;
            if (!input->doubleCandidates.empty()) {
                if (input->walk.empty()) return _Bad(label + ": double segment has no walk");
                const uint32_t start = input->propertyCandidates.empty() ? input->walk.front()
                    : input->propertyCandidates.back().slot;
                const auto at = std::find(input->walk.begin(), input->walk.end(), start);
                if (at == input->walk.end() || _f.inputs[start].type() != InputTag::Double ||
                    input->doubleCandidates.front().slot != start)
                    return _Bad(label + ": double segment does not start at its double hop");
                doubleStart = size_t(at - input->walk.begin());
                const size_t suffix = input->walk.size() - doubleStart;
                if (input->doubleCandidates.size() < suffix)
                    return _Bad(label + ": double segment omits its walk suffix");
                for (size_t k = 0; k < suffix; ++k)
                    if (input->doubleCandidates[k].slot != input->walk[doubleStart + k])
                        return _Bad(label + ": double segment differs from its walk suffix");
            }
            for (const auto *list : {&input->propertyCandidates, &input->doubleCandidates}) {
                size_t previous = 0;
                bool first = true;
                for (const auto &candidate : *list) {
                    const auto &slot = _f.inputs[candidate.slot];
                    const bool chain = slot.chain() >= 0 && slot.chain() < chainLimit;
                    const bool record = slot.phased() >= 0 &&
                        _f.phasedConsumers[size_t(slot.phased())].chain < uint32_t(chainLimit);
                    const auto expected = chain ? fb::PropertyCandidateKind::ChainFinal :
                        record ? fb::PropertyCandidateKind::PhasedRecord : fb::PropertyCandidateKind::SlotOnly;
                    if (candidate.kind != uint8_t(expected))
                        return _Bad(label + ": candidate kind disagrees with its reader phase");
                    const bool raw = list == &input->doubleCandidates || input->doubleCandidates.empty();
                    if (candidate.raw != raw)
                        return _Bad(label + ": candidate raw flag disagrees with its segment");
                    const auto at = std::find(input->walk.begin(), input->walk.end(), candidate.slot);
                    if (at == input->walk.end()) return _Bad(label + ": candidate slot is not on its walk");
                    size_t position = size_t(at - input->walk.begin());
                    if (list == &input->doubleCandidates)
                        position = (position + input->walk.size() - doubleStart) % input->walk.size();
                    if (!first && position <= previous) return _Bad(label + ": property candidates are out of walk order");
                    first = false; previous = position;
                    if (candidate.version >= 0 && owner != owners.end() &&
                        !_Declares(owner->second, fb::SlotDomain::PropertyResult, uint32_t(candidate.version)))
                        return _Bad(_StepName(owner->second) + ": candidate version is not a declared read");
                }
            }
            if (input->rawFallbackSlot >= 0 &&
                (input->walk.empty() || input->rawFallbackSlot != int32_t(input->walk.front())))
                return _Bad(label + ": own raw fallback is not its head slot");
        }
        for (size_t v = 0; v < writers.size(); ++v)
            if (writers[v] < 0) return _Bad("PropertyRevision version " + _N(v) + ": no writer");
        for (size_t i = 0; i < _f.steps.size(); ++i) {
            const auto &step = _f.steps[i];
            for (const auto &range : step.reads) {
                if (!producerDomain(range.domain())) continue;
                const auto *table = range.domain() == fb::SlotDomain::PropertyResult ? &writers
                    : range.domain() == fb::SlotDomain::Rest ? &rest
                    : range.domain() == fb::SlotDomain::Ladder ? &ladder : &topology;
                if (range.end() > table->size()) return _Bad(_StepName(i) + ": head read out of range");
                for (uint32_t v = range.begin(); v < range.end(); ++v)
                    if ((*table)[v] < 0 || size_t((*table)[v]) >= i)
                        return _Bad(_StepName(i) + ": head read has no earlier producer");
            }
            const auto require = [&](fb::SlotDomain domain, int slot) {
                if (slot < 0 || size_t(slot) >= slots) return true;
                return _Declares(i, domain, uint32_t(slot)) ||
                    _Bad(_StepName(i) + ": missing " + rigExecStepGraphDetail::DomainName(uint8_t(domain)) +
                         "[" + _N(size_t(slot)) + "] read");
            };
            if (step.kind == fb::StepKind::RestCompose || step.kind == fb::StepKind::LadderCompose) {
                const auto &group = _f.pose->composeGroups[size_t(step.object)];
                const bool ladderBody = step.kind == fb::StepKind::LadderCompose;
                for (int slot = group.begin; slot < group.end; ++slot) {
                    if (ladderBody && !require(fb::SlotDomain::Rest, slot)) return false;
                    if (_f.slotMeta->slotKind[size_t(slot)] != fb::SlotKind::FirstFramePose) continue;
                    const int parent = _f.slotMeta->parent[size_t(slot)];
                    if (parent >= 0 && (parent < group.begin || parent >= group.end)) {
                        if (!require(fb::SlotDomain::Rest, parent) ||
                            (ladderBody && !require(fb::SlotDomain::Ladder, parent))) return false;
                    }
                }
            }
            const fb::RigExecWireRevision *bodyRevision = nullptr;
            size_t layoutId = 0;
            if ((step.kind == fb::StepKind::RevisionStatic || step.kind == fb::StepKind::RevisionChunk ||
                 step.kind == fb::StepKind::RevisionFuse) && _Has(_f.geometry->revisionIndex, step.object)) {
                const auto &index = _f.geometry->revisionIndex[size_t(step.object)];
                bodyRevision = &_f.geometry->chains[size_t(index.first)].revisions[size_t(index.second)];
                layoutId = size_t(step.object);
            } else if (step.kind == fb::StepKind::Derived && _Has(_f.geometry->derivedIndex, step.object)) {
                const auto &index = _f.geometry->derivedIndex[size_t(step.object)];
                bodyRevision = _f.geometry->chains[size_t(index.first)].derived[size_t(index.second)].revision.get();
                layoutId = _f.geometry->revisionIndex.size() + size_t(step.object);
                if (bodyRevision && (!require(fb::SlotDomain::Rest, bodyRevision->transformSlot) ||
                    !require(fb::SlotDomain::Rest, bodyRevision->transformSpaceSlot) ||
                    !require(fb::SlotDomain::Rest, bodyRevision->carrySpaceSlot))) return false;
            }
            if (bodyRevision && bodyRevision->chunked &&
                step.kind == fb::StepKind::RevisionChunk) {
                if (!_Has(bodyRevision->chunks, step.part))
                    return _Bad(_StepName(i) + ": chunk part out of range");
                const auto domain = bodyRevision->finalPhase ? fb::SlotDomain::FinalMatrix
                                                             : fb::SlotDomain::BaseMatrix;
                std::set<uint32_t> expected, actual;
                for (const int position : bodyRevision->chunks[size_t(step.part)].key) {
                    if (!_Has(bodyRevision->influenceSlots, position))
                        return _Bad(_StepName(i) + ": chunk influence out of range");
                    const int slot = bodyRevision->influenceSlots[size_t(position)];
                    if (slot >= 0) expected.insert(uint32_t(slot));
                }
                for (const auto &range : step.reads) {
                    if (range.domain() != fb::SlotDomain::BaseMatrix &&
                        range.domain() != fb::SlotDomain::FinalMatrix &&
                        range.domain() != fb::SlotDomain::FrameMatrix) continue;
                    if (range.domain() != domain)
                        return _Bad(_StepName(i) + ": chunk matrix reads differ from its influence key");
                    for (uint32_t slot = range.begin(); slot < range.end(); ++slot)
                        actual.insert(slot);
                }
                if (actual != expected)
                    return _Bad(_StepName(i) + ": chunk matrix reads differ from its influence key");
            }
            if (bodyRevision && bodyRevision->skinTopologyFixed &&
                !_Declares(i, fb::SlotDomain::SkinTopology, uint32_t(layoutId)))
                return _Bad(_StepName(i) + ": missing SkinTopology[" + _N(layoutId) + "] read");
            if (step.kind == fb::StepKind::ProviderMatrix && !require(fb::SlotDomain::Rest, step.object)) return false;
            if (step.kind == fb::StepKind::FrameMatrix && _Has(_f.pose->frameRecords, step.object) &&
                !require(fb::SlotDomain::Rest, int(_f.pose->frameRecords[size_t(step.object)].slot()))) return false;
            if (step.kind == fb::StepKind::Solve && _Has(_f.pose->solvers, step.object))
                for (const auto slot : _f.pose->solvers[size_t(step.object)].restSlots)
                    if (!require(fb::SlotDomain::Rest, slot)) return false;
            if (step.kind == fb::StepKind::ComposeSubtree && _Has(_f.pose->composeGroups, step.object)) {
                const auto &group = _f.pose->composeGroups[size_t(step.object)];
                for (int slot = group.begin; slot < group.end; ++slot)
                    if (!require(fb::SlotDomain::Ladder, slot)) return false;
                for (const int slot : group.parentSlots)
                    if (!require(fb::SlotDomain::Ladder, slot)) return false;
                for (const auto &sw : _f.pose->spaceSwitches) {
                    if (sw.slot < group.begin || sw.slot >= group.end) continue;
                    for (const int slot : sw.sourceSlots)
                        if (!require(fb::SlotDomain::Ladder, slot)) return false;
                    if (!require(fb::SlotDomain::Ladder, sw.spaceSlot) ||
                        !require(fb::SlotDomain::Ladder, _f.slotMeta->parent[size_t(sw.slot)])) return false;
                    for (const auto *read : {sw.parentRead.get(), sw.spaceRead.get()})
                        if (read) for (const auto slot : read->recompose)
                            if (!require(fb::SlotDomain::Ladder, slot)) return false;
                    for (const auto &read : sw.sourceReads)
                        for (const auto slot : read.recompose)
                            if (!require(fb::SlotDomain::Ladder, slot)) return false;
                }
            }
            if (step.kind == fb::StepKind::PoseInterpolator && _Has(_f.pose->poseInterpolators, step.object)) {
                const auto &interp = _f.pose->poseInterpolators[size_t(step.object)];
                if (interp.valueInputs.empty() &&
                    (!require(fb::SlotDomain::Rest, interp.driverSlot) ||
                     !require(fb::SlotDomain::Rest, interp.parentSlot))) return false;
            }
            if (step.kind == fb::StepKind::Constraint && _Has(_f.pose->walkSteps, step.object)) {
                const auto &walk = _f.pose->walkSteps[size_t(step.object)];
                if (walk.solverBatch || !_Has(_f.pose->constraints, walk.index)) continue;
                const auto &constraint = _f.pose->constraints[size_t(walk.index)];
                if (!constraint.useAnimatedTs)
                    for (const int slot : constraint.targetSlots)
                        if (!require(fb::SlotDomain::Rest, slot)) return false;
                if (RigExecFormatPathText(_f, constraint.type) == "RigExecMatrixMover")
                    for (const int slot : constraint.sources)
                        if (!require(fb::SlotDomain::Rest, slot)) return false;
                if (!require(fb::SlotDomain::Ladder, constraint.spaceSlot)) return false;
            }
        }
        for (size_t layout = 0; layout < layouts; ++layout) {
            const bool derived = layout >= _f.geometry->revisionIndex.size();
            const auto &index = derived ? _f.geometry->derivedIndex[layout - _f.geometry->revisionIndex.size()]
                                        : _f.geometry->revisionIndex[layout];
            const auto &chain = _f.geometry->chains[size_t(index.first)];
            const auto *revision = derived ? chain.derived[size_t(index.second)].revision.get()
                                           : &chain.revisions[size_t(index.second)];
            if (revision && revision->skinTopologyFixed && topology[layout] < 0)
                return _Bad("SkinTopology layout " + _N(layout) + ": fixed layout body has no producer");
        }
        return true;
    }

    /// A step's read or write range as the step graph checks it.
    static RigExecStepGraphRange _GraphRange(const fb::SlotRange &range)
    {
        return RigExecStepGraphRange{uint8_t(range.domain()), range.begin(),
                                     range.end()};
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
        // Every slot's avars and every revision's static step belong to a
        // cluster: the runtime dirties it without a -1 check.
        if (!_Size(c.avarCluster.size(), _slots, row, "avar_cluster") ||
            !_Indices(c.avarCluster, _clusters, false, row, "avar_cluster") ||
            !_Size(c.chainBaseClusters.size(), _chains, row,
                   "chain_base_clusters") ||
            !_Size(c.revisionClusters.size(), _revisions, row,
                   "revision_clusters") ||
            !_Size(c.revisionStaticCluster.size(), _revisions, row,
                   "revision_static_cluster") ||
            !_Indices(c.revisionStaticCluster, _clusters, false, row,
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
        // Slot -> the switch storing it. The runtime looks a switch up by
        // its slot, so a slot is switched at most once.
        std::vector<int32_t> switchOf(_slots, -1);
        for (size_t i = 0; i < p.spaceSwitches.size(); ++i) {
            const fb::RigExecWireSpaceSwitch &sw = p.spaceSwitches[i];
            const std::string row = "pose.space_switches[" + _N(i) + "]";
            if (!_Index(sw.slot, _slots, false, row, "slot")) {
                return false;
            }
            if (switchOf[size_t(sw.slot)] != -1) {
                return _Bad(row + ".slot: slot " + _N(size_t(sw.slot)) +
                            " is already switched by "
                            "pose.space_switches[" +
                            _N(size_t(switchOf[size_t(sw.slot)])) + "]");
            }
            switchOf[size_t(sw.slot)] = int32_t(i);
        }
        for (size_t i = 0; i < p.spaceSwitches.size(); ++i) {
            const fb::RigExecWireSpaceSwitch &sw = p.spaceSwitches[i];
            const std::string row = "pose.space_switches[" + _N(i) + "]";
            if (!_Indices(sw.sourceSlots, _slots, true, row,
                          "source_slots") ||
                !_Index(sw.spaceSlot, _slots, true, row, "space_slot") ||
                !_ReadPtr(sw.active, int(InputTag::Double), _Baked, row,
                          "active")) {
                return false;
            }
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
                       "source_reads") ||
                !_ResolvedAnchor(*sw.parentRead, switchOf, i,
                                 row + ".parent_read") ||
                !_ResolvedAnchor(*sw.spaceRead, switchOf, i,
                                 row + ".space_read")) {
                return false;
            }
            for (size_t k = 0; k < sw.sourceReads.size(); ++k) {
                const std::string at = _At(row, "source_reads", long(k));
                if (!_FrameVersion(&sw.sourceReads[k], at) ||
                    (sw.sourceSlots[k] == -1 &&
                     !_UnreadVersion(sw.sourceReads[k], at)) ||
                    !_ResolvedAnchor(sw.sourceReads[k], switchOf, i, at)) {
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
        _finPool = finPool;
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
        for (size_t c = 0, first = 0; c < _chains; ++c) {
            if (!_Chain(g.chains[c], "geometry.chains[" + _N(c) + "]",
                        first)) {
                return false;
            }
            first += g.chains[c].revisions.size();
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

    /// An input slot field: -1, or a slot of tag \p tag.
    bool _InputSlotOf(int32_t slot, InputTag tag, const std::string &row,
                      const char *field)
    {
        if (slot == -1) {
            return true;
        }
        if (!_Index(slot, _f.inputs.size(), false, row, field)) {
            return false;
        }
        if (_f.inputs[size_t(slot)].type() != tag) {
            return _Bad(_At(row, field) + ": slot " + _N(size_t(slot)) +
                        " is not a " + _TagName(tag) + " input");
        }
        return true;
    }

    /// The default of input slot \p slot.
    const fb::RigExecWireValue &_DefaultOf(int32_t slot) const
    {
        return _f.values[_f.inputs[size_t(slot)].value()];
    }

    /// Whether input slot \p slot is attribute \p name of the prim at path
    /// id \p prim, so that a set of one attribute reaches only its reads.
    bool _SlotIs(int32_t slot, uint32_t prim, const char *name) const
    {
        return RigExecFormatPathText(_f, _f.inputs[size_t(slot)].name()) ==
               RigExecFormatPathText(_f, prim) + "." + name;
    }

    /// \p chain, whose revisions take flat ids from \p firstRevision. Its
    /// base slot is the target's points input, a float3[] slot whose
    /// default is the base itself.
    bool _Chain(const fb::RigExecWireChain &chain, const std::string &row,
                size_t firstRevision)
    {
        if (!_PathId(chain.target, _AnyPath, row, "target") ||
            !_Pool(chain.base, _f.vec3fArrays.size(), row, "base")) {
            return false;
        }
        if (!chain.haveBase && chain.base != 0) {
            return _Bad(row + ": a base without have_base");
        }
        if (!_InputSlotOf(chain.baseSlot, InputTag::Vec3fArray, row,
                          "base_slot")) {
            return false;
        }
        if (chain.baseSlot != -1) {
            const std::string at = _At(row, "base_slot");
            const std::string slot = "slot " + _N(size_t(chain.baseSlot));
            const fb::RigExecWireValue &value = _DefaultOf(chain.baseSlot);
            if (!chain.haveBase) {
                return _Bad(at + ": a base slot on a chain without a base");
            }
            if (_f.inputs[size_t(chain.baseSlot)].name() != chain.target) {
                return _Bad(at + ": " + slot +
                            " is not the chain target's input");
            }
            if (value.arraySource != fb::ArraySource::Pool ||
                value.array != chain.base) {
                return _Bad(at + ": " + slot +
                            "'s default is not the chain's base");
            }
        }
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            if (!_Revision(chain.revisions[r], false,
                           row + ".revisions[" + _N(r) + "]",
                           firstRevision + r)) {
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
            if (!_Revision(*derived.revision, true, at + ".revision", 0)) {
                return false;
            }
        }
        return true;
    }

    /// Revision \p r, flat id \p id when not \p derived.
    bool _Revision(const fb::RigExecWireRevision &r, bool derived,
                   const std::string &row, size_t id)
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
        if (!_LayoutSlots(r, derived, row, id)) {
            return false;
        }
        return derived || _Chunks(r, row);
    }

    /// A revision's layout slots: both or neither, only on a main skin
    /// whose layout the epoch fixes, its mover's int[] rigExec:jointIndices
    /// and float[] rigExec:jointWeights, and with a stored layout defaulting
    /// to its expansion, otherwise to pool arrays.
    bool _LayoutSlots(const fb::RigExecWireRevision &r, bool derived,
                      const std::string &row, size_t id)
    {
        const bool indices = r.jointIndicesSlot != -1;
        const bool weights = r.jointWeightsSlot != -1;
        if (!indices && !weights) {
            return true;
        }
        if (indices != weights) {
            return _Bad(row + ": joint_indices_slot and joint_weights_slot "
                              "are set together");
        }
        if (derived || r.op != uint8_t(fb::RevisionOp::Skin) ||
            !r.skinTopologyFixed) {
            return _Bad(row + ": layout slots on a revision that is not a "
                              "main skin whose layout the epoch fixes");
        }
        const bool stored = r.topologyResolved && r.topology;
        const std::tuple<int32_t, InputTag, fb::ArraySource, const char *,
                         const char *>
            slots[] = {{r.jointIndicesSlot, InputTag::IntArray,
                        fb::ArraySource::SkinIndices, "joint_indices_slot",
                        "rigExec:jointIndices"},
                       {r.jointWeightsSlot, InputTag::FloatArray,
                        fb::ArraySource::SkinWeights, "joint_weights_slot",
                        "rigExec:jointWeights"}};
        for (const auto &[slot, tag, source, field, attribute] : slots) {
            if (!_InputSlotOf(slot, tag, row, field)) {
                return false;
            }
            if (!_SlotIs(slot, r.moverPath, attribute)) {
                return _Bad(_At(row, field) + ": slot " + _N(size_t(slot)) +
                            " is not the mover's " + attribute + " input");
            }
            const fb::RigExecWireValue &value = _DefaultOf(slot);
            if (stored && (value.arraySource != source ||
                           value.array != uint32_t(id))) {
                return _Bad(_At(row, field) + ": its default does not name "
                                              "this revision's layout");
            }
            if (!stored && value.arraySource != fb::ArraySource::Pool) {
                return _Bad(_At(row, field) +
                            ": its default names a layout this revision "
                            "does not store");
            }
        }
        return true;
    }

    /// A main revision's vertex partition, as the bake cuts one. Its point
    /// count is its index count in rows of its element size. A chunked
    /// revision is a skin whose layout the epoch fixes, cut into at least
    /// two ranges that tile the partition's points in order, each keyed by
    /// ascending positions in influence_slots, which size the chunk's
    /// influence rows; its partition layout, when it has one, holds the
    /// partition's element size and index count. An unchunked revision is
    /// at most one range, with no key.
    bool _Chunks(const fb::RigExecWireRevision &r, const std::string &row)
    {
        const uint64_t width = r.partitionElementSize < 1
                                   ? 0
                                   : uint64_t(r.partitionElementSize);
        const uint64_t rows = width ? r.partitionIndexCount / width : 0;
        if (r.partitionPointCount != rows) {
            return _Bad(row + ": partition_point_count " +
                        std::to_string(r.partitionPointCount) +
                        ", but partition_index_count " +
                        std::to_string(r.partitionIndexCount) + " is " +
                        std::to_string(rows) +
                        " row(s) of partition_element_size " +
                        std::to_string(r.partitionElementSize));
        }
        if (!r.chunked) {
            if (r.chunks.size() > 1) {
                return _Bad(row + ": " + _N(r.chunks.size()) +
                            " chunks, but not chunked");
            }
            for (size_t k = 0; k < r.chunks.size(); ++k) {
                if (!r.chunks[k].key.empty()) {
                    return _Bad(_At(row, "chunks", long(k)) +
                                ": a key on an unchunked revision");
                }
            }
            return true;
        }
        if (r.chunks.size() < 2) {
            return _Bad(row + ": chunked with " + _N(r.chunks.size()) +
                        " chunk(s)");
        }
        if (r.op != uint8_t(fb::RevisionOp::Skin) || !r.skinTopologyFixed) {
            return _Bad(row + ": chunked, but not a skin whose layout the "
                              "epoch fixes");
        }
        const size_t influences = r.influenceSlots.size();
        uint64_t at = 0;
        for (size_t k = 0; k < r.chunks.size(); ++k) {
            const fb::RigExecWireChunk &chunk = r.chunks[k];
            const std::string where = _At(row, "chunks", long(k));
            const std::string range = "[" + std::to_string(chunk.begin) +
                                      ", " + std::to_string(chunk.end) +
                                      ")";
            if (chunk.begin < 0 || uint64_t(chunk.begin) != at) {
                return _Bad(where + ": vertex range " + range +
                            " does not continue the partition at " +
                            std::to_string(at));
            }
            if (chunk.end < chunk.begin) {
                return _Bad(where + ": vertex range " + range +
                            " ends before it begins");
            }
            at = uint64_t(chunk.end);
            for (size_t j = 0; j < chunk.key.size(); ++j) {
                const int32_t position = chunk.key[j];
                if (position < 0 || size_t(position) >= influences ||
                    (j > 0 && position <= chunk.key[j - 1])) {
                    return _Bad(_At(where, "key", long(j)) + ": " +
                                std::to_string(position) +
                                " is not an ascending influence position "
                                "below " +
                                _N(influences));
                }
            }
        }
        if (at != r.partitionPointCount) {
            return _Bad(_At(row, "chunks", long(r.chunks.size() - 1)) +
                        ": the last range ends at " + std::to_string(at) +
                        ", not at the partition's " +
                        std::to_string(r.partitionPointCount) + " point(s)");
        }
        const fb::RigExecWireSkinTopology *layout =
            r.partitionTopology ? r.partitionTopology.get()
            : r.partitionSameAsTopology ? r.topology.get()
                                        : nullptr;
        if (!layout) {
            return _Bad(row + ": chunked partition has no layout");
        }
        if (layout) {
            const uint64_t entries =
                layout->raw ? uint64_t(layout->rawIndices.size())
                            : layout->pointCount *
                                  uint64_t(std::max(layout->elementSize, 0));
            if (r.partitionElementSize != layout->elementSize ||
                r.partitionIndexCount != entries) {
                return _Bad(row + ": the partition's element_size and "
                                  "index_count are not its layout's");
            }
            std::vector<int32_t> indices;
            std::vector<float> weights;
            RigExecFormatExpandTopology(*layout, &indices, &weights);
            const size_t width = size_t(std::max(r.partitionElementSize, 0));
            for (const auto &chunk : r.chunks) {
                std::vector<int32_t> key;
                for (size_t point = size_t(chunk.begin); point < size_t(chunk.end); ++point) {
                    for (size_t k = 0; k < width; ++k) {
                        const size_t at = point * width + k;
                        if (at >= indices.size())
                            return _Bad(row + ": chunk range exceeds its partition layout");
                        const int32_t position = indices[at];
                        if (position >= 0 && size_t(position) < r.influenceSlots.size())
                            key.push_back(position);
                    }
                }
                std::sort(key.begin(), key.end());
                key.erase(std::unique(key.begin(), key.end()), key.end());
                if (key != chunk.key) {
                    std::string owner = row;
                    for (size_t step = 0; step < _f.steps.size(); ++step) {
                        const auto &read = _f.steps[step];
                        if (read.kind != fb::StepKind::RevisionChunk ||
                            !_Has(_f.geometry->revisionIndex, read.object) ||
                            read.part != int(&chunk - r.chunks.data())) continue;
                        const auto &index = _f.geometry->revisionIndex[size_t(read.object)];
                        if (&_f.geometry->chains[size_t(index.first)].revisions[size_t(index.second)] == &r) {
                            owner = _StepName(step); break;
                        }
                    }
                    return _Bad(owner + ": chunk key differs from its partition layout");
                }
            }
        }
        return true;
    }

    bool _Binding(const fb::RigExecWireRevisionBinding &b,
                  const std::string &row)
    {
        // bind_coords is rigExec:bindCoordinates' own target, which an
        // authoring error can aim at a prim: every evaluator then reads no
        // coordinates there, and so does the runtime.
        for (const uint32_t *id :
             {&b.moverPath, &b.target, &b.transform, &b.transformSpace,
              &b.weightObject, &b.driverFrames, &b.bindCoords}) {
            if (!_PathId(*id, _AnyPath, row, "path role")) {
                return false;
            }
        }
        // The roles the binders build as attribute paths.
        const std::pair<uint32_t, const char *> attributes[] = {
            {b.base, "base"},
            {b.topologyCounts, "topology_counts"},
            {b.topologyIndices, "topology_indices"},
            {b.cagePoints, "cage_points"},
            {b.surfacePoints, "surface_points"},
            {b.driverCurvePoints, "driver_curve_points"},
            {b.driverCurveOrder, "driver_curve_order"},
            {b.driverCurveKnots, "driver_curve_knots"},
            {b.widths, "widths"}};
        for (const auto &[id, field] : attributes) {
            if (!_PathId(id, _Property | _Zero, row, field)) {
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
                !_ReadPtr(sample.activationRead, int(InputTag::Float),
                          _Resolved, at, "activation_read")) {
                return false;
            }
            // A dense sample's points read through its slots.
            if (sample.pointsRead) {
                if (sample.blendShape != 0) {
                    return _Bad(at + ".points_read: on a sample with a blend "
                                     "shape");
                }
                if (!_ReadPtr(sample.pointsRead, int(InputTag::Vec3fArray),
                              _Resolved | _Raw, at, "points_read")) {
                    return false;
                }
                const std::vector<uint32_t> &walk = sample.pointsRead->walk;
                if (walk.empty() ||
                    _f.inputs[walk[0]].name() != sample.pointsPath) {
                    return _Bad(at + ".points_read: a read headed by "
                                     "points_path expected");
                }
            }
            // Offsets pair with indices, or, with no indices, run over
            // every point (UsdSkelBlendShape's dense form). A layout the
            // accumulation applies indexes only the points it counts.
            const bool dense =
                sample.indices.empty() &&
                uint64_t(sample.offsets.size()) == sample.pointCount;
            if (!dense && !_Size(sample.offsets.size(),
                                 sample.indices.size(), at, "offsets")) {
                return false;
            }
            if (sample.layoutValid) {
                for (size_t k = 0; k < sample.indices.size(); ++k) {
                    const int32_t index = sample.indices[k];
                    if (index < 0 || uint64_t(index) >= sample.pointCount) {
                        return _Bad(_At(at, "indices", long(k)) + ": " +
                                    std::to_string(index) +
                                    " outside the layout's " +
                                    _N(sample.pointCount) + " points");
                    }
                }
            }
        }
        return true;
    }

    /// A skin layout in the form its flag names, validated exactly when the
    /// evaluator's layout rules pass. The sparse form: one index width and
    /// its vector, one counts vector, every count within element_size, the
    /// counts summing to the kept entries, no point whose last kept entry
    /// is (0, +0), which the sparse form drops, and no raw arrays.
    bool _Topology(const fb::RigExecWireSkinTopology &t,
                   const std::string &row)
    {
        if (t.raw) {
            return _RawTopology(t, row);
        }
        if (!t.rawIndices.empty() || !t.rawWeights.empty()) {
            return _Bad(row + ": raw_indices or raw_weights on a sparse "
                              "layout");
        }
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
        // The sparse form drops each row's trailing (0, +0) entries, so a
        // layout has one encoding of its kept entries.
        size_t end = 0;
        for (size_t p = 0; p < counted; ++p) {
            const size_t count =
                narrow ? size_t(t.counts8[p]) : size_t(t.counts16[p]);
            end += count;
            if (count > 0 && _TopologyIndex(t, end - 1) == 0 &&
                _IsPositiveZero(t.weights[end - 1])) {
                return _Bad(row + ": point " + _N(p) +
                            " keeps a trailing (0, +0) entry, which the "
                            "sparse form drops");
            }
        }
        return _Validated(t, row);
    }

    /// The raw form, for a layout the sparse form cannot hold: its arrays
    /// alone, with the point count the evaluators give it.
    bool _RawTopology(const fb::RigExecWireSkinTopology &t,
                      const std::string &row)
    {
        if (!t.counts8.empty() || !t.counts16.empty() ||
            !t.indices8.empty() || !t.indices16.empty() ||
            !t.indices32.empty() || !t.weights.empty() ||
            t.indexWidth != 0) {
            return _Bad(row + ": a raw layout with sparse vectors or an "
                              "index_width");
        }
        const size_t indices = t.rawIndices.size();
        const size_t weights = t.rawWeights.size();
        const std::string shape = row + ": a raw layout of " + _N(indices) +
                                  " indices and " + _N(weights) +
                                  " weights at element_size " +
                                  std::to_string(t.elementSize);
        if (_SparseCanHold(indices, weights, t.elementSize)) {
            return _Bad(shape + ", which the sparse form holds");
        }
        const uint64_t points = _RowCount(indices, weights, t.elementSize);
        if (t.pointCount != points) {
            return _Bad(row + ": point_count " +
                        std::to_string(t.pointCount) +
                        ", but its raw layout holds " +
                        std::to_string(points) + " point(s)");
        }
        return _Validated(t, row);
    }

    /// validated exactly when the evaluator's layout rules pass, in either
    /// form, so playback fails the layouts the evaluators fail and the
    /// kernels index only layouts they pass.
    bool _Validated(const fb::RigExecWireSkinTopology &t,
                    const std::string &row)
    {
        const std::string broken = _RuleBreak(t);
        if (t.validated && !broken.empty()) {
            return _Bad(row + ": validated" + broken);
        }
        if (!t.validated && broken.empty()) {
            return _Bad(row + ": not validated, but its layout passes the "
                              "evaluator's rules");
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
                !_Index(w.providerSlot, _slots, true, row, "provider_slot")) {
                return false;
            }
            // The painted arrays and the oracle's points, as input slots:
            // typed, with pool defaults; the painted ones are the object's
            // own attributes. The oracle's attribute is the one the stage
            // resolves its relationship to, which the file does not hold.
            const std::tuple<int32_t, InputTag, const char *, const char *>
                slots[] = {
                    {w.valuesSlot, InputTag::FloatArray, "values_slot",
                     "rigExec:values"},
                    {w.indicesSlot, InputTag::IntArray, "indices_slot",
                     "rigExec:indices"},
                    {w.oracleSamplesSlot, InputTag::Vec3fArray,
                     "oracle_samples_slot", nullptr},
                    {w.oracleCurveSlot, InputTag::Vec3fArray,
                     "oracle_curve_slot", nullptr}};
            for (const auto &[slot, tag, field, attribute] : slots) {
                if (!_InputSlotOf(slot, tag, row, field)) {
                    return false;
                }
                if (slot != -1 && attribute &&
                    !_SlotIs(slot, w.path, attribute)) {
                    return _Bad(_At(row, field) + ": slot " +
                                _N(size_t(slot)) + " is not the object's " +
                                attribute + " input");
                }
                if (slot != -1 &&
                    _DefaultOf(slot).arraySource != fb::ArraySource::Pool) {
                    return _Bad(_At(row, field) +
                                ": its default is not a pool array");
                }
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

    /// Path reads sorted strictly by (path, rest), each a static value, a
    /// live connection-following scalar read headed by its own attribute,
    /// or an array read headed by it: Raw or Resolved when live, Raw at
    /// rest, where it also holds the Default-time value of its type.
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
            if (read.read && RigExecFormatIsArrayTag(read.read->tag)) {
                if (!_ArrayPathRead(read, row)) {
                    return false;
                }
                continue;
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

    bool _ArrayPathRead(const fb::RigExecWirePathRead &read,
                        const std::string &row)
    {
        if (!_ReadPtr(read.read, _AnyArrayTag, _Resolved | _Raw, row,
                      "read")) {
            return false;
        }
        if (read.read->walk.empty() ||
            _f.inputs[read.read->walk[0]].name() != read.path) {
            return _Bad(row + ".read: an array read headed by its own "
                              "attribute expected");
        }
        if (read.headFallback) {
            return _Bad(row + ": head_fallback on an array read");
        }
        if (read.rest != bool(read.value)) {
            return _Bad(row + ": an array read holds a value exactly when "
                              "it reads at rest");
        }
        if (!read.rest) {
            return true;
        }
        if (read.read->mode != ReadMode::Raw) {
            return _Bad(row + ".read: an array read at rest is Raw");
        }
        if (!_PathValue(*read.value, row + ".value")) {
            return false;
        }
        if (read.value->tag != _PathArrayTag(read.read->tag)) {
            return _Bad(row + ".value: tag " + _N(size_t(read.value->tag)) +
                        " is not the read's " + _TagName(read.read->tag));
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

    // ---------------------------------------------------- phase bindings

    /// What a phased read was bound to at bake, against the steps that
    /// evaluate and read it: frame records and their FrameMatrix steps, the
    /// record lists an AtPrim transform phase folds, and the chain versions
    /// a phased point read or blend sample reads. A reader declares what
    /// it reads, so the step graph's producer check orders it after the
    /// writer. A record's PoseFin version is ordered here instead: the step
    /// graph sees one PoseFin slot per provider, which all of the
    /// provider's versions share.
    bool _PhaseBindings()
    {
        return _PhaseSteps() && _FrameRecords() && _RevisionPhases();
    }

    /// Where a revision's phase tables sit: revision \p index of chain
    /// \p chain, or the revision of its derived target \p index.
    struct _RevisionRow {
        size_t chain = 0;
        size_t index = 0;
        bool derived = false;
    };

    std::string _RowText(const _RevisionRow &row) const
    {
        return "geometry.chains[" + _N(row.chain) +
               (row.derived ? "].derived[" + _N(row.index) + "].revision"
                            : "].revisions[" + _N(row.index) + "]");
    }

    /// "step N (<label>)" when \p step names a step, else \p row's text.
    std::string _ReaderName(int64_t step, const _RevisionRow &row) const
    {
        return step >= 0 ? _StepName(size_t(step)) : _RowText(row);
    }

    /// The path of provider slot \p slot, which is in range.
    std::string _SlotText(size_t slot) const
    {
        return RigExecFormatPathText(_f, _f.slotMeta->paths[slot]);
    }

    /// Whether step \p step declares a read of slot \p slot of \p domain.
    bool _Declares(size_t step, fb::SlotDomain domain, uint64_t slot) const
    {
        for (const fb::SlotRange &range : _f.steps[step].reads) {
            if (range.domain() == domain && range.begin() <= slot &&
                slot < range.end()) {
                return true;
            }
        }
        return false;
    }

    /// The steps the phase tables name, met in playback order (the source
    /// pass, then every other step, each in index order): each frame
    /// record's one FrameMatrix step, which writes that record's slot of
    /// FrameMatrix and nothing else, while no other step writes the domain;
    /// each commit's first and last step; and the first InfluenceFold and
    /// RevisionStatic step of each revision and Derived step of each
    /// derived target.
    bool _PhaseSteps()
    {
        const fb::RigExecWireDomainPose &p = *_f.pose;
        const size_t records = p.frameRecords.size();
        _played.clear();
        _played.reserve(_steps);
        for (const bool sourcePass : {true, false}) {
            for (size_t s = 0; s < _steps; ++s) {
                if (bool(_f.steps[s].isSource) == sourcePass) {
                    _played.push_back(s);
                }
            }
        }
        _playAt.assign(_steps, 0);
        for (size_t k = 0; k < _played.size(); ++k) {
            _playAt[_played[k]] = k;
        }
        _recordStep.assign(records, -1);
        _commitFirst.assign(p.commits.size(), -1);
        _commitLast.assign(p.commits.size(), -1);
        _foldStep.assign(_revisions, -1);
        _staticStep.assign(_revisions, -1);
        _derivedStep.assign(_derived, -1);
        for (const size_t i : _played) {
            const fb::RigExecWireStep &step = _f.steps[i];
            if (step.kind == fb::StepKind::FrameMatrix) {
                // A FrameMatrix step has no "no record" object: a negative
                // one is out of range like a too-large one.
                if (!_Has(p.frameRecords, step.object)) {
                    return _Bad(_StepName(i) + " names frame record " +
                                std::to_string(step.object) + " of " +
                                _N(records));
                }
                const size_t o = size_t(step.object);
                if (_recordStep[o] >= 0) {
                    return _Bad(_StepName(i) +
                                " evaluates the same frame record as " +
                                _StepName(size_t(_recordStep[o])));
                }
                _recordStep[o] = int64_t(i);
                if (step.writes.size() != 1 ||
                    step.writes[0].domain() != fb::SlotDomain::FrameMatrix ||
                    step.writes[0].begin() != o ||
                    step.writes[0].end() != o + 1) {
                    return _Bad(_StepName(i) + " writes other than "
                                               "FrameMatrix[" +
                                _N(o) + "]");
                }
                continue;
            }
            for (const fb::SlotRange &range : step.writes) {
                if (range.domain() == fb::SlotDomain::FrameMatrix) {
                    return _Bad(_StepName(i) +
                                " writes FrameMatrix, which only a "
                                "FrameMatrix step writes");
                }
            }
            const auto first = [i](std::vector<int64_t> *table,
                                   int32_t object) {
                if (_Has(*table, object) && (*table)[size_t(object)] < 0) {
                    (*table)[size_t(object)] = int64_t(i);
                }
            };
            switch (step.kind) {
            case fb::StepKind::SolverCommit:
            case fb::StepKind::Constraint:
            case fb::StepKind::CommitDelta:
            case fb::StepKind::PropagateChunk:
            case fb::StepKind::CommitApply:
                if (_Has(_commitLast, step.object)) {
                    first(&_commitFirst, step.object);
                    _commitLast[size_t(step.object)] = int64_t(i);
                }
                break;
            case fb::StepKind::InfluenceFold:
                first(&_foldStep, step.object);
                break;
            case fb::StepKind::RevisionStatic:
                first(&_staticStep, step.object);
                break;
            case fb::StepKind::Derived:
                first(&_derivedStep, step.object);
                break;
            default:
                break;
            }
        }
        return true;
    }

    /// Each frame record: in range; a constraint's names a target and
    /// position -1, a solver's target -1 and the position of its provider
    /// in the commit's slots; evaluated by its FrameMatrix step, which reads
    /// the provider's PoseFin slot and the commit's table and runs after
    /// the step that writes the record's version and after the commit's
    /// first step, which writes the table. As the program's own check, a
    /// version below the slot count is that slot's compose and every other
    /// one a commit's write-back, written by the commit's last step.
    bool _FrameRecords()
    {
        const fb::RigExecWireDomainPose &p = *_f.pose;
        if (p.frameRecords.empty()) {
            return true;
        }
        for (size_t r = 0; r < p.frameRecords.size(); ++r) {
            const fb::FrameRecord &record = p.frameRecords[r];
            if (_recordStep[r] < 0) {
                // No step to name: the record's own row, then the missing
                // step.
                const std::string row = "pose.frame_records[" + _N(r) + "]";
                if (!_Index(int64_t(record.slot()), _slots, false, row,
                            "slot") ||
                    !_Index(int64_t(record.commit()), p.commits.size(), false,
                            row, "commit") ||
                    !_Index(int64_t(record.version()), size_t(_finPool),
                            false, row, "version") ||
                    !_PathId(record.moverLabel(), _AnyPath, row,
                             "mover_label")) {
                    return false;
                }
                return _Bad("frame record " + _N(r) + " of " +
                            _SlotText(record.slot()) + " after " +
                            RigExecFormatPathText(_f, record.moverLabel()) +
                            " has no FrameMatrix step");
            }
            // Named only on a violation: this runs over every record at
            // each Open.
            const size_t reader = size_t(_recordStep[r]);
            const auto step = [this, reader] { return _StepName(reader); };
            if (record.slot() >= _slots) {
                return _Bad(step() + " names slot " + _N(record.slot()) +
                            " of " + _N(_slots));
            }
            if (record.commit() >= p.commits.size()) {
                return _Bad(step() + " names commit " + _N(record.commit()) +
                            " of " + _N(p.commits.size()));
            }
            if (uint64_t(record.version()) >= _finPool) {
                return _Bad(step() + " is bound to PoseFin version " +
                            _N(record.version()) + ", which no step writes");
            }
            // Every other id names a prim, property or token (_Paths), so
            // the range is the whole of the label's rule.
            if (record.moverLabel() >= _f.paths.size()) {
                return _Bad(step() + " names mover label path id " +
                            _N(record.moverLabel()) + " of " +
                            _N(_f.paths.size()));
            }
            if (!p.commits[record.commit()].solverOutput) {
                if (record.target() < 0 || record.position() != -1) {
                    return _Bad(step() + " records commit " +
                                _N(record.commit()) +
                                ", a constraint's, at target " +
                                std::to_string(record.target()) +
                                " and position " +
                                std::to_string(record.position()) +
                                "; a constraint's record has a target and "
                                "position -1");
                }
            } else if (record.target() != -1) {
                return _Bad(step() + " records commit " + _N(record.commit()) +
                            ", a solver's, at target " +
                            std::to_string(record.target()) +
                            "; a solver's record has target -1");
            }
        }

        // Which step writes each PoseFin version, and of which slot. One
        // writer per version.
        std::vector<int64_t> writer(size_t(_finPool), -1);
        std::vector<int64_t> slotOf(size_t(_finPool), -1);
        const auto write = [&](uint64_t entry, size_t step, int64_t slot) {
            if (writer[entry] >= 0) {
                return _Bad(_StepName(step) + " writes PoseFin version " +
                            _N(entry) + ", which " +
                            _StepName(size_t(writer[entry])) + " writes too");
            }
            writer[entry] = int64_t(step);
            slotOf[entry] = slot;
            return true;
        };
        for (const size_t i : _played) {
            const fb::RigExecWireStep &step = _f.steps[i];
            if (step.kind != fb::StepKind::ComposeSubtree ||
                !_Has(p.composeGroups, step.object)) {
                continue;
            }
            const fb::RigExecWireComposeGroup &group =
                p.composeGroups[size_t(step.object)];
            for (int32_t slot = group.begin; slot < group.end; ++slot) {
                if (!write(uint64_t(slot), i, slot)) {
                    return false;
                }
            }
        }
        for (size_t w = 0; w < p.commits.size(); ++w) {
            const fb::RigExecWireCommit &commit = p.commits[w];
            if (_commitLast[w] < 0) {
                continue;
            }
            const size_t last = size_t(_commitLast[w]);
            // Each entry is inside the pool (_Commits).
            for (size_t k = 0; k < commit.slots.size(); ++k) {
                if (!write(commit.slotWrites[k], last, commit.slots[k])) {
                    return false;
                }
            }
            for (size_t k = 0; k < commit.propagate.size(); ++k) {
                if (!write(commit.descendantWrites[k], last,
                           commit.propagate[k].first)) {
                    return false;
                }
            }
        }

        for (size_t r = 0; r < p.frameRecords.size(); ++r) {
            const fb::FrameRecord &record = p.frameRecords[r];
            const size_t reader = size_t(_recordStep[r]);
            const auto step = [this, reader] { return _StepName(reader); };
            const uint32_t version = record.version();
            const int64_t by = writer[version];
            if (by < 0 || _playAt[size_t(by)] >= _playAt[reader]) {
                std::string line =
                    step() + " is bound to PoseFin version " + _N(version);
                if (slotOf[version] >= 0) {
                    line += " of " + _SlotText(size_t(slotOf[version]));
                }
                return _Bad(line + (by < 0 ? std::string(", which no step "
                                                         "writes")
                                           : ", which " +
                                                 _StepName(size_t(by)) +
                                                 " writes at or after it"));
            }
            if (slotOf[version] != int64_t(record.slot())) {
                return _Bad(step() + " is bound to PoseFin version " +
                            _N(version) + " of " +
                            _SlotText(size_t(slotOf[version])) +
                            ", not of the provider it records");
            }
            const fb::RigExecWireCommit &commit = p.commits[record.commit()];
            if (commit.solverOutput &&
                (!_Has(commit.slots, record.position()) ||
                 commit.slots[size_t(record.position())] !=
                     int32_t(record.slot()))) {
                return _Bad(step() + " reads position " +
                            std::to_string(record.position()) +
                            " of commit " + _N(record.commit()) +
                            ", which is not the provider it records");
            }
            const int64_t head = _commitFirst[record.commit()];
            if (head < 0 || _playAt[size_t(head)] >= _playAt[reader]) {
                return _Bad(step() + " reads the exit of commit " +
                            _N(record.commit()) + ", which " +
                            (head < 0 ? std::string("no step writes")
                                      : _StepName(size_t(head)) +
                                            " writes at or after it"));
            }
            if (!_Declares(reader, fb::SlotDomain::PoseFin, record.slot())) {
                return _Bad(step() + " does not declare PoseFin[" +
                            _N(record.slot()) + "]");
            }
            if (!_Declares(reader, fb::SlotDomain::CommitTable,
                           record.commit())) {
                return _Bad(step() + " does not declare CommitTable[" +
                            _N(record.commit()) + "]");
            }
        }
        return true;
    }

    /// The phase tables of every revision, main ones then derived ones,
    /// chain-major as their ids; then that every step reading them
    /// declares what it reads: an InfluenceFold its revision's frame
    /// records, a RevisionStatic its revision's point versions, and a
    /// Derived step both.
    bool _RevisionPhases()
    {
        const fb::RigExecWireDomainGeometry &g = *_f.geometry;
        size_t id = 0;
        for (size_t c = 0; c < _chains; ++c) {
            const fb::RigExecWireChain &chain = g.chains[c];
            for (size_t r = 0; r < chain.revisions.size(); ++r, ++id) {
                if (!_RevisionTables(chain.revisions[r], _foldStep[id],
                                     _staticStep[id], "InfluenceFold",
                                     "RevisionStatic", {c, r, false})) {
                    return false;
                }
            }
        }
        id = 0;
        for (size_t c = 0; c < _chains; ++c) {
            const fb::RigExecWireChain &chain = g.chains[c];
            for (size_t d = 0; d < chain.derived.size(); ++d, ++id) {
                if (!_RevisionTables(*chain.derived[d].revision,
                                     _derivedStep[id], _derivedStep[id],
                                     "Derived", "Derived", {c, d, true})) {
                    return false;
                }
            }
        }
        for (size_t i = 0; i < _steps; ++i) {
            const fb::RigExecWireStep &step = _f.steps[i];
            const bool fold = step.kind == fb::StepKind::InfluenceFold;
            const bool reads = step.kind == fb::StepKind::RevisionStatic;
            const bool derived = step.kind == fb::StepKind::Derived;
            const fb::RigExecWireRevision *revision = nullptr;
            if ((fold || reads) && _Has(g.revisionIndex, step.object)) {
                const RigExecWireIntPair &at =
                    g.revisionIndex[size_t(step.object)];
                revision =
                    &g.chains[size_t(at.first)].revisions[size_t(at.second)];
            } else if (derived && _Has(g.derivedIndex, step.object)) {
                const RigExecWireIntPair &at =
                    g.derivedIndex[size_t(step.object)];
                revision = g.chains[size_t(at.first)]
                               .derived[size_t(at.second)]
                               .revision.get();
            }
            if (!revision) {
                continue;
            }
            if ((fold || derived) && !_DeclaresRecords(i, *revision)) {
                return false;
            }
            if ((reads || derived) && !_DeclaresVersions(i, *revision)) {
                return false;
            }
        }
        return true;
    }

    /// One revision's frame-record lists and point bindings: the lists only
    /// under an AtPrim transform phase, the influence lists one per
    /// influence slot, every record in range and of the provider it stands
    /// for; one point binding per phased input, naming that input and its
    /// phase; a binding on exactly the dense blend samples with a non-base
    /// phase, naming the sample's points and phase; every candidate in
    /// range. \p fold and \p reader are the first steps of \p foldKind and
    /// \p readerKind that read them, which name the refusals.
    bool _RevisionTables(const fb::RigExecWireRevision &r, int64_t fold,
                         int64_t reader, const char *foldKind,
                         const char *readerKind, const _RevisionRow &row)
    {
        const fb::RigExecWireDomainPose &p = *_f.pose;
        const fb::RigExecWireRevisionBinding &b = *r.binding;
        // The names below are composed only for a violation: this runs over
        // every revision, binding and blend sample at each Open.
        const auto folder = [&] { return _ReaderName(fold, row); };
        const bool records =
            !r.transformRecords.empty() || !r.influenceRecords.empty();
        if (records &&
            b.transformPhase.kind != uint8_t(fb::ReadPhaseKind::AtPrim)) {
            return _Bad(folder() + " holds frame records for a transform "
                                   "phase that is not AtPrim");
        }
        if (!r.influenceRecords.empty() &&
            r.influenceRecords.size() != r.influenceSlots.size()) {
            return _Bad(folder() + " holds " + _N(r.influenceRecords.size()) +
                        " influence record lists for " +
                        _N(r.influenceSlots.size()) + " influence slots");
        }
        // Record \p k of the transform's list (\p influence -1) or of
        // influence \p influence's, whose provider is slot \p slot (-1:
        // none).
        const auto listed = [&](uint32_t k, int64_t slot, int64_t influence) {
            if (k >= p.frameRecords.size()) {
                return _Bad(folder() + " reads frame record " + _N(k) +
                            " of " + _N(p.frameRecords.size()));
            }
            const uint32_t provider = p.frameRecords[k].slot();
            if (int64_t(provider) != slot) {
                return _Bad(folder() + " reads frame record " + _N(k) +
                            " of " + _SlotText(provider) + " for " +
                            (influence < 0
                                 ? std::string("its transform")
                                 : "influence " + _N(size_t(influence))) +
                            ", whose provider is " +
                            (slot >= 0 ? _SlotText(size_t(slot))
                                       : std::string("no slot")));
            }
            return true;
        };
        for (const uint32_t k : r.transformRecords) {
            if (!listed(k, r.transformSlot, -1)) {
                return false;
            }
        }
        for (size_t i = 0; i < r.influenceRecords.size(); ++i) {
            for (const uint32_t k : r.influenceRecords[i].v) {
                if (!listed(k, r.influenceSlots[i], int64_t(i))) {
                    return false;
                }
            }
        }
        if (records && fold < 0) {
            return _Bad(_RowText(row) + ": frame records with no " +
                        std::string(foldKind) + " step to read them");
        }

        const auto who = [&] { return _ReaderName(reader, row); };
        if (r.pointBindings.size() != b.phaseInputs.size()) {
            return _Bad(who() + " holds " + _N(r.pointBindings.size()) +
                        " point bindings for " + _N(b.phaseInputs.size()) +
                        " phased inputs");
        }
        bool bound = !r.pointBindings.empty();
        for (size_t i = 0; i < r.pointBindings.size(); ++i) {
            const RigExecWirePointsBinding &binding = r.pointBindings[i];
            const auto input = [&] {
                return RigExecFormatPathText(_f, binding.inputPath);
            };
            if (binding.inputPath != b.phaseInputs[i]) {
                return _Bad(who() + " binds " + input() +
                            " for phased input " + _N(i) + ", which is " +
                            RigExecFormatPathText(_f, b.phaseInputs[i]));
            }
            if (binding.phase.kind != b.phases[i].kind ||
                binding.phase.prim != b.phases[i].prim) {
                return _Bad(who() + " binds " + input() +
                            " at a phase other than the one it declares");
            }
            if (!_Candidates(binding, who, input)) {
                return false;
            }
        }
        for (size_t ch = 0; ch < r.blendChannels.size(); ++ch) {
            const fb::RigExecWireBlendChannel &channel = r.blendChannels[ch];
            for (size_t s = 0; s < channel.samples.size(); ++s) {
                const fb::RigExecWireBlendSample &sample = channel.samples[s];
                const auto at = [&] {
                    return "blend_channels[" + _N(ch) + "].samples[" +
                           _N(s) + "]";
                };
                const bool phased =
                    sample.blendShape == 0 &&
                    sample.phase.kind != uint8_t(fb::ReadPhaseKind::Base);
                if (!sample.pointBinding) {
                    if (phased) {
                        return _Bad(who() + " leaves " + at() + " unbound, "
                                    "though it reads phased points");
                    }
                    continue;
                }
                if (!phased) {
                    return _Bad(who() + " binds " + at() +
                                ", which reads no phased points");
                }
                const RigExecWirePointsBinding &binding = *sample.pointBinding;
                const auto input = [&] {
                    return RigExecFormatPathText(_f, binding.inputPath);
                };
                if (binding.inputPath != sample.pointsPath) {
                    return _Bad(who() + " binds " + at() + " to " + input() +
                                ", not to its points " +
                                RigExecFormatPathText(_f, sample.pointsPath));
                }
                if (binding.phase.kind != sample.phase.kind ||
                    binding.phase.prim != sample.phase.prim) {
                    return _Bad(who() + " binds " + at() +
                                " at a phase other than its own");
                }
                if (binding.diagnoseMiss) {
                    return _Bad(who() + " binds " + at() +
                                " with a miss diagnostic, which a blend "
                                "sample never emits");
                }
                if (!_Candidates(binding, who, input)) {
                    return false;
                }
                bound = true;
            }
        }
        if (bound && reader < 0) {
            return _Bad(_RowText(row) + ": point bindings with no " +
                        std::string(readerKind) + " step to read them");
        }
        return true;
    }

    /// Every candidate of \p binding a chain of the file and a version of
    /// it, 0 the authored base to the revision count; a final read one
    /// candidate, the chain's published points at its last version. \p who
    /// and \p input compose the reader's and the input's names for a
    /// refusal.
    template <class Who, class Input>
    bool _Candidates(const RigExecWirePointsBinding &binding, const Who &who,
                     const Input &input)
    {
        const auto &chains = _f.geometry->chains;
        for (const fb::PointVersion &candidate : binding.candidates) {
            const int32_t c = candidate.chain();
            const int32_t v = candidate.version();
            if (!_Has(chains, c)) {
                return _Bad(who() + " binds " + input() + " to chain " +
                            std::to_string(c) + " of " + _N(_chains));
            }
            const size_t last = chains[size_t(c)].revisions.size();
            if (v < 0 || uint64_t(v) > last) {
                return _Bad(who() + " binds " + input() + " to version " +
                            std::to_string(v) + " of chain " +
                            std::to_string(c) +
                            (v < 0 ? std::string(", which is no version")
                                   : ", past its last version " + _N(last)));
            }
        }
        if (!binding.finalRead) {
            return true;
        }
        if (binding.candidates.size() != 1) {
            return _Bad(who() + " binds " + input() + " as a final read of " +
                        _N(binding.candidates.size()) +
                        " candidates; a final read has one");
        }
        const fb::PointVersion &only = binding.candidates[0];
        const size_t last = chains[size_t(only.chain())].revisions.size();
        if (uint64_t(only.version()) != last) {
            return _Bad(who() + " binds " + input() +
                        " as a final read of version " +
                        std::to_string(only.version()) + " of chain " +
                        std::to_string(only.chain()) +
                        ", not its last version " + _N(last));
        }
        return true;
    }

    /// Step \p step folds \p r: it declares FrameMatrix for every record
    /// \p r's lists hold.
    bool _DeclaresRecords(size_t step, const fb::RigExecWireRevision &r)
    {
        const auto declared = [&](uint32_t k) {
            return _Declares(step, fb::SlotDomain::FrameMatrix, k) ||
                   _Bad(_StepName(step) + " does not declare FrameMatrix[" +
                        _N(k) + "]");
        };
        for (const uint32_t k : r.transformRecords) {
            if (!declared(k)) {
                return false;
            }
        }
        for (const fb::RigExecWireUintList &list : r.influenceRecords) {
            for (const uint32_t k : list.v) {
                if (!declared(k)) {
                    return false;
                }
            }
        }
        return true;
    }

    /// Step \p step reads \p r's point bindings: it declares every
    /// candidate's version -- the chain's published points (ChainPoints)
    /// for a final read, its base (ChainBase) for version 0, else the
    /// RevisionDone and ChainDirty slots of the revision whose fuse left
    /// that version.
    bool _DeclaresVersions(size_t step, const fb::RigExecWireRevision &r)
    {
        const auto declared = [&](const RigExecWirePointsBinding &binding) {
            const auto need = [&](fb::SlotDomain domain, uint64_t slot) {
                return _Declares(step, domain, slot) ||
                       _Bad(_StepName(step) + " does not declare " +
                            rigExecStepGraphDetail::DomainName(
                                uint8_t(domain)) +
                            "[" + _N(slot) + "] for " +
                            RigExecFormatPathText(_f, binding.inputPath));
            };
            for (const fb::PointVersion &candidate : binding.candidates) {
                const uint64_t c = uint64_t(candidate.chain());
                if (binding.finalRead) {
                    if (!need(fb::SlotDomain::ChainPoints, c)) {
                        return false;
                    }
                } else if (candidate.version() == 0) {
                    if (!need(fb::SlotDomain::ChainBase, c)) {
                        return false;
                    }
                } else {
                    const uint64_t revision =
                        uint64_t(_f.geometry->chainRevisionBegin[c]) +
                        uint64_t(candidate.version()) - 1;
                    if (!need(fb::SlotDomain::RevisionDone, revision) ||
                        !need(fb::SlotDomain::ChainDirty, revision)) {
                        return false;
                    }
                }
            }
            return true;
        };
        for (const RigExecWirePointsBinding &binding : r.pointBindings) {
            if (!declared(binding)) {
                return false;
            }
        }
        for (const fb::RigExecWireBlendChannel &channel : r.blendChannels) {
            for (const fb::RigExecWireBlendSample &sample : channel.samples) {
                if (sample.pointBinding && !declared(*sample.pointBinding)) {
                    return false;
                }
            }
        }
        return true;
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
            if (_f.inputs[chain.target].type() !=
                _ChainTargetTag(chain.valueType)) {
                return _Bad(row + ": target slot " + _N(chain.target) +
                            " has tag " +
                            _N(size_t(_f.inputs[chain.target].type())) +
                            ", not its value type's " +
                            _N(size_t(_ChainTargetTag(chain.valueType))));
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
            for (size_t j = 0; j < phased.hops.size(); ++j) {
                if (RigExecFormatIsArrayTag(
                        _f.inputs[phased.hops[j]].type())) {
                    return _Bad(_At(row, "hops", long(j)) + ": slot " +
                                _N(phased.hops[j]) + " is an array input");
                }
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
            if (!mover.v2FrameValid && !mover.v2Frame.empty()) {
                return _Bad(row + ".v2_frame: bytes of an assembly that "
                                  "failed (v2_frame_valid is false)");
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
    /// The PoseFin versions the commits size: version references are
    /// below it.
    uint64_t _finPool = 0;
    std::vector<uint32_t> _overrideUses;
    std::vector<std::pair<const RigExecWireInput *, std::string>> _readUses;
    /// Per path node: 0 for paths[0], else its parent's depth plus one.
    std::vector<uint32_t> _depth;
    /// The steps in playback order, and each step's position in it.
    std::vector<size_t> _played;
    std::vector<size_t> _playAt;
    /// Per frame record, commit, revision id and derived id: the step
    /// _PhaseSteps maps to it, or -1.
    std::vector<int64_t> _recordStep;
    std::vector<int64_t> _commitFirst;
    std::vector<int64_t> _commitLast;
    std::vector<int64_t> _foldStep;
    std::vector<int64_t> _staticStep;
    std::vector<int64_t> _derivedStep;
};

}  // namespace

bool
RigExecFormatValidate(const fb::RigExecWireFile &file, std::string *error)
{
    return _Validator(file).Run(error);
}

namespace {

/// Why a file of format \p version does not open. The previous version
/// holds no array inputs, which only an export from the rig's stage can
/// supply; any other version is a rebake.
std::string
_VersionRefusal(uint32_t version)
{
    static_assert(RigExecFormatVersion == 9,
                  "name what the previous format version lacks");
    return "unsupported .rigexec format version " + _N(version) +
           " (this reader reads " + _N(RigExecFormatVersion) + "); " +
           (version < RigExecFormatVersion ? "re-export: S3 head tier"
                                              : "rebake");
}

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
                return _Fail(error, _VersionRefusal(version));
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
        return _Fail(error, _VersionRefusal(root->formatVersion()));
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
        // "v4" names the FlatBuffer generation of the format, not
        // format_version.
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
#if defined(__cpp_exceptions) || defined(_CPPUNWIND)
    // Allocation failures (the aligned copy, UnPack, the validator) refuse
    // the file rather than escape.
    try {
        return _OpenAligned(bytes, size, file, error);
    } catch (const std::bad_alloc &) {
        file->reset();
        return _Fail(error, "cannot open .rigexec: out of memory (" +
                                _N(size) + " bytes)");
    }
#else
    // Built without exceptions (game engines), an allocation failure ends
    // the process as every other allocation there does.
    return _OpenAligned(bytes, size, file, error);
#endif
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

namespace {

/// RigExecBakedStepKindName.
const char *
_StepKindName(fb::StepKind kind)
{
    switch (kind) {
    case fb::StepKind::ComposeSubtree: return "ComposeSubtree";
    case fb::StepKind::Solve: return "Solve";
    case fb::StepKind::SolverCommit: return "SolverCommit";
    case fb::StepKind::Constraint: return "Constraint";
    case fb::StepKind::CommitDelta: return "CommitDelta";
    case fb::StepKind::PropagateChunk: return "PropagateChunk";
    case fb::StepKind::CommitApply: return "CommitApply";
    case fb::StepKind::ProviderMatrix: return "ProviderMatrix";
    case fb::StepKind::SnapshotFinals: return "SnapshotFinals";
    case fb::StepKind::PoseInterpolator: return "PoseInterpolator";
    case fb::StepKind::VolumePlacements: return "VolumePlacements";
    case fb::StepKind::WeightPacket: return "WeightPacket";
    case fb::StepKind::InfluenceFold: return "InfluenceFold";
    case fb::StepKind::RevisionStatic: return "RevisionStatic";
    case fb::StepKind::RevisionChunk: return "RevisionChunk";
    case fb::StepKind::RevisionFuse: return "RevisionFuse";
    case fb::StepKind::ChainStatus: return "ChainStatus";
    case fb::StepKind::Derived: return "Derived";
    case fb::StepKind::FrameMatrix: return "FrameMatrix";
    case fb::StepKind::PropertyRevision: return "PropertyRevision";
    case fb::StepKind::RestCompose: return "RestCompose";
    case fb::StepKind::LadderCompose: return "LadderCompose";
    case fb::StepKind::SkinTopology: return "SkinTopology";
    }
    return "unknown";
}

/// The text after the kind: the path of the object \p step works on, in
/// StepLabel's words. False when an index is past its table.
bool
_StepObject(const RigExecWireFile &file, const fb::RigExecWireStep &step,
            std::string *out)
{
    const auto text = [&file](uint32_t id) {
        return RigExecFormatPathText(file, id);
    };
    const fb::RigExecWireSlotMeta *meta = file.slotMeta.get();
    const fb::RigExecWireDomainPose *pose = file.pose.get();
    const fb::RigExecWireDomainGeometry *geometry = file.geometry.get();
    const int32_t object = step.object;
    switch (step.kind) {
    case fb::StepKind::ComposeSubtree: {
        if (!pose || !meta || !_Has(pose->composeGroups, object)) {
            return false;
        }
        const int32_t begin = pose->composeGroups[size_t(object)].begin;
        if (!_Has(meta->paths, begin)) {
            return false;
        }
        *out = text(meta->paths[size_t(begin)]);
        return true;
    }
    case fb::StepKind::Solve:
        if (!pose || !_Has(pose->solvers, object)) {
            return false;
        }
        *out = text(pose->solvers[size_t(object)].path);
        return true;
    case fb::StepKind::SolverCommit:
    case fb::StepKind::Constraint:
    case fb::StepKind::CommitDelta:
    case fb::StepKind::PropagateChunk:
    case fb::StepKind::CommitApply: {
        if (!pose || !_Has(pose->commits, object)) {
            return false;
        }
        // An empty mover path is a batch of commits with no one mover.
        const std::string mover =
            text(pose->commits[size_t(object)].moverPath);
        *out = mover.empty() ? "batch " + std::to_string(object) : mover;
        return true;
    }
    case fb::StepKind::ProviderMatrix:
        if (!meta || !_Has(meta->paths, object)) {
            return false;
        }
        *out = text(meta->paths[size_t(object)]) +
               (step.part ? " final" : " base");
        return true;
    case fb::StepKind::SnapshotFinals:
        *out = "every provider";
        return true;
    case fb::StepKind::PropertyRevision: {
        if (!_Has(file.propertyChains, object)) return false;
        const auto &chain = file.propertyChains[size_t(object)];
        if (step.part == 0) {
            if (chain.target >= file.inputs.size()) return false;
            *out = text(file.inputs[chain.target].name()) + " base";
        } else {
            if (!_Has(chain.revisions, int64_t(step.part) - 1)) return false;
            *out = text(chain.revisions[size_t(step.part - 1)].mover);
        }
        return true;
    }
    case fb::StepKind::RestCompose:
    case fb::StepKind::LadderCompose: {
        if (!pose || !meta || !_Has(pose->composeGroups, object)) return false;
        const int first = pose->composeGroups[size_t(object)].begin;
        if (!_Has(meta->paths, first)) return false;
        *out = text(meta->paths[size_t(first)]);
        return true;
    }
    case fb::StepKind::SkinTopology: {
        if (!geometry || object < 0) return false;
        const size_t layout = size_t(object);
        const bool derived = layout >= geometry->revisionIndex.size();
        if (derived && layout - geometry->revisionIndex.size() >= geometry->derivedIndex.size()) return false;
        const auto &index = derived ? geometry->derivedIndex[layout - geometry->revisionIndex.size()]
                                    : geometry->revisionIndex[layout];
        if (!_Has(geometry->chains, index.first)) return false;
        const auto &chain = geometry->chains[size_t(index.first)];
        if (derived) {
            if (!_Has(chain.derived, index.second) || !chain.derived[size_t(index.second)].revision) return false;
            *out = text(chain.derived[size_t(index.second)].revision->moverPath);
        } else {
            if (!_Has(chain.revisions, index.second)) return false;
            *out = text(chain.revisions[size_t(index.second)].moverPath);
        }
        return true;
    }
    case fb::StepKind::FrameMatrix: {
        if (!pose || !_Has(pose->frameRecords, object)) {
            *out = "record " + std::to_string(object);
            return true;
        }
        const fb::FrameRecord &record = pose->frameRecords[size_t(object)];
        *out = (meta && _Has(meta->paths, int64_t(record.slot()))
                    ? text(meta->paths[size_t(record.slot())])
                    : "slot " + std::to_string(record.slot())) +
               " after " + text(record.moverLabel());
        return true;
    }
    case fb::StepKind::PoseInterpolator:
        if (!pose || !_Has(pose->poseInterpolators, object)) {
            return false;
        }
        *out = text(pose->poseInterpolators[size_t(object)].path);
        return true;
    case fb::StepKind::VolumePlacements:
        // part 1 places one volume, at slot `object`; any other form has
        // no slot to name.
        if (step.part == 1 && meta && _Has(meta->paths, object)) {
            *out = text(meta->paths[size_t(object)]);
        } else {
            *out = "every volume weight";
        }
        return true;
    case fb::StepKind::WeightPacket:
        if (!geometry || !_Has(geometry->weightObjects, object)) {
            return false;
        }
        *out = text(geometry->weightObjects[size_t(object)].path);
        return true;
    case fb::StepKind::InfluenceFold:
    case fb::StepKind::RevisionStatic:
    case fb::StepKind::RevisionChunk:
    case fb::StepKind::RevisionFuse: {
        if (!geometry || !_Has(geometry->revisionIndex, object)) {
            return false;
        }
        const RigExecWireIntPair &entry =
            geometry->revisionIndex[size_t(object)];
        if (!_Has(geometry->chains, entry.first) ||
            !_Has(geometry->chains[size_t(entry.first)].revisions,
                  entry.second)) {
            return false;
        }
        *out = text(geometry->chains[size_t(entry.first)]
                        .revisions[size_t(entry.second)]
                        .moverPath);
        return true;
    }
    case fb::StepKind::ChainStatus:
        if (!geometry || !_Has(geometry->chains, object)) {
            return false;
        }
        *out = text(geometry->chains[size_t(object)].target);
        return true;
    case fb::StepKind::Derived: {
        if (!geometry || !_Has(geometry->derivedIndex, object)) {
            return false;
        }
        const RigExecWireIntPair &entry =
            geometry->derivedIndex[size_t(object)];
        if (!_Has(geometry->chains, entry.first) ||
            !_Has(geometry->chains[size_t(entry.first)].derived,
                  entry.second)) {
            return false;
        }
        *out = text(geometry->chains[size_t(entry.first)]
                        .derived[size_t(entry.second)]
                        .target);
        return true;
    }
    }
    out->clear();
    return true;
}

}  // namespace

std::string
RigExecFormatStepLabel(const fb::RigExecWireFile &file, size_t step)
{
    if (step >= file.steps.size()) {
        return std::to_string(step);
    }
    const fb::RigExecWireStep &wire = file.steps[step];
    std::string object;
    if (!_StepObject(file, wire, &object)) {
        object = std::to_string(wire.object);
    }
    return std::string(_StepKindName(wire.kind)) + " " + object;
}

bool
RigExecFormatSparseTopology(const std::vector<int32_t> &indices,
                            const std::vector<float> &weights,
                            int32_t elementSize, uint64_t pointCount,
                            uint64_t influenceCount, bool validated,
                            fb::RigExecWireSkinTopology *out,
                            std::string *error)
{
    if (!out) {
        return _Fail(error, "no skin topology to write into");
    }
    if (elementSize < 0 || elementSize > 65535) {
        return _Fail(error, "skin topology: element size " +
                                std::to_string(elementSize) +
                                " is outside [0, 65535]");
    }
    if (pointCount > uint64_t(FLATBUFFERS_MAX_BUFFER_SIZE)) {
        return _Fail(error, "skin topology: " + std::to_string(pointCount) +
                                " points do not fit a .rigexec");
    }
    const size_t width = size_t(elementSize);
    const bool rectangular =
        indices.size() == weights.size() &&
        (width == 0 ? indices.empty()
                    : indices.size() % width == 0 &&
                          uint64_t(indices.size() / width) == pointCount);
    if (!rectangular) {
        return _Fail(error, "skin topology: " + _N(indices.size()) +
                                " indices and " + _N(weights.size()) +
                                " weights are not " +
                                std::to_string(pointCount) + " points of " +
                                _N(width) + " entries");
    }
    fb::RigExecWireSkinTopology t;
    t.elementSize = elementSize;
    t.pointCount = pointCount;
    t.influenceCount = influenceCount;
    t.validated = validated;
    const bool narrow = width <= 255;
    const size_t points = size_t(pointCount);
    if (narrow) {
        t.counts8.reserve(points);
    } else {
        t.counts16.reserve(points);
    }
    std::vector<int32_t> kept;
    kept.reserve(indices.size());
    t.weights.reserve(weights.size());
    bool negative = false;
    int32_t largest = 0;
    for (size_t p = 0; p < points; ++p) {
        // The row up to its trailing (0, +0) run, which the padding
        // restores bit for bit.
        size_t count = width;
        while (count > 0 && indices[p * width + count - 1] == 0 &&
               _IsPositiveZero(weights[p * width + count - 1])) {
            --count;
        }
        for (size_t e = p * width; e < p * width + count; ++e) {
            kept.push_back(indices[e]);
            t.weights.push_back(weights[e]);
            negative = negative || indices[e] < 0;
            largest = std::max(largest, indices[e]);
        }
        if (narrow) {
            t.counts8.push_back(uint8_t(count));
        } else {
            t.counts16.push_back(uint16_t(count));
        }
    }
    if (negative || largest > 65535) {
        t.indexWidth = 4;
        t.indices32 = std::move(kept);
    } else if (largest > 255) {
        t.indexWidth = 2;
        t.indices16.reserve(kept.size());
        for (const int32_t index : kept) {
            t.indices16.push_back(uint16_t(index));
        }
    } else {
        t.indexWidth = 1;
        t.indices8.reserve(kept.size());
        for (const int32_t index : kept) {
            t.indices8.push_back(uint8_t(index));
        }
    }
    *out = std::move(t);
    return true;
}

bool
RigExecFormatTopology(const std::vector<int32_t> &indices,
                      const std::vector<float> &weights, int32_t elementSize,
                      uint64_t pointCount, uint64_t influenceCount,
                      bool validated, fb::RigExecWireSkinTopology *out,
                      std::string *error)
{
    if (_SparseCanHold(indices.size(), weights.size(), elementSize)) {
        return RigExecFormatSparseTopology(indices, weights, elementSize,
                                           pointCount, influenceCount,
                                           validated, out, error);
    }
    if (!out) {
        return _Fail(error, "no skin topology to write into");
    }
    const uint64_t points =
        _RowCount(indices.size(), weights.size(), elementSize);
    if (pointCount != points) {
        return _Fail(error, "skin topology: " + _N(indices.size()) +
                                " indices and " + _N(weights.size()) +
                                " weights at element size " +
                                std::to_string(elementSize) + " hold " +
                                std::to_string(points) + " point(s), not " +
                                std::to_string(pointCount));
    }
    // The validator holds validated to the evaluator's layout rules.
    fb::RigExecWireSkinTopology t;
    t.elementSize = elementSize;
    t.pointCount = pointCount;
    t.influenceCount = influenceCount;
    t.validated = validated;
    t.raw = true;
    t.rawIndices = indices;
    t.rawWeights = weights;
    *out = std::move(t);
    return true;
}

void
RigExecFormatExpandTopology(const fb::RigExecWireSkinTopology &topology,
                            std::vector<int32_t> *indices,
                            std::vector<float> *weights)
{
    if (!indices || !weights) {
        return;
    }
    if (topology.raw) {
        *indices = topology.rawIndices;
        *weights = topology.rawWeights;
        return;
    }
    // Bounded by the stored vectors, so a topology the validator refused
    // cannot read past them either.
    const size_t width =
        topology.elementSize > 0 ? size_t(topology.elementSize) : 0;
    const bool narrow = width <= 255;
    const size_t points =
        narrow ? topology.counts8.size() : topology.counts16.size();
    const size_t kept =
        std::min(topology.weights.size(), _TopologyIndexCount(topology));
    indices->assign(points * width, 0);
    weights->assign(points * width, 0.0f);
    size_t next = 0;
    for (size_t p = 0; p < points; ++p) {
        const size_t count = std::min<size_t>(
            narrow ? topology.counts8[p] : topology.counts16[p], width);
        for (size_t e = 0; e < count && next < kept; ++e, ++next) {
            (*indices)[p * width + e] = _TopologyIndex(topology, next);
            (*weights)[p * width + e] = topology.weights[next];
        }
    }
}

}  // namespace rigExec
