#include "rigExecBinary/transport.h"
// The .rigexec FlatBuffer: Open refuses, bounds, verifies, unpacks and
// validates; Write validates and packs; the validator holds every rule a
// file must satisfy that the file alone can decide.
#include "rigExecBinary/format.h"

#include "rigExecBinary/stepGraph.h"
#include "rigExecGraph/providerRecords.h"
#include "rigExecMath/affineFrameKernel.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iterator>
#include <new>
#include <map>
#include <set>
#include <string_view>
#include <tuple>
#include <unordered_set>
#include <unordered_map>
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
static_assert(fb::StepKind::MAX == fb::StepKind::ProviderRefresh &&
                  fb::SlotDomain::MAX == fb::SlotDomain::RequiredStageFramesAdmission,
              "a step kind or slot domain was appended: give it its rules");
// The array tags and sources are frozen; the runtime mirrors the numbers.
static_assert(uint8_t(fb::InputTag::IntArray) == 8 &&
                  uint8_t(fb::InputTag::FloatArray) == 9 &&
                  uint8_t(fb::InputTag::DoubleArray) == 10 &&
                  uint8_t(fb::InputTag::Vec2fArray) == 11 &&
                  uint8_t(fb::InputTag::Vec3fArray) == 12 &&
                  fb::InputTag::MAX == fb::InputTag::Vec3i,
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
               _PhaseBindings() && _PropertyChains() && _CrossDomainTables() && _ConsumerLeafSites() && _WeightFieldTables() && _ProviderTables() && _PoseBodyDeclarations() && _HeadChecks() && _StepGraph() && _SolverSemanticRequirements() && _RequiredStageFrames() &&
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
    // File-owned proof index: no cross-file cache and no source queries.
    void _IndexSourceBackedOwners()
    {
        if(_sourceBackedIndexed)return;
        _sourceBackedIndexed=true;
        if(!_f.pose || !_f.slotMeta)return;
        using Key=std::tuple<uint8_t,uint32_t,int32_t,uint32_t>;
        struct Owners {std::set<size_t> rests,signs,constraints;};
        std::map<Key,Owners> sites;
        const auto direct=[&](const RigExecWireInput *read) {
            return read && read->flags==_Flags(fb::InputReadFlags::SourceBacked) && read->mode==ReadMode::Baked &&
                read->sampleTime==0 && read->selected==0 && read->walk.size()==1 &&
                read->walk[0]<_f.inputs.size() && _f.inputs[read->walk[0]].type()==read->tag &&
                (read->tag==InputTag::Double || read->tag==InputTag::Vec3d) &&
                read->propertyCandidates.empty() && read->doubleCandidates.empty() && read->rawFallbackSlot==-1;
        };
        const auto key=[](const RigExecWireInput &read) {
            return Key{uint8_t(read.tag),read.constant,read.overrideIndex,read.walk[0]};
        };
        const char *names[]={"rest:tx","rest:ty","rest:tz","rest:rx","rest:ry","rest:rz"};
        for(size_t slot=0;slot<_f.pose->ladders.size() && slot<_f.slotMeta->paths.size();++slot) {
            const uint32_t path=_f.slotMeta->paths[slot];
            if(path>=_f.paths.size())continue;
            for(size_t k=0;k<_f.pose->ladders[slot].restAvars.size() && k<6;++k) {
                const auto *body=&_f.pose->ladders[slot].restAvars[k];
                if(direct(body) && body->tag==InputTag::Double && _SlotIs(int32_t(body->walk[0]),path,names[k])) {
                    _sourceBackedOwners.insert(body);sites[key(*body)].rests.insert(slot);
                }
            }
            const auto *sign=_f.pose->ladders[slot].rotationSign.get();
            if(direct(sign) && sign->tag==InputTag::Vec3d &&
               _SlotIs(int32_t(sign->walk[0]),path,"avars:rotationSign") &&
               sign->constant<_f.values.size() && _f.values[sign->constant].tag==InputTag::Vec3d &&
               _f.values[sign->constant].vec3d &&
               (*_f.values[sign->constant].vec3d)[0]==1.0 &&
               (*_f.values[sign->constant].vec3d)[1]==1.0 &&
               (*_f.values[sign->constant].vec3d)[2]==1.0) {
                _sourceBackedOwners.insert(sign);sites[key(*sign)].signs.insert(slot);
            }
        }
        for(size_t index=0;index<_f.pose->constraints.size();++index) {
            const auto &constraint=_f.pose->constraints[index];
            const auto *body=constraint.offset.get();
            if(constraint.path<_f.paths.size() && direct(body) && body->tag==InputTag::Vec3d &&
               _SlotIs(int32_t(body->walk[0]),constraint.path,"inputs:translationOffset")) {
                _sourceBackedOwners.insert(body);sites[key(*body)].constraints.insert(index);
            }
        }
        for(const auto &step:_f.steps) {
            if(step.object<0)continue;
            for(const auto &read:step.headInputReads) {
                if(!direct(&read))continue;
                const auto found=sites.find(key(read));if(found==sites.end())continue;
                bool eligible=false;
                if(step.kind==fb::StepKind::RestCompose && size_t(step.object)<_f.pose->composeGroups.size()) {
                    const auto &group=_f.pose->composeGroups[size_t(step.object)];
                    if(group.begin>=0 && group.end>=group.begin) {
                        const auto at=found->second.rests.lower_bound(size_t(group.begin));
                        eligible=at!=found->second.rests.end() && *at<size_t(group.end);
                    }
                } else if(step.kind==fb::StepKind::LadderCompose && size_t(step.object)<_f.pose->composeGroups.size()) {
                    const auto &group=_f.pose->composeGroups[size_t(step.object)];
                    if(group.begin>=0 && group.end>=group.begin) {
                        const auto at=found->second.signs.lower_bound(size_t(group.begin));
                        eligible=at!=found->second.signs.end() && *at<size_t(group.end);
                    }
                } else if(step.kind==fb::StepKind::Constraint && size_t(step.object)<_f.pose->walkSteps.size()) {
                    const auto &walk=_f.pose->walkSteps[size_t(step.object)];
                    eligible=!walk.solverBatch && walk.index>=0 && size_t(walk.index)<_f.pose->constraints.size() &&
                        found->second.constraints.count(size_t(walk.index))!=0;
                }
                if(eligible)_sourceBackedOwners.insert(&read);
            }
        }
    }

    bool _Read(const RigExecWireInput *input, int tag, unsigned modes,
               const std::string &row, const char *field, long index = -1)
    {
        const auto where = [&] { return _At(row, field, index); };
        if (!input) {
            return _Bad(where() + ": missing");
        }
        if (input->tag > InputTag::MAX || input->mode > ReadMode::MAX || input->sampleTime > 1 ||
            (input->flags & ~_Flags(fb::InputReadFlags::ANY)) != 0) {
            return _Bad(where() + ": tag, mode or flags out of range");
        }
        if((input->flags&_Flags(fb::InputReadFlags::SourceBacked))!=0 &&
           (input->flags!=_Flags(fb::InputReadFlags::SourceBacked) || input->mode!=ReadMode::Baked ||
            input->sampleTime!=0 || input->selected!=0 || input->walk.size()!=1 ||
            (input->tag!=InputTag::Double && input->tag!=InputTag::Vec3d) ||
            !input->propertyCandidates.empty() || !input->doubleCandidates.empty() || input->rawFallbackSlot!=-1))
            return _Bad(where()+": invalid source-backed direct read");
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
                if(candidate.poseWeight < -1 || (candidate.poseWeight>=0 &&
                    size_t(candidate.poseWeight)>=_f.pose->poseWeightPaths.size()) ||
                    candidate.crossDomain < -1 || (candidate.crossDomain>=0 &&
                    size_t(candidate.crossDomain)>=_f.crossDomainReads.size()))
                    return _Bad(where()+": candidate typed source index out of range");
                if(candidate.poseWeight>=0 && input->tag!=InputTag::Float)
                    return _Bad(where()+": pose weight candidate requires Float read");
                if(candidate.poseWeight>=0 && candidate.crossDomain>=0)
                    return _Bad(where()+": incompatible pose and cross-domain candidates");
                if(candidate.crossDomain>=0) {
                    const auto &cross=_f.crossDomainReads[size_t(candidate.crossDomain)];
                    if((cross.kind==fb::CrossDomainReadKind::PointElement && input->tag!=InputTag::Vec3f) ||
                       (cross.kind==fb::CrossDomainReadKind::Points && input->tag!=InputTag::Vec3fArray) ||
                       (cross.kind==fb::CrossDomainReadKind::PoseFrame && input->tag!=InputTag::Matrix4d))
                        return _Bad(where()+": cross-domain candidate type differs from read");
                }
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
        for(const auto &candidate:input->doubleCandidates) if(candidate.poseWeight>=0)
            return _Bad(where()+": Double traversal cannot use Float pose publication");
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
        const bool sourceBacked=(input->flags & _Flags(fb::InputReadFlags::SourceBacked))!=0;
        if(sourceBacked) {
            if(input->flags!=_Flags(fb::InputReadFlags::SourceBacked) || !baked || input->sampleTime!=0 ||
               input->selected!=0 || input->walk.size()!=1 ||
               (input->tag!=InputTag::Double && input->tag!=InputTag::Vec3d) ||
               !input->propertyCandidates.empty() || !input->doubleCandidates.empty() || input->rawFallbackSlot!=-1 ||
               _f.inputs[input->walk[0]].type()!=input->tag)
                return _Bad(where()+": invalid source-backed direct read");
            _IndexSourceBackedOwners();
            const bool owner=_sourceBackedOwners.count(input)!=0;
            if(!owner)return _Bad(where()+": source-backed read has no eligible numeric owner");
        }
        if (input->selected < -1 ||
            (input->selected >= 0 &&
             (size_t(input->selected) >= input->walk.size() || !baked ||
              (!varying && !sourceBacked) || longWay))) {
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
        if (!_Index(read->context, _f.pose->spaceCheckpoints.size(), true, where, "context")) return false;
        if (read->context >= 0) {
            if (read->anchor != -1 || !read->recompose.empty())
                return _Bad(where + ": checkpoint read also carries an inline version");
            return true;
        }
        if (!read->recompose.empty())
            return _Bad(where + ": inline recomposition requires a checkpoint operation");
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
        if (read.context != -1 || read.anchor != -1 || !read.recompose.empty()) {
            return _Bad(where + ": reads no slot, so its version is "
                                "{-1, []}");
        }
        return true;
    }

    /// Scheduling belongs to the serialized typed producer graph. Switch
    /// descriptor enumeration does not choose a provider version.
    bool _ResolvedAnchor(const fb::RigExecWireFrameVersion &,
                         const std::vector<int32_t> &, size_t,
                         const std::string &)
    {
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
        if (!RigExecFormatReads(_f.formatVersion)) {
            return _Bad("format_version is " + _N(_f.formatVersion) +
                        "; this reader reads " +
                        _N(RigExecFormatOldestReadable) + " through " +
                        _N(RigExecFormatVersion));
        }
        if (!_f.slotMeta || !_f.constants || !_f.clustering || !_f.cones ||
            !_f.pose || !_f.geometry || !_f.commonGraph) {
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
                bool(value.vec3f) != (value.tag == InputTag::Vec3f) ||
                bool(value.vec3i) != (value.tag == InputTag::Vec3i)) {
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
        case InputTag::Vec3dArray: return _f.vec3dArrays.size();
        case InputTag::Matrix4dArray: return _f.matrix4dArrays.size();
        case InputTag::TokenArray: return _f.tokenArrays.size();
        case InputTag::BoolArray: return _f.boolArrays.size();
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
               first(_f.vec3fArrays, "vec3f_arrays") &&
               first(_f.vec3dArrays, "vec3d_arrays") &&
               first(_f.matrix4dArrays, "matrix4d_arrays") &&
               first(_f.tokenArrays, "token_arrays") &&
               first(_f.boolArrays, "bool_arrays") &&
               std::all_of(_f.boolArrays.begin(),_f.boolArrays.end(),[](const auto &row) {
                   return std::all_of(row.v.begin(),row.v.end(),[](uint8_t value){return value<=1;});
               });
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
            if(slot.type()>=InputTag::Vec3dArray && i<_f.listedInputs)
                return _Bad(where+": provider-only values cannot be public");
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
            !_Size(m.providerActive.size(), _slots, row, "provider_active") ||
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
        if(!_Bools(m.providerActive,row,"provider_active"))return false;
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
        if (!_Size(m.publicationRoles.size(), _slots, row, "publication_roles") ||
            !_Indices(m.solverArrayElements, _f.pose->solvers.size(), false,
                      row, "solver_array_elements")) return false;
        std::vector<uint8_t> roles(_slots, 0);
        for (int slot : m.jointSlots) roles[size_t(slot)] |= uint8_t(1);
        for (int slot : m.controlSlots) roles[size_t(slot)] |= uint8_t(2);
        if (roles != m.publicationRoles)
            return _Bad(row + ": publication_roles differs from actual joint/control membership");
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
        if (step.part != 1 && step.part != 2) {
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
        const auto domain = step.part == 2 ? fb::SlotDomain::WeightFramesBase
                                           : fb::SlotDomain::WeightFrames;
        if (step.writes.size() != 1 ||
            step.writes[0].domain() != domain ||
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
        std::vector<int64_t> placer(_slots, -1), basePlacer(_slots, -1);
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
                if (!_VolumePlacement(i, step.part == 2 ? &basePlacer : &placer)) {
                    return false;
                }
                continue;
            }
            for (const fb::SlotRange &range : step.writes) {
                if (range.domain() == fb::SlotDomain::WeightFrames ||
                    range.domain() == fb::SlotDomain::WeightFramesBase) {
                    return _Bad(_StepName(i) +
                                " writes WeightFrames, which only a "
                                "VolumePlacements step writes");
                }
            }
        }
        const std::vector<uint8_t> &volumes = _f.constants->noScaleAvars;
        for (size_t slot = 0; slot < volumes.size(); ++slot) {
            if (volumes[slot] &&
                ((placer[slot] < 0 && !_ExcludedValue(fb::SlotDomain::WeightFrames,uint32_t(slot))) ||
                 (basePlacer[slot] < 0 && !_ExcludedValue(fb::SlotDomain::WeightFramesBase,uint32_t(slot))))) {
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
    bool _ExcludedValue(fb::SlotDomain domain,uint32_t slot) const
    {
        if(!_f.commonGraph) return false;
        for(uint64_t id:_f.commonGraph->excludedValues) {
            if(id>=_f.commonGraph->valueSpecs.size()) continue;
            const auto &spec=_f.commonGraph->valueSpecs[size_t(id)];
            if(spec.domain==uint32_t(domain) && spec.slot==slot) return true;
        }
        return false;
    }

    bool _CrossDomainTables()
    {
        for(size_t i=0;i<_f.crossDomainReads.size();++i) {
            const auto &read=_f.crossDomainReads[i];
            const std::string row="cross_domain_reads["+_N(i)+"]";
            if(!_Phase(read.phase,row,"phase")) return false;
            if(read.kind>fb::CrossDomainReadKind::Points ||
               !_PathId(read.consumer,_Property|_Prim,row,"consumer") ||
               !_PathId(read.source,_Property|_Prim,row,"source") ||
               !_PathId(read.reader,_Property|_Prim|_Zero,row,"reader"))
                return _Bad(row+": invalid kind/owning paths");
            if(read.kind==fb::CrossDomainReadKind::PointElement || read.kind==fb::CrossDomainReadKind::Points) {
                if(read.kind==fb::CrossDomainReadKind::PointElement && read.element<0)
                    return _Bad(row+": selected element is negative");
                for(const auto &candidate:read.points)
                    if(candidate.first<0 || size_t(candidate.first)>=_f.geometry->chains.size() ||
                       candidate.second<0 || size_t(candidate.second)>_f.geometry->chains[size_t(candidate.first)].revisions.size())
                        return _Bad(row+": points candidate exceeds actual chain versions");
                for(const auto &candidate:read.points) {
                    if(_f.geometry->chains[size_t(candidate.first)].target!=read.source)
                        return _Bad(row+": point candidate belongs to another source");
                    if(read.phase.kind==uint8_t(fb::ReadPhaseKind::Base) && candidate.second!=0)
                        return _Bad(row+": Base point candidate is not version zero");
                }
                if(read.finalPoints!=(read.phase.kind==uint8_t(fb::ReadPhaseKind::Final)))
                    return _Bad(row+": point final selection differs from phase");
                if(read.rawSlot < -1 || (read.rawSlot>=0 &&
                    (size_t(read.rawSlot)>=_f.inputs.size() || _f.inputs[size_t(read.rawSlot)].type()!=InputTag::Vec3fArray)))
                    return _Bad(row+": raw points fallback has incompatible slot");
            } else if(read.kind==fb::CrossDomainReadKind::PoseFrame) {
                if(read.provider<0 || size_t(read.provider)>=_slots)
                    return _Bad(row+": provider slot exceeds actual pose");
                if(RigExecFormatPathText(_f,read.source)!=RigExecFormatPathText(_f,_f.slotMeta->paths[size_t(read.provider)])+".posed:space")
                    return _Bad(row+": pose frame source belongs to another provider");
                if(read.baseFrame!=(read.phase.kind==uint8_t(fb::ReadPhaseKind::Base)))
                    return _Bad(row+": pose frame pool differs from phase");
                std::set<uint32_t> ownedFrames{uint32_t(read.provider)};
                const auto note=[&](const auto &values,size_t index) { if(index<values.size()) ownedFrames.insert(values[index]); };
                bool written=false;
                for(const auto &commit:_f.pose->commits) {
                    if(read.baseFrame && !commit.solverOutput) continue;
                    for(size_t k=0;k<commit.slots.size();++k) if(commit.slots[k]==read.provider) {
                        const auto &writes=read.baseFrame?commit.slotBaseWrites:commit.slotWrites;
                        const auto &carry=read.baseFrame?commit.slotBaseCarry:commit.slotCarry;
                        note(writes,k); note(carry,k); written=written || k<writes.size();
                    }
                    for(size_t k=0;k<commit.propagate.size();++k) if(commit.propagate[k].first==read.provider) {
                        const auto &writes=read.baseFrame?commit.descendantBaseWrites:commit.descendantWrites;
                        const auto &carry=read.baseFrame?commit.descendantBaseCarry:commit.descendantCarry;
                        note(writes,k); note(carry,k); written=written || k<writes.size();
                    }
                }
                for(const auto &refresh:_f.pose->providerRefreshes) {
                    if(refresh.slot==read.provider) {
                        ownedFrames.insert(read.baseFrame?refresh.baseWrite:refresh.finWrite);
                        ownedFrames.insert(read.baseFrame?refresh.baseRead:refresh.finRead);
                    }
                    for(const auto &carry:refresh.carries)if(carry.slot==read.provider) {
                        ownedFrames.insert(read.baseFrame?carry.baseWrite:carry.finWrite);
                        ownedFrames.insert(read.baseFrame?carry.baseRead:carry.finRead);
                    }
                }
                if(written) ownedFrames.insert(uint32_t(_slots)+uint32_t(read.provider));
                for(uint32_t version:read.frames) if(!ownedFrames.count(version))
                    return _Bad(row+": frame version belongs to another provider");
                for(uint32_t version:read.frames)
                    if(!_GraphValueSlot(uint32_t(read.baseFrame?fb::SlotDomain::PoseBase:fb::SlotDomain::PoseFin),version))
                        return _Bad(row+": frame candidate exceeds actual version storage");
            } else if(read.kind==fb::CrossDomainReadKind::SpaceValue) {
                if(read.spaceValue<0 || !_GraphValueSlot(uint32_t(fb::SlotDomain::SpaceValue),uint32_t(read.spaceValue)))
                    return _Bad(row+": provider expression value exceeds actual storage");
            } else if(read.kind==fb::CrossDomainReadKind::PropertyResult) {
                if(read.propertyChain<0 || size_t(read.propertyChain)>=_f.propertyChains.size())
                    return _Bad(row+": property candidate has no chain owner");
                const auto &chain=_f.propertyChains[size_t(read.propertyChain)];
                if(_f.inputs[chain.target].name()!=read.source)
                    return _Bad(row+": property source belongs to another chain");
                if(read.phase.kind==uint8_t(fb::ReadPhaseKind::Base) && read.propertyVersion!=chain.versionBase)
                    return _Bad(row+": Base property source is not version zero");
                if(read.phase.kind==uint8_t(fb::ReadPhaseKind::Final) && read.propertyVersion!=chain.versionBase+chain.revisions.size())
                    return _Bad(row+": Final property source is not final version");
                if(read.propertyVersion<chain.versionBase ||
                   uint64_t(read.propertyVersion)>uint64_t(chain.versionBase)+chain.revisions.size())
                    return _Bad(row+": property candidate version differs from actual owner");
            }
        }
        return true;
    }

    bool _RequireCandidateReads(size_t owner,const RigExecWireInput &input)
    {
        const auto require=[&](fb::SlotDomain domain,uint32_t slot) {
            return _BodyRead(owner,domain,slot) || _Bad(_StepName(owner)+": nested input omitted typed producer read");
        };
        for(const auto *segment:{&input.propertyCandidates,&input.doubleCandidates})
            for(const auto &hop:*segment) {
                if(hop.version>=0 && !require(fb::SlotDomain::PropertyResult,uint32_t(hop.version))) return false;
                if(hop.poseWeight>=0 && !require(fb::SlotDomain::PoseWeight,uint32_t(hop.poseWeight))) return false;
                if(hop.crossDomain<0) continue;
                const auto &cross=_f.crossDomainReads[size_t(hop.crossDomain)];
                if(cross.kind==fb::CrossDomainReadKind::PropertyResult) {
                    if(!require(fb::SlotDomain::PropertyResult,cross.propertyVersion)) return false;
                } else if(cross.kind==fb::CrossDomainReadKind::SpaceValue) {
                    if(!require(fb::SlotDomain::SpaceValue,uint32_t(cross.spaceValue))) return false;
                } else if(cross.kind==fb::CrossDomainReadKind::PoseFrame) {
                    for(uint32_t version:cross.frames) if(!require(cross.baseFrame?fb::SlotDomain::PoseBase:fb::SlotDomain::PoseFin,version)) return false;
                } else for(const auto &point:cross.points) {
                    if(cross.finalPoints) { if(!require(fb::SlotDomain::ChainPoints,uint32_t(point.first))) return false; }
                    else if(point.second==0) { if(!require(fb::SlotDomain::ChainBase,uint32_t(point.first))) return false; }
                    else {
                        const uint32_t revision=uint32_t(_f.geometry->chainRevisionBegin[size_t(point.first)])+uint32_t(point.second)-1;
                        if(!require(fb::SlotDomain::RevisionDone,revision) || !require(fb::SlotDomain::ChainDirty,revision)) return false;
                    }
                }
            }
        return true;
    }

    bool _SolverSemanticRequirements()
    {
        const auto &solvers=_f.pose->solvers;
        std::vector<int64_t> active(solvers.size(),-1);
        std::map<std::string,size_t> solverKeys;
        for(size_t i=0;i<solvers.size();++i) {
            const auto &solver=solvers[i];
            if(!solver.solveDescriptorKey.empty()) {
                const std::string prefix=RigExecFormatPathText(_f,solver.path)+"/category:"+std::to_string(int(fb::StepKind::Solve))+"/";
                if(solver.solveDescriptorKey.compare(0,prefix.size(),prefix)!=0 ||
                    !solverKeys.emplace(solver.solveDescriptorKey,i).second)
                    return _Bad("pose.solvers: invalid or duplicate canonical Solve identity");
            }
        }
        for(size_t step=0;step<_f.steps.size();++step) {
            const auto &body=_f.steps[step];
            if(body.kind!=fb::StepKind::Solve) {
                if(!body.semanticPredecessorKeys.empty())return _Bad(_StepName(step)+": semantic prerequisites on a non-Solve body");
                continue;
            }
            if(!_Has(solvers,body.object))return _Bad(_StepName(step)+": invalid semantic Solve owner");
            const size_t owner=size_t(body.object);
            if(active[owner]>=0)return _Bad(_StepName(step)+": duplicate semantic Solve owner");
            active[owner]=int64_t(step);
            if(!solvers[owner].solveDescriptorKey.empty() && body.descriptorKey!=solvers[owner].solveDescriptorKey)
                return _Bad(_StepName(step)+": canonical Solve identity differs from its solver");
        }
        std::unordered_map<std::string,size_t> commonKeys;
        commonKeys.reserve(_f.commonGraph->ops.size());
        for(size_t i=0;i<_f.commonGraph->ops.size();++i)
            if(!commonKeys.emplace(_f.commonGraph->ops[i].key,i).second)return _Bad("common_graph: duplicate semantic operation identity");
        for(size_t i=0;i<solvers.size();++i) {
            const auto &solver=solvers[i];
            const std::string type=RigExecFormatPathText(_f,solver.type);
            const auto supported=[&](const std::string &port) {
                if(type=="RigExecFkChain")return port=="rigExec:controls" || port=="rigExec:startFrame";
                if(type=="RigExecTwoBoneIk")return port=="rigExec:rootControl" || port=="rigExec:effectorControl" || port=="rigExec:poleControl" || port=="rigExec:space";
                if(type=="RigExecBlendPointFrames")return port=="rigExec:inputA" || port=="rigExec:inputB";
                if(type=="RigExecTwistDistribution")return port=="rigExec:start" || port=="rigExec:end";
                if(type=="RigExecRibbon")return port=="rigExec:driverCurve" || port=="rigExec:startFrame" ||
                    port=="rigExec:endFrame" || port=="rigExec:twistFrames";
                if(type=="RigExecSplineIk")return port=="rigExec:rootControl" || port=="rigExec:midControl" || port=="rigExec:endControl" || port=="rigExec:space";
                return false;
            };
            std::set<std::pair<std::string,int32_t>> facts;
            std::set<std::string> expected;
            std::set<std::string> singlePorts;
            for(const auto &requirement:solver.relationshipRequirements) {
                if(!supported(requirement.port) || !_Has(solvers,requirement.solver) ||
                    !facts.emplace(requirement.port,requirement.solver).second ||
                    (!(type=="RigExecFkChain" && requirement.port=="rigExec:controls") &&
                     !(type=="RigExecRibbon" && requirement.port!="rigExec:driverCurve") &&
                     !singlePorts.insert(requirement.port).second))
                    return _Bad("pose.solvers["+_N(i)+"]: invalid or duplicate solver relationship requirement");
                if(type=="RigExecBlendPointFrames" &&
                   ((requirement.port=="rigExec:inputA" && requirement.solver!=solver.inA) ||
                    (requirement.port=="rigExec:inputB" && requirement.solver!=solver.inB)))
                    return _Bad("pose.solvers["+_N(i)+"]: relationship target differs from its numerical solver binding");
                const size_t target=size_t(requirement.solver);
                const auto &key=solvers[target].solveDescriptorKey;
                if(key.empty())return _Bad("pose.solvers["+_N(i)+"]: relationship target has no canonical Solve identity");
                expected.insert(key);
                if(active[target]<0 && (!_ExcludedValue(fb::SlotDomain::Aggregate,uint32_t(target)) ||
                    !_ExcludedValue(fb::SlotDomain::Candidates,uint32_t(target))))
                    return _Bad("pose.solvers["+_N(i)+"]: relationship target has no Solve producer or typed exclusion");
            }
            if(type=="RigExecBlendPointFrames" &&
               ((solver.inA>=0 && !facts.count({"rigExec:inputA",solver.inA})) ||
                (solver.inB>=0 && !facts.count({"rigExec:inputB",solver.inB}))))
                return _Bad("pose.solvers["+_N(i)+"]: numerical solver binding has no relationship requirement");
            if(active[i]<0) {
                if(!expected.empty() && (!_ExcludedValue(fb::SlotDomain::Aggregate,uint32_t(i)) ||
                    !_ExcludedValue(fb::SlotDomain::Candidates,uint32_t(i))))
                    return _Bad("pose.solvers["+_N(i)+"]: required solver has no Solve producer or typed exclusion");
                continue;
            }
            const size_t step=size_t(active[i]);const auto &body=_f.steps[step];
            const std::set<std::string> actual(body.semanticPredecessorKeys.begin(),body.semanticPredecessorKeys.end());
            if(actual.size()!=body.semanticPredecessorKeys.size() || actual!=expected ||
               !std::is_sorted(body.semanticPredecessorKeys.begin(),body.semanticPredecessorKeys.end()))
                return _Bad(_StepName(step)+": semantic prerequisites differ from supported solver relationships");
            for(const auto &key:expected) {
                const size_t target=solverKeys.at(key);
                if(active[target]<0) {
                    // A surviving solver may read an excluded aggregate through
                    // its explicit typed fallback, exactly as runtime closure does.
                    if(!_ExcludedValue(fb::SlotDomain::Aggregate,uint32_t(target)) ||
                       !_ExcludedValue(fb::SlotDomain::Candidates,uint32_t(target)))
                        return _Bad(_StepName(step)+": excluded semantic prerequisite lacks typed fallback");
                    continue;
                }
                if(std::find(body.preds.begin(),body.preds.end(),int32_t(active[target]))==body.preds.end())
                    return _Bad(_StepName(step)+": generated graph omitted semantic solver prerequisite");
                const auto after=commonKeys.find(body.descriptorKey),before=commonKeys.find(key);
                if(after==commonKeys.end() || before==commonKeys.end() ||
                    std::find(_f.commonGraph->ops[after->second].predecessors.begin(),_f.commonGraph->ops[after->second].predecessors.end(),uint32_t(before->second))==_f.commonGraph->ops[after->second].predecessors.end())
                    return _Bad(_StepName(step)+": common graph omitted semantic solver prerequisite");
            }
        }
        // Reconstruct edges from typed producers plus declared relationships.
        // Serialized predecessor lists are outputs of this proof, never its inputs.
        if(!_f.commonGraph->ops.empty()) {
            const auto &ops=_f.commonGraph->ops;
            if(ops.size()!=_f.steps.size())return _Bad("common_graph: semantic projection body count differs");
            using Run=std::tuple<uint32_t,uint64_t,uint64_t>;
            const auto normalize=[](std::vector<Run> runs) {
                std::sort(runs.begin(),runs.end());std::vector<Run> result;
                for(const auto &run:runs) {
                    if(std::get<1>(run)==std::get<2>(run))continue;
                    if(!result.empty() && std::get<0>(result.back())==std::get<0>(run) && std::get<2>(result.back())>=std::get<1>(run))
                        std::get<2>(result.back())=std::max(std::get<2>(result.back()),std::get<2>(run));
                    else result.push_back(run);
                }
                return result;
            };
            std::unordered_map<uint64_t,size_t> producers;
            producers.reserve(_f.commonGraph->valueSpecs.size());
            std::unordered_map<std::string,size_t> bodyKeys;
            bodyKeys.reserve(_f.steps.size());
            for(size_t i=0;i<_f.steps.size();++i)
                if(!bodyKeys.emplace(_f.steps[i].descriptorKey,i).second)return _Bad("steps: duplicate semantic body identity");
            for(size_t i=0;i<ops.size();++i) {
                const auto &op=ops[i];const auto body=bodyKeys.find(op.key);
                if(body==bodyKeys.end() || op.kind!=uint32_t(_f.steps[body->second].kind))
                    return _Bad("common_graph: invalid semantic body or imported predecessor authority");
                const auto project=[&](const std::vector<uint64_t> &ids,std::vector<Run> *runs) {
                    for(uint64_t id:ids) {
                        if(id>=_f.commonGraph->valueSpecs.size())return false;
                        const auto &value=_f.commonGraph->valueSpecs[size_t(id)];
                        runs->emplace_back(value.domain,uint64_t(value.slot),uint64_t(value.slot)+1);
                    }
                    return true;
                };
                for(const auto *ids:{&op.reads,&op.writes}) {
                    std::vector<Run> actual,wanted;
                    if(!project(*ids,&actual))return _Bad("common_graph: semantic projection value out of range");
                    const auto &ranges=ids==&op.reads?_f.steps[body->second].reads:_f.steps[body->second].writes;
                    for(const auto &range:ranges)wanted.emplace_back(uint32_t(range.domain()),range.begin(),range.end());
                    if(normalize(std::move(actual))!=normalize(std::move(wanted)))return _Bad("common_graph: semantic typed projection differs from body");
                }
                for(uint64_t value:op.writes)
                    if(!producers.emplace(value,i).second)return _Bad("common_graph: duplicate semantic typed producer");
            }
            for(size_t i=0;i<ops.size();++i) {
                const auto &op=ops[i];const size_t body=bodyKeys.at(op.key);
                std::set<uint32_t> expected;
                for(uint64_t value:op.reads) {
                    const auto producer=producers.find(value);
                    if(producer!=producers.end())expected.insert(uint32_t(producer->second));
                }
                for(const auto &key:_f.steps[body].semanticPredecessorKeys) {
                    const auto producer=commonKeys.find(key);
                    if(producer==commonKeys.end()) {
                        const auto solver=solverKeys.find(key);
                        if(solver==solverKeys.end() ||
                           !_ExcludedValue(fb::SlotDomain::Aggregate,uint32_t(solver->second)) ||
                           !_ExcludedValue(fb::SlotDomain::Candidates,uint32_t(solver->second)))
                            return _Bad("common_graph: semantic prerequisite has no active producer or typed fallback");
                        continue;
                    }
                    expected.insert(uint32_t(producer->second));
                }
                std::set<uint32_t> described;
                for(uint32_t predecessor:op.descriptorPredecessors)
                    if(!expected.count(predecessor) || !described.insert(predecessor).second)
                        return _Bad("common_graph: imported predecessor is not a typed or semantic requirement");
                const std::set<uint32_t> actual(op.predecessors.begin(),op.predecessors.end());
                if(actual.size()!=op.predecessors.size() || actual!=expected)
                    return _Bad("common_graph: predecessors differ from typed and semantic producers");
                std::set<int32_t> bodyExpected;
                for(uint32_t before:expected)bodyExpected.insert(int32_t(bodyKeys.at(ops[before].key)));
                const auto &preds=_f.steps[body].preds;
                if(std::set<int32_t>(preds.begin(),preds.end())!=bodyExpected)
                    return _Bad(_StepName(body)+": predecessors differ from typed and semantic producers");
            }
        }
        return true;
    }

    bool _ConsumerLeafSites()
    {
        const auto validate = [&](const fb::RigExecWireExternalDeclaredInput &site,
                                  const std::string &row) {
            if (!_PathId(site.path, _Property, row, "path") || site.time > fb::ExternalInputTime::AtDefault || site.flavour > fb::ExternalInputFlavour::Present)
                return _Bad(row + ": invalid owning leaf descriptor");
            if (!_ReadPtr(site.read, site.read ? int(site.read->tag) : _AnyTag, _AnyMode, row, "read")) return false;
            if (site.allowFloatToDouble && site.read->tag!=RigExecWireInputTag::Double)
                return _Bad(row + ": float widening requires a Double transport tag");
            if (site.read->overrideIndex >= 0 || site.exactVersion < -1 ||
                site.exactRecord < -1 || site.exactValueType < -1 || site.exactValueType > 3)
                return _Bad(row + ": invalid owning leaf producer metadata");
            if (site.exactVersion >= 0 &&
                !_GraphValueSlot(uint32_t(fb::SlotDomain::PropertyResult),uint32_t(site.exactVersion)))
                return _Bad(row + ": exact producer version exceeds property storage");
            if (site.exactRecord >= 0) {
                if (size_t(site.exactRecord) >= _f.phasedConsumers.size() ||
                    int32_t(_f.phasedConsumers[size_t(site.exactRecord)].version) != site.exactVersion)
                    return _Bad(row + ": record does not own the exact producer version");
            }
            if (site.exactVersion >= 0) {
                int actual=-1;
                if (site.exactRecord >= 0)
                    actual=int(_f.phasedConsumers[size_t(site.exactRecord)].consumerType);
                else for (const auto &chain:_f.propertyChains)
                    if (uint32_t(site.exactVersion)>=chain.versionBase &&
                        uint64_t(site.exactVersion)<=uint64_t(chain.versionBase)+chain.revisions.size()) {
                        actual=int(chain.valueType); break;
                    }
                if (actual<0 || actual!=site.exactValueType)
                    return _Bad(row + ": exact producer arm differs from its actual owner");
            }
            if (site.exactVersion < 0 && (site.exactRecord >= 0 || site.exactValueType >= 0))
                return _Bad(row + ": typed producer metadata has no exact version");
            if (site.bodyWalk && ((site.exactVersion >= 0 &&
                (site.read->tag!=RigExecWireInputTag::Double || site.allowFloatToDouble)) ||
                !_ReadPtr(site.bodyWalk, int(site.read->tag), _AnyMode, row, "body_walk")))
                return _Bad(row + ": invalid owning bound walk");
            for (const auto *segment : {&site.read->propertyCandidates,&site.read->doubleCandidates})
                for (const auto &hop : *segment)
                    if (hop.kind != uint8_t(fb::PropertyCandidateKind::SlotOnly) ||
                        hop.version != -1 || hop.crossDomain >= 0 || hop.poseWeight >= 0)
                        return _Bad(row + ": sampled fallback contains a computed producer");
            _sampledLeafReads.insert(site.read.get());
            if(site.bodyWalk) _boundWalkReads.insert(site.bodyWalk.get());
            return true;
        };
        for (size_t c=0;c<_f.geometry->chains.size();++c) {
            const auto &chain=_f.geometry->chains[c];
            const auto check=[&](const fb::RigExecWireRevision &rev,const std::string &row,
                                 size_t revision,bool derived) {
                int ordinary=-1,layout=-1;
                const auto &indices=derived?_f.geometry->derivedIndex:_f.geometry->revisionIndex;
                for(size_t i=0;i<indices.size();++i)
                    if(indices[i].first==int32_t(c) && indices[i].second==int32_t(revision)) {
                        ordinary=int(i); layout=int(_revisions*(derived?1:0)+i); break;
                    }
                for (const auto *sites : {&rev.leafSites,&rev.layoutLeafSites}) {
                    const bool isLayout=sites==&rev.layoutLeafSites;
                    std::vector<size_t> owners;
                    for(size_t step=0;step<_f.steps.size();++step) {
                        const auto &body=_f.steps[step];
                        if(isLayout ? (body.kind==fb::StepKind::SkinTopology && body.object==layout)
                            : (body.kind==(derived?fb::StepKind::Derived:fb::StepKind::RevisionStatic) && body.object==ordinary))
                            owners.push_back(step);
                    }
                    for (size_t k=0;k<sites->size();++k) {
                        const auto &site=(*sites)[k];
                        if (!validate(site,row+".leaf_sites["+_N(k)+"]")) return false;
                        std::vector<int32_t> versions;
                        if(site.exactVersion>=0) versions.push_back(site.exactVersion);
                        if(site.bodyWalk) for(const auto *segment:{&site.bodyWalk->propertyCandidates,&site.bodyWalk->doubleCandidates})
                            for(const auto &candidate:*segment) if(candidate.version>=0) versions.push_back(candidate.version);
                        if(site.bodyWalk) for(size_t owner:owners)
                            if(!_RequireCandidateReads(owner,*site.bodyWalk)) return false;
                        for(size_t owner:owners) for(int32_t version:versions)
                            if(!_Declares(owner,fb::SlotDomain::PropertyResult,uint32_t(version)))
                                return _Bad(_StepName(owner)+": owning leaf omitted exact PropertyResult dependency");
                    }
                }
                // Sparse normalization consumes the shape's raw Default opinions,
                // not the serialized normalized layout or a produced point version.
                std::map<std::string,const fb::RigExecWireExternalDeclaredInput *> sparseSites;
                std::set<std::string> repeatedSparseSites;
                for (const auto &site:rev.leafSites) {
                    const std::string path=RigExecFormatPathText(_f,site.path);
                    if (!sparseSites.emplace(path,&site).second) repeatedSparseSites.insert(path);
                }
                bool haveSparse=false;
                for (const auto &channel:rev.blendChannels) for (const auto &sample:channel.samples) {
                    if (!sample.blendShape) {
                        if (sample.shapeValid) return _Bad(row+": dense blend sample has sparse shape fact");
                        continue;
                    }
                    haveSparse=true;
                    if (!_PathId(sample.blendShape,_Prim,row,"blend_shape")) return false;
                    const auto requireSparse=[&](const char *attribute,InputTag tag) {
                        const std::string path=RigExecFormatPathText(_f,sample.blendShape)+"."+attribute;
                        if (repeatedSparseSites.count(path)) return _Bad(row+": sparse blend source has duplicate owning leaf site");
                        const auto at=sparseSites.find(path);
                        const auto *found=at==sparseSites.end()?nullptr:at->second;
                        if (!found || !found->read || found->read->tag!=tag ||
                            found->time!=fb::ExternalInputTime::AtDefault ||
                            found->flavour!=fb::ExternalInputFlavour::Raw ||
                            found->exactVersion!=-1 || found->exactRecord!=-1 || found->exactValueType!=-1)
                            return _Bad(row+": sparse blend source requires exact Raw Default owning leaf site");
                        const auto &read=*found->read;
                        const bool source=read.mode==ReadMode::Raw && read.walk.size()==1 && read.sampleTime==1 &&
                            _f.inputs[read.walk.front()].name()==found->path;
                        const bool missing=read.mode==ReadMode::Resolved && read.walk.empty() &&
                            read.propertyCandidates.empty() && read.doubleCandidates.empty() && read.rawFallbackSlot==-1;
                        if (found->bodyWalk || (!source && !missing))
                            return _Bad(row+": sparse blend source requires raw Default head or missing-source fallback");
                        if (missing) {
                            const auto &fallback=_f.values[read.constant];
                            const bool empty=tag==InputTag::Vec3fArray ? _f.vec3fArrays[fallback.array].v.empty() : _f.intArrays[fallback.array].v.empty();
                            if (!empty) return _Bad(row+": missing sparse blend source requires empty typed fallback");
                        }
                        return true;
                    };
                    if (!requireSparse("offsets",InputTag::Vec3fArray) ||
                        !requireSparse("pointIndices",InputTag::IntArray)) return false;
                }
                if (haveSparse) for (size_t step=0;step<_f.steps.size();++step) {
                    const auto &body=_f.steps[step];
                    if (body.kind==(derived?fb::StepKind::Derived:fb::StepKind::RevisionStatic) && body.object==ordinary &&
                        !_Declares(step,fb::SlotDomain::ChainBase,uint32_t(c)))
                        return _Bad(_StepName(step)+": sparse blend layout omitted raw chain cardinality read");
                }
                return true;
            };
            for (size_t r=0;r<chain.revisions.size();++r)
                if (!check(chain.revisions[r],"geometry.chain["+_N(c)+"].revision["+_N(r)+"]",r,false)) return false;
            for (size_t r=0;r<chain.derived.size();++r)
                if (!check(*chain.derived[r].revision,"geometry.chain["+_N(c)+"].derived["+_N(r)+"]",r,true)) return false;
        }
        for (size_t m=0;m<_f.externalMovers.size();++m)
            for (size_t k=0;k<_f.externalMovers[m].declaredInputs.size();++k)
                if (!validate(_f.externalMovers[m].declaredInputs[k],"external_movers["+_N(m)+"].declared_inputs["+_N(k)+"]")) return false;
        return true;
    }

    bool _GraphValueSlot(uint32_t domain, uint32_t slot) const
    {
        using D = fb::SlotDomain;
        uint64_t width = 0;
        switch (D(domain)) {
        case D::Avars: case D::PosedM: case D::FinalMatrix: case D::BaseMatrix:
        case D::WeightFrames: case D::WeightFramesBase: case D::Rest: case D::Ladder:
            width = _slots; break;
        case D::PoseBase: width = _basePool; break;
        case D::PoseFin: width = _finPool; break;
        case D::Aggregate: case D::SolverPoints: case D::Candidates:
            width = _f.pose->solvers.size(); break;
        case D::CommitTable: case D::CommitDelta: width = _f.pose->commits.size(); break;
        case D::CommitStaging:
            for (const auto &commit : _f.pose->commits)
                if (commit.stagingBase >= 0 && slot >= uint32_t(commit.stagingBase) &&
                    uint64_t(slot) - uint32_t(commit.stagingBase) < commit.propagate.size()) return true;
            return false;
        case D::ConstraintDelta: width = _f.geometry->deltaBasePaths.size(); break;
        case D::PropertyResult:
            width = _f.phasedConsumers.size();
            for (const auto &chain : _f.propertyChains) width += chain.revisions.size() + 1;
            break;
        case D::ChainBase: case D::ChainPoints: case D::ChainInput: width = _chains; break;
        case D::RevisionPacket: case D::RevisionTransforms: case D::RevisionDone:
        case D::ChainDirty: width = _revisions; break;
        case D::RevisionOut:
            for (size_t i = 0; i < _f.geometry->revisionChunkBase.size(); ++i)
                if (_f.geometry->revisionChunkBase[i] >= 0 &&
                    slot >= uint32_t(_f.geometry->revisionChunkBase[i]) &&
                    uint64_t(slot) - uint32_t(_f.geometry->revisionChunkBase[i]) <
                        uint64_t(_f.geometry->revisionChunkCount[i])) return true;
            return false;
        case D::DerivedOut: case D::DerivedBase: width = _derived; break;
        case D::WeightPacket: width = _weights; break;
        case D::PoseWeight: width = _f.pose->poseWeightPaths.size(); break;
        case D::FrameMatrix: width = _f.pose->frameRecords.size(); break;
        case D::SkinTopology: width = _revisions + _derived; break;
        case D::WeightField: width = _f.geometry->weightFields.size(); break;
        case D::SpaceValue: width = _f.providerProgram ? _f.providerProgram->valueKeys.size() : 0; break;
        case D::SpaceLeaf: width = _f.providerProgram ? _f.providerProgram->sampled.size() : 0; break;
        case D::ConstraintInputs: width = _f.pose->constraintArrays.size(); break;
        case D::SwitchFrame: width = _f.pose->spaceCheckpoints.size(); break;
        case D::RequiredStageFramesAdmission: width = 1; break;
        case D::Snapshots: default: return false;
        }
        return uint64_t(slot) < width;
    }

    bool _StepGraph()
    {
        if (_f.commonGraph) {
            std::set<std::pair<uint32_t,uint32_t>> typedValues;
            for (size_t i = 0; i < _f.commonGraph->valueSpecs.size(); ++i) {
                const auto &spec = _f.commonGraph->valueSpecs[i];
                if(!typedValues.insert({spec.domain,spec.slot}).second)
                    return _Bad("common_graph.value_specs: duplicate typed storage identity");
                if (!_GraphValueSlot(spec.domain, spec.slot))
                    return _Bad("common_graph.value_specs[" + _N(i) + "]: typed slot exceeds its owning storage");
            }
        }
        std::vector<RigExecStepGraphRange> sampledPoseSeeds;
        if(_f.commonGraph) for(uint64_t id:_f.commonGraph->leaves) {
            const auto &graph=*_f.commonGraph;
            if(id>=graph.valueSpecs.size()) return _Bad("common_graph.leaves: value ID out of range");
            if(std::find(graph.excludedValues.begin(),graph.excludedValues.end(),id)!=graph.excludedValues.end()) continue;
            const auto &spec=graph.valueSpecs[size_t(id)];
            const auto domain=fb::SlotDomain(spec.domain);
            const bool source=domain==fb::SlotDomain::SolverPoints || domain==fb::SlotDomain::SpaceLeaf ||
                domain==fb::SlotDomain::DerivedBase || domain==fb::SlotDomain::ChainInput || domain==fb::SlotDomain::ConstraintInputs ||
                domain==fb::SlotDomain::RequiredStageFramesAdmission;
            const bool poseSeed=(domain==fb::SlotDomain::PoseBase || domain==fb::SlotDomain::PoseFin) &&
                spec.slot<_f.slotMeta->paths.size() &&
                std::find(_f.slotMeta->xformSlots.begin(),_f.slotMeta->xformSlots.end(),int32_t(spec.slot))!=_f.slotMeta->xformSlots.end();
            if(!source && !poseSeed) return _Bad("common_graph.leaves: produced domain has no active producer or exclusion");
            if(poseSeed) sampledPoseSeeds.push_back({uint8_t(domain),spec.slot,spec.slot+1});
        }
        std::vector<RigExecStepGraphRange> excluded = std::move(sampledPoseSeeds);
        if(_f.commonGraph) for(uint64_t id:_f.commonGraph->excludedValues) {
            const auto &graph=*_f.commonGraph;
            if(id>=graph.valueSpecs.size() ||
               std::find(graph.leaves.begin(),graph.leaves.end(),id)==graph.leaves.end())
                return _Bad("common_graph.excluded_values: not a declared value leaf");
            const auto &spec=graph.valueSpecs[size_t(id)];
            if(spec.domain>uint32_t(fb::SlotDomain::MAX) || spec.slot==UINT32_MAX)
                return _Bad("common_graph.excluded_values: invalid typed value");
            for(const auto &step:_f.steps) for(const auto &write:step.writes)
                if(uint32_t(write.domain())==spec.domain && spec.slot>=write.begin() && spec.slot<write.end())
                    return _Bad("common_graph.excluded_values: active writer remains");
            excluded.push_back({uint8_t(spec.domain),spec.slot,spec.slot+1});
        }
        const std::string why =
            RigExecStepGraphError(_f.steps, *_f.clustering, _GraphRange,excluded);
        return why.empty() || _Bad(why);
    }

    bool _WeightFieldTables()
    {
        std::vector<int64_t> producers(_f.geometry->weightFields.size(), -1);
        for (size_t i = 0; i < _f.steps.size(); ++i) {
            const auto &step = _f.steps[i];
            if (step.kind == fb::StepKind::WeightField) {
                if (!_Has(_f.geometry->weightFields, step.object) || step.part != -1)
                    return _Bad(_StepName(i) + ": invalid WeightField descriptor");
                const size_t field = size_t(step.object);
                if (producers[field] >= 0)
                    return _Bad(_StepName(i) + ": duplicate WeightField producer");
                producers[field] = int64_t(i);
                if (step.writes.size() != 1 ||
                    step.writes[0].domain() != fb::SlotDomain::WeightField ||
                    step.writes[0].begin() != uint32_t(field) ||
                    step.writes[0].end() != uint32_t(field + 1))
                    return _Bad(_StepName(i) + ": WeightField output ownership differs");
            } else {
                for (const auto &range : step.writes)
                    if (range.domain() == fb::SlotDomain::WeightField)
                        return _Bad(_StepName(i) + ": non-WeightField producer writes a field");
            }
        }
        for (size_t f = 0; f < _f.geometry->weightFields.size(); ++f) {
            const auto &field = _f.geometry->weightFields[f];
            const std::string who = "WeightField " + _N(f);
            if (producers[f] < 0 && !_ExcludedValue(fb::SlotDomain::WeightField,uint32_t(f))) return _Bad(who + ": no producer");
            if (field.form > fb::WeightFieldForm::MAX ||
                field.placementPhase > fb::WeightFieldPlacementPhase::MAX ||
                !_Has(_f.geometry->weightObjects, field.object))
                return _Bad(who + ": invalid form, placement phase or object");
            if (field.form == fb::WeightFieldForm::EnvelopeProperty) {
                if (!_Has(_f.propertyChains, field.consumer) || field.part <= 0 ||
                    size_t(field.part) > _f.propertyChains[size_t(field.consumer)].revisions.size())
                    return _Bad(who + ": property consumer out of range");
                const auto &revision = _f.propertyChains[size_t(field.consumer)].revisions[size_t(field.part - 1)];
                if (revision.weightField != int32_t(f) || revision.envelope != field.object)
                    return _Bad(who + ": property consumer backlink or object differs");
            } else if (field.form == fb::WeightFieldForm::EnvelopeConstraint) {
                if (!_Has(_f.pose->walkSteps, field.consumer) || field.part != 0)
                    return _Bad(who + ": constraint consumer out of range");
                const auto &walk = _f.pose->walkSteps[size_t(field.consumer)];
                if (walk.solverBatch || !_Has(_f.pose->constraints, walk.index))
                    return _Bad(who + ": consumer is not a constraint walk");
                const auto &constraint = _f.pose->constraints[size_t(walk.index)];
                if (constraint.weightField != int32_t(f) ||
                    constraint.weightObjectIndex != field.object || constraint.pointsTarget != 0)
                    return _Bad(who + ": constraint consumer backlink or object differs");
            } else {
                if (!_Has(_f.geometry->revisionIndex, field.consumer) || field.part != 0)
                    return _Bad(who + ": revision consumer out of range");
                const auto &entry = _f.geometry->revisionIndex[size_t(field.consumer)];
                const auto &revision = _f.geometry->chains[size_t(entry.first)].revisions[size_t(entry.second)];
                if (revision.weightField != int32_t(f) || revision.weightObject != field.object)
                    return _Bad(who + ": revision consumer backlink or object differs");
            }
            std::vector<int32_t> reachable,pending{field.object};
            std::set<int32_t> seen;
            while(!pending.empty()) {
                const int32_t object=pending.back();pending.pop_back();
                if(!seen.insert(object).second) continue;
                if(!_Has(_f.geometry->weightObjects,object)) return _Bad(who+": closure object out of range");
                reachable.push_back(object);
                const auto &weight=_f.geometry->weightObjects[size_t(object)];
                if(weight.base>=0) pending.push_back(weight.base);
                pending.insert(pending.end(),weight.inputs.begin(),weight.inputs.end());
            }
            if (!std::is_sorted(field.availableChains.begin(), field.availableChains.end()) ||
                std::adjacent_find(field.availableChains.begin(), field.availableChains.end()) !=
                    field.availableChains.end())
                return _Bad(who + ": publication context is not canonical");
            for (int32_t chain : field.availableChains)
                if (!_Has(_f.propertyChains, chain))
                    return _Bad(who + ": publication context chain out of range");
            std::vector<int32_t> expectedContext;
            const size_t contextEnd=field.form==fb::WeightFieldForm::EnvelopeProperty
                ? size_t(field.consumer) : _f.propertyChains.size();
            for(size_t c=0;c<contextEnd;++c) expectedContext.push_back(int32_t(c));
            if(field.availableChains!=expectedContext)
                return _Bad(who+": publication context differs from its consumer");
            if (field.scalarReads.size() != field.scalarObjects.size() ||
                field.scalarReads.size() != field.scalarMembers.size())
                return _Bad(who + ": scalar descriptor cardinality differs");
            std::set<std::pair<int32_t, uint8_t>> members;
            for (size_t r = 0; r < field.scalarReads.size(); ++r) {
                if (!_Has(_f.geometry->weightObjects, field.scalarObjects[r]) ||
                    field.scalarMembers[r] > fb::WeightFieldScalarMember::MAX ||
                    !members.emplace(field.scalarObjects[r],
                                     uint8_t(field.scalarMembers[r])).second)
                    return _Bad(who + ": invalid or duplicate scalar member");
                if (field.scalarReads[r].tag != InputTag::Float ||
                    field.scalarReads[r].mode != ReadMode::Resolved)
                    return _Bad(who + ": scalar read has a non-oracle origin");
                _oracleScalarContexts.emplace(&field.scalarReads[r], &field.availableChains);
                if (!_Read(&field.scalarReads[r], -1, _Resolved, who,
                           "scalar_reads", long(r))) return false;
                if(producers[f]>=0 && !_RequireCandidateReads(size_t(producers[f]),field.scalarReads[r])) return false;
                if(producers[f]>=0) for(const auto *segment:{&field.scalarReads[r].propertyCandidates,&field.scalarReads[r].doubleCandidates})
                    for(const auto &candidate:*segment) for(int32_t available:field.availableChains) {
                        if(!_Has(_f.propertyChains,available)) return _Bad(who+": publication context chain out of range");
                        const auto &chain=_f.propertyChains[size_t(available)];
                        const uint32_t path=_f.inputs[candidate.slot].name();
                        if(_f.inputs[chain.target].name()==path && !_BodyRead(size_t(producers[f]),fb::SlotDomain::PropertyResult,chain.versionBase+uint32_t(chain.revisions.size())))
                            return _Bad(who+": missing contextual oracle chain read");
                        for(const auto &record:_f.phasedConsumers)
                            if(record.chain==available && _f.inputs[record.consumer].name()==path &&
                               !_BodyRead(size_t(producers[f]),fb::SlotDomain::PropertyResult,record.version))
                                return _Bad(who+": missing contextual oracle record read");
                    }
            }
            for (int32_t slot : field.volumes)
                if (slot < 0 || size_t(slot) >= _f.slotMeta->paths.size())
                    return _Bad(who + ": placement slot out of range");
            if(producers[f]>=0) for(int32_t slot:field.volumes)
                if(!_BodyRead(size_t(producers[f]),field.placementPhase==fb::WeightFieldPlacementPhase::Base?
                    fb::SlotDomain::WeightFramesBase:fb::SlotDomain::WeightFrames,uint32_t(slot)))
                    return _Bad(who+": missing selected placement producer read");
            for(const auto &point:field.pointReads) {
                if(!_Has(_f.geometry->weightObjects,point.object) || point.leaf<0 || point.leaf>2 || !point.binding)
                    return _Bad(who+": malformed point read owner");
                if(!_Phase(point.binding->phase,who,"point_read.phase")) return false;
                for(const auto &candidate:point.binding->candidates) {
                    if(candidate.chain()<0 || size_t(candidate.chain())>=_f.geometry->chains.size() ||
                       candidate.version()<0 || size_t(candidate.version())>_f.geometry->chains[size_t(candidate.chain())].revisions.size())
                        return _Bad(who+": point read version out of range");
                    if(producers[f]<0) continue;
                    const size_t owner=size_t(producers[f]);
                    if(point.binding->finalRead) {
                        if(!_BodyRead(owner,fb::SlotDomain::ChainPoints,uint32_t(candidate.chain()))) return _Bad(who+": missing phased points producer read");
                    } else if(candidate.version()==0) {
                        if(!_BodyRead(owner,fb::SlotDomain::ChainBase,uint32_t(candidate.chain()))) return _Bad(who+": missing phased base producer read");
                    } else {
                        const uint32_t revision=uint32_t(_f.geometry->chainRevisionBegin[size_t(candidate.chain())])+uint32_t(candidate.version())-1;
                        if(!_BodyRead(owner,fb::SlotDomain::RevisionDone,revision) || !_BodyRead(owner,fb::SlotDomain::ChainDirty,revision))
                            return _Bad(who+": missing phased revision producer read");
                    }
                }
            }
        }
        for (size_t c = 0; c < _f.propertyChains.size(); ++c) {
            const auto &chain = _f.propertyChains[c];
            for (size_t r = 0; r < chain.revisions.size(); ++r) {
                const auto &revision = chain.revisions[r];
                if ((revision.envelope >= 0) != (revision.weightField >= 0) ||
                    (revision.weightField >= 0 &&
                     !_Has(_f.geometry->weightFields, revision.weightField)))
                    return _Bad("PropertyRevision chain " + _N(c) + " part " +
                                _N(r + 1) + ": missing or foreign WeightField");
            }
        }
        for (size_t c = 0; c < _f.pose->constraints.size(); ++c) {
            const auto &constraint = _f.pose->constraints[c];
            const bool envelope = constraint.weightObject != 0 &&
                                  constraint.pointsTarget == 0;
            if (envelope != (constraint.weightField >= 0) ||
                (constraint.weightField >= 0 &&
                 !_Has(_f.geometry->weightFields, constraint.weightField)))
                return _Bad("Constraint " + _N(c) + ": missing or foreign WeightField");
        }
        return true;
    }

    bool _ProviderTables()
    {
        if (!_f.providerProgram) {
            for (size_t i=0;i<_f.steps.size();++i)
                if (_f.steps[i].kind == fb::StepKind::SpaceExpression)
                    return _Bad(_StepName(i)+": missing provider program");
            return true;
        }
        const auto &p=*_f.providerProgram;
        const size_t n=p.valueKeys.size();
        if (p.defaults.size()!=n) return _Bad("provider default/value cardinality differs");
        std::set<std::string> keys;
        for (size_t i=0;i<n;++i) {
            if (p.valueKeys[i].empty() || !keys.insert(p.valueKeys[i]).second)
                return _Bad("provider value keys are empty or duplicated");
            const auto &v=p.defaults[i];
            using Kind=fb::ProviderValueKind;
            if (v.kind>Kind::Vec3i ||
                bool(v.scalarDouble)!=(v.kind==Kind::Double) ||
                bool(v.scalarFloat)!=(v.kind==Kind::Float) ||
                bool(v.vector)!=(v.kind==Kind::Vector) ||
                bool(v.matrix)!=(v.kind==Kind::Matrix) ||
                bool(v.vec3f)!=(v.kind==Kind::Vec3f) ||
                bool(v.vec2f)!=(v.kind==Kind::Vec2f) ||
                bool(v.vec3i)!=(v.kind==Kind::Vec3i) ||
                (v.kind==Kind::Frame && (v.framePoints.size()!=4 || (v.frameFlags&~uint32_t(31)))) ||
                (v.kind!=Kind::Frame && (!v.framePoints.empty() || v.frameFlags)) ||
                (v.kind!=Kind::FloatArray && !v.floats.empty()) ||
                (v.kind!=Kind::DoubleArray && !v.doubles.empty()) ||
                (v.kind!=Kind::Vec3fArray && !v.vec3fs.empty()) ||
                (v.kind!=Kind::Vec3dArray && !v.vec3ds.empty()) ||
                (v.kind!=Kind::IntArray && !v.ints.empty()) ||
                (v.kind!=Kind::MatrixArray && !v.matrices.empty()) ||
                (v.kind!=Kind::TokenArray && !v.tokens.empty()) ||
                (v.kind!=Kind::BoolArray && !v.bools.empty()) ||
                (v.kind!=Kind::Vec2fArray && !v.vec2fs.empty()))
                return _Bad("provider value "+_N(i)+": malformed typed state");
            for(uint8_t bit:v.bools) if(bit>1)
                return _Bad("provider value "+_N(i)+": invalid bool array byte");
            uint64_t count=1;
            switch(v.kind) {
            case Kind::Empty:count=0;break;
            case Kind::FloatArray:count=v.floats.size();break;
            case Kind::DoubleArray:count=v.doubles.size();break;
            case Kind::Vec3fArray:count=v.vec3fs.size();break;
            case Kind::Vec3dArray:count=v.vec3ds.size();break;
            case Kind::IntArray:count=v.ints.size();break;
            case Kind::MatrixArray:count=v.matrices.size();break;
            case Kind::TokenArray:count=v.tokens.size();break;
            case Kind::BoolArray:count=v.bools.size();break;
            case Kind::Vec2fArray:count=v.vec2fs.size();break;
            default:break;
            }
            if(v.initialized && v.count!=count)
                return _Bad("provider value "+_N(i)+": typed count differs from payload");
        }
        size_t propertyVersions=_f.phasedConsumers.size();
        for(const auto &chain:_f.propertyChains) propertyVersions+=chain.revisions.size()+1;
        std::vector<int> writers(n,-1);
        for (size_t i=0;i<p.ops.size();++i) {
            const auto &op=p.ops[i];
            if(op.kind>12 || op.kind==8 || op.kind==9 || op.output>=n || op.owner.empty())
                return _Bad("provider op "+_N(i)+": invalid kind/output/owner");
            // An affine frame expression (format 21) names its expression
            // and its inputs match that expression's layout; every other
            // kind names none.
            const bool affine=op.kind==uint32_t(RigExecProviderOpKind::AffineFrame);
            if(affine && _f.formatVersion<RigExecFormatAffineFramesVersion)
                return _Bad("provider op "+_N(i)+": format "+_N(_f.formatVersion)+
                            " holds no affine frame expressions");
            if(affine ? !RigExecAffineFrameInputsMatch(op.affineKind,op.inputs.size(),op.affineTargets)
                      : (op.affineKind!=0 || op.affineTargets!=0))
                return _Bad("provider op "+_N(i)+": affine frame expression does not match its inputs");
            if(writers[size_t(op.output)]>=0)
                return _Bad("provider op "+_N(i)+": duplicate output producer");
            writers[size_t(op.output)]=int(i);
            for(auto input:op.inputs)
                if(input!=UINT64_MAX && input>=n)
                    return _Bad("provider op "+_N(i)+": input out of range");
        }
        for (auto value:p.leaves)
            if(value>=n) return _Bad("provider leaf value out of range");
        const auto leaves=[&](const auto &rows,bool sampled) {
            for(size_t i=0;i<rows.size();++i) {
                const auto &leaf=rows[i];
                if(leaf.value>=n || leaf.path.empty()) return _Bad("provider bridge value/path invalid");
                if(writers[size_t(leaf.value)]>=0) return _Bad("provider bridge duplicates producer");
                writers[size_t(leaf.value)]=int(p.ops.size()+i);
                if(sampled) {
                    if(leaf.inputSlot < -1 || (leaf.inputSlot>=0 && size_t(leaf.inputSlot)>=_f.inputs.size()))
                        return _Bad("provider sampled input slot out of range");
                    if(leaf.propertyVersion < -1 || (leaf.propertyVersion>=0 &&
                       size_t(leaf.propertyVersion)>=propertyVersions))
                        return _Bad("provider sampled property version out of range");
                } else {
                    if(leaf.providerSlot<0 || size_t(leaf.providerSlot)>=_f.slotMeta->paths.size() ||
                       RigExecFormatPathText(_f,_f.slotMeta->paths[size_t(leaf.providerSlot)])!=leaf.path)
                        return _Bad("provider external slot out of range");
                    if(leaf.interveningRead && !_Read(leaf.interveningRead.get(), int(InputTag::Matrix4d), _Baked, "provider external", "intervening_read")) return false;
                }
            }
            return true;
        };
        if(!leaves(p.sampled,true) || !leaves(p.externalInputs,false)) return false;
        for(size_t i=0;i<p.routedInputs.size();++i) {
            const auto &route=p.routedInputs[i];
            if(route.value>=n || route.consumer.empty() || route.source.empty() ||
               route.readPhase.empty() || !_Has(_f.crossDomainReads,route.crossRead))
                return _Bad("provider routed input "+_N(i)+": invalid value/context/cross read");
            if(writers[size_t(route.value)]>=0)
                return _Bad("provider routed input "+_N(i)+": duplicate output producer");
            writers[size_t(route.value)]=int(p.ops.size()+p.sampled.size()+p.externalInputs.size()+i);
            const auto &read=_f.crossDomainReads[size_t(route.crossRead)];
            if(RigExecFormatPathText(_f,read.consumer)!=route.consumer ||
               RigExecFormatPathText(_f,read.source)!=route.source)
                return _Bad("provider routed input "+_N(i)+": cross read ownership differs");
        }
        std::vector<uint8_t> bodyOps(p.ops.size()),bodySampled(p.sampled.size()),
            bodyExternal(p.externalInputs.size()),bodyRouted(p.routedInputs.size()),
            bodyFrames(_f.pose->providerFrameInputs.size());
        for(size_t i=0;i<_f.steps.size();++i) {
            const auto &step=_f.steps[i];
            if(step.kind!=fb::StepKind::SpaceExpression) continue;
            const auto contains=[&](const auto &,fb::SlotDomain domain,uint64_t slot) {
                return _BodyRead(i,domain,slot);
            };
            uint64_t output=UINT64_MAX;
            const std::string who=_StepName(i);
            if(step.object<0) return _Bad(who+": negative provider body index");
            const size_t object=size_t(step.object);
            if(step.part==0) {
                if(object>=p.ops.size() || bodyOps[object]++) return _Bad(who+": missing or duplicate provider op body");
                const auto &op=p.ops[object]; output=op.output;
                for(uint64_t input:op.inputs) if(input!=UINT64_MAX &&
                    !contains(step.reads,fb::SlotDomain::SpaceValue,input))
                    return _Bad(who+": missing SpaceValue operand read");
            } else if(step.part==2) {
                if(object>=p.sampled.size() || bodySampled[object]++) return _Bad(who+": missing or duplicate provider sampled body");
                const auto &leaf=p.sampled[object]; output=leaf.value;
                if(!contains(step.reads,fb::SlotDomain::SpaceLeaf,object)) return _Bad(who+": missing SpaceLeaf read");
                if(leaf.propertyVersion!=-1) return _Bad(who+": raw provider source cannot select a property version");
            } else if(step.part==1) {
                if(object>=p.externalInputs.size() || bodyExternal[object]++) return _Bad(who+": missing or duplicate provider external body");
                const auto &leaf=p.externalInputs[object]; output=leaf.value;
                if(leaf.computation=="computeRestFrame") {
                    if(leaf.providerSlot>=0 && !contains(step.reads,fb::SlotDomain::Rest,uint32_t(leaf.providerSlot)))
                        return _Bad(who+": missing provider Rest read");
                } else if(leaf.computation=="computePointFrame" || leaf.computation=="computeBasePointFrame") {
                    uint32_t expected=uint32_t(leaf.providerSlot);
                    if(leaf.frameVersion!=int32_t(expected)) return _Bad(who+": provider frame version belongs to another phase or owner");
                    if(leaf.providerSlot>=0 && (leaf.frameVersion<0 || !_GraphValueSlot(uint32_t(fb::SlotDomain::PoseBase),uint32_t(leaf.frameVersion)) ||
                        !contains(step.reads,fb::SlotDomain::PoseBase,uint32_t(leaf.frameVersion))))
                        return _Bad(who+": missing or invalid provider PoseBase read");
                } else {
                    if(leaf.computation!="interveningSpace" || !leaf.interveningRead)
                        return _Bad(who+": unsupported provider external computation");
                    if(!_RequireCandidateReads(i,*leaf.interveningRead)) return false;
                    for(const auto *segment:{&leaf.interveningRead->propertyCandidates,&leaf.interveningRead->doubleCandidates})
                        for(const auto &candidate:*segment) if(candidate.version>=0 &&
                            !contains(step.reads,fb::SlotDomain::PropertyResult,uint32_t(candidate.version)))
                            return _Bad(who+": missing intervening property read");
                    for(const auto *segment:{&leaf.interveningRead->propertyCandidates,&leaf.interveningRead->doubleCandidates})
                        for(const auto &candidate:*segment) if(candidate.crossDomain>=0) {
                            const auto &cross=_f.crossDomainReads[size_t(candidate.crossDomain)];
                            if(cross.kind==fb::CrossDomainReadKind::PropertyResult &&
                               !contains(step.reads,fb::SlotDomain::PropertyResult,cross.propertyVersion))
                                return _Bad(who+": missing intervening routed property read");
                            if(cross.kind==fb::CrossDomainReadKind::SpaceValue &&
                               !contains(step.reads,fb::SlotDomain::SpaceValue,uint32_t(cross.spaceValue)))
                                return _Bad(who+": missing intervening SpaceValue read");
                            if(cross.kind==fb::CrossDomainReadKind::PoseFrame)
                                for(uint32_t frame:cross.frames) if(!contains(step.reads,
                                    cross.baseFrame?fb::SlotDomain::PoseBase:fb::SlotDomain::PoseFin,frame))
                                    return _Bad(who+": missing intervening pose frame read");
                        }
                }
            } else if(step.part==3) {
                if(object>=p.routedInputs.size() || bodyRouted[object]++) return _Bad(who+": missing or duplicate provider routed body");
                const auto &route=p.routedInputs[object]; output=route.value;
                const auto &read=_f.crossDomainReads[size_t(route.crossRead)];
                if(read.kind==fb::CrossDomainReadKind::PropertyResult &&
                    !contains(step.reads,fb::SlotDomain::PropertyResult,read.propertyVersion))
                    return _Bad(who+": missing routed PropertyResult read");
                if(read.kind==fb::CrossDomainReadKind::SpaceValue &&
                    !contains(step.reads,fb::SlotDomain::SpaceValue,uint32_t(read.spaceValue)))
                    return _Bad(who+": missing routed SpaceValue read");
                if(read.kind==fb::CrossDomainReadKind::PoseFrame)
                    for(uint32_t frame:read.frames) if(!contains(step.reads,
                        read.baseFrame?fb::SlotDomain::PoseBase:fb::SlotDomain::PoseFin,frame))
                        return _Bad(who+": missing routed pose frame read");
                if(read.kind==fb::CrossDomainReadKind::Points || read.kind==fb::CrossDomainReadKind::PointElement)
                    for(const auto &point:read.points) {
                        const uint32_t chain=uint32_t(point.first);
                        if(read.finalPoints && !contains(step.reads,fb::SlotDomain::ChainPoints,chain))
                            return _Bad(who+": missing routed ChainPoints read");
                        if(!read.finalPoints && point.second==0 && !contains(step.reads,fb::SlotDomain::ChainBase,chain))
                            return _Bad(who+": missing routed ChainBase read");
                        if(!read.finalPoints && point.second>0) {
                            const uint32_t revision=uint32_t(_f.geometry->chainRevisionBegin[chain])+uint32_t(point.second)-1;
                            if(!contains(step.reads,fb::SlotDomain::RevisionDone,revision) ||
                               !contains(step.reads,fb::SlotDomain::ChainDirty,revision))
                                return _Bad(who+": missing routed point-version read");
                        }
                    }
            } else if(step.part==4) {
                if(object>=_f.pose->providerFrameInputs.size() || bodyFrames[object]++)
                    return _Bad(who+": missing or duplicate provider frame body");
                const auto &frame=_f.pose->providerFrameInputs[object]; output=frame.value;
                if(frame.slot<0 || size_t(frame.slot)>=_slots || output>=n ||
                   !_PathId(frame.reader,_Prim,who,"reader") ||
                   (p.defaults[size_t(output)].kind!=fb::ProviderValueKind::Frame && p.defaults[size_t(output)].kind!=fb::ProviderValueKind::Empty) ||
                   !_GraphValueSlot(uint32_t(frame.base?fb::SlotDomain::PoseBase:fb::SlotDomain::PoseFin),frame.version) ||
                   !contains(step.reads,frame.base?fb::SlotDomain::PoseBase:fb::SlotDomain::PoseFin,frame.version))
                    return _Bad(who+": invalid or undeclared contextual provider frame");
                std::set<uint32_t> owned{uint32_t(frame.slot)};
                for(const auto &commit:_f.pose->commits) {
                    if(frame.base && !commit.solverOutput)continue;
                    const auto &target=frame.base?commit.slotBaseWrites:commit.slotWrites;
                    const auto &carry=frame.base?commit.slotBaseCarry:commit.slotCarry;
                    for(size_t k=0;k<commit.slots.size();++k)if(commit.slots[k]==frame.slot) {
                        if(k<target.size())owned.insert(target[k]);
                        if(k<carry.size())owned.insert(carry[k]);
                    }
                    const auto &desc=frame.base?commit.descendantBaseWrites:commit.descendantWrites;
                    const auto &descCarry=frame.base?commit.descendantBaseCarry:commit.descendantCarry;
                    for(size_t k=0;k<commit.propagate.size();++k)if(commit.propagate[k].first==frame.slot) {
                        if(k<desc.size())owned.insert(desc[k]);
                        if(k<descCarry.size())owned.insert(descCarry[k]);
                    }
                }
                for(const auto &refresh:_f.pose->providerRefreshes) {
                    if(refresh.slot==frame.slot)owned.insert(frame.base?refresh.baseWrite:refresh.finWrite);
                    for(const auto &carry:refresh.carries)if(carry.slot==frame.slot)
                        owned.insert(frame.base?carry.baseWrite:carry.finWrite);
                }
                if(!owned.count(frame.version))return _Bad(who+": provider frame version belongs to another owner");
                if(writers[size_t(output)]>=0)return _Bad(who+": provider frame output has another producer");
                writers[size_t(output)]=int(p.ops.size()+p.sampled.size()+p.externalInputs.size()+p.routedInputs.size()+object);
            } else return _Bad(who+": unsupported provider body part");
            if(step.writes.size()!=1 || step.writes[0].domain()!=fb::SlotDomain::SpaceValue ||
                step.writes[0].begin()!=output || step.writes[0].end()!=output+1)
                return _Bad(who+": provider output ownership differs");
        }
        const auto complete=[&](const auto &rows,const auto &seen) {
            for(size_t i=0;i<rows.size();++i) if(!seen[i] &&
                !_ExcludedValue(fb::SlotDomain::SpaceValue,uint32_t(rows[i].output))) return false;
            return true;
        };
        if(!complete(p.ops,bodyOps)) return _Bad("provider op has no body or excluded output");
        const auto bridges=[&](const auto &rows,const auto &seen) {
            for(size_t i=0;i<rows.size();++i) if(!seen[i] &&
                !_ExcludedValue(fb::SlotDomain::SpaceValue,uint32_t(rows[i].value))) return false;
            return true;
        };
        if(!bridges(p.sampled,bodySampled) || !bridges(p.externalInputs,bodyExternal) || !bridges(p.routedInputs,bodyRouted) ||
           !bridges(_f.pose->providerFrameInputs,bodyFrames))
            return _Bad("provider bridge has no body or excluded output");
        return true;
    }

    // The common graph proves its rows against Step declarations. This
    // separate check proves those declarations contain the actual body SSA
    // bindings, so a self-consistent graph cannot hide a body dependency.
    bool _PoseBodyDeclarations()
    {
        using D=fb::SlotDomain;
        using K=fb::StepKind;
        const auto &pose=*_f.pose;
        // Count exact kind/object identities once for this immutable file.
        // Invalid objects remain subject to the unchanged body/range checks.
        std::vector<size_t> checkpointProducers(pose.spaceCheckpoints.size(),0);
        std::vector<size_t> refreshProducers(pose.providerRefreshes.size(),0);
        for(const auto &step:_f.steps) {
            if(step.object<0)continue;
            if(step.kind==K::SpaceCheckpoint && size_t(step.object)<checkpointProducers.size())
                ++checkpointProducers[size_t(step.object)];
            else if(step.kind==K::ProviderRefresh && size_t(step.object)<refreshProducers.size())
                ++refreshProducers[size_t(step.object)];
        }
        std::vector<uint32_t> finLast(_slots),baseLast(_slots);
        for(size_t slot=0;slot<_slots;++slot) finLast[slot]=baseLast[slot]=uint32_t(slot);
        for(const auto &commit:pose.commits) {
            for(int32_t slot:commit.slots) {
                finLast[size_t(slot)]=uint32_t(_slots+size_t(slot));
                if(commit.solverOutput) baseLast[size_t(slot)]=uint32_t(_slots+size_t(slot));
            }
            for(const auto &entry:commit.propagate) {
                finLast[size_t(entry.first)]=uint32_t(_slots+size_t(entry.first));
                if(commit.solverOutput) baseLast[size_t(entry.first)]=uint32_t(_slots+size_t(entry.first));
            }
        }
        for(const auto &refresh:pose.providerRefreshes) {
            finLast[size_t(refresh.slot)]=baseLast[size_t(refresh.slot)]=uint32_t(_slots+size_t(refresh.slot));
            for(const auto &carry:refresh.carries)
                finLast[size_t(carry.slot)]=baseLast[size_t(carry.slot)]=uint32_t(_slots+size_t(carry.slot));
        }
        _bodyFinLast=finLast; _bodyBaseLast=baseLast;
        for (size_t context=0;context<pose.spaceCheckpoints.size();++context) {
            const auto &record=pose.spaceCheckpoints[context];
            const std::string label="pose.space_checkpoints["+_N(context)+"]";
            if(record.key.empty()) return _Bad(label+": empty key");
            if(!_Index(record.anchor,_slots,true,label,"anchor") ||
               !_Indices(record.recompose,_slots,false,label,"recompose")) return false;
            for(int slot:record.recompose)
                if(_f.slotMeta->slotKind[size_t(slot)]!=fb::SlotKind::FirstFramePose)
                    return _Bad(label+": recompose is not a provider");
            const size_t producers=checkpointProducers[context];
            if(producers>1 || (producers==0 && !_ExcludedValue(D::SwitchFrame,uint32_t(context))))
                return _Bad("pose.space_checkpoints["+_N(context)+"]: missing or duplicate producer");
        }
        for(size_t context=0;context<pose.providerRefreshes.size();++context) {
            const auto &refresh=pose.providerRefreshes[context];
            const size_t producers=refreshProducers[context];
            bool excluded=_ExcludedValue(D::PoseBase,refresh.baseWrite) && _ExcludedValue(D::PoseFin,refresh.finWrite);
            for(const auto &carry:refresh.carries)
                excluded=excluded && _ExcludedValue(D::PoseBase,carry.baseWrite) && _ExcludedValue(D::PoseFin,carry.finWrite);
            if(producers>1 || (!producers && !excluded))
                return _Bad("pose.provider_refreshes["+_N(context)+"]: missing or duplicate producer");
        }
        for(size_t owner=0;owner<_f.steps.size();++owner) {
            const auto &step=_f.steps[owner];
            std::set<std::pair<D,uint32_t>> reads,writes;
            const auto read=[&](D domain,uint32_t value) { reads.emplace(domain,value); };
            const auto readAll=[&](D domain,const auto &values) { for(auto value:values) read(domain,uint32_t(value)); };
            const auto writeAll=[&](D domain,const auto &values) { for(auto value:values) writes.emplace(domain,uint32_t(value)); };
            if(step.kind==K::RestCompose || step.kind==K::LadderCompose) {
                if(!_Has(pose.composeGroups,step.object))return _Bad(_StepName(owner)+": invalid compose group");
                const auto &group=pose.composeGroups[size_t(step.object)];
                for(int32_t slot=group.begin;slot<group.end;++slot) {
                    if(_f.slotMeta->slotKind[size_t(slot)]!=fb::SlotKind::FirstFramePose)continue;
                    const auto &values=pose.ladders[size_t(slot)].spaceValues;
                    for(size_t channel=0;channel<values.size();++channel)
                        if((step.kind==K::RestCompose ? channel==0 : channel!=0) && values[channel]>=0)
                            read(D::SpaceValue,uint32_t(values[channel]));
                }
            } else if(step.kind==K::SpaceCheckpoint) {
                if(step.object<0 || size_t(step.object)>=pose.spaceCheckpoints.size() || step.part!=-1)
                    return _Bad(_StepName(owner)+": invalid checkpoint body");
                const auto &checkpoint=pose.spaceCheckpoints[size_t(step.object)];
                if(checkpoint.key.empty() || !_Index(checkpoint.anchor,_slots,true,_StepName(owner),"anchor") ||
                   !_Indices(checkpoint.recompose,_slots,false,_StepName(owner),"recompose")) return false;
                if(checkpoint.anchor>=0) read(D::PosedM,uint32_t(checkpoint.anchor));
                for(int slot:checkpoint.recompose) {
                    if(pose.ladders.size()<=size_t(slot) || _f.slotMeta->slotKind[size_t(slot)]!=fb::SlotKind::FirstFramePose)
                        return _Bad(_StepName(owner)+": checkpoint recompose is not a provider");
                    read(D::Avars,uint32_t(slot)); read(D::Ladder,uint32_t(slot));
                }
                writes.emplace(D::SwitchFrame,uint32_t(step.object));
            } else if(step.kind==K::ProviderRefresh) {
                if(step.part!=0 || step.object<0 || size_t(step.object)>=pose.providerRefreshes.size())
                    return _Bad(_StepName(owner)+": invalid provider refresh body");
                const auto &refresh=pose.providerRefreshes[size_t(step.object)];
                if(!_f.providerProgram || refresh.baseValue>=_f.providerProgram->valueKeys.size() ||
                   refresh.currentValue>=_f.providerProgram->valueKeys.size() ||
                   (_f.providerProgram->defaults[size_t(refresh.baseValue)].kind!=fb::ProviderValueKind::Frame &&
                    _f.providerProgram->defaults[size_t(refresh.baseValue)].kind!=fb::ProviderValueKind::Empty) ||
                   (_f.providerProgram->defaults[size_t(refresh.currentValue)].kind!=fb::ProviderValueKind::Frame &&
                    _f.providerProgram->defaults[size_t(refresh.currentValue)].kind!=fb::ProviderValueKind::Empty))
                    return _Bad(_StepName(owner)+": provider refresh has invalid expression values");
                const auto expressionOwned=[&](uint64_t value) {
                    size_t producers=0;
                    for(const auto &op:_f.providerProgram->ops)
                        if(op.output==value && op.kind==uint32_t(RigExecProviderOpKind::PosedFrame) &&
                           op.owner==RigExecFormatPathText(_f,_f.slotMeta->paths[size_t(refresh.slot)]))++producers;
                    return producers==1;
                };
                if(!expressionOwned(refresh.baseValue) || !expressionOwned(refresh.currentValue))
                    return _Bad(_StepName(owner)+": provider refresh expression belongs to another owner or kind");
                read(D::SpaceValue,uint32_t(refresh.baseValue));read(D::SpaceValue,uint32_t(refresh.currentValue));
                read(D::PoseBase,refresh.baseRead);read(D::PoseFin,refresh.finRead);
                writes.emplace(D::PoseBase,refresh.baseWrite);writes.emplace(D::PoseFin,refresh.finWrite);
                for(const auto &carry:refresh.carries) {
                    read(D::PoseBase,carry.baseRead);read(D::PoseFin,carry.finRead);
                    writes.emplace(D::PoseBase,carry.baseWrite);writes.emplace(D::PoseFin,carry.finWrite);
                    for(int32_t blocker:carry.blockingSlots) {
                        if(!_Has(pose.ladders,blocker) || !pose.ladders[size_t(blocker)].parentSpace)
                            return _Bad(_StepName(owner)+": provider refresh blocker lacks parent-space binding");
                        if(!_RequireCandidateReads(owner,*pose.ladders[size_t(blocker)].parentSpace))return false;
                        const std::string rawPath=RigExecFormatPathText(_f,_f.slotMeta->paths[size_t(blocker)])+".parent:space";
                        size_t rawRow=0,rawMatches=0;
                        for(size_t index=0;index<_f.providerProgram->sampled.size();++index)
                            if(_f.providerProgram->sampled[index].path==rawPath){rawRow=index;++rawMatches;}
                        if(rawMatches!=1)return _Bad(_StepName(owner)+": provider refresh blocker lacks exact raw source");
                        const auto &raw=_f.providerProgram->sampled[rawRow];
                        if(raw.propertyVersion!=-1 || raw.inputSlot<0 || size_t(raw.inputSlot)>=_f.inputs.size() ||
                           _f.inputs[size_t(raw.inputSlot)].type()!=InputTag::Matrix4d ||
                           RigExecFormatPathText(_f,_f.inputs[size_t(raw.inputSlot)].name())!=rawPath)
                            return _Bad(_StepName(owner)+": provider refresh blocker raw source type or identity differs");
                        read(D::SpaceLeaf,uint32_t(rawRow));
                    }
                }
                for(const auto &guard:refresh.priorConstraints)read(D::CommitTable,guard.first);
                std::set<std::pair<D,uint32_t>> declared;
                for(const auto &range:step.writes)
                    for(uint32_t slot=range.begin();slot<range.end();++slot)declared.emplace(range.domain(),slot);
                if(declared!=writes)return _Bad(_StepName(owner)+": provider refresh write ownership differs");
            } else if(step.kind==K::ComposeSubtree) {
                if(step.object<0 || size_t(step.object)>=pose.composeGroups.size())
                    return _Bad(_StepName(owner)+": invalid compose group");
                const auto &group=pose.composeGroups[size_t(step.object)];
                for(const auto &sw:pose.spaceSwitches) if(sw.slot>=int32_t(group.begin) && sw.slot<int32_t(group.end)) {
                    const auto version=[&](const fb::RigExecWireFrameVersion &value) {
                        if(value.context>=0) read(D::SwitchFrame,uint32_t(value.context));
                    };
                    version(*sw.parentRead); version(*sw.spaceRead);
                    for(const auto &value:sw.sourceReads) version(value);
                }
                for (const auto &ac : pose.autoClavicles) {
                    if (ac.slot < group.begin || ac.slot >= group.end) continue;
                    for (const auto &frame : ac.frames) {
                        if (frame.slot < 0) continue;
                        if (frame.computation == 2) read(D::Rest, uint32_t(frame.slot));
                        else if (frame.computation == 1) read(D::Ladder, uint32_t(frame.slot));
                        else if (frame.recompose.empty() && frame.slot != ac.slot)
                            read(D::PoseFin, finLast[size_t(frame.slot)]);
                        for (int child : frame.recompose) {
                            read(D::Avars, uint32_t(child));
                            read(D::Ladder, uint32_t(child));
                        }
                    }
                    for (const auto &input : ac.scalars)
                        if (!_RequireCandidateReads(owner, input)) return false;
                }
            } else if(step.kind==K::Solve) {
                if(!_Has(pose.solvers,step.object)) return _Bad(_StepName(owner)+": solver body object out of range");
                const auto &solver=pose.solvers[size_t(step.object)];
                readAll(D::PoseFin,solver.controlReads);
                for(const auto binding:{std::make_pair(solver.start,solver.startRead),
                    std::make_pair(solver.root,solver.rootRead),std::make_pair(solver.mid,solver.midRead),
                    std::make_pair(solver.end,solver.endRead),std::make_pair(solver.pole,solver.poleRead),
                    std::make_pair(solver.spaceSlot,uint32_t(solver.spaceRead))})
                    if(binding.first>=0) read(D::PoseFin,binding.second);
                for(size_t k=0;k<solver.restReads.size();++k)
                    if(k<solver.restIsLive.size() && solver.restIsLive[k]) read(D::PoseFin,solver.restReads[k]);
            } else if(step.kind==K::FrameMatrix) {
                if(!_Has(pose.frameRecords,step.object)) return _Bad(_StepName(owner)+": frame body object out of range");
                read(D::PoseFin,pose.frameRecords[size_t(step.object)].version());
            } else if(step.kind==K::ProviderMatrix) {
                if(step.object<0 || size_t(step.object)>=_slots) return _Bad(_StepName(owner)+": provider body object out of range");
                const size_t slot=size_t(step.object);
                if(step.part) read(D::PoseFin,finLast[slot]);
                read(D::PoseBase,baseLast[slot]);
            } else if(step.kind==K::PoseInterpolator) {
                if(!_Has(pose.poseInterpolators,step.object)) return _Bad(_StepName(owner)+": interpolator body object out of range");
                const auto &interp=pose.poseInterpolators[size_t(step.object)];
                if(interp.driverSlot>=0) read(D::PoseFin,finLast[size_t(interp.driverSlot)]);
                if(interp.parentSlot>=0) read(D::PoseFin,finLast[size_t(interp.parentSlot)]);
            } else if(step.kind==K::Constraint || step.kind==K::SolverCommit ||
                      step.kind==K::CommitDelta || step.kind==K::PropagateChunk || step.kind==K::CommitApply) {
                if(!_Has(pose.commits,step.object)) return _Bad(_StepName(owner)+": commit body object out of range");
                const auto &commit=pose.commits[size_t(step.object)];
                if(step.kind==K::Constraint) {
                    if(!_Has(pose.walkSteps,step.object) || pose.walkSteps[size_t(step.object)].solverBatch ||
                       !_Has(pose.constraints,pose.walkSteps[size_t(step.object)].index))
                        return _Bad(_StepName(owner)+": constraint body object out of range");
                    const auto &c=pose.constraints[size_t(pose.walkSteps[size_t(step.object)].index)];
                    if(commit.sourceReads.size()<c.sources.size() || commit.poleReads.size()<c.poleObjects.size())
                        return _Bad(_StepName(owner)+": incomplete constraint body SSA bindings");
                    for(size_t k=0;k<c.sources.size();++k) if(c.sources[k]>=0) read(D::PoseFin,commit.sourceReads[k]);
                    if(c.target>=0) read(D::PoseFin,commit.targetRead);
                    readAll(D::PoseFin,commit.targetReads);
                    if(c.worldUpObject>=0) read(D::PoseFin,commit.worldUpRead);
                    if(c.effector>=0) read(D::PoseFin,commit.effectorRead);
                    for(size_t k=0;k<c.poleObjects.size();++k) if(c.poleObjects[k]>=0) read(D::PoseFin,commit.poleReads[k]);
                    const auto ancestors=[&](const auto &values) { for(const auto &value:values) { read(D::PoseFin,value.fin); read(D::PoseBase,value.base); } };
                    for(const auto &values:commit.sourceAncestors) ancestors(values.v);
                    for(const auto &values:commit.poleAncestors) ancestors(values.v);
                    ancestors(commit.worldUpAncestors); ancestors(commit.effectorAncestors);
                }
                const bool apply=step.kind==K::CommitApply || (!commit.split && (step.kind==K::Constraint || step.kind==K::SolverCommit));
                if(step.kind==K::CommitDelta || (apply && !commit.split)) readAll(D::PoseFin,commit.slotReads);
                if(step.kind==K::PropagateChunk || (apply && !commit.split)) {
                    const size_t begin=step.kind==K::PropagateChunk ? size_t(step.part)*64 : 0;
                    const size_t end=step.kind==K::PropagateChunk ? std::min(begin+64,commit.propagate.size()) : commit.propagate.size();
                    if(commit.descendantReads.size()<end || commit.closestReads.size()<end)
                        return _Bad(_StepName(owner)+": incomplete propagation body SSA bindings");
                    for(size_t k=begin;k<end;++k) { read(D::PoseFin,commit.descendantReads[k]); read(D::PoseFin,commit.closestReads[k]); }
                }
                if(apply) {
                    writeAll(D::PoseFin,commit.slotWrites); writeAll(D::PoseFin,commit.descendantWrites);
                    writeAll(D::PoseBase,commit.slotBaseWrites); writeAll(D::PoseBase,commit.descendantBaseWrites);
                    const auto carries=[&](D domain,const auto &values) { for(auto value:values) {
                        if(!writes.count({domain,uint32_t(value)})) read(domain,uint32_t(value));
                    } };
                    carries(D::PoseFin,commit.slotCarry); carries(D::PoseFin,commit.descendantCarry);
                    carries(D::PoseBase,commit.slotBaseCarry); carries(D::PoseBase,commit.descendantBaseCarry);
                }
            }
            for(const auto &value:reads)
                if(!_BodyRead(owner,value.first,value.second))
                    return _Bad(_StepName(owner)+": body SSA missing read "+fb::EnumNameSlotDomain(value.first)+"["+_N(value.second)+"]");
            for(const auto &value:writes) {
                const bool declared=std::any_of(step.writes.begin(),step.writes.end(),[&](const auto &range) {
                    return range.domain()==value.first && range.begin()<=value.second && value.second<range.end();
                });
                if(!declared) return _Bad(_StepName(owner)+": body SSA missing write "+fb::EnumNameSlotDomain(value.first)+"["+_N(value.second)+"]");
            }
        }
        return true;
    }

    void _CollectGeometryBodyReads()
    {
        using D=fb::SlotDomain; using K=fb::StepKind;
        const auto &g=*_f.geometry;
        for(size_t i=0;i<_f.steps.size();++i) {
            const auto &step=_f.steps[i];
            const auto read=[&](D domain,uint32_t slot) { _bodyReads[i].emplace(domain,slot); };
            const auto point=[&](uint32_t chain,uint32_t version) {
                if(version==0)read(D::ChainBase,chain);
                else {const uint32_t revision=uint32_t(g.chainRevisionBegin[chain])+version-1;
                    read(D::RevisionDone,revision);read(D::ChainDirty,revision);}
            };
            if(step.kind==K::WeightPacket && _Has(g.weightObjects,step.object)) {
                const auto &weight=g.weightObjects[size_t(step.object)];
                if(weight.base>=0)read(D::WeightPacket,uint32_t(weight.base));
                for(int32_t input:weight.inputs)read(D::WeightPacket,uint32_t(input));
                if(weight.providerSlot>=0 && size_t(weight.providerSlot)<_bodyBaseLast.size())
                    read(D::PoseBase,_bodyBaseLast[size_t(weight.providerSlot)]);
                continue;
            }
            if(step.kind==K::WeightField && _Has(g.weightFields,step.object)) {
                const auto &field=g.weightFields[size_t(step.object)];
                if(field.form==fb::WeightFieldForm::Revision && _Has(g.revisionIndex,field.consumer)) {
                    const auto &index=g.revisionIndex[size_t(field.consumer)];
                    read(D::ChainBase,uint32_t(index.first));
                    if(index.second>0) {
                        read(D::ChainDirty,uint32_t(field.consumer)-1);
                        for(int32_t k=0;k<index.second;++k)
                            read(D::RevisionDone,uint32_t(g.chainRevisionBegin[size_t(index.first)])+uint32_t(k));
                    }
                }
                for(int32_t slot:field.volumes)
                    read(field.placementPhase==fb::WeightFieldPlacementPhase::Base?D::WeightFramesBase:D::WeightFrames,uint32_t(slot));
                continue;
            }
            if(step.kind==K::VolumePlacements && step.object>=0) {
                if(size_t(step.object)<_bodyFinLast.size() && step.part==1)read(D::PoseFin,_bodyFinLast[size_t(step.object)]);
                else if(size_t(step.object)<_bodyBaseLast.size() && step.part==2)read(D::PoseBase,_bodyBaseLast[size_t(step.object)]);
                continue;
            }
            if(step.kind==K::Constraint && _Has(_f.pose->walkSteps,step.object)) {
                const auto &walk=_f.pose->walkSteps[size_t(step.object)];
                if(!walk.solverBatch && _Has(_f.pose->constraints,walk.index)) {
                    const auto &constraint=_f.pose->constraints[size_t(walk.index)];
                    if(constraint.weightField>=0)read(D::WeightField,uint32_t(constraint.weightField));
                }
            }
            if(step.kind==K::ChainStatus && _Has(g.chains,step.object)) {
                const size_t chain=size_t(step.object);read(D::ChainBase,uint32_t(chain));
                for(int32_t revision=g.chainRevisionBegin[chain];revision<g.chainRevisionEnd[chain];++revision)
                    read(D::RevisionDone,uint32_t(revision));
                continue;
            }
            const bool derived=step.kind==K::Derived;
            const bool regular=step.kind==K::InfluenceFold || step.kind==K::RevisionStatic ||
                step.kind==K::RevisionChunk || step.kind==K::RevisionFuse;
            if(!derived && !regular)continue;
            const auto &indices=derived?g.derivedIndex:g.revisionIndex;
            if(!_Has(indices,step.object))continue;
            const auto &index=indices[size_t(step.object)];
            const auto &chain=g.chains[size_t(index.first)];
            const auto &revision=derived?*chain.derived[size_t(index.second)].revision:
                chain.revisions[size_t(index.second)];
            const uint32_t id=uint32_t(step.object),c=uint32_t(index.first);
            const bool skin=revision.op==uint8_t(fb::RevisionOp::Skin);
            read(D::ChainBase,c);
            if(derived || step.kind==K::InfluenceFold) {
                const D own=revision.finalPhase?D::FinalMatrix:D::BaseMatrix;
                const bool both=revision.op==uint8_t(fb::RevisionOp::SurfaceProjector) ||
                    revision.op==uint8_t(fb::RevisionOp::ShaderDials);
                for(int32_t slot:{revision.transformSlot,revision.transformSpaceSlot,revision.carrySpaceSlot})
                    if(slot>=0) {if(both){read(D::BaseMatrix,uint32_t(slot));read(D::FinalMatrix,uint32_t(slot));}
                        else read(own,uint32_t(slot));}
                for(int32_t slot:revision.influenceSlots)if(slot>=0)read(own,uint32_t(slot));
            }
            if(derived) {read(D::ChainPoints,c);read(D::DerivedBase,id);continue;}
            if(step.kind==K::InfluenceFold) {
                if(revision.constraintDelta>=0)read(D::ConstraintDelta,uint32_t(revision.constraintDelta));
                if(skin)read(D::RevisionPacket,id);
            } else if(step.kind==K::RevisionStatic) {
                if(!skin)read(D::RevisionTransforms,id);
                if(revision.weightObject>=0)read(D::WeightPacket,uint32_t(revision.weightObject));
                if(revision.weightField>=0)read(D::WeightField,uint32_t(revision.weightField));
                if(revision.driverFramesSolver>=0)read(D::Aggregate,uint32_t(revision.driverFramesSolver));
                for(const auto &channel:revision.blendChannels)
                    if(channel.poseWeight>=0)read(D::PoseWeight,uint32_t(channel.poseWeight));
            } else if(!_groupWritten[c].empty()) {
                // A chain with groups (format 20), as _RangeSteps holds the
                // declarations: a group step reads group `part` of the
                // entering version; a join its written parts; a Whole fuse
                // its chunks and every group of the entering version.
                const size_t r=size_t(index.second);
                const auto group=[&](size_t k) {
                    const int w=_groupEntering[c][r][k];
                    if(w<0)return;
                    const auto &writer=chain.revisions[size_t(w)];
                    read(D::RevisionOut,uint32_t(writer.chunkBase)+uint32_t(k)+
                        uint32_t(RigExecFormatIsRangeRevision(writer)?0:writer.chunks.size()));
                };
                const bool range=RigExecFormatIsRangeRevision(revision);
                const bool keyed=RigExecFormatIsKeyedRevision(revision);
                read(D::RevisionPacket,id);
                if(step.kind==K::RevisionChunk) {
                    if(!_Has(revision.chunks,step.part))continue;
                    if(!range && !keyed) {
                        point(c,uint32_t(r));
                        if(skin)read(D::RevisionTransforms,id);
                        continue;
                    }
                    group(size_t(step.part));
                    if(range && skin)read(D::RevisionTransforms,id);
                    if(keyed) {
                        const D own=revision.finalPhase?D::FinalMatrix:D::BaseMatrix;
                        for(int32_t position:revision.chunks[size_t(step.part)].key)
                            if(_Has(revision.influenceSlots,position) && revision.influenceSlots[size_t(position)]>=0)
                                read(own,uint32_t(revision.influenceSlots[size_t(position)]));
                    }
                    continue;
                }
                read(D::RevisionTransforms,id);point(c,uint32_t(r));
                if(revision.weightObject>=0)read(D::WeightPacket,uint32_t(revision.weightObject));
                const std::vector<char> &written=_groupWritten[c][r];
                if(range) {
                    for(size_t k=0;k<written.size();++k)
                        if(written[k])read(D::RevisionOut,uint32_t(revision.chunkBase)+uint32_t(k));
                    continue;
                }
                for(size_t k=0;k<revision.chunks.size();++k)read(D::RevisionOut,uint32_t(revision.chunkBase)+uint32_t(k));
                for(size_t k=0;k<written.size();++k)group(k);
            } else if(step.kind==K::RevisionChunk) {
                read(D::RevisionPacket,id);point(c,uint32_t(index.second));
                if(RigExecFormatIsKeyedRevision(revision) && _Has(revision.chunks,step.part)) {
                    const D own=revision.finalPhase?D::FinalMatrix:D::BaseMatrix;
                    for(int32_t position:revision.chunks[size_t(step.part)].key)
                        if(_Has(revision.influenceSlots,position) && revision.influenceSlots[size_t(position)]>=0)
                            read(own,uint32_t(revision.influenceSlots[size_t(position)]));
                } else if(skin)read(D::RevisionTransforms,id);
            } else {
                read(D::RevisionPacket,id);read(D::RevisionTransforms,id);point(c,uint32_t(index.second));
                if(revision.weightObject>=0)read(D::WeightPacket,uint32_t(revision.weightObject));
                for(size_t k=0;k<revision.chunks.size();++k)read(D::RevisionOut,uint32_t(revision.chunkBase)+uint32_t(k));
            }
        }
    }

    bool _RequiredStageFrames()
    {
        using D=fb::SlotDomain; using K=fb::StepKind;
        _CollectGeometryBodyReads();
        const auto *admission=_f.pose->requiredStageFramesAdmission.get();
        if(!admission || admission->admitted>1 || (admission->admitted ? admission->firstBadTarget!=-1 :
           admission->firstBadTarget<0 || size_t(admission->firstBadTarget)>=_f.slotMeta->xformSlots.size()))
            return _Bad("pose.required_stage_frames_admission: invalid admission or first failed target");
        for(uint64_t id:_f.commonGraph->excludedValues)
            if(id<_f.commonGraph->valueSpecs.size() &&
               _f.commonGraph->valueSpecs[size_t(id)].domain==uint32_t(D::RequiredStageFramesAdmission))
                return _Bad("common_graph.excluded_values: required stage-frame admission cannot be excluded");
        const auto prep=[](K kind) { return kind==K::PropertyRevision || kind==K::RestCompose ||
            kind==K::LadderCompose || kind==K::SkinTopology; };
        std::map<std::pair<D,uint32_t>,std::vector<size_t>> producers;
        std::vector<size_t> pending;
        for(size_t i=0;i<_f.steps.size();++i) {
            const auto &step=_f.steps[i];
            for(const auto &range:step.writes) {
                if(range.domain()==D::RequiredStageFramesAdmission)
                    return _Bad(_StepName(i)+": required stage-frame admission is source-only");

            }
            if(!prep(step.kind))continue;
            pending.push_back(i);
            if(step.kind==K::PropertyRevision && step.part>0) {
                const auto &chain=_f.propertyChains[size_t(step.object)];
                const auto &revision=chain.revisions[size_t(step.part-1)];
                if(!_BodyRead(i,D::PropertyResult,chain.versionBase+uint32_t(step.part)-1))return false;
                if(revision.weightField>=0 && !_BodyRead(i,D::WeightField,uint32_t(revision.weightField)))
                    return _Bad(_StepName(i)+": missing actual property weight-field read");
                for(const auto *read:{revision.enabled.get(),revision.defaultWeight.get(),revision.value.get(),revision.min.get(),revision.max.get()})
                    if(read && !_RequireCandidateReads(i,*read))return false;
            }
            // These memo bindings were checked against the exact body bindings above.
            for(const auto &read:step.headInputReads)
                if(!_RequireCandidateReads(i,read))return false;
        }
        // Only actual body reads can be queried by ancestry. Intersect each
        // writer range with those keys instead of expanding unused slots.
        std::set<std::pair<D,uint32_t>> actualReads;
        for(const auto &owner:_bodyReads)
            actualReads.insert(owner.second.begin(),owner.second.end());
        for(size_t i=0;i<_f.steps.size();++i)
            for(const auto &range:_f.steps[i].writes) {
                auto read=actualReads.lower_bound(std::make_pair(range.domain(),range.begin()));
                for(;read!=actualReads.end() && read->first==range.domain() &&
                     read->second<range.end();++read)
                    producers[*read].push_back(i);
            }
        std::set<size_t> ancestors;
        while(!pending.empty()) {
            const size_t owner=pending.back();pending.pop_back();
            if(!ancestors.insert(owner).second)continue;
            const auto found=_bodyReads.find(owner);
            if(found==_bodyReads.end())continue;
            for(const auto &read:found->second) {
                const auto producer=producers.find(read);
                if(producer!=producers.end())pending.insert(pending.end(),producer->second.begin(),producer->second.end());
            }
        }
        for(size_t i=0;i<_f.steps.size();++i) {
            const auto &step=_f.steps[i];
            const bool helper=step.kind==K::WeightField || step.kind==K::SpaceExpression;
            const bool required=step.kind!=K::SnapshotFinals && !prep(step.kind) &&
                step.kind!=K::AvarInputs && !(helper && ancestors.count(i));
            size_t count=0;
            for(const auto &range:step.reads)if(range.domain()==D::RequiredStageFramesAdmission) {
                if(range.begin()!=0 || range.end()!=1)
                    return _Bad(_StepName(i)+": required stage-frame admission must read exactly source slot zero");
                ++count;
            }
            if(count!=size_t(required))
                return _Bad(_StepName(i)+": required stage-frame admission read differs from actual preparation role");
        }
        return true;
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
        const auto producerDomain = [](fb::SlotDomain domain) {
            return domain == fb::SlotDomain::PropertyResult || domain == fb::SlotDomain::Rest ||
                   domain == fb::SlotDomain::Ladder || domain == fb::SlotDomain::SkinTopology;
        };
        for (size_t i = 0; i < _f.steps.size(); ++i) {
            const auto &step = _f.steps[i];
            const bool kindHead =
                (step.kind >= fb::StepKind::PropertyRevision &&
                 step.kind <= fb::StepKind::SkinTopology) ||
                (step.kind == fb::StepKind::WeightField && step.object >= 0 &&
                 size_t(step.object) < _f.geometry->weightFields.size() &&
                 _f.geometry->weightFields[size_t(step.object)].form !=
                     fb::WeightFieldForm::Revision);
            if (step.isHead != kindHead)
                return _Bad(_StepName(i) + ": head category differs from its body kind");
            if (step.isHead && (step.isSource || step.externalReads))
                return _Bad(_StepName(i) + ": a head is never a source or always region step");
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
            if (step.kind == fb::StepKind::PropertyRevision) {
                const auto &chain = _f.propertyChains[size_t(step.object)];
                std::set<uint32_t> expectedSlots;
                bool volatileBody = false;
                if (step.part == 0) expectedSlots.insert(chain.target);
                else {
                    const auto &revision = chain.revisions[size_t(step.part - 1)];
                    volatileBody = false;

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
                if (revision.op!=uint8_t(fb::RevisionOp::Skin))
                    return _Bad(_StepName(i) + ": topology head has no Skin layout body");
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
                        expected.push_back(ladderBody.interveningSpace.get());
                    } else {
                        expected.push_back(ladderBody.posedSpace.get());
                        expected.push_back(ladderBody.defaultSpace.get());
                        expected.push_back(ladderBody.parentSpace.get());
                        expected.push_back(ladderBody.parentDefaultSpace.get());
                        expected.push_back(ladderBody.avarDefaultSpace.get());
                        expected.push_back(ladderBody.posedDefaultSpace.get());
                        expected.push_back(ladderBody.rotationSign.get());
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
                if (!_Read(&step.headInputReads[r], int(step.headInputReads[r].tag), _AnyMode,
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
            // Bound property reads use per-consumer typed phase authority;
            // producer chain discovery order does not constrain visibility.
            const int readerChain = owner!=owners.end() && _f.steps[owner->second].kind==fb::StepKind::PropertyRevision
                ? _f.steps[owner->second].object : -1;
            // Property revisions own entering phases and suppress later target overlays.
            // Ordinary readers retain conditional Final fallbacks after typed records.
            const auto oracle = _oracleScalarContexts.find(input);
            const auto oracleKind = [&](uint32_t slot) {
                auto kind = fb::PropertyCandidateKind::SlotOnly;
                if (oracle == _oracleScalarContexts.end()) return kind;
                // Oracle overlays are per-consumer publication contexts, in
                // chain target/record order, distinct from ordinary bindings.
                for (int32_t c : *oracle->second) {
                    if (_f.propertyChains[size_t(c)].target == slot)
                        kind = fb::PropertyCandidateKind::ChainFinal;
                    for (const auto &record : _f.phasedConsumers)
                        if (record.chain == uint32_t(c) && record.consumer == slot)
                            kind = fb::PropertyCandidateKind::PhasedRecord;
                }
                return kind;
            };
            // Resolved oracle/path reads consult the publication overlay at
            // every hop; their target Final remains a valid later fallback.
            using CandidateKind = fb::PropertyCandidateKind;
            std::map<uint32_t,CandidateKind> expectedProperty, expectedDouble;
            std::set<uint32_t> selected;
            const bool sourceOnly = _sampledLeafReads.count(input) != 0;
            const bool boundWalk = owner!=owners.end() || input->mode==ReadMode::Baked ||
                _boundWalkReads.count(input)!=0;
            const auto derive = [&](uint32_t slotId, InputTag readAs) {
                const auto &slot = _f.inputs[slotId];
                const bool ownEntering = boundWalk && readerChain>=0 && slot.chain()==readerChain &&
                    input->mode==ReadMode::Pinned && !input->walk.empty() && input->walk.front()==slotId;
                const bool chain = slot.chain()>=0 && !ownEntering && (readerChain<0 || !selected.count(uint32_t(slot.chain())));
                const bool record = slot.phased()>=0 && (!boundWalk || input->mode!=ReadMode::Pinned);
                const auto kind = oracle!=_oracleScalarContexts.end() ? oracleKind(slotId) :
                    sourceOnly ? CandidateKind::SlotOnly : chain ? CandidateKind::ChainFinal :
                    record ? CandidateKind::PhasedRecord : CandidateKind::SlotOnly;
                if(readerChain>=0 && record && _ChainTargetTag(_f.phasedConsumers[size_t(slot.phased())].consumerType)==readAs)
                    selected.insert(_f.phasedConsumers[size_t(slot.phased())].chain);
                return kind;
            };
            size_t typedDoubleStart = input->walk.size();
            if(!input->doubleCandidates.empty()) {
                for(size_t k=0;k<input->walk.size();++k)
                    if(_f.inputs[input->walk[k]].type()==InputTag::Double) {typedDoubleStart=k;break;}
                if(typedDoubleStart==input->walk.size())return _Bad(label+": Double traversal has no Double hop");
            }
            // The Float prefix includes the first Double hop; the Double
            // suffix revisits it with its exact type and carries selected records.
            const size_t prefixEnd = typedDoubleStart<input->walk.size() ? typedDoubleStart+1 : input->walk.size();
            if(!input->propertyCandidates.empty() || input->doubleCandidates.empty())
                for(size_t k=0;k<prefixEnd;++k)
                    expectedProperty.emplace(input->walk[k],derive(input->walk[k],input->tag));
            for(size_t k=typedDoubleStart;k<input->walk.size();++k)
                expectedDouble.emplace(input->walk[k],derive(input->walk[k],InputTag::Double));
            auto requiredDouble = expectedDouble;
            // DoubleTail starts a fresh cycle guard at its Double boundary.
            // A cycle can revisit the Float prefix after the required suffix.
            // Those optional hops retain exact typed phase and version checks.
            if(typedDoubleStart<input->walk.size())
                for(size_t k=0;k<typedDoubleStart;++k)
                    expectedDouble.emplace(input->walk[k],derive(input->walk[k],InputTag::Double));
            if(!sourceOnly) for(const auto &segment : {std::make_pair(&expectedProperty,&input->propertyCandidates),
                                                     std::make_pair(&requiredDouble,&input->doubleCandidates)})
                for(const auto &[slotId,expected] : *segment.first) {
                    if(expected==CandidateKind::SlotOnly)continue;
                    bool found=false;
                    for(const auto &candidate:*segment.second)
                        found=found || (candidate.slot==slotId && candidate.kind==uint8_t(expected));
                    if(!found)return _Bad((owner==owners.end()?label:_StepName(owner->second))+
                        ": chain crossing lacks its property candidate at input slot "+std::to_string(slotId));
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
                    const auto &expectedSegment = list==&input->doubleCandidates ? expectedDouble : expectedProperty;
                    const auto typed = expectedSegment.find(candidate.slot);
                    if(typed==expectedSegment.end())return _Bad(label+": candidate is outside its typed walk segment");
                    const auto expected = typed->second;
                    if (candidate.kind != uint8_t(expected))
                        return _Bad(label + ": candidate kind disagrees with its reader phase at input slot " +
                            std::to_string(candidate.slot) + " (kind " + std::to_string(candidate.kind) +
                            ", expected " + std::to_string(uint8_t(expected)) + ", version " +
                            std::to_string(candidate.version) + ", cross-domain " + std::to_string(candidate.crossDomain) + ")");
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
            if (writers[v] < 0 && !_ExcludedValue(fb::SlotDomain::PropertyResult,uint32_t(v))) return _Bad("PropertyRevision version " + _N(v) + ": no writer");
        for (size_t i = 0; i < _f.steps.size(); ++i) {
            const auto &step = _f.steps[i];
            for (const auto &range : step.reads) {
                if (!producerDomain(range.domain())) continue;
                const auto *table = range.domain() == fb::SlotDomain::PropertyResult ? &writers
                    : range.domain() == fb::SlotDomain::Rest ? &rest
                    : range.domain() == fb::SlotDomain::Ladder ? &ladder : &topology;
                if (range.end() > table->size()) return _Bad(_StepName(i) + ": head read out of range");
                for (uint32_t v = range.begin(); v < range.end(); ++v)
                    if (((*table)[v] < 0 && !_ExcludedValue(range.domain(),v)) ||
                        ((*table)[v] >= 0 && size_t((*table)[v]) >= i))
                        return _Bad(_StepName(i) + ": head read has no earlier producer");
            }
            const auto require = [&](fb::SlotDomain domain, int slot) {
                if (slot < 0 || size_t(slot) >= slots) return true;
                if ((domain==fb::SlotDomain::Rest || domain==fb::SlotDomain::Ladder) &&
                    _f.slotMeta->slotKind[size_t(slot)]!=fb::SlotKind::FirstFramePose) return true;
                return _BodyRead(i, domain, uint32_t(slot)) ||
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
            if (bodyRevision && RigExecFormatIsKeyedRevision(*bodyRevision) &&
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
            if (bodyRevision && bodyRevision->op==uint8_t(fb::RevisionOp::Skin) &&
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
            if (revision && revision->op==uint8_t(fb::RevisionOp::Skin) && topology[layout] < 0 &&
                !_ExcludedValue(fb::SlotDomain::SkinTopology,uint32_t(layout)))
                return _Bad("SkinTopology layout " + _N(layout) + ": Skin layout body has no producer");
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
        if (!_ClusterSet(c.always.get(), "cones.always") ||
            !_ClusterSet(c.poseClusters.get(), "cones.pose_clusters")) {
            return false;
        }
        // Ordinary producer metadata names a real cluster. An explicitly
        // excluded Avars producer has no active cluster in the common graph.
        if (!_Size(c.avarCluster.size(), _slots, row, "avar_cluster") ||
            !_Indices(c.avarCluster, _clusters, true, row, "avar_cluster") ||
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
        for (size_t slot=0;slot<c.avarCluster.size();++slot)
            if(c.avarCluster[slot]<0 && !_ExcludedValue(fb::SlotDomain::Avars,uint32_t(slot)))
                return _Bad("cones.avar_cluster["+_N(slot)+"]: missing active Avars producer without an excluded value");
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
        std::set<int> clavicleTargets;
        for(size_t i=0;i<p.autoClavicles.size();++i) {
            const auto &ac=p.autoClavicles[i];const auto row="pose.auto_clavicles["+_N(i)+"]";
            if(!_Index(ac.slot,_slots,false,row,"slot") || !clavicleTargets.insert(ac.slot).second ||
               !_Size(ac.basis.size(),9,row,"basis") || !_Size(ac.frames.size(),13,row,"frames") ||
               !_Size(ac.scalarIndices.size(),11,row,"scalar_indices") || ac.kernel<0 || ac.kernel>1)return _Bad(row+": malformed clavicle record");
            const auto count=ac.widths.size();
            if(ac.swings.size()/4!=count || ac.swings.size()%4 || ac.gains.size()!=count ||
               (count && (ac.weights.size()/count!=count || ac.weights.size()%count)) || (!count && !ac.weights.empty()))return _Bad(row+": invalid swing dimensions");
            for(size_t k=0;k<13;++k) {
                const auto &read=ac.frames[k];const bool optional=k==8 || k==9 || (k>=10 && !ac.hasLimb);
                if(!_Index(read.slot,_slots,optional,row,"frame.slot") || read.computation>2 ||
                   !_Indices(read.recompose,_slots,false,row,"frame.recompose"))return _Bad(row+": invalid frame read");
                int parent=ac.slot;
                for(int child:read.recompose) {
                    if(_f.slotMeta->parent[size_t(child)]!=parent)return _Bad(row+": invalid entering subtree");
                    parent=child;
                }
                if(!read.recompose.empty() && parent!=read.slot)return _Bad(row+": entering subtree ends at wrong provider");
            }
            if(ac.frames[0].slot!=ac.slot)return _Bad(row+": target frame mismatch");
            if(!_Indices(ac.scalarIndices,ac.scalars.size(),true,row,"scalar_indices"))return false;
            for(const auto &input:ac.scalars) {
                if(input.tag!=InputTag::Float && input.tag!=InputTag::Double)return _Bad(row+": scalar type must be float or double");
                if(!_Read(&input,int(input.tag),_Baked,row,"scalars"))return false;
            }
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
                !_ReadPtr(sw.active, int(sw.tokenIndex?InputTag::Token:InputTag::Double), _Baked, row,
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
        for (size_t i = 0; i < p.xformFrames.size(); ++i) {
            if (p.xformFrames[i].flags & ~uint32_t(31))
                return _Bad("pose.xform_frames[" + _N(i) + "]: unknown frame flags");
        }
        if (!_Size(p.xformFrames.size(), _f.slotMeta->xformSlots.size(),
                   "pose", "xform_frames")) return false;
        return _Size(p.xformBase.size(), _f.slotMeta->xformSlots.size(),
                     "pose", "xform_base");
    }

    bool _Ladder(const fb::RigExecWireLadder &ladder, const std::string &row)
    {
        const int matrix = int(InputTag::Matrix4d);
        if(!ladder.spaceValues.empty() && ladder.spaceValues.size()!=7)
            return _Bad(row+": space_values must be empty or contain the seven declared space channels");
        for(int32_t value:ladder.spaceValues)
            if(value < -1 || (value>=0 && (!_f.providerProgram || size_t(value)>=_f.providerProgram->valueKeys.size())))
                return _Bad(row+": space_values references an unknown provider value");
        if (!_ReadPtr(ladder.restSpace, matrix, _Baked, row, "rest_space") ||
            !_ReadPtr(ladder.defaultSpace, matrix, _Baked, row,
                      "default_space") ||
            !_ReadPtr(ladder.posedSpace, matrix, _Baked, row,
                      "posed_space") ||
            !_ReadPtr(ladder.parentSpace,matrix,_Baked,row,"parent_space") ||
            !_ReadPtr(ladder.parentDefaultSpace,matrix,_Baked,row,"parent_default_space") ||
            !_ReadPtr(ladder.avarDefaultSpace,matrix,_Baked,row,"avar_default_space") ||
            !_ReadPtr(ladder.posedDefaultSpace,matrix,_Baked,row,"posed_default_space") ||
            !_ReadPtr(ladder.interveningSpace,matrix,_Baked,row,"intervening_space") ||
            !_ReadPtr(ladder.rotationSign,int(InputTag::Vec3d),_Baked,row,"rotation_sign") ||
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
               (!s.pin || _ReadPtr(s.pin,flt,_Baked,row,"pin")) &&
               (!s.upperScale || _ReadPtr(s.upperScale,dbl,_Baked,row,"upper_scale")) &&
               (!s.lowerScale || _ReadPtr(s.lowerScale,dbl,_Baked,row,"lower_scale")) &&
               (!s.softDistance || _ReadPtr(s.softDistance,flt,_Baked,row,"soft_distance")) &&
               (!s.limbTwist || _ReadPtr(s.limbTwist,flt,_Baked,row,"limb_twist")) &&
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
                        "twist_degrees") &&
               (!c.stretch || _ReadPtr(c.stretch,int(InputTag::Float),_Baked,row,"stretch"));
    }

    bool _ConstraintArrays(const fb::RigExecWireConstraintArrays &a,
                           const std::string &row)
    {
        if (!_PathId(a.prim, _PrimOrZero, row, "prim")) {
            return false;
        }
        if (!_Size(a.rawSlots.size(),4,row,"raw_slots")) return false;
        for(size_t k=0;k<4;++k) {
            const int slot=a.rawSlots[k];
            if(slot < -1 || (slot>=0 && size_t(slot)>=_f.inputs.size()))
                return _Bad(_At(row,"raw_slots",long(k))+": out of range");
            if(slot>=0) {
                const auto expected=(k==1 || k==2)?fb::InputTag::Vec3dArray:fb::InputTag::FloatArray;
                if(_f.inputs[size_t(slot)].type()!=expected)
                    return _Bad(_At(row,"raw_slots",long(k))+": wrong raw array type");
            }
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
        std::set<std::string> refreshKeys;
        for(size_t i=0;i<p.providerRefreshes.size();++i) {
            const auto &refresh=p.providerRefreshes[i];
            const std::string row="pose.provider_refreshes["+_N(i)+"]";
            if(refresh.key.empty() || !refreshKeys.insert(refresh.key).second ||
               refresh.slot<0 || size_t(refresh.slot)>=_slots ||
               refresh.checkpoint>p.walkSteps.size() ||
               !_PathId(refresh.reader,_Prim,row,"reader"))
                return _Bad(row+": invalid or duplicate provider refresh context");
            uint32_t reader=_f.rig;
            if(refresh.checkpoint<p.walkSteps.size()) {
                const auto &walk=p.walkSteps[size_t(refresh.checkpoint)];
                if(walk.solverBatch) {
                    if(walk.batchSolvers.empty() || !_Has(p.solvers,walk.batchSolvers.back()))return _Bad(row+": invalid solver reader context");
                    reader=p.solvers[size_t(walk.batchSolvers.back())].path;
                } else {
                    if(!_Has(p.constraints,walk.index))return _Bad(row+": invalid constraint reader context");
                    reader=p.constraints[size_t(walk.index)].path;
                }
            }
            if(refresh.reader!=reader || refresh.key!="providerRefresh:"+RigExecFormatPathText(_f,reader)+":"+
                std::to_string(refresh.checkpoint)+":"+RigExecFormatPathText(_f,_f.slotMeta->paths[size_t(refresh.slot)]))
                return _Bad(row+": provider refresh key or reader differs from canonical context");
            const auto frames=[&](int32_t slot,uint32_t br,uint32_t fr,uint32_t bw,uint32_t fw) {
                if(slot<0 || size_t(slot)>=_slots || br==bw || fr==fw ||
                   bw<_slots || fw<_slots)return false;
                ++baseWrites[size_t(slot)];++finWrites[size_t(slot)];
                base(br);base(bw);fin(fr);fin(fw);return true;
            };
            if(!frames(refresh.slot,refresh.baseRead,refresh.finRead,refresh.baseWrite,refresh.finWrite))
                return _Bad(row+": refresh target lacks fresh owned frame versions");
            std::set<int32_t> carrySlots;
            for(const auto &carry:refresh.carries) {
                if(carry.slot==refresh.slot || !carrySlots.insert(carry.slot).second ||
                   !frames(carry.slot,carry.baseRead,carry.finRead,carry.baseWrite,carry.finWrite))
                    return _Bad(row+": invalid or duplicate refresh carry");
                int32_t ancestor=carry.slot;
                while(ancestor>=0 && ancestor!=refresh.slot)ancestor=_f.slotMeta->propParent[size_t(ancestor)];
                if(ancestor!=refresh.slot)return _Bad(row+": refresh carry is outside its provider subtree");
                std::set<int32_t> blockers;
                for(int32_t blocker:carry.blockingSlots)
                    if(blocker<0 || size_t(blocker)>=_slots || !blockers.insert(blocker).second)
                        return _Bad(row+": invalid or duplicate carry blocker");
                std::vector<int32_t> expectedBlockers;
                for(int32_t at=carry.slot;at>=0 && at!=refresh.slot;at=_f.slotMeta->propParent[size_t(at)])
                    if(_f.slotMeta->slotKind[size_t(at)]==fb::SlotKind::FirstFramePose)expectedBlockers.push_back(at);
                if(carry.blockingSlots!=expectedBlockers)return _Bad(row+": carry blocker ancestry differs");
            }
            std::set<std::pair<uint32_t,uint32_t>> guards;
            for(const auto &guard:refresh.priorConstraints)
                if(guard.first>=refresh.checkpoint || guard.first>=p.commits.size() ||
                   p.commits[guard.first].solverOutput || guard.second>=p.commits[guard.first].slots.size() ||
                   p.commits[guard.first].slots[guard.second]!=refresh.slot ||
                   !guards.emplace(guard.first,guard.second).second)
                    return _Bad(row+": invalid or duplicate prior constraint guard");
        }
        for(const auto &frame:p.providerFrameInputs) {
            if(frame.base)base(frame.version);else fin(frame.version);
        }
        std::map<uint32_t,int32_t> baseOwners,finOwners;
        for(size_t slot=0;slot<_slots;++slot) {
            baseOwners[uint32_t(slot)]=finOwners[uint32_t(slot)]=int32_t(slot);
            if(baseWrites[slot])baseOwners[uint32_t(_slots+slot)]=int32_t(slot);
            if(finWrites[slot])finOwners[uint32_t(_slots+slot)]=int32_t(slot);
        }
        const auto assign=[&](auto &owners,uint32_t version,int32_t slot) {
            const auto found=owners.find(version);
            if(found!=owners.end() && found->second!=slot)return false;
            owners[version]=slot;return true;
        };
        for(const auto &commit:p.commits) {
            for(size_t k=0;k<commit.slots.size();++k) {
                if(k<commit.slotWrites.size() && !assign(finOwners,commit.slotWrites[k],commit.slots[k]))return _Bad("pose: frame version has multiple provider owners");
                if(commit.solverOutput && k<commit.slotBaseWrites.size() && !assign(baseOwners,commit.slotBaseWrites[k],commit.slots[k]))return _Bad("pose: frame version has multiple provider owners");
            }
            for(size_t k=0;k<commit.propagate.size();++k) {
                if(k<commit.descendantWrites.size() && !assign(finOwners,commit.descendantWrites[k],commit.propagate[k].first))return _Bad("pose: frame version has multiple provider owners");
                if(commit.solverOutput && k<commit.descendantBaseWrites.size() && !assign(baseOwners,commit.descendantBaseWrites[k],commit.propagate[k].first))return _Bad("pose: frame version has multiple provider owners");
            }
        }
        for(const auto &refresh:p.providerRefreshes) {
            if(!assign(baseOwners,refresh.baseWrite,refresh.slot) || !assign(finOwners,refresh.finWrite,refresh.slot))return _Bad("pose: refresh frame belongs to another provider");
            for(const auto &carry:refresh.carries)
                if(!assign(baseOwners,carry.baseWrite,carry.slot) || !assign(finOwners,carry.finWrite,carry.slot))return _Bad("pose: refresh carry frame belongs to another provider");
        }
        const auto owned=[&](const auto &owners,uint32_t version,int32_t slot) {
            const auto found=owners.find(version);return found!=owners.end() && found->second==slot;
        };
        for(const auto &refresh:p.providerRefreshes) {
            if(!owned(baseOwners,refresh.baseRead,refresh.slot) || !owned(finOwners,refresh.finRead,refresh.slot))return _Bad("pose: refresh entering frame belongs to another provider");
            for(const auto &carry:refresh.carries)
                if(!owned(baseOwners,carry.baseRead,carry.slot) || !owned(finOwners,carry.finRead,carry.slot))return _Bad("pose: refresh carry entering frame belongs to another provider");
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
        _basePool = basePool;
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
            // A Whole revision of a chain with groups (format 20) owns, past
            // its chunks, one RevisionOut id per group, which its fuse
            // publishes; the chain's chunk range holds every id it owns.
            const size_t groups = RigExecFormatChainGroups(_f, c);
            int64_t owned = 0;
            for (size_t r = 0; r < chain.revisions.size(); ++r, ++id) {
                const fb::RigExecWireRevision &revision = chain.revisions[r];
                const size_t count =
                    revision.chunks.size() +
                    (groups > 0 && !RigExecFormatIsRangeRevision(revision)
                         ? groups
                         : 0);
                if (g.revisionIndex[id].first != int32_t(c) ||
                    g.revisionIndex[id].second != int32_t(r) ||
                    g.revisionChunkCount[id] != int32_t(count) ||
                    g.revisionChunkBase[id] != revision.chunkBase) {
                    return _Bad(row + ": revision_index or chunk tables "
                                      "disagree at revision " +
                                _N(id));
                }
                owned += int64_t(count);
            }
            if (groups > 0 && int64_t(g.chainChunkEnd[c]) -
                                      int64_t(g.chainChunkBegin[c]) !=
                                  owned) {
                return _Bad(row + ": chain_chunk range of chain " + _N(c) +
                            " does not hold its revisions' " +
                            _N(size_t(owned)) + " RevisionOut ids");
            }
            for (size_t d = 0; d < chain.derived.size(); ++d, ++derivedId) {
                if (g.derivedIndex[derivedId].first != int32_t(c) ||
                    g.derivedIndex[derivedId].second != int32_t(d)) {
                    return _Bad(row + ": derived_index disagrees at " +
                                _N(derivedId));
                }
            }
        }
        // The group writers, from the revision ranges checked above; the
        // step rules, the constants and the body reads all read them.
        _groupWritten.assign(_chains, {});
        _groupEntering.assign(_chains, {});
        for (size_t c = 0; c < _chains; ++c) {
            RigExecFormatGroupWriters(_f, c, &_groupWritten[c],
                                      &_groupEntering[c]);
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
        return _PathReads(g) && _RangeSteps() && _GroupConstants();
    }

    /// The steps of every chain with groups (format 20) declare what their
    /// bodies read and write, as the program's ValidatePointVersions holds
    /// them. Group k of the version entering revision r is the RevisionOut
    /// slot of the last earlier revision that writes k (a Range one's
    /// chunk_base + k, a Whole one's chunk_base + chunks + k), else the
    /// chain base (RigExecFormatGroupWriters). Every step reads its packet
    /// and the chain base. A Range revision's group step for part k, at
    /// most one per part, reads group k of its entering version and no
    /// other RevisionOut, a Range Skin's also its transforms, and never
    /// that version's RevisionDone or ChainDirty; its join reads its
    /// transforms, its weight packet when it has one, the slots of its
    /// written parts and none other of its own, and the entering version
    /// (which keeps the joins in chain order), and writes no RevisionOut. A
    /// Whole keyed skin has one chunk step per part, each reading group k
    /// of its entering version and no other RevisionOut; any other Whole
    /// revision has one step per chunk, each reading the entering version
    /// (a skin also its transforms). A Whole fuse reads its transforms, its
    /// weight packet when it has one, its chunks, the entering version and
    /// group k of it for every k, and writes every group it publishes.
    bool _RangeSteps()
    {
        using D = fb::SlotDomain;
        const fb::RigExecWireDomainGeometry &g = *_f.geometry;
        // RevisionChunk steps per (revision id, part) of chains with groups.
        std::map<std::pair<uint64_t, int64_t>, size_t> chunkSteps;
        for (size_t i = 0; i < _f.steps.size(); ++i) {
            const fb::RigExecWireStep &step = _f.steps[i];
            const bool chunk = step.kind == fb::StepKind::RevisionChunk;
            if ((!chunk && step.kind != fb::StepKind::RevisionFuse) ||
                !_Has(g.revisionIndex, step.object)) {
                continue;
            }
            const RigExecWireIntPair &at = g.revisionIndex[size_t(step.object)];
            const size_t c = size_t(at.first);
            const size_t r = size_t(at.second);
            const std::vector<std::vector<char>> &written = _groupWritten[c];
            if (written.empty()) {
                continue;
            }
            const size_t groups = written[r].size();
            const fb::RigExecWireChain &chain = g.chains[c];
            const fb::RigExecWireRevision &revision = chain.revisions[r];
            const bool range = RigExecFormatIsRangeRevision(revision);
            const bool keyed = RigExecFormatIsKeyedRevision(revision);
            const bool skin = revision.op == uint8_t(fb::RevisionOp::Skin);
            const uint64_t id = uint64_t(step.object);
            const std::string name = _StepName(i);
            const auto need = [&](D domain, uint64_t slot) {
                return _Declares(i, domain, slot) ||
                       _Bad(name + " does not declare " +
                            rigExecStepGraphDetail::DomainName(
                                uint8_t(domain)) +
                            "[" + _N(slot) + "]");
            };
            // Group k as revision w of the chain publishes it.
            const auto groupSlot = [&](size_t w, size_t k) {
                const fb::RigExecWireRevision &writer = chain.revisions[w];
                return uint64_t(int64_t(writer.chunkBase) +
                                int64_t(RigExecFormatIsRangeRevision(writer)
                                            ? 0
                                            : writer.chunks.size()) +
                                int64_t(k));
            };
            // Group k of the version entering this revision: false for the
            // chain base, else true with its slot.
            const auto enteringSlot = [&](size_t k, uint64_t *slot) {
                const int w = _groupEntering[c][r][k];
                if (w >= 0) {
                    *slot = groupSlot(size_t(w), k);
                }
                return w >= 0;
            };
            const auto needVersion = [&]() {
                return r == 0 || (need(D::RevisionDone, id - 1) &&
                                  need(D::ChainDirty, id - 1));
            };
            const auto readsVersion = [&]() {
                return r > 0 && (_Declares(i, D::RevisionDone, id - 1) ||
                                 _Declares(i, D::ChainDirty, id - 1));
            };
            if (!need(D::RevisionPacket, id) ||
                !need(D::ChainBase, uint64_t(c))) {
                return false;
            }
            if (chunk) {
                if (!_Has(revision.chunks, step.part)) {
                    return _Bad(name + (range ? ": range part " : ": part ") +
                                std::to_string(step.part) + " of " +
                                _N(revision.chunks.size()));
                }
                if (++chunkSteps[{id, int64_t(step.part)}] > 1) {
                    return _Bad(name + ": a second step for part " +
                                std::to_string(step.part));
                }
                if (!range && !keyed) {
                    if (!needVersion() ||
                        (skin && !need(D::RevisionTransforms, id))) {
                        return false;
                    }
                    continue;
                }
                // A group step: group `part` of the entering version, and
                // no other RevisionOut slot.
                const size_t k = size_t(step.part);
                uint64_t want = 0;
                const bool slotted = enteringSlot(k, &want);
                if (slotted && !need(D::RevisionOut, want)) {
                    return false;
                }
                for (const fb::SlotRange &read : step.reads) {
                    if (read.domain() != D::RevisionOut ||
                        read.begin() >= read.end() ||
                        (slotted && read.begin() == want &&
                         read.end() == want + 1)) {
                        continue;
                    }
                    const uint64_t other =
                        slotted && read.begin() == want ? want + 1
                                                        : read.begin();
                    return _Bad(name + " reads RevisionOut[" + _N(other) +
                                "], which is not group " + _N(k) +
                                " of the version entering it");
                }
                if (range && skin && !need(D::RevisionTransforms, id)) {
                    return false;
                }
                if (range && readsVersion()) {
                    return _Bad(name + " declares point version " + _N(r) +
                                " of chain " + _N(c) +
                                ", which a range-pipelined range step does "
                                "not read");
                }
                continue;
            }
            // The fuse: a Range revision's join, or a Whole revision's.
            if (!need(D::RevisionTransforms, id) ||
                (revision.weightObject >= 0 &&
                 !need(D::WeightPacket, uint64_t(revision.weightObject))) ||
                !needVersion()) {
                return false;
            }
            const uint64_t base = uint64_t(int64_t(revision.chunkBase));
            if (range) {
                for (size_t k = 0; k < groups; ++k) {
                    if (written[r][k] && !need(D::RevisionOut, base + k)) {
                        return false;
                    }
                    if (!written[r][k] &&
                        _Declares(i, D::RevisionOut, base + k)) {
                        return _Bad(name + " reads RevisionOut[" +
                                    _N(base + k) + "] of part " + _N(k) +
                                    ", which no step writes");
                    }
                }
                for (const fb::SlotRange &write : step.writes) {
                    if (write.domain() == D::RevisionOut &&
                        write.begin() < write.end()) {
                        return _Bad(name + " writes RevisionOut[" +
                                    _N(write.begin()) +
                                    "], but a join publishes no points");
                    }
                }
                continue;
            }
            const size_t chunks = revision.chunks.size();
            for (size_t k = 0; k < chunks; ++k) {
                if (!need(D::RevisionOut, base + k)) {
                    return false;
                }
            }
            for (size_t k = 0; k < groups; ++k) {
                uint64_t slot = 0;
                if (enteringSlot(k, &slot) && !need(D::RevisionOut, slot)) {
                    return false;
                }
                const uint64_t published = base + chunks + k;
                const bool writes = std::any_of(
                    step.writes.begin(), step.writes.end(),
                    [&](const fb::SlotRange &write) {
                        return write.domain() == D::RevisionOut &&
                               write.begin() <= published &&
                               published < write.end();
                    });
                if (!writes) {
                    return _Bad(name + " does not write RevisionOut[" +
                                _N(published) + "], group " + _N(k) +
                                " of the version it publishes");
                }
            }
        }
        // Every part of a Whole revision has its chunk step; a Range
        // revision's missing parts are its gated groups (_GroupConstants).
        for (size_t c = 0; c < _chains; ++c) {
            const fb::RigExecWireChain &chain = g.chains[c];
            if (_groupWritten[c].empty()) {
                continue;
            }
            for (size_t r = 0; r < chain.revisions.size(); ++r) {
                const fb::RigExecWireRevision &revision = chain.revisions[r];
                if (RigExecFormatIsRangeRevision(revision)) {
                    continue;
                }
                const uint64_t id = uint64_t(g.chainRevisionBegin[c]) + r;
                for (size_t k = 0; k < revision.chunks.size(); ++k) {
                    if (!chunkSteps.count({id, int64_t(k)})) {
                        return _Bad("geometry.chains[" + _N(c) +
                                    "].revisions[" + _N(r) +
                                    "]: no RevisionChunk step for part " +
                                    _N(k) + " of a Whole revision");
                    }
                }
            }
        }
        return true;
    }

    /// Whether input slot \p slot is private and unanimated: no integration
    /// can set it, and its default is its value at every time.
    bool _PrivateConstant(uint32_t slot) const
    {
        const uint8_t flags = _f.inputs[slot].flags();
        return (flags & (uint8_t(fb::InputSlotFlags::Listed) |
                         uint8_t(fb::InputSlotFlags::Animated))) == 0;
    }

    /// Whether \p value is a Float holding 0 (either sign).
    static bool _FloatZero(const fb::RigExecWireValue &value)
    {
        if (value.tag != InputTag::Float) {
            return false;
        }
        const uint32_t bits = uint32_t(value.bits);
        float f = 1.0f;
        std::memcpy(&f, &bits, sizeof f);
        return f == 0.0f;
    }

    /// What the Range roles and gates of a chain with groups rest on, as
    /// private constants (format 20). A Range Skin's rigExec:skinningMethod
    /// and rigExec:elementSize reads are one hop over a private, unanimated
    /// slot (or a static value), the method classicLinear, and its
    /// joint_indices_slot is private and unanimated; a Range lattice's
    /// rigExec:evaluation (format 21) is likewise one, holding legacy. A
    /// Range revision that writes fewer parts than the chain has groups is
    /// gated: its weight object is a static sparse weight, neither
    /// current-phase nor operation-domain, whose default weight reads one
    /// private, unanimated slot holding 0 (its Baked constant 0 too) and
    /// whose indices are a private slot, every index of whose default
    /// inside the chain's points lies in a written part.
    bool _GroupConstants()
    {
        const fb::RigExecWireDomainGeometry &g = *_f.geometry;
        for (size_t c = 0; c < _chains; ++c) {
            const std::vector<std::vector<char>> &written = _groupWritten[c];
            if (written.empty()) {
                continue;
            }
            const fb::RigExecWireChain &chain = g.chains[c];
            for (size_t r = 0; r < chain.revisions.size(); ++r) {
                const fb::RigExecWireRevision &revision = chain.revisions[r];
                if (!RigExecFormatIsRangeRevision(revision)) {
                    continue;
                }
                const std::string row = "geometry.chains[" + _N(c) +
                                        "].revisions[" + _N(r) + "]";
                if (revision.op == uint8_t(fb::RevisionOp::Skin) &&
                    !_RangeSkinConstants(revision, row)) {
                    return false;
                }
                if (revision.op == uint8_t(fb::RevisionOp::Lattice) &&
                    !_RangeLatticeConstants(revision, row)) {
                    return false;
                }
                const size_t groups = written[r].size();
                const size_t count = size_t(
                    std::count(written[r].begin(), written[r].end(), 1));
                if (count == groups) {
                    continue;
                }
                const std::string gated = row + ": writes " + _N(count) +
                                          " of " + _N(groups) + " groups";
                if (revision.weightObject < 0) {
                    return _Bad(gated + ", but has no weight object to "
                                        "gate on");
                }
                const fb::RigExecWireWeightObject &w =
                    g.weightObjects[size_t(revision.weightObject)];
                const std::string object =
                    " weight object " + _N(size_t(revision.weightObject));
                if (RigExecFormatPathText(_f, w.type) !=
                        "RigExecStaticWeight" ||
                    RigExecFormatPathText(_f, w.representation) != "sparse") {
                    return _Bad(gated + ", but" + object +
                                " is not a static sparse weight");
                }
                if (revision.weightCurrentPhase ||
                    revision.weightOperationDomain) {
                    return _Bad(gated + ", but its weight is current-phase "
                                        "or operation-domain");
                }
                const RigExecWireInput *d = w.defaultWeight.get();
                const uint8_t avoid =
                    uint8_t(fb::InputReadFlags::Varying) |
                    uint8_t(fb::InputReadFlags::ViaChain);
                if (!d || d->walk.size() != 1 || (d->flags & avoid) != 0 ||
                    !_PrivateConstant(d->walk[0]) ||
                    !_FloatZero(_DefaultOf(int32_t(d->walk[0]))) ||
                    (d->mode == fb::ReadMode::Baked &&
                     !_FloatZero(_f.values[d->constant]))) {
                    return _Bad(gated + ", but" + object +
                                "'s default weight is not one private, "
                                "unanimated slot holding 0");
                }
                if (w.indicesSlot < 0 ||
                    (_f.inputs[size_t(w.indicesSlot)].flags() &
                     uint8_t(fb::InputSlotFlags::Listed)) != 0) {
                    return _Bad(gated + ", but" + object +
                                "'s indices are not a private slot");
                }
                const int64_t points = revision.chunks.back().end;
                const std::vector<int32_t> &indices =
                    _f.intArrays[_DefaultOf(w.indicesSlot).array].v;
                for (const int32_t index : indices) {
                    if (index < 0 || int64_t(index) >= points) {
                        continue;
                    }
                    size_t k = 0;
                    while (k + 1 < groups &&
                           int64_t(revision.chunks[k].end) <= index) {
                        ++k;
                    }
                    if (!written[r][k]) {
                        return _Bad(gated + ", but index " +
                                    std::to_string(index) + " of" + object +
                                    " lies in part " + _N(k) +
                                    ", which no step writes");
                    }
                }
            }
        }
        return true;
    }

    /// A Range Skin's method, element size and joint indices as private
    /// constants, the method classicLinear (_GroupConstants).
    bool _RangeSkinConstants(const fb::RigExecWireRevision &revision,
                             const std::string &row)
    {
        const fb::RigExecWireDomainGeometry &g = *_f.geometry;
        const std::string mover = RigExecFormatPathText(_f, revision.moverPath);
        for (const char *attribute :
             {"rigExec:skinningMethod", "rigExec:elementSize"}) {
            const bool method =
                std::strcmp(attribute, "rigExec:skinningMethod") == 0;
            const std::string path = mover + "." + attribute;
            for (const fb::RigExecWirePathRead &read : g.pathReads) {
                if (read.rest ||
                    RigExecFormatPathText(_f, read.path) != path) {
                    continue;
                }
                std::string text = "classicLinear";
                if (read.read) {
                    const uint32_t head =
                        read.read->walk.empty() ? 0 : read.read->walk[0];
                    if (read.read->walk.size() != 1 ||
                        !_PrivateConstant(head)) {
                        return _Bad(row + ": a Range Skin's " + attribute +
                                    " is not one private, unanimated slot");
                    }
                    const fb::RigExecWireValue &value =
                        _DefaultOf(int32_t(head));
                    if (method) {
                        text = value.tag == InputTag::Token
                                   ? RigExecFormatPathText(
                                         _f, uint32_t(value.bits))
                                   : std::string();
                    }
                } else if (method && read.value) {
                    text = read.value->tag == fb::PathTag::Token
                               ? RigExecFormatPathText(
                                     _f, uint32_t(read.value->bits))
                               : std::string();
                }
                if (method && text != "classicLinear") {
                    return _Bad(row + ": a Range Skin's "
                                      "rigExec:skinningMethod holds '" +
                                text + "', not classicLinear");
                }
            }
        }
        if (revision.jointIndicesSlot >= 0 &&
            !_PrivateConstant(uint32_t(revision.jointIndicesSlot))) {
            return _Bad(_At(row, "joint_indices_slot") +
                        ": a Range Skin's joint indices are listed or "
                        "animated");
        }
        return true;
    }

    /// A Range lattice's rigExec:evaluation reads (format 21): one hop over
    /// a private, unanimated slot (or a static value) holding legacy, the
    /// one evaluation with a group form.
    bool _RangeLatticeConstants(const fb::RigExecWireRevision &revision,
                                const std::string &row)
    {
        const std::string path =
            RigExecFormatPathText(_f, revision.moverPath) +
            ".rigExec:evaluation";
        for (const fb::RigExecWirePathRead &read : _f.geometry->pathReads) {
            if (read.rest || RigExecFormatPathText(_f, read.path) != path) {
                continue;
            }
            std::string text = "legacy";
            if (read.read) {
                const uint32_t head =
                    read.read->walk.empty() ? 0 : read.read->walk[0];
                if (read.read->walk.size() != 1 || !_PrivateConstant(head)) {
                    return _Bad(row + ": a Range lattice's "
                                      "rigExec:evaluation is not one "
                                      "private, unanimated slot");
                }
                const fb::RigExecWireValue &value = _DefaultOf(int32_t(head));
                text = value.tag == InputTag::Token
                           ? RigExecFormatPathText(_f, uint32_t(value.bits))
                           : std::string();
            } else if (read.value && read.value->tag == fb::PathTag::Token) {
                text = RigExecFormatPathText(_f, uint32_t(read.value->bits));
            }
            if (text != "legacy") {
                return _Bad(row + ": a Range lattice's rigExec:evaluation "
                                  "holds '" + text + "', not legacy");
            }
        }
        return true;
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
    /// default is the base itself. Its Range revisions and, when it has
    /// any, its Whole keyed skins all cut the chain's one group partition,
    /// so that a group step for part k reads group k of what came before.
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
        // The chain's group partition is its first Range revision's; every
        // other Range revision and every Whole keyed skin cuts the same
        // groups.
        const fb::RigExecWireRevision *partition = nullptr;
        for (const fb::RigExecWireRevision &revision : chain.revisions) {
            if (RigExecFormatIsRangeRevision(revision)) {
                partition = &revision;
                break;
            }
        }
        for (size_t r = 0; partition && r < chain.revisions.size(); ++r) {
            const fb::RigExecWireRevision &revision = chain.revisions[r];
            if (&revision == partition ||
                (!RigExecFormatIsKeyedRevision(revision) &&
                 !RigExecFormatIsRangeRevision(revision))) {
                continue;
            }
            bool same = revision.chunks.size() == partition->chunks.size();
            for (size_t k = 0; same && k < revision.chunks.size(); ++k) {
                same = revision.chunks[k].begin == partition->chunks[k].begin &&
                       revision.chunks[k].end == partition->chunks[k].end;
            }
            if (!same) {
                return _Bad(row + ".revisions[" + _N(r) +
                            "]: its point ranges differ from revisions[" +
                            _N(size_t(partition - chain.revisions.data())) +
                            "]'s, the chain's partition");
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
        if (!_LayoutSlots(r, derived, row, id) ||
            !_ExtendedSettings(r, row)) {
            return false;
        }
        return derived || _Chunks(r, row);
    }

    /// Whether path id \p path is one of the \p count properties \p names
    /// of prim \p prim.
    bool _IsPropertyOf(uint32_t path, uint32_t prim,
                       const char *const *names, size_t count) const
    {
        if (path == 0 || path >= _f.paths.size()) {
            return false;
        }
        const fb::PathNode &node = _f.paths[path];
        if (node.kind() != PathKind::Property || node.parent() != prim ||
            node.name() >= _f.names.size()) {
            return false;
        }
        const std::string &name = _f.names[node.name()];
        for (size_t k = 0; k < count; ++k) {
            if (name == names[k]) {
                return true;
            }
        }
        return false;
    }

    /// The format-21 settings of a Delta Mush, lattice or surface revision
    /// \p r. Format 21: its frame providers, as many as the op takes.
    /// Format 20: none of them -- no influence, no path read and no leaf
    /// site of a settings attribute of its mover -- so the file plays the
    /// legacy deformer.
    bool _ExtendedSettings(const fb::RigExecWireRevision &r,
                           const std::string &row)
    {
        size_t count = 0;
        const char *const *names =
            RigExecFormatExtendedSettingNames(r.op, &count);
        if (!names) {
            return true;
        }
        const char *op = fb::EnumNameRevisionOp(fb::RevisionOp(r.op));
        if (_f.formatVersion >= RigExecFormatExtendedSettingsVersion) {
            if (!RigExecFormatExtendedInfluencesValid(
                    r.op, r.influenceSlots.size())) {
                return _Bad(_At(row, "influence_slots") + ": " +
                            _N(r.influenceSlots.size()) +
                            " frame providers on a " + op + " revision");
            }
            return true;
        }
        if (!r.influenceSlots.empty()) {
            return _Bad(_At(row, "influence_slots") + ": format " +
                        _N(_f.formatVersion) + " holds no frame providers "
                        "on a " + op + " revision");
        }
        if (r.moverPrim == 0) {
            return true;
        }
        const auto refuse = [&](uint32_t path, const std::string &where) {
            return _Bad(where + ": format " + _N(_f.formatVersion) +
                        " holds no " + op + " setting " +
                        RigExecFormatPathText(_f, path));
        };
        for (const auto *sites : {&r.leafSites, &r.layoutLeafSites}) {
            for (size_t k = 0; k < sites->size(); ++k) {
                const uint32_t path = (*sites)[k].path;
                if (_IsPropertyOf(path, r.moverPrim, names, count)) {
                    return refuse(path, row + (sites == &r.leafSites
                                                   ? ".leaf_sites["
                                                   : ".layout_leaf_sites[") +
                                            _N(k) + "]");
                }
            }
        }
        const auto &reads = _f.geometry->pathReads;
        for (size_t i = 0; i < reads.size(); ++i) {
            if (_IsPropertyOf(reads[i].path, r.moverPrim, names, count)) {
                return refuse(reads[i].path,
                              "geometry.path_reads[" + _N(i) + "]");
            }
        }
        return true;
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
    /// partition's element size and index count. A Range Skin (format 20,
    /// unchunked with two or more chunks) is keyed the same way, its ranges
    /// its chain's groups, none empty. Any other unchunked revision has no
    /// key and is at most one range, unless it is a Range revision: a
    /// Matrix, Wire, Lattice or BlendShape revision with no partition
    /// producer set, whose two or more non-empty ranges tile its chain's
    /// points from 0, one group each.
    bool _Chunks(const fb::RigExecWireRevision &r, const std::string &row)
    {
        const bool keyed = RigExecFormatIsKeyedRevision(r);
        using ProducerKey=std::pair<uint8_t,uint32_t>;
        std::set<std::vector<ProducerKey>> distinct;
        size_t minimum=SIZE_MAX,maximum=0;
        for (size_t set=0;set<r.partitionProducerSets.size();++set) {
            std::vector<ProducerKey> keys;
            for (const auto &value:r.partitionProducerSets[set].values) {
                if (value.end()!=uint64_t(value.begin())+1 ||
                    !_GraphValueSlot(uint32_t(value.domain()),value.begin()))
                    return _Bad(row+": partition producer set has invalid typed identity");
                keys.emplace_back(uint8_t(value.domain()),value.begin());
            }
            if (!std::is_sorted(keys.begin(),keys.end()) ||
                std::adjacent_find(keys.begin(),keys.end())!=keys.end())
                return _Bad(row+": partition producer set is not sorted and unique");
            minimum=std::min(minimum,keys.size()); maximum=std::max(maximum,keys.size());
            distinct.insert(keys);
            if(keyed) {
                if(set>=r.chunks.size()) return _Bad(row+": partition producer set has no natural chunk");
                std::vector<ProducerKey> expected;
                const auto domain=r.finalPhase?fb::SlotDomain::FinalMatrix:fb::SlotDomain::BaseMatrix;
                for(int position:r.chunks[set].key) {
                    if(position<0 || size_t(position)>=r.influenceSlots.size())
                        return _Bad(row+": partition producer influence position exceeds bindings");
                    const int slot=r.influenceSlots[size_t(position)];
                    if(slot>=0) expected.emplace_back(uint8_t(domain),uint32_t(slot));
                }
                std::sort(expected.begin(),expected.end());
                expected.erase(std::unique(expected.begin(),expected.end()),expected.end());
                if(keys!=expected) return _Bad(row+": partition producer set differs from actual influence bindings");
            }
        }
        if(minimum==SIZE_MAX) minimum=0;
        if(r.partitionProducerMin<0 || r.partitionProducerMax<0 ||
            uint64_t(r.partitionProducerMin)!=minimum || uint64_t(r.partitionProducerMax)!=maximum ||
            r.partitionDistinctReads!=distinct.size() ||
            (keyed && r.partitionProducerSets.size()!=r.chunks.size()))
            return _Bad(row+": partition producer summary differs from exact natural sets");

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
        const bool ranges = RigExecFormatIsRangeRevision(r);
        if (!keyed) {
            if (ranges && !RigExecFormatIsRangeOp(r.op)) {
                return _Bad(row + ": " + _N(r.chunks.size()) +
                            " chunks, but not chunked");
            }
            for (size_t k = 0; k < r.chunks.size(); ++k) {
                if (!r.chunks[k].key.empty()) {
                    return _Bad(_At(row, "chunks", long(k)) +
                                ": a key on an unchunked revision");
                }
            }
            if (!ranges) {
                return true;
            }
            if (!r.partitionProducerSets.empty()) {
                return _Bad(row + ": partition producer sets on a "
                                  "range-pipelined revision");
            }
            int64_t at = 0;
            for (size_t k = 0; k < r.chunks.size(); ++k) {
                const fb::RigExecWireChunk &chunk = r.chunks[k];
                if (chunk.begin != at || chunk.end <= chunk.begin) {
                    return _Bad(_At(row, "chunks", long(k)) +
                                ": point range [" +
                                std::to_string(chunk.begin) + ", " +
                                std::to_string(chunk.end) +
                                ") is empty or does not continue the "
                                "chain's point partition at " +
                                std::to_string(at));
                }
                at = chunk.end;
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
            if (ranges && chunk.end == chunk.begin) {
                return _Bad(where + ": vertex range " + range +
                            " is an empty group of a Range Skin");
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
            for (const auto &entry : {
                     std::make_pair(w.oraclePlaneAxisSlot, "rigExec:planeAxis"),
                     std::make_pair(w.oraclePlaneBoundsSlot, "rigExec:planeBounds")}) {
                const int32_t slot = entry.first;
                if (!_InputSlotOf(slot, InputTag::Token, row, entry.second)) return false;
                if (slot >= 0 && (!_SlotIs(slot, w.path, entry.second) ||
                    (_f.inputs[size_t(slot)].flags() & uint8_t(fb::InputSlotFlags::Listed))))
                    return _Bad(row + ": oracle token must be its private raw attribute slot");
            }
            // Dependency order bounds the oracle's recursion.
            if (!_Index(w.base, _weights, true, row, "base") ||
                !_Indices(w.inputs, _weights, false, row, "inputs")) {
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

    // Record only reads reconstructed from validated body fields, never extra declarations.
    bool _BodyRead(size_t step, fb::SlotDomain domain, uint64_t slot)
    {
        if (!_Declares(step,domain,slot)) return false;
        _bodyReads[step].emplace(domain,uint32_t(slot));
        return true;
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
            if (!_BodyRead(reader, fb::SlotDomain::PoseFin, record.version())) {
                return _Bad(step() + " does not declare PoseFin[" +
                            _N(record.version()) + "]");
            }
            if (!_BodyRead(reader, fb::SlotDomain::CommitTable,
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
            return _BodyRead(step, fb::SlotDomain::FrameMatrix, k) ||
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
                return _BodyRead(step, domain, slot) ||
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
            for(size_t k=0;k<mover.declaredInputs.size();++k) {
                const auto &declaration=mover.declaredInputs[k];
                const auto at=_At(row,"declared_inputs",long(k));
                if(declaration.time > fb::ExternalInputTime::AtDefault || declaration.flavour > fb::ExternalInputFlavour::Present)
                    return _Bad(at+": invalid read time/flavour");
                if(!_ReadPtr(declaration.read,declaration.read ? int(declaration.read->tag) : _AnyTag,_AnyMode,at,"read")) return false;
                const bool raw=declaration.flavour == fb::ExternalInputFlavour::Raw || declaration.flavour == fb::ExternalInputFlavour::OverlayThenRaw || declaration.flavour == fb::ExternalInputFlavour::Present;
                if(raw && declaration.read->mode!=ReadMode::Raw)
                    return _Bad(at+": raw/present declaration requires raw read metadata");
                if(!raw && declaration.read->mode!=ReadMode::Resolved)
                    return _Bad(at+": resolved declaration requires resolved read metadata");
                if(declaration.read->overrideIndex>=0)
                    return _Bad(at+": external declaration cannot own a native interactive override");
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

    bool _sourceBackedIndexed=false;
    std::unordered_set<const RigExecWireInput *> _sourceBackedOwners;
    std::vector<uint32_t> _bodyFinLast,_bodyBaseLast;
    std::map<size_t,std::set<std::pair<fb::SlotDomain,uint32_t>>> _bodyReads;
    /// Per chain, RigExecFormatGroupWriters' tables (empty without groups).
    std::vector<std::vector<std::vector<char>>> _groupWritten;
    std::vector<std::vector<std::vector<int>>> _groupEntering;
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
    uint64_t _basePool = 0;
    std::vector<uint32_t> _overrideUses;
    std::vector<std::pair<const RigExecWireInput *, std::string>> _readUses;
    std::set<const RigExecWireInput *> _sampledLeafReads;
    std::set<const RigExecWireInput *> _boundWalkReads;
    std::map<const RigExecWireInput *, const std::vector<int32_t> *> _oracleScalarContexts;
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

size_t
RigExecFormatChainGroups(const fb::RigExecWireFile &file, size_t chain)
{
    if (!file.geometry || chain >= file.geometry->chains.size()) {
        return 0;
    }
    for (const fb::RigExecWireRevision &revision :
         file.geometry->chains[chain].revisions) {
        if (RigExecFormatIsRangeRevision(revision)) {
            return revision.chunks.size();
        }
    }
    return 0;
}

void
RigExecFormatGroupWriters(const fb::RigExecWireFile &file, size_t chain,
                          std::vector<std::vector<char>> *written,
                          std::vector<std::vector<int>> *enteringWriter)
{
    written->clear();
    enteringWriter->clear();
    const size_t groups = RigExecFormatChainGroups(file, chain);
    if (groups == 0) {
        return;
    }
    const fb::RigExecWireDomainGeometry &g = *file.geometry;
    const std::vector<fb::RigExecWireRevision> &revisions =
        g.chains[chain].revisions;
    const size_t count = revisions.size();
    written->assign(count, std::vector<char>(groups, 0));
    enteringWriter->assign(count, std::vector<int>(groups, -1));
    for (size_t r = 0; r < count; ++r) {
        if (!RigExecFormatIsRangeRevision(revisions[r])) {
            (*written)[r].assign(groups, 1);
        }
    }
    // A Range revision writes the parts its RevisionChunk steps name; the
    // chain's revisions take flat ids from its chain_revision_begin. Range-
    // checked, so an unvalidated file only yields fewer written parts.
    if (chain < g.chainRevisionBegin.size() && g.chainRevisionBegin[chain] >= 0) {
        const int64_t first = g.chainRevisionBegin[chain];
        for (const fb::RigExecWireStep &step : file.steps) {
            const int64_t r = int64_t(step.object) - first;
            if (step.kind != fb::StepKind::RevisionChunk || r < 0 ||
                uint64_t(r) >= count || step.part < 0 ||
                uint64_t(step.part) >= groups ||
                !RigExecFormatIsRangeRevision(revisions[size_t(r)])) {
                continue;
            }
            (*written)[size_t(r)][size_t(step.part)] = 1;
        }
    }
    for (size_t k = 0; k < groups; ++k) {
        int last = -1;
        for (size_t r = 0; r < count; ++r) {
            (*enteringWriter)[r][k] = last;
            if ((*written)[r][k]) {
                last = int(r);
            }
        }
    }
}

namespace {

// The settings attributes format 21 adds, per op, in assembly order.
constexpr const char *_kDeltaMushSettings[] = {
    "inputs:smoothing", "inputs:frameTransport", "inputs:smoothWeights",
    "inputs:edges", "inputs:onlySmooth", "inputs:computationToTarget"};
constexpr const char *_kLatticeSettings[] = {
    "rigExec:evaluation", "rigExec:interpolationU", "rigExec:interpolationV",
    "rigExec:interpolationW", "rigExec:origin", "rigExec:spacing",
    "rigExec:strength", "rigExec:mask", "rigExec:cageMatrix",
    "rigExec:targetMatrix", "rigExec:pointSpace"};
constexpr const char *_kSurfaceSettings[] = {
    "rigExec:snapMode", "rigExec:offset", "rigExec:mask",
    "rigExec:triangles", "rigExec:surfaceMatrix", "rigExec:targetMatrix",
    "rigExec:pointSpace"};

}  // namespace

const char *const *
RigExecFormatExtendedSettingNames(uint8_t op, size_t *count)
{
    const auto list = [count](const auto &names) {
        *count = std::size(names);
        return static_cast<const char *const *>(names);
    };
    switch (fb::RevisionOp(op)) {
    case fb::RevisionOp::DeltaMush:
        return list(_kDeltaMushSettings);
    case fb::RevisionOp::Lattice:
        return list(_kLatticeSettings);
    case fb::RevisionOp::SurfaceProject:
        return list(_kSurfaceSettings);
    default:
        *count = 0;
        return nullptr;
    }
}

bool
RigExecFormatExtendedInfluencesValid(uint8_t op, size_t influences)
{
    switch (fb::RevisionOp(op)) {
    case fb::RevisionOp::DeltaMush:
        return influences <= 1;
    case fb::RevisionOp::Lattice:
    case fb::RevisionOp::SurfaceProject:
        return influences == 0 || influences == 2;
    default:
        return true;
    }
}

namespace {

/// Files older than RigExecFormatOldestReadable require re-export from the
/// stage; future formats require a matching exporter/reader.
std::string
_VersionRefusal(uint32_t version)
{
    static_assert(RigExecFormatOldestReadable == 20,
                  "name what the oldest refused format version lacks");
    return "unsupported .rigexec format version " + _N(version) +
           " (this reader reads " + _N(RigExecFormatOldestReadable) +
           " through " + _N(RigExecFormatVersion) + "); " +
           (version < RigExecFormatOldestReadable
                ? "re-export: per-group range chains"
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
            if (!RigExecFormatReads(version)) {
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
    if (!RigExecFormatReads(root->formatVersion())) {
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
#if defined(__cpp_exceptions) || defined(_CPPUNWIND)
    try {
#endif
    if (!bytes || size < 8) {
        return _Fail(error, "not a .rigexec file (" + _N(size) + " bytes)");
    }
    transport::Buffer decoded;
    if (transport::IsEnvelope(bytes, size)) {
        if (!transport::Decode(bytes, size, &decoded, error)) return false;
        bytes = decoded.data.get();
        size = decoded.size;
        if (size < 8) return _Fail(error, "not a .rigexec file (decoded payload too short)");
        // Exactly one transport layer; raw identifier validation follows.
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
    return _OpenAligned(bytes, size, file, error);
#if defined(__cpp_exceptions) || defined(_CPPUNWIND)
    } catch (const std::bad_alloc &) {
        file->reset();
        return _Fail(error, "cannot open .rigexec: out of memory");
    }
#endif
}

bool
RigExecFormatWrite(const fb::RigExecWireFile &file,
                   std::vector<uint8_t> *bytes, std::string *error)
{
    if (!bytes) {
        return _Fail(error, "no buffer to write into");
    }
    bytes->clear(); // Preserve the public clear-on-failure contract.
#if defined(__cpp_exceptions) || defined(_CPPUNWIND)
    try {
#endif
    std::string why;
    if (!RigExecFormatValidate(file, &why)) {
        return _Fail(error, "invalid .rigexec: " + why);
    }
    // Readable older versions open, but every file written is current.
    if (file.formatVersion != RigExecFormatVersion) {
        return _Fail(error, "invalid .rigexec: format_version is " +
                                _N(file.formatVersion) + "; a writer writes " +
                                _N(RigExecFormatVersion));
    }
        flatbuffers::FlatBufferBuilder builder(1u << 16);
        fb::FinishFileBuffer(builder, fb::File::Pack(builder, &file));
        if (builder.GetSize() >= FLATBUFFERS_MAX_BUFFER_SIZE)
            return _Fail(error, "cannot write .rigexec: past the FlatBuffers limit");
        std::vector<uint8_t> candidate;
        if (!transport::Encode(builder.GetBufferPointer(), builder.GetSize(),
                               &candidate, error)) return false;
        if (candidate.empty())
            candidate.assign(builder.GetBufferPointer(),
                             builder.GetBufferPointer() + builder.GetSize());
        bytes->swap(candidate);
        return true;
#if defined(__cpp_exceptions) || defined(_CPPUNWIND)
    } catch (const std::bad_alloc &) {
        return _Fail(error, "cannot write .rigexec: out of memory");
    }
#endif
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
    case fb::StepKind::WeightField: return "WeightField";
    case fb::StepKind::SpaceCheckpoint: return "SpaceCheckpoint";
    case fb::StepKind::ProviderRefresh: return "ProviderRefresh";
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
    case fb::StepKind::SpaceCheckpoint: {
        if (!pose || !_Has(pose->spaceCheckpoints, object)) return false;
        *out = pose->spaceCheckpoints[size_t(object)].key;
        return true;
    }
    case fb::StepKind::ProviderRefresh:
        if(!pose || !_Has(pose->providerRefreshes,object))return false;
        *out=pose->providerRefreshes[size_t(object)].key;
        return true;
    case fb::StepKind::WeightField: {
        if (!geometry || !_Has(geometry->weightFields, object)) return false;
        const auto &field = geometry->weightFields[size_t(object)];
        if (!_Has(geometry->weightObjects, field.object)) return false;
        *out = text(geometry->weightObjects[size_t(field.object)].path) +
               " form " + std::to_string(int(field.form)) +
               " consumer " + std::to_string(field.consumer) +
               " part " + std::to_string(field.part);
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
        if ((step.part == 1 || step.part == 2) && meta && _Has(meta->paths, object)) {
            *out = text(meta->paths[size_t(object)]) + (step.part == 2 ? " base" : "");
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
