// RigExec per-frame cache. See frameCache.h.
#include "frameCache.h"

#include "frozenContext.h"

#include "pxr/base/gf/half.h"
#include "pxr/base/gf/matrix2d.h"
#include "pxr/base/gf/matrix2f.h"
#include "pxr/base/gf/matrix3d.h"
#include "pxr/base/gf/matrix3f.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/matrix4f.h"
#include "pxr/base/gf/quatd.h"
#include "pxr/base/gf/quatf.h"
#include "pxr/base/gf/quath.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/gf/vec2d.h"
#include "pxr/base/gf/vec2f.h"
#include "pxr/base/gf/vec2h.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/gf/vec3h.h"
#include "pxr/base/gf/vec3i.h"
#include "pxr/base/gf/vec4d.h"
#include "pxr/base/gf/vec4f.h"
#include "pxr/base/gf/vec4h.h"
#include "pxr/base/vt/array.h"

#include <algorithm>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <type_traits>
#include <typeinfo>
#include <utility>
#include <vector>

namespace rigExec {

namespace {

// Within-process mixing (never persisted, never compared across runs, never
// baked into tests): a word-at-a-time splitmix64 finalizer over the
// identical logical stream the old byte loop folded -- same call order,
// same tags, same counts. Large arrays dominate the digest cost, and one
// mix per 8 bytes costs roughly a third of the multiplies of one FNV round
// per byte, with stronger diffusion for short streams as well. The memcpy
// reads are alignment-safe and platform-ordered, which is fine within one
// process.
uint64_t
_MixWord(uint64_t hash, uint64_t word)
{
    // splitmix64 finalizer for the word, FNV-style chaining for the state.
    // The multiply matters: an xor-only fold is commutative, so duplicate
    // words cancel anywhere in the stream ([W,W] folds to the seed whatever
    // W is) and permutations collide -- both serve stale poses.
    word += 0x9E3779B97F4A7C15ull;
    word = (word ^ (word >> 30)) * 0xBF58476D1CE4E5B9ull;
    word = (word ^ (word >> 27)) * 0x94D049BB133111EBull;
    word ^= word >> 31;
    hash ^= word;
    hash *= 1099511628211ull;
    return hash;
}

uint64_t
_FoldBytes(uint64_t hash, const void *data, size_t size)
{
    const unsigned char *bytes = static_cast<const unsigned char *>(data);
    while (size >= 8) {
        uint64_t word;
        std::memcpy(&word, bytes, sizeof(word));
        hash = _MixWord(hash, word);
        bytes += 8;
        size -= 8;
    }
    if (size > 0) {
        // Zero-padded, with the length mixed in so a short tail cannot
        // equal a longer stream's shared prefix. An empty fold stays a
        // no-op, as before.
        uint64_t tail = 0;
        std::memcpy(&tail, bytes, size);
        hash = _MixWord(hash, tail + size);
    }
    return hash;
}

uint64_t
_FoldU64(uint64_t hash, uint64_t value)
{
    return _MixWord(hash, value);
}

uint64_t
_FoldString(uint64_t hash, const std::string &s)
{
    return _FoldBytes(hash, s.data(), s.size());
}

template <class T>
uint64_t
_FoldScalar(uint64_t hash, const T &value)
{
    static_assert(std::is_trivially_copyable<T>::value,
                  "only raw storage folds bitwise");
    return _FoldBytes(hash, &value, sizeof(T));
}

// How one point frame folds: its twelve landmark doubles plus its flags,
// bitwise like every other float in the digest.
uint64_t
_FoldPointFrame(uint64_t hash, const RigExecPointFrame &frame)
{
    for (int i = 0; i < 4; ++i) {
        for (int a = 0; a < 3; ++a) {
            hash = _FoldScalar(hash, frame.points[i][a]);
        }
    }
    return _FoldScalar(hash, frame.flags);
}

// The constant head leaves: their table's own digest, which folds each
// leaf as a sample (RigExecHeadLeafConstants). Null folds nothing.
uint64_t
_FoldHeadLeafConstants(uint64_t hash, const RigExecFrameInputs &inputs)
{
    if (!inputs.headLeafConstants) {
        return hash;
    }
    hash = _FoldBytes(hash, "head", 5);
    return _FoldU64(hash, inputs.headLeafConstants->digest);
}

// How one frame's stage seeds fold into the control digest: a tag, the six
// counts, then every entry bitwise, in program order (no sort: the vectors
// parallel the program tables positionally). The seeds are fresh stage
// data -- a moved constraint target must miss the cache -- so they fold as
// control state even though they travel as a transport.
uint64_t
_FoldStageSeeds(uint64_t hash, const RigExecStageFrameSeeds &seeds)
{
    hash = _FoldBytes(hash, "seeds\x1f", 6);
    hash = _FoldU64(hash, static_cast<uint64_t>(seeds.xformBase.size()));
    for (const GfMatrix4d &m : seeds.xformBase) {
        for (int r = 0; r < 4; ++r) {
            const GfVec4d row = m.GetRow(r);
            for (int c = 0; c < 4; ++c) {
                hash = _FoldScalar(hash, row[c]);
            }
        }
    }
    hash = _FoldU64(hash, static_cast<uint64_t>(seeds.xformFrames.size()));
    for (const RigExecPointFrame &frame : seeds.xformFrames) {
        hash = _FoldPointFrame(hash, frame);
    }
    hash = _FoldU64(hash, static_cast<uint64_t>(seeds.deltaOk.size()));
    for (const char ok : seeds.deltaOk) {
        hash = _FoldScalar(hash, ok);
    }
    hash = _FoldU64(hash, static_cast<uint64_t>(seeds.deltaBase.size()));
    for (const GfMatrix4d &m : seeds.deltaBase) {
        for (int r = 0; r < 4; ++r) {
            const GfVec4d row = m.GetRow(r);
            for (int c = 0; c < 4; ++c) {
                hash = _FoldScalar(hash, row[c]);
            }
        }
    }
    hash = _FoldU64(hash, static_cast<uint64_t>(seeds.nativeOk.size()));
    for (const char ok : seeds.nativeOk) {
        hash = _FoldScalar(hash, ok);
    }
    hash = _FoldU64(hash, static_cast<uint64_t>(seeds.nativeFrames.size()));
    for (const RigExecPointFrame &frame : seeds.nativeFrames) {
        hash = _FoldPointFrame(hash, frame);
    }
    hash = _FoldU64(hash, static_cast<uint64_t>(seeds.intervening.size()));
    for (const GfMatrix4d &matrix : seeds.intervening)
        for (int row = 0; row < 4; ++row)
            for (int column = 0; column < 4; ++column)
                hash = _FoldScalar(hash, matrix[row][column]);
    for (char reset : seeds.interveningReset) hash = _FoldScalar(hash, reset);
    return hash;
}

// How one VtValue folds into the control digest.
enum class _ValueFold {
    // Exact bits: scalars, float vectors/matrices/quats and their arrays
    // fold storage, everything else without floating point folds its type
    // name plus VtValue hash (exact for those types), and empty folds a
    // marker of its own.
    Exact,
    // Hashable, but holding floating point the digest does not fold
    // directly (dual quats, ranges, rotations of exotic form): the VtValue
    // hash, which conflates +0.0 and -0.0 the way TfHash does. Rare enough
    // to accept; a custom held type that wants exactness provides a
    // bitwise-stable TfHash.
    Generic,
    // VtValue cannot hash it and the digest has no fold for it: the frame
    // must bypass the cache (RigExecControlStateDigestible answers false).
    // The digest still folds the type name so it stays defined.
    Unhashable,
};

bool
_HoldsDirectFloat(const VtValue &value)
{
    return value.IsHolding<float>() || value.IsHolding<double>() ||
           value.IsHolding<GfHalf>() || value.IsHolding<GfVec2f>() ||
           value.IsHolding<GfVec3f>() || value.IsHolding<GfVec4f>() ||
           value.IsHolding<GfVec2d>() || value.IsHolding<GfVec3d>() ||
           value.IsHolding<GfVec4d>() || value.IsHolding<GfVec2h>() ||
           value.IsHolding<GfVec3h>() || value.IsHolding<GfVec4h>() ||
           value.IsHolding<GfQuatf>() || value.IsHolding<GfQuatd>() ||
           value.IsHolding<GfQuath>() || value.IsHolding<GfMatrix2f>() ||
           value.IsHolding<GfMatrix2d>() || value.IsHolding<GfMatrix3f>() ||
           value.IsHolding<GfMatrix3d>() || value.IsHolding<GfMatrix4f>() ||
           value.IsHolding<GfMatrix4d>() || value.IsHolding<GfRotation>() ||
           value.IsHolding<VtArray<float>>() ||
           value.IsHolding<VtArray<double>>() ||
           value.IsHolding<VtArray<GfHalf>>() ||
           value.IsHolding<VtArray<GfVec2f>>() ||
           value.IsHolding<VtArray<GfVec3f>>() ||
           value.IsHolding<VtArray<GfVec4f>>() ||
           value.IsHolding<VtArray<GfVec2d>>() ||
           value.IsHolding<VtArray<GfVec3d>>() ||
           value.IsHolding<VtArray<GfVec4d>>() ||
           value.IsHolding<VtArray<GfVec2h>>() ||
           value.IsHolding<VtArray<GfVec3h>>() ||
           value.IsHolding<VtArray<GfVec4h>>() ||
           value.IsHolding<VtArray<GfQuatf>>() ||
           value.IsHolding<VtArray<GfQuatd>>() ||
           value.IsHolding<VtArray<GfQuath>>() ||
           value.IsHolding<VtArray<GfMatrix4f>>() ||
           value.IsHolding<VtArray<GfMatrix4d>>();
}

_ValueFold
_ClassifyVtValue(const VtValue &value)
{
    if (value.IsEmpty() || _HoldsDirectFloat(value)) {
        return _ValueFold::Exact;
    }
    return value.CanHash() ? _ValueFold::Generic : _ValueFold::Unhashable;
}

template <class T>
uint64_t
_FoldArray(uint64_t hash, const VtArray<T> &array)
{
    static_assert(std::is_trivially_copyable<T>::value,
                  "only raw storage folds bitwise");
    hash = _FoldU64(hash, static_cast<uint64_t>(array.size()));
    if (!array.empty()) {
        hash = _FoldBytes(hash, array.data(), array.size() * sizeof(T));
    }
    return hash;
}

// _FoldVtValue past the type name and its separator: a non-empty value's
// storage.
uint64_t
_FoldVtValueAfterName(uint64_t hash, const VtValue &value);

uint64_t
_FoldVtValue(uint64_t hash, const VtValue &value)
{
    if (value.IsEmpty()) {
        return _FoldBytes(hash, "empty", 5);
    }
    hash = _FoldString(hash, value.GetTypeName());
    hash = _FoldBytes(hash, "\x1f", 1);
    return _FoldVtValueAfterName(hash, value);
}

// The type names one digest call folds, by type_info address, so a vector
// of a few types spells each name once instead of once per sample. Local to
// the call: two addresses of one type only spell its name twice.
class _TypeNames {
public:
    const std::string &Of(const VtValue &value)
    {
        const std::type_info *type = &value.GetTypeid();
        for (const auto &[known, name] : _names) {
            if (known == type) {
                return name;
            }
        }
        _names.emplace_back(type, value.GetTypeName());
        return _names.back().second;
    }

private:
    std::vector<std::pair<const std::type_info *, std::string>> _names;
};

// _FoldVtValue with the type name from \p names: the same stream.
uint64_t
_FoldVtValue(uint64_t hash, const VtValue &value, _TypeNames *names)
{
    if (value.IsEmpty()) {
        return _FoldBytes(hash, "empty", 5);
    }
    hash = _FoldString(hash, names->Of(value));
    hash = _FoldBytes(hash, "\x1f", 1);
    return _FoldVtValueAfterName(hash, value);
}

uint64_t
_FoldVtValueAfterName(uint64_t hash, const VtValue &value)
{
    // Bitwise for everything holding floating point: TfHash deliberately
    // conflates +0.0 and -0.0, but the evaluator need not evaluate them
    // equally, so the digest folds storage instead.
#define _RIGEXEC_FOLD_HELD(type, expr)                 \
    if (value.IsHolding<type>()) {                     \
        const type &held = value.UncheckedGet<type>(); \
        (void)held;                                    \
        return expr;                                   \
    }
    _RIGEXEC_FOLD_HELD(float, _FoldScalar(hash, held));
    _RIGEXEC_FOLD_HELD(double, _FoldScalar(hash, held));
    _RIGEXEC_FOLD_HELD(GfHalf, _FoldScalar(hash, held));
    _RIGEXEC_FOLD_HELD(GfVec2f, _FoldBytes(hash, held.data(), 2 * sizeof(float)));
    _RIGEXEC_FOLD_HELD(GfVec3f, _FoldBytes(hash, held.data(), 3 * sizeof(float)));
    _RIGEXEC_FOLD_HELD(GfVec4f, _FoldBytes(hash, held.data(), 4 * sizeof(float)));
    _RIGEXEC_FOLD_HELD(GfVec2d, _FoldBytes(hash, held.data(), 2 * sizeof(double)));
    _RIGEXEC_FOLD_HELD(GfVec3d, _FoldBytes(hash, held.data(), 3 * sizeof(double)));
    _RIGEXEC_FOLD_HELD(GfVec4d, _FoldBytes(hash, held.data(), 4 * sizeof(double)));
    _RIGEXEC_FOLD_HELD(GfVec2h, _FoldBytes(hash, held.data(), 2 * sizeof(GfHalf)));
    _RIGEXEC_FOLD_HELD(GfVec3h, _FoldBytes(hash, held.data(), 3 * sizeof(GfHalf)));
    _RIGEXEC_FOLD_HELD(GfVec4h, _FoldBytes(hash, held.data(), 4 * sizeof(GfHalf)));
    _RIGEXEC_FOLD_HELD(GfMatrix2f, _FoldBytes(hash, held.data(), 4 * sizeof(float)));
    _RIGEXEC_FOLD_HELD(GfMatrix2d, _FoldBytes(hash, held.data(), 4 * sizeof(double)));
    _RIGEXEC_FOLD_HELD(GfMatrix3f, _FoldBytes(hash, held.data(), 9 * sizeof(float)));
    _RIGEXEC_FOLD_HELD(GfMatrix3d, _FoldBytes(hash, held.data(), 9 * sizeof(double)));
    _RIGEXEC_FOLD_HELD(GfMatrix4f, _FoldBytes(hash, held.data(), 16 * sizeof(float)));
    _RIGEXEC_FOLD_HELD(GfMatrix4d, _FoldBytes(hash, held.data(), 16 * sizeof(double)));
    _RIGEXEC_FOLD_HELD(VtArray<float>, _FoldArray(hash, held));
    _RIGEXEC_FOLD_HELD(VtArray<double>, _FoldArray(hash, held));
    _RIGEXEC_FOLD_HELD(VtArray<GfHalf>, _FoldArray(hash, held));
    _RIGEXEC_FOLD_HELD(VtArray<GfVec2f>, _FoldArray(hash, held));
    _RIGEXEC_FOLD_HELD(VtArray<GfVec3f>, _FoldArray(hash, held));
    _RIGEXEC_FOLD_HELD(VtArray<GfVec4f>, _FoldArray(hash, held));
    _RIGEXEC_FOLD_HELD(VtArray<GfVec2d>, _FoldArray(hash, held));
    _RIGEXEC_FOLD_HELD(VtArray<GfVec3d>, _FoldArray(hash, held));
    _RIGEXEC_FOLD_HELD(VtArray<GfVec4d>, _FoldArray(hash, held));
    _RIGEXEC_FOLD_HELD(VtArray<GfVec2h>, _FoldArray(hash, held));
    _RIGEXEC_FOLD_HELD(VtArray<GfVec3h>, _FoldArray(hash, held));
    _RIGEXEC_FOLD_HELD(VtArray<GfVec4h>, _FoldArray(hash, held));
    _RIGEXEC_FOLD_HELD(VtArray<GfMatrix4f>, _FoldArray(hash, held));
    _RIGEXEC_FOLD_HELD(VtArray<GfMatrix4d>, _FoldArray(hash, held));
#undef _RIGEXEC_FOLD_HELD
    if (value.IsHolding<GfQuatf>()) {
        const GfQuatf &q = value.UncheckedGet<GfQuatf>();
        hash = _FoldScalar(hash, q.GetReal());
        return _FoldBytes(hash, q.GetImaginary().data(), 3 * sizeof(float));
    }
    if (value.IsHolding<GfQuatd>()) {
        const GfQuatd &q = value.UncheckedGet<GfQuatd>();
        hash = _FoldScalar(hash, q.GetReal());
        return _FoldBytes(hash, q.GetImaginary().data(), 3 * sizeof(double));
    }
    if (value.IsHolding<GfQuath>()) {
        const GfQuath &q = value.UncheckedGet<GfQuath>();
        hash = _FoldScalar(hash, q.GetReal());
        return _FoldBytes(hash, q.GetImaginary().data(), 3 * sizeof(GfHalf));
    }
    if (value.IsHolding<VtArray<GfQuatf>>()) {
        const VtArray<GfQuatf> &q = value.UncheckedGet<VtArray<GfQuatf>>();
        hash = _FoldU64(hash, static_cast<uint64_t>(q.size()));
        for (const GfQuatf &e : q) {
            hash = _FoldScalar(hash, e.GetReal());
            hash = _FoldBytes(hash, e.GetImaginary().data(), 3 * sizeof(float));
        }
        return hash;
    }
    if (value.IsHolding<VtArray<GfQuatd>>()) {
        const VtArray<GfQuatd> &q = value.UncheckedGet<VtArray<GfQuatd>>();
        hash = _FoldU64(hash, static_cast<uint64_t>(q.size()));
        for (const GfQuatd &e : q) {
            hash = _FoldScalar(hash, e.GetReal());
            hash = _FoldBytes(hash, e.GetImaginary().data(), 3 * sizeof(double));
        }
        return hash;
    }
    if (value.IsHolding<VtArray<GfQuath>>()) {
        const VtArray<GfQuath> &q = value.UncheckedGet<VtArray<GfQuath>>();
        hash = _FoldU64(hash, static_cast<uint64_t>(q.size()));
        for (const GfQuath &e : q) {
            hash = _FoldScalar(hash, e.GetReal());
            hash = _FoldBytes(hash, e.GetImaginary().data(), 3 * sizeof(GfHalf));
        }
        return hash;
    }
    if (value.IsHolding<GfRotation>()) {
        const GfRotation &r = value.UncheckedGet<GfRotation>();
        hash = _FoldBytes(hash, r.GetAxis().data(), 3 * sizeof(double));
        return _FoldScalar(hash, r.GetAngle());
    }

    // No floating point, or an exotic float holder: the VtValue hash, which
    // is exact for everything without floating point.
    if (value.CanHash()) {
        return _FoldU64(hash, static_cast<uint64_t>(value.GetHash()));
    }
    return _FoldBytes(hash, "unhashable", 10);
}

// One transport leaf, by the holders RigExecSampleRevisionLeaf answers: a
// holder tag, then its storage bitwise. Folds no type name, so it neither
// allocates nor demangles; any other holder (none is sampled) takes the
// general fold under a tag of its own.
uint64_t
_FoldLeafValue(uint64_t hash, const VtValue &value)
{
    if (value.IsEmpty()) {
        return _FoldU64(hash, 0);
    }
#define _RIGEXEC_FOLD_LEAF(tag, type, expr)            \
    if (value.IsHolding<type>()) {                     \
        const type &held = value.UncheckedGet<type>(); \
        hash = _FoldU64(hash, tag);                    \
        return expr;                                   \
    }
    _RIGEXEC_FOLD_LEAF(1, VtIntArray, _FoldArray(hash, held));
    _RIGEXEC_FOLD_LEAF(2, VtFloatArray, _FoldArray(hash, held));
    _RIGEXEC_FOLD_LEAF(3, int, _FoldScalar(hash, held));
    _RIGEXEC_FOLD_LEAF(4, float, _FoldScalar(hash, held));
    _RIGEXEC_FOLD_LEAF(5, double, _FoldScalar(hash, held));
    _RIGEXEC_FOLD_LEAF(6, bool, _FoldScalar(hash, held));
    _RIGEXEC_FOLD_LEAF(7, GfVec3f, _FoldBytes(hash, held.data(), 3 * sizeof(float)));
    _RIGEXEC_FOLD_LEAF(8, GfVec3d, _FoldBytes(hash, held.data(), 3 * sizeof(double)));
    _RIGEXEC_FOLD_LEAF(9, GfVec3i, _FoldBytes(hash, held.data(), 3 * sizeof(int)));
    _RIGEXEC_FOLD_LEAF(10, GfMatrix4d, _FoldBytes(hash, held.data(), 16 * sizeof(double)));
    _RIGEXEC_FOLD_LEAF(11, VtArray<GfVec2f>, _FoldArray(hash, held));
    _RIGEXEC_FOLD_LEAF(12, VtVec3fArray, _FoldArray(hash, held));
    _RIGEXEC_FOLD_LEAF(13, VtDoubleArray, _FoldArray(hash, held));
    // The token's own string, length first; an empty token reads no
    // string at all.
    _RIGEXEC_FOLD_LEAF(14, TfToken,
                       held.IsEmpty()
                           ? _FoldU64(hash, 0)
                           : _FoldString(_FoldU64(hash, held.size()),
                                         held.GetString()));
#undef _RIGEXEC_FOLD_LEAF
    return _FoldVtValue(_FoldU64(hash, 15), value);
}

// The transport values no sample covers that can move with the time
// (RigExecFrameInputs::varyingLayoutRows, varyingRevisionLeaves): a tag, then
// per list its count and per entry its coordinates and values. A coordinate
// the vector does not hold folds a marker (RigExecControlStateDigestible
// refuses it). Both lists empty fold nothing.
uint64_t
_FoldVaryingTransport(uint64_t hash, const RigExecFrameInputs &inputs)
{
    if (inputs.varyingLayoutRows.empty() &&
        inputs.varyingRevisionLeaves.empty()) {
        return hash;
    }
    constexpr uint64_t kAbsent = ~uint64_t(0);
    hash = _FoldBytes(hash, "vary\x1f", 5);
    hash = _FoldU64(hash,
                    static_cast<uint64_t>(inputs.varyingLayoutRows.size()));
    for (const uint32_t row : inputs.varyingLayoutRows) {
        hash = _FoldU64(hash, row);
        if (row >= inputs.layoutLeaves.size()) {
            hash = _FoldU64(hash, kAbsent);
            continue;
        }
        const std::vector<VtValue> &values = inputs.layoutLeaves[row];
        hash = _FoldU64(hash, static_cast<uint64_t>(values.size()));
        for (const VtValue &value : values) {
            hash = _FoldLeafValue(hash, value);
        }
    }
    hash = _FoldU64(hash,
                    static_cast<uint64_t>(inputs.varyingRevisionLeaves.size()));
    for (const auto &[row, key] : inputs.varyingRevisionLeaves) {
        hash = _FoldU64(hash, (uint64_t(row) << 32) | uint64_t(key));
        hash = row < inputs.revisionLeaves.size() &&
                       key < inputs.revisionLeaves[row].size()
                   ? _FoldLeafValue(hash, inputs.revisionLeaves[row][key])
                   : _FoldU64(hash, kAbsent);
    }
    return hash;
}

// Payload bytes of one VtValue, by the bench's counting: array payloads and
// scalar sizes; unknown types count the shell (the report says so).
size_t
_VtValueBytes(const VtValue &value)
{
    if (value.IsEmpty()) {
        return 0;
    }
#define _RIGEXEC_CACHE_ARRAY(type, element)                     \
    if (value.IsHolding<type>()) {                              \
        return value.UncheckedGet<type>().size() * sizeof(element); \
    }
    _RIGEXEC_CACHE_ARRAY(VtVec3fArray, GfVec3f)
    _RIGEXEC_CACHE_ARRAY(VtVec3dArray, GfVec3d)
    _RIGEXEC_CACHE_ARRAY(VtVec2fArray, GfVec2f)
    _RIGEXEC_CACHE_ARRAY(VtVec4fArray, GfVec4f)
    _RIGEXEC_CACHE_ARRAY(VtFloatArray, float)
    _RIGEXEC_CACHE_ARRAY(VtDoubleArray, double)
    _RIGEXEC_CACHE_ARRAY(VtIntArray, int)
    _RIGEXEC_CACHE_ARRAY(VtBoolArray, bool)
    _RIGEXEC_CACHE_ARRAY(VtStringArray, std::string)
#undef _RIGEXEC_CACHE_ARRAY
#define _RIGEXEC_CACHE_SCALAR(type)        \
    if (value.IsHolding<type>()) {         \
        return sizeof(type);               \
    }
    _RIGEXEC_CACHE_SCALAR(float)
    _RIGEXEC_CACHE_SCALAR(double)
    _RIGEXEC_CACHE_SCALAR(int)
    _RIGEXEC_CACHE_SCALAR(bool)
    _RIGEXEC_CACHE_SCALAR(GfVec2f)
    _RIGEXEC_CACHE_SCALAR(GfVec3f)
    _RIGEXEC_CACHE_SCALAR(GfVec3d)
    _RIGEXEC_CACHE_SCALAR(GfVec4f)
    _RIGEXEC_CACHE_SCALAR(GfMatrix4d)
    _RIGEXEC_CACHE_SCALAR(TfToken)
    _RIGEXEC_CACHE_SCALAR(SdfPath)
#undef _RIGEXEC_CACHE_SCALAR
    if (value.IsHolding<std::string>()) {
        return value.UncheckedGet<std::string>().size();
    }
    return sizeof(VtValue);
}

}  // namespace

uint64_t
RigExecControlStateDigest(const RigExecFrameInputs &inputs)
{
    const std::vector<RigExecValueOverride> noOverrides;
    return RigExecControlStateDigest(inputs, noOverrides);
}

// The upstream block: a tag, the count, then per path (sorted, the last
// entry of a path winning) its path and value type, then the value. Empty
// folds nothing, so a frame with no upstream value keys as before.
uint64_t
_FoldUpstream(uint64_t hash, const std::vector<RigExecUpstreamValue> &upstream)
{
    if (upstream.empty()) {
        return hash;
    }
    std::map<SdfPath, const RigExecUpstreamValue *> byPath;
    for (const RigExecUpstreamValue &value : upstream) {
        byPath[value.path] = &value;
    }
    hash = _FoldBytes(hash, "ups\x1f", 4);
    hash = _FoldU64(hash, static_cast<uint64_t>(byPath.size()));
    for (const auto &[path, value] : byPath) {
        hash = _FoldString(hash, path.GetString());
        hash = _FoldBytes(hash, "\x1f", 1);
        hash = _FoldString(hash, value->value.GetTypeName());
        hash = _FoldBytes(hash, "\x1f", 1);
        // An array folds the hash its producer computed once
        // (RigExecUpstreamFoldHash); one handed over without folds its
        // bytes.
        hash = value->value.IsArrayValued() && value->foldHash != 0
                   ? _FoldU64(hash, value->foldHash)
                   : _FoldVtValue(hash, value->value);
    }
    return hash;
}

uint64_t
RigExecUpstreamFoldHash(const VtValue &value)
{
    if (!value.IsArrayValued()) {
        return 0;
    }
    uint64_t hash = 1469598103934665603ull;
    hash = _FoldString(hash, value.GetTypeName());
    hash = _FoldVtValue(hash, value);
    return hash != 0 ? hash : 1;
}

uint64_t
_FoldOverrides(uint64_t hash,
               const std::vector<RigExecValueOverride> &overrides)
{
    // Overrides sort by (prim, computation, attribute); a repeated key keeps
    // the LAST override, matching the evaluator's replace-not-join rule.
    using _OverrideKey = std::tuple<std::string, std::string, std::string>;
    std::map<_OverrideKey, const RigExecValueOverride *> byKey;
    for (const RigExecValueOverride &o : overrides) {
        byKey[_OverrideKey(o.prim.GetString(), o.computation.GetString(),
                           o.attribute.GetString())] = &o;
    }
    hash = _FoldU64(hash, static_cast<uint64_t>(byKey.size()));
    for (const auto &kv : byKey) {
        hash = _FoldBytes(hash, "ovr\x1f", 4);
        hash = _FoldString(hash, std::get<0>(kv.first));
        hash = _FoldBytes(hash, "\x1f", 1);
        hash = _FoldString(hash, std::get<1>(kv.first));
        hash = _FoldBytes(hash, "\x1f", 1);
        hash = _FoldString(hash, std::get<2>(kv.first));
        hash = _FoldBytes(hash, "\x1f", 1);
        hash = _FoldVtValue(hash, kv.second->value);
    }
    return hash;
}

namespace {

// A level-1's state after its path: the "src" tag and the path's text.
uint64_t
_SamplePrefix(const SdfPath &path)
{
    uint64_t hash = 1469598103934665603ull;
    hash = _FoldBytes(hash, "src\x1f", 4);
    return _FoldString(hash, path.GetString());
}

// The rest of a level-1 from its path's state \p prefix.
template <class Fold>
uint64_t
_SampleFromPrefix(uint64_t hash, const RigExecSampledInput &sample,
                  Fold &&foldValue)
{
    hash = _FoldBytes(hash, sample.hasValue ? "\x01" : "\x00", 1);
    hash = _FoldBytes(hash, sample.valueBlocked ? "\x01" : "\x00", 1);
    if (sample.hasValue) {
        hash = foldValue(hash, sample.value);
    } else {
        // Valueless is a value of its own: only hasValue decides, so a
        // stale VtValue riding along cannot move the digest.
        hash = _FoldBytes(hash, "novalue", 7);
    }
    return hash;
}

// Whether \p sample is marked as copying an entry of \p inputs' static
// samples table, and which (RigExecSampledInput::staticSample).
bool
_StaticEntry(const RigExecFrameInputs &inputs,
             const RigExecSampledInput &sample, size_t *entry)
{
    const RigExecFrozenStaticSamples *table = inputs.staticSamples.get();
    if (!table || sample.staticSample < 0 ||
        size_t(sample.staticSample) >= table->samples.size() ||
        table->level1.size() != table->samples.size() ||
        table->digestible.size() != table->samples.size() ||
        table->samples[size_t(sample.staticSample)].path != sample.path) {
        return false;
    }
    *entry = size_t(sample.staticSample);
    return true;
}

// One sample's level-1 under a recorded order: the static table's memo for
// a sample copied from it, else folded from the recorded path state.
uint64_t
_OrderedLevel1(const RigExecFrameInputs &inputs,
               const RigExecSampledInput &sample, uint64_t prefix,
               _TypeNames *names)
{
    size_t entry = 0;
    if (_StaticEntry(inputs, sample, &entry)) {
        return inputs.staticSamples->level1[entry];
    }
    return _SampleFromPrefix(prefix, sample,
                             [names](uint64_t hash, const VtValue &value) {
                                 return _FoldVtValue(hash, value, names);
                             });
}

}  // namespace

uint64_t
RigExecSampleDigest(const RigExecSampledInput &sample)
{
    return _SampleFromPrefix(_SamplePrefix(sample.path), sample,
                             [](uint64_t hash, const VtValue &value) {
                                 return _FoldVtValue(hash, value);
                             });
}

bool
RigExecSampleDigestible(const RigExecSampledInput &sample)
{
    return !sample.hasValue ||
           _ClassifyVtValue(sample.value) != _ValueFold::Unhashable;
}

// `paths` is the recorded sequence; `order[j]` the index of the j-th
// first-win path in sorted order and `prefix[j]` its _SamplePrefix;
// `pathTextBytes` the text bytes of every path in `paths`.
struct RigExecFrameDigestOrder {
    std::vector<SdfPath> paths;
    std::vector<uint32_t> order;
    std::vector<uint64_t> prefix;
    size_t pathTextBytes = 0;
};

std::shared_ptr<const RigExecFrameDigestOrder>
RigExecRecordFrameDigestOrder(const std::vector<RigExecSampledInput> &values)
{
    auto order = std::make_shared<RigExecFrameDigestOrder>();
    order->paths.reserve(values.size());
    // First sample per path wins, sorted by path: the plain digest's map.
    std::map<SdfPath, uint32_t> ordered;
    for (size_t i = 0; i < values.size(); ++i) {
        order->paths.push_back(values[i].path);
        order->pathTextBytes += values[i].path.GetString().size();
        ordered.emplace(values[i].path, uint32_t(i));
    }
    order->order.reserve(ordered.size());
    order->prefix.reserve(ordered.size());
    for (const auto &[path, index] : ordered) {
        order->order.push_back(index);
        order->prefix.push_back(_SamplePrefix(path));
    }
    return order;
}

bool
RigExecFrameDigestOrderMatches(const RigExecFrameDigestOrder *order,
                               const std::vector<RigExecSampledInput> &values)
{
    if (!order || order->paths.size() != values.size() ||
        order->prefix.size() != order->order.size()) {
        return false;
    }
    for (size_t i = 0; i < values.size(); ++i) {
        if (order->paths[i] != values[i].path) {
            return false;
        }
    }
    return true;
}

size_t
RigExecFrameDigestOrderPathTextBytes(const RigExecFrameDigestOrder *order)
{
    return order ? order->pathTextBytes : 0;
}

bool
RigExecFrameDigestOrderHasPath(const RigExecFrameDigestOrder *order,
                               const SdfPath &path)
{
    if (!order) {
        return false;
    }
    // `order` lists the first-win paths ascending by SdfPath's operator<,
    // the recording map's own comparison.
    const auto found = std::lower_bound(
        order->order.begin(), order->order.end(), path,
        [order](uint32_t index, const SdfPath &key) {
            return order->paths[index] < key;
        });
    return found != order->order.end() && order->paths[*found] == path;
}

uint64_t
RigExecFoldConstantDigest(uint64_t controlDigest, uint64_t constantDigest)
{
    uint64_t hash = 1469598103934665603ull;
    hash = _FoldBytes(hash, "const\x1f", 6);
    hash = _FoldU64(hash, controlDigest);
    hash = _FoldU64(hash, constantDigest);
    return hash;
}

uint64_t
RigExecControlStateDigestWithConstants(
    const RigExecFrameInputs &inputs,
    const std::vector<RigExecValueOverride> &overrides,
    uint64_t constantDigest)
{
    return RigExecFoldConstantDigest(
        RigExecControlStateDigest(inputs, overrides), constantDigest);
}

uint64_t
RigExecControlStateDigestWithConstants(
    const RigExecFrameInputs &inputs,
    const std::vector<RigExecValueOverride> &overrides,
    const std::vector<RigExecUpstreamValue> &upstream,
    uint64_t constantDigest)
{
    return RigExecFoldConstantDigest(
        RigExecControlStateDigest(inputs, overrides, upstream),
        constantDigest);
}

uint64_t
RigExecControlStateDigest(const RigExecFrameInputs &inputs,
                         const std::vector<RigExecValueOverride> &overrides)
{
    return RigExecControlStateDigest(inputs, overrides, inputs.upstream);
}

uint64_t
RigExecControlStateDigest(const RigExecFrameInputs &inputs,
                         const std::vector<RigExecValueOverride> &overrides,
                         const std::vector<RigExecUpstreamValue> &upstream)
{
    // First sample per path wins, matching FrameInputs::Find; the survivors
    // sort by path so enqueue order cannot move the digest. Two levels --
    // one level-1 per first-win path, chained non-commutatively -- so a
    // burst can memoize the time-invariant level-1s and fold only the rest
    // per frame. The digest always folds the served vector's contents: a
    // memoized level-1 names the value the sampler served, never a re-read.
    uint64_t hash = 1469598103934665603ull;
    const RigExecFrameDigestOrder *recorded = inputs.digestOrder.get();
    if (RigExecFrameDigestOrderMatches(recorded, inputs.values)) {
        // The order the sampler recorded for this path sequence: the same
        // first-win paths in the same sorted order, each path's state
        // precomputed.
        _TypeNames names;
        hash = _FoldU64(hash, static_cast<uint64_t>(recorded->order.size()));
        for (size_t j = 0; j < recorded->order.size(); ++j) {
            hash = _FoldU64(
                hash, _OrderedLevel1(inputs, inputs.values[recorded->order[j]],
                                     recorded->prefix[j], &names));
        }
    } else {
        std::map<SdfPath, const RigExecSampledInput *> ordered;
        for (const RigExecSampledInput &sampled : inputs.values) {
            ordered.emplace(sampled.path, &sampled);
        }
        hash = _FoldU64(hash, static_cast<uint64_t>(ordered.size()));
        for (const auto &kv : ordered) {
            hash = _FoldU64(hash, RigExecSampleDigest(*kv.second));
        }
    }
    hash = _FoldHeadLeafConstants(hash, inputs);
    hash = _FoldVaryingTransport(hash, inputs);
    hash = _FoldStageSeeds(hash, inputs.stageSeeds);
    hash = _FoldUpstream(hash, upstream);

    return _FoldOverrides(hash, overrides);
}

uint64_t
RigExecControlStateDigestWithBurstCache(
    const RigExecFrameInputs &inputs,
    const std::vector<RigExecValueOverride> &overrides,
    RigExecBurstSampleCache *cache)
{
    return RigExecControlStateDigestWithBurstCache(inputs, overrides,
                                                   inputs.upstream, cache);
}

uint64_t
RigExecControlStateDigestWithBurstCache(
    const RigExecFrameInputs &inputs,
    const std::vector<RigExecValueOverride> &overrides,
    const std::vector<RigExecUpstreamValue> &upstream,
    RigExecBurstSampleCache *cache)
{
    // Anything the cache cannot serve falls back to the plain digest:
    // same answer, full cost. Overrides first: a differing list means a
    // differing vector shape and differing placement, which the recorded
    // order and the static maps do not describe.
    if (!cache || !cache->usable || overrides != cache->overrides) {
        return RigExecControlStateDigest(inputs, overrides, upstream);
    }
    if (cache->sortedOrder.empty()) {
        // First frame of the burst: record the emission indices in
        // sorted-path order, first-wins, exactly as the plain digest
        // orders them. (An empty vector records an empty order and
        // rebuilds trivially per frame; a non-empty vector always records
        // at least its first sample.)
        std::map<SdfPath, size_t> ordered;
        for (size_t i = 0; i < inputs.values.size(); ++i) {
            ordered.emplace(inputs.values[i].path, i);
        }
        cache->sortedOrder.reserve(ordered.size());
        for (const auto &kv : ordered) {
            cache->sortedOrder.push_back(kv.second);
        }
        cache->orderValuesSize = inputs.values.size();
        if (!inputs.values.empty()) {
            cache->orderFirstPath = inputs.values.front().path;
            cache->orderLastPath = inputs.values.back().path;
        }
    } else if (inputs.values.size() != cache->orderValuesSize ||
               inputs.values.front().path != cache->orderFirstPath ||
               inputs.values.back().path != cache->orderLastPath) {
        // A vector shaped unlike the recorded order's: the burst sampler
        // emits the same path sequence every frame (every Add guard is
        // validity-only), so this is a foreign vector, not a later frame.
        // The size check short-circuits first, so front/back are safe.
        return RigExecControlStateDigest(inputs, overrides, upstream);
    }
    // The level-1 of a sample one of the burst's maps served, from its memo;
    // false for any other sample.
    const auto burstLevel1 = [cache](const RigExecSampledInput &sample,
                                     uint64_t *level1) {
        if (sample.burstSampleRoute != RigExecBurstRouteStage &&
            sample.burstSampleRoute != RigExecBurstRouteResolved) {
            return false;
        }
        auto &memo = sample.burstSampleRoute == RigExecBurstRouteStage
                         ? cache->staticStage
                         : cache->staticResolved;
        const auto found = memo.find(sample.path);
        if (found == memo.end()) {
            return false;
        }
        if (!found->second.digestValid) {
            found->second.digest = RigExecSampleDigest(sample);
            found->second.digestValid = true;
        }
        *level1 = found->second.digest;
        return true;
    };
    uint64_t hash = 1469598103934665603ull;
    const RigExecFrameDigestOrder *recorded = inputs.digestOrder.get();
    if (RigExecFrameDigestOrderMatches(recorded, inputs.values)) {
        // The sampler's recorded order is this vector's exact first-win sort,
        // which the burst's order describes too; it also carries each path's
        // state and the static table's level-1s.
        _TypeNames names;
        hash = _FoldU64(hash, static_cast<uint64_t>(recorded->order.size()));
        for (size_t j = 0; j < recorded->order.size(); ++j) {
            const RigExecSampledInput &sample =
                inputs.values[recorded->order[j]];
            uint64_t level1 = 0;
            if (!burstLevel1(sample, &level1)) {
                level1 = _OrderedLevel1(inputs, sample, recorded->prefix[j],
                                        &names);
            }
            hash = _FoldU64(hash, level1);
        }
    } else {
        hash = _FoldU64(hash,
                        static_cast<uint64_t>(cache->sortedOrder.size()));
        for (size_t i : cache->sortedOrder) {
            const RigExecSampledInput &sample = inputs.values[i];
            uint64_t level1 = 0;
            if (!burstLevel1(sample, &level1)) {
                level1 = RigExecSampleDigest(sample);
            }
            hash = _FoldU64(hash, level1);
        }
    }
    hash = _FoldHeadLeafConstants(hash, inputs);
    // Per-frame values with no memo, folded as the plain digest folds them.
    hash = _FoldVaryingTransport(hash, inputs);
    // The seeds fold fresh every frame, exactly as in the plain digest:
    // they are per-frame stage reads, so no static memo serves them.
    hash = _FoldStageSeeds(hash, inputs.stageSeeds);
    hash = _FoldUpstream(hash, upstream);
    return _FoldOverrides(hash, overrides);
}

uint64_t
RigExecRefusalControlDigest(
    UsdTimeCode time, uint64_t stageEditSerial,
    const std::vector<RigExecValueOverride> &overrides)
{
    return RigExecRefusalControlDigest(time, stageEditSerial, overrides, {});
}

uint64_t
RigExecRefusalControlDigest(
    UsdTimeCode time, uint64_t stageEditSerial,
    const std::vector<RigExecValueOverride> &overrides,
    const std::vector<RigExecUpstreamValue> &upstream)
{
    uint64_t hash = 1469598103934665603ull;
    // The domain tag: a refusal digest must never equal a sampled digest
    // under the same epoch (a mode toggle crosses the two without moving
    // the epoch, and entries outlive the proofs that scope them).
    hash = _FoldBytes(hash, "refusal\x1f", 8);
    // The edit sensor: every stage notice advances the evaluator's serial,
    // so an edit moves this digest at every frame and stale entries become
    // unreachable -- the D1 rule for a rig that cannot re-sample.
    hash = _FoldU64(hash, stageEditSerial);
    if (time.IsDefault()) {
        hash = _FoldBytes(hash, "default", 7);
    } else {
        hash = _FoldBytes(hash, "time\x1f", 5);
        const double value = time.GetValue();
        hash = _FoldScalar(hash, value);
    }
    hash = _FoldUpstream(hash, upstream);
    return _FoldOverrides(hash, overrides);
}

bool
RigExecControlStateDigestible(const RigExecFrameInputs &inputs)
{
    const std::vector<RigExecValueOverride> noOverrides;
    return RigExecControlStateDigestible(inputs, noOverrides);
}

bool
RigExecControlStateDigestible(
    const RigExecFrameInputs &inputs,
    const std::vector<RigExecValueOverride> &overrides)
{
    for (const RigExecSampledInput &sampled : inputs.values) {
        if (!sampled.hasValue) {
            continue;
        }
        size_t entry = 0;
        if (_StaticEntry(inputs, sampled, &entry)) {
            if (!inputs.staticSamples->digestible[entry]) {
                return false;
            }
            continue;
        }
        if (_ClassifyVtValue(sampled.value) == _ValueFold::Unhashable) {
            return false;
        }
    }
    if (inputs.headLeafConstants && !inputs.headLeafConstants->digestible) {
        return false;
    }
    for (const uint32_t row : inputs.varyingLayoutRows) {
        if (row >= inputs.layoutLeaves.size()) {
            return false;
        }
        for (const VtValue &value : inputs.layoutLeaves[row]) {
            if (_ClassifyVtValue(value) == _ValueFold::Unhashable) {
                return false;
            }
        }
    }
    for (const auto &[row, key] : inputs.varyingRevisionLeaves) {
        if (row >= inputs.revisionLeaves.size() ||
            key >= inputs.revisionLeaves[row].size() ||
            _ClassifyVtValue(inputs.revisionLeaves[row][key]) ==
                _ValueFold::Unhashable) {
            return false;
        }
    }
    for (const RigExecUpstreamValue &value : inputs.upstream) {
        if (_ClassifyVtValue(value.value) == _ValueFold::Unhashable) {
            return false;
        }
    }
    for (const RigExecValueOverride &o : overrides) {
        if (_ClassifyVtValue(o.value) == _ValueFold::Unhashable) {
            return false;
        }
    }
    return true;
}

bool
RigExecRefusalControlDigestible(
    const std::vector<RigExecValueOverride> &overrides)
{
    return RigExecRefusalControlDigestible(overrides, {});
}

bool
RigExecRefusalControlDigestible(
    const std::vector<RigExecValueOverride> &overrides,
    const std::vector<RigExecUpstreamValue> &upstream)
{
    // Time always folds; only the overrides and the upstream values can
    // carry an unhashable type.
    for (const RigExecValueOverride &o : overrides) {
        if (_ClassifyVtValue(o.value) == _ValueFold::Unhashable) {
            return false;
        }
    }
    for (const RigExecUpstreamValue &value : upstream) {
        if (_ClassifyVtValue(value.value) == _ValueFold::Unhashable) {
            return false;
        }
    }
    return true;
}

size_t
RigExecFrameCachePoseBytes(const RigExecRigPose &pose)
{
    size_t total = sizeof(RigExecRigPose);
    total += pose.jointFramesBase.size() *
             (sizeof(SdfPath) + sizeof(RigExecPointFrame));
    total += pose.jointFramesFinal.size() *
             (sizeof(SdfPath) + sizeof(RigExecPointFrame));
    total += pose.jointMatricesFinal.size() *
             (sizeof(SdfPath) + sizeof(GfMatrix4d));
    total += pose.controlFrames.size() *
             (sizeof(SdfPath) + sizeof(RigExecPointFrame));
    total += pose.providerXforms.size() *
             (sizeof(SdfPath) + sizeof(GfMatrix4d));
    total += pose.providerBaseXforms.size() *
             (sizeof(SdfPath) + sizeof(GfMatrix4d));
    for (const auto &kv : pose.solverFrames) {
        total += sizeof(SdfPath) + kv.second.size() * sizeof(RigExecPointFrame);
    }
    for (const auto &kv : pose.movedProperties) {
        total += sizeof(SdfPath) + _VtValueBytes(kv.second);
    }
    for (const auto &kv : pose.movedPropertiesCpu) {
        total += sizeof(SdfPath) + _VtValueBytes(kv.second);
    }
    for (const auto &kv : pose.weightFields) {
        total += sizeof(SdfPath) + sizeof(SdfPath) +
                 kv.second.weights.size() * sizeof(float);
    }
    total += pose.weightFrames.size() * (sizeof(SdfPath) + sizeof(GfMatrix4d));
    for (const std::string &d : pose.diagnostics) {
        total += d.size();
    }
    return total;
}

RigExecFrameCache::RigExecFrameCache() = default;

RigExecFrameCache::~RigExecFrameCache() = default;

void
RigExecFrameCache::SetEvictionCallback(
    RigExecFrameCacheEvictionCallback callback)
{
    std::lock_guard<std::mutex> lock(_callbackMutex);
    _evictionCallback = std::move(callback);
}

void
RigExecFrameCache::_ReportEvictions(
    std::vector<RigExecFrameCacheEviction> evicted) const
{
    if (evicted.empty()) {
        return;
    }
    RigExecFrameCacheEvictionCallback callback;
    {
        std::lock_guard<std::mutex> lock(_callbackMutex);
        callback = _evictionCallback;
    }
    if (callback) {
        callback(evicted);
    }
}

bool
RigExecFrameCache::Lookup(const RigExecFrameCacheKey &key,
                          RigExecRigPose *pose) const
{
    return Lookup(key, pose, nullptr);
}

bool
RigExecFrameCache::Lookup(const RigExecFrameCacheKey &key,
                          RigExecRigPose *pose,
                          std::shared_ptr<const void> *retainedOut) const
{
    return Lookup(key, pose, retainedOut, nullptr);
}

bool
RigExecFrameCache::Lookup(const RigExecFrameCacheKey &key,
                          RigExecRigPose *pose,
                          std::shared_ptr<const void> *retainedOut,
                          size_t *bytesOut) const
{
    if (!pose) {
        _misses.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    // The handle crosses the lock; the pose crosses outside it.
    std::shared_ptr<_Entry> entry;
    bool corrupt = false;
    {
        const _Shard &shard = _GetShard(key);
        std::lock_guard<std::mutex> lock(shard.mutex);
        const auto it = shard.map.find(key);
        if (it == shard.map.end()) {
            _misses.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        entry = it->second;
        if (!entry || !entry->pose || entry->bytes == 0 ||
            !entry->pose->valid) {
            // Drop, don't guess: a null, weightless, or invalid entry is
            // evicted and the lookup misses, never partially served.
            if (entry) {
                _bytes.fetch_sub(entry->bytes, std::memory_order_relaxed);
            }
            _entryCount.fetch_sub(1, std::memory_order_relaxed);
            _evictions.fetch_add(1, std::memory_order_relaxed);
            shard.map.erase(it);
            corrupt = true;
        } else {
            entry->lastUse.store(
                _clock.fetch_add(1, std::memory_order_relaxed),
                std::memory_order_relaxed);
        }
    }
    if (corrupt) {
        RigExecFrameCacheEviction dropped;
        dropped.key = key;
        dropped.time = entry ? entry->time : UsdTimeCode::Default();
        _ReportEvictions({dropped});
        _misses.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    *pose = *entry->pose;
    if (retainedOut) {
        *retainedOut = entry->retained;
    }
    if (bytesOut) {
        *bytesOut = entry->bytes;
    }
    _hits.fetch_add(1, std::memory_order_relaxed);
    return true;
}

bool
RigExecFrameCache::LookupProvenance(
    const RigExecFrameCacheKey &key,
    RigExecEntryProvenance *provenance) const
{
    if (!provenance) {
        return false;
    }
    const _Shard &shard = _GetShard(key);
    std::lock_guard<std::mutex> lock(shard.mutex);
    const auto it = shard.map.find(key);
    if (it == shard.map.end() || !it->second) {
        return false;
    }
    *provenance = it->second->provenance;
    return true;
}

void
RigExecFrameCache::NoteProvenanceAlias(const RigExecFrameCacheKey &key,
                                       UsdTimeCode time)
{
    const double stamped = time.IsDefault() ? 0.0 : time.GetValue();
    _Shard &shard = _GetShard(key);
    std::lock_guard<std::mutex> lock(shard.mutex);
    const auto it = shard.map.find(key);
    if (it == shard.map.end() || !it->second) {
        return;
    }
    RigExecEntryProvenance &provenance = it->second->provenance;
    if (provenance.aliasTimes.size() >= kRigExecProvenanceAliasCap) {
        return;
    }
    for (const double listed : provenance.aliasTimes) {
        if (listed == stamped) {
            return;
        }
    }
    provenance.aliasTimes.push_back(stamped);
}

bool
RigExecFrameCache::Publish(const RigExecFrameCacheKey &key, UsdTimeCode time,
                           const RigExecRigPose &pose,
                           const RigExecEntryProvenance *provenance)
{
    return Publish(key, time, pose, 0, nullptr, provenance);
}

bool
RigExecFrameCache::Publish(const RigExecFrameCacheKey &key, UsdTimeCode time,
                           const RigExecRigPose &pose, size_t retainedBytes,
                           std::shared_ptr<const void> retained,
                           const RigExecEntryProvenance *provenance)
{
    // Everything that allocates, copies, or measures happens before the
    // lock; under it the entry is only linked in.
    if (!pose.valid) {
        return false;
    }
    if (retainedBytes > 0 && !retained) {
        return false;
    }
    const size_t bytes = RigExecFrameCachePoseBytes(pose) + retainedBytes;
    if (bytes == 0 || bytes > _byteCap.load(std::memory_order_relaxed)) {
        return false;
    }
    std::shared_ptr<_Entry> entry = std::make_shared<_Entry>();
    entry->time = time;
    entry->pose = std::make_shared<RigExecRigPose>(pose);
    entry->bytes = bytes;
    entry->retained = std::move(retained);
    if (provenance) {
        entry->provenance = *provenance;
    }
    // The publishing time is the first alias: one digest may serve many
    // frames, and the publication is the first frame it served. A
    // provenance that already names it (see RigExecFullEvalProvenance)
    // is not listed twice.
    const double stamped = time.IsDefault() ? 0.0 : time.GetValue();
    if (entry->provenance.aliasTimes.empty() ||
        entry->provenance.aliasTimes.front() != stamped) {
        entry->provenance.aliasTimes.insert(
            entry->provenance.aliasTimes.begin(), stamped);
    }
    entry->lastUse.store(_clock.fetch_add(1, std::memory_order_relaxed),
                         std::memory_order_relaxed);
    // Any entry this call displaces is moved here and destroyed only after
    // the shard lock closes: a biped pose is hundreds of KB of maps, and the
    // lock rule keeps that teardown off the critical section.
    std::shared_ptr<_Entry> retired;
    {
        _Shard &shard = _GetShard(key);
        std::lock_guard<std::mutex> lock(shard.mutex);
        const auto it = shard.map.find(key);
        if (it != shard.map.end()) {
            _bytes.fetch_sub(it->second->bytes, std::memory_order_relaxed);
            // The displaced entry dies after the lock closes (see retired).
            retired = std::move(it->second);
            it->second = std::move(entry);
            _bytes.fetch_add(bytes, std::memory_order_relaxed);
        } else {
            shard.map.emplace(key, std::move(entry));
            _entryCount.fetch_add(1, std::memory_order_relaxed);
            _bytes.fetch_add(bytes, std::memory_order_relaxed);
        }
        _published.fetch_add(1, std::memory_order_relaxed);
    }
    _EvictUntilUnderCap();
    return true;
}

bool
RigExecFrameCache::Evict(const RigExecFrameCacheKey &key)
{
    std::shared_ptr<_Entry> retired;
    {
        _Shard &shard = _GetShard(key);
        std::lock_guard<std::mutex> lock(shard.mutex);
        const auto it = shard.map.find(key);
        if (it == shard.map.end()) {
            return false;
        }
        _bytes.fetch_sub(it->second->bytes, std::memory_order_relaxed);
        _entryCount.fetch_sub(1, std::memory_order_relaxed);
        _evictions.fetch_add(1, std::memory_order_relaxed);
        retired = std::move(it->second);
        shard.map.erase(it);
    }
    // The payload dies here, outside the lock.
    RigExecFrameCacheEviction dropped;
    dropped.key = key;
    dropped.time = retired->time;
    _ReportEvictions({dropped});
    return true;
}

size_t
RigExecFrameCache::EvictEpoch(uint64_t epochDigest)
{
    // One shard at a time, like _EvictUntilUnderCap: no caller ever needs
    // the whole store locked at once.
    size_t dropped = 0;
    std::vector<std::shared_ptr<_Entry>> retired;
    std::vector<RigExecFrameCacheEviction> evicted;
    for (_Shard &shard : _shards) {
        std::lock_guard<std::mutex> lock(shard.mutex);
        for (auto it = shard.map.begin(); it != shard.map.end();) {
            if (it->first.epochDigest != epochDigest) {
                ++it;
                continue;
            }
            _bytes.fetch_sub(it->second->bytes, std::memory_order_relaxed);
            _entryCount.fetch_sub(1, std::memory_order_relaxed);
            _evictions.fetch_add(1, std::memory_order_relaxed);
            RigExecFrameCacheEviction gone;
            gone.key = it->first;
            gone.time = it->second->time;
            evicted.push_back(gone);
            retired.push_back(std::move(it->second));
            it = shard.map.erase(it);
            ++dropped;
        }
    }
    // Payloads die here, outside every lock.
    _ReportEvictions(std::move(evicted));
    return dropped;
}

void
RigExecFrameCache::Clear()
{
    // Accounted per shard, under that shard's lock, exactly like the
    // eviction paths: a background Publish landing in an already-cleared
    // shard keeps its entry AND its bytes, so the counters stay honest. A
    // blanket store(0) here would orphan that entry's bytes and let its
    // later eviction wrap _bytes below zero, after which every publish
    // would drain the store. Payloads die after the last lock closes.
    std::vector<std::shared_ptr<_Entry>> retired;
    std::vector<RigExecFrameCacheEviction> evicted;
    for (_Shard &shard : _shards) {
        std::lock_guard<std::mutex> lock(shard.mutex);
        for (auto &kv : shard.map) {
            _bytes.fetch_sub(kv.second->bytes, std::memory_order_relaxed);
            _entryCount.fetch_sub(1, std::memory_order_relaxed);
            RigExecFrameCacheEviction gone;
            gone.key = kv.first;
            gone.time = kv.second->time;
            evicted.push_back(gone);
            retired.push_back(std::move(kv.second));
        }
        shard.map.clear();
    }
    _hits.store(0, std::memory_order_relaxed);
    _misses.store(0, std::memory_order_relaxed);
    _published.store(0, std::memory_order_relaxed);
    _evictions.store(0, std::memory_order_relaxed);
    _ReportEvictions(std::move(evicted));
}

void
RigExecFrameCache::SetByteCap(size_t bytes)
{
    _byteCap.store(bytes, std::memory_order_relaxed);
    _EvictUntilUnderCap();
}

size_t
RigExecFrameCache::GetByteCap() const
{
    return _byteCap.load(std::memory_order_relaxed);
}

RigExecFrameCacheStats
RigExecFrameCache::Stats() const
{
    RigExecFrameCacheStats stats;
    stats.hits = _hits.load(std::memory_order_relaxed);
    stats.misses = _misses.load(std::memory_order_relaxed);
    stats.published = _published.load(std::memory_order_relaxed);
    stats.evictions = _evictions.load(std::memory_order_relaxed);
    stats.entryCount = _entryCount.load(std::memory_order_relaxed);
    stats.bytes = _bytes.load(std::memory_order_relaxed);
    return stats;
}

void
RigExecFrameCache::_EvictUntilUnderCap()
{
    const size_t cap = _byteCap.load(std::memory_order_relaxed);
    // Each pass drops at most one entry; a pass that raced a touch rescans.
    // Bounded so a publisher racing us cannot spin the loop: whatever still
    // exceeds the cap is left for the next Publish or SetByteCap.
    size_t guard =
        _entryCount.load(std::memory_order_relaxed) + _kShardCount;
    std::vector<RigExecFrameCacheEviction> evicted;
    while (guard-- > 0) {
        if (_bytes.load(std::memory_order_relaxed) <= cap) {
            _ReportEvictions(std::move(evicted));
            return;
        }
        bool found = false;
        RigExecFrameCacheKey victim{0, 0};
        uint64_t victimUse = 0;
        for (_Shard &shard : _shards) {
            std::lock_guard<std::mutex> lock(shard.mutex);
            for (const auto &kv : shard.map) {
                const uint64_t use = kv.second->lastUse.load(
                    std::memory_order_relaxed);
                if (!found || use < victimUse) {
                    victim = kv.first;
                    victimUse = use;
                    found = true;
                }
            }
        }
        if (!found) {
            _ReportEvictions(std::move(evicted));
            return;
        }
        std::shared_ptr<_Entry> retired;
        {
            _Shard &shard = _GetShard(victim);
            std::lock_guard<std::mutex> lock(shard.mutex);
            const auto it = shard.map.find(victim);
            if (it == shard.map.end()) {
                continue;
            }
            if (it->second->lastUse.load(std::memory_order_relaxed) !=
                victimUse) {
                continue;
            }
            _bytes.fetch_sub(it->second->bytes, std::memory_order_relaxed);
            _entryCount.fetch_sub(1, std::memory_order_relaxed);
            _evictions.fetch_add(1, std::memory_order_relaxed);
            RigExecFrameCacheEviction gone;
            gone.key = victim;
            gone.time = it->second->time;
            evicted.push_back(gone);
            retired = std::move(it->second);
            shard.map.erase(it);
        }
        // The victim's payload dies here, outside the lock.
    }
    _ReportEvictions(std::move(evicted));
}

}  // namespace rigExec
