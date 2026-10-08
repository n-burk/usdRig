// .rigexec static capture.
#include "rigExecBake/staticCapture.h"
#include "rigExecBake/computedCapture.h"
#include "rigExecBake/pathTable.h"
#include "rigExec/bakedProgramImpl.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec2f.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/gf/vec3i.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/types.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <algorithm>
#include <cstring>
#include <memory>
#include <set>
#include <utility>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {
namespace {

RigExecWireMatrix4d
_ToMatrix(const GfMatrix4d &m)
{
    RigExecWireMatrix4d out;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            out[size_t(r * 4 + c)] = m[r][c];
        }
    }
    return out;
}

RigExecWireVec3d
_ToVec3d(const GfVec3d &v)
{
    return RigExecWireVec3d{v[0], v[1], v[2]};
}

RigExecWireFrame
_ToFrame(const RigExecPointFrame &f)
{
    RigExecWireFrame out;
    for (size_t i = 0; i < 4; ++i) {
        out.points[i] = _ToVec3d(f.points[i]);
    }
    out.flags = f.flags;
    return out;
}

/// \p member's bytes, or a default-constructed (zero) value's when absent.
template <class T>
void
_AppendMember(const std::unique_ptr<T> &member, std::string *key)
{
    const T value = member ? *member : T{};
    key->append(reinterpret_cast<const char *>(value.data()),
                sizeof(value[0]) * value.size());
}

/// \p member's value, or zero when absent.
template <class T>
std::unique_ptr<T>
_CopyMember(const std::unique_ptr<T> &member)
{
    return std::make_unique<T>(member ? *member : T{});
}

/// The id of the \p count elements at \p data in \p pool, appended when no
/// entry holds the same bytes. \p Element is the pool's element type, built
/// from \p Arity consecutive scalars.
template <class Element, size_t Arity, class Scalar, class Pool>
uint32_t
_Intern(std::map<std::string, uint32_t> *ids, std::vector<Pool> *pool,
        const Scalar *data, size_t count)
{
    std::string key;
    if (count > 0) {
        key.assign(reinterpret_cast<const char *>(data),
                   sizeof(Scalar) * Arity * count);
    }
    const auto found = ids->emplace(std::move(key), uint32_t(pool->size()));
    if (found.second) {
        Pool entry;
        entry.v.resize(count);
        for (size_t i = 0; i < count; ++i) {
            std::memcpy(&entry.v[i], data + i * Arity,
                        sizeof(Scalar) * Arity);
        }
        pool->push_back(std::move(entry));
    }
    return found.first->second;
}

/// A stage value as a path read's value row holds it: an empty value is
/// Absent, a token is its Token node, an array its pool entry. False for a
/// type no row can hold.
bool
_EncodePathValue(const VtValue &held, RigExecBakePathTable *paths,
                 RigExecBakePools *pools, fb::RigExecWirePathValue *out)
{
    using Tag = fb::PathTag;
    if (held.IsEmpty()) {
        out->tag = Tag::Absent;
    } else if (held.IsHolding<bool>()) {
        out->tag = Tag::Bool;
        out->bits = held.UncheckedGet<bool>() ? 1 : 0;
    } else if (held.IsHolding<int>()) {
        out->tag = Tag::Int;
        out->bits = uint32_t(int32_t(held.UncheckedGet<int>()));
    } else if (held.IsHolding<float>()) {
        out->tag = Tag::Float;
        const float f = held.UncheckedGet<float>();
        uint32_t bits = 0;
        std::memcpy(&bits, &f, sizeof(f));
        out->bits = bits;
    } else if (held.IsHolding<double>()) {
        out->tag = Tag::Double;
        const double d = held.UncheckedGet<double>();
        std::memcpy(&out->bits, &d, sizeof(d));
    } else if (held.IsHolding<TfToken>()) {
        out->tag = Tag::Token;
        out->bits = paths->Token(held.UncheckedGet<TfToken>());
    } else if (held.IsHolding<GfMatrix4d>()) {
        out->tag = Tag::Matrix4d;
        out->matrix = std::make_unique<RigExecWireMatrix4d>(
            _ToMatrix(held.UncheckedGet<GfMatrix4d>()));
    } else if (held.IsHolding<GfVec3d>()) {
        out->tag = Tag::Vec3d;
        out->vec3d = std::make_unique<RigExecWireVec3d>(
            _ToVec3d(held.UncheckedGet<GfVec3d>()));
    } else if (held.IsHolding<VtIntArray>()) {
        out->tag = Tag::IntArray;
        const VtIntArray &array = held.UncheckedGet<VtIntArray>();
        const std::vector<int32_t> ints(array.begin(), array.end());
        out->array = pools->Ints(ints.data(), ints.size());
    } else if (held.IsHolding<VtFloatArray>()) {
        out->tag = Tag::FloatArray;
        const VtFloatArray &array = held.UncheckedGet<VtFloatArray>();
        out->array = pools->Floats(array.cdata(), array.size());
    } else if (held.IsHolding<VtArray<GfVec2f>>()) {
        out->tag = Tag::Vec2fArray;
        const VtArray<GfVec2f> &array = held.UncheckedGet<VtArray<GfVec2f>>();
        std::vector<float> xy;
        xy.reserve(array.size() * 2);
        for (const GfVec2f &v : array) {
            xy.push_back(v[0]);
            xy.push_back(v[1]);
        }
        out->array = pools->Vec2fs(xy.data(), array.size());
    } else if (held.IsHolding<VtVec3fArray>()) {
        out->tag = Tag::Vec3fArray;
        const VtVec3fArray &array = held.UncheckedGet<VtVec3fArray>();
        std::vector<float> xyz;
        xyz.reserve(array.size() * 3);
        for (const GfVec3f &v : array) {
            xyz.push_back(v[0]);
            xyz.push_back(v[1]);
            xyz.push_back(v[2]);
        }
        out->array = pools->Vec3fs(xyz.data(), array.size());
    } else if (held.IsHolding<GfVec3i>()) {
        out->tag = Tag::Vec3i;
        const GfVec3i &v = held.UncheckedGet<GfVec3i>();
        out->vec3i = std::make_unique<RigExecWireVec3i>(
            RigExecWireVec3i{{int32_t(v[0]), int32_t(v[1]), int32_t(v[2])}});
    } else if (held.IsHolding<VtDoubleArray>()) {
        out->tag = Tag::DoubleArray;
        const VtDoubleArray &array = held.UncheckedGet<VtDoubleArray>();
        out->array = pools->Doubles(array.cdata(), array.size());
    } else {
        return false;
    }
    return true;
}

/// \p points as float triples, in order.
template <class Points>
std::vector<float>
_Xyz(const Points &points)
{
    std::vector<float> xyz;
    xyz.reserve(points.size() * 3);
    for (const GfVec3f &p : points) {
        xyz.push_back(p[0]);
        xyz.push_back(p[1]);
        xyz.push_back(p[2]);
    }
    return xyz;
}

template <class Points>
uint32_t
_PoolPoints(RigExecBakePools *pools, const Points &points)
{
    const std::vector<float> xyz = _Xyz(points);
    return pools->Vec3fs(xyz.data(), points.size());
}

/// The PathValue tag of a static array of input tag \p tag.
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

/// Whether \p a and \p b hold the same elements, bit for bit.
template <class A, class B>
bool
_SameBytes(const A &a, const B &b)
{
    return a.size() == b.size() &&
           (a.empty() || std::memcmp(a.data(), b.data(),
                                     sizeof(a[0]) * a.size()) == 0);
}

/// The defaults of \p inputs' array slots, which the bake's run decides,
/// patched into \p file's slots with their HasValue: a chain's base as the
/// pool entry the chain stores, a fixed skin layout's array as the stored
/// layout's expansion when the run stored one, every other the attribute's
/// value at \p time, or at Default where every read of it is at Default.
/// Each is the stage's array bit for bit. False, naming the input, when
/// the run left a base or a layout other than its attributes' values.
bool
_ArrayDefaults(const RigExecBakedProgramImpl &program, double time,
               const RigExecBakeInputs &inputs, RigExecBakePathTable *paths,
               RigExecBakePools *pools, fb::RigExecWireFile *file,
               std::string *error)
{
    const fb::RigExecWireDomainGeometry &geometry = *file->geometry;
    for (const RigExecBakeArraySlot &entry : inputs.arraySlots) {
        if (entry.slot >= file->inputs.size()) {
            *error = "array input " + std::to_string(entry.slot) +
                     " is not a slot of the file";
            return false;
        }
        const fb::InputSlot slot = file->inputs[entry.slot];
        const std::string name = paths->Text(slot.name());
        const UsdAttribute a =
            program.stage->GetAttributeAtPath(SdfPath(name));
        const UsdTimeCode at =
            entry.atDefault ? UsdTimeCode::Default() : UsdTimeCode(time);
        fb::RigExecWireValue value;
        value.tag = entry.tag;
        bool has = false;
        VtIntArray ints;
        VtFloatArray floats;
        VtDoubleArray doubles;
        VtVec2fArray pairs;
        VtVec3fArray points;
        VtVec3dArray vectors; VtMatrix4dArray matrices; VtTokenArray tokens; VtBoolArray bools;
        switch (entry.tag) {
        case fb::InputTag::IntArray:
            has = a && a.Get(&ints, at);
            break;
        case fb::InputTag::FloatArray:
            has = a && a.Get(&floats, at);
            break;
        case fb::InputTag::DoubleArray:
            has = a && a.Get(&doubles, at);
            break;
        case fb::InputTag::Vec2fArray:
            has = a && a.Get(&pairs, at);
            break;
        case fb::InputTag::Vec3fArray:
            has = a && a.Get(&points, at);
            break;
        case fb::InputTag::Vec3dArray: has=a && a.Get(&vectors,at); break;
        case fb::InputTag::Matrix4dArray: has=a && a.Get(&matrices,at); break;
        case fb::InputTag::TokenArray: has=a && a.Get(&tokens,at); break;
        case fb::InputTag::BoolArray: has=a && a.Get(&bools,at); break;
        default:
            *error = "input " + name + " is no array input";
            return false;
        }
        // A stored layout is its arrays' one copy.
        bool stored = false;
        if (entry.layoutChain >= 0) {
            const size_t c = size_t(entry.layoutChain);
            const size_t r = size_t(entry.layoutRevision);
            const RigExecBakedProgramImpl::GeomRevision &revision =
                program.chains[c].revisions[r];
            stored = revision.topologyResolved && revision.topology;
            if (stored) {
                const bool same =
                    entry.layoutIndices
                        ? _SameBytes(revision.topology->indices, ints)
                        : _SameBytes(revision.topology->weights, floats);
                if (!same) {
                    *error = "the layout of " +
                             revision.moverPath.GetString() +
                             " is not its inputs' values at the bake time";
                    return false;
                }
                size_t flat = r;
                for (size_t k = 0; k < c; ++k) {
                    flat += geometry.chains[k].revisions.size();
                }
                value.arraySource = entry.layoutIndices
                                        ? fb::ArraySource::SkinIndices
                                        : fb::ArraySource::SkinWeights;
                value.array = uint32_t(flat);
            }
        }
        if (!stored) {
            switch (entry.tag) {
            case fb::InputTag::IntArray: {
                const std::vector<int32_t> held(ints.begin(), ints.end());
                value.array = pools->Ints(held.data(), held.size());
                break;
            }
            case fb::InputTag::FloatArray:
                value.array = pools->Floats(floats.cdata(), floats.size());
                break;
            case fb::InputTag::DoubleArray:
                value.array = pools->Doubles(doubles.cdata(), doubles.size());
                break;
            case fb::InputTag::Vec2fArray:
                value.array = pools->Vec2fs(
                    reinterpret_cast<const float *>(pairs.cdata()),
                    pairs.size());
                break;
            case fb::InputTag::Vec3dArray:
                value.array=pools->Vec3ds(reinterpret_cast<const double *>(vectors.cdata()),vectors.size()); break;
            case fb::InputTag::Matrix4dArray:
                value.array=pools->Matrices(reinterpret_cast<const double *>(matrices.cdata()),matrices.size()); break;
            case fb::InputTag::TokenArray: {
                std::vector<std::string> text; for(const auto &token:tokens) text.push_back(token.GetString());
                value.array=pools->Tokens(text); break;
            }
            case fb::InputTag::BoolArray: {
                std::vector<uint8_t> bits; for(bool bit:bools) bits.push_back(bit?1:0);
                value.array=pools->Bools(bits.data(),bits.size()); break;
            }
            default:
                value.array = pools->Vec3fs(
                    reinterpret_cast<const float *>(points.cdata()),
                    points.size());
                break;
            }
        }
        if (entry.chain >= 0) {
            const size_t c = size_t(entry.chain);
            const RigExecBakedProgramImpl::GeomChain &chain =
                program.chains[c];
            if (!has || !chain.haveBase ||
                !_SameBytes(points, chain.lastBase) ||
                value.array != geometry.chains[c].base) {
                *error = "the base of chain " + chain.target.GetString() +
                         " is not its input's value at the bake time";
                return false;
            }
        }
        const uint8_t hasValue = uint8_t(fb::InputSlotFlags::HasValue);
        file->inputs[entry.slot] = fb::InputSlot(
            slot.name(), pools->Value(value), slot.chain(), slot.phased(),
            slot.type(),
            uint8_t((slot.flags() & ~hasValue) | (has ? hasValue : 0)));
    }
    return true;
}

}  // namespace

std::string
RigExecBakeValueKey(const fb::RigExecWireValue &value)
{
    std::string key(1, char(value.tag));
    key.append(reinterpret_cast<const char *>(&value.bits),
               sizeof(value.bits));
    switch (value.tag) {
    case fb::InputTag::Matrix4d:
        _AppendMember(value.matrix, &key);
        break;
    case fb::InputTag::Vec3d:
        _AppendMember(value.vec3d, &key);
        break;
    case fb::InputTag::Vec3i:
        _AppendMember(value.vec3i,&key);
        break;
    case fb::InputTag::Vec3f:
        _AppendMember(value.vec3f, &key);
        break;
    default:
        // An array value names its elements by source and id.
        if (RigExecFormatIsArrayTag(value.tag)) {
            key.push_back(char(value.arraySource));
            key.append(reinterpret_cast<const char *>(&value.array),
                       sizeof(value.array));
        }
        break;
    }
    return key;
}

RigExecBakePools::RigExecBakePools()
{
    fb::RigExecWireValue zero;
    zero.tag = fb::InputTag::Double;
    Value(zero);
    const float none = 0.0f;
    Ints(nullptr, 0);
    Floats(&none, 0);
    Doubles(nullptr, 0);
    Vec2fs(&none, 0);
    Vec3fs(&none, 0);
    Vec3ds(nullptr,0); Matrices(nullptr,0); Tokens({}); Bools(nullptr,0);
}

uint32_t
RigExecBakePools::Value(const fb::RigExecWireValue &value)
{
    const auto found = _valueIds.emplace(RigExecBakeValueKey(value),
                                         uint32_t(_values.size()));
    if (found.second) {
        fb::RigExecWireValue out;
        out.tag = value.tag;
        out.bits = value.bits;
        out.arraySource = value.arraySource;
        out.array = value.array;
        switch (value.tag) {
        case fb::InputTag::Matrix4d:
            out.matrix = _CopyMember(value.matrix);
            break;
        case fb::InputTag::Vec3d:
            out.vec3d = _CopyMember(value.vec3d);
            break;
        case fb::InputTag::Vec3f:
            out.vec3f = _CopyMember(value.vec3f);
            break;
        case fb::InputTag::Vec3i:
            out.vec3i = _CopyMember(value.vec3i);
            break;
        default:
            break;
        }
        _values.push_back(std::move(out));
    }
    return found.first->second;
}

uint32_t
RigExecBakePools::Ints(const int32_t *data, size_t count)
{
    return _Intern<int32_t, 1>(&_intIds, &_ints, data, count);
}

uint32_t
RigExecBakePools::Floats(const float *data, size_t count)
{
    return _Intern<float, 1>(&_floatIds, &_floats, data, count);
}

uint32_t
RigExecBakePools::Doubles(const double *data, size_t count)
{
    return _Intern<double, 1>(&_doubleIds, &_doubles, data, count);
}

uint32_t
RigExecBakePools::Vec2fs(const float *xy, size_t count)
{
    return _Intern<RigExecWireVec2f, 2>(&_vec2fIds, &_vec2fs, xy, count);
}

uint32_t
RigExecBakePools::Vec3fs(const float *xyz, size_t count)
{
    return _Intern<RigExecWireVec3f, 3>(&_vec3fIds, &_vec3fs, xyz, count);
}

bool
RigExecBakePools::Seed(const RigExecBakeInputs &inputs, std::string *error)
{
    const auto fail = [&](const std::string &what) {
        if (error) {
            *error = what;
        }
        return false;
    };
    if (_values.size() != 1) {
        return fail("the pools already hold entries of their own");
    }
    for (size_t i = 0; i < inputs.values.size(); ++i) {
        if (Value(inputs.values[i]) != i) {
            return fail("value " + std::to_string(i) +
                        " is not where the capture put it");
        }
    }
    return true;
}

uint32_t RigExecBakePools::Vec3ds(const double *data,size_t count) {
    std::string key(1,char(13));
    if(count) key.append(reinterpret_cast<const char *>(data),count*3*sizeof(double));
    const auto found=_extraIds.find(key); if(found!=_extraIds.end()) return found->second;
    fb::RigExecWireVec3dArray row;
    row.v.resize(count);
    for(size_t i=0;i<count;++i) std::copy(data+i*3,data+(i+1)*3,row.v[i].begin());
    const uint32_t id=uint32_t(_vec3ds.size()); _vec3ds.push_back(std::move(row));
    _extraIds.emplace(std::move(key),id); return id;
}
uint32_t RigExecBakePools::Matrices(const double *data,size_t count) {
    std::string key(1,char(14));
    if(count) key.append(reinterpret_cast<const char *>(data),count*16*sizeof(double));
    const auto found=_extraIds.find(key); if(found!=_extraIds.end()) return found->second;
    fb::RigExecWireMatrix4dArray row;
    row.v.resize(count);
    for(size_t i=0;i<count;++i) std::copy(data+i*16,data+(i+1)*16,row.v[i].begin());
    const uint32_t id=uint32_t(_matrices.size()); _matrices.push_back(std::move(row));
    _extraIds.emplace(std::move(key),id); return id;
}
uint32_t RigExecBakePools::Tokens(const std::vector<std::string> &data) {
    std::string key(1,char(15));
    for(const auto &text:data) { const uint64_t count=text.size();
        key.append(reinterpret_cast<const char *>(&count),sizeof(count)); key.append(text); }
    const auto found=_extraIds.find(key); if(found!=_extraIds.end()) return found->second;
    fb::RigExecWireTokenArray row; row.v=data;
    const uint32_t id=uint32_t(_tokens.size()); _tokens.push_back(std::move(row));
    _extraIds.emplace(std::move(key),id); return id;
}
uint32_t RigExecBakePools::Bools(const uint8_t *data,size_t count) {
    std::string key(1,char(16)); if(count) key.append(reinterpret_cast<const char *>(data),count);
    const auto found=_extraIds.find(key); if(found!=_extraIds.end()) return found->second;
    fb::RigExecWireBoolArray row; if(count) row.v.assign(data,data+count);
    const uint32_t id=uint32_t(_bools.size()); _bools.push_back(std::move(row));
    _extraIds.emplace(std::move(key),id); return id;
}

void
RigExecBakePools::MoveInto(fb::RigExecWireFile *file)
{
    file->values = std::move(_values);
    file->intArrays = std::move(_ints);
    file->floatArrays = std::move(_floats);
    file->doubleArrays = std::move(_doubles);
    file->vec2fArrays = std::move(_vec2fs);
    file->vec3fArrays = std::move(_vec3fs);
    file->vec3dArrays=std::move(_vec3ds); file->matrix4dArrays=std::move(_matrices);
    file->tokenArrays=std::move(_tokens); file->boolArrays=std::move(_bools);
    _values.clear();
    _ints.clear();
    _floats.clear();
    _doubles.clear();
    _vec2fs.clear();
    _vec3fs.clear();
    _valueIds.clear();
    _intIds.clear();
    _floatIds.clear();
    _doubleIds.clear();
    _vec2fIds.clear();
    _vec3fIds.clear();
}

bool
RigExecBakeCaptureStatics(
    const RigExecBakedProgramImpl &program, const RigExecBakeInputs &inputs,
    const std::vector<RigExecBakeRevisionRead> &enumerated,
    RigExecBakePathTable *paths, RigExecBakePools *pools,
    fb::RigExecWireFile *file, std::string *error)
{
    const auto fail = [&](const std::string &what) {
        if (error) {
            *error = what;
        }
        return false;
    };
    if (!paths || !pools || !file || !file->pose || !file->geometry) {
        return fail("the file's tables are not filled");
    }
    fb::RigExecWireDomainPose &pose = *file->pose;
    fb::RigExecWireDomainGeometry &geometry = *file->geometry;

    // Transforms the prologue read off the stage.
    pose.xformBase.clear();
    pose.xformBase.reserve(program.xformBase.size());
    for (const GfMatrix4d &m : program.xformBase) {
        pose.xformBase.push_back(_ToMatrix(m));
    }
    pose.xformFrames.clear();
    pose.xformFrames.reserve(program.xformSlots.size());
    for (int slot : program.xformSlots) {
        pose.xformFrames.push_back(_ToFrame(program.base[size_t(slot)]));
    }
    if (pose.nativeSources.size() != program.nativeFrames.size() ||
        program.nativeFrameOk.size() != program.nativeFrames.size()) {
        return fail("the native sources and the run's frames disagree");
    }
    for (size_t k = 0; k < pose.nativeSources.size(); ++k) {
        pose.nativeSources[k].frame = _ToFrame(program.nativeFrames[k]);
        pose.nativeSources[k].ok = program.nativeFrameOk[k] != 0;
    }
    geometry.deltaBaseMatrix.clear();
    geometry.deltaBaseMatrix.reserve(program.deltaBaseMatrix.size());
    for (const GfMatrix4d &m : program.deltaBaseMatrix) {
        geometry.deltaBaseMatrix.push_back(_ToMatrix(m));
    }
    geometry.deltaBaseOk.clear();
    geometry.deltaBaseOk.reserve(program.deltaBaseOk.size());
    for (char v : program.deltaBaseOk) {
        geometry.deltaBaseOk.push_back(v ? uint8_t(1) : uint8_t(0));
    }

    // The constraint arrays the prologue read, with the lines they
    // reported.
    if (pose.constraintArrays.size() != program.constraintArrays.size()) {
        return fail("the constraint arrays and the program disagree");
    }
    for (size_t k = 0; k < program.constraintArrays.size(); ++k) {
        const RigExecBakedProgramImpl::ConstraintArrays &arrays =
            program.constraintArrays[k];
        fb::RigExecWireConstraintArrays &out = pose.constraintArrays[k];
        out.weights = arrays.weights;
        out.translationOffsets.clear();
        out.translationOffsets.reserve(arrays.translationOffsets.size());
        for (const GfVec3d &v : arrays.translationOffsets) {
            out.translationOffsets.push_back(_ToVec3d(v));
        }
        out.rotationOffsets.clear();
        out.rotationOffsets.reserve(arrays.rotationOffsets.size());
        for (const GfVec3d &v : arrays.rotationOffsets) {
            out.rotationOffsets.push_back(_ToVec3d(v));
        }
        out.ok = arrays.ok;
        out.poleWeights = arrays.poleWeights;
        out.poleOk = arrays.poleOk;
        out.diagnostics = arrays.diagnostics;
        out.poleDiagnostics = arrays.poleDiagnostics;
    }

    // Chain and derived bases; a stale base without its flag is not
    // copied.
    if (geometry.chains.size() != program.chains.size()) {
        return fail("the geometry chains and the program disagree");
    }
    // Every blend sample's dense points as the run's assembly consumed
    // them, and the layout its prologue resolved, a cache-refused one
    // included.
    const auto samples =
        [&](const RigExecBakedProgramImpl::GeomRevision &revision,
            fb::RigExecWireRevision &out) {
            if (out.blendChannels.size() != revision.blendChannels.size()) {
                return fail("the blend channels of " +
                            revision.moverPath.GetString() +
                            " and the program disagree");
            }
            for (size_t ch = 0; ch < revision.blendChannels.size(); ++ch) {
                const auto &channel = revision.blendChannels[ch];
                fb::RigExecWireBlendChannel &wire = out.blendChannels[ch];
                if (wire.samples.size() != channel.samples.size()) {
                    return fail("the blend samples of " +
                                channel.weightPath.GetString() +
                                " and the program disagree");
                }
                for (size_t s = 0; s < channel.samples.size(); ++s) {
                    const auto &sample = channel.samples[s];
                    fb::RigExecWireBlendSample &wireSample = wire.samples[s];
                    wireSample.pointsValue =
                        _PoolPoints(pools, sample.lastPoints);
                    if (!sample.layoutRefused) {
                        continue;
                    }
                    if (sample.blendShape.IsEmpty()) {
                        return fail("a dense blend sample is flagged "
                                    "cache-refused: " +
                                    sample.samplePath.GetString());
                    }
                    if (!sample.layout) {
                        return fail("a refused blend layout has no shape: " +
                                    sample.samplePath.GetString());
                    }
                    wireSample.hasLayout = true;
                    wireSample.offsets.clear();
                    wireSample.offsets.reserve(sample.layout->offsets.size());
                    for (const GfVec3f &p : sample.layout->offsets) {
                        wireSample.offsets.push_back(
                            RigExecWireVec3f{p[0], p[1], p[2]});
                    }
                    wireSample.indices.assign(sample.layout->indices.begin(),
                                              sample.layout->indices.end());
                    wireSample.pointCount =
                        uint64_t(sample.layout->pointCount);
                    wireSample.layoutValid = sample.layout->valid;
                }
            }
            return true;
        };
    for (size_t c = 0; c < program.chains.size(); ++c) {
        const RigExecBakedProgramImpl::GeomChain &chain = program.chains[c];
        fb::RigExecWireChain &wire = geometry.chains[c];
        if (wire.revisions.size() != chain.revisions.size() ||
            wire.derived.size() != chain.derived.size()) {
            return fail("the revisions of chain " +
                        chain.target.GetString() +
                        " and the program disagree");
        }
        wire.haveBase = chain.haveBase;
        wire.base = chain.haveBase ? _PoolPoints(pools, chain.lastBase) : 0;
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            if (!samples(chain.revisions[r], wire.revisions[r])) {
                return false;
            }
        }
        for (size_t d = 0; d < chain.derived.size(); ++d) {
            const RigExecBakedProgramImpl::GeomChain::Derived &derived =
                chain.derived[d];
            fb::RigExecWireDerived &out = wire.derived[d];
            out.haveBase = derived.haveBase;
            out.base =
                derived.haveBase ? _PoolPoints(pools, derived.lastBase) : 0;
            if (!out.revision) {
                return fail("a derived revision of chain " +
                            chain.target.GetString() + " is missing");
            }
            if (!samples(derived.revision, *out.revision)) {
                return false;
            }
        }
    }

    // A varying ribbon driver's points as the run read them.
    if (pose.solvers.size() != program.solvers.size()) {
        return fail("the solvers and the program disagree");
    }
    for (size_t s = 0; s < program.solvers.size(); ++s) {
        const RigExecBakedProgramImpl::Solver &solver = program.solvers[s];
        if (!solver.ribbonPointsVarying) {
            continue;
        }
        std::vector<RigExecWireVec3f> &points =
            pose.solvers[s].ribbonConstantPoints;
        points.clear();
        points.reserve(solver.ribbonPoints.size());
        for (const GfVec3f &p : solver.ribbonPoints) {
            points.push_back(RigExecWireVec3f{p[0], p[1], p[2]});
        }
    }

    if (!_ArrayDefaults(program, time, inputs, paths, pools, file, error)) {
        return false;
    }

    // The path reads: the connection-following scalar reads evaluate over
    // the slots, and so does every key an array read binds (at rest beside
    // the attribute's Default-time value); every other key the assembly
    // can read is a value.
    std::map<std::pair<uint32_t, bool>, const RigExecBakeArrayRow *> bound;
    for (const RigExecBakeArrayRow &row : inputs.arrayRows) {
        bound.emplace(std::make_pair(row.path, row.rest), &row);
    }
    std::vector<fb::RigExecWirePathRead> rows;
    std::set<std::pair<uint32_t, bool>> keys;
    for (const RigExecBakePathScalarRead &entry : inputs.pathScalarReads) {
        if (!keys.emplace(entry.path, false).second) {
            return fail("two connection-following reads of " +
                        paths->Text(entry.path));
        }
        fb::RigExecWirePathRead row;
        row.path = entry.path;
        row.rest = false;
        row.read = std::make_unique<fb::RigExecWireInput>(entry.read);
        row.headFallback = entry.headFallback;
        rows.push_back(std::move(row));
    }
    for (const RigExecBakeRevisionRead &candidate : enumerated) {
        if (candidate.overlaid) {
            continue;
        }
        const uint32_t path = paths->Path(candidate.path);
        if (!keys.emplace(path, candidate.rest).second) {
            continue;
        }
        fb::RigExecWirePathRead row;
        row.path = path;
        row.rest = candidate.rest;
        const auto read = bound.find(std::make_pair(path, candidate.rest));
        if (read != bound.end()) {
            row.read =
                std::make_unique<fb::RigExecWireInput>(read->second->read);
            if (!candidate.rest) {
                rows.push_back(std::move(row));
                continue;
            }
        }
        row.value = std::make_unique<fb::RigExecWirePathValue>();
        if (!_EncodePathValue(candidate.value, paths, pools,
                              row.value.get())) {
            return fail("path read " + candidate.path.GetString() +
                        " enumerated with an unencodable value of type " +
                        candidate.value.GetTypeName());
        }
        if (row.read && row.value->tag != _PathArrayTag(row.read->tag)) {
            return fail("path read " + candidate.path.GetString() +
                        " at rest holds no array of its input's type");
        }
        rows.push_back(std::move(row));
    }
    // A gather whose read answered nothing at the bake time is no key of
    // the enumeration; its row reads its input all the same.
    for (const RigExecBakeArrayRow &gather : inputs.arrayRows) {
        if (!gather.gather || gather.rest ||
            !keys.emplace(gather.path, false).second) {
            continue;
        }
        fb::RigExecWirePathRead row;
        row.path = gather.path;
        row.read = std::make_unique<fb::RigExecWireInput>(gather.read);
        rows.push_back(std::move(row));
    }
    std::sort(rows.begin(), rows.end(),
              [](const fb::RigExecWirePathRead &a,
                 const fb::RigExecWirePathRead &b) {
                  return std::make_pair(a.path, a.rest) <
                         std::make_pair(b.path, b.rest);
              });
    geometry.pathReads = std::move(rows);
    return true;
}

}  // namespace rigExec
