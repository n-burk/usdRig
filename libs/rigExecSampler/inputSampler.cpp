// rigExecSampler: see inputSampler.h for the contract.
#include "rigExecSampler/inputSampler.h"
#include "rigExecRuntime/stageArrayInputs.h"
#include <algorithm>

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/tf/type.h"
#include "pxr/base/vt/value.h"
#include "pxr/base/vt/types.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/sdf/valueTypeName.h"

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

TfType
RigExecInputTagType(RrInputTag tag)
{
    switch (tag) {
    case RrInputTag::Double:
        return TfType::Find<double>();
    case RrInputTag::Float:
        return TfType::Find<float>();
    case RrInputTag::Bool:
        return TfType::Find<bool>();
    case RrInputTag::Int:
        return TfType::Find<int>();
    case RrInputTag::Matrix4d:
        return TfType::Find<GfMatrix4d>();
    case RrInputTag::Token:
        return TfType::Find<TfToken>();
    case RrInputTag::Vec3d:
        return TfType::Find<GfVec3d>();
    case RrInputTag::Vec3f:
        return TfType::Find<GfVec3f>();
    case RrInputTag::IntArray:
        return TfType::Find<VtIntArray>();
    case RrInputTag::FloatArray:
        return TfType::Find<VtFloatArray>();
    case RrInputTag::DoubleArray:
        return TfType::Find<VtDoubleArray>();
    case RrInputTag::Vec2fArray:
        return TfType::Find<VtVec2fArray>();
    case RrInputTag::Vec3fArray:
        return TfType::Find<VtVec3fArray>();
    default:
        break;
    }
    return TfType();
}

const char *
RigExecInputTagName(RrInputTag tag)
{
    switch (tag) {
    case RrInputTag::Double:
        return "double";
    case RrInputTag::Float:
        return "float";
    case RrInputTag::Bool:
        return "bool";
    case RrInputTag::Int:
        return "int";
    case RrInputTag::Matrix4d:
        return "matrix4d";
    case RrInputTag::Token:
        return "token";
    case RrInputTag::Vec3d:
        return "vec3d";
    case RrInputTag::Vec3f:
        return "vec3f";
    case RrInputTag::IntArray:
        return "int[]";
    case RrInputTag::FloatArray:
        return "float[]";
    case RrInputTag::DoubleArray:
        return "double[]";
    case RrInputTag::Vec2fArray:
        return "float2[]";
    case RrInputTag::Vec3fArray:
        return "float3[]";
    default:
        break;
    }
    return "unknown";
}

bool
RigExecInputValueFrom(const VtValue &value, RrInputTag tag,
                      RrInputValue *out)
{
    if (!out || tag == RrInputTag::Token || RrInputTagIsArray(tag) ||
        value.GetType() != RigExecInputTagType(tag)) {
        return false;
    }
    RrInputValue v;
    v.tag = tag;
    switch (tag) {
    case RrInputTag::Double:
        v.f64 = value.UncheckedGet<double>();
        break;
    case RrInputTag::Float:
        v.f32 = value.UncheckedGet<float>();
        break;
    case RrInputTag::Bool:
        v.boolean = value.UncheckedGet<bool>();
        break;
    case RrInputTag::Int:
        v.i32 = int32_t(value.UncheckedGet<int>());
        break;
    case RrInputTag::Matrix4d: {
        const GfMatrix4d &m = value.UncheckedGet<GfMatrix4d>();
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                v.matrix[r][c] = m[r][c];
            }
        }
        break;
    }
    case RrInputTag::Vec3d: {
        const GfVec3d &d = value.UncheckedGet<GfVec3d>();
        v.vec = RrVec3d(d[0], d[1], d[2]);
        break;
    }
    case RrInputTag::Vec3f: {
        const GfVec3f &f = value.UncheckedGet<GfVec3f>();
        v.vec3f = RrVec3f(f[0], f[1], f[2]);
        break;
    }
    case RrInputTag::Token:
        return false;
    }
    *out = v;
    return true;
}

bool
RigExecInputArrayFrom(const VtValue &value, RrInputTag tag,
                      RigExecRuntimeArray *out)
{
    if (!out || !RrInputTagIsArray(tag) ||
        value.GetType() != RigExecInputTagType(tag)) {
        return false;
    }
    out->tag = tag;
    out->count = value.GetArraySize();
    switch (tag) {
    case RrInputTag::IntArray:
        out->data = value.UncheckedGet<VtIntArray>().cdata();
        return true;
    case RrInputTag::FloatArray:
        out->data = value.UncheckedGet<VtFloatArray>().cdata();
        return true;
    case RrInputTag::DoubleArray:
        out->data = value.UncheckedGet<VtDoubleArray>().cdata();
        return true;
    case RrInputTag::Vec2fArray:
        out->data = value.UncheckedGet<VtVec2fArray>().cdata();
        return true;
    case RrInputTag::Vec3fArray:
        out->data = value.UncheckedGet<VtVec3fArray>().cdata();
        return true;
    default:
        return false;
    }
}

bool
RigExecSampleInputAt(const UsdAttribute &a, size_t index,
                     const std::string &name, RrInputTag tag,
                     UsdTimeCode time, RigExecRuntimeReader *reader,
                     std::string *error)
{
    if (RrInputTagIsArray(tag)) {
        const bool bound =
            RigExecRuntimeStageArrayInputs::CanSample(*reader, index);
        if (!bound) {
            return reader->ResetInput(name, error);
        }
        VtValue value;
        if (!a.Get(&value, time)) {
            return RigExecRuntimeStageArrayInputs::ClearSample(*reader, index, error);
        }
        RigExecRuntimeArray array;
        if (!RigExecInputArrayFrom(value, tag, &array)) {
            if (error) *error = name + ": the stage sample is not a " +
                                RigExecInputTagName(tag);
            return false;
        }
        return RigExecRuntimeStageArrayInputs::SetSample(
            *reader, index, array, error);
    }
    RrInputValue value;
    value.tag = tag;
    bool read = false;
    switch (tag) {
    case RrInputTag::Double:
        read = a.Get(&value.f64, time);
        break;
    case RrInputTag::Float:
        read = a.Get(&value.f32, time);
        break;
    case RrInputTag::Bool:
        read = a.Get(&value.boolean, time);
        break;
    case RrInputTag::Int: {
        int v = 0;
        read = a.Get(&v, time);
        value.i32 = int32_t(v);
        break;
    }
    case RrInputTag::Matrix4d: {
        GfMatrix4d v(1.0);
        if ((read = a.Get(&v, time))) {
            for (int r = 0; r < 4; ++r) {
                for (int c = 0; c < 4; ++c) {
                    value.matrix[r][c] = v[r][c];
                }
            }
        }
        break;
    }
    case RrInputTag::Token: {
        TfToken v;
        if (a.Get(&v, time)) {
            return reader->SetInputToken(name, v.GetString(), error);
        }
        break;
    }
    case RrInputTag::Vec3d: {
        GfVec3d v(0.0);
        if ((read = a.Get(&v, time))) {
            value.vec = RrVec3d(v[0], v[1], v[2]);
        }
        break;
    }
    case RrInputTag::Vec3f: {
        GfVec3f v(0.0f);
        if ((read = a.Get(&v, time))) {
            value.vec3f = RrVec3f(v[0], v[1], v[2]);
        }
        break;
    }
    }
    // A failed read leaves the input with no value, as the stage has none
    // there; a value is taken as the stage holds it, finite or not.
    return read ? reader->SetSampledInputAt(index, value, error)
                : reader->ClearInputAt(index, error);
}

bool
RigExecInputSampler::Bind(const UsdStagePtr &stage,
                          const RigExecRuntimeReader &reader,
                          std::string *error)
{
    _animated.clear();
    _warnings.clear();
    _sampled = false;
    if (!stage) {
        if (error) {
            *error = "the input sampler has no stage to read";
        }
        return false;
    }
    const size_t count = reader.GetInputCount();
    for (size_t i = 0; i < count; ++i) {
        const RigExecRuntimeInputInfo &info = reader.GetInputInfo(i);
        // Array bindings include private slots through the stage transport below.
        if (RrInputTagIsArray(info.type)) {
            continue;
        }
        const SdfPath path = SdfPath::IsValidPathString(info.name)
                                 ? SdfPath(info.name)
                                 : SdfPath();
        const UsdAttribute attribute =
            path.IsPropertyPath() ? stage->GetAttributeAtPath(path)
                                  : UsdAttribute();
        if (!attribute) {
            _warnings.push_back(info.name +
                                ": the stage has no such attribute");
            continue;
        }
        const SdfValueTypeName typeName = attribute.GetTypeName();
        if (!typeName || typeName.IsArray() ||
            typeName.GetType() != RigExecInputTagType(info.type)) {
            _warnings.push_back(info.name + ": the stage's attribute is a " +
                                typeName.GetAsToken().GetString() +
                                ", the input a " + RigExecInputTagName(info.type));
            continue;
        }
        if (info.animated) {
            _Bound bound;
            bound.index = i;
            bound.name = info.name;
            bound.type = info.type;
            bound.attribute = attribute;
            _animated.push_back(std::move(bound));
        }
    }
    for (const auto &info : RigExecRuntimeStageArrayInputs::Enumerate(reader)) {
        const SdfPath path(info.name);
        const auto attribute = stage->GetAttributeAtPath(path);
        if (!attribute || attribute.GetTypeName().GetType() !=
                              RigExecInputTagType(info.tag)) {
            _warnings.push_back(info.name + ": no matching stage array");
            continue;
        }
        _Bound bound;
        bound.index = info.slot;
        bound.name = info.name;
        bound.type = info.tag;
        bound.attribute = attribute;
        _animated.push_back(std::move(bound));
    }
    _last = UsdTimeCode(reader.GetBakeTime());
    _sampled = true;
    return true;
}

bool
RigExecInputSampler::Apply(UsdTimeCode time, RigExecRuntimeReader *reader,
                           std::string *error, bool *sampled)
{
    if (sampled) {
        *sampled = false;
    }
    if (_sampled && time == _last) {
        return true;
    }
    for (const _Bound &bound : _animated) {
        if (!RigExecSampleInputAt(bound.attribute, bound.index, bound.name,
                                  bound.type, time, reader, error)) {
            return false;
        }
    }
    reader->TouchAnimatedInputs();
    _last = time;
    _sampled = true;
    if (sampled) {
        *sampled = true;
    }
    return true;
}

}  // namespace rigExec
