// A built-in external mover's .rigexec playback through its own stage-side
// callbacks: the declared inputs and provider values a playback kernel
// receives (rigExecBinary/external.h), turned back into what the mover's
// assembleExternal reads, so playback runs the same assembly and
// deformation as native and frozen evaluation.
#ifndef RIGEXEC_MOVERS_EXTERNAL_PLAYBACK_H
#define RIGEXEC_MOVERS_EXTERNAL_PLAYBACK_H

#include "moverRegistry.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/types.h"
#include "pxr/base/vt/value.h"

#include <cstring>
#include <string>
#include <vector>

namespace rigExec {

/// \p count declared inputs as the assembly's values, in declaration order:
/// each by its type tag, a token as its std::string text (playback builds
/// no TfToken), and an unavailable input empty.
inline std::vector<VtValue>
RigExecExternalPlaybackInputs(const RigExecExternalInputValue *inputs,
                              size_t count)
{
    using Type = RigExecExternalInputType;
    std::vector<VtValue> values(count);
    for (size_t k = 0; k < count; ++k) {
        const RigExecExternalInputValue &input = inputs[k];
        if (!input.hasValue || (!input.data && input.count > 0)) {
            continue;
        }
        const size_t n = input.count;
        switch (Type(input.type)) {
        case Type::Double: {
            double v; std::memcpy(&v, input.data, sizeof(v)); values[k] = VtValue(v); break;
        }
        case Type::Float: {
            float v; std::memcpy(&v, input.data, sizeof(v)); values[k] = VtValue(v); break;
        }
        case Type::Bool:
            values[k] = VtValue(*static_cast<const uint8_t *>(input.data) != 0); break;
        case Type::Int: {
            int32_t v; std::memcpy(&v, input.data, sizeof(v)); values[k] = VtValue(int(v)); break;
        }
        case Type::Token:
            values[k] = VtValue(std::string(static_cast<const char *>(input.data))); break;
        case Type::Matrix4d: {
            double m[4][4]; std::memcpy(m, input.data, sizeof(m));
            values[k] = VtValue(GfMatrix4d(m)); break;
        }
        case Type::IntArray: {
            const auto *v = static_cast<const int32_t *>(input.data);
            values[k] = VtValue(VtIntArray(v, v + n)); break;
        }
        case Type::FloatArray: {
            const auto *v = static_cast<const float *>(input.data);
            values[k] = VtValue(VtFloatArray(v, v + n)); break;
        }
        case Type::Vec3fArray: {
            const auto *v = static_cast<const float *>(input.data);
            VtVec3fArray points(n);
            for (size_t i = 0; i < n; ++i) points[i] = GfVec3f(v[i * 3], v[i * 3 + 1], v[i * 3 + 2]);
            values[k] = VtValue(points); break;
        }
        default:
            break;
        }
    }
    return values;
}

/// The provider values a playback kernel received, owned in the assembly's
/// form: the transform and influences when named, and the base points.
struct RigExecExternalPlaybackProviders {
    explicit RigExecExternalPlaybackProviders(
        const RigExecExternalProviders &providers)
    {
        if (providers.transform) {
            double m[4][4]; std::memcpy(m, providers.transform, sizeof(m));
            transform = GfMatrix4d(m);
            values.transform = &transform;
        }
        influences.resize(providers.influenceCount);
        for (size_t k = 0; k < providers.influenceCount; ++k) {
            double m[4][4];
            std::memcpy(m, providers.influences + k * 16, sizeof(m));
            influences[k] = GfMatrix4d(m);
        }
        if (!influences.empty()) {
            values.influenceTransforms = &influences;
        }
        values.basePoints.resize(providers.basePointCount);
        for (size_t i = 0; i < providers.basePointCount; ++i) {
            const float *p = providers.basePoints + i * 3;
            values.basePoints[i] = GfVec3f(p[0], p[1], p[2]);
        }
    }
    GfMatrix4d transform{1.0};
    std::vector<GfMatrix4d> influences;
    RigExecProviderValues values;
};

/// \p apply over \p count xyz triples in place; false when it fails or
/// changes the point count.
inline bool
RigExecExternalPlaybackApply(const VtValue &data,
    bool (*apply)(const VtValue &, std::vector<GfVec3f> *),
    float *xyz, size_t count)
{
    std::vector<GfVec3f> points(count);
    for (size_t i = 0; i < count; ++i) {
        points[i] = GfVec3f(xyz[i * 3], xyz[i * 3 + 1], xyz[i * 3 + 2]);
    }
    if (!apply(data, &points) || points.size() != count) {
        return false;
    }
    for (size_t i = 0; i < count; ++i) {
        for (size_t a = 0; a < 3; ++a) {
            xyz[i * 3 + a] = points[i][a];
        }
    }
    return true;
}

}  // namespace rigExec

#endif
