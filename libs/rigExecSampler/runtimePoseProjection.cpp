#include "runtimePoseProjection.h"
#include "pxr/base/vt/array.h"
#include <algorithm>
#include <utility>

namespace rigExec {
namespace {
GfMatrix4d Matrix(const RrMat4d &source)
{
    GfMatrix4d result(0.0);
    for (int row = 0; row < 4; ++row)
        for (int column = 0; column < 4; ++column)
            result[row][column] = source[row][column];
    return result;
}
RigExecPointFrame Frame(const RrPointFrame &source)
{
    RigExecPointFrame result;
    result.flags = source.flags;
    for (size_t point = 0; point < 4; ++point)
        for (size_t axis = 0; axis < 3; ++axis)
            result.points[point][axis] = source.points[point][axis];
    return result;
}
template<class Map, class Value>
bool Add(Map *map, const std::string &path, Value &&value, std::string *error)
{
    if (!map->emplace(SdfPath(path), std::forward<Value>(value)).second) {
        if (error) *error = "duplicate runtime publication: " + path;
        return false;
    }
    return true;
}
}

bool RigExecProjectRuntimePose(const RigExecRuntimeReader &reader,
    UsdTimeCode time, bool valid, RigExecRigPose *pose, std::string *error)
{
    if (!pose) { if (error) *error = "null runtime pose output"; return false; }
    RigExecRigPose result;
    result.time = time;
    result.valid = valid;
    for (const auto &provider : reader.GetProviderFramePublications()) {
        if (provider.publicationRole & 1) {
            if (!Add(&result.jointFramesBase, provider.path, Frame(provider.base), error) ||
                !Add(&result.jointFramesFinal, provider.path, Frame(provider.final), error)) return false;
        }
        if (provider.publicationRole & 2)
            if (!Add(&result.controlFrames, provider.path, Frame(provider.final), error)) return false;
    }
    for (const auto &joint : reader.GetJointMatrices())
        if (!Add(&result.jointMatricesFinal, joint.path, Matrix(joint.matrix), error)) return false;
    for (const auto &provider : reader.GetProviderXforms()) {
        if (!Add(&result.providerXforms, provider.path, Matrix(provider.matrix), error) ||
            !Add(&result.providerBaseXforms, provider.path, Matrix(provider.base), error)) return false;
    }
    for (const auto &solver : reader.GetSolverFramePublications()) {
        std::vector<RigExecPointFrame> frames;
        frames.reserve(solver.frames.size());
        for (const auto &frame : solver.frames) frames.push_back(Frame(frame));
        if (!Add(&result.solverFrames, solver.path, std::move(frames), error)) return false;
    }
    for (const auto &property : reader.GetPropertyValues()) {
        VtValue value;
        switch (property.value.tag) {
        case RrPropertyValue::Tag::Float: value = VtValue(property.value.f32); break;
        case RrPropertyValue::Tag::Double: value = VtValue(property.value.f64); break;
        case RrPropertyValue::Tag::Matrix4d: value = VtValue(Matrix(property.value.matrix)); break;
        case RrPropertyValue::Tag::Vec3f:
            value = VtValue(GfVec3f(property.value.vec[0], property.value.vec[1], property.value.vec[2])); break;
        default: if (error) *error = "unknown runtime property type"; return false;
        }
        if (!Add(&result.movedProperties, property.path, std::move(value), error)) return false;
    }
    for (const auto &property : reader.GetPoints()) {
        VtVec3fArray points(property.points.size());
        for (size_t index = 0; index < points.size(); ++index)
            for (size_t axis = 0; axis < 3; ++axis)
                points[index][axis] = property.points[index][axis];
        if (!Add(&result.movedProperties, property.path, VtValue(std::move(points)), error)) return false;
    }
    for (const auto &property : reader.GetMatrixPrimvars())
        if (!Add(&result.movedProperties, property.path, VtValue(Matrix(property.matrix)), error)) return false;
    for (const auto &field : reader.GetWeightFields()) {
        RigExecResolvedWeightField value;
        value.target = SdfPath(field.target);
        value.weights = VtFloatArray(field.weights.size());
        std::copy(field.weights.begin(), field.weights.end(), value.weights.begin());
        if (!Add(&result.weightFields, field.path, std::move(value), error)) return false;
    }
    for (const auto &field : reader.GetWeightFrames())
        if (!Add(&result.weightFrames, field.path, Matrix(field.matrix), error)) return false;
    result.diagnostics = reader.GetDiagnostics();
    result.executedOpCount = size_t(reader.GetCounters().executedOpCount);
    *pose = std::move(result);
    return true;
}
}
