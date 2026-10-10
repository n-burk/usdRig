#include "rigExec/goldenPose.h"
#include "rigExec/types.h"

#include "pxr/base/gf/quatd.h"
#include "pxr/base/gf/quatf.h"
#include "pxr/base/gf/range3d.h"
#include "pxr/base/gf/range3f.h"
#include "pxr/base/gf/vec2d.h"
#include "pxr/base/gf/vec2f.h"
#include "pxr/base/gf/vec4d.h"
#include "pxr/base/gf/vec4f.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/sdf/assetPath.h"
#include <cstring>
#include <iomanip>
#include <map>
#include <sstream>

namespace rigExec {
std::string RigExecOracleGoldenDouble(double value);
std::string RigExecOracleGoldenEscape(const std::string &value);
namespace {
// The retired scheduler summary is work metadata, with an exact grammar.
// Other diagnostics, including similar prefixes, remain part of the verdict.
bool IsRetiredWorkSummary(const std::string &text)
{
    size_t at = 0;
    const auto literal = [&](const char *part) {
        const size_t size = std::strlen(part);
        if (text.compare(at, size, part) != 0) return false;
        at += size; return true;
    };
    const auto number = [&] {
        const size_t begin = at;
        while (at < text.size() && text[at] >= '0' && text[at] <= '9') ++at;
        return at > begin;
    };
    return literal("mover graph: ") && number() && literal(" chain(s), ") &&
        number() && literal(" revision(s); ") && number() && literal(" created, ") &&
        number() && literal(" executed, ") && number() && literal(" schedule(s) built") && at == text.size();
}

std::string Hex(uint64_t value, int width)
{
    std::ostringstream stream;
    stream << std::hex << std::setfill('0') << std::setw(width) << value;
    return stream.str();
}
std::string Bits(float value)
{
    uint32_t bits; std::memcpy(&bits, &value, sizeof bits);
    return Hex(bits, 8);
}
std::string Bits(double value) { return RigExecOracleGoldenDouble(value); }
template<class Vector> std::string VectorBits(const Vector &value)
{
    std::string result;
    for (size_t i = 0; i < Vector::dimension; ++i) {
        if (i) result += ' ';
        result += Bits(value[i]);
    }
    return result;
}
std::string MatrixBits(const GfMatrix4d &matrix)
{
    std::string result;
    for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) {
        if (!result.empty()) result += ' ';
        result += Bits(matrix[r][c]);
    }
    return result;
}
std::string FrameBits(const RigExecPointFrame &frame)
{
    std::string result = "frame " + Hex(frame.flags, 8);
    for (const auto &point : frame.points) result += " " + VectorBits(point);
    return result;
}
template<class Array, class Encoder>
std::string ArrayBits(const Array &array, const char *type, Encoder encode)
{
    std::string result = std::string(type) + "[" + std::to_string(array.size()) + "]";
    for (const auto &item : array) result += " " + encode(item);
    return result;
}
bool ValueBits(const VtValue &value, std::string *out, std::string *error)
{
    if (value.IsEmpty()) { *out = "empty"; return true; }
#define RIGEXEC_GOLDEN_SCALAR(Type, Name, Encode) \
    if (value.IsHolding<Type>()) { \
        *out = std::string(Name) + " " + Encode(value.UncheckedGet<Type>()); \
        return true; \
    }
    RIGEXEC_GOLDEN_SCALAR(RigExecPointFrame, "pointFrame", [](const RigExecPointFrame &v){return FrameBits(v);})
    RIGEXEC_GOLDEN_SCALAR(RigExecPointFrameArray, "pointFrameArray", [](const RigExecPointFrameArray &v){
        return ArrayBits(v.frames, "frame", [](const RigExecPointFrame &f){return FrameBits(f);}) + " " +
            ArrayBits(v.rests, "rest", [](const std::array<GfVec3d, 4> &rest){
                return ArrayBits(rest, "vec3d", [](const GfVec3d &p){return VectorBits(p);});});
    })
    RIGEXEC_GOLDEN_SCALAR(RigExecWeightPacket, "weightPacket", [](const RigExecWeightPacket &v){
        return RigExecOracleGoldenEscape(v.representation.GetString()) + " " +
            RigExecOracleGoldenEscape(v.rangePolicy.GetString()) + " " + Bits(v.defaultWeight) + " " +
            (v.valid ? "1 " : "0 ") +
            ArrayBits(v.values, "float", [](float x){return Bits(x);}) + " " +
            ArrayBits(v.indices, "int", [](int x){return std::to_string(x);});
    })
    RIGEXEC_GOLDEN_SCALAR(RigExecFalloffLut, "falloffLut", [](const RigExecFalloffLut &v){
        return ArrayBits(v.samples, "float", [](float x){return Bits(x);});
    })
    RIGEXEC_GOLDEN_SCALAR(bool, "bool", [](bool v){return std::string(v ? "1" : "0");})
    RIGEXEC_GOLDEN_SCALAR(int, "int", [](int v){return std::to_string(v);})
    RIGEXEC_GOLDEN_SCALAR(unsigned int, "uint", [](unsigned int v){return std::to_string(v);})
    RIGEXEC_GOLDEN_SCALAR(int64_t, "int64", [](int64_t v){return std::to_string(v);})
    RIGEXEC_GOLDEN_SCALAR(uint64_t, "uint64", [](uint64_t v){return std::to_string(v);})
    RIGEXEC_GOLDEN_SCALAR(float, "float", [](float v){return Bits(v);})
    RIGEXEC_GOLDEN_SCALAR(double, "double", [](double v){return Bits(v);})
    RIGEXEC_GOLDEN_SCALAR(GfVec2f, "vec2f", [](const GfVec2f &v){return VectorBits(v);})
    RIGEXEC_GOLDEN_SCALAR(GfVec2d, "vec2d", [](const GfVec2d &v){return VectorBits(v);})
    RIGEXEC_GOLDEN_SCALAR(GfVec3f, "vec3f", [](const GfVec3f &v){return VectorBits(v);})
    RIGEXEC_GOLDEN_SCALAR(GfVec3d, "vec3d", [](const GfVec3d &v){return VectorBits(v);})
    RIGEXEC_GOLDEN_SCALAR(GfVec4f, "vec4f", [](const GfVec4f &v){return VectorBits(v);})
    RIGEXEC_GOLDEN_SCALAR(GfVec4d, "vec4d", [](const GfVec4d &v){return VectorBits(v);})
    RIGEXEC_GOLDEN_SCALAR(GfMatrix4d, "matrix4d", [](const GfMatrix4d &v){return MatrixBits(v);})
    RIGEXEC_GOLDEN_SCALAR(TfToken, "token", [](const TfToken &v){return RigExecOracleGoldenEscape(v.GetString());})
    RIGEXEC_GOLDEN_SCALAR(std::string, "string", [](const std::string &v){return RigExecOracleGoldenEscape(v);})
    RIGEXEC_GOLDEN_SCALAR(SdfPath, "path", [](const SdfPath &v){return RigExecOracleGoldenEscape(v.GetString());})
    RIGEXEC_GOLDEN_SCALAR(SdfAssetPath, "asset", [](const SdfAssetPath &v){return RigExecOracleGoldenEscape(v.GetAssetPath());})
    RIGEXEC_GOLDEN_SCALAR(GfRange3f, "range3f", [](const GfRange3f &v){return VectorBits(v.GetMin()) + " " + VectorBits(v.GetMax());})
    RIGEXEC_GOLDEN_SCALAR(GfRange3d, "range3d", [](const GfRange3d &v){return VectorBits(v.GetMin()) + " " + VectorBits(v.GetMax());})
    RIGEXEC_GOLDEN_SCALAR(GfQuatf, "quatf", [](const GfQuatf &v){return Bits(v.GetReal()) + " " + VectorBits(v.GetImaginary());})
    RIGEXEC_GOLDEN_SCALAR(GfQuatd, "quatd", [](const GfQuatd &v){return Bits(v.GetReal()) + " " + VectorBits(v.GetImaginary());})
#undef RIGEXEC_GOLDEN_SCALAR
#define RIGEXEC_GOLDEN_ARRAY(Type, Name, Encode) \
    if (value.IsHolding<Type>()) { \
        *out = ArrayBits(value.UncheckedGet<Type>(), Name, Encode); return true; \
    }
    RIGEXEC_GOLDEN_ARRAY(VtFloatArray, "float", [](float v){return Bits(v);})
    RIGEXEC_GOLDEN_ARRAY(VtDoubleArray, "double", [](double v){return Bits(v);})
    RIGEXEC_GOLDEN_ARRAY(VtIntArray, "int", [](int v){return std::to_string(v);})
    RIGEXEC_GOLDEN_ARRAY(VtVec2fArray, "vec2f", [](const GfVec2f &v){return VectorBits(v);})
    RIGEXEC_GOLDEN_ARRAY(VtVec3fArray, "vec3f", [](const GfVec3f &v){return VectorBits(v);})
    RIGEXEC_GOLDEN_ARRAY(VtVec3dArray, "vec3d", [](const GfVec3d &v){return VectorBits(v);})
    RIGEXEC_GOLDEN_ARRAY(VtMatrix4dArray, "matrix4d", [](const GfMatrix4d &v){return MatrixBits(v);})
    RIGEXEC_GOLDEN_ARRAY(VtTokenArray, "token", [](const TfToken &v){return RigExecOracleGoldenEscape(v.GetString());})
#undef RIGEXEC_GOLDEN_ARRAY
    if (error) *error = "unsupported golden value type " + value.GetTypeName();
    return false;
}

} // namespace

bool RigExecOracleEncodeGoldenValue(const VtValue &value, std::string *encoded, std::string *error)
{
    if (!encoded) { if (error) *error = "null golden value output"; return false; }
    return ValueBits(value, encoded, error);
}

std::string RigExecOracleGoldenDouble(double value)
{
    uint64_t bits; std::memcpy(&bits, &value, sizeof bits);
    return Hex(bits, 16);
}
std::string RigExecOracleGoldenHex(uint64_t value) { return Hex(value, 16); }
uint64_t RigExecOracleGoldenDigest(const std::string &bytes)
{
    uint64_t hash = UINT64_C(14695981039346656037);
    for (unsigned char byte : bytes) { hash ^= byte; hash *= UINT64_C(1099511628211); }
    return hash;
}
std::string RigExecOracleGoldenEscape(const std::string &value)
{
    std::string result = "\"";
    for (unsigned char ch : value) {
        if (ch == '\\' || ch == '"') { result += '\\'; result += char(ch); }
        else if (ch == '\n') result += "\\n";
        else if (ch == '\r') result += "\\r";
        else if (ch == '\t') result += "\\t";
        else if (ch < 32 || ch == 127) result += "\\x" + Hex(ch, 2);
        else result += char(ch);
    }
    return result + '"';
}

bool RigExecOracleEncodeGoldenPose(const RigExecRigPose &pose,
    std::vector<RigExecGoldenValue> *values, std::string *error, bool includeOracle)
{
    if (!values) { if (error) *error = "null golden output"; return false; }
    std::vector<RigExecGoldenValue> result;
    result.push_back({"valid", "", pose.valid ? "1" : "0"});
    const auto frames = [&](const auto &map, const char *domain) {
        for (const auto &entry : map)
            result.push_back({domain, entry.first.GetString(), FrameBits(entry.second)});
    };
    const auto matrices = [&](const auto &map, const char *domain) {
        for (const auto &entry : map)
            result.push_back({domain, entry.first.GetString(), "matrix4d " + MatrixBits(entry.second)});
    };
    frames(pose.jointFramesBase, "jointFramesBase");
    frames(pose.jointFramesFinal, "jointFramesFinal");
    matrices(pose.jointMatricesFinal, "jointMatricesFinal");
    frames(pose.controlFrames, "controlFrames");
    matrices(pose.providerXforms, "providerXforms");
    matrices(pose.providerBaseXforms, "providerBaseXforms");
    for (const auto &entry : pose.solverFrames)
        result.push_back({"solverFrames", entry.first.GetString(),
            ArrayBits(entry.second, "frame", [](const RigExecPointFrame &v){return FrameBits(v);})});
    const auto properties = [&](const auto &map, const char *domain) {
        for (const auto &entry : map) {
            std::string encoded;
            if (!ValueBits(entry.second, &encoded, error)) return false;
            result.push_back({domain, entry.first.GetString(), std::move(encoded)});
        }
        return true;
    };
    if (!properties(pose.movedProperties, "movedProperties") ||
        (includeOracle && !properties(pose.movedPropertiesCpu, "movedPropertiesCpu"))) return false;
    for (const auto &entry : pose.weightFields) {
        const auto &field = entry.second;
        result.push_back({"weightFields", entry.first.GetString(),
            RigExecOracleGoldenEscape(field.target.GetString()) + " " +
            ArrayBits(field.weights, "float", [](float v){return Bits(v);})});
    }
    matrices(pose.weightFrames, "weightFrames");
    size_t diagnosticIndex = 0;
    for (const std::string &diagnostic : pose.diagnostics) {
        if (IsRetiredWorkSummary(diagnostic)) continue;
        result.push_back({"diagnostics", std::to_string(diagnosticIndex++), RigExecOracleGoldenEscape(diagnostic)});
    }
    *values = std::move(result);
    return true;
}

std::string RigExecOracleGoldenVisit(const std::string &leg, size_t ordinal,
    const RigExecRigPose &pose, const std::vector<RigExecGoldenValue> &values,
    bool digestDomains)
{
    std::string result = "visit " + RigExecOracleGoldenEscape(leg) + " " +
        std::to_string(ordinal) + " time=" +
        (pose.time.IsDefault() ? "default" : RigExecOracleGoldenDouble(pose.time.GetValue())) + "\n";
    std::map<std::string, std::pair<size_t, std::string>> domains;
    for (const auto &value : values) {
        const std::string row = "v " + value.domain + " " + RigExecOracleGoldenEscape(value.key) +
            " " + value.value + "\n";
        if (digestDomains && value.domain != "diagnostics" && value.domain != "valid") {
            auto &domain = domains[value.domain]; ++domain.first; domain.second += row;
        } else result += row;
    }
    for (const auto &domain : domains)
        result += "h " + domain.first + " " + std::to_string(domain.second.first) + " " +
            RigExecOracleGoldenHex(RigExecOracleGoldenDigest(domain.second.second)) + "\n";
    return result;
}

bool RigExecOracleCompareGolden(const std::string &expected,
    const std::string &actual, std::string *error)
{
    if (expected == actual) return true;
    size_t at = 0, line = 1;
    while (at < expected.size() && at < actual.size() && expected[at] == actual[at]) {
        if (expected[at] == '\n') ++line;
        ++at;
    }
    const size_t start = at == 0 ? 0 : expected.rfind('\n', at - 1) + 1;
    const size_t actualStart = at == 0 ? 0 : actual.rfind('\n', at - 1) + 1;
    if (error) *error = "golden differs at row " + std::to_string(line) +
        "\nexpected: " + expected.substr(start, expected.find('\n', at) - start) +
        "\nactual: " + actual.substr(actualStart, actual.find('\n', at) - actualStart);
    return false;
}

} // namespace rigExec
