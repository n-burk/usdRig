// rigExecPose evaluates the compiled native program and preserves exact pose/golden output.
// Optional --cpu-reference checks point chains; --exec-crosscheck verifies builtin rows.
#include "rigExec/frozenContext.h"
#include "rigExecSampler/runtimePoseProjection.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/frameExtraction.h"
#include "rigExec/goldenPose.h"
#include "rigExec/inputReplay.h"
#include "rigExec/bakedExecCrossCheckRows.h"
#include "rigExec/frozenContextInternal.h"
#include "rigExec/rigEvaluator.h"
#include "rigExec/types.h"
#include "rigExecMath/pointFrame.h"
#include "rigExecRuntime/runtime.h"
#include "rigExecSampler/inputSampler.h"

#include "pxr/base/arch/env.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/range3f.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/base/tf/type.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/value.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/gf/vec3d.h"
// usd/stage.h only forward-declares UsdAttribute and UsdPrim.
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/editContext.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <filesystem>
#include <iterator>
#include <cstdlib>
#include <map>
#include <string>
#include <type_traits>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace {
bool gCpuReference=false,gExecCrossCheck=false;
struct CheckVisitState {uint64_t serial=0;const rigExec::RigExecBakedProgram *program=nullptr;};
std::map<const rigExec::RigExecRigEvaluator *,CheckVisitState> checkVisits;
rigExec::RigExecRigPose EvaluateChecked(rigExec::RigExecRigEvaluator &evaluator,UsdTimeCode time) {
    evaluator.cpuReference=gCpuReference;
    if(gExecCrossCheck) {
        auto &state=checkVisits[&evaluator];
        if(!evaluator.GetBakedProgram() || (state.program && state.serial!=evaluator.GetStageEditSerial())) {
            std::vector<std::string> errors;
            if(!evaluator.Compile(&errors)) {
                rigExec::RigExecRigPose failed;failed.time=time;failed.diagnostics=std::move(errors);return failed;
            }
        }
        const auto *program=evaluator.GetBakedProgram();std::string error;
        if(!program || (!rigExec::RigExecBakedProgramTesting::ExecCrossCheckRows(*program) &&
            !rigExec::RigExecBakedProgramTesting::EnableExecCrossCheck(*program,&error))) {
            rigExec::RigExecRigPose failed;failed.time=time;
            failed.diagnostics.push_back("exec crosscheck preparation failed: "+error);return failed;
        }
        state={evaluator.GetStageEditSerial(),program};
    }
    auto pose=evaluator.Evaluate(time);
    if(gExecCrossCheck) {
        const auto *program=evaluator.GetBakedProgram();
        const auto rows=program?rigExec::RigExecBakedProgramTesting::ExecCrossCheckRows(*program):nullptr;
        if(!rows) {pose.valid=false;pose.diagnostics.push_back("exec crosscheck rows unavailable after generation");}
        else {
            const auto report=rows->Evaluate(time);
            pose.diagnostics.insert(pose.diagnostics.end(),report.diagnostics.begin(),report.diagnostics.end());
            if(!report.Passed() || (!rows->Descriptors().empty() && !report.checked)) {
                pose.valid=false;pose.diagnostics.push_back("exec crosscheck failed or checked no declared rows");
            }
        }
    }
    if(gCpuReference && pose.valid) {
        const auto *program=evaluator.GetBakedProgram();size_t expected=0;
        if(program)for(const auto &chain:program->GetStepGraph().chains)if(chain.haveBase && chain.haveResult)++expected;
        if(pose.referenceAgreements+pose.referenceMismatches!=expected) {
            pose.valid=false;pose.diagnostics.push_back("scalar reference did not check every published point chain");
        }
    }
    return pose;
}

// The generated resource directory is the one that carries the LibraryPath
// implementsComputeExtent needs; the source-tree copy is a data-only
// fallback for an ad hoc build (see the CMakeLists commentary).
std::string
SchemaResourceDir()
{
#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    return TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR);
#else
    return std::string();
#endif
}

SdfPath
FindRig(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->TraverseAll()) {
        if (prim.GetTypeName() == TfToken("RigExecRoot")) {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

std::string
FormatVec(const GfVec3d &v)
{
    return TfStringPrintf("(%8.4f %8.4f %8.4f)", v[0], v[1], v[2]);
}

// --pose-out: every published domain of a generation, in a canonical text
// form that is byte-for-byte reproducible. %.17g round-trips a double and
// %.9g a float, so two dumps are equal exactly when the poses are, which is
// what a refactor of either evaluation path is measured against.
std::string
FormatD(double v)
{
    return TfStringPrintf("%.17g", v);
}

std::string
FormatF(float v)
{
    return TfStringPrintf("%.9g", v);
}

std::string
FormatFrame(const rigExec::RigExecPointFrame &f)
{
    std::string out;
    for (const GfVec3d &pnt : f.points) {
        out += " " + FormatD(pnt[0]) + " " + FormatD(pnt[1]) + " " +
               FormatD(pnt[2]);
    }
    out += TfStringPrintf(" flags=%u", unsigned(f.flags));
    return out;
}

std::string
FormatMatrix(const GfMatrix4d &m)
{
    std::string out;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            out += " " + FormatD(m[r][c]);
        }
    }
    return out;
}

std::string
FormatValue(const VtValue &value)
{
    if (value.IsHolding<float>()) {
        return "float " + FormatF(value.UncheckedGet<float>());
    }
    if (value.IsHolding<double>()) {
        return "double " + FormatD(value.UncheckedGet<double>());
    }
    if (value.IsHolding<GfVec3f>()) {
        const GfVec3f &v = value.UncheckedGet<GfVec3f>();
        return "vec3f " + FormatF(v[0]) + " " + FormatF(v[1]) + " " +
               FormatF(v[2]);
    }
    if (value.IsHolding<GfVec3d>()) {
        const GfVec3d &v = value.UncheckedGet<GfVec3d>();
        return "vec3d " + FormatD(v[0]) + " " + FormatD(v[1]) + " " +
               FormatD(v[2]);
    }
    if (value.IsHolding<GfMatrix4d>()) {
        return "matrix4d" + FormatMatrix(value.UncheckedGet<GfMatrix4d>());
    }
    if (value.IsHolding<VtVec3fArray>()) {
        const VtVec3fArray &a = value.UncheckedGet<VtVec3fArray>();
        std::string out = TfStringPrintf("vec3f[%zu]", a.size());
        for (const GfVec3f &v : a) {
            out += " " + FormatF(v[0]) + " " + FormatF(v[1]) + " " +
                   FormatF(v[2]);
        }
        return out;
    }
    if (value.IsHolding<VtFloatArray>()) {
        const VtFloatArray &a = value.UncheckedGet<VtFloatArray>();
        std::string out = TfStringPrintf("float[%zu]", a.size());
        for (float v : a) {
            out += " " + FormatF(v);
        }
        return out;
    }
    if (value.IsHolding<GfRange3f>()) {
        const GfRange3f &r = value.UncheckedGet<GfRange3f>();
        return "range3f " + FormatF(r.GetMin()[0]) + " " +
               FormatF(r.GetMin()[1]) + " " + FormatF(r.GetMin()[2]) + " " +
               FormatF(r.GetMax()[0]) + " " + FormatF(r.GetMax()[1]) + " " +
               FormatF(r.GetMax()[2]);
    }
    return value.GetTypeName() + " " + TfStringify(value);
}

void
WritePoseDump(FILE *out, const rigExec::RigExecRigPose &pose, double frame)
{
    std::fprintf(out, "frame %s valid=%d\n", FormatD(frame).c_str(),
                 int(pose.valid));
    for (const auto &[path, f] : pose.jointFramesBase) {
        std::fprintf(out, "jointFramesBase %s%s\n", path.GetText(),
                     FormatFrame(f).c_str());
    }
    for (const auto &[path, f] : pose.jointFramesFinal) {
        std::fprintf(out, "jointFramesFinal %s%s\n", path.GetText(),
                     FormatFrame(f).c_str());
    }
    for (const auto &[path, m] : pose.jointMatricesFinal) {
        std::fprintf(out, "jointMatricesFinal %s%s\n", path.GetText(),
                     FormatMatrix(m).c_str());
    }
    for (const auto &[path, f] : pose.controlFrames) {
        std::fprintf(out, "controlFrames %s%s\n", path.GetText(),
                     FormatFrame(f).c_str());
    }
    for (const auto &[path, m] : pose.providerXforms) {
        std::fprintf(out, "providerXforms %s%s\n", path.GetText(),
                     FormatMatrix(m).c_str());
    }
    for (const auto &[path, m] : pose.providerBaseXforms) {
        std::fprintf(out, "providerBaseXforms %s%s\n", path.GetText(),
                     FormatMatrix(m).c_str());
    }
    for (const auto &[path, frames] : pose.solverFrames) {
        std::fprintf(out, "solverFrames %s [%zu]\n", path.GetText(),
                     frames.size());
        for (const rigExec::RigExecPointFrame &f : frames) {
            std::fprintf(out, "  %s\n", FormatFrame(f).c_str());
        }
    }
    for (const auto &[path, value] : pose.movedProperties) {
        std::fprintf(out, "movedProperties %s %s\n", path.GetText(),
                     FormatValue(value).c_str());
    }
    for (const auto &[path, value] : pose.movedPropertiesCpu) {
        std::fprintf(out, "movedPropertiesCpu %s %s\n", path.GetText(),
                     FormatValue(value).c_str());
    }
    for (const auto &[path, field] : pose.weightFields) {
        std::string line = TfStringPrintf("weightFields %s target=%s [%zu]",
                                          path.GetText(),
                                          field.target.GetText(),
                                          field.weights.size());
        for (float w : field.weights) {
            line += " " + FormatF(w);
        }
        std::fprintf(out, "%s\n", line.c_str());
    }
    for (const auto &[path, m] : pose.weightFrames) {
        std::fprintf(out, "weightFrames %s%s\n", path.GetText(),
                     FormatMatrix(m).c_str());
    }
    for (const std::string &d : pose.diagnostics) {
        std::fprintf(out, "diagnostic %s\n", d.c_str());
    }
    std::fprintf(out,"counters referenceMismatches=%zu referenceAgreements=%zu comparisonMismatches=%zu executedOpCount=%zu\n",
        pose.referenceMismatches,pose.referenceAgreements,pose.comparisonMismatches,pose.executedOpCount);
}

// A moved property is only interesting as a change: printing 1864 points
// says nothing, but the bound they occupy and the largest single
// displacement say whether the chain did anything and whether it went mad.
void
ReportPoints(const std::string &label, const VtValue &value,
             const VtVec3fArray &rest)
{
    // Property-domain results share this map with the point chains, and for
    // those the VALUE is the whole report -- a clamped dial that printed only
    // its type name would say nothing about whether the clamp happened.
    if (value.IsHolding<float>()) {
        std::printf("      %-52s %.5f\n", label.c_str(),
                    double(value.UncheckedGet<float>()));
        return;
    }
    if (value.IsHolding<GfVec3f>()) {
        std::printf("      %-52s %s\n", label.c_str(),
                    FormatVec(GfVec3d(value.UncheckedGet<GfVec3f>())).c_str());
        return;
    }
    if (value.IsHolding<GfMatrix4d>()) {
        const GfMatrix4d m = value.UncheckedGet<GfMatrix4d>();
        std::printf("      %-52s t%s\n", label.c_str(),
                    FormatVec(m.ExtractTranslation()).c_str());
        std::printf("        x%s y%s z%s\n",
                    FormatVec(m.GetRow3(0)).c_str(),
                    FormatVec(m.GetRow3(1)).c_str(),
                    FormatVec(m.GetRow3(2)).c_str());
        return;
    }
    if (!value.IsHolding<VtVec3fArray>()) {
        std::printf("      %-52s <%s>\n", label.c_str(),
                    value.GetTypeName().c_str());
        return;
    }
    const VtVec3fArray points = value.UncheckedGet<VtVec3fArray>();
    GfRange3f bound;
    double worst = 0.0;
    double total = 0.0;
    for (size_t i = 0; i < points.size(); ++i) {
        bound.UnionWith(points[i]);
        if (i < rest.size()) {
            const double d = (points[i] - rest[i]).GetLength();
            worst = std::max(worst, d);
            total += d;
        }
    }
    std::printf("      %-52s n=%zu\n", label.c_str(), points.size());
    std::printf("        bound %s .. %s\n",
                FormatVec(GfVec3d(bound.GetMin())).c_str(),
                FormatVec(GfVec3d(bound.GetMax())).c_str());
    if (!rest.empty()) {
        std::printf("        moved max %.5f  mean %.5f\n",
                    worst, points.empty() ? 0.0 : total / points.size());
    }
}

// Collects one asset-space matrix per joint per evaluated frame, in a stable
// joint order, and writes them out as a neutral USD layer.
class JointExport {
public:
    void Add(const rigExec::RigExecRigPose &pose, double frame)
    {
        std::vector<GfMatrix4d> row;
        row.reserve(pose.jointFramesFinal.size());
        std::vector<SdfPath> order;
        order.reserve(pose.jointFramesFinal.size());
        for (const auto &[path, frameValue] : pose.jointFramesFinal) {
            GfMatrix4d matrix(1.0);
            // The same construction the imaging bridge uses for guides: the
            // affine map that carries the identity landmarks onto the posed
            // ones IS the joint's local-to-asset transform.
            if (!frameValue.IsValid() ||
                !rigExec::RigExecPointsToMatrix(
                    rigExec::RigExecIdentityLandmarks(), frameValue.points,
                    &matrix)) {
                matrix.SetIdentity();
                ++_degenerate;
            }
            order.push_back(path);
            row.push_back(matrix);
        }
        // A joint set that changes shape mid-export would silently misalign
        // the arrays against the path list, so it fails instead.
        if (_paths.empty()) {
            _paths = order;
        } else if (_paths != order) {
            _inconsistent = true;
            return;
        }
        _frames.push_back(frame);
        _rows.push_back(std::move(row));
    }

    bool Write(const std::string &path, std::string *error) const
    {
        if (_inconsistent) {
            *error = "the joint set changed between frames";
            return false;
        }
        if (_paths.empty()) {
            *error = "no joints were evaluated";
            return false;
        }
        const UsdStageRefPtr out = UsdStage::CreateNew(path);
        if (!out) {
            *error = "cannot create " + path;
            return false;
        }
        const UsdPrim prim =
            out->DefinePrim(SdfPath("/RigExecJoints"), TfToken("Scope"));
        out->SetDefaultPrim(prim);

        VtArray<TfToken> tokens;
        tokens.reserve(_paths.size());
        for (const SdfPath &jointPath : _paths) {
            tokens.push_back(TfToken(jointPath.GetString()));
        }
        UsdAttribute paths = prim.CreateAttribute(
            TfToken("rigExec:jointPaths"), SdfValueTypeNames->TokenArray,
            /* custom = */ false, SdfVariabilityUniform);
        paths.Set(tokens);

        UsdAttribute transforms = prim.CreateAttribute(
            TfToken("rigExec:jointTransforms"),
            SdfValueTypeNames->Matrix4dArray);
        for (size_t i = 0; i < _frames.size(); ++i) {
            VtArray<GfMatrix4d> row(_rows[i].begin(), _rows[i].end());
            transforms.Set(row, UsdTimeCode(_frames[i]));
        }
        out->SetStartTimeCode(_frames.front());
        out->SetEndTimeCode(_frames.back());
        out->GetRootLayer()->SetComment(
            "Asset-space joint transforms evaluated by RigExec "
            "(rigExecPose --joints-out). rigExec:jointPaths is parallel to "
            "every rigExec:jointTransforms time sample.");
        out->GetRootLayer()->Save();
        return true;
    }

    size_t Degenerate() const { return _degenerate; }

private:
    std::vector<SdfPath> _paths;
    std::vector<double> _frames;
    std::vector<std::vector<GfMatrix4d>> _rows;
    size_t _degenerate = 0;
    bool _inconsistent = false;
};

/// \p attribute's value at \p time as a double, whatever scalar type it is
/// declared with.
bool
AttributeDouble(const UsdAttribute &attribute, UsdTimeCode time, double *out)
{
    if (!attribute) {
        return false;
    }
    if (attribute.GetTypeName() == SdfValueTypeNames->Double) {
        return attribute.Get(out, time);
    }
    if (attribute.GetTypeName() == SdfValueTypeNames->Float) {
        float value = 0;
        if (!attribute.Get(&value, time)) return false;
        *out = value;
        return true;
    }
    return false;
}

/// \p attribute's value at \p time, displaced by \p bump, as a VtValue of
/// the attribute's own type.
///
/// A drag is a value an animator holds a manipulator at, so it has to reach
/// the rig as the type the attribute is declared with: a float avar handed a
/// double override is an override the program cannot place, and the whole
/// generation falls back to the dynamic path -- which is exactly the number
/// the benchmark is trying not to measure.
bool
DisplacedValue(const UsdAttribute &attribute, UsdTimeCode time, double bump,
               VtValue *out)
{
    if (!attribute) {
        return false;
    }
    if (attribute.GetTypeName() == SdfValueTypeNames->Double) {
        double value = 0;
        if (!attribute.Get(&value, time)) return false;
        *out = VtValue(value + bump);
        return true;
    }
    if (attribute.GetTypeName() == SdfValueTypeNames->Float) {
        float value = 0;
        if (!attribute.Get(&value, time)) return false;
        *out = VtValue(float(value + bump));
        return true;
    }
    return false;
}

std::vector<UsdTimeCode>
ParseFrames(const std::string &text)
{
    std::vector<UsdTimeCode> frames;
    for (const std::string &piece : TfStringSplit(text, ",")) {
        if (!piece.empty()) {
            frames.push_back(UsdTimeCode(std::atof(piece.c_str())));
        }
    }
    return frames;
}

// Verify every published pose value, status, and ordered diagnostic through
// the exact common encoding. Work counters and opt-in oracle rows are outside
// the published output contract.
void
_VerifyPush(std::vector<std::string> *diffs, const std::string &line)
{
    if (diffs->size() < 12) {
        diffs->push_back(line);
    }
}

void
_VerifyOutputs(const rigExec::RigExecRigPose &native,
               const rigExec::RigExecRuntimeReader &reader,
               std::vector<std::string> *diffs)
{
    rigExec::RigExecRigPose runtime;
    std::string error;
    if (!rigExec::RigExecProjectRuntimePose(reader,native.time,true,&runtime,&error)) {
        _VerifyPush(diffs,error); return;
    }
    std::vector<rigExec::RigExecGoldenValue> expected,actual;
    if (!rigExec::RigExecEncodeGoldenPose(native,&expected,&error) ||
        !rigExec::RigExecEncodeGoldenPose(runtime,&actual,&error)) {
        _VerifyPush(diffs,error); return;
    }
    const auto a=rigExec::RigExecGoldenVisit("verify",0,native,expected);
    const auto b=rigExec::RigExecGoldenVisit("verify",0,runtime,actual);
    if (!rigExec::RigExecCompareGolden(a,b,&error)) _VerifyPush(diffs,error);
}

bool
_InvalidLeg(const rigExec::RigExecRigPose &pose)
{
    return !pose.valid || pose.referenceMismatches ||
           pose.comparisonMismatches;
}

// An input value as FormatValue prints the stage's: type, then value.
std::string
_FormatInput(const rigExec::RrInputValue &value,
             const rigExec::RigExecRuntimeReader &reader)
{
    switch (value.tag) {
    case rigExec::RrInputTag::Double:
        return "double " + FormatD(value.f64);
    case rigExec::RrInputTag::Float:
        return "float " + FormatF(value.f32);
    case rigExec::RrInputTag::Bool:
        return value.boolean ? "bool 1" : "bool 0";
    case rigExec::RrInputTag::Int:
        return "int " + std::to_string(value.i32);
    case rigExec::RrInputTag::Matrix4d: {
        std::string out = "matrix4d";
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                out += " " + FormatD(value.matrix[r][c]);
            }
        }
        return out;
    }
    case rigExec::RrInputTag::Token:
        return "token " + reader.GetTokenText(value.token);
    case rigExec::RrInputTag::Vec3d:
        return "vec3d " + FormatD(value.vec[0]) + " " + FormatD(value.vec[1]) +
               " " + FormatD(value.vec[2]);
    case rigExec::RrInputTag::Vec3f:
        return "vec3f " + FormatF(value.vec3f[0]) + " " +
               FormatF(value.vec3f[1]) + " " + FormatF(value.vec3f[2]);
    // An array input's value carries its tag alone.
    case rigExec::RrInputTag::IntArray:
        return "int[]";
    case rigExec::RrInputTag::FloatArray:
        return "float[]";
    case rigExec::RrInputTag::DoubleArray:
        return "double[]";
    case rigExec::RrInputTag::Vec2fArray:
        return "float2[]";
    case rigExec::RrInputTag::Vec3fArray:
        return "float3[]";
    }
    return "unknown";
}

template <class T>
bool
_SameBits(const T &a, const T &b)
{
    return std::memcmp(&a, &b, sizeof(T)) == 0;
}

// The program's read of an attribute against the binary's input value of
// it, bit for bit, in the input's type.
bool
_SameSampled(const VtValue &read, const rigExec::RrInputValue &value,
             const rigExec::RigExecRuntimeReader &reader)
{
    switch (value.tag) {
    case rigExec::RrInputTag::Double:
        return read.IsHolding<double>() &&
               _SameBits(read.UncheckedGet<double>(), value.f64);
    case rigExec::RrInputTag::Float:
        return read.IsHolding<float>() &&
               _SameBits(read.UncheckedGet<float>(), value.f32);
    case rigExec::RrInputTag::Bool:
        return read.IsHolding<bool>() &&
               read.UncheckedGet<bool>() == value.boolean;
    case rigExec::RrInputTag::Int:
        return read.IsHolding<int>() &&
               read.UncheckedGet<int>() == value.i32;
    case rigExec::RrInputTag::Matrix4d:
        if (!read.IsHolding<GfMatrix4d>()) {
            return false;
        }
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                if (!_SameBits(read.UncheckedGet<GfMatrix4d>()[r][c],
                               value.matrix[r][c])) {
                    return false;
                }
            }
        }
        return true;
    case rigExec::RrInputTag::Token:
        return read.IsHolding<TfToken>() &&
               read.UncheckedGet<TfToken>().GetString() ==
                   reader.GetTokenText(value.token);
    case rigExec::RrInputTag::Vec3d:
        if (!read.IsHolding<GfVec3d>()) {
            return false;
        }
        for (int i = 0; i < 3; ++i) {
            if (!_SameBits(read.UncheckedGet<GfVec3d>()[i], value.vec[i])) {
                return false;
            }
        }
        return true;
    case rigExec::RrInputTag::Vec3f:
        if (!read.IsHolding<GfVec3f>()) {
            return false;
        }
        for (int i = 0; i < 3; ++i) {
            if (!_SameBits(read.UncheckedGet<GfVec3f>()[i], value.vec3f[i])) {
                return false;
            }
        }
        return true;
    default:
        // The sampler never samples an array input.
        return false;
    }
    return false;
}

// What one frame's check of the sampled inputs compared and matched.
struct _SampledCheck {
    size_t compared = 0;
    size_t matched = 0;
    // Animated inputs no program input read: compared with their own
    // attribute.
    size_t own = 0;
};

// The sampler against the program: every program input whose read at
// \p time lands on an Animated input of the binary reads there exactly the
// value the sampler set. An input pinned to one attribute (a valid query)
// reads it through the query; one read the long way (a spline, a selection
// that moves) reads the attribute its walk selects at \p time, the way the
// program classifies it; one that crosses a property chain reads the
// chain's result and is left out. Every Animated input no program input
// read that way (one a geometry assembly or the computed section reads) is
// compared with its own attribute's typed value at \p time. Array inputs,
// which the sampler does not sample, are left out.
_SampledCheck
_VerifySampledInputs(const rigExec::RigExecRigEvaluator &evaluator,
                     const rigExec::RigExecRuntimeReader &reader,
                     UsdTimeCode time, std::vector<std::string> *diffs)
{
    _SampledCheck check;
    const rigExec::RigExecBakedProgram *program = evaluator.GetBakedProgram();
    if (!program) {
        _VerifyPush(diffs, "no baked program to check the sampled inputs "
                           "against");
        return check;
    }
    const rigExec::RigExecBakedProgramImpl &graph = program->GetStepGraph();
    std::vector<char> visited(reader.GetInputCount(), 0);
    const auto compare = [&](const std::string &name, size_t index,
                             bool answered, const VtValue &read) {
        ++check.compared;
        const rigExec::RrInputValue &value = reader.GetInputValue(index);
        if (answered && _SameSampled(read, value, reader)) {
            ++check.matched;
            return;
        }
        _VerifyPush(diffs, "input " + name + " sampled " +
                               _FormatInput(value, reader) + " program " +
                               (answered ? FormatValue(read)
                                         : std::string("no value")));
    };
    rigExec::frozenDetail::_ForEachPatchableInput(
        graph, [&](const auto &input) {
            using T = std::decay_t<decltype(input.constant)>;
            UsdAttribute attribute;
            if (input.query.IsValid()) {
                attribute = input.query.GetAttribute();
            } else if (input.resolvedAttr) {
                bool viaChain = false;
                bool varying = false;
                rigExec::RigExecBakedClassifyInput<T>(
                    input.resolvedAttr, time, graph.chainTargets, &viaChain,
                    &varying, &attribute);
                if (viaChain) {
                    return;
                }
            }
            if (!attribute) {
                return;
            }
            const std::string name = attribute.GetPath().GetString();
            size_t index = 0;
            if (!reader.FindInput(name, &index) ||
                !reader.GetInputInfo(index).animated ||
                rigExec::RrInputTagIsArray(reader.GetInputInfo(index).type)) {
                return;
            }
            visited[index] = 1;
            VtValue read;
            const bool answered = input.query.IsValid()
                                      ? input.query.Get(&read, time)
                                      : attribute.Get(&read, time);
            compare(name, index, answered, read);
        });
    for (size_t index = 0; index < visited.size(); ++index) {
        const rigExec::RigExecRuntimeInputInfo &info =
            reader.GetInputInfo(index);
        // The sampler leaves array inputs at their defaults.
        if (visited[index] || !info.animated ||
            rigExec::RrInputTagIsArray(info.type)) {
            continue;
        }
        const UsdAttribute attribute =
            SdfPath::IsValidPathString(info.name)
                ? graph.stage->GetAttributeAtPath(SdfPath(info.name))
                : UsdAttribute();
        VtValue read;
        const bool answered = attribute && attribute.Get(&read, time);
        ++check.own;
        compare(info.name, index, answered, read);
    }
    return check;
}

int
RunVerifyBinary(const UsdStageRefPtr &stage, const SdfPath &rigPath,
                const std::vector<UsdTimeCode> &frames,
                const std::vector<std::string> &dragInputs,
                const std::string &binaryPath, const std::string &poseOut,
                const std::string &jointsOut)
{
    std::ifstream stream(binaryPath, std::ios::binary);
    const std::vector<char> raw(
        (std::istreambuf_iterator<char>(stream)),
        std::istreambuf_iterator<char>());
    const std::vector<uint8_t> bytes(raw.begin(), raw.end());
    if (bytes.empty()) {
        std::printf("  FATAL: cannot read %s\n", binaryPath.c_str());
        return 2;
    }
    std::string error;
    std::unique_ptr<rigExec::RigExecRuntimeReader> reader =
        rigExec::RigExecRuntimeReader::Open(bytes.data(), bytes.size(),
                                            &error);
    if (!reader) {
        std::printf("  FATAL: cannot open %s: %s\n", binaryPath.c_str(),
                    error.c_str());
        return 2;
    }

    rigExec::RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetSolverGuidesEnabled(true);
    std::vector<std::string> errors;
    if (!evaluator.Compile(&errors)) {
        std::printf("  FAIL: verify leg did not compile\n");
        for (const std::string &line : errors) {
            std::printf("    %s\n", line.c_str());
        }
        return 1;
    }
    std::vector<std::string> reasons;
    if (!evaluator.IsBakeable(&reasons)) {
        std::printf("  FAIL: native program cannot be exported faithfully\n");
        for (const std::string &reason : reasons) {
            std::printf("    %s\n", reason.c_str());
        }
        return 1;
    }
    int status = 0;
    rigExec::RigExecInputSampler sampler;
    if (!sampler.Bind(stage, *reader, &error)) {
        std::printf("  FAIL: %s\n", error.c_str());
        return 1;
    }
    // The stage is the one the binary was baked from: every input resolves.
    for (const std::string &warning : sampler.GetWarnings()) {
        std::printf("  sampler: FAIL: %s\n", warning.c_str());
        status = 1;
    }
    std::printf("  sampler: %zu input(s), %zu sampled per frame\n",
                reader->GetInputCount(), sampler.GetAnimatedCount());

    FILE *poseDump = nullptr;
    if (!poseOut.empty()) {
        poseDump = std::fopen(poseOut.c_str(), "w");
        if (!poseDump) {
            std::printf("  cannot open %s for writing\n", poseOut.c_str());
            return 2;
        }
    }
    JointExport jointExport;
    // The explicit Compile above consumed what a fresh session's first
    // generation would have settled: notices (inert movers, purpose
    // warnings) the runtime replays ahead of its first Execute. They are
    // attached to the first run the runtime completes, restoring exactly
    // the no-precompile flow.
    bool seedAttached = errors.empty();
    const auto expectedPose =
        [&](const rigExec::RigExecRigPose &pose) {
            rigExec::RigExecRigPose expected = pose;
            if (!seedAttached) {
                seedAttached = true;
                expected.diagnostics.insert(expected.diagnostics.begin(), errors.begin(),
                                            errors.end());
            }
            return expected;
        };

    // The defaults: a fresh reader's run is the bake time's generation, in
    // every published output and ordered diagnostic. Not counted below.
    const double bakeTime = reader->GetBakeTime();
    // What the drags are measured against: whether a value moved anything.
    {
        const rigExec::RigExecRigPose pose =
            EvaluateChecked(evaluator,UsdTimeCode(bakeTime));
        if (_InvalidLeg(pose)) {
            std::printf("  defaults t=%g: native leg INVALID "
                        "(native generation or requested checks failed)\n",
                        bakeTime);
            status = 1;
        } else if (!reader->Execute(&error)) {
            std::printf("  defaults t=%g: FAIL: %s\n", bakeTime,
                        error.c_str());
            status = 1;
        } else {
            std::vector<std::string> diffs;
            _VerifyOutputs(expectedPose(pose), *reader, &diffs);

            if (diffs.empty()) {
                std::printf("  defaults t=%g: runtime==native (%zu joints, "
                            "%zu moved, %zu weight frames, %zu fields, "
                            "%zu provider xforms, %zu diagnostics)\n",
                            bakeTime, reader->GetJointMatrices().size(),
                            reader->GetPoints().size(),
                            reader->GetWeightFrames().size(),
                            reader->GetWeightFields().size(),
                            reader->GetProviderXforms().size(),
                            reader->GetDiagnostics().size());
            } else {
                std::printf("  defaults t=%g: MISMATCH (%zu differences)\n",
                            bakeTime, diffs.size());
                for (const std::string &line : diffs) {
                    std::printf("    %s\n", line.c_str());
                }
                status = 1;
            }
        }
    }

    // Every frame: the stage's values of the binary's Animated inputs at
    // t, then the run, against the native evaluator's generation at t.
    size_t matched = 0;
    _SampledCheck sampled;
    for (UsdTimeCode frame : frames) {
        const double t = frame.GetValue();
        const rigExec::RigExecRigPose pose = EvaluateChecked(evaluator,frame);
        if (_InvalidLeg(pose)) {
            std::printf("\n  frame %g: native leg INVALID "
                        "(native generation or requested checks failed)\n",
                        t);
            status = 1;
            continue;
        }
        if (!sampler.Apply(frame, reader.get(), &error) ||
            !reader->Execute(&error)) {
            std::printf("\n  frame %g: FAIL: %s\n", t, error.c_str());
            status = 1;
            continue;
        }
        std::vector<std::string> diffs;
        _VerifyOutputs(expectedPose(pose), *reader, &diffs);

        const _SampledCheck frameSampled =
            _VerifySampledInputs(evaluator, *reader, frame, &diffs);
        sampled.compared += frameSampled.compared;
        sampled.matched += frameSampled.matched;
        sampled.own += frameSampled.own;
        if (diffs.empty()) {
            ++matched;
            std::printf("  frame %g: runtime==native (%zu joints, "
                        "%zu moved, %zu weight frames, %zu fields, "
                        "%zu provider xforms, %zu diagnostics)\n",
                        t, reader->GetJointMatrices().size(),
                        reader->GetPoints().size(),
                        reader->GetWeightFrames().size(),
                        reader->GetWeightFields().size(),
                        reader->GetProviderXforms().size(),
                        reader->GetDiagnostics().size());
        } else {
            std::printf("  frame %g: MISMATCH (%zu differences)\n", t,
                        diffs.size());
            for (const std::string &line : diffs) {
                std::printf("    %s\n", line.c_str());
            }
            status = 1;
        }
        if (poseDump) {
            WritePoseDump(poseDump, pose, t);
        }
        if (!jointsOut.empty()) {
            jointExport.Add(pose, t);
        }
    }
    if (poseDump) {
        std::fclose(poseDump);
    }
    if (!jointsOut.empty()) {
        if (!jointExport.Write(jointsOut, &error)) {
            std::printf("  joint export FAILED: %s\n", error.c_str());
            status = 1;
        }
    }
    std::printf("  verify-binary: %zu of %zu frame(s) match\n", matched,
                frames.size());
    std::printf("  sampler: %zu of %zu sampled input value(s) match the "
                "program's reads (%zu read through no program input, "
                "compared with their own attribute)\n",
                sampled.matched, sampled.compared, sampled.own);

    // The drags, at the bake time: the binary's input set to a value
    // against the admitted native typed value or a connected head's authored
    // fallback in an isolated session, then both put back. The runtime first returns to the bake time's samples. Each
    // value's reference is a fresh evaluator compiled before the override, so
    // it answers the first override after a compile, as the binary answers a
    // set on its bake, whatever drags came before.
    const UsdTimeCode dragTime(bakeTime);
    size_t dragsMatched = 0;
    size_t dragsTried = 0;
    if (!dragInputs.empty() &&
        !sampler.Apply(dragTime, reader.get(), &error)) {
        std::printf("  drag: FAIL: %s\n", error.c_str());
        status = 1;
    }
    // Preserve the session-layer restoration guard for each drag, not merely
    // cleared: a spec left behind changes how the evaluator takes the next
    // edit.
    const SdfLayerHandle session = stage->GetSessionLayer();
    const SdfLayerRefPtr sessionBefore = SdfLayer::CreateAnonymous();
    sessionBefore->TransferContent(session);
    for (const std::string &name : dragInputs) {
        size_t index = 0;
        const UsdAttribute attribute =
            SdfPath::IsValidPathString(name)
                ? stage->GetAttributeAtPath(SdfPath(name))
                : UsdAttribute();
        const bool found = reader->FindInput(name, &index);
        const rigExec::RigExecRuntimeInputInfo &info =
            reader->GetInputInfo(index);
        const bool isFloat =
            attribute &&
            attribute.GetTypeName().GetType() == TfType::Find<float>();
        const bool isDouble =
            attribute &&
            attribute.GetTypeName().GetType() == TfType::Find<double>();
        if (!found || !attribute || (!isFloat && !isDouble) ||
            (info.type != rigExec::RrInputTag::Double &&
             info.type != rigExec::RrInputTag::Float)) {
            std::printf("  drag %s: FAIL: %s\n", name.c_str(),
                        !found       ? "no input of the binary"
                        : !attribute ? "no attribute on the stage"
                                     : "not a double or float input");
            dragsTried += 2;
            status = 1;
            continue;
        }
        const double base = info.type == rigExec::RrInputTag::Float
                                ? double(info.defaultValue.f32)
                                : info.defaultValue.f64;
        for (const double bump : {0.25, -0.5}) {
            ++dragsTried;
            const double value = base + bump;
            bool authored = false;
            bool compiled = false;
            std::vector<std::string> notices;
            std::string sourceBefore, sessionBeforeDrag;
            const bool capturedLayers = stage->GetRootLayer()->ExportToString(&sourceBefore) &&
                session->ExportToString(&sessionBeforeDrag);
            rigExec::RigExecRigPose pose;
            {
                // Preserve source wiring through the admitted native channel.
                // Connected heads instead retain their authored fallback in an
                // isolated session; the input stage stays intact.
                const SdfLayerRefPtr referenceSession = SdfLayer::CreateAnonymous();
                referenceSession->TransferContent(sessionBefore);
                const UsdStageRefPtr referenceStage = UsdStage::Open(
                    stage->GetRootLayer(), referenceSession,
                    stage->GetPathResolverContext());
                if (referenceStage) {
                    rigExec::RigExecRigEvaluator reference(referenceStage, rigPath);
                    reference.SetSolverGuidesEnabled(true);
                    compiled = reference.Compile(&notices);
                    const UsdAttribute referenceAttribute =
                        referenceStage->GetAttributeAtPath(attribute.GetPath());
                    reference.SetUpstreamInputs({rigExec::RigExecValueOverride{
                        attribute.GetPrimPath(), TfToken(), attribute.GetName(),
                        isFloat ? VtValue(float(value)) : VtValue(value)}});
                    const auto admitted = reference.GetUpstreamInputPaths();
                    authored = std::find(admitted.begin(), admitted.end(),
                                         attribute.GetPath()) != admitted.end();
                    SdfPathVector referenceConnections;
                    if (!authored && referenceAttribute)
                        referenceAttribute.GetConnections(&referenceConnections);
                    if (!authored && !referenceConnections.empty()) {
                        // A connected head is not an admitted upstream input.
                        // Its own authored fallback retains SetInput semantics.
                        reference.SetUpstreamInputs({});
                        UsdEditContext edit(referenceStage, referenceSession);
                        authored = isFloat ? referenceAttribute.Set(float(value))
                                           : referenceAttribute.Set(value);
                    }
                    pose = EvaluateChecked(reference,dragTime);
                }

            }
            std::vector<std::string> diffs;
            std::string sourceAfter, sessionAfterDrag;
            if (!capturedLayers || !stage->GetRootLayer()->ExportToString(&sourceAfter) ||
                !session->ExportToString(&sessionAfterDrag) || sourceBefore != sourceAfter ||
                sessionBeforeDrag != sessionAfterDrag) {
                _VerifyPush(&diffs, "native input override changed the source stage");
            }
            if (!compiled) {
                _VerifyPush(&diffs, "the reference did not compile");
            }
            if (!authored) {
                _VerifyPush(&diffs, "the native upstream channel refused the value");
            }
            if (_InvalidLeg(pose)) {
                _VerifyPush(&diffs,
                            TfStringPrintf(
                                "native leg INVALID (valid=%d, %zu reference "
                                "mismatch(es), %zu comparison "
                                "mismatch(es))",
                                int(pose.valid),
                                pose.referenceMismatches,
                                pose.comparisonMismatches));
                for (const std::string &line : pose.diagnostics) {
                    _VerifyPush(&diffs, "  " + line.substr(0, 200));
                }
            } else if (!reader->SetInput(name, value, &error) ||
                       !reader->Execute(&error)) {
                _VerifyPush(&diffs, error);
            } else {
                _VerifyOutputs(pose, *reader, &diffs);
            }
            // The reference edits only its isolated session; the original
            // session is unchanged, as the exact guards above require.
            reader->ResetInput(name, nullptr);
            if (diffs.empty()) {
                ++dragsMatched;
                std::printf("  drag %s = %.17g: runtime==native exact published pose\n",
                            name.c_str(), value);
            } else {
                std::printf("  drag %s = %.17g: MISMATCH (%zu differences)\n",
                            name.c_str(), value, diffs.size());
                for (const std::string &line : diffs) {
                    std::printf("    %s\n", line.c_str());
                }
                status = 1;
            }
        }
    }
    std::printf("  verify-binary: %zu of %zu drag(s) match\n", dragsMatched,
                dragsTried);
    return status;
}


struct GoldenAction {
    std::string source;
    SdfPath path;
    double displacement = 0;
};

int
RunGoldenProtocol(const UsdStageRefPtr &stage, const SdfPath &rigPath,
                  rigExec::RigExecRigEvaluator &evaluator,
                  const std::vector<UsdTimeCode> &frames,
                  const std::string &fixture, const std::string &output,
                  const std::string &check, bool compact,
                  const std::vector<GoldenAction> &actions,
                  const std::string &backend,
                  const std::string &programPath)
{
    if (backend != "native" && backend != "frozen" && backend != "runtime") {
        std::printf("golden: unknown backend %s\n", backend.c_str());
        return 2;
    }
    if (backend == "frozen" && gExecCrossCheck) {
        std::printf("golden: --exec-crosscheck currently requires --golden-backend native\n");
        return 2;
    }
    std::vector<uint8_t> runtimeBytes;
    if (backend=="runtime") {
        if (programPath.empty()) {
            std::printf("golden: runtime backend requires --golden-program\n"); return 2;
        }
        if (gExecCrossCheck || gCpuReference) {
            std::printf("golden: runtime backend does not support native reference or exec-row checks\n"); return 2;
        }
        for(const auto &action:actions) if(action.source=="interactive") {
            std::printf("golden: runtime binary has no distinct interactive-input semantics\n"); return 2;
        }
        std::ifstream stream(programPath,std::ios::binary);
        const std::vector<char> raw{std::istreambuf_iterator<char>(stream),std::istreambuf_iterator<char>()};
        runtimeBytes.assign(raw.begin(),raw.end());
        if(runtimeBytes.empty()) {
            std::printf("golden: cannot read runtime program %s\n",programPath.c_str()); return 2;
        }
    }
    std::string bytes = "rigexec-golden 2\nfixture " +
        rigExec::RigExecGoldenEscape(fixture) + " rig " +
        rigExec::RigExecGoldenEscape(rigPath.GetString()) +
        "\nprotocol exact-bits guides=1 fields=1 visits=first,held,forward,reverse,cold,actions\n";
    std::vector<std::string> layerDigests;
    for (const SdfLayerHandle &layer : stage->GetUsedLayers()) {
        std::string contents;
        if (!layer->ExportToString(&contents)) {
            std::printf("golden: cannot encode source layer\n"); return 1;
        }
        layerDigests.push_back(rigExec::RigExecGoldenHex(
            rigExec::RigExecGoldenDigest(contents)));
    }
    std::sort(layerDigests.begin(), layerDigests.end());
    bytes += "layers";
    for (const std::string &digest : layerDigests) bytes += " " + digest;
    bytes += "\n";
    size_t visits = 0;
    std::shared_ptr<const rigExec::RigExecFrozenProgram> frozen;
    std::unique_ptr<rigExec::RigExecFrozenWorkspace> workspace;
    rigExec::RigExecRigEvaluator *frozenSource = nullptr;
    uint64_t frozenSerial = 0;
    std::vector<rigExec::RigExecValueOverride> frozenOverrides;
    std::unique_ptr<rigExec::RigExecRuntimeReader> runtimeReader;
    rigExec::RigExecInputSampler runtimeSampler;
    std::map<SdfPath,VtValue> runtimeUpstream;
    std::set<SdfPath> runtimeTouched;

    const auto visit = [&](rigExec::RigExecRigEvaluator &source,
                           const std::string &leg, size_t ordinal, UsdTimeCode time) {
        rigExec::RigExecRigPose pose;
        if (backend == "native") pose = EvaluateChecked(source,time);
        else if (backend=="runtime") {
            std::string error;
            if(!runtimeReader) {
                runtimeReader=rigExec::RigExecRuntimeReader::Open(runtimeBytes.data(),runtimeBytes.size(),&error);
                if(!runtimeReader || !runtimeSampler.Bind(stage,*runtimeReader,&error)) {
                    std::printf("golden runtime open: %s\n",error.c_str()); return false;
                }
                if(!runtimeSampler.GetWarnings().empty()) {
                    for(const auto &warning:runtimeSampler.GetWarnings())
                        std::printf("golden runtime input: %s\n",warning.c_str());
                    return false;
                }
            }
            if(!runtimeSampler.Apply(time,runtimeReader.get(),&error)) {
                std::printf("golden runtime sample: %s\n",error.c_str()); return false;
            }
            for(const auto &path:runtimeTouched) {
                size_t index=0;
                if(!runtimeReader->FindInput(path.GetString(),&index)) {
                    std::printf("golden runtime: action names no declared input %s\n",path.GetText()); return false;
                }
                const auto &info=runtimeReader->GetInputInfo(index);
                const auto upstream=runtimeUpstream.find(path);
                if(upstream==runtimeUpstream.end()) {
                    if(!rigExec::RigExecSampleInputAt(stage->GetAttributeAtPath(path),index,path.GetString(),
                            info.type,time,runtimeReader.get(),&error)) {
                        std::printf("golden runtime restore: %s\n",error.c_str()); return false;
                    }
                } else {
                    rigExec::RrInputValue value;
                    if(!rigExec::RigExecInputValueFrom(upstream->second,info.type,&value) ||
                       !runtimeReader->SetInputAt(index,value,&error)) {
                        std::printf("golden runtime upstream: %s\n",error.c_str()); return false;
                    }
                }
            }
            const bool valid=runtimeReader->Execute(&error);
            if(!valid || !rigExec::RigExecProjectRuntimePose(*runtimeReader,time,valid,&pose,&error)) {
                std::printf("golden runtime execute: %s\n",error.c_str()); return false;
            }
        } else {
            source.cpuReference = gCpuReference;
            std::string error;
            const bool changed = frozenSource != &source ||
                frozenSerial != source.GetStageEditSerial();
            if (changed || !workspace) {
                std::vector<std::string> errors;
                if (!source.Compile(&errors) ||
                    !rigExec::RigExecFreezeProgram(source,&frozen,&error)) {
                    for (const auto &line : errors) std::printf("golden frozen: %s\n",line.c_str());
                    std::printf("golden frozen: %s\n",error.c_str()); return false;
                }
                workspace = rigExec::RigExecCreateFrozenWorkspace(frozen);
                frozenSource = &source;
                frozenSerial = source.GetStageEditSerial();
            }
            rigExec::RigExecFrameInputs inputs;
            std::vector<rigExec::RigExecUpstreamValue> frozenUpstream;
            for(const auto &value:source.GetUpstreamInputs())
                frozenUpstream.push_back({value.prim.AppendProperty(value.attribute),value.value});
            if (!rigExec::RigExecSampleFrameInputs(source,time,frozenOverrides,
                    frozenUpstream,&inputs,&error)) {
                std::printf("golden frozen sample: %s\n",error.c_str()); return false;
            }
            rigExec::RigExecFrozenEvalContext context;
            context.epochDigest = source.GetBindingEpochDigest();
            context.programDigest = context.epochDigest ^
                (uint64_t(source.GetBakedProgram()->GetBoundInputCount()) << 32) ^
                source.GetBakedProgram()->GetVaryingInputCount();
            context.slotCount = source.GetBakedProgram()->GetProviderCount();
            context.varyingInputCount = inputs.values.size();
            context.flags = rigExec::kRigExecFrozenSolverGuidesEnabled |
                            rigExec::kRigExecFrozenPublishWeightFields;
            context.frozen = frozen.get(); context.workspace = workspace.get();
            pose = rigExec::RigExecEvaluateFrozen(context,inputs,
                rigExec::RigExecMakeProductionStepRunner(),nullptr,rigPath);
        }
        std::vector<rigExec::RigExecGoldenValue> values;
        std::string error;
        if (!rigExec::RigExecEncodeGoldenPose(pose, &values, &error)) {
            std::printf("golden: %s\n", error.c_str()); return false;
        }
        bytes += rigExec::RigExecGoldenVisit(leg, ordinal, pose, values, compact);
        ++visits;
        if (!pose.valid) {
            std::printf("golden: invalid visit %s[%zu]\n", leg.c_str(), ordinal);
            return false;
        }
        return true;
    };
    if (!visit(evaluator, "first", 0, frames.front()) ||
        !visit(evaluator, "held", 0, frames.front())) return 1;
    for (size_t i = 0; i < frames.size(); ++i)
        if (!visit(evaluator, "forward", i, frames[i])) return 1;
    for (size_t i = 0; i < frames.size(); ++i)
        if (!visit(evaluator, "reverse", i, frames[frames.size() - i - 1])) return 1;
    for (size_t i = 0; i < frames.size(); ++i) {
        if(backend=="runtime") {
            runtimeReader.reset();
            if(!visit(evaluator,"cold",i,frames[i])) return 1;
            continue;
        }
        rigExec::RigExecRigEvaluator cold(stage, rigPath);
        cold.SetSolverGuidesEnabled(true);

        std::vector<std::string> errors;
        if (!cold.Compile(&errors)) {
            for (const auto &error : errors) std::printf("golden cold: %s\n", error.c_str());
            return 1;
        }
        workspace.reset(); frozen.reset(); frozenSource = nullptr;
        if (!visit(cold, "cold", i, frames[i])) return 1;
    }
    const auto session = stage->GetSessionLayer();
    std::string originalSession;
    if (!session->ExportToString(&originalSession)) return 1;
    for (size_t i = 0; i < actions.size(); ++i) {
        const GoldenAction &action = actions[i];
        const UsdAttribute attribute = stage->GetAttributeAtPath(action.path);
        VtValue value;
        if (!DisplacedValue(attribute, frames.front(), action.displacement, &value)) {
            std::printf("golden: action requires a readable float/double: %s\n", action.path.GetText());
            return 1;
        }
        const rigExec::RigExecValueOverride input{action.path.GetPrimPath(), TfToken(),
            action.path.GetNameToken(), value};
        const std::string leg = action.source + ":" + action.path.GetString() +
            "@" + rigExec::RigExecGoldenDouble(action.displacement);
        if(backend=="runtime") {
            runtimeTouched.insert(action.path);
            if(action.source=="upstream") runtimeUpstream[action.path]=value;
        }
        if (action.source == "upstream") evaluator.SetUpstreamInputs({input});
        else if (action.source == "interactive") {
            evaluator.SetInteractiveOverrides({input}); frozenOverrides = {input};
        }
        else {
            UsdEditContext edit(stage, session);
            if (!attribute.Set(value, frames.front())) return 1;
        }
        const bool placed = visit(evaluator, leg + ":place", i, frames.front()) &&
                            visit(evaluator, leg + ":held", i, frames.front());
        if(backend=="runtime") runtimeUpstream.erase(action.path);
        if (action.source == "upstream") evaluator.SetUpstreamInputs({});
        else if (action.source == "interactive") {
            evaluator.ClearInteractiveOverrides(); frozenOverrides.clear();
        }
        else if (!session->ImportFromString(originalSession)) return 1;
        if (!placed || !visit(evaluator, leg + ":lift", i, frames.front())) return 1;
    }
    if (!check.empty()) {
        std::ifstream input(check, std::ios::binary);
        if (!input) { std::printf("golden: cannot read %s\n", check.c_str()); return 1; }
        const std::string expected{std::istreambuf_iterator<char>(input),
                                   std::istreambuf_iterator<char>()};
        std::string error;
        if (!rigExec::RigExecCompareGolden(expected, bytes, &error)) {
            std::printf("golden: %s\n", error.c_str()); return 1;
        }
    }
    if (!output.empty()) {
        std::ofstream stream(output, std::ios::binary);
        stream.write(bytes.data(), std::streamsize(bytes.size()));
        if (!stream) { std::printf("golden: cannot write %s\n", output.c_str()); return 1; }
    }
    std::printf("golden: %zu explicit visits, %s\n", visits,
                check.empty() ? "captured" : "exact match");
    return 0;
}

}  // namespace

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::printf(
            "usage: rigExecPose <stage> [--rig <primPath>] "
            "[--frames a,b,c] [--joints] [--targets] "
            "[--joints-out <file.usda>] [--pose-out <file.txt>] "
            "[--profile <file.trace>] "
            "[--golden-out file|--golden-check file] [--golden-compact] [--golden-backend native|frozen|runtime] [--golden-program file.rigexec] "
            "[--cpu-reference] [--exec-crosscheck] "
            "[--guides] "
            "[--drag <prim> <attr> <steps>] "
            "[--verify-binary <file.rigexec> --frames a,b,c "
            "[--drag-input <prim.attr>]...]\n");
        return 2;
    }
    std::string stagePath = argv[1];
    std::string rigArg;
    std::string jointsOut;
    std::string poseOut;
    std::string goldenOut, goldenCheck, goldenFixture;
    std::string goldenBackend = "native", goldenProgram;
    bool goldenCompact = false;
    std::vector<GoldenAction> goldenActions;
    std::string profileOut;
    std::string verifyBinary;
    bool cpuReference=false,execCrossCheck=false;
    std::vector<UsdTimeCode> frames;
    std::vector<std::string> dragInputs;
    std::string dragPrim, dragAttr;
    int dragSteps = 0;
    int repeat = 1;
    bool showJoints = false;
    bool showTargets = false;
    bool solverGuides = false;
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--rig" && i + 1 < argc) {
            rigArg = argv[++i];
        } else if (arg == "--frames" && i + 1 < argc) {
            frames = ParseFrames(argv[++i]);
        } else if (arg == "--joints") {
            showJoints = true;
        } else if (arg == "--targets") {
            showTargets = true;
        } else if (arg == "--guides") {
            solverGuides = true;
        } else if (arg == "--cpu-reference") {
            cpuReference=true;
        } else if (arg == "--exec-crosscheck") {
            execCrossCheck=true;
        } else if (arg == "--joints-out" && i + 1 < argc) {
            jointsOut = argv[++i];
        } else if (arg == "--pose-out" && i + 1 < argc) {
            poseOut = argv[++i];
        } else if (arg == "--golden-out" && i + 1 < argc) {
            goldenOut = argv[++i];
        } else if (arg == "--golden-check" && i + 1 < argc) {
            goldenCheck = argv[++i];
        } else if (arg == "--golden-program" && i + 1 < argc) {
            goldenProgram = argv[++i];
        } else if (arg == "--golden-backend" && i + 1 < argc) {
            goldenBackend = argv[++i];
        } else if (arg == "--golden-fixture" && i + 1 < argc) {
            goldenFixture = argv[++i];
        } else if (arg == "--golden-compact") {
            goldenCompact = true;
        } else if ((arg == "--golden-upstream" || arg == "--golden-edit" ||
                    arg == "--golden-interactive") && i + 2 < argc) {
            const std::string source = arg == "--golden-upstream" ? "upstream" :
                arg == "--golden-edit" ? "authored" : "interactive";
            const SdfPath path(argv[++i]);
            char *end = nullptr;
            const char *text = argv[++i];
            const double delta = std::strtod(text, &end);
            if (!path.IsPropertyPath() || end == text || *end) {
                std::printf("golden action needs a property path and numeric displacement\n");
                return 2;
            }
            goldenActions.push_back({source, path, delta});
        } else if (arg == "--repeat" && i + 1 < argc) {
            repeat = std::atoi(argv[++i]);
            if (repeat < 1) {
                std::printf("--repeat wants a count of 1 or more\n");
                return 2;
            }
        } else if (arg == "--drag" && i + 3 < argc) {
            dragPrim = argv[++i];
            dragAttr = argv[++i];
            dragSteps = std::atoi(argv[++i]);
            if (dragSteps < 1) {
                std::printf("--drag wants a step count of 1 or more\n");
                return 2;
            }
        } else if (arg == "--profile" && i + 1 < argc) {
            profileOut = argv[++i];
        } else if (arg == "--verify-binary" && i + 1 < argc) {
            verifyBinary = argv[++i];
        } else if (arg == "--drag-input" && i + 1 < argc) {
            dragInputs.push_back(argv[++i]);
        } else {
            std::printf("unknown argument: %s\n", arg.c_str());
            return 2;
        }
    }

    const bool golden = !goldenOut.empty() || !goldenCheck.empty();
    if (golden) {
        solverGuides = true;
        if (repeat != 1 || dragSteps || !verifyBinary.empty() || !profileOut.empty()) {
            std::printf("golden protocol cannot be combined with benchmark/drag/binary/profile runs\n");
            return 2;
        }
    } else if (goldenCompact || !goldenActions.empty() || !goldenFixture.empty()) {
        std::printf("golden options need --golden-out or --golden-check\n"); return 2;
    }

    // The binary holds one time; the frames to verify it at are the
    // caller's to name.
    if (!verifyBinary.empty() && frames.empty()) {
        std::printf("FATAL: --verify-binary needs --frames\n");
        return 2;
    }

    const std::string resources = SchemaResourceDir();
    if (!resources.empty() &&
        PlugRegistry::GetInstance().RegisterPlugins(resources).empty()) {
        std::printf("FATAL: no schema plugin at %s\n", resources.c_str());
        return 2;
    }

    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    if (!stage) {
        std::printf("FATAL: cannot open %s\n", stagePath.c_str());
        return 2;
    }
    const SdfPath rigPath =
        rigArg.empty() ? FindRig(stage) : SdfPath(rigArg);
    if (rigPath.IsEmpty() || !stage->GetPrimAtPath(rigPath)) {
        std::printf("FATAL: no RigExecRoot in %s\n", stagePath.c_str());
        return 2;
    }
    std::printf("stage %s\n  rig %s\n", stagePath.c_str(),
                rigPath.GetText());

    rigExec::RigExecRigEvaluator evaluator(stage, rigPath);
    // Ordinary reports publish guides when requested. Golden and binary
    // verification enable guide publication explicitly for full coverage.
    evaluator.SetSolverGuidesEnabled(solverGuides);
    // Enabled before Compile so the trace holds the compile itself plus
    // every evaluated frame.
    if (!profileOut.empty()) {
        evaluator.SetProfilingEnabled(true);
    }
    gCpuReference=cpuReference;gExecCrossCheck=execCrossCheck;
    evaluator.cpuReference=cpuReference;
    std::vector<std::string> errors;
    const bool compiled = evaluator.Compile(&errors);
    for (const std::string &error : errors) {
        std::printf("  %s\n", error.c_str());
    }
    std::printf("  compile: %s (%zu mover applications, digest %zu)\n",
        compiled?"ok":"FAILED",evaluator.GetMoverOrder().size(),evaluator.GetBindingEpochDigest());
    if(!compiled)return 1;
    int status=0;
    if(execCrossCheck && evaluator.GetBakedProgram()) {
        std::string error;
        if(!rigExec::RigExecBakedProgramTesting::EnableExecCrossCheck(*evaluator.GetBakedProgram(),&error)) {
            std::printf("  exec crosscheck preparation failed: %s\n",error.c_str());return 1;
        }
        checkVisits[&evaluator]={evaluator.GetStageEditSerial(),evaluator.GetBakedProgram()};
    }
    if(!evaluator.GetBakedProgram()) {
        std::printf("  native program unavailable\n");return 1;
    }
    if (showTargets) {
        for (const rigExec::RigExecMoverRecord &record :
                 evaluator.GetMoverOrder()) {
            std::printf("    %3d %-28s %s\n", record.ordinal,
                        record.schemaType.GetText(),
                        record.moverPath.GetName().c_str());
            for (const SdfPath &target : record.targets) {
                std::printf("        -> %s\n", target.GetText());
            }
        }
    }

    if (!verifyBinary.empty()) {
        if (repeat > 1 || dragSteps > 0 || !profileOut.empty()) {
            std::printf("  note: --verify-binary ignores "
                        "--repeat/--drag/--profile\n");
        }
        return RunVerifyBinary(stage, rigPath, frames, dragInputs,
                               verifyBinary, poseOut, jointsOut);
    }
    if (!dragInputs.empty()) {
        std::printf("  note: --drag-input applies to --verify-binary "
                    "only; ignored\n");
    }

    if (frames.empty()) {
        // Default, not the NaN behind it: a stage with no authored range is
        // an unanimated asset, and the default time code is the time it
        // authors its values at. (The two are the same object -- UsdTimeCode
        // stores Default AS a NaN and IsDefault() tests for it -- so this
        // says what was already meant rather than changing it.)
        frames.push_back(stage->HasAuthoredTimeCodeRange()
                             ? UsdTimeCode(stage->GetStartTimeCode())
                             : UsdTimeCode::Default());
    }

    if (golden) {
        return RunGoldenProtocol(stage, rigPath, evaluator, frames,
            goldenFixture.empty() ? std::filesystem::path(stagePath).filename().string() : goldenFixture,
            goldenOut, goldenCheck, goldenCompact, goldenActions, goldenBackend, goldenProgram);
    }

    // Rest values, so displacements are reported against the authored
    // geometry rather than against the previous frame.
    std::map<SdfPath, VtVec3fArray> rest;
    // Steady-state warmup: evaluate each frame once before the timed loop so
    // the metric reflects per-frame cost with warm exec caches, not one-time
    // rig setup or first-visit exec warmup. The benchmark repeats each frame
    // `repeat` times, so first-visit cost is amortized anyway; the pose
    // returned here is discarded (the reporting pass feeds the gate).
    for (UsdTimeCode frame : frames) {
        (void)EvaluateChecked(evaluator,frame);
    }
    // The silent passes. They are the same call the reporting loop makes,
    // so they cost what a frame costs -- including the pose the evaluator
    // returns by value, which is most of what a caller pays for. Timed here
    // rather than around the process, because a process start costs more
    // than half a second and swamps the microseconds being compared.
    if (repeat > 1) {
        const auto began = std::chrono::steady_clock::now();
        for (int pass = 1; pass < repeat; ++pass) {
            for (UsdTimeCode frame : frames) {
                const rigExec::RigExecRigPose pose = EvaluateChecked(evaluator,frame);
                if (!pose.valid || pose.referenceMismatches ||
                    pose.comparisonMismatches) {
                    status = 1;
                }
            }
        }
        const double seconds =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - began).count();
        const size_t ran = frames.size() * size_t(repeat - 1);
        std::printf("  repeat: %zu frame(s) in %.3fs (%.1fus/frame)\n", ran,
                    seconds, ran ? seconds * 1e6 / double(ran) : 0.0);
    }

    JointExport jointExport;
    FILE *poseDump = nullptr;
    if (!poseOut.empty()) {
        poseDump = std::fopen(poseOut.c_str(), "w");
        if (!poseDump) {
            std::printf("  cannot open %s for writing\n", poseOut.c_str());
            return 2;
        }
    }
    for (UsdTimeCode frame : frames) {
        const rigExec::RigExecRigPose pose = EvaluateChecked(evaluator,frame);
        if (!jointsOut.empty()) {
            jointExport.Add(pose, frame.GetValue());
        }
        if (poseDump) {
            WritePoseDump(poseDump, pose, frame.GetValue());
        }
        std::printf("\n  frame %g: %s (%zu moved properties, %zu reference agreements / %zu mismatches, %zu executed ops)\n",
            frame.GetValue(),pose.valid?"valid":"INVALID",pose.movedProperties.size(),
            pose.referenceAgreements,pose.referenceMismatches,pose.executedOpCount);
        if (!pose.valid || pose.referenceMismatches ||
            pose.comparisonMismatches) {
            status = 1;
        }
        for (const std::string &diagnostic : pose.diagnostics) {
            std::printf("    diagnostic: %s\n", diagnostic.c_str());
        }
        if (showJoints) {
            for (const auto &[path, frameValue] : pose.jointFramesFinal) {
                std::printf("    %-46s %s%s\n", path.GetText(),
                            FormatVec(frameValue.points[0]).c_str(),
                            frameValue.IsValid() ? "" : "  INVALID");
            }
            for (const auto &[path, matrix] : pose.providerXforms) {
                // The basis, not just the origin. A constraint's whole job is
                // usually orientation -- the default preserve set pins the
                // origin -- so a translation-only report is blank exactly
                // where the interesting answer is.
                std::printf("    xform %-40s t%s\n", path.GetText(),
                            FormatVec(matrix.ExtractTranslation()).c_str());
                std::printf("      x%s y%s z%s\n",
                            FormatVec(matrix.GetRow3(0)).c_str(),
                            FormatVec(matrix.GetRow3(1)).c_str(),
                            FormatVec(matrix.GetRow3(2)).c_str());
            }
        }
        for (const auto &[path, value] : pose.movedProperties) {
            if (rest.find(path) == rest.end()) {
                VtValue authored;
                if (const UsdAttribute attribute =
                        stage->GetAttributeAtPath(path)) {
                    attribute.Get(&authored, UsdTimeCode::Default());
                }
                rest[path] = authored.IsHolding<VtVec3fArray>()
                                 ? authored.UncheckedGet<VtVec3fArray>()
                                 : VtVec3fArray();
            }
            ReportPoints(path.GetString(), value, rest[path]);
        }
    }

    // A drag is not a frame change: the time stands still and one attribute
    // moves, over and over, with a pose drawn between each pair of values.
    // That is the generation the interactive path has to be quick at, and it
    // is a different shape from an animation frame -- no time moved, so
    // every input that is a function of time is unchanged and only the cone
    // below the dragged control has anything to do.
    if (dragSteps > 0) {
        const SdfPath dragPath(dragPrim);
        const UsdPrim prim = stage->GetPrimAtPath(dragPath);
        const UsdAttribute attribute =
            prim ? prim.GetAttribute(TfToken(dragAttr)) : UsdAttribute();
        const UsdTimeCode dragFrame = frames.back();
        double original = 0;
        if (!AttributeDouble(attribute, dragFrame, &original)) {
            std::printf("\n  drag FAILED: %s has no double or float %s\n",
                        dragPrim.c_str(), dragAttr.c_str());
            status = 1;
        } else {
            // A triangle wave rather than a ramp: an animator's drag stays
            // in the neighbourhood of the value it started at, and a ramp
            // long enough to time would walk an envelope out of [0, 1] and
            // measure a constraint passing through instead of applying.
            // Away from zero for a value near it, back towards it for one
            // that is not, for the same reason.
            const double direction = original > 0.5 ? -1.0 : 1.0;

            // One generation at this time before the first measured step, so
            // that what every step measures is a drag and not the first
            // frame's cold caches.
            EvaluateChecked(evaluator,dragFrame);
            std::vector<double> stepUs;
            stepUs.reserve(size_t(dragSteps));
            for (int k = 0; k < dragSteps; ++k) {
                const double phase = double(k % 20);
                // 1..10 up, 11..2 down: twenty DISTINCT displacements, so
                // no step ever hands the rig the value the step before it
                // did -- which a cone re-execution would skip, and a
                // benchmark would then report as a cheap drag frame.
                const double bump =
                    direction * 0.005 *
                    (phase < 10 ? phase + 1 : 21 - phase);
                VtValue value;
                if (!DisplacedValue(attribute, dragFrame, bump, &value)) {
                    break;
                }
                const auto began = std::chrono::steady_clock::now();
                evaluator.SetInteractiveOverrides(
                    {rigExec::RigExecValueOverride{
                        dragPath, TfToken(), TfToken(dragAttr), value}});
                const rigExec::RigExecRigPose pose =
                    EvaluateChecked(evaluator,dragFrame);
                stepUs.push_back(
                    std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - began).count() *
                    1e6);
                if (!pose.valid || pose.referenceMismatches ||
                    pose.comparisonMismatches) {
                    status = 1;
                }
            }
            evaluator.ClearInteractiveOverrides();
            std::vector<double> sorted = stepUs;
            std::sort(sorted.begin(), sorted.end());
            const double median =
                sorted.empty() ? 0.0 : sorted[sorted.size() / 2];
            std::printf("\n  drag %s.%s: %zu step(s), median %.1fus, "
                        "min %.1fus, max %.1fus\n",
                        dragPrim.c_str(), dragAttr.c_str(), stepUs.size(),
                        median, sorted.empty() ? 0.0 : sorted.front(),
                        sorted.empty() ? 0.0 : sorted.back());
            for (size_t k = 0; k < stepUs.size(); ++k) {
                std::printf("    step %3zu %8.1fus\n", k, stepUs[k]);
            }
        }
    }

    if (poseDump) {
        std::fclose(poseDump);
        std::printf("\n  wrote %s (%zu frames)\n", poseOut.c_str(),
                    frames.size());
    }

    if (!jointsOut.empty()) {
        std::string error;
        if (!jointExport.Write(jointsOut, &error)) {
            std::printf("\n  joint export FAILED: %s\n", error.c_str());
            status = 1;
        } else {
            std::printf("\n  wrote %s (%zu frames%s)\n", jointsOut.c_str(),
                        frames.size(),
                        jointExport.Degenerate()
                            ? TfStringPrintf(", %zu degenerate frames written "
                                             "as identity",
                                             jointExport.Degenerate()).c_str()
                            : "");
        }
    }

    if (!profileOut.empty()) {
        std::string error;
        if (!evaluator.WriteProfileTrace(profileOut, &error)) {
            std::printf("\n  profile FAILED: %s\n", error.c_str());
            status = 1;
        } else {
            std::printf("\n  wrote %s (%zu events)\n", profileOut.c_str(),
                        evaluator.GetProfiler().GetEventCount());
            std::printf("  %10s %10s %7s  %s\n", "total_ms", "max_ms",
                        "count", "phase");
            for (const rigExec::RigExecProfileSummaryRow &row :
                 evaluator.GetProfiler().Summarize()) {
                std::printf("  %10.2f %10.2f %7zu  [%s] %s\n",
                            row.totalUs / 1000.0, row.maxUs / 1000.0,
                            row.count, row.category.c_str(),
                            row.name.c_str());
            }
        }
    }
    return status;
}
