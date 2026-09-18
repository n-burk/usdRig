//
// rigExecPose -- evaluate a rig and print what came out.
//
// The test suites assert against fixtures they own. This is the tool for the
// other case: an arbitrary stage carrying a RigExecRoot, evaluated at chosen
// frames so an author can see whether the rig they just wrote compiles, what
// the compiler objected to, where the joints ended up, and how far each moved
// property actually travelled.
//
//   rigExecPose <stage> [--rig <primPath>] [--frames 1001,1024,1048]
//               [--joints] [--targets] [--joints-out <file.usda>]
//               [--pose-out <file.txt>] [--repeat N]
//               [--profile <file.trace>] [--mode dynamic|baked|parity]
//               [--guides] [--require-baked]
//               [--drag <prim> <attr> <steps>]
//
// With no --frames it evaluates the stage's start time code (or Default when
// the stage has no time range). Exit status is non-zero when the rig fails to
// compile or an evaluation comes back invalid, so it can gate a build.
//
// --mode is a request, and giving it takes the decision away from everything
// else: without it the tool leaves the evaluator to decide for itself, which
// is RIGEXEC_EVALUATION_MODE if the session set one and the stage's own
// `uniform bool rigExec:baked` otherwise. The compile line reports the mode
// that resulted and who chose it, so a run that expected the program and got
// the dynamic path says so in its first three lines rather than in an
// accounting total at the end.
//
// --guides re-enables the observational solver-guide request, which is off
// by default here. pose.solverFrames is one of the domains the parity check
// compares, and with guides off both paths fill it with nothing -- so a
// ctest built on this tool compares an empty map unless it asks for them.
//
// --require-baked turns a fallback into a failure: it sets
// RIGEXEC_BAKE_REQUIRED for the evaluator, fails when a non-dynamic mode
// finds the rig unbakeable, and fails when fewer generations came from the
// program than frames were asked for. The second half is the one that
// catches a silent fallback that is not a refusal -- an interactive override
// the program cannot place, or a Run that handed the generation back.
//
// --profile records scoped phase timings (compile, property chains, pose
// seed, each solver batch and constraint, the exec snapshot, each geometry
// chain, derived maintenance) across every evaluated frame, writes them as
// Chrome Trace Event JSON to <file.trace> -- openable in Perfetto
// (ui.perfetto.dev) or chrome://tracing -- and prints a per-phase summary.
//
// --repeat N cycles the frame list N times instead of once. Only the last
// pass reports: the N-1 before it evaluate and throw the pose away, so the
// printed output of `--repeat 1` -- and of a command line that never names
// the option -- is exactly what it always was, while the wall clock of the
// process divides by N frames instead of one. It exists because the frames
// of this rig cost a few hundred microseconds each and a process start
// costs half a second; timing one frame means timing the process.
//
// --drag <prim> <attr> <steps> ramps one attribute through <steps> values
// with SetInteractiveOverrides, evaluating after each one, and prints the
// wall clock of every step plus the median and the minimum. It is the
// manipulator's frame: an override placed, a generation asked for, a pose
// drawn, over and over on one control. Nothing else about the run changes --
// the drag happens after the reported frames, so every line above it is the
// line a command line without the option prints.
//
// --pose-out writes every published domain of every evaluated generation in
// a canonical text form (%.17g doubles, %.9g floats), so two runs -- two
// builds, two modes -- can be compared byte for byte with `cmp`.
//
// --joints-out writes the evaluated joint frames, as asset-space matrices
// sampled at every requested frame, to a plain USD layer. It is deliberately
// schema-neutral -- a joint path list and a parallel matrix array per time
// sample, nothing else -- because its point is to hand the rig's own answer to
// something that is not RigExec: a converter to another skinning schema, or a
// comparison against one. It is a diagnostic export of what the evaluator
// computed, not a baked character.
//
#include "rigExec/frameExtraction.h"
#include "rigExec/rigEvaluator.h"
#include "rigExec/types.h"
#include "rigExecMath/pointFrame.h"

#include "pxr/base/arch/env.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/range3f.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/value.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/gf/vec3d.h"
// usd/stage.h only forward-declares UsdAttribute and UsdPrim.
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

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
    std::fprintf(out,
                 "counters parityMismatches=%zu parityAgreements=%zu "
                 "bakedParityMismatches=%zu solverOverrideRounds=%zu "
                 "solverOverridesConverged=%d solverEvaluations=%zu "
                 "revisionsCreated=%zu revisionsExecuted=%zu "
                 "schedulesBuilt=%zu\n",
                 pose.moverGraphParityMismatches,
                 pose.moverGraphParityAgreements, pose.bakedParityMismatches,
                 pose.solverOverrideRounds, int(pose.solverOverridesConverged),
                 pose.solverEvaluations, pose.moverGraphRevisionsCreated,
                 pose.moverGraphRevisionsExecuted,
                 pose.moverGraphSchedulesBuilt);
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

// The two halves of "which path is answering this rig", for the compile
// line. Spelled the way --mode spells them, so a reader can paste the
// reported mode back onto the command line and pin what they just saw.
const char *
ModeName(rigExec::RigExecEvaluationMode mode)
{
    switch (mode) {
    case rigExec::RigExecEvaluationMode::Baked: return "baked";
    case rigExec::RigExecEvaluationMode::BakedWithParityCheck: return "parity";
    case rigExec::RigExecEvaluationMode::Dynamic: break;
    }
    return "dynamic";
}

const char *
ModeSourceName(rigExec::RigExecEvaluationModeSource source)
{
    switch (source) {
    case rigExec::RigExecEvaluationModeSource::Explicit: return "--mode";
    case rigExec::RigExecEvaluationModeSource::Environment:
        return "RIGEXEC_EVALUATION_MODE";
    case rigExec::RigExecEvaluationModeSource::Attribute:
        return "rigExec:baked";
    case rigExec::RigExecEvaluationModeSource::Default: break;
    }
    return "the default";
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
            "[--profile <file.trace>] [--mode dynamic|baked|parity] "
            "[--guides] [--require-baked] "
            "[--drag <prim> <attr> <steps>]\n");
        return 2;
    }
    std::string stagePath = argv[1];
    std::string rigArg;
    std::string jointsOut;
    std::string poseOut;
    std::string profileOut;
    // Two facts, not one: which mode --mode named, and whether it was given
    // at all. An absent --mode is not a request for Dynamic -- it is this
    // tool declining to make the choice, which is what lets a stage carrying
    // rigExec:baked be opened through the program by running the tool the
    // way an author would.
    rigExec::RigExecEvaluationMode mode =
        rigExec::RigExecEvaluationMode::Dynamic;
    bool modeGiven = false;
    std::vector<UsdTimeCode> frames;
    std::string dragPrim, dragAttr;
    int dragSteps = 0;
    int repeat = 1;
    bool showJoints = false;
    bool showTargets = false;
    bool solverGuides = false;
    bool requireBaked = false;
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
        } else if (arg == "--require-baked") {
            requireBaked = true;
        } else if (arg == "--joints-out" && i + 1 < argc) {
            jointsOut = argv[++i];
        } else if (arg == "--pose-out" && i + 1 < argc) {
            poseOut = argv[++i];
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
        } else if (arg == "--mode" && i + 1 < argc) {
            const std::string value = argv[++i];
            modeGiven = true;
            if (value == "dynamic") {
                mode = rigExec::RigExecEvaluationMode::Dynamic;
            } else if (value == "baked") {
                mode = rigExec::RigExecEvaluationMode::Baked;
            } else if (value == "parity") {
                mode = rigExec::RigExecEvaluationMode::BakedWithParityCheck;
            } else {
                std::printf("unknown mode: %s "
                            "(dynamic | baked | parity)\n", value.c_str());
                return 2;
            }
        } else {
            std::printf("unknown argument: %s\n", arg.c_str());
            return 2;
        }
    }

    // The evaluator reads RIGEXEC_BAKE_REQUIRED once, into a function-local
    // static, so it has to be in the environment before the first evaluator
    // exists -- which is why this sits above the stage open rather than
    // beside the IsBakeable check it belongs with. It makes the evaluator
    // report a generation that fell back to the dynamic path as a baked
    // parity mismatch; the two checks below are this tool's own, independent
    // half, so --require-baked still means something in a build whose
    // evaluator does not honour the variable.
    if (requireBaked) {
        ArchSetEnv("RIGEXEC_BAKE_REQUIRED", "1", /* overwrite = */ true);
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
    // This tool reports joints, targets, and diagnostics; it never reads
    // pose.solverFrames, so the observational guide request is skipped --
    // unless --guides asks for it, which is what a parity run needs: the
    // guide frames are a compared domain, and two empty maps compare equal
    // no matter what the program would have put in them.
    evaluator.SetSolverGuidesEnabled(solverGuides);
    // Enabled before Compile so the trace holds the compile itself plus
    // every evaluated frame.
    if (!profileOut.empty()) {
        evaluator.SetProfilingEnabled(true);
    }
    // Before Compile, so the bake happens inside it rather than on the first
    // frame; the mode is a request either way.
    //
    // Only when --mode was GIVEN. SetEvaluationMode is the top of the
    // precedence ladder and setting it unasked would pin every run of this
    // tool to Dynamic -- which would make the tool the one place a rig's
    // own rigExec:baked can never be honoured, and the attribute untestable
    // through it.
    if (modeGiven) {
        evaluator.SetEvaluationMode(mode);
    }
    std::vector<std::string> errors;
    const bool compiled = evaluator.Compile(&errors);
    for (const std::string &error : errors) {
        std::printf("  %s\n", error.c_str());
    }
    // The mode is READ BACK rather than reported from the parsed argument:
    // Compile is where the rig's own rigExec:baked is consulted, so the
    // evaluator is the only thing that knows which path this run ended up
    // on, and printing what was asked for instead would name Dynamic on
    // every attribute-driven run.
    std::printf("  compile: %s (%zu mover applications, digest %zu, "
                "mode %s from %s)\n",
                compiled ? "ok" : "FAILED",
                evaluator.GetMoverOrder().size(),
                evaluator.GetBindingEpochDigest(),
                ModeName(evaluator.GetEvaluationMode()),
                ModeSourceName(evaluator.GetEvaluationModeSource()));
    if (!compiled) {
        return 1;
    }
    // Everything below asks the evaluator rather than the command line, for
    // the reason the compile line does.
    const rigExec::RigExecEvaluationMode resolvedMode =
        evaluator.GetEvaluationMode();
    int status = 0;
    // Only in a non-default mode: the reasons are the actionable half of a
    // fallback, and printing them unasked would change every existing run.
    if (resolvedMode != rigExec::RigExecEvaluationMode::Dynamic) {
        std::vector<std::string> reasons;
        if (!evaluator.IsBakeable(&reasons)) {
            std::printf("  not bakeable; evaluating dynamically\n");
            for (const std::string &reason : reasons) {
                std::printf("    %s\n", reason.c_str());
            }
            // A fallback is a correct answer, so it is only a failure when
            // the caller said the bake was the point.
            if (requireBaked) {
                std::printf("  FAIL: not bakeable\n");
                status = 1;
            }
        }
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

    // Rest values, so displacements are reported against the authored
    // geometry rather than against the previous frame.
    std::map<SdfPath, VtVec3fArray> rest;
    // Steady-state warmup: evaluate each frame once before the timed loop so
    // the metric reflects per-frame cost with warm exec caches, not one-time
    // rig setup or first-visit exec warmup. The benchmark repeats each frame
    // `repeat` times, so first-visit cost is amortized anyway; the pose
    // returned here is discarded (the reporting pass feeds the gate).
    for (UsdTimeCode frame : frames) {
        (void)evaluator.Evaluate(frame);
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
                const rigExec::RigExecRigPose pose = evaluator.Evaluate(frame);
                if (!pose.valid || pose.moverGraphParityMismatches ||
                    pose.bakedParityMismatches) {
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
        const rigExec::RigExecRigPose pose = evaluator.Evaluate(frame);
        if (!jointsOut.empty()) {
            jointExport.Add(pose, frame.GetValue());
        }
        if (poseDump) {
            WritePoseDump(poseDump, pose, frame.GetValue());
        }
        std::printf("\n  frame %g: %s  (%zu moved properties, "
                    "%zu parity agreements / %zu mismatches, "
                    "%zu override rounds%s)\n",
                    frame.GetValue(), pose.valid ? "valid" : "INVALID",
                    pose.movedProperties.size(),
                    pose.moverGraphParityAgreements,
                    pose.moverGraphParityMismatches,
                    pose.solverOverrideRounds,
                    pose.solverOverridesConverged ? "" : ", NOT CONVERGED");
        if (!pose.valid || pose.moverGraphParityMismatches ||
            pose.bakedParityMismatches) {
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

    // The accounting, in every non-dynamic mode. A run that reports a mode
    // it never took is the failure this tool used to print as success, and a
    // human reading the output should see the same fact a ctest asserts.
    if (resolvedMode != rigExec::RigExecEvaluationMode::Dynamic) {
        // frames.size() * repeat, which is frames.size() itself unless
        // --repeat asked for more: the line a reader has always seen.
        const size_t evaluated = frames.size() * size_t(repeat);
        std::printf("\n  baked: %zu/%zu generation(s), %zu build(s), "
                    "%zu attempt(s)\n",
                    evaluator.GetBakedGenerationCount(), evaluated,
                    evaluator.GetBakedProgramBuildCount(),
                    evaluator.GetBakedProgramBuildAttemptCount());
        // Asked after the loop rather than per frame: an in-epoch rebuild is
        // legal and still answers its generation from the program. This is
        // the half that catches a fallback which is NOT a refusal -- an
        // override the program cannot place, or a Run that declined -- since
        // neither of those makes IsBakeable false.
        if (requireBaked &&
            evaluator.GetBakedGenerationCount() != evaluated) {
            std::printf("  FAIL: only %zu of %zu generation(s) came from the "
                        "program\n",
                        evaluator.GetBakedGenerationCount(), evaluated);
            status = 1;
        }
    }

    // ---- the manipulator's frame ------------------------------------------
    //
    // A drag is not a frame change: the time stands still and one attribute
    // moves, over and over, with a pose drawn between each pair of values.
    // That is the generation the interactive path has to be quick at, and it
    // is a different shape from an animation frame -- no time moved, so
    // every input that is a function of time is unchanged and only the cone
    // below the dragged control has anything to do.
    //
    // Measured AFTER the accounting above, so the generation counts a reader
    // (and --require-baked) sees still describe the requested frames alone;
    // what the drag itself did with the program is reported here instead.
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
            const size_t generationsBefore =
                evaluator.GetBakedGenerationCount();
            // One generation at this time before the first measured step, so
            // that what every step measures is a drag and not the first
            // frame's cold caches.
            evaluator.Evaluate(dragFrame);
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
                    evaluator.Evaluate(dragFrame);
                stepUs.push_back(
                    std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - began).count() *
                    1e6);
                if (!pose.valid || pose.moverGraphParityMismatches ||
                    pose.bakedParityMismatches) {
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
            if (resolvedMode != rigExec::RigExecEvaluationMode::Dynamic) {
                std::printf("    baked: %zu of %zu drag generation(s)\n",
                            evaluator.GetBakedGenerationCount() -
                                generationsBefore,
                            stepUs.size() + 1);
            }
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
