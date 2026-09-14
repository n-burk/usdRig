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
//               [--profile <file.trace>] [--mode dynamic|baked|parity]
//
// With no --frames it evaluates the stage's start time code (or Default when
// the stage has no time range). Exit status is non-zero when the rig fails to
// compile or an evaluation comes back invalid, so it can gate a build.
//
// --profile records scoped phase timings (compile, property chains, pose
// seed, each solver batch and constraint, the exec snapshot, each geometry
// chain, derived maintenance) across every evaluated frame, writes them as
// Chrome Trace Event JSON to <file.trace> -- openable in Perfetto
// (ui.perfetto.dev) or chrome://tracing -- and prints a per-phase summary.
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

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/range3f.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/base/vt/array.h"
// usd/stage.h only forward-declares UsdAttribute and UsdPrim.
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
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

std::vector<double>
ParseFrames(const std::string &text)
{
    std::vector<double> frames;
    for (const std::string &piece : TfStringSplit(text, ",")) {
        if (!piece.empty()) {
            frames.push_back(std::atof(piece.c_str()));
        }
    }
    return frames;
}

}  // namespace

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::printf(
            "usage: rigExecPose <stage> [--rig <primPath>] "
            "[--frames a,b,c] [--joints] [--targets] "
            "[--joints-out <file.usda>] [--profile <file.trace>] "
            "[--mode dynamic|baked|parity]\n");
        return 2;
    }
    std::string stagePath = argv[1];
    std::string rigArg;
    std::string jointsOut;
    std::string profileOut;
    rigExec::RigExecEvaluationMode mode =
        rigExec::RigExecEvaluationMode::Dynamic;
    std::vector<double> frames;
    bool showJoints = false;
    bool showTargets = false;
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
        } else if (arg == "--joints-out" && i + 1 < argc) {
            jointsOut = argv[++i];
        } else if (arg == "--profile" && i + 1 < argc) {
            profileOut = argv[++i];
        } else if (arg == "--mode" && i + 1 < argc) {
            const std::string value = argv[++i];
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
    // pose.solverFrames, so the observational guide request is skipped.
    evaluator.SetSolverGuidesEnabled(false);
    // Enabled before Compile so the trace holds the compile itself plus
    // every evaluated frame.
    if (!profileOut.empty()) {
        evaluator.SetProfilingEnabled(true);
    }
    // Before Compile, so the bake happens inside it rather than on the first
    // frame; the mode is a request either way.
    evaluator.SetEvaluationMode(mode);
    std::vector<std::string> errors;
    const bool compiled = evaluator.Compile(&errors);
    for (const std::string &error : errors) {
        std::printf("  %s\n", error.c_str());
    }
    std::printf("  compile: %s (%zu mover applications, digest %zu)\n",
                compiled ? "ok" : "FAILED",
                evaluator.GetMoverOrder().size(),
                evaluator.GetBindingEpochDigest());
    if (!compiled) {
        return 1;
    }
    // Only in a non-default mode: the reasons are the actionable half of a
    // fallback, and printing them unasked would change every existing run.
    if (mode != rigExec::RigExecEvaluationMode::Dynamic) {
        std::vector<std::string> reasons;
        if (!evaluator.IsBakeable(&reasons)) {
            std::printf("  not bakeable; evaluating dynamically\n");
            for (const std::string &reason : reasons) {
                std::printf("    %s\n", reason.c_str());
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
        frames.push_back(stage->HasAuthoredTimeCodeRange()
                             ? stage->GetStartTimeCode()
                             : UsdTimeCode::Default().GetValue());
    }

    // Rest values, so displacements are reported against the authored
    // geometry rather than against the previous frame.
    std::map<SdfPath, VtVec3fArray> rest;

    JointExport jointExport;
    int status = 0;
    for (double frame : frames) {
        const rigExec::RigExecRigPose pose = evaluator.Evaluate(frame);
        if (!jointsOut.empty()) {
            jointExport.Add(pose, frame);
        }
        std::printf("\n  frame %g: %s  (%zu moved properties, "
                    "%zu parity agreements / %zu mismatches, "
                    "%zu override rounds%s)\n",
                    frame, pose.valid ? "valid" : "INVALID",
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
