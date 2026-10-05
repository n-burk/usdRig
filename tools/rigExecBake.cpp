// rigExecBake -- bake a rig to a .rigexec binary.
//   rigExecBake <stage> [--rig <primPath>] [--time T]
//               [--presentation <file.rexp>] [--report-static]
//               -o <file.rigexec>
// Where rigExecPose evaluates and prints, this compiles the epoch through
// the baked program, runs it once at T (default: the stage's start time
// code when it authors a range, else 0) and serializes the static graph
// with every input's value at T as its default. --presentation embeds a
// REXP buffer whose controls name listed inputs. --report-static lists, after
// the bake, every static datum whose source the stage animates, which the
// binary holds at T only; it never changes the exit status.
// A bake is of the PROGRAM, so the tool pins the Baked mode and treats
// anything else as a failure -- the --require-baked rigExecPose opts into,
// here unconditional, because baking dynamic numbers into a file whose
// reader promises program semantics would be a binary no parity check can
// hold to account. Exit status is 2 on usage errors and 1 when the rig fails
// to compile, to bake, or to write, so it can gate a build the way
// rigExecPose does.
#include "rigExecBake/bake.h"
#include "rigExecBake/staticReport.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/rigEvaluator.h"

#include "pxr/base/arch/env.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

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

// The whole of \p text as one finite number.
bool
ParseTime(const char *text, double *out)
{
    char *end = nullptr;
    const double value = std::strtod(text, &end);
    if (end == text || *end != '\0' || !std::isfinite(value)) {
        return false;
    }
    *out = value;
    return true;
}

}  // namespace

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::printf(
            "usage: rigExecBake <stage> [--rig <primPath>] [--time T] "
            "[--presentation <file.rexp>] [--report-static] "
            "-o <file.rigexec>\n");
        return 2;
    }
    std::string stagePath = argv[1];
    std::string rigArg;
    std::string output;
    std::string presentationPath;
    double time = std::nan("");
    bool haveTime = false;
    bool reportStatic = false;
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--rig" && i + 1 < argc) {
            rigArg = argv[++i];
        } else if (arg == "--time" && i + 1 < argc) {
            const char *text = argv[++i];
            if (!ParseTime(text, &time)) {
                std::printf("--time wants a finite number, not '%s'\n",
                            text);
                return 2;
            }
            haveTime = true;
        } else if (arg == "--presentation" && i + 1 < argc) {
            presentationPath = argv[++i];
        } else if (arg == "--report-static") {
            reportStatic = true;
        } else if (arg == "-o" && i + 1 < argc) {
            output = argv[++i];
        } else {
            std::printf("unknown argument: %s\n", arg.c_str());
            return 2;
        }
    }
    if (output.empty()) {
        std::printf("-o wants an output file\n");
        return 2;
    }
    std::vector<uint8_t> presentation;
    if (!presentationPath.empty()) {
        std::ifstream stream(presentationPath, std::ios::binary);
        if (!stream) {
            std::printf("FATAL: cannot read %s\n", presentationPath.c_str());
            return 2;
        }
        presentation.assign(std::istreambuf_iterator<char>(stream),
                            std::istreambuf_iterator<char>());
        if (presentation.empty()) {
            std::printf("FATAL: %s is empty\n", presentationPath.c_str());
            return 2;
        }
    }

    // Before the first evaluator exists: the evaluator reads
    // RIGEXEC_BAKE_REQUIRED once, into a function-local static. Baking IS
    // the point of this tool, so a fallback is always a failure here.
    ArchSetEnv("RIGEXEC_BAKE_REQUIRED", "1", /* overwrite = */ true);

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
    // Pinned, not requested: this tool has no dynamic mode. Set before
    // Compile so the program builds inside it rather than on first use.
    evaluator.SetEvaluationMode(rigExec::RigExecEvaluationMode::Baked);
    rigExec::RigExecBakeOpts opts;
    opts.time = haveTime ? time
                         : rigExec::RigExecBakedProbeTime(stage).GetValue();
    opts.presentation = std::move(presentation);
    rigExec::RigExecBakeResult result;
    std::string error;
    if (!rigExec::RigExecBakeToBinary(evaluator, opts, &result, &error)) {
        std::printf("  FAIL: %s\n", error.c_str());
        return 1;
    }
    {
        std::ofstream stream(output, std::ios::binary);
        if (!stream) {
            std::printf("  FAIL: cannot open %s for writing\n",
                        output.c_str());
            return 1;
        }
        stream.write(reinterpret_cast<const char *>(result.bytes.data()),
                     std::streamsize(result.bytes.size()));
        if (!stream) {
            std::printf("  FAIL: cannot write %s\n", output.c_str());
            return 1;
        }
    }
    std::printf("  baked at time %.17g to %s (%zu bytes)\n", opts.time,
                output.c_str(), result.bytes.size());
    if (reportStatic) {
        std::vector<rigExec::RigExecBakeStaticEntry> entries;
        if (rigExec::RigExecBakeStaticReport(evaluator, &entries, &error)) {
            for (const rigExec::RigExecBakeStaticEntry &entry : entries) {
                std::printf("  static %s: %s\n", entry.field.c_str(),
                            entry.source.c_str());
            }
            std::printf("  static report: %zu animated source(s) held at "
                        "the bake time\n",
                        entries.size());
        } else {
            std::printf("  static report unavailable: %s\n", error.c_str());
        }
    }
    return 0;
}
