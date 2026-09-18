//
// rigExecBake -- bake a rig to a .rigexec binary.
//
//   rigExecBake <stage> [--rig <primPath>] --frames a,b,c -o <file.rigexec>
//               [--manifest-out <file.json>]
//
// Where rigExecPose evaluates and prints, this compiles the epoch through
// the baked program and serializes it. A bake is of the PROGRAM, so the
// tool pins the Baked mode and treats anything else as a failure -- the
// --require-baked rigExecPose opts into, here unconditional, because baking
// dynamic numbers into a file whose reader promises program semantics would
// be a binary no parity check can hold to account. Exit status is 2 on
// usage errors and 1 when the rig fails to compile, to bake, or to write,
// so it can gate a build the way rigExecPose does.
//

#include "rigExecBake/bake.h"
#include "rigExec/rigEvaluator.h"

#include "pxr/base/arch/env.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
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
            "usage: rigExecBake <stage> [--rig <primPath>] "
            "--frames a,b,c -o <file.rigexec> "
            "[--manifest-out <file.json>]\n");
        return 2;
    }
    std::string stagePath = argv[1];
    std::string rigArg;
    std::string output;
    std::string manifestOut;
    std::vector<double> frames;
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--rig" && i + 1 < argc) {
            rigArg = argv[++i];
        } else if (arg == "--frames" && i + 1 < argc) {
            frames = ParseFrames(argv[++i]);
        } else if (arg == "-o" && i + 1 < argc) {
            output = argv[++i];
        } else if (arg == "--manifest-out" && i + 1 < argc) {
            manifestOut = argv[++i];
        } else {
            std::printf("unknown argument: %s\n", arg.c_str());
            return 2;
        }
    }
    if (frames.empty()) {
        std::printf("--frames wants a non-empty frame list\n");
        return 2;
    }
    if (output.empty()) {
        std::printf("-o wants an output file\n");
        return 2;
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
    opts.frames = frames;
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
    if (!manifestOut.empty()) {
        std::ofstream stream(manifestOut);
        if (!stream) {
            std::printf("  FAIL: cannot open %s for writing\n",
                        manifestOut.c_str());
            return 1;
        }
        stream << result.manifestJson;
        if (!stream) {
            std::printf("  FAIL: cannot write %s\n", manifestOut.c_str());
            return 1;
        }
    }
    std::printf("  baked %zu frame(s) to %s (%zu bytes)\n", frames.size(),
                output.c_str(), result.bytes.size());
    return 0;
}
