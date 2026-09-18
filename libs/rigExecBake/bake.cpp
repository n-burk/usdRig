//
// .rigexec baking.
//

#include "rigExecBake/bake.h"
#include "rigExecBake/serialize.h"
#include "rigExecBake/capture.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecBinary/container.h"

#include "pxr/usd/usd/timeCode.h"

#include <cstdio>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {
namespace {

// %.17g round-trips a double, the same canonical form rigExecPose --pose-out
// writes: a manifest compared across builds compares exactly.
std::string
_FormatDouble(double value, char *buffer, size_t size)
{
    std::snprintf(buffer, size, "%.17g", value);
    return std::string(buffer);
}

// JSON string escaping for the manifest. Paths carry no quotes or controls,
// but the manifest also echoes the rig path a caller supplied, so this is
// complete rather than trusting the input.
std::string
_EscapeJson(const std::string &text)
{
    std::string out;
    out.reserve(text.size() + 2);
    out.push_back('"');
    for (const char c : text) {
        switch (c) {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
            if (static_cast<unsigned char>(c) < 0x20) {
                char buffer[8];
                std::snprintf(buffer, sizeof(buffer), "\\u%04x", c);
                out += buffer;
            } else {
                out.push_back(c);
            }
            break;
        }
    }
    out.push_back('"');
    return out;
}

}  // namespace

bool
RigExecBakeToBinary(RigExecRigEvaluator &evaluator,
                    const RigExecBakeOpts &opts, RigExecBakeResult *result,
                    std::string *error)
{
    auto Fail = [&](const std::string &what) {
        if (error) {
            *error = what;
        }
        return false;
    };
    if (!result) {
        return Fail("no result to bake into");
    }
    if (opts.frames.empty()) {
        return Fail("no frames to bake");
    }
    if (opts.targetReaderVersion != RigExecBinaryMajor(RigExecBinaryVersion)) {
        return Fail("unsupported target reader version");
    }
    std::vector<std::string> compileErrors;
    if (!evaluator.Compile(&compileErrors)) {
        std::string joined = "compile failed";
        for (const std::string &message : compileErrors) {
            joined += "\n  " + message;
        }
        return Fail(joined);
    }
    // A bake is of the program: an epoch it cannot express fails HERE,
    // naming the feature, rather than baking dynamic numbers into a file
    // whose reader promises program semantics.
    std::vector<std::string> reasons;
    if (!evaluator.IsBakeable(&reasons)) {
        std::string joined = "epoch is not bakeable";
        for (const std::string &reason : reasons) {
            joined += "\n  " + reason;
        }
        return Fail(joined);
    }
    if (evaluator.HasInteractiveOverrides()) {
        return Fail("cannot bake with interactive overrides standing");
    }
    RigExecBinaryWriter writer;
    std::string captureError;
    RigExecBakeCapture capture(evaluator, &writer, &captureError);
    if (!capture.Valid()) {
        return Fail(captureError);
    }
    const size_t bakedBefore = evaluator.GetBakedGenerationCount();
    std::vector<SdfPath> joints;
    bool first = true;
    char number[32];
    for (double frame : opts.frames) {
        RigExecRigPose pose;
        if (!capture.CaptureFrame(frame, &pose, error)) {
            return false;
        }
        if (!pose.valid) {
            return Fail("invalid generation at frame " +
                        _FormatDouble(frame, number, sizeof(number)));
        }
        // The joint set is epoch-structural: a generation that changes it
        // is a different rig mid-bake, and the binary's joint table could
        // not name both.
        std::vector<SdfPath> order;
        order.reserve(pose.jointFramesFinal.size());
        for (const auto &[path, _] : pose.jointFramesFinal) {
            order.push_back(path);
        }
        if (first) {
            joints = order;
            first = false;
        } else if (joints != order) {
            return Fail("the joint set changed between frames");
        }
    }
    // The IsBakeable gate above is necessary but not sufficient: an
    // in-epoch refusal the reasons do not name -- an override the program
    // cannot place, or a Run that handed the generation back -- still
    // answers dynamically. The generation count is what catches those, the
    // same accounting rigExecPose --require-baked performs.
    if (evaluator.GetBakedGenerationCount() - bakedBefore != opts.frames.size()) {
        return Fail("not every frame came from the baked program");
    }

    const RigExecBakedProgram *baked = evaluator.GetBakedProgram();
    if (!baked) {
        return Fail("no baked program standing after baked generations");
    }
    const RigExecBakedProgramImpl &program = baked->GetStepGraph();
    // The writer was created before the frame loop, so the capture could
    // intern into its string table.
    writer.AddString(evaluator.GetRigPath().GetString());
    std::vector<uint32_t> jointStrings;
    jointStrings.reserve(joints.size());
    for (const SdfPath &joint : joints) {
        jointStrings.push_back(writer.AddString(joint.GetString()));
    }
    std::vector<uint8_t> payload;
    if (!RigExecWireEncodeSlotMeta(
            RigExecBakeConvertSlotMeta(program, &writer), &payload)) {
        return Fail("cannot encode the slot inventory");
    }
    writer.AddSection(RigExecBinarySection::SlotMeta, payload);
    payload.clear();
    if (!RigExecWireEncodeConstants(
            RigExecBakeConvertConstants(program, &writer), &payload)) {
        return Fail("cannot encode the epoch constants");
    }
    writer.AddSection(RigExecBinarySection::Constants, payload);
    payload.clear();
    const std::vector<RigExecWireStep> wireSteps =
        RigExecBakeConvertSteps(program, &writer);
    if (!RigExecWireEncodeSteps(wireSteps, &payload)) {
        return Fail("cannot encode the step list");
    }
    writer.AddSection(RigExecBinarySection::Steps, payload);
    payload.clear();
    const RigExecWireClustering wireClustering =
        RigExecBakeConvertClustering(program);
    if (!RigExecWireEncodeClustering(wireClustering, &payload)) {
        return Fail("cannot encode the clusters");
    }
    writer.AddSection(RigExecBinarySection::Clusters, payload);
    payload.clear();
    if (!RigExecWireEncodeCones(RigExecBakeConvertCones(program),
                                &payload)) {
        return Fail("cannot encode the cones");
    }
    writer.AddSection(RigExecBinarySection::Cones, payload);
    payload.clear();
    if (!RigExecWireEncodeDomainPose(
            RigExecBakeConvertDomainPose(program, &writer), &payload)) {
        return Fail("cannot encode the pose tables");
    }
    writer.AddSection(RigExecBinarySection::DomainPose, payload);
    payload.clear();
    const RigExecWireDomainGeometry wireGeometry =
        RigExecBakeConvertDomainGeometry(program, &writer);
    size_t revisionCount = 0;
    size_t curvenetRevisions = 0;
    size_t curvenetBindsHeld = 0;
    for (const RigExecWireChain &chain : wireGeometry.chains) {
        revisionCount += chain.revisions.size();
        for (const RigExecWireRevision &revision : chain.revisions) {
            // Curvenet is op 10 in RigExecRevisionOp order.
            if (revision.op == 10) {
                ++curvenetRevisions;
                if (revision.curvenetBindInputsHeld) {
                    ++curvenetBindsHeld;
                }
            }
        }
    }
    if (!RigExecWireEncodeDomainGeometry(wireGeometry, &payload)) {
        return Fail("cannot encode the geometry tables");
    }
    writer.AddSection(RigExecBinarySection::DomainGeometry, payload);
    payload.clear();
    if (!RigExecWireEncodeInputTable(capture.GetTable(), &payload)) {
        return Fail("cannot encode the input table");
    }
    writer.AddSection(RigExecBinarySection::InputTable, payload);
    payload.clear();
    std::string manifest = "{\n";
    manifest += "  \"format\": 1,\n";
    manifest += "  \"rig\": " + _EscapeJson(evaluator.GetRigPath().GetString()) +
                ",\n";
    manifest += "  \"frames\": [";
    for (size_t i = 0; i < opts.frames.size(); ++i) {
        manifest += (i ? ", " : "") +
                    _FormatDouble(opts.frames[i], number, sizeof(number));
    }
    manifest += "],\n";
    manifest += "  \"joints\": [";
    for (size_t i = 0; i < jointStrings.size(); ++i) {
        manifest += (i ? ", " : "") + std::to_string(jointStrings[i]);
    }
    manifest += "],\n";
    manifest += "  \"steps\": " + std::to_string(wireSteps.size()) + ",\n";
    manifest += "  \"clusters\": " +
                std::to_string(wireClustering.clusters.size()) + ",\n";
    manifest += "  \"chains\": " +
                std::to_string(wireGeometry.chains.size()) + ",\n";
    manifest += "  \"revisions\": " + std::to_string(revisionCount) + ",\n";
    manifest += "  \"weightObjects\": " +
                std::to_string(wireGeometry.weightObjects.size()) + ",\n";
    manifest += "  \"curvenetRevisions\": " +
                std::to_string(curvenetRevisions) + ",\n";
    manifest += "  \"curvenetBindsHeld\": " +
                std::to_string(curvenetBindsHeld) + ",\n";
    manifest += "  \"inputs\": " +
                std::to_string(capture.GetTable().directory.size()) +
                ",\n";
    manifest += "  \"movers\": " +
                std::to_string(evaluator.GetMoverOrder().size()) + ",\n";
    // The compile notices a fresh evaluator seeds its first generation
    // with (inert movers, purpose warnings). The runtime replays them
    // ahead of the program lines on its first Execute, exactly once.
    manifest += "  \"compileDiagnostics\": [";
    for (size_t i = 0; i < compileErrors.size(); ++i) {
        manifest += (i ? ", " : "") + _EscapeJson(compileErrors[i]);
    }
    manifest += "]\n";
    manifest += "}\n";
    writer.AddSection(RigExecBinarySection::Manifest,
                      reinterpret_cast<const uint8_t *>(manifest.data()),
                      manifest.size());

    result->bytes = writer.Finish();
    result->manifestJson = manifest;
    return true;
}

}  // namespace rigExec
