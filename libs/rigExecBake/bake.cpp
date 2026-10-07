// .rigexec baking.
#include "rigExecBake/bake.h"
#include "rigExecBake/propertyChainsBake.h"
#include "rigExecBake/serialize.h"
#include "rigExecBake/capture.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/movers/moverRegistry.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecBinary/container.h"
#include "rigExecBinary/external.h"

#include "pxr/usd/usd/timeCode.h"

#include <cstdio>
#include <map>

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

// The plugin movers' half of the file: each one's epoch bytes, and the
// frame bytes it encoded on every baked frame, interned so a payload that
// does not change is stored once.
class _ExternalExport {
public:
    // Lists the program's plugin revisions on the first frame, when the
    // program certainly stands; refuses one whose plugin cannot encode.
    bool List(const RigExecBakedProgramImpl &program,
              RigExecBinaryWriter *writer, std::string *error)
    {
        for (size_t c = 0; c < program.chains.size(); ++c) {
            const auto &revisions = program.chains[c].revisions;
            for (size_t r = 0; r < revisions.size(); ++r) {
                const auto &revision = revisions[r];
                if (revision.op != RigExecRevisionOp::External) {
                    continue;
                }
                const TfToken type = revision.moverPrim.GetTypeName();
                const RigExecMoverHandler *handler =
                    RigExecFindMoverHandler(type);
                if (!handler || !handler->encodeExternal) {
                    *error = "cannot export external mover " +
                             revision.moverPath.GetString() + ": " +
                             type.GetString() +
                             " provides no .rigexec encoding";
                    return false;
                }
                RigExecWireExternalRevision entry;
                entry.chain = uint32_t(c);
                entry.revision = uint32_t(r);
                entry.type = writer->AddString(type.GetString());
                _wire.revisions.push_back(std::move(entry));
                _handlers.push_back(handler);
            }
        }
        _haveEpoch.assign(_handlers.size(), false);
        return true;
    }

    // Encodes every plugin revision's payload as the frame just captured
    // left it. A revision with no payload -- its assembly failed -- records
    // that, and playback fails it on the same frame.
    bool Capture(const RigExecBakedProgramImpl &program, double frame,
                 std::string *error)
    {
        std::vector<uint32_t> row(_handlers.size(),
                                  RigExecWireExternalNoFrame);
        for (size_t k = 0; k < _handlers.size(); ++k) {
            RigExecWireExternalRevision &entry = _wire.revisions[k];
            const auto &revision = program.chains[entry.chain]
                                       .revisions[entry.revision];
            const RigExecMoverParameters &parameters = revision.parameters;
            if (!parameters.valid || parameters.externalData.IsEmpty()) {
                continue;
            }
            std::vector<uint8_t> epoch, bytes;
            char number[32];
            if (!_handlers[k]->encodeExternal(parameters.externalData,
                                              revision.binding, &epoch,
                                              &bytes)) {
                *error = "external mover " + revision.moverPath.GetString() +
                         " could not encode its payload at frame " +
                         _FormatDouble(frame, number, sizeof(number));
                return false;
            }
            if (!_haveEpoch[k]) {
                entry.epoch = std::move(epoch);
                _haveEpoch[k] = true;
            } else if (epoch != entry.epoch) {
                *error = "external mover " + revision.moverPath.GetString() +
                         " encoded different epoch bytes at frame " +
                         _FormatDouble(frame, number, sizeof(number)) +
                         "; they must not vary within an epoch";
                return false;
            }
            const auto interned = _blobs.emplace(
                std::move(bytes), uint32_t(_wire.blobs.size()));
            if (interned.second) {
                _wire.blobs.push_back(interned.first->first);
            }
            row[k] = interned.first->second;
        }
        _wire.frames.push_back(std::move(row));
        return true;
    }

    // Queues the section; a rig with no plugin mover writes none, so its
    // file is what an earlier writer produced.
    bool Write(RigExecBinaryWriter *writer, std::string *error) const
    {
        if (_handlers.empty()) {
            return true;
        }
        std::vector<uint8_t> payload;
        if (!RigExecWireEncodeExternalMovers(_wire, &payload)) {
            *error = "cannot encode the external movers";
            return false;
        }
        writer->AddSection(RigExecBinarySection::ExternalMovers, payload);
        return true;
    }

private:
    std::vector<const RigExecMoverHandler *> _handlers;
    std::vector<bool> _haveEpoch;
    std::map<std::vector<uint8_t>, uint32_t> _blobs;
    RigExecWireExternalMovers _wire;
};

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
    RigExecBakeCapture capture(evaluator, &writer, &captureError,
                               opts.overridableInputs);
    if (opts.overridableInputs) {
        // One byte, written only when the rule was widened, so a default
        // bake is byte-for-byte what it always was.
        writer.AddSection(RigExecBinarySection::InputPolicy,
                          std::vector<uint8_t>{1});
    }
    if (!capture.Valid()) {
        return Fail(captureError);
    }
    const size_t bakedBefore = evaluator.GetBakedGenerationCount();
    std::vector<SdfPath> joints;
    _ExternalExport external;
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
        const RigExecBakedProgram *program = evaluator.GetBakedProgram();
        if (!program) {
            return Fail("no baked program at frame " +
                        _FormatDouble(frame, number, sizeof(number)));
        }
        std::string externalError;
        if ((first && !external.List(program->GetStepGraph(), &writer,
                                     &externalError)) ||
            !external.Capture(program->GetStepGraph(), frame,
                              &externalError)) {
            return Fail(externalError);
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
    const RigExecWireDomainPose wirePose =
        RigExecBakeConvertDomainPose(program, &writer);
    if (!RigExecWireEncodeDomainPose(wirePose, &payload)) {
        return Fail("cannot encode the pose tables");
    }
    writer.AddSection(RigExecBinarySection::DomainPose, payload);
    payload.clear();
    // Sparse: only solvers hanging from a start provider. Absent in
    // binaries baked by major 1 minor 0, which is not an error: those load
    // with every chain absolute, exactly as before.
    std::vector<RigExecWireSolverStart> starts;
    for (size_t i = 0; i < wirePose.solvers.size(); ++i) {
        const RigExecWireSolver &solver = wirePose.solvers[i];
        if (solver.start < 0) {
            continue;
        }
        RigExecWireSolverStart entry;
        entry.solver = uint32_t(i);
        entry.start = solver.start;
        entry.rest = solver.startRest;
        entry.read = solver.startRead;
        starts.push_back(entry);
    }
    if (!RigExecWireEncodeSolverStarts(starts, &payload)) {
        return Fail("cannot encode the solver starts");
    }
    writer.AddSection(RigExecBinarySection::SolverStart, payload);
    payload.clear();
    // Sparse: only interpolators that read dials or measure translation.
    // Absent in binaries baked before minor 1, which load with every
    // interpolator solving a transform's rotation alone -- which is what
    // those files were baked from.
    std::vector<RigExecWirePoseNumeric> numerics;
    for (size_t i = 0; i < wirePose.poseInterpolators.size(); ++i) {
        const RigExecWirePoseInterpolator &interp =
            wirePose.poseInterpolators[i];
        if (interp.valueInputs.empty() && !interp.enableTranslation) {
            continue;
        }
        RigExecWirePoseNumeric entry;
        entry.interpolator = uint32_t(i);
        entry.enableTranslation = interp.enableTranslation;
        entry.values = interp.valueInputs;
        numerics.push_back(std::move(entry));
    }
    if (!RigExecWireEncodePoseNumerics(numerics, &payload)) {
        return Fail("cannot encode the pose numerics");
    }
    writer.AddSection(RigExecBinarySection::PoseNumeric, payload);
    payload.clear();
    // Sparse in the same way: a rig with no space switch writes an empty
    // section, and a binary baked before minor 1 carries none at all.
    if (!RigExecWireEncodeSpaceSwitches(wirePose.spaceSwitches, &payload)) {
        return Fail("cannot encode the space switches");
    }
    writer.AddSection(RigExecBinarySection::SpaceSwitch, payload);
    payload.clear();
    // Written only when the rig has one, so every other rig's file is
    // unchanged.
    if (!wirePose.autoClavicles.empty()) {
        if (!RigExecWireEncodeAutoClavicles(wirePose.autoClavicles,
                                            &payload)) {
            return Fail("cannot encode the auto clavicles");
        }
        writer.AddSection(RigExecBinarySection::AutoClavicle, payload);
        payload.clear();
    }
    if (!wirePose.limbSolvers.empty()) {
        if (!RigExecWireEncodeLimbSolvers(wirePose.limbSolvers, &payload)) {
            return Fail("cannot encode the limb solvers");
        }
        writer.AddSection(RigExecBinarySection::LimbSolvers, payload);
        payload.clear();
    }
    const RigExecWireDomainGeometry wireGeometry =
        RigExecBakeConvertDomainGeometry(program, &writer);
    size_t revisionCount = 0;
    for (const RigExecWireChain &chain : wireGeometry.chains) {
        revisionCount += chain.revisions.size();
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
    // A poseable bake carries the property chains as programs, so the
    // runtime computes them from the inputs a client sets rather than
    // replaying their recorded values (rigExecBinary/propertyChains.h).
    RigExecWirePropertyChains propertyChains;
    std::vector<std::string> chainsSkipped;
    if (opts.overridableInputs) {
        std::string chainError;
        if (!RigExecBakePropertyChains(evaluator, opts.frames,
                                       capture.GetTable(), &writer,
                                       &propertyChains, &chainsSkipped,
                                       &chainError)) {
            return Fail("cannot bake the property chains: " + chainError);
        }
        if (!propertyChains.chains.empty()) {
            if (!RigExecWireEncodePropertyChains(propertyChains,
                                                 &payload)) {
                return Fail("cannot encode the property chains");
            }
            writer.AddSection(RigExecBinarySection::PropertyChains,
                              payload);
            payload.clear();
        }
    }
    std::string externalError;
    if (!external.Write(&writer, &externalError)) {
        return Fail(externalError);
    }
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
    manifest += "  \"inputs\": " +
                std::to_string(capture.GetTable().directory.size()) +
                ",\n";
    manifest += "  \"movers\": " +
                std::to_string(evaluator.GetMoverOrder().size()) + ",\n";
    if (!propertyChains.chains.empty()) {
        manifest += "  \"propertyChains\": " +
                    std::to_string(propertyChains.chains.size()) + ",\n";
        manifest += "  \"propertyChainsSkipped\": [";
        for (size_t i = 0; i < chainsSkipped.size(); ++i) {
            manifest += (i ? ", " : "") + _EscapeJson(chainsSkipped[i]);
        }
        manifest += "],\n";
    }
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
