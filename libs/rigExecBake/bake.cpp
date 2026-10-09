// .rigexec baking.
#include "rigExecBake/bake.h"
#include "rigExecBake/computedCapture.h"
#include "rigExecBake/pathTable.h"
#include "rigExecBake/revisionReads.h"
#include "rigExecBake/serialize.h"
#include "rigExecBake/staticCapture.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/movers/moverRegistry.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecBinary/format.h"
#include "rigExecBinary/generated/presentation_generated.h"

#include "pxr/base/vt/array.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>
#include <utility>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {
namespace {

// %.17g round-trips a double, the same canonical form rigExecPose --pose-out
// writes.
std::string
_FormatDouble(double value, char *buffer, size_t size)
{
    std::snprintf(buffer, size, "%.17g", value);
    return std::string(buffer);
}

// The plugin movers' half of the file: each plugin revision's epoch bytes
// and the frame bytes it encoded in the bake's run.
class _ExternalExport {
public:
    // Lists the program's plugin revisions after the run, when the program
    // certainly stands; refuses one whose plugin cannot encode.
    bool List(const RigExecBakedProgramImpl &program, std::string *error)
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
                _Entry entry;
                entry.chain = uint32_t(c);
                entry.revision = uint32_t(r);
                entry.handler = handler;
                _entries.push_back(std::move(entry));
            }
        }
        return true;
    }

    // Encodes every plugin revision's payload as the run just left it. A
    // revision with no payload -- its assembly failed -- holds no frame,
    // and playback fails it the same way.
    bool Capture(const RigExecBakedProgramImpl &program, double time,
                 std::string *error)
    {
        for (_Entry &entry : _entries) {
            const auto &revision =
                program.chains[entry.chain].revisions[entry.revision];
            const RigExecMoverParameters &parameters = revision.parameters;
            if(entry.handler->encodeExternalEpoch &&
               !entry.handler->encodeExternalEpoch(revision.binding,&entry.epoch)) {
                *error="external mover "+revision.moverPath.GetString()+" could not encode its immutable epoch";
                return false;
            }
            if (!parameters.valid || parameters.externalData.IsEmpty()) {
                continue;
            }
            char number[32];
            std::vector<uint8_t> sampledEpoch;
            auto *epoch = entry.handler->encodeExternalEpoch ? &sampledEpoch : &entry.epoch;
            if (!entry.handler->encodeExternal(parameters.externalData,
                                               revision.binding,
                                               epoch, &entry.frame)) {
                *error = "external mover " + revision.moverPath.GetString() +
                         " could not encode its payload at time " +
                         _FormatDouble(time, number, sizeof(number));
                return false;
            }
            entry.valid = true;
        }
        return true;
    }

    // The file's entries, in the same order: each plugin revision's type,
    // its epoch, the frame bytes of the bake's run (none when its assembly
    // failed there), and per declared phase the points authored at
    // \p time, 0 when nothing is read.
    void Fill(const RigExecBakedProgramImpl &program, double time,
              const RigExecBakeInputs &inputs,
              RigExecBakePathTable *paths, RigExecBakePools *pools,
              std::vector<fb::RigExecWireExternalMover> *out) const
    {
        out->clear();
        for (const _Entry &entry : _entries) {
            const auto &revision =
                program.chains[entry.chain].revisions[entry.revision];
            fb::RigExecWireExternalMover mover;
            mover.chain = entry.chain;
            mover.revision = entry.revision;
            mover.type = paths->Token(revision.moverPrim.GetTypeName());
            const auto declarations=inputs.externalInputs.find({entry.chain,entry.revision});
            if(declarations!=inputs.externalInputs.end()) {
                mover.declaredInputs=declarations->second.reads;
                for(size_t k=0;k<mover.declaredInputs.size();++k) {
                    auto &row=mover.declaredInputs[k];
                    const auto &fallback=declarations->second.fallbacks[k];
                    if(!RigExecFormatIsArrayTag(row.read->tag)) continue;
                    fb::RigExecWireValue value;
                    value.tag=row.read->tag;
                    if(fallback.IsHolding<VtIntArray>()) {
                        const auto &v=fallback.UncheckedGet<VtIntArray>(); value.array=pools->Ints(v.cdata(),v.size());
                    } else if(fallback.IsHolding<VtFloatArray>()) {
                        const auto &v=fallback.UncheckedGet<VtFloatArray>(); value.array=pools->Floats(v.cdata(),v.size());
                    } else if(fallback.IsHolding<VtDoubleArray>()) {
                        const auto &v=fallback.UncheckedGet<VtDoubleArray>(); value.array=pools->Doubles(v.cdata(),v.size());
                    } else if(fallback.IsHolding<VtVec2fArray>()) {
                        const auto &v=fallback.UncheckedGet<VtVec2fArray>();
                        value.array=pools->Vec2fs(v.empty()?nullptr:v.cdata()->data(),v.size());
                    } else if(fallback.IsHolding<VtVec3fArray>()) {
                        const auto &v=fallback.UncheckedGet<VtVec3fArray>();
                        value.array=pools->Vec3fs(v.empty()?nullptr:v.cdata()->data(),v.size());
                    }
                    row.read->constant=pools->Value(value);
                }
            }
            mover.v2FrameValid = entry.valid;
            mover.epoch = entry.epoch;
            if (entry.valid) {
                mover.v2Frame = entry.frame;
            }
            mover.phasedFallback.reserve(revision.binding.phases.size());
            for (const auto &phase : revision.binding.phases) {
                const UsdAttribute attribute =
                    program.stage->GetAttributeAtPath(phase.first);
                VtVec3fArray points;
                uint32_t id = 0;
                if (attribute && attribute.Get(&points, UsdTimeCode(time))) {
                    std::vector<float> xyz;
                    xyz.reserve(points.size() * 3);
                    for (const GfVec3f &p : points) {
                        xyz.push_back(p[0]);
                        xyz.push_back(p[1]);
                        xyz.push_back(p[2]);
                    }
                    id = pools->Vec3fs(xyz.data(), points.size());
                }
                mover.phasedFallback.push_back(id);
            }
            out->push_back(std::move(mover));
        }
    }

private:
    struct _Entry {
        uint32_t chain = 0;
        uint32_t revision = 0;
        const RigExecMoverHandler *handler = nullptr;
        std::vector<uint8_t> epoch;
        std::vector<uint8_t> frame;
        bool valid = false;
    };
    std::vector<_Entry> _entries;
};

// Adds to \p paths the input each control of \p bytes names, when \p bytes
// is a REXP buffer that verifies; adds nothing otherwise, and
// _VerifyPresentation reports that buffer after the capture.
void
_PresentationInputPaths(const std::vector<uint8_t> &bytes,
                        std::set<SdfPath> *paths)
{
    if (bytes.empty() || bytes.size() >= FLATBUFFERS_MAX_BUFFER_SIZE) {
        return;
    }
    flatbuffers::Verifier verifier(bytes.data(), bytes.size());
    if (!fb::VerifyPresentationBuffer(verifier)) {
        return;
    }
    const fb::Presentation *presentation = fb::GetPresentation(bytes.data());
    if (!presentation->controls()) {
        return;
    }
    for (const fb::PresentationControl *control :
         *presentation->controls()) {
        // A name that is no path is no listed input either, which
        // _VerifyPresentation reports.
        const std::string input = control->input()->str();
        if (SdfPath::IsValidPathString(input)) {
            paths->insert(SdfPath(input));
        }
    }
}

// A REXP buffer that verifies, every control naming one of \p listed
// (ascending). The bytes are embedded unchanged; nothing here reads more of
// them than the controls' inputs.
bool
_VerifyPresentation(const std::vector<uint8_t> &bytes,
                    const std::vector<std::string> &listed,
                    std::string *error)
{
    if (bytes.size() >= FLATBUFFERS_MAX_BUFFER_SIZE) {
        *error = "the presentation is past the FlatBuffers size limit";
        return false;
    }
    flatbuffers::Verifier verifier(bytes.data(), bytes.size());
    if (!fb::VerifyPresentationBuffer(verifier)) {
        *error = "the presentation is not a valid REXP buffer";
        return false;
    }
    const fb::Presentation *presentation = fb::GetPresentation(bytes.data());
    if (!presentation->controls()) {
        return true;
    }
    for (const fb::PresentationControl *control :
         *presentation->controls()) {
        // Both fields are required, so the verifier saw them present.
        const std::string input = control->input()->str();
        if (!std::binary_search(listed.begin(), listed.end(), input)) {
            *error = "presentation control " + control->name()->str() +
                     " names " + input + ", which is not a listed input";
            return false;
        }
    }
    return true;
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
    if (std::isinf(opts.time)) {
        return Fail("the bake time must be finite");
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
    // A bake is of the authored epoch: a held drag would print its value
    // into the defaults.
    if (evaluator.HasInteractiveOverrides()) {
        return Fail("cannot bake with interactive overrides standing");
    }
    const std::vector<SdfPath> upstream = evaluator.GetUpstreamInputPaths();
    // The file's program is the export build: a Range skin or a group gate
    // rests only on reads the file holds as constants (private slots), never
    // on one it must leave listed -- an admitted upstream input or an input
    // the presentation names. The guard restores the evaluator's role mode
    // on every return, and its next Evaluate rebuilds.
    std::set<SdfPath> keep(upstream.begin(), upstream.end());
    _PresentationInputPaths(opts.presentation, &keep);
    RigExecScopedBakedRoleMode roles(evaluator, RigExecBakedRoleMode::Export,
                                     std::move(keep));
    RigExecScopedUpstreamSuspension suspension(evaluator);
    const RigExecBakedProgram *compiled = evaluator.GetBakedProgram();
    if (!compiled) {
        return Fail("no baked program standing to capture from");
    }
    const double bakeTime =
        std::isnan(opts.time)
            ? RigExecBakedProbeTime(compiled->GetStepGraph().stage)
                  .GetValue()
            : opts.time;
    // One generation at the bake time builds the export program, with no
    // interactive override standing and upstream lifted, so its roles read
    // the stage; the full run below is of that program.
    evaluator.Evaluate(UsdTimeCode(bakeTime));
    const RigExecBakedProgram *standing = evaluator.GetBakedProgram();
    if (!standing) {
        return Fail("no baked program standing to capture from");
    }
    char number[32];
    std::string why;
    const size_t bakedBefore = evaluator.GetBakedGenerationCount();
    // The one run reads every step's inputs, so the static data it leaves
    // behind is complete whatever the closure would have skipped.
    standing->RequestFullRun();
    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(bakeTime));
    if (evaluator.GetBakedProgram() != standing) {
        return Fail("the program rebuilt during the bake run at time " +
                    _FormatDouble(bakeTime, number, sizeof(number)));
    }
    if (!pose.valid) {
        return Fail("invalid generation at time " +
                    _FormatDouble(bakeTime, number, sizeof(number)));
    }
    _ExternalExport external;
    if (!external.List(standing->GetStepGraph(), &why) ||
        !external.Capture(standing->GetStepGraph(), bakeTime, &why)) {
        return Fail(why);
    }
    // The IsBakeable gate above is necessary but not sufficient: an
    // in-epoch refusal the reasons do not name -- an override the program
    // cannot place, or a Run that handed the generation back -- still
    // answers dynamically. The generation count is what catches those, the
    // same accounting rigExecPose --require-baked performs.
    if (evaluator.GetBakedGenerationCount() - bakedBefore != 1) {
        return Fail("the bake run did not come from the baked program");
    }
    const RigExecBakedProgramImpl &program = standing->GetStepGraph();

    // The input list, from the standing program: the slots every read
    // takes, with their values at the bake time as the defaults, every read
    // in its table field, and the facts the oracle needs. Facts of an
    // animated attribute hold their value at the bake time, which
    // RigExecBakeStaticReport names. Paths and tokens intern in the order
    // this visits them, then the tables', so two bakes write the same ids.
    RigExecBakePathTable paths;
    RigExecBakeComputedCapture inputs(evaluator, bakeTime, &paths, &why);
    if (!inputs.Valid()) {
        return Fail(why);
    }
    const auto &listed = inputs.GetListedInputNames();
    std::vector<std::string> upstreamNames;
    upstreamNames.reserve(upstream.size());
    for (const SdfPath &path : upstream) {
        const std::string name = path.GetString();
        if (!std::binary_search(listed.begin(), listed.end(), name)) {
            return Fail("upstream input " + name +
                        " is admitted but has no input slot");
        }
        upstreamNames.push_back(name);
    }
    std::sort(upstreamNames.begin(), upstreamNames.end());
    if (!opts.presentation.empty() &&
        !_VerifyPresentation(opts.presentation, inputs.GetListedInputNames(),
                             &why)) {
        return Fail(why);
    }

    // The file of the program and its run: the tables, the static data the
    // run left, the plugin movers, then the root's fields. The path reads
    // are every key the assembly's enumeration lists, so an input set on
    // the file that takes another branch reads the stage's data there. It
    // is written only if it validates, and kept only if it opens again.
    std::vector<RigExecBakeRevisionRead> enumerated;
    RigExecBakeEnumerateProgramReads(program, bakeTime, &enumerated);
    fb::RigExecWireFile file;
    RigExecBakePools pools;
    if (!RigExecBakeFillFile(program, inputs.GetInputs(), &paths, &pools,
                             &file, &why) ||
        !RigExecBakeCaptureStatics(program, bakeTime, inputs.GetInputs(),
                                   enumerated, &paths, &pools, &file, &why)) {
        return Fail("cannot build the .rigexec file: " + why);
    }
    external.Fill(program, bakeTime, inputs.GetInputs(), &paths, &pools, &file.externalMovers);
    file.formatVersion = RigExecFormatVersion;
    file.rig = paths.Path(evaluator.GetRigPath());
    file.bakeTime = bakeTime;
    file.compileDiagnostics = compileErrors;
    file.presentation = opts.presentation;
    paths.MoveInto(&file);
    pools.MoveInto(&file);
    std::vector<uint8_t> bytes;
    if (!RigExecFormatWrite(file, &bytes, &why)) {
        return Fail("cannot write the .rigexec file: " + why);
    }
    {
        std::unique_ptr<fb::RigExecWireFile> reopened;
        if (!RigExecFormatOpen(bytes.data(), bytes.size(), &reopened, &why)) {
            return Fail("the written .rigexec file does not open: " + why);
        }
    }
    std::set<std::pair<SdfPath, bool>> keys;
    for (const RigExecBakeRevisionRead &read : enumerated) {
        keys.emplace(read.path, read.rest);
    }
    result->bytes = std::move(bytes);
    result->upstreamInputs = std::move(upstreamNames);
    result->pathReadsWritten = file.geometry->pathReads.size();
    result->pathReadsEnumerated = keys.size();
    return true;
}

}  // namespace rigExec
