// Typed phased point and frame reads bind to explicit graph versions.
// Exact captured reader values and producer metadata check binding conformance.
// CPU scalar-reference agreement and literal checkpoint arithmetic provide
// independent numeric checks; no retained legacy walk store is used as judge.
// Held runs retain prior answers without executing clean reader bodies.
//
// argv[1] = path to the examples directory.
#include "rigExec/inputReplay.h"
#include "rigExecFrameRecordCheck.h"

#include "rigExec/bakedProgramImpl.h"
#include "rigExec/bakedSchedule.h"
#include "rigExec/bakedTrace.h"
#include "rigExec/rigEvaluator.h"
#include "rigExec/scalarReferenceAdapter.h"

#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/sdf/primSpec.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <cstdio>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE

static int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

#define CHECK_AT(where, cond)                                              \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d [%s]: %s\n", __FILE__, __LINE__,       \
                        std::string(where).c_str(), #cond);                \
        }                                                                  \
    } while (0)

namespace {

const SdfPath kReadPhaseRig("/ReadPhaseAsset/Rig");
const SdfPath kSlab("/ReadPhaseAsset/Geom/Slab.points");
const SdfPath kCage("/ReadPhaseAsset/Geom/Cage.points");
const SdfPath kOther("/ReadPhaseAsset/Geom/Other.points");
const SdfPath kSlabLattice("/ReadPhaseAsset/Rig/Movers/Geometry/SlabLattice");
const SdfPath kLiftCtl("/ReadPhaseAsset/Rig/Controls/LiftCtl");
const TfToken kReadPhase("rigExecReadPhase");

/// One binding of the program, with the reader it belongs to.
struct BindingRef {
    const RigExecBakedPointsBinding *binding = nullptr;
    SdfPath reader;
    SdfPath readerTarget;
    size_t readerChain = 0;
    bool derived = false;
    bool sample = false;
    /// The step that reads it: the revision's RevisionStatic, or the derived
    /// target's Derived step.
    int step = -1;
};

std::vector<BindingRef>
Bindings(const RigExecBakedProgramImpl &B)
{
    std::vector<BindingRef> out;
    const auto add = [&out](const RigExecBakedProgramImpl::GeomRevision &r,
                            const SdfPath &target, size_t chain,
                            bool derived, int step) {
        for (const RigExecBakedPointsBinding &binding : r.pointBindings) {
            out.push_back({&binding, r.moverPath, target, chain, derived,
                           false, step});
        }
        for (const auto &channel : r.blendChannels) {
            for (const auto &sample : channel.samples) {
                if (sample.pointBinding.id >= 0) {
                    out.push_back({&sample.pointBinding, r.moverPath, target,
                                   chain, derived, true, step});
                }
            }
        }
    };
    const auto readerStep = [&B](RigExecBakedStepKind kind, size_t chain,
                                 size_t index) {
        for (size_t s = 0; s < B.steps.size(); ++s) {
            const RigExecBakedStep &step = B.steps[s];
            if (step.kind != kind || step.object < 0) {
                continue;
            }
            const auto [c, i] =
                kind == RigExecBakedStepKind::Derived
                    ? B.derivedIndex[size_t(step.object)]
                    : B.revisionIndex[size_t(step.object)];
            if (size_t(c) == chain && size_t(i) == index) {
                return int(s);
            }
        }
        return -1;
    };
    for (size_t c = 0; c < B.chains.size(); ++c) {
        for (size_t r = 0; r < B.chains[c].revisions.size(); ++r) {
            add(B.chains[c].revisions[r], B.chains[c].target, c, false,
                readerStep(RigExecBakedStepKind::RevisionStatic, c, r));
        }
        for (size_t d = 0; d < B.chains[c].derived.size(); ++d) {
            const auto &derived = B.chains[c].derived[d];
            add(derived.revision, derived.target, c, true,
                readerStep(RigExecBakedStepKind::Derived, c, d));
        }
    }
    return out;
}

const BindingRef *
Find(const std::vector<BindingRef> &refs, const SdfPath &reader,
     const SdfPath &readerTarget, const SdfPath &input)
{
    for (const BindingRef &ref : refs) {
        if (ref.reader == reader && ref.readerTarget == readerTarget &&
            ref.binding->input == input) {
            return &ref;
        }
    }
    return nullptr;
}

std::string
MissLine(const BindingRef &ref)
{
    return "diag " + ref.reader.GetString() + ": read phase '" +
           ref.binding->phase.GetAsString() + "' for " +
           ref.binding->input.GetString() +
           " resolved to nothing; read the authored base";
}

bool
Mentions(const std::vector<std::string> &lines, const std::string &line)
{
    return std::find(lines.begin(), lines.end(), line) != lines.end();
}

/// What one generation's captures said, summed over its bindings.
struct Tally {
    size_t read = 0;
    size_t answered = 0;
    size_t tails = 0;
};

/// Holds every binding of \p evaluator's program: the answer captured inside
/// the reader (when \p captured) to the binding's resolution after the run
/// -- nothing after a reader writes a version it names -- and to the
/// reader's diagnostic; and, for a read of an earlier chain, complete before
/// any reader of a later one runs, the binding to the store the dynamic
/// walk of this cpuReference generation left in the evaluator.
Tally
CheckBindings(const std::string &where, const RigExecRigEvaluator &evaluator,
              const RigExecRigPose &pose, bool captured)
{
    Tally tally;
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    CHECK_AT(where, program != nullptr);
    if (!program) {
        return tally;
    }
    const RigExecBakedProgramImpl &B = program->GetStepGraph();
    CHECK_AT(where, evaluator.cpuReference ==
                        true);
    CHECK_AT(where, pose.referenceMismatches == 0);
    CHECK_AT(where, pose.referenceAgreements > 0);
    std::map<SdfPath, size_t> chainOf;
    for (size_t c = 0; c < B.chains.size(); ++c) {
        chainOf.emplace(B.chains[c].target, c);
    }
    for (const BindingRef &ref : Bindings(B)) {
        const RigExecBakedPointsBinding &binding = *ref.binding;
        const std::string at = where + " " + ref.reader.GetString() + " on " +
                               ref.readerTarget.GetString() + " reading " +
                               binding.input.GetString() + " at " +
                               binding.phase.GetAsString();
        CHECK_AT(at, ref.step >= 0 && size_t(ref.step) < B.steps.size());
        if (ref.step < 0 || size_t(ref.step) >= B.steps.size()) continue;
        const auto source = chainOf.find(binding.input);
        if (rigExecOriginalQuery::configuration.check && source != chainOf.end() &&
            source->second < ref.readerChain) {
            const GfVec3f *queryPoints = nullptr;
            size_t queryCount = 0;
            const bool queryAnswered = RigExecBakedResolvePoints(B,binding,&queryPoints,&queryCount);
            const VtValue queryValue = queryAnswered
                ? VtValue(queryCount ? VtVec3fArray(queryPoints,queryPoints+queryCount) : VtVec3fArray())
                : VtValue();
            rigExecOriginalQuery::Emit(where,ref.reader.GetString(),binding.input.GetString(),
                binding.phase.GetAsString(),pose.time.IsDefault(),pose.time.GetValue(),
                queryAnswered ? &queryValue : nullptr);
        }
        if (captured) {
            CHECK_AT(at, size_t(binding.id) < B.pointCaptures.size());
            if (size_t(binding.id) >= B.pointCaptures.size()) {
                continue;
            }
            const RigExecBakedPointCapture &capture =
                B.pointCaptures[size_t(binding.id)];
            if (!capture.read) {
                // Only a reader whose own chain read no base sits a run out.
                CHECK_AT(at, (size_t(ref.step) >= B.opExecution.ran.size() || !B.opExecution.ran[size_t(ref.step)]) ||
                             !B.chains[ref.readerChain].haveBase);
            } else {
                CHECK_AT(at, (size_t(ref.step) < B.opExecution.ran.size() && B.opExecution.ran[size_t(ref.step)]));
                ++tally.read;
                const GfVec3f *points = nullptr;
                size_t count = 0;
                const bool answered =
                    RigExecBakedResolvePoints(B, binding, &points, &count);
                CHECK_AT(at, capture.bindingAnswered == answered);
                CHECK_AT(at, capture.bound ==
                                 (answered ? VtVec3fArray(points,
                                                          points + count)
                                           : VtVec3fArray()));
                if (capture.bindingAnswered) {
                    ++tally.answered;
                } else {
                    ++tally.tails;
                }
                if (binding.diagnoseMiss) {
                    CHECK_AT(at, Mentions(pose.diagnostics, MissLine(ref)) ==
                                     !capture.bindingAnswered);
                } else {
                    CHECK_AT(at, !Mentions(pose.diagnostics, MissLine(ref)));
                }
            }
        }
    }
    return tally;
}

std::unique_ptr<RigExecRigEvaluator>
MakeEvaluator(const UsdStageRefPtr &stage, const SdfPath &rig,
              bool referenceChecks)
{
    auto evaluator = std::make_unique<RigExecRigEvaluator>(stage, rig);
    evaluator->cpuReference = true; // Both capture styles retain the independent oracle.
    std::vector<std::string> errors;
    const bool compiled = evaluator->Compile(&errors);
    for (const std::string &error : errors) {
        std::printf("compile error: %s\n", error.c_str());
    }
    CHECK(compiled);
    return compiled ? std::move(evaluator) : nullptr;
}

/// One baked generation with every binding's answer captured inside its
/// reader.
RigExecRigPose
EvaluateCaptured(const std::string &where, RigExecRigEvaluator *evaluator,
                 UsdTimeCode time, Tally *tally)
{
    bool captured = false;
    if (const RigExecBakedProgram *program = evaluator->GetBakedProgram()) {
        captured = RigExecBakedProgramTesting::CapturePointReads(*program);
    }
    const size_t generations = evaluator->GetBakedGenerationCount();
    const RigExecRigPose pose = evaluator->Evaluate(time);
    CHECK_AT(where, pose.valid);
    CHECK_AT(where, pose.referenceMismatches == 0);
    CHECK_AT(where, evaluator->GetBakedGenerationCount() == generations + 1);
    if (pose.referenceMismatches != 0) {
        for (const std::string &line : pose.diagnostics) {
            std::printf("  %s\n", line.c_str());
        }
    }
    // The capture is serial-only; the parallel registrations still hold
    // the end-of-run reads to the typed version metadata and the parity comparison.
    if (captured && !evaluator->GetBakedProgram()) {
        captured = false;
    }
    const Tally one = CheckBindings(where, *evaluator, pose, captured);
    tally->read += one.read;
    tally->answered += one.answered;
    tally->tails += one.tails;
    return pose;
}

bool
SerialSchedule()
{
    return RigExecBakedScheduleModeFromEnvironment() ==
           RigExecBakedScheduleMode::Serial;
}

VtVec3fArray
Moved(const RigExecRigPose &pose, const SdfPath &target)
{
    const auto it = pose.movedProperties.find(target);
    CHECK(it != pose.movedProperties.end());
    if (it == pose.movedProperties.end() ||
        !it->second.IsHolding<VtVec3fArray>()) {
        return VtVec3fArray();
    }
    return it->second.UncheckedGet<VtVec3fArray>();
}

UsdStageRefPtr
OpenReadPhases(const std::string &examples)
{
    const UsdStageRefPtr stage =
        UsdStage::Open(examples + "/13_ReadPhases.usda");
    CHECK(stage);
    if (stage) {
        // Test-time edits go to the session layer, so the shared root layer
        // stays as authored for the next case.
        stage->SetEditTarget(stage->GetSessionLayer());
    }
    return stage;
}

void
SetPhase(const UsdStageRefPtr &stage, const SdfPath &mover,
         const char *relationship, const std::string &phase)
{
    const UsdRelationship rel =
        stage->GetPrimAtPath(mover).GetRelationship(TfToken(relationship));
    CHECK(rel && rel.SetMetadata(kReadPhase, VtValue(phase)));
}

/// A lattice under Movers/Geometry over the cage, moving \p targets.
void
DefineLattice(const UsdStageRefPtr &stage, const char *name,
              const SdfPathVector &targets, const std::string &phase)
{
    const SdfPath path = SdfPath("/ReadPhaseAsset/Rig/Movers/Geometry")
                             .AppendChild(TfToken(name));
    const UsdPrim lattice =
        stage->DefinePrim(path, TfToken("RigExecLatticeMover"));
    CHECK(lattice.ApplyAPI(TfToken("RigExecMoverAPI")));
    lattice.CreateAttribute(TfToken("rigExec:basis"), SdfValueTypeNames->Token,
                            false, SdfVariabilityUniform)
        .Set(TfToken("bernstein"));
    lattice.CreateAttribute(TfToken("rigExec:divisions"),
                            SdfValueTypeNames->Int3)
        .Set(GfVec3i(2, 2, 2));
    const UsdRelationship cage =
        lattice.CreateRelationship(TfToken("rigExec:cage"), false);
    CHECK(cage.SetTargets({kCage.GetPrimPath()}));
    CHECK(cage.SetMetadata(kReadPhase, VtValue(phase)));
    CHECK(lattice.CreateRelationship(TfToken("rigExec:moves"), false)
              .SetTargets(targets));
}

/// A half-size copy of the slab's points at \p target, as a Points prim.
void
DefinePoints(const UsdStageRefPtr &stage, const SdfPath &target)
{
    const UsdPrim prim =
        stage->DefinePrim(target.GetPrimPath(), TfToken("Points"));
    VtVec3fArray points;
    CHECK(stage->GetAttributeAtPath(kSlab).Get(&points));
    for (GfVec3f &p : points) {
        p *= 0.5f;
    }
    CHECK(prim.CreateAttribute(TfToken("points"),
                               SdfValueTypeNames->Point3fArray)
              .Set(points));
}

/// A surface mover under Movers/Geometry projecting \p targets onto the
/// slab, read at \p phase. Unlike a lattice it may move several targets.
void
DefineProjector(const UsdStageRefPtr &stage, const char *name,
                const SdfPathVector &targets, const std::string &phase)
{
    const SdfPath path = SdfPath("/ReadPhaseAsset/Rig/Movers/Geometry")
                             .AppendChild(TfToken(name));
    const UsdPrim mover =
        stage->DefinePrim(path, TfToken("RigExecSurfaceMover"));
    CHECK(mover.ApplyAPI(TfToken("RigExecMoverAPI")));
    mover.CreateAttribute(TfToken("rigExec:mode"), SdfValueTypeNames->Token,
                          false, SdfVariabilityUniform)
        .Set(TfToken("project"));
    const UsdRelationship surface =
        mover.CreateRelationship(TfToken("rigExec:surface"), false);
    CHECK(surface.SetTargets({kSlab.GetPrimPath()}));
    CHECK(surface.SetMetadata(kReadPhase, VtValue(phase)));
    CHECK(mover.CreateRelationship(TfToken("rigExec:moves"), false)
              .SetTargets(targets));
}

const std::vector<UsdTimeCode> kFrames{UsdTimeCode(1001), UsdTimeCode(1003),
                                       UsdTimeCode(1007)};

/// The scalar reference under the scalar oracle (cpuReference), which reads
/// every phase itself and refuses to publish a generation it disagrees
/// with: each of \p times publishes, with no mismatch and at least one
/// point chain compared.
std::vector<RigExecRigPose>
EvaluateUnderTheOracle(const std::string &where, const UsdStageRefPtr &stage,
                       const SdfPath &rig,
                       const std::vector<UsdTimeCode> &times)
{
    std::vector<RigExecRigPose> poses;
    auto oracle = MakeEvaluator(stage, rig, true);
    if (!oracle) {
        return poses;
    }
    oracle->cpuReference = true;
    for (const UsdTimeCode &time : times) {
        const std::string at =
            where + " oracle at " + std::to_string(time.GetValue());
        RigExecRigPose pose = oracle->Evaluate(time);
        for (const std::string &line : pose.diagnostics) {
            if (!pose.valid && line.find("parity") != std::string::npos) {
                std::printf("  %s: %s\n", at.c_str(), line.c_str());
            }
        }
        CHECK_AT(at, pose.valid);
        CHECK_AT(at, pose.referenceMismatches == 0);
        CHECK_AT(at, pose.referenceAgreements > 0);
        poses.push_back(std::move(pose));
    }
    return poses;
}

/// The frames, then a drag on the cage's lift control at the last one.
Tally
RunFramesAndDrag(const std::string &where, RigExecRigEvaluator *evaluator)
{
    Tally tally;
    for (const UsdTimeCode &time : kFrames) {
        EvaluateCaptured(where + " at " + std::to_string(time.GetValue()),
                         evaluator, time, &tally);
    }
    evaluator->SetInteractiveOverrides({RigExecValueOverride{
        kLiftCtl, TfToken(), TfToken("avars:ty"), VtValue(1.25)}});
    EvaluateCaptured(where + " dragged", evaluator, kFrames.back(), &tally);
    evaluator->ClearInteractiveOverrides();
    EvaluateCaptured(where + " released", evaluator, kFrames.back(), &tally);
    return tally;
}

/// SlabLattice's RevisionStatic declares the cage version its binding names
/// and no read of the store, and so follows that version's only writer: the
/// cage chain's ChainStatus for `final`, and otherwise the fuse of the
/// named revision, which lies under \p phasePrim. Held structurally because
/// nothing at run time would catch a missing edge: the serial schedule runs
/// in program order whatever is declared, and the parallel one fails only
/// when a race happens to land.
void
CheckSlabAssembleEdges(const std::string &where,
                       const RigExecBakedProgramImpl &B,
                       const RigExecBakedPointsBinding &binding,
                       const SdfPath &phasePrim)
{
    int slabChain = -1;
    int cageChain = -1;
    for (size_t c = 0; c < B.chains.size(); ++c) {
        if (B.chains[c].target == kSlab) {
            slabChain = int(c);
        } else if (B.chains[c].target == kCage) {
            cageChain = int(c);
        }
    }
    int slabRevision = -1;
    for (size_t id = 0; id < B.revisionIndex.size(); ++id) {
        const auto [c, r] = B.revisionIndex[id];
        if (c == slabChain &&
            B.chains[size_t(c)].revisions[size_t(r)].moverPath ==
                kSlabLattice) {
            slabRevision = int(id);
        }
    }
    int assemble = -1;
    int status = -1;
    for (size_t s = 0; s < B.steps.size(); ++s) {
        const RigExecBakedStep &step = B.steps[s];
        if (step.kind == RigExecBakedStepKind::RevisionStatic &&
            step.object == slabRevision) {
            assemble = int(s);
        } else if (step.kind == RigExecBakedStepKind::ChainStatus &&
                   step.object == cageChain) {
            status = int(s);
        }
    }
    CHECK_AT(where, slabChain >= 0 && cageChain >= 0 && slabRevision >= 0);
    CHECK_AT(where, assemble >= 0 && status >= 0);
    const bool cageScope = phasePrim == SdfPath("/ReadPhaseAsset/Rig/Movers/Cage");
    const size_t expectedCandidates = cageScope ? 2 : 1;
    CHECK_AT(where, binding.candidates.size() == expectedCandidates);
    if (assemble < 0 || status < 0 || binding.candidates.size() != expectedCandidates) {
        return;
    }
    const RigExecBakedStep &step = B.steps[size_t(assemble)];
    const auto reads = [&step](RigExecBakedSlotDomain domain, int slot) {
        for (const RigExecBakedSlotRange &range : step.reads) {
            if (range.domain == domain && range.begin <= uint32_t(slot) &&
                uint32_t(slot) < range.end) {
                return true;
            }
        }
        return false;
    };
    const auto follows = [&step](int producer) {
        return std::binary_search(step.preds.begin(), step.preds.end(),
                                  producer);
    };
    for (const RigExecBakedSlotRange &range : step.reads) {
        CHECK_AT(where, range.domain != RigExecBakedSlotDomain::Snapshots);
    }
    for (size_t k = 0; k < binding.candidates.size(); ++k) {
        const RigExecBakedPointVersion &candidate = binding.candidates[k];
        CHECK_AT(where, candidate.chain == cageChain);
        CHECK_AT(where, candidate.version == (binding.finalRead || cageScope ? 2 - int(k) : 1));
        if (binding.finalRead) {
            CHECK_AT(where, reads(RigExecBakedSlotDomain::ChainPoints, cageChain));
            CHECK_AT(where, follows(status));
            return;
        }
        CHECK_AT(where, candidate.version > 0);
        if (candidate.version <= 0) {
            return;
        }
        const int revision =
            B.chainRevisionBegin[size_t(cageChain)] + candidate.version - 1;
        if (!phasePrim.IsEmpty()) {
            const auto [c, r] = B.revisionIndex[size_t(revision)];
            CHECK_AT(where, B.chains[size_t(c)]
                                .revisions[size_t(r)]
                                .moverPath.HasPrefix(phasePrim));
        }
        CHECK_AT(where, reads(RigExecBakedSlotDomain::RevisionDone, revision));
        CHECK_AT(where, reads(RigExecBakedSlotDomain::ChainDirty, revision));
        CHECK_AT(where, follows(B.revisionFuseStep[size_t(revision)]));
    }
}

// 13_ReadPhases as shipped (the lattice reads the cage at `final`), and with
// the lattice's cage phase on the first cage mover and on the cage's Scope:
// cross-chain AtPrim, the form a Scope abbreviates.
void
TestPointBindingsCaptureTypedVersions(const std::string &examples)
{
    for (const char *phase :
         {"", "/ReadPhaseAsset/Rig/Movers/Cage/CageLift",
          "/ReadPhaseAsset/Rig/Movers/Cage"}) {
        const UsdStageRefPtr stage = OpenReadPhases(examples);
        if (!stage) {
            return;
        }
        if (*phase) {
            SetPhase(stage, kSlabLattice, "rigExec:cage", phase);
        }
        const std::string where =
            std::string("13 cage at ") + (*phase ? phase : "final");
        auto evaluator = MakeEvaluator(
            stage, kReadPhaseRig, true);
        if (!evaluator) {
            return;
        }
        const Tally tally = RunFramesAndDrag(where, evaluator.get());
        if (!evaluator->GetBakedProgram()) {
            CHECK_AT(where, !"the rig bakes");
            continue;
        }
        if (SerialSchedule()) {
            CHECK_AT(where, tally.read == 5);
            CHECK_AT(where, tally.answered == 5);
        }
        const std::vector<BindingRef> refs =
            Bindings(evaluator->GetBakedProgram()->GetStepGraph());
        const BindingRef *slab =
            Find(refs, kSlabLattice, kSlab, kCage);
        CHECK_AT(where, slab != nullptr);
        if (slab) {
            const size_t expectedCandidates =
                std::string(phase) == "/ReadPhaseAsset/Rig/Movers/Cage" ? 2 : 1;
            CHECK_AT(where, slab->binding->candidates.size() == expectedCandidates);
            CHECK_AT(where, slab->binding->finalRead == !*phase);
            CHECK_AT(where, slab->binding->diagnoseMiss);
            CheckSlabAssembleEdges(
                where, evaluator->GetBakedProgram()->GetStepGraph(),
                *slab->binding, *phase ? SdfPath(phase) : SdfPath());
        }
    }
}

// A mover that writes two chains reads the other one at `preceding`.
// Plan3.2 binds the last authored producer below the reader independently
// of whether a later reader names a checkpoint. On the reader's own chain
// it likewise reads the entering version. Explicit Base and AtPrim controls
// retain phase sensitivity and independent scalar-kernel agreement.
//
// Dual projects Other and the slab itself onto the slab at `preceding`, after
// SlabLattice on the slab's chain; in the named variant Probe projects Third
// onto the slab as of Dual.
void
TestPrecedingAcrossChains(const std::string &examples)
{
    const SdfPath geometry("/ReadPhaseAsset/Rig/Movers/Geometry");
    const SdfPath dual = geometry.AppendChild(TfToken("Dual"));
    const SdfPath third("/ReadPhaseAsset/Geom/Third.points");
    std::vector<VtVec3fArray> others;
    for (const bool named : {false, true}) {
        const UsdStageRefPtr stage = OpenReadPhases(examples);
        if (!stage) {
            return;
        }
        DefinePoints(stage, kOther);
        DefineProjector(stage, "Dual", {kOther, kSlab}, "preceding");
        if (named) {
            DefinePoints(stage, third);
            DefineProjector(stage, "Probe", {third}, dual.GetString());
        }
        // Walked last child first: SlabLattice, Dual, then Probe.
        const SdfPrimSpecHandle spec =
            stage->GetSessionLayer()->GetPrimAtPath(geometry);
        CHECK(spec);
        if (spec) {
            spec->SetNameChildrenOrder(
                {TfToken("Probe"), TfToken("Dual"), TfToken("SlabLattice")});
        }
        const std::string where =
            std::string("preceding across chains, ") +
            (named ? "named" : "unnamed");
        auto evaluator = MakeEvaluator(
            stage, kReadPhaseRig, true);
        if (!evaluator) {
            return;
        }
        Tally tally;
        RigExecRigPose pose;
        for (const UsdTimeCode &time : kFrames) {
            pose = EvaluateCaptured(
                where + " at " + std::to_string(time.GetValue()),
                evaluator.get(), time, &tally);
        }
        others.push_back(Moved(pose, kOther));
        const std::vector<RigExecRigPose> checked =
            EvaluateUnderTheOracle(where, stage, kReadPhaseRig, kFrames);
        if (checked.size() == kFrames.size() && checked.back().valid) {
            CHECK_AT(where, Moved(checked.back(), kOther) == others.back());
            CHECK_AT(where, Moved(checked.back(), kSlab) == Moved(pose, kSlab));
        }
        if (!evaluator->GetBakedProgram()) {
            CHECK_AT(where, !"the rig bakes");
            continue;
        }
        const std::vector<BindingRef> refs =
            Bindings(evaluator->GetBakedProgram()->GetStepGraph());
        const BindingRef *across = Find(refs, dual, kOther, kSlab);
        const BindingRef *own = Find(refs, dual, kSlab, kSlab);
        CHECK_AT(where, across && own);
        if (!across || !own) {
            continue;
        }
        CHECK_AT(where, !across->binding->diagnoseMiss);
        CHECK_AT(where, !own->binding->diagnoseMiss);
        // On the slab's chain Dual reads the slab entering it: version 1,
        // after SlabLattice.
        CHECK_AT(where, own->binding->candidates.size() == 1);
        if (own->binding->candidates.size() == 1) {
            CHECK_AT(where, own->binding->candidates[0].chain ==
                                int(own->readerChain));
            CHECK_AT(where, own->binding->candidates[0].version == 1);
        }
        // Plan3.2 selects the last authored writer below Dual regardless
        // of whether a later Probe additionally names Dual's checkpoint.
        CHECK_AT(where, across->binding->candidates.size() == 1);
        if (across->binding->candidates.size() == 1) {
            CHECK_AT(where, across->binding->candidates[0].chain ==
                                int(own->readerChain));
            CHECK_AT(where, across->binding->candidates[0].version == 1);
        }
        if (SerialSchedule()) {
            const RigExecBakedProgramImpl &B =
                evaluator->GetBakedProgram()->GetStepGraph();
            const RigExecBakedPointCapture &a =
                B.pointCaptures[size_t(across->binding->id)];
            const RigExecBakedPointCapture &o =
                B.pointCaptures[size_t(own->binding->id)];
            CHECK_AT(where, a.read && a.bindingAnswered);
            CHECK_AT(where, o.read && o.bindingAnswered);
        }
    }
    // A naming-only change cannot change this positional version. Verify
    // it against an explicitly selected SlabLattice checkpoint, and retain
    // phase sensitivity against the authored Base surface.
    CHECK(others.size() == 2 && others[0] == others[1]);
    std::map<std::string, VtVec3fArray> controls;
    for (const std::string phase : {std::string("base"),
                                   kSlabLattice.GetString()}) {
        const auto stage = OpenReadPhases(examples);
        CHECK(stage);
        if (!stage) continue;
        DefinePoints(stage, kOther);
        DefineProjector(stage, "Dual", {kOther, kSlab}, phase);
        const auto spec = stage->GetSessionLayer()->GetPrimAtPath(geometry);
        CHECK(spec);
        if (spec) spec->SetNameChildrenOrder(
            {TfToken("Probe"), TfToken("Dual"), TfToken("SlabLattice")});
        VtVec3fArray authoredSurface;
        CHECK(stage->GetAttributeAtPath(kSlab).Get(&authoredSurface));
        auto evaluator = MakeEvaluator(stage, kReadPhaseRig, true);
        CHECK(evaluator);
        if (!evaluator) continue;
        const std::string where = "preceding control " + phase;
        Tally tally;
        RigExecRigPose pose;
        for (const auto time : kFrames)
            pose = EvaluateCaptured(where, evaluator.get(), time, &tally);
        controls[phase] = Moved(pose, kOther);
        const auto checked = EvaluateUnderTheOracle(where, stage, kReadPhaseRig,
                                                    kFrames);
        CHECK(checked.size() == kFrames.size());
        if (checked.size() == kFrames.size())
            CHECK(Moved(checked.back(), kOther) == controls[phase]);
        CHECK(evaluator->GetBakedProgram());
        if (evaluator->GetBakedProgram()) {
            const auto refs = Bindings(evaluator->GetBakedProgram()->GetStepGraph());
            const auto *across = Find(refs, dual, kOther, kSlab);
            const auto *own = Find(refs, dual, kSlab, kSlab);
            if (phase == "base") {
                // Base surface inputs use the authored head value, not a phased binding.
                CHECK(across == nullptr && own == nullptr);
                size_t readers = 0;
                const auto &B = evaluator->GetBakedProgram()->GetStepGraph();
                for (const auto &chain : B.chains) {
                    if (chain.target != kOther && chain.target != kSlab) continue;
                    for (const auto &revision : chain.revisions) {
                        if (revision.moverPath != dual) continue;
                        ++readers;
                        CHECK(revision.binding.surfacePoints == kSlab);
                        CHECK(revision.binding.phases.count(kSlab) == 0);
                    }
                }
                CHECK(readers == 2);
            } else {
                CHECK(across && own);
            }
            if (across && own) {
                CHECK(across->binding->candidates.size() == 1);
                if (across->binding->candidates.size() == 1) {
                    CHECK(across->binding->candidates[0].chain ==
                          int(own->readerChain));
                    CHECK(across->binding->candidates[0].version ==
                          (phase == "base" ? 0 : 1));
                }
            }
        }
        VtVec3fArray afterSurface;
        CHECK(stage->GetAttributeAtPath(kSlab).Get(&afterSurface));
        CHECK(afterSurface == authoredSurface);
        if (phase == "base") {
            // The half-size cube's first point projects to the authored
            // cube's z=-1 face, preserving its x/y coordinates.
            CHECK(!controls[phase].empty());
            if (!controls[phase].empty())
                CHECK(controls[phase][0] == GfVec3f(-0.5f, -0.5f, -1.0f));
        }
    }
    CHECK(controls.size() == 2);
    if (others.size() == 2 && controls.size() == 2) {
        CHECK(others[0] == controls.at(kSlabLattice.GetString()));
        CHECK(others[1] == controls.at(kSlabLattice.GetString()));
        CHECK(others[0] != controls.at("base"));
    }
}

// `preceding` on the reader's own chain reads the points entering the
// reader, in the program, the independent scalar reference and the scalar oracle alike. A
// lattice over the cage it deforms, after CageLift and CageTwist, gives the
// same cage and slab as one naming CageTwist, the revision before it, and
// not those of one reading the cage at `base`.
void
TestPrecedingOnTheOwnChainReadsTheEnteringPoints(const std::string &examples)
{
    std::map<std::string, std::vector<VtVec3fArray>> answers;
    for (const char *phase :
         {"base", "preceding", "/ReadPhaseAsset/Rig/Movers/Cage/CageTwist"}) {
        const UsdStageRefPtr stage = OpenReadPhases(examples);
        if (!stage) {
            return;
        }
        DefineLattice(stage, "Self", {kCage}, phase);
        const std::string where = std::string("own chain at ") + phase;
        for (const bool mode :
             {false,
              true}) {
            auto evaluator = MakeEvaluator(stage, kReadPhaseRig, mode);
            if (!evaluator) {
                return;
            }
            Tally tally;
            for (const UsdTimeCode &time : kFrames) {
                const std::string at =
                    where + " at " + std::to_string(time.GetValue());
                const RigExecRigPose pose =
                    mode == true
                        ? evaluator->Evaluate(time)
                        : EvaluateCaptured(at, evaluator.get(), time,
                                           &tally);
                CHECK_AT(at, pose.valid);
                answers[phase].push_back(Moved(pose, kCage));
                answers[phase].push_back(Moved(pose, kSlab));
            }
        }
        // The ExecReference run's answers, cage and slab per frame.
        std::vector<VtVec3fArray> checked;
        for (const RigExecRigPose &pose :
             EvaluateUnderTheOracle(where, stage, kReadPhaseRig, kFrames)) {
            checked.push_back(Moved(pose, kCage));
            checked.push_back(Moved(pose, kSlab));
        }
        const auto &walked = answers[phase];
        CHECK_AT(where, walked.size() == 4 * kFrames.size() &&
                            checked == std::vector<VtVec3fArray>(
                                           walked.begin() + walked.size() / 2,
                                           walked.end()));
    }
    const auto &base = answers["base"];
    const auto &twist = answers["/ReadPhaseAsset/Rig/Movers/Cage/CageTwist"];
    CHECK(!base.empty());
    CHECK(answers["preceding"] == twist);
    CHECK(twist != base);
}

// tests/fixtures/preceding_own_chain.usda: Echo, a lattice whose cage is the
// box it deforms, reads that cage at `preceding`. Its binding is the one
// version entering it (2: after Lift and Settle), answered at every run, and
// the box it leaves equals the box with the cage read at Settle, the
// revision before it, in the program and the independent scalar reference alike, at frames
// 1, 5 and 9 and under a drag on LiftCtl. Reading `base` leaves another box.
// The scalar oracle (cpuReference) reads the phase the same way: the walk
// it checks publishes, with no mismatch, and the oracle's box is the walk's.
void
TestPrecedingOwnChainFixture(const std::string &examples)
{
    const SdfPath rig("/PrecedingAsset/Rig");
    const SdfPath box("/PrecedingAsset/Geom/Box.points");
    const SdfPath echo("/PrecedingAsset/Rig/Movers/Echo");
    const SdfPath liftCtl("/PrecedingAsset/Rig/Controls/LiftCtl");
    const std::string settle = "/PrecedingAsset/Rig/Movers/Settle";
    std::map<std::string, std::vector<VtVec3fArray>> answers;
    std::map<std::string, std::vector<VtVec3fArray>> oracleAnswers;
    for (const std::string phase : {std::string("preceding"), settle,
                                    std::string("base")}) {
        const UsdStageRefPtr stage = UsdStage::Open(
            examples + "/../tests/fixtures/preceding_own_chain.usda");
        CHECK(stage);
        if (!stage) {
            return;
        }
        stage->SetEditTarget(stage->GetSessionLayer());
        SetPhase(stage, echo, "rigExec:cage", phase);
        const std::string where = "preceding_own_chain, cage at " + phase;
        for (const bool mode :
             {false,
              true}) {
            auto evaluator = MakeEvaluator(stage, rig, mode);
            if (!evaluator) {
                return;
            }
            const bool baked =
                mode == true;
            Tally tally;
            const auto evaluate = [&](const std::string &at,
                                      UsdTimeCode time) {
                const RigExecRigPose pose =
                    baked ? EvaluateCaptured(at, evaluator.get(), time,
                                             &tally)
                          : evaluator->Evaluate(time);
                CHECK_AT(at, pose.valid);
                answers[phase].push_back(Moved(pose, box));
            };
            for (const double frame : {1.0, 5.0, 9.0}) {
                evaluate(where + " at " + std::to_string(frame),
                         UsdTimeCode(frame));
            }
            evaluator->SetInteractiveOverrides({RigExecValueOverride{
                liftCtl, TfToken(), TfToken("avars:ty"), VtValue(0.75)}});
            evaluate(where + " dragged", UsdTimeCode(9.0));
            evaluator->ClearInteractiveOverrides();
            evaluate(where + " released", UsdTimeCode(9.0));
            if (!baked || phase != "preceding") {
                continue;
            }
            const RigExecBakedProgram *program = evaluator->GetBakedProgram();
            CHECK_AT(where, program != nullptr);
            if (!program) {
                continue;
            }
            const std::vector<BindingRef> refs =
                Bindings(program->GetStepGraph());
            const BindingRef *own = Find(refs, echo, box, box);
            CHECK_AT(where, own != nullptr);
            if (!own) {
                continue;
            }
            CHECK_AT(where, !own->binding->diagnoseMiss);
            CHECK_AT(where, own->binding->candidates.size() == 1);
            if (own->binding->candidates.size() == 1) {
                CHECK_AT(where, own->binding->candidates[0].chain ==
                                    int(own->readerChain));
                CHECK_AT(where, own->binding->candidates[0].version == 2);
            }
            // The validator holds the assemble to that read: Echo's weight
            // field is not current-phase, so only the binding names
            // version 2, and dropping its RevisionDone read is refused.
            RigExecBakedProgramImpl &B = const_cast<RigExecBakedProgramImpl &>(
                program->GetStepGraph());
            CHECK_AT(where, own->step >= 0);
            if (own->step >= 0 && own->binding->candidates.size() == 1) {
                RigExecBakedStep &assemble = B.steps[size_t(own->step)];
                CHECK_AT(where, assemble.kind ==
                                    RigExecBakedStepKind::RevisionStatic);
                const auto &[chain, r] =
                    B.revisionIndex[size_t(assemble.object)];
                CHECK_AT(where, !B.chains[size_t(chain)]
                                     .revisions[size_t(r)]
                                     .weightCurrentPhase);
                const RigExecBakedSlotRange done = RigExecBakedOne(
                    RigExecBakedSlotDomain::RevisionDone,
                    B.chainRevisionBegin[own->readerChain] + 1);
                const std::vector<RigExecBakedSlotRange> reads =
                    assemble.reads;
                assemble.reads.erase(std::remove(assemble.reads.begin(),
                                                 assemble.reads.end(), done),
                                     assemble.reads.end());
                CHECK_AT(where, assemble.reads.size() + 1 == reads.size());
                std::string error;
                CHECK_AT(where, !RigExecBakedValidateStepGraph(B, &error));
                const std::string expected =
                    "binds point version 2 of chain " +
                    std::to_string(own->readerChain) +
                    " without declaring it";
                CHECK_AT(where + ": " + error,
                         error.find("step " + std::to_string(own->step) + " (") != std::string::npos &&
                             error.find(own->reader.GetString()) != std::string::npos &&
                             error.find(expected) != std::string::npos);
                assemble.reads = reads;
                error.clear();
                CHECK_AT(where + ": " + error,
                         RigExecBakedValidateStepGraph(B, &error));
            }
            if (SerialSchedule()) {
                CHECK_AT(where, tally.read == 5);
                CHECK_AT(where, tally.answered == 5);
            }
        }
        for (const RigExecRigPose &pose : EvaluateUnderTheOracle(
                 where, stage, rig,
                 {UsdTimeCode(1.0), UsdTimeCode(5.0), UsdTimeCode(9.0)})) {
            if (!pose.valid) {
                continue;
            }
            // The one point chain, compared and published as the oracle's.
            CHECK_AT(where, pose.referenceAgreements == 1);
            const auto cpu = pose.movedPropertiesCpu.find(box);
            CHECK_AT(where, cpu != pose.movedPropertiesCpu.end() &&
                                cpu->second == VtValue(Moved(pose, box)));
            oracleAnswers[phase].push_back(Moved(pose, box));
        }
        // The ExecReference run's frames 1, 5 and 9.
        const auto &walked = answers[phase];
        CHECK_AT(where, walked.size() == 10 &&
                            oracleAnswers[phase] ==
                                std::vector<VtVec3fArray>(
                                    walked.begin() + 5, walked.begin() + 8));
    }
    const auto &preceding = answers["preceding"];
    CHECK(preceding.size() == 10);
    CHECK(preceding == answers[settle]);
    CHECK(preceding != answers["base"]);
}

// A chain that reads no base records nothing, so a `final` read of it falls
// to the resolved input with its line -- in the independent scalar reference and in the
// program alike. A value block makes the base read fail (an empty array
// would be a successful one).
void
TestAChainWithoutABaseFallsThrough(const std::string &examples)
{
    const UsdStageRefPtr stage = OpenReadPhases(examples);
    if (!stage) {
        return;
    }
    const UsdAttribute cage = stage->GetAttributeAtPath(kCage);
    VtVec3fArray authored;
    CHECK(cage.Get(&authored));
    CHECK(cage.Set(authored, UsdTimeCode(1001)));
    CHECK(cage.Set(SdfValueBlock(), UsdTimeCode(1002)));
    CHECK(cage.Set(authored, UsdTimeCode(1003)));
    auto baked = MakeEvaluator(stage, kReadPhaseRig,
                               true);
    auto reference = MakeEvaluator(stage, kReadPhaseRig,
                                   true);
    if (!baked || !reference) {
        return;
    }
    const std::string line =
        "diag " + kSlabLattice.GetString() + ": read phase 'final' for " +
        kCage.GetString() + " resolved to nothing; read the authored base";
    const std::string nativeFailure =
        "MoverFailed " + kSlabLattice.GetString() +
        ": execution rejected its inputs; revision passed through";
    const std::string oracleOnly =
        "MoverFailed " + kSlabLattice.GetString() +
        ": lattice cage/divisions mismatch";
    for (const double frame : {1001.0, 1002.0, 1003.0}) {
        const std::string where = "blocked cage at " + std::to_string(frame);
        Tally tally;
        const RigExecRigPose pose =
            EvaluateCaptured(where, baked.get(), UsdTimeCode(frame), &tally);
        const RigExecRigPose walk = reference->Evaluate(UsdTimeCode(frame));
        CHECK_AT(where, walk.valid);
        const bool blocked = frame == 1002.0;
        CHECK_AT(where, Mentions(pose.diagnostics, line) == blocked);
        CHECK_AT(where, Mentions(walk.diagnostics, line) == blocked);
        CHECK_AT(where, Moved(pose, kSlab) == Moved(walk, kSlab));
        CHECK_AT(where, pose.referenceMismatches == 0 &&
                        walk.referenceMismatches == 0);
        CHECK_AT(where, Mentions(pose.diagnostics, nativeFailure) == blocked);
        CHECK_AT(where, Mentions(walk.diagnostics, nativeFailure) == blocked);
        CHECK_AT(where, !Mentions(pose.diagnostics, oracleOnly));
        CHECK_AT(where, !Mentions(walk.diagnostics, oracleOnly));
        CHECK_AT(where, baked->GetBakedProgram() != nullptr);
        if (blocked && baked->GetBakedProgram()) {
            auto &B = const_cast<RigExecBakedProgramImpl &>(
                baked->GetBakedProgram()->GetStepGraph());
            const auto chain = std::find_if(
                B.chains.begin(), B.chains.end(),
                [](const auto &item) { return item.target == kSlab; });
            CHECK_AT(where, chain != B.chains.end());
            CHECK_AT(where, chain != B.chains.end() && !chain->result.empty());
            if (chain != B.chains.end() && !chain->result.empty()) {
                const GfVec3f saved = chain->result[0];
                chain->result[0] = GfVec3f(1234, 5678, 9012);
                RigExecRigPose mismatch;
                mismatch.diagnostics = pose.diagnostics;
                const bool agreed = RigExecBakedRunScalarReference(
                    &B, UsdTimeCode(frame), &mismatch);
                chain->result[0] = saved;
                CHECK_AT(where, !agreed && mismatch.referenceMismatches > 0);
                CHECK_AT(where, Mentions(mismatch.diagnostics, nativeFailure));
                CHECK_AT(where, Mentions(mismatch.diagnostics, oracleOnly));
                CHECK_AT(where, Mentions(mismatch.diagnostics,
                    "cpu reference parity: value mismatch on " +
                    kSlab.GetString()));
            }
        }
        if (SerialSchedule()) {
            CHECK_AT(where, tally.read == 1);
            CHECK_AT(where, tally.tails == (blocked ? 1u : 0u));
        }
    }
}

// Blend samples bound to chain versions: one sample reads its target shape
// at `final`, the other at `preceding`, which compile turns into the last
// writer of the shape before the blend in the walk (LiftA, not LiftB).
const char *const kBlendRig = R"(#usda 1.0
(
    startTimeCode = 1
    endTimeCode = 9
)
def Xform "Asset"
{
    def RigExecRoot "Rig"
    {
        def Scope "Controls"
        {
            def RigExecControl "CtlA" (prepend apiSchemas = ["RigExecControlAPI"])
            {
                double avars:ty = 0
                double avars:ty.spline = {
                    1: 0; pre (0, 0); post linear,
                    9: 2; post linear,
                }
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
            }
            def RigExecControl "CtlB" (prepend apiSchemas = ["RigExecControlAPI"])
            {
                double avars:tx = 0
                double avars:tx.spline = {
                    1: 0; pre (0, 0); post linear,
                    9: 1; post linear,
                }
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
            }
        }
        def Scope "Solvers"
        {
            def RigExecFkChain "ChainA"
            {
                rel rigExec:controls = </Asset/Rig/Controls/CtlA>
                rel rigExec:joints = </Asset/Rig/Joints/A>
            }
            def RigExecFkChain "ChainB"
            {
                rel rigExec:controls = </Asset/Rig/Controls/CtlB>
                rel rigExec:joints = </Asset/Rig/Joints/B>
            }
        }
        def Scope "Joints"
        {
            def RigExecJoint "A"
            {
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
            }
            def RigExecJoint "B"
            {
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
            }
        }
        def Scope "Movers"
        {
            # Walked last child first: LiftA, then Blend, then LiftB.
            reorder nameChildren = ["LiftB", "Blend", "LiftA"]
            def RigExecMatrixMover "LiftB" (prepend apiSchemas = ["RigExecMoverAPI"])
            {
                rel rigExec:transform = </Asset/Rig/Joints/B> (rigExecReadPhase = "final")
                rel rigExec:weightObject = </Asset/Rig/Weights/ShapeW>
                rel rigExec:moves = </Asset/Geom/Shape.points>
            }
            def RigExecBlendShapeMover "Blend" (prepend apiSchemas = ["RigExecMoverAPI"])
            {
                rel rigExec:moves = </Asset/Geom/Body.points>
                rel rigExec:blendInputs = [</Asset/Rig/BlendInputs/Final>, </Asset/Rig/BlendInputs/Preceding>]
                rel rigExec:weightObject = </Asset/Rig/Weights/BodyW>
            }
            def RigExecMatrixMover "LiftA" (prepend apiSchemas = ["RigExecMoverAPI"])
            {
                rel rigExec:transform = </Asset/Rig/Joints/A> (rigExecReadPhase = "final")
                rel rigExec:weightObject = </Asset/Rig/Weights/ShapeW>
                rel rigExec:moves = </Asset/Geom/Shape.points>
            }
        }
        def Scope "BlendInputs"
        {
            def RigExecBlendInput "Final"
            {
                float inputs:weight = 0.5
                rel rigExec:samples = </Asset/Rig/BlendInputs/Final/Full>
                def RigExecBlendSample "Full"
                {
                    float rigExec:activation = 1
                    rel rigExec:targetPoints = </Asset/Geom/Shape.points> (rigExecReadPhase = "final")
                }
            }
            def RigExecBlendInput "Preceding"
            {
                float inputs:weight = 0.25
                rel rigExec:samples = </Asset/Rig/BlendInputs/Preceding/Full>
                def RigExecBlendSample "Full"
                {
                    float rigExec:activation = 1
                    rel rigExec:targetPoints = </Asset/Geom/Shape.points> (rigExecReadPhase = "preceding")
                }
            }
        }
        def Scope "Weights"
        {
            def RigExecStaticWeight "ShapeW"
            {
                uniform float rigExec:defaultWeight = 1
                uniform token rigExec:rangePolicy = "strict"
                uniform token rigExec:representation = "constant"
                rel rigExec:weightTarget = </Asset/Geom/Shape.points>
            }
            def RigExecStaticWeight "BodyW"
            {
                uniform float rigExec:defaultWeight = 1
                uniform token rigExec:rangePolicy = "strict"
                uniform token rigExec:representation = "constant"
                rel rigExec:weightTarget = </Asset/Geom/Body.points>
            }
        }
    }
    def Scope "Geom"
    {
        def Points "Body"
        {
            point3f[] points = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)]
        }
        def Points "Shape"
        {
            point3f[] points = [(0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)]
        }
    }
}
)";

void
TestBlendSamplesBindToVersions()
{
    const SdfLayerRefPtr layer = SdfLayer::CreateAnonymous(".usda");
    CHECK(rigExec::RigExecInputReplayImportFromString(layer, kBlendRig));
    const UsdStageRefPtr stage = UsdStage::Open(layer);
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath blend("/Asset/Rig/Movers/Blend");
    const SdfPath body("/Asset/Geom/Body.points");
    const SdfPath shape("/Asset/Geom/Shape.points");
    auto evaluator = MakeEvaluator(stage, SdfPath("/Asset/Rig"),
                                   true);
    if (!evaluator || !evaluator->GetBakedProgram()) {
        CHECK(!"the blend rig bakes");
        return;
    }
    const std::vector<BindingRef> refs =
        Bindings(evaluator->GetBakedProgram()->GetStepGraph());
    size_t samples = 0;
    for (const BindingRef &ref : refs) {
        if (!ref.sample) {
            continue;
        }
        ++samples;
        CHECK(ref.reader == blend && ref.binding->input == shape);
        CHECK(!ref.binding->diagnoseMiss);
        CHECK(ref.binding->candidates.size() == 1);
        if (ref.binding->candidates.size() == 1) {
            // `final`: the published chain; `preceding`: after LiftA, the
            // shape chain's first revision.
            CHECK(ref.binding->finalRead
                      ? ref.binding->candidates[0].version == 2
                      : ref.binding->candidates[0].version == 1);
        }
    }
    CHECK(samples == 2);
    Tally tally;
    VtVec3fArray first;
    for (const double frame : {1.0, 3.0, 7.0}) {
        const RigExecRigPose pose = EvaluateCaptured(
            "blend samples at " + std::to_string(frame), evaluator.get(),
            UsdTimeCode(frame), &tally);
        if (frame == 1.0) {
            first = Moved(pose, body);
        } else {
            CHECK(Moved(pose, body) != first);
        }
    }
    evaluator->SetInteractiveOverrides({RigExecValueOverride{
        SdfPath("/Asset/Rig/Controls/CtlB"), TfToken(), TfToken("avars:tx"),
        VtValue(3.0)}});
    EvaluateCaptured("blend samples dragged", evaluator.get(), UsdTimeCode(7),
                     &tally);
    evaluator->ClearInteractiveOverrides();
    if (SerialSchedule()) {
        CHECK(tally.read == 8 && tally.answered == 8);
    }
}

// AtPrim read phases on rigExec:transform, resolved through frame records
// (tests/fixtures/frame_record_fallbacks.usda): three constraints revise one
// joint, and five matrix movers read it at C1, at the Group holding C1 and
// C2, at C2, at the Constrain scope and at `base`.

const SdfPath kRecordRig("/RecordAsset/Rig");
const SdfPath kRecordJoint("/RecordAsset/Rig/Joints/X");
const SdfPath kRecordC1("/RecordAsset/Rig/Movers/Constrain/Group/C1");
const SdfPath kRecordC2("/RecordAsset/Rig/Movers/Constrain/Group/C2");
const SdfPath kRecordC3("/RecordAsset/Rig/Movers/Constrain/C3");
const SdfPath kRecordDial("/RecordAsset/Rig/Controls/Dial");

SdfPath
RecordReader(const char *name)
{
    return SdfPath("/RecordAsset/Rig/Movers/Geometry").AppendChild(
        TfToken(name));
}

SdfPath
RecordPoints(const char *name)
{
    return SdfPath("/RecordAsset/Geom")
        .AppendChild(TfToken(name))
        .AppendProperty(TfToken("points"));
}

using rigExecTest::FindRevision;
using rigExecTest::FirstValidRecord;
using rigExecTest::RecordMovers;

/// Holds every AtPrim transform reader of \p evaluator's program to the
/// scalar reference's phased-read store at the end of the run. Returns the
/// readers a record answered.
size_t
CheckFrameRecords(const std::string &where,
                  const RigExecRigEvaluator &evaluator)
{
    return rigExecTest::CheckFrameRecords(&failures, where, evaluator);
}

/// The FrameMatrix step of each record: one, after every step of its
/// commit, reading the provider and the commit's table; and \p reader's
/// fold waits for each of its records.
void
CheckFrameRecordSteps(const std::string &where,
                      const RigExecBakedProgramImpl &B, const SdfPath &reader)
{
    std::vector<int> stepOf(B.frameRecords.size(), -1);
    std::vector<int> commitLast(B.commits.size(), -1);
    for (size_t s = 0; s < B.steps.size(); ++s) {
        const RigExecBakedStep &step = B.steps[s];
        switch (step.kind) {
        case RigExecBakedStepKind::SolverCommit:
        case RigExecBakedStepKind::Constraint:
        case RigExecBakedStepKind::CommitDelta:
        case RigExecBakedStepKind::PropagateChunk:
        case RigExecBakedStepKind::CommitApply:
            commitLast[size_t(step.object)] = int(s);
            break;
        case RigExecBakedStepKind::FrameMatrix:
            CHECK_AT(where, size_t(step.object) < stepOf.size());
            if (size_t(step.object) < stepOf.size()) {
                CHECK_AT(where, stepOf[size_t(step.object)] < 0);
                stepOf[size_t(step.object)] = int(s);
            }
            break;
        default:
            break;
        }
    }
    for (size_t r = 0; r < B.frameRecords.size(); ++r) {
        const RigExecBakedFrameRecord &record = B.frameRecords[r];
        const std::string at = where + " record after " +
                               record.mover.GetString();
        CHECK_AT(at, stepOf[r] >= 0);
        if (stepOf[r] < 0) {
            continue;
        }
        const RigExecBakedStep &step = B.steps[size_t(stepOf[r])];
        CHECK_AT(at, stepOf[r] > commitLast[size_t(record.commit)]);
        CHECK_AT(at, std::find(step.reads.begin(), step.reads.end(),
                               RigExecBakedOne(
                                   RigExecBakedSlotDomain::PoseFin,
                                   record.version)) != step.reads.end());
        CHECK_AT(at, std::find(step.reads.begin(), step.reads.end(),
                               RigExecBakedOne(
                                   RigExecBakedSlotDomain::CommitTable,
                                   record.commit)) != step.reads.end());
        // The provider is the constraint's candidate, so the record reads
        // the version its write-back produces.
        const RigExecBakedCommit &commit = B.commits[size_t(record.commit)];
        const auto found = std::find(commit.slots.begin(), commit.slots.end(),
                                     record.slot);
        CHECK_AT(at, found != commit.slots.end());
        if (found != commit.slots.end()) {
            CHECK_AT(at, record.version ==
                             commit.slotWrites[size_t(
                                 found - commit.slots.begin())]);
        }
    }
    const RigExecBakedProgramImpl::GeomRevision *revision =
        FindRevision(B, reader);
    CHECK_AT(where, revision != nullptr);
    if (!revision) {
        return;
    }
    int fold = -1;
    for (size_t s = 0; s < B.steps.size(); ++s) {
        const RigExecBakedStep &step = B.steps[s];
        if (step.kind == RigExecBakedStepKind::InfluenceFold &&
            B.chains[size_t(B.revisionIndex[size_t(step.object)].first)]
                    .revisions[size_t(
                        B.revisionIndex[size_t(step.object)].second)]
                    .moverPath == reader) {
            fold = int(s);
        }
    }
    CHECK_AT(where, fold >= 0);
    if (fold < 0) {
        return;
    }
    const std::vector<int> &preds = B.steps[size_t(fold)].preds;
    for (const int record : revision->transformRecords) {
        CHECK_AT(where + " fold waits for record " + std::to_string(record),
                 std::find(preds.begin(), preds.end(),
                           stepOf[size_t(record)]) != preds.end());
    }
    std::string error;
    CHECK_AT(where, RigExecBakedValidateStepGraph(B, &error));
    if (!error.empty()) {
        std::printf("  %s\n", error.c_str());
    }
}

/// Which steps the last generation ran, held to the cone: exactly the
/// FrameMatrix steps of the records whose constraint is in \p movers, and
/// the InfluenceFold of \p folded, when named.
void
CheckRecordStepsRan(const std::string &where,
                    const RigExecRigEvaluator &evaluator,
                    const std::vector<SdfPath> &movers, const char *folded)
{
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    CHECK_AT(where, program != nullptr);
    if (!program) {
        return;
    }
    const RigExecBakedProgramImpl &B = program->GetStepGraph();
    const auto moverOf = [&B](const RigExecBakedStep &step) {
        const auto [c, r] = B.revisionIndex[size_t(step.object)];
        return B.chains[size_t(c)].revisions[size_t(r)].moverPath;
    };
    size_t frameSteps = 0;
    bool foldRan = false;
    for (const RigExecBakedStep &step : B.steps) {
        if (step.kind == RigExecBakedStepKind::FrameMatrix) {
            ++frameSteps;
            const SdfPath &mover = B.frameRecords[size_t(step.object)].mover;
            const bool expected =
                std::find(movers.begin(), movers.end(), mover) !=
                movers.end();
            CHECK_AT(where + ": FrameMatrix " + step.label,
                     ((size_t(&step - B.steps.data()) < B.opExecution.ran.size() && B.opExecution.ran[size_t(&step - B.steps.data())])) == expected);
        } else if (*folded &&
                   step.kind == RigExecBakedStepKind::InfluenceFold &&
                   moverOf(step) == RecordReader(folded)) {
            foldRan = (size_t(&step - B.steps.data()) < B.opExecution.ran.size() && B.opExecution.ran[size_t(&step - B.steps.data())]);
        }
    }
    CHECK_AT(where, frameSteps == B.frameRecords.size());
    CHECK_AT(where, !*folded || foldRan);
}

bool
Near(const VtVec3fArray &a, const VtVec3fArray &b)
{
    if (a.size() != b.size()) {
        return false;
    }
    for (size_t i = 0; i < a.size(); ++i) {
        if ((GfVec3d(a[i]) - GfVec3d(b[i])).GetLength() > 1e-5) {
            return false;
        }
    }
    return true;
}

/// Each reader's points at one evaluation.
struct RecordReads {
    VtVec3fArray r1, r2, r4, r5, rb;
};

RecordReads
ReadRecordPoints(const RigExecRigPose &pose, bool withR1)
{
    RecordReads out;
    if (withR1) {
        out.r1 = Moved(pose, RecordPoints("P1"));
    }
    out.r2 = Moved(pose, RecordPoints("P2"));
    out.r4 = Moved(pose, RecordPoints("P4"));
    out.r5 = Moved(pose, RecordPoints("P5"));
    out.rb = Moved(pose, RecordPoints("PB"));
    return out;
}

// The lists are the store's: only pairs some reader names are recorded, and
// an AtPrim(P) list holds every named record of the provider at or under P,
// newest first. So R2's list at Group holds C1's record only while R1 names
// it, and R5's at Constrain holds all three, newest (C3) first. C2's
// disabled exit at frame 4 records X as C1 left it, which moves R2 and R4
// onto R1. Every record of a catalogued provider is valid in an authored rig
// (see RigExecBakedEvalFrameRecord's gates, held by TestFrameRecordGates), so
// the variant without R1 reads the same values through a shorter list.
void
TestFrameRecordTypedProvenance(const std::string &examples)
{
    const std::string path =
        examples + "/../tests/fixtures/frame_record_fallbacks.usda";
    std::map<double, RecordReads> withR1Reads;
    for (const bool withR1 : {true, false}) {
        const std::string label =
            withR1 ? "frame records" : "frame records without R1";
        const UsdStageRefPtr stage = UsdStage::Open(path);
        CHECK_AT(label, stage);
        if (!stage) {
            return;
        }
        stage->SetEditTarget(stage->GetSessionLayer());
        if (!withR1) {
            CHECK_AT(label,
                     stage->GetPrimAtPath(RecordReader("R1")).SetActive(false));
        }
        auto evaluator = MakeEvaluator(
            stage, kRecordRig, true);
        if (!evaluator) {
            return;
        }
        Tally tally;
        bool structureChecked = false;
        for (const double frame : {1.0, 4.0, 8.0}) {
            const std::string where =
                label + " frame " + std::to_string(int(frame));
            const RigExecRigPose pose = EvaluateCaptured(
                where, evaluator.get(), UsdTimeCode(frame), &tally);
            // R1, R2, R4 and R5 all answer from a record; RB reads `base`.
            CHECK_AT(where,
                     CheckFrameRecords(where, *evaluator) == (withR1 ? 4 : 3));
            const RigExecBakedProgram *program = evaluator->GetBakedProgram();
            if (!program) {
                CHECK_AT(where, !"the rig bakes");
                return;
            }
            const RigExecBakedProgramImpl &B = program->GetStepGraph();
            if (!structureChecked) {
                structureChecked = true;
                CHECK_AT(where, B.frameRecords.size() == (withR1 ? 3u : 2u));
                const auto list = [&](const char *reader) {
                    const auto *revision = FindRevision(B, RecordReader(reader));
                    CHECK_AT(where + " " + reader, revision != nullptr);
                    return revision
                               ? RecordMovers(B, revision->transformRecords)
                               : std::vector<SdfPath>{SdfPath("/missing")};
                };
                if (withR1) {
                    CHECK_AT(where, list("R1") ==
                                        std::vector<SdfPath>{kRecordC1});
                    CHECK_AT(where, list("R2") ==
                                        (std::vector<SdfPath>{kRecordC2,
                                                              kRecordC1}));
                    CHECK_AT(where, list("R5") ==
                                        (std::vector<SdfPath>{
                                            kRecordC3, kRecordC2, kRecordC1}));
                } else {
                    CHECK_AT(where, list("R2") ==
                                        std::vector<SdfPath>{kRecordC2});
                    CHECK_AT(where, list("R5") ==
                                        (std::vector<SdfPath>{kRecordC3,
                                                              kRecordC2}));
                }
                CHECK_AT(where, list("R4") ==
                                    std::vector<SdfPath>{kRecordC2});
                CHECK_AT(where, list("RB").empty());
                for (const RigExecBakedFrameRecord &record : B.frameRecords) {
                    CHECK_AT(where, B.paths[size_t(record.slot)] ==
                                        kRecordJoint);
                    CHECK_AT(where, record.target == 0);
                }
                CheckFrameRecordSteps(where, B, RecordReader("R2"));
            }
            for (size_t r = 0; r < B.frameRecords.size(); ++r) {
                CHECK_AT(where, B.frameMatrixValid[r] == 1);
            }
            const RecordReads reads = ReadRecordPoints(pose, withR1);
            CHECK_AT(where, Near(reads.r2, reads.r4));
            CHECK_AT(where, !Near(reads.r5, reads.r2));
            CHECK_AT(where, !Near(reads.r5, reads.rb));
            if (withR1) {
                CHECK_AT(where, !Near(reads.r1, reads.rb));
                // C2 rotates X except where it is disabled, and its disabled
                // exit records X as C1 left it.
                CHECK_AT(where, Near(reads.r2, reads.r1) == (frame == 4.0));
                withR1Reads[frame] = reads;
            } else {
                const RecordReads &named = withR1Reads[frame];
                CHECK_AT(where, reads.r2 == named.r2);
                CHECK_AT(where, reads.r4 == named.r4);
                CHECK_AT(where, reads.r5 == named.r5);
                CHECK_AT(where, reads.rb == named.rb);
            }
        }
        // Dial weights R2 alone: the drag moves R2's points and no other
        // reader's, and the release restores them. Under the cone it runs no
        // FrameMatrix step: the fold's table is the one the kept records gave
        // last run.
        const RecordReads before = ReadRecordPoints(
            evaluator->Evaluate(UsdTimeCode(8.0)), withR1);
        evaluator->SetInteractiveOverrides({RigExecValueOverride{
            kRecordDial, TfToken(), TfToken("avars:amount"), VtValue(0.5)}});
        const RigExecRigPose dragged = EvaluateCaptured(
            label + " dragged", evaluator.get(), UsdTimeCode(8.0), &tally);
        CHECK_AT(label + " dragged", CheckFrameRecords(label + " dragged",
                                                       *evaluator) ==
                                         (withR1 ? 4 : 3));
        CheckRecordStepsRan(label + " dragged", *evaluator, {}, "");
        const RecordReads moved = ReadRecordPoints(dragged, withR1);
        CHECK_AT(label + " dragged", !Near(moved.r2, before.r2));
        CHECK_AT(label + " dragged", moved.r4 == before.r4);
        CHECK_AT(label + " dragged", moved.r5 == before.r5);
        evaluator->ClearInteractiveOverrides();
        const RigExecRigPose released = EvaluateCaptured(
            label + " released", evaluator.get(), UsdTimeCode(8.0), &tally);
        CHECK_AT(label + " released",
                 ReadRecordPoints(released, withR1).r2 == before.r2);
        // Late is C3's source: the drag re-runs C3's record and R5's fold,
        // which reads it beside C2's and C1's records, both kept from the
        // last run. Every reader is held to the typed version metadata and to the
        // scalar reference's points.
        evaluator->SetInteractiveOverrides({RigExecValueOverride{
            SdfPath("/RecordAsset/Rig/Controls/Late"), TfToken(),
            TfToken("avars:tz"), VtValue(0.25)}});
        const RigExecRigPose late = EvaluateCaptured(
            label + " Late dragged", evaluator.get(), UsdTimeCode(8.0),
            &tally);
        CHECK_AT(label + " Late dragged",
                 CheckFrameRecords(label + " Late dragged", *evaluator) ==
                     (withR1 ? 4 : 3));
        CheckRecordStepsRan(label + " Late dragged", *evaluator, {kRecordC3},
                            "R5");
        const RecordReads lateReads = ReadRecordPoints(late, withR1);
        CHECK_AT(label + " Late dragged", !Near(lateReads.r5, before.r5));
        CHECK_AT(label + " Late dragged", lateReads.r2 == before.r2);
        CHECK_AT(label + " Late dragged", lateReads.r4 == before.r4);
        evaluator->ClearInteractiveOverrides();
        const RigExecRigPose lateReleased = EvaluateCaptured(
            label + " Late released", evaluator.get(), UsdTimeCode(8.0),
            &tally);
        CHECK_AT(label + " Late released",
                 ReadRecordPoints(lateReleased, withR1).r5 == before.r5);
    }
}

// The fold reads the FrameMatrix records it declares: C3's record rebound to
// C1's version of X moves R5 onto R1, while the independent scalar reference still answers
// C3's matrix for R5. The validator accepts the rebinding (a version of the
// record's own slot, written before the step), so only the fold's source
// decides R5. Plain Baked, since the independent scalar reference is not rebound.
void
TestTheFoldReadsItsRecords(const std::string &examples)
{
    const std::string where = "fold reads its records";
    const UsdStageRefPtr stage = UsdStage::Open(
        examples + "/../tests/fixtures/frame_record_fallbacks.usda");
    CHECK_AT(where, stage);
    if (!stage) {
        return;
    }
    auto evaluator =
        MakeEvaluator(stage, kRecordRig, false);
    if (!evaluator) {
        return;
    }
    const RigExecRigPose first = evaluator->Evaluate(UsdTimeCode(8.0));
    CHECK_AT(where, first.valid);
    const RigExecBakedProgram *program = evaluator->GetBakedProgram();
    CHECK_AT(where, program != nullptr);
    if (!program) {
        return;
    }
    RigExecBakedProgramImpl &B =
        const_cast<RigExecBakedProgramImpl &>(program->GetStepGraph());
    CHECK_AT(where, RecordMovers(B, {0, 1, 2}) ==
                        (std::vector<SdfPath>{kRecordC1, kRecordC2,
                                              kRecordC3}));
    const RigExecBakedProgramImpl::GeomRevision *r5 =
        FindRevision(B, RecordReader("R5"));
    CHECK_AT(where, r5 != nullptr);
    if (B.frameRecords.size() != 3 || !r5) {
        return;
    }
    const RecordReads before = ReadRecordPoints(first, true);
    const GfMatrix4d afterC1 = B.frameMatrix[0];
    const GfMatrix4d afterC3 = B.frameMatrix[2];
    CHECK_AT(where, afterC1 != afterC3);
    CHECK_AT(where, before.r5 != before.r1);

    // Another frame first, so frame 8 is a fresh run.
    CHECK_AT(where, evaluator->Evaluate(UsdTimeCode(1.0)).valid);
    const uint32_t bound = B.frameRecords[2].version;
    B.frameRecords[2].version = B.frameRecords[0].version;
    std::string error;
    CHECK_AT(where, !RigExecBakedValidateStepGraph(B, &error));
    CHECK_AT(where, error.find("without declaring it") != std::string::npos);
    int reboundStep = -1;
    for (size_t s = 0; s < B.steps.size(); ++s)
        if (B.steps[s].kind == RigExecBakedStepKind::FrameMatrix && B.steps[s].object == 2)
            reboundStep = int(s);
    CHECK_AT(where, reboundStep >= 0);
    if (reboundStep < 0) { B.frameRecords[2].version = bound; return; }
    auto &declared = B.steps[size_t(reboundStep)];
    const auto originalReads = declared.reads;
    const auto originalPreds = declared.preds;
    const uint32_t reboundVersion = B.frameRecords[0].version;
    size_t replaced = 0;
    for (auto &read : declared.reads)
        if (read.domain == RigExecBakedSlotDomain::PoseFin && read.begin == bound && read.end == bound + 1) {
            read = RigExecBakedOne(RigExecBakedSlotDomain::PoseFin, int(reboundVersion));
            ++replaced;
        }
    CHECK_AT(where, replaced == 1);
    size_t producers = 0;
    std::vector<std::pair<int, std::vector<int>>> originalSuccessors;
    struct SavedClusterEdges { int cluster; std::vector<int> preds, succs; };
    std::vector<SavedClusterEdges> originalClusterEdges;
    for (size_t s = 0; s < B.steps.size(); ++s)
        for (const auto &write : B.steps[s].writes)
            if (write.domain == RigExecBakedSlotDomain::PoseFin && write.begin <= reboundVersion && reboundVersion < write.end) {
                CHECK_AT(where, int(s) < reboundStep);
                ++producers;
                declared.preds.push_back(int(s));
                originalSuccessors.emplace_back(int(s), B.steps[s].succs);
                auto &successors = B.steps[s].succs;
                successors.push_back(reboundStep);
                std::sort(successors.begin(), successors.end());
                successors.erase(std::unique(successors.begin(), successors.end()), successors.end());
                const int from = B.clustering.clusterOf[s];
                const int to = B.clustering.clusterOf[size_t(reboundStep)];
                CHECK_AT(where, from >= 0 && to >= 0);
                if (from >= 0 && to >= 0 && from != to) {
                    for (const int cluster : {from, to}) {
                        const auto &saved = B.clustering.clusters[size_t(cluster)];
                        originalClusterEdges.push_back({cluster, saved.preds, saved.succs});
                    }
                    auto &clusterPreds = B.clustering.clusters[size_t(to)].preds;
                    auto &clusterSuccs = B.clustering.clusters[size_t(from)].succs;
                    clusterPreds.push_back(from);
                    clusterSuccs.push_back(to);
                    std::sort(clusterPreds.begin(), clusterPreds.end());
                    clusterPreds.erase(std::unique(clusterPreds.begin(), clusterPreds.end()), clusterPreds.end());
                    std::sort(clusterSuccs.begin(), clusterSuccs.end());
                    clusterSuccs.erase(std::unique(clusterSuccs.begin(), clusterSuccs.end()), clusterSuccs.end());
                    const auto &order = B.clustering.topologicalOrder;
                    const auto sourcePosition = std::find(order.begin(), order.end(), from);
                    const auto consumerPosition = std::find(order.begin(), order.end(), to);
                    CHECK_AT(where, sourcePosition != order.end() && consumerPosition != order.end());
                    CHECK_AT(where, sourcePosition < consumerPosition);
                }
            }
    CHECK_AT(where, producers == 1);
    std::sort(declared.preds.begin(), declared.preds.end());
    declared.preds.erase(std::unique(declared.preds.begin(), declared.preds.end()), declared.preds.end());
    error.clear();
    CHECK_AT(where, RigExecBakedValidateStepGraph(B, &error));
    if (!error.empty()) {
        std::printf("  %s\n", error.c_str());
    }
    const size_t generations = evaluator->GetBakedGenerationCount();
    const RigExecRigPose rebound = evaluator->Evaluate(UsdTimeCode(8.0));
    CHECK_AT(where, rebound.valid);
    CHECK_AT(where, evaluator->GetBakedGenerationCount() == generations + 1);
    CHECK_AT(where, evaluator->GetBakedProgram() == program);
    const RecordReads reads = ReadRecordPoints(rebound, true);
    CHECK_AT(where, B.frameMatrix[2] == afterC1);
    CHECK_AT(where, reads.r5 == reads.r1);
    CHECK_AT(where, reads.r1 == before.r1);
    CHECK_AT(where, reads.r2 == before.r2);
    auto reference =
        MakeEvaluator(stage, kRecordRig, true);
    if (reference) {
        const RigExecRigPose walked = reference->Evaluate(UsdTimeCode(8.0));
        CHECK_AT(where, walked.valid);
        CHECK_AT(where, ReadRecordPoints(walked, true).r5 == before.r5);
        CHECK_AT(where, reads.r5 != before.r5);
    }

    B.frameRecords[2].version = bound;
    declared.reads = originalReads;
    declared.preds = originalPreds;
    for (const auto &saved : originalClusterEdges) {
        B.clustering.clusters[size_t(saved.cluster)].preds = saved.preds;
        B.clustering.clusters[size_t(saved.cluster)].succs = saved.succs;
    }
    for (const auto &saved : originalSuccessors)
        B.steps[size_t(saved.first)].succs = saved.second;
    error.clear();
    CHECK_AT(where, RigExecBakedValidateStepGraph(B, &error));
    CHECK_AT(where, evaluator->Evaluate(UsdTimeCode(1.0)).valid);
    CHECK_AT(where,
             ReadRecordPoints(evaluator->Evaluate(UsdTimeCode(8.0)), true)
                     .r5 == before.r5);
}

// 13_ReadPhases plus an unrelated chain: Other, moved by a matrix mover on a
// joint FreeCtl drives, which reaches neither the cage nor the slab. A phased
// rig runs under the cone like any other: an unchanged repeat runs no pose
// step, and a drag of FreeCtl leaves the cage's providers clean. The kept
// answers equal a fresh walk's under the same drag. (A RevisionStatic that
// reads another step's slots is dirty every run whatever its phases, so the
// lattice's assemble itself still runs; that is the cone's rule for every
// rig, ArmRig included.)
const char *const kFreeChain = R"(#usda 1.0
over "ReadPhaseAsset"
{
    over "Rig"
    {
        over "Controls"
        {
            def RigExecControl "FreeCtl" (prepend apiSchemas = ["RigExecControlAPI"])
            {
                double avars:ty = 0
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
            }
        }
        over "Solvers"
        {
            def RigExecFkChain "FreeChain"
            {
                rel rigExec:controls = </ReadPhaseAsset/Rig/Controls/FreeCtl>
                rel rigExec:joints = </ReadPhaseAsset/Rig/Joints/Free>
            }
        }
        over "Joints"
        {
            def RigExecJoint "Free"
            {
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
            }
        }
        over "Movers"
        {
            over "Geometry"
            {
                def RigExecMatrixMover "OtherLift" (prepend apiSchemas = ["RigExecMoverAPI"])
                {
                    rel rigExec:transform = </ReadPhaseAsset/Rig/Joints/Free> (rigExecReadPhase = "final")
                    rel rigExec:weightObject = </ReadPhaseAsset/Rig/Weights/OtherW>
                    rel rigExec:moves = </ReadPhaseAsset/Geom/Other.points>
                }
            }
        }
        over "Weights"
        {
            def RigExecStaticWeight "OtherW"
            {
                uniform float rigExec:defaultWeight = 1
                uniform token rigExec:rangePolicy = "strict"
                uniform token rigExec:representation = "constant"
                rel rigExec:weightTarget = </ReadPhaseAsset/Geom/Other.points>
            }
        }
    }
    over "Geom"
    {
        def Points "Other"
        {
            point3f[] points = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)]
        }
    }
}
)";

void
TestAPhasedRigSkipsOnRepeat(const std::string &examples)
{
    const std::string where = "phased rig under the cone";
    const UsdStageRefPtr stage = OpenReadPhases(examples);
    if (!stage) {
        return;
    }
    CHECK_AT(where, rigExec::RigExecInputReplayImportFromString(stage->GetSessionLayer(), kFreeChain));
    auto baked =
        MakeEvaluator(stage, kReadPhaseRig, false);
    if (!baked) {
        return;
    }
    const UsdTimeCode time(1007);
    const RigExecRigPose first = baked->Evaluate(time);
    CHECK_AT(where, first.valid);
    const RigExecBakedProgram *program = baked->GetBakedProgram();
    CHECK_AT(where, program != nullptr);
    if (!program) {
        return;
    }
    const RigExecBakedProgramImpl &B = program->GetStepGraph();
    // Some revision declares a read phase: an input phase, an AtPrim
    // transform, or a phased blend sample.
    const auto declaresPhase =
        [](const RigExecBakedProgramImpl::GeomRevision &revision) {
            const RigExecRevisionBinding &binding = revision.binding;
            bool phased =
                !binding.phases.empty() ||
                binding.transformPhase.kind == RigExecReadPhaseKind::AtPrim;
            for (const auto &[input, samples] : binding.blendSamples) {
                for (const RigExecBlendSampleBinding &sample : samples) {
                    phased = phased || !sample.phase.IsBase();
                }
            }
            return phased;
        };
    bool phasedRig = false;
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        for (const auto &revision : chain.revisions) {
            phasedRig = phasedRig || declaresPhase(revision);
        }
        for (const auto &derived : chain.derived) {
            phasedRig = phasedRig || declaresPhase(derived.revision);
        }
    }
    CHECK_AT(where, phasedRig);
    const auto providerMatrices = [&B](const char *joint) {
        std::vector<int> out;
        const SdfPath path =
            SdfPath("/ReadPhaseAsset/Rig/Joints").AppendChild(TfToken(joint));
        for (size_t s = 0; s < B.steps.size(); ++s) {
            const RigExecBakedStep &step = B.steps[s];
            if (step.kind == RigExecBakedStepKind::ProviderMatrix &&
                B.paths[size_t(step.object)] == path) {
                out.push_back(int(s));
            }
        }
        return out;
    };
    const std::vector<int> lift = providerMatrices("Lift");
    const std::vector<int> twist = providerMatrices("Twist");
    const std::vector<int> free = providerMatrices("Free");
    CHECK_AT(where, !lift.empty() && !twist.empty() && !free.empty());
    const auto anyRan = [&B](const std::vector<int> &steps) {
        for (const int step : steps) {
            if ((size_t(step) < B.opExecution.ran.size() && B.opExecution.ran[size_t(step)])) {
                return true;
            }
        }
        return false;
    };
    const auto poseRan = [&B]() {
        size_t count = 0;
        for (const RigExecBakedStep &step : B.steps) {
            count += (size_t(&step - B.steps.data()) < B.opExecution.ran.size() && B.opExecution.ran[size_t(&step - B.steps.data())]) &&
                             std::string(RigExecBakedStepDomainName(
                                 step.kind)) == "pose"
                         ? 1
                         : 0;
        }
        return count;
    };
    CHECK_AT(where + " first", anyRan(lift) && anyRan(free));

    // The same time again: nothing in the pose half moved, so none of it
    // runs -- before, a phased rig ran every step of every run.
    const RigExecRigPose repeat = baked->Evaluate(time);
    CHECK_AT(where + " repeat", repeat.valid);
    CHECK_AT(where + " repeat", repeat.executedOpCount == 0);
    CHECK_AT(where + " repeat", poseRan() == 0);
    for (const SdfPath &target : {kSlab, kCage, kOther}) {
        CHECK_AT(where + " repeat " + target.GetString(),
                 Moved(repeat, target) == Moved(first, target));
    }

    // FreeCtl reaches Other only: Lift's and Twist's matrices stay clean,
    // and the cage the lattice reads at `final` is the one they left.
    const std::vector<RigExecValueOverride> drag{RigExecValueOverride{
        SdfPath("/ReadPhaseAsset/Rig/Controls/FreeCtl"), TfToken(),
        TfToken("avars:ty"), VtValue(0.75)}};
    baked->SetInteractiveOverrides(drag);
    const RigExecRigPose dragged = baked->Evaluate(time);
    CHECK_AT(where + " dragged", dragged.valid);
    CHECK_AT(where + " dragged", anyRan(free));
    CHECK_AT(where + " dragged", !anyRan(lift) && !anyRan(twist));
    CHECK_AT(where + " dragged",
             Moved(dragged, kOther) != Moved(first, kOther));
    auto reference = MakeEvaluator(stage, kReadPhaseRig,
                                   true);
    if (!reference) {
        return;
    }
    reference->SetInteractiveOverrides(drag);
    const RigExecRigPose walked = reference->Evaluate(time);
    CHECK_AT(where + " dragged", walked.valid);
    for (const SdfPath &target : {kSlab, kCage, kOther}) {
        CHECK_AT(where + " dragged " + target.GetString(),
                 Moved(dragged, target) == Moved(walked, target));
    }
}

// The op graph of tests/fixtures/solver_checkpoint.usda: a solver's record is
// a FrameMatrix step after its SolverCommit, reading the knee and the commit's
// table. "knee after LegFK" waits on LegFK's commit, and P1's fold -- which
// names LegFK -- waits on it. The values are testRigExecSolverStacking's.
void
TestSolverCheckpointSteps(const std::string &examples)
{
    const std::string where = "solver checkpoint steps";
    const UsdStageRefPtr stage = UsdStage::Open(
        examples + "/../tests/fixtures/solver_checkpoint.usda");
    CHECK_AT(where, stage);
    if (!stage) {
        return;
    }
    const SdfPath p1("/CheckpointAsset/Rig/Movers/P1");
    const SdfPath legFk("/CheckpointAsset/Rig/Stack/LegFK");
    auto evaluator =
        MakeEvaluator(stage, SdfPath("/CheckpointAsset/Rig"),
                      true);
    if (!evaluator) {
        return;
    }
    const RigExecRigPose pose = evaluator->Evaluate(UsdTimeCode(5.0));
    CHECK_AT(where, pose.valid && pose.referenceMismatches == 0);
    const RigExecBakedProgram *program = evaluator->GetBakedProgram();
    CHECK_AT(where, program != nullptr);
    if (!program) {
        return;
    }
    const RigExecBakedProgramImpl &B = program->GetStepGraph();
    CheckFrameRecordSteps(where, B, p1);
    int recordStep = -1;
    int commitStep = -1;
    for (size_t s = 0; s < B.steps.size(); ++s) {
        const RigExecBakedStep &step = B.steps[s];
        if (step.kind == RigExecBakedStepKind::FrameMatrix &&
            B.frameRecords[size_t(step.object)].mover == legFk) {
            recordStep = int(s);
            const auto &record = B.frameRecords[size_t(step.object)];
            CHECK_AT(where, B.paths[size_t(record.slot)] ==
                                SdfPath("/CheckpointAsset/Rig/Joints/Hip/Knee"));
            CHECK_AT(where, record.mover == legFk);
            CHECK_AT(where, record.commit >= 0 && size_t(record.commit) < B.commits.size());
            if (record.commit >= 0 && size_t(record.commit) < B.commits.size()) {
                const auto &commit = B.commits[size_t(record.commit)];
                CHECK_AT(where, record.position >= 0 && size_t(record.position) < commit.slotWrites.size());
                if (record.position >= 0 && size_t(record.position) < commit.slotWrites.size())
                    CHECK_AT(where, record.version == commit.slotWrites[size_t(record.position)]);
            }
        }
    }
    CHECK_AT(where, recordStep >= 0);
    if (recordStep < 0) {
        return;
    }
    const int commit =
        B.frameRecords[size_t(B.steps[size_t(recordStep)].object)].commit;
    CHECK_AT(where, B.commits[size_t(commit)].solverOutput);
    for (size_t s = 0; s < B.steps.size(); ++s) {
        const RigExecBakedStep &step = B.steps[s];
        if (step.object == commit &&
            (step.kind == RigExecBakedStepKind::SolverCommit ||
             step.kind == RigExecBakedStepKind::CommitApply)) {
            commitStep = int(s);
        }
    }
    CHECK_AT(where, commitStep >= 0);
    const std::vector<int> &preds = B.steps[size_t(recordStep)].preds;
    CHECK_AT(where, std::find(preds.begin(), preds.end(), commitStep) !=
                        preds.end());
}

// C1 is a pure positional constraint: Lead rest origin (1,0,0) plus the
// literal tx=3 and ty=1 puts X at (4,1,0). X's rest origin is (0,.5,0),
// so this checkpoint matrix is exactly Translate(4,.5,0), independent of
// the later C2/C3 revisions and of the program's stored matrix.
void TestLiteralCheckpointUnderOverride(const std::string &examples)
{
    const auto stage = UsdStage::Open(examples + "/../tests/fixtures/frame_record_fallbacks.usda");
    CHECK(stage);
    if (!stage) return;
    auto evaluator = MakeEvaluator(stage,kRecordRig,true);
    if (!evaluator) return;
    RigExecValueOverride drag;
    drag.prim = SdfPath("/RecordAsset/Rig/Controls/Lead");
    drag.attribute = TfToken("avars:tx");
    drag.value = VtValue(3.0);
    evaluator->SetInteractiveOverrides({drag});
    const auto pose = evaluator->Evaluate(UsdTimeCode(1));
    CHECK(pose.valid && pose.referenceMismatches == 0);
    const auto &B = evaluator->GetBakedProgram()->GetStepGraph();
    std::map<int,rigExecTest::LiteralFrameFact> facts;
    for (size_t id = 0; id < B.frameRecords.size(); ++id) {
        if (B.frameRecords[id].mover != kRecordC1) continue;
        rigExecTest::LiteralFrameFact fact;
        fact.frame.points = {GfVec3d(4,1,0),GfVec3d(5,1,0),
                             GfVec3d(4,2,0),GfVec3d(4,1,1)};
        fact.matrix.SetTranslate(GfVec3d(4,.5,0));
        fact.hasValue = true;
        facts.emplace(int(id),fact);
    }
    CHECK(facts.size() == 1);
    CHECK(rigExecTest::CheckFrameRecords(&failures,"literal C1 checkpoint",*evaluator,&facts) > 0);
}
// RigExecBakedEvalFrameRecord's gates on a hand-built program, including
// the two exits no authored rig reaches with a catalogued transform
// provider: a geometry-domain constraint's (recordAfter false), whose target
// is a PointBased prim no matrix mover may read, and a non-IK constraint
// with several targets (recordEveryTarget false past its first), which the
// validation refuses.
void
TestFrameRecordGates()
{
    RigExecBakedProgramImpl B;
    B.paths = {SdfPath("/A")};
    B.pathTexts = RigExecBakedSpellPathTexts(B.paths);
    RigExecPointFrame rest;
    rest.points = {GfVec3d(0, 1, 0), GfVec3d(1, 1, 0), GfVec3d(0, 2, 0),
                   GfVec3d(0, 1, 1)};
    B.restFrames = {rest};
    RigExecPointFrame posed;
    posed.points = {GfVec3d(2, 0, 0), GfVec3d(2, 1, 0), GfVec3d(1, 0, 0),
                    GfVec3d(2, 0, 2)};
    RigExecPointFrame invalid = posed;
    invalid.flags = 0;
    B.fin = {posed, invalid};
    B.commits.resize(1);
    RigExecBakedFrameRecord record;
    record.slot = 0;
    record.commit = 0;
    record.target = 0;
    record.version = 0;

    GfMatrix4d expected(1.0);
    CHECK(RigExecPointsToMatrix(rest.points, posed.points, &expected));
    GfMatrix4d matrix(0.0);
    CHECK(RigExecBakedEvalFrameRecord(B, record, &matrix));
    CHECK(matrix == expected);

    // Every exit but the ordinary one records every target; past it only
    // targets[0] is recorded.
    B.commits[0].recordEveryTarget = false;
    CHECK(RigExecBakedEvalFrameRecord(B, record, &matrix));
    record.target = 1;
    CHECK(!RigExecBakedEvalFrameRecord(B, record, &matrix));
    B.commits[0].recordEveryTarget = true;
    CHECK(RigExecBakedEvalFrameRecord(B, record, &matrix));
    record.target = 0;

    // A geometry-domain constraint with its sources records nothing.
    B.commits[0].recordAfter = false;
    CHECK(!RigExecBakedEvalFrameRecord(B, record, &matrix));
    CHECK(matrix == GfMatrix4d(1.0));
    B.commits[0].recordAfter = true;

    // An unusable frame is not recorded.
    record.version = 1;
    CHECK(!RigExecBakedEvalFrameRecord(B, record, &matrix));
    record.version = 0;

    // A rest without a valid frame measures from the identity landmarks; a
    // valid but singular one does not decompose.
    B.restFrames[0].flags = 0;
    CHECK(RigExecBakedEvalFrameRecord(B, record, &matrix));
    GfMatrix4d fromIdentity(1.0);
    CHECK(RigExecPointsToMatrix(RigExecIdentityLandmarks(), posed.points,
                                &fromIdentity));
    CHECK(matrix == fromIdentity);
    B.restFrames[0] = rest;
    B.restFrames[0].points[3] = B.restFrames[0].points[0];
    CHECK(!RigExecBakedEvalFrameRecord(B, record, &matrix));
    B.restFrames[0] = rest;

    // A solver's record is gated on its slot's `present` byte alone: the
    // constraint exit flags do not apply to a solver commit.
    B.commits[0].solverOutput = true;
    B.commits[0].slots = {0};
    B.commits[0].present = {1};
    B.commits[0].recordAfter = false;
    record.target = -1;
    record.position = 0;
    CHECK(RigExecBakedEvalFrameRecord(B, record, &matrix));
    CHECK(matrix == expected);
    B.commits[0].present = {0};
    CHECK(!RigExecBakedEvalFrameRecord(B, record, &matrix));
    record.position = -1;
    B.commits[0].present = {1};
    CHECK(!RigExecBakedEvalFrameRecord(B, record, &matrix));
}

// The solver-checkpoint guard of IsBakeable and Build on hand-built maps: a
// read phase naming a solver is refused unless a batched solver binds that
// joint. A read phase naming a non-solver is not the guard's business.
void
TestSolverCheckpointGuard()
{
    const SdfPath knee("/A/Rig/Joints/Hip/Knee");
    const SdfPath ankle("/A/Rig/Joints/Hip/Knee/Ankle");
    const SdfPath solver("/A/Rig/Stack/LegFK");
    const SdfPath constraint("/A/Rig/Stack/KneeMove");
    const std::map<SdfPath, std::set<SdfPath>> snapshots{
        {knee, {solver, constraint}}};
    const std::map<SdfPath, std::set<SdfPath>> dependencies{{solver, {}}};
    const std::map<SdfPath, std::vector<std::pair<SdfPath, int>>> binds{
        {solver, {{knee, 1}}}};
    const std::map<SdfPath, std::vector<std::pair<SdfPath, int>>> elsewhere{
        {solver, {{ankle, 2}}}};
    const auto missing = [&](const auto &joints, const std::set<SdfPath> &b) {
        return RigExecBakedProgramTesting::SolverCheckpointsWithoutAnOutput(
            snapshots, dependencies, joints, b);
    };
    CHECK(missing(binds, {solver}).empty());
    CHECK(missing(binds, {}) == std::vector<SdfPath>{solver});
    CHECK(missing(elsewhere, {solver}) == std::vector<SdfPath>{solver});
    CHECK(missing(decltype(binds){}, {solver}) ==
          std::vector<SdfPath>{solver});
}

std::string
DefaultResourceDir()
{
#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    return TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR);
#else
    return std::string();
#endif
}

}  // namespace

int
main(int argc, char **argv)
{
    std::string resources = DefaultResourceDir();
    if (argc > 1 && resources.empty()) {
        resources = TfAbsPath(std::string(argv[1]) +
                              "/../plugin/rigExecSchema/resources");
    }
    if (!resources.empty()) {
        PlugRegistry::GetInstance().RegisterPlugins(resources);
    }
    if (argc < 2) {
        std::printf("usage: testRigExecPhaseBindings <examples>\n");
        return 2;
    }
    const std::string examples = argv[1];
    TestPointBindingsCaptureTypedVersions(examples);
    TestPrecedingAcrossChains(examples);
    TestPrecedingOnTheOwnChainReadsTheEnteringPoints(examples);
    TestPrecedingOwnChainFixture(examples);
    TestAChainWithoutABaseFallsThrough(examples);
    TestBlendSamplesBindToVersions();
    TestFrameRecordTypedProvenance(examples);
    TestTheFoldReadsItsRecords(examples);
    TestAPhasedRigSkipsOnRepeat(examples);
    TestSolverCheckpointSteps(examples);
    TestLiteralCheckpointUnderOverride(examples);
    TestFrameRecordGates();
    TestSolverCheckpointGuard();

    rigExecOriginalQuery::Finish();
    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecPhaseBindings: all tests passed\n");
    return 0;
}
