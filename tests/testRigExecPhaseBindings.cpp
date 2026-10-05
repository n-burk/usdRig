// Phased point reads bound to chain versions at Build.
//
// A baked point read declared at a phase -- an input's rigExecReadPhase, or a
// blend sample's -- resolves through a RigExecBakedPointsBinding: the chain
// versions the dynamic walk's phased-read store would answer from, then the
// resolved input. The program keeps no store. Each generation runs in
// BakedWithParityCheck, so the baked answer is held to the dynamic walk's bit
// for bit, and the walk -- which runs after the program over the same
// evaluator -- leaves its store behind: a read of an earlier chain, complete
// before any reader of a later one runs, is held to that store directly. The
// binding's answer is also captured inside the reader and held to its
// diagnostic. An AtPrim read phase on a transform resolves through
// RigExecBakedFrameRecord lists, held to the walk's store the same way.
// Phased rigs run under cones like any other, so a reader the cone leaves
// clean keeps last run's answer (TestAPhasedRigSkipsOnRepeat).
//
// argv[1] = path to the examples directory.
#include "rigExecFrameRecordCheck.h"

#include "rigExec/bakedProgramImpl.h"
#include "rigExec/bakedSchedule.h"
#include "rigExec/bakedTrace.h"
#include "rigExec/rigEvaluator.h"

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
/// walk of this BakedWithParityCheck generation left in the evaluator.
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
    CHECK_AT(where, evaluator.GetEvaluationMode() ==
                        RigExecEvaluationMode::BakedWithParityCheck);
    CHECK_AT(where, B.chainSnapshots != nullptr);
    if (!B.chainSnapshots) {
        return tally;
    }
    const RigExecChainSnapshots &walk = *B.chainSnapshots;
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
        // A point reader's step is a source or reads outside the program, so
        // it runs every run and the cone never leaves one clean.
        CHECK_AT(at, ref.step >= 0 && B.steps[size_t(ref.step)].runSeq != 0);
        if (captured) {
            CHECK_AT(at, size_t(binding.id) < B.pointCaptures.size());
            if (size_t(binding.id) >= B.pointCaptures.size()) {
                continue;
            }
            const RigExecBakedPointCapture &capture =
                B.pointCaptures[size_t(binding.id)];
            if (!capture.read) {
                // Only a reader whose own chain read no base sits a run out.
                CHECK_AT(at, !B.chains[ref.readerChain].haveBase);
            } else {
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
        const auto source = chainOf.find(binding.input);
        if (source != chainOf.end() && source->second < ref.readerChain) {
            const VtValue *recorded =
                walk.Lookup(binding.input, binding.phase, ref.reader);
            const GfVec3f *points = nullptr;
            size_t count = 0;
            const bool answered =
                RigExecBakedResolvePoints(B, binding, &points, &count);
            CHECK_AT(at + " (end of run)", answered == (recorded != nullptr));
            if (answered && recorded) {
                CHECK_AT(at + " (end of run)",
                         recorded->IsHolding<VtVec3fArray>() &&
                             recorded->UncheckedGet<VtVec3fArray>() ==
                                 VtVec3fArray(points, points + count));
            }
        }
    }
    return tally;
}

std::unique_ptr<RigExecRigEvaluator>
MakeEvaluator(const UsdStageRefPtr &stage, const SdfPath &rig,
              RigExecEvaluationMode mode)
{
    auto evaluator = std::make_unique<RigExecRigEvaluator>(stage, rig);
    evaluator->SetEvaluationMode(mode);
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
    CHECK_AT(where, pose.bakedParityMismatches == 0);
    CHECK_AT(where, evaluator->GetBakedGenerationCount() == generations + 1);
    if (pose.bakedParityMismatches != 0) {
        for (const std::string &line : pose.diagnostics) {
            std::printf("  %s\n", line.c_str());
        }
    }
    // The capture is serial-only; the parallel registrations still hold
    // the end-of-run reads to the walk's store and the parity comparison.
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
    CHECK_AT(where, binding.candidates.size() == 1);
    if (assemble < 0 || status < 0 || binding.candidates.size() != 1) {
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
    const RigExecBakedPointVersion &candidate = binding.candidates[0];
    CHECK_AT(where, candidate.chain == cageChain);
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

// 13_ReadPhases as shipped (the lattice reads the cage at `final`), and with
// the lattice's cage phase on the first cage mover and on the cage's Scope:
// cross-chain AtPrim, the form a Scope abbreviates.
void
TestPointBindingsMatchTheStore(const std::string &examples)
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
            stage, kReadPhaseRig, RigExecEvaluationMode::BakedWithParityCheck);
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
            CHECK_AT(where, slab->binding->candidates.size() == 1);
            CHECK_AT(where, slab->binding->finalRead == !*phase);
            CHECK_AT(where, slab->binding->diagnoseMiss);
            CheckSlabAssembleEdges(
                where, evaluator->GetBakedProgram()->GetStepGraph(),
                *slab->binding, *phase ? SdfPath(phase) : SdfPath());
        }
    }
}

// A mover that writes two chains reads the other one at `preceding`. The
// walk's store answers only when the reader's own revision on that chain was
// recorded, which takes a second reader naming it; otherwise the read is the
// authored base, silently. On the reader's own chain `preceding` never
// resolves (TestPrecedingOnTheOwnChainReadsTheBase).
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
            stage, kReadPhaseRig, RigExecEvaluationMode::BakedWithParityCheck);
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
        CHECK_AT(where, own->binding->candidates.empty());
        // Named: the record before Dual's own on the slab's chain, which is
        // SlabLattice's, version 1.
        CHECK_AT(where, across->binding->candidates.size() ==
                            (named ? 1u : 0u));
        if (named && across->binding->candidates.size() == 1) {
            CHECK_AT(where, across->binding->candidates[0].version == 1);
        }
        if (SerialSchedule()) {
            const RigExecBakedProgramImpl &B =
                evaluator->GetBakedProgram()->GetStepGraph();
            const RigExecBakedPointCapture &a =
                B.pointCaptures[size_t(across->binding->id)];
            const RigExecBakedPointCapture &o =
                B.pointCaptures[size_t(own->binding->id)];
            CHECK_AT(where, a.read && a.bindingAnswered == named);
            CHECK_AT(where, o.read && !o.bindingAnswered);
        }
    }
    // Resolving moves Other: the fallback is not the answer.
    CHECK(others.size() == 2 && others[0] != others[1]);
}

// `preceding` on the reader's own chain reads the authored base in every
// backend: the reader's record is made after it reads. A lattice over the
// cage it deforms gives the same cage and slab as one reading the cage at
// `base`, while naming the revision before it (CageTwist) does not.
void
TestPrecedingOnTheOwnChainReadsTheBase(const std::string &examples)
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
        for (const RigExecEvaluationMode mode :
             {RigExecEvaluationMode::BakedWithParityCheck,
              RigExecEvaluationMode::ExecReference}) {
            auto evaluator = MakeEvaluator(stage, kReadPhaseRig, mode);
            if (!evaluator) {
                return;
            }
            Tally tally;
            for (const UsdTimeCode &time : kFrames) {
                const std::string at =
                    where + " at " + std::to_string(time.GetValue());
                const RigExecRigPose pose =
                    mode == RigExecEvaluationMode::ExecReference
                        ? evaluator->Evaluate(time)
                        : EvaluateCaptured(at, evaluator.get(), time,
                                           &tally);
                CHECK_AT(at, pose.valid);
                answers[phase].push_back(Moved(pose, kCage));
                answers[phase].push_back(Moved(pose, kSlab));
            }
        }
    }
    const auto &base = answers["base"];
    CHECK(!base.empty());
    CHECK(answers["preceding"] == base);
    CHECK(answers["/ReadPhaseAsset/Rig/Movers/Cage/CageTwist"] != base);
}

// A chain that reads no base records nothing, so a `final` read of it falls
// to the resolved input with its line -- in the dynamic walk and in the
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
                               RigExecEvaluationMode::BakedWithParityCheck);
    auto reference = MakeEvaluator(stage, kReadPhaseRig,
                                   RigExecEvaluationMode::ExecReference);
    if (!baked || !reference) {
        return;
    }
    const std::string line =
        "diag " + kSlabLattice.GetString() + ": read phase 'final' for " +
        kCage.GetString() + " resolved to nothing; read the authored base";
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
    CHECK(layer->ImportFromString(kBlendRig));
    const UsdStageRefPtr stage = UsdStage::Open(layer);
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath blend("/Asset/Rig/Movers/Blend");
    const SdfPath body("/Asset/Geom/Body.points");
    const SdfPath shape("/Asset/Geom/Shape.points");
    auto evaluator = MakeEvaluator(stage, SdfPath("/Asset/Rig"),
                                   RigExecEvaluationMode::BakedWithParityCheck);
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
/// dynamic walk's phased-read store at the end of the run. Returns the
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
                                   record.slot)) != step.reads.end());
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
                     (step.runSeq != 0) == expected);
        } else if (*folded &&
                   step.kind == RigExecBakedStepKind::InfluenceFold &&
                   moverOf(step) == RecordReader(folded)) {
            foldRan = step.runSeq != 0;
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
TestFrameRecordsMatchTheStore(const std::string &examples)
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
            stage, kRecordRig, RigExecEvaluationMode::BakedWithParityCheck);
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
        // last run. Every reader is held to the walk's store and to the
        // dynamic walk's points.
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
// C1's version of X moves R5 onto R1, while the dynamic walk still answers
// C3's matrix for R5. The validator accepts the rebinding (a version of the
// record's own slot, written before the step), so only the fold's source
// decides R5. Plain Baked, since the dynamic walk is not rebound.
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
        MakeEvaluator(stage, kRecordRig, RigExecEvaluationMode::Baked);
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
        MakeEvaluator(stage, kRecordRig, RigExecEvaluationMode::ExecReference);
    if (reference) {
        const RigExecRigPose walked = reference->Evaluate(UsdTimeCode(8.0));
        CHECK_AT(where, walked.valid);
        CHECK_AT(where, ReadRecordPoints(walked, true).r5 == before.r5);
        CHECK_AT(where, reads.r5 != before.r5);
    }

    B.frameRecords[2].version = bound;
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
    CHECK_AT(where, stage->GetSessionLayer()->ImportFromString(kFreeChain));
    auto baked =
        MakeEvaluator(stage, kReadPhaseRig, RigExecEvaluationMode::Baked);
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
    CHECK_AT(where, B.phasedReads);
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
            if (B.steps[size_t(step)].runSeq != 0) {
                return true;
            }
        }
        return false;
    };
    const auto poseRan = [&B]() {
        size_t count = 0;
        for (const RigExecBakedStep &step : B.steps) {
            count += step.runSeq != 0 &&
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
    CHECK_AT(where + " repeat", repeat.moverGraphRevisionsExecuted == 0);
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
                                   RigExecEvaluationMode::ExecReference);
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
    TestPointBindingsMatchTheStore(examples);
    TestPrecedingAcrossChains(examples);
    TestPrecedingOnTheOwnChainReadsTheBase(examples);
    TestAChainWithoutABaseFallsThrough(examples);
    TestBlendSamplesBindToVersions();
    TestFrameRecordsMatchTheStore(examples);
    TestTheFoldReadsItsRecords(examples);
    TestAPhasedRigSkipsOnRepeat(examples);
    TestFrameRecordGates();

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecPhaseBindings: all tests passed\n");
    return 0;
}
