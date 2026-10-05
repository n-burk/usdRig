// Phased point reads bound to chain versions at Build.
//
// A baked point read declared at a phase -- an input's rigExecReadPhase, or a
// blend sample's -- resolves through a RigExecBakedPointsBinding: the chain
// versions the run's phased-read store would have answered from, then the
// resolved input. The store is still filled, so every case here holds the
// binding to it exactly: inside the reader, at the moment it reads, the
// store's answer is captured beside the binding's, because the store at the
// end of the run holds records the reader never saw. Each generation also
// runs in BakedWithParityCheck, so the baked answer is held to the dynamic
// walk's bit for bit.
//
// argv[1] = path to the examples directory.
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/bakedSchedule.h"
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
};

std::vector<BindingRef>
Bindings(const RigExecBakedProgramImpl &B)
{
    std::vector<BindingRef> out;
    const auto add = [&out](const RigExecBakedProgramImpl::GeomRevision &r,
                            const SdfPath &target, size_t chain,
                            bool derived) {
        for (const RigExecBakedPointsBinding &binding : r.pointBindings) {
            out.push_back({&binding, r.moverPath, target, chain, derived,
                           false});
        }
        for (const auto &channel : r.blendChannels) {
            for (const auto &sample : channel.samples) {
                if (sample.pointBinding.id >= 0) {
                    out.push_back({&sample.pointBinding, r.moverPath, target,
                                   chain, derived, true});
                }
            }
        }
    };
    for (size_t c = 0; c < B.chains.size(); ++c) {
        for (const auto &revision : B.chains[c].revisions) {
            add(revision, B.chains[c].target, c, false);
        }
        for (const auto &derived : B.chains[c].derived) {
            add(derived.revision, derived.target, c, true);
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

/// Holds every binding of \p evaluator's program to the store: the answer
/// captured inside the reader (when \p captured), and, for a read of an
/// earlier chain -- complete before any reader of a later one runs -- the
/// store at the end of the run as well.
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
                CHECK_AT(at, capture.storeAnswered == capture.bindingAnswered);
                CHECK_AT(at, capture.store == capture.bound);
                if (capture.bindingAnswered) {
                    ++tally.answered;
                } else {
                    ++tally.tails;
                }
                if (binding.diagnoseMiss) {
                    CHECK_AT(at, Mentions(pose.diagnostics, MissLine(ref)) ==
                                     !capture.storeAnswered);
                } else {
                    CHECK_AT(at, !Mentions(pose.diagnostics, MissLine(ref)));
                }
            }
        }
        const auto source = chainOf.find(binding.input);
        if (source != chainOf.end() && source->second < ref.readerChain) {
            const VtValue *recorded = B.runSnapshots.Lookup(
                binding.input, binding.phase, ref.reader);
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

/// One baked generation with the store captured inside every reader.
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
    // the end-of-run reads and the parity comparison.
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
// store answers only when the reader's own revision on that chain was
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
            CHECK_AT(where, a.read && a.storeAnswered == named);
            CHECK_AT(where, o.read && !o.storeAnswered);
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

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecPhaseBindings: all tests passed\n");
    return 0;
}
