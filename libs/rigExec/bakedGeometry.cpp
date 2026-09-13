//
// The baked program's geometry half: the chain/revision bake and the frame
// path that runs those revisions and the derived maintenance behind them.
//
// Split out of bakedProgram.cpp for the reason bakedPose.cpp was: a domain's
// bake and its frame path have to agree about what was captured and what is
// re-read. The evaluator is not visible here -- the compiled chains arrive
// restated as RigExecBakedChainSpec, and the caches the frame path shares
// with the dynamic walk were captured into RigExecBakedProgramImpl at Build.
//
#include "bakedProgramImpl.h"

#include "moverGraph.h"
#include "rigEvaluator.h"
#include "types.h"

#include "rigExecMath/envelope.h"
#include "rigExecMath/geometryKernels.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/attributeQuery.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

// The two names a step body needs a TOKEN for rather than a comparison: the
// status a failed revision publishes, and the attribute the defaultWeight
// diagnostic re-reads. Hoisted because TfToken(const char *) takes the token
// registry's spin lock on every construction, and a step body may take no
// lock (docs/baked-step-graph.md §2).
TF_DEFINE_PRIVATE_TOKENS(
    _tokens,
    ((moverFailed, "moverFailed"))
    ((defaultWeight, "inputs:defaultWeight"))
);

namespace rigExec {

// ---------------------------------------------------------------------------
// Bake.
// ---------------------------------------------------------------------------

void
RigExecBakedBuildGeometry(RigExecBakedBuildContext *ctx,
                          const std::vector<RigExecBakedChainSpec> &chains)
{
    RigExecBakedProgramImpl &B = *ctx->program;
    auto refuse = [&](const std::string &what, const SdfPath &where) {
        ctx->Refuse(what, where);
    };
    auto slotOf = [&](const SdfPath &path) { return ctx->SlotOf(path); };

    auto bakeRevision = [&](const RigExecBakedRevisionSpec &r) {
        RigExecBakedProgramImpl::GeomRevision out;
        out.moverPath = r.moverPath;
        out.target = r.target;
        out.moverPrim = B.stage->GetPrimAtPath(r.moverPath);
        out.op = r.op;
        out.binding = r.binding;
        out.finalPhase = r.transformFinalPhase;
        out.skinTopologyFixed = r.skinTopologyFixed;
        out.snapshotAfter = r.snapshotAfter;
        // A declared phase is the only thing that reads the snapshot store,
        // and the pose half fills its half of that store only when one
        // exists. All THREE consumers the dynamic walk has are counted: the
        // per-input phases, an AtPrim transform (answered out of the same
        // store), and a blend sample whose points carry a phase -- that last
        // one reads chain-point records, which the geometry half writes
        // ungated, but leaving it out would make this flag mean "some phases"
        // rather than "some phased read", which is what the next consumer
        // will read it as.
        bool phased = !r.binding.phases.empty() ||
                      r.binding.transformPhase.kind ==
                          RigExecReadPhaseKind::AtPrim;
        for (const auto &[input, samples] : r.binding.blendSamples) {
            for (const RigExecBlendSampleBinding &sample : samples) {
                phased = phased || !sample.phase.IsBase();
            }
        }
        if (phased) {
            B.phasedReads = true;
        }
        if (!out.moverPrim) {
            refuse("mover prim is missing", r.moverPath);
        }
        // The packet is assembled live from this prim every frame through
        // the generation's resolved inputs: nothing of it is captured, so a
        // value edit needs no rebuild and an override places itself. The
        // layout arrays are still NAMED, because bakeability judged them --
        // connecting one is a resync, and the judgement was that none is
        // connected. They do not belong in `rebuild` even though the layout
        // is no longer read per frame: it is held in the evaluator's skin
        // topology cache, which every notice clears, so a weight-paint edit
        // reaches the next generation through the re-read rather than
        // through a rebuild of this program.
        B.prims.insert(r.moverPath);
        B.resolvedRoutedPrims.insert(r.moverPath);
        for (const char *name : {"rigExec:jointIndices", "rigExec:jointWeights",
                                 "rigExec:elementSize",
                                 "rigExec:skinningMethod"}) {
            B.named.insert(r.moverPath.AppendProperty(TfToken(name)));
        }
        if (!r.binding.transform.IsEmpty()) {
            out.transformSlot = slotOf(r.binding.transform);
            if (out.transformSlot < 0) {
                refuse("mover transform provider is not a pose provider",
                       r.binding.transform);
            }
        }
        for (const SdfPath &influence : r.binding.influences) {
            const int slot = slotOf(influence);
            if (slot < 0) refuse("skin influence is not a provider", influence);
            out.influenceSlots.push_back(slot);
        }
        return out;
    };
    for (const RigExecBakedChainSpec &spec : chains) {
        const SdfPath &target = spec.target;
        RigExecBakedProgramImpl::GeomChain chain;
        chain.target = target;
        // The base points are read per frame through a query, so only the
        // query's own validity depends on this surviving a resync.
        B.prims.insert(target.GetPrimPath());
        B.named.insert(target);
        if (const UsdAttribute a = B.stage->GetAttributeAtPath(target)) {
            chain.baseQuery = UsdAttributeQuery(a);
        } else {
            refuse("chain target has no attribute", target);
        }
        for (const RigExecBakedRevisionSpec &revision : spec.revisions) {
            chain.revisions.push_back(bakeRevision(revision));
        }
        for (const RigExecBakedRevisionSpec &derived : spec.derived) {
            RigExecBakedProgramImpl::GeomChain::Derived d;
            d.target = derived.target;
            B.prims.insert(derived.target.GetPrimPath());
            B.named.insert(derived.target);
            if (const UsdAttribute a =
                    B.stage->GetAttributeAtPath(derived.target)) {
                d.baseQuery = UsdAttributeQuery(a);
            } else {
                refuse("derived target has no attribute", derived.target);
            }
            d.revision = bakeRevision(derived);
            chain.derived.push_back(std::move(d));
        }
        B.chains.push_back(std::move(chain));
    }
}


// ---------------------------------------------------------------------------
// Build: the geometry half of the program, in program order.
// ---------------------------------------------------------------------------

namespace {

RigExecBakedStep &
AddGeometryStep(RigExecBakedProgramImpl *program, RigExecBakedStepKind kind,
                int object, int part = -1)
{
    RigExecBakedStep step;
    step.kind = kind;
    step.object = object;
    step.part = part;
    step.maxDiagnostics = 0;
    program->steps.push_back(std::move(step));
    return program->steps.back();
}

/// Declares the matrices \p revision folds, which are the slots the pose half
/// published for it: final or base per the revision's phase, exactly the set
/// the ProviderMatrix steps were created for.
void
DeclareMatrixReads(const RigExecBakedProgramImpl::GeomRevision &revision,
                   RigExecBakedStep *step)
{
    const RigExecBakedSlotDomain domain =
        revision.finalPhase ? RigExecBakedSlotDomain::FinalMatrix
                            : RigExecBakedSlotDomain::BaseMatrix;
    if (revision.transformSlot >= 0) {
        step->reads.push_back(
            RigExecBakedOne(domain, revision.transformSlot));
    }
    for (const int slot : revision.influenceSlots) {
        if (slot >= 0) {
            step->reads.push_back(RigExecBakedOne(domain, slot));
        }
    }
}

}  // namespace

void
RigExecBakedBuildGeometrySteps(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    B.chainRevisionBegin.assign(B.chains.size(), 0);
    B.chainRevisionEnd.assign(B.chains.size(), 0);
    for (size_t c = 0; c < B.chains.size(); ++c) {
        RigExecBakedProgramImpl::GeomChain &chain = B.chains[c];
        // Revision ids are handed out chain by chain, so one chain's
        // revisions are a contiguous RANGE: "the buffers this chain has
        // produced so far" -- which is what a revision reads its preceding
        // points from and what the status sweep reads at the end -- is then
        // one declared range rather than a list.
        B.chainRevisionBegin[c] = int(B.revisionIndex.size());
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            B.revisionIndex.emplace_back(int(c), int(r));
        }
        B.chainRevisionEnd[c] = int(B.revisionIndex.size());
        const int first = B.chainRevisionBegin[c];
        const int last = B.chainRevisionEnd[c];
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            RigExecBakedProgramImpl::GeomRevision &revision =
                chain.revisions[r];
            const int id = first + int(r);
            revision.influences.assign(revision.influenceSlots.size(),
                                       GfMatrix4d(1.0));
            {
                RigExecBakedStep &fold = AddGeometryStep(
                    &B, RigExecBakedStepKind::InfluenceFold, id);
                DeclareMatrixReads(revision, &fold);
                fold.writes.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::RevisionTransforms, id));
            }
            {
                RigExecBakedStep &assemble = AddGeometryStep(
                    &B, RigExecBakedStepKind::RevisionStatic, id);
                // One line per declared phase that resolved to nothing, plus
                // the one the defaultWeight check can emit.
                assemble.maxDiagnostics = revision.binding.phases.size() + 1;
                assemble.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::RevisionTransforms, id));
                assemble.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::ChainBase, int(c)));
                if (r == 0) {
                    // The chain's own dirtiness is where the sticky bit
                    // starts, and the prologue decided it.
                } else {
                    assemble.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::ChainDirty, id - 1));
                }
                if (!revision.binding.phases.empty()) {
                    assemble.reads.push_back(RigExecBakedRange(
                        RigExecBakedSlotDomain::Snapshots, 0,
                        int(B.steps.size()) - 1));
                }
                assemble.writes.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::RevisionPacket, id));
                assemble.writes.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::ChainDirty, id));
            }
            {
                // One chunk over the whole range in this stage: the vertex
                // partition and the per-chunk influence key arrive with the
                // parallel executor, and a whole-range chunk is the
                // degenerate case of the same step.
                RigExecBakedStep &chunk = AddGeometryStep(
                    &B, RigExecBakedStepKind::RevisionChunk, id, 0);
                chunk.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::RevisionPacket, id));
                chunk.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::ChainBase, int(c)));
                // Which earlier buffer holds the preceding points is a
                // runtime indirection, so the declaration is the upper
                // bound: every buffer this chain has filled before it.
                if (id > first) {
                    chunk.reads.push_back(RigExecBakedRange(
                        RigExecBakedSlotDomain::RevisionOut, first, id));
                }
                chunk.writes.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::RevisionOut, id));
            }
            {
                RigExecBakedStep &fuse = AddGeometryStep(
                    &B, RigExecBakedStepKind::RevisionFuse, id);
                fuse.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::RevisionPacket, id));
                fuse.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::ChainBase, int(c)));
                fuse.reads.push_back(RigExecBakedRange(
                    RigExecBakedSlotDomain::RevisionOut, first, id + 1));
                if (id > first) {
                    fuse.reads.push_back(RigExecBakedRange(
                        RigExecBakedSlotDomain::RevisionDone, first, id));
                }
                fuse.writes.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::RevisionDone, id));
                if (revision.snapshotAfter) {
                    fuse.writes.push_back(
                        RigExecBakedOne(RigExecBakedSlotDomain::Snapshots,
                                        int(B.steps.size()) - 1));
                }
            }
        }
        {
            RigExecBakedStep &status =
                AddGeometryStep(&B, RigExecBakedStepKind::ChainStatus, int(c));
            // One MoverFailed line per revision of the chain, which is the
            // sweep's whole output.
            status.maxDiagnostics = chain.revisions.size();
            status.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::ChainBase, int(c)));
            if (last > first) {
                status.reads.push_back(RigExecBakedRange(
                    RigExecBakedSlotDomain::RevisionDone, first, last));
                status.reads.push_back(RigExecBakedRange(
                    RigExecBakedSlotDomain::RevisionOut, first, last));
            }
            status.writes.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::ChainPoints, int(c)));
            status.writes.push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::Snapshots, int(B.steps.size()) - 1));
        }
        for (size_t d = 0; d < chain.derived.size(); ++d) {
            RigExecBakedProgramImpl::GeomChain::Derived &derived =
                chain.derived[d];
            const int id = int(B.derivedIndex.size());
            B.derivedIndex.emplace_back(int(c), int(d));
            derived.revision.influences.assign(
                derived.revision.influenceSlots.size(), GfMatrix4d(1.0));
            RigExecBakedStep &step =
                AddGeometryStep(&B, RigExecBakedStepKind::Derived, id);
            step.maxDiagnostics =
                derived.revision.binding.phases.size() + 1;
            // The chain's FINAL points, deliberately: recomputeNormals and
            // recomputeExtent describe the geometry as published. The
            // target's own authored base is prologue state, read beside the
            // chain's.
            step.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::ChainPoints, int(c)));
            step.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::ChainBase, int(c)));
            // A derived revision's binding carries phases like any other, and
            // its assemble reads the run's store for them -- so it declares
            // the store the same way a chain revision does: every step before
            // it, because which of them recorded what is a runtime answer.
            if (!derived.revision.binding.phases.empty()) {
                step.reads.push_back(RigExecBakedRange(
                    RigExecBakedSlotDomain::Snapshots, 0,
                    int(B.steps.size()) - 1));
            }
            DeclareMatrixReads(derived.revision, &step);
            step.writes.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::DerivedOut, id));
        }
    }
}

// ---------------------------------------------------------------------------
// One frame, geometry half.
//
// The packet is assembled by the same RigExecAssembleParameters the dynamic
// path calls and the kernel is the same shared kernel; only the VdfNetwork
// around them is baked away, so the accounting it performed -- a revision
// runs when its inputs changed, and not otherwise -- is performed here
// instead, by the four steps a revision is made of.
// ---------------------------------------------------------------------------

namespace {

/// The points the chain holds BEFORE revision \p r.
///
/// Not a buffer of its own: `currentSource` names the revision whose output
/// buffer last held the running value, which is -1 for the authored base. It
/// is the indirection that replaces today's `revision.output = current` copy
/// of a revision that applied nothing.
void
PointsBefore(const RigExecBakedProgramImpl::GeomChain &chain, size_t r,
             const GfVec3f **points, size_t *count)
{
    const int source =
        r == 0 ? -1 : chain.revisions[r - 1].currentSource;
    if (source < 0) {
        *points = chain.lastBase.cdata();
        *count = chain.lastBase.size();
        return;
    }
    *points = chain.revisions[size_t(source)].output.data();
    *count = chain.revisions[size_t(source)].output.size();
}

/// The points the chain holds AFTER revision \p r.
void
PointsAfter(const RigExecBakedProgramImpl::GeomChain &chain, size_t r,
            const GfVec3f **points, size_t *count)
{
    const int source = chain.revisions[r].currentSource;
    if (source < 0) {
        *points = chain.lastBase.cdata();
        *count = chain.lastBase.size();
        return;
    }
    *points = chain.revisions[size_t(source)].output.data();
    *count = chain.revisions[size_t(source)].output.size();
}

/// The influence matrices of \p revision, out of the tables the pose half's
/// ProviderMatrix steps published.
void
FoldInfluences(const RigExecBakedProgramImpl &B,
               RigExecBakedProgramImpl::GeomRevision *revision)
{
    revision->haveTransform = revision->transformSlot >= 0;
    if (revision->haveTransform) {
        revision->transform =
            revision->finalPhase
                ? B.finalMatrix[size_t(revision->transformSlot)]
                : B.baseMatrix[size_t(revision->transformSlot)];
    }
    // A geometry-domain constraint's delta, if one was measured for this
    // mover: the pose walk solved the constraint and stashed the map from the
    // target's authored transform to the solved one, and THIS is the Matrix
    // revision that same constraint contributes. Find-guarded, and here
    // rather than in the assemble because the transform table is what this
    // step declares -- the assemble only READS it, and a step that writes a
    // slot it declared as a read is the declaration the executor trusts
    // being wrong.
    //
    // The dynamic path applies it before the final-phase substitution rather
    // than after. The two orders can only disagree for a revision that has
    // both a bound transform provider and a delta, which cannot arise: a
    // geometry-domain constraint's binding.transform is empty -- that is what
    // makes the delta the only source of the matrix.
    if (const auto delta = B.constraintDeltas.find(revision->moverPath);
        delta != B.constraintDeltas.end()) {
        revision->transform = delta->second;
        revision->haveTransform = true;
    }
    for (size_t k = 0; k < revision->influenceSlots.size(); ++k) {
        const size_t slot = size_t(revision->influenceSlots[k]);
        revision->influences[k] = revision->finalPhase ? B.finalMatrix[slot]
                                                       : B.baseMatrix[slot];
    }
}

// WHICH points a revision is assembled against, stated once because the two
// callers below pass different ones and the difference is invisible until an
// operator reads them:
//
//  * a CHAIN revision gets the chain's AUTHORED base -- the attribute as the
//    stage holds it -- for every revision of the chain, not the running
//    value. The dynamic walk reads the base once per target and hands that
//    same array to every revision, so a lattice's rest points and a
//    volumeCorrect's reference volume are measured against the mesh as
//    authored however deep in the chain they sit.
//  * a DERIVED revision gets the chain's FINAL points, which is what it is
//    for: recomputeNormals and recomputeExtent take auxPoints = basePoints
//    and must describe the geometry as published.
//
// Taken as a range rather than a vector because the two callers hold the
// points in different containers and RigExecProviderValues copies them
// anyway.
RigExecMoverParameters
AssembleRevision(RigExecBakedProgramImpl &B,
                 RigExecBakedProgramImpl::GeomRevision *revision,
                 const GfVec3f *basePoints, size_t basePointCount,
                 UsdTimeCode time, RigExecBakedStep *step)
{
    const RigExecResolvedInputs &R = *B.resolvedInputs;
    RigExecProviderValues values;
    values.resolved = &R;
    // One overlay per REVISION: the generation-wide resolved inputs, plus
    // whatever this revision's declared phases resolve to out of the run's
    // snapshot store. The assembler reads inputs by path and never learns a
    // phase exists, which is what lets a phase apply to any input. Built only
    // for a revision that declares one, and owned by that revision, because
    // one buffer shared by the walk is state two steps could be inside at
    // once.
    if (!revision->binding.phases.empty()) {
        revision->revisionInputs = R;
        for (const auto &[inputPath, phase] : revision->binding.phases) {
            if (const VtValue *recorded = B.runSnapshots.Lookup(
                    inputPath, phase, revision->moverPath)) {
                revision->revisionInputs.SetProperty(inputPath, *recorded);
            } else if (phase.kind != RigExecReadPhaseKind::Preceding) {
                // Preceding falling through to the stage is correct: the
                // reader is the chain's first revision, so its preceding
                // value IS the base. Anything else means the phase named
                // something that produced nothing.
                step->diagnostics.push_back(
                    "diag " + revision->moverPath.GetString() +
                    ": read phase '" + phase.GetAsString() + "' for " +
                    inputPath.GetString() +
                    " resolved to nothing; read the authored base");
            }
        }
        values.resolved = &revision->revisionInputs;
    }
    // The fold decided whether there is a matrix at all -- a bound transform
    // provider, or a geometry-domain constraint's delta -- and wrote it.
    if (revision->haveTransform) {
        values.transform = &revision->transform;
    }
    if (!revision->influences.empty()) {
        values.influenceTransforms = &revision->influences;
    }
    values.basePoints.assign(basePoints, basePoints + basePointCount);
    // Epoch-fixed layouts were resolved in the PROLOGUE, through the same
    // evaluator cache the dynamic path's assembly resolves through -- so the
    // two paths cannot hold different arrays, and no step body takes the
    // cache's lock.
    if (revision->topologyResolved) {
        values.skinTopology = &revision->topology;
    }
    return RigExecAssembleParameters(revision->moverPrim, revision->op,
                                     revision->binding, values, time);
}

/// The envelope is applied exactly once, against the preceding revision.
/// Derived maintenance is the last caller: the chain's own revisions run
/// through RigExecRunRevisionKernel, which owns the same blend, and this copy
/// goes when the derived path joins it.
bool
BlendEnvelope(const RigExecMoverParameters &parameters,
              const std::vector<GfVec3f> &preceding,
              std::vector<GfVec3f> *result)
{
    if (result->size() != preceding.size()) return false;
    std::vector<float> envelope;
    if (!parameters.weights.ResolveAll(result->size(), &envelope)) {
        return false;
    }
    for (size_t i = 0; i < result->size(); ++i) {
        (*result)[i] =
            RigExecBlendEnvelope(preceding[i], (*result)[i], envelope[i]);
    }
    return true;
}

}  // namespace

void
RigExecBakedRunGeometryPrologue(RigExecBakedProgramImpl *program,
                                UsdTimeCode time, RigExecRigPose *pose)
{
    RigExecBakedProgramImpl &B = *program;
    RIGEXEC_PROFILE_SCOPE_CAT(*B.profiler, "BakedChainBase", "geometry");
    // Everything a frame reads off the stage for a chain, plus the one lock a
    // frame takes. Building the geometry state is reported once per node and
    // once per schedule, not once per program: a program rebuilt over state
    // that survived (AdoptGeometryStateFrom) has nothing to report, which is
    // what the dynamic path says about the same edit. Counted here, because a
    // chain can lose its state at the head of a frame -- a point count that
    // moved with time -- and that is a node the dynamic path adds again too.
    const auto resetRevision =
        [](RigExecBakedProgramImpl::GeomRevision *revision) {
        revision->created = true;
        revision->ran = false;
        revision->output.clear();
        revision->currentSource = -1;
        revision->lastParameters = RigExecMoverParameters();
        revision->lastStatus = RigExecMoverStatus();
    };
    const auto resolveTopology =
        [&B, time](RigExecBakedProgramImpl::GeomRevision *revision) {
        if (!revision->skinTopologyFixed) {
            return;
        }
        revision->topology = RigExecResolveSkinTopology(
            revision->moverPrim, revision->influenceSlots.size(), time,
            B.resolvedInputs, B.skinTopologies);
        revision->topologyResolved = true;
    };
    for (RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        VtVec3fArray basePoints;
        chain.haveBase =
            chain.baseQuery.IsValid() && chain.baseQuery.Get(&basePoints, time);
        if (!chain.haveBase) {
            // The target's points do not read at this time at all; the
            // dynamic path drives nothing for it and neither does the
            // program, counters included.
            for (RigExecBakedProgramImpl::GeomChain::Derived &derived :
                     chain.derived) {
                derived.haveBase = false;
            }
            continue;
        }
        if (chain.haveResult && basePoints.size() != chain.lastBase.size()) {
            // A time-varying point count replaces THIS target's graph in the
            // dynamic path and leaves every other target's standing. Same
            // here: the cached run describes a different mesh, so it is
            // dropped and the chain's nodes are reported as created -- which
            // is what the dynamic walk reports for the nodes it has to add
            // again. Handing the whole generation back instead would degrade
            // every other chain of the rig for one mesh whose vertex count is
            // keyed.
            chain.haveResult = false;
            chain.result = VtVec3fArray();
            chain.scheduleDirty = true;
            for (RigExecBakedProgramImpl::GeomRevision &revision :
                     chain.revisions) {
                resetRevision(&revision);
            }
        }
        for (RigExecBakedProgramImpl::GeomRevision &revision :
                 chain.revisions) {
            if (revision.created) {
                ++pose->moverGraphRevisionsCreated;
                revision.created = false;
            }
        }
        if (chain.scheduleDirty && !chain.revisions.empty()) {
            ++pose->moverGraphSchedulesBuilt;
        }
        chain.scheduleDirty = false;
        chain.baseDirty = !chain.haveResult || basePoints != chain.lastBase;
        chain.lastBase = basePoints;
        for (RigExecBakedProgramImpl::GeomRevision &revision :
                 chain.revisions) {
            resolveTopology(&revision);
        }
        for (RigExecBakedProgramImpl::GeomChain::Derived &derived :
                 chain.derived) {
            VtVec3fArray derivedBase;
            derived.haveBase = derived.baseQuery.IsValid() &&
                               derived.baseQuery.Get(&derivedBase, time);
            if (!derived.haveBase) {
                continue;
            }
            if (derived.haveResult &&
                derivedBase.size() != derived.lastBase.size()) {
                // As above, and for the same reason: the dynamic walk
                // replaces this derived target's graph alone.
                derived.haveResult = false;
                derived.result = VtVec3fArray();
                resetRevision(&derived.revision);
            }
            // A derived target is one node with one schedule of its own,
            // created together, exactly as the dynamic walk creates them.
            if (derived.revision.created) {
                ++pose->moverGraphRevisionsCreated;
                ++pose->moverGraphSchedulesBuilt;
                derived.revision.created = false;
            }
            derived.baseDirty =
                !derived.haveResult || derivedBase != derived.lastBase;
            derived.lastBase = derivedBase;
            resolveTopology(&derived.revision);
        }
    }
}

void
RigExecBakedRunGeometryStep(RigExecBakedProgramImpl *program,
                            RigExecBakedStep *step, UsdTimeCode time)
{
    RigExecBakedProgramImpl &B = *program;
    const RigExecResolvedInputs &R = *B.resolvedInputs;
    if (step->kind == RigExecBakedStepKind::Derived) {
        const auto &[chainIndex, derivedIndex] =
            B.derivedIndex[size_t(step->object)];
        RigExecBakedProgramImpl::GeomChain &chain =
            B.chains[size_t(chainIndex)];
        RigExecBakedProgramImpl::GeomChain::Derived &derived =
            chain.derived[size_t(derivedIndex)];
        if (!chain.haveBase || !derived.haveBase) {
            return;
        }
        RigExecBakedProgramImpl::GeomRevision &revision = derived.revision;
        FoldInfluences(B, &revision);
        const RigExecMoverParameters parameters = AssembleRevision(
            B, &revision, chain.result.cdata(), chain.result.size(), time,
            step);
        const RigExecMoverStatus status =
            RigExecStatusForParameters(parameters, revision.moverPath);
        step->counters.revisionsBuilt = 1;
        if (derived.baseDirty || !revision.ran ||
            parameters != revision.lastParameters ||
            status != revision.lastStatus) {
            step->counters.revisionsExecuted = 1;
            const std::vector<GfVec3f> preceding(derived.lastBase.begin(),
                                                 derived.lastBase.end());
            std::vector<GfVec3f> values;
            const bool wanted =
                status.AllowsApply() && parameters.valid &&
                parameters.kind ==
                    (revision.op == RigExecRevisionOp::RecomputeExtent
                         ? "recomputeExtent"
                         : "recomputeNormals");
            if (wanted) {
                values = revision.op == RigExecRevisionOp::RecomputeExtent
                             ? RigExecComputeExtent(parameters.auxPoints,
                                                    parameters.widths)
                             : RigExecComputeVertexNormals(
                                   parameters.auxPoints,
                                   parameters.topologyCounts,
                                   parameters.topologyIndices);
            }
            bool applied =
                wanted && !values.empty() &&
                (revision.op != RigExecRevisionOp::RecomputeExtent ||
                 values.size() == 2) &&
                // The derived property keeps its authored cardinality: an
                // in-place write cannot resize it.
                values.size() == preceding.size() &&
                BlendEnvelope(parameters, preceding, &values);
            revision.resultStatus = status.state;
            if (!applied) {
                values = preceding;
                if (status.AllowsApply()) {
                    revision.resultStatus = _tokens->moverFailed;
                }
            }
            revision.output = values;
            revision.lastParameters = parameters;
            revision.lastStatus = status;
            revision.ran = true;
        }
        if (revision.resultStatus == "moverFailed") {
            step->diagnostics.push_back(
                "MoverFailed " + derived.target.GetString() +
                ": derived geometry input/cardinality validation failed");
        }
        derived.spare.resize(revision.output.size());
        std::copy(revision.output.begin(), revision.output.end(),
                  derived.spare.data());
        derived.result.swap(derived.spare);
        derived.haveResult = true;
        step->counters.chainsBuilt = 1;
        return;
    }

    // A ChainStatus step is about a CHAIN and every other one about a
    // revision, so the two payloads index different tables.
    if (step->kind == RigExecBakedStepKind::ChainStatus) {
        RigExecBakedProgramImpl::GeomChain &chain =
            B.chains[size_t(step->object)];
        if (!chain.haveBase) {
            return;
        }
        // The per-chain status sweep, over the status each revision LAST
        // published -- which outlives an evaluation it did not take part in,
        // and so does its diagnostic.
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
                 chain.revisions) {
            if (revision.resultStatus == "moverFailed") {
                step->diagnostics.push_back(
                    "MoverFailed " + revision.moverPath.GetString() +
                    ": execution rejected its inputs; revision passed "
                    "through");
            }
        }
        const GfVec3f *points = chain.lastBase.cdata();
        size_t count = chain.lastBase.size();
        if (!chain.revisions.empty()) {
            PointsAfter(chain, chain.revisions.size() - 1, &points, &count);
        }
        // Double-buffered: publication is a refcount bump for the consumer
        // and the array one may still hold from last frame is never the one
        // being written.
        chain.spare.resize(count);
        std::copy(points, points + count, chain.spare.data());
        chain.result.swap(chain.spare);
        chain.haveResult = true;
        // `final` costs nothing extra: this is the value the chain
        // publishes.
        step->snapshots.RecordFinal(chain.target, VtValue(chain.result));
        step->counters.chainsBuilt = 1;
        return;
    }

    const auto &[chainIndex, revisionIndex] =
        B.revisionIndex[size_t(step->object)];
    RigExecBakedProgramImpl::GeomChain &chain = B.chains[size_t(chainIndex)];
    if (!chain.haveBase) {
        return;
    }
    RigExecBakedProgramImpl::GeomRevision &revision =
        chain.revisions[size_t(revisionIndex)];
    switch (step->kind) {
    case RigExecBakedStepKind::InfluenceFold:
        FoldInfluences(B, &revision);
        return;

    case RigExecBakedStepKind::RevisionStatic: {
        revision.parameters =
            AssembleRevision(B, &revision, chain.lastBase.cdata(),
                             chain.lastBase.size(), time, step);
        if (revision.parameters.enabled && !revision.parameters.valid) {
            float scalar = 1.0f;
            if (const UsdAttribute a = revision.moverPrim.GetAttribute(
                    _tokens->defaultWeight)) {
                R.GetAttribute(a, time, &scalar);
            }
            if (!std::isfinite(scalar) || scalar < 0.0f || scalar > 1.0f) {
                step->diagnostics.push_back(
                    "MoverFailed " + revision.moverPath.GetString() +
                    ": inputs:defaultWeight must be finite and in "
                    "[0, 1]; revision passed through");
            }
        }
        revision.status =
            RigExecStatusForParameters(revision.parameters,
                                       revision.moverPath);
        step->counters.revisionsBuilt = 1;
        // Whether the revision has to run at all, by VALUE and never by
        // dirtiness: a control dragged back to where it started leaves an
        // identical packet, and the VdfNetwork does not re-execute for one.
        // The sticky chain bit is the predecessor's own answer, because a
        // revision that executed makes every later one execute.
        const bool chainDirty =
            revisionIndex == 0
                ? chain.baseDirty
                : chain.revisions[size_t(revisionIndex) - 1].executed;
        revision.executed = chainDirty || !revision.ran ||
                            revision.parameters != revision.lastParameters ||
                            revision.status != revision.lastStatus;
        step->counters.revisionsExecuted = revision.executed ? 1 : 0;
        return;
    }

    case RigExecBakedStepKind::RevisionChunk: {
        revision.chunkOk = false;
        if (!revision.executed || !revision.status.AllowsApply()) {
            return;
        }
        const GfVec3f *points = nullptr;
        size_t count = 0;
        PointsBefore(chain, size_t(revisionIndex), &points, &count);
        // The revision's OWN buffer, seeded from the preceding one: there is
        // no `scratch = current` and no `revision.output = current` afterwards
        // -- the fuse decides which buffer the chain's running value is in
        // rather than copying one into another.
        revision.output.assign(points, points + count);
        // Not a second dispatch that mirrors _RevisionNode::Compute -- the
        // same function the node calls. The packet check, the full-strength
        // fast path, the kernel and the "apply once" blend all live in
        // RigExecRunRevisionKernel, so an operation cannot mean one thing
        // here and another there.
        revision.chunkOk =
            RigExecRunRevisionKernel(revision.op, revision.parameters,
                                     &revision.output,
                                     /*controlFrames=*/nullptr);
        return;
    }

    case RigExecBakedStepKind::RevisionFuse: {
        if (revision.executed) {
            revision.resultStatus = revision.status.state;
            if (revision.chunkOk) {
                revision.currentSource = int(revisionIndex);
            } else {
                // Nothing was applied, so the chain's running value stays
                // where it was -- an indirection, not a copy. A chunk that
                // ran and produced points is discarded with it.
                revision.currentSource =
                    revisionIndex == 0
                        ? -1
                        : chain.revisions[size_t(revisionIndex) - 1]
                              .currentSource;
                if (revision.status.AllowsApply()) {
                    revision.resultStatus = _tokens->moverFailed;
                }
            }
            revision.lastParameters = revision.parameters;
            revision.lastStatus = revision.status;
            revision.ran = true;
        }
        // Snapshot only where a phased read named this revision, which is the
        // whole cost of the feature for a rig that uses it and nothing at all
        // for one that does not.
        if (revision.snapshotAfter) {
            const GfVec3f *points = nullptr;
            size_t count = 0;
            PointsAfter(chain, size_t(revisionIndex), &points, &count);
            step->snapshots.Record(
                chain.target, revision.moverPath,
                VtValue(VtVec3fArray(points, points + count)));
        }
        return;
    }

    default:
        return;
    }
}

void
RigExecBakedPublishGeometry(RigExecBakedProgramImpl *program,
                            RigExecRigPose *pose)
{
    RigExecBakedProgramImpl &B = *program;
    // Chain by chain, in chain order, and inside a chain exactly where the
    // straight line put each line: every revision's assemble diagnostics
    // first, then the status sweep's, then the chain's points, then each
    // derived target's diagnostic and its points. Program order is that
    // order, so this is one pass over the steps.
    for (const RigExecBakedStep &step : B.steps) {
        if (!RigExecBakedIsGeometryStep(step.kind)) {
            continue;
        }
        for (const std::string &diagnostic : step.diagnostics) {
            pose->diagnostics.push_back(diagnostic);
        }
        if (step.kind == RigExecBakedStepKind::ChainStatus) {
            const RigExecBakedProgramImpl::GeomChain &chain =
                B.chains[size_t(step.object)];
            if (chain.haveBase) {
                pose->movedProperties[chain.target] = VtValue(chain.result);
            }
        } else if (step.kind == RigExecBakedStepKind::Derived) {
            const auto &[chainIndex, derivedIndex] =
                B.derivedIndex[size_t(step.object)];
            const RigExecBakedProgramImpl::GeomChain &chain =
                B.chains[size_t(chainIndex)];
            const RigExecBakedProgramImpl::GeomChain::Derived &derived =
                chain.derived[size_t(derivedIndex)];
            if (chain.haveBase && derived.haveBase) {
                pose->movedProperties[derived.target] =
                    VtValue(derived.result);
            }
        }
    }
}

}  // namespace rigExec
