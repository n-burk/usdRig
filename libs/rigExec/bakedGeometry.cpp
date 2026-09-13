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
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/attributeQuery.h"
#include "pxr/usd/usd/stage.h"

#include <cmath>
#include <string>
#include <utility>
#include <vector>

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
        // exists. transformPhase is counted too: an AtPrim transform is
        // answered out of the same store.
        if (!r.binding.phases.empty() ||
            r.binding.transformPhase.kind == RigExecReadPhaseKind::AtPrim) {
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
// One frame, geometry half.
// ---------------------------------------------------------------------------

void
RigExecBakedRunGeometry(RigExecBakedProgramImpl *program, UsdTimeCode time,
                        RigExecRigPose *pose)
{
    RigExecBakedProgramImpl &B = *program;
    const RigExecResolvedInputs &R = *B.resolvedInputs;
    // The packet is assembled by the same RigExecAssembleParameters the
    // dynamic path calls and the kernel is the same shared kernel; only the
    // VdfNetwork around them is baked away, so the accounting it performed --
    // a revision runs when its inputs changed, and not otherwise -- is
    // performed here instead.
    size_t graphChainsBuilt = 0, graphRevisionsBuilt = 0;
    // Building the geometry state is reported once per node and once per
    // schedule, not once per program: a program rebuilt over state that
    // survived (AdoptGeometryStateFrom) has nothing to report, which is what
    // the dynamic path says about the same edit. Counted inside the walk
    // below, because a chain can lose its state mid-walk -- a point count
    // that moved with time -- and that is a node the dynamic path adds again
    // too.
    const auto accountForChain =
        [&pose](RigExecBakedProgramImpl::GeomChain *chain) {
        for (auto &revision : chain->revisions) {
            if (revision.created) {
                ++pose->moverGraphRevisionsCreated;
                revision.created = false;
            }
        }
        if (chain->scheduleDirty && !chain->revisions.empty()) {
            ++pose->moverGraphSchedulesBuilt;
        }
        chain->scheduleDirty = false;
    };
    // WHICH points a revision is assembled against, stated once because the
    // two callers below pass different ones and the difference is invisible
    // until an operator reads them:
    //
    //  * a CHAIN revision gets the chain's AUTHORED base -- the attribute as
    //    the stage holds it -- for every revision of the chain, not the
    //    running value. The dynamic walk reads the base once per target and
    //    hands that same array to every revision (the values.basePoints
    //    assignment in the chain loop), so a lattice's rest points and a
    //    volumeCorrect's reference volume are measured against the mesh as
    //    authored however deep in the chain they sit.
    //  * a DERIVED revision gets the chain's FINAL points, which is what it
    //    is for: recomputeNormals and recomputeExtent take auxPoints =
    //    basePoints and must describe the geometry as published.
    //
    // Skin and Matrix -- the only two operations IsBakeable admits today --
    // read neither, so this is the contract being made right before an
    // operator that does read them arrives.
    // One overlay per revision: the generation-wide resolved inputs, plus
    // whatever THIS revision's declared phases resolve to out of the run's
    // snapshot store. The assembler reads inputs by path and never learns a
    // phase exists, which is what lets a phase apply to any input. Built only
    // for a revision that declares one -- with none the overlay is a copy of
    // the resolved inputs with nothing added to it, and the copy is the whole
    // cost.
    RigExecResolvedInputs revisionInputs;
    auto assemble = [&](RigExecBakedProgramImpl::GeomRevision &revision,
                        const std::vector<GfVec3f> &basePoints) {
        RigExecProviderValues values;
        values.resolved = &R;
        if (!revision.binding.phases.empty()) {
            revisionInputs = R;
            for (const auto &[inputPath, phase] : revision.binding.phases) {
                if (const VtValue *recorded = B.runSnapshots.Lookup(
                        inputPath, phase, revision.moverPath)) {
                    revisionInputs.SetProperty(inputPath, *recorded);
                } else if (phase.kind != RigExecReadPhaseKind::Preceding) {
                    // Preceding falling through to the stage is correct: the
                    // reader is the chain's first revision, so its preceding
                    // value IS the base. Anything else means the phase named
                    // something that produced nothing.
                    pose->diagnostics.push_back(
                        "diag " + revision.moverPath.GetString() +
                        ": read phase '" + phase.GetAsString() + "' for " +
                        inputPath.GetString() +
                        " resolved to nothing; read the authored base");
                }
            }
            values.resolved = &revisionInputs;
        }
        GfMatrix4d transform(1.0);
        if (revision.transformSlot >= 0) {
            transform = revision.finalPhase
                            ? B.FinalMatrixOf(revision.transformSlot)
                            : B.BaseMatrixOf(revision.transformSlot);
            values.transform = &transform;
        }
        B.influenceScratch.clear();
        if (!revision.influenceSlots.empty()) {
            B.influenceScratch.reserve(revision.influenceSlots.size());
            for (int slot : revision.influenceSlots) {
                B.influenceScratch.push_back(revision.finalPhase
                                                 ? B.FinalMatrixOf(slot)
                                                 : B.BaseMatrixOf(slot));
            }
            values.influenceTransforms = &B.influenceScratch;
        }
        values.basePoints = basePoints;
        // Epoch-fixed layouts resolve through the evaluator's cache, exactly
        // as the dynamic path's assembly does -- the same cache, so the two
        // paths cannot even hold different arrays.
        if (revision.skinTopologyFixed) {
            values.skinTopologyCache = B.skinTopologies;
        }
        RIGEXEC_PROFILE_SCOPE_CAT(
            *B.profiler, "Assemble " + revision.moverPath.GetName(),
            "geometry");
        return RigExecAssembleParameters(revision.moverPrim, revision.op,
                                         revision.binding, values, time);
    };
    // The envelope is applied exactly once, against the preceding revision.
    // Derived maintenance below is the last caller: the chain loop runs it
    // through RigExecRunRevisionKernel, which owns the same blend, and this
    // copy goes when the derived path joins it.
    auto blendEnvelope = [](const RigExecMoverParameters &parameters,
                            const std::vector<GfVec3f> &preceding,
                            std::vector<GfVec3f> *result) {
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
    };

    for (auto &chain : B.chains) {
        RIGEXEC_PROFILE_SCOPE_CAT(
            *B.profiler, "Chain " + chain.target.GetString(), "geometry");
        VtVec3fArray basePoints;
        if (!chain.baseQuery.IsValid() ||
            !chain.baseQuery.Get(&basePoints, time)) {
            continue;
        }
        if (chain.haveResult && basePoints.size() != chain.lastBase.size()) {
            // A time-varying point count replaces THIS target's graph in the
            // dynamic path and leaves every other target's standing. Same
            // here: the cached run describes a different mesh, so it is
            // dropped and the chain's nodes are reported as created -- which
            // is what the dynamic walk reports for the nodes it has to add
            // again. Handing the whole generation back instead would degrade
            // every other chain of the rig for one mesh whose vertex count
            // is keyed.
            chain.haveResult = false;
            chain.result = VtVec3fArray();
            chain.scheduleDirty = true;
            for (RigExecBakedProgramImpl::GeomRevision &revision :
                     chain.revisions) {
                revision.created = true;
                revision.ran = false;
                revision.output.clear();
                revision.lastParameters = RigExecMoverParameters();
                revision.lastStatus = RigExecMoverStatus();
            }
        }
        accountForChain(&chain);
        bool dirty = !chain.haveResult || basePoints != chain.lastBase;
        chain.lastBase = basePoints;
        const std::vector<GfVec3f> base(basePoints.begin(), basePoints.end());
        std::vector<GfVec3f> current = base;
        for (RigExecBakedProgramImpl::GeomRevision &revision :
                 chain.revisions) {
            const RigExecMoverParameters parameters =
                assemble(revision, base);
            if (parameters.enabled && !parameters.valid) {
                float scalar = 1.0f;
                if (const UsdAttribute a = revision.moverPrim.GetAttribute(
                        TfToken("inputs:defaultWeight"))) {
                    R.GetAttribute(a, time, &scalar);
                }
                if (!std::isfinite(scalar) || scalar < 0.0f || scalar > 1.0f) {
                    pose->diagnostics.push_back(
                        "MoverFailed " + revision.moverPath.GetString() +
                        ": inputs:defaultWeight must be finite and in "
                        "[0, 1]; revision passed through");
                }
            }
            const RigExecMoverStatus status =
                RigExecStatusForParameters(parameters, revision.moverPath);
            ++graphRevisionsBuilt;
            if (dirty || !revision.ran ||
                parameters != revision.lastParameters ||
                status != revision.lastStatus) {
                ++pose->moverGraphRevisionsExecuted;
                std::vector<GfVec3f> scratch = current;
                // Not a second dispatch that mirrors _RevisionNode::Compute
                // -- the same function the node calls. The packet check, the
                // full-strength fast path, the kernel and the "apply once"
                // blend all live in RigExecRunRevisionKernel, so an operation
                // cannot mean one thing here and another there.
                //
                // Only skin and matrix reach this loop today: IsBakeable
                // refuses every other operation, and the curvenet adjuster's
                // control frames therefore have nowhere to be written yet.
                const bool applied =
                    status.AllowsApply() &&
                    RigExecRunRevisionKernel(revision.op, parameters, &scratch,
                                             /*controlFrames=*/nullptr);
                revision.resultStatus = status.state;
                if (applied) {
                    current.swap(scratch);
                } else if (status.AllowsApply()) {
                    revision.resultStatus = TfToken("moverFailed");
                }
                revision.output = current;
                revision.lastParameters = parameters;
                revision.lastStatus = status;
                revision.ran = true;
                dirty = true;
            } else {
                current = revision.output;
            }
            // Snapshot only where a phased read named this revision, which
            // is the whole cost of the feature for a rig that uses it and
            // nothing at all for one that does not.
            if (revision.snapshotAfter) {
                B.runSnapshots.Record(
                    chain.target, revision.moverPath,
                    VtValue(VtVec3fArray(current.begin(), current.end())));
            }
        }
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
                 chain.revisions) {
            if (revision.resultStatus == "moverFailed") {
                pose->diagnostics.push_back(
                    "MoverFailed " + revision.moverPath.GetString() +
                    ": execution rejected its inputs; revision passed "
                    "through");
            }
        }
        chain.result = VtVec3fArray(current.begin(), current.end());
        chain.haveResult = true;
        pose->movedProperties[chain.target] = VtValue(chain.result);
        // `final` costs nothing extra: this is the value the chain publishes.
        B.runSnapshots.RecordFinal(chain.target, VtValue(chain.result));
        ++graphChainsBuilt;

        // Derived maintenance reads this chain's FINAL points, which is why
        // it runs here rather than as another revision of the chain.
        for (auto &derived : chain.derived) {
            RIGEXEC_PROFILE_SCOPE_CAT(
                *B.profiler, "Derived " + derived.target.GetString(),
                "geometry");
            VtVec3fArray derivedBase;
            if (!derived.baseQuery.IsValid() ||
                !derived.baseQuery.Get(&derivedBase, time)) {
                continue;
            }
            if (derived.haveResult &&
                derivedBase.size() != derived.lastBase.size()) {
                // As above, and for the same reason: the dynamic walk
                // replaces this derived target's graph alone.
                derived.haveResult = false;
                derived.result = VtVec3fArray();
                derived.revision.created = true;
                derived.revision.ran = false;
                derived.revision.output.clear();
                derived.revision.lastParameters = RigExecMoverParameters();
                derived.revision.lastStatus = RigExecMoverStatus();
            }
            // A derived target is one node with one schedule of its own,
            // created together, exactly as the dynamic walk creates them.
            if (derived.revision.created) {
                ++pose->moverGraphRevisionsCreated;
                ++pose->moverGraphSchedulesBuilt;
                derived.revision.created = false;
            }
            const bool derivedDirty =
                !derived.haveResult || derivedBase != derived.lastBase;
            derived.lastBase = derivedBase;
            // The chain's FINAL points, deliberately: see the assemble
            // contract above.
            const RigExecMoverParameters parameters =
                assemble(derived.revision, current);
            const RigExecMoverStatus status = RigExecStatusForParameters(
                parameters, derived.revision.moverPath);
            ++graphRevisionsBuilt;
            if (derivedDirty || !derived.revision.ran ||
                parameters != derived.revision.lastParameters ||
                status != derived.revision.lastStatus) {
                ++pose->moverGraphRevisionsExecuted;
                const std::vector<GfVec3f> preceding(derivedBase.begin(),
                                                     derivedBase.end());
                std::vector<GfVec3f> values;
                const bool wanted =
                    status.AllowsApply() && parameters.valid &&
                    parameters.kind ==
                        (derived.revision.op ==
                                 RigExecRevisionOp::RecomputeExtent
                             ? "recomputeExtent"
                             : "recomputeNormals");
                if (wanted) {
                    values = derived.revision.op ==
                                     RigExecRevisionOp::RecomputeExtent
                                 ? RigExecComputeExtent(parameters.auxPoints,
                                                        parameters.widths)
                                 : RigExecComputeVertexNormals(
                                       parameters.auxPoints,
                                       parameters.topologyCounts,
                                       parameters.topologyIndices);
                }
                bool applied =
                    wanted && !values.empty() &&
                    (derived.revision.op !=
                         RigExecRevisionOp::RecomputeExtent ||
                     values.size() == 2) &&
                    // The derived property keeps its authored cardinality: an
                    // in-place write cannot resize it.
                    values.size() == preceding.size() &&
                    blendEnvelope(parameters, preceding, &values);
                derived.revision.resultStatus = status.state;
                if (!applied) {
                    values = preceding;
                    if (status.AllowsApply()) {
                        derived.revision.resultStatus = TfToken("moverFailed");
                    }
                }
                derived.revision.output = values;
                derived.revision.lastParameters = parameters;
                derived.revision.lastStatus = status;
                derived.revision.ran = true;
            }
            if (derived.revision.resultStatus == "moverFailed") {
                pose->diagnostics.push_back(
                    "MoverFailed " + derived.target.GetString() +
                    ": derived geometry input/cardinality validation failed");
            }
            derived.result = VtVec3fArray(derived.revision.output.begin(),
                                          derived.revision.output.end());
            derived.haveResult = true;
            pose->movedProperties[derived.target] = VtValue(derived.result);
            ++graphChainsBuilt;
        }
    }

    // Whatever the Profile Mover binds reported; drained so a cached bind
    // stays silent on every later frame. Empty on a bakeable rig, drained
    // anyway so the two paths leave the evaluator in the same state.
    for (std::string &message :
             B.curvenetBindings->TakeDiagnostics()) {
        pose->diagnostics.push_back(std::move(message));
    }
    pose->diagnostics.push_back(
        "mover graph: " + std::to_string(graphChainsBuilt) +
        " chain(s), " + std::to_string(graphRevisionsBuilt) +
        " revision(s); " + std::to_string(pose->moverGraphRevisionsCreated) +
        " created, " + std::to_string(pose->moverGraphRevisionsExecuted) +
        " executed, " + std::to_string(pose->moverGraphSchedulesBuilt) +
        " schedule(s) built");
}

}  // namespace rigExec
