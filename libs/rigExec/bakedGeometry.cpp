#include "rigExecGraph/geometryProgram.h"
#include "rigExecGraph/blendLayout.h"
#include "rigExec/weightField.h"
// The baked program's geometry half: the chain/revision bake and the frame
// path that runs those revisions and the derived maintenance behind them.
// Split out of bakedProgram.cpp for the reason bakedPose.cpp was: a domain's
// bake and its frame path have to agree about what was captured and what is
// re-read. The evaluator is not visible here -- the compiled chains arrive
// restated as RigExecBakedChainSpec, and the caches the frame path shares
// with the dynamic walk were captured into RigExecBakedProgramImpl at Build.
#include "bakedOpValues.h"
#include "bakedProgramImpl.h"
#include "bakedSchedule.h"

#include "frameExtraction.h"
#include "moverGraph.h"
#include "movers/moverRegistry.h"
#include "parallel.h"
#include "rigEvaluator.h"
#include "types.h"

#include "rigExecMath/dualQuat.h"
#include "rigExecMath/envelope.h"
#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/latticeKernel.h"
#include "rigExecMath/pointRanges.h"
#include "rigExecMath/simdKernels.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/usd/usdSkel/blendShape.h"
#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/attributeQuery.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// The two names a step body needs a TOKEN for rather than a comparison: the
// status a failed revision publishes, and the attribute the defaultWeight
// diagnostic re-reads. Hoisted because TfToken(const char *) takes the token
// registry's spin lock on every construction, and a step body may take no
// lock (docs/specs/baked-step-graph.md §2).
TF_DEFINE_PRIVATE_TOKENS(
    _tokens,
    ((moverFailed, "moverFailed"))
    ((defaultWeight, "inputs:defaultWeight"))
    ((classicLinear, "classicLinear"))
    ((dualQuaternion, "dualQuaternion"))
    ((jointIndices, "rigExec:jointIndices"))
    ((elementSize, "rigExec:elementSize"))
    ((denseRepresentation, "dense"))
    ((sparseRepresentation, "sparse"))
    ((staticWeight, "RigExecStaticWeight"))
    ((weightDefault, "rigExec:defaultWeight"))
    ((targetSpace, "target"))
    ((operationCycle, "operation cycle"))
);

namespace rigExec {

void
RigExecBakedGeometryTouchTokens()
{
    (void)_tokens.Get();
}

// Bake.

namespace {

/// Binds every phased point read of the program to the chain versions the
/// dynamic walk answers it from (RigExecBakedPointsBinding): its phased-read
/// store, or for `preceding` on the reader's own chain the points entering
/// the reader.
///
/// "Before the reader" is program order, which RigExecBakedBuildGeometrySteps
/// emits chain by chain in `chains` order: every step of an earlier chain,
/// and on the reader's own chain the fuses of the revisions before it -- or,
/// for a derived reader, every fuse and the chain's status step. The walk
/// records a revision only where `snapshotAfter` is set and a chain's final
/// after its status sweep, and both only when the chain read a base.
void
BindPointReads(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    RigExecBakedCaptureCrossDomainOrder(&B);
    int nextId = 0;
    const auto bind = [&](const SdfPath &input, const RigExecReadPhase &phase,
                          const SdfPath &reader, size_t, size_t, bool) {
        auto out = RigExecBakedBindPointInput(B,input,phase,reader);
        out.id = nextId++;
        return out;
    };
    const auto bindRevision =
        [&](RigExecBakedProgramImpl::GeomRevision *revision, size_t chain,
            size_t before, bool afterStatus) {
            revision->pointBindings.clear();
            for (const auto &[input, phase] : revision->binding.phases) {
                RigExecBakedPointsBinding bound =
                    bind(input, phase, revision->moverPath, chain, before,
                         afterStatus);
                bound.diagnoseMiss =
                    phase.kind != RigExecReadPhaseKind::Preceding;
                if (bound.diagnoseMiss) {
                    bound.missDiagnostic =
                        "diag " + revision->moverPath.GetString() +
                        ": read phase '" + bound.phase.GetAsString() +
                        "' for " + bound.input.GetString() +
                        " resolved to nothing; read the authored base";
                }
                revision->pointBindings.push_back(std::move(bound));
            }
            for (RigExecBakedProgramImpl::GeomBlendChannel &channel :
                     revision->blendChannels) {
                for (auto &sample : channel.samples) {
                    sample.pointBinding = RigExecBakedPointsBinding();
                    if (sample.phase.IsBase() || !sample.blendShape.IsEmpty()) {
                        continue;
                    }
                    sample.pointBinding =
                        bind(sample.pointsPath, sample.phase,
                             revision->moverPath, chain, before, afterStatus);
                }
            }
        };
    for (size_t c = 0; c < B.chains.size(); ++c) {
        RigExecBakedProgramImpl::GeomChain &chain = B.chains[c];
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            bindRevision(&chain.revisions[r], c, r, false);
        }
        for (RigExecBakedProgramImpl::GeomChain::Derived &derived :
                 chain.derived) {
            bindRevision(&derived.revision, c, chain.revisions.size(), true);
        }
    }
    B.pointBindingCount = nextId;
}

}  // namespace

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
        out.moverPathText = r.moverPath.GetString();
        out.target = r.target;
        out.moverPrim = B.stage->GetPrimAtPath(r.moverPath);
        out.op = r.op;
        out.binding = r.binding;
        out.finalPhase = r.transformFinalPhase;
        out.posedPoints = r.transformPosedPoints;
        out.skinTopologyFixed = r.skinTopologyFixed;
        out.snapshotAfter = r.snapshotAfter;
        // The geometry-domain constraint whose measured delta IS this
        // revision's transform. The join key is the MOVER path, which is
        // what the dynamic walk's own constraintDeltas map is keyed by; the
        // pose half is baked first, so the constraint is already here.
        for (const RigExecBakedProgramImpl::Constraint &constraint :
                 B.constraints) {
            if (constraint.deltaBase >= 0 && constraint.path == r.moverPath) {
                out.constraintDelta = constraint.deltaBase;
                break;
            }
        }
        if (!out.moverPrim) {
            refuse("mover prim is missing", r.moverPath);
        }
        // The packet is assembled live from this prim every frame through
        // the generation's resolved inputs: nothing of it is captured, so a
        // value edit needs no rebuild and an override places itself. The
        // layout arrays are still NAMED, because bakeability judged them --
        // connecting one is a resync, and the judgement was that none is
        // connected. The weights do not belong in `rebuild`: the SkinTopology
        // op holds the layout, and its layout leaves are filed under these
        // paths, so a weight-paint edit reaches the next generation through
        // the re-read rather than through a rebuild of this program. The
        // indices and element size are topology, and go to `rebuild` with
        // the other topology reads below.
        B.prims.insert(r.moverPath);
        for (const SdfPath &structural : r.binding.externalStructure) {
            if (structural.IsPropertyPath()) B.named.insert(structural);
            B.prims.insert(structural.GetPrimPath());
        }
        B.resolvedRoutedPrims.insert(r.moverPath);
        for (const char *name : {"rigExec:jointIndices", "rigExec:jointWeights",
                                 "rigExec:elementSize",
                                 "rigExec:skinningMethod"}) {
            B.named.insert(r.moverPath.AppendProperty(TfToken(name)));
        }
        // Every path an operation's per-frame assembler reads, whatever the
        // operation is: the cage a lattice deforms through, the surface a
        // projection lands on, the topology a smooth walks, the bind
        // coordinates a ribbon rides. None of them is folded into the
        // program -- they are re-read on every frame through the
        // generation's resolved inputs -- so they belong in `named`, where a
        // RESYNC (the property appearing, disappearing or being retargeted,
        // which also invalidates any retained query) finds them, and not in
        // `rebuild`, which is for values the program captured -- except the
        // topology among them, which is epoch state (below).
        // Named unconditionally rather than per operation: the binding
        // carries exactly the paths the operation resolved, so an empty one
        // is an operation that does not read it and an authored one is a
        // path some assembler will ask for.
        const auto name = [&](const SdfPath &path) {
            if (path.IsEmpty()) {
                return;
            }
            B.named.insert(path);
            B.prims.insert(path.GetPrimPath());
        };
        name(r.binding.base);
        name(r.binding.topologyCounts);
        name(r.binding.topologyIndices);
        name(r.binding.cagePoints);
        name(r.binding.surfacePoints);
        name(r.binding.bindCoords);
        name(r.binding.driverCurvePoints);
        name(r.binding.driverCurveOrder);
        name(r.binding.driverCurveKnots);
        name(r.binding.widths);
        // A phased input reads the chain version it is bound to when a
        // candidate answers and the resolved input -- the stage -- otherwise,
        // so the path is one the bake asked about either way.
        for (const auto &[input, phase] : r.binding.phases) {
            name(input);
        }
        // The blend channels, in the order the accumulation is defined over.
        // Only the HANDLES are captured: every channel weight, every
        // activation and every target shape is re-read per frame through the
        // generation's resolved inputs, which is what lets an animator drag a
        // channel weight and see it -- so the input and sample prims go into
        // `resolvedRoutedPrims` beside the mover's, where an override on one
        // of their properties places with nothing else to do.
        for (const SdfPath &input : r.binding.blendInputs) {
            RigExecBakedProgramImpl::GeomBlendChannel channel;
            B.prims.insert(input);
            B.resolvedRoutedPrims.insert(input);
            const SdfPath weightPath =
                input.AppendProperty(TfToken("inputs:weight"));
            B.named.insert(weightPath);
            channel.weight = B.stage->GetAttributeAtPath(weightPath);
            channel.weightPath = weightPath;
            // A weight connected to a pose interpolator's output reads the
            // interpolator's slot. The walk is the one RigExecResolvedInputs::
            // GetAttribute makes -- single authored connections, followed
            // until one lands on a property the program publishes -- so the
            // slot answers exactly where the dynamic walk's in-memory lookup
            // would have.
            {
                std::set<SdfPath> visiting;
                UsdAttribute a = channel.weight;
                while (a && visiting.insert(a.GetPath()).second) {
                    const auto found = B.poseWeightIndex.find(a.GetPath());
                    if (found != B.poseWeightIndex.end()) {
                        channel.poseWeight = found->second;
                        break;
                    }
                    SdfPathVector connections;
                    if (a.HasAuthoredConnections()) {
                        a.GetConnections(&connections);
                    }
                    if (connections.size() != 1) {
                        break;
                    }
                    a = B.stage->GetAttributeAtPath(connections[0]);
                }
            }
            const auto samples = r.binding.blendSamples.find(input);
            if (samples != r.binding.blendSamples.end()) {
                for (const RigExecBlendSampleBinding &binding :
                         samples->second) {
                    RigExecBakedProgramImpl::GeomBlendChannel::Sample sample;
                    B.prims.insert(binding.sample);
                    B.resolvedRoutedPrims.insert(binding.sample);
                    const SdfPath activationPath =
                        binding.sample.AppendProperty(
                            TfToken("rigExec:activation"));
                    B.named.insert(activationPath);
                    sample.activation =
                        B.stage->GetAttributeAtPath(activationPath);
                    sample.activationPath = activationPath;
                    sample.samplePath = binding.sample;
                    sample.pointsPath = binding.points;
                    sample.phase = binding.phase;
                    sample.blendShape = binding.blendShape;
                    if (!sample.blendShape.IsEmpty()) {
                        const UsdSkelBlendShape shape(B.stage->GetPrimAtPath(sample.blendShape.GetPrimPath()));
                        sample.shapeValid = bool(shape);
                        sample.layoutRefused = shape &&
                            (shape.GetOffsetsAttr().HasAuthoredConnections() ||
                             shape.GetPointIndicesAttr().HasAuthoredConnections());
                    }
                    if (binding.blendShape.IsEmpty()) {
                        name(binding.points);
                        sample.points =
                            B.stage->GetAttributeAtPath(binding.points);
                    } else {
                        // A sparse sample's shape is read from its leaves;
                        // what is named here is the shape prim, so a resync
                        // under it rebuilds. Its offsets and point indices
                        // are topology, so an edit to either rebuilds too
                        // (the epoch reads below).
                        B.prims.insert(binding.blendShape.GetPrimPath());
                    }
                    channel.samples.push_back(std::move(sample));
                }
            }
            out.blendChannels.push_back(std::move(channel));
        }
        if (!r.binding.transform.IsEmpty()) {
            out.transformSlot = slotOf(r.binding.transform);
            if (out.transformSlot < 0) {
                refuse("mover transform provider is not a pose provider",
                       r.binding.transform);
            }
        }
        if (!r.binding.transformSpace.IsEmpty()) {
            out.transformSpaceSlot = slotOf(r.binding.transformSpace);
            if (out.transformSpaceSlot < 0) {
                refuse("mover transform space is not a pose provider",
                       r.binding.transformSpace);
            }
        }
        // rigExec:space: the rig's carry. Refused on a miss for the same
        // reason the measuring space is -- a carry that quietly resolved to
        // nothing looks exactly like the bug it was named to fix.
        if (!r.binding.carrySpace.IsEmpty()) {
            out.carrySpaceSlot = slotOf(r.binding.carrySpace);
            if (out.carrySpaceSlot < 0) {
                refuse("mover carry space is not a pose provider",
                       r.binding.carrySpace);
            }
        }
        // The solver whose aggregate supplies values.driverFrames, resolved
        // once here: the walk's solver table is built before the geometry
        // half, so the per-revision tap the dynamic path reads becomes an
        // index into it.
        if (!r.binding.driverFrames.IsEmpty()) {
            B.prims.insert(r.binding.driverFrames);
            const auto solver = B.solverIndex.find(r.binding.driverFrames);
            if (solver == B.solverIndex.end()) {
                // Compile admitted this revision against its aggregate
                // batch. A miss here is a lowering correspondence failure,
                // not the optional-driver admission policy.
                refuse("admitted driver frames solver is absent from the native table",
                       r.binding.driverFrames);
            } else {
                out.driverFramesSolver = solver->second;
            }
        }
        for (const SdfPath &influence : r.binding.influences) {
            const int slot = slotOf(influence);
            if (slot < 0) refuse("skin influence is not a provider", influence);
            out.influenceSlots.push_back(slot);
        }
        // The weight object, baked once however many revisions bind it: what
        // this revision holds is its index in the shared table, and the
        // packet itself is built once per frame by that object's own step.
        out.weightObject =
            RigExecBakedBakeWeightObject(ctx, r.binding.weightObject);
        if (out.weightObject >= 0) {
            // WHAT the published field says it weights, decided here because
            // it is a relationship read and a relationship is epoch state.
            // The condition is the dynamic path's, verbatim: exactly one
            // declared target, and it is this mover, means the field weights
            // the OPERATION and carries one element; anything else weights
            // the chain's points.
            const UsdPrim weightPrim =
                B.stage->GetPrimAtPath(r.binding.weightObject);
            const SdfPathVector declared =
                ctx->Targets(weightPrim, "rigExec:weightTarget");
            out.weightOperationDomain =
                declared.size() == 1 && declared[0] == r.moverPath;
            out.weightFieldTarget =
                out.weightOperationDomain ? r.moverPath : r.target;
            out.weightCurrentPhase =
                B.currentPhaseWeights.count(r.binding.weightObject) > 0;
        }
        // The packet assembly's reads as path leaves, sampled in the
        // prologue: the assembler's, for an operation the leaves assemble,
        // and the blend gather's, which reads the generation's resolved
        // inputs and not the phase overlay.
        RigExecRevisionLeafDecl &decl = out.leaves.decl;
        RigExecDeclareRevisionLeaves(out.op, out.moverPath, out.binding,
                                     &decl);
        if (decl.assembles) {
            using Type = RigExecRevisionLeafType;
            using Flavour = RigExecRevisionLeafFlavour;
            const RigExecRevisionLeafTime at =
                RigExecRevisionLeafTime::AtTime;
            for (RigExecBakedProgramImpl::GeomBlendChannel &channel :
                     out.blendChannels) {
                channel.weightHeldLeaf =
                    decl.Add({channel.weightPath, Type::Bool, at,
                              Flavour::Present, VtValue(false)});
                channel.weightLeaf = decl.Add(
                    {channel.weightPath, Type::Float, at,
                     Flavour::ResolvedOnly,
                     VtValue(RigExecBlendChannel().weight)});
                for (auto &sample : channel.samples) {
                    sample.activationLeaf = decl.Add(
                        {sample.activationPath, Type::Float, at,
                         Flavour::ResolvedOnly,
                         VtValue(RigExecBlendSampleData().activation)});
                    if (sample.blendShape.IsEmpty()) {
                        sample.pointsLeaf =
                            decl.Add({sample.pointsPath, Type::Vec3fArray, at,
                                      Flavour::ResolvedOnly,
                                      VtValue(VtVec3fArray())});
                    } else {
                        // Sparse layouts are sampled at Default in the prologue;
                        // their exact source arrays must also reach readiness.
                        const SdfPath shape = sample.blendShape.GetPrimPath();
                        sample.offsetsLeaf = decl.Add({shape.AppendProperty(TfToken("offsets")),
                                  Type::Vec3fArray, RigExecRevisionLeafTime::AtDefault,
                                  Flavour::Raw, VtValue(VtVec3fArray())});
                        sample.indicesLeaf = decl.Add({shape.AppendProperty(TfToken("pointIndices")),
                                  Type::IntArray, RigExecRevisionLeafTime::AtDefault,
                                  Flavour::Raw, VtValue(VtIntArray())});
                    }
                }
            }
        }
        // Topology is epoch state: the reads the role table marks
        // (RigExecRevisionLeafRoleIsTopology) and a sparse sample's offsets
        // and point indices. An authored edit to one -- a value, a time
        // sample, a connection -- rebuilds the program into a new epoch
        // instead of reaching the next generation through a re-read, so
        // inside one program such a read moves only with the time or an
        // override (RigExecBakedSamplePathLeaves).
        const auto epoch = [&](int k) {
            if (k < 0 || size_t(k) >= decl.keys.size() ||
                !decl.keys[size_t(k)].path.IsPropertyPath()) {
                return;
            }
            const SdfPath &path = decl.keys[size_t(k)].path;
            B.rebuild.insert(path);
            B.named.insert(path);
            B.prims.insert(path.GetPrimPath());
        };
        for (size_t role = 0; role < decl.roles.size(); ++role) {
            if (RigExecRevisionLeafRoleIsTopology(
                    RigExecRevisionLeafRole(role))) {
                epoch(decl.roles[role]);
            }
        }
        for (const auto &channel : out.blendChannels) {
            for (const auto &sample : channel.samples) {
                epoch(sample.offsetsLeaf);
                epoch(sample.indicesLeaf);
            }
        }
        out.kernelRecord.op=out.op;
        out.kernelRecord.binding=out.binding;
        out.kernelRecord.leaves=decl;
        // Detached lowering consumes the already captured scene facts. USD
        // handles above remain confined to the native sampling adapter.
        if(B.sceneDescriptors) {
            std::string error;
            if(!RigExecLowerSceneGeometry(*B.sceneDescriptors,out.moverPath,out.target,
                    &out.sceneGeometry,&error,&out.kernelRecord))
                refuse(error,out.moverPath);
        }
        return out;
    };
    // A revision's leaves bound, its sparse samples' offsets and point
    // indices marked epoch keys beside the role table's.
    const auto bindRevisionLeaves =
        [&](RigExecBakedProgramImpl::GeomRevision *revision) {
            RigExecBakedBindPathLeaves(B.stage, &revision->leaves);
            std::vector<char> &marks = revision->leaves.epoch;
            for (const auto &channel : revision->blendChannels) {
                for (const auto &sample : channel.samples) {
                    for (const int k : {sample.offsetsLeaf, sample.indicesLeaf}) {
                        if (k >= 0 && size_t(k) < marks.size()) {
                            marks[size_t(k)] = 1;
                        }
                    }
                }
            }
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
            // RevisionStatic's own defaultWeight read, which the fuse's
            // diagnostic is about: the generation's resolved inputs alone.
            RigExecBakedProgramImpl::GeomRevision &baked =
                chain.revisions.back();
            baked.defaultWeightLeaf = baked.leaves.decl.Add(
                {baked.moverPath.AppendProperty(_tokens->defaultWeight),
                 RigExecRevisionLeafType::Float,
                 RigExecRevisionLeafTime::AtTime,
                 RigExecRevisionLeafFlavour::ResolvedOnly, VtValue(1.0f)});
            bindRevisionLeaves(&baked);
        }
        for (const RigExecBakedRevisionSpec &derived : spec.derived) {
            RigExecBakedProgramImpl::GeomChain::Derived d;
            d.target = derived.target;
            d.targetText = derived.target.GetString();
            d.matrixTarget = RigExecIsDerivedMatrixOp(derived.op);
            B.prims.insert(derived.target.GetPrimPath());
            B.named.insert(derived.target);
            if (d.matrixTarget) {
                // A projector's primvar is published, never authored, so
                // there is no base to read; its inputs are the chain's.
            } else if (const UsdAttribute a =
                    B.stage->GetAttributeAtPath(derived.target)) {
                d.baseQuery = UsdAttributeQuery(a);
            } else {
                refuse("derived target has no attribute", derived.target);
            }
            d.revision = bakeRevision(derived);
            bindRevisionLeaves(&d.revision);
            chain.derived.push_back(std::move(d));
        }
        B.chains.push_back(std::move(chain));
    }
    BindPointReads(&B);
}


// The vertex partition.
// A skin revision's per-vertex work is separable -- point i reads the indices
// and weights at i * elementSize, the influence matrices they name and its
// own incoming position, and nothing else -- so a contiguous range of
// vertices is an independent sub-problem whose result is bit-identical to the
// same vertices computed as part of the whole array. What is NOT separable is
// everything around that body, which is why the range is cut here and the
// decisions stay whole (§6 of docs/specs/baked-step-graph.md).
// The cut is by vertex COUNT and the key follows from it, rather than the
// other way round: the vertex order is the mesh's and is never permuted, so
// two body regions that happen to share a range simply wait for both their
// joints -- and the other ranges still start when their own joints land,
// which is what the whole exercise buys. The schedule report prints |key| per
// chunk so that trade-off is measured per asset rather than assumed.

namespace {

// Natural chunk producer identities are independent of any scheduling order.
using SkinProducerSet = std::vector<std::pair<uint8_t, int>>;

SkinProducerSet
ChunkProducers(const RigExecBakedProgramImpl::GeomRevision &revision,
               const RigExecBakedProgramImpl::GeomChunk &chunk, bool chunked)
{
    SkinProducerSet producers;
    const uint8_t domain = uint8_t(RigExecBakedOwnMatrixDomain(revision));
    const size_t count = chunked ? chunk.key.size() : revision.influenceSlots.size();
    for (size_t k = 0; k < count; ++k) {
        const size_t position = chunked ? size_t(chunk.key[k]) : k;
        if (position < revision.influenceSlots.size() &&
            revision.influenceSlots[position] >= 0)
            producers.emplace_back(domain, revision.influenceSlots[position]);
    }
    std::sort(producers.begin(), producers.end());
    producers.erase(std::unique(producers.begin(), producers.end()), producers.end());
    return producers;
}

bool
ChunkAlways()
{
    return TfGetenvInt("RIGEXEC_BAKED_CHUNK_ALWAYS", 0) != 0;
}

/// Whether every element of ascending \p a appears in ascending \p b.
bool
IsSubset(const std::vector<int> &a, const std::vector<int> &b)
{
    size_t j = 0;
    for (const int value : a) {
        while (j < b.size() && b[j] < value) {
            ++j;
        }
        if (j >= b.size() || b[j] != value) {
            return false;
        }
    }
    return true;
}

}  // namespace

/// How many vertices one chunk covers before the cap takes over.
///
/// The default is the point count at which the geometry kernels start
/// splitting work themselves, because a range smaller than that is a range
/// the deformation was never worth spreading over. Read at Build into
/// chunkVertexTarget.
size_t
RigExecBakedChunkVertexTargetFromEnvironment()
{
    const int authored = TfGetenvInt("RIGEXEC_BAKED_CHUNK_VERTS",
                                     int(RigExecGeometryParallelThreshold));
    return authored < 1 ? size_t(1) : size_t(authored);
}

/// The most chunks one revision is cut into; the range grows to meet it.
///
/// A cap rather than a target: chunk count decides STEP count, and a mesh
/// with a million vertices must not add two hundred steps to the program for
/// a machine that can run twenty of them at once. Read at Build into
/// chunkCap.
size_t
RigExecBakedChunkCapFromEnvironment()
{
    const int authored = TfGetenvInt("RIGEXEC_BAKED_MAX_CHUNKS", 32);
    return authored < 1 ? size_t(1) : size_t(authored);
}

/// Whether a chain of more than chunkVertexTarget points runs its Matrix,
/// Wire and Lattice revisions as one step per point range. Read at Build
/// into rangeChains.
bool
RigExecBakedRangeChainsFromEnvironment()
{
    return TfGetenvBool("RIGEXEC_BAKED_RANGE_CHAINS", true);
}

/// How many points one vertex group of a range chain covers before the cap
/// takes over. Read at Build into groupVertexTarget.
size_t
RigExecBakedGroupVertexTargetFromEnvironment()
{
    const int authored = TfGetenvInt("RIGEXEC_BAKED_GROUP_VERTS", 1024);
    return authored < 1 ? size_t(1) : size_t(authored);
}

/// The most vertex groups one range chain is cut into. Read at Build into
/// groupCap.
size_t
RigExecBakedGroupCapFromEnvironment()
{
    const int authored = TfGetenvInt("RIGEXEC_BAKED_GROUP_CAP", 16);
    return authored < 1 ? size_t(1) : size_t(authored);
}

/// Whether a Range revision over a static sparse zero-default weight writes
/// only the groups its indices touch. Read at Build into groupGates.
bool
RigExecBakedGroupGatesFromEnvironment()
{
    return TfGetenvBool("RIGEXEC_BAKED_GROUP_GATES", true);
}

void
RigExecBakedPartitionRevision(
    const RigExecBakedProgramImpl &program,
    RigExecBakedProgramImpl::GeomRevision *revision,
    const VtIntArray &indices, int elementSize)
{
    using GeomChunk = RigExecBakedProgramImpl::GeomChunk;
    const size_t indexCount = indices.size();
    const size_t slots = elementSize < 1 ? 0 : size_t(elementSize);
    const size_t points = slots ? indexCount / slots : 0;
    const size_t influences = revision->influenceSlots.size();
    revision->partitionElementSize = elementSize;
    revision->partitionIndexCount = indexCount;
    revision->partitionPointCount = points;

    // How many ranges, and how long.
    size_t target = program.chunkVertexTarget;
    size_t count = std::max<size_t>(1, (points + target - 1) / target);
    if (count > program.chunkCap) {
        count = program.chunkCap;
        target = std::max<size_t>(1, (points + count - 1) / count);
        count = std::max<size_t>(1, (points + target - 1) / target);
    }

    std::vector<GeomChunk> chunks;
    chunks.reserve(count);
    // One pass over the indices, and one byte per influence to remember what
    // this range has already claimed -- the key is a SET of the range's
    // influence positions, and the vertices arrive in no particular order.
    std::vector<char> claimed(influences, 0);
    for (size_t c = 0; c < count; ++c) {
        GeomChunk chunk;
        chunk.begin = int(std::min(points, c * target));
        chunk.end = int(c + 1 == count ? points
                                       : std::min(points, (c + 1) * target));
        for (size_t point = size_t(chunk.begin); point < size_t(chunk.end);
             ++point) {
            for (size_t slot = 0; slot < slots; ++slot) {
                const int index = indices.cdata()[point * slots + slot];
                // An index outside the table is a layout the packet will
                // reject; leaving it out of the key costs nothing, because
                // the revision never applies.
                if (index < 0 || size_t(index) >= influences ||
                    claimed[size_t(index)]) {
                    continue;
                }
                claimed[size_t(index)] = 1;
                chunk.key.push_back(index);
            }
        }
        std::sort(chunk.key.begin(), chunk.key.end());
        for (const int index : chunk.key) {
            claimed[size_t(index)] = 0;
        }
        chunks.push_back(std::move(chunk));
    }

    // Adjacent ranges merge while they stay under the vertex target and one
    // key contains the other: a range waiting for a superset of its
    // neighbour's joints is not waiting any longer for holding both, and one
    // range is one step's worth of scheduling instead of two.
    for (size_t i = 0; i + 1 < chunks.size();) {
        const size_t merged = size_t(chunks[i + 1].end - chunks[i].begin);
        const bool contains = IsSubset(chunks[i].key, chunks[i + 1].key);
        const bool contained = IsSubset(chunks[i + 1].key, chunks[i].key);
        if (merged <= target && (contains || contained)) {
            chunks[i].end = chunks[i + 1].end;
            if (contains) {
                chunks[i].key = chunks[i + 1].key;  // the union
            }
            chunks.erase(chunks.begin() + long(i) + 1);
        } else {
            ++i;
        }
    }

    // The table every chunk skins against: identity everywhere, with its own
    // influences copied in per run. Filled here so that a run writes only the
    // |key| entries that moved, and so that the rows a SIMD kernel loads are
    // never out of step with the matrices beside them.
    const bool keyed = chunks.size() > 1;
    for (GeomChunk &chunk : chunks) {
        chunk.ok = false;
        chunk.keyChanged = false;
        chunk.palette.clear();
        if (!keyed) {
            // One chunk is the whole array, and the whole array is what the
            // revision's own folded table already holds.
            chunk.key.clear();
            chunk.transforms.clear();
            chunk.rows.clear();
            continue;
        }
        chunk.transforms.assign(influences, GfMatrix4d(1.0));
        chunk.rows.assign(influences * RigExecSkinRowStride, 0.0f);
        for (size_t t = 0; t < influences; ++t) {
            RigExecNarrowSkinRows(chunk.transforms[t],
                                  &chunk.rows[t * RigExecSkinRowStride]);
        }
    }
    revision->chunks = std::move(chunks);
    revision->chunked = keyed;
    revision->partitionIndices = keyed ? indices : VtIntArray();
}

void
RigExecBakedAdoptPartition(RigExecBakedProgramImpl::GeomRevision *revision)
{
    // A null handle is a refusal: the packet reads the arrays per frame and
    // RevisionStatic runs the revision whole. Recording it here would make
    // the handles agree by both being null.
    if (!revision->chunked || !revision->topology ||
        revision->topology == revision->partitionTopology) {
        return;
    }
    // The keys stay Build's whatever the layout says, because each chunk
    // step's reads were declared from its key: a re-cut would let a chunk
    // read a joint it never declared, and a cone that skips it would leave
    // its range stale. So a handle is adopted only when it has the arrays
    // the keys were cut from; a weight-only edit keeps the chunks running.
    const RigExecSkinTopology &topology = *revision->topology;
    const VtIntArray &indices = revision->partitionIndices;
    if (topology.elementSize == revision->partitionElementSize &&
        topology.indices.size() == indices.size() &&
        std::equal(topology.indices.begin(), topology.indices.end(),
                   indices.cdata())) {
        revision->partitionTopology = revision->topology;
    }
}

// Build: the geometry half of the program, in program order.

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

/// Declares the matrices \p revision reads, which are the slots the pose half
/// published for it: the same enumeration the ProviderMatrix steps were
/// created from.
void
DeclareMatrixReads(const RigExecBakedProgramImpl::GeomRevision &revision,
                   RigExecBakedStep *step)
{
    RigExecBakedForEachMatrixRead(
        revision, [step](RigExecBakedSlotDomain domain, int slot) {
        step->reads.push_back(RigExecBakedOne(domain, slot));
    });
}

/// Declares a read of point version \p version: the authored base for 0, and
/// otherwise the RevisionDone and ChainDirty slots of revision
/// first + version - 1, whose only writer is that revision's fuse. The buffer
/// the version resolves to is not declared: whichever earlier chunk or fuse
/// filled it precedes that fuse.
void
DeclarePointVersionRead(const RigExecBakedProgramImpl &B,
                        RigExecBakedPointVersion version,
                        std::vector<RigExecBakedSlotRange> *reads)
{
    RigExecBakedDeclarePointVersion(B,version.chain,version.version,reads);
}

/// Declares what \p binding can read: each candidate version, or the
/// chain's published points for a final read. The tail is the resolved
/// input, which is prologue state.
void
DeclarePointBindingReads(const RigExecBakedProgramImpl &B,
                         const RigExecBakedPointsBinding &binding,
                         std::vector<RigExecBakedSlotRange> *reads)
{
    RigExecBakedDeclarePointInput(B,binding,reads);
}

/// The frame records an AtPrim transform phase of \p revision can read,
/// for its transform and for each influence entry. Their FrameMatrix steps
/// are the only writers, and all of them are pose steps.
void
DeclareFrameRecordReads(const RigExecBakedProgramImpl::GeomRevision &revision,
                        std::vector<RigExecBakedSlotRange> *reads)
{
    for (const int record : revision.transformRecords) {
        reads->push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::FrameMatrix, record));
    }
    for (const std::vector<int> &records : revision.influenceRecords) {
        for (const int record : records) {
            reads->push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::FrameMatrix, record));
        }
    }
}

/// The point bindings of \p revision: its declared input phases and its
/// blend samples'.
void
DeclareRevisionPointReads(const RigExecBakedProgramImpl &B,
                          const RigExecBakedProgramImpl::GeomRevision &revision,
                          std::vector<RigExecBakedSlotRange> *reads)
{
    for (const RigExecBakedPointsBinding &binding : revision.pointBindings) {
        DeclarePointBindingReads(B, binding, reads);
    }
    for (const RigExecBakedProgramImpl::GeomBlendChannel &channel :
             revision.blendChannels) {
        for (const auto &sample : channel.samples) {
            DeclarePointBindingReads(B, sample.pointBinding, reads);
        }
    }
}

}  // namespace

namespace {

/// Cuts \p revision at Build, from the layout the stage authors.
///
/// Only a SKIN revision whose layout the epoch fixed is cut into more than
/// one chunk. Two reasons, and they are the same reason twice: a layout that
/// can move within the epoch is one whose keys a Build-time cut cannot
/// promise anything about, and a mover with such a layout already re-reads
/// and re-validates every element of it once per frame, which is far more
/// than a partition would save. Everything else -- every other operation, and
/// a skin whose arrays are animated, connected or written by a property chain
/// -- is one chunk over the whole array, which is the degenerate case of the
/// same step.
void
PartitionAtBuild(const RigExecBakedProgramImpl &B,
                 RigExecBakedProgramImpl::GeomRevision *revision)
{
    revision->chunks.assign(1, RigExecBakedProgramImpl::GeomChunk());
    revision->chunked = false;
    revision->partitionCandidates = 0;
    revision->partitionProducerMin = 0;
    revision->partitionProducerMax = 0;
    revision->partitionDistinctReads = 0;
    revision->partitionProducerSets.clear();
    if (revision->op != RigExecRevisionOp::Skin ||
        !revision->skinTopologyFixed || !revision->moverPrim) {
        return;
    }
    // Read directly, at Default: Build holds no layout handle yet. Fixed
    // means the arrays do not vary in time, so the default-time read IS the
    // epoch's. A handle whose arrays disagree with it (an override standing
    // on the indices at Build) is never adopted, and the revision runs whole
    // (RigExecBakedAdoptPartition).
    VtIntArray indices;
    int elementSize = 1;
    if (const UsdAttribute a =
            revision->moverPrim.GetAttribute(_tokens->jointIndices)) {
        a.Get(&indices, UsdTimeCode::Default());
    }
    if (const UsdAttribute a =
            revision->moverPrim.GetAttribute(_tokens->elementSize)) {
        a.Get(&elementSize, UsdTimeCode::Default());
    }
    if (elementSize < 1 || indices.empty() ||
        indices.size() % size_t(elementSize) != 0) {
        return;
    }
    RigExecBakedPartitionRevision(B, revision, indices, elementSize);
    revision->partitionCandidates = revision->chunks.size();

    // The vertex target and cap bound the work per operation. Distinct exact
    // producer sets let the common graph release independent ranges; no pose
    // scheduling information is needed to make the cut.
    std::set<SkinProducerSet> distinct;
    int producerMin = -1, producerMax = 0;
    for (const auto &chunk : revision->chunks) {
        auto producers = ChunkProducers(*revision, chunk, revision->chunked);
        const int count = int(producers.size());
        producerMin = producerMin < 0 ? count : std::min(producerMin, count);
        producerMax = std::max(producerMax, count);
        distinct.insert(producers);
        revision->partitionProducerSets.push_back(std::move(producers));
    }
    revision->partitionProducerMin = std::max(0, producerMin);
    revision->partitionProducerMax = producerMax;
    revision->partitionDistinctReads = distinct.size();
    if (revision->chunked && distinct.size() < 2 && !ChunkAlways()) {
        // Back to the degenerate cut, which is the shape every other
        // operation already has: one range over the whole array, no keys and
        // no per-chunk tables, so the fuse's whole-array path runs the
        // revision through the self-parallelising kernel. The partition
        // FIELDS stay where the cut left them -- the point and index counts
        // are the layout's, not the cut's, and the report reads them -- but
        // the indices, which only an adoption reads, are let go.
        revision->chunks.assign(1, RigExecBakedProgramImpl::GeomChunk());
        revision->chunked = false;
        revision->partitionIndices = VtIntArray();
    }
}

/// Declares WeightFrames[slot] for every volume in weight object \p object's
/// composition closure (its baseWeight and inputWeights, recursively): the
/// placements the oracle reads when it resolves that object. A closure
/// member outside the table, or a volume slot that has no placement step,
/// should be unreachable; it falls back to every volume slot.
void
DeclarePlacementReads(const RigExecBakedProgramImpl &B, int object,
                      std::vector<RigExecBakedSlotRange> *reads)
{
    std::vector<char> seen(B.weightObjects.size(), 0);
    std::vector<int> stack{object};
    std::vector<int> slots;
    bool complete = true;
    while (!stack.empty()) {
        const int o = stack.back();
        stack.pop_back();
        if (o < 0 || size_t(o) >= B.weightObjects.size()) {
            complete = false;
            continue;
        }
        if (seen[size_t(o)]) {
            continue;
        }
        seen[size_t(o)] = 1;
        const RigExecBakedProgramImpl::WeightObject &weight =
            B.weightObjects[size_t(o)];
        if (weight.providerSlot >= 0) {
            if (size_t(weight.providerSlot) < B.noScaleAvars.size() &&
                B.noScaleAvars[size_t(weight.providerSlot)]) {
                slots.push_back(weight.providerSlot);
            } else {
                complete = false;
            }
        }
        if (weight.base >= 0) {
            stack.push_back(weight.base);
        }
        stack.insert(stack.end(), weight.inputs.begin(), weight.inputs.end());
    }
    if (!TF_VERIFY(complete, "a weight object's closure is outside the "
                             "program's volume table")) {
        slots.clear();
        for (size_t i = 0; i < B.noScaleAvars.size(); ++i) {
            if (B.noScaleAvars[i]) {
                slots.push_back(int(i));
            }
        }
    }
    for (const int slot : slots) {
        reads->push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::WeightFrames, slot));
    }
}

/// The point count Build cuts \p chain's ranges from, read on the owning
/// thread in the prologue's order: the upstream value at the target, else the
/// authored base at Default, else at the earliest time; 0 when none reads.
/// Only the partition depends on it: a run clips the ranges to the count it
/// applies to, and any partition gives a per-point kernel the same bits.
size_t
BuildPointCount(const RigExecBakedProgramImpl &B,
                const RigExecBakedProgramImpl::GeomChain &chain)
{
    const auto upstream = B.upstream.find(chain.target);
    if (upstream != B.upstream.end() &&
        upstream->second.IsHolding<VtVec3fArray>()) {
        return upstream->second.UncheckedGet<VtVec3fArray>().size();
    }
    if (!chain.baseQuery.IsValid()) {
        return 0;
    }
    VtVec3fArray points;
    if (!chain.baseQuery.Get(&points, UsdTimeCode::Default()) ||
        points.empty()) {
        points = VtVec3fArray();
        chain.baseQuery.Get(&points, UsdTimeCode::EarliestTime());
    }
    return points.size();
}

/// How many ranges a chain of \p points points is cut into: the shared count,
/// less the trailing ranges the ceil-sized bounds would leave empty (only
/// when the cap or a tiny vertex target makes the ranges nearly as many as
/// the points), so every range is non-empty. 1 is no pipelining.
size_t
ChainRangeCount(const RigExecBakedProgramImpl &B, size_t points)
{
    if (!B.rangeChains) {
        return 1;
    }
    size_t ranges =
        RigExecPointRangeCount(points, B.chunkVertexTarget, B.chunkCap);
    while (ranges > 1 &&
           RigExecPointRangeBound(points, ranges, ranges - 1) >= points) {
        --ranges;
    }
    return ranges;
}

/// How many vertex groups a range chain of \p points points is cut into:
/// the group target and cap, trimmed like ChainRangeCount so every group is
/// non-empty. Live and Export alike.
size_t
ChainGroupCount(const RigExecBakedProgramImpl &B, size_t points)
{
    if (!B.rangeChains) {
        return 1;
    }
    size_t groups =
        RigExecPointRangeCount(points, B.groupVertexTarget, B.groupCap);
    while (groups > 1 &&
           RigExecPointRangeBound(points, groups, groups - 1) >= points) {
        --groups;
    }
    return groups;
}

using GeomRole = RigExecBakedRevisionRole;
using RolePin = RigExecBakedRolePin;

/// What Build's role classification reads, the way the geometry prologue
/// will read it: the stage, the evaluator's standing interactive overrides
/// and the admitted upstream values (B.upstream). Owning thread, Build only.
struct RoleReads {
    explicit RoleReads(const RigExecBakedProgramImpl &program)
        : B(program),
          exportMode(program.roleMode == RigExecBakedRoleMode::Export)
    {
        if (B.interactiveOverrides) {
            for (const RigExecValueOverride &o : *B.interactiveOverrides) {
                const SdfPath path = o.prim.AppendProperty(
                    o.attribute.IsEmpty() ? o.computation : o.attribute);
                overlay.SetProperty(path, o.value);
                overrides.emplace_back(path, o.value);
            }
        }
    }

    /// Whether a standing override or an admitted upstream value stands on
    /// one of \p hops.
    bool Reaches(const std::vector<SdfPath> &hops) const
    {
        for (const SdfPath &hop : hops) {
            if (B.upstream.count(hop)) {
                return true;
            }
            for (const auto &entry : overrides) {
                if (entry.first == hop) {
                    return true;
                }
            }
        }
        return false;
    }

    /// Whether \p path may never become a file constant: a property-chain
    /// target, an admitted upstream path or a path the bake keeps listed.
    bool KeptListed(const SdfPath &path) const
    {
        return B.chainTargets.count(path) || B.upstream.count(path) ||
               B.exportKeep.count(path);
    }

    /// Live: key \p k's read cannot move with the time -- no hop varies or
    /// holds a time sample, so an animated method or space is never pinned --
    /// and its walk meets no value the region computes.
    bool Pinnable(const RigExecBakedPathLeaves &leaves, int k) const
    {
        if (k < 0 || size_t(k) >= leaves.decl.keys.size() ||
            size_t(k) >= leaves.hops.size()) {
            return false;
        }
        if (size_t(k) < leaves.walks.size() && leaves.walks[size_t(k)] >= 0) {
            return false;
        }
        if (RigExecBakedLeafVaryingNow(B, leaves, size_t(k))) {
            return false;
        }
        for (const SdfPath &hop : leaves.hops[size_t(k)]) {
            if (const UsdAttribute a = B.stage->GetAttributeAtPath(hop)) {
                if (a.ValueMightBeTimeVarying() || a.GetNumTimeSamples() > 0) {
                    return false;
                }
            }
        }
        return true;
    }

    /// Export: pinnable, a walk of exactly the head (no connection), and
    /// never kept listed.
    bool Bakeable(const RigExecBakedPathLeaves &leaves, int k) const
    {
        if (!Pinnable(leaves, k)) {
            return false;
        }
        const std::vector<SdfPath> &hops = leaves.hops[size_t(k)];
        const SdfPath &head = leaves.decl.keys[size_t(k)].path;
        if (hops.size() > 1 || (hops.size() == 1 && hops[0] != head) ||
            KeptListed(head)) {
            return false;
        }
        const UsdAttribute &a = size_t(k) < leaves.attributes.size()
                                    ? leaves.attributes[size_t(k)]
                                    : UsdAttribute();
        return !(a && a.HasAuthoredConnections());
    }

    /// The rule of the mode Build runs in.
    bool Usable(const RigExecBakedPathLeaves &leaves, int k) const
    {
        return exportMode ? Bakeable(leaves, k) : Pinnable(leaves, k);
    }

    /// Key \p k's value as the prologue will sample it. A usable key does not
    /// vary, so Default reads what every time reads.
    TfToken Token(const RigExecBakedPathLeaves &leaves, int k) const
    {
        const VtValue value = RigExecSampleRevisionLeaf(
            leaves.decl.keys[size_t(k)], leaves.attributes[size_t(k)],
            &overlay, UsdTimeCode::Default(), &B.upstream);
        return value.IsHolding<TfToken>() ? value.UncheckedGet<TfToken>()
                                          : TfToken();
    }

    const RigExecBakedProgramImpl &B;
    const bool exportMode;
    RigExecResolvedInputs overlay;
    std::vector<std::pair<SdfPath, VtValue>> overrides;
};

/// A skin mover's layout arrays as PartitionAtBuild reads them, at Default;
/// false when they cannot be cut.
bool
ReadSkinLayoutAtBuild(const RigExecBakedProgramImpl::GeomRevision &revision,
                      VtIntArray *indices, int *elementSize)
{
    *elementSize = 1;
    if (!revision.moverPrim) {
        return false;
    }
    if (const UsdAttribute a =
            revision.moverPrim.GetAttribute(_tokens->jointIndices)) {
        a.Get(indices, UsdTimeCode::Default());
    }
    if (const UsdAttribute a =
            revision.moverPrim.GetAttribute(_tokens->elementSize)) {
        a.Get(elementSize, UsdTimeCode::Default());
    }
    return *elementSize >= 1 && !indices->empty() &&
           indices->size() % size_t(*elementSize) == 0;
}

/// Cuts a skin revision of a range chain at the chain's group bounds: chunk
/// g is group g, keyed by the union of its vertices' joint indices (zero-
/// weight slots included) and cut exactly at the bounds -- no merge, no
/// degenerate fallback, RIGEXEC_BAKED_CHUNK_ALWAYS not consulted -- with its
/// tables filled as a keyed chunk's and the indices kept for adoption.
/// False, leaving the revision as it was, when the layout cannot be cut.
bool
PartitionSkinGroups(RigExecBakedProgramImpl::GeomRevision *revision,
                    const std::vector<int> &bounds)
{
    using GeomChunk = RigExecBakedProgramImpl::GeomChunk;
    VtIntArray indices;
    int elementSize = 1;
    if (bounds.size() < 2 ||
        !ReadSkinLayoutAtBuild(*revision, &indices, &elementSize)) {
        return false;
    }
    const size_t slots = size_t(elementSize);
    const size_t layoutPoints = indices.size() / slots;
    const size_t influences = revision->influenceSlots.size();
    std::vector<GeomChunk> chunks(bounds.size() - 1);
    std::vector<char> claimed(influences, 0);
    for (size_t g = 0; g + 1 < bounds.size(); ++g) {
        GeomChunk &chunk = chunks[g];
        chunk.begin = bounds[g];
        chunk.end = bounds[g + 1];
        const size_t end = std::min(size_t(chunk.end), layoutPoints);
        for (size_t point = size_t(chunk.begin); point < end; ++point) {
            for (size_t slot = 0; slot < slots; ++slot) {
                const int index = indices.cdata()[point * slots + slot];
                // As RigExecBakedPartitionRevision: an index outside the
                // table is a layout the packet rejects.
                if (index < 0 || size_t(index) >= influences ||
                    claimed[size_t(index)]) {
                    continue;
                }
                claimed[size_t(index)] = 1;
                chunk.key.push_back(index);
            }
        }
        std::sort(chunk.key.begin(), chunk.key.end());
        for (const int index : chunk.key) {
            claimed[size_t(index)] = 0;
        }
        chunk.ok = false;
        chunk.keyChanged = false;
        chunk.transforms.assign(influences, GfMatrix4d(1.0));
        chunk.rows.assign(influences * RigExecSkinRowStride, 0.0f);
        for (size_t t = 0; t < influences; ++t) {
            RigExecNarrowSkinRows(chunk.transforms[t],
                                  &chunk.rows[t * RigExecSkinRowStride]);
        }
    }
    revision->chunks = std::move(chunks);
    revision->chunked = true;
    revision->partitionIndices = indices;
    revision->partitionElementSize = elementSize;
    revision->partitionIndexCount = indices.size();
    revision->partitionPointCount = layoutPoints;
    revision->partitionCandidates = revision->chunks.size();
    revision->partitionProducerSets.clear();
    std::set<SkinProducerSet> distinct;
    int producerMin = -1, producerMax = 0;
    for (const GeomChunk &chunk : revision->chunks) {
        SkinProducerSet producers = ChunkProducers(*revision, chunk, true);
        const int count = int(producers.size());
        producerMin = producerMin < 0 ? count : std::min(producerMin, count);
        producerMax = std::max(producerMax, count);
        distinct.insert(producers);
        revision->partitionProducerSets.push_back(std::move(producers));
    }
    revision->partitionProducerMin = std::max(0, producerMin);
    revision->partitionProducerMax = producerMax;
    revision->partitionDistinctReads = distinct.size();
    return true;
}

/// The role \p revision of an eligible chain takes, from what Build reads,
/// with the pins it rests on (2.2). A role a usable method or space leaf
/// chose by value is pinned, whichever it chose; a Range skin also pins that
/// no override or upstream value reaches its layout topology. Export adds
/// the Range skin's reads to \p pinned.
GeomRole
ClassifyRevision(const RoleReads &reads,
                 RigExecBakedProgramImpl::GeomRevision *revision,
                 std::vector<SdfPath> *pinned)
{
    using Role = RigExecRevisionLeafRole;
    const RigExecBakedPathLeaves &leaves = revision->leaves;
    const auto pinToken = [&](int k, const TfToken &token) {
        RolePin pin;
        pin.kind = RolePin::Kind::LeafToken;
        pin.leaf = k;
        pin.token = token;
        revision->pins.push_back(pin);
    };
    switch (revision->op) {
    case RigExecRevisionOp::Matrix:
    case RigExecRevisionOp::Wire:
    case RigExecRevisionOp::Lattice:
        return GeomRole::Range;
    case RigExecRevisionOp::BlendShape: {
        // rigExec:deltaSpace never enters the file, so Export pins it as
        // Live does.
        const int space = leaves.decl.Role(Role::DeltaSpace);
        if (!reads.Pinnable(leaves, space)) {
            return GeomRole::Whole;
        }
        const TfToken value = reads.Token(leaves, space);
        pinToken(space, value);
        return value == _tokens->targetSpace ? GeomRole::Range
                                             : GeomRole::Whole;
    }
    case RigExecRevisionOp::Skin: {
        // The keys of a Range skin rest on an epoch-fixed layout; without
        // one the method decides nothing and nothing is pinned.
        VtIntArray indices;
        int elementSize = 1;
        if (!revision->skinTopologyFixed ||
            !RigExecSkinLayoutTopologyIsFixed(revision->moverPrim) ||
            !ReadSkinLayoutAtBuild(*revision, &indices, &elementSize)) {
            return GeomRole::Whole;
        }
        const int method = leaves.decl.Role(Role::SkinningMethod);
        const int jointIndices = leaves.decl.Role(Role::JointIndices);
        const int size = leaves.decl.Role(Role::ElementSize);
        if (!reads.Usable(leaves, method) || jointIndices < 0 || size < 0 ||
            size_t(jointIndices) >= leaves.hops.size() ||
            size_t(size) >= leaves.hops.size()) {
            return GeomRole::Whole;
        }
        // A layout override or upstream value standing at Build keeps the
        // stale-partition fallback of a Whole fuse.
        if (reads.Reaches(leaves.hops[size_t(jointIndices)]) ||
            reads.Reaches(leaves.hops[size_t(size)]) ||
            (reads.exportMode && (!reads.Bakeable(leaves, jointIndices) ||
                                  !reads.Bakeable(leaves, size)))) {
            return GeomRole::Whole;
        }
        const TfToken value = reads.Token(leaves, method);
        pinToken(method, value);
        if (value != _tokens->classicLinear) {
            return GeomRole::Whole;
        }
        for (const int k : {jointIndices, size}) {
            RolePin pin;
            pin.kind = RolePin::Kind::NoLayoutOverride;
            pin.leaf = k;
            revision->pins.push_back(pin);
        }
        if (reads.exportMode) {
            for (const int k : {method, jointIndices, size}) {
                pinned->push_back(leaves.decl.keys[size_t(k)].path);
            }
        }
        return GeomRole::Range;
    }
    default:
        return GeomRole::Whole;
    }
}

/// Static sparse weight object \p object's resolved unlisted value -- its
/// defaultWeight, the only read that enters it (types.cpp) -- as the
/// prologue will read it, when a gate may rest on it: the read is pinnable
/// (Live) or a bakeable constant (Export). \p head receives the read's
/// attribute path.
bool
GateDefault(const RoleReads &reads, int object, float *value, SdfPath *head)
{
    const RigExecBakedProgramImpl &B = reads.B;
    if (object < 0 || size_t(object) >= B.weightObjects.size()) {
        return false;
    }
    const RigExecBakedProgramImpl::WeightObject &weight =
        B.weightObjects[size_t(object)];
    if (weight.type != _tokens->staticWeight ||
        weight.representation != _tokens->sparseRepresentation) {
        return false;
    }
    const RigExecBakedInput<float> &input = weight.defaultWeight;
    if (input.varying || input.walk >= 0) {
        return false;
    }
    SdfPathVector walk;
    UsdAttribute selected;
    if (input.head) {
        bool viaChain = false, varying = false;
        RigExecBakedClassifyInput<float>(input.head, UsdTimeCode::Default(),
                                         B.chainTargets, &viaChain, &varying,
                                         &selected, &walk);
        if (viaChain || varying) {
            return false;
        }
        *head = input.head.GetPath();
        if (reads.exportMode && (walk.size() > 1 ||
                                 input.head.HasAuthoredConnections())) {
            return false;
        }
    } else {
        *head = weight.path.AppendProperty(_tokens->weightDefault);
    }
    if (reads.exportMode) {
        if (reads.KeptListed(*head)) {
            return false;
        }
        for (const SdfPath &hop : walk) {
            if (reads.KeptListed(hop)) {
                return false;
            }
        }
    }
    // The value the walk selects -- it does not vary, so Default reads what
    // every time reads -- unless a standing override or upstream value
    // answers first along the walk.
    *value = input.constant;
    const auto number = [value](const VtValue &v) {
        if (v.IsHolding<float>()) {
            *value = v.UncheckedGet<float>();
        } else if (v.IsHolding<double>()) {
            *value = float(v.UncheckedGet<double>());
        } else {
            return false;
        }
        return true;
    };
    VtValue authored;
    if (selected && selected.Get(&authored, UsdTimeCode::Default())) {
        number(authored);
    }
    for (const SdfPath &hop : walk) {
        for (const auto &entry : reads.overrides) {
            if (entry.first == hop) {
                return number(entry.second);
            }
        }
        const auto upstream = B.upstream.find(hop);
        if (upstream != B.upstream.end()) {
            return number(upstream->second);
        }
    }
    return true;
}

/// Gates a Range \p revision (2.3): a static sparse weight object with an
/// exactly-zero resolved default the pins can hold, a packet the kernel half
/// (RigExecRevisionGateHolds) admits, and no current-phase or operation-
/// domain weighting. Writes the groups the authored indices inside
/// [0, N0) touch into \p written, pins the default and, in Export, adds its
/// read to \p pinned.
bool
GateRevision(const RoleReads &reads,
             const RigExecBakedProgramImpl::GeomChain &chain,
             RigExecBakedProgramImpl::GeomRevision *revision,
             std::vector<char> *written, std::vector<SdfPath> *pinned)
{
    const RigExecBakedProgramImpl &B = reads.B;
    if (!B.groupGates || revision->weightObject < 0 ||
        revision->weightCurrentPhase || revision->weightOperationDomain) {
        return false;
    }
    float value = 1.0f;
    SdfPath head;
    if (!GateDefault(reads, revision->weightObject, &value, &head) ||
        value != 0.0f) {
        return false;
    }
    const RigExecBakedProgramImpl::WeightObject &weight =
        B.weightObjects[size_t(revision->weightObject)];
    // The packet a run assembles from this object, as far as the gate reads
    // it: the static weight's arrays at the default the pin holds.
    RigExecMoverParameters probe;
    probe.enabled = true;
    probe.valid = true;
    probe.weights.representation = weight.representation;
    probe.weights.rangePolicy = weight.rangePolicy;
    probe.weights.values = weight.values;
    probe.weights.indices = weight.indices;
    probe.weights.defaultWeight = 0.0f;
    probe.weights.valid = true;
    probe.skinningMethod = _tokens->classicLinear;
    probe.blendSurfaceFrame = false;
    if (!RigExecRevisionGateHolds(revision->op, probe)) {
        return false;
    }
    const std::vector<int> &bounds = chain.groupBounds;
    written->assign(bounds.size() - 1, 0);
    for (const int index : weight.indices) {
        if (index < 0 || size_t(index) >= chain.groupPointCount) {
            continue;
        }
        const size_t g = size_t(std::upper_bound(bounds.begin(), bounds.end(),
                                                 index) -
                                bounds.begin()) - 1;
        if (g < written->size()) {
            (*written)[g] = 1;
        }
    }
    RolePin pin;
    pin.kind = RolePin::Kind::WeightDefaultZero;
    pin.weightObject = revision->weightObject;
    revision->pins.push_back(pin);
    if (reads.exportMode) {
        pinned->push_back(head);
    }
    return true;
}

/// Gives every revision of \p chain its role, groups, gates, pins and
/// chunks (2.1-2.3, 5.2). A chain is range-pipelined when wave 5 would split
/// it, it holds two or more groups and one of its revisions takes the Range
/// role; every other chain keeps its Legacy chunks. The skinning methods and
/// delta spaces of an eligible chain are epoch keys: an authored edit to one
/// rebuilds, so only an override or upstream value flips a role in-epoch.
void
AssignChainRoles(RigExecBakedProgramImpl *program, const RoleReads &reads,
                 RigExecBakedProgramImpl::GeomChain *chain, size_t points)
{
    RigExecBakedProgramImpl &B = *program;
    const size_t groups = ChainGroupCount(B, points);
    const bool eligible = ChainRangeCount(B, points) >= 2 && groups >= 2;
    chain->groupBounds.clear();
    chain->groupPointCount = 0;
    chain->baseOwner.reset();
    chain->baseGroups.clear();
    chain->resultIds.clear();
    chain->spareIds.clear();
    std::vector<GeomRole> candidate(chain->revisions.size(), GeomRole::Legacy);
    std::vector<std::vector<SdfPath>> pinned(chain->revisions.size());
    bool anyRange = false;
    for (size_t r = 0; r < chain->revisions.size(); ++r) {
        RigExecBakedProgramImpl::GeomRevision &revision = chain->revisions[r];
        revision.role = GeomRole::Legacy;
        revision.rangeRole = false;
        revision.groupWritten.clear();
        revision.enteringWriter.clear();
        revision.groups.clear();
        revision.groupIds.clear();
        revision.pins.clear();
        PartitionAtBuild(B, &revision);
        if (!eligible) {
            continue;
        }
        for (const RigExecRevisionLeafRole role :
                 {RigExecRevisionLeafRole::SkinningMethod,
                  RigExecRevisionLeafRole::DeltaSpace}) {
            const int k = revision.leaves.decl.Role(role);
            if (k >= 0 && revision.leaves.decl.keys[size_t(k)]
                              .path.IsPropertyPath()) {
                const SdfPath &path = revision.leaves.decl.keys[size_t(k)].path;
                B.rebuild.insert(path);
                B.named.insert(path);
                B.prims.insert(path.GetPrimPath());
            }
        }
        candidate[r] = ClassifyRevision(reads, &revision, &pinned[r]);
        anyRange = anyRange || candidate[r] == GeomRole::Range;
    }
    if (!anyRange) {
        // Legacy. A skin or blend that a usable leaf made Whole keeps its
        // LeafToken pin, so a flip to Range rebuilds into a range chain.
        return;
    }
    chain->groupPointCount = points;
    chain->groupBounds.resize(groups + 1);
    for (size_t g = 0; g <= groups; ++g) {
        chain->groupBounds[g] = int(RigExecPointRangeBound(points, groups, g));
    }
    chain->baseGroups.resize(groups);
    chain->resultIds.assign(groups, RigExecGroupSource());
    chain->spareIds.assign(groups, RigExecGroupSource());
    std::vector<int> lastWriter(groups, -1);
    for (size_t r = 0; r < chain->revisions.size(); ++r) {
        RigExecBakedProgramImpl::GeomRevision &revision = chain->revisions[r];
        revision.role = candidate[r];
        revision.rangeRole = revision.role == GeomRole::Range;
        revision.groupWritten.assign(groups, 1);
        const bool skin = revision.op == RigExecRevisionOp::Skin;
        if (revision.rangeRole && skin &&
            !PartitionSkinGroups(&revision, chain->groupBounds)) {
            // Unreachable: the classification read the same layout.
            revision.role = GeomRole::Whole;
            revision.rangeRole = false;
            pinned[r].clear();
        }
        if (revision.rangeRole) {
            if (!skin) {
                revision.chunks.assign(groups,
                                       RigExecBakedProgramImpl::GeomChunk());
                for (size_t g = 0; g < groups; ++g) {
                    revision.chunks[g].begin = chain->groupBounds[g];
                    revision.chunks[g].end = chain->groupBounds[g + 1];
                }
                revision.chunked = false;
                revision.partitionIndices = VtIntArray();
            }
            std::vector<char> written;
            if (GateRevision(reads, *chain, &revision, &written, &pinned[r])) {
                revision.groupWritten = std::move(written);
            }
            B.exportPinnedPaths.insert(pinned[r].begin(), pinned[r].end());
        } else if (skin && revision.skinTopologyFixed &&
                   PartitionSkinGroups(&revision, chain->groupBounds)) {
            // A Whole keyed skin: one speculative chunk per group.
        } else {
            // A Whole one-chunk revision: PartitionAtBuild's single chunk,
            // which a skin's whole fuse runs unkeyed.
            revision.chunks.assign(1, RigExecBakedProgramImpl::GeomChunk());
            revision.chunked = false;
            revision.partitionIndices = VtIntArray();
        }
        revision.enteringWriter = lastWriter;
        revision.groups.assign(groups, RigExecGroupState<GfVec3f>());
        revision.groupIds.assign(groups, RigExecGroupSource());
        for (size_t g = 0; g < groups; ++g) {
            if (!revision.groupWritten[g]) {
                continue;
            }
            lastWriter[g] = int(r);
            // The two buffers its writer alternates between, sized to the
            // group here, on the owning thread, so a run allocates nothing.
            const size_t size =
                size_t(chain->groupBounds[g + 1] - chain->groupBounds[g]);
            for (auto &own : revision.groups[g].own) {
                own = std::make_shared<std::vector<GfVec3f>>(size);
            }
        }
    }
}

/// The chain index of the revision whose slot holds group \p g of point
/// version \p version of \p chain, or -1 for the base: the last revision
/// before the version that writes g, unless a revision a cycle set aside
/// (which passes the base through) comes first.
int
GroupWriter(const RigExecBakedProgramImpl::GeomChain &chain, size_t version,
            size_t g)
{
    for (size_t q = std::min(version, chain.revisions.size()); q > 0; --q) {
        const RigExecBakedProgramImpl::GeomRevision &revision =
            chain.revisions[q - 1];
        if (revision.rangeSetAside) {
            return -1;
        }
        if (g < revision.groupWritten.size() && revision.groupWritten[g]) {
            return int(q - 1);
        }
    }
    return -1;
}

/// Appends RevisionOut reads of `chunkBase + g` for every written group g,
/// one range per run of consecutive written groups.
void
DeclareWrittenGroupReads(const RigExecBakedProgramImpl::GeomRevision &revision,
                         std::vector<RigExecBakedSlotRange> *reads)
{
    const size_t groups = revision.groupWritten.size();
    for (size_t g = 0; g < groups;) {
        if (!revision.groupWritten[g]) {
            ++g;
            continue;
        }
        size_t end = g + 1;
        while (end < groups && revision.groupWritten[end]) {
            ++end;
        }
        reads->push_back(RigExecBakedRange(RigExecBakedSlotDomain::RevisionOut,
                                           revision.chunkBase + int(g),
                                           revision.chunkBase + int(end)));
        g = end;
    }
}

}  // namespace

void
RigExecBakedBuildGeometrySteps(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    B.chainChunkBegin.assign(B.chains.size(), 0);
    B.chainChunkEnd.assign(B.chains.size(), 0);
    B.chunkRevision.clear();
    B.exportPinnedPaths.clear();
    const RoleReads roleReads(B);
    int nextChunk = 0;
    for (size_t c = 0; c < B.chains.size(); ++c) {
        RigExecBakedProgramImpl::GeomChain &chain = B.chains[c];
        // The chain's vertex groups, roles, gates and pins: every Range
        // revision of it carries the group bounds as its chunks.
        AssignChainRoles(&B, roleReads, &chain, BuildPointCount(B, chain));
        const size_t groups =
            chain.groupBounds.empty() ? 0 : chain.groupBounds.size() - 1;
        RigExecBakedStep &input = AddGeometryStep(&B,RigExecBakedStepKind::ChainInputs,int(c));
        input.maxDiagnostics = 0;
        input.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::ChainInput,int(c)));
        input.writes.push_back(RigExecBakedOne(RigExecBakedSlotDomain::ChainBase,int(c)));
        // Revision ids are handed out chain by chain, so one chain's
        // revisions are a contiguous RANGE: point version v of the chain is
        // revision first + v - 1's, and the status sweep reads the whole
        // chain as one declared range. The chunk ids inside them are
        // contiguous for the same reason and at the same grain.
        // Build preindexed every revision before WeightField capture;
        // all readers and operations consume those same immutable ids.
        B.chainChunkBegin[c] = nextChunk;
        const int first = B.chainRevisionBegin[c];
        const int last = B.chainRevisionEnd[c];
        B.revisionFuseStep.resize(size_t(last), -1);
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            RigExecBakedProgramImpl::GeomRevision &revision =
                chain.revisions[r];
            const int id = first + int(r);
            // The points entering this revision, which its chunks, its fuse
            // and a current-phase assemble read.
            const RigExecBakedPointVersion entering{int(c), int(r)};
            const bool skin = revision.op == RigExecRevisionOp::Skin;
            revision.influences.assign(revision.influenceSlots.size(),
                                       GfMatrix4d(1.0));
            // The table the PACKET carries for a skin revision, and never
            // anything else: identity, sized to the influence count and
            // written once, here.
            // The packet is assembled before the matrices are folded, so it
            // cannot carry them -- that is what lets a chunk start on its own
            // joints without waiting for every joint of the rig. What the
            // assembler does with the table is check its SHAPE and its
            // elements' finiteness, and identities pass both; the real
            // table's finite/affine check is the fold's, and the fuse ANDs it
            // in exactly where the assembler's would have landed.
            revision.packetInfluences.assign(revision.influenceSlots.size(),
                                             GfMatrix4d(1.0));
            revision.rangeInputs = RigExecRevisionRangeInputs();
            revision.joinSeen.clear();
            revision.rangeRefusals = 0;
            const GeomRole role = revision.role;
            // RevisionOut ids: a Range revision's G groups (an unwritten one
            // has no writer and no reader); a Whole one's chunks, then the G
            // groups its fuse publishes; a Legacy one's chunks.
            const size_t own = revision.chunks.size();
            const size_t published = role == GeomRole::Range ? groups
                                     : role == GeomRole::Whole ? own + groups
                                                               : own;
            revision.chunkBase = nextChunk;
            nextChunk += int(published);
            B.revisionChunkBase.push_back(revision.chunkBase);
            B.revisionChunkCount.push_back(int(published));
            B.chunkRevision.insert(B.chunkRevision.end(), published, id);

            // The fold comes FIRST for every operation but a skin, because
            // every other operation's packet carries the matrix it was folded
            // from. A skin revision's does not, so its fold comes after the
            // assemble -- which is also where it learns the skinning method,
            // and so which form of the table the chunks will want.
            const auto addFold = [&] {
                RigExecBakedStep &fold = AddGeometryStep(
                    &B, RigExecBakedStepKind::InfluenceFold, id);
                // The body gates folding on this chain's current base availability.
                fold.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::ChainBase, int(c)));
                DeclareMatrixReads(revision, &fold);
                // The delta a geometry-domain constraint measured for this
                // mover IS this revision's transform, so the fold waits for
                // the constraint step that writes it.
                if (revision.constraintDelta >= 0) {
                    fold.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::ConstraintDelta,
                        revision.constraintDelta));
                }
                // A pose-walk read phase reads the frame records its lists
                // name, each written by one FrameMatrix step.
                DeclareFrameRecordReads(revision, &fold.reads);
                if (skin) {
                    fold.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::RevisionPacket, id));
                }
                fold.writes.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::RevisionTransforms, id));
            };
            if (revision.weightField >= 0) {
                RigExecBakedStep &field = AddGeometryStep(
                    &B,RigExecBakedStepKind::WeightField,revision.weightField);
                field.maxDiagnostics = 0;
                RigExecBakedDeclareWeightField(&B,&field);
            }
            const auto addStatic = [&] {
                RigExecBakedStep &assemble = AddGeometryStep(
                    &B, RigExecBakedStepKind::RevisionStatic, id);
                // One line per declared phase that resolved to nothing. The
                // defaultWeight line moved to the fuse, which is the first
                // step that knows both halves of "the packet is valid".
                assemble.maxDiagnostics = revision.binding.phases.size();
                if (!skin) {
                    assemble.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::RevisionTransforms, id));
                }
                assemble.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::ChainBase, int(c)));
                if (revision.weightObject >= 0) {
                    assemble.reads.push_back(
                        RigExecBakedOne(RigExecBakedSlotDomain::WeightPacket,
                                        revision.weightObject));
                }
                if (revision.weightField >= 0)
                    assemble.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::WeightField,revision.weightField));
                if (revision.weightCurrentPhase) assemble.maxDiagnostics += 1;
                if (revision.driverFramesSolver >= 0) {
                    assemble.reads.push_back(
                        RigExecBakedOne(RigExecBakedSlotDomain::Aggregate,
                                        revision.driverFramesSolver));
                }
                // A channel driven by a pose interpolator waits for the
                // interpolator's step, which is the edge that makes a drag
                // that cannot reach the driver leave this revision alone.
                for (const RigExecBakedProgramImpl::GeomBlendChannel &channel :
                         revision.blendChannels) {
                    if (channel.poseWeight >= 0) {
                        assemble.reads.push_back(RigExecBakedOne(
                            RigExecBakedSlotDomain::PoseWeight,
                            channel.poseWeight));
                    }
                }
                // The versions its phased inputs and blend samples are bound
                // to. The AtPrim transform phase is the fold's read.
                DeclareRevisionPointReads(B, revision, &assemble.reads);
                assemble.writes.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::RevisionPacket, id));
            };
            if (skin) {
                addStatic();
                addFold();
            } else {
                addFold();
                addStatic();
            }
            // A Range revision's group steps and a Whole keyed skin's
            // speculative chunks are one per group and read group g of the
            // entering version alone; every other chunk reads the version.
            const bool perGroup =
                role == GeomRole::Range ||
                (role == GeomRole::Whole && revision.chunked);
            for (size_t k = 0; k < revision.chunks.size(); ++k) {
                if (role == GeomRole::Range && !revision.groupWritten[k]) {
                    // A gated group: the entering group is this revision's.
                    continue;
                }
                RigExecBakedStep &chunk = AddGeometryStep(
                    &B, RigExecBakedStepKind::RevisionChunk, id, int(k));
                chunk.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::RevisionPacket, id));
                chunk.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::ChainBase, int(c)));
                if (role == GeomRole::Range && skin) {
                    // The fold's validity, which the group step's decision
                    // shares with the join.
                    chunk.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::RevisionTransforms, id));
                }
                if (revision.chunked) {
                    // The whole point: this chunk waits for ITS influences
                    // and for nothing else. Not the fold, not the other
                    // chunks' joints -- it starts the instant its own are
                    // final and the fuse decides afterwards whether the work
                    // was wanted.
                    const RigExecBakedSlotDomain domain =
                        RigExecBakedOwnMatrixDomain(revision);
                    for (const int position : revision.chunks[k].key) {
                        const int slot =
                            revision.influenceSlots[size_t(position)];
                        if (slot >= 0) {
                            chunk.reads.push_back(
                                RigExecBakedOne(domain, slot));
                        }
                    }
                } else if (skin) {
                    // One chunk is the whole array, so there is nothing to
                    // speculate about: it skins against the folded table.
                    chunk.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::RevisionTransforms, id));
                }
                if (perGroup) {
                    // The pipelining edge: group k of the entering version,
                    // in its last writer's slot, and nothing of any join.
                    RigExecBakedDeclareGroupRead(B, int(c), r, k,
                                                 &chunk.reads);
                } else {
                    // Which buffer holds the entering points is decided by
                    // the previous fuse, so the read is that fuse's version.
                    DeclarePointVersionRead(B, entering, &chunk.reads);
                }
                chunk.writes.push_back(
                    RigExecBakedOne(RigExecBakedSlotDomain::RevisionOut,
                                    revision.chunkBase + int(k)));
            }
            {
                RigExecBakedStep &fuse = AddGeometryStep(
                    &B, RigExecBakedStepKind::RevisionFuse, id);
                // The one the defaultWeight check can emit.
                fuse.maxDiagnostics = 1;
                fuse.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::RevisionPacket, id));
                fuse.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::RevisionTransforms, id));
                fuse.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::ChainBase, int(c)));
                if (revision.weightObject >= 0) {
                    // The packet the MoverFailed arm consults. The assemble
                    // declares it too and the fuse waits on the assemble
                    // either way, so this changes no order -- but a step
                    // that touches a slot it did not declare is a bug even
                    // when a neighbour's declaration happens to cover it,
                    // and the next reader of either builder would have to
                    // rediscover why it was safe.
                    fuse.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::WeightPacket,
                        revision.weightObject));
                }
                // Its own chunks' ranges -- a join, its written groups'.
                // The entering points, which the whole-revision fallback and
                // a pass-through read, are a version like a chunk's. A join
                // reads no points, but it declares the entering version too:
                // that keeps the joins in chain order, so their lines keep
                // the unsplit chain's order in the canonical step order the
                // epilogue reports in, and hands it the predecessor's group
                // ids.
                if (role == GeomRole::Range) {
                    DeclareWrittenGroupReads(revision, &fuse.reads);
                } else {
                    fuse.reads.push_back(RigExecBakedRange(
                        RigExecBakedSlotDomain::RevisionOut,
                        revision.chunkBase,
                        revision.chunkBase + int(revision.chunks.size())));
                }
                DeclarePointVersionRead(B, entering, &fuse.reads);
                if (role == GeomRole::Whole) {
                    // A Whole fuse publishes every group: an applied group
                    // from its chunks, a refused one passed from the
                    // entering group it reads here.
                    for (size_t g = 0; g < groups; ++g) {
                        RigExecBakedDeclareGroupRead(B, int(c), r, g,
                                                     &fuse.reads);
                    }
                    fuse.writes.push_back(RigExecBakedRange(
                        RigExecBakedSlotDomain::RevisionOut,
                        revision.chunkBase + int(revision.chunks.size()),
                        revision.chunkBase + int(revision.chunks.size()) +
                            int(groups)));
                }
                fuse.writes.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::RevisionDone, id));
                fuse.writes.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::ChainDirty, id));
                B.revisionFuseStep[size_t(id)] = int(B.steps.size()) - 1;
            }
        }
        B.chainChunkEnd[c] = nextChunk;
        {
            RigExecBakedStep &status =
                AddGeometryStep(&B, RigExecBakedStepKind::ChainStatus, int(c));
            // One MoverFailed line per revision of the chain, which is the
            // sweep's whole output.
            status.maxDiagnostics = chain.revisions.size();
            status.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::ChainBase, int(c)));
            // Every revision's published status; the last one's
            // `currentSource` among them names version n, the final points.
            if (last > first) {
                status.reads.push_back(RigExecBakedRange(
                    RigExecBakedSlotDomain::RevisionDone, first, last));
            }
            status.writes.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::ChainPoints, int(c)));
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
            step.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::DerivedBase,id));
            step.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::ChainBase, int(c)));
            // A derived revision's binding carries phases like any other, so
            // it reads the versions they are bound to. Its fold runs inside
            // this step, so it reads the frame records an AtPrim transform
            // phase names too.
            DeclareRevisionPointReads(B, derived.revision, &step.reads);
            DeclareFrameRecordReads(derived.revision, &step.reads);
            DeclareMatrixReads(derived.revision, &step);
            step.writes.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::DerivedOut, id));
        }
    }
}

// The vertex groups of a range chain (bakedProgramImpl.h).

namespace {
// What RigExecBakedGroupAt answers for a group the chain does not have.
// Constant-initialized, so no reader meets an initialization guard.
const RigExecPointsRef<GfVec3f> kNoGroup{};
}  // namespace

const RigExecPointsRef<GfVec3f> &
RigExecBakedGroupAt(const RigExecBakedProgramImpl::GeomChain &chain,
                    size_t version, size_t g)
{
    const int writer = GroupWriter(chain, version, g);
    if (writer < 0) {
        return g < chain.baseGroups.size() ? chain.baseGroups[g].published
                                           : kNoGroup;
    }
    const auto &groups = chain.revisions[size_t(writer)].groups;
    return g < groups.size() ? groups[g].published : kNoGroup;
}

RigExecGroupSource
RigExecBakedGroupSourceAt(const RigExecBakedProgramImpl &B, int chain,
                          size_t version, size_t g)
{
    RigExecGroupSource source;
    if (chain < 0 || size_t(chain) >= B.chains.size()) {
        return source;
    }
    const RigExecBakedProgramImpl::GeomChain &geom = B.chains[size_t(chain)];
    const int writer = GroupWriter(geom, version, g);
    if (writer < 0) {
        source.slot = -1 - int64_t(g);
        source.version =
            g < geom.baseGroups.size() ? geom.baseGroups[g].version : 0;
        return source;
    }
    const RigExecBakedProgramImpl::GeomRevision &revision =
        geom.revisions[size_t(writer)];
    source.slot = RigExecBakedGroupSlot(
        B, B.chainRevisionBegin[size_t(chain)] + writer, g);
    source.version = g < revision.groups.size() ? revision.groups[g].version
                                                : 0;
    return source;
}

void
RigExecBakedDeclareGroupRead(const RigExecBakedProgramImpl &B, int chain,
                             size_t version, size_t g,
                             std::vector<RigExecBakedSlotRange> *reads)
{
    if (chain < 0 || size_t(chain) >= B.chains.size()) {
        return;
    }
    const int writer = GroupWriter(B.chains[size_t(chain)], version, g);
    const int slot =
        writer < 0 ? -1
                   : RigExecBakedGroupSlot(
                         B, B.chainRevisionBegin[size_t(chain)] + writer, g);
    reads->push_back(
        slot < 0 ? RigExecBakedOne(RigExecBakedSlotDomain::ChainBase, chain)
                 : RigExecBakedOne(RigExecBakedSlotDomain::RevisionOut, slot));
}

int
RigExecBakedGroupSlot(const RigExecBakedProgramImpl &B, int id, size_t g)
{
    if (id < 0 || size_t(id) >= B.revisionIndex.size()) {
        return -1;
    }
    const auto &[chain, r] = B.revisionIndex[size_t(id)];
    const RigExecBakedProgramImpl::GeomRevision &revision =
        B.chains[size_t(chain)].revisions[size_t(r)];
    if (g >= revision.groupWritten.size()) {
        return -1;
    }
    switch (revision.role) {
    case RigExecBakedRevisionRole::Range:
        return revision.groupWritten[g] ? revision.chunkBase + int(g) : -1;
    case RigExecBakedRevisionRole::Whole:
        return revision.chunkBase + int(revision.chunks.size()) + int(g);
    case RigExecBakedRevisionRole::Legacy:
        break;
    }
    return -1;
}

bool
RigExecBakedRolesStand(const RigExecBakedProgramImpl &B)
{
    // Worker-safe (the frozen worker calls it on its clone): reads sampled
    // leaves and the clone's own tables, compares tokens and paths by
    // identity, builds no token or path and posts no diagnostic.
    const auto reached = [&B](const std::vector<SdfPath> &hops) {
        for (const SdfPath &hop : hops) {
            if (B.interactiveOverrides) {
                for (const RigExecValueOverride &o : *B.interactiveOverrides) {
                    const TfToken &name =
                        o.attribute.IsEmpty() ? o.computation : o.attribute;
                    if (name == hop.GetNameToken() &&
                        o.prim == hop.GetPrimPath()) {
                        return true;
                    }
                }
            }
            for (const auto &entry : B.upstream) {
                if (entry.first == hop) {
                    return true;
                }
            }
        }
        return false;
    };
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        // Only this run's sample counts (the prologue just took it):
        // `haveBase` is the previous run's, false on a program's first run.
        if (chain.groupPointCount > 0 && chain.sampledHaveBase &&
            chain.sampledBase.size() != chain.groupPointCount) {
            return false;
        }
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
                 chain.revisions) {
            const RigExecBakedPathLeaves &leaves = revision.leaves;
            for (const RigExecBakedRolePin &pin : revision.pins) {
                switch (pin.kind) {
                case RigExecBakedRolePin::Kind::LeafToken:
                    if (leaves.sampled &&
                        leaves.Value<TfToken>(pin.leaf, TfToken()) !=
                            pin.token) {
                        return false;
                    }
                    break;
                case RigExecBakedRolePin::Kind::WeightDefaultZero:
                    if (pin.weightObject < 0 ||
                        size_t(pin.weightObject) >= B.weightObjects.size() ||
                        RigExecBakedLeaf(
                            B, B.weightObjects[size_t(pin.weightObject)]
                                   .defaultWeight) != 0.0f) {
                        return false;
                    }
                    break;
                case RigExecBakedRolePin::Kind::NoLayoutOverride:
                    if (pin.leaf >= 0 && size_t(pin.leaf) < leaves.hops.size() &&
                        reached(leaves.hops[size_t(pin.leaf)])) {
                        return false;
                    }
                    break;
                }
            }
        }
    }
    return true;
}

// One frame, geometry half.
// The packet is assembled by the same RigExecAssembleParameters the dynamic
// path calls and the kernel is the same shared kernel; only the VdfNetwork
// around them is baked away, so the accounting it performed -- a revision
// runs when its inputs changed, and not otherwise -- is performed here
// instead, by the four steps a revision is made of.

namespace {

/// The points of version \p version of \p chain (RigExecBakedPointVersion):
/// 0 is the authored base, v > 0 what revision v - 1's fuse left.
///
/// Not a buffer of its own: revision v - 1's `currentSource` names the
/// revision whose output buffer holds the value, so a pass-through version
/// names an earlier revision's buffer, or the base when it is -1.
void
PointsAt(const RigExecBakedProgramImpl::GeomChain &chain, size_t version,
         const GfVec3f **points, size_t *count)
{
    const int source =
        version == 0 ? -1 : chain.revisions[version - 1].currentSource;
    if (source < 0) {
        *points = chain.lastBase.cdata();
        *count = chain.lastBase.size();
        return;
    }
    *points = chain.revisions[size_t(source)].output.data();
    *count = chain.revisions[size_t(source)].output.size();
}

bool
SamePoints(const std::vector<GfVec3f> &a, const std::vector<GfVec3f> &b)
{
    return RigExecBakedSamePoints(a.data(), a.size(), b.data(), b.size());
}

bool
SamePoints(const VtVec3fArray &a, const VtVec3fArray &b)
{
    return RigExecBakedSamePoints(a.cdata(), a.size(), b.cdata(), b.size());
}

/// Makes \p output hold \p staging's points, copying only the blocks whose
/// bytes differ, and returns whether any did (a size change included). A
/// chunked skin's ranges stay in staging, where a chunk that sat a run out
/// keeps its range, so its fuse cannot swap the buffers.
bool
CopyMovedPoints(const std::vector<GfVec3f> &staging,
                std::vector<GfVec3f> *output)
{
    if (output->size() != staging.size()) {
        output->assign(staging.begin(), staging.end());
        return true;
    }
    constexpr size_t kBlock = 1024;
    bool moved = false;
    for (size_t begin = 0; begin < staging.size(); begin += kBlock) {
        const size_t count = std::min(kBlock, staging.size() - begin);
        if (std::memcmp(staging.data() + begin, output->data() + begin,
                        count * sizeof(GfVec3f)) != 0) {
            std::copy(staging.begin() + long(begin),
                      staging.begin() + long(begin + count),
                      output->begin() + long(begin));
            moved = true;
        }
    }
    return moved;
}

/// The influence matrices of \p revision, out of the tables the pose half's
/// ProviderMatrix steps published. Returns whether the table MOVED.
///
/// The compare is here rather than in the packet comparison because a skin
/// revision's packet no longer carries the table: the half of the executed
/// decision that used to read `skinTransforms == o.skinTransforms` reads this
/// instead, and it is the same comparison over the same values.
bool
FoldInfluences(const RigExecBakedProgramImpl &B,
               RigExecBakedProgramImpl::GeomRevision *revision)
{
    bool changed = false;
    revision->haveTransform = revision->transformSlot >= 0;
    if (revision->haveTransform) {
        revision->transform =
            revision->finalPhase
                ? B.finalMatrix[size_t(revision->transformSlot)]
                : B.baseMatrix[size_t(revision->transformSlot)];
    }
    // rigExec:space, out of the same table at the same phase, so this reads
    // exactly what the dynamic path's carry tap reads. Held on the revision
    // for the assemble, which may not read the tables itself: the fold is
    // the step that declared them.
    revision->haveCarry = revision->carrySpaceSlot >= 0;
    if (revision->haveCarry) {
        const GfMatrix4d &carry =
            revision->finalPhase
                ? B.finalMatrix[size_t(revision->carrySpaceSlot)]
                : B.baseMatrix[size_t(revision->carrySpaceSlot)];
        if (revision->carry != carry) {
            revision->carry = carry;
            changed = true;
        }
    }
    // A geometry-domain constraint's delta, if one was measured for this
    // mover: the pose walk solved the constraint and stashed the map from the
    // target's authored transform to the solved one, and THIS is the Matrix
    // revision that same constraint contributes. Find-guarded, and here
    // rather than in the assemble because the transform table is what this
    // step declares -- the assemble only READS it, and a step that writes a
    // slot it declared as a read is the declaration the executor trusts
    // being wrong.
    // The dynamic path applies it before the final-phase substitution rather
    // than after. The two orders can only disagree for a revision that has
    // both a bound transform provider and a delta, which cannot arise: a
    // geometry-domain constraint's binding.transform is empty -- that is what
    // makes the delta the only source of the matrix.
    if (revision->constraintDelta >= 0 &&
        B.deltaPresent[size_t(revision->constraintDelta)]) {
        revision->transform = B.deltaValues[size_t(revision->constraintDelta)];
        revision->haveTransform = true;
    }
    // A read phase naming a POINT IN THE POSE WALK, which is the general
    // form `base`, `preceding` and `final` abbreviate: the provider's matrix
    // as it stood immediately after one named constraint, out of the frame
    // records bound at Build instead of out of the dense tables.
    // AFTER the delta, exactly as the dynamic path orders the two, and the
    // two can no more both apply here than they can there: an AtPrim phase
    // needs a bound rigExec:transform to name a frame chain of, and a
    // geometry-domain constraint's binding.transform is empty -- which is
    // what makes its delta the only source of the matrix.
    // The first valid record of the list answers: the last record the walk
    // made under the phase's prim. A list with none valid leaves the
    // dense-table value standing and says nothing, as the dynamic path does:
    // every named constraint under the prim declined to record this run (a
    // geometry-domain exit, an unusable frame).
    const auto phased = [&B](const std::vector<int> &records,
                             GfMatrix4d *matrix) {
        for (const int record : records) {
            if (B.frameMatrixValid[size_t(record)]) {
                *matrix = B.frameMatrix[size_t(record)];
                return true;
            }
        }
        return false;
    };
    const bool atPrim = revision->binding.transformPhase.kind ==
                        RigExecReadPhaseKind::AtPrim;
    if (atPrim && phased(revision->transformRecords, &revision->transform)) {
        // Set rather than left alone: the dynamic path binds its matrix
        // pointer whenever the store answered, whatever the tap held.
        revision->haveTransform = true;
    }
    const bool hasReference = revision->op == RigExecRevisionOp::Matrix &&
                              !revision->influenceSlots.empty();
    const auto referenceMatrix = [&B, revision](size_t index) {
        const size_t slot = size_t(revision->influenceSlots[index]);
        return revision->finalPhase ? B.finalMatrix[slot] : B.baseMatrix[slot];
    };
    if (revision->haveTransform) {
        GfMatrix4d space(1.0), reference(1.0), referenceSpace(1.0);
        const bool hasSpace=revision->transformSpaceSlot>=0;
        if(hasSpace)space=revision->finalPhase
            ? B.finalMatrix[size_t(revision->transformSpaceSlot)]
            : B.baseMatrix[size_t(revision->transformSpaceSlot)];
        if(hasReference)reference=referenceMatrix(0);
        const bool hasReferenceSpace=hasReference && revision->influenceSlots.size()>1;
        if(hasSpace && hasReferenceSpace)referenceSpace=referenceMatrix(1);
        revision->transform=RigExecGeometryMatrixInPointFrame(revision->transform,
            hasSpace?&space:nullptr,hasReference?&reference:nullptr,
            hasReferenceSpace?&referenceSpace:nullptr,revision->posedPoints,
            revision->haveCarry?&revision->carry:nullptr);
    }
    for (size_t k = 0; k < revision->influenceSlots.size(); ++k) {
        const size_t slot = size_t(revision->influenceSlots[k]);
        GfMatrix4d matrix = revision->finalPhase ? B.finalMatrix[slot]
                                                 : B.baseMatrix[slot];
        // Every influence shares the one declared phase, so the substitution
        // is per ENTRY and the provider is the entry's own. A skin never
        // gets here: compile refuses an AtPrim phase whose binding.transform
        // is empty, as a skin's is. A matrix mover's influences are its
        // reference providers; the Matrix op does not read them, but its
        // packet carries this table into the executed decision, as the
        // walk's fold does.
        if (atPrim && k < revision->influenceRecords.size()) {
            phased(revision->influenceRecords[k], &matrix);
        }
        if (revision->influences[k] != matrix) {
            revision->influences[k] = matrix;
            changed = true;
        }
    }
    return changed;
}

/// The revision's own folded table, in the forms the kernels want it in.
///
/// What a chunk of a revision cut into ONE chunk skins against, and what the
/// fuse falls back to when the partition no longer describes the layout. A
/// chunked revision's chunks keep their own tables beside these and never
/// read them; the fold writes them anyway, because they are part of the
/// RevisionTransforms slot and the fuse may only READ that slot.
RigExecSkinTransformsView
WholeTransformsView(RigExecBakedProgramImpl::GeomRevision *revision)
{
    RigExecSkinTransformsView view;
    view.transforms = revision->influences.data();
    view.transformCount = revision->influences.size();
    if (!revision->rows.empty()) {
        view.rows = revision->rows.data();
    }
    if (!revision->palette.empty()) {
        view.palette = revision->palette.data();
        view.paletteSize = revision->palette.size();
    }
    return view;
}

/// Narrows and splits \p revision's folded table for the method its packet
/// names, once per run rather than once per range.
///
/// Part of what InfluenceFold writes, for every skin revision and not only
/// an unchunked one: the whole-array fallback the fuse falls back to reads
/// these, and a step may not write a slot it declared as a read.
void
FoldTransformForms(RigExecBakedProgramImpl::GeomRevision *revision,
                   bool useSimd)
{
    const size_t count = revision->influences.size();
    if (revision->parameters.skinningMethod == _tokens->dualQuaternion) {
        revision->rows.clear();
        revision->palette.resize(count + 1);
        for (size_t t = 0; t < count; ++t) {
            revision->palette[t] =
                RigExecScaledDualQuatFromMatrix(revision->influences[t]);
        }
        // The trailing entry is the weight complement's identity influence,
        // exactly as RigExecSkinDualQuatPalette appends it.
        revision->palette[count] = RigExecScaledDualQuat();
        return;
    }
    revision->palette.clear();
    if (!useSimd) {
        revision->rows.clear();
        return;
    }
    revision->rows.resize(count * RigExecSkinRowStride);
    for (size_t t = 0; t < count; ++t) {
        RigExecNarrowSkinRows(revision->influences[t],
                              &revision->rows[t * RigExecSkinRowStride]);
    }
}

/// Copies \p chunk's OWN influences out of the matrix slots, keeping the
/// narrowed and split forms beside them, and records whether any moved.
///
/// Only |key| entries, never the whole table: that is what makes a chunk's
/// per-run cost proportional to the joints its vertices actually reference.
/// The entries outside the key stay identity, and linear blend skinning
/// reaches an entry only through an index of one of the chunk's own vertices,
/// so they are never read.
void
GatherChunkTransforms(const RigExecBakedProgramImpl &B,
                      const RigExecBakedProgramImpl::GeomRevision &revision,
                      RigExecBakedProgramImpl::GeomChunk *chunk)
{
    const bool dualQuat =
        revision.parameters.skinningMethod == _tokens->dualQuaternion;
    const size_t count = chunk->transforms.size();
    const bool splitAlready = chunk->palette.size() == count + 1;
    for (const int position : chunk->key) {
        const int slot = revision.influenceSlots[size_t(position)];
        if (slot < 0) {
            continue;
        }
        const GfMatrix4d &matrix = revision.finalPhase
                                       ? B.finalMatrix[size_t(slot)]
                                       : B.baseMatrix[size_t(slot)];
        if (chunk->transforms[size_t(position)] == matrix) {
            continue;
        }
        chunk->transforms[size_t(position)] = matrix;
        chunk->keyChanged = true;
        RigExecNarrowSkinRows(
            matrix, &chunk->rows[size_t(position) * RigExecSkinRowStride]);
        if (splitAlready) {
            chunk->palette[size_t(position)] =
                RigExecScaledDualQuatFromMatrix(matrix);
        }
    }
    if (dualQuat && !splitAlready) {
        // First run of a dual-quaternion revision: the whole table at once,
        // which is also what keeps the entries outside the key -- identity,
        // and never read -- in step with the matrices beside them.
        chunk->palette.resize(count + 1);
        for (size_t t = 0; t < count; ++t) {
            chunk->palette[t] =
                RigExecScaledDualQuatFromMatrix(chunk->transforms[t]);
        }
        chunk->palette[count] = RigExecScaledDualQuat();
    }
}

/// The view onto \p chunk's own tables.
RigExecSkinTransformsView
ChunkTransformsView(const RigExecBakedProgramImpl::GeomChunk &chunk,
                    bool useSimd)
{
    RigExecSkinTransformsView view;
    view.transforms = chunk.transforms.data();
    view.transformCount = chunk.transforms.size();
    if (useSimd && !chunk.rows.empty()) {
        view.rows = chunk.rows.data();
    }
    if (!chunk.palette.empty()) {
        view.palette = chunk.palette.data();
        view.paletteSize = chunk.palette.size();
    }
    return view;
}

/// The skin kernel's validation over the half RevisionStatic holds: the
/// packet, the layout against the entering count and the envelope it
/// resolved (the fold's `influencesValid` is the matrix half, which the fuse
/// ANDs in). One definition for the revision's decision and every chunk's
/// guard.
bool
SkinPacketIsUsable(const RigExecBakedProgramImpl::GeomRevision &revision)
{
    return revision.parameters.valid && revision.parameters.kind == "skin" &&
           revision.layoutUsable && revision.envelopeOk;
}

/// A skin revision's apply-or-fail answer from that half: past it a linear
/// blend cannot fail, while a dual-quaternion blend can still be degenerate
/// at a vertex, which only its chunks see.
RigExecRevisionAcceptance
SkinAcceptance(const RigExecBakedProgramImpl::GeomRevision &revision)
{
    if (!SkinPacketIsUsable(revision)) {
        return RigExecRevisionAcceptance::Refuses;
    }
    switch (RigExecSkinMethodOf(revision.parameters)) {
    case RigExecSkinMethod::ClassicLinear:
        return RigExecRevisionAcceptance::Applies;
    case RigExecSkinMethod::DualQuaternion:
        return RigExecRevisionAcceptance::Deferred;
    case RigExecSkinMethod::Unknown:
        break;
    }
    return RigExecRevisionAcceptance::Refuses;
}

/// One vertex range of a skin revision: seed the revision's own output buffer
/// from the preceding points, deform in place, blend the envelope back.
///
/// The three pieces RigExecRunRevisionKernel performs for the whole array,
/// performed over a range -- with every whole-array decision (the packet
/// check, the layout validation, the envelope's resolution, the size check)
/// already made by the steps that own them.
///
/// \p whole runs the full-range kernel instead, which is the same body
/// inside the kernel's own parallel loop; a revision cut into one chunk uses
/// it so that an unpartitioned mesh keeps the threading it has today.
/// \p target is the buffer written, at the full count: staging for a chunk.
bool
SkinRange(RigExecBakedProgramImpl::GeomRevision *revision,
          const GfVec3f *preceding, const RigExecSkinTransformsView &view,
          size_t begin, size_t end, bool whole, bool useSimd,
          std::vector<GfVec3f> *target)
{
    std::vector<GfVec3f> &out = *target;
    std::copy(preceding + begin, preceding + end, out.begin() + long(begin));
    if (whole) {
        if (!RigExecApplySkinKernelWithTransforms(revision->parameters, view,
                                                  &out, useSimd)) {
            return false;
        }
    } else if (!RigExecApplySkinKernelRange(revision->parameters, view, begin,
                                            end, &out, useSimd)) {
        return false;
    }
    // "Apply once", over the same range: the envelope was resolved at the
    // full count by RevisionStatic, because resolving it is atomic over the
    // whole array, and every array here is indexed absolutely. The whole
    // array goes through the kernel's own split for the reason the skinning
    // above does: an unpartitioned mesh keeps the threading it has today.
    if (!revision->fullStrength) {
        if (whole) {
            // `whole` means the whole array -- the full-range kernel above
            // ignores the bounds -- so begin is 0 and the count is `end`.
            RigExecBlendEnvelopeAll(preceding, revision->envelope.data(), end,
                                    out.data());
        } else {
            RigExecBlendEnvelopeRange(preceding, revision->envelope.data(),
                                      begin, end, out.data());
        }
    }
    return true;
}

/// The test capture (RigExecBakedProgramTesting::CapturePointReads): the
/// binding's answer for \p binding as its reader took it this run.
void
CapturePointRead(RigExecBakedProgramImpl *program,
                 const RigExecBakedPointsBinding &binding,
                 const GfVec3f *points, size_t count, bool answered)
{
    RigExecBakedProgramImpl &B = *program;
    if (!B.capturePointReads || binding.id < 0 ||
        size_t(binding.id) >= B.pointCaptures.size()) {
        return;
    }
    RigExecBakedPointCapture &capture = B.pointCaptures[size_t(binding.id)];
    capture.read = true;
    capture.bindingAnswered = answered;
    capture.bound =
        answered ? VtVec3fArray(points, points + count) : VtVec3fArray();
}

}  // namespace

void
RigExecBakedVersionPoints(const RigExecBakedProgramImpl::GeomChain &chain,
                          size_t version, std::vector<GfVec3f> *scratch,
                          const GfVec3f **points, size_t *count)
{
    // The base, and every version of a chain without groups: one buffer.
    if (version == 0 || chain.groupBounds.size() < 2) {
        PointsAt(chain, version, points, count);
        return;
    }
    // Group g of the version is the ref its writer published. Consecutive
    // slices of one owner are one buffer, read in place; anything else is
    // gathered into the reader's own scratch.
    const size_t groups = chain.groupBounds.size() - 1;
    size_t total = 0;
    bool contiguous = true;
    const void *owner = nullptr;
    const GfVec3f *start = nullptr;
    const GfVec3f *next = nullptr;
    for (size_t g = 0; g < groups; ++g) {
        const RigExecPointsRef<GfVec3f> &ref =
            RigExecBakedGroupAt(chain, version, g);
        total += ref.count;
        if (!contiguous || ref.count == 0) {
            continue;
        }
        if (!start) {
            start = ref.data;
            owner = ref.owner.get();
        } else if (ref.owner.get() != owner || ref.data != next) {
            contiguous = false;
            continue;
        }
        next = ref.data + ref.count;
    }
    if (contiguous) {
        *points = start;
        *count = total;
        return;
    }
    scratch->resize(total);
    GfVec3f *out = scratch->data();
    for (size_t g = 0; g < groups; ++g) {
        const RigExecPointsRef<GfVec3f> &ref =
            RigExecBakedGroupAt(chain, version, g);
        if (ref.count) {
            std::memcpy(out, ref.data, ref.count * sizeof(GfVec3f));
        }
        out += ref.count;
    }
    *points = scratch->data();
    *count = total;
}

bool
RigExecBakedResolvePoints(const RigExecBakedProgramImpl &B,
                          const RigExecBakedPointsBinding &binding,
                          const GfVec3f **points, size_t *count)
{
    for (const RigExecBakedPointVersion &candidate : binding.candidates) {
        const RigExecBakedProgramImpl::GeomChain &chain =
            B.chains[size_t(candidate.chain)];
        if (binding.finalRead) {
            if(!chain.haveResult)continue;
            *points = chain.result.cdata();
            *count = chain.result.size();
        } else {
            if(!chain.haveBase)continue;
            // A version cut into groups gathers into the binding's own
            // buffer, which only its one reading step uses.
            RigExecBakedVersionPoints(chain, size_t(candidate.version),
                                      &binding.gather, points, count);
        }
        return true;
    }
    return false;
}

void
RigExecBakedOverlayPointReads(RigExecBakedProgramImpl *program,
                              RigExecBakedProgramImpl::GeomRevision *revision,
                              const RigExecResolvedInputs &resolved,
                              std::vector<std::string> *diagnostics)
{
    if (revision->binding.phases.empty()) {
        return;
    }
    RigExecBakedProgramImpl &B = *program;
    revision->revisionInputs.SetChainedBase(&resolved);
    for (const RigExecBakedPointsBinding &binding : revision->pointBindings) {
        const GfVec3f *points = nullptr;
        size_t count = 0;
        const bool answered =
            RigExecBakedResolvePoints(B, binding, &points, &count);
        CapturePointRead(&B, binding, points, count, answered);
        if (answered) {
            // A final read shares the chain's published buffer, as the
            // walk's record of it does.
            revision->revisionInputs.SetProperty(
                binding.input,
                binding.finalRead
                    ? VtValue(B.chains[size_t(binding.candidates[0].chain)]
                                  .result)
                    : VtValue(VtVec3fArray(points, points + count)));
        } else if (binding.diagnoseMiss) {
            // A `preceding` tail is silent, as it is in the dynamic walk.
            diagnostics->push_back(binding.missDiagnostic);
        }
    }
}

namespace {

// WHICH points a revision is assembled against, stated once because the two
// callers below pass different ones and the difference is invisible until an
// operator reads them:
//  * a CHAIN revision gets the chain's AUTHORED base -- the attribute as the
//    stage holds it -- for every revision of the chain, not the running
//    value. The dynamic walk reads the base once per target and hands that
//    same array to every revision, so a lattice's rest points and a
//    volumeCorrect's reference volume are measured against the mesh as
//    authored however deep in the chain they sit.
//  * a DERIVED revision gets the chain's FINAL points, which is what it is
//    for: recomputeNormals and recomputeExtent take auxPoints = basePoints
//    and must describe the geometry as published.
// Taken as a range rather than a vector because the two callers hold the
// points in different containers and RigExecProviderValues copies them
// anyway.
//
// The provider values both assemblies take: everything but the stage reads.
RigExecProviderValues
RevisionValues(RigExecBakedProgramImpl &B,
               RigExecBakedProgramImpl::GeomRevision *revision,
               const GfVec3f *basePoints, size_t basePointCount)
{
    RigExecProviderValues values;
    // The fold decided whether there is a matrix at all -- a bound transform
    // provider, or a geometry-domain constraint's delta -- and wrote it.
    // A SKIN revision is assembled without one, and that is not an omission:
    // its static step runs BEFORE its fold, precisely so its chunks wait for
    // their own joints rather than for the rig's, so the fold's matrix here
    // would be the one last run measured. Nothing reads it --
    // RigExecAssembleSkinParameters takes no transform, which is also how
    // the dynamic path treats a delta landing on a skin mover -- so the
    // packet is assembled without a matrix rather than with a stale one.
    if (revision->haveTransform && revision->op != RigExecRevisionOp::Skin) {
        values.transform = &revision->transform;
    }
    // The carry the fold read beside the transform, for the wire assembler.
    // A skin's fold runs after its assemble, but a skin never names one.
    if (revision->haveCarry) {
        values.carry = &revision->carry;
    }
    // A skin revision's packet carries the IDENTITY table and never the
    // folded one: it is assembled before the matrices are folded, which is
    // what lets its chunks start on their own joints. Every other operation's
    // packet carries what the fold wrote -- which is nothing today, because a
    // skin is the only operation that binds influences at all.
    const std::vector<GfMatrix4d> &table =
        revision->op == RigExecRevisionOp::Skin ? revision->packetInfluences
                                                : revision->influences;
    if (!table.empty()) {
        values.influenceTransforms = &table;
    }
    // The weight object's packet, shared with every other mover that binds
    // it and built by its own step earlier in the frame. A mover copies EXEC
    // here (see bakedWeights.cpp): this is the packet the tap would have
    // carried, and `inputs:defaultWeight` is deliberately NOT consulted
    // beside it -- the assembler falls back to that scalar only when no
    // packet is bound at all.
    if (revision->weightObject >= 0) {
        values.weights = revision->weightCurrentPhase
                             ? &revision->currentPhasePacket
                             : &B.weightPackets[size_t(revision->weightObject)];
    }
    // Only the operations that measure against the authored base read it; a
    // matrix, skin or wire revision never does, and copying a whole body's
    // points per revision per frame was most of what such a step cost.
    if (revision->op != RigExecRevisionOp::Matrix &&
        revision->op != RigExecRevisionOp::Skin &&
        revision->op != RigExecRevisionOp::Wire) {
        if (revision->op == RigExecRevisionOp::External) {
            // API4 currently exposes an owning vector view to plugin code.
            if (basePointCount) values.basePoints.assign(basePoints,basePoints+basePointCount);
        } else {
            values.borrowsBasePoints = true;
            values.borrowedBasePoints = basePoints;
            values.borrowedBaseCount = basePointCount;
        }
    }
    // The driver solver's aggregate, which is what the dynamic path's
    // per-revision tap resolves to: exec computes one and the pose walk then
    // OVERRIDES the tap with its own solve, so the program's table -- the
    // same solve, by the same kernel -- is the authoritative value on both
    // paths. Declared as a read of the Aggregate slot, which is the edge
    // from the solver's Solve step to this revision.
    if (revision->driverFramesSolver >= 0) {
        values.driverFrames =
            &B.aggregates[size_t(revision->driverFramesSolver)];
    }
    // Epoch-fixed layouts were adopted in the PROLOGUE from the SkinTopology
    // op, which builds them by the dynamic cache's own rules
    // (RigExecBuildSkinTopology) -- so the two paths cannot hold different
    // arrays, and no step body reads one off the stage.
    if (revision->topologyResolved) {
        values.skinTopology = &revision->topology;
    }
    return values;
}

// The blend channels. Gathered here rather than inside the assembler
// because the dynamic walk gathers them here too, and the ORDER is the
// whole contract: channels in `binding.blendInputs` order, which compile
// sorted, and each channel's samples stable-sorted by activation. Float
// addition is not associative, so a different order is a different last
// bit of every point.
// The reads are the generation-wide resolved inputs' and NOT this
// revision's phase overlay's: a sample's own phase is looked up directly,
// which is what lets one blend sample read another chain while the rest of
// the channel reads the stage. \p read answers each one: the stage gather
// through the resolved inputs, the leaf gather from the leaves the prologue
// sampled through them. \p record writes each dense sample's `lastPoints`.
template <class Read>
void
GatherBlendChannels(RigExecBakedProgramImpl &B,
                    RigExecBakedProgramImpl::GeomRevision *revision,
                    const Read &read, bool record,
                    RigExecProviderValues *values)
{
    if (revision->blendChannels.empty()) {
        return;
    }
    std::vector<RigExecBlendChannel> channels;
    channels.reserve(revision->blendChannels.size());
    for (size_t c = 0; c < revision->blendChannels.size(); ++c) {
        RigExecBakedProgramImpl::GeomBlendChannel &bound =
            revision->blendChannels[c];
        RigExecBlendChannel channel;
        // A pose-driven weight is this run's slot -- unless an override
        // stands on the weight itself, which the dynamic walk's lookup
        // finds before it follows the connection and which the resolved
        // inputs therefore answer here too.
        if (bound.poseWeight >= 0 && !read.WeightHeld(bound)) {
            channel.weight = B.poseWeights[size_t(bound.poseWeight)];
        } else {
            read.Weight(bound, &channel.weight);
        }
        for (size_t s = 0; s < bound.samples.size(); ++s) {
            auto &boundSample = bound.samples[s];
            RigExecBlendSampleData sample;
            read.Activation(boundSample, &sample.activation);
            if (!boundSample.blendShape.IsEmpty()) {
                // Sparse: the shape the prologue resolved, shared by
                // pointer. The packet compares layouts by pointer, so an
                // unchanged sample costs one compare and not an array.
                sample.layout = boundSample.layout;
                channel.samples.push_back(std::move(sample));
                continue;
            }
            // A phased sample reads the version it is bound to, and the
            // resolved input when that chain read no base: silently,
            // as the dynamic gather falls back.
            const GfVec3f *phased = nullptr;
            size_t phasedCount = 0;
            const bool answered =
                boundSample.pointBinding.id >= 0 &&
                RigExecBakedResolvePoints(B, boundSample.pointBinding,
                                          &phased, &phasedCount);
            if (boundSample.pointBinding.id >= 0) {
                CapturePointRead(&B, boundSample.pointBinding, phased,
                                 phasedCount, answered);
            }
            if (record) {
                // The consumer needs an immutable view; export retention
                // changes only when the actual sample bytes/count changed.
                if (answered) {
                    const bool same = boundSample.lastPoints.size() == phasedCount &&
                        (!phasedCount || !std::memcmp(boundSample.lastPoints.data(),
                            phased,phasedCount*sizeof(GfVec3f)));
                    if (!same) {
                        if (phasedCount) boundSample.lastPoints.assign(phased,phased+phasedCount);
                        else boundSample.lastPoints.clear();
                    }
                } else {
                    read.Points(boundSample,&boundSample.lastPoints);
                }
                sample.borrowsPoints = true;
                sample.borrowedPoints = boundSample.lastPoints.data();
                sample.borrowedCount = boundSample.lastPoints.size();
            } else if (answered) {
                if (phasedCount) sample.points.assign(phased,phased+phasedCount);
            } else {
                read.Points(boundSample,&sample.points);
            }
            channel.samples.push_back(std::move(sample));
        }
        std::stable_sort(channel.samples.begin(), channel.samples.end(),
                         [](const RigExecBlendSampleData &a,
                            const RigExecBlendSampleData &b) {
                             return a.activation < b.activation;
                         });
        channels.push_back(std::move(channel));
    }
    // A structural failure leaves blendDeltas empty, which is what makes
    // the assembled packet invalid -- the same atomic MoverFailed
    // pass-through the kernel produces.
    if (!RigExecGeometryBlendDeltas(channels, values->BasePointData(),values->BasePointCount(),
                                 &values->blendDeltas)) {
        values->blendDeltas.clear();
    }
}

// The blend gather's reads, off the stage through the resolved inputs.
struct _StageBlendReads {
    const RigExecResolvedInputs &R;
    UsdTimeCode time;

    bool WeightHeld(
        const RigExecBakedProgramImpl::GeomBlendChannel &bound) const {
        return R.Find(bound.weightPath) != nullptr;
    }
    void Weight(const RigExecBakedProgramImpl::GeomBlendChannel &bound,
                float *out) const {
        R.GetAttribute(bound.weight, time, out);
    }
    void Activation(
        const RigExecBakedProgramImpl::GeomBlendChannel::Sample &sample,
        float *out) const {
        R.GetAttribute(sample.activation, time, out);
    }
    void Points(const RigExecBakedProgramImpl::GeomBlendChannel::Sample &sample,
                std::vector<GfVec3f> *out) const {
        VtVec3fArray points;
        R.GetAttribute(sample.points, time, &points);
        out->assign(points.begin(), points.end());
    }
};

// The same reads, from the revision's path leaves.
struct _LeafBlendReads {
    const std::vector<VtValue> &values;
    template<class T> T Value(int key,const T &fallback) const {
        return key >= 0 && size_t(key) < values.size() && values[size_t(key)].IsHolding<T>()
            ? values[size_t(key)].UncheckedGet<T>() : fallback;
    }

    bool WeightHeld(
        const RigExecBakedProgramImpl::GeomBlendChannel &bound) const {
        return Value<bool>(bound.weightHeldLeaf, false);
    }
    void Weight(const RigExecBakedProgramImpl::GeomBlendChannel &bound,
                float *out) const {
        *out = Value<float>(bound.weightLeaf, *out);
    }
    void Activation(
        const RigExecBakedProgramImpl::GeomBlendChannel::Sample &sample,
        float *out) const {
        *out = Value<float>(sample.activationLeaf, *out);
    }
    void Points(const RigExecBakedProgramImpl::GeomBlendChannel::Sample &sample,
                std::vector<GfVec3f> *out) const {
        const VtVec3fArray points =
            Value<VtVec3fArray>(sample.pointsLeaf, VtVec3fArray());
        const bool same = out->size() == points.size() &&
            (points.empty() || !std::memcmp(out->data(),points.cdata(),
                                           points.size()*sizeof(GfVec3f)));
        if (!same) {
            if (points.empty()) out->clear();
            else out->assign(points.cbegin(),points.cend());
        }
    }
};

// The packet off the stage, through \p R (or the revision's phase overlay
// over it), with RigExecAssembleParameters: a plugin revision bound to a
// value the region computes, and the shadow test's reference for the rest.
RigExecMoverParameters
AssembleRevisionFromStage(RigExecBakedProgramImpl &B,
                          const RigExecResolvedInputs &R,
                          RigExecBakedProgramImpl::GeomRevision *revision,
                          const GfVec3f *basePoints, size_t basePointCount,
                          UsdTimeCode time,
                          std::vector<std::string> *diagnostics, bool record)
{
    RigExecProviderValues values =
        RevisionValues(B, revision, basePoints, basePointCount);
    if (values.borrowsBasePoints) {
        values.CopyBasePoints(&values.basePoints);
        values.borrowsBasePoints = false;
    }
    values.resolved = &R;
    // One overlay per REVISION: the generation-wide resolved inputs, plus
    // whatever this revision's declared phases resolve to through their
    // bindings. The assembler reads inputs by path and never learns a phase
    // exists, which is what lets a phase apply to any input. Built only for
    // a revision that declares one, and owned by that revision, because one
    // buffer shared by the walk is state two steps could be inside at once.
    if (!revision->binding.phases.empty()) {
        RigExecBakedOverlayPointReads(&B, revision, R, diagnostics);
        values.resolved = &revision->revisionInputs;
    }
    // Independent host-side reference keeps the original stage Get route;
    // it must not reuse the production leaf-built layout handle.
    for (auto &channel : revision->blendChannels) for (auto &sample : channel.samples) {
        if (sample.blendShape.IsEmpty()) continue;
        auto layout = std::make_shared<RigExecBlendSampleLayout>();
        RigExecResolveBlendSampleLayout(B.stage,sample.blendShape,basePointCount,layout.get());
        sample.layout = std::move(layout);
    }
    GatherBlendChannels(B, revision, _StageBlendReads{R, time}, record,
                        &values);
    return RigExecAssembleParameters(revision->moverPrim, revision->op,
                                     revision->binding, values, time);
}

}  // namespace

RigExecMoverParameters
RigExecBakedAssembleFromLeaves(
    RigExecBakedProgramImpl *program,
    RigExecBakedProgramImpl::GeomRevision *revision,
    const GfVec3f *basePoints, size_t basePointCount,
    std::vector<std::string> *diagnostics, std::vector<std::string> *missing,
    size_t layoutPointCount)
{
    RigExecBakedProgramImpl &B = *program;
    if (layoutPointCount == size_t(-1)) layoutPointCount = basePointCount;
    RigExecProviderValues values =
        RevisionValues(B, revision, basePoints, basePointCount);
    // Two arrays the packet takes over rather than copies: the summed blend
    // deltas `values` owns, and a Range lattice's held rest (the chain base
    // at the version RevisionStatic checked, `restBaseHeld`), which the last
    // packet gives up because this assembly replaces it.
    values.lendBlendDeltas = true;
    if (revision->op == RigExecRevisionOp::Lattice && revision->restBaseHeld &&
        revision->parameters.restPoints.size() == basePointCount) {
        values.retainedRest = &revision->parameters.restPoints;
    }
    RigExecRevisionLeafView view;
    view.decl = &revision->leaves.decl;
    view.values = &RigExecBakedResolvePathLeaves(B,revision->leaves);
    view.missing = missing;
    if (revision->op == RigExecRevisionOp::External) {
        view.external = &revision->externalPayload;
    }
    // The phase overlay, as the stage assembly builds it: a phased points
    // read takes the version it is bound to before its leaf.
    if (!revision->binding.phases.empty()) {
        RigExecBakedOverlayPointReads(&B, revision, *B.resolvedInputs,
                                      diagnostics);
        view.phased = &revision->revisionInputs;
    }
    // Derived normalization consumes only the declared sparse RawDefault
    // leaves and this target's raw base cardinality, inside the selected body.
    // A layout is a pure function of those, so it is rebuilt only when the
    // leaves' content versions or the point count moved since it was built;
    // a leaf a walk or a produced value answers is not keyed so, and
    // rebuilds every time.
    const RigExecBakedPathLeaves &sources = revision->leaves;
    const auto rawVersion = [&sources, &view](int k, uint64_t *version) {
        const auto at = [k](const std::vector<int> &v) {
            return size_t(k) < v.size() ? v[size_t(k)] : -1;
        };
        if (k < 0 || size_t(k) >= sources.versions.size() ||
            size_t(k) >= view.values->size() ||
            at(sources.walks) >= 0 || at(sources.exactVersions) >= 0 ||
            at(sources.exactRecordIndices) >= 0) {
            return false;
        }
        *version = sources.versions[size_t(k)];
        return true;
    };
    for (auto &channel : revision->blendChannels) for (auto &sample : channel.samples) {
        if (sample.blendShape.IsEmpty()) continue;
        uint64_t offsetsVersion = 0, indicesVersion = 0;
        const bool keyed = rawVersion(sample.offsetsLeaf, &offsetsVersion) &&
                           rawVersion(sample.indicesLeaf, &indicesVersion);
        if (keyed && sample.layoutKeyed && sample.layout &&
            sample.layoutOffsetsVersion == offsetsVersion &&
            sample.layoutIndicesVersion == indicesVersion &&
            sample.layoutPointCount == layoutPointCount) {
            continue;
        }
        sample.layoutKeyed = keyed;
        sample.layoutOffsetsVersion = offsetsVersion;
        sample.layoutIndicesVersion = indicesVersion;
        sample.layoutPointCount = layoutPointCount;
        ++sample.layoutBuilds;
        const auto &raw = *view.values;
        const VtVec3fArray offsets = sample.offsetsLeaf >= 0 && size_t(sample.offsetsLeaf) < raw.size() &&
            raw[size_t(sample.offsetsLeaf)].IsHolding<VtVec3fArray>() ?
            raw[size_t(sample.offsetsLeaf)].UncheckedGet<VtVec3fArray>() : VtVec3fArray();
        const VtIntArray indices = sample.indicesLeaf >= 0 && size_t(sample.indicesLeaf) < raw.size() &&
            raw[size_t(sample.indicesLeaf)].IsHolding<VtIntArray>() ?
            raw[size_t(sample.indicesLeaf)].UncheckedGet<VtIntArray>() : VtIntArray();
        auto layout = std::make_shared<RigExecBlendSampleLayout>();
        layout->pointCount = layoutPointCount;
        if (sample.shapeValid)
            RigExecBuildGeometryBlendLayout(offsets,indices,layoutPointCount,layout.get());
        if (!sample.layout || !RigExecSameBlendLayout(*sample.layout,*layout))
            sample.layout = std::move(layout);
    }
    GatherBlendChannels(B, revision, _LeafBlendReads{*view.values},
                        /*record=*/true, &values);
    return RigExecAssembleGeometry(revision->kernelRecord.op, revision->kernelRecord.binding, view,
                                     values);
}

namespace {

// The packet of \p revision this run: from its path leaves wherever they
// assemble it, off the stage for a plugin revision bound to region values.
RigExecMoverParameters
AssembleRevision(RigExecBakedProgramImpl &B,
                 RigExecBakedProgramImpl::GeomRevision *revision,
                 const GfVec3f *basePoints, size_t basePointCount,
                 UsdTimeCode time, RigExecBakedStep *step, size_t layoutPointCount)
{
    if (revision->leaves.decl.assembles) {
        return RigExecBakedAssembleFromLeaves(&B, revision, basePoints,
                                              basePointCount,
                                              &step->diagnostics,nullptr,layoutPointCount);
    }
    RigExecMoverParameters invalid;
    invalid.valid = false;
    step->diagnostics.push_back(revision->moverPathText +
        ": mover has no declared stage-free API4 assembly");
    return invalid;
}

/// The packet's envelope resolved at the full \p count, as ResolveAll into
/// `envelope` would leave it (unchanged when it fails), with
/// `envelopeVersion` moved exactly when the bytes did.
void
ResolveEnvelope(RigExecBakedProgramImpl::GeomRevision *revision, size_t count)
{
    revision->envelopeOk = revision->parameters.weights.ResolveAll(
        count, &revision->resolveScratch);
    if (revision->envelopeOk) {
        RigExecBakedNoteFloats(&revision->envelope, &revision->resolveScratch,
                               &revision->envelopeVersion);
    }
}

/// The skin revision over its whole array, out of the fuse.
///
/// The path where the partition no longer describes the layout the packet
/// carries -- a weight-paint edit the epoch let through, an interactive
/// override on the indices, a cache that refused the mover after all. The
/// keys cannot be trusted, so the chunks stand down and the one step that
/// holds both the folded table and the preceding points runs the revision
/// whole. Serial, cold, and exactly the arithmetic an unchunked revision
/// would have performed.
///
/// Into \p fused, never staging or a group buffer: they hold what the
/// chunks' published RevisionOut keys describe, and a retained or cloned key
/// must keep matching it. \p points are the \p count points entering the
/// revision.
bool
FuseWholeRevision(RigExecBakedProgramImpl::GeomRevision *revision,
                  const GfVec3f *points, size_t count, bool useSimd,
                  std::vector<GfVec3f> *fused)
{
    if (!revision->layoutUsable || !revision->envelopeOk ||
        count != revision->precedingCount) {
        return false;
    }
    // Against the forms the FOLD wrote -- this step reads RevisionTransforms
    // and writes none of it, so the table it skins against is the one that
    // slot already holds.
    fused->resize(count);
    return SkinRange(revision, points, WholeTransformsView(revision), 0, count,
                     /*whole=*/true, useSimd, fused);
}

/// The fuse's one diagnostic, about an enabled revision whose packet is
/// invalid: the out-of-range scalar weight, or the invalid common envelope
/// of a bound weight object. A whole revision's fuse and a range-pipelined
/// revision's join emit it in the same place.
void
EmitInvalidPacketLine(const RigExecBakedProgramImpl &B,
                      const RigExecBakedProgramImpl::GeomRevision &revision,
                      bool packetValid, std::vector<std::string> *diagnostics)
{
    if (revision.parameters.enabled && !packetValid &&
        revision.weightObject < 0) {
        // Read by RevisionStatic, which always runs: a step body that
        // went to the stage for a value would be a step outside its own
        // declarations, and the cone could not tell when it moved.
        const float scalar = revision.defaultWeight;
        if (!std::isfinite(scalar) || scalar < 0.0f || scalar > 1.0f) {
            diagnostics->push_back(
                "MoverFailed " + revision.moverPathText +
                ": inputs:defaultWeight must be finite and in "
                "[0, 1]; revision passed through");
        }
    } else if (revision.parameters.enabled && !packetValid &&
               revision.weightObject >= 0 &&
               !(revision.weightCurrentPhase
                     ? revision.currentPhasePacket
                     : B.weightPackets[size_t(revision.weightObject)])
                    .valid) {
        // The other half of the same sentence, and the reason the arms
        // are exclusive: with a weight object bound, the assembler never
        // reads inputs:defaultWeight, so a rig whose scalar is out of
        // range and whose object is fine must stay silent.
        diagnostics->push_back(
            "MoverFailed " + revision.moverPathText +
            ": rigExec:weightObject produced an invalid common "
            "envelope; revision passed through");
    }
}

/// The version a fuse a cycle set aside publishes, the base passed
/// through (`currentSource` -1, which the set-aside reset writes before the
/// region): it follows \p base against the copy of it last carried.
void
PassBaseThrough(const VtVec3fArray &base,
                RigExecBakedProgramImpl::GeomRevision *revision)
{
    if (!RigExecBakedSamePoints(base.cdata(), base.size(),
                                revision->passedPoints.data(),
                                revision->passedPoints.size())) {
        ++revision->doneVersion;
        revision->passedPoints.assign(base.cbegin(), base.cend());
    }
}

/// A gated packet that failed RigExecRevisionGateHolds while applying, or a
/// Range group step that met a stale partition or a count other than its
/// group's: a pin hole, counted for the owner to report after the region.
void
NoteGateViolation(RigExecBakedProgramImpl &B)
{
    B.gateViolations.fetch_add(1, std::memory_order_relaxed);
}

/// The "apply once" envelope RevisionStatic resolved at the full count, for
/// a group kernel that blends separately below full strength; else null.
const float *
GroupEnvelope(const RigExecBakedProgramImpl::GeomRevision &revision)
{
    return !revision.fullStrength && revision.envelopeOk
               ? revision.envelope.data()
               : nullptr;
}

/// Group \p g of the chain base as a content id (pointBlocks.h).
RigExecGroupSource
BaseGroupSource(const RigExecBakedProgramImpl::GeomChain &chain, size_t g)
{
    RigExecGroupSource source;
    source.slot = -1 - int64_t(g);
    source.version = g < chain.baseGroups.size() ? chain.baseGroups[g].version
                                                  : 0;
    return source;
}

/// Makes \p revision's `groupIds[g]` \p source and returns whether it moved.
bool
NoteGroupId(RigExecBakedProgramImpl::GeomRevision *revision, size_t g,
            const RigExecGroupSource &source)
{
    if (revision->groupIds[g] == source) {
        return false;
    }
    revision->groupIds[g] = source;
    return true;
}

/// Sizes \p revision's `groupIds` to its groups (Build sizes them; a size
/// change is a publication that moved).
bool
SizeGroupIds(RigExecBakedProgramImpl::GeomRevision *revision)
{
    if (revision->groupIds.size() == revision->groups.size()) {
        return false;
    }
    revision->groupIds.assign(revision->groups.size(), RigExecGroupSource());
    return true;
}

/// The version a join or Whole fuse of a revision a cycle set aside
/// publishes: the base groups passed through, by their content ids;
/// `doneVersion` moves only when those ids do.
void
PassBaseGroups(const RigExecBakedProgramImpl::GeomChain &chain,
               RigExecBakedProgramImpl::GeomRevision *revision)
{
    revision->ran = false;
    revision->executed = false;
    revision->resultStatus = _tokens->operationCycle;
    bool moved = SizeGroupIds(revision);
    for (size_t g = 0; g < revision->groupIds.size(); ++g) {
        moved = NoteGroupId(revision, g, BaseGroupSource(chain, g)) || moved;
    }
    if (moved) {
        ++revision->doneVersion;
    }
}

/// Group step \p g of the Range \p revision (chain index \p revisionIndex of
/// chain \p chainIndex): publishes group g of version
/// revisionIndex + 1 -- the group kernel's points where the revision applies,
/// the entering group shared by reference where it does not or where the
/// kernel leaves the group untouched -- with a content version that moves
/// exactly when the published bytes do (pointBlocks.h). Reads only group g of
/// the entering version (and, for a skin, the matrix slots of its key);
/// writes only `groups[g]` and its own chunk table, so the groups of one
/// revision run concurrently. The decision is RevisionStatic's and the
/// fold's (RigExecBakedRevisionApplies); a kernel refusal after an Applies
/// acceptance passes the group through and the join counts it.
void
RunGroupStep(RigExecBakedProgramImpl &B,
             const RigExecBakedProgramImpl::GeomChain &chain, int chainIndex,
             RigExecBakedProgramImpl::GeomRevision *revision,
             size_t revisionIndex, size_t g)
{
    if (g >= revision->groups.size() || g + 1 >= chain.groupBounds.size() ||
        g >= chain.baseGroups.size()) {
        return;
    }
    RigExecGroupState<GfVec3f> &state = revision->groups[g];
    if (revision->rangeSetAside) {
        // A cycle set the revision aside: it passes the base through, as an
        // excluded fuse does, and computes nothing.
        state.ok = false;
        RigExecPublishPassedGroup(&state, chain.baseGroups[g].published,
                                  BaseGroupSource(chain, g));
        return;
    }
    const RigExecPointsRef<GfVec3f> &entering =
        RigExecBakedGroupAt(chain, revisionIndex, g);
    const RigExecGroupSource from =
        RigExecBakedGroupSourceAt(B, chainIndex, revisionIndex, g);
    const size_t begin = size_t(chain.groupBounds[g]);
    const size_t end = size_t(chain.groupBounds[g + 1]);
    const size_t count = revision->precedingCount;
    // The count is a pin (RigExecBakedRolesStand), so the group always has
    // its Build size; a run that met another refuses rather than reading
    // past a group.
    if (count != chain.groupPointCount || end < begin ||
        entering.count != end - begin) {
        state.ok = false;
        RigExecPublishPassedGroup(&state, entering, from);
        NoteGateViolation(B);
        return;
    }
    if (!RigExecBakedRevisionApplies(*revision)) {
        state.ok = true;
        RigExecPublishPassedGroup(&state, entering, from);
        return;
    }
    const bool skin = revision->op == RigExecRevisionOp::Skin;
    RigExecSkinTransformsView view;
    if (skin) {
        // Range skins rest on an epoch-fixed layout (a NoLayoutOverride pin),
        // so the keys describe the vertices; a stale partition is a pin hole
        // and refuses rather than skinning against identities.
        if (revision->partitionStale || g >= revision->chunks.size()) {
            state.ok = false;
            RigExecPublishPassedGroup(&state, entering, from);
            NoteGateViolation(B);
            return;
        }
        RigExecBakedProgramImpl::GeomChunk &chunk = revision->chunks[g];
        chunk.keyChanged = false;
        GatherChunkTransforms(B, *revision, &chunk);
        view = ChunkTransformsView(chunk, B.useSimd);
    }
    const int k = RigExecGroupScratch(&state, end - begin);
    bool untouched = false;
    const bool ok = RigExecRunRevisionGroup(
        revision->op, revision->parameters, revision->rangeInputs,
        skin ? &view : nullptr, entering.data, state.own[size_t(k)]->data(),
        count, begin, end, GroupEnvelope(*revision), B.useSimd, &untouched);
    state.ok = ok;
    if (!ok || untouched) {
        RigExecPublishPassedGroup(&state, entering, from);
        return;
    }
    RigExecPublishOwnGroup(&state, k);
}

/// Speculative chunk \p g of the Whole keyed skin \p revision: group g of
/// the entering version skinned against the chunk's own key-filled table
/// into a buffer of the group's state, kept as its computed result
/// (RigExecNoteComputedGroup) for the fuse to publish or discard. Like a
/// Legacy keyed chunk it does not wait for the fold, and its `ok` is sticky
/// across runs it sits out.
void
RunWholeSkinChunk(const RigExecBakedProgramImpl &B,
                  const RigExecBakedProgramImpl::GeomChain &chain,
                  RigExecBakedProgramImpl::GeomRevision *revision,
                  size_t revisionIndex, size_t g)
{
    if (g >= revision->chunks.size() || g >= revision->groups.size() ||
        g + 1 >= chain.groupBounds.size()) {
        return;
    }
    RigExecBakedProgramImpl::GeomChunk &chunk = revision->chunks[g];
    RigExecGroupState<GfVec3f> &state = revision->groups[g];
    chunk.keyChanged = false;
    const RigExecPointsRef<GfVec3f> &entering =
        RigExecBakedGroupAt(chain, revisionIndex, g);
    const size_t begin = size_t(chain.groupBounds[g]);
    const size_t end = size_t(chain.groupBounds[g + 1]);
    if (!revision->status.AllowsApply() || !SkinPacketIsUsable(*revision) ||
        revision->partitionStale ||
        revision->precedingCount != chain.groupPointCount || end < begin ||
        entering.count != end - begin) {
        state.ok = false;
        chunk.ok = false;
        return;
    }
    GatherChunkTransforms(B, *revision, &chunk);
    const RigExecSkinTransformsView view = ChunkTransformsView(chunk, B.useSimd);
    const int k = RigExecGroupScratch(&state, end - begin);
    const bool ok = RigExecRunRevisionGroup(
        revision->op, revision->parameters, revision->rangeInputs, &view,
        entering.data, state.own[size_t(k)]->data(), revision->precedingCount,
        begin, end, GroupEnvelope(*revision), B.useSimd);
    if (ok) {
        RigExecNoteComputedGroup(&state, k);
    }
    state.ok = ok;
    chunk.ok = ok;
}

/// Whether chunk \p k of a Whole or Legacy revision holds an answer: a keyed
/// Whole skin's in its group state (with a computed result to publish), any
/// other chunk's in the chunk.
bool
ChunkHoldsAnswer(const RigExecBakedProgramImpl::GeomRevision &revision,
                 size_t k)
{
    if (revision.role == RigExecBakedRevisionRole::Whole && revision.chunked) {
        return k < revision.groups.size() && revision.groups[k].ok &&
               revision.groups[k].ownComputed >= 0;
    }
    return revision.chunks[k].ok;
}

/// The fuse of the Whole \p revision (revision id \p id, chain index
/// \p revisionIndex of chain \p chainIndex): the decision a Legacy fuse makes
/// -- packet and fold, acceptance or the AND of the speculative chunks, the
/// whole-array skin where the partition went stale -- and then every group
/// of version revisionIndex + 1 published with its own exact version: the
/// chunk's computed result (a keyed skin), the group's slice of the result
/// copied into a buffer of the group (one whole chunk, or the whole-array
/// skin), or the entering group shared where the revision did not apply.
/// `groupIds` and `doneVersion` follow (2.7): a downstream group reruns only
/// if its group moved.
void
RunWholeFuse(RigExecBakedProgramImpl &B,
             const RigExecBakedProgramImpl::GeomChain &chain, int chainIndex,
             RigExecBakedProgramImpl::GeomRevision *revision, int id,
             size_t revisionIndex, RigExecBakedStep *step)
{
    if (revision->rangeSetAside) {
        PassBaseGroups(chain, revision);
        return;
    }
    const bool skin = revision->op == RigExecRevisionOp::Skin;
    const bool packetValid =
        revision->parameters.valid && (!skin || revision->influencesValid);
    EmitInvalidPacketLine(B, *revision, packetValid, &step->diagnostics);
    revision->executed = true;
    step->counters.revisionsExecuted = 1;
    revision->resultStatus = revision->status.state;
    const size_t groupCount = revision->groups.size();
    const bool keyed = revision->chunked;
    const size_t count = revision->precedingCount;
    // The count is a pin; a run at another one refuses.
    const bool sized =
        count == chain.groupPointCount &&
        chain.groupBounds.size() == groupCount + 1 &&
        (keyed || revision->stagingOutput.size() == count);
    bool applied = packetValid && revision->status.AllowsApply() && sized;
    const bool whole = applied && skin && revision->partitionStale;
    std::vector<GfVec3f> fused;
    if (whole) {
        // The whole-array skin; a stale partition is rare.
        const GfVec3f *points = nullptr;
        size_t enteringCount = 0;
        RigExecBakedVersionPoints(chain, revisionIndex,
                                  &revision->wholeEntering, &points,
                                  &enteringCount);
        applied = FuseWholeRevision(revision, points, enteringCount,
                                    B.useSimd, &fused);
    } else if (applied && revision->acceptance !=
                              RigExecRevisionAcceptance::Deferred) {
        applied = revision->acceptance == RigExecRevisionAcceptance::Applies;
    } else if (applied) {
        for (size_t k = 0; k < revision->chunks.size(); ++k) {
            if (!ChunkHoldsAnswer(*revision, k)) {
                applied = false;
                break;
            }
        }
    }
    // A keyed skin publishes its chunks' results: each must hold one.
    if (applied && keyed && !whole) {
        for (size_t g = 0; g < groupCount; ++g) {
            if (!ChunkHoldsAnswer(*revision, g)) {
                applied = false;
                break;
            }
        }
    }
    if (!applied && revision->status.AllowsApply()) {
        revision->resultStatus = _tokens->moverFailed;
    }
    const std::vector<GfVec3f> *slices =
        whole ? &fused : (keyed ? nullptr : &revision->stagingOutput);
    bool moved = SizeGroupIds(revision);
    for (size_t g = 0; g < groupCount; ++g) {
        RigExecGroupState<GfVec3f> &state = revision->groups[g];
        if (applied && !slices) {
            RigExecPublishOwnGroup(&state, state.ownComputed);
        } else if (applied) {
            const size_t begin = size_t(chain.groupBounds[g]);
            const size_t end = size_t(chain.groupBounds[g + 1]);
            const int k = RigExecGroupScratch(&state, end - begin);
            std::copy(slices->begin() + long(begin),
                      slices->begin() + long(end),
                      state.own[size_t(k)]->begin());
            RigExecPublishOwnGroup(&state, k);
        } else {
            RigExecPublishPassedGroup(
                &state, RigExecBakedGroupAt(chain, revisionIndex, g),
                RigExecBakedGroupSourceAt(B, chainIndex, revisionIndex, g));
        }
        RigExecGroupSource source;
        source.slot = int64_t(RigExecBakedGroupSlot(B, id, g));
        source.version = state.version;
        moved = NoteGroupId(revision, g, source) || moved;
    }
    if (moved) {
        ++revision->doneVersion;
    }
    revision->lastStatus = revision->status;
    revision->ran = true;
}

/// The join of the Range \p revision (revision id \p id, chain index
/// \p revisionIndex of chain \p chainIndex): the fuse's line and status from
/// the decision its group steps shared, the refusal count over its written
/// groups, and the content ids of version revisionIndex + 1 -- its own for
/// the groups it writes, the predecessor's (the base's for the first
/// revision) for the groups it aliases. Reads no points. `doneVersion` moves
/// exactly when those ids do, so a version whose groups all kept their bytes
/// keeps its version. A cycle that set the revision aside makes it publish
/// the base passed through.
void
RunJoin(const RigExecBakedProgramImpl &B,
        const RigExecBakedProgramImpl::GeomChain &chain,
        RigExecBakedProgramImpl::GeomRevision *revision, int id,
        size_t revisionIndex, RigExecBakedStep *step)
{
    if (revision->rangeSetAside) {
        PassBaseGroups(chain, revision);
        return;
    }
    // The fuse's own predicate, which for a skin ANDs in the fold.
    const bool packetValid =
        revision->parameters.valid &&
        (revision->op != RigExecRevisionOp::Skin || revision->influencesValid);
    EmitInvalidPacketLine(B, *revision, packetValid, &step->diagnostics);
    revision->executed = true;
    step->counters.revisionsExecuted = 1;
    const bool applied = RigExecBakedRevisionApplies(*revision);
    revision->resultStatus = revision->status.state;
    if (!applied && revision->status.AllowsApply()) {
        revision->resultStatus = _tokens->moverFailed;
    }
    const RigExecBakedProgramImpl::GeomRevision *predecessor =
        revisionIndex > 0 ? &chain.revisions[revisionIndex - 1] : nullptr;
    bool moved = SizeGroupIds(revision);
    uint32_t refusals = 0;
    for (size_t g = 0; g < revision->groupIds.size(); ++g) {
        RigExecGroupSource source;
        if (g < revision->groupWritten.size() && revision->groupWritten[g] &&
            g < revision->groups.size()) {
            if (applied && !revision->groups[g].ok) {
                ++refusals;
            }
            source.slot = int64_t(RigExecBakedGroupSlot(B, id, g));
            source.version = revision->groups[g].version;
        } else if (predecessor && g < predecessor->groupIds.size()) {
            source = predecessor->groupIds[g];
        } else {
            source = BaseGroupSource(chain, g);
        }
        moved = NoteGroupId(revision, g, source) || moved;
    }
    revision->rangeRefusals = refusals;
    if (moved) {
        ++revision->doneVersion;
    }
    revision->lastStatus = revision->status;
    revision->ran = true;
}

}  // namespace

namespace {
void ResetGeometryRevision(RigExecBakedProgramImpl::GeomRevision *revision)
{
    revision->created = true; revision->ran = false; revision->output.clear();
    revision->currentSource = -1; revision->lastParameters = RigExecMoverParameters();
    revision->lastAuxPoints = VtVec3fArray(); revision->lastStatus = RigExecMoverStatus();
    // No baseline survives: the next publication bumps the version anyway.
    revision->stagingFresh = false; revision->passedPoints.clear();
    // Nor a published group: each group and the join or fuse publish a new
    // version, above the one they last published (versions never go down).
    for (auto &group : revision->groups) {
        RigExecResetGroup(&group);
        group.ok = false;
    }
    std::fill(revision->groupIds.begin(), revision->groupIds.end(),
              RigExecGroupSource());
}
}
void RigExecBakedAdoptRevisionLayout(RigExecBakedProgramImpl::GeomRevision *revision)
{
    if (revision->op != RigExecRevisionOp::Skin) return;
    revision->topology = revision->layoutHandle;
    revision->topologyResolved = true;
    RigExecBakedAdoptPartition(revision);
}
namespace {
/// ChainInputs, a chain cut into groups: `lastBase` as groups. One immutable
/// owner shares the array's buffer and each base group is a ref into it, so
/// publishing the base copies no point. Each group's version moves exactly
/// when its bytes differ from the ones it last published, or on its first
/// publication since a reset. While the bytes stand (\p moved false) the
/// held refs already hold them and nothing is rebuilt.
void
PublishBaseGroups(RigExecBakedProgramImpl::GeomChain *chain, bool moved)
{
    if (chain->groupBounds.size() < 2) {
        return;
    }
    const size_t groups = chain->groupBounds.size() - 1;
    if (!moved && chain->baseOwner && chain->baseGroups.size() == groups) {
        return;
    }
    chain->baseGroups.resize(groups);
    auto owner = std::make_shared<const VtVec3fArray>(chain->lastBase);
    const size_t count = owner->size();
    for (size_t g = 0; g < groups; ++g) {
        size_t begin = 0, end = 0;
        RigExecPointRangeAt(chain->groupBounds[g], chain->groupBounds[g + 1],
                            g + 1 == groups, count, &begin, &end);
        RigExecPointsRef<GfVec3f> ref;
        ref.owner = owner;
        ref.data = owner->cdata() + begin;
        ref.count = end - begin;
        RigExecGroupState<GfVec3f> &state = chain->baseGroups[g];
        if (!state.ran ||
            !RigExecPointsBitsEqual(ref.data, ref.count, state.published.data,
                                    state.published.count)) {
            ++state.version;
        }
        state.published = std::move(ref);
        state.ran = true;
        state.ok = true;
    }
    chain->baseOwner = std::move(owner);
}

/// Group \p g of the base as a set-aside writer passes it through: \p state
/// shares the base group's ref, its id the base group's.
void
PassSetAsideBaseGroup(const RigExecBakedProgramImpl::GeomChain &chain,
                      size_t g, RigExecGroupState<GfVec3f> *state)
{
    if (g >= chain.baseGroups.size()) {
        return;
    }
    const RigExecGroupState<GfVec3f> &base = chain.baseGroups[g];
    RigExecGroupSource from;
    from.slot = -1 - int64_t(g);
    from.version = base.version;
    RigExecPublishPassedGroup(state, base.published, from);
}

/// What a set-aside join or Whole fuse publishes without running (owner
/// thread, before the region): every group it writes passes the base group
/// through, and its ids are the base groups'. RevisionDone's version moves
/// only when those ids differ from the ones it last published.
void
PassSetAsideBaseGroups(const RigExecBakedProgramImpl::GeomChain &chain,
                       RigExecBakedProgramImpl::GeomRevision *revision)
{
    const size_t groups = chain.groupBounds.size() - 1;
    bool moved = revision->groupIds.size() != groups;
    revision->groupIds.resize(groups);
    for (size_t g = 0; g < groups; ++g) {
        if (g < revision->groupWritten.size() && revision->groupWritten[g] &&
            g < revision->groups.size()) {
            PassSetAsideBaseGroup(chain, g, &revision->groups[g]);
        }
        RigExecGroupSource id;
        id.slot = -1 - int64_t(g);
        id.version = g < chain.baseGroups.size() ? chain.baseGroups[g].version
                                                 : 0;
        if (revision->groupIds[g] != id) {
            revision->groupIds[g] = id;
            moved = true;
        }
    }
    if (moved) {
        ++revision->doneVersion;
    }
}
}  // namespace
void RigExecBakedRunChainInputs(RigExecBakedProgramImpl *program,RigExecBakedStep *step)
{
    auto &chain = program->chains[size_t(step->object)];
    chain.haveBase = chain.sampledHaveBase;
    if (!chain.haveBase) { chain.baseDirty = false; return; }
    if (chain.haveResult && chain.sampledBase.size() != chain.lastBase.size()) {
        // ChainStatus republishes in this run at the new count, so bumping
        // here and there cannot return to the bytes last published.
        if (!chain.result.empty()) ++chain.resultVersion;
        chain.haveResult = false; chain.result = VtVec3fArray(); chain.scheduleDirty = true;
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            auto &revision = chain.revisions[r];
            ResetGeometryRevision(&revision);
            // A range-pipelined revision's own buffer holds its version,
            // unless a cycle set it aside and it passes the base through.
            if (revision.rangeRole && !revision.rangeSetAside)
                revision.currentSource = int(r);
        }
        // The base groups too: each republishes with a bumped version, and
        // ChainStatus gathers whatever ids it then reads.
        for (auto &group : chain.baseGroups) RigExecResetGroup(&group);
        chain.baseOwner.reset();
        chain.resultIds.clear(); chain.spareIds.clear();
    }
    for (auto &revision : chain.revisions) if (revision.created) {
        ++step->counters.revisionsCreated; revision.created = false;
    }
    if (chain.scheduleDirty && !chain.revisions.empty()) ++step->counters.schedulesBuilt;
    chain.scheduleDirty = false;
    chain.baseDirty = !chain.haveResult || !RigExecBakedHeadValueSame(
        VtValue(chain.sampledBase),VtValue(chain.lastBase));
    const bool moved = !SamePoints(chain.sampledBase, chain.lastBase);
    if (moved) ++chain.baseVersion;
    chain.lastBase = chain.sampledBase;
    PublishBaseGroups(&chain, moved);
}
void RigExecBakedPrepareDerivedBase(RigExecBakedProgramImpl::GeomChain::Derived *derived,
                                    RigExecBakedStep *step)
{
    if (derived->matrixTarget) {
        derived->haveBase = true; derived->baseDirty = true;
        derived->revision.created = false; return;
    }
    derived->haveBase = derived->sampledHaveBase;
    if (!derived->haveBase) { derived->baseDirty = false; return; }
    if (derived->haveResult && derived->sampledBase.size() != derived->lastBase.size()) {
        derived->haveResult = false; derived->result = VtVec3fArray();
        ResetGeometryRevision(&derived->revision);
    }
    if (derived->revision.created) {
        ++step->counters.revisionsCreated; ++step->counters.schedulesBuilt;
        derived->revision.created = false;
    }
    derived->baseDirty = !derived->haveResult || !RigExecBakedHeadValueSame(
        VtValue(derived->sampledBase),VtValue(derived->lastBase));
    derived->lastBase = derived->sampledBase;
}
void RigExecBakedShareLatticeBinds(RigExecBakedProgramImpl *program)
{
    RigExecLatticeBindSharing<GfVec3f> binds;
    for (auto &chain : program->chains) {
        for (auto &revision : chain.revisions) {
            binds.Offer(&revision.surfaceCache);
            // A range-pipelined lattice's range steps read the basis its
            // RevisionStatic retained, through `rangeInputs`, which holds
            // that bind alive for a step that follows a skipped
            // RevisionStatic. When the cache now holds an equal shared bind,
            // name that one, so the one it replaced may be released.
            if (revision.rangeInputs.latticeBind) {
                const auto &bind = revision.surfaceCache.RetainedLatticeBind();
                if (bind && bind != revision.rangeInputs.latticeBind) {
                    revision.rangeInputs.latticeBind = bind;
                    revision.rangeInputs.latticeBasis = &bind->value;
                }
            }
        }
        for (auto &derived : chain.derived) {
            binds.Offer(&derived.revision.surfaceCache);
        }
    }
}
void RigExecBakedRunGeometryPrologue(RigExecBakedProgramImpl *program,
                                    UsdTimeCode time,RigExecRigPose *,bool all)
{
    auto &B = *program;
    // Source capture only. No head output, topology adoption, node reset or
    // generation accounting is consumed before the declared graph operations.
    for (auto &object : B.weightObjects) {
        RigExecBakedSamplePathLeaves(&B,&object.pointLeaves,time,all);
        RigExecBakedSamplePathLeaves(&B,&object.oracleLeaves,time,all);
    }
    for (auto &chain : B.chains) {
        chain.sampledBase = VtVec3fArray();
        const auto upstream = B.upstream.find(chain.target);
        if (upstream != B.upstream.end() && upstream->second.IsHolding<VtVec3fArray>()) {
            chain.sampledBase = upstream->second.UncheckedGet<VtVec3fArray>();
            chain.sampledHaveBase = true;
        } else {
            chain.sampledHaveBase = chain.baseQuery.IsValid() && chain.baseQuery.Get(&chain.sampledBase,time);
        }
        for (auto &revision : chain.revisions) {
            RigExecBakedSamplePathLeaves(&B,&revision.leaves,time,all);
        }
        for (auto &derived : chain.derived) {
            derived.sampledBase = VtVec3fArray();
            derived.sampledHaveBase = derived.matrixTarget ||
                (derived.baseQuery.IsValid() && derived.baseQuery.Get(&derived.sampledBase,time));
            RigExecBakedSamplePathLeaves(&B,&derived.revision.leaves,time,all);
        }
    }
}
void
RigExecBakedSkipGeometryStep(RigExecBakedProgramImpl *program,
                             RigExecBakedStep *step)
{
    RigExecBakedProgramImpl &B = *program;
    if (step->kind == RigExecBakedStepKind::WeightField) {
        // Field IDs are not revision IDs; a clean skip preserves the field.
        return;
    }
    if (step->kind == RigExecBakedStepKind::ChainInputs) {
        B.chains[size_t(step->object)].baseDirty = false; return;
    }
    if (step->kind == RigExecBakedStepKind::Derived) {
        RigExecBakedProgramImpl::GeomRevision &revision =
            B.chains[size_t(B.derivedIndex[size_t(step->object)].first)]
                .derived[size_t(B.derivedIndex[size_t(step->object)].second)]
                .revision;
        revision.influencesChanged = false;
        return;
    }
    if (step->kind == RigExecBakedStepKind::ChainStatus) {
        return;  // it writes no delta: the sweep is over persisted status
    }
    const auto &[chainIndex, revisionIndex] =
        B.revisionIndex[size_t(step->object)];
    RigExecBakedProgramImpl::GeomRevision &revision =
        B.chains[size_t(chainIndex)].revisions[size_t(revisionIndex)];
    switch (step->kind) {
    case RigExecBakedStepKind::InfluenceFold:
        // The table is where the fold left it, so nothing moved into it.
        revision.influencesChanged = false;
        return;
    case RigExecBakedStepKind::RevisionStatic:
        revision.staticDirty = false;
        return;
    case RigExecBakedStepKind::RevisionChunk:
        // A chunk step's part is a chunk (a Range revision's group g is its
        // chunk g); its group state keeps the published or computed result.
        if (step->part >= 0 && size_t(step->part) < revision.chunks.size()) {
            revision.chunks[size_t(step->part)].keyChanged = false;
        }
        // A skipped body retains its published output validity and bytes.
        // Current-generation execution is reported by portable ran metadata.
        return;
    case RigExecBakedStepKind::RevisionFuse:
        // The chain's sticky dirty bit as the NEXT revision reads it: this
        // revision did not execute this run, whatever it did during the last
        // one. `currentSource`, `resultStatus` and the points stay where the
        // fuse left them -- those are the values, not the comparison.
        revision.executed = false;
        return;
    default:
        return;
    }
}

void
RigExecBakedRunGeometryStep(RigExecBakedProgramImpl *program,
                            RigExecBakedStep *step, UsdTimeCode time)
{
    RigExecBakedProgramImpl &B = *program;
    if (step->kind == RigExecBakedStepKind::ChainInputs) {
        RigExecBakedRunChainInputs(&B,step); return;
    }
    const RigExecResolvedInputs &R = *B.resolvedInputs;
    if (step->kind == RigExecBakedStepKind::Derived) {
        const auto &[chainIndex, derivedIndex] =
            B.derivedIndex[size_t(step->object)];
        RigExecBakedProgramImpl::GeomChain &chain =
            B.chains[size_t(chainIndex)];
        RigExecBakedProgramImpl::GeomChain::Derived &derived =
            chain.derived[size_t(derivedIndex)];
        // DerivedOut's content version moves exactly when this step leaves
        // `result` holding other bytes than it published last, a size reset
        // included. Held by handle, so the compare costs no copy.
        const VtVec3fArray published = derived.result;
        const auto noteResult = [&] {
            if (!SamePoints(published, derived.result)) ++derived.resultVersion;
        };
        RigExecBakedPrepareDerivedBase(&derived,step);
        if (!chain.haveBase || !derived.haveBase) {
            noteResult();
            return;
        }
        RigExecBakedProgramImpl::GeomRevision &revision = derived.revision;
        RigExecBakedAdoptRevisionLayout(&revision);
        if (derived.matrixTarget) {
            step->counters.revisionsBuilt = 1;
            derived.haveMatrix = RigExecBakedRunProjectorTarget(
                B, chain, revision, &derived.matrix, &step->diagnostics);
            derived.haveResult = true;
            step->counters.chainsBuilt = 1;
            return;
        }
        FoldInfluences(B, &revision);
        RigExecMoverParameters parameters = AssembleRevision(
            B, &revision, chain.result.cdata(), chain.result.size(), time,
            step,chain.lastBase.size());
        const RigExecMoverStatus status =
            RigExecStatusForParameters(parameters, revision.moverPathText);
        step->counters.revisionsBuilt = 1;
        // A selected derived operation computes its declared typed result.
        {
            step->counters.revisionsExecuted = 1;
            // The derived target's own authored array, which is what the
            // recomputation writes over: two vectors for an extent, the
            // whole mesh for normals. Built once and re-assigned on the
            // failure arm rather than copied into a `preceding` that exists
            // only to be copied again.
            std::vector<GfVec3f> values;
            // The same function the revision node calls, and not a second
            // arrangement of the same rules: the kind check, the empty and
            // size-2 guards, the derived property keeping its authored
            // cardinality (an in-place write cannot resize it) and the
            // envelope folded into the recomputation's own arithmetic all
            // live in RigExecApplyDerivedKernel. The status is the one half
            // the kernel does not own, because the packet is what it is told
            // about and the status is what the graph decided.
            const bool applied =
                status.AllowsApply() &&
                RigExecRunGeometryDerived(revision.op, parameters,
                    derived.lastBase.cdata(),derived.lastBase.size(),&values);
            revision.resultStatus = status.state;
            if (!applied) {
                values.assign(derived.lastBase.begin(),
                              derived.lastBase.end());
                if (status.AllowsApply()) {
                    revision.resultStatus = _tokens->moverFailed;
                }
            }
            revision.output = std::move(values);
            // The packet WITHOUT its points, and the points beside it. Both
            // are the run's, and both are replaced only where the revision
            // executed -- the comparison above is against the last packet
            // that produced an answer, not against last frame's inputs.
            parameters.auxPoints.clear();
            parameters.auxPoints.shrink_to_fit();
            revision.lastParameters = std::move(parameters);
            revision.lastAuxPoints = chain.result;
            revision.lastStatus = status;
            revision.ran = true;
        }
        if (revision.resultStatus == "moverFailed") {
            step->diagnostics.push_back(
                "MoverFailed " + derived.targetText +
                ": derived geometry input/cardinality validation failed");
        }
        derived.spare.resize(revision.output.size());
        std::copy(revision.output.begin(), revision.output.end(),
                  derived.spare.data());
        derived.result.swap(derived.spare);
        noteResult();
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
            // ChainPoints owns current availability. Retain cached numerical
            // storage, but no Final read may consume a previous valid frame.
            chain.haveResult = false;
            return;
        }
        // The per-chain status sweep, over the status each revision LAST
        // published -- which outlives an evaluation it did not take part in,
        // and so does its diagnostic.
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
                 chain.revisions) {
            if (revision.resultStatus == "moverFailed") {
                step->diagnostics.push_back(
                    "MoverFailed " + revision.moverPathText +
                    ": execution rejected its inputs; revision passed "
                    "through");
            }
        }
        step->counters.chainsBuilt = 1;
        const size_t last = chain.revisions.size();
        if (chain.groupBounds.size() >= 2 && last > 0) {
            // A chain cut into groups: the last revision's group ids name
            // version n's content exactly. Ids `result` was gathered from
            // that still stand leave it in place, identity and all.
            const size_t groups = chain.groupBounds.size() - 1;
            const std::vector<RigExecGroupSource> &ids =
                chain.revisions[last - 1].groupIds;
            const bool known = ids.size() == groups;
            if (chain.haveResult && known && chain.resultIds == ids) {
                return;
            }
            size_t total = 0;
            for (size_t g = 0; g < groups; ++g) {
                total += RigExecBakedGroupAt(chain, last, g).count;
            }
            // Gathered into the other half of the double buffer: cleared
            // first, so a buffer a consumer still holds is let go rather
            // than copied, and filled once.
            chain.spare.clear();
            chain.spare.resize(total, [&chain, last, groups](GfVec3f *out,
                                                            GfVec3f *) {
                for (size_t g = 0; g < groups; ++g) {
                    const RigExecPointsRef<GfVec3f> &ref =
                        RigExecBakedGroupAt(chain, last, g);
                    out = std::uninitialized_copy(ref.data, ref.data + ref.count,
                                                  out);
                }
            });
            // ChainPoints' content version moves exactly when the bytes do.
            if (!SamePoints(chain.spare, chain.result)) {
                ++chain.resultVersion;
            }
            chain.result.swap(chain.spare);
            chain.spareIds.swap(chain.resultIds);
            if (known) {
                chain.resultIds = ids;
            } else {
                chain.resultIds.clear();
            }
            chain.haveResult = true;
            return;
        }
        // Version n, the chain's final points; a chain of no revisions
        // publishes its base.
        const GfVec3f *points = nullptr;
        size_t count = 0;
        PointsAt(chain, last, &points, &count);
        // ChainPoints' content version moves exactly when the bytes do.
        if (!RigExecBakedSamePoints(points, count, chain.result.cdata(),
                                    chain.result.size())) {
            ++chain.resultVersion;
        }
        // Double-buffered: publication is a refcount bump for the consumer
        // and the array one may still hold from last frame is never the one
        // being written. Cleared first, so a held buffer is let go, never
        // copied.
        chain.spare.clear();
        chain.spare.resize(count, [points](GfVec3f *out, GfVec3f *end) {
            std::uninitialized_copy(points, points + (end - out), out);
        });
        chain.result.swap(chain.spare);
        chain.haveResult = true;
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
    const bool skin = revision.op == RigExecRevisionOp::Skin;
    switch (step->kind) {
    case RigExecBakedStepKind::InfluenceFold: {
        revision.influencesChanged = FoldInfluences(B, &revision);
        // The finite/affine check the dynamic path's assembler makes over the
        // same matrices. It is the fold's because the packet no longer
        // carries them, and the fuse ANDs it in where the assembler's answer
        // would have landed: a revision with one bad influence passes its
        // preceding value through and reports MoverFailed.
        revision.influencesValid =
            !skin || RigExecSkinTransformsAreUsable(revision.influences.data(),
                                                    revision.influences.size());
        // Both forms of the table, whether or not the revision is chunked:
        // a chunked one's chunks keep their own, but the fuse's whole-array
        // fallback reads these and cannot write them. O(influences) of pure
        // per-matrix arithmetic, which is the size of this step anyway. A
        // Range skin has no whole-array fallback (its RevisionTransforms
        // value is the validity alone), so it folds no forms.
        if (skin && revision.influencesValid &&
            revision.role != RigExecBakedRevisionRole::Range) {
            FoldTransformForms(&revision, B.useSimd);
        }
        return;
    }

    case RigExecBakedStepKind::RevisionStatic: {
        RigExecBakedAdoptRevisionLayout(&revision);
        // The field consumes the declared preceding point version. Preserve
        // the source packet's range policy when adopting its dense values.
        if (revision.weightCurrentPhase && revision.weightObject >= 0) {
            revision.currentPhasePacket =
                B.weightPackets[size_t(revision.weightObject)];
            const auto &weightField = B.weightFields[size_t(revision.weightField)];
            const auto &field = weightField.values;
            const auto &error = weightField.error;
            const bool resolved = weightField.ok;
            if (resolved) {
                revision.currentPhasePacket.representation =
                    _tokens->denseRepresentation;
                revision.currentPhasePacket.values = field;
                revision.currentPhasePacket.indices.clear();
                revision.currentPhasePacket.defaultWeight = 0.0f;
                revision.currentPhasePacket.valid = true;
            } else {
                // An invalid packet is the kernel's atomic MoverFailed
                // pass-through, which is the right answer here: publishing
                // the reference-phase field instead would silently be a
                // different deformation.
                revision.currentPhasePacket = RigExecWeightPacket();
                step->diagnostics.push_back("current-phase weight failed: " +
                                            error);
            }
        }
        const bool rangeChain =
            revision.role != RigExecBakedRevisionRole::Legacy;
        const bool rangeLattice = revision.role == RigExecBakedRevisionRole::Range &&
                                  revision.op == RigExecRevisionOp::Lattice;
        // A Range lattice's packet holds the chain base as its rest points
        // (`restBaseHeld`, a copy taken at base version `restBaseVersion`);
        // the assembly takes that copy back instead of copying the base
        // again while the version stands (RigExecBakedAssembleFromLeaves).
        if (revision.restBaseHeld &&
            (!rangeLattice || revision.restBaseVersion != chain.baseVersion)) {
            revision.restBaseHeld = false;
        }
        RigExecMoverParameters assembled =
            AssembleRevision(B, &revision, chain.lastBase.cdata(),
                             chain.lastBase.size(), time, step,chain.lastBase.size());
        // A range-chain blend keys its deltas by a content version moved
        // exactly when their bytes do, not by the bytes.
        if (rangeChain && revision.op == RigExecRevisionOp::BlendShape &&
            !RigExecBakedSamePoints(assembled.blendDeltas.data(),
                                    assembled.blendDeltas.size(),
                                    revision.parameters.blendDeltas.data(),
                                    revision.parameters.blendDeltas.size())) {
            ++revision.deltasVersion;
        }
        revision.parameters = std::move(assembled);
        if (rangeLattice) {
            // The lattice arm copies the base (or takes the held copy back)
            // into a valid packet's rest points and leaves them empty
            // otherwise, so a non-empty rest of the base's count is the base
            // at this version.
            revision.restBaseHeld =
                !revision.parameters.restPoints.empty() &&
                revision.parameters.restPoints.size() == chain.lastBase.size();
            revision.restBaseVersion = chain.baseVersion;
        }
        // The status of the PACKET. For a skin revision that is half of the
        // answer -- the packet carries identities where the matrices would
        // be -- and the fold's `influencesValid` is the other half; the fuse
        // is where the two meet and where `moverFailed` is published.
        revision.status =
            RigExecStatusForParameters(revision.parameters,
                                       revision.moverPathText);
        step->counters.revisionsBuilt = 1;
        // Whether the revision has to run at all, by VALUE and never by
        // dirtiness: a control dragged back to where it started leaves an
        // identical packet, and the VdfNetwork does not re-execute for one.
        // This is the half of the decision the packet and the status carry;
        // the chain's sticky bit and the influence table's own comparison are
        // ORed in by the fuse, which is the first step that has all three.
        // The scalar the fuse's one diagnostic is about, taken here (its
        // leaf) because this step always runs and the fuse may not. Compared
        // like every other source value: two different out-of-range weights
        // can leave the packet identical, and the line the fuse emits is
        // about the weight rather than about the packet.
        // Read the same declared resolved source the effective memo compares.
        const VtValue diagnosticWeight = RigExecBakedResolvePathLeaf(
            B, revision.leaves, size_t(revision.defaultWeightLeaf));
        revision.defaultWeight = diagnosticWeight.IsHolding<float>()
            ? diagnosticWeight.UncheckedGet<float>() : 1.0f;
        // Selection already compares the exact declared packet inputs.
        // This flag records actual current-generation assembly only; it is
        // not a second cache authority or a deep-copy packet fingerprint.
        revision.staticDirty = true;
        revision.lastDefaultWeight = revision.defaultWeight;
        // The field an authoring tool paints as an influence overlay, taken
        // from the packet the mover is about to consume so that what a
        // rigger sees is what deformed the geometry. Published from HERE
        // because this is where the dynamic path publishes it -- beside the
        // assemble, whether or not the revision then executes -- and the
        // epilogue drains it in chain and revision order, which is what
        // makes two revisions sharing one weight object last-writer-wins the
        // way the dynamic walk's per-chain merge does.
        revision.weightFieldPublished = false;
        if (revision.weightObject >= 0 && B.publishWeightFields) {
            const RigExecWeightPacket &packet =
                revision.weightCurrentPhase
                    ? revision.currentPhasePacket
                    : B.weightPackets[size_t(revision.weightObject)];
            if (packet.valid) {
                const size_t logicalCount =
                    revision.weightOperationDomain ? size_t(1)
                                                   : chain.lastBase.size();
                // The field is a function of the packet and the count alone.
                // The shared packet's op value is a declared read whose
                // producer published before this step was dispatched, so its
                // revision is final here, and an equal revision is an equal
                // exact key: the packet the held field was resolved from.
                const bool shared = !revision.weightCurrentPhase &&
                    revision.weightPacketValue >= 0 &&
                    size_t(revision.weightPacketValue) <
                        B.opAdapter.values.size();
                const uint64_t packetRevision = shared
                    ? B.opAdapter.values[size_t(revision.weightPacketValue)]
                          .revision
                    : 0;
                if (!shared || !revision.weightValuesHeld ||
                    revision.weightValuesPacketRevision != packetRevision ||
                    revision.weightValuesCount != logicalCount) {
                    // One scatter rather than a search per point: Resolve()
                    // binary-searches a sparse packet's indices, which for a
                    // face cluster on a body is tens of thousands of
                    // searches to find a few hundred weights. An
                    // unresolvable packet publishes what the per-point loop
                    // did: zero wherever Resolve() answered out of range.
                    std::vector<float> &resolved = revision.resolveScratch;
                    if (!packet.ResolveAll(logicalCount, &resolved)) {
                        resolved.assign(logicalCount, 0.0f);
                        for (size_t i = 0; i < logicalCount; ++i) {
                            const float w = packet.Resolve(i, logicalCount);
                            resolved[i] = w < 0.0f ? 0.0f : w;
                        }
                    }
                    RigExecBakedNoteFloats(&revision.publishedWeightValues,
                                           &resolved,
                                           &revision.weightValuesVersion);
                    revision.weightValuesHeld = shared;
                    revision.weightValuesPacketRevision = packetRevision;
                    revision.weightValuesCount = logicalCount;
                }
                revision.weightFieldPublished = true;
            }
        }
        // Every revision of a chain is applied to the same number of points:
        // a kernel that resized its output failed the application, so no
        // buffer the chain ever reads holds a different count. Sizing the
        // output here rather than in a chunk is what lets the chunks write
        // disjoint ranges of it without one of them owning its length. A
        // Range revision and a keyed Whole skin write group buffers Build
        // sized instead, and keep no staging.
        const size_t count = chain.lastBase.size();
        const bool staged =
            revision.role == RigExecBakedRevisionRole::Legacy ||
            (revision.role == RigExecBakedRevisionRole::Whole &&
             !revision.chunked);
        if (staged && revision.stagingOutput.size() != count) {
            revision.stagingOutput.resize(count);
            // A resized buffer holds no answers, so every chunk of it has to
            // produce one again.
            revision.staticDirty = true;
        }
        revision.precedingCount = count;
        // The whole-array decisions that are the skin kernel's and are made
        // here because a range may not repeat them: the layout validation
        // against the points that arrived, and the envelope, which resolves
        // atomically at the FULL count or not at all.
        revision.layoutUsable = false;
        revision.envelopeOk = true;
        revision.fullStrength = true;
        revision.partitionStale = false;
        // A gated Range revision writes only the groups its weight names and
        // aliases the rest, which is exact only while its packet keeps every
        // unnamed point at its entering bytes; one that applies without
        // that is a pin hole (counted, reported by the owner).
        const auto noteGate = [&B, &revision]() {
            if (revision.role != RigExecBakedRevisionRole::Range ||
                std::find(revision.groupWritten.begin(),
                          revision.groupWritten.end(),
                          char(0)) == revision.groupWritten.end()) {
                return;
            }
            if (revision.parameters.valid && revision.status.AllowsApply() &&
                revision.acceptance == RigExecRevisionAcceptance::Applies &&
                !RigExecRevisionGateHolds(revision.op, revision.parameters)) {
                NoteGateViolation(B);
            }
        };
        if (revision.role == RigExecBakedRevisionRole::Range && !skin) {
            // The revision's one writer of everything its group steps read
            // besides the packet and the entering groups: the "apply once"
            // envelope at the full count for every op that blends
            // separately, and the immutable kernel inputs (`rangeInputs`,
            // which fills the revision's own caches; a blend's per-point
            // weights). A lattice whose rest is the held base names that
            // base's version, so its bind skips only the rest comparison.
            if (RigExecRevisionTakesSeparateBlend(
                    revision.op, revision.parameters.weights)) {
                revision.fullStrength =
                    RigExecEnvelopeIsFullStrength(revision.parameters.weights);
                if (!revision.fullStrength) {
                    ResolveEnvelope(&revision, count);
                }
            }
            revision.acceptance = RigExecPrepareRevisionRanges(
                revision.op, revision.parameters, count, &revision.wireBasis,
                &revision.surfaceCache, &revision.rangeInputs,
                revision.restBaseHeld ? revision.restBaseVersion : 0);
            noteGate();
            return;
        }
        if (!skin) {
            // A wire's dense walk blends a separate envelope after its
            // kernel; resolved here at the full count, as a skin's is, and
            // its chunk blends with this array.
            if (revision.op == RigExecRevisionOp::Wire &&
                RigExecRevisionTakesSeparateBlend(
                    revision.op, revision.parameters.weights)) {
                revision.fullStrength =
                    RigExecEnvelopeIsFullStrength(revision.parameters.weights);
                if (!revision.fullStrength) {
                    ResolveEnvelope(&revision, count);
                }
            }
            // From the validation the chunk's kernel runs first, over the
            // count it is applied to. A wire's envelope validation is the
            // resolve above: wherever the acceptance asks for it, the wire
            // branch took the same predicates and `envelopeOk` holds its
            // answer.
            revision.acceptance = RigExecRevisionKernelAcceptance(
                revision.op, revision.parameters, count,
                &revision.envelopeOk);
            return;
        }
        // Every skin, Legacy, Whole or Range: a Range skin's group steps
        // share this decision, which never defers for its classicLinear
        // method.
        revision.layoutUsable =
            RigExecSkinLayoutIsUsable(revision.parameters, count);
        revision.fullStrength =
            RigExecEnvelopeIsFullStrength(revision.parameters.weights);
        if (!revision.fullStrength) {
            ResolveEnvelope(&revision, count);
        }
        revision.acceptance = SkinAcceptance(revision);
        if (revision.chunked) {
            // Whether the keys still describe the vertices. The handle is
            // the identity the SkinTopology op preserves for a binding that
            // did not move, and the prologue adopted it only if it has the
            // partition's arrays; anything else here means the packet is
            // reading arrays the keys were not cut from, and the fuse runs
            // the revision whole instead.
            const RigExecSkinTopology *const topology =
                revision.parameters.skinTopology.get();
            const size_t indexCount =
                topology ? topology->indices.size()
                         : revision.parameters.skinIndices.size();
            const int elementSize = topology
                                        ? topology->elementSize
                                        : revision.parameters.skinElementSize;
            revision.partitionStale =
                // No resolved layout at all is a layout the keys cannot be
                // checked against: the packet is reading the mover's arrays
                // per frame, and nothing here can say they are the arrays
                // the cut was made from.
                !revision.parameters.skinTopology ||
                revision.parameters.skinTopology !=
                    revision.partitionTopology ||
                indexCount != revision.partitionIndexCount ||
                elementSize != revision.partitionElementSize ||
                count != revision.partitionPointCount;
        }
        noteGate();
        return;
    }

    case RigExecBakedStepKind::RevisionChunk: {
        if (revision.role == RigExecBakedRevisionRole::Range) {
            RunGroupStep(B, chain, chainIndex, &revision,
                         size_t(revisionIndex), size_t(step->part));
            return;
        }
        if (revision.role == RigExecBakedRevisionRole::Whole &&
            revision.chunked) {
            RunWholeSkinChunk(B, chain, &revision, size_t(revisionIndex),
                              size_t(step->part));
            return;
        }
        RigExecBakedProgramImpl::GeomChunk &chunk =
            revision.chunks[size_t(step->part)];
        const GfVec3f *points = nullptr;
        size_t count = 0;
        if (revision.role == RigExecBakedRevisionRole::Whole) {
            // A Whole revision's one chunk reads the whole entering version:
            // in place where its groups are one buffer, else gathered into
            // the revision's own buffer.
            RigExecBakedVersionPoints(chain, size_t(revisionIndex),
                                      &revision.wholeEntering, &points,
                                      &count);
        } else {
            PointsAt(chain, size_t(revisionIndex), &points, &count);
        }
        const bool sized = count == revision.precedingCount &&
                           revision.stagingOutput.size() == count;

        if (!revision.chunked) {
            // One chunk is the whole array, so there is nothing to speculate
            // about: it knows everything the fuse knows.
            chunk.ok = false;
            if (!revision.status.AllowsApply()) {
                return;
            }
            if (!skin) {
                // The revision's OWN unpublished buffer, written out of
                // place from the preceding points -- no seed copy before a
                // kernel that rewrites every point -- and swapped in by the
                // fuse rather than copied.
                revision.stagingOutput.resize(count);
                revision.stagingFresh = true;
                // Not a second dispatch that mirrors _RevisionNode::Compute
                // -- the same function the node calls. The packet check, the
                // full-strength fast path, the kernel and the "apply once"
                // blend all live in RigExecRunRevisionKernel, so an
                // operation cannot mean one thing here and another there.
                // This unpublished buffer is discarded by Fuse on failure.
                // A wire blends with the envelope RevisionStatic resolved.
                chunk.ok = geometryDetail::RunDiscardableGeometry(
                    revision.op, revision.parameters, points, count,
                    &revision.stagingOutput, B.useSimd, &revision.wireBasis,
                    &revision.surfaceCache,
                    !revision.fullStrength && revision.envelopeOk
                        ? &revision.envelope : nullptr);
                return;
            }
            if (!SkinPacketIsUsable(revision) ||
                !revision.influencesValid || !sized) {
                return;
            }
            revision.stagingFresh = true;
            chunk.ok = SkinRange(&revision, points,
                                 WholeTransformsView(&revision), 0, count,
                                 /*whole=*/true, B.useSimd,
                                 &revision.stagingOutput);
            return;
        }

        // Chunked, and therefore SPECULATIVE: it has not waited for the
        // fold, so it cannot know whether the revision will apply at all --
        // only that its own vertices, its own joints and the packet say what
        // its range of the output buffer should hold. The fuse discards the
        // work if the revision turns out not to apply.
        chunk.keyChanged = false;
        if (!revision.status.AllowsApply() || !SkinPacketIsUsable(revision) ||
            revision.partitionStale || !sized) {
            chunk.ok = false;
            return;
        }
        GatherChunkTransforms(B, revision, &chunk);
        chunk.ok = SkinRange(&revision, points,
                             ChunkTransformsView(chunk, B.useSimd),
                             size_t(chunk.begin), size_t(chunk.end),
                             /*whole=*/false, B.useSimd,
                             &revision.stagingOutput);
        return;
    }

    case RigExecBakedStepKind::RevisionFuse: {
        if (revision.role == RigExecBakedRevisionRole::Range) {
            RunJoin(B, chain, &revision, step->object, size_t(revisionIndex),
                    step);
            return;
        }
        if (revision.role == RigExecBakedRevisionRole::Whole) {
            RunWholeFuse(B, chain, chainIndex, &revision, step->object,
                         size_t(revisionIndex), step);
            return;
        }
        // Where the dynamic path's assembler would have failed the packet:
        // for a skin revision the matrices are not in it, so "the packet is
        // valid" is the packet's own answer AND the fold's.
        const bool packetValid =
            revision.parameters.valid &&
            (!skin || revision.influencesValid);
        EmitInvalidPacketLine(B, revision, packetValid, &step->diagnostics);
        // Selection by the common graph is the sole execution authority.
        revision.executed = true;
        step->counters.revisionsExecuted = revision.executed ? 1 : 0;
        if (revision.executed) {
            // What the previous publication carried, which the content
            // version is decided against: this revision's own output when it
            // applied, the copy kept of its entering points when it passed
            // through, nothing before its first.
            const bool hadOwn =
                revision.ran && revision.currentSource == int(revisionIndex);
            bool moved = !revision.ran;
            revision.resultStatus = revision.status.state;
            bool applied = packetValid && revision.status.AllowsApply();
            const bool whole = applied && skin && revision.partitionStale;
            // The whole-revision skin's points; a stale partition is rare.
            std::vector<GfVec3f> fused;
            if (whole) {
                const GfVec3f *points = nullptr;
                size_t count = 0;
                PointsAt(chain, size_t(revisionIndex), &points, &count);
                applied = revision.stagingOutput.size() == count &&
                          FuseWholeRevision(&revision, points, count,
                                            B.useSimd, &fused);
            } else if (applied && revision.acceptance !=
                                      RigExecRevisionAcceptance::Deferred) {
                // RevisionStatic's decision, from the validation each chunk's
                // kernel runs first, so every chunk's `ok` is this answer.
                applied = revision.acceptance ==
                          RigExecRevisionAcceptance::Applies;
            } else if (applied) {
                for (const RigExecBakedProgramImpl::GeomChunk &chunk :
                         revision.chunks) {
                    if (!chunk.ok) {
                        applied = false;
                        break;
                    }
                }
            }
            if (applied) {
                if (revision.chunked || whole) {
                    const std::vector<GfVec3f> &result =
                        whole ? fused : revision.stagingOutput;
                    if (!hadOwn) {
                        moved = moved ||
                                !SamePoints(result, revision.passedPoints);
                    }
                    const bool copied = CopyMovedPoints(result, &revision.output);
                    moved = moved || (hadOwn && copied);
                } else if (revision.stagingFresh) {
                    moved = moved ||
                            !SamePoints(revision.stagingOutput,
                                        hadOwn ? revision.output
                                               : revision.passedPoints);
                    // Ownership flips instead of a copy: the chunk's buffer
                    // is published, and the one it replaces is the next
                    // chunk's, at the size the chunk left (RevisionOut keys
                    // the staging size).
                    revision.output.swap(revision.stagingOutput);
                    revision.stagingOutput.resize(revision.output.size());
                    revision.stagingFresh = false;
                } else if (!hadOwn) {
                    // The chunk's latest result is already `output`.
                    moved = moved || !SamePoints(revision.output,
                                                 revision.passedPoints);
                }
                revision.currentSource = int(revisionIndex);
            } else {
                // Nothing was applied, so the chain's running value stays
                // where it was -- an indirection, not a copy. A chunk that
                // ran and produced points is discarded with it, and `output`
                // keeps the last applied points a later apply may republish.
                const GfVec3f *points = nullptr;
                size_t count = 0;
                PointsAt(chain, size_t(revisionIndex), &points, &count);
                const std::vector<GfVec3f> &previous =
                    hadOwn ? revision.output : revision.passedPoints;
                const bool same = RigExecBakedSamePoints(
                    points, count, previous.data(), previous.size());
                moved = moved || !same;
                if (hadOwn || !same) {
                    revision.passedPoints.assign(points, points + count);
                }
                revision.currentSource =
                    revisionIndex == 0
                        ? -1
                        : chain.revisions[size_t(revisionIndex) - 1]
                              .currentSource;
                if (revision.status.AllowsApply()) {
                    revision.resultStatus = _tokens->moverFailed;
                }
            }
            if (moved) {
                ++revision.doneVersion;
            }
            // Keep the published RevisionPacket intact. Ordinary revisions
            // have no duplicate last-parameters snapshot; exact value keys
            // retain the previous semantic opinion in the common workspace.
            revision.lastStatus = revision.status;
            revision.ran = true;
        }
        return;
    }

    default:
        return;
    }
}

// The report.

namespace {

/// Two decimals, so a ratio in the report lines up with the scheduler's.
std::string
Fixed2(double value)
{
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.2f", value);
    return buffer;
}

/// Whether \p step is one of the four steps that make up revision \p id.
bool
IsStepOfRevision(const RigExecBakedStep &step, int id)
{
    if (step.object != id) {
        return false;
    }
    switch (step.kind) {
    case RigExecBakedStepKind::InfluenceFold:
    case RigExecBakedStepKind::RevisionStatic:
    case RigExecBakedStepKind::RevisionChunk:
    case RigExecBakedStepKind::RevisionFuse:
        return true;
    default:
        return false;
    }
}

/// The cost of revision \p id run one step at a time, and the cost of its
/// longest dependency chain -- the floor the chunks cannot go below however
/// many threads there are. The longest path is taken over the revision's own
/// steps only: an edge leaving the revision is the rest of the frame's
/// business, and what this number answers is "how much of this skin is
/// width and how much is depth".
void
SkinCosts(const RigExecBakedProgramImpl &B, int id, double *serial,
          double *criticalPath)
{
    *serial = 0.0;
    *criticalPath = 0.0;
    // Steps are in topological order, so one forward pass settles the
    // longest path into each of them.
    std::vector<double> through(B.steps.size(), 0.0);
    for (size_t index = 0; index < B.steps.size(); ++index) {
        const RigExecBakedStep &step = B.steps[index];
        if (!IsStepOfRevision(step, id)) {
            continue;
        }
        double before = 0.0;
        for (const int pred : step.preds) {
            if (IsStepOfRevision(B.steps[size_t(pred)], id)) {
                before = std::max(before, through[size_t(pred)]);
            }
        }
        through[index] = before + step.cost;
        *serial += step.cost;
        *criticalPath = std::max(*criticalPath, through[index]);
    }
}

}  // namespace

std::string
RigExecBakedGeometryReport(const RigExecBakedProgramImpl &B)
{
    std::string out;
    char line[256];
    for (size_t c = 0; c < B.chains.size(); ++c) {
        const RigExecBakedProgramImpl::GeomChain &chain = B.chains[c];
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            const RigExecBakedProgramImpl::GeomRevision &revision =
                chain.revisions[r];
            if (revision.op != RigExecRevisionOp::Skin) {
                continue;
            }
            const int id = B.chainRevisionBegin[c] + int(r);
            const size_t influences = revision.influenceSlots.size();
            const size_t chunks = revision.chunks.size();
            size_t vertexMin = size_t(-1), vertexMax = 0, vertexSum = 0;
            size_t keyMin = size_t(-1), keyMax = 0, keySum = 0, wide = 0;
            for (const RigExecBakedProgramImpl::GeomChunk &chunk :
                     revision.chunks) {
                // An unpartitioned revision is one chunk over everything,
                // which is every influence and every vertex it has.
                const size_t vertices =
                    revision.chunked ? size_t(chunk.end - chunk.begin)
                                     : revision.partitionPointCount;
                const size_t key =
                    revision.chunked ? chunk.key.size() : influences;
                vertexMin = std::min(vertexMin, vertices);
                vertexMax = std::max(vertexMax, vertices);
                vertexSum += vertices;
                keyMin = std::min(keyMin, key);
                keyMax = std::max(keyMax, key);
                keySum += key;
                if (influences && key * 2 >= influences) {
                    ++wide;
                }
            }
            std::snprintf(
                line, sizeof(line),
                "  %s: %zu chunk(s), %zu vertex(es), %zu influence(s)%s\n",
                revision.moverPath.GetString().c_str(), chunks, vertexSum,
                influences, revision.chunked ? "" : " [not partitioned]");
            out += line;
            std::snprintf(line, sizeof(line),
                          "    vertices/chunk min %zu mean %.1f max %zu\n",
                          vertexMin == size_t(-1) ? 0 : vertexMin,
                          chunks ? double(vertexSum) / double(chunks) : 0.0,
                          vertexMax);
            out += line;
            std::snprintf(
                line, sizeof(line),
                "    |key| min %zu mean %.1f max %zu; %zu/%zu chunk(s) reach "
                "half the influences\n",
                keyMin == size_t(-1) ? 0 : keyMin,
                chunks ? double(keySum) / double(chunks) : 0.0, keyMax, wide,
                chunks);
            out += line;
            // What the partition bought, in the scheduler's own units: the
            // revision's serial cost against its longest dependency chain.
            double skinSerial = 0.0, skinPath = 0.0;
            SkinCosts(B, id, &skinSerial, &skinPath);
            std::snprintf(line, sizeof(line),
                          "    skin serial %.2fus critical path %.2fus%s\n",
                          skinSerial, skinPath,
                          skinPath > 0.0
                              ? (" (" + Fixed2(skinSerial / skinPath) +
                                 "x)").c_str()
                              : "");
            out += line;
            for (size_t k = 0; k < chunks; ++k) {
                const auto &chunk = revision.chunks[k];
                const auto producers = ChunkProducers(revision,chunk,revision.chunked);
                std::snprintf(line, sizeof(line),
                              "    [%zu] %d..%d producers", k, chunk.begin, chunk.end);
                out += line;
                for (const auto &producer : producers)
                    out += " " + std::to_string(producer.first) + ":" +
                           std::to_string(producer.second);
                out += "\n";
            }
            std::snprintf(line, sizeof(line),
                          "    natural producer count min %d max %d distinct sets %zu\n",
                          revision.partitionProducerMin, revision.partitionProducerMax,
                          revision.partitionDistinctReads);
            out += line;
        }
    }
    return out;
}

void
RigExecBakedPublishGeometry(RigExecBakedProgramImpl *program,
                            RigExecRigPose *pose)
{
    RigExecBakedProgramImpl &B = *program;
    RigExecBakedEnsureEpilogueIndex(&B);
    // Chain by chain, in chain order, and inside a chain exactly where the
    // straight line put each line: every revision's assemble diagnostics
    // first, then the status sweep's, then each derived target's. Program
    // order is that order.
    RigExecBakedAppendStepLines(&B, RigExecBakedStepLines::Geometry,
                                &pose->diagnostics);
    // The chains' points, weight fields and derived targets, in the same
    // program order: the maps are keyed, so the order shows only where two
    // steps name one key, and the later step's value stands as it did.
    for (const uint32_t index : B.epilogue.geometrySteps) {
        const RigExecBakedStep &step = B.steps[index];
        if (step.kind == RigExecBakedStepKind::RevisionStatic) {
            const auto &[chainIndex, revisionIndex] =
                B.revisionIndex[size_t(step.object)];
            const RigExecBakedProgramImpl::GeomChain &chain =
                B.chains[size_t(chainIndex)];
            const RigExecBakedProgramImpl::GeomRevision &revision =
                chain.revisions[size_t(revisionIndex)];
            // `haveBase` for the same reason the two arms below test it: a
            // chain whose points stopped reading at this time drives
            // nothing, and the assemble that would have refreshed the field
            // returned before it ever looked at the packet. Its last answer
            // is then last FRAME's, and publishing that is the one shape of
            // staleness a value comparison cannot catch -- the dynamic path
            // publishes no field at all for such a chain.
            if (B.publishWeightFields && chain.haveBase && revision.weightFieldPublished) {
                RigExecResolvedWeightField &field =
                    pose->weightFields[
                        B.weightObjects[size_t(revision.weightObject)].path];
                field.target = revision.weightFieldTarget;
                // Shared, never copied: RevisionStatic replaces the array
                // rather than writing into it.
                field.weights = revision.publishedWeightValues;
            }
        } else if (step.kind == RigExecBakedStepKind::ChainStatus) {
            RigExecBakedProgramImpl::GeomChain &chain =
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
                if (!derived.matrixTarget) {
                    pose->movedProperties[derived.target] =
                        VtValue(derived.result);
                } else if (derived.haveMatrix) {
                    pose->movedProperties[derived.target] =
                        VtValue(derived.matrix);
                }
            }
        }
    }
}

RigExecSurfaceProjectorFrames
RigExecBakedProjectorFrames(const RigExecBakedProgramImpl &B,
                            const RigExecBakedProgramImpl::GeomRevision &revision)
{
    // Asset frames: each provider's rest times its base and final
    // computeMatrix, the tables every geometry revision reads.
    RigExecSurfaceProjectorFrames frames;
    const SdfPath providers[3] = {revision.binding.transform,
                                  revision.binding.transformSpace,
                                  revision.binding.carrySpace};
    const int slots[3] = {revision.transformSlot,
                          revision.transformSpaceSlot,
                          revision.carrySpaceSlot};
    for (int k = 0; k < 3; ++k) {
        if (providers[k].IsEmpty()) continue;
        frames.named[k] = true;
        const int slot = slots[k];
        if (slot < 0 || !B.restFrames[size_t(slot)].IsValid()) continue;
        const auto &rest = B.restFrames[size_t(slot)].points;
        frames.base[k] =
            RigExecWorldFromRest(rest, B.baseMatrix[size_t(slot)]);
        frames.final[k] =
            RigExecWorldFromRest(rest, B.finalMatrix[size_t(slot)]);
        frames.resolved[k] = true;
    }
    return frames;
}

bool
RigExecBakedRunProjectorTarget(
    const RigExecBakedProgramImpl &B,
    const RigExecBakedProgramImpl::GeomChain &chain,
    const RigExecBakedProgramImpl::GeomRevision &revision,
    GfMatrix4d *matrix, std::vector<std::string> *diagnostics)
{
    RigExecRevisionLeafView view;
    view.decl = &revision.leaves.decl;
    view.values = &RigExecBakedResolvePathLeaves(B,revision.leaves);
    RigExecProjectorReads reads;
    RigExecReadProjectorTargetFromLeaves(revision.op, revision.binding, view,
                                         &reads);
    return RigExecRunGeometryMatrix(
        revision.op, revision.binding,
        RigExecBakedProjectorFrames(B, revision), reads,
        revision.surfaceCache.PointSamples(false,chain.lastBase.cdata(),chain.lastBase.size()),
        revision.surfaceCache.PointSamples(true,chain.result.cdata(),chain.result.size()),
        revision.moverPathText, matrix, diagnostics,&revision.surfaceCache);
}

void
RigExecBakedBindPathLeaves(const UsdStageRefPtr &stage,
                           RigExecBakedPathLeaves *leaves)
{
    const size_t n = leaves->decl.keys.size();
    leaves->attributes.assign(n, UsdAttribute());
    leaves->hops.assign(n, std::vector<SdfPath>());
    leaves->varying.assign(n, 0);
    leaves->values.clear();
    leaves->values.reserve(n);
    for (size_t k = 0; k < n; ++k) {
        const RigExecRevisionLeafKey &key = leaves->decl.keys[k];
        if (stage && key.path.IsPropertyPath()) {
            leaves->attributes[k] = stage->GetAttributeAtPath(key.path);
        }
        bool varying = false;
        RigExecRevisionLeafHops(key, leaves->attributes[k], &leaves->hops[k],
                                &varying);
        leaves->varying[k] = varying ? 1 : 0;
        leaves->values.push_back(key.fallback);
    }
    leaves->epoch.assign(n, 0);
    leaves->overrideReached.assign(n, 0);
    for (size_t role = 0; role < leaves->decl.roles.size(); ++role) {
        const int k = leaves->decl.roles[role];
        if (k >= 0 && size_t(k) < n &&
            RigExecRevisionLeafRoleIsTopology(RigExecRevisionLeafRole(role))) {
            leaves->epoch[size_t(k)] = 1;
        }
    }
    leaves->changed.assign(n, 0);
    leaves->mustSample.assign(n, 0);
    leaves->sampled = false;
    RigExecBakedResetPathLeafVersions(leaves);
}

namespace {
// No run has read the key since its versions restarted.
constexpr uint64_t kPathLeafUnread = std::numeric_limits<uint64_t>::max();
}

void
RigExecBakedResetPathLeafVersions(RigExecBakedPathLeaves *leaves)
{
    const size_t n = leaves->values.size();
    leaves->versions.assign(n, 0);
    leaves->observed.assign(n, VtValue());
    leaves->observedVersions.assign(n, 0);
    leaves->observedRuns.assign(n, kPathLeafUnread);
}

bool
RigExecBakedSetPathLeaf(RigExecBakedPathLeaves *leaves, size_t k,
                        VtValue value, uint64_t run)
{
    if (k >= leaves->values.size()) {
        return false;
    }
    const bool changed = !RigExecBakedHeadValueSame(value, leaves->values[k]);
    // RigExecBakedResetPathLeafVersions sizes the four tables together;
    // unsized, every key over them is inexact (InputKey), so a version not
    // kept here can never pass for an unchanged one.
    if (k < leaves->versions.size() && k < leaves->observed.size() &&
        k < leaves->observedVersions.size() && k < leaves->observedRuns.size()) {
        // The first write since a run read the key: what it held is what
        // that run saw.
        if (leaves->observedRuns[k] != run) {
            leaves->observed[k] = leaves->values[k];
            leaves->observedVersions[k] = leaves->versions[k];
            leaves->observedRuns[k] = run;
        }
        // At the observed version the held value has the observed bytes.
        const bool moved =
            leaves->versions[k] == leaves->observedVersions[k]
                ? changed
                : !RigExecBakedHeadValueSame(value, leaves->observed[k]);
        leaves->versions[k] = leaves->observedVersions[k] + (moved ? 1 : 0);
    }
    leaves->values[k] = std::move(value);
    return changed;
}

bool
RigExecBakedHasExternalInputRoute(const RigExecBakedProgramImpl &B,
                                const SdfPath &path)
{
    const auto hasRoute=[&](const auto &revision) {
        if(revision.op!=RigExecRevisionOp::External)return false;
        const auto &binding=revision.binding;
        const auto &leaves=revision.leaves;
        for(size_t i=0;i<binding.externalInputs.size();++i) {
            const auto &key=binding.externalInputs[i];
            using Flavour=RigExecRevisionLeafFlavour;
            if(key.time!=RigExecRevisionLeafTime::AtTime ||
               (key.flavour!=Flavour::Resolved && key.flavour!=Flavour::ResolvedOnly &&
                key.flavour!=Flavour::OverlayThenRaw))continue;
            if(key.path==path)return true;
            if(leaves.decl.externalBegin<0)continue;
            const size_t slot=size_t(leaves.decl.externalBegin)+i;
            if(slot<leaves.hops.size() &&
               std::find(leaves.hops[slot].begin(),leaves.hops[slot].end(),path)!=leaves.hops[slot].end())
                return true;
        }
        return false;
    };
    for(const auto &chain:B.chains) {
        for(const auto &revision:chain.revisions)if(hasRoute(revision))return true;
        for(const auto &derived:chain.derived)if(hasRoute(derived.revision))return true;
    }
    return false;
}

VtValue
RigExecBakedResolvePathLeaf(const RigExecBakedProgramImpl &B,
                           const RigExecBakedPathLeaves &leaves,size_t k)
{
    if(k>=leaves.decl.keys.size()) return VtValue();
    VtValue value=k<leaves.values.size()?leaves.values[k]:VtValue();
    const auto &key = leaves.decl.keys[k];
    // The compiled typed walk owns overlay selection and the original
    // tail-first authored fallback. A direct version shortcut must not
    // discard that fallback when a produced opinion is absent or mistyped.
    if(key.flavour!=RigExecRevisionLeafFlavour::Present &&
       k<leaves.walks.size() && leaves.walks[k]>=0)
        return RigExecBakedSampleWalkedPathLeaf(B,key,leaves.walks[k]);
    const int version = k < leaves.exactVersions.size() ? leaves.exactVersions[k] : -1;
    const int recordIndex = k < leaves.exactRecordIndices.size()
        ? leaves.exactRecordIndices[k] : -1;
    const auto exactDoubleFallback=[&]() {
        if(k<leaves.walks.size() && leaves.walks[k]>=0)
            return RigExecBakedSampleWalkedPathLeaf(B,key,leaves.walks[k]);
        return value.IsHolding<double>()?value:key.fallback;
    };
    if (recordIndex >= 0 && size_t(recordIndex) < B.propertyRecords.size()) {
        const size_t r = size_t(recordIndex);
        const auto &record = B.propertyRecords[r];
        const auto &chain = B.propertyChains[record.chain];
        const size_t source = chain.versionBase + std::min(record.applied,chain.revisions.size());
        const bool present = !B.recordStoodAside[r] &&
            source < B.propertyVersionValid.size() && B.propertyVersionValid[source];
        if (key.flavour == RigExecRevisionLeafFlavour::Present) {
            const bool rawPresent = k < leaves.values.size() &&
                leaves.values[k].IsHolding<bool>() && leaves.values[k].UncheckedGet<bool>();
            value = VtValue(rawPresent || present);
        } else if (present) {
            if(key.type==RigExecRevisionLeafType::Double && !B.recordValues[r].IsHolding<double>())
                return exactDoubleFallback();
            value = B.recordValues[r];
            if (key.type == RigExecRevisionLeafType::Dial &&
                value.IsHolding<float>())
                value = VtValue(double(value.UncheckedGet<float>()));
            else if (key.type == RigExecRevisionLeafType::Float &&
                     value.IsHolding<double>())
                value = VtValue(float(value.UncheckedGet<double>()));
        } else if (key.flavour == RigExecRevisionLeafFlavour::ResolvedOnly) {
            value = key.fallback;
        }
        return value;
    }
    if (version >= 0 && size_t(version) < B.propertyVersionValid.size()) {
        const bool present = B.propertyVersionValid[size_t(version)] != 0;
        if (key.flavour == RigExecRevisionLeafFlavour::Present) {
            const bool rawPresent = k < leaves.values.size() &&
                leaves.values[k].IsHolding<bool>() && leaves.values[k].UncheckedGet<bool>();
            value = VtValue(rawPresent || present);
        } else if (present) {
            const auto &v = B.propertyValues[size_t(version)];
            using Arm = RigExecBakedPropertyChain::Arm;
            const int type = k < leaves.exactValueTypes.size() ? leaves.exactValueTypes[k] : -1;
            const bool asFloat = type == int(Arm::Float);
            const bool numeric = asFloat || type == int(Arm::Double);
            const bool compatible =
                ((key.type == RigExecRevisionLeafType::Float ||
                  key.type == RigExecRevisionLeafType::Dial) && numeric) ||
                (key.type == RigExecRevisionLeafType::Double && type == int(Arm::Double)) ||
                (key.type == RigExecRevisionLeafType::Matrix4d && type == int(Arm::Matrix4d)) ||
                (key.type == RigExecRevisionLeafType::Vec3f && type == int(Arm::Vec3f));
            if (!compatible) {
                if(key.type==RigExecRevisionLeafType::Double)return exactDoubleFallback();
                if (key.flavour == RigExecRevisionLeafFlavour::ResolvedOnly)
                    value = key.fallback;
                return value;
            }
            switch (key.type) {
            case RigExecRevisionLeafType::Float: value = VtValue(asFloat ? v.f : float(v.d)); break;
            case RigExecRevisionLeafType::Double: value = VtValue(v.d); break;
            case RigExecRevisionLeafType::Dial: {
                value = VtValue(asFloat ? double(v.f) : v.d);
                break;
            }
            case RigExecRevisionLeafType::Matrix4d: value = VtValue(v.m); break;
            case RigExecRevisionLeafType::Vec3f: value = VtValue(v.v); break;
            default: break;
            }
        } else if (key.flavour == RigExecRevisionLeafFlavour::ResolvedOnly) {
            value = key.fallback;
        }
    } else if (k < leaves.walks.size() && leaves.walks[k] >= 0) {
        value = RigExecBakedSampleWalkedPathLeaf(B,key,leaves.walks[k]);
    }
    return value;
}

const std::vector<VtValue> &
RigExecBakedResolvePathLeaves(const RigExecBakedProgramImpl &B,
                             const RigExecBakedPathLeaves &leaves)
{
    leaves.consumedValues.resize(leaves.decl.keys.size());
    for(size_t k=0;k<leaves.decl.keys.size();++k)
        leaves.consumedValues[k]=RigExecBakedResolvePathLeaf(B,leaves,k);
    return leaves.consumedValues;
}

void
RigExecBakedSamplePathLeaves(RigExecBakedProgramImpl *program,
                             RigExecBakedPathLeaves *leaves, UsdTimeCode time,
                             bool all, const std::vector<int> &skip)
{
    RigExecBakedProgramImpl &B = *program;
    const bool overrides =
        B.interactiveOverrides && !B.interactiveOverrides->empty();
    all = all || !leaves->sampled;
    const bool overridden = overrides || leaves->overrides;
    const bool chainsMoved = leaves->chainSerial != B.pathLeafChainSerial;
    const bool timeMoved = !leaves->sampled || time != leaves->time;
    // Default and a numeric time read different opinions of an attribute
    // that holds both, whatever its variance.
    const bool defaultMoved =
        timeMoved && (time.IsDefault() || leaves->time.IsDefault());
    RigExecResolvedInputs sourceOnly;
    std::vector<SdfPath> overridePaths;
    if (B.interactiveOverrides) {
        for (const auto &overrideValue : *B.interactiveOverrides) {
            const SdfPath path = overrideValue.attribute.IsEmpty()
                ? overrideValue.prim.AppendProperty(overrideValue.computation)
                : overrideValue.prim.AppendProperty(overrideValue.attribute);
            sourceOnly.SetProperty(path,overrideValue.value);
            overridePaths.push_back(path);
        }
    }
    // Whether a standing override is at a path key \p k's read can reach:
    // the overlay the read consults holds nothing anywhere else.
    const auto overrideReaches = [&](size_t k) {
        if (!overrides || k >= leaves->hops.size()) {
            return false;
        }
        const std::vector<SdfPath> &reach = leaves->hops[k];
        return std::any_of(overridePaths.begin(), overridePaths.end(),
                           [&reach](const SdfPath &path) {
                               return std::find(reach.begin(), reach.end(),
                                                path) != reach.end();
                           });
    };
    std::vector<SdfPath> hops;
    for (size_t k = 0; k < leaves->decl.keys.size(); ++k) {
        leaves->changed[k] = 0;
        if (std::find(skip.begin(), skip.end(), int(k)) != skip.end()) {
            leaves->mustSample[k] = 1;
            continue;
        }
        const RigExecRevisionLeafKey &key = leaves->decl.keys[k];
        const bool rebind = all || leaves->VarianceStale(B.programStamp, k);
        const bool atTime = key.time == RigExecRevisionLeafTime::AtTime;
        // A topology key is epoch state: a standing override moves it only
        // where one stands on its hops, now or at its last sample.
        const bool epoch = k < leaves->epoch.size() && leaves->epoch[k] != 0;
        bool reached = epoch && overrideReaches(k);
        const bool overrideMoves =
            overridden &&
            (!epoch || reached ||
             (k < leaves->overrideReached.size() &&
              leaves->overrideReached[k] != 0));
        if (!rebind && !overrideMoves && !chainsMoved &&
            !(atTime && timeMoved && (leaves->varying[k] || defaultMoved))) {
            continue;
        }
        if (rebind) {
            // An edit can author time samples where there were none.
            bool varying = false;
            RigExecRevisionLeafHops(key, leaves->attributes[k], &hops,
                                    &varying);
            leaves->varying[k] = varying ? 1 : 0;
            if (epoch && k < leaves->hops.size()) {
                leaves->hops[k] = hops;
                reached = overrideReaches(k);
            }
        }
        if (epoch && k < leaves->overrideReached.size()) {
            leaves->overrideReached[k] = reached ? 1 : 0;
        }
        leaves->mustSample[k] = 0;
        ++B.pathLeafSamples;
        // Sampling publishes source opinions only. Produced values are resolved
        // by the owning body after its graph dependencies complete.
        const int walk = k < leaves->walks.size() ? leaves->walks[k] : -1;
        VtValue value = RigExecSampleRevisionLeaf(key, leaves->attributes[k],
                                                &sourceOnly, time, &B.upstream);
        if (key.flavour == RigExecRevisionLeafFlavour::Present) {
            bool present = false;
            if (B.interactiveOverrides) {
                for (const auto &overrideValue : *B.interactiveOverrides) {
                    const SdfPath path = overrideValue.attribute.IsEmpty()
                        ? overrideValue.prim.AppendProperty(overrideValue.computation)
                        : overrideValue.prim.AppendProperty(overrideValue.attribute);
                    if (path == key.path && !overrideValue.value.IsEmpty()) present = true;
                }
            }
            value = VtValue(present);
        }
        leaves->changed[k] =
            RigExecBakedSetPathLeaf(leaves, k, std::move(value), B.pathLeafRun) ? 1 : 0;
        if (walk >= 0 && leaves->changed[k]) {
            B.readerWalkChanged[size_t(walk)] = 1;
        }
    }
    leaves->sampled = true;
    leaves->time = time;
    leaves->stamp = B.programStamp;
    leaves->overrides = overrides;
    leaves->chainSerial = B.pathLeafChainSerial;
}

bool
RigExecBakedLeafVaryingNow(const RigExecBakedProgramImpl &B,
                           const RigExecBakedPathLeaves &leaves, size_t k)
{
    if (k >= leaves.decl.keys.size() || k >= leaves.attributes.size()) {
        return false;
    }
    if (!leaves.VarianceStale(B.programStamp, k)) {
        return k < leaves.varying.size() && leaves.varying[k] != 0;
    }
    std::vector<SdfPath> hops;
    bool varying = false;
    RigExecRevisionLeafHops(leaves.decl.keys[k], leaves.attributes[k], &hops,
                            &varying);
    return varying;
}

// The skin layouts.

RigExecBakedProgramImpl::GeomRevision *
RigExecBakedLayoutRevision(RigExecBakedProgramImpl *program, size_t r)
{
    RigExecBakedProgramImpl &B = *program;
    if (r < B.revisionIndex.size()) {
        const auto &[chain, revision] = B.revisionIndex[r];
        return &B.chains[size_t(chain)].revisions[size_t(revision)];
    }
    r -= B.revisionIndex.size();
    if (r < B.derivedIndex.size()) {
        const auto &[chain, derived] = B.derivedIndex[r];
        return &B.chains[size_t(chain)].derived[size_t(derived)].revision;
    }
    return nullptr;
}

const RigExecBakedProgramImpl::GeomRevision *
RigExecBakedLayoutRevision(const RigExecBakedProgramImpl &program, size_t r)
{
    return RigExecBakedLayoutRevision(
        const_cast<RigExecBakedProgramImpl *>(&program), r);
}

void
RigExecBakedBuildLayoutSteps(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    const size_t count = B.revisionIndex.size() + B.derivedIndex.size();
    for (size_t r = 0; r < count; ++r) {
        RigExecBakedProgramImpl::GeomRevision &revision =
            *RigExecBakedLayoutRevision(&B, r);
        if (revision.op != RigExecRevisionOp::Skin) {
            continue;
        }
        revision.layoutLeaves = RigExecBakedPathLeaves();
        RigExecDeclareSkinLayoutLeaves(revision.moverPath,
                                       &revision.layoutLeaves.decl);
        RigExecBakedBindPathLeaves(B.stage, &revision.layoutLeaves);
        revision.layoutOverlay.assign(revision.layoutLeaves.decl.keys.size(),
                                      VtValue());
        revision.layoutTopologyFixed =
            RigExecSkinLayoutTopologyIsFixed(revision.moverPrim);
        revision.layoutFixed =
            revision.layoutTopologyFixed &&
            RigExecSkinLayoutWeightsAreFixed(revision.moverPrim);
        revision.layoutFixedChanged = false;
        revision.layoutRan = false;
        revision.layoutHandle = nullptr;
        RigExecBakedStep step;
        step.isHead = true;
        step.part = 0;
        step.kind = RigExecBakedStepKind::SkinTopology;
        step.object = int(r);
        step.label = "SkinTopology " + revision.moverPath.GetString();
        step.writes.push_back(RigExecBakedOne(
            RigExecBakedSlotDomain::SkinTopology, uint32_t(r)));
        B.steps.push_back(std::move(step));
    }
}

void
RigExecBakedDeclareLayoutReads(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    for (RigExecBakedStep &step : B.steps) {
        size_t r = 0;
        switch (step.kind) {
        case RigExecBakedStepKind::RevisionStatic:
        case RigExecBakedStepKind::RevisionChunk:
        case RigExecBakedStepKind::RevisionFuse:
            r = size_t(step.object);
            break;
        case RigExecBakedStepKind::Derived:
            r = B.revisionIndex.size() + size_t(step.object);
            break;
        default:
            continue;
        }
        const RigExecBakedProgramImpl::GeomRevision *revision =
            RigExecBakedLayoutRevision(B, r);
        if (revision && revision->op == RigExecRevisionOp::Skin) {
            step.reads.push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::SkinTopology, uint32_t(r)));
        }
    }
}

void
RigExecBakedSampleLayoutLeaves(RigExecBakedProgramImpl *program,
                               RigExecBakedProgramImpl::GeomRevision *revision,
                               UsdTimeCode time, bool all)
{
    RigExecBakedProgramImpl &B = *program;
    RigExecBakedPathLeaves &leaves = revision->layoutLeaves;
    revision->layoutFixedChanged = false;
    // The weights' variance can change on a retained program after an
    // authored weights edit; an edit to the indices or the element size
    // rebuilds it, so the topology half stands from Build. Capture source
    // metadata only when its owning-thread bindings refresh.
    revision->layoutFixed = all || !leaves.sampled
        ? revision->layoutTopologyFixed &&
              RigExecSkinLayoutWeightsAreFixed(revision->moverPrim)
        : RigExecBakedLayoutFixedNow(B, *revision);
    RigExecBakedSamplePathLeaves(&B, &leaves, time, all);

}

bool
RigExecBakedLayoutFixedNow(const RigExecBakedProgramImpl &B,
                           const RigExecBakedProgramImpl::GeomRevision &revision)
{
    if (!revision.layoutTopologyFixed) {
        return false;
    }
    const RigExecBakedPathLeaves &leaves = revision.layoutLeaves;
    const int weights = leaves.decl.Role(RigExecRevisionLeafRole::JointWeights);
    const bool stale = weights >= 0
        ? leaves.VarianceStale(B.programStamp, size_t(weights))
        : leaves.AnyVarianceStale(B.programStamp);
    return stale ? RigExecSkinLayoutWeightsAreFixed(revision.moverPrim)
                 : revision.layoutFixed;
}

void
RigExecBakedRunLayoutOp(const RigExecBakedProgramImpl &B,
                       RigExecBakedProgramImpl::GeomRevision *revision)
{
    using Role = RigExecRevisionLeafRole;
    const RigExecBakedPathLeaves &leaves = revision->layoutLeaves;
    const auto &values = RigExecBakedResolvePathLeaves(B,leaves);
    const auto value = [&](Role role) -> const VtValue * {
        const int key = leaves.decl.Role(role);
        return key >= 0 && size_t(key) < values.size() ? &values[size_t(key)] : nullptr;
    };
    const VtValue *indicesValue = value(Role::JointIndices);
    const VtValue *weightsValue = value(Role::JointWeights);
    const VtValue *sizeValue = value(Role::ElementSize);
    const VtIntArray indices = indicesValue && indicesValue->IsHolding<VtIntArray>()
        ? indicesValue->UncheckedGet<VtIntArray>() : VtIntArray();
    const VtFloatArray weights = weightsValue && weightsValue->IsHolding<VtFloatArray>()
        ? weightsValue->UncheckedGet<VtFloatArray>() : VtFloatArray();
    const int elementSize = sizeValue && sizeValue->IsHolding<int>()
        ? sizeValue->UncheckedGet<int>() : 1;
    auto built = std::make_shared<RigExecSkinTopology>();
    RigExecBuildSkinTopology(
        TfSpan<const int>(indices.cdata(), indices.size()),
        TfSpan<const float>(weights.cdata(), weights.size()), elementSize,
        revision->influenceSlots.size(), built.get());
    // The same layout keeps its object, so the packet that carries it still
    // compares equal and the kernel does not re-run.
    if (revision->layoutCandidate && *revision->layoutCandidate == *built) {
        revision->layoutHandle = revision->layoutCandidate;
        return;
    }
    revision->layoutHandle = std::move(built);
    revision->layoutCandidate = revision->layoutHandle;
    // `layoutSerial` names `layoutCandidate`: the SkinTopology and
    // RevisionPacket keys carry it in place of the layout's bytes. Only a
    // newly built layout -- which differs from the candidate it replaces --
    // moves it; handing the candidate back keeps it.
    ++revision->layoutSerial;
}

namespace {

// The packet fields two assemblies disagree on, by name.
std::string
PacketDifference(const RigExecMoverParameters &a,
                 const RigExecMoverParameters &b)
{
    std::string out;
    const auto field = [&out](bool same, const char *name) {
        if (!same) {
            out += out.empty() ? name : std::string(", ") + name;
        }
    };
    field(a.kind == b.kind, "kind");
    field(a.enabled == b.enabled, "enabled");
    field(a.valid == b.valid, "valid");
    field(a.transform == b.transform, "transform");
    field(a.radialWeight == b.radialWeight, "radialWeight");
    field(a.weights == b.weights, "weights");
    field(a.blendDeltas == b.blendDeltas, "blendDeltas");
    field(a.blendSurfaceFrame == b.blendSurfaceFrame, "blendSurfaceFrame");
    field(a.referenceVolume == b.referenceVolume, "referenceVolume");
    field(a.strength == b.strength, "strength");
    field(a.topologyCounts == b.topologyCounts, "topologyCounts");
    field(a.topologyIndices == b.topologyIndices, "topologyIndices");
    field(a.restPoints == b.restPoints, "restPoints");
    field(a.auxPoints == b.auxPoints, "auxPoints");
    field(a.bindCoords == b.bindCoords, "bindCoords");
    field(a.widths == b.widths, "widths");
    field(a.skinTransforms == b.skinTransforms, "skinTransforms");
    field(a.skinIndices == b.skinIndices, "skinIndices");
    field(a.skinWeights == b.skinWeights, "skinWeights");
    field(a.skinElementSize == b.skinElementSize, "skinElementSize");
    field(a.skinningMethod == b.skinningMethod, "skinningMethod");
    field(a.skinTopology == b.skinTopology, "skinTopology");
    return out.empty() ? std::string("another field") : out;
}

}  // namespace

std::vector<std::string>
RigExecBakedProgramTesting::ShadowAssembly(
    const RigExecBakedProgram &program, const RigExecResolvedInputs &resolved,
    UsdTimeCode time,
    const std::function<void(const SdfPath &, RigExecRevisionLeafDecl *)>
        &edit,
    size_t *compared)
{
    RigExecBakedProgramImpl &B = *program._impl;
    std::vector<std::string> report;
    size_t count = 0;
    // Both assemblies over the caller's resolved inputs, each on a copy of
    // the revision, so the program keeps the state the run left.
    RigExecResolvedInputs local = resolved;
    RigExecResolvedInputs *const saved = B.resolvedInputs;
    B.resolvedInputs = &local;
    const auto shadow = [&](const RigExecBakedProgramImpl::GeomRevision &live,
                            const GfVec3f *base, size_t baseCount) {
        if (!live.leaves.decl.assembles) {
            return;
        }
        ++count;
        RigExecBakedProgramImpl::GeomRevision fromLeaves = live;
        RigExecBakedProgramImpl::GeomRevision fromStage = live;
        if (edit) {
            edit(live.moverPath, &fromLeaves.leaves.decl);
        }
        std::vector<std::string> missing, leafLines, stageLines;
        const RigExecMoverParameters a = RigExecBakedAssembleFromLeaves(
            &B, &fromLeaves, base, baseCount, &leafLines, &missing);
        const RigExecMoverParameters b = AssembleRevisionFromStage(
            B, local, &fromStage, base, baseCount, time, &stageLines,
            /*record=*/false);
        const std::string where = live.moverPath.GetString();
        for (const std::string &name : missing) {
            report.push_back(where + ": undeclared read " + name);
        }
        if (!(a == b)) {
            report.push_back(where + ": packets differ (" +
                             PacketDifference(a, b) + ")");
        }
        if (RigExecStatusForParameters(a, live.moverPath) !=
            RigExecStatusForParameters(b, live.moverPath)) {
            report.push_back(where + ": statuses differ");
        }
        if (leafLines != stageLines) {
            report.push_back(where + ": diagnostic lines differ");
        }
    };
    // A projector's matrix target: its reads from leaves and off the stage.
    const auto projector =
        [&](const RigExecBakedProgramImpl::GeomRevision &live) {
            ++count;
            RigExecBakedProgramImpl::GeomRevision fromLeaves = live;
            if (edit) {
                edit(live.moverPath, &fromLeaves.leaves.decl);
            }
            std::vector<std::string> missing;
            RigExecRevisionLeafView view;
            view.decl = &fromLeaves.leaves.decl;
            view.values = &fromLeaves.leaves.values;
            view.missing = &missing;
            RigExecProjectorReads a, b;
            RigExecReadProjectorTargetFromLeaves(live.op, live.binding, view,
                                                 &a);
            RigExecReadProjectorTarget(live.moverPrim, live.op, live.binding,
                                       &local, time, &b);
            const std::string where = live.moverPath.GetString();
            for (const std::string &name : missing) {
                report.push_back(where + ": undeclared read " + name);
            }
            if (a.rayOrigin != b.rayOrigin ||
                a.rayDirection != b.rayDirection || a.rayUp != b.rayUp ||
                a.shaderOffset != b.shaderOffset ||
                a.reproject != b.reproject || a.dials != b.dials ||
                a.faceVertexCounts != b.faceVertexCounts ||
                a.faceVertexIndices != b.faceVertexIndices) {
                report.push_back(where + ": projector reads differ");
            }
        };
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        if (!chain.haveBase) {
            continue;
        }
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
             chain.revisions) {
            shadow(revision, chain.lastBase.cdata(), chain.lastBase.size());
        }
        if (!chain.haveResult) {
            continue;
        }
        for (const RigExecBakedProgramImpl::GeomChain::Derived &derived :
             chain.derived) {
            if (!derived.matrixTarget && derived.haveBase) {
                shadow(derived.revision, chain.result.cdata(),
                       chain.result.size());
            }
            if (derived.matrixTarget) {
                projector(derived.revision);
            }
        }
    }
    B.resolvedInputs = saved;
    if (compared) {
        *compared = count;
    }
    return report;
}

size_t
RigExecBakedVerifyRangeChains(RigExecBakedProgramImpl *program)
{
    // An independent judge, never a fallback: each range-pipelined revision
    // of a chain that read a base, run whole by the shared kernel over the
    // points entering it, with no caches so nothing the program holds moves.
    // Its ranges hold that answer whether or not they ran this run.
    RigExecBakedProgramImpl &B = *program;
    size_t mismatches = 0;
    std::vector<GfVec3f> whole, enteringGather, publishedGather;
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        if (!chain.haveBase) {
            continue;
        }
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            const RigExecBakedProgramImpl::GeomRevision &revision =
                chain.revisions[r];
            // A revision a cycle set aside passes the base through: no
            // range result to judge.
            if (!revision.rangeRole || revision.rangeSetAside) {
                continue;
            }
            const GfVec3f *entering = nullptr;
            size_t count = 0;
            RigExecBakedVersionPoints(chain, r, &enteringGather, &entering,
                                      &count);
            whole.assign(entering, entering + count);
            const bool decided = RigExecBakedRevisionApplies(revision);
            bool applied = false;
            if (revision.op == RigExecRevisionOp::Skin) {
                // The packet carries identities for a skin's influences, so
                // the judge skins against the table the fold wrote, then
                // blends the envelope RevisionStatic resolved; the decision
                // is the revision's own.
                if (decided) {
                    RigExecSkinTransformsView view;
                    view.transforms = revision.influences.data();
                    view.transformCount = revision.influences.size();
                    applied = RigExecApplySkinKernelWithTransforms(
                        revision.parameters, view, &whole, B.useSimd);
                    if (applied && !revision.fullStrength) {
                        applied = revision.envelopeOk &&
                                  revision.envelope.size() == count;
                        if (applied) {
                            RigExecBlendEnvelopeAll(entering,
                                                    revision.envelope.data(),
                                                    count, whole.data());
                        }
                    }
                }
            } else {
                applied = revision.parameters.valid &&
                          revision.status.AllowsApply() &&
                          RigExecRunRevisionKernel(revision.op,
                                                   revision.parameters, &whole,
                                                   B.useSimd, nullptr, nullptr);
            }
            if (!applied) {
                whole.assign(entering, entering + count);
            }
            // The version the revision left, gathered from its groups: the
            // written ones and, through the gates, the entering ones.
            const GfVec3f *published = nullptr;
            size_t publishedCount = 0;
            RigExecBakedVersionPoints(chain, r + 1, &publishedGather,
                                      &published, &publishedCount);
            const bool same =
                applied == decided && revision.rangeRefusals == 0 &&
                RigExecBakedSamePoints(whole.data(), whole.size(), published,
                                       publishedCount);
            if (!same) {
                ++mismatches;
                TF_VERIFY(false, "%s: the range-pipelined revision's ranges "
                          "disagree with the revision run whole",
                          revision.moverPathText.c_str());
            }
        }
    }
    // Gate and partition holes the bodies counted this run.
    const uint64_t holes = B.gateViolations.load(std::memory_order_relaxed);
    if (holes) {
        mismatches += size_t(holes);
        TF_VERIFY(false, "%llu gated or Range-skin group step(s) met a pin "
                  "hole", static_cast<unsigned long long>(holes));
    }
    B.rangeVerifyMismatches += mismatches;
    return mismatches;
}

void RigExecBakedResetSetAsideGeometryValue(
    RigExecBakedProgramImpl *program, RigExecBakedSlotDomain domain, uint32_t slot)
{
    auto &B = *program;
    if (domain == RigExecBakedSlotDomain::ChainBase ||
        domain == RigExecBakedSlotDomain::ChainPoints) {
        if (slot >= B.chains.size()) return;
        auto &chain = B.chains[slot];
        // An excluded writer never runs, so clearing is the only way the
        // points move: the content version moves with them.
        if (domain == RigExecBakedSlotDomain::ChainBase) {
            chain.haveBase = false;
            if (!chain.lastBase.empty()) ++chain.baseVersion;
            chain.lastBase.clear();
            // The base groups go with it; their versions never go down.
            for (auto &group : chain.baseGroups) {
                RigExecResetGroup(&group);
                group.published = RigExecPointsRef<GfVec3f>();
            }
            chain.baseOwner.reset();
        } else {
            chain.haveResult = false;
            if (!chain.result.empty()) ++chain.resultVersion;
            chain.result.clear();
            chain.resultIds.clear();
        }
        return;
    }
    if (domain == RigExecBakedSlotDomain::DerivedOut) {
        if (slot >= B.derivedIndex.size()) return;
        const auto [chain, part] = B.derivedIndex[slot];
        auto &derived = B.chains[size_t(chain)].derived[size_t(part)];
        derived.haveResult = false;
        derived.haveMatrix = false;
        if (!derived.result.empty()) ++derived.resultVersion;
        derived.result.clear();
        derived.revision.output.clear();
        derived.revision.ran = false;
        return;
    }
    if (domain == RigExecBakedSlotDomain::SkinTopology && slot >= B.revisionIndex.size()) {
        const size_t derivedSlot = slot - B.revisionIndex.size();
        if (derivedSlot >= B.derivedIndex.size()) return;
        const auto [chain, part] = B.derivedIndex[derivedSlot];
        auto &revision = B.chains[size_t(chain)].derived[size_t(part)].revision;
        revision.layoutHandle.reset();
        revision.topology.reset();
        revision.topologyResolved = false;
        revision.layoutUsable = false;
        return;
    }    if (domain == RigExecBakedSlotDomain::RevisionOut) {
        for (size_t revisionIndex = 0; revisionIndex < B.revisionIndex.size(); ++revisionIndex) {
            const int first = B.revisionChunkBase[revisionIndex];
            const int count = B.revisionChunkCount[revisionIndex];
            if (slot < uint32_t(first) || slot >= uint32_t(first + count)) continue;
            const auto [chain, part] = B.revisionIndex[revisionIndex];
            const auto &owner = B.chains[size_t(chain)];
            auto &revision = B.chains[size_t(chain)].revisions[size_t(part)];
            const size_t at = size_t(slot - uint32_t(first));
            if (revision.rangeRole) {
                // A set-aside group makes the revision pass the base through,
                // as an excluded whole fuse does: every reader of a version
                // past it resolves to the base groups (RigExecBakedGroupAt),
                // never to this slot. The slot itself holds the base group,
                // refused; its join publishes the base ids (RunJoin) and
                // ChainInputs leaves the source alone.
                if (at < revision.groups.size()) {
                    revision.groups[at].ok = false;
                    PassSetAsideBaseGroup(owner, at, &revision.groups[at]);
                }
                if (at < revision.chunks.size()) revision.chunks[at].ok = false;
                revision.rangeSetAside = true;
                revision.currentSource = -1;
                return;
            }
            if (owner.groupBounds.size() >= 2 && at >= revision.chunks.size()) {
                // A Whole revision's published group: its fuse is set aside,
                // and passes the base groups through as a set-aside join does.
                const size_t g = at - revision.chunks.size();
                if (g < revision.groups.size()) {
                    PassSetAsideBaseGroup(owner, g, &revision.groups[g]);
                }
                revision.rangeSetAside = true;
                revision.currentSource = -1;
                return;
            }
            revision.stagingOutput.clear();
            revision.stagingFresh = false;
            // The excluded writer owns the full retained staging output;
            // invalidate every chunk opinion along with its cleared bytes,
            // and a Whole revision's speculative group answers with them.
            for (auto &chunk : revision.chunks) chunk.ok = false;
            if (owner.groupBounds.size() >= 2) {
                for (auto &group : revision.groups) group.ok = false;
            }
            return;
        }
        return;
    }    if (slot >= B.revisionIndex.size()) return;
    const auto [chain, part] = B.revisionIndex[slot];
    auto &revision = B.chains[size_t(chain)].revisions[size_t(part)];
    switch (domain) {
    case RigExecBakedSlotDomain::RevisionPacket:
        revision.parameters.valid = false;
        revision.acceptance = RigExecRevisionAcceptance::Refuses;
        break;
    case RigExecBakedSlotDomain::RevisionTransforms:
        revision.influencesValid = false;
        break;
    case RigExecBakedSlotDomain::RevisionOut:
        revision.output.clear();
        break;
    case RigExecBakedSlotDomain::RevisionDone:
    case RigExecBakedSlotDomain::ChainDirty: {
        revision.ran = false;
        revision.executed = false;
        revision.resultStatus = _tokens->operationCycle;
        revision.currentSource = -1;
        const auto &owner = B.chains[size_t(chain)];
        if (owner.groupBounds.size() >= 2) {
            // A join or Whole fuse of a chain cut into groups, set aside:
            // readers past it resolve to the base groups, and its version
            // follows their ids, which is the base it passes through.
            revision.rangeSetAside = true;
            PassSetAsideBaseGroups(owner, &revision);
            break;
        }
        // A range-pipelined revision set aside passes the base through
        // exactly as a whole one does; its ranges' buffer is then read by
        // no one, and ChainInputs leaves the source alone.
        if (revision.rangeRole) revision.rangeSetAside = true;
        // An excluded fuse never runs, so the version follows the base it
        // passes through, against the copy of the base it last carried.
        PassBaseThrough(owner.lastBase, &revision);
        break;
    }
    case RigExecBakedSlotDomain::SkinTopology:
        revision.layoutHandle.reset();
        revision.topology.reset();
        revision.topologyResolved = false;
        revision.layoutUsable = false;
        break;
    default: break;
    }
}
}  // namespace rigExec
