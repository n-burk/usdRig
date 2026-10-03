// Evaluates geometry revisions from the completed pose and exec snapshot.

#include "rigEvaluatorInternal.h"
#include "rigEvaluatorConstraints.h"
#include "parallel.h"
#include "movers/moverRegistry.h"
#include "frameExtraction.h"
#include "solverKernels.h"
#include "rigExecMath/singleChainIk.h"
#include "rigExecMath/solvers.h"

#include "pxr/base/work/dispatcher.h"
#include "pxr/base/work/withScopedParallelism.h"
#include "pxr/base/tf/diagnostic.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usdGeom/xformCache.h"
#include "pxr/usd/usdGeom/xformable.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <unordered_map>

namespace rigExec {

using namespace evaluatorDetail;

namespace {

// Blend attribute names read per channel per frame, interned once.
const TfToken _kEvalBlendWeight("inputs:weight");
const TfToken _kEvalBlendActivation("rigExec:activation");
const TfToken _kEvalDense("dense");
const TfToken _kEvalWeightTarget("rigExec:weightTarget");
const TfToken _kEvalDefaultWeight("inputs:defaultWeight");

} // namespace

bool
RigExecRigEvaluator::_EvaluateGeometry(
    UsdTimeCode time, const RigExecSnapshot &snapshot,
    const std::unordered_map<SdfPath, GfMatrix4d, SdfPath::Hash> &finalMatrices,
    const std::unordered_map<SdfPath, GfMatrix4d, SdfPath::Hash> &constraintDeltas,
    UsdGeomXformCache &constraintXformCache,
    RigExecRigPose &pose)
{
    const UsdPrim assetRoot = _stage->GetPrimAtPath(_rigPath.GetParentPath());
    // The scalar oracle resolves the same provider phases independently.
    std::unordered_map<SdfPath, GfMatrix4d, SdfPath::Hash> baseProviderMatrices;
    std::unordered_map<SdfPath, GfMatrix4d, SdfPath::Hash> finalProviderMatrices;
    // Consumed only by the cpuParityMode _EvaluateChain oracle, so skip
    // populating them on the hot dynamic path.
    if (cpuParityMode) {
        for (const auto &[target, revisions] : _graphChains) {
            for (const _GraphRevision &revision : revisions) {
                if (revision.transformTap >= 0 &&
                    !revision.binding.transform.IsEmpty() &&
                    !baseProviderMatrices.count(revision.binding.transform)) {
                    baseProviderMatrices[revision.binding.transform] =
                        snapshot.Get<GfMatrix4d>(revision.transformTap);
                }
                if (revision.transformSpaceTap >= 0 &&
                    !baseProviderMatrices.count(
                        revision.binding.transformSpace)) {
                    baseProviderMatrices[revision.binding.transformSpace] =
                        snapshot.Get<GfMatrix4d>(revision.transformSpaceTap);
                }
                for (size_t k = 0; k < revision.influenceTaps.size() &&
                                   k < revision.binding.influences.size(); ++k) {
                    const SdfPath &provider = revision.binding.influences[k];
                    if (!baseProviderMatrices.count(provider)) {
                        baseProviderMatrices[provider] =
                            snapshot.Get<GfMatrix4d>(revision.influenceTaps[k]);
                    }
                }
            }
        }
        finalProviderMatrices = baseProviderMatrices;
        for (const auto &[provider, matrix] : finalMatrices) {
            finalProviderMatrices[provider] = matrix;
        }
    }

    // 3. Geometry point chains from the generated applications: the chain
    // head's passive outputs:value bridge extracts the exact native
    // VtVec3fArray (spec §7.2). Unsupported v0.1-alpha operations were
    // skipped by the compiler and pass through with diagnostics.
    // 3b. The compiled mover graph, which is the ONLY producer of geometry
    // (spec §7.2): no generated prim and no derived-stage read stands between
    // the authored mover chain and the value written to movedProperties.
    // Every op reachable in a point chain has its provider values: matrix
    // (computeMatrix + computeWeightPacket), blendShape (summed
    // computeBlendChannel), ribbon / emitGuidePoints
    // (computePointFrameArray), and volumeCorrect / smooth / lattice /
    // surfaceProject, whose inputs are static reads through the binding plus
    // the authored base.
    // Keep topology and computed checkpoints across pulls. Updating a source
    // or packet invalidates only its downstream revisions. The independent
    // CPU reference is available in cpuParityMode for validation.
    // ORDERING, ASSERTED RATHER THAN TRUSTED (second half).
    // WHAT MUST RUN AFTER THE POSE-INTERPOLATOR PHASE: these chains. A
    // RigExecBlendInput's inputs:weight carries a single authored connection
    // to <pose>.outputs:weight, and RigExecResolvedInputs::GetAttribute
    // follows it into the in-memory map -- so a phase that had not run yet
    // would be read as the attribute's AUTHORED zero, with no error raised
    // anywhere and every corrective silently off. An assertion of exactly
    // this kind caught a real inversion on 2026-09-13, where the shapes chain
    // landed after the skin.
    // One map probe per published weight: 121 on the biped, and the whole
    // check measures below the profiler's resolution.
    if (!_poseWeightProperties.empty()) {
        size_t unpublished = 0;
        for (const SdfPath &weight : _poseWeightProperties) {
            if (!_resolvedInputs.Find(weight)) {
                ++unpublished;
            }
        }
        if (!TF_VERIFY(unpublished == 0,
                       "%zu of %zu pose weights were not published before the "
                       "geometry chains; the pose-interpolator phase is out "
                       "of order",
                       unpublished, _poseWeightProperties.size())) {
            pose.diagnostics.push_back(
                std::to_string(unpublished) + " of " +
                std::to_string(_poseWeightProperties.size()) +
                " pose weights were not published before the geometry "
                "chains read them");
        }
    }

    size_t graphChainsBuilt = 0;
    size_t graphRevisionsBuilt = 0;

    // Chains run in dependency order, computed at compile (_chainPlan.order).
    // What one chain produced, held apart from the pose until the walk folds
    // it in.
    // Independent chains can run at the same time, and two of them appending
    // to one diagnostics vector or inserting into one map would be a data
    // race -- and, worse, whichever finished first would decide the order the
    // rig reports things in. So a chain writes only into its own buffer and
    // the walk merges the buffers in chain order: what a serial walk wrote,
    // in the order it wrote it, however the tasks were scheduled.
    struct _ChainWork {
        std::vector<std::string> diagnostics;
        std::vector<std::pair<SdfPath, VtValue>> movedProperties;
        std::map<SdfPath, RigExecResolvedWeightField> weightFields;
        RigExecChainSnapshots snapshots;
        size_t revisionsCreated = 0;
        size_t revisionsExecuted = 0;
        size_t schedulesBuilt = 0;
        size_t chainsBuilt = 0;
        size_t revisionsBuilt = 0;
    };
    const auto mergeChainWork = [&](_ChainWork &work) {
        for (std::string &message : work.diagnostics) {
            pose.diagnostics.push_back(std::move(message));
        }
        for (auto &[path, value] : work.movedProperties) {
            pose.movedProperties[path] = std::move(value);
        }
        for (auto &[path, field] : work.weightFields) {
            pose.weightFields[path] = std::move(field);
        }
        _chainSnapshots.Merge(std::move(work.snapshots));
        pose.moverGraphRevisionsCreated += work.revisionsCreated;
        pose.moverGraphRevisionsExecuted += work.revisionsExecuted;
        pose.moverGraphSchedulesBuilt += work.schedulesBuilt;
        graphChainsBuilt += work.chainsBuilt;
        graphRevisionsBuilt += work.revisionsBuilt;
    };

    // One chain, start to finish. Everything it touches outside `work` is
    // read-only for the duration: the compiled bindings, the exec snapshot
    // this generation extracted, the pose as the constraint walk left it, and
    // its own live graph, which no other chain can reach.
    const auto runChain = [&](const SdfPath &target, _ChainWork &work,
                              UsdGeomXformCache &chainXformCache) {
        // This chain's own records answer first. A phase that names a mover
        // in THIS chain is answered by what the chain has recorded so far,
        // which is in this task's buffer and does not reach the evaluator's
        // store until the walk merges it; everything else -- a provider's
        // frame from the pose walk, another chain from an earlier level --
        // is already there and unchanging while this runs.
        const auto lookupChainSnapshot =
            [&](const SdfPath &path, const RigExecReadPhase &phase,
                const SdfPath &reader) -> const VtValue * {
            if (const VtValue *recorded =
                    work.snapshots.Lookup(path, phase, reader)) {
                return recorded;
            }
            return _chainSnapshots.Lookup(path, phase, reader);
        };
        const auto chainIt = _graphChains.find(target);
        if (chainIt == _graphChains.end()) {
            return;
        }
        const std::vector<_GraphRevision> &revisions = chainIt->second;
        VtVec3fArray basePoints;
        // Compile created a node for every chain and derived target, so this
        // never inserts. An insertion here would be a write into a map the
        // other chains of this level are reading at the same time.
        auto &live = _liveGraphs[target];
        if (live && live->basePointsPushed && live->basePointsStatic) {
            // Already in the source, and an authored base that is not
            // time-varying cannot have moved since: this skips the attribute
            // read and the element-by-element compare inside
            // UpdatePointSource, both of which can only conclude "unchanged".
            // The array itself is still needed below (blend deltas, painted
            // weight fields), and handing it over is a refcount.
            basePoints = live->basePoints;
        } else {
            const UsdAttribute baseAttr = _stage->GetAttributeAtPath(target);
            if (!baseAttr || !baseAttr.Get(&basePoints, time)) {
                return;
            }
            if (live &&
                !live->graph.UpdatePointSource(live->source, basePoints)) {
                // A time-varying point count changes VDF element masks.
                // Replace this target's graph only; independent targets keep
                // their caches.
                live.reset();
            }
            if (!live) {
                live = std::make_unique<_LiveGraph>();
                live->source = live->graph.AddPointSource(target, basePoints);
            }
            live->basePoints = basePoints;
            live->basePointsPushed = true;
            live->basePointsStatic = !baseAttr.ValueMightBeTimeVarying();
        }
        // Match stable operation identities, reconnect surviving nodes, and
        // delete removed nodes. A rebind only updates packets; insertion,
        // removal and reordering rebuild this target's schedule, not its nodes
        // or other targets' checkpoints.
        std::vector<std::pair<SdfPath, RigExecRevisionOp>> identities;
        for (const auto &revision : revisions)
            identities.emplace_back(revision.moverPath, revision.op);
        if (identities != live->identities) {
            std::map<std::pair<SdfPath, RigExecRevisionOp>, VdfMaskedOutput> retained;
            for (size_t i = 0; i < live->identities.size(); ++i)
                retained.emplace(live->identities[i], live->revisions[i]);
            std::vector<VdfMaskedOutput> outputs;
            VdfMaskedOutput previous = live->source;
            for (const auto &identity : identities) {
                const auto found = retained.find(identity);
                VdfMaskedOutput output;
                if (found != retained.end()) {
                    output = found->second;
                    live->graph.ReconnectRevision(output, previous);
                    retained.erase(found);
                } else {
                    output = live->graph.AddRevision(identity.second, previous,
                        RigExecMoverParameters(), RigExecMoverStatus());
                    ++work.revisionsCreated;
                }
                outputs.push_back(output);
                previous = output;
            }
            for (const auto &[identity, output] : retained)
                live->graph.RemoveRevision(output);
            live->identities = std::move(identities);
            live->revisions = std::move(outputs);
        }
        RigExecMoverGraph &graph = live->graph;
        const size_t executionsBefore = graph.GetRevisionExecutionCount();
        const size_t schedulesBefore = graph.GetScheduleBuildCount();
        VdfMaskedOutput head = live->source;
        size_t revisionIndex = 0;
        bool built = true;
        for (const _GraphRevision &revision : revisions) {
            const UsdPrim moverPrim = [&]() {
                RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "GetPrim", "geometry");
                return _stage->GetPrimAtPath(revision.moverPath);
            }();
            if (!moverPrim) {
                built = false;
                break;
            }
            // One overlay per revision: the generation-wide property results,
            // plus whatever THIS revision's declared phases resolve to. The
            // assembler reads inputs by path and never learns a phase exists
            // -- which is what lets a phase apply to any input, including
            // ones added later, without touching the assembler.
            // MEASURED 2026-09-13: declared phases are RARE -- almost every
            // revision has none -- and the copy this used to make
            // unconditionally was then bit-identical to _resolvedInputs, one
            // whole std::map<SdfPath, VtValue> per revision per frame. Copy
            // only when there is actually something to overlay.
            const RigExecResolvedInputs *resolved = &_resolvedInputs;
            std::optional<RigExecResolvedInputs> revisionInputs;
            if (!revision.binding.phases.empty()) {
                revisionInputs = _resolvedInputs;
                resolved = &*revisionInputs;
            }
            for (const auto &[inputPath, phase] : revision.binding.phases) {
                if (const VtValue *v = lookupChainSnapshot(
                        inputPath, phase, revision.moverPath)) {
                    revisionInputs->SetProperty(inputPath, *v);
                } else if (phase.kind != RigExecReadPhaseKind::Preceding) {
                    // Preceding falling through to the stage is correct (the
                    // reader is the chain's first revision, so its preceding
                    // value IS the base). Anything else means the phase named
                    // something that produced nothing.
                    work.diagnostics.push_back(
                        "diag " + revision.moverPath.GetString() +
                        ": read phase '" + phase.GetAsString() + "' for " +
                        inputPath.GetString() +
                        " resolved to nothing; read the authored base");
                }
            }
            RigExecProviderValues values;
            values.resolved = resolved;
            GfMatrix4d transform(1.0);
            RigExecWeightPacket weights;
            RigExecPointFrameArray driverFrames;
            if (revision.driverFramesTap >= 0) {
                driverFrames = snapshot.Get<RigExecPointFrameArray>(
                    revision.driverFramesTap);
                values.driverFrames = &driverFrames;
            }
            if (revision.transformTap >= 0) {
                transform = snapshot.Get<GfMatrix4d>(revision.transformTap);
                values.transform = &transform;
            }
            if (const auto delta = constraintDeltas.find(revision.moverPath);
                delta != constraintDeltas.end()) {
                transform = delta->second;
                values.transform = &transform;
            }
            // A "final" read phase takes the provider's aim-revised matrix.
            // That used to be expressed by binding the generated frame-chain
            // head as the transform provider; now the binding names the
            // provider itself and the revised value is substituted here.
            if (revision.transformFinalPhase) {
                const auto revisedIt =
                    finalMatrices.find(revision.binding.transform);
                if (revisedIt != finalMatrices.end()) {
                    transform = revisedIt->second;
                    values.transform = &transform;
                }
            } else if (revision.binding.transformPhase.kind ==
                       RigExecReadPhaseKind::AtPrim) {
                // The provider's frame as of a named point in the pose walk,
                // rather than its base or its final. Same store the point
                // chains use; the value here is a matrix instead of an array.
                if (const VtValue *v = lookupChainSnapshot(
                        revision.binding.transform,
                        revision.binding.transformPhase,
                        revision.moverPath)) {
                    if (v->IsHolding<GfMatrix4d>()) {
                        transform = v->UncheckedGet<GfMatrix4d>();
                        values.transform = &transform;
                    }
                }
            }
            // Normalize against a matching neutral solve before removing
            // the parent's delta. Both providers must have the same authored
            // rest: inverse(R^-1 N) * (R^-1 P) = N^-1 P.
            const auto referenceMatrix = [&](size_t index) {
                GfMatrix4d matrix = snapshot.Get<GfMatrix4d>(
                    revision.influenceTaps[index]);
                if (revision.transformFinalPhase) {
                    const auto it = finalMatrices.find(
                        revision.binding.influences[index]);
                    if (it != finalMatrices.end()) matrix = it->second;
                }
                return matrix;
            };
            const bool hasReference = revision.op == RigExecRevisionOp::Matrix &&
                                      !revision.influenceTaps.empty();
            if (values.transform && hasReference) {
                transform = RigExecMeasureFromReference(transform, referenceMatrix(0));
            }
            // A space provider: the transform measured against it, read at
            // the same phase, so the points take only the handle's motion
            // inside the space.
            if (values.transform && revision.transformSpaceTap >= 0) {
                GfMatrix4d space =
                    snapshot.Get<GfMatrix4d>(revision.transformSpaceTap);
                if (revision.transformFinalPhase) {
                    const auto revisedIt =
                        finalMatrices.find(revision.binding.transformSpace);
                    if (revisedIt != finalMatrices.end()) {
                        space = revisedIt->second;
                    }
                }
                if (hasReference && revision.influenceTaps.size() > 1) {
                    space = RigExecMeasureFromReference(space, referenceMatrix(1));
                }
                transform = RigExecMeasureInSpace(transform, space);
            }
            // Skin influences: the phase rules of the single transform
            // above, applied to every rigExec:influences entry in order.
            std::vector<GfMatrix4d> influenceTransforms;
            if (!revision.binding.influences.empty()) {
                influenceTransforms.reserve(revision.binding.influences.size());
                for (size_t k = 0; k < revision.binding.influences.size(); ++k) {
                    const SdfPath &provider = revision.binding.influences[k];
                    GfMatrix4d m(1.0);
                    if (k < revision.influenceTaps.size() &&
                        revision.influenceTaps[k] >= 0) {
                        m = snapshot.Get<GfMatrix4d>(revision.influenceTaps[k]);
                    }
                    if (revision.transformFinalPhase) {
                        const auto revisedIt = finalMatrices.find(provider);
                        if (revisedIt != finalMatrices.end()) {
                            m = revisedIt->second;
                        }
                    } else if (revision.binding.transformPhase.kind ==
                               RigExecReadPhaseKind::AtPrim) {
                        if (const VtValue *v = lookupChainSnapshot(
                                provider, revision.binding.transformPhase,
                                revision.moverPath)) {
                            if (v->IsHolding<GfMatrix4d>()) {
                                m = v->UncheckedGet<GfMatrix4d>();
                            }
                        }
                    }
                    influenceTransforms.push_back(m);
                }
                values.influenceTransforms = &influenceTransforms;
            }
            if (revision.weightTap >= 0) {
                weights =
                    snapshot.Get<RigExecWeightPacket>(revision.weightTap);
                values.weights = &weights;

                // rigExec:samplePhase = "current": the field is measured
                // against the points AS THEY STAND HERE, not the
                // authored base, so the volume grabs whatever is inside
                // it right now.
                // It cannot come from exec. The revision node's only
                // inputs are its parameters, its status, and the
                // read-write point buffer, and the parameters are baked
                // as a VDF constant when the graph is built -- nothing
                // in the packet can depend on a value the graph has not
                // computed yet. What CAN be done is evaluate the chain
                // built SO FAR (RigExecMoverGraph::Evaluate is const and
                // takes any masked output), measure against that, and
                // bake the result into this revision's parameters. One
                // extra graph evaluation per current-phase revision,
                // paid only by rigs that ask for it.
                if (_currentPhaseWeights.count(
                        revision.binding.weightObject)) {
                    RIGEXEC_PROFILE_SCOPE_CAT(
                        _profiler, "CurrentPhaseEvaluate", "geometry");
                    const VtVec3fArray inFlight = graph.Evaluate(head);
                    const std::vector<GfVec3f> currentPoints(
                        inFlight.begin(), inFlight.end());
                    std::vector<float> field;
                    std::string weightError;
                    if (_ResolveWeights(revision.binding.weightObject,
                                        currentPoints.size(), time, &field,
                                        &weightError, &currentPoints)) {
                        weights.representation = _kEvalDense;
                        weights.values = std::move(field);
                        weights.indices.clear();
                        weights.defaultWeight = 0.0f;
                        weights.valid = true;
                    } else {
                        // An invalid packet is the kernel's atomic
                        // MoverFailed pass-through, which is the right
                        // answer here: publishing the reference-phase
                        // field instead would silently be a different
                        // deformation.
                        weights = RigExecWeightPacket();
                        work.diagnostics.push_back(
                            "current-phase weight failed: " + weightError);
                    }
                }
                if (weights.valid && _publishWeightFields) {
                    RigExecResolvedWeightField &field =
                        work.weightFields[revision.binding.weightObject];
                    SdfPathVector declaredTargets;
                    if (const UsdPrim weightPrim = _stage->GetPrimAtPath(
                            revision.binding.weightObject)) {
                        if (const UsdRelationship rel =
                                weightPrim.GetRelationship(
                                    _kEvalWeightTarget)) {
                            rel.GetTargets(&declaredTargets);
                        }
                    }
                    const bool operationDomain =
                        declaredTargets.size() == 1 &&
                        declaredTargets[0] == revision.moverPath;
                    field.target = operationDomain ? revision.moverPath : target;
                    const size_t logicalCount =
                        operationDomain ? size_t(1) : basePoints.size();
                    // ResolveAll matches the kernel: O(n+m) scatter for
                    // sparse packets, range-policy checks, atomic on
                    // failure -- the same values the mover consumed.
                    if (!weights.ResolveAll(logicalCount, &field.weights)) {
                        field.weights.clear();
                    }
                }
            }
            // basePoints is the chain-wide authored base, invariant across
            // revisions. Only the ops whose assemble path reads
            // values.basePoints (and the blend-input path below) need it;
            // every other revision would pay a full base copy for nothing.
            const bool needsBasePoints =
                !revision.binding.blendInputs.empty() ||
                revision.op == RigExecRevisionOp::BlendShape ||
                revision.op == RigExecRevisionOp::VolumeCorrect ||
                revision.op == RigExecRevisionOp::DeltaMush ||
                revision.op == RigExecRevisionOp::Wrinkle ||
                revision.op == RigExecRevisionOp::External ||
                revision.op == RigExecRevisionOp::Lattice ||
                revision.op == RigExecRevisionOp::RecomputeNormals ||
                revision.op == RigExecRevisionOp::RecomputeExtent;
            if (needsBasePoints) {
                values.basePoints.assign(basePoints.begin(), basePoints.end());
            }
            if (revision.op == RigExecRevisionOp::Skin &&
                revision.skinTopologyFixed) {
                values.skinTopologyCache = &_skinTopologies;
            }
            if (!revision.binding.blendInputs.empty()) {
                std::vector<RigExecBlendChannel> channels;
                for (const SdfPath &input : revision.binding.blendInputs) {
                    RigExecBlendChannel channel;
                    const UsdPrim inputPrim = _stage->GetPrimAtPath(input);
                    _resolvedInputs.GetAttribute(inputPrim.GetAttribute(_kEvalBlendWeight),
                                                 time, &channel.weight);
                    const auto sampleBindings = revision.binding.blendSamples.find(input);
                    if (sampleBindings != revision.binding.blendSamples.end()) {
                        for (const auto &binding : sampleBindings->second) {
                            RigExecBlendSampleData sample;
                            const UsdPrim samplePrim = _stage->GetPrimAtPath(binding.sample);
                            _resolvedInputs.GetAttribute(samplePrim.GetAttribute(_kEvalBlendActivation),
                                                         time, &sample.activation);
                            if (!binding.blendShape.IsEmpty()) {
                                // Sparse: the shape is epoch-constant, so it
                                // is resolved once and shared by pointer.
                                // This is the whole reason the relationship
                                // exists -- the dense branch below reads and
                                // copies 26,276 points per sample per frame
                                // whether the channel is at 0 or at 1.
                                sample.layout = _blendSampleShapes.Resolve(
                                    binding.sample,
                                    [&](RigExecBlendSampleLayout *layout) {
                                        return RigExecResolveBlendSampleLayout(
                                            _stage, binding.blendShape,
                                            values.basePoints.size(), layout);
                                    });
                                if (!sample.layout) {
                                    // Refused the cache: something about the
                                    // shape can move inside this epoch, so it
                                    // is read per frame instead.
                                    auto perFrame = std::make_shared<
                                        RigExecBlendSampleLayout>();
                                    RigExecResolveBlendSampleLayout(
                                        _stage, binding.blendShape,
                                        values.basePoints.size(),
                                        perFrame.get());
                                    sample.layout = perFrame;
                                }
                                channel.samples.push_back(std::move(sample));
                                continue;
                            }
                            VtVec3fArray points;
                            const VtValue *phased = lookupChainSnapshot(
                                binding.points, binding.phase, revision.moverPath);
                            if (phased && phased->IsHolding<VtVec3fArray>()) {
                                points = phased->UncheckedGet<VtVec3fArray>();
                            } else {
                                _resolvedInputs.GetAttribute(_stage->GetAttributeAtPath(binding.points),
                                                             time, &points);
                            }
                            sample.points.assign(points.begin(), points.end());
                            channel.samples.push_back(std::move(sample));
                        }
                    }
                    std::stable_sort(channel.samples.begin(), channel.samples.end(),
                        [](const auto &a, const auto &b) { return a.activation < b.activation; });
                    channels.push_back(std::move(channel));
                }
                // A structural failure leaves blendDeltas empty, which is
                // what makes the assembled packet invalid -- the same atomic
                // MoverFailed pass-through the kernel produces.
                if (!RigExecSumBlendChannels(channels, values.basePoints,
                                             &values.blendDeltas)) {
                    values.blendDeltas.clear();
                }
            }

            const RigExecMoverParameters parameters = [&]() {
                RIGEXEC_PROFILE_SCOPE_CAT(
                    _profiler,
                    "Assemble " + revision.moverPath.GetName(), "geometry");
                return RigExecAssembleParameters(moverPrim, revision.op,
                                                 revision.binding, values,
                                                 time);
            }();
            if (parameters.enabled && !parameters.valid &&
                revision.binding.weightObject.IsEmpty()) {
                const float scalar = _ResolvedRead(
                    _resolvedInputs, moverPrim, _kEvalDefaultWeight,
                    1.0f, time);
                if (!std::isfinite(scalar) || scalar < 0.0f ||
                    scalar > 1.0f) {
                    work.diagnostics.push_back(
                        "MoverFailed " + revision.moverPath.GetString() +
                        ": inputs:defaultWeight must be finite and in "
                        "[0, 1]; revision passed through");
                }
            } else if (parameters.enabled && !parameters.valid &&
                       !revision.binding.weightObject.IsEmpty() &&
                       (!values.weights || !values.weights->valid)) {
                work.diagnostics.push_back(
                    "MoverFailed " + revision.moverPath.GetString() +
                    ": rigExec:weightObject produced an invalid common "
                    "envelope; revision passed through");
            }
            head = live->revisions[revisionIndex++];
            {
                RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "UpdateRev", "geometry");
                graph.UpdateRevision(
                    head, parameters,
                    RigExecStatusForParameters(parameters, revision.moverPath));
            }
            ++work.revisionsBuilt;

            // Snapshot only where a phased read named this revision. The
            // compile pass reduced every phase to one revision, so this is
            // the whole cost of the feature for a rig that uses it, and
            // nothing at all for one that does not.
            const auto wanted = _chainPlan.snapshots.find(target);
            if (wanted != _chainPlan.snapshots.end() &&
                wanted->second.count(revision.moverPath)) {
                RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "SnapshotEvaluate", "geometry");
                work.snapshots.Record(target, revision.moverPath,
                                       VtValue(graph.Evaluate(head)));
            }
        }
        if (!built) {
            return;
        }

        const VtVec3fArray graphPoints = [&]() {
            RIGEXEC_PROFILE_SCOPE_CAT(
                _profiler, "GraphEvaluate", "geometry");
            return graph.Evaluate(head);
        }();
        for (size_t i = 0; i < live->revisions.size(); ++i) {
            if (graph.GetRevisionStatus(live->revisions[i]).state == "moverFailed") {
                work.diagnostics.push_back("MoverFailed " + revisions[i].moverPath.GetString() +
                    ": execution rejected its inputs; revision passed through");
            }
        }
        work.revisionsExecuted +=
            graph.GetRevisionExecutionCount() - executionsBefore;
        work.schedulesBuilt +=
            graph.GetScheduleBuildCount() - schedulesBefore;
        work.movedProperties.emplace_back(target, VtValue(graphPoints));
        // `final` costs nothing extra: this is the value the chain publishes.
        work.snapshots.RecordFinal(target, VtValue(graphPoints));
        ++work.chainsBuilt;

        // Derived maintenance reads this chain's final points, which is why it
        // runs here rather than as another entry in _graphChains.
        const auto derivedIt = _graphDerivedChains.find(target);
        if (derivedIt == _graphDerivedChains.end()) {
            return;
        }
        RIGEXEC_PROFILE_SCOPE_CAT(
            _profiler, "Derived " + target.GetString(), "geometry");
        for (const _GraphRevision &derived : derivedIt->second) {
            VtVec3fArray derivedBase;
            const UsdAttribute derivedAttr =
                _stage->GetAttributeAtPath(derived.target);
            if (!derivedAttr || !derivedAttr.Get(&derivedBase, time)) {
                continue;
            }
            RigExecProviderValues values;
            values.resolved = &_resolvedInputs;
            values.basePoints.assign(graphPoints.begin(), graphPoints.end());

            const RigExecMoverParameters parameters = [&]() {
                RIGEXEC_PROFILE_SCOPE_CAT(
                    _profiler,
                    "AssembleDerived " + derived.target.GetName(),
                    "geometry");
                return RigExecAssembleParameters(
                    _stage->GetPrimAtPath(derived.moverPath), derived.op,
                    derived.binding, values, time);
            }();
            // The recompute is a pure function of the assembled inputs: an
            // unchanged tuple republishes the stored result and defers the
            // derived graph entirely.
            auto &deferred = _derivedCache[derived.target];
            if (deferred.cached &&
                parameters.auxPoints == deferred.points &&
                derivedBase == deferred.base &&
                parameters.topologyCounts == deferred.topologyCounts &&
                parameters.topologyIndices == deferred.topologyIndices &&
                parameters.widths == deferred.widths) {
                work.movedProperties.emplace_back(
                    derived.target, VtValue(deferred.result));
                // A republished revision is still a revision the chain
                // holds; the count is what the walk reports.
                ++work.revisionsBuilt;
                ++work.chainsBuilt;
                continue;
            }
            // Pre-created at Compile with every other chain node.
            auto &derivedLive = _liveGraphs[derived.target];
            if (derivedLive && !derivedLive->graph.UpdatePointSource(
                    derivedLive->source, derivedBase)) {
                derivedLive.reset();
            }
            if (!derivedLive) {
                derivedLive = std::make_unique<_LiveGraph>();
                derivedLive->source = derivedLive->graph.AddPointSource(
                    derived.target, derivedBase);
                derivedLive->revisions.push_back(
                    derivedLive->graph.AddRevision(
                        derived.op, derivedLive->source, parameters,
                        RigExecStatusForParameters(parameters,
                                                   derived.moverPath)));
                ++work.revisionsCreated;
            }
            RigExecMoverGraph &derivedGraph = derivedLive->graph;
            const size_t derivedExecutions =
                derivedGraph.GetRevisionExecutionCount();
            const size_t derivedSchedules = derivedGraph.GetScheduleBuildCount();
            const VdfMaskedOutput &derivedHead = derivedLive->revisions.front();
            derivedGraph.UpdateRevision(
                derivedHead, parameters,
                RigExecStatusForParameters(parameters, derived.moverPath));
            ++work.revisionsBuilt;

            const VtVec3fArray derivedResult = [&]() {
                RIGEXEC_PROFILE_SCOPE_CAT(
                    _profiler, "DerivedEvaluate", "geometry");
                return derivedGraph.Evaluate(derivedHead);
            }();
            if (derivedGraph.GetRevisionStatus(derivedHead).state == "moverFailed") {
                deferred.cached = false;
                work.diagnostics.push_back("MoverFailed " + derived.target.GetString() +
                    ": derived geometry input/cardinality validation failed");
            } else {
                deferred.points = parameters.auxPoints;
                deferred.base = derivedBase;
                deferred.topologyCounts = parameters.topologyCounts;
                deferred.topologyIndices = parameters.topologyIndices;
                deferred.widths = parameters.widths;
                deferred.result = derivedResult;
                deferred.cached = true;
            }
            work.revisionsExecuted +=
                derivedGraph.GetRevisionExecutionCount() - derivedExecutions;
            work.schedulesBuilt +=
                derivedGraph.GetScheduleBuildCount() - derivedSchedules;
            work.movedProperties.emplace_back(derived.target, VtValue(derivedResult));
            ++work.chainsBuilt;
        }
    };

    // Level by level, and inside a level one task per chain wherever Compile
    // classified that as safe. A level that is not -- and every level when
    // the kill switch is off -- is walked in order on this thread, which is
    // what the whole walk was before levels existed.
    const auto runChainHere = [&](const SdfPath &target) {
        _ChainWork work;
        runChain(target, work, constraintXformCache);
        mergeChainWork(work);
    };
    if (_chainPlan.levels.empty()) {
        // No partition at all -- a rig with no point chains. The chain order
        // is then the whole walk, and it is empty too.
        for (const SdfPath &target : _chainPlan.order) {
            runChainHere(target);
        }
    }
    for (const _ChainLevel &level : _chainPlan.levels) {
        if (!level.parallel || !RigExecParallelEvaluationEnabled()) {
            for (const SdfPath &target : level.targets) {
                runChainHere(target);
            }
            continue;
        }
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "ChainLevel", "geometry");
        std::vector<_ChainWork> levelWork(level.targets.size());
        WorkWithScopedParallelism([&]() {
            WorkDispatcher dispatcher;
            for (size_t i = 0; i < level.targets.size(); ++i) {
                dispatcher.Run([&, i]() {
                    // Its own transform cache: the cache memoizes, so one
                    // shared between tasks would be a shared mutable map, and
                    // what it memoizes is a stage read each task can make for
                    // itself.
                    UsdGeomXformCache taskXformCache(time);
                    runChain(level.targets[i], levelWork[i], taskXformCache);
                });
            }
            // The dispatcher joins on the way out of this scope, so nothing
            // below reads a buffer a task is still writing.
        });
        for (_ChainWork &work : levelWork) {
            mergeChainWork(work);
        }
    }
    // The same chains against the scalar CPU reference (spec §7.4). This is
    // the oracle that OUTLIVES the generated-prim chains: it resolves every
    // input off the authored stage itself and authors nothing, so it survives
    // the compiler's deletion, and it is a genuinely independent
    // implementation -- it does not call RigExecAssembleParameters, which is
    // why it can catch a packet-assembly drift rather than share one.
    size_t parityAgreements = 0;
    if (cpuParityMode) {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Parity", "parity");
        std::map<SdfPath, std::vector<const RigExecMoverRecord *>> chains;
        for (const RigExecMoverRecord &mover : _movers) {
            for (const SdfPath &target : mover.targets) {
                if (target.IsPropertyPath() &&
                    target.GetNameToken() == "points") {
                    chains[target].push_back(&mover);
                }
            }
        }
        for (const auto &[target, chain] : chains) {
            const auto graphIt = pose.movedProperties.find(target);
            if (graphIt == pose.movedProperties.end() ||
                !graphIt->second.IsHolding<VtVec3fArray>()) {
                continue;
            }
            const RigExecMoverRecord *unoracled = nullptr;
            for (const RigExecMoverRecord *mover : chain) {
                const RigExecMoverHandler *parityHandler =
                    RigExecFindMoverHandler(mover->schemaType);
                if (parityHandler && !parityHandler->hasScalarOracle) {
                    unoracled = mover;
                    break;
                }
            }
            if (unoracled) {
                pose.diagnostics.push_back(
                    "cpu reference parity: " + target.GetString() +
                    " skipped, its chain contains a " +
                    unoracled->schemaType.GetString());
                continue;
            }
            RIGEXEC_PROFILE_SCOPE_CAT(
                _profiler, "ParityChain " + target.GetString(), "parity");
            std::vector<std::string> quiet;
            const VtVec3fArray reference =
                _EvaluateChain(
                    target, chain, pose, baseProviderMatrices,
                    finalProviderMatrices, time, &quiet, constraintDeltas);
            const VtVec3fArray &graphValue =
                graphIt->second.UncheckedGet<VtVec3fArray>();
            if (reference.size() != graphValue.size()) {
                pose.diagnostics.push_back(
                    "cpu reference parity: size mismatch on " +
                    target.GetString());
                ++pose.moverGraphParityMismatches;
                continue;
            }
            bool agrees = true;
            for (size_t i = 0; i < reference.size(); ++i) {
                if (!GfIsClose(reference[i], graphValue[i], 1e-4)) {
                    pose.diagnostics.push_back(
                        "cpu reference parity: value mismatch on " +
                        target.GetString() + " at element " +
                        std::to_string(i));
                    ++pose.moverGraphParityMismatches;
                    agrees = false;
                    break;
                }
            }
            if (agrees) {
                ++parityAgreements;
            }
        }
    }

    // Report actual reference comparisons, not every graph/derived chain that
    // happened to build.  Derived normals/extents are not part of this scalar
    // point-chain oracle and a mismatched point chain is never an agreement.
    pose.moverGraphParityAgreements = parityAgreements;
    if (cpuParityMode) {
        pose.diagnostics.push_back(
            "mover graph parity: " + std::to_string(parityAgreements) +
            " point chain(s) agreed, " +
            std::to_string(pose.moverGraphParityMismatches) + " mismatched");
    }
    pose.diagnostics.push_back(
        "mover graph: " + std::to_string(graphChainsBuilt) +
        " chain(s), " + std::to_string(graphRevisionsBuilt) +
        " revision(s); " + std::to_string(pose.moverGraphRevisionsCreated) +
        " created, " + std::to_string(pose.moverGraphRevisionsExecuted) +
        " executed, " + std::to_string(pose.moverGraphSchedulesBuilt) +
        " schedule(s) built");
    if (pose.moverGraphParityMismatches != 0) {
        pose.diagnostics.push_back(
            "mover graph parity failed; refusing to publish generation");
        return false;
    }
    // Optional CPU reference-kernel parity for the lowered chains
    // (scalar-reference goldens, spec §7.4).
    if (cpuParityMode) {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "ParityPublish", "parity");
        std::map<SdfPath, std::vector<const RigExecMoverRecord *>> chains;
        for (const RigExecMoverRecord &mover : _movers) {
            for (const SdfPath &target : mover.targets) {
                if (target.IsPropertyPath() &&
                    target.GetNameToken() == "points") {
                    chains[target].push_back(&mover);
                }
            }
        }
        for (const auto &[target, chain] : chains) {
            const RigExecMoverRecord *unoracled = nullptr;
            for (const RigExecMoverRecord *mover : chain) {
                const RigExecMoverHandler *parityHandler =
                    RigExecFindMoverHandler(mover->schemaType);
                if (parityHandler && !parityHandler->hasScalarOracle) {
                    unoracled = mover;
                    break;
                }
            }
            if (unoracled) {
                pose.diagnostics.push_back(
                    "cpu parity: " + target.GetString() +
                    " skipped, its chain contains a " +
                    unoracled->schemaType.GetString());
                continue;
            }
            pose.movedPropertiesCpu[target] = VtValue(_EvaluateChain(
                target, chain, pose, baseProviderMatrices,
                finalProviderMatrices, time, &pose.diagnostics, constraintDeltas));
        }
    }

    return true;
}

} // namespace rigExec
