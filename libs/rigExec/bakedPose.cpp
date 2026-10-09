#include "rigExec/weightField.h"
// The baked program's pose half: the interleaved solver/constraint walk, at
// bake and at run, plus the rest->pose matrices and the frame publication it
// feeds.
// Split out of bakedProgram.cpp so that a domain's bake and its frame path
// are read side by side -- the two have to agree about what was captured and
// what is re-read, and that agreement is the whole correctness argument. The
// evaluator is not visible here: everything this file needs of it was
// captured into RigExecBakedProgramImpl at Build (see bakedProgramImpl.h),
// and the compiled walk arrives restated as RigExecBakedWalkEntry.
#include "bakedProgramImpl.h"
#include "bakedExecCrossCheckRows.h"

#include "frameExtraction.h"
#include "frozenContextInternal.h"
#include "moverGraph.h"
#include "rigEvaluator.h"
#include "solverKernels.h"
#include "rigExecGraph/sceneCompileInputs.h"
#include "rigExecGraph/solverSceneLowering.h"
#include "rigExecGraph/constraintSceneLowering.h"
#include "types.h"

#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/pointFrame.h"
#include "rigExecMath/solvers.h"
#include "rigExecMath/splineIk.h"

#include "pxr/base/gf/math.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/tf/diagnostic.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/metrics.h"
#include "pxr/base/work/threadLimits.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace rigExec {

// Bake.

void
RigExecBakedBuildWalk(RigExecBakedBuildContext *ctx,
                      const std::vector<RigExecBakedWalkEntry> &walk)
{
    RigExecBakedProgramImpl &B = *ctx->program;
    const int N = int(B.paths.size());
    // The bake bodies below were lambdas of Build and closed over its
    // helpers; naming them here keeps each body the one that was reviewed
    // against the dynamic walk it mirrors.
    auto refuse = [&](const std::string &what, const SdfPath &where) {
        ctx->Refuse(what, where);
    };
    auto fold = [&](const UsdPrim &prim, const char *name) {
        ctx->Fold(prim, name);
    };
    auto foldShape = [&](const UsdPrim &prim, const char *name) {
        ctx->FoldShape(prim, name);
    };
    const RigExecSceneCompileInputs compileInputs(*B.sceneDescriptors);
    const auto readFact=[&](const SdfPath &path,UsdTimeCode identity,auto *value) {
        RigExecSceneBoundInput input;
        return compileInputs.Bind(path,RigExecSceneReadRoute::Raw,&input) &&
               compileInputs.Read(input,identity,value);
    };
    auto readToken = [&](const UsdPrim &prim, const char *name,
                         const char *fallback) {
        TfToken value(fallback);
        readFact(prim.GetPath().AppendProperty(TfToken(name)),UsdTimeCode::Default(),&value);
        return value;
    };
    auto targets = [&](const UsdPrim &prim, const char *name) {
        return compileInputs.Targets(prim.GetPath().AppendProperty(TfToken(name)));
    };
    auto bind = [&](const UsdPrim &prim, const char *name, auto fallback) {
        return ctx->Bind(prim, name, fallback);
    };
    auto slotOf = [&](const SdfPath &path) { return ctx->SlotOf(path); };
    // "Publishes computePointFrame and computeRestFrame", which is what a
    // solver relationship target has to do for its frame to reach the
    // computation at all: the two are registered for RigExecJoint,
    // RigExecControl and the volume weights, and for nothing else. An
    // xform-derived slot is in the table so a constraint can name it, but
    // the walk seeds it from the stage and it declares no computation, so a
    // solver naming one reads a null pointer and publishes nothing.
    auto providerSlot = [&](const SdfPath &path) {
        const int slot = slotOf(path);
        return slot >= 0 && B.slotKind[size_t(slot)] ==
                                RigExecBakedSlotKind::FirstFramePose
                   ? slot
                   : -1;
    };

    std::map<SdfPath,RigExecPointFrame> providerRests;
    for(size_t slot=0;slot<B.paths.size();++slot)
        if(B.slotKind[slot]==RigExecBakedSlotKind::FirstFramePose)
            providerRests.emplace(B.paths[slot],B.restFrames[slot]);

    auto bakeSolver = [&](const SdfPath &solverPath,
                          const std::vector<std::pair<SdfPath, int>>
                              &jointOutputs,
                          const std::vector<char> &snapshotJoints,
                          const SdfPathVector &liveRestJoints) {
        RigExecBakedProgramImpl::Solver s;
        // Every slot whose REST this description measures from. Recorded at
        // the point the rest is actually read, so the list is exactly what
        // RigExecBakedRefreshSolverRests has to rebuild from and cannot
        // drift from it by one solver type.
        auto foldRest = [&s](int slot) {
            if (slot >= 0) {
                s.restSlots.push_back(slot);
            }
            return slot;
        };
        s.path = solverPath;
        s.pathText = solverPath.GetString();
        const UsdPrim prim = B.stage->GetPrimAtPath(solverPath);
        const auto *node=compileInputs.Node(solverPath);
        s.type = node?node->fact.type:TfToken();

        // rigExec:joints is the single declaration of the chain: a solver
        // whose output is consumed downstream claims no joints but still
        // names them for their REST measurements.
        std::vector<std::pair<int, int>> restRefs;
        {
            const SdfPathVector joints = targets(prim, "rigExec:joints");
            VtIntArray elements;
            fold(prim, "rigExec:jointElements");
            readFact(solverPath.AppendProperty(TfToken("rigExec:jointElements")),UsdTimeCode::Default(),&elements);
            s.execJointElements = elements;
            // exec binds ONE rest input per rigExec:joints target that
            // PUBLISHES computeRestFrame -- a target that publishes none
            // contributes no input at all -- and the computation then
            // compares the AUTHORED jointElements length against THAT count
            // and indexes the remap by position within it (the TwoBoneIk and
            // SplineIk rest loops in computations.cpp). Resolving against the
            // relationship's target count instead would remap by the wrong
            // index, and would keep remapping where exec gives up.
            // A FirstFramePose slot is what "publishes computeRestFrame" means: an
            // xform-derived slot exists in the table so a constraint can name
            // it, but exec seeds it from the stage and it declares no rest
            // computation, so it contributes no input here either.
            std::vector<int> restSlots;
            for (const SdfPath &joint : joints) {
                const int slot = slotOf(joint);
                if (slot >= 0 && B.slotKind[size_t(slot)] ==
                                     RigExecBakedSlotKind::FirstFramePose) {
                    restSlots.push_back(slot);
                }
            }
            // Only the two computations that DECLARE jointElements read it
            // (computations.cpp's TwoBoneIk and SplineIk input lists); for
            // FkChain and BlendPointFrames the attribute is not an input at
            // all, so an authored one -- whatever its length -- changes
            // nothing about what exec publishes and must not be allowed to
            // make this solver degenerate.
            const bool consumesElements =
                s.type == "RigExecTwoBoneIk" || s.type == "RigExecSplineIk";
            const bool remapped = consumesElements && !elements.empty();
            if (remapped && elements.size() != restSlots.size()) {
                // "joint/rest element cardinality mismatch": the computation
                // warns and returns an empty result.
                s.degenerate = true;
            }
            const bool useRemap = remapped && !s.degenerate;
            for (size_t k = 0; k < restSlots.size(); ++k) {
                restRefs.emplace_back(restSlots[k],
                                      useRemap ? elements[k] : int(k));
            }
            s.restRefs = restRefs;
            // The LIVE half (spec 4.2): a rest ref whose joint a pose step
            // BELOW this solver already wrote is measured from the frame that
            // step left, not from the authored rest. This is the baked mirror
            // of the dynamic path computeRestFrame override, and it is bound
            // to a version by bindSolverReads rather than captured here,
            // because the frame is not known until the run reaches this
            // place in the walk.
            std::set<int> liveSlots;
            for (const SdfPath &joint : liveRestJoints) {
                const int slot = slotOf(joint);
                if (slot >= 0) liveSlots.insert(slot);
            }
            s.restIsLive.assign(restRefs.size(), false);
            for (size_t k = 0; k < restRefs.size(); ++k) {
                if (liveSlots.count(restRefs[k].first)) {
                    s.restIsLive[k] = true;
                    s.hasLiveRest = true;
                }
            }
            s.restReads.assign(restRefs.size(), 0u);
            // The output basis, captured like every other rest description
            // and rebuilt by RefreshSolverRests. Folded into restSlots so a
            // ladder recompute -- or a drag on one of these rests -- dirties
            // this solver exactly as a control rest does.
            s.jointRests.reserve(restRefs.size());
            for (const auto &[slot, element] : restRefs) {
                s.jointRests.push_back(B.restPts[foldRest(slot)]);
            }
        }
        for (size_t k = 0; k < jointOutputs.size(); ++k) {
            const auto &[joint, element] = jointOutputs[k];
            const int slot = slotOf(joint);
            if (slot < 0) {
                // Unreachable: these joints come from the compiler's joint
                // binding, so they are in _jointPaths, and IsBakeable
                // refuses a rig whose joint is not a seeded pose provider.
                // Skipped rather than refused because exec's answer to it is
                // simply that no element is ever extracted for the joint.
                continue;
            }
            s.outputs.emplace_back(slot, element);
            s.outputSnapshots.push_back(
                k < snapshotJoints.size() ? snapshotJoints[k] : 0);
        }

        if (s.type == "RigExecFkChain") {
            s.parentRelative =
                readToken(prim, "rigExec:controlSpace", "") ==
                "parentRelative";
            // The chain is the FILTERED list. The computation reads its
            // controls through a read iterator over the relationship's
            // targeted objects, so a target that publishes no frame
            // contributes no input at all: the chain SHORTENS and every
            // later element renumbers, which is what `int(k) - 1` then
            // means by a parent. The outputs mapping is not filtered -- its
            // element indices are the compiler's, over the same shortened
            // aggregate.
            for (const SdfPath &t : targets(prim, "rigExec:controls")) {
                const int slot = providerSlot(t);
                if (slot < 0) {
                    continue;
                }
                s.controls.push_back(slot);
                s.controlRests.push_back(B.restPts[foldRest(slot)]);
            }
            // rigExec:startFrame: the dynamic computation prepends the
            // provider's rest-to-pose delta as a synthetic element 0 and
            // drops its frame from the aggregate. The bake records the
            // slot and rest here (foldRest registers the refresh/dirty
            // channels like any other rest read); the Solve arm replays
            // the synthetic shape. An unwired target, a second target,
            // or a non-provider target is the computation's answer, not
            // a degenerate: no frame input, so the chain solves absolute.
            const auto startTargets = targets(prim, "rigExec:startFrame");
            if (!startTargets.empty()) {
                const int slot = providerSlot(startTargets[0]);
                if (slot >= 0) {
                    s.start = slot;
                    s.startRest = B.restPts[foldRest(slot)];
                }
            }
        } else if (s.type == "RigExecTwoBoneIk") {
            const auto r = targets(prim, "rigExec:rootControl");
            const auto e = targets(prim, "rigExec:effectorControl");
            const auto p = targets(prim, "rigExec:poleControl");
            s.root = r.empty() ? -1 : providerSlot(r[0]);
            s.end = e.empty() ? -1 : providerSlot(e[0]);
            s.pole = p.empty() ? -1 : providerSlot(p[0]);
            if (s.root < 0 || s.end < 0 || s.pole < 0) {
                // All three frames are REQUIRED inputs: an unwired or
                // non-publishing control is a null pointer and an empty
                // aggregate, not a rig the program cannot express.
                s.degenerate = true;
            }
            // Bone lengths are MEASURED from the rests of the joints this
            // solver binds, so all three chain slots have to be bound by
            // one. The computation returns empty on the first element out
            // of [0, 3) and again when the three are not all seen; the two
            // give the same aggregate, so one flag answers both.
            std::array<bool, 3> seen{false, false, false};
            for (const auto &[slot, element] : restRefs) {
                if (element < 0 || element >= 3) {
                    s.degenerate = true;
                    continue;
                }
                s.ikRests[size_t(element)] = B.restPts[foldRest(slot)];
                seen[size_t(element)] = true;
            }
            if (!(seen[0] && seen[1] && seen[2])) {
                s.degenerate = true;
            }
            // The bone lengths exec measures every evaluation, measured once.
            s.upperLengthBase = (s.ikRests[1][0] - s.ikRests[0][0]).GetLength();
            s.lowerLengthBase = (s.ikRests[2][0] - s.ikRests[1][0]).GetLength();
            s.ikSpace = bind(prim, "rigExec:spaceMatrix", GfMatrix4d(1.0));
            const auto sp = targets(prim, "rigExec:space");
            s.spaceSlot = sp.empty() ? -1 : providerSlot(sp[0]);
            if (s.spaceSlot >= 0) {
                s.spaceRest = B.restPts[foldRest(s.spaceSlot)];
            }
            s.bend = bind(prim, "rigExec:preferredBendRadians", 0.0);
            s.upperOffset = bind(prim, "rigExec:upperLengthOffset", 0.0);
            s.lowerOffset = bind(prim, "rigExec:lowerLengthOffset", 0.0);
            s.stretch = bind(prim, "inputs:stretch", 1.0f);
            s.softness = bind(prim, "inputs:softness", 0.0f);
            s.pin=bind(prim,"inputs:pin",0.0f);s.softDistance=bind(prim,"inputs:softDistance",0.0f);
            s.limbTwist=bind(prim,"inputs:twist",0.0f);
            s.upperScale=bind(prim,"inputs:upperScale",1.0);s.lowerScale=bind(prim,"inputs:lowerScale",1.0);
            s.ikParams.softDistancePolicy=readToken(prim,"rigExec:stretchPolicy","")=="softDistance";
            s.ikParams.scaleSegments=readToken(prim,"rigExec:segmentScale","")=="toChild";
            fold(prim,"rigExec:scaleCalibration");
            readFact(solverPath.AppendProperty(TfToken("rigExec:scaleCalibration")),UsdTimeCode::Default(),&s.ikParams.limb.scaleCalibration);
            s.ikParams.preferredBendRadians = s.bend.constant;
            s.ikParams.stretch = s.stretch.constant;
            s.ikParams.softness = s.softness.constant;
            RigExecTwoBoneIkLengths(s.ikRests, s.ikSpace.constant,
                                  s.upperOffset.constant,
                                  s.lowerOffset.constant,
                                  &s.ikParams.upperLength,
                                  &s.ikParams.lowerLength);
        } else if (s.type == "RigExecBlendPointFrames") {
            // Aggregate inputs bind after every solver descriptor exists.
            // Missing inputs retain the kernel's null-input behavior.
            auto solverSlot = [&](const SdfPathVector &v) {
                if (v.empty()) return -1;
                const auto it = B.solverIndex.find(v[0]);
                if (it != B.solverIndex.end()) return it->second;
                return -1;
            };
            s.inA = solverSlot(targets(prim, "rigExec:inputA"));
            s.inB = solverSlot(targets(prim, "rigExec:inputB"));
            s.blendWeight = bind(prim, "inputs:weight", 0.0f);
            s.scaleMode =
                readToken(prim, "rigExec:scaleBlend", "") == "linear"
                    ? RigExecScaleBlend::Linear
                    : RigExecScaleBlend::Log;
            // The fallback is the only mode there is, so an unauthored
            // token is never rejected; anything else warns and publishes an
            // empty array -- but only once the computation has TWO inputs in
            // hand. A null input returns the other before the token is ever
            // read, so this cannot be `degenerate`, which would answer the
            // half-wired blend with an empty aggregate the computation never
            // publishes.
            if (readToken(prim, "rigExec:rotationBlend", "shortestArc") !=
                "shortestArc") {
                s.blendRotationRejected = true;
            }
        } else if (s.type == "RigExecSplineIk") {
            const auto r = targets(prim, "rigExec:rootControl");
            const auto m = targets(prim, "rigExec:midControl");
            const auto e = targets(prim, "rigExec:endControl");
            s.root = r.empty() ? -1 : providerSlot(r[0]);
            s.mid = m.empty() ? -1 : providerSlot(m[0]);
            s.end = e.empty() ? -1 : providerSlot(e[0]);
            if (s.root < 0 || s.mid < 0 || s.end < 0) {
                // Both halves of the computation's answer at once: the three
                // posed frames are REQUIRED inputs, and it refuses again
                // when one of the three rest frames is missing. Either way
                // it publishes an empty aggregate.
                s.degenerate = true;
            }
            const size_t count = restRefs.size();
            if (count == 0) {
                // "rigExec:joints binds no joints; there is no chain to
                // measure rest CVs from". The empty rest description would
                // have solved to an empty aggregate anyway; saying so is
                // what keeps the two paths' reasons the same shape.
                s.degenerate = true;
            }
            s.splineCount = count;
            std::vector<RigExecPointFrame> restJoints(count);
            s.splineJointRests.resize(count);
            // The remap must be a PERMUTATION of the chain slots: a slot
            // filled twice leaves another empty, and the computation refuses
            // to build a rest curve out of that ("chain slot %d is filled
            // twice") rather than solving against a hole.
            std::vector<bool> filled(count, false);
            for (const auto &[slot, element] : restRefs) {
                if (element < 0 || size_t(element) >= count) {
                    s.degenerate = true;
                    continue;
                }
                if (filled[size_t(element)]) {
                    s.degenerate = true;
                    continue;
                }
                filled[size_t(element)] = true;
                restJoints[size_t(element)] = B.restFrames[foldRest(slot)];
                s.splineJointRests[size_t(element)] = B.restPts[slot];
            }
            std::vector<double> weights;
            VtFloatArray authoredWeights;
            fold(prim, "rigExec:volumeWeights");
            readFact(solverPath.AppendProperty(TfToken("rigExec:volumeWeights")),UsdTimeCode::Default(),&authoredWeights);
            s.execSplineWeights = authoredWeights;
            for (float w : authoredWeights) weights.push_back(w);
            if (!weights.empty() && weights.size() != count) {
                s.degenerate = true;
            }
            const TfToken restTok = readToken(prim, "rigExec:restLength", "");
            if (!restTok.IsEmpty() && restTok != "curve" &&
                restTok != "chain") {
                s.degenerate = true;
            }
            // Exec rebuilds this description every evaluation. It is a pure
            // function of the rests plus these two, so the two are kept and
            // the rebuild happens again only on a run that recomposed them.
            s.splineRestWeights = weights;
            s.splineRestMode = restTok == "chain"
                                   ? RigExecSplineIkRestLength::Chain
                                   : RigExecSplineIkRestLength::Curve;
            const auto splineSpaceTargets = targets(prim, "rigExec:space");
            s.spaceSlot = splineSpaceTargets.empty()
                              ? -1
                              : providerSlot(splineSpaceTargets[0]);
            if (s.spaceSlot >= 0) {
                s.spaceRest = B.restPts[foldRest(s.spaceSlot)];
            }
            s.splineRestFrames = restJoints;
            s.splineRootRest = s.root >= 0 ? B.restFrames[foldRest(s.root)]
                                           : RigExecPointFrame();
            s.splineMidRest = s.mid >= 0 ? B.restFrames[foldRest(s.mid)]
                                         : RigExecPointFrame();
            s.splineEndRest = s.end >= 0 ? B.restFrames[foldRest(s.end)]
                                         : RigExecPointFrame();
            s.splineRest = RigExecSplineIkMakeRest(
                restJoints,
                s.root >= 0 ? B.restFrames[foldRest(s.root)]
                            : RigExecPointFrame(),
                s.mid >= 0 ? B.restFrames[foldRest(s.mid)]
                           : RigExecPointFrame(),
                s.end >= 0 ? B.restFrames[foldRest(s.end)]
                           : RigExecPointFrame(),
                weights,
                restTok == "chain" ? RigExecSplineIkRestLength::Chain
                                   : RigExecSplineIkRestLength::Curve);
            s.preserveVolume = bind(prim, "inputs:preserveVolume", 1.0);
            s.midFollowWeight = bind(prim, "inputs:midFollowWeight", 0.5);
            s.roll = bind(prim, "inputs:roll", 0.0);
            s.twist = bind(prim, "inputs:twist", 0.0);
            s.minLengthRatio = bind(prim, "inputs:minLengthRatio", 0.0);
            s.splineParamsVary =
                s.preserveVolume.varying || s.midFollowWeight.varying ||
                s.roll.varying || s.twist.varying || s.minLengthRatio.varying;
            s.splineParams.preserveVolume = s.preserveVolume.constant;
            s.splineParams.midFollowWeight = s.midFollowWeight.constant;
            s.splineParams.roll = GfDegreesToRadians(s.roll.constant);
            s.splineParams.twist = GfDegreesToRadians(s.twist.constant);
            s.splineParams.minLengthRatio = s.minLengthRatio.constant;
            const TfToken tangent = readToken(prim, "rigExec:rootTangent", "");
            if (tangent == "aim") {
                s.splineParams.aimRootTangent = true;
            } else if (!tangent.IsEmpty() && tangent != "rigid") {
                s.degenerate = true;
            }
        } else if (s.type == "RigExecTwistDistribution") {
            // The two endpoint frames arrive on the batch request as FINAL
            // frames, the same way an FkChain's controls do, so they are
            // provider slots read out of `fin`.
            // A target that is no provider slot, or one the exec network
            // seeds from the stage rather than computing (an xform-derived
            // slot declares neither computePointFrame nor computeRestFrame),
            // leaves the computation's REQUIRED start or end input unbound:
            // it publishes an empty aggregate and every joint it names falls
            // back. That is `degenerate`, not a refusal.
            const auto endpoint = [&](const SdfPathVector &v) {
                return v.empty() ? -1 : providerSlot(v[0]);
            };
            s.root = endpoint(targets(prim, "rigExec:start"));
            s.end = endpoint(targets(prim, "rigExec:end"));
            if (s.root < 0 || s.end < 0) {
                s.degenerate = true;
            } else {
                // The rests are OPTIONAL inputs of the computation, but
                // every seeded provider publishes computeRestFrame, so a
                // resolved endpoint always has one and the identity
                // substitution exec makes for a missing one is unreachable
                // from here.
                s.twistStartRest = B.restPts[size_t(foldRest(s.root))];
                s.twistEndRest = B.restPts[size_t(foldRest(s.end))];
            }
            // rigExec:weights and rigExec:count define the frame cardinality,
            // which compile refuses to let vary with time, so both are read
            // once and folded. The float -> double widening is element-wise
            // and in array order, as the computation's read iterator does it.
            VtFloatArray authoredWeights;
            fold(prim, "rigExec:weights");
            readFact(solverPath.AppendProperty(TfToken("rigExec:weights")),UsdTimeCode::Default(),&authoredWeights);
            s.execTwistWeights = authoredWeights;
            for (float w : authoredWeights) s.twistWeights.push_back(w);
            int count = 1;
            fold(prim, "rigExec:count");
            readFact(solverPath.AppendProperty(TfToken("rigExec:count")),UsdTimeCode::Default(),&count);
            s.execTwistCount = count;
            RigExecResolveTwistWeights(count, &s.twistWeights);
            s.twistTurns = bind(prim, "inputs:twistTurns", 0.0);
        } else if (s.type == "RigExecRibbon") {
            // The driver curve's points, resolved by the compiler to the
            // exact native attribute (a prim target becomes its .points).
            // The dynamic path reads that attribute with UsdAttribute::Get
            // and hands exec the two values as packet overrides, so it
            // honours neither a connection on it nor the generation's
            // resolved inputs -- which is why this is NOT bind(): the
            // connection walk bind() performs would answer differently on a
            // connected points attribute.
            const auto found = ctx->ribbonDriverPoints.find(solverPath);
            if (found != ctx->ribbonDriverPoints.end()) {
                s.ribbonPointsPath = found->second;
            }
            std::string phaseError;
            if (!RigExecResolveReadPhase(prim.GetRelationship(TfToken("rigExec:driverCurve")),
                                         &s.ribbonPointsPhase,&phaseError))
                B.crossDomainErrors.push_back(phaseError);
            if (const UsdAttribute a =
                    B.stage->GetAttributeAtPath(s.ribbonPointsPath)) {
                // Read at Default, and left empty when nothing answers
                // there: a curve carrying only time samples has no bind-time
                // value, and an empty rest is what makes the sampler publish
                // nothing at all.
                VtVec3fArray rest;
                readFact(s.ribbonPointsPath,UsdTimeCode::Default(),&rest);
                s.ribbonRestPoints.assign(rest.begin(), rest.end());
                s.ribbonPointsVarying = a.ValueMightBeTimeVarying() ||
                                        a.GetNumTimeSamples() > 0;
                if (s.ribbonPointsVarying) {
                    s.ribbonPointsQuery = UsdAttributeQuery(a);
                } else {
                    // One value at every time code, so the prologue has
                    // nothing to read and the cone nothing to compare.
                    VtVec3fArray live;
                    readFact(s.ribbonPointsPath,ctx->capture,&live);
                    s.ribbonConstantPoints.assign(live.begin(), live.end());
                }
                // Registered BY PATH rather than through fold(prim, name):
                // the driver curve is another prim entirely, and the target
                // may be an arbitrary property path. The rest capture makes
                // it folded -- a value edit on the curve rebuilds, and an
                // override on it cannot be placed, which is right because
                // the dynamic path would ignore that override.
                // Inside the read, and deliberately so: a resolved path
                // that names nothing on this stage was never read, and the
                // day an attribute appears there it is the epoch digest
                // that sees it -- creating a property is a structural edit,
                // so the epoch recompiles and this program is rebuilt with
                // it before the next generation chooses a path. Registering
                // a path the bake could not read would claim an
                // invalidation this index does not owe. The fixture "the
                // driver curve's points created later" is where that is
                // measured.
                B.rebuild.insert(s.ribbonPointsPath);
                B.folded.insert(s.ribbonPointsPath);
                B.named.insert(s.ribbonPointsPath);
                B.prims.insert(s.ribbonPointsPath.GetPrimPath());
            }
            // bind(), not fold(): a sampleCount edit is a value edit exec
            // answers per frame, and the epoch digest already forces a
            // recompile when the count changes the frame cardinality.
            s.ribbonSampleCount = bind(prim, "rigExec:sampleCount", 5);
            // rigExec:parameterization, frameTransport, startFrame,
            // endFrame and twistFrames are
            // deliberately NOT read: the computation does not read them
            // either -- they shape batching and the epoch digest -- and
            // folding one would rebuild the program for an edit that cannot
            // change what it publishes.
        }
        // What can move this description, asked once. `restsVary` is the
        // whole rest CHAIN of every slot it measured from -- a provider's
        // rest is its ancestors' rests as well -- and `restOverrides` is
        // every rest channel of that same chain, so that a drag on one
        // dirties this solver rather than leaving it solving against the
        // authored rest the drag is standing in for.
        {
            // A LIVE rest is by definition per-frame: it is this frame pose as
            // an earlier step left it, so the description can never be
            // concluded constant.
            s.restsVary = s.restsVary || s.hasLiveRest;
            std::set<int> chain;
            for (const int named : s.restSlots) {
                for (int slot = named; slot >= 0;
                     slot = B.parent[size_t(slot)]) {
                    if (!chain.insert(slot).second) {
                        break;
                    }
                }
                s.restsVary =
                    s.restsVary || B.restChainVaries[size_t(named)] != 0;
            }
            for (const int slot : chain) {
                const RigExecBakedProgramImpl::Ladder &ladder =
                    B.ladders[size_t(slot)];
                const RigExecBakedInput<double> *avars = ladder.restAvars;
                if (ladder.restSpace.overrideIndex >= 0) {
                    s.restOverrides.push_back(ladder.restSpace.overrideIndex);
                }
                for (int c = 0; c < 6; ++c) {
                    if (avars[c].overrideIndex >= 0) {
                        s.restOverrides.push_back(avars[c].overrideIndex);
                    }
                }
            }
            std::sort(s.restOverrides.begin(), s.restOverrides.end());
            s.restOverrides.erase(
                std::unique(s.restOverrides.begin(), s.restOverrides.end()),
                s.restOverrides.end());
        }
        RigExecSceneSolverDescriptor descriptor;
        std::string lowerError;
        if(!RigExecLowerSceneSolver(*B.sceneDescriptors,solverPath,providerRests,&descriptor,&lowerError))
            refuse(lowerError,solverPath);
        else {
            // Native input binding selects live/rest SSA versions; the common
            // structural record owns the solver's schema interpretation.
            auto &record=descriptor.record;
            record.degenerate=record.degenerate||s.degenerate;
            record.restIsLive=std::move(s.restIsLive);
            record.jointRests=std::move(s.jointRests);
            static_cast<RigExecSolverRecord &>(s)=std::move(record);
        }
        s.kernelInputs.controls.resize(s.controls.size());
        s.kernelInputs.ribbonPoints.reserve(std::max(s.ribbonPoints.size(),s.ribbonConstantPoints.size()));
        s.kernelWorkspace.fkElements.resize(s.controls.size()+size_t(s.start>=0));
        s.kernelWorkspace.spacedFrames.reserve(s.splineRestFrames.size());
        const int slot = int(B.solvers.size());
        B.solverIndex[solverPath] = slot;
        B.solvers.push_back(std::move(s));
        return slot;
    };

    // One plain Xformable a constraint reads off the stage, registered once
    // however many bindings name it.
    // The ancestor list is pure namespace topology -- every slot that is a
    // STRICT prefix of the path, shallowest first -- so it is settled here;
    // WHICH of them the source rides is a per-frame question about frames
    // the step reads out of declared slots. The prim and its ancestors go
    // into the resync index only: the transform is re-read every frame, so
    // a value edit on it must not rebuild the program.
    auto nativeSource = [&](const SdfPath &path) {
        for (size_t k = 0; k < B.nativeSources.size(); ++k) {
            if (B.nativeSources[k].path == path) {
                return int(k);
            }
        }
        RigExecBakedProgramImpl::NativeXformSource source;
        source.path = path;
        for (int i = 0; i < N; ++i) {
            if (B.paths[size_t(i)] != path &&
                path.HasPrefix(B.paths[size_t(i)])) {
                source.ancestorSlots.push_back(i);
            }
        }
        for (SdfPath p = path;
             !p.IsEmpty() && p != SdfPath::AbsoluteRootPath();
             p = p.GetParentPath()) {
            B.prims.insert(p);
            if (p == B.assetRootPath) {
                break;
            }
        }
        B.nativeSources.push_back(std::move(source));
        return int(B.nativeSources.size()) - 1;
    };

    auto bakeConstraint = [&](const RigExecBakedConstraintSpec &fc) {
        RigExecBakedProgramImpl::Constraint c;
        c.path = fc.moverPath;
        c.pathText = fc.moverPath.GetString();
        c.type = fc.schemaType;
        c.singleChainIk = fc.schemaType == "RigExecSingleChainIkConstraint";
        const UsdPrim prim = B.stage->GetPrimAtPath(fc.moverPath);
        RigExecSceneConstraintDescriptor sceneConstraint;
        bool loweredConstraint=false;
        const auto *constraintNode=compileInputs.Node(fc.moverPath);
        // Pose MatrixMover has its own matrix record and numerical body;
        // the shared constraint descriptor covers the six constraint schemas.
        if(fc.schemaType!="RigExecMatrixMover" && constraintNode &&
           constraintNode->fact.type==fc.schemaType) {
            std::string error;
            loweredConstraint=RigExecLowerSceneConstraint(*B.sceneDescriptors,fc.moverPath,
                fc.targets,&sceneConstraint,&error);
            if(!loweredConstraint) refuse(error,fc.moverPath);
            else c.kernelRecord=sceneConstraint.record;
        }
        // Every target, in compiled order, with the read-phase membership
        // the dynamic recordFrame tests per call. A SingleChainIK's targets
        // ARE its joint chain (rigEvaluator's compile sets the two equal),
        // so this is also the chain the commit revises atomically.
        for (size_t k = 0; k < fc.targets.size(); ++k) {
            const int slot = slotOf(fc.targets[k]);
            if (slot < 0) {
                refuse("constraint target is not a seeded pose provider",
                       fc.targets[k]);
                continue;
            }
            c.targetSlots.push_back(slot);
            c.snapshotTargets.push_back(
                k < fc.snapshotTargets.size() ? fc.snapshotTargets[k] : 0);
        }
        c.target = c.targetSlots.empty() ? -1 : c.targetSlots[0];
        for (size_t k = 0; k < fc.sources.size(); ++k) {
            // resolveBinding's order, decided once: the walk's own frame
            // when it holds one -- which is every RigExec provider and every
            // plain Xformable some constraint targets -- and the stage
            // otherwise. A source that is itself an xform-derived target must
            // take the SLOT, or a constraint chained onto another
            // constraint's target would read the unrevised transform.
            const int slot = slotOf(fc.sources[k]);
            const SdfPath &xform =
                k < fc.sourceXforms.size() ? fc.sourceXforms[k] : SdfPath();
            const int native =
                slot < 0 && !xform.IsEmpty() ? nativeSource(xform) : -1;
            if (slot < 0 && native < 0) {
                refuse("constraint source is not a pose provider",
                       fc.sources[k]);
            }
            c.sources.push_back(slot);
            c.sourceNatives.push_back(native);
            c.sourcePaths.push_back(fc.sources[k]);
            c.sourcePathTexts.push_back(fc.sources[k].GetString());
        }
        c.enabled = bind(prim, "inputs:enabled", true);
        c.defaultWeight = bind(prim, "inputs:defaultWeight", 1.0f);
        // The envelope object, resolved per frame by the oracle and not
        // captured here: what the bake records is only that the constraint
        // HAS one, and enough of the invalidation index to notice the object
        // changing. Every read the oracle makes goes through the
        // generation's resolved inputs, so the prim is routed rather than
        // bound -- an interactive override on any property of it reaches the
        // next generation with nothing to place.
        c.weightObject = fc.weightObject;
        c.weightScratch.reserve(1);
        if (!c.weightObject.IsEmpty()) {
            B.prims.insert(c.weightObject);
            B.resolvedRoutedPrims.insert(c.weightObject);
        }

        // inputs:sourceWeights and the parent offsets are authored TABLES
        // the operator reads raw at the frame's own time. They are epoch
        // state rather than captured: the prologue reads them again when
        // the program stamp or the stage edit serial moved, or when the time
        // moved and their read can move with it (ConstraintArrays), so an
        // animated blend between two parents bakes -- and the cardinality
        // diagnostic the dynamic walk gives a malformed one is reproduced
        // from the read's numbers rather than refused at bake.
        // FoldShape and not Fold: nothing about the VALUE is folded, so an
        // interactive override on one of these tables is not placeable --
        // which agrees with the dynamic path, whose raw read ignores such an
        // override too. What FoldShape does NOT buy is edit-time cheapness:
        // like Fold it puts the property in `rebuild`, so authoring a new
        // weight rebuilds the program even though the run would have re-read
        // it anyway. Conservative and correct; the day that costs a rigger
        // scrubbing a blend curve, the fix is a `Name` that indexes without
        // rebuilding, not a quieter comment here.
        {
            RigExecBakedProgramImpl::ConstraintArrays arrays;
            arrays.prim = prim;
            arrays.path=fc.moverPath;
            arrays.pathText=c.pathText;
            const auto &names =
                RigExecBakedProgramImpl::ConstraintArrays::Names();
            for (size_t channel = 0; prim && channel < 4; ++channel) {
                arrays.attributes[channel] = prim.GetAttribute(names[channel]);
                arrays.keys[channel] =
                    prim.GetPath().AppendProperty(names[channel]);
            }
            arrays.sourceCount = c.sources.size();
            // An unconsumed table has the same neutral shape as a missing
            // authored array. Admitted bodies still validate and replace it
            // from the current raw samples before using any entry.
            arrays.weights.assign(arrays.sourceCount, 1.0);
            arrays.translationOffsets.assign(arrays.sourceCount, GfVec3d(0));
            arrays.rotationOffsets.assign(arrays.sourceCount, GfVec3d(0));
            arrays.parentOffsets =
                fc.schemaType == "RigExecParentConstraint";
            foldShape(prim, "inputs:sourceWeights");
            if (prim.GetAttribute(TfToken("inputs:sourceWeights"))) {
                ++B.boundInputs;
            }
            if (arrays.parentOffsets) {
                for (const char *name : {"inputs:translationOffsets",
                                         "inputs:rotationOffsets"}) {
                    foldShape(prim, name);
                    if (prim.GetAttribute(TfToken(name))) {
                        ++B.boundInputs;
                    }
                }
            }
            c.arrays = int(B.constraintArrays.size());
            B.constraintArrays.push_back(std::move(arrays));
        }
        // The GEOMETRY domain, decided once. The delta is measured against
        // the TARGET PRIM's authored transform, which is a stage read the
        // prologue makes; the prim goes into the resync index only, because
        // nothing about its value is captured.
        c.pointsTarget = fc.pointsTarget;
        if (!fc.pointsTarget.IsEmpty() && !fc.targets.empty()) {
            c.deltaBasePath = fc.targets[0];
            c.deltaBasePathText = fc.targets[0].GetString();
            c.deltaBase = int(B.deltaBasePaths.size());
            B.deltaBasePaths.push_back(fc.targets[0]);
            B.prims.insert(fc.targets[0]);
        }
        // Three uniform tokens, read the way the dynamic walk reads them
        // (plain Get, no time), plus the two bindings it resolves beside
        // them. The evaluation mode's "autoDetect" is settled here because
        // _IkUsesAnimatedTs asks only structural questions -- a solver
        // binding, a time sample, a connection -- and the attributes it
        // inspects are folded for their SHAPE so that authoring one rebuilds
        // the program rather than silently changing the answer.
        if (c.singleChainIk) {
            c.stretch=bind(prim,"inputs:stretch",0.0f);
            c.preserveJointOrientation =
                readToken(prim, "rigExec:orientationMode", "aimX") == "preserve";
            c.ikMode = readToken(prim, "rigExec:solverMode", "rotatePlane") ==
                               "singleChain"
                           ? RigExecSingleChainIkMode::SingleChain
                           : RigExecSingleChainIkMode::RotatePlane;
            c.poleModeObject =
                readToken(prim, "rigExec:poleVectorMode", "vector") ==
                "object";
            const TfToken evaluationMode =
                readToken(prim, "rigExec:evaluationMode", "neverTS");
            c.useAnimatedTs =
                evaluationMode == "alwaysTS" ||
                (evaluationMode == "autoDetect" &&
                 fc.ikUsesAnimatedTs);
            c.ikRestLive = fc.ikRestLive;
            for (const SdfPath &joint : fc.ikChain) {
                const UsdPrim jointPrim = B.stage->GetPrimAtPath(joint);
                for (const char *name : {"posed:space", "avars:tx",
                                         "avars:ty", "avars:tz", "avars:sx",
                                         "avars:sy", "avars:sz"}) {
                    foldShape(jointPrim, name);
                }
            }
            const auto bindSource = [&](const SdfPath &path,
                                        const SdfPath &xform, int *slot,
                                        int *native) {
                *slot = path.IsEmpty() ? -1 : slotOf(path);
                *native = *slot < 0 && !xform.IsEmpty() ? nativeSource(xform)
                                                        : -1;
            };
            bindSource(fc.effector, fc.effectorXform, &c.effector,
                       &c.effectorNative);
            c.effectorPath = fc.effector;
            for (size_t k = 0; k < fc.poleObjects.size(); ++k) {
                int slot = -1, native = -1;
                bindSource(fc.poleObjects[k],
                           k < fc.poleObjectXforms.size()
                               ? fc.poleObjectXforms[k]
                               : SdfPath(),
                           &slot, &native);
                c.poleObjects.push_back(slot);
                c.poleObjectNatives.push_back(native);
            }
            // Bound only in RotatePlane mode, which is the only mode the
            // dynamic walk reads them in: binding them everywhere would let
            // an override on one place itself on a solve that ignores it.
            if (c.ikMode == RigExecSingleChainIkMode::RotatePlane) {
                c.poleVector =
                    bind(prim, "inputs:poleVector", GfVec3d(0, 1, 0));
                c.twistDegrees = bind(prim, "inputs:twistDegrees", 0.0);
            }
            RigExecBakedProgramImpl::ConstraintArrays &arrays =
                B.constraintArrays[size_t(c.arrays)];
            arrays.readPole =
            c.ikMode == RigExecSingleChainIkMode::RotatePlane &&
                c.poleModeObject && !c.poleObjects.empty();
            arrays.poleCount = c.poleObjects.size();
            arrays.poleWeights.assign(arrays.poleCount, 1.0);
            if (arrays.readPole &&
                prim.GetAttribute(TfToken("inputs:poleVectorWeights"))) {
                foldShape(prim, "inputs:poleVectorWeights");
                ++B.boundInputs;
            }
        }
        c.order = RigExecBakedParseEulerOrder(
            readToken(prim, "rigExec:rotationOrder", "XYZ"));

        if (fc.schemaType == "RigExecPositionConstraint") {
            c.affectX = bind(prim, "inputs:affectTranslationX", true);
            c.affectY = bind(prim, "inputs:affectTranslationY", true);
            c.affectZ = bind(prim, "inputs:affectTranslationZ", true);
            c.offset = ctx->Bind(prim, "inputs:translationOffset", GfVec3d(0),
                                 /*sourceValue=*/true);
        } else if (fc.schemaType == "RigExecRotationConstraint") {
            c.affectX = bind(prim, "inputs:affectRotationX", true);
            c.affectY = bind(prim, "inputs:affectRotationY", true);
            c.affectZ = bind(prim, "inputs:affectRotationZ", true);
            c.offset = bind(prim, "inputs:rotationOffset", GfVec3d(0));
        } else if (fc.schemaType == "RigExecScaleConstraint") {
            c.affectX = bind(prim, "inputs:affectScaleX", true);
            c.affectY = bind(prim, "inputs:affectScaleY", true);
            c.affectZ = bind(prim, "inputs:affectScaleZ", true);
            c.offset = bind(prim, "inputs:scaleOffset", GfVec3d(0));
        } else if (fc.schemaType == "RigExecParentConstraint") {
            c.tX = bind(prim, "inputs:affectTranslationX", true);
            c.tY = bind(prim, "inputs:affectTranslationY", true);
            c.tZ = bind(prim, "inputs:affectTranslationZ", true);
            c.rX = bind(prim, "inputs:affectRotationX", true);
            c.rY = bind(prim, "inputs:affectRotationY", true);
            c.rZ = bind(prim, "inputs:affectRotationZ", true);
            // FBX disables scale by default; the false fallback is the
            // authored contract (schema.usda), not an oversight.
            c.sX = bind(prim, "inputs:affectScaleX", false);
            c.sY = bind(prim, "inputs:affectScaleY", false);
            c.sZ = bind(prim, "inputs:affectScaleZ", false);
        } else if (fc.schemaType == "RigExecAimConstraint") {
            c.affectX = bind(prim, "inputs:affectRotationX", true);
            c.affectY = bind(prim, "inputs:affectRotationY", true);
            c.affectZ = bind(prim, "inputs:affectRotationZ", true);
            c.aimVector = bind(prim, "inputs:aimVector", GfVec3d(1, 0, 0));
            const UsdAttribute aimAttr =
                prim.GetAttribute(TfToken("inputs:aimVector"));
            // Whether it is authored, not what it holds: the value stays a
            // per-frame read (and so an override can stand on it), but the
            // choice between aimVector and the legacy aimAxis is baked.
            foldShape(prim, "inputs:aimVector");
            c.aimVectorAuthored = aimAttr && aimAttr.HasAuthoredValueOpinion();
            // Existing assets author aimAxis but predate aimVector; keep that
            // meaning until they opt into the vector form.
            const TfToken axis = readToken(prim, "rigExec:aimAxis", "x");
            c.aimAxisFallback = axis == "y" ? GfVec3d(0, 1, 0)
                              : axis == "z" ? GfVec3d(0, 0, 1)
                                            : GfVec3d(1, 0, 0);
            c.upVector = bind(prim, "inputs:upVector", GfVec3d(0, 1, 0));
            c.rotationOffset = bind(prim, "inputs:rotationOffset", GfVec3d(0));
            c.worldUpVector = bind(prim, "inputs:worldUpVector",
                                   GfVec3d(0, 1, 0));
            c.worldUpType = readToken(prim, "rigExec:worldUpType", "none");
            const std::string up = UsdGeomGetStageUpAxis(B.stage).GetString();
            c.sceneUp = (up == "Z" || up == "z") ? GfVec3d(0, 0, 1)
                                                 : GfVec3d(0, 1, 0);
            // The legacy aimTarget/aimAxis contract preserves input up; FBX
            // WorldUpType=None is the distinct minimum-swing mode.
            c.preserveInputUp =
                targets(prim, "rigExec:sources").empty();
            c.worldUpObjectNamed = !fc.worldUpObject.IsEmpty();
            c.worldUpPath = fc.worldUpObject;
            c.worldUpObject =
                c.worldUpObjectNamed ? slotOf(fc.worldUpObject) : -1;
            if (c.worldUpObjectNamed && c.worldUpObject < 0 &&
                !fc.worldUpXform.IsEmpty()) {
                c.worldUpNative = nativeSource(fc.worldUpXform);
            }
            if (c.worldUpObjectNamed && c.worldUpObject < 0 &&
                c.worldUpNative < 0) {
                refuse("aim world-up object is not a pose provider",
                       fc.worldUpObject);
            }
        }
        // rigExec:space, on the operators the compile reads it for. The
        // compile only keeps a provider here, so a named space that has no
        // slot is a bake-time contradiction, not a per-frame one.
        c.blendShear = fc.blendShear;
        c.worldUpRotationOnly = fc.worldUpRotationOnly;
        c.radialBlend = fc.radialBlend;
        if (!fc.space.IsEmpty()) {
            c.spaceSlot = slotOf(fc.space);
            if (c.spaceSlot < 0) {
                refuse("constraint space is not a pose provider", fc.space);
            }
        }
        if(loweredConstraint) {
            c.order=sceneConstraint.order;
            c.preserveJointOrientation=sceneConstraint.record.singleChain.preserveJointOrientation;
            c.ikMode=sceneConstraint.record.singleChain.mode;
            c.poleModeObject=sceneConstraint.poleModeObject;
            c.useAnimatedTs=sceneConstraint.useAnimatedTs;
            c.aimVectorAuthored=sceneConstraint.aimVectorAuthored;
            c.aimAxisFallback=sceneConstraint.aimAxisFallback;
            c.worldUpType=sceneConstraint.worldUpType;
            c.sceneUp=sceneConstraint.sceneUp;
            c.preserveInputUp=sceneConstraint.preserveInputUp;
            c.blendShear=sceneConstraint.blendShear;
            c.worldUpRotationOnly=sceneConstraint.worldUpRotationOnly;
        }
        c.kernelInputs.sources.reserve(c.sources.size());
        c.kernelInputs.chain.reserve(c.targetSlots.size());
        c.kernelResult.chain.reserve(c.targetSlots.size());
        B.constraints.push_back(std::move(c));
        return int(B.constraints.size()) - 1;
    };

    std::vector<bool> ownedBySolver(N, false);
    for (int i = 0; i < N; ++i) {
        ownedBySolver[i] = B.jointSolverBinding->count(B.paths[i]) > 0;
    }
    // Every provider inherits its namespace pose: an authored or connected
    // parent:space would have refused the bake above, which is exactly the
    // condition the dynamic walk re-reads off the stage per descendant.
    auto buildPropagation = [&](const std::vector<int> &candidates,
                                std::vector<std::pair<int, int>> *out) {
        const std::set<int> candidateSet(candidates.begin(), candidates.end());
        std::set<int> covered;
        // Disjoint changed subtrees, in slot (== namespace) order, which is
        // the order the dynamic walk enumerates them in.
        std::vector<int> roots(candidateSet.begin(), candidateSet.end());
        int coveredRoot = -1;
        for (int root : roots) {
            if (coveredRoot >= 0 &&
                B.paths[root].HasPrefix(B.paths[coveredRoot])) {
                continue;
            }
            coveredRoot = root;
            for (int j = root + 1; j < N; ++j) {
                if (!B.paths[j].HasPrefix(B.paths[root])) break;
                // The dynamic walk enumerates descendants out of
                // hierarchicalProviders, which holds the exec-seeded
                // providers alone: a plain Xformable a constraint targets is
                // never propagated TO, only seeded and revised. It still
                // takes a slot, so the scan passes over it rather than
                // stopping at it.
                if (B.slotKind[size_t(j)] != RigExecBakedSlotKind::FirstFramePose) {
                    continue;
                }
                if (candidateSet.count(j) || !covered.insert(j).second) {
                    continue;
                }
                // The closest revised ancestor is a PURE namespace climb in
                // the dynamic walk, so it rides propParent and not parent: an
                // xform-derived ancestor can be the revised one.
                int closest = B.propParent[j];
                while (closest >= 0 && !candidateSet.count(closest)) {
                    closest = B.propParent[closest];
                }
                if (closest < 0) continue;
                // An independently solved joint is an absolute posed
                // override: propagation cannot pass through it.
                bool blocked = false;
                for (int p = j; p >= 0 && p != closest; p = B.propParent[p]) {
                    if (ownedBySolver[p]) { blocked = true; break; }
                }
                if (blocked) continue;
                out->emplace_back(j, closest);
            }
        }
    };

    for (const RigExecBakedWalkEntry &entry : walk) {
        RigExecBakedProgramImpl::WalkStep st;
        st.solverBatch = entry.solverBatch;
        if (entry.solverBatch) {
            st.level = entry.level;
            std::vector<int> candidates;
            for (size_t k = 0; k < entry.batchSolvers.size(); ++k) {
                const int slot =
                    bakeSolver(entry.batchSolvers[k],
                               entry.solverJoints[k],
                               k < entry.solverSnapshotJoints.size()
                                   ? entry.solverSnapshotJoints[k]
                                   : std::vector<char>(),
                               k < entry.solverLiveRestJoints.size()
                                   ? entry.solverLiveRestJoints[k]
                                   : SdfPathVector());
                st.batchSolvers.push_back(slot);
                for (const auto &[providerSlot, element] :
                         B.solvers[slot].outputs) {
                    candidates.push_back(providerSlot);
                }
            }
            buildPropagation(candidates, &st.propagate);
        } else {
            st.index = bakeConstraint(entry.constraint);
            const RigExecBakedProgramImpl::Constraint &baked =
                B.constraints[size_t(st.index)];
            // A geometry-domain constraint revises NO transform: it measures
            // a delta and hands it to a Matrix revision. It still reads its
            // target's frame as the input it solves from, which is the
            // commit's targetRead and not a candidate. A SingleChainIK
            // revises its whole chain atomically, so the propagation is
            // built over all of it at once -- buildPropagation already
            // collapses a nested chain to its root, which is what the
            // dynamic commitConstraintFrames does with a candidate set.
            if (baked.target >= 0 && baked.pointsTarget.IsEmpty()) {
                buildPropagation(baked.singleChainIk
                                     ? baked.targetSlots
                                     : std::vector<int>{baked.target},
                                 &st.propagate);
            }
        }
        B.walkSteps.push_back(std::move(st));
    }

    // The solvers no batch runs. A solver that binds no joint and that no
    // mover names on rigExec:driverFrames is not required, so the walk never
    // reaches it -- but it is still an aggregate solver, and the dynamic
    // path still publishes its frames as guides, out of a SECOND exec
    // request that genuinely computes it against the walk's FINAL provider
    // frames. Baked here, in the dependency order Build resolved, and run as
    // ordinary Solve steps after the walk.
    for (const SdfPath &solverPath : ctx->guideOnlySolvers) {
        B.guideSolvers.push_back(bakeSolver(solverPath, {}, {}, {}));
    }
    // Aggregate identities are bound after all descriptors exist; authored
    // discovery order does not decide whether an upstream solver is visible.
    for(auto &solver:B.solvers) if(solver.type=="RigExecBlendPointFrames") {
        const auto prim=B.stage->GetPrimAtPath(solver.path);
        const auto source=[&](const char *name) {
            SdfPathVector paths;
            if(const auto rel=prim.GetRelationship(TfToken(name))) rel.GetTargets(&paths);
            const auto found=paths.empty()?B.solverIndex.end():B.solverIndex.find(paths[0].GetPrimPath());
            return found==B.solverIndex.end()?-1:found->second;
        };
        solver.inA=source("rigExec:inputA"); solver.inB=source("rigExec:inputB");
    }
    // Retain only supported solver-input relationships as semantic
    // prerequisites. Generic relationships and authored stack order are not
    // imported into this list. All solver identities exist before this census.
    for (auto &solver : B.solvers) {
        for (const auto &port : RigExecSolverRelationshipPorts(solver.kind)) {
            auto targets = compileInputs.Targets(solver.path.AppendProperty(TfToken(port)));
            // Structural-only Ribbon ports retain every authored target.
            // Numerical input ports select the binder's first target; FK
            // controls retain their ordered list.
            const bool ribbonStructural = solver.kind == RigExecSolverKind::Ribbon &&
                port != "rigExec:driverCurve";
            if (port != "rigExec:controls" && !ribbonStructural && targets.size() > 1)
                targets.resize(1);
            for (const auto &target : targets) {
                const auto found = B.solverIndex.find(target.GetPrimPath());
                if (found != B.solverIndex.end())
                    solver.relationshipRequirements.emplace_back(port, found->second);
            }
        }
        std::sort(solver.relationshipRequirements.begin(), solver.relationshipRequirements.end());
        solver.relationshipRequirements.erase(std::unique(solver.relationshipRequirements.begin(),
            solver.relationshipRequirements.end()), solver.relationshipRequirements.end());
    }
    B.aggregates.resize(B.solvers.size());
}


// Lower authored pose checkpoints to typed operation reads and writes.
// The common compiler assigns execution order after all domains are present.

namespace {

/// Appends a step of \p kind about \p object and returns it.
RigExecBakedStep &
AddStep(RigExecBakedProgramImpl *program, RigExecBakedStepKind kind,
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

/// Binds every pose read and write of the walk to a VERSION of its slot
/// (§3.1).
///
/// One sweep in program order over the same writes the edge sweep sees,
/// keeping the entry that holds each slot's current version. A read is bound
/// to the entry standing when the reader runs; a write gets an entry of its
/// own, which nothing else in the program writes. Slot i's first version is
/// entry i -- the compose writes there, an xform-derived slot's seed sits
/// there -- so a rig with no commits allocates nothing and the compose body
/// needs no table at all.
///
/// The carry entries are what a write site that does not write means: it
/// copies the version standing at ITS point into its own storage, so every
/// version a later reader can name is well formed however the run went.
void
BindPoseVersions(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    const size_t n = B.paths.size();
    // How many versions each slot ends up with, counted before any is handed
    // out. The LAST version of a slot is what the matrices, the phased-read
    // records and the publication all read -- 252 matrix steps and ~850 keys
    // on a biped -- so those entries are laid out DENSELY, one per slot at
    // [n, 2n), and only the versions in between go into the arena past them.
    // Without that the frame's most-walked reads scatter over the whole
    // version table and the publication alone costs a third more.
    std::vector<uint32_t> finWrites(n, 0), baseWrites(n, 0);
    RigExecBakedPlanProviderRefreshes(&B);
    for(const auto &refresh:B.providerRefreshes) {
        ++baseWrites[size_t(refresh.slot)];++finWrites[size_t(refresh.slot)];
        for(const auto &carry:refresh.carries) {++baseWrites[size_t(carry.slot)];++finWrites[size_t(carry.slot)];}
    }
    for (const RigExecBakedCommit &commit : B.commits) {
        for (const int slot : commit.slots) {
            ++finWrites[size_t(slot)];
            if (commit.solverOutput) {
                ++baseWrites[size_t(slot)];
            }
        }
        for (const auto &[descendant, closest] : commit.propagate) {
            ++finWrites[size_t(descendant)];
            if (commit.solverOutput) {
                ++baseWrites[size_t(descendant)];
            }
        }
    }
    // Slot -> the entry holding its current version, as the sweep stands,
    // and how many of its versions are still to come.
    std::vector<uint32_t> liveFin(n), liveBase(n);
    for (size_t i = 0; i < n; ++i) {
        liveFin[i] = uint32_t(i);
        liveBase[i] = uint32_t(i);
    }
    uint32_t finNext = uint32_t(2 * n), baseNext = uint32_t(2 * n);
    // The next entry for one slot: its dense last-version entry when this is
    // the last write of it, an arena entry otherwise.
    const auto take = [n](std::vector<uint32_t> *remaining, uint32_t *next,
                          size_t slot) {
        return --(*remaining)[slot] == 0 ? uint32_t(n + slot) : (*next)++;
    };
    const auto readFin = [&liveFin, n](int slot) {
        return slot >= 0 && size_t(slot) < n ? liveFin[size_t(slot)] : 0u;
    };
    const auto readBase = [&liveBase, n](int slot) {
        return slot >= 0 && size_t(slot) < n ? liveBase[size_t(slot)] : 0u;
    };
    const auto refreshBefore=[&](size_t checkpoint) {
        for(uint32_t index:B.providerRefreshBefore[checkpoint])
            if(!RigExecBakedBindProviderRefresh(&B,index,&liveBase,&liveFin,
                [&](bool base,int slot) {return base?take(&baseWrites,&baseNext,size_t(slot)):take(&finWrites,&finNext,size_t(slot));}))return false;
        return true;
    };
    // One solver's frame reads, bound to the versions live where it runs.
    // Written once because two callers need it at two different points of
    // the sweep: a batch's solvers read the versions live where their batch
    // begins, and the guide-only solvers -- which run after the whole walk
    // -- read the last version of everything.
    const auto bindSolverReads =
        [&readFin](RigExecBakedProgramImpl::Solver &solver) {
        solver.controlReads.clear();
        for (const int control : solver.controls) {
            solver.controlReads.push_back(readFin(control));
        }
        solver.startRead = readFin(solver.start);
        solver.rootRead = readFin(solver.root);
        solver.midRead = readFin(solver.mid);
        solver.endRead = readFin(solver.end);
        solver.poleRead = readFin(solver.pole);
        solver.spaceRead = readFin(solver.spaceSlot);
        // The live joint rests, bound exactly like the control reads are: to
        // the version standing where this batch begins, which is where the
        // dynamic path reads finalFrames for the same override. That is what
        // makes the two paths bit-identical on a live rest.
        if (solver.hasLiveRest) {
            solver.restReads.assign(solver.restRefs.size(), 0u);
            for (size_t k = 0; k < solver.restRefs.size(); ++k) {
                if (k < solver.restIsLive.size() && solver.restIsLive[k]) {
                    solver.restReads[k] = readFin(solver.restRefs[k].first);
                }
            }
        }
    };

    for (size_t w = 0; w < B.walkSteps.size(); ++w) {
        if(!refreshBefore(w))return;
        const RigExecBakedProgramImpl::WalkStep &walk = B.walkSteps[w];
        RigExecBakedCommit &commit = B.commits[w];
        // The producers first: a batch's solvers and a constraint's own
        // inputs read the versions live where the batch begins, which is
        // where the commit's own reads are taken too.
        if (walk.solverBatch) {
            for (const int si : walk.batchSolvers) {
                bindSolverReads(B.solvers[size_t(si)]);
            }
        } else {
            const RigExecBakedProgramImpl::Constraint &constraint =
                B.constraints[size_t(walk.index)];
            commit.sourceReads.clear();
            for (const int source : constraint.sources) {
                commit.sourceReads.push_back(readFin(source));
            }
            commit.worldUpRead = readFin(constraint.worldUpObject);
            // The frame the constraint solves FROM, whichever domain it
            // writes. For a transform-domain constraint this is the same
            // version its one candidate reads; a geometry-domain one has no
            // candidate at all and this is its only read of the target.
            commit.targetRead = readFin(constraint.target);
            commit.targetReads.clear();
            for (const int slot : constraint.targetSlots) {
                commit.targetReads.push_back(readFin(slot));
            }
            // A native source rides the deepest ancestor the walk has moved
            // BY THIS POINT, so both halves of every candidate ancestor --
            // its base and its final -- are read at the versions live here.
            const auto ancestorReads =
                [&](int native,
                    std::vector<RigExecBakedCommit::AncestorRead> *out) {
                out->clear();
                if (native < 0) {
                    return;
                }
                for (const int slot :
                         B.nativeSources[size_t(native)].ancestorSlots) {
                    out->push_back({slot, readFin(slot), readBase(slot)});
                }
            };
            commit.sourceAncestors.resize(constraint.sources.size());
            for (size_t k = 0; k < constraint.sourceNatives.size(); ++k) {
                ancestorReads(constraint.sourceNatives[k],
                              &commit.sourceAncestors[k]);
            }
            ancestorReads(constraint.worldUpNative,
                          &commit.worldUpAncestors);
            commit.effectorRead = readFin(constraint.effector);
            ancestorReads(constraint.effectorNative,
                          &commit.effectorAncestors);
            commit.poleReads.clear();
            commit.poleAncestors.assign(constraint.poleObjects.size(), {});
            for (size_t k = 0; k < constraint.poleObjects.size(); ++k) {
                commit.poleReads.push_back(
                    readFin(constraint.poleObjects[k]));
                ancestorReads(constraint.poleObjectNatives[k],
                              &commit.poleAncestors[k]);
            }
        }
        commit.slotReads.clear();
        for (const int slot : commit.slots) {
            commit.slotReads.push_back(readFin(slot));
        }
        commit.descendantReads.clear();
        commit.closestReads.clear();
        for (const auto &[descendant, closest] : commit.propagate) {
            commit.descendantReads.push_back(readFin(descendant));
            commit.closestReads.push_back(readFin(closest));
        }

        // Then the write-back, in the order it happens: the candidates in
        // slot order, then the descendants in propagation order. A slot that
        // is both gets two versions, and the second carries the first --
        // which is exactly what the one storage used to end up holding.
        commit.slotWrites.assign(commit.slots.size(), 0);
        commit.slotCarry.assign(commit.slots.size(), 0);
        commit.slotBaseWrites.assign(
            commit.solverOutput ? commit.slots.size() : 0, 0);
        commit.slotBaseCarry.assign(
            commit.solverOutput ? commit.slots.size() : 0, 0);
        for (size_t pos = 0; pos < commit.slots.size(); ++pos) {
            const size_t slot = size_t(commit.slots[pos]);
            commit.slotCarry[pos] = liveFin[slot];
            commit.slotWrites[pos] = take(&finWrites, &finNext, slot);
            liveFin[slot] = commit.slotWrites[pos];
            if (commit.solverOutput) {
                commit.slotBaseCarry[pos] = liveBase[slot];
                commit.slotBaseWrites[pos] =
                    take(&baseWrites, &baseNext, slot);
                liveBase[slot] = commit.slotBaseWrites[pos];
            }
        }
        commit.descendantWrites.assign(commit.propagate.size(), 0);
        commit.descendantCarry.assign(commit.propagate.size(), 0);
        commit.descendantBaseWrites.assign(
            commit.solverOutput ? commit.propagate.size() : 0, 0);
        commit.descendantBaseCarry.assign(
            commit.solverOutput ? commit.propagate.size() : 0, 0);
        for (size_t k = 0; k < commit.propagate.size(); ++k) {
            const size_t slot = size_t(commit.propagate[k].first);
            commit.descendantCarry[k] = liveFin[slot];
            commit.descendantWrites[k] = take(&finWrites, &finNext, slot);
            liveFin[slot] = commit.descendantWrites[k];
            if (commit.solverOutput) {
                commit.descendantBaseCarry[k] = liveBase[slot];
                commit.descendantBaseWrites[k] =
                    take(&baseWrites, &baseNext, slot);
                liveBase[slot] = commit.descendantBaseWrites[k];
            }
        }
    }

    if(!refreshBefore(B.walkSteps.size()))return;
    // All writes are indexed before ordinary data-flow inputs are bound.
    // A dependency can name a later authored writer; scheduling never changes
    // the positional carry versions or the producer selected here.
    std::set<uint32_t> refreshedBase,refreshedFin;
    for(const auto &refresh:B.providerRefreshes) {
        refreshedBase.insert(refresh.baseWrite);refreshedFin.insert(refresh.finWrite);
        for(const auto &carry:refresh.carries) {refreshedBase.insert(carry.baseWrite);refreshedFin.insert(carry.finWrite);}
    }
    const auto selected=[&](const SdfPath &reader,int slot,uint32_t positional,bool base=false) {
        if(slot<0 || size_t(slot)>=n) return positional;
        if((base?refreshedBase:refreshedFin).count(positional))return positional;
        const auto dependencies=B.poseDependencyPaths.find(reader);
        if(dependencies==B.poseDependencyPaths.end()) return positional;
        uint32_t value=positional;
        for(size_t w=0;w<B.walkSteps.size();++w) {
            const auto &walk=B.walkSteps[w]; const auto &commit=B.commits[w];
            bool reads=false;
            if(walk.solverBatch) for(const int si:walk.batchSolvers)
                reads=reads || dependencies->second.count(B.solvers[size_t(si)].path);
            else reads=dependencies->second.count(B.constraints[size_t(walk.index)].path);
            if(!reads || (base && !commit.solverOutput)) continue;
            const auto &writes=base?commit.slotBaseWrites:commit.slotWrites;
            for(size_t k=0;k<commit.slots.size();++k)
                if(commit.slots[k]==slot && k<writes.size()) value=writes[k];
            const auto &descendants=base?commit.descendantBaseWrites:commit.descendantWrites;
            for(size_t k=0;k<commit.propagate.size();++k)
                if(commit.propagate[k].first==slot && k<descendants.size()) value=descendants[k];
        }
        return value;
    };
    for(auto &input:B.providerFrameInputs)
        input.version=selected(input.reader,input.slot,input.version,input.base);
    for(size_t w=0;w<B.walkSteps.size();++w) {
        const auto &walk=B.walkSteps[w]; auto &commit=B.commits[w];
        if(walk.solverBatch) {
            for(const int si:walk.batchSolvers) {
                auto &solver=B.solvers[size_t(si)];
                for(size_t k=0;k<solver.controls.size();++k)
                    solver.controlReads[k]=selected(solver.path,solver.controls[k],solver.controlReads[k]);
                solver.startRead=selected(solver.path,solver.start,solver.startRead);
                solver.rootRead=selected(solver.path,solver.root,solver.rootRead);
                solver.midRead=selected(solver.path,solver.mid,solver.midRead);
                solver.endRead=selected(solver.path,solver.end,solver.endRead);
                solver.poleRead=selected(solver.path,solver.pole,solver.poleRead);
                solver.spaceRead=selected(solver.path,solver.spaceSlot,solver.spaceRead);
            }
            continue;
        }
        const auto &constraint=B.constraints[size_t(walk.index)];
        for(size_t k=0;k<constraint.sources.size();++k)
            commit.sourceReads[k]=selected(constraint.path,constraint.sources[k],commit.sourceReads[k]);
        commit.worldUpRead=selected(constraint.path,constraint.worldUpObject,commit.worldUpRead);
        commit.effectorRead=selected(constraint.path,constraint.effector,commit.effectorRead);
        for(size_t k=0;k<constraint.poleObjects.size();++k)
            commit.poleReads[k]=selected(constraint.path,constraint.poleObjects[k],commit.poleReads[k]);
        const auto ancestors=[&](auto &reads) {
            for(auto &read:reads) {
                read.fin=selected(constraint.path,read.slot,read.fin);
                read.base=selected(constraint.path,read.slot,read.base,true);
            }
        };
        for(auto &reads:commit.sourceAncestors) ancestors(reads);
        ancestors(commit.worldUpAncestors); ancestors(commit.effectorAncestors);
        for(auto &reads:commit.poleAncestors) ancestors(reads);
    }

    // And the guide-only solvers, against what the walk left standing: the
    // dynamic path's guide request overrides every provider with its FINAL
    // frame, which is what liveFin holds now that the sweep is over.
    for (const int si : B.guideSolvers) {
        bindSolverReads(B.solvers[size_t(si)]);
    }

    B.finLast.assign(liveFin.begin(), liveFin.end());
    B.baseLast.assign(liveBase.begin(), liveBase.end());
    B.fin.resize(finNext);
    B.base.resize(baseNext);
}

/// Builds `frameRecords` and every AtPrim reader's lists of them, before any
/// commit step exists: records are named by (provider, writer) and need no
/// version. A record is a target a constraint's exit records
/// (`snapshotTargets`) or an output a solver's commit records
/// (`outputSnapshots`) -- the compile's named pairs -- kept only when some
/// reader's list holds it. A list is the original AtPrim lookup
/// rule stated statically: the provider's records whose writer is the
/// phase's prim or under it, newest first in walk order.
void
EnumerateFrameRecords(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    std::vector<RigExecBakedFrameRecord> all;
    for (size_t w = 0; w < B.walkSteps.size(); ++w) {
        const RigExecBakedProgramImpl::WalkStep &walk = B.walkSteps[w];
        if (walk.solverBatch) {
            // The walk records every named joint of every solver in the
            // batch once the batch commits. `position` is bound with the
            // version, once the commit's slot table exists.
            for (const int si : walk.batchSolvers) {
                const RigExecBakedProgramImpl::Solver &solver =
                    B.solvers[size_t(si)];
                for (size_t k = 0; k < solver.outputs.size(); ++k) {
                    if (k < solver.outputSnapshots.size() &&
                        solver.outputSnapshots[k] &&
                        solver.outputs[k].first >= 0) {
                        RigExecBakedFrameRecord record;
                        record.slot = solver.outputs[k].first;
                        record.commit = int(w);
                        record.mover = solver.path;
                        all.push_back(std::move(record));
                    }
                }
            }
            continue;
        }
        const RigExecBakedProgramImpl::Constraint &constraint =
            B.constraints[size_t(walk.index)];
        for (size_t k = 0; k < constraint.targetSlots.size(); ++k) {
            if (k < constraint.snapshotTargets.size() &&
                constraint.snapshotTargets[k] &&
                constraint.targetSlots[k] >= 0) {
                RigExecBakedFrameRecord record;
                record.slot = constraint.targetSlots[k];
                record.commit = int(w);
                record.target = int(k);
                record.mover = constraint.path;
                all.push_back(std::move(record));
            }
        }
    }
    std::vector<char> used(all.size(), 0);
    const auto listFor = [&all, &used](int slot, const SdfPath &prim,
                                       std::vector<int> *out) {
        out->clear();
        if (slot < 0) {
            return;
        }
        for (size_t r = all.size(); r-- > 0;) {
            if (all[r].slot == slot &&
                (all[r].mover == prim || all[r].mover.HasPrefix(prim))) {
                out->push_back(int(r));
                used[r] = 1;
            }
        }
    };
    const auto bindReader =
        [&listFor](RigExecBakedProgramImpl::GeomRevision *revision) {
        revision->transformRecords.clear();
        revision->influenceRecords.clear();
        const RigExecReadPhase &phase = revision->binding.transformPhase;
        if (phase.kind != RigExecReadPhaseKind::AtPrim) {
            return;
        }
        listFor(revision->transformSlot, phase.prim,
                &revision->transformRecords);
        // The fold substitutes an influence entry only where the binding
        // names one, as the walk's fold does.
        revision->influenceRecords.resize(revision->influenceSlots.size());
        for (size_t k = 0; k < revision->influenceSlots.size() &&
                           k < revision->binding.influences.size();
             ++k) {
            listFor(revision->influenceSlots[k], phase.prim,
                    &revision->influenceRecords[k]);
        }
    };
    for (RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        for (RigExecBakedProgramImpl::GeomRevision &revision :
                 chain.revisions) {
            bindReader(&revision);
        }
        for (RigExecBakedProgramImpl::GeomChain::Derived &derived :
                 chain.derived) {
            bindReader(&derived.revision);
        }
    }
    std::vector<int> renumbered(all.size(), -1);
    B.frameRecords.clear();
    for (size_t r = 0; r < all.size(); ++r) {
        if (used[r]) {
            renumbered[r] = int(B.frameRecords.size());
            B.frameRecords.push_back(std::move(all[r]));
        }
    }
    const auto renumber = [&renumbered](std::vector<int> *records) {
        for (int &record : *records) {
            record = renumbered[size_t(record)];
        }
    };
    for (RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        for (RigExecBakedProgramImpl::GeomRevision &revision :
                 chain.revisions) {
            renumber(&revision.transformRecords);
            for (std::vector<int> &records : revision.influenceRecords) {
                renumber(&records);
            }
        }
        for (RigExecBakedProgramImpl::GeomChain::Derived &derived :
                 chain.derived) {
            renumber(&derived.revision.transformRecords);
            for (std::vector<int> &records :
                     derived.revision.influenceRecords) {
                renumber(&records);
            }
        }
    }
    B.frameMatrix.assign(B.frameRecords.size(), GfMatrix4d(1.0));
    B.frameMatrixValid.assign(B.frameRecords.size(), 0);
}

/// Binds each record to the `fin` entry its writer left the provider in,
/// once BindPoseVersions has handed the entries out: the commit's LAST
/// version of the slot where it declares one -- whether it revised it or
/// carried the one it found -- which is the frame the walk records after the
/// commit's propagation; else the version it read, which is where a
/// geometry-domain constraint leaves its target. A solver's record also
/// takes the slot's position in the commit, whose `present` byte gates it.
void
BindFrameRecordVersions(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    for (RigExecBakedFrameRecord &record : B.frameRecords) {
        const RigExecBakedCommit &commit = B.commits[size_t(record.commit)];
        const auto found = std::lower_bound(commit.slots.begin(),
                                            commit.slots.end(), record.slot);
        const bool candidate =
            found != commit.slots.end() && *found == record.slot;
        if (commit.solverOutput) {
            // Every solver output is a slot of its commit (the sizing loop
            // adds each one), so a miss is a broken table.
            TF_VERIFY(candidate);
            record.position =
                candidate ? int(found - commit.slots.begin()) : -1;
        }
        if (!candidate) {
            record.version =
                record.target >= 0 &&
                        size_t(record.target) < commit.targetReads.size()
                    ? commit.targetReads[size_t(record.target)]
                    : 0;
            continue;
        }
        record.version =
            commit.slotWrites[size_t(found - commit.slots.begin())];
        // The propagation pairs exclude every candidate (buildPropagation),
        // so this finds nothing today; it keeps the binding at the slot's
        // last version in the commit should a pair ever carry a candidate.
        for (size_t k = 0; k < commit.propagate.size(); ++k) {
            if (commit.propagate[k].first == record.slot) {
                record.version = commit.descendantWrites[k];
            }
        }
    }
}

/// A commit split into delta / staging / apply once its descendant list is
/// this long. Below it the three phases run as one step: the split buys
/// parallelism inside one propagation and costs two more steps, which is a
/// bad trade for the two- and three-descendant commits a rig is full of.
constexpr size_t kPropagateSplitThreshold = 64;
/// Descendants staged per PropagateChunk of a split commit.
constexpr size_t kPropagateChunkSize = 64;

/// What a switched slot's compose reads besides its own avars and its
/// namespace parent's anchor (which the group lists in parentSlots): the
/// PosedM slots read at their last version, and the slots whose avars an
/// earlier version is recomposed from.
void
_SwitchVersionReads(const RigExecBakedProgramImpl::SpaceSwitch &sw,
                    std::vector<int> *posed, std::vector<int> *avars)
{
    const auto add =
        [&](const RigExecBakedProgramImpl::SpaceSwitch::FrameVersion &read) {
            if (read.anchor >= 0) {
                posed->push_back(read.anchor);
            }
            avars->insert(avars->end(), read.recompose.begin(),
                          read.recompose.end());
        };
    for (size_t k = 0; k < sw.sourceSlots.size(); ++k) {
        if (sw.sourceSlots[k] >= 0 && k < sw.sourceReads.size()) {
            add(sw.sourceReads[k]);
        }
    }
    if (sw.spaceSlot >= 0) {
        add(sw.spaceRead);
    }
    avars->insert(avars->end(), sw.parentRead.recompose.begin(),
                  sw.parentRead.recompose.end());
}

/// The PosedM slot a slot's compose inherits from: its namespace parent, or
/// for a switched slot the anchor of the parent version it reads.
int
_ComposeParentRead(const RigExecBakedProgramImpl &B, int slot)
{
    const int switchIndex =
        B.spaceSwitchBySlot.empty() ? -1 : B.spaceSwitchBySlot[size_t(slot)];
    return switchIndex >= 0
        ? B.spaceSwitches[size_t(switchIndex)].parentRead.anchor
        : B.parent[size_t(slot)];
}

}  // namespace

bool
RigExecBakedBindSpaceSwitchVersions(RigExecBakedProgramImpl *program,std::string *error)
{
    auto &B=*program;
    using FrameVersion=RigExecBakedProgramImpl::SpaceSwitch::FrameVersion;
    std::vector<uint8_t> composed(B.paths.size(),0);
    for(size_t slot=0;slot<B.paths.size();++slot)
        composed[slot]=B.slotKind[slot]==RigExecBakedSlotKind::FirstFramePose;
    std::vector<int> targets,carrySlots;
    std::vector<std::vector<int>> sources;
    for(const auto &sw:B.spaceSwitches) {
        targets.push_back(sw.slot);sources.push_back(sw.sourceSlots);carrySlots.push_back(sw.spaceSlot);
    }
    std::vector<RigExecSpaceParentContext> contexts,carryContexts;
    if(!RigExecDeriveSpaceSwitchParentContexts(B.parent,composed,targets,sources,
        &contexts,&carrySlots,&carryContexts)) {
        if(error)*error="space switch checkpoint has an invalid provider identity";
        return false;
    }
    B.switchFrameContexts.clear();
    for(size_t index=0;index<B.spaceSwitches.size();++index) {
        auto &sw=B.spaceSwitches[index];
        const auto bind=[&](const RigExecSpaceParentContext &selected,const char *site) {
            FrameVersion read;
            if(selected.recompose.empty())read.anchor=selected.anchor;
            else {
                RigExecBakedProgramImpl::SpaceCheckpoint checkpoint;
                checkpoint.key="switchCheckpoint:"+B.paths[size_t(sw.slot)].GetString()+":"+site;
                checkpoint.anchor=selected.anchor;checkpoint.recompose=selected.recompose;
                checkpoint.kernelInputs.resize(checkpoint.recompose.size());
                read.context=int(B.switchFrameContexts.size());
                B.switchFrameContexts.push_back(std::move(checkpoint));
            }
            return read;
        };
        sw.parentRead=bind(contexts[index],"parent");
        sw.spaceRead=bind(carryContexts[index],"carry");
        sw.sourceReads.assign(sw.sourceSlots.size(),FrameVersion());
        for(size_t k=0;k<sw.sourceSlots.size();++k)sw.sourceReads[k].anchor=sw.sourceSlots[k];
    }
    B.switchFrames.assign(B.switchFrameContexts.size(),GfMatrix4d(1.0));
    return true;
}

void
RigExecBakedBuildPoseSteps(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    const int N = int(B.paths.size());
    for(size_t index=0;index<B.switchFrameContexts.size();++index) {
        const auto &context=B.switchFrameContexts[index];
        auto &step=AddStep(&B,RigExecBakedStepKind::SpaceCheckpoint,int(index));
        step.label=context.key;
        if(context.anchor>=0)
            step.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::PosedM,context.anchor));
        for(int slot:context.recompose) {
            step.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::Avars,slot));
            step.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::Ladder,slot));
        }
        step.writes.push_back(RigExecBakedOne(RigExecBakedSlotDomain::SwitchFrame,int(index)));
    }

    // A provider is a semantic operation. Packing occurs after the common
    // graph has bound every connected and phased input to its producer.
    std::vector<size_t> emission;
    for (int slot=0;slot<N;++slot) {
        // Ordinary Xform targets are sampled frame seeds, not composed
        // providers. A skipped body must not claim their source versions.
        if (B.slotKind[size_t(slot)] != RigExecBakedSlotKind::FirstFramePose) continue;
        RigExecBakedComposeGroup group;
        group.begin=slot; group.end=slot+1;
        const int parent=_ComposeParentRead(B,slot);
        if(parent>=0) group.parentSlots.push_back(parent);
        emission.push_back(B.composeGroups.size());
        B.composeGroups.push_back(std::move(group));
    }
    for (const size_t g : emission) {
        const RigExecBakedComposeGroup &group = B.composeGroups[g];
        RigExecBakedStep &step = AddStep(
            &B, RigExecBakedStepKind::ComposeSubtree, int(g));
        step.reads.push_back(RigExecBakedRange(
            RigExecBakedSlotDomain::Avars, group.begin, group.end));
        for (const int parent : group.parentSlots) {
            step.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::PosedM, parent));
        }
        // A switched slot reads its spaces instead of its namespace parent,
        // so those frames are reads of this step too -- that is what orders
        // a pole vector's group after its IK handle's. A source at -1 is
        // world; an own-source read remains a real dependency for SCC policy.
        for (int slot = group.begin; slot < group.end; ++slot) {
            const int switchIndex =
                B.spaceSwitchBySlot.empty()
                    ? -1 : B.spaceSwitchBySlot[size_t(slot)];
            if (switchIndex < 0) continue;
            const RigExecBakedProgramImpl::SpaceSwitch &sw =
                B.spaceSwitches[size_t(switchIndex)];
            const auto checkpointRead=[&](const auto &read) {
                if(read.context>=0)step.reads.push_back(
                    RigExecBakedOne(RigExecBakedSlotDomain::SwitchFrame,read.context));
            };
            checkpointRead(sw.parentRead);checkpointRead(sw.spaceRead);
            for(const auto &read:sw.sourceReads)checkpointRead(read);
            std::vector<int> reads, recomposed;
            _SwitchVersionReads(sw, &reads, &recomposed);
            // An earlier version recomposed here reads those slots' avars,
            // declared so that moving one dirties this step as well as the
            // group that composes the slot's last version
            // (RigExecBakedCones::avarVersionSteps).
            std::sort(recomposed.begin(), recomposed.end());
            recomposed.erase(
                std::unique(recomposed.begin(), recomposed.end()),
                recomposed.end());
            for (const int at : recomposed) {
                if (at < group.begin || at >= group.end) {
                    step.reads.push_back(RigExecBakedRange(
                        RigExecBakedSlotDomain::Avars, at, at + 1));
                }
            }
            for(const int source:reads)if(source>=0)
                step.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::PosedM,source));
            // The index itself is declared by
            // RigExecBakedDeclareInputDependencies.
        }
        for(int slot=group.begin;slot<group.end;++slot) {
            const int index=(size_t(slot)<B.autoClavicleBySlot.size()?B.autoClavicleBySlot[size_t(slot)]:-1);if(index<0)continue;
            const auto &ac=B.autoClavicles[size_t(index)];
            for(const auto &read:ac.frames) {
                if(read.computation=="computeRestFrame")step.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::Rest,read.slot));
                else if(read.computation=="computeDefaultFrame")step.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::Ladder,read.slot));
                for(int child:read.recompose) {
                    step.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::Avars,child));
                    step.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::Ladder,child));
                }
            }
        }
        step.writes.push_back(RigExecBakedRange(
            RigExecBakedSlotDomain::PoseBase, group.begin, group.end));
        step.writes.push_back(RigExecBakedRange(
            RigExecBakedSlotDomain::PoseFin, group.begin, group.end));
        step.writes.push_back(RigExecBakedRange(
            RigExecBakedSlotDomain::PosedM, group.begin, group.end));
    }

    B.commits.resize(B.walkSteps.size());
    EnumerateFrameRecords(&B);
    // One FrameMatrix step per record, right after its commit's last step.
    // It reads the provider's frame -- an edge from the latest writer of the
    // slot so far, which is this commit or the producer of the version it
    // read -- and the commit's table (a constraint's exit flags, a solver
    // batch's `present` bytes), which its first step writes (CommitTable).
    size_t nextRecord = 0;
    const auto addFrameRecordSteps = [&B, &nextRecord](size_t w) {
        for (; nextRecord < B.frameRecords.size() &&
               size_t(B.frameRecords[nextRecord].commit) == w;
             ++nextRecord) {
            const RigExecBakedFrameRecord &record =
                B.frameRecords[nextRecord];
            RigExecBakedStep &step = AddStep(
                &B, RigExecBakedStepKind::FrameMatrix, int(nextRecord));
            step.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::PoseFin, record.slot));
            step.reads.push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::CommitTable, record.commit));
            step.writes.push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::FrameMatrix, int(nextRecord)));
        }
    };
    // Where the next split commit's staging pairs start. The scratch behind
    // them is per commit, but the SLOT IDS are handed out once for the whole
    // program: two commits declaring the same [0, n) would be ordered against
    // each other by the sweep for sharing a slot neither can reach.
    int stagingSlots = 0;
    for (size_t w = 0; w < B.walkSteps.size(); ++w) {
        const RigExecBakedProgramImpl::WalkStep &walk = B.walkSteps[w];
        RigExecBakedCommit &commit = B.commits[w];
        commit.solverOutput = walk.solverBatch;
        commit.propagate = walk.propagate;
        // The candidate table is dense and in SLOT order, because both
        // halves of the commit -- the usability sweep and the write-back --
        // walk it in slot order. Where each producer lands is decided here,
        // so the merge is an indexed store and "last writer wins on a
        // duplicate slot" survives as the order the stores happen in.
        if (walk.solverBatch) {
            // Each solver owns its authored commit and distinct pose versions.
            for (const int si : walk.batchSolvers) {
                for (const auto &[slot, element] : B.solvers[size_t(si)]
                                                       .outputs) {
                    commit.slots.push_back(slot);
                }
            }
        } else {
            const RigExecBakedProgramImpl::Constraint &constraint =
                B.constraints[size_t(walk.index)];
            commit.moverPath = constraint.path;
            commit.moverPathText = constraint.pathText;
            if (constraint.target >= 0 && constraint.pointsTarget.IsEmpty()) {
                if (constraint.singleChainIk) {
                    // The one multi-target built-in: every chain joint is a
                    // candidate, and the commit is atomic over all of them.
                    commit.slots.insert(commit.slots.end(),
                                        constraint.targetSlots.begin(),
                                        constraint.targetSlots.end());
                } else {
                    commit.slots.push_back(constraint.target);
                }
            }
            commit.sources.resize(constraint.sources.size());
            commit.ikChain.resize(constraint.targetSlots.size());
            commit.ikRest.resize(constraint.targetSlots.size());
            commit.ikPrepared.reserve(constraint.targetSlots.size());
        }
        std::sort(commit.slots.begin(), commit.slots.end());
        commit.slots.erase(
            std::unique(commit.slots.begin(), commit.slots.end()),
            commit.slots.end());
        const auto positionOf = [&commit](int slot) {
            const auto found = std::lower_bound(commit.slots.begin(),
                                                commit.slots.end(), slot);
            return found != commit.slots.end() && *found == slot
                       ? int(found - commit.slots.begin())
                       : -1;
        };
        commit.present.assign(commit.slots.size(), 0);
        commit.frames.resize(commit.slots.size());
        commit.deltas.assign(commit.slots.size(), GfMatrix4d(1.0));
        commit.deltaOk.assign(commit.slots.size(), 0);
        commit.staged.resize(commit.propagate.size());
        commit.outcome.assign(commit.propagate.size(), 0);
        commit.closestPos.reserve(commit.propagate.size());
        for (const auto &[descendant, closest] : commit.propagate) {
            commit.closestPos.push_back(positionOf(closest));
        }
        commit.split = commit.propagate.size() > kPropagateSplitThreshold;
        if (commit.split) {
            commit.stagingBase = stagingSlots;
            stagingSlots += int(commit.propagate.size());
        }

        if (walk.solverBatch) {
            for (const int si : walk.batchSolvers) {
                RigExecBakedProgramImpl::Solver &solver =
                    B.solvers[size_t(si)];
                solver.outFrames.resize(solver.outputs.size());
                solver.outPresent.assign(solver.outputs.size(), 0);
                solver.outPosition.clear();
                for (const auto &[slot, element] : solver.outputs) {
                    solver.outPosition.push_back(positionOf(slot));
                }
                // +1 for the start provider's synthetic base element.
                solver.kernelWorkspace.fkElements.resize(solver.controls.size() +
                                       (solver.start >= 0 ? 1 : 0));
                RigExecBakedStep &step =
                    AddStep(&B, RigExecBakedStepKind::Solve, si);
                for (const int control : solver.controls) {
                    if (control >= 0) {
                        step.reads.push_back(RigExecBakedOne(
                            RigExecBakedSlotDomain::PoseFin, control));
                    }
                }
                for (const int control : {solver.root, solver.mid,
                                          solver.end, solver.pole}) {
                    if (control >= 0) {
                        step.reads.push_back(RigExecBakedOne(
                            RigExecBakedSlotDomain::PoseFin, control));
                    }
                }
                // The start provider's posed frame: undeclared, the
                // scheduler is free to run this solve before the write.
                if (solver.start >= 0) {
                    step.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::PoseFin, solver.start));
                }
                // The LIVE joint rests: a solver whose rest reference is the
                // frame a pose step below it left READS that slot's PoseFin,
                // and a read a step does not declare is a read the scheduler
                // is free to run before the write (spec 4.2, live rests).
                // Empty on every solver with no step below it.
                if (solver.hasLiveRest) {
                    for (size_t k = 0; k < solver.restRefs.size(); ++k) {
                        if (k < solver.restIsLive.size() &&
                            solver.restIsLive[k] &&
                            solver.restRefs[k].first >= 0) {
                            step.reads.push_back(RigExecBakedOne(
                                RigExecBakedSlotDomain::PoseFin,
                                solver.restRefs[k].first));
                        }
                    }
                }
                // The ribbon's driver points, filled by the prologue.
                // Nothing in the program writes the domain, so the read
                // raises no edge; it is declared because it is what the cone
                // follows when the curve moves, and because a step's
                // declaration is meant to say what the step touches.
                if (!solver.ribbonPointsPath.IsEmpty()) {
                    step.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::SolverPoints, si));
                }
                // An input solver's Solve step precedes this one; the
                // validator refuses a program where it does not.
                for (const int input : {solver.inA, solver.inB}) {
                    if (input >= 0) {
                        step.reads.push_back(RigExecBakedOne(
                            RigExecBakedSlotDomain::Aggregate, input));
                    }
                }
                step.writes.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::Aggregate, si));
                step.writes.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::Candidates, si));
            }
        }

        // The commit itself: one step that merges (or computes) the
        // candidates, plus -- for a propagation long enough to be worth it --
        // the delta / staging / apply split. Both arrangements call the same
        // three functions, so there is one definition of what a commit means.
        const RigExecBakedStepKind head =
            walk.solverBatch ? RigExecBakedStepKind::SolverCommit
                             : RigExecBakedStepKind::Constraint;
        RigExecBakedStep &commitStep = AddStep(&B, head, int(w));
        commitStep.maxDiagnostics = RigExecBakedMaxStepDiagnostics;
        if (walk.solverBatch) {
            for (const int si : walk.batchSolvers) {
                commitStep.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::Candidates, si));
            }
        } else {
            const RigExecBakedProgramImpl::Constraint &constraint =
                B.constraints[size_t(walk.index)];
            for (const int source : constraint.sources) {
                if (source >= 0) {
                    commitStep.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::PoseFin, source));
                }
            }
            if (constraint.target >= 0) {
                commitStep.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::PoseFin, constraint.target));
            }
            // A geometry-domain constraint's one write: the delta the Matrix
            // revision of the same mover rides.
            if (constraint.deltaBase >= 0) {
                commitStep.writes.push_back(
                    RigExecBakedOne(RigExecBakedSlotDomain::ConstraintDelta,
                                    constraint.deltaBase));
            }
            if (constraint.worldUpObject >= 0) {
                commitStep.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::PoseFin,
                    constraint.worldUpObject));
            }
            // rigExec:space: the carry is read off the compose's own
            // table, exactly as a switched slot reads its space, so moving
            // a master dirties this step.
            if (constraint.spaceSlot >= 0) {
                commitStep.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::PosedM, constraint.spaceSlot));
            }
            // Both frames of every provider a native source might ride: the
            // step compares them to decide which ancestor moved, so both are
            // reads whatever this frame's answer turns out to be.
            const auto declareNative = [&B, &commitStep](int native) {
                if (native < 0) {
                    return;
                }
                for (const int slot :
                         B.nativeSources[size_t(native)].ancestorSlots) {
                    commitStep.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::PoseFin, slot));
                    commitStep.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::PoseBase, slot));
                }
            };
            for (const int native : constraint.sourceNatives) {
                declareNative(native);
            }
            declareNative(constraint.worldUpNative);
            // A SingleChainIK reads its whole chain, its effector and every
            // pole object.
            for (const int slot : constraint.targetSlots) {
                commitStep.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::PoseFin, slot));
            }
            if (constraint.effector >= 0) {
                commitStep.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::PoseFin, constraint.effector));
            }
            declareNative(constraint.effectorNative);
            for (size_t k = 0; k < constraint.poleObjects.size(); ++k) {
                if (constraint.poleObjects[k] >= 0) {
                    commitStep.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::PoseFin,
                        constraint.poleObjects[k]));
                }
                declareNative(constraint.poleObjectNatives[k]);
            }
        }
        commitStep.writes.push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::CommitTable, int(w)));
        // The commit reads its descendants and their closest revised
        // ancestors and writes them back: a read-modify-write of both, which
        // is what the propagation IS.
        const auto declarePropagation =
            [&B, &commit](RigExecBakedStep *step, bool writes) {
            for (const auto &[descendant, closest] : commit.propagate) {
                step->reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::PoseFin, descendant));
                step->reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::PoseFin, closest));
                if (!writes) {
                    continue;
                }
                step->writes.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::PoseFin, descendant));
                if (commit.solverOutput) {
                    step->writes.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::PoseBase, descendant));
                }
            }
            if (!writes) {
                return;
            }
            for (const int slot : commit.slots) {
                step->writes.push_back(
                    RigExecBakedOne(RigExecBakedSlotDomain::PoseFin, slot));
                if (commit.solverOutput) {
                    step->writes.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::PoseBase, slot));
                }
            }
        };
        // What the write-back CARRIES is a read too, and of a different
        // version than the delta's: FinishCommit copies the version it found
        // into the storage it owns wherever it declines to write, so a step
        // that carries TOUCHES the version it carries. PoseFin's half is the
        // candidate read the unsplit arrangement declares just below for the
        // delta -- but the split arrangement performs the carry on its APPLY
        // step, whose reads say nothing about it, and the PoseBase half of a
        // solver commit is declared nowhere at all. Both are edges the
        // write-after-write pass already raises against the same writers, so
        // stating them adds no edge and changes no cone; what it buys is a
        // graph that still describes what the step reads, which is what the
        // next writer of one of these steps will reason from.
        const auto declareCarryReads = [&commit](RigExecBakedStep *step) {
            for (const int slot : commit.slots) {
                step->reads.push_back(
                    RigExecBakedOne(RigExecBakedSlotDomain::PoseFin, slot));
                if (commit.solverOutput) {
                    step->reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::PoseBase, slot));
                }
            }
            if (!commit.solverOutput) {
                return;
            }
            for (const auto &pair : commit.propagate) {
                step->reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::PoseBase, pair.first));
            }
        };
        if (!commit.split) {
            // The delta this arrangement measures itself is against each
            // candidate slot's value BEFORE the commit revises it:
            // ComputeCommitDeltas reads B.fin[slot] and the write-back then
            // overwrites it, so the read is real and is not implied by the
            // write. The split arrangement declares exactly these on its
            // CommitDelta step; declaring them here too is what keeps a cone
            // from skipping a commit in a generation that moved one of them
            // and leaving it measuring against its own last answer.
            for (const int slot : commit.slots) {
                commitStep.reads.push_back(
                    RigExecBakedOne(RigExecBakedSlotDomain::PoseFin, slot));
            }
            declarePropagation(&commitStep, /* writes = */ true);
            declareCarryReads(&commitStep);
            addFrameRecordSteps(w);
            continue;
        }
        {
            RigExecBakedStep &delta =
                AddStep(&B, RigExecBakedStepKind::CommitDelta, int(w));
            delta.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::CommitTable, int(w)));
            for (const int slot : commit.slots) {
                delta.reads.push_back(
                    RigExecBakedOne(RigExecBakedSlotDomain::PoseFin, slot));
            }
            delta.writes.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::CommitDelta, int(w)));
        }
        for (size_t begin = 0; begin < commit.propagate.size();
             begin += kPropagateChunkSize) {
            const size_t end = std::min(begin + kPropagateChunkSize,
                                        commit.propagate.size());
            RigExecBakedStep &chunk =
                AddStep(&B, RigExecBakedStepKind::PropagateChunk, int(w),
                        int(begin / kPropagateChunkSize));
            chunk.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::CommitTable, int(w)));
            chunk.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::CommitDelta, int(w)));
            for (size_t k = begin; k < end; ++k) {
                chunk.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::PoseFin,
                    commit.propagate[k].first));
                chunk.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::PoseFin,
                    commit.propagate[k].second));
            }
            chunk.writes.push_back(RigExecBakedRange(
                RigExecBakedSlotDomain::CommitStaging,
                commit.stagingBase + int(begin),
                commit.stagingBase + int(end)));
        }
        {
            RigExecBakedStep &apply =
                AddStep(&B, RigExecBakedStepKind::CommitApply, int(w));
            apply.maxDiagnostics = RigExecBakedMaxStepDiagnostics;
            apply.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::CommitTable, int(w)));
            apply.reads.push_back(RigExecBakedRange(
                RigExecBakedSlotDomain::CommitStaging, commit.stagingBase,
                commit.stagingBase + int(commit.propagate.size())));
            declarePropagation(&apply, /* writes = */ true);
            declareCarryReads(&apply);
        }
        addFrameRecordSteps(w);
    }
    // Before the matrices, which read the LAST version of a slot: the table
    // that says which entry that is comes out of this sweep.
    BindPoseVersions(&B);
    for (auto &step : B.steps) {
        if (step.kind!=RigExecBakedStepKind::ComposeSubtree) continue;
        const auto &group=B.composeGroups[size_t(step.object)];
        for (const auto &ac : B.autoClavicles) {
            if (ac.slot<group.begin || ac.slot>=group.end) continue;
            for (const auto &read : ac.frames)
                if (read.computation.empty() && read.recompose.empty() && read.slot!=ac.slot)
                    step.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::PoseFin,
                        B.finLast[size_t(read.slot)]));
        }
    }

    if(!B.providerRefreshError.empty())return;
    for(size_t index=0;index<B.providerRefreshes.size();++index) {
        const auto &refresh=B.providerRefreshes[index];
        auto &step=AddStep(&B,RigExecBakedStepKind::ProviderRefresh,int(index));
        step.part=0;
        step.maxDiagnostics=1;
        step.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::SpaceValue,refresh.baseValue));
        step.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::SpaceValue,refresh.currentValue));
        step.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::PoseBase,refresh.baseRead));
        step.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::PoseFin,refresh.finRead));
        step.writes.push_back(RigExecBakedOne(RigExecBakedSlotDomain::PoseBase,refresh.baseWrite));
        step.writes.push_back(RigExecBakedOne(RigExecBakedSlotDomain::PoseFin,refresh.finWrite));
        for(const auto &carry:refresh.carries) {
            step.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::PoseBase,carry.baseRead));
            step.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::PoseFin,carry.finRead));
            step.writes.push_back(RigExecBakedOne(RigExecBakedSlotDomain::PoseBase,carry.baseWrite));
            step.writes.push_back(RigExecBakedOne(RigExecBakedSlotDomain::PoseFin,carry.finWrite));
            for(int slot:carry.blockingSlots) {
                const int raw=B.providerParentRawLeaves[size_t(slot)];
                if(raw>=0)step.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::SpaceLeaf,uint32_t(raw)));
            }
        }
        for(const auto &[commit,pos]:refresh.priorConstraints)
            step.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::CommitTable,commit));
    }
    BindFrameRecordVersions(&B);

    // The dynamic path answers these from a second exec request, whose
    // per-provider override is the FINAL frame rather than the mid-walk one
    // a batch sees -- so they run HERE, after the whole walk, and read the
    // last version of everything. They write no candidate and bump no
    // counter: a solver that binds a joint or drives geometry is required
    // and therefore batched, so one of these poses nothing and the only
    // thing that reads it is the guide publication.
    // Not gated on the guide toggle. The toggle can move without the epoch
    // moving, so gating would be a runtime branch in a step body; running
    // and not publishing is the same published generation, because a
    // solver step only reads its sampled leaves and the epilogue consults
    // the toggle where the dynamic path does. The leaves are sampled in the
    // prologue whether or not a guide solver runs, so the static-input
    // cache's counters (spec 5.2 [P37][S36]) do not move with the toggle
    // either.
    for (const int si : B.guideSolvers) {
        RigExecBakedProgramImpl::Solver &solver = B.solvers[size_t(si)];
        // +1 for the start provider's synthetic base element.
        solver.kernelWorkspace.fkElements.resize(solver.controls.size() +
                               (solver.start >= 0 ? 1 : 0));
        RigExecBakedStep &step =
            AddStep(&B, RigExecBakedStepKind::Solve, si);
        for (const int control : solver.controls) {
            if (control >= 0) {
                step.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::PoseFin, control));
            }
        }
        for (const int control :
                 {solver.root, solver.mid, solver.end, solver.pole}) {
            if (control >= 0) {
                step.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::PoseFin, control));
            }
        }
        if (solver.start >= 0) {
            step.reads.push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::PoseFin, solver.start));
        }
        if (!solver.ribbonPointsPath.IsEmpty()) {
            step.reads.push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::SolverPoints, si));
        }
        for (const int input : {solver.inA, solver.inB}) {
            if (input >= 0) {
                step.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::Aggregate, input));
            }
        }
        step.writes.push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::Aggregate, si));
        step.writes.push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::Candidates, si));
    }

    // Final for every published joint, and the reads
    // RigExecBakedForEachMatrixRead enumerates for every geometry revision.
    // A ProviderMatrix step never gives the generation back --
    // RigExecPointsToMatrix leaves the identity and says so -- so the one
    // bail of this phase stays where it is, in the joint publication.
    B.needFinal.assign(size_t(N), 0);
    B.needBase.assign(size_t(N), 0);
    B.finalMatrix.assign(size_t(N), GfMatrix4d(1.0));
    B.baseMatrix.assign(size_t(N), GfMatrix4d(1.0));
    for (const int slot : B.jointSlots) {
        if (slot >= 0) {
            B.needFinal[size_t(slot)] = 1;
        }
    }
    const auto needForRevision =
        [&B](const RigExecBakedProgramImpl::GeomRevision &revision) {
        RigExecBakedForEachMatrixRead(
            revision, [&B](RigExecBakedSlotDomain domain, int slot) {
            std::vector<char> &table =
                domain == RigExecBakedSlotDomain::FinalMatrix ? B.needFinal
                                                              : B.needBase;
            table[size_t(slot)] = 1;
        });
    };
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
                 chain.revisions) {
            needForRevision(revision);
        }
        for (const RigExecBakedProgramImpl::GeomChain::Derived &derived :
                 chain.derived) {
            needForRevision(derived.revision);
        }
    }
    for (int slot = 0; slot < N; ++slot) {
        if (B.needFinal[size_t(slot)]) {
            RigExecBakedStep &step = AddStep(
                &B, RigExecBakedStepKind::ProviderMatrix, slot, 1);
            step.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::PoseFin, slot));
            step.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::PoseBase, slot));
            step.writes.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::FinalMatrix, slot));
        }
        if (B.needBase[size_t(slot)]) {
            RigExecBakedStep &step = AddStep(
                &B, RigExecBakedStepKind::ProviderMatrix, slot, 0);
            step.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::PoseBase, slot));
            step.writes.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::BaseMatrix, slot));
        }
    }

    // After the matrices, which is where the dynamic path's phase sits: it
    // reads the FINAL pose, every constraint included, and writes what the
    // geometry chains consume. One step per interpolator, reading the last
    // version of its driver's slot and of the parent's, writing its own range
    // of weights.
    for (size_t k = 0; k < B.poseInterpolators.size(); ++k) {
        const RigExecBakedProgramImpl::PoseInterpolator &interpolator =
            B.poseInterpolators[k];
        RigExecBakedStep &step =
            AddStep(&B, RigExecBakedStepKind::PoseInterpolator, int(k));
        // One line, and terminal: a driver without a usable frame, or a
        // solve of the wrong size, zeroes the weights and says so.
        step.maxDiagnostics = 1;
        // A numeric driver reads dials and no frame at all, so it names no
        // slot to read: pushing one would read slot -1.
        if (interpolator.driverSlot >= 0) {
            step.reads.push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::PoseFin, interpolator.driverSlot));
        }
        if (interpolator.parentSlot >= 0) {
            step.reads.push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::PoseFin, interpolator.parentSlot));
        }
        step.writes.push_back(RigExecBakedRange(
            RigExecBakedSlotDomain::PoseWeight, interpolator.weightBegin,
            interpolator.weightEnd));
    }

    // The body bindings are the operation's value identities. Provider slots
    // name only the initial compose values; every authored commit owns its
    // assigned SSA outputs, independent of the common graph's eventual order.
    using Domain = RigExecBakedSlotDomain;
    using Kind = RigExecBakedStepKind;
    for (auto &step : B.steps) {
        if (step.kind != Kind::Solve && step.kind != Kind::FrameMatrix &&
            step.kind != Kind::ProviderMatrix && step.kind != Kind::PoseInterpolator &&
            step.kind != Kind::Constraint && step.kind != Kind::SolverCommit &&
            step.kind != Kind::CommitDelta && step.kind != Kind::PropagateChunk &&
            step.kind != Kind::CommitApply) continue;
        const auto poseRange = [](const RigExecBakedSlotRange &range) {
            return range.domain == Domain::PoseFin || range.domain == Domain::PoseBase;
        };
        step.reads.erase(std::remove_if(step.reads.begin(), step.reads.end(), poseRange), step.reads.end());
        step.writes.erase(std::remove_if(step.writes.begin(), step.writes.end(), poseRange), step.writes.end());
        const auto read = [&](Domain domain, uint32_t value) {
            step.reads.push_back(RigExecBakedOne(domain, value));
        };
        const auto readAll = [&](Domain domain, const auto &values) {
            for (const auto value : values) read(domain, value);
        };
        const auto writeAll = [&](Domain domain, const auto &values) {
            for (const auto value : values) step.writes.push_back(RigExecBakedOne(domain, value));
        };
        if (step.kind == Kind::Solve) {
            const auto &solver = B.solvers[size_t(step.object)];
            readAll(Domain::PoseFin, solver.controlReads);
            for (const auto binding : {std::make_pair(solver.start, solver.startRead),
                     std::make_pair(solver.root, solver.rootRead),
                     std::make_pair(solver.mid, solver.midRead),
                     std::make_pair(solver.end, solver.endRead),
                     std::make_pair(solver.pole, solver.poleRead),
                     std::make_pair(solver.spaceSlot, uint32_t(solver.spaceRead))}) {
                if (binding.first >= 0) read(Domain::PoseFin, binding.second);
            }
            for (size_t k = 0; k < solver.restReads.size(); ++k)
                if (k < solver.restIsLive.size() && solver.restIsLive[k])
                    read(Domain::PoseFin, solver.restReads[k]);
        } else if (step.kind == Kind::FrameMatrix) {
            read(Domain::PoseFin, B.frameRecords[size_t(step.object)].version);
        } else if (step.kind == Kind::ProviderMatrix) {
            const size_t slot = size_t(step.object);
            if (step.part) read(Domain::PoseFin, B.finLast[slot]);
            read(Domain::PoseBase, B.baseLast[slot]);
        } else if (step.kind == Kind::PoseInterpolator) {
            const auto &interp = B.poseInterpolators[size_t(step.object)];
            if (interp.driverSlot >= 0) read(Domain::PoseFin, B.finLast[size_t(interp.driverSlot)]);
            if (interp.parentSlot >= 0) read(Domain::PoseFin, B.finLast[size_t(interp.parentSlot)]);
        } else if (step.kind == Kind::Constraint || step.kind == Kind::SolverCommit ||
                   step.kind == Kind::CommitDelta || step.kind == Kind::PropagateChunk ||
                   step.kind == Kind::CommitApply) {
            const auto &commit = B.commits[size_t(step.object)];
            if (step.kind == Kind::Constraint) {
                const auto &c = B.constraints[size_t(B.walkSteps[size_t(step.object)].index)];
                for (size_t k = 0; k < c.sources.size(); ++k)
                    if (c.sources[k] >= 0) read(Domain::PoseFin, commit.sourceReads[k]);
                if (c.target >= 0) read(Domain::PoseFin, commit.targetRead);
                readAll(Domain::PoseFin, commit.targetReads);
                if (c.worldUpObject >= 0) read(Domain::PoseFin, commit.worldUpRead);
                if (c.effector >= 0) read(Domain::PoseFin, commit.effectorRead);
                for (size_t k = 0; k < c.poleObjects.size(); ++k)
                    if (c.poleObjects[k] >= 0) read(Domain::PoseFin, commit.poleReads[k]);
                const auto ancestors = [&](const auto &values) {
                    for (const auto &value : values) {
                        read(Domain::PoseFin, value.fin); read(Domain::PoseBase, value.base);
                    }
                };
                for (const auto &values : commit.sourceAncestors) ancestors(values);
                for (const auto &values : commit.poleAncestors) ancestors(values);
                ancestors(commit.worldUpAncestors); ancestors(commit.effectorAncestors);
            }
            const bool apply = step.kind == Kind::CommitApply ||
                (!commit.split && (step.kind == Kind::Constraint || step.kind == Kind::SolverCommit));
            if (step.kind == Kind::CommitDelta || (apply && !commit.split))
                readAll(Domain::PoseFin, commit.slotReads);
            if (step.kind == Kind::PropagateChunk || (apply && !commit.split)) {
                const size_t begin = step.kind == Kind::PropagateChunk
                    ? size_t(step.part) * kPropagateChunkSize : 0;
                const size_t end = step.kind == Kind::PropagateChunk
                    ? std::min(begin + kPropagateChunkSize, commit.propagate.size())
                    : commit.propagate.size();
                for (size_t k = begin; k < end; ++k) {
                    read(Domain::PoseFin, commit.descendantReads[k]);
                    read(Domain::PoseFin, commit.closestReads[k]);
                }
            }
            if (apply) {
                writeAll(Domain::PoseFin, commit.slotWrites);
                writeAll(Domain::PoseFin, commit.descendantWrites);
                writeAll(Domain::PoseBase, commit.slotBaseWrites);
                writeAll(Domain::PoseBase, commit.descendantBaseWrites);
                for(auto value:commit.slotCarry)read(Domain::PoseFin,value);
                for(auto value:commit.slotBaseCarry)read(Domain::PoseBase,value);
                const auto carries = [&](Domain domain,const auto &values,const auto &candidateWrites) {
                    for(size_t k=0;k<values.size();++k) {
                        bool internal=false;
                        for(size_t pos=0;pos<commit.slots.size() && pos<candidateWrites.size();++pos)
                            internal=internal || (commit.propagate[k].first==commit.slots[pos] && values[k]==candidateWrites[pos]);
                        if(!internal)read(domain,values[k]);
                    }
                };
                carries(Domain::PoseFin,commit.descendantCarry,commit.slotWrites);
                carries(Domain::PoseBase,commit.descendantBaseCarry,commit.slotBaseWrites);
            }
        }
    }
}

// One frame, pose half.
// Everything below is a STEP BODY or one half of the serial prologue or
// epilogue. A body touches no pose, takes no lock, opens no profile scope and
// writes only the slots its step declared; whatever it has to say it says
// into its own step.

namespace {

/// Records \p input against \p step: what a frame can move it with.
///
/// The rule itself is RigExecBakedNoteInput in bakedProgramImpl.h, because
/// the weight half declares its inputs the same way and two spellings of
/// "what can move this" is a step the cone cannot dirty.
template <class T>
void
NoteInput(const RigExecBakedInput<T> &input, RigExecBakedDependencySink *sink)
{
    RigExecBakedNoteInput(input, sink);
}

/// Every per-frame input one solver's Solve step reads.
void
NoteSolverInputs(const RigExecBakedProgramImpl::Solver &solver,
                 RigExecBakedDependencySink *sink)
{
    // The rest description is one of them. A rest that varies with time
    // moves it whenever the time moves, and a drag on a rest channel of the
    // same chain moves it where no time did -- the two halves the schedule
    // asks about separately.
    sink->step->varyingInputs = sink->step->varyingInputs || solver.restsVary;
    sink->step->overrideInputs.insert(sink->step->overrideInputs.end(),
                                solver.restOverrides.begin(),
                                solver.restOverrides.end());
    NoteInput(solver.bend, sink);
    NoteInput(solver.upperOffset, sink);
    NoteInput(solver.lowerOffset, sink);
    NoteInput(solver.stretch, sink);
    NoteInput(solver.softness, sink);
    NoteInput(solver.pin,sink);NoteInput(solver.upperScale,sink);NoteInput(solver.lowerScale,sink);
    NoteInput(solver.softDistance,sink);NoteInput(solver.limbTwist,sink);
    NoteInput(solver.blendWeight, sink);
    NoteInput(solver.preserveVolume, sink);
    NoteInput(solver.midFollowWeight, sink);
    NoteInput(solver.roll, sink);
    NoteInput(solver.twist, sink);
    NoteInput(solver.minLengthRatio, sink);
    NoteInput(solver.twistTurns, sink);
    NoteInput(solver.ribbonSampleCount, sink);
    NoteInput(solver.ikSpace, sink);
    // The spline parameters the bake could not fold, which the solve re-reads
    // as a group rather than one input at a time.
    sink->step->varyingInputs = sink->step->varyingInputs || solver.splineParamsVary;
}

/// Every per-frame input one constraint's step reads.
void
NoteConstraintInputs(const RigExecBakedProgramImpl::Constraint &constraint,
                     RigExecBakedDependencySink *sink)
{
    NoteInput(constraint.enabled, sink);
    NoteInput(constraint.defaultWeight, sink);
    NoteInput(constraint.offset, sink);
    NoteInput(constraint.affectX, sink);
    NoteInput(constraint.affectY, sink);
    NoteInput(constraint.affectZ, sink);
    NoteInput(constraint.tX, sink);
    NoteInput(constraint.tY, sink);
    NoteInput(constraint.tZ, sink);
    NoteInput(constraint.rX, sink);
    NoteInput(constraint.rY, sink);
    NoteInput(constraint.rZ, sink);
    NoteInput(constraint.sX, sink);
    NoteInput(constraint.sY, sink);
    NoteInput(constraint.sZ, sink);
    NoteInput(constraint.aimVector, sink);
    NoteInput(constraint.upVector, sink);
    NoteInput(constraint.rotationOffset, sink);
    NoteInput(constraint.worldUpVector, sink);
    // The two SingleChainIK-only inputs. Bound only in RotatePlane mode, so
    // in every other mode these are the default-constructed inputs and note
    // nothing -- which is the same answer as not listing them, and a good
    // deal harder to forget.
    NoteInput(constraint.poleVector, sink);
    NoteInput(constraint.twistDegrees, sink);
    NoteInput(constraint.stretch,sink);
    // The authored source-weight, offset and pole-weight tables are NOT
    // noted here: they are not inputs the step reads at all. The prologue
    // reads them off the stage as epoch state (again whenever the stage edit
    // serial, the program stamp or a read that varies with the time moved;
    // ConstraintArrays) and compares them by value, and
    // `constraintArrayClusters` is what dirties this step when one moved
    // (RigExecBakedComputeClosure) -- the §7 source mechanism, not the
    // varying-input one.
}

// One binding-to-step association shared by declarations and cone classification.
void NoteStepInputs(const RigExecBakedProgramImpl &B, RigExecBakedDependencySink *sink)
{
    RigExecBakedStep &step = *sink->step;
    switch (step.kind) {
    case RigExecBakedStepKind::Solve:
        if (step.object < 0 || size_t(step.object) >= B.solvers.size()) break;
        NoteSolverInputs(B.solvers[size_t(step.object)], sink);
        break;
    case RigExecBakedStepKind::Constraint: {
        if (step.object < 0 || size_t(step.object) >= B.walkSteps.size()) break;
        // A commit step and its walk entry are the same index, and a
        // constraint entry names the constraint it commits.
        const RigExecBakedProgramImpl::WalkStep &walk =
            B.walkSteps[size_t(step.object)];
        if (!walk.solverBatch && walk.index >= 0 && size_t(walk.index) < B.constraints.size()) {
            const auto &constraint=B.constraints[size_t(walk.index)];
            NoteConstraintInputs(constraint,sink);
            if(constraint.arrays>=0) sink->step->reads.push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::ConstraintInputs,uint32_t(constraint.arrays)));
        }
        break;
    }
    case RigExecBakedStepKind::WeightPacket:
        if (step.object < 0 || size_t(step.object) >= B.weightObjects.size()) break;
        RigExecBakedNoteWeightInputs(
            B.weightObjects[size_t(step.object)], sink);
        break;
    case RigExecBakedStepKind::AvarInputs: {
        const auto note=[&](const auto &bindings,const std::vector<uint32_t> &begin) {
            const auto range=RigExecBakedAvarBindingRange(begin,step.object);
            for(size_t i=range.first;i<range.second;++i) NoteInput(bindings[i].input,sink);
        };
        note(B.avarBindings,B.avarBindingBegin);
        note(B.avarConstantBindings,B.avarConstantBindingBegin);
        break;
    }
    case RigExecBakedStepKind::ProviderRefresh:
        for(const auto &carry:B.providerRefreshes[size_t(step.object)].carries)
            for(int slot:carry.blockingSlots)NoteInput(B.ladders[size_t(slot)].parentSpace,sink);
        break;
    case RigExecBakedStepKind::ComposeSubtree: {
        if (step.object < 0 || size_t(step.object) >= B.composeGroups.size()) break;
        // A switched slot's active index, so that keying a space dirties
        // the compose group that reads it and only that one.
        const RigExecBakedComposeGroup &group =
            B.composeGroups[size_t(step.object)];
        for (int slot = group.begin; slot < group.end; ++slot) {
            const int ac=(size_t(slot)<B.autoClavicleBySlot.size()?B.autoClavicleBySlot[size_t(slot)]:-1);
            if(ac>=0)for(const auto &read:B.autoClavicles[size_t(ac)].scalars) {
                if(read.isFloat)NoteInput(read.narrow,sink);else NoteInput(read.wide,sink);
            }
            const int switchIndex =
                slot < 0 || size_t(slot) >= B.spaceSwitchBySlot.size()
                    ? -1 : B.spaceSwitchBySlot[size_t(slot)];
            if (switchIndex >= 0 && size_t(switchIndex) < B.spaceSwitches.size()) {
                const auto &sw = B.spaceSwitches[size_t(switchIndex)];
                if (sw.tokenIndex) NoteInput(sw.activeTokenInput, sink);
                else NoteInput(sw.activeInput, sink);
            }
        }
        break;
    }
    case RigExecBakedStepKind::PoseInterpolator: {
        if (step.object < 0 || size_t(step.object) >= B.poseInterpolators.size()) break;
        // The owning body resolves the current enable and numeric channels.
        const RigExecBakedProgramImpl::PoseInterpolator &interpolator =
            B.poseInterpolators[size_t(step.object)];
        NoteInput(interpolator.enabled, sink);
        for (const RigExecBakedInput<double> &value :
                 interpolator.valueInputs) {
            NoteInput(value, sink);
        }
        break;
    }
    default:
        break;
    }
}

}  // namespace

std::vector<char>
RigExecBakedResolvedReaders(const RigExecBakedProgramImpl &B)
{
    std::vector<char> resolved(B.steps.size(), 0);
    for (size_t i = 0; i < B.steps.size(); ++i) {
        if (B.steps[i].isHead) continue;
        RigExecBakedStep scratch;
        scratch.kind = B.steps[i].kind;
        scratch.object = B.steps[i].object;
        RigExecBakedDependencySink sink{&scratch};
        NoteStepInputs(B, &sink);
        resolved[i] = sink.resolvedReads;
    }
    return resolved;
}

void
RigExecBakedDeclareInputDependencies(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    for (RigExecBakedStep &step : B.steps) {
        if (step.isHead) {
            if (step.kind == RigExecBakedStepKind::WeightField) RigExecBakedDeclareWeightField(&B,&step);
            continue;
        }
        step.varyingInputs = false;
        step.overrideInputs.clear();
        step.readerWalks.clear();
        step.reads.erase(std::remove_if(step.reads.begin(),step.reads.end(),
            [](const auto &r) { return RigExecBakedIsHeadDomain(r.domain); }),step.reads.end());
        RigExecBakedDependencySink sink{&step};
        NoteStepInputs(B, &sink);
        if (step.kind == RigExecBakedStepKind::WeightField) {
            RigExecBakedDeclareWeightField(&B,&step);
            const auto &field = B.weightFields[size_t(step.object)];
            for (const auto &read : field.scalarReads)
                for (uint32_t leaf : read.leaves)
                    step.varyingInputs = step.varyingInputs || B.headLeaves[leaf].varying;
        }
        if (step.kind == RigExecBakedStepKind::Constraint) {
            const auto &walk = B.walkSteps[size_t(step.object)];
            if (!walk.solverBatch && walk.index >= 0) {
                const auto &constraint = B.constraints[size_t(walk.index)];
                if (constraint.weightField >= 0)
                    step.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::WeightField,constraint.weightField));
            }
        }
        std::sort(step.overrideInputs.begin(), step.overrideInputs.end());
        step.overrideInputs.erase(
            std::unique(step.overrideInputs.begin(),
                        step.overrideInputs.end()),
            step.overrideInputs.end());
    }
    // The property versions those inputs' walks, and the path leaves'
    // walks, can read; then the rests and ladders the bodies index.
    RigExecBakedDeclareHeadReads(&B);
    RigExecBakedDeclareRestReads(&B);
}

namespace {

// Bit for bit: a NaN that stays NaN has not moved, and -0 is not 0.
template <class T>
bool
_LeafSame(const T &a, const T &b)
{
    if constexpr (std::is_trivially_copyable_v<T>) {
        return std::memcmp(&a, &b, sizeof(T)) == 0;
    } else {
        return a == b;
    }
}

}  // namespace

void
RigExecBakedNumberLeaves(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    B.leaves = RigExecBakedLeafPools();
    B.leafRefs.clear();
    B.sourceBackedPaths.clear();
    B.leafOfOverride.assign(B.overridden.size(), -1);
    frozenDetail::_ForEachPatchableInput(B, [&B](auto &input) {
        using T = std::decay_t<decltype(input.constant)>;
        using Stored = typename RigExecBakedLeafTraits<T>::Stored;
        if (input.sourceBacked && input.head)
            B.sourceBackedPaths.insert(input.head.GetPath());
        RigExecBakedLeafPool<T> &pool = B.leaves.Of<T>();
        const uint32_t id = uint32_t(B.leafRefs.size());
        input.leaf = int(pool.value.size());
        pool.value.push_back(Stored(input.constant));
        pool.changed.push_back(0);
        pool.mustSample.push_back(0);
        pool.id.push_back(id);
        B.leafRefs.push_back(
            {RigExecBakedLeafTraits<T>::type, uint32_t(input.leaf)});
        if (input.overrideIndex >= 0 &&
            size_t(input.overrideIndex) < B.leafOfOverride.size()) {
            B.leafOfOverride[size_t(input.overrideIndex)] = int(id);
        }
    });
    // RigExecBakedRecordBind files every hop of a registered binding's walk
    // (its head alone when the walk is empty) under the binding's number,
    // so the leaf's paths are exactly those entries.
    B.leafByPath.clear();
    for (const auto &[path, indices] : B.overridableInputs) {
        std::vector<uint32_t> ids;
        for (const int index : indices) {
            if (index >= 0 && size_t(index) < B.leafOfOverride.size() &&
                B.leafOfOverride[size_t(index)] >= 0) {
                ids.push_back(uint32_t(B.leafOfOverride[size_t(index)]));
            }
        }
        if (!ids.empty()) {
            B.leafByPath.emplace_hint(B.leafByPath.end(), path,
                                      std::move(ids));
        }
    }
    // The path leaves after them, in program order: chain revisions and
    // derived targets chain by chain, then weight objects, then the skin
    // layouts. Each is filed under every path its read can reach, so a value
    // edit there or on its routed prim marks it (ApplyValueEdits).
    B.pathLeafRefs.clear();
    const auto number = [&B](RigExecBakedPathLeaves &leaves,
                             RigExecBakedPathLeafRef ref) {
        leaves.exactVersions.assign(leaves.decl.keys.size(),-1);
        leaves.exactRecordIndices.assign(leaves.decl.keys.size(),-1);
        leaves.exactValueTypes.assign(leaves.decl.keys.size(),-1);
        for (size_t k = 0; k < leaves.decl.keys.size(); ++k) {
            const auto &key=leaves.decl.keys[k];
            using F=RigExecRevisionLeafFlavour;
            if(ref.owner!=RigExecBakedPathLeafOwner::WeightOracle &&
               key.time==RigExecRevisionLeafTime::AtTime && key.flavour!=F::Raw) {
                for(size_t r=0;r<B.propertyRecords.size();++r) {
                    const auto &record=B.propertyRecords[r];
                    if(record.consumer!=key.path) continue;
                    leaves.exactVersions[k]=int(record.id);
                    leaves.exactRecordIndices[k]=int(r);
                    leaves.exactValueTypes[k]=int(B.propertyChains[record.chain].arm);
                    break;
                }
                if(leaves.exactVersions[k]<0) for(const auto &chain:B.propertyChains)
                    if(chain.target==key.path) {
                        leaves.exactVersions[k]=int(chain.versionBase+chain.revisions.size());
                        leaves.exactValueTypes[k]=int(chain.arm);
                        break;
                    }

            }
            ref.key = uint32_t(k);
            const uint32_t id =
                uint32_t(B.leafRefs.size() + B.pathLeafRefs.size());
            B.pathLeafRefs.push_back(ref);
            if (k < leaves.hops.size()) {
                for (const SdfPath &hop : leaves.hops[k]) {
                    B.leafByPath[hop].push_back(id);
                }
            }
        }
    };
    for (size_t c = 0; c < B.chains.size(); ++c) {
        RigExecBakedProgramImpl::GeomChain &chain = B.chains[c];
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            number(chain.revisions[r].leaves,
                   {RigExecBakedPathLeafOwner::Revision, uint32_t(c),
                    uint32_t(r), 0});
        }
        for (size_t d = 0; d < chain.derived.size(); ++d) {
            number(chain.derived[d].revision.leaves,
                   {RigExecBakedPathLeafOwner::Derived, uint32_t(c),
                    uint32_t(d), 0});
        }
    }
    for (size_t w = 0; w < B.weightObjects.size(); ++w) {
        number(B.weightObjects[w].pointLeaves,
               {RigExecBakedPathLeafOwner::Weight, uint32_t(w), 0, 0});
        number(B.weightObjects[w].oracleLeaves,
               {RigExecBakedPathLeafOwner::WeightOracle,uint32_t(w),0,0});
    }
    // The SkinTopology ops' layout leaves, chain by chain.
    for (size_t c = 0; c < B.chains.size(); ++c) {
        RigExecBakedProgramImpl::GeomChain &chain = B.chains[c];
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            number(chain.revisions[r].layoutLeaves,
                   {RigExecBakedPathLeafOwner::RevisionLayout, uint32_t(c),
                    uint32_t(r), 0});
        }
        for (size_t d = 0; d < chain.derived.size(); ++d) {
            number(chain.derived[d].revision.layoutLeaves,
                   {RigExecBakedPathLeafOwner::DerivedLayout, uint32_t(c),
                    uint32_t(d), 0});
        }
    }
    number(B.providerLeaves,{RigExecBakedPathLeafOwner::Provider,0,0,0});
    const auto declareExact=[&](const RigExecBakedPathLeaves &leaves,RigExecBakedStep *step) {
        // Every declared source opinion participates in the readiness-time
        // memo, including sites without a produced-value reader walk.
        // Numbering precedes this binding so IDs reference the final census.
        for(size_t i=0;i<B.pathLeafRefs.size();++i)
            if(RigExecBakedPathLeavesOf(B,B.pathLeafRefs[i])==&leaves) {
                const uint32_t id=uint32_t(B.leafRefs.size()+i);
                if(std::find(step->bindingLeaves.begin(),step->bindingLeaves.end(),id)==step->bindingLeaves.end())
                    step->bindingLeaves.push_back(id);
            }
        for(int version:leaves.exactVersions) if(version>=0)
            step->reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::PropertyResult,uint32_t(version)));
    };
    const auto directBindings=[&](RigExecBakedStep *step,const auto &visit) {
        visit([&](const auto &input) {
            using T=std::decay_t<decltype(input.constant)>;
            if(input.leaf<0) return;
            const uint32_t id=B.leaves.Of<T>().id[size_t(input.leaf)];
            if(std::find(step->bindingLeaves.begin(),step->bindingLeaves.end(),id)==step->bindingLeaves.end())
                step->bindingLeaves.push_back(id);
            if(input.walk>=0) {
                const auto binding=std::make_pair(RigExecBakedLeafTraits<T>::type,uint32_t(input.leaf));
                if(std::find(step->walkedBindingLeaves.begin(),step->walkedBindingLeaves.end(),binding)==step->walkedBindingLeaves.end())
                    step->walkedBindingLeaves.push_back(binding);
            }
        });
    };
    for(auto &step:B.steps) {
        // Direct bindings are numbered only now. Their current values must
        // reach both source candidates and the effective numerical memo.
        if(step.kind==RigExecBakedStepKind::Solve) {
            directBindings(&step,[&](const auto &fn) { frozenDetail::_VisitSolverInputs(B.solvers[size_t(step.object)],fn); });
        } else if(step.kind==RigExecBakedStepKind::Constraint) {
            const auto &walk=B.walkSteps[size_t(step.object)];
            if(!walk.solverBatch && walk.index>=0)
                directBindings(&step,[&](const auto &fn) { frozenDetail::_VisitConstraintInputs(B.constraints[size_t(walk.index)],fn); });
        } else if(step.kind==RigExecBakedStepKind::WeightPacket) {
            directBindings(&step,[&](const auto &fn) { frozenDetail::_VisitWeightInputs(B.weightObjects[size_t(step.object)],fn); });
        } else if(step.kind==RigExecBakedStepKind::PoseInterpolator) {
            directBindings(&step,[&](const auto &fn) { frozenDetail::_VisitInterpolatorInputs(B.poseInterpolators[size_t(step.object)],fn); });
        } else if(step.kind==RigExecBakedStepKind::ProviderRefresh) {
            directBindings(&step,[&](const auto &fn) {
                for(const auto &carry:B.providerRefreshes[size_t(step.object)].carries)
                    for(int slot:carry.blockingSlots)fn(B.ladders[size_t(slot)].parentSpace);
            });
            for(const auto &carry:B.providerRefreshes[size_t(step.object)].carries)for(int slot:carry.blockingSlots) {
                const int raw=B.providerParentRawLeaves[size_t(slot)];
                if(raw<0)continue;
                for(size_t i=0;i<B.pathLeafRefs.size();++i) {
                    const auto &ref=B.pathLeafRefs[i];
                    if(ref.owner==RigExecBakedPathLeafOwner::Provider && ref.key==uint32_t(raw)) {
                        const uint32_t id=uint32_t(B.leafRefs.size()+i);
                        if(std::find(step.bindingLeaves.begin(),step.bindingLeaves.end(),id)==step.bindingLeaves.end())step.bindingLeaves.push_back(id);
                        break;
                    }
                }
            }
        } else if(step.kind==RigExecBakedStepKind::ComposeSubtree) {
            const auto &group=B.composeGroups[size_t(step.object)];
            for(int slot=group.begin;slot<group.end;++slot) {
                const int sw=B.spaceSwitchBySlot[size_t(slot)];
                if(sw>=0) directBindings(&step,[&](const auto &fn) { frozenDetail::_VisitSpaceSwitchInputs(B.spaceSwitches[size_t(sw)],fn); });
                const int ac=(size_t(slot)<B.autoClavicleBySlot.size()?B.autoClavicleBySlot[size_t(slot)]:-1);
                if(ac>=0)directBindings(&step,[&](const auto &fn){frozenDetail::_VisitAutoClavicleInputs(B.autoClavicles[size_t(ac)],fn);});
            }
        }
        if(step.kind==RigExecBakedStepKind::RevisionStatic) {
            const auto &at=B.revisionIndex[size_t(step.object)];
            declareExact(B.chains[size_t(at.first)].revisions[size_t(at.second)].leaves,&step);
        } else if(step.kind==RigExecBakedStepKind::Derived) {
            const auto &at=B.derivedIndex[size_t(step.object)];
            declareExact(B.chains[size_t(at.first)].derived[size_t(at.second)].revision.leaves,&step);
        } else if(step.kind==RigExecBakedStepKind::SkinTopology) {
            const auto &at=B.revisionIndex[size_t(step.object)];
            declareExact(B.chains[size_t(at.first)].revisions[size_t(at.second)].layoutLeaves,&step);
        } else if(step.kind==RigExecBakedStepKind::WeightPacket)
            declareExact(B.weightObjects[size_t(step.object)].pointLeaves,&step);
    }
    // Oracle path leaves are allocated by field capture after the initial
    // census; bind field memo IDs only against this final numbered census.
    for (auto &step : B.steps) if (step.kind == RigExecBakedStepKind::WeightField) {
        step.bindingLeaves.clear();
        RigExecBakedDeclareWeightField(&B, &step);
    }
    // The head leaves after those, each under its own path: a head leaf
    // reads one attribute and follows no connection.
    const size_t headBase = B.leafRefs.size() + B.pathLeafRefs.size();
    for (size_t h = 0; h < B.headLeaves.size(); ++h) {
        B.leafByPath[B.headLeaves[h].path].push_back(uint32_t(headBase + h));
    }
}

void
RigExecBakedSampleLeaves(RigExecBakedProgramImpl *program, UsdTimeCode time,
                         bool all, RigExecBakedLeafPass pass)
{
    RigExecBakedProgramImpl &B = *program;
    const RigExecResolvedInputs &R = *B.resolvedInputs;
    const bool first = pass != RigExecBakedLeafPass::ChainRouted;
    // Rule 6, carried into `mustSample`: the leaves under a routed override
    // whose value moved (placed, changed or lifted).
    if (first && B.routedOverrides != B.lastRoutedOverrides) {
        const auto markPath = [&B](const SdfPath &path) {
            const auto found = B.leafByPath.find(path);
            if (found != B.leafByPath.end()) {
                for (const uint32_t id : found->second) {
                    RigExecBakedMarkLeaf(&B, id);
                }
            }
        };
        for (const auto &[path, value] : B.routedOverrides) {
            const auto last = B.lastRoutedOverrides.find(path);
            if (last == B.lastRoutedOverrides.end() ||
                !(last->second == value)) {
                markPath(path);
            }
        }
        for (const auto &[path, value] : B.lastRoutedOverrides) {
            if (!B.routedOverrides.count(path)) {
                markPath(path);
            }
        }
        B.lastRoutedOverrides = B.routedOverrides;
    }
    // Rules 1 and 2: the caller's `all`, the first run, a moved stamp.
    all = all || !B.everRan || B.programStamp != B.lastProgramStamp;
    const bool timeMoved = time != B.lastTime;
    const bool edited = B.anyEdited;
    const auto flagged = [](const std::vector<char> &flags, int index) {
        return index >= 0 && size_t(index) < flags.size() &&
               flags[size_t(index)];
    };
    if (first) {
        B.leaves.ForEach([](auto &pool) {
            std::fill(pool.changed.begin(), pool.changed.end(), char(0));
        });
    }
    frozenDetail::_ForEachPatchableInput(B, [&](auto &input) {
        using T = std::decay_t<decltype(input.constant)>;
        using Stored = typename RigExecBakedLeafTraits<T>::Stored;
        if (input.leaf < 0) {
            return;
        }
        // Bound walks are resolved by their consumer against current graph
        // values. Their source heads are sampled separately as raw leaves.
        if(input.walk>=0) return;
        const bool routed = bool(input.resolvedAttr);
        if ((pass == RigExecBakedLeafPass::BeforeHead && routed) ||
            (pass == RigExecBakedLeafPass::ChainRouted && !routed)) {
            return;
        }
        RigExecBakedLeafPool<T> &pool = B.leaves.Of<T>();
        const size_t k = size_t(input.leaf);
        const int o = input.overrideIndex;
        const bool walked = input.walk >= 0;
        // `all` first: a frozen job's handles are dead.
        const bool resample =
            all || pool.mustSample[k] || (timeMoved && input.varying) ||
            flagged(B.overridden, o) || flagged(B.lastOverridden, o) ||
            (edited && flagged(B.edited, o)) ||
            (walked && B.readerWalkMoved[size_t(input.walk)]);
        if (!resample) {
            return;
        }
        pool.mustSample[k] = 0;
        ++B.leafSamples;
        // An upstream value on a hop reads the long way through the layer;
        // rule 8 marked the leaf when it was placed, moved or lifted.
        const Stored value =
            walked ? Stored(RigExecBakedReadWalked(B, input, B.overridden,
                                                   &B.upstreamOn))
                   : Stored(RigExecBakedRead(input, R, time, &B.overridden,
                                             &B.upstream, &B.upstreamOn));
        pool.changed[k] = _LeafSame(value, pool.value[k]) ? 0 : 1;
        pool.value[k] = value;
        if (walked && pool.changed[k]) {
            B.readerWalkChanged[size_t(input.walk)] = 1;
        }
    });
}

namespace {
bool _ProviderActive(const RigExecBakedProgramImpl &B,size_t slot)
{
    // Isolated hand-built kernel fixtures may have no captured scene. Actual
    // source-built programs always retain the complete activity table.
    return B.providerActive.empty() ||
           (slot<B.providerActive.size() && B.providerActive[slot]);
}
GfMatrix4d _UnavailableProviderSpace()
{
    GfMatrix4d value(1.0);
    value[3][0]=std::numeric_limits<double>::quiet_NaN();
    return value;
}
bool _SpaceReady(const RigExecBakedProgramImpl &B,const RigExecBakedProgramImpl::Ladder &L,size_t channel)
{
    const int id=L.spaceValues[channel];
    return id>=0 && B.providerValues.Read<GfMatrix4d>(RigExecValueId(id));
}
GfMatrix4d _SpaceMatrix(const RigExecBakedProgramImpl &B,const RigExecBakedProgramImpl::Ladder &L,
    size_t channel,const GfMatrix4d &fallback)
{
    const int id=L.spaceValues[channel];
    const auto *value=id>=0?B.providerValues.Read<GfMatrix4d>(RigExecValueId(id)):nullptr;
    return value?*value:fallback;
}
}

void
RigExecBakedComposeRestRange(RigExecBakedProgramImpl *program, int begin,
                             int end, bool trackMoves)
{
    RigExecBakedProgramImpl &B = *program;
    const GfMatrix4d identity(1.0);
    // The channels as sampled for this run (RigExecBakedSampleLeaves ran
    // first, at Build and in the prologue).
    const auto rd = [&B](const auto &input) {
        return RigExecBakedLeafRead(B, input);
    };
    for (int i = begin; i < end; ++i) {
        const size_t slot = size_t(i);
        if (B.slotKind[slot] != RigExecBakedSlotKind::FirstFramePose) {
            // An xform-derived slot has no rest chain: the dynamic path
            // gives it the identity rest frame outright. Build set it and
            // nothing here may move it -- including the round trip, which
            // stays the identity a descendant of one would inherit.
            continue;
        }
        if (B.execCheckRows) B.execCheckRows->BeforeSlot(
            B, RigExecBakedStepKind::RestCompose, begin, i);
        if(!_ProviderActive(B,slot)) {
            RigExecPointFrame unavailableFrame;
            unavailableFrame.flags=0;
            const auto unavailable=_UnavailableProviderSpace();
            bool changed=!RigExecTypedSame(B.restFrames[slot],unavailableFrame) ||
                !RigExecTypedSame(B.restM[slot],unavailable) ||
                !RigExecTypedSame(B.restRoundTrip[slot],unavailable);
            for(size_t k=0;k<unavailableFrame.points.size();++k)
                changed=changed || !RigExecTypedSame(B.restPts[slot][k],unavailableFrame.points[k]);
            B.restFrames[slot]=unavailableFrame;
            B.restPts[slot]=unavailableFrame.points;
            B.restM[slot]=unavailable;
            B.restRoundTrip[slot]=unavailable;
            if(trackMoves && changed) B.restChanged[slot]=1;
            continue;
        }
        const RigExecBakedProgramImpl::Ladder &L = B.ladders[slot];
        GfMatrix4d rest =
            RigExecBakedComposeAvars(rd(L.restAvars[0]), rd(L.restAvars[1]),
                                     rd(L.restAvars[2]), 1, 1, 1,
                                     rd(L.restAvars[3]), rd(L.restAvars[4]),
                                     rd(L.restAvars[5]), 0, B.xyzToken) *
            _SpaceMatrix(B,L,0,rd(L.restSpace));
        rest.Orthonormalize(/* issueWarning = */ false);
        const int parent = B.parent[slot];
        const GfMatrix4d parentRest =
            parent >= 0 ? B.restRoundTrip[size_t(parent)] : identity;
        const GfMatrix4d intervening = rd(L.interveningSpace);
        if (intervening != identity) rest = rest * intervening;
        B.restM[slot] = rest * parentRest;
        B.restFrames[slot] = RigExecFrameFromMatrix(B.restM[slot]);
        B.restPts[slot] = B.restFrames[slot].points;
        B.restRoundTrip[slot] = RigExecBakedRoundTrip(B.restM[slot]);

        // Compared by VALUE: a recompose that landed on the same numbers
        // moved nothing, and the readers of the rest may sit it out.
        // `restPts`, `restFrames` and `restRoundTrip` are functions of
        // `restM`, so it stands for all four.
        if (trackMoves && B.restM[slot] != B.lastRestM[slot]) {
            B.lastRestM[slot] = B.restM[slot];
            if (!B.restChanged[slot]) {
                B.restChanged[slot] = 1;

            }
        }
    }
}

void
RigExecBakedComposeLadderRange(RigExecBakedProgramImpl *program, int begin,
                               int end, bool trackMoves)
{
    RigExecBakedProgramImpl &B = *program;
    const GfMatrix4d identity(1.0);
    const auto rd = [&B](const auto &input) {
        return RigExecBakedLeafRead(B, input);
    };
    for (int i = begin; i < end; ++i) {
        const size_t slot = size_t(i);
        if (B.slotKind[slot] != RigExecBakedSlotKind::FirstFramePose) {
            // No default-space ladder either; see the rest above.
            continue;
        }
        if (B.execCheckRows) B.execCheckRows->BeforeSlot(
            B, RigExecBakedStepKind::LadderCompose, begin, i);
        if(!_ProviderActive(B,slot)) {
            const auto unavailable=_UnavailableProviderSpace();
            const bool changed=!RigExecTypedSame(B.selfD[slot],unavailable) ||
                !RigExecTypedSame(B.parentDinv[slot],unavailable) ||
                !RigExecTypedSame(B.restRoundTrip[slot],unavailable) ||
                !RigExecTypedSame(B.defaultRoundTrip[slot],unavailable) ||
                !RigExecTypedSame(B.posedAuthoredM[slot],unavailable) ||
                !RigExecTypedSame(B.posedD[slot],unavailable) ||
                !RigExecTypedSame(B.parentSpaceM[slot],unavailable) ||
                B.posedAuthored[slot]!=0 || B.parentSpaceAuthored[slot]!=0 ||
                B.rotOrder[slot]!=B.xyzToken || B.rotationSign[slot]!=0;
            B.selfD[slot]=B.parentDinv[slot]=B.restRoundTrip[slot]=B.defaultRoundTrip[slot]=unavailable;
            B.posedAuthoredM[slot]=B.posedD[slot]=B.parentSpaceM[slot]=unavailable;
            B.posedAuthored[slot]=B.parentSpaceAuthored[slot]=0;
            B.rotOrder[slot]=B.xyzToken;
            B.rotationSign[slot]=0;
            if(trackMoves && changed) B.ladderChanged[slot]=1;
            continue;
        }
        const RigExecBakedProgramImpl::Ladder &L = B.ladders[slot];
        const auto authoritative = [&B](const auto &input, bool connected) {
            const int index = input.overrideIndex;
            return connected || (index >= 0 &&
                ((size_t(index) < B.overridden.size() && B.overridden[size_t(index)]) ||
                 (size_t(index) < B.upstreamOn.size() && B.upstreamOn[size_t(index)])));
        };
        const GfMatrix4d posed = _SpaceMatrix(B,L,2,rd(L.posedSpace));
        B.posedAuthored[slot] = authoritative(L.posedSpace, L.posedSpaceConnected) || posed != identity;
        B.posedAuthoredM[slot] = posed;
        const int parent = B.parent[slot];
        const GfMatrix4d parentRest = parent >= 0 ? B.restRoundTrip[size_t(parent)] : identity;
        const GfMatrix4d authoredParentDefault = _SpaceMatrix(B,L,4,rd(L.parentDefaultSpace));
        const GfMatrix4d parentDefault =
            _SpaceReady(B,L,4) || authoritative(L.parentDefaultSpace, L.parentDefaultSpaceConnected) || authoredParentDefault != identity
                ? authoredParentDefault
                : (parent >= 0 ? B.defaultRoundTrip[size_t(parent)] : identity);
        const GfMatrix4d authoredDefault = _SpaceMatrix(B,L,1,rd(L.defaultSpace));
        if (_SpaceReady(B,L,1) || authoritative(L.defaultSpace, L.defaultSpaceConnected) || authoredDefault != identity) {
            B.selfD[slot] = authoredDefault;
        } else {
            const GfMatrix4d offset = RigExecBakedComposeAvars(
                rd(L.defaultAvars[0]), rd(L.defaultAvars[1]),
                rd(L.defaultAvars[2]), 1, 1, 1, rd(L.defaultAvars[3]),
                rd(L.defaultAvars[4]), rd(L.defaultAvars[5]), 0, B.xyzToken);
            B.selfD[slot] = offset * B.restRoundTrip[slot] * parentRest.GetInverse() * parentDefault;
        }
        B.defaultRoundTrip[slot] = RigExecBakedRoundTrip(B.selfD[slot]);
        const GfMatrix4d authoredAvarDefault = _SpaceMatrix(B,L,5,rd(L.avarDefaultSpace));
        const GfMatrix4d avarDefault =
            _SpaceReady(B,L,5) || authoritative(L.avarDefaultSpace, L.avarDefaultSpaceConnected) || authoredAvarDefault != identity
                ? authoredAvarDefault : B.selfD[slot];
        const GfMatrix4d authoredPosedDefault = _SpaceMatrix(B,L,6,rd(L.posedDefaultSpace));
        B.posedD[slot] =
            _SpaceReady(B,L,6) || authoritative(L.posedDefaultSpace, L.posedDefaultSpaceConnected) || authoredPosedDefault != identity
                ? authoredPosedDefault : avarDefault;
        B.parentDinv[slot] = parentDefault.GetInverse();
        B.parentSpaceM[slot] = _SpaceMatrix(B,L,3,rd(L.parentSpace));
        B.parentSpaceAuthored[slot] = authoritative(L.parentSpace, L.parentSpaceConnected) || rd(L.parentSpace) != identity;
        const GfVec3d sign = rd(L.rotationSign);
        B.rotationSign[slot] = RigExecRotationSignMask(sign[0], sign[1], sign[2]);
        const TfToken order = rd(L.rotationOrder);
        B.rotOrder[slot] = order.IsEmpty() ? B.xyzToken : order;

        // By value, as the rest is; `defaultRoundTrip` is a function of
        // `selfD`.
        if (trackMoves &&
            (B.selfD[slot] != B.lastSelfD[slot] ||
             B.parentDinv[slot] != B.lastParentDinv[slot] ||
             B.posedAuthored[slot] != B.lastPosedAuthored[slot] ||
             B.posedAuthoredM[slot] != B.lastPosedAuthoredM[slot] ||
             B.rotOrder[slot] != B.lastRotOrder[slot] ||
             B.posedD[slot] != B.lastPosedD[slot] ||
             B.parentSpaceM[slot] != B.lastParentSpaceM[slot] ||
             B.parentSpaceAuthored[slot] != B.lastParentSpaceAuthored[slot] ||
             B.rotationSign[slot] != B.lastRotationSign[slot])) {
            B.lastSelfD[slot] = B.selfD[slot];
            B.lastParentDinv[slot] = B.parentDinv[slot];
            B.lastPosedAuthored[slot] = B.posedAuthored[slot];
            B.lastPosedAuthoredM[slot] = B.posedAuthoredM[slot];
            B.lastRotOrder[slot] = B.rotOrder[slot];
            B.lastPosedD[slot] = B.posedD[slot];
            B.lastParentSpaceM[slot] = B.parentSpaceM[slot];
            B.lastParentSpaceAuthored[slot] = B.parentSpaceAuthored[slot];
            B.lastRotationSign[slot] = B.rotationSign[slot];
            if (!B.ladderChanged[slot]) {
                B.ladderChanged[slot] = 1;

            }
        }
    }
}

namespace {

// The ladder channels a RestCompose (\p rest) or LadderCompose reads for
// one slot, in a fixed order.
template <class Fn>
void
_ForEachLadderChannel(const RigExecBakedProgramImpl::Ladder &L, bool rest,
                      Fn &&fn)
{
    if (rest) {
        for (const RigExecBakedInput<double> &input : L.restAvars) {
            fn(input);
        }
        fn(L.restSpace);
        fn(L.interveningSpace);
    } else {
        fn(L.posedSpace);
        fn(L.defaultSpace);
        fn(L.parentSpace);
        fn(L.parentDefaultSpace);
        fn(L.avarDefaultSpace);
        fn(L.posedDefaultSpace);
        fn(L.rotationSign);
        for (const RigExecBakedInput<double> &input : L.defaultAvars) {
            fn(input);
        }
        fn(L.rotationOrder);
    }
}

}  // namespace

void
RigExecBakedBuildRestSteps(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    const auto ranges = [](RigExecBakedSlotDomain domain,
                           const std::set<uint32_t> &slots,
                           std::vector<RigExecBakedSlotRange> *out) {
        for (const uint32_t slot : slots) {
            if (!out->empty() && out->back().domain == domain &&
                out->back().end == slot) {
                ++out->back().end;
            } else {
                out->push_back(RigExecBakedOne(domain, slot));
            }
        }
    };
    for (size_t g = 0; g < B.composeGroups.size(); ++g) {
        const RigExecBakedComposeGroup &group = B.composeGroups[g];
        const SdfPath &first = B.paths[size_t(group.begin)];
        for (const bool rest : {true, false}) {
            RigExecBakedStep step;
            step.isHead = true;
            step.kind = rest ? RigExecBakedStepKind::RestCompose
                             : RigExecBakedStepKind::LadderCompose;
            step.object = int(g);
            step.part = rest ? 0 : 1;
            step.label =
                std::string(rest ? "RestCompose " : "LadderCompose ") +
                first.GetString();
            std::set<uint32_t> versions, rests, ladders, spaces;
            for (int i = group.begin; i < group.end; ++i) {
                const size_t slot = size_t(i);
                if (B.slotKind[slot] != RigExecBakedSlotKind::FirstFramePose) {
                    continue;
                }
                const auto &L=B.ladders[slot];
                for(size_t channel=0;channel<7;++channel) {
                    if((rest && channel!=0) || (!rest && channel==0)) continue;
                    if(L.spaceValues[channel]>=0) spaces.insert(uint32_t(L.spaceValues[channel]));
                }
                const int parent = B.parent[slot];
                if (parent >= 0 &&
                    (parent < group.begin || parent >= group.end)) {
                    rests.insert(uint32_t(parent));
                    if (!rest) {
                        ladders.insert(uint32_t(parent));
                    }
                }
                _ForEachLadderChannel(
                    B.ladders[slot], rest, [&](const auto &input) {
                        if (input.walk >= 0) {
                            const RigExecBakedReaderWalk &reader =
                                B.readerWalks[size_t(input.walk)];
                            versions.insert(reader.versions.begin(),
                                            reader.versions.end());
                        }
                    });
            }
            if (!rest) {
                // The ladder reads the rests of its own slots.
                for (int i = group.begin; i < group.end; ++i) {
                    rests.insert(uint32_t(i));
                }
            }
            ranges(RigExecBakedSlotDomain::SpaceValue,spaces,&step.reads);
            ranges(RigExecBakedSlotDomain::PropertyResult, versions,
                   &step.reads);
            ranges(RigExecBakedSlotDomain::Rest, rests, &step.reads);
            ranges(RigExecBakedSlotDomain::Ladder, ladders, &step.reads);
            step.writes.push_back(RigExecBakedSlotRange{
                rest ? RigExecBakedSlotDomain::Rest
                     : RigExecBakedSlotDomain::Ladder,
                uint32_t(group.begin), uint32_t(group.end)});
            B.steps.push_back(std::move(step));
        }
    }
    const size_t N = B.paths.size();
    B.restChanged.assign(N, 0);
    B.ladderChanged.assign(N, 0);
    B.restMoved.clear();
    B.ladderMoved.clear();
}

void
RigExecBakedNoteRestLeaves(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    for (RigExecBakedStep &step : B.steps) {
        if (step.kind != RigExecBakedStepKind::RestCompose &&
            step.kind != RigExecBakedStepKind::LadderCompose) {
            continue;
        }
        step.bindingLeaves.clear();
        step.varyingLeaves = false;
        const RigExecBakedComposeGroup &group =
            B.composeGroups[size_t(step.object)];
        const bool rest = step.kind == RigExecBakedStepKind::RestCompose;
        for (int i = group.begin; i < group.end; ++i) {
            const size_t slot = size_t(i);
            if (B.slotKind[slot] != RigExecBakedSlotKind::FirstFramePose) {
                continue;
            }
            _ForEachLadderChannel(
                B.ladders[slot], rest, [&](const auto &input) {
                    using T = std::decay_t<decltype(input.constant)>;
                    if (input.leaf < 0) {
                        return;
                    }
                    step.bindingLeaves.push_back(
                        B.leaves.Of<T>().id[size_t(input.leaf)]);
                    step.varyingLeaves = step.varyingLeaves || input.varying;
                });
        }
    }
    // A notice between Build and the first run moves the stamp past this.
}

std::vector<RigExecBakedSlotRange>
RigExecBakedRequiredRestReads(const RigExecBakedProgramImpl &B,
                              const RigExecBakedStep &source)
{
    RigExecBakedStep step = source;
    step.reads.clear();
    const int N = int(B.paths.size());
    {
        std::set<uint32_t> rests, ladders;
        // Plain Xform slots have immutable identity rest/default tables.
        // Their sampled pose is a separate PoseBase/PoseFin value.
        const auto rest = [&B, &rests, N](int slot) {
            if (slot >= 0 && slot < N &&
                B.slotKind[size_t(slot)] == RigExecBakedSlotKind::FirstFramePose) {
                rests.insert(uint32_t(slot));
            }
        };
        const auto ladder = [&B, &ladders, N](int slot) {
            if (slot >= 0 && slot < N &&
                B.slotKind[size_t(slot)] == RigExecBakedSlotKind::FirstFramePose) {
                ladders.insert(uint32_t(slot));
            }
        };
        const auto version =
            [&ladder](
                const RigExecBakedProgramImpl::SpaceSwitch::FrameVersion
                    &read) {
                for (const int slot : read.recompose) {
                    ladder(slot);
                }
            };
        switch (step.kind) {
        case RigExecBakedStepKind::ComposeSubtree: {
            if (step.object < 0 || size_t(step.object) >= B.composeGroups.size()) break;
            const RigExecBakedComposeGroup &group =
                B.composeGroups[size_t(step.object)];
            for (int slot = std::max(0, group.begin); slot < std::min(N, group.end); ++slot) {
                ladder(slot);
            }
            for (const auto &ac : B.autoClavicles) {
                if (ac.slot < group.begin || ac.slot >= group.end) continue;
                for (const auto &read : ac.frames) {
                    if (read.computation == "computeRestFrame") rest(read.slot);
                    if (read.computation == "computeDefaultFrame") ladder(read.slot);
                    for (int child : read.recompose) ladder(child);
                }
            }
            for (const int parent : group.parentSlots) {
                ladder(parent);
            }
            for (int slot = std::max(0, group.begin); slot < std::min(N, group.end); ++slot) {
                const int switchIndex =
                    size_t(slot) >= B.spaceSwitchBySlot.size()
                        ? -1 : B.spaceSwitchBySlot[size_t(slot)];
                if (switchIndex < 0 || size_t(switchIndex) >= B.spaceSwitches.size()) {
                    continue;
                }
                const RigExecBakedProgramImpl::SpaceSwitch &sw =
                    B.spaceSwitches[size_t(switchIndex)];
                if (size_t(slot) < B.parent.size()) ladder(B.parent[size_t(slot)]);
                ladder(sw.spaceSlot);
                for (const int source : sw.sourceSlots) {
                    ladder(source);
                }
                version(sw.parentRead);
                version(sw.spaceRead);
                for (const auto &read : sw.sourceReads) {
                    version(read);
                }
            }
            break;
        }
        case RigExecBakedStepKind::SpaceCheckpoint:
            if (step.object < 0 || size_t(step.object) >= B.switchFrameContexts.size()) break;
            for (const int slot : B.switchFrameContexts[size_t(step.object)].recompose)
                ladder(slot);
            break;
        case RigExecBakedStepKind::Solve:
            if (step.object < 0 || size_t(step.object) >= B.solvers.size()) break;
            for (const int slot : B.solvers[size_t(step.object)].restSlots) {
                rest(slot);
            }
            break;
        case RigExecBakedStepKind::Constraint: {
            if (step.object < 0 || size_t(step.object) >= B.walkSteps.size()) break;
            const RigExecBakedProgramImpl::WalkStep &walk =
                B.walkSteps[size_t(step.object)];
            if (walk.solverBatch || walk.index < 0 || size_t(walk.index) >= B.constraints.size()) {
                break;
            }
            const RigExecBakedProgramImpl::Constraint &c =
                B.constraints[size_t(walk.index)];
            if (!c.useAnimatedTs) {
                for (const int slot : c.targetSlots) {
                    rest(slot);
                }
            }
            if (c.type == "RigExecMatrixMover") {
                for (const int slot : c.sources) {
                    rest(slot);
                }
            }
            ladder(c.spaceSlot);
            break;
        }
        case RigExecBakedStepKind::PropagateChunk: {
            if(step.object<0 || size_t(step.object)>=B.commits.size()) break;
            const auto &commit=B.commits[size_t(step.object)];
            for(const auto &pair:commit.propagate)
                for(int slot=pair.first;slot>=0 && slot!=pair.second;slot=B.propParent[size_t(slot)]) ladder(slot);
            break;
        }
        case RigExecBakedStepKind::ProviderMatrix:
            rest(step.object);
            break;
        case RigExecBakedStepKind::FrameMatrix:
            if (step.object < 0 || size_t(step.object) >= B.frameRecords.size()) break;
            rest(B.frameRecords[size_t(step.object)].slot);
            break;
        case RigExecBakedStepKind::PoseInterpolator: {
            if (step.object < 0 || size_t(step.object) >= B.poseInterpolators.size()) break;
            const RigExecBakedProgramImpl::PoseInterpolator &interpolator =
                B.poseInterpolators[size_t(step.object)];
            if (interpolator.valueInputs.empty()) {
                rest(interpolator.driverSlot);
                rest(interpolator.parentSlot);
            }
            break;
        }
        case RigExecBakedStepKind::Derived: {
            if (step.object < 0 || size_t(step.object) >= B.derivedIndex.size()) break;
            const auto &[c, d] = B.derivedIndex[size_t(step.object)];
            if (c < 0 || size_t(c) >= B.chains.size() ||
                d < 0 || size_t(d) >= B.chains[size_t(c)].derived.size()) break;
            const RigExecBakedProgramImpl::GeomRevision &revision =
                B.chains[size_t(c)].derived[size_t(d)].revision;
            rest(revision.transformSlot);
            rest(revision.transformSpaceSlot);
            rest(revision.carrySpaceSlot);
            break;
        }
        default:
            break;
        }
        for (const auto &[domain, slots] :
             {std::make_pair(RigExecBakedSlotDomain::Rest, &rests),
              std::make_pair(RigExecBakedSlotDomain::Ladder, &ladders)}) {
            for (const uint32_t slot : *slots) {
                if (!step.reads.empty() &&
                    step.reads.back().domain == domain &&
                    step.reads.back().end == slot) {
                    ++step.reads.back().end;
                } else {
                    step.reads.push_back(
                        RigExecBakedOne(domain, slot));
                }
            }
        }
    }
    return step.reads;
}

void
RigExecBakedDeclareRestReads(RigExecBakedProgramImpl *program)
{
    for (auto &step : program->steps) {
        if (step.isHead) continue;
        const auto reads = RigExecBakedRequiredRestReads(*program,step);
        step.reads.insert(step.reads.end(),reads.begin(),reads.end());
    }
}

RigExecBakedProgramTesting::LadderTables
RigExecBakedProgramTesting::LadderTablesOf(const RigExecBakedProgram &program)
{
    const RigExecBakedProgramImpl &B = *program._impl;
    LadderTables T;
    T.restM = B.restM;
    T.restPts = B.restPts;
    T.restFrames = B.restFrames;
    T.selfD = B.selfD;
    T.parentDinv = B.parentDinv;
    T.rotOrder = B.rotOrder;
    T.restRoundTrip = B.restRoundTrip;
    T.defaultRoundTrip = B.defaultRoundTrip;
    T.posedAuthored = B.posedAuthored;
    T.posedAuthoredM = B.posedAuthoredM;
    return T;
}

void
RigExecBakedBuildAvarSteps(RigExecBakedProgramImpl *program)
{
    auto &B=*program;
    for(size_t slot=0;slot<B.paths.size();++slot) {
        RigExecBakedStep step; step.kind=RigExecBakedStepKind::AvarInputs; step.object=int(slot);
        step.label="AvarInputs "+B.paths[slot].GetString();
        step.writes.push_back(RigExecBakedOne(RigExecBakedSlotDomain::Avars,slot));
        B.steps.push_back(std::move(step));
    }
}
void
RigExecBakedIndexAvarBindings(RigExecBakedProgramImpl *program)
{
    auto &B=*program;
    size_t providers=B.avarConstants.size()/11;
    for(const auto *bindings:{&B.avarBindings,&B.avarConstantBindings})
        for(const auto &binding:*bindings) providers=std::max(providers,binding.slot/11+1);
    const auto index=[providers](const std::vector<RigExecBakedProgramImpl::AvarBinding> &bindings,
                                 std::vector<uint32_t> *begin) {
        // Contiguous runs per provider need the list sorted by provider;
        // ascending flat slots give that, and Build appends them so.
        TF_VERIFY(std::is_sorted(bindings.begin(),bindings.end(),
            [](const auto &a,const auto &b){return a.slot<b.slot;}));
        begin->assign(providers+1,0);
        for(const auto &binding:bindings) ++(*begin)[binding.slot/11+1];
        for(size_t p=0;p<providers;++p) (*begin)[p+1]+=(*begin)[p];
    };
    index(B.avarBindings,&B.avarBindingBegin);
    index(B.avarConstantBindings,&B.avarConstantBindingBegin);
}
void
RigExecBakedRunAvarOp(RigExecBakedProgramImpl *program,RigExecBakedStep *step)
{
    auto &B=*program;
    const auto read=[&](const auto &bindings,const std::vector<uint32_t> &begin) {
        const auto range=RigExecBakedAvarBindingRange(begin,step->object);
        for(size_t i=range.first;i<range.second;++i) {
            const auto &binding=bindings[i];
            B.avars[binding.slot]=RigExecBakedLeafRead(B,binding.input);
        }
    };
    read(B.avarBindings,B.avarBindingBegin);
    read(B.avarConstantBindings,B.avarConstantBindingBegin);
}

void
RigExecBakedRunSolverSources(RigExecBakedProgramImpl *program,
                             UsdTimeCode time)
{
    RigExecBakedProgramImpl &B = *program;
    for (RigExecBakedProgramImpl::Solver &s : B.solvers) {
        if (!s.ribbonPointsVarying || !s.ribbonPointsQuery.IsValid()) {
            continue;
        }
        // The dynamic path's read, verbatim: the driver attribute's value at
        // this generation's time, with a failed read leaving the array as it
        // found it -- which is empty, the way the dynamic path's freshly
        // constructed VtVec3fArray is. The pinned query answers the same
        // value the attribute does; only the resolve is cached.
        VtVec3fArray live;
        s.ribbonPointsQuery.Get(&live, time);
        s.lastRibbonPoints.swap(s.ribbonPoints);
        s.ribbonPoints.assign(live.begin(), live.end());
        // By VALUE, never "the time moved": a curve keyed on two frames
        // holds the same points across most of a sweep, and comparing is
        // what lets the ribbon's whole cone sit out those frames.
        s.ribbonPointsDirty = s.ribbonPoints != s.lastRibbonPoints;
    }
}

namespace {

/// The hierarchy delta each candidate of \p commit carries, once.
///
/// Today's loop computes this inside the descendant walk; it depends only on
/// the candidate's frame and the ancestor's frame before the commit, neither
/// of which the walk touches, so hoisting it changes no number and lets the
/// descendants be staged in any order.
void
ComputeCommitDeltas(const RigExecBakedProgramImpl &B,
                    RigExecBakedCommit *commit)
{
    for (size_t pos = 0; pos < commit->slots.size(); ++pos) {
        if (!commit->present[pos]) {
            commit->deltaOk[pos] = 0;
            continue;
        }
        commit->deltas[pos] = GfMatrix4d(1.0);
        commit->deltaOk[pos] =
            RigExecPreparePoseDelta(
                B.fin[size_t(commit->slotReads[pos])],
                commit->frames[pos], &commit->deltas[pos])
                ? 1
                : 2;
    }
}

/// Stages descendants [\p begin, \p end) of \p commit.
///
/// Each pair is decided exactly where today's loop decides it and in the same
/// order of tests; what it does with the answer is recorded rather than
/// returned, because the pair that decides the commit is the LOWEST-indexed
/// failing one and a chunk cannot know whether an earlier chunk failed.
void
StageCommitPairs(const RigExecBakedProgramImpl &B, RigExecBakedCommit *commit,
                 size_t begin, size_t end)
{
    for (size_t k = begin; k < end; ++k) {
        const int pos = commit->closestPos[k];
        bool parentBlocked = false;
        for (int slot = commit->propagate[k].first;
             slot >= 0 && slot != commit->propagate[k].second;
             slot = B.propParent[size_t(slot)]) {
            if (B.parentSpaceAuthored[size_t(slot)]) { parentBlocked = true; break; }
        }
        if (parentBlocked || pos < 0 || !commit->present[size_t(pos)]) {
            // No candidate leaves this descendant unchanged.
            commit->outcome[k] =
                uint8_t(RigExecBakedPropagateOutcome::Skipped);
            continue;
        }
        const RigExecPointFrame &current =
            B.fin[size_t(commit->descendantReads[k])];
        const RigExecPointFrame &before =
            B.fin[size_t(commit->closestReads[k])];
        commit->outcome[k]=uint8_t(RigExecPropagatePoseFrame(current,before,
            commit->frames[size_t(pos)],commit->deltas[size_t(pos)],
            commit->deltaOk[size_t(pos)]==1,commit->solverOutput,false,true,&commit->staged[k]));
    }
}

/// Decides \p commit and writes it back, or says why it passed through.
///
/// The first pair that neither staged nor was stepped over decides, which is
/// where today's loop returns; everything after it was staged for nothing and
/// is dropped. A commit that passes through writes NOTHING -- not even its
/// candidates -- which is what makes it atomic.
void
FinishCommit(RigExecBakedProgramImpl *program, RigExecBakedStep *step,
             RigExecBakedCommit *commit)
{
    RigExecBakedProgramImpl &B = *program;
    // What a write site that does not write leaves behind: the version this
    // commit found, copied into the storage this commit owns, so that a
    // reader bound to the commit's version reads the value the one frame map
    // used to hold at this point in the walk (§3.1). Cheap where it matters
    // -- a commit that applies carries only the candidates its batch
    // published nothing for -- and paid in full only by a commit that passes
    // through, which is the arithmetic it did not do.
    const auto carry = [&B, commit](size_t pos, size_t k, bool candidates,
                                    bool descendants) {
        if (candidates) {
            B.fin[size_t(commit->slotWrites[pos])] =
                B.fin[size_t(commit->slotCarry[pos])];
            if (commit->solverOutput) {
                B.base[size_t(commit->slotBaseWrites[pos])] =
                    B.base[size_t(commit->slotBaseCarry[pos])];
            }
        }
        if (descendants) {
            B.fin[size_t(commit->descendantWrites[k])] =
                B.fin[size_t(commit->descendantCarry[k])];
            if (commit->solverOutput) {
                B.base[size_t(commit->descendantBaseWrites[k])] =
                    B.base[size_t(commit->descendantBaseCarry[k])];
            }
        }
    };
    const auto carryEverything = [&] {
        for (size_t pos = 0; pos < commit->slots.size(); ++pos) {
            carry(pos, 0, /* candidates = */ true, /* descendants = */ false);
        }
        for (size_t k = 0; k < commit->propagate.size(); ++k) {
            carry(0, k, /* candidates = */ false, /* descendants = */ true);
        }
    };
    if (commit->abandoned) {
        carryEverything();
        return;
    }
    // Spelled at Build: a body may not ask SdfPath for text.
    const std::string &mover = commit->moverPathText;
    for (size_t k = 0; k < commit->propagate.size(); ++k) {
        const auto outcome = RigExecBakedPropagateOutcome(commit->outcome[k]);
        if (outcome == RigExecBakedPropagateOutcome::Staged ||
            outcome == RigExecBakedPropagateOutcome::Skipped) {
            continue;
        }
        switch (outcome) {
        case RigExecBakedPropagateOutcome::UnusableDescendant:
            step->diagnostics.push_back(
                mover + " could not propagate its pose revision through " +
                (*B.pathTexts)[size_t(commit->propagate[k].first)] +
                "; constraint passed through");
            break;
        case RigExecBakedPropagateOutcome::SingularDelta:
            step->diagnostics.push_back(
                mover + " produced a singular hierarchy delta; constraint "
                "passed through");
            break;
        default:
            step->diagnostics.push_back(
                mover + " produced an invalid descendant frame for " +
                (*B.pathTexts)[size_t(commit->propagate[k].first)] +
                "; constraint passed through");
            break;
        }
        carryEverything();
        return;
    }
    // In slot order for the candidates -- which is the order the dense table
    // is in -- then in propagation order for the descendants, exactly as the
    // two write-back loops of commitConstraintFrames run.
    for (size_t pos = 0; pos < commit->slots.size(); ++pos) {
        if (!commit->present[pos]) {
            carry(pos, 0, /* candidates = */ true, /* descendants = */ false);
            continue;
        }
        B.fin[size_t(commit->slotWrites[pos])] = commit->frames[pos];
        if (commit->solverOutput) {
            B.base[size_t(commit->slotBaseWrites[pos])] = commit->frames[pos];
        }
    }
    for (size_t k = 0; k < commit->propagate.size(); ++k) {
        if (RigExecBakedPropagateOutcome(commit->outcome[k]) !=
            RigExecBakedPropagateOutcome::Staged) {
            carry(0, k, /* candidates = */ false, /* descendants = */ true);
            continue;
        }
        B.fin[size_t(commit->descendantWrites[k])] = commit->staged[k];
        if (commit->solverOutput) {
            B.base[size_t(commit->descendantBaseWrites[k])] =
                commit->staged[k];
        }
    }
}

/// Rebuilds one solver's rest description from the ladder this run composed.
///
/// Exec rebuilds every one of these from computeRestFrame on EVERY
/// evaluation; the bake resolves them once and this resolves them again,
/// from the same B.restPts/B.restFrames the bake read, so the two cannot
/// disagree about anything but which rests were in the arrays.
void
RefreshSolverRests(const RigExecBakedProgramImpl &B,
                   RigExecBakedProgramImpl::Solver *solver)
{
    RigExecBakedProgramImpl::Solver &s = *solver;
    // Where a rest ref is LIVE, the rest is the frame the pose step below this
    // solver left -- read off the ladder by the version bindSolverReads bound
    // -- and not the authored rest of the slot (spec 4.2). Every arithmetic
    // line below is written against these two accessors and needs no other
    // change.
    const auto liveRest = [&s, &B](size_t k) {
        return k < s.restIsLive.size() && s.restIsLive[k] &&
               k < s.restReads.size() &&
               size_t(s.restReads[k]) < B.fin.size();
    };
    const auto restPtsOf = [&s, &B, &liveRest](
                               size_t k, int slot) -> const
        std::array<GfVec3d, 4> & {
        return liveRest(k) ? B.fin[size_t(s.restReads[k])].points
                           : B.restPts[size_t(slot)];
    };
    const auto restFrameOf = [&s, &B, &liveRest](
                                 size_t k, int slot) -> const
        RigExecPointFrame & {
        return liveRest(k) ? B.fin[size_t(s.restReads[k])]
                           : B.restFrames[size_t(slot)];
    };
    for (size_t k = 0; k < s.restRefs.size() && k < s.jointRests.size();
         ++k) {
        s.jointRests[k] = restPtsOf(k, s.restRefs[k].first);
    }
    if (s.type == "RigExecFkChain") {
        for (size_t k = 0; k < s.controls.size(); ++k) {
            s.controlRests[k] = B.restPts[size_t(s.controls[k])];
        }
        if (s.start >= 0) {
            s.startRest = B.restPts[size_t(s.start)];
        }
    } else if (s.type == "RigExecTwoBoneIk") {
        for (size_t k = 0; k < s.restRefs.size(); ++k) {
            const int slot = s.restRefs[k].first;
            const int element = s.restRefs[k].second;
            if (element >= 0 && element < 3) {
                s.ikRests[size_t(element)] = restPtsOf(k, slot);
            }
        }
        s.upperLengthBase =
            (s.ikRests[1][0] - s.ikRests[0][0]).GetLength();
        s.lowerLengthBase =
            (s.ikRests[2][0] - s.ikRests[1][0]).GetLength();
        // The constant arm of the params below reads these two, so they are
        // written back into it here; the live arm recomputes them from the
        // same bases and agrees by construction.
        RigExecTwoBoneIkLengths(s.ikRests, s.ikSpace.constant,
                                s.upperOffset.constant,
                                s.lowerOffset.constant,
                                &s.ikParams.upperLength,
                                &s.ikParams.lowerLength);
        // The space target's computeRestFrame, read where Build read it.
        // Never live: a live ref is a joint rest, and Build reads the space
        // slot's rest without one.
        if (s.spaceSlot >= 0) {
            s.spaceRest = B.restPts[size_t(s.spaceSlot)];
        }
    } else if (s.type == "RigExecSplineIk") {
        std::vector<RigExecPointFrame> restJoints(s.splineCount);
        for (size_t k = 0; k < s.restRefs.size(); ++k) {
            const int slot = s.restRefs[k].first;
            const int element = s.restRefs[k].second;
            if (element >= 0 && size_t(element) < s.splineCount) {
                restJoints[size_t(element)] = restFrameOf(k, slot);
                s.splineJointRests[size_t(element)] = restPtsOf(k, slot);
            }
        }
        s.splineRestFrames = restJoints;
        s.splineRootRest =
            s.root >= 0 ? B.restFrames[size_t(s.root)] : RigExecPointFrame();
        s.splineMidRest =
            s.mid >= 0 ? B.restFrames[size_t(s.mid)] : RigExecPointFrame();
        s.splineEndRest =
            s.end >= 0 ? B.restFrames[size_t(s.end)] : RigExecPointFrame();
        s.splineRest = RigExecSplineIkMakeRest(
            restJoints, s.splineRootRest, s.splineMidRest, s.splineEndRest,
            s.splineRestWeights, s.splineRestMode);
        // As for the TwoBoneIk above.
        if (s.spaceSlot >= 0) {
            s.spaceRest = B.restPts[size_t(s.spaceSlot)];
        }
    } else if (s.type == "RigExecTwistDistribution") {
        if (s.root >= 0 && s.end >= 0) {
            s.twistStartRest = B.restPts[size_t(s.root)];
            s.twistEndRest = B.restPts[size_t(s.end)];
        }
    }
}

/// Slot \p i's avars as one matrix, the way computations.cpp composes them.
GfMatrix4d
_ComposeAvarsOf(const RigExecBakedProgramImpl &B, int i)
{
    const double *a = &B.avars[size_t(i) * 11];
    const double units = a[10];
    // A volume weight's placement is RIGID: its shape is
    // inputs:scaleX/Y/Z's alone, so the transform-scale avars are read and
    // discarded here rather than zeroed at bake -- exec never binds them at
    // all, and a captured zero would be walked straight past by an override
    // or an animated channel. The authored-posed:space branch reads no avar
    // at all, so a volume that takes it discards the scale for free, which
    // is what exec does with the same rig.
    const bool noScale = B.noScaleAvars[size_t(i)] != 0;
    // avars:rotationSign, applied to the avar exactly where computations.cpp
    // applies it, so a mirrored limb composes the same numbers on both paths.
    const unsigned sign =
        size_t(i) < B.rotationSign.size() ? B.rotationSign[size_t(i)] : 0u;
    const double sx = RigExecRotationSignFromMask(sign, 0);
    return RigExecBakedComposeAvars(
        a[0] * units, a[1] * units, a[2] * units,
        noScale ? 1.0 : a[3], noScale ? 1.0 : a[4], noScale ? 1.0 : a[5],
        a[6] * sx,
        a[7] * RigExecRotationSignFromMask(sign, 1),
        a[8] * RigExecRotationSignFromMask(sign, 2),
        a[9] * sx, B.rotOrder[size_t(i)]);
}

/// The ordinary compose of slot \p i against \p parentPosed, which is what
/// exec answers for a provider no switch has replaced.
RigExecPointFrame
_ComposeUnswitched(const RigExecBakedProgramImpl &B, int i,
                   const GfMatrix4d &parentPosed)
{
    if (B.posedAuthored[size_t(i)]) {
        // A non-identity authored posed:space is the pose: exec returns its
        // frame and reads neither the avars nor the parent.
        return RigExecFrameFromMatrix(B.posedAuthoredM[size_t(i)]);
    }
    return RigExecFrameFromMatrix(RigExecComposeUnswitchedPoseMatrix(
        _ComposeAvarsOf(B,i),B.posedD[size_t(i)],B.parentDinv[size_t(i)],
        B.parentSpaceAuthored[size_t(i)]?B.parentSpaceM[size_t(i)]:parentPosed));
}

/// _SpaceFromFrame: an unusable frame selects the NaN sentinel, so the
/// failure survives into every descendant instead of being scrubbed into a
/// plausible identity.
GfMatrix4d
_SpaceOfFrame(const RigExecPointFrame &frame)
{
    GfMatrix4d space(1.0);
    if (!frame.IsValid() || frame.IsDegenerate() ||
        !RigExecPointsToMatrix(RigExecIdentityLandmarks(), frame.points,
                               &space)) {
        space = GfMatrix4d(1.0);
        space[3][0] = std::numeric_limits<double>::quiet_NaN();
    }
    return space;
}

/// Reads an explicit composed provider or a separately produced checkpoint.
/// The consumer performs no recomposition or source walk.
GfMatrix4d
_ReadFrameVersion(
    const RigExecBakedProgramImpl &B,
    const RigExecBakedProgramImpl::SpaceSwitch::FrameVersion &read)
{
    if(read.context>=0)return B.switchFrames[size_t(read.context)];
    return read.anchor>=0?B.posedM[size_t(read.anchor)]:GfMatrix4d(1.0);
}

}  // namespace

void RigExecBakedRunSpaceCheckpoint(RigExecBakedProgramImpl *program,RigExecBakedStep *step)
{
    auto &B=*program;
    auto &context=B.switchFrameContexts[size_t(step->object)];
    for(size_t k=0;k<context.recompose.size();++k) {
        const size_t slot=size_t(context.recompose[k]);
        auto &input=context.kernelInputs[k];
        input.avars=_ComposeAvarsOf(B,int(slot));
        input.posedDefault=B.posedD[slot];
        input.parentDefaultInverse=B.parentDinv[slot];
        input.parentExpression=B.parentSpaceM[slot];
        input.parentExpressionAuthored=B.parentSpaceAuthored[slot]!=0;
        input.posedAuthoredMatrix=B.posedAuthoredM[slot];
        input.posedAuthored=B.posedAuthored[slot]!=0;
    }
    B.switchFrames[size_t(step->object)]=RigExecRunSpaceCheckpoint(
        context.anchor>=0?B.posedM[size_t(context.anchor)]:GfMatrix4d(1.0),context.kernelInputs);
}


RigExecBakedProgramImpl::Solver
RigExecBakedProgramTesting::RefreshedSolverRests(
    const RigExecBakedProgram &program, size_t index)
{
    const RigExecBakedProgramImpl &B = *program._impl;
    RigExecBakedProgramImpl::Solver copy = B.solvers[index];
    RefreshSolverRests(B, &copy);
    return copy;
}

bool
RigExecBakedEvalFrameRecord(const RigExecBakedProgramImpl &B,
                            const RigExecBakedFrameRecord &record,
                            GfMatrix4d *matrix)
{
    *matrix = GfMatrix4d(1.0);
    const RigExecBakedCommit &commit = B.commits[size_t(record.commit)];
    if (commit.solverOutput) {
        // This descriptor's current output presence gates its checkpoint.
        if (record.position < 0 ||
            size_t(record.position) >= commit.present.size() ||
            !commit.present[size_t(record.position)]) {
            return false;
        }
    } else if (!commit.recordAfter ||
               (record.target > 0 && !commit.recordEveryTarget)) {
        // The exit gates the walk's recordFrame calls sit behind: set by the
        // commit's constraint step on every run, per exit.
        return false;
    }
    const RigExecPointFrame &frame = B.fin[size_t(record.version)];
    if (!frame.IsValid()) {
        return false;
    }
    const RigExecPointFrame &rest = B.restFrames[size_t(record.slot)];
    const std::array<GfVec3d, 4> landmarks =
        rest.IsValid() ? rest.points : RigExecIdentityLandmarks();
    return RigExecPointsToMatrix(landmarks, frame.points, matrix);
}

namespace {
void PrepareConstraintArrays(RigExecBakedProgramImpl::ConstraintArrays *arrays) {
    auto &a=*arrays;a.diagnostics.clear();a.poleDiagnostics.clear();
    const auto weights=[&](size_t channel,size_t count,const char *name,
        std::vector<std::string> *diagnostics,std::vector<double> *values) {
        const auto *raw=a.raw[channel].IsHolding<VtFloatArray>()?
            &a.raw[channel].UncheckedGet<VtFloatArray>():nullptr;
        if(raw && !raw->empty() && raw->size()!=count) {
            diagnostics->push_back(a.pathText+" "+name+" has "+std::to_string(raw->size())+
                " entries for "+std::to_string(count)+" sources");return false;
        }
        values->assign(count,1.0);
        if(raw) for(size_t i=0;i<raw->size();++i) (*values)[i]=(*raw)[i];
        return true;
    };
    const auto offsets=[&](size_t channel,const char *name,std::vector<GfVec3d> *values) {
        const auto *raw=a.raw[channel].IsHolding<VtVec3dArray>()?
            &a.raw[channel].UncheckedGet<VtVec3dArray>():nullptr;
        if(raw && !raw->empty() && raw->size()!=a.sourceCount) {
            a.diagnostics.push_back(a.pathText+" "+name+" has "+std::to_string(raw->size())+
                " entries for "+std::to_string(a.sourceCount)+" sources");return false;
        }
        values->assign(a.sourceCount,GfVec3d(0));
        if(raw) for(size_t i=0;i<raw->size();++i) (*values)[i]=(*raw)[i];
        return true;
    };
    a.ok=weights(0,a.sourceCount,"inputs:sourceWeights",&a.diagnostics,&a.weights);
    if(a.parentOffsets) a.ok=a.ok && offsets(1,"inputs:translationOffsets",&a.translationOffsets) &&
        offsets(2,"inputs:rotationOffsets",&a.rotationOffsets);
    else {a.translationOffsets.assign(a.sourceCount,GfVec3d(0));a.rotationOffsets.assign(a.sourceCount,GfVec3d(0));}
    if(a.readPole) a.poleOk=weights(3,a.poleCount,"inputs:poleVectorWeights",&a.poleDiagnostics,&a.poleWeights);
}
}

void
RigExecBakedRunPoseStep(RigExecBakedProgramImpl *program,
                        RigExecBakedStep *step, UsdTimeCode time)
{
    RigExecBakedProgramImpl &B = *program;
    // Every per-frame read goes through these two: `rd` reads one input's
    // leaf, sampled in the prologue the way this generation must read it
    // (through the resolved inputs while an override stands on it, through
    // the pinned query otherwise), and `live` answers whether a set of baked
    // parameters has to be re-read at all.
    const auto rd = [&B](const auto &input) {
        return RigExecBakedLeafRead(B, input);
    };
    // Whether a solver input is read from its leaf rather than the
    // parameters Build folded: it varies, or a drag or an upstream value
    // stands on it.
    const auto live = [&](const auto &input) {
        const auto flagged = [&input](const std::vector<char> &flags) {
            return input.overrideIndex >= 0 &&
                   size_t(input.overrideIndex) < flags.size() &&
                   flags[size_t(input.overrideIndex)];
        };
        return input.varying || flagged(B.overridden) ||
               flagged(B.upstreamOn);
    };

    switch (step->kind) {
    case RigExecBakedStepKind::ComposeSubtree: {
        const RigExecBakedComposeGroup &group =
            B.composeGroups[size_t(step->object)];
        for (int i = group.begin; i < group.end; ++i) {
            if (B.slotKind[size_t(i)] != RigExecBakedSlotKind::FirstFramePose) {
                // An xform-derived slot is not composed from avars: the
                // dynamic path seeds it from the stage, and until the program
                // does the same nothing writes it.
                continue;
            }
            if (B.execCheckRows) B.execCheckRows->BeforeSlot(
                B, RigExecBakedStepKind::ComposeSubtree, group.begin, i);
            if(!_ProviderActive(B,size_t(i))) {
                B.base[size_t(i)]=RigExecPointFrame();
                B.base[size_t(i)].flags=0;
                B.fin[size_t(i)]=B.base[size_t(i)];
                B.posedM[size_t(i)]=_UnavailableProviderSpace();
                continue;
            }
            const int switchIndex =
                B.spaceSwitchBySlot.empty()
                    ? -1 : B.spaceSwitchBySlot[size_t(i)];
            if (switchIndex < 0) {
                const int parent = B.parent[size_t(i)];
                B.base[size_t(i)] = _ComposeUnswitched(
                    B, i,
                    parent >= 0 ? B.posedM[size_t(parent)] : GfMatrix4d(1.0));
            } else if (B.posedAuthored[size_t(i)]) {
                // A non-identity authored posed:space is the pose: exec
                // returns its frame and reads neither the avars nor the
                // parent, so neither does this.
                B.base[size_t(i)] =
                    RigExecFrameFromMatrix(B.posedAuthoredM[size_t(i)]);
            } else {
                auto &sw=B.spaceSwitches[size_t(switchIndex)];
                auto &input=sw.kernelInputs;
                input.avars=_ComposeAvarsOf(B,i);
                input.posedDefault=B.posedD[size_t(i)];
                input.parentDefaultInverse=B.parentDinv[size_t(i)];
                input.parentPosed=_ReadFrameVersion(B,sw.parentRead);
                input.parentDefault=B.parent[size_t(i)]>=0?
                    B.defaultRoundTrip[size_t(B.parent[size_t(i)])]:GfMatrix4d(1.0);
                input.active=0.0;
                if(sw.tokenIndex) {
                    const TfToken selected=rd(sw.activeTokenInput);
                    const auto label=std::find(sw.labels.begin(),sw.labels.end(),selected);
                    if(label!=sw.labels.end()) input.active=double(label-sw.labels.begin());
                } else input.active=rd(sw.activeInput);
                input.hasCarry=sw.spaceSlot>=0;
                if(input.hasCarry) {
                    input.spaceDefault=B.defaultRoundTrip[size_t(sw.spaceSlot)];
                    input.spacePosed=_ReadFrameVersion(B,sw.spaceRead);
                }
                for(size_t source=0;source<sw.sourceSlots.size();++source) {
                    const int slot=sw.sourceSlots[source];
                    auto &value=input.sources[source];value.world=slot<0;
                    if(slot>=0) {
                        value.defaultSpace=B.defaultRoundTrip[size_t(slot)];
                        value.posedSpace=_ReadFrameVersion(B,sw.sourceReads[source]);
                    }
                }
                if(!RigExecRunSpaceSwitch(sw.kernelRecord,input,&B.base[size_t(i)])) {
                    B.base[size_t(i)].flags=0;
                    step->diagnostics.push_back("space switch has no source for "+(*B.pathTexts)[size_t(i)]);
                }
            }
            const int autoIndex=(size_t(i)<B.autoClavicleBySlot.size()?B.autoClavicleBySlot[size_t(i)]:-1);
            if(autoIndex>=0) {
                auto &ac=B.autoClavicles[size_t(autoIndex)];
                ac.values.Publish(0,B.base[size_t(i)]);
                for(const auto &read:ac.frames) {
                    RigExecPointFrame frame;
                    if(read.computation=="computeRestFrame")frame=B.restFrames[size_t(read.slot)];
                    else if(read.computation=="computeDefaultFrame")frame=RigExecFrameFromMatrix(B.defaultRoundTrip[size_t(read.slot)]);
                    else if(B.paths[size_t(read.slot)].HasPrefix(B.paths[size_t(i)])) {
                        frame=B.base[size_t(i)];
                        for(int child:read.recompose)frame=_ComposeUnswitched(B,child,_SpaceOfFrame(frame));
                    } else frame=B.fin[B.finLast[size_t(read.slot)]];
                    ac.values.Publish(read.value,frame);
                }
                for(const auto &read:ac.scalars) {
                    if(read.isFloat)ac.values.Publish(read.value,rd(read.narrow),false,true);
                    else ac.values.Publish(read.value,rd(read.wide),false,true);
                }
                std::string invalid;
                if(!RigExecRunAutoClavicle(ac.operation,&ac.values,&invalid))step->diagnostics.push_back(invalid);
                if(const auto *frame=ac.values.Read<RigExecPointFrame>(ac.operation.output))B.base[size_t(i)]=*frame;
            }
            B.fin[size_t(i)] = B.base[size_t(i)];
            B.posedM[size_t(i)] = _SpaceOfFrame(B.base[size_t(i)]);
        }
        return;
    }

    case RigExecBakedStepKind::Solve: {
        RigExecBakedProgramImpl::Solver &s = B.solvers[size_t(step->object)];
        // The rest description, where the rest tier moved a rest it is
        // measured from this run (`restSlots` holds the space slot too), or
        // where the closure trusts nothing it holds: a full run can follow
        // a generation whose rest tier moved rests and whose region never
        // ran. The refresh is a pure function of those rests, so skipping
        // it over unchanged ones keeps the description it would write
        // (TestRefreshSolverRestsOnBuildStateIsIdentity). Pure arithmetic
        // over the tables the prologue left: no lock, no USD read and no
        // pose write, which is what lets it sit in a step body at all.
        // A LIVE rest depends on THIS frame pose, so a solver carrying one
        // refreshes on every evaluation.
        bool refresh =
            s.hasLiveRest || (B.closureFull && !s.restSlots.empty());
        for (size_t k = 0; !refresh && k < s.restSlots.size(); ++k) {
            const size_t slot = size_t(s.restSlots[k]);
            refresh = slot < B.restChanged.size() && B.restChanged[slot];
        }
        if (refresh) {
            RefreshSolverRests(B, &s);
            if (!B.measurementSuspended) {
                ++s.restRefreshes;
            }
        }
        RigExecPointFrameArray &aggregate = B.aggregates[size_t(step->object)];
        s.fallbackSlots.clear();
        auto &input=s.kernelInputs;
        s.hasStart=s.start>=0;
        input.hasSpace=s.spaceSlot>=0;
        if(input.hasSpace) {input.spaceRest=s.spaceRest;input.space=B.fin[size_t(s.spaceRead)];}
        const auto frame=[&](int provider,uint32_t version) {
            return provider>=0?B.fin[size_t(version)]:RigExecPointFrame();
        };
        input.root=frame(s.root,s.rootRead);
        input.mid=frame(s.mid,s.midRead);
        input.end=frame(s.end,s.endRead);
        input.pole=frame(s.pole,s.poleRead);
        input.start=frame(s.start,s.startRead);
        if(s.type=="RigExecFkChain") {
            s.kind=RigExecSolverKind::FkChain;
            input.controls.resize(s.controls.size());
            for(size_t k=0;k<s.controls.size();++k) input.controls[k]=B.fin[size_t(s.controlReads[k])];
        } else if(s.type=="RigExecTwoBoneIk") {
            s.kind=RigExecSolverKind::TwoBoneIk;
            input.ikSpace=rd(s.ikSpace);
            input.refreshIkParams=true;
            input.pin=rd(s.pin);input.softDistance=rd(s.softDistance);input.limbTwist=rd(s.limbTwist);
            input.upperScale=rd(s.upperScale);input.lowerScale=rd(s.lowerScale);
            input.bend=rd(s.bend); input.stretch=rd(s.stretch);input.softness=rd(s.softness);
            input.upperOffset=rd(s.upperOffset);input.lowerOffset=rd(s.lowerOffset);
        } else if(s.type=="RigExecBlendPointFrames") {
            s.kind=RigExecSolverKind::BlendPointFrames;
            input.blendA=s.inA>=0?&B.aggregates[size_t(s.inA)]:nullptr;
            input.blendB=s.inB>=0?&B.aggregates[size_t(s.inB)]:nullptr;
            input.blendWeight=rd(s.blendWeight);
        } else if(s.type=="RigExecTwistDistribution") {
            s.kind=RigExecSolverKind::TwistDistribution; input.twistTurns=rd(s.twistTurns);
        } else if(s.type=="RigExecRibbon") {
            s.kind=RigExecSolverKind::Ribbon;
            const auto &sampled=s.ribbonPointsVarying?s.ribbonPoints:s.ribbonConstantPoints;
            const GfVec3f *points=nullptr;size_t count=0;
            const bool available=RigExecBakedResolvePoints(B,s.ribbonPointsBinding,&points,&count);
            if(available && count) input.ribbonPoints.assign(points,points+count);
            else if(available) input.ribbonPoints.clear();
            else {
                input.ribbonPoints=sampled;
                if(!s.ribbonPointsBinding.candidates.empty()) step->diagnostics.push_back(
                    "diag "+s.pathText+": ribbon driver points are unavailable; using sampled base");
            }
            input.ribbonSampleCount=rd(s.ribbonSampleCount);
        } else if(s.type=="RigExecSplineIk") {
            s.kind=RigExecSolverKind::SplineIk;
            input.refreshSplineParams=s.splineParamsVary||live(s.preserveVolume)||live(s.midFollowWeight)||
                live(s.roll)||live(s.twist)||live(s.minLengthRatio);
            input.preserveVolume=rd(s.preserveVolume);input.midFollowWeight=rd(s.midFollowWeight);
            input.rollDegrees=rd(s.roll);input.twistDegrees=rd(s.twist);input.minLengthRatio=rd(s.minLengthRatio);
        }
        std::string error;
        if(!RigExecRunSolver(s,input,&s.kernelWorkspace,&aggregate,&error) && !error.empty())
            step->diagnostics.push_back("diag "+s.pathText+": "+error);
        for (size_t k = 0; k < s.outputs.size(); ++k) {
            const auto &[slot, element] = s.outputs[k];
            if (element < 0 || size_t(element) >= aggregate.GetSize()) {
                s.outPresent[k] = 0;
                s.fallbackSlots.push_back(slot);
                continue;
            }
            s.outFrames[k] =
                RigExecExtractElementFrame(&aggregate, size_t(element));
            s.outPresent[k] = 1;
        }
        return;
    }

    case RigExecBakedStepKind::SolverCommit: {
        RigExecBakedCommit &commit = B.commits[size_t(step->object)];
        std::fill(commit.present.begin(), commit.present.end(), 0);
        for (const int si : B.walkSteps[size_t(step->object)].batchSolvers) {
            const RigExecBakedProgramImpl::Solver &s = B.solvers[size_t(si)];
            for (size_t k = 0; k < s.outputs.size(); ++k) {
                if (!s.outPresent[k] || s.outPosition[k] < 0) {
                    continue;
                }
                // Last writer wins on a duplicate slot, because the stores
                // happen in batch order: `candidates[slot] = ...`.
                commit.frames[size_t(s.outPosition[k])] = s.outFrames[k];
                commit.present[size_t(s.outPosition[k])] = 1;
            }
        }
        commit.abandoned = std::find(commit.present.begin(),
                                     commit.present.end(), 1) ==
                           commit.present.end();
        if (commit.split) {
            return;  // CommitApply finishes, and carries
        }
        // A batch that published NOTHING still reaches FinishCommit, the
        // same way a constraint that passes through does. It writes no
        // frame, but it owns a version of every slot it could have written
        // (§3.1) and a write site that leaves its own storage untouched
        // leaves every reader bound to it -- the matrices and the
        // publication among them -- reading whatever the last run left
        // there. Reachable only from a batch whose every solver published an
        // empty aggregate, which until RigExecTwistDistribution baked no rig
        // in the tree could produce.
        if (!commit.abandoned) {
            ComputeCommitDeltas(B, &commit);
            StageCommitPairs(B, &commit, 0, commit.propagate.size());
        }
        FinishCommit(&B, step, &commit);
        return;
    }

    case RigExecBakedStepKind::Constraint: {
        RigExecBakedCommit &commit = B.commits[size_t(step->object)];
        commit.abandoned = true;
        std::fill(commit.present.begin(), commit.present.end(), 0);
        // Non-const because the envelope arm below resolves into this
        // constraint's own scratch, which is storage one step owns and no
        // other step names.
        RigExecBakedProgramImpl::Constraint &c =
            B.constraints[size_t(B.walkSteps[size_t(step->object)].index)];
        const auto runSourceConstraint=[&](const RigExecPointFrame &incoming,
            const std::vector<RigExecConstraintSource> &sources,const auto &params) {
            auto &record=c.kernelRecord;auto &inputs=c.kernelInputs;
            using T=std::decay_t<decltype(params)>;
            inputs.incoming=incoming;inputs.sources=sources;inputs.carry.reset();
            if constexpr(std::is_same_v<T,RigExecPositionConstraintParams>) {
                record.kind=RigExecConstraintKind::Position;record.position=params;
            } else if constexpr(std::is_same_v<T,RigExecRotationConstraintParams>) {
                record.kind=RigExecConstraintKind::Rotation;record.rotation=params;
                if(params.carry) inputs.carry=*params.carry;record.rotation.carry=nullptr;
            } else if constexpr(std::is_same_v<T,RigExecScaleConstraintParams>) {
                record.kind=RigExecConstraintKind::Scale;record.scale=params;
            } else {
                record.kind=RigExecConstraintKind::Parent;record.parent=params;
                if(params.carry) inputs.carry=*params.carry;record.parent.carry=nullptr;
            }
            RigExecRunConstraint(record,inputs,&c.kernelResult);
            return c.kernelResult.frame;
        };
        // Every exit below leaves `recordAfter`/`recordEveryTarget` saying
        // whether the dynamic walk records the target's frame there; the
        // commit's FrameMatrix steps, which run after its write-back, read
        // them. A constraint that passed through still leaves its target
        // standing at the point a phase names.
        const auto finish = [&] {
            if (commit.split) {
                return;  // CommitApply finishes
            }
            if (!commit.abandoned) {
                ComputeCommitDeltas(B, &commit);
                StageCommitPairs(B, &commit, 0, commit.propagate.size());
            }
            FinishCommit(&B, step, &commit);
        };
        // The GEOMETRY domain writes no transform: it measures a delta
        // against the target's authored transform and hands it to the Matrix
        // revision the same mover contributes. Both halves of what makes it
        // different are set up here -- the delta is absent until this run
        // produces one, and the target's frame is recorded for a read phase
        // only on the exits the dynamic walk records it on.
        const int deltaBase = c.deltaBase;
        commit.recordAfter = true;
        commit.recordEveryTarget = true;
        if (deltaBase >= 0) {
            B.deltaPresent[size_t(deltaBase)] = 0;
        }
        if (c.target < 0) {
            // No slot to revise: nothing the walk can commit, and nothing to
            // record. The dynamic path reaches its target through the frame
            // map and finds nothing either.
            finish();
            return;
        }
        if (!rd(c.enabled)) {
            finish();
            return;
        }
        // The envelope, in the dynamic path's THREE exclusive arms. A
        // constraint copies the ORACLE -- the dynamic constraint path does
        // not go through exec at all, it calls _ResolveWeights and takes its
        // error string -- so this calls the same function with the same
        // arguments and the answer is identical by construction.
        // The finite-[0, 1] check belongs to the OTHER arm and must not be
        // applied to a resolved envelope: the dynamic path does not check
        // there, so checking would emit a diagnostic it never emits.
        // The third arm is a GEOMETRY-domain constraint that binds a weight
        // object, and it is the one this block did not have while the two
        // features lived on different branches: weight objects on
        // constraints were this group's, geometry-domain constraints were
        // another's, and neither branch alone could build the rig that needs
        // it. Such a constraint resolves NO envelope here. Its weight is per
        // POINT and resolves after the solve, on the revision this delta
        // feeds -- and because a constraint and its revision are the same
        // mover prim, that revision's own weight packet IS this constraint's
        // weight object. Leaving `weight` at 1.0 is what hands the revision
        // a full-strength delta for the per-point lerp to scale; resolving
        // one element here instead would apply a one-point answer to the
        // whole mesh AND square the envelope on the point the packet also
        // covers.
        double weight = 1.0;
        if (!c.weightObject.IsEmpty() && c.pointsTarget.IsEmpty()) {
            const auto &field = B.weightFields[size_t(c.weightField)];
            c.weightScratch = field.values;
            c.weightError = field.error;
            const bool resolved = field.ok;
            if (!resolved || c.weightScratch.size() != 1) {
                step->diagnostics.push_back(
                    c.pathText + ": " + c.weightError +
                    "; constraint passed through");
                finish();
                return;
            }
            weight = c.weightScratch[0];
        } else if (c.weightObject.IsEmpty()) {
            weight = rd(c.defaultWeight);
            if (!std::isfinite(weight) || weight < 0.0 || weight > 1.0) {
                step->diagnostics.push_back(
                    c.pathText +
                    " has inputs:defaultWeight outside finite [0, 1]; "
                    "constraint passed through");
                finish();
                return;
            }
        }
        // Textually the dynamic path's gate, second clause included. With
        // the three arms above the clause cannot change the answer -- the
        // only constraint that reaches here with a bound geometry weight
        // object has weight == 1.0 -- but the two conditions are kept alike
        // on purpose: this is the pair that silently disagreed when the
        // third arm was missing, and the next person to add an arm should
        // find one shape to compare, not two.
        if (weight <= 0.0 &&
            (c.pointsTarget.IsEmpty() || c.weightObject.IsEmpty())) {
            // A zero envelope is an exact dormant pass-through, decided
            // before any source is resolved so a malformed disconnected input
            // cannot make a disabled constraint fail. A DORMANT geometry
            // constraint still publishes a delta -- the identity one -- so
            // that the revision it feeds moves the points nowhere rather
            // than falling back to whatever matrix it last held.
            if (deltaBase >= 0) {
                B.deltaValues[size_t(deltaBase)] = GfMatrix4d(1.0);
                B.deltaPresent[size_t(deltaBase)] = 1;
            }
            finish();
            return;
        }
        // One frame source, resolved in resolveBinding's order: the walk's
        // own frame when it holds one, and otherwise the transform the
        // prologue read off the stage, ridden on the revision of the deepest
        // provider above it that the walk has already moved. The ride is the
        // shared routine both paths call, so the two cannot pick different
        // ancestors or measure different deltas.
        const auto resolveSource =
            [&B](int slot, uint32_t finRead, int native,
                 const std::vector<RigExecBakedCommit::AncestorRead>
                     &ancestors,
                 RigExecPointFrame *out) {
            if (slot >= 0) {
                *out = B.fin[size_t(finRead)];
                return out->IsValid();
            }
            if (native < 0 || !B.nativeFrameOk[size_t(native)]) {
                return false;
            }
            *out = B.nativeFrames[size_t(native)];
            if (!out->IsValid()) {
                return false;
            }
            const auto enumerate =
                [&B, &ancestors](const RigExecPoseFrameVisitor &visit) {
                for (const RigExecBakedCommit::AncestorRead &a : ancestors) {
                    visit(B.paths[size_t(a.slot)], B.base[size_t(a.base)],
                          B.fin[size_t(a.fin)]);
                }
            };
            const RigExecPoseFrameEnumerator providers(enumerate);
            return RigExecApplyRevisedAncestorDelta(
                B.nativeSources[size_t(native)].path, providers, out);
        };
        // This constraint's authored tables, as the prologue read them at
        // this frame's time.
        auto &arrays=B.constraintArrays[size_t(c.arrays)];
        PrepareConstraintArrays(&arrays);
        if (c.singleChainIk) {
            // The one multi-target built-in, in the dynamic walk's order:
            // the chain, the effector, the parameters, the pole, the
            // rest-derived preparation, the solve, one atomic validity
            // check, and then a commit over the whole chain.
            bool inputsValid = true;
            for (size_t k = 0; k < c.targetSlots.size(); ++k) {
                commit.ikChain[k] = B.fin[size_t(commit.targetReads[k])];
            }
            RigExecPointFrame effector;
            if (!resolveSource(c.effector, commit.effectorRead,
                               c.effectorNative, commit.effectorAncestors,
                               &effector)) {
                step->diagnostics.push_back(
                    c.pathText +
                    " could not resolve its effector; constraint passed "
                    "through");
                inputsValid = false;
            }
            RigExecSingleChainIkParams params;
            params.stretch=rd(c.stretch);
            params.mode = c.ikMode;
            params.preserveJointOrientation = c.preserveJointOrientation;
            params.weight = weight;
            // Read BEFORE the pole mode is consulted, and only in
            // RotatePlane mode, which is where the dynamic walk reads them:
            // object mode then overwrites params.pole. A different read
            // order is numerically invisible and changes which inputs a drag
            // reaches.
            if (c.ikMode == RigExecSingleChainIkMode::RotatePlane) {
                params.pole = rd(c.poleVector);
                params.twistDegrees = rd(c.twistDegrees);
            }
            if (inputsValid &&
            c.ikMode == RigExecSingleChainIkMode::RotatePlane &&
                c.poleModeObject) {
                if (c.poleObjects.empty()) {
                    step->diagnostics.push_back(
                        c.pathText +
                        " uses object pole mode with no pole-vector objects; "
                        "constraint passed through");
                    inputsValid = false;
                }
                if (inputsValid && !arrays.poleOk) {
                    for (const std::string &line : arrays.poleDiagnostics) {
                        step->diagnostics.push_back(line);
                    }
                    inputsValid = false;
                }
                GfVec3d polePoint(0);
                double total = 0;
                for (size_t k = 0;
                     inputsValid && k < c.poleObjects.size(); ++k) {
                    RigExecPointFrame poleFrame;
                    if (!resolveSource(c.poleObjects[k], commit.poleReads[k],
                                       c.poleObjectNatives[k],
                                       commit.poleAncestors[k], &poleFrame) ||
                        !std::isfinite(arrays.poleWeights[k]) ||
                        arrays.poleWeights[k] < 0) {
                        step->diagnostics.push_back(
                            c.pathText +
                            " has an invalid pole-vector source or weight; "
                            "constraint passed through");
                        inputsValid = false;
                        break;
                    }
                    polePoint += poleFrame.Origin() * arrays.poleWeights[k];
                    total += arrays.poleWeights[k];
                }
                if (inputsValid && total <= 0) {
                    step->diagnostics.push_back(
                        c.pathText +
                        " has zero total pole-vector weight; constraint "
                        "passed through");
                    inputsValid = false;
                }
                if (inputsValid) {
                    params.pole = polePoint / total;
                }
            }
            // "neverTS" rebuilds the chain's layout from its rests, which is
            // the better-conditioned question for a chain with no animated
            // translations. Which of the two it is was settled at bake.
            const std::vector<RigExecPointFrame> *solveChain = &commit.ikChain;
            if (inputsValid && !c.useAnimatedTs) {
                for (size_t k = 0; k < c.targetSlots.size(); ++k) {
                    // The incoming frame IS the rest reference where a step
                    // below this constraint wrote the joint.
                    commit.ikRest[k] =
                        (k < c.ikRestLive.size() && c.ikRestLive[k] &&
                         k < commit.ikChain.size())
                            ? commit.ikChain[k]
                            : B.restFrames[size_t(c.targetSlots[k])];
                }
                if (!RigExecPrepareRestDerivedIkChain(
                        commit.ikChain, commit.ikRest, &commit.ikPrepared)) {
                    step->diagnostics.push_back(
                        c.pathText +
                        " could not prepare rest-derived IK inputs; "
                        "constraint passed through");
                    inputsValid = false;
                } else {
                    solveChain = &commit.ikPrepared;
                }
            }
            commit.ikSolved.clear();
            if (inputsValid) {
                c.kernelRecord.kind=RigExecConstraintKind::SingleChainIk;
                c.kernelRecord.singleChain=params;c.kernelInputs.chain=*solveChain;
                c.kernelInputs.effector=effector;
                RigExecRunConstraint(c.kernelRecord,c.kernelInputs,&c.kernelResult);
                commit.ikSolved=c.kernelResult.chain;
            }
            // Atomic: the whole chain or none of it, and before any write.
            if (inputsValid &&
                (commit.ikSolved.size() != c.targetSlots.size() ||
                 std::any_of(commit.ikSolved.begin(), commit.ikSolved.end(),
                             [](const RigExecPointFrame &frame) {
                                 return !RigExecBakedUsable(frame);
                             }))) {
                step->diagnostics.push_back(
                    c.pathText +
                    " failed to solve its joint chain; constraint passed "
                    "through atomically");
                inputsValid = false;
            }
            if (inputsValid) {
                // commit.slots is the chain, sorted; targetSlots is the
                // chain in SOLVE order, so each solved frame lands at its
                // own slot's position rather than at its own index.
                for (size_t k = 0; k < c.targetSlots.size(); ++k) {
                    const auto found = std::lower_bound(
                        commit.slots.begin(), commit.slots.end(),
                        c.targetSlots[k]);
                    if (found == commit.slots.end() ||
                        *found != c.targetSlots[k]) {
                        continue;
                    }
                    const size_t pos = size_t(found - commit.slots.begin());
                    commit.frames[pos] = commit.ikSolved[k];
                    commit.present[pos] = 1;
                }
                commit.abandoned = false;
            }
            finish();
            return;
        }

        // Past the IK exit, and so past the last exit the dynamic walk
        // records every target on: its remaining two record targets[0].
        commit.recordEveryTarget = false;

        // buildSources' order, which is a diagnostic order as much as an
        // arithmetic one: the authored tables first, then the source frames.
        // A rig with both a bad cardinality and an unresolvable source says
        // so in that order, and the comparator compares diagnostics in
        // order.
        bool sourcesReady = arrays.ok;
        for (const std::string &line : arrays.diagnostics) {
            step->diagnostics.push_back(line);
        }
        for (size_t k = 0; sourcesReady && k < c.sources.size(); ++k) {
            RigExecPointFrame frame;
            if (!resolveSource(c.sources[k], commit.sourceReads[k],
                               c.sourceNatives[k], commit.sourceAncestors[k],
                               &frame)) {
                step->diagnostics.push_back(
                    c.pathText + " could not resolve source " +
                    c.sourcePathTexts[k]);
                sourcesReady = false;
                break;
            }
            commit.sources[k].frame = frame;
            commit.sources[k].normalizedWeight = arrays.weights[k];
            commit.sources[k].translationOffset =
                arrays.translationOffsets[k];
            commit.sources[k].rotationOffsetDegrees =
                arrays.rotationOffsets[k];
        }
        if (!sourcesReady) {
            step->diagnostics.push_back(
                c.pathText +
                " has unusable constraint inputs; constraint passed through");
            finish();
            return;
        }
        // Past this point the dynamic walk records nothing for a
        // geometry-domain constraint: it revises no transform, so there is
        // no "the target as of this mover" to record.
        commit.recordAfter = deltaBase < 0;
        // The envelope is applied exactly ONCE. In the transform domain the
        // kernel's per-channel blend carries it; in the geometry domain the
        // per-point lerp does, so the solve runs UNWEIGHTED and hands back
        // the full-strength delta. Passing the envelope to both would square
        // it, and 0.5 would come out as 0.25 on points.
        const double solveWeight = deltaBase < 0 ? weight : 1.0;
        const std::vector<RigExecConstraintSource> &sources = commit.sources;
        const RigExecPointFrame input = B.fin[size_t(commit.targetRead)];
        RigExecPointFrame candidate = input;
        bool candidateReady = true;
        RigExecConstraintAxisMask affect;
        affect.x = rd(c.affectX);
        affect.y = rd(c.affectY);
        affect.z = rd(c.affectZ);
        // rigExec:space: the space switch's carry, from the same two
        // tables (see the compose above). -1 is "no space named", and the
        // params keep their null carry so the kernel's untouched branch
        // runs -- the dynamic path does the same.
        GfMatrix4d carry(1.0);
        const bool hasCarry = c.spaceSlot >= 0;
        if (hasCarry) {
            carry = B.defaultRoundTrip[size_t(c.spaceSlot)].GetInverse() *
                    B.posedM[size_t(c.spaceSlot)];
        }
        if (c.type == "RigExecMatrixMover") {
            // The transform-domain matrix mover, as the dynamic walk's arm:
            // each source is measured as its rest->posed map, the transform
            // in its space, and the envelope is applied to the target's four
            // landmarks with the point kernel's rule.
            const auto mapOf = [&](size_t k, GfMatrix4d *out) {
                const int slot = c.sources[k];
                const bool haveRest =
                    slot >= 0 && B.restFrames[size_t(slot)].IsValid();
                const auto &landmarks =
                    haveRest ? B.restFrames[size_t(slot)].points
                             : RigExecIdentityLandmarks();
                return RigExecPointsToMatrix(landmarks, sources[k].frame,
                                             out);
            };
            GfMatrix4d transform(1.0);
            if (sources.empty() || !mapOf(0, &transform)) {
                step->diagnostics.push_back(
                    c.pathText +
                    " could not resolve rigExec:transform; mover passed "
                    "through");
                finish();
                return;
            }
            if (sources.size() > 1) {
                GfMatrix4d space(1.0);
                if (!mapOf(1, &space)) {
                    step->diagnostics.push_back(
                        c.pathText +
                        " could not resolve rigExec:transformSpace; mover "
                        "passed through");
                    finish();
                    return;
                }
                transform = RigExecMeasureInSpace(transform, space);
            }
            RigExecPointFrame solved = input;
            if (c.radialBlend) {
                const GfMatrix4d partial =
                    RigExecPartialTransform(transform, weight);
                for (GfVec3d &q : solved.points) {
                    q = partial.TransformAffine(q);
                }
            } else {
                for (GfVec3d &q : solved.points) {
                    q = q + weight * (transform.TransformAffine(q) - q);
                }
            }
            candidate = solved;
        } else if (c.type == "RigExecPositionConstraint") {
            RigExecPositionConstraintParams params;
            params.offset = rd(c.offset);
            params.affect = affect;
            params.weight = solveWeight;
            candidate = runSourceConstraint(input, sources, params);
        } else if (c.type == "RigExecRotationConstraint") {
            RigExecRotationConstraintParams params;
            params.offsetDegrees = rd(c.offset);
            params.affect = affect;
            params.rotationOrder = c.order;
            params.weight = solveWeight;
            params.carry = hasCarry ? &carry : nullptr;
            candidate = runSourceConstraint(input, sources, params);
        } else if (c.type == "RigExecScaleConstraint") {
            RigExecScaleConstraintParams params;
            params.offset = rd(c.offset);
            params.affect = affect;
            params.weight = solveWeight;
            params.blendShear = c.blendShear;
            candidate = runSourceConstraint(input, sources, params);
        } else if (c.type == "RigExecParentConstraint") {
            RigExecParentConstraintParams params;
            params.translationAxes.x = rd(c.tX);
            params.translationAxes.y = rd(c.tY);
            params.translationAxes.z = rd(c.tZ);
            params.rotationAxes.x = rd(c.rX);
            params.rotationAxes.y = rd(c.rY);
            params.rotationAxes.z = rd(c.rZ);
            params.scaleAxes.x = rd(c.sX);
            params.scaleAxes.y = rd(c.sY);
            params.scaleAxes.z = rd(c.sZ);
            params.rotationOrder = c.order;
            params.weight = solveWeight;
            params.carry = hasCarry ? &carry : nullptr;
            params.blendShear = c.blendShear;
            candidate = runSourceConstraint(input, sources, params);
        } else {
            // Aim: the same weighted source set reduced to the target point
            // FBX's AimAtObjects contract specifies.
            GfVec3d target(0);
            double total = 0;
            for (const RigExecConstraintSource &source : sources) {
                if (!std::isfinite(source.normalizedWeight) ||
                    source.normalizedWeight < 0) {
                    step->diagnostics.push_back(
                        c.pathText +
                        " has an invalid source weight; constraint passed "
                        "through");
                    candidateReady = false;
                    break;
                }
                target += source.frame.Origin() * source.normalizedWeight;
                total += source.normalizedWeight;
            }
            if (candidateReady && total > 0) {
                target /= total;
                RigExecAimConstraintParams params;
                params.localAimVector = c.aimVectorAuthored
                                            ? rd(c.aimVector)
                                            : c.aimAxisFallback;
                params.localUpVector = rd(c.upVector);
                params.rotationOffsetDegrees = rd(c.rotationOffset);
                params.affectRotation = affect;
                params.rotationOrder = c.order;
                params.weight = solveWeight;
                params.preserveInputUp = c.preserveInputUp;
                const GfVec3d authoredWorldUp = rd(c.worldUpVector);
                if (c.worldUpType == "sceneUp") {
                    params.worldUpDirection = c.sceneUp;
                } else if (c.worldUpType == "vector") {
                    params.worldUpDirection = authoredWorldUp;
                } else if (c.worldUpType == "objectUp") {
                    // FBX ObjectUp with no reference object uses the world
                    // origin as the object point.
                    RigExecPointFrame upObject;
                    if (!c.worldUpObjectNamed) {
                        params.worldUpDirection = GfVec3d(-input.Origin());
                    } else if (!resolveSource(c.worldUpObject,
                                              commit.worldUpRead,
                                              c.worldUpNative,
                                              commit.worldUpAncestors,
                                              &upObject)) {
                        step->diagnostics.push_back(
                            c.pathText +
                            " could not resolve its world-up object; "
                            "constraint passed through");
                        candidateReady = false;
                    } else {
                        params.worldUpDirection =
                            upObject.Origin() - input.Origin();
                    }
                } else if (c.worldUpType == "objectRotationUp") {
                    // With no object, FBX applies WorldUpVector directly in
                    // world space rather than treating a missing binding as
                    // a failed constraint.
                    if (!c.worldUpObjectNamed) {
                        params.worldUpDirection = authoredWorldUp;
                    } else {
                        RigExecPointFrame upObject;
                        if (!resolveSource(c.worldUpObject,
                                           commit.worldUpRead,
                                           c.worldUpNative,
                                           commit.worldUpAncestors,
                                           &upObject)) {
                            step->diagnostics.push_back(
                                c.pathText +
                                " could not resolve its world-up object; "
                                "constraint passed through");
                            candidateReady = false;
                        }
                        GfMatrix4d up(1.0);
                        if (candidateReady &&
                            !RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                                   upObject.points, &up)) {
                            step->diagnostics.push_back(
                                c.pathText +
                                " has a degenerate world-up object");
                            candidateReady = false;
                        } else if (candidateReady) {
                            // rigExec:worldUpRotationOnly, as the
                            // dynamic path takes it.
                            params.worldUpDirection =
                                (c.worldUpRotationOnly
                                     ? up.GetOrthonormalized(false)
                                     : up)
                                    .ExtractRotation()
                                    .TransformDir(authoredWorldUp);
                        }
                    }
                }
                if (candidateReady) {
                    c.kernelRecord.kind=RigExecConstraintKind::Aim;
                    c.kernelRecord.aim=params;c.kernelInputs.incoming=input;
                    c.kernelInputs.aimTarget=target;
                    RigExecRunConstraint(c.kernelRecord,c.kernelInputs,&c.kernelResult);
                    candidate=c.kernelResult.frame;
                }
            }
        }
        if (candidateReady && B.execCheckRows)
            B.execCheckRows->ObserveConstraintCandidate(step->object,candidate);
        if (candidateReady && deltaBase >= 0) {
            // The solve produced the same full-strength frame the transform
            // domain would publish; the delta against the prim's own base
            // transform is what the points ride.
            //     D = F_solved * F_base^-1
            // The prim's transform is NOT revised: a geometry-domain
            // constraint writes points and nothing else.
            const GfMatrix4d &baseMatrix =
                B.deltaBaseMatrix[size_t(deltaBase)];
            GfMatrix4d solvedMatrix(1.0);
            if (RigExecBakedUsable(candidate) &&
                B.deltaBaseOk[size_t(deltaBase)] &&
                std::isfinite(baseMatrix.GetDeterminant()) &&
                baseMatrix.GetDeterminant() != 0.0 &&
                RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                      candidate.points, &solvedMatrix)) {
                B.deltaValues[size_t(deltaBase)] =
                    solvedMatrix * baseMatrix.GetInverse();
                B.deltaPresent[size_t(deltaBase)] = 1;
            } else {
                step->diagnostics.push_back(
                    c.pathText + " could not measure its delta "
                    "against " + c.deltaBasePathText +
                    "; constraint passed through");
            }
        } else if (candidateReady) {
            // The candidate sweep a constraint commit opens with: an unusable
            // revision is diagnosed and the whole commit passes through.
            if (!RigExecBakedUsable(candidate)) {
                step->diagnostics.push_back(
                    c.pathText +
                    " produced an invalid or degenerate frame for " +
                    (*B.pathTexts)[size_t(c.target)] +
                    "; constraint passed through");
            } else {
                commit.frames[0] = candidate;
                commit.present[0] = 1;
                commit.abandoned = false;
            }
        }
        finish();
        return;
    }

    case RigExecBakedStepKind::CommitDelta: {
        RigExecBakedCommit &commit = B.commits[size_t(step->object)];
        if (!commit.abandoned) {
            ComputeCommitDeltas(B, &commit);
        }
        return;
    }

    case RigExecBakedStepKind::PropagateChunk: {
        RigExecBakedCommit &commit = B.commits[size_t(step->object)];
        if (commit.abandoned) {
            return;
        }
        const size_t begin = size_t(step->part) * kPropagateChunkSize;
        const size_t end =
            std::min(begin + kPropagateChunkSize, commit.propagate.size());
        StageCommitPairs(B, &commit, begin, end);
        return;
    }

    case RigExecBakedStepKind::CommitApply: {
        FinishCommit(&B, step, &B.commits[size_t(step->object)]);
        return;
    }

    case RigExecBakedStepKind::ProviderMatrix: {
        const size_t slot = size_t(step->object);
        // Never a bail: RigExecPointsToMatrix leaves the identity and says
        // so, and the one failure this phase can report -- a published joint
        // whose rest or final frame is not usable -- is the epilogue's.
        GfMatrix4d matrix(1.0);
        if (step->part) {
            // The LAST version of the slot: these steps run after the whole
            // walk, so what they describe is where the provider ended up.
            const RigExecPointFrame &frame = B.fin[size_t(B.finLast[slot])];
            const bool primary = RigExecBakedUsable(B.restFrames[slot]) &&
                RigExecBakedUsable(frame) &&
                RigExecPointsToMatrix(B.restPts[slot], frame.points, &matrix);
            // Unavailable final frames retain ORIGINAL's identity provider.
            // The rest/base fallback is only meaningful for a usable final;
            // PointsToMatrix validates its reference basis, not NaN pose data.
            if (!primary && RigExecBakedUsable(frame)) {
                const auto &baseFrame = B.base[size_t(B.baseLast[slot])];
                matrix = GfMatrix4d(1.0);
                if (B.restFrames[slot].IsValid() && baseFrame.IsValid()) {
                    RigExecPointsToMatrix(B.restPts[slot], baseFrame.points, &matrix);
                }
                if (std::find(B.jointSlots.begin(), B.jointSlots.end(),
                              int(slot)) != B.jointSlots.end()) {
                    GfMatrix4d delta(1.0);
                    if (RigExecPointsToMatrix(baseFrame.points, frame.points, &delta))
                        matrix = matrix * delta;
                }
            }
            B.finalMatrix[slot] = matrix;
        } else {
            const RigExecPointFrame &frame = B.base[size_t(B.baseLast[slot])];
            if (RigExecBakedUsable(B.restFrames[slot]) &&
                RigExecBakedUsable(frame)) {
                RigExecPointsToMatrix(B.restPts[slot], frame.points, &matrix);
            }
            B.baseMatrix[slot] = matrix;
        }
        return;
    }

    case RigExecBakedStepKind::FrameMatrix: {
        // Both fields every run, so a reader never sees a valid byte from
        // another run beside this run's matrix.
        const size_t record = size_t(step->object);
        GfMatrix4d matrix(1.0);
        const bool valid =
            RigExecBakedEvalFrameRecord(B, B.frameRecords[record], &matrix);
        B.frameMatrix[record] = matrix;
        B.frameMatrixValid[record] = valid ? 1 : 0;
        return;
    }

    case RigExecBakedStepKind::PoseInterpolator: {
        RigExecBakedProgramImpl::PoseInterpolator &interpolator =
            B.poseInterpolators[size_t(step->object)];
        interpolator.enabledValue=rd(interpolator.enabled);
        for(size_t i=0;i<interpolator.valueInputs.size();++i)
            interpolator.values[i]=rd(interpolator.valueInputs[i]);
        // Every slot the step owns starts at zero: a disabled pose was left
        // out of the solve and has no weight to be told, and a disabled
        // interpolator -- a shape-preserving enable -- is off, not frozen at
        // its last value.
        for (int slot = interpolator.weightBegin; slot < interpolator.weightEnd;
             ++slot) {
            B.poseWeights[size_t(slot)] = 0.0f;
        }
        if (!interpolator.enabledValue || interpolator.poseSlots.empty()) {
            return;
        }
        const bool numeric=!interpolator.valueInputs.empty();
        const size_t d=size_t(numeric?0:interpolator.driverSlot);
        auto &inputs=interpolator.kernelInputs;
        inputs.enabled=interpolator.enabledValue;
        inputs.numeric=interpolator.values;
        inputs.driverFinal=numeric?nullptr:&B.fin[size_t(B.finLast[d])];
        inputs.driverRest=numeric?nullptr:&B.restFrames[d];
        inputs.parentFinal=inputs.parentRest=nullptr;
        if(!numeric && interpolator.parentSlot>=0) {
            const size_t parent=size_t(interpolator.parentSlot);
            inputs.parentFinal=&B.fin[size_t(B.finLast[parent])];
            inputs.parentRest=&B.restFrames[parent];
        }
        const auto status=RigExecRunPoseInterpolator(interpolator,inputs,&interpolator.scratch);
        using Status=RigExecPoseInterpolatorStatus;
        if(status==Status::UnusableRotation) {
            step->diagnostics.push_back("pose interpolator "+interpolator.pathText+
                " has no usable frame for its driver "+(*B.pathTexts)[d]+
                " after the pose walk; its weights are zero this generation");
            return;
        }
        if(status==Status::UnusableTranslation) {
            step->diagnostics.push_back("pose interpolator "+interpolator.pathText+
                " could not measure its driver's translation; its weights are zero this generation");
            return;
        }
        if(status==Status::CountMismatch) {
            step->diagnostics.push_back("pose interpolator "+interpolator.pathText+
                " solved "+std::to_string(interpolator.scratch.size())+" weights for "+
                std::to_string(interpolator.poseSlots.size())+" poses");
            return;
        }
        if(status==Status::Disabled) return;
        for (size_t i = 0; i < interpolator.poseSlots.size(); ++i) {
            // float, and that is load-bearing: a consumer reads inputs:weight
            // as a float, and the published property has to hold the type
            // the dynamic phase publishes.
            B.poseWeights[size_t(interpolator.poseSlots[i])] =
                static_cast<float>(interpolator.scratch[i]);
        }
        return;
    }

    default:
        return;
    }
}

bool
RigExecBakedPublishPose(RigExecBakedProgramImpl *program, RigExecRigPose *pose)
{
    RigExecBakedProgramImpl &B = *program;
    // SCC-excluded solvers did not attempt publication this generation.
    // The canonical operation inventory is the authority, not authored bindings.
    std::vector<char> aliveSolver(B.solvers.size(), 0);
    for (const RigExecBakedStep &step : B.steps)
        if (step.kind == RigExecBakedStepKind::Solve)
            aliveSolver[size_t(step.object)] = 1;
    // ORIGINAL's property prologue reported the semantic chain/part
    // inventory before pose diagnostics. Canonical graph execution may
    // interleave independent bases; reporting must not expose that order.
    std::vector<const RigExecBakedStep *> propertyDiagnostics;
    for (const RigExecBakedStep &step : B.steps) {
        if (step.kind == RigExecBakedStepKind::PropertyRevision &&
            (!step.diagnostics.empty() || !step.lines.empty())) {
            propertyDiagnostics.push_back(&step);
        }
    }
    std::sort(propertyDiagnostics.begin(), propertyDiagnostics.end(),
              [](const RigExecBakedStep *a, const RigExecBakedStep *b) {
                  return std::make_pair(a->object, a->part) <
                         std::make_pair(b->object, b->part);
              });
    for (const RigExecBakedStep *step : propertyDiagnostics) {
        pose->diagnostics.insert(pose->diagnostics.end(),
                                 step->diagnostics.begin(), step->diagnostics.end());
        pose->diagnostics.insert(pose->diagnostics.end(),
                                 step->lines.begin(), step->lines.end());
    }
    // The walk's diagnostics, in step order: commit lines, constraint lines,
    // world-up lines. They were pushed into the pose as the walk produced
    // them; they are replayed here instead, so that no step body ever touches
    // the generation and the order is program order rather than completion
    // order.
    for (const RigExecBakedStep &step : B.steps) {
        // The pose interpolators' lines come AFTER the joint block, where the
        // dynamic path's phase emits them; replayed below.
        if (RigExecBakedIsGeometryStep(step.kind) ||
            step.kind == RigExecBakedStepKind::PoseInterpolator ||
            step.kind == RigExecBakedStepKind::PropertyRevision) {
            continue;
        }
        for (const std::string &diagnostic : step.diagnostics) {
            pose->diagnostics.push_back(diagnostic);
        }
        // Memoized step lines remain observable on clean skips.
        for (const std::string &line : step.lines) pose->diagnostics.push_back(line);
    }

    // An incomplete solver is an authoring gap, not a silent one. Merged
    // here rather than accumulated during the walk, because the ORDER is a
    // property of the whole run and a step must not hold a shared one.
    // The dynamic walk's rule, expressed identically: one line per failing
    // WRITER (a stacked joint may have several, and the one that failed is
    // not necessarily the one a joint->solver lookup would name), collected
    // in walk order, stable-sorted by joint path -- and the sentence says
    // "fell back to its rest chain" only when NO writer published, because a
    // joint another writer published keeps that writer's frame and fell back
    // to nothing. The two streams are compared verbatim, so neither half of
    // this may drift from the other.
    // The element comes from jointSolverBinding rather than from
    // Solver::fallbackSlots, which holds slots only: widening it would
    // change a serialized, round-trip-verified field for a diagnostic.
    std::vector<std::pair<SdfPath, std::pair<SdfPath, int>>> fallbackJoints;
    std::map<SdfPath, SdfPath> lastPublishingWriter;
    for (const RigExecBakedProgramImpl::WalkStep &walk : B.walkSteps) {
        if (!walk.solverBatch) {
            continue;
        }
        for (const int si : walk.batchSolvers) {
            if (!aliveSolver[size_t(si)]) continue;
            const RigExecBakedProgramImpl::Solver &s = B.solvers[size_t(si)];
            for (size_t k = 0; k < s.outputs.size(); ++k) {
                const SdfPath &jointPath = B.paths[size_t(s.outputs[k].first)];
                if (k < s.outPresent.size() && s.outPresent[k]) {
                    lastPublishingWriter[jointPath] = s.path;
                    continue;
                }
                int element = -1;
                const auto binding = (*B.jointSolverBinding).find(jointPath);
                if (binding != (*B.jointSolverBinding).end()) {
                    for (const auto &[writer, writerElement] :
                         binding->second) {
                        if (writer == s.path) {
                            element = writerElement;
                            break;
                        }
                    }
                }
                fallbackJoints.push_back({jointPath, {s.path, element}});
            }
        }
    }
    std::stable_sort(fallbackJoints.begin(), fallbackJoints.end(),
                     [](const std::pair<SdfPath, std::pair<SdfPath, int>> &a,
                        const std::pair<SdfPath, std::pair<SdfPath, int>> &b) {
                         return a.first < b.first;
                     });
    for (const auto &[jointPath, writer] : fallbackJoints) {
        const auto kept = lastPublishingWriter.find(jointPath);
        pose->diagnostics.push_back(
            "solver " + writer.first.GetString() + " published no element " +
            std::to_string(writer.second) + " for joint " +
            jointPath.GetString() + "; " +
            (kept == lastPublishingWriter.end()
                 ? std::string("joint fell back to its rest chain")
                 : "the joint keeps the frame " + kept->second.GetString() +
                       " left"));
    }

    // The plain Xformables a constraint targets, in slot (== path) order.
    // The dynamic walk publishes these from one pass over its frame map,
    // BETWEEN the fallback-joint lines above and the joint publication's own
    // "degenerate final frame" lines below, and the comparator compares
    // diagnostics in order -- so the position of this loop is as much of the
    // answer as its contents are.
    for (size_t k = 0; k < B.xformSlots.size(); ++k) {
        const size_t slot = size_t(B.xformSlots[k]);
        const RigExecPointFrame &frame = B.fin[size_t(B.finLast[slot])];
        if (!RigExecBakedUsable(frame)) {
            pose->diagnostics.push_back(
                "constraint target " + B.paths[slot].GetString() +
                " has an invalid final frame; transform omitted");
            continue;
        }
        GfMatrix4d revised(1.0);
        if (RigExecPointsToMatrix(RigExecIdentityLandmarks(), frame.points,
                                  &revised)) {
            // Slot order is path order and no slot is named twice, so both
            // maps are filled strictly ascending from empty.
            RigExecBakedEmplace(&pose->providerXforms, true, B.paths[slot],
                                revised);
            RigExecBakedEmplace(&pose->providerBaseXforms, true,
                                B.paths[slot], B.xformBase[k]);
        }
    }

    {
        RIGEXEC_PROFILE_SCOPE_CAT(*B.profiler, "BakedPublish", "baked");
        // A biped frame publishes about 850 keys into five ordered maps and
        // every one of them used to be a search from the root. The two
        // passes below split that walk in half: the first decides, in the
        // publication list's own order, which joints publish a matrix and
        // which say why they do not -- the order those lines have always
        // been in -- and the second fills the maps in PATH order, where
        // every key belongs immediately past the one before it and a hint at
        // the map's end makes the insertion one comparison.
        // Deciding first also settles the bail: a frame that cannot publish
        // returns before a single key is inserted, where it used to return
        // with the maps half filled. The caller drops the pose either way.
        for (size_t k = 0; k < B.jointPaths.size(); ++k) {
            const int slot = B.jointSlots[k];
            const RigExecPointFrame &finalFrame =
                B.fin[size_t(B.finLast[size_t(slot)])];
            // The point frame is the status bearer; publishing an identity
            // matrix for a degenerate frame would let a matrix-only consumer
            // deform with a plausible-but-wrong transform.
            if (finalFrame.IsValid() && !finalFrame.IsDegenerate()) {

                B.jointMatrixPublished[k] = 1;
            } else {
                B.jointMatrixPublished[k] = 0;
                pose->diagnostics.push_back(
                    "joint " + B.jointPaths[k].GetString() +
                    " has a degenerate final frame; matrix omitted");
            }
        }
        const bool jointsInOrder = B.jointPathsAscending;
        for (const int index : B.jointPublishOrder) {
            const size_t k = size_t(index);
            const int slot = B.jointSlots[k];
            RigExecBakedEmplace(&pose->jointFramesBase, jointsInOrder,
                                B.jointPaths[k],
                                B.base[size_t(B.baseLast[size_t(slot)])]);
            RigExecBakedEmplace(&pose->jointFramesFinal, jointsInOrder,
                                B.jointPaths[k],
                                B.fin[size_t(B.finLast[size_t(slot)])]);
            if (B.jointMatrixPublished[k]) {
                // A skipped key leaves the hint at the last one that landed,
                // which is still the largest in the map: omitting entries
                // keeps the sequence ascending.
                RigExecBakedEmplace(&pose->jointMatricesFinal, jointsInOrder,
                                    B.jointPaths[k],
                                    B.finalMatrix[size_t(slot)]);
            }
        }
        // Most controls are animator inputs and publish their base frame; a
        // control a constraint names publishes the revised one. Both are the
        // LAST version of the same slot here, because the walk wrote the
        // revision into a version of it.
        for (const int index : B.controlPublishOrder) {
            const size_t k = size_t(index);
            RigExecBakedEmplace(
                &pose->controlFrames, B.controlPathsAscending,
                B.controlPaths[k],
                B.fin[size_t(B.finLast[size_t(B.controlSlots[k])])]);
        }
    }

    // Observational solver guides. The dynamic path re-evaluates them through
    // a second exec request whose per-solver override IS the aggregate the
    // walk produced, so the published array is that aggregate either way --
    // and it publishes nothing at all when the consumer disabled the guides
    // or the guide request never prepared, which this mirrors so a parity
    // check compares like with like. Read from the aggregates HERE, under the
    // runtime toggle, rather than cached in a step: either half of the toggle
    // can move without the epoch moving.
    if (*B.solverGuidesEnabled) {
        for (const int index : B.solverPublishOrder) {
            const auto &[solverPath, si] = B.solverArrays[size_t(index)];
            if (!aliveSolver[size_t(si)]) continue;
            RigExecBakedEmplace(&pose->solverFrames, B.solverArraysAscending,
                                solverPath, B.aggregates[size_t(si)].frames);
        }
    }

    // Preserve diagnostic publication order: interpolators follow joint,
    // control and guide publications and precede geometry diagnostics.
    for (const RigExecBakedStep &step : B.steps) {
        if (step.kind != RigExecBakedStepKind::PoseInterpolator) {
            continue;
        }
        for (const std::string &diagnostic : step.diagnostics) {
            pose->diagnostics.push_back(diagnostic);
        }
    }

    // Property-domain results, in the same map as the point chains: a
    // consumer tells them apart by the type the VtValue holds.
    // propertyResults is itself an ordered map and movedProperties is empty
    // until here, so this walk is always ascending -- no flag to consult.
    for (const auto &[target, value] : B.propertyResults) {
        pose->movedProperties.emplace_hint(pose->movedProperties.end(),
                                           target, value);
    }
    // The pose weights, published into BOTH places the dynamic phase
    // publishes into: movedProperties is what makes the number observable to
    // a host, a test and the picker, and the generation's resolved inputs
    // are what a consumer outside the program -- the standalone reader of a
    // blend input's connection -- resolves through after the frame.
    for (size_t k = 0; k < B.poseWeightPaths.size(); ++k) {
        const VtValue value(B.poseWeights[k]);
        pose->movedProperties[B.poseWeightPaths[k]] = value;
        B.resolvedInputs->SetProperty(B.poseWeightPaths[k], value);
    }
    return true;
}

}  // namespace rigExec
