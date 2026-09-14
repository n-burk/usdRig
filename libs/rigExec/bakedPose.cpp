//
// The baked program's pose half: the interleaved solver/constraint walk, at
// bake and at run, plus the rest->pose matrices and the frame publication it
// feeds.
//
// Split out of bakedProgram.cpp so that a domain's bake and its frame path
// are read side by side -- the two have to agree about what was captured and
// what is re-read, and that agreement is the whole correctness argument. The
// evaluator is not visible here: everything this file needs of it was
// captured into RigExecBakedProgramImpl at Build (see bakedProgramImpl.h),
// and the compiled walk arrives restated as RigExecBakedWalkEntry.
//
#include "bakedProgramImpl.h"

#include "frameExtraction.h"
#include "moverGraph.h"
#include "rigEvaluator.h"
#include "solverKernels.h"
#include "types.h"

#include "rigExecMath/pointFrame.h"
#include "rigExecMath/solvers.h"
#include "rigExecMath/splineIk.h"

#include "pxr/base/gf/math.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/metrics.h"
#include "pxr/base/work/threadLimits.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace rigExec {

// ---------------------------------------------------------------------------
// Bake.
// ---------------------------------------------------------------------------

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
    auto readToken = [&](const UsdPrim &prim, const char *name,
                         const char *fallback) {
        return ctx->ReadToken(prim, name, fallback);
    };
    auto targets = [&](const UsdPrim &prim, const char *name) {
        return ctx->Targets(prim, name);
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
                                RigExecBakedSlotKind::PoseSeed
                   ? slot
                   : -1;
    };

    auto bakeSolver = [&](const SdfPath &solverPath,
                          const std::vector<std::pair<SdfPath, int>>
                              &jointOutputs) {
        RigExecBakedProgramImpl::Solver s;
        s.path = solverPath;
        const UsdPrim prim = B.stage->GetPrimAtPath(solverPath);
        s.type = prim.GetTypeName();

        // rigExec:joints is the single declaration of the chain: a solver
        // whose output is consumed downstream claims no joints but still
        // names them for their REST measurements.
        std::vector<std::pair<int, int>> restRefs;
        {
            const SdfPathVector joints = targets(prim, "rigExec:joints");
            VtIntArray elements;
            fold(prim, "rigExec:jointElements");
            if (const UsdAttribute a =
                    prim.GetAttribute(TfToken("rigExec:jointElements"))) {
                a.Get(&elements);
            }
            // exec binds ONE rest input per rigExec:joints target that
            // PUBLISHES computeRestFrame -- a target that publishes none
            // contributes no input at all -- and the computation then
            // compares the AUTHORED jointElements length against THAT count
            // and indexes the remap by position within it (the TwoBoneIk and
            // SplineIk rest loops in computations.cpp). Resolving against the
            // relationship's target count instead would remap by the wrong
            // index, and would keep remapping where exec gives up.
            //
            // A PoseSeed slot is what "publishes computeRestFrame" means: an
            // xform-derived slot exists in the table so a constraint can name
            // it, but exec seeds it from the stage and it declares no rest
            // computation, so it contributes no input here either.
            std::vector<int> restSlots;
            for (const SdfPath &joint : joints) {
                const int slot = slotOf(joint);
                if (slot >= 0 && B.slotKind[size_t(slot)] ==
                                     RigExecBakedSlotKind::PoseSeed) {
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
        }
        for (const auto &[joint, element] : jointOutputs) {
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
                s.controlRests.push_back(B.restPts[slot]);
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
                s.ikRests[size_t(element)] = B.restPts[slot];
                seen[size_t(element)] = true;
            }
            if (!(seen[0] && seen[1] && seen[2])) {
                s.degenerate = true;
            }
            // The bone lengths exec measures every evaluation, measured once.
            s.upperLengthBase = (s.ikRests[1][0] - s.ikRests[0][0]).GetLength();
            s.lowerLengthBase = (s.ikRests[2][0] - s.ikRests[1][0]).GetLength();
            s.bend = bind(prim, "rigExec:preferredBendRadians", 0.0);
            s.upperOffset = bind(prim, "rigExec:upperLengthOffset", 0.0);
            s.lowerOffset = bind(prim, "rigExec:lowerLengthOffset", 0.0);
            s.stretch = bind(prim, "inputs:stretch", 1.0f);
            s.softness = bind(prim, "inputs:softness", 0.0f);
            s.ikParams.preferredBendRadians = s.bend.constant;
            s.ikParams.stretch = s.stretch.constant;
            s.ikParams.softness = s.softness.constant;
            s.ikParams.upperLength =
                s.upperLengthBase + s.upperOffset.constant;
            s.ikParams.lowerLength =
                s.lowerLengthBase + s.lowerOffset.constant;
        } else if (s.type == "RigExecBlendPointFrames") {
            auto solverSlot = [&](const SdfPathVector &v) {
                if (v.empty()) return -1;
                const auto it = B.solverIndex.find(v[0]);
                return it == B.solverIndex.end() ? -1 : it->second;
            };
            // -1 is the computation's null pointer, and it means exactly
            // what a null pointer means there: one missing input passes the
            // OTHER through unchanged, rests and all, and two missing
            // inputs publish nothing. It can never mean "not baked yet" --
            // an inputA/inputB solver is a dependency, so the Kahn levels
            // put it in an earlier batch.
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
                restJoints[size_t(element)] = B.restFrames[slot];
                s.splineJointRests[size_t(element)] = B.restPts[slot];
            }
            std::vector<double> weights;
            VtFloatArray authoredWeights;
            fold(prim, "rigExec:volumeWeights");
            if (const UsdAttribute a =
                    prim.GetAttribute(TfToken("rigExec:volumeWeights"))) {
                a.Get(&authoredWeights);
            }
            for (float w : authoredWeights) weights.push_back(w);
            if (!weights.empty() && weights.size() != count) {
                s.degenerate = true;
            }
            const TfToken restTok = readToken(prim, "rigExec:restLength", "");
            if (!restTok.IsEmpty() && restTok != "curve" &&
                restTok != "chain") {
                s.degenerate = true;
            }
            // Exec rebuilds this description every evaluation; it is a pure
            // function of epoch-constant rests, so it bakes out.
            s.splineRest = RigExecSplineIkMakeRest(
                restJoints,
                s.root >= 0 ? B.restFrames[s.root] : RigExecPointFrame(),
                s.mid >= 0 ? B.restFrames[s.mid] : RigExecPointFrame(),
                s.end >= 0 ? B.restFrames[s.end] : RigExecPointFrame(),
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
            //
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
                s.twistStartRest = B.restPts[size_t(s.root)];
                s.twistEndRest = B.restPts[size_t(s.end)];
            }
            // rigExec:weights and rigExec:count define the frame cardinality,
            // which compile refuses to let vary with time, so both are read
            // once and folded. The float -> double widening is element-wise
            // and in array order, as the computation's read iterator does it.
            VtFloatArray authoredWeights;
            fold(prim, "rigExec:weights");
            if (const UsdAttribute a =
                    prim.GetAttribute(TfToken("rigExec:weights"))) {
                a.Get(&authoredWeights);
            }
            for (float w : authoredWeights) s.twistWeights.push_back(w);
            int count = 1;
            fold(prim, "rigExec:count");
            if (const UsdAttribute a =
                    prim.GetAttribute(TfToken("rigExec:count"))) {
                a.Get(&count);
            }
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
            if (const UsdAttribute a =
                    B.stage->GetAttributeAtPath(s.ribbonPointsPath)) {
                // Read at Default, and left empty when nothing answers
                // there: a curve carrying only time samples has no bind-time
                // value, and an empty rest is what makes the sampler publish
                // nothing at all.
                VtVec3fArray rest;
                a.Get(&rest, UsdTimeCode::Default());
                s.ribbonRestPoints.assign(rest.begin(), rest.end());
                s.ribbonPointsVarying = a.ValueMightBeTimeVarying() ||
                                        a.GetNumTimeSamples() > 0;
                if (s.ribbonPointsVarying) {
                    s.ribbonPointsQuery = UsdAttributeQuery(a);
                } else {
                    // One value at every time code, so the prologue has
                    // nothing to read and the cone nothing to compare.
                    VtVec3fArray live;
                    a.Get(&live, ctx->capture);
                    s.ribbonConstantPoints.assign(live.begin(), live.end());
                }
                // Registered BY PATH rather than through fold(prim, name):
                // the driver curve is another prim entirely, and the target
                // may be an arbitrary property path. The rest capture makes
                // it folded -- a value edit on the curve rebuilds, and an
                // override on it cannot be placed, which is right because
                // the dynamic path would ignore that override.
                //
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
            // endFrame, twistFrames and driverCurveReadPhase are
            // deliberately NOT read: the computation does not read them
            // either -- they shape batching and the epoch digest -- and
            // folding one would rebuild the program for an edit that cannot
            // change what it publishes.
        }
        const int slot = int(B.solvers.size());
        B.solverIndex[solverPath] = slot;
        B.solvers.push_back(std::move(s));
        return slot;
    };

    // ---- constraints --------------------------------------------------------
    auto bakeConstraint = [&](const RigExecBakedConstraintSpec &fc) {
        RigExecBakedProgramImpl::Constraint c;
        c.path = fc.moverPath;
        c.type = fc.schemaType;
        c.snapshotAfter = fc.snapshotAfter;
        const UsdPrim prim = B.stage->GetPrimAtPath(fc.moverPath);
        c.target = fc.targets.empty() ? -1 : slotOf(fc.targets[0]);
        for (const SdfPath &source : fc.sources) {
            const int slot = slotOf(source);
            if (slot < 0) {
                refuse("constraint source is not a pose provider", source);
            }
            c.sources.push_back(slot);
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

        // inputs:sourceWeights / the parent offsets are authored tables, read
        // as arrays rather than as a per-source scalar.
        const size_t n = c.sources.size();
        VtFloatArray weights;
        fold(prim, "inputs:sourceWeights");
        if (const UsdAttribute a =
                prim.GetAttribute(TfToken("inputs:sourceWeights"))) {
            if (RigExecBakedAnimatedOrConnected(a)) {
                refuse("animated inputs:sourceWeights", fc.moverPath);
            }
            a.Get(&weights);
            ++B.boundInputs;
        }
        c.authoredSourceWeights = weights.size();
        c.sourceWeights.resize(n);
        for (size_t k = 0; k < n; ++k) {
            c.sourceWeights[k].constant = k < weights.size() ? weights[k] : 1.0f;
        }
        if (!weights.empty() && weights.size() != n) {
            // The dynamic path diagnoses this and passes the constraint
            // through; the program refuses instead of reproducing a
            // malformed-input message from baked state.
            refuse("inputs:sourceWeights cardinality", fc.moverPath);
        }
        c.translationOffsets.assign(n, GfVec3d(0));
        c.rotationOffsets.assign(n, GfVec3d(0));
        if (fc.schemaType == "RigExecParentConstraint") {
            VtVec3dArray translations, rotations;
            for (const char *name : {"inputs:translationOffsets",
                                     "inputs:rotationOffsets"}) {
                fold(prim, name);
                if (const UsdAttribute a = prim.GetAttribute(TfToken(name))) {
                    if (RigExecBakedAnimatedOrConnected(a)) {
                        refuse(std::string("animated ") + name, fc.moverPath);
                    }
                    ++B.boundInputs;
                }
            }
            if (const UsdAttribute a = prim.GetAttribute(
                    TfToken("inputs:translationOffsets"))) {
                a.Get(&translations);
            }
            if (const UsdAttribute a =
                    prim.GetAttribute(TfToken("inputs:rotationOffsets"))) {
                a.Get(&rotations);
            }
            if ((!translations.empty() && translations.size() != n) ||
                (!rotations.empty() && rotations.size() != n)) {
                refuse("parent constraint offset cardinality", fc.moverPath);
            }
            for (size_t k = 0; k < translations.size() && k < n; ++k) {
                c.translationOffsets[k] = translations[k];
            }
            for (size_t k = 0; k < rotations.size() && k < n; ++k) {
                c.rotationOffsets[k] = rotations[k];
            }
        }
        c.order = RigExecBakedParseEulerOrder(
            readToken(prim, "rigExec:rotationOrder", "XYZ"));

        if (fc.schemaType == "RigExecPositionConstraint") {
            c.affectX = bind(prim, "inputs:affectTranslationX", true);
            c.affectY = bind(prim, "inputs:affectTranslationY", true);
            c.affectZ = bind(prim, "inputs:affectTranslationZ", true);
            c.offset = bind(prim, "inputs:translationOffset", GfVec3d(0));
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
            c.worldUpObject =
                c.worldUpObjectNamed ? slotOf(fc.worldUpObject) : -1;
            if (c.worldUpObjectNamed && c.worldUpObject < 0) {
                refuse("aim world-up object is not a pose provider",
                       fc.worldUpObject);
            }
        }
        B.constraints.push_back(std::move(c));
        return int(B.constraints.size()) - 1;
    };

    // ---- descendant propagation, decided once -------------------------------
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
                if (B.slotKind[size_t(j)] != RigExecBakedSlotKind::PoseSeed) {
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

    // ---- the walk ------------------------------------------------------------
    for (const RigExecBakedWalkEntry &entry : walk) {
        RigExecBakedProgramImpl::WalkStep st;
        st.solverBatch = entry.solverBatch;
        if (entry.solverBatch) {
            st.level = entry.level;
            std::vector<int> candidates;
            for (size_t k = 0; k < entry.batchSolvers.size(); ++k) {
                const int slot =
                    bakeSolver(entry.batchSolvers[k], entry.solverJoints[k]);
                st.batchSolvers.push_back(slot);
                for (const auto &[providerSlot, element] :
                         B.solvers[slot].outputs) {
                    candidates.push_back(providerSlot);
                }
            }
            buildPropagation(candidates, &st.propagate);
        } else {
            st.index = bakeConstraint(entry.constraint);
            if (B.constraints[st.index].target >= 0) {
                buildPropagation({B.constraints[st.index].target},
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
        B.guideSolvers.push_back(bakeSolver(solverPath, {}));
    }
    B.aggregates.resize(B.solvers.size());
}


// ---------------------------------------------------------------------------
// Build: the pose half of the program, in program order.
//
// Program order is today's straight line. What changes is that each piece of
// it now says which slots it reads and which it writes, so that the edges
// between the pieces follow from the declarations rather than from the order
// -- and the order becomes one valid schedule instead of the only one.
// ---------------------------------------------------------------------------

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
        solver.rootRead = readFin(solver.root);
        solver.midRead = readFin(solver.mid);
        solver.endRead = readFin(solver.end);
        solver.poleRead = readFin(solver.pole);
    };

    for (size_t w = 0; w < B.walkSteps.size(); ++w) {
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

/// A commit split into delta / staging / apply once its descendant list is
/// this long. Below it the three phases run as one step: the split buys
/// parallelism inside one propagation and costs two more steps, which is a
/// bad trade for the two- and three-descendant commits a rig is full of.
constexpr size_t kPropagateSplitThreshold = 64;
/// Descendants staged per PropagateChunk of a split commit.
constexpr size_t kPropagateChunkSize = 64;

}  // namespace

void
RigExecBakedBuildPoseSteps(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    const int N = int(B.paths.size());

    // ---- compose, partitioned into subtrees ---------------------------------
    //
    // One step per provider would be ~2 500 steps on a biped for work that is
    // a few hundred nanoseconds each. The partition cuts the provider forest
    // into subtrees of about total/(4P) slots -- enough of them that the work
    // spreads, few enough that a step is worth its edge. Slots are in
    // namespace DFS pre-order, so a subtree is CONTIGUOUS: the whole compose
    // pass is a sequence of adjacent ranges in increasing order, and each
    // range's parents are either inside it or below its first slot.
    {
        std::vector<int> subtreeSize(size_t(N), 1);
        for (int i = N - 1; i >= 0; --i) {
            if (B.propParent[size_t(i)] >= 0) {
                subtreeSize[size_t(B.propParent[size_t(i)])] +=
                    subtreeSize[size_t(i)];
            }
        }
        const int concurrency = std::max(1, int(WorkGetConcurrencyLimit()));
        const int budget =
            std::max(1, N / std::max(1, 4 * concurrency));
        std::vector<int> starts;
        for (int i = 0; i < N;) {
            // The shallowest node whose whole subtree fits is a cut; one
            // whose subtree does not starts a group of its own and the scan
            // continues into its children, which is what "cut at the deepest
            // nodes whose subtree exceeds the budget" comes to.
            starts.push_back(i);
            i += subtreeSize[size_t(i)] <= budget ? subtreeSize[size_t(i)] : 1;
        }
        // Then merge ADJACENT groups while the result is still inside the
        // budget. Without this an internal node too big to be its own
        // subtree becomes a one-slot step, and a biped -- whose controls
        // hang off a handful of deep scopes -- ends up with a step per
        // provider for exactly the work the budget exists to avoid. Merging
        // is always sound: slots are in namespace DFS pre-order, so any
        // contiguous range's parents are either inside it or below its first
        // slot.
        for (size_t k = 0; k < starts.size(); ++k) {
            RigExecBakedComposeGroup group;
            group.begin = starts[k];
            group.end = k + 1 < starts.size() ? starts[k + 1] : N;
            while (k + 1 < starts.size() &&
                   (k + 2 < starts.size() ? starts[k + 2] : N) -
                           group.begin <= budget) {
                ++k;
                group.end = k + 1 < starts.size() ? starts[k + 1] : N;
            }
            for (int slot = group.begin; slot < group.end; ++slot) {
                const int parent = B.parent[size_t(slot)];
                if (parent >= 0 && parent < group.begin) {
                    group.parentSlots.push_back(parent);
                }
            }
            std::sort(group.parentSlots.begin(), group.parentSlots.end());
            group.parentSlots.erase(
                std::unique(group.parentSlots.begin(),
                            group.parentSlots.end()),
                group.parentSlots.end());
            B.composeGroups.push_back(std::move(group));
        }
    }
    for (size_t g = 0; g < B.composeGroups.size(); ++g) {
        const RigExecBakedComposeGroup &group = B.composeGroups[g];
        RigExecBakedStep &step = AddStep(
            &B, RigExecBakedStepKind::ComposeSubtree, int(g));
        step.reads.push_back(RigExecBakedRange(
            RigExecBakedSlotDomain::Avars, group.begin * 11, group.end * 11));
        for (const int parent : group.parentSlots) {
            step.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::PosedM, parent));
        }
        step.writes.push_back(RigExecBakedRange(
            RigExecBakedSlotDomain::PoseBase, group.begin, group.end));
        step.writes.push_back(RigExecBakedRange(
            RigExecBakedSlotDomain::PoseFin, group.begin, group.end));
        step.writes.push_back(RigExecBakedRange(
            RigExecBakedSlotDomain::PosedM, group.begin, group.end));
    }

    // ---- the interleaved solver/constraint walk -----------------------------
    B.commits.resize(B.walkSteps.size());
    std::set<size_t> levels;
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
            levels.insert(walk.level);
            B.solverEvaluations += walk.batchSolvers.size();
            for (const int si : walk.batchSolvers) {
                for (const auto &[slot, element] : B.solvers[size_t(si)]
                                                       .outputs) {
                    commit.slots.push_back(slot);
                }
            }
        } else {
            commit.moverPath = B.constraints[size_t(walk.index)].path;
            if (B.constraints[size_t(walk.index)].target >= 0) {
                commit.slots.push_back(
                    B.constraints[size_t(walk.index)].target);
            }
            commit.sources.resize(
                B.constraints[size_t(walk.index)].sources.size());
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
                solver.elements.resize(solver.controls.size());
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
                // The ribbon's driver points, filled by the prologue.
                // Nothing in the program writes the domain, so the read
                // raises no edge; it is declared because it is what the cone
                // follows when the curve moves, and because a step's
                // declaration is meant to say what the step touches.
                if (!solver.ribbonPointsPath.IsEmpty()) {
                    step.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::SolverPoints, si));
                }
                // A BlendPointFrames input solver normally runs in an
                // earlier batch; when it does not, the read is of last run's
                // aggregate, which is why Aggregate is a source domain.
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
            if (constraint.worldUpObject >= 0) {
                commitStep.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::PoseFin,
                    constraint.worldUpObject));
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
            if (!walk.solverBatch) {
                commitStep.writes.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::Snapshots,
                    int(B.steps.size()) - 1));
            }
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
            if (!walk.solverBatch) {
                apply.writes.push_back(
                    RigExecBakedOne(RigExecBakedSlotDomain::Snapshots,
                                    int(B.steps.size()) - 1));
            }
        }
    }
    B.solverOverrideRounds = levels.size();
    // Before the matrices, which read the LAST version of a slot: the table
    // that says which entry that is comes out of this sweep.
    BindPoseVersions(&B);

    // ---- the solvers no batch runs ------------------------------------------
    //
    // The dynamic path answers these from a second exec request, whose
    // per-provider override is the FINAL frame rather than the mid-walk one
    // a batch sees -- so they run HERE, after the whole walk, and read the
    // last version of everything. They write no candidate and bump no
    // counter: a solver that binds a joint or drives geometry is required
    // and therefore batched, so one of these poses nothing and the only
    // thing that reads it is the guide publication.
    //
    // Not gated on the guide toggle. The toggle can move without the epoch
    // moving, so gating would be a runtime branch in a step body; running
    // and not publishing is the same published generation, because
    // RigExecBakedRead only READS the override table and the epilogue
    // consults the toggle where the dynamic path does.
    //
    // The one thing it does move with the guides off is the static-input
    // cache: RigExecBakedRead's resolved path goes through
    // RigExecResolvedInputs::GetAttribute, whose hit/miss/bypass counters
    // are an observable of their own (spec 5.2 [P37][S36]). Nothing in the
    // pose can see it, and no rig asserts on those counters for a
    // guide-only solver -- but anyone measuring that cache should know the
    // guide pass is a client of it whether the guides are drawn or not.
    for (const int si : B.guideSolvers) {
        RigExecBakedProgramImpl::Solver &solver = B.solvers[size_t(si)];
        solver.elements.resize(solver.controls.size());
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

    // ---- the rest->pose matrices --------------------------------------------
    //
    // Exactly the set today's lazy finalMatrixOf/baseMatrixOf computed:
    // final for every published joint, and final or base per use for every
    // matrix and influence a geometry revision reads. A ProviderMatrix step
    // never gives the generation back -- RigExecPointsToMatrix leaves the
    // identity and says so -- so the one bail of this phase stays where it
    // is, in the joint publication.
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
        std::vector<char> &table = revision.finalPhase ? B.needFinal
                                                       : B.needBase;
        if (revision.transformSlot >= 0) {
            table[size_t(revision.transformSlot)] = 1;
        }
        for (const int slot : revision.influenceSlots) {
            if (slot >= 0) {
                table[size_t(slot)] = 1;
            }
        }
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

    // The store's other pose-half record: every provider's rest -> final
    // matrix, which is what a `final` phase on a provider resolves to. Only
    // a rig that declares a phase can look one up, so only such a rig pays
    // for the step at all.
    if (B.phasedReads) {
        RigExecBakedStep &step =
            AddStep(&B, RigExecBakedStepKind::SnapshotFinals, 0);
        step.reads.push_back(
            RigExecBakedRange(RigExecBakedSlotDomain::PoseFin, 0, N));
        step.writes.push_back(RigExecBakedOne(
            RigExecBakedSlotDomain::Snapshots, int(B.steps.size()) - 1));
    }
}

// ---------------------------------------------------------------------------
// One frame, pose half.
//
// Everything below is a STEP BODY or one half of the serial prologue or
// epilogue. A body touches no pose, takes no lock, opens no profile scope and
// writes only the slots its step declared; whatever it has to say it says
// into its own step.
// ---------------------------------------------------------------------------

namespace {

/// Records \p input against \p step: what a frame can move it with.
///
/// The rule itself is RigExecBakedNoteInput in bakedProgramImpl.h, because
/// the weight half declares its inputs the same way and two spellings of
/// "what can move this" is a step the cone cannot dirty.
template <class T>
void
NoteInput(const RigExecBakedInput<T> &input, RigExecBakedStep *step)
{
    RigExecBakedNoteInput(input, step);
}

/// Every per-frame input one solver's Solve step reads.
void
NoteSolverInputs(const RigExecBakedProgramImpl::Solver &solver,
                 RigExecBakedStep *step)
{
    NoteInput(solver.bend, step);
    NoteInput(solver.upperOffset, step);
    NoteInput(solver.lowerOffset, step);
    NoteInput(solver.stretch, step);
    NoteInput(solver.softness, step);
    NoteInput(solver.blendWeight, step);
    NoteInput(solver.preserveVolume, step);
    NoteInput(solver.midFollowWeight, step);
    NoteInput(solver.roll, step);
    NoteInput(solver.twist, step);
    NoteInput(solver.minLengthRatio, step);
    NoteInput(solver.twistTurns, step);
    NoteInput(solver.ribbonSampleCount, step);
    // The spline parameters the bake could not fold, which the solve re-reads
    // as a group rather than one input at a time.
    step->varyingInputs = step->varyingInputs || solver.splineParamsVary;
}

/// Every per-frame input one constraint's step reads.
void
NoteConstraintInputs(const RigExecBakedProgramImpl::Constraint &constraint,
                     RigExecBakedStep *step)
{
    NoteInput(constraint.enabled, step);
    NoteInput(constraint.defaultWeight, step);
    NoteInput(constraint.offset, step);
    NoteInput(constraint.affectX, step);
    NoteInput(constraint.affectY, step);
    NoteInput(constraint.affectZ, step);
    NoteInput(constraint.tX, step);
    NoteInput(constraint.tY, step);
    NoteInput(constraint.tZ, step);
    NoteInput(constraint.rX, step);
    NoteInput(constraint.rY, step);
    NoteInput(constraint.rZ, step);
    NoteInput(constraint.sX, step);
    NoteInput(constraint.sY, step);
    NoteInput(constraint.sZ, step);
    NoteInput(constraint.aimVector, step);
    NoteInput(constraint.upVector, step);
    NoteInput(constraint.rotationOffset, step);
    NoteInput(constraint.worldUpVector, step);
    // The authored source-weight and offset tables are folded -- an animated
    // one is refused at bake -- so there is nothing per-frame about them.
}

}  // namespace

void
RigExecBakedDeclareInputDependencies(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    for (RigExecBakedStep &step : B.steps) {
        step.varyingInputs = false;
        step.resolvedInputReads = false;
        step.overrideInputs.clear();
        switch (step.kind) {
        case RigExecBakedStepKind::Solve:
            NoteSolverInputs(B.solvers[size_t(step.object)], &step);
            break;
        case RigExecBakedStepKind::Constraint: {
            // A commit step and its walk entry are the same index, and a
            // constraint entry names the constraint it commits.
            const RigExecBakedProgramImpl::WalkStep &walk =
                B.walkSteps[size_t(step.object)];
            if (!walk.solverBatch && walk.index >= 0) {
                NoteConstraintInputs(B.constraints[size_t(walk.index)], &step);
            }
            break;
        }
        case RigExecBakedStepKind::WeightPacket:
            RigExecBakedNoteWeightInputs(
                B.weightObjects[size_t(step.object)], &step);
            break;
        default:
            break;
        }
        std::sort(step.overrideInputs.begin(), step.overrideInputs.end());
        step.overrideInputs.erase(
            std::unique(step.overrideInputs.begin(),
                        step.overrideInputs.end()),
            step.overrideInputs.end());
    }
}

void
RigExecBakedRunInputs(RigExecBakedProgramImpl *program, UsdTimeCode time)
{
    RigExecBakedProgramImpl &B = *program;
    const RigExecResolvedInputs &R = *B.resolvedInputs;
    RIGEXEC_PROFILE_SCOPE_CAT(*B.profiler, "BakedInputs", "baked");
    for (const auto &binding : B.avarBindings) {
        B.avars[binding.slot] =
            RigExecBakedRead(binding.input, R, time, &B.overridden);
    }
    // A drag lands on avars the bake captured as constants -- that is what
    // dragging a control on a still rig IS -- so the varying list above is not
    // the whole table while one stands. The constant slots are walked while a
    // drag stands and once more after it is released, because the released
    // slot holds the dragged value until something writes the constant back
    // over it. Once per run: the pass and the flag update are the prologue's,
    // not a step's, so no arrangement of the graph can perform them twice.
    if (B.anyOverridden || B.avarsDisturbed) {
        for (const auto &binding : B.avarConstantBindings) {
            B.avars[binding.slot] =
                B.overridden[size_t(binding.input.overrideIndex)]
                    ? RigExecBakedRead(binding.input, R, time, &B.overridden)
                    : binding.input.constant;
        }
        B.avarsDisturbed = B.anyOverridden;
    }
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
            RigExecPointsToMatrix(
                B.fin[size_t(commit->slotReads[pos])].points,
                commit->frames[pos].points, &commit->deltas[pos])
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
        if (pos < 0 || !commit->present[size_t(pos)]) {
            // A solver published no element for this ancestor, so the baked
            // propagation pairs no longer describe the walk.
            commit->outcome[k] =
                uint8_t(RigExecBakedPropagateOutcome::NoCandidate);
            continue;
        }
        const RigExecPointFrame &current =
            B.fin[size_t(commit->descendantReads[k])];
        const RigExecPointFrame &before =
            B.fin[size_t(commit->closestReads[k])];
        if (commit->solverOutput &&
            (!RigExecBakedUsable(current) || !RigExecBakedUsable(before) ||
             !RigExecBakedUsable(commit->frames[size_t(pos)]))) {
            commit->outcome[k] =
                uint8_t(RigExecBakedPropagateOutcome::Skipped);
            continue;
        }
        if (!RigExecBakedUsable(current)) {
            commit->outcome[k] =
                uint8_t(RigExecBakedPropagateOutcome::UnusableDescendant);
            continue;
        }
        if (commit->deltaOk[size_t(pos)] != 1) {
            commit->outcome[k] =
                uint8_t(RigExecBakedPropagateOutcome::SingularDelta);
            continue;
        }
        const RigExecPointFrame frame =
            RigExecMatrixToPoints(current.points, commit->deltas[size_t(pos)]);
        if (!RigExecBakedUsable(frame)) {
            commit->outcome[k] =
                uint8_t(RigExecBakedPropagateOutcome::InvalidResult);
            continue;
        }
        commit->staged[k] = frame;
        commit->outcome[k] = uint8_t(RigExecBakedPropagateOutcome::Staged);
    }
}

/// The phased-read store's pose-half record: a provider's rest -> final
/// matrix as it stood immediately AFTER one constraint. Mirrors the dynamic
/// walk's recordFrame, including which frames it declines to record; the
/// _snapshotPoints membership it tests per call was decided at bake, so a rig
/// that declares no phase pays one branch per constraint.
void
RecordFrame(const RigExecBakedProgramImpl &B,
            const RigExecBakedProgramImpl::Constraint &constraint,
            uint32_t targetVersion, RigExecBakedStep *step)
{
    if (!constraint.snapshotAfter || constraint.target < 0) {
        return;
    }
    // The target as THIS commit left it, which is the commit's own version
    // of the slot whether it revised it or carried the one it found: a
    // phase names a point in the walk, and a constraint that passed through
    // still leaves its target standing at that point.
    const RigExecPointFrame &frame = B.fin[size_t(targetVersion)];
    if (!frame.IsValid()) {
        return;
    }
    const RigExecPointFrame &rest = B.restFrames[size_t(constraint.target)];
    const std::array<GfVec3d, 4> landmarks =
        rest.IsValid() ? rest.points : RigExecIdentityLandmarks();
    GfMatrix4d matrix(1.0);
    if (RigExecPointsToMatrix(landmarks, frame.points, &matrix)) {
        step->snapshots.Record(B.paths[size_t(constraint.target)],
                               constraint.path, VtValue(matrix));
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
    const RigExecBakedProgramImpl::Constraint *constraint =
        commit->solverOutput
            ? nullptr
            : &B.constraints[size_t(
                  B.walkSteps[size_t(step->object)].index)];
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
    const auto record = [&] {
        if (constraint) {
            RecordFrame(B, *constraint,
                        commit->slotWrites.empty() ? 0
                                                   : commit->slotWrites[0],
                        step);
        }
    };
    if (commit->abandoned) {
        carryEverything();
        record();
        return;
    }
    const std::string mover = commit->moverPath.GetString();
    for (size_t k = 0; k < commit->propagate.size(); ++k) {
        const auto outcome = RigExecBakedPropagateOutcome(commit->outcome[k]);
        if (outcome == RigExecBakedPropagateOutcome::Staged ||
            outcome == RigExecBakedPropagateOutcome::Skipped) {
            continue;
        }
        switch (outcome) {
        case RigExecBakedPropagateOutcome::NoCandidate:
            // The generation is given back here, and the caller destroys the
            // program, so nothing will ever read these versions -- but a
            // step that leaves storage it declared unwritten is a rule with
            // an exception, and this one is not worth having.
            carryEverything();
            step->bail = true;
            return;
        case RigExecBakedPropagateOutcome::UnusableDescendant:
            step->diagnostics.push_back(
                mover + " could not propagate its pose revision through " +
                B.paths[size_t(commit->propagate[k].first)].GetString() +
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
                B.paths[size_t(commit->propagate[k].first)].GetString() +
                "; constraint passed through");
            break;
        }
        carryEverything();
        record();
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
    record();
}

}  // namespace

void
RigExecBakedRunPoseStep(RigExecBakedProgramImpl *program,
                        RigExecBakedStep *step, UsdTimeCode time)
{
    RigExecBakedProgramImpl &B = *program;
    const RigExecResolvedInputs &R = *B.resolvedInputs;
    // Every per-frame read goes through these two: `rd` reads one input the
    // way this generation must (through the resolved inputs while an override
    // stands on it, through the pinned query otherwise), and `live` answers
    // whether a set of baked parameters has to be re-read at all.
    const auto rd = [&](const auto &input) {
        return RigExecBakedRead(input, R, time, &B.overridden);
    };
    const auto live = [&](const auto &input) {
        return input.varying ||
               (input.overrideIndex >= 0 &&
                B.overridden[size_t(input.overrideIndex)]);
    };

    switch (step->kind) {
    case RigExecBakedStepKind::ComposeSubtree: {
        const RigExecBakedComposeGroup &group =
            B.composeGroups[size_t(step->object)];
        for (int i = group.begin; i < group.end; ++i) {
            if (B.slotKind[size_t(i)] != RigExecBakedSlotKind::PoseSeed) {
                // An xform-derived slot is not composed from avars: the
                // dynamic path seeds it from the stage, and until the program
                // does the same nothing writes it.
                continue;
            }
            const double *a = &B.avars[size_t(i) * 11];
            const double units = a[10];
            // A volume weight's placement is RIGID: its shape is
            // inputs:scaleX/Y/Z's alone, so the transform-scale avars are
            // read and discarded here rather than zeroed at bake -- exec
            // never binds them at all, and a captured zero would be walked
            // straight past by an override or an animated channel.
            const bool noScale = B.noScaleAvars[size_t(i)] != 0;
            const GfMatrix4d avars = RigExecBakedComposeAvars(
                a[0] * units, a[1] * units, a[2] * units,
                noScale ? 1.0 : a[3], noScale ? 1.0 : a[4],
                noScale ? 1.0 : a[5],
                a[6], a[7], a[8], a[9], B.rotOrder[size_t(i)]);
            const GfMatrix4d parentPosed =
                B.parent[size_t(i)] >= 0
                    ? B.posedM[size_t(B.parent[size_t(i)])]
                    : GfMatrix4d(1.0);
            B.base[size_t(i)] = RigExecFrameFromMatrix(
                avars * B.selfD[size_t(i)] * B.parentDinv[size_t(i)] *
                parentPosed);
            B.fin[size_t(i)] = B.base[size_t(i)];
            // _SpaceFromFrame: an unusable frame selects the NaN sentinel, so
            // the failure survives into every descendant instead of being
            // scrubbed into a plausible identity.
            GfMatrix4d space(1.0);
            if (!B.base[size_t(i)].IsValid() ||
                B.base[size_t(i)].IsDegenerate() ||
                !RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                       B.base[size_t(i)].points, &space)) {
                space = GfMatrix4d(1.0);
                space[3][0] = std::numeric_limits<double>::quiet_NaN();
            }
            B.posedM[size_t(i)] = space;
        }
        return;
    }

    case RigExecBakedStepKind::Solve: {
        RigExecBakedProgramImpl::Solver &s = B.solvers[size_t(step->object)];
        RigExecPointFrameArray &aggregate = B.aggregates[size_t(step->object)];
        aggregate.frames.clear();
        aggregate.rests.clear();
        s.fallbackJoints.clear();
        // The one degeneracy guard, and it sits AFTER the clear: a solver
        // whose joint binding the bake found malformed publishes the EMPTY
        // aggregate its computation returns, and "publishes an empty
        // aggregate" is what the clear makes true. The output loop below
        // still runs, so every joint the solver names falls back to its rest
        // chain and says so -- which is exactly what the dynamic path
        // reports.
        if (s.degenerate) {
        } else if (s.type == "RigExecFkChain") {
            for (size_t k = 0; k < s.controls.size(); ++k) {
                s.elements[k].restPoints = s.controlRests[k];
                // Every entry is a provider: the bake filtered the list
                // the way the computation's read iterator does.
                s.elements[k].posePoints =
                    B.fin[size_t(s.controlReads[k])].points;
                s.elements[k].parentIndex =
                    s.parentRelative ? -1 : int(k) - 1;
            }
            aggregate.frames = RigExecSolveFkChain(s.elements);
            aggregate.rests = s.controlRests;
        } else if (s.type == "RigExecTwoBoneIk") {
            RigExecTwoBoneIkParams params = s.ikParams;
            if (live(s.bend) || live(s.stretch) || live(s.softness) ||
                live(s.upperOffset) || live(s.lowerOffset)) {
                params.preferredBendRadians = rd(s.bend);
                params.stretch = rd(s.stretch);
                params.softness = rd(s.softness);
                params.upperLength = s.upperLengthBase + rd(s.upperOffset);
                params.lowerLength = s.lowerLengthBase + rd(s.lowerOffset);
            }
            const auto frames = RigExecSolveTwoBoneIk(
                B.fin[size_t(s.rootRead)], B.fin[size_t(s.endRead)],
                B.fin[size_t(s.poleRead)], s.ikRests, params);
            aggregate.frames.assign(frames.begin(), frames.end());
            aggregate.rests.assign(s.ikRests.begin(), s.ikRests.end());
        } else if (s.type == "RigExecBlendPointFrames") {
            // An unwired input is the computation's null pointer, and a null
            // pointer passes the OTHER input through BY VALUE -- rests
            // included, because element extraction reads them and dropping
            // them would move every posed joint without moving a frame.
            const RigExecPointFrameArray *a =
                s.inA >= 0 ? &B.aggregates[size_t(s.inA)] : nullptr;
            const RigExecPointFrameArray *b =
                s.inB >= 0 ? &B.aggregates[size_t(s.inB)] : nullptr;
            if (!a) {
                if (b) aggregate = *b;
            } else if (!b) {
                aggregate = *a;
            } else if (s.blendRotationRejected) {
                // Both bound, so the token is reached: the computation warns
                // and returns nothing. The aggregate is already clear.
            } else {
                const size_t n = a->GetSize();
                // Clamp to [0, 1]: a blend weight outside the unit interval
                // extrapolates past both inputs.
                const double w =
                    std::min(std::max(double(rd(s.blendWeight)), 0.0), 1.0);
                if (n == b->GetSize() && a->rests.size() == n) {
                    aggregate.frames.reserve(n);
                    aggregate.rests.reserve(n);
                    for (size_t k = 0; k < n; ++k) {
                        aggregate.frames.push_back(RigExecBlendFrames(
                            a->frames[k], b->frames[k], a->rests[k], w,
                            RigExecRotationBlend::ShortestArc, s.scaleMode));
                        aggregate.rests.push_back(a->rests[k]);
                    }
                }
            }
        } else if (s.type == "RigExecTwistDistribution") {
            aggregate = RigExecSolveTwistDistribution(
                B.fin[size_t(s.rootRead)], B.fin[size_t(s.endRead)],
                s.twistStartRest, s.twistEndRest, s.twistWeights,
                rd(s.twistTurns));
        } else if (s.type == "RigExecRibbon") {
            // Both curves go in as the dynamic path supplies them: the live
            // points the prologue read (or the folded constant, for a curve
            // that cannot vary with time), and the bind-time points captured
            // at Build. Either one empty is the sampler's own guard, and an
            // empty aggregate is what it answers with.
            aggregate = RigExecSampleRibbonFrames(
                s.ribbonPointsVarying ? s.ribbonPoints
                                      : s.ribbonConstantPoints,
                s.ribbonRestPoints, rd(s.ribbonSampleCount));
        } else if (s.type == "RigExecSplineIk") {
            RigExecSplineIkParams params = s.splineParams;
            if (s.splineParamsVary || live(s.preserveVolume) ||
                live(s.midFollowWeight) || live(s.roll) || live(s.twist) ||
                live(s.minLengthRatio)) {
                params.preserveVolume = rd(s.preserveVolume);
                params.midFollowWeight = rd(s.midFollowWeight);
                params.roll = GfDegreesToRadians(rd(s.roll));
                params.twist = GfDegreesToRadians(rd(s.twist));
                params.minLengthRatio = rd(s.minLengthRatio);
            }
            RigExecSplineIkControls controls;
            controls.root = B.fin[size_t(s.rootRead)];
            controls.mid = B.fin[size_t(s.midRead)];
            controls.end = B.fin[size_t(s.endRead)];
            RigExecSplineIkResult solved;
            RigExecSolveSplineIk(s.splineRest, controls, params, &solved);
            if (solved.joints.size() == s.splineCount) {
                aggregate.frames.reserve(s.splineCount);
                for (const auto &joint : solved.joints) {
                    aggregate.frames.push_back(joint.frame);
                }
                aggregate.rests = s.splineJointRests;
            }
        }
        for (size_t k = 0; k < s.outputs.size(); ++k) {
            const auto &[slot, element] = s.outputs[k];
            if (element < 0 || size_t(element) >= aggregate.GetSize()) {
                s.outPresent[k] = 0;
                s.fallbackJoints.push_back(B.paths[size_t(slot)]);
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
        // Every exit below records the target's frame, because the dynamic
        // walk does: a phase names a POINT in the walk, and a constraint that
        // passed through still leaves its target standing at that point.
        // FinishCommit is the recorder, so that a split commit records after
        // its write-back rather than before it.
        const auto finish = [&] {
            if (commit.split) {
                return;  // CommitApply finishes, and records
            }
            if (!commit.abandoned) {
                ComputeCommitDeltas(B, &commit);
                StageCommitPairs(B, &commit, 0, commit.propagate.size());
            }
            FinishCommit(&B, step, &commit);
        };
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
        // The envelope, in the dynamic path's two exclusive arms. A
        // constraint copies the ORACLE -- the dynamic constraint path does
        // not go through exec at all, it calls _ResolveWeights and takes its
        // error string -- so this calls the same function with the same
        // arguments and the answer is identical by construction.
        //
        // The finite-[0, 1] check belongs to the OTHER arm and must not be
        // applied to a resolved envelope: the dynamic path does not check
        // there, so checking would emit a diagnostic it never emits.
        double weight = 1.0;
        if (!c.weightObject.IsEmpty()) {
            c.weightScratch.clear();
            c.weightError.clear();
            if (!B.resolveWeights(c.weightObject, 1, time, &c.weightScratch,
                                  &c.weightError, nullptr) ||
                c.weightScratch.size() != 1) {
                step->diagnostics.push_back(
                    c.path.GetString() + ": " + c.weightError +
                    "; constraint passed through");
                finish();
                return;
            }
            weight = c.weightScratch[0];
        } else {
            weight = rd(c.defaultWeight);
            if (!std::isfinite(weight) || weight < 0.0 || weight > 1.0) {
                step->diagnostics.push_back(
                    c.path.GetString() +
                    " has inputs:defaultWeight outside finite [0, 1]; "
                    "constraint passed through");
                finish();
                return;
            }
        }
        if (weight <= 0.0) {
            // A zero envelope is an exact dormant pass-through, decided
            // before any source is resolved so a malformed disconnected input
            // cannot make a disabled constraint fail.
            finish();
            return;
        }
        bool sourcesReady = true;
        for (size_t k = 0; k < c.sources.size(); ++k) {
            const RigExecPointFrame &frame =
                B.fin[size_t(commit.sourceReads[k])];
            if (!frame.IsValid()) {
                step->diagnostics.push_back(
                    c.path.GetString() + " could not resolve source " +
                    B.paths[size_t(c.sources[k])].GetString());
                sourcesReady = false;
                break;
            }
            commit.sources[k].frame = frame;
            commit.sources[k].normalizedWeight = c.sourceWeights[k].constant;
            commit.sources[k].translationOffset = c.translationOffsets[k];
            commit.sources[k].rotationOffsetDegrees = c.rotationOffsets[k];
        }
        if (!sourcesReady) {
            step->diagnostics.push_back(
                c.path.GetString() +
                " has unusable constraint inputs; constraint passed through");
            finish();
            return;
        }
        const std::vector<RigExecConstraintSource> &sources = commit.sources;
        const RigExecPointFrame input = B.fin[size_t(commit.slotReads[0])];
        RigExecPointFrame candidate = input;
        bool candidateReady = true;
        RigExecConstraintAxisMask affect;
        affect.x = rd(c.affectX);
        affect.y = rd(c.affectY);
        affect.z = rd(c.affectZ);
        if (c.type == "RigExecPositionConstraint") {
            RigExecPositionConstraintParams params;
            params.offset = rd(c.offset);
            params.affect = affect;
            params.weight = weight;
            candidate = RigExecApplyPositionConstraint(input, sources, params);
        } else if (c.type == "RigExecRotationConstraint") {
            RigExecRotationConstraintParams params;
            params.offsetDegrees = rd(c.offset);
            params.affect = affect;
            params.rotationOrder = c.order;
            params.weight = weight;
            candidate = RigExecApplyRotationConstraint(input, sources, params);
        } else if (c.type == "RigExecScaleConstraint") {
            RigExecScaleConstraintParams params;
            params.offset = rd(c.offset);
            params.affect = affect;
            params.weight = weight;
            candidate = RigExecApplyScaleConstraint(input, sources, params);
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
            params.weight = weight;
            candidate = RigExecApplyParentConstraint(input, sources, params);
        } else {
            // Aim: the same weighted source set reduced to the target point
            // FBX's AimAtObjects contract specifies.
            GfVec3d target(0);
            double total = 0;
            for (const RigExecConstraintSource &source : sources) {
                if (!std::isfinite(source.normalizedWeight) ||
                    source.normalizedWeight < 0) {
                    step->diagnostics.push_back(
                        c.path.GetString() +
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
                params.weight = weight;
                params.preserveInputUp = c.preserveInputUp;
                const GfVec3d authoredWorldUp = rd(c.worldUpVector);
                if (c.worldUpType == "sceneUp") {
                    params.worldUpDirection = c.sceneUp;
                } else if (c.worldUpType == "vector") {
                    params.worldUpDirection = authoredWorldUp;
                } else if (c.worldUpType == "objectUp") {
                    // FBX ObjectUp with no reference object uses the world
                    // origin as the object point.
                    params.worldUpDirection =
                        c.worldUpObjectNamed
                            ? GfVec3d(B.fin[size_t(commit.worldUpRead)]
                                          .Origin() - input.Origin())
                            : GfVec3d(-input.Origin());
                } else if (c.worldUpType == "objectRotationUp") {
                    if (!c.worldUpObjectNamed) {
                        params.worldUpDirection = authoredWorldUp;
                    } else {
                        GfMatrix4d up(1.0);
                        if (!RigExecPointsToMatrix(
                                RigExecIdentityLandmarks(),
                                B.fin[size_t(commit.worldUpRead)].points,
                                &up)) {
                            step->diagnostics.push_back(
                                c.path.GetString() +
                                " has a degenerate world-up object");
                            candidateReady = false;
                        } else {
                            params.worldUpDirection =
                                up.ExtractRotation().TransformDir(
                                    authoredWorldUp);
                        }
                    }
                }
                if (candidateReady) {
                    candidate =
                        RigExecApplyAimConstraint(input, target, params);
                }
            }
        }
        if (candidateReady) {
            // The candidate sweep a constraint commit opens with: an unusable
            // revision is diagnosed and the whole commit passes through.
            if (!RigExecBakedUsable(candidate)) {
                step->diagnostics.push_back(
                    c.path.GetString() +
                    " produced an invalid or degenerate frame for " +
                    B.paths[size_t(c.target)].GetString() +
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
            if (RigExecBakedUsable(B.restFrames[slot]) &&
                RigExecBakedUsable(frame)) {
                RigExecPointsToMatrix(B.restPts[slot], frame.points, &matrix);
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

    case RigExecBakedStepKind::SnapshotFinals: {
        // The dynamic walk fills this for every provider it holds a usable
        // rest and frame for; the program does the same, but only when
        // something can look it up, because filling it otherwise would
        // compute a matrix per slot per frame that no step reads.
        for (size_t i = 0; i < B.paths.size(); ++i) {
            const RigExecPointFrame &frame = B.fin[size_t(B.finLast[i])];
            if (!RigExecBakedUsable(B.restFrames[i]) ||
                !RigExecBakedUsable(frame)) {
                continue;
            }
            GfMatrix4d matrix(1.0);
            if (RigExecPointsToMatrix(B.restPts[i], frame.points,
                                      &matrix)) {
                step->snapshots.RecordFinal(B.paths[i], VtValue(matrix));
            }
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
    // The walk's diagnostics, in step order: commit lines, constraint lines,
    // world-up lines. They were pushed into the pose as the walk produced
    // them; they are replayed here instead, so that no step body ever touches
    // the generation and the order is program order rather than completion
    // order.
    for (const RigExecBakedStep &step : B.steps) {
        if (RigExecBakedIsGeometryStep(step.kind)) {
            continue;
        }
        for (const std::string &diagnostic : step.diagnostics) {
            pose->diagnostics.push_back(diagnostic);
        }
    }

    // An incomplete solver is an authoring gap, not a silent one. Merged into
    // one ordered set here rather than accumulated during the walk, because
    // the set's ORDER is path order and a step must not hold a shared one.
    std::set<SdfPath> fallbackJoints;
    for (const RigExecBakedProgramImpl::Solver &solver : B.solvers) {
        fallbackJoints.insert(solver.fallbackJoints.begin(),
                              solver.fallbackJoints.end());
    }
    for (const SdfPath &jointPath : fallbackJoints) {
        const auto binding = (*B.jointSolverBinding).find(jointPath);
        const std::string solver =
            binding != (*B.jointSolverBinding).end()
                ? binding->second.first.GetString()
                : std::string("<unknown>");
        const int element = binding != (*B.jointSolverBinding).end()
                                ? binding->second.second
                                : -1;
        pose->diagnostics.push_back(
            "solver " + solver + " published no element " +
            std::to_string(element) + " for joint " + jointPath.GetString() +
            "; joint fell back to its rest chain");
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
        //
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
                if (!RigExecBakedUsable(B.restFrames[size_t(slot)]) ||
                    !RigExecBakedUsable(finalFrame)) {
                    return false;  // the dynamic fallback needs exec
                }
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
    if (*B.guideTaps && *B.solverGuidesEnabled) {
        for (const int index : B.solverPublishOrder) {
            const auto &[solverPath, si] = B.solverArrays[size_t(index)];
            RigExecBakedEmplace(&pose->solverFrames, B.solverArraysAscending,
                                solverPath, B.aggregates[size_t(si)].frames);
        }
    }

    // Two published domains have no baked counterpart YET, because
    // bakeability still rules out everything that fills them -- so leaving
    // them empty is what agrees with the dynamic path rather than a gap in
    // the publication: providerXforms/providerBaseXforms come only from
    // _xformDerivedProviders ("constraint target is a plain Xformable").
    // solverOverridesConverged stays true for the same kind of reason: it is
    // cleared only by an incomplete exec snapshot, and there is no exec
    // here.
    //
    // The weight domains used to be on that list and no longer are: a
    // volume weight object bakes, so weightFrames is published from the
    // placement map the walk left (bakedProgram.cpp's epilogue), and a
    // mover's weight object bakes, so weightFields is drained per revision
    // beside the geometry it deformed (RigExecBakedPublishGeometry).
    //
    // RigExecComparePoses compares all of them regardless, so this block is
    // a checklist rather than a licence: as each refusal above goes away,
    // the domain it gated has to start being FILLED somewhere, and the
    // parity mode says so on the first generation that publishes one on the
    // dynamic side and nothing on this one.

    // Property-domain results, in the same map as the point chains: a
    // consumer tells them apart by the type the VtValue holds.
    //
    // propertyResults is itself an ordered map and movedProperties is empty
    // until here, so this walk is always ascending -- no flag to consult.
    for (const auto &[target, value] : B.propertyResults) {
        pose->movedProperties.emplace_hint(pose->movedProperties.end(),
                                           target, value);
    }
    return true;
}

}  // namespace rigExec
