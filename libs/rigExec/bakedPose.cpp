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
            for (size_t k = 0; k < joints.size(); ++k) {
                restRefs.emplace_back(
                    slotOf(joints[k]),
                    elements.size() == joints.size() ? elements[k] : int(k));
            }
        }
        for (const auto &[joint, element] : jointOutputs) {
            const int slot = slotOf(joint);
            if (slot < 0) {
                refuse("solver output is not a pose provider", joint);
                continue;
            }
            s.outputs.emplace_back(slot, element);
        }

        if (s.type == "RigExecFkChain") {
            s.parentRelative =
                readToken(prim, "rigExec:controlSpace", "") ==
                "parentRelative";
            for (const SdfPath &t : targets(prim, "rigExec:controls")) {
                const int slot = slotOf(t);
                if (slot < 0) refuse("FkChain control is not a provider", t);
                s.controls.push_back(slot);
                s.controlRests.push_back(
                    slot >= 0 ? B.restPts[slot] : RigExecIdentityLandmarks());
            }
        } else if (s.type == "RigExecTwoBoneIk") {
            const auto r = targets(prim, "rigExec:rootControl");
            const auto e = targets(prim, "rigExec:effectorControl");
            const auto p = targets(prim, "rigExec:poleControl");
            s.root = r.empty() ? -1 : slotOf(r[0]);
            s.end = e.empty() ? -1 : slotOf(e[0]);
            s.pole = p.empty() ? -1 : slotOf(p[0]);
            if (s.root < 0 || s.end < 0 || s.pole < 0) {
                refuse("TwoBoneIk control is not a provider", solverPath);
            }
            for (const auto &[slot, element] : restRefs) {
                if (slot >= 0 && element >= 0 && element < 3) {
                    s.ikRests[size_t(element)] = B.restPts[slot];
                }
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
            s.inA = solverSlot(targets(prim, "rigExec:inputA"));
            s.inB = solverSlot(targets(prim, "rigExec:inputB"));
            if (s.inA < 0 || s.inB < 0) {
                refuse("BlendPointFrames input is not an earlier solver",
                       solverPath);
            }
            s.blendWeight = bind(prim, "inputs:weight", 0.0f);
            s.scaleMode =
                readToken(prim, "rigExec:scaleBlend", "") == "linear"
                    ? RigExecScaleBlend::Linear
                    : RigExecScaleBlend::Log;
            if (readToken(prim, "rigExec:rotationBlend", "shortestArc") !=
                "shortestArc") {
                refuse("BlendPointFrames rotationBlend is not shortestArc",
                       solverPath);
            }
        } else if (s.type == "RigExecSplineIk") {
            const auto r = targets(prim, "rigExec:rootControl");
            const auto m = targets(prim, "rigExec:midControl");
            const auto e = targets(prim, "rigExec:endControl");
            s.root = r.empty() ? -1 : slotOf(r[0]);
            s.mid = m.empty() ? -1 : slotOf(m[0]);
            s.end = e.empty() ? -1 : slotOf(e[0]);
            if (s.root < 0 || s.mid < 0 || s.end < 0) {
                refuse("SplineIk control is not a provider", solverPath);
            }
            const size_t count = restRefs.size();
            s.splineCount = count;
            std::vector<RigExecPointFrame> restJoints(count);
            s.splineJointRests.resize(count);
            for (const auto &[slot, element] : restRefs) {
                if (slot < 0 || element < 0 || size_t(element) >= count) {
                    refuse("SplineIk joint element is out of range",
                           solverPath);
                    continue;
                }
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
                refuse("SplineIk volumeWeights cardinality", solverPath);
            }
            const TfToken restTok = readToken(prim, "rigExec:restLength", "");
            if (!restTok.IsEmpty() && restTok != "curve" &&
                restTok != "chain") {
                refuse("SplineIk restLength is unsupported", solverPath);
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
                refuse("SplineIk rootTangent is unsupported", solverPath);
            }
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
                if (candidateSet.count(j) || !covered.insert(j).second) {
                    continue;
                }
                int closest = B.parent[j];
                while (closest >= 0 && !candidateSet.count(closest)) {
                    closest = B.parent[closest];
                }
                if (closest < 0) continue;
                // An independently solved joint is an absolute posed
                // override: propagation cannot pass through it.
                bool blocked = false;
                for (int p = j; p >= 0 && p != closest; p = B.parent[p]) {
                    if (ownedBySolver[p]) { blocked = true; break; }
                }
                if (blocked) continue;
                out->emplace_back(j, closest);
            }
        }
    };

    // ---- the walk ------------------------------------------------------------
    for (const RigExecBakedWalkEntry &entry : walk) {
        RigExecBakedProgramImpl::Step st;
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
        B.steps.push_back(std::move(st));
    }
    B.aggregates.resize(B.solvers.size());
}

// ---------------------------------------------------------------------------
// One frame, pose half.
// ---------------------------------------------------------------------------

bool
RigExecBakedRunPose(RigExecBakedProgramImpl *program, UsdTimeCode time,
                    RigExecRigPose *pose)
{
    RigExecBakedProgramImpl &B = *program;
    const int N = int(B.paths.size());
    const RigExecResolvedInputs &R = *B.resolvedInputs;
    // Every per-frame read goes through these two: `rd` reads one input the
    // way this generation must (through the resolved inputs while an
    // override stands on it, through the pinned query otherwise), and `live`
    // answers whether a set of baked parameters has to be re-read at all.
    const auto rd = [&](const auto &input) {
        return RigExecBakedRead(input, R, time, &B.overridden);
    };
    const auto live = [&](const auto &input) {
        return input.varying ||
               (input.overrideIndex >= 0 &&
                B.overridden[size_t(input.overrideIndex)]);
    };

    // ---- the bound inputs --------------------------------------------------
    {
        RIGEXEC_PROFILE_SCOPE_CAT(*B.profiler, "BakedInputs", "baked");
        for (const auto &binding : B.avarBindings) {
            B.avars[binding.slot] = rd(binding.input);
        }
        // A drag lands on avars the bake captured as constants -- that is
        // what dragging a control on a still rig IS -- so the varying list
        // above is not the whole table while one stands. The constant slots
        // are walked while a drag stands and once more after it is released,
        // because the released slot holds the dragged value until something
        // writes the constant back over it.
        if (B.anyOverridden || B.avarsDisturbed) {
            for (const auto &binding : B.avarConstantBindings) {
                B.avars[binding.slot] =
                    B.overridden[size_t(binding.input.overrideIndex)]
                        ? rd(binding.input)
                        : binding.input.constant;
            }
            B.avarsDisturbed = B.anyOverridden;
        }
    }

    // ---- provider frames ---------------------------------------------------
    {
        RIGEXEC_PROFILE_SCOPE_CAT(*B.profiler, "BakedCompose", "baked");
        for (int i = 0; i < N; ++i) {
            const double *a = &B.avars[size_t(i) * 11];
            const double units = a[10];
            const GfMatrix4d avars = RigExecBakedComposeAvars(
                a[0] * units, a[1] * units, a[2] * units, a[3], a[4], a[5],
                a[6], a[7], a[8], a[9], B.rotOrder[i]);
            const GfMatrix4d parentPosed =
                B.parent[i] >= 0 ? B.posedM[B.parent[i]] : GfMatrix4d(1.0);
            B.base[i] = RigExecFrameFromMatrix(
                avars * B.selfD[i] * B.parentDinv[i] * parentPosed);
            B.fin[i] = B.base[i];
            // _SpaceFromFrame: an unusable frame selects the NaN sentinel, so
            // the failure survives into every descendant instead of being
            // scrubbed into a plausible identity.
            GfMatrix4d space(1.0);
            if (!B.base[i].IsValid() || B.base[i].IsDegenerate() ||
                !RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                       B.base[i].points, &space)) {
                space = GfMatrix4d(1.0);
                space[3][0] = std::numeric_limits<double>::quiet_NaN();
            }
            B.posedM[i] = space;
        }
    }

    // The write bundle of one step, with the descendant propagation the bake
    // already decided. Mirrors commitConstraintFrames in rigEvaluator.cpp,
    // including which failures are diagnosed and which are silent.
    std::map<int, RigExecPointFrame> candidates;
    std::vector<std::pair<int, RigExecPointFrame>> propagated;
    enum class _Commit { Applied, PassedThrough, Bail };
    auto commit = [&](const SdfPath &moverPath,
                      const std::vector<std::pair<int, int>> &propagate,
                      bool solverOutput) {
        if (!solverOutput) {
            for (const auto &[slot, frame] : candidates) {
                if (!RigExecBakedUsable(frame)) {
                    pose->diagnostics.push_back(
                        moverPath.GetString() +
                        " produced an invalid or degenerate frame for " +
                        B.paths[slot].GetString() +
                        "; constraint passed through");
                    return _Commit::PassedThrough;
                }
            }
        }
        propagated.clear();
        for (const auto &[j, closest] : propagate) {
            const RigExecPointFrame &current = B.fin[j];
            const auto candidate = candidates.find(closest);
            if (candidate == candidates.end()) {
                // A solver published no element for this ancestor, so the
                // baked propagation pairs no longer describe the walk.
                return _Commit::Bail;
            }
            const RigExecPointFrame &before = B.fin[closest];
            if (solverOutput &&
                (!RigExecBakedUsable(current) || !RigExecBakedUsable(before) ||
                 !RigExecBakedUsable(candidate->second))) {
                continue;
            }
            if (!RigExecBakedUsable(current)) {
                pose->diagnostics.push_back(
                    moverPath.GetString() +
                    " could not propagate its pose revision through " +
                    B.paths[j].GetString() + "; constraint passed through");
                return _Commit::PassedThrough;
            }
            GfMatrix4d delta(1.0);
            if (!RigExecPointsToMatrix(before.points,
                                       candidate->second.points, &delta)) {
                pose->diagnostics.push_back(
                    moverPath.GetString() +
                    " produced a singular hierarchy delta; constraint passed "
                    "through");
                return _Commit::PassedThrough;
            }
            const RigExecPointFrame frame =
                RigExecMatrixToPoints(current.points, delta);
            if (!RigExecBakedUsable(frame)) {
                pose->diagnostics.push_back(
                    moverPath.GetString() +
                    " produced an invalid descendant frame for " +
                    B.paths[j].GetString() + "; constraint passed through");
                return _Commit::PassedThrough;
            }
            propagated.emplace_back(j, frame);
        }
        for (const auto &[slot, frame] : candidates) {
            B.fin[slot] = frame;
            if (solverOutput) B.base[slot] = frame;
        }
        for (const auto &[slot, frame] : propagated) {
            B.fin[slot] = frame;
            if (solverOutput) B.base[slot] = frame;
        }
        return _Commit::Applied;
    };

    // ---- the interleaved solver/constraint walk ----------------------------
    std::set<SdfPath> fallbackJoints;
    std::set<size_t> visitedSolverLevels;
    for (const RigExecBakedProgramImpl::Step &step : B.steps) {
        if (step.solverBatch) {
            RIGEXEC_PROFILE_SCOPE_CAT(
                *B.profiler, "SolverBatch L" + std::to_string(step.level),
                "pose");
            candidates.clear();
            for (int si : step.batchSolvers) {
                RigExecBakedProgramImpl::Solver &s = B.solvers[si];
                RigExecPointFrameArray &aggregate = B.aggregates[si];
                aggregate.frames.clear();
                aggregate.rests.clear();
                if (s.type == "RigExecFkChain") {
                    std::vector<RigExecFkChainElement> elements(
                        s.controls.size());
                    for (size_t k = 0; k < s.controls.size(); ++k) {
                        elements[k].restPoints = s.controlRests[k];
                        elements[k].posePoints =
                            s.controls[k] >= 0
                                ? B.fin[s.controls[k]].points
                                : RigExecIdentityLandmarks();
                        elements[k].parentIndex =
                            s.parentRelative ? -1 : int(k) - 1;
                    }
                    aggregate.frames = RigExecSolveFkChain(elements);
                    aggregate.rests = s.controlRests;
                } else if (s.type == "RigExecTwoBoneIk") {
                    RigExecTwoBoneIkParams params = s.ikParams;
                    if (live(s.bend) || live(s.stretch) ||
                        live(s.softness) || live(s.upperOffset) ||
                        live(s.lowerOffset)) {
                        params.preferredBendRadians = rd(s.bend);
                        params.stretch = rd(s.stretch);
                        params.softness = rd(s.softness);
                        params.upperLength =
                            s.upperLengthBase + rd(s.upperOffset);
                        params.lowerLength =
                            s.lowerLengthBase + rd(s.lowerOffset);
                    }
                    const auto frames = RigExecSolveTwoBoneIk(
                        B.fin[s.root], B.fin[s.end], B.fin[s.pole], s.ikRests,
                        params);
                    aggregate.frames.assign(frames.begin(), frames.end());
                    aggregate.rests.assign(s.ikRests.begin(), s.ikRests.end());
                } else if (s.type == "RigExecBlendPointFrames") {
                    const RigExecPointFrameArray &a = B.aggregates[s.inA];
                    const RigExecPointFrameArray &b = B.aggregates[s.inB];
                    const size_t n = a.GetSize();
                    // Clamp to [0, 1]: a blend weight outside the unit
                    // interval extrapolates past both inputs.
                    const double w = std::min(
                        std::max(double(rd(s.blendWeight)), 0.0),
                        1.0);
                    if (n == b.GetSize() && a.rests.size() == n) {
                        aggregate.frames.reserve(n);
                        aggregate.rests.reserve(n);
                        for (size_t k = 0; k < n; ++k) {
                            aggregate.frames.push_back(RigExecBlendFrames(
                                a.frames[k], b.frames[k], a.rests[k], w,
                                RigExecRotationBlend::ShortestArc,
                                s.scaleMode));
                            aggregate.rests.push_back(a.rests[k]);
                        }
                    }
                } else if (s.type == "RigExecSplineIk") {
                    RigExecSplineIkParams params = s.splineParams;
                    if (s.splineParamsVary || live(s.preserveVolume) ||
                        live(s.midFollowWeight) || live(s.roll) ||
                        live(s.twist) || live(s.minLengthRatio)) {
                        params.preserveVolume = rd(s.preserveVolume);
                        params.midFollowWeight =
                            rd(s.midFollowWeight);
                        params.roll =
                            GfDegreesToRadians(rd(s.roll));
                        params.twist =
                            GfDegreesToRadians(rd(s.twist));
                        params.minLengthRatio =
                            rd(s.minLengthRatio);
                    }
                    RigExecSplineIkControls controls;
                    controls.root = B.fin[s.root];
                    controls.mid = B.fin[s.mid];
                    controls.end = B.fin[s.end];
                    RigExecSplineIkResult solved;
                    RigExecSolveSplineIk(s.splineRest, controls, params,
                                         &solved);
                    if (solved.joints.size() == s.splineCount) {
                        aggregate.frames.reserve(s.splineCount);
                        for (const auto &joint : solved.joints) {
                            aggregate.frames.push_back(joint.frame);
                        }
                        aggregate.rests = s.splineJointRests;
                    }
                }
                for (const auto &[slot, element] : s.outputs) {
                    if (element < 0 ||
                        size_t(element) >= aggregate.GetSize()) {
                        fallbackJoints.insert(B.paths[slot]);
                        continue;
                    }
                    candidates[slot] =
                        RigExecExtractElementFrame(&aggregate, size_t(element));
                }
            }
            if (visitedSolverLevels.insert(step.level).second) {
                ++pose->solverOverrideRounds;
            }
            pose->solverEvaluations += step.batchSolvers.size();
            // A failed commit is a pass-through, exactly as in the dynamic
            // walk, which ignores the result here and carries on.
            if (!candidates.empty() &&
                commit(SdfPath(), step.propagate, true) == _Commit::Bail) {
                return false;
            }
            continue;
        }

        const RigExecBakedProgramImpl::Constraint &c =
            B.constraints[step.index];
        RIGEXEC_PROFILE_SCOPE_CAT(
            *B.profiler, c.type.GetString() + " " + c.path.GetName(), "pose");
        if (!rd(c.enabled)) {
            continue;
        }
        const double weight = rd(c.defaultWeight);
        if (!std::isfinite(weight) || weight < 0.0 || weight > 1.0) {
            pose->diagnostics.push_back(
                c.path.GetString() +
                " has inputs:defaultWeight outside finite [0, 1]; "
                "constraint passed through");
            continue;
        }
        if (weight <= 0.0) {
            // A zero envelope is an exact dormant pass-through, decided
            // before any source is resolved so a malformed disconnected
            // input cannot make a disabled constraint fail.
            continue;
        }
        std::vector<RigExecConstraintSource> sources(c.sources.size());
        bool sourcesReady = true;
        for (size_t k = 0; k < c.sources.size(); ++k) {
            const RigExecPointFrame &frame = B.fin[c.sources[k]];
            if (!frame.IsValid()) {
                pose->diagnostics.push_back(
                    c.path.GetString() + " could not resolve source " +
                    B.paths[c.sources[k]].GetString());
                sourcesReady = false;
                break;
            }
            sources[k].frame = frame;
            sources[k].normalizedWeight = c.sourceWeights[k].constant;
            sources[k].translationOffset = c.translationOffsets[k];
            sources[k].rotationOffsetDegrees = c.rotationOffsets[k];
        }
        if (!sourcesReady) {
            pose->diagnostics.push_back(
                c.path.GetString() +
                " has unusable constraint inputs; constraint passed through");
            continue;
        }
        const RigExecPointFrame input = B.fin[c.target];
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
                    pose->diagnostics.push_back(
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
                            ? GfVec3d(B.fin[c.worldUpObject].Origin() -
                                      input.Origin())
                            : GfVec3d(-input.Origin());
                } else if (c.worldUpType == "objectRotationUp") {
                    if (!c.worldUpObjectNamed) {
                        params.worldUpDirection = authoredWorldUp;
                    } else {
                        GfMatrix4d up(1.0);
                        if (!RigExecPointsToMatrix(
                                RigExecIdentityLandmarks(),
                                B.fin[c.worldUpObject].points, &up)) {
                            pose->diagnostics.push_back(
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
            candidates.clear();
            candidates[c.target] = candidate;
            if (commit(c.path, step.propagate, false) == _Commit::Bail) {
                return false;
            }
        }
    }

    // An incomplete solver is an authoring gap, not a silent one.
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

    // ---- rest->pose matrices ------------------------------------------------
    // Emptied here and filled on demand below and by the geometry half, which
    // must read exactly the matrices this half published; see
    // RigExecBakedProgramImpl::FinalMatrixOf.
    B.ResetMatrices(size_t(N));

    {
        RIGEXEC_PROFILE_SCOPE_CAT(*B.profiler, "BakedMatrices", "baked");
        for (size_t k = 0; k < B.jointPaths.size(); ++k) {
            const int slot = B.jointSlots[k];
            const RigExecPointFrame &baseFrame = B.base[slot];
            const RigExecPointFrame &finalFrame = B.fin[slot];
            pose->jointFramesBase[B.jointPaths[k]] = baseFrame;
            pose->jointFramesFinal[B.jointPaths[k]] = finalFrame;
            // The point frame is the status bearer; publishing an identity
            // matrix for a degenerate frame would let a matrix-only consumer
            // deform with a plausible-but-wrong transform.
            if (finalFrame.IsValid() && !finalFrame.IsDegenerate()) {
                if (!RigExecBakedUsable(B.restFrames[slot]) ||
                    !RigExecBakedUsable(finalFrame)) {
                    return false;  // the dynamic fallback needs exec
                }
                pose->jointMatricesFinal[B.jointPaths[k]] =
                    B.FinalMatrixOf(slot);
            } else {
                pose->diagnostics.push_back(
                    "joint " + B.jointPaths[k].GetString() +
                    " has a degenerate final frame; matrix omitted");
            }
        }
        // Most controls are animator inputs and publish their base frame; a
        // control a constraint names publishes the revised one. Both are the
        // same slot here, because the walk wrote the revision into it.
        for (size_t k = 0; k < B.controlPaths.size(); ++k) {
            pose->controlFrames[B.controlPaths[k]] = B.fin[B.controlSlots[k]];
        }
    }
    // Observational solver guides. The dynamic path re-evaluates them through
    // a second exec request whose per-solver override IS the aggregate the
    // walk produced, so the published array is that aggregate either way --
    // and it publishes nothing at all when the consumer disabled the guides
    // or the guide request never prepared, which this mirrors so a parity
    // check compares like with like.
    if (*B.guideTaps && *B.solverGuidesEnabled) {
        for (const auto &[solverPath, si] : B.solverArrays) {
            pose->solverFrames[solverPath] = B.aggregates[si].frames;
        }
    }

    // Four published domains have no baked counterpart YET, because
    // bakeability still rules out everything that fills them -- so leaving
    // them empty is what agrees with the dynamic path rather than a gap in
    // the publication: providerXforms/providerBaseXforms come only from
    // _xformDerivedProviders ("constraint target is a plain Xformable"),
    // weightFrames only from volume weight objects, weightFields only from a
    // mover's weight object. solverOverridesConverged stays true for the
    // same kind of reason: it is cleared only by an incomplete exec
    // snapshot, and there is no exec here.
    //
    // RigExecComparePoses compares all four regardless, so this block is a
    // checklist rather than a licence: as each refusal above goes away, the
    // domain it gated has to start being FILLED here, and the parity mode
    // says so on the first generation that publishes one on the dynamic side
    // and nothing on this one.

    // Property-domain results, in the same map as the point chains: a
    // consumer tells them apart by the type the VtValue holds.
    for (const auto &[target, value] : B.propertyResults) {
        pose->movedProperties[target] = value;
    }

    return true;
}

}  // namespace rigExec
