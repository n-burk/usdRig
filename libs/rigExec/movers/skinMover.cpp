//
// RigExecSkinMover: everything about the skin mover (spec §4.1).
//
// A skin mover deforms points by a weighted blend of influence-provider
// matrices, classic-linear or dual-quaternion. The skin mover has no
// exec-side computation: its packet is assembled directly by
// RigExecAssembleSkinParameters (see moverGraph.cpp), which the revision
// application calls. This TU owns its revision binder, its compile
// validator, and its parity-oracle branch, and registers the row that
// points at them.
//

#include "moverRegistry.h"

#include "rigExecMath/dualQuat.h"

#include "pxr/base/gf/dualQuatd.h"
#include "pxr/usd/usdGeom/pointBased.h"

#include <cmath>

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

void
_BindSkinMover(const rigExec::RigExecMoverBindContext &ctx)
{
    rigExec::RigExecRevisionBinding &binding = *ctx.binding;
    const UsdPrim &moverPrim = ctx.moverPrim;
    const std::map<SdfPath, SdfPath> &frameChainHeads = ctx.frameChainHeads;
    // Every influence shares one declared phase, on rigExec:influences
    // or the legacy attribute, and "final" binds each provider's
    // frame-chain head exactly as the matrix mover does for its one.
    binding.influences = rigExec::RigExecRelationshipTargets(
        moverPrim, "rigExec:influences");
    binding.transformPhase = rigExec::RigExecPhaseForInput(
        moverPrim, "rigExec:influences", "rigExec:transformReadPhase");
    if (binding.transformPhase.kind == rigExec::RigExecReadPhaseKind::Final) {
        for (SdfPath &provider : binding.influences) {
            const auto it = frameChainHeads.find(provider);
            if (it != frameChainHeads.end()) {
                provider = it->second;
            }
        }
    }
}

bool
_ValidateSkinMover(
    const rigExec::RigExecMoverValidateContext &ctx, std::string *error)
{
    const UsdStageRefPtr &stage = ctx.stage;
    const UsdPrim &prim = ctx.prim;
    const std::vector<SdfPath> &targets = ctx.targets;
    const std::string who = "SkinMover " + prim.GetPath().GetString();
    // The target rules are the matrix mover's: one native points property,
    // because the jointIndices/jointWeights layout is written against one
    // point count and a fan-out would alias it across targets.
    if (targets.size() != 1 ||
        !targets[0].IsPropertyPath() ||
        targets[0].GetNameToken() != "points") {
        *error = who + ": moves must resolve to exactly one native "
                       "PointBased points property" +
                 (targets.size() == 1
                      ? rigExec::RigExecPointsTargetHint(stage, targets[0])
                      : std::string());
        return false;
    }
    const UsdPrim owner =
        stage->GetPrimAtPath(targets[0].GetPrimPath());
    if (!owner || !owner.IsA<UsdGeomPointBased>()) {
        *error = who + ": move target owner is not a stock PointBased prim";
        return false;
    }
    const UsdAttribute targetAttr =
        stage->GetAttributeAtPath(targets[0]);
    if (!targetAttr ||
        targetAttr.GetTypeName() != SdfValueTypeNames->Point3fArray) {
        *error = who + ": move target is not an exact point3f[] property";
        return false;
    }

    SdfPathVector influences;
    if (UsdRelationship rel =
            prim.GetRelationship(TfToken("rigExec:influences"))) {
        rel.GetTargets(&influences);
    }
    if (influences.empty()) {
        *error = who + ": rigExec:influences must name at least one matrix "
                       "provider";
        return false;
    }
    static const std::set<TfToken> frameProviderTypes = {
        TfToken("RigExecControl"), TfToken("RigExecJoint")};
    for (const SdfPath &influence : influences) {
        const UsdPrim provider = stage->GetPrimAtPath(influence);
        if (!provider || !frameProviderTypes.count(provider.GetTypeName())) {
            *error = who + ": rigExec:influences target " +
                     influence.GetString() +
                     " is not a catalogued matrix provider";
            return false;
        }
    }
    TfToken phase("base");
    if (UsdAttribute a =
            prim.GetAttribute(TfToken("rigExec:transformReadPhase"))) {
        a.Get(&phase);
    }
    if (phase != "base" && phase != "final") {
        *error = who + ": unsupported transformReadPhase '" +
                 phase.GetString() + "' (v0.1 supports base and final)";
        return false;
    }

    // Strict about the method: a declared token the kernel cannot honour is
    // a compile error, never a silent fallback to different maths.
    TfToken method("classicLinear");
    if (UsdAttribute a = prim.GetAttribute(TfToken("rigExec:skinningMethod"))) {
        a.Get(&method);
    }
    if (method != "classicLinear" && method != "dualQuaternion") {
        *error = who + ": unknown rigExec:skinningMethod '" +
                 method.GetString() + "'";
        return false;
    }

    // The layout is a value, re-validated by the assembler at every
    // evaluation; checking the authored default here is what turns a
    // mis-sized export into a compile diagnostic instead of a silently
    // passed-through mesh.
    int elementSize = 1;
    if (UsdAttribute a = prim.GetAttribute(TfToken("rigExec:elementSize"))) {
        a.Get(&elementSize);
    }
    if (elementSize < 1) {
        *error = who + ": rigExec:elementSize must be at least 1";
        return false;
    }
    VtIntArray indices;
    VtFloatArray weights;
    if (UsdAttribute a = prim.GetAttribute(TfToken("rigExec:jointIndices"))) {
        a.Get(&indices);
    }
    if (UsdAttribute a = prim.GetAttribute(TfToken("rigExec:jointWeights"))) {
        a.Get(&weights);
    }
    if (indices.size() != weights.size()) {
        *error = who + ": rigExec:jointIndices length " +
                 std::to_string(indices.size()) +
                 " must equal rigExec:jointWeights length " +
                 std::to_string(weights.size());
        return false;
    }
    VtVec3fArray points;
    if (targetAttr.Get(&points) && !points.empty() &&
        indices.size() != points.size() * size_t(elementSize)) {
        *error = who + ": rigExec:jointIndices length " +
                 std::to_string(indices.size()) + " must equal " +
                 std::to_string(points.size()) + " points * elementSize " +
                 std::to_string(elementSize);
        return false;
    }
    for (size_t i = 0; i < indices.size(); ++i) {
        if (indices[i] < 0 || size_t(indices[i]) >= influences.size()) {
            *error = who + ": rigExec:jointIndices[" + std::to_string(i) +
                     "] = " + std::to_string(indices[i]) +
                     " is outside the " + std::to_string(influences.size()) +
                     " influences";
            return false;
        }
        if (!std::isfinite(weights[i]) || weights[i] < 0.0f) {
            *error = who + ": rigExec:jointWeights[" + std::to_string(i) +
                     "] must be finite and non-negative";
            return false;
        }
    }
    return true;
}

rigExec::RigExecOracleResult
_OracleSkinMover(const rigExec::RigExecMoverOracleContext &ctx)
{
    using rigExec::RigExecOracleResult;
    const UsdPrim &prim = ctx.prim;
    const SdfPath &moverPath = ctx.moverPath;
    const UsdTimeCode time = ctx.time;
    const rigExec::RigExecResolvedInputs &resolved = ctx.resolved;
    std::vector<std::string> *diagnostics = ctx.diagnostics;
    VtVec3fArray &points = *ctx.points;
    // p' = (1 - sum_k w_k) p + sum_k w_k T_k p per point, in double,
    // read straight off the stage: independent of
    // RigExecAssembleSkinParameters and of the SIMD kernel on
    // purpose, so parity is a real check.
    SdfPathVector influences;
    if (UsdRelationship rel =
            prim.GetRelationship(TfToken("rigExec:influences"))) {
        rel.GetTargets(&influences);
    }
    TfToken phase("base");
    if (const UsdAttribute a = prim.GetAttribute(
            TfToken("rigExec:transformReadPhase"))) {
        a.Get(&phase);
    }
    const auto &matrices =
        phase == "final" ? ctx.finalProviderMatrices
                         : ctx.baseProviderMatrices;
    std::vector<GfMatrix4d> transforms;
    bool failed = false;
    for (const SdfPath &provider : influences) {
        const auto matrixIt = matrices.find(provider);
        if (matrixIt == matrices.end()) {
            diagnostics->push_back(
                "MoverFailed " + moverPath.GetString() +
                ": no " + phase.GetString() + " matrix provider at " +
                provider.GetString());
            failed = true;
            break;
        }
        transforms.push_back(matrixIt->second);
    }
    if (failed) {
        return RigExecOracleResult::PassThrough;
    }
    VtIntArray indices;
    VtFloatArray weights;
    int elementSize = 1;
    TfToken method("classicLinear");
    if (const UsdAttribute a = prim.GetAttribute(
            TfToken("rigExec:jointIndices"))) {
        if (!resolved.Get(a.GetPath(), &indices)) {
            a.Get(&indices, time);
        }
    }
    if (const UsdAttribute a = prim.GetAttribute(
            TfToken("rigExec:jointWeights"))) {
        if (!resolved.Get(a.GetPath(), &weights)) {
            a.Get(&weights, time);
        }
    }
    if (const UsdAttribute a = prim.GetAttribute(
            TfToken("rigExec:elementSize"))) {
        a.Get(&elementSize, time);
    }
    if (const UsdAttribute a = prim.GetAttribute(
            TfToken("rigExec:skinningMethod"))) {
        a.Get(&method, time);
    }
    if (method != "classicLinear" && method != "dualQuaternion") {
        diagnostics->push_back(
            "MoverFailed " + moverPath.GetString() +
            ": skinning method '" + method.GetString() +
            "' has no scalar reference kernel");
        return RigExecOracleResult::PassThrough;
    }
    if (elementSize < 1 || transforms.empty() ||
        indices.size() != weights.size() ||
        indices.size() != points.size() * size_t(elementSize)) {
        diagnostics->push_back(
            "MoverFailed " + moverPath.GetString() +
            ": jointIndices/jointWeights layout does not match "
            "the target's point count");
        return RigExecOracleResult::PassThrough;
    }
    VtVec3fArray next = points;
    bool degenerate = false;
    if (method == "dualQuaternion") {
        // Independent DQS reference, written from the rule rather
        // than taken from the kernel. Every influence is split into
        // a pre-rotation stretch S_j and a rigid motion by the
        // library's polar split (the one piece shared with the
        // kernel: Gf's Factor is a Gram-Schmidt split that
        // legitimately disagrees with it under shear). Per point:
        // the pivot is the largest-weight slot; every influence
        // whose rotation opposes the pivot's is negated; the
        // complement 1 - sum w enters as the identity; the stretch
        // is sum w_j S_j + (1 - sum w) I; the sum is normalised
        // once; p' = (p S) rotated and translated. Accumulation,
        // normalisation and the point transform are Pixar's
        // GfDualQuatd, whose formulas differ from dualQuat.cpp's.
        std::vector<GfDualQuatd> rigid;
        std::vector<GfMatrix3d> stretch;
        for (const GfMatrix4d &t : transforms) {
            const rigExec::RigExecScaledDualQuat sdq =
                rigExec::RigExecScaledDualQuatFromMatrix(t);
            rigid.emplace_back(sdq.rigid.real, sdq.rigid.dual);
            stretch.push_back(sdq.stretch);
        }
        for (size_t i = 0; i < points.size() && !failed; ++i) {
            int pivot = -1;
            float pivotWeight = -1.0f;
            for (int k = 0; k < elementSize; ++k) {
                const size_t slot =
                    i * size_t(elementSize) + size_t(k);
                const int j = indices[slot];
                const float w = weights[slot];
                if (j < 0 || size_t(j) >= transforms.size() ||
                    !std::isfinite(w) || w < 0.0f) {
                    failed = true;
                    break;
                }
                if (pivotWeight < w) {
                    pivotWeight = w;
                    pivot = j;
                }
            }
            if (failed) {
                break;
            }
            const GfQuatd pivotReal = rigid[pivot].GetReal();
            GfDualQuatd sum = GfDualQuatd::GetZero();
            GfMatrix3d s(0.0);
            double total = 0.0;
            for (int k = 0; k < elementSize; ++k) {
                const size_t slot =
                    i * size_t(elementSize) + size_t(k);
                const int j = indices[slot];
                const double w = weights[slot];
                if (w == 0.0) {
                    continue;
                }
                const double signedW =
                    GfDot(rigid[j].GetReal(), pivotReal) < 0.0 ? -w
                                                               : w;
                sum += rigid[j] * signedW;
                s += stretch[j] * w;
                total += w;
            }
            const double complement = 1.0 - total;
            if (complement != 0.0) {
                const GfDualQuatd identity =
                    GfDualQuatd::GetIdentity();
                const double signedW =
                    GfDot(identity.GetReal(), pivotReal) < 0.0
                        ? -complement
                        : complement;
                sum += identity * signedW;
                s += GfMatrix3d(1.0) * complement;
            }
            if (sum.Normalize().first < 1e-9) {
                degenerate = true;
                break;
            }
            next[i] = GfVec3f(sum.Transform(GfVec3d(points[i]) * s));
        }
    } else {
        for (size_t i = 0; i < points.size() && !failed; ++i) {
            const GfVec3d p(points[i]);
            GfVec3d sum(0.0);
            double total = 0.0;
            for (int k = 0; k < elementSize; ++k) {
                const size_t slot =
                    i * size_t(elementSize) + size_t(k);
                const int j = indices[slot];
                const float w = weights[slot];
                if (j < 0 || size_t(j) >= transforms.size() ||
                    !std::isfinite(w) || w < 0.0f) {
                    failed = true;
                    break;
                }
                if (w == 0.0f) {
                    continue;
                }
                sum += transforms[j].TransformAffine(p) * double(w);
                total += w;
            }
            next[i] = GfVec3f(p * (1.0 - total) + sum);
        }
    }
    if (failed) {
        diagnostics->push_back(
            "MoverFailed " + moverPath.GetString() +
            ": jointIndices out of range or jointWeights not finite "
            "and non-negative");
        return RigExecOracleResult::PassThrough;
    }
    if (degenerate) {
        diagnostics->push_back(
            "MoverFailed " + moverPath.GetString() +
            ": degenerate dual-quaternion blend (over-driven "
            "weights cancelled the rotation)");
        return RigExecOracleResult::PassThrough;
    }
    points = next;
    return RigExecOracleResult::Blend;
}

rigExec::RigExecMoverHandler
_MakeHandler()
{
    rigExec::RigExecMoverHandler handler(
        "RigExecSkinMover",
        &rigExec::RigExecFixedMoverOp<rigExec::RigExecRevisionOp::Skin>,
        rigExec::RigExecMoverDomain::Points);
    handler.customTargetValidation = true;
    handler.frameRelationships = {"rigExec:influences"};
    handler.transformRelationship = "rigExec:influences";
    // The four the layout is assembled from: the two arrays and the
    // element size RigExecResolveSkinTopology reads, and the method
    // RigExecAssembleSkinParameters reads beside them. inputs:enabled
    // and inputs:defaultWeight are deliberately NOT here -- they are
    // read per frame, never cached, so an override on one reaches the
    // next generation without anything being dropped.
    handler.layoutAttributes = {
        TfToken("rigExec:jointIndices"), TfToken("rigExec:jointWeights"),
        TfToken("rigExec:elementSize"), TfToken("rigExec:skinningMethod")};
    handler.bind = &_BindSkinMover;
    handler.validate = &_ValidateSkinMover;
    handler.oracle = &_OracleSkinMover;
    return handler;
}

}  // namespace

RIGEXEC_REGISTER_MOVER(_MakeHandler());
