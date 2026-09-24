//
// RigExecMatrixMover: everything about the matrix mover (spec §4.1).
//
// A matrix mover applies one provider's transform to a points array,
// measured against an optional transform space: p' = q + w (T q - q).
// This TU owns its exec-side computeMoverParameters registration and
// builder, its revision binder, its compile validator, and its
// parity-oracle branch, and registers the row that points at them.
//

#include "moverRegistry.h"
#include "moverExecCommon.h"

#include "pxr/exec/exec/builtinComputations.h"
#include "pxr/exec/exec/registerSchema.h"
#include "pxr/exec/vdf/context.h"
#include "pxr/exec/vdf/readIterator.h"
#include "pxr/usd/usdGeom/pointBased.h"

#include <cmath>

using rigExec::RigExecMoverParameters;
using rigExec::RigExecMoverExecTokens;

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

RigExecMoverParameters
_BuildMatrixMoverParameters(const VdfContext &ctx)
{
    RigExecMoverParameters params;
    params.kind = TfToken("matrix");
    const bool *enabled =
        ctx.GetInputValuePtr<bool>(RigExecMoverExecTokens->enabled);
    params.enabled = enabled ? *enabled : true;
    if (!params.enabled) {
        params.valid = true;  // disabled is an ordinary pass-through
        return params;
    }

    const GfMatrix4d *transform =
        ctx.GetInputValuePtr<GfMatrix4d>(RigExecMoverExecTokens->transform);
    if (!transform ||
        !rigExec::RigExecMoverSetCommonEnvelope(ctx, &params)) {
        return params;  // MoverFailed
    }
    // The matrix must be finite and affine (spec §7.4).
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!std::isfinite((*transform)[i][j])) {
                return params;
            }
        }
    }
    if ((*transform)[0][3] != 0 || (*transform)[1][3] != 0 ||
        (*transform)[2][3] != 0 || (*transform)[3][3] != 1) {
        return params;
    }
    params.transform = *transform;
    params.valid = true;
    return params;
}

void
_BindMatrixMover(const rigExec::RigExecMoverBindContext &ctx)
{
    rigExec::RigExecRevisionBinding &binding = *ctx.binding;
    const UsdPrim &moverPrim = ctx.moverPrim;
    const std::map<SdfPath, SdfPath> &frameChainHeads = ctx.frameChainHeads;
    // "final" binds the provider's frame-chain head instead of the
    // provider itself; every other phase binds the provider (spec §12.1).
    const SdfPathVector transforms =
        rigExec::RigExecRelationshipTargets(moverPrim, "rigExec:transform");
    SdfPath provider = transforms.empty() ? SdfPath() : transforms[0];
    binding.transformPhase = rigExec::RigExecPhaseForInput(
        moverPrim, "rigExec:transform", "rigExec:transformReadPhase");
    if (binding.transformPhase.kind == rigExec::RigExecReadPhaseKind::Final) {
        const auto it = frameChainHeads.find(provider);
        if (it != frameChainHeads.end()) {
            provider = it->second;
        }
    }
    binding.transform = provider;
    const SdfPathVector spaces = rigExec::RigExecRelationshipTargets(
        moverPrim, "rigExec:transformSpace");
    SdfPath space = spaces.empty() ? SdfPath() : spaces[0];
    if (!space.IsEmpty() &&
        binding.transformPhase.kind == rigExec::RigExecReadPhaseKind::Final) {
        const auto it = frameChainHeads.find(space);
        if (it != frameChainHeads.end()) {
            space = it->second;
        }
    }
    binding.transformSpace = space;
}

bool
_ValidateMatrixMover(
    const rigExec::RigExecMoverValidateContext &ctx, std::string *error)
{
    const UsdStageRefPtr &stage = ctx.stage;
    const UsdPrim &prim = ctx.prim;
    const std::vector<SdfPath> &targets = ctx.targets;
    const std::string who = "MatrixMover " + prim.GetPath().GetString();
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
    // Exact Sdf type/role check: equal C++ element types never infer
    // compatibility (spec §7.2).
    const UsdAttribute targetAttr =
        stage->GetAttributeAtPath(targets[0]);
    if (!targetAttr ||
        targetAttr.GetTypeName() != SdfValueTypeNames->Point3fArray) {
        *error = who + ": move target is not an exact point3f[] property";
        return false;
    }

    SdfPathVector transforms;
    if (UsdRelationship rel =
            prim.GetRelationship(TfToken("rigExec:transform"))) {
        rel.GetTargets(&transforms);
    }
    if (transforms.size() != 1) {
        *error = who + ": rigExec:transform must have exactly one target";
        return false;
    }
    // preceding is legal only for a dependency specialized to one
    // consuming application ordinal (spec §4.2); the v0.1 compiler
    // supports base and acyclic final.
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
    // The transform target must be a catalogued computeMatrix provider.
    const UsdPrim transformPrim = stage->GetPrimAtPath(transforms[0]);
    static const std::set<TfToken> frameProviderTypes = {
        TfToken("RigExecControl"), TfToken("RigExecJoint")};
    SdfPathVector spaces;
    if (UsdRelationship rel =
            prim.GetRelationship(TfToken("rigExec:transformSpace"))) {
        rel.GetTargets(&spaces);
    }
    if (spaces.size() > 1) {
        *error = who + ": rigExec:transformSpace takes at most one target";
        return false;
    }
    if (!spaces.empty()) {
        const UsdPrim spacePrim = stage->GetPrimAtPath(spaces[0]);
        if (!spacePrim || !frameProviderTypes.count(spacePrim.GetTypeName())) {
            *error = who + ": rigExec:transformSpace target is not a "
                           "catalogued matrix provider";
            return false;
        }
    }
    // The applied-API arm is gone with RigExecPointTransformAPI: that was the
    // pre-alignment landmark transform model, superseded by RigExecXformable
    // (matrix rest/posed spaces plus avars) and applied by nothing.
    const bool isProvider =
        transformPrim && frameProviderTypes.count(transformPrim.GetTypeName());
    if (!isProvider) {
        *error = who + ": rigExec:transform target is not a catalogued "
                       "matrix provider";
        return false;
    }
    return true;
}

rigExec::RigExecOracleResult
_OracleMatrixMover(const rigExec::RigExecMoverOracleContext &ctx)
{
    using rigExec::RigExecOracleResult;
    const UsdPrim &prim = ctx.prim;
    const SdfPath &moverPath = ctx.moverPath;
    std::vector<std::string> *diagnostics = ctx.diagnostics;
    VtVec3fArray &points = *ctx.points;
    // p' = q + w (T q - q) (spec §7.4).
    SdfPathVector transforms;
    if (UsdRelationship rel =
            prim.GetRelationship(TfToken("rigExec:transform"))) {
        rel.GetTargets(&transforms);
    }
    if (transforms.size() != 1) {
        diagnostics->push_back(
            "MoverFailed " + moverPath.GetString() +
            ": transform must have exactly one target");
        return RigExecOracleResult::PassThrough;
    }
    TfToken phase("base");
    if (const UsdAttribute a = prim.GetAttribute(
            TfToken("rigExec:transformReadPhase"))) {
        a.Get(&phase);
    }
    const auto &matrices =
        phase == "final" ? ctx.finalProviderMatrices
                         : ctx.baseProviderMatrices;
    const auto matrixIt = matrices.find(transforms[0]);
    if (matrixIt == matrices.end()) {
        diagnostics->push_back(
            "MoverFailed " + moverPath.GetString() +
            ": no " + phase.GetString() + " matrix provider at " +
            transforms[0].GetString());
        return RigExecOracleResult::PassThrough;
    }
    GfMatrix4d m = matrixIt->second;
    SdfPathVector spaces;
    if (UsdRelationship rel = prim.GetRelationship(
            TfToken("rigExec:transformSpace"))) {
        rel.GetTargets(&spaces);
    }
    if (!spaces.empty()) {
        const auto spaceIt = matrices.find(spaces[0]);
        if (spaceIt == matrices.end()) {
            diagnostics->push_back(
                "MoverFailed " + moverPath.GetString() +
                ": no " + phase.GetString() +
                " matrix provider at " + spaces[0].GetString());
            return RigExecOracleResult::PassThrough;
        }
        m = rigExec::RigExecMeasureInSpace(m, spaceIt->second);
    }
    for (size_t i = 0; i < points.size(); ++i) {
        const GfVec3d moved = rigExec::RigExecApplyWeightedMatrix(
            GfVec3d(points[i]), m, 1.0f);
        points[i] = GfVec3f(moved);
    }
    return RigExecOracleResult::Blend;
}

rigExec::RigExecMoverHandler
_MakeHandler()
{
    rigExec::RigExecMoverHandler handler(
        "RigExecMatrixMover",
        &rigExec::RigExecFixedMoverOp<rigExec::RigExecRevisionOp::Matrix>,
        rigExec::RigExecMoverDomain::Points);
    handler.customTargetValidation = true;
    handler.frameRelationships = {
        "rigExec:transform", "rigExec:transformSpace"};
    handler.transformRelationship = "rigExec:transform";
    handler.spaceRelationship = "rigExec:transformSpace";
    handler.bind = &_BindMatrixMover;
    handler.validate = &_ValidateMatrixMover;
    handler.oracle = &_OracleMatrixMover;
    return handler;
}

}  // namespace

// EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA opens/closes the pxr namespace
// itself, so the registration block stays at global scope.
EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecMatrixMover)
{
    self.PrimComputation(RigExecMoverExecTokens->computeMoverParameters)
        .Callback<RigExecMoverParameters>(&_BuildMatrixMoverParameters)
        .Inputs(
            RIGEXEC_MOVER_COMMON_INPUTS,
            Relationship(RigExecMoverExecTokens->resolvedTransform)
                .TargetedObjects<GfMatrix4d>(
                    RigExecMoverExecTokens->computeMatrix)
                .InputName(RigExecMoverExecTokens->transform));

    self.PrimComputation(RigExecMoverExecTokens->computeMoverStatus)
        .Callback<rigExec::RigExecMoverStatus>(
            &rigExec::RigExecMoverBuildStatus)
        .Inputs(
            Computation<RigExecMoverParameters>(
                RigExecMoverExecTokens->computeMoverParameters)
                .Required(),
            Computation<SdfPath>(ExecBuiltinComputations->computePath)
                .InputName(RigExecMoverExecTokens->moverPath));
}

RIGEXEC_REGISTER_MOVER(_MakeHandler());
