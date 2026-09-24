//
// RigExecCurvenetMover: everything about the Profile Mover (spec §4.1).
//
// A curvenet mover deforms a mesh from a drawn curvenet: a mesh cut plus
// two sparse solves. The Profile Mover has no exec-side computation and
// no scalar oracle -- a second "independent" copy of a mesh cut plus two
// sparse solves would be the same code with the same bugs, so parity
// skips chains containing one (see hasScalarOracle) and
// testRigExecCurvenet covers it instead. This TU owns its revision
// binder and registers the row that points at it. Compile validation is
// the generic points-target + single-target rules.
//

#include "moverRegistry.h"

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

void
_BindCurvenetMover(const rigExec::RigExecMoverBindContext &ctx)
{
    rigExec::RigExecRevisionBinding &binding = *ctx.binding;
    const UsdPrim &moverPrim = ctx.moverPrim;
    const SdfPath &target = ctx.target;
    const SdfPath &ownerPath = ctx.ownerPath;
    // The Profile Mover reads the target's own topology to cut it, and
    // its authored base points are the projection pose the cut is
    // computed against (§4.1).
    binding.base = target;
    binding.topologyCounts =
        ownerPath.AppendProperty(TfToken("faceVertexCounts"));
    binding.topologyIndices =
        ownerPath.AppendProperty(TfToken("faceVertexIndices"));
    const SdfPathVector nets = rigExec::RigExecRelationshipTargets(
        moverPrim, "rigExec:curvenet");
    if (!nets.empty()) {
        binding.curvenet = nets[0].GetPrimPath();
        binding.curvenetPoints =
            binding.curvenet.AppendProperty(TfToken("points"));
        const rigExec::RigExecReadPhase phase =
            rigExec::RigExecPhaseForInput(
                moverPrim, "rigExec:curvenet", nullptr);
        if (!phase.IsBase()) {
            binding.phases[binding.curvenetPoints] = phase;
        }
    }
}

rigExec::RigExecOracleResult
_OracleCurvenetMover(const rigExec::RigExecMoverOracleContext &ctx)
{
    // No scalar oracle. Every other mover here is a few lines of
    // arithmetic that can be written twice independently, which is
    // what makes the parity check worth anything; the Profile Mover
    // is a mesh cut plus two sparse solves, and a second
    // "independent" copy of that would be the same code with the
    // same bugs. Say so rather than leave the points untouched and
    // let parity mode report a difference it cannot explain.
    if (ctx.diagnostics) {
        ctx.diagnostics->push_back(
            "cpu parity: " + ctx.prim.GetPath().GetString() +
            " is a RigExecCurvenetMover, which has no scalar "
            "reference kernel; its chain is covered by "
            "testRigExecCurvenet instead");
    }
    return rigExec::RigExecOracleResult::PassThrough;
}

rigExec::RigExecMoverHandler
_MakeHandler()
{
    rigExec::RigExecMoverHandler handler(
        "RigExecCurvenetMover",
        &rigExec::RigExecFixedMoverOp<rigExec::RigExecRevisionOp::Curvenet>,
        rigExec::RigExecMoverDomain::Points);
    handler.singleTarget = true;
    handler.legacyEnvelopeAttribute = "inputs:strength";
    handler.hasScalarOracle = false;
    handler.bind = &_BindCurvenetMover;
    handler.oracle = &_OracleCurvenetMover;
    return handler;
}

}  // namespace

RIGEXEC_REGISTER_MOVER(_MakeHandler());
