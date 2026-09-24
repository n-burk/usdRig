//
// RigExec math movers: the three property-domain movers (spec §4.1).
//
// A math mover revises one exact scalar property -- a float (or a
// control avar's double), a float3-family vector, or a matrix4d -- the
// way another mover revises a points array. The three share everything
// but the value type they are statically typed for, so they share one
// TU: one operation validator and three registry rows. They have no
// exec-side computation, no revision binding, and no parity-oracle
// branch; compile validation is the generic property-target rules plus
// the shared operation check below.
//

#include "moverRegistry.h"

#include "rigExecMath/propertyMath.h"

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

bool
_ValidateMathMoverOp(
    const rigExec::RigExecMoverValidateContext &ctx, std::string *error)
{
    const UsdPrim &prim = ctx.prim;
    const rigExec::RigExecMoverDomain domain = ctx.handler->domain;
    const std::string who = std::string(ctx.handler->schemaType) + " " +
                            prim.GetPath().GetString();
    // allowedTokens is advisory in USD, and an unparsed
    // operation would otherwise fall through to a silent
    // pass-through every frame.
    TfToken operation;
    if (const UsdAttribute a = prim.GetAttribute(
            TfToken("rigExec:operation"))) {
        a.Get(&operation);
    }
    rigExec::RigExecPropertyOp op;
    if (!rigExec::RigExecParsePropertyOp(operation, &op)) {
        *error = who + ": unknown rigExec:operation '" +
                 operation.GetString() + "'";
        return false;
    }
    if (op == rigExec::RigExecPropertyOp::Curve) {
        VtArray<GfVec2f> keys;
        const UsdAttribute keysAttr =
            prim.GetAttribute(TfToken("inputs:keys"));
        if (domain != rigExec::RigExecMoverDomain::PropertyFloat) {
            *error = who + ": rigExec:operation 'curve' is defined only "
                           "for RigExecFloatMathMover";
            return false;
        }
        VtArray<GfVec2f> tangents;
        if (const UsdAttribute t = prim.GetAttribute(
                TfToken("inputs:tangents"))) {
            t.Get(&tangents);
        }
        if (!keysAttr || !keysAttr.Get(&keys) ||
            keys.empty() ||
            !rigExec::RigExecValidateLinearKeys(
                keys.cdata(), keys.size()) ||
            (!tangents.empty() &&
             tangents.size() != keys.size())) {
            *error = who + ": rigExec:operation 'curve' needs "
                           "inputs:keys with finite keys strictly "
                           "increasing in input, and inputs:tangents "
                           "empty or one per key";
            return false;
        }
    }
    if (domain == rigExec::RigExecMoverDomain::PropertyMatrix &&
        op != rigExec::RigExecPropertyOp::Multiply &&
        op != rigExec::RigExecPropertyOp::Blend) {
        *error = who + ": rigExec:operation '" + operation.GetString() +
                 "' has no matrix meaning (multiply or blend)";
        return false;
    }
    return true;
}

rigExec::RigExecMoverHandler
_MakeFloatHandler()
{
    rigExec::RigExecMoverHandler handler(
        "RigExecFloatMathMover", &rigExec::RigExecNoMoverOp,
        rigExec::RigExecMoverDomain::PropertyFloat);
    handler.singleTarget = true;
    handler.legacyEnvelopeAttribute = "inputs:weight";
    handler.validate = &_ValidateMathMoverOp;
    return handler;
}

rigExec::RigExecMoverHandler
_MakeVec3fHandler()
{
    rigExec::RigExecMoverHandler handler(
        "RigExecVec3fMathMover", &rigExec::RigExecNoMoverOp,
        rigExec::RigExecMoverDomain::PropertyVec3f);
    handler.singleTarget = true;
    handler.legacyEnvelopeAttribute = "inputs:weight";
    handler.validate = &_ValidateMathMoverOp;
    return handler;
}

rigExec::RigExecMoverHandler
_MakeMatrixHandler()
{
    rigExec::RigExecMoverHandler handler(
        "RigExecMatrixMathMover", &rigExec::RigExecNoMoverOp,
        rigExec::RigExecMoverDomain::PropertyMatrix);
    handler.singleTarget = true;
    handler.legacyEnvelopeAttribute = "inputs:weight";
    handler.validate = &_ValidateMathMoverOp;
    return handler;
}

}  // namespace

RIGEXEC_REGISTER_MOVER(_MakeFloatHandler());
RIGEXEC_REGISTER_MOVER(_MakeVec3fHandler());
RIGEXEC_REGISTER_MOVER(_MakeMatrixHandler());
