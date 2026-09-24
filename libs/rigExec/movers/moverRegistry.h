//
// RigExec mover registry (spec §4.1).
//
// One row per concrete mover schema, registered by the mover's own
// translation unit under libs/rigExec/movers/. Every dispatch site that
// used to compare schema-type strings -- the revision-op map, revision
// binding, compile validation, the parity oracle, and the evaluator's
// one-off type checks -- reads this table instead, so adding a mover is
// adding a file rather than editing every switch in the engine.
//
// A mover TU owns everything about its mover: the EXEC_REGISTER block and
// computeMoverParameters builder (where the mover has exec-side
// computations at all -- skin, the curvenet pair, and the math movers do
// not), the revision binder, the compile validator, and the parity-oracle
// branch. Only genuinely shared logic lives here: registration, lookup,
// predicates, and the small stage-reading helpers every mover TU needs.
//
// Registration runs during library load, before any evaluation, exactly
// like the EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA blocks beside it (which
// is also why rigExec must stay a SHARED library: a static archive would
// let the linker drop a TU nothing else references, and with it that
// mover's row). After load the table is immutable, so lookups need no
// lock.
//

#ifndef RIGEXEC_MOVERS_MOVER_REGISTRY_H
#define RIGEXEC_MOVERS_MOVER_REGISTRY_H

#include "../moverGraph.h"

#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"

#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// Which value domain a mover revises. Points movers revise a native
/// UsdGeomPointBased point3f[] points property through the revision
/// graph; property movers revise one exact scalar property through a
/// property chain, and the enumerant names the value type the mover is
/// statically typed for.
enum class RigExecMoverDomain {
    Points,
    PropertyFloat,   ///< float, or a control avar's double
    PropertyVec3f,   ///< float3 and every GfVec3f-backed role
    PropertyMatrix,  ///< matrix4d
};

/// What a parity-oracle branch did with its revision. PassThrough is the
/// oracle's `continue`: the revision keeps the preceding value (after
/// recording whatever diagnostic the branch pushed). Blend falls through
/// to the shared envelope blend over the branch's full-strength
/// candidate -- including the curve mover's `goto envelope`, which
/// exists only to skip the ribbon arm after the wire arm ran.
enum class RigExecOracleResult {
    PassThrough,
    Blend,
};

/// Inputs to a mover's revision-binding step. See
/// RigExecResolveRevisionBinding: the handler fills in the binding fields
/// its mover reads. The weight-object tail stays common and is applied
/// after the handler runs.
struct RigExecMoverBindContext {
    const UsdPrim &moverPrim;
    const SdfPath &target;
    const SdfPath &ownerPath;
    const std::map<SdfPath, SdfPath> &frameChainHeads;
    RigExecRevisionBinding *binding;
};

/// Inputs to a mover's parity-oracle branch. See _EvaluateChain: the
/// common envelope prologue has already run (enabled check, envelope
/// resolution), and the shared envelope blend runs after a Blend return.
/// `points` is the working array, in and out.
struct RigExecMoverOracleContext {
    const UsdStageRefPtr &stage;
    const UsdPrim &prim;
    const SdfPath &moverPath;
    const SdfPath &target;
    UsdTimeCode time;
    const RigExecResolvedInputs &resolved;
    const RigExecChainSnapshots &snapshots;
    const std::unordered_map<SdfPath, GfMatrix4d, SdfPath::Hash>
        &baseProviderMatrices;
    const std::unordered_map<SdfPath, GfMatrix4d, SdfPath::Hash>
        &finalProviderMatrices;
    std::vector<std::string> *diagnostics;
    VtVec3fArray *points;
    /// The chain target's authored base points, which blend deltas derive
    /// against (spec §7.3).
    const VtVec3fArray &basePoints;
    /// For blend samples: the recorded snapshot for a dense sample's
    /// phased points read, or null when no phased read was recorded for
    /// (inputPath, samplePath). Encapsulates the compiled-chain lookup
    /// so the oracle needs no evaluator compile state.
    std::function<const VtValue *(const SdfPath &, const SdfPath &)>
        sampleSnapshot;
};

struct RigExecMoverHandler;

/// Inputs to a mover's compile-validation step. The generic target rules
/// (domain + cardinality, from the handler's own columns) have already
/// run; the handler checks only what is specific to its mover. `targets`
/// are the record's canonicalized move targets.
struct RigExecMoverValidateContext {
    const UsdStageRefPtr &stage;
    const UsdPrim &prim;
    const std::vector<SdfPath> &targets;
    const RigExecMoverHandler *handler;
};

/// One registered mover. Constructed with the three columns every mover
/// has; the rest default to "nothing special" and the mover TU sets what
/// it needs before registering.
struct RigExecMoverHandler {
    RigExecMoverHandler(
        const char *schemaType,
        std::optional<RigExecRevisionOp> (*resolveOp)(const TfToken &),
        RigExecMoverDomain domain)
        : schemaType(schemaType)
        , resolveOp(resolveOp)
        , domain(domain)
    {}

    /// The schema type name, e.g. "RigExecMatrixMover". The lookup key.
    const char *schemaType;
    /// The revision op this mover performs. Most movers return a fixed
    /// op; the curve mover resolves its authored mode; property movers
    /// return nullopt (they revise no point chain).
    std::optional<RigExecRevisionOp> (*resolveOp)(const TfToken &curveMode);
    /// Which value domain the mover revises (see RigExecMoverDomain).
    RigExecMoverDomain domain;
    /// v0.1: exactly one canonical target. A fan-out would alias the
    /// mover-level parameters across targets.
    bool singleTarget = false;
    /// The handler's validate() owns the target rules outright (matrix
    /// and skin, whose diagnostics predate the generic check); the
    /// generic target validation is skipped for it.
    bool customTargetValidation = false;
    /// Relationships naming frame providers, for the pose-cycle check.
    /// Empty for movers that read no frames.
    std::vector<const char *> frameRelationships;
    /// The relationship naming this mover's transform providers, for the
    /// unsatisfied-final-read check. Null for movers with none.
    const char *transformRelationship = nullptr;
    /// The relationship naming this mover's measure-against spaces, for
    /// the same check. Null for movers with none (only the matrix mover
    /// measures against a space).
    const char *spaceRelationship = nullptr;
    /// The concrete envelope property this mover's schema used before
    /// MoverAPI inputs:defaultWeight replaced it. An authored value is a
    /// strict-migration compile error (an old layer opinion would
    /// otherwise compose as a custom attribute and be ignored
    /// silently). Null for movers with no such predecessor.
    const char *legacyEnvelopeAttribute = nullptr;
    /// Mover attributes the epoch layout is assembled from (skin only):
    /// override tracking re-reads exactly these on a notice.
    std::vector<TfToken> layoutAttributes;
    /// False when the mover has no scalar oracle (the Profile Mover):
    /// parity skips a chain containing it and says so, rather than
    /// counting the missing reference as a defect.
    bool hasScalarOracle = true;
    /// Fills the revision binding. Null for property movers.
    void (*bind)(const RigExecMoverBindContext &ctx) = nullptr;
    /// Mover-specific compile validation. Null when the generic target
    /// rules are the whole of it.
    bool (*validate)(
        const RigExecMoverValidateContext &ctx, std::string *error) = nullptr;
    /// The parity-oracle branch. Null for property movers.
    RigExecOracleResult (*oracle)(const RigExecMoverOracleContext &ctx) =
        nullptr;
};

/// Registers one mover. Called once per mover TU during library load;
/// never after evaluation has begun.
void RigExecRegisterMoverHandler(RigExecMoverHandler handler);

/// The registered handler for \p schemaType, or null for an unregistered
/// type (constraints, solvers, weight objects -- everything that is not
/// a mover). The pointer is stable: the table never moves after load.
const RigExecMoverHandler *RigExecFindMoverHandler(const TfToken &schemaType);

/// Every registered handler, in registration order.
const std::vector<RigExecMoverHandler> &RigExecMoverHandlers();

/// Registers the handler \p handlerExpr evaluates to. Used once at the
/// bottom of each mover TU.
#define _RIGEXEC_MOVER_ANCHOR(line) _rigExecMoverAnchor##line
#define _RIGEXEC_MOVER_ANCHOR_LINE(line) _RIGEXEC_MOVER_ANCHOR(line)
#define RIGEXEC_REGISTER_MOVER(handlerExpr)                               \
    static const bool _RIGEXEC_MOVER_ANCHOR_LINE(__LINE__) =              \
        (rigExec::RigExecRegisterMoverHandler(handlerExpr), true)

/// Whether \p schemaType is a registered property-domain (math) mover.
inline bool
RigExecIsPropertyMover(const TfToken &schemaType)
{
    const RigExecMoverHandler *handler =
        RigExecFindMoverHandler(schemaType);
    return handler && handler->domain != RigExecMoverDomain::Points;
}

/// Whether \p schemaType is a registered points-domain mover.
inline bool
RigExecIsPointMover(const TfToken &schemaType)
{
    const RigExecMoverHandler *handler =
        RigExecFindMoverHandler(schemaType);
    return handler && handler->domain == RigExecMoverDomain::Points;
}

/// A resolveOp returning the fixed \p Op for every mode. What ten of the
/// thirteen movers register.
template <RigExecRevisionOp Op>
std::optional<RigExecRevisionOp>
RigExecFixedMoverOp(const TfToken &)
{
    return Op;
}

/// A resolveOp for movers that revise no point chain (the math movers).
inline std::optional<RigExecRevisionOp>
RigExecNoMoverOp(const TfToken &)
{
    return std::nullopt;
}

/// The targets of \p prim's \p rel, or empty when the relationship is
/// absent. Every binder reads its inputs through this.
SdfPathVector RigExecRelationshipTargets(
    const UsdPrim &prim, const char *rel);

/// \p path as a points property: a prim path canonicalizes to its
/// `.points`, a property path is already one.
inline SdfPath
RigExecPointsOf(const SdfPath &path)
{
    return path.IsPrimPath() ? path.AppendProperty(TfToken("points")) : path;
}

/// The read phase declared for the input \p rel names: metadata on the
/// relationship first, then the legacy attribute, then Base. See
/// RigExecResolveReadPhase.
RigExecReadPhase RigExecPhaseForInput(
    const UsdPrim &moverPrim, const char *rel, const char *legacyAttr);

/// The oracle's phased read, as a free function: resolves the phase for
/// the input \p relName names and reads the recorded snapshot for it, or
/// the authored stage value when the phase is Base or no snapshot was
/// recorded. Deliberately independent of RigExecResolveRevisionBinding
/// (that independence is what makes parity a real check) while sharing
/// the authored intent.
void RigExecReadPhasedPoints(
    const UsdStageRefPtr &stage,
    const RigExecChainSnapshots &snapshots,
    UsdTimeCode time,
    const UsdPrim &prim,
    const char *relName,
    const char *legacyAttr,
    const SdfPath &pointsPath,
    const SdfPath &readerMover,
    VtVec3fArray *out);

/// The "did you mean .points?" hint appended to a target diagnostic when
/// the target names a bare PointBased prim. Empty for anything else, so
/// callers append unconditionally.
std::string RigExecPointsTargetHint(
    const UsdStageRefPtr &stage, const SdfPath &target);

/// The generic target rules for \p handler: every target's domain (a
/// native PointBased point3f[] points property, or one exact scalar
/// property of the mover's static type) and the single-target
/// cardinality. False with \p error filled on the first violation.
/// Skipped outright for customTargetValidation handlers, whose
/// validate() owns the target rules.
bool RigExecValidateMoverTargets(
    const UsdStageRefPtr &stage,
    const UsdPrim &prim,
    const RigExecMoverHandler *handler,
    const std::vector<SdfPath> &targets,
    std::string *error);

/// Resolves a UsdSkelBlendShape's offsets into a sample layout for a
/// pointCount-point target. Returns false to REFUSE the cache (a
/// connected shape, whose value can move within the epoch); the layout
/// itself is still filled in, because a refusal means "read this every
/// frame", not "this is unusable". Shared by the packet assembly and the
/// oracle, which deliberately reads through this rather than the
/// epoch cache.
bool RigExecResolveBlendSampleLayout(
    const UsdStageRefPtr &stage,
    const SdfPath &blendShapePath,
    size_t pointCount,
    RigExecBlendSampleLayout *layout);

}  // namespace rigExec

#endif  // RIGEXEC_MOVERS_MOVER_REGISTRY_H
