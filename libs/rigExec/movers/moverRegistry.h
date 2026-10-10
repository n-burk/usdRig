// Mover registration and discovery for built-in and external libraries.
#ifndef RIGEXEC_MOVERS_MOVER_REGISTRY_H
#define RIGEXEC_MOVERS_MOVER_REGISTRY_H

#include "../moverGraphTypes.h"
#include "../types.h"
#include "../oracleInputs.h"
#include "rigExecBinary/external.h"

#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"

#include <functional>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {
struct RigExecSceneDescriptors;

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
/// exists only to skip the ribbon arm after the wire arm ran. Applied
/// returns an already weighted result and skips that final linear blend.
enum class RigExecOracleResult {
    PassThrough,
    Blend,
    Applied, ///< already consumed the independently resolved envelope
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

/// Inputs to a mover's parity-oracle branch over captured source facts: the
/// common envelope prologue has already run (enabled check, envelope
/// resolution), and the shared envelope blend runs after a Blend return.
/// `points` is the working array, in and out.
struct RigExecOracleFrameInput {
    SdfPath owner;
    UsdTimeCode time;
    uint64_t generation = 0;
    bool available = false;
    RigExecPointFrameArray value;
};

struct RigExecMoverOracleContext {
    const RigExecOracleScene &stage;
    const RigExecOraclePrim &prim;
    const SdfPath &moverPath;
    const SdfPath &target;
    UsdTimeCode time;
    const RigExecOracleScene &resolved;
    std::function<const VtValue *(const SdfPath &, const RigExecReadPhase &, const SdfPath &)> phasedPoints;
    std::function<const GfMatrix4d *(const SdfPath &, const RigExecReadPhase &, const SdfPath &)> phasedMatrix;
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
    /// The points entering this mover: the chain built so far, or null on
    /// the chain's first mover. A side input naming `target` at `preceding`
    /// reads these (RigExecReadPhasedPoints). Last, so the members before
    /// it keep their offsets.
    const VtVec3fArray *entering = nullptr;
    /// Independent scalar envelope; kernels with nonlinear weighting apply it once.
    const std::vector<float> *envelope = nullptr;
    /// Copied declared upstream solver input, never this mover's result.
    /// A present adapter returning no value is an unavailable input.
    std::function<const RigExecOracleFrameInput *(const SdfPath &, const SdfPath &,
                                                 UsdTimeCode)> boundFrames = {};
};

/// Pure external assembly inputs. Provider values contain only explicit arrays
/// and matrices; no stage or resolved-input overlay is exposed to the plugin.
struct RigExecExternalProviderValues {
    const GfMatrix4d *transform;
    const std::vector<GfMatrix4d> *influenceTransforms;
    const GfMatrix4d *carry;
    const RigExecWeightPacket *weights;
    const RigExecPointFrameArray *driverFrames;
    const std::vector<GfVec3f> &basePoints;
    const std::vector<GfVec3f> &blendDeltas;
    explicit RigExecExternalProviderValues(const RigExecProviderValues &v)
        : transform(v.transform), influenceTransforms(v.influenceTransforms),
          carry(v.carry), weights(v.weights), driverFrames(v.driverFrames),
          basePoints(v.basePoints), blendDeltas(v.blendDeltas) {}
};
struct RigExecExternalInputContext {
    const RigExecRevisionBinding &binding;
    const RigExecExternalProviderValues &values;
    const std::vector<VtValue> &inputs;
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
    /// False when the mover has no scalar oracle: parity reports that
    /// the chain was skipped instead of claiming a reference comparison.
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

    /// External points movers declare source reads at compile and assemble a
    /// payload from sampled values. The engine owns enabled/envelope handling.
    /// Declaration order is also the assembly and runtime input order.
    void (*declareExternalInputs)(
        const RigExecMoverBindContext &ctx,
        std::vector<RigExecRevisionLeafKey> *inputs) = nullptr;
    /// Owner-thread detached compilation over composed, owned source facts.
    /// Produces the same immutable binding/declared inputs as native capture.
    /// Plugin-owned scene data schemas consumed by immutable detached capture.
    /// They are data records, not separately scheduled rig operations.
    std::vector<std::string> sceneDataSchemas;
    bool (*compileScene)(const RigExecSceneDescriptors &,const SdfPath &mover,
        const SdfPath &target,RigExecRevisionBinding *,std::string *error) = nullptr;
    bool (*assembleExternal)(
        const RigExecExternalInputContext &ctx, VtValue *data) = nullptr;
    /// Pure, thread-safe computation over the payload and preceding points.
    /// Produce the full-strength candidate without changing point count.
    /// No stage access, mutable shared state, or envelope application here.
    /// Return false for invalid inputs; the engine preserves the preceding
    /// revision. The library and payload types must remain loaded.
    bool (*applyExternal)(
        const VtValue &data, std::vector<GfVec3f> *points) = nullptr;

    /// .rigexec export, optional: a rig holding a mover without it does not
    /// export. Splits a payload assembleExternal produced into the bytes
    /// runtimeKernel reads back (see rigExecBinary/external.h). \p epoch
    /// must come out identical on every baked frame -- the export fails
    /// otherwise -- and is written once; \p frame is written per frame.
    /// Points the binding reads at a declared phase (binding.phases) reach
    /// the kernel as playback evaluates them, so a posed playback moves
    /// them; the frame bytes may still carry what the export read, for a
    /// phase playback holds no value at.
    /// Immutable runtime schema/state encoded from compiled facts even when a
    /// declared dynamic sample is missing. No sampled payload is required.
    bool (*encodeExternalEpoch)(const RigExecRevisionBinding &binding,
        std::vector<uint8_t> *epoch) = nullptr;
    bool (*encodeExternal)(
        const VtValue &data, const RigExecRevisionBinding &binding,
        std::vector<uint8_t> *epoch, std::vector<uint8_t> *frame) = nullptr;
    /// The playback half of encodeExternal, USD-free. A host that opens a
    /// .rigexec file installs it into the runtime; a runtime without it
    /// passes this mover's points through with a warning.
    RigExecExternalKernel runtimeKernel;
};

/// External libraries identify this contract with
/// Info.RigExecMoverPlugin in their OpenUSD plugInfo.json. Plugins must also
/// use the same compiler, USD build, and RigExec SDK as the host.
/// Version 2 added encodeExternal and runtimeKernel to the handler.
/// Version 4 declares sampled external inputs, detached compileScene facts,
/// and explicit phased oracle lookups.
/// Version 3 passes the oracle context to RigExecReadPhasedPoints and adds
/// RigExecMoverOracleContext::entering.
/// Version 5 adds RigExecExternalKernel::applyWithProviders, which hands a
/// playback kernel the provider values its binding reads.
inline constexpr int RigExecMoverPluginApiVersion = 5;

/// Registers one mover, retaining an immutable copy and its schema name.
/// Duplicate schema names and incomplete external callbacks are rejected.
/// Existing handler pointers remain valid when another library registers.
/// A library must remain loaded for the lifetime of every evaluator.
bool RigExecRegisterMoverHandler(
    RigExecMoverHandler handler, std::string *error = nullptr);

/// Loads registered OpenUSD Plug libraries with Info.RigExecMoverPlugin
/// equal to RigExecMoverPluginApiVersion. PXR_PLUGINPATH_NAME and
/// PlugRegistry::RegisterPlugins determine the search paths. Repeated calls
/// also discover newly registered libraries. Returns false with diagnostics
/// for incompatible versions or load failures; unrelated plugins are ignored.
bool RigExecLoadMoverPlugins(std::vector<std::string> *diagnostics = nullptr);

/// The registered handler for \p schemaType, or null for an unregistered
/// type (constraints, solvers, weight objects -- everything that is not
/// a mover). The first miss loads discoverable external mover libraries.
/// The pointer stays valid across subsequent plugin registration.
const RigExecMoverHandler *RigExecFindMoverHandler(const TfToken &schemaType);

/// Snapshot of every registered handler, in registration order.
std::vector<RigExecMoverHandler> RigExecMoverHandlers();

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

/// A resolveOp returning the fixed \p Op for every mode.
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

/// The read phase declared for the input \p rel names: the relationship's
/// rigExecReadPhase metadata, or Base. See RigExecResolveReadPhase.
RigExecReadPhase RigExecPhaseForInput(
    const UsdPrim &moverPrim, const char *rel);

/// A property-chain value as a phased input of \p consumerType holds it
/// (RigExecPhasedConnection). Compile admits only the chain's own
/// type, or float and double either way round, which this converts.
VtValue RigExecPhasedConsumerValue(
    const VtValue &chainValue, const SdfValueTypeName &consumerType);

/// The oracle's phased read of \p pointsPath, the input \p relName names
/// on \p ctx's mover: resolves the phase and reads `ctx.entering` for
/// `preceding` on the oracle's own chain past its first mover, otherwise
/// the recorded snapshot, or the authored stage value when the phase is
/// Base or no snapshot was recorded. Deliberately independent of
/// RigExecResolveRevisionBinding (that independence is what makes parity a
/// real check) while sharing the authored intent.
void RigExecReadPhasedPoints(
    const RigExecMoverOracleContext &ctx,
    const char *relName,
    const SdfPath &pointsPath,
    VtVec3fArray *out);

/// A bare prim path names the transform domain -- except on a PointBased
/// prim, where the same prim owns both write sets and the path is
/// ambiguous. There it is in practice a typo for <prim>.points.
/// Shared because the compiler asks it to decide frame work, and the
/// matrix mover's validator asks it to decide a diagnostic.
bool RigExecIsTransformDomainAmbiguous(
    const UsdStageRefPtr &stage, const SdfPath &target);

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
