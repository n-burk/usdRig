// Revision value types other headers hold by value: the operation enum,
// read phases, compiled revision bindings, provider values and the
// revision leaf declarations. Stage-free; the kernels are in moverGraph.h.
#ifndef RIGEXEC_MOVER_GRAPH_TYPES_H
#define RIGEXEC_MOVER_GRAPH_TYPES_H

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/sdf/path.h"

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

// Held by pointer only (types.h, moverGraphCaches.h).
struct RigExecWeightPacket;
struct RigExecPointFrameArray;
struct RigExecSkinTopology;
class RigExecSkinTopologyCache;
class RigExecResolvedInputs;

/// The operation a revision performs. One node type per operation and value
/// type, mirroring the frozen application signatures (spec §4.1): no runtime
/// operation dispatch inside a node.
enum class RigExecRevisionOp {
    Matrix,
    Skin,
    BlendShape,
    VolumeCorrect,
    Smooth,
    Lattice,
    SurfaceProject,
    Ribbon,
    /// RigExecCurveMover "wire": points follow a NURBS driver curve's
    /// displacement at their bind parameter (RigExecApplyWire).
    Wire,
    EmitGuidePoints,
    // Values 10 and 11 are reserved by the binary wire format.
    RecomputeNormals = 12,
    RecomputeExtent,
    // Append new ops: existing values are pinned by the binary wire format.
    DeltaMush,
    Wrinkle,
    /// A registered plugin supplies parameter assembly and point deformation.
    External,
    /// RigExecSurfaceProjector: a derived matrix primvar measured from the
    /// chain's final points (rigExec:shaderPrimvar).
    SurfaceProjector,
    /// RigExecSurfaceProjector: its shader dials packed into a derived
    /// matrix primvar (rigExec:shaderDialPrimvar).
    ShaderDials,
};

/// Whether \p op publishes a derived MATRIX primvar rather than revising
/// points or a vec3f array.
inline bool
RigExecIsDerivedMatrixOp(RigExecRevisionOp op)
{
    return op == RigExecRevisionOp::SurfaceProjector ||
           op == RigExecRevisionOp::ShaderDials;
}

/// When in the walk a side input takes its value from.
///
/// A mover or solver reads things other operators write. Which REVISION of
/// those things it gets is a separate authored choice from which path it
/// reads, declared as rigExecReadPhase metadata ON the relationship that
/// names the input: the phase sits where the binding is, so any input can
/// carry one and no schema attribute is needed to introduce another.
enum class RigExecReadPhaseKind {
    Base,       ///< the authored value: what the stage resolves at this time
    Preceding,  ///< the value immediately before the reading mover
    Final,      ///< the value after every mover that writes it has run
    AtPrim,     ///< the value as of when the walk finished with a named prim
};

/// A resolved read phase. \p prim is meaningful only for AtPrim.
///
/// AtPrim is the general form the other three are shorthands for: the walk is
/// reverse-sibling post-order over the whole composed rig, so "as of
/// this prim" means the moment that prim was finished with -- for a mover,
/// immediately after it applied; for a grouping Scope, after everything
/// beneath it applied, because post-order visits a parent last. Naming a Scope
/// is therefore how an author says "after that whole rigging stage", without
/// listing its contents.
struct RigExecReadPhase {
    RigExecReadPhaseKind kind = RigExecReadPhaseKind::Base;
    SdfPath prim;

    bool IsBase() const { return kind == RigExecReadPhaseKind::Base; }
    bool operator==(const RigExecReadPhase &o) const {
        return kind == o.kind && prim == o.prim;
    }
    bool operator!=(const RigExecReadPhase &o) const { return !(*this == o); }
    /// Stable text for digests and diagnostics.
    std::string GetAsString() const;
};

/// The metadata field an input's read phase is authored in.
///
/// Not namespaced: USD metadata field names are plain identifiers, and
/// `rigExec:readPhase` does not parse in a metadata position.
inline constexpr const char *RigExecReadPhaseMetadataName = "rigExecReadPhase";

/// Parses an authored phase string.
///
/// Accepts `base`, `preceding`, `final`, or an absolute prim path. Returns
/// false for anything else -- including a relative path, which has no
/// unambiguous meaning here -- and fills \p error.
bool RigExecParseReadPhase(
    const std::string &authored, RigExecReadPhase *phase, std::string *error);

/// Where one revision reads its side inputs from.
///
/// These are the bindings the compiler used to author onto the mover prim as
/// rigExec:resolved* relationships so a registered computation could name them.
/// Every one is a build-time path choice -- which provider supplies the matrix,
/// which prim carries the topology, which attribute holds the cage -- so in the
/// graph they are just the paths an edge will be built from, and nothing needs
/// to be authored anywhere to express them.
struct RigExecBlendSampleBinding {
    SdfPath sample;
    SdfPath points;
    RigExecReadPhase phase;
    /// The UsdSkelBlendShape prim named by rigExec:blendShape, when the
    /// sample carries its shape sparsely instead of as a full points array.
    /// Empty and `points` set is the dense form; set and `points` empty is
    /// the sparse one. Never both: the compiler rejects a sample that
    /// authors both relationships rather than picking a winner.
    SdfPath blendShape;
    bool operator==(const RigExecBlendSampleBinding &o) const {
        return sample == o.sample && points == o.points &&
               phase == o.phase && blendShape == o.blendShape;
    }
};

struct RigExecMoverHandler;
struct RigExecRevisionLeafKey;

struct RigExecRevisionBinding {
    const RigExecMoverHandler *handler = nullptr;
    TfToken externalSchema;
    VtValue externalCompileData;
    /// Existence/type/connection/subtree facts used by the compile manifest.
    std::vector<SdfPath> externalStructure;
    std::vector<RigExecRevisionLeafKey> externalInputs;
    SdfPath moverPath;        ///< the authored mover
    SdfPath target;           ///< canonical exact write target
    SdfPath transform;        ///< computeMatrix provider (matrix)
    /// Optional provider the transform is measured against (matrix):
    /// T = M(transform) * inverse(M(transformSpace)).
    SdfPath transformSpace;
    /// rigExec:space -- the provider whose own rest->pose map carries the
    /// WHOLE RIG, normally a TRS master, and which is a different thing
    /// from transformSpace above: that one is what the offset is MEASURED
    /// against, this one is what the points the offset is applied to have
    /// already been carried by. Empty when none is named, and then a
    /// post-skin cluster keeps the scale-only correction it had before.
    /// See RigExecClusterInPointFrame.
    SdfPath carrySpace;
    /// Ordered computeMatrix providers (skin): rigExec:influences, which
    /// rigExec:jointIndices index. Every entry shares transformPhase.
    /// Matrix movers use [referenceTransform, referenceTransformSpace]
    /// here when an explicit neutral solve supplies the deformation bind.
    std::vector<SdfPath> influences;
    SdfPath weightObject;     ///< computeWeightPacket provider
    SdfPath base;             ///< authored-base points (blend/volume/lattice)
    SdfPath topologyCounts;   ///< faceVertexCounts (smooth/surface)
    SdfPath topologyIndices;  ///< faceVertexIndices (smooth/surface)
    SdfPath cagePoints;       ///< lattice cage points
    SdfPath surfacePoints;    ///< driver surface points
    SdfPath bindCoords;       ///< ribbon / wire bind coordinates
    SdfPath driverCurvePoints; ///< wire: driver NURBS curve points
    SdfPath driverCurveOrder;  ///< wire: that curve's order
    SdfPath driverCurveKnots;  ///< wire: that curve's knots
    /// wire: how many of `influences` are rigExec:driverTransforms; the
    /// rest are rigExec:driverTransformSpaces. Zero when the curve's own
    /// points drive the wire.
    int driverTransformCount = 0;
    /// wire: how many of `influences` after the transforms are their
    /// spaces, and how many after those are rigExec:driverBaseTransforms;
    /// the rest are rigExec:driverBaseTransformSpaces.
    int driverSpaceCount = 0;
    int driverBaseTransformCount = 0;
    SdfPath driverFrames;     ///< aggregate frame provider
    SdfPath widths;           ///< authored widths (extent maintenance)
    /// Surface projector: rigExec:shaderDialSources, in order, at most
    /// sixteen, and the static asset-to-mesh map. The historical field name
    /// is retained on the C++/binary binding; provider frames are asset-space.
    std::vector<SdfPath> shaderDials;
    GfMatrix4d meshWorldInverse{1.0};
    std::vector<SdfPath> blendInputs;  ///< sorted blend channels
    std::map<SdfPath, std::vector<RigExecBlendSampleBinding>> blendSamples;

    /// Declared read phase per side input, keyed by the exact property path
    /// the phase governs. Absent means Base, which is what an unannotated
    /// input has always meant.
    std::map<SdfPath, RigExecReadPhase> phases;

    /// The phase declared for the transform provider. Kept apart from
    /// `phases` because it is answered from the FRAME chains rather than the
    /// point chains -- a different store with a different value type.
    RigExecReadPhase transformPhase;

    std::vector<std::pair<SdfPath, RigExecReadPhase>> GetPhasedInputs() const {
        std::vector<std::pair<SdfPath, RigExecReadPhase>> result(
            phases.begin(), phases.end());
        for (const auto &[input, samples] : blendSamples)
            for (const auto &sample : samples)
                if (!sample.phase.IsBase()) result.emplace_back(sample.points, sample.phase);
        return result;
    }

    bool operator==(const RigExecRevisionBinding &o) const;
};

/// Provider results a revision needs that only evaluation can supply.
///
/// Everything else an assembler needs is a static read off the authored stage
/// through the revision binding. These are the dynamic ones: results of
/// computations on authored prims, pulled through a tap set on that same stage.
/// A null/empty member means the provider produced nothing and normally fails
/// the application rather than substituting a default (spec §6.6). The one
/// deliberate exception is weights: null means no weight object was bound,
/// so inputs:defaultWeight supplies the common envelope.
struct RigExecProviderValues {
    const GfMatrix4d *transform = nullptr;          ///< computeMatrix
    /// computeMatrix per binding.influences entry, in that order (skin).
    const std::vector<GfMatrix4d> *influenceTransforms = nullptr;
    /// computeMatrix of binding.carrySpace (rigExec:space), read at the
    /// revision's declared phase, or null when the mover named none. A
    /// POINTER because "named" is the guard, never "identity": a
    /// transform-driven wire whose points are posed carries both control
    /// polygons by it (RigExecCarryWireCurves), and a rig naming nothing
    /// must take the untouched branch.
    const GfMatrix4d *carry = nullptr;
    /// Bound computeWeightPacket, or null to use inputs:defaultWeight.
    const RigExecWeightPacket *weights = nullptr;
    const RigExecPointFrameArray *driverFrames = nullptr;
    std::vector<GfVec3f> basePoints;   ///< owning stage/reference adapter form
    const GfVec3f *borrowedBasePoints = nullptr;
    size_t borrowedBaseCount = 0;
    bool borrowsBasePoints = false;
    const GfVec3f *BasePointData() const {
        return borrowsBasePoints ? borrowedBasePoints : basePoints.data();
    }
    size_t BasePointCount() const {
        return borrowsBasePoints ? borrowedBaseCount : basePoints.size();
    }
    void CopyBasePoints(std::vector<GfVec3f> *out) const {
        const size_t n = BasePointCount();
        if (n) out->assign(BasePointData(),BasePointData()+n);
        else out->clear();
    }
    /// Summed channel deltas. Mutable so a caller that lends them
    /// (`lendBlendDeltas`) gives them up through the const assembler input.
    mutable std::vector<GfVec3f> blendDeltas;
    /// Cache for a skin mover's epoch-fixed per-point layout. Null re-reads
    /// and re-validates the arrays every call, which is what a layout that
    /// is animated, connected, or written by a property chain requires.
    RigExecSkinTopologyCache *skinTopologyCache = nullptr;
    /// A layout the CALLER already resolved, for a caller that may not take
    /// the cache's lock where it assembles. Set -- even to a shared_ptr
    /// holding null, which is a remembered refusal -- it is used and
    /// `skinTopologyCache` is not consulted at all.
    const std::shared_ptr<const RigExecSkinTopology> *skinTopology = nullptr;
    /// Values already resolved this generation, preferred by every static
    /// read the assembler makes. Null reads the stage throughout, which is
    /// what a rig with no property chains wants and what a test may pass.
    const RigExecResolvedInputs *resolved = nullptr;
    /// The admitted upstream values by path, or null. Authored-level, so
    /// an array read that bypasses `resolved` (a lattice's or a wire's rest
    /// data at Default) still answers from it before the stage.
    const std::map<SdfPath, VtValue> *upstream = nullptr;
    /// When true the BlendShape arm swaps `blendDeltas` into the packet
    /// instead of copying them (the caller gives them up).
    bool lendBlendDeltas = false;
    /// When non-null and non-empty, the Lattice arm swaps it into
    /// `restPoints` instead of copying the base; the caller guarantees it
    /// holds the base's bytes (the chain base version did not move).
    std::vector<GfVec3f> *retainedRest = nullptr;
};

/// The provider frames a surface projector reads, as ASSET frames: each
/// provider's rest frame times its base or final computeMatrix. Index 0 is
/// binding.transform (the source), 1 binding.transformSpace (the source's
/// sibling space), 2 binding.carrySpace (the rig's space). `named` says the
/// binding names the provider; `resolved` says its frames were read.
struct RigExecSurfaceProjectorFrames {
    bool named[3] = {false, false, false};
    bool resolved[3] = {false, false, false};
    GfMatrix4d base[3] = {GfMatrix4d(1.0), GfMatrix4d(1.0), GfMatrix4d(1.0)};
    GfMatrix4d final[3] = {GfMatrix4d(1.0), GfMatrix4d(1.0), GfMatrix4d(1.0)};
};

/// What a surface projector target reads besides frames and points: its
/// settings, its dials and its surface's topology. Gathered from the stage
/// by RigExecReadProjectorTarget and from samples by the frozen replay, so
/// both hand RigExecRunProjectorTarget the same values.
struct RigExecProjectorReads {
    GfVec3d rayOrigin{0.0, 0.0, 0.0};
    GfVec3d rayDirection{0.0, 0.0, 1.0};
    GfVec3d rayUp{0.0, 1.0, 0.0};
    GfMatrix4d shaderOffset{1.0};
    bool reproject = false;
    std::vector<double> dials;
    std::vector<int> faceVertexCounts;
    std::vector<int> faceVertexIndices;
};

/// What an external mover's assembleExternal answered: the plugin's opaque
/// payload, compared by its own operator==.
struct RigExecExternalPayload {
    const RigExecMoverHandler *handler = nullptr;
    /// The mover's schema type, which names its handler.
    TfToken schema;
    /// A handler with an assembleExternal was found and it returned true.
    bool valid = false;
    VtValue data;

    bool operator==(const RigExecExternalPayload &o) const {
        return schema == o.schema && valid == o.valid && data == o.data;
    }
};

/// What a revision leaf holds. Arrays are VtArrays, shared, not copied.
enum class RigExecRevisionLeafType : uint8_t {
    Bool,
    Int,
    Float,
    Token,
    IntArray,
    FloatArray,
    Vec2fArray,
    Vec3fArray,
    Vec3i,
    Vec3d,
    Matrix4d,
    DoubleArray,
    /// A shader dial as a double: a float attribute read as a float and
    /// widened, any other read as a double (RigExecReadProjectorTarget).
    Dial,
    Double,
    Vec3f,
};

/// When a revision leaf is read: at the evaluated time, or at Default (an
/// authored rest value).
enum class RigExecRevisionLeafTime : uint8_t { AtTime, AtDefault };

/// How a revision leaf is read; each is one read site of the stage
/// assemblers, restated exactly (RigExecSampleRevisionLeaf).
enum class RigExecRevisionLeafFlavour : uint8_t {
    /// UsdAttribute::Get over the fallback (_Token, _RecordedToken).
    Raw,
    /// RigExecResolvedInputs::GetAttribute, then Raw (_Read, _Enabled).
    Resolved,
    /// RigExecResolvedInputs::GetAttribute alone, over the fallback (the
    /// blend gather, RevisionStatic's defaultWeight, the weight gathers).
    ResolvedOnly,
    /// RigExecResolvedInputs::Get at the exact path, then Raw (_Array).
    OverlayThenRaw,
    /// Whether the resolved inputs hold the exact path, as a Bool.
    Present,
};

/// The reads RigExecAssembleFromLeaves takes, one per read site of
/// RigExecAssembleParameters for the operations it covers.
enum class RigExecRevisionLeafRole : uint8_t {
    Enabled,
    DefaultWeight,
    SkinningMethod,
    ElementSize,
    JointIndices,
    JointWeights,
    WeightBlend,
    PointFrame,
    DeltaSpace,
    TopologyCounts,
    TopologyIndices,
    SurfacePoints,
    BindCoords,
    Widths,
    /// deltaMush and wrinkle: inputs:restPoints at Default; wrinkle's
    /// inputs:topology and inputs:pinPoints, both at Default.
    RestPoints,
    WrinkleTopology,
    PinPoints,
    /// Lattice: the cage at Default past the overlay, the cage at the time
    /// through it, and rigExec:divisions.
    RestCage,
    LiveCage,
    Divisions,
    /// Wire: the driver curve at Default and at the time, its order and
    /// knots, the driver weight arrays, rigExec:driverDeltaFrame (the wire's
    /// rigExec:pointFrame is PointFrame) and inputs:dropoffDistance, which
    /// answers empty where no attribute stands.
    CurveRest,
    CurveLive,
    CurveOrder,
    CurveKnots,
    DriverWeights,
    DriverBaseWeights,
    DeltaFrame,
    Dropoff,
    /// A surface projector's settings.
    RayOrigin,
    RayDirection,
    RayUp,
    ShaderOffset,
    ProjectionMode,
    /// deltaMush's extended settings: inputs:smoothing and
    /// inputs:frameTransport at Default, inputs:smoothWeights, inputs:edges
    /// at Default, inputs:onlySmooth and inputs:computationToTarget.
    MushSmoothing,
    MushFrameTransport,
    SmoothWeights,
    MushEdges,
    OnlySmooth,
    ComputationToTarget,
    /// Lattice: rigExec:evaluation, and the regular grid's interpolations,
    /// origin, spacing and strength.
    LatticeEvaluation,
    InterpolationU,
    InterpolationV,
    InterpolationW,
    GridOrigin,
    GridSpacing,
    GridStrength,
    /// Surface snap: rigExec:snapMode, rigExec:offset, rigExec:triangles.
    SnapMode,
    SnapOffset,
    SnapTriangles,
    /// Lattice and surface snap: rigExec:mask, rigExec:pointSpace, the
    /// source frame (rigExec:cageMatrix, rigExec:surfaceMatrix) and
    /// rigExec:targetMatrix.
    Mask,
    PointSpace,
    SourceMatrix,
    TargetMatrix,
    Count,
};

/// One read a revision's assembly makes: the path, the value type, the time
/// policy and the read site's flavour, plus what the site answers when the
/// read finds nothing. A Raw key with an empty fallback answers empty where
/// no attribute stands (a site that reads only an attribute that exists).
struct RigExecRevisionLeafKey {
    SdfPath path;
    RigExecRevisionLeafType type = RigExecRevisionLeafType::Float;
    RigExecRevisionLeafTime time = RigExecRevisionLeafTime::AtTime;
    RigExecRevisionLeafFlavour flavour = RigExecRevisionLeafFlavour::Raw;
    VtValue fallback;
    bool operator==(const RigExecRevisionLeafKey &o) const {
        return path == o.path && type == o.type && time == o.time &&
            flavour == o.flavour && fallback == o.fallback;
    }
};

/// The reads of one revision (or weight object), in declaration order, and
/// which of them answers each role.
struct RigExecRevisionLeafDecl {
    std::vector<RigExecRevisionLeafKey> keys;
    /// Key index per RigExecRevisionLeafRole, or -1 when the operation does
    /// not read it (an empty binding path reads an empty array).
    std::array<int, size_t(RigExecRevisionLeafRole::Count)> roles;
    /// The first of a deltaMush's or wrinkle's scalar inputs, declared in
    /// the stage assembler's order, or -1.
    int scalarBegin = -1;
    /// First declared external input, in handler declaration order.
    int externalBegin = -1;
    /// The first of a ShaderDials target's dials, one key per
    /// binding.shaderDials entry in its order, or -1.
    int dialBegin = -1;
    /// RigExecAssembleFromLeaves covers the operation.
    bool assembles = false;

    RigExecRevisionLeafDecl() { roles.fill(-1); }
    int Add(RigExecRevisionLeafKey key) {
        keys.push_back(std::move(key));
        return int(keys.size()) - 1;
    }
    int Role(RigExecRevisionLeafRole role) const {
        return roles[size_t(role)];
    }
};

/// Sampled revision leaves as RigExecAssembleFromLeaves reads them.
struct RigExecRevisionLeafView {
    const RigExecRevisionLeafDecl *decl = nullptr;
    /// One value per key of \p decl.
    const std::vector<VtValue> *values = nullptr;
    /// The revision's phase overlay (the generation's resolved inputs plus
    /// the points its phased reads resolved to this run), or null. A points
    /// read takes the overlay's array at its path before the leaf, as the
    /// stage assembler reads through that overlay. The overlay adds only
    /// point arrays, so no other read can see it.
    const RigExecResolvedInputs *phased = nullptr;
    /// When set, every scalar role the operation reads that \p decl does not
    /// declare is appended by attribute name (a test's report); the read
    /// answers the site's fallback.
    std::vector<std::string> *missing = nullptr;
    /// Legacy payload transport retained for callers during API4 migration.
    /// API4 assembly reads declared input leaves directly.
    const RigExecExternalPayload *external = nullptr;
};

}  // namespace rigExec

#endif  // RIGEXEC_MOVER_GRAPH_TYPES_H
