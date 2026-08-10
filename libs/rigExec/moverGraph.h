//
// RigExec compiled mover graph (spec §7.2).
//
// The per-(mover, target) revision chain is a VdfNetwork built in memory from
// the relationships already authored on the user's stage. Nothing is authored
// anywhere to express it: no generated prims, no compiler, no derived stage, and
// no schema types for the revisions themselves. Compiled nodes live only on the
// graph side.
//
// This is what the write-set authoring model always meant. A mover says "I
// write this target" and its namespace position says "in this order"; the chain
// of revisions that implies is dataflow, and dataflow is what a VdfNetwork is
// for. Materializing it as USD prims required a second stage to hold them, cost
// a recomposition, and put engine machinery on the authoring surface -- while
// still not being able to express the optimizations the graph form makes
// natural (splitting one revision across face sets, cloning legs, per-element
// masks driving sparse recomputation).
//
#ifndef RIGEXEC_MOVER_GRAPH_H
#define RIGEXEC_MOVER_GRAPH_H

#include "types.h"

#include "rigExecMath/profileMover.h"

#include "pxr/base/vt/array.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/exec/vdf/maskedOutput.h"
#include "pxr/exec/vdf/network.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/timeCode.h"

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// The operation a revision performs. One node type per operation and value
/// type, mirroring the frozen application signatures (spec §4.1): no runtime
/// operation dispatch inside a node.
enum class RigExecRevisionOp {
    Matrix,
    BlendShape,
    VolumeCorrect,
    Smooth,
    Lattice,
    SurfaceProject,
    Ribbon,
    EmitGuidePoints,
    Curvenet,
    RecomputeNormals,
    RecomputeExtent,
};

/// Where one revision reads its side inputs from.
///
/// These are the bindings the compiler used to author onto the mover prim as
/// rigExec:resolved* relationships so a registered computation could name them.
/// Every one is a build-time path choice -- which provider supplies the matrix,
/// which prim carries the topology, which attribute holds the cage -- so in the
/// graph they are just the paths an edge will be built from, and nothing needs
/// to be authored anywhere to express them.
struct RigExecRevisionBinding {
    SdfPath moverPath;        ///< the authored mover
    SdfPath target;           ///< canonical exact write target
    SdfPath transform;        ///< computeMatrix provider (matrix)
    SdfPath weightObject;     ///< computeWeightPacket provider
    SdfPath base;             ///< authored-base points (blend/volume/lattice)
    SdfPath topologyCounts;   ///< faceVertexCounts (smooth/surface)
    SdfPath topologyIndices;  ///< faceVertexIndices (smooth/surface)
    SdfPath cagePoints;       ///< lattice cage points
    SdfPath surfacePoints;    ///< driver surface points
    SdfPath bindCoords;       ///< ribbon bind coordinates
    SdfPath driverFrames;     ///< aggregate frame provider
    SdfPath widths;           ///< authored widths (extent maintenance)
    SdfPath curvenet;         ///< RigExecCurvenet prim (Profile Mover)
    SdfPath curvenetPoints;   ///< that curvenet's points property
    std::vector<SdfPath> blendInputs;  ///< sorted blend channels

    bool operator==(const RigExecRevisionBinding &o) const;
};

/// Profile Mover bindings, held across frames and keyed by (mover, target).
///
/// Cutting a mesh and factorizing its Laplacian is the expensive half of the
/// technique and depends only on the layout -- the curvenet's rest points,
/// its spline indices, and the target's base geometry. Rebuilding it per
/// frame would make the mover unusable, and rebuilding it never would make an
/// edited curvenet silently stale, so the cache is keyed by a digest of
/// exactly those inputs.
///
/// A failed bind is remembered too: a rig whose curvenet cannot be bound
/// should report that once, not re-attempt the cut on every frame.
class RigExecCurvenetBindCache
{
public:
    /// Returns the binding for \p key, calling \p build when the cached
    /// digest differs. Returns null on a failed bind and fills \p error.
    std::shared_ptr<const RigExecProfileMoverBinding> Resolve(
        const SdfPath &mover, const SdfPath &target, size_t digest,
        const std::function<bool(RigExecProfileMoverBinding *, std::string *)>
            &build,
        std::string *error);

    /// Messages accumulated by binds since the last call, and clears them.
    ///
    /// Cutting a mesh is where a curvenet's real problems surface -- faces
    /// the cut lost, segments it could not route, a net floating so far off
    /// the surface that its profile means nothing. None of that is fatal, so
    /// none of it can be an error return, and a silently bad deformation is
    /// exactly the failure mode worth spending a diagnostic on.
    std::vector<std::string> TakeDiagnostics();

    void Clear() { _entries.clear(); }
    size_t GetSize() const { return _entries.size(); }

private:
    struct _Entry {
        size_t digest = 0;
        std::shared_ptr<const RigExecProfileMoverBinding> binding;
        std::string error;
    };
    std::map<std::pair<SdfPath, SdfPath>, _Entry> _entries;
    std::vector<std::string> _pending;
};

/// Resolves a mover's side-input bindings from the authored stage.
///
/// \p frameChainHeads maps a transform provider to its final frame-chain head,
/// which is what a "final" read phase selects; an empty map resolves every
/// phase to the authored provider. Pure resolution: reads the stage, authors
/// nothing.
RigExecRevisionBinding RigExecResolveRevisionBinding(
    const UsdPrim &moverPrim,
    const SdfPath &target,
    const std::map<SdfPath, SdfPath> &frameChainHeads);

/// The revision op a mover's schema type performs, or nullopt when the type is
/// not a point-chain mover.
std::optional<RigExecRevisionOp> RigExecRevisionOpForSchema(
    const TfToken &schemaType, const TfToken &curveMode);

/// Assembles a matrix mover's parameter packet.
///
/// Peer of _BuildMatrixMoverParameters in moverKernels.cpp, but built from
/// values rather than from a VdfContext: \p transform and \p weights are the
/// already-evaluated results of the providers named by the revision binding,
/// pulled through a tap set on the authored stage. Null means the provider did
/// not produce a value, which fails the application (spec §7.4).
RigExecMoverParameters RigExecAssembleMatrixParameters(
    const UsdPrim &moverPrim,
    const GfMatrix4d *transform,
    const RigExecWeightPacket *weights,
    UsdTimeCode time = UsdTimeCode::Default());

/// Derives a mover's status from its packet (spec §6.6): disabled and failed
/// movers both pass their preceding revision through, and a failure records the
/// first bad canonical address.
RigExecMoverStatus RigExecStatusForParameters(
    const RigExecMoverParameters &parameters, const SdfPath &moverPath);

/// Provider results a revision needs that only evaluation can supply.
///
/// Everything else an assembler needs is a static read off the authored stage
/// through the revision binding. These are the dynamic ones: results of
/// computations on authored prims, pulled through a tap set on that same stage.
/// A null/empty member means the provider produced nothing, which fails the
/// application rather than substituting a default (spec §6.6).
struct RigExecProviderValues {
    const GfMatrix4d *transform = nullptr;          ///< computeMatrix
    const RigExecWeightPacket *weights = nullptr;   ///< computeWeightPacket
    const RigExecPointFrameArray *driverFrames = nullptr;
    std::vector<GfVec3f> basePoints;   ///< authored base of the target
    std::vector<GfVec3f> blendDeltas;  ///< summed channel deltas
    /// Posed control points of the mover's curvenet. Supplied by the
    /// evaluator, which runs curvenet chains first so a curvenet posed by
    /// ordinary movers reaches the Profile Mover already articulated; empty
    /// falls back to reading the curvenet's authored points at the time.
    std::vector<GfVec3f> curvenetPoints;
    /// Cache for the expensive half of the Profile Mover. Null binds fresh
    /// every call, which is correct but only sane in a test.
    RigExecCurvenetBindCache *curvenetCache = nullptr;
};

/// Sums blend channels into dense per-point deltas against \p base
/// (spec §7.3): deltas derive against the authored base, never the preceding
/// revision, and each channel's weight is clamped to [0, lastActivation] then
/// interpolated between the bracketing samples.
///
/// Shared by the mover-owned computeMoverParameters kernel and by
/// RigExecRigEvaluator, which assembles the same packet from tapped channels
/// with no derived stage. One definition, so the two cannot drift.
///
/// Returns false on a structural error -- a channel with no samples, a
/// non-finite weight, a non-positive or repeated activation, or a sample whose
/// point count disagrees with \p base -- which fails the mover atomically
/// rather than applying a partial blend.
bool RigExecSumBlendChannels(
    const std::vector<RigExecBlendChannel> &channels,
    const std::vector<GfVec3f> &base,
    std::vector<GfVec3f> *deltas);

/// Assembles any revision's parameter packet without a derived stage.
///
/// Peer of the _Build*MoverParameters family in moverKernels.cpp. Static inputs
/// (strength, divisions, mode, topology, cage and bind arrays) are read from the
/// authored stage through \p binding; dynamic ones arrive in \p values.
/// \p time is the evaluation time for every static scene read the packet
/// needs (cage, surface, topology, bind coords, strength, enable). The kernels
/// read the same inputs through exec at the current time, so passing anything
/// else silently diverges on animated input.
RigExecMoverParameters RigExecAssembleParameters(
    const UsdPrim &moverPrim,
    RigExecRevisionOp op,
    const RigExecRevisionBinding &binding,
    const RigExecProviderValues &values,
    UsdTimeCode time = UsdTimeCode::Default());

/// A compiled mover graph.
///
/// Build order mirrors the composed post-order mover walk: seed a target's
/// chain with its authored base points, then append one revision per mover that
/// writes it. Each append returns the new chain head, which is the input to the
/// next revision and, at the end, the published result.
class RigExecMoverGraph
{
public:
    RigExecMoverGraph();
    ~RigExecMoverGraph();

    RigExecMoverGraph(const RigExecMoverGraph &) = delete;
    RigExecMoverGraph &operator=(const RigExecMoverGraph &) = delete;

    /// Seeds a chain with the authored base points of \p target.
    VdfMaskedOutput AddPointSource(
        const SdfPath &target, const VtVec3fArray &points);

    /// Appends one revision reading \p previous.
    ///
    /// \p parameters and \p status are the mover's own resolved packet and
    /// status. They arrive as values rather than as scene lookups because the
    /// resolution that used to be authored as rigExec:resolved* relationships
    /// is just a build-time choice of which output to read.
    VdfMaskedOutput AddRevision(
        RigExecRevisionOp op,
        const VdfMaskedOutput &previous,
        const RigExecMoverParameters &parameters,
        const RigExecMoverStatus &status);

    /// Evaluates \p output and returns its points.
    VtVec3fArray Evaluate(const VdfMaskedOutput &output) const;

    /// Number of revision nodes currently in the graph (excludes sources).
    size_t GetRevisionCount() const { return _revisionCount; }

    const VdfNetwork &GetNetwork() const { return _network; }

private:
    VdfNetwork _network;
    size_t _revisionCount = 0;
};

}  // namespace rigExec

#endif  // RIGEXEC_MOVER_GRAPH_H
