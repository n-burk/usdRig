//
// The baked program's implementation state, and the helpers its build and run
// halves share.
//
// bakedProgram.cpp is the only translation unit the evaluator declares a
// friend, so everything the frame path needs out of RigExecRigEvaluator is
// captured ONCE, at Build, into the pointers below; bakedPose.cpp and
// bakedGeometry.cpp then read the program rather than the evaluator. That is
// what lets the program be split by domain without widening the evaluator's
// friendship -- and the capture is also the list of exactly what a running
// frame reads from outside itself, which Phase 2's step graph needs stated
// rather than discovered.
//
// Nothing here is a second expression of semantics: the helpers are the ones
// bakedProgram.cpp already had, moved so more than one file can call them.
//
#ifndef RIGEXEC_BAKED_PROGRAM_IMPL_H
#define RIGEXEC_BAKED_PROGRAM_IMPL_H

#include "bakedProgram.h"
#include "frameExtraction.h"
#include "moverGraph.h"
#include "profiler.h"
#include "tapSet.h"
#include "types.h"
#include "weightPackets.h"

#include "rigExecMath/avarScale.h"
#include "rigExecMath/pointFrame.h"
#include "rigExecMath/solvers.h"
#include "rigExecMath/splineIk.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/attributeQuery.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

class RigExecRigEvaluator;
struct RigExecRigPose;

// ---------------------------------------------------------------------------
// Shared helpers.
// ---------------------------------------------------------------------------

// Mirrors computations.cpp _ComposeAvars (the provider compose exec runs).
// The program must produce the same numbers as that callback, so the two are
// written the same way rather than algebraically simplified.
inline GfMatrix4d
RigExecBakedComposeAvars(double tx, double ty, double tz, double sx,
                         double sy, double sz, double rx, double ry,
                         double rz, double rspin, const TfToken &order)
{
    static const GfVec3d axes[3] = {
        GfVec3d(1, 0, 0), GfVec3d(0, 1, 0), GfVec3d(0, 0, 1)};
    const double angles[3] = {rx, ry, rz};
    std::string sequence = order.GetString();
    if (sequence.size() != 3) {
        sequence = "XYZ";
    }
    GfMatrix4d m(1.0);
    m.SetScale(GfVec3d(RigExecNormalizeAvarScale(sx),
                       RigExecNormalizeAvarScale(sy),
                       RigExecNormalizeAvarScale(sz)));
    for (const char axis : sequence) {
        const int index = axis == 'X' ? 0 : axis == 'Y' ? 1 : 2;
        if (angles[index] != 0.0) {
            m = m * GfMatrix4d(GfRotation(axes[index], angles[index]),
                               GfVec3d(0));
        }
    }
    if (rspin != 0.0) {
        m = m * GfMatrix4d(GfRotation(axes[0], rspin), GfVec3d(0));
    }
    GfMatrix4d t(1.0);
    t.SetTranslate(GfVec3d(tx, ty, tz));
    return m * t;
}

inline RigExecEulerOrder
RigExecBakedParseEulerOrder(const TfToken &t)
{
    if (t == "XZY") return RigExecEulerOrder::XZY;
    if (t == "YXZ") return RigExecEulerOrder::YXZ;
    if (t == "YZX") return RigExecEulerOrder::YZX;
    if (t == "ZXY") return RigExecEulerOrder::ZXY;
    if (t == "ZYX") return RigExecEulerOrder::ZYX;
    return RigExecEulerOrder::XYZ;
}

// Exec hands a FRAME between computations and every matrix-typed consumer
// turns it back into a matrix (_SpaceFromFrame in computations.cpp). The
// round trip is not the identity in floating point, so the program performs
// it wherever exec does -- otherwise a deep chain drifts.
inline GfMatrix4d
RigExecBakedRoundTrip(const GfMatrix4d &m)
{
    GfMatrix4d out(1.0);
    RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                          RigExecFrameFromMatrix(m).points, &out);
    return out;
}

// rigEvaluator.cpp _IsUsableConstraintFrame, which gates every commit in the
// pose walk this program reproduces.
inline bool
RigExecBakedUsable(const RigExecPointFrame &frame)
{
    if (!frame.IsValid() || frame.IsDegenerate()) {
        return false;
    }
    for (const GfVec3d &point : frame.points) {
        if (!std::isfinite(point[0]) || !std::isfinite(point[1]) ||
            !std::isfinite(point[2])) {
            return false;
        }
    }
    return true;
}

// True when \p attribute or anything it resolves through carries animation,
// which is what makes the value unbakeable. A single time sample counts: it
// reads back at a numeric time but not at Default, so capturing it would make
// the answer depend on which time code the caller passed.
inline bool
RigExecBakedAnimatedOrConnected(const UsdAttribute &attribute)
{
    if (!attribute) {
        return false;
    }
    // A single time sample counts as animated: it reads back at a numeric
    // time but not at Default, so capturing it would make the answer depend
    // on which time code the caller passed.
    if (attribute.ValueMightBeTimeVarying() ||
        attribute.GetNumTimeSamples() > 0) {
        return true;
    }
    // A connection means the value comes from somewhere this check has not
    // looked at. HasAuthoredConnections first: see _AuthoredConnections in
    // rigEvaluator.cpp for why the guard is worth its line.
    SdfPathVector connections;
    return attribute.HasAuthoredConnections() &&
           attribute.GetConnections(&connections) && !connections.empty();
}

// ---------------------------------------------------------------------------
// The input binding table.
//
// RigExecResolvedInputs::GetAttribute -- which is what both exec's
// AttributeValue accessor and every static read in the evaluator resolve
// through -- walks a single authored connection chain and takes the nearest
// readable upstream value, preferring anything this generation already
// resolved. That WALK is epoch-structural: connections cannot change without
// a resync, and a resync recompiles. Only the VALUES on it can move.
//
// So each input is classified once: if nothing on its walk is written by a
// property chain and nothing on it might vary with time, the value is an
// epoch constant and the frame path never touches USD for it. Otherwise it is
// read per frame -- through a retained UsdAttributeQuery when USD alone can
// answer, and through the generation's resolved inputs when a property chain
// is in the way, which is the same route the dynamic path takes.
// ---------------------------------------------------------------------------

template <class T>
struct RigExecBakedInput {
    T constant{};
    UsdAttributeQuery query;  ///< set when USD alone answers, per frame
    UsdAttribute resolvedAttr;  ///< set when a property chain is on the walk
    UsdAttribute head;        ///< where the walk starts, for an override
    bool varying = false;
    /// Index into RigExecBakedProgramImpl::overridden, or -1 when the
    /// input was never registered (the rest ladder, which is folded rather
    /// than read).
    int overrideIndex = -1;
};

// The attribute GetAttribute would end up reading, plus why it cannot be
// captured. Mirrors that function's loop exactly.
template <class T>
inline void
RigExecBakedClassifyInput(const UsdAttribute &attribute, UsdTimeCode time,
                          const std::set<SdfPath> &chainTargets,
                          bool *viaChain, bool *varying,
                          UsdAttribute *selected,
                          SdfPathVector *walk = nullptr)
{
    std::set<SdfPath> visiting;
    std::vector<UsdAttribute> fallback;
    UsdAttribute a = attribute;
    while (a && visiting.insert(a.GetPath()).second) {
        if (walk) {
            walk->push_back(a.GetPath());
        }
        if (chainTargets.count(a.GetPath())) {
            *viaChain = true;
        }
        // Both halves of the OR are needed and neither implies the other.
        // ValueMightBeTimeVarying is false for an attribute whose strongest
        // opinion is exactly ONE time sample of a non-composable type
        // (UsdStage::_ValueMightBeTimeVaryingFromResolveInfo), and the
        // capture below reads at Default, which never sees a time sample --
        // so a single-keyed avar would be captured as its schema fallback.
        // GetNumTimeSamples is 0 for a Ts-spline valued attribute in this
        // USD build while it varies. This is the same predicate
        // RigExecBakedAnimatedOrConnected applies further down, for the
        // same reason.
        if (a.ValueMightBeTimeVarying() || a.GetNumTimeSamples() > 0) {
            *varying = true;
        }
        fallback.push_back(a);
        SdfPathVector connections;
        a.GetConnections(&connections);
        if (connections.size() != 1) {
            break;
        }
        a = a.GetPrim().GetStage()->GetAttributeAtPath(connections[0]);
    }
    for (auto it = fallback.rbegin(); it != fallback.rend(); ++it) {
        T probe{};
        if (it->Get(&probe, time)) {
            *selected = *it;
            return;
        }
    }
}

template <class T>
inline RigExecBakedInput<T>
RigExecBakedBindInput(const UsdPrim &prim, const char *name, T fallback,
                      UsdTimeCode time,
                      const std::set<SdfPath> &chainTargets,
                      SdfPathVector *walk = nullptr)
{
    RigExecBakedInput<T> input;
    input.constant = fallback;
    if (!prim) {
        return input;
    }
    const UsdAttribute attribute = prim.GetAttribute(TfToken(name));
    if (!attribute) {
        return input;
    }
    input.head = attribute;
    bool viaChain = false, varying = false;
    UsdAttribute selected;
    RigExecBakedClassifyInput<T>(attribute, time, chainTargets, &viaChain,
                                 &varying, &selected, walk);
    if (viaChain) {
        input.varying = true;
        input.resolvedAttr = attribute;
        return input;
    }
    if (varying) {
        input.varying = true;
        if (selected) {
            input.query = UsdAttributeQuery(selected);
        }
        return input;
    }
    if (selected) {
        selected.Get(&input.constant, time);
    }
    return input;
}

template <class T>
inline T
RigExecBakedRead(const RigExecBakedInput<T> &input,
                 const RigExecResolvedInputs &resolved, UsdTimeCode time,
                 const std::vector<char> *overridden = nullptr)
{
    T value = input.constant;
    // A held drag is placed by reading THE LONG WAY. GetAttribute is the same
    // walk exec's accessor performs and _resolvedInputs already carries the
    // override, so the answer is the dynamic one rather than a second
    // approximation of it -- and the pinned query, which knows nothing about
    // the drag, is bypassed for as long as it stands.
    if (input.overrideIndex >= 0 && overridden &&
        (*overridden)[size_t(input.overrideIndex)]) {
        resolved.GetAttribute(input.head, time, &value);
        return value;
    }
    if (!input.varying) {
        return value;
    }
    if (input.resolvedAttr) {
        resolved.GetAttribute(input.resolvedAttr, time, &value);
        return value;
    }
    if (input.query.IsValid()) {
        input.query.Get(&value, time);
    }
    return value;
}

// True when \p input has to be re-read; used only to size the report.
template <class T>
inline bool
RigExecBakedIsVarying(const RigExecBakedInput<T> &input)
{
    return input.varying;
}
inline constexpr const char *const RigExecBakedAvarNames[11] = {
    "avars:tx", "avars:ty", "avars:tz", "avars:sx", "avars:sy", "avars:sz",
    "avars:rx", "avars:ry", "avars:rz", "avars:rspin",
    "avars:unitScaleFactor"};
inline constexpr double RigExecBakedAvarDefaults[11] = {
    0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1};
inline const char *
RigExecBakedOpName(RigExecRevisionOp op)
{
    switch (op) {
    case RigExecRevisionOp::Matrix: return "matrix";
    case RigExecRevisionOp::Skin: return "skin";
    case RigExecRevisionOp::BlendShape: return "blendShape";
    case RigExecRevisionOp::VolumeCorrect: return "volumeCorrect";
    case RigExecRevisionOp::Smooth: return "smooth";
    case RigExecRevisionOp::Lattice: return "lattice";
    case RigExecRevisionOp::SurfaceProject: return "surfaceProject";
    case RigExecRevisionOp::Ribbon: return "ribbon";
    case RigExecRevisionOp::EmitGuidePoints: return "emitGuidePoints";
    case RigExecRevisionOp::Curvenet: return "curvenet";
    case RigExecRevisionOp::CurvenetAdjuster: return "curvenetAdjuster";
    case RigExecRevisionOp::RecomputeNormals: return "recomputeNormals";
    case RigExecRevisionOp::RecomputeExtent: return "recomputeExtent";
    }
    return "unknown";
}

// ---------------------------------------------------------------------------
// The program.
// ---------------------------------------------------------------------------

/// What a provider slot IS, which decides both what writes it and what the
/// walk may do with it.
///
/// The dynamic path keeps one frame map covering both families and tells them
/// apart by asking whether the path is in _poseSeedFrames -- its
/// `hierarchicalProviders` set. The program answers the same question off a
/// dense table instead, because it asks it once per descendant per commit.
enum class RigExecBakedSlotKind {
    /// An exec-seeded RigExec provider: composed from avars, propagated to as
    /// a descendant, and the only kind a solver or the compose writes.
    PoseSeed,
    /// A plain Xformable a constraint targets. The dynamic walk seeds it from
    /// the stage and revises it like any other target, but deliberately keeps
    /// it out of hierarchicalProviders, so it is never propagated TO -- while
    /// it can still be the closest revised ancestor a RigExec descendant
    /// rides.
    XformDerived,
};

// ---------------------------------------------------------------------------
// The step graph.
//
// A run is a dependency graph of steps over dense slots rather than one
// straight line. PROGRAM ORDER -- the order the straight line used -- is still
// the reference: a step's index is its position in it, every edge points
// forward, and the serial executor runs the steps in index order. That is what
// makes "byte-identical to the old Run" a property any topological order of
// this graph has, because floating-point results depend only on operand values
// and each step's arithmetic is fixed.
//
// A step declares the slot RANGES it reads and writes; the edges are computed
// from those declarations alone (bakedSchedule.cpp). Declared writes are an
// UPPER BOUND: a disabled constraint, a revision that did not execute and a
// commit that passed through all write fewer slots than they declared, and no
// executor may ever "write what was declared".
// ---------------------------------------------------------------------------

/// Which dense table a slot indexes.
///
/// The specification calls this a slot KIND. That word is already spent in
/// this header on RigExecBakedSlotKind, which says what a PROVIDER slot is
/// (PoseSeed or XformDerived), and two similarly named enums in one header is
/// how the wrong one gets declared -- so the step graph's is a slot's DOMAIN,
/// the table it indexes, and the provider one keeps its name.
enum class RigExecBakedSlotDomain : uint8_t {
    Avars,               ///< avars[i*11 .. +10]; filled by the prologue
    PoseBase,            ///< base[i]
    PoseFin,             ///< fin[i]
    PosedM,              ///< posedM[i]
    FinalMatrix,         ///< finalMatrix[i]
    BaseMatrix,          ///< baseMatrix[i]
    Aggregate,           ///< aggregates[s]
    Candidates,          ///< one Solve step's own (slot, frame) scratch
    CommitTable,         ///< one commit's merged candidate table
    CommitDelta,         ///< one commit's per-candidate hierarchy delta
    CommitStaging,       ///< one propagation pair's staged frame and outcome
    PropertyResult,      ///< propertyResults[t]; filled by the prologue
    ChainBase,           ///< chains[c].lastBase; filled by the prologue
    RevisionPacket,      ///< one revision's assembled packet, status, executed
    RevisionTransforms,  ///< one revision's influence table
    RevisionOut,         ///< one revision's OWN output buffer
    RevisionDone,        ///< one revision's applied/status flags
    ChainDirty,          ///< the chain's sticky dirty bit as of one revision
    ChainPoints,         ///< the chain's published points
    DerivedOut,          ///< one derived target's output
    Snapshots,           ///< the phased-read records one step made
};
/// Derived from the last enumerator rather than written out: an array
/// indexed by domain is how the edge sweep is written, and a count that
/// drifted from the enum is an out-of-bounds write with no symptom at Build.
inline constexpr size_t RigExecBakedSlotDomainCount =
    size_t(RigExecBakedSlotDomain::Snapshots) + 1;

/// A slot id: the domain in the top 8 bits, the index in the low 24.
using RigExecBakedSlot = uint32_t;

inline RigExecBakedSlot
RigExecBakedMakeSlot(RigExecBakedSlotDomain domain, uint32_t index)
{
    return (uint32_t(domain) << 24) | (index & 0xffffffu);
}

/// A half-open run of slots of one domain.
///
/// Provider slots are in namespace DFS pre-order, so a subtree is contiguous
/// and a commit's descendants are a scan rather than a set; declaring ranges
/// rather than slots is what collapses the edge count and makes the edge
/// computation an interval sweep.
struct RigExecBakedSlotRange {
    RigExecBakedSlotDomain domain = RigExecBakedSlotDomain::Avars;
    uint32_t begin = 0;
    uint32_t end = 0;

    bool IsEmpty() const { return end <= begin; }
    bool Overlaps(const RigExecBakedSlotRange &other) const {
        return domain == other.domain && begin < other.end &&
               other.begin < end;
    }
    bool operator==(const RigExecBakedSlotRange &other) const {
        return domain == other.domain && begin == other.begin &&
               end == other.end;
    }
    bool operator<(const RigExecBakedSlotRange &other) const {
        if (domain != other.domain) return domain < other.domain;
        if (begin != other.begin) return begin < other.begin;
        return end < other.end;
    }
};

inline RigExecBakedSlotRange
RigExecBakedRange(RigExecBakedSlotDomain domain, int begin, int end)
{
    RigExecBakedSlotRange range;
    range.domain = domain;
    range.begin = uint32_t(begin);
    range.end = uint32_t(end);
    return range;
}

inline RigExecBakedSlotRange
RigExecBakedOne(RigExecBakedSlotDomain domain, int index)
{
    return RigExecBakedRange(domain, index, index + 1);
}

/// The name of \p domain, for the schedule report.
const char *RigExecBakedSlotDomainName(RigExecBakedSlotDomain domain);

/// Whether the prologue, and not a step, fills \p domain.
///
/// These are the graph's SOURCES: a read of one with no writer in the graph
/// is well formed. Every other domain's storage is written by a step, and a
/// read of it with no writer is either a bug or the loop-carried read of
/// Aggregate that BlendPointFrames makes when its input solver runs in no
/// earlier batch (the reader list is therefore seeded from program start).
///
/// Snapshots is deliberately NOT one of them although the prologue empties
/// the store: every record in it is written by a step, so a read of it must
/// name the steps it reads, and calling the domain a source would excuse the
/// one declaration the verifier exists to check.
inline bool
RigExecBakedIsSourceDomain(RigExecBakedSlotDomain domain)
{
    return domain == RigExecBakedSlotDomain::Avars ||
           domain == RigExecBakedSlotDomain::PropertyResult ||
           domain == RigExecBakedSlotDomain::ChainBase ||
           domain == RigExecBakedSlotDomain::Aggregate;
}

/// What one step of the program does.
enum class RigExecBakedStepKind {
    ComposeSubtree,   ///< one subtree of the provider forest
    Solve,            ///< one solver's aggregate and candidate frames
    SolverCommit,     ///< one solver batch's merged candidate table
    Constraint,       ///< one constraint's candidate frame
    CommitDelta,      ///< a split commit's per-candidate hierarchy deltas
    PropagateChunk,   ///< a split commit's staged descendant frames
    CommitApply,      ///< a split commit's decision and write-back
    ProviderMatrix,   ///< one provider's rest -> final or rest -> base matrix
    SnapshotFinals,   ///< every provider's final matrix, for a phased read
    InfluenceFold,    ///< one revision's influence table
    RevisionStatic,   ///< one revision's packet, status and executed decision
    RevisionChunk,    ///< one vertex range of one revision
    RevisionFuse,     ///< one revision's applied decision and chain dirty bit
    ChainStatus,      ///< one chain's status sweep and published points
    Derived,          ///< one derived target maintained from a chain
};

/// The name of \p kind, for the schedule report.
const char *RigExecBakedStepKindName(RigExecBakedStepKind kind);

/// Whether \p kind belongs to the geometry half of a frame.
///
/// The executor uses it to pick a body and the epilogue to replay the two
/// halves' diagnostics where the published generation puts them: the walk's
/// before the joint block, the geometry's interleaved with the chains.
inline bool
RigExecBakedIsGeometryStep(RigExecBakedStepKind kind)
{
    switch (kind) {
    case RigExecBakedStepKind::InfluenceFold:
    case RigExecBakedStepKind::RevisionStatic:
    case RigExecBakedStepKind::RevisionChunk:
    case RigExecBakedStepKind::RevisionFuse:
    case RigExecBakedStepKind::ChainStatus:
    case RigExecBakedStepKind::Derived:
        return true;
    default:
        return false;
    }
}

/// How many diagnostics a walk step may emit in one run.
///
/// Every diagnostic site in the walk is terminal -- it passes the constraint
/// through or gives the generation back -- so a walk step's list is short and
/// bounded, which is what lets the epilogue replay it without ever growing a
/// shared vector inside the region. The two-line case is a constraint whose
/// source did not resolve: it names the source, then says the constraint
/// passed through. Geometry steps carry their own bound (a chain's status
/// sweep has one line per failed revision), which Build records per step.
inline constexpr size_t RigExecBakedMaxStepDiagnostics = 4;

/// What one step added to the generation's work counters.
///
/// Counted per step and summed in the epilogue rather than incremented on the
/// pose, because a step body never touches the pose.
struct RigExecBakedStepCounters {
    uint32_t revisionsExecuted = 0;
    uint32_t revisionsCreated = 0;
    uint32_t schedulesBuilt = 0;
    uint32_t chainsBuilt = 0;
    uint32_t revisionsBuilt = 0;

    void Clear() { *this = RigExecBakedStepCounters(); }
};

/// One step of the program.
struct RigExecBakedStep {
    RigExecBakedStepKind kind = RigExecBakedStepKind::ComposeSubtree;
    /// What this step is about, by kind:
    ///   ComposeSubtree                              index into composeGroups
    ///   Solve                                       index into solvers
    ///   SolverCommit/Constraint/CommitDelta/
    ///     PropagateChunk/CommitApply                index into commits
    ///   ProviderMatrix                              provider slot
    ///   SnapshotFinals                              unused
    ///   InfluenceFold/RevisionStatic/
    ///     RevisionChunk/RevisionFuse                index into revisionIndex
    ///   ChainStatus                                 index into chains
    ///   Derived                                     index into derivedIndex
    int object = -1;
    /// The part of it, by kind: the vertex chunk of a RevisionChunk, the
    /// propagation-pair chunk of a PropagateChunk, and 1 for a final-phase
    /// ProviderMatrix against 0 for a base-phase one. -1 where unused.
    int part = -1;

    /// Sorted, deduplicated. Writes are an upper bound (see above).
    std::vector<RigExecBakedSlotRange> reads, writes;
    /// Program indices, sorted; every entry of preds is < this step's index.
    std::vector<int> preds, succs;

    /// Filled by the clustering pass; the serial executor ignores both.
    int cluster = -1;
    double cost = 0;

    /// This run's output, all of it per step so that nothing in a body
    /// touches shared state.
    std::vector<std::string> diagnostics;
    size_t maxDiagnostics = RigExecBakedMaxStepDiagnostics;
    RigExecBakedStepCounters counters;
    /// The phased-read records this step made, merged into the run's store
    /// by the executor in step order.
    RigExecChainSnapshots snapshots;
    /// The step gave the generation back: the baked propagation pairs no
    /// longer describe the walk. Nothing after it runs.
    bool bail = false;
    /// When the profiler is on: the step's interval, replayed into it by the
    /// epilogue in step order so the trace is deterministic.
    uint64_t startUs = 0, endUs = 0;
    /// What the step is called in the report and the trace, built once at
    /// Build so that neither costs a string per step per frame.
    std::string label;

    /// Empties this run's output. Called by the executor before the body, so
    /// that a step that is skipped keeps last run's lines for the epilogue
    /// to replay.
    void BeginRun() {
        diagnostics.clear();
        counters.Clear();
        snapshots.Clear();
        bail = false;
    }
};

/// One contiguous group of provider slots the compose pass runs as one step.
struct RigExecBakedComposeGroup {
    int begin = 0, end = 0;
    /// The PosedM slots below `begin` that slots inside the group inherit
    /// from. One entry on any rig that bakes today; more only once a plain
    /// Xformable can sit between a provider and its compose ancestor.
    std::vector<int> parentSlots;
};

/// Everything one commit -- a solver batch's or a constraint's -- needs.
///
/// The candidate table is DENSE and in slot order, because both halves of
/// the commit walk it in slot order: the usability sweep and the write-back.
/// Which position each producer writes is decided at Build, so the merge is
/// an indexed store rather than a map insertion, and "last writer wins on a
/// duplicate slot" stays what it is today.
struct RigExecBakedCommit {
    /// Empty for a solver batch, whose diagnostics carry no mover path.
    SdfPath moverPath;
    bool solverOutput = false;
    /// Sorted, unique.
    std::vector<int> slots;
    /// Filled this run, in `slots` order.
    std::vector<char> present;
    std::vector<RigExecPointFrame> frames;
    /// (descendant, closest revised ancestor), in the order the dynamic walk
    /// enumerates them.
    std::vector<std::pair<int, int>> propagate;
    /// For each pair, the position of its `closest` in `slots`, or -1 when
    /// the batch declares no candidate for it at all.
    std::vector<int> closestPos;
    /// Per candidate position: the hierarchy delta and whether it resolved.
    std::vector<GfMatrix4d> deltas;
    std::vector<char> deltaOk;
    /// Per pair: the staged frame and what happened to it.
    std::vector<RigExecPointFrame> staged;
    std::vector<uint8_t> outcome;
    /// The candidate set was not produced at all this run -- a disabled
    /// constraint, an unusable candidate, a solver batch that published
    /// nothing -- so the whole commit writes nothing.
    bool abandoned = true;
    /// Build's decision to run the commit as three steps rather than one.
    bool split = false;
    /// Where this commit's pairs start in the CommitStaging domain. The
    /// staging scratch is per commit, but the slot ids may not be: two split
    /// commits both declaring [0, 64) would make the edge sweep order their
    /// chunks against each other and hide a real conflict behind a false one.
    int stagingBase = 0;
    /// A constraint step's source scratch, sized at Build.
    std::vector<RigExecConstraintSource> sources;
};

/// What a staged propagation pair turned into. Ordered so that the first
/// non-Staged, non-Skipped outcome in pair order decides the commit, which is
/// where today's loop returns.
enum class RigExecBakedPropagateOutcome : uint8_t {
    Staged,             ///< a usable descendant frame is waiting
    Skipped,            ///< a solver commit stepped over an unusable pair
    NoCandidate,        ///< the baked pairs no longer describe the walk
    UnusableDescendant, ///< the descendant's own frame is unusable
    SingularDelta,      ///< the hierarchy delta would not resolve
    InvalidResult,      ///< the propagated frame is unusable
};

struct RigExecBakedProgramImpl {
    RigExecRigEvaluator *evaluator = nullptr;
    UsdStageRefPtr stage;

    // ---- the evaluator state the frame path reads --------------------------
    //
    // Captured once, at Build, inside the one translation unit the evaluator
    // declares a friend. Pointers rather than copies wherever the value can
    // move between frames -- a drag rewrites the override list, a cache fills
    // in, the profiler is switched on mid-session -- so the program reads what
    // the evaluator holds NOW and can never answer from a stale copy.
    RigExecResolvedInputs *resolvedInputs = nullptr;
    /// The evaluator's phased-read store, EMPTIED at the head of a run
    /// exactly as the dynamic walk empties it at the head of its own, so the
    /// two paths leave the evaluator in the same state. Nothing is ever
    /// recorded into it: in a parity generation the baked run precedes the
    /// dynamic one over the same evaluator, and a record left here would be
    /// read back by the dynamic walk as if its own pose walk had produced it.
    /// The program records into `runSnapshots` below instead.
    RigExecChainSnapshots *chainSnapshots = nullptr;
    RigExecSkinTopologyCache *skinTopologies = nullptr;
    RigExecProfiler *profiler = nullptr;
    const std::vector<RigExecValueOverride> *interactiveOverrides = nullptr;
    /// Joint -> (solver, element), which names the solver in the diagnostic a
    /// joint gets when its solver published no element for it.
    const std::map<SdfPath, std::pair<SdfPath, int>> *jointSolverBinding =
        nullptr;
    /// The observational guide request, which exists only while a consumer
    /// asked for one, and the runtime toggle beside it. Read per frame rather
    /// than folded: either can move without the epoch moving.
    const std::unique_ptr<RigExecTapSet> *guideTaps = nullptr;
    const bool *solverGuidesEnabled = nullptr;
    /// Whether the epoch has any property chain at all. A chain appearing or
    /// disappearing is a structural edit, which recompiles and rebuilds this
    /// program, so this one is a captured constant and not a pointer.
    bool hasPropertyChains = false;

    // ---- dense provider slots, namespace DFS order ------------------------
    // The slot table is the ordered UNION of the two provider families the
    // dynamic walk holds in one frame map: the RigExec providers exec seeds
    // (_poseSeedFrames) and the plain Xformables a constraint targets
    // (_xformDerivedProviders). Both are keyed by SdfPath, whose order IS
    // namespace DFS pre-order, so a provider's parent always has a lower slot
    // than it does and one forward pass composes the whole hierarchy.
    std::vector<SdfPath> paths;
    std::map<SdfPath, int> index;
    std::vector<RigExecBakedSlotKind> slotKind;
    /// Nearest COMPOSE ancestor -- the nearest PoseSeed slot above this one.
    /// The compose ladder inherits from it, because exec's NamespaceAncestor
    /// resolves only RigExec provider types and skips anything else.
    std::vector<int> parent;
    /// Nearest ancestor slot of ANY kind, which is what the dynamic walk's
    /// pure namespace climb finds when it looks for the closest revised
    /// ancestor a propagated descendant rides.
    std::vector<int> propParent;

    // ---- epoch constants resolved at bake ---------------------------------
    std::vector<GfMatrix4d> restM;                     // asset-space rest
    std::vector<std::array<GfVec3d, 4>> restPts;
    std::vector<RigExecPointFrame> restFrames;
    std::vector<GfMatrix4d> selfD;                     // default:space
    std::vector<GfMatrix4d> parentDinv;                // parent default^-1
    std::vector<TfToken> rotOrder;

    // ---- the input binding table ------------------------------------------
    std::vector<double> avarConstants;                 // providers * 11
    struct AvarBinding {
        size_t slot = 0;
        RigExecBakedInput<double> input;
    };
    std::vector<AvarBinding> avarBindings;            // the varying ones only
    /// The rest, kept only so a drag can reach one. Walked per frame while an
    /// interactive override stands and never otherwise.
    std::vector<AvarBinding> avarConstantBindings;
    size_t boundInputs = 0;
    size_t varyingInputs = 0;

    // ---- per-frame working state (dense) ----------------------------------
    std::vector<double> avars;
    std::vector<GfMatrix4d> posedM;
    std::vector<RigExecPointFrame> base, fin;

    /// This frame's property-chain results, per target. The prologue fills it
    /// and the pose half publishes it; program-owned so the two halves cannot
    /// be handed different maps.
    std::map<SdfPath, VtValue> propertyResults;

    /// This run's phased-read store: what each chain held at each point of
    /// the walk, and what each provider's matrix was after each constraint
    /// that named it. Run-local by design (see `chainSnapshots` above), and
    /// the only store the program looks a phase up in -- the evaluator's
    /// holds the previous generation's dynamic records.
    RigExecChainSnapshots runSnapshots;

    /// The program's OWN curvenet bind cache, not the evaluator's.
    ///
    /// Two reasons, and either alone would be enough. It is drained by
    /// TakeDiagnostics, so sharing one cache with the dynamic path means
    /// whichever ran first reports the bind lines and the other reports none
    /// -- which in a parity generation is a diagnostic difference the
    /// comparator is right to call a mismatch. And the cache has no locking
    /// at all, so it belongs to one walk at a time.
    ///
    /// THREAD SAFETY, for the step graph being built over this: Resolve()
    /// mutates the entry map and appends to the pending diagnostics with no
    /// synchronisation whatever. It may only ever be touched from serial
    /// code -- the run's prologue or its epilogue -- and never from a step
    /// body. Carried across a rebuild by AdoptGeometryStateFrom, because a
    /// bind survives an edit that rebuilds the program the same way a
    /// revision's cached result does.
    RigExecCurvenetBindCache curvenetBindings;
    /// True when some revision of this epoch declares a read phase, which is
    /// the only thing that can LOOK the store up. While it is false nothing
    /// can observe a record, so the pose half does not pay to fill it -- and
    /// in particular the rest -> final matrix of a provider no step reads is
    /// still never computed, which is the lazy set this program keeps.
    bool phasedReads = false;

    // ---- rest->pose matrices ------------------------------------------------
    //
    // What computeMatrix publishes, over dense slots: the whole
    // AuthoritativeSnapshot request is a re-derivation of values the walk
    // already holds. Program-owned rather than a per-frame allocation,
    // because BOTH halves of a frame read them, the geometry half must see
    // exactly the matrix the pose half published, and a 267 KB zero-fill per
    // frame is work that belongs at Build.
    //
    // One ProviderMatrix step writes each entry the program can read -- the
    // set today's lazy finalMatrixOf/baseMatrixOf computed, no larger -- so
    // there is no on-demand fill and no per-frame reset: an entry no step
    // writes simply keeps a value nothing reads.
    std::vector<GfMatrix4d> finalMatrix, baseMatrix;
    /// Whether some step or the epilogue reads this slot's final / base
    /// matrix, which is what decides that a ProviderMatrix step exists for
    /// it. Build fills both.
    std::vector<char> needFinal, needBase;

    // ---- solvers -----------------------------------------------------------
    struct Solver {
        SdfPath path;
        TfToken type;
        /// The bake found this solver's rigExec:joints / jointElements
        /// binding malformed in one of the ways its exec computation checks
        /// at runtime. Such a computation warns and returns an EMPTY
        /// aggregate, so the program publishes one too: every joint the
        /// solver names then falls back to its rest chain, with the
        /// diagnostic that carries.
        bool degenerate = false;
        // FkChain
        std::vector<int> controls;
        bool parentRelative = false;
        std::vector<std::array<GfVec3d, 4>> controlRests;
        // IK / spline controls
        int root = -1, mid = -1, end = -1, pole = -1;
        // TwoBoneIk: rests and the measured bone lengths, both epoch-constant
        std::array<std::array<GfVec3d, 4>, 3> ikRests{};
        RigExecTwoBoneIkParams ikParams;
        RigExecBakedInput<double> bend, upperOffset, lowerOffset;
        RigExecBakedInput<float> stretch, softness;
        double upperLengthBase = 0, lowerLengthBase = 0;
        // BlendPointFrames
        int inA = -1, inB = -1;
        RigExecBakedInput<float> blendWeight;
        RigExecScaleBlend scaleMode = RigExecScaleBlend::Log;
        // SplineIk: the rest description is a pure function of epoch-constant
        // rests, so exec's per-evaluation rebuild bakes out.
        RigExecSplineIkRest splineRest;
        RigExecSplineIkParams splineParams;
        std::vector<std::array<GfVec3d, 4>> splineJointRests;
        size_t splineCount = 0;
        RigExecBakedInput<double> preserveVolume, midFollowWeight, roll, twist,
            minLengthRatio;
        bool splineParamsVary = false;
        // (providerSlot, element) pairs this solver writes
        std::vector<std::pair<int, int>> outputs;

        // ---- the Solve step's own scratch, sized at Build -----------------
        //
        // The candidates this solver published, in `outputs` order and
        // nowhere else: the merge into the batch's table is the commit's
        // job, so two solvers of one batch never write the same storage.
        std::vector<RigExecPointFrame> outFrames;
        std::vector<char> outPresent;
        /// Where each output lands in its commit's slot-ordered table.
        std::vector<int> outPosition;
        /// Joints this solver published no element for. Merged with every
        /// other solver's list into one ordered set by the epilogue.
        std::vector<SdfPath> fallbackJoints;
        /// FkChain's element table, so the solve allocates nothing.
        std::vector<RigExecFkChainElement> elements;
    };
    std::vector<Solver> solvers;
    std::map<SdfPath, int> solverIndex;
    std::vector<RigExecPointFrameArray> aggregates;

    // ---- constraints -------------------------------------------------------
    struct Constraint {
        SdfPath path;
        TfToken type;
        int target = -1;
        std::vector<int> sources;
        RigExecBakedInput<bool> enabled;
        RigExecBakedInput<float> defaultWeight;
        // The authored table only.
        std::vector<RigExecBakedInput<float>> sourceWeights;
        size_t authoredSourceWeights = 0;
        std::vector<GfVec3d> translationOffsets, rotationOffsets;
        bool offsetsVary = false;
        // position/rotation/scale
        RigExecBakedInput<GfVec3d> offset;
        // The operator's own affect group.
        RigExecBakedInput<bool> affectX, affectY, affectZ;
        RigExecBakedInput<bool> tX, tY, tZ, rX, rY, rZ, sX, sY, sZ;  // parent
        RigExecEulerOrder order = RigExecEulerOrder::XYZ;
        // aim
        RigExecBakedInput<GfVec3d> aimVector, upVector, rotationOffset,
            worldUpVector;
        GfVec3d aimAxisFallback{1, 0, 0};
        bool aimVectorAuthored = false;
        bool preserveInputUp = false;
        TfToken worldUpType;
        GfVec3d sceneUp{0, 1, 0};
        int worldUpObject = -1;
        bool worldUpObjectNamed = false;
        /// A read phase named this constraint as the point in the walk it
        /// wants its target's frame from, so the walk records the target's
        /// matrix after it. Decided at bake out of the evaluator's
        /// _snapshotPoints, which is the same membership the dynamic
        /// recordFrame tests per call.
        bool snapshotAfter = false;
    };
    std::vector<Constraint> constraints;

    // ---- the walk ----------------------------------------------------------
    struct WalkStep {
        bool solverBatch = false;
        size_t level = 0;
        int index = 0;                                 // constraint slot
        std::vector<int> batchSolvers;
        // Descendant propagation, decided at bake: the closest changed
        // ancestor each descendant rides, with the ownership-blocking rule
        // already applied. The dynamic walk rediscovers these by climbing
        // the namespace per constraint per frame, which is what makes a long
        // chain quadratic.
        std::vector<std::pair<int, int>> propagate;
    };
    std::vector<WalkStep> walkSteps;

    // ---- the step graph ----------------------------------------------------
    //
    // Built from the tables above, once, at the end of Build. `steps` is in
    // program order -- the order today's straight-line Run visited the same
    // work in -- and every edge points forward in it.
    std::vector<RigExecBakedStep> steps;
    /// The compose pass, partitioned into contiguous subtrees at Build.
    std::vector<RigExecBakedComposeGroup> composeGroups;
    /// One per walk entry, in walk order; `walkSteps[w]` and `commits[w]`
    /// describe the same commit.
    std::vector<RigExecBakedCommit> commits;
    /// (chain, revision) per dense revision id, and (chain, derived) per
    /// dense derived id. Both are assigned chain by chain, so one chain's
    /// revisions are a contiguous range and a chain-wide read is one range.
    std::vector<std::pair<int, int>> revisionIndex;
    std::vector<std::pair<int, int>> derivedIndex;
    /// First and last-plus-one revision id of each chain.
    std::vector<int> chainRevisionBegin, chainRevisionEnd;

    /// Program constants the epilogue adds to the generation's counters.
    ///
    /// Both are structural, not observations: the dynamic path's override
    /// rounds are the number of distinct batch levels the schedule holds,
    /// and the baked meaning of solverEvaluations is the number of solver
    /// computations the program requests, which is the sum of the batch
    /// sizes and is the same every run.
    size_t solverOverrideRounds = 0;
    size_t solverEvaluations = 0;

    // ---- publication -------------------------------------------------------
    std::vector<int> jointSlots;
    std::vector<SdfPath> jointPaths;
    std::vector<int> controlSlots;
    std::vector<SdfPath> controlPaths;
    std::vector<std::pair<SdfPath, int>> solverArrays;  // guide publication

    // ---- geometry ----------------------------------------------------------
    // One entry per revision of one chain, in chain order. The packet is
    // assembled by the SAME RigExecAssembleParameters the dynamic path calls
    // and the kernel is the same shared kernel, so only the plumbing around
    // them is baked.
    struct GeomRevision {
        SdfPath moverPath;
        SdfPath target;
        UsdPrim moverPrim;
        RigExecRevisionOp op = RigExecRevisionOp::Skin;
        RigExecRevisionBinding binding;
        std::vector<int> influenceSlots;
        int transformSlot = -1;
        /// The solver whose aggregate supplies values.driverFrames, as an
        /// index into `solvers`, or -1 when this revision reads none. The
        /// dynamic path takes it off a per-revision tap on the solver's
        /// computePointFrameArray; the program already holds that aggregate,
        /// so the tap becomes a slot. Nothing fills it yet: IsBakeable still
        /// refuses "driver frames on mover".
        int driverFramesSolver = -1;
        bool finalPhase = false;
        /// Compile's judgement that none of this skin mover's layout arrays
        /// can change within the epoch, so the packet may carry the layout
        /// by handle out of the evaluator's cache instead of re-reading and
        /// re-validating 105k elements every frame.
        bool skinTopologyFixed = false;
        /// The node's last published status, which outlives an evaluation it
        /// did not take part in -- so does its diagnostic.
        TfToken resultStatus;
        std::vector<GfVec3f> output;
        // Last evaluated packet/status and result, so an unchanged input
        // re-publishes instead of re-running -- which is the accounting the
        // VdfNetwork performs for the dynamic path.
        RigExecMoverParameters lastParameters;
        RigExecMoverStatus lastStatus;
        bool ran = false;
        /// A read phase named this revision as the point in the chain it
        /// wants the target's points from, so the chain records them after
        /// it. Decided at bake out of the evaluator's _snapshotPoints.
        bool snapshotAfter = false;
        /// This node is new to the rig's geometry state and its creation has
        /// not been reported yet. Cleared by AdoptGeometryStateFrom for a
        /// node the outgoing program already held, which is the same
        /// bookkeeping the dynamic walk does when it reconnects a surviving
        /// VdfNetwork node instead of adding one.
        bool created = true;

        // ---- what the four steps of a revision hand each other ------------
        /// InfluenceFold writes these; RevisionStatic assembles against
        /// them. Sized at Build, never resized in the region.
        std::vector<GfMatrix4d> influences;
        GfMatrix4d transform{1.0};
        /// Whether `transform` holds one at all this run: a bound transform
        /// provider, or a geometry-domain constraint's delta. Written by the
        /// fold beside the table, because "there is a matrix" is part of what
        /// the table says.
        bool haveTransform = false;
        /// This revision's overlay of the run's snapshot store, built only
        /// when the revision declares a read phase. Per revision, never one
        /// buffer shared by the walk.
        RigExecResolvedInputs revisionInputs;
        /// RevisionStatic writes these three.
        RigExecMoverParameters parameters;
        RigExecMoverStatus status;
        /// Whether this revision has to run at all: chain dirty so far, or
        /// it never ran, or its packet or status moved. A VALUE comparison,
        /// never a dirtiness flag -- a control dragged back to where it
        /// started must not count as executed.
        bool executed = false;
        /// RevisionChunk writes this: the kernel accepted the packet.
        bool chunkOk = false;
        /// Which buffer holds the chain's points AFTER this revision: this
        /// revision's index when its own output was kept, the index of an
        /// earlier revision when it was not, and -1 for the chain's base.
        /// The indirection is what replaces today's `revision.output =
        /// current` copy of a revision that applied nothing.
        int currentSource = -1;
        /// The epoch-fixed skin layout, resolved in the prologue because
        /// RigExecSkinTopologyCache::Resolve holds a mutex across the build
        /// and no step body may take a lock. Null is a remembered refusal,
        /// which is why `topologyResolved` and not the pointer says whether
        /// the prologue answered.
        std::shared_ptr<const RigExecSkinTopology> topology;
        bool topologyResolved = false;
    };
    struct GeomChain {
        SdfPath target;
        UsdAttributeQuery baseQuery;
        std::vector<GeomRevision> revisions;
        VtVec3fArray lastBase;
        VtVec3fArray result;
        /// The other half of the published double buffer. Publication
        /// alternates between `result` and this one, so the array a consumer
        /// may still hold from last frame is never the one being written --
        /// which is the COW aliasing rule VtArray leaves to its callers.
        VtVec3fArray spare;
        bool haveResult = false;
        /// The base attribute read at this time, by the prologue. False
        /// skips every step of the chain, exactly as today's `continue`
        /// skips the chain and its counters.
        bool haveBase = false;
        /// Whether the chain's points moved before its first revision, which
        /// is where the sticky chain-dirty bit starts.
        bool baseDirty = false;
        /// This chain's schedule has to be built and that has not been
        /// reported yet. The dynamic path rebuilds a chain's schedule when
        /// the identity sequence of its revisions changed, and not for a
        /// packet that merely holds different numbers.
        bool scheduleDirty = true;
        // Derived maintenance reads this chain's FINAL points.
        struct Derived {
            SdfPath target;
            UsdAttributeQuery baseQuery;
            GeomRevision revision;
            VtVec3fArray lastBase;
            VtVec3fArray result;
            /// The other half of the published double buffer; see the
            /// chain's.
            VtVec3fArray spare;
            bool haveResult = false;
            /// As the chain's, read by the prologue.
            bool haveBase = false;
            bool baseDirty = false;
        };
        std::vector<Derived> derived;
    };
    std::vector<GeomChain> chains;


    /// What each geometry-domain constraint measured, keyed by the MOVER
    /// that produced it: the delta between its solved frame and its target's
    /// authored transform. Emptied at the head of every run and consumed by
    /// the geometry half's packet assembly; the WRITER is the pose walk's
    /// geometry-domain constraint, which does not exist yet -- IsBakeable
    /// refuses one, so the map is empty on every rig that bakes today. The
    /// hand-off is here, in the same direction, because it is the one the
    /// dynamic walk performs with its own constraintDeltas map, and the
    /// group that adds the writer should not have to invent it.
    std::map<SdfPath, GfMatrix4d> constraintDeltas;

    // ---- weight objects ----------------------------------------------------
    //
    // One entry per weight object the epoch reaches, in DEPENDENCY ORDER
    // (post-order over rigExec:baseWeight and rigExec:inputWeights, children
    // before parents), so one forward pass per frame builds every packet and
    // a composed object finds its inputs already built.
    //
    // The sharing matters as much as the order: exec's
    // Relationship().TargetedObjects<RigExecWeightPacket>() accessor gives
    // one packet per weight object per generation no matter how many movers
    // bind it. Building one per REVISION would be numerically identical and
    // would recompute a volume field over a whole mesh once per mover, which
    // is a performance regression rather than a parity one -- so the table is
    // keyed by the object, not by the consumer.
    struct WeightObject {
        SdfPath path;
        TfToken type;
        TfToken representation, rangePolicy, operation;
        // RigExecStaticWeight: every field is uniform, so all three fold --
        // but they are still REGISTERED, so a drag on a painted weight can
        // be placed.
        std::vector<float> values;
        std::vector<int> indices;
        RigExecBakedInput<float> defaultWeight;
        /// rigExec:baseWeight, as an index into `weightObjects`, or -1.
        int base = -1;
        /// rigExec:inputWeights in AUTHORED order (subtract and overlay are
        /// order dependent), as indices into `weightObjects`.
        std::vector<int> inputs;
        // RigExecDynamicWeight.
        RigExecBakedInput<float> driver, scale, bias;
        // Combine.
        TfToken combineMode;
        RigExecBakedInput<float> strength, invert;
        size_t weightTargetCount = 0;
        /// True when anything this object reads can move between frames --
        /// its own bound inputs, or any object it composes. A false one is
        /// built once and replayed.
        bool varying = false;
        RigExecWeightPacket cached;
        bool haveCached = false;
    };
    std::vector<WeightObject> weightObjects;
    /// Path to index in weightObjects. A NEGATIVE entry is an object whose
    /// composition walk is under way (the bake is depth first and enters the
    /// table on the way out), so meeting one is a cycle.
    std::map<SdfPath, int> weightIndex;

    // ---- the invalidation index --------------------------------------------
    //
    // What the bake looked at, so a notice can be answered without rebuilding
    // and without guessing. Four sets, because a notice asks four different
    // questions and one set would have to answer the bluntest of them:
    //
    //  * `rebuild` -- properties whose VALUE decided something the program
    //    holds: a folded constant, or the selection of a per-frame query.
    //    A changed-info notice on one of these rebuilds the program.
    //  * `named` -- every property the bake ASKED ABOUT, found or not. A
    //    resync is a property appearing, disappearing or being retargeted,
    //    and it also invalidates any retained query on that property, so
    //    this is a superset of `rebuild`: it includes the inputs whose
    //    values are re-read per frame and the names that resolved to
    //    nothing.
    //  * `prims` -- every prim the bake read anything from, for a resync
    //    that names a whole subtree rather than one property.
    //  * `xformPrims` -- the Xforms between an intervening-Xform candidate
    //    and its anchor. Bakeability accepted those BECAUSE they compose to
    //    the identity today, which is a judgement about their composed
    //    transform and not about one attribute, so any property of theirs
    //    counts.
    //
    // A changed-info notice that misses all of them is a value edit on an
    // input the frame path re-reads, and needs nothing.
    std::set<SdfPath> rebuild;
    std::set<SdfPath> named;
    std::set<SdfPath> prims;
    std::set<SdfPath> xformPrims;

    // ---- interactive override placement -------------------------------------
    // One flag per registered input; see RigExecBakedRead. Kept as a dense
    // vector so the frame path costs an index rather than a map lookup per
    // read.
    std::vector<char> overridden;
    /// Property path -> the inputs reading it, for the placeable case.
    std::map<SdfPath, std::vector<int>> overridableInputs;
    /// Prims whose reads already route through the generation's resolved
    /// inputs (property-chain movers, geometry movers), so an override on
    /// any of their properties reaches them with nothing else to do.
    std::set<SdfPath> resolvedRoutedPrims;
    /// True while the dense avar table still holds a value written for a
    /// drag; see the input block in Run.
    bool avarsDisturbed = false;
    /// Properties folded into bake state. An override here cannot be placed
    /// without rebaking, so it forces the dynamic path instead.
    std::set<SdfPath> folded;
    bool anyOverridden = false;

    /// Registers \p input so an override can be placed on it, and records
    /// what a notice would have to touch to invalidate what was captured.
    template <class T>
    void Register(RigExecBakedInput<T> *input, const SdfPathVector &walk) {
        if (!input->head) {
            return;
        }
        input->overrideIndex = int(overridden.size());
        overridden.push_back(0);
        // EVERY attribute on the resolution walk, not only the head one.
        // RigExecBakedClassifyInput followed an authored connection chain
        // and may have captured the value several hops upstream; an
        // interactive override
        // standing on one of those hops is an override on this input, and
        // GetAttribute -- which is what the flag makes RigExecBakedRead
        // use -- consults the generation's resolved values at every step of
        // the same walk.
        // Keyed by the head alone, such an override found no binding and was
        // reported placeable with nothing placed.
        for (const SdfPath &path : walk) {
            overridableInputs[path].push_back(input->overrideIndex);
        }
        if (walk.empty()) {
            overridableInputs[input->head.GetPath()].push_back(
                input->overrideIndex);
        }
        prims.insert(input->head.GetPrim().GetPath());
        named.insert(walk.begin(), walk.end());
        // A chain-resolved input redoes the whole walk live every frame, so
        // nothing about it was captured. A query was pinned to ONE attribute
        // of the walk, so every other attribute on it decided that choice;
        // with a walk of one there is no choice left to invalidate, and a
        // value moving on it -- including being cleared, which drops the
        // input back to the same fallback both paths use -- is answered by
        // the query itself.
        const bool live = input->resolvedAttr || (input->varying &&
                                                  walk.size() == 1);
        if (live) {
            return;
        }
        rebuild.insert(walk.begin(), walk.end());
    }
};

// ---------------------------------------------------------------------------
// The compiled epoch, in terms the program can name.
//
// RigExecRigEvaluator's compiled structures are private nested types, so only
// bakedProgram.cpp can read them. It restates the parts each domain's bake
// needs as the plain records below and hands those over; the per-domain bake
// functions then depend on the SHAPE of the epoch rather than on the
// evaluator, which is what keeps the friendship to one file.
// ---------------------------------------------------------------------------

/// One frame constraint of the compiled walk.
struct RigExecBakedConstraintSpec {
    SdfPath moverPath;
    TfToken schemaType;
    SdfPathVector targets;
    /// Whether a read phase asked for the target's frame as of this
    /// constraint (the evaluator's _snapshotPoints membership).
    bool snapshotAfter = false;
    /// One source path per binding, in the compiled order.
    SdfPathVector sources;
    /// Empty when the aim constraint named no world-up object.
    SdfPath worldUpObject;
};

/// One geometry revision of a compiled chain.
struct RigExecBakedRevisionSpec {
    SdfPath moverPath;
    SdfPath target;
    RigExecRevisionOp op = RigExecRevisionOp::Skin;
    RigExecRevisionBinding binding;
    bool transformFinalPhase = false;
    bool skinTopologyFixed = false;
    /// Whether a read phase asked for the target's points as of this
    /// revision (the evaluator's _snapshotPoints membership).
    bool snapshotAfter = false;
};

/// One geometry chain and the derived targets maintained from its result.
struct RigExecBakedChainSpec {
    SdfPath target;
    std::vector<RigExecBakedRevisionSpec> revisions;
    std::vector<RigExecBakedRevisionSpec> derived;
};

/// One entry of the interleaved solver/constraint walk.
struct RigExecBakedWalkEntry {
    bool solverBatch = false;
    /// Solver batches only: the batch's dependency level, and its solvers in
    /// batch order with the (joint, element) outputs each one publishes.
    size_t level = 0;
    SdfPathVector batchSolvers;
    std::vector<std::vector<std::pair<SdfPath, int>>> solverJoints;
    /// Constraints only.
    RigExecBakedConstraintSpec constraint;
};

/// The Build-time scratch every per-domain bake function shares.
///
/// The lambdas Build used to close over -- refuse, fold, readToken, targets,
/// bind, slotOf -- became these members when the bake split across files, so
/// that every domain records the same invalidation facts for the same read.
/// Dropping one of them is how a captured constant stops being invalidated,
/// which is silent until an edit produces a wrong answer.
struct RigExecBakedBuildContext {
    RigExecBakedProgramImpl *program = nullptr;
    UsdStageRefPtr stage;
    /// Where a captured value is read, and where input classification probes
    /// so that a selection cannot depend on the caller's time code.
    UsdTimeCode capture = UsdTimeCode::Default();
    UsdTimeCode probe = UsdTimeCode::Default();
    /// Every property a chain writes; an input resolving through one cannot
    /// be captured.
    std::set<SdfPath> chainTargets;
    std::vector<std::string> *reasons = nullptr;
    bool ok = true;

    /// Records one reason the epoch cannot be expressed, and refuses it.
    void Refuse(const std::string &what, const SdfPath &where);
    /// Records \p name as read for its VALUE: an edit rebuilds the program
    /// and an interactive override on it cannot be placed.
    void Fold(const UsdPrim &prim, const char *name);
    /// Records \p name as read for its SHAPE -- whether it is authored at
    /// all. An edit rebuilds; an override, which authors nothing, places.
    void FoldShape(const UsdPrim &prim, const char *name);
    /// Fold plus the plain uniform-token read the dynamic path performs.
    TfToken ReadToken(const UsdPrim &prim, const char *name,
                      const char *fallback);
    /// Fold plus the relationship's targets.
    SdfPathVector Targets(const UsdPrim &prim, const char *name);
    /// The provider slot of \p path, or -1.
    ///
    /// EITHER kind: the table is the ordered union of the exec-seeded
    /// providers and the plain Xformables a constraint targets, so a
    /// `SlotOf(x) < 0` refusal reads "is no provider slot at all", NOT "is
    /// not a pose provider". The two coincide only because IsBakeable still
    /// refuses a rig with any xform-derived provider ("constraint target is
    /// a plain Xformable"); that refusal is the sole guard. A caller that
    /// means "publishes computeRestFrame" or "is composed from avars" must
    /// test slotKind[slot] == PoseSeed itself, as the solver rest binding
    /// does, and the Phase 3 group that lifts the refusal has to visit every
    /// site that does not.
    int SlotOf(const SdfPath &path) const;
    /// Binds \p name as a per-frame input, registering it for overrides and
    /// for invalidation.
    template <class T>
    RigExecBakedInput<T> Bind(const UsdPrim &prim, const char *name,
                              T fallback);
};

template <class T>
RigExecBakedInput<T>
RigExecBakedBuildContext::Bind(const UsdPrim &prim, const char *name,
                               T fallback)
{
    RigExecBakedProgramImpl &B = *program;
    SdfPathVector walk;
    RigExecBakedInput<T> input =
        RigExecBakedBindInput(prim, name, fallback, capture, chainTargets,
                              &walk);
    // A selection that moves with the time code is read the long way,
    // through this generation's resolved inputs, rather than through a
    // query pinned to the wrong attribute.
    if (input.varying && input.query.IsValid()) {
        bool viaChain = false, varying = false;
        UsdAttribute atProbe;
        RigExecBakedClassifyInput<T>(prim.GetAttribute(TfToken(name)), probe,
                                     chainTargets, &viaChain, &varying,
                                     &atProbe);
        if (atProbe.GetPath() != input.query.GetAttribute().GetPath()) {
            input.query = UsdAttributeQuery();
            input.resolvedAttr = prim.GetAttribute(TfToken(name));
        }
    }
    if (prim && prim.GetAttribute(TfToken(name))) {
        ++B.boundInputs;
        if (input.varying) ++B.varyingInputs;
    }
    if (prim) {
        B.prims.insert(prim.GetPath());
        // Named even when absent: a resync that CREATES this property is
        // an input appearing, and the bake captured the default it did
        // not find.
        B.named.insert(prim.GetPath().AppendProperty(TfToken(name)));
    }
    B.Register(&input, walk);
    return input;
}

// ---------------------------------------------------------------------------
// The per-domain halves of Build and Run.
//
// Build calls the two builders in program order and Run calls the two
// executors in program order; each pair lives in one file so that a domain's
// bake and its frame path are read side by side.
// ---------------------------------------------------------------------------

/// Bakes the interleaved solver/constraint walk into \p ctx's program.
void RigExecBakedBuildWalk(RigExecBakedBuildContext *ctx,
                           const std::vector<RigExecBakedWalkEntry> &walk);

/// Bakes the geometry chains and their derived targets into \p ctx's program.
void RigExecBakedBuildGeometry(
    RigExecBakedBuildContext *ctx,
    const std::vector<RigExecBakedChainSpec> &chains);

/// Appends the pose half of the program in program order: the compose
/// subtrees, then one Solve and one commit per walk entry, then the
/// rest->pose matrices and the phased-read finals.
void RigExecBakedBuildPoseSteps(RigExecBakedProgramImpl *program);

/// Appends the geometry half: per revision an InfluenceFold, a
/// RevisionStatic, its chunks and a RevisionFuse, then the chain's status
/// sweep, then its derived targets.
void RigExecBakedBuildGeometrySteps(RigExecBakedProgramImpl *program);

/// Runs one pose step. Never touches the pose: everything it produces goes
/// into \p step.
void RigExecBakedRunPoseStep(RigExecBakedProgramImpl *program,
                             RigExecBakedStep *step, UsdTimeCode time);

/// Runs one geometry step, under the same rule.
void RigExecBakedRunGeometryStep(RigExecBakedProgramImpl *program,
                                 RigExecBakedStep *step, UsdTimeCode time);

/// The pose half of the prologue: the bound inputs, once per run.
void RigExecBakedRunInputs(RigExecBakedProgramImpl *program, UsdTimeCode time);

/// The geometry half of the prologue: every chain's and derived target's
/// authored base, the point-count-moved reset, the node-creation accounting,
/// and the skin layouts -- the one lock a frame takes, kept out of the
/// region.
void RigExecBakedRunGeometryPrologue(RigExecBakedProgramImpl *program,
                                     UsdTimeCode time, RigExecRigPose *pose);

/// The pose half of the epilogue: the joint and control publication, the
/// solver guides and the property-domain results.
///
/// Returns false where today's walk returns false from the same line: a
/// joint with a valid, non-degenerate final frame whose rest or frame is
/// not usable needs exec.
bool RigExecBakedPublishPose(RigExecBakedProgramImpl *program,
                             RigExecRigPose *pose);

/// The geometry half of the epilogue: each chain's diagnostics and points
/// and each derived target's, in chain order.
void RigExecBakedPublishGeometry(RigExecBakedProgramImpl *program,
                                 RigExecRigPose *pose);

/// Bakes the weight object at \p path, and everything it composes, into
/// \p ctx's table; returns its index, or -1 when there is nothing there.
///
/// Memoized on RigExecBakedProgramImpl::weightIndex, so a weight object ten
/// movers bind is baked once and every consumer gets the same index; the
/// same map carries an under-way marker, so a composition cycle is refused
/// rather than recursed into. Binds every readable field through the build
/// context, which is what keeps the invalidation index and interactive-
/// override placement honest with no further code.
int RigExecBakedBakeWeightObject(RigExecBakedBuildContext *ctx,
                                 const SdfPath &path);

/// The packet \p object publishes this frame.
///
/// \p packets holds the packets already built for the objects BEFORE this
/// one in the table, which dependency order guarantees are the ones it
/// composes.
RigExecWeightPacket RigExecBakedWeightPacket(
    const RigExecBakedProgramImpl &program,
    const RigExecBakedProgramImpl::WeightObject &object,
    const std::vector<RigExecWeightPacket> &packets, UsdTimeCode time);

}  // namespace rigExec

#endif  // RIGEXEC_BAKED_PROGRAM_IMPL_H
