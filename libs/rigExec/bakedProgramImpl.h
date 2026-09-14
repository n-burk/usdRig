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
#include "curvenetWeightComputations.h"
#include "weightPackets.h"

#include "rigExecMath/avarScale.h"
#include "rigExecMath/dualQuat.h"
#include "rigExecMath/pointFrame.h"
#include "rigExecMath/singleChainIk.h"
#include "rigExecMath/solvers.h"
#include "rigExecMath/splineIk.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
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
#include <atomic>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#if defined(_MSC_VER)
#include <intrin.h>
#endif

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

/// Publishes \p value at \p key into \p map, in one comparison when the
/// caller walks its keys in ascending order.
///
/// \p inOrder is the caller's promise that every key it publishes into this
/// map is strictly greater than the last one it published, which makes the
/// map's end the correct hint and the insertion O(1) rather than a search
/// from the root. Strictly greater matters twice over: it is what makes the
/// hint right, and it is what makes an emplace the same publication as the
/// assignment it replaced, because no key is ever published twice. Without
/// the promise this IS that assignment.
template <class Map, class Value>
inline void
RigExecBakedEmplace(Map *map, bool inOrder, const SdfPath &key,
                    const Value &value)
{
    if (inOrder) {
        map->emplace_hint(map->end(), key, value);
    } else {
        (*map)[key] = value;
    }
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
    SolverPoints,        ///< one ribbon's live driver points; the prologue
    Candidates,          ///< one Solve step's own (slot, frame) scratch
    CommitTable,         ///< one commit's merged candidate table
    CommitDelta,         ///< one commit's per-candidate hierarchy delta
    CommitStaging,       ///< one propagation pair's staged frame and outcome
    ConstraintDelta,     ///< one geometry-domain constraint's measured delta
    PropertyResult,      ///< propertyResults[t]; filled by the prologue
    ChainBase,           ///< chains[c].lastBase; filled by the prologue
    RevisionPacket,      ///< one revision's assembled packet, status, executed
    RevisionTransforms,  ///< one revision's influence table
    RevisionOut,         ///< one revision's OWN output buffer
    RevisionDone,        ///< one revision's applied/status flags
    ChainDirty,          ///< the chain's sticky dirty bit as of one revision
    ChainPoints,         ///< the chain's published points
    DerivedOut,          ///< one derived target's output
    WeightPacket,        ///< one weight object's packet for this frame
    WeightFrames,        ///< where every volume weight is placed, as one slot
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

/// Whether \p domain's storage is SSA, one entry per writer (§3.1).
///
/// The two pose frame domains are. Everything else has one writer per slot
/// per run, or is scratch a single step owns, so there is no version to
/// speak of: the entry IS the value. The distinction is the edge sweep's:
/// a write-after-read edge exists only where a writer can land on storage a
/// reader is still entitled to, and in a versioned domain it never can.
inline bool
RigExecBakedIsVersionedDomain(RigExecBakedSlotDomain domain)
{
    return domain == RigExecBakedSlotDomain::PoseFin ||
           domain == RigExecBakedSlotDomain::PoseBase;
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
    VolumePlacements, ///< every volume weight's placement, from the walk
    WeightPacket,     ///< one weight object's packet, built once per frame
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
    ///   VolumePlacements                            unused
    ///   WeightPacket                                index into weightObjects
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
    ///
    /// There is no second list beside this one. A step used to also carry
    /// its RESTORE predecessors -- the writers whose version the end of a
    /// run no longer held, because a later step had overwritten the slot --
    /// and cone re-execution had to close over them. Versioned storage
    /// (§3.1) makes that list empty by construction: no version is ever
    /// overwritten, so re-running a reader never means re-running a writer
    /// to put a slot back.
    std::vector<int> preds, succs;

    // ---- what a run outside the graph can change ---------------------------
    //
    // A step is a pure function of its declared reads with three exceptions,
    // and every one of them is recorded here so that the dirty set can name
    // it rather than the executor having to guess (§7).
    /// The step reads nothing but source slots, so it can be -- and is --
    /// run before the dirty set is computed, every run, and compared by
    /// VALUE. RevisionStatic on a skin revision is the only one today.
    bool isSource = false;
    /// The step reads outside the program at a point the graph cannot order
    /// -- a packet assembled against the influence table, a derived target's
    /// own inputs -- and is not a source because it has predecessors. Such a
    /// step's cluster is dirty every run; its own value comparisons are what
    /// keep the counters honest.
    bool externalReads = false;
    /// The step reads a baked input whose value is a function of TIME, so a
    /// frame at a different time may hand it different numbers.
    bool varyingInputs = false;
    /// The step reads a baked input that resolves through the generation's
    /// resolved inputs every frame -- a property chain's output.
    bool resolvedInputReads = false;
    /// The override indices of the baked inputs this step reads, sorted and
    /// deduplicated. A step is dirty while an override stands on one of
    /// them, and for the one run after it is lifted.
    std::vector<int> overrideInputs;

    /// Filled by the clustering pass; the serial executor ignores all four.
    /// `level` is the longest-path level the packing groups by, `sizeUnits`
    /// the size term of the cost model (§5.1) and `cost` the microseconds
    /// that model predicts.
    int cluster = -1;
    int level = 0;
    double sizeUnits = 0;
    double cost = 0;

    /// What the calibration mode measured: the summed interval of this step
    /// over the frames it watched, and how many of them it saw. Untouched
    /// unless RIGEXEC_BAKED_SCHEDULE_CALIBRATE asked for a measurement, and
    /// written only by the serial executor on the one thread it runs on --
    /// which is what "lock-free per-step timer" comes to.
    double measuredUs = 0;
    uint32_t measuredRuns = 0;

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
    /// to replay. The timestamps are NOT cleared here for that same reason:
    /// a skipped step never reaches this, so RigExecBakedRunSteps clears
    /// every step's interval at the start of the run instead.
    void BeginRun() {
        diagnostics.clear();
        counters.Clear();
        snapshots.Clear();
        bail = false;
    }

    /// Records that this run skipped the step (§7).
    ///
    /// The diagnostics stay, and so do the two counters that are PROGRAM
    /// CONSTANTS: a skipped step's lines describe state nothing changed, and
    /// how many chains and revisions the program holds is a property of the
    /// program rather than of a run (§4.2). What does not stay is the three
    /// counters that are OBSERVATIONS of this run -- a revision the frame
    /// did not execute did not execute, a node it did not report did not
    /// report its creation, a schedule it did not build did not build one --
    /// nor the bail flag, which belongs to the run that raised it. The three
    /// are reported again by the run that does the work, because what they
    /// count is held in state that outlives a skip (`executed` off a value
    /// comparison, `created` and `scheduleDirty` off one-shot flags the
    /// geometry prologue clears only when it counts them).
    void MarkSkipped() {
        counters.revisionsExecuted = 0;
        counters.revisionsCreated = 0;
        counters.schedulesBuilt = 0;
        bail = false;
    }
};

/// One cluster: the steps one task runs, back to back, on one thread.
///
/// A cluster's members are in increasing PROGRAM index, which is always a
/// topological order because every edge of the step graph points forward in
/// program order (§4.1). So a cluster needs no internal schedule: the loop
/// that runs its members in order is the schedule.
struct RigExecBakedCluster {
    /// Step indices, strictly increasing.
    std::vector<int> members;
    /// Cluster indices, sorted and deduplicated.
    std::vector<int> preds, succs;
    /// The summed cost of the members, in the cost model's microseconds.
    double cost = 0;
    /// The highest step level in the cluster, which is what the packing
    /// grouped by and what the report sorts on.
    int level = 0;

    // ---- what the last run did, filled by the parallel executor ----------
    /// When the cluster's last predecessor finished, when it started and
    /// when it ended, as RigExecProfiler::NowUs() reads. Recorded only while
    /// the schedule report or the profiler asks for them, so a production
    /// frame pays no clock reads for a table nobody prints.
    uint64_t readyUs = 0, startUs = 0, endUs = 0;
};

/// A partition of one program's steps into clusters.
///
/// A VALUE, not program state: RigExecBakedBuildClusters computes one from a
/// program and a grain without touching it, which is what lets a test ask
/// the same program for the schedule at three different grains and compare
/// them. The program holds one of these -- the partition Build chose -- and
/// the parallel executor runs that one.
struct RigExecBakedClustering {
    std::vector<RigExecBakedCluster> clusters;
    /// The cluster of each step, one entry per step. Every step is in
    /// exactly one cluster.
    std::vector<int> clusterOf;
    /// The grain this partition was packed to, in microseconds. Zero means
    /// "one step per cluster", which is the finest schedule the graph admits
    /// and the strongest test of it.
    double grainUs = 0;
    /// The summed cost of every step, and the longest path through the
    /// cluster graph by cost. Their ratio is the speed-up this schedule can
    /// reach with threads to spare.
    double serialCost = 0, criticalPathCost = 0;
    /// Whether the last run stamped the per-cluster times above. Only the
    /// parallel executor has clusters to time: a serial run walks the steps
    /// and never asks which cluster they are in, so its run report says so
    /// rather than printing a table of zeros that reads as "every cluster
    /// was free".
    bool lastRunTimed = false;
};

/// The remaining-predecessor counter of one cluster.
///
/// One cache line each. Two counters in one line would make every finishing
/// cluster's decrement invalidate its neighbour's line, which on a graph
/// this wide is the only contention the executor has -- and the only lock,
/// mutex or condition variable in the whole region is this atomic (§2.2).
struct alignas(64) RigExecBakedClusterCounter {
    std::atomic<int> remaining{0};
    char padding[64 - sizeof(std::atomic<int>)] = {};
};

/// Population count of one 64-bit word, on every supported compiler.
///
/// MSVC has no __builtin_popcountll; its equivalent is __popcnt64 from
/// <intrin.h>. The bit-twiddling fallback is for a compiler with neither
/// (C++20's std::popcount is not available under this project's C++17).
inline size_t _RigExecPopcount64(uint64_t word)
{
#if defined(_MSC_VER)
    return size_t(__popcnt64(word));
#elif defined(__GNUC__) || defined(__clang__)
    return size_t(__builtin_popcountll(word));
#else
    word -= (word >> 1) & uint64_t(0x5555555555555555);
    word = (word & uint64_t(0x3333333333333333)) +
           ((word >> 2) & uint64_t(0x3333333333333333));
    word = (word + (word >> 4)) & uint64_t(0x0f0f0f0f0f0f0f0f);
    return size_t((word * uint64_t(0x0101010101010101)) >> 56);
#endif
}

/// A bitset over clusters, as many 64-bit words as the program needs.
///
/// Cone re-execution is a handful of set unions per frame over sets whose
/// membership was decided at Build, so the representation that matters is
/// the one a union is a word loop over. |clusters| is a few dozen on a
/// biped, so this is one or two words.
struct RigExecBakedClusterSet {
    std::vector<uint64_t> words;

    void Resize(size_t clusters) {
        words.assign((clusters + 63) / 64, 0);
    }
    void Clear() { std::fill(words.begin(), words.end(), uint64_t(0)); }
    bool Test(int cluster) const {
        return cluster >= 0 &&
               (words[size_t(cluster) >> 6] >> (size_t(cluster) & 63)) & 1;
    }
    void Set(int cluster) {
        if (cluster >= 0) {
            words[size_t(cluster) >> 6] |=
                uint64_t(1) << (size_t(cluster) & 63);
        }
    }
    void SetAll(size_t clusters) {
        Resize(clusters);
        for (size_t c = 0; c < clusters; ++c) {
            Set(int(c));
        }
    }
    /// Whether anything was added, which is the fixpoint test.
    bool Union(const RigExecBakedClusterSet &other) {
        bool grew = false;
        for (size_t w = 0; w < words.size(); ++w) {
            const uint64_t before = words[w];
            words[w] |= other.words[w];
            grew = grew || words[w] != before;
        }
        return grew;
    }
    bool Any() const {
        for (const uint64_t word : words) {
            if (word) return true;
        }
        return false;
    }
    size_t Count() const {
        size_t count = 0;
        for (const uint64_t word : words) {
            count += _RigExecPopcount64(word);
        }
        return count;
    }
};

/// What Build knows about re-running part of a program (§7).
///
/// Both families are closures computed once, over clusters rather than over
/// steps, because a cluster is what the executor can skip: `cone[c]` is
/// everything that has to run when c does, and `restore[c]` is everything
/// that has to run BEFORE c can, so that the slots c reads hold the version
/// its program point saw rather than the version the end of the last run
/// left behind.
struct RigExecBakedCones {
    /// Forward closure of each cluster, including itself.
    ///
    /// The only closure there is. The restore closure that used to sit
    /// beside it -- "run this cluster and all of THIS had to have run first"
    /// -- is gone with the storage that made it necessary (§3.1).
    std::vector<RigExecBakedClusterSet> cone;
    /// Clusters holding a step that reads outside the graph at a point the
    /// graph cannot order. Dirty every run.
    RigExecBakedClusterSet always;
    /// Clusters holding any step that is not a geometry step. The FIRST run
    /// of a program is dirty here and in the geometry clusters of the
    /// revisions a rebuild did not carry, rather than everywhere (§7 [S28]).
    RigExecBakedClusterSet poseClusters;
    /// Provider slot -> the cluster of the compose step that reads its
    /// avars, or -1. What an avar that moved makes dirty.
    std::vector<int> avarCluster;
    /// Chain -> the clusters of every step that reads its base points.
    std::vector<std::vector<int>> chainBaseClusters;
    /// Solver -> the clusters of every step that reads its driver points.
    /// What a ribbon's moved driver curve makes dirty, and nothing else.
    std::vector<std::vector<int>> solverPointsClusters;
    /// Dense revision id -> the clusters of every step of that revision, and
    /// the cluster of its RevisionStatic alone.
    std::vector<std::vector<int>> revisionClusters;
    std::vector<int> revisionStaticCluster;
    /// Native source -> the clusters of every constraint step that reads the
    /// frame the prologue read for it. What a moved stage transform under a
    /// constraint source makes dirty.
    std::vector<std::vector<int>> nativeSourceClusters;
    /// Geometry-domain delta base -> the cluster of the constraint step that
    /// measures against it.
    std::vector<std::vector<int>> deltaBaseClusters;
    /// Constraint-array entry -> the cluster of the constraint step that
    /// reads it.
    std::vector<std::vector<int>> constraintArrayClusters;
    /// Steps whose dirtiness depends on time or on a standing override.
    std::vector<int> varyingSteps, overrideSteps;
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
    /// One provider above a NATIVE Xformable source: the slot, and the
    /// versions of its base and final frames live where this commit runs.
    /// The deepest one whose points moved is the revision such a source
    /// rides (RigExecApplyRevisedAncestorDelta).
    struct AncestorRead {
        int slot = -1;
        uint32_t fin = 0;
        uint32_t base = 0;
    };
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

    // ---- where this commit's frames come from and go (§3.1) ---------------
    //
    // Every index below is into `fin` / `base`, and every one of them was
    // decided at Build: a read names the VERSION of a slot that was live at
    // this commit's point in the program, and a write names storage no other
    // step writes. Which is why a re-run of this commit cannot need an
    // earlier step re-run first to put a slot back the way it found it --
    // nothing ever puts it back, because nothing else writes there.
    /// Per candidate position: the version the delta is measured against and
    /// the version the write-back produces, in `fin` and (solver commits
    /// only) in `base`.
    std::vector<uint32_t> slotReads, slotWrites, slotBaseWrites;
    /// Per propagation pair: the descendant's version and its closest
    /// revised ancestor's, then the descendant's own new versions.
    std::vector<uint32_t> descendantReads, closestReads;
    std::vector<uint32_t> descendantWrites, descendantBaseWrites;
    /// Per write site, the version whose value stands where the site does
    /// not write one -- a candidate the batch published nothing for, a pair
    /// that was stepped over, a commit that passed through. Usually the read
    /// beside it; not always, because a slot can be both a candidate and a
    /// propagated descendant and the second write carries the first.
    std::vector<uint32_t> slotCarry, slotBaseCarry;
    std::vector<uint32_t> descendantCarry, descendantBaseCarry;
    /// A constraint's own reads: one per source, plus the world-up object,
    /// plus the target frame it solves FROM -- which a geometry-domain
    /// constraint reads without ever declaring the target a candidate.
    std::vector<uint32_t> sourceReads;
    uint32_t worldUpRead = 0;
    uint32_t targetRead = 0;
    /// Per target, the version live where this commit runs. Used to record
    /// the target of a commit that declares no candidate for it.
    std::vector<uint32_t> targetReads;
    /// The SingleChainIK bindings' reads, in the same two shapes a source's
    /// are: the version of a slot, and the ancestors of a native Xformable.
    uint32_t effectorRead = 0;
    std::vector<AncestorRead> effectorAncestors;
    std::vector<uint32_t> poleReads;
    std::vector<std::vector<AncestorRead>> poleAncestors;
    /// Scratch for the chain the solver is handed and the chain it returns.
    /// The first three are sized at Build; `ikSolved` cannot be, because the
    /// shared solve kernel RETURNS its chain by value and the assignment
    /// takes that buffer -- reserving here would only be discarded. It is
    /// the one per-frame allocation a SingleChainIK step makes, and closing
    /// it means giving the kernel an out-parameter form, which is a change
    /// to a kernel the dynamic path calls too.
    std::vector<RigExecPointFrame> ikChain, ikPrepared, ikRest, ikSolved;
    /// Whether this run's exit records the target's frame for a read phase.
    /// True for every transform-domain exit and for a geometry-domain
    /// constraint that never got as far as solving; false once a
    /// geometry-domain constraint has its sources, which is where the
    /// dynamic walk stops recording for one.
    bool recordAfter = true;
    /// And HOW MANY targets that exit records. The dynamic walk is not
    /// uniform about it: its early exits -- disabled, an unusable weight
    /// object, a bad envelope, a dormant one -- loop every target, and so
    /// does the SingleChainIK exit, whose targets ARE the solved chain; but
    /// its two late exits, unusable sources and the ordinary one, record
    /// targets[0] alone. Only a multi-target NON-IK constraint can tell the
    /// two apart, and no rig in the tree is one -- but the program must not
    /// invent a snapshot the reference path never published.
    bool recordEveryTarget = true;
    /// Parallel to `sources`, and empty for a source the walk holds a frame
    /// for: the ancestor slots of a native source, in increasing depth.
    std::vector<std::vector<AncestorRead>> sourceAncestors;
    std::vector<AncestorRead> worldUpAncestors;
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
    /// The rig's asset root: the parent of the RigExecRoot, which is the
    /// space every control frame the rig publishes is expressed in, and the
    /// prim a plain Xformable's transform is measured relative to. Two
    /// groups arrived at it independently -- the epilogue's curvenet-adjuster
    /// publication composes the ladder from the net's own prim up to here,
    /// and the xform-derived provider seed measures against it -- and it is
    /// ONE field because it is one prim: rigEvaluator.cpp's pose walk uses
    /// the same `_rigPath.GetParentPath()` for both.
    SdfPath assetRootPath;

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

    // ---- xform-derived provider slots -------------------------------------
    //
    // A plain Xformable a constraint targets has no rest chain and no avars:
    // its pose is whatever the stage says its transform is, measured relative
    // to the asset root. The run reads that in its PROLOGUE -- it is a stage
    // read, which no step may make -- and leaves the frame in the slot's
    // FIRST version, where the compose would have left one.
    /// The XformDerived slots, ascending, and the prim each one reads.
    std::vector<int> xformSlots;
    std::vector<UsdPrim> xformPrimsBySlot;
    /// The asset root every relative transform is measured against, which is
    /// the rig prim's parent (rigEvaluator.cpp's `assetRoot`).
    UsdPrim assetRoot;
    /// Per entry of `xformSlots`: the relative transform this run read, and
    /// the one the run before it read. The seed is a SOURCE (docs §7) -- it
    /// reads outside the program and always runs -- so what makes its
    /// readers dirty is the two compared by VALUE, never "the time moved".
    /// It is also what `providerBaseXforms` publishes.
    std::vector<GfMatrix4d> xformBase, lastXformBase;

    // ---- a constraint's own per-frame arrays ------------------------------
    //
    // inputs:sourceWeights and the parent offsets are read RAW off the
    // attribute at the frame's time -- no connection walk, no resolved
    // input, no interactive override -- because that is what the dynamic
    // walk does with them, and an operator input is not a rig input. The
    // read is USD, so it is the PROLOGUE's; the cardinality diagnostic
    // belongs to the constraint step, at the constraint's own place in the
    // walk, so what the prologue leaves behind is the line rather than the
    // pose it would have gone into.
    struct ConstraintArrays {
        UsdPrim prim;
        size_t sourceCount = 0;
        bool parentOffsets = false;
        std::vector<double> weights;
        std::vector<GfVec3d> translationOffsets, rotationOffsets;
        /// At most one line: the dynamic walk's buildSources stops at the
        /// first array whose cardinality is wrong.
        std::vector<std::string> diagnostics;
        bool ok = true;
        /// What the run before read. A source is compared by VALUE.
        std::vector<double> lastWeights;
        std::vector<GfVec3d> lastTranslationOffsets, lastRotationOffsets;
        std::vector<std::string> lastDiagnostics;
        bool lastOk = true;
        /// inputs:poleVectorWeights, read the same way for a SingleChainIK
        /// in object pole mode. Separate because it is read at a different
        /// point in the walk and owns its own diagnostic.
        bool readPole = false;
        size_t poleCount = 0;
        std::vector<double> poleWeights, lastPoleWeights;
        std::vector<std::string> poleDiagnostics, lastPoleDiagnostics;
        bool poleOk = true, lastPoleOk = true;
    };
    std::vector<ConstraintArrays> constraintArrays;

    // ---- native Xformable constraint sources ------------------------------
    //
    // A constraint source (or aim world-up object) that is neither a
    // RigExecControl nor a RigExecJoint is read off the stage, exactly as a
    // target is -- and then RIDDEN on the revision of the deepest provider
    // above it that the walk has already moved. The stage read is the
    // prologue's; the ride is the constraint step's, out of declared slots.
    struct NativeXformSource {
        SdfPath path;
        /// Every slot that is a STRICT namespace prefix of `path`, in
        /// increasing depth. Which of them is the revised one is a per-frame
        /// question the step asks; which of them could be is namespace
        /// topology and is settled here.
        std::vector<int> ancestorSlots;
    };
    std::vector<NativeXformSource> nativeSources;
    /// Per entry: the frame the prologue read, whether it read at all, and
    /// the pair the run before left -- the value comparison that dirties the
    /// constraint steps reading it, because a source is never dirtied by
    /// "the time moved".
    std::vector<RigExecPointFrame> nativeFrames, lastNativeFrames;
    std::vector<char> nativeFrameOk, lastNativeFrameOk;

    // ---- the provider ladder ----------------------------------------------
    //
    // The rest chain and the default-space ladder, resolved once at Build
    // and again on any frame that can move them. Every channel below is a
    // BOUND input rather than a folded constant, which is what lets an
    // animated, connected or chain-written rest bake and what lets a drag
    // on one be placed; when nothing varies and nothing is dragged the
    // Build-time answer stands and the frame path never looks at any of
    // them, which is the fast path the whole rig shape used to be.
    struct Ladder {
        RigExecBakedInput<GfMatrix4d> restSpace, defaultSpace, posedSpace;
        /// rest:tx/ty/tz/rx/ry/rz and default:tx/ty/tz/rx/ry/rz, in that
        /// order, which is the order RigExecBakedComposeAvars takes them in.
        RigExecBakedInput<double> restAvars[6];
        RigExecBakedInput<double> defaultAvars[6];
        RigExecBakedInput<TfToken> rotationOrder;
    };
    /// One per slot; an xform-derived slot's is unused and stays default.
    std::vector<Ladder> ladders;
    /// True when any channel of any ladder is read per frame. False is the
    /// ordinary rig, and it is what keeps the recompute off the frame path.
    bool ladderVarying = false;
    /// Every override index a ladder channel registered, sorted and unique.
    /// Consulted only while a drag stands, to decide whether that drag is
    /// one of THESE inputs.
    std::vector<int> ladderOverrides;
    /// True while the ladder still holds values recomputed for a drag; the
    /// same one-more-pass rule the avar table's `avarsDisturbed` states.
    bool ladderDisturbed = false;
    /// The slots whose ladder values moved this run, which is the skip hook
    /// a recomputed ladder owes the schedule: a provider whose rest or
    /// default space moved has to recompose, and so does everything reading
    /// the rest -> pose matrices below it.
    std::vector<int> ladderMovedSlots;
    /// True on a run whose prologue recomposed the ladder. A solver rest
    /// description is rebuilt from the rests, so it is rebuilt exactly on
    /// these runs -- including the one after a drag is released, which is
    /// why the flag is "did it run" and not "does it vary".
    bool ladderRecomputed = false;
    /// Per slot, whether the REST CHAIN reaching it can move within the
    /// epoch: its own rest channels, or any ancestor's. A solver measures
    /// its description from these, so it is the question a solver asks.
    std::vector<char> restChainVaries;

    /// A non-identity AUTHORED posed:space, per slot. Exec returns its frame
    /// directly and reads neither the avars nor the parent, so the compose
    /// takes the same branch: the flag is the "authored" test
    /// computations.cpp makes, re-taken on any frame the ladder is
    /// recomputed on, because an animated posed:space can cross identity.
    std::vector<char> posedAuthored;
    std::vector<GfMatrix4d> posedAuthoredM;
    std::vector<GfMatrix4d> restM;                     // asset-space rest
    std::vector<std::array<GfVec3d, 4>> restPts;
    std::vector<RigExecPointFrame> restFrames;
    std::vector<GfMatrix4d> selfD;                     // default:space
    std::vector<GfMatrix4d> parentDinv;                // parent default^-1
    std::vector<TfToken> rotOrder;
    /// The frame round trips exec performs between its ladder computations,
    /// which a deep chain drifts without. Per slot, in slot order, because
    /// a child reads its parent's.
    std::vector<GfMatrix4d> restRoundTrip, defaultRoundTrip;
    /// What the run before composed, for the move comparison above. Sized
    /// only when a ladder can actually move.
    std::vector<GfMatrix4d> lastRestM, lastSelfD, lastParentDinv,
        lastPosedAuthoredM;
    std::vector<char> lastPosedAuthored;
    std::vector<TfToken> lastRotOrder;
    /// Per provider slot: the scale avars are read and DISCARDED.
    ///
    /// A volume weight is a RigExecXformable whose point frame is composed
    /// with readScaleAvars = false, because a volume's shape is
    /// inputs:scaleX/Y/Z's alone and a transform scale left in the placement
    /// would deform the field without deforming the rigid guide a viewer
    /// draws. Exec expresses that by never binding avars:sx/sy/sz at all --
    /// so the discard has to happen at COMPOSE time and not by zeroing the
    /// captured constants, which an override or an animated channel would
    /// walk straight past.
    std::vector<char> noScaleAvars;

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
    /// The pose frames, in SSA form (§3.1).
    ///
    /// Slot i's FIRST version is entry i -- what the compose writes, and for
    /// an xform-derived slot what the seed leaves -- and every later writer
    /// of that slot gets storage of its own beyond the slot table: entry
    /// [N, ...) is one commit's answer for one slot, written by that commit
    /// and by nothing else. A reader binds at Build to the exact entry
    /// holding the version live at its own point in the program, so a read
    /// is one indirection and no version is ever overwritten by a later one.
    ///
    /// That is what makes "a skipped step keeps last run's values" true
    /// without a restore closure: the value a clean reader wants is still
    /// in the storage its writer left it in, however many times the SLOT
    /// was revised afterwards. A step that declares a write it does not
    /// perform carries the version it found into its own storage, so every
    /// version a reader can name is well formed whatever a run decided.
    ///
    /// Cost: Sigma|writes| frames, 96 bytes each -- a biped's walk is under
    /// 200 KB -- allocated once at Build and never resized in a run.
    std::vector<RigExecPointFrame> base, fin;
    /// Slot -> the entry holding its LAST version, which is what the
    /// matrices and the publication read. Sized N at Build.
    std::vector<uint32_t> finLast, baseLast;

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
        // ---- the rest description ------------------------------------
        //
        // Exec rebuilds every one of the rest members below from the
        // epoch's rests on EVERY evaluation, because it is a pure function
        // of them. The bake resolves it once, and RigExecBakedRefreshSolver
        // Rests resolves it again on any run whose prologue recomposed the
        // ladder -- which is what lets a solver measure against an animated,
        // chain-written or dragged rest instead of refusing the rig.
        /// Every provider slot whose rest this description folded in.
        std::vector<int> restSlots;
        /// (slot, element) for the two computations that remap by element:
        /// TwoBoneIk and SplineIk.
        std::vector<std::pair<int, int>> restRefs;
        /// True when any slot's whole rest CHAIN can move with time.
        bool restsVary = false;
        /// Every override index a rest channel of that chain registered, so
        /// a drag on one dirties this solver's step.
        std::vector<int> restOverrides;
        /// SplineIk rebuilds its rest curve from these two beside the rests.
        std::vector<double> splineRestWeights;
        RigExecSplineIkRestLength splineRestMode =
            RigExecSplineIkRestLength::Curve;
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
        /// An unsupported rigExec:rotationBlend, which is NOT a
        /// `degenerate`: the computation returns the surviving input before
        /// it ever looks at the token, so the rejection only bites when
        /// BOTH inputs are bound. It is checked where the computation
        /// checks it -- in the arm that has two aggregates in hand.
        bool blendRotationRejected = false;
        // SplineIk: the rest description is a pure function of epoch-constant
        // rests, so exec's per-evaluation rebuild bakes out.
        RigExecSplineIkRest splineRest;
        RigExecSplineIkParams splineParams;
        std::vector<std::array<GfVec3d, 4>> splineJointRests;
        size_t splineCount = 0;
        RigExecBakedInput<double> preserveVolume, midFollowWeight, roll, twist,
            minLengthRatio;
        bool splineParamsVary = false;
        // TwistDistribution. rigExec:start and rigExec:end ride on the shared
        // `root` and `end` members rather than a pair of their own: they are
        // provider slots read out of `fin` exactly as an IK control is, and
        // the version binding and the Solve step's read declaration are each
        // written once over those members. The two landmark sets are the
        // START and END rests, which exec substitutes with identity landmarks
        // for an unwired end -- a case the bake answers with `degenerate`
        // instead, because an unwired end also leaves the REQUIRED frame
        // input unbound and the computation publishes nothing at all.
        std::array<GfVec3d, 4> twistStartRest{}, twistEndRest{};
        std::vector<double> twistWeights;
        RigExecBakedInput<double> twistTurns;
        // Ribbon. The driver curve's points are the one solver input that
        // is scene data rather than rig state: the dynamic path reads the
        // attribute straight off the stage and hands exec both values as
        // packet overrides, honouring neither connections nor the resolved
        // inputs. So they are read the same way -- but in the PROLOGUE, into
        // the SolverPoints slot the Solve step declares, because a step body
        // may not touch USD.
        SdfPath ribbonPointsPath;
        UsdAttributeQuery ribbonPointsQuery;
        /// The bind-time value, read once at Default. An attribute carrying
        /// only time samples answers nothing there, which is how the dynamic
        /// path ends up with an empty rest and an empty aggregate.
        std::vector<GfVec3f> ribbonRestPoints;
        /// This run's live value and the last run's, compared by value so a
        /// moved driver curve dirties this solver's cluster and nothing
        /// else. Both empty for a curve that cannot vary with time, whose
        /// value is folded into ribbonConstantPoints instead.
        std::vector<GfVec3f> ribbonPoints, lastRibbonPoints;
        std::vector<GfVec3f> ribbonConstantPoints;
        bool ribbonPointsVarying = false;
        bool ribbonPointsDirty = false;
        RigExecBakedInput<int> ribbonSampleCount;
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
        /// The `fin` versions this solve reads: one per control, then the
        /// four named controls. Decided at Build like every other read
        /// (§3.1); -1 controls are bound to their own slot's seed and never
        /// read.
        std::vector<uint32_t> controlReads;
        uint32_t rootRead = 0, midRead = 0, endRead = 0, poleRead = 0;
    };
    std::vector<Solver> solvers;
    /// The solver slots no batch runs, in dependency order. Their Solve
    /// steps sit after the whole walk, because the frames the dynamic path's
    /// guide request hands them are the FINAL ones.
    std::vector<int> guideSolvers;
    std::map<SdfPath, int> solverIndex;
    std::vector<RigExecPointFrameArray> aggregates;

    // ---- constraints -------------------------------------------------------
    struct Constraint {
        SdfPath path;
        TfToken type;
        /// rigExec:weightObject, empty when the constraint binds none.
        ///
        /// Deliberately NOT an index into `weightObjects`: a constraint
        /// copies the ORACLE, and the oracle resolves the whole composition
        /// itself from the stage (see the head of bakedWeights.cpp).
        SdfPath weightObject;
        /// The one float the oracle resolves into, and the string it would
        /// report. Sized at Build so the step body allocates neither.
        std::vector<float> weightScratch;
        std::string weightError;
        int target = -1;
        /// Every target this constraint names, in compiled order, and which
        /// of them a read phase wants recorded after it. `target` is the
        /// first of them, which is the only one an ordinary constraint
        /// revises.
        std::vector<int> targetSlots;
        std::vector<char> snapshotTargets;
        /// Per source, in compiled order: the slot the walk holds a frame
        /// for, or -1; the entry in `nativeSources` read off the stage when
        /// it does not; and the path either way, which is what a diagnostic
        /// about the source names.
        std::vector<int> sources;
        std::vector<int> sourceNatives;
        std::vector<SdfPath> sourcePaths;
        /// This constraint's entry in `constraintArrays`, or -1 when it
        /// reads no per-frame array at all.
        int arrays = -1;
        RigExecBakedInput<bool> enabled;
        RigExecBakedInput<float> defaultWeight;
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
        /// The GEOMETRY domain: non-empty when rigExec:moves named
        /// <prim>.points, in which case this constraint revises no
        /// transform at all -- it measures a delta against the target's own
        /// authored transform and hands it to the Matrix revision the same
        /// mover contributes. `deltaBase` indexes the dense delta tables.
        SdfPath pointsTarget;
        SdfPath deltaBasePath;
        int deltaBase = -1;
        int worldUpObject = -1;
        int worldUpNative = -1;
        SdfPath worldUpPath;
        bool worldUpObjectNamed = false;
        /// True when any target wants a record, which is the one branch a
        /// rig with no read phase pays per constraint.
        bool snapshotAfter = false;

        // ---- SingleChainIK -------------------------------------------------
        //
        // The one multi-target built-in: it revises its whole inferred joint
        // chain atomically, so `targetSlots` IS the chain and the commit
        // declares every one of them.
        bool singleChainIk = false;
        RigExecSingleChainIkMode ikMode =
            RigExecSingleChainIkMode::RotatePlane;
        /// rigExec:poleVectorMode == "object", and rigExec:evaluationMode
        /// resolved against _IkUsesAnimatedTs. Both are uniform tokens over
        /// epoch-structural state, so both are settled at Build.
        bool poleModeObject = false;
        bool useAnimatedTs = false;
        /// The effector and the pole objects, as source references: a slot
        /// when the walk holds a frame, an entry in `nativeSources` when the
        /// stage does.
        int effector = -1, effectorNative = -1;
        SdfPath effectorPath;
        std::vector<int> poleObjects, poleObjectNatives;
        /// Read only in RotatePlane mode, which is where the dynamic walk
        /// reads them.
        RigExecBakedInput<GfVec3d> poleVector;
        RigExecBakedInput<double> twistDegrees;
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
    /// The RevisionOut domain is indexed by CHUNK, not by revision: a
    /// revision's chunks write disjoint vertex ranges of one buffer, and two
    /// steps that declared the same slot would be ordered against each other
    /// for a conflict they do not have. These are the dense chunk ids --
    /// [chunkBase, chunkBase + chunkCount) per revision, handed out revision
    /// by revision, so one chain's chunks are a contiguous range too.
    std::vector<int> revisionChunkBase, revisionChunkCount;
    std::vector<int> chainChunkBegin, chainChunkEnd;

    /// The partition of `steps` the parallel executor runs, chosen once at
    /// Build. Its grain comes from the cost model and the machine's
    /// concurrency, never from a measurement: a schedule that depended on
    /// what the box was doing at Build time would make "the same program
    /// produces the same answers at every grain" a claim nobody could test.
    RigExecBakedClustering clustering;
    /// One remaining-predecessor counter per cluster, allocated at Build so
    /// that the region allocates nothing. An array rather than a vector
    /// because std::atomic is neither copyable nor movable.
    std::unique_ptr<RigExecBakedClusterCounter[]> clusterCounters;

    // ---- cone re-execution -------------------------------------------------
    //
    // What a run may SKIP. The sets are Build's; everything below them is the
    // last run's answer, kept so that this run's sources can be compared with
    // it by VALUE. There is no "time changed" and no "overridden" predicate
    // deciding whether a source ran: sources always run and their outputs are
    // compared, which is what makes an override on a routed prim, a released
    // drag, a cleared topology cache and a moved keyframe all reach the graph
    // through one test (§7).
    RigExecBakedCones cones;
    /// The clusters this run decided to run. Every step of a cluster outside
    /// it keeps its slots, its diagnostics and its structural counters.
    RigExecBakedClusterSet closed;
    /// The avar table as the last run left it, for the per-provider compare.
    std::vector<double> lastAvars;
    /// The property-chain results as the last run left them.
    std::map<SdfPath, VtValue> lastPropertyResults;
    /// The override flags as the last run left them, so that the run AFTER a
    /// drag is released re-runs what the drag was holding.
    std::vector<char> lastOverridden;
    /// Whether each chain's base read at all last run.
    std::vector<char> lastHaveBase;
    UsdTimeCode lastTime = UsdTimeCode::Default();
    bool everRan = false;
    /// Bumped by the evaluator for a notice that does NOT invalidate the
    /// program -- a value edit on an input the frame path re-reads, which is
    /// IsInvalidatedBy's documented gap. One run of everything answers it.
    /// Set/ClearInteractiveOverrides do NOT bump it: an override is a source
    /// value like any other and is compared like one (§7).
    uint64_t programStamp = 0;
    uint64_t lastProgramStamp = 0;
    /// How many clusters the last run ran, and how many there are, for the
    /// schedule run report.
    size_t lastClosedClusters = 0;

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
    /// The order to fill the published maps in: the publication list's
    /// indices, sorted by path.
    ///
    /// A biped frame publishes about 850 keys into five ordered maps and
    /// every one of them used to be a search from the root. Filled in path
    /// order instead, each key belongs immediately past the one before it,
    /// so a hint at the map's end makes the insertion one comparison. The
    /// order is a PERMUTATION and not the list itself, because the lists
    /// arrive from the evaluator in binding order -- measured on the biped,
    /// neither the joints nor the controls come out in path order, so the
    /// obvious "just walk the list" is wrong on the one rig this was written
    /// for. Everything the walk ORDER is observable through (the degenerate
    /// frame diagnostics) stays in the list's own order; only the map
    /// filling moves.
    std::vector<int> jointPublishOrder;
    std::vector<int> controlPublishOrder;
    std::vector<int> solverPublishOrder;
    /// Whether the sorted orders above are STRICTLY ascending -- which they
    /// are unless a list names one path twice. A repeated path would make an
    /// emplace keep the first value where the assignment it replaces kept
    /// the last, so such a list is published the old way -- and the sort
    /// that built the permutation is stable, so "the last" is still the last
    /// of the two in the publication list, as it was before the permutation
    /// existed.
    bool jointPathsAscending = false;
    bool controlPathsAscending = false;
    bool solverArraysAscending = false;
    /// Under RIGEXEC_BAKED_STEP_TIMING, the sum over the frames watched of
    /// what each third of a frame cost, in microseconds, and how many frames
    /// are in the sums. Untouched -- and unread -- when the variable is off.
    double timedPrologueUs = 0;
    double timedRegionUs = 0;
    double timedEpilogueUs = 0;
    size_t timedFrames = 0;
    /// Set while the cone verifier's second, whole-program pass runs --
    /// always, not only when a step timing asked, so that the flag means
    /// one thing -- and read by both executors. That pass is the
    /// instrument, not the frame: letting it into the per-step accumulators
    /// would make every step of a verified frame report two runs, and the
    /// table would describe a frame nobody asked for.
    bool measurementSuspended = false;

    /// Per joint, whether this run's final frame earned a published matrix.
    /// Written by the diagnostic pass, read by the fill pass; sized at
    /// Build, so the epilogue allocates nothing.
    std::vector<char> jointMatrixPublished;

    // ---- geometry ----------------------------------------------------------
    // One entry per revision of one chain, in chain order. The packet is
    // assembled by the SAME RigExecAssembleParameters the dynamic path calls
    // and the kernel is the same shared kernel, so only the plumbing around
    // them is baked.
    /// One contiguous vertex range of one revision, and the influences the
    /// vertices in it reference.
    ///
    /// A chunk is SPECULATIVE: it starts as soon as its own influences are
    /// final, without waiting for the revision's validity fold, and writes
    /// its range of the revision's own output buffer. The fuse is what
    /// decides afterwards whether the revision applied at all; a chunk whose
    /// work is not wanted is discarded rather than undone.
    struct GeomChunk {
        /// The range, half-open, in the vertex order the mesh is authored
        /// in. Vertex order is never permuted: chunk boundaries are the only
        /// thing the partition chooses.
        int begin = 0, end = 0;
        /// The influence positions the range's vertices index, ascending.
        /// An upper bound is safe and a missing entry is not, so this is the
        /// UNION over the range, computed from the same indices the kernel
        /// reads.
        std::vector<int> key;
        /// The revision's influence table as this chunk sees it: identity
        /// everywhere, with the key's entries copied from the matrix slots
        /// each run. Linear blend skinning reads an entry only through an
        /// index of one of its own vertices, so the identities are never
        /// read -- they are there so the table has the shape the layout
        /// describes rather than to contribute a value.
        std::vector<GfMatrix4d> transforms;
        /// The same table narrowed for the SIMD path, and split for the
        /// dual-quaternion one. Filled entry by entry beside `transforms`,
        /// so the chunk pays for |key| matrices rather than for all of them.
        std::vector<float> rows;
        std::vector<RigExecScaledDualQuat> palette;
        /// Whether the chunk's key matrices moved since it last ran, decided
        /// while they are copied in. A chunk whose own influences stand
        /// still, over points that stand still, already holds its answer.
        bool keyChanged = false;
        /// The range of the output buffer holds this run's value for it.
        /// Sticky across a run the chunk sat out, which is what makes the
        /// skip above sound.
        bool ok = false;
    };

    /// One blend channel's stage handles, resolved once at Build.
    ///
    /// The channels themselves are per-frame reads -- a channel weight is
    /// what an animator drags -- so nothing here is a VALUE. What is captured
    /// is which attribute to ask, in the order the accumulation is defined
    /// over: `binding.blendInputs` is canonically sorted at compile and the
    /// samples of one channel are stable-sorted by activation every frame.
    /// Float addition is not associative, so an order that differs from the
    /// dynamic walk's is a different last bit.
    struct GeomBlendChannel {
        /// `inputs:weight` on the RigExecBlendInput prim. Invalid when the
        /// prim or the attribute is absent, which reads as the channel's
        /// default exactly as the dynamic walk's invalid handle does.
        UsdAttribute weight;
        struct Sample {
            /// `rigExec:activation` on the RigExecBlendSample prim.
            UsdAttribute activation;
            /// The sample's target-shape points, and the path a declared
            /// read phase looks that array up under.
            UsdAttribute points;
            SdfPath pointsPath;
            RigExecReadPhase phase;
        };
        std::vector<Sample> samples;
    };

    struct GeomRevision {
        SdfPath moverPath;
        SdfPath target;
        UsdPrim moverPrim;
        RigExecRevisionOp op = RigExecRevisionOp::Skin;
        RigExecRevisionBinding binding;
        /// The chain whose published points are this curvenet mover's POSED
        /// net, as an index into `chains`, or -1. E._chainOrder runs a net's
        /// own chain before any mover that reads it, so the value is this
        /// run's by the time the revision is assembled -- and the static
        /// step declares the chain's ChainPoints slot, which is what says so
        /// to the scheduler.
        int curvenetChain = -1;
        /// The curvenet adjuster's second output: one fully adjusted frame
        /// per adjustment, in RigExecCurvenetAdjustmentPaths order.
        ///
        /// Persistent, because the revision node keeps its own as MUTABLE
        /// member state that survives a Compute it did not run -- a revision
        /// whose inputs stood still still publishes the frames it last
        /// produced, and the published map is one of the seven the
        /// comparator checks.
        std::vector<GfMatrix4d> controlFrames;
        /// The Profile Mover bind, resolved in the PROLOGUE.
        ///
        /// RigExecCurvenetBindCache has no locking at all and reports one
        /// diagnostic per bind, so it belongs to serial code; a null pointer
        /// here is a remembered failed bind and not "not asked yet", which
        /// `curvenetBindResolved` says.
        std::shared_ptr<const RigExecProfileMoverBinding> curvenetBind;
        bool curvenetBindResolved = false;
        /// This revision's blend channels, in `binding.blendInputs` order.
        /// Empty for every operation but a blend shape.
        std::vector<GeomBlendChannel> blendChannels;
        std::vector<int> influenceSlots;
        int transformSlot = -1;
        /// The geometry-domain constraint whose delta IS this revision's
        /// transform, as an index into the dense delta tables, or -1. Joined
        /// on the mover path at Build, because that is the key the dynamic
        /// walk's own hand-off uses.
        int constraintDelta = -1;
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
        /// A DERIVED revision's `auxPoints` -- the chain's final points, up
        /// to 315KB of them -- held here as the handle the chain published
        /// rather than copied into `lastParameters` above, which is left
        /// with that field empty. The comparison agrees with the elementwise
        /// one by construction: VtArray is copy-on-write and its operator==
        /// short-circuits on a shared buffer, so this is the same test with
        /// the identity case taken first. Empty, and never read, for every
        /// other revision.
        VtVec3fArray lastAuxPoints;
        RigExecMoverStatus lastStatus;
        bool ran = false;
        /// A read phase named this revision as the point in the chain it
        /// wants the target's points from, so the chain records them after
        /// it. Decided at bake out of the evaluator's _snapshotPoints.
        bool snapshotAfter = false;
        /// This revision LOOKS the run's phased-read store up, so its static
        /// step declares every step before it as a read. Two things can make
        /// it true and the second is easy to miss: a declared input phase
        /// (`binding.phases`), and a blend sample whose target shape carries
        /// one -- that lookup is made directly by the channel gather rather
        /// than through the revision's overlay, so the overlay's own
        /// predicate does not cover it.
        bool readsSnapshots = false;
        /// This node is new to the rig's geometry state and its creation has
        /// not been reported yet. Cleared by AdoptGeometryStateFrom for a
        /// node the outgoing program already held, which is the same
        /// bookkeeping the dynamic walk does when it reconnects a surviving
        /// VdfNetwork node instead of adding one.
        bool created = true;

        // ---- what the steps of a revision hand each other -----------------
        /// The influence table the PACKET carries, which for a skin revision
        /// is identity and nothing else: the packet is assembled before the
        /// matrices are folded, precisely so a chunk can start on its own
        /// joints without waiting for every joint of the rig. The assembler
        /// checks the table's shape and its elements' finiteness, which
        /// identities pass; the real table's check is the fold's, and the
        /// fuse ANDs it in where the assembler's would have landed. Written
        /// once, at Build.
        std::vector<GfMatrix4d> packetInfluences;
        /// InfluenceFold writes these; every operation but a skin assembles
        /// against them. Sized at Build, never resized in the region.
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
        ///
        /// `status` is the status OF THE PACKET, which for a skin revision
        /// is not the whole answer: the packet carries identities where the
        /// influence matrices would be, so a table the dynamic path's
        /// assembler would have failed leaves this saying "ok" and
        /// `influencesValid` below saying otherwise. The fuse ANDs the two
        /// and publishes `moverFailed` where the dynamic path publishes it;
        /// read `influencesValid` beside this one, never this one alone.
        RigExecMoverParameters parameters;
        RigExecMoverStatus status;
        /// Whether this revision has to run at all: chain dirty so far, or
        /// it never ran, or its packet or status moved. A VALUE comparison,
        /// never a dirtiness flag -- a control dragged back to where it
        /// started must not count as executed.
        bool executed = false;
        // ---- the vertex partition ------------------------------------------
        /// The revision's vertex chunks, in vertex order, covering
        /// [0, pointCount) exactly once. Always at least one; more only for
        /// a skin revision whose layout the epoch fixed (see
        /// RigExecBakedPartitionRevision).
        std::vector<GeomChunk> chunks;
        /// Where this revision's chunks start in the RevisionOut domain.
        int chunkBase = 0;
        /// The layout the partition was cut from, so a frame can tell in
        /// O(1) whether the keys still describe the vertices. The handle is
        /// the identity the skin topology cache preserves across a notice
        /// that touched no layout, so an unchanged binding never re-cuts and
        /// a changed one always does.
        std::shared_ptr<const RigExecSkinTopology> partitionTopology;
        int partitionElementSize = 0;
        size_t partitionIndexCount = 0;
        size_t partitionPointCount = 0;
        /// The keys are used -- more than one chunk, so an influence outside
        /// a chunk's key is an influence that chunk will not see.
        bool chunked = false;
        /// What the cut decision saw, recorded so it can be asserted and
        /// printed rather than re-derived. `partitionCandidates` is how many
        /// ranges the vertex target and the cap produced, and the two levels
        /// are the lowest and highest level at which one of those ranges has
        /// every joint it reads. Cutting pays only when they differ -- a
        /// range that cannot start before the whole revision could is a
        /// serial loop where a self-parallelising kernel call used to be --
        /// so `chunked` is `partitionCandidates > 1 && readyMin < readyMax`,
        /// unless RIGEXEC_BAKED_CHUNK_ALWAYS asked for the cut regardless.
        size_t partitionCandidates = 0;
        int partitionReadyMin = 0;
        int partitionReadyMax = 0;

        // ---- what RevisionStatic decides for the whole array ---------------
        /// The layout half of the skin kernel's validation (the matrix half
        /// is `influencesValid` below).
        bool layoutUsable = false;
        /// The envelope, resolved ONCE at the full point count because it
        /// resolves atomically, and the predicate that says the blend is the
        /// identity and the resolution therefore dead.
        std::vector<float> envelope;
        bool envelopeOk = false;
        bool fullStrength = false;
        /// The points this revision is applied to, which is the size every
        /// chunk writes within.
        size_t precedingCount = 0;
        /// The packet or the status moved, or the revision never ran. The
        /// chain's sticky bit and the influence table's compare are ORed in
        /// by the fuse; this is only what the static half saw.
        bool staticDirty = false;
        /// `inputs:defaultWeight` as the stage reads it, and the value the
        /// last run read.
        ///
        /// The one number the FUSE would otherwise have to read off the
        /// stage, for the one diagnostic it can emit. A source reads it
        /// instead, and compares it, so that the fuse is a pure function of
        /// its declared slots and a run that skips the fuse is a run in
        /// which this did not move either (§7).
        float defaultWeight = 0, lastDefaultWeight = 0;
        /// The partition no longer describes the layout the packet carries,
        /// so the keys cannot be trusted and the fuse runs the revision
        /// whole rather than a chunk deforming a vertex against an identity.
        bool partitionStale = false;

        // ---- the weight object this revision binds -------------------------
        /// rigExec:weightObject as an index into `weightObjects`, or -1.
        /// The packet itself is shared -- one per object per frame, however
        /// many movers bind it -- so what a revision holds is the index.
        int weightObject = -1;
        /// Whether the weight object weights the MOVER (one declared target,
        /// and it is this mover) rather than the points. Decided at Build
        /// because it is a relationship read, and the relationship is epoch
        /// state; the condition is the dynamic path's, verbatim.
        bool weightOperationDomain = false;
        /// The property the published field says it weights: the mover for
        /// an operation-domain object, the chain's target otherwise.
        SdfPath weightFieldTarget;
        /// The field is measured against the points ENTERING this revision,
        /// so the shared packet is not this revision's answer: it patches a
        /// copy of its own, exactly the fields the dynamic path patches, and
        /// assembles against that.
        bool weightCurrentPhase = false;
        RigExecWeightPacket currentPhasePacket;
        /// The field this revision published this run, and whether it
        /// published one at all. Written by RevisionStatic -- which is where
        /// the dynamic path publishes it, from the packet the mover is about
        /// to consume -- and drained by the epilogue in chain order.
        std::vector<float> weightField;
        bool weightFieldPublished = false;

        // ---- what InfluenceFold decides for the whole array ----------------
        /// Every influence matrix finite and affine. For a skin revision the
        /// packet cannot answer this -- it is assembled before the matrices
        /// are folded -- so the fuse ANDs this in where the dynamic path's
        /// assembler would have failed the packet.
        bool influencesValid = false;
        /// The table moved since the last run, which is the half of the
        /// executed decision the packet comparison no longer carries.
        bool influencesChanged = false;
        /// The table in the two forms the kernels want it in, written by
        /// the fold for every skin revision: an unchunked one's single
        /// chunk skins against them, and so does the fuse when it has to run
        /// a revision whole. A chunked revision's chunks keep their own
        /// beside these and never read them -- but the fuse may only READ
        /// this slot, so the fold writes them regardless.
        std::vector<float> rows;
        std::vector<RigExecScaledDualQuat> palette;
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


    /// What each geometry-domain constraint measured: the delta between its
    /// solved frame and its target's own authored transform, which is the
    /// matrix the Matrix revision that same mover contributes rides. Dense
    /// and indexed by `Constraint::deltaBase`, because it is written by a
    /// STEP -- a shared std::map is an allocation and a race, whatever the
    /// slots say -- and read by the InfluenceFold of the revision whose
    /// mover produced it, which is the same hand-off the dynamic walk's own
    /// constraintDeltas map performs, in the same direction.
    ///
    /// It is a SLOT and not a per-run delta: what a skipped constraint step
    /// leaves here is what it would have measured again, so it is kept
    /// across runs rather than cleared at the head of one.
    std::vector<GfMatrix4d> deltaValues;
    std::vector<char> deltaPresent;
    /// Per geometry-domain constraint, the target's own transform relative
    /// to the asset root -- a STAGE read, made by the prologue, and a source
    /// the cone compares by value. It is the AUTHORED transform even when
    /// the target is a RigExecJoint: RigExecXformable inherits Xformable and
    /// the dynamic walk measures against the stage unconditionally.
    std::vector<SdfPath> deltaBasePaths;
    std::vector<GfMatrix4d> deltaBaseMatrix, lastDeltaBaseMatrix;
    std::vector<char> deltaBaseOk, lastDeltaBaseOk;

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
        TfToken representation, rangePolicy;
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
        /// The combine's own rigExec:weightTarget, read for its SIZE alone.
        std::vector<UsdAttribute> combineTargetPoints;
        /// Roughly how many elements this object's packet carries, for the
        /// cost model alone. Measured once at Build off the same arrays the
        /// chain point counts are measured from; a packet whose field turns
        /// out to be a different size costs the schedule a bin, never an
        /// answer.
        size_t costElements = 1;

        // ---- the volumetric three ------------------------------------------
        /// The provider slot the volume is posed into, and therefore the
        /// BASE frame its field is placed against -- base and not final,
        /// because the evaluator overrides every seeded provider's
        /// computePointFrame with its base frame before the authoritative
        /// snapshot, and a mover's packet is what that snapshot carries.
        int providerSlot = -1;
        RigExecBakedInput<float> falloffMin, falloffMax;
        RigExecBakedInput<float> scaleX, scaleY, scaleZ;
        RigExecBakedInput<float> extentU, extentV;
        TfToken planeAxis, planeBounds;
        /// The points-bearing relationships, as the attributes their
        /// AUTHORED targets name, in authored order.
        ///
        /// Authored and not canonicalized: exec reaches these through
        /// Relationship().TargetedObjects<GfVec3f>(computeValue), which
        /// computes a value on each targeted OBJECT, so a target naming a
        /// prim rather than its .points contributes nothing there however
        /// readily the CPU oracle infers one. A mover copies exec, so this
        /// list holds exactly the properties exec would have reached. No
        /// shipped rig authors the prim form -- every weightTarget in the
        /// tree and in the fixtures names `.points` -- so the two readings
        /// agree everywhere today, and where they would not, this is the
        /// one that is a mover's answer.
        std::vector<UsdAttribute> targetPoints, samplePoints, curvePoints;
        /// The epoch's resampled falloff remap, copied from falloffLuts.
        std::vector<float> falloffCurve;

        // ---- RigExecCurvenetWeight -----------------------------------------
        //
        // The one weight object whose field is not a formula over a few
        // floats: it is the solution of a factorized system over the CUT
        // mesh, so the packet needs the BIND that factorization lives in.
        // The exec computation keeps a process-wide LRU of those behind a
        // mutex, and a step body may take no lock -- so the program keeps
        // one binding per object, in the object, written by that object's
        // OWN step and read by nothing else. It is the same cache with a
        // capacity of one entry per weight object, which is all a program
        // can use: a weight object has exactly one layout per frame.
        /// The five array relationships, as the attributes their authored
        /// targets name, read exactly the way `targetPoints` is.
        std::vector<UsdAttribute> curvenetMeshPoints, curvenetPoints;
        std::vector<UsdAttribute> curvenetCounts, curvenetIndices;
        std::vector<UsdAttribute> curvenetSplines;
        /// inputs:weights and rigExec:autoSmooth, which exec reads per frame
        /// off the prim itself. Arrays, so they go through the generation's
        /// resolved inputs rather than through a RigExecBakedInput.
        UsdAttribute curvenetWeights, curvenetAutoSmooth;
        TfToken curvenetBasis;
        RigExecBakedInput<int> curvenetSamples;
        RigExecBakedInput<float> curvenetUnreached;
        /// This object's binding, and the layout it was cut for. Compared by
        /// VALUE, like every other decision in the frame path: an equal
        /// layout is the same cut, and a rig whose mesh or net moves per
        /// frame re-cuts on both paths alike.
        std::shared_ptr<RigExecCurvenetWeightBinding> curvenetBinding;
        std::vector<GfVec3f> boundMesh, boundNet;
        std::vector<int> boundCounts, boundIndices, boundSplines, boundSmooth;
        int boundSamples = -1;
        bool bound = false;
    };
    std::vector<WeightObject> weightObjects;
    /// Path to index in weightObjects. A NEGATIVE entry is an object whose
    /// composition walk is under way (the bake is depth first and enters the
    /// table on the way out), so meeting one is a cycle.
    std::map<SdfPath, int> weightIndex;
    /// The weight oracle, as the evaluator's own RigExecRigEvaluator::
    /// _ResolveWeights.
    ///
    /// A pointer to a member function rather than a call, because only
    /// bakedProgram.cpp is the evaluator's friend and the constraint that
    /// needs it lives in bakedPose.cpp. It is the SAME function the dynamic
    /// constraint path calls with the same arguments, which is what makes
    /// the answer and the error string identical by construction rather
    /// than by review -- a constraint's envelope is one of the two places
    /// the dynamic path does not go through exec at all.
    std::function<bool(const SdfPath &, size_t, UsdTimeCode,
                       std::vector<float> *, std::string *,
                       const std::vector<GfVec3f> *)> resolveWeights;

    /// Where each volume weight object is placed, as the walk left it.
    ///
    /// A POINTER to the evaluator's own _volumeWeightMatrices, because the
    /// oracle reads that member and nothing else: a program-owned copy would
    /// be a second map the oracle never looks at. Written by the one
    /// VolumePlacements step, which declares it, so no two steps can be
    /// inside it at once.
    std::map<SdfPath, GfMatrix4d> *volumeWeightMatrices = nullptr;
    /// RigExecRigEvaluator::_UpdateVolumePlacements, bound at Build.
    ///
    /// The body has a subtlety worth not restating: a frame no matrix can be
    /// built from leaves whatever the failed decomposition wrote, over an
    /// identity seed, rather than the identity. Calling the evaluator's own
    /// is how the program cannot drift from that.
    std::function<void(const std::function<
        bool(const SdfPath &, RigExecPointFrame *)> &)> updateVolumePlacements;
    /// Weight objects whose field is measured against the points AS THEY
    /// STAND at the revision that binds them, rather than the authored base
    /// (the evaluator's _currentPhaseWeights). A combine is in here when
    /// anything inside it is.
    std::set<SdfPath> currentPhaseWeights;

    /// Every volumetric weight object's baked falloff remap, by prim path.
    ///
    /// Copied out of the evaluator's own _falloffLutOverrides at Build, not
    /// recomputed: a falloff curve is epoch-structural (exec has no accessor
    /// for an attribute's spline, so the curve is resampled once at Compile
    /// and replayed unchanged), and these are literally the bytes exec
    /// receives.
    std::map<SdfPath, std::vector<float>> falloffLuts;
    /// This frame's packet per weight object -- the WeightPacket slot
    /// domain's storage. Sized at Build and never resized in a run, like
    /// every other slot: a consumer holds a pointer into it for the whole
    /// region.
    std::vector<RigExecWeightPacket> weightPackets;

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
    /// Properties a step reads through the generation's resolved inputs but
    /// that EXEC reads as a typed scalar input of its own -- a curvenet
    /// weight's inputs:weights and rigExec:autoSmooth are the case, and the
    /// only one: they are arrays, and exec's computeWeightPacket declares
    /// them AttributeValue<float>/<int>, so an override carrying a VtArray
    /// is rejected there by type and the dynamic path answers from the
    /// authored value. The program would see it, so placing it would make
    /// the program answer a question the dynamic path refuses -- a
    /// divergence with pose.valid on both sides. Unplaceable on purpose,
    /// which sends the generation down the path that decides.
    std::set<SdfPath> execTypedArrayInputs;
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
    /// Per target, whether a read phase asked for its frame as of this
    /// constraint (the evaluator's _snapshotPoints membership). A
    /// SingleChainIK names its whole chain here, and the dynamic walk
    /// records every one of them.
    std::vector<char> snapshotTargets;
    /// SingleChainIK: the inferred joint chain, first joint through end, and
    /// the two bindings it resolves beside its sources.
    SdfPathVector ikChain;
    SdfPath effector, effectorXform;
    SdfPathVector poleObjects, poleObjectXforms;
    /// _IkUsesAnimatedTs over the chain, answered where the evaluator's
    /// private state is visible. "autoDetect" resolves against it; the other
    /// two modes ignore it.
    bool ikUsesAnimatedTs = false;
    /// One source path per binding, in the compiled order, and beside it the
    /// plain Xformable the binding reads off the stage when the walk holds
    /// no frame for it (the compiled _FrameSourceBinding::xformPath). Empty
    /// for a RigExec provider, whose frame the walk always has.
    SdfPathVector sources;
    SdfPathVector sourceXforms;
    /// Non-empty when rigExec:moves named <prim>.points: the constraint
    /// writes the GEOMETRY domain and revises no transform.
    SdfPath pointsTarget;
    /// Empty when the aim constraint named no world-up object.
    SdfPath worldUpObject;
    SdfPath worldUpXform;
    /// rigExec:weightObject, empty when the constraint binds none.
    SdfPath weightObject;
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
    /// Each RigExecRibbon's resolved driver-points attribute, restated from
    /// the evaluator's own compiled map (which only Build's file can name).
    std::map<SdfPath, SdfPath> ribbonDriverPoints;
    /// The aggregate solvers no batch runs, in dependency order: a guide-only
    /// RigExecBlendPointFrames may read another one's aggregate, and the
    /// order is what makes the reader run second.
    std::vector<SdfPath> guideOnlySolvers;
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
    /// providers and the plain Xformables a constraint targets, and both
    /// kinds are LIVE -- the prologue seeds an xform-derived slot from the
    /// stage and a constraint revises it like any other target. So a
    /// `SlotOf(x) < 0` refusal reads "is no provider slot at all", NOT "is
    /// not a pose provider", and there is no longer a refusal standing
    /// between the two meanings. A caller that means "publishes
    /// computeRestFrame" or "is composed from avars" must test
    /// slotKind[slot] == PoseSeed itself, as the solver rest binding does.
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

/// Appends the placement step, if the epoch has any volume weight at all,
/// then one WeightPacket step per weight object in the table's dependency
/// order -- all of it between the pose half and the geometry half.
void RigExecBakedBuildWeightSteps(RigExecBakedProgramImpl *program);

/// Runs one WeightPacket step, under the same rule as the other two.
void RigExecBakedRunWeightStep(RigExecBakedProgramImpl *program,
                               RigExecBakedStep *step, UsdTimeCode time);

/// Every per-frame input one weight object's step reads.
void RigExecBakedNoteWeightInputs(
    const RigExecBakedProgramImpl::WeightObject &weight,
    RigExecBakedStep *step);

/// Resets the per-run DELTAS a geometry step would have written, for a run
/// that skipped it (§7).
///
/// A revision's state is two kinds of thing: values that outlive a run it
/// sat out -- the influence table, the packet, the points, the status -- and
/// flags that say how this run's values differ from the last run's. The
/// values are exactly what a skipped step is keeping; the flags are
/// statements about a comparison that was never made, and a later step
/// reading one would be told a change happened this run when it happened
/// during the last one. A skipped step compared nothing and therefore found
/// nothing, which is what this writes.
void RigExecBakedSkipGeometryStep(RigExecBakedProgramImpl *program,
                                  RigExecBakedStep *step);

/// Resolves every provider's rest chain and default-space ladder from the
/// bound channels of RigExecBakedProgramImpl::ladders, in slot order.
///
/// ONE definition, called from Build (once, at the capture time) and from
/// the prologue (on any frame a channel can have moved). computations.cpp
/// resolves the same eight computations per provider per evaluation; the
/// frame round trips exec performs between them are reproduced, not
/// simplified away, because a deep chain drifts without them.
///
/// \p trackMoves fills RigExecBakedProgramImpl::ladderMovedSlots by
/// comparing what it composes against what the run before composed, which
/// is what dirties the compose of a provider whose rest moved. Build passes
/// false: there is no run before, and the first run dirties everything.
void RigExecBakedComposeLadder(RigExecBakedProgramImpl *program,
                               UsdTimeCode time, bool trackMoves);

/// The pose half of the prologue: the bound inputs, once per run.
void RigExecBakedRunInputs(RigExecBakedProgramImpl *program, UsdTimeCode time);

/// The solver half of the prologue: every ribbon's live driver-curve points,
/// read off the stage and compared with the last run's.
///
/// A source in the sense of §7 -- it reads outside the program and its
/// comparison is what dirties the solvers that read it -- but a prologue
/// pass rather than a step, for the same reason the bound inputs are one:
/// it takes the stage's locks, and a step body may not.
void RigExecBakedRunSolverSources(RigExecBakedProgramImpl *program,
                                  UsdTimeCode time);

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
/// and each derived target's, in chain order, plus the control frames a
/// curvenet adjuster publishes -- which need \p time, because the ladder
/// from the net to the asset root is composed at the evaluated time.
void RigExecBakedPublishGeometry(RigExecBakedProgramImpl *program,
                                 UsdTimeCode time, RigExecRigPose *pose);

/// Cuts \p revision's vertices into chunks, from \p indices and
/// \p elementSize.
///
/// Contiguous ranges of RIGEXEC_BAKED_CHUNK_VERTS vertices, capped at
/// RIGEXEC_BAKED_MAX_CHUNKS (the range grows to meet the cap); each range's
/// key is the union of its vertices' influence positions; adjacent ranges
/// merge while they stay under the vertex cap and one key contains the
/// other, because a range that waits for a superset of another's joints is
/// not waiting any longer for holding both. Vertex order is never permuted.
///
/// \p chunkCount, when positive, is the number of chunks the caller must
/// end up with -- the number of STEPS a revision is made of is fixed at
/// Build, so a re-cut against a layout that moved redistributes the same
/// number of ranges rather than changing the program.
void RigExecBakedPartitionRevision(
    RigExecBakedProgramImpl::GeomRevision *revision,
    const int *indices, size_t indexCount, int elementSize, int chunkCount);

/// The partition statistics of every skin revision, for the schedule report.
///
/// Per revision: how many chunks, how many vertices each holds, the key
/// sizes, how many chunks depend on half the influences or more, and each
/// chunk's influence positions -- which is what makes "the arm vertices no
/// longer wait for the leg constraints" a measured claim per asset rather
/// than a design intention.
std::string RigExecBakedGeometryReport(const RigExecBakedProgramImpl &program);

/// Records \p input against \p step: what a frame can move it with.
///
/// Three answers, and each is a different way a value can arrive: it is a
/// function of time (a keyframe), it resolves through the generation's
/// resolved inputs every frame (a property chain writes it), or an
/// interactive override can be placed on it. Anything else was folded at
/// bake and cannot move without a rebuild.
///
/// It is here rather than beside one domain's step builders because all
/// three domains declare inputs and a domain that declared them its own way
/// would be a step the cone cannot dirty.
template <class T>
inline void
RigExecBakedNoteInput(const RigExecBakedInput<T> &input,
                      RigExecBakedStep *step)
{
    step->varyingInputs = step->varyingInputs || input.varying;
    step->resolvedInputReads =
        step->resolvedInputReads || bool(input.resolvedAttr);
    if (input.overrideIndex >= 0) {
        step->overrideInputs.push_back(input.overrideIndex);
    }
}

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
///
/// \p object is NOT const, and only a curvenet weight uses that: it carries
/// its own bind, which is state this frame may replace and which nothing
/// but this object's own step ever touches.
RigExecWeightPacket RigExecBakedWeightPacket(
    const RigExecBakedProgramImpl &program,
    RigExecBakedProgramImpl::WeightObject *object,
    const std::vector<RigExecWeightPacket> &packets, UsdTimeCode time);

/// Records, per pose step, which of its baked inputs a run can move.
///
/// A Solve and a Constraint read their own parameters off the stage every
/// frame -- weights, offsets, axis masks, world-up vectors -- so neither is
/// a pure function of its declared slots. Build walks those inputs once here
/// and leaves each step saying whether any of them varies with time and
/// which override indices reach it, which is what lets the dirty set name
/// the steps a keyframe or a standing drag can have moved instead of
/// re-running the walk for either.
void RigExecBakedDeclareInputDependencies(RigExecBakedProgramImpl *program);

/// Shadows the whole of a run's mutable state, for RIGEXEC_BAKED_VERIFY_CONES.
///
/// Capture/Restore are what let one frame run twice from one starting point:
/// the cone run's answer is captured, the starting point is restored, the
/// whole program is re-run, and Compare reports every slot, counter and
/// diagnostic the two runs disagree about. Nothing here is on a production
/// path -- the state is large and copying it is the point.
///
/// WHAT IT DELIBERATELY DOES NOT COMPARE, and why each one is scratch rather
/// than an answer. An instrument that agrees about state it never looked at
/// is worse than no instrument, so this list is meant to be exhaustive
/// against the mutable fields of RigExecBakedProgramImpl, RigExecBakedStep,
/// GeomChain, GeomRevision and GeomChunk; a field added to any of those
/// belongs either in Compare or in this list.
///
///  * everything Build wrote and no run touches -- the slot tables, the
///    rests, the input bindings, the step graph, the clustering, the cones.
///    A run that changed one of those would be a run editing the program.
///  * what the PROLOGUE writes: `propertyResults`, `avarsDisturbed`,
///    `overridden`/`anyOverridden`/`folded`, `curvenetBindings`, and the
///    geometry prologue's `haveBase`/`baseDirty`/`lastBase`/`created`/
///    `scheduleDirty`/`topology`/the partition. The prologue runs ONCE per
///    generation, before either pass, so both passes see one value of each
///    by construction. Several are captured and restored anyway, because
///    they are cheap and putting the second pass back on exactly the first
///    pass's footing is what the mode is for.
///  * the cone bookkeeping itself -- `closed`, `lastAvars`, `lastOverridden`,
///    `lastPropertyResults`, `lastHaveBase`, `lastTime`, `everRan`,
///    `lastProgramStamp`. The second pass is FORCED, so its closure differs
///    from the first's on purpose; comparing them would report the mode
///    rather than the program. The run statistics that observers read are
///    put back by RigExecBakedRunStatistics instead.
///  * each chain's `spare`, the other half of the published double buffer.
///    A pass that publishes swaps it with `result`; a pass that skips the
///    publication does not. The two therefore hold DIFFERENT generations'
///    arrays after runs that agree exactly about the published one, which is
///    `result` -- the buffer is storage, and only what `result` names in it
///    is an answer.
///  * the snapshot stores: `RigExecBakedProgramImpl::runSnapshots` and each
///    step's `snapshots`. A program in which any step records one sets
///    `phasedReads`, and `phasedReads` forces every run whole (§7), so a
///    program that fills them has no cone for this mode to check. Each
///    revision's `revisionInputs` overlay is captured and restored for the
///    same reason turned around -- it costs nothing and it keeps the second
///    pass starting from exactly the first's state -- but not compared.
///  * the RUN STATISTICS, which are restored rather than compared, because
///    the second pass is forced and so writes different ones by
///    construction: `lastClosedClusters`, the clustering's `lastRunTimed`
///    and each `RigExecBakedCluster`'s `readyUs`/`startUs`/`endUs`, and each
///    step's `startUs`/`endUs`. RigExecBakedRunStatistics takes all of them
///    before the second pass and puts them back after it, so that every
///    observer of the frame -- the run report, the profiler trace,
///    GetClustersRunLastGeneration() -- describes the one run that published
///    a pose.
///  * `clusterCounters` and per-step `measuredUs`/`measuredRuns`: the
///    parallel executor's arrival counters and the calibrator's running
///    averages. The counters are stored afresh at the head of every parallel
///    run and mean nothing between runs; the averages are a fit over frames,
///    which an opt-in calibration reads and this mode does not.
struct RigExecBakedRunShadow {
    /// Copies everything a step reads or writes out of \p program.
    void Capture(const RigExecBakedProgramImpl &program);
    /// Puts it back, so that a second run starts where the first one did.
    void Restore(RigExecBakedProgramImpl *program) const;
    /// Appends one line per disagreement between this shadow and
    /// \p program's current state, and returns how many there were.
    size_t Compare(const RigExecBakedProgramImpl &program,
                   std::vector<std::string> *differences) const;

    struct StepState {
        std::vector<std::string> diagnostics;
        RigExecBakedStepCounters counters;
        bool bail = false;
    };
    struct ChunkState {
        std::vector<GfMatrix4d> transforms;
        std::vector<float> rows;
        std::vector<RigExecScaledDualQuat> palette;
        bool keyChanged = false, ok = false;
    };
    struct RevisionState {
        std::vector<GfVec3f> output;
        std::vector<GfMatrix4d> packetInfluences, influences;
        RigExecResolvedInputs revisionInputs;
        std::vector<float> rows, envelope;
        std::vector<RigExecScaledDualQuat> palette;
        std::vector<ChunkState> chunks;
        RigExecMoverParameters parameters, lastParameters;
        VtVec3fArray lastAuxPoints;
        RigExecMoverStatus status, lastStatus;
        TfToken resultStatus;
        GfMatrix4d transform{1.0};
        size_t precedingCount = 0;
        int currentSource = -1;
        float defaultWeight = 0, lastDefaultWeight = 0;
        bool haveTransform = false, ran = false, executed = false;
        bool influencesValid = false, influencesChanged = false;
        bool staticDirty = false, partitionStale = false;
        bool layoutUsable = false, envelopeOk = false, fullStrength = false;
        std::vector<float> weightField;
        RigExecWeightPacket currentPhasePacket;
        bool weightFieldPublished = false;
    };
    struct DerivedState {
        RevisionState revision;
        VtVec3fArray result, spare, lastBase;
        bool haveResult = false, haveBase = false, baseDirty = false;
    };
    struct ChainState {
        std::vector<RevisionState> revisions;
        std::vector<DerivedState> derived;
        VtVec3fArray lastBase, result, spare;
        bool haveResult = false, haveBase = false, baseDirty = false;
    };
    struct SolverState {
        std::vector<RigExecPointFrame> outFrames;
        std::vector<char> outPresent;
        std::vector<SdfPath> fallbackJoints;
    };
    struct CommitState {
        std::vector<char> present, deltaOk;
        std::vector<RigExecPointFrame> frames, staged;
        std::vector<GfMatrix4d> deltas;
        std::vector<uint8_t> outcome;
        std::vector<RigExecConstraintSource> sources;
        bool abandoned = true;
    };

    std::vector<double> avars;
    std::vector<RigExecWeightPacket> weightPackets;
    std::vector<GfMatrix4d> posedM, finalMatrix, baseMatrix;
    std::vector<RigExecPointFrame> base, fin;
    std::vector<RigExecPointFrameArray> aggregates;
    std::vector<SolverState> solvers;
    std::vector<CommitState> commits;
    std::vector<ChainState> chains;
    std::vector<StepState> steps;
    std::vector<GfMatrix4d> deltaValues;
    std::vector<char> deltaPresent;
    bool avarsDisturbed = false;
};

/// The run statistics of the generation that produced the pose (§8.3).
///
/// RIGEXEC_BAKED_VERIFY_CONES runs the frame a second time, forced, and that
/// pass writes the same bookkeeping the first one did: how many clusters the
/// closure held, and the intervals the run report and the profiler trace
/// print. But a verification pass is not a generation -- it publishes
/// nothing -- so what an observer asks the program afterwards has to be the
/// cone run's answer. Without this, GetClustersRunLastGeneration() reports
/// every cluster whenever the verifier is on, and the assertions that prove
/// a cone skipped anything hold or fail on whether the verifier is on rather
/// than on the cone.
///
/// The per-STEP intervals are here for the same reason and not a weaker one:
/// the epilogue replays them into the profiler after the second pass, so
/// leaving them would put a trace of the verification pass beside a cluster
/// table of the cone run and let a reader believe the two describe one run.
struct RigExecBakedRunStatistics {
    /// Takes the statistics \p program currently holds.
    explicit RigExecBakedRunStatistics(
        const RigExecBakedProgramImpl &program);
    /// Puts them back.
    void Restore(RigExecBakedProgramImpl *program) const;

    struct ClusterTimes {
        uint64_t readyUs = 0, startUs = 0, endUs = 0;
    };
    struct StepTimes {
        uint64_t startUs = 0, endUs = 0;
    };
    std::vector<ClusterTimes> clusters;
    std::vector<StepTimes> steps;
    size_t closedClusters = 0;
    bool timed = false;
};

/// Whether RIGEXEC_BAKED_VERIFY_CONES asks a run to prove its cone.
bool RigExecBakedVerifyConesRequested();

}  // namespace rigExec

#endif  // RIGEXEC_BAKED_PROGRAM_IMPL_H
