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
    RigExecChainSnapshots *chainSnapshots = nullptr;
    RigExecSkinTopologyCache *skinTopologies = nullptr;
    RigExecCurvenetBindCache *curvenetBindings = nullptr;
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
    // _poseSeedFrames is keyed by SdfPath, whose order IS namespace DFS
    // pre-order, so a provider's parent always has a lower slot than it does
    // and one forward pass composes the whole hierarchy.
    std::vector<SdfPath> paths;
    std::map<SdfPath, int> index;
    std::vector<int> parent;

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

    // ---- rest->pose matrices, resolved on demand once per frame ------------
    //
    // What computeMatrix publishes, over dense slots: the whole
    // AuthoritativeSnapshot request is a re-derivation of values the walk
    // already holds. Program-owned rather than a per-frame allocation, because
    // BOTH halves of a frame read them and the geometry half must see exactly
    // the matrix the pose half published.
    std::vector<GfMatrix4d> finalMatrix, baseMatrix;
    std::vector<char> haveFinal, haveBase;

    /// Empties the on-demand matrix tables for a frame over \p slots slots.
    void ResetMatrices(size_t slots) {
        finalMatrix.assign(slots, GfMatrix4d(1.0));
        baseMatrix.assign(slots, GfMatrix4d(1.0));
        haveFinal.assign(slots, 0);
        haveBase.assign(slots, 0);
    }

    /// \p slot's rest -> final matrix, computed on its first use this frame.
    const GfMatrix4d &FinalMatrixOf(int slot) {
        if (!haveFinal[size_t(slot)]) {
            if (RigExecBakedUsable(restFrames[slot]) &&
                RigExecBakedUsable(fin[slot])) {
                RigExecPointsToMatrix(restPts[slot], fin[slot].points,
                                      &finalMatrix[size_t(slot)]);
            }
            haveFinal[size_t(slot)] = 1;
        }
        return finalMatrix[size_t(slot)];
    }

    /// \p slot's rest -> base matrix, computed on its first use this frame.
    const GfMatrix4d &BaseMatrixOf(int slot) {
        if (!haveBase[size_t(slot)]) {
            if (RigExecBakedUsable(restFrames[slot]) &&
                RigExecBakedUsable(base[slot])) {
                RigExecPointsToMatrix(restPts[slot], base[slot].points,
                                      &baseMatrix[size_t(slot)]);
            }
            haveBase[size_t(slot)] = 1;
        }
        return baseMatrix[size_t(slot)];
    }

    // ---- solvers -----------------------------------------------------------
    struct Solver {
        SdfPath path;
        TfToken type;
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
    };
    std::vector<Constraint> constraints;

    // ---- the walk ----------------------------------------------------------
    struct Step {
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
    std::vector<Step> steps;

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
        /// This node is new to the rig's geometry state and its creation has
        /// not been reported yet. Cleared by AdoptGeometryStateFrom for a
        /// node the outgoing program already held, which is the same
        /// bookkeeping the dynamic walk does when it reconnects a surviving
        /// VdfNetwork node instead of adding one.
        bool created = true;
    };
    struct GeomChain {
        SdfPath target;
        UsdAttributeQuery baseQuery;
        std::vector<GeomRevision> revisions;
        VtVec3fArray lastBase;
        VtVec3fArray result;
        bool haveResult = false;
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
            bool haveResult = false;
        };
        std::vector<Derived> derived;
    };
    std::vector<GeomChain> chains;

    std::vector<GfMatrix4d> influenceScratch;

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

/// Runs the pose half of one frame: inputs, compose, the solver/constraint
/// walk, the rest->pose matrices and the frame publication.
///
/// Returns false having published diagnostics when the frame cannot complete,
/// exactly where today's straight-line Run returns false.
bool RigExecBakedRunPose(RigExecBakedProgramImpl *program, UsdTimeCode time,
                         RigExecRigPose *pose);

/// Runs the geometry half of one frame: every chain's revisions, the derived
/// maintenance that reads their final points, and the mover-graph accounting.
void RigExecBakedRunGeometry(RigExecBakedProgramImpl *program,
                             UsdTimeCode time, RigExecRigPose *pose);

}  // namespace rigExec

#endif  // RIGEXEC_BAKED_PROGRAM_IMPL_H
