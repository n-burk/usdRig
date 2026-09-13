//
// The baked program: build and run. See bakedProgram.h for what it is and
// why it is a request rather than a promise.
//
// Everything here is a second expression of semantics that live elsewhere --
// the provider compose and the default-space ladder in computations.cpp, the
// pose walk in rigEvaluator.cpp, the geometry revision in moverGraph.cpp --
// so every piece is written against the one it mirrors and nothing is
// re-derived from first principles. Where a kernel can simply be CALLED
// instead of mirrored (the constraint operators, the skin kernel, the
// extent/normal kernels, the packet assembler, the property chains) it is,
// because a shared call cannot drift and a copy can.
//
#include "bakedProgram.h"

#include "frameExtraction.h"
#include "moverGraph.h"
#include "rigEvaluator.h"
#include "types.h"

#include "rigExecMath/avarScale.h"
#include "rigExecMath/envelope.h"
#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/pointFrame.h"
#include "rigExecMath/simdKernels.h"
#include "rigExecMath/solvers.h"
#include "rigExecMath/splineIk.h"

#include "pxr/base/gf/math.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/attributeQuery.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/metrics.h"
#include "pxr/usd/usdGeom/xformCache.h"
#include "pxr/usd/usdGeom/xformable.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace rigExec {

namespace {

// Mirrors computations.cpp _ComposeAvars (the provider compose exec runs).
// The program must produce the same numbers as that callback, so the two are
// written the same way rather than algebraically simplified.
GfMatrix4d
_ComposeAvars(double tx, double ty, double tz, double sx, double sy, double sz,
              double rx, double ry, double rz, double rspin,
              const TfToken &order)
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

RigExecEulerOrder
_ParseEulerOrder(const TfToken &t)
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
GfMatrix4d
_RoundTrip(const GfMatrix4d &m)
{
    GfMatrix4d out(1.0);
    RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                          RigExecFrameFromMatrix(m).points, &out);
    return out;
}

// rigEvaluator.cpp _IsUsableConstraintFrame, which gates every commit in the
// pose walk this program reproduces.
bool
_Usable(const RigExecPointFrame &frame)
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
struct _Input {
    T constant{};
    UsdAttributeQuery query;  ///< set when USD alone answers, per frame
    UsdAttribute resolvedAttr;  ///< set when a property chain is on the walk
    UsdAttribute head;        ///< where the walk starts, for an override
    bool varying = false;
    /// Index into _Impl::overridden, or -1 when the input was never
    /// registered (the rest ladder, which is folded rather than read).
    int overrideIndex = -1;
};

// The attribute GetAttribute would end up reading, plus why it cannot be
// captured. Mirrors that function's loop exactly.
template <class T>
void
_ClassifyInput(const UsdAttribute &attribute, UsdTimeCode time,
               const std::set<SdfPath> &chainTargets,
               bool *viaChain, bool *varying, UsdAttribute *selected,
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
        // _AnimatedOrConnected applies further down, for the same reason.
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
_Input<T>
_BindInput(const UsdPrim &prim, const char *name, T fallback,
           UsdTimeCode time, const std::set<SdfPath> &chainTargets,
           SdfPathVector *walk = nullptr)
{
    _Input<T> input;
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
    _ClassifyInput<T>(attribute, time, chainTargets, &viaChain, &varying,
                      &selected, walk);
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
T
_Read(const _Input<T> &input, const RigExecResolvedInputs &resolved,
      UsdTimeCode time, const std::vector<char> *overridden = nullptr)
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
bool
_IsVarying(const _Input<T> &input)
{
    return input.varying;
}

// A plain attribute read with no time and no resolution walk, for the
// uniform tokens the dynamic path reads exactly this way
// (rigExec:rotationOrder, rigExec:worldUpType, rigExec:aimAxis, ...).
TfToken
_ReadToken(const UsdPrim &prim, const char *name, const char *fallback)
{
    TfToken value(fallback);
    if (prim) {
        if (const UsdAttribute a = prim.GetAttribute(TfToken(name))) {
            TfToken read;
            if (a.Get(&read) && !read.IsEmpty()) {
                value = read;
            }
        }
    }
    return value;
}

SdfPathVector
_Targets(const UsdPrim &prim, const char *name)
{
    SdfPathVector targets;
    if (prim) {
        if (const UsdRelationship rel = prim.GetRelationship(TfToken(name))) {
            rel.GetTargets(&targets);
        }
    }
    return targets;
}

const char *const _kAvarNames[11] = {
    "avars:tx", "avars:ty", "avars:tz", "avars:sx", "avars:sy", "avars:sz",
    "avars:rx", "avars:ry", "avars:rz", "avars:rspin",
    "avars:unitScaleFactor"};
const double _kAvarDefaults[11] = {0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1};

// The constraint operators the program expresses. A type outside this set
// makes the epoch unbakeable rather than silently passing through.
bool
_IsBakedConstraintType(const TfToken &type)
{
    return type == "RigExecPositionConstraint" ||
           type == "RigExecRotationConstraint" ||
           type == "RigExecScaleConstraint" ||
           type == "RigExecParentConstraint" ||
           type == "RigExecAimConstraint";
}

bool
_IsBakedSolverType(const TfToken &type)
{
    return type == "RigExecFkChain" || type == "RigExecTwoBoneIk" ||
           type == "RigExecBlendPointFrames" || type == "RigExecSplineIk";
}

const char *
_OpName(RigExecRevisionOp op)
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

}  // namespace

// ---------------------------------------------------------------------------
// The program.
// ---------------------------------------------------------------------------

struct RigExecBakedProgram::_Impl {
    RigExecRigEvaluator *evaluator = nullptr;
    UsdStageRefPtr stage;

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
    struct _AvarBinding {
        size_t slot = 0;
        _Input<double> input;
    };
    std::vector<_AvarBinding> avarBindings;            // the varying ones only
    /// The rest, kept only so a drag can reach one. Walked per frame while an
    /// interactive override stands and never otherwise.
    std::vector<_AvarBinding> avarConstantBindings;
    size_t boundInputs = 0;
    size_t varyingInputs = 0;

    // ---- per-frame working state (dense) ----------------------------------
    std::vector<double> avars;
    std::vector<GfMatrix4d> posedM;
    std::vector<RigExecPointFrame> base, fin;

    // ---- solvers -----------------------------------------------------------
    struct _Solver {
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
        _Input<double> bend, upperOffset, lowerOffset;
        _Input<float> stretch, softness;
        double upperLengthBase = 0, lowerLengthBase = 0;
        // BlendPointFrames
        int inA = -1, inB = -1;
        _Input<float> blendWeight;
        RigExecScaleBlend scaleMode = RigExecScaleBlend::Log;
        // SplineIk: the rest description is a pure function of epoch-constant
        // rests, so exec's per-evaluation rebuild bakes out.
        RigExecSplineIkRest splineRest;
        RigExecSplineIkParams splineParams;
        std::vector<std::array<GfVec3d, 4>> splineJointRests;
        size_t splineCount = 0;
        _Input<double> preserveVolume, midFollowWeight, roll, twist,
            minLengthRatio;
        bool splineParamsVary = false;
        // (providerSlot, element) pairs this solver writes
        std::vector<std::pair<int, int>> outputs;
    };
    std::vector<_Solver> solvers;
    std::map<SdfPath, int> solverIndex;
    std::vector<RigExecPointFrameArray> aggregates;

    // ---- constraints -------------------------------------------------------
    struct _Constraint {
        SdfPath path;
        TfToken type;
        int target = -1;
        std::vector<int> sources;
        _Input<bool> enabled;
        _Input<float> defaultWeight;
        std::vector<_Input<float>> sourceWeights;      // authored table only
        size_t authoredSourceWeights = 0;
        std::vector<GfVec3d> translationOffsets, rotationOffsets;
        bool offsetsVary = false;
        _Input<GfVec3d> offset;                        // position/rotation/scale
        _Input<bool> affectX, affectY, affectZ;        // the operator's group
        _Input<bool> tX, tY, tZ, rX, rY, rZ, sX, sY, sZ;  // parent
        RigExecEulerOrder order = RigExecEulerOrder::XYZ;
        // aim
        _Input<GfVec3d> aimVector, upVector, rotationOffset, worldUpVector;
        GfVec3d aimAxisFallback{1, 0, 0};
        bool aimVectorAuthored = false;
        bool preserveInputUp = false;
        TfToken worldUpType;
        GfVec3d sceneUp{0, 1, 0};
        int worldUpObject = -1;
        bool worldUpObjectNamed = false;
    };
    std::vector<_Constraint> constraints;

    // ---- the walk ----------------------------------------------------------
    struct _Step {
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
    std::vector<_Step> steps;

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
    struct _GeomRevision {
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
    struct _GeomChain {
        SdfPath target;
        UsdAttributeQuery baseQuery;
        std::vector<_GeomRevision> revisions;
        VtVec3fArray lastBase;
        VtVec3fArray result;
        bool haveResult = false;
        /// This chain's schedule has to be built and that has not been
        /// reported yet. The dynamic path rebuilds a chain's schedule when
        /// the identity sequence of its revisions changed, and not for a
        /// packet that merely holds different numbers.
        bool scheduleDirty = true;
        // Derived maintenance reads this chain's FINAL points.
        struct _Derived {
            SdfPath target;
            UsdAttributeQuery baseQuery;
            _GeomRevision revision;
            VtVec3fArray lastBase;
            VtVec3fArray result;
            bool haveResult = false;
        };
        std::vector<_Derived> derived;
    };
    std::vector<_GeomChain> chains;

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
    // One flag per registered input; see _Read. Kept as a dense vector so the
    // frame path costs an index rather than a map lookup per read.
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
    void Register(_Input<T> *input, const SdfPathVector &walk) {
        if (!input->head) {
            return;
        }
        input->overrideIndex = int(overridden.size());
        overridden.push_back(0);
        // EVERY attribute on the resolution walk, not only the head one.
        // _ClassifyInput followed an authored connection chain and may have
        // captured the value several hops upstream; an interactive override
        // standing on one of those hops is an override on this input, and
        // GetAttribute -- which is what the flag makes _Read use -- consults
        // the generation's resolved values at every step of the same walk.
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

namespace {

// A numeric probe time. Selection along a connection chain must not depend on
// which time code the caller asks for, and Default cannot read an attribute
// that carries only time samples, so stability is checked against both.
UsdTimeCode
_ProbeTime(const UsdStageRefPtr &stage)
{
    return stage && stage->HasAuthoredTimeCodeRange()
               ? UsdTimeCode(stage->GetStartTimeCode())
               : UsdTimeCode(0.0);
}

// True when \p attribute or anything it resolves through carries animation,
// which is what makes the value unbakeable. A single time sample counts: it
// reads back at a numeric time but not at Default, so capturing it would make
// the answer depend on which time code the caller passed.
bool
_AnimatedOrConnected(const UsdAttribute &attribute)
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

bool
_AttributeIsIdentity(const UsdPrim &prim, const char *name, bool *connected)
{
    const UsdAttribute a = prim.GetAttribute(TfToken(name));
    if (!a) {
        return true;
    }
    SdfPathVector connections;
    if (a.HasAuthoredConnections() && a.GetConnections(&connections) &&
        !connections.empty()) {
        *connected = true;
        return false;
    }
    GfMatrix4d value(1.0);
    return !a.Get(&value) || value == GfMatrix4d(1.0);
}

}  // namespace

// ---------------------------------------------------------------------------
// Bakeability.
// ---------------------------------------------------------------------------

bool
RigExecBakedProgram::IsBakeable(const RigExecRigEvaluator &evaluator,
                               std::vector<std::string> *reasons)
{
    std::vector<std::string> local;
    std::vector<std::string> &out = reasons ? *reasons : local;
    const size_t before = out.size();
    const RigExecRigEvaluator &E = evaluator;
    auto say = [&out](const std::string &what, const SdfPath &where) {
        out.push_back(what + ": " + where.GetString());
    };

    if (!E._compiled) {
        out.push_back("the rig is not compiled");
        return false;
    }

    // ---- pose providers ---------------------------------------------------
    for (const auto &[path, taps] : E._connectedPoseTaps) {
        say("connected-space provider", path);
    }
    // _interveningXformProviders is a CANDIDATE list: it holds every provider
    // whose parent is not its anchor, which on an ordinary rig means a
    // grouping Scope with no transform at all. The dynamic path composes
    // nothing unless one of them resolves to a non-identity X(P), so that --
    // not the candidacy -- is what the program cannot express.
    if (!E._interveningXformProviders.empty()) {
        const UsdPrim assetRoot =
            E._stage->GetPrimAtPath(E._rigPath.GetParentPath());
        UsdGeomXformCache cache(UsdTimeCode::Default());
        for (const SdfPath &path : E._interveningXformProviders) {
            const auto anchorIt = E._poseProviderAnchors.find(path);
            const SdfPath anchorPath = anchorIt == E._poseProviderAnchors.end()
                                           ? SdfPath()
                                           : anchorIt->second;
            const UsdPrim anchor = anchorPath.IsEmpty()
                                       ? assetRoot
                                       : E._stage->GetPrimAtPath(anchorPath);
            const UsdPrim parent =
                E._stage->GetPrimAtPath(path.GetParentPath());
            if (!parent || !anchor || parent == anchor) {
                continue;
            }
            bool resets = false;
            const GfMatrix4d x =
                cache.ComputeRelativeTransform(parent, anchor, &resets);
            if (resets || x != GfMatrix4d(1.0)) {
                say("intervening Xform above provider", path);
                continue;
            }
            // Identity today is not identity for the epoch if it animates.
            for (UsdPrim walk = parent; walk && walk != anchor;
                 walk = walk.GetParent()) {
                const UsdGeomXformable xformable(walk);
                if (xformable && xformable.TransformMightBeTimeVarying()) {
                    say("animated Xform above provider", path);
                    break;
                }
            }
        }
    }
    for (const SdfPath &path : E._xformDerivedProviders) {
        say("constraint target is a plain Xformable", path);
    }
    for (const auto &[path, tap] : E._volumeWeightMatrixTaps) {
        say("volume weight object", path);
    }
    for (const SdfPath &path : E._currentPhaseWeights) {
        say("current-phase volume weight", path);
    }
    for (const auto &[path, movers] : E._snapshotPoints) {
        say("read-phase snapshot required on", path);
    }
    for (const auto &[path, points] : E._ribbonDriverPoints) {
        say("ribbon driver curve", path);
    }

    const UsdTimeCode probe = _ProbeTime(E._stage);
    // A property chain RECOMPUTES its target every generation, so a value
    // read once at bake time is not that target's value -- it is whatever
    // the previous generation happened to leave behind. Inputs the program
    // re-reads per frame resolve through the chain and are fine; the ones
    // resolved once into the rest chain and the default-space ladder are
    // not, so a chain aimed at one of those refuses the bake.
    std::set<SdfPath> chainTargets;
    for (const auto &[target, revisions] : E._propertyChains) {
        chainTargets.insert(target);
    }
    for (const auto &[path, tap] : E._poseSeedFrames) {
        const UsdPrim prim = E._stage->GetPrimAtPath(path);
        if (!prim) {
            say("pose provider has no prim", path);
            continue;
        }
        const TfToken type = prim.GetTypeName();
        if (type != "RigExecJoint" && type != "RigExecControl") {
            say("provider type not baked (" + type.GetString() + ")", path);
        }
        // A space expression the compose cannot express: the program builds
        // the default-space ladder from rest + default avars and follows the
        // namespace parent, which is exactly what exec does only while these
        // stay unauthored and unconnected.
        for (const char *name : {"posed:space", "parent:space",
                                 "parent:defaultSpace", "avars:defaultSpace",
                                 "posed:defaultSpace"}) {
            bool connected = false;
            if (!_AttributeIsIdentity(prim, name, &connected)) {
                say(std::string(connected ? "connected " : "authored ") +
                        name + " on provider",
                    path);
            }
            if (chainTargets.count(path.AppendProperty(TfToken(name)))) {
                say(std::string("property chain writes ") + name +
                        " on provider",
                    path);
            }
        }
        // The ladder is resolved once, so anything feeding it must be still.
        for (const char *name : {"rest:space", "rest:tx", "rest:ty", "rest:tz",
                                 "rest:rx", "rest:ry", "rest:rz",
                                 "default:space", "default:tx", "default:ty",
                                 "default:tz", "default:rx", "default:ry",
                                 "default:rz", "avars:rotationOrder"}) {
            if (const UsdAttribute a = prim.GetAttribute(TfToken(name))) {
                if (_AnimatedOrConnected(a)) {
                    say(std::string("animated or connected ") + name +
                            " on provider",
                        path);
                }
            }
            if (chainTargets.count(path.AppendProperty(TfToken(name)))) {
                say(std::string("property chain writes ") + name +
                        " on provider",
                    path);
            }
        }
        (void)probe;
    }
    for (const SdfPath &joint : E._jointPaths) {
        if (!E._poseSeedFrames.count(joint)) {
            say("joint is not a seeded pose provider", joint);
        }
    }
    for (const SdfPath &control : E._controlPaths) {
        if (!E._poseSeedFrames.count(control)) {
            say("control is not a seeded pose provider", control);
        }
    }

    // ---- solvers ----------------------------------------------------------
    std::set<SdfPath> batched;
    for (const auto &batch : E._solverBatches) {
        for (const auto &[solverPath, tap] : batch.solvers) {
            batched.insert(solverPath);
            const UsdPrim prim = E._stage->GetPrimAtPath(solverPath);
            const TfToken type = prim ? prim.GetTypeName() : TfToken();
            if (!_IsBakedSolverType(type)) {
                say("solver type not baked (" + type.GetString() + ")",
                    solverPath);
                continue;
            }
            if (type == "RigExecTwoBoneIk") {
                // Bone lengths are MEASURED from the rests of the joints the
                // solver names; without all three there is nothing to bake.
                std::array<bool, 3> seen{false, false, false};
                const SdfPathVector joints =
                    _Targets(prim, "rigExec:joints");
                VtIntArray elements;
                if (const UsdAttribute a = prim.GetAttribute(
                        TfToken("rigExec:jointElements"))) {
                    a.Get(&elements);
                }
                for (size_t k = 0; k < joints.size(); ++k) {
                    const int element =
                        elements.size() == joints.size() ? elements[k] : int(k);
                    if (element >= 0 && element < 3) seen[element] = true;
                }
                if (!(seen[0] && seen[1] && seen[2])) {
                    say("TwoBoneIk does not bind three joint rests",
                        solverPath);
                }
            }
        }
    }
    for (const auto &[solverPath, tap] : E._solverArrayTaps) {
        if (!batched.count(solverPath)) {
            say("solver publishes guides but is in no batch", solverPath);
        }
    }

    // ---- constraints -------------------------------------------------------
    for (const auto &constraint : E._frameConstraints) {
        if (!constraint.pointsTarget.IsEmpty()) {
            say("geometry-domain constraint", constraint.moverPath);
        }
        if (constraint.schemaType == "RigExecSingleChainIkConstraint") {
            say("SingleChainIK constraint", constraint.moverPath);
            continue;
        }
        if (!_IsBakedConstraintType(constraint.schemaType)) {
            say("constraint type not baked (" +
                    constraint.schemaType.GetString() + ")",
                constraint.moverPath);
            continue;
        }
        if (!constraint.weightObject.IsEmpty()) {
            say("weight object on constraint", constraint.moverPath);
        }
        if (constraint.targets.size() != 1) {
            say("constraint does not write exactly one target",
                constraint.moverPath);
            continue;
        }
        if (!E._poseSeedFrames.count(constraint.targets[0])) {
            say("constraint target is not a seeded pose provider",
                constraint.targets[0]);
        }
        for (const auto &source : constraint.sources) {
            if (!source.xformPath.IsEmpty()) {
                say("native Xformable constraint source", source.sourcePath);
            }
        }
        if (!constraint.worldUpObject.xformPath.IsEmpty()) {
            say("native Xformable world-up object",
                constraint.worldUpObject.sourcePath);
        }
    }

    // ---- geometry ----------------------------------------------------------
    auto checkRevision = [&](const RigExecRigEvaluator::_GraphRevision &r,
                             bool derived) {
        const bool supported =
            derived ? (r.op == RigExecRevisionOp::RecomputeExtent ||
                       r.op == RigExecRevisionOp::RecomputeNormals)
                    : (r.op == RigExecRevisionOp::Skin ||
                       r.op == RigExecRevisionOp::Matrix);
        if (!supported) {
            say(std::string("mover operation not baked (") + _OpName(r.op) +
                    ")",
                r.moverPath);
            return;
        }
        if (!r.binding.weightObject.IsEmpty()) {
            say("weight object on mover", r.moverPath);
        }
        if (!r.binding.blendInputs.empty()) {
            say("blend shape inputs on mover", r.moverPath);
        }
        if (!r.binding.phases.empty()) {
            say("read phase on a mover input", r.moverPath);
        }
        if (!r.binding.curvenetPoints.IsEmpty() ||
            !r.binding.curvenet.IsEmpty()) {
            say("curvenet on mover", r.moverPath);
        }
        if (r.driverFramesTap >= 0) {
            say("driver frames on mover", r.moverPath);
        }
        if (r.op == RigExecRevisionOp::Skin) {
            const UsdPrim prim = E._stage->GetPrimAtPath(r.moverPath);
            // The layout is captured once; the kernel then does an O(1) shape
            // check per frame instead of re-reading three arrays.
            for (const char *name : {"rigExec:jointIndices",
                                     "rigExec:jointWeights",
                                     "rigExec:elementSize",
                                     "rigExec:skinningMethod"}) {
                if (const UsdAttribute a = prim.GetAttribute(TfToken(name))) {
                    if (_AnimatedOrConnected(a)) {
                        say(std::string("time-varying or connected ") + name,
                            r.moverPath);
                    }
                }
            }
            for (const SdfPath &influence : r.binding.influences) {
                if (!E._poseSeedFrames.count(influence)) {
                    say("skin influence is not a seeded pose provider",
                        influence);
                }
            }
        }
    };
    for (const auto &[target, revisions] : E._graphChains) {
        for (const auto &revision : revisions) {
            checkRevision(revision, /* derived = */ false);
        }
    }
    for (const auto &[target, revisions] : E._graphDerivedChains) {
        for (const auto &revision : revisions) {
            checkRevision(revision, /* derived = */ true);
        }
    }

    // The reasons are a set in spirit: one line per feature, not per mention.
    std::sort(out.begin() + long(before), out.end());
    out.erase(std::unique(out.begin() + long(before), out.end()), out.end());
    return out.size() == before;
}

// ---------------------------------------------------------------------------
// The bake.
// ---------------------------------------------------------------------------

RigExecBakedProgram::RigExecBakedProgram(std::unique_ptr<_Impl> impl)
    : _impl(std::move(impl))
{
}

RigExecBakedProgram::~RigExecBakedProgram() = default;

size_t RigExecBakedProgram::GetProviderCount() const {
    return _impl->paths.size();
}
size_t RigExecBakedProgram::GetBoundInputCount() const {
    return _impl->boundInputs;
}
size_t RigExecBakedProgram::GetVaryingInputCount() const {
    return _impl->varyingInputs;
}

void RigExecBakedProgram::AdoptGeometryStateFrom(
    RigExecBakedProgram &previous) {
    _Impl &B = *_impl;
    _Impl &P = *previous._impl;
    // One revision's run state, moved from the node the outgoing program
    // held. Everything here is a CACHE of the last run; the compiled
    // description around it comes from the epoch the new program was built
    // from and is left alone.
    //
    // \p keepRun says whether the cached RESULT may be kept as well as the
    // node: a revision spliced into or out of a chain changes the point
    // stream every revision after it reads, which is why the dynamic path's
    // VdfNetwork re-executes them, so from the first divergence on the node
    // survives but its result does not.
    const auto adopt = [](_Impl::_GeomRevision *destination,
                          _Impl::_GeomRevision *source, bool keepRun) {
        destination->created = false;
        if (!keepRun) {
            return;
        }
        destination->resultStatus = source->resultStatus;
        destination->output = std::move(source->output);
        destination->lastParameters = std::move(source->lastParameters);
        destination->lastStatus = source->lastStatus;
        destination->ran = source->ran;
    };
    std::map<SdfPath, _Impl::_GeomChain *> outgoing;
    for (_Impl::_GeomChain &chain : P.chains) {
        outgoing.emplace(chain.target, &chain);
    }
    for (_Impl::_GeomChain &chain : B.chains) {
        const auto found = outgoing.find(chain.target);
        if (found == outgoing.end()) {
            // A target the outgoing program did not drive: every revision of
            // it is genuinely new, which is also what the dynamic path says
            // about a live graph it has just created.
            continue;
        }
        _Impl::_GeomChain &old = *found->second;
        std::map<std::pair<SdfPath, RigExecRevisionOp>,
                 _Impl::_GeomRevision *> retained;
        for (_Impl::_GeomRevision &revision : old.revisions) {
            retained.emplace(std::make_pair(revision.moverPath, revision.op),
                             &revision);
        }
        // The first position at which the two identity sequences disagree:
        // everything before it reads the same point stream as before and
        // everything from it on does not.
        size_t divergence = 0;
        while (divergence < chain.revisions.size() &&
               divergence < old.revisions.size() &&
               old.revisions[divergence].moverPath ==
                   chain.revisions[divergence].moverPath &&
               old.revisions[divergence].op == chain.revisions[divergence].op) {
            ++divergence;
        }
        const bool sameSequence = divergence == chain.revisions.size() &&
                                  divergence == old.revisions.size();
        for (size_t i = 0; i < chain.revisions.size(); ++i) {
            _Impl::_GeomRevision &revision = chain.revisions[i];
            const auto node =
                retained.find(std::make_pair(revision.moverPath, revision.op));
            if (node == retained.end()) {
                continue;
            }
            adopt(&revision, node->second, /* keepRun = */ i < divergence);
            retained.erase(node);
        }
        // Insertion, removal and reordering rebuild the schedule; a rebind
        // only updates packets. Same rule, same words, as the dynamic walk.
        chain.scheduleDirty = !sameSequence;
        chain.lastBase = std::move(old.lastBase);
        chain.result = std::move(old.result);
        chain.haveResult = old.haveResult;

        std::map<SdfPath, _Impl::_GeomChain::_Derived *> outgoingDerived;
        for (_Impl::_GeomChain::_Derived &derived : old.derived) {
            outgoingDerived.emplace(derived.target, &derived);
        }
        for (_Impl::_GeomChain::_Derived &derived : chain.derived) {
            const auto match = outgoingDerived.find(derived.target);
            if (match == outgoingDerived.end() ||
                match->second->revision.moverPath !=
                    derived.revision.moverPath ||
                match->second->revision.op != derived.revision.op) {
                continue;
            }
            adopt(&derived.revision, &match->second->revision,
                  /* keepRun = */ true);
            derived.lastBase = std::move(match->second->lastBase);
            derived.result = std::move(match->second->result);
            derived.haveResult = match->second->haveResult;
        }
    }
}

// ---------------------------------------------------------------------------
// Invalidation.
// ---------------------------------------------------------------------------

bool
RigExecBakedProgram::IsInvalidatedBy(
    const UsdNotice::ObjectsChanged &notice) const
{
    const _Impl &B = *_impl;
    // A changed-info notice is a VALUE edit on a property that already
    // existed. It matters only where the bake read that value -- and, for
    // the Xforms bakeability accepted for composing to the identity, on any
    // property of theirs at all, since it is their composed transform and
    // not one named attribute that was judged. The prim-level info that
    // arrives for every ancestor of an edit (an `over` being created above
    // it) is not that, and is exactly what must not rebuild.
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        if (B.rebuild.count(path) ||
            (path.IsPropertyPath() &&
             B.xformPrims.count(path.GetPrimPath()))) {
            return true;
        }
    }
    // A resync names a subtree whose composition changed: properties can have
    // appeared, disappeared or been retargeted anywhere inside it, so the
    // question is whether the subtree and the index overlap in EITHER
    // direction -- the resync above something the bake read, or at a property
    // of a prim it read from.
    const auto overlaps = [&B](const SdfPath &path) {
        for (const std::set<SdfPath> *index :
                 {&B.named, &B.prims, &B.xformPrims}) {
            // Descendants of `path` are contiguous from lower_bound: SdfPath
            // sorts in namespace order, which _OnObjectsChanged already
            // relies on for the solver-batch subtree walk.
            const auto it = index->lower_bound(path);
            if (it != index->end() && it->HasPrefix(path)) {
                return true;
            }
        }
        // A resync that names a PROPERTY is that property appearing,
        // disappearing or being retargeted. It matters where the bake asked
        // about that property by name -- not merely somewhere on the prim,
        // or every rig would rebuild for a property it never reads.
        const SdfPath prim = path.GetPrimPath();
        if (path.IsPropertyPath()) {
            return B.xformPrims.count(prim) > 0;
        }
        return B.prims.count(prim) > 0 || B.xformPrims.count(prim) > 0;
    };
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        if (overlaps(path)) {
            return true;
        }
    }
    for (const SdfPath &path : notice.GetResolvedAssetPathsResyncedPaths()) {
        if (overlaps(path)) {
            return true;
        }
    }
    return false;
}

// Exact equality, not a tolerance: the program exists to produce the same
// numbers, so "close" is a failure with extra steps.
void
RigExecComparePoses(const RigExecRigPose &reference,
                    const RigExecRigPose &baked, RigExecRigPose *out)
{
    // Read FIRST, before anything below appends: `out` is allowed to be the
    // reference pose -- the evaluator passes it that way, so a disagreement
    // is reported on the generation it publishes -- and a diagnostic appended
    // by an earlier domain would otherwise make this domain disagree about
    // a pose that agreed.
    const bool sameDiagnostics = reference.diagnostics == baked.diagnostics;
    const auto sameFrame = [](const RigExecPointFrame &a,
                              const RigExecPointFrame &b) {
        return a.flags == b.flags && a.points == b.points;
    };
    const auto compare = [&out](const auto &referenceMap, const auto &bakedMap,
                                const char *what, auto equal) {
        for (const auto &[path, value] : referenceMap) {
            const auto found = bakedMap.find(path);
            if (found == bakedMap.end()) {
                out->diagnostics.push_back(
                    std::string("baked parity: no ") + what + " for " +
                    path.GetString());
                ++out->bakedParityMismatches;
            } else if (!equal(value, found->second)) {
                out->diagnostics.push_back(
                    std::string("baked parity: ") + what + " differs at " +
                    path.GetString());
                ++out->bakedParityMismatches;
            }
        }
        for (const auto &[path, value] : bakedMap) {
            if (!referenceMap.count(path)) {
                out->diagnostics.push_back(
                    std::string("baked parity: unexpected ") + what + " at " +
                    path.GetString());
                ++out->bakedParityMismatches;
            }
        }
    };
    const auto sameMatrix = [](const GfMatrix4d &a, const GfMatrix4d &b) {
        return a == b;
    };
    compare(reference.jointFramesBase, baked.jointFramesBase,
            "base joint frame", sameFrame);
    compare(reference.jointFramesFinal, baked.jointFramesFinal,
            "final joint frame", sameFrame);
    compare(reference.jointMatricesFinal, baked.jointMatricesFinal,
            "joint matrix", sameMatrix);
    compare(reference.controlFrames, baked.controlFrames, "control frame",
            sameFrame);
    compare(reference.providerXforms, baked.providerXforms,
            "provider transform", sameMatrix);
    compare(reference.providerBaseXforms, baked.providerBaseXforms,
            "provider base transform", sameMatrix);
    compare(reference.movedProperties, baked.movedProperties,
            "moved property",
            [](const VtValue &a, const VtValue &b) { return a == b; });
    // The observational guides too: they are a published domain like any
    // other, and a solver aggregate that drifts shows up here first.
    compare(reference.solverFrames, baked.solverFrames, "solver frames",
            [&sameFrame](const std::vector<RigExecPointFrame> &a,
                         const std::vector<RigExecPointFrame> &b) {
                return a.size() == b.size() &&
                       std::equal(a.begin(), a.end(), b.begin(), sameFrame);
            });

    // The SCALARS of the generation. They are published state a consumer
    // reads -- an editor shows the work counters, a test asserts on them --
    // so a program that lands on the right points while reporting different
    // work is still a second rig, and the difference is exactly the shape a
    // map comparison cannot see. Counted separately from the maps: each is
    // its own domain, so a mismatch of one names which one moved. The
    // diagnostics are compared in ORDER, because the order is the walk order
    // and a consumer reading "MoverFailed X" after "constraint Y passed
    // through" is being told a sequence.
    const auto compareCount = [&out](size_t referenceValue, size_t bakedValue,
                                     const char *what) {
        if (referenceValue != bakedValue) {
            out->diagnostics.push_back(
                std::string("baked parity: ") + what + " differs (" +
                std::to_string(referenceValue) + " vs " +
                std::to_string(bakedValue) + ")");
            ++out->bakedParityMismatches;
        }
    };
    compareCount(reference.moverGraphRevisionsCreated,
                 baked.moverGraphRevisionsCreated,
                 "mover graph revisions created");
    compareCount(reference.moverGraphRevisionsExecuted,
                 baked.moverGraphRevisionsExecuted,
                 "mover graph revisions executed");
    compareCount(reference.moverGraphSchedulesBuilt,
                 baked.moverGraphSchedulesBuilt,
                 "mover graph schedules built");
    compareCount(reference.solverOverrideRounds, baked.solverOverrideRounds,
                 "solver override rounds");
    // solverEvaluations is deliberately NOT compared. It counts the solver
    // computations the dependency schedule actually REQUESTED, and the
    // dynamic path's per-batch exec cache lets it skip a batch whose time and
    // inputs are the ones it already answered -- so re-evaluating the same
    // frame twice costs it nothing and costs the program, which holds no such
    // cache and re-solves, its whole schedule. Both numbers are true of the
    // path that reported them; they are not two answers to one question, and
    // making them agree would mean either the program inventing a cache or
    // the dynamic path giving one up.
    if (!sameDiagnostics) {
        out->diagnostics.push_back(
            "baked parity: diagnostics differ (" +
            std::to_string(reference.diagnostics.size()) + " vs " +
            std::to_string(baked.diagnostics.size()) + " line(s))");
        ++out->bakedParityMismatches;
    }
}

// ---------------------------------------------------------------------------
// Interactive overrides.
// ---------------------------------------------------------------------------

bool
RigExecBakedProgram::SetOverrides(
    const std::vector<RigExecValueOverride> &overrides)
{
    _Impl &B = *_impl;
    if (overrides.empty()) {
        if (B.anyOverridden) {
            std::fill(B.overridden.begin(), B.overridden.end(), 0);
            B.anyOverridden = false;
        }
        return true;
    }
    if (B.anyOverridden) {
        std::fill(B.overridden.begin(), B.overridden.end(), 0);
        B.anyOverridden = false;
    }
    bool placeable = true;
    for (const RigExecValueOverride &o : overrides) {
        // A computation override names an exec computation, and there is no
        // exec here to hold it against.
        if (o.attribute.IsEmpty()) {
            placeable = false;
            continue;
        }
        const SdfPath path = o.prim.AppendProperty(o.attribute);
        // Folded first: a property can be both a per-frame input and the
        // source of something resolved once, and the once wins.
        if (B.folded.count(path)) {
            placeable = false;
            continue;
        }
        const auto found = B.overridableInputs.find(path);
        if (found != B.overridableInputs.end()) {
            for (int index : found->second) {
                B.overridden[size_t(index)] = 1;
            }
            B.anyOverridden = true;
            continue;
        }
        // Nothing to place: every read of this prim already goes through the
        // resolved inputs the override was written into.
        if (B.resolvedRoutedPrims.count(o.prim)) {
            continue;
        }
        placeable = false;
    }
    return placeable;
}

std::unique_ptr<RigExecBakedProgram>
RigExecBakedProgram::Build(RigExecRigEvaluator *evaluator,
                           std::vector<std::string> *reasons)
{
    if (!evaluator || !IsBakeable(*evaluator, reasons)) {
        return nullptr;
    }
    RigExecRigEvaluator &E = *evaluator;
    auto impl = std::make_unique<_Impl>();
    _Impl &B = *impl;
    B.evaluator = evaluator;
    B.stage = E._stage;
    const UsdTimeCode capture = UsdTimeCode::Default();
    const UsdTimeCode probe = _ProbeTime(B.stage);
    bool ok = true;
    auto refuse = [&](const std::string &what, const SdfPath &where) {
        if (reasons) reasons->push_back(what + ": " + where.GetString());
        ok = false;
    };

    // Every property a chain writes. An input resolving through one of these
    // cannot be captured, because the chain recomputes it every generation.
    std::set<SdfPath> chainTargets;
    for (const auto &[target, revisions] : E._propertyChains) {
        chainTargets.insert(target);
    }

    // Everything the bake READ but did not register as a per-frame input: its
    // value is folded into program state, so an edit rebuilds and an
    // interactive override on it cannot be placed.
    auto fold = [&](const UsdPrim &prim, const char *name) {
        if (!prim) {
            return;
        }
        const SdfPath path = prim.GetPath().AppendProperty(TfToken(name));
        B.rebuild.insert(path);
        B.folded.insert(path);
        B.named.insert(path);
        B.prims.insert(prim.GetPath());
    };
    // Read at bake for something other than its value -- whether it is
    // authored at all, how many samples it has. An edit rebuilds; an
    // interactive override, which changes no authored opinion, still places.
    auto foldShape = [&](const UsdPrim &prim, const char *name) {
        if (prim) {
            const SdfPath path = prim.GetPath().AppendProperty(TfToken(name));
            B.rebuild.insert(path);
            B.named.insert(path);
            B.prims.insert(prim.GetPath());
        }
    };
    auto readToken = [&](const UsdPrim &prim, const char *name,
                         const char *fallback) {
        fold(prim, name);
        return _ReadToken(prim, name, fallback);
    };
    auto targets = [&](const UsdPrim &prim, const char *name) {
        fold(prim, name);
        return _Targets(prim, name);
    };
    auto bind = [&](const UsdPrim &prim, const char *name, auto fallback) {
        SdfPathVector walk;
        auto input =
            _BindInput(prim, name, fallback, capture, chainTargets, &walk);
        // A selection that moves with the time code is read the long way,
        // through this generation's resolved inputs, rather than through a
        // query pinned to the wrong attribute.
        if (input.varying && input.query.IsValid()) {
            bool viaChain = false, varying = false;
            UsdAttribute atProbe;
            _ClassifyInput<decltype(fallback)>(
                prim.GetAttribute(TfToken(name)), probe, chainTargets,
                &viaChain, &varying, &atProbe);
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
    };

    // Stage metadata, which arrives as a changed-info notice on the
    // pseudo-root: the up axis an aim constraint resolves against, and the
    // time-code range the input classification probes at.
    B.rebuild.insert(SdfPath::AbsoluteRootPath());

    // ---- dense provider slots ---------------------------------------------
    for (const auto &[path, tap] : E._poseSeedFrames) {
        B.index[path] = int(B.paths.size());
        B.paths.push_back(path);
    }
    const int N = int(B.paths.size());
    B.parent.assign(N, -1);
    for (int i = 0; i < N; ++i) {
        for (SdfPath p = B.paths[i].GetParentPath();
             !p.IsEmpty() && p != SdfPath::AbsoluteRootPath();
             p = p.GetParentPath()) {
            const auto it = B.index.find(p);
            if (it != B.index.end()) {
                B.parent[i] = it->second;
                break;
            }
        }
        if (B.parent[i] >= i) {
            refuse("provider slots are not in namespace DFS order",
                   B.paths[i]);
        }
    }
    auto slotOf = [&](const SdfPath &path) {
        const auto it = B.index.find(path);
        return it == B.index.end() ? -1 : it->second;
    };

    // ---- the rest chain and the default-space ladder -----------------------
    //
    // computations.cpp resolves both per provider per evaluation through
    // eight computations; every input to them is epoch-constant on a bakeable
    // rig, so both resolve once here. The frame round trips exec performs
    // between computations are reproduced, not simplified away.
    B.restM.assign(N, GfMatrix4d(1.0));
    B.restPts.resize(N);
    B.restFrames.resize(N);
    B.selfD.assign(N, GfMatrix4d(1.0));
    B.parentDinv.assign(N, GfMatrix4d(1.0));
    B.rotOrder.assign(N, TfToken("XYZ"));
    std::vector<GfMatrix4d> restRoundTrip(N, GfMatrix4d(1.0));
    std::vector<GfMatrix4d> defaultRoundTrip(N, GfMatrix4d(1.0));
    for (int i = 0; i < N; ++i) {
        const UsdPrim prim = B.stage->GetPrimAtPath(B.paths[i]);
        B.prims.insert(B.paths[i]);
        // The ladder is resolved once and folded into restM/selfD, so every
        // attribute feeding it is a captured constant -- including the space
        // EXPRESSIONS, which bakeability accepted because they are unauthored
        // and would change the compose if they stopped being so.
        for (const char *name : {"posed:space", "parent:space",
                                 "parent:defaultSpace", "avars:defaultSpace",
                                 "posed:defaultSpace", "avars:rotationOrder"}) {
            fold(prim, name);
        }
        auto number = [&](const char *name, double fallback) {
            SdfPathVector walk;
            const _Input<double> input = _BindInput(
                prim, name, fallback, capture, chainTargets, &walk);
            fold(prim, name);
            B.rebuild.insert(walk.begin(), walk.end());
            B.folded.insert(walk.begin(), walk.end());
            return _Read(input, E._resolvedInputs, capture);
        };
        GfMatrix4d space(1.0);
        fold(prim, "rest:space");
        if (const UsdAttribute a = prim.GetAttribute(TfToken("rest:space"))) {
            a.Get(&space);
        }
        GfMatrix4d rest = _ComposeAvars(
            number("rest:tx", 0), number("rest:ty", 0), number("rest:tz", 0),
            1, 1, 1, number("rest:rx", 0), number("rest:ry", 0),
            number("rest:rz", 0), 0, TfToken("XYZ")) * space;
        rest.Orthonormalize(/* issueWarning = */ false);
        const GfMatrix4d parentRest =
            B.parent[i] >= 0 ? restRoundTrip[B.parent[i]] : GfMatrix4d(1.0);
        B.restM[i] = rest * parentRest;
        B.restFrames[i] = RigExecFrameFromMatrix(B.restM[i]);
        B.restPts[i] = B.restFrames[i].points;
        restRoundTrip[i] = _RoundTrip(B.restM[i]);

        // default:space is a space EXPRESSION: a non-identity authored value
        // wins, otherwise the computed ladder.
        GfMatrix4d authoredDefault(1.0);
        bool haveAuthored = false;
        fold(prim, "default:space");
        if (const UsdAttribute a =
                prim.GetAttribute(TfToken("default:space"))) {
            a.Get(&authoredDefault);
            haveAuthored = authoredDefault != GfMatrix4d(1.0);
        }
        const GfMatrix4d parentDefault =
            B.parent[i] >= 0 ? defaultRoundTrip[B.parent[i]]
                             : GfMatrix4d(1.0);
        if (haveAuthored) {
            B.selfD[i] = authoredDefault;
        } else {
            const GfMatrix4d offset = _ComposeAvars(
                number("default:tx", 0), number("default:ty", 0),
                number("default:tz", 0), 1, 1, 1, number("default:rx", 0),
                number("default:ry", 0), number("default:rz", 0), 0,
                TfToken("XYZ"));
            B.selfD[i] = offset * restRoundTrip[i] * parentRest.GetInverse() *
                         parentDefault;
        }
        defaultRoundTrip[i] = _RoundTrip(B.selfD[i]);
        B.parentDinv[i] = parentDefault.GetInverse();
        if (const UsdAttribute a =
                prim.GetAttribute(TfToken("avars:rotationOrder"))) {
            TfToken order;
            if (a.Get(&order) && !order.IsEmpty()) {
                B.rotOrder[i] = order;
            }
        }
    }

    // ---- the input binding table -------------------------------------------
    B.avarConstants.assign(size_t(N) * 11, 0.0);
    for (int i = 0; i < N; ++i) {
        const UsdPrim prim = B.stage->GetPrimAtPath(B.paths[i]);
        for (int c = 0; c < 11; ++c) {
            const size_t slot = size_t(i) * 11 + size_t(c);
            _Input<double> input =
                bind(prim, _kAvarNames[c], _kAvarDefaults[c]);
            B.avarConstants[slot] = input.constant;
            if (input.varying) {
                B.avarBindings.push_back({slot, std::move(input)});
            } else if (input.overrideIndex >= 0) {
                B.avarConstantBindings.push_back({slot, std::move(input)});
            }
        }
    }
    B.avars = B.avarConstants;

    // ---- solvers, in compiled batch order ----------------------------------
    auto bakeSolver = [&](const SdfPath &solverPath) {
        _Impl::_Solver s;
        s.path = solverPath;
        const UsdPrim prim = B.stage->GetPrimAtPath(solverPath);
        s.type = prim.GetTypeName();

        // rigExec:joints is the single declaration of the chain: a solver
        // whose output is consumed downstream claims no joints but still
        // names them for their REST measurements.
        std::vector<std::pair<int, int>> restRefs;
        {
            const SdfPathVector joints = targets(prim, "rigExec:joints");
            VtIntArray elements;
            fold(prim, "rigExec:jointElements");
            if (const UsdAttribute a =
                    prim.GetAttribute(TfToken("rigExec:jointElements"))) {
                a.Get(&elements);
            }
            for (size_t k = 0; k < joints.size(); ++k) {
                restRefs.emplace_back(
                    slotOf(joints[k]),
                    elements.size() == joints.size() ? elements[k] : int(k));
            }
        }
        const auto joints = E._solverJoints.find(solverPath);
        if (joints != E._solverJoints.end()) {
            for (const auto &[joint, element] : joints->second) {
                const int slot = slotOf(joint);
                if (slot < 0) {
                    refuse("solver output is not a pose provider", joint);
                    continue;
                }
                s.outputs.emplace_back(slot, element);
            }
        }

        if (s.type == "RigExecFkChain") {
            s.parentRelative =
                readToken(prim, "rigExec:controlSpace", "") ==
                "parentRelative";
            for (const SdfPath &t : targets(prim, "rigExec:controls")) {
                const int slot = slotOf(t);
                if (slot < 0) refuse("FkChain control is not a provider", t);
                s.controls.push_back(slot);
                s.controlRests.push_back(
                    slot >= 0 ? B.restPts[slot] : RigExecIdentityLandmarks());
            }
        } else if (s.type == "RigExecTwoBoneIk") {
            const auto r = targets(prim, "rigExec:rootControl");
            const auto e = targets(prim, "rigExec:effectorControl");
            const auto p = targets(prim, "rigExec:poleControl");
            s.root = r.empty() ? -1 : slotOf(r[0]);
            s.end = e.empty() ? -1 : slotOf(e[0]);
            s.pole = p.empty() ? -1 : slotOf(p[0]);
            if (s.root < 0 || s.end < 0 || s.pole < 0) {
                refuse("TwoBoneIk control is not a provider", solverPath);
            }
            for (const auto &[slot, element] : restRefs) {
                if (slot >= 0 && element >= 0 && element < 3) {
                    s.ikRests[size_t(element)] = B.restPts[slot];
                }
            }
            // The bone lengths exec measures every evaluation, measured once.
            s.upperLengthBase = (s.ikRests[1][0] - s.ikRests[0][0]).GetLength();
            s.lowerLengthBase = (s.ikRests[2][0] - s.ikRests[1][0]).GetLength();
            s.bend = bind(prim, "rigExec:preferredBendRadians", 0.0);
            s.upperOffset = bind(prim, "rigExec:upperLengthOffset", 0.0);
            s.lowerOffset = bind(prim, "rigExec:lowerLengthOffset", 0.0);
            s.stretch = bind(prim, "inputs:stretch", 1.0f);
            s.softness = bind(prim, "inputs:softness", 0.0f);
            s.ikParams.preferredBendRadians = s.bend.constant;
            s.ikParams.stretch = s.stretch.constant;
            s.ikParams.softness = s.softness.constant;
            s.ikParams.upperLength =
                s.upperLengthBase + s.upperOffset.constant;
            s.ikParams.lowerLength =
                s.lowerLengthBase + s.lowerOffset.constant;
        } else if (s.type == "RigExecBlendPointFrames") {
            auto solverSlot = [&](const SdfPathVector &v) {
                if (v.empty()) return -1;
                const auto it = B.solverIndex.find(v[0]);
                return it == B.solverIndex.end() ? -1 : it->second;
            };
            s.inA = solverSlot(targets(prim, "rigExec:inputA"));
            s.inB = solverSlot(targets(prim, "rigExec:inputB"));
            if (s.inA < 0 || s.inB < 0) {
                refuse("BlendPointFrames input is not an earlier solver",
                       solverPath);
            }
            s.blendWeight = bind(prim, "inputs:weight", 0.0f);
            s.scaleMode =
                readToken(prim, "rigExec:scaleBlend", "") == "linear"
                    ? RigExecScaleBlend::Linear
                    : RigExecScaleBlend::Log;
            if (readToken(prim, "rigExec:rotationBlend", "shortestArc") !=
                "shortestArc") {
                refuse("BlendPointFrames rotationBlend is not shortestArc",
                       solverPath);
            }
        } else if (s.type == "RigExecSplineIk") {
            const auto r = targets(prim, "rigExec:rootControl");
            const auto m = targets(prim, "rigExec:midControl");
            const auto e = targets(prim, "rigExec:endControl");
            s.root = r.empty() ? -1 : slotOf(r[0]);
            s.mid = m.empty() ? -1 : slotOf(m[0]);
            s.end = e.empty() ? -1 : slotOf(e[0]);
            if (s.root < 0 || s.mid < 0 || s.end < 0) {
                refuse("SplineIk control is not a provider", solverPath);
            }
            const size_t count = restRefs.size();
            s.splineCount = count;
            std::vector<RigExecPointFrame> restJoints(count);
            s.splineJointRests.resize(count);
            for (const auto &[slot, element] : restRefs) {
                if (slot < 0 || element < 0 || size_t(element) >= count) {
                    refuse("SplineIk joint element is out of range",
                           solverPath);
                    continue;
                }
                restJoints[size_t(element)] = B.restFrames[slot];
                s.splineJointRests[size_t(element)] = B.restPts[slot];
            }
            std::vector<double> weights;
            VtFloatArray authoredWeights;
            fold(prim, "rigExec:volumeWeights");
            if (const UsdAttribute a =
                    prim.GetAttribute(TfToken("rigExec:volumeWeights"))) {
                a.Get(&authoredWeights);
            }
            for (float w : authoredWeights) weights.push_back(w);
            if (!weights.empty() && weights.size() != count) {
                refuse("SplineIk volumeWeights cardinality", solverPath);
            }
            const TfToken restTok = readToken(prim, "rigExec:restLength", "");
            if (!restTok.IsEmpty() && restTok != "curve" &&
                restTok != "chain") {
                refuse("SplineIk restLength is unsupported", solverPath);
            }
            // Exec rebuilds this description every evaluation; it is a pure
            // function of epoch-constant rests, so it bakes out.
            s.splineRest = RigExecSplineIkMakeRest(
                restJoints,
                s.root >= 0 ? B.restFrames[s.root] : RigExecPointFrame(),
                s.mid >= 0 ? B.restFrames[s.mid] : RigExecPointFrame(),
                s.end >= 0 ? B.restFrames[s.end] : RigExecPointFrame(),
                weights,
                restTok == "chain" ? RigExecSplineIkRestLength::Chain
                                   : RigExecSplineIkRestLength::Curve);
            s.preserveVolume = bind(prim, "inputs:preserveVolume", 1.0);
            s.midFollowWeight = bind(prim, "inputs:midFollowWeight", 0.5);
            s.roll = bind(prim, "inputs:roll", 0.0);
            s.twist = bind(prim, "inputs:twist", 0.0);
            s.minLengthRatio = bind(prim, "inputs:minLengthRatio", 0.0);
            s.splineParamsVary =
                s.preserveVolume.varying || s.midFollowWeight.varying ||
                s.roll.varying || s.twist.varying || s.minLengthRatio.varying;
            s.splineParams.preserveVolume = s.preserveVolume.constant;
            s.splineParams.midFollowWeight = s.midFollowWeight.constant;
            s.splineParams.roll = GfDegreesToRadians(s.roll.constant);
            s.splineParams.twist = GfDegreesToRadians(s.twist.constant);
            s.splineParams.minLengthRatio = s.minLengthRatio.constant;
            const TfToken tangent = readToken(prim, "rigExec:rootTangent", "");
            if (tangent == "aim") {
                s.splineParams.aimRootTangent = true;
            } else if (!tangent.IsEmpty() && tangent != "rigid") {
                refuse("SplineIk rootTangent is unsupported", solverPath);
            }
        }
        const int slot = int(B.solvers.size());
        B.solverIndex[solverPath] = slot;
        B.solvers.push_back(std::move(s));
        return slot;
    };

    // ---- constraints --------------------------------------------------------
    auto bakeConstraint = [&](const RigExecRigEvaluator::_FrameConstraint &fc) {
        _Impl::_Constraint c;
        c.path = fc.moverPath;
        c.type = fc.schemaType;
        const UsdPrim prim = B.stage->GetPrimAtPath(fc.moverPath);
        c.target = fc.targets.empty() ? -1 : slotOf(fc.targets[0]);
        for (const auto &source : fc.sources) {
            const int slot = slotOf(source.sourcePath);
            if (slot < 0) {
                refuse("constraint source is not a pose provider",
                       source.sourcePath);
            }
            c.sources.push_back(slot);
        }
        c.enabled = bind(prim, "inputs:enabled", true);
        c.defaultWeight = bind(prim, "inputs:defaultWeight", 1.0f);

        // inputs:sourceWeights / the parent offsets are authored tables, read
        // as arrays rather than as a per-source scalar.
        const size_t n = c.sources.size();
        VtFloatArray weights;
        fold(prim, "inputs:sourceWeights");
        if (const UsdAttribute a =
                prim.GetAttribute(TfToken("inputs:sourceWeights"))) {
            if (_AnimatedOrConnected(a)) {
                refuse("animated inputs:sourceWeights", fc.moverPath);
            }
            a.Get(&weights);
            ++B.boundInputs;
        }
        c.authoredSourceWeights = weights.size();
        c.sourceWeights.resize(n);
        for (size_t k = 0; k < n; ++k) {
            c.sourceWeights[k].constant = k < weights.size() ? weights[k] : 1.0f;
        }
        if (!weights.empty() && weights.size() != n) {
            // The dynamic path diagnoses this and passes the constraint
            // through; the program refuses instead of reproducing a
            // malformed-input message from baked state.
            refuse("inputs:sourceWeights cardinality", fc.moverPath);
        }
        c.translationOffsets.assign(n, GfVec3d(0));
        c.rotationOffsets.assign(n, GfVec3d(0));
        if (fc.schemaType == "RigExecParentConstraint") {
            VtVec3dArray translations, rotations;
            for (const char *name : {"inputs:translationOffsets",
                                     "inputs:rotationOffsets"}) {
                fold(prim, name);
                if (const UsdAttribute a = prim.GetAttribute(TfToken(name))) {
                    if (_AnimatedOrConnected(a)) {
                        refuse(std::string("animated ") + name, fc.moverPath);
                    }
                    ++B.boundInputs;
                }
            }
            if (const UsdAttribute a = prim.GetAttribute(
                    TfToken("inputs:translationOffsets"))) {
                a.Get(&translations);
            }
            if (const UsdAttribute a =
                    prim.GetAttribute(TfToken("inputs:rotationOffsets"))) {
                a.Get(&rotations);
            }
            if ((!translations.empty() && translations.size() != n) ||
                (!rotations.empty() && rotations.size() != n)) {
                refuse("parent constraint offset cardinality", fc.moverPath);
            }
            for (size_t k = 0; k < translations.size() && k < n; ++k) {
                c.translationOffsets[k] = translations[k];
            }
            for (size_t k = 0; k < rotations.size() && k < n; ++k) {
                c.rotationOffsets[k] = rotations[k];
            }
        }
        c.order =
            _ParseEulerOrder(readToken(prim, "rigExec:rotationOrder", "XYZ"));

        if (fc.schemaType == "RigExecPositionConstraint") {
            c.affectX = bind(prim, "inputs:affectTranslationX", true);
            c.affectY = bind(prim, "inputs:affectTranslationY", true);
            c.affectZ = bind(prim, "inputs:affectTranslationZ", true);
            c.offset = bind(prim, "inputs:translationOffset", GfVec3d(0));
        } else if (fc.schemaType == "RigExecRotationConstraint") {
            c.affectX = bind(prim, "inputs:affectRotationX", true);
            c.affectY = bind(prim, "inputs:affectRotationY", true);
            c.affectZ = bind(prim, "inputs:affectRotationZ", true);
            c.offset = bind(prim, "inputs:rotationOffset", GfVec3d(0));
        } else if (fc.schemaType == "RigExecScaleConstraint") {
            c.affectX = bind(prim, "inputs:affectScaleX", true);
            c.affectY = bind(prim, "inputs:affectScaleY", true);
            c.affectZ = bind(prim, "inputs:affectScaleZ", true);
            c.offset = bind(prim, "inputs:scaleOffset", GfVec3d(0));
        } else if (fc.schemaType == "RigExecParentConstraint") {
            c.tX = bind(prim, "inputs:affectTranslationX", true);
            c.tY = bind(prim, "inputs:affectTranslationY", true);
            c.tZ = bind(prim, "inputs:affectTranslationZ", true);
            c.rX = bind(prim, "inputs:affectRotationX", true);
            c.rY = bind(prim, "inputs:affectRotationY", true);
            c.rZ = bind(prim, "inputs:affectRotationZ", true);
            // FBX disables scale by default; the false fallback is the
            // authored contract (schema.usda), not an oversight.
            c.sX = bind(prim, "inputs:affectScaleX", false);
            c.sY = bind(prim, "inputs:affectScaleY", false);
            c.sZ = bind(prim, "inputs:affectScaleZ", false);
        } else if (fc.schemaType == "RigExecAimConstraint") {
            c.affectX = bind(prim, "inputs:affectRotationX", true);
            c.affectY = bind(prim, "inputs:affectRotationY", true);
            c.affectZ = bind(prim, "inputs:affectRotationZ", true);
            c.aimVector = bind(prim, "inputs:aimVector", GfVec3d(1, 0, 0));
            const UsdAttribute aimAttr =
                prim.GetAttribute(TfToken("inputs:aimVector"));
            // Whether it is authored, not what it holds: the value stays a
            // per-frame read (and so an override can stand on it), but the
            // choice between aimVector and the legacy aimAxis is baked.
            foldShape(prim, "inputs:aimVector");
            c.aimVectorAuthored = aimAttr && aimAttr.HasAuthoredValueOpinion();
            // Existing assets author aimAxis but predate aimVector; keep that
            // meaning until they opt into the vector form.
            const TfToken axis = readToken(prim, "rigExec:aimAxis", "x");
            c.aimAxisFallback = axis == "y" ? GfVec3d(0, 1, 0)
                              : axis == "z" ? GfVec3d(0, 0, 1)
                                            : GfVec3d(1, 0, 0);
            c.upVector = bind(prim, "inputs:upVector", GfVec3d(0, 1, 0));
            c.rotationOffset = bind(prim, "inputs:rotationOffset", GfVec3d(0));
            c.worldUpVector = bind(prim, "inputs:worldUpVector",
                                   GfVec3d(0, 1, 0));
            c.worldUpType = readToken(prim, "rigExec:worldUpType", "none");
            const std::string up = UsdGeomGetStageUpAxis(B.stage).GetString();
            c.sceneUp = (up == "Z" || up == "z") ? GfVec3d(0, 0, 1)
                                                 : GfVec3d(0, 1, 0);
            // The legacy aimTarget/aimAxis contract preserves input up; FBX
            // WorldUpType=None is the distinct minimum-swing mode.
            c.preserveInputUp =
                targets(prim, "rigExec:sources").empty();
            c.worldUpObjectNamed =
                !fc.worldUpObject.sourcePath.IsEmpty();
            c.worldUpObject = c.worldUpObjectNamed
                                  ? slotOf(fc.worldUpObject.sourcePath)
                                  : -1;
            if (c.worldUpObjectNamed && c.worldUpObject < 0) {
                refuse("aim world-up object is not a pose provider",
                       fc.worldUpObject.sourcePath);
            }
        }
        B.constraints.push_back(std::move(c));
        return int(B.constraints.size()) - 1;
    };

    // ---- descendant propagation, decided once -------------------------------
    std::vector<bool> ownedBySolver(N, false);
    for (int i = 0; i < N; ++i) {
        ownedBySolver[i] = E._jointSolverBinding.count(B.paths[i]) > 0;
    }
    // Every provider inherits its namespace pose: an authored or connected
    // parent:space would have refused the bake above, which is exactly the
    // condition the dynamic walk re-reads off the stage per descendant.
    auto buildPropagation = [&](const std::vector<int> &candidates,
                                std::vector<std::pair<int, int>> *out) {
        const std::set<int> candidateSet(candidates.begin(), candidates.end());
        std::set<int> covered;
        // Disjoint changed subtrees, in slot (== namespace) order, which is
        // the order the dynamic walk enumerates them in.
        std::vector<int> roots(candidateSet.begin(), candidateSet.end());
        int coveredRoot = -1;
        for (int root : roots) {
            if (coveredRoot >= 0 &&
                B.paths[root].HasPrefix(B.paths[coveredRoot])) {
                continue;
            }
            coveredRoot = root;
            for (int j = root + 1; j < N; ++j) {
                if (!B.paths[j].HasPrefix(B.paths[root])) break;
                if (candidateSet.count(j) || !covered.insert(j).second) {
                    continue;
                }
                int closest = B.parent[j];
                while (closest >= 0 && !candidateSet.count(closest)) {
                    closest = B.parent[closest];
                }
                if (closest < 0) continue;
                // An independently solved joint is an absolute posed
                // override: propagation cannot pass through it.
                bool blocked = false;
                for (int p = j; p >= 0 && p != closest; p = B.parent[p]) {
                    if (ownedBySolver[p]) { blocked = true; break; }
                }
                if (blocked) continue;
                out->emplace_back(j, closest);
            }
        }
    };

    // ---- the walk ------------------------------------------------------------
    for (const RigExecRigEvaluator::_PoseStep &step : E._poseSteps) {
        _Impl::_Step st;
        st.solverBatch = step.solverBatch;
        if (step.solverBatch) {
            const auto &batch = E._solverBatches[step.index];
            st.level = batch.level;
            std::vector<int> candidates;
            for (const auto &[solverPath, tap] : batch.solvers) {
                const int slot = bakeSolver(solverPath);
                st.batchSolvers.push_back(slot);
                for (const auto &[providerSlot, element] :
                         B.solvers[slot].outputs) {
                    candidates.push_back(providerSlot);
                }
            }
            buildPropagation(candidates, &st.propagate);
        } else {
            st.index = bakeConstraint(E._frameConstraints[step.index]);
            if (B.constraints[st.index].target >= 0) {
                buildPropagation({B.constraints[st.index].target},
                                 &st.propagate);
            }
        }
        B.steps.push_back(std::move(st));
    }
    B.aggregates.resize(B.solvers.size());

    // ---- publication ---------------------------------------------------------
    for (const SdfPath &joint : E._jointPaths) {
        B.jointSlots.push_back(slotOf(joint));
        B.jointPaths.push_back(joint);
    }
    for (const SdfPath &control : E._controlPaths) {
        B.controlSlots.push_back(slotOf(control));
        B.controlPaths.push_back(control);
    }
    for (const auto &[solverPath, tap] : E._solverArrayTaps) {
        const auto it = B.solverIndex.find(solverPath);
        if (it == B.solverIndex.end()) {
            refuse("solver publishes guides but was not baked", solverPath);
            continue;
        }
        B.solverArrays.emplace_back(solverPath, it->second);
    }

    // ---- geometry ------------------------------------------------------------
    auto bakeRevision = [&](const RigExecRigEvaluator::_GraphRevision &r) {
        _Impl::_GeomRevision out;
        out.moverPath = r.moverPath;
        out.target = r.target;
        out.moverPrim = B.stage->GetPrimAtPath(r.moverPath);
        out.op = r.op;
        out.binding = r.binding;
        out.finalPhase = r.transformFinalPhase;
        out.skinTopologyFixed = r.skinTopologyFixed;
        if (!out.moverPrim) {
            refuse("mover prim is missing", r.moverPath);
        }
        // The packet is assembled live from this prim every frame through
        // the generation's resolved inputs: nothing of it is captured, so a
        // value edit needs no rebuild and an override places itself. The
        // layout arrays are still NAMED, because bakeability judged them --
        // connecting one is a resync, and the judgement was that none is
        // connected. They do not belong in `rebuild` even though the layout
        // is no longer read per frame: it is held in the evaluator's skin
        // topology cache, which every notice clears, so a weight-paint edit
        // reaches the next generation through the re-read rather than
        // through a rebuild of this program.
        B.prims.insert(r.moverPath);
        B.resolvedRoutedPrims.insert(r.moverPath);
        for (const char *name : {"rigExec:jointIndices", "rigExec:jointWeights",
                                 "rigExec:elementSize",
                                 "rigExec:skinningMethod"}) {
            B.named.insert(r.moverPath.AppendProperty(TfToken(name)));
        }
        if (!r.binding.transform.IsEmpty()) {
            out.transformSlot = slotOf(r.binding.transform);
            if (out.transformSlot < 0) {
                refuse("mover transform provider is not a pose provider",
                       r.binding.transform);
            }
        }
        for (const SdfPath &influence : r.binding.influences) {
            const int slot = slotOf(influence);
            if (slot < 0) refuse("skin influence is not a provider", influence);
            out.influenceSlots.push_back(slot);
        }
        return out;
    };
    for (const SdfPath &target : E._chainOrder) {
        const auto chainIt = E._graphChains.find(target);
        if (chainIt == E._graphChains.end()) continue;
        _Impl::_GeomChain chain;
        chain.target = target;
        // The base points are read per frame through a query, so only the
        // query's own validity depends on this surviving a resync.
        B.prims.insert(target.GetPrimPath());
        B.named.insert(target);
        if (const UsdAttribute a = B.stage->GetAttributeAtPath(target)) {
            chain.baseQuery = UsdAttributeQuery(a);
        } else {
            refuse("chain target has no attribute", target);
        }
        for (const auto &revision : chainIt->second) {
            chain.revisions.push_back(bakeRevision(revision));
        }
        const auto derivedIt = E._graphDerivedChains.find(target);
        if (derivedIt != E._graphDerivedChains.end()) {
            for (const auto &derived : derivedIt->second) {
                _Impl::_GeomChain::_Derived d;
                d.target = derived.target;
                B.prims.insert(derived.target.GetPrimPath());
                B.named.insert(derived.target);
                if (const UsdAttribute a =
                        B.stage->GetAttributeAtPath(derived.target)) {
                    d.baseQuery = UsdAttributeQuery(a);
                } else {
                    refuse("derived target has no attribute", derived.target);
                }
                d.revision = bakeRevision(derived);
                chain.derived.push_back(std::move(d));
            }
        }
        B.chains.push_back(std::move(chain));
    }

    // ---- the rest of the invalidation index ---------------------------------
    //
    // Property chains run INSIDE the program, off the authored stage through
    // the generation's resolved inputs, so their movers are read live like a
    // geometry mover: nothing captured, and an override on one places itself.
    for (const auto &[target, revisions] : E._propertyChains) {
        B.prims.insert(target.GetPrimPath());
        B.resolvedRoutedPrims.insert(target.GetPrimPath());
        for (const RigExecRigEvaluator::_PropertyRevision &revision :
                 revisions) {
            B.prims.insert(revision.moverPath);
            B.resolvedRoutedPrims.insert(revision.moverPath);
        }
    }
    // Bakeability accepted every intervening-Xform candidate BECAUSE the
    // Xforms between it and its anchor compose to the identity and do not
    // animate. That is a value judgement about prims the rest of the index
    // never looks at, so it is recorded here or moving one of them would
    // silently change what the dynamic path composes and not the program.
    {
        const UsdPrim assetRoot =
            B.stage->GetPrimAtPath(E._rigPath.GetParentPath());
        for (const SdfPath &path : E._interveningXformProviders) {
            const auto anchorIt = E._poseProviderAnchors.find(path);
            const UsdPrim anchor =
                anchorIt == E._poseProviderAnchors.end() ||
                        anchorIt->second.IsEmpty()
                    ? assetRoot
                    : B.stage->GetPrimAtPath(anchorIt->second);
            for (UsdPrim walk = B.stage->GetPrimAtPath(path.GetParentPath());
                 walk && walk != anchor; walk = walk.GetParent()) {
                B.xformPrims.insert(walk.GetPath());
            }
        }
    }

    B.posedM.assign(N, GfMatrix4d(1.0));
    B.base.resize(N);
    B.fin.resize(N);
    if (!ok) {
        return nullptr;
    }
    return std::unique_ptr<RigExecBakedProgram>(
        new RigExecBakedProgram(std::move(impl)));
}

// ---------------------------------------------------------------------------
// One frame.
// ---------------------------------------------------------------------------

bool
RigExecBakedProgram::Run(UsdTimeCode time, RigExecRigPose *pose)
{
    _Impl &B = *_impl;
    RigExecRigEvaluator &E = *B.evaluator;
    const int N = int(B.paths.size());
    RIGEXEC_PROFILE_SCOPE_CAT(E._profiler, "Baked", "baked");

    // ---- property chains ---------------------------------------------------
    // A math mover's inputs are all authored on itself, so its chain owes exec
    // nothing and resolves first -- which is why this op can be the same
    // routine the dynamic path runs rather than a second copy of it.
    std::map<SdfPath, VtValue> propertyResults;
    E._resolvedInputs.Clear();
    E._chainSnapshots.Clear();
    // Interactive overrides are applied on BOTH sides of the property chains,
    // for the reason _EvaluateDynamic gives at the same two points: an
    // override can be either end of a chain and the two ends want opposite
    // orderings, and which end a given one is at is not knowable here.
    const bool dragging = !E._interactiveOverrides.empty();
    if (dragging) {
        E._ApplyInteractiveOverridesToResolved(&E._resolvedInputs, nullptr);
    }
    if (!E._propertyChains.empty()) {
        RIGEXEC_PROFILE_SCOPE_CAT(E._profiler, "PropertyChains", "property");
        std::vector<RigExecValueOverride> overrides;
        E._EvaluatePropertyChains(time, &propertyResults, &overrides,
                                  &pose->diagnostics);
        for (const auto &[path, value] : propertyResults) {
            E._resolvedInputs.SetProperty(path, value);
        }
    }
    if (dragging) {
        E._ApplyInteractiveOverridesToResolved(&E._resolvedInputs,
                                               &propertyResults);
    }
    const RigExecResolvedInputs &R = E._resolvedInputs;
    // Every per-frame read goes through these two: `rd` reads one input the
    // way this generation must (through the resolved inputs while an
    // override stands on it, through the pinned query otherwise), and `live`
    // answers whether a set of baked parameters has to be re-read at all.
    const auto rd = [&](const auto &input) {
        return _Read(input, R, time, &B.overridden);
    };
    const auto live = [&](const auto &input) {
        return input.varying ||
               (input.overrideIndex >= 0 &&
                B.overridden[size_t(input.overrideIndex)]);
    };

    // ---- the bound inputs --------------------------------------------------
    {
        RIGEXEC_PROFILE_SCOPE_CAT(E._profiler, "BakedInputs", "baked");
        for (const auto &binding : B.avarBindings) {
            B.avars[binding.slot] = rd(binding.input);
        }
        // A drag lands on avars the bake captured as constants -- that is
        // what dragging a control on a still rig IS -- so the varying list
        // above is not the whole table while one stands. The constant slots
        // are walked while a drag stands and once more after it is released,
        // because the released slot holds the dragged value until something
        // writes the constant back over it.
        if (B.anyOverridden || B.avarsDisturbed) {
            for (const auto &binding : B.avarConstantBindings) {
                B.avars[binding.slot] =
                    B.overridden[size_t(binding.input.overrideIndex)]
                        ? rd(binding.input)
                        : binding.input.constant;
            }
            B.avarsDisturbed = B.anyOverridden;
        }
    }

    // ---- provider frames ---------------------------------------------------
    {
        RIGEXEC_PROFILE_SCOPE_CAT(E._profiler, "BakedCompose", "baked");
        for (int i = 0; i < N; ++i) {
            const double *a = &B.avars[size_t(i) * 11];
            const double units = a[10];
            const GfMatrix4d avars = _ComposeAvars(
                a[0] * units, a[1] * units, a[2] * units, a[3], a[4], a[5],
                a[6], a[7], a[8], a[9], B.rotOrder[i]);
            const GfMatrix4d parentPosed =
                B.parent[i] >= 0 ? B.posedM[B.parent[i]] : GfMatrix4d(1.0);
            B.base[i] = RigExecFrameFromMatrix(
                avars * B.selfD[i] * B.parentDinv[i] * parentPosed);
            B.fin[i] = B.base[i];
            // _SpaceFromFrame: an unusable frame selects the NaN sentinel, so
            // the failure survives into every descendant instead of being
            // scrubbed into a plausible identity.
            GfMatrix4d space(1.0);
            if (!B.base[i].IsValid() || B.base[i].IsDegenerate() ||
                !RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                       B.base[i].points, &space)) {
                space = GfMatrix4d(1.0);
                space[3][0] = std::numeric_limits<double>::quiet_NaN();
            }
            B.posedM[i] = space;
        }
    }

    // The write bundle of one step, with the descendant propagation the bake
    // already decided. Mirrors commitConstraintFrames in rigEvaluator.cpp,
    // including which failures are diagnosed and which are silent.
    std::map<int, RigExecPointFrame> candidates;
    std::vector<std::pair<int, RigExecPointFrame>> propagated;
    enum class _Commit { Applied, PassedThrough, Bail };
    auto commit = [&](const SdfPath &moverPath,
                      const std::vector<std::pair<int, int>> &propagate,
                      bool solverOutput) {
        if (!solverOutput) {
            for (const auto &[slot, frame] : candidates) {
                if (!_Usable(frame)) {
                    pose->diagnostics.push_back(
                        moverPath.GetString() +
                        " produced an invalid or degenerate frame for " +
                        B.paths[slot].GetString() +
                        "; constraint passed through");
                    return _Commit::PassedThrough;
                }
            }
        }
        propagated.clear();
        for (const auto &[j, closest] : propagate) {
            const RigExecPointFrame &current = B.fin[j];
            const auto candidate = candidates.find(closest);
            if (candidate == candidates.end()) {
                // A solver published no element for this ancestor, so the
                // baked propagation pairs no longer describe the walk.
                return _Commit::Bail;
            }
            const RigExecPointFrame &before = B.fin[closest];
            if (solverOutput &&
                (!_Usable(current) || !_Usable(before) ||
                 !_Usable(candidate->second))) {
                continue;
            }
            if (!_Usable(current)) {
                pose->diagnostics.push_back(
                    moverPath.GetString() +
                    " could not propagate its pose revision through " +
                    B.paths[j].GetString() + "; constraint passed through");
                return _Commit::PassedThrough;
            }
            GfMatrix4d delta(1.0);
            if (!RigExecPointsToMatrix(before.points,
                                       candidate->second.points, &delta)) {
                pose->diagnostics.push_back(
                    moverPath.GetString() +
                    " produced a singular hierarchy delta; constraint passed "
                    "through");
                return _Commit::PassedThrough;
            }
            const RigExecPointFrame frame =
                RigExecMatrixToPoints(current.points, delta);
            if (!_Usable(frame)) {
                pose->diagnostics.push_back(
                    moverPath.GetString() +
                    " produced an invalid descendant frame for " +
                    B.paths[j].GetString() + "; constraint passed through");
                return _Commit::PassedThrough;
            }
            propagated.emplace_back(j, frame);
        }
        for (const auto &[slot, frame] : candidates) {
            B.fin[slot] = frame;
            if (solverOutput) B.base[slot] = frame;
        }
        for (const auto &[slot, frame] : propagated) {
            B.fin[slot] = frame;
            if (solverOutput) B.base[slot] = frame;
        }
        return _Commit::Applied;
    };

    // ---- the interleaved solver/constraint walk ----------------------------
    std::set<SdfPath> fallbackJoints;
    std::set<size_t> visitedSolverLevels;
    for (const _Impl::_Step &step : B.steps) {
        if (step.solverBatch) {
            RIGEXEC_PROFILE_SCOPE_CAT(
                E._profiler, "SolverBatch L" + std::to_string(step.level),
                "pose");
            candidates.clear();
            for (int si : step.batchSolvers) {
                _Impl::_Solver &s = B.solvers[si];
                RigExecPointFrameArray &aggregate = B.aggregates[si];
                aggregate.frames.clear();
                aggregate.rests.clear();
                if (s.type == "RigExecFkChain") {
                    std::vector<RigExecFkChainElement> elements(
                        s.controls.size());
                    for (size_t k = 0; k < s.controls.size(); ++k) {
                        elements[k].restPoints = s.controlRests[k];
                        elements[k].posePoints =
                            s.controls[k] >= 0
                                ? B.fin[s.controls[k]].points
                                : RigExecIdentityLandmarks();
                        elements[k].parentIndex =
                            s.parentRelative ? -1 : int(k) - 1;
                    }
                    aggregate.frames = RigExecSolveFkChain(elements);
                    aggregate.rests = s.controlRests;
                } else if (s.type == "RigExecTwoBoneIk") {
                    RigExecTwoBoneIkParams params = s.ikParams;
                    if (live(s.bend) || live(s.stretch) ||
                        live(s.softness) || live(s.upperOffset) ||
                        live(s.lowerOffset)) {
                        params.preferredBendRadians = rd(s.bend);
                        params.stretch = rd(s.stretch);
                        params.softness = rd(s.softness);
                        params.upperLength =
                            s.upperLengthBase + rd(s.upperOffset);
                        params.lowerLength =
                            s.lowerLengthBase + rd(s.lowerOffset);
                    }
                    const auto frames = RigExecSolveTwoBoneIk(
                        B.fin[s.root], B.fin[s.end], B.fin[s.pole], s.ikRests,
                        params);
                    aggregate.frames.assign(frames.begin(), frames.end());
                    aggregate.rests.assign(s.ikRests.begin(), s.ikRests.end());
                } else if (s.type == "RigExecBlendPointFrames") {
                    const RigExecPointFrameArray &a = B.aggregates[s.inA];
                    const RigExecPointFrameArray &b = B.aggregates[s.inB];
                    const size_t n = a.GetSize();
                    // Clamp to [0, 1]: a blend weight outside the unit
                    // interval extrapolates past both inputs.
                    const double w = std::min(
                        std::max(double(rd(s.blendWeight)), 0.0),
                        1.0);
                    if (n == b.GetSize() && a.rests.size() == n) {
                        aggregate.frames.reserve(n);
                        aggregate.rests.reserve(n);
                        for (size_t k = 0; k < n; ++k) {
                            aggregate.frames.push_back(RigExecBlendFrames(
                                a.frames[k], b.frames[k], a.rests[k], w,
                                RigExecRotationBlend::ShortestArc,
                                s.scaleMode));
                            aggregate.rests.push_back(a.rests[k]);
                        }
                    }
                } else if (s.type == "RigExecSplineIk") {
                    RigExecSplineIkParams params = s.splineParams;
                    if (s.splineParamsVary || live(s.preserveVolume) ||
                        live(s.midFollowWeight) || live(s.roll) ||
                        live(s.twist) || live(s.minLengthRatio)) {
                        params.preserveVolume = rd(s.preserveVolume);
                        params.midFollowWeight =
                            rd(s.midFollowWeight);
                        params.roll =
                            GfDegreesToRadians(rd(s.roll));
                        params.twist =
                            GfDegreesToRadians(rd(s.twist));
                        params.minLengthRatio =
                            rd(s.minLengthRatio);
                    }
                    RigExecSplineIkControls controls;
                    controls.root = B.fin[s.root];
                    controls.mid = B.fin[s.mid];
                    controls.end = B.fin[s.end];
                    RigExecSplineIkResult solved;
                    RigExecSolveSplineIk(s.splineRest, controls, params,
                                         &solved);
                    if (solved.joints.size() == s.splineCount) {
                        aggregate.frames.reserve(s.splineCount);
                        for (const auto &joint : solved.joints) {
                            aggregate.frames.push_back(joint.frame);
                        }
                        aggregate.rests = s.splineJointRests;
                    }
                }
                for (const auto &[slot, element] : s.outputs) {
                    if (element < 0 ||
                        size_t(element) >= aggregate.GetSize()) {
                        fallbackJoints.insert(B.paths[slot]);
                        continue;
                    }
                    candidates[slot] =
                        RigExecExtractElementFrame(&aggregate, size_t(element));
                }
            }
            if (visitedSolverLevels.insert(step.level).second) {
                ++pose->solverOverrideRounds;
            }
            pose->solverEvaluations += step.batchSolvers.size();
            // A failed commit is a pass-through, exactly as in the dynamic
            // walk, which ignores the result here and carries on.
            if (!candidates.empty() &&
                commit(SdfPath(), step.propagate, true) == _Commit::Bail) {
                return false;
            }
            continue;
        }

        const _Impl::_Constraint &c = B.constraints[step.index];
        RIGEXEC_PROFILE_SCOPE_CAT(
            E._profiler, c.type.GetString() + " " + c.path.GetName(), "pose");
        if (!rd(c.enabled)) {
            continue;
        }
        const double weight = rd(c.defaultWeight);
        if (!std::isfinite(weight) || weight < 0.0 || weight > 1.0) {
            pose->diagnostics.push_back(
                c.path.GetString() +
                " has inputs:defaultWeight outside finite [0, 1]; "
                "constraint passed through");
            continue;
        }
        if (weight <= 0.0) {
            // A zero envelope is an exact dormant pass-through, decided
            // before any source is resolved so a malformed disconnected
            // input cannot make a disabled constraint fail.
            continue;
        }
        std::vector<RigExecConstraintSource> sources(c.sources.size());
        bool sourcesReady = true;
        for (size_t k = 0; k < c.sources.size(); ++k) {
            const RigExecPointFrame &frame = B.fin[c.sources[k]];
            if (!frame.IsValid()) {
                pose->diagnostics.push_back(
                    c.path.GetString() + " could not resolve source " +
                    B.paths[c.sources[k]].GetString());
                sourcesReady = false;
                break;
            }
            sources[k].frame = frame;
            sources[k].normalizedWeight = c.sourceWeights[k].constant;
            sources[k].translationOffset = c.translationOffsets[k];
            sources[k].rotationOffsetDegrees = c.rotationOffsets[k];
        }
        if (!sourcesReady) {
            pose->diagnostics.push_back(
                c.path.GetString() +
                " has unusable constraint inputs; constraint passed through");
            continue;
        }
        const RigExecPointFrame input = B.fin[c.target];
        RigExecPointFrame candidate = input;
        bool candidateReady = true;
        RigExecConstraintAxisMask affect;
        affect.x = rd(c.affectX);
        affect.y = rd(c.affectY);
        affect.z = rd(c.affectZ);
        if (c.type == "RigExecPositionConstraint") {
            RigExecPositionConstraintParams params;
            params.offset = rd(c.offset);
            params.affect = affect;
            params.weight = weight;
            candidate = RigExecApplyPositionConstraint(input, sources, params);
        } else if (c.type == "RigExecRotationConstraint") {
            RigExecRotationConstraintParams params;
            params.offsetDegrees = rd(c.offset);
            params.affect = affect;
            params.rotationOrder = c.order;
            params.weight = weight;
            candidate = RigExecApplyRotationConstraint(input, sources, params);
        } else if (c.type == "RigExecScaleConstraint") {
            RigExecScaleConstraintParams params;
            params.offset = rd(c.offset);
            params.affect = affect;
            params.weight = weight;
            candidate = RigExecApplyScaleConstraint(input, sources, params);
        } else if (c.type == "RigExecParentConstraint") {
            RigExecParentConstraintParams params;
            params.translationAxes.x = rd(c.tX);
            params.translationAxes.y = rd(c.tY);
            params.translationAxes.z = rd(c.tZ);
            params.rotationAxes.x = rd(c.rX);
            params.rotationAxes.y = rd(c.rY);
            params.rotationAxes.z = rd(c.rZ);
            params.scaleAxes.x = rd(c.sX);
            params.scaleAxes.y = rd(c.sY);
            params.scaleAxes.z = rd(c.sZ);
            params.rotationOrder = c.order;
            params.weight = weight;
            candidate = RigExecApplyParentConstraint(input, sources, params);
        } else {
            // Aim: the same weighted source set reduced to the target point
            // FBX's AimAtObjects contract specifies.
            GfVec3d target(0);
            double total = 0;
            for (const RigExecConstraintSource &source : sources) {
                if (!std::isfinite(source.normalizedWeight) ||
                    source.normalizedWeight < 0) {
                    pose->diagnostics.push_back(
                        c.path.GetString() +
                        " has an invalid source weight; constraint passed "
                        "through");
                    candidateReady = false;
                    break;
                }
                target += source.frame.Origin() * source.normalizedWeight;
                total += source.normalizedWeight;
            }
            if (candidateReady && total > 0) {
                target /= total;
                RigExecAimConstraintParams params;
                params.localAimVector = c.aimVectorAuthored
                                            ? rd(c.aimVector)
                                            : c.aimAxisFallback;
                params.localUpVector = rd(c.upVector);
                params.rotationOffsetDegrees = rd(c.rotationOffset);
                params.affectRotation = affect;
                params.rotationOrder = c.order;
                params.weight = weight;
                params.preserveInputUp = c.preserveInputUp;
                const GfVec3d authoredWorldUp = rd(c.worldUpVector);
                if (c.worldUpType == "sceneUp") {
                    params.worldUpDirection = c.sceneUp;
                } else if (c.worldUpType == "vector") {
                    params.worldUpDirection = authoredWorldUp;
                } else if (c.worldUpType == "objectUp") {
                    // FBX ObjectUp with no reference object uses the world
                    // origin as the object point.
                    params.worldUpDirection =
                        c.worldUpObjectNamed
                            ? GfVec3d(B.fin[c.worldUpObject].Origin() -
                                      input.Origin())
                            : GfVec3d(-input.Origin());
                } else if (c.worldUpType == "objectRotationUp") {
                    if (!c.worldUpObjectNamed) {
                        params.worldUpDirection = authoredWorldUp;
                    } else {
                        GfMatrix4d up(1.0);
                        if (!RigExecPointsToMatrix(
                                RigExecIdentityLandmarks(),
                                B.fin[c.worldUpObject].points, &up)) {
                            pose->diagnostics.push_back(
                                c.path.GetString() +
                                " has a degenerate world-up object");
                            candidateReady = false;
                        } else {
                            params.worldUpDirection =
                                up.ExtractRotation().TransformDir(
                                    authoredWorldUp);
                        }
                    }
                }
                if (candidateReady) {
                    candidate =
                        RigExecApplyAimConstraint(input, target, params);
                }
            }
        }
        if (candidateReady) {
            candidates.clear();
            candidates[c.target] = candidate;
            if (commit(c.path, step.propagate, false) == _Commit::Bail) {
                return false;
            }
        }
    }

    // An incomplete solver is an authoring gap, not a silent one.
    for (const SdfPath &jointPath : fallbackJoints) {
        const auto binding = E._jointSolverBinding.find(jointPath);
        const std::string solver =
            binding != E._jointSolverBinding.end()
                ? binding->second.first.GetString()
                : std::string("<unknown>");
        const int element = binding != E._jointSolverBinding.end()
                                ? binding->second.second
                                : -1;
        pose->diagnostics.push_back(
            "solver " + solver + " published no element " +
            std::to_string(element) + " for joint " + jointPath.GetString() +
            "; joint fell back to its rest chain");
    }

    // ---- rest->pose matrices ------------------------------------------------
    // What computeMatrix publishes, over dense slots: the whole
    // AuthoritativeSnapshot request is a re-derivation of values the walk
    // above already holds.
    std::vector<GfMatrix4d> finalMatrix(size_t(N), GfMatrix4d(1.0));
    std::vector<GfMatrix4d> baseMatrix(size_t(N), GfMatrix4d(1.0));
    std::vector<char> haveFinal(size_t(N), 0), haveBase(size_t(N), 0);
    auto finalMatrixOf = [&](int slot) -> const GfMatrix4d & {
        if (!haveFinal[size_t(slot)]) {
            if (_Usable(B.restFrames[slot]) && _Usable(B.fin[slot])) {
                RigExecPointsToMatrix(B.restPts[slot], B.fin[slot].points,
                                      &finalMatrix[size_t(slot)]);
            }
            haveFinal[size_t(slot)] = 1;
        }
        return finalMatrix[size_t(slot)];
    };
    auto baseMatrixOf = [&](int slot) -> const GfMatrix4d & {
        if (!haveBase[size_t(slot)]) {
            if (_Usable(B.restFrames[slot]) && _Usable(B.base[slot])) {
                RigExecPointsToMatrix(B.restPts[slot], B.base[slot].points,
                                      &baseMatrix[size_t(slot)]);
            }
            haveBase[size_t(slot)] = 1;
        }
        return baseMatrix[size_t(slot)];
    };

    {
        RIGEXEC_PROFILE_SCOPE_CAT(E._profiler, "BakedMatrices", "baked");
        for (size_t k = 0; k < B.jointPaths.size(); ++k) {
            const int slot = B.jointSlots[k];
            const RigExecPointFrame &baseFrame = B.base[slot];
            const RigExecPointFrame &finalFrame = B.fin[slot];
            pose->jointFramesBase[B.jointPaths[k]] = baseFrame;
            pose->jointFramesFinal[B.jointPaths[k]] = finalFrame;
            // The point frame is the status bearer; publishing an identity
            // matrix for a degenerate frame would let a matrix-only consumer
            // deform with a plausible-but-wrong transform.
            if (finalFrame.IsValid() && !finalFrame.IsDegenerate()) {
                if (!_Usable(B.restFrames[slot]) || !_Usable(finalFrame)) {
                    return false;  // the dynamic fallback needs exec
                }
                pose->jointMatricesFinal[B.jointPaths[k]] =
                    finalMatrixOf(slot);
            } else {
                pose->diagnostics.push_back(
                    "joint " + B.jointPaths[k].GetString() +
                    " has a degenerate final frame; matrix omitted");
            }
        }
        // Most controls are animator inputs and publish their base frame; a
        // control a constraint names publishes the revised one. Both are the
        // same slot here, because the walk wrote the revision into it.
        for (size_t k = 0; k < B.controlPaths.size(); ++k) {
            pose->controlFrames[B.controlPaths[k]] = B.fin[B.controlSlots[k]];
        }
    }
    // Observational solver guides. The dynamic path re-evaluates them through
    // a second exec request whose per-solver override IS the aggregate the
    // walk produced, so the published array is that aggregate either way --
    // and it publishes nothing at all when the consumer disabled the guides
    // or the guide request never prepared, which this mirrors so a parity
    // check compares like with like.
    if (E._guideTaps && E._solverGuidesEnabled) {
        for (const auto &[solverPath, si] : B.solverArrays) {
            pose->solverFrames[solverPath] = B.aggregates[si].frames;
        }
    }

    // Three published domains have no baked counterpart because bakeability
    // rules out everything that fills them, so leaving them empty is what
    // agrees with the dynamic path rather than a gap in the publication:
    // providerXforms/providerBaseXforms come only from _xformDerivedProviders
    // ("constraint target is a plain Xformable"), weightFrames only from
    // volume weight objects, weightFields only from a mover's weight object.
    // solverOverridesConverged stays true for the same kind of reason: it is
    // cleared only by an incomplete exec snapshot, and there is no exec here.

    // Property-domain results, in the same map as the point chains: a
    // consumer tells them apart by the type the VtValue holds.
    for (const auto &[target, value] : propertyResults) {
        pose->movedProperties[target] = value;
    }

    // ---- geometry ------------------------------------------------------------
    // The packet is assembled by the same RigExecAssembleParameters the
    // dynamic path calls and the kernel is the same shared kernel; only the
    // VdfNetwork around them is baked away, so the accounting it performed --
    // a revision runs when its inputs changed, and not otherwise -- is
    // performed here instead.
    size_t graphChainsBuilt = 0, graphRevisionsBuilt = 0;
    // Building the geometry state is reported once per node and once per
    // schedule, not once per program: a program rebuilt over state that
    // survived (AdoptGeometryStateFrom) has nothing to report, which is what
    // the dynamic path says about the same edit. Counted inside the walk
    // below, because a chain can lose its state mid-walk -- a point count
    // that moved with time -- and that is a node the dynamic path adds again
    // too.
    const auto accountForChain = [&pose](_Impl::_GeomChain *chain) {
        for (auto &revision : chain->revisions) {
            if (revision.created) {
                ++pose->moverGraphRevisionsCreated;
                revision.created = false;
            }
        }
        if (chain->scheduleDirty && !chain->revisions.empty()) {
            ++pose->moverGraphSchedulesBuilt;
        }
        chain->scheduleDirty = false;
    };
    auto assemble = [&](_Impl::_GeomRevision &revision,
                        const std::vector<GfVec3f> &basePoints) {
        RigExecProviderValues values;
        values.resolved = &R;
        GfMatrix4d transform(1.0);
        if (revision.transformSlot >= 0) {
            transform = revision.finalPhase
                            ? finalMatrixOf(revision.transformSlot)
                            : baseMatrixOf(revision.transformSlot);
            values.transform = &transform;
        }
        B.influenceScratch.clear();
        if (!revision.influenceSlots.empty()) {
            B.influenceScratch.reserve(revision.influenceSlots.size());
            for (int slot : revision.influenceSlots) {
                B.influenceScratch.push_back(revision.finalPhase
                                                 ? finalMatrixOf(slot)
                                                 : baseMatrixOf(slot));
            }
            values.influenceTransforms = &B.influenceScratch;
        }
        values.basePoints = basePoints;
        // Epoch-fixed layouts resolve through the evaluator's cache, exactly
        // as the dynamic path's assembly does -- the same cache, so the two
        // paths cannot even hold different arrays.
        if (revision.skinTopologyFixed) {
            values.skinTopologyCache = &E._skinTopologies;
        }
        RIGEXEC_PROFILE_SCOPE_CAT(
            E._profiler, "Assemble " + revision.moverPath.GetName(),
            "geometry");
        return RigExecAssembleParameters(revision.moverPrim, revision.op,
                                         revision.binding, values, time);
    };
    // The same fast path _RunScratchKernel takes, through the same predicate
    // -- one definition in moverGraph.h, so the two loops cannot disagree
    // about when skipping the blend is safe.
    auto fullStrengthEnvelope = [](const RigExecWeightPacket &w) {
        return RigExecEnvelopeIsFullStrength(w);
    };
    // The envelope is applied exactly once, against the preceding revision --
    // _RunScratchKernel / _ComputeRecomputed in moverGraph.cpp.
    auto blendEnvelope = [](const RigExecMoverParameters &parameters,
                            const std::vector<GfVec3f> &preceding,
                            std::vector<GfVec3f> *result) {
        if (result->size() != preceding.size()) return false;
        std::vector<float> envelope;
        if (!parameters.weights.ResolveAll(result->size(), &envelope)) {
            return false;
        }
        for (size_t i = 0; i < result->size(); ++i) {
            (*result)[i] =
                RigExecBlendEnvelope(preceding[i], (*result)[i], envelope[i]);
        }
        return true;
    };

    for (auto &chain : B.chains) {
        RIGEXEC_PROFILE_SCOPE_CAT(
            E._profiler, "Chain " + chain.target.GetString(), "geometry");
        VtVec3fArray basePoints;
        if (!chain.baseQuery.IsValid() ||
            !chain.baseQuery.Get(&basePoints, time)) {
            continue;
        }
        if (chain.haveResult && basePoints.size() != chain.lastBase.size()) {
            // A time-varying point count replaces THIS target's graph in the
            // dynamic path and leaves every other target's standing. Same
            // here: the cached run describes a different mesh, so it is
            // dropped and the chain's nodes are reported as created -- which
            // is what the dynamic walk reports for the nodes it has to add
            // again. Handing the whole generation back instead would degrade
            // every other chain of the rig for one mesh whose vertex count
            // is keyed.
            chain.haveResult = false;
            chain.result = VtVec3fArray();
            chain.scheduleDirty = true;
            for (_Impl::_GeomRevision &revision : chain.revisions) {
                revision.created = true;
                revision.ran = false;
                revision.output.clear();
                revision.lastParameters = RigExecMoverParameters();
                revision.lastStatus = RigExecMoverStatus();
            }
        }
        accountForChain(&chain);
        bool dirty = !chain.haveResult || basePoints != chain.lastBase;
        chain.lastBase = basePoints;
        std::vector<GfVec3f> current(basePoints.begin(), basePoints.end());
        for (_Impl::_GeomRevision &revision : chain.revisions) {
            const RigExecMoverParameters parameters =
                assemble(revision, current);
            if (parameters.enabled && !parameters.valid) {
                float scalar = 1.0f;
                if (const UsdAttribute a = revision.moverPrim.GetAttribute(
                        TfToken("inputs:defaultWeight"))) {
                    R.GetAttribute(a, time, &scalar);
                }
                if (!std::isfinite(scalar) || scalar < 0.0f || scalar > 1.0f) {
                    pose->diagnostics.push_back(
                        "MoverFailed " + revision.moverPath.GetString() +
                        ": inputs:defaultWeight must be finite and in "
                        "[0, 1]; revision passed through");
                }
            }
            const RigExecMoverStatus status =
                RigExecStatusForParameters(parameters, revision.moverPath);
            ++graphRevisionsBuilt;
            if (dirty || !revision.ran ||
                parameters != revision.lastParameters ||
                status != revision.lastStatus) {
                ++pose->moverGraphRevisionsExecuted;
                const bool fullStrength =
                    fullStrengthEnvelope(parameters.weights);
                std::vector<GfVec3f> scratch = current;
                const size_t precedingSize = scratch.size();
                std::vector<GfVec3f> preceding;
                if (!fullStrength) {
                    preceding = scratch;
                }
                // The same dispatch _RevisionNode::Compute performs, over
                // the same kernels: skin blends its finished result against
                // the preceding revision, matrix folds the envelope into the
                // movement itself.
                bool applied = status.AllowsApply() && parameters.valid;
                if (applied && parameters.kind == "skin") {
                    applied =
                        RigExecApplySkinKernel(parameters, &scratch) &&
                        (fullStrength
                             ? scratch.size() == precedingSize
                             : blendEnvelope(parameters, preceding, &scratch));
                } else if (applied && parameters.kind == "matrix") {
                    applied = RigExecApplyMatrixKernel(parameters, &scratch);
                } else {
                    applied = false;
                }
                revision.resultStatus = status.state;
                if (applied) {
                    current.swap(scratch);
                } else if (status.AllowsApply()) {
                    revision.resultStatus = TfToken("moverFailed");
                }
                revision.output = current;
                revision.lastParameters = parameters;
                revision.lastStatus = status;
                revision.ran = true;
                dirty = true;
            } else {
                current = revision.output;
            }
        }
        for (const _Impl::_GeomRevision &revision : chain.revisions) {
            if (revision.resultStatus == "moverFailed") {
                pose->diagnostics.push_back(
                    "MoverFailed " + revision.moverPath.GetString() +
                    ": execution rejected its inputs; revision passed "
                    "through");
            }
        }
        chain.result = VtVec3fArray(current.begin(), current.end());
        chain.haveResult = true;
        pose->movedProperties[chain.target] = VtValue(chain.result);
        ++graphChainsBuilt;

        // Derived maintenance reads this chain's FINAL points, which is why
        // it runs here rather than as another revision of the chain.
        for (auto &derived : chain.derived) {
            RIGEXEC_PROFILE_SCOPE_CAT(
                E._profiler, "Derived " + derived.target.GetString(),
                "geometry");
            VtVec3fArray derivedBase;
            if (!derived.baseQuery.IsValid() ||
                !derived.baseQuery.Get(&derivedBase, time)) {
                continue;
            }
            if (derived.haveResult &&
                derivedBase.size() != derived.lastBase.size()) {
                // As above, and for the same reason: the dynamic walk
                // replaces this derived target's graph alone.
                derived.haveResult = false;
                derived.result = VtVec3fArray();
                derived.revision.created = true;
                derived.revision.ran = false;
                derived.revision.output.clear();
                derived.revision.lastParameters = RigExecMoverParameters();
                derived.revision.lastStatus = RigExecMoverStatus();
            }
            // A derived target is one node with one schedule of its own,
            // created together, exactly as the dynamic walk creates them.
            if (derived.revision.created) {
                ++pose->moverGraphRevisionsCreated;
                ++pose->moverGraphSchedulesBuilt;
                derived.revision.created = false;
            }
            const bool derivedDirty =
                !derived.haveResult || derivedBase != derived.lastBase;
            derived.lastBase = derivedBase;
            const RigExecMoverParameters parameters =
                assemble(derived.revision, current);
            const RigExecMoverStatus status = RigExecStatusForParameters(
                parameters, derived.revision.moverPath);
            ++graphRevisionsBuilt;
            if (derivedDirty || !derived.revision.ran ||
                parameters != derived.revision.lastParameters ||
                status != derived.revision.lastStatus) {
                ++pose->moverGraphRevisionsExecuted;
                const std::vector<GfVec3f> preceding(derivedBase.begin(),
                                                     derivedBase.end());
                std::vector<GfVec3f> values;
                const bool wanted =
                    status.AllowsApply() && parameters.valid &&
                    parameters.kind ==
                        (derived.revision.op ==
                                 RigExecRevisionOp::RecomputeExtent
                             ? "recomputeExtent"
                             : "recomputeNormals");
                if (wanted) {
                    values = derived.revision.op ==
                                     RigExecRevisionOp::RecomputeExtent
                                 ? RigExecComputeExtent(parameters.auxPoints,
                                                        parameters.widths)
                                 : RigExecComputeVertexNormals(
                                       parameters.auxPoints,
                                       parameters.topologyCounts,
                                       parameters.topologyIndices);
                }
                bool applied =
                    wanted && !values.empty() &&
                    (derived.revision.op !=
                         RigExecRevisionOp::RecomputeExtent ||
                     values.size() == 2) &&
                    // The derived property keeps its authored cardinality: an
                    // in-place write cannot resize it.
                    values.size() == preceding.size() &&
                    blendEnvelope(parameters, preceding, &values);
                derived.revision.resultStatus = status.state;
                if (!applied) {
                    values = preceding;
                    if (status.AllowsApply()) {
                        derived.revision.resultStatus = TfToken("moverFailed");
                    }
                }
                derived.revision.output = values;
                derived.revision.lastParameters = parameters;
                derived.revision.lastStatus = status;
                derived.revision.ran = true;
            }
            if (derived.revision.resultStatus == "moverFailed") {
                pose->diagnostics.push_back(
                    "MoverFailed " + derived.target.GetString() +
                    ": derived geometry input/cardinality validation failed");
            }
            derived.result = VtVec3fArray(derived.revision.output.begin(),
                                          derived.revision.output.end());
            derived.haveResult = true;
            pose->movedProperties[derived.target] = VtValue(derived.result);
            ++graphChainsBuilt;
        }
    }

    // Whatever the Profile Mover binds reported; drained so a cached bind
    // stays silent on every later frame. Empty on a bakeable rig, drained
    // anyway so the two paths leave the evaluator in the same state.
    for (std::string &message : E._curvenetBindings.TakeDiagnostics()) {
        pose->diagnostics.push_back(std::move(message));
    }
    pose->diagnostics.push_back(
        "mover graph: " + std::to_string(graphChainsBuilt) +
        " chain(s), " + std::to_string(graphRevisionsBuilt) +
        " revision(s); " + std::to_string(pose->moverGraphRevisionsCreated) +
        " created, " + std::to_string(pose->moverGraphRevisionsExecuted) +
        " executed, " + std::to_string(pose->moverGraphSchedulesBuilt) +
        " schedule(s) built");
    pose->valid = true;
    return true;
}

}  // namespace rigExec
