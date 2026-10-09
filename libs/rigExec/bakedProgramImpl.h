// The baked program's implementation state, and the helpers its build and run
// halves share.
// bakedProgram.cpp is the only translation unit the evaluator declares a
// friend, so everything the frame path needs out of RigExecRigEvaluator is
// captured ONCE, at Build, into the pointers below; bakedPose.cpp and
// bakedGeometry.cpp then read the program rather than the evaluator. That is
// what lets the program be split by domain without widening the evaluator's
// friendship -- and the capture is also the list of exactly what a running
// frame reads from outside itself, which Phase 2's step graph needs stated
// rather than discovered.
// Nothing here is a second expression of semantics: the helpers are the ones
// bakedProgram.cpp already had, moved so more than one file can call them.
#ifndef RIGEXEC_BAKED_PROGRAM_IMPL_H
#define RIGEXEC_BAKED_PROGRAM_IMPL_H

#include "rigExecGraph/weightProgram.h"
#include "rigExecGraph/geometrySceneLowering.h"
#include "bakedProgram.h"
#include "rigExecGraph/autoClavicleGraph.h"
#include "frameExtraction.h"
#include "moverGraph.h"
#include "crossDomainInputs.h"
#include "scalarReferenceAdapter.h"
#include "parallel.h"
#include "profiler.h"
#include "solverKernels.h"
#include "tapSet.h"
#include "types.h"
#include "weightPackets.h"
#include "rigExecGraph/opGraph.h"
#include "rigExecGraph/opValues.h"
#include "rigExecGraph/providerProgram.h"
#include "rigExecGraph/solverProgram.h"
#include "rigExecGraph/constraintProgram.h"
#include "rigExecGraph/poseProgram.h"

#include "rigExecMath/avarScale.h"
#include "rigExecMath/dualQuat.h"
#include "rigExecMath/pointFrame.h"
#include "rigExecMath/propertyMath.h"
#include "rigExecMath/rbf.h"
#include "rigExecMath/singleChainIk.h"
#include "rigExecMath/solvers.h"
#include "rigExecMath/splineIk.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/tf/type.h"
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
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#if defined(_MSC_VER)
#include <intrin.h>
#endif

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

class RigExecRigEvaluator;
struct RigExecRigPose;
struct RigExecHeadLeafConstants;

// Shared helpers.

// Provider compose uses the same arithmetic as detached scene kernels.
inline GfMatrix4d
RigExecBakedComposeAvars(double tx,double ty,double tz,double sx,double sy,double sz,
    double rx,double ry,double rz,double spin,const TfToken &order)
{
    return RigExecComposePoseAvars(tx,ty,tz,sx,sy,sz,rx,ry,rz,spin,order);
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

inline bool
RigExecBakedUsable(const RigExecPointFrame &frame)
{
    return RigExecConstraintFrameUsable(frame);
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

// The input binding table.
// RigExecResolvedInputs::GetAttribute -- which is what both exec's
// AttributeValue accessor and every static read in the evaluator resolve
// through -- walks a single authored connection chain and takes the nearest
// readable upstream value, preferring anything this generation already
// resolved. That WALK is epoch-structural: connections cannot change without
// a resync, and a resync recompiles. Only the VALUES on it can move.
// So each input is classified once: if nothing on its walk is written by a
// property chain and nothing on it might vary with time, the value is an
// epoch constant and the frame path never touches USD for it. Otherwise it is
// read per frame -- through a retained UsdAttributeQuery when USD alone can
// answer, and through the generation's resolved inputs when a property chain
// is in the way, which is the same route the dynamic path takes.

template <class T>
struct RigExecBakedInput {
    T constant{};
    /// Direct source sampling is independent of temporal variation.
    bool sourceBacked = false;
    T sourceFallback{};
    UsdAttributeQuery query;  ///< set when USD alone answers, per frame
    UsdAttribute resolvedAttr;  ///< set when a property chain is on the walk
    UsdAttribute head;        ///< where the walk starts, for an override
    bool varying = false;
    /// Index into RigExecBakedProgramImpl::overridden, or -1 when the
    /// input was never registered (the rest ladder, which is folded rather
    /// than read).
    int overrideIndex = -1;
    /// Index into the program's leaf pool for T (RigExecBakedLeafPools), or
    /// -1 before RigExecBakedNumberLeaves numbered it. The leaf holds this
    /// input's value at the run's time; the fields above stay the binding.
    int leaf = -1;
    /// Index into RigExecBakedProgramImpl::readerWalks when the walk from
    /// `resolvedAttr` meets a chain target or a phased record consumer: the
    /// leaf is then resolved from head leaves, overrides and versions
    /// (RigExecBakedReadWalked), never through the resolved inputs. -1
    /// otherwise. Set at Build only.
    int walk = -1;
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
        // A keyed attribute with no default answers at every frame but not
        // at Default, so it is probed at its first key; otherwise the input
        // has no handle and every frame reads its fallback.
        const bool keyed = it->GetNumTimeSamples() > 0 || it->HasSpline();
        if (it->Get(&probe, time) ||
            (keyed && it->Get(&probe, UsdTimeCode::EarliestTime()))) {
            *selected = *it;
            return;
        }
    }
}

/// \p name pre-interned: the parallel resolve lanes pass file-scope tokens,
/// because TfToken(const char *) takes the token registry's lock.
template <class T>
inline RigExecBakedInput<T>
RigExecBakedBindInput(const UsdPrim &prim, const TfToken &name, T fallback,
                      UsdTimeCode time,
                      const std::set<SdfPath> &chainTargets,
                      SdfPathVector *walk = nullptr)
{
    RigExecBakedInput<T> input;
    input.constant = fallback;
    if (!prim) {
        return input;
    }
    const UsdAttribute attribute = prim.GetAttribute(name);
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

// What \p input resolves to at \p time. Only the leaf samplers call it (live:
// RigExecBakedSampleLeaves in the prologue; frozen: the worker prologue over
// its patched constants); step bodies and the input fill read the sampled
// leaf through RigExecBakedLeafRead.
template <class T>
inline T
RigExecBakedRead(const RigExecBakedInput<T> &input,
                 const RigExecResolvedInputs &resolved, UsdTimeCode time,
                 const std::vector<char> *overridden = nullptr,
                 const std::map<SdfPath, VtValue> *upstream = nullptr,
                 const std::vector<char> *upstreamOn = nullptr)
{
    RIGEXEC_PURITY_CHECK();
    // Detached jobs have already resolved overrides and upstream sources.
    if (input.sourceBacked && !input.head) return input.constant;
    T value = input.sourceBacked ? input.sourceFallback : input.constant;
    const auto flagged = [&input](const std::vector<char> *flags) {
        return input.overrideIndex >= 0 && flags &&
               size_t(input.overrideIndex) < flags->size() &&
               (*flags)[size_t(input.overrideIndex)];
    };
    // A held drag is placed by reading THE LONG WAY. GetAttribute is the same
    // walk exec's accessor performs and _resolvedInputs already carries the
    // override, so the answer is the dynamic one rather than a second
    // approximation of it -- and the pinned query, which knows nothing about
    // the drag, is bypassed for as long as it stands. An upstream value on
    // a hop is read the same way, through the upstream layer in front of
    // the stage.
    if (flagged(overridden) || flagged(upstreamOn)) {
        resolved.GetAttributeOverStageLayer(input.head, time, upstream,
                                            &value);
        return value;
    }
    if (input.sourceBacked) {
        // A live source without a usable query retains the argument fallback.
        if (!input.query.IsValid()) return input.sourceFallback;
        input.query.Get(&value, time);
        return value;
    }
    if (!input.varying) {
        return value;
    }
    if (input.resolvedAttr) {
        resolved.GetAttributeOverStageLayer(input.resolvedAttr, time,
                                            upstream, &value);
        return value;
    }
    if (input.query.IsValid()) {
        input.query.Get(&value, time);
    }
    return value;
}

// Sampled leaves.
// One leaf per binding frozenDetail::_ForEachPatchableInput visits, numbered
// in its order. RigExecBakedSampleLeaves fills them on the owning thread
// before the region from RigExecBakedRead, under the re-sample rules it
// states; everything after it reads the leaf. A leaf has no fold or varying
// bit of its own: it reads its binding's, so Build and
// RigExecProgramAvarPatch stay the only writers of what a binding is.

enum class RigExecBakedLeafType : uint8_t {
    Double, Float, Int, Bool, Token, Matrix4d, Vec3d, Vec3f
};

/// The pool a binding of type T samples into, and how the pool stores it
/// (bool as a byte, so a pool is a plain array).
template <class T>
struct RigExecBakedLeafTraits {
    static_assert(sizeof(T) == 0, "no leaf pool holds this input type");
};
template <>
struct RigExecBakedLeafTraits<double> {
    static constexpr RigExecBakedLeafType type = RigExecBakedLeafType::Double;
    using Stored = double;
};
template <>
struct RigExecBakedLeafTraits<float> {
    static constexpr RigExecBakedLeafType type = RigExecBakedLeafType::Float;
    using Stored = float;
};
template <>
struct RigExecBakedLeafTraits<int> {
    static constexpr RigExecBakedLeafType type = RigExecBakedLeafType::Int;
    using Stored = int;
};
template <>
struct RigExecBakedLeafTraits<bool> {
    static constexpr RigExecBakedLeafType type = RigExecBakedLeafType::Bool;
    using Stored = unsigned char;
};
template <>
struct RigExecBakedLeafTraits<TfToken> {
    static constexpr RigExecBakedLeafType type = RigExecBakedLeafType::Token;
    using Stored = TfToken;
};
template <>
struct RigExecBakedLeafTraits<GfMatrix4d> {
    static constexpr RigExecBakedLeafType type =
        RigExecBakedLeafType::Matrix4d;
    using Stored = GfMatrix4d;
};
template <>
struct RigExecBakedLeafTraits<GfVec3d> {
    static constexpr RigExecBakedLeafType type = RigExecBakedLeafType::Vec3d;
    using Stored = GfVec3d;
};
template <>
struct RigExecBakedLeafTraits<GfVec3f> {
    static constexpr RigExecBakedLeafType type = RigExecBakedLeafType::Vec3f;
    using Stored = GfVec3f;
};

template <class T>
struct RigExecBakedLeafPool {
    using Type = T;
    using Stored = typename RigExecBakedLeafTraits<T>::Stored;
    std::vector<Stored> value;
    /// 1 when this run's sample differs bitwise from the one before it; 0
    /// for a leaf this run did not re-sample.
    std::vector<char> changed;
    /// 1 when the next sample must re-read the leaf whatever else holds.
    std::vector<char> mustSample;
    /// The leaf id (RigExecBakedProgramImpl::leafRefs) of each entry.
    std::vector<uint32_t> id;
};

/// Where leaf id k lives: its pool and its index in that pool.
struct RigExecBakedLeafRef {
    RigExecBakedLeafType type = RigExecBakedLeafType::Double;
    uint32_t index = 0;
};

struct RigExecBakedLeafPools {
    std::tuple<RigExecBakedLeafPool<double>, RigExecBakedLeafPool<float>,
               RigExecBakedLeafPool<int>, RigExecBakedLeafPool<bool>,
               RigExecBakedLeafPool<TfToken>, RigExecBakedLeafPool<GfMatrix4d>,
               RigExecBakedLeafPool<GfVec3d>, RigExecBakedLeafPool<GfVec3f>>
        pools;

    template <class T>
    RigExecBakedLeafPool<T> &Of()
    {
        return std::get<RigExecBakedLeafPool<T>>(pools);
    }
    template <class T>
    const RigExecBakedLeafPool<T> &Of() const
    {
        return std::get<RigExecBakedLeafPool<T>>(pools);
    }
    template <class Fn>
    void ForEach(Fn &&fn)
    {
        std::apply([&fn](auto &...pool) { (fn(pool), ...); }, pools);
    }
};

/// The path leaves of one revision or weight object: the reads its packet
/// assembly makes (RigExecRevisionLeafDecl), sampled on the owning thread in
/// the geometry prologue (RigExecBakedSamplePathLeaves) and read by the step
/// bodies instead of the stage. No binding of _ForEachPatchableInput owns
/// them; RigExecBakedNumberLeaves numbers them after the visitor's leaves.
struct RigExecBakedPathLeaves {
    /// Build state.
    RigExecRevisionLeafDecl decl;
    /// Per key, the attribute at its path (invalid when none stands there),
    /// every path its read can reach, and whether the read can move with the
    /// time. Owning thread only: a frozen worker never touches them.
    std::vector<UsdAttribute> attributes;
    std::vector<std::vector<SdfPath>> hops;
    std::vector<char> varying;
    /// Per key, whether it reads topology (RigExecRevisionLeafRoleIsTopology,
    /// or a sparse blend sample's offsets and point indices): epoch state,
    /// whose authored edits rebuild the program. Build state. For such a key
    /// `overrideReached` says whether an interactive override stood on one
    /// of its hops at its last sample.
    std::vector<char> epoch;
    std::vector<char> overrideReached;
    /// Per key, its RigExecBakedProgramImpl::readerWalks index when its
    /// read goes through the resolved inputs and its hops meet a chain
    /// target or a record consumer, else -1 (or empty: none). Build state.
    std::vector<int> walks;
    std::vector<int> exactVersions; ///< per-key current property/record output, -1 for raw
    std::vector<int> exactRecordIndices; ///< record index or -1 for direct version
    std::vector<int> exactValueTypes; ///< producer PropertyChain::Arm or -1 for raw
    /// Per key, the value the last sample read, and whether that sample
    /// moved it (bitwise, as the binding leaves compare).
    std::vector<VtValue> values;
    mutable std::vector<VtValue> consumedValues; ///< per-owner body values, never sampled memo keys
    std::vector<char> changed;
    /// Per key, the content version the operation keys of a geometry owner
    /// carry for its value: two runs that read the key read the same
    /// version exactly when they read the same bytes. Written only through
    /// RigExecBakedSetPathLeaf, which compares a new value with `observed`,
    /// the value the last run to read the key saw (taken at the first write
    /// after that run, `observedRuns` naming it), and its `observedVersions`.
    std::vector<uint64_t> versions;
    std::vector<VtValue> observed;
    std::vector<uint64_t> observedVersions;
    std::vector<uint64_t> observedRuns;
    /// Per key, the next sample must re-read it whatever else holds: a value
    /// edit reached one of its paths, or a sample skipped it.
    std::vector<char> mustSample;
    /// What the last sample saw: whether there was one, its time, the
    /// program stamp, whether an interactive override stood, and the chain
    /// results' serial (RigExecBakedProgramImpl::pathLeafChainSerial).
    bool sampled = false;
    UsdTimeCode time = UsdTimeCode::Default();
    uint64_t stamp = 0;
    bool overrides = false;
    uint64_t chainSerial = 0;

    /// Whether the variance answers for key \p k (`varying[k]`, and for a
    /// skin's layout leaves GeomRevision::layoutFixed) may predate an edit:
    /// a notice the program could not route moved \p programStamp since the
    /// last sample, or a routed edit or a skipped sample marked the key.
    /// The next sample asks them again (RigExecBakedSamplePathLeaves,
    /// RigExecBakedSampleLayoutLeaves); a reader before it asks too
    /// (RigExecBakedLeafVaryingNow, RigExecBakedLayoutFixedNow). The bind
    /// answers them as well, so a table not yet sampled is current unless
    /// one of these holds.
    bool VarianceStale(uint64_t programStamp, size_t k) const
    {
        return stamp != programStamp ||
               (k < mustSample.size() && mustSample[k] != 0);
    }
    /// VarianceStale for any key.
    bool AnyVarianceStale(uint64_t programStamp) const
    {
        if (stamp != programStamp) {
            return true;
        }
        for (const char marked : mustSample) {
            if (marked != 0) {
                return true;
            }
        }
        return false;
    }

    /// Key \p k's value as \p T, or \p fallback when \p k is out of range or
    /// holds another type.
    template <class T>
    T Value(int k, const T &fallback) const
    {
        if (k < 0 || size_t(k) >= values.size() ||
            !values[size_t(k)].IsHolding<T>()) {
            return fallback;
        }
        return values[size_t(k)].UncheckedGet<T>();
    }
};

/// Which path-leaf owner a path leaf id names (ids from
/// RigExecBakedProgramImpl::leafRefs.size() on).
enum class RigExecBakedPathLeafOwner : uint8_t {
    Revision,
    Derived,
    Weight,
    WeightOracle,
    /// The layout leaves of a chain revision's or a derived target's
    /// SkinTopology op.
    RevisionLayout,
    DerivedLayout,
    Provider,
};

/// One path leaf: its owner (chain and revision, chain and derived target,
/// or weight object) and its key there.
struct RigExecBakedPathLeafRef {
    RigExecBakedPathLeafOwner owner = RigExecBakedPathLeafOwner::Revision;
    uint32_t a = 0;
    uint32_t b = 0;
    uint32_t key = 0;
};

/// The provider leaves' publication tables, built with the operation graph
/// from Build-only state and shared, immutable, by every clone. A provider
/// leaf's key reads its sample and the overlay RigExecBakedSpaceLeafOverlay
/// selects; these name, per leaf, every table that overlay reads, so a run
/// re-keys only leaves whose key can have moved and the overlay skips the
/// map finds.
struct RigExecBakedSpaceLeafIndex {
    /// Provider leaf k is operation value `first + k`. The program's other
    /// leaves, re-keyed on every run, fall below or above that range.
    RigExecValueId first = 0;
    std::vector<RigExecValueId> before, after;
    /// Per leaf: its `headOverrideSlots` slot, or -1, and the override
    /// numbers `overridableInputs` files at its path, in that order:
    /// numbers[numberBegin[k], numberBegin[k + 1]).
    std::vector<int> headSlot;
    std::vector<uint32_t> numberBegin;
    std::vector<int> numbers;
    /// The same facts the other way round: (path, leaf) sorted by path, for
    /// the routed and upstream maps, and (number, leaf) and (slot, leaf).
    std::vector<std::pair<SdfPath, uint32_t>> byPath;
    std::vector<std::pair<uint32_t, uint32_t>> byNumber, byHeadSlot;
    /// RIGEXEC_VERIFY_SPARSE_LEAVES, read at Build: every leaf a run skips
    /// is re-keyed anyway and verified unchanged.
    bool verify = false;
};

/// Bits of RigExecBakedProgramImpl::spaceLeafRekey.
enum : uint8_t {
    /// The sampler moved the leaf's value or blocked flag since its key.
    kRigExecSpaceLeafSampled = 1,
    /// Its last key had an overlay in reach or was not exact, so the same
    /// sample can key differently.
    kRigExecSpaceLeafHeld = 2,
    /// An overlay can stand on it in this publication.
    kRigExecSpaceLeafOverlaid = 4,
};

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
    case RigExecRevisionOp::DeltaMush: return "deltaMush";
    case RigExecRevisionOp::Wrinkle: return "wrinkle";
    case RigExecRevisionOp::Lattice: return "lattice";
    case RigExecRevisionOp::SurfaceProject: return "surfaceProject";
    case RigExecRevisionOp::Ribbon: return "ribbon";
    case RigExecRevisionOp::Wire: return "wire";
    case RigExecRevisionOp::EmitGuidePoints: return "emitGuidePoints";
    case RigExecRevisionOp::External: return "external";
    case RigExecRevisionOp::RecomputeNormals: return "recomputeNormals";
    case RigExecRevisionOp::RecomputeExtent: return "recomputeExtent";
    case RigExecRevisionOp::SurfaceProjector: return "surfaceProjector";
    case RigExecRevisionOp::ShaderDials: return "shaderDials";
    }
    return "unknown";
}

struct RigExecWeightOracleFacts {
    /// _VolumeWeightSamplesInFlight: a volume whose rigExec:weightTarget
    /// reads `preceding`.
    bool samplesInFlight = false;
    /// A volume not in flight: _ReadTargetPoints of rigExec:sampleSource,
    /// else of rigExec:weightTarget. False when neither reads.
    bool haveSamples = false;
    std::vector<GfVec3f> samples;
    /// A curve volume: _ReadTargetPoints of rigExec:curve, non-empty.
    bool haveCurve = false;
    std::vector<GfVec3f> curve;
    /// A plane volume's rigExec:planeAxis and rigExec:planeBounds as the
    /// oracle reads them: Get at the time, the fallback only when the
    /// attribute is absent. Empty for every other type.
    TfToken planeAxis, planeBounds;
    /// The first attribute whose answer these facts hold at one time while
    /// the oracle reads it at every evaluation time -- the points a volume
    /// not in flight samples, a curve volume's curve points, a plane's axis
    /// and bounds -- when it is animated (time samples, or a value that
    /// might vary); empty otherwise.
    SdfPath timeVarying;
    /// The oracle's whole error text when the read-phase check fails.
    std::string phaseError;
    /// The oracle's whole error text for the object's first structural
    /// failure: an unknown type, combine mode or range policy, a dynamic
    /// weight's operation, base, target, representation or sparse-support
    /// rule, or a static field carrying time samples or connections. Each
    /// precedes every per-run read of the object, so the port reports it
    /// on entering the object.
    std::string staticError;
};


// The program.

/// What a provider slot IS, which decides both what writes it and what the
/// walk may do with it.
///
/// The dynamic path keeps one frame map covering both families and tells them
/// apart by asking whether the path is in _nativeProviderPaths -- its
/// `hierarchicalProviders` set. The program answers the same question off a
/// dense table instead, because it asks it once per descendant per commit.
enum class RigExecBakedSlotKind {
    /// An exec-seeded RigExec provider: composed from avars, propagated to as
    /// a descendant, and the only kind a solver or the compose writes.
    FirstFramePose,
    /// A plain Xformable a constraint targets. The dynamic walk seeds it from
    /// the stage and revises it like any other target, but deliberately keeps
    /// it out of hierarchicalProviders, so it is never propagated TO -- while
    /// it can still be the closest revised ancestor a RigExec descendant
    /// rides.
    XformDerived,
};

// The step graph.
// A run is a dependency graph of steps over dense slots rather than one
// straight line. PROGRAM ORDER -- the order the straight line used -- is still
// the reference: a step's index is its position in it, every edge points
// forward, and the serial executor runs the steps in index order. That is what
// makes "byte-identical to the old Run" a property any topological order of
// this graph has, because floating-point results depend only on operand values
// and each step's arithmetic is fixed.
// A step declares the slot RANGES it reads and writes; the edges are computed
// from those declarations alone (bakedSchedule.cpp). Declared writes are an
// UPPER BOUND: a disabled constraint, a revision that did not execute and a
// commit that passed through all write fewer slots than they declared, and no
// executor may ever "write what was declared".

/// Which dense table a slot indexes.
///
/// The specification calls this a slot KIND. That word is already spent in
/// this header on RigExecBakedSlotKind, which says what a PROVIDER slot is
/// (FirstFramePose or XformDerived), and two similarly named enums in one header is
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
    PropertyResult,      ///< propertyValues[v]; written by PropertyRevision
    ChainBase,           ///< chains[c].lastBase; filled by the prologue
    RevisionPacket,      ///< one revision's assembled packet, status, executed
    RevisionTransforms,  ///< one revision's influence table
    RevisionOut,         ///< one revision's OWN output buffer
    RevisionDone,        ///< one revision's applied/status flags
    ChainDirty,          ///< the chain's sticky dirty bit as of one revision
    ChainPoints,         ///< the chain's published points
    DerivedOut,          ///< one derived target's output
    WeightPacket,        ///< one weight object's packet for this frame
    WeightFrames,        ///< where one volume weight is placed, by provider slot
    PoseWeight,          ///< poseWeights[k]: one pose interpolator's weights
    /// Reserved: the wire value of the retired phased-read store. Never
    /// declared; the step-graph validator refuses a step that names it.
    Snapshots,
    /// frameMatrix[k] and frameMatrixValid[k]: one RigExecBakedFrameRecord.
    /// Appended after Snapshots, so every earlier enumerator keeps its
    /// exported value.
    FrameMatrix,
    Rest = 27,
    Ladder = 28,
    SkinTopology = 29,
    WeightField = 30,
    WeightFramesBase = 31,
    SpaceValue = 32,
    SpaceLeaf = 33,
    DerivedBase = 34,
    ChainInput = 35,
    ConstraintInputs = 36,
    SwitchFrame = 37,
    RequiredStageFramesAdmission = 38,
};
/// Derived from the last enumerator rather than written out: an array
/// indexed by domain is how the edge sweep is written, and a count that
/// drifted from the enum is an out-of-bounds write with no symptom at Build.
inline constexpr size_t RigExecBakedSlotDomainCount =
    size_t(RigExecBakedSlotDomain::RequiredStageFramesAdmission) + 1;

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

// Head operations occupy the shared graph's prefix. Their leaves are sampled
// before their serial bodies; rest/layout sampling follows chain publication.

/// The type a head leaf holds and a walk reads.
enum class RigExecBakedHeadValueType : uint8_t {
    Bool, Float, Double, Vec3f, Matrix4d, Vec2fArray, Int, Token, Vec3d, Vec3fArray
};

/// One stage value a head op reads: an attribute's own value at the run's
/// time as one type, with no connection followed and no overlay consulted;
/// empty when the attribute has no value of that type then. Sampled on the
/// owning thread (RigExecBakedSampleHeadLeaves); leaf id
/// `leafRefs.size() + pathLeafRefs.size() + i` in `leafByPath`.
struct RigExecBakedHeadLeaf {
    SdfPath path;
    /// Invalid when no attribute stands at `path`. Owning thread only.
    UsdAttribute attribute;
    RigExecBakedHeadValueType type = RigExecBakedHeadValueType::Float;
    /// The attribute's value type is `type`. A typed read of another type
    /// has no value, so such a leaf stays empty and is never read.
    bool typeMatches = false;
    bool varying = false;
    VtValue value;
    /// This run's sample differs bitwise from the one before it.
    char changed = 0;
    /// The next sample re-reads it whatever else holds (a value edit
    /// reached `path`), and re-derives `varying`.
    char mustSample = 0;
    /// The key a frozen job samples it under, set at Build for a leaf whose
    /// type matches (`<prim>.frozenChainOwn:<name>` for a chain target's own
    /// value, `<prim>.frozenChainHop:<name>` otherwise); empty for a leaf
    /// that never holds a value. Never `path`: a reader binding whose head
    /// is a chain target samples the chain's result at that path, and
    /// first-wins lookup would serve one to the other.
    SdfPath frozenKey;
};

/// Calls \p fn on every head leaf a frozen job samples (those with a
/// `frozenKey`), in leaf order: the sampler's emission order and the
/// worker's patch order.
template <class Program, class Fn>
inline void
RigExecForEachHeadLeaf(Program &program, Fn &&fn)
{
    for (auto &leaf : program.headLeaves) {
        if (!leaf.frozenKey.IsEmpty()) {
            fn(leaf);
        }
    }
}

/// One attribute of a walk, and what can stand in the overlay there when a
/// head op reads it.
struct RigExecBakedWalkHop {
    SdfPath path;
    /// `headOverrideSlots`' slot of `path`.
    int overrideSlot = -1;
    /// An earlier chain whose target `path` is: its final version stands
    /// here while the chain is valid, in place of any override.
    int chain = -1;
    /// An earlier chain's phased record whose consumer `path` is.
    int record = -1;
    /// The head leaf holding `path`'s own value as the walk reads it; -1
    /// where the walk never falls back to it.
    int leaf = -1;
    int poseWeight = -1; ///< same-generation Float publication, revision oracle only
    int crossDomain = -1; ///< explicit S9 pose/geometry version read
};

/// One input of a property revision as _EvaluatePropertyChains reads it
/// (_PinnedRead, rigEvaluatorProperties.cpp), restated over head leaves,
/// overrides and versions by RigExecBakedResolveWalk (bakedProperties.cpp).
/// The hops are fixed at Build: a connection, a type or an attribute
/// appearing on them moves the epoch digest, which recompiles.
struct RigExecBakedWalk {
    enum class Flavour : uint8_t {
        Absent,     ///< no attribute: the read's fallback
        Pinned,     ///< unconnected: the overlay at its path, else its value
        Connected,  ///< RigExecResolvedInputs::GetAttribute's walk
    };
    Flavour flavour = Flavour::Absent;
    RigExecBakedHeadValueType type = RigExecBakedHeadValueType::Float;
    /// The hops read as `type`, in walk order. A float walk that meets a
    /// double hop ends `hops` there (that hop still answers a float overlay
    /// first) and goes on in `doubleHops`, GetAttribute's double recursion
    /// from that hop with its own cycle guard; a float walk whose head is a
    /// double has only `doubleHops`.
    std::vector<RigExecBakedWalkHop> hops, doubleHops;
};

/// A read after the head tier that RigExecResolvedInputs::GetAttribute
/// would answer over the published overlay, restated over head leaves,
/// overrides and versions: a chain-routed binding's walk, or a path leaf's
/// whose hops meet a chain target or a record consumer. Every chain and
/// record counts on its hops, since all have run. Bound at Build.
struct RigExecBakedReaderWalk {
    /// Always the Connected flavour: GetAttribute's walk from `head`, with
    /// the float-from-double recursion.
    RigExecBakedWalk walk;
    SdfPath head;
    /// The head's own value as the walk's type, for a path-leaf read that
    /// falls back to the raw attribute when the walk answers nothing; -1
    /// for a binding, which keeps its constant.
    int rawLeaf = -1;
    /// What the value depends on beside the time: the head leaves of its
    /// hops (and `rawLeaf`), their override slots, and the PropertyVersion
    /// ids it declares (each met chain's final version and each met
    /// record), sorted.
    std::vector<uint32_t> leaves, slots, versions;
    /// Declared final versions the walk reaches only past one of that
    /// chain's records, as (version id, record index): while the record
    /// does not stand aside it answers the walk (or, with the chain
    /// skipped, the target holds no version), so a move of the version
    /// alone moves nothing here (RigExecBakedStep::shadowedReads).
    std::vector<std::pair<uint32_t, uint32_t>> shadowed;
};

RigExecBakedReaderWalk RigExecBakedBuildOracleRead(
    RigExecBakedProgramImpl *, const UsdAttribute &, const std::vector<int> &);
float RigExecBakedReadOracleScalar(const RigExecBakedProgramImpl &,
    const RigExecBakedReaderWalk &, const std::vector<int> &, float);
bool RigExecBakedResolveWalkValue(const RigExecBakedProgramImpl &,
    const RigExecBakedWalk &,VtValue *);

/// One version's value, in the member the chain's arm computes.
struct RigExecBakedPropertyValue {
    float f = 0.0f;
    double d = 0.0;
    GfVec3f v{0.0f};
    GfMatrix4d m{1.0};
};

/// A property chain as the head tier runs it, bound at Build from the
/// evaluator's compiled chain the way _EvaluatePropertyChains binds it
/// (rigEvaluatorProperties.cpp).
struct RigExecBakedPropertyChain {
    /// The runChain arm the target's type name selects.
    enum class Arm : uint8_t { Float, Double, Matrix4d, Vec3f };
    SdfPath target;
    bool targetExists = false;
    SdfValueTypeName valueType;
    Arm arm = Arm::Float;
    /// The target's own value (no connection, no overlay) as the arm's type.
    int ownLeaf = -1;
    int targetSlot = -1;
    /// The PropertyVersion id of part 0; part k writes versionBase + k.
    uint32_t versionBase = 0;
    struct Revision {
        SdfPath mover;
        bool moverExists = false;
        /// rigExec:operation as _ReadOperation reads it (its own value at
        /// Default), parsed at Build. Structural: an edit rebuilds.
        bool opValid = false;
        RigExecPropertyOp op = RigExecPropertyOp::Add;
        /// rigExec:weightObject's first target; empty when none.
        SdfPath weightObject;
        int weightField = -1;
        RigExecBakedWalk enabled, defaultWeight, value, minimum, maximum,
            keys, tangents;
        /// inputs:tangents exists, which is what decides it is read.
        bool hasTangents = false;
        /// "diag <mover>", the head of every line the revision emits.
        std::string linePrefix;
    };
    std::vector<Revision> revisions;
    /// Its phased records, as indices into `propertyRecords`.
    std::vector<uint32_t> records;
    /// "property chain <target>", the head of every line the base emits.
    std::string linePrefix;
};

/// A phased record (RigExecPhasedConnection): its consumer reads the chain
/// after `applied` revisions, converted to `consumerType`, unless an
/// override stands on one of its hops.
struct RigExecBakedPropertyRecord {
    uint32_t chain = 0;
    SdfPath consumer;
    SdfValueTypeName consumerType;
    size_t applied = 0;
    /// The override slots of the consumer and every hop before the target.
    std::vector<int> hopSlots;
    /// Its PropertyVersion id.
    uint32_t id = 0;
};

/// One version of a chain's running points: 0 is the authored base, v > 0
/// what the fuse of the chain's revision v - 1 left.
struct RigExecBakedPointVersion {
    int chain = -1;
    int version = 0;
};

/// A phased point read, bound at Build to the versions the dynamic walk's
/// phased-read store would have answered from: the records of the revisions some phase
/// names (`snapshotAfter`) whose fuse -- or, for `final`, whose chain's
/// status step -- runs before the reader, matched to the phase as
/// the original AtPrim lookup matches them, newest first. `preceding` on
/// the reader's own chain is instead the one version entering the reader,
/// which the walk reads off the chain built so far. The reader
/// takes the first candidate whose chain read a base this run -- every
/// record of a chain is written exactly then -- and otherwise the tail: the
/// resolved input, with the "resolved to nothing" line when `diagnoseMiss`.
struct RigExecBakedPointsBinding {
    /// The chain target the reader reads, which the overlay sets.
    SdfPath input;
    /// As declared; the diagnostic names it.
    RigExecReadPhase phase;
    /// Versions per RigExecBakedPointVersion. For a `finalRead` the one
    /// candidate is the chain's published ChainPoints, at version = the
    /// chain's revision count.
    std::vector<RigExecBakedPointVersion> candidates;
    bool finalRead = false;
    /// `phases` reads other than `preceding`; never a blend sample.
    bool diagnoseMiss = false;
    /// The miss diagnostic, spelled at Build: a body may not ask SdfPath for
    /// text, which interns it under Sdf's table locks.
    std::string missDiagnostic;
    /// Dense over every binding of the program, for the test capture.
    int id = -1;
};

/// One provider frame as one writer of its stack -- a constraint or a
/// solver -- left it, for an AtPrim read phase on a transform that names that
/// writer. A record exists only for a pair (provider, writer) the compile
/// named (`_nativePhaseCheckpoints`), exactly as the walk records only named
/// pairs; its FrameMatrix step writes the matrix and a valid byte that is 0
/// exactly where the walk declines to record (RigExecBakedEvalFrameRecord).
struct RigExecBakedFrameRecord {
    /// The provider slot.
    int slot = -1;
    /// The writer's commit: an index into `commits` / `walkSteps`.
    int commit = -1;
    /// A constraint's record: the provider's position in its `targetSlots`.
    /// -1 for a solver's.
    int target = -1;
    /// A solver's record: the provider's position in the commit's `slots`,
    /// whose `present` byte says whether the solver published it this run.
    /// -1 for a constraint's.
    int position = -1;
    /// The `fin` entry the record reads: the commit's last version of the
    /// slot where it declares one, else the version it read. Bound after
    /// BindPoseVersions.
    uint32_t version = 0;
    /// The constraint or solver, which the read phase names.
    SdfPath mover;
};

/// Test-only: one point binding's answer as its reader took it this run
/// (RigExecBakedProgramTesting::CapturePointReads). `read` is false when the
/// reader did not run.
struct RigExecBakedPointCapture {
    bool read = false;
    bool bindingAnswered = false;
    VtVec3fArray bound;
};

/// The name of \p domain, for the schedule report.
const char *RigExecBakedSlotDomainName(RigExecBakedSlotDomain domain);

/// Whether the prologue, and not a step, fills \p domain: a read of one needs
/// no producer in the graph. Aggregate is not one: a blend reads its inputs'
/// aggregates from this run, so their Solve steps must precede it.
inline bool
RigExecBakedIsSourceDomain(RigExecBakedSlotDomain domain)
{
    return domain == RigExecBakedSlotDomain::SolverPoints ||
           domain == RigExecBakedSlotDomain::SpaceLeaf ||
           domain == RigExecBakedSlotDomain::DerivedBase ||
           domain == RigExecBakedSlotDomain::ChainInput ||
           domain == RigExecBakedSlotDomain::ConstraintInputs ||
           domain == RigExecBakedSlotDomain::RequiredStageFramesAdmission;
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
    /// Reserved: the wire value of a retired step. Never emitted.
    SnapshotFinals,
    PoseInterpolator, ///< one pose interpolator's weights, from the final pose
    VolumePlacements, ///< one volume weight's placement, from the walk
    WeightPacket,     ///< one weight object's packet, built once per frame
    InfluenceFold,    ///< one revision's influence table
    RevisionStatic,   ///< one revision's packet, status and executed decision
    RevisionChunk,    ///< one vertex range of one revision
    RevisionFuse,     ///< one revision's applied decision and chain dirty bit
    ChainStatus,      ///< one chain's status sweep and published points
    Derived,          ///< one derived target maintained from a chain
    /// One RigExecBakedFrameRecord. A pose step; appended after Derived so
    /// every earlier enumerator keeps its exported value.
    FrameMatrix,
    PropertyRevision = 19,
    RestCompose = 20,
    LadderCompose = 21,
    SkinTopology = 22,
    WeightField = 23,
    SpaceExpression = 24,
    AvarInputs = 26,
    SpaceCheckpoint = 27,
    ProviderRefresh = 28,
    ChainInputs = 25,
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
    case RigExecBakedStepKind::ChainInputs:
    case RigExecBakedStepKind::WeightField:
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
    bool isHead = false;
    std::vector<uint32_t> leaves, overrideSlots, bindingLeaves;
    bool varyingLeaves = false;
    bool alwaysRuns = false;
    std::vector<std::string> lines;
    RigExecBakedStepKind kind = RigExecBakedStepKind::ComposeSubtree;
    /// What this step is about, by kind:
    ///   ComposeSubtree                              index into composeGroups
    ///   Solve                                       index into solvers
    ///   SolverCommit/Constraint/CommitDelta/
    ///     PropagateChunk/CommitApply                index into commits
    ///   ProviderMatrix                              provider slot
    ///   SnapshotFinals                              never emitted
    ///   PoseInterpolator                            index into poseInterpolators
    ///   VolumePlacements                            provider slot (part 1)
    ///   WeightPacket                                index into weightObjects
    ///   InfluenceFold/RevisionStatic/
    ///     RevisionChunk/RevisionFuse                index into revisionIndex
    ///   ChainStatus                                 index into chains
    ///   Derived                                     index into derivedIndex
    ///   FrameMatrix                                 index into frameRecords
    int object = -1;
    /// The part of it, by kind: the vertex chunk of a RevisionChunk, the
    /// propagation-pair chunk of a PropagateChunk, 1 for a final-phase
    /// ProviderMatrix against 0 for a base-phase one, and 1 for the
    /// per-volume VolumePlacements form (the whole-map form, -1, is no
    /// longer emitted but keeps its wire meaning). -1 where unused.
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
    /// Declared authored prerequisites, separate from generated graph edges.
    /// Stable keys survive canonicalization and local SCC exclusion.
    std::vector<std::string> semanticPredecessorKeys;

    // A step is a pure function of its declared reads with three exceptions,
    // and every one of them is recorded here so that the dirty set can name
    // it rather than the executor having to guess (§7).
    /// The step reads source slots or completed head outputs, so it can be
    /// run before the dirty set is computed, every run, and compared by
    /// VALUE. Heads themselves never participate in this source pass.
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
    /// The override indices of the baked inputs this step reads, sorted and
    /// deduplicated. A step is dirty while an override stands on one of
    /// them, and for the one run after it is lifted.
    std::vector<int> overrideInputs;
    /// The reader walks (RigExecBakedProgramImpl::readerWalks) of the
    /// inputs and path leaves this step reads, sorted and deduplicated, and
    /// the head outputs it reads: the PropertyVersion ids those walks
    /// declare, then the Rest and Ladder slots its body indexes. Not
    /// exported; a version that moved dirties the step
    /// (RigExecBakedCones::headReaders), and so does a walk whose value
    /// moved (`walkReaders`) and a Rest or Ladder slot that moved
    /// (`restReaders`, `ladderReaders`).
    std::vector<int> readerWalks;
    std::vector<std::pair<RigExecBakedLeafType,uint32_t>> walkedBindingLeaves;
    /// The `reads` ids every walk of the step that declares them reads
    /// only past a record (RigExecBakedReaderWalk::shadowed), with those
    /// records: such an id seeds the step only while one of them stands
    /// aside.
    std::vector<std::pair<uint32_t, uint32_t>> shadowedReads;

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
    /// unless RIGEXEC_BAKED_SCHEDULE_CALIBRATE or RIGEXEC_BAKED_STEP_TIMING
    /// asked for a measurement, and written only by the thread that runs the
    /// step -- which is what "lock-free per-step timer" comes to.
    double measuredUs = 0;
    uint32_t measuredRuns = 0;
    /// Beside it, under the same requests: the summed memo of every run that
    /// evaluated this op's memo (one that then skipped the body included),
    /// how many did, and the summed value publication of the runs that ran
    /// the body. Folded from the stamps below by the executor's owner after
    /// the join.
    double measuredMemoUs = 0, measuredPublishUs = 0;
    uint32_t measuredMemoRuns = 0;

    /// This run's output, all of it per step so that nothing in a body
    /// touches shared state.
    std::vector<std::string> diagnostics;
    size_t maxDiagnostics = RigExecBakedMaxStepDiagnostics;
    RigExecBakedStepCounters counters;
    /// When the profiler is on: the step's interval, replayed into it by the
    /// epilogue in step order so the trace is deterministic.
    uint64_t startUs = 0, endUs = 0;
    /// Thread that ran this body, retained for profiler replay.
    std::thread::id runner;
    /// This run's op stamps in nanoseconds on the same steady clock, written
    /// only by the thread that runs the op: memo start and publication end
    /// while op timing, the profiler or a measurement is on; memo end and
    /// body end only for a measurement. The op holds its thread from
    /// memoStartNs to publishEndNs. Cleared with the body interval at the
    /// start of the run after any run that stamped (`runStamped`).
    uint64_t memoStartNs = 0, memoEndNs = 0, bodyEndNs = 0, publishEndNs = 0;
    /// What the step is called in the report and the trace, built once at
    /// Build so that neither costs a string per step per frame.
    std::string label;
    std::string descriptorKey; ///< owner/category/authored revision/subindex identity

    /// Empties this run's output. Called by the executor before the body, so
    /// that a step that is skipped keeps last run's lines for the epilogue
    /// to replay. The timestamps are NOT cleared here for that same reason:
    /// a skipped step never reaches this, so RigExecBakedClearRunStamps
    /// clears every step's interval at the start of the run instead.
    void BeginRun() {
        diagnostics.clear();
        counters.Clear();
    }

    /// Records that this run skipped the step (§7).
    ///
    /// The diagnostics stay, and so do the two counters that are PROGRAM
    /// CONSTANTS: a skipped step's lines describe state nothing changed, and
    /// how many chains and revisions the program holds is a property of the
    /// program rather than of a run (§4.2). What does not stay is the three
    /// counters that are OBSERVATIONS of this run -- a revision the frame
    /// did not execute did not execute, a node it did not report did not
    /// report its creation, a schedule it did not build did not build one.
    /// These observations are reported by the operations that do the work.
    void MarkSkipped() {
        counters.revisionsExecuted = 0;
        counters.revisionsCreated = 0;
        counters.schedulesBuilt = 0;
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

    /// When the cluster's last predecessor finished, when it started and
    /// when it ended, as RigExecProfiler::NowUs() reads. Recorded only while
    /// the schedule report or the profiler asks for them, so a production
    /// frame pays no clock reads for a table nobody prints.
    uint64_t readyUs = 0, startUs = 0, endUs = 0;
    /// The thread that ran it, stamped beside startUs. A cluster is the unit
    /// one task runs back to back (see above), so this is also the thread of
    /// every step in `members` -- which is what lets the epilogue put a
    /// step's interval on the row it really ran on, having been handed the
    /// step long after that thread moved on. Default-constructed after a
    /// serial run, which RecordOn reads as "the calling thread".
    std::thread::id runner;
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
    /// Every cluster once, each after all of its predecessors, derived from
    /// the cluster edges (RigExecBakedClusterTopologicalOrder) rather than
    /// trusted from the ids. RigExecBakedBuildCones fills it, which is where
    /// Build first needs it.
    std::vector<int> topologicalOrder;
    /// Whether the last run stamped the per-cluster times above. Only the
    /// parallel executor has clusters to time: a serial run walks the steps
    /// and never asks which cluster they are in, so its run report says so
    /// rather than printing a table of zeros that reads as "every cluster
    /// was free".
    bool lastRunTimed = false;
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
    /// Whether anything was added, which is the fixpoint test. Widths may
    /// differ: a narrower other reads as zero past its end (the
    /// RigExecIntersectClusterSets rule), and a wider one grows this set
    /// rather than dropping its high members.
    bool Union(const RigExecBakedClusterSet &other) {
        bool grew = false;
        if (words.size() < other.words.size()) {
            words.resize(other.words.size(), 0);
        }
        for (size_t w = 0; w < words.size(); ++w) {
            const uint64_t o =
                w < other.words.size() ? other.words[w] : 0;
            const uint64_t before = words[w];
            words[w] |= o;
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

/// The readers RigExecBakedCones::editRoute names per override index.
enum : uint8_t {
    kEditRouteStep = 1,
    kEditRouteAvar = 2,
    /// A ladder channel: the rest and ladder ops that read its leaf re-run,
    /// and their moved outputs seed their readers (RigExecBakedHeadSeeds).
    kEditRouteHead = 4,
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
    /// PropertyVersion id -> the steps that declare it in `reads`, and
    /// reader walk -> the steps that read it. What a moved version and a
    /// moved walk value dirty.
    std::vector<std::vector<int>> headReaders, walkReaders, fieldReaders;
    std::vector<int> fieldSteps;
    /// Provider slot -> the steps that declare its Rest, and its Ladder, in
    /// `reads`. What a moved rest or ladder output dirties.
    std::vector<std::vector<int>> restReaders, ladderReaders;
    /// Override index -> what re-reads an input of that number when the
    /// stage value under it moves: a step that lists it in its
    /// `overrideInputs` (kEditRouteStep), the prologue's avar table
    /// (kEditRouteAvar), or the rest and ladder ops (kEditRouteHead). Zero
    /// where nothing does -- a value read into the prologue and handed to a
    /// step no dirty set can name -- and an edit there bumps the program
    /// stamp instead of being routed (unified-program spec rules S2, S3).
    std::vector<uint8_t> editRoute;

    // What a live run decides with. A cluster is the unit a task runs, not
    // the unit that moved: one dirty step does not make its cluster-mates
    // dirty, because they read exactly what they read last run and versioned
    // storage (§3.1) left it where it was. So the live closure is taken over
    // `step.succs` and a run skips every step outside it, including the clean
    // members of a cluster it does dispatch.
    // The per-cluster tables above are KEPT beside these rather than derived
    // from them on demand: the output-affected index, the sparse frame-cache
    // planner and the frozen clone read them, and each wants a cluster
    // answer. Every entry below is the step-grain twin of the entry of the
    // same name above; a seed added to one family belongs in both.

    /// Steps that read outside the graph (dirty every run), and steps that
    /// are not geometry steps (dirty on a program's first run).
    RigExecBakedClusterSet alwaysSteps, poseSteps;
    /// Provider slot -> the compose step that reads its avars, or -1.
    std::vector<int> avarStep;
    /// Provider slot -> the OTHER compose steps that declare a read of its
    /// avars: a switched group recomposing an earlier version of an
    /// ancestor (RigExecBakedProgramImpl::SpaceSwitch::FrameVersion).
    /// Whatever dirties avarStep dirties these too. Empty on most rigs.
    std::vector<std::vector<int>> avarVersionSteps;
    std::vector<std::vector<int>> chainBaseSteps;
    std::vector<std::vector<int>> solverPointsSteps;
    std::vector<std::vector<int>> revisionSteps;
    std::vector<int> revisionStaticStep;
    std::vector<std::vector<int>> nativeSourceSteps;
    std::vector<std::vector<int>> deltaBaseSteps;
    std::vector<std::vector<int>> constraintArraySteps;
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
///
/// A solver STACK is two commits on one slot, never two producers inside one
/// commit: each solver batch holds exactly one solver, so each write of a
/// stacked joint gets its own SSA version. If a batch is ever widened to hold
/// two solvers that write one slot, the merge below would silently collapse
/// two stack entries into one -- so Build REFUSES that shape by name (see the
/// duplicate-slot check in bakedPose.cpp's commit sizing).
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
    /// moverPath spelled at Build, for the body's diagnostics.
    std::string moverPathText;
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
    /// invent a snapshot the reference path never published. Both flags are
    /// what a FrameMatrix step reads through CommitTable (and what the
    /// exporter carries).
    bool recordEveryTarget = true;
    /// Parallel to `sources`, and empty for a source the walk holds a frame
    /// for: the ancestor slots of a native source, in increasing depth.
    std::vector<std::vector<AncestorRead>> sourceAncestors;
    std::vector<AncestorRead> worldUpAncestors;
};

/// What a staged propagation pair turned into. Ordered so that the first
/// non-Staged, non-Skipped outcome in pair order decides the commit, which is
/// where today's loop returns.
using RigExecBakedPropagateOutcome=RigExecPosePropagateOutcome;

class RigExecBakedExecCheckRows;
struct RigExecBakedProgramImpl {
    RigExecRigEvaluator *evaluator = nullptr;
    UsdStageRefPtr stage;
    /// The rig's asset root: the parent of the RigExecRoot, which is the
    /// space every control frame the rig publishes is expressed in, and the
    /// prim a plain Xformable's transform is measured relative to. The
    /// xform-derived provider seeds are measured relative to this prim.
    SdfPath assetRootPath;

    // Captured once, at Build, inside the one translation unit the evaluator
    // declares a friend. Pointers rather than copies wherever the value can
    // move between frames -- a drag rewrites the override list, a cache fills
    // in, the profiler is switched on mid-session -- so the program reads what
    // the evaluator holds NOW and can never answer from a stale copy.
    RigExecResolvedInputs *resolvedInputs = nullptr;
    RigExecProfiler *profiler = nullptr;
    bool recordOpTimings = false;
    const std::vector<RigExecValueOverride> *interactiveOverrides = nullptr;
    /// Joint -> the ordered stack of (solver, element) that write it, which
    /// is where the epilogue recovers the ELEMENT a writer that published
    /// nothing was bound to. rigExec:joints is an ordered write, so a joint
    /// may carry several entries; the last one supplies its base frame.
    const std::map<SdfPath, std::vector<std::pair<SdfPath, int>>>
        *jointSolverBinding = nullptr;
    // Compile-owned normalization. No evaluator table is edited and no
    // original SCC body can become runnable when its writer is removed.
    std::map<SdfPath, std::vector<std::pair<SdfPath, int>>> ownedJointSolverBinding;
    RigExecOpExclusionProof cycleExclusionProof;
    std::map<SdfPath, std::string> cycleSkipReasons;
    /// The observational guide request, which exists only while a consumer
    /// asked for one, and the runtime toggle beside it. Read per frame rather
    /// than folded: either can move without the epoch moving.
    const bool *solverGuidesEnabled = nullptr;
    /// Whether the epoch has any property chain at all. A chain appearing or
    /// disappearing is a structural edit, which recompiles and rebuilds this
    /// program, so this one is a captured constant and not a pointer.
    bool hasPropertyChains = false;

    // The slot table is the ordered UNION of the two provider families the
    // dynamic walk holds in one frame map: the RigExec providers exec seeds
    // (_nativeProviderPaths) and the plain Xformables a constraint targets
    // (_xformDerivedProviders). Both are keyed by SdfPath, whose order IS
    // namespace DFS pre-order, so a provider's parent always has a lower slot
    // than it does and one forward pass composes the whole hierarchy.
    std::vector<SdfPath> paths;
    /// `paths` spelled once at Build on the owning thread, one text per slot.
    /// Step bodies and worker-side keys read this text: SdfPath::GetString
    /// interns under Sdf's table locks. Immutable, so clones share it.
    std::shared_ptr<const std::vector<std::string>> pathTexts;
    std::map<SdfPath, int> index;
    std::vector<RigExecBakedSlotKind> slotKind;
    /// Captured prim activity, distinct from individual attribute availability.
    /// Production capture stores one immutable epoch bit per provider slot.
    std::vector<char> providerActive;
    /// Nearest COMPOSE ancestor -- the nearest FirstFramePose slot above this one.
    /// The compose ladder inherits from it, because exec's NamespaceAncestor
    /// resolves only RigExec provider types and skips anything else.
    std::vector<int> parent;
    /// Nearest ancestor slot of ANY kind, which is what the dynamic walk's
    /// pure namespace climb finds when it looks for the closest revised
    /// ancestor a propagated descendant rides.
    std::vector<int> propParent;

    // A plain Xformable a constraint targets has no rest chain and no avars:
    // its pose is whatever the stage says its transform is, measured relative
    // to the asset root. The run reads that in its PROLOGUE -- it is a stage
    // read, which no step may make -- and leaves the frame in the slot's
    // FIRST version, where the compose would have left one.
    /// The XformDerived slots, ascending, and the prim each one reads.
    std::vector<int> xformSlots;
    RigExecRequiredStageFramesAdmission requiredStageFramesAdmission;
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
        SdfPath path;
        /// `path` spelled at Build, for the body's diagnostics.
        std::string pathText;
        std::array<VtValue,4> raw; ///< source weights/translation/rotation/pole raw samples
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

        /// The four channels in `raw` order (Names()), taken off `prim` at
        /// Build, and the frozen samplers' keys for them. A handle is what
        /// prim.GetAttribute answers: its validity is asked at every read,
        /// so a property authored later reads through it.
        std::array<UsdAttribute,4> attributes;
        std::array<SdfPath,4> keys;
        static const std::array<TfToken,4> &Names();
        /// Whether the frozen samplers read channel \p k: the weights
        /// always, the parent offsets for a ParentConstraint, the pole
        /// weights where the step reads them (frozenSnapshot arrayKeys).
        bool Sampled(size_t k) const
        {
            return k == 0 || (k < 3 ? parentOffsets : readPole);
        }
        /// Whether channel \p k's read can move with the time: the
        /// predicate RigExecBakedClassifyInput applies (neither half
        /// implies the other: a single time sample; a Ts spline).
        bool Varies(size_t k) const
        {
            const UsdAttribute &a = attributes[k];
            return a && (a.ValueMightBeTimeVarying() ||
                         a.GetNumTimeSamples() > 0);
        }
        /// The prologue's epoch state, owner thread only. Every channel is
        /// read again on a forced run and whenever the program stamp or
        /// the evaluator's stage edit serial moved since `sampled`: every
        /// stage notice advances the serial, so between them only the time
        /// can move a raw read. Then a channel is read again only when the
        /// time moves and its read can move with it (`variance`, asked on
        /// the first such move after a full read), or the time moves to or
        /// from Default, which read different opinions.
        enum : uint8_t { kVarianceUnknown = 0, kFixed, kVaries };
        std::array<uint8_t,4> variance{};
        bool sampled = false;
        UsdTimeCode time = UsdTimeCode::Default();
        uint64_t stamp = 0;
        uint64_t serial = 0;
    };
    std::vector<ConstraintArrays> constraintArrays;
    /// Channels the prologue read since Build. Test observable.
    uint64_t constraintArrayReads = 0;
    /// The frozen plain sampler's memo of the channels whose read cannot
    /// move with the time, per constraintArrays entry: the sample one read
    /// added (whether it added one, and its value, hasValue and blocked
    /// bits), kept while the program stamp, the evaluator's stage edit
    /// serial and the Default-ness of the time it was read under stand, as
    /// headLeafConstants is. UI thread only, written through a const
    /// program by the sampler; never cloned.
    struct FrozenArraySample {
        uint8_t variance = ConstraintArrays::kVarianceUnknown;
        bool present = false, hasValue = false, blocked = false;
        VtValue value;
    };
    mutable std::vector<std::array<FrozenArraySample,4>> frozenArraySamples;
    mutable uint64_t frozenArrayStamp = 0;
    mutable uint64_t frozenArraySerial = 0;
    mutable bool frozenArrayDefault = false;
    mutable bool frozenArrayBuilt = false;

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

    // The rest chain and the default-space ladder, resolved once at Build
    // and again on any frame that can move them. Every channel below is a
    // BOUND input rather than a folded constant, which is what lets an
    // animated, connected or chain-written rest bake and what lets a drag
    // on one be placed; when nothing varies and nothing is dragged the
    // Build-time answer stands and the frame path never looks at any of
    // them, which is the fast path the whole rig shape used to be.
    struct Ladder {
        RigExecBakedInput<GfMatrix4d> restSpace, defaultSpace, posedSpace;
        RigExecBakedInput<GfMatrix4d> parentSpace, parentDefaultSpace,
            avarDefaultSpace, posedDefaultSpace;
        RigExecBakedInput<GfVec3d> rotationSign;
        RigExecBakedInput<GfMatrix4d> interveningSpace;
        bool interveningReset = false;
        std::array<int, 7> spaceValues{{-1,-1,-1,-1,-1,-1,-1}};
        bool posedSpaceConnected = false, defaultSpaceConnected = false;
        bool parentSpaceConnected = false, parentDefaultSpaceConnected = false;
        bool avarDefaultSpaceConnected = false, posedDefaultSpaceConnected = false;
        /// rest:tx/ty/tz/rx/ry/rz and default:tx/ty/tz/rx/ry/rz, in that
        /// order, which is the order RigExecBakedComposeAvars takes them in.
        RigExecBakedInput<double> restAvars[6];
        RigExecBakedInput<double> defaultAvars[6];
        RigExecBakedInput<TfToken> rotationOrder;
    };
    /// One per slot; an xform-derived slot's is unused and stays default.
    std::vector<Ladder> ladders;
    std::map<SdfPath,std::set<SdfPath>> poseDependencyPaths;
    std::shared_ptr<const RigExecSceneDescriptors> sceneDescriptors; ///< detached compile facts
    RigExecProviderProgram providerProgram;
    RigExecTypedValueStore providerValues{0};
    RigExecBakedPathLeaves providerLeaves;
    std::vector<uint8_t> providerLeafBlocked;
    /// Built by RigExecBakedCompileOpGraph; null when the provider leaves
    /// are not one dense run of values, and every run re-keys them all.
    std::shared_ptr<const RigExecBakedSpaceLeafIndex> spaceLeafIndex;
    /// Per provider leaf, the kRigExecSpaceLeaf* reasons its next
    /// publication re-keys it; sized with spaceLeafIndex. The sampler sets
    /// Sampled and publication rewrites the byte, so it outlives a sample
    /// that no run published.
    std::vector<uint8_t> spaceLeafRekey;
    /// How many provider leaves the last publication re-keyed.
    size_t spaceLeafKeys = 0;
    std::vector<RigExecValueId> providerLeafValues;
    std::vector<int> providerLeafChains;
    std::vector<int> providerRoutedReads;
    std::vector<SdfPath> providerFrozenKeys;
    std::vector<int> providerExternalSlots;
    struct ProviderFrameInput {
        RigExecValueId value=RigExecNoProviderValue;
        int slot=-1;
        bool base=false;
        uint32_t version=0;
        SdfPath reader;
    };
    std::vector<ProviderFrameInput> providerFrameInputs;
    std::vector<std::vector<int>> poseProviderInputs;
    std::vector<char> connectedPoseProviders;
    std::vector<int> providerParentRawLeaves;
    std::vector<RigExecValueId> providerRefreshTemplates;
    std::vector<size_t> providerRefreshTemplateOps;
    struct ProviderRefresh {
        struct Carry {
            int slot=-1;
            uint32_t baseRead=0,finRead=0,baseWrite=0,finWrite=0;
            std::vector<int> blockingSlots;
        };
        std::string key;
        SdfPath reader;
        int slot=-1;
        size_t checkpoint=0;
        RigExecValueId baseValue=RigExecNoProviderValue,currentValue=RigExecNoProviderValue;
        uint32_t baseRead=0,finRead=0,baseWrite=0,finWrite=0;
        std::vector<Carry> carries;
        std::vector<std::pair<uint32_t,uint32_t>> priorConstraints;
        std::vector<RigExecPointFrame> baseInputs,finInputs,baseOutputs,finOutputs;
        std::vector<uint8_t> blocked;
    };
    std::vector<ProviderRefresh> providerRefreshes;
    std::vector<std::vector<uint32_t>> providerRefreshBefore;
    std::string providerRefreshError;
    std::vector<int> interveningSlots;
    std::vector<SdfPath> interveningAnchors;
    /// True when any channel of any ladder is read per frame. False is the
    /// ordinary rig, and it is what keeps the recompute off the frame path.
    bool ladderVarying = false;
    /// Every override index a ladder channel registered, sorted and unique.
    /// Consulted only while a drag stands, to decide whether that drag is
    /// one of THESE inputs.
    std::vector<int> ladderOverrides;
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
    std::vector<GfMatrix4d> posedD, parentSpaceM;
    std::vector<char> parentSpaceAuthored;
    std::vector<GfMatrix4d> lastPosedD, lastParentSpaceM;
    std::vector<char> lastParentSpaceAuthored;
    std::vector<unsigned char> lastRotationSign;
    std::vector<GfMatrix4d> parentDinv;                // parent default^-1
    std::vector<TfToken> rotOrder;
    /// The frame round trips exec performs between its ladder computations,
    /// which a deep chain drifts without. Per slot, in slot order, because
    /// a child reads its parent's.
    std::vector<GfMatrix4d> restRoundTrip, defaultRoundTrip;
    /// What the last compose of each slot left, for the move comparison
    /// that sets `restChanged` and `ladderChanged`. Seeded by Build.
    std::vector<GfMatrix4d> lastRestM, lastSelfD, lastParentDinv,
        lastPosedAuthoredM;
    std::vector<char> lastPosedAuthored;
    std::vector<TfToken> lastRotOrder;
    /// The rest tier's outputs that moved this run: per slot, and as the
    /// slots in the order they moved, which is what the closure seeds from
    /// and what the next tier run clears.
    std::vector<char> restChanged, ladderChanged;
    std::vector<int> restMoved, ladderMoved;
    /// The rotation order a rest or default offset composes in, and the
    /// order an empty `avars:rotationOrder` stands for. Built at Build, so
    /// no compose builds a token from text.
    TfToken xyzToken;
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

    /// Per slot, avars:rotationSign packed by RigExecRotationSignMask. A
    /// mirrored limb declares it so the same avar value turns both sides the
    /// same way; it multiplies the avar at compose time, never the frame,
    /// so a slot's rest pose is untouched whatever it holds.
    std::vector<unsigned char> rotationSign;

    // A RigExecPoseInterpolator reads the FINAL pose of its driver and writes
    // floats the geometry chains consume, so on the dynamic path it is a
    // phase of its own between the pose walk and the chains. Here it is a
    // step: it reads its driver's (and the driver's parent's) last PoseFin
    // version and writes a range of PoseWeight slots, and a blend channel
    // whose inputs:weight is connected to one of those reads the slot. The
    // edges follow, and so does the cone -- a drag that cannot reach the
    // driver leaves every corrective it drives untouched.
    struct PoseInterpolator : RigExecPoseInterpolatorRecord {
        SdfPath path;
        /// `path` spelled at Build, for the body's diagnostics.
        std::string pathText;
        int driverSlot = -1;
        /// The nearest frame-publishing ancestor the driver's local rotation
        /// is measured against, or -1 when its local rotation is its world
        /// one. Compiled by the evaluator, restated here as a slot.
        int parentSlot = -1;
        /// A NUMERIC driver (rigExec:driverAttributes): the dials read in
        /// place of a transform's translation, one per axis, bound like any
        /// other per-frame input so a dragged or animated one is honoured.
        /// Empty on a transform-driven interpolator.
        std::vector<RigExecBakedInput<double>> valueInputs;
        std::vector<double> values;
        /// `inputs:enabled`, a per-frame input the prologue reads into
        /// `enabledValue` so the body touches no USD.
        RigExecBakedInput<bool> enabled;
        bool enabledValue = true;
        /// This interpolator's slots in poseWeights: the whole range it
        /// writes, the subset the solver fills in ITS pose order, and the
        /// disabled poses' slots, which publish a hard zero.
        int weightBegin = 0, weightEnd = 0;
        std::vector<int> poseSlots;
        std::vector<int> disabledSlots;
        /// The solved table, copied from the evaluator's compiled record: a
        /// constant of the epoch, and the program is dropped with the epoch.
        RigExecPoseInterpolatorInputs kernelInputs;
        /// The solve's output, retained so a frame allocates nothing.
        std::vector<double> scratch;
    };
    std::vector<PoseInterpolator> poseInterpolators;
    /// Every weight property the interpolators publish, its value this run,
    /// and property path -> slot for the blend channel that reads one.
    std::vector<SdfPath> poseWeightPaths;
    std::vector<float> poseWeights;
    std::map<SdfPath, int> poseWeightIndex;

    std::vector<double> avarConstants;                 // providers * 11
    struct AvarBinding {
        size_t slot = 0;
        RigExecBakedInput<double> input;
    };
    std::vector<AvarBinding> avarBindings;            // the varying ones only
    /// The rest, kept only so a drag can reach one. Walked per frame while an
    /// interactive override stands and never otherwise.
    std::vector<AvarBinding> avarConstantBindings;
    /// Avar property path -> its index in avarConstantBindings, for the
    /// constants a value edit can PATCH rather than rebuild: resolved from
    /// the attribute itself (a walk of one, no connection upstream), so the
    /// value a notice names is exactly the value the slot holds. See
    /// RigExecBakedProgram::ApplyAvarValueEdits.
    std::map<SdfPath, size_t> patchableAvars;
    /// Per provider slot p (flat avar slot / 11), its bindings in each list:
    /// avarBindings[avarBindingBegin[p], avarBindingBegin[p + 1]) and the
    /// same over avarConstantBindings with avarConstantBindingBegin. Both
    /// lists are appended in ascending flat slot order, so a provider's
    /// bindings are one run in list order. Built with the lists
    /// (RigExecBakedIndexAvarBindings) and never written by a run; read
    /// through RigExecBakedAvarBindingRange.
    std::vector<uint32_t> avarBindingBegin;
    std::vector<uint32_t> avarConstantBindingBegin;
    /// Indices into avarConstantBindings of the patchable avars an edit has
    /// since animated (a spline key on a released drag). Read per frame the
    /// long way, like a varying binding, until an edit makes them constant
    /// again. Kept apart from avarBindings so the patch map's indices hold.
    std::vector<size_t> promotedAvars;
    size_t boundInputs = 0;
    size_t varyingInputs = 0;

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

    /// Every record an AtPrim transform phase can read, in walk order (by
    /// `commit`, then the commit's target or output order), and what each
    /// one's FrameMatrix step wrote
    /// this run. Kept across runs like deltaValues: a step the cone skipped
    /// left the record it would write again.
    std::vector<RigExecBakedFrameRecord> frameRecords;
    std::vector<GfMatrix4d> frameMatrix;
    std::vector<char> frameMatrixValid;

    /// The number of RigExecBakedPointsBinding ids handed out at Build.
    int pointBindingCount = 0;
    /// Test-only, off unless RigExecBakedProgramTesting::CapturePointReads
    /// turned it on: indexed by binding id, each entry written only by its
    /// binding's reader. Never cloned into a frozen program.
    bool capturePointReads = false;
    std::vector<RigExecBakedPointCapture> pointCaptures;

    // What computeMatrix publishes, over dense slots: the whole
    // AuthoritativeSnapshot request is a re-derivation of values the walk
    // already holds. Program-owned rather than a per-frame allocation,
    // because BOTH halves of a frame read them, the geometry half must see
    // exactly the matrix the pose half published, and a 267 KB zero-fill per
    // frame is work that belongs at Build.
    // One ProviderMatrix step writes each entry the program can read -- the
    // set today's lazy finalMatrixOf/baseMatrixOf computed, no larger -- so
    // there is no on-demand fill and no per-frame reset: an entry no step
    // writes simply keeps a value nothing reads.
    std::vector<GfMatrix4d> finalMatrix, baseMatrix;
    /// Whether some step or the epilogue reads this slot's final / base
    /// matrix, which is what decides that a ProviderMatrix step exists for
    /// it. Build fills both.
    std::vector<char> needFinal, needBase;

    struct Solver : RigExecSolverRecord {
        RigExecSolverInputs kernelInputs;
        RigExecSolverWorkspace kernelWorkspace;
        SdfPath path;
        /// `path` spelled at Build, for the body's diagnostics.
        std::string pathText;
        TfToken type;
        /// Exact supported schema port and compiled target solver identity.
        /// This is a structural prerequisite, not a fabricated numerical read.
        std::vector<std::pair<std::string, int>> relationshipRequirements;
        /// Canonical Solve identity, retained even when the common SCC excludes it.
        std::string solveDescriptorKey;
        // Exec rebuilds every one of the rest members below from the
        // epoch's rests on EVERY evaluation, because it is a pure function
        // of them. The bake resolves it once, and the Solve step resolves it
        // again (RefreshSolverRests) on a run on which one of its
        // `restSlots` moved -- which is what lets a solver measure against
        // an animated, chain-written or dragged rest instead of refusing
        // the rig.
        /// Every provider slot whose rest this description folded in.
        std::vector<int> restSlots;
        /// (slot, element) for the two computations that remap by element:
        /// TwoBoneIk and SplineIk.
        std::vector<std::pair<int, int>> restRefs;
        /// Parallel to restRefs: true where a pose step BELOW this solver in
        /// the rig hierarchical stack wrote that joint, so the rest is the
        /// frame that step left rather than the authored one (spec 4.2).
        /// Parallel to restRefs: the `fin` version each LIVE rest reads, bound
        /// by bindSolverReads to the version standing where this batch begins
        /// -- the same moment the dynamic path reads finalFrames for its
        /// computeRestFrame override.
        std::vector<unsigned> restReads;
        /// True when any rest ref is live, which makes the description
        /// per-frame: it is refreshed on EVERY evaluation and can never be
        /// concluded constant.
        bool hasLiveRest = false;
        /// Test hook: how many times the Solve step refreshed this
        /// description, the cone verifier's second pass excluded. Written
        /// only by this solver's own Solve step; read by no step.
        uint64_t restRefreshes = 0;
        /// One per rest ref, in element order: the basis a solver APPLIES its
        /// solved map to. For RigExecFkChain that is the joint's rest
        /// reference rather than the control's own rest, so a step below the
        /// chain is carried through the solve instead of replaced (spec 4.2).
        /// Empty when the solver's joints do not line up one-for-one with its
        /// elements, and the solver then keeps its own basis.
        /// True when any slot's whole rest CHAIN can move with time.
        bool restsVary = false;
        /// Every override index a rest channel of that chain registered, so
        /// a drag on one dirties this solver's step.
        std::vector<int> restOverrides;
        /// SplineIk rebuilds its rest curve from these two beside the rests.
        /// The bake found this solver's rigExec:joints / jointElements
        /// binding malformed in one of the ways its exec computation checks
        /// at runtime. Such a computation warns and returns an EMPTY
        /// aggregate, so the program publishes one too: every joint the
        /// solver names then falls back to its rest chain, with the
        /// diagnostic that carries.
        // FkChain
        std::vector<int> controls;
        /// rigExec:startFrame provider slot (-1 when unwired or not a
        /// provider, the computation's silent absolute path), its rest,
        /// and the `fin` version bound where this batch begins -- the
        /// baked mirror of the computation's synthetic base element.
        int start = -1;
        uint32_t startRead = 0;
        // IK / spline controls
        int root = -1, mid = -1, end = -1, pole = -1;
        // TwoBoneIk: rests and the measured bone lengths, both epoch-constant
        RigExecBakedInput<double> bend, upperOffset, lowerOffset;
        RigExecBakedInput<float> stretch, softness, pin, softDistance, limbTwist;
        RigExecBakedInput<double> upperScale, lowerScale;
        double upperLengthBase = 0, lowerLengthBase = 0;
        /// rigExec:spaceMatrix: an explicit factor, composed after the
        /// space prim below.
        RigExecBakedInput<GfMatrix4d> ikSpace;
        /// rigExec:space: the prim whose movement from its own rest is the
        /// space the chain is measured in. Read as a frame, exactly like
        /// the root/effector/pole, so an animated master is handled and
        /// the baked path cannot disagree with the dynamic one.
        int spaceSlot = -1, spaceRead = -1;
        /// The rest frames a spline needs to rebuild its rest description
        /// when the space has moved. Kept because the bind-time
        /// splineRest is measured at identity.
        std::array<GfVec3d, 4> spaceRest = {GfVec3d(0), GfVec3d(1, 0, 0),
                                            GfVec3d(0, 1, 0),
                                            GfVec3d(0, 0, 1)};
        // BlendPointFrames
        int inA = -1, inB = -1;
        RigExecBakedInput<float> blendWeight;
        /// An unsupported rigExec:rotationBlend, which is NOT a
        /// `degenerate`: the computation returns the surviving input before
        /// it ever looks at the token, so the rejection only bites when
        /// BOTH inputs are bound. It is checked where the computation
        /// checks it -- in the arm that has two aggregates in hand.
        // SplineIk: the rest description is a pure function of epoch-constant
        // rests, so exec's per-evaluation rebuild bakes out.
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
        // Exact immutable authored array inputs, retained before remapping
        // or default expansion for the optional independent exec witness.
        VtIntArray execJointElements;
        VtFloatArray execTwistWeights;
        VtFloatArray execSplineWeights;
        int execTwistCount = 1;
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
        RigExecReadPhase ribbonPointsPhase;
        RigExecBakedPointsBinding ribbonPointsBinding;
        /// The bind-time value, read once at Default. An attribute carrying
        /// only time samples answers nothing there, which is how the dynamic
        /// path ends up with an empty rest and an empty aggregate.
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
        /// Parallel to `outputs`: whether a read phase names this solver's
        /// checkpoint of that joint (the evaluator's _nativePhaseCheckpoints
        /// membership), which is what gives the output a frame record.
        std::vector<char> outputSnapshots;

        // The candidates this solver published, in `outputs` order and
        // nowhere else: the merge into the batch's table is the commit's
        // job, so two solvers of one batch never write the same storage --
        // an invariant Build now refuses to violate rather than assumes (the
        // duplicate-slot check in bakedPose.cpp's commit sizing).
        std::vector<RigExecPointFrame> outFrames;
        std::vector<char> outPresent;
        /// Where each output lands in its commit's slot-ordered table.
        std::vector<int> outPosition;
        /// Slots of the joints this solver published no element for. Their
        /// key spells each as `pathTexts` holds it.
        std::vector<int> fallbackSlots;
        /// FkChain's element table, so the solve allocates nothing.
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

    struct Constraint {
        RigExecConstraintRecord kernelRecord;
        RigExecConstraintInputs kernelInputs;
        RigExecConstraintResult kernelResult;
        SdfPath path;
        /// `path`, `sourcePaths` and `deltaBasePath` spelled at Build, for
        /// the body's diagnostics.
        std::string pathText;
        std::vector<std::string> sourcePathTexts;
        std::string deltaBasePathText;
        TfToken type;
        /// rigExec:weightObject, empty when the constraint binds none.
        ///
        /// Deliberately NOT an index into `weightObjects`: a constraint
        /// copies the ORACLE, and the oracle resolves the whole composition
        /// itself from the stage (see the head of bakedWeights.cpp).
        SdfPath weightObject;
        int weightField = -1;
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
        /// rigExec:space on a rotation constraint: the provider whose own
        /// rest->pose map carries the whole rig, or -1 when none is named.
        /// The carry is the space switch's, from the same two tables
        /// (posedM and defaultRoundTrip); see the dynamic path, which
        /// derives it from the same pair of taps.
        int spaceSlot = -1;
        /// rigExec:blendShear (Scale, Parent) and
        /// rigExec:worldUpRotationOnly (Aim), compiled; both off is the
        /// original arithmetic.
        bool blendShear = false;
        bool worldUpRotationOnly = false;
        /// rigExec:weightBlend == "radial" on a transform-domain
        /// RigExecMatrixMover, compiled.
        bool radialBlend = false;

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
        bool preserveJointOrientation = false;
        bool useAnimatedTs = false;
        /// Parallel to targetSlots: the compiled "a step below wrote this
        /// joint" flags, so the rest reference is that step's frame.
        std::vector<char> ikRestLive;
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
        RigExecBakedInput<float> stretch;
    };
    std::vector<Constraint> constraints;

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

    // Built from the tables above, once, at the end of Build. `steps` is in
    // program order -- the order today's straight-line Run visited the same
    // work in -- and every edge points forward in it.
    std::vector<RigExecBakedStep> steps;
    std::vector<RigExecBakedStep> excludedSteps;
    /// The compose pass, partitioned into contiguous subtrees at Build.
    /// One compiled RigExecSpaceSwitch, in the program's own terms.
    ///
    /// The compose for an ordinary slot is
    ///
    ///     base = avars * selfD * parentDinv * posedM[parent]
    ///
    /// and a switched slot's is the same shape with the namespace parent's
    /// pair of spaces replaced by the SELECTED source's pair:
    ///
    ///     base = avars * selfD * inverse(selfD[source]) * posedM[source]
    ///
    /// so the two halves cancel at rest and no space can move the rig
    /// standing still. A source slot of -1 is world: it contributes
    /// identity, which pins the control at its zero pose.
    ///
    /// Every provider frame a switch reads -- its namespace parent, each
    /// source, its space -- is read at a VERSION bound at Build (see
    /// FrameVersion), never as whatever the slot holds when the step runs.
    struct SpaceSwitch {
        /// Which version of one provider frame a switch reads: the version
        /// standing when the dynamic walk resolves this switch, with every
        /// switch it resolves earlier applied and every other one not.
        ///
        /// `anchor`'s last version (identity at -1), composed UNSWITCHED
        /// through `recompose`, top down. Empty `recompose` is the anchor's
        /// last version as is, which is every read on a rig whose switches
        /// do not nest; a non-empty one re-derives the pre-switch frame of a
        /// control whose own switch resolves at or after this one.
        struct FrameVersion {
            int anchor = -1;
            std::vector<int> recompose;
            int context = -1;
        };
        int slot = -1;
        std::vector<int> sourceSlots;
        /// Parallel to sourceSlots; unused where the slot is -1 (world).
        std::vector<FrameVersion> sourceReads;
        /// The namespace parent's frame `local` is divided back out of.
        FrameVersion parentRead;
        /// spaceSlot's frame, for the carry.
        FrameVersion spaceRead;
        /// Parallel to sourceSlots: which part of that source's rotation
        /// reaches the target (a pole vector in its hand's space takes the
        /// twist and not the swing).
        std::vector<RigExecRotationFilter> filters;
        GfVec3d twistAxis = GfVec3d(1, 0, 0);
        /// rigExec:space -- the provider whose own rest->pose map carries
        /// the whole rig. -1 when none is named, and then the filter runs
        /// exactly as it did before the masters existed. See the dynamic
        /// path, which derives the same carry from a pair of taps.
        int spaceSlot = -1;
        RigExecBakedInput<double> activeInput;
        RigExecBakedInput<TfToken> activeTokenInput;
        bool tokenIndex = false;
        std::vector<TfToken> labels;
        bool affectTranslation[3] = {true, true, true};
        bool affectRotation[3] = {true, true, true};
        bool affectScale[3] = {true, true, true};
        RigExecSpaceSwitchRecord kernelRecord;
        RigExecSpaceSwitchInputs kernelInputs;
    };
    struct SpaceCheckpoint {
        std::string key;
        int anchor=-1;
        std::vector<int> recompose;
        std::vector<RigExecSpaceCheckpointInput> kernelInputs;
    };
    std::vector<SpaceCheckpoint> switchFrameContexts;
    std::vector<GfMatrix4d> switchFrames;
    struct AutoClavicle {
        struct FrameRead {
            RigExecValueId value=UINT64_MAX;
            int slot=-1;
            std::string computation;
            std::vector<int> recompose;
        };
        struct ScalarRead {
            RigExecValueId value=UINT64_MAX;
            bool isFloat=false;
            RigExecBakedInput<float> narrow;
            RigExecBakedInput<double> wide;
        };
        int slot=-1;
        RigExecBoundAutoClavicle operation;
        RigExecTypedValueStore values;
        std::vector<FrameRead> frames;
        std::vector<ScalarRead> scalars;
    };
    std::vector<AutoClavicle> autoClavicles;
    std::vector<int> autoClavicleBySlot;
    std::vector<SpaceSwitch> spaceSwitches;
    /// Per provider slot: its switch's index, or -1. Read once per slot by
    /// the compose, so the ordinary rig pays one array lookup and nothing
    /// else for a feature it does not use.
    std::vector<int> spaceSwitchBySlot;

    /// Set when the compose groups could not be put in dependency order.
    /// With every switch read bound to its version this never happens on a
    /// rig the compile accepted; Build refuses the program if it does.
    bool composeCycle = false;

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
    /// The RevisionFuse step of each revision id: the only writer of its
    /// RevisionDone and ChainDirty slots, and so the producer of point
    /// version r + 1 of its chain (RigExecBakedPointVersion).
    std::vector<int> revisionFuseStep;
    /// The layout (RigExecBakedLayoutRevision index) of every SkinTopology
    /// step, set where the op graph is compiled, so head preparation clears
    /// their changed flags without walking every step.
    std::vector<int> skinTopologyLayouts;

    /// The partition of `steps` the parallel executor runs, chosen once at
    /// Build. Its grain comes from the cost model and the machine's
    /// concurrency, never from a measurement: a schedule that depended on
    /// what the box was doing at Build time would make "the same program
    /// produces the same answers at every grain" a claim nobody could test.
    RigExecBakedClustering clustering;
    // Settings a body path needs, read from the environment once at Build so
    // that no body reads the environment or a function-local static.
    /// RIGEXEC_BAKED_CHUNK_VERTS: vertices one chunk covers before the cap.
    size_t chunkVertexTarget = RigExecGeometryParallelThreshold;
    /// RIGEXEC_BAKED_MAX_CHUNKS: the most chunks one revision is cut into.
    size_t chunkCap = 32;
    /// RIGEXEC_ENABLE_SIMD, as RigExecSimdEnabled() answers it.
    bool useSimd = true;
    /// RIGEXEC_PURITY_AUDIT: whether RunStepBody hands its bodies
    /// purityViolations to count reads under RIGEXEC_PURITY_CHECK.
    bool purityAudit = false;
    /// Body reads counted under purityAudit. Relaxed, because nothing is
    /// published through it; the test thread reads it after the run joins.
    /// A copy or move starts at zero, so it does not make the program
    /// immovable; _CloneImpl copies purityAudit and zeroes this, so a frozen
    /// clone counts its own.
    struct PurityCounter {
        std::atomic<uint64_t> count{0};
        PurityCounter() = default;
        PurityCounter(const PurityCounter &) {}
        PurityCounter &operator=(const PurityCounter &) {
            count.store(0, std::memory_order_relaxed);
            return *this;
        }
    };
    PurityCounter purityViolations;

    // What a run may SKIP. The sets are Build's; everything below them is the
    // last run's answer, kept so that this run's sources can be compared with
    // it by VALUE. There is no "time changed" and no "overridden" predicate
    // deciding whether a source ran: sources always run and their outputs are
    // compared, which is what makes an override on a routed prim, a released
    // drag, a rebuilt skin layout and a moved keyframe all reach the graph
    // through one test (§7).
    RigExecBakedCones cones;
    /// The steps this run decided to run. Every step outside it keeps its
    /// slots, its diagnostics and its structural counters, whether or not
    /// its cluster runs.
    RigExecBakedClusterSet closedSteps;
    /// The clusters that hold a step of `closedSteps`: what the parallel
    /// executor dispatches and counts. The frozen serial runner and a
    /// partial cone re-run still run whole clusters of it, which is a
    /// superset of the closed steps and so the same answer.
    RigExecBakedClusterSet closed;
    /// The avar table as the last run left it, for the per-provider compare.
    std::vector<double> lastAvars;
    /// The override flags as the last run left them, so that the run AFTER a
    /// drag is released re-runs what the drag was holding.
    std::vector<char> lastOverridden;
    /// One flag per override index whose input a stage VALUE edit reached
    /// since the last run: set by RigExecBakedProgram::ApplyValueEdits,
    /// OR-ed across notices, and consumed only by the closure's override
    /// rule, beside `lastOverridden` -- the step that reads such an input
    /// re-runs once, exactly as it does for the run after a drag is lifted.
    /// Pending bits survive a generation the program did not answer (a
    /// dynamic fallback), and a frozen clone carries them, because the
    /// clone's first run is the one that owes them.
    std::vector<char> edited;
    bool anyEdited = false;
    /// How many notices ApplyValueEdits has marked inputs for, and, per
    /// override index, the count at the last one that marked it. Unlike
    /// `edited`, nothing consumes these: a frozen snapshot records the count
    /// it was taken or patched at, and a later patch marks every index
    /// edited since -- including edits the live program has already run
    /// and cleared, which the snapshot's own state has never seen.
    uint64_t valueEditSerial = 0;
    std::vector<uint64_t> editSerial;
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
    /// Whether the closure this run computed trusts nothing it holds (a
    /// forced run, or a moved program stamp). Set by
    /// RigExecBakedComputeClosure before any region step runs and read by
    /// the Solve step's rest-refresh gate: such a run can follow a
    /// generation whose rest tier moved rests and whose region never ran.
    bool closureFull = false;
    /// How many clusters the last run ran, and how many there are, for the
    /// schedule run report.
    size_t lastClosedClusters = 0;
    /// How many steps the last run's closure held, sources it seeded from
    /// included.
    size_t lastClosedSteps = 0;

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
    /// Whether a step may hold a nonzero stamp (startUs/endUs or an op
    /// stamp). The owner sets it before any run whose ops stamp;
    /// RigExecBakedRunStatistics::Restore only puts back stamps such a run
    /// wrote since the last clear. RigExecBakedClearRunStamps zeroes the
    /// stamps and resets it.
    bool runStamped = false;

    /// Per joint, whether this run's final frame earned a published matrix.
    /// Written by the diagnostic pass, read by the fill pass; sized at
    /// Build, so the epilogue allocates nothing.
    std::vector<char> jointMatrixPublished;

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
        SdfPath weightPath;
        /// The PoseWeight slot `inputs:weight` resolves to through its
        /// authored connection, or -1 when it resolves to the stage. The
        /// dynamic walk finds the same value by following the connection
        /// into the generation's resolved inputs, which the interpolator
        /// phase filled; here the phase is a step and the value is a slot,
        /// which is what puts the edge in the graph.
        int poseWeight = -1;
        struct Sample {
            /// The RigExecBlendSample prim, which keys the shape cache.
            SdfPath samplePath;
            /// `rigExec:activation` on the RigExecBlendSample prim.
            UsdAttribute activation;
            /// The activation attribute's path, beside the handle: the frozen
            /// worker keys the sampled activation by this, because it may
            /// not touch the cloned handle -- not even GetPath.
            SdfPath activationPath;
            /// The sample's target-shape points, and the path a declared
            /// read phase looks that array up under. Dense form only.
            UsdAttribute points;
            SdfPath pointsPath;
            RigExecReadPhase phase;
            /// A dense sample's non-base phase, bound at Build; `id` is -1
            /// for a base or sparse sample, which never reads a record.
            RigExecBakedPointsBinding pointBinding;
            /// Sparse shape identity and the last body-normalized immutable layout.
            SdfPath blendShape;
            std::shared_ptr<const RigExecBlendSampleLayout> layout;
            bool shapeValid = false;
            /// What `layout` was built from while `layoutKeyed`: the content
            /// versions of the offsets and indices leaves and the point
            /// count. The assembly rebuilds the layout only when one of them
            /// moved; `layoutBuilds` counts the builds. Owned by the body
            /// that assembles the revision.
            bool layoutKeyed = false;
            uint64_t layoutOffsetsVersion = 0;
            uint64_t layoutIndicesVersion = 0;
            size_t layoutPointCount = 0;
            uint64_t layoutBuilds = 0;
            /// Structural cache refusal retained for the runtime stream policy.
            bool layoutRefused = false;
            /// The dense points the last assembly consumed; the bake stores
            /// them as the sample's static points. Dense form only; a sparse
            /// sample's shape rides its layout instead.
            std::vector<GfVec3f> lastPoints;
            /// The revision's declared activation, dense points, and sparse
            /// Raw Default offsets/indices leaf IDs, or -1.
            int activationLeaf = -1;
            int pointsLeaf = -1;
            int offsetsLeaf = -1;
            int indicesLeaf = -1;
        };
        std::vector<Sample> samples;
        /// The revision's path leaves holding `inputs:weight` and whether the
        /// generation's resolved inputs hold its path, or -1.
        int weightLeaf = -1;
        int weightHeldLeaf = -1;
    };

    struct GeomRevision {
        RigExecGeometryRecord kernelRecord;
        RigExecSceneGeometryDescriptor sceneGeometry;
        SdfPath moverPath;
        /// moverPath spelled at Build, for the bodies' diagnostics and
        /// status addresses.
        std::string moverPathText;
        SdfPath target;
        UsdPrim moverPrim;
        RigExecRevisionOp op = RigExecRevisionOp::Skin;
        RigExecRevisionBinding binding;
        /// This revision's blend channels, in `binding.blendInputs` order.
        /// Empty for every operation but a blend shape.
        std::vector<GeomBlendChannel> blendChannels;
        std::vector<int> influenceSlots;
        int transformSlot = -1;
        /// The rigExec:transformSpace provider's slot, or -1.
        int transformSpaceSlot = -1;
        /// The rigExec:space provider's slot, or -1: the prim whose own
        /// rest->pose map carries the whole rig, normally a TRS master.
        /// Read out of the SAME base/final matrix table the two above come
        /// from, because that table is what computeMatrix publishes and
        /// the dynamic path's carry tap reads computeMatrix.
        int carrySpaceSlot = -1;
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
        /// rigExec:pointFrame == "posed": a skin already carried
        /// this cluster's points into the rig's posed frame, so the
        /// measured offset is conjugated by the rig's carry -- or, with no
        /// rigExec:space named, by the measuring space's own scale.
        bool posedPoints = false;
        /// Compile's judgement that none of this skin mover's layout arrays
        /// can change within the epoch, so the packet may carry the layout
        /// by handle out of the evaluator's cache instead of re-reading and
        /// re-validating 105k elements every frame.
        bool skinTopologyFixed = false;
        /// The node's last published status, which outlives an evaluation it
        /// did not take part in -- so does its diagnostic.
        TfToken resultStatus;
        /// The revision's applied points, which `currentSource` publishes,
        /// and the buffer its chunks write. An applying fuse swaps an
        /// unchunked revision's two buffers instead of copying one into the
        /// other; a chunked skin keeps its ranges in staging.
        std::vector<GfVec3f> output;
        std::vector<GfVec3f> stagingOutput;
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
        /// This revision's own wire-basis memo: only its step runs its
        /// kernel, so nothing else touches it. A clone shares the entries.
        RigExecWireBasisCache wireBasis;
        /// Mutated only by this revision's unchunked body/projector. Copies
        /// share immutable entries; each clone replaces its own cache slots.
        mutable RigExecSurfaceKernelCache<GfVec3f,GfVec3d> surfaceCache;
        /// A read phase named this revision as the point in the chain it
        /// wants the target's points from (the dynamic walk records them
        /// after it). Decided at bake out of the evaluator's
        /// _nativePhaseCheckpoints; BindPointReads binds `preceding` reads of
        /// another chain and AtPrim reads to it. The plan also names the
        /// predecessor of an own-chain `preceding` reader, but that read
        /// binds to the version entering the reader and reads no record.
        bool snapshotAfter = false;
        /// One per `binding.phases` entry, in the map's order.
        std::vector<RigExecBakedPointsBinding> pointBindings;
        /// For an AtPrim `binding.transformPhase` only: the `frameRecords`
        /// of the transform provider whose writer (constraint or solver) is
        /// the phase's prim or under it, newest first. The fold takes the first valid one,
        /// and otherwise keeps the dense-table matrix. `influenceRecords`
        /// is the same per `influenceSlots` entry.
        std::vector<int> transformRecords;
        std::vector<std::vector<int>> influenceRecords;
        /// This node is new to the rig's geometry state and its creation has
        /// not been reported yet. Cleared by AdoptGeometryStateFrom for a
        /// node the outgoing program already held, which is the same
        /// bookkeeping the dynamic walk does when it reconnects a surviving
        /// VdfNetwork node instead of adding one.
        bool created = true;

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
        /// rigExec:space's matrix this run, out of the same base/final
        /// table as `transform`, and whether the revision named one at all.
        /// Written by the fold, because the fold is the step that declares
        /// the matrix reads; the assemble only reads what the fold wrote.
        /// A cluster conjugates its offset by it, a posed transform-driven
        /// wire carries its control polygons by it.
        GfMatrix4d carry{1.0};
        bool haveCarry = false;
        /// The resolved inputs overlaid with what this revision's point
        /// bindings resolve to (RigExecBakedOverlayPointReads), built only
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
        /// The revision's vertex chunks, in vertex order, covering
        /// [0, pointCount) exactly once. Always at least one; more only for
        /// a skin revision whose layout the epoch fixed (see
        /// RigExecBakedPartitionRevision).
        std::vector<GeomChunk> chunks;
        /// Where this revision's chunks start in the RevisionOut domain.
        int chunkBase = 0;
        /// The handle the chunk keys describe, so a frame can tell in O(1)
        /// whether they still describe the vertices: null until the first
        /// adoption, then the last handle whose arrays were Build's
        /// (RigExecBakedAdoptPartition). The keys themselves never move
        /// after Build.
        std::shared_ptr<const RigExecSkinTopology> partitionTopology;
        /// The `jointIndices` and element size Build cut the keys from.
        /// The indices are held only while the revision is chunked.
        VtIntArray partitionIndices;
        int partitionElementSize = 0;
        size_t partitionIndexCount = 0;
        size_t partitionPointCount = 0;
        /// The keys are used -- more than one chunk, so an influence outside
        /// a chunk's key is an influence that chunk will not see.
        bool chunked = false;
        /// The natural vertex cut and its exact typed matrix producer sets.
        /// Bounds count distinct producers, independently of graph scheduling.
        size_t partitionCandidates = 0;
        int partitionProducerMin = 0;
        int partitionProducerMax = 0;
        size_t partitionDistinctReads = 0;
        std::vector<std::vector<std::pair<uint8_t,int>>> partitionProducerSets;

        /// The layout half of the skin kernel's validation (the matrix half
        /// is `influencesValid` below).
        bool layoutUsable = false;
        /// The envelope, resolved ONCE at the full point count because it
        /// resolves atomically, and the predicate that says the blend is the
        /// identity and the resolution therefore dead. A skin's, and a dense
        /// wire's separate blend.
        std::vector<float> envelope;
        bool envelopeOk = false;
        bool fullStrength = false;
        /// Whether the revision applies, decided by RevisionStatic over
        /// `precedingCount` points from the validation its kernel runs first
        /// (RigExecRevisionKernelAcceptance; a skin's from the packet half,
        /// the fuse ANDing in the fold's `influencesValid`). Part of the
        /// packet: the fuse selects by it and every chunk's `ok` agrees with
        /// it. Deferred leaves the answer to the chunks.
        RigExecRevisionAcceptance acceptance =
            RigExecRevisionAcceptance::Refuses;
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

        /// rigExec:weightObject as an index into `weightObjects`, or -1.
        /// The packet itself is shared -- one per object per frame, however
        /// many movers bind it -- so what a revision holds is the index.
        int weightObject = -1;
        int weightField = -1;
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
        std::vector<float> publishedWeightValues;
        bool weightFieldPublished = false;

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
        /// An unchunked revision's chunk wrote `stagingOutput` and no fuse
        /// has applied it yet; while false, `output` holds the chunk's
        /// latest result.
        bool stagingFresh = false;
        /// An exact copy of the points the last RevisionDone publication
        /// carried when the revision passed through: the baseline the next
        /// publication is compared against. While it applied, the baseline
        /// is `output` itself.
        std::vector<GfVec3f> passedPoints;
        /// The content version RevisionDone and ChainDirty key the points
        /// by: the fuse bumps it exactly when the points it publishes differ,
        /// byte for byte, from the ones its previous publication carried.
        uint64_t doneVersion = 0;
        /// The epoch-fixed skin layout the packet carries: the SkinTopology
        /// op's `layoutHandle`, adopted where the geometry prologue resolves
        /// a topology (a chain revision whose chain read a base, a derived
        /// revision with a base). Null is a refused layout, which is why
        /// `topologyResolved` and not the pointer says whether the prologue
        /// answered. Both are exported.
        std::shared_ptr<const RigExecSkinTopology> topology;
        bool topologyResolved = false;
        /// The SkinTopology head op of a `skinTopologyFixed` revision.
        /// `layoutLeaves` are the layout's three reads
        /// (RigExecDeclareSkinLayoutLeaves), sampled on the owning thread
        /// before the op runs, and `layoutOverlay` the overlay entry each
        /// last saw at its path. `layoutFixed` is RigExecSkinLayoutIsFixed:
        /// its topology half (`layoutTopologyFixed`) asked at Build only, as
        /// an edit to either path rebuilds the program, and its weights half
        /// asked at Build and again when a value edit reaches the weights or
        /// the program stamp moves; `layoutFixedChanged` says
        /// the last sample moved it. `layoutHandle` is the op's output: null
        /// while the layout is not epoch state, else the layout the leaves
        /// describe, the same object for as long as they describe the same
        /// layout. `layoutCandidate` is the last non-null handle, handed back
        /// when a rebuild finds the same layout so that the packet still
        /// compares equal; AdoptGeometryStateFrom carries it into a rebuilt
        /// program.
        RigExecBakedPathLeaves layoutLeaves;
        std::vector<VtValue> layoutOverlay;
        bool layoutTopologyFixed = false;
        bool layoutFixed = false;
        bool layoutFixedChanged = false;
        bool layoutRan = false;
        bool layoutOutputChanged = false;
        std::shared_ptr<const RigExecSkinTopology> layoutHandle;
        std::shared_ptr<const RigExecSkinTopology> layoutCandidate;
        /// Every read RevisionStatic and the packet assembly make, sampled
        /// in the geometry prologue. For an operation the leaves assemble
        /// (`leaves.decl.assembles`), the assembler's reads and the blend
        /// gather's; for every chain revision, `defaultWeightLeaf`.
        RigExecBakedPathLeaves leaves;
        /// RevisionStatic's `inputs:defaultWeight` read, as a key of
        /// `leaves`; -1 for a derived revision, which reads none.
        int defaultWeightLeaf = -1;
        /// An External revision's payload leaf: what the plugin's
        /// assembleExternal answered in the geometry prologue, on the owning
        /// thread. Volatile: the plugin's reads are opaque, so it is
        /// assembled on every run that reaches the prologue, and
        /// `externalPayloadChanged` says only whether it differs
        /// (operator==) from the last one; RevisionStatic still decides by
        /// comparing the whole packet. Held only where the leaves assemble
        /// the revision (`leaves.decl.assembles`): a plugin revision bound
        /// to a value the region computes (a declared phase, a weight
        /// object, a transform, a carry, influences, driver frames, a
        /// constraint delta or blend channels) calls the plugin from
        /// RevisionStatic through the stage assembler.
        RigExecExternalPayload externalPayload;
        bool externalPayloadChanged = false;
    };
    struct GeomChain {
        SdfPath target;
        UsdAttributeQuery baseQuery;
        std::vector<GeomRevision> revisions;
        VtVec3fArray sampledBase;
        bool sampledHaveBase = false;
        VtVec3fArray lastBase;
        VtVec3fArray result;
        /// ChainInput, ChainBase and ChainPoints key their points by these
        /// content versions, each bumped exactly when the array's bytes
        /// differ from the ones last published. `publishedInput` is the
        /// sampled base ChainInput last published, held by handle.
        VtVec3fArray publishedInput;
        uint64_t inputVersion = 0, baseVersion = 0, resultVersion = 0;
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
            /// `target` spelled at Build, for the body's diagnostic.
            std::string targetText;
            UsdAttributeQuery baseQuery;
            GeomRevision revision;
            VtVec3fArray sampledBase;
            bool sampledHaveBase = false;
            VtVec3fArray lastBase;
            VtVec3fArray result;
            /// The other half of the published double buffer; see the
            /// chain's.
            VtVec3fArray spare;
            /// DerivedOut's content version of `result`, as the chain's.
            uint64_t resultVersion = 0;
            bool haveResult = false;
            /// As the chain's, read by the prologue.
            bool haveBase = false;
            bool baseDirty = false;
            /// A surface projector target (RigExecIsDerivedMatrixOp): no
            /// authored base, and a MATRIX published in place of a vec3f
            /// array -- when this run measured one.
            bool matrixTarget = false;
            GfMatrix4d matrix{1.0};
            bool haveMatrix = false;
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

    // One entry per weight object the epoch reaches, in DEPENDENCY ORDER
    // (post-order over rigExec:baseWeight and rigExec:inputWeights, children
    // before parents), so one forward pass per frame builds every packet and
    // a composed object finds its inputs already built.
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
        bool envelopeOnly = false;
        RigExecWeightOracleFacts oracleFacts;
        RigExecBakedPathLeaves oracleLeaves;
        std::vector<SdfPath> oracleFrozenKeys;
        int oracleKind = -1, oracleRepresentation = -1, oracleCombineMode = -1;
        int oracleAxis = -1;
        bool oracleAxisAttribute = false, oracleBoundsAttribute = false;
        bool oracleClamp = false, oracleBounded = false, oracleUnbounded = false;
        std::string oracleName, oracleAxisName, oracleBoundsName;
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

        /// The provider slot the volume is posed into, and therefore the
        /// BASE frame its field is placed against -- base and not final,
        /// because the evaluator overrides every seeded provider's
        /// computePointFrame with its base frame before the authoritative
        /// snapshot, and a mover's packet is what that snapshot carries.
        int providerSlot = -1;
        RigExecBakedInput<float> falloffMin, falloffMax;
        RigExecBakedInput<float> scaleX, scaleY, scaleZ;
        RigExecBakedInput<float> scaleXPos, scaleYPos, scaleZPos, scaleXNeg, scaleYNeg, scaleZNeg;
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
        std::array<VtVec3fArray,3> packetPointViews;
        std::array<std::vector<GfVec3f>,3> packetPointScratch;
        std::vector<GfVec3f> packetLocalCurve;
        std::vector<const RigExecWeightPacket *> packetInputRefs;
        RigExecWeightPacketWorkspace packetWorkspace;
        /// The point gathers' reads, one key per attribute of
        /// `targetPoints`, `samplePoints`, `curvePoints` and
        /// `combineTargetPoints` in that order, each list starting at its
        /// index below.
        RigExecBakedPathLeaves pointLeaves;
        size_t targetLeaves = 0, sampleLeaves = 0, curveLeaves = 0,
               combineLeaves = 0;

    };
    std::vector<WeightObject> weightObjects;
    std::vector<RigExecWeightRecord> weightProgram;
    std::vector<char> weightCycleBlocked; ///< immutable common SCC outcome
    struct WeightField {
        enum class Form : uint8_t {
            EnvelopeProperty = 0, EnvelopeConstraint = 1, Revision = 2
        };
        enum class PlacementPhase : uint8_t { Final = 0, Base = 1 };
        Form form = Form::Revision;
        PlacementPhase placementPhase = PlacementPhase::Base;
        int object = -1;
        int consumer = -1;
        int part = 0;
        std::vector<int> volumes;
        std::vector<int> availableChains;
        bool changed = false;
        std::vector<RigExecBakedReaderWalk> scalarReads;
        std::vector<int> scalarObjects, scalarMembers;
        std::vector<int> objects; ///< immutable reachable object identities
        std::vector<int> scalarReadIndex; ///< object * 19 + member, built once
        struct PointInput {
            int object = -1, leaf = -1;
            RigExecBakedPointsBinding binding;
        };
        std::vector<PointInput> pointReads;
        std::vector<float> values;
        std::vector<float> nextValues; ///< producer-owned reusable result scratch
        RigExecWeightFieldWorkspace workspace;
        std::vector<RigExecWeightFieldInputs> currentInputs; ///< producer-owned current packet scratch
        mutable std::vector<RigExecWeightFieldInputs> effectiveInputs; ///< exclusive producer key scratch
        size_t count = 0;
        bool ok = false;
        std::string error;
    };
    RigExecCompiledGraph opGraph;
    RigExecOpWorkspace opWorkspace;
    RigExecOpExecution opExecution;
    RigExecOpAdapterState opAdapter;
    /// RIGEXEC_VERIFY_CHAIN_VERSIONS, read at compile. After a run, the
    /// owner rebuilds each published point-version key over the points'
    /// bytes (RigExecBakedChainContentKey) and checks the two told the same
    /// change; `chainContentKeys` holds the last such key per value id.
    bool verifyChainVersions = false;
    std::vector<std::string> chainContentKeys;
    size_t chainVersionMismatches = 0;
    std::shared_ptr<RigExecBakedExecCheckRows> execCheckRows;
    std::function<void(uint32_t)> opBeforeBody, opAfterBody; ///< opt-in test observation
    std::vector<WeightField> weightFields;
    std::vector<GfMatrix4d> volumePlacementBase;
    /// Discovery allocates identities on entry, including authored back-edges.
    /// The canonical graph diagnoses cycles after all producer reads exist.
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
    /// Per provider slot, where the volume at that slot is placed: the
    /// output of that slot's VolumePlacements step (WeightFrames[slot]), read
    /// by the oracle and by pose.weightFrames. Every noScaleAvars slot has
    /// a live and a frozen step; the oracle and every publication (live and
    /// frozen) read only the placedVolumes slots. Non-volume slots stay
    /// identity and are never read. Kept across runs like deltaValues, and
    /// carried by a retained frozen workspace: an operation that
    /// skipped a step keeps its last run's placement; its inputs are unchanged.
    std::vector<GfMatrix4d> volumePlacement;
    /// Per provider slot: 1 where the dynamic walk places this volume (a key
    /// of the evaluator's _volumeWeightMatrixTaps). A subset of
    /// noScaleAvars: a volume outside the rig that only a constraint names
    /// is a provider but is not tapped, so the walk neither places nor
    /// publishes it, and neither do the oracle view and pose.weightFrames.
    std::vector<char> placedVolumes;
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

    // What the bake looked at, so a notice can be answered without rebuilding
    // and without guessing. Four sets, because a notice asks four different
    // questions and one set would have to answer the bluntest of them:
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
    // A changed-info notice that misses all of them is a value edit on an
    // input the frame path re-reads, and needs nothing.
    std::set<SdfPath> rebuild;
    std::set<SdfPath> sourceBackedPaths;
    std::set<SdfPath> named;
    std::set<SdfPath> prims;
    std::set<SdfPath> xformPrims;

    // One flag per registered input; see RigExecBakedRead. Kept as a dense
    // vector so the frame path costs an index rather than a map lookup per
    // read.
    std::vector<char> overridden;
    /// Property path -> the inputs reading it, for the placeable case.
    std::map<SdfPath, std::vector<int>> overridableInputs;

    /// The sampled leaves (RigExecBakedNumberLeaves, RigExecBakedSampleLeaves).
    /// Prologue state: the sampler writes them before the region, on the
    /// owning thread, and nothing after it does.
    RigExecBakedLeafPools leaves;
    /// Leaf id -> pool and index, in _ForEachPatchableInput order.
    std::vector<RigExecBakedLeafRef> leafRefs;
    /// Override number -> the leaf of the binding holding it, or -1.
    std::vector<int> leafOfOverride;
    /// Every path on a leaf's walk -> the leaves reading it. Built from
    /// `overridableInputs`, which files every hop of a registered binding's
    /// walk under its number; Build only, read on the owning thread.
    std::map<SdfPath, std::vector<uint32_t>> leafByPath;
    /// The overrides SetOverrides placed through the routed arm (no number),
    /// by path, and the table the last sample compared against. A leaf filed
    /// under a path whose value differs between the two is re-sampled.
    std::map<SdfPath, VtValue> routedOverrides;
    std::map<SdfPath, VtValue> lastRoutedOverrides;
    /// The upstream layer (RigExecBakedProgram::SetUpstreamInputs): this
    /// run's admitted values by path, and the table the last prologue
    /// placed. Every hop read through the layer answers from `upstream`
    /// before the stage; the prologue compares the two by value (rule 8).
    std::map<SdfPath, VtValue> upstream;
    std::map<SdfPath, VtValue> lastUpstream;
    /// Per override number: an upstream value stands on a hop of its walk,
    /// so its binding reads the long way, through the layer.
    std::vector<char> upstreamOn;
    /// Per override number: a value on a hop of its walk was placed, moved
    /// or lifted this run. The closure's override seed; never a standing
    /// flag, so a value that stays costs nothing.
    std::vector<char> upstreamChanged;
    /// Something in `upstream` moved this run.
    bool upstreamMovedThisRun = false;
    /// RigExecBakedProgram::GetUpstreamAdmissible and GetUpstreamOracle,
    /// built on first use (`upstreamSetsBuilt`).
    std::map<SdfPath, TfType> upstreamAdmissible;
    std::set<SdfPath> upstreamOracle;
    bool upstreamSetsBuilt = false;
    /// Leaves re-read by every sample so far, Build's included. Test
    /// observable; written only by the sampler.
    uint64_t leafSamples = 0;
    /// Every path leaf (RigExecBakedPathLeaves of a chain revision, a
    /// derived target or a weight object), in program order; path leaf id
    /// `leafRefs.size() + i` is entry i, filed in `leafByPath` under every
    /// path its read can reach.
    std::vector<RigExecBakedPathLeafRef> pathLeafRefs;
    /// The chain results the path leaves last compared against, and a serial
    /// that moves whenever they differ from the run's (a path leaf whose
    /// sample saw another serial re-reads).
    std::map<SdfPath, VtValue> pathLeafChainResults;
    uint64_t pathLeafChainSerial = 0;
    /// Path leaves re-read by every sample so far. Test observable; written
    /// only by RigExecBakedSamplePathLeaves.
    uint64_t pathLeafSamples = 0;
    /// Executes that built operation keys, which read the path leaves'
    /// versions (RigExecBakedSetPathLeaf). Owner of the program only.
    uint64_t pathLeafRun = 0;
    /// Every property a chain writes: RigExecBakedBuildContext::chainTargets
    /// as Build classified the inputs against it, kept so a bake can
    /// recompute each input's walk the same way.
    std::set<SdfPath> chainTargets;
    /// Prims whose reads already route through the generation's resolved
    /// inputs (property-chain movers, geometry movers), so an override on
    /// any of their properties reaches them with nothing else to do.
    std::set<SdfPath> resolvedRoutedPrims;
    /// Every attribute (and any other target) a read through the resolved
    /// inputs can reach by following authored connections out of a
    /// `resolvedRoutedPrims` prim, hop by hop. The bake records the prims of
    /// those readers and never the sources their connections name, so a
    /// source on a prim nothing else reads would otherwise look read by
    /// nothing: the program itself is still right (the reader re-resolves
    /// and compares by value), but the frame cache would be told no read
    /// path moved and keep frames the edit changed.
    ///
    /// Filled on the notice thread, only when a notice names a property the
    /// rest of the index does not place, and answered for the program stamp
    /// it was filled under. Every change that can move a connection -- a
    /// connection or target field, a resync, layer metadata -- is refused by
    /// the value-edit routing or rebuilds the program, and a refusal bumps
    /// the stamp, so a stamp that has not moved is a connection graph that
    /// has not either. Nothing in a run reads it, and a frozen clone never
    /// routes a notice, so it is not cloned.
    mutable std::set<SdfPath> connectedSources;
    mutable uint64_t connectedSourcesStamp = 0;
    mutable bool connectedSourcesFilled = false;
    /// True while the dense avar table still holds a value written for a
    /// drag; see the input block in Run.
    bool avarsDisturbed = false;
    /// Properties folded into bake state. An override here cannot be placed
    /// without rebaking, so it forces the dynamic path instead.
    std::set<SdfPath> folded;
    bool anyOverridden = false;
    /// RigExecBakedProgram::SetPublishWeightFields.
    bool publishWeightFields = true;

    /// Property operation metadata. Build state: the chains,
    /// records, leaves, override slots and steps. Everything after
    /// the head prefix is prologue state, written on the owning thread before
    /// the region.
    std::vector<RigExecCrossDomainRead> crossDomainReads;
    std::vector<std::string> crossDomainErrors;
    std::map<SdfPath,int> crossDomainOrdinals;
    std::vector<RigExecBakedPropertyChain> propertyChains;
    std::vector<RigExecBakedPropertyRecord> propertyRecords;
    /// The PropertyVersion domain's size: every chain's versions, then the
    /// records.
    uint32_t propertyVersionCount = 0;
    std::vector<RigExecBakedHeadLeaf> headLeaves;
    /// Every path a head op reads an override at, and its slot.
    std::map<SdfPath, uint32_t> headOverrideSlots;
    /// The same slots keyed as an override names its property, (prim,
    /// attribute), so placement builds no path: the frozen worker runs it
    /// and must not take the path table's lock.
    std::map<std::pair<SdfPath, TfToken>, uint32_t> headOverrideSlotsByName;
    /// Per version id: its value, and whether its chain was valid when it
    /// was written.
    std::vector<RigExecBakedPropertyValue> propertyValues;
    std::vector<char> propertyVersionValid;
    /// Per PropertyVersion id, records included: moved this run.
    std::vector<char> propertyChanged;
    /// Per chain: valid this run, and its final version as published.
    std::vector<char> chainValid;
    std::vector<VtValue> chainFinal;
    /// Per record: its value, and whether an override on one of its hops
    /// stands it aside this run.
    std::vector<VtValue> recordValues;
    std::vector<int> propertyRecordById; ///< immutable PropertyResult id to record index, -1 for chain versions
    std::vector<char> recordStoodAside;
    /// Independent reference publication markers, enabled only for checks.
    std::optional<RigExecOraclePublicationContext> oraclePublications;
    std::map<SdfPath,RigExecWeightReferenceContext> oracleWeightInputs;
    /// Per override slot: the interactive override standing there this run
    /// (empty when none), last run's, and whether it moved.
    std::vector<VtValue> headOverrides, lastHeadOverrides;
    std::vector<char> headOverrideMoved;
    /// What the head leaves were last sampled at, and the last tier run.
    bool headLeavesSampled = false;
    UsdTimeCode headLeafTime = UsdTimeCode::Default();
    uint64_t headLeafStamp = 0;
    bool headEverRan = false;
    uint64_t headStamp = 0;
    /// Test observables: head leaves re-read and head ops executed since
    /// Build.
    uint64_t headLeafSamples = 0;
    uint64_t headOpsRun = 0;
    /// The frozen samplers' table of the constant head leaves, kept while
    /// the state it was read under stands (RigExecHeadLeafConstants). UI
    /// thread only, written through a const program by the samplers; never
    /// cloned, because a snapshot never samples. A re-read whose every
    /// entry is bitwise the standing one keeps the standing table, so
    /// frames sampled on either side of an edit that left the constants
    /// alone share one.
    mutable std::shared_ptr<const RigExecHeadLeafConstants>
        headLeafConstants;
    /// The state headLeafConstants was last read under: the program stamp,
    /// the evaluator's stage edit serial and whether the time was Default.
    mutable uint64_t headLeafConstantsStamp = 0;
    mutable uint64_t headLeafConstantsSerial = 0;
    mutable bool headLeafConstantsDefault = false;
    /// The reads after the head tier that a chain result or a record can
    /// answer (RigExecBakedReaderWalk), bound at Build; per walk, whether
    /// something it depends on moved this run (RigExecBakedNoteReaderWalks)
    /// and whether the value read through it did (set by the samplers).
    std::vector<RigExecBakedReaderWalk> readerWalks;
    std::vector<char> readerWalkMoved, readerWalkChanged;
    /// Per provider slot, the PropertyVersion ids its chain-routed avar
    /// binding declares. The avar table is compared by value, so these
    /// validate and re-sample; they seed nothing.
    std::vector<std::vector<uint32_t>> avarHeadReads;
};


inline bool RigExecBakedIsHeadDomain(RigExecBakedSlotDomain domain)
{
    return domain == RigExecBakedSlotDomain::PropertyResult ||
           domain == RigExecBakedSlotDomain::Rest ||
           domain == RigExecBakedSlotDomain::Ladder ||
           domain == RigExecBakedSlotDomain::SkinTopology;
}

inline std::vector<uint32_t>
RigExecBakedHeadIndices(const RigExecBakedProgramImpl &B)
{
    std::vector<uint32_t> indices;
    for (size_t i = 0; i < B.steps.size(); ++i)
        if (B.steps[i].isHead) indices.push_back(uint32_t(i));
    return indices;
}

/// \p input's sampled leaf, or its constant when it has none.
template <class T>
inline T
RigExecBakedLeaf(const RigExecBakedProgramImpl &program,
                 const RigExecBakedInput<T> &input)
{
    if (input.leaf < 0) {
        return input.constant;
    }
    return T(program.leaves.Of<T>().value[size_t(input.leaf)]);
}

bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, double *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, float *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, int *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, bool *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, TfToken *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, GfMatrix4d *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, GfVec3d *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, GfVec3f *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, VtVec3fArray *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, VtArray<GfVec2f> *out);

/// The read every step body and the prologue's input fill make: the value
/// RigExecBakedRead answered for \p input when the leaf was last sampled,
/// which the re-sample rules make this run's answer. No stage, no resolved
/// inputs, no lock.
template <class T>
inline T
RigExecBakedLeafRead(const RigExecBakedProgramImpl &program,
                     const RigExecBakedInput<T> &input)
{
    TF_VERIFY(input.leaf >= 0 || !input.varying);
    T value=input.walk>=0?input.constant:RigExecBakedLeaf(program,input);
    if(input.walk>=0) RigExecBakedResolveReaderWalk(program,input.walk,&value);
    return value;
}

/// The path leaves path leaf \p ref belongs to, or null when the program no
/// longer holds its owner.
inline RigExecBakedPathLeaves *
RigExecBakedPathLeavesOf(RigExecBakedProgramImpl *program,
                         const RigExecBakedPathLeafRef &ref)
{
    switch (ref.owner) {
    case RigExecBakedPathLeafOwner::Provider:
        return &program->providerLeaves;
    case RigExecBakedPathLeafOwner::Revision:
        if (ref.a < program->chains.size() &&
            ref.b < program->chains[ref.a].revisions.size()) {
            return &program->chains[ref.a].revisions[ref.b].leaves;
        }
        return nullptr;
    case RigExecBakedPathLeafOwner::Derived:
        if (ref.a < program->chains.size() &&
            ref.b < program->chains[ref.a].derived.size()) {
            return &program->chains[ref.a].derived[ref.b].revision.leaves;
        }
        return nullptr;
    case RigExecBakedPathLeafOwner::Weight:
        if (ref.a < program->weightObjects.size()) {
            return &program->weightObjects[ref.a].pointLeaves;
        }
        return nullptr;
    case RigExecBakedPathLeafOwner::WeightOracle:
        return ref.a < program->weightObjects.size()
            ? &program->weightObjects[ref.a].oracleLeaves : nullptr;
    case RigExecBakedPathLeafOwner::RevisionLayout:
        if (ref.a < program->chains.size() &&
            ref.b < program->chains[ref.a].revisions.size()) {
            return &program->chains[ref.a].revisions[ref.b].layoutLeaves;
        }
        return nullptr;
    case RigExecBakedPathLeafOwner::DerivedLayout:
        if (ref.a < program->chains.size() &&
            ref.b < program->chains[ref.a].derived.size()) {
            return &program->chains[ref.a].derived[ref.b].revision.layoutLeaves;
        }
        return nullptr;
    }
    return nullptr;
}

inline const RigExecBakedPathLeaves *
RigExecBakedPathLeavesOf(const RigExecBakedProgramImpl &program,
                         const RigExecBakedPathLeafRef &ref)
{
    return RigExecBakedPathLeavesOf(
        const_cast<RigExecBakedProgramImpl *>(&program), ref);
}

/// Every writer of a provider leaf's value or blocked flag calls this when
/// either moved, so the next publication re-keys leaf \p k.
inline void
RigExecBakedNoteSpaceLeafSampled(RigExecBakedProgramImpl *program, size_t k)
{
    if (k < program->spaceLeafRekey.size()) {
        program->spaceLeafRekey[k] |= kRigExecSpaceLeafSampled;
    }
}

/// Sets leaf \p id's `mustSample` byte, so the next sample re-reads it. An
/// id past the binding leaves names a path leaf.
inline void
RigExecBakedMarkLeaf(RigExecBakedProgramImpl *program, uint32_t id)
{
    if (id >= program->leafRefs.size()) {
        const size_t i = id - program->leafRefs.size();
        if (i >= program->pathLeafRefs.size()) {
            const size_t h = i - program->pathLeafRefs.size();
            if (h < program->headLeaves.size()) {
                program->headLeaves[h].mustSample = 1;
            }
            return;
        }
        const RigExecBakedPathLeafRef &ref = program->pathLeafRefs[i];
        RigExecBakedPathLeaves *leaves = RigExecBakedPathLeavesOf(program, ref);
        if (leaves && ref.key < leaves->mustSample.size()) {
            leaves->mustSample[ref.key] = 1;
        }
        return;
    }
    const RigExecBakedLeafRef &ref = program->leafRefs[id];
    program->leaves.ForEach([&ref](auto &pool) {
        using T = typename std::decay_t<decltype(pool)>::Type;
        if (RigExecBakedLeafTraits<T>::type == ref.type &&
            ref.index < pool.mustSample.size()) {
            pool.mustSample[ref.index] = 1;
        }
    });
}

/// Whether binding leaf \p id (a `leafRefs` id) moved in this run's sample.
inline bool
RigExecBakedLeafChanged(const RigExecBakedProgramImpl &program, uint32_t id)
{
    if (id >= program.leafRefs.size()) {
        return false;
    }
    const RigExecBakedLeafRef &ref = program.leafRefs[id];
    const auto at = [&ref](const auto &pool) {
        return ref.index < pool.changed.size() &&
               pool.changed[ref.index] != 0;
    };
    const RigExecBakedLeafPools &pools = program.leaves;
    switch (ref.type) {
    case RigExecBakedLeafType::Double: return at(pools.Of<double>());
    case RigExecBakedLeafType::Float: return at(pools.Of<float>());
    case RigExecBakedLeafType::Int: return at(pools.Of<int>());
    case RigExecBakedLeafType::Bool: return at(pools.Of<bool>());
    case RigExecBakedLeafType::Token: return at(pools.Of<TfToken>());
    case RigExecBakedLeafType::Matrix4d: return at(pools.Of<GfMatrix4d>());
    case RigExecBakedLeafType::Vec3d: return at(pools.Of<GfVec3d>());
    case RigExecBakedLeafType::Vec3f: return at(pools.Of<GfVec3f>());
    }
    return false;
}

/// Numbers one leaf per binding _ForEachPatchableInput visits, in its order,
/// seeded with the binding's constant, and files each under every path
/// `overridableInputs` gives its override number. Then numbers every path
/// leaf after them (`pathLeafRefs`), filed under every path its read can
/// reach, and every head leaf after those, filed under its path. Renumbers
/// from scratch, so Build calls it again once every binding exists.
const VtValue *RigExecBakedSpaceLeafOverlay(const RigExecBakedProgramImpl &, size_t);
GfMatrix4d RigExecBakedProviderParentRaw(const RigExecBakedProgramImpl &,int slot);
bool RigExecBakedBuildSpaces(RigExecBakedProgramImpl *program, UsdTimeCode capture,
    std::string *error, const RigExecSceneDescriptors *scene = nullptr,
    const UsdStageWeakPtr &sceneStage = UsdStageWeakPtr(), uint64_t sceneSerial = 0);
bool RigExecBakedBindOwnPropertySpaces(RigExecBakedProgramImpl *program, std::string *error);
void RigExecBakedBuildSpaceSteps(RigExecBakedProgramImpl *program);
void RigExecBakedSampleSpaces(RigExecBakedProgramImpl *program, UsdTimeCode time, bool all);
void RigExecBakedRunSpaceOp(RigExecBakedProgramImpl *program, RigExecBakedStep *step);
void RigExecBakedPlanProviderRefreshes(RigExecBakedProgramImpl *program);
bool RigExecBakedBindProviderRefresh(RigExecBakedProgramImpl *program,uint32_t index,
    std::vector<uint32_t> *base,std::vector<uint32_t> *fin,
    const std::function<uint32_t(bool,int)> &allocate);
void RigExecBakedRunProviderRefresh(RigExecBakedProgramImpl *program,RigExecBakedStep *step);

void RigExecBakedNumberLeaves(RigExecBakedProgramImpl *program);

/// Resolves \p leaves' keys on \p stage at Build: each key's attribute, the
/// paths its read can reach and whether it can move with the time; seeds
/// every value with its key's fallback. Owning thread.
void RigExecBakedBindPathLeaves(const UsdStageRefPtr &stage,
                                RigExecBakedPathLeaves *leaves);

/// Restarts \p leaves' content versions at their current values: version
/// 0, which no run has read yet.
void RigExecBakedResetPathLeafVersions(RigExecBakedPathLeaves *leaves);

/// Stores \p value as key \p k of \p leaves, \p run being the program's
/// `pathLeafRun`. The key's version moves exactly when the bytes differ
/// from those the last run to read it saw (RigExecBakedHeadValueSame), so
/// writes between two runs that end on the same bytes leave it unmoved.
/// Returns whether the bytes differ from the value replaced. Every writer
/// of a geometry owner's `values` goes through here.
bool RigExecBakedSetPathLeaf(RigExecBakedPathLeaves *leaves, size_t k,
                             VtValue value, uint64_t run);

/// Whether operation keys carry the content versions of \p ref's owner's
/// leaves in place of their values: every geometry owner. Provider leaves
/// keep their values.
inline bool
RigExecBakedPathLeafVersioned(const RigExecBakedPathLeafRef &ref)
{
    return ref.owner != RigExecBakedPathLeafOwner::Provider;
}

/// Re-reads, through RigExecSampleRevisionLeaf over the generation's
/// resolved inputs at \p time, every key of \p leaves that one of these says
/// can have moved since its last sample, and sets its `changed` byte:
///  1. the first sample, a moved program stamp, or \p all (a forced run);
///  2. an interactive override standing now or at the last sample: a drag on
///     any hop, numbered or routed, reaches the read through the resolved
///     inputs, so every key re-reads while one stands and once after; a
///     topology key (`epoch`) re-reads only while one stands on one of its
///     hops, the only paths its read consults the overlay at, and once
///     after;
///  3. a moved time, for a key read at the time that can vary with it (or
///     when the time moves to or from Default);
///  4. its `mustSample` byte: a value edit reached one of its paths
///     (ApplyValueEdits files it under each), and its time variance is
///     re-derived with the read;
///  5. moved chain results (`pathLeafChainSerial`).
/// A key with a reader walk is read through it
/// (RigExecBakedSampleWalkedPathLeaf), not through the resolved inputs.
/// Keys in \p skip are not read; each is marked to re-read when it next is
/// not skipped. Every other key keeps its value. Owning thread, prologue
/// only.
void RigExecBakedSamplePathLeaves(RigExecBakedProgramImpl *program,
                                  RigExecBakedPathLeaves *leaves,
                                  UsdTimeCode time, bool all,
                                  const std::vector<int> &skip = {});

/// Whether key \p k of \p leaves can read another value at another time,
/// as the next sample answers it: `varying[k]` while that answer is current
/// (RigExecBakedPathLeaves::VarianceStale false), else the rebind's own
/// question (RigExecRevisionLeafHops) asked again without storing it.
/// Owning thread.
bool RigExecBakedLeafVaryingNow(const RigExecBakedProgramImpl &program,
                                const RigExecBakedPathLeaves &leaves,
                                size_t k);

/// Re-reads, through RigExecBakedRead at \p time, every leaf that one of
/// these says can have moved since it was last sampled, and sets its
/// `changed` byte by bitwise comparison:
///  1. the first run of the program, a moved program stamp, or \p all from
///     Build or a forced run;
///  2. \p all from a frozen job, whose clone carries live's flags and time;
///  3. a moved time, for a varying binding;
///  4. an override on its number now or at the last run: while a drag
///     stands and once after it lifts, since a drag scrubbed at a held frame
///     keeps both flags set;
///  5. an edit on its number (`edited`), or its `mustSample` byte: a value
///     edit that reached one of its paths (ApplyValueEdits), or
///     RigExecProgramAvarPatch;
///  6. a value in `routedOverrides` that differs from `lastRoutedOverrides`
///     on one of its paths;
///  7. for a binding with a reader walk, a head leaf, override slot or
///     declared version of that walk that moved (`readerWalkMoved`);
///  8. an upstream value placed, moved or lifted on one of its paths
///     (RigExecBakedPlaceUpstream sets `mustSample`).
/// Every other leaf keeps its value: none of its read's inputs can have
/// moved for it. A binding with a reader walk is read through it
/// (RigExecBakedReadWalked), any other through RigExecBakedRead, the long
/// way through the upstream layer while a value stands on its number
/// (`upstreamOn`). Owning thread, prologue only.
///
/// \p pass splits the live prologue around the head tier: BeforeHead reads
/// every leaf but the chain-routed ones (a binding with `resolvedAttr` or a
/// reader walk), whose walks no chain result can reach, and applies rule 6;
/// ChainRouted reads the rest once the property chains are published and
/// RigExecBakedNoteReaderWalks has run. All does both, in one pass.
enum class RigExecBakedLeafPass : uint8_t { All, BeforeHead, ChainRouted };

/// Prologue step 1's upstream half, after the resolved inputs are cleared
/// and before the interactive overrides are placed. Compares `upstream`
/// with `lastUpstream` by value; for every path placed, moved or lifted
/// (rule 8) it marks the leaves `leafByPath` files there and sets
/// `upstreamChanged` for the override numbers filed there, and it rebuilds
/// `upstreamOn`. Then places every value on an `upstreamOracle` path into
/// the resolved inputs. `upstreamChanged` holds this run's moves only.
/// Runs on the thread that owns \p program: live's owning thread, or a
/// frozen worker over its private clone, which passes \p placeOracle false:
/// it builds no admission set (that reads the stage), and the freeze
/// refuses every rig with an object the oracle resolves.
void RigExecBakedPlaceUpstream(RigExecBakedProgramImpl *program,
                               bool placeOracle = true);

void RigExecBakedSampleLeaves(
    RigExecBakedProgramImpl *program, UsdTimeCode time, bool all,
    RigExecBakedLeafPass pass = RigExecBakedLeafPass::All);

/// Emits one PropertyRevision head step per part of every property chain
/// (part 0 the base, part k revision k), numbers the PropertyVersion domain
/// (`versionBase`, record ids) and sizes the head tier's run state. Each
/// step declares the head leaves and override slots its body reads and,
/// as head reads, the version before it and every chain version or record
/// one of its walks meets. Build only.
void RigExecBakedBuildPropertySteps(RigExecBakedProgramImpl *program);

/// Re-reads every head leaf that can have moved since its last sample and
/// sets its `changed` byte by bitwise comparison:
///  1. the first sample, a moved program stamp, or \p all (a forced run);
///  2. its `mustSample` byte (a value edit reached its path), which also
///     re-derives whether it varies;
///  3. a moved time, for a leaf that varies (or a time moving to or from
///     Default).
/// A head leaf is a raw stage value, so overrides and chain results never
/// move it: the walk resolver reads those beside it. Owning thread,
/// prologue only.
void RigExecBakedSampleHeadLeaves(RigExecBakedProgramImpl *program,
                                  UsdTimeCode time, bool all);

/// \p leaf's value at \p time as RigExecBakedSampleHeadLeaves reads it: the
/// attribute's own typed value, empty when it has none of the leaf's type.
/// Reads the stage: owning thread (the live prologue, or the frozen sampler
/// on the UI thread).
VtValue RigExecBakedReadHeadLeaf(const RigExecBakedHeadLeaf &leaf,
                                 UsdTimeCode time);

/// Whether \p value holds exactly \p leaf's type.
bool RigExecBakedHeadLeafHolds(const RigExecBakedHeadLeaf &leaf,
                               const VtValue &value);

/// Whether \p leaf's attribute might hold a different value at another
/// time: the predicate RigExecBakedSampleHeadLeaves re-derives `varying`
/// with. A leaf that answers false reads one value at every numeric time
/// code. Reads the stage: owning thread.
bool RigExecBakedHeadLeafVaries(const RigExecBakedHeadLeaf &leaf);

/// The body of one PropertyRevision head step: part 0 sets the base and the
/// chain's valid byte, part k applies revision k. Each is the loop body of
/// _EvaluatePropertyChains it replaces (rigEvaluatorProperties.cpp), over
/// head leaves, overrides and earlier versions; its lines go to
/// `step->lines`, and it sets the `propertyChanged` byte of the version it
/// writes. A weight-object envelope consumes its declared WeightField result
/// inside RigExecVolatileRead. Owning thread.
void RigExecBakedRunPropertyStep(RigExecBakedProgramImpl *program,
                                 RigExecBakedStep *step,
                                 UsdTimeCode time);

/// Publishes chains [\p begin, \p end) as _EvaluatePropertyChains leaves
/// them: each valid chain's final version at its target and each record
/// not standing aside at its consumer, into the generation's resolved
/// inputs and, when \p results, into `propertyResults`. A skipped chain
/// publishes nothing, so an override on its target stays. Runs after the
/// head tier on every run, whether its ops ran or replayed.
void RigExecBakedPublishPropertyChains(RigExecBakedProgramImpl *program,
                                       size_t begin, size_t end,
                                       bool results);
inline void
RigExecBakedPublishPropertyChains(RigExecBakedProgramImpl *program)
{
    RigExecBakedPublishPropertyChains(program, 0,
                                      program->propertyChains.size(), true);
}

/// Places this run's interactive overrides into the head tier's override
/// slots (the last at a path wins, as the overlay takes them), marks each
/// slot whose value moved since the last run and each record an override
/// stands aside, and clears every PropertyVersion `changed` byte except a
/// record's whose hops' overrides moved. The head tier's first act on every
/// run.
void RigExecBakedPlaceHeadOverrides(RigExecBakedProgramImpl *program);

/// After head step \p step ran or replayed: when the version it writes
/// moved, the records that read that version (value and `changed` byte)
/// and, for a chain's last part, the chain's final value.
void RigExecBakedFinishPropertyStep(RigExecBakedProgramImpl *program,
                                    const RigExecBakedStep &step);

/// Whether two head values are the same bit for bit (a signed zero or a NaN
/// payload is a difference); empty equals only empty.
bool RigExecBakedHeadValueSame(const VtValue &a, const VtValue &b);

/// Each of \p paths spelled once, on the owning thread, for
/// RigExecBakedProgramImpl::pathTexts.
inline std::shared_ptr<const std::vector<std::string>>
RigExecBakedSpellPathTexts(const std::vector<SdfPath> &paths)
{
    auto texts = std::make_shared<std::vector<std::string>>();
    texts->reserve(paths.size());
    for (const SdfPath &path : paths) {
        texts->push_back(path.GetString());
    }
    return texts;
}

/// What reader walk \p walk answers after this run's head tier and
/// publication: RigExecResolvedInputs::GetAttribute over the published
/// overlay, restated over head leaves, override slots, chain finals and
/// records. False, leaving \p out alone, where that read answers nothing.
/// Pure: no stage, no path work, no lock; a frozen worker calls it.
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, double *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, float *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, int *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, bool *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, TfToken *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, GfMatrix4d *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, GfVec3d *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, GfVec3f *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, VtVec3fArray *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, VtArray<GfVec2f> *out);

/// RigExecBakedRead for a binding with a reader walk: the walk read while
/// an override or an upstream value (\p upstreamOn) stands on its number or
/// the binding varies (the walk is the one from its head, over head leaves
/// that read the upstream layer), else its constant.
template <class T>
inline T
RigExecBakedReadWalked(const RigExecBakedProgramImpl &program,
                       const RigExecBakedInput<T> &input,
                       const std::vector<char> &overridden,
                       const std::vector<char> *upstreamOn = nullptr)
{
    T value = input.constant;
    const auto on = [&input](const std::vector<char> &flags) {
        return input.overrideIndex >= 0 &&
               size_t(input.overrideIndex) < flags.size() &&
               flags[size_t(input.overrideIndex)];
    };
    const bool flagged =
        on(overridden) || (upstreamOn && on(*upstreamOn));
    if (flagged || input.varying) {
        RigExecBakedResolveReaderWalk(program, input.walk, &value);
    }
    return value;
}

/// Whether a move of version \p id is shadowed for a reader by \p shadowed
/// (a reader walk's or a step's): it is read only past records, and none
/// of them stands aside this run.
bool RigExecBakedReadIsShadowed(
    const RigExecBakedProgramImpl &program,
    const std::vector<std::pair<uint32_t, uint32_t>> &shadowed, uint32_t id);

/// After the head tier and its publication: marks each reader walk whose
/// head leaves, override slots or declared versions moved this run
/// (`readerWalkMoved`), and clears `readerWalkChanged`. Owning thread.
void RigExecBakedNoteReaderWalks(RigExecBakedProgramImpl *program);

/// RigExecSampleRevisionLeaf for a path-leaf key whose read has reader walk
/// \p walk, over the head tier's state: Resolved reads the walk, else the
/// head's own value, else the key's fallback; ResolvedOnly the walk, else
/// the fallback; Dial the walk as the dial attribute's type, cast to
/// double. Pure.
VtValue RigExecBakedSampleWalkedPathLeaf(
    const RigExecBakedProgramImpl &program, const RigExecRevisionLeafKey &key,
    int walk);

/// Fills every step's `reads` from the reader walks in its
/// `readerWalks`, after RigExecBakedDeclareInputDependencies, and
/// `avarHeadReads`. Build only.
void RigExecBakedDeclareHeadReads(RigExecBakedProgramImpl *program);

/// Appends to every region step's `reads` the Rest and Ladder slots its
/// body indexes: ComposeSubtree the ladders of its group, of the parents
/// it composes against and of every switch source, space and recomposed
/// slot; Solve the rests of its `restSlots`; Constraint the rests of its
/// IK targets (without animated translations) and of a matrix mover's
/// sources, and the ladder of its space; ProviderMatrix and FrameMatrix
/// their slot's rest; PoseInterpolator its driver's and parent's rests;
/// Derived the rests of its projector slots. Build only, after
/// RigExecBakedDeclareHeadReads.
std::vector<RigExecBakedSlotRange> RigExecBakedRequiredRestReads(
    const RigExecBakedProgramImpl &program, const RigExecBakedStep &step);

void RigExecBakedDeclareRestReads(RigExecBakedProgramImpl *program);

/// The region half of the head tier's validation, once the steps exist:
/// every head output a step or an avar slot declares has a head producer,
/// and every reader walk a step reads declares each chain target and record
/// consumer it meets ("walk <path> meets chain target without declaring
/// it"). False with the first violation in \p error.
bool RigExecBakedValidateHeadReads(const RigExecBakedProgramImpl &program,
                                   std::string *error);

/// The matrix table \p revision reads at its own phase: FinalMatrix for a
/// final-phase revision, BaseMatrix otherwise.
inline RigExecBakedSlotDomain
RigExecBakedOwnMatrixDomain(
    const RigExecBakedProgramImpl::GeomRevision &revision)
{
    return revision.finalPhase ? RigExecBakedSlotDomain::FinalMatrix
                               : RigExecBakedSlotDomain::BaseMatrix;
}

/// Calls visit(domain, slot) for every provider matrix \p revision reads.
/// Transform, transformSpace, carrySpace and influences are read at the
/// revision's own phase; for a matrix target (RigExecIsDerivedMatrixOp) the
/// first three are visited at BOTH phases, because its projector frames are
/// built from base and final of all three (RigExecBakedProjectorFrames). The
/// ProviderMatrix need tables and the steps' declared reads both come from
/// here.
template <class Visit>
void
RigExecBakedForEachMatrixRead(
    const RigExecBakedProgramImpl::GeomRevision &revision, Visit &&visit)
{
    const RigExecBakedSlotDomain own = RigExecBakedOwnMatrixDomain(revision);
    const bool bothPhases = RigExecIsDerivedMatrixOp(revision.op);
    for (const int slot : {revision.transformSlot,
                           revision.transformSpaceSlot,
                           revision.carrySpaceSlot}) {
        if (slot < 0) continue;
        if (bothPhases) {
            visit(RigExecBakedSlotDomain::BaseMatrix, slot);
            visit(RigExecBakedSlotDomain::FinalMatrix, slot);
        } else {
            visit(own, slot);
        }
    }
    for (const int slot : revision.influenceSlots) {
        if (slot >= 0) visit(own, slot);
    }
}

// What committing an input records, and where it can be recorded.
// A committed input records two kinds of fact. Its override NUMBER is
// order-bearing: numbers are handed out from one running counter, so the
// order inputs are committed in IS the numbering the program carries, and
// the entry of `overridableInputs` a number is appended to is ordered by it.
// Everything else is order-free -- two counts that sum, and paths landing
// in the sets of the invalidation index, where inserting a path a second
// time is a no-op whenever it happens.
// RigExecBakedRecordBind and RigExecBakedRecordFold state both kinds once,
// against a sink that receives them, and there are two sinks:
//  * RigExecBakedProgramSink writes straight into the program, numbering
//    from `overridden.size()`. This is the serial commit, CommitBind's, one
//    input at a time in program order.
//  * RigExecBakedCommitShard gathers one CHUNK of a parallel pass into
//    vectors, numbering from zero within the chunk. A chunk's first real
//    number is the running counter at the phase's entry plus every number
//    the chunks before it handed out -- a prefix sum, known only once every
//    chunk has counted -- and RigExecBakedMergeCommitShards adds it.
// So a parallel commit cannot drift from the serial one: the rule is
// written here once and only WHERE the facts land differs. Chunks are
// contiguous runs of the program order and are merged in that order, which
// is what makes the numbers, and each `overridableInputs` entry's order,
// the serial ones exactly.

/// The serial sink: every fact straight into \p program.
struct RigExecBakedProgramSink {
    RigExecBakedProgramImpl *program = nullptr;

    int Number()
    {
        const int index = int(program->overridden.size());
        program->overridden.push_back(0);
        return index;
    }
    void Overridable(const SdfPath &path, int index)
    {
        program->overridableInputs[path].push_back(index);
    }
    void Bound(bool varying)
    {
        ++program->boundInputs;
        if (varying) ++program->varyingInputs;
    }
    void Prim(const SdfPath &path) { program->prims.insert(path); }
    void Named(const SdfPath &path) { program->named.insert(path); }
    void Rebuild(const SdfPath &path) { program->rebuild.insert(path); }
    void Folded(const SdfPath &path) { program->folded.insert(path); }
};

/// The parallel sink: one chunk's facts, gathered on whichever thread runs
/// the chunk and touching nothing shared.
struct RigExecBakedCommitShard {
    SdfPathVector prims, named, rebuild, folded;
    /// (path, number RELATIVE to the chunk's first) per overridable walk
    /// step, in the order the program sink would have appended them.
    std::vector<std::pair<SdfPath, int>> overridable;
    int numbered = 0;
    size_t boundInputs = 0;
    size_t varyingInputs = 0;

    int Number() { return numbered++; }
    void Overridable(const SdfPath &path, int index)
    {
        overridable.emplace_back(path, index);
    }
    void Bound(bool varying)
    {
        ++boundInputs;
        if (varying) ++varyingInputs;
    }
    void Prim(const SdfPath &path) { prims.push_back(path); }
    void Named(const SdfPath &path) { named.push_back(path); }
    void Rebuild(const SdfPath &path) { rebuild.push_back(path); }
    void Folded(const SdfPath &path) { folded.push_back(path); }

    /// Sorts and dedups the four path vectors, and sorts `overridable` by
    /// (path, number), so the merge inserts ascending runs. Run at the end
    /// of the chunk, on its own thread: this is the part of the set
    /// insertion that CAN be parallel.
    void Seal();
};

/// Folds \p shards into \p program in chunk order, and returns each shard's
/// first override number.
///
/// Shard k's numbers start where shard k-1's stop, and shard 0's where the
/// program's running counter stands on entry -- this phase continues the
/// numbering the phases before it started, it does not restart it. The
/// caller adds shard k's first number to every input shard k numbered.
std::vector<int> RigExecBakedMergeCommitShards(
    RigExecBakedProgramImpl *program,
    std::vector<RigExecBakedCommitShard> *shards);

/// Registers \p input, read as \p name on \p prim through \p walk, into
/// \p sink: the bound/varying counts, the invalidation index, and -- for an
/// input with a head -- an override number and the paths an override on it
/// can stand on. What CommitBind records, for either sink.
template <class Sink, class T>
void
RigExecBakedRecordBind(Sink *sink, const UsdPrim &prim, const TfToken &name,
                       RigExecBakedInput<T> *input, const SdfPathVector &walk)
{
    if (prim && prim.GetAttribute(name)) {
        sink->Bound(input->varying);
    }
    if (prim) {
        sink->Prim(prim.GetPath());
        // Named even when absent: a resync that CREATES this property is
        // an input appearing, and the bake captured the default it did
        // not find.
        sink->Named(prim.GetPath().AppendProperty(name));
    }
    // The registration proper: an input with no head has nothing an
    // override could stand on, and no number.
    if (!input->head) {
        return;
    }
    input->overrideIndex = sink->Number();
    // EVERY attribute on the resolution walk, not only the head one.
    // RigExecBakedClassifyInput followed an authored connection chain and
    // may have captured the value several hops upstream; an interactive
    // override standing on one of those hops is an override on this input,
    // and GetAttribute -- which is what the flag makes RigExecBakedRead use
    // -- consults the generation's resolved values at every step of the
    // same walk. Keyed by the head alone, such an override found no binding
    // and was reported placeable with nothing placed.
    for (const SdfPath &path : walk) {
        sink->Overridable(path, input->overrideIndex);
    }
    if (walk.empty()) {
        sink->Overridable(input->head.GetPath(), input->overrideIndex);
    }
    sink->Prim(input->head.GetPrim().GetPath());
    for (const SdfPath &path : walk) {
        sink->Named(path);
    }
    // A chain-resolved input redoes the whole walk live every frame, so
    // nothing about it was captured. A query was pinned to ONE attribute of
    // the walk, so every other attribute on it decided that choice; with a
    // walk of one there is no choice left to invalidate, and a value moving
    // on it -- including being cleared, which drops the input back to the
    // same fallback both paths use -- is answered by the query itself.
    const bool live = input->sourceBacked || input->resolvedAttr || (input->varying &&
                                              walk.size() == 1);
    if (live) {
        return;
    }
    for (const SdfPath &path : walk) {
        sink->Rebuild(path);
    }
}

/// The serial commit's spelling; the parallel lanes pass interned tokens.
template <class Sink, class T>
void
RigExecBakedRecordBind(Sink *sink, const UsdPrim &prim, const char *name,
                       RigExecBakedInput<T> *input, const SdfPathVector &walk)
{
    RigExecBakedRecordBind(sink, prim, TfToken(name), input, walk);
}

/// Records \p name on \p prim as read for its VALUE into \p sink; see
/// RigExecBakedBuildContext::Fold, which is this into the program.
template <class Sink>
void
RigExecBakedRecordFold(Sink *sink, const UsdPrim &prim, const TfToken &name)
{
    if (!prim) {
        return;
    }
    const SdfPath path = prim.GetPath().AppendProperty(name);
    sink->Rebuild(path);
    sink->Folded(path);
    sink->Named(path);
    sink->Prim(prim.GetPath());
}

template <class Sink>
void
RigExecBakedRecordFold(Sink *sink, const UsdPrim &prim, const char *name)
{
    if (!prim) {
        return;
    }
    RigExecBakedRecordFold(sink, prim, TfToken(name));
}

// The compiled epoch, in terms the program can name.
// RigExecRigEvaluator's compiled structures are private nested types, so only
// bakedProgram.cpp can read them. It restates the parts each domain's bake
// needs as the plain records below and hands those over; the per-domain bake
// functions then depend on the SHAPE of the epoch rather than on the
// evaluator, which is what keeps the friendship to one file.

/// One frame constraint of the compiled walk.
struct RigExecBakedConstraintSpec {
    SdfPath moverPath;
    TfToken schemaType;
    SdfPathVector targets;
    /// Per target, whether a read phase asked for its frame as of this
    /// constraint (the evaluator's _nativePhaseCheckpoints membership). A
    /// SingleChainIK names its whole chain here, and the dynamic walk
    /// records every one of them.
    std::vector<char> snapshotTargets;
    /// SingleChainIK: the inferred joint chain, first joint through end, and
    /// the two bindings it resolves beside its sources.
    SdfPathVector ikChain;
    /// Parallel to ikChain: 1 where a pose step BELOW this constraint wrote
    /// that joint, so the chain measures from what that step left (spec 4.2).
    std::vector<char> ikRestLive;
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
    /// rigExec:space on a rotation constraint; empty when none is named.
    SdfPath space;
    /// rigExec:blendShear and rigExec:worldUpRotationOnly, compiled.
    bool blendShear = false;
    bool worldUpRotationOnly = false;
    /// rigExec:weightBlend == "radial" (transform-domain matrix mover).
    bool radialBlend = false;
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
    /// rigExec:pointFrame == "posed": the points this cluster moves
    /// were already carried into the rig's posed frame by a skin. See
    /// RigExecClusterInPointFrame.
    bool transformPosedPoints = false;
    bool skinTopologyFixed = false;
    /// Whether a read phase asked for the target's points as of this
    /// revision (the evaluator's _nativePhaseCheckpoints membership).
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
    /// Parallel to solverJoints: whether a read phase asked for that joint's
    /// frame as of that solver (the evaluator's _nativePhaseCheckpoints
    /// membership).
    std::vector<std::vector<char>> solverSnapshotJoints;
    /// Per solver, the joints whose REST is LIVE: a pose step below this
    /// solver in the rig's hierarchical stack already wrote them, so the
    /// solver measures from the frame that step left rather than from the
    /// authored rest (spec 4.2). Empty on every rig with no pose step below a
    /// solver that writes one of its joints, which is every shipped rig.
    std::vector<SdfPathVector> solverLiveRestJoints;
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
    /// Every aggregate solver of the epoch, baked or not: a blend input that
    /// names one must already be baked, or the bake is refused.
    std::set<SdfPath> aggregateSolvers;
    std::vector<std::string> *reasons = nullptr;
    bool ok = true;

    /// Records one reason the epoch cannot be expressed, and refuses it.
    void Refuse(const std::string &what, const SdfPath &where);
    /// Records \p name as read for its VALUE: an edit rebuilds the program
    /// and an interactive override on it cannot be placed.
    void Fold(const UsdPrim &prim, const char *name);
    void Fold(const UsdPrim &prim, const TfToken &name);
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
    /// slotKind[slot] == FirstFramePose itself, as the solver rest binding does.
    int SlotOf(const SdfPath &path) const;
    /// The RESOLVING half of Bind: reads the stage and decides what the
    /// channel is, recording nothing.
    ///
    /// Every call is a read of the composed stage and of `chainTargets`, so
    /// calls for distinct channels are independent and may run concurrently
    /// -- which is the point: a rig's providers carry about twenty channels
    /// each and resolving them is most of what Build costs. The compile-time
    /// structure digest already reads this stage from its own thread, so
    /// concurrent readers are not a new assumption here.
    ///
    /// The caller must commit every result afterwards: through CommitBind,
    /// ON ONE THREAD and in program order, or through RigExecBakedRecordBind
    /// into a RigExecBakedCommitShard per contiguous chunk of that order,
    /// merged by RigExecBakedMergeCommitShards. Skipping that leaves the
    /// input unregistered -- no override index, nothing in the invalidation
    /// index -- which is a silently wrong program rather than a failure.
    ///
    /// Concurrent callers pass \p name as an interned token: TfToken(const
    /// char *) takes the token registry's lock on every construction.
    template <class T>
    RigExecBakedInput<T> ResolveBind(const UsdPrim &prim, const TfToken &name,
                                     T fallback, SdfPathVector *walk,
                                     bool sourceValue = false) const;
    template <class T>
    RigExecBakedInput<T> ResolveBind(const UsdPrim &prim, const char *name,
                                     T fallback, SdfPathVector *walk,
                                     bool sourceValue = false) const
    {
        return ResolveBind(prim, TfToken(name), fallback, walk, sourceValue);
    }

    /// The RECORDING half: the counters, the invalidation index and the
    /// override index.
    ///
    /// Order-bearing, and that is why it is separate rather than locked.
    /// It hands out `overrideIndex` from a running counter, so the order
    /// these are committed in IS the numbering a program carries. Call it in
    /// the same order the bindings appear in the program and a parallel
    /// resolve produces a bake identical to a serial one, byte for byte;
    /// call it in completion order and the program still works while no two
    /// builds of one rig agree. A loop with enough inputs to be worth it
    /// commits through shards instead, which keeps the numbering by a prefix
    /// sum rather than by order (see RigExecBakedRecordBind).
    template <class T>
    void CommitBind(const UsdPrim &prim, const char *name,
                    RigExecBakedInput<T> *input, const SdfPathVector &walk);

    /// Binds \p name as a per-frame input, registering it for overrides and
    /// for invalidation. Resolve and commit together, for the call sites
    /// that have no reason to separate them.
    template <class T>
    RigExecBakedInput<T> Bind(const UsdPrim &prim, const char *name,
                              T fallback, bool sourceValue = false);
};

template <class T>
RigExecBakedInput<T>
RigExecBakedBuildContext::Bind(const UsdPrim &prim, const char *name,
                               T fallback, bool sourceValue)
{
    SdfPathVector walk;
    RigExecBakedInput<T> input = ResolveBind(prim, name, fallback, &walk, sourceValue);
    CommitBind(prim, name, &input, walk);
    return input;
}

template <class T>
RigExecBakedInput<T>
RigExecBakedBuildContext::ResolveBind(const UsdPrim &prim, const TfToken &name,
                                      T fallback, SdfPathVector *walk,
                                      bool sourceValue) const
{
    RigExecBakedInput<T> input =
        RigExecBakedBindInput(prim, name, fallback, capture, chainTargets,
                              walk);
    // Only opted-in direct numeric arguments sample ordinary source edits.
    if constexpr (std::is_same_v<T,double> || std::is_same_v<T,GfVec3d>) {
        if (sourceValue && !input.varying && input.head &&
            input.head.GetTypeName().GetType()==TfType::Find<T>() && walk &&
            walk->size()==1 && walk->front()==input.head.GetPath() &&
            !input.head.ValueMightBeTimeVarying() &&
            input.head.GetNumTimeSamples()==0 && !input.head.HasSpline()) {
            input.sourceBacked=true;
            input.sourceFallback=fallback;
            input.query=UsdAttributeQuery(input.head);
        }
    }
    // A selection that moves with the time code is read the long way,
    // through this generation's resolved inputs, rather than through a
    // query pinned to the wrong attribute.
    if (input.varying && input.query.IsValid()) {
        bool viaChain = false, varying = false;
        UsdAttribute atProbe;
        RigExecBakedClassifyInput<T>(prim.GetAttribute(name), probe,
                                     chainTargets, &viaChain, &varying,
                                     &atProbe);
        if (atProbe.GetPath() != input.query.GetAttribute().GetPath()) {
            input.query = UsdAttributeQuery();
            input.resolvedAttr = prim.GetAttribute(name);
        }
    }
    // And a SPLINE is read the long way for the same reason, one step
    // further out: a UsdAttributeQuery pinned to a spline-valued attribute
    // does not follow a later edit to that spline. A query re-reads a time
    // sample and a default -- it holds where the value comes from, and the
    // layer it names still has the new number -- but it resolves the spline
    // itself once and keeps answering from the copy it took, silently and
    // with nothing downstream able to tell.
    // That is not a hypothetical: Animation mode authors a released gizmo
    // drag as a spline knot (gizmoMath.SetAnimated), so the FIRST release on
    // a control creates the property spec -- a resync, which rebakes and
    // pins a fresh query -- and every release after it re-authors that same
    // knot, which is changed-info only. RigExecBakedRecordBind leaves a
    // pinned-query input out of `rebuild` precisely because the query was
    // supposed to answer such an edit, so nothing rebaked and the rig kept
    // publishing the first drag's pose: the control moved under the preview
    // and sprang back the moment the artist let go, until some other
    // control's first release resynced and rebaked the program for it.
    // Reading through the generation's resolved inputs is what the dynamic
    // path does for the same attribute, so the two still agree by
    // construction, and `live` in RigExecBakedRecordBind stays honest: the
    // value really is re-read every frame now.
    if (input.varying && input.query.IsValid() &&
        input.query.GetAttribute().HasSpline()) {
        input.query = UsdAttributeQuery();
        input.resolvedAttr = prim.GetAttribute(name);
    }
    return input;
}

template <class T>
void
RigExecBakedBuildContext::CommitBind(const UsdPrim &prim, const char *name,
                                     RigExecBakedInput<T> *input,
                                     const SdfPathVector &walk)
{
    RigExecBakedProgramSink sink{program};
    RigExecBakedRecordBind(&sink, prim, name, input, walk);
}

// The per-domain halves of Build and Run.
// Build calls the two builders in program order and Run calls the two
// executors in program order; each pair lives in one file so that a domain's
// bake and its frame path are read side by side.

/// Bakes the interleaved solver/constraint walk into \p ctx's program.
void RigExecBakedBuildWalk(RigExecBakedBuildContext *ctx,
                           const std::vector<RigExecBakedWalkEntry> &walk);

/// Bakes the geometry chains and their derived targets into \p ctx's program.
void RigExecBakedBuildGeometry(
    RigExecBakedBuildContext *ctx,
    const std::vector<RigExecBakedChainSpec> &chains);



/// Binds source and namespace parent anchors to typed provider compose outputs.
bool RigExecBakedBindSpaceSwitchVersions(
    RigExecBakedProgramImpl *program,std::string *error=nullptr);
void RigExecBakedRunSpaceCheckpoint(
    RigExecBakedProgramImpl *program,RigExecBakedStep *step);

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

/// The matrix \p record holds this run, rest -> the recorded frame, into
/// \p matrix. False exactly where the walk records nothing: a constraint's
/// exit records no frame (`recordAfter`), or records targets[0] alone
/// (`recordEveryTarget`) and this is another target; a solver published no
/// element for the joint (`present`); or the frame is unusable or does not
/// decompose. \p matrix is written either way.
bool RigExecBakedEvalFrameRecord(const RigExecBakedProgramImpl &program,
                                 const RigExecBakedFrameRecord &record,
                                 GfMatrix4d *matrix);

/// The points \p binding resolves to this run: its first candidate whose
/// chain read a base, as `*points` / `*count`. False for the tail, and then
/// the reader reads the resolved input. Valid until the candidate's chain
/// next runs.
bool RigExecBakedResolvePoints(const RigExecBakedProgramImpl &program,
                               const RigExecBakedPointsBinding &binding,
                               const GfVec3f **points, size_t *count);

/// Sets \p revision's `revisionInputs` to \p resolved overlaid with what each
/// of its point bindings resolves to, and appends the "resolved to nothing"
/// line for each tail that asks for one. The live and frozen assembles both
/// build their overlay here. Does nothing for a revision with no phases.
void RigExecBakedOverlayPointReads(
    RigExecBakedProgramImpl *program,
    RigExecBakedProgramImpl::GeomRevision *revision,
    const RigExecResolvedInputs &resolved,
    std::vector<std::string> *diagnostics);

/// \p revision's packet from its path leaves, for an operation they
/// assemble (`leaves.decl.assembles`): the provider values as the stage
/// assembly takes them, the phase overlay (RigExecBakedOverlayPointReads
/// over the generation's resolved inputs), the blend gather from leaves --
/// which writes each dense sample's `lastPoints`, as the stage gather did --
/// and RigExecAssembleFromLeaves. \p basePoints are the chain's authored
/// base for a chain revision and its final points for a derived one. Reads
/// no stage: the live RevisionStatic and Derived bodies and the frozen
/// RevisionStatic all assemble here. \p missing: see
/// RigExecRevisionLeafView. \p layoutPointCount is the owning chain's raw
/// base cardinality for sparse layouts; the default uses basePointCount.
RigExecMoverParameters RigExecBakedAssembleFromLeaves(
    RigExecBakedProgramImpl *program,
    RigExecBakedProgramImpl::GeomRevision *revision,
    const GfVec3f *basePoints, size_t basePointCount,
    std::vector<std::string> *diagnostics,
    std::vector<std::string> *missing = nullptr,
    size_t layoutPointCount = size_t(-1));

/// Appends one VolumePlacements step per volume provider slot (object =
/// slot, part 1), then one WeightPacket step per weight object in the
/// table's dependency order -- all of it between the pose half and the
/// geometry half.
void RigExecBakedBuildWeightSteps(RigExecBakedProgramImpl *program);

/// Runs one WeightPacket step, under the same rule as the other two.
void RigExecBakedRunWeightStep(RigExecBakedProgramImpl *program,
                               RigExecBakedStep *step, UsdTimeCode time);

/// Replaces \p frames (pose.weightFrames) with one entry per placedVolumes
/// slot, from volumePlacement: the dynamic walk's key set, for live and
/// frozen runs alike, whichever placement steps this run's closure held.
void RigExecBakedPublishVolumePlacements(
    const RigExecBakedProgramImpl &program,
    std::map<SdfPath, GfMatrix4d> *frames);

// Build-only declaration sink; resolved reads are never persistent step state.
struct RigExecBakedDependencySink {
    RigExecBakedStep *step;
    bool resolvedReads = false;
};
std::vector<char> RigExecBakedResolvedReaders(const RigExecBakedProgramImpl &program);

/// Every per-frame input one weight object's step reads.
void RigExecBakedNoteWeightInputs(
    const RigExecBakedProgramImpl::WeightObject &weight,
    RigExecBakedDependencySink *sink);

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

/// Resolves the rest chain of slots [\p begin, \p end) from the bound
/// channels of RigExecBakedProgramImpl::ladders, in slot order, as their
/// leaves were last sampled: `restM`, `restFrames`, `restPts` and
/// `restRoundTrip`. A parent outside the range must be composed already.
///
/// ONE definition, called from Build (once, over every slot, at the capture
/// time) and by the RestCompose head op. computations.cpp resolves the same
/// computations per provider per evaluation; the frame round trips exec
/// performs between them are reproduced, not simplified away, because a
/// deep chain drifts without them.
///
/// \p trackMoves compares each slot's `restM` with `lastRestM` and, where it
/// moved, sets `restChanged`, appends to `restMoved` and updates
/// `lastRestM`. Build passes false and seeds the comparison buffers itself.
void RigExecBakedComposeRestRange(RigExecBakedProgramImpl *program,
                                  int begin, int end, bool trackMoves);

/// The default-space ladder of slots [\p begin, \p end), as above:
/// `posedAuthored`, `posedAuthoredM`, `selfD`, `defaultRoundTrip`,
/// `parentDinv` and `rotOrder`. Reads the range's rests and its parents'
/// rests and ladders, which must be composed already. \p trackMoves
/// compares `selfD`, `parentDinv`, `posedAuthored`, `posedAuthoredM` and
/// `rotOrder` with their `last` buffers and records a move in
/// `ladderChanged` and `ladderMoved`.
void RigExecBakedComposeLadderRange(RigExecBakedProgramImpl *program,
                                    int begin, int end, bool trackMoves);

/// One RestCompose (part 0) and one LadderCompose (part 1) head op per
/// compose group, appended to `steps`, with their declared reads: the
/// parent rests and ladders outside the group, the group's own rests for
/// the ladder, and the PropertyVersion ids the chain-routed channels'
/// reader walks declare. The caller sorts and validates the tier.
void RigExecBakedBuildRestSteps(RigExecBakedProgramImpl *program);

/// Fills each rest and ladder op's `bindingLeaves` and `varyingLeaves`
/// from the ladder channels' leaves. Called after the last
/// RigExecBakedNumberLeaves of Build.
void RigExecBakedNoteRestLeaves(RigExecBakedProgramImpl *program);

/// The pose half of the prologue: the bound inputs, once per run.
void RigExecBakedBuildAvarSteps(RigExecBakedProgramImpl *program);
void RigExecBakedRunAvarOp(RigExecBakedProgramImpl *program, RigExecBakedStep *step);

/// Fills avarBindingBegin and avarConstantBindingBegin from the two binding
/// lists, which must be sorted by flat slot (verified). Build calls it once
/// the lists are complete; a program assembled by hand calls it after
/// filling them.
void RigExecBakedIndexAvarBindings(RigExecBakedProgramImpl *program);

/// Provider slot \p provider's bindings in the list \p begin indexes, as
/// [first, second) list positions: exactly the entries whose slot / 11 is
/// \p provider, in list order. Empty for a negative provider or one past
/// every binding.
inline std::pair<size_t, size_t>
RigExecBakedAvarBindingRange(const std::vector<uint32_t> &begin,
                             int64_t provider)
{
    if (provider < 0 || size_t(provider) + 1 >= begin.size()) {
        return {0, 0};
    }
    return {begin[size_t(provider)], begin[size_t(provider) + 1]};
}

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
/// and the adoption of each fixed skin revision's layout handle (topology,
/// topologyResolved and the partition's adoption) wherever a base reads.
/// Also samples the weight objects' and the revisions' path leaves
/// (RigExecBakedSamplePathLeaves; \p all re-reads every one).
void RigExecBakedRunGeometryPrologue(RigExecBakedProgramImpl *program,
                                     UsdTimeCode time, RigExecRigPose *pose,
                                     bool all);

/// SkinTopology op \p r's revision: entry r of `revisionIndex`, then of
/// `derivedIndex` past those; null past both.
RigExecBakedProgramImpl::GeomRevision *
RigExecBakedLayoutRevision(RigExecBakedProgramImpl *program, size_t r);
const RigExecBakedProgramImpl::GeomRevision *
RigExecBakedLayoutRevision(const RigExecBakedProgramImpl &program, size_t r);

/// Declares and binds the layout leaves of every `skinTopologyFixed`
/// revision and appends one SkinTopology head op per such revision
/// (`object` = its RigExecBakedLayoutRevision index), writing SkinTopology
/// slot `object` and reading only its leaves. Build, owning thread.
void RigExecBakedBuildLayoutSteps(RigExecBakedProgramImpl *program);

/// Appends SkinTopology[r] to the `reads` of the RevisionStatic,
/// RevisionChunk and RevisionFuse steps of every revision a SkinTopology op
/// serves. Build, once the geometry steps exist.
void RigExecBakedDeclareLayoutReads(RigExecBakedProgramImpl *program);

/// Re-reads \p revision's layout leaves at \p time through the generation's
/// resolved inputs, and re-asks RigExecSkinLayoutWeightsAreFixed (the
/// topology half stands from Build), when one of these
/// says they can have moved since the last sample:
///  1. the first sample, \p all (a forced run), or a moved program stamp;
///  2. a leaf's `mustSample` byte: a value edit or a routed override reached
///     one of the three paths (`leafByPath`);
///  3. the overlay entry at one of the three paths differs from the one the
///     last sample saw (an override placed, moved or lifted there).
/// The time alone moves nothing: a fixed layout reads the same at every
/// time, as the evaluator's cache holds it across frames. A layout that is
/// not fixed is not read. Sets each leaf's `changed` byte and
/// `layoutFixedChanged`. Owning thread, prologue only.
void RigExecBakedSampleLayoutLeaves(
    RigExecBakedProgramImpl *program,
    RigExecBakedProgramImpl::GeomRevision *revision, UsdTimeCode time,
    bool all);

/// Whether \p revision's skin layout is fixed, as the next sample answers
/// it: false while its topology half (`layoutTopologyFixed`, Build) is;
/// else GeomRevision::layoutFixed while that answer is current (its weights
/// leaf's VarianceStale false), else RigExecSkinLayoutWeightsAreFixed asked
/// again, without storing it. Owning thread.
bool RigExecBakedLayoutFixedNow(
    const RigExecBakedProgramImpl &program,
    const RigExecBakedProgramImpl::GeomRevision &revision);

/// The SkinTopology op's body over \p revision's leaves: a null handle while
/// the layout is not fixed, else RigExecBuildSkinTopology of the leaves,
/// handed back as `layoutCandidate` when that holds the same layout. Writes
/// `layoutHandle` and `layoutCandidate` and nothing else. Pure.
void RigExecBakedRunLayoutOp(const RigExecBakedProgramImpl &,
                           RigExecBakedProgramImpl::GeomRevision *revision);


/// The pose half of the epilogue: the joint and control publication, the
/// solver guides and the property-domain results.
///
/// Returns false where today's walk returns false from the same line: a
/// joint with a valid, non-degenerate final frame whose rest or frame is
/// not usable needs exec.
bool RigExecBakedPublishPose(RigExecBakedProgramImpl *program,
                             RigExecRigPose *pose);

/// Publishes each geometry chain and derived target in chain order.
void RigExecBakedPublishGeometry(RigExecBakedProgramImpl *program,
                                 RigExecRigPose *pose);

/// Construct the token tables the step bodies of each translation unit use,
/// on the calling thread. Build calls them, with RigExecRevisionKernelTouchTokens
/// and RigExecWeightPacketsTouchTokens, so a table's lazy construction --
/// which builds tokens from text -- never runs first on a worker.
void RigExecBakedGeometryTouchTokens();
void RigExecBakedWeightsTouchTokens();
void RigExecFrozenGeometryTouchTokens();

/// RIGEXEC_BAKED_CHUNK_VERTS and RIGEXEC_BAKED_MAX_CHUNKS, read by Build
/// into chunkVertexTarget and chunkCap.
size_t RigExecBakedChunkVertexTargetFromEnvironment();
size_t RigExecBakedChunkCapFromEnvironment();

/// Cuts \p revision's vertices into chunks, from \p indices and
/// \p elementSize, and records both as the partition's arrays (the indices
/// only when the cut has more than one chunk). Build only: each chunk step
/// declares its reads from its key.
///
/// Contiguous ranges of \p program's chunkVertexTarget vertices, capped at
/// its chunkCap (the range grows to meet the cap); each range's
/// key is the union of its vertices' influence positions; adjacent ranges
/// merge while they stay under the vertex cap and one key contains the
/// other, because a range that waits for a superset of another's joints is
/// not waiting any longer for holding both. Vertex order is never permuted.
void RigExecBakedPartitionRevision(
    const RigExecBakedProgramImpl &program,
    RigExecBakedProgramImpl::GeomRevision *revision,
    const VtIntArray &indices, int elementSize);

/// Makes \p revision's adopted `topology` the handle its chunk keys
/// describe when it carries the partition's arrays (the same `jointIndices`
/// and element size; the weights may differ). Otherwise the partition is
/// left as it is, and `partitionStale` runs the revision whole until a
/// handle with those arrays is adopted again or the program is rebuilt.
/// Nothing for an unchunked revision or a null handle. Called where the
/// handle is adopted: the live geometry prologue and the frozen worker.
void RigExecBakedAdoptRevisionLayout(RigExecBakedProgramImpl::GeomRevision *);
void RigExecBakedRunChainInputs(RigExecBakedProgramImpl *,RigExecBakedStep *);
void RigExecBakedPrepareDerivedBase(RigExecBakedProgramImpl::GeomChain::Derived *,RigExecBakedStep *);
void RigExecBakedAdoptPartition(
    RigExecBakedProgramImpl::GeomRevision *revision);

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
                      RigExecBakedDependencySink *sink)
{
    RigExecBakedStep *step = sink->step;
    step->varyingInputs = step->varyingInputs || input.varying;
    sink->resolvedReads = sink->resolvedReads || bool(input.resolvedAttr);
    if (input.overrideIndex >= 0) {
        step->overrideInputs.push_back(input.overrideIndex);
    }
    if (input.walk >= 0) {
        step->readerWalks.push_back(input.walk);
        if(input.leaf>=0)step->walkedBindingLeaves.emplace_back(RigExecBakedLeafTraits<T>::type,uint32_t(input.leaf));
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
/// composes. Every stage read is a leaf the prologue sampled at \p time
/// (`pointLeaves` and the bound inputs').
RigExecWeightPacket RigExecBakedWeightPacket(
    const RigExecBakedProgramImpl &program,
    RigExecBakedProgramImpl::WeightObject *object,
    const std::vector<RigExecWeightPacket> &packets, UsdTimeCode time);

/// The numeric time Build probes input selections at: the stage's start
/// time code when it authors a time-code range, else 0. Never Default.
UsdTimeCode RigExecBakedProbeTime(const UsdStageRefPtr &stage);

/// What a port of RigExecRigEvaluator::_ResolveWeights needs beyond a
/// weight object's composition and its scalar reads: the answers the oracle
/// takes from the stage alone. Read at one time.
/// Fills \p facts for the weight object at \p path, reading the stage as
/// the oracle does at \p time.
inline void
RigExecBakedDescribeWeightOracle(const RigExecRigEvaluator &evaluator,
                                 const SdfPath &path, UsdTimeCode time,
                                 RigExecWeightOracleFacts *facts)
{
    RigExecBakedProgram::DescribeWeightOracle(evaluator, path, time, facts);
}

/// A weight object the oracle reaches that no WeightPacket step bakes: a
/// constraint's or a property mover's envelope, or an object one composes
/// (the constraint step's envelope arm, bakedPose.cpp). Composed as
/// RigExecBakedBakeWeightObject composes, and registered nowhere: the
/// oracle reads its scalars through the
/// generation's resolved inputs (_ResolvedRead), so each read is kept as
/// the attribute that walk starts from plus the oracle's fallback.
struct RigExecBakedEnvelopeObject {
    struct Read {
        UsdAttribute head;  ///< invalid when the prim has no such attribute
        float fallback = 0.0f;
    };
    SdfPath path;
    TfToken type, representation, rangePolicy, combineMode;
    std::vector<float> values;
    std::vector<int> indices;
    /// rigExec:baseWeight's first target and rigExec:inputWeights in
    /// authored order, as indices into the oracle's table: below
    /// program.weightObjects.size() a step-backed object, else
    /// weightObjects.size() plus an index into the envelope list.
    int base = -1;
    std::vector<int> inputs;
    Read defaultWeight, driver, scale, bias, strength, invert;
};

/// Composes every weight object a constraint envelope (a weight object and
/// no points target) or a property mover's rigExec:weightObject[0] reaches
/// through rigExec:baseWeight and rigExec:inputWeights that \p program
/// does not already bake, depth first, so the list is in dependency order.
/// \p index maps each composed path to its oracle-table index. False with
/// the reason on what the oracle could not resolve structurally -- a
/// missing prim, a type that is no weight object or is volumetric, an
/// unknown token, a second base, a cycle -- which Compile rejects first.
inline bool
RigExecBakedComposeEnvelopeObjects(
    const RigExecRigEvaluator &evaluator,
    const RigExecBakedProgramImpl &program,
    std::vector<RigExecBakedEnvelopeObject> *objects,
    std::map<SdfPath, int> *index, std::string *error)
{
    return RigExecBakedProgram::ComposeEnvelopeObjects(
        evaluator, program, objects, index, error);
}

/// One of the evaluator's property chains as a bake states it: the
/// target, which runChain arm its type selects, and per revision what
/// _EvaluatePropertyChains binds and reads on the mover
/// (rigEvaluatorProperties.cpp). The reads stay attributes; the bake turns
/// each into a walk.
struct RigExecBakedPropertyChainDesc {
    /// The runChain arm: the target's type name is float, double or
    /// matrix4d, else the GfVec3f-backed arm.
    enum class ValueType : uint8_t { Float = 0, Double, Matrix4d, Vec3f };
    struct Revision {
        SdfPath mover;
        /// rigExec:operation as _ReadOperation reads it (the attribute's
        /// own value at Default), parsed; false when it does not parse.
        bool opValid = false;
        RigExecPropertyOp op = RigExecPropertyOp::Add;
        /// The inputs _BindInput binds; invalid when the mover has none.
        UsdAttribute enabled, defaultWeight, value, minimum, maximum;
        /// rigExec:weightObject's first target; empty when it has none.
        SdfPath weightObject;
        int weightField = -1;
        /// inputs:keys and inputs:tangents at Default, read only for a
        /// float or double chain whose operation is curve (the only arm
        /// that reads them).
        std::vector<GfVec2f> keys;
        bool hasTangentsAttr = false;
        std::vector<GfVec2f> tangents;
    };
    /// A RigExecPhasedConnection on this chain.
    struct Phased {
        SdfPath consumer;
        /// Float or Double when the consumer has that type name, else the
        /// chain's own type: RigExecPhasedConsumerValue converts between
        /// float and double and passes every other value through.
        ValueType consumerType = ValueType::Float;
        size_t applied = 0;
        /// RigExecPhasedConnection::hops: the consumer, then each attribute
        /// its walk passes before the target.
        SdfPathVector hops;
    };
    SdfPath target;
    UsdAttribute targetAttr;
    ValueType valueType = ValueType::Float;
    std::vector<Revision> revisions;
    std::vector<Phased> phased;
};

/// Restates every property chain of \p evaluator's standing compile, in
/// _propertyChainOrder, with its phased connections in
/// _phasedConnections order. False with the reason when a chain cannot be
/// stated for a file read at one time: a curve revision whose keys or
/// tangents are animated or connected, or (refused by Compile first) a
/// missing target or mover.
inline bool
RigExecBakedDescribePropertyChains(
    const RigExecRigEvaluator &evaluator,
    std::vector<RigExecBakedPropertyChainDesc> *chains, std::string *error)
{
    return RigExecBakedProgram::DescribePropertyChains(evaluator, chains,
                                                       error);
}

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
/// whole program is re-run, and Compare checks retained values and diagnostics.
/// Work observations are projected through the cone's actual body selection. Nothing here is on a production
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
///    input bindings, the step graph, the clustering, the cones.
///    A run that changed one of those would be a run editing the program.
///  * what only a NOTICE writes and no run reads: `valueEditSerial`/
///    `editSerial` and the routing's `connectedSources` cache.
///  * what the PROLOGUE writes: `avarsDisturbed`,
///    `overridden`/`anyOverridden`/`folded`, the sampled `leaves`,
///    `routedOverrides`/`lastRoutedOverrides` and `leafSamples`, and the
///    geometry prologue's sampler pools and routing stamps. Persistent
///    property versions/validity/records, property publication overlays,
///    rest and ladder tables, layout handles and shared-step memo lines
///    are captured below.
///    The prologue runs ONCE per
///    generation, before either pass, so both passes see one value of each
///    by construction. Several are captured and restored anyway, because
///    they are cheap and putting the second pass back on exactly the first
///    pass's footing is what the mode is for.
///  * the cone bookkeeping itself -- `closedSteps`, `closed`, `lastAvars`,
///    `lastOverridden`, `edited`/`anyEdited` (the first pass consumes them
///    and the forced second pass needs none),
///    `lastHaveBase`, `lastTime`, `everRan`,
///    `lastProgramStamp`, and the provider leaves' `spaceLeafRekey`. The
///    second pass is FORCED, so its closure differs
///    from the first's on purpose; comparing them would report the mode
///    rather than the program. The run statistics that observers read are
///    put back by RigExecBakedRunStatistics instead.
///  * each chain's `spare`, the other half of the published double buffer.
///    A pass that publishes swaps it with `result`; a pass that skips the
///    publication does not. The two therefore hold DIFFERENT generations'
///    arrays after runs that agree exactly about the published one, which is
///    `result` -- the buffer is storage, and only what `result` names in it
///    is an answer.
///  * each revision's `revisionInputs` overlay, which its RevisionStatic
///    writes and reads within the one body, so no other step can see a
///    stale one. It is captured and restored -- it costs nothing and it
///    keeps the second pass starting from exactly the first's state -- but
///    not compared.
///  * the RUN STATISTICS, which are restored rather than compared, because
///    the second pass is forced and so writes different ones by
///    construction: `lastClosedClusters`, `lastClosedSteps`,
///    `spaceLeafKeys`, the
///    clustering's `lastRunTimed`
///    the closed-operation sets and full-run flag,
///    and each `RigExecBakedCluster`'s `readyUs`/`startUs`/`endUs`, and each
///    step's `startUs`/`endUs` and memo/publication stamps.
///    RigExecBakedRunStatistics takes all of them before the second pass and
///    puts them back after it, so that every
///    observer of the frame -- the run report, the profiler trace,
///    GetClustersRunLastGeneration() -- describes the one run that published
///    a pose.
///  * per-step `measured*` accumulators: the calibrator's running averages,
///    a fit over frames, which an opt-in calibration reads and this mode
///    does not.
struct RigExecBakedLadderTables {
    std::vector<GfMatrix4d> restM;
    std::vector<std::array<GfVec3d, 4>> restPts;
    std::vector<RigExecPointFrame> restFrames;
    std::vector<GfMatrix4d> selfD, parentDinv, posedD, parentSpaceM;
    std::vector<char> parentSpaceAuthored;
    std::vector<unsigned char> rotationSign;
    std::vector<TfToken> rotOrder;
    std::vector<GfMatrix4d> restRoundTrip, defaultRoundTrip;
    std::vector<char> posedAuthored;
    std::vector<GfMatrix4d> posedAuthoredM;
};

struct RigExecBakedRunShadow {
    /// Copies everything a step reads or writes out of \p program.
    void Capture(const RigExecBakedProgramImpl &program);
    /// Puts it back, so that a second run starts where the first one did.
    void Restore(RigExecBakedProgramImpl *program) const;
    /// Appends one line per disagreement between this shadow and
    /// \p program's current state, and returns how many there were.
    size_t Compare(const RigExecBakedProgramImpl &program,
                   std::vector<std::string> *differences) const;


    std::vector<RigExecBakedPropertyValue> values;
    std::vector<char> versionValid, changed, chainValid, recordStoodAside;
    std::vector<VtValue> chainFinal, recordValues;
    std::map<SdfPath,VtValue> propertyResults;
    RigExecResolvedInputs resolvedInputs;
    bool hasResolvedInputs = false;
    std::vector<VtValue> headOverrides, lastHeadOverrides;
    std::vector<char> headOverrideMoved, readerWalkMoved, readerWalkChanged;


    RigExecBakedLadderTables tables;
    std::vector<GfMatrix4d> lastRestM, lastSelfD, lastParentDinv,
        lastPosedAuthoredM;
    std::vector<char> lastPosedAuthored;
    std::vector<GfMatrix4d> lastPosedD, lastParentSpaceM;
    std::vector<char> lastParentSpaceAuthored;
    std::vector<unsigned char> lastRotationSign;
    std::vector<TfToken> lastRotOrder;
    std::vector<char> restChanged, ladderChanged;
    std::vector<int> restMoved, ladderMoved;
    uint64_t opsRun = 0;

    struct StepState {
        std::vector<std::string> diagnostics, lines;
            RigExecBakedStepCounters counters;
    };
    struct ChunkState {
        std::vector<GfMatrix4d> transforms;
        std::vector<float> rows;
        std::vector<RigExecScaledDualQuat> palette;
        bool keyChanged = false, ok = false;
    };
    struct BlendSampleState {
        std::shared_ptr<const RigExecBlendSampleLayout> layout;
        std::vector<GfVec3f> lastPoints;
        bool layoutRefused = false;
        bool layoutKeyed = false;
        uint64_t layoutOffsetsVersion = 0;
        uint64_t layoutIndicesVersion = 0;
        size_t layoutPointCount = 0;
        uint64_t layoutBuilds = 0;
    };
    struct RevisionState {
        std::vector<std::vector<BlendSampleState>> blendSamples;
        std::shared_ptr<const RigExecSkinTopology> topology, partitionTopology;
        bool topologyResolved = false;
        std::shared_ptr<const RigExecSkinTopology> layoutHandle, layoutCandidate;
        bool layoutFixed = false, layoutRan = false;
        std::vector<GfVec3f> output;
        std::vector<GfVec3f> stagingOutput, passedPoints;
        bool stagingFresh = false;
        uint64_t doneVersion = 0;
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
        bool haveTransform = false, ran = false, executed = false, created = false;
        bool influencesValid = false, influencesChanged = false;
        bool staticDirty = false, partitionStale = false;
        bool layoutUsable = false, envelopeOk = false, fullStrength = false;
        RigExecRevisionAcceptance acceptance = RigExecRevisionAcceptance::Refuses;
        std::vector<float> publishedWeightValues;
        RigExecWeightPacket currentPhasePacket;
        bool weightFieldPublished = false;
    };
    struct DerivedState {
        RevisionState revision;
        VtVec3fArray result, spare, lastBase;
        uint64_t resultVersion = 0;
        bool haveResult = false, haveBase = false, baseDirty = false;
        GfMatrix4d matrix{1.0};
        bool haveMatrix = false;
    };
    struct ChainState {
        std::vector<RevisionState> revisions;
        std::vector<DerivedState> derived;
        VtVec3fArray lastBase, result, spare;
        uint64_t baseVersion = 0, resultVersion = 0;
        bool haveResult = false, haveBase = false, baseDirty = false, scheduleDirty = false;
    };
    struct SolverState {
        std::vector<RigExecPointFrame> outFrames;
        std::vector<char> outPresent;
        std::vector<int> fallbackSlots;
    };
    struct CommitState {
        std::vector<char> present, deltaOk;
        std::vector<RigExecPointFrame> frames, staged;
        std::vector<GfMatrix4d> deltas;
        std::vector<uint8_t> outcome;
        std::vector<RigExecConstraintSource> sources;
        bool abandoned = true;
        bool recordAfter = true, recordEveryTarget = true;
    };

    std::optional<RigExecOraclePublicationContext> oraclePublications;
    std::map<SdfPath,RigExecWeightReferenceContext> oracleWeightInputs;
    RigExecTypedValueStore providerValues{0};
    std::vector<GfMatrix4d> switchFrames;
    RigExecOpAdapterState opAdapter;
    std::vector<std::string> chainContentKeys;
    RigExecOpExecution opExecution;
    std::vector<double> avars;
    std::vector<float> poseWeights;
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
    std::vector<RigExecBakedProgramImpl::WeightField> weightFields;
    std::vector<GfMatrix4d> volumePlacementBase;
    std::vector<GfMatrix4d> volumePlacement;
    std::vector<GfMatrix4d> frameMatrix;
    std::vector<char> frameMatrixValid;
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
        std::thread::id runner;
    };
    struct StepTimes {
        uint64_t startUs = 0, endUs = 0;
        /// The op stamps the trace reads. memoEndNs and bodyEndNs need no
        /// copy: only a measurement writes them, and the second pass runs
        /// with measurementSuspended, so it leaves the first pass's.
        uint64_t memoStartNs = 0, publishEndNs = 0;
        std::thread::id runner;
    };
    RigExecOpExecution opExecution;
    std::vector<ClusterTimes> clusters;
    std::vector<StepTimes> steps;
    RigExecBakedClusterSet selectedClusters, selectedSteps;
    bool closureFull = false;
    size_t closedClusters = 0;
    size_t closedSteps = 0;
    size_t spaceLeafKeys = 0;
    bool timed = false;
};

/// Whether RIGEXEC_BAKED_VERIFY_CONES asks a run to prove its cone.
bool RigExecBakedVerifyConesRequested();

/// A projector target's provider world frames, out of the program's rest
/// frames and its base and final matrix tables.
RigExecSurfaceProjectorFrames RigExecBakedProjectorFrames(
    const RigExecBakedProgramImpl &B,
    const RigExecBakedProgramImpl::GeomRevision &revision);

/// One projector target of \p chain, run against its authored base and
/// final points over the reads its path leaves hold
/// (RigExecReadProjectorTargetFromLeaves): what the live and frozen Derived
/// steps do for a matrix target. Reads no stage.
bool RigExecBakedRunProjectorTarget(
    const RigExecBakedProgramImpl &B,
    const RigExecBakedProgramImpl::GeomChain &chain,
    const RigExecBakedProgramImpl::GeomRevision &revision,
    GfMatrix4d *matrix, std::vector<std::string> *diagnostics);

/// Test-only access to evaluator state a baked run must not touch.
struct RigExecBakedProgramTesting {
    /// Enables live per-operation exec witnesses. Build and Evaluate are
    /// owning-thread operations; native bodies only copy their bound values.
    static bool EnableExecCrossCheck(const RigExecBakedProgram &program,
                                    std::string *error = nullptr);
    static std::shared_ptr<RigExecBakedExecCheckRows> ExecCrossCheckRows(
        const RigExecBakedProgram &program);
    static void SetOpObservers(const RigExecBakedProgram &program,
        std::function<void(uint32_t)> before,std::function<void(uint32_t)> after);
    /// Clears \p program's point captures and has every later run record,
    /// per point binding, the binding's answer as its reader took it.
    /// Returns false and captures nothing under the parallel schedule: the
    /// capture is a test aid and stays off the parallel executor.
    static bool CapturePointReads(const RigExecBakedProgram &program);
    /// The condition behind IsBakeable's and Build's solver-checkpoint
    /// guard: the solvers in \p snapshots (joint -> read-phase movers) that
    /// no batched solver commits for that joint. No compiled rig reaches it,
    /// so it is tested on hand-built maps.
    static std::vector<SdfPath> SolverCheckpointsWithoutAnOutput(
        const std::map<SdfPath, std::set<SdfPath>> &snapshots,
        const std::map<SdfPath, std::set<SdfPath>> &solverDependencies,
        const std::map<SdfPath, std::vector<std::pair<SdfPath, int>>>
            &solverJoints,
        const std::set<SdfPath> &batched);
    /// A copy of \p program's solver \p index with the Solve step's rest
    /// refresh (RefreshSolverRests, bakedPose.cpp) applied to it, from the
    /// rests and `fin` the program holds now. The program is not modified.
    /// \p index must be below `solvers.size()`.
    static RigExecBakedProgramImpl::Solver RefreshedSolverRests(
        const RigExecBakedProgram &program, size_t index);
    /// The ten rest and ladder tables the bake exports.
    using LadderTables = RigExecBakedLadderTables;
    /// \p program's ten tables as they stand.
    static LadderTables LadderTablesOf(const RigExecBakedProgram &program);
    /// Assembles every revision of \p program whose packet its path leaves
    /// assemble (chain revisions whose chain read a base this run, and
    /// non-matrix derived targets) twice, from the leaves the last run
    /// sampled and from the stage through \p resolved at \p time, as the
    /// stage assembly did before the leaves, and returns one line per
    /// revision whose packets, statuses or lines differ or whose leaf
    /// assembly read a role its declaration lacks (named). A projector's
    /// matrix target compares its reads the same way
    /// (RigExecReadProjectorTargetFromLeaves against
    /// RigExecReadProjectorTarget). \p edit, when set, changes a copy of
    /// each declaration first (by mover path). Leaves the program as it
    /// found it. \p compared counts the revisions.
    static std::vector<std::string> ShadowAssembly(
        const RigExecBakedProgram &program,
        const RigExecResolvedInputs &resolved, UsdTimeCode time,
        const std::function<void(const SdfPath &, RigExecRevisionLeafDecl *)>
            &edit,
        size_t *compared);
};

VtValue RigExecBakedResolvePathLeaf(const RigExecBakedProgramImpl &,const RigExecBakedPathLeaves &,size_t);
/// Exact captured current API4 input heads and connection hops. No stage access.
bool RigExecBakedHasExternalInputRoute(const RigExecBakedProgramImpl &,const SdfPath &);
const std::vector<VtValue> &RigExecBakedResolvePathLeaves(const RigExecBakedProgramImpl &, const RigExecBakedPathLeaves &);
void RigExecBakedResetWeightField(RigExecBakedProgramImpl *, int);
void RigExecBakedResetSetAsideGeometryValue(RigExecBakedProgramImpl *, RigExecBakedSlotDomain, uint32_t);

}  // namespace rigExec

#endif  // RIGEXEC_BAKED_PROGRAM_IMPL_H
