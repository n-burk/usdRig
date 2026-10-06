// Property revisions as head-tier ops: the binding at Build,
// the head leaves, the walk resolver, the op bodies and the publication.
// Each body is the revision-loop body of
// RigExecRigEvaluator::_EvaluatePropertyChains (rigEvaluatorProperties.cpp)
// it replaces, with every stage read and overlay walk restated over values
// the prologue sampled. The dynamic path keeps its own function as the
// reference.
#include "bakedProgram.h"
#include "bakedProgramImpl.h"
#include "bodyPurity.h"
#include "frozenContextInternal.h"
#include "movers/moverRegistry.h"
#include "rigEvaluator.h"
#include "rigEvaluatorPropertyBindings.h"

#include "rigExecMath/propertyMath.h"

#include "pxr/base/gf/vec2f.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/tf/type.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/relationship.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <set>
#include <unordered_map>
#include <utility>

namespace rigExec {

namespace {

// Interned at load; read at Build on the owning thread.
const TfToken kEnabled("inputs:enabled");
const TfToken kDefaultWeight("inputs:defaultWeight");
const TfToken kOperation("rigExec:operation");
const TfToken kValue("inputs:value");
const TfToken kMin("inputs:min");
const TfToken kMax("inputs:max");
const TfToken kKeys("inputs:keys");
const TfToken kTangents("inputs:tangents");
const TfToken kWeightObject("rigExec:weightObject");

using HeadType = RigExecBakedHeadValueType;
using Chain = RigExecBakedPropertyChain;

template <class T>
bool
BitSame(const T &a, const T &b)
{
    return std::memcmp(&a, &b, sizeof(T)) == 0;
}

bool
IsFinite(float v)
{
    return std::isfinite(v);
}

// _EvaluatePropertyChains has no double overload: a double chain's base and
// result reach its float one, so a double past float's range is not finite.
bool
IsFinite(double v)
{
    return std::isfinite(static_cast<float>(v));
}

bool
IsFinite(const GfVec3f &v)
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

bool
IsFinite(const GfMatrix4d &m)
{
    for (size_t r = 0; r < 4; ++r) {
        for (size_t c = 0; c < 4; ++c) {
            if (!std::isfinite(m[r][c])) {
                return false;
            }
        }
    }
    return true;
}

// The TfType a head value type reads as.
TfType
TypeOf(HeadType type)
{
    switch (type) {
    case HeadType::Bool: return TfType::Find<bool>();
    case HeadType::Float: return TfType::Find<float>();
    case HeadType::Double: return TfType::Find<double>();
    case HeadType::Vec3f: return TfType::Find<GfVec3f>();
    case HeadType::Matrix4d: return TfType::Find<GfMatrix4d>();
    case HeadType::Vec2fArray: return TfType::Find<VtArray<GfVec2f>>();
    case HeadType::Int: return TfType::Find<int>();
    case HeadType::Token: return TfType::Find<TfToken>();
    case HeadType::Vec3d: return TfType::Find<GfVec3d>();
    }
    return TfType();
}

// The attribute's own value at \p time as \p type: the typed Get the read
// it restates makes, empty when that Get answers nothing.
template <class T>
VtValue
TypedGet(const UsdAttribute &attribute, UsdTimeCode time)
{
    T value;
    if (attribute.Get(&value, time)) {
        return VtValue(std::move(value));
    }
    return VtValue();
}

VtValue
SampleHead(const RigExecBakedHeadLeaf &leaf, UsdTimeCode time)
{
    if (!leaf.attribute || !leaf.typeMatches) {
        return VtValue();
    }
    switch (leaf.type) {
    case HeadType::Bool: return TypedGet<bool>(leaf.attribute, time);
    case HeadType::Float: return TypedGet<float>(leaf.attribute, time);
    case HeadType::Double: return TypedGet<double>(leaf.attribute, time);
    case HeadType::Vec3f: return TypedGet<GfVec3f>(leaf.attribute, time);
    case HeadType::Matrix4d:
        return TypedGet<GfMatrix4d>(leaf.attribute, time);
    case HeadType::Vec2fArray:
        return TypedGet<VtArray<GfVec2f>>(leaf.attribute, time);
    case HeadType::Int: return TypedGet<int>(leaf.attribute, time);
    case HeadType::Token: return TypedGet<TfToken>(leaf.attribute, time);
    case HeadType::Vec3d: return TypedGet<GfVec3d>(leaf.attribute, time);
    }
    return VtValue();
}

bool
Varies(const UsdAttribute &attribute)
{
    // The predicate RigExecBakedClassifyInput applies: neither half
    // implies the other (a single time sample; a Ts spline).
    return attribute && (attribute.ValueMightBeTimeVarying() ||
                         attribute.GetNumTimeSamples() > 0);
}

HeadType
ArmType(Chain::Arm arm)
{
    switch (arm) {
    case Chain::Arm::Float: return HeadType::Float;
    case Chain::Arm::Double: return HeadType::Double;
    case Chain::Arm::Matrix4d: return HeadType::Matrix4d;
    case Chain::Arm::Vec3f: return HeadType::Vec3f;
    }
    return HeadType::Float;
}

// What Build binds a program's chains with.
struct BindState {
    RigExecBakedProgramImpl *program = nullptr;
    std::map<std::pair<SdfPath, int>, uint32_t> leafOf;
    std::unordered_map<SdfPath, int, SdfPath::Hash> chainOfTarget;
    std::unordered_map<SdfPath, int, SdfPath::Hash> recordOfConsumer;

    int Slot(const SdfPath &path)
    {
        RigExecBakedProgramImpl &B = *program;
        const auto [it, inserted] = B.headOverrideSlots.emplace(
            path, uint32_t(B.headOverrideSlots.size()));
        return int(it->second);
    }

    // One leaf per (attribute, type), shared by every walk that reads it.
    int Leaf(const UsdAttribute &attribute, HeadType type)
    {
        RigExecBakedProgramImpl &B = *program;
        const auto key = std::make_pair(attribute.GetPath(), int(type));
        const auto found = leafOf.find(key);
        if (found != leafOf.end()) {
            return int(found->second);
        }
        RigExecBakedHeadLeaf leaf;
        leaf.path = attribute.GetPath();
        leaf.attribute = attribute;
        leaf.type = type;
        leaf.typeMatches =
            attribute && attribute.GetTypeName().GetType() == TypeOf(type);
        leaf.varying = Varies(attribute);
        const uint32_t index = uint32_t(B.headLeaves.size());
        B.headLeaves.push_back(std::move(leaf));
        leafOf.emplace(key, index);
        return int(index);
    }

    // A hop read by chain \p reader: a chain target or a record consumer
    // only counts when its chain runs first, which is the only way the
    // overlay can hold its value when the reader runs.
    RigExecBakedWalkHop Hop(const UsdAttribute &attribute, int reader)
    {
        RigExecBakedWalkHop hop;
        hop.path = attribute.GetPath();
        hop.overrideSlot = Slot(hop.path);
        const auto chain = chainOfTarget.find(hop.path);
        if (chain != chainOfTarget.end() && chain->second < reader) {
            hop.chain = chain->second;
        }
        const auto record = recordOfConsumer.find(hop.path);
        if (record != recordOfConsumer.end() &&
            int(program->propertyRecords[size_t(record->second)].chain) <
                reader) {
            hop.record = record->second;
        }
        return hop;
    }

    // GetAttribute<double>'s walk from \p from, with its own cycle guard.
    void DoubleTail(const UsdAttribute &from, int reader,
                    std::vector<RigExecBakedWalkHop> *out)
    {
        std::set<SdfPath> visiting;
        UsdAttribute a = from;
        while (a && visiting.insert(a.GetPath()).second) {
            RigExecBakedWalkHop hop = Hop(a, reader);
            hop.leaf = Leaf(a, HeadType::Double);
            out->push_back(std::move(hop));
            SdfPathVector connections;
            if (a.HasAuthoredConnections()) {
                a.GetConnections(&connections);
            }
            if (connections.size() != 1) {
                break;
            }
            a = a.GetPrim().GetStage()->GetAttributeAtPath(connections[0]);
        }
    }

    // _PinnedRead's arms as a walk: no attribute, an unconnected one (the
    // overlay at its own path, else its own value), or a connected one
    // (RigExecResolvedInputs::GetAttribute, hop for hop). \p pinned false
    // takes GetAttribute's walk for an unconnected attribute too, whose
    // float read of a double is the double read, cast.
    RigExecBakedWalk Walk(const UsdAttribute &attribute, HeadType type,
                          int reader, bool pinned = true)
    {
        RigExecBakedWalk walk;
        walk.type = type;
        if (!attribute) {
            return walk;
        }
        if (pinned && !attribute.HasAuthoredConnections()) {
            walk.flavour = RigExecBakedWalk::Flavour::Pinned;
            RigExecBakedWalkHop hop = Hop(attribute, reader);
            // A record's consumer is connected by definition.
            hop.record = -1;
            hop.leaf = Leaf(attribute, type);
            walk.hops.push_back(std::move(hop));
            return walk;
        }
        walk.flavour = RigExecBakedWalk::Flavour::Connected;
        const bool isFloat = type == HeadType::Float;
        // A float read of a double attribute is GetAttribute<double> from
        // it, cast: no float overlay is consulted at its head.
        if (isFloat &&
            attribute.GetTypeName() == SdfValueTypeNames->Double) {
            DoubleTail(attribute, reader, &walk.doubleHops);
            return walk;
        }
        std::set<SdfPath> visiting;
        std::vector<UsdAttribute> attributes;
        UsdAttribute a = attribute;
        while (a && visiting.insert(a.GetPath()).second) {
            walk.hops.push_back(Hop(a, reader));
            attributes.push_back(a);
            if (isFloat && a.GetTypeName() == SdfValueTypeNames->Double) {
                DoubleTail(a, reader, &walk.doubleHops);
                // The fallback over these hops is never reached.
                return walk;
            }
            SdfPathVector connections;
            if (a.HasAuthoredConnections()) {
                a.GetConnections(&connections);
            }
            if (connections.size() != 1) {
                break;
            }
            a = a.GetPrim().GetStage()->GetAttributeAtPath(connections[0]);
        }
        for (size_t i = 0; i < walk.hops.size(); ++i) {
            walk.hops[i].leaf = Leaf(attributes[i], type);
        }
        return walk;
    }

    // A read after the head tier: GetAttribute's walk from \p attribute,
    // on which every chain and record counts. -1 when no hop is a chain
    // target or a record consumer, so the published overlay can hold
    // nothing there but overrides.
    int ReaderWalk(const UsdAttribute &attribute, HeadType type,
                   bool rawFallback)
    {
        if (!attribute) {
            return -1;
        }
        RigExecBakedProgramImpl &B = *program;
        // Walk() files leaves and slots as it goes; a walk that meets
        // nothing takes them back, so only walks a chain can answer add
        // head leaves (and frozen samples).
        const size_t leafCount = B.headLeaves.size();
        const size_t slotCount = B.headOverrideSlots.size();
        RigExecBakedReaderWalk reader;
        reader.walk = Walk(attribute, type,
                           std::numeric_limits<int>::max(),
                           /*pinned=*/false);
        bool meets = false;
        std::set<uint32_t> leaves, slots;
        for (const std::vector<RigExecBakedWalkHop> *hops :
             {&reader.walk.hops, &reader.walk.doubleHops}) {
            for (const RigExecBakedWalkHop &hop : *hops) {
                meets = meets || hop.chain >= 0 || hop.record >= 0;
                if (hop.leaf >= 0) {
                    leaves.insert(uint32_t(hop.leaf));
                }
                if (hop.overrideSlot >= 0) {
                    slots.insert(uint32_t(hop.overrideSlot));
                }
            }
        }
        if (!meets) {
            B.headLeaves.resize(leafCount);
            for (auto it = leafOf.begin(); it != leafOf.end();) {
                it = it->second >= leafCount ? leafOf.erase(it) : ++it;
            }
            for (auto it = B.headOverrideSlots.begin();
                 it != B.headOverrideSlots.end();) {
                it = it->second >= slotCount ? B.headOverrideSlots.erase(it)
                                             : ++it;
            }
            return -1;
        }
        reader.head = attribute.GetPath();
        if (rawFallback) {
            reader.rawLeaf = Leaf(attribute, type);
            leaves.insert(uint32_t(reader.rawLeaf));
        }
        reader.leaves.assign(leaves.begin(), leaves.end());
        reader.slots.assign(slots.begin(), slots.end());
        B.readerWalks.push_back(std::move(reader));
        return int(B.readerWalks.size()) - 1;
    }
};

template <class T>
HeadType
HeadTypeOf();
template <>
HeadType
HeadTypeOf<double>()
{
    return HeadType::Double;
}
template <>
HeadType
HeadTypeOf<float>()
{
    return HeadType::Float;
}
template <>
HeadType
HeadTypeOf<int>()
{
    return HeadType::Int;
}
template <>
HeadType
HeadTypeOf<bool>()
{
    return HeadType::Bool;
}
template <>
HeadType
HeadTypeOf<TfToken>()
{
    return HeadType::Token;
}
template <>
HeadType
HeadTypeOf<GfMatrix4d>()
{
    return HeadType::Matrix4d;
}
template <>
HeadType
HeadTypeOf<GfVec3d>()
{
    return HeadType::Vec3d;
}
template <>
HeadType
HeadTypeOf<GfVec3f>()
{
    return HeadType::Vec3f;
}

// The head type a path-leaf key reads through the resolved inputs as, or
// false for a read no chain or record value can answer (arrays) or one at
// Default (head leaves hold the run's time).
bool
PathLeafHeadType(const RigExecRevisionLeafKey &key,
                 const UsdAttribute &attribute, HeadType *type)
{
    using Flavour = RigExecRevisionLeafFlavour;
    using Type = RigExecRevisionLeafType;
    if ((key.flavour != Flavour::Resolved &&
         key.flavour != Flavour::ResolvedOnly) ||
        key.time != RigExecRevisionLeafTime::AtTime) {
        return false;
    }
    switch (key.type) {
    case Type::Bool: *type = HeadType::Bool; return true;
    case Type::Int: *type = HeadType::Int; return true;
    case Type::Float: *type = HeadType::Float; return true;
    case Type::Token: *type = HeadType::Token; return true;
    case Type::Vec3d: *type = HeadType::Vec3d; return true;
    case Type::Matrix4d: *type = HeadType::Matrix4d; return true;
    case Type::Dial:
        // RigExecReadProjectorTarget reads a float dial as a float.
        *type = attribute &&
                        attribute.GetTypeName() == SdfValueTypeNames->Float
                    ? HeadType::Float
                    : HeadType::Double;
        return true;
    default:
        return false;
    }
}

// What stands in the generation's resolved inputs at \p hop while a head op
// runs: the overlay _EvaluatePropertyChains reads at that point, which holds
// the interactive overrides and every earlier chain's publication.
const VtValue *
Overlay(const RigExecBakedProgramImpl &B, const RigExecBakedWalkHop &hop)
{
    const VtValue *standing =
        hop.overrideSlot >= 0 &&
                !B.headOverrides[size_t(hop.overrideSlot)].IsEmpty()
            ? &B.headOverrides[size_t(hop.overrideSlot)]
            : nullptr;
    if (hop.chain >= 0) {
        // publishTarget replaces the override with the result.
        return B.chainValid[size_t(hop.chain)]
                   ? &B.chainFinal[size_t(hop.chain)]
                   : standing;
    }
    if (hop.record >= 0) {
        // An override on any hop of the record stands the reader aside.
        if (standing) {
            return standing;
        }
        const RigExecBakedPropertyRecord &record =
            B.propertyRecords[size_t(hop.record)];
        if (B.recordStoodAside[size_t(hop.record)] ||
            !B.chainValid[record.chain]) {
            return nullptr;
        }
        return &B.recordValues[size_t(hop.record)];
    }
    return standing;
}

const VtValue *
LeafValue(const RigExecBakedProgramImpl &B, const RigExecBakedWalkHop &hop)
{
    return hop.leaf >= 0 ? &B.headLeaves[size_t(hop.leaf)].value : nullptr;
}

// GetAttribute<double>'s recursion over a float walk's double tail, cast.
bool
ResolveDouble(const RigExecBakedProgramImpl &B, const RigExecBakedWalk &walk,
              float *out)
{
    for (const RigExecBakedWalkHop &hop : walk.doubleHops) {
        const VtValue *v = Overlay(B, hop);
        if (v && v->IsHolding<double>()) {
            *out = static_cast<float>(v->UncheckedGet<double>());
            return true;
        }
    }
    for (auto it = walk.doubleHops.rbegin(); it != walk.doubleHops.rend();
         ++it) {
        const VtValue *v = LeafValue(B, *it);
        if (v && v->IsHolding<double>()) {
            *out = static_cast<float>(v->UncheckedGet<double>());
            return true;
        }
    }
    return false;
}

}  // namespace

/// \p walk's value as the read it restates answers it, or false where that
/// read answers nothing (and its caller keeps its fallback). Pure: no stage,
/// no path work, no lock.
template <class T>
bool
RigExecBakedResolveWalk(const RigExecBakedProgramImpl &B,
                        const RigExecBakedWalk &walk, T *out)
{
    using Flavour = RigExecBakedWalk::Flavour;
    switch (walk.flavour) {
    case Flavour::Absent:
        return false;
    case Flavour::Pinned: {
        const RigExecBakedWalkHop &hop = walk.hops.front();
        const VtValue *v = Overlay(B, hop);
        if (v && v->IsHolding<T>()) {
            *out = v->UncheckedGet<T>();
            return true;
        }
        v = LeafValue(B, hop);
        if (v && v->IsHolding<T>()) {
            *out = v->UncheckedGet<T>();
            return true;
        }
        return false;
    }
    case Flavour::Connected:
        for (const RigExecBakedWalkHop &hop : walk.hops) {
            const VtValue *v = Overlay(B, hop);
            if (v && v->IsHolding<T>()) {
                *out = v->UncheckedGet<T>();
                return true;
            }
        }
        if (!walk.doubleHops.empty()) {
            if constexpr (std::is_same_v<T, float>) {
                return ResolveDouble(B, walk, out);
            } else {
                return false;
            }
        }
        // The nearest readable upstream value, deepest first.
        for (auto it = walk.hops.rbegin(); it != walk.hops.rend(); ++it) {
            const VtValue *v = LeafValue(B, *it);
            if (v && v->IsHolding<T>()) {
                *out = v->UncheckedGet<T>();
                return true;
            }
        }
        return false;
    }
    return false;
}

bool
RigExecBakedHeadValueSame(const VtValue &a, const VtValue &b)
{
    if (a.IsEmpty() || b.IsEmpty()) {
        return a.IsEmpty() && b.IsEmpty();
    }
    // By typeid: TfType lookups take the type registry's lock.
    if (a.GetTypeid() != b.GetTypeid()) {
        return false;
    }
    if (a.IsHolding<float>()) {
        return BitSame(a.UncheckedGet<float>(), b.UncheckedGet<float>());
    }
    if (a.IsHolding<double>()) {
        return BitSame(a.UncheckedGet<double>(), b.UncheckedGet<double>());
    }
    if (a.IsHolding<bool>()) {
        return a.UncheckedGet<bool>() == b.UncheckedGet<bool>();
    }
    if (a.IsHolding<GfVec3f>()) {
        return BitSame(a.UncheckedGet<GfVec3f>(), b.UncheckedGet<GfVec3f>());
    }
    if (a.IsHolding<GfVec3d>()) {
        return BitSame(a.UncheckedGet<GfVec3d>(), b.UncheckedGet<GfVec3d>());
    }
    if (a.IsHolding<int>()) {
        return a.UncheckedGet<int>() == b.UncheckedGet<int>();
    }
    if (a.IsHolding<GfMatrix4d>()) {
        return BitSame(a.UncheckedGet<GfMatrix4d>(),
                       b.UncheckedGet<GfMatrix4d>());
    }
    if (a.IsHolding<VtArray<GfVec2f>>()) {
        const VtArray<GfVec2f> &x = a.UncheckedGet<VtArray<GfVec2f>>();
        const VtArray<GfVec2f> &y = b.UncheckedGet<VtArray<GfVec2f>>();
        return x.size() == y.size() &&
               (x.empty() || std::memcmp(x.cdata(), y.cdata(),
                                         x.size() * sizeof(GfVec2f)) == 0);
    }
    return a == b;
}

// Reads after the head tier.

namespace {

template <class T>
bool
ResolveReader(const RigExecBakedProgramImpl &B, int walk, T *out)
{
    return walk >= 0 && size_t(walk) < B.readerWalks.size() &&
           RigExecBakedResolveWalk(B, B.readerWalks[size_t(walk)].walk, out);
}

}  // namespace

bool
RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &B, int walk,
                              double *out)
{
    return ResolveReader(B, walk, out);
}

bool
RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &B, int walk,
                              float *out)
{
    return ResolveReader(B, walk, out);
}

bool
RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &B, int walk,
                              int *out)
{
    return ResolveReader(B, walk, out);
}

bool
RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &B, int walk,
                              bool *out)
{
    return ResolveReader(B, walk, out);
}

bool
RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &B, int walk,
                              TfToken *out)
{
    return ResolveReader(B, walk, out);
}

bool
RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &B, int walk,
                              GfMatrix4d *out)
{
    return ResolveReader(B, walk, out);
}

bool
RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &B, int walk,
                              GfVec3d *out)
{
    return ResolveReader(B, walk, out);
}

bool
RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &B, int walk,
                              GfVec3f *out)
{
    return ResolveReader(B, walk, out);
}

bool
RigExecBakedReadIsShadowed(
    const RigExecBakedProgramImpl &B,
    const std::vector<std::pair<uint32_t, uint32_t>> &shadowed, uint32_t id)
{
    bool found = false;
    for (const auto &[version, record] : shadowed) {
        if (version != id) {
            continue;
        }
        if (B.recordStoodAside[record]) {
            return false;
        }
        found = true;
    }
    return found;
}

void
RigExecBakedNoteReaderWalks(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    for (size_t w = 0; w < B.readerWalks.size(); ++w) {
        const RigExecBakedReaderWalk &reader = B.readerWalks[w];
        bool moved = false;
        for (const uint32_t leaf : reader.leaves) {
            moved = moved || B.headLeaves[leaf].changed;
        }
        for (const uint32_t slot : reader.slots) {
            moved = moved || B.headOverrideMoved[slot];
        }
        for (const uint32_t id : reader.versions) {
            moved = moved || (B.propertyChanged[id] &&
                              !RigExecBakedReadIsShadowed(B, reader.shadowed,
                                                          id));
        }
        B.readerWalkMoved[w] = moved ? 1 : 0;
        B.readerWalkChanged[w] = 0;
    }
}

namespace {

// _LeafRead's Resolved (the walk, else the head's own value) and
// ResolvedOnly (the walk) arms over \p fallback.
template <class T>
T
WalkedPathRead(const RigExecBakedProgramImpl &B, int walk, bool raw,
               T value)
{
    if (RigExecBakedResolveReaderWalk(B, walk, &value) || !raw) {
        return value;
    }
    const int leaf = B.readerWalks[size_t(walk)].rawLeaf;
    if (leaf >= 0 && B.headLeaves[size_t(leaf)].value.IsHolding<T>()) {
        value = B.headLeaves[size_t(leaf)].value.UncheckedGet<T>();
    }
    return value;
}

template <class T>
VtValue
WalkedPathSample(const RigExecBakedProgramImpl &B,
                 const RigExecRevisionLeafKey &key, int walk)
{
    const T fallback =
        key.fallback.IsHolding<T>() ? key.fallback.UncheckedGet<T>() : T();
    return VtValue(WalkedPathRead<T>(
        B, walk, key.flavour == RigExecRevisionLeafFlavour::Resolved,
        fallback));
}

}  // namespace

VtValue
RigExecBakedSampleWalkedPathLeaf(const RigExecBakedProgramImpl &B,
                                 const RigExecRevisionLeafKey &key, int walk)
{
    using Type = RigExecRevisionLeafType;
    switch (key.type) {
    case Type::Bool: return WalkedPathSample<bool>(B, key, walk);
    case Type::Int: return WalkedPathSample<int>(B, key, walk);
    case Type::Float: return WalkedPathSample<float>(B, key, walk);
    case Type::Token: return WalkedPathSample<TfToken>(B, key, walk);
    case Type::Vec3d: return WalkedPathSample<GfVec3d>(B, key, walk);
    case Type::Matrix4d: return WalkedPathSample<GfMatrix4d>(B, key, walk);
    case Type::Dial:
        // A float dial reads as a float and widens; any other as a double.
        if (B.readerWalks[size_t(walk)].walk.type == HeadType::Float) {
            return VtValue(double(WalkedPathRead<float>(B, walk, true, 0.0f)));
        }
        return VtValue(WalkedPathRead<double>(B, walk, true, 0.0));
    default:
        break;
    }
    return key.fallback;
}

// Build.

void
RigExecBakedProgram::_BindPropertyChains(const RigExecRigEvaluator &E,
                                         RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    B.propertyChains.clear();
    B.propertyRecords.clear();
    B.headLeaves.clear();
    B.headOverrideSlots.clear();
    B.headOverrideSlotsByName.clear();
    BindState state;
    state.program = &B;

    // The chains, in _propertyChainOrder, which is the order the evaluator
    // binds and runs them in.
    std::vector<const std::vector<RigExecRigEvaluator::_PropertyRevision> *>
        revisionsOf;
    for (const SdfPath &target : E._propertyChainOrder) {
        const auto found = E._propertyChains.find(target);
        if (found == E._propertyChains.end()) {
            continue;
        }
        state.chainOfTarget.emplace(target, int(B.propertyChains.size()));
        Chain chain;
        chain.target = target;
        chain.linePrefix = "property chain " + target.GetString();
        B.propertyChains.push_back(std::move(chain));
        revisionsOf.push_back(&found->second);
    }
    // Their phased records, chain by chain in _phasedConnections order.
    for (size_t c = 0; c < B.propertyChains.size(); ++c) {
        Chain &chain = B.propertyChains[c];
        for (const RigExecPhasedConnection &connection :
             E._phasedConnections) {
            if (connection.target != chain.target) {
                continue;
            }
            RigExecBakedPropertyRecord record;
            record.chain = uint32_t(c);
            record.consumer = connection.consumer;
            record.consumerType = connection.consumerType;
            record.applied = connection.applied;
            for (const SdfPath &hop : connection.hops) {
                record.hopSlots.push_back(state.Slot(hop));
            }
            state.recordOfConsumer.emplace(
                connection.consumer, int(B.propertyRecords.size()));
            chain.records.push_back(uint32_t(B.propertyRecords.size()));
            B.propertyRecords.push_back(std::move(record));
        }
    }
    const UsdStageRefPtr &stage = B.stage;
    for (size_t c = 0; c < B.propertyChains.size(); ++c) {
        Chain &chain = B.propertyChains[c];
        const int reader = int(c);
        const UsdAttribute target = stage->GetAttributeAtPath(chain.target);
        chain.targetExists = bool(target);
        chain.targetSlot = state.Slot(chain.target);
        if (target) {
            chain.valueType = target.GetTypeName();
        }
        // runChain's arm: every type the compiler admits beyond these three
        // is GfVec3f-backed.
        chain.arm = chain.valueType == SdfValueTypeNames->Float
                        ? Chain::Arm::Float
                    : chain.valueType == SdfValueTypeNames->Double
                        ? Chain::Arm::Double
                    : chain.valueType == SdfValueTypeNames->Matrix4d
                        ? Chain::Arm::Matrix4d
                        : Chain::Arm::Vec3f;
        if (target) {
            chain.ownLeaf = state.Leaf(target, ArmType(chain.arm));
        }
        const bool floatArm = chain.arm == Chain::Arm::Float ||
                              chain.arm == Chain::Arm::Double;
        for (const RigExecRigEvaluator::_PropertyRevision &revision :
             *revisionsOf[c]) {
            Chain::Revision r;
            r.mover = revision.moverPath;
            r.linePrefix = "diag " + r.mover.GetString();
            const UsdPrim mover = stage->GetPrimAtPath(r.mover);
            r.moverExists = bool(mover);
            // Structural: the operation is folded and the weight object
            // decides whether the op is volatile.
            B.rebuild.insert(r.mover.AppendProperty(kOperation));
            B.rebuild.insert(r.mover.AppendProperty(kWeightObject));
            if (!mover) {
                chain.revisions.push_back(std::move(r));
                continue;
            }
            if (const UsdRelationship rel =
                    mover.GetRelationship(kWeightObject)) {
                SdfPathVector targets;
                rel.GetTargets(&targets);
                if (!targets.empty()) {
                    r.weightObject = targets[0];
                }
            }
            r.enabled = state.Walk(mover.GetAttribute(kEnabled),
                                   HeadType::Bool, reader);
            r.defaultWeight = state.Walk(mover.GetAttribute(kDefaultWeight),
                                         HeadType::Float, reader);
            // rigExec:operation at Default, off the attribute, as
            // _ReadOperation reads it.
            TfToken operation;
            if (const UsdAttribute a = mover.GetAttribute(kOperation)) {
                a.Get(&operation, UsdTimeCode::Default());
            }
            r.opValid = RigExecParsePropertyOp(operation, &r.op);
            const UsdAttribute value = mover.GetAttribute(kValue);
            switch (chain.arm) {
            case Chain::Arm::Float:
            case Chain::Arm::Double:
                r.value = state.Walk(value, HeadType::Float, reader);
                r.minimum = state.Walk(mover.GetAttribute(kMin),
                                       HeadType::Float, reader);
                r.maximum = state.Walk(mover.GetAttribute(kMax),
                                       HeadType::Float, reader);
                break;
            case Chain::Arm::Vec3f:
                r.value = state.Walk(value, HeadType::Vec3f, reader);
                r.minimum = state.Walk(mover.GetAttribute(kMin),
                                       HeadType::Vec3f, reader);
                r.maximum = state.Walk(mover.GetAttribute(kMax),
                                       HeadType::Vec3f, reader);
                break;
            case Chain::Arm::Matrix4d:
                r.value = state.Walk(value, HeadType::Matrix4d, reader);
                break;
            }
            // Keys and tangents are read only by a curve, which only the
            // float arm applies.
            if (floatArm && r.opValid && r.op == RigExecPropertyOp::Curve) {
                const UsdAttribute tangents = mover.GetAttribute(kTangents);
                r.keys = state.Walk(mover.GetAttribute(kKeys),
                                    HeadType::Vec2fArray, reader);
                r.tangents =
                    state.Walk(tangents, HeadType::Vec2fArray, reader);
                r.hasTangents = bool(tangents);
            }
            chain.revisions.push_back(std::move(r));
        }
    }
    // The reads after the head tier that a chain or record can answer: the
    // chain-routed bindings, and the path leaves read through the resolved
    // inputs. The rest read nothing but overrides through the overlay.
    B.readerWalks.clear();
    frozenDetail::_ForEachPatchableInput(B, [&](auto &input) {
        using T = std::decay_t<decltype(input.constant)>;
        input.walk = input.resolvedAttr
                         ? state.ReaderWalk(input.resolvedAttr,
                                            HeadTypeOf<T>(),
                                            /*rawFallback=*/false)
                         : -1;
    });
    const auto bindPathLeaves = [&](RigExecBakedPathLeaves *leaves) {
        const size_t n = leaves->decl.keys.size();
        leaves->walks.assign(n, -1);
        for (size_t k = 0; k < n && k < leaves->attributes.size(); ++k) {
            const RigExecRevisionLeafKey &key = leaves->decl.keys[k];
            HeadType type;
            if (PathLeafHeadType(key, leaves->attributes[k], &type)) {
                leaves->walks[k] = state.ReaderWalk(
                    leaves->attributes[k], type,
                    key.flavour == RigExecRevisionLeafFlavour::Resolved);
            }
        }
    };
    for (RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        for (RigExecBakedProgramImpl::GeomRevision &revision :
             chain.revisions) {
            bindPathLeaves(&revision.leaves);
        }
        for (RigExecBakedProgramImpl::GeomChain::Derived &derived :
             chain.derived) {
            bindPathLeaves(&derived.revision.leaves);
        }
    }
    // The frozen sample keys. One leaf per attribute can match its type, so
    // the keys are distinct, and none is an attribute's own path.
    std::vector<char> own(B.headLeaves.size(), 0);
    for (const Chain &chain : B.propertyChains) {
        if (chain.ownLeaf >= 0) {
            own[size_t(chain.ownLeaf)] = 1;
        }
    }
    for (size_t i = 0; i < B.headLeaves.size(); ++i) {
        RigExecBakedHeadLeaf &leaf = B.headLeaves[i];
        if (!leaf.typeMatches || !leaf.path.IsPropertyPath()) {
            continue;
        }
        leaf.frozenKey = leaf.path.GetPrimPath().AppendProperty(
            TfToken((own[i] ? "frozenChainOwn:" : "frozenChainHop:") +
                    leaf.path.GetName()));
    }
    for (const auto &[path, slot] : B.headOverrideSlots) {
        if (path.IsPrimPropertyPath()) {
            B.headOverrideSlotsByName.emplace(
                std::make_pair(path.GetParentPath(), path.GetNameToken()),
                slot);
        }
    }
}

namespace {

// Ascending ids as head ranges, one per run of consecutive ids.
std::vector<RigExecBakedSlotRange>
Ranges(const std::set<uint32_t> &ids)
{
    std::vector<RigExecBakedSlotRange> out;
    for (const uint32_t id : ids) {
        if (!out.empty() && out.back().end == id) {
            ++out.back().end;
        } else {
            out.push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::PropertyResult, id));
        }
    }
    return out;
}

// Whether record \p record answers a walk that reads its consumer hop as
// \p type. The overlay answers a read only in its exact type, so a walk
// passes a record of another type by and can still reach the target's
// final version behind it.
bool
RecordAnswers(const RigExecBakedProgramImpl &B, int record, HeadType type)
{
    return B.propertyRecords[size_t(record)].consumerType.GetType() ==
           TypeOf(type);
}

template <class Fn>
void
ForEachWalk(const Chain::Revision &r, Fn &&fn)
{
    for (const RigExecBakedWalk *walk :
         {&r.enabled, &r.defaultWeight, &r.value, &r.minimum, &r.maximum,
          &r.keys, &r.tangents}) {
        fn(*walk);
    }
}

}  // namespace

void
RigExecBakedDeclareHeadReads(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    const auto pathWalks = [](const RigExecBakedPathLeaves &leaves,
                              std::vector<int> *out) {
        for (const int walk : leaves.walks) {
            if (walk >= 0) {
                out->push_back(walk);
            }
        }
    };
    for (RigExecBakedStep &step : B.steps) {
        if (step.isHead) continue;
        if (step.kind == RigExecBakedStepKind::RevisionStatic) {
            const auto &[c, r] = B.revisionIndex[size_t(step.object)];
            pathWalks(B.chains[size_t(c)].revisions[size_t(r)].leaves,
                      &step.readerWalks);
        } else if (step.kind == RigExecBakedStepKind::Derived) {
            const auto &[c, d] = B.derivedIndex[size_t(step.object)];
            pathWalks(B.chains[size_t(c)].derived[size_t(d)].revision.leaves,
                      &step.readerWalks);
        }
        std::sort(step.readerWalks.begin(), step.readerWalks.end());
        step.readerWalks.erase(
            std::unique(step.readerWalks.begin(), step.readerWalks.end()),
            step.readerWalks.end());
        std::set<uint32_t> versions, unshadowed;
        std::set<std::pair<uint32_t, uint32_t>> shadowed;
        for (const int walk : step.readerWalks) {
            const RigExecBakedReaderWalk &reader =
                B.readerWalks[size_t(walk)];
            versions.insert(reader.versions.begin(), reader.versions.end());
            for (const uint32_t id : reader.versions) {
                bool byRecord = false;
                for (const auto &pair : reader.shadowed) {
                    if (pair.first == id) {
                        shadowed.insert(pair);
                        byRecord = true;
                    }
                }
                if (!byRecord) {
                    unshadowed.insert(id);
                }
            }
        }
        const auto ranges = Ranges(versions);
        step.reads.insert(step.reads.end(),ranges.begin(),ranges.end());
        step.shadowedReads.clear();
        for (const auto &[id, record] : shadowed) {
            if (!unshadowed.count(id)) {
                step.shadowedReads.emplace_back(id, record);
            }
        }
    }
    B.avarHeadReads.assign(B.paths.size(), {});
    for (const RigExecBakedProgramImpl::AvarBinding &binding :
         B.avarBindings) {
        if (binding.input.walk >= 0 && binding.slot / 11 < B.paths.size()) {
            const RigExecBakedReaderWalk &reader =
                B.readerWalks[size_t(binding.input.walk)];
            std::vector<uint32_t> &reads = B.avarHeadReads[binding.slot / 11];
            reads.insert(reads.end(), reader.versions.begin(),
                         reader.versions.end());
            std::sort(reads.begin(), reads.end());
            reads.erase(std::unique(reads.begin(), reads.end()), reads.end());
        }
    }
}

void
RigExecBakedBuildPropertySteps(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    uint32_t id = 0;
    for (Chain &chain : B.propertyChains) {
        chain.versionBase = id;
        id += uint32_t(chain.revisions.size()) + 1;
    }
    for (RigExecBakedPropertyRecord &record : B.propertyRecords) {
        record.id = id++;
    }
    B.propertyVersionCount = id;
    B.steps.clear();
    for (size_t c = 0; c < B.propertyChains.size(); ++c) {
        const Chain &chain = B.propertyChains[c];
        const size_t n = chain.revisions.size();
        for (size_t k = 0; k <= n; ++k) {
            RigExecBakedStep step;
            step.isHead = true;
            step.kind = RigExecBakedStepKind::PropertyRevision;
            step.object = int(c);
            step.part = int(k);
            std::set<uint32_t> reads, writes, leaves, slots;
            std::set<std::pair<uint32_t, uint32_t>> shadowed;
            std::set<uint32_t> unshadowed;
            if (k == 0) {
                if (chain.ownLeaf >= 0) {
                    leaves.insert(uint32_t(chain.ownLeaf));
                }
                if (chain.targetSlot >= 0) {
                    slots.insert(uint32_t(chain.targetSlot));
                }
                step.label = "PropertyRevision " + chain.target.GetString() +
                             " base";
            } else {
                const Chain::Revision &r = chain.revisions[k - 1];
                reads.insert(chain.versionBase + uint32_t(k) - 1);
                ForEachWalk(r, [&](const RigExecBakedWalk &walk) {
                    // The record that answered the walk last, per chain.
                    std::map<int, uint32_t> metRecord;
                    for (const std::vector<RigExecBakedWalkHop> *list :
                         {&walk.hops, &walk.doubleHops}) {
                        const HeadType readAs = list == &walk.doubleHops
                                                    ? HeadType::Double
                                                    : walk.type;
                        for (const RigExecBakedWalkHop &hop : *list) {
                            if (hop.overrideSlot >= 0) {
                                slots.insert(uint32_t(hop.overrideSlot));
                            }
                            if (hop.leaf >= 0) {
                                leaves.insert(uint32_t(hop.leaf));
                            }
                            if (hop.chain >= 0) {
                                const Chain &other =
                                    B.propertyChains[size_t(hop.chain)];
                                const uint32_t last =
                                    other.versionBase +
                                    uint32_t(other.revisions.size());
                                reads.insert(last);
                                const auto met = metRecord.find(hop.chain);
                                if (met != metRecord.end()) {
                                    shadowed.emplace(last, met->second);
                                } else {
                                    unshadowed.insert(last);
                                }
                            }
                            if (hop.record >= 0) {
                                const RigExecBakedPropertyRecord &record =
                                    B.propertyRecords[size_t(hop.record)];
                                reads.insert(record.id);
                                if (RecordAnswers(B, hop.record, readAs)) {
                                    metRecord[int(record.chain)] =
                                        uint32_t(hop.record);
                                }
                            }
                        }
                    }
                });
                step.alwaysRuns = !r.weightObject.IsEmpty();
                step.label = "PropertyRevision " + r.mover.GetString();
            }
            writes.insert(chain.versionBase + uint32_t(k));
            for (const uint32_t record : chain.records) {
                const RigExecBakedPropertyRecord &rec =
                    B.propertyRecords[record];
                if (std::min(rec.applied, n) == k) {
                    writes.insert(rec.id);
                }
            }
            step.reads = Ranges(reads);
            step.writes = Ranges(writes);
            for (const auto &[id, record] : shadowed) {
                if (!unshadowed.count(id)) {
                    step.shadowedReads.emplace_back(id, record);
                }
            }
            step.leaves.assign(leaves.begin(), leaves.end());
            step.overrideSlots.assign(slots.begin(), slots.end());
            B.steps.push_back(std::move(step));
        }
    }
    // Run state, sized once.
    B.propertyValues.assign(B.propertyVersionCount,
                            RigExecBakedPropertyValue());
    B.propertyVersionValid.assign(B.propertyVersionCount, 0);
    B.propertyChanged.assign(B.propertyVersionCount, 0);
    B.chainValid.assign(B.propertyChains.size(), 0);
    B.chainFinal.assign(B.propertyChains.size(), VtValue());
    B.recordValues.assign(B.propertyRecords.size(), VtValue());
    B.recordStoodAside.assign(B.propertyRecords.size(), 0);
    B.headOverrides.assign(B.headOverrideSlots.size(), VtValue());
    B.lastHeadOverrides.assign(B.headOverrideSlots.size(), VtValue());
    B.headOverrideMoved.assign(B.headOverrideSlots.size(), 0);
    B.headLeavesSampled = false;
    B.headEverRan = false;
    // What each reader walk declares: the final version of every chain
    // whose target it meets and every record whose consumer it meets.
    for (RigExecBakedReaderWalk &reader : B.readerWalks) {
        std::set<uint32_t> versions, unshadowed;
        std::set<std::pair<uint32_t, uint32_t>> shadowed;
        // The record that answered the walk last, per chain.
        std::map<uint32_t, uint32_t> metRecord;
        for (const std::vector<RigExecBakedWalkHop> *hops :
             {&reader.walk.hops, &reader.walk.doubleHops}) {
            const HeadType readAs = hops == &reader.walk.doubleHops
                                        ? HeadType::Double
                                        : reader.walk.type;
            for (const RigExecBakedWalkHop &hop : *hops) {
                if (hop.chain >= 0) {
                    const Chain &chain = B.propertyChains[size_t(hop.chain)];
                    const uint32_t last =
                        chain.versionBase + uint32_t(chain.revisions.size());
                    versions.insert(last);
                    const auto met = metRecord.find(uint32_t(hop.chain));
                    if (met != metRecord.end()) {
                        shadowed.emplace(last, met->second);
                    } else {
                        unshadowed.insert(last);
                    }
                }
                if (hop.record >= 0) {
                    const RigExecBakedPropertyRecord &record =
                        B.propertyRecords[size_t(hop.record)];
                    versions.insert(record.id);
                    if (RecordAnswers(B, hop.record, readAs)) {
                        metRecord[record.chain] = uint32_t(hop.record);
                    }
                }
            }
        }
        reader.versions.assign(versions.begin(), versions.end());
        reader.shadowed.clear();
        for (const auto &[id, record] : shadowed) {
            if (!unshadowed.count(id)) {
                reader.shadowed.emplace_back(id, record);
            }
        }
    }
    B.readerWalkMoved.assign(B.readerWalks.size(), 0);
    B.readerWalkChanged.assign(B.readerWalks.size(), 0);
}

// The prologue.

void
RigExecBakedSampleHeadLeaves(RigExecBakedProgramImpl *program,
                             UsdTimeCode time, bool all)
{
    RigExecBakedProgramImpl &B = *program;
    all = all || !B.headLeavesSampled || B.headLeafStamp != B.programStamp;
    const bool timeMoved = !B.headLeavesSampled || time != B.headLeafTime;
    // Default and a numeric time read different opinions of an attribute
    // that holds both, whatever its variance.
    const bool defaultMoved =
        timeMoved && (time.IsDefault() || B.headLeafTime.IsDefault());
    for (RigExecBakedHeadLeaf &leaf : B.headLeaves) {
        leaf.changed = 0;
        const bool rebind = all || leaf.mustSample;
        if (!rebind && !(timeMoved && (leaf.varying || defaultMoved))) {
            continue;
        }
        if (rebind) {
            // An edit can author time samples where there were none.
            leaf.varying = Varies(leaf.attribute);
        }
        leaf.mustSample = 0;
        ++B.headLeafSamples;
        // The upstream layer stands where the stage value stood (rule 8
        // marks the leaf when a value there is placed, moved or lifted).
        const auto upstream =
            B.upstream.empty() ? B.upstream.end() : B.upstream.find(leaf.path);
        VtValue value =
            upstream != B.upstream.end() && leaf.typeMatches &&
                    RigExecBakedHeadLeafHolds(leaf, upstream->second)
                ? upstream->second
                : SampleHead(leaf, time);
        leaf.changed = RigExecBakedHeadValueSame(value, leaf.value) ? 0 : 1;
        leaf.value = std::move(value);
    }
    B.headLeavesSampled = true;
    B.headLeafTime = time;
    B.headLeafStamp = B.programStamp;
}

VtValue
RigExecBakedReadHeadLeaf(const RigExecBakedHeadLeaf &leaf, UsdTimeCode time)
{
    return SampleHead(leaf, time);
}

bool
RigExecBakedHeadLeafVaries(const RigExecBakedHeadLeaf &leaf)
{
    return Varies(leaf.attribute);
}

bool
RigExecBakedHeadLeafHolds(const RigExecBakedHeadLeaf &leaf,
                          const VtValue &value)
{
    switch (leaf.type) {
    case HeadType::Bool: return value.IsHolding<bool>();
    case HeadType::Float: return value.IsHolding<float>();
    case HeadType::Double: return value.IsHolding<double>();
    case HeadType::Vec3f: return value.IsHolding<GfVec3f>();
    case HeadType::Matrix4d: return value.IsHolding<GfMatrix4d>();
    case HeadType::Vec2fArray: return value.IsHolding<VtArray<GfVec2f>>();
    case HeadType::Int: return value.IsHolding<int>();
    case HeadType::Token: return value.IsHolding<TfToken>();
    case HeadType::Vec3d: return value.IsHolding<GfVec3d>();
    }
    return false;
}

void
RigExecBakedPlaceHeadOverrides(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    const size_t slots = B.headOverrideSlots.size();
    B.headOverrides.assign(slots, VtValue());
    if (slots && B.interactiveOverrides) {
        // As _ApplyInteractiveOverrides places them: an attribute override
        // at its property, the last of several winning.
        for (const RigExecValueOverride &o : *B.interactiveOverrides) {
            if (o.attribute.IsEmpty()) {
                continue;
            }
            const auto found = B.headOverrideSlotsByName.find(
                std::make_pair(o.prim, o.attribute));
            if (found != B.headOverrideSlotsByName.end()) {
                B.headOverrides[found->second] = o.value;
            }
        }
    }
    for (size_t s = 0; s < slots; ++s) {
        B.headOverrideMoved[s] = RigExecBakedHeadValueSame(
                                     B.headOverrides[s],
                                     B.lastHeadOverrides[s])
                                     ? 0
                                     : 1;
    }
    std::fill(B.propertyChanged.begin(), B.propertyChanged.end(), char(0));
    for (size_t r = 0; r < B.propertyRecords.size(); ++r) {
        const RigExecBakedPropertyRecord &record = B.propertyRecords[r];
        bool standing = false, moved = false;
        for (const int slot : record.hopSlots) {
            standing = standing || !B.headOverrides[size_t(slot)].IsEmpty();
            moved = moved || B.headOverrideMoved[size_t(slot)];
        }
        B.recordStoodAside[r] = standing ? 1 : 0;
        B.propertyChanged[record.id] = moved ? 1 : 0;
    }
}

namespace {

template <class T>
T &
Member(RigExecBakedPropertyValue &v);
template <>
float &
Member<float>(RigExecBakedPropertyValue &v)
{
    return v.f;
}
template <>
double &
Member<double>(RigExecBakedPropertyValue &v)
{
    return v.d;
}
template <>
GfVec3f &
Member<GfVec3f>(RigExecBakedPropertyValue &v)
{
    return v.v;
}
template <>
GfMatrix4d &
Member<GfMatrix4d>(RigExecBakedPropertyValue &v)
{
    return v.m;
}

// The version as a published value, in the chain's own type.
VtValue
VersionValue(const RigExecBakedProgramImpl &B, const Chain &chain,
             uint32_t id)
{
    const RigExecBakedPropertyValue &v = B.propertyValues[id];
    switch (chain.arm) {
    case Chain::Arm::Float: return VtValue(v.f);
    case Chain::Arm::Double: return VtValue(v.d);
    case Chain::Arm::Matrix4d: return VtValue(v.m);
    case Chain::Arm::Vec3f: return VtValue(v.v);
    }
    return VtValue();
}

// runChain's base: a drag on the target converted to the chain's type,
// else the target's own value; skipped when neither is a usable number.
template <class ValueT>
bool
Base(const RigExecBakedProgramImpl &B, const Chain &chain,
     std::vector<std::string> *lines, ValueT *value)
{
    bool haveBase = false;
    const VtValue *drag = chain.targetSlot >= 0
                              ? &B.headOverrides[size_t(chain.targetSlot)]
                              : nullptr;
    if (drag && !drag->IsEmpty()) {
        const VtValue held =
            RigExecPhasedConsumerValue(*drag, chain.valueType);
        if (held.IsHolding<ValueT>()) {
            *value = held.UncheckedGet<ValueT>();
            haveBase = true;
        }
    }
    if (!haveBase) {
        const VtValue *own =
            chain.ownLeaf >= 0 ? &B.headLeaves[size_t(chain.ownLeaf)].value
                               : nullptr;
        if (!own || !own->IsHolding<ValueT>()) {
            lines->push_back(chain.linePrefix +
                             ": target has no authored value; chain skipped");
            return false;
        }
        *value = own->UncheckedGet<ValueT>();
    }
    // The same line for a held base as for an authored one.
    if (!IsFinite(*value)) {
        lines->push_back(chain.linePrefix +
                         ": authored base is not finite; chain skipped");
        return false;
    }
    return true;
}

template <class T>
T
Read(const RigExecBakedProgramImpl &B, const RigExecBakedWalk &walk,
     T fallback)
{
    T value;
    return RigExecBakedResolveWalk(B, walk, &value) ? value : fallback;
}

// _ReadPinnedPropertyMathParams' value, min and max over the walks.
template <class T>
bool
MathParams(const RigExecBakedProgramImpl &B, const Chain::Revision &r,
           RigExecPropertyMathParams<T> *params)
{
    if (!r.opValid) {
        return false;
    }
    params->op = r.op;
    params->value = Read(B, r.value, params->value);
    params->min = Read(B, r.minimum, params->min);
    params->max = Read(B, r.maximum, params->max);
    return true;
}

// applyFloat.
bool
ApplyFloat(const RigExecBakedProgramImpl &B, const Chain::Revision &r,
           float in, float envelope, float *out)
{
    RigExecPropertyMathParams<float> params;
    if (!MathParams(B, r, &params) || !IsFinite(params.value) ||
        !IsFinite(params.min) || !IsFinite(params.max)) {
        return false;
    }
    // Held here so the borrowed key pointer outlives the apply.
    VtArray<GfVec2f> keys;
    VtArray<GfVec2f> tangents;
    if (params.op == RigExecPropertyOp::Curve) {
        keys = Read(B, r.keys, keys);
        if (keys.empty() ||
            !RigExecValidateLinearKeys(keys.cdata(), keys.size())) {
            return false;
        }
        params.keys = keys.cdata();
        params.keyCount = keys.size();
        if (r.hasTangents) {
            tangents = Read(B, r.tangents, tangents);
        }
        if (!tangents.empty()) {
            if (tangents.size() != keys.size()) {
                return false;
            }
            params.tangents = tangents.cdata();
            params.tangentCount = tangents.size();
        }
    }
    params.weight = envelope;
    *out = RigExecApplyFloatMath(in, params);
    return true;
}

bool
Apply(const RigExecBakedProgramImpl &B, const Chain::Revision &r, float in,
      float envelope, float *out)
{
    return ApplyFloat(B, r, in, envelope, out);
}

// A double chain is computed in float.
bool
Apply(const RigExecBakedProgramImpl &B, const Chain::Revision &r, double in,
      float envelope, double *out)
{
    float result = 0.0f;
    if (!ApplyFloat(B, r, float(in), envelope, &result)) {
        return false;
    }
    *out = double(result);
    return true;
}

bool
Apply(const RigExecBakedProgramImpl &B, const Chain::Revision &r,
      const GfMatrix4d &in, float envelope, GfMatrix4d *out)
{
    if (!r.opValid) {
        return false;
    }
    const GfMatrix4d opValue = Read(B, r.value, GfMatrix4d(1.0));
    if (!IsFinite(opValue)) {
        return false;
    }
    return RigExecApplyMatrixMath(in, r.op, opValue, envelope, out);
}

bool
Apply(const RigExecBakedProgramImpl &B, const Chain::Revision &r,
      const GfVec3f &in, float envelope, GfVec3f *out)
{
    RigExecPropertyMathParams<GfVec3f> params;
    if (!MathParams(B, r, &params) || !IsFinite(params.value) ||
        !IsFinite(params.min) || !IsFinite(params.max)) {
        return false;
    }
    params.weight = envelope;
    *out = RigExecApplyVec3fMath(in, params);
    return true;
}

// One revision of runChain's loop: \p out is \p in where the revision
// passes through.
template <class ValueT>
void
Revise(const RigExecBakedProgramImpl &B, const Chain::Revision &r,
       const ValueT &in, UsdTimeCode time, std::vector<std::string> *lines,
       ValueT *out)
{
    *out = in;
    if (!r.moverExists) {
        return;
    }
    if (!Read(B, r.enabled, true)) {
        lines->push_back(r.linePrefix + ": disabled; revision passed through");
        return;
    }
    float envelope = 1.0f;
    if (!r.weightObject.IsEmpty()) {
        std::vector<float> weights;
        std::string error;
        bool resolved = false;
        {
            // Volatile until S4: the oracle reads the weight object off the
            // stage and through the generation's resolved inputs.
            const RigExecVolatileRead oracle;
            resolved = B.resolveWeights &&
                       B.resolveWeights(r.weightObject, 1, time, &weights,
                                        &error, nullptr);
        }
        if (!resolved || weights.size() != 1) {
            lines->push_back(r.linePrefix + ": " + error +
                             "; revision passed through");
            return;
        }
        envelope = weights[0];
    } else {
        envelope = Read(B, r.defaultWeight, 1.0f);
        if (!std::isfinite(envelope) || envelope < 0.0f || envelope > 1.0f) {
            lines->push_back(r.linePrefix +
                             ": inputs:defaultWeight must be finite and in "
                             "[0, 1]; revision passed through");
            return;
        }
    }
    ValueT next = in;
    if (!Apply(B, r, in, envelope, &next)) {
        lines->push_back(r.linePrefix +
                         ": inputs unusable; revision passed through");
        return;
    }
    if (!IsFinite(next)) {
        lines->push_back(r.linePrefix +
                         ": produced a non-finite value; revision passed "
                         "through");
        return;
    }
    *out = next;
}

template <class ValueT>
void
RunPart(RigExecBakedProgramImpl *program, RigExecBakedStep *step,
        UsdTimeCode time)
{
    RigExecBakedProgramImpl &B = *program;
    const size_t c = size_t(step->object);
    const Chain &chain = B.propertyChains[c];
    const uint32_t id = chain.versionBase + uint32_t(step->part);
    std::vector<std::string> *lines = &step->lines;
    ValueT value{};
    bool valid = false;
    if (step->part == 0) {
        if (!chain.targetExists) {
            lines->push_back(chain.linePrefix +
                             ": target attribute disappeared; chain "
                             "skipped");
        } else {
            valid = Base(B, chain, lines, &value);
        }
    } else if (B.chainValid[c]) {
        // A skipped chain runs no revision and says nothing about them.
        const ValueT in = Member<ValueT>(B.propertyValues[id - 1]);
        Revise(B, chain.revisions[size_t(step->part) - 1], in, time, lines,
               &value);
        valid = true;
    }
    ValueT &stored = Member<ValueT>(B.propertyValues[id]);
    const bool wasValid = B.propertyVersionValid[id] != 0;
    B.propertyChanged[id] =
        valid != wasValid || (valid && !BitSame(value, stored)) ? 1 : 0;
    if (valid) {
        stored = value;
    }
    B.propertyVersionValid[id] = valid ? 1 : 0;
    if (step->part == 0) {
        B.chainValid[c] = valid ? 1 : 0;
    }
}

}  // namespace

void
RigExecBakedRunPropertyStep(RigExecBakedProgramImpl *program,
                            RigExecBakedStep *step, UsdTimeCode time)
{
    switch (program->propertyChains[size_t(step->object)].arm) {
    case Chain::Arm::Float: RunPart<float>(program, step, time); return;
    case Chain::Arm::Double: RunPart<double>(program, step, time); return;
    case Chain::Arm::Matrix4d:
        RunPart<GfMatrix4d>(program, step, time);
        return;
    case Chain::Arm::Vec3f: RunPart<GfVec3f>(program, step, time); return;
    }
}

void
RigExecBakedFinishPropertyStep(RigExecBakedProgramImpl *program,
                               const RigExecBakedStep &step)
{
    RigExecBakedProgramImpl &B = *program;
    const Chain &chain = B.propertyChains[size_t(step.object)];
    const size_t n = chain.revisions.size();
    const uint32_t id = chain.versionBase + uint32_t(step.part);
    if (!B.propertyChanged[id]) {
        return;
    }
    // What the version feeds, refreshed only when it moved: the records
    // reading it and, for the last part, the chain's final value.
    for (const uint32_t r : chain.records) {
        RigExecBakedPropertyRecord &record = B.propertyRecords[r];
        if (std::min(record.applied, n) != size_t(step.part)) {
            continue;
        }
        B.propertyChanged[record.id] = 1;
        if (B.propertyVersionValid[id]) {
            B.recordValues[r] = RigExecPhasedConsumerValue(
                VersionValue(B, chain, id), record.consumerType);
        }
    }
    if (size_t(step.part) == n && B.propertyVersionValid[id]) {
        B.chainFinal[size_t(step.object)] = VersionValue(B, chain, id);
    }
}

void
RigExecBakedPublishPropertyChains(RigExecBakedProgramImpl *program,
                                  size_t begin, size_t end, bool results)
{
    RigExecBakedProgramImpl &B = *program;
    end = std::min(end, B.propertyChains.size());
    for (size_t c = begin; c < end; ++c) {
        if (!B.chainValid[c]) {
            continue;
        }
        const Chain &chain = B.propertyChains[c];
        const VtValue &result = B.chainFinal[c];
        if (results) {
            B.propertyResults[chain.target] = result;
        }
        B.resolvedInputs->SetProperty(chain.target, result);
        for (const uint32_t r : chain.records) {
            if (B.recordStoodAside[r]) {
                continue;
            }
            const RigExecBakedPropertyRecord &record = B.propertyRecords[r];
            if (results) {
                B.propertyResults[record.consumer] = B.recordValues[r];
            }
            B.resolvedInputs->SetProperty(record.consumer,
                                          B.recordValues[r]);
        }
    }
}

// The test hook.

bool
RigExecBakedProgram::_EvaluateChainsDetached(
    RigExecRigEvaluator *evaluator, UsdTimeCode time,
    std::map<SdfPath, VtValue> *results, std::vector<std::string> *lines)
{
    RigExecRigEvaluator &E = *evaluator;
    const void *memo = E._propertyChainBindings.get();
    // Swapped by value: the program points at the member, and keeps
    // pointing at the live object once it is swapped back.
    RigExecResolvedInputs live;
    std::swap(E._resolvedInputs, live);
    E._ApplyValueInputsToResolved(&E._resolvedInputs);
    std::unique_ptr<RigExecPropertyChainBindings> saved =
        std::move(E._propertyChainBindings);
    E._EvaluatePropertyChains(time, results, nullptr, lines);
    E._propertyChainBindings = std::move(saved);
    std::swap(E._resolvedInputs, live);
    return E._propertyChainBindings.get() == memo;
}

const void *
RigExecBakedProgram::_ChainMemo(const RigExecRigEvaluator &evaluator)
{
    return evaluator._propertyChainBindings.get();
}

bool
RigExecBakedProgramTesting::EvaluateChainsDetached(
    RigExecRigEvaluator *evaluator, UsdTimeCode time,
    std::map<SdfPath, VtValue> *results, std::vector<std::string> *lines)
{
    return RigExecBakedProgram::_EvaluateChainsDetached(evaluator, time,
                                                        results, lines);
}

const void *
RigExecBakedProgramTesting::ChainMemo(const RigExecRigEvaluator &evaluator)
{
    return RigExecBakedProgram::_ChainMemo(evaluator);
}

}  // namespace rigExec
