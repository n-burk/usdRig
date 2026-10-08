// Rig discovery and transitive pose-input dependencies.

#include "rigEvaluatorInternal.h"
#include "rigEvaluatorDependencies.h"
#include "rigEvaluatorConstraints.h"
#include "parallel.h"
#include "movers/moverRegistry.h"

#include "pxr/base/work/loops.h"
#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"

#include <algorithm>
#include <functional>
#include <set>
#include <unordered_map>

namespace rigExec {

using namespace evaluatorDetail;

namespace {

// Namespace pre-order is part of the binding-epoch identity. Simple path
// discovery shares this traversal and retains the default USD predicate.
template <class Predicate>
std::vector<SdfPath>
_DiscoverPaths(const UsdStageRefPtr &stage, const SdfPath &rigPath,
               Predicate matches)
{
    std::vector<SdfPath> paths;
    const UsdPrim rig = stage ? stage->GetPrimAtPath(rigPath) : UsdPrim();
    if (rig) {
        for (const UsdPrim &prim : UsdPrimRange(rig)) {
            if (matches(prim)) {
                paths.push_back(prim.GetPath());
            }
        }
    }
    return paths;
}

// Resolve a validation subject to its owning operation without inspecting
// diagnostic text. Data prims may live beneath an operation (for example poses).
SdfPath
_OwningOperation(UsdPrim prim, const SdfPath &rigPath)
{
    for (; prim && prim.GetPath().HasPrefix(rigPath); prim = prim.GetParent()) {
        if (prim.GetRelationship(_movesRel) ||
            _IsAggregateSolverType(prim.GetTypeName()) ||
            prim.GetTypeName() == "RigExecPoseInterpolator" ||
            prim.GetRelationship(TfToken("rigExec:joints"))) {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

// Exact, iterative connection closure used by both the solver cache's
// authored-input index and its topology digest. Keep missing sources and
// cycles in the identity without reading any values or time-sample counts.
static std::set<SdfPath>
_CollectAttributeConnectionInputs(const UsdPrim &prim)
{
    std::set<SdfPath> inputs;
    if (!prim) {
        return inputs;
    }
    std::vector<SdfPath> pending;
    for (const UsdAttribute &attribute : prim.GetAttributes()) {
        pending.push_back(attribute.GetPath());
    }
    while (!pending.empty()) {
        const SdfPath path = pending.back();
        pending.pop_back();
        if (!inputs.insert(path).second) {
            continue;
        }
        const UsdAttribute attribute = prim.GetStage()->GetAttributeAtPath(path);
        if (attribute) {
            const SdfPathVector sources = _AuthoredConnections(attribute);
            pending.insert(pending.end(), sources.begin(), sources.end());
        }
    }
    return inputs;
}

// RIGEXEC_VERIFY_POSEINFO=1: every closure _PoseInputGraph computes is
// recomputed by _CollectPoseInputInfo, the per-prim walker it replaces, and
// the compile fails fatally unless providers, attributes and connectedPose
// all agree.
static bool
_PoseInfoVerifyRequested()
{
    static const bool requested =
        TfGetenvBool("RIGEXEC_VERIFY_POSEINFO", false);
    return requested;
}

// A PoseInfoPrefetch round (_PoseInputGraph::Extend) is spread across the
// pool only when it holds at least this many attribute reads, counting a
// prim listed whole as _kPoseInfoReadsPerListedPrim of them (a RigExec joint
// or control carries 30 to 45 attributes). MEASURED, rounds by task count,
// the first one listing prims whole and the rest reading named attributes:
//   puppetA  84, 12, 22, 10, 20, 10, 12, 2, 2
//   biped    312, 24, 42, 18, 28, 10, 12, 2, 2
// Every round used to be forked, and all but the first hold a few
// microseconds of reads each. With this floor, cold compiles, medians of 8:
// puppetA 3.91 -> 3.66 ms, biped 5.46 -> 5.28 ms. Keeping the first round on
// one thread too is worse (4.96 / 8.19 ms): beside the digest and the exec
// lane a lone reader runs at about half its unshared speed.
constexpr size_t _kPoseInfoReadsPerListedPrim = 32;

constexpr size_t _kPoseInfoParallelMinReads = 256;

} // namespace

namespace evaluatorDetail {

// Discovers the rig's joint output set implicitly (spec §4.1: the rig is a
// namespace root, not a manifest). Movers are already found this way -- a
// reverse-sibling post-order walk where carrying rigExec:moves is what makes a
// prim a mover -- and joints now follow the same rule: being a RigExecJoint
// under the rig is what makes a prim a joint output. Returned in namespace
// pre-order, which reproduces the parent-before-child ordering the authored
// lists used and keeps the binding-epoch digest stable against unrelated edits.
// Operator-declared joints are unioned in afterwards. Solver rigExec:joints
// targets are validated to be RigExecJoint prims later in Compile, so in a
// valid rig they are already a subset of the namespace walk; including them
// means a rig that is midway through an edit still compiles the joints its
// operators actually drive, instead of failing on a set that disagrees with the
// graph. The schema is codeless (skipCodeGeneration), so type identity is a
// type-name comparison -- the same idiom the imaging registry uses to find the
// rig itself. RigExecJoint has no derived types.
std::vector<SdfPath>
_DiscoverJointOutputs(const UsdStageRefPtr &stage, const SdfPath &rigPath,
                      const std::map<SdfPath, std::string> &skipped)
{
    static const TfToken kJointType("RigExecJoint");
    static const TfToken kJointsRel("rigExec:joints");

    std::vector<SdfPath> joints;
    std::set<SdfPath> seen;

    const UsdPrim rig = stage->GetPrimAtPath(rigPath);
    if (!rig) {
        return joints;
    }

    for (const UsdPrim &prim : UsdPrimRange(rig)) {
        if (prim.GetTypeName() == kJointType && seen.insert(prim.GetPath()).second) {
            joints.push_back(prim.GetPath());
        }
    }

    // Union in whatever the operators name, in solver namespace order --
    // leaving out an operator the compile set aside.
    for (const UsdPrim &prim : UsdPrimRange(rig)) {
        if (skipped.count(prim.GetPath())) {
            continue;
        }
        const UsdRelationship jointsRel = prim.GetRelationship(kJointsRel);
        if (!jointsRel) {
            continue;
        }
        SdfPathVector targets;
        jointsRel.GetTargets(&targets);
        for (const SdfPath &target : targets) {
            const UsdPrim joint = stage->GetPrimAtPath(target);
            if (joint && joint.GetTypeName() == kJointType &&
                seen.insert(target).second) {
                joints.push_back(target);
            }
        }
    }

    return joints;
}

// Discovers the rig's pose interpolators the same implicit way: being a
// RigExecPoseInterpolator under the rig is what makes a prim one.
// The interpolators conventionally live at <rig>/PoseInterpolators/<name>,
// but like every other rig element they are found by type wherever they sit.
// An interpolator writes no transform and no points, so it carries no
// rigExec:moves and has no place in the mover ordering.
std::vector<SdfPath>
_DiscoverPoseInterpolators(const UsdStageRefPtr &stage, const SdfPath &rigPath)
{
    return _DiscoverPaths(stage, rigPath, [](const UsdPrim &prim) {
        return prim.GetTypeName() == "RigExecPoseInterpolator";
    });
}

// Discovers the rig's controls the same implicit way (spec §4.1): being a
// RigExecControl under the rig is what makes a prim a control. Returned in
// namespace pre-order so the discovered order -- and with it the epoch
// digest -- is stable against unrelated edits.
// No union pass over operator wiring, unlike the joints. A solver's
// rigExec:controls names inputs it READS, and reading a control does not
// make it one; the type does. And no emptiness rule either: a rig whose
// joints are animated directly has no control prims, which is a legal rig
// that simply draws no control guides.
std::vector<SdfPath>
_DiscoverControls(const UsdStageRefPtr &stage, const SdfPath &rigPath)
{
    return _DiscoverPaths(stage, rigPath, [](const UsdPrim &prim) {
        return prim.GetTypeName() == "RigExecControl";
    });
}

// Discovers every concrete placed weight volume beneath the rig. A volume is
// a viewport output in its own right: the authored falloff surfaces are useful
// while the rigger is placing the field, before any mover consumes it. Keep
// this namespace-based, like joints and controls, so no manifest or temporary
// weight binding is required merely to make the schema's guide contract work.
std::vector<SdfPath>
_DiscoverVolumeWeights(const UsdStageRefPtr &stage, const SdfPath &rigPath)
{
    return _DiscoverPaths(stage, rigPath, [](const UsdPrim &prim) {
        return _IsVolumeWeightType(prim.GetTypeName());
    });
}

// Every solver type that publishes computePointFrameArray for view-free
// joint extraction.
bool
_IsAggregateSolverType(const TfToken &typeName)
{
    static const std::set<TfToken> kAggregateSolverTypes = {
        TfToken("RigExecFkChain"), TfToken("RigExecTwoBoneIk"),
        TfToken("RigExecBlendPointFrames"),
        TfToken("RigExecTwistDistribution"), TfToken("RigExecRibbon"),
        TfToken("RigExecSplineIk")};
    return kAggregateSolverTypes.count(typeName) > 0;
}

// Discovers aggregate solvers by TYPE anywhere beneath the rig (spec §4.1:
// no membership lists). The scope a solver sits under is an authoring
// convenience, not identity -- "Solvers" is the convention, not a
// requirement. Returned in namespace pre-order so the discovered order,
// and with it the epoch digest, is stable against unrelated edits.
std::vector<UsdPrim>
_DiscoverAggregateSolvers(
    const UsdStageRefPtr &stage, const SdfPath &rigPath)
{
    std::vector<UsdPrim> solvers;
    const UsdPrim rig = stage->GetPrimAtPath(rigPath);
    if (!rig) {
        return solvers;
    }
    for (const UsdPrim &prim : UsdPrimRange(rig)) {
        if (_IsAggregateSolverType(prim.GetTypeName())) {
            solvers.push_back(prim);
        }
    }
    return solvers;
}

UsdPrim
_NamespaceFrameProvider(UsdPrim prim)
{
    for (prim = prim.GetParent(); prim; prim = prim.GetParent()) {
        if (_IsFrameProvider(prim)) return prim;
    }
    return UsdPrim();
}

// One prim's closure, walked on its own. _PoseInputGraph computes the same
// closures for many prims at once and is what the compile uses; this stays
// as the reference RIGEXEC_VERIFY_POSEINFO checks the graph against.
_PoseInputInfo
_CollectPoseInputInfo(const UsdPrim &prim)
{
    _PoseInputInfo info;
    if (!prim) return info;
    std::set<SdfPath> attributes;
    std::vector<std::pair<SdfPath, bool>> pending;
    std::set<std::pair<SdfPath, bool>> visited;
    for (const UsdAttribute &attribute : prim.GetAttributes()) {
        pending.emplace_back(attribute.GetPath(), false);
    }
    while (!pending.empty()) {
        const auto entry = pending.back();
        pending.pop_back();
        if (!visited.insert(entry).second) continue;
        const auto &[path, connected] = entry;
        attributes.insert(path);
        const UsdAttribute attribute = prim.GetStage()->GetAttributeAtPath(path);
        if (!attribute) continue;
        const SdfPathVector sources = _AuthoredConnections(attribute);
        for (const SdfPath &source : sources) pending.emplace_back(source, true);
        const UsdPrim provider = attribute.GetPrim();
        SdfPathVector frameInputs;
        if (auto rel = provider.GetRelationship(TfToken("rigExec:poseInputs")))
            rel.GetTargets(&frameInputs);
        for (const SdfPath &input : frameInputs) {
            info.providers.insert(input);
            info.connectedPose = info.connectedPose || connected;
        }
        if (!_IsFrameProvider(provider)) continue;
        const TfToken name = attribute.GetName();
        const auto add = [&](const UsdPrim &owner, const char *input) {
            if (owner) pending.emplace_back(owner.GetPath().AppendProperty(
                TfToken(input)), connected);
        };
        if (name == "parent:space") {
            const UsdPrim parent = _NamespaceFrameProvider(provider);
            if (parent) {
                info.providers.insert(parent.GetPath());
                info.connectedPose = info.connectedPose || connected;
            }
        } else if (name == "parent:defaultSpace") {
            add(_NamespaceFrameProvider(provider), "default:space");
        } else if (name == "posed:defaultSpace") {
            add(provider, "avars:defaultSpace");
        } else if (name == "avars:defaultSpace") {
            add(provider, "default:space");
        } else if (name == "default:space") {
            add(provider, "parent:defaultSpace");
            for (const char *input : {"default:tx", "default:ty", "default:tz",
                 "default:rx", "default:ry", "default:rz", "rest:space",
                 "rest:tx", "rest:ty", "rest:tz", "rest:rx", "rest:ry", "rest:rz"}) {
                add(provider, input);
            }
            const UsdPrim parent = _NamespaceFrameProvider(provider);
            for (const char *input : {"rest:space", "rest:tx", "rest:ty",
                 "rest:tz", "rest:rx", "rest:ry", "rest:rz"}) add(parent, input);
        }
    }
    info.attributes.assign(attributes.begin(), attributes.end());
    return info;
}

uint32_t
_PoseInputGraph::_PrimOf(const SdfPath &path)
{
    const auto [it, added] =
        _primIndex.emplace(path, static_cast<uint32_t>(_prims.size()));
    if (added) {
        _prims.emplace_back().path = path;
    }
    return it->second;
}

uint32_t
_PoseInputGraph::_AttrOf(const SdfPath &path)
{
    const auto [it, added] =
        _attrIndex.emplace(path, static_cast<uint32_t>(_attrs.size()));
    if (!added) {
        return it->second;
    }
    const uint32_t index = it->second;
    _Attr attr;
    attr.path = path;
    // UsdStage::GetAttributeAtPath resolves an absolute prim property path
    // and nothing else, so any other path is a node that exists nowhere.
    if (path.IsAbsolutePath() && path.IsPrimPropertyPath()) {
        attr.prim = _PrimOf(path.GetPrimPath());
    } else {
        attr.read = true;
    }
    _PushAttr(std::move(attr));
    return index;
}

uint32_t
_PoseInputGraph::_AttrOf(const SdfPath &path, uint32_t prim)
{
    const auto [it, added] =
        _attrIndex.emplace(path, static_cast<uint32_t>(_attrs.size()));
    if (added) {
        _Attr attr;
        attr.path = path;
        attr.prim = prim;
        _PushAttr(std::move(attr));
    }
    return it->second;
}

void
_PoseInputGraph::_Reserve(size_t attrs)
{
    if (attrs <= _attrs.capacity()) {
        return;
    }
    _attrs.reserve(attrs);
    _attrIndex.reserve(attrs);
    _edgeBegin.reserve(2 * attrs);
    _edgeCount.reserve(2 * attrs);
    _nodeProvider.reserve(2 * attrs);
    _nodeComp.reserve(2 * attrs);
}

void
_PoseInputGraph::_PushAttr(_Attr &&attr)
{
    _attrs.push_back(std::move(attr));
    for (int connected = 0; connected < 2; ++connected) {
        _edgeBegin.push_back(0);
        _edgeCount.push_back(0);
        _nodeProvider.push_back(_None);
        _nodeComp.push_back(_None);
    }
}

void
_PoseInputGraph::_Ask(uint32_t prim,
                      const std::function<bool(const SdfPath &)> &skip)
{
    if (_prims[prim].asked) {
        return;
    }
    _prims[prim].asked = true;
    if (_prims[prim].path.IsEmpty() || skip(_prims[prim].path)) {
        return;
    }
    _askQueue.push_back(prim);
}

void
_PoseInputGraph::_CreateNode(uint32_t attr, uint32_t connected)
{
    _Attr &a = _attrs[attr];
    const uint8_t bit = uint8_t(1u << connected);
    if (a.created & bit) {
        return;
    }
    a.created |= bit;
    if (a.read) {
        _expand.push_back(2 * attr + connected);
    } else if (!a.pending) {
        a.pending = true;
        std::vector<uint32_t> &pending = _prims[a.prim].pendingAttrs;
        if (pending.empty()) {
            _waiting.push_back(a.prim);
        }
        pending.push_back(attr);
    }
}

void
_PoseInputGraph::_MarkRead(uint32_t attr, bool exists, SdfPathVector &&sources)
{
    _Attr &a = _attrs[attr];
    if (a.read) {
        return;
    }
    a.read = true;
    a.exists = exists;
    a.sources = std::move(sources);
    for (uint32_t connected = 0; connected < 2; ++connected) {
        if (a.created & (1u << connected)) {
            _expand.push_back(2 * attr + connected);
        }
    }
}

void
_PoseInputGraph::_ExpandNode(uint32_t node,
                             const std::function<bool(const SdfPath &)> &skip)
{
    static const TfToken parentSpace("parent:space");
    static const TfToken parentDefaultSpace("parent:defaultSpace");
    static const TfToken posedDefaultSpace("posed:defaultSpace");
    static const TfToken avarsDefaultSpace("avars:defaultSpace");
    static const TfToken defaultSpace("default:space");
    static const TfToken ownInputs[] = {
        TfToken("parent:defaultSpace"), TfToken("default:tx"),
        TfToken("default:ty"), TfToken("default:tz"), TfToken("default:rx"),
        TfToken("default:ry"), TfToken("default:rz"), TfToken("rest:space"),
        TfToken("rest:tx"), TfToken("rest:ty"), TfToken("rest:tz"),
        TfToken("rest:rx"), TfToken("rest:ry"), TfToken("rest:rz")};
    static const TfToken restInputs[] = {
        TfToken("rest:space"), TfToken("rest:tx"), TfToken("rest:ty"),
        TfToken("rest:tz"), TfToken("rest:rx"), TfToken("rest:ry"),
        TfToken("rest:rz")};

    const uint32_t attr = node >> 1;
    const uint32_t connected = node & 1u;
    const uint32_t begin = static_cast<uint32_t>(_edges.size());
    const auto edgeTo = [&](const SdfPath &path, uint32_t toConnected) {
        const uint32_t to = _AttrOf(path);
        _edges.push_back(2 * to + toConnected);
        _CreateNode(to, toConnected);
    };
    if (_attrs[attr].exists) {
        // By index and by value: edgeTo can grow _attrs, and a reference
        // into it would dangle.
        for (size_t i = 0; i < _attrs[attr].sources.size(); ++i) {
            const SdfPath source = _attrs[attr].sources[i];
            edgeTo(source, 1);
        }
        const uint32_t owner = _attrs[attr].prim;
        // Native expression schemas declare frame reads that are carried by
        // Exec relationship inputs rather than USD attribute connections.
        for (size_t i = 0; i < _prims[owner].frameInputs.size(); ++i)
            _Ask(_prims[owner].frameInputs[i], skip);
        const TfToken &name = _attrs[attr].path.GetNameToken();
        const bool ruled = name == parentSpace ||
            name == parentDefaultSpace || name == posedDefaultSpace ||
            name == avarsDefaultSpace || name == defaultSpace;
        if (ruled && _prims[owner].providerType) {
            // Copies: edgeTo can grow _prims.
            const SdfPath self = _prims[owner].path;
            const uint32_t parent = _prims[owner].nsProvider;
            const SdfPath parentPath =
                parent == _None ? SdfPath() : _prims[parent].path;
            if (name == parentSpace) {
                if (parent != _None) {
                    _nodeProvider[node] = parent;
                    _Ask(parent, skip);
                }
            } else if (name == parentDefaultSpace) {
                if (parent != _None) {
                    edgeTo(parentPath.AppendProperty(defaultSpace), connected);
                }
            } else if (name == posedDefaultSpace) {
                edgeTo(self.AppendProperty(avarsDefaultSpace), connected);
            } else if (name == avarsDefaultSpace) {
                edgeTo(self.AppendProperty(defaultSpace), connected);
            } else if (name == defaultSpace) {
                for (const TfToken &input : ownInputs) {
                    edgeTo(self.AppendProperty(input), connected);
                }
                if (parent != _None) {
                    for (const TfToken &input : restInputs) {
                        edgeTo(parentPath.AppendProperty(input), connected);
                    }
                }
            }
        }
    }
    _edgeBegin[node] = begin;
    _edgeCount[node] = static_cast<uint32_t>(_edges.size()) - begin;
}

void
_PoseInputGraph::_Tarjan()
{
    const size_t nodeCount = _nodeComp.size();
    std::vector<uint32_t> index(nodeCount, _None);
    std::vector<uint32_t> low(nodeCount, 0);
    std::vector<uint8_t> onStack(nodeCount, 0);
    std::vector<uint32_t> stack;
    std::vector<std::pair<uint32_t, uint32_t>> calls;  // node, next edge
    std::vector<uint32_t> own;
    std::vector<uint32_t> succSets;
    uint32_t counter = 0;

    const auto enter = [&](uint32_t node) {
        index[node] = low[node] = counter++;
        stack.push_back(node);
        onStack[node] = 1;
        calls.emplace_back(node, 0u);
    };
    const auto complete = [&](uint32_t root) {
        const uint32_t comp = static_cast<uint32_t>(_compProviders.size());
        const size_t membersBegin = _compNodes.size();
        uint32_t member;
        do {
            member = stack.back();
            stack.pop_back();
            onStack[member] = 0;
            _nodeComp[member] = comp;
            _compNodes.push_back(member);
        } while (member != root);
        _compNodesBegin.push_back(static_cast<uint32_t>(_compNodes.size()));
        _compMark.push_back(_None);

        own.clear();
        succSets.clear();
        bool connected = false;
        for (size_t m = membersBegin; m < _compNodes.size(); ++m) {
            const uint32_t node = _compNodes[m];
            if (_nodeProvider[node] != _None) {
                own.push_back(_nodeProvider[node]);
                connected = connected || (node & 1u);
            }
            const auto &attr = _attrs[node >> 1];
            if (attr.exists && attr.prim != _None) {
                const auto &inputs = _prims[attr.prim].frameInputs;
                own.insert(own.end(), inputs.begin(), inputs.end());
                connected = connected || (!inputs.empty() && (node & 1u));
            }
            const uint32_t edgesEnd = _edgeBegin[node] + _edgeCount[node];
            for (uint32_t e = _edgeBegin[node]; e < edgesEnd; ++e) {
                const uint32_t to = _nodeComp[_edges[e]];
                if (to == comp || _compMark[to] == comp) continue;
                _compMark[to] = comp;
                _compSuccs.push_back(to);
                connected = connected || _compConnected[to];
                if (_compProviders[to] != 0) {
                    succSets.push_back(_compProviders[to]);
                }
            }
        }
        _compSuccsBegin.push_back(static_cast<uint32_t>(_compSuccs.size()));

        std::sort(succSets.begin(), succSets.end());
        succSets.erase(std::unique(succSets.begin(), succSets.end()),
                       succSets.end());
        uint32_t providers = 0;
        if (own.empty() && succSets.size() == 1) {
            providers = succSets.front();
        } else if (!own.empty() || !succSets.empty()) {
            for (const uint32_t set : succSets) {
                own.insert(own.end(), _providerSets[set].begin(),
                           _providerSets[set].end());
            }
            std::sort(own.begin(), own.end());
            own.erase(std::unique(own.begin(), own.end()), own.end());
            providers = static_cast<uint32_t>(_providerSets.size());
            _providerSets.push_back(own);
        }
        _compProviders.push_back(providers);
        _compConnected.push_back(connected ? 1 : 0);
    };

    for (uint32_t start = 0; start < nodeCount; ++start) {
        if (!(_attrs[start >> 1].created & (1u << (start & 1u))) ||
            _nodeComp[start] != _None || index[start] != _None) {
            continue;
        }
        enter(start);
        while (!calls.empty()) {
            const uint32_t node = calls.back().first;
            const uint32_t next = calls.back().second;
            if (next < _edgeCount[node]) {
                ++calls.back().second;
                const uint32_t to = _edges[_edgeBegin[node] + next];
                if (_nodeComp[to] != _None) {
                    // Completed, in this run or an earlier Extend: folded
                    // already, and never on the stack.
                    continue;
                }
                if (index[to] == _None) {
                    enter(to);
                } else if (onStack[to]) {
                    low[node] = std::min(low[node], index[to]);
                }
                continue;
            }
            if (low[node] == index[node]) {
                complete(node);
            }
            calls.pop_back();
            if (!calls.empty()) {
                const uint32_t caller = calls.back().first;
                low[caller] = std::min(low[caller], low[node]);
            }
        }
    }
}

void
_PoseInputGraph::_Reach(uint32_t prim, uint32_t stamp,
                        std::vector<uint32_t> *compMark,
                        std::vector<uint32_t> *attrMark,
                        std::vector<uint32_t> *out) const
{
    std::vector<uint32_t> pending;
    for (uint32_t r = _prims[prim].rootsBegin; r < _prims[prim].rootsEnd;
         ++r) {
        const uint32_t comp = _nodeComp[2 * _roots[r]];
        if ((*compMark)[comp] != stamp) {
            (*compMark)[comp] = stamp;
            pending.push_back(comp);
        }
    }
    while (!pending.empty()) {
        const uint32_t comp = pending.back();
        pending.pop_back();
        for (uint32_t m = _compNodesBegin[comp]; m < _compNodesBegin[comp + 1];
             ++m) {
            const uint32_t attr = _compNodes[m] >> 1;
            if ((*attrMark)[attr] != stamp) {
                (*attrMark)[attr] = stamp;
                out->push_back(attr);
            }
        }
        for (uint32_t s = _compSuccsBegin[comp]; s < _compSuccsBegin[comp + 1];
             ++s) {
            const uint32_t to = _compSuccs[s];
            if ((*compMark)[to] != stamp) {
                (*compMark)[to] = stamp;
                pending.push_back(to);
            }
        }
    }
}

void
_PoseInputGraph::Extend(
    const UsdStageRefPtr &stage, const std::vector<SdfPath> &seeds,
    const std::function<bool(const SdfPath &)> &skip, bool parallel,
    RigExecProfiler &profiler,
    std::unordered_map<SdfPath, _PoseInputInfo, SdfPath::Hash> *infos)
{
    std::vector<uint32_t> asked;
    {
        RIGEXEC_PROFILE_SCOPE_CAT(profiler, "PoseInfoPrefetch.Read",
                                  "compile");
        for (const SdfPath &path : seeds) {
            if (!path.IsEmpty()) _Ask(_PrimOf(path), skip);
        }
        while (!_askQueue.empty() || !_waiting.empty()) {
            // One task per prim: all of its attributes if it is asked for,
            // and the ones reached on it either way.
            std::vector<_Task> tasks;
            std::unordered_map<uint32_t, size_t> taskOf;
            for (const uint32_t prim : _askQueue) {
                taskOf.emplace(prim, tasks.size());
                _Task &task = tasks.emplace_back();
                task.prim = prim;
                task.all = true;
            }
            for (const uint32_t prim : _waiting) {
                const auto [it, added] = taskOf.emplace(prim, tasks.size());
                if (added) tasks.emplace_back().prim = prim;
                tasks[it->second].attrs = std::move(_prims[prim].pendingAttrs);
                _prims[prim].pendingAttrs.clear();
            }
            _askQueue.clear();
            _waiting.clear();

            std::vector<_Read> reads(tasks.size());
            const auto readTasks = [&](size_t begin, size_t end) {
                for (size_t t = begin; t < end; ++t) {
                    const _Task &task = tasks[t];
                    _Read &read = reads[t];
                    const UsdPrim prim =
                        stage->GetPrimAtPath(_prims[task.prim].path);
                    if (prim && !_prims[task.prim].read) {
                        if (auto rel = prim.GetRelationship(TfToken("rigExec:poseInputs")))
                            rel.GetTargets(&read.frameInputs);
                        const TfToken type = prim.GetTypeName();
                        read.providerType = _IsFrameProviderType(type);
                        if (const UsdPrim parent =
                                _NamespaceFrameProvider(prim)) {
                            read.nsProvider = parent.GetPath();
                        }
                    }
                    if (task.all && prim) {
                        for (const UsdAttribute &attribute :
                             prim.GetAttributes()) {
                            read.listed.emplace_back(
                                attribute.GetPath(),
                                _AuthoredConnections(attribute));
                        }
                    }
                    read.named.reserve(task.attrs.size());
                    for (const uint32_t attr : task.attrs) {
                        const UsdAttribute attribute =
                            stage->GetAttributeAtPath(_attrs[attr].path);
                        read.named.emplace_back(
                            bool(attribute), _AuthoredConnections(attribute));
                    }
                }
            };
            // Only a round with enough reads in it goes to the pool. After
            // the first, which lists every seed prim whole, a round is the
            // handful of attributes the last one's connections named -- two
            // to forty cheap reads -- and the fork and join cost more than
            // the reads they spread (see _kPoseInfoParallelMinReads).
            size_t roundReads = 0;
            for (const _Task &task : tasks) {
                roundReads += (task.all ? _kPoseInfoReadsPerListedPrim : 0) +
                              task.attrs.size();
            }
            if (parallel && tasks.size() > 1 &&
                roundReads >= _kPoseInfoParallelMinReads) {
                WorkParallelForN(tasks.size(), readTasks);
            } else {
                readTasks(0, tasks.size());
            }

            size_t listed = 0;
            for (const _Read &read : reads) listed += read.listed.size();
            _Reserve(_attrs.size() + listed);
            for (size_t t = 0; t < tasks.size(); ++t) {
                const uint32_t prim = tasks[t].prim;
                _Read &read = reads[t];
                if (!_prims[prim].read) {
                    _prims[prim].read = true;
                    _prims[prim].providerType = read.providerType;
                    for (const SdfPath &input : read.frameInputs) {
                        const uint32_t provider = _PrimOf(input);
                        _prims[prim].frameInputs.push_back(provider);
                    }
                    if (!read.nsProvider.IsEmpty()) {
                        const uint32_t parent = _PrimOf(read.nsProvider);
                        _prims[prim].nsProvider = parent;
                    }
                }
                if (tasks[t].all) {
                    const uint32_t rootsBegin =
                        static_cast<uint32_t>(_roots.size());
                    for (auto &[path, sources] : read.listed) {
                        const uint32_t attr = _AttrOf(path, prim);
                        _MarkRead(attr, true, std::move(sources));
                        _roots.push_back(attr);
                    }
                    _prims[prim].rootsBegin = rootsBegin;
                    _prims[prim].rootsEnd =
                        static_cast<uint32_t>(_roots.size());
                    for (uint32_t r = rootsBegin; r < _prims[prim].rootsEnd;
                         ++r) {
                        _CreateNode(_roots[r], 0);
                    }
                    asked.push_back(prim);
                    if (!_prims[prim].providerType &&
                        _prims[prim].nsProvider != _None) {
                        _Ask(_prims[prim].nsProvider, skip);
                    }
                }
                for (size_t i = 0; i < tasks[t].attrs.size(); ++i) {
                    _MarkRead(tasks[t].attrs[i], read.named[i].first,
                              std::move(read.named[i].second));
                }
            }
            while (!_expand.empty()) {
                const uint32_t node = _expand.back();
                _expand.pop_back();
                _ExpandNode(node, skip);
            }
        }
    }

    {
        RIGEXEC_PROFILE_SCOPE_CAT(profiler, "PoseInfoPrefetch.Scc", "compile");
        _Tarjan();
        std::vector<uint32_t> sets;
        for (const uint32_t prim : asked) {
            _PoseInputInfo info;
            info.graphPrim = static_cast<int32_t>(prim);
            sets.clear();
            for (uint32_t r = _prims[prim].rootsBegin;
                 r < _prims[prim].rootsEnd; ++r) {
                const uint32_t comp = _nodeComp[2 * _roots[r]];
                info.connectedPose =
                    info.connectedPose || _compConnected[comp];
                if (_compProviders[comp] != 0) {
                    sets.push_back(_compProviders[comp]);
                }
            }
            std::sort(sets.begin(), sets.end());
            sets.erase(std::unique(sets.begin(), sets.end()), sets.end());
            for (const uint32_t set : sets) {
                for (const uint32_t provider : _providerSets[set]) {
                    info.providers.insert(_prims[provider].path);
                }
            }
            infos->emplace(_prims[prim].path, std::move(info));
        }
    }

    if (_PoseInfoVerifyRequested()) {
        std::vector<uint32_t> compMark(_compProviders.size(), _None);
        std::vector<uint32_t> attrMark(_attrs.size(), _None);
        std::vector<uint32_t> reached;
        uint32_t stamp = 0;
        for (const uint32_t prim : asked) {
            const SdfPath &path = _prims[prim].path;
            const _PoseInputInfo &info = infos->at(path);
            reached.clear();
            _Reach(prim, stamp++, &compMark, &attrMark, &reached);
            std::vector<SdfPath> attributes;
            for (const uint32_t attr : reached) {
                attributes.push_back(_attrs[attr].path);
            }
            std::sort(attributes.begin(), attributes.end());
            const _PoseInputInfo walked =
                _CollectPoseInputInfo(stage->GetPrimAtPath(path));
            const char *field =
                walked.providers != info.providers ? "providers"
                : walked.attributes != attributes  ? "attributes"
                : walked.connectedPose != info.connectedPose
                    ? "connectedPose"
                    : nullptr;
            if (field) {
                TF_FATAL_ERROR(
                    "RIGEXEC_VERIFY_POSEINFO: the pose-input closure of <%s> "
                    "differs from the per-prim walker's in %s (graph: %zu "
                    "providers, %zu attributes, connectedPose %d; walker: "
                    "%zu, %zu, %d)",
                    path.GetText(), field, info.providers.size(),
                    attributes.size(), int(info.connectedPose),
                    walked.providers.size(), walked.attributes.size(),
                    int(walked.connectedPose));
            }
        }
    }
}

void
_PoseInputGraph::MaterializeAttributes(
    std::map<SdfPath, _PoseInputInfo> *infos, bool parallel)
{
    std::vector<_PoseInputInfo *> pending;
    for (auto &[path, info] : *infos) {
        if (info.graphPrim >= 0) pending.push_back(&info);
    }
    if (pending.empty()) {
        return;
    }
    // A fixed number of blocks, each marking with arrays of its own: the
    // entries a block fills are distinct map values, so blocks share nothing
    // they write. Fixed because the marks span the whole graph, and a pool
    // left to split the range finely would allocate and clear them for every
    // few entries.
    constexpr size_t maxBlocks = 16;
    const size_t blocks =
        parallel ? std::min(pending.size(), maxBlocks) : size_t(1);
    const auto fillBlock = [&](size_t block) {
        const size_t begin = pending.size() * block / blocks;
        const size_t end = pending.size() * (block + 1) / blocks;
        std::vector<uint32_t> compMark(_compProviders.size(), _None);
        std::vector<uint32_t> attrMark(_attrs.size(), _None);
        std::vector<uint32_t> reached;
        for (size_t i = begin; i < end; ++i) {
            _PoseInputInfo &info = *pending[i];
            reached.clear();
            _Reach(static_cast<uint32_t>(info.graphPrim),
                   static_cast<uint32_t>(i), &compMark, &attrMark, &reached);
            info.attributes.reserve(reached.size());
            for (const uint32_t attr : reached) {
                info.attributes.push_back(_attrs[attr].path);
            }
            info.graphPrim = -1;
        }
    };
    if (blocks > 1) {
        WorkParallelForN(blocks, [&](size_t begin, size_t end) {
            for (size_t block = begin; block < end; ++block) fillBlock(block);
        });
    } else {
        fillBlock(0);
    }
}

// The solver-input index (_solverInputBatches): authored input prim -> the
// batches that read it, which is how the notice handler turns an edit into
// the batch.dirty and batch.cache resets the dynamic pose walk honours.
// A pure function of the stage and of four things the compile decided: which
// solvers each batch holds; the joint binding, where a bound joint stops the
// upward walk because its frame is the evaluator's and not the stage's; the
// solver DAG, where a relationship to another solver is a pose edge and not
// an authored input; and each solver's pose-provider closure with the
// attributes that closure read. That last pair is the one a later caller
// cannot get back off the stage without re-walking every closure, which is
// why the compile hands it over instead of this recomputing it -- and it has
// to be handed over, not dropped: a provider's attribute set names prims its
// own connection closure does not (a namespace parent's rest channels, the
// source of a connected default:space), and an edit to one of those has to
// reach the batch.
// Three passes, because only the middle one is stage work worth a pool: the
// walk that decides which prims a batch registers is path arithmetic and one
// relationship read per solver; each registered prim's connection closure is
// an independent read of the composed stage, as the pose-info prefetch's
// are; and the fill is serial, because the index is an ordered map (the
// handler range-scans it for a resynced subtree) and a std::map takes no
// concurrent inserts. The index is a set of facts, so neither the pool's
// order nor the fill's can change it.
std::map<SdfPath, std::set<size_t>>
_BuildSolverInputIndex(
    const UsdStageRefPtr &stage,
    const std::vector<std::vector<SdfPath>> &batchSolvers,
    const std::map<SdfPath, std::vector<std::pair<SdfPath, int>>> &jointBinding,
    const std::map<SdfPath, std::set<SdfPath>> &solverDependencies,
    const std::map<SdfPath, std::set<SdfPath>> &solverPoseReads,
    const std::map<SdfPath, _PoseInputInfo> &poseInputInfo)
{
    std::map<SdfPath, std::set<size_t>> index;
    if (!stage) {
        return index;
    }
    const auto isFrameProvider = [&stage](const SdfPath &path) {
        const UsdPrim prim = stage->GetPrimAtPath(path);
        return _IsFrameProvider(prim);
    };

    // The walk. A registered prim contributes itself and the prims of its
    // connection closure; a pose read contributes the prims of the attributes
    // its closure reached. Both are recorded as slots and resolved by the
    // pool, so a prim that many batches register is read once.
    std::unordered_map<SdfPath, size_t, SdfPath::Hash> primSlot;
    std::unordered_map<const std::vector<SdfPath> *, size_t> poseSlot;
    std::vector<SdfPath> primPaths;
    std::vector<const std::vector<SdfPath> *> poseAttributes;
    std::vector<std::pair<size_t, size_t>> registrations;      // batch, slot
    std::vector<std::pair<size_t, size_t>> poseRegistrations;  // batch, slot
    const auto slotOfPrim = [&](const SdfPath &path) {
        const auto [it, added] = primSlot.emplace(path, primPaths.size());
        if (added) primPaths.push_back(path);
        return it->second;
    };
    const auto slotOfPose = [&](const std::vector<SdfPath> *attributes) {
        const auto [it, added] =
            poseSlot.emplace(attributes, poseAttributes.size());
        if (added) poseAttributes.push_back(attributes);
        return it->second;
    };
    const auto registerInput = [&](size_t batch, const SdfPath &inputPath,
                                   bool followParents) {
        for (SdfPath path = inputPath.GetPrimPath();
             !path.IsEmpty() && path != SdfPath::AbsoluteRootPath();
             path = followParents ? path.GetParentPath() : SdfPath()) {
            registrations.emplace_back(batch, slotOfPrim(path));
            if (jointBinding.count(path)) break;
        }
    };
    for (size_t batch = 0; batch < batchSolvers.size(); ++batch) {
        for (const SdfPath &solver : batchSolvers[batch]) {
            registerInput(batch, solver, false);
            const auto reads = solverPoseReads.find(solver);
            if (reads != solverPoseReads.end()) {
                for (const SdfPath &provider : reads->second) {
                    registerInput(batch, provider, false);
                    const auto info = poseInputInfo.find(provider);
                    if (info != poseInputInfo.end() &&
                        !info->second.attributes.empty()) {
                        poseRegistrations.emplace_back(
                            batch, slotOfPose(&info->second.attributes));
                    }
                }
            }
            const UsdPrim prim = stage->GetPrimAtPath(solver);
            if (!prim) continue;
            for (const UsdRelationship &rel : prim.GetRelationships()) {
                SdfPathVector targets;
                rel.GetTargets(&targets);
                for (const SdfPath &target : targets) {
                    if (solverDependencies.count(target.GetPrimPath())) continue;
                    registerInput(batch, target,
                                  isFrameProvider(target.GetPrimPath()));
                }
            }
        }
    }

    // The reads, each slot reduced to the distinct prims it names, so the
    // serial fill below inserts a (prim, batch) fact once per slot rather
    // than once per attribute.
    std::vector<std::vector<SdfPath>> primsOf(primPaths.size());
    std::vector<std::vector<SdfPath>> posePrimsOf(poseAttributes.size());
    const auto distinctPrims = [](std::vector<SdfPath> *prims) {
        std::sort(prims->begin(), prims->end());
        prims->erase(std::unique(prims->begin(), prims->end()), prims->end());
    };
    const auto readSlots = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            if (i < primPaths.size()) {
                std::vector<SdfPath> &prims = primsOf[i];
                prims.push_back(primPaths[i]);
                for (const SdfPath &attribute :
                     _CollectAttributeConnectionInputs(
                         stage->GetPrimAtPath(primPaths[i]))) {
                    prims.push_back(attribute.GetPrimPath());
                }
                distinctPrims(&prims);
            } else {
                const size_t pose = i - primPaths.size();
                std::vector<SdfPath> &prims = posePrimsOf[pose];
                for (const SdfPath &attribute : *poseAttributes[pose]) {
                    prims.push_back(attribute.GetPrimPath());
                }
                distinctPrims(&prims);
            }
        }
    };
    const size_t slots = primPaths.size() + poseAttributes.size();
    if (RigExecParallelEvaluationEnabled() && slots > 1) {
        WorkParallelForN(slots, readSlots);
    } else {
        readSlots(0, slots);
    }

    for (const auto &[batch, slot] : registrations) {
        for (const SdfPath &prim : primsOf[slot]) index[prim].insert(batch);
    }
    for (const auto &[batch, slot] : poseRegistrations) {
        for (const SdfPath &prim : posePrimsOf[slot]) index[prim].insert(batch);
    }
    return index;
}

} // namespace evaluatorDetail

} // namespace rigExec
