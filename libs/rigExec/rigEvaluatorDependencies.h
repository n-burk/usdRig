// Private evaluator implementation types and helpers.
#ifndef RIGEXEC_RIG_EVALUATOR_DEPENDENCIES_H
#define RIGEXEC_RIG_EVALUATOR_DEPENDENCIES_H

#include "rigEvaluatorInternal.h"
#include "pxr/base/tf/pxrTslRobinMap/robin_map.h"
#include <functional>
#include <limits>

namespace rigExec {

namespace evaluatorDetail {

std::vector<SdfPath>
_DiscoverJointOutputs(const UsdStageRefPtr &stage, const SdfPath &rigPath,
                      const std::map<SdfPath, std::string> &skipped = {});

std::vector<SdfPath>
_DiscoverPoseInterpolators(const UsdStageRefPtr &stage, const SdfPath &rigPath);

std::vector<SdfPath>
_DiscoverControls(const UsdStageRefPtr &stage, const SdfPath &rigPath);

std::vector<SdfPath>
_DiscoverVolumeWeights(const UsdStageRefPtr &stage, const SdfPath &rigPath);

bool
_IsAggregateSolverType(const TfToken &typeName);

std::vector<UsdPrim>
_DiscoverAggregateSolvers(
    const UsdStageRefPtr &stage, const SdfPath &rigPath);

using _PathDependencies = std::map<SdfPath, std::set<SdfPath>>;

struct _DependencyOrder {
    SdfPathVector ordered;
    SdfPathVector blocked;  // Cycles and anything waiting on them.
};

_DependencyOrder
_OrderDependencies(const _PathDependencies &dependencies);

// Matrix expressions can expose a posed ancestor through a connection to
// another provider's parent:space, including through default-space fallbacks.
// Trace the registered inputs conservatively: value-only identity toggles do
// not change this graph. Only an evaluator-owned joint override cuts it.
struct _PoseInputInfo {
    std::set<SdfPath> providers;
    // Every attribute path the closure visited, whether it exists or not,
    // each once and in no particular order. Only the solver-input index
    // reads these, for the distinct prims they name, so a closure the
    // pose-input graph computed leaves them to
    // _PoseInputGraph::MaterializeAttributes.
    std::vector<SdfPath> attributes;
    bool connectedPose = false;
    // The _PoseInputGraph prim whose attributes are still to be
    // materialized, or -1 once `attributes` is complete.
    int32_t graphPrim = -1;
};

UsdPrim
_NamespaceFrameProvider(UsdPrim prim);

bool
_ValidateAdjustmentPoseConsumers(const UsdStageRefPtr &stage,
    const UsdPrim &rig, std::string *error, SdfPath *operation,
    const std::map<SdfPath, std::string> &skipped = {});

_PoseInputInfo
_CollectPoseInputInfo(const UsdPrim &prim);

// Shared pose-input closure graph. Stage reads discover nodes in parallel;
// a serial Tarjan pass folds reachability over strongly connected components.
// Attribute sets are materialized only when the solver-input index needs them.
class _PoseInputGraph
{
public:
    // Asks for the closures of `seeds` -- except an empty path and one that
    // `skip` names, which the closure walks do not read either -- and of
    // every prim they ask for in turn. Adds one entry to `*infos` per prim
    // newly asked for: its providers and connectedPose, with attributes left
    // to MaterializeAttributes. A prim already asked for is not asked again.
    void Extend(const UsdStageRefPtr &stage,
                const std::vector<SdfPath> &seeds,
                const std::function<bool(const SdfPath &)> &skip,
                bool parallel, RigExecProfiler &profiler,
                std::unordered_map<SdfPath, _PoseInputInfo, SdfPath::Hash>
                    *infos);

    // Fills the attributes of every entry that still names a prim of this
    // graph, and clears that name. Entries it did not compute are left as
    // they are.
    void MaterializeAttributes(std::map<SdfPath, _PoseInputInfo> *infos,
                               bool parallel);

private:
    static constexpr uint32_t _None = std::numeric_limits<uint32_t>::max();

    struct _Prim {
        SdfPath path;
        bool read = false;
        bool asked = false;         // asked for, or skipped, already
        bool providerType = false;  // a joint, control or volume weight
        uint32_t nsProvider = _None;
        std::vector<uint32_t> frameInputs;
        uint32_t rootsBegin = 0;    // its attributes, in _roots
        uint32_t rootsEnd = 0;
        std::vector<uint32_t> pendingAttrs;  // reached, not yet read
    };
    struct _Attr {
        SdfPath path;
        uint32_t prim = _None;  // owner; _None when the path names no
                                // prim property, which no stage resolves
        bool read = false;
        bool exists = false;
        bool pending = false;
        uint8_t created = 0;    // bit c: node (this, c) exists
        SdfPathVector sources;
    };
    // One task of phase A.
    struct _Task {
        uint32_t prim = _None;
        bool all = false;
        std::vector<uint32_t> attrs;
    };
    struct _Read {
        bool providerType = false;
        SdfPath nsProvider;
        SdfPathVector frameInputs;
        std::vector<std::pair<SdfPath, SdfPathVector>> listed;
        std::vector<std::pair<bool, SdfPathVector>> named;
    };

    uint32_t _PrimOf(const SdfPath &path);
    uint32_t _AttrOf(const SdfPath &path);
    // _AttrOf for a path already known to be a property of `prim`.
    uint32_t _AttrOf(const SdfPath &path, uint32_t prim);
    void _PushAttr(_Attr &&attr);
    void _Reserve(size_t attrs);
    void _Ask(uint32_t prim, const std::function<bool(const SdfPath &)> &skip);
    void _CreateNode(uint32_t attr, uint32_t connected);
    void _MarkRead(uint32_t attr, bool exists, SdfPathVector &&sources);
    void _ExpandNode(uint32_t node,
                     const std::function<bool(const SdfPath &)> &skip);
    void _Tarjan();
    // Appends the attributes reachable from `prim`'s own attributes to
    // `*out`, each once; `compMark` and `attrMark` hold `stamp` for what is
    // visited.
    void _Reach(uint32_t prim, uint32_t stamp, std::vector<uint32_t> *compMark,
                std::vector<uint32_t> *attrMark,
                std::vector<uint32_t> *out) const;

    std::vector<_Prim> _prims;
    pxr_tsl::robin_map<SdfPath, uint32_t, SdfPath::Hash> _primIndex;
    std::vector<_Attr> _attrs;
    pxr_tsl::robin_map<SdfPath, uint32_t, SdfPath::Hash> _attrIndex;
    std::vector<uint32_t> _roots;
    std::vector<uint32_t> _askQueue;  // asked for, not yet read
    std::vector<uint32_t> _waiting;   // prims with pendingAttrs
    std::vector<uint32_t> _expand;    // nodes read but not yet expanded

    // Per node, 2 * attribute + connected.
    std::vector<uint32_t> _edgeBegin;
    std::vector<uint32_t> _edgeCount;
    std::vector<uint32_t> _nodeProvider;  // a parent:space node's provider
    std::vector<uint32_t> _nodeComp;
    std::vector<uint32_t> _edges;

    // Per component, in completion order.
    std::vector<uint32_t> _compNodesBegin{0};
    std::vector<uint32_t> _compNodes;
    std::vector<uint32_t> _compSuccsBegin{0};
    std::vector<uint32_t> _compSuccs;
    std::vector<uint32_t> _compProviders;  // into _providerSets
    std::vector<uint8_t> _compConnected;
    std::vector<uint32_t> _compMark;
    // Sorted prim indices; [0] is the empty set every provider-less
    // component shares.
    std::vector<std::vector<uint32_t>> _providerSets{{}};
};

std::map<SdfPath, std::set<size_t>>
_BuildSolverInputIndex(
    const UsdStageRefPtr &stage,
    const std::vector<std::vector<SdfPath>> &batchSolvers,
    const std::map<SdfPath, std::vector<std::pair<SdfPath, int>>> &jointBinding,
    const std::map<SdfPath, std::set<SdfPath>> &solverDependencies,
    const std::map<SdfPath, std::set<SdfPath>> &solverPoseReads,
    const std::map<SdfPath, _PoseInputInfo> &poseInputInfo);

} // namespace evaluatorDetail

// Deferred inputs retained until dynamic request preparation needs the index.
struct RigExecRigEvaluator::_SolverInputIndexInputs {
    std::map<SdfPath, std::set<SdfPath>> solverPoseReads;
    // Attribute sets not yet materialized: the graph below fills them.
    std::map<SdfPath, evaluatorDetail::_PoseInputInfo> poseInputInfo;
    std::shared_ptr<evaluatorDetail::_PoseInputGraph> poseInputGraph;
};

} // namespace rigExec

#endif
