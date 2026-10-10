#ifndef RIGEXEC_STANDALONE_SYSTEM_H
#define RIGEXEC_STANDALONE_SYSTEM_H
#include "sceneDb.h"
#include "rigExec/tapSet.h"
#include <memory>

namespace rigExec {
struct RigExecStandaloneResolvedState {
    VtValue value;
    bool blocked=false;
};
struct RigExecStandaloneResult {
    /// True means every requested value was extracted without execution errors.
    /// Typed packet validity/degeneracy flags retain their own semantic meaning.
    bool valid = false;
    uint64_t generation = 0;
    UsdTimeCode time = UsdTimeCode::Default();
    std::vector<VtValue> values;
    std::vector<std::string> diagnostics;
    const VtValue &Get(int tap) const {
        static const VtValue empty;
        return tap >= 0 && size_t(tap) < values.size() ? values[tap] : empty;
    }
    template<class T> T Get(int tap) const {
        const VtValue &value = Get(tap);
        return value.IsHolding<T>() ? value.UncheckedGet<T>() : T();
    }
};

/// Shared production scene-graph execution over an owned compact database.
/// The host serializes calls; no USD stage is created or retained here.
/// Solver, pose, property, weight and geometry producers share typed IDs and
/// the same compiler/executor as detached scene playback.
class RigExecStandaloneSystem {
public:
    explicit RigExecStandaloneSystem(const RigExecSceneDb &database);
    ~RigExecStandaloneSystem();
    RigExecStandaloneSystem(const RigExecStandaloneSystem &) = delete;
    RigExecStandaloneSystem &operator=(const RigExecStandaloneSystem &) = delete;

    int AddTap(const RigExecValueAddress &address);
    bool Prepare(std::string *error = nullptr);
    RigExecStandaloneResult Evaluate(UsdTimeCode time);
    /// Supply every attribute's already-resolved value/block for an arbitrary
    /// identity. Missing slots fail; no row is borrowed from an exported time.
    /// Empty boxes preserve an existing identity's block bit; new identities
    /// need the typed overload below to distinguish blocks from no value.
    RigExecStandaloneResult EvaluateResolved(
        UsdTimeCode time, const std::map<SdfPath, VtValue> &resolvedStates);

    /// Explicit block/no-value provenance for an arbitrary identity. A block
    /// requires an empty value; captured structural Default facts must match.
    RigExecStandaloneResult EvaluateResolved(UsdTimeCode time,
        const std::map<SdfPath,RigExecStandaloneResolvedState> &resolvedStates);

    bool SetValue(const SdfPath &attribute, UsdTimeCode time,
                  const VtValue &value, std::string *error = nullptr);
    bool SetConnections(const SdfPath &attribute, const SdfPathVector &sources,
                        std::string *error = nullptr);
    bool SetTargets(const SdfPath &relationship, const SdfPathVector &targets,
                    std::string *error = nullptr);
    bool SetPrimActive(const SdfPath &prim, bool active, std::string *error = nullptr);
    /// Actual compiled scene artifact identity: retained for numeric row edits,
    /// replaced after structural declarations change.
    const void *GetCompilerIdentity() const;
    /// Requested outputs reached by changed sampled IDs in the compiled graph.
    size_t GetValueInvalidationCount() const;
private:
    struct _Impl;
    std::unique_ptr<_Impl> _impl;
};
} // namespace rigExec
#endif
