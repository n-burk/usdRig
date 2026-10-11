#ifndef RIGEXEC_GRAPH_TYPED_VALUES_H
#define RIGEXEC_GRAPH_TYPED_VALUES_H
#include "rigExecGraph/opGraph.h"
#include "rigExecMath/pointFrame.h"
#include "pxr/base/vt/value.h"
#include <cstring>
#include <variant>

namespace rigExec {
using RigExecTypedValue = std::variant<std::monostate, double, float,
    GfVec3d, GfMatrix4d, TfToken, RigExecPointFrame, VtValue>;
struct RigExecTypedValueState {
    RigExecTypedValue value;
    /// Retained source boxing lets pure xform arithmetic inspect exact USD
    /// numeric precision without boxing values during computation.
    VtValue raw;
    bool initialized = false, blocked = false, authoritative = false;
    bool changed = false;
    uint64_t revision = 0;
    size_t count = 0;
    std::string error;
};
inline bool RigExecTypedSame(const GfMatrix4d &a, const GfMatrix4d &b) {
    for (int r=0;r<4;++r) for (int c=0;c<4;++c)
        if (std::memcmp(&a[r][c],&b[r][c],sizeof(double))) return false;
    return true;
}
inline bool RigExecTypedSame(const GfVec3d &a, const GfVec3d &b) {
    for (int i=0;i<3;++i) if (std::memcmp(&a[i],&b[i],sizeof(double))) return false;
    return true;
}
inline bool RigExecTypedSame(const RigExecPointFrame &a, const RigExecPointFrame &b) {
    if (a.flags != b.flags) return false;
    for (size_t i=0;i<4;++i) if (!RigExecTypedSame(a.points[i],b.points[i])) return false;
    return true;
}
inline bool RigExecTypedSame(double a, double b) { return !std::memcmp(&a,&b,sizeof(a)); }
inline bool RigExecTypedSame(float a, float b) { return !std::memcmp(&a,&b,sizeof(a)); }
/// Boxed numeric values retain type, cardinality, signed zero and NaN bits.
/// Unsupported opaque values have no equality proof and propagate change.
bool RigExecTypedSame(const VtValue &a,const VtValue &b);
template<class T> inline bool RigExecTypedSame(const T &a,const T &b) { return a == b; }

/// Storage is sized once from the canonical compiler layout. Distinct ops
/// publish distinct slots; no map mutation or source access occurs in Run.
class RigExecTypedValueStore {
public:
    explicit RigExecTypedValueStore(size_t count=0) : values(count) {}
    std::vector<RigExecTypedValueState> values;
    void ResetChanges() { for (auto &v:values) v.changed=false; }
    template<class T> const T *Read(RigExecValueId id) const {
        return id < values.size() && values[size_t(id)].initialized &&
            !values[size_t(id)].blocked ?
            std::get_if<T>(&values[size_t(id)].value) : nullptr;
    }
    template<class T> bool Publish(RigExecValueId id,const T &value,
        bool blocked=false,bool authoritative=false,size_t count=1,
        const std::string &error={}) {
        auto &state=values.at(size_t(id));
        const auto *old=std::get_if<T>(&state.value);
        state.changed=!state.initialized || !old || !RigExecTypedSame(*old,value) ||
            state.blocked!=blocked || state.authoritative!=authoritative ||
            state.count!=count || state.error!=error;
        if (state.changed) { state.value=value; ++state.revision; }
        state.initialized=true; state.blocked=blocked; state.authoritative=authoritative;
        state.count=count; state.error=error;
        return state.changed;
    }
    bool PublishSource(RigExecValueId id,const VtValue &value,
        bool blocked=false,bool authoritative=false,const std::string &error={});
    bool Copy(RigExecValueId to,RigExecValueId from,bool authoritative=false);
};
}
#endif
