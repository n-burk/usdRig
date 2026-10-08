#include "sceneGraphBinding.h"
#include "rigExec/frameExtraction.h"
#include <type_traits>
namespace rigExec {
bool RigExecReadSceneGraphValue(const RigExecTypedValueStore &store,RigExecValueId id,
    VtValue *output,bool frameAsMatrix) {
    if(!output)return false;*output=VtValue();
    if(id>=store.values.size())return false;
    const auto &state=store.values[size_t(id)];
    if(!state.initialized || state.blocked)return false;
    return std::visit([&](const auto &value) {
        using T=std::decay_t<decltype(value)>;
        if constexpr(std::is_same_v<T,std::monostate>)return false;
        else if constexpr(std::is_same_v<T,VtValue>) { *output=value;return !value.IsEmpty(); }
        else if constexpr(std::is_same_v<T,RigExecPointFrame>) {
            if(frameAsMatrix) {
                if(!value.IsValid() || value.IsDegenerate())return false;
                GfMatrix4d matrix(1.0);
                if(!RigExecPointsToMatrix(RigExecIdentityLandmarks(),value,&matrix))return false;
                *output=VtValue(matrix);
            } else *output=VtValue(value);
            return true;
        } else { *output=VtValue(value);return true; }
    },state.value);
}
}
