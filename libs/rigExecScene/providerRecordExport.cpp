#include "pxr/base/gf/vec3i.h"
#include "providerRecordExport.h"
#include <type_traits>
#include "pxr/base/gf/vec2f.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/vt/array.h"
namespace rigExec {
namespace {
bool Boxed(const VtValue &value,RigExecProviderPlainValue *output) {
    if(value.IsEmpty()) {*output=std::monostate();return true;}
    if(value.IsHolding<bool>()) {*output=value.UncheckedGet<bool>();return true;}
    if(value.IsHolding<int>()) {*output=int32_t(value.UncheckedGet<int>());return true;}
    if(value.IsHolding<GfVec3i>()) {
        const auto &v=value.UncheckedGet<GfVec3i>();*output=std::array<int32_t,3>{int32_t(v[0]),int32_t(v[1]),int32_t(v[2])};return true;
    }
    if(value.IsHolding<GfVec3f>()) {
        const auto &v=value.UncheckedGet<GfVec3f>();*output=std::array<float,3>{v[0],v[1],v[2]};return true;
    }
    if(value.IsHolding<GfVec2f>()) {
        const auto &v=value.UncheckedGet<GfVec2f>();*output=std::array<float,2>{v[0],v[1]};return true;
    }
    if(value.IsHolding<VtFloatArray>()) {const auto &v=value.UncheckedGet<VtFloatArray>();*output=std::vector<float>(v.begin(),v.end());return true;}
    if(value.IsHolding<VtDoubleArray>()) {const auto &v=value.UncheckedGet<VtDoubleArray>();*output=std::vector<double>(v.begin(),v.end());return true;}
    if(value.IsHolding<VtIntArray>()) {const auto &v=value.UncheckedGet<VtIntArray>();*output=std::vector<int32_t>(v.begin(),v.end());return true;}
    if(value.IsHolding<VtBoolArray>()) {const auto &v=value.UncheckedGet<VtBoolArray>();*output=std::vector<bool>(v.begin(),v.end());return true;}
    if(value.IsHolding<VtVec3fArray>()) {
        std::vector<std::array<float,3>> result;
        for(const auto &v:value.UncheckedGet<VtVec3fArray>())result.push_back({v[0],v[1],v[2]});
        *output=std::move(result);return true;
    }
    if(value.IsHolding<VtVec3dArray>()) {
        std::vector<std::array<double,3>> result;
        for(const auto &v:value.UncheckedGet<VtVec3dArray>())result.push_back({v[0],v[1],v[2]});
        *output=std::move(result);return true;
    }
    if(value.IsHolding<VtVec2fArray>()) {
        std::vector<std::array<float,2>> result;
        for(const auto &v:value.UncheckedGet<VtVec2fArray>())result.push_back({v[0],v[1]});
        *output=std::move(result);return true;
    }
    if(value.IsHolding<VtMatrix4dArray>()) {
        std::vector<std::array<double,16>> result;
        for(const auto &v:value.UncheckedGet<VtMatrix4dArray>()) {
            std::array<double,16> matrix;
            for(int r=0;r<4;++r)for(int c=0;c<4;++c)matrix[size_t(r*4+c)]=v[r][c];
            result.push_back(matrix);
        }
        *output=std::move(result);return true;
    }
    if(value.IsHolding<VtTokenArray>()) {
        std::vector<std::string> result;
        for(const auto &v:value.UncheckedGet<VtTokenArray>())result.push_back(v.GetString());
        *output=std::move(result);return true;
    }
    return false;
}
}
bool RigExecExportProviderRecords(const RigExecProviderProgram &program,
    const RigExecTypedValueStore &store,RigExecProviderPlainProgram *output,std::string *error) {
    const auto fail=[&](const std::string &message){if(error)*error=message;return false;};
    if(!output || store.values.size()!=program.valueKeys.size())return fail("provider export store/layout mismatch");
    RigExecProviderPlainProgram records;
    records.valueKeys=program.valueKeys;records.leaves=program.leaves;
    records.defaults.resize(store.values.size());
    for(size_t i=0;i<store.values.size();++i) {
        const auto &source=store.values[i];auto &target=records.defaults[i];
        target.initialized=source.initialized;target.blocked=source.blocked;target.authoritative=source.authoritative;
        target.revision=source.revision;target.count=source.count;target.error=source.error;
        const bool valid=std::visit([&](const auto &value) {
            using T=std::decay_t<decltype(value)>;
            if constexpr(std::is_same_v<T,GfMatrix4d>) {
                std::array<double,16> matrix;
                for(int r=0;r<4;++r)for(int c=0;c<4;++c)matrix[size_t(r*4+c)]=value[r][c];
                target.value=matrix;
            } else if constexpr(std::is_same_v<T,GfVec3d>) {
                target.value=std::array<double,3>{value[0],value[1],value[2]};
            } else if constexpr(std::is_same_v<T,TfToken>) {
                target.value=value.GetString();
            } else if constexpr(std::is_same_v<T,RigExecPointFrame>) {
                RigExecProviderPlainFrame frame;
                frame.flags=value.flags;
                for(size_t p=0;p<4;++p)for(size_t a=0;a<3;++a)frame.points[p][a]=value.points[p][int(a)];
                target.value=frame;
            } else if constexpr(std::is_same_v<T,VtValue>) {
                return Boxed(value,&target.value);
            } else target.value=value;
            return true;
        },source.value);
        if(!valid)return fail("provider export has an unsupported typed raw value: "+program.valueKeys[i]);
    }
    for(const auto &op:program.ops) {
        if(op.kind==RigExecProviderOpKind::LocalXform || op.kind==RigExecProviderOpKind::InterveningXform)
            return fail("provider wire export requires intervening transforms lowered to external matrix leaves");
        records.ops.push_back({op.kind,op.owner.GetString(),op.output,op.inputs,op.scaleAvars,
            op.affineKind,op.affineTargets});
    }
    for(const auto &leaf:program.sampled)records.sampled.push_back({leaf.value,leaf.attribute.GetString(),{}});
    for(const auto &leaf:program.externalInputs)records.externalInputs.push_back({leaf.value,leaf.owner.GetString(),leaf.computation});
    for(const auto &leaf:program.routedInputs)records.routedInputs.push_back({leaf.value,leaf.consumer.GetString(),
        leaf.source.GetString(),leaf.readPhase.GetString()});
    *output=std::move(records);return true;
}
}
