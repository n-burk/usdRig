#include "typedValues.h"
#include "rigExec/bakedOpValues.h"
#include "rigExec/weightPackets.h"
namespace rigExec {
namespace {
template<class Array> bool SameArray(const Array &a,const Array &b) {
    if(a.size()!=b.size())return false;
    for(size_t i=0;i<a.size();++i)if(!RigExecTypedSame(a[i],b[i]))return false;
    return true;
}
}
bool RigExecTypedSame(const VtValue &a,const VtValue &b) {
    if(a.IsHolding<RigExecPointFrame>()) {
        return b.IsHolding<RigExecPointFrame>() &&
            RigExecTypedSame(a.UncheckedGet<RigExecPointFrame>(),
                             b.UncheckedGet<RigExecPointFrame>());
    }
    if(a.IsHolding<RigExecPointFrameArray>()) {
        if(!b.IsHolding<RigExecPointFrameArray>())return false;
        const auto &x=a.UncheckedGet<RigExecPointFrameArray>();
        const auto &y=b.UncheckedGet<RigExecPointFrameArray>();
        if(!SameArray(x.frames,y.frames) || x.rests.size()!=y.rests.size())return false;
        for(size_t i=0;i<x.rests.size();++i)for(size_t p=0;p<4;++p)
            if(!RigExecTypedSame(x.rests[i][p],y.rests[i][p]))return false;
        return true;
    }
    if(a.IsHolding<RigExecWeightPacket>()) {
        if(!b.IsHolding<RigExecWeightPacket>())return false;
        const auto &x=a.UncheckedGet<RigExecWeightPacket>();
        const auto &y=b.UncheckedGet<RigExecWeightPacket>();
        return x.valid==y.valid && x.representation==y.representation &&
            x.rangePolicy==y.rangePolicy && RigExecTypedSame(x.defaultWeight,y.defaultWeight) &&
            SameArray(x.values,y.values) && x.indices==y.indices;
    }
    if(a.IsHolding<RigExecFalloffLut>())return b.IsHolding<RigExecFalloffLut>() &&
        SameArray(a.UncheckedGet<RigExecFalloffLut>().samples,b.UncheckedGet<RigExecFalloffLut>().samples);
    return RigExecExactSourceValueEqual(a,b);
}
bool RigExecTypedValueStore::PublishSource(RigExecValueId id,const VtValue &value,
    bool blocked,bool authoritative,const std::string &error) {
    values.at(size_t(id)).raw=value;
    if (value.IsEmpty()) return Publish(id,std::monostate(),blocked,authoritative,0,error);
    if (value.IsHolding<double>()) return Publish(id,value.UncheckedGet<double>(),blocked,authoritative,1,error);
    if (value.IsHolding<float>()) return Publish(id,value.UncheckedGet<float>(),blocked,authoritative,1,error);
    if (value.IsHolding<GfVec3d>()) return Publish(id,value.UncheckedGet<GfVec3d>(),blocked,authoritative,1,error);
    if (value.IsHolding<GfMatrix4d>()) return Publish(id,value.UncheckedGet<GfMatrix4d>(),blocked,authoritative,1,error);
    if (value.IsHolding<TfToken>()) return Publish(id,value.UncheckedGet<TfToken>(),blocked,authoritative,1,error);
    if (value.IsHolding<RigExecPointFrame>()) return Publish(id,value.UncheckedGet<RigExecPointFrame>(),blocked,authoritative,1,error);
    return Publish(id,value,blocked,authoritative,value.IsArrayValued()?value.GetArraySize():1,error);
}
bool RigExecTypedValueStore::Copy(RigExecValueId to,RigExecValueId from,bool authoritative) {
    const auto &source=values.at(size_t(from));
    const bool result=std::visit([&](const auto &value) {
        return Publish(to,value,source.blocked,authoritative || source.authoritative,source.count,source.error);
    },source.value);
    values.at(size_t(to)).raw=source.raw;
    return result;
}
}
