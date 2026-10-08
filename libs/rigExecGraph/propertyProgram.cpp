#include "propertyProgram.h"
#include <cmath>
namespace rigExec {
bool RigExecRunProperty(const RigExecFloatPropertyRecord &record,float base,float envelope,float *result) {
    if(!result || !record.opValid || !std::isfinite(record.value) ||
       !std::isfinite(record.minimum) || !std::isfinite(record.maximum))return false;
    RigExecPropertyMathParams<float> params;
    params.op=record.op;params.value=record.value;params.min=record.minimum;params.max=record.maximum;
    if(record.op==RigExecPropertyOp::Curve) {
        params.keys=record.keyData?record.keyData:record.keys.data();
        params.keyCount=record.keyData?record.keyCount:record.keys.size();
        if(!params.keyCount || !RigExecValidateLinearKeys(params.keys,params.keyCount))return false;
        const auto *tangents=record.tangentData?record.tangentData:record.tangents.data();
        const size_t count=record.tangentData?record.tangentCount:record.tangents.size();
        if(record.hasTangents && count) {
            if(count!=params.keyCount)return false;
            params.tangents=tangents;params.tangentCount=count;
        }
    }
    params.weight=envelope;*result=RigExecApplyFloatMath(base,params);return true;
}
bool RigExecRunProperty(const RigExecFloatPropertyRecord &record,double base,float envelope,double *result) {
    if(!result)return false;
    float narrowed=0;
    if(!RigExecRunProperty(record,float(base),envelope,&narrowed))return false;
    *result=double(narrowed);return true;
}
bool RigExecRunProperty(const RigExecVec3PropertyRecord &record,const GfVec3f &base,float envelope,GfVec3f *result) {
    if(!result || !record.opValid)return false;
    for(size_t i=0;i<3;++i)if(!std::isfinite(record.value[i]) || !std::isfinite(record.minimum[i]) ||
        !std::isfinite(record.maximum[i]))return false;
    RigExecPropertyMathParams<GfVec3f> params;
    params.op=record.op;params.value=record.value;params.min=record.minimum;params.max=record.maximum;
    params.weight=envelope;*result=RigExecApplyVec3fMath(base,params);return true;
}
bool RigExecRunProperty(const RigExecMatrixPropertyRecord &record,const GfMatrix4d &base,float envelope,GfMatrix4d *result) {
    if(!result || !record.opValid)return false;
    for(size_t r=0;r<4;++r)for(size_t c=0;c<4;++c)if(!std::isfinite(record.value[r][c]))return false;
    return RigExecApplyMatrixMath(base,record.op,record.value,envelope,result);
}
}
