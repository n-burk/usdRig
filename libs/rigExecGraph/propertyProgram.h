#ifndef RIGEXEC_GRAPH_PROPERTY_PROGRAM_H
#define RIGEXEC_GRAPH_PROPERTY_PROGRAM_H
#include "rigExecMath/propertyMath.h"
#include <vector>
namespace rigExec {
struct RigExecFloatPropertyRecord {
    RigExecPropertyOp op=RigExecPropertyOp::Add;
    bool opValid=true,hasTangents=false;
    float value=0,minimum=0,maximum=0;
    std::vector<GfVec2f> keys,tangents;
    /// Optional current input views. Null uses retained vectors; borrowed
    /// arrays remain alive through Run and are never retained by the runner.
    const GfVec2f *keyData=nullptr,*tangentData=nullptr;
    size_t keyCount=0,tangentCount=0;
};
struct RigExecVec3PropertyRecord {
    RigExecPropertyOp op=RigExecPropertyOp::Add;
    bool opValid=true;
    GfVec3f value{0},minimum{0},maximum{0};
};
struct RigExecMatrixPropertyRecord {
    RigExecPropertyOp op=RigExecPropertyOp::Multiply;
    bool opValid=true;
    GfMatrix4d value=GfMatrix4d(1.0);
};
/// These guards and arithmetic are shared by native and SceneDb revisions.
/// False leaves result untouched; the domain publishes an exact pass-through.
bool RigExecRunProperty(const RigExecFloatPropertyRecord &,float base,float envelope,float *result);
bool RigExecRunProperty(const RigExecFloatPropertyRecord &,double base,float envelope,double *result);
bool RigExecRunProperty(const RigExecVec3PropertyRecord &,const GfVec3f &base,float envelope,GfVec3f *result);
bool RigExecRunProperty(const RigExecMatrixPropertyRecord &,const GfMatrix4d &base,float envelope,GfMatrix4d *result);
}
#endif
