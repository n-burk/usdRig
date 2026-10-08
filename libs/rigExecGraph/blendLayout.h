#ifndef RIGEXEC_GRAPH_BLEND_LAYOUT_H
#define RIGEXEC_GRAPH_BLEND_LAYOUT_H
#include <cmath>
#include <cstddef>
#include <cstring>
namespace rigExec {
// Original sparse layout validation/order, shared by native and USD-free runtime.
template<class Offsets,class Indices,class Layout>
bool RigExecBuildBlendLayout(const Offsets &offsets,const Indices &indices,
                            size_t pointCount,Layout *layout) {
    if(!layout)return false;
    layout->pointCount=pointCount;layout->valid=false;
    if(!indices.empty()) {
        if(indices.size()!=offsets.size())return false;
        for(int index:indices)if(index<0 || size_t(index)>=pointCount)return false;
    } else if(!offsets.empty() && offsets.size()!=pointCount)return false;
    for(const auto &offset:offsets)for(int axis=0;axis<3;++axis)
        if(!std::isfinite(offset[axis]))return false;
    layout->offsets.assign(offsets.begin(),offsets.end());
    layout->indices.assign(indices.begin(),indices.end());layout->valid=true;return true;
}
template<class Layout>
bool RigExecSameBlendLayout(const Layout &a,const Layout &b) {
    if(a.pointCount!=b.pointCount || a.valid!=b.valid || a.indices!=b.indices ||
       a.offsets.size()!=b.offsets.size())return false;
    for(size_t i=0;i<a.offsets.size();++i)for(int axis=0;axis<3;++axis) {
        const float x=a.offsets[i][axis],y=b.offsets[i][axis];
        if(std::memcmp(&x,&y,sizeof(float)))return false;
    }
    return true;
}
}
#endif
