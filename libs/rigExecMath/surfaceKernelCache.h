#ifndef RIGEXEC_MATH_SURFACE_KERNEL_CACHE_H
#define RIGEXEC_MATH_SURFACE_KERNEL_CACHE_H
#include "deltaMushKernel.h"
#include "spatialAccel.h"
#include <cstring>
#include <memory>
namespace rigExec {
/// One revision owns and mutates this cache. Clones share only immutable
/// entries; no global state or locking. Keys compare every scalar's raw bits,
/// including signed zero/NaN payloads, and every topology element.
template<class Point,class Wide> class RigExecSurfaceKernelCache {
    static bool Same(const std::vector<Point> &a,const std::vector<Point> &b) {
        if(a.size()!=b.size()) return false;
        for(size_t i=0;i<a.size();++i) for(int axis=0;axis<3;++axis) {
            const auto x=a[i][axis], y=b[i][axis];
            if(std::memcmp(&x,&y,sizeof(x))) return false;
        }
        return true;
    }
    struct MeshKey { std::vector<Point> points; std::vector<int> counts,indices;
        bool Matches(const std::vector<Point> &p,const std::vector<int> &c,
            const std::vector<int> &i) const { return counts==c && indices==i && Same(points,p); }
    };
    struct Transport : MeshKey { RigExecTransportRestData<Point,Wide> value; };
    struct Mush : MeshKey { int iterations=0; double step=0,distance=0; bool pin=false;
        RigExecDeltaMushRestData<Point,Wide> value; };
    std::shared_ptr<const Mush> mush;
    struct Wrinkle { size_t points=0; std::vector<int> counts,indices;
        RigExecWrinkleTopology mode; int distance=0; RigExecWrinkleMesh value; };
    std::shared_ptr<const Wrinkle> wrinkle;
    struct Normals : MeshKey { std::vector<Point> value; };
    struct Adjacency { size_t points=0; std::vector<int> counts,indices; RigExecMeshAdjacency value; };
    std::shared_ptr<const Adjacency> adjacency;
    struct Fan { size_t points=0; std::vector<int> counts,indices; RigExecFanTrisStrict value; };
    std::shared_ptr<const Transport> transport;
    std::shared_ptr<const Normals> normals[2];
    std::shared_ptr<const Fan> fan;
    std::shared_ptr<const std::vector<Point>> pointSamples[2];
    std::vector<Point> invalidSamples;
public:
    /// A boxed-array caller can retain the vector adapter without converting
    /// unchanged source arrays every frame. No borrowed pointer survives here.
    const std::vector<Point> &PointSamples(bool posed,const Point *data,size_t count) {
        if(count && !data) return invalidSamples;
        auto &slot=pointSamples[posed?1:0];
        bool same=slot && slot->size()==count;
        if(same) for(size_t i=0;i<count && same;++i) for(int axis=0;axis<3;++axis) {
            const auto a=(*slot)[i][axis],b=data[i][axis];
            if(std::memcmp(&a,&b,sizeof(a))) { same=false; break; }
        }
        if(!same) {
            auto value=std::make_shared<std::vector<Point>>();
            if(count) value->assign(data,data+count); slot=value;
        }
        return *slot;
    }
    const RigExecTransportRestData<Point,Wide> *TransportRest(
        const std::vector<Point> &p,const std::vector<int> &c,const std::vector<int> &i) {
        if(transport && transport->Matches(p,c,i)) return &transport->value;
        auto entry=std::make_shared<Transport>();
        if(!RigExecBuildTransportRestData<Point,Wide>(p,c,i,&entry->value)) return nullptr;
        entry->points=p; entry->counts=c; entry->indices=i; transport=entry;
        return &transport->value;
    }
    const RigExecDeltaMushRestData<Point,Wide> *MushRest(
        const std::vector<Point> &p,const std::vector<int> &c,const std::vector<int> &i,
        int iterations,double step,bool pin,double distance) {
        if(mush && mush->Matches(p,c,i) && mush->iterations==iterations && mush->pin==pin &&
            !std::memcmp(&mush->step,&step,sizeof(step)) &&
            !std::memcmp(&mush->distance,&distance,sizeof(distance))) return &mush->value;
        auto entry=std::make_shared<Mush>();
        if(!RigExecBuildDeltaMushRestData<Point,Wide>(p,c,i,iterations,step,pin,distance,&entry->value)) return nullptr;
        entry->points=p; entry->counts=c; entry->indices=i;
        entry->iterations=iterations; entry->step=step; entry->pin=pin; entry->distance=distance;
        mush=entry; return &mush->value;
    }
    const RigExecWrinkleMesh *WrinkleTopology(size_t points,
        const std::vector<int> &c,const std::vector<int> &i,
        RigExecWrinkleTopology mode,int distance) {
        if(wrinkle && wrinkle->points==points && wrinkle->counts==c && wrinkle->indices==i &&
            wrinkle->mode==mode && wrinkle->distance==distance) return &wrinkle->value;
        auto entry=std::make_shared<Wrinkle>();
        if(!RigExecBuildWrinkleMesh(points,c,i,mode,distance,&entry->value)) return nullptr;
        entry->points=points; entry->counts=c; entry->indices=i; entry->mode=mode; entry->distance=distance;
        wrinkle=entry; return &wrinkle->value;
    }
    template<class Compute> const std::vector<Point> &VertexNormals(bool posed,
        const std::vector<Point> &p,const std::vector<int> &c,
        const std::vector<int> &i,Compute &&compute) {
        auto &slot=normals[posed?1:0];
        if(!slot || !slot->Matches(p,c,i)) {
            auto entry=std::make_shared<Normals>(); entry->points=p;
            entry->counts=c; entry->indices=i; entry->value=compute(p,c,i); slot=entry;
        }
        return slot->value;
    }
    const RigExecMeshAdjacency *MeshAdjacency(size_t points,
        const std::vector<int> &c,const std::vector<int> &i) {
        if(!adjacency || adjacency->points!=points || adjacency->counts!=c || adjacency->indices!=i) {
            auto entry=std::make_shared<Adjacency>();
            if(!RigExecBuildMeshAdjacency(points,c.data(),c.size(),i.data(),i.size(),&entry->value)) return nullptr;
            entry->points=points; entry->counts=c; entry->indices=i; adjacency=entry;
        }
        return &adjacency->value;
    }
    const RigExecFanTrisStrict &FanTriangles(size_t points,
        const std::vector<int> &c,const std::vector<int> &i) {
        if(!fan || fan->points!=points || fan->counts!=c || fan->indices!=i) {
            auto entry=std::make_shared<Fan>(); entry->points=points;
            entry->counts=c; entry->indices=i;
            entry->value=RigExecBuildFanTrisStrict(c,i,points); fan=entry;
        }
        return fan->value;
    }
};
}
#endif
