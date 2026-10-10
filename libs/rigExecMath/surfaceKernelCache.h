#ifndef RIGEXEC_MATH_SURFACE_KERNEL_CACHE_H
#define RIGEXEC_MATH_SURFACE_KERNEL_CACHE_H
#include "deltaMushKernel.h"
#include "spatialAccel.h"
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
namespace rigExec {
// Named only: the lattice kernel (latticeKernel.h) defines the bind and
// instantiates the members below that build or compare one.
struct RigExecLatticeBasis;
template<class Point> struct RigExecLatticeBind;
/// The most one revision retains for a lattice bind, in bytes. A bind over
/// it is not retained: the kernel streams that bind's factors per point
/// instead, the same bits at (dx+dy+dz) Bernstein calls per point per frame.
/// 64 MiB keeps every sample rig's binds (bust_anim: 29K points at 13 and 16
/// divisions, under 5 MB a bind) and meshes up to ~400K points at that
/// density, while a dense lattice on a large mesh (1M points at 30
/// divisions per axis, ~760 MB) streams.
constexpr size_t RigExecLatticeBindBudgetBytes = size_t(64) << 20;
/// One revision owns and mutates this cache. Clones share only immutable
/// entries; no global state or locking. Keys compare every scalar's raw bits,
/// including signed zero/NaN payloads, and every topology element. Between
/// runs, the thread that runs the owning program may also hand the cache an
/// equal, immutable lattice bind (RigExecLatticeBindSharing).
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
        RigExecDeltaMushSettings settings;
        RigExecDeltaMushRestData<Point,Wide> value; };
    // The smoothing settings by value, the influence weights by their bits.
    static bool SameSettings(const RigExecDeltaMushSettings &a,const RigExecDeltaMushSettings &b) {
        return a.smoothing==b.smoothing && a.frameTransport==b.frameTransport &&
            a.onlySmooth==b.onlySmooth && a.edges==b.edges &&
            a.smoothWeights.size()==b.smoothWeights.size() &&
            (a.smoothWeights.empty() || !std::memcmp(a.smoothWeights.data(),b.smoothWeights.data(),
                a.smoothWeights.size()*sizeof(float)));
    }
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
    std::shared_ptr<const RigExecLatticeBind<Point>> lattice;
    /// The nonzero content version of the rest points `lattice` was built
    /// from, or 0 (unknown). Kept by ShareLatticeBind (equal bytes) and
    /// cleared with the bind.
    uint64_t latticeRestVersion=0;
    size_t latticeBuilds=0;
    size_t latticeBudget=RigExecLatticeBindBudgetBytes;
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
        int iterations,double step,bool pin,double distance,
        const RigExecDeltaMushSettings &settings=RigExecDeltaMushSettings()) {
        if(mush && mush->Matches(p,c,i) && mush->iterations==iterations && mush->pin==pin &&
            !std::memcmp(&mush->step,&step,sizeof(step)) &&
            !std::memcmp(&mush->distance,&distance,sizeof(distance)) &&
            SameSettings(mush->settings,settings)) return &mush->value;
        auto entry=std::make_shared<Mush>();
        if(!RigExecBuildDeltaMushRestData<Point,Wide>(p,c,i,iterations,step,pin,distance,settings,&entry->value)) return nullptr;
        entry->points=p; entry->counts=c; entry->indices=i;
        entry->iterations=iterations; entry->step=step; entry->pin=pin; entry->distance=distance;
        entry->settings=settings;
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
    /// The lattice bind of \p count rest points in the rest cage's bound
    /// (\p lo, \p size) at \p dx x \p dy x \p dz divisions: everything the
    /// basis reads, so a frame that only moves the posed cage is a compare.
    /// Null when the bind would retain more than the budget: nothing is kept
    /// and the kernel streams the factors, the same bits (latticeKernel.h).
    /// \p restVersion (0: unknown) is the caller's content version of
    /// \p rest: equal to the nonzero one the bind was built from, only the
    /// rest-point compare is skipped.
    const RigExecLatticeBasis *LatticeBasis(const Point *rest,size_t count,
        const Point &lo,const Point &size,int dx,int dy,int dz,
        uint64_t restVersion=0) {
        if(count && !rest) return nullptr;
        const bool restKnown=restVersion!=0 && restVersion==latticeRestVersion;
        if(lattice && lattice->Matches(rest,count,lo,size,dx,dy,dz,restKnown)) return &lattice->value;
        lattice.reset(); latticeRestVersion=0;
        if(RigExecLatticeBind<Point>::Bytes(count,dx,dy,dz)>double(latticeBudget)) return nullptr;
        auto entry=std::make_shared<RigExecLatticeBind<Point>>();
        entry->Build(rest,count,lo,size,dx,dy,dz);
        lattice=entry; latticeRestVersion=restVersion; ++latticeBuilds;
        return &lattice->value;
    }
    /// The lattice bind retained, or null (RigExecLatticeBindSharing).
    const std::shared_ptr<const RigExecLatticeBind<Point>> &RetainedLatticeBind() const {
        return lattice;
    }
    /// Retains \p bind, which equals the bind held: the owning program's
    /// thread, between runs (RigExecLatticeBindSharing).
    void ShareLatticeBind(const std::shared_ptr<const RigExecLatticeBind<Point>> &bind) {
        lattice=bind;
    }
    /// The bytes a lattice bind is retained under; a test may lower it.
    void SetLatticeBudget(size_t bytes) { latticeBudget=bytes; }
    /// Test observable: lattice binds built, counted across copies.
    size_t LatticeBuilds() const { return latticeBuilds; }
    /// Test observable: the rest version the retained bind was built from.
    uint64_t LatticeRestVersion() const { return latticeRestVersion; }
};
}
#endif
