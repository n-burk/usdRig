#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/surfaceKernelCache.h"
#include "rigExecMath/wireKernelCache.h"
#include <cstdio>
using namespace rigExec;
int main() {
    int failures=0;
#define CHECK(x) do { if(!(x)) { ++failures; std::printf("failure line %d\n",__LINE__); } } while(0)
    std::vector<GfVec3f> points={{0,0,0},{1,0,0},{0,1,0}};
    std::vector<int> counts={3},indices={0,1,2};
    RigExecSurfaceKernelCache<GfVec3f,GfVec3d> cache;
    const auto *samples=&cache.PointSamples(false,points.data(),points.size());
    CHECK(&cache.PointSamples(false,points.data(),points.size())==samples);
    CHECK(cache.PointSamples(false,nullptr,1).empty());
    auto *rest=cache.TransportRest(points,counts,indices); CHECK(rest);
    CHECK(cache.TransportRest(points,counts,indices)==rest);
    auto clone=cache;
    auto signedPoints=points; signedPoints[0][0]=-0.0f;
    CHECK(cache.TransportRest(signedPoints,counts,indices)!=rest);
    CHECK(clone.TransportRest(points,counts,indices)==rest);
    auto *adj=cache.MeshAdjacency(points.size(),counts,indices); CHECK(adj);
    CHECK(cache.MeshAdjacency(points.size(),counts,indices)==adj);
    CHECK(cache.MeshAdjacency(points.size(),counts,{0,1,99})==nullptr);
    CHECK(cache.MeshAdjacency(points.size(),counts,indices)==adj);
    auto *mush=cache.MushRest(points,counts,indices,2,0.5,true,0.0); CHECK(mush);
    CHECK(cache.MushRest(points,counts,indices,2,0.5,true,0.0)==mush);
    auto retained=cache;
    CHECK(cache.MushRest(points,counts,indices,3,0.5,true,0.0)!=mush);
    CHECK(retained.MushRest(points,counts,indices,2,0.5,true,0.0)==mush);
    auto *wrinkle=cache.WrinkleTopology(points.size(),counts,indices,RigExecWrinkleTopology::Cloth,2);
    CHECK(wrinkle && cache.WrinkleTopology(points.size(),counts,indices,RigExecWrinkleTopology::Cloth,2)==wrinkle);
    int normalsCalls=0;
    auto normals=[&](const auto &p,const auto &c,const auto &i) {
        ++normalsCalls; return RigExecComputeVertexNormals(p,c,i);
    };
    cache.VertexNormals(false,points,counts,indices,normals);
    cache.VertexNormals(false,points,counts,indices,normals); CHECK(normalsCalls==1);
    cache.VertexNormals(true,points,counts,indices,normals); CHECK(normalsCalls==2);
    cache.VertexNormals(false,signedPoints,counts,indices,normals); CHECK(normalsCalls==3);
    auto *fan=&cache.FanTriangles(points.size(),counts,indices);
    CHECK(fan->valid && &cache.FanTriangles(points.size(),counts,indices)==fan);
    RigExecWireRestCache<GfVec3f,GfVec2f> wire;
    std::vector<GfVec3f> controls={{0,0,0},{1,0,0}};
    std::vector<double> knots={0,0,1,1};
    RigExecNurbsCurve curve{&controls,2,&knots};
    std::vector<GfVec2f> binds={{0.25f,0},{0.75f,0}};
    const auto *evals=wire.Get(curve,binds.data(),binds.size()); CHECK(evals);
    CHECK(wire.Get(curve,binds.data(),binds.size())==evals);
    CHECK((*evals)[0]==curve.Evaluate(0.25));
    auto wireClone=wire; binds[0][1]=-0.0f;
    CHECK(wire.Get(curve,binds.data(),binds.size())!=evals);
    binds[0][1]=0; CHECK(wireClone.Get(curve,binds.data(),binds.size())==evals);
    std::printf("SurfaceKernelCache: %d failures\n",failures);
    return failures?1:0;
}
