#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/latticeKernel.h"
#include "rigExecMath/surfaceKernelCache.h"
#include "rigExecMath/wireKernelCache.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
using namespace rigExec;
namespace {
// The lattice as it read before binds were retained: every factor recomputed
// for every term and every term visited. An independent judge, never a path.
double RefBernstein(int degree,int index,double t) {
    double coefficient=1.0;
    for(int k=0;k<index;++k) coefficient*=double(degree-k)/double(index-k);
    return coefficient*std::pow(t,index)*std::pow(1.0-t,degree-index);
}
void RefLattice(std::vector<GfVec3f> *points,const std::vector<GfVec3f> &rest,
    const std::vector<GfVec3f> &restCage,const std::vector<GfVec3f> &posedCage,const GfVec3i &d) {
    const size_t cageCount=size_t(d[0])*size_t(d[1])*size_t(d[2]);
    if(points->empty() || rest.size()!=points->size() || restCage.size()!=cageCount ||
        posedCage.size()!=cageCount || d[0]<2 || d[1]<2 || d[2]<2) return;
    GfVec3f lo=restCage[0],hi=restCage[0];
    for(const GfVec3f &c:restCage) for(int a=0;a<3;++a) {
        lo[a]=std::min(lo[a],c[a]); hi[a]=std::max(hi[a],c[a]);
    }
    const GfVec3f size=hi-lo;
    if(size[0]<=0 || size[1]<=0 || size[2]<=0) return;
    for(size_t i=0;i<points->size();++i) {
        GfVec3f uvw;
        for(int a=0;a<3;++a) uvw[a]=std::min(1.0f,std::max(0.0f,(rest[i][a]-lo[a])/size[a]));
        GfVec3f delta(0.0f);
        for(int c=0;c<d[2];++c) for(int b=0;b<d[1];++b) for(int a=0;a<d[0];++a) {
            const size_t k=size_t((c*d[1]+b)*d[0]+a);
            delta+=(posedCage[k]-restCage[k])*float(RefBernstein(d[0]-1,a,uvw[0])*
                RefBernstein(d[1]-1,b,uvw[1])*RefBernstein(d[2]-1,c,uvw[2]));
        }
        (*points)[i]+=delta;
    }
}
bool SameBits(const std::vector<GfVec3f> &a,const std::vector<GfVec3f> &b) {
    return a.size()==b.size() && (a.empty() || !std::memcmp(a.data(),b.data(),a.size()*sizeof(GfVec3f)));
}
}
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

    // Lattice: a retained bind answers the per-term recomputation's bits for
    // points inside, on and outside the cage's bound (one, two and three
    // clamped axes), signed zeros and NaN, and is rebuilt only when the rest
    // points, the rest bound or the divisions change.
    {
        const float inf=std::numeric_limits<float>::infinity();
        const float nan=std::numeric_limits<float>::quiet_NaN();
        const GfVec3i divs(3,4,2);
        std::vector<GfVec3f> restCage;
        for(int c=0;c<2;++c) for(int b=0;b<4;++b) for(int a=0;a<3;++a)
            restCage.push_back(GfVec3f(float(a),1.5f*float(b),0.5f*float(c)));
        const auto posedAt=[&](float s) {
            std::vector<GfVec3f> posed=restCage;
            for(size_t k=0;k<posed.size();++k)
                posed[k]+=GfVec3f(0.1f*s*float(k%5),-0.05f*s*float(k%3),0.3f*s);
            return posed;
        };
        std::vector<GfVec3f> rest={{0.3f,1.1f,0.2f},{0,0,0},{2,4.5f,0.5f},
            {-1,2,0.25f},{5,-3,0.1f},{-2,9,-4},{-0.0f,0.7f,0.4f},
            {1.25f,nan,0.3f},{0.9f,3.3f,7}};
        std::vector<GfVec3f> start=rest;
        start[1]=GfVec3f(-0.0f,-0.0f,-0.0f);
        start[7]=GfVec3f(0.5f,0.5f,0.5f);
        RigExecSurfaceKernelCache<GfVec3f,GfVec3d> lattice;
        const auto same=[&](const std::vector<GfVec3f> &r,const std::vector<GfVec3f> &rc,
            const std::vector<GfVec3f> &pc,const GfVec3i &dv) {
            std::vector<GfVec3f> expected=start,once=start,retained=start;
            RefLattice(&expected,r,rc,pc,dv);
            RigExecApplyLattice(&once,r,rc,pc,dv,nullptr);
            RigExecApplyLattice(&retained,r,rc,pc,dv,&lattice);
            return SameBits(expected,once) && SameBits(expected,retained);
        };
        // Frames that move only the posed cage reuse the bind.
        CHECK(same(rest,restCage,posedAt(1.0f),divs));
        CHECK(lattice.LatticeBuilds()==1);
        CHECK(same(rest,restCage,posedAt(-2.5f),divs));
        CHECK(same(rest,restCage,posedAt(1e30f),divs));
        // At rest every delta is +0 and the -0 components become +0.
        CHECK(same(rest,restCage,restCage,divs));
        // Non-finite deltas visit every term, as the recomputation did. (Only
        // infinities: every NaN is then the default one, whichever operand
        // order an addition is compiled with.)
        std::vector<GfVec3f> wild=posedAt(1.0f);
        wild[3][0]=inf; wild[5][1]=-inf; wild[20][2]=inf;
        CHECK(same(rest,restCage,wild,divs));
        CHECK(lattice.LatticeBuilds()==1);
        // Clamped axes keep a single factor: point 4 sits at u=1 and v=0.
        GfVec3f lo,size;
        CHECK(RigExecLatticeBindBox(restCage.data(),restCage.size(),&lo,&size));
        const RigExecLatticeBasis *basis=lattice.LatticeBasis(
            rest.data(),rest.size(),lo,size,divs[0],divs[1],divs[2]);
        CHECK(basis && basis->bounded && basis->ranges.size()==rest.size()*6);
        if(basis && basis->ranges.size()==rest.size()*6) {
            const int *r=&basis->ranges[4*6];
            CHECK(r[0]==2 && r[1]==3 && r[2]==0 && r[3]==1 && r[4]==0 && r[5]==2);
        }
        auto copied=lattice;
        CHECK(copied.LatticeBasis(rest.data(),rest.size(),lo,size,
            divs[0],divs[1],divs[2])==basis);
        CHECK(copied.LatticeBuilds()==1);
        // An interior rest-cage point leaves the bound, and the bind, alone.
        std::vector<GfVec3f> interior=restCage;
        interior[4]=GfVec3f(1.2f,1.4f,0.1f);
        CHECK(same(rest,interior,posedAt(1.0f),divs));
        CHECK(lattice.LatticeBuilds()==1);
        // A moved bound, a rest point's signed zero and the divisions rebuild.
        std::vector<GfVec3f> grown=restCage;
        grown.back()=GfVec3f(2,4.5f,0.75f);
        CHECK(same(rest,grown,posedAt(1.0f),divs));
        CHECK(lattice.LatticeBuilds()==2);
        std::vector<GfVec3f> signedRest=rest;
        signedRest[6][0]=0.0f;
        CHECK(same(signedRest,restCage,posedAt(1.0f),divs));
        CHECK(lattice.LatticeBuilds()==3);
        const GfVec3i flat(4,3,2);
        CHECK(same(rest,restCage,posedAt(1.0f),flat));
        CHECK(lattice.LatticeBuilds()==4);
        CHECK(copied.LatticeBuilds()==1);
        // A degenerate bound and a short cage pass through.
        std::vector<GfVec3f> squashed=restCage;
        for(GfVec3f &p:squashed) p[2]=0.0f;
        CHECK(same(rest,squashed,posedAt(1.0f),divs));
        CHECK(same(rest,std::vector<GfVec3f>(restCage.begin(),restCage.end()-1),
            posedAt(1.0f),divs));
        // A degree whose binomial overflows yields inf/NaN factors: every term
        // is visited and the NaNs land where the recomputation put them.
        const GfVec3i wide(1100,2,2);
        std::vector<GfVec3f> wideCage;
        for(int c=0;c<2;++c) for(int b=0;b<2;++b) for(int a=0;a<wide[0];++a)
            wideCage.push_back(GfVec3f(float(a)/float(wide[0]-1),float(b),float(c)));
        std::vector<GfVec3f> widePosed=wideCage;
        for(GfVec3f &p:widePosed) p+=GfVec3f(0,0.1f,0);
        const std::vector<GfVec3f> wideRest={{0.5f,0.5f,0.5f},{0,0,0},{1.2f,0.3f,0.7f}};
        std::vector<GfVec3f> wideExpected=wideRest,wideOut=wideRest;
        RefLattice(&wideExpected,wideRest,wideCage,widePosed,wide);
        RigExecSurfaceKernelCache<GfVec3f,GfVec3d> wideCache;
        RigExecApplyLattice(&wideOut,wideRest,wideCage,widePosed,wide,&wideCache);
        CHECK(SameBits(wideExpected,wideOut));
        std::vector<GfVec3f> wideStreamed=wideRest;
        RigExecApplyLattice(&wideStreamed,wideRest,wideCage,widePosed,wide,nullptr);
        CHECK(SameBits(wideExpected,wideStreamed));
        CHECK(RigExecLatticeBindBox(wideCage.data(),wideCage.size(),&lo,&size));
        const RigExecLatticeBasis *wideBasis=wideCache.LatticeBasis(
            wideRest.data(),wideRest.size(),lo,size,wide[0],wide[1],wide[2]);
        CHECK(wideBasis && !wideBasis->bounded && wideCache.LatticeBuilds()==1);

        // Over the budget nothing is retained and the factors stream, with
        // the bits of the retained basis; at the bound it is retained.
        CHECK(RigExecLatticeBindBox(restCage.data(),restCage.size(),&lo,&size));
        const double bytes=RigExecLatticeBind<GfVec3f>::Bytes(
            rest.size(),divs[0],divs[1],divs[2]);
        CHECK(bytes>0 && bytes<double(RigExecLatticeBindBudgetBytes));
        RigExecSurfaceKernelCache<GfVec3f,GfVec3d> tight,roomy;
        tight.SetLatticeBudget(size_t(bytes)-1);
        roomy.SetLatticeBudget(size_t(bytes));
        CHECK(tight.LatticeBasis(rest.data(),rest.size(),lo,size,
            divs[0],divs[1],divs[2])==nullptr);
        for(const float s:{1.0f,-2.5f,1e30f}) {
            std::vector<GfVec3f> expected=start,streamed=start,retained=start;
            RefLattice(&expected,rest,restCage,posedAt(s),divs);
            RigExecApplyLattice(&streamed,rest,restCage,posedAt(s),divs,&tight);
            RigExecApplyLattice(&retained,rest,restCage,posedAt(s),divs,&roomy);
            CHECK(SameBits(expected,streamed) && SameBits(expected,retained));
        }
        std::vector<GfVec3f> wildStreamed=start,wildExpected=start;
        RefLattice(&wildExpected,rest,restCage,wild,divs);
        RigExecApplyLattice(&wildStreamed,rest,restCage,wild,divs,&tight);
        CHECK(SameBits(wildExpected,wildStreamed));
        CHECK(tight.LatticeBuilds()==0 && !tight.RetainedLatticeBind());
        CHECK(roomy.LatticeBuilds()==1 && roomy.RetainedLatticeBind());
        // A retained bind that no longer matches is released, not kept, when
        // the new one is over the budget.
        roomy.SetLatticeBudget(size_t(bytes)-1);
        std::vector<GfVec3f> released=start,releasedExpected=start;
        RigExecApplyLattice(&released,signedRest,restCage,posedAt(1.0f),divs,&roomy);
        RefLattice(&releasedExpected,signedRest,restCage,posedAt(1.0f),divs);
        CHECK(SameBits(releasedExpected,released));
        CHECK(roomy.LatticeBuilds()==1 && !roomy.RetainedLatticeBind());

        // Caches that built equal binds apart are handed one instance, and
        // answer the bits they did with their own; another bind keeps its own.
        RigExecSurfaceKernelCache<GfVec3f,GfVec3d> first,second,other;
        std::vector<GfVec3f> a=start,b=start,c=start;
        RigExecApplyLattice(&a,rest,restCage,posedAt(1.0f),divs,&first);
        RigExecApplyLattice(&b,rest,restCage,posedAt(1.0f),divs,&second);
        RigExecApplyLattice(&c,rest,restCage,posedAt(1.0f),flat,&other);
        CHECK(first.RetainedLatticeBind() && second.RetainedLatticeBind() &&
              first.RetainedLatticeBind()!=second.RetainedLatticeBind());
        const auto otherBind=other.RetainedLatticeBind();
        {
            RigExecLatticeBindSharing<GfVec3f> sharing;
            sharing.Offer(&first); sharing.Offer(&other); sharing.Offer(&second);
            sharing.Offer(&tight);
        }
        CHECK(first.RetainedLatticeBind() &&
              second.RetainedLatticeBind()==first.RetainedLatticeBind());
        CHECK(other.RetainedLatticeBind()==otherBind);
        CHECK(!tight.RetainedLatticeBind());
        for(const float s:{-2.5f,1e30f}) {
            std::vector<GfVec3f> expected=start,shared=start;
            RefLattice(&expected,rest,restCage,posedAt(s),divs);
            RigExecApplyLattice(&shared,rest,restCage,posedAt(s),divs,&second);
            CHECK(SameBits(expected,shared));
        }
        CHECK(first.LatticeBuilds()==1 && second.LatticeBuilds()==1);
        CHECK(second.RetainedLatticeBind()==first.RetainedLatticeBind());
        // A shared bind is immutable: one holder's rebuild leaves the other's.
        const auto shared=first.RetainedLatticeBind();
        RigExecApplyLattice(&a,signedRest,restCage,posedAt(1.0f),divs,&first);
        CHECK(first.LatticeBuilds()==2 && first.RetainedLatticeBind()!=shared);
        CHECK(second.RetainedLatticeBind()==shared);
    }
    std::printf("SurfaceKernelCache: %d failures\n",failures);
    return failures?1:0;
}
