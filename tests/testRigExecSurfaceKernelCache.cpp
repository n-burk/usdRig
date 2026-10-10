#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/latticeKernel.h"
#include "rigExecMath/pointBlocks.h"
#include "rigExecMath/pointRanges.h"
#include "rigExecMath/surfaceKernelCache.h"
#include "rigExecMath/wireKernelCache.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <vector>
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
// Runs \p range over each [bounds[k], bounds[k+1]) of a copy of \p start
// whose other points hold a NaN-payload poison, and checks that each range
// leaves those other points alone; returns the ranges' points assembled.
template<class Range>
std::vector<GfVec3f> RunRanges(const std::vector<GfVec3f> &start,
    const std::vector<size_t> &bounds,Range &&range,bool *untouchedOutside) {
    float poisonValue; const uint32_t bits=0x7fc5a5a5u;
    std::memcpy(&poisonValue,&bits,sizeof(bits));
    const GfVec3f poison(poisonValue,-0.0f,poisonValue);
    std::vector<GfVec3f> assembled(start.size(),poison);
    for(size_t k=0;k+1<bounds.size();++k) {
        const size_t b=bounds[k],e=bounds[k+1];
        std::vector<GfVec3f> points(start.size(),poison);
        std::copy(start.begin()+b,start.begin()+e,points.begin()+b);
        range(&points,b,e);
        for(size_t i=0;i<points.size();++i)
            if((i<b || i>=e) && std::memcmp(&points[i],&poison,sizeof(GfVec3f)))
                *untouchedOutside=false;
        std::copy(points.begin()+b,points.begin()+e,assembled.begin()+b);
    }
    return assembled;
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

    // Point ranges: Build's count and bounds, a run's clip of them, and the
    // copy a range publishes with.
    {
        CHECK(RigExecPointRangeCount(26276,4096,8)==7);
        CHECK(RigExecPointRangeCount(4096,4096,8)==1);
        CHECK(RigExecPointRangeCount(4097,4096,8)==2);
        CHECK(RigExecPointRangeCount(0,4096,8)==1);
        CHECK(RigExecPointRangeCount(100000,4096,8)==8);
        CHECK(RigExecPointRangeCount(5,0,8)==5 && RigExecPointRangeCount(5,1,0)==1);
        CHECK(RigExecPointRangeBound(26276,7,0)==0);
        CHECK(RigExecPointRangeBound(26276,7,1)==3754);
        CHECK(RigExecPointRangeBound(26276,7,6)==22524);
        CHECK(RigExecPointRangeBound(26276,7,7)==26276);
        CHECK(RigExecPointRangeBound(10,3,1)==4 && RigExecPointRangeBound(10,3,2)==8 &&
              RigExecPointRangeBound(10,3,3)==10);
        CHECK(RigExecPointRangeBound(10,0,1)==10);
        // Clipped to any run count, the Build ranges cover [0, count) once.
        const size_t built=10000,ranges=RigExecPointRangeCount(built,4096,8);
        CHECK(ranges==3);
        for(const size_t run:{size_t(0),size_t(1),size_t(3333),size_t(9999),
                              size_t(10000),size_t(10007)}) {
            size_t covered=0,next=0; bool contiguous=true;
            for(size_t k=0;k<ranges;++k) {
                size_t b=0,e=0;
                RigExecPointRangeAt(int(RigExecPointRangeBound(built,ranges,k)),
                    int(RigExecPointRangeBound(built,ranges,k+1)),k+1==ranges,run,&b,&e);
                contiguous=contiguous && b==next && e>=b;
                covered+=e-b; next=e;
            }
            CHECK(contiguous && covered==run && next==run);
        }
        size_t b=9,e=9;
        RigExecPointRangeAt(-3,4,false,10,&b,&e); CHECK(b==0 && e==4);
        RigExecPointRangeAt(8,10,false,5,&b,&e); CHECK(b==5 && e==5);
        RigExecPointRangeAt(8,10,true,5,&b,&e); CHECK(b==5 && e==5);
        RigExecPointRangeAt(8,10,true,12,&b,&e); CHECK(b==8 && e==12);
        // Blocks are compared by bytes: a signed zero and a NaN payload move.
        std::vector<GfVec3f> in(2500),out;
        for(size_t i=0;i<in.size();++i) in[i]=GfVec3f(float(i),-float(i),0.5f);
        out=in;
        CHECK(!RigExecCopyMovedRange(in.data(),out.data(),in.size()) && SameBits(in,out));
        in[1500][2]=-0.0f; out[1500][2]=0.0f;
        CHECK(RigExecCopyMovedRange(in.data(),out.data(),in.size()) && SameBits(in,out));
        float payload; const uint32_t payloadBits=0x7fc00011u;
        std::memcpy(&payload,&payloadBits,sizeof(payload));
        in[2499][0]=payload; out[2499][0]=std::numeric_limits<float>::quiet_NaN();
        CHECK(RigExecCopyMovedRange(in.data(),out.data(),in.size()) && SameBits(in,out));
        CHECK(!RigExecCopyMovedRange(in.data(),out.data(),0));
    }

    // Lattice ranges: over an uneven partition and over one range, the
    // range kernel writes each point's bits of the whole kernel, through a
    // retained basis and per point, with non-finite cage deltas, and with an
    // overflowing degree (an unbounded basis); and it touches no point
    // outside its range.
    {
        const float inf=std::numeric_limits<float>::infinity();
        const GfVec3i divs(3,4,2);
        std::vector<GfVec3f> restCage;
        for(int c=0;c<2;++c) for(int b=0;b<4;++b) for(int a=0;a<3;++a)
            restCage.push_back(GfVec3f(float(a),1.5f*float(b),0.5f*float(c)));
        std::vector<GfVec3f> posedCage=restCage;
        for(size_t k=0;k<posedCage.size();++k)
            posedCage[k]+=GfVec3f(0.1f*float(k%5),-0.05f*float(k%3),0.3f);
        const size_t count=1001;
        std::vector<GfVec3f> rest(count),start(count);
        for(size_t i=0;i<count;++i) {
            const float t=float(i);
            rest[i]=GfVec3f(1.0f+1.6f*std::sin(0.7f*t),2.25f+3.0f*std::cos(0.3f*t),
                0.25f+0.4f*std::sin(1.3f*t));
            start[i]=GfVec3f(std::sin(0.37f*t),std::cos(0.11f*t),0.01f*t);
        }
        start[0]=GfVec3f(-0.0f,0.0f,-0.0f); start[333]=GfVec3f(inf,-0.0f,-inf);
        start[700]=GfVec3f(-0.0f,-0.0f,-0.0f);
        const std::vector<std::vector<size_t>> partitions={{0,1,333,334,700,1001},{0,1001}};
        const auto rangesMatch=[&](const std::vector<GfVec3f> &r,const std::vector<GfVec3f> &rc,
            const std::vector<GfVec3f> &pc,const GfVec3i &dv,const std::vector<GfVec3f> &enter,
            const std::vector<std::vector<size_t>> &cuts) {
            RigExecSurfaceKernelCache<GfVec3f,GfVec3d> cache;
            std::vector<GfVec3f> streamed=enter,retained=enter;
            RigExecApplyLattice(&streamed,r,rc,pc,dv,nullptr);
            RigExecApplyLattice(&retained,r,rc,pc,dv,&cache);
            GfVec3f lo,size;
            const RigExecLatticeBasis *basis=nullptr;
            if(RigExecLatticeKernelSetup(enter.size(),r.data(),r.size(),rc.data(),rc.size(),
                pc.data(),pc.size(),dv[0],dv[1],dv[2],&lo,&size,
                static_cast<std::vector<GfVec3f>*>(nullptr)))
                basis=cache.LatticeBasis(r.data(),r.size(),lo,size,dv[0],dv[1],dv[2]);
            bool ok=basis!=nullptr && !SameBits(retained,enter);
            for(const auto &bounds:cuts) {
                for(const RigExecLatticeBasis *use:{basis,(const RigExecLatticeBasis*)nullptr}) {
                    bool outside=true;
                    const auto ranged=RunRanges(enter,bounds,
                        [&](std::vector<GfVec3f> *points,size_t b,size_t e) {
                            RigExecApplyLatticeKernelRange(points,b,e,r.data(),r.size(),
                                rc.data(),rc.size(),pc.data(),pc.size(),dv[0],dv[1],dv[2],use);
                        },&outside);
                    ok=ok && outside && SameBits(ranged,use ? retained : streamed);
                }
            }
            return ok;
        };
        CHECK(rangesMatch(rest,restCage,posedCage,divs,start,partitions));
        std::vector<GfVec3f> wild=posedCage;
        wild[3][0]=inf; wild[5][1]=-inf;
        CHECK(rangesMatch(rest,restCage,wild,divs,start,partitions));
        // An overflowing degree: +inf and NaN factors, an unbounded basis
        // that visits every term while some points' own factors are
        // bounded. Its factors cost O(divisions^2) each, so fewer points.
        const GfVec3i wide(1100,2,2);
        std::vector<GfVec3f> wideCage;
        for(int c=0;c<2;++c) for(int b=0;b<2;++b) for(int a=0;a<wide[0];++a)
            wideCage.push_back(GfVec3f(float(a)/float(wide[0]-1),float(b),float(c)));
        std::vector<GfVec3f> widePosed=wideCage;
        for(GfVec3f &p:widePosed) p+=GfVec3f(0,0.1f,0);
        std::vector<GfVec3f> wideRest(25),wideStart(25);
        for(size_t i=0;i<wideRest.size();++i) {
            wideRest[i]=GfVec3f(float(i)/20.0f-0.1f,0.3f+0.02f*float(i),0.7f);
            wideStart[i]=GfVec3f(float(i),-float(i),0.25f);
        }
        {
            RigExecSurfaceKernelCache<GfVec3f,GfVec3d> probe;
            GfVec3f lo,size;
            CHECK(RigExecLatticeBindBox(wideCage.data(),wideCage.size(),&lo,&size));
            const RigExecLatticeBasis *wideBasis=probe.LatticeBasis(wideRest.data(),
                wideRest.size(),lo,size,wide[0],wide[1],wide[2]);
            CHECK(wideBasis && !wideBasis->bounded);
        }
        CHECK(rangesMatch(wideRest,wideCage,widePosed,wide,wideStart,
            {{0,1,8,9,17,25},{0,25}}));
        // A basis for other points passes the range through, as the whole
        // call does.
        RigExecLatticeBasis other;
        other.divisions[0]=3; other.divisions[1]=4; other.divisions[2]=2;
        std::vector<GfVec3f> through=start;
        RigExecApplyLatticeBasisRange(through.data(),through.size(),1,333,other,
            restCage.data());
        CHECK(SameBits(through,start));
    }

    // Wire basis ranges: each point receives the whole call's additions in
    // the same order, a range with no weighted point writes nothing, and the
    // range form validates as the whole one does.
    {
        std::vector<GfVec3f> rest={{0,0,0},{1,0,0},{2,0,0},{3,0,0},{4,0,0}};
        std::vector<GfVec3f> posed={{0,0,0},{1,0.5f,0},{2,-0.25f,0.3f},{3,0.75f,-0.1f},{4,0,0.4f}};
        const std::vector<double> knots={0,0,0,1,2,3,3,3};
        const size_t count=1001;
        std::vector<int> indices{0};
        for(int i=3;i<333;i+=7) indices.push_back(i);
        for(int i=340;i<700;i+=12) indices.push_back(i);
        std::vector<float> weights;
        std::vector<GfVec2f> binds;
        for(size_t k=0;k<indices.size();++k) {
            weights.push_back(float(k%6)/5.0f);
            binds.push_back(GfVec2f(3.0f*float(k)/float(indices.size()),0.25f*float(k%11)));
        }
        RigExecWireBasis basis;
        CHECK(RigExecBuildWireBasis(binds.data(),binds.size(),count,indices,3,knots,
            rest.size(),2.0,&basis));
        std::vector<GfVec3f> start(count);
        for(size_t i=0;i<count;++i) start[i]=GfVec3f(std::sin(0.37f*float(i)),-0.0f,0.5f);
        start[0]=GfVec3f(-0.0f,-0.0f,-0.0f);
        std::vector<GfVec3f> whole=start;
        CHECK(RigExecApplyWireBasis(&whole,basis,indices,weights,rest,posed));
        CHECK(!SameBits(whole,start));
        for(const std::vector<size_t> &bounds:
                std::vector<std::vector<size_t>>{{0,1,333,334,700,1001},{0,1001}}) {
            bool outside=true;
            const auto ranged=RunRanges(start,bounds,
                [&](std::vector<GfVec3f> *points,size_t b,size_t e) {
                    CHECK(RigExecApplyWireBasisRange(points,basis,indices,weights,
                        rest,posed,b,e));
                },&outside);
            CHECK(outside && SameBits(ranged,whole));
        }
        // No weighted point in [700, 1001): untouched by the whole call.
        CHECK(!std::memcmp(whole.data()+700,start.data()+700,301*sizeof(GfVec3f)));
        std::vector<GfVec3f> shortPosed=posed; shortPosed.pop_back();
        std::vector<GfVec3f> refused=start;
        CHECK(!RigExecApplyWireBasisRange(&refused,basis,indices,weights,rest,
            shortPosed,0,count) && SameBits(refused,start));
        CHECK(!RigExecApplyWireBasis(&refused,basis,indices,weights,rest,shortPosed));
        CHECK(!RigExecApplyWireBasisRange(nullptr,basis,indices,weights,rest,posed,0,1));
    }
    // Lattice groups: each group's points in buffers of their own -- the
    // entering ones read at in[k], the result written over a poison at
    // out[k] -- give the whole kernel's bits through a retained basis and
    // per point, with non-finite cage deltas and with an overflowing degree
    // (an unbounded basis); an invalid cage, or a basis for other points,
    // leaves each group's buffer holding its entering points.
    {
        const float inf=std::numeric_limits<float>::infinity();
        float poisonValue; const uint32_t bits=0x7fc5a5a5u;
        std::memcpy(&poisonValue,&bits,sizeof(bits));
        const GfVec3f poison(poisonValue,-0.0f,poisonValue);
        const GfVec3i divs(3,4,2);
        std::vector<GfVec3f> restCage;
        for(int c=0;c<2;++c) for(int b=0;b<4;++b) for(int a=0;a<3;++a)
            restCage.push_back(GfVec3f(float(a),1.5f*float(b),0.5f*float(c)));
        std::vector<GfVec3f> posedCage=restCage;
        for(size_t k=0;k<posedCage.size();++k)
            posedCage[k]+=GfVec3f(0.1f*float(k%5),-0.05f*float(k%3),0.3f);
        const size_t count=1001;
        std::vector<GfVec3f> rest(count),start(count);
        for(size_t i=0;i<count;++i) {
            const float t=float(i);
            rest[i]=GfVec3f(1.0f+1.6f*std::sin(0.7f*t),2.25f+3.0f*std::cos(0.3f*t),
                0.25f+0.4f*std::sin(1.3f*t));
            start[i]=GfVec3f(std::sin(0.37f*t),std::cos(0.11f*t),0.01f*t);
        }
        start[0]=GfVec3f(-0.0f,0.0f,-0.0f); start[333]=GfVec3f(inf,-0.0f,-inf);
        start[700]=GfVec3f(-0.0f,-0.0f,-0.0f);
        const std::vector<std::vector<size_t>> partitions={{0,1,333,334,700,1001},{0,1001}};
        // The groups of \p bounds assembled, each run from its own buffers;
        // a guard point on each side of a group's output must survive.
        bool guarded=true;
        const auto groups=[&](const std::vector<GfVec3f> &r,const std::vector<GfVec3f> &rc,
            const std::vector<GfVec3f> &pc,const GfVec3i &dv,const std::vector<GfVec3f> &enter,
            const std::vector<size_t> &bounds,const RigExecLatticeBasis *use) {
            std::vector<GfVec3f> assembled(enter.size(),poison);
            for(size_t k=0;k+1<bounds.size();++k) {
                const size_t b=bounds[k],e=bounds[k+1];
                const std::vector<GfVec3f> in(enter.begin()+long(b),enter.begin()+long(e));
                std::vector<GfVec3f> out(e-b+2,poison);
                RigExecApplyLatticeKernelGroup(in.data(),out.data()+1,enter.size(),b,e,
                    r.data(),r.size(),rc.data(),rc.size(),pc.data(),pc.size(),
                    dv[0],dv[1],dv[2],use);
                if(std::memcmp(&out.front(),&poison,sizeof(GfVec3f)) ||
                   std::memcmp(&out.back(),&poison,sizeof(GfVec3f))) guarded=false;
                std::copy(out.begin()+1,out.end()-1,assembled.begin()+long(b));
            }
            return assembled;
        };
        const auto groupsMatch=[&](const std::vector<GfVec3f> &r,const std::vector<GfVec3f> &rc,
            const std::vector<GfVec3f> &pc,const GfVec3i &dv,const std::vector<GfVec3f> &enter,
            const std::vector<std::vector<size_t>> &cuts) {
            RigExecSurfaceKernelCache<GfVec3f,GfVec3d> cache;
            std::vector<GfVec3f> streamed=enter,retained=enter;
            RigExecApplyLattice(&streamed,r,rc,pc,dv,nullptr);
            RigExecApplyLattice(&retained,r,rc,pc,dv,&cache);
            GfVec3f lo,size;
            const RigExecLatticeBasis *basis=nullptr;
            if(RigExecLatticeKernelSetup(enter.size(),r.data(),r.size(),rc.data(),rc.size(),
                pc.data(),pc.size(),dv[0],dv[1],dv[2],&lo,&size,
                static_cast<std::vector<GfVec3f>*>(nullptr)))
                basis=cache.LatticeBasis(r.data(),r.size(),lo,size,dv[0],dv[1],dv[2]);
            bool ok=basis!=nullptr && !SameBits(retained,enter);
            for(const auto &bounds:cuts)
                for(const RigExecLatticeBasis *use:{basis,(const RigExecLatticeBasis*)nullptr})
                    ok=ok && SameBits(groups(r,rc,pc,dv,enter,bounds,use),use ? retained : streamed);
            return ok;
        };
        CHECK(groupsMatch(rest,restCage,posedCage,divs,start,partitions));
        std::vector<GfVec3f> wild=posedCage;
        wild[3][0]=inf; wild[5][1]=-inf;
        CHECK(groupsMatch(rest,restCage,wild,divs,start,partitions));
        const GfVec3i wide(1100,2,2);
        std::vector<GfVec3f> wideCage;
        for(int c=0;c<2;++c) for(int b=0;b<2;++b) for(int a=0;a<wide[0];++a)
            wideCage.push_back(GfVec3f(float(a)/float(wide[0]-1),float(b),float(c)));
        std::vector<GfVec3f> widePosed=wideCage;
        for(GfVec3f &p:widePosed) p+=GfVec3f(0,0.1f,0);
        std::vector<GfVec3f> wideRest(25),wideStart(25);
        for(size_t i=0;i<wideRest.size();++i) {
            wideRest[i]=GfVec3f(float(i)/20.0f-0.1f,0.3f+0.02f*float(i),0.7f);
            wideStart[i]=GfVec3f(float(i),-float(i),0.25f);
        }
        CHECK(groupsMatch(wideRest,wideCage,widePosed,wide,wideStart,
            {{0,1,8,9,17,25},{0,25}}));
        // Refused setups pass every point through, over the poison: a short
        // posed cage and a single division.
        const std::vector<GfVec3f> shortCage(posedCage.begin(),posedCage.end()-1);
        CHECK(SameBits(groups(rest,restCage,shortCage,divs,start,partitions[0],nullptr),start));
        CHECK(SameBits(groups(rest,restCage,posedCage,GfVec3i(1,4,2),start,partitions[0],nullptr),
            start));
        RigExecLatticeBasis other;
        other.divisions[0]=3; other.divisions[1]=4; other.divisions[2]=2;
        CHECK(SameBits(groups(rest,restCage,posedCage,divs,start,partitions[0],&other),start));
        CHECK(guarded);
    }

    // Wire groups: the basis and the dense walk over a group's own buffer,
    // seeded with its entering points, give the whole calls' bits, write no
    // point outside the buffer, and validate as the whole calls do.
    {
        std::vector<GfVec3f> rest={{0,0,0},{1,0,0},{2,0,0},{3,0,0},{4,0,0}};
        std::vector<GfVec3f> posed={{0,0,0},{1,0.5f,0},{2,-0.25f,0.3f},{3,0.75f,-0.1f},{4,0,0.4f}};
        const std::vector<double> knots={0,0,0,1,2,3,3,3};
        const size_t count=1001;
        std::vector<int> indices{0};
        for(int i=3;i<333;i+=7) indices.push_back(i);
        for(int i=340;i<700;i+=12) indices.push_back(i);
        std::vector<float> weights;
        std::vector<GfVec2f> sparseBinds,binds(count);
        for(size_t k=0;k<indices.size();++k) {
            weights.push_back(float(k%6)/5.0f);
            sparseBinds.push_back(GfVec2f(3.0f*float(k)/float(indices.size()),0.25f*float(k%11)));
        }
        for(size_t i=0;i<count;++i) binds[i]=GfVec2f(3.0f*float(i%97)/96.0f,0.2f*float(i%13));
        RigExecWireBasis basis;
        CHECK(RigExecBuildWireBasis(sparseBinds.data(),sparseBinds.size(),count,indices,3,knots,
            rest.size(),2.0,&basis));
        std::vector<GfVec3f> start(count);
        for(size_t i=0;i<count;++i) start[i]=GfVec3f(std::sin(0.37f*float(i)),-0.0f,0.5f);
        start[0]=GfVec3f(-0.0f,-0.0f,-0.0f);
        const RigExecNurbsCurve restCurve{&rest,3,&knots},posedCurve{&posed,3,&knots};
        std::vector<GfVec3f> evals(count);
        for(size_t i=0;i<count;++i) evals[i]=restCurve.Evaluate(binds[i][0]);
        std::vector<GfVec3f> wholeBasis=start,wholeDense=start,wholeTable=start;
        CHECK(RigExecApplyWireBasis(&wholeBasis,basis,indices,weights,rest,posed));
        CHECK(RigExecApplyWire(&wholeDense,restCurve,posedCurve,binds.data(),binds.size(),
            2.0,0,count));
        CHECK(RigExecApplyWire(&wholeTable,restCurve,posedCurve,binds.data(),binds.size(),
            2.0,0,count,evals.data(),evals.size()));
        CHECK(!SameBits(wholeBasis,start) && !SameBits(wholeDense,start) &&
              SameBits(wholeTable,wholeDense));
        float poisonValue; const uint32_t bits=0x7fc00abcu;
        std::memcpy(&poisonValue,&bits,sizeof(bits));
        const GfVec3f poison(poisonValue,poisonValue,-0.0f);
        bool guarded=true;
        for(const std::vector<size_t> &bounds:
                std::vector<std::vector<size_t>>{{0,1,333,334,700,1001},{0,1001}}) {
            std::vector<GfVec3f> byBasis(count),byDense(count),byTable(count);
            for(size_t k=0;k+1<bounds.size();++k) {
                const size_t b=bounds[k],e=bounds[k+1];
                // A guard point on each side of the group's seeded buffer.
                std::vector<GfVec3f> seeded(e-b+2,poison);
                std::copy(start.begin()+long(b),start.begin()+long(e),seeded.begin()+1);
                std::vector<GfVec3f> a=seeded,d=seeded,t=seeded;
                CHECK(RigExecApplyWireBasisGroup(a.data()+1,b,e,basis,indices,weights,rest,posed));
                CHECK(RigExecApplyWireGroup(d.data()+1,b,e,count,restCurve,posedCurve,
                    binds.data(),binds.size(),2.0));
                CHECK(RigExecApplyWireGroup(t.data()+1,b,e,count,restCurve,posedCurve,
                    binds.data(),binds.size(),2.0,evals.data(),evals.size()));
                for(const std::vector<GfVec3f> *g:{&a,&d,&t}) {
                    if(std::memcmp(&g->front(),&poison,sizeof(GfVec3f)) ||
                       std::memcmp(&g->back(),&poison,sizeof(GfVec3f))) guarded=false;
                }
                std::copy(a.begin()+1,a.end()-1,byBasis.begin()+long(b));
                std::copy(d.begin()+1,d.end()-1,byDense.begin()+long(b));
                std::copy(t.begin()+1,t.end()-1,byTable.begin()+long(b));
            }
            CHECK(SameBits(byBasis,wholeBasis) && SameBits(byDense,wholeDense) &&
                  SameBits(byTable,wholeDense));
        }
        CHECK(guarded);
        // Refused as the whole calls refuse, writing nothing: a short posed
        // polygon, a bind table for other points, a short rest table, and a
        // non-empty group with no buffer. An empty group needs none.
        std::vector<GfVec3f> shortPosed=posed; shortPosed.pop_back();
        const std::vector<GfVec3f> first(start.begin(),start.begin()+10);
        std::vector<GfVec3f> refused=first;
        CHECK(!RigExecApplyWireBasisGroup(refused.data(),0,10,basis,indices,weights,rest,
            shortPosed));
        CHECK(!RigExecApplyWireGroup(refused.data(),0,10,count+1,restCurve,posedCurve,
            binds.data(),binds.size(),2.0));
        CHECK(!RigExecApplyWireGroup(refused.data(),0,10,count,restCurve,posedCurve,
            binds.data(),binds.size(),2.0,evals.data(),evals.size()-1));
        CHECK(!RigExecApplyWireGroup(nullptr,0,10,count,restCurve,posedCurve,
            binds.data(),binds.size(),2.0));
        CHECK(!RigExecApplyWireBasisGroup(nullptr,0,10,basis,indices,weights,rest,posed));
        CHECK(SameBits(refused,first));
        CHECK(RigExecApplyWireGroup(nullptr,5,5,count,restCurve,posedCurve,
            binds.data(),binds.size(),2.0));
        CHECK(RigExecApplyWireBasisGroup(nullptr,5,5,basis,indices,weights,rest,posed));
    }

    // Point blocks: the copy-on-write protocol of one vertex group.
    {
        using State=RigExecGroupState<GfVec3f>;
        using Ref=RigExecPointsRef<GfVec3f>;
        using Buffer=std::vector<GfVec3f>;
        // Writes \p v's pattern into own buffer \p k.
        const auto fill=[](State *s,int k,float v) {
            for(GfVec3f &p:*s->own[k]) p=GfVec3f(v,-v,0.0f);
        };
        // A scratch is never a buffer someone else holds; a free buffer
        // nobody holds is preferred (no allocation), and a count change
        // resizes it.
        {
            State s;
            s.own[0]=std::make_shared<Buffer>(4);
            s.own[1]=std::make_shared<Buffer>(4);
            const GfVec3f *first=s.own[0]->data(),*second=s.own[1]->data();
            CHECK(RigExecGroupScratch(&s,4)==0 && s.own[0]->data()==first);
            const auto held=s.own[0];
            CHECK(RigExecGroupScratch(&s,4)==1 && s.own[1]->data()==second);
            const auto heldToo=s.own[1];
            const int k=RigExecGroupScratch(&s,4);
            CHECK(k==0 && s.own[k]!=held && s.own[k]!=heldToo &&
                  s.own[k].use_count()==1 && s.own[k]->size()==4);
            CHECK(held->data()==first && heldToo->data()==second && s.own[1]==heldToo);
            State r;
            r.own[0]=std::make_shared<Buffer>(4);
            CHECK(RigExecGroupScratch(&r,7)==0 && r.own[0]->size()==7);
            CHECK(RigExecGroupScratch(&r,2)==0 && r.own[0]->size()==2);
        }
        // Own publications: a first one bumps; equal bytes keep the published
        // ref; a signed zero is other bytes. A reset keeps the version, and
        // the next publication of the same bytes still bumps above it.
        {
            State g;
            int k=RigExecGroupScratch(&g,3);
            fill(&g,k,1.0f);
            CHECK(RigExecPublishOwnGroup(&g,k) && g.version==1 && g.ran && g.ownPublished==k);
            CHECK(g.published.data==g.own[k]->data() && g.published.count==3 &&
                  g.published.owner==g.own[k]);
            int m=RigExecGroupScratch(&g,3);
            CHECK(m!=k);
            fill(&g,m,1.0f);
            CHECK(!RigExecPublishOwnGroup(&g,m) && g.version==1 && g.ownPublished==k &&
                  g.published.data==g.own[k]->data());
            m=RigExecGroupScratch(&g,3);
            CHECK(m!=k);
            fill(&g,m,1.0f);
            (*g.own[m])[1][2]=-0.0f;
            CHECK(RigExecPublishOwnGroup(&g,m) && g.version==2 && g.ownPublished==m);
            const GfVec3f *stale=g.published.data;
            RigExecResetGroup(&g);
            CHECK(!g.ran && g.version==2 && g.ownPublished==-1 && g.ownComputed==-1 &&
                  g.own[0] && g.own[1]);
            // The buffer the stale ref still holds is not a scratch, though
            // no index names it published any more.
            const auto other=g.own[1-m];
            k=RigExecGroupScratch(&g,3);
            CHECK(k==1-m && g.own[k]!=other && g.own[m]->data()==stale);
            fill(&g,k,1.0f);
            (*g.own[k])[1][2]=-0.0f;
            CHECK(RigExecPublishOwnGroup(&g,k) && g.version==3 && g.ownPublished==k);
        }
        // Passed publications: the same slot inherits its version (no byte
        // compare), another slot is compared, and an own publication in
        // between forgets the source.
        {
            const auto a=std::make_shared<Buffer>(3,GfVec3f(2.0f,0.0f,1.0f));
            const auto b=std::make_shared<Buffer>(*a);
            const auto c=std::make_shared<Buffer>(3,GfVec3f(2.0f,-0.0f,1.0f));
            const Ref ra{a,a->data(),3},rb{b,b->data(),3},rc{c,c->data(),3};
            State s;
            CHECK(RigExecPublishPassedGroup(&s,ra,{5,3}) && s.version==1 &&
                  s.published.data==ra.data && s.ownPublished==-1 &&
                  (s.passedFrom==RigExecGroupSource{5,3}));
            CHECK(!RigExecPublishPassedGroup(&s,ra,{5,3}) && s.version==1);
            CHECK(RigExecPublishPassedGroup(&s,ra,{5,4}) && s.version==2);
            CHECK(!RigExecPublishPassedGroup(&s,rb,{-1,7}) && s.version==2 &&
                  s.published.data==rb.data && (s.passedFrom==RigExecGroupSource{-1,7}));
            CHECK(RigExecPublishPassedGroup(&s,rc,{9,1}) && s.version==3);
            s.own[0]=std::make_shared<Buffer>(3,GfVec3f(5.0f));
            CHECK(RigExecPublishOwnGroup(&s,0) && s.version==4 &&
                  s.passedFrom.slot==kRigExecNoGroupSource && s.ownPublished==0);
            CHECK(RigExecPublishPassedGroup(&s,rc,{9,1}) && s.version==5 &&
                  s.ownPublished==-1);
            State fresh;
            CHECK(RigExecPublishPassedGroup(&fresh,rb,{}) && fresh.version==1);
            CHECK(!RigExecPublishPassedGroup(&fresh,ra,{}) && fresh.version==1);
        }
        // Computed results: equal bytes keep the kept buffer; a fuse that
        // publishes the result shares it. With a computed result in one
        // buffer and another publication in the other (a chunk ran, its fuse
        // did not), the scratch forgets the result and hands its buffer
        // back, and the next result bumps.
        {
            State chunk;
            int k=RigExecGroupScratch(&chunk,2);
            fill(&chunk,k,3.0f);
            CHECK(RigExecNoteComputedGroup(&chunk,k) && chunk.computedVersion==1 &&
                  chunk.ownComputed==k && chunk.computedRan);
            int m=RigExecGroupScratch(&chunk,2);
            CHECK(m!=k);
            fill(&chunk,m,3.0f);
            CHECK(!RigExecNoteComputedGroup(&chunk,m) && chunk.computedVersion==1 &&
                  chunk.ownComputed==k);
            CHECK(RigExecPublishOwnGroup(&chunk,chunk.ownComputed) && chunk.ownPublished==k);
            m=RigExecGroupScratch(&chunk,2);
            CHECK(m!=k);
            fill(&chunk,m,4.0f);
            CHECK(RigExecNoteComputedGroup(&chunk,m) && chunk.computedVersion==2 &&
                  chunk.ownComputed==m && chunk.ownPublished==k);

            State f5;
            f5.own[0]=std::make_shared<Buffer>(2,GfVec3f(1.0f));
            f5.own[1]=std::make_shared<Buffer>(2,GfVec3f(1.0f));
            f5.ownComputed=0; f5.computedRan=true; f5.computedVersion=6;
            f5.ownPublished=1; f5.ran=true; f5.version=4;
            f5.published=Ref{f5.own[1],f5.own[1]->data(),2};
            const GfVec3f *kept=f5.own[0]->data();
            CHECK(RigExecGroupScratch(&f5,2)==0 && f5.own[0]->data()==kept);
            CHECK(f5.ownComputed==-1 && !f5.computedRan && f5.ownPublished==1);
            CHECK(RigExecNoteComputedGroup(&f5,0) && f5.computedVersion==7 &&
                  f5.ownComputed==0);
        }
        // A clone shares every buffer; dropping its spare leaves the source's
        // writer a unique scratch, and the clone allocates on its first write
        // without touching a buffer the source holds.
        {
            State live;
            live.own[0]=std::make_shared<Buffer>(2,GfVec3f(1.0f));
            live.own[1]=std::make_shared<Buffer>(2,GfVec3f(2.0f));
            CHECK(RigExecPublishOwnGroup(&live,0));
            const GfVec3f *spare=live.own[1]->data();
            State clone=live;
            RigExecDropGroupSpare(&clone);
            CHECK(clone.own[0]==live.own[0] && !clone.own[1] &&
                  clone.published.data==live.published.data);
            CHECK(RigExecGroupScratch(&live,2)==1 && live.own[1]->data()==spare);
            const int first=RigExecGroupScratch(&clone,2);
            CHECK(first==1 && clone.own[1] && clone.own[1]!=live.own[1] &&
                  clone.own[1].use_count()==1);
            fill(&clone,first,9.0f);
            CHECK(RigExecPublishOwnGroup(&clone,first) && clone.ownPublished==1);
            CHECK(RigExecGroupScratch(&clone,2)==0 && clone.own[0]!=live.own[0]);
            CHECK((*live.own[0])[0]==GfVec3f(1.0f) && live.published.data==live.own[0]->data());
            State speculative;
            speculative.own[0]=std::make_shared<Buffer>(2);
            speculative.own[1]=std::make_shared<Buffer>(2);
            speculative.ownComputed=1;
            RigExecDropGroupSpare(&speculative);
            CHECK(!speculative.own[0] && speculative.own[1]);
        }
        // Contiguous groups are slices of one owner at their bounds' offsets
        // (from bounds[0]); anything else is gathered, with the same bits.
        {
            const auto whole=std::make_shared<Buffer>(10);
            for(size_t i=0;i<10;++i) (*whole)[i]=GfVec3f(float(i),-0.0f,0.5f*float(i));
            const int bounds[5]={0,3,3,7,10};
            const Ref g0{whole,whole->data(),3},g1{whole,whole->data()+3,0},
                g2{whole,whole->data()+3,4},g3{whole,whole->data()+7,3};
            const GfVec3f *at=nullptr;
            const Ref *slices[4]={&g0,&g1,&g2,&g3};
            CHECK(RigExecGroupsContiguous(slices,bounds,4,&at) && at==whole->data());
            const auto copy=std::make_shared<Buffer>(whole->begin()+3,whole->begin()+7);
            const Ref moved{copy,copy->data(),4};
            const Ref *mixed[4]={&g0,&g1,&moved,&g3};
            CHECK(!RigExecGroupsContiguous(mixed,bounds,4,&at));
            Buffer gathered(10,GfVec3f(-1.0f));
            RigExecGatherGroups(mixed,bounds,4,gathered.data());
            CHECK(SameBits(gathered,*whole));
            const Ref shortGroup{whole,whole->data()+3,3};
            const Ref *wrong[4]={&g0,&g1,&shortGroup,&g3};
            CHECK(!RigExecGroupsContiguous(wrong,bounds,4,&at));
            const Ref shifted{whole,whole->data()+6,3};
            const Ref *gap[4]={&g0,&g1,&g2,&shifted};
            CHECK(!RigExecGroupsContiguous(gap,bounds,4,&at));
            const int tail[3]={3,7,10};
            const Ref *last[2]={&g2,&g3};
            CHECK(RigExecGroupsContiguous(last,tail,2,&at) && at==whole->data()+3);
            Buffer tailOut(7);
            RigExecGatherGroups(last,tail,2,tailOut.data());
            CHECK(SameBits(tailOut,Buffer(whole->begin()+3,whole->end())));
        }
        CHECK(RigExecPointsBitsEqual<GfVec3f>(nullptr,0,nullptr,0));
        const GfVec3f plus(0.0f),minus(-0.0f);
        CHECK(!RigExecPointsBitsEqual(&plus,1,&minus,1) && !RigExecPointsBitsEqual(&plus,1,&plus,0));
    }

    // The lattice rest version: on the nonzero version a bind was built
    // from, the cache skips only the rest-point compare (a mutated rest copy
    // a compare would reject is accepted); an unequal or unknown version
    // compares; a changed cage is rejected whatever the version; a shared
    // bind keeps each cache's version; dropping the bind clears it.
    {
        std::vector<GfVec3f> restCage;
        for(int c=0;c<2;++c) for(int b=0;b<4;++b) for(int a=0;a<3;++a)
            restCage.push_back(GfVec3f(float(a),1.5f*float(b),0.5f*float(c)));
        std::vector<GfVec3f> grown=restCage;
        grown.back()=GfVec3f(2,4.5f,0.75f);
        const std::vector<GfVec3f> rest={{0.3f,1.1f,0.2f},{0,0,0},{2,4.5f,0.5f},
            {-1,2,0.25f},{1.25f,0.5f,0.3f}};
        std::vector<GfVec3f> mutated=rest;
        mutated[1][0]=-0.0f;
        GfVec3f lo,size,grownLo,grownSize;
        CHECK(RigExecLatticeBindBox(restCage.data(),restCage.size(),&lo,&size));
        CHECK(RigExecLatticeBindBox(grown.data(),grown.size(),&grownLo,&grownSize));
        RigExecSurfaceKernelCache<GfVec3f,GfVec3d> cache;
        const RigExecLatticeBasis *basis=
            cache.LatticeBasis(rest.data(),rest.size(),lo,size,3,4,2,7);
        CHECK(basis && cache.LatticeBuilds()==1 && cache.LatticeRestVersion()==7);
        CHECK(cache.LatticeBasis(mutated.data(),mutated.size(),lo,size,3,4,2,7)==basis);
        CHECK(cache.LatticeBuilds()==1);
        CHECK(cache.LatticeBasis(mutated.data(),mutated.size(),lo,size,3,4,2,8));
        CHECK(cache.LatticeBuilds()==2 && cache.LatticeRestVersion()==8);
        CHECK(cache.LatticeBasis(rest.data(),rest.size(),grownLo,grownSize,3,4,2,8));
        CHECK(cache.LatticeBuilds()==3 && cache.LatticeRestVersion()==8);
        CHECK(cache.LatticeBasis(rest.data(),rest.size(),grownLo,grownSize,3,4,2,0));
        CHECK(cache.LatticeBuilds()==3);
        CHECK(cache.LatticeBasis(mutated.data(),mutated.size(),grownLo,grownSize,3,4,2,0));
        CHECK(cache.LatticeBuilds()==4 && cache.LatticeRestVersion()==0);
        RigExecSurfaceKernelCache<GfVec3f,GfVec3d> first,second;
        CHECK(first.LatticeBasis(rest.data(),rest.size(),lo,size,3,4,2,3));
        CHECK(second.LatticeBasis(rest.data(),rest.size(),lo,size,3,4,2,9));
        {
            RigExecLatticeBindSharing<GfVec3f> sharing;
            sharing.Offer(&first); sharing.Offer(&second);
        }
        CHECK(first.RetainedLatticeBind() &&
              second.RetainedLatticeBind()==first.RetainedLatticeBind());
        CHECK(first.LatticeRestVersion()==3 && second.LatticeRestVersion()==9);
        CHECK(second.LatticeBasis(mutated.data(),mutated.size(),lo,size,3,4,2,9)==
              &first.RetainedLatticeBind()->value);
        CHECK(second.LatticeBuilds()==1);
        second.SetLatticeBudget(0);
        CHECK(!second.LatticeBasis(rest.data(),rest.size(),lo,size,4,3,2,9));
        CHECK(!second.RetainedLatticeBind() && second.LatticeRestVersion()==0);
    }
    std::printf("SurfaceKernelCache: %d failures\n",failures);
    return failures?1:0;
}
