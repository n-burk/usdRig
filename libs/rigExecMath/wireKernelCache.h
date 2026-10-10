#ifndef RIGEXEC_MATH_WIRE_KERNEL_CACHE_H
#define RIGEXEC_MATH_WIRE_KERNEL_CACHE_H
#include <cstring>
#include <algorithm>
#include <memory>
#include <vector>
namespace rigExec {
/// Revision-owned rest evaluations, using the original curve evaluator.
/// Copies share immutable entries; each owner replaces its own pointer.
template<class Point,class Bind> class RigExecWireRestCache {
    struct Entry { std::vector<Point> points,evaluated; std::vector<Bind> binds;
        std::vector<double> knots; int order=0; double dropoff=0; };
    std::shared_ptr<const Entry> entry;
    template<class V> static bool SameVectors(const std::vector<V> &a,
        const V *b,size_t n,int axes) {
        if(a.size()!=n || (n && !b)) return false;
        for(size_t i=0;i<n;++i) for(int axis=0;axis<axes;++axis) {
            const auto x=a[i][axis],y=b[i][axis];
            if(std::memcmp(&x,&y,sizeof(x))) return false;
        }
        return true;
    }
public:
    template<class Curve> const std::vector<Point> *Get(const Curve &curve,
        const Bind *binds,size_t count,double dropoff=0.0) {
        if(!curve.IsValid() || (count && !binds)) return nullptr;
        bool knotsSame=entry && entry->knots.size()==curve.knots->size();
        if(knotsSame) for(size_t i=0;i<entry->knots.size();++i) {
            const double a=entry->knots[i],b=(*curve.knots)[i];
            if(std::memcmp(&a,&b,sizeof(a))) { knotsSame=false; break; }
        }
        if(entry && entry->order==curve.order && knotsSame &&
            !std::memcmp(&entry->dropoff,&dropoff,sizeof(dropoff)) &&
            SameVectors(entry->points,curve.points->data(),curve.points->size(),3) &&
            SameVectors(entry->binds,binds,count,2)) return &entry->evaluated;
        auto built=std::make_shared<Entry>(); built->points=*curve.points;
        built->knots=*curve.knots; built->order=curve.order; built->dropoff=dropoff;
        if(count) built->binds.assign(binds,binds+count);
        built->evaluated.resize(count);
        for(size_t i=0;i<count;++i) {
            double f=1.0;
            if(dropoff>0.0) {
                const double x=std::min(std::max(double(binds[i][1])/dropoff,0.0),1.0);
                f=1.0-x*x*(3.0-2.0*x);
            }
            if(f<=0.0) continue; // Preserve the old skip before curve evaluation.
            built->evaluated[i]=curve.Evaluate(double(binds[i][0]));
        }
        entry=built; return &entry->evaluated;
    }
};
}
#endif
