#include "curvenetAdjustments.h"
#include "pxr/base/gf/quatd.h"
#include "pxr/base/gf/rotation.h"

#include <algorithm>
#include <cmath>
#include <set>

namespace rigExec {
namespace {
constexpr double eps = 1e-12;
bool _Fail(std::string *error, const char *why) {
    if (error) *error = why;
    return false;
}
bool _Finite(const GfVec3f &p) {
    return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}
GfQuatd _Between(const GfVec3d &a, const GfVec3d &b) {
    if (a.GetLengthSq() < eps || b.GetLengthSq() < eps)
        return GfQuatd::GetIdentity();
    return GfRotation(a.GetNormalized(), b.GetNormalized()).GetQuat();
}
GfMatrix4d _Frame(const GfQuatd &q, const GfVec3f &p) {
    GfMatrix4d frame(1.0);
    frame.SetRotate(q);
    frame.SetTranslateOnly(GfVec3d(p));
    return frame;
}

// Horn's symmetric quaternion eigenproblem, solved with Jacobi rotations.
// Normalizing each tangent prevents a stretched incident branch dominating
// the orientation. The eigensolve handles half-turns without power-iteration
// seed ambiguity.
GfQuatd _BestFit(const std::vector<GfVec3d> &from,
                 const std::vector<GfVec3d> &to) {
    double b[3][3]{};
    int count = 0;
    size_t firstValid = 0;
    for (size_t i = 0; i < from.size(); ++i) {
        if (from[i].GetLengthSq() < eps || to[i].GetLengthSq() < eps) continue;
        const auto a = from[i].GetNormalized(), c = to[i].GetNormalized();
        if (!count) firstValid = i;
        for (int r = 0; r < 3; ++r)
            for (int s = 0; s < 3; ++s) b[r][s] += a[r] * c[s];
        ++count;
    }
    if (!count) return GfQuatd::GetIdentity();
    const double tr = b[0][0] + b[1][1] + b[2][2];
    double n[4][4] = {
        {tr, b[1][2]-b[2][1], b[2][0]-b[0][2], b[0][1]-b[1][0]},
        {b[1][2]-b[2][1], 2*b[0][0]-tr, b[0][1]+b[1][0], b[0][2]+b[2][0]},
        {b[2][0]-b[0][2], b[0][1]+b[1][0], 2*b[1][1]-tr, b[1][2]+b[2][1]},
        {b[0][1]-b[1][0], b[0][2]+b[2][0], b[1][2]+b[2][1], 2*b[2][2]-tr}};
    double v[4][4]{};
    for (int i = 0; i < 4; ++i) v[i][i] = 1;
    for (int sweep = 0; sweep < 32; ++sweep) {
        double residual = 0;
        for (int p = 0; p < 4; ++p) for (int q = p+1; q < 4; ++q) {
            residual += std::abs(n[p][q]);
            if (std::abs(n[p][q]) < 1e-15) continue;
            const double tau = (n[q][q]-n[p][p])/(2*n[p][q]);
            const double t = std::copysign(1.0, tau)/
                (std::abs(tau) + std::sqrt(1+tau*tau));
            const double c = 1/std::sqrt(1+t*t), s = t*c;
            const double off = n[p][q];
            n[p][p] -= t*off;
            n[q][q] += t*off;
            n[p][q] = n[q][p] = 0;
            for (int k = 0; k < 4; ++k) {
                if (k != p && k != q) {
                    const double x=n[k][p], y=n[k][q];
                    n[k][p]=n[p][k]=c*x-s*y;
                    n[k][q]=n[q][k]=s*x+c*y;
                }
                const double x=v[k][p], y=v[k][q];
                v[k][p]=c*x-s*y; v[k][q]=s*x+c*y;
            }
        }
        if (residual < 1e-13) break;
    }
    int largest = 0;
    for (int i = 1; i < 4; ++i) if (n[i][i] > n[largest][largest]) largest=i;
    GfQuatd result(v[0][largest], GfVec3d(v[1][largest],v[2][largest],v[3][largest]));
    result.Normalize();
    // A lone tangent leaves twist unspecified; use the shortest rotation.
    bool collinear = true;
    for (size_t i=0; i<from.size(); ++i)
        if (GfCross(from[firstValid],from[i]).GetLengthSq() > eps) collinear=false;
    return collinear ? _Between(from[firstValid],to[firstValid]) : result;
}

std::vector<GfVec3d> _Tangents(const RigExecCurvenetSampling &samples, size_t c) {
    const int begin=samples.curveBegin[c], count=samples.GetCurveSampleCount(c);
    std::vector<GfVec3d> tangents(count);
    for (int i=0; i<count; ++i) {
        tangents[i] = samples.positions[begin+std::min(i+1,count-1)] -
                      samples.positions[begin+std::max(i-1,0)];
    }
    return tangents;
}
std::vector<GfQuatd> _Transport(const std::vector<GfVec3d> &rest,
    const std::vector<GfVec3d> &posed, GfQuatd seed, bool reverse) {
    std::vector<GfQuatd> result(rest.size());
    int i=reverse ? int(rest.size())-1 : 0;
    const int step=reverse ? -1 : 1;
    result[i]=seed;
    for (int j=i+step; j>=0 && j<int(rest.size()); j+=step) {
        seed=(_Between(posed[i],posed[j])*seed*
              _Between(rest[i],rest[j]).GetInverse()).GetNormalized();
        result[j]=seed; i=j;
    }
    return result;
}
std::set<int> _Handles(const RigExecCurvenetTopology &topology, int knot) {
    std::set<int> handles;
    if (topology.basis != RigExecCurvenetBasis::Bezier) return handles;
    for (size_t s=0; s<topology.GetSplineCount(); ++s) {
        if (topology.GetSplineStartKnot(s)==knot) handles.insert(topology.splineIndices[4*s+1]);
        if (topology.GetSplineEndKnot(s)==knot) handles.insert(topology.splineIndices[4*s+2]);
    }
    return handles;
}
} // namespace

bool RigExecComputeCurvenetAdjustmentFrames(
    const RigExecCurvenetTopology &topology,
    const std::vector<GfVec3f> &rest, const std::vector<GfVec3f> &posed,
    std::vector<GfMatrix4d> *frames, std::string *error) {
    if (!frames || rest.size()!=posed.size() || rest.size()!=size_t(topology.pointCount))
        return _Fail(error,"curvenet adjustment point cardinality mismatch");
    for (size_t i=0; i<rest.size(); ++i)
        if (!_Finite(rest[i]) || !_Finite(posed[i])) return _Fail(error,"nonfinite curvenet point");
    const std::vector<int> density(topology.GetSplineCount(), 16);
    const auto a=RigExecSampleCurvenet(topology,rest,density);
    const auto b=RigExecSampleCurvenet(topology,posed,density);
    std::vector<std::vector<GfVec3d>> tangentsA(a.GetCurveCount()), tangentsB(b.GetCurveCount());
    for (size_t c=0; c<a.GetCurveCount(); ++c) {
        if (a.GetCurveSampleCount(c)<2 || a.GetCurveSampleCount(c)!=b.GetCurveSampleCount(c))
            return _Fail(error,"degenerate curvenet adjustment curve");
        tangentsA[c]=_Tangents(a,c); tangentsB[c]=_Tangents(b,c);
    }
    std::vector<GfQuatd> rotations(rest.size(),GfQuatd::GetIdentity());
    for (const auto &intersection:topology.intersections) {
        std::vector<GfVec3d> from,to;
        for (const auto &spoke:intersection.spokes) {
            if (topology.basis == RigExecCurvenetBasis::Bezier) {
                const auto &curve = topology.curves[spoke.curve];
                const int spline = spoke.atCurveEnd ? curve.splines.back() : curve.splines.front();
                const int handle = topology.splineIndices[4*spline +
                    (topology.GetSplineStartKnot(spline)==intersection.knot ? 1 : 2)];
                from.push_back(GfVec3d(rest[handle]-rest[intersection.knot]));
                to.push_back(GfVec3d(posed[handle]-posed[intersection.knot]));
                continue;
            }
            const auto &x=tangentsA[spoke.curve], &y=tangentsB[spoke.curve];
            from.push_back(spoke.atCurveEnd ? -x.back() : x.front());
            to.push_back(spoke.atCurveEnd ? -y.back() : y.front());
        }
        rotations[intersection.knot]=_BestFit(from,to);
    }
    for (size_t c=0; c<a.GetCurveCount(); ++c) {
        const auto &curve=topology.curves[c];
        const auto fallback=_BestFit(tangentsA[c],tangentsB[c]);
        const auto left=_Transport(tangentsA[c],tangentsB[c],
            curve.startIsIntersection ? rotations[curve.startKnot] : fallback,false);
        const auto right=_Transport(tangentsA[c],tangentsB[c],
            curve.endIsIntersection ? rotations[curve.endKnot] : fallback,true);
        std::vector<double> distance(left.size(),0.0);
        const int begin=b.curveBegin[c];
        for (size_t i=1; i<distance.size(); ++i)
            distance[i]=distance[i-1]+(b.positions[begin+i]-b.positions[begin+i-1]).GetLength();
        if (distance.back()<eps) return _Fail(error,"collapsed curvenet adjustment curve");
        for (size_t i=0; i<distance.size(); ++i) {
            const int knot=a.knotOfSample[begin+i];
            if (knot<0 || topology.knotKinds[knot]==RigExecCurvenetKnotKind::Intersection) continue;
            rotations[knot]=curve.startIsIntersection && curve.endIsIntersection
                ? GfSlerp(distance[i]/distance.back(),left[i],right[i]).GetNormalized()
                : (curve.endIsIntersection ? right[i] : left[i]);
        }
    }
    std::vector<GfMatrix4d> result(rest.size());
    for (size_t i=0; i<rest.size(); ++i) result[i]=_Frame(rotations[i],posed[i]);
    *frames=std::move(result);
    return true;
}

bool RigExecApplyCurvenetAdjustments(
    std::vector<GfVec3f> *points, const std::vector<GfVec3f> &rest,
    const std::vector<int> &indices, RigExecCurvenetBasis basis,
    const std::vector<RigExecCurvenetAdjustmentCommand> &commands,
    std::vector<GfMatrix4d> *commandFrames, std::string *error) {
    if (!points) return _Fail(error,"missing curvenet adjustment points");
    RigExecCurvenetTopology topology;
    if (!RigExecBuildCurvenetTopology(indices,rest.size(),basis,rest,nullptr,&topology,error)) return false;
    std::vector<GfMatrix4d> frames;
    if (!RigExecComputeCurvenetAdjustmentFrames(topology,rest,*points,&frames,error)) return false;
    auto output=*points;
    std::vector<GfMatrix4d> adjustedFrames;
    std::set<int> commanded;
    for (size_t i=0; i<commands.size(); ++i) {
        const auto &command=commands[i];
        const int index=command.pointIndex;
        if (index<0 || size_t(index)>=rest.size() || !commanded.insert(index).second)
            return _Fail(error,"invalid or duplicate adjustment point index");
        for (int r=0; r<4; ++r) for (int c=0; c<4; ++c)
            if (!std::isfinite(command.localTransform[r][c])) return _Fail(error,"nonfinite adjustment transform");
        if (command.localTransform[0][3]!=0 || command.localTransform[1][3]!=0 ||
            command.localTransform[2][3]!=0 || command.localTransform[3][3]!=1)
            return _Fail(error,"adjustment transform must be affine");
        GfMatrix4d frame=frames[index];
        std::set<int> affected{index};
        if (command.parentCommand>=0) {
            if (size_t(command.parentCommand)>=i || commands[command.parentCommand].parentCommand>=0 ||
                !_Handles(topology,commands[command.parentCommand].pointIndex).count(index))
                return _Fail(error,"tangent adjustment must follow its incident knot parent");
            frame=adjustedFrames[command.parentCommand];
            frame.SetTranslateOnly(GfVec3d(output[index]));
        } else {
            if (topology.knotValence[index]==0)
                return _Fail(error,"knot adjustment must name a curve endpoint");
            if (command.includeTangents) {
                const auto handles=_Handles(topology,index);
                affected.insert(handles.begin(),handles.end());
            }
        }
        const GfMatrix4d transform=frame.GetInverse()*command.localTransform*frame;
        for (const int point:affected) {
            if (command.localTransform==GfMatrix4d(1.0)) continue;
            output[point]=GfVec3f(transform.Transform(GfVec3d(output[point])));
            if (!_Finite(output[point])) return _Fail(error,"adjustment produced a nonfinite point");
        }
        adjustedFrames.push_back(command.localTransform*frame);
    }
    points->swap(output);
    if (commandFrames) *commandFrames=std::move(adjustedFrames);
    return true;
}
} // namespace rigExec
