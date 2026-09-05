#include "rigExecMath/curvenetAdjustments.h"
#include "pxr/base/gf/rotation.h"
#include <cmath>
#include <cstdio>
#include <limits>

using namespace rigExec;
static int failures=0;
#define CHECK(x) do { if (!(x)) { ++failures; std::printf("FAIL %d: %s\n",__LINE__,#x); } } while(0)
static bool Near(const GfVec3f &a,const GfVec3f &b) { return (a-b).GetLength()<1e-4; }

static void TestIntersectionAndParentedTangents() {
    // Three noncoplanar incident branches determine a full rigid orientation.
    const std::vector<GfVec3f> rest{
        {0,0,0},{1,0,0},{2,0,0},{3,0,0},
        {0,1,0},{0,2,0},{0,3,0}, {0,0,1},{0,0,2},{0,0,3}};
    const std::vector<int> indices{0,1,2,3, 0,4,5,6, 0,7,8,9};
    GfMatrix4d warp(1);
    warp.SetRotate(GfRotation(GfVec3d(0,0,1),90));
    warp.SetTranslateOnly(GfVec3d(5,6,7));
    std::vector<GfVec3f> preceding;
    for (const auto &p:rest) preceding.emplace_back(warp.Transform(GfVec3d(p)));
    RigExecCurvenetAdjustmentCommand knot;
    knot.pointIndex=0;
    knot.localTransform.SetTranslate(GfVec3d(1,0,0));
    auto posed=preceding;
    std::vector<GfMatrix4d> frames;
    CHECK(RigExecApplyCurvenetAdjustments(&posed,rest,indices,
        RigExecCurvenetBasis::Bezier,{knot},&frames));
    const GfVec3f delta(0,1,0);
    for (int i:{0,1,4,7}) CHECK(Near(posed[i],preceding[i]+delta));
    for (int i:{2,3,5,6,8,9}) CHECK(posed[i]==preceding[i]);
    CHECK(frames.size()==1 && Near(GfVec3f(frames[0].ExtractTranslation()),posed[0]));

    // A parent rotation rotates attached handles; a tangent child translates
    // along that parent's rotated axes, after the parent's adjustment.
    knot.localTransform.SetRotate(GfRotation(GfVec3d(0,0,1),90));
    RigExecCurvenetAdjustmentCommand tangent;
    tangent.pointIndex=1; tangent.parentCommand=0;
    tangent.localTransform.SetTranslate(GfVec3d(1,0,0));
    posed=preceding;
    CHECK(RigExecApplyCurvenetAdjustments(&posed,rest,indices,
        RigExecCurvenetBasis::Bezier,{knot,tangent},&frames));
    CHECK(Near(posed[1],preceding[0]+GfVec3f(-2,0,0)));
    CHECK(Near(GfVec3f(frames[1].ExtractTranslation()),posed[1]));

    // Edits to the rest pool rebuild the frame from the new neutral basis.
    auto changedRest=rest;
    for (auto &p:changedRest) p=GfVec3f(warp.TransformDir(GfVec3d(p)));
    knot.localTransform.SetTranslate(GfVec3d(1,0,0));
    posed=preceding;
    CHECK(RigExecApplyCurvenetAdjustments(&posed,changedRest,indices,
        RigExecCurvenetBasis::Bezier,{knot}));
    CHECK(Near(posed[0],preceding[0]+GfVec3f(1,0,0)));

    // Every invalid command fails atomically, even after a valid command.
    tangent.pointIndex=5; // handle belongs to the opposite knot
    posed=preceding;
    CHECK(!RigExecApplyCurvenetAdjustments(&posed,rest,indices,
        RigExecCurvenetBasis::Bezier,{knot,tangent}));
    CHECK(posed==preceding);
    knot.localTransform[3][0]=std::numeric_limits<double>::quiet_NaN();
    CHECK(!RigExecApplyCurvenetAdjustments(&posed,rest,indices,
        RigExecCurvenetBasis::Bezier,{knot}));
    CHECK(posed==preceding);
}

static void TestIsolatedCurveAndHalfTurn() {
    const std::vector<GfVec3f> rest{{0,0,0},{1,1,0},{2,1,0},{3,0,0}};
    const std::vector<int> indices{0,1,2,3};
    GfMatrix4d warp(1); warp.SetRotate(GfRotation(GfVec3d(1,0,0),180));
    std::vector<GfVec3f> posed;
    for (const auto &p:rest) posed.emplace_back(warp.Transform(GfVec3d(p)));
    const auto preceding=posed;
    RigExecCurvenetAdjustmentCommand cmd;
    cmd.pointIndex=0; cmd.includeTangents=false;
    cmd.localTransform.SetTranslate(GfVec3d(0,1,0));
    CHECK(RigExecApplyCurvenetAdjustments(&posed,rest,indices,
        RigExecCurvenetBasis::Bezier,{cmd}));
    CHECK(Near(posed[0],preceding[0]+GfVec3f(0,-1,0)));
    CHECK(posed[1]==preceding[1]);
}

static void TestTransportBetweenIntersections() {
    std::vector<GfVec3f> rest{{0,0,0},{1,0,0},{4,0,0},
        {0,2,0},{0,0,2},{4,2,0},{4,0,2}};
    std::vector<int> indices;
    const auto edge=[&](int a,int b) {
        const int handle=int(rest.size());
        rest.push_back(rest[a]+(rest[b]-rest[a])/3.0f);
        rest.push_back(rest[a]+(rest[b]-rest[a])*2.0f/3.0f);
        indices.insert(indices.end(),{a,handle,handle+1,b});
    };
    edge(0,1); edge(1,2); edge(0,3); edge(0,4); edge(2,5); edge(2,6);
    std::vector<GfVec3f> posed;
    for (const auto &p:rest) {
        const GfRotation rotation(GfVec3d(1,0,0),90.0*p[0]/4.0);
        posed.emplace_back(rotation.TransformDir(GfVec3d(p)));
    }
    RigExecCurvenetAdjustmentCommand cmd;
    cmd.pointIndex=1; cmd.includeTangents=false;
    cmd.localTransform.SetTranslate(GfVec3d(0,1,0));
    CHECK(RigExecApplyCurvenetAdjustments(&posed,rest,indices,
        RigExecCurvenetBasis::Bezier,{cmd}));
    // Distance 1 from the left and 3 from the right means 1/4 of the
    // right intersection's 90-degree rotation after parallel transport.
    const double angle=std::acos(-1.0)/8;
    CHECK(Near(posed[1],GfVec3f(1,float(std::cos(angle)),float(std::sin(angle)))));
}

static void TestCatmullRomEndpointIndices() {
    const std::vector<GfVec3f> rest{{-1,0,0},{0,0,0},{1,0,0},{2,0,0}};
    std::vector<GfVec3f> posed{{0,-1,0},{0,0,0},{0,1,0},{0,2,0}};
    const auto preceding=posed;
    RigExecCurvenetAdjustmentCommand cmd;
    cmd.pointIndex=1;
    cmd.localTransform.SetTranslate(GfVec3d(1,0,0));
    CHECK(RigExecApplyCurvenetAdjustments(&posed,rest,{0,1,2,3},
        RigExecCurvenetBasis::CatmullRom,{cmd}));
    CHECK(Near(posed[1],GfVec3f(0,1,0)));
    CHECK(posed[0]==preceding[0] && posed[2]==preceding[2] && posed[3]==preceding[3]);
}

int main() {
    TestIntersectionAndParentedTangents();
    TestIsolatedCurveAndHalfTurn();
    TestTransportBetweenIntersections();
    TestCatmullRomEndpointIndices();
    if (failures) return 1;
    std::puts("testRigExecCurvenetAdjustments: all tests passed");
    return 0;
}
