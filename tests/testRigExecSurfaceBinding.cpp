#include "rigExec/rigEvaluator.h"
#include "pxr/base/plug/registry.h"
#include "pxr/usd/usdGeom/mesh.h"
#include <cstdio>
#include <cmath>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;
static int failures=0;
#define CHECK(x) do {if(!(x)){std::printf("FAIL %d: %s\n",__LINE__,#x);++failures;}}while(0)
const char *fixture=R"USD(#usda 1.0
def RigExecRoot "Rig" {
 def RigExecControl "Driver" { }
 def Mesh "Cage" {
  point3f[] points = [(0,0,0),(1,0,0),(0,1,0)]
  int[] faceVertexCounts = [3]
  int[] faceVertexIndices = [0,1,2]
 }
 def Points "Target" {
  point3f[] points = [(0,0,0),(9,8,7)]
 }
 def Scope "Movers" {
  def RigExecMatrixMover "CagePose" (prepend apiSchemas = ["RigExecMoverAPI"]) {
   rel rigExec:moves = </Rig/Cage.points>
   rel rigExec:transform = </Rig/Driver>
  }
  def RigExecSurfaceBindingMover "Bind" (prepend apiSchemas = ["RigExecMoverAPI"]) {
   rel rigExec:moves = </Rig/Target.points>
   rel rigExec:surface = </Rig/Cage> (rigExecReadPhase = "final")
   int[] rigExec:vertices = [0]
   int[] rigExec:vertexOffsets = [0,1]
   int[] rigExec:polygonOffsets = [0,3]
   int[] rigExec:pointIndices = [0,1,2]
   float[] rigExec:barycentricWeights = [0.2,0.3,0.5]
   vector3f[] rigExec:offsets = [(0.1,0.2,0.4)]
   float[] rigExec:bindingWeights = [1]
  }
 }
}
)USD";
UsdStageRefPtr Stage() {
 auto s=UsdStage::CreateInMemory();CHECK(s->GetRootLayer()->ImportFromString(fixture));return s;
}
VtVec3fArray Evaluate(RigExecRigEvaluator &rig) {
 auto p=rig.Evaluate(UsdTimeCode::Default());CHECK(p.valid);
 auto i=p.movedProperties.find(SdfPath("/Rig/Target.points"));
 CHECK(i!=p.movedProperties.end());return i==p.movedProperties.end()?VtVec3fArray():i->second.Get<VtVec3fArray>();
}
void Near(const VtVec3fArray &p,GfVec3f expected) {
 CHECK(p.size()==2);if(p.size()!=2)return;
 if((p[0]-expected).GetLength()>2e-6f)std::printf("actual %g %g %g expected %g %g %g\n",p[0][0],p[0][1],p[0][2],expected[0],expected[1],expected[2]);
 CHECK((p[0]-expected).GetLength()<2e-6f);CHECK(p[1]==GfVec3f(9,8,7));
}
int main() {
 PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
 for(auto mode:{RigExecEvaluationMode::Dynamic,RigExecEvaluationMode::Baked}) {
  auto s=Stage();auto bind=s->GetPrimAtPath(SdfPath("/Rig/Movers/Bind"));
  RigExecRigEvaluator rig(s,SdfPath("/Rig"));rig.SetEvaluationMode(mode);
  std::vector<std::string> errors;CHECK(rig.Compile(&errors));
  for(const auto &e:errors)std::printf("%s\n",e.c_str());
  std::vector<std::string> reasons;CHECK(rig.IsBakeable(&reasons));
  std::string before,after;s->GetRootLayer()->ExportToString(&before);
  Near(Evaluate(rig),GfVec3f(.4,.7,.4));
  s->GetRootLayer()->ExportToString(&after);CHECK(before==after);
  // Offset scaling is distinct from surface attachment and the envelope.
  bind.GetAttribute(TfToken("rigExec:deltaMultiplier")).Set(0.f);
  Near(Evaluate(rig),GfVec3f(.3,.5,0));
  bind.GetAttribute(TfToken("rigExec:deltaMultiplier")).Set(-2.f);
  Near(Evaluate(rig),GfVec3f(.1,.1,-.8));
  bind.GetAttribute(TfToken("rigExec:deltaMultipliers")).Set(VtFloatArray{-.5f,99.f});
  Near(Evaluate(rig),GfVec3f(.4,.7,.4));
  bind.GetAttribute(TfToken("inputs:defaultWeight")).Set(.5f);
  Near(Evaluate(rig),GfVec3f(.2,.35,.2));
  bind.GetAttribute(TfToken("inputs:defaultWeight")).Set(1.f);
  bind.GetAttribute(TfToken("rigExec:deltaMultiplier")).Set(1.f);
  bind.GetAttribute(TfToken("rigExec:deltaMultipliers")).Set(VtFloatArray());
  auto driver=s->GetPrimAtPath(SdfPath("/Rig/Driver"));
  driver.GetAttribute(TfToken("avars:rz")).Set(90.0);
  Near(Evaluate(rig),GfVec3f(-.7,.4,.4));
  driver.GetAttribute(TfToken("avars:tx")).Set(2.0);
  Near(Evaluate(rig),GfVec3f(1.3,.4,.4));
  driver.GetAttribute(TfToken("avars:rz")).Set(0.0);
  driver.GetAttribute(TfToken("avars:tx")).Set(0.0);
  bind.GetAttribute(TfToken("rigExec:mask")).Set(VtFloatArray{.5f,0});
  Near(Evaluate(rig),GfVec3f(.2,.35,.2));
  bind.GetAttribute(TfToken("rigExec:mask")).Set(VtFloatArray());
  bind.GetAttribute(TfToken("rigExec:normalMode")).Set(TfToken("smooth"));
  Near(Evaluate(rig),GfVec3f(.4,.7,.4));
  // Shared vertices interpolate area-weighted normals across a crease.
  auto cage=s->GetPrimAtPath(SdfPath("/Rig/Cage"));
  cage.GetAttribute(TfToken("points")).Set(VtVec3fArray{{0,0,0},{1,0,0},{0,1,0},{0,0,1}});
  cage.GetAttribute(TfToken("faceVertexCounts")).Set(VtIntArray{3,3});
  cage.GetAttribute(TfToken("faceVertexIndices")).Set(VtIntArray{0,1,2,0,3,1});
  const float sn=std::sin(float(M_PI/8)),cs=std::cos(float(M_PI/8));
  Near(Evaluate(rig),GfVec3f(.4,.5+.2*cs+.4*sn,-.2*sn+.4*cs));
  bind.GetAttribute(TfToken("rigExec:normalMode")).Set(TfToken("geometric"));
  Near(Evaluate(rig),GfVec3f(.4,.7,.4));
  // Limit stencils for a linear triangular patch: analytic position and
  // derivatives. No closest-point search and no tessellation approximation.
  bind.GetAttribute(TfToken("rigExec:limitOffsets")).Set(VtIntArray{0,3});
  bind.GetAttribute(TfToken("rigExec:limitIndices")).Set(VtIntArray{0,1,2});
  bind.GetAttribute(TfToken("rigExec:limitWeights")).Set(VtFloatArray{.2f,.3f,.5f});
  bind.GetAttribute(TfToken("rigExec:limitDuWeights")).Set(VtFloatArray{-1,1,0});
  bind.GetAttribute(TfToken("rigExec:limitDvWeights")).Set(VtFloatArray{-1,0,1});
  bind.GetAttribute(TfToken("rigExec:surfaceMode")).Set(TfToken("limit"));
  Near(Evaluate(rig),GfVec3f(.4,.7,.4));
  driver.GetAttribute(TfToken("avars:rz")).Set(90.0);
  Near(Evaluate(rig),GfVec3f(-.7,.4,.4));
 }
 // A regular bicubic B-spline patch is a Catmull-Clark limit patch.
 // Analytic separable basis and derivatives provide an independent curved
 // limit-surface test, including all three vector offset components.
 for(auto mode:{RigExecEvaluationMode::Dynamic,RigExecEvaluationMode::Baked}) {
  auto s=Stage();auto cage=s->GetPrimAtPath(SdfPath("/Rig/Cage"));
  auto bind=s->GetPrimAtPath(SdfPath("/Rig/Movers/Bind"));
  VtVec3fArray points;VtIntArray indices;VtFloatArray weights,du,dv;
  const float w[4]={1.f/48,23.f/48,23.f/48,1.f/48};
  const float derivative[4]={-.125f,-.625f,.625f,.125f};
  for(int j=0;j<4;++j)for(int i=0;i<4;++i) {
   points.push_back(GfVec3f(i,j,(i-1)*(i-1)+(j-1)*(j-1)));
   indices.push_back(j*4+i);weights.push_back(w[i]*w[j]);
   du.push_back(derivative[i]*w[j]);dv.push_back(w[i]*derivative[j]);
  }
  cage.GetAttribute(TfToken("points")).Set(points);
  bind.GetAttribute(TfToken("rigExec:surfaceMode")).Set(TfToken("limit"));
  bind.GetAttribute(TfToken("rigExec:limitOffsets")).Set(VtIntArray{0,16});
  bind.GetAttribute(TfToken("rigExec:limitIndices")).Set(indices);
  bind.GetAttribute(TfToken("rigExec:limitWeights")).Set(weights);
  bind.GetAttribute(TfToken("rigExec:limitDuWeights")).Set(du);
  bind.GetAttribute(TfToken("rigExec:limitDvWeights")).Set(dv);
  RigExecRigEvaluator rig(s,SdfPath("/Rig"));rig.SetEvaluationMode(mode);
  std::vector<std::string> errors;CHECK(rig.Compile(&errors));CHECK(errors.empty());
  GfVec3f tangent(1,0,1),normal(-1,-1,1);tangent.Normalize();normal.Normalize();
  Near(Evaluate(rig),GfVec3f(1.5,1.5,7.f/6)+.1f*tangent+.2f*GfCross(normal,tangent)+.4f*normal);
 }
 for(const char *bad:{"pointIndices","vertexOffsets","polygonOffsets","vertices","limitOffsets"}) {
  auto s=Stage();auto p=s->GetPrimAtPath(SdfPath("/Rig/Movers/Bind"));
  p.GetAttribute(TfToken(std::string("rigExec:")+bad)).Set(VtIntArray{-1});
  if(std::string(bad)=="limitOffsets")p.GetAttribute(TfToken("rigExec:surfaceMode")).Set(TfToken("limit"));
  RigExecRigEvaluator rig(s,SdfPath("/Rig"));std::vector<std::string> errors;
  CHECK(rig.Compile(&errors));CHECK(!errors.empty());
  auto pose=rig.Evaluate(UsdTimeCode::Default());CHECK(pose.valid);
  CHECK(pose.movedProperties.count(SdfPath("/Rig/Target.points"))==0);
 }
 for(const auto &bad: {VtFloatArray{1.f}, VtFloatArray{NAN,1.f}}) {
  auto s=Stage();auto bind=s->GetPrimAtPath(SdfPath("/Rig/Movers/Bind"));
  bind.GetAttribute(TfToken("rigExec:deltaMultipliers")).Set(bad);
  RigExecRigEvaluator rig(s,SdfPath("/Rig"));std::vector<std::string> errors;
  CHECK(rig.Compile(&errors));CHECK(!errors.empty());
  CHECK(rig.Evaluate(UsdTimeCode::Default()).movedProperties.count(SdfPath("/Rig/Target.points"))==0);
 }
 return failures?1:0;
}
