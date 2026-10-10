#include "rigExec/rigEvaluator.h"
#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/latticeKernel.h"
#include "rigExecBake/bake.h"
#include "rigExecRuntime/runtime.h"
#include "pxr/base/plug/registry.h"
#include <cmath>
#include <cstdio>
#include <stdexcept>
using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE
#define CHECK(x) do {if(!(x))throw std::runtime_error(std::string("line ")+std::to_string(__LINE__)+": " #x);}while(0)
#include "rigExecFrozenParity.h"
#include "rigExecRuntimeDrive.h"
#include "fixtures/lattice/latticeReferenceCases.h"
static const SdfPath rigPath("/Rig"),targetPath("/Rig/Target.points");
static void Near(const std::vector<GfVec3f> &a,const std::vector<GfVec3f> &b) {
 CHECK(a.size()==b.size());for(size_t i=0;i<a.size();++i) {
  if((a[i]-b[i]).GetLength()>3e-6f)std::printf("point %zu got %.8g %.8g %.8g expected %.8g %.8g %.8g\n",i,a[i][0],a[i][1],a[i][2],b[i][0],b[i][1],b[i][2]);
  CHECK((a[i]-b[i]).GetLength()<3e-6f);
 }
}
static std::vector<GfVec3f> Points(const RigExecRigPose &p) {
 CHECK(p.valid);CHECK(p.movedProperties.count(targetPath));
 const auto &a=p.movedProperties.at(targetPath).Get<VtVec3fArray>();return {a.begin(),a.end()};
}
static UsdStageRefPtr Stage(const char *text) {
 auto s=UsdStage::CreateInMemory();CHECK(s->GetRootLayer()->ImportFromString(text));return s;
}
static void Compile(RigExecRigEvaluator &e) {
 std::vector<std::string> errors;CHECK(e.Compile(&errors));for(auto &x:errors)std::printf("%s\n",x.c_str());CHECK(errors.empty());
}
// The frame-cache worker reproduces live evaluation bit for bit.
static void Frozen(RigExecRigEvaluator &e,double frame) {
 std::string why;if(!RigExecFrozenMatchesLive(&e,rigPath,UsdTimeCode(frame),&why))throw std::runtime_error("frozen: "+why);
}
// Baked at the first frame and played through the file's inputs, each
// played frame publishes live evaluation's points.
static void Binary(RigExecRigEvaluator &e,const UsdStageRefPtr &stage) {
 RigExecBakeOpts opts;opts.time=1.;RigExecBakeResult result;std::string error;
 if(!RigExecBakeToBinary(e,opts,&result,&error))throw std::runtime_error(error);
 RigExecTestPlayer reader;
 if(!reader.Open(result.bytes,stage,&error))throw std::runtime_error(error);
 CHECK(reader->GetMissingExternalKernels().empty());
 for(double frame:{2.,1.,2.}) {
  if(!reader.Play(frame,&error))throw std::runtime_error(error);
  const auto expected=Points(e.Evaluate(UsdTimeCode(frame)));bool found=false;
  for(const auto &p:reader->GetPoints())if(p.path==targetPath.GetString()) {
   found=true;std::vector<GfVec3f> actual;for(const auto &v:p.points)actual.emplace_back(v[0],v[1],v[2]);Near(actual,expected);
  }
  CHECK(found);
 }
}
static void Numeric() {
 RigExecLatticeSettings s;s.regularGrid=true;s.origin={0,0,0};s.spacing={1,1,0};
 s.interpolation={RigExecLatticeInterpolation::Linear,RigExecLatticeInterpolation::Linear,RigExecLatticeInterpolation::Linear};
 const std::vector<GfVec3f> cage={{0,0,1},{1,0,2},{0,1,3},{1,1,4}};
 std::vector<GfVec3f> p={{.25,.5,0},{-4,.5,0},{4,.5,0}};
 CHECK((RigExecApplyLatticeGridKernel<GfVec3f,GfVec3d>(&p,cage,GfVec3i(2,2,1),s,GfMatrix4d(1),GfMatrix4d(1),GfMatrix4d(1))));
 Near(p,{{.25,.5,2.25},{-4,.5,2},{4,.5,3}});
 // Sequential grids bind at each incoming point, preserving point order.
 s.strength=.5;s.mask={.25f};p={{.25,.5,0}};
 CHECK((RigExecApplyLatticeGridKernel<GfVec3f,GfVec3d>(&p,cage,GfVec3i(2,2,1),s,GfMatrix4d(1),GfMatrix4d(1),GfMatrix4d(1))));Near(p,{{.25,.5,.28125}});
 const auto original=p;s.spacing[0]=0;
 CHECK(!(RigExecApplyLatticeGridKernel<GfVec3f,GfVec3d>(&p,cage,GfVec3i(2,2,1),s,GfMatrix4d(1),GfMatrix4d(1),GfMatrix4d(1))));CHECK(p==original);
}
int main() {
 try {
  PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);Numeric();
  for(const auto &c:latticeReferenceCases) {
   auto stage=Stage(c.stage);RigExecRigEvaluator e(stage,rigPath);Compile(e);
   std::string before,after;stage->GetRootLayer()->ExportToString(&before);
   Near(Points(e.Evaluate(UsdTimeCode(1))),c.expected);stage->GetRootLayer()->ExportToString(&after);CHECK(before==after);
   Frozen(e,1);Binary(e,stage);
  }
  {
   auto stage=Stage(latticeReferenceCases[0].stage);
   stage->DefinePrim(SdfPath("/Rig/CageFrame"),TfToken("RigExecControl"));
   stage->DefinePrim(SdfPath("/Rig/TargetFrame"),TfToken("RigExecControl"));
   auto frame=stage->GetPrimAtPath(SdfPath("/Rig/CageFrame"));
   CHECK(frame.GetAttribute(TfToken("avars:tx")).Set(.15,UsdTimeCode(1)));
   CHECK(frame.GetAttribute(TfToken("avars:tx")).Set(.3,UsdTimeCode(2)));
   auto mover=stage->GetPrimAtPath(SdfPath("/Rig/Lattice"));
   CHECK(mover.GetRelationship(TfToken("rigExec:frames")).SetTargets({SdfPath("/Rig/CageFrame"),SdfPath("/Rig/TargetFrame")}));
   CHECK(mover.GetRelationship(TfToken("rigExec:frames")).SetMetadata(TfToken("rigExecReadPhase"),VtValue(TfToken("final"))));
   RigExecRigEvaluator e(stage,rigPath);Compile(e);
   const auto a=Points(e.Evaluate(UsdTimeCode(1))),b=Points(e.Evaluate(UsdTimeCode(2)));CHECK(a!=b);
   Frozen(e,1);Frozen(e,2);Binary(e,stage);
  }
  // Regular grids permit a posed cage whose only values are time samples.
  auto stage=Stage(latticeReferenceCases[0].stage);auto cage=stage->GetAttributeAtPath(SdfPath("/Rig/Cage.points"));
  VtVec3fArray p;CHECK(cage.Get(&p));CHECK(cage.Clear());CHECK(cage.Set(p,UsdTimeCode(1)));
  RigExecRigEvaluator e(stage,rigPath);Compile(e);Near(Points(e.Evaluate(UsdTimeCode(1))),latticeReferenceCases[0].expected);
  Frozen(e,1);Binary(e,stage);
 } catch(const std::exception &e) {std::printf("FAIL %s\n",e.what());return 1;}
 return 0;
}
