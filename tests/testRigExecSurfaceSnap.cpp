#include "rigExec/rigEvaluator.h"
#include "rigExecMath/geometryKernels.h"
#include "rigExecBake/bake.h"
#include "rigExecRuntime/runtime.h"
#include "pxr/base/plug/registry.h"
#include <cmath>
#include <cstdio>
#include <stdexcept>
using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE
#define CHECK(x) do {if(!(x))throw std::runtime_error(std::string("line ")+std::to_string(__LINE__)+": " #x);}while(0)
#include "fixtures/surfaceSnap/nearestReferenceCases.h"
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
static void Binary(RigExecRigEvaluator &e) {
 e.SetEvaluationMode(RigExecEvaluationMode::Baked);
 RigExecBakeOpts opts;opts.frames={1.,2.};RigExecBakeResult result;std::string error;
 if(!RigExecBakeToBinary(e,opts,&result,&error))throw std::runtime_error(error);
 auto reader=RigExecRuntimeReader::Open(result.bytes.data(),result.bytes.size(),&error);CHECK(reader);
 CHECK(reader->GetMissingExternalKernels().empty());
 for(double frame:{2.,1.,2.}) {
  CHECK(reader->SetFrame(frame,&error));if(!reader->Execute(&error))throw std::runtime_error(error);
  const auto expected=Points(e.Evaluate(UsdTimeCode(frame)));bool found=false;
  for(const auto &p:reader->GetPoints())if(p.path==targetPath.GetString()) {
   found=true;std::vector<GfVec3f> actual;for(const auto &v:p.points)actual.emplace_back(v[0],v[1],v[2]);Near(actual,expected);
  }
  CHECK(found);
 }
}
static void Numeric() {
 const std::vector<GfVec3f> surface={{0,0,0},{1,0,0},{0,1,0}};
 const std::vector<GfVec3f> original={{.2,.2,.1},{.2,.2,-.02},{.2,.2,-.001},{.2,.2,0}};
 RigExecSurfaceSnapSettings s;s.mode=RigExecSurfaceSnapMode::Inside;s.offset=.003;
 auto result=original;
 CHECK((RigExecApplySurfaceSnapKernel<GfVec3f,GfVec3d>(&result,surface,{3},{0,1,2},s,GfMatrix4d(1),GfMatrix4d(1),GfMatrix4d(1))));
 Near(result,{{.2,.2,-.003},{.2,.2,-.02},{.2,.2,-.003},{.2,.2,-.003}});
 // Common points have already inherited the owner matrix; only the
 // driver's inverse defines the nearest-distance metric and clearance.
 GfMatrix4d matrix(1);matrix.SetScale(GfVec3d(2,.5,3));matrix.SetTranslateOnly(GfVec3d(4,2,-1));
 std::vector<GfVec3f> commonSurface;for(const auto &p:surface)commonSurface.emplace_back(matrix.TransformAffine(GfVec3d(p)));
 result={GfVec3f(matrix.TransformAffine(GfVec3d(original[0])))};
 CHECK((RigExecApplySurfaceSnapKernel<GfVec3f,GfVec3d>(&result,commonSurface,{3},{0,1,2},s,matrix.GetInverse(),matrix,matrix.GetInverse())));
 Near(result,{GfVec3f(matrix.TransformAffine(GfVec3d(.2,.2,-.003)))});
 s.mask={.5f};result={original[0]};
 CHECK((RigExecApplySurfaceSnapKernel<GfVec3f,GfVec3d>(&result,surface,{3},{0,1,2},s,GfMatrix4d(1),GfMatrix4d(1),GfMatrix4d(1))));
 Near(result,{{.2,.2,.0485}});
 s.mask={NAN};const auto before=result;
 CHECK(!(RigExecApplySurfaceSnapKernel<GfVec3f,GfVec3d>(&result,surface,{3},{0,1,2},s,GfMatrix4d(1),GfMatrix4d(1),GfMatrix4d(1))));CHECK(result==before);
 s.mask.clear();s.mode=RigExecSurfaceSnapMode::OnSurface;result={{2,0,0}};
 CHECK((RigExecApplySurfaceSnapKernel<GfVec3f,GfVec3d>(&result,surface,{3},{0,1,2},s,GfMatrix4d(1),GfMatrix4d(1),GfMatrix4d(1))));
 Near(result,{{1.003,0,0}}); // zero normal-dot uses the positive side
 GfMatrix4d computed(1);computed[3][3]=1+2e-16;
 CHECK(!RigExecSurfaceSnapValidMatrix(computed));CHECK(RigExecSurfaceSnapCanonicalComputedMatrix(&computed));CHECK(computed==GfMatrix4d(1));
 GfMatrix4d affine(-6.322167339137704e-10,.019656658172607415,4.064542741551313e-9,0,
     -1.6661504774526412e-9,-4.1840512292352225e-9,.019656658172606943,0,
     .01965665865321867,1.4303880183252863e-9,1.538209049330683e-9,0,
     .06024686975539988,-.004817602755219631,.8883842231592127,1);
 CHECK(RigExecSurfaceSnapValidMatrix(affine));const auto inverse=RigExecSurfaceSnapAffineInverse(affine);
 CHECK(RigExecSurfaceSnapValidMatrix(inverse));s.mode=RigExecSurfaceSnapMode::Inside;
 result={GfVec3f(affine.TransformAffine(GfVec3d(original[0])))};commonSurface.clear();
 for(const auto &p:surface)commonSurface.emplace_back(affine.TransformAffine(GfVec3d(p)));
 CHECK((RigExecApplySurfaceSnapKernel<GfVec3f,GfVec3d>(&result,commonSurface,{3},{0,1,2},s,inverse,affine,inverse)));
 Near(result,{GfVec3f(affine.TransformAffine(GfVec3d(.2,.2,-.003)))});
}
int main() {
 try {
  PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);Numeric();
  for(const auto &c:surfaceReferenceCases)for(auto mode:{RigExecEvaluationMode::Dynamic,RigExecEvaluationMode::Baked}) {
   auto stage=Stage(c.stage);RigExecRigEvaluator e(stage,rigPath);e.SetEvaluationMode(mode);Compile(e);
   std::string before,after;stage->GetRootLayer()->ExportToString(&before);
   Near(Points(e.Evaluate(UsdTimeCode(1))),c.expected);stage->GetRootLayer()->ExportToString(&after);CHECK(before==after);
   // Animated clearance reaches both captured binary frames and repeated playback.
   const auto mover=stage->GetPrimAtPath(SdfPath("/Rig/Snap"));
   mover.GetAttribute(TfToken("rigExec:offset")).Set(.08f,UsdTimeCode(1));
   mover.GetAttribute(TfToken("rigExec:offset")).Set(.04f,UsdTimeCode(2));Binary(e);
  }
  for(int invalid=0;invalid<3;++invalid) {
   auto stage=Stage(surfaceReferenceCases[0].stage);auto mover=stage->GetPrimAtPath(SdfPath("/Rig/Snap"));
   GfMatrix4d matrix(1);if(invalid==0)matrix[0][3]=.5;if(invalid==1)matrix[0][0]=NAN;if(invalid==2)matrix[0][0]=0;
   CHECK(mover.GetAttribute(TfToken("rigExec:surfaceMatrix")).Set(matrix));
   RigExecRigEvaluator e(stage,rigPath);std::vector<std::string> errors;
   const bool compiled=e.Compile(&errors);CHECK(!errors.empty());
   if(compiled)CHECK(e.Evaluate(UsdTimeCode::Default()).movedProperties.count(targetPath)==0);
  }
  for(bool degenerateDefault:{false,true})for(auto mode:{RigExecEvaluationMode::Dynamic,RigExecEvaluationMode::Baked}) {
   auto stage=Stage(surfaceReferenceCases[0].stage);
   for(const SdfPath &path:{SdfPath("/Rig/Surface.points"),targetPath}) {
    auto a=stage->GetAttributeAtPath(path);VtVec3fArray p;CHECK(a.Get(&p));CHECK(a.Clear());
    CHECK(a.Set(p,UsdTimeCode(1)));
    if(degenerateDefault)CHECK(a.Set(path==targetPath?p:VtVec3fArray(p.size(),GfVec3f(0))));
   }
   RigExecRigEvaluator e(stage,rigPath);e.SetEvaluationMode(mode);Compile(e);
   Near(Points(e.Evaluate(UsdTimeCode(1))),surfaceReferenceCases[0].expected);
  }
  // Explicit triangles are sufficient without a duplicate polygon topology.
  for(auto mode:{RigExecEvaluationMode::Dynamic,RigExecEvaluationMode::Baked}) {
   auto stage=Stage(surfaceReferenceCases[0].stage);auto surface=stage->GetPrimAtPath(SdfPath("/Rig/Surface"));
   VtIntArray counts,indices,triangles;surface.GetAttribute(TfToken("faceVertexCounts")).Get(&counts);
   surface.GetAttribute(TfToken("faceVertexIndices")).Get(&indices);size_t cursor=0;
   for(int n:counts) {for(int j=1;j+1<n;++j) {triangles.push_back(indices[cursor]);triangles.push_back(indices[cursor+j]);triangles.push_back(indices[cursor+j+1]);}cursor+=n;}
   CHECK(surface.GetAttribute(TfToken("faceVertexCounts")).Clear());CHECK(surface.GetAttribute(TfToken("faceVertexIndices")).Clear());
   CHECK(stage->GetPrimAtPath(SdfPath("/Rig/Snap")).GetAttribute(TfToken("rigExec:triangles")).Set(triangles));
   RigExecRigEvaluator e(stage,rigPath);e.SetEvaluationMode(mode);Compile(e);Near(Points(e.Evaluate(UsdTimeCode(1))),surfaceReferenceCases[0].expected);Binary(e);
  }
 } catch(const std::exception &e) {std::printf("FAIL %s\n",e.what());return 1;}
 return 0;
}
