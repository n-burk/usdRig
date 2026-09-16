//
// TouchPose, below the app: the pick geometry, the generated shader, and the
// scene index that carries the highlight to Hydra.
//
//   1. THE CAST. The BVH answer must equal the brute-force answer for every
//      ray -- at rest, after a pose that only refits, after a pose so large
//      it forces a rebuild, and through a non-identity mesh transform.
//   2. MARQUEE and BRUSH on a cube whose answers are known by hand.
//   3. THE C SURFACE composes the highlight table in the documented order
//      (edit < selected < lead < hover) and reports no change for a state
//      that is already drawn -- which is what makes a hover inside one
//      region free.
//   4. THE SHADER: UsdPreviewSurface wraps, the wrapper parses as glslfx and
//      as an Sdr node with every UsdPreviewSurface input, and its source
//      renames the base terminal and reads both primvars.
//   5. THE SCENE INDEX: attaching publishes both primvars with the right
//      interpolation, wraps exactly the materials bound to the mesh and its
//      GeomSubsets, and dirties the right locators; a table change dirties
//      ONLY the table's value; detaching puts everything back.
//
// No GL: nothing here draws. The drawn result is asserted in usdview by
// tests/testUsdviewTouchPose.py.
//
#include "rigExecImaging/touchPose.h"
#include "rigExecImaging/touchPoseHighlight.h"
#include "rigExecImaging/touchPoseMesh.h"

#include "pxr/base/gf/frustum.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/tf/token.h"
#include "pxr/imaging/hd/materialBindingSchema.h"
#include "pxr/imaging/hd/materialBindingsSchema.h"
#include "pxr/imaging/hd/materialConnectionSchema.h"
#include "pxr/imaging/hd/materialNetworkSchema.h"
#include "pxr/imaging/hd/materialNodeParameterSchema.h"
#include "pxr/imaging/hd/materialNodeSchema.h"
#include "pxr/imaging/hd/materialSchema.h"
#include "pxr/imaging/hd/primvarSchema.h"
#include "pxr/imaging/hd/primvarsSchema.h"
#include "pxr/imaging/hd/retainedDataSource.h"
#include "pxr/imaging/hd/retainedSceneIndex.h"
#include "pxr/imaging/hd/sceneIndexObserver.h"
#include "pxr/imaging/hd/tokens.h"
#include "pxr/imaging/hio/glslfx.h"
#include "pxr/usd/sdr/registry.h"
#include "pxr/usd/sdr/shaderNode.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <sstream>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;

namespace {

int _failures = 0;

void _Check(bool condition, const std::string &message)
{
    if (!condition) {
        std::printf("FAIL: %s\n", message.c_str());
        ++_failures;
    }
}

// A lumpy closed surface: a UV sphere of quads with triangle caps, radius
// jittered per vertex so no two faces are coplanar and rays hit genuinely
// different depths.
struct Sphere {
    VtIntArray counts;
    VtIntArray indices;
    VtVec3fArray points;
};

Sphere _MakeSphere(int rings, int segments, std::mt19937 *rng)
{
    Sphere s;
    std::uniform_real_distribution<float> jitter(0.9f, 1.1f);
    s.points.push_back(GfVec3f(0, 1, 0));
    for (int r = 1; r < rings; ++r) {
        const double phi = M_PI * r / rings;
        for (int k = 0; k < segments; ++k) {
            const double theta = 2.0 * M_PI * k / segments;
            const float radius = jitter(*rng);
            s.points.push_back(GfVec3f(
                float(radius * std::sin(phi) * std::cos(theta)),
                float(radius * std::cos(phi)),
                float(radius * std::sin(phi) * std::sin(theta))));
        }
    }
    s.points.push_back(GfVec3f(0, -1, 0));
    const int south = int(s.points.size()) - 1;
    auto ring = [&](int r, int k) { return 1 + (r - 1) * segments + (k % segments); };
    for (int k = 0; k < segments; ++k) {
        s.counts.push_back(3);
        s.indices.push_back(0);
        s.indices.push_back(ring(1, k + 1));
        s.indices.push_back(ring(1, k));
    }
    for (int r = 1; r + 1 < rings; ++r) {
        for (int k = 0; k < segments; ++k) {
            s.counts.push_back(4);
            s.indices.push_back(ring(r, k));
            s.indices.push_back(ring(r, k + 1));
            s.indices.push_back(ring(r + 1, k + 1));
            s.indices.push_back(ring(r + 1, k));
        }
    }
    for (int k = 0; k < segments; ++k) {
        s.counts.push_back(3);
        s.indices.push_back(south);
        s.indices.push_back(ring(rings - 1, k));
        s.indices.push_back(ring(rings - 1, k + 1));
    }
    return s;
}

int _CompareCasts(const RigExecTouchPoseMesh &mesh, std::mt19937 *rng,
                  int rays, double *bvhMs, double *bruteMs)
{
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    GfVec3d lo, hi;
    mesh.GetWorldBounds(&lo, &hi);
    const GfVec3d center = 0.5 * (lo + hi);
    const double size = (hi - lo).GetLength();
    std::vector<GfVec3d> origins, targets;
    for (int i = 0; i < rays; ++i) {
        GfVec3d dir(unit(*rng), unit(*rng), unit(*rng));
        if (dir.GetLength() < 1e-3) {
            dir = GfVec3d(1, 0, 0);
        }
        dir.Normalize();
        origins.push_back(center + dir * size * 2.0);
        targets.push_back(center + GfVec3d(unit(*rng), unit(*rng), unit(*rng)) *
                                       size * 0.4);
    }
    int mismatches = 0;
    int hits = 0;
    std::vector<int> bvhFaces(rays), bruteFaces(rays);
    std::vector<double> bvhT(rays), bruteT(rays);
    mesh.Prepare();
    auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < rays; ++i) {
        bvhFaces[i] = mesh.Cast(origins[i], targets[i] - origins[i], &bvhT[i]);
    }
    auto t1 = std::chrono::steady_clock::now();
    for (int i = 0; i < rays; ++i) {
        bruteFaces[i] = mesh.CastBruteForce(origins[i], targets[i] - origins[i],
                                            &bruteT[i]);
    }
    auto t2 = std::chrono::steady_clock::now();
    for (int i = 0; i < rays; ++i) {
        hits += bruteFaces[i] >= 0;
        // Two faces can tie at a shared edge; the DISTANCE is what has to
        // agree, and the face whenever the distances are not a tie.
        const bool same = bvhFaces[i] == bruteFaces[i] ||
                          (bvhFaces[i] >= 0 && bruteFaces[i] >= 0 &&
                           std::fabs(bvhT[i] - bruteT[i]) < 1e-6);
        mismatches += same ? 0 : 1;
    }
    *bvhMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    *bruteMs = std::chrono::duration<double, std::milli>(t2 - t1).count();
    std::printf("cast: %d of %d rays hit\n", hits, rays);
    _Check(hits > rays / 10, "enough rays hit the mesh to mean something");
    return mismatches;
}

void TestCast()
{
    std::mt19937 rng(7);
    Sphere s = _MakeSphere(120, 220, &rng);
    RigExecTouchPoseMesh mesh;
    _Check(mesh.SetTopology(s.counts, s.indices, s.points.size()),
           "topology accepted");
    _Check(mesh.SetPoints(s.points), "points accepted");
    std::printf("cast: %zu faces, %zu triangles\n", mesh.GetFaceCount(),
                mesh.GetTriangleCount());

    double bvh, brute;
    int bad = _CompareCasts(mesh, &rng, 2000, &bvh, &brute);
    _Check(bad == 0, "rest: BVH == brute force (" + std::to_string(bad) +
                         " mismatches)");
    std::printf("cast: 2000 rays %.2f ms BVH vs %.2f ms brute (%.4f ms/ray, "
                "%.0fx)\n", bvh, brute, bvh / 2000.0, brute / std::max(bvh, 1e-6));
    _Check(mesh.GetBuildCount() == 1, "built once");

    // A small pose: refit only.
    VtVec3fArray posed = s.points;
    for (size_t i = 0; i < posed.size(); ++i) {
        posed[i] += GfVec3f(0.02f * std::sin(float(i)), 0.05f, 0.0f);
    }
    mesh.SetPoints(posed);
    auto t0 = std::chrono::steady_clock::now();
    mesh.Prepare();
    const double refitMs = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
    bad = _CompareCasts(mesh, &rng, 2000, &bvh, &brute);
    _Check(bad == 0, "refit: BVH == brute force (" + std::to_string(bad) + ")");
    _Check(mesh.GetRefitCount() >= 1, "the small pose refit");
    std::printf("cast: refit %.3f ms for %zu triangles\n", refitMs,
                mesh.GetTriangleCount());

    // A violent pose: every point swaps hemispheres. Correct either way; the
    // tree degrades enough that it should rebuild.
    VtVec3fArray scrambled = s.points;
    for (size_t i = 0; i < scrambled.size(); ++i) {
        scrambled[i] = -scrambled[i] * (1.0f + 0.5f * float(i % 7));
    }
    mesh.SetPoints(scrambled);
    bad = _CompareCasts(mesh, &rng, 1000, &bvh, &brute);
    _Check(bad == 0, "scrambled: BVH == brute force (" + std::to_string(bad) + ")");

    // Through a transform.
    GfMatrix4d xform = GfMatrix4d().SetRotate(
        GfRotation(GfVec3d(0.3, 1.0, 0.2), 37.0));
    xform *= GfMatrix4d().SetScale(GfVec3d(2.0, 0.5, 1.5));
    xform *= GfMatrix4d().SetTranslate(GfVec3d(10.0, -4.0, 3.0));
    mesh.SetPoints(s.points);
    mesh.SetTransform(xform);
    bad = _CompareCasts(mesh, &rng, 2000, &bvh, &brute);
    _Check(bad == 0, "transformed: BVH == brute force (" + std::to_string(bad) + ")");

    // And a known hit through the transform: straight down onto the north
    // pole, which the transform put somewhere else entirely.
    const GfVec3d pole = xform.Transform(GfVec3d(0, 1, 0));
    const GfVec3d up = xform.TransformDir(GfVec3d(0, 1, 0)).GetNormalized();
    double t = -1;
    const int face = mesh.Cast(pole + up * 5.0, -up, &t);
    _Check(face >= 0 && face < 220, "the pole ray lands on a north cap face: " +
                                        std::to_string(face));
    _Check(std::fabs(t - 5.0) < 1e-3, "at the pole's world distance: " +
                                          std::to_string(t));
    _Check(mesh.Cast(pole + up * 5.0, up, &t) == -1, "and away from it misses");
}

// Unit cube, faces: 0 +z, 1 -z, 2 +x, 3 -x, 4 +y, 5 -y (outward winding).
void _Cube(VtIntArray *counts, VtIntArray *indices, VtVec3fArray *points)
{
    *points = VtVec3fArray{
        {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
        {-1, -1, 1},  {1, -1, 1},  {1, 1, 1},  {-1, 1, 1}};
    *counts = VtIntArray(6, 4);
    *indices = VtIntArray{
        4, 5, 6, 7,     // +z
        0, 3, 2, 1,     // -z
        1, 2, 6, 5,     // +x
        0, 4, 7, 3,     // -x
        3, 7, 6, 2,     // +y
        0, 1, 5, 4};    // -y
}

void TestMarqueeAndBrush()
{
    VtIntArray counts, indices;
    VtVec3fArray points;
    _Cube(&counts, &indices, &points);
    RigExecTouchPoseMesh mesh;
    mesh.SetTopology(counts, indices, points.size());
    mesh.SetPoints(points);
    const int32_t regions[6] = {0, 1, 2, 2, -1, -1};
    mesh.SetFaceRegions(regions, 6);

    // Camera at z = +10 looking down -z, 90 degree fov.
    GfMatrix4d view = GfMatrix4d().SetTranslate(GfVec3d(0, 0, -10));
    GfFrustum frustum;
    frustum.SetPerspective(90.0, 1.0, 1.0, 100.0);
    GfMatrix4d vp = view * frustum.ComputeProjectionMatrix();
    const GfVec3d eye(0, 0, 10);

    std::vector<int> all = mesh.RegionsInRect(vp, 200, 200, eye, 0, 0, 200, 200);
    // +z faces the eye (region 0); +x and -x are edge-on-ish but their
    // centroids are at x=+-1 and their normals are perpendicular to the view
    // from their own centroid... only +z is strictly front-facing from there.
    _Check(all.size() == 1 && all[0] == 0,
           "the whole frame catches only the front-facing region");
    _Check(mesh.RegionsInRect(vp, 200, 200, eye, 0, 0, 5, 5).empty(),
           "a corner band catches nothing");

    // Brush: around +z face centroid with a direction looking down -z.
    std::vector<int> faces = mesh.Brush(GfVec3d(0, 0, 1), GfVec3d(0, 0, -1), 0.5);
    _Check(faces.size() == 1 && faces[0] == 0, "brush catches the face it is on");
    faces = mesh.Brush(GfVec3d(0, 0, 0), GfVec3d(0, 0, 0), 5.0);
    _Check(faces.size() == 6, "a huge undirected brush catches all six");
    faces = mesh.Brush(GfVec3d(0, 0, 0), GfVec3d(0, 0, -1), 5.0);
    _Check(faces.size() == 1 && faces[0] == 0,
           "a directed brush drops the faces pointing away");
}

void TestCApi()
{
    VtIntArray counts, indices;
    VtVec3fArray points;
    _Cube(&counts, &indices, &points);
    // No mesh path: pick-only, the highlight never attaches.
    const long long handle = RigExecTouchPose_OpenTopology(
        counts.cdata(), int(counts.size()), indices.cdata(),
        int(indices.size()), int(points.size()), nullptr);
    _Check(handle > 0, "handle opened");
    _Check(RigExecTouchPose_SetPoints(handle, points.cdata()->data(),
                                      int(points.size())) == 0, "points set");
    const int32_t regionOf[6] = {0, 1, 2, 2, -1, -1};
    RigExecTouchPose_SetFaceRegions(handle, regionOf, 6, 3);
    const float hover[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    const float edit[9] = {.5f, .5f, 0, 0, .5f, .5f, .5f, 0, .5f};
    RigExecTouchPose_SetRegionColors(handle, hover, edit, 3);

    const double origin[3] = {0, 0, 10}, down[3] = {0, 0, -1};
    double t;
    _Check(RigExecTouchPose_Cast(handle, origin, down, &t) == 0 &&
               std::fabs(t - 9.0) < 1e-6,
           "the C cast hits the +z face at distance 9");

    const float lead[3] = {0, 1, 0}, selected[3] = {.3f, .3f, .3f};
    const int32_t sel[2] = {1, 2};
    int changed = RigExecTouchPose_SetHighlightState(
        handle, 0, 2, sel, 2, 0, 0.85f, lead, selected);
    _Check(changed == 1, "a new state changes the table");
    float table[16];
    _Check(RigExecTouchPose_GetHighlightTable(handle, table, 4) == 4,
           "table has regionCount + 1 rows");
    _Check(table[3] == 0.0f, "row 0 is unlit");
    _Check(table[4] == 1 && table[5] == 0 && table[7] == 0.85f,
           "hovered region 0 is its hover colour at the opacity");
    _Check(std::fabs(table[8] - 0.3f) < 1e-6, "region 1 is selected grey");
    _Check(table[12] == 0 && table[13] == 1 && table[14] == 0,
           "region 2 is BOTH selected and lead, and lead wins");
    _Check(RigExecTouchPose_SetHighlightState(
               handle, 0, 2, sel, 2, 0, 0.85f, lead, selected) == 0,
           "the same state again changes nothing");
    RigExecTouchPose_SetHighlightState(handle, 1, -1, nullptr, 0, 1, 0.5f,
                                       lead, selected);
    RigExecTouchPose_GetHighlightTable(handle, table, 4);
    _Check(table[4] == .5f && table[7] == .5f,
           "paint mode lights every region in its edit colour");
    _Check(table[8] == 0 && table[9] == .5f && table[10] == .5f,
           "and the hovered one in its EDIT colour, not its palette colour");

    int32_t faces[8];
    const double center[3] = {0, 0, 0};
    _Check(RigExecTouchPose_Brush(handle, center, nullptr, 5.0, faces, 8) == 6,
           "the C brush");
    RigExecTouchPose_Close(handle);
    _Check(RigExecTouchPose_GetFaceCount(handle) == -1, "closed handles are gone");
}

void TestShader()
{
    const TfToken base("UsdPreviewSurface");
    const TfToken wrapped = RigExecTouchPoseGetWrappedTerminal(base);
    _Check(!wrapped.IsEmpty(), "UsdPreviewSurface can be wrapped");
    if (wrapped.IsEmpty()) {
        return;
    }
    const std::string source = RigExecTouchPoseGetWrappedSource(base);
    std::istringstream stream(source);
    HioGlslfx glslfx(stream);
    std::string reason;
    _Check(glslfx.IsValid(&reason), "the wrapper is valid glslfx: " + reason);
    const std::string surface = glslfx.GetSurfaceSource();
    _Check(surface.find("#define surfaceShader rigExecTouchPose_BaseSurfaceShader")
               != std::string::npos, "the base terminal is renamed");
    _Check(surface.find("HdGet_rigExecTouchTable(slot)") != std::string::npos &&
               surface.find("HdGet_rigExecTouchRegion()") != std::string::npos,
           "the wrapper reads both primvars");
    _Check(surface.find("evaluateLights") != std::string::npos,
           "and carries the base's own lighting code");

    const SdrShaderNodeConstPtr baseNode =
        SdrRegistry::GetInstance().GetShaderNodeByIdentifierAndType(
            base, TfToken("glslfx"));
    const SdrShaderNodeConstPtr node =
        SdrRegistry::GetInstance().GetShaderNodeByIdentifierAndType(
            wrapped, TfToken("glslfx"));
    _Check(node != nullptr, "the wrapper is registered with Sdr");
    if (node && baseNode) {
        for (const TfToken &input : baseNode->GetShaderInputNames()) {
            _Check(node->GetShaderInput(input) != nullptr,
                   "the wrapper has input " + input.GetString());
        }
        const SdrTokenVec primvars = node->GetPrimvars();
        _Check(std::find(primvars.begin(), primvars.end(),
                         TfToken("rigExecTouchTable")) != primvars.end(),
               "the wrapper asks for its primvars, so filtering keeps them");
    }
    _Check(RigExecTouchPoseGetWrappedTerminal(base) == wrapped,
           "wrapping is cached");
}

// -- scene index -------------------------------------------------------------

class _Recorder : public HdSceneIndexObserver {
public:
    void PrimsAdded(const HdSceneIndexBase &, const AddedPrimEntries &e) override
    { added.insert(added.end(), e.begin(), e.end()); }
    void PrimsRemoved(const HdSceneIndexBase &, const RemovedPrimEntries &) override {}
    void PrimsDirtied(const HdSceneIndexBase &, const DirtiedPrimEntries &e) override
    { dirtied.insert(dirtied.end(), e.begin(), e.end()); }
    void PrimsRenamed(const HdSceneIndexBase &, const RenamedPrimEntries &) override {}
    void Clear() { added.clear(); dirtied.clear(); }
    bool Dirtied(const SdfPath &path, const HdDataSourceLocator &locator) const
    {
        for (const auto &entry : dirtied) {
            if (entry.primPath == path && entry.dirtyLocators.Contains(locator)) {
                return true;
            }
        }
        return false;
    }
    AddedPrimEntries added;
    DirtiedPrimEntries dirtied;
};

HdContainerDataSourceHandle _Binding(const SdfPath &material)
{
    return HdRetainedContainerDataSource::New(
        HdMaterialBindingsSchema::GetSchemaToken(),
        HdMaterialBindingsSchema::BuildRetained(
            1, &HdMaterialBindingsSchemaTokens->allPurpose,
            std::vector<HdDataSourceBaseHandle>{
                HdMaterialBindingSchema::Builder()
                    .SetPath(HdRetainedTypedSampledDataSource<SdfPath>::New(
                        material))
                    .Build()}.data()));
}

HdContainerDataSourceHandle _PreviewMaterial()
{
    const TfToken surfaceNode("Surface");
    const HdDataSourceBaseHandle node =
        HdMaterialNodeSchema::Builder()
            .SetNodeIdentifier(
                HdRetainedTypedSampledDataSource<TfToken>::New(
                    TfToken("UsdPreviewSurface")))
            .SetParameters(HdRetainedContainerDataSource::New(
                TfToken("opacityMode"),
                HdMaterialNodeParameterSchema::Builder()
                    .SetValue(HdRetainedTypedSampledDataSource<TfToken>::New(
                        TfToken("presence")))
                    .Build()))
            .Build();
    const HdDataSourceBaseHandle network =
        HdMaterialNetworkSchema::Builder()
            .SetNodes(HdRetainedContainerDataSource::New(surfaceNode, node))
            .SetTerminals(HdRetainedContainerDataSource::New(
                HdMaterialTerminalTokens->surface,
                HdMaterialConnectionSchema::Builder()
                    .SetUpstreamNodePath(
                        HdRetainedTypedSampledDataSource<TfToken>::New(
                            surfaceNode))
                    .SetUpstreamNodeOutputName(
                        HdRetainedTypedSampledDataSource<TfToken>::New(
                            TfToken("surface")))
                    .Build()))
            .Build();
    return HdRetainedContainerDataSource::New(
        HdMaterialSchema::GetSchemaToken(),
        HdRetainedContainerDataSource::New(
            HdMaterialSchemaTokens->universalRenderContext, network));
}

TfToken _SurfaceIdentifier(const HdSceneIndexBaseRefPtr &scene,
                           const SdfPath &material, VtValue *opacityMode)
{
    const HdMaterialNetworkSchema network =
        HdMaterialSchema::GetFromParent(scene->GetPrim(material).dataSource)
            .GetMaterialNetwork();
    const HdMaterialNodeSchema node =
        network.GetNodes().Get(TfToken("Surface"));
    if (opacityMode) {
        const auto param = node.GetParameters().Get(
            TfToken("opacityMode"));
        *opacityMode = param.GetValue() ? param.GetValue()->GetValue(0.0f)
                                        : VtValue();
    }
    const auto id = node.GetNodeIdentifier();
    return id ? id->GetTypedValue(0.0f) : TfToken();
}

void TestSceneIndex()
{
    const SdfPath mesh("/Body/geo");
    const SdfPath subset("/Body/geo/skin");
    const SdfPath other("/Body/other");
    const SdfPath pants("/Materials/pants");
    const SdfPath skin("/Materials/skin");
    const SdfPath unused("/Materials/unused");

    HdRetainedSceneIndexRefPtr retained = HdRetainedSceneIndex::New();
    retained->AddPrims({
        {SdfPath("/Body"), TfToken(), HdRetainedContainerDataSource::New()},
        {mesh, HdPrimTypeTokens->mesh, _Binding(pants)},
        {subset, HdPrimTypeTokens->geomSubset, _Binding(skin)},
        {other, HdPrimTypeTokens->mesh, _Binding(unused)},
        {SdfPath("/Materials"), TfToken(), HdRetainedContainerDataSource::New()},
        {pants, HdPrimTypeTokens->material, _PreviewMaterial()},
        {skin, HdPrimTypeTokens->material, _PreviewMaterial()},
        {unused, HdPrimTypeTokens->material, _PreviewMaterial()},
    });
    RigExecTouchPoseSceneIndexRefPtr index =
        RigExecTouchPoseSceneIndex::New(retained);
    _Recorder recorder;
    index->AddObserver(HdSceneIndexObserverPtr(&recorder));

    _Check(!HdPrimvarsSchema::GetFromParent(index->GetPrim(mesh).dataSource)
                .GetPrimvar(TfToken("rigExecTouchRegion")),
           "nothing is added before attach");
    _Check(_SurfaceIdentifier(index, skin, nullptr) == TfToken("UsdPreviewSurface"),
           "and no material is touched");

    VtFloatArray slots{1, 0, 2};
    VtVec4fArray table(3, GfVec4f(0));
    RigExecTouchPoseHighlights &highlights = RigExecTouchPoseHighlights::GetInstance();
    highlights.SetMesh(mesh, slots, table);

    const HdDataSourceLocator region =
        HdPrimvarsSchema::GetDefaultLocator().Append(TfToken("rigExecTouchRegion"));
    const HdDataSourceLocator tableLoc =
        HdPrimvarsSchema::GetDefaultLocator().Append(TfToken("rigExecTouchTable"));
    _Check(recorder.Dirtied(mesh, region) && recorder.Dirtied(mesh, tableLoc),
           "attach dirties both primvars (as descriptors, not just values)");
    _Check(recorder.Dirtied(pants, HdMaterialSchema::GetDefaultLocator()) &&
               recorder.Dirtied(skin, HdMaterialSchema::GetDefaultLocator()),
           "attach dirties the mesh's material AND its subset's");
    _Check(!recorder.Dirtied(unused, HdMaterialSchema::GetDefaultLocator()),
           "but not a material only another mesh uses");

    const HdPrimvarsSchema primvars =
        HdPrimvarsSchema::GetFromParent(index->GetPrim(mesh).dataSource);
    const HdPrimvarSchema regionPv = primvars.GetPrimvar(TfToken("rigExecTouchRegion"));
    const HdPrimvarSchema tablePv = primvars.GetPrimvar(TfToken("rigExecTouchTable"));
    _Check(regionPv && regionPv.GetInterpolation() &&
               regionPv.GetInterpolation()->GetTypedValue(0.0f) ==
                   HdPrimvarSchemaTokens->uniform,
           "the region primvar is uniform");
    _Check(tablePv && tablePv.GetInterpolation() &&
               tablePv.GetInterpolation()->GetTypedValue(0.0f) ==
                   HdPrimvarSchemaTokens->constant,
           "the table primvar is constant");
    if (regionPv && regionPv.GetPrimvarValue()) {
        const VtValue v = regionPv.GetPrimvarValue()->GetValue(0.0f);
        _Check(v.IsHolding<VtFloatArray>() && v.UncheckedGet<VtFloatArray>() == slots,
               "the region primvar carries the slots");
    }

    VtValue mode;
    const TfToken wrapped = RigExecTouchPoseGetWrappedTerminal(TfToken("UsdPreviewSurface"));
    _Check(_SurfaceIdentifier(index, skin, &mode) == wrapped,
           "the subset's material now runs the wrapper");
    _Check(mode.IsHolding<int>() && mode.UncheckedGet<int>() == 0,
           "and its authored opacityMode 'presence' became the int 0");
    _Check(_SurfaceIdentifier(index, unused, nullptr) == TfToken("UsdPreviewSurface"),
           "the unused material is untouched");
    std::set<SdfPath> wrappedSet = index->GetWrappedMaterials(mesh);
    _Check(wrappedSet.size() == 2, "exactly two materials wrapped");

    // A hover: ONLY the table value moves.
    recorder.Clear();
    table[1] = GfVec4f(1, 0, 0, 0.85f);
    _Check(highlights.SetTable(mesh, table), "table accepted");
    _Check(recorder.dirtied.size() == 1 &&
               recorder.Dirtied(mesh, tableLoc.Append(HdPrimvarSchemaTokens->primvarValue)) &&
               !recorder.Dirtied(mesh, region),
           "a hover dirties the table's VALUE and nothing else");
    _Check(recorder.added.empty(), "and adds nothing");
    recorder.Clear();
    _Check(highlights.SetTable(mesh, table), "same table accepted");
    _Check(recorder.dirtied.empty(), "an identical table sends nothing");

    // A paint stroke: only the slots.
    slots[1] = 2;
    highlights.SetFaceSlots(mesh, slots);
    _Check(recorder.dirtied.size() == 1 &&
               recorder.Dirtied(mesh, region.Append(HdPrimvarSchemaTokens->primvarValue)),
           "a stroke dirties the region VALUE only");

    // Detach.
    recorder.Clear();
    highlights.RemoveMesh(mesh);
    _Check(recorder.Dirtied(skin, HdMaterialSchema::GetDefaultLocator()),
           "detach dirties the materials again");
    _Check(_SurfaceIdentifier(index, skin, nullptr) == TfToken("UsdPreviewSurface"),
           "and they are back to UsdPreviewSurface");
    _Check(!HdPrimvarsSchema::GetFromParent(index->GetPrim(mesh).dataSource)
                .GetPrimvar(TfToken("rigExecTouchTable")),
           "and the primvars are gone");
    index->RemoveObserver(HdSceneIndexObserverPtr(&recorder));
}

}  // namespace

int main(int, char **)
{
    TestCast();
    TestMarqueeAndBrush();
    TestCApi();
    TestShader();
    TestSceneIndex();
    if (_failures) {
        std::printf("testRigExecTouchPose: %d FAILURE(S)\n", _failures);
        return 1;
    }
    std::printf("testRigExecTouchPose: OK\n");
    return 0;
}
