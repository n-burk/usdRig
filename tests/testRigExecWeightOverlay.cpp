//
// RigExec influence-overlay and volume-guide tests (spec §4.1 volumetric
// extension, drawn side; spec §10.3 influence-overlay extension).
//
// The thing under test is a PICTURE: a rigger placing an influence volume
// must see the region it grabs painted onto the geometry, and must see the
// falloff band's own iso-surfaces as wire guides. Both are only useful if
// they agree with the field that actually deformed the mesh, so every case
// here starts from a real evaluated pose and asserts against the weights
// the mover consumed -- never against a re-derivation.
//
// argv[1] = path to the examples directory; the codeless schema plugin is
// expected at <examples>/../plugin/rigExecSchema/resources.
//
#include "rigExecImaging/bridge.h"
#include "rigExecImaging/registry.h"
#include "rigExecImaging/sceneIndices.h"

#include "pxr/base/gf/range3d.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/ts/knot.h"
#include "pxr/base/ts/spline.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/imaging/hd/basisCurvesSchema.h"
#include "pxr/imaging/hd/dataSourceLocator.h"
#include "pxr/imaging/hd/extentSchema.h"
#include "pxr/imaging/hd/meshSchema.h"
#include "pxr/imaging/hd/meshTopologySchema.h"
#include "pxr/imaging/hd/sceneIndexObserver.h"
#include "pxr/imaging/hd/basisCurvesTopologySchema.h"
#include "pxr/imaging/hd/primvarSchema.h"
#include "pxr/imaging/hd/primvarsSchema.h"
#include "pxr/imaging/hd/retainedDataSource.h"
#include "pxr/imaging/hd/retainedSceneIndex.h"
#include "pxr/imaging/hd/tokens.h"
#include "pxr/imaging/hd/xformSchema.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <utility>
#include <vector>

using namespace rigExec;

static int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

namespace {

const SdfPath _kMeshPath("/Asset/Geom/M");
const SdfPath _kRigPath("/Asset/Rig");
const SdfPath _kVolumePath("/Asset/Rig/Weights/Sphere");
const SdfPath _kTarget("/Asset/Geom/M.points");

// The fixture, copied from testRigExecVolumeWeights' working idiom: a
// four-point mesh, a rig, a joint posed as a pure +2Y translation, and a
// sphere volume weight consumed by one matrix mover. The four points sit at
// radii 0, 0.5, 1 and 2 in a [0, 2] band, so the resolved field is
// 1, 0.75, 0.5, 0 -- one saturated end, one fully-off end, and two readable
// values between them, which is exactly what a gradient assertion needs.
struct Fixture {
    UsdStageRefPtr stage;
    VtVec3fArray base;

    Fixture()
    {
        stage = UsdStage::CreateInMemory();
        base = VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(0.5f, 0, 0),
                            GfVec3f(1, 0, 0), GfVec3f(2, 0, 0)};
        UsdPrim mesh = stage->DefinePrim(_kMeshPath, TfToken("Points"));
        mesh.CreateAttribute(TfToken("points"),
                             SdfValueTypeNames->Point3fArray).Set(base);
        stage->DefinePrim(_kRigPath, TfToken("RigExecRoot"));
        UsdPrim joint = stage->DefinePrim(SdfPath("/Asset/Rig/Joints/J"),
                                          TfToken("RigExecJoint"));
        GfMatrix4d posed(1.0);
        posed.SetTranslate(GfVec3d(0, 2, 0));
        joint.CreateAttribute(TfToken("posed:space"),
                              SdfValueTypeNames->Matrix4d).Set(posed);

        UsdPrim volume =
            stage->DefinePrim(_kVolumePath, TfToken("RigExecSphereWeight"));
        GfMatrix4d at(1.0);
        at.SetTranslate(GfVec3d(0, 0, 0));
        volume.CreateAttribute(TfToken("posed:space"),
                               SdfValueTypeNames->Matrix4d).Set(at);
        volume.CreateRelationship(TfToken("rigExec:weightTarget"))
            .SetTargets({_kTarget});
        volume.CreateAttribute(TfToken("inputs:falloffMin"),
                               SdfValueTypeNames->Float).Set(0.0f);
        volume.CreateAttribute(TfToken("inputs:falloffMax"),
                               SdfValueTypeNames->Float).Set(2.0f);
        // Linear keeps the expected weights arithmetic rather than
        // smoothstep evaluations.
        volume.CreateAttribute(TfToken("rigExec:falloffProfile"),
                               SdfValueTypeNames->Token)
            .Set(TfToken("linear"));

        UsdPrim mover = stage->DefinePrim(SdfPath("/Asset/Rig/Movers/M"),
                                          TfToken("RigExecMatrixMover"));
        mover.ApplyAPI(TfToken("RigExecMoverAPI"));
        mover.CreateRelationship(TfToken("rigExec:moves"))
            .SetTargets({_kTarget});
        mover.CreateRelationship(TfToken("rigExec:transform"))
            .SetTargets({joint.GetPath()});
        mover.CreateRelationship(TfToken("rigExec:weightObject"))
            .SetTargets({_kVolumePath});
    }
};

// The upstream a results scene index is driven over: a mesh shell for the
// weighted geometry and one for the volume weight, since a synthesized
// guide child is only served while its parent exists upstream.
HdRetainedSceneIndexRefPtr
_BuildUpstream()
{
    HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
    upstream->AddPrims(
        {{_kMeshPath, HdPrimTypeTokens->mesh,
          HdRetainedContainerDataSource::New(0, nullptr, nullptr)},
         {_kVolumePath, TfToken("RigExecSphereWeight"),
          HdRetainedContainerDataSource::New(0, nullptr, nullptr)}});
    return upstream;
}

VtVec3fArray
_GetPointsPrimvar(const HdSceneIndexPrim &prim)
{
    HdPrimvarsSchema primvars =
        HdPrimvarsSchema::GetFromParent(prim.dataSource);
    HdPrimvarSchema points = primvars.GetPrimvar(HdTokens->points);
    if (HdSampledDataSourceHandle value = points.GetPrimvarValue()) {
        const VtValue v = value->GetValue(0.0f);
        if (v.IsHolding<VtVec3fArray>()) {
            return v.UncheckedGet<VtVec3fArray>();
        }
    }
    return VtVec3fArray();
}

// The displayColor primvar and its interpolation, or an empty array and an
// empty token when the prim publishes none.
VtVec3fArray
_GetDisplayColorPrimvar(const HdSceneIndexPrim &prim, TfToken *interpolation)
{
    if (interpolation) {
        *interpolation = TfToken();
    }
    HdPrimvarsSchema primvars =
        HdPrimvarsSchema::GetFromParent(prim.dataSource);
    HdPrimvarSchema color = primvars.GetPrimvar(HdTokens->displayColor);
    if (!color) {
        return VtVec3fArray();
    }
    if (interpolation) {
        if (HdTokenDataSourceHandle ds = color.GetInterpolation()) {
            *interpolation = ds->GetTypedValue(0.0f);
        }
    }
    if (HdSampledDataSourceHandle value = color.GetPrimvarValue()) {
        const VtValue v = value->GetValue(0.0f);
        if (v.IsHolding<VtVec3fArray>()) {
            return v.UncheckedGet<VtVec3fArray>();
        }
    }
    return VtVec3fArray();
}

// Records the dirtied notices the results scene index actually forwards, so
// a test can assert Hydra was told rather than that the snapshot changed.
class _OverlayRecordingObserver : public HdSceneIndexObserver {
public:
    void PrimsAdded(const HdSceneIndexBase &,
                    const AddedPrimEntries &) override {}
    void PrimsRemoved(const HdSceneIndexBase &,
                      const RemovedPrimEntries &) override {}
    void PrimsDirtied(const HdSceneIndexBase &,
                      const DirtiedPrimEntries &entries) override {
        for (const auto &e : entries) {
            dirtied.push_back(e.primPath);
            dirtiedLocators.push_back(e.dirtyLocators);
        }
    }
    void PrimsRenamed(const HdSceneIndexBase &,
                      const RenamedPrimEntries &) override {}

    SdfPathVector dirtied;
    std::vector<HdDataSourceLocatorSet> dirtiedLocators;
};

bool
Near(float a, float b, float tol = 1e-4f)
{
    return std::abs(a - b) <= tol;
}

bool
Near(const GfVec3f &a, const GfVec3f &b, float tol = 1e-4f)
{
    return (a - b).GetLength() <= tol;
}

// The weights the shared fixture resolves, restated so the assertions read
// as intent rather than as arithmetic: radii 0, 0.5, 1, 2 over a [0, 2]
// linear band.
const float _kExpectedWeights[4] = {1.0f, 0.75f, 0.5f, 0.0f};

}  // namespace

// The contract this whole feature is built on: the evaluator publishes the
// field a mover actually consumed, keyed by the WEIGHT OBJECT and carrying
// its geometry target inside. Asserted first and on its own, because every
// later case is meaningless if this is not true.
static void
TestPoseCarriesWeightField()
{
    Fixture f;
    RigExecRigEvaluator evaluator(f.stage, _kRigPath);
    std::vector<std::string> errors;
    if (!evaluator.Compile(&errors)) {
        for (const std::string &e : errors) {
            std::printf("pose-weight-field: compile error: %s\n", e.c_str());
        }
        ++failures;
        return;
    }
    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pose.valid);
    if (!pose.valid) {
        return;
    }

    const auto it = pose.weightFields.find(_kVolumePath);
    CHECK(it != pose.weightFields.end());
    if (it == pose.weightFields.end()) {
        return;
    }
    CHECK(it->second.target == _kTarget);
    CHECK(it->second.weights.size() == 4);
    if (it->second.weights.size() != 4) {
        return;
    }
    for (size_t i = 0; i < 4; ++i) {
        CHECK(Near(it->second.weights[i], _kExpectedWeights[i]));
    }
    // A gradient, not a mask: strictly decreasing with radius, saturated at
    // the centre and off at the rim. That is the property a rigger reads,
    // and it is worth asserting separately from the exact values -- a
    // field that got the endpoints right and the middle flat would still
    // be a useless picture.
    for (size_t i = 1; i < 4; ++i) {
        CHECK(it->second.weights[i] < it->second.weights[i - 1]);
    }

    // The placements the volume guides are drawn in travel with it.
    const auto frame = pose.weightFrames.find(_kVolumePath);
    CHECK(frame != pose.weightFrames.end());
}

// The overlay off, then on, then off again, over one live chain: the
// displayColor primvar must be absent, then track the weights, then be
// absent again -- and the upstream points primvar must survive throughout.
static void
TestOverlayPublishesDisplayColor()
{
    Fixture f;
    HdRetainedSceneIndexRefPtr upstream = _BuildUpstream();
    auto store = std::make_shared<RigExecSnapshotStore>();
    RigExecImagingBridge bridge(f.stage, _kRigPath, store);
    std::vector<std::string> errors;
    if (!bridge.Compile(&errors)) {
        for (const std::string &e : errors) {
            std::printf("overlay: compile error: %s\n", e.c_str());
        }
        ++failures;
        return;
    }
    RigExecInternalPrimPruningSceneIndexRefPtr pruning =
        RigExecInternalPrimPruningSceneIndex::New(upstream);
    RigExecBindingResolvingSceneIndexRefPtr binding =
        RigExecBindingResolvingSceneIndex::New(pruning);
    RigExecResultsSceneIndexRefPtr results =
        RigExecResultsSceneIndex::New(binding, store);
    bridge.SetSceneIndices(binding, results);

    // ---- overlay OFF: an ordinary render, untouched.
    CHECK(bridge.EvaluateAndPublish(UsdTimeCode::Default()));
    HdSceneIndexPrim mesh = results->GetPrim(_kMeshPath);
    CHECK(mesh.dataSource);
    TfToken interpolation;
    CHECK(_GetDisplayColorPrimvar(mesh, &interpolation).empty());
    const VtVec3fArray movedWithoutOverlay = _GetPointsPrimvar(mesh);
    CHECK(movedWithoutOverlay.size() == 4);

    // ---- overlay ON.
    bridge.SetWeightOverlay(_kVolumePath);
    CHECK(bridge.EvaluateAndPublish(UsdTimeCode::Default()));
    mesh = results->GetPrim(_kMeshPath);
    CHECK(mesh.dataSource);

    const VtVec3fArray colors = _GetDisplayColorPrimvar(mesh, &interpolation);
    CHECK(colors.size() == 4);
    // VERTEX, not constant: the entire point is that the value varies per
    // point, and a constant primvar would flat-fill the mesh with the
    // first weight's colour and look plausible doing it.
    CHECK(interpolation == HdPrimvarSchemaTokens->vertex);
    if (colors.size() == 4) {
        // Grey where the weight is 0, saturated red where it is 1, and
        // monotone red-ward in between. The endpoint values are the ramp's
        // published contract; the ordering is what makes it a gradient.
        CHECK(Near(colors[0], GfVec3f(1.0f, 0.05f, 0.05f)));
        CHECK(Near(colors[3], GfVec3f(0.55f, 0.55f, 0.55f)));
        for (size_t i = 1; i < 4; ++i) {
            CHECK(colors[i][0] < colors[i - 1][0]);  // less red
            CHECK(colors[i][1] > colors[i - 1][1]);  // more grey
        }
        // ...and it TRACKS the weights rather than merely being ordered
        // like them: each colour is the ramp evaluated at that point's
        // resolved weight.
        for (size_t i = 0; i < 4; ++i) {
            const float w = _kExpectedWeights[i];
            const GfVec3f expected =
                GfVec3f(0.55f, 0.55f, 0.55f) +
                (GfVec3f(1.0f, 0.05f, 0.05f) - GfVec3f(0.55f, 0.55f, 0.55f)) *
                    w;
            CHECK(Near(colors[i], expected));
        }
    }

    // The overlay is an ADDITION. The deformed points still arrive, byte
    // for byte what they were without it -- a strong root that replaced
    // the primvars container instead of composing into it would drop them
    // and the mesh would snap back to its rest shape the moment a rigger
    // turned the overlay on.
    const VtVec3fArray movedWithOverlay = _GetPointsPrimvar(mesh);
    CHECK(movedWithOverlay.size() == 4);
    CHECK(movedWithOverlay == movedWithoutOverlay);
    for (size_t i = 0; i < movedWithOverlay.size(); ++i) {
        CHECK(Near(movedWithOverlay[i],
                   f.base[i] + GfVec3f(0, _kExpectedWeights[i] * 2.0f, 0)));
    }

    // ---- overlay OFF again: the primvar goes away with it.
    bridge.SetWeightOverlay(SdfPath());
    CHECK(bridge.EvaluateAndPublish(UsdTimeCode::Default()));
    mesh = results->GetPrim(_kMeshPath);
    CHECK(_GetDisplayColorPrimvar(mesh, nullptr).empty());
    CHECK(_GetPointsPrimvar(mesh).size() == 4);
}

// Selecting a weight object that this generation resolved no field for
// leaves the render alone. Worth its own case: the alternative -- an
// all-zero field painted flat grey over the mesh -- looks exactly like a
// correctly-computed empty influence, so a silent miss here is invisible.
static void
TestOverlayOfUnknownWeightObjectPaintsNothing()
{
    Fixture f;
    auto store = std::make_shared<RigExecSnapshotStore>();
    RigExecImagingBridge bridge(f.stage, _kRigPath, store);
    if (!bridge.Compile()) {
        ++failures;
        return;
    }
    bridge.SetWeightOverlay(SdfPath("/Asset/Rig/Weights/DoesNotExist"));
    CHECK(bridge.EvaluateAndPublish(UsdTimeCode::Default()));

    const RigExecImagingSnapshotConstPtr snapshot = store->Get();
    CHECK(snapshot);
    if (!snapshot) {
        return;
    }
    const auto it = snapshot->prims.find(_kMeshPath);
    CHECK(it != snapshot->prims.end());
    if (it != snapshot->prims.end()) {
        CHECK(!it->second.hasWeightOverlay);
    }
}

// Turning the overlay on must dirty the mesh, or the viewport keeps the
// picture it already cached and the rigger sees nothing until they scrub.
// The transition changes the owned leaf set (a primvar appears), so it is
// published as structural and dirties universally.
static void
TestOverlayToggleDirtiesTheMesh()
{
    Fixture f;
    auto store = std::make_shared<RigExecSnapshotStore>();
    RigExecImagingBridge bridge(f.stage, _kRigPath, store);
    if (!bridge.Compile()) {
        ++failures;
        return;
    }
    CHECK(bridge.EvaluateAndPublishResult(UsdTimeCode::Default()).ok);

    bridge.SetWeightOverlay(_kVolumePath);
    const RigExecImagingBridge::PublishResult result =
        bridge.EvaluateAndPublishResult(UsdTimeCode::Default());
    CHECK(result.ok);
    bool meshDirtied = false;
    for (const RigExecPublishedDirty &entry : result.dirtied) {
        if (entry.path == _kMeshPath) {
            meshDirtied = true;
            CHECK((entry.changes & RigExecChangeStructural) != 0);
        }
    }
    CHECK(meshDirtied);
}

// The volume's own guides: the falloffMin and falloffMax iso-surfaces,
// synthesized as wire children under the weight prim by the same protocol
// the joint and control guides use.
static void
TestSphereVolumeGuides()
{
    Fixture f;
    // falloffMin non-zero so BOTH iso-surfaces are drawable: a zero inner
    // radius is a point, and the bridge correctly declines to draw it.
    f.stage->GetPrimAtPath(_kVolumePath)
        .GetAttribute(TfToken("inputs:falloffMin")).Set(1.0f);

    HdRetainedSceneIndexRefPtr upstream = _BuildUpstream();
    auto store = std::make_shared<RigExecSnapshotStore>();
    RigExecImagingBridge bridge(f.stage, _kRigPath, store);
    if (!bridge.Compile()) {
        ++failures;
        return;
    }
    RigExecInternalPrimPruningSceneIndexRefPtr pruning =
        RigExecInternalPrimPruningSceneIndex::New(upstream);
    RigExecBindingResolvingSceneIndexRefPtr binding =
        RigExecBindingResolvingSceneIndex::New(pruning);
    RigExecResultsSceneIndexRefPtr results =
        RigExecResultsSceneIndex::New(binding, store);
    bridge.SetSceneIndices(binding, results);
    CHECK(bridge.EvaluateAndPublish(UsdTimeCode::Default()));

    // Two surfaces, announced as traversable children.
    const SdfPathVector children = results->GetChildPrimPaths(_kVolumePath);
    CHECK(children.size() == 2);
    if (children.size() != 2) {
        return;
    }

    // Every announced child must be servable: a traversal that finds a
    // child GetPrim refuses to answer for is the one failure this whole
    // announcement protocol exists to prevent.
    double radii[2] = {0.0, 0.0};
    for (size_t i = 0; i < 2; ++i) {
        const HdSceneIndexPrim guide = results->GetPrim(children[i]);
        CHECK(guide.dataSource);
        CHECK(guide.primType == HdPrimTypeTokens->basisCurves);
        if (!guide.dataSource) {
            continue;
        }
        // Three orthogonal great circles: the classic wire sphere.
        HdBasisCurvesSchema curves =
            HdBasisCurvesSchema::GetFromParent(guide.dataSource);
        CHECK(static_cast<bool>(curves));
        if (HdIntArrayDataSourceHandle ds =
                curves.GetTopology().GetCurveVertexCounts()) {
            CHECK(ds->GetTypedValue(0.0f).size() == 3);
        } else {
            ++failures;
        }
        // The radius rides in the transform, so the iso-surface's size is
        // exactly the scale on the guide's matrix.
        HdXformSchema xform = HdXformSchema::GetFromParent(guide.dataSource);
        CHECK(static_cast<bool>(xform));
        if (HdMatrixDataSourceHandle ds = xform.GetMatrix()) {
            radii[i] = ds->GetTypedValue(0.0f).GetRow3(0).GetLength();
        }
        // Guide purpose and the schema's red default, so the volume and
        // the region it grabs read as one object.
        HdPrimvarsSchema primvars =
            HdPrimvarsSchema::GetFromParent(guide.dataSource);
        HdPrimvarSchema color = primvars.GetPrimvar(HdTokens->displayColor);
        CHECK(static_cast<bool>(color));
        if (HdSampledDataSourceHandle value = color.GetPrimvarValue()) {
            const VtValue v = value->GetValue(0.0f);
            CHECK(v.IsHolding<VtVec3fArray>());
            if (v.IsHolding<VtVec3fArray>()) {
                const VtVec3fArray a = v.UncheckedGet<VtVec3fArray>();
                CHECK(a.size() == 1);
                if (a.size() == 1) {
                    CHECK(Near(a[0], GfVec3f(1.0f, 0.2f, 0.2f)));
                }
            }
        }
    }
    // The authored band, drawn: inner radius 1 and outer radius 2.
    CHECK(Near(float(radii[0]), 1.0f));
    CHECK(Near(float(radii[1]), 2.0f));

    // guide:drawMode = "none" suppresses the guide without disturbing the
    // field -- and takes the announced children back with it.
    f.stage->GetPrimAtPath(_kVolumePath)
        .CreateAttribute(TfToken("guide:drawMode"), SdfValueTypeNames->Token)
        .Set(TfToken("none"));
    CHECK(bridge.EvaluateAndPublish(UsdTimeCode::Default()));
    CHECK(results->GetChildPrimPaths(_kVolumePath).empty());
    CHECK(!results->GetPrim(_kVolumePath.AppendChild(
                                TfToken("rigGuideVol_0")))
               .dataSource);
    // The mesh still moved: drawing is a viewer concern and the field is
    // untouched by it.
    CHECK(_GetPointsPrimvar(results->GetPrim(_kMeshPath)).size() == 4);
}

// A math mover that drives an influence radius must move its guide and its
// field together, including same-frame edits to the operator's inputs.
static void
TestVolumeGuidesFollowPropertyMovers()
{
    Fixture f;
    const UsdPrim radiusMover = f.stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Radius"), TfToken("RigExecFloatMathMover"));
    CHECK(radiusMover.ApplyAPI(TfToken("RigExecMoverAPI")));
    CHECK(radiusMover.GetAttribute(TfToken("rigExec:operation"))
              .Set(TfToken("multiply")));
    const UsdAttribute factor =
        radiusMover.GetAttribute(TfToken("inputs:value"));
    CHECK(factor.Set(2.0f));
    CHECK(radiusMover.GetRelationship(TfToken("rigExec:moves"))
              .SetTargets({_kVolumePath.AppendProperty(
                  TfToken("inputs:falloffMax"))}));

    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    registry.Deactivate();
    std::vector<std::string> errors;
    CHECK(registry.Activate(f.stage, _kRigPath, UsdTimeCode(1), &errors));
    for (float scale : {2.0f, 3.0f}) {
        if (scale != 2.0f) CHECK(factor.Set(scale));
        const RigExecImagingSnapshotConstPtr snapshot =
            registry.GetStore()->Get();
        CHECK(snapshot);
        if (!snapshot) continue;
        const auto volume = snapshot->prims.find(_kVolumePath);
        CHECK(volume != snapshot->prims.end());
        if (volume == snapshot->prims.end()) continue;
        CHECK(volume->second.volumeGuides.size() == 1);
        if (volume->second.volumeGuides.size() != 1) continue;
        const float radius = float(volume->second.volumeGuides[0]
                                       .xform.GetRow3(0).GetLength());
        CHECK(Near(radius, 2.0f * scale));
        const auto mesh = snapshot->prims.find(_kMeshPath);
        CHECK(mesh != snapshot->prims.end());
        if (mesh != snapshot->prims.end()) {
            CHECK(mesh->second.points.size() == 4);
            if (mesh->second.points.size() == 4) {
                CHECK(Near(mesh->second.points[3][1],
                           2.0f * (1.0f - 1.0f / scale)));
            }
        }
    }
    registry.Deactivate();
}

// A placed volume is useful while it is being authored, before any mover has
// been wired to consume it. Prove the complete standalone path: a rig whose
// ONLY outputs are SphereWeight, PlaneWeight, and a valid CurveWeight must
// compile, evaluate, publish each placement, and announce servable guide
// children beneath all three typed prims.
static void
TestStandaloneVolumeGuides()
{
    // Each fallback-only primitive is independently enough to make a rig
    // nonempty; neither relies on the other (or on the valid CurveWeight case
    // below) to sneak through the output gate.
    for (const TfToken &type : {TfToken("RigExecSphereWeight"),
                                TfToken("RigExecPlaneWeight")}) {
        UsdStageRefPtr one = UsdStage::CreateInMemory();
        const SdfPath oneRig("/Only/Rig");
        const SdfPath oneWeight = oneRig.AppendChild(TfToken("Weight"));
        one->DefinePrim(oneRig, TfToken("RigExecRoot"));
        one->DefinePrim(oneWeight, type);
        RigExecRigEvaluator evaluator(one, oneRig);
        std::vector<std::string> oneErrors;
        CHECK(evaluator.Compile(&oneErrors));
        const RigExecRigPose pose =
            evaluator.Evaluate(UsdTimeCode::Default());
        CHECK(pose.valid);
        CHECK(pose.weightFrames.count(oneWeight) == 1);
    }

    const SdfPath rigPath("/Standalone/Rig");
    const SdfPath spherePath("/Standalone/Rig/Weights/Sphere");
    const SdfPath planePath("/Standalone/Rig/Weights/Plane");
    const SdfPath curveWeightPath("/Standalone/Rig/Weights/CurveWeight");
    const SdfPath curvePath("/Standalone/Rig/Curve");

    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(rigPath, TfToken("RigExecRoot"));
    UsdPrim sphere =
        stage->DefinePrim(spherePath, TfToken("RigExecSphereWeight"));
    sphere.CreateAttribute(TfToken("inputs:falloffMin"),
                           SdfValueTypeNames->Float).Set(0.5f);
    UsdPrim plane =
        stage->DefinePrim(planePath, TfToken("RigExecPlaneWeight"));
    UsdPrim curve = stage->DefinePrim(curvePath, TfToken("BasisCurves"));
    curve.CreateAttribute(TfToken("points"),
                          SdfValueTypeNames->Point3fArray)
        .Set(VtVec3fArray{GfVec3f(-2, 0, 0), GfVec3f(0, 1, 0),
                          GfVec3f(2, 0, 0)});
    UsdPrim curveWeight = stage->DefinePrim(
        curveWeightPath, TfToken("RigExecCurveWeight"));
    curveWeight.CreateRelationship(TfToken("rigExec:curve"))
        .SetTargets({curvePath.AppendProperty(TfToken("points"))});
    curveWeight.CreateAttribute(TfToken("inputs:falloffMin"),
                                SdfValueTypeNames->Float).Set(0.5f);

    HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
    upstream->AddPrims(
        {{spherePath, TfToken("RigExecSphereWeight"),
          HdRetainedContainerDataSource::New(0, nullptr, nullptr)},
         {planePath, TfToken("RigExecPlaneWeight"),
          HdRetainedContainerDataSource::New(0, nullptr, nullptr)},
         {curveWeightPath, TfToken("RigExecCurveWeight"),
          HdRetainedContainerDataSource::New(0, nullptr, nullptr)}});
    auto store = std::make_shared<RigExecSnapshotStore>();
    RigExecImagingBridge bridge(stage, rigPath, store);
    std::vector<std::string> errors;
    if (!bridge.Compile(&errors)) {
        for (const std::string &e : errors) {
            std::printf("standalone-volume: compile error: %s\n", e.c_str());
        }
        ++failures;
        return;
    }
    RigExecInternalPrimPruningSceneIndexRefPtr pruning =
        RigExecInternalPrimPruningSceneIndex::New(upstream);
    RigExecBindingResolvingSceneIndexRefPtr binding =
        RigExecBindingResolvingSceneIndex::New(pruning);
    RigExecResultsSceneIndexRefPtr results =
        RigExecResultsSceneIndex::New(binding, store);
    bridge.SetSceneIndices(binding, results);
    CHECK(bridge.EvaluateAndPublish(UsdTimeCode::Default()));

    const RigExecImagingSnapshotConstPtr snapshot = store->Get();
    CHECK(snapshot);
    if (!snapshot) {
        return;
    }
    for (const SdfPath &path : {spherePath, planePath, curveWeightPath}) {
        const auto it = snapshot->prims.find(path);
        CHECK(it != snapshot->prims.end());
        if (it != snapshot->prims.end()) {
            CHECK(it->second.hasVolumeGuides);
            CHECK(!it->second.volumeGuides.empty());
        }
        const SdfPathVector children = results->GetChildPrimPaths(path);
        CHECK(!children.empty());
        for (const SdfPath &child : children) {
            CHECK(results->GetPrim(child).dataSource);
        }
    }

    // A CurveWeight still requires exactly one real point3f[] source. Making
    // volumes standalone must not turn malformed schema authoring into a
    // plausible empty guide generation.
    UsdStageRefPtr invalidStage = UsdStage::CreateInMemory();
    const SdfPath invalidRig("/Invalid/Rig");
    invalidStage->DefinePrim(invalidRig, TfToken("RigExecRoot"));
    invalidStage->DefinePrim(
        invalidRig.AppendChild(TfToken("BadCurve")),
        TfToken("RigExecCurveWeight"));
    RigExecRigEvaluator invalid(invalidStage, invalidRig);
    errors.clear();
    CHECK(!invalid.Compile(&errors));
    CHECK(std::any_of(errors.begin(), errors.end(), [](const std::string &e) {
        return e.find("rigExec:curve must name exactly one points source") !=
               std::string::npos;
    }));
}

// Plane and curve weights draw guides too, and each draws a DIFFERENT
// shape from the sphere: a square in the plane perpendicular to
// rigExec:planeAxis, and a tube along the curve's polyline. Asserted
// together because the property that matters is the same for both -- every
// announced child is servable, and there is one per iso-surface -- and
// because a shape that quietly drew nothing would pass every sphere test
// in this file.
static void
TestPlaneAndCurveVolumeGuides()
{
    struct Case {
        const char *label;
        TfToken type;
        float falloffMin;
        float falloffMax;
        size_t children;
    };
    const Case cases[2] = {
        // A plane's distance is SIGNED, so offset 0 is a real surface and
        // both ends of the band draw. FOUR children, not two: an
        // unbounded plane draws its rectangle plus a separate curve set
        // of outward continuation ticks per iso-surface, and the ticks
        // have to be their own element because in `geometry` draw mode
        // the rectangle is a mesh and cannot carry them.
        {"plane", TfToken("RigExecPlaneWeight"), 0.0f, 1.0f, 4},
        // A tube of radius 0 is a line, so the inner radius is lifted off
        // zero to get two drawn surfaces rather than one.
        {"curve", TfToken("RigExecCurveWeight"), 0.5f, 2.0f, 2}};

    for (const Case &c : cases) {
        Fixture f;
        // Replace the fixture's sphere in place: same path, same mover
        // wiring, different volume type.
        f.stage->RemovePrim(_kVolumePath);
        UsdPrim volume = f.stage->DefinePrim(_kVolumePath, c.type);
        GfMatrix4d at(1.0);
        at.SetTranslate(GfVec3d(0, 0, 0));
        volume.CreateAttribute(TfToken("posed:space"),
                               SdfValueTypeNames->Matrix4d).Set(at);
        volume.CreateRelationship(TfToken("rigExec:weightTarget"))
            .SetTargets({_kTarget});
        volume.CreateAttribute(TfToken("inputs:falloffMin"),
                               SdfValueTypeNames->Float).Set(c.falloffMin);
        volume.CreateAttribute(TfToken("inputs:falloffMax"),
                               SdfValueTypeNames->Float).Set(c.falloffMax);
        volume.CreateAttribute(TfToken("rigExec:falloffProfile"),
                               SdfValueTypeNames->Token)
            .Set(TfToken("linear"));
        if (c.type == "RigExecCurveWeight") {
            UsdPrim curve = f.stage->DefinePrim(SdfPath("/Asset/Rig/Curve"),
                                                TfToken("BasisCurves"));
            curve.CreateAttribute(TfToken("points"),
                                  SdfValueTypeNames->Point3fArray)
                .Set(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(5, 0, 0),
                                  GfVec3f(10, 0, 0)});
            volume.CreateRelationship(TfToken("rigExec:curve"))
                .SetTargets({SdfPath("/Asset/Rig/Curve.points")});
        }

        HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
        upstream->AddPrims(
            {{_kMeshPath, HdPrimTypeTokens->mesh,
              HdRetainedContainerDataSource::New(0, nullptr, nullptr)},
             {_kVolumePath, c.type,
              HdRetainedContainerDataSource::New(0, nullptr, nullptr)}});
        auto store = std::make_shared<RigExecSnapshotStore>();
        RigExecImagingBridge bridge(f.stage, _kRigPath, store);
        std::vector<std::string> errors;
        if (!bridge.Compile(&errors)) {
            for (const std::string &e : errors) {
                std::printf("%s: compile error: %s\n", c.label, e.c_str());
            }
            ++failures;
            continue;
        }
        RigExecInternalPrimPruningSceneIndexRefPtr pruning =
            RigExecInternalPrimPruningSceneIndex::New(upstream);
        RigExecBindingResolvingSceneIndexRefPtr binding =
            RigExecBindingResolvingSceneIndex::New(pruning);
        RigExecResultsSceneIndexRefPtr results =
            RigExecResultsSceneIndex::New(binding, store);
        bridge.SetSceneIndices(binding, results);
        if (!bridge.EvaluateAndPublish(UsdTimeCode::Default())) {
            std::printf("%s: publication failed\n", c.label);
            ++failures;
            continue;
        }

        const SdfPathVector children =
            results->GetChildPrimPaths(_kVolumePath);
        if (children.size() != c.children) {
            std::printf("%s: %zu guide children, expected %zu\n", c.label,
                        children.size(), c.children);
            ++failures;
            continue;
        }
        for (const SdfPath &child : children) {
            const HdSceneIndexPrim guide = results->GetPrim(child);
            CHECK(guide.dataSource);
            CHECK(guide.primType == HdPrimTypeTokens->basisCurves);
            if (!guide.dataSource) {
                continue;
            }
            HdBasisCurvesSchema curves =
                HdBasisCurvesSchema::GetFromParent(guide.dataSource);
            CHECK(static_cast<bool>(curves));
            HdIntArrayDataSourceHandle ds =
                curves.GetTopology().GetCurveVertexCounts();
            CHECK(static_cast<bool>(ds));
            if (!ds) {
                continue;
            }
            const VtIntArray counts = ds->GetTypedValue(0.0f);
            // A closed rectangle (one 5-vertex ring) or its four
            // 2-vertex continuation ticks for the plane;
            // rings-plus-rails for the tube.
            if (c.type == "RigExecPlaneWeight") {
                CHECK(counts.size() == size_t(1) || counts.size() == size_t(4));
            } else {
                CHECK(counts.size() == size_t(7));
            }
            int total = 0;
            for (const int n : counts) {
                CHECK(n >= 2);
                total += n;
            }
            // The topology has to describe exactly the points published,
            // or the curve set is malformed in a way only a renderer would
            // notice.
            HdPrimvarsSchema primvars =
                HdPrimvarsSchema::GetFromParent(guide.dataSource);
            HdPrimvarSchema points = primvars.GetPrimvar(HdTokens->points);
            CHECK(static_cast<bool>(points));
            if (HdSampledDataSourceHandle value = points.GetPrimvarValue()) {
                const VtValue v = value->GetValue(0.0f);
                CHECK(v.IsHolding<VtVec3fArray>());
                if (v.IsHolding<VtVec3fArray>()) {
                    CHECK(int(v.UncheckedGet<VtVec3fArray>().size()) ==
                          total);
                }
            }
        }
    }
}

// Curve geometry mode is a genuinely closed tube, not the wire ladder with
// its width removed. Besides checking the advertised prim type, validate the
// mesh contract a renderer actually consumes: every face count covers the
// index buffer, every edge is shared by two oppositely wound faces, normals
// are face-varying and outward, and extent encloses exactly the local points.
static void
TestCurveGeometryVolumeGuides()
{
    constexpr int kSegments = 8;
    constexpr int kRings = 3;

    Fixture f;
    f.stage->RemovePrim(_kVolumePath);
    UsdPrim volume =
        f.stage->DefinePrim(_kVolumePath, TfToken("RigExecCurveWeight"));
    volume.CreateAttribute(TfToken("posed:space"),
                           SdfValueTypeNames->Matrix4d)
        .Set(GfMatrix4d(1.0));
    volume.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({_kTarget});
    volume.CreateAttribute(TfToken("inputs:falloffMin"),
                           SdfValueTypeNames->Float).Set(0.5f);
    volume.CreateAttribute(TfToken("inputs:falloffMax"),
                           SdfValueTypeNames->Float).Set(2.0f);
    volume.CreateAttribute(TfToken("rigExec:falloffProfile"),
                           SdfValueTypeNames->Token).Set(TfToken("linear"));
    volume.CreateAttribute(TfToken("guide:drawMode"),
                           SdfValueTypeNames->Token).Set(TfToken("geometry"));
    // Exercise the same split as the field: the curve is divided into local
    // space while this scale rides in the synthesized child's transform.
    const double axisScale[3] = {2.0, 3.0, 4.0};
    const char *scaleNames[3] = {
        "inputs:scaleX", "inputs:scaleY", "inputs:scaleZ"};
    for (int axis = 0; axis < 3; ++axis) {
        volume.CreateAttribute(TfToken(scaleNames[axis]),
                               SdfValueTypeNames->Float)
            .Set(float(axisScale[axis]));
    }

    UsdPrim curve = f.stage->DefinePrim(SdfPath("/Asset/Rig/Curve"),
                                        TfToken("BasisCurves"));
    curve.CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
        .Set(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(4, 3, 0),
                          GfVec3f(8, 3, 4)});
    volume.CreateRelationship(TfToken("rigExec:curve"))
        .SetTargets({SdfPath("/Asset/Rig/Curve.points")});

    HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
    upstream->AddPrims(
        {{_kMeshPath, HdPrimTypeTokens->mesh,
          HdRetainedContainerDataSource::New(0, nullptr, nullptr)},
         {_kVolumePath, TfToken("RigExecCurveWeight"),
          HdRetainedContainerDataSource::New(0, nullptr, nullptr)}});
    auto store = std::make_shared<RigExecSnapshotStore>();
    RigExecImagingBridge bridge(f.stage, _kRigPath, store);
    std::vector<std::string> errors;
    if (!bridge.Compile(&errors)) {
        for (const std::string &e : errors) {
            std::printf("curve-geometry: compile error: %s\n", e.c_str());
        }
        ++failures;
        return;
    }
    RigExecInternalPrimPruningSceneIndexRefPtr pruning =
        RigExecInternalPrimPruningSceneIndex::New(upstream);
    RigExecBindingResolvingSceneIndexRefPtr binding =
        RigExecBindingResolvingSceneIndex::New(pruning);
    RigExecResultsSceneIndexRefPtr results =
        RigExecResultsSceneIndex::New(binding, store);
    bridge.SetSceneIndices(binding, results);
    CHECK(bridge.EvaluateAndPublish(UsdTimeCode::Default()));

    const SdfPathVector children = results->GetChildPrimPaths(_kVolumePath);
    CHECK(children.size() == 2);  // both falloff iso-surfaces
    if (children.size() != 2) {
        return;
    }

    const int sideFaces = (kRings - 1) * kSegments;
    double previousDiagonal = 0.0;
    for (const SdfPath &child : children) {
        const HdSceneIndexPrim guide = results->GetPrim(child);
        CHECK(guide.dataSource);
        CHECK(guide.primType == HdPrimTypeTokens->mesh);
        if (!guide.dataSource) {
            continue;
        }
        // Geometry mode must not accidentally retain a curves schema.
        CHECK(!HdBasisCurvesSchema::GetFromParent(guide.dataSource));
        HdMeshSchema mesh = HdMeshSchema::GetFromParent(guide.dataSource);
        CHECK(static_cast<bool>(mesh));
        if (!mesh) {
            continue;
        }
        CHECK(mesh.GetDoubleSided() &&
              !mesh.GetDoubleSided()->GetTypedValue(0.0f));

        HdMeshTopologySchema topology = mesh.GetTopology();
        HdIntArrayDataSourceHandle countsSource =
            topology.GetFaceVertexCounts();
        HdIntArrayDataSourceHandle indicesSource =
            topology.GetFaceVertexIndices();
        CHECK(countsSource && indicesSource);
        if (!countsSource || !indicesSource) {
            continue;
        }
        const VtIntArray counts = countsSource->GetTypedValue(0.0f);
        const VtIntArray indices = indicesSource->GetTypedValue(0.0f);
        CHECK(counts.size() == size_t(sideFaces + 2));
        CHECK(indices.size() ==
              size_t(sideFaces * 4 + 2 * kSegments));
        if (counts.size() != size_t(sideFaces + 2)) {
            continue;
        }
        for (int face = 0; face < sideFaces; ++face) {
            CHECK(counts[face] == 4);
        }
        CHECK(counts[sideFaces] == kSegments);
        CHECK(counts[sideFaces + 1] == kSegments);

        const VtVec3fArray points = _GetPointsPrimvar(guide);
        CHECK(points.size() == size_t(kRings * kSegments));
        if (points.size() != size_t(kRings * kSegments)) {
            continue;
        }

        // Every undirected edge must occur exactly twice, with opposite
        // directions. This proves the two caps share the side-ring vertices
        // and close the tube topologically, not only visually.
        struct EdgeUses {
            int count = 0;
            int orientation = 0;
        };
        std::map<std::pair<int, int>, EdgeUses> edgeUses;
        size_t cursor = 0;
        for (const int count : counts) {
            CHECK(count >= 3);
            CHECK(cursor + size_t(count) <= indices.size());
            if (cursor + size_t(count) > indices.size()) {
                break;
            }
            for (int corner = 0; corner < count; ++corner) {
                const int from = indices[cursor + corner];
                const int to = indices[cursor + (corner + 1) % count];
                CHECK(from >= 0 && size_t(from) < points.size());
                CHECK(to >= 0 && size_t(to) < points.size());
                EdgeUses &uses = edgeUses[std::minmax(from, to)];
                ++uses.count;
                uses.orientation += from < to ? 1 : -1;
            }
            cursor += size_t(count);
        }
        CHECK(cursor == indices.size());
        for (const auto &[edge, uses] : edgeUses) {
            (void)edge;
            CHECK(uses.count == 2);
            CHECK(uses.orientation == 0);
        }

        HdPrimvarsSchema primvars =
            HdPrimvarsSchema::GetFromParent(guide.dataSource);
        HdPrimvarSchema normalsPrimvar =
            primvars.GetPrimvar(HdTokens->normals);
        CHECK(static_cast<bool>(normalsPrimvar));
        CHECK(normalsPrimvar.GetInterpolation() &&
              normalsPrimvar.GetInterpolation()->GetTypedValue(0.0f) ==
                  HdPrimvarSchemaTokens->faceVarying);
        VtVec3fArray normals;
        if (HdSampledDataSourceHandle value =
                normalsPrimvar.GetPrimvarValue()) {
            const VtValue v = value->GetValue(0.0f);
            CHECK(v.IsHolding<VtVec3fArray>());
            if (v.IsHolding<VtVec3fArray>()) {
                normals = v.UncheckedGet<VtVec3fArray>();
            }
        }
        CHECK(normals.size() == indices.size());

        // Ring centers recover the transported centerline from the points.
        GfVec3f ringCenters[kRings] = {
            GfVec3f(0.0f), GfVec3f(0.0f), GfVec3f(0.0f)};
        for (int ring = 0; ring < kRings; ++ring) {
            for (int j = 0; j < kSegments; ++j) {
                ringCenters[ring] += points[ring * kSegments + j];
            }
            ringCenters[ring] /= float(kSegments);
        }

        cursor = 0;
        for (size_t face = 0; face < counts.size(); ++face) {
            const int count = counts[face];
            const GfVec3f origin = points[indices[cursor]];
            GfVec3f geometricNormal(0.0f);
            GfVec3f center(0.0f);
            for (int corner = 0; corner < count; ++corner) {
                center += points[indices[cursor + corner]];
                if (corner > 0 && corner + 1 < count) {
                    geometricNormal += GfCross(
                        points[indices[cursor + corner]] - origin,
                        points[indices[cursor + corner + 1]] - origin);
                }
            }
            center /= float(count);
            CHECK(geometricNormal.GetLength() > 1e-6f);
            if (geometricNormal.GetLength() > 1e-6f) {
                geometricNormal.Normalize();
            }

            GfVec3f outward(0.0f);
            if (int(face) < sideFaces) {
                const int span = int(face) / kSegments;
                outward = center -
                    0.5f * (ringCenters[span] + ringCenters[span + 1]);
            } else if (int(face) == sideFaces) {
                outward = ringCenters[0] - ringCenters[1];
            } else {
                outward = ringCenters[kRings - 1] -
                           ringCenters[kRings - 2];
            }
            CHECK(GfDot(geometricNormal, outward) > 0.0f);
            if (normals.size() == indices.size()) {
                for (int corner = 0; corner < count; ++corner) {
                    const GfVec3f &normal = normals[cursor + corner];
                    CHECK(Near(normal.GetLength(), 1.0f));
                    CHECK(GfDot(normal, geometricNormal) > 0.999f);
                }
            }
            cursor += size_t(count);
        }

        GfRange3d expectedExtent;
        for (const GfVec3f &point : points) {
            expectedExtent.UnionWith(GfVec3d(point));
        }
        HdExtentSchema extent =
            HdExtentSchema::GetFromParent(guide.dataSource);
        CHECK(extent.GetMin() && extent.GetMax());
        if (extent.GetMin() && extent.GetMax()) {
            const GfVec3d min = extent.GetMin()->GetTypedValue(0.0f);
            const GfVec3d max = extent.GetMax()->GetTypedValue(0.0f);
            for (int axis = 0; axis < 3; ++axis) {
                CHECK(Near(float(min[axis]),
                           float(expectedExtent.GetMin()[axis])));
                CHECK(Near(float(max[axis]),
                           float(expectedExtent.GetMax()[axis])));
            }
            const double diagonal = (max - min).GetLength();
            CHECK(diagonal > previousDiagonal);
            previousDiagonal = diagonal;
        }

        HdXformSchema xform =
            HdXformSchema::GetFromParent(guide.dataSource);
        CHECK(xform.GetMatrix());
        if (xform.GetMatrix()) {
            const GfMatrix4d matrix =
                xform.GetMatrix()->GetTypedValue(0.0f);
            for (int axis = 0; axis < 3; ++axis) {
                CHECK(Near(float(matrix.GetRow3(axis).GetLength()),
                           float(axisScale[axis])));
            }
        }
    }
}

// The plane guide's SIZE comes from inputs:extentU/extentV and from
// nothing else; the falloff band only slides the two rectangles apart
// along the axis.
//
// This is the regression test for the reported bug. The guide used to size
// itself max(1, |falloffMin|, |falloffMax|), so a rigger dragging the band
// watched the square grow and shrink instead of watching the two surfaces
// separate -- the band's job -- and had no way to say how big the plane
// actually was. Measured on the published POINTS, because that is where a
// plane's dimensions live (it has no implicit Hydra form whose transform
// could carry them, unlike the sphere a few cases up).
static void
TestPlaneGuideSizeIsExtentsNotBand()
{
    Fixture f;
    f.stage->RemovePrim(_kVolumePath);
    UsdPrim volume =
        f.stage->DefinePrim(_kVolumePath, TfToken("RigExecPlaneWeight"));
    GfMatrix4d at(1.0);
    volume.CreateAttribute(TfToken("posed:space"),
                           SdfValueTypeNames->Matrix4d).Set(at);
    volume.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({_kTarget});
    volume.CreateAttribute(TfToken("rigExec:falloffProfile"),
                           SdfValueTypeNames->Token).Set(TfToken("linear"));
    UsdAttribute falloffMin = volume.CreateAttribute(
        TfToken("inputs:falloffMin"), SdfValueTypeNames->Float);
    UsdAttribute falloffMax = volume.CreateAttribute(
        TfToken("inputs:falloffMax"), SdfValueTypeNames->Float);
    falloffMin.Set(0.0f);
    falloffMax.Set(1.0f);
    // planeAxis defaults to y, so U = z and V = x. Deliberately unequal:
    // a square would pass even if the two extents were swapped.
    UsdAttribute extentU = volume.CreateAttribute(
        TfToken("inputs:extentU"), SdfValueTypeNames->Float);
    UsdAttribute extentV = volume.CreateAttribute(
        TfToken("inputs:extentV"), SdfValueTypeNames->Float);
    extentU.Set(3.0f);
    extentV.Set(2.0f);

    HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
    upstream->AddPrims(
        {{_kMeshPath, HdPrimTypeTokens->mesh,
          HdRetainedContainerDataSource::New(0, nullptr, nullptr)},
         {_kVolumePath, TfToken("RigExecPlaneWeight"),
          HdRetainedContainerDataSource::New(0, nullptr, nullptr)}});
    auto store = std::make_shared<RigExecSnapshotStore>();
    RigExecImagingBridge bridge(f.stage, _kRigPath, store);
    std::vector<std::string> errors;
    if (!bridge.Compile(&errors)) {
        for (const std::string &e : errors) {
            std::printf("plane-guide-size: compile error: %s\n", e.c_str());
        }
        ++failures;
        return;
    }
    RigExecInternalPrimPruningSceneIndexRefPtr pruning =
        RigExecInternalPrimPruningSceneIndex::New(upstream);
    RigExecBindingResolvingSceneIndexRefPtr binding =
        RigExecBindingResolvingSceneIndex::New(pruning);
    RigExecResultsSceneIndexRefPtr results =
        RigExecResultsSceneIndex::New(binding, store);
    bridge.SetSceneIndices(binding, results);

    // What one publication drew: the rectangles' half-extents in x and z,
    // their offsets along y, and how many tick elements came with them.
    struct Drawn {
        std::vector<double> offsets;   // sorted, along planeAxis y
        double halfX = -1.0;
        double halfZ = -1.0;
        size_t rectangles = 0;
        size_t tickSets = 0;
    };
    auto measure = [&](const char *label) {
        Drawn drawn;
        if (!bridge.EvaluateAndPublish(UsdTimeCode::Default())) {
            std::printf("plane-guide-size: %s: publication failed\n", label);
            ++failures;
            return drawn;
        }
        for (const SdfPath &child : results->GetChildPrimPaths(_kVolumePath)) {
            const HdSceneIndexPrim guide = results->GetPrim(child);
            HdBasisCurvesSchema curves =
                HdBasisCurvesSchema::GetFromParent(guide.dataSource);
            HdIntArrayDataSourceHandle ds =
                curves.GetTopology().GetCurveVertexCounts();
            if (!ds) {
                continue;
            }
            const VtVec3fArray points = _GetPointsPrimvar(guide);
            if (ds->GetTypedValue(0.0f).size() != 1) {
                ++drawn.tickSets;  // the four 2-vertex continuation marks
                continue;
            }
            ++drawn.rectangles;
            double halfX = 0.0, halfZ = 0.0, offset = 0.0;
            for (const GfVec3f &p : points) {
                halfX = std::max(halfX, double(std::abs(p[0])));
                halfZ = std::max(halfZ, double(std::abs(p[2])));
                offset = double(p[1]);  // constant over one rectangle
            }
            drawn.offsets.push_back(offset);
            // Every rectangle in one publication is the same size; take
            // the first and assert the rest match it.
            if (drawn.halfX < 0.0) {
                drawn.halfX = halfX;
                drawn.halfZ = halfZ;
            } else {
                CHECK(Near(float(drawn.halfX), float(halfX)));
                CHECK(Near(float(drawn.halfZ), float(halfZ)));
            }
        }
        std::sort(drawn.offsets.begin(), drawn.offsets.end());
        return drawn;
    };

    const Drawn narrow = measure("band 0..1");
    CHECK(narrow.rectangles == 2);
    CHECK(narrow.tickSets == 2);  // unbounded: ticks say "this continues"
    CHECK(Near(float(narrow.halfZ), 3.0f));  // U = z
    CHECK(Near(float(narrow.halfX), 2.0f));  // V = x
    CHECK(narrow.offsets.size() == 2);
    if (narrow.offsets.size() == 2) {
        CHECK(Near(float(narrow.offsets[0]), 0.0f));
        CHECK(Near(float(narrow.offsets[1]), 1.0f));
    }

    // THE MEASUREMENT: a seven-fold change in the band must move the
    // surfaces and leave their size alone.
    falloffMax.Set(7.0f);
    const Drawn wideBand = measure("band 0..7");
    CHECK(Near(float(wideBand.halfZ), float(narrow.halfZ)));
    CHECK(Near(float(wideBand.halfX), float(narrow.halfX)));
    CHECK(wideBand.offsets.size() == 2);
    if (wideBand.offsets.size() == 2) {
        CHECK(Near(float(wideBand.offsets[0]), 0.0f));
        CHECK(Near(float(wideBand.offsets[1]), 7.0f));
    }
    // ...including a band that does not straddle zero at all, which the
    // old max(1, |min|, |max|) sizing would have blown up to 12.
    falloffMin.Set(-12.0f);
    falloffMax.Set(-4.0f);
    const Drawn negativeBand = measure("band -12..-4");
    CHECK(Near(float(negativeBand.halfZ), float(narrow.halfZ)));
    CHECK(Near(float(negativeBand.halfX), float(narrow.halfX)));

    // And the extents, which are the only thing that may resize it.
    extentU.Set(6.0f);
    const Drawn wideU = measure("extentU 6");
    CHECK(Near(float(wideU.halfZ), 6.0f));
    CHECK(Near(float(wideU.halfX), 2.0f));

    // bounded draws the closed rectangle ALONE: with the field stopping
    // at the edge there is nothing continuing, and the missing ticks are
    // what tells the two modes apart at a glance.
    volume.CreateAttribute(TfToken("rigExec:planeBounds"),
                           SdfValueTypeNames->Token).Set(TfToken("bounded"));
    const Drawn bounded = measure("bounded");
    CHECK(bounded.rectangles == 2);
    CHECK(bounded.tickSets == 0);
    CHECK(Near(float(bounded.halfZ), 6.0f));
    CHECK(Near(float(bounded.halfX), 2.0f));
}

// The registry entry point behind RigExecImaging_SetWeightOverlay.
//
// Its one real hazard is a self-deadlock: it takes the same plain
// std::mutex SetTime takes, and it has to call SetTime to republish. This
// case would hang rather than fail if that were got wrong, which is the
// only honest way to test it. It also proves the republication actually
// happens -- an overlay that needed a frame change to appear would be
// useless to the artist toggling it.
static void
TestRegistrySetWeightOverlay()
{
    Fixture f;
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    if (!registry.Activate(f.stage, _kRigPath, UsdTimeCode::Default(),
                           &errors)) {
        for (const std::string &e : errors) {
            std::printf("registry-overlay: activate error: %s\n", e.c_str());
        }
        ++failures;
        return;
    }

    auto meshOverlay = [&registry]() -> const RigExecPublishedPrim * {
        const RigExecImagingSnapshotConstPtr snapshot =
            registry.GetStore()->Get();
        if (!snapshot) {
            return nullptr;
        }
        const auto it = snapshot->prims.find(_kMeshPath);
        return it == snapshot->prims.end() ? nullptr : &it->second;
    };

    const RigExecPublishedPrim *published = meshOverlay();
    CHECK(published != nullptr);
    if (published) {
        CHECK(!published->hasWeightOverlay);
    }
    const uint64_t before = registry.GetStore()->Get()->generation;

    // ---- on. Returns success, republishes, and the mesh carries the field.
    CHECK(registry.SetWeightOverlay(_kVolumePath.GetString()));
    CHECK(registry.GetStore()->Get()->generation > before);
    published = meshOverlay();
    CHECK(published != nullptr);
    if (published) {
        CHECK(published->hasWeightOverlay);
        CHECK(published->weightOverlay.size() == 4);
        if (published->weightOverlay.size() == 4) {
            for (size_t i = 0; i < 4; ++i) {
                CHECK(Near(published->weightOverlay[i],
                           _kExpectedWeights[i]));
            }
        }
    }

    // ---- off, by empty string, which is the C surface's null case.
    CHECK(registry.SetWeightOverlay(std::string()));
    published = meshOverlay();
    CHECK(published != nullptr);
    if (published) {
        CHECK(!published->hasWeightOverlay);
    }

    // A malformed path is rejected rather than reaching SdfPath's loud
    // constructor, and leaves the selection alone.
    CHECK(!registry.SetWeightOverlay("not a path"));

    // The registry is a process-global singleton: leave it as it was found
    // or the next case in this binary inherits an activated rig.
    registry.Deactivate();
}

// An authored edit republishes the field THAT EDIT produced, not the one
// before it.
//
// This is the artist-visible bug it was written for: with the overlay on,
// dragging inputs:falloffMax repainted the gradient one drag-step late, so
// the picture always described the previous value. It looked like a
// missing invalidation and was not one -- every notice was emitted, and
// the values inside them were simply stale.
//
// The cause is delivery ORDER, and it is why this case has to go through
// the registry's own UsdNotice::ObjectsChanged handler rather than calling
// SetTime directly. Tf_NoticeRegistry::_Register prepends, so the
// last-registered listener is delivered first, and OpenExec's
// ExecUsdSystem::_NoticeListener -- which drops every cached computed
// value on an authored change -- is constructed inside Compile(). Register
// after Compile and the re-evaluation runs against exec's pre-edit cache.
// A direct SetTime() call cannot see that: by then the dispatch has ended
// and exec is invalidated, which is exactly why the original bug survived
// a test suite full of SetTime assertions.
//
// Both authoring shapes are covered because they are NOT equivalent
// through USD's change classification: a plain default Set and the spline
// knot the authoring panel writes arrive as different notices, and only
// one of them reproduced in usdview.
static void
TestAuthoredEditRepublishesFreshField()
{
    Fixture f;
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    // A real time code, not Default(): a spline knot is authored at a
    // frame, and an evaluator sitting at Default() would never read one.
    if (!registry.Activate(f.stage, _kRigPath, UsdTimeCode(0.0), &errors)) {
        for (const std::string &e : errors) {
            std::printf("edit-republish: activate error: %s\n", e.c_str());
        }
        ++failures;
        return;
    }
    CHECK(registry.SetWeightOverlay(_kVolumePath.GetString()));

    // The published field, and the generation it arrived in.
    auto overlay = [&registry](std::vector<float> *out) -> uint64_t {
        out->clear();
        const RigExecImagingSnapshotConstPtr snapshot =
            registry.GetStore()->Get();
        if (!snapshot) {
            return 0;
        }
        const auto it = snapshot->prims.find(_kMeshPath);
        if (it != snapshot->prims.end() && it->second.hasWeightOverlay) {
            out->assign(it->second.weightOverlay.begin(),
                        it->second.weightOverlay.end());
        }
        return snapshot->generation;
    };
    auto expect = [](const std::vector<float> &got, const float (&want)[4],
                     const char *what) {
        if (got.size() != 4) {
            ++failures;
            std::printf("FAIL %s: %zu weights published, expected 4\n", what,
                        got.size());
            return;
        }
        for (size_t i = 0; i < 4; ++i) {
            if (!Near(got[i], want[i])) {
                ++failures;
                std::printf("FAIL %s: weight[%zu] = %.4f, expected %.4f "
                            "(the edit before this one's answer?)\n",
                            what, i, got[i], want[i]);
            }
        }
    };

    std::vector<float> field;
    uint64_t generation = overlay(&field);
    expect(field, _kExpectedWeights, "baseline");

    const UsdAttribute falloffMax = f.stage->GetAttributeAtPath(
        _kVolumePath.AppendProperty(TfToken("inputs:falloffMax")));
    CHECK(bool(falloffMax));

    // ---- a default-value edit. Band [0, 4] over radii 0, 0.5, 1, 2.
    falloffMax.Set(4.0f);
    const uint64_t afterDefaultEdit = overlay(&field);
    CHECK(afterDefaultEdit > generation);  // the notice republished at all
    const float wideBand[4] = {1.0f, 0.875f, 0.75f, 0.5f};
    expect(field, wideBand, "after inputs:falloffMax = 4 (default)");
    generation = afterDefaultEdit;

    // ---- the shape the authoring panel writes: a knot on the attribute's
    // spline at the current frame. Band [0, 1], so the outer two points
    // fall out of the field entirely.
    TsSpline spline = falloffMax.GetSpline();
    TsKnot knot(falloffMax.GetTypeName().GetType());
    knot.SetTime(0.0);
    knot.SetValue(1.0f);
    knot.SetNextInterpolation(TsInterpCurve);
    spline.SetKnot(knot);
    CHECK(falloffMax.SetSpline(spline));
    const uint64_t afterSplineEdit = overlay(&field);
    CHECK(afterSplineEdit > generation);
    const float narrowBand[4] = {1.0f, 0.5f, 0.0f, 0.0f};
    expect(field, narrowBand, "after inputs:falloffMax knot = 1 (spline)");

    registry.Deactivate();
}

// The overlay staying ON while the field underneath it CHANGES -- which is
// what every scrub, every avar drag, and every falloff edit does.
//
// Distinct from the toggle case, and the one a rigger actually lives in.
// The toggle changes the owned leaf set and is published as structural, so
// it dirties universally and would redraw even if the narrow locator were
// wrong. A value-only change is published as RigExecChangeWeightOverlay and
// dirties exactly one leaf, so if that leaf is misnamed the gradient FREEZES
// while the geometry keeps deforming -- a stale picture that still looks
// like a plausible influence field, which is the failure this whole
// visualisation exists to prevent.
//
// Asserted through a real HdSceneIndexObserver rather than off the snapshot
// diff: the diff only proves the bridge noticed, not that Hydra was told.
static void
TestOverlayValueChangeDirtiesDisplayColor()
{
    Fixture f;
    HdRetainedSceneIndexRefPtr upstream = _BuildUpstream();
    auto store = std::make_shared<RigExecSnapshotStore>();
    RigExecImagingBridge bridge(f.stage, _kRigPath, store);
    std::vector<std::string> errors;
    if (!bridge.Compile(&errors)) {
        for (const std::string &e : errors) {
            std::printf("overlay-value: compile error: %s\n", e.c_str());
        }
        ++failures;
        return;
    }
    RigExecInternalPrimPruningSceneIndexRefPtr pruning =
        RigExecInternalPrimPruningSceneIndex::New(upstream);
    RigExecBindingResolvingSceneIndexRefPtr binding =
        RigExecBindingResolvingSceneIndex::New(pruning);
    RigExecResultsSceneIndexRefPtr results =
        RigExecResultsSceneIndex::New(binding, store);
    bridge.SetSceneIndices(binding, results);

    // The overlay is already on and already published before the observer
    // attaches, so the change it witnesses is purely a value change.
    bridge.SetWeightOverlay(_kVolumePath);
    CHECK(bridge.EvaluateAndPublish(UsdTimeCode::Default()));

    _OverlayRecordingObserver observer;
    results->AddObserver(HdSceneIndexObserverPtr(&observer));

    // Narrow the band: the same four points now resolve 1, 0.5, 0, 0.
    UsdPrim volume = f.stage->GetPrimAtPath(_kVolumePath);
    CHECK(volume);
    volume.GetAttribute(TfToken("inputs:falloffMax")).Set(1.0f);

    const RigExecImagingBridge::PublishResult result =
        bridge.EvaluateAndPublishResult(UsdTimeCode::Default());
    CHECK(result.ok);
    // EvaluateAndPublishResult publishes without notifying (the registry
    // normally broadcasts); drive the notice the same way it does.
    results->NotifyGenerationPublished(result.dirtied);

    // The diff must report this as a VALUE change, not as structural --
    // otherwise the narrow locator below is never the thing that redraws
    // and this test proves nothing about it.
    bool sawValueChange = false;
    for (const RigExecPublishedDirty &entry : result.dirtied) {
        if (entry.path == _kMeshPath) {
            sawValueChange = (entry.changes & RigExecChangeWeightOverlay) != 0;
            CHECK((entry.changes & RigExecChangeStructural) == 0);
        }
    }
    CHECK(sawValueChange);

    // ...and the notice that reached Hydra names the displayColor leaf.
    static const HdDataSourceLocator kDisplayColorValue(
        HdPrimvarsSchemaTokens->primvars, HdTokens->displayColor,
        HdPrimvarSchemaTokens->primvarValue);
    bool meshDirtiedWithColor = false;
    for (size_t i = 0; i < observer.dirtied.size(); ++i) {
        if (observer.dirtied[i] == _kMeshPath &&
            observer.dirtiedLocators[i].Intersects(kDisplayColorValue)) {
            meshDirtiedWithColor = true;
        }
    }
    CHECK(meshDirtiedWithColor);

    // And the repaint a consumer would then pull is the NEW field, not the
    // old one: a correct notice over stale data is the same frozen picture.
    const float expected[4] = {1.0f, 0.5f, 0.0f, 0.0f};
    const VtVec3fArray colors =
        _GetDisplayColorPrimvar(results->GetPrim(_kMeshPath), nullptr);
    CHECK(colors.size() == 4);
    if (colors.size() == 4) {
        for (size_t i = 0; i < 4; ++i) {
            const GfVec3f want =
                GfVec3f(0.55f, 0.55f, 0.55f) +
                (GfVec3f(1.0f, 0.05f, 0.05f) - GfVec3f(0.55f, 0.55f, 0.55f)) *
                    expected[i];
            CHECK(Near(colors[i], want));
        }
    }

    results->RemoveObserver(HdSceneIndexObserverPtr(&observer));
}

static std::string
DefaultResourceDir()
{
#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    return TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR);
#else
    return std::string();
#endif
}

int
main(int argc, char **argv)
{
    std::string resources = DefaultResourceDir();
    if (argc > 1 && resources.empty()) {
        resources = TfAbsPath(std::string(argv[1]) +
                              "/../plugin/rigExecSchema/resources");
    }
    if (!resources.empty()) {
        PlugRegistry::GetInstance().RegisterPlugins(resources);
    }

    TestPoseCarriesWeightField();
    TestOverlayPublishesDisplayColor();
    TestOverlayOfUnknownWeightObjectPaintsNothing();
    TestOverlayToggleDirtiesTheMesh();
    TestOverlayValueChangeDirtiesDisplayColor();
    TestStandaloneVolumeGuides();
    TestSphereVolumeGuides();
    TestVolumeGuidesFollowPropertyMovers();
    TestPlaneAndCurveVolumeGuides();
    TestCurveGeometryVolumeGuides();
    TestPlaneGuideSizeIsExtentsNotBand();
    TestRegistrySetWeightOverlay();
    TestAuthoredEditRepublishesFreshField();

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecWeightOverlay: all tests passed\n");
    return 0;
}
