# Control viewport guides — design (2026-08-02)

Controls (`RigExecControl`) gain synthesized viewport guide drawing through the
existing Hydra 2.0 results scene index, exactly parallel to the joint/solver
sphere+cone guides (spec §10.3 extension), with authorable shape, draw mode,
and positive per-axis guide-scale multipliers.

Build target: OpenUSD PR #4156 (usdNoodles branch), installed to the
`CMakeLists.txt` default `../usd-install` (or the path supplied through
`USD_INSTALL_DIR`).

## Requirements

- Every `RigExecControl` under a compiled rig can draw a guide in the viewport
  (Storm, via the RigExec filtering scene indices — never authored to the stage).
- `guide:shape`: one of `sphere`, `circle`, `box`, `cube`, `diamond`, `pyramid`.
- `guide:drawMode`: `wire` or `geometry`.
- Positive per-axis guide-scale multipliers as **separate double properties**
  (`guide:scaleX`, `guide:scaleY`, `guide:scaleZ`) — deliberately NOT a vec3,
  per direction. Final drawn size is the evaluated control-frame axis
  magnitude multiplied by this authored guide multiplier.
- Style parity with joint guides: `guide:displayColor`, `guide:displayOpacity`,
  purpose `guide`, visibility and `primOrigin` inherited from the control prim
  so picking a guide selects its control.

### Locked interpretations (flagged as assumptions)

- `circle` and `box` are the planar shapes (normal +Y, drawn in the local XZ
  plane); `cube` is the 3D box. This is the conventional rigging distinction
  between the two box-ish tokens; `rest:space` and per-axis scale reorient.
- All unit shapes are centered at the frame origin with half-extent 1:
  sphere/circle radius 1, box/cube spanning ±1, diamond (octahedron) vertices
  at ±1 on each axis, pyramid base corners (±1, −1, ±1) with apex (0, 1, 0).
- Guide placement is rigid: the control's posed frame is orthonormalized
  exactly like `_AppendGuideFrame` does for joints. Its three removed axis
  magnitudes are retained as guide dimensions, so each final axis size is
  `abs(evaluated frame axis) * guide:scaleAxis`. The authored guide multiplier
  must be finite and positive on every axis; otherwise the guide draws nothing
  (mirrors the joint `guide:radius` rule).
- Control and joint `avars:sx/sy/sz` use a signed nonzero floor: every finite
  magnitude below `1e-4` resolves to `copysign(1e-4, value)`, including signed
  zero. Negative values therefore keep reflection semantics while a collapsed
  axis remains invertible and publishable. Strict authoring rejects non-finite
  values; raw non-finite USD resolves to identity scale on that axis. This
  floor applies before the evaluated frame magnitudes above are measured and
  is distinct from `guide:scaleX/Y/Z`, whose non-positive values still hide
  the guide.
- Wire curves author a constant widths primvar from `guide:wireWidth`
  (default 0.05, local pre-scale units). usdview's interactive pick window
  is a single physical pixel, so an unwidthed hairline is effectively
  unpickable — the default width is what makes wire controls selectable.
  Zero/negative wireWidth authors no widths (hairline fallback); geometry
  mode ignores it.
- **The width alone is not enough**: Storm honours a curve's width only once
  the curve is refined (`HdStBasisCurves::_SupportsRefinement` is
  `refineLevel > 0`), and usdview's default complexity ("low") is
  refineLevel 0, where linear basisCurves draw as one-pixel GL lines with
  widths ignored outright. A widthed wire guide therefore also authors
  `displayStyle.refineLevel = 1` on itself, which decouples control
  pickability from the viewer's global complexity setting — that setting is
  a display preference about the asset and should not decide whether the
  rig's controls can be clicked. Measured over 1681 single-pixel picks at
  the default complexity: 8 hits hairline, 8 widthed-but-unrefined, 89
  widthed and self-refined (97 at complexity "high").
- The planar shapes (`circle`, `box`, normal +Y) are edge-on — a line —
  from a front view; they suit rigs viewed from above. Examples favor the
  3D shapes for front-facing legibility.

## Schema (`libs/rigExecSchema/schema.usda`, class `RigExecControl`)

```usda
uniform token guide:shape = "circle" (
    allowedTokens = ["sphere", "circle", "box", "cube", "diamond", "pyramid"]
)
uniform token guide:drawMode = "wire" (
    allowedTokens = ["wire", "geometry"]
)
double guide:scaleX = 1.0
double guide:scaleY = 1.0
double guide:scaleZ = 1.0
double guide:wireWidth = 0.05
color3f guide:displayColor = (1.0, 0.85, 0.2)
float guide:displayOpacity = 1.0
```

Each attribute carries a doc string in the style of the `RigExecJoint` guide
attrs (see `guide:radius` there). The codeless plugin
`plugin/rigExecSchema/resources/generatedSchema.usda` is regenerated with
`usdGenSchema` from the new install once USD finishes building (recipe:
`bin/gen_schema.bat`, run its steps manually on macOS with
`PYTHONPATH=$USD/lib/python`).

## Evaluator (`libs/rigExec/rigEvaluator.{h,cpp}`)

- Compile: discover `_controlPaths` (all `RigExecControl` prims beneath the
  rig, namespace walk parallel to `_DiscoverJointOutputs`; empty is fine) and
  tap each control's `computePointFrame` (base phase — controls are inputs)
  into `_controlFrameTaps`, following the `_jointFrameTaps` pattern. Phase-A
  validation discipline applies: locals first, commit on success.
- `RigExecRigPose` gains
  `std::map<SdfPath, RigExecPointFrame> controlFrames;` (ASSET-space, like
  joint frames), filled in `Evaluate` from the taps.

## Bridge (`libs/rigExecImaging/bridge.cpp`, `snapshotStore.h`)

`RigExecPublishedPrim` additions:

```cpp
bool hasControlGuide = false;
GfMatrix4d controlGuideFrame;   // rigidized, ASSET-space
TfToken controlGuideShape;      // sphere|circle|box|cube|diamond|pyramid
TfToken controlGuideDrawMode;   // wire|geometry
GfVec3d controlGuideScale;      // |evaluated axes| * authored guide multipliers
```

`_FillControlGuides(pose, snapshot)` (called from both `Evaluate` snapshot
paths, like `_FillGuides`): for each `pose.controlFrames` entry, rigidize the
frame (factor the orthonormalize/determinant/finite-check block out of
`_AppendGuideFrame` into a shared helper) while retaining its three axis
magnitudes. Read `guide:shape/drawMode/scaleX/scaleY/scaleZ` at `pose.time`,
reject invalid authored multipliers, and publish their product with those
evaluated magnitudes as `controlGuideScale`; reuse `_ReadGuideStyle` for
color/opacity. `snapshot->assetRoot` must be set on this path too.

Snapshot diff (`RigExecComputeChanges`): fold the new fields into the
existing guide comparisons — `hasControlGuide`/shape/drawMode changes follow
the structural (resync) arm; frame/scale/color/opacity changes set
`RigExecChangeGuides`.

## Results scene index (`libs/rigExecImaging/sceneIndices.cpp`)

- One synthesized child per guide-bearing control: fixed name `rigGuideCtrl`
  (single child; no index needed), announced/synced through the same
  machinery as the sphere/cone children (`_DesiredGuideCount`-style
  bookkeeping extended, `GetChildPrimPaths`, `_SyncGuideChildren` analog,
  pulls resolved in `GetPrim` only while the parent exists upstream).
- Prim type:
  - `wire` → `HdPrimTypeTokens->basisCurves`: linear, nonperiodic,
    closed rings by repeating the first point. Topologies:
    circle 1×33-pt ring; sphere 3 orthogonal 33-pt rings; box 1×5-pt ring;
    cube 2×5-pt rings + 4×2-pt pillars; diamond 3 orthogonal 5-pt rings
    through the axis vertices (the exact octahedron edge set);
    pyramid 1×5-pt base ring + 4×2-pt apex edges. No widths authored.
  - `geometry` → implicit `sphere` (radius 1) and `cube` (size 2) reuse the
    Hydra implicits exactly as joint guides do; `circle` (one 32-vert face),
    `box` (one quad), `diamond` (8 tris), `pyramid` (4 tris + base quad) are
    meshes, `doubleSided = true`, no authored normals (flat shading is fine
    for guides). Meshes and curves publish local unit `HdExtentSchema`
    min/max; scale lives in the xform.
- Xform: `S(abs(frameAxisX) * guide:scaleX, abs(frameAxisY) * guide:scaleY,
  abs(frameAxisZ) * guide:scaleZ) × rigidFrame × assetRoot placement`, anchored
  exactly the way `_BuildGuidePrim` places sphere/cone guides (ASSET-space
  frames; asset root, not the guide's namespace parent). Signed avar scale
  affects frame handedness; the synthesized guide uses positive dimensions
  and a proper rigid placement.
- Style/pick parity with `_BuildGuidePrim`: purpose `guide` render tag,
  constant `displayColor`/`displayOpacity` primvars, hand-inherited
  visibility and `primOrigin` from the parent control.
- Dirtying: `RigExecChangeGuides` on a control entry dirties/resyncs its
  `rigGuideCtrl` child the same way sphere/cone children react today.

## Host-durability redesign (user-approved 2026-08-02)

Two usdview-adapter behaviors were patches, not architecture; both are
replaced so the adapter shrinks to its irreducible duties (stage handoff +
time forwarding):

1. **Native bounds instead of the `computeWorldBound` monkeypatch.**
   `RigExecXformable` inherits `UsdGeomBoundable` (accepting Imageable and
   Xformable in the chain), and the plugin registers compute-extent
   functions via the public TfType-keyed
   `UsdGeomRegisterComputeExtentFunction(TfType, fn)` overload — no
   generated schema classes. Extent(t) is the guide's drawn bounds baked
   into asset-relative space: answered from the active imaging snapshot
   when one exists, else a rest-pose fallback computed purely from
   authored attrs (`rest:space` chain × unit shape × normalized avar-scale
   magnitude × positive per-axis guide multiplier).
   `UsdGeomBBoxCache` — what usdview, Solaris, and mayaUsd consult — then
   answers natively in every host, and the usdview monkeypatch is deleted
   (the C API bounds exports remain for hosts that want live bounds
   directly).
   - The extent bakes the posed frame because the prim itself carries no
     stage transform; this is correct exactly when no Xformable sits
     between the asset root and the provider — the compiler validates and
     warns otherwise.
   - `xformOps` now exist on RigExec types but are NOT a transform
     authority (`rest:space` + avars remain the only one, per the Ir
     alignment); the compiler warns when ops are authored on a provider.
   - **Extent contract (measured, not assumed).** The callback is a PURE
     FUNCTION of (stage, time). The snapshot carries the stage OBJECT (a
     `UsdStageWeakPtr`, not a root-layer identifier — two stages routinely
     share a root layer and differ only by session layer) and its sample
     time, and is consulted only when both match the query; otherwise the
     rest fallback answers. Consequences, in full:
     - A host holding ONE `UsdGeomBBoxCache` across generation swaps at
       the SAME time keeps the generation that was active when it first
       asked. `UsdGeomBBoxCache` caches a plugin-computed extent as
       constant, and there is no plugin-side invalidation mechanism over
       unauthored state — RigExec authors no extent attribute and must not
       (`testRigExecNoAuthoring`), so nothing marks it time-varying. This
       is an architectural property of BBoxCache, not a RigExec defect.
       Measured on `01_FkChainTail`: after `SetTime(1024)` a long-lived
       cache still returned the 1001 box while a fresh cache returned
       1024's.
     - usdview's per-frame flow re-queries per time, so ordinary scrubbing
       is correct; `rootDataModel.py:161` calls `_bboxCache.SetTime()` on
       every frame change, and `_clearCaches()` `Clear()`s on stage edits.
     - A host needing live exactness against the current generation
       without a cache round-trip uses the C API bounds exports
       (`RigExecImaging_GetGuideBoundsAssetSpace` and the all-bounds
       variant), which read the published snapshot directly.
     `TestExtentIsPureFunctionOfStageAndTime` pins all of this, including
     the caching behavior, so that if USD ever begins re-querying, the
     test fails and this section gets corrected rather than quietly going
     wrong.
   - **One extent, one purpose.** A boundable's extent is filed by
     `UsdGeomBBoxCache` under that prim's own resolved purpose, so the
     subtree union — snapshot and rest-fallback paths alike — includes
     only descendants whose resolved purpose matches. A default-purpose
     control nested under a guide-purpose joint is excluded (a viewer with
     guides off would otherwise frame around geometry it is not showing),
     and the compiler warns about the nesting.
   - **Subtree authority.** A Boundable's extent speaks for its whole
     subtree, because `UsdGeomBBoxCache` stops descending at one
     ("Boundables should always provide their own extent and do not
     require participation from descendants"). RigExec nests providers as
     a matter of course, so a provider's extent unions every published
     guide beneath it; the rest fallback walks nested providers the same
     way. Authored gprims parented under a provider are still outside it
     — the compiler warns about that authoring pattern.
   - **Renderer equivalence.** The extent and the synthesized prim go
     through one predicate: an unrecognized `guide:shape`/`guide:drawMode`
     pair draws nothing and bounds nothing, a frame that cannot be
     orthonormalized draws nothing and bounds nothing, and a wire guide's
     box is inflated by half its effective wire width.
   - Discovery mechanics: the checked-in schema resources stay a pure
     data-only codeless plugin — no `implementsComputeExtent`, no
     LibraryPath, and `schema.usda` carries no `extraPlugInfo` (an earlier
     revision put the flag there, which regeneration then re-emitted into
     the unloadable checked-in plugInfo). CMake's generated-copy rewrite
     is the SINGLE injection point: it stamps `implementsComputeExtent`
     onto the boundable types and a LibraryPath to the imaging library
     into `build/usd/rigExecSchema/resources/plugInfo.json` (and the
     installed copy), so Plug loads the registering code on demand exactly
     there. Environments must therefore point PXR_PLUGINPATH_NAME at the
     GENERATED schema resources, never the source ones. Verified by
     `testRigExecBounds`, which links zero RigExec libraries and queries
     `UsdGeomBBoxCache` in a fresh process.

2. **Control guides draw as regular geometry, not purpose guide.**
   Implemented on the STOCK `UsdGeomImageable` purpose attribute rather
   than a RigExec token: `UsdGeomBBoxCache` buckets a prim's extent by
   that exact attribute, so routing the drawn render tag through it is
   what keeps bounds and drawing in the same bucket by construction.
   `RigExecJoint` and the aggregate solvers override the fallback to
   `guide`; `RigExecControl` keeps the inherited `default`.
   Controls are the rig's interaction surface: their guides publish
   purpose `default` (geometry render tag) so they draw whenever the rig
   draws — no viewer setting involved, and the plugin stops flipping
   `displayGuide`. A new `uniform token guide:purpose = "default"`
   (allowedTokens `default`, `guide`) on `RigExecControl` lets pipelines
   opt back into guide-tag behavior per control. Joint/solver guides stay
   purpose `guide` (diagnostics).

## Tests (`tests/testRigExecImaging.cpp` + example)

- A control-guides section: author each shape × mode on controls of an
  example rig; assert synthesized child exists with the expected prim type,
  curve-count/vertex-count or face-count topology, evaluated-axis magnitude ×
  authored guide scale visible in the xform's basis lengths, purpose guide,
  constant color/opacity, parent visibility inheritance, and `primOrigin`
  resolving to the control path.
- Zero, signed zero, and sub-`1e-4` avar scales resolve to the signed floor and
  keep the guide publishable; ordinary negative avar scale preserves reflected
  deformation while the guide uses its positive magnitude.
- Zero/negative `guide:scaleY` (e.g.) draws nothing.
- `testRigExecNoAuthoring` (whole-scene equality) must keep passing — the
  guides are synthesized, never authored.
- One example (`examples/01_FkChainTail.usda`) authors a spread of
  shapes/modes so `launch_usdview` shows them.
