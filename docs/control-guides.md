# Control viewport guides — design (2026-08-02)

Controls (`RigExecControl`) gain synthesized viewport guide drawing through the
existing Hydra 2.0 results scene index, exactly parallel to the joint/solver
sphere+cone guides (spec §10.3 extension), with authorable shape, draw mode,
and per-axis scale.

Build target: OpenUSD PR #4156 (usdNoodles branch) checkout at
`/Users/burkard/work/usd-pr4156`, installed to `/Users/burkard/work/usd-install`
(the `CMakeLists.txt` default `../usd-install`).

## Requirements

- Every `RigExecControl` under a compiled rig can draw a guide in the viewport
  (Storm, via the RigExec filtering scene indices — never authored to the stage).
- `guide:shape`: one of `sphere`, `circle`, `box`, `cube`, `diamond`, `pyramid`.
- `guide:drawMode`: `wire` or `geometry`.
- Per-axis draw scale as **separate double properties** (`guide:scaleX`,
  `guide:scaleY`, `guide:scaleZ`) — deliberately NOT a vec3, per direction.
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
- Guides are rigid: the control's posed frame is orthonormalized exactly like
  `_AppendGuideFrame` does for joints; `guide:scaleX/Y/Z` is the only
  dimensional scale. A non-finite or non-positive scale on any axis draws
  nothing (mirrors the joint `guide:radius` rule).
- Wire curves author no widths; UsdImaging/Storm fallback width draws them as
  hairlines, which is the desired wire look.

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
color3f guide:displayColor = (1.0, 0.85, 0.2)
float guide:displayOpacity = 1.0
```

Each attribute carries a doc string in the style of the `RigExecJoint` guide
attrs (see `guide:radius` there). The codeless plugin
`plugin/rigExecSchema/resources/generatedSchema.usda` is regenerated with
`usdGenSchema` from the new install once USD finishes building (recipe:
`gen_schema.bat`, run its steps manually on macOS with
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
GfVec3d controlGuideScale;      // internal storage; authored as 3 doubles
```

`_FillControlGuides(pose, snapshot)` (called from both `Evaluate` snapshot
paths, like `_FillGuides`): for each `pose.controlFrames` entry, rigidize the
frame (factor the orthonormalize/determinant/finite-check block out of
`_AppendGuideFrame` into a shared helper), read
`guide:shape/drawMode/scaleX/scaleY/scaleZ` at `pose.time`, reject invalid
scales, reuse `_ReadGuideStyle` for color/opacity. `snapshot->assetRoot` must
be set on this path too.

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
- Xform: `S(guide:scaleX, scaleY, scaleZ) × rigidFrame × assetRoot placement`,
  anchored exactly the way `_BuildGuidePrim` places sphere/cone guides
  (ASSET-space frames; asset root, not the guide's namespace parent).
- Style/pick parity with `_BuildGuidePrim`: purpose `guide` render tag,
  constant `displayColor`/`displayOpacity` primvars, hand-inherited
  visibility and `primOrigin` from the parent control.
- Dirtying: `RigExecChangeGuides` on a control entry dirties/resyncs its
  `rigGuideCtrl` child the same way sphere/cone children react today.

## Tests (`tests/testRigExecImaging.cpp` + example)

- A control-guides section: author each shape × mode on controls of an
  example rig; assert synthesized child exists with the expected prim type,
  curve-count/vertex-count or face-count topology, per-axis scale visible in
  the xform's basis lengths, purpose guide, constant color/opacity, parent
  visibility inheritance, and `primOrigin` resolving to the control path.
- Zero/negative `guide:scaleY` (e.g.) draws nothing.
- `testRigExecNoAuthoring` (whole-scene equality) must keep passing — the
  guides are synthesized, never authored.
- One example (`examples/01_FkChainTail.usda`) authors a spread of
  shapes/modes so `launch_usdview` shows them.
