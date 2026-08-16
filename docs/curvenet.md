# Curvenets and the Profile Mover

Pixar's curve-based articulation, brought into RigExec. Added 2026-08-07.

Source papers, downloaded to `docs/papers/`:

| File | Paper |
|---|---|
| `2022-ProfileCurves-deGoes-Sheffler-Fleischer.pdf` | de Goes, Sheffler, Fleischer. *Character Articulation through Profile Curves*. ACM TOG 41(4), Article 139, SIGGRAPH 2022. **The** curvenet paper. |
| `2023-Elemental-CurvenetAnimationControls.pdf` | Nguyen, Talbot, Sheffler, Hessler, Fleischer, de Goes. *Shaping the Elements: Curvenet Animation Controls in Pixar's Elemental*. SIGGRAPH 2023 Talks. |
| `2024-InsideOut2-RigChallenges.pdf` | Hoffman, Nieves, Speirs, Zhang. *Pixar's Inside Out 2: Character Rig Challenges and Techniques*. SIGGRAPH 2024 Talks. §2 is curvenet usage. |
| `2026-FaceRigging-CurvenetParametrization.pdf` | Talbot, Sheffler, de Goes. *Face Rigging through Curvenet Parametrization*. SIGGRAPH 2026 Talks. |

Section numbers below in the form §3, §4.1 refer to the 2022 paper.
`.txt` sidecars next to each PDF hold the extracted text.

## What the papers actually claim

The pitch is not "curves are a nicer control than joints". It is a
specific separation:

> By analyzing the layout of the rigged curvenets, we quantify the
> deformation along each curve side **independent of the mesh
> connectivity**, thus separating the articulation controllers from the
> underlying surface representation.

Three consequences the papers lean on, and which this implementation has
to reproduce or it is a different technique wearing the name:

1. **The rig is not bound to the tessellation.** The same rigged curvenet
   articulates a quad cage, a triangulation, a subdivided shell and a
   93k-vertex sculpt (Fig. 8). Nothing in the setup names a vertex.
2. **Each side of a curve deforms independently.** A curve is a *hinge*,
   not a soft handle. Averaging the two sides is explicitly shown as the
   wrong answer (Fig. 11: it bows the surface instead of creasing it).
   This is what the mesh cutting is *for*.
3. **Frames come from the net, not from the artist.** Nobody authors
   normals or twist along a curve. Orientation and non-uniform scale are
   *deduced* from how curves meet at intersections and propagated by
   parallel transport (§3). "Bypassing the need for any curve
   optimization or manual authoring of normals and handles."

Point 3 is the one that is easy to miss and expensive to retrofit, so it
drives the data model below.

## §3 — the curvenet

### Definition and encoding

A curvenet is a collection of 3D curves that may intersect, with **no
topological restriction**: open endpoints, intersections of arbitrary
valence, multiple components, curves not aligned to mesh edges and not
embedded in the surface.

The encoding is the important part:

> we encode each cubic Bézier spline as a tuple of four indices mapping
> to a pool of control points with their respective 3D positions

So the primitive is *(pool of points, list of 4-index tuples)*, and
connectivity is **index sharing**, not proximity. Two splines meet
because they name the same pool entry.

Derived structure, computed once per layout:

- **intersection** — an endpoint shared by **three or more** splines
- **anchor** — an endpoint incident to exactly one spline
- **curve** — a maximal chain of splines bridging intersections and/or
  anchors (an endpoint shared by exactly two splines is interior to a
  curve, not a junction)
- isolated **closed curves** — remaining chains with no labelled endpoint
- per intersection: the emanating curves **sorted counter-clockwise** by
  their tangents projected orthogonal to the normal of the closest
  surface point in the neutral pose

### Sampling

Splines are converted to polylines, "first refined uniformly in
parametric space and then resampled evenly based on arc-length". Count
per spline:

```
n = samplesPerSpline * (length of the spline's control polygon)
                     / (mean edge length of the surface mesh)
```

both measured in the neutral pose, `samplesPerSpline` defaulting to 5.
Because the count is fixed per spline and computed in the neutral pose,
sample *i* of a spline corresponds to sample *i* of the same spline in
any other pose — that one-to-one correspondence is what makes a
deformation gradient definable at all.

A **segment** is a pair of consecutive samples within a curve.

### Frames: normal and width at intersections

Per segment `s`: unit tangent `t_s`, length `l_s`. A lone curve gives no
normal and no width; the net does. At an intersection with ordered
emanating segments `{(t_i, l_i)}`, for each corner of consecutive
tangents:

```
c_i = t_i × t_{i+1}
m_i = c_i / ‖c_i‖                                  (corner normal)
m_i = (c_{i+1} + c_{i-1}) / ‖c_{i+1} + c_{i-1}‖    (parallel tangents, e.g. T-junction)
```

Each segment leaving an intersection gets two of these, one per side:

```
n⁺_i = m_i        (left)
n⁻_i = m_{i-1}    (right)
```

and a width per side, from the corner's opening and the length
difference:

```
w⁺_i = l_i + ‖c_i‖   (l_{i+1} - l_i)
w⁻_i = l_i + ‖c_{i-1}‖ (l_{i-1} - l_i)
```

> the width in each segment side is a positive scale with an anisotropy
> relative to the segment length proportional to the length difference
> and the orthogonality of its intersection corner.

This is where "curvenets are implicitly a net of ribbons" comes from.

### Frames: interpolation along a curve

Per side, independently. Walk from the intersection, parallel
transporting the normal with the smallest rotation `R_i` taking `t_i` to
`t_{i+1}`:

```
Ω_i = Π_{j<i} R_j          n_i = Ω_i n_1
```

- **curve ending at an anchor** — copy the intersection width to every
  segment (uniform width), no torsion correction.
- **curve with intersections at both ends** — transported `Ω_k n_1`
  generally disagrees with the `n_k` the far intersection wants. The gap
  is the curve's torsion:

```
θ  = atan( (Ω_k n_1)·(n_k × t_k) / (Ω_k n_1)·n_k )
α_i = (Σ_{j<i} l_j) / (Σ_{j≤k} l_j)              (normalized arc length)
Θ_i = rotation of α_i·θ about t_i
n_i = Θ_i Ω_i n_1
w_i = (1 - α_i) w_1 + α_i w_k
```

Completing the frame:

```
b_i = n_i × t_i            h_i = sqrt(l_i w_i)
B_i = [ t_i  b_i  n_i ]    S_i = diag(l_i, w_i, h_i)
```

`B_i S_i` is the **scaled frame** of segment side *i*. Note `h`, the
scale along the normal, is the geometric mean of the along- and
across-curve scales — the net never measures thickness, it infers it.

### Deformation gradient

With rest (`˘`) and posed scaled frames in correspondence:

```
F_i = (B_i S_i)(B̆_i S̆_i)⁻¹
    = (l_i/l̆_i)(t_i ⊗ t̆_i) + (w_i/w̆_i)(b_i ⊗ b̆_i) + (h_i/h̆_i)(n_i ⊗ n̆_i)   (Eq. 1)
```

per side. Both sides share the tangential term and differ in twist and
non-tangential stretch — that difference is precisely the hinge.

**Isolated curves** (closed, or both ends at anchors) have no net
structure to lean on: `F_i` = smallest rotation `t̆_i → t_i`, scaled
uniformly by `l_i / l̆_i`.

## §4 — the Profile Mover

Precompute in the projection pose; solve per frame.

```
precompute:  cut-mesh (§4.1);  assemble {L, C, V} (§4.2);  factorize Vᵀ L V (§4.3)
runtime:     sample + frames + gradients (§3)
             → solve for f_v      (Eq. 4)  — harmonic interpolation of the gradient field
             → deform cut-faces   → y_h
             → solve for x_v      (Eq. 5)  — Poisson reconstruction of positions
```

### §4.1 Mesh cutting

A Cartesian cut-cell method adapted to curved surfaces. The cut-mesh
keeps every input vertex and every curvenet sample, and splits faces into
smaller polygons that may be non-planar, non-convex, **and cracked**.

- project each rest sample `q̆` to its closest point `p̆` on the mesh;
  classify as vertex-, edge- or face-sample (tolerance 0.001% of the
  bbox diagonal)
- vertex-samples tag the coincident cut-vertex; edge-samples split the
  cut-edge; face-samples become isolated cut-vertices
- per segment, the sample-pair cases of Fig. 6: existing edge / inside
  one face / crack / traced across faces. The last uses **straightest
  geodesics** (Polthier & Schmies 1998), inserting a cut-vertex at each
  crossed mesh edge.
- rebuild faces: per cut-vertex compute a tangent space (face normal /
  unfolded edge pair / flattened one-ring), project incident halfedges,
  sort CCW, circulate loops into cut-faces
- **cracks** are cut-edges whose two halfedges point at the *same*
  cut-face
- curvenet islands entirely inside a single input face are removed —
  finer than the surface can represent

The residual `q̆ - p̆` (curve floats off the surface) is kept and
transported at runtime; that is why loosely drawn curves still work.

### §4.2 Discretization

Discrete functions live on **face corners** (halfedges), not vertices —
that is what lets a value be discontinuous across a curve. `ϕ_h` has one
entry per halfedge.

Per cut-face, from the corner positions `X_f` (a crack's duplicated
corners make the polygon *simple*), with `D_f` the consecutive-difference
matrix and `A_f` the consecutive-average matrix (Appendix A):

```
E_f = D_f X_f                       [a_f] = E_fᵀ A_f X_f      a_f = ‖a_f‖   n_f = a_f/a_f
G_f = (-1/a_f) [n_f] E_fᵀ A_f                                 (gradient)
Q_f = D_f - E_f G_f                                           (null-space projector)
L_f = a_f G_fᵀ G_f + λ Q_fᵀ Q_f          λ = 1                (Eq. 10)
```

`L_f` is symmetric, PSD, scale-invariant, linear-precise, and reduces to
cotan weights on a triangle. `L_h` gathers every `L_f`.

Solution space: mesh vertices carry one smooth value shared by their
halfedges (`V`, `n_h × n_v`); curvenet samples and segment/edge crossings
carry **two** values, one per side (`C`, `n_h × n_c`, `n_c` = twice the
sample count). Curvenet wins where both apply, so `VᵀC = 0`, and
`V1_v + C1_c = 1_h`.

```
ϕ_h = V ϕ_v + C ϕ_c            (Eq. 3)      E_D(ϕ_h) = ϕ_hᵀ L_h ϕ_h    (Eq. 2)
```

### §4.3 The two solves

Deformation gradients are remapped segments → samples: interior samples
average the two adjacent segments per side; a curve endpoint is
duplicated per incident segment. Flattened row-wise into `f_c` (`n_c × 9`).

```
min_{f_v}  E_D(V f_v + C f_c)                                   (Eq. 4)
F_f        = average of f's corner gradients, unflattened
y_h        = rows of X̆_f F_fᵀ per cut-face                      (deformed rest polygons)
p_i        = q_i - F_i (q̆_i - p̆_i)                              (offset-preserving targets)
min_{x_v}  E_D(V x_v + C x_c - y_h)                             (Eq. 5)
```

both being unconstrained convex quadratics:

```
(Vᵀ L_h V) f_v = -Vᵀ L_h (C f_c)
(Vᵀ L_h V) x_v = -Vᵀ L_h (C x_c - y_h)                          (Eq. 6)
```

One matrix, factorized once, two solves per frame with 9 and 3
right-hand sides. `Vᵀ L_h V` keeps a one-ring sparsity pattern on the
*input* mesh — the cutting does not grow the system.

### §5 — projection pose vs rest pose

The pose the curvenet was drawn in (**projection**) and the pose the
surface arrives in (**rest**) need not be the same. Keeping them apart is
what lets curvenets layer on top of an existing rig, a simulation, or
another curvenet:

- cut-mesh and factorization are built once, in the projection pose
- each frame the curvenet and the cut-mesh are **warped** to the rest
  surface by reusing the cached closest-point binding
- rest scaled frames and rest cut-face polygons are then read off the
  warped configuration

Blend shapes ride on top as vertex offsets reconstructed through vertex
frames, and are deliberately *not* used to fix deformation artifacts.

## The talks

**2023 (Elemental)** adds the animation-facing half:

- **Curvenet Adjustment** — a primitive marking one curvenet knot as
  exposed to animation, with built-in translate/rotate/scale
  manipulators and optional parented tangent controls.
- **Curvenet Adjuster Mover** — a deformer that applies those controls
  after every earlier deformer has fired, so the control frame is
  relative to the *deformed* surface. Orientation per knot:
  best-fit rotation of the incident tangents at each intersection
  (original → warped); elsewhere parallel transport from nearby
  intersections, and between two intersections a blend weighted by
  inverse distance to each.
- confirms the §5 projection/rest split as the shipping configuration.

**2024 (Inside Out 2)** is production experience, not method: a shareable
hand curvenet reused across characters regardless of topology, a
"shape-n-bake" workflow using curvenets as a sculpting proxy that bakes
into the classical stack, and the known weakness — *candy-wrapping under
twist*, matching the paper's own volume-loss limitation (§5: 93% of
volume at 90°, 81% at 180°).

**2026 (Hoppers)** repurposes the same machinery for **weights** instead
of positions: author scalar maps on curvenet points, interpolate to the
mesh with

```
min_x  xᵀ L x + κ ‖B x - S w‖²        κ = 100 × mean mesh edge length
```

`S` = the curve basis' subdivision stencil (control points → samples),
`B` = barycentric coordinates of each sample's closest point. Curves are
**centripetal Catmull-Rom** here so every control point lies exactly on
the surface. Points tagged *auto-smooth* get their value from a discrete
Laplacian solve along the curvenet connectivity instead of by hand.

## Mapping onto RigExec

### Why this fits the engine as it stands

RigExec already has the two things the technique needs and that a
from-scratch implementation would have to invent:

- a **revision chain per target** (`moverGraph.h`): movers write a
  target in namespace order, each reading the previous value. §5's
  "rest pose = the result of any surface deformation performed before
  the curvenet articulation" *is* the incoming chain value. No new
  concept.
- movers that write any `UsdGeomPointBased.points`. Make the curvenet a
  PointBased and its knots are posed by the existing matrix / blend /
  aim machinery — which is exactly the paper's "articulate their knots
  and tangents using traditional rigging tools", with no curvenet-specific
  rigging code at all.

### Schema

```
class RigExecCurvenet : UsdGeomPoints
    point3f[]  points                  # the shared control-point pool (projection pose)
    int[]      rigExec:splineIndices   # 4 per cubic spline, into points
    uniform token rigExec:basis        # "bezier" | "catmullRom"
    uniform int   rigExec:samplesPerSpline = 5
    color3f    guide:displayColor / float guide:displayOpacity / double guide:radius
```

`points` is a native `point3f[]` on a `UsdGeomPointBased`, so it passes
the existing points-target validation unchanged and every mover already
in the engine can pose it. Index sharing carries the net topology,
exactly as the paper encodes it. Knots are `splineIndices[4k]` and
`splineIndices[4k+3]`; handles are the two between.

```
class RigExecCurvenetMover : Typed            # the Profile Mover
    rel rigExec:curvenet                       # exactly one RigExecCurvenet
    uniform token rigExec:restPose = "projection"   # | "preceding"
    uniform token rigExec:curvenetReadPhase = "base"  # | "preceding" | "final"
    float inputs:strength = 1
```

writes the target mesh's `points` like any other geometry mover.
`restPose = "projection"` is the plain §4 formulation; `"preceding"` is
§5, warping the curvenet onto whatever the chain handed in.

Later, for the talks:

```
class RigExecCurvenetAdjustment : RigExecXformable   # 2023 talk
    rel rigExec:curvenet ; int rigExec:knotIndex ; bool rigExec:includeTangents
class RigExecCurvenetAdjusterMover : Typed           # writes the curvenet's own points
class RigExecCurvenetWeight : RigExecWeightObject    # 2026 talk
```

The adjuster writing the *curvenet's* points and the Profile Mover
reading them is one chain in the existing graph — Presto's stack falls
out of the model rather than being bolted on.

### Where the code goes

| Piece | Home |
|---|---|
| Bézier/Catmull-Rom sampling, net topology, corner normals+widths, transport+torsion, scaled frames, `F` per side | `rigExecMath/curvenet.{h,cpp}` |
| cut-mesh, polygonal Laplacian with cracks, `V`/`C`, sparse Cholesky | `rigExecMath/cutMesh.{h,cpp}`, `rigExecMath/sparseSolve.{h,cpp}` |
| bind capture, per-frame solve, packet assembly | `rigExec/moverKernels.cpp` + a `RigExecRevisionOp::Curvenet` |
| drawing the posed net | `rigExecImaging` synthesized children, as guides already do |
| authoring | `plugin/rigExecUsdview/curvenetUI.py` |

There is no sparse direct solver in the USD build (no Eigen, no
CHOLMOD), so `sparseSolve` supplies a supernodal-free sparse Cholesky
with a minimum-degree ordering — factor once per epoch, reuse for the 12
right-hand sides per frame.

### Two engine constraints this has to respect

1. **A points-writing mover applies asset-space values to a gprim's raw
   `points`** — there is no local-to-asset conjugation anywhere in the
   chain (see `docs/`… and the `chars/puppetA` rig-space bake). The
   curvenet's `points` and the target mesh's `points` must therefore be
   in the *same* space, which for a real asset means both identity
   relative to the asset root.
2. **Derived normals need vertex interpolation.** faceVarying normals
   fail the cardinality check and pass through stale. A curvenet-driven
   mesh needs `normals` with `interpolation = "vertex"` or
   `rigExec:derived = off`.

### Degeneracies the paper does not discuss, and what this does

- **A mesh component no curve touches** leaves `VᵀL_hV` singular on that
  component (constants are in its null space). Detected per component;
  such components are left at rest and diagnosed, rather than being
  handed an arbitrary solution.
- **Curvenet with no intersections at all** — every curve is isolated, so
  frames fall back to the rotation-only branch of §3. Legal, and the
  right answer, but worth surfacing in the UI since it usually means the
  artist meant to weld two endpoints.
- **Zero-length segments** (coincident samples) make `t_s` undefined.
  Rejected at bind with the offending spline named.

## The usdview authoring tools

The paper's toolkit — "insert control points at arbitrary locations on
the surface and click-and-drag curves", plus "split and merge splines,
weld and break control points, project endpoints to the surface mesh,
and flatten tangents" — is the specification for the panel.

`plugin/rigExecUsdview/curvenetUI.py`, opened from the RigExec menu,
following `volumeWeightUI.py`'s shape (a singleton Qt panel authoring
directly to the stage, with an `Usd.Notice.ObjectsChanged` listener
keeping the UI honest).

**Viewport interaction.** `stageView.computePickFrustum(x, y)` +
`stageView.pick(frustum)` returns hits carrying `hitPoint` and
`hitNormal` in world space. That is the whole enabler: a click on the
model is a 3D point on the surface. Note that `pickObject()`'s
`signalPrimSelected` is *not* usable for this — it overwrites the hit
point's x/y with scaled mouse coordinates — so the panel installs an
event filter on the stage view and calls `pick()` itself.

Two things that path gets wrong if copied naively, both of which present
as "draw mode does nothing":

- **Pick coordinates are PHYSICAL pixels.** `computePickFrustum` divides
  by `computeWindowViewport`, which is in device pixels, while Qt reports
  logical ones. usdview's own `mousePressEvent` bridges that by
  multiplying by `devicePixelRatioF()` — "only necessary because this is
  a QGLWidget" — and so must anything picking by hand. Omitting it costs
  nothing at ratio 1.0 and, on a HiDPI display, sends every pick to a
  fraction of where the artist clicked, which is usually off the model.
  `tests/testUsdviewCurvenetDraw.py` asserts the convention against
  `devicePixelRatioF()` directly, so it holds at any ratio rather than
  passing by accident at 1.0.
- **Alt and Meta must reach usdview.** They are its camera-manipulation
  modifiers; an editing filter that eats them leaves the artist unable to
  orbit while drawing, which on a character is fatal.

And a click that misses the model has to *say so*. Silence there is
indistinguishable from a broken tool.

**Selection is a SCREEN-space query, not a world-space one.** Matching a
click to a knot by world distance cannot be made to work: any tolerance
small enough to separate neighbouring knots is a handful of pixels on
screen, any tolerance large enough to hit reliably swallows the
neighbours, and the meaning changes with every camera move. The panel
projects the pool through the stage view's own camera — the exact
inverse of `computePickFrustum`'s mapping, so the two agree by
construction — and ranks candidates within a pixel radius by:

1. **screen distance, in ~2px buckets** — what the artist is aiming with;
2. **knot over handle** — within a bucket, the structure wins;
3. **nearer the camera** — a net wraps round a head, and a ring seen down
   its axis projects its front and back knots onto the *same pixel*.

Getting that order wrong fails in both directions, and both were tried:
depth first lets a tangent handle a millimetre nearer the eye steal every
click aimed at its own knot, and an absolute knot-over-handle rule makes
handles unreachable altogether — on a real net every handle has a knot
within the pick radius — so tangents can never be shaped.

Selecting also must NOT require the pick ray to hit the mesh. Knots float
just off the surface by design (§3), so a click on a knot near the
silhouette misses the geometry, and demanding a surface hit made exactly
those knots unselectable.

**The click radius has to come from the geometry.** It decides whether a
click lands on an existing knot (and welds) or places a new one, and a
stage-unit fallback — `metersPerUnit * 5` — is 5.0 on a character 0.6
units tall. On a *new* net, which has no bounds of its own yet, that made
every click after the first "hit" the first knot and weld to it: no second
knot, no spline, and nothing ever drawn. It now measures the net's own
bounds, then the bound mesh's extent, then the stage's.

**The viewport aids are defined once and thereafter only edited.** Adding
or removing a prim resyncs the stage, and on a stage carrying a
`RigExecRoot` every resync re-evaluates the rig — so create/destroy per
redraw meant a full rig evaluation per knot placed. They are authored
into the session layer with `purpose = "default"` and hidden with
`visibility`, never removed.

> **Known engine defect, pre-existing and not curvenet-specific.**
> Removing *any* prim from a stage with an active RigExec rig posts
> `Usd_PrimFlagsPredicate: Applying predicate to invalid prim`, which
> Python then raises out of whatever call comes next. Reproduced with a
> bare `UsdGeom.Points` prim: it throws on `puppetA_curvenet.usda` and
> not on the rig-free `puppetA_model_rigspace.usda`. The authoring panel
> no longer removes prims, so it does not hit this, but anything else
> that deletes a prim on a rigged stage will. Origin not yet located —
> the `_PrimsRemoved` handlers in `rigExecImaging/sceneIndices.cpp` are
> Hd-side only, so the invalid access is a USD traversal elsewhere.

Modes:

| Mode | Interaction |
|---|---|
| **Draw** | click places a knot on the surface; drag out of it sets the outgoing tangent handle (pen-tool convention, handles mirrored); clicking an existing knot within tolerance **welds** to it, which is how intersections are made; `Esc`/double-click ends the chain |
| **Select** | click picks knots/handles; marquee for many; shows valence and the derived label (intersection / anchor / interior) |
| **Move** | drag re-projects onto the surface by default, free-drag with a modifier, so knots stay near the mesh as §3 assumes |
| **Tangent** | drag handles only; `flatten` projects them into the surface tangent plane, `mirror` keeps G1 across a knot |

Structural operations, all straight edits to `points` +
`rigExec:splineIndices`: weld / break, split spline at parameter, merge
two splines at a shared 2-valence knot, delete spline, project knots to
surface, reverse, close loop.

**Feedback the artist needs and the papers imply.** The panel computes
the §3 derived structure live and shows it, because the deformation is
unreadable without it:

- intersection / anchor / interior classification per knot, colour-coded
- curve decomposition (how splines grouped into curves)
- **scaled frames drawn as the paper's deformed boxes** (Fig. 5), red-green-blue
  on the left side and cyan-yellow-magenta on the right — this is the
  single most diagnostic overlay, since it shows the inferred normals and
  widths that nobody authored
- sample count and its ratio to the mesh's mean edge length, with a
  warning when a spline is sampled below the mesh resolution
- residual `‖q̆ - p̆‖` per sample, flagged when a curve drifts far from
  the surface

**Binding.** A "Bind to mesh" action creates the `RigExecCurvenetMover`
under the rig, targets the selected mesh, runs the cut, and reports the
cut-mesh statistics and any component the net failed to reach — the
errors that matter are all at bind time, not at solve time.

**Drawing the net.** The panel regenerates a `UsdGeomBasisCurves` under
the curvenet prim in the **session layer** on every edit, with
`purpose = "guide"`. Session-layer means it never touches the artist's
file and nothing downstream can come to depend on it. It draws the net as
*authored*, not as posed — publishing the posed net through the imaging
side would follow the guide-synthesis pattern and is the natural next
step, but authoring feedback is what the tool is for.

## What the implementation found

Six things the papers do not say, each of which was a wrong answer on the
way to a right one. All are locked by `tests/testRigExecCurvenet.cpp`.

**1. Intersections must be oriented by the SURFACE, not by themselves.**
§3 sorts each intersection's fan about "the normal of the closest surface
point", and it reads like a convenience — any consistent plane would do
for a sort. It is not. Orientation derived from a fan's own geometry is
sign-arbitrary *per intersection*, so two ends of a curve can disagree by
a half turn; the curve then measures a 180° discrepancy between its two
end normals, calls it torsion, and twists the surface through a right
angle in the middle. The surface normal is the only globally consistent
choice available. `RigExecBindProfileMover` re-orients every net against
the mesh it is about to deform before it does anything else.

Worse, the obvious fallback is degenerate exactly where it matters: for a
four-valence intersection whose spokes come in two antiparallel pairs —
what any grid-like net produces at *every* junction — the sum of
consecutive cross products cancels to precisely zero. The surface-free
path now takes the least-variance direction of the spokes instead.

**2. The residual has to be measured from the cut-vertex, not the
sample.** §4.3 gives `p_i = q_i − F_i(q̆_i − p̆_i)` per sample. Cutting
introduces vertices *between* samples, and interpolating the finished
`p_i` between the two neighbours is not equivalent: on a curved surface a
traced crossing does not lie on the straight chord between the two
samples' projections, so at rest the interpolated constraint pulls the
vertex off itself and the mover stops being the identity. Measuring the
residual from each cut-vertex's own rest position is exact for both — at
a sample the node *is* that sample's projection, so it reduces to the
paper's formula unchanged.

**3. A curve traced along an edge loop is the common case, not a corner
case.** Riggers align profiles to edge loops, so most segments run along
mesh edges rather than across faces. Such a segment belongs to *both*
adjacent faces — the edge is the two sides of the curve there — and
routing it as an interior chord instead lays an edge on top of the face
boundary, which makes the angular sort at the shared vertex ambiguous and
loses the face outright. Segments spanning two collinear edges are split
at the vertex between them and handled as two such hops.

**4. A traced crossing that lands on a vertex IS that vertex.** Creating
a separate coincident cut-vertex leaves the arrangement's angular sort
meaningless there. On the tube example this alone accounted for 27 lost
input faces, 101 phantom cracks, and a singular system; snapping them
took the cut to exactly 192 cut-faces from 192 input faces with no
cracks.

**5. The unknown set follows the CORNERS, not the samples.** §4.2's
precedence rule ("a cut-vertex incident to both a mesh vertex and the
curvenet belongs to the curvenet") is about corners. A curve running
along an edge loop passes through vertices no sample lands on, and every
corner there still reads a constraint — so testing "did a sample land
here" gives such a vertex an unknown whose row is entirely empty, and the
factorization fails on a zero pivot.

**6. `atan`, in the torsion formula, has to be the two-argument one.**
θ = atan(y/x) folds any disagreement past 90° back into (−π/2, π/2), so
the 180° twist the paper itself illustrates (Fig. 12) would receive no
correction at all. `atan2` is used.

Two smaller notes: mesh components no curve reaches leave the system
singular, so they are found explicitly, held at rest and reported rather
than regularized into an invented answer; and the Profile Mover has no
scalar parity oracle, because every other mover in the engine is a few
lines of arithmetic that can honestly be written twice, while a second
"independent" mesh cut plus two sparse solves would be the same code with
the same bugs. The parity pass says so instead of reporting a mismatch it
cannot explain.

## What is not implemented

- **Curvenet Adjustments and the Adjuster Mover** (2023 talk): the
  auto-oriented per-knot manipulator. The mover chain already supports it
  — a mover writing the curvenet's own points, read by the Profile Mover
  — and the evaluator runs curvenet chains first for exactly that reason.
- **Curvenet parametrization** (2026 talk): weight maps authored on
  curvenet points and interpolated to the surface. It reuses this
  Laplacian and the sample-to-surface projection matrix directly.
- **Catmull-Rom** is implemented in the sampler and selectable on the
  schema, but no example uses it and no test covers it beyond sampling.
- **Blend-shape superposition** on top of a curvenet deformation (§5).
- Publishing the *posed* net to the viewport through the imaging side.
