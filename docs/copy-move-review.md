# Copy/Move and Per-Frame Allocation Review

Scope: erroneous copy/move operations and avoidable memory- or
speed-intensive operations in the C++ sources (`libs`, `plugin`,
`python`, `tools`; 189 files). Each finding cites a location and a
proposed change. Severity is relative to per-frame evaluation cost:
High = runs per frame (or per revision/vertex/CV) with significant
allocation or copying; Medium = per-frame but small, or per-evaluation
moderate; Low = cold paths and cleanups.

Status: v33 final -- codex-astra review converged at round 33 (NO NOTES) (see Revision history).

## High

### H1. Topology-derived structures rebuilt on every kernel call

The smooth, delta-mush, and wrinkle kernels rebuild neighbor/edge
topology from `counts`/`indices` on every invocation, although the
connectivity is constant across frames:

- `libs/rigExecMath/geometryKernels.cpp:64-94` (`_BuildAdjacency` builds
  `vector<set<int>>`, then converts to `vector<vector<int>>`) called
  from `RigExecApplyLaplacianSmooth` at `:248`.
- `libs/rigExecMath/deltaMushKernel.h:165-187` (per-call
  `std::map<pair<int,int>,int>` edge table plus per-vertex neighbor
  vectors) and the second rebuild inside the transport kernel at
  `:107` (`deltaMushDetail::BuildAdjacency`).
- `libs/rigExecMath/wrinkleKernel.h:109-146,158-165` (two
  `std::map` edge tables, one `std::set` per face, per-vertex neighbor
  vectors).
- Runtime twins: `libs/rigExecRuntime/geometry.cpp:1571`
  (`RrGeoBuildAdjacency` per call); delta-mush/wrinkle share the same
  template kernels (`:2596`, `:2601`).

Production callers run per revision per frame
(`libs/rigExec/moverGraph.cpp:936-945`,
`libs/rigExecRuntime/geometry.cpp:2592-2601`).

Proposed change: precompute connectivity once per topology epoch,
cache it in revision/baked state, and pass it to the kernels by const
reference. Preserve dynamic/baked parity: the live, baked, and runtime
paths must share the cached representation. Constraints the cache must
respect: traversal order is semantic (wrinkle's sequential constraint
updates at `wrinkleKernel.h:327-338` over map-ordered edges from
`:177`; delta-mush relies on index-ordered neighbors,
`deltaMushKernel.h:122-124`), so use order-preserving sorted-vector
builds rather than unordered containers; cache connectivity separately
from rest-dependent weights (`deltaMushKernel.h:183-186`, where
neighbor weights derive from rest lengths).

### H2. Kernel signatures force per-frame array conversions

Geometry kernels take `std::vector` (for example
`libs/rigExecMath/geometryKernels.h:54-58`), so callers holding USD
arrays convert on every evaluation. Per-frame instances:

- `libs/rigExec/frozenGeometry.cpp:37` (`auxPoints.assign` copies the
  full base point set per derived revision per frame), `:55`, `:75`
  (`VtIntArray`/`VtFloatArray` to `std::vector` per frame).
- The mover-oracle branches copy points/counts/indices the same way
  (for example `libs/rigExec/movers/smoothMover.cpp:70-84`,
  `curveMover.cpp:351-370`, `latticeMover.cpp:140-144`), but those are
  parity-reference paths, not production evaluation.

Proposed change: migrate kernels to span-style inputs (pointer plus
size, or `TfSpan<const T>` from `pxr/base/tf/span.h`) so movers pass
`VtArray::cdata()`/`size()` with zero copies. Spans keep the math
layer free of stage access. This is a
structural migration across kernel signatures AND packet/provider
storage: `RigExecMoverParameters` owns its vectors
(`libs/rigExec/types.h:290-298`), so changing kernel arguments alone
leaves the assemble-time copies intact. Owners must keep span storage
alive across execution and subsequent retained-input comparisons. H1's
caching composes with it.

### H3. Frozen/baked replay copies the resolved-inputs map per phased revision per frame

`libs/rigExec/frozenGeometry.cpp:1133`, its baked twin
`libs/rigExec/bakedGeometry.cpp:1513`, and the dynamic path at
`libs/rigExec/rigEvaluatorGeometryEvaluation.cpp:292-295`
(`revisionInputs = _resolvedInputs;` inside the optional overlay):

```cpp
revision.revisionInputs = *B.resolvedInputs;  // guarded by !binding.phases.empty()
```

`RigExecResolvedInputs` holds an
`unordered_map<SdfPath, VtValue>` (`libs/rigExec/moverGraph.h:525`),
so every phase-declaring revision per frame copies the map and its
entries (value payloads are shared, not deep-copied -- `VtValue`
carries remote payloads by reference-counted handle), only to overlay
a few phased entries in the loop below.

Proposed change: share the per-frame base as immutable state with a
small revision-owned overlay on all three paths, adding chained lookup
(overlay first, then base) to the input reads. Do NOT apply-and-roll-back
on a shared map: revisions can execute concurrently, and the ownership
comment at `bakedGeometry.cpp:1509-1511` explicitly requires
revision-owned buffers for this reason.

### H4. Baked last-parameters snapshot deep-copies vectors per execution

`libs/rigExec/bakedGeometry.cpp:2444`:
`revision.lastParameters = revision.parameters;` copies every
parameter vector per revision per execution for next-run change
detection (compared elementwise at `:2198`).

Proposed change: where the source packet is reassembled or dead after
the snapshot, move instead of copy (as the derived path already does
at `:2027-2029`); otherwise carry versioned immutable inputs so the
"unchanged" test does not need a retained copy. Keep exact equality --
a digest comparison alone would change invalidation behavior on
collision. (The adjacent `lastAuxPoints` assignment at `:2030` is a
`VtVec3fArray` handle share, not a copy; see Cleared.)

### H5. Read-only mutable VtArray iterators trigger full-array detach copies

`VtArray::begin()/end()` on a non-const array detach shared storage
(`usd-install/include/pxr/base/vt/array.h:363-368,401-403`: mutable
`begin()` goes through `data()` and `_DetachIfNotUnique()`). These
read-only conversions use the mutable iterators on shared arrays:

- `libs/rigExec/bakedGeometry.cpp:1642` (`sample.points.assign` after
  the shared assignment at `:1638`), `:2001-2002`, `:2016-2017`
  (`derived.lastBase.begin()/end()`).
- `libs/rigExec/rigEvaluatorGeometryEvaluation.cpp:593` (after the
  shared assignment at `:588`).
- The dynamic base points: `basePoints` (`VtVec3fArray`, `:200`)
  shares storage at `:212`/`:229`, then `:535` converts with mutable
  iterators -- detaching the mesh before copying it -- and the
  projector conversion at `:730-731` can trigger the same detach when
  no earlier consumer did.
- `libs/rigExec/frozenGeometry.cpp:1402`, `:1409` (derived twin).
- `libs/rigExec/bakedWeights.cpp:373` (`out->insert` with mutable
  iterators): the local can share cached storage
  (`libs/rigExec/moverGraph.h:287`) and capture retains it
  (`:369-372`) -- detaching before copying. Runs for target,
  sample, and curve points during weight-packet rebuilding.
- The shared production array assembler `moverGraph.cpp:1619-1656`:
  `resolved->Get` shares storage into the mutable local at `:1630`
  (via `moverGraph.h:402`), then `:1631` converts with mutable
  iterators -- detaching before copying. This is the central instance:
  every assembled topology/rest/surface array on the dynamic and baked
  paths. The stage-read arm at `:1654` can detach too: the recorder
  retains `VtValue(value)` at `:1644-1645` (`moverGraph.h:361`), so
  during capture the buffer is shared at conversion time. Use const
  iterators in both arms.

Each site pays a full detach copy plus the vector copy, per sample or
per derived revision per frame. (On the derived paths the failure-arm
assign -- `bakedGeometry.cpp:2016-2017`, `frozenGeometry.cpp:1409` --
copies again but does not detach a second time: nothing re-shares the
buffer between the initial conversion and the fallback.)

Proposed change: use `cbegin()/cend()` (or `cdata()`/`size()`) for
every read-only access. Sweep the evaluation paths for further mutable
`begin()/data()` uses in read-only positions; unique ownership makes
the detach a no-op, but shared snapshots are exactly where this
pattern appears.

### H6. Delta-mush repeats the entire rest-side smoothing solve per deformation

`libs/rigExecMath/deltaMushKernel.h:207`:
`auto smoothRest = smooth(rest), smoothPosed = smooth(*points);`
The rest half repeats all smoothing iterations and their allocations
on every call, although rest inputs typically do not change between
frames. The shared template serves dynamic, baked, and runtime
evaluation, so one fix covers all three.

Proposed change: cache the smoothed rest result (with the rest-derived
neighbor weights from `:183-186`, per H1's split) keyed by resolved
rest points, topology, iterations, step, border policy, and distance
weighting; re-solve only when the key changes. Key comparison itself
is O(vertices), still cheaper than the O(iterations x vertices x
degree) solve it skips. Preserve validation and arithmetic order.

### H7. Surface projection scans every triangle for every destination point

`libs/rigExecMath/geometryKernels.cpp:520-544` nests the per-point loop
over every face and fan triangle before `_ClosestPointOnTriangle`;
the runtime duplicates it at
`libs/rigExecRuntime/geometry.cpp:1788-1809`. Cost is O(points x
triangles) per evaluation with no reuse across frames.

Proposed change: add reusable triangulation plus a nearest-triangle
acceleration structure, rebuilt/refit when topology or points change;
keep the existing scan as fallback. Preserve the triangle arithmetic,
first-triangle tie behavior, and malformed/non-finite-input behavior.

### H8. Runtime retained inputs deep-copy vectors per execution

The runtime counterpart of H4: `libs/rigExecRuntime/geometry.cpp:5552`
(`rev.lastParameters = rev.parameters`) copies the ordinary packet per
revision per execution. Worse, the derived path at `:5142-5144`
clears the assembled `auxPoints` and then executes
`rev.lastAuxPoints = chain.result` -- where `lastAuxPoints` is an
owning `std::vector<RrVec3f>` (`:2812`), so unlike the USD side's
`VtVec3fArray` handle share this deep-copies the whole chain result
every execution.

Proposed change: move the assembled auxiliary vector into retention
before emptying the retained packet (verifying it holds the same
content `chain.result` would contribute), and apply H4's
ownership-qualified move to the ordinary packet. Keep exact
comparisons.

### H9. Runtime publication deep-copies whole meshes six times per chain

Each chain publication copies the result mesh repeatedly: the initial
materialization into `chain.result`
(`libs/rigExecRuntime/geometry.cpp:5210-5214`),
`final.points = chain.result` (`:5218`), `chain.final = value`
inside `RecordFinal` (`libs/rigExecRuntime/values.h:235-240`),
`chainPublish[...].result = chain.result` (`geometry.cpp:5222`),
`movedProperties[...] = chainPublish[...].result`
(`libs/rigExecRuntime/publish.cpp:239-240`), and the final public
assembly `moved.points = entry.second`
(`libs/rigExecRuntime/exec.cpp:311`). All are owning vectors, and the
chain-status path runs even when revision results are reused.

Proposed change: share immutable result storage across
snapshots/publication, preserving snapshot lifetimes across
subsequent evaluations; as immediate steps, give `RecordFinal` a
move-taking overload to remove the temporary-to-snapshot copy, and
move from the `movedProperties` entries during public assembly --
that map has no readers after assembly (only the assembly loop at
`exec.cpp:307-308` reads it) and is cleared at the next publish
(`publish.cpp:201`), so moving preserves the successful-output
commit.

## Medium

### M1. Wire-driver measurement copies two matrices per CV

`libs/rigExec/moverGraph.h:1152-1168`
`RigExecMeasureWireDriver(GfMatrix4d transform, GfMatrix4d space, ...)`
takes 256 bytes of matrices by value and is called once per CV per wire
evaluation (`moverGraph.h:1211`, `:1219` via `:1198`,
`libs/rigExec/movers/curveMover.cpp:317`).
Only the unscale branch mutates the copies.

Proposed change: take `const GfMatrix4d &` parameters and copy into
locals inside the `posedPoints || posedDelta` branch.

### M2. Envelope resolution allocates a temporary per sparse resolve

`RigExecWeightPacket::ResolveAll`'s sparse arm
(`libs/rigExec/types.cpp:128-132`) fills a fresh `valuesOut` vector
and swaps it into the destination on every call. All validation exits
sit above that point, so the atomicity the swap protects (no partial
write on failure) already holds.

Proposed change: write directly into `*resolved` (assign the default,
then scatter), keeping the temporary only when `resolved == &values`:
no current caller aliases the output with the packet's own weights
(all call sites pass distinct locals or members), but the temp-plus-swap
is what makes such a call work today, and direct writes would clobber
the source weights mid-scatter. Separately, the dense caller at
`libs/rigExec/moverGraph.cpp:501` zero-fills `weights(count)` and then
`ResolveAll` assigns over it -- declare the vector empty and let
`ResolveAll` size it in one pass. (The other production callers at
`moverGraph.cpp:823,900` already declare empty vectors.) The runtime
twins both halves: temp-plus-swap at
`libs/rigExecRuntime/geometry.cpp:376-380` (per-frame callers at
`:2275`, `:2320`, `:2728`, `:5324`, `:5359`) and the redundant sizing
at `:871-872`. Apply the same direct-write plus alias safeguard
there.

### M3. (Fixed in tree) Disabled profile scopes took three mutexes each

`_enabled` is now `std::atomic<bool>` with a lock-free `IsEnabled()`
(`libs/rigExec/profiler.h:104`, `:477`), so the macro's three checks
no longer acquire mutexes. See Cleared.

### M4. Missing move into frozen out-parameter

`libs/rigExec/frozenGeometry.cpp:83`: `*parameters = params;`
copies the assembled packet into the out-parameter; the packet holds a
full `auxPoints` copy plus topology, and `params` is dead after.

Proposed change: `*parameters = std::move(params);`. (The twin
assignment at `:34` is unreachable: the guard tests the validity of
`Constant(1.0f)`, which `types.cpp:44` makes valid unconditionally.)

### M5. Frozen skin packet copied per revision per frame

`libs/rigExec/frozenGeometry.cpp:1151`:
`revision.parameters = inputs.revisionPackets[...];` copies the skin
packet per skin revision per frame, with cached layouts included.
Epoch-fixed layouts already avoid the layout arrays via the shared
`skinTopology` handle (`libs/rigExec/types.h:332-336`,
`moverGraph.cpp:1871-1897`); what still copies every frame is the
packet shell plus the transform table -- and that table is the FIXED
identity table (`bakedGeometry.cpp:829-830`), passed to the assembler
at `frozenSampling.cpp:1644-1647` and copied at
`moverGraph.cpp:1807`. The real folded joint transforms travel
separately. Full index/weight vectors copy on top only when the
topology cache refuses the mover.

Proposed change: share the fixed identity table (or a static identity
source) instead of copying it per frame, and narrow the remaining copy
to what varies per frame -- envelope state, enabled/valid state
(`moverGraph.cpp:1790-1795`), and the resolved skinning method
(`:1836-1849`); note `inputs` is `const RigExecFrameInputs &`
(`frozenGeometry.cpp:1094`), so moving requires an ownership or API
change.

### M6. TfToken interning on per-eval paths (dynamic geometry fixed)

`TfToken(name)` construction does a registry lookup; these sites pay it
repeatedly:

- Mover parameter builds: `params.kind = TfToken("smooth")` and twins
  (`movers/smoothMover.cpp:31`, `blendShapeMover.cpp:40`,
  `latticeMover.cpp:32`, `deltaMushMover.cpp:19`, and the other
  `_Build*Parameters` functions).
- Imaging guide sync: `libs/rigExecImaging/bridge.cpp:249,259,272,277,`
  `425,441,492,749` and similar.

(The dynamic-geometry examples are fixed: file-static tokens at
`libs/rigExec/rigEvaluatorGeometryEvaluation.cpp:33-37`, used at
`:478`, `:546`, `:553`.)

Proposed change: hoist each to a file-static `const TfToken`, matching
the existing convention (`libs/rigExec/moverGraph.cpp:2842-2846`;
  `libs/rigExec/rigEvaluatorInternal.h:102-114`;
  `frozenGeometry.cpp:1411` already does this locally).
Related: `GetPrimAtPath` per blend input/sample per frame
(`rigEvaluatorGeometryEvaluation.cpp:545,552`) could cache prims per
epoch; counts are small, so this is the minor half of the finding.

### M7. Derived publish converts vector to VtArray per execution

`libs/rigExec/bakedGeometry.cpp:2039-2042` resizes `derived.spare`
(a `VtVec3fArray`) and copies `revision.output` (a
`std::vector<GfVec3f>`) into it on every publication, then swaps;
the frozen path repeats it at
`libs/rigExec/frozenGeometry.cpp:1428-1431`. The publish sits outside
the recomputation guards (`bakedGeometry.cpp:1993`,
`frozenGeometry.cpp:1399`), so the copy runs even when the revision
was skipped and `revision.output` is unchanged. The output must stay
retained, and the containers differ, so neither a move nor an
in-place publish applies as written.

Proposed change: retain `derived.result` untouched when no
recomputation occurred -- eliminating the cache-hit copies needs no
type unification. For the recomputed case, unify the publication
storage types so the kernel writes into (or shares with) the
published buffer. H2's span migration subsumes the latter.

### M8. Wrinkle allocates mesh-sized scratch inside iteration loops

`libs/rigExecMath/wrinkleKernel.h:278-279` runs
`auto next = phase;` on each of 10 smoothing passes, and `:348-349`
runs `auto next = deltas;` per smoothing iteration. Each copy is
mesh-sized and the loops already swap buffers (`phase.swap(next)`).

Proposed change: allocate each alternate buffer once outside its loop
and swap between passes. Preserve unchanged values for zero-weight,
pinned, and isolated vertices. The shared template fixes dynamic,
baked, and runtime paths together.

### M9. Lattice recomputes identical Bernstein factors in nested loops

`libs/rigExecMath/geometryKernels.cpp:455-460` recomputes
`_Bernstein(dx - 1, a, uvw[0])` for every `(b, c)` and the Y factor
for every `c`; each call rebuilds the coefficient and calls `pow`
twice (`:399-406`). Per point the nesting evaluates dz + dy x dz +
dx x dy x dz factors where dx + dy + dz suffice. The runtime
duplicates it at `libs/rigExecRuntime/geometry.cpp:1728-1733`.

Proposed change: compute the three axis-factor tables once per point,
retaining the existing product and accumulation order; mirror in the
runtime twin.

### M10. Dense blend samples copied into bake-retention storage every frame

`libs/rigExec/bakedGeometry.cpp:1642-1643` assigns the sampled points
both to the evaluation packet and to `boundSample.lastPoints`, a
`std::vector<GfVec3f>` (`bakedProgramImpl.h:2105`) read only by bake
capture (`libs/rigExecBake/capture.cpp:501-503`).

Proposed change: retain only while a capture is active (skip the
assign otherwise), or store the sampled `VtArray` by shared handle and
adapt capture to read it. Either removes a full-sample copy per sample
per ordinary frame.

### M11. Frozen projector evaluation detaches both input arrays to read them

`libs/rigExec/frozenGeometry.cpp:1378-1379` builds the projector's
input vectors from `chain.lastBase.begin()`/`chain.result.begin()`
through a mutable chain reference, so each shared buffer detaches
before it is copied. The result buffer is published-shared
(`bakedGeometry.cpp:2082`: `VtValue(chain.result)`). The baked twin at
`bakedGeometry.cpp:2729-2741` takes a const chain, so the same
expressions resolve to the const iterators and do not detach -- and
the line directly below the frozen site (`frozenGeometry.cpp:1386`)
already uses `chain.result.cdata()` correctly.

Proposed change: use `cbegin()/cend()` at `:1378-1379` immediately,
then consider span inputs per H2.

### M12. Baked assembly copies base points for operations that never consume them

`libs/rigExec/bakedGeometry.cpp:1574-1577` copies the full base point
set for every operation except Matrix, Skin, and Wire. Smooth,
SurfaceProject, Ribbon, and EmitGuidePoints never read it in their
assembly branches (`libs/rigExec/moverGraph.cpp:2209-2214`,
`:2328-2358`: topology, driver frames, and bind coordinates only),
and the dynamic path already skips them with its explicit consumer
predicate (`rigEvaluatorGeometryEvaluation.cpp:524-533`).

Proposed change: use that same consumer predicate in the baked
assembly, preserving the blend-input and external-handler
requirements. Removes a mesh-sized allocation plus copy per affected
revision per frame without touching any kernel. The runtime twin at
`libs/rigExecRuntime/geometry.cpp:4076-4078` has the same
Matrix/Skin/Wire-only exclusion while its Smooth (`:4092-4099`),
SurfaceProject (`:4178-4188`), and Ribbon/EmitGuidePoints
(`:4189-4194`, which never receive the `base` local) branches ignore
the copy -- extend the predicate there too. The runtime External
branch (`:4204-4205`) likewise ignores it: `RrGeoAssembleExternal`
(`:4005-4042`) never reads `base` or `in.basePoints`. Exclude
External from the runtime copy predicate (runtime-specific; the
USD external-handler inputs are preserved).

### M13. Asymmetric sphere guides regenerate fixed mesh geometry per refresh

`libs/rigExecImaging/bridge.cpp:493-528` rebuilds a 64 x 32 sphere's
points, normals, and topology -- including repeated trigonometry per
vertex -- on every call, invoked per volume guide from
`_FillVolumeGuides` (`:1570-1582`) during each publication (`:2718`,
`:2935`).

Proposed change: cache the unit-sphere topology and samples, retaining
shaped geometry keyed by directional scales; update placement, radius,
and display state separately while respecting animated shape inputs.

### M14. Wrinkle phase weights allocate per-vertex buffers every solve

`libs/rigExecMath/wrinkleKernel.h:265` builds
`std::vector<std::vector<double>> phaseWeights(pointCount)`, then
`:274` pushes per-neighbor weights with no reserved capacity: one
allocation per connected vertex per solve (plus growth reallocations),
separate from H1's connectivity rebuild and M8's iteration buffers.

Proposed change: reserve `neighbors[i].size()` per row, or use one
contiguous weight buffer with neighbor offsets. Preserve neighbor
order; the values depend on the deformed `incoming` points
(`:271-275`), so they must still be recomputed each call. The shared
template preserves dynamic/baked/runtime parity.

### M15. Wire assembly repeats identical matrix inversions across CVs

`RigExecPoseWireDrivers` (`libs/rigExec/moverGraph.h:1182-1234`) calls
`measured()` per CV (`:1211`, `:1219`), and each measurement inverts
(`:973` `transform * space.GetInverse()`, `:1008`
`space.GetInverse() * transform`) plus a scale extraction. Driver
selection (`:1190-1192`: `count <= 1 ? 0 : j % count`) maps many CVs
onto few pairs -- one pair for the common single driver/space -- so
the same inversion and products repeat per CV. M1's reference
parameters do not remove this work. The runtime twins it at
`libs/rigExecRuntime/geometry.cpp:3856-3905`; the curve mover's
per-CV lambda (`libs/rigExec/movers/curveMover.cpp:300-321`) has the
same shape.

Proposed change: cache each selected pair's measured matrix and scale
within the assembly call, preserving multiplication order and frame
options; mirror in the runtime twin.

### M16. Dynamic derived-cache retention deep-copies vectors

`libs/rigExec/rigEvaluatorGeometryEvaluation.cpp:819-823` copies
`parameters.auxPoints`, both topology arrays, and `widths` into the
deferred cache after each successful derived recomputation (the
adjacent `:820` base assignment is a `VtVec3fArray` handle share).
The local packet is dead after graph update/evaluation (`:804-813`),
so nothing past retention reads it.

Proposed change: make the local packet non-const and move the vectors
into the cache; retain the existing exact comparisons.

### M17. Sparse-wire cache hits rehash and compare full inputs under a lock

`_CachedWireBasis` (`libs/rigExec/moverGraph.cpp:383-415`) hashes the
bind coordinates, indices, knots, and scalars on every call
(`:391-399`), then compares the retained arrays under a process-wide
static mutex (`:401-413`) -- including on a hit. Every sparse-wire
execution pays this (`:1021`). The runtime repeats the
hashing/comparison without the mutex
(`libs/rigExecRuntime/geometry.cpp:2364-2394`).

Proposed change: retain the validated basis per revision with owned
immutable keys or versioning, so a hit is a handle share with no
rehash, no array walk, and no lock; preserve exact invalidation and
the collision checks.

### M18. Surface projectors compute whole-mesh normals twice per solve

Each `RigExecSurfaceFrameAtHitT` call computes whole-mesh normals
(`libs/rigExecMath/surfaceProjectorKernel.h:177-178`), and a normal
solve invokes it for both the base surface
(`:350-355`) and the posed surface (`:377-379`; the reproject-miss arm
at `:362-368` can add a third).

Proposed change: cache base normals by resolved points/topology and
share posed normals among projectors using identical inputs. Preserve
each adapter's normal arithmetic and validation; the shared template
serves live, baked, frozen, and runtime paths.

### M19. Dense-wire deformation re-evaluates the rest curve per point

`RigExecApplyWire` evaluates `restCurve.Evaluate(u)` per destination
point (`libs/rigExecMath/geometryKernels.cpp:944-974`, evaluation at
`:970`; the runtime twin at
`libs/rigExecRuntime/geometry.cpp:1998`). Each evaluation repeats the
span search and de Boor solve (`geometryKernels.cpp:781-799`), while
the rest side is frame-invariant for fixed rest CVs, knots, order,
and bind coordinates. (`RigExecApplyWireSparse`, `:977-1012`, has no
callers; production sparse execution uses `RigExecApplyWireBasis`
via `moverGraph.cpp:1025`.)

Proposed change: cache the rest evaluations keyed by resolved rest
CVs, knots, order, and bind coordinates, invalidating on
base-driver/carry changes. Keep the existing arithmetic rather than
substituting the sparse-basis formulation.

### M20. Runtime projector's const reference copies the entire base mesh

`libs/rigExecRuntime/geometry.cpp:5022-5025`:

```cpp
const std::vector<RrVec3f> &basePoints =
    chainIndex < store.chainBases.size()
        ? store.chainBases[chainIndex]
        : std::vector<RrVec3f>();
```

Mixing the vector lvalue with a prvalue makes the conditional produce
a value, so the selected base array is copied in full on every
projector solve -- despite the const reference.

Proposed change: use a named const empty vector so both alternatives
are lvalues, or select a pointer. Preserve the existing empty
fallback. (The adjacent `finalPoints` at `:5026` binds directly and
is fine.)

### M21. Runtime derived publication copies whole meshes four times

Each derived publication copies unconditionally, even when the
recomputation is skipped: `rev.output` into `derived.spare`
(`libs/rigExecRuntime/geometry.cpp:5155-5157`),
`derivedPublish[...].result = derived.result` (`:5161`),
`movedProperties[...] = derived.result`
(`libs/rigExecRuntime/publish.cpp:251`), and the final public
assembly (`libs/rigExecRuntime/exec.cpp:311`). All owning vectors.

Proposed change: as M7/H9 -- retain immutable output storage and
share it through publication, preserving previous-result lifetimes
and failure pass-through.

### M22. Runtime duplicates base meshes during frame preparation

`libs/rigExecRuntime/geometry.cpp:4775` copies the base points into
`store.chainBases[c]`, then `:4813` moves the same local into
`chain.lastBase`. The store copy's reader is the projector
(M20's site at `:5022-5025`).

Proposed change: read the chain's retained base directly (or share
immutable storage), preserving the missing-base guards -- note
`:4775` runs while `:4813` is skipped when the base is missing
(`:4779-4789`). Removes a mesh-sized copy independently of M20's
conditional-expression fix.

### M23. Frozen surface-frame blend assembly copies a dead local mesh

`libs/rigExec/frozenGeometry.cpp:756` builds local `basePoints`; its
reads are the blend summation at `:758` and the copy into
`params->restPoints` at `:775`. Nothing after reads it.

Proposed change: `params->restPoints = std::move(basePoints);` --
one less full-mesh copy per enabled surface-frame blend assembly,
with no arithmetic or parity change.

### M24. Baked/frozen ordinary chains copy unchanged results per publication

The `ChainStatus` step at `libs/rigExec/bakedGeometry.cpp:2076-2078`
unconditionally resizes `chain.spare` and copies the current points
into it, even when every revision output was reused and
`chain.result` already holds the value. The frozen path shares this
code (`libs/rigExec/frozenGeometry.cpp:1464` delegates to
`RigExecBakedRunGeometryStep`), so one fix covers both.

Proposed change: reuse the existing published handle when the base
and revision results are unchanged, respecting first-publication and
invalidation state; keep recording final snapshots and diagnostics.
Removes a mesh-sized copy per unchanged chain per frame. The dynamic
twin is `RigExecMoverGraph::Evaluate`
(`libs/rigExec/moverGraph.cpp:2770-2772`), which allocates and fills
a fresh `VtVec3fArray` per call even when VDF reuses all revision
outputs (invoked per chain per frame at
`rigEvaluatorGeometryEvaluation.cpp:663-667`): retain the
materialized array per requested output/mask and reuse its handle
while that output stays valid, invalidating on source, parameter, or
topology change while preserving phased snapshots and published
lifetimes.

### M25. Runtime intermediate snapshots copy a disposable mesh twice

For every `snapshotAfter` revision, including reused ones,
`libs/rigExecRuntime/geometry.cpp:5569` materializes the mesh into a
local (`value.points.assign(...)`), then `Record` copies it again
(`libs/rigExecRuntime/values.h:232`). The local is dead after.

Proposed change: add a move-taking `Record` overload and pass
`std::move(value)` after materialization. Snapshot ownership and
phased-read behavior stay intact.

### M26. Dynamic graph updates copy disposable parameter packets

`RigExecMoverGraph::UpdateRevision`
(`libs/rigExec/moverGraph.cpp:2663-2683`) takes the packet by const
reference and, when it differs, `SetValue(0, parameters)` at
`:2674-2675` deep-copies every vector into the VDF input. The
ordinary evaluation caller's packet is dead after the call
(`libs/rigExec/rigEvaluatorGeometryEvaluation.cpp:638-644`). VDF
already offers a moving assignment
(`usd-install/include/pxr/exec/vdf/inputVector.h:96-104`).

Proposed change: add an rvalue `UpdateRevision` overload that moves
into the input, make the caller's packet non-const, compute the
status before moving, and retain the existing equality/invalidation
checks.

### M27. Runtime rebuilds epoch-fixed blend layouts every frame

`libs/rigExecRuntime/geometry.cpp:3581` constructs
`RrGeoWireLayout(boundSample)` per sample per assembly, allocating a
layout and copying every offset and index (`:2961-2966`) -- although
the wire data is epoch-fixed. Zero-weight channels still pay this:
the skip happens later in the summation (`:2228-2229`). The baked
path instead shares the cached handle
(`libs/rigExec/bakedGeometry.cpp:1780`, assigned at `:1629`).

Proposed change: cache these immutable layouts per program/geometry
epoch and share their handles. Preserve per-frame refused-layout
precedence, missing/invalid layouts, and existing validation and
accumulation order.

### M28. Runtime copies derived bases into storage nothing reads

`libs/rigExecRuntime/geometry.cpp:4877` copies the full derived base
into `store.derivedBases[id]`, before `:4898` moves the same local
into the retained `derived.lastBase`. `RrStore::derivedBases`
(`libs/rigExecRuntime/store.h:182`) has no readers anywhere; its only
other use is initialization (`libs/rigExecRuntime/open.cpp:897`).

Proposed change: remove this runtime-only member, its initialization,
and the copy. Preserve `derived.lastBase`, `derivedHaveBase`, dirty
comparisons, and the separate serialized
`RigExecWireFrameInputs::derivedBases`; evaluation and binary
compatibility stay unchanged.

### M29. Shader-dial evaluation materializes two meshes it never consumes

`RigExecRunProjectorTarget` returns immediately for `ShaderDials`,
using only the dial reads (`libs/rigExec/moverGraph.cpp:2875-2877`) --
but all three USD call sites build two full mesh vectors first: baked
(`libs/rigExec/bakedGeometry.cpp:2739-2740`), dynamic
(`libs/rigExec/rigEvaluatorGeometryEvaluation.cpp:730-733`), and
frozen (`libs/rigExec/frozenGeometry.cpp:1378-1379`, reached via the
dial-reading branch at `:1326-1333` with no early return). The runtime
already takes the early path
(`libs/rigExecRuntime/geometry.cpp:4934-4943`).

Proposed change: skip mesh materialization for this operation on the
three USD paths while preserving existing guards and dial reads.

### M30. Runtime weight publication copies dense fields three times

Dense weight fields copy per frame through publication:
`store.revisionPublish[...].weightField = rev.weightField`
(`libs/rigExecRuntime/geometry.cpp:5337`),
`field.weights = publish.weightField`
(`libs/rigExecRuntime/publish.cpp:228`), and
`field.weights = entry.second.weights`
(`libs/rigExecRuntime/exec.cpp:336`). Owning `vector<float>` buffers,
potentially mesh-sized, independent of M2's resolver allocation.

Proposed change: resolve directly into the persistent publication
row to remove the first copy, and move the disposable
`store.weightFields` vectors into public outputs to remove the last.
Retain publication rows for skipped steps and preserve last-writer
ordering and failure behavior.

### M31. Baked/frozen weight publication deep-copies dense fields

`libs/rigExec/bakedGeometry.cpp:2672` and
`libs/rigExec/frozenWorker.cpp:732` execute
`field.weights = revision.weightField` per consuming revision per
publication. The destination owns a `vector<float>`
(`libs/rigExec/rigEvaluator.h:72-75`), so potentially mesh-sized
fields copy even when deformation results are reused. M30 covers
only runtime publication.

Proposed change: share immutable field storage between retained
revisions and published poses; preserve previous-pose lifetimes,
skipped-step retention, and last-writer ordering rather than
unconditionally moving from retained state.

### M32. Runtime weight publication copies a packet merely to read it

`libs/rigExecRuntime/geometry.cpp:5315` builds
`const RrGeoWeightPacket packet` from `rev.currentPhasePacket` (a
full copy) or `RrGeoStorePacket` (which copies values and indices at
`:2918-2919`), only to resolve it at `:5324`. Dense/sparse array
allocations plus copies per weight-bound revision, independent of M2
and M30.

Proposed change: resolve through a non-owning packet view, or share
a resolver over the original arrays by const reference. Preserve
source selection, validation, and fallback resolution; using
`rev.parameters.weights` indiscriminately would change behavior when
assembly exits early.

### M33. Runtime derived-target lookup is quadratic per frame

`libs/rigExecRuntime/geometry.cpp:4845-4854` scans the whole
`derivedIndex` separately for every derived target of every chain
during frame preparation (reached per frame via
`libs/rigExecRuntime/exec.cpp:237`) -- O(D squared) integer
compares even when results are reused.

Proposed change: precompute the (chain, local-derived-index) to
dense-id map once during scratch initialization. Preserve missing-ID
validation and traversal order; no arithmetic or binary-format
change. (Absolute cost is modest -- derived counts are small -- but
the fix is trivial.)

### M34. Weight-overlay publication converts the dense field per frame

`libs/rigExecImaging/bridge.cpp:1657` builds a fresh
`VtFloatArray` from the selected weight field on every active overlay
publication (`:2719`, `:2936`), even when the field is unchanged --
one more dense copy beyond M31's revision-to-pose copy.

Proposed change: retain the converted array with exact
field-change detection, or extend shared immutable field storage
through imaging. Preserve target/selection changes, empty-field
behavior, and previous snapshot lifetimes.

### M35. Envelope blending copies the preceding mesh twice

`libs/rigExec/bakedGeometry.cpp:2322` seeds `revision.output` from
the preceding buffer, then the shared wrapper copies it again into
`preceding` (`libs/rigExec/moverGraph.cpp:1115-1117`) for the
non-full-strength envelope blend -- two mesh-sized copies where the
original preceding storage (`PointsBefore`,
`bakedGeometry.cpp:1147-1155`) would serve. The runtime repeats
both (`libs/rigExecRuntime/geometry.cpp:5426` and `:2718`); frozen
replay shares the baked implementation.

Proposed change: let the wrapper accept an optional immutable
preceding-point range and pass the existing source from baked/runtime
callers. Keep the owned-copy fallback for callers without a stable
source. Preserve envelope validation, blend arithmetic, cardinality
checks, and failure pass-through. Removes one full-mesh
allocation/copy per affected execution.

### M36. Runtime derived assembly copies a disposable full mesh

`libs/rigExecRuntime/geometry.cpp:4078` materializes `base` from the
input range, then `RrGeoAssembleDerived` copies it again into
`params->auxPoints` (`:3985`, via a const-reference parameter). The
normals/extent branches at `:4199,4202` are its last readers before
`return params` at `:4210`. Assembly (`:5115`) precedes the reuse
check (`:5123-5126`), so even cache hits pay both copies.

Proposed change: make the helper accept an rvalue and move `base`
into `auxPoints`. Removes one mesh-sized allocation/copy per derived
assembly while preserving validation, exact comparisons, and
dynamic/baked parity; separate from H8's later retention copy.

### M37. Assembly copies disposable provider vectors into packets

The delta-mush and wrinkle fallbacks copy the local base when no
rest points were read, and the lattice helper copies it
unconditionally -- on both sides:

- Runtime: `libs/rigExecRuntime/geometry.cpp:4106`, `:4130`,
  `:3779` (via a const-reference helper parameter). Each is the last
  use of the local in its branch, and assembly (`:5300`) precedes
  change detection (`:5305-5307`), so unchanged evaluations pay too.
- USD: six copies out of the same dead locals --
  `params.blendDeltas = values.blendDeltas` (`:2184`),
  `params.restPoints = values.basePoints` for surface-frame
  blend-shape (`:2190`), the delta-mush/wrinkle fallbacks (`:2219`,
  `:2233`), lattice (`:2286`), and `params.auxPoints` for derived
  maintenance (`:2523`). Those vectors are NOT shared across
  revisions: they are owning members (`moverGraph.h:1485-1486`) of
  per-revision locals (`rigEvaluatorGeometryEvaluation.cpp:313`,
  `:751`; baked: `bakedGeometry.cpp:1503`, consumed by the returning
  call at `:1668`), dead after their assembler call.

Proposed change: move the fallback vectors into the packets. The
runtime needs an ownership-taking lattice-helper overload; the USD
side needs an ownership-taking assembler overload, since the
assembler takes `const RigExecProviderValues &`
(`moverGraph.cpp:2088`). Removes one mesh-sized allocation/copy per
affected assembly without changing validation, comparisons, or
parity.

### M38. Smoothing duplicates the already-seeded preceding mesh

Baked/frozen execution seeds its output
(`libs/rigExec/bakedGeometry.cpp:2322`), then smoothing allocates a
second full copy as its read source
(`const std::vector<GfVec3f> source = *points;`,
`libs/rigExecMath/geometryKernels.cpp:253`). The runtime repeats
both (`libs/rigExecRuntime/geometry.cpp:5426` and `:1576`).

Proposed change: let smoothing read the stable preceding range
directly, retaining the owned-copy fallback when input and output
alias. Preserve neighbor traversal, arithmetic, isolated vertices,
and invalid-topology pass-through across implementations. Saves one
mesh-sized allocation/copy per smoothing execution, including
full-strength cases M35 does not cover.

### M39. Derived kernels blend through a disposable full-strength envelope

`RigExecApplyDerivedKernel`
(`libs/rigExec/moverGraph.cpp:881-909`) computes normals/extent into
a const local (`:886`), resolves a dense envelope (`:900-901`), and
blends elementwise (`:905-906`). The runtime twins it
(`libs/rigExecRuntime/geometry.cpp:2308-2325`). Synthesized derived
packets are `Constant(1.0f)` (`moverGraph.cpp:2171-2172`), and at
full strength the blend returns its input
(`libs/rigExecMath/envelope.h:22-23`) -- so the envelope resolve and
the per-point copy loop just reproduce `values` into `*pts`.

Proposed change: make `values` non-const and, after the existing
cardinality checks, move it into `*pts` when the existing
full-strength predicate succeeds. Retain the general envelope path
otherwise and mirror the runtime change. Removes a mesh-sized copy
plus a float-buffer allocation per normals recomputation.

### M40. Blend-shape kernels resolve a disposable full-strength envelope

`RigExecApplyBlendShapeKernel`
(`libs/rigExec/moverGraph.cpp:823-826`) resolves the full envelope
array unconditionally; the runtime twins it
(`libs/rigExecRuntime/geometry.cpp:2274-2275`). At full strength the
per-point result is just `preceding + delta[i]`, so the mesh-sized
float buffer is allocated only to be read as ones.

Proposed change: as M39 -- bypass the resolve with the existing
full-strength predicate and compute `preceding + delta[i]` directly.
Preserve cardinality checks, surface-frame transport, and the general
envelope path; mirror in the runtime for parity.

### M41. Surface-projector raycasts traverse the whole mesh per solve

`RigExecRaycastSurfaceT`
(`libs/rigExecMath/surfaceProjectorKernel.h:101-144`) walks every
face (`:102`) and fan triangle (`:114`) per call, invoked for the
base surface at `:350` and again for the posed surface in reproject
mode at `:359`. Separate from H7's nearest-point projection and
M18's normal computation.

Proposed change: share reusable triangulation plus a ray-acceleration
structure across projectors, refitting for changed points. Preserve
topology validation, triangle arithmetic, first-hit tie ordering
(`:135`), and fallback behavior across the shared USD/runtime
template.

### M42. Current-phase weight assembly copies arrays immediately discarded

`libs/rigExec/bakedGeometry.cpp:2141-2142` copies the whole retained
packet into `revision.currentPhasePacket`; success then replaces
values/indices/default/valid at `:2154-2159`, failure replaces the
entire packet at `:2165`. Only the source `rangePolicy` survives
(`:2135-2139`) -- the vector copies are pure waste per current-phase
revision assembly. The dynamic twin at
`libs/rigExec/rigEvaluatorGeometryEvaluation.cpp:446-447`
(`:478-482`) has the same shape.

Proposed change: construct the replacement directly, preserving the
source `rangePolicy` on success and the default invalid packet on
failure; on the dynamic path, read the retained packet by const
reference when only its policy is needed.

### M43. Matrix kernels resolve a disposable full-strength envelope

`libs/rigExec/moverGraph.cpp:501-505` sizes a weights vector and
resolves the full envelope even for a valid constant-one envelope;
`libs/rigExecRuntime/geometry.cpp:871-875` duplicates it. M2 removes
the redundant initialization but leaves this allocation and fill.

Proposed change: add a constant-envelope execution path guarded by
the existing full-strength predicate. Preserve the respective SIMD,
scalar, and radial arithmetic rather than replacing all three with
one transform implementation; mirror the runtime change.

### M44. Dynamic snapshot extraction copies provider arrays twice

Per revision, `weights = snapshot.Get<RigExecWeightPacket>(tap)`
(`libs/rigExec/rigEvaluatorGeometryEvaluation.cpp:446-447`)
deep-copies the packet's vectors even for ordinary, non-current-phase
revisions (M42 covers only the replacement case), and the assembler
copies them again (`libs/rigExec/moverGraph.cpp:1563-1564`). The
same pattern affects driver frames: `:319-320` copies
`snapshot.Get<RigExecPointFrameArray>(...)`, then `:2349` copies
`params.frames = *frames`. The templated `Get` returns by value
(`libs/rigExec/tapSet.h:109-111`), and both packet types own vectors
(`types.h:28-29,71-72`).

Proposed change: borrow checked const references through the
untyped `Get(tap)` accessor (`tapSet.h:102-106`), preserving the
existing default value for missing/wrong-type taps. Construct a
mutable weight packet only for current-phase replacement. Removes an
intermediate allocation/copy per affected revision per frame while
preserving packet contents, snapshot lifetime, and dynamic/baked
parity.

### M45. Surface-offset transport rebuilds rest frames per execution

`RigExecTransportSurfaceOffsetsKernel`
(`libs/rigExecMath/deltaMushKernel.h:79-138`) allocates and
accumulates `restNormals` (`:79-102`), then repeats the rest-neighbor
search and tangent construction per vertex (`:124-138`). This runs
for surface-frame blends (`libs/rigExec/moverGraph.cpp:832`) and
delta-mush (`deltaMushKernel.h:210`). H1's connectivity cache and
H6's smoothed-rest cache leave this rest-side work intact.

Proposed change: cache rest normals, selected neighbors, and tangent
frames by exact rest/topology inputs. Preserve arithmetic order and
the `:116` zero-delta bypass -- invalid cached frames must fail only
when consumed. Use the shared template for parity.

### M46. Curve-guide generation allocates a vector per quad

`libs/rigExecImaging/bridge.cpp:765` declares
`appendFace(const std::vector<int> &face)`, while `:792` calls
`appendFace({a, b, c, d})` inside the nested side-face loops --
one heap-allocated temporary per quad per solid surface, repeated
during guide publication (`:1606-1608`, `:2718`, `:2935`).

Proposed change: accept a pointer/count or span and pass a stack
array for quads; pass the existing cap buffers through the same
interface. Removes per-face allocations while preserving winding,
normal arithmetic, and published geometry.

### M47. Current-phase volume weights copy an already-materialized mesh

`libs/rigExec/rigEvaluatorGeometry.cpp:355`
(`samplePoints = *currentPoints`) copies the entire input, used
afterward only for cardinality checks (`:367`) and read-only weight
kernels (`:424`, `:460`, `:472`, taking `const vector&`). Production
callers include `rigEvaluatorGeometryEvaluation.cpp:475` and
`bakedGeometry.cpp:2151`; combined weights repeat this copy per
volume input through `rigEvaluatorGeometry.cpp:297`.

Proposed change: borrow `*currentPoints` by const reference after
the existing null check; retain owned storage for stage-read inputs
(`:356-366`). Removes a mesh-sized allocation/copy per resolve
without changing validation, arithmetic, or dynamic/baked parity.
M42 covers packet replacement, not this point-buffer copy.

### M48. Combine-weight assembly deep-copies every input packet

`libs/rigExec/bakedWeights.cpp:290`
(`inputs.push_back(packets[size_t(input)])`; frozen twin at
`libs/rigExec/frozenGeometry.cpp:187`) copies each input packet's
owning vectors before the read-only builder consumes them
(`libs/rigExec/weightPackets.cpp:453`). Baked execution rebuilds
these per frame (`bakedWeights.cpp:552-553`).

Proposed change: add a builder overload accepting ordered const
packet references or pointers, retaining source lifetimes through
the call. Preserve validation and fold order; removes potentially
mesh-sized copies without changing dynamic/baked results.

### M49. Curve-volume weights scan every segment per sampled point

`libs/rigExecMath/weightFields.cpp:230-233` calls
`RigExecCurveDistance` per point, which loops over every segment at
`:140-142` -- O(points x segments) per field evaluation,
independent of M47's input copy. Production callers include
`rigEvaluatorGeometry.cpp:471-472` and `weightPackets.cpp:397-400`;
the runtime duplicates the nested scan at
`libs/rigExecRuntime/weights.cpp:351-354,434-437`.

Proposed change: precompute segment data and use reusable
nearest-segment acceleration for long curves, invalidated by
curve/placement changes. Preserve distance arithmetic,
degenerate/non-finite behavior, and a short-curve fallback; mirror
the runtime implementation.

### M50. Static-weight assembly copies disposable input arrays

`libs/rigExec/bakedWeights.cpp:267-268`
(`inputs.values/indices = object.values/indices`; frozen twin at
`libs/rigExec/frozenGeometry.cpp:162-163`) copies the owning vectors
into a disposable inputs struct, and the read-only builder copies
dense values again (`libs/rigExec/weightPackets.cpp:62`). The baked
step rebuilds packets per frame (`bakedWeights.cpp:552-553`).

Proposed change: add a builder interface accepting borrowed array
ranges to eliminate the intermediate copies. Preserve sparse
canonicalization, range-policy handling, and invalid-packet
contents; the retained source arrays must remain unchanged.

### M51. Combine-weight resolution binary-searches sparse support per point

`libs/rigExec/weightPackets.cpp:518-520`
(`field[i] = in.Resolve(i, elementCount)`) reaches
`std::lower_bound` per point (`libs/rigExec/types.cpp:26-27`) --
O(N log S) per sparse input per combine evaluation. The runtime
repeats it (`libs/rigExecRuntime/weights.cpp:1023`, `:253-254`).

Proposed change: resolve sorted sparse support with a linear
merge/scatter, preserving existing default, cardinality, and
rejection behavior across both implementations. Do not blindly
substitute `ResolveAll`: its validation is stricter than `Resolve`.

### M52. Combine-weight evaluation retains every expanded dense input

`libs/rigExec/weightPackets.cpp:515-525` allocates one dense field
per input and retains them all before folding into `:528-529` --
O(inputs x points) temporary storage, even for constant or sparse
inputs. The runtime duplicates it
(`libs/rigExecRuntime/weights.cpp:1017-1032`); current-phase
resolution has the same structure
(`libs/rigExec/rigEvaluatorGeometry.cpp:293-303`).

Proposed change: validate first, then fold inputs into a private
accumulator with reusable scratch, reducing temporary storage to
O(points). Preserve authored fold order, subtract/overlay
first-input seeding, average normalization, and atomic failure
behavior. M48's reference-based input collection does not remove
these expanded buffers.

### M53. Radial matrix deformation repeats one decomposition per weight change

`libs/rigExec/moverGraph.cpp:330-332` calls
`RigExecPartialTransform(p.transform, w)` whenever the weight
differs from the preceding one (the sparse branch repeats it at
`:485-487`). Each fractional weight repeats scale extraction,
`basis.ExtractRotation()`
(`libs/rigExecMath/geometryKernels.cpp:199`), and pivot recovery
(`:225-228`). The cache remembers only the preceding weight, so
alternating fractional weights repeat decomposition per point.
Exact 0/1 weights bypass decomposition entirely
(`geometryKernels.cpp:164-168`). Runtime twin:
`libs/rigExecRuntime/runtimeMath.h:2391-2434`.

Proposed change: hoist the weight-independent decomposition once,
lazily for fractional weights, preserving endpoint branches and
arithmetic; mirror the runtime implementation. M43's full-strength
bypass does not address fractional weights.

### M54. Static sparse weights re-sort epoch-fixed support every frame

`libs/rigExec/weightPackets.cpp:68` sorts the sparse pairs on every
build, reached through the per-frame rebuild at
`libs/rigExec/bakedWeights.cpp:552-553` -- although the source
arrays are folded at build time (`bakedWeights.cpp:131-143`) and
only the default weight changes per frame. Frozen replay uses the
same builder; the runtime repeats the sort at
`libs/rigExecRuntime/weights.cpp:861`.

Proposed change: cache canonical support and its validation result
per structural epoch, while continuing to evaluate the changing
default weight and preserving invalid-packet contents and
validation order. M50 removes intermediate copies but leaves this
O(S log S) work.

### M55. Current-phase volume resolution rebuilds the falloff LUT per resolve

`libs/rigExec/rigEvaluatorGeometry.cpp:381`
(`params.curve = _BakeFalloffLut(prim)`) rebakes the table on every
resolve. Authored curves allocate a table and call
`spline.Eval(x, &value)` for all 257 samples (`:188-198`; size at
`libs/rigExecMath/weightFields.h:41`). Production callers include
`rigEvaluatorGeometryEvaluation.cpp:475` and
`bakedGeometry.cpp:2151`; combined inputs repeat it.

Proposed change: cache the table by profile/spline content or an
appropriately invalidated epoch, preserving sampling and fallback
arithmetic. Compile already builds these tables
(`rigEvaluatorCompile.cpp:1655`).

### M56. Current-phase volume/combine resolution fills a discarded dense output

`libs/rigExec/rigEvaluatorGeometry.cpp:485`
(`weights->assign(count, 1.0f)`) fills the output, but the
volume/combine branch then resolves into separate storage (`:505`)
and move-assigns over it (`:529`) -- discarding the entire initial
allocation on success. Recursive combinations repeat this per
input.

Proposed change: defer the initial fill on this branch, retaining
the existing all-ones output on failure and existing behavior for
other branches. Removes a mesh-sized allocation/fill independently
of M47/M52.

### M57. Runtime combine resolution reclassifies representation strings per point

`libs/rigExecRuntime/weights.cpp:1023`
(`field[i] = _RrResolvePacket(...)`) classifies the same packet on
every point (`:243`), performing up to three `TokenEquals` calls
(`:225-232`). Each constructs a string
(`libs/rigExecRuntime/store.h:309`, reaching
`libs/rigExecBinary/container.cpp:262`: `*out =
std::string(text)`).

Proposed change: hoist representation classification and
cardinality checks outside the point loop, then use
representation-specific resolution. Preserve
unknown-representation fallback and existing rejection behavior.
Affects dense and constant inputs too, independently of M51's
sparse searches.

### M58. Volume weights gather a target mesh used only for cardinality

`libs/rigExec/bakedWeights.cpp:378-379` gathers both point arrays,
but `libs/rigExec/weightPackets.cpp:263-275` only checks the
target's size before selecting `&inputs.samplePoints` when samples
are nonempty. Frozen replay repeats the unnecessary copy at
`libs/rigExec/frozenGeometry.cpp:239-240`; the runtime repeats it
at `libs/rigExecRuntime/weights.cpp:1278-1283` -- a mesh-sized
allocation/copy per weight rebuild, independently of H5's detach
and M47's current-phase copy.

Proposed change: pass target cardinality separately and
materialize target coordinates only when the sampled array is
empty. The runtime already has `_RrGatherPointCount` at
`weights.cpp:174`. Preserve read/capture behavior, missing-source
and empty-sample fallback semantics, cardinality validation, and
parity across implementations.

### M59. Current-phase sparse resolution rebuilds duplicate-detection trees

`libs/rigExec/rigEvaluatorGeometry.cpp:730,739` creates
`std::set<int> seen` and inserts every sparse entry -- O(S log S)
work and per-entry node allocations on each resolve. Reached
through mixed current-phase combinations
(`rigEvaluatorCompile.cpp:1694-1697` marks a combination current
when any input is current; `rigEvaluatorGeometry.cpp:297`
recursively resolves every input) from both dynamic and baked
evaluation (`rigEvaluatorGeometryEvaluation.cpp:475`,
`bakedGeometry.cpp:2151`).

Proposed change: cache uniqueness metadata, including the first
duplicate position, by exact authored indices or an appropriately
invalidated epoch; preserve per-call cardinality checks, authored
scatter order, error precedence, and partial-output behavior.
M54's builder-side sort cache does not eliminate this separate
work.

### M60. Current-phase dynamic weights rebuild two sparse-support trees

`libs/rigExec/rigEvaluatorGeometry.cpp:634-636` constructs
`std::set<int>` from both the dynamic and base support arrays --
repeating sorting and per-entry allocations whenever a sparse
dynamic weight authors nonempty support. Reached through mixed
current-phase combinations (`:295-298`) from both dynamic and
baked evaluation (`rigEvaluatorGeometryEvaluation.cpp:475`,
`bakedGeometry.cpp:2151`).

Proposed change: cache the support-equivalence result by exact
input arrays or an appropriately invalidated epoch. Preserve
empty-support inheritance, order-insensitive and
duplicate-insensitive comparison at this check, and its error
precedence before recursive base validation (`:648`). Apply
through the shared resolver to preserve dynamic/baked parity.
M59's static-base duplicate-detection cache does not eliminate
these two additional trees.

### M61. External movers duplicate an already-disposable mesh buffer

`libs/rigExec/moverGraph.cpp:1056`
(`std::vector<GfVec3f> candidate = *pts`) copies the mesh on every
external-kernel execution. Both callers already hold a disposable
buffer with the preceding result preserved elsewhere: dynamic
evaluation materializes `scratch` at `:260-265` (failure falls
back via `passThrough()` at `:268-270`); baked evaluation seeds
`revision.output` at `bakedGeometry.cpp:2322` (failure selects the
preceding `currentSource` at `:2435-2439`). A mesh-sized
allocation/copy even at full strength, independently of M35.

Proposed change: provide an internal execution path letting the
callback mutate the disposable buffer directly, retaining
cardinality/finiteness checks and discarding failed output. Keep
the transactional wrapper for callers requiring unchanged input
on failure, and preserve the same envelope and failure behavior
across dynamic/baked evaluation.

### M62. Derived normals copy authored output that full-strength recomputation replaces

`libs/rigExec/bakedGeometry.cpp:2001` copies `derived.lastBase`
into `values` before the kernel; frozen
(`libs/rigExec/frozenGeometry.cpp:1402`) and runtime
(`libs/rigExecRuntime/geometry.cpp:5128`) repeat it. Synthesized
derived packets are `Constant(1.0f)`
(`libs/rigExec/moverGraph.cpp:2171-2172`), so the successful
kernel needs only the authored cardinality, not those copied
values. M39's result move still leaves this initial
allocation/copy.

Proposed change: add a full-strength derived execution interface
accepting expected cardinality and returning the computed vector;
materialize authored values only for failure/pass-through.
Preserve validation, status, and dynamic/baked/runtime results.

### M63. Publication detaches shared spare buffers immediately before overwriting them

`libs/rigExec/bakedGeometry.cpp:2076-2077` resizes `chain.spare`
(a `VtVec3fArray`) and copies into `spare.data()`. In the
steady state `resize` is a size-match no-op (`array.h:577-578`),
so when an older pose retains that buffer the mutable `data()`
detaches and copies its entire stale contents (`array.h:403`)
before `std::copy` overwrites everything. The derived sites at
`bakedGeometry.cpp:2039-2041` and
`frozenGeometry.cpp:1428-1430` do the same. Affects recomputed
publications too, beyond M7/M24's cache-hit cases.

Proposed change: replace resize-plus-copy with range `assign`:
it calls `clear()` first (`array.h:894-895`), which drops a
shared reference without copying (`array.h:639-642`). Preserves
retained snapshots, unique-buffer capacity reuse, and evaluation
parity.

## Low

Cold paths and minor cleanups, in decreasing order of value:

- L1. `libs/rigExec/bakedProgram.cpp:852-853`: `retained.emplace`
  with an explicitly built `pair` key; fully in-place construction
  needs `std::piecewise_construct` with separate key/value tuples.
- L2. `RigExecResolvedInputs::SetProperty`
  (`libs/rigExec/moverGraph.h:382-384`): `_values[path] = value;`
  looks up then copy-assigns; take the `VtValue` by value and
  `insert_or_assign` with a move.
- L3. `libs/rigExec/moverGraph.cpp:465`:
  `p.transform == GfMatrix4d(1.0)` builds an identity temporary per
  call; compare against a shared static const identity.
- L4. `libs/rigExec/frozenWorker.cpp:1057`: `.reset()` after moving
  `_lastFrozenSlots` out is redundant (a moved-from `shared_ptr` is
  already null); drop the line.
- L5. `libs/rigExecRigging/rigBuilder.cpp:2463-2464`:
  `AddMatrixMathMover` takes `GfMatrix4d value` by value on an
  authoring (cold) path; take `const GfMatrix4d &` for consistency.
- L6. `libs/rigExecImaging/touchPoseHighlight.cpp:153-157,631-637`:
  copies token text into `std::string` for comparison; compare
  `TfToken` to `TfToken` (pointer comparison) instead.
- L7. Oracle-only conversions (smooth/blend-shape/curve/lattice
  movers): same VtArray-to-vector copies as H2 but on parity-reference
  paths. Convert only if oracle runtime matters; otherwise leave for
  readability.

## Checked and cleared

Patterns audited with no finding; recorded so re-reviews do not
re-raise them:

- Range-for loops: no by-value iteration over heavy types; explicit
  typed loops iterate scalars or use references.
- `return std::move(...)`: none.
- By-value sink parameters that immediately `std::move` (profiler
  `Record*`, `RigExecHandleBase`/`RigExecMoverChain`/`RigExecRigBuilder`/`RigExecSchemaPrim`
  constructors, `SetAttr`'s `VtValue`): correct sink idiom.
- `TfToken == "literal"`: direct string comparison, no registry
  interning (verified in
  `usd-install/include/pxr/base/tf/token.h:211-227`).
- `SdfPath::GetString()`: returns `const std::string &`, not a copy
  (`usd-install/include/pxr/usd/sdf/path.h:445`); string comparisons
  against it do not allocate.
- `VtValue` copies share remote payloads by reference-counted handle
  (`usd-install/include/pxr/base/vt/value.h:745-747`); only the map
  entries, not the payloads, are duplicated in H3.
- `lastAuxPoints` (`bakedProgramImpl.h:2168`): a `VtVec3fArray`
  handle share with short-circuit comparison, by design (`:2160-2168`).
- `_ReadAttribute`'s by-value fallback: all call sites pass small types
  (`bool`, `float`, `TfToken`, `GfQuatf`, `GfVec3f`).
- `sceneIndices.cpp` guide-shape lambda taking `VtArray` by value:
  one-time cold setup.
- Frame cache: shared-handle design, pose copied outside shard locks.
- Skin kernel: per-matrix tables hoisted once per array
  (`moverGraph.cpp:755-781`), chunked parallel ranges, SIMD path;
  epoch-fixed layouts shared by handle (`types.h:332-336`).
- Skin oracle (`movers/skinMover.cpp:208-209,248,337-338`): works in
  `VtArray`s directly, no VtArray-to-vector conversion.
- RBF matrix inversion: solver-construction time, not per frame.
- `UpdateRevision`'s signature takes const references, but the VDF
  `SetValue` inside still copies (now tracked as M26).
- Profile-scope name/category expressions: not evaluated when disabled.
- M3 fixed in tree: `_enabled` is `std::atomic<bool>` with a lock-free
  `IsEnabled()` (`profiler.h:104`, `:477`).
- M6 dynamic-geometry examples fixed in tree: file-static tokens at
  `rigEvaluatorGeometryEvaluation.cpp:33-37` (mover-builder and
  imaging examples remain open).
- Moves from members (`_bakedProgram` handoff, registry swaps,
  profiler destructor): legitimate non-const handoffs; the moved-from
  null check at `rigEvaluator.cpp:373` is intentional.
- `push_back({...})` into aggregate vectors (`exec.cpp:134,152`):
  correct as written; `emplace_back` with separate arguments is not
  valid C++17 for aggregates without a constructor.
- `bakedProgram.cpp:852`: already two-argument `emplace`; only
  `piecewise_construct` would go further (L1).
- `memcpy`: hashing and (de)serialization only.
- `RrProgram::pathIndex`: built at Open, not per frame.
- `GetWeightFields()`: returns a const reference.
- `std::map::count` uses: boolean membership tests, no
  find-then-index double lookups.
- `std::endl`: none. `operator[]` bulk inserts with end hints in the
  baked publish path are already optimal.

## Revision history

- v1 (2026-10-03): initial Muse review.
- v2 (2026-10-03): incorporated codex-astra round 1 (17 notes, all
  verified against code and accepted). H1 gained ordering constraints
  and the transport-kernel rebuild; H3 scoped to phased revisions with
  a parallel-safe proposal; H4 dropped `lastAuxPoints`; H5 reworked to
  M7; M2 narrowed to live paths plus the sparse-resolver temp; M5
  scoped to uncached layouts; M7 (v1) removed as a false positive
  (`GetString` returns a const reference); L1 reworded to
  `piecewise_construct`; L3 (v1) removed (aggregates -- current code
  correct); L8 dropped skin from its examples. New from astra: H5
  (VtArray detach), H6 (delta-mush rest solve), H7 (surface scan), M8
  (wrinkle loop scratch), M9 (lattice Bernstein), M10 (lastPoints
  retention).
- v3 (2026-10-03): incorporated codex-astra round 2 (10 notes, all
  verified against code and accepted). M2 gained the alias guard; M5
  scoped to the fixed identity table; H5 scoped to one detach plus the
  failure-arm copy; H2 extended to packet storage; H3 extended to the
  dynamic twin; H4 citation fixed to `:2198`; M9 count corrected. New
  from astra: M11 (frozen projector detach), M12 (baked basePoints
  predicate), M13 (sphere guide rebuild).
- v4 (2026-10-03): incorporated codex-astra round 3 (2 notes, both
  verified against code and accepted). New from astra: M14 (wrinkle
  phase-weight buffers), M15 (wire per-CV inversion caching).
- v5 (2026-10-03): incorporated codex-astra round 4 (6 notes, all
  verified against code and accepted). M4 narrowed to the reachable
  copy; M6 convention citation refreshed. New from astra: M16
  (derived-cache retention moves), M17 (wire-basis cache-hit cost),
  M18 (projector double normals), M19 (dense-wire rest re-evaluation).
- v6 (2026-10-03): incorporated codex-astra round 5 (8 notes, all
  verified against code and accepted). M5 now preserves enabled/valid
  and skinning method; M19 fixed to `RigExecApplyWire` with the sparse
  implementation marked uncalled; H5 gained the shared `_Array`
  assembler; M12 extended to the runtime twin; M16 last-read
  corrected. New from astra: H8 (runtime retained-input copies), H9
  (runtime publication chain), M20 (projector conditional copy).
- v7 (2026-10-03): incorporated codex-astra round 6 (5 notes, all
  verified against code and accepted). H5 corrected (stage-read arm
  can detach during capture); M2 extended to the runtime twin. New
  from astra: M21 (runtime derived publication), M22 (runtime base
  duplication), M23 (frozen blend restPoints move).
- v8 (2026-10-03): incorporated codex-astra round 7 (4 notes, all
  verified against code and accepted). M7 scoped to per-publication
  with a no-unification cache-hit fix; H9 corrected to five copies.
  New from astra: M24 (unchanged chain publication), M25 (snapshot
  Record move).
- v9 (2026-10-03): incorporated codex-astra round 8 (3 notes, all
  verified against code and accepted). New from astra: M26 (VDF packet
  move), M27 (runtime blend layouts), M28 (dead derivedBases store).
  The `UpdateRevision` cleared item now points at M26.
- v10 (2026-10-03): incorporated codex-astra round 9 (5 notes, all
  verified against code and accepted). H2 span type fixed to
  `TfSpan`; M27 citation fixed to `:1780`; H5 gained the dynamic
  base-points sites; M24 gained the dynamic `Evaluate` twin. New
  from astra: M29 (shader-dial mesh skip).
- v11 (2026-10-03): incorporated codex-astra round 10 (2 notes, both
  verified against code and accepted). H9 corrected to six copies and
  M21 to four (final public assembly); M21/H9 gained the
  movedProperties-move proposal. New from astra: M30 (weight
  publication chain).
- v12 (2026-10-03): incorporated codex-astra round 11 (2 notes, both
  verified against code and accepted). New from astra: M31
  (baked/frozen weight publication), M32 (runtime packet-view
  resolution).
- v13 (2026-10-03): incorporated codex-astra round 12 (2 notes, both
  verified against code and accepted). New from astra: M33 (runtime
  derived-index lookup), M34 (overlay field conversion).
- v14 (2026-10-03): incorporated codex-astra round 13 (1 note,
  verified against code and accepted). New from astra: M35 (envelope
  preceding-range overload).
- v15 (2026-10-03): incorporated codex-astra round 14 (1 note,
  verified against code and accepted). New from astra: M36 (runtime
  derived-assembly move).
- v16 (2026-10-03): incorporated codex-astra round 15 (2 notes, both
  verified against code and accepted). New from astra: M37 (runtime
  fallback moves), M38 (smoothing preceding range).
- v17 (2026-10-03): incorporated codex-astra round 16 (2 notes, both
  verified against code and accepted). M37 corrected and extended to
  the six USD-side provider copies. New from astra: M39 (derived
  full-strength move).
- v18 (2026-10-03): incorporated codex-astra round 17 (3 notes, all
  verified against code and accepted). Refreshed drifted citations
  after the rigexec-forward-port merge (M13, M34, M5, M10, L1, L5,
  cleared `lastAuxPoints`); full citation pass over the remaining
  100 cites found no further drift. New from astra: M40 (blend-shape
  full-strength bypass), M41 (projector raycast acceleration).
- v19 (2026-10-03): incorporated codex-astra round 18 (4 notes, all
  verified against code and accepted). M3 moved to Cleared (fixed in
  tree: atomic profiler flag); M6 trimmed to the still-open
  mover-builder and imaging examples. Remapped citations drifted by
  ongoing commits (moverGraph.h +13, moverGraph.cpp +7,
  rigEvaluatorGeometryEvaluation hunks, Internal.h hoists); all other
  cites re-verified clean. New from astra: M42 (current-phase packet
  copy), M43 (matrix full-strength bypass).
- v20 (2026-10-03): incorporated codex-astra round 19 (1 note,
  verified against code and accepted). New from astra: M44 (snapshot
  extraction double copy).
- v21 (2026-10-03): incorporated codex-astra round 20 (2 notes, both
  verified against code and accepted). New from astra: M45 (transport
  rest frames), M46 (guide quad vectors).
- v22 (2026-10-03): incorporated codex-astra round 21 (2 notes, both
  verified against code and accepted). M6 blend-shape citation fixed
  to `:40`. New from astra: M47 (volume sample-points borrow).
- v23 (2026-10-03): incorporated codex-astra round 22 (2 notes, both
  verified against code and accepted). New from astra: M48 (combine
  packet references), M49 (curve-segment acceleration).
- v24 (2026-10-03): incorporated codex-astra round 23 (3 notes, all
  verified against code and accepted). New from astra: M50
  (static-weight disposable-array copies), M51 (per-point sparse
  lower_bound in combine resolution), M52 (retained expanded dense
  combine inputs).
- v25 (2026-10-03): incorporated codex-astra round 24 (2 findings +
  1 nit, all verified against code and accepted). Remapped all
  runtime citations drifted by the plugin-movers commit (geometry.cpp
  +3/+14/+20/+75/+120/+123 by region, exec.cpp +1/+16, open.cpp
  +58, store.h +1; publish/values/weights unchanged); every
  remapped line verified by content. New from astra: M53 (radial
  per-weight decomposition), M54 (per-frame sparse re-sort).
- v26 (2026-10-03): incorporated codex-astra round 25 (1 correction
  + 2 findings + 1 nit, all verified against code and accepted).
  M53 corrected (preceding-weight cache, 0/1 bypass). Fixed four
  runtime cites missed by the v25 remap (H8 `:2812`, M20 `:5026`,
  M27 `:2961-2966` tightened to the copy range, M27 `:2228-2229`
  tightened to the skip); full bare-cite sweep found no further
  misses. New from astra: M55 (per-resolve falloff LUT rebuild),
  M56 (discarded dense weight fill).
- v27 (2026-10-03): incorporated codex-astra round 26 (2 notes, both
  verified against code and accepted). H5 extended to the
  baked-weights gather detach; new from astra: M57 (per-point
  runtime representation-string reconstruction).
- v28 (2026-10-03): incorporated codex-astra round 27 (1 note,
  verified against code and accepted). New from astra: M58
  (target mesh gathered for cardinality only).
- v29 (2026-10-03): incorporated codex-astra round 28 (1 note,
  verified against code and accepted). New from astra: M59
  (per-resolve sparse duplicate-detection tree).
- v30 (2026-10-03): incorporated codex-astra round 29 (1 note,
  verified against code and accepted). New from astra: M60
  (per-resolve dynamic/base support-equivalence trees).
- v31 (2026-10-03): incorporated codex-astra round 30 (1 note,
  verified against code and accepted). New from astra: M61
  (external-mover redundant transactional copy).
- v32 (2026-10-03): incorporated codex-astra round 31 (2 notes, both
  verified against code and accepted). M12 extended to the runtime
  External exclusion; new from astra: M62 (derived lastBase copy
  replaced at full strength).
- v33 (2026-10-03): incorporated codex-astra round 32 (1 note,
  verified against code and accepted). New from astra: M63
  (spare-buffer detach before overwrite). Round 33 returned
  NO NOTES: every finding verified accurate, severities fair,
  nothing Medium-or-higher missed. Review converged.
