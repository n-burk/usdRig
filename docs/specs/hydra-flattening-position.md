# Where RigExec sits relative to Hydra's flattening

The bug this documents: `10_AimXformTurret` showed `Turret.xform.matrix`
animating in usdview's scene index debugger while the viewport stayed
completely static.

## The chain

`UsdImagingSceneIndexPlugin::AppendSceneIndex` is the only hook RigExec has
(stock usdview offers no application-level insertion point). In OpenUSD 26.08
`UsdImagingCreateSceneIndices` calls it from `_AddPluginSceneIndices` at
`usdImaging/sceneIndices.cpp:302`.

The chain's flattening is created inside
`UsdImagingNiPrototypePropagatingSceneIndex`
(`usdImaging/niPrototypePropagatingSceneIndex.cpp:204`), which the same
function constructs at `sceneIndices.cpp:281` — **twenty-one lines earlier**.

(There is one *construction site*, not one runtime instance: prototype
propagation inserts a flattening index at every native-instancing recursion
level. The ordering conclusion is unaffected — all of them precede the plugin
hook.)

So the order is fixed and not negotiable:

```
UsdImagingStageSceneIndex
  ... -> HdFlatteningSceneIndex   (inside NiPrototypePropagating, line 281)
      -> RigExecInternalPrimPruning
      -> RigExecBindingResolving
      -> RigExecResults                            (line 302, via plugin)
      -> UsdImagingSelectionSceneIndex             (line 304)
```

Verify with `grep -rn "HdFlatteningSceneIndex::New" pxr/` — outside test code
there is exactly one non-`renderSettings` construction site.

## What follows from it

**1. Transforms arriving at RigExec are world-space.** Publishing the
evaluator's revised *local* matrix onto the prim is a space error. It is
invisible whenever every ancestor is identity, which is exactly the shape of a
minimal test scene — so it survives a green suite. `10_AimXformTurret` had an
identity `/TurretAsset/Geom`, which is why this went unnoticed; that prim now
carries a deliberate offset.

("World" is precise only outside instancing. Inside a native or procedural
prototype these are prototype-common space. The delta algebra below holds in
that space too, but instance paths are untested.)

**1a. A separate space error exists one layer up, in the evaluator — and it
has three candidate answers, two of them wrong.**

A constraint solves its provider's base frame against an aim target frame, and
the solver subtracts the two origins directly, so both must be in one space.
Joint and control frames come from authored `rest:space` composed with avars
(`_JointRestSpace`, `computations.cpp`) and never acquire the asset's stage
placement. They are **asset-common space**. Therefore the provider's base must
be its transform **relative to the asset root** (the rig prim's parent):

| base frame | wrong how |
|---|---|
| `GetLocalTransformation` | omits everything between provider and asset root |
| `ComputeLocalToWorldTransform` | adds the asset's stage placement to one side of the subtraction only |
| `ComputeRelativeTransform(provider, assetRoot)` | correct |

Measured on `10_AimXformTurret`: with the asset placed at `x=+100`, the
local-to-world base aims the turret at `(-0.991, -0.010, 0.131)` — pointing
away along −X while the target is physically at +X. The local-to-parent base
aims at `(0.625, 0, 0.781)`, ignoring `Geom`. The correct answer is
`(0.152, -0.076, 0.986)`.

Identity ancestors collapse all three, which is how the first error hid and
how the over-correction hid after it. The example now carries **two** distinct
non-identity offsets — `/TurretAsset` at `(100,0,0)` and `/TurretAsset/Geom`
at `(6,1,-3)` — precisely so each mistake fails a different way.

Note `UsdGeomXformCache::ComputeRelativeTransform`'s `resetXformStack`
out-parameter is documented as required to be valid; passing `nullptr`
silently yields an identity base.

**2. Nothing downstream recomposes descendants.** Their world transforms were
already composed, upstream, from the driven prim's *original* matrix.
`HdSceneIndexAdapterSceneDelegate::GetTransform`
(`hd/sceneIndexAdapterSceneDelegate.cpp:2478`) reads a prim's own xform with no
ancestor accumulation, so whatever RigExec publishes is what Storm draws.
Dirtying only the driven prim leaves every mesh under it on a stale matrix —
the exact reported symptom.

Hydra dirtiness is explicitly non-hierarchical (`hd/sceneIndexObserver.h:74`):
dirtying `/Turret` does not dirty `/Turret/Barrel`. This applies to
**structural** changes as much as value changes, and that is the easier half
to miss — every `hasXform` transition (first publication, a provider gaining
or losing its constraint, recompilation, `Deactivate()`) arrives as
`RigExecChangeStructural`, never as `RigExecChangeXform`. Handling only the
latter reproduces the original bug on exactly the transitions that matter
most. `_announcedDrivenXforms` exists for the removal case: once a provider
leaves the snapshot, nothing else records that its subtree carries a stale
delta.

**2a. Fail closed on singular transforms.** `GfMatrix4d::GetInverse()` does not
report failure — it returns `FLT_MAX * identity`. Unchecked, a zero-scale prim
(an ordinary way to hide geometry) turns into components around `1e77`. A
collapsed dimension also means no uniform right-delta need exist, so there is
nothing correct to publish and the resolution declines.

**2b. Publish `resetXformStack = true`.** The matrix RigExec publishes is fully
composed, exactly like the one flattening produced. Marking it `false` would
invite a later flattener to compose the parent in a second time, and consumers
that key off the flag skip it — hdPrman's world-offset index applies its offset
only to matrices marked composed.

**3. `resetXformStack` is no longer authored intent.**
`HdFlattenedXformDataSourceProvider` consumes the authored flag and stamps
`resetXformStack = true` on *every* flattened prim to mark the matrix as
absolute (`hd/flattenedXformDataSourceProvider.cpp:124,132`). Downstream of
flattening it means "already world-space", not "ignores ancestors". Treating it
as a reset boundary breaks the ancestor walk on its very first step — which is
how the first version of `_ComputeDrivenXform` failed.

## The rule RigExec implements

USD is row-vector, so `world = local * parentWorld`. For a driven prim `A`:

```
W_old = L_old * P        W_new = L_new * P        P = L_old^-1 * W_old
=> W_new = L_new * L_old^-1 * W_old
```

A descendant `D` has `F_old = C * W_old` for the accumulated locals `C`
between `D` and `A`, so `F_new = C * W_new = F_old * (W_old^-1 * W_new)`.
That is one delta, **post-multiplied**, applying uniformly to `A` and to
everything beneath it:

```
delta_A = W_old_A^-1 * L_new_A * L_old_A^-1 * W_old_A
F_new   = F_old * delta_A1 * delta_A2 * ...      (innermost ancestor first)
```

Nesting composes because each delta is expressed in the un-revised world frame
at its own level, which is what the input scene index still reports. Verified
algebraically for two nested driven prims.

This is why `RigExecPublishedPrim` carries **both** `xform` (revised local) and
`xformBase` (the local it revised): the delta cannot be recovered from either
alone, and `_Diff` must treat a change in either as a transform change.

### Known limitation

A descendant with an authored `resetXformStack` should be immune to a driven
ancestor, but per point 3 that information is destroyed upstream and is not
recoverable. Such a prim is carried along instead of staying put.

## Why the unit test missed it

`TestConstraintDrivenXformPublishes` originally built
`HdFlatteningSceneIndex::New(results, nullptr)` — flattening *downstream* of
RigExec, the reverse of reality, and with `nullptr` inputArgs, which means
**zero** flattened data source providers. Such an index flattens nothing, so
the child read back identity at every frame. The test could not have detected
any invalidation change, and four successive fixes were evaluated against it
without effect.

Two lessons are now encoded in the tree:

- The unit test builds flattening **upstream**, with
  `HdFlattenedDataSourceProviders()`, and its scene has a **non-identity**
  asset root so a local/world confusion is detectable. It also asserts
  invalidation via a recording observer — `GetPrim` recomputes on every pull,
  so pull-based assertions alone cannot prove a viewer is ever told to redraw.
- `tests/probeImagingPipeline.cpp` asserts against the real
  `UsdImagingCreateSceneIndices` chain, activating through the same C entry
  point the usdview plugin uses. When a hand-built harness and the real chain
  disagree, the harness is what needs checking first.
