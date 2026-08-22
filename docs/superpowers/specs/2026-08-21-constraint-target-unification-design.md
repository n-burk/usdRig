# Constraint target unification

**Date:** 2026-08-21
**Status:** design approved, not yet implemented
**Scope:** `libs/rigExecSchema/schema.usda`, `libs/rigExec/rigEvaluator.cpp`, `libs/rigExec/moverGraph.cpp`, `tests/testRigExecConstraints.cpp`, three committed examples

All line numbers refer to the working tree as of 2026-08-21, which carries a large
uncommitted change to `rigEvaluator.cpp` (+1321/-222) introducing the FBX-style
constraint family.

---

## 1. Problem

RigExec has three output domains — geometry (point chains), transforms, and
properties (commit `2d7f5ba`) — but only one *target* concept. A mover names its
write set with `rigExec:moves`, and `_CanonicalizeTarget` (`rigEvaluator.cpp:251`)
rewrites that path by sniffing the prim type:

```cpp
if (target.IsPrimPath()) {
    const UsdPrim prim = stage->GetPrimAtPath(target);
    if (prim && prim.IsA<UsdGeomPointBased>()) {
        return target.AppendProperty(TfToken("points"));
    }
}
return target;
```

Domain membership for a constraint is therefore literally
`IsA<UsdGeomPointBased>() == false`. Two consequences follow.

**A constraint cannot target a mesh or a curve.** There are two distinct failure
modes, which matters for phasing:

- **By rewrite.** `</World/Geom/Cylinder>` becomes `</World/Geom/Cylinder.points>`,
  and the source-frame gate at `:1198-1204` requires `targets[0].IsPrimPath()`.
  Affects `examples/rotateConstraint.usda` and `examples/aimtest.usda`.
- **By direct rejection.** `examples/aimtest_points.usda:73` authors
  `</World/Geom/Sphere.points>` itself. `_CanonicalizeTarget` leaves a property
  path alone (`:251-259`), so no rewrite occurs; it is rejected at the same
  `!record.targets[0].IsPrimPath()` gate on `:1200`.

The first mode is confirmed end-to-end: `RigExecImaging_Activate` returns 3 and the
prim's transform is identical at every frame.

The file contradicts itself here. Three other sites accept exactly what the gate
rejects: `appendFrameBindingIdentity` classifies a Mesh target as `nativeXform`
(`:445`), `bindFrameSource` accepts any `UsdGeomXformable` (`:2209`), and the
provider classifier does the same (`:2414`). `examples/10_AimXformTurret.usda:19`
states the contract in prose: *"A constraint target only has to be a
UsdGeomXformable; its own composed transform is the base frame the constraint
revises."* A Mesh is one. Only the canonicalization disagrees.

At `HEAD` the same authoring compiled "ok" and silently did nothing
(`if (!target.IsPrimPath()) { continue; }`). The new gate did not cause the defect;
it made a pre-existing one loud, with a message that blames the author for
something the compiler did.

**Behavior is not consistent across operators.** The evaluator dispatches on
hardcoded type-name string lists — `_IsSourceFrameConstraintType` (`:112-119`) and
`_IsFrameConstraintType` (`:123-127`) — not on `IsA<>` against the abstract schema
bases. The schema is codeless (`schema.usda:23`, `skipCodeGeneration = true`), so a
new common base class is **invisible to the evaluator**. Roughly twenty sites would
need editing to teach it one: 15 `schemaType == "RigExec*Constraint"` comparisons,
6 predicate call sites (`:772, 1100, 1198, 1611, 2220, 2231`), and the 2 predicate
bodies. Section 3 catalogues what has drifted apart as a result.

---

## 2. Design decisions

| # | Decision |
|---|---|
| D1 | Output domain is chosen by **explicit target spelling**. A bare prim path means the transform domain; `<prim>.points` means the geometry domain. `_CanonicalizeTarget`'s inference is deleted **on the write path**; it is retained for read-side geometry inputs (see 4.1). |
| D2 | The geometry domain applies **the same delta the transform domain would publish**, per-point. With no weight object bound and the same `inputs:defaultWeight`, the two spellings are indistinguishable. |
| D3 | **Full unification, phased** — a handler registry, one target-binding facade, one channel model, delivered in four independently testable landings. |
| D4 | Masks and offsets use **channel-group addressing**: `(group, axis)`, where group is one of translation / rotation / scale. |
| D5 | `inputs:defaultWeight` (float, default 1) is the constraint's **envelope**, refined per element by an optional weight object. |
| D6 | The envelope **collapses to one attribute**: `inputs:weight` is removed from `RigExecConstraint`. |

### Why D1 is safe

The inference is load-bearing nowhere on the write path. Across every example in
the working tree and every test, a bare prim path resolving to a
`UsdGeomPointBased` prim occurs exactly twice — `aimtest.usda` and
`rotateConstraint.usda`, the two broken files (both still untracked). The near miss
is `rigexec_flat.usda:61`, which targets a `def Sphere` and escapes the rewrite
only because `UsdGeomSphere` is not `UsdGeomPointBased`; the same authoring on a
Mesh breaks. Every point-domain mover in the corpus already spells `.points`
explicitly (37 `rigExec:moves` targets across 13 `.usda` examples;
`_kTarget("/Asset/Geom/M.points")` and its peers across the tests).

---

## 3. What is inconsistent today

### 3.1 Dispatch

| operator | schema base | evaluator predicate | dispatch |
|---|---|---|---|
| Aim | `RigExecSourceConstraint` | `_IsSourceFrameConstraintType` | evaluator-inline blend `:5182-5312` |
| Position / Rotation / Scale / Parent | `RigExecSourceConstraint` | same | kernel `:5135-5181` |
| SingleChainIk | `RigExecConstraint` | `_IsFrameConstraintType` only | early-`continue` branch `:4955-5110` |
| Custom | `RigExecConstraint` | neither | hard compile error `:1221-1231` |

### 3.2 Axis masks

`inputs:affectX/Y/Z` is declared four times — Aim (`schema.usda:680-682`), Position
(`:711-713`), Rotation (`:727-729`), Scale (`:746-748`) — and controls a
**different channel group in each**: rotation, translation, rotation, scale
respectively. Parent instead declares nine booleans (`:763-771`) with scale
defaulting to `false`. SingleChainIk has no mask at all.

### 3.3 Offsets — two incompatible algebras

Position, Rotation, Scale and Aim carry a single `double3` offset applied after the
blend. Parent carries **arrays parallel to `rigExec:sources`**, pre-multiplied
per-source inside the accumulation loop (`solvers.cpp:825-829`), and has no scale
offset at all.

`inputs:scaleOffset` defaults to `(0,0,0)` (`schema.usda:749`). This is *correct*
for the kernel, which is additive (`solvers.cpp:758`,
`targetScale = targetScale / totalWeight + params.offset`), but nothing in the doc
string says the algebra is additive — which is the actual defect.

### 3.4 Rotation order

Declared by Aim, Rotation and Parent; compile-validated only for those three
(`:1505-1509`). **Position and Scale read it at `:5127-5133` and ignore it**, so
`rigExec:rotationOrder = "ZYX"` on a Position constraint is accepted silently and
does nothing.

### 3.5 Source weights

Three names, three validators, three diagnostics. The kernels skip zero-weight
sources *before* validating the frame (`solvers.cpp:612-617`); Aim's inline loop
does not — the loop at `:5185-5197` validates only `source.normalizedWeight` and
never `source.frame`, so
`target += source.frame.Origin() * source.normalizedWeight` at `:5195` poisons the
blend with NaN where Position would skip the source. Aim with a zero weight total
falls through with `candidate == inputFrame` and still commits (`:5311-5314`) with
no diagnostic.

`inputs:weight` reads through `_resolvedInputs` (`:4933-4934`), but both array
weights — `inputs:sourceWeights` (`:4685`) and `inputs:poleVectorWeights`
(`:5015`) — go through the `readWeights` lambda at `:4641-4659`, whose `a.Get()` at
`:4645` bypasses it. A property mover can drive the envelope but neither array.

### 3.6 Source-order sensitivity

Rotation and Parent anchor their Euler blend on the first positive-weight source
(`solvers.cpp:672-701`, `:846-861`) and are therefore order-dependent. Position,
Scale and Aim blend linearly and are not. `schema.usda:631-634` (the
`rigExec:sources` doc) justifies source order only as *"it identifies entries in
every parallel source array"* — a much weaker claim than "it selects the rotation
anchor".

### 3.7 Secondary reference direction

Aim's `rigExec:worldUpObject` (cardinality ≤ 1) and SingleChainIk's
`rigExec:poleVectorObjects` (ordered N, weighted) are the same concept with
different names, different cardinality, different mode tokens, and opposite
failure behavior — Aim silently substitutes a default up direction when
`rigExec:worldUpObject` is unbound (`:5259` objectUp, `:5279` objectRotationUp),
IK raises a diagnostic and passes through (`:5024-5028`).

### 3.8 Weight-object optionality

`RigExecMatrixMover` declares *"Exactly one compatible weight object"*
(`schema.usda:1306`); `RigExecBlendShapeMover` declares *"Optional"* (`:1325`).
Neither is a constraint, so this design does not close it — see 4.5.

---

## 4. Design

### 4.1 The target model

One resolution routine, mirroring the `bindFrameSource` facade that already works
for sources (`:2186-2214`). Targets currently get a bare
`std::set<SdfPath> _xformDerivedProviders` (`rigEvaluator.h:426`) that cannot carry
a second kind.

```cpp
enum class RigExecTargetDomain { Transform, Geometry };

struct _ConstraintTargetBinding {
    SdfPath             authored;       // exactly as written; never rewritten
    SdfPath             primPath;       // owning prim
    RigExecTargetDomain domain;
    // Transform arm - exactly one populated
    RigExecTapId        frameTap;       // RigExecControl / RigExecJoint
    bool                nativeXform;    // any other UsdGeomXformable
    // Geometry arm
    SdfPath             pointsProperty;
    RigExecTapId        weightTap;      // optional; absent => constant envelope
};
```

Resolution is total:

| authored spelling | resolves to |
|---|---|
| prim path, `RigExecControl` / `RigExecJoint` | Transform, tapped frame |
| prim path, any other `UsdGeomXformable` | Transform, native xform |
| prim path, not Xformable | error — not a transform provider |
| `<prim>.points`, prim `IsA<UsdGeomPointBased>` | Geometry |
| `<prim>.points`, prim not PointBased | error |
| any other property path | error — constraints do not write arbitrary properties |

The `IsPrimPath()` proxy at `:1200` is replaced by the real predicate
`UsdGeomXformable(prim)` — the same one sources use at `:2209`.

**`_CanonicalizeTarget` is split in two, because it currently conflates two
different jobs** — resolving a *write* target and resolving a *read* domain. Its own
doc comment at `:246-249` already admits the exception.

| new function | rule | call sites |
|---|---|---|
| `_CanonicalizeWriteTarget` | **no inference.** Exact spelling, as authored. | `:1136` (mover records), `:727` (epoch digest) |
| `_ResolveGeometryInput` | keeps the `PointBased -> .points` rule | `:3002` `_ReadTargetPoints`, `:2981` and `:3356` weight-target matching |

Deleting the inference on the **write** path is what fixes constraints, and is safe
(see 2). Deleting it on the **read** path would not be, and must not happen. Four
committed examples name a bare geometry prim on a read relationship and rely on the
rewrite:

| file | relationship | target |
|---|---|---|
| `06_LatticeBulge.usda` | `rigExec:cage` | `/LatticeAsset/Geom/Cage` (Points) |
| `07_SurfaceDrape.usda` (×2) | `rigExec:surface` | `/DrapeAsset/Geom/Ground` (Mesh) |
| `13_ReadPhases.usda` | `rigExec:cage` | `/ReadPhaseAsset/Geom/Cage` (Points) |

The asymmetry is deliberate and is the right ergonomic in each direction. On a read
input, "name the mesh, I will read its points" is unambiguous — there is only one
thing to read. On a write target it is not, which is the entire bug.

A point-domain mover whose *write* target is a bare mesh prim now errors with the
fix in hand:

```
Mover /Asset/Rig/Movers/M targets prim /Asset/Geom/Mesh, but a point-domain
mover writes a point set. Did you mean </Asset/Geom/Mesh.points>?
```

"Point-domain mover" is not a single lookup today. `rigEvaluator.cpp:1261-1268`
hardcodes seven types and emits its own wording at `:1283-1289`;
`RigExecMatrixMover` is not in that set and goes through `_ValidateMatrixMover`
with a third wording at `:2901-2903`. Phase 1 edits both sites.

### 4.2 The handler registry

This is what makes consistency structural rather than decorative. A schema base
class alone cannot deliver it, because the codeless schema is invisible to the
evaluator's type-name switches.

```cpp
struct RigExecConstraintHandler {
    TfToken                        schemaType;
    RigExecChannelGroups           writes;         // what the solve produces
    RigExecChannelGroups           maskable;       // per-axis masks honored
    RigExecChannelGroups           offsettable;    // offsets honored
    RigExecOffsetAlgebra           offsetAlgebra;  // None | ScalarAdditive | PerSourceArray
    bool                           usesRotationOrder;
    RigExecTargetArity             arity;          // One | Chain(min) | None
    RigExecDomainMask              domains;        // Transform | Geometry | both
    std::vector<RigExecSourceRole> roles;          // primary, worldUp, pole, effector
    RigExecConstraintSolveFn       solve;          // frame in, frame out
    RigExecConstraintValidateFn    validate;       // optional cross-role rule
};
```

Three capability sets rather than one, because they are independent axes. A single
bitmask would reintroduce the defect this design removes: SingleChainIk *writes*
translation and rotation but honors no masks and no offsets, so a mask on an IK
constraint would compile and do nothing — 3.4 in new clothes.

| operator | writes | maskable | offsettable | rotationOrder | domains |
|---|---|---|---|---|---|
| Position | {T} | {T} | {T} additive | no | both |
| Rotation | {R} | {R} | {R} additive | yes | both |
| Scale | {S} | {S} | {S} additive | no | both |
| Aim | {R} | {R} | {R} additive | yes | both |
| Parent | {T,R,S} | {T,R,S} | {T,R} per-source array | yes | both |
| SingleChainIk | {T,R} | {} | {} | no | Transform only |
| Custom | {} | {} | {} | no | none |

Parent's `{T,R}` reflects that no scale offset exists today (`schema.usda:772-777`
declares `translationOffsets` and `rotationOffsets` only). Adding one is a
follow-up, not part of this change.

SingleChainIk needs the `validate` hook: `:2337-2348` requires set *equality*
between `rigExec:moves` and the chain inferred by walking `rigExec:endJoint`'s
ancestors to `rigExec:firstJoint` — a cross-check between the target set and two
source roles that `arity` has no slot for.

One registration per operator. Every compile check, arity rule, mask validation and
dispatch reads the table. `_IsSourceFrameConstraintType` and
`_IsFrameConstraintType` collapse into registry lookups. `RigExecCustomConstraint`
joins as a registration with a null `solve`, so its "no registered evaluator" error
becomes a table property rather than the hand-written special case at
`:1221-1231`. Adding a seventh operator becomes one registration instead of ~20
edits.

The six kernel *bodies* are unchanged — they are already `RigExecPointFrame` in,
`RigExecPointFrame` out (`solvers.h:167-207`) and know nothing about targets. What
changes is the **call**: the geometry arm invokes them at weight 1 so the envelope
is applied exactly once, downstream (see 4.4).

### 4.3 The authored surface

`RigExecConstraint` gains `RigExecMoverAPI` as a **built-in** API schema:

```
class "RigExecConstraint" (
    inherits = </Typed>
    prepend apiSchemas = ["RigExecMoverAPI"]
)
```

Every constraint then owns `rigExec:moves` and `inputs:enabled` by construction.
Existing `apiSchemas = ["RigExecMoverAPI"]` authoring stays valid, just redundant.

This has one confirmed consequence, and it is **not** grouping prims — those are
typed `Scope` (`testRigExecConstraints.cpp:665`) and never inherit
`RigExecConstraint`. It is a *typed constraint prim authored without
`rigExec:moves`*: today it takes the `if (!moves)` branch at `:1099` and reports
*"has executable constraint semantics but no rigExec:moves relationship"*; with the
API built in, `GetRelationship()` is valid-but-empty and it reports *"Mover has no
moves targets"* (`:1113`) instead. Both are hard errors, so nothing compiles away
silently — but the two must not both survive for the same authoring state. `:1113`
wins; the `_IsFrameConstraintType` guard at `:1100-1106` becomes dead and is
deleted, and the binding-epoch digest's matching `if (!moves)` at `:711-714` is
updated in lockstep so the digest and the compiled mover set agree.
`testRigExecConstraints.cpp:669-679` asserts on the substring "no moves targets"
and keeps passing.

Channel model on the base:

```
bool    inputs:affectTranslationX / Y / Z = true
bool    inputs:affectRotationX    / Y / Z = true
bool    inputs:affectScaleX       / Y / Z = true
double3 inputs:translationOffset = (0, 0, 0)
double3 inputs:rotationOffset    = (0, 0, 0)
double3 inputs:scaleOffset       = (0, 0, 0)   # additive; see below
uniform token rigExec:rotationOrder = "XYZ"    # declared once, not three times
float   inputs:defaultWeight = 1               # the envelope; see 4.5
rel     rigExec:weightObject                   # optional, <= 1; geometry domain only
uniform bool rigExec:locked  = false
```

`inputs:scaleOffset` stays `(0,0,0)`. The Scale kernel is **additive** —
`solvers.cpp:758` is `targetScale = targetScale / totalWeight + params.offset` — so
`(0,0,0)` is the correct identity and the default does not change. What 3.3 gets is
the algebra written into the doc string and recorded in the registry's
`offsetAlgebra`. Switching to a multiplicative offset changes the kernel and must
not be smuggled in as a default flip.

**Parent restates `inputs:affectScaleX/Y/Z = false` in its own block.** The FBX
default is deliberate (`schema.usda:769-771`), the evaluator passes an explicit
`false` fallback at `:5173-5176`, and `testRigExecConstraints.cpp:175-178` asserts
it. Because `generatedSchema.usda` is flattened, the override must be authored in
Parent's block or every existing Parent constraint starts writing scale.

**The legacy `inputs:affectX/Y/Z` triples are removed** — all four declarations
(`schema.usda:680, 711, 727, 746`), read by name at `:5139-5141`. An authored
`inputs:affectX` on a constraint is a compile error naming the group-qualified
spelling, exactly as `inputs:weight` is. Without this, 3.2 is unfixed.

Validation reads the capability sets from 4.2:

- a per-axis mask authored outside `maskable` is a compile error;
- an offset authored outside `offsettable` is a compile error;
- `rigExec:rotationOrder` on an operator with `usesRotationOrder == false` is a
  compile error;
- `rigExec:weightObject` authored on a Transform-domain constraint is a compile
  error naming `inputs:defaultWeight`; its element count must match the target's
  point count;
- read-phase metadata authored on a constraint source relationship is a compile
  error. `bindFrameSource` pins every source tap to `basePhase` (`:2206`) and
  `kPhased` (`:1417-1424`) covers only `rigExec:transform`, `cage`, `surface`,
  `curvenet`, `bindCoordinates` and `driverCurve` — yet `RigExecResolveReadPhase`
  (`moverGraph.cpp:592-604`) reads phase metadata off any `UsdObject`, so an author
  can legally write it on `rigExec:sources` today and have it silently ignored.

**No attribute is ever silently ignored.**

Source roles keep their readable names — `rigExec:worldUpObject`,
`rigExec:poleVectorObjects`, `rigExec:aimTarget` are *not* renamed. The defect is
not the names; it is that each role has its own binding, validator and diagnostic
vocabulary.

Aim is the exception the registry must **encode rather than erase**: whether
`rigExec:sources` is authored selects between two different up-resolution
algorithms — `params.preserveInputUp = authoredSources.empty()` (`:5242`). Every
shipped Aim asset (`08_AimEyes:135`, `10_AimXformTurret:92`, `ArmRig:283`) uses the
legacy `aimTarget` spelling and depends on `preserveInputUp = true`. Aim therefore
registers two distinct primary roles — legacy `aimTarget` (preserveInputUp) and FBX
`sources` (worldUpType) — and authoring both is a compile error. The
`rigExec:aimAxis` / `inputs:aimVector` precedence rule (`:5206-5219`) is likewise a
registry property.

#### `rigExec:preserve` and `rigExec:upPolicy`: translated, not implemented

`rigExec:preserve` (`schema.usda:696-698`, default `["origin", "scale"]`, authored
in `08_AimEyes:118`, `:137` and `ArmRig:284`) is **the channel mask at opposite
polarity**: it names the components of the constrained prim's transform the solve
must leave alone, where `inputs:affect*` names the ones it writes.
`tools/rigExecPose.cpp:365-368` states the intent — *"A constraint's whole job is
usually orientation — the default preserve set pins the origin."*

It is read nowhere in `rigEvaluator.cpp`, and `docs/dead-surface-removal.md:163`
deliberately verdicts it **"keep — authored intent for aim"**, in the category
*"authored in the examples and silently ignored by the engine. Deleting them would
discard expressed intent; implementing them is the real fix."* `README.md:102`
tracks "aim `upPolicy`/`preserve` consumption" as an open conformance gap.

Under D4 it needs no implementation, because it is redundant with the channel
model:

```
preserve = ["origin", "scale"]  ==  affectTranslation* = false
                                    affectScale*       = false
                                    affectRotation*    = true
```

which is exactly what Aim's registry row `writes = {R}` already encodes. Two facts
make translation lossless rather than lossy:

- every authored value in the corpus is the default — all three are
  `["origin", "scale"]`, nothing asks for anything else;
- the Aim kernel already behaves that way structurally. It decomposes the input
  frame, writes only `inputParams.rotation`, and reconstructs through
  `_FrameFromConstraintParams`, so translation and scale pass through untouched;
  the legacy 4-arg variant rebuilds each axis as `origin + dir * length`,
  preserving origin and axis lengths alike.

So `preserve` is currently *descriptive of what the kernel does*, not a control
being disregarded in a way that changes results. The resolution:

- **map it at bind time** — `"origin"` clears `affectTranslation*`, `"scale"` clears
  `affectScale*`;
- **authoring both `preserve` and the group masks is a compile error**, the same
  rule as `aimTarget` versus `sources`;
- **a non-default `preserve` is a compile error.** `["scale"]` alone would ask Aim
  to move the origin, which requires `writes` to include translation and which the
  kernel does not do. This is the genuinely dangerous case today: currently
  accepted, silently ignored.

`rigExec:upPolicy` (`schema.usda:692-695`) resolves for free: its `allowedTokens`
list has exactly one entry, `"preserveInputUp"`, so it carries no information, and
the behavior it names is already the `preserveInputUp = authoredSources.empty()`
rule encoded in Aim's two primary roles above.

Together these close both halves of the `README.md:102` gap without implementing
any new behavior.

Further behavioral fixes:

- Zero-weight sources are skipped before frame validation everywhere (Position's
  behavior wins; fixes Aim's NaN at `:5195`).
- Array weights read through `_resolvedInputs`, so the array read path stops
  diverging from `inputs:weight`. This is uniformity, not a new capability:
  `inputs:sourceWeights` is `float[]` (`schema.usda:636`) and the three
  property-mover types are type-gated to Float, the float3 family and Matrix4d
  (`:1351-1377`), so nothing can drive it until a float-array mover exists.
- **Source-order sensitivity is documented.** Anchoring is the correct behavior —
  averaging Eulers without an anchor is ill-defined — so the schema doc changes to
  say what the code does, rather than the code changing.

### 4.4 The geometry domain

```
D        = F_solved(weight = 1) * F_base^-1
F_base   = the target prim's own asset-relative matrix (frameFromXform, :4504-4520)

transform domain:  publish F_solved(defaultWeight) * M_prim -> pose.providerXforms
geometry domain:   P'[i] = lerp(P[i], D.TransformAffine(P[i]), w[i])
                                                            -> pose.movedProperties
```

The envelope is applied exactly **once** per domain: through the kernel's
per-channel blend in the transform domain, and through the per-point lerp in the
geometry domain. `D` itself is **unweighted**, so the geometry arm must call the
kernel with `params.weight = 1.0` — today `:5141, 5151, 5160, 5177, 5228` all pass
the authored envelope, and `solvers.cpp:724-730` folds it into the result. Leaving
that in place would square the envelope: `defaultWeight = 0.5` would give 0.5 in the
transform domain and 0.25 in the geometry domain.

The two blends are not the same interpolation at intermediate weights — the
transform arm lerps decomposed channels, the geometry arm lerps positions — so D2
is exact at `w ∈ {0, 1}` and holds to chord-vs-arc error elsewhere.

`TransformAffine` is USD's row-vector convention, matching
`RigExecApplyWeightedMatrix` (`solvers.cpp:367-378`), which already is exactly this
kernel. No new math.

**The op needs its own input edge.** The constraint gains a
`RigExecRevisionOp::Matrix` arm in `RigExecRevisionOpForSchema`
(`moverGraph.cpp:443-473`, currently `nullopt` for every constraint type), and the
chain gate at `:2506-2509` (currently `.points`-only) is relaxed to admit it. That
alone is not sufficient: `RigExecAssembleMatrixParameters` (`moverGraph.cpp:903`)
returns MoverFailed unless both `values.transform` and `values.weights` are
non-null, and those come from `rigExec:transform` (`:2524-2528`) and
`rigExec:weightObject` (`:2530-2534`), neither of which a constraint has today. The
constraint's solved delta therefore needs its own tap, and the constant
`RigExecWeightPacket` of 4.5 must be synthesized upstream of `values.weights`, not
only in the evaluator.

**Which maps re-key.** `_frameChains` (`rigEvaluator.h:441`),
`_providerBaseFrameTaps` (`:446`), `_xformDerivedProviders` (`:426`) and the
evaluation-time `baseFrames` / `finalFrames` / `restFrames` / `xformDerivedBases`
stay keyed by `binding.primPath`. Only `pose.movedProperties` is keyed by
`binding.pointsProperty`. A `.points` key reaching `newFrameChains` (`:2385`) is
rejected by the provider classifier at `:2394-2415` and, if that gate is loosened,
hard-fails the whole generation at `:4543-4546` ("constraint target has no base
frame").

**The parity twin is mandatory.** The oracle at `:5736-5800` independently
re-derives every points chain and refuses to publish the whole generation on
disagreement (`:5820-5822`). `RigExecCurvenetMover` gets an explicit skip at
`:5764`; constraints must not. The twin is nearly free because the solve is already
scalar frame-in/frame-out, and skipping it would forfeit the check exactly where
the newest code is riskiest.

A geometry-domain constraint joins `newGraphChains` and therefore participates in
derived normals/extent maintenance exactly like a deformer, including the hard
rejection of authored normals on a non-mesh target (`:2583-2590`).

**Competing-writer detection** (`:1557-1559`) currently keys `byTarget` on raw
paths. It changes to key on **`primPath` alone** — under `(primPath, domain)` the
constraint on `/M` and the deformer on `/M.points` still land in different buckets
and still never meet. Domain rides along per-writer, for diagnostic text only. The
same edit applies at `:1615`, where `lastFrameWriterOrdinal` is recorded only when
`t.IsPrimPath()`, so a geometry-domain constraint is currently invisible to the
unsatisfied-final-read rule at `:1645-1653`.

### 4.5 The envelope

```
w[i]      = packet.Resolve(i, pointCount)
effect[i] = w[i]
```

The packet is **synthesized when no weight object is bound**:

```cpp
RigExecWeightPacket{ representation = "constant",
                     rangePolicy    = "strict",
                     defaultWeight  = <inputs:defaultWeight>,
                     valid          = true }
```

`RigExecWeightPacket::Resolve(i, count)` (`types.h:85-87`) already broadcasts a
constant, so there is no new resolve machinery. A `-1` return is a cardinality
mismatch and is MoverFailed for the whole target, never a usable weight —
`lerp(P[i], …, -1)` would extrapolate backwards.

Every constraint has a per-element weight field. `inputs:defaultWeight` is its
constant value; a weight object refines it per element. The transform domain is the
one-element case, which is why the same attribute is meaningful in both domains
rather than being a geometry-side special case.

`inputs:weight` is **removed** from `RigExecConstraint`. Both attributes default to
1, so the rename is value-neutral — but it is **not** behavior-neutral: the
`<= 0` dormant pass-through at `:4947-4953` fires *before* sources, effectors or
poles are resolved, so a `defaultWeight = 0` constraint with a bound map would
never deform. That early-out becomes conditional on **no weight object being
bound**. An authored `inputs:weight` on a constraint is a compile error naming
`inputs:defaultWeight`.

This deletes real boilerplate: any constraint that today would need a
`RigExecStaticWeight` prim carrying only `defaultWeight = 1`,
`representation = "constant"`, `rangePolicy = "strict"` says it in one line
instead. (`examples/08_AimEyes.usda:76-95` is *not* such a case — `EyeLW`/`EyeRW`
are consumed by the two `RigExecMatrixMover` prims at `:145-165`, not by the Aim
constraints, which move the joints.) Within constraints `rigExec:weightObject` is
uniformly optional; 3.8's divergence between `RigExecMatrixMover` and
`RigExecBlendShapeMover` is untouched here and stays open.

**Known trade-off.** Because a weight object supersedes the envelope rather than
multiplying with it, once a map is bound there is no scalar knob to dial that
constraint down without editing the map. This only affects the geometry domain.
Switching to "envelope always multiplies" is a one-line change to the resolve.

---

## 5. Phasing

| phase | contents | schema regen |
|---|---|---|
| 1 | target binding, real `UsdGeomXformable` predicate, split the canonicalizer | no |
| 2 | handler registry; the ~20 type-name sites collapse | no |
| 3 | channel model, source roles, envelope, `rigExec:weightObject`, validation fixes | **yes** |
| 4 | geometry domain, parity twin, competing-writer keying | no (weightObject landed in phase 3) |

Phase 1 alone unbreaks **two of the three** failing examples —
`rotateConstraint.usda` and `aimtest.usda`, both bare `def Mesh` targets.
`aimtest_points.usda:73` authors the geometry-domain spelling, so it is the
**phase-4** acceptance case.

This is verified: the minimal form of phase 1 (skip canonicalization for
frame-constraint movers at `:1136`) compiles `rotateConstraint.usda`, drives the
cylinder's transform from identity at frame 1 to a 180° Y rotation at frame 100,
and passes all 12 ctest suites. That patch is recorded but not applied; the tree is
unmodified.

---

## 6. Testing

The gap that let this ship: `tests/testRigExecConstraints.cpp` has **zero** Mesh,
BasisCurves, Points, or `.points` coverage. Every target across all 744 lines is an
`Xform` or a `RigExecJoint`.

**The headline test** is the D2 invariant:

> Constraining `</Geom/M>` and `</Geom/M.points>` with no weight object and the same
> `inputs:defaultWeight` produces identical world-space geometry — asserted at
> `defaultWeight` 1 **and** 0.5, since 1 is the one value where a double-applied
> envelope is invisible.

Per phase:

- **1** — each operator against a Mesh, a BasisCurves and a Points target; the
  non-Xformable rejection; the point-domain "did you mean `.points`" diagnostic,
  table-driven over all eight point-domain mover types including
  `RigExecMatrixMover`.
- **2** — registry coverage for all seven operators; a table-driven test asserting
  every registered operator honors arity, domains and the capability sets
  identically.
- **3** — mask/offset/rotationOrder outside the capability sets is rejected;
  zero-weight source parity between Aim and Position; authored `inputs:weight` or
  `inputs:affectX` on a constraint is rejected; Parent's `affectScale = false`
  survives the hoist; read-phase metadata on `rigExec:sources` is rejected; the
  default `rigExec:preserve` translates to the same masks the Aim kernel already
  honors (assert `08_AimEyes` and `ArmRig` are byte-identical before and after),
  a non-default `preserve` is rejected, and `preserve` alongside the group masks
  is rejected.
- **4** — the invariant above; envelope at 0, 0.5 and 1; a weight object superseding
  the envelope; `defaultWeight = 0` **with** a weight object resolving 1 must still
  deform; the parity twin agreeing; competing-writer ordering across `/M` and
  `/M.points`; a Points/BasisCurves target with authored normals is rejected, not
  silently skipped.

Beware `tests/testRigExecArm.cpp:1615-1616` (inside `TestAimConstraintDrivesXform`,
declared at `:1573`), which asserts
`movedProperties.count("/TurretAsset/Geom/Turret/Barrel.points") == 0`. The turret
targets an Xform so it should keep passing, but its meaning inverts the moment a
constraint writes points.

---

## 7. Migration and blast radius

**`inputs:weight` -> `inputs:defaultWeight`**, complete surface:

| | |
|---|---|
| schema | `RigExecConstraint`: drop `inputs:weight`, add `inputs:defaultWeight = 1` and `rel rigExec:weightObject` |
| code | `:4934` (the read); `:4947-4953` (the `<= 0` dormant pass-through) becomes conditional on no weight object being bound |
| examples | **4 opinions in 3 files** — `08_AimEyes:108, :127`, `10_AimXformTurret:90`, `ArmRig:280`. `08_AimEyes:109` and `:128` also carry `float inputs:weight.spline` timesamples that must rename with the attribute or the animation silently detaches. **Not** `ArmShotAnim:60`, `ArmRig:113` or `ArmRig:256` — those are `RigExecBlendPointFrames` / `RigExecBlendInput` weights and a blind rename kills the IK/FK blend. |
| masks | `inputs:affectX/Y/Z` -> `inputs:affectRotation*` (Aim, Rotation), `inputs:affectTranslation*` (Position), `inputs:affectScale*` (Scale); schema `:680, 711, 727, 746`; code `:5139-5141`; tests `testRigExecConstraints.cpp:250, 268` |
| legacy aim | `rigExec:preserve` (`schema.usda:696-698`) and `rigExec:upPolicy` (`:692-695`) stay in the schema and stay authored — **no asset edits**. They are translated at bind time onto the channel masks and the `preserveInputUp` role (4.3). `08_AimEyes:118, :137` and `ArmRig:284` must evaluate byte-identically before and after. |
| tests | `testRigExecConstraints.cpp:97, 106, 371, 384` |
| docs | rewrite the doc strings that instruct authors to use `inputs:weight` — `schema.usda:601, 617, 708, 724, 743`. All re-emit into all seven flattened concrete blocks. `RigExecFloatMathMover` (`:870`), `RigExecVec3fMathMover` (`:1401`), `:1422` and `:856` keep their own `inputs:weight` and must not be caught by a blanket rename. |
| untouched | `RigExecBlendPointFrames`, `RigExecBlendInput`, `RigExecMatrixMathMover` keep their own `inputs:weight`. `:3647` is `RigExecBlendInput`'s weight read from the geometry chain (`_EvaluateChain`, `:3544`); `:4106` and `:4259` are property-domain math movers. None is a constraint. |

**Schema regeneration** (phase 3 only). Run `bin/gen_schema.sh`, then strip the
`LibraryPath` / `@PLUG_INFO_*@` placeholders from `plugInfo.json`. Both outputs are
checked in. `generatedSchema.usda` is fully flattened — inheritance is erased — so
one property added to the base re-emits into all seven concrete blocks, roughly a
7× diff. `CMakeLists.txt:286-289` puts both files on `CONFIGURE_DEPENDS`, so a
schema edit forces a cmake reconfigure before tests see it.

**Imaging: zero changes.** `_ComputeDrivenXform` (`sceneIndices.cpp:1442-1505`) is
prim-type-agnostic and already walks ancestors downstream of flatten, so a driven
Mesh carries its children today. `snapshotStore.h:93-118` already dirties and diffs
both `hasPoints` and `hasXform`. The only genuinely new case is a mesh that is both
moved and deformed in one generation.

**The whole constraint family is unshipped** — `git diff` on `schema.usda` is two
hunks: a one-line doc-string edit at `@@ -4,7 +4,7 @@` and the whole constraint
family at `@@ -591,24 +591,257 @@` (`--numstat` 240/7). Re-basing costs nothing in
compatibility, which is why this is the moment to do it.

---

## 8. Deliberately out of scope

- **Geometry-derived pivots** (centroid, index triple, surface sample, curve
  parameter). The math exists — `geometryKernels.cpp:47-51`, `curvenet.h:212`,
  `cutMesh.cpp:367` — but behind no constraint-facing API.
- **Points as a constraint source.** A Mesh is already a legal source (`:2209`) but
  only its xform is read. Blocked structurally: constraints evaluate at `:4920`,
  geometry chains at `:5490`, and there is no geometry→transform edge.
- **A multiplicative `inputs:scaleOffset`**, which changes the kernel (4.3).
- **A float-array property mover**, without which `inputs:sourceWeights` cannot be
  driven (4.3).
- **3.8's weight-object optionality divergence** between `RigExecMatrixMover` and
  `RigExecBlendShapeMover`; neither is a constraint.
- **Per-element broadcast** and **curve-parameter constraints**, which are arguably
  different operators rather than target kinds.
