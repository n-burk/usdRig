# Intervening Xforms between the asset root and a provider

Date: 2026-09-09. Status: implemented 2026-09-10 (mechanism C).

## 0. Request

"When Xform1 in `examples/spider_legs_assembly_ref.usda` is rotated, the
joint children and controller children do not inherit the transforms
currently."

Answered during triage: the evaluator is not silent about this — it
warns seven times per compile — but the warning describes the behaviour
as an extent inaccuracy, and what an author actually sees is a
transform that does nothing. The fix composes the intervening chain into
the provider's **rest frame**, which is asset-space; every consumer
downstream then needs no change at all.

## 1. The observed behaviour

Reproduced through the `_rigexec` Python binding only — no manipulator,
no imaging. `examples/spider_legs_assembly_ref.usda` places a plain
`Xform "Xform1"` between `RigExecRoot1` and the `Joints` / `Controller`
/ `Solvers` scopes. Adding `rotateZ = 90` to it:

| provider | frame before | frame after |
|---|---|---|
| `Joints/Shoulder` | `(-1.2342, 0.0750, 0)` | `(-1.2342, 0.0750, 0)` |
| `Joints/…/ankle` | `(-0.6996, 5.0652, 0)` | `(-0.6996, 5.0652, 0)` |
| `Controller/foot` | `(6.8811, 0, 0)` | `(6.8811, 0, 0)` |

Bit-identical. Compiling the rig emits seven copies of:

```
Xformable /World/RigExecRoot1/Xform1 sits between the asset root and
provider …; its transform is not composed into the provider's frames,
so the computed extent places the guide as if it were identity
```

Moving the same op **above** the `RigExecRoot` — so it becomes the asset
root — behaves correctly, with no warnings:

| shape | asset root | guide drawn at |
|---|---|---|
| `Xform1` inside the rig | `/World` | `(-1.2342, 0.0750, 0)` — unmoved |
| `Xform1` above `RigExecRoot1` | `/World/Xform1` | `(-0.0750, -1.2342, 0)` — the 90° |

So there is exactly one band where a transform is dropped: strictly
between the asset root and a provider.

## 2. Facts the design relies on

Each was read in the tree at `02c86c5` plus the working-tree changes,
and the runtime behaviour in §1 was measured against the same build.

1. **`computeRestFrame`'s parent input is a `NamespaceAncestor`, and a
   plain Xform does not satisfy it.** `computations.cpp:432-435`
   declares `NamespaceAncestor<RigExecPointFrame>(computeRestFrame)
   .InputName(parentRestFrame)`. Only three schemas register that
   computation — `RigExecJoint`, `RigExecControl`, `RigExecVolumeWeight`
   (`computations.cpp:529-559`, via `RIGEXEC_REGISTER_XFORMABLE`). A
   `Scope` or a `UsdGeomXform` provides nothing, so the walk passes
   through it, and with no RigExec ancestor at all the input is null.

2. **A null parent input means identity.** `_JointRestSpace`
   (`computations.cpp:250-270`) ends `return rest * _SpaceFromFrame(…
   parentRestFrame)`, and its own comment states the rule: "A provider
   with no RigExec ancestor resolves against identity." That single
   resolution is the whole defect, exactly as
   [parent-local rest](2026-09-07-parent-local-rest-design.md) §2.1
   found for the ancestor input that was missing entirely.

3. **In the reported file, only chain ROOTS have a dropped transform.**
   A nested joint's nearest RigExec ancestor is its parent joint, which
   resolves normally; the providers with an absent RigExec ancestor are
   `Joints/Shoulder`, the three controls and the solver. That is a
   property of this file, not of the problem — an Xform between two
   joints in a chain drops a transform at a non-root, which is why §3
   does not stop at the chain roots.

4. **The evaluator does not do its own rest math.** `RigExecRigEvaluator`
   seeds `restFrames` / `baseFrames` from exec taps
   (`rigEvaluator.cpp:6591-6597`, `_poseSeedFrames` / `_poseSeedRests`).
   Fixing the computation fixes the evaluator with it — there is no
   second copy of the math to keep in step, which is the hazard
   `frameExtraction.h` exists to prevent.

5. **The asset root is `_rigPath.GetParentPath()`**, in both the
   evaluator (`rigEvaluator.cpp:2017`) and the imaging bridge
   (`bridge.cpp:714`).

6. **`RigExecRoot` is not Xformable.** `schema.usda:95-96` inherits
   `Imageable`, and `UsdGeomXformable(rigRoot)` is false at runtime. It
   can never carry an op itself, so it is a safe terminator for an
   upward walk and there is no third place to get this wrong.

7. **The asset-relative ancestor math already exists.**
   `rigEvaluator.cpp:6570-6585` uses
   `UsdGeomXformCache::ComputeRelativeTransform(prim, assetRoot,
   &resetsBelowAsset)` to place `_xformDerivedProviders` — constraint
   targets that are plain Xformables. That is precisely the quantity
   this design needs, already computed against a per-evaluate cache.

8. **`rigExec:restFrameVersion` plus `tools/migrateRestToLocal.py`
   exist** as the lever a previous rest-convention change used; v2 is
   parent-local rest. Recorded because it is the obvious tool to reach
   for here, and §5 declines it.

9. **OpenUSD already registers exec computations for
   `UsdGeomXformable`.** `pxr/exec/execGeom/xformable.cpp:50` opens that
   schema and defines `computeLocalToWorldTransform`. RigExec therefore
   *cannot* open it a second time — that is the one-schema-one-opening
   rule `computations.cpp:549-556` states, and
   `exec/testenv/TestExecConflictingPluginRegistration{1,2}.cpp` exists
   to test the conflict.

10. **That computation reads `xformOp:transform` and nothing else.**
    `execGeom/xformable.cpp:24-48` declares exactly two inputs: the
    `xformOp:transform` attribute and the same computation on its
    namespace ancestor. It never reads `xformOpOrder`, so a prim posed
    with `xformOp:rotateZ` — which is what the reported file uses, and
    what usdview and the gizmo author — is invisible to it. It also
    returns **world**, not asset-relative.

## 3. Semantics

A provider's rest frame is measured **in asset space**. Today that is
true only when nothing Xformable sits between the asset root and the
provider; the proposal makes it true unconditionally:

> `restFrame(P) = local(rest avars of P) * rest:space(P) * X(P) * restFrame(A(P))`
>
> where `A(P)` is P's nearest RigExec ancestor — its rest frame, or
> identity when there is none, which is the asset root — and `X(P)` is
> the composed transform of the non-RigExec Xformables lying strictly
> between P and `A(P)`.

`X(P)` is the whole change. Today it is silently identity for every
provider; the defect is that omission and nothing else.

Nothing here is authored. `X(P)` is read from the stage on each
evaluation and composed into the frame in memory. The reported file
works with `Xform1` exactly where it is, and no file — the rig, the
asset, or the shot — is ever rewritten by this feature.

### It does not reduce to a right-multiply

Worth pinning down, because the cheap implementation is wrong in a case
that will occur. When every intervening Xform sits **above** every chain
root — one Xform containing the whole rig, which is the reported case
and the common one — then `frame_true(P) = frame_exec(P) * A` for every
P in the subtree, with one shared `A`. That tempts a single multiply
applied to every seeded frame.

It breaks as soon as an Xform sits **between** two joints in a chain.
With `A1` above the root joint R and `A2` between R and its child C, the
correct answer is `local(C) * A2 * local(R) * A1`, while the uniform
multiply gives `local(C) * local(R) * A2 * A1` — `A2` in the wrong
place. The implementation therefore walks providers in namespace order,
parents before children, and rebuilds each frame from its own local
factor:

> `frame_true(P) = frame_exec(P) * frame_exec(A(P))⁻¹ * X(P) * frame_true(A(P))`

with `frame_exec(A(P))` taken as identity when P is a chain root, which
collapses the expression to `frame_exec(P) * X(P)` there.

### What does NOT change

This is the part that makes the change small, and it is worth stating
before the code:

- **Imaging.** Guides are published as `frame * assetRootWorld`
  (`sceneIndices.cpp:590`, `:936`, `:1118`) with `assetRootWorld` read
  from the asset root (`_ResolveAssetRootWorld`, `:2180-2210`). With the
  chain inside the frame, that product is the correct world transform.
  No edit — and the standing comment at `sceneIndices.cpp:581-586`
  ("that Xform's contribution is already baked into the rig's own
  frames — composing the parent's flattened matrix would apply it a
  second time") stops being aspirational and becomes true.
- **Extents.** ~~Computed asset-relative; they inherit the fix.~~
  **Wrong, and it shipped that way for one round.** "Asset-relative" is
  not "local", and an extent is local: `UsdGeomBBoxCache` multiplies it
  by the prim's own local-to-world, which contains the very Xform the
  frames now carry. The published bounds went out at that transform
  applied twice, so the guide drew in one place and its bounding box in
  another, offset by exactly the intervening Xform.
  `registry.cpp:_AssetSpaceToLocal` divides it back out on the snapshot
  branch. The rest-pose fallback needs no division -- it reads authored
  attributes, which never carried the transform -- so both branches
  return the same local quantity and the box does not jump when a
  generation is published.
- **The compile-time ancestor walk at `rigEvaluator.cpp:2069-2088`.**
  It stays, and only its message changes: from "not composed into the
  provider's frames" to a note that the transform now IS composed. See
  §5 — it is what tells a compensating author why their rig moved.

### What does change

- `rigEvaluator.cpp`: the provider seeding at `:6591-6597`, per the
  mechanism chosen in §4 and the namespace-ordered walk in §3, plus
  rewording the ancestor-walk diagnostic at `:2069-2088`.
- `computations.cpp`: no code, but a comment beside `_JointRestSpace`
  recording that its "resolves against identity" rule is now completed
  by the evaluator (see §4 C, and risk 3).
- `plugin/rigExecUsdview/gizmoMath.py`: it reimplements the rest
  composition in Python and walks past an intervening Xform at the same
  three points exec does (`_FindParentXformable` at `:407`, `:457`,
  `:652`). Gains `InterveningXform()`, applied in `RestSpace` -- from
  which the default and posed families inherit it, since they compose
  `RestSpace(prim) * RestSpace(parent)^-1` -- and in `Q`, whose `Qrest`
  reads no namespace ancestor and so cannot inherit it. Its
  asset-to-world handling at `:645-647` and `:730-733` already mirrors
  the imaging and needs nothing.

## 4. Mechanism

The composition must ride on a **declared dependency**. Reading ancestor
`xformOps` inside a `VdfContext` callback without declaring them would
leave the rig posed from a stale ancestor after any edit to it, which is
a worse failure than the one being fixed.

Three candidates. The first two were the obvious ones and facts 9 and 10
kill both, which is the whole reason this section exists.

### A. Register the chain on `UsdGeomXformable` — **not available**

The shape one would reach for: give `UsdGeomXformable` a computation
returning `localTransform * NamespaceAncestor(sameComputation)`, give
`RigExecRoot` one returning identity to terminate the walk at the rig
root, and let each provider's existing
`NamespaceAncestor(computeRestFrame)` find the intervening `Xform1`
instead of nothing.

Refuted by fact 9: `execGeom` has already opened `UsdGeomXformable`.
A second opening is the conflict OpenUSD tests for. Dead — and cheap to
have found, which is what §7 risk 1 was for.

### B. Consume `execGeom`'s `computeLocalToWorldTransform` — **too narrow**

Declaring
`NamespaceAncestor<GfMatrix4d>(ExecGeomXformableTokens->computeLocalToWorldTransform)`
as an input to `computeRestFrame` needs no foreign registration and gets
invalidation for free.

Refuted by fact 10 on two counts, either of which is fatal:

- it reads `xformOp:transform` alone, so the `xformOp:rotateZ` in the
  reported file — and everything usdview's own manipulator authors — is
  simply not seen. A fix that silently ignores the ops people actually
  use is worse than the bug;
- it returns world, not asset-relative, so it would have to be divided
  by the asset root's own value, and §3 depends on frames staying
  asset-space.

Worth revisiting only if execGeom grows full `xformOpOrder` support.

### C. Compose in the evaluator, per evaluation — **recommended**

`RigExecRigEvaluator` already supplies solver-posed joint frames as
value overrides (`computations.cpp:325-330`), already builds a fresh
`UsdGeomXformCache` per `Evaluate` (`rigEvaluator.cpp:6568`), and
already computes exactly this matrix for constraint targets via
`ComputeRelativeTransform(prim, assetRoot, …)` (fact 7). Compose the
chain into the chain-root seeds at `rigEvaluator.cpp:6591-6597`, using
the machinery three lines below it.

`UsdGeomXformCache` is the real transform stack, so unlike B this sees
every op form. It is asset-relative by construction, so §3 holds and
nothing downstream changes. It is contained to one file. And a fresh
cache per evaluation means an edit to the intervening Xform is picked up
on the next evaluate — the same guarantee `_xformDerivedProviders`
already relies on.

**Cost, stated plainly:** a bare exec request for `computeRestFrame`
that never goes through `RigExecRigEvaluator` keeps the old answer. That
is the divergence `frameExtraction.h` exists to prevent, and this design
accepts it knowingly rather than by omission. Every surface a user can
see — pose, guides, extents, manipulator — goes through the evaluator,
so the divergence is invisible in practice, which is exactly what makes
it worth a comment at both sites rather than one.

**Recommendation: C**, with the divergence recorded in
`computations.cpp` beside `_JointRestSpace` and in `frameExtraction.h`'s
banner, and revisited if execGeom ever reads `xformOpOrder`.

## 5. No migration, and nothing authored

**Requirement (user-directed 2026-09-10): the Xform is not baked into
the USD; it is composed at execution time.** That is a constraint on the
design, not a preference, and it rules out three things that would
otherwise have been on the table:

- no migration tool and no rewrite of anybody's layers — `X(P)` is read
  fresh on each evaluation and composed in memory;
- no `restFrameVersion` bump and no version gate. A gate would mean the
  reported file keeps misbehaving until it is migrated, which answers
  the request with a second step the requester has explicitly declined;
- no API schema to be applied to the intervening Xform. Tagging the prim
  would be authoring by another name, and it is also why §4's
  register-on-a-schema-we-own variant is not merely awkward but
  disqualified.

The file works as authored. `Xform1` stays exactly where the author put
it, with the ops the author wrote, and the rig follows it.

### The one rig this changes underneath

A rig whose author **compensated** — cancelling the dropped transform by
hand inside `rest:space` or the rest avars — has been getting the right
answer for the wrong reason, and will double-apply once `X(P)` starts
contributing.

That rig is rare and it is loud today: a provider under an intervening
Xformable warns seven times per compile and draws its guides as though
the Xform were identity, so compensating means having worked against a
standing diagnostic. The shipped examples are not exposed at all —
`spider_leg.usd`, `spider_leg_ik.usd` and `ArmRig.usda` place the
RigExecRoot directly under the asset root with nothing between it and
the providers.

The remedy is a diagnostic, not a rewrite: the compile-time ancestor
walk that raises today's warning stays, and says instead that the
intervening transform is now composed — so an author whose rig moves
unexpectedly is told why, in the same place they were previously told it
was ignored. Their fix is to delete their own compensation, which is
their edit to make.

## 6. Testing

Implemented as `tests/python/test_intervening_xform.py`
(`ctest -R testInterveningXform`, 9 groups). What it pins:

- a rig with no intervening Xform is unchanged, on both the evaluator
  and the gizmo -- the fast path, which is every shipped example;
- one Xform above the whole rig moves every provider, joints and
  controls alike;
- a **rotation**, not just a translation: composing on the wrong side of
  the rest offset is invisible under translation and obvious under a
  90-degree turn;
- an Xform **between two joints** applies at its own level, which is the
  case the uniform right-multiply gets wrong;
- an Xform **above the asset root** is still not composed here, so the
  imaging's own `* assetRootWorld` cannot double-apply;
- the Xform is re-read on **every evaluation**, on the same evaluator
  object, after an edit -- the stale-pose risk in §7.4;
- evaluation authors **nothing**: the layer is byte-identical after two
  evaluations (§5's requirement, as a test);
- the **manipulator agrees with the pose** across three
  translate/rotate combinations, which is the property that matters once
  both sides compose it.

`tests/testRigExecImaging.cpp` now asserts the walk stays *quiet* about
an intervening Xformable, where it previously asserted the warning, and
adds `TestInterveningXformLeavesTheLocalExtentAlone`: with a pure
translation on the intervening Xform, the LOCAL extent must come back
bit-for-bit unchanged. Verified to fail without the fix -- it reports the
box moved by exactly the +13 that was authored.

Two tests were already failing before this change and still are:
`testRestLocal` and `testRestMigration`, both against the re-baked
`examples/components/spider_leg_ik.usd` from `2e95684`. Baselined by
stashing this change and re-running; unrelated to it.

### Original plan

- **Headless, no Qt** (`tests/python/`): the §1 table as a regression —
  rotate an intervening Xform, assert every chain-root frame moves by
  exactly that rotation and every nested joint follows; assert a rig
  with no intervening Xform is byte-identical before and after the
  change; assert a provider whose nearest RigExec ancestor is a joint
  still resolves against the joint, not the asset root.
- **C++** (`tests/testRigExecArm.cpp` and friends): a two-level chain
  under two different intervening Xforms, so a joint under `Xform_A` and
  its control under `Xform_B` solve correctly — the case that a
  cancel-out design would pass by accident and this one has to earn.
- **Imaging** (`tests/testRigExecImaging.cpp`): assert the guide lands
  at `frame * assetRootWorld` with no second application — the
  double-apply this design claims cannot happen.
- **Gizmo** (`tests/python/test_gizmo_math.py`): a drag on a joint under
  an intervening Xform maps to the avar it would have without one.
- **The warning is gone**: compiling
  `examples/spider_legs_assembly_ref.usda` emits none.

## 7. Risks

1. ~~**The §4 registration may not be permitted.**~~ Retired while
   writing this note, by reading `pxr/exec/execGeom/xformable.cpp`: it
   is not permitted, and the consume-it-instead variant is too narrow to
   use. Both are written up in §4 rather than deleted, because the next
   person will reach for them in the same order.
2. **Compensating rigs double-apply** (§5). Mitigated by the reworded
   diagnostic, not prevented — and deliberately not prevented by a
   version gate, which §5 rules out.
3. **The evaluator/exec divergence C accepts** (§4). Bounded today
   because every user-visible surface goes through the evaluator; it
   stops being bounded the moment something reads `computeRestFrame`
   through exec directly. The comments at both sites are the only thing
   that will make that audible.
4. **Invalidation.** The failure mode of getting this wrong is a stale
   pose that looks plausible, which is the kind that survives review.
   The test in §6 for "rotate the Xform, frames move" must run through
   an edit-then-re-evaluate cycle, not a fresh stage per case.
5. **`resetXformStack` on an intervening Xform.** `ComputeRelativeTransform`
   already reports it (`resetsBelowAsset`, `rigEvaluator.cpp:6575`) and
   the imaging snapshot already carries reset boundaries
   (`bridge.cpp:690-703`). The design must say what a reset between the
   asset root and a provider means; the honest answer is that it detaches
   the provider from the asset root, and that it should warn rather than
   compose.
