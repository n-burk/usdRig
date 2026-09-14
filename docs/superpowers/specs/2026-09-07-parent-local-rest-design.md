# Parent-local rest frames

Date: 2026-09-07. Status: approved, awaiting implementation.

## 0. Request

"If I adjust the rest positions of the shoulder joint (after
disconnecting from the ik solver) with the pivot/default mode in the
manipulator the child joints do not follow, I expect all rest positions
to be describing a local offset not a worldspace. Also the manipulator
should have a mode that only adjusts the currently selected joint(s)
and compensates any children joints."

Answered during brainstorming: the fix lands in the **data model**, not
in the manipulator; `rest:space` becomes **parent-relative** alongside
`rest:t`/`rest:r`; and the child-compensating mode is a **held
modifier**, like the existing snap keys, rather than a persistent
toggle.

## 1. The observed behaviour

Reproduced against `examples/components/spider_leg_ik.usd` with the IK
solver disconnected, through the `_rigexec` Python binding only -- no
manipulator in the loop. Bumping `Shoulder` by +3 in X:

| edit            | Shoulder | ankle  | foot   |
|-----------------|----------|--------|--------|
| `avars:tx += 3` | +3       | **+3** | **+3** |
| `default:tx += 3` | +3     | **+3** | **+3** |
| `rest:tx += 3`  | +3       | **0**  | **0**  |

Pose motion is hierarchical; rest motion is not. `foot`'s world origin
is `Shoulder + foot.rest` = `(6.8998, 0.6173)`, not `ankle + foot.rest`
-- every joint's rest is anchored to `rest:space`, which
`libs/rigExecSchema/schema.usda:243` documents as the "Local-to-world
bind transform" and which defaults to identity. So today `rest:tx/ty/tz`
are world-space offsets.

This contradicts the schema's own prose two hundred lines earlier
(`schema.usda:213`): a xformable "follows its namespace-parent
xformable's posed space with its local rest offset and avars applied".
The prose describes the design in this document; the code implements
something else.

## 2. Facts the design relies on

Each was read in the tree at `bc0f0e2` plus the working-tree changes.

1. **`computeRestFrame` reads only its own prim.**
   `libs/rigExec/computations.cpp:415-425` registers it with seven
   `AttributeValue` inputs -- `rest:space` and the six `rest:t`/`rest:r`
   scalars -- and no ancestor input of any kind. `_JointRestSpace`
   (`computations.cpp:230`) composes them as `local * space` and
   orthonormalizes. This single omission is the whole defect.

2. **The registration pattern for an ancestor input already exists in
   the same macro.** `computedDefaultSpace`, three lines below at
   `computations.cpp:428-432`, declares
   `NamespaceAncestor<RigExecPointFrame>(computeRestFrame).InputName(parentRestFrame)`.
   Nothing new has to be invented to express the dependency.

3. **`_ComputeDefaultSpace` already divides the parent rest out.**
   `computations.cpp:289-307` ends with

   ```cpp
   return offset * rest * parentRest.GetInverse() * parentDefault;
   ```

   That `parentRest.GetInverse()` exists only to convert an absolute
   child rest into a parent-relative one. Once `computeRestFrame`
   returns `restLocal * parentRest`, the expression collapses to
   `offset * restLocal * parentDefault` on its own. **This function is
   not edited.** The same cancellation carries the change through every
   other consumer written in terms of world rest frames.

4. **Implied IK lengths read world rest origins.**
   `libs/rigExec/rigEvaluator.cpp:6525-6560` evaluates `_restFrameTaps`
   and measures `rests[i].Origin()` distances. Those origins stay
   correct by fact 3, so bone-length measurement is unchanged.

5. **Invalidation follows the declared computation inputs.** The rest
   taps are `RigExecTapSet`s over `computeRestFrame` addresses; exec
   derives their dependencies from the registration, so adding the
   ancestor input propagates invalidation to descendants automatically.
   `_CollectPoseInputInfo` (`rigEvaluator.cpp:1196-1207`) already adds
   the immediate parent's rest attributes for `default:space`, and
   reaches further ancestors transitively through `parent:defaultSpace`
   -> the parent's `default:space`. `_CollectExternalReads`
   (`rigEvaluator.cpp:1144-1149`) is transitive for the same reason.
   **No invalidation code changes.**

6. **The gizmo mirrors the C++ rest composition in Python.**
   `plugin/rigExecUsdview/gizmoMath.py:429` is
   `rest = RestLocal(prim, time) * _MatrixAttr(prim, REST_SPACE, time)`,
   documented at line 16 as `orthonormalize(compose(rest:t, rest:r,
   XYZ) * rest:space)` and explicitly labelled a mirror of
   `_JointRestSpace`. It needs the mirrored change, and its drag
   inverse needs a compensating factor (section 4).

7. **Held drag modifiers are an established mechanism.**
   `plugin/rigExecUsdview/gizmoUI.py:2227` lists `_DRAG_KEYS` as
   Escape, `J`, `X`, `C`, `V`, with press handlers at 2369-2392 and
   release handlers at 2435-2452. `docs/viewport-gizmos.md:154-168`
   documents the table. `B` is free and sits next to `X`/`C`/`V` on the
   same keyboard row.

8. **Seven examples carry nested joints.** Five express them as
   absolute `rest:space` matrices (`01_FkChainTail`, `02_TwoBoneIkLeg`,
   `03_IkFkBlendClamp`, `05_TwistRibbonSpine`, `ArmRig`); two use
   `rest:tx/ty/tz` (`components/spider_leg.usd`,
   `components/spider_leg_ik.usd`). Verified absolute by evaluation:
   `01_FkChainTail`'s authored `(0,5,0) (2,5,0) (4,5,0) (6,5,0)` land
   at exactly those world origins with no accumulation. 46 call sites
   across `tests/` and `rigBuilder` set rest translation.

## 3. Semantics

A frame provider's rest frame becomes

```
restWorld = orthonormalize(compose(rest:t, rest:r, XYZ) * rest:space) * restWorld(parent)
```

where `parent` is the namespace frame provider, and the identity when
there is none. Row-vector convention throughout, matching the existing
`local * space` order.

`rest:space` is redefined from "Local-to-world bind transform" to the
bind basis relative to the parent frame provider's rest frame. A
provider with no RigExec ancestor is unaffected: its parent frame is
identity, so top-level rests keep their current meaning and values.

Orthonormalization stays where it is, on the local factor, before the
parent multiply. The parent's frame is itself already orthonormal, so
the product is orthonormal and the `Ir` contract holds.

### Code changes

- `computations.cpp:415-425`: add
  `NamespaceAncestor<RigExecPointFrame>(computeRestFrame).InputName(parentRestFrame)`
  to the `computeRestFrame` registration inside
  `RIGEXEC_REGISTER_XFORMABLE`.
- `computations.cpp:230` `_JointRestSpace`: read `parentRestFrame`, and
  return `rest * _SpaceFromFrame(parentRestFrame)` after
  orthonormalizing `rest`.
- `schema.usda:243`: reword the `rest:space` doc.

Nothing else in `libs/` changes.

## 4. Manipulator

### Default: children follow

A Pivot drag authors only the dragged joint's `rest:t`/`rest:r`, exactly
as today. Children follow because the data model now says they do. No
new authoring path.

`gizmoMath.py:429` gains the parent factor to mirror section 3. The drag
inverse -- solving `rest:t`/`rest:r` for a target world position --
gains `parentRest⁻¹`, because the value being solved for is now
parent-relative. This is the only non-trivial edit in the plugin.

### Held `B`: compensate children

While `B` is held for the duration of a drag, each **immediate** child
frame provider that is not itself in the selection is counter-authored
so its world rest is invariant:

```
restLocal(child)' = restLocal(child) * restWorld(parent) * restWorld(parent)'⁻¹
```

Immediate children suffice: deeper descendants ride on their parents,
which no longer move. The identity is a full matrix, so rotation is
compensated as well as translation. A child already in the selection is
skipped -- it is being moved deliberately.

`B` joins `_DRAG_KEYS` with press/release handlers beside the existing
held keys, and the status label reports it armed the way `X`/`C`/`V`
do. `docs/viewport-gizmos.md`'s hotkey table gains the row.

## 5. Migration

`tools/migrateRestToLocal.py` rewrites authored rest transforms
parents-first: for each frame provider with a frame-provider ancestor,
replace its composed rest `R` with `R * R_parent⁻¹`, decomposing back
into whichever of `rest:space` / `rest:t` / `rest:r` the prim actually
authored. Both spellings compose into one matrix, so one routine covers
the `rest:space` examples and the `rest:t` examples alike.

It stamps `rigExec:restFrameVersion = 2` on the rig root. Re-running is
then a no-op; without the stamp a second run silently double-compensates.
The evaluator warns once per rig that lacks the stamp, which is also
what tells someone their external asset needs migrating -- the
reinterpretation is otherwise silent and looks like a broken rig.

### Correctness

Migration is verified, not eyeballed: for every example, evaluate before
and after and assert world joint origins are identical to 1e-9. This is
the acceptance test for the whole change and runs over all seven
nested-joint examples.

## 6. Testing

Test-driven, in this order:

1. **Failing test first** -- a parent's `rest:tx += 3` moves its
   descendants by +3. Today it moves them by 0. This is the bug, and it
   is the first thing written.
2. **Migration round trip** -- world joint origins bit-identical across
   all seven examples before and after migration.
3. **Top-level providers unchanged** -- a provider with no RigExec
   ancestor keeps its current rest values and world frame.
4. **Implied IK lengths** -- `spider_leg_ik` still measures
   `upperLength=4.658311` / `lowerLength=3.583845` after migration,
   confirming fact 4's cancellation.
5. **Compensation** -- holding `B` holds an unselected child's world
   rest invariant under both translation and rotation; a selected child
   is not compensated.
6. **Re-baseline** the 46 existing rest-setting call sites.

## 7. Risks

- **Silent reinterpretation of external assets.** An unmigrated asset
  evaluates to a different pose with no error. Mitigated by the version
  stamp and the one-time warning (section 5), not eliminated.
- **`rest:space` carrying scale or shear.** Orthonormalization already
  discards it today; the parent multiply does not make this worse, but
  the migration's decomposition must not reintroduce it.
- **Deep chains.** Rest frames now cost a parent walk. They are cached
  per evaluation through the existing tap sets, so this is a constant
  factor on a path that already walks parents for `default:space`.
