---
title: How operators fire
summary: The mental model and the evaluation order behind every UsdRig rig — who reads what, who writes what, and when.
order: 10
---

A UsdRig rig is not a pile of nodes with hidden wires. It is a piece of USD
namespace, and where a prim sits in that namespace is what decides when it
runs. Once you can read the hierarchy, you can predict the frame.

## The mental model

Everything lives under one `RigExecRoot`. That prim *is* the rig: the
partition, the namespace root that controls, joints, solvers and movers are
discovered beneath, and the unit that gets compiled and evaluated.

Inside it there are four jobs:

- **Controls carry the animation.** A [control](nodes/control.md) is the
  animator's handle. Keys go on its `avars:*` channels, composed over its
  `rest:space`. Nothing in the rig revises a control — its base frame *is* its
  posed frame — so everything downstream simply follows it.
- **Solvers and constraints pose joints.** An [FK chain](nodes/fk_chain.md),
  a [two-bone IK](nodes/two_bone_ik.md) or an
  [aim constraint](nodes/aim_constraint.md) reads controls and writes frames
  onto [joints](nodes/joint.md). A joint is where solved posing becomes
  readable data.
- **Weights say how much.** A [static weight](nodes/static_weight.md) is one
  scalar per moved point, painted once and held for the shot. Bound through a
  mover's `rigExec:weightObject`, it scales that mover's effect per point.
- **Movers move geometry.** A [matrix mover](nodes/matrix_mover.md) reads a
  provider's rest-to-posed delta and carries points by it, per point, under the
  bound weight.

A frame runs the pose work first and the geometry work after it, so a mover
never has to wait in the hierarchy for a solver — it only has to name which
version of the joint it wants.

## The order you can predict

The compiler walks the **final composed namespace of the whole rig**, and it
reverses sibling order: **the bottom sibling executes first, the top sibling
executes last**, and a parent runs after all of its descendants. This is
exactly the usdview stack presentation, read from the bottom up.

Two consequences are worth memorising:

1. **Sibling order comes only from the parent's child order.** Not from
   relationship-target order, not from plugin registration, not from which
   worker finishes first. If you want a different order, change the composed
   order — `reorder nameChildren = [...]` on the parent, or nest one prim under
   the other. Layer strength participates in that composed order exactly as a
   reorder opinion does.
2. **Solvers and constraints on the same joint are one stack, not two
   phases.** `rigExec:joints` is an ordered write, not an exclusive claim. Any
   number of solvers may name a joint and any number of pose constraints may
   name it on `rigExec:moves`; they are steps of one kind in one stack, in that
   same bottom-first order. A constraint **below** a solver runs first and
   *feeds* it; a constraint **above** it *revises* its output.

Because each step receives the frame the step below it left, a solver takes its
joints' **incoming** frames as its rest reference. Two-bone IK re-measures
root-to-mid and mid-to-end on every evaluation, so a step below it
*re-proportions* the limb rather than merely re-orienting it. An FK chain
instead composes its control deltas onto whatever incoming frame it finds. A
joint no earlier step wrote simply hands over its authored `rest:space`, which
is why a rig whose constraints all sit above its solvers is unchanged by any of
this. Put `Solvers` near the bottom of the rig root to get the classic
"solve, then revise" shape — that is what every shipped example authors.

Steps that share no joint and no data are independent: overlapping writes
serialise, disjoint chains stay parallel. Nothing about that changes the
answer, only how fast it arrives.

## Read phases: which version a mover sees

A mover names the moment it reads its provider with
`rigExec:transformReadPhase`:

- `base` — the joint **after its last solver**. Note that this is not "before
  every constraint": a constraint that fell *below* the last solver is already
  folded in, through that solver's rest reference.
- `preceding` — the value standing immediately before this one mover's own
  application in the point stack. It is only meaningful to a mover that has a
  place in that stack.
- `final` — the top of the chain, after every writer of that target.
- **a checkpoint** — an absolute prim path in place of a token, meaning "the
  provider as it stood right after that named step". Naming the `Solvers` scope
  means "after the last solver" (which is exactly `base`); naming the `Movers`
  scope means "after the last constraint".

```usda
def RigExecMatrixMover "HandSkin" (
    prepend apiSchemas = ["RigExecMoverAPI"]
)
{
    rel rigExec:moves = </Asset/Geom/Hand.points>
    rel rigExec:transform = </Asset/Rig/Joints/Shoulder/Elbow/Wrist>
    uniform token rigExec:transformReadPhase = "final"
}
```

Change that one token to `base` and the hand card follows the FK solve but
ignores the aim constraint stacked above it. Nothing else in the file moves.

## A frame, walked through

![Two-bone IK](gifs/two_bone_ik.gif)

`two_bone_ik.usda` is a two-card arm. Under the rig root, in composed order:
`Controls`, `Solvers`, `Joints`, `Weights`, `Movers`.

At frame 1001 the `HandIK` control sits at its rest, `x = 5.85`, just inside
full reach of the 3 + 3 unit chain, so the arm holds a slight bend. By frame
1006 its `avars:tx` spline has reached `-2.35` and `avars:ty` `1.5`.

1. **Controls are read.** `HandIK` composes its avars over `rest:space` and
   publishes a posed frame. `ElbowPole` and `ShoulderRoot` publish their rests
   unchanged — they are unanimated, and they are still real inputs.
2. **The solver fires.** `ArmIK` is the only step in the pose stack. It
   measures its two bones from the frames `Shoulder`, `Elbow` and `Wrist`
   carry on entry — here, their authored rests, because nothing sits below it —
   then solves the chain in the plane through the pole, with `inputs:softness`
   at `0.15` shaping the approach to full extension.
3. **Joints are written.** All three joints in `rigExec:joints` get new frames:
   the shoulder stays planted, the elbow swings out into the pole plane, the
   wrist lands on the effector.
4. **Movers read them.** `UpperSkin` reads `Shoulder` and `ForeSkin` reads
   `Elbow`, both at `final`, each scaled by a constant `1.0` static weight —
   the rigid-attachment idiom. Each four-point card is carried by its joint's
   rest-to-posed delta.
5. **The viewer sees** the orange upper card and the green forearm card hinge
   at the elbow as the hand swings in and lifts, then unwind by 1012.

## Common mistakes

- **"My constraint does nothing."** It is probably below the solver that
  overwrites the joint. Move its scope above the `Solvers` scope — i.e. earlier
  in composed order — or accept that it is now feeding the solve.
- **"My skinning ignores the constraint."** The mover is reading `base`. Set
  `rigExec:transformReadPhase = "final"`, which is what every shipped example
  does.
- **"Two movers on one mesh fight."** They do not fight; they stack, bottom
  sibling first, descendants before their parent. Reorder or renest to choose.
  Stacking matrix movers is how you layer rigid follows, not how you blend
  influences on one point — use the skin mover for that.
- **"I changed the relationship order and nothing happened."** Target-list
  permutation changes neither mapping nor result. Change the *namespace*.
- **"Editing a joint rest changed my limb proportions."** It is meant to: IK
  measures bone lengths from rests every evaluation and caches nothing.

## Glossary

- **Avar** — an animator-facing scalar parameter on a control, such as
  `avars:tx` or an IK blend.
- **Rest** — the bind-pose frame a prim carries in `rest:space`, measured
  against its namespace parent's rest.
- **Posed** — the evaluated frame for the current time.
- **Provider** — anything that publishes a frame other things can read: a
  control or a joint.
- **Aggregate** — one packed array of frames published by a solver, whose
  element *N* is written to `rigExec:joints[N]`.
- **Stack** — the ordered list of steps writing one target, be it a joint's
  solvers and constraints or a mesh's movers.
- **Phase** — which version of a provider a reader asks for: `base`,
  `preceding`, `final`, or a named checkpoint.

## Where this is specified

Traversal order, the one pose stack over solvers and constraints, rest
references, `base`/`preceding`/`final` and `AtPrim` checkpoints: spec §4.2
(*Mover targeting, hierarchy order, and last-writer semantics*). Phase
boundaries: §6.1. Bottom-up order and the other terms: §16.
