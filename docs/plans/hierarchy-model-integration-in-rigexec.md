# Hierarchy Models (Premo/DreamWorks) → integration study for rigExec

Date: 2026-09-16
Status: investigation + design options. Companion to
`evaluation-engine-gaps-vs-premo-libee.md` (this doc deepens the §4.3 "Missing" row and
the P3 `HierarchyFrames` opportunity).
Evidence bases: Premo research brief (public sources only), this repo's code
(`file:line` anchors as read 2026-09-16), and two fresh investigations
(`/tmp/hiermodel-research.md`, `/tmp/jointflow-report.md`).

## 1. What a Hierarchy Model actually is (public evidence)

Source: Hutchinson, Kao, Ochs, Davoud, Powell — *Hierarchy Models: Building Blocks for
Procedural Rigging*, SIGGRAPH 2019 Talks ([PDF](https://research.dreamworks.com/wp-content/uploads/2019/10/talk_hierModel_stage2.pdf)).

Core idea: **stop making joints individually first-class scene objects.** A *Hierarchy
Model* is a compound data value — a joint hierarchy as an atomic entity that flows
through the dependency graph exactly like a mesh's vertices flow through deformation
nodes:

- **Components, not objects.** Joints are array elements of the bundle. Each carries a
  per-joint attribute set — parent, rotation order, scale propagation, flipping
  behavior, color, plus base transform data (non-exhaustive per the talk).
- **Ops act on the whole bundle.** Each DG node takes a hierarchy in, emits a distinct,
  *inspectable* hierarchy state out: transform/set ops; constraint ops that modify one
  or more joints in a single node; topological ops (add / remove / re-parent); a
  reverse-hierarchy op that pivots an arbitrary joint and modifies the whole bundle
  including itself.
- **Propagation is a declared attribute, not scene hierarchy.** Scale propagation and
  transient per-operation propagation state ("curl/spin a chain in place") live on the
  components; there is no DCC DAG carrying them.
- **The IK/FK worked example.** Classic first-class joints force duplicated, slightly
  different hierarchies (IK path, FK path, local-space blend, secondary offsets) whose
  rotation orders must stay synced by hand; with bundles the graph is a *flowchart of
  intent* — one hierarchy value, ops layered, every intermediate state inspectable.

Claimed benefits (talk): fewer nodes/connections → lower graph complexity and better
DG parallelism; joints become "points" → constraint-to-surface work reuses SIMD/parallel
*geometry* libraries; downstream systems consume one output, not a joint list; crowd/mocap
(FBX) drives the character and any section is intercepted by swapping **one** bundle
connection. USD is named there as promising *interchange* — and serialization, attribute
type system, SIMD packing, and the inheritance math are explicit public gaps.
Patent adjacency: US10,297,064 multi-rep (Papp/Bryson) is about *edit-vs-eval*
representations — a different axis; whether HM ops are first-class LibEE 2 authoring
types is not public.

**The single most important reading for us:** HM is a *value type + op vocabulary*, not
an engine. It sits on top of LibEE's normal node rules. rigExec already has LibEE-style
execution machinery; the missing piece is the type and the ops — which makes this an
integrable increment, not a rewrite.

## 2. Where rigExec stands today (the honest starting picture)

Anchored facts (code read 2026-09-16; full inventory in `/tmp/jointflow-report.md`):

- Joints are individual `RigExecJoint` prims with their own authored transform
  (UsdGeomXform) and avar-style channels; authoring handles in
  `libs/rigExecRigging/rigBuilder.h:153-165`.
- `rigExec:joints` is a `rel` naming the **ordered output joints of one aggregate
  solver** (`generatedSchema.usda:591`; per-solver resolution at
  `rigEvaluator.cpp:2119-2123`; documented at `rigBuilder.h:167-169` — "list position is
  the element index"). It is the *one* place rigExec already thinks bundle-shaped.
- Aggregate solver computations publish `computePointFrameArray` aggregates — a
  whole-array joint-frame payload — already (`computations.cpp` header, per-schema
  blocks at :699/:831/:945/:1003/:1190). `BlendPointFrames` and `TwistDistribution`
  already consume whole chains.
- The baked program is a dense-slot design: joint frames are per-slot *versions*
  (PoseFin-style domains, `bakedProgramImpl.h`), and some baked steps already read
  **every** provider's final frame as one batch (`VolumePlacements` "reads the final
  frame of every volume weight provider", `docs/baked-step-graph.md:51-52`).
- The published `RigExecRigPose` hands out per-joint frame maps; consumers (skin
  influences, gizmos, control guides, `--joints-out`, imaging) all do per-joint lookups.

So: rigExec is *per-prim at authoring, array-shaped inside solvers, per-slot at bake,
per-joint on the way out*. HM's proposition is to promote the middle (array-shaped)
representation to a first-class value at the seams too.
