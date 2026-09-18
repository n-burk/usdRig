# Gap catalog: usdRig vs the ZebraSample Unreal character rigs

Companion to [`docs/plans/ue-zebrasample-rig-gaps.md`](../../docs/plans/ue-zebrasample-rig-gaps.md), which explains the method, the verdict and severity scales, and the cross-cutting findings. This file lists every gap row in full, grouped as in that report's section 5 and ordered by severity.

Path conventions: repository paths are relative to the repo root; `<dump>/` is the output of [`tools/ueDumpRigs.py`](../../tools/ueDumpRigs.py) run on the ZebraSample project (one directory per asset, e.g. `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/graphs.txt`); `<UE>/` is the UE 5.8 engine install; `<ZebraSample>/` is the project; `<usd-install>/` is the sibling OpenUSD install (`../usd-install`). `libs/rigExec/moverGraph.cpp` line numbers include an uncommitted working-tree edit (+25 lines after it). UE feature ids (`UE1-…`…`UE8-…`) refer to [`ue-rig-analysis.md`](ue-rig-analysis.md). Line numbers were read on 2026-09-16.

Errata applied to the analysts' text: MR_Zebra has **34** module instances and MR_FN_Biped **48** (counted from `modular_rig_model.txt` and the instance sub-objects in `asset.t3d`); where a sentence still says 36 or 50, read 34 or 48.

Direction set after this catalogue was written: the ordering-related rows below (for example `G6-deform-exec-order`, `G2-postprocess-pass`, `G1-face-forward-order`, `G4-space-dependency-cycles`) propose a third pose-step kind, late property chains or a separate `PostPose` phase. The report's section 6.1 supersedes those recommendations. The spec (`docs/spec.md` 4.2) already defines a single hierarchy-ordered mover stack with phase-bound reads, IK included, and the fix is to implement that stack. Probe `reports/ue-zebrasample/probes/order/` shows the current behaviour.

Each row was produced by a group analyst reading the usdRig source, then reviewed by an independent adversarial verifier. **Verification** records that review: where the verifier changed the verdict or severity, the row shows the corrected value and the original. Rows marked *added by verifier* were missed by the analyst.

## G1 — Modular rig system

The UE Modular Rig system is almost entirely build-time scaffolding. The ModularRig host graph is empty, and every Zebra/Monster element is spawned by per-module Construction events from connectors and config. usdRig has no module, connector, rule, binding or construction concept: it evaluates whatever composed USD sits under a RigExecRoot, discovered by type with no membership lists, and it deliberately forbids creating topology during evaluation (spec.md:58). The modular layer therefore has to become offline authoring: a Python module-generator library plus a UE importer that resolves connectors, config values (read from asset.t3d, because ConfigOverrides dump as empty), bindings and module metadata into concrete prims, relationships and connections. USD composition already supplies most of the reuse mechanics: references to component layers, stronger-layer value overrides, variants (for DMC or neck mode), deactivation, relocates, and reference-based L/R mirroring as in Biped_layered. What is missing is a declared module/connector interface. The cheapest high-value engine change is to read relationships through GetForwardedTargets, so a referenced module can route its internal wiring through one connector relationship; a probe confirmed that USD remaps such forwarding through references and drops targets outside the referenced subtree. A few gaps affect runtime behaviour. Cross-module request flags (Match IK/FK, Key Controls) have no event or matching model. Scalars derived from solved poses cannot be interleaved with constraints (face step F->G), because property chains run before exec and pose interpolators run after the pose walk. Hidden per-frame state (the Jaw Normalize lag, pivot-change compensation) is rejected by design. Attaching the Boombox to the Zebra across rigs needs the multipass from spec §6.2, which is not built. usdRig's strict compile rules also constrain the importer. A missing source or target, or an empty source list, fails the whole rig; USD de-duplicates relationship targets; paths are case-sensitive. The importer must therefore prune dead UE entries and expand paired arrays. Mirroring metadata on controls is explicitly deferred, and the biped split/mirror tool is not in the repo.

| Row | Verdict | Severity | Effort | Summary |
|---|---|---|---|---|
| [`G1-face-forward-order`](#g1-face-forward-order) | Partial | major | L | Pose-derived scalars cannot gate later constraints: FloatMathMover runs before exec, pose interpolators after the walk |
| [`G1-ikfk-module-config`](#g1-ikfk-module-config) | Partial | major | L | TwoBoneIk lacks PV Twist Follow, IK End Align, segment scale; Default IK/FK space map to weight/sourceWeights defaults |
| [`G1-metadata-bus-runtime-flags`](#g1-metadata-bus-runtime-flags) | Partial | major | L | IK Solve maps to float envelope or bool inputs:enabled; no event requests for Match IK/FK, Key Controls, IsInteracting |
| [`G1-movable-pivot-state`](#g1-movable-pivot-state) | Divergent-by-design | major | M | No hidden state or interaction events; Spine/Body movable pivots need animatable pivot-offset controls, not rest edits |
| [`G1-stateful-module-variables`](#g1-stateful-module-variables) | Divergent-by-design | major | M | No state channels (hidden state rejected): Jaw lag, Root snap, Prop pivot compensation, Body aim buffers unreproducible |
| [`G1-candidate-matching`](#g1-candidate-matching) | Missing | minor | M | No connector candidate filtering or auto-resolve; arc panel is module-agnostic; Fortnite rigs disable auto-resolve |
| [`G1-connection-rules`](#g1-connection-rules) | Missing | minor | M | No connection rule model or element tags; only compile checks (Xformable source, target inside rig asset) |
| [`G1-control-mirror-metadata`](#g1-control-mirror-metadata) | Missing | minor | M | No mirror axis, behavior or counterpart on RigExecControlAPI (deferred, reflectionAxis unbuilt); no pose mirror tool |
| [`G1-array-connectors`](#g1-array-connectors) | Partial | minor | S | Ordered multi-target rels exist, but USD dedupes duplicates, missing targets fail compile, index pairs need expanding |
| [`G1-boombox-prop-rig`](#g1-boombox-prop-rig) | Partial | minor | L | Prop maps to RigExecRoot, controls, ParentConstraints; no socket or cross-rig attach (partition inert, no multipass) |
| [`G1-config-overrides`](#g1-config-overrides) | Partial | minor | M | Eval-read config becomes per-instance attr overrides; no module param surface or defaults; build-only config not in USD |
| [`G1-connector-as-parent`](#g1-connector-as-parent) | Partial | minor | M | Nesting under a solver-posed joint follows it (probe); plain Xform nulls there are stale; no single 'attach' helper |
| [`G1-connectors`](#g1-connectors) | Partial | minor | L | No connector interface (optional, array, is-connected); importer writes targets into operator rels; no rel forwarding |
| [`G1-dmc-shape-layers`](#g1-dmc-shape-layers) | Partial | minor | M | Closest is RigExecTouchRegions face sets: one layer only, no IK/FK layer switching, no polygroup generator, not a gizmo |
| [`G1-element-metadata-channel`](#g1-element-metadata-channel) | Partial | minor | S | No typed element metadata or provenance; use custom attrs or customData, bake consumed values into operator inputs |
| [`G1-face-member-tables`](#g1-face-member-tables) | Partial | minor | M | No table inputs; keys become static rels, rows spread into ParentConstraint offsets, TwistDistribution, RigExecPose |
| [`G1-face-module-shell`](#g1-face-module-shell) | Partial | minor | S | Parent=head maps to nesting the face root under the head joint (follows the solved head) or a ParentConstraint |
| [`G1-foot-module-config`](#g1-foot-module-config) | Partial | minor | M | Reverse foot is an authored pivot stack under leg IK with FloatMathMover chains; no foot generator or half-circle shape |
| [`G1-metadata-bus-static`](#g1-metadata-bus-static) | Partial | minor | S | Published keys become rels or nesting (foot pivots under leg_l_ik), scalars via connections; no Self/Parent/Root lookup |
| [`G1-mirror-module`](#g1-mirror-module) | Partial | minor | M | Mirroring via references plus overrides (Biped_layered_right); no mirror operator; tools/biped/split_layers.py missing |
| [`G1-module-instance-model`](#g1-module-instance-model) | Partial | minor | L | No module type, parent or connectors; modules are referenced component prims; external rels re-authored per instance |
| [`G1-monster-deformers-in-module`](#g1-monster-deformers-in-module) | Partial | minor | M | Placement ports natively; bend/twist/squash kernels are missing and double avars cannot feed float FloatMathMover remaps |
| [`G1-optional-connectors`](#g1-optional-connectors) | Partial | minor | S | Empty rigExec:sources is a hard compile error even when disabled; generator must omit the operator or set active=false |
| [`G1-template-lineage`](#g1-template-lineage) | Partial | minor | L | Lineage via references, overrides, active=false, relocates; no module-aware diff tool; external rels re-authored per rig |
| [`G1-variable-bindings`](#g1-variable-bindings) | Partial | minor | S | Bindings only as single same-typed attr connections (no coercion); no binding list; guide scale ignores connections |
| [`G1-construction-offline`](#g1-construction-offline) | Divergent-by-design | minor | XL | No runtime spawning; all 14 module construction graphs must become offline rigexec.schema generators, with no provenance |
| [`G1-module-exec-order`](#g1-module-exec-order) | Divergent-by-design | minor | S | No module queue: reversed pre-order Movers walk plus pose DAG; child scopes precede parent movers; needs reorder pins |
| [`G1-structural-config-switch`](#g1-structural-config-switch) | Divergent-by-design | minor | M | Is Neck becomes a generator param or Spine variantSet; SplineIk already does neck mode, spine mode via ParentConstraint |
| [`G1-authoring-quirks`](#g1-authoring-quirks) | Not-applicable | minor | S | Porting policy: missing prims are hard compile errors, so importer strips them; FloatMathMover blend gives last-writer |
| [`G1-connection-list-hygiene`](#g1-connection-list-hygiene) | Not-applicable | minor | S | UE bookkeeping; importer must honor case-sensitive SdfPath/TfToken and drop dead entries (dangling targets fail compile) |
| [`G1-connector-default-match`](#g1-connector-default-match) | Missing | cosmetic | S | No authoring-time default-target suggestion hook; belongs in an offline module library, UX only with no eval impact |
| [`G1-biped-template-extras`](#g1-biped-template-extras) | Partial | cosmetic | M | Pins and Attach map to ParentConstraints; no debug-line stretch feedback or proxy delta controls (Zebra/Monster unused) |
| [`G1-dmc-template-variant`](#g1-dmc-template-variant) | Partial | cosmetic | S | Extra module plus host toggle maps to a variantSet as rigComplexity does; no shape libraries or search order (cosmetic) |
| [`G1-module-identity`](#g1-module-identity) | Partial | cosmetic | S | No module identity, category or icon fields and no browser; assetInfo, doc or customData on a component can hold it |
| [`G1-module-namespace-naming`](#g1-module-namespace-naming) | Partial | cosmetic | S | Prim path hierarchy namespaces modules; UE names with spaces or '/' need sanitizing; no RigExec panel reads displayName |
| [`G1-negative-side-detect`](#g1-negative-side-detect) | Partial | cosmetic | S | Side flags unneeded (TwoBoneIk solves from positions); only guide mirroring lost: non-positive guide:scale draws nothing |
| [`G1-rig-wide-settings`](#g1-rig-wide-settings) | Partial | cosmetic | S | No rig-wide guide scale (guide:scaleX/Y/Z ignore connections) or side palette; colors via per-control guide:displayColor |
| [`G1-asset-settings-limits`](#g1-asset-settings-limits) | Not-applicable | cosmetic | S | Nothing spawns so no element cap; multiple instances are independent referenced RigExecRoots; prototype rigs rejected |
| [`G1-module-asset-kinds`](#g1-module-asset-kinds) | Not-applicable | cosmetic | S | UE class and asset plumbing; in usdRig every module is the same kind of thing, a component layer referenced by a prim |
| [`G1-supported-events`](#g1-supported-events) | Not-applicable | cosmetic | S | No event model or registry tags; bake-to-controls maps to generic Python LM inverse (solve_parameters), not per module |

### G1-face-forward-order

**Face forward-solve ordering: pose-derived scalar logic interleaved with transform steps**

**Verdict:** Partial · **Severity:** major · **Effort:** L · **Confidence:** high · **Domain:** D16

UE features: `UE6-forward-order`

**UE rigs.** The face Forwards Solve first runs the head attach, the control-to-bone SetTransforms and the visibility loops. An aggregate Sequence A..K then runs (Monster adds L): A brow, corner and lip curves plus helper constraints; B lip constraints; C rolls; D Correctives; E re-parenting; F the jaw pose reader and Jaw Open Logic; G lip tweakers; H lid blink, extend, open and rotate; I soft eyes; J lid skin; K squash curves, pupil and eye aim. Scalar logic (curves) and transform writes alternate, and later transform steps use scalars computed from earlier solved poses (for example, the jaw reader feeds the lip offsets). As a result, Correctives see the previous frame's Jaw Normalize, the eye bones lag the aim by one evaluation, and pure library functions run as data dependencies. bResetInputPoseToInitial keeps additive stacks from accumulating.

**usdRig today.** Transform steps map to constraints ordered by the Movers namespace, with solvers interleaved by the pose DAG. Additive stacks never accumulate, because every evaluation runs from base to final. Scalar logic, however, is confined to two fixed phases. Property chains (FloatMathMover) run before exec and cannot read solved poses. Pose interpolators run after the whole pose walk and feed only geometry. So a scalar derived from a pose (the jaw reader, or curves driven by pose readers) cannot feed a later constraint's envelope or offset in the same evaluation. The UE one-frame lags cannot be reproduced either.

**Gap.** There are no pose-phase scalar operators interleaved with constraints, so scalar logic that reads solved transforms and feeds later transform operators cannot be ordered correctly.

**Porting impact.** Face step F->G (Jaw Open Logic driving the lip offsets), the corner and lip-height logic, and any constraint weight driven by a pose reader cannot run in one evaluation. The face either loses that coupling or needs restructuring. This is the ordering side of the G6 pose-reader gap.

**Recommendation.** Make property movers schedulable inside the pose DAG. A FloatMathMover (or a new RigExecPoseReader output) whose inputs read a provider's final frame becomes a pose step placed by dependency: after the writers of the frames it reads, and before the constraints whose inputs:defaultWeight or offset it revises. This touches the pose-DAG build and constraint walk in libs/rigExec/rigEvaluator.cpp (5794-6039), _EvaluatePropertyChains (split pre-exec chains from pose-phase chains), and the baked path in libs/rigExec/bakedPose.cpp and bakedSchedule.cpp. The no-hidden-state rule stays, so the ported face runs without lag.

**Evidence:** `docs/dead-surface-removal.md:198-208`; `libs/rigExec/rigEvaluator.cpp:11667-11675`; `docs/spec.md:331`; `docs/spec.md:1050`

**Verification (holds).** Confirmed on both sides:
- Property chains resolve before exec (dead-surface-removal.md:198-208).
- Pose interpolators run only after the full pose walk (rigEvaluator.cpp:11667-11675).

I tried the one remaining route, a pose-placed volume weight used as a constraint envelope. Volume placements are refreshed after every commit (rigEvaluator.h:1190-1200), but volumetric fields are refused on transform-domain constraints (testRigExecVolumeWeights.cpp:1198-1206; examples/14_VolumeConstrainedSweep.usda:4-12). DynamicWeight's inputs:driver is a plain float (schema.usda:1284-1301).

So no pose-derived scalar can gate a later transform operator in the same evaluation. Major stands.

Verifier evidence: `docs/dead-surface-removal.md:198-208`; `libs/rigExec/rigEvaluator.cpp:11667-11675`; `libs/rigExec/rigEvaluator.h:1190-1200`; `tests/testRigExecVolumeWeights.cpp:1198-1206`; `libs/rigExecSchema/schema.usda:1284-1301`

### G1-ikfk-module-config

**IkFk2Bones module option surface (about 25 public options)**

**Verdict:** Partial · **Severity:** major · **Effort:** L · **Confidence:** medium · **Domain:** D16

UE features: `UE3-ikfk-config`

**UE rigs.** IkFk2Bones exposes about 25 options: Control Scale, Color (white means use the side color), IK Rotation Offset, PV Distance Scale, PV Twist Follow, IK End Align, Default IK, IK Compensate World Orient, FK Rotation Offset, Rotation Order (bound to a host variable), FK Mirror Behavior, Use Scale, Segment Scale Control, Primary and Secondary Axis, IK FK Auto Matching, Debug, End Bone Rotation Offset, display names, Default FK Space Index, shape structs and an FK scale profile curve. Per-limb overrides differ between Biped and Zebra: legs set PV Twist Follow 1 and IK End Align False; Zebra sets PV Distance 1.25 on legs and 2.0 on arms; arms set Default IK False.

**usdRig today.** Options used only at construction (offsets, PV distance, shapes, names, colors) become generated rest frames and guide attributes. Runtime options with a counterpart: Default IK maps to the authored default of the BlendPointFrames weight dial, Rotation Order to avars:rotationOrder, and stretch and softness to TwoBoneIk inputs. Runtime options without a counterpart: PV Twist Follow, IK End Align, IK Compensate World Orient, Use Scale and Segment Scale Control, Primary/Secondary Axis, Auto Matching, Default FK Space Index (usdRig has no space enum) and Debug. TwoBoneIk offers only length offsets, preferredBendRadians, stretch and softness.

**Gap.** The runtime IK options that change visible behavior are missing (detailed in the G5 IK rows and the G4 space rows).

**Porting impact.** Leg and arm behavior differs wherever PV Twist Follow, IK End Align or segment scale matter. Construction-time options can be baked by the generator.

**Recommendation.** Add the runtime options as TwoBoneIk inputs in schema.usda and libs/rigExecMath/solvers.cpp, as specified in the G5 rows: for example float inputs:poleTwistFollow, bool inputs:endAlign or rel rigExec:endAlignControl, and token rigExec:segmentScale. The IkFk2Bones generator maps each UE option to a rest bake, an attribute, or an 'unsupported' warning.

**Evidence:** `libs/rigExecSchema/schema.usda:524-597`; `libs/rigExecSchema/schema.usda:599-650`; `libs/rigExecSchema/schema.usda:334-336`; `docs/biped-rig.md:190-193`

**Verification (holds).** The UE runtime options are animator-visible channels that construction spawns:
- 'PV Twist Follow' float channel (IkFk2Bones graphs.txt:729).
- 'IK End Align' bool channel (graphs.txt:780).
- Upper and Lower Segment Scale channels (graphs.txt:922-923).

TwoBoneIk exposes only length offsets, stretch, softness and preferredBendRadians (schema.usda:548-576), so major stands.

One correction: 'Default FK Space Index' does have a counterpart. It is the authored default of inputs:sourceWeights on the FK space ParentConstraint (schema.usda:1115-1142).

Verifier evidence: `libs/rigExecSchema/schema.usda:548-576`; `libs/rigExecSchema/schema.usda:1115-1142`; `ue/<dump>/FortniteRigs__Modules__IkSolves__IkFk2Bones__CRM_FN_IkFk2Bones/graphs.txt:729,780,922-923`

### G1-metadata-bus-runtime-flags

**Runtime cross-module request flags (IK Solve, Match IK/FK, Key Controls, IsInteracting)**

**Verdict:** Partial · **Severity:** major · **Effort:** L · **Confidence:** medium · **Domain:** D16

UE features: `UE3-module-metadata-bus`, `UE1-module-metadata-bus`

**UE rigs.** At runtime the Leg writes bools in its Self namespace, and the Foot reads them each frame through its Parent namespace. 'IK Solve' (written in Pre Forwards) decides whether the foot drives the IK effector. 'Match FK'/'Match IK' (set inside the match functions and the 'To IK'/'To FK' events) and 'Key Controls' make the foot follow match and keying requests. 'IsInteracting' and 'Sec Controls Visibilty' are also published. Post Forwards clears the Match and Key flags.

**usdRig today.** The continuous part works: the biped connects its foot constraints' inputs:defaultWeight to the leg's IK/FK blend weight, which is 'IK Solve' expressed as a float envelope. A bool cannot be connected to inputs:enabled, because that input is read without following connections. The request part (match, key and interaction flags raised by events and cleared the next frame) has no counterpart: there is no event layer, no IK/FK matching and no interaction state in the evaluator.

**Gap.** There are no event-driven requests between modules (match IK/FK, key controls, interaction state).

**Porting impact.** Foot IK/FK matching and the 'key the foot too' behaviour are lost; animators must match and key the foot controls by hand.

**Recommendation.** Implement matching as tooling rather than rig state. Add python/rigexec/matching.py, which computes FK<->IK snaps from evaluated frames (Rig/Pose API) using per-module match recipes declared on RigExecModuleAPI (for example rel module:match:fkControls, ikControls, footControls). Add a usdview action in plugin/rigExecUsdview that keys the result. The Foot's recipe lists its controls as dependents of the Leg's recipe, which replaces the Match and Key flags.

**Evidence:** `examples/biped/Biped.usda:3055`; `libs/rigExec/rigEvaluator.cpp:7686-7695`; `docs/spec.md:207`; `docs/biped-rig.md:190-193`; `docs/python-bake-inverse.md:71-74`

**Verification (holds).** One mapping claim is wrong: a bool can be connected to a constraint's inputs:enabled. It is validated at rigEvaluator.cpp:3631-3648 and resolved through connections at 10990-10991. 'IK Solve' can therefore be a bool dial connected to inputs:enabled as well as a float envelope (Biped.usda:3055).

The request half (Match IK/FK and Key Controls raised by events and cleared in Post Forwards) has no counterpart:
- The event layer is deferred (spec.md:207).
- IK/FK matching is not built (biped-rig.md:190-193).
- The numeric inverse leaves matching and committing to the caller (python-bake-inverse.md:71-74).

Major stands: matching and keying the foot with the leg are animator workflows.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:3631-3648`; `libs/rigExec/rigEvaluator.cpp:10990-10991`; `examples/biped/Biped.usda:3055`; `docs/spec.md:207`; `docs/biped-rig.md:190-193`; `docs/python-bake-inverse.md:71-74`

### G1-movable-pivot-state

**Interaction-gated movable pivots whose state persists in element metadata (Spine / Body)**

**Verdict:** Divergent-by-design · **Severity:** major · **Effort:** M · **Confidence:** medium · **Domain:** D16 · *added by verifier*

UE features: `UE2-spine-neck-mode`, `UE2-module-config-variables`, `UE3-module-metadata-bus`

**UE rigs.** Spine (non-neck mode) and Body each build a movable pivot:
- a proxy control: 'Spine/End Movable Pivot' (display name 'Chest Moveable Pivot') and 'Body/Body Movable Pivot';
- a buffer null ('... Bfr') carrying element metadata 'IsSet';
- a 'Movable Pivot Vis' bool channel.

Each evaluation, CRFL_Hierarchy 'Forward Movable Proxy v01' reads RigUnit_IsInteracting:
- While the proxy is being translated, it copies the driven control's global transform into the buffer once and sets IsSet=true.
- While it is being rotated, it rewrites the buffer.
- When there is no interaction, it resets IsSet=false, snaps the pivot null back to 'Proxy Snap To' and zeroes the proxy rotation.

The result depends on interaction state and on values persisted from earlier evaluations. Both proxies exist in MR_Zebra.

**usdRig today.** usdRig rejects hidden previous-evaluation state (spec.md:59, :1050, :2285), and the evaluator has no interaction state because the event and direct-manipulation layer is deferred (spec.md:207).

A movable pivot therefore has to be a pure function of authored avars: an animatable pivot-offset control, with the driven control rotating about it through nesting or a ParentConstraint. The gizmo's pivot mode edits rest:t/r, which is a rig edit, not an animatable pivot. G1-stateful-module-variables covers Root, Body aim, Prop and Face state, but not these proxies.

**Gap.** No stateless animatable-pivot pattern exists, and no interaction-time compensation for the Spine and Body movable pivots.

**Porting impact.** The chest and body movable-pivot controls cannot behave as in UE (rotate about a freely dragged pivot that snaps back on release). Animators lose that pivot workflow unless tooling replaces it.

**Recommendation.** Author the pivot statelessly: pivot control, then a pivot null, then an inverse-pivot null wrapping the driven control, with the pivot offset as avars.

Implement drag compensation and snap-back as a gizmo transaction in plugin/rigExecUsdview/gizmoDrag.py that writes compensating avars and keys through rigExecUndo.py. This is the same mechanism proposed for the Prop change-pivot in G1-stateful-module-variables.

Dedupe with the G3 proxy-control and interaction rows.

**Evidence:** `docs/spec.md:59`; `docs/spec.md:1050`; `docs/spec.md:2285`; `docs/spec.md:207`

### G1-stateful-module-variables

**Module member variables that carry state between evaluations**

**Verdict:** Divergent-by-design · **Severity:** major · **Effort:** M · **Confidence:** medium · **Domain:** D16

UE features: `UE2-module-config-variables`, `UE6-member-config-tables`

**UE rigs.** Several private variables persist across evaluations and feed the solve. Root keeps 'Global Control Snapped' and 'Global Control Transform' for a first-frame snap in Backwards. Body keeps aim buffers. Prop keeps pivot-previous and previous-buffer transforms for change-pivot compensation. The face keeps a 'Jaw Normalize' double that Correctives read from the previous evaluation. Several of these are gated on IsInteracting or RequestAutoKey.

**usdRig today.** usdRig rejects hidden state. Callbacks are pure, previous-frame continuity is rejected, and every evaluation is a static single assignment from base to final over authored values. State would only be allowed as an explicit declared input, and no schema provides one today.

**Gap.** There are no state channels. As authored, the Jaw Normalize lag, the Root first-frame Global snap, Prop change-pivot compensation and the Body aim buffers cannot be reproduced.

**Porting impact.** Face correctives must read the jaw reader within the same evaluation, which removes the one-frame lag: usually better, but a parity difference. Change-pivot compensation on Prop and Body cannot work, so those controls jump when the pivot changes. The Root bake snap has to move into offline bake tooling.

**Recommendation.** Keep the evaluator stateless. Port the lagged face reads as same-evaluation dependencies (see G1-face-forward-order). Implement pivot-change compensation as a tool-side transaction in plugin/rigExecUsdview: on gizmo drag commit (next to gizmoDrag.py), write compensating avars and keys. Put the Root first-frame snap in python/rigexec/bake.py. Only if parity is required, add an explicit declared state input (a RigExecStateAPI attribute that a host-side cache fills between frames), as spec.md:59 allows.

**Evidence:** `docs/spec.md:59`; `docs/spec.md:1050`; `docs/spec.md:2285`; `docs/spec.md:331`

**Verification (holds).** The non-goals exist:
- spec.md:59: no hidden stateful simulation; state only as an explicit input.
- spec.md:1050: hidden previous-frame continuity rejected; future state must be a declared input.
- spec.md:2285: hidden previous-frame state prohibited.
- spec.md:331: static single assignment.

The UE side checks out. CRM_FN_Prop carries 'Prop Global/Local Pivot Previous' and 'Previous Global/Local/Prop Buffer Transform' across evaluations and sends RequestAutoKey (Prop graphs.txt:14-26, 99, 109-123, 262-281). Major holds: pivot and aim toggles visibly pop without compensation.

Omission: the row does not list the Spine and Body movable-pivot proxies. Their 'IsSet' element metadata and buffer nulls persist between evaluations and are gated by IsInteracting (CRFL_Hierarchy graphs.txt:1084-1198; MR_Zebra runtime_hierarchy.txt:1944-1946, 2038-2040). Added as a missed gap.

Verifier evidence: `docs/spec.md:59`; `docs/spec.md:1050`; `docs/spec.md:2285`; `docs/spec.md:331`; `ue/<dump>/FortniteRigs__Modules__FkSolves__Prop__CRM_FN_Prop/graphs.txt:14-26,109-123,262-281`

### G1-candidate-matching

**Connector candidate filtering and auto-resolve**

**Verdict:** Missing · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D16

UE features: `UE1-rule-manager-matching`, `UE1-auto-resolve`, `UE8-asset-variant-validator-limits`

**UE rigs.** FindMatches starts from every hierarchy element and removes: curves and connectors; elements in the module's own namespace; elements spawned at or after the module's construction index (only earlier modules and imported elements remain eligible); targets that fail the rules; and targets pruned by the module's Connector event, which can also promote a default. bAutoResolve (engine default true) connects a secondary automatically when it has a single match or a default match. All the Fortnite-based rigs set bAutoResolve=False.

**usdRig today.** usdRig has no assembly tooling. The Layer Opinions / composition-arc panel authors arcs generically and knows nothing about modules or connectors.

**Gap.** There is no candidate filtering, no 'earlier modules only' constraint and no auto-connect.

**Porting impact.** No runtime impact, because the rigs store explicit connections. Re-assembling or extending a ported rig, for example adding a finger module, means wiring paths by hand.

**Recommendation.** Add python/rigexec/modules/matching.py with find_matches(stage, module, connector) and auto_connect(module). find_matches runs the UE pipeline over the composed stage: it excludes the module's own subtree and modules later in rigExec:module:parent order, applies the RigExecConnectorAPI rules, and calls the generator's suggest hook. Add plugin/rigExecUsdview/moduleAssemblyModel.py and moduleAssemblyUI.py (the same headless/Qt split as layerOpinionsModel/UI). Store a per-rig autoResolve preference as customData on the RigExecRoot.

**Evidence:** `docs/composition-arcs.md:1-30`; `plugin/rigExecUsdview/compositionArcsModel.py:1-25`; `libs/rigExecRigging/rigBuilder.h:1030-1189`

**Verification (holds).** Nothing already provides this under another name:
- The composition-arc tooling is generic (compositionArcsModel.py:1-30; composition-arcs.md:1-30).
- usdNoodles' Blueprint and Container prims (primAuthoring.py:29, 274-290; nodeGraphStage.py:58) are node-editor groupings with no connector or candidate semantics.

All Fortnite rigs set bAutoResolve=False, so there is no runtime impact. Minor is right.

Verifier evidence: `plugin/rigExecUsdview/compositionArcsModel.py:1-30`; `docs/composition-arcs.md:1-30`; `plugin/usdNoodles/primAuthoring.py:274-290`

### G1-connection-rules

**Connection rules (Type, Tag, ChildOfPrimary, And, Or, ArraySize)**

**Verdict:** Missing · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D16

UE features: `UE1-connection-rules`

**UE rigs.** Rule semantics: Type accepts a target when IsTypeOf matches. Tag requires HasTag. ChildOfPrimary requires the target to sit under the primary's target without being that target, substituting a socket's parent for the socket. And returns the first invalid result; Or returns the first valid result, otherwise the last. ArraySize only bounds the number of targets. Zebra uses ChildOfPrimary on 32 connectors. Boombox uses Type=Socket and Or(Null|Control|Bone). Tag, And and ArraySize are unused.

**usdRig today.** Nothing validates a wiring choice beyond operator-level compile checks: a constraint source must be a RigExec provider or a UsdGeomXformable, and a mover target must exist inside the rig asset.

**Gap.** There is no rule model (type, tag, child-of-primary, and/or, array size), and rig elements carry no tags to test against.

**Porting impact.** The rules only validate, and every connection is explicit (bAutoResolve=False), so the port evaluates identically without them. What is lost is the guard against mis-wiring when a module is reconnected.

**Recommendation.** Express rules on RigExecConnectorAPI: uniform token[] connector:<n>:elementTypes (a list, which covers Or), uniform token[] connector:<n>:requiredTags, uniform token connector:<n>:rule = none|childOfPrimary, and uniform int connector:<n>:minTargets/maxTargets. Evaluate them in a headless python/rigexec/modules/rules.py, shared by the assembly panel and by a notice-only pass in rigEvaluator.cpp that never fails compilation. Provide element tags as uniform token[] rigExec:tags on RigExecControlAPI and on joints, or use stock USD collections.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:4958-4990`; `libs/rigExec/rigEvaluator.cpp:3299-3313`; `libs/rigExec/rigEvaluator.cpp:3244-3254`

**Verification (holds).** The only validation of wiring choices is:
- a source must be a RigExec provider or an Xformable (rigEvaluator.cpp:4958-4990);
- a target must exist inside the asset (rigEvaluator.cpp:3299-3313).

schema.usda has no tag or rule concept; a grep finds no tag attributes. The rules only validate, and every connection is explicit, so minor is right. Alternative worth noting: stock UsdCollectionAPI could supply tag sets instead of a new rigExec:tags attribute.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:4958-4990`; `libs/rigExec/rigEvaluator.cpp:3299-3313`; `libs/rigExecSchema/schema.usda:33-47`

### G1-control-mirror-metadata

**Per-control mirror axis / mirror behavior metadata for pose mirroring**

**Verdict:** Missing · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D16

UE features: `UE2-mirror-metadata`, `UE1-module-mirroring`

**UE rigs.** CRFL_Control's 'Set Mirror Axis' (FVector, default (0,1,1)) and 'Set Mirror Behavior' (bool; FkChain defaults to false, FkArray and the IkFk2Bones FK controls to true) write the per-control metadata 'Mirror Axis' and 'Mirror Behavioral' at 13 call sites. The rig graphs never read these values; external pose mirror and flip tools do.

**usdRig today.** Mirroring metadata on RigExecControlAPI is explicitly deferred, and plugin/ has no pose mirror or flip tool.

**Gap.** There is no per-control mirror axis, mirror behavior or counterpart, and no pose mirror tool.

**Porting impact.** Animators lose mirror-pose and flip-pose on FK, finger and twist controls.

**Recommendation.** Extend RigExecControlAPI in schema.usda with uniform float3 rigExec:mirror:axisMask = (0,1,1), uniform bool rigExec:mirror:behavioral and rel rigExec:mirror:counterpart (the opposite-side twin). Add 'Mirror Pose' and 'Flip Pose' actions in plugin/rigExecUsdview (avarEditorUI.py or gizmoUI.py) that read these values and write avars through the shared undo stack (rigExecUndo.py).

**Evidence:** `libs/rigExecSchema/schema.usda:33-47`; `docs/spec.md:262`

**Verification (holds).** Mirroring metadata is deferred (schema.usda:39-41; spec.md:262). spec.md:1052 even specifies 'an authored mirror reflectionAxis' for controls, but a grep of schema, libs and plugin finds none, so it is documented but missing. plugin/ has no mirror or flip pose action. The rig graphs never read the UE metadata, so minor holds.

Verifier evidence: `libs/rigExecSchema/schema.usda:39-41`; `docs/spec.md:262`; `docs/spec.md:1052`

### G1-array-connectors

**Array connectors: ordered targets, index pairing, silent drop, duplicates**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D16

UE features: `UE1-array-connector-semantics`, `UE1-zebra-connections`

**UE rigs.** Array connectors store an ordered Targets[] list, read with ResolveArrayConnector. Order carries meaning: Pin maps Drivers[i] to Driven[i], FkArray applies Override Parents[i] to Bones[i], and space-list order is the space menu order (Arm L IK Spaces has 9 entries). Targets that do not exist are dropped silently; Monster's Prop Spaces keeps only spine_05. Duplicates are allowed; Pin Drivers lists hand_r twice.

**usdRig today.** Ordered multi-target relationships exist, and their order is preserved and meaningful: RigExecSourceConstraint.rigExec:sources is parallel to inputs:sourceWeights. Three differences remain. (1) A missing source or target is a compile error that fails the whole rig; UE drops it silently. (2) USD relationship list-ops de-duplicate targets, so a list like [hand_r, hand_l, ..., hand_r] cannot be stored as one parallel list (confirmed by probe). (3) Index pairing must be expanded at import into one operator per pair.

**Gap.** There is no skip-missing policy and no ordered list that tolerates duplicates.

**Porting impact.** The importer must prune absent targets (the Monster Prop hand spaces) and expand paired arrays (Pin, FkArray Override Parents) into one constraint per element. Space-menu order carries over through rigExec:sources order.

**Recommendation.** Keep compile strictness and do the pruning and expansion in the importer (python/rigexec/modules/). Optionally add uniform token rigExec:missingSourcePolicy = "error"|"skip" on RigExecSourceConstraint in schema.usda, honored in rigEvaluator.cpp bindFrameSource and in bakedPose.cpp, with a notice for each dropped source. Model paired arrays as one operator per pair, which USD de-duplication cannot break.

**Evidence:** `libs/rigExecSchema/schema.usda:999-1019`; `libs/rigExec/rigEvaluator.cpp:4966-4970`; `libs/rigExec/rigEvaluator.cpp:3299-3305`; `probe (OpenUSD 26.08 behaviour, not repo code): reports/ue-zebrasample/probes/g1/g1_probe.py -- SetTargets([h,g,h]) composes to [h,g]`

**Verification (holds).** Re-running the probe confirms SetTargets([h,g,h]) composes to [h,g], so duplicates cannot be stored.

A missing source is a compile error that restores the previous epoch (rigEvaluator.cpp:4966-4970, 5040-5049), not a silent drop. Ordered sources run parallel to inputs:sourceWeights (schema.usda:999-1019). The importer can prune and expand, so minor holds.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:4966-4970`; `libs/rigExec/rigEvaluator.cpp:5040-5049`; `libs/rigExecSchema/schema.usda:999-1019`; `reports/ue-zebrasample/probes/g1/g1_probe.py (re-run)`

### G1-boombox-prop-rig

**MR_Boombox prop rig: generic Root + socket-chained AddControl modules**

**Verdict:** Partial · **Severity:** minor · **Effort:** L · **Confidence:** medium · **Domain:** D16

UE features: `UE1-boombox-addcontrol-chain`, `UE8-boombox-prop-rig`

**UE rigs.** MR_Boombox is a separate modular prop rig built from the engine's Modules58: one Root plus seven AddControl modules over an 8-bone skeleton. Each AddControl's primary connector requires a Socket (Type rule), and its optional 'Parent Control' accepts a Null, Control or Bone (Or rule). Each AddControl creates '<bone>_ctrl' to drive its socket's bone and spawns '<child>_socket' sockets for the next module. The result is the chain handle -> boombox -> {button, button2, antenna, tape1, tape2} with 10 controls. Per-instance struct config sets ControlSize, the shape (Box_Thick; Box_Thin with offset and scale; Sphere_Thick; Square_Thick at z=1) and CharacterFacingDownAxis.

**usdRig today.** The prop rig itself is easy to express: one RigExecRoot asset with nested RigExecControls and one ParentConstraint per bone (or an FkChain). usdRig has no socket element: a constraint can target the joint directly, or read a plain Xform child of the joint as its source, which then follows the joint's revised delta. Shape offsets and non-primitive shapes are missing. Attaching the boombox to the Zebra's hand across rigs is not supported: mover targets outside the rig asset are rejected, the multipass in spec §6.2 is not built, and rigExec:partition is inert.

**Gap.** There is no socket element and no path for constraints between characters.

**Porting impact.** The Boombox ports as a standalone rig with plain shapes. A shot where the Zebra holds it needs either a shared-rig assembly (the boombox inside the Zebra rig asset) or a baked transform. How zebra_audition links the two is still unknown.

**Recommendation.** Add a socket convention: Xform children of a RigExecJoint marked with rigExec:socket metadata, or a lightweight RigExecSocket (a RigExecXformable subtype without avars) in schema.usda that can be read as a constraint source. For props held by characters, implement the shot-level multipass from spec §6.2: new coordination in libs/rigExecImaging/registry.cpp keyed on rigExec:partition, allowing read-only cross-rig sources resolved from the previous pass's final frames.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:3308-3313`; `libs/rigExec/rigEvaluator.cpp:9566-9624`; `docs/spec.md:1140`; `libs/rigExecSchema/schema.usda:167-179`; `docs/biped-rig.md:181-184`; `libs/rigExecImaging/registry.cpp:293-311`

**Verification (holds).** Confirmed:
- Mover targets must share the asset root (rigEvaluator.cpp:3306-3313).
- rigExec:partition is only authored (rigBuilder.cpp:2615) and nothing reads it.
- The multipass exists only in the spec (spec.md:1140).
- No socket type exists.

Nuance: bindFrameSource does accept a constraint source in another rig (rigEvaluator.cpp:4958-4990). It reads only what this rig's own exec graph computes for that prim, not the other rig's constraint- or solver-revised frame, so it is not a real attach. Minor holds.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:3306-3313`; `libs/rigExecRigging/rigBuilder.cpp:2615`; `docs/spec.md:1140`; `libs/rigExec/rigEvaluator.cpp:4958-4990`

### G1-config-overrides

**Per-instance module configuration (public variables / ConfigOverrides)**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D16

UE features: `UE2-module-config-variables`, `UE1-module-config-values`

**UE rigs.** Public module variables (6-26 per module, grouped under Module Options, Control Shapes Options, Mirroring and Display Name Options) are per-instance configuration. ExecuteQueue re-applies ConfigOverrides after VM init. In Zebra, 32 of 34 modules differ from their class defaults: control scales, shape transforms and colors, display names, Is Neck, IK/PV offsets and distances, rotation orders, foot pivot offsets, twist search strings and weight arrays, and visibility initials. The dump prints ConfigOverrides=(); the effective values are on the asset.t3d preview sub-objects.

**usdRig today.** Values that operators read at evaluation become authored attributes on the generated prims: TwistDistribution weights, constraint offsets, TwoBoneIk offsets and softness, avars:rotationOrder, and guide:*. Any stronger layer or reference site can override them per instance as value-only edits, with no recompile. Values used only by construction (names, counts, shape choice, scale factors, pivot fallbacks) exist only as generator inputs. There is no declared per-module parameter interface, no default or reset, and no grouping.

**Gap.** There is no typed module-parameter surface. Construction-time config is not represented in USD at all. Runtime-read config can be tied to a single module-level value only by wiring single connections by hand.

**Porting impact.** The importer must read per-instance values from asset.t3d, not modular_rig_model.txt, and bake them into the generated prims. A later edit (for example Leg L 'PV Distance Scale' or Neck 'Control Scale') then requires regeneration instead of a single override.

**Recommendation.** Define a module:config:* property namespace on RigExecModuleAPI prims: custom typed attributes with doc and default metadata, for example double module:config:controlScale, double[] module:config:twistWeights and token module:config:rotationOrder. The generators in python/rigexec/modules/ read these values. Generated prims connect their runtime-read inputs back to them with same-typed single connections (AttributeValue already follows one connection), so an edit stays value-only. Add a Module Config section to plugin/rigExecUsdview/avarEditorUI.py that lists module:config:* with reset to the component layer's class default.

**Evidence:** `docs/spec.md:335`; `docs/spec.md:345-356`; `libs/rigExecSchema/schema.usda:334-336`; `libs/rigExecSchema/schema.usda:167-222`; `docs/exec-api-notes.md:241-242`; `docs/exec-api-notes.md:398`

**Verification (holds).** Values read at evaluation can be overridden per instance, and a single same-typed connection is followed:
- OpenExec computeValue: exec-api-notes.md:241-242 and :398.
- RigExecResolvedInputs::GetAttribute: moverGraph.h:314-370.

No module-parameter surface, default or reset exists anywhere in schema.usda. Construction-only values (names, shapes, scale factors, the twist search string) have no USD home. Minor is appropriate.

Verifier evidence: `docs/exec-api-notes.md:241-242`; `docs/exec-api-notes.md:398`; `libs/rigExec/moverGraph.h:314-370`; `libs/rigExecSchema/schema.usda:334-336`

### G1-connector-as-parent

**Module elements parented to connector targets posed by other modules**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D16 · *reconciled after probes* · *added by verifier*

UE features: `UE2-module-connectors`, `UE3-ikfk-connectors`, `UE1-zebra-connections`

**UE rigs.** Modules parent spawned elements directly to connector keys or their resolved targets:
- FkChain and Spine set Parent, Start Parent and End Parent to the literal connector key.
- Finger FK chains hang under the hand bone that the Arm module writes.
- Foot pivots hang under the Leg IK control.
- Ear Base uses skull_tp, which the Face module writes, as its override parent.

In UE, a control or null parented to a bone follows that bone's current global transform once an earlier module writes it. Connector-based parenting therefore gives cross-module follow automatically.

**usdRig today.** Where nesting works and where it stops:
- A prim nested under a control, or under a joint that a constraint poses, follows through namespace propagation.
- A prim nested under a joint that a solver poses does not follow. A solver-bound joint owns its pose and propagation stops there, so nested control guides stay at rest.

Workarounds, none automatic:
- Connect the control's parent:space or default:space to a relay provider nested under the solver-posed joint. The connection is structural, and the regression test exercises it.
- Use FkChain rigExec:startFrame for joint chains.
- Add an explicit ParentConstraint.

The importer must choose per element, depending on which operator poses the target.

**Gap.** 'Parent to connector target' has no single construct that works whatever kind of operator poses the target.

**Porting impact.** If the importer simply nests Zebra finger FK, clavicle and spine-space controls under joints that solvers pose, their guides and avar frames stay at rest while the limb moves. The biped shows this with its finger guides. Every such case must be rewired through parent-space relays or constraints.

**Recommendation.** Add a builder and generator helper, rigexec.modules.common.attach(child, target), that picks the mechanism by target kind:
- namespace nesting for a control or a constraint-posed joint;
- a relay control under the joint plus a parent:space connection for a solver-posed joint;
- rigExec:startFrame for FK joint chains.

Alternatively, let namespace propagation carry solver-output deltas to non-joint descendants (controls, Xforms) in rigEvaluator.cpp commitConstraintFrames and the solver-commit path. Cross-reference G4 (spaces) and G3 (control guides).

**Evidence:** `docs/biped-rig.md:136-141`; `libs/rigExec/rigEvaluator.cpp:10629-10632`; `tests/testRigExecConstraints.cpp:3093-3140`; `libs/rigExecSchema/schema.usda:345-357`; `libs/rigExecSchema/schema.usda:468`

**Report reconciliation.** Contradicted in part by probe g2review/probe_follow2.py: an unclaimed RigExecJoint or RigExecControl nested under a solver-posed joint does follow it. What does not follow is a plain UsdGeomXform null read as a constraint source (probe_follow3.py, see G2-xform-null-constraint-read), and a chain whose joints are themselves solver-claimed (use rigExec:startFrame).

### G1-connectors

**Module connectors: named, typed module inputs (primary/secondary, optional, array)**

**Verdict:** Partial · **Severity:** minor · **Effort:** L · **Confidence:** high · **Domain:** D16 · *(analyst said Missing / minor)*

UE features: `UE1-connector-settings`, `UE2-module-connectors`, `UE3-ikfk-connectors`, `UE1-zebra-connections`, `UE4-unit-module-connector`

**UE rigs.** Each module declares ExposedConnectors: exactly one Primary plus Secondary connectors with bOptional, bIsArray, bPostConstruction and Rules. Examples: IkFk2Bones has Start (primary), Mid and End (ChildOfPrimary), Parent (optional), and IK Spaces / FK Spaces (optional arrays); Prop has Control Vis Channel Host (Type=Control). Inside the module graph, a connector key resolves through the element-key redirector to its ordered targets (ResolveConnector / ResolveArrayConnector, with bIsConnected gating), and a connector can be used directly as a parent. Zebra has 149 connectors and 129 live connection entries, targeting bones, socket nulls and other modules' controls.

**usdRig today.** There is no connector element. An importer resolves each connector to concrete targets and writes them into operator relationships: constraint rigExec:sources, solver rigExec:rootControl/effectorControl/poleControl/joints, FkChain rigExec:startFrame. Targets can be a RigExecJoint, a RigExecControl or any UsdGeomXformable; a plain Xform under a joint follows the joint's revised delta, so it can stand in for a socket. The evaluator reads relationships only with GetTargets (64 call sites, no GetForwardedTargets), so a module cannot route its internal wiring through one interface relationship. USD itself supports this: a probe showed that a forwarded relationship inside a referenced component is remapped, and only the connector's target has to be authored at the instance site.

**Gap.** There is no declared connector interface (name, primary/secondary, optional, array, allowed element types, description). Neither the evaluator nor the baked program follows relationship forwarding. Operators have no 'is connected' state to branch on.

**Porting impact.** All 129 Zebra connections (26 for Monster) are baked into operator relationships at import. Retargeting a module, for example Arm IK Spaces, means editing every consuming relationship instead of one connector.

**Recommendation.** Add a multiple-apply RigExecConnectorAPI to libs/rigExecSchema/schema.usda, with the connector name as the instance name. Properties: rel connector:<n>:targets (ordered), uniform token connector:<n>:kind (primary|secondary), uniform bool connector:<n>:optional, uniform bool connector:<n>:array, uniform token[] connector:<n>:elementTypes (joint|control|xform|any), uniform token connector:<n>:rule, uniform string connector:<n>:description. Switch relationship reads to UsdRelationship::GetForwardedTargets in libs/rigExec/rigEvaluator.cpp (the getTargets helpers), libs/rigExec/bakedProgram.cpp (ctx.Targets) and moverGraph.cpp, and fold the forwarding chain into the epoch digest so a connector edit is structural. Module-internal operators then target <module>.connector:<n>:targets. Add AddConnector/Connect helpers to rigBuilder.{h,cpp} and python/rigexec.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:4949-4956`; `libs/rigExec/rigEvaluator.cpp:4958-4990`; `libs/rigExec/rigEvaluator.cpp:9566-9624`; `libs/rigExecSchema/schema.usda:999-1019`; `examples/biped/Biped_layered_right.usda:842-848`; `probe (OpenUSD 26.08 behaviour, not repo code): reports/ue-zebrasample/probes/g1/g1_probe2.py -- forwarded rel remapped through a reference, targets outside the referenced root dropped`

**Verification (corrected).** Connection itself is expressible, so Missing is too strong:
- Operator relationships carry ordered multi-targets (schema.usda:999-1019).
- Targets may be joints, controls or any Xformable (rigEvaluator.cpp:4958-4990).
- Solver computations read those relationships as OpenExec Relationship() inputs (computations.cpp:704-854), which OpenExec resolves with transitive forwarding (exec-api-notes.md:344-347).

Only the compile-time readers block indirection:
- rigEvaluator.cpp has 60 GetTargets calls and no GetForwardedTargets anywhere in libs/.
- bindFrameSource rejects a property-path target ('must target a prim', rigEvaluator.cpp:4961-4965).

The declared interface is missing: primary/secondary, optional, array, element types and an is-connected state. Re-running the probe confirms forwarding survives a reference. Partial and minor.

Verifier evidence: `libs/rigExecSchema/schema.usda:999-1019`; `libs/rigExec/rigEvaluator.cpp:4949-4990`; `libs/rigExec/computations.cpp:704-854`; `docs/exec-api-notes.md:344-347`; `reports/ue-zebrasample/probes/g1/g1_probe2.py (re-run)`

### G1-dmc-shape-layers

**DMC module: mesh polygroup layers as IK/FK control shapes, published via Root metadata**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D16

UE features: `UE7-dmc-module-crm-fn-dmc`, `UE7-dmc-module-consumers`

**UE rigs.** When enabled, CRM_FN_DMC construction runs SetupShapeLibraryFromLayer('ik-layer') and ('fk-layer') on the mesh polygroup layers and publishes 'Direct Mesh Control Libraries' and 'Direct Mesh Control' in the Root namespace. IkFk2Bones, FkChain, FkArray, Spine and Foot then name their control shapes '<layer>.<bone>': IK uses ik-layer.<end bone>, PV uses ik-layer.<mid>, FK uses fk-layer.<bone>, and the toes use the Ball Joint. The controls are drawn and picked as patches of the deformed mesh. IkFk2Bones also hides its PV debug line when DMC is found. The FkArray forward-graph DMC branch is dead code. In the headless dump, no control resolved to a layer shape.

**usdRig today.** TouchPose is the closest usdRig feature. RigExecTouchRegions and RigExecTouchRegion name face sets on a mesh, each with rel rigExec:touch:control; a shipped overlay mesh draws them on the deformed points, and a click selects the control. It supports only one region layer, has no IK-vs-FK layer switching, cannot generate regions from mesh polygroup layers by dominant bone, and the patch is not the control's gizmo.

**Gap.** There are no multi-layer region sets keyed by solve mode, and no generator from polygroup layers.

**Porting impact.** DMC-style mesh picking can be approximated with TouchPose regions painted per bone. The IK-layer/FK-layer switch and the mesh-patch gizmo look are lost. The feature is experimental and inactive in the dump, so the impact is low.

**Recommendation.** Add uniform token rigExec:touch:layer (for example "ik" or "fk") to RigExecTouchRegions, and let the touch overlay pick the active layer from a connected dial such as the limb's IK/FK weight (plugin/touchPose, libs/rigExecImaging/touchOverlayAdapter.cpp). Add python/rigexec/modules/dmc.py to build regions from mesh face sets by dominant skin influence. The DMC GPU overlay deformer belongs to G7.

**Evidence:** `libs/rigExecSchema/schema.usda:2518-2635`; `plugin/touchPose/touchPoseModel.py:1-20`

**Verification (holds).** TouchRegions and TouchRegion exist: a face set with a rel control, and an overlay drawn on deformed points (schema.usda:2530-2635). There is no layer token and no switching by solve mode. The feature is experimental and inactive in the dump, so minor.

Verifier evidence: `libs/rigExecSchema/schema.usda:2530-2635`

### G1-element-metadata-channel

**Per-element typed metadata written by modules (construction-computed offsets, provenance)**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** medium · **Domain:** D16 · *added by verifier*

UE features: `UE1-module-metadata-bus`, `UE3-module-metadata-bus`, `UE1-module-instances-zebra`

**UE rigs.** Beyond module metadata, modules call SetMetadata and GetMetadata on individual elements with NameSpace=Self, which prefixes keys with the module name.

Where each key is written and read:
- Foot 'Delta' (TRANSFORM): written in Construction (Foot graphs.txt:346-421), read in Forwards (:55).
- Spine 'Offset' (TRANSFORM): written in Construction (:295-303), read in Backward Solve (:1281-1285) and the Attach Sec FKs function (:2067-2072).
- Spine 'FkDeltaTransform' (TRANSFORM): written in Construction (:338), read in Backward Solve (:1242).
- Spine 'Bone Percentage' (FLOAT): written at :257 (Construction), read at :2005.
- Body 'Body Delta Transform' and Prop 'Color': also written by their modules.

Every one of the 878 module-spawned Zebra elements also carries provenance metadata: Module, DesiredName and DesiredKey. runtime_hierarchy.txt prints these keys only as key:type, never their values.

**usdRig today.** Static per-element data can be custom namespaced attributes or customData on the prim. Values that operators consume must be baked into operator inputs:
- ParentConstraint offsets are per-source double3[] rotation and translation arrays, not matrices.
- Scalar connections must be single and same-typed.
- Matrix-valued data can feed default:space or posed:defaultSpace through a matrix connection.

No schema defines a metadata or tag namespace; RigExecControlAPI has only channelRole. Nothing records which generator or module produced a prim, so the prim path is the only ownership signal.

**Gap.** No typed per-element metadata convention with a reader contract, and no element-provenance record. The construction-computed values are not in the UE dump, so they must be recomputed.

**Porting impact.** A 'frozen' import is incomplete: the importer must re-run the Foot, Spine and Body construction math (Delta, Offset, FkDeltaTransform, Bone Percentage) from rest frames and bake the results into operator attributes. Which module owns an element is lost except through its path.

**Recommendation.** Document a custom-attribute convention on RigExecControl and RigExecJoint, 'rigMeta:<module>:<name>' (or a customData dictionary), in the schema docs.

Record ownership with rel rigExec:module:owner, or derive it from the proposed RigExecModuleAPI subtree.

In python/rigexec/modules/import_ue.py, recompute construction metadata from rest frames. Emit matrix-valued runtime metadata as matrix4d attributes connected into default:space or posed:defaultSpace, which connections already support.

**Evidence:** `libs/rigExecSchema/schema.usda:33-47`; `libs/rigExecSchema/schema.usda:1115-1142`; `libs/rigExecSchema/schema.usda:307-357`; `libs/rigExec/rigEvaluator.cpp:522-569`

### G1-face-member-tables

**Face member variables: cached element keys and authored tuning tables**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D16

UE features: `UE6-member-config-tables`

**UE rigs.** Construction stores every spawned element key in a private member variable (for example 'Corner L', or 'Lip Roll Ot Tp' for a channel), plus arrays of keys (lid spawn bones, lip nulls and controls) that the forward solve reads. The tuning tables are member defaults: 13 TArray<FQuat> lid tables, 4 Soft Eyes quaternion arrays of 12, 4 FVector arrays, and 3 user-struct arrays (Lid Tp 16 rows, Lid Bt 13, Lip Null 20; Monster has 17/11/20). The modules have 88 (Zebra) and 108 (Monster) variables.

**usdRig today.** Element keys become static relationships, so no lookup is needed. Tuning tables have no generic home: their values must be spread into the consuming operators' own attributes, such as the per-source rotation and translation offset arrays on ParentConstraint, TwistDistribution weights and RigExecPose values. No struct or table attribute exists that an operator can index at runtime. The one state variable (Jaw Normalize) is covered in G1-stateful-module-variables.

**Gap.** There are no table-valued inputs; every table row must become authored values on generated operators.

**Porting impact.** The lid and lip tables are expanded at import into many per-operator values. Retuning a table means regenerating, not editing one array.

**Recommendation.** Keep the tables as module:config:* arrays on the Face RigExecModuleAPI prim (quatf[], float3[], and parallel arrays for struct columns). The face generator (python/rigexec/modules/face.py) expands them into operator attributes, and a regeneration hook on edit keeps the two in sync. Add a runtime indexed-table input only if live tuning is required.

**Evidence:** `libs/rigExecSchema/schema.usda:999-1019`; `libs/rigExecSchema/schema.usda:1115-1142`; `libs/rigExecSchema/schema.usda:1928-1945`

**Verification (holds).** No table-valued or struct-valued runtime inputs exist. Per-source offsets on ParentConstraint (schema.usda:1115-1142) and RigExecPose values (schema.usda:1928-1945) are the only homes for the expanded rows. Minor.

Verifier evidence: `libs/rigExecSchema/schema.usda:1115-1142`; `libs/rigExecSchema/schema.usda:1928-1945`

### G1-face-module-shell

**Face module shell: Root/Parent connectors and placement in the modular rig**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** medium · **Domain:** D16 · *reconciled after probes*

UE features: `UE6-face-module-shell`

**UE rigs.** The face module declares Root (primary), Parent (secondary, optional, ChildOfPrimary) and a Root socket. Zebra connects Root to root and Parent to head, with parent module root. Monster places Face under module Spine and leaves Parent unconnected. The Zebra Ears and Mohawk use Face as their parent module; this only affects ordering, since they depend on skull_tp. The face has no config overrides or bindings, and all 107 (Zebra) or 120 (Monster) face controls are spawned as Face/<name>.

**usdRig today.** The face is a subtree or component with its own Movers scope. 'Parent = head' becomes a ParentConstraint on a face root control with the head joint as its source; the committed delta propagates to all nested face controls. Nesting face controls directly under a solver-posed head joint would not make them follow. For Monster (no Parent), the face root is left unconstrained. The Ears and Mohawk run after the face because their constraints read skull_tp, which the face writes.

**Gap.** This shell adds no face-specific engine gap; it is covered by the generic connector and module gaps.

**Porting impact.** Face controls follow the head only through an explicit constraint. The generator must add that constraint and make it execute before the face's own constraints.

**Recommendation.** In python/rigexec/modules/face.py, emit <rig>/Movers/Face/{follow_head (a ParentConstraint whose source is the Parent connector, at the bottom of the stack so it runs first), then the face constraints}. Skip follow_head when the Parent connector is empty (Monster).

**Evidence:** `libs/rigExec/rigEvaluator.cpp:10569-10660`; `docs/biped-rig.md:136-141`; `libs/rigExec/rigEvaluator.cpp:5794-5806`

**Report reconciliation.** The mapping's claim that nesting under a solver-posed head will not follow is contradicted by probe g2review/probe_follow2.py: unclaimed joints and controls nested under a solver-posed joint follow its solved frame (rigEvaluator.cpp commitConstraintFrames, solverOutput path). Nesting the face root under the head joint works; a ParentConstraint is only needed for maintain-offset or weighting.

**Verification (holds).** Confirmed: namespace propagation stops at solver-bound joints (rigEvaluator.cpp:10629-10632; biped-rig.md:136-141).

One correction: 'follows the head only through an explicit constraint' is too narrow. A face root control can also follow a solver-posed head through a connected parent:space or default:space relay nested under the joint. tests/testRigExecConstraints.cpp:3093-3140 exercises exactly this. Minor stands.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:10629-10632`; `docs/biped-rig.md:136-141`; `tests/testRigExecConstraints.cpp:3093-3140`

### G1-foot-module-config

**Foot module connectors, pivot fallbacks and config**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D16

UE features: `UE3-foot-connectors-config`

**UE rigs.** CRM_FN_Foot's connectors are Foot Joint (primary); Ball Joint and Toe Joints (optional array), both ChildOfPrimary; and the optional Toe Tip, Heel, Inner Bank and Outer Bank pivots. Zebra connects the four pivots to authored nulls. Public config: pivot offsets in footprint space, used only when a pivot is unconnected; Negative Side (true on Foot R); Control Scale; FK Mirror Behavior; and shape structs (Box_Thick, Sphere_Solid, HalfCircle_Thick, Sphere_Thick). The Biped leaves the pivots unconnected. Zebra uses Z offsets of -3 and -2.5 and a pivot shape scale of 0.35.

**usdRig today.** The biped already builds a reverse foot as nested pivot controls under the leg IK control, with roll and bank driven by FloatMathMover chains and rotation constraints. The generator would place those pivots from the connected nulls, or from the offsets when a pivot is unconnected. HalfCircle and custom shapes are not in the guide set. The reverse-foot solve is a pattern authored in the asset, not an engine solver (see G5).

**Gap.** There is no foot generator and no HalfCircle shape.

**Porting impact.** The foot ports as a generated pivot stack at the Zebra null positions. Toe shapes are drawn with the nearest available primitive.

**Recommendation.** Add python/rigexec/modules/foot.py, producing the biped-style pivot stack from the resolved pivots or the offsets. Add a "halfCircle" token to guide:shape in schema.usda and support it in libs/rigExecImaging/sceneIndices.cpp. Solver semantics are covered in G5.

**Evidence:** `examples/biped/Biped.usda:3048-3157`; `libs/rigExecSchema/schema.usda:167-179`; `libs/rigExecSchema/schema.usda:1192-1211`

**Verification (holds).** The biped's reverse-foot pattern exists: a pivot stack under the leg IK, plus FloatMathMover chains driving rotation-constraint envelopes (Biped.usda:3048-3157). guide:shape has no half-circle token (schema.usda:167-169). Minor.

Verifier evidence: `examples/biped/Biped.usda:3048-3157`; `libs/rigExecSchema/schema.usda:167-169`

### G1-metadata-bus-static

**Module metadata bus: published element keys and values (Self/Parent/Root namespaces)**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D16

UE features: `UE1-module-metadata-bus`, `UE3-module-metadata-bus`, `UE2-root-published-metadata`

**UE rigs.** SetModuleMetadata and GetModuleMetadata (25 and 44 uses) let modules publish typed values on their primary connector and read them through the Self, Parent or Root namespace. Values published at construction include the Leg's 'IK Null', 'IK Control' and 'IK Driver' (the Foot, a child module, parents its pivots under the leg IK), the Root and Body control keys, the global scale and colors, and the DMC module's 'Direct Mesh Control Libraries'.

**usdRig today.** Published element keys become direct relationships or namespace nesting written by the generator. The biped nests its whole reverse-foot pivot stack under leg_l_ik, which is exactly the Foot-to-Leg 'IK Control' contract. Published scalars become custom attributes consumed through same-typed connections. There is no lookup by module namespace (Self, Parent or Root) and no typed publication point on a module.

**Gap.** There is no module publication interface and no Parent or Root namespace resolution.

**Porting impact.** The importer resolves each GetModuleMetadata call to a concrete path, and the live coupling survives as relationships.

**Recommendation.** Allow rel module:publish:<name> and typed module:publish:<name> attributes on RigExecModuleAPI. Consumers forward to <parentModule>.module:publish:<name> (using the relationship forwarding from G1-connectors) or connect to the attribute. The assembler resolves the Parent and Root namespaces through rigExec:module:parent at authoring time.

**Evidence:** `examples/biped/Biped.usda:3048-3060`; `examples/biped/Biped.usda:4541-4545`; `libs/rigExec/rigEvaluator.cpp:522-569`

**Verification (holds).** Biped.usda:3048-3060 nests the reverse-foot pivot stack under leg_l_ik and connects the foot aim envelopes to leg_l_ikfk.inputs:weight. That is the 'IK Control' contract expressed statically. No typed publication point or Parent/Root namespace lookup exists. Minor.

Verifier evidence: `examples/biped/Biped.usda:3048-3060`; `libs/rigExec/rigEvaluator.cpp:522-569`

### G1-mirror-module

**MirrorModule: create the opposite-side module with mirrored config and connections**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D16

UE features: `UE1-module-mirroring`

**UE rigs.** UModularRigController::MirrorModule(module, {MirrorAxis, AxisToFlip, SearchString, ReplaceString}) creates a module of the same class under the same parent, with the name search-replaced. It recreates connections with search-replaced target names (_l -> _r), search-replaces binding paths, writes MirrorVector/MirrorTransform of public FVector and FTransform config values, and copies all other values. Zebra's right-side modules show the sign-flipped results (Foot R bank offsets, Clavicle R shape offset, twist shape Z).

**usdRig today.** Mirroring through composition is demonstrated. Biped_layered_right references each left prim from the left layer file and overrides only the mirrored rest:space, color, aim vector and relationships. It uses references rather than inherits so that animation does not leak across sides. The tool that produced this (tools/biped/split_layers.py) is not in the repo, and neither the builder nor the Python package has a mirror operation.

**Gap.** There is no in-repo mirror operator (name search/replace, relationship and connection retargeting, mirrored vector and matrix attributes).

**Porting impact.** Importing both sides from UE avoids the problem, but re-mirroring after a left-side edit is manual.

**Recommendation.** Add python/rigexec/mirror.py with mirror_subtree(stage, src, dst, plane='YZ', search='_l', replace='_r', edit_layer). It authors 'prepend references' to the source prim, then overrides rest:space and every double3, float3 and matrix4d input with its mirrored value (the equivalents of MirrorVector and MirrorTransform). It retargets relationships and connections by search/replace, including connector targets once RigExecConnectorAPI exists. Add a 'Mirror Module' action next to the composition-arc tooling (plugin/rigExecUsdview/compositionArcsUI.py).

**Evidence:** `examples/biped/Biped_layered_right.usda:842-848`; `docs/biped-rig.md:227-252`; `docs/biped-rig.md:280-290`; `bin/run_biped.bat:2-11`; `libs/rigExecRigging/rigBuilder.h:1030-1189`

**Verification (holds).** Reference-based mirroring is demonstrated (Biped_layered_right.usda:842-848; biped-rig.md:227-252). The tool that produced it is absent: git ls-files lists nothing under tools/biped, although bin/run_biped.bat:2-11 resolves scripts there. python/rigexec has no mirror module (only bake, curvenet, inverse). Minor.

Verifier evidence: `examples/biped/Biped_layered_right.usda:842-848`; `docs/biped-rig.md:227-252`; `bin/run_biped.bat:2-11`

### G1-module-instance-model

**Modular rig model: module instances, parent module, component reference, overrides**

**Verdict:** Partial · **Severity:** minor · **Effort:** L · **Confidence:** high · **Domain:** D16

UE features: `UE1-modular-rig-model`, `UE1-module-instances-zebra`

**UE rigs.** A ModularRig host holds FModularRigModel with three parts. Modules[] entries are {Name, ParentModuleName, ControlRigAssetReference (BP class or runtime asset), ConfigOverrides, Bindings}. Connections[] map a connector 'Module/Name' to ordered element keys. PreviousModulePaths records old names. The host graph is empty; all 359 Zebra controls come from 34 module instances (8 LimbTwist, 8 FkChain, 6 FkArray, 4 IkFk2Bones, 2 Foot, 2 Spine, plus Root, Body, Prop and Face), and each instance name prefixes every element it spawns.

**usdRig today.** The nearest construct is USD composition. A module instance can be a prim that references a component layer (spider_leg / ArmShotAnim pattern) with stronger-layer overrides. The evaluator discovers controls, joints and solvers by type anywhere under the RigExecRoot, with no membership lists. However, there is no module type, identity, parent module or connector interface. Movers are discovered only under the single <rig>/Movers scope. The C++ builder forces controls, joints, solvers and movers into four fixed scopes, so one module ends up split across four subtrees. USD drops relationship targets that point outside a referenced component (probe), so every external wire must be re-authored at the instance site, as Biped_layered_right does.

**Gap.** There is no first-class module: no RigExecModule prim or API carrying class identity, parent module and instance name. The stock builder cannot keep a module's controls, solvers and movers in one referenceable subtree. External wiring cannot travel inside a referenced component.

**Porting impact.** An importer has to flatten MR_Zebra (34 modules) and MR_Monster (7) into one generated rig of about 2000 prims, as the biped builder does. The result evaluates correctly, but after import the rig loses its module boundaries, per-module regeneration, and module reuse across Zebra, Monster and the Biped template.

**Recommendation.** Add a single-apply RigExecModuleAPI in libs/rigExecSchema/schema.usda with uniform token rigExec:module:class, uniform string rigExec:module:version, rel rigExec:module:parent and doc. Apply it to a grouping Scope at <rig>/Movers/<Module>, so the module's controls, solvers and constraints can share one subtree (the evaluator already accepts controls and solvers anywhere). Add RigExecRigBuilder::AddModule(name, componentAsset, parentModule) and module-scoped AddControl/NewMoverChain variants in libs/rigExecRigging/rigBuilder.{h,cpp} and python/rigexec. Add a notice-only validation in rigEvaluator.cpp for unknown parents and parent cycles, and group rows by module in plugin/rigExecUsdview/execStackUI.py. Pair this with RigExecConnectorAPI (G1-connectors).

**Evidence:** `libs/rigExecSchema/schema.usda:95-111`; `libs/rigExec/rigEvaluator.cpp:1065-1084`; `libs/rigExec/rigEvaluator.cpp:3239-3243`; `libs/rigExecRigging/rigBuilder.h:11-23`; `libs/rigExecRigging/rigBuilder.cpp:2688-2705`; `examples/biped/Biped_layered_right.usda:842-848`; `docs/biped-rig.md:280-290`; `examples/ArmShotAnim.usda:8-16`; `docs/spec.md:270`

**Verification (holds).** Confirmed:
- RigExecRoot declares no membership (schema.usda:95-111).
- Solvers are discovered by type anywhere under the rig (rigEvaluator.cpp:1065-1084).
- Movers are read only from <rig>/Movers (rigEvaluator.cpp:3239-3243).
- No module, connector or socket class exists among the 54 schema classes.
- Re-running the analyst's probe confirms USD drops relationship targets outside a referenced root (warning 'refers to a path outside the scope of the reference'), matching biped-rig.md:280-290.

One overstatement: 'the stock builder cannot keep a module in one subtree' is true only of RigExecRigBuilder::Add* (rigBuilder.h:11-23; rigBuilder.cpp:2688-2705 forces nested controls to stay inside Controls). The strict schema layer, rigexec.schema.<Type>.define(stage, path) (python/rigexec/__init__.py:340-365, 391-403), defines any typed prim at any path. Controls and solvers are discovered anywhere, so a generator can already group one module's controls, solvers and constraints under <rig>/Movers/<Module>.

Minor is right: evaluation is unaffected and the loss is TD-facing structure.

Verifier evidence: `libs/rigExecSchema/schema.usda:95-111`; `libs/rigExec/rigEvaluator.cpp:1065-1084`; `libs/rigExec/rigEvaluator.cpp:3239-3243`; `libs/rigExecRigging/rigBuilder.cpp:2688-2705`; `python/rigexec/__init__.py:340-365`; `python/rigexec/__init__.py:391-403`; `docs/biped-rig.md:280-290`; `reports/ue-zebrasample/probes/g1/g1_probe2.py (re-run: out-of-scope targets dropped, forwarded rel remapped)`

### G1-monster-deformers-in-module

**Monster runs its 9 deformers inside the animator face module, not in post-process**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D16 · *(analyst said Divergent-by-design / minor)*

UE features: `UE-monster-deformers-in-face-module`, `UE7-monster-face-inline-chain`

**UE rigs.** In MR_Monster, the Face module's Forwards Solve first writes the nine *_deformer curves from the squash controls (SetCurveValue_39..79; squash curves are remapped from -1..1 to 0..1). It then chains 9 AddOptimusDeformer nodes: Head, HeadBend, HeadTwist, MuzzleBend, MuzzleSquash, MouthBend, MouthSquash, SkullTpBend, SkullTpSquash. CR_Monster_Deform adds none, so Monster deformation runs only while the animator modular rig evaluates. Zebra's deformers run in the post-process rig for any animation source.

**usdRig today.** usdRig has a single evaluation contract: a rig's movers always run when the rig is evaluated, and there is no split between an animator rig and a post-process rig. The layer stack or variants choose what is present. The curve-to-trait remaps map to a FloatMathMover remap followed by a clamp, since remap never clamps. The deformers themselves do not exist: there are no bend, twist or squash kernels (G7). Driving the float remaps from transform-control avars is also blocked, because avars are double and scalar connections must match type exactly.

**Gap.** Where the deformers run needs no equivalent. The real gaps are the missing parametric deformers and the missing bridge from double avars to float movers.

**Porting impact.** The placement ports trivially: the deformers run on every evaluation for both characters. The Monster head deforms only once G7's deformers exist, and the control-to-curve step needs a typed bridge.

**Recommendation.** Author the Monster deformer movers in the face component under <rig>/Movers/Face, after the face constraints. For the bridge, let RigExecFloatMathMover and other float scalar inputs accept a single double source with an explicit narrowing conversion, in _ValidateScalarConnection (rigEvaluator.cpp:522-569), RigExecResolvedInputs and the baked ResolveBind. Alternatively, add a double-typed RigExecDoubleMathMover. The deformer kernels are tracked in G7.

**Evidence:** `libs/rigExecSchema/schema.usda:1192-1211`; `libs/rigExec/rigEvaluator.cpp:547-553`; `libs/rigExecSchema/schema.usda:1663-2315`; `examples/biped/Biped_stack.usda:1-70`

**Verification (corrected).** No non-goal is cited, and none exists: grepping spec.md and README.md for post-process or animator/post-process rigs finds nothing. Catalog R3-no-runtime-game-integration records an absence, not a design decision.

What exists and what does not:
- Placement ports natively, because movers run whenever the rig evaluates.
- The scalar stage exists: FloatMathMover remap plus clamp (schema.usda:1192-1214).
- UE derives the curves from GetTransform(LocalSpace) of the squash controls, then Remap (CRM_Monster_Face graphs.txt:1234-1300). usdRig cannot read those double avars into float movers, because connections must match type exactly (rigEvaluator.cpp:560-567; avarEditorModel.py:37-45).
- The kernels are absent: no bend, twist or squash class in schema.usda.

Some of it exists and some semantics are missing, so Partial. Minor, because the kernels are tracked in G7.

Verifier evidence: `libs/rigExecSchema/schema.usda:1192-1214`; `libs/rigExec/rigEvaluator.cpp:560-567`; `plugin/rigExecUsdview/avarEditorModel.py:37-45`; `ue/<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/graphs.txt:1234-1300`

### G1-optional-connectors

**Optional connectors left unconnected (module fallback paths)**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D16

UE features: `UE8-conn-optional-unconnected`

**UE rigs.** Every connector left unconnected is declared bOptional. Zebra has 27: Foot Toe Joints, Clavicle End, finger Orient Spaces, Neck Start Snap To, and the FkArray Spaces, Orient Spaces and Override Parents. Monster has 3; the Biped template has 20, including all 8 foot pivots. Modules branch on bIsConnected and take a fallback; for example, Foot places its pivots from the 'Heel/Toe Tip/Bank Pivot Offset' variables when the socket nulls are not connected.

**usdRig today.** An unauthored relationship is normal in USD, and a mover whose rigExec:moves is empty is treated as inert with a notice. A source constraint with an empty rigExec:sources, however, is a compile error that fails the whole rig. No operator has an 'if connected, else fallback' form, so the generator has to decide the fallback.

**Gap.** Operators fed by an optional connector have no 'absent input means inert' behaviour.

**Porting impact.** The importer must evaluate each fallback (pivot offsets, no orient spaces) and leave out the operators that depend on the missing connection. A single empty space list authored by mistake takes down the whole rig.

**Recommendation.** Once RigExecConnectorAPI exists, treat a constraint whose rigExec:sources forward only to an empty optional connector the way an empty-moves mover is treated: skip it with a notice (rigEvaluator.cpp near line 5033, and the constraint build in bakedPose.cpp). Generators emit fallback prims (for example pivot Xforms at the configured offset) under a variant or an inactive branch selected by connector state.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:3256-3276`; `libs/rigExec/rigEvaluator.cpp:5026-5039`

**Verification (holds).** Confirmed:
- An empty rigExec:moves is inert with a notice (rigEvaluator.cpp:3256-3276).
- Empty rigExec:sources is a hard error (rigEvaluator.cpp:5033-5039). The check runs before any enable or weight read, so disabling the constraint does not help.

Workaround: the generator omits the operator or sets active=false, which is structural (spec.md:335). Minor.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:3256-3276`; `libs/rigExec/rigEvaluator.cpp:5033-5039`; `docs/spec.md:335`

### G1-template-lineage

**Template-derived character rigs (Biped -> Zebra, Monster) and forked modules**

**Verdict:** Partial · **Severity:** minor · **Effort:** L · **Confidence:** medium · **Domain:** D16

UE features: `UE1-zebra-vs-biped-template`, `UE1-zebra-vs-monster`, `UE6-zebra-vs-monster`

**UE rigs.** Zebra and Monster were created from MR_FN_Biped; their PreviousModulePaths are identical. Zebra removed Meta, Middle, Ring, the 12 proxies, Attach, Stretch Feedback and IK Bone Pins. It re-parented Index and Pinky, added Tweakers, Face, Ears and Mohawk, retuned scales, PV distances, twist weights and foot pivots, and connected the foot pivots. Monster is a 7-module bust whose body config is copied from Zebra, with Face placed under Spine. Its face module is a fork (+39/-28 construction nodes, +110/-5 forward nodes): the layout is rescaled, the mouth, lips and cheeks are re-parented, squint, sticky, mouth squash, puff, ch and 9 deformers are added, the lid constraints are fixed, and DMC switching and the smile lid push are removed.

**usdRig today.** USD composition supports this lineage directly: a template layer that each character references, with character layers that add modules, deactivate removed ones (active=false is structural), override values and retarget relationships. Renames can use relocates, which the composition-arc panel authors. There are no module or template semantics on top of this, and relationships that leave a referenced subtree must be re-authored for each character. A forked module such as the Monster face is simply another component; no diff or inheritance tooling is aware of modules.

**Gap.** There is no diff or override tooling at the template or module level, and external wiring is not inherited through the arc (connector forwarding would fix this).

**Porting impact.** Zebra, Monster and the Biped can share a template layer only if the importer factors it out. Otherwise each character is imported flat, and the lineage is lost: shared fixes no longer flow to both characters.

**Recommendation.** Have the importer emit a biped template component, with modules as referenced sub-components carrying RigExecModuleAPI and RigExecConnectorAPI, and emit Zebra and Monster as override layers (deactivate, add, retune, set connector targets). With relationship forwarding (G1-connectors), external wiring lives only in connector targets, so the template carries its internal wiring unchanged.

**Evidence:** `docs/spec.md:345-356`; `docs/spec.md:335`; `docs/composition-arcs.md:1-30`; `examples/biped/Biped_stack.usda:1-70`; `docs/biped-rig.md:280-290`

**Verification (holds).** USD composition covers lineage (references, overrides, active=false per spec.md:335, relocates per composition-arcs.md:17). External relationships are not rebased by per-prim arcs (biped-rig.md:280-290). Minor.

Verifier evidence: `docs/spec.md:335`; `docs/composition-arcs.md:17`; `docs/biped-rig.md:280-290`

### G1-variable-bindings

**Module variable bindings to host-rig variables**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D16

UE features: `UE1-variable-bindings`

**UE rigs.** FRigModuleReference.Bindings maps a module's public variable to a host variable ('Var') or to another module's variable ('Module.Var'). UpdateModuleVariables copies the source value before every module execution, so a binding overrides config. Zebra and Biped bind Arm L/R 'Rotation Order' to the host EEulerRotationOrder variables Arm_L/R_RotateOrder (XYZ), so the Arm FK controls use XYZ at runtime even though the Arm L config says XZY. The DMC templates bind CRM_FN_DMC 'Direct_Mesh_Control' to a host bool, although the module variable is spelled 'Direct Mesh Control'.

**usdRig today.** A single same-typed attribute connection acts as a binding for any input the evaluator reads through AttributeValue, RigExecResolvedInputs or the baked ResolveBind walk. For example, each Arm FK control's token avars:rotationOrder can connect to a rig-level custom token attribute (say, on the RigExecRoot), and editing the source is a value edit. Limits: exactly one source of exactly the same type, so no enum or int to token coercion. Not every read follows connections: inputs:enabled is read with a plain Get, and guide colors follow only one hop. There is no declared binding list and no module-to-module variable path.

**Gap.** There is no rig-settings surface and no binding declaration. Readers are inconsistent about following connections (the enable bool and guide scale do not).

**Porting impact.** Arm rotation order can stay live through connections. The importer must apply the bound value (XYZ), not the XZY config value. The DMC bool is only used during construction, so it becomes a variant choice.

**Recommendation.** Standardize a 'settings:' custom-attribute namespace on RigExecRoot for host variables, documented in the RigExecRoot doc in schema.usda. Route every scalar and token read through RigExecResolvedInputs: fix _IsEnabled (rigEvaluator.cpp:7688-7694) and the guide scale reads in libs/rigExecImaging/registry.cpp:1313 and bridge.cpp. List settings:* in the Avar Editor (plugin/rigExecUsdview/avarEditorModel.py) so animators can change them.

**Evidence:** `docs/exec-api-notes.md:241-242`; `docs/exec-api-notes.md:398`; `libs/rigExec/moverGraph.h:314-318`; `libs/rigExec/bakedProgram.cpp:1473-1484`; `libs/rigExecSchema/schema.usda:334-336`; `libs/rigExec/rigEvaluator.cpp:522-569`; `libs/rigExec/rigEvaluator.cpp:7686-7695`

**Verification (holds).** The verdict and severity stand, but one mapping claim is wrong: inputs:enabled does follow connections on every production path.
- Compile validates at most one bool connection for every mover (rigEvaluator.cpp:3631-3648).
- The pose walk reads it through _ResolvedRead, i.e. RigExecResolvedInputs::GetAttribute (rigEvaluator.cpp:89-101, 10990-10991; moverGraph.h:314-370).
- The geometry graph reads it through resolved->GetAttribute (moverGraph.cpp:1477-1487, 1576-1586).
- The baked programs bind it (bakedPose.cpp:603; bakedProgram.cpp:1927).

Only _IsEnabled uses a plain Get (rigEvaluator.cpp:7688-7694). Its only caller is _EvaluateChain (7750), which is called only for the CPU parity check (12484, 12574), so this is a parity-oracle bug, not a binding gap.

The other claims hold:
- Guide-scale reads ignore connections (registry.cpp:1273-1279, 1313; bridge.cpp:965-971).
- guide:displayColor follows exactly one hop (bridge.cpp:196-217).

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:3631-3648`; `libs/rigExec/rigEvaluator.cpp:10990-10991`; `libs/rigExec/moverGraph.cpp:1477-1487`; `libs/rigExec/bakedPose.cpp:603`; `libs/rigExec/rigEvaluator.cpp:7688-7694`; `libs/rigExecImaging/registry.cpp:1273-1279`; `libs/rigExecImaging/bridge.cpp:196-217`

### G1-construction-offline

**Construction events spawn the whole rig at runtime**

**Verdict:** Divergent-by-design · **Severity:** minor · **Effort:** XL · **Confidence:** high · **Domain:** D16 · *(analyst said Divergent-by-design / major)*

UE features: `UE1-modular-rig-model`, `UE1-module-execution-order`, `UE2-module-config-variables`

**UE rigs.** Each module's Construction event runs in module-tree order and records spawn indices. It creates controls, channels, nulls, sockets and metadata from the resolved connectors and public config, then fills the private element-key variables that the Forwards and Backwards solves use. Changing a connection or a config value re-runs construction, which rebuilds the hierarchy. The host ModularRig graph is empty.

**usdRig today.** usdRig never creates prims during evaluation: dynamic topology is a v1 non-goal, and nothing is authored onto the stage. Construction therefore becomes offline authoring. RigExecRigBuilder / rigexec.Builder write typed prims and wiring into a layer, and the evaluator compiles whatever is composed. A structural edit starts a new epoch, which is the analogue of re-running construction. No generator exists for any Fortnite module class, and the builders have no construction hooks (no spawn-from-skeleton, mirror, auto-orient or offset capture).

**Gap.** There is no module-generator layer. The 12 Fortnite module construction graphs (Root, Body, Spine, IkFk2Bones, Foot, LimbTwist, FkChain, FkArray, Prop, Pin, ProxyControl, DMC) and the 2 face-module construction graphs must be re-implemented as deterministic offline generators. Nothing records which generator and config produced a subtree.

**Porting impact.** Every Zebra/Monster element must come either from ported generators or from the UE runtime_hierarchy dump imported as frozen data. Frozen data cannot be regenerated when a connection or config value changes.

**Recommendation.** Add python/rigexec/modules/ with one generator per module class, each a pure function (stage, moduleScope, connectors, config) that authors prims through rigexec.Builder. Add python/rigexec/modules/assemble.py to walk RigExecModuleAPI prims in rigExec:module:parent order and regenerate each module into its own sublayer or variant. Record provenance on RigExecModuleAPI (rigExec:module:generator, rigExec:module:configHash) so tools can detect stale generated subtrees. The evaluator does not change, which is consistent with spec.md:58.

**Evidence:** `docs/spec.md:58`; `README.md:22-23`; `libs/rigExecRigging/rigBuilder.h:1-23`; `libs/rigExecRigging/rigBuilder.h:1030-1189`; `docs/spec.md:335`

**Verification (corrected).** The verdict stands:
- spec.md:58 rejects dynamic topology during evaluation.
- README.md:22-23 says nothing is authored onto the stage.
- The builder is imperative, with no construction hooks (rigBuilder.h:1030-1189).

The severity is overstated. On this scale, major means visible behavior or a key animator workflow is lost; the stated loss (no regeneration after a connector or config edit) affects riggers only. An importer that runs the construction math offline, or imports frozen data, gives animator-equivalent results, and nothing in usdRig stops an offline Python generator from authoring through rigexec.schema.*.

Caveat for the frozen route: runtime_hierarchy.txt lists construction-computed element metadata only as key:type ('Foot L/Delta:TRANSFORM', 'Spine/Offset:TRANSFORM'), with no values. Part of the construction math must therefore be ported anyway (see missed G1-element-metadata-channel).

Verifier evidence: `docs/spec.md:58`; `README.md:22-23`; `libs/rigExecRigging/rigBuilder.h:1030-1189`; `ue/<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt (metadata listed as key:type only)`

### G1-module-exec-order

**Per-event, depth-first module execution order**

**Verdict:** Divergent-by-design · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D16

UE features: `UE1-module-execution-order`, `UE3-module-metadata-bus`

**UE rigs.** For each event (Construction, Pre Forwards, Forwards, Post Forwards, Backwards, Connector, Interaction, user events), UModularRig queues every module that supports it, in depth-first pre-order of the module tree. It then runs each module VM with its namespace, redirector, config and bindings. Interaction events go only to modules that own an interacted element. Zebra's order is root, Prop, Body, Spine, Leg L, Foot L, ... Face, Ears, Mohawk. The leg/foot coupling depends on these phases: the Foot's Pre Forwards (which drives the leg IK effector) runs before the Leg's Forwards, which runs before the Foot's Forwards.

**usdRig today.** usdRig has no events and no module queue. The pose DAG schedules solvers by data dependency. Constraints and property movers follow the composed Movers namespace in reverse-sibling post-order, deliberately chosen over an explicit list. Constraints that write a solver's input chain are scheduled before that solver, so the Foot-before-Leg staging follows from dependencies. A module tree cannot be mapped to naively nested scopes, because descendants run before their parent: child-module scopes must sit above the parent module's own movers, or the tree must be flattened. Composing module scopes from several sublayers needs 'reorder nameChildren' pins, otherwise the rig fails to compile.

**Gap.** There is no module-aware ordering helper or order validation, and no interaction-event routing (see G3).

**Porting impact.** Constraint order within and across modules must be emitted carefully; the biped hit several ordering bugs of exactly this kind. Once ordered correctly, transform results match.

**Recommendation.** In python/rigexec/modules/assemble.py, emit module scopes under <rig>/Movers in reverse UE pre-order (each child module's scope above its parent's own movers), and always author 'reorder nameChildren' pins for composed module scopes. Add a notice-level check in rigEvaluator.cpp that flags a mover whose rigExec:module:parent module executes after it, and group execStackUI.py rows by RigExecModuleAPI.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:139-163`; `libs/rigExec/rigEvaluator.cpp:3230-3243`; `libs/rigExec/rigEvaluator.cpp:5794-5806`; `docs/spec.md:962`; `docs/biped-rig.md:254-266`; `docs/biped-rig.md:195-225`; `plugin/rigExecUsdview/execStackUI.py:7-31`

**Verification (holds).** The design choice is documented:
- spec.md:962 chooses composed-hierarchy order over explicit lists.
- spec.md:207 defers the event layer.

The mechanics match the row:
- The walk is a reversed pre-order (rigEvaluator.cpp:139-163).
- The pose DAG orders constraints before solvers whose input chains they write (rigEvaluator.cpp:5794-5806; catalog R1-pose-dag).
- The reorder-nameChildren pins are required, or the layered rig fails to compile (biped-rig.md:254-266).

Minor holds once order is emitted correctly.

Verifier evidence: `docs/spec.md:962`; `docs/spec.md:207`; `libs/rigExec/rigEvaluator.cpp:139-163`; `docs/biped-rig.md:254-266`

### G1-structural-config-switch

**Structural config switch selecting a module variant (Spine 'Is Neck')**

**Verdict:** Divergent-by-design · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D16

UE features: `UE2-spine-neck-mode`

**UE rigs.** CRM_FN_Spine's public bool 'Is Neck' changes both construction and solve. As a spine, the module creates End Movable Pivot, Pelvis Local and the Pelvis TXY space, runs Forward Movable Proxy and writes the pelvis. As a neck, the first bone takes the interpolated spline rotation, and Start IK is added to the IK visibility set. Zebra uses the same class twice, once as Spine and once as Neck.

**usdRig today.** A config value that changes topology is structural in usdRig. It maps either to a generator parameter (two differently generated subtrees) or to a USD variantSet on the Spine component; selecting a variant starts a new epoch, as rigComplexity does. SplineIk already has a spine/neck switch, rigExec:rootTangent = rigid|aim, but it changes the curve's root tangent, not which rotation the first bone takes.

**Gap.** No module generator exists to express the two modes. SplineIk has no option to choose between the interpolated rotation and the start-control rotation for the first bone.

**Porting impact.** Spine and Neck become two separately generated subtrees, or one component with variantSet {spine, neck}. Once generated, nothing is lost at runtime.

**Recommendation.** Model the switch as variantSet "moduleMode" = {spine, neck} on the Spine component layer written by python/rigexec/modules/spine.py. Expose the solve-side difference as a SplineIk attribute (for example uniform token rigExec:firstBoneRotation = "startControl"|"curve") in schema.usda and libs/rigExecMath/splineIk.cpp, tracked with the G5 spine gaps.

**Evidence:** `docs/spec.md:354`; `examples/ArmRig.usda:8-13`; `examples/ArmShotAnim.usda:8-16`; `libs/rigExecSchema/schema.usda:887-897`

**Verification (holds).** spec.md:354 is a rule that variant selection is structural, not a non-goal. The divergence (topology-changing config handled by a generator or variant) still follows from spec.md:58 and :335. rigExec:rootTangent rigid|aim is confirmed at schema.usda:887-897.

The solve-side gap is overstated:
- SplineIk already aims every joint, the first included, along the spline (schema.usda:795-798). That is UE's neck mode.
- Spine mode (first bone takes the Start IK rotation) is reachable today with a downstream ParentConstraint from the start control. The biped does exactly this, writing hips_bind from spine_root_ctl (Biped.usda:3042-3043).

The proposed rigExec:firstBoneRotation token is therefore a convenience, not a requirement.

Verifier evidence: `docs/spec.md:354`; `docs/spec.md:58`; `libs/rigExecSchema/schema.usda:795-798`; `libs/rigExecSchema/schema.usda:887-897`; `examples/biped/Biped.usda:3042-3043`

### G1-authoring-quirks

**Face authoring defects to reproduce or fix when porting**

**Verdict:** Not-applicable · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D16

UE features: `UE6-authoring-quirks`

**UE rigs.** There are 12 verified face authoring defects that change results: double writes of wide_open_c_r and lip_roll_in_bt_l (the last writer wins); pose readers that reference nonexistent 'Lip Main Tp/Bt' controls and always output 0; position constraints that target nonexistent 'Brow In/Ot' nulls and do nothing; crossed brow-squeeze L/R outputs; source mix-ups in Zebra ParentConstraint_164/165; Monster writing lid_bt_blink_extend_l twice and never writing _r; orphan nodes; the one-evaluation Jaw Normalize lag; and unused members.

**usdRig today.** This is a porting-policy question, not an engine capability. Two usdRig behaviours matter. An operator whose source or target is a missing prim is a compile error that fails the whole rig, whereas UE silently does nothing. Repeated writes to one float form an ordered FloatMathMover stack in which 'blend' sets the value, so last-writer semantics can be reproduced.

**Gap.** By design there is no 'ignore missing reference' mode.

**Porting impact.** The importer must drop constraints and readers that reference nonexistent elements (in UE their output is 0 or has no effect) and decide per quirk whether to reproduce or fix it. The Jaw Normalize lag cannot be reproduced.

**Recommendation.** Add a quirk table with a reproduce/fix flag per item to the UE importer (python/rigexec/modules/import_ue.py), and constant-fold dead readers to 0. No schema change is needed.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:4966-4970`; `libs/rigExec/rigEvaluator.cpp:3299-3305`; `libs/rigExecMath/propertyMath.cpp:56-60`

**Verification (holds).** This is porting policy. Blend sets r = value (propertyMath.cpp:56-58), so last-writer stacks are reproducible. A missing prim is a hard compile error (rigEvaluator.cpp:3299-3305, 4966-4970), which makes the importer rule mandatory.

Verifier evidence: `libs/rigExecMath/propertyMath.cpp:56-58`; `libs/rigExec/rigEvaluator.cpp:4966-4970`

### G1-connection-list-hygiene

**Connection list resolution: case-insensitive keys, dead entries, legacy maps**

**Verdict:** Not-applicable · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D16

UE features: `UE8-conn-resolution-dead-entries`

**UE rigs.** FindConnectionIndex compares FRigElementKey by FName, which is case-insensitive, so 'Root/RootJoint' matches 'root/RootJoint'. Entries that name connectors which no longer exist (20 in Zebra, 38 in Biped, 11 in Monster) are kept but ignored. ArrayConnectionMap mirrors ConnectionList. PreviousModulePaths records old module names, including modules that have been removed.

**usdRig today.** This is UE bookkeeping. What matters for usdRig: SdfPath and TfToken are case-sensitive, and a dangling relationship target is a compile error rather than an ignored entry.

**Gap.** None in usdRig; this is an importer rule.

**Porting impact.** The importer must match connector keys case-insensitively, discard dead entries, and ignore ArrayConnectionMap and PreviousModulePaths. If it does not, the generated rig fails to compile.

**Recommendation.** Implement the rule in the UE importer (python/rigexec/modules/import_ue.py). If recovery by old name is ever needed, map it to USD relocates, which the composition-arc panel already authors (docs/composition-arcs.md:17).

**Evidence:** `libs/rigExec/rigEvaluator.cpp:3299-3305`; `libs/rigExec/rigEvaluator.cpp:4966-4970`

**Verification (holds).** This is UE bookkeeping and an importer rule. Dangling targets are compile errors in usdRig (rigEvaluator.cpp:3299-3305, 4966-4970), so the importer rule is load-bearing. Relocates can be authored by the arc panel (composition-arcs.md:17).

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:3299-3305`; `libs/rigExec/rigEvaluator.cpp:4966-4970`; `docs/composition-arcs.md:17`

### G1-connector-default-match

**Connector event: default-match suggestions**

**Verdict:** Missing · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D16

UE features: `UE1-connector-event-default-match`, `UE4-default-match-to-connector`, `UE4-connect-to-module-metadata`, `UE2-fkchain-connector-event`, `UE3-ikfk-connectors`, `UE4-unit-module-connector`

**UE rigs.** A module's 'Connector' event runs during candidate resolution. GetCandidates tells it which connector is being resolved, and SetDefaultMatch proposes a default. FkChain defaults End to Start's last recursive bone child and Parent to Start's parent. IkFk2Bones defaults Parent to Start's parent. LimbTwist defaults Parent and a second-child target, and still references the stale connectors 'Start Socket' and 'Clavicle'. CRFL provides 'Set Default Match To Connector v01' (5 call sites) and 'Connect to Module Metadata' (unused).

**usdRig today.** usdRig has no counterpart for suggestion logic at authoring time. It belongs in the offline module library, not in the evaluator.

**Gap.** There is no per-module hook that suggests a default target.

**Porting impact.** No effect on evaluation. Connecting modules by hand loses the auto-suggest UX.

**Recommendation.** Give each generator in python/rigexec/modules/ an optional suggest_default(connector, candidates, resolved) callback: FkChain suggests the last descendant joint and the parent of Start; IkFk2Bones the parent of Start; LimbTwist the parent of Start. moduleAssemblyModel.py calls it to order candidates. Drop LimbTwist's stale connector names.

**Evidence:** `libs/rigExecRigging/rigBuilder.h:1030-1189`; `plugin/rigExecUsdview/compositionArcsModel.py:1-25`

**Verification (holds).** No authoring-time suggestion hook exists; the builder is imperative (rigBuilder.h:1030-1189). This is UX only, with 5 call sites and no evaluation impact, so cosmetic is fair.

Verifier evidence: `libs/rigExecRigging/rigBuilder.h:1030-1189`

### G1-biped-template-extras

**Biped-template-only modules: Stretch Feedback, IK Bone Pins, Attach, ProxyControls**

**Verdict:** Partial · **Severity:** cosmetic · **Effort:** M · **Confidence:** medium · **Domain:** D16

UE features: `UE1-biped-template-extras`

**UE rigs.** The Biped template adds four kinds of module that Zebra drops. Stretch Feedback draws colored debug lines comparing segment length to rest length (the right side is derived by renaming _l to _r), toggled by a 'Stretch FeedBack Vis' bool. IK Bone Pins copies global transforms from Drivers[i] onto the ik_* virtual bones Driven[i]. Attach is an FkArray over the attach bones with override parents. Twelve ProxyControl modules apply an interaction-time delta to several finger and meta controls.

**usdRig today.** The pins map to one ParentConstraint per pair. Attach maps to FK controls with ParentConstraints to the override parents. Stretch feedback has no equivalent, because the rig cannot draw debug lines: guides are synthesized only for controls, joints, solvers and volumes. Proxy controls need an interaction model and delta-offset application, which usdRig lacks (see G3).

**Gap.** There is no debug-line or diagnostic-color drawing and no proxy (delta) control.

**Porting impact.** None for Zebra and Monster, which do not use these modules. They matter only if the Biped template itself is ported.

**Recommendation.** If the template is ported, add a RigExecSegmentStretchGuide diagnostic (rel rigExec:joints, color3f rest/squash/stretch colors, double maxStretch), synthesized in libs/rigExecImaging/sceneIndices.cpp like the solver guides. Proxy controls are tracked in G3.

**Evidence:** `libs/rigExecSchema/schema.usda:1115-1142`; `libs/rigExecImaging/sceneIndices.cpp:641`; `docs/spec.md:207`

**Verification (holds).** Guides are synthesized only for controls, joints, solvers and volumes (sceneIndices.cpp:638-643). emitGuidePoints (schema.usda:2091) can publish frame origins as native points, but not a stretch-coloured segment. Zebra and Monster do not use these modules, so cosmetic.

Verifier evidence: `libs/rigExecImaging/sceneIndices.cpp:638-643`; `libs/rigExecSchema/schema.usda:2091`

### G1-dmc-template-variant

**DMC template variants (extra module + host toggle + shape-library order)**

**Verdict:** Partial · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D16 · *(analyst said Partial / minor)*

UE features: `UE7-dmc-template-variants`, `UE1-zebra-vs-zebradmc`, `UE7-dmc-module-crm-fn-dmc`

**UE rigs.** MR_ZebraDMC and MR_FN_BipedDMC equal their base rigs plus three additions: one CRM_FN_DMC module (parent root, Root connected to bone root, binding Direct_Mesh_Control), a host bool Direct_Mesh_Control=True, and ModularRigGizmoLibrary_DMC in ShapeLibraries. The library order differs: [DMC, Regular] in Zebra and [Regular, DMC] in Biped, and lookup searches later libraries first. Control counts do not change. The only runtime differences are the Mohawk falling back to the 'Default' shape and extra metadata on the root.

**usdRig today.** Adding a module plus a template toggle maps directly to a USD variantSet on the character asset (for example directMeshControl = {off, on}), as rigComplexity already does, or to an extra sublayer in the layer stack. Selecting a variant is structural and triggers a recompile. usdRig has no shape libraries or library search order to vary; guides are six built-in primitives.

**Gap.** There is no control-shape library concept, so the library-order difference has nothing to map to. The DMC feature itself is covered in G1-dmc-shape-layers and G7.

**Porting impact.** MR_Zebra and MR_ZebraDMC collapse into one asset with a variant. The shape-library search order and the Mohawk 'Default' fallback difference cannot be reproduced.

**Recommendation.** Have the importer author variantSet "directMeshControl" on the character asset. If shape libraries are added (G3), give RigExecRoot an ordered rel guide:shapeLibraries with 'last wins' lookup, matching the ControlRigGizmoLibrary search order.

**Evidence:** `examples/ArmRig.usda:8-13`; `docs/spec.md:354`; `examples/biped/Biped_stack.usda:1-70`; `libs/rigExecSchema/schema.usda:167-179`

**Verification (corrected).** The structural part (one extra module plus a template toggle) maps directly to a variantSet, as rigComplexity already does (ArmRig.usda:8-13; spec.md:354).

What cannot be reproduced is the gizmo shape-library search order and the Mohawk 'Default' shape. Both are drawing-only; control, null and bone counts are identical (UE1-zebra-vs-zebradmc). That is cosmetic, not minor.

Verifier evidence: `examples/ArmRig.usda:8-13`; `docs/spec.md:354`; `libs/rigExecSchema/schema.usda:167-179`

### G1-module-identity

**Module identity and library metadata (RigModuleSettings)**

**Verdict:** Partial · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D16

UE features: `UE1-module-settings-identity`

**UE rigs.** RigModuleSettings holds {Identifier(Name, Type), Icon, Category ('Fortnite Modules'), Keywords, Description, ExposedConnectors}. For example, LimbTwist's identifier is 'CRM_Epic_LimbTwist_v02', and the DMC module's description says to add it directly below CRM_FN_Root. Host modular rigs have empty identifiers.

**usdRig today.** Generic USD metadata (assetInfo name/version/identifier, doc, customData) can hold identity and description on a component layer's default prim. usdRig defines no module fields and has no module library browser.

**Gap.** There are no standard identity, category or icon fields, and no browser.

**Porting impact.** None on evaluation.

**Recommendation.** Put identity on RigExecModuleAPI (rigExec:module:class, uniform token rigExec:module:category, uniform token[] rigExec:module:keywords, asset rigExec:module:icon, doc) and use USD assetInfo for the version. Add a 'Module Library' list to the proposed moduleAssemblyUI.py that scans component layers for default prims carrying the API.

**Evidence:** `libs/rigExecSchema/schema.usda:95-111`; `docs/composition-arcs.md:1-30`

**Verification (holds).** Generic USD metadata (assetInfo, doc, customData) can carry identity. No module fields or library browser exist. No evaluation impact.

Verifier evidence: `libs/rigExecSchema/schema.usda:95-111`

### G1-module-namespace-naming

**Module namespace prefixes, short names and display names**

**Verdict:** Partial · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D16

UE features: `UE4-unit-module-connector`, `UE1-module-instances-zebra`

**UE rigs.** A module's name, including its side suffix (for example 'Arm L'), is the namespace prefix of every element it spawns ('Arm L/FK 0'). GetModuleName returns that prefix, GetItemShortName returns the short UI name, and ItemToName converts element keys to names. Duplicate channel names get suffixes ('Sneer Tp_2'). Elements also carry separate display names ('UpperArm FK').

**usdRig today.** The prim path hierarchy provides namespacing (<rig>/Movers/Arm_L/... or <rig>/Controls/Arm_L/...), and USD guarantees unique sibling names. UE names contain spaces and '/', which are not valid in USD identifiers, so they must be sanitized. usdRig defines no display-name attribute. Stock USD has prim displayName metadata, but no RigExec panel reads it; the picker has its own ui:text labels.

**Gap.** There is no display-name convention and no table for sanitizing and mapping names.

**Porting impact.** Control names change ('Arm L/FK 0' becomes Arm_L/FK_0). Animator-facing labels are lost unless displayName is honored, and animation retargeted by UE control name needs a name mapping.

**Recommendation.** Have the importer write the sanitized path, USD displayName metadata ('UpperArm FK') and a customData ueName for round-tripping. Show UsdPrim::GetDisplayName when present in the Avar Editor, Execution Stack and graph editor (plugin/rigExecUsdview/avarEditorModel.py, execStackUI.py, graphModel.py).

**Evidence:** `libs/rigExecRigging/rigBuilder.h:11-23`; `libs/rigExecSchema/schema.usda:2447`

**Verification (holds).** A grep finds GetDisplayName used only for layers (compositionArcsModel.py:508; layerOpinionsModel.py:575-576). No RigExec panel reads a prim's displayName. Picker labels are ui:text (schema.usda:2447). Cosmetic.

Verifier evidence: `plugin/rigExecUsdview/compositionArcsModel.py:508`; `plugin/rigExecUsdview/layerOpinionsModel.py:575-576`; `libs/rigExecSchema/schema.usda:2447`

### G1-negative-side-detect

**Automatic negative-side detection during construction**

**Verdict:** Partial · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D16 · *(analyst said Divergent-by-design / cosmetic)*

UE features: `UE3-negative-side-detect`

**UE rigs.** IkFk2Bones construction sets 'Negative Side' = dot(mid - start, startRot * X) < 0. The flag flips the Soft IK primary/secondary axis signs, the FK aim negation and the PV parent offset sign. It also mirrors FK shapes by Scale(-1,1,1) and chooses the Mid shape rotation and its ±6 Y offset. Foot uses an authored 'Negative Side' instead (true on Foot R).

**usdRig today.** An inference made at construction time belongs in the offline generator. TwoBoneIk has no primary/secondary axis attributes to flip, because it solves from joint rest landmarks. Guide shapes cannot be mirrored with a negative scale, since any non-positive guide:scale draws nothing, and there is no guide offset to flip.

**Gap.** There is no side-inference helper, no mirrored guide rendering and no guide offset.

**Porting impact.** Right-side limbs import correctly only if the generator performs the inference. Asymmetric or offset control shapes on right-side limbs cannot be mirrored visually, which is cosmetic.

**Recommendation.** Add rigexec.modules.common.negative_side(stage, joints) in python/rigexec/modules/. For guides, add uniform bool guide:mirrorX (or accept a signed guide:scaleX and use abs() for sizing) and guide:offsetX/Y/Z in schema.usda, handled in libs/rigExecImaging/sceneIndices.cpp and in the registry.cpp bounds. Track this with the G3 control-shape gaps.

**Evidence:** `libs/rigExecSchema/schema.usda:524-597`; `libs/rigExecSchema/schema.usda:201-209`; `docs/biped-rig.md:181-184`

**Verification (corrected).** None of the cited lines is a non-goal:
- schema.usda:524-597 is the TwoBoneIk schema.
- schema.usda:201-209 is guide:scale.
- biped-rig.md:181-184 is a to-do note about a missing guide offset.

The solve needs no side flag. TwoBoneIk builds its element frames from positions, and joints follow by rest-to-pose delta, so UE's axis-sign flips have nothing to flip.

What is missing is visual: a non-positive guide multiplier draws nothing (bridge.cpp:965-985; schema.usda:201-209), and there is no guide offset. The behavior is partly expressible and only the shape mirroring is lost, so Partial and cosmetic.

Verifier evidence: `libs/rigExecSchema/schema.usda:524-597`; `libs/rigExecSchema/schema.usda:201-209`; `libs/rigExecImaging/bridge.cpp:965-985`; `docs/biped-rig.md:181-184`

### G1-rig-wide-settings

**Rig-wide control scale, side colors and root control keys published by the Root module**

**Verdict:** Partial · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D16

UE features: `UE4-root-global-metadata`, `UE2-root-published-metadata`

**UE rigs.** CRM_FN_Root publishes module metadata in the Root namespace: 'Global Control Scale' (float, 1.1 in Zebra), 'Global Left/Right/Center Control Color' (blue/red/yellow), and the element keys 'Global Control', 'Local Control' and 'Root Control'. Body adds 'Body Control'. The values are stored on the root/RootJoint connector. Other modules read them, with defaults of 1.0 and yellow: Set Control Scale uses the global scale in 9 modules, side colors apply wherever a module's color is white or grey, and Spine adds Root Control as a space.

**usdRig today.** Side colors: guide:displayColor is connectable, so every left control can connect to one custom color3f on the root, and the imaging bridge follows the connection at evaluation time. The biped instead authors a color on each control. Global scale: there is no rig-wide guide multiplier; guide size is |evaluated frame axis| times the per-control guide:scale. Element keys: the generator writes relationships to the root's controls.

**Gap.** There is no rig-level guide scale and no named side-color palette; colors work only by wiring each control.

**Porting impact.** The importer multiplies Global Control Scale into each guide:scale and either bakes or connects the side colors. Changing the global scale later requires regeneration.

**Recommendation.** Add uniform double guide:globalScale = 1 and color3f guide:leftColor/rightColor/centerColor to RigExecRoot in schema.usda. Have the imaging code (registry.cpp bounds, sceneIndices.cpp synthesis) multiply globalScale into every control guide. Add uniform token guide:side = "none"|"left"|"right"|"center" to RigExecControl to pick the palette color when guide:displayColor is unauthored.

**Evidence:** `libs/rigExecSchema/schema.usda:216-222`; `libs/rigExecImaging/bridge.cpp:196-217`; `libs/rigExecImaging/bridge.cpp:251-258`; `libs/rigExecSchema/schema.usda:201-209`; `examples/biped/Biped.usda:4541-4545`

**Verification (holds).** Confirmed:
- guide:displayColor is documented as connectable (schema.usda:216-222) and followed at read time (bridge.cpp:251-258).
- guide:scaleX/Y/Z are read with a plain Get (bridge.cpp:965-971), so no rig-wide multiplier can be wired.

These are drawing-only differences, so cosmetic is right.

Verifier evidence: `libs/rigExecSchema/schema.usda:216-222`; `libs/rigExecImaging/bridge.cpp:251-258`; `libs/rigExecImaging/bridge.cpp:965-971`

### G1-asset-settings-limits

**Asset-level settings: AssetVariant tags, validator, multiple instances, procedural element limit**

**Verdict:** Not-applicable · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D16

UE features: `UE8-asset-variant-validator-limits`

**UE rigs.** The asset-level settings are: AssetVariant tags ('Stable', and 'AnimatorKit_Utility' with bAllowMultipleInstances on CRU_PropAim); a ControlRigValidator with no passes; bAutoResolve=False on all 8 modular rigs; and ProceduralElementLimit = 2000 + the static element count (Zebra 3486, against 2215 elements at runtime).

**usdRig today.** This is UE asset bookkeeping. usdRig needs no procedural element cap, because nothing is spawned during evaluation. Multiple instances of one rig asset are ordinary references, each its own RigExecRoot, and this is tested. Rigs hosted in prototypes (instanceable) are rejected.

**Gap.** None here. A utility rig used through the constraint system (CRU_PropAim) would need cross-rig constraints, which v1 rejects.

**Porting impact.** None for Zebra and Monster. On-demand utility rigs in the style of CRU_PropAim cannot constrain another character's elements.

**Recommendation.** No change here. Cross-character utility rigs depend on the multipass in spec §6.2 (see G1-boombox-prop-rig).

**Evidence:** `tests/testRigExecImaging.cpp:2716-2740`; `libs/rigExecImaging/registry.cpp:293-311`; `libs/rigExec/rigEvaluator.cpp:2959-2965`; `docs/spec.md:2169`

**Verification (holds).** Confirmed:
- Two references to one rig asset activate independently (testRigExecImaging.cpp:2716-2740).
- The registry discovers every RigExecRoot (registry.cpp:293-311).
- Prototype-hosted rigs are rejected (rigEvaluator.cpp:2959-2965).

Nothing is spawned, so no element cap is needed.

Verifier evidence: `tests/testRigExecImaging.cpp:2716-2740`; `libs/rigExecImaging/registry.cpp:293-311`; `libs/rigExec/rigEvaluator.cpp:2959-2965`

### G1-module-asset-kinds

**Blueprint-class modules vs runtime-asset modules**

**Verdict:** Not-applicable · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D16

UE features: `UE1-module-asset-kinds`

**UE rigs.** A module reference is either a Blueprint-generated class, whose variables are UProperties, or a ControlRigRuntimeAsset: a plain UControlRig whose variables live in a transient PropertyBag. The Zebra and Monster Face modules, CRM_FN_DMC and MR_Boombox's host are runtime assets.

**usdRig today.** This is UE class and asset plumbing. In usdRig every module would be the same kind of thing: a component layer referenced by a prim.

**Gap.** None.

**Porting impact.** The importer must read config from both UProperty diffs and PropertyBag dumps (the Face PropertyBag in asset.t3d).

**Recommendation.** No engine change; handle both sources in the UE importer.

**Evidence:** `examples/spider_legs_assembly_ref.usda:14-101`; `examples/ArmShotAnim.usda:8-16`

**Verification (holds).** This is UE class and asset plumbing. In usdRig every component is the same kind of thing: a referenced layer (spider_legs_assembly_ref.usda:14-101).

Verifier evidence: `examples/spider_legs_assembly_ref.usda:14-101`

### G1-supported-events

**Saved SupportedEventNames vs runtime event union (layered picker, bake-to-control-rig)**

**Verdict:** Not-applicable · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D16

UE features: `UE8-asset-supported-events`

**UE rigs.** UModularRig rebuilds SupportedEvents from its modules. MR_Zebra's saved SupportedEventNames tag, [Construction, Forwards Solve], is stale: it hides MR_Zebra from Sequencer's 'Add Layered Control Rig' picker, which requires Backwards Solve or Inverse, while bake still works at runtime. Among module events, IkFk2Bones adds To IK, To FK, Key Controls, Connector and Post Forwards, and Foot adds Pre Forwards and Key Controls. The face modules have no Backwards Solve, so face controls are never inverted by 'Bake to Control Rig'.

**usdRig today.** usdRig has no event model and no asset-registry tags. Baking animation onto controls maps to the Python numeric inverse (solve_parameters), which works on channel subsets chosen by the caller rather than on per-module Backwards Solve graphs.

**Gap.** Nothing to add for the stale tag. Per-module inverse (bake-to-controls) is a separate gap in the inverse group.

**Porting impact.** The layered-rig picker issue does not arise. Baking animation onto the Zebra body controls needs the inverse tooling tracked elsewhere.

**Recommendation.** No change for the tag. Track per-module inverse recipes (the Backwards Solve equivalents) with the inverse/bake gap in python/rigexec/inverse.py.

**Evidence:** `docs/python-bake-inverse.md:34-74`; `docs/spec.md:207`

**Verification (holds).** usdRig has no event model (spec.md:207). The inverse is the generic LM solver (python/rigexec/inverse.py:44; python-bake-inverse.md:34-74). The stale tag has no counterpart.

Verifier evidence: `docs/spec.md:207`; `python/rigexec/inverse.py:44`; `docs/python-bake-inverse.md:34-74`

## G2 — Rig structure, construction, libraries and runtime integration

G2 covers how the UE rigs are structured and run: the rig element model, construction-time procedural spawning, function libraries, naming, and runtime integration (post-process ABP, IK rig, retargeter, physics, LOD, sequencer, USD export). usdRig's element model (RigExecJoint, RigExecControl, relationships and connections under one RigExecRoot) can carry the static result of the UE construction graphs. What it lacks are null/socket, curve-set and typed-channel element kinds. Nesting under a solver-posed joint also does not propagate (schema.usda:484-489), so sockets, helper bones and deformer frames under the head or foot need explicit constraints with authored offsets. Procedural construction, RigVM control flow, item metadata and in-rig string ops are divergent by design (spec.md:58-60). They move to offline Python generators, which do not exist in the repo yet (tools/biped is absent), so the port needs a rebuildable generator layer, a skeleton/UsdSkel importer and UE-compatible naming helpers. The most serious runtime gap is layering. A second rig cannot write the first rig's joints (rigEvaluator.cpp:3308-3313, registry.cpp:244-251). Inside one rig, pose-derived values reach only geometry chains, never transform movers (rigEvaluator.cpp:11666-11673). CR_Zebra_Deform's pose-driven helper-bone pass therefore has nowhere to run, which blocks that asset. Pose-driven correctives that only feed blend shapes, and the morph-then-skin order, do port. Sequencer animation maps well onto double avars with per-control rotation order in a shot layer. Space keys, enum/bool channel semantics, rotation-order re-keying and an importer are missing. IK Rig, retargeter and physics have no counterpart, but the shipped retargeter cannot run and physics is unused by the animation path, so their severity is low. UsdSkel data (blend shapes, skin layout) is directly consumable, but no converter exists because UsdSkel bridges are a stated non-goal.

| Row | Verdict | Severity | Effort | Summary |
|---|---|---|---|---|
| [`G2-deformer-function-libraries`](#g2-deformer-function-libraries) | Missing | major | L | No shared deform-function library; frozen 13-op RigExecRevisionOp set has no bend/twist/squash kernels (G7) |
| [`G2-mirror-metadata`](#g2-mirror-metadata) | Missing | major | M | No per-control mirror axis, L/R counterpart link or pose mirror/flip tool; RigExecControlAPI defers mirror metadata |
| [`G2-control-type-census`](#g2-control-type-census) | Partial | major | M | RigExecControl is always a full 9-channel transform; no rotator/bool/int/enum control types; channelRole is inert |
| [`G2-postprocess-pass`](#g2-postprocess-pass) | Partial | major | L | No post-pose transform phase, so pose-driven helper-bone offsets cannot run; RigExecPose corrective weights can |
| [`G2-runtime-layering`](#g2-runtime-layering) | Partial | major | M | Pose DAG, pose interpolators, then blend-before-skin order maps; helper-offset pass and post-skin deformers missing |
| [`G2-sequencer-channels`](#g2-sequencer-channels) | Partial | major | L | Avars tx..sz + avars:rotationOrder map the 9 channels; no typed enum/bool/int channels, UE importer or space-switch keys |
| [`G2-skeleton-import`](#g2-skeleton-import) | Partial | major | M | No skeleton/UsdSkel/FBX importer or resync; joints must be authored as nested RigExecJoint rest:space via AddJoint |
| [`G2-construction-event`](#g2-construction-event) | Divergent-by-design | major | XL | OpenExec cannot spawn prims; construction becomes offline RigExecRigBuilder authoring with no rebuild-on-change |
| [`G2-stateful-interaction`](#g2-stateful-interaction) | Divergent-by-design | major | M | No interaction signal, rig-requested auto-key or persistent state by design; interaction is tool-side only |
| [`G2-usd-export-import`](#g2-usd-export-import) | Divergent-by-design | major | M | UsdSkel bridge is a non-goal: no UsdSkel-to-RigExecJoint converter or SkelAnimation export; blend/skin data reads direct |
| [`G2-import-data-gaps`](#g2-import-data-gaps) | Not-applicable | major | S | Not applicable: truncated face arrays and missing sequencer keys are source-data gaps any importer must fill |
| [`G2-dmc-shape-resolution`](#g2-dmc-shape-resolution) | Missing | minor | M | No shape library or layered lookup; guide:shape is a fixed six-token enum with no custom-shape relationship |
| [`G2-ik-rig`](#g2-ik-rig) | Missing | minor | XL | No full-body IK, per-joint stiffness/locks or retarget chains; only TwoBoneIk, SingleChainIkConstraint and SplineIk |
| [`G2-manny-reference-skeleton`](#g2-manny-reference-skeleton) | Missing | minor | S | No Manny-layout test skeleton or oracle test against MR_FN_Biped; only the 252-joint converted biped exists |
| [`G2-retargeter`](#g2-retargeter) | Missing | minor | L | No live or offline retargeting; cross-rig writes are rejected, so a source skeleton must be constrained in-asset |
| [`G2-stock-modules`](#g2-stock-modules) | Missing | minor | S | No stock Root/AddControl components; build from RigExecControl plus RigExecParentConstraint with per-source offsets |
| [`G2-baked-anim-input`](#g2-baked-anim-input) | Partial | minor | M | Joints can play baked avars or posed:space, but no AnimSequence importer; solver-claimed joints need a variant swap |
| [`G2-curve-bus-layering`](#g2-curve-bus-layering) | Partial | minor | M | Float attrs + FloatMathMover act as the bus; double avars feeding float movers fail silently; no deformer consumers |
| [`G2-curve-channels`](#g2-curve-channels) | Partial | minor | M | No curve element, per-curve morph/material flags or name binding; floats bind by explicit connection, not to materials |
| [`G2-deformer-child-components`](#g2-deformer-child-components) | Partial | minor | S | No attached-mesh fan-out; only BlendShapeMover takes multiple targets, Skin/Smooth/Lattice movers are single-target |
| [`G2-dmc-editor-scope`](#g2-dmc-editor-scope) | Partial | minor | M | TouchPose regions also keep picking out of evaluation, but only select the bound control; no on-surface drag (G7) |
| [`G2-function-libraries`](#g2-function-libraries) | Partial | minor | L | Reuse is USD composition (references, sublayers, variants); no versioned library of rig-construction functions |
| [`G2-mesh-runtime-binding`](#g2-mesh-runtime-binding) | Partial | minor | S | Meshes bind via rigExec:moves targets; SkinMover takes one mesh, so each resolution mesh needs its own movers |
| [`G2-module-namespacing`](#g2-module-namespacing) | Partial | minor | S | Prim-path scopes give unique names, but no control display names, stored UE names or case-insensitive lookup |
| [`G2-multi-character-shot`](#g2-multi-character-shot) | Partial | minor | L | Rigs evaluate serially and isolated; cross-rig constraint sources silently read the other rig's unevaluated xforms |
| [`G2-null-insertion-helpers`](#g2-null-insertion-helpers) | Partial | minor | S | Builder lacks insert-above/below, control-stack and maintain-global helpers; offsets via default:space or nested prims |
| [`G2-rig-internal-bones`](#g2-rig-internal-bones) | Partial | minor | S | Internal bones map to non-influence RigExecJoints, but every joint is published; no internal flag or guide offset |
| [`G2-socket-null-elements`](#g2-socket-null-elements) | Partial | minor | M | No socket/null type, tags or mesh-socket import; nested RigExecJoint follows solver-posed bones, plain Xforms do not |
| [`G2-xform-null-constraint-read`](#g2-xform-null-constraint-read) | Partial | minor | S | Plain UsdGeomXform under a solver-posed joint is a silently stale constraint source; use a nested RigExecJoint |
| [`G2-construction-queries`](#g2-construction-queries) | Divergent-by-design | minor | S | Hierarchy queries belong in the offline builder; lists are static rels and guide:scaleX/Y/Z is not chain-derived |
| [`G2-item-metadata`](#g2-item-metadata) | Divergent-by-design | minor | S | No tags or runtime metadata by design; element pointers become builder-authored static relationships |
| [`G2-naming-helpers`](#g2-naming-helpers) | Divergent-by-design | minor | S | No in-rig string ops by design and no offline naming library ships (tools/biped/rig_names.py is absent) |
| [`G2-physics`](#g2-physics) | Divergent-by-design | minor | S | No physics bodies or constraints by design (simulation is a non-goal); static UsdPhysics data could serve other tools |
| [`G2-rigvm-control-flow`](#g2-rigvm-control-flow) | Divergent-by-design | minor | S | No node graph or enum switch; branches map to inputs:enabled, weights, clamp/remap; solvers lack inputs:enabled |
| [`G2-seq-embedded-snapshot`](#g2-seq-embedded-snapshot) | Divergent-by-design | minor | S | Shots reference the rig live so nothing goes stale, but renamed controls silently orphan shot overs (no detector) |
| [`G2-bone-axis-convention`](#g2-bone-axis-convention) | Implemented | minor | S | Maps to parent-relative orthonormal rest:space matrices (no Euler aliasing); a UE-to-USD basis helper is still needed |
| [`G2-deformer-frame-nulls`](#g2-deformer-frame-nulls) | Implemented | cosmetic | S | Maps to static nested RigExecJoint (purpose guide) under the head; follows spline IK with no constraint needed |
| [`G2-helper-joint-layout`](#g2-helper-joint-layout) | Implemented | cosmetic | S | Maps to nested leaf RigExecJoints with parent-relative rest; they follow solver-posed parents with no extra drivers |
| [`G2-lod`](#g2-lod) | Implemented | cosmetic | S | Maps to rigComplexity variant with inputs:enabled overrides (switch starts a new epoch); no automatic distance LOD |
| [`G2-reuse-composition`](#g2-reuse-composition) | Implemented | cosmetic | S | Maps to USD references and sublayers (right side references left); relationships must stay inside the arc |
| [`G2-seq-blend-layers`](#g2-seq-blend-layers) | Implemented | cosmetic | S | Maps to one resolved USD opinion per channel (absolute, weight 1); no additive/weighted layers or masks exist |
| [`G2-seq-nonrig-tracks`](#g2-seq-nonrig-tracks) | Implemented | cosmetic | S | Maps to plain USD (xformOps, UsdGeomCamera, UsdLux, UsdShade samples); no importer, spawn maps only to visibility |
| [`G2-asset-hygiene`](#g2-asset-hygiene) | Not-applicable | cosmetic | S | Not applicable: engine plugin/asset bookkeeping; USD asset paths or resolver remaps cover the redirectors |
| [`G2-procedural-element-limit`](#g2-procedural-element-limit) | Not-applicable | cosmetic | S | Not applicable: nothing spawns during evaluation, so element count is the composed stage, validated at compile |

### G2-deformer-function-libraries

**Shared deformer function / source libraries (DG_Function_*, DSL_*)**

**Verdict:** Missing · **Severity:** major · **Effort:** L · **Confidence:** high · **Domain:** D17

UE features: `UE7-dg-shared-function-library`

**UE rigs.** All 17 deformer graphs reference engine function assets (DG_Function_Bend, Twist, SquashStretch, ComputeNormalsTangentsAndKeepInputNormals) and HLSL source libraries (DSL_Matrix, DSL_Quaternion). None authors custom deformation math apart from a CacheGeometry copy. The engine folder also ships Flare, LatticeDeform, BlendPositions, and LBS/DQS functions.

**usdRig today.** Geometry kernels are a frozen operation set with no runtime dispatch and no user-kernel extension point. Bend, twist, squash and flare kernels do not exist. The closest analogues are the lattice mover, skinning and derived normals.

**Gap.** No shared deform-function library and no way to add one without engine changes.

**Porting impact.** The 4 engine functions the rigs use must be added as native movers (G7 carries the per-deformer detail).

**Recommendation.** Add Bend, Twist and SquashStretch as native kernels in libs/rigExecMath/geometryKernels.cpp with new RigExecRevisionOp entries (moverGraph.h), built on one shared 'deform frame' helper (origin matrix, local +Z axis, low/high bounds) so the three share code and baked steps. Longer term, add a declared-signature native kernel registry (the spec's 'user native kernels').

**Evidence:** `libs/rigExec/moverGraph.h:59-73`; `libs/rigExec/moverGraph.cpp:193-196`; `libs/rigExecSchema/schema.usda:2139-2160`

**Verification (holds).** Confirmed: the RigExecRevisionOp enum is a frozen 13-op set (moverGraph.h:59-73) with no runtime dispatch (moverGraph.cpp:193-196), and grep finds no bend, twist, squash or flare kernel. This duplicates G7; count it once.

Verifier evidence: `libs/rigExec/moverGraph.h:59-73`; `libs/rigExec/moverGraph.cpp:188-198`; `libs/rigExecSchema/schema.usda:2139-2160`

### G2-mirror-metadata

**Per-control mirror axis / behavioral mirror tags**

**Verdict:** Missing · **Severity:** major · **Effort:** M · **Confidence:** high · **Domain:** D1

UE features: `UE4-mirror-metadata`

**UE rigs.** Set Mirror Axis writes item metadata 'Mirror Axis' (vector, library default (0,1,1)) and Set Mirror Behavior writes 'Mirror Behavioral' (bool), both with NameSpace=None. Nothing in the rig computes mirroring; external pose mirror/flip tools read the tags. There are 10 behavior sites and 3 axis sites across Foot, IkFk2Bones, LimbTwist, FkArray and FkChain.

**usdRig today.** RigExecControlAPI explicitly defers mirroring metadata. Side mirroring exists only as composition of rig data (the right side references its left twin), not as pose mirroring, and no counterpart relationship or mirror tool exists.

**Gap.** No per-control mirror axis or behavioral flag, no L/R counterpart link, and no pose mirror/flip tool.

**Porting impact.** Animators lose pose mirror and flip on the tagged FK and IK controls. Face asymmetry workflows lose it too.

**Recommendation.** Add to RigExecControlAPI: uniform float3 rigExec:mirror:axis = (0,1,1), uniform bool rigExec:mirror:behavioral, and rel rigExec:mirror:counterpart. Implement plugin/rigExecUsdview/mirrorPoseModel.py (Qt-free, like avarEditorModel) that mirrors or flips avars through the counterpart's rest frames on the shared undo stack. Add a builder helper that pairs counterparts by side token.

**Evidence:** `libs/rigExecSchema/schema.usda:39-41`; `docs/spec.md:262`; `docs/biped-rig.md:243-259`

**Verification (holds).** Confirmed.
- The ControlAPI doc defers mirroring metadata (schema.usda:39-41; spec.md:262).
- grep for 'mirror' in plugin/, python/ and tools/ finds only math and comment uses, with no pose mirror or flip tool.
- spec.md:1050 mentions 'an authored mirror reflectionAxis' for controls and joints, but no such attribute exists. The only reflectionAxis is an internal decomposition parameter (pointFrame.h:106).
- Biped side mirroring is rig-data composition only (docs/biped-rig.md:243-259).

With no pose mirror or flip at all, major is justified.

Verifier evidence: `libs/rigExecSchema/schema.usda:39-41`; `docs/spec.md:262`; `docs/spec.md:1050`; `libs/rigExecMath/pointFrame.h:106`; `docs/biped-rig.md:243-259`

### G2-control-type-census

**Control value types and animation roles at MR_Zebra scale**

**Verdict:** Partial · **Severity:** major · **Effort:** M · **Confidence:** high · **Domain:** D1

UE features: `UE1-runtime-hierarchy-census`

**UE rigs.** After construction, MR_Zebra has 359 controls: 221 EulerTransform, 14 Rotator and 4 Position animation controls, plus 69 bool, 32 float, 14 scale-float and 1 integer channels and 4 proxy controls. It also has 326 nulls, 423 bones (52 procedural) and 958 curves. MR_Monster has 177 controls, 85 nulls and 202 bones. MR_FN_Biped has 283 controls. A per-module breakdown is known (e.g. Face 107/32/31).

**usdRig today.** RigExecControl is always a full transform provider (tx..rz, rspin, sx..sz avars, 6 rotation orders). Rotator-only and position-only controls are a convention with no channel locking. Bool, float and int channels are untyped custom attributes found by name prefix, and proxy controls do not exist. rigExec:channelRole is declared but read by no evaluator code. Scale reference: the biped (126 controls, 252 joints) runs baked at 0.71 ms per animated frame and 112-139 ms per compile, and discovery costs about 14 µs per prim under the root.

**Gap.** Control value types (rotator/position/scale-float/bool/integer/enum) and animation roles (control vs channel vs proxy) are not modeled, and channelRole is inert.

**Porting impact.** The 120 channels become untyped customs with no ranges. Rotator and position controls expose extra keyable channels. Proxy selection and keying behavior is lost (see G3). A Zebra-size rig is roughly 2-3 times the biped's prim count, which is still acceptable for compile time.

**Recommendation.** Extend RigExecControlAPI (schema.usda:33-47) with uniform token rigExec:controlType {transform, eulerTransform, rotator, position, scale, float, scaleFloat, bool, integer} and uniform token rigExec:animationType {control, channel, proxy}. Add a multiple-apply RigExecChannelAPI declaring typed channel attributes (default, min, max, enum labels). Make plugin/rigExecUsdview avarEditorModel.py, the gizmo and the graph editor honor them (hide or lock unused axes). Coordinate with G3.

**Evidence:** `libs/rigExecSchema/schema.usda:144-166`; `libs/rigExecSchema/schema.usda:327-336`; `libs/rigExecSchema/schema.usda:33-47`; `plugin/rigExecUsdview/avarEditorModel.py:14-54`; `libs/rigExecRigging/rigBuilder.cpp:2602-2616`; `docs/plans/evaluation-engine-gaps-vs-premo-libee.md:74-78`; `examples/biped/README.md:226-229`

**Verification (holds).** Confirmed:
- RigExecControl is always a full transform, and RigExecControlAPI declares only channelRole (schema.usda:33-47).
- channelRole is only authored (rigBuilder.cpp:384; python/rigexec/__init__.py:381,483). No evaluator or plugin reads it (grep).
- Custom channels are found by name prefix (avarEditorModel.py:14-54).
- The scale figures check out: Biped.usda has 126 RigExecControl and 252 RigExecJoint defs (grep count), baked 0.71 ms/frame and compile 112-139 ms (docs/plans...:74-78), and about 14 µs per prim for discovery (examples/biped/README.md:226-229).

This overlaps G3.

Verifier evidence: `libs/rigExecSchema/schema.usda:33-47`; `libs/rigExecRigging/rigBuilder.cpp:384`; `python/rigexec/__init__.py:381`; `plugin/rigExecUsdview/avarEditorModel.py:14-54`; `docs/plans/evaluation-engine-gaps-vs-premo-libee.md:74-78`; `examples/biped/README.md:226-229`; `examples/biped/Biped.usda (126 RigExecControl / 252 RigExecJoint)`

### G2-postprocess-pass

**Post-process control rig pass on the final pose (AnimBP ControlRig node, control-less deform rig)**

**Verdict:** Partial · **Severity:** major · **Effort:** L · **Confidence:** high · **Domain:** D18 · *(analyst said Partial / blocker)*

UE features: `UE-postprocess-abp`, `UE-deform-runtime-asset`

**UE rigs.** SKM_Zebra, SKM_Zebra_Hi and SKM_Monster run a post-process AnimBP whose only node is ControlRig(CR_*_Deform): Alpha 1, all LODs, reset input pose to initial, transfer pose and curves. The control-less rig takes whatever pose and curves arrive (AnimSequence, sequencer MR_Zebra, retargeter). It recomputes 50 corrective curves (overwriting incoming values), applies 82 AdditiveLocal helper/twist-bone offsets driven by 45 pose readers, and enqueues 7 deformers after the main instance on every evaluation.

**usdRig today.** Layering must happen inside one RigExecRoot, because another rig cannot write this rig's joints and two rigs cannot publish one prim. Inside a rig the phases are fixed: property chains, then the pose walk (solvers + constraints), then pose interpolators (reading the final pose), then geometry chains. Pose-driven corrective curves therefore work post-pose (RigExecPose weights feeding BlendInputs). Pose-driven TRANSFORM offsets cannot run after the pose walk: property movers resolve before exec, constraints cannot read pose-interpolator outputs, and no post-pose transform phase exists. For input-pose independence, joints can be animated through avars or posed:space samples once the animator solvers are removed by a (structural) variant.

**Gap.** No post-pose transform phase, no path from pose-derived scalars into transform movers, no 'overwrite incoming curve' semantics, and no per-mesh attachable post-process rig asset.

**Porting impact.** The helper-bone pass of CR_Zebra_Deform (82 offsets) and CR_Monster_Deform (4 offsets) cannot be built at all. Only its curve-to-morph correctives port, so shoulder, elbow, knee and hip volume preservation is lost.

**Recommendation.** Add a post-pose transform phase: movers under <rig>/PostPose (or RigExecMoverAPI uniform token rigExec:phase = 'postPose') run after _EvaluatePoseInterpolators and before geometry chains. They may read RigExecPose.outputs:weight and future pose-reader outputs and write RigExecJoint frames. Add RigExecOffsetTransformMover (additive local translate/rotate/scale * weight, the ModifyTransforms AdditiveLocal equivalent); skin movers read it with rigExec:transformReadPhase=final. Files: schema.usda, rigEvaluator.cpp (phase ordering and dependency validation near 11666), bakedPose.cpp (new step), rigBuilder.h/python bindings. A corrective layer can then be sublayered into the character's RigExecRoot as its 'post-process rig'.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:3296-3314`; `libs/rigExecImaging/registry.cpp:244-251`; `libs/rigExec/rigEvaluator.cpp:11666-11673`; `libs/rigExec/rigEvaluator.cpp:11724-11736`; `libs/rigExec/rigEvaluator.cpp:2710-2722`; `libs/rigExec/computations.cpp:345-380`; `docs/spec.md:354`

**Verification (corrected).** The phase analysis is correct:
- property chains resolve before exec;
- pose interpolators run after the whole pose walk and before geometry (rigEvaluator.cpp:11666-11673, asserted at 11724-11756);
- no post-pose transform phase exists.
It is also subtler than stated: compile rejects only connections authored ON outputs:weight (rigEvaluator.cpp:2578-2585). A constraint or solver input connected to a pose weight is not rejected and silently reads the authored value.

Two parts are overstated:
- 'No overwrite-incoming-curve semantics' is wrong. A connected input outranks its own animated value (moverGraph.h:314-378), so a BlendInput weight connected to RigExecPose.outputs:weight overwrites any baked curve value, which is exactly UE's behavior.
- Blocker is too strong. The animator rig and skin still work; what is lost is helper-bone volume preservation (a visible deformation-quality loss, i.e. major).

Partial workarounds exist:
- bake the UE helper-pass deltas into corrective blend shapes driven by PoseInterpolators before the SkinMover;
- geometric volume tricks with position constraints between nested locators.
Neither is exact.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:11666-11673`; `libs/rigExec/rigEvaluator.cpp:11724-11756`; `libs/rigExec/rigEvaluator.cpp:2578-2585`; `libs/rigExec/rigEvaluator.cpp:2710-2722`; `libs/rigExec/moverGraph.h:314-378`; `libs/rigExec/rigEvaluator.cpp:3296-3314`; `libs/rigExecImaging/registry.cpp:240-251`

### G2-runtime-layering

**Per-frame evaluation layering: animator rig, correctives, morphs, skin, deformers**

**Verdict:** Partial · **Severity:** major · **Effort:** M · **Confidence:** high · **Domain:** D18

UE features: `UE-runtime-layering`, `UE7-zebra-deform-postprocess-chain`

**UE rigs.** Per frame, UE evaluates in this order:
1. Main pose: baked anim, retargeter, or sequencer MR_Zebra (including the face curves).
2. Post-process CR_Zebra_Deform: Sequence A runs correctives and helper offsets, Sequence B enqueues 7 deformers.
3. Skinning with the modified helpers, plus morph targets weighted by same-named curves.
4. Optimus deformers after default skinning, in group-1 enqueue order.
Correctives are invisible to the animator and a pure function of the pose.

**usdRig today.** Order is explicit and deterministic: pose DAG, then pose interpolators, then geometry chains in reverse-sibling mover order. A BlendShapeMover placed before the SkinMover gives morph-then-skin, because skin treats its incoming revision as the bind pose. Movers placed after the skin could host post-skin deformers. Correctives are pure (RigExecPose). Missing: the helper-offset post pass (G2-postprocess-pass), post-skin parametric deformers (G7), and any GPU path (CPU only).

**Gap.** Two of the four UE layers (helper offsets, GPU deformers) have no equivalent, and all deformation is CPU-only.

**Porting impact.** The ordering ports as-is. Deformation quality depends on the post-pose phase and on G7's deformers. CPU cost on the 132k-vertex Zebra_Hi is untested.

**Recommendation.** Document and validate a canonical per-character stack: Movers/Pose (animator constraints), PostPose (new), and Movers/Geometry/<mesh> holding [Deformers, Skin, Shapes] in reverse-sibling order. Add a compile check that post-skin deformers read final frames. Profile Zebra-size meshes (34k and 132k vertices) with rigExecPose --profile.

**Evidence:** `docs/spec.md:314-321`; `libs/rigExec/rigEvaluator.cpp:11666-11673`; `libs/rigExecSchema/schema.usda:1686-1707`; `libs/rigExecSchema/schema.usda:1737-1753`; `libs/rigExec/moverGraph.h:59-73`

**Verification (holds).** Confirmed:
- ordering is reverse-sibling post-order (spec.md:314-321);
- pose interpolators run before geometry (rigEvaluator.cpp:11666-11673);
- the skin treats its incoming revision as the bind pose, so blend shapes placed before the skin work.
The missing layers overlap G2-postprocess-pass and G7. CPU-only is the design (spec.md:1398 per catalog), not a defect.

Verifier evidence: `docs/spec.md:314-321`; `libs/rigExec/rigEvaluator.cpp:11666-11673`; `libs/rigExecSchema/schema.usda:1681-1707`; `libs/rigExec/moverGraph.h:59-73`

### G2-sequencer-channels

**Sequencer Control Rig tracks: per-control channel layout and rotation orders**

**Verdict:** Partial · **Severity:** major · **Effort:** L · **Confidence:** high · **Domain:** D18

UE features: `UE-level-sequences`, `UE8-seq-cr-channel-layout`

**UE rigs.** Characters are animated through MovieSceneControlRigParameterTrack: MR_Zebra in 3 sequences and MR_Boombox in 1, each with one section. Channels are built per animatable control by type:
- Transform: 9 Euler float channels (translation; rotation in degrees read in a per-control EEulerRotationOrder; scale).
- Rotator/position: X/Y/Z.
- Float and scale-float: scalar. Bool: bool channel. Integer with enum: byte channel.
ControlChannelMap is keyed by full control FName. ControlsRotationOrder is per control (YZX 261, XYZ 82, ...). Space channels (Parent/World/ControlRig) exist. Actors also have Transform and Spawn tracks. Key data was not dumped.

**usdRig today.** A shot layer that references the asset authors avars: tx/ty/tz, rx/ry/rz in degrees with a per-control avars:rotationOrder (6 orders), and sx/sy/sz. This is a direct analogue of the 9-channel Euler layout, keyed as Ts splines on doubles. Bool, int and enum channels are custom attributes with timeSamples; the graph editor handles scalar splines only. The actor transform is an animated Xform above the asset root. Space-switch keys have no equivalent, since parent:space is a static connection (G4).

**Gap.** No typed channel semantics (enum labels, bool, int), no rotation-order change with re-key, no UE sequence importer or tangent mapping, and no animatable space switching (G4).

**Porting impact.** The transform and float channels of zebra_audition, zebra_marketingPoseFaces and MR_Zebra_Take1 can be converted once key data is re-dumped. Rotations need a handedness and axis conversion per control rest frame. 'Bake Root On' and the 69 bool channels lose editor semantics, and the space keys cannot be expressed.

**Recommendation.** Add python/rigexec/ue_sequence_import.py. It reads re-dumped channel keys, maps control FName to prim path through the name table (G2-module-namespacing), and converts rotations through quaternions into degrees consistent with avars:rotationOrder. It writes Ts splines with converted tangents on double avars and timeSamples on bool/int/token customs. Add uniform token[] rigExec:enumLabels metadata for enum channels, and a rotation-order change tool in plugin/rigExecUsdview that re-keys through quaternions.

**Evidence:** `examples/ArmShotAnim.usda:9-16`; `examples/biped/Biped_anim.usda:12-30`; `libs/rigExecSchema/schema.usda:327-336`; `docs/spec.md:1094-1104`; `docs/graph-editor.md:18-27`; `docs/viewport-gizmos.md:110-128`

**Verification (holds).** Confirmed:
- shot layers author avar splines and time samples (ArmShotAnim.usda:9-20; Biped_anim.usda:12-30);
- six rotation orders (schema.usda:327-336);
- the graph editor handles scalar splines only (graph-editor.md:18-27);
- Ts spline and time-sample authority (spec.md:1094-1104).

Refinement: parent:space is a static connection, but RigExecParentConstraint with animatable inputs:sourceWeights already gives keyable multi-space following, without switch compensation (G4). 'No equivalent' for space keys is therefore slightly strong, but the gap stands.

Verifier evidence: `examples/ArmShotAnim.usda:9-20`; `examples/biped/Biped_anim.usda:12-30`; `libs/rigExecSchema/schema.usda:327-336`; `docs/spec.md:1094-1104`; `docs/graph-editor.md:18-27`; `libs/rigExecSchema/schema.usda:1115-1142`

### G2-skeleton-import

**Skeleton import into the rig hierarchy (imported bones + reference pose)**

**Verdict:** Partial · **Severity:** major · **Effort:** M · **Confidence:** high · **Domain:** D20

UE features: `UE1-static-hierarchy-import`, `UE-deform-runtime-asset`

**UE rigs.** Every modular rig's static hierarchy is the whole skeleton as BoneType=Imported bones with the reference pose (Zebra 371, Monster 165, Manny 106). CR_Zebra_Deform re-imports the skeleton in its construction event (HierarchyImportFromSkeleton, curves and virtual bones included), so it always matches the mesh. CR_Monster_Deform stores its 165 bones statically.

**usdRig today.** A skeleton is a nested RigExecJoint hierarchy with parent-relative rest:space (hierarchy = namespace nesting). RigExecRigBuilder::AddJoint(name, restSpace, parentJoint) authors it. No import step exists: there is no reader for UE skeletons, FBX or UsdSkelSkeleton, the biped joints were generated by off-repo tooling, and the rig cannot re-sync to a changed skeleton at evaluation time (no topology changes during evaluation). Rest edits are value-only and re-measured live, so an importer that rewrites rest values needs no recompile.

**Gap.** No skeleton/UsdSkel/FBX importer, no 'imported vs rig-created' joint flag, and no resync-on-skeleton-change workflow. tools/biped (the only generator referenced by the docs) is absent from the checkout.

**Porting impact.** The 371 Zebra and 165 Monster joints and their reference poses must be generated by a new script. An axis or parent-relative error corrupts every downstream solver and skin binding.

**Recommendation.** Add python/rigexec/skeleton_import.py with import_skeleton(stage, rig_root, source). It reads a UsdSkelSkeleton (joints tokens + restTransforms, e.g. the UE USD exporter output) and emits nested RigExecJoint prims with parent-relative rest:space, reusing the compose/decompose math of tools/migrateRestToLocal.py. It records customData {ue:boneName, rigExec:imported=true}. Re-running it only updates rest values (value edits, no recompile). Add a C++ counterpart RigExecRigBuilder::ImportSkeleton in libs/rigExecRigging/rigBuilder.{h,cpp} with a python/_rigexec.cpp binding.

**Evidence:** `libs/rigExecSchema/schema.usda:298-300`; `libs/rigExecSchema/schema.usda:360-371`; `libs/rigExecRigging/rigBuilder.h:1060-1067`; `docs/biped-rig.md:24-27`; `docs/spec.md:58`; `README.md:36-45`; `tools/migrateRestToLocal.py:1-24`

**Verification (holds).** Refutation failed. Nothing in the repo imports a skeleton:
- grep for UsdSkel/Skeleton/restTransforms/SkelRoot in python/, tools/ and plugin/rigExecUsdview finds no reader. The only UsdSkel use in libs is UsdSkelBlendShape consumption.
- tools/ holds only bakeGizmoIcons.py, diagnoseSolverBinding.py, migrateRestToLocal.py and rigExecPose.cpp; there is no tools/biped.
- docs/biped-rig.md:24-27 says the conversion tooling is not in the repository.
The representation itself exists: nested RigExecJoint with parent-relative rest:space, AddJoint(parentJoint), and live rest re-measure (README.md:31-45).
All 371 SK_Zebra bones (and the Monster bones) have S(1,1,1) in bones.txt, so the always-orthonormalized rest loses nothing.
This row duplicates the importer in G2-usd-export-import; count the work once.

Verifier evidence: `libs/rigExecSchema/schema.usda:298-300`; `libs/rigExecSchema/schema.usda:315-321`; `libs/rigExecRigging/rigBuilder.h:1060-1067`; `docs/biped-rig.md:24-27`; `README.md:31-45`; `tools/migrateRestToLocal.py:1-24`; `ue/<dump>/Game__Assets__Zebra__Meshes__SKM_Zebra/bones.txt (all S(1,1,1))`

### G2-construction-event

**Procedural construction event (runtime spawning of nulls/bones/controls/channels)**

**Verdict:** Divergent-by-design · **Severity:** major · **Effort:** XL · **Confidence:** high · **Domain:** D20

UE features: `UE2-construction-spawn-pattern`, `UE4-unit-dynamic-hierarchy-spawn`, `UE6-construction-order`, `UE-deform-runtime-asset`

**UE rigs.** All Fortnite modules, both face modules and CR_Zebra_Deform build their rigs in the Construction event. Each runs a Sequence: reset arrays, resolve connectors, spawn nulls, bones, controls and channels (construction-only HierarchyAdd* units) from INITIAL bone transforms, cache keys in variables, set metadata, channel hosts and spaces, and apply the control scale. The Spine graph alone has 439 nodes, and the face graphs are ordered Sequences of 274 and 285 nodes. Spawned names are module-namespaced, and construction re-runs whenever the rig or skeleton changes.

**usdRig today.** OpenExec cannot add prims or author values, topology is fixed per epoch, and structural edits recompile. The substitute is offline imperative authoring with RigExecRigBuilder / rigexec.Builder, which defines typed prims under fixed scopes. It has no module concept, no re-run-on-edit, and no access to connector or skeleton inputs beyond what the caller passes.

**Gap.** No rebuildable construction layer: no generator registry, no 'rebuild from inputs' action, and no ownership of generated layers, so hand edits and regenerated prims can collide.

**Porting impact.** Every module's construction graph must be re-implemented as a Python generator that emits static USD. The generated hierarchy must be regenerated when module config or the skeleton changes.

**Recommendation.** Add python/rigexec/construct/. A Generator protocol takes skeleton prims, a module config dict and connector targets, and writes one Sdf.Layer per module (<asset>/rig_generated/<module>.usda) with stable prim names and composed-order 'reorder nameChildren' pins (docs/biped-rig.md:261-271). Add a usdview 'Rebuild Rig' action (plugin/rigExecUsdview) that regenerates idempotently beneath a stronger user-override layer. Construction order becomes generator order; namespacing is one Scope per module.

**Evidence:** `docs/spec.md:58`; `docs/spec.md:335`; `README.md:22-23`; `libs/rigExecRigging/rigBuilder.h:1-24`; `libs/rigExecRigging/rigBuilder.h:1030-1070`; `python/rigexec/__init__.py:1-47`

**Verification (holds).** The non-goal is verified (spec.md:58: OpenExec cannot add or remove stage objects or author values), as is the non-destructive rule (README.md:22-23). The builder is offline and imperative, with fixed scopes and no re-run (rigBuilder.h:1028-1070; rigBuilder.cpp:2598-2617). Major reflects the XL re-implementation and the lost rebuild-on-change workflow.

Verifier evidence: `docs/spec.md:58`; `README.md:22-23`; `libs/rigExecRigging/rigBuilder.h:1028-1070`; `libs/rigExecRigging/rigBuilder.cpp:2598-2617`

### G2-stateful-interaction

**Interaction-dependent and stateful solves, rig-requested auto-key**

**Verdict:** Divergent-by-design · **Severity:** major · **Effort:** M · **Confidence:** high · **Domain:** D18

UE features: `UE2-stateful-interaction-and-autokey`

**UE rigs.** Several solves depend on host interaction and persistent state:
- RigUnit_IsInteracting (Movable Proxy, Body aim).
- SendEvent 'RequestAutoKey' (Body aim/twist sync, Prop pivot release).
- Variables carried across evaluations (Body aim buffers, Prop pivot/buffer transforms, Root 'Global Control Snapped', proxy 'IsSet').
- SetControlOffset and SetControlColor called every forward solve (Spine, Prop).

**usdRig today.** Hidden state is rejected, and so is previous-frame continuity; the event layer is deferred. Interaction exists only tool-side: the gizmo previews through SetInteractiveOverrides and authors one key per dragged channel on release. Offsets are default:space value edits (no recompile), not runtime writes.

**Gap.** No interaction signal visible to rig logic, no rig-declared auto-key of other controls, and no per-instance persistent state.

**Porting impact.** Body aim/twist sync, the prop movable-pivot release, the root first-frame snap and the proxy IsSet behavior must be redesigned as explicit channels plus tool-side compensation-and-key actions. Without that, animators see pops when pivots move.

**Recommendation.** Add scene-declared manipulation hooks consumed only by plugin/rigExecUsdview: rel rigExec:autoKey:controls on RigExecControlAPI (controls keyed together on release) and uniform token rigExec:onRelease (e.g. 'compensatePivot', which solves the compensating avars with rigexec.solve_parameters and keys them). Keep evaluation stateless, and model movable pivots as explicit pivot channels composed into default:space.

**Evidence:** `docs/spec.md:59`; `docs/spec.md:1050`; `docs/spec.md:207`; `libs/rigExec/rigEvaluator.h:397-428`; `docs/viewport-gizmos.md:83-128`; `libs/rigExecSchema/schema.usda:301-314`

**Verification (holds).** All cited lines exist:
- hidden stateful simulation is a non-goal (spec.md:59);
- hidden previous-frame continuity is rejected (spec.md:1050);
- the event/direct-manipulation layer is deferred (spec.md:207).
Interaction exists only tool-side, through SetInteractiveOverrides (rigEvaluator.h:397-424) and commit on release (viewport-gizmos.md:83-128).

Verifier evidence: `docs/spec.md:59`; `docs/spec.md:1050`; `docs/spec.md:207`; `libs/rigExec/rigEvaluator.h:397-428`; `docs/viewport-gizmos.md:83-128`

### G2-usd-export-import

**UE to USD export (UsdSkel + blend shapes) and round-trip**

**Verdict:** Divergent-by-design · **Severity:** major · **Effort:** M · **Confidence:** high · **Domain:** D18

UE features: `UE-monster-usd-export`

**UE rigs.** Monster.usd from the UE exporter contains a SkelRoot, a Skeleton (165 full-path joints), and a Mesh with SkelBindingAPI, 79 BlendShapes bound by morph name, 12 influences per vertex, and 5 GeomSubsets with UnrealMaterial shaders. It has no animation and no rig logic.

**usdRig today.** UsdSkel bridges are a non-goal, and export_baked writes a sampled point/xform cache rather than UsdSkel. The exported data is still consumable:
- RigExecBlendSample.rigExec:blendShape reads UsdSkelBlendShape offsets and pointIndices directly.
- RigExecSkinMover uses UsdSkel's jointIndices/jointWeights layout, and these attributes may be connected.
No converter creates RigExecJoints from Skeleton.joints/restTransforms, and geomBindTransform is not supported (bind = provider rest).

**Gap.** No offline UsdSkel-to-RigExec converter and no SkelAnimation export of evaluated joints for a game round-trip.

**Porting impact.** The Monster mesh, weights and 79 shapes can be reused almost directly, but the skeleton and bind conversion must be scripted. Results cannot go back to UE as SkelAnimation.

**Recommendation.** Add python/rigexec/usdskel.py, offline and outside the evaluator, consistent with the no-live-bridge rule.
- import_skel_root(stage, skel_root, rig_root): builds the RigExecJoint tree (joint tokens to nested prims, restTransforms to parent-relative rest:space, geomBindTransform baked into points), a SkinMover with rigExec:influences in Skeleton.joints order and jointIndices/jointWeights connected to the primvars, and one BlendInput per blendShape.
- export_skel_animation(rig, times, path): writes a SkelAnimation from pose joint frames.

**Evidence:** `docs/spec.md:61`; `python/rigexec/bake.py:8-22`; `docs/python-bake-inverse.md:26-31`; `libs/rigExecSchema/schema.usda:1783-1802`; `libs/rigExec/rigEvaluator.cpp:3805-3808`; `libs/rigExecSchema/schema.usda:1716-1727`; `libs/rigExec/rigEvaluator.cpp:5471-5486`

**Verification (holds).** The non-goal exists (spec.md:61). The data-level consumption is as described:
- rigExec:blendShape must name a UsdSkelBlendShape (rigEvaluator.cpp:3800-3810);
- skin jointIndices and jointWeights may be connected (rigEvaluator.cpp:5471-5486);
- the bind is the provider rest, with no inverse bind (schema.usda:1681-1735).
export_baked produces no UsdSkel (python-bake-inverse.md:29-32).

The offline converter is compatible with the non-goal. The skeleton half duplicates G2-skeleton-import; count it once.

Verifier evidence: `docs/spec.md:61`; `libs/rigExec/rigEvaluator.cpp:3800-3810`; `libs/rigExec/rigEvaluator.cpp:5468-5490`; `libs/rigExecSchema/schema.usda:1681-1735`; `docs/python-bake-inverse.md:29-32`; `python/rigexec/bake.py:8-22`

### G2-import-data-gaps

**UE extraction gaps blocking an exact port**

**Verdict:** Not-applicable · **Severity:** major · **Effort:** S · **Confidence:** high · **Domain:** D18

UE features: `UE8-dump-gaps-runtime-assets`

**UE rigs.** The dump is incomplete in several ways:
- The 7 runtime-asset rigs have no runtime hierarchy or module settings.
- 69 variable records are truncated at 500 characters, including the face Soft Eyes, Lid Blink and Lid Micro arrays.
- ConfigOverrides export as '()'.
- Skeleton curve metadata reads failed, and AnimSequence vector curves were not dumped.
- Sequencer channel keys are missing.

**usdRig today.** This is an input-data limitation, not a usdRig capability. Any usdRig importer needs the complete values.

**Gap.** Missing source data (external to usdRig).

**Porting impact.** Face lid tables, module config values, curve routing and all keyframes cannot be ported exactly until re-dumped.

**Recommendation.** Re-dump before porting:
- Variables without truncation.
- Sequencer channel proxies and space keys.
- AnimCurveMetaData and vector curves.
- Runtime-asset rigs, instantiated.
Store the results as JSON inputs to the python/rigexec importer modules.

**Evidence:** `docs/biped-rig.md:24-27`

**Verification (holds).** The limitation is in the source data, not in usdRig. Major is accepted as porting-blocking input: the face arrays are truncated and there are no sequencer keys.

Verifier evidence: `docs/biped-rig.md:24-27`

### G2-dmc-shape-resolution

**Get Control Shape Name From Item v02 (layered / DMC-aware shape lookup)**

**Verdict:** Missing · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D17

UE features: `UE7-dmc-shape-name-resolution`

**UE rigs.** A library function resolves the control's shape name. If the module's ShapeLib namespace is in the DMC library list and shape '<layer>.<resolved bone>' exists, it uses that; otherwise it falls back to the default gizmo shape (with a 'Default' special case). It is used at 9 call sites in 5 modules.

**usdRig today.** guide:shape is a fixed 6-token enum with no shape library, no namespaced lookup and no exists-fallback. Mesh-surface picking is provided separately by TouchPose regions.

**Gap.** No shape library and no layered shape resolution.

**Porting impact.** DMC-shaped controls fall back to primitive guides, and selection goes through TouchPose (cross-ref G3/G7).

**Recommendation.** Add rel guide:shapeSource (a BasisCurves or Mesh prim in a shape-library scope), falling back to guide:shape, resolved in libs/rigExecImaging/sceneIndices.cpp. The importer converts '<layer>.<bone>' shapes to TouchPose regions when the polygroup layer exists.

**Evidence:** `libs/rigExecSchema/schema.usda:167-179`; `libs/rigExecSchema/schema.usda:2610-2635`

**Verification (holds).** Confirmed: guide:shape is a fixed six-token enum (schema.usda:167-168), with no custom-shape relationship or library.

Verifier evidence: `libs/rigExecSchema/schema.usda:167-179`; `libs/rigExecSchema/schema.usda:2610-2635`

### G2-ik-rig

**IK Rig asset (full-body IK solver, goals, bone settings, retarget chains)**

**Verdict:** Missing · **Severity:** minor · **Effort:** XL · **Confidence:** high · **Domain:** D18

UE features: `UE-ikrig-zebra`

**UE rigs.** IK_Zebra is the retarget target rig:
- One IKRigFullBodyIKSolver (root pelvis, 20 iterations / 10 sub-iterations, no stretch, MaxAngle 30, OverRelaxation 1.3).
- 4 goals on the hands and feet (ChainDepth 2).
- RotationStiffness 0.95 on the clavicles and pelvis; lowerarm and calf hinges with X/Y locked and PreferredAngles Z=90.
- 14 retarget chains with PelvisBone=pelvis.

**usdRig today.** Only per-chain solvers exist: RigExecTwoBoneIk, RigExecSingleChainIkConstraint (no stiffness, limits or preferred angles) and RigExecSplineIk. There is no multi-effector full-body solver, no per-joint lock or stiffness, and no retarget-chain definition schema.

**Gap.** No FBIK and no retarget chain data.

**Porting impact.** IK_Zebra is only used by the non-functional retargeter, so animator-rig parity does not depend on it.

**Recommendation.** Defer. If needed, add a RigExecFullBodyIk atomic bundle solver (rigExec:joints claim; goal relationships with chainDepth; per-joint stiffness, lock and preferred-angle arrays) in libs/rigExecMath (new fbik.cpp), plus a data-only RigExecRetargetChain prim (startJoint, endJoint, goal) for retarget tooling.

**Evidence:** `libs/rigExecSchema/schema.usda:524-598`; `libs/rigExecSchema/schema.usda:1144-1190`; `libs/rigExecSchema/schema.usda:788-800`

**Verification (holds).** Confirmed:
- only TwoBoneIk (schema.usda:524), SplineIk (788) and SingleChainIkConstraint (1144) exist;
- no multi-effector solver or retarget-chain schema (grep 'retarget' finds only unrelated comments).
IK_Zebra serves only the non-functional retargeter, so minor.

Verifier evidence: `libs/rigExecSchema/schema.usda:524-598`; `libs/rigExecSchema/schema.usda:788-800`; `libs/rigExecSchema/schema.usda:1144-1190`

### G2-manny-reference-skeleton

**Reference mannequin (SKM_Manny) for developing and validating ported module generators**

**Verdict:** Missing · **Severity:** minor · **Effort:** S · **Confidence:** medium · **Domain:** D20 · *added by verifier*

UE features: `UE8-manny-reference-mesh`

**UE rigs.** SKM_Manny (106 bones, UE5 mannequin layout, 3 LODs, no morphs or physics) is the preview mesh of all 20 FortniteRigs assets (5 libraries, 13 modules, 2 templates). The template connection targets (metacarpals, 5 fingers, twist bones) use its bone names, and MR_FN_Biped's runtime hierarchy (283 controls, 329 nulls, 127 bones) was produced against it. SK_Mannequin and SKM_Manny_Simple are ObjectRedirectors to it.

**usdRig today.** The repo's only character skeleton is the conversion-generated biped (252 joints, _l/_r and _bind naming; examples/biped). There is no mannequin-layout skeleton, so generic Fortnite-module generators cannot be tested against the configuration they were authored for. MR_FN_Biped's dumped runtime hierarchy is the natural oracle.

**Gap.** There is no Manny-layout test skeleton or mesh and no oracle test comparing generated module output with MR_FN_Biped.

**Porting impact.** Module generators can be validated only on the Zebra and Monster, whose layouts differ (no middle or ring fingers on the Zebra; the Monster is upper body only), so generic-template bugs go undetected.

**Recommendation.** Import SK_Manny through the proposed skeleton importer (JSON or USD export) into tests/fixtures/manny_skeleton.usda. Add a tests/python test that runs the module generators against it and compares control, null and joint names and counts with the MR_FN_Biped runtime_hierarchy.txt dump.

**Evidence:** `examples/biped/README.md:39`; `docs/biped-rig.md:24-27`; `docs/biped-rig.md:229-231`

### G2-retargeter

**IK Retargeter op stack (UEFN mannequin to Zebra), broken as shipped**

**Verdict:** Missing · **Severity:** minor · **Effort:** L · **Confidence:** high · **Domain:** D18

UE features: `UE-retargeter-ops`, `UE8-rtg-broken`

**UE rigs.** RTG_UEFN_to_Zebra stacks 9 ops: Pelvis Motion, FK Chains (14), Run IK Rig, Blend to Source, Body Intersect IK (PA_Zebra), Offset Goals, Root Motion, Remap Curves (copy all) and Filter Bones (neck/head). TargetMeshOffset X=85.39, and the retarget pose carries offsets. As shipped it has no SourceIKRigAsset and a missing source mesh, so it cannot run, and the Run IK Rig chain map contains a RightClav->RightLeg copy error.

**usdRig today.** No retargeting exists (no retarget or physics code in libs, python or plugin). Each rig has its own evaluator, and the evaluator rejects cross-rig writes. A source skeleton could be brought into the same asset and constrained, but that is authoring, not a retarget op stack.

**Gap.** No live or offline retargeting.

**Porting impact.** None for animator equivalence, since the asset cannot run as shipped. Reuse of mocap and UEFN animation is lost.

**Recommendation.** Keep retargeting outside the live evaluator. Add offline python/rigexec/retarget.py that maps source joint animation onto target controls (FK chain copy; IK goals matched with rigexec.solve_parameters) and bakes the result to avars in a shot layer. Do not port the broken asset.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:3308-3313`; `libs/rigExecImaging/registry.cpp:155-262`; `docs/plans/evaluation-engine-gaps-vs-premo-libee.md:145`

**Verification (holds).** Confirmed:
- grep for retarget, UsdPhysics and ragdoll in libs, python and plugin/rigExecUsdview finds no feature code;
- cross-rig writes are rejected (rigEvaluator.cpp:3307-3313).
The UE asset cannot run as shipped, so minor.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:3307-3313`; `libs/rigExecImaging/registry.cpp:155-262`; `docs/plans/evaluation-engine-gaps-vs-premo-libee.md:145`

### G2-stock-modules

**Engine stock Root and AddControl modules (MR_Boombox)**

**Verdict:** Missing · **Severity:** minor · **Effort:** S · **Confidence:** medium · **Domain:** D20

UE features: `UE8-engine-addcontrol-root-modules`

**UE rigs.** Engine Modules58, as inferred from name tables:
- Root: a root/global/body_offset '_ctrl' stack sized from the skeleton bounding box, global metadata for child modules, sockets pelvis/spine/spine_01/biped_physics, and an 'INV Root' backwards solve.
- AddControl: Control Stack at Item, a '_socket' per child, ParentConstraint attaching bone and control, side colouring by plane, and ModuleSettings.
MR_Boombox is Root + 7 chained AddControl modules.

**usdRig today.** There is no stock component library (examples/components holds only the spider_leg files) and no 'add a control on a bone' macro. The pieces exist: RigExecControl plus RigExecParentConstraint with per-source offsets.

**Gap.** No stock root or add-control components.

**Porting impact.** The boombox prop rig (8 bones, about 7 controls) must be hand-built; its complexity is low.

**Recommendation.** Ship examples/components/root.usda and add_control.usda, plus a python builder add_control_on_joint(joint, parent_control, offset, shape) that creates the control and a ParentConstraint driving the joint. Re-dump the binary engine modules before porting their exact behavior.

**Evidence:** `libs/rigExecSchema/schema.usda:1115-1142`; `libs/rigExecRigging/rigBuilder.h:383-397`; `libs/rigExecRigging/rigBuilder.h:1030-1070`

**Verification (holds).** Confirmed:
- examples/components contains only spider_leg.usd and spider_leg_ik.usd;
- no add-control macro exists;
- the building blocks do exist: ParentConstraint with per-source offsets (schema.usda:1115-1142; rigBuilder.h:383-397).
The Boombox is small, so minor.

Verifier evidence: `libs/rigExecSchema/schema.usda:1115-1142`; `libs/rigExecRigging/rigBuilder.h:383-397`; `libs/rigExecRigging/rigBuilder.h:1028-1070`

### G2-baked-anim-input

**Baked skeletal animation (AnimSequence bone tracks) as input to the corrective/deformer layer**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D18 · *added by verifier*

UE features: `UE-deform-runtime-asset`, `UE-postprocess-abp`, `UE-runtime-layering`, `UE7-zebra-deform-postprocess-chain`

**UE rigs.** The post-process deform rig runs on whatever pose arrives, including a plain AnimSequence with no animator rig. Zeb_Face_Expressions has 163 frames, 371 bone tracks (every def_* and twist helper included) and 102 float curves.

The ABP node sets bResetInputPoseToInitial and bTransferInputPose, so bones missing from the input keep the reference pose. The correctives and deformer drivers are then recomputed from that pose.

**usdRig today.** A joint can be posed from sampled data with no solver, in two ways:
- Local avars (tx..rz, sx..sz), rest-relative and following the namespace parent (computations.cpp:350-383).
- An authored non-identity posed:space, an asset-space matrix used directly (computations.cpp:343-352; schema.usda:315-321).
Pose interpolators and skin then consume the final pose, so a joint-only rig can play baked animation.

What is missing:
- No importer for AnimSequence or UsdSkelAnimation; only export_baked exists (bake.py:8-22), and UsdSkel bridges are a non-goal (spec.md:61).
- A joint claimed by an animator solver ignores its own avars and posed:space (value override: computations.cpp:338-341; ownership at rigEvaluator.cpp:10629-10631). Switching one asset between animator-rig and baked playback therefore needs a variant that removes the solver claims, which is structural and starts a new epoch (spec.md:345).

**Gap.** There is no conversion of baked local bone tracks into per-joint avar splines or posed:space samples, and no value-only switch between rig-driven and baked joint poses on one asset.

**Porting impact.** Zeb_Face_Expressions, and any mocap or UEFN animation, can drive the correctives, morphs and skin only after an offline conversion of 371 tracks x 163 frames. The animator and playback modes must be separate variants. No shipped sequence uses the AnimSequence; their SkeletalAnimation tracks are empty.

**Recommendation.** Add python/rigexec/anim_import.py with import_joint_animation(stage, rig_root, source).
- Read a UsdSkelAnimation or a JSON re-dump of the AnimSequence tracks.
- Convert each local transform to rest-relative avars (local * restLocal^-1), decomposed through quaternions in the joint's avars:rotationOrder.
- Write Ts splines into a shot layer.
- Route curves to the proposed RigExecCurveSet attributes.

Add a 'rigMode' variant set {animator, playback} whose playback variant deactivates the animator Solvers and constraint scopes.

**Evidence:** `libs/rigExec/computations.cpp:336-383`; `libs/rigExecSchema/schema.usda:315-321`; `libs/rigExec/rigEvaluator.cpp:10629-10631`; `python/rigexec/bake.py:8-22`; `docs/spec.md:61`; `docs/spec.md:345`

### G2-curve-bus-layering

**Curve bus between animator face rig, baked animation and post-process deform rig**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D18 · *(analyst said Partial / major)*

UE features: `UE6-zebra-deformer-curve-interface`, `UE7-zebra-deform-postprocess-chain`

**UE rigs.** CRM_Zebra_Face writes 7 deformer-driver curves (head_squash, head_twist, head_bend, muzzle/skull_tp *_deformer). CR_Zebra_Deform (post-process) reads them with GetCurveValue. Squash curves are remapped without clamping (-1..1 to 0..1; the muzzle range is inverted) before driving 7 Optimus deformers placed by its own nulls. Because the contract is plain curves, baked animation that carries them (Zeb_Face_Expressions) drives the deformers with no animator rig.

**usdRig today.** Within one RigExecRoot a float attribute can act as the curve: rig logic writes it with FloatMathMover chains, the squash remap is FloatMathMover 'remap' (unclamped), and consumers connect to it. Property chains revise from the authored base, which may be animated, so a baked-curve playback path exists. However, a connection outranks the animated value, so one float cannot be both rig-driven and animation-driven. FloatMathMover reads and writes only float, while control avars are double and connections must match type exactly, so a translate-driven squash control cannot feed the curve directly. A separate post-process rig cannot receive the bus (cross-rig writes rejected, shared prim publication rejected). The consumers (bend/twist/squash) do not exist.

**Gap.** There is no pose-level scalar bus shared between rig passes and animation, no 'animated unless rig-driven' selection for one channel, a double-to-float typing wall between control avars and math movers, and no deformer consumers (G7).

**Porting impact.** Face controls, driver floats and deformers must all live in one rig. Supporting both the animator path and baked-curve playback needs a variant or switch. Squash controls need custom float dials instead of transform avars.

**Recommendation.** (1) Allow double sources on float math-mover inputs with explicit narrowing, or add RigExecDoubleMathMover (propertyMath.{h,cpp}, rigEvaluator.cpp _ValidateScalarConnection, baked step). (2) Standardize a 'curve source' pattern: a FloatMathMover 'blend' whose envelope connects to a rig-level useBakedCurves dial, so the same RigExecCurveSet attribute can take either the animated base or the rig value. (3) Share the curve set between the animator-rig and corrective layers by reference (see G2-curve-channels).

**Evidence:** `libs/rigExecSchema/schema.usda:1192-1213`; `libs/rigExecMath/propertyMath.h:48-60`; `libs/rigExec/rigEvaluator.cpp:543-566`; `plugin/rigExecUsdview/avarEditorModel.py:37-45`; `libs/rigExec/rigEvaluator.cpp:11666-11673`; `libs/rigExec/rigEvaluator.cpp:3308-3313`; `libs/rigExecImaging/registry.cpp:244-251`

**Verification (corrected).** One headline gap is refuted by a probe: 'animated unless rig-driven' selection for one channel exists today.
- Setup: a FloatMathMover 'blend' on the curve float, with inputs:value connected to the rig float and inputs:defaultWeight connected to a switch float. Connected defaultWeight is validated as float at rigEvaluator.cpp:3625-3650.
- Result: the curve kept its animated -0.3 at switch 0 and took the rig value 0.4 at switch 1 (probe_double.py).

The inverted muzzle remap (1..-1) also works: remap with min=1, max=-1 is (v-1)/(-2), and only a zero span is special-cased (propertyMath.cpp:48-51).

The double-to-float wall is real and worse than described: it is SILENT.
- inputs:value connected to a double avars:tx is not validated at compile.
- It resolved to the mover's own authored -9.0 with no diagnostic (probe_double.py; moverGraph.h:340-378 fallback walk).

Remaining gaps and where they are counted:
- no separate post-process rig; one rig suffices in usdRig;
- the silent typing wall, worked around with float dials;
- the missing deformer consumers, counted in G7.

With a workaround for each residual item, minor fits better than major.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:3625-3650`; `libs/rigExecMath/propertyMath.cpp:28-61`; `libs/rigExec/moverGraph.h:314-378`; `libs/rigExec/rigEvaluator.cpp:8695-8720`; `plugin/rigExecUsdview/avarEditorModel.py:37-45`; `reports/ue-zebrasample/probes/g2review/probe_double.py`

### G2-curve-channels

**Rig CURVE elements, curve metadata and name-based morph binding**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D1

UE features: `UE-curve-metadata`, `UE1-static-hierarchy-import`

**UE rigs.** Rigs hold CURVE elements: named floats that flow in and out with the pose (MR_Zebra 958, CR_Zebra_Deform 50 static plus imported, MR_Monster 993). Skeleton AnimCurveMetaData flags morph/material curves (4 on Monster; Zebra's export is empty). Curve names match morph targets case-insensitively ('squetch' == 'Squetch'). Curves with no morph feed rig logic or materials (cornea_size, pupil_dilation).

**usdRig today.** There is no curve element or pose-level scalar namespace. The equivalents are float attributes: custom float dials on a host prim (the biped's foot:*/avars:* customs), RigExecBlendInput.inputs:weight and RigExecPose.outputs:weight. Binding is by an explicit single, same-type connection, never by name. No morph/material flag exists. Evaluated floats reach hosts only as movedProperties (RigExecImaging_GetMovedFloats), never as material inputs.

**Gap.** No declared curve set with per-curve metadata (morph/material/rig), no name-based auto-binding, and no route from rig floats to UsdShade material inputs.

**Porting impact.** The 958/993 imported curves reduce to the roughly 150 that are consumed. Every morph weight must be connected explicitly. Material curves (pupil_dilation, cornea_size, highlight_*) have no destination.

**Recommendation.** Add a codeless RigExecCurveSet typed prim (under <rig>/Curves) whose declared 'float curves:<name>' attributes carry customData {morph, material}. The importer creates it from skeleton curves and authors BlendInput.inputs:weight.connect by case-insensitive name match. In libs/rigExecImaging/sceneIndices.cpp, publish material-flagged curves as overrides on bound UsdShade inputs (an explicit rel rigExec:materialInputs), keeping the stage unauthored.

**Evidence:** `libs/rigExecSchema/schema.usda:1755-1766`; `libs/rigExecSchema/schema.usda:2065`; `libs/rigExec/rigEvaluator.cpp:543-566`; `plugin/rigExecUsdview/avarEditorModel.py:37-54`; `libs/rigExecImaging/registry.h:249-259`

**Verification (holds).** Confirmed. There is no curve element type, no per-curve flags and no name-based binding: a BlendInput weight is a float with at most one same-type connection (schema.usda:1755-1766; rigEvaluator.cpp:544-566; moverGraph.h:314-378).

Live imaging publishes only points, normals and extent. Every other moved scalar goes to movedFloats (bridge.cpp:1385-1413), so a FloatMathMover could revise a shader float input under the asset root but Hydra would never see it.

Minor nuance: export_baked writes every moved property as time samples (bake.py:8-22), so an offline route to material inputs exists.

Minor severity is right: the material-curve routing is itself an open UE question (ue_open_questions ue:filler).

Verifier evidence: `libs/rigExecSchema/schema.usda:1755-1766`; `libs/rigExec/rigEvaluator.cpp:544-566`; `libs/rigExec/moverGraph.h:314-378`; `libs/rigExecImaging/bridge.cpp:1385-1413`; `libs/rigExecImaging/registry.h:245-259`; `python/rigexec/bake.py:8-22`

### G2-deformer-child-components

**Deformer propagation to child skeletal mesh components**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** medium · **Domain:** D18

UE features: `UE7-deformer-child-components`

**UE rigs.** DeformChildComponents=True on all 16 AddOptimusDeformer nodes enqueues the same deformer on every child USkeletalMeshComponent (recursively). Each child gets its own instance with pushed variables and a mask from its own skin weights; no tag exclusion is set.

**usdRig today.** BlendShapeMover fans out over multiple targets with per-target deltas. Matrix, Skin, Smooth, Lattice, VolumeCorrect and Curvenet movers require exactly one target. No mover discovers attached meshes from the hierarchy, and weight objects are per target.

**Gap.** No 'apply to attached meshes' fan-out for deformers, and no per-target mask derived from each target's own skin weights.

**Porting impact.** The sample has no child meshes on the Zebra, so the impact is limited to future hair or clothing. G7 deformers would have to be duplicated per mesh.

**Recommendation.** Make G7's parametric deformer movers multi-target with per-target application (the BlendShapeMover model) and per-target weight objects. Add a builder helper that expands one deformer spec over every PointBased prim under a scope.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:3464-3466`; `libs/rigExec/rigEvaluator.cpp:3505-3521`; `libs/rigExec/rigEvaluator.cpp:6944-6955`

**Verification (holds).** Confirmed: Smooth, VolumeCorrect, Lattice and Curvenet movers are single-target (rigEvaluator.cpp:3505-3521) while blend packets fan out per target (3464-3466). Zebra has no child mesh components, so the impact is small.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:3464-3466`; `libs/rigExec/rigEvaluator.cpp:3505-3521`; `libs/rigExec/rigEvaluator.cpp:6944-6955`

### G2-dmc-editor-scope

**DMC editor-only scope (picking surface never changes the pose)**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D18

UE features: `UE7-dmc-editor-only-scope`

**UE rigs.** DirectMeshControl is an experimental plugin with editor-only modules that is disabled by default (the sample enables it). The rig unit only works at construction with the editor delegate bound. DMC changes only the animator's picking and gizmo surface, never the pose; DMC templates animate exactly like non-DMC ones. While active, it forces LOD0 and a mesh deformer.

**usdRig today.** usdRig also keeps mesh picking out of evaluation. TouchPose regions are scene data consumed only by a usdview plugin, and they select the bound control without changing the pose. They support selection only, with no on-surface drag or proxy-mesh gizmo.

**Gap.** No mesh-patch manipulation surface (covered in G7).

**Porting impact.** Animators using the DMC templates can pick by touching the mesh but must drag with the standard gizmos.

**Recommendation.** See G7. Extend plugin/touchPose with a drag mode that forwards the gesture to the region's control gizmo, still leaving evaluation untouched.

**Evidence:** `libs/rigExecSchema/schema.usda:2517-2527`; `libs/rigExecSchema/schema.usda:2610-2635`; `libs/rigExecSchema/schema.usda:2572-2605`

**Verification (holds).** Confirmed:
- TouchPose regions are scene data holding a face set and a rel rigExec:touch:control (schema.usda:2620-2635);
- the plugin only paints and selects, with no drag gesture, and stands down during gizmo drags (touchPoseUI.py:72-95).

Verifier evidence: `libs/rigExecSchema/schema.usda:2530-2575`; `libs/rigExecSchema/schema.usda:2610-2635`; `plugin/touchPose/touchPoseUI.py:72-95`

### G2-function-libraries

**Versioned rig function libraries (CRFL) and module-local copies**

**Verdict:** Partial · **Severity:** minor · **Effort:** L · **Confidence:** high · **Domain:** D17

UE features: `UE4-library-structure-versioning`, `UE8-crfl-blend-twist-dead`

**UE rigs.** There are 5 CRFL ControlRigBlueprint libraries (46 public functions, 26 used) referenced through FunctionReference nodes. Changed behavior ships as a new versioned name (v01/v02) and old versions are kept. Some helpers are duplicated module-locally, e.g. LimbTwist's Blend Twist, which differs from the dead CRFL copy with its unexecuted axis-angle path. Event graphs contain ad-hoc test nodes.

**usdRig today.** Reuse is USD composition: referenceable component files (examples/components spider_leg), per-concern sublayer stacks, and references for mirrored sides. Versioning is asset paths or variants. Operator math lives in C++ schema computations. There is no user-authored function, macro or collapsed-graph layer, and no builder library ships (tools/biped is absent). Relationships that point outside a component cannot be rebased by a per-prim arc.

**Gap.** No reusable, versioned library of rig-construction functions or components.

**Porting impact.** The 26 used CRFL functions must be re-implemented once, as builder code or new schemas. Dead content (CRFL Blend Twist, unused v01 versions, test stubs) should be dropped. The LimbTwist-local variant is the one to port.

**Recommendation.** Create python/rigexec/components/ with versioned module generators (fk_chain_v1, ikfk_two_bones_v1, limb_twist_v1, ...), each emitting a self-contained layer and stamping customData rigExec:componentVersion. Adopt the class-instance limb layout proposed in docs/biped-rig.md:285-290 so a component's relationships stay inside the arc.

**Evidence:** `examples/biped/Biped_stack.usda:1-20`; `docs/biped-rig.md:243-259`; `docs/biped-rig.md:279-290`; `docs/biped-rig.md:24-27`

**Verification (holds).** Confirmed:
- reuse is by USD composition (Biped_stack.usda:1-20; biped-rig.md:243-259);
- no builder library ships;
- the relationship-rebasing limit is documented (biped-rig.md:279-290);
- examples/components holds only spider_leg.usd and spider_leg_ik.usd.

Verifier evidence: `examples/biped/Biped_stack.usda:1-20`; `docs/biped-rig.md:243-259`; `docs/biped-rig.md:279-290`; `docs/biped-rig.md:24-27`

### G2-mesh-runtime-binding

**Skeletal-mesh runtime wiring (post-process rig, deformer, resolution meshes)**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** medium · **Domain:** D18

UE features: `UE-skm-runtime-config`

**UE rigs.** Each skeletal mesh names its post-process ABP, physics asset and default/target mesh deformer (None; the rig's Add Deformer enables deformation). Each has 1 LOD, no cloth, and sockets. SKM_Zebra (34,459 verts) and SKM_Zebra_Hi (132,333 verts) share SK_Zebra and the same ABP/rig; SKM_Monster has 107,867 verts.

**usdRig today.** The rig binds meshes by explicit mover targets (rigExec:moves on SkinMover and BlendShapeMover), and a mesh has no back-reference to its rig. SkinMover takes exactly one points target because its layout is per point count, so Zebra and Zebra_Hi need separate skin and blend movers selected by a geometry variant or payload.

**Gap.** No mesh-level rig-binding concept, and no way for one rig to drive several resolution meshes without duplicating movers.

**Porting impact.** Zebra and Zebra_Hi each need their own SkinMover plus about 130 BlendInputs/samples, with weights imported twice. The morph sets also differ slightly.

**Recommendation.** Use a 'geomResolution' variant set on the asset (like rigComplexity in examples/ArmRig.usda) holding per-resolution Geom and Movers/Geometry scopes. Optionally add RigExecGeometryBindingAPI (rel rigExec:rig) on meshes for tool discovery only. Share the influence list through one referenced prim.

**Evidence:** `libs/rigExecSchema/schema.usda:66-68`; `libs/rigExec/rigEvaluator.cpp:6944-6955`; `libs/rigExecSchema/schema.usda:1681-1753`; `docs/spec.md:354`

**Verification (holds).** Confirmed: SkinMover takes exactly one points target (rigEvaluator.cpp:6944-6955), and meshes carry no back-reference to a rig. Each resolution needs its own movers.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:6940-6958`; `libs/rigExecSchema/schema.usda:66-68`; `libs/rigExecSchema/schema.usda:1681-1735`

### G2-module-namespacing

**Module-namespaced element names, de-duplication and display names**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D1

UE features: `UE1-module-namespacing`

**UE rigs.** Every module element is named '<Module>/<ShortName>' (e.g. 'Leg L/IK') and carries DesiredName, DesiredKey and Module metadata. Duplicate short names get _2/_3 suffixes while display_name keeps the clean label. Connection targets are FNames matched case-insensitively ('Control:Root/Local' resolves 'root/Local'). Controls without a display name show the full namespaced name.

**usdRig today.** USD prim paths namespace elements by construction (one scope per module), so uniqueness needs no suffixing, and relationships follow namespace edits. Prim names cannot contain spaces or '/', paths are case-sensitive, and no animator label is used anywhere: the repo never reads prim displayName, and the picker has its own ui:label/ui:text. Biped naming is convention only.

**Gap.** No display-name support for controls, no stored original UE name or module identity for round-trips, and no case-insensitive resolution.

**Porting impact.** 'Arm L/FK 0' must be sanitized into a valid path, so sequencer import needs a reversible name table. Animators lose clean labels such as 'Gimbal Control Vis' in the avar editor, outliner and graph editor.

**Recommendation.** Use stock USD prim displayName metadata (UsdPrim.SetDisplayName) for animator labels, shown by avarEditorModel, the picker and the graph editor. The importer writes customData {ue:name, ue:module, ue:desiredName}. Add python/rigexec/naming.py with Tf.MakeValidIdentifier-based sanitization and a persisted bidirectional map.

**Evidence:** `libs/rigExecRigging/rigBuilder.h:11-24`; `libs/rigExecSchema/schema.usda:2333-2365`; `docs/biped-rig.md:229-231`; `plugin/rigExecUsdview/avarEditorModel.py:14-20`

**Verification (holds).** Confirmed.
- grep finds no prim GetDisplayName/SetDisplayName in plugin/ or python/; the hits are layer display names only.
- The picker carries its own ui:label (schema.usda:2351-2356).
- Scopes come from the builder (rigBuilder.h:11-24).
- The USD path namespace gives uniqueness without _2 suffixes, and paths are case-sensitive.

Verifier evidence: `libs/rigExecRigging/rigBuilder.h:11-24`; `libs/rigExecSchema/schema.usda:2333-2365`; `plugin/rigExecUsdview/avarEditorModel.py:14-20`; `docs/biped-rig.md:229-231`

### G2-multi-character-shot

**Multiple rigs in one shot (Zebra + Boombox)**

**Verdict:** Partial · **Severity:** minor · **Effort:** L · **Confidence:** high · **Domain:** D18

UE features: `UE-level-sequences`

**UE rigs.** zebra_audition animates two Control Rig tracks (spawnable 'SKM Zebra' with MR_Zebra; MR_Boombox), plus actor transform and spawn tracks, static props, lights and a camera. There are no constraint channels between the characters.

**usdRig today.** The imaging registry evaluates every RigExecRoot on the stage serially under one mutex, each within its own asset root. Cross-rig writes are rejected, the shot-level multipass coordinator exists only in the spec, and rigExec:partition is read by nothing.

**Gap.** No cross-character constraints (e.g. a hand-held prop) and no parallel multi-rig evaluation.

**Porting impact.** The audition shot works as two independent rigs. Any hand-to-boombox relationship carried by space keys would have to be baked.

**Recommendation.** Implement spec §6.2 multipass in libs/rigExecImaging/registry.cpp: pass 0 evaluates independent poses in parallel; pass 1 resolves cross-character targets from pass-0 snapshots through a new RigExecExternalFrame source prim; pass 2 applies corrections.

**Evidence:** `libs/rigExecImaging/registry.cpp:155-262`; `docs/plans/evaluation-engine-gaps-vs-premo-libee.md:116`; `docs/spec.md:1136-1146`; `libs/rigExec/rigEvaluator.cpp:3308-3313`

**Verification (holds).** Confirmed:
- rigs are evaluated serially and a shared prim publication is rejected (registry.cpp:155-251);
- the multipass coordinator exists only in the spec (spec.md:1136-1146).

Addition: constraint sources outside the asset are NOT rejected; only mover targets are (rigEvaluator.cpp:3307-3313). Such sources are read from the stage xform cache relative to this asset (rigEvaluator.cpp:9541-9563), so a hand-to-prop constraint silently reads the other rig's unevaluated transforms. Still minor: the audition shot has no cross-character constraint channels.

Verifier evidence: `libs/rigExecImaging/registry.cpp:155-251`; `libs/rigExec/rigEvaluator.cpp:3307-3313`; `libs/rigExec/rigEvaluator.cpp:9541-9563`; `docs/spec.md:1136-1146`

### G2-null-insertion-helpers

**Add Null Above / Below and Control Stack helpers**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D20

UE features: `UE4-add-null-above`, `UE4-add-null-below`, `UE4-control-stack`

**UE rigs.** - Add Null Above: spawns an offset null at the item's initial global under its default parent, then re-parents the item keeping its global transform.
- Add Null Below: spawns a child null.
- Control Stack at Position/Item: builds null + control (initial zeroed, optionally world-oriented) + optional secondary control + bottom null, with suffixes _null/_ctrl/_sec/_ctrl_null and Self metadata.
The face rigs use the engine StandardFunctionLibrary Add Null Above for the Eye Aim Null.

**usdRig today.** Offset groups exist in two forms. default:space and default:t*/r* give an animator zero pose distinct from rest with no extra prim. Plain UsdGeomXform groups between providers are composed by the evaluator, but a non-identity one refuses the baked program. The builder supports nested controls inside Controls only, and has no insert-above, maintain-global re-parent, 'zero the control' or stack helper.

**Gap.** The builder lacks InsertNullAbove, AddNullBelow and AddControlStack with maintain-global re-parenting and zeroing semantics.

**Porting impact.** The Eye Aim Null and any stack uses must be hand-built. Modeling nulls as plain transformed Xforms silently drops the rig to dynamic evaluation (7.14 ms vs 0.71 ms on the biped).

**Recommendation.** Add RigExecRigBuilder::InsertNullAbove(provider, suffix): a guide-purpose RigExecJoint or RigExecControl with the provider's rest, moving the provider beneath it (Sdf namespace edit) and re-expressing rest parent-relatively with the migrateRestToLocal math. Also add AddControlStack(...) returning null, control, secondary and bottom handles, with Python bindings in python/_rigexec.cpp. Prefer default:space offsets over Xform groups so the rig stays bakeable.

**Evidence:** `libs/rigExecSchema/schema.usda:301-314`; `libs/rigExec/rigEvaluator.cpp:9023-9045`; `libs/rigExec/bakedProgram.cpp:253-282`; `libs/rigExecRigging/rigBuilder.cpp:2687-2702`; `docs/plans/evaluation-engine-gaps-vs-premo-libee.md:74-78`

**Verification (holds).** Confirmed:
- the baked program refuses a non-identity or animated intervening Xform above a provider (bakedProgram.cpp:253-282);
- the builder nests controls only inside Controls (rigBuilder.cpp:2686-2702);
- there is no insert-above or maintain-global helper.

Addition: offset nulls authored as nested RigExecJoint/RigExecControl follow solver-posed parents (probes), so a bakeable stack is possible without Xform groups.

Verifier evidence: `libs/rigExec/bakedProgram.cpp:253-282`; `libs/rigExecRigging/rigBuilder.cpp:2686-2702`; `libs/rigExec/rigEvaluator.cpp:9023-9048`; `libs/rigExecSchema/schema.usda:301-314`; `docs/plans/evaluation-engine-gaps-vs-premo-libee.md:74-78`

### G2-rig-internal-bones

**Rig-internal procedural bones and helper chains (spine virtual/reoriented/match, lid bones)**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** medium · **Domain:** D20

UE features: `UE2-spine-virtual-bones-percentages`, `UE6-lid-spawn`

**UE rigs.** Spine and Neck spawn several helper chains:
- '<bone>_virtual' BONE chains (fit targets).
- '_reoriented' NULL chains (spawned in LocalSpace with global values, a quirk overwritten each forward solve).
- '_match' NULL chains.
- Float metadata 'Bone Percentage' = straight-line distance / spline length.
The face spawns 24 lid bones under eye_main_l/r (identity local, pivot at the eye centre), each with a micro control (scale 0.05 Zebra / 0.03 Monster, side-negated shape offsets, red/blue colour). These rig bones are not skinned; skeleton lid bones are constrained to them.

**usdRig today.** Static equivalents are extra RigExecJoints that are not skin influences (influences are an explicit list), with RigExecControls nested for micro controls. Guide shape, colour and scale are static attributes, but there is no guide offset. Per-bone spline parameters belong to the SplineIk schema (G5). Everything under the rig is a published joint: there is no internal/non-output flag.

**Gap.** No non-published (internal) joint flag, no guide shape offset for side-mirrored micro controls, and no consumer schema for per-bone percentages.

**Porting impact.** The 24 lid bones and controls and the 9 spine/neck helper chains port as static prims. Internal bones leak into Hydra and bake output as extra joints. Micro-control shapes sit at the pivots.

**Recommendation.** Add uniform bool rigExec:internal on RigExecJoint, honored by libs/rigExecImaging/sceneIndices.cpp (guide-only) and python/rigexec/bake.py (skip). Add guide:offsetTranslate and guide:offsetRotate to RigExecControl (schema.usda:167-230, docs/control-guides.md). Generators build the chains, reproducing the reoriented-null quirk only if parity demands it.

**Evidence:** `libs/rigExecSchema/schema.usda:360-371`; `libs/rigExecSchema/schema.usda:1710-1715`; `libs/rigExecSchema/schema.usda:167-230`; `docs/biped-rig.md:180-183`

**Verification (holds).** Confirmed:
- RigExecInternalPrimPruningSceneIndex removes only generated scopes (sceneIndices.h:57-61), so every RigExecJoint is published (guide purpose) and baked.
- There is no guide offset (schema guide attribute list; biped-rig.md:180-183 explicitly wants guide:offsetX/Y/Z).
- A joint is a skin influence only if listed, so internal bones do not deform.

Verifier evidence: `libs/rigExecImaging/sceneIndices.h:57-61`; `docs/biped-rig.md:180-183`; `libs/rigExecSchema/schema.usda:167-230`; `libs/rigExecSchema/schema.usda:360-371`

### G2-socket-null-elements

**Null / socket elements that follow a bone (mesh sockets, foot pivots)**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D1 · *(analyst said Partial / major)*

UE features: `UE1-static-hierarchy-import`, `UE8-manny-reference-mesh`

**UE rigs.** Skeletal-mesh sockets become NULL elements under the socket bone. They carry the local offset, tag 'MeshSocket' and 'Tags' metadata. Zebra has 8 foot pivots (foot_[l|r]_inner/outer/heel/toe_tip on ball_[l|r]); Manny has the same names with mirror-inconsistent Y offsets. MR_Boombox has a user SOCKET element (handle_socket) with SocketDesiredParent metadata. The Foot module's pivot connectors target these nulls, and nulls are plain transforms that follow their parent bone.

**usdRig today.** No null or socket type exists. Options: (a) a leaf RigExecJoint or RigExecControl with a rest offset under the bone. It follows its namespace parent through the parent:space fallback, but NOT when the parent is solver-posed (an absolute override), so a RigExecParentConstraint with authored per-source offsets is needed (there is no maintain-offset). (b) A plain UsdGeomXform child. It is accepted as a constraint source, but its frame is the authored xform plus only the closest constraint-revised ancestor delta. A non-identity intervening Xform refuses the baked program. There is no tag system.

**Gap.** No socket/null schema whose follow semantics also work under solver-posed bones, no element tags, and no import of mesh sockets.

**Porting impact.** The 8 foot-pivot nulls used by the reverse foot, and any socket-attached prop, need hand-authored constraints with baked offsets. If nulls are simply nested under IK-posed bones they silently stay at rest.

**Recommendation.** Add a concrete RigExecSocket (inherits RigExecXformable, purpose=guide) with rel rigExec:socketParent (any joint/control, solver-posed allowed) and token[] rigExec:tags. Evaluate posed = restOffset * parentFinal, ordered in the pose DAG after the parent's solver batch (the same dependency mechanism as FkChain rigExec:startFrame). Files: schema.usda, computations.cpp (socket frame), rigEvaluator.cpp (pose DAG dependency), bakedPose.cpp (program step), rigBuilder.h (AddSocket). The importer maps UE mesh sockets and Manny/Zebra foot pivots onto it, preserving the authored (asymmetric) offsets.

**Evidence:** `libs/rigExecSchema/schema.usda:95-111`; `libs/rigExec/computations.cpp:356-380`; `libs/rigExecSchema/schema.usda:484-489`; `libs/rigExecRigging/rigBuilder.h:202-205`; `libs/rigExec/rigEvaluator.cpp:4974-4989`; `libs/rigExec/rigEvaluator.cpp:9566-9606`; `libs/rigExec/bakedProgram.cpp:253-282`; `libs/rigExecSchema/schema.usda:1115-1142`

**Verification (corrected).** The central mapping claim is wrong. A leaf RigExecJoint or RigExecControl nested under a solver-posed bone DOES follow it; no ParentConstraint is needed.

Code:
- Solver outputs are committed through commitConstraintFrames(..., solverOutput=true) (rigEvaluator.cpp:10971-10973).
- That function re-poses every unclaimed hierarchical descendant by the delta of the closest changed ancestor. The skip applies only when a DIFFERENT pose owner sits between them (rigEvaluator.cpp:10690-10704, nearestBlocking at 10629-10654).
- The 'absolute override' wording in schema.usda:484-489 and rigBuilder.h:198-206 is about propagation THROUGH a joint that is itself solver-claimed (finger chains), not about unclaimed children.
- testRigExecConstraints.cpp:3068-3072 states the bound joint's own descendant 'stays with it'.

Runtime probes (existing build, dynamic, parity and baked modes, parity 0, bakeable):
- An unclaimed joint under an FK-rotated joint moved to (0,5,0).
- Pivot/Deeper joints and a nested control under a TwoBoneIk end joint moved from (2,0,1) to (6,3,1) as the effector moved.
- A PositionConstraint sourced on that nested joint followed to (6,3,1).

The real pitfall is option (b): a plain UsdGeomXform null under the IK joint, read as a constraint source, stayed at (2,0,1) (probe_follow3).

What remains missing: a socket/null type with tags ('MeshSocket', SocketDesiredParent), an importer for mesh sockets, and the builder cannot nest a control under a joint (rigBuilder.cpp:2686-2702).

The Foot module mostly reads the pivots' initial transforms, so the impact is minor.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:10629-10654`; `libs/rigExec/rigEvaluator.cpp:10690-10704`; `libs/rigExec/rigEvaluator.cpp:10971-10973`; `libs/rigExec/rigEvaluator.cpp:9567-9608`; `libs/rigExec/rigEvaluator.cpp:10459-10491`; `libs/rigExec/computations.cpp:336-383`; `tests/testRigExecConstraints.cpp:2977-3082`; `libs/rigExecRigging/rigBuilder.cpp:2686-2702`; `reports/ue-zebrasample/probes/g2review/probe_follow.py`; `reports/ue-zebrasample/probes/g2review/probe_follow2.py`; `reports/ue-zebrasample/probes/g2review/probe_follow3.py`

### G2-xform-null-constraint-read

**Plain UsdGeomXform nulls under solver-posed joints are stale when read by constraints**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D1 · *added by verifier*

UE features: `UE1-static-hierarchy-import`, `UE1-runtime-hierarchy-census`, `UE-deformer-pivot-nulls`

**UE rigs.** UE NULL elements (326 in MR_Zebra, 85 in MR_Monster, plus mesh-socket nulls) are plain transforms that always follow their parent, whether the parent is FK-, IK- or spline-posed. Other solves read them every frame with GetTransform(global).

**usdRig today.** Mapping a UE null to a nested RigExecJoint or RigExecControl works: it follows solver-posed parents and is a valid constraint source.

Mapping it to a plain UsdGeomXform does not work for evaluator reads:
- A native source frame is its stage transform relative to the asset root, plus the delta of the closest ancestor whose base and current frames differ (rigEvaluator.cpp:9567-9608).
- Solver commits write both baseFrames and finalFrames (rigEvaluator.cpp:10740-10748), so a solver-posed joint never counts as 'revised'.
- In Hydra the Xform still rides its published parent, so the viewport and the evaluator disagree.

Probe: a PositionConstraint sourced on an Xform under a TwoBoneIk end joint stayed at (2,0,1) while the joint moved to (4,3,0). The same setup with a RigExecJoint source followed to (6,3,1). No diagnostic was emitted and the rig remained bakeable.

**Gap.** There is no diagnostic and no correct follow when a constraint source or up-object is a native Xform beneath a solver-bound joint. The follow semantics silently differ between Hydra and the evaluator.

**Porting impact.** An importer that maps UE nulls (foot pivots, deformer frames, space nulls) to plain Xforms produces rigs that look right in the viewport but whose constraints read rest positions under IK/FK. The errors are silent.

**Recommendation.** Two changes in the evaluator:
1. In RigExecApplyRevisedAncestorDelta (rigEvaluator.cpp) and its baked twin (bakedPose.cpp:2779), treat a solver-bound ancestor as revised by comparing its committed frame with its unsolved exec frame. Equivalently, record a separate pre-solve frame for solver joints in enumerateProviderFrames.
2. Until then, emit a compile diagnostic in the frame-source binding (rigEvaluator.cpp around 4970-4990) when a native Xform source has a solver-bound ancestor.

Importers and generators should emit UE nulls as nested RigExecJoint (purpose guide).

**Evidence:** `libs/rigExec/rigEvaluator.cpp:9567-9608`; `libs/rigExec/rigEvaluator.cpp:10459-10491`; `libs/rigExec/rigEvaluator.cpp:10740-10748`; `libs/rigExec/bakedPose.cpp:2779`; `reports/ue-zebrasample/probes/g2review/probe_follow3.py`

### G2-construction-queries

**Construction-time hierarchy queries and list building**

**Verdict:** Divergent-by-design · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D17

UE features: `UE4-children-by-contained-strings`, `UE4-get-chain-length`, `UE8-array-union-clone`

**UE rigs.** Get Children by Contained Strings finds children by case-sensitive token match (LimbTwist uses 'upperarm|twist' to find twist bones). Get Chain Length sums initial-pose segment lengths, and 5 modules divide it by a per-module reference (55, 96, ...) to auto-scale shapes. ArrayUnion/ArrayClone build de-duplicated control lists for Set Control Scale.

**usdRig today.** Membership lists are static relationships (rigExec:joints, controls, influences) authored by the builder. Guide size is static guide:scaleX/Y/Z. Nothing is queried at evaluation, although TwoBoneIk and SplineIk re-measure rest lengths live.

**Gap.** No builder-side query helpers, and guide scale is not derived from chain length (it does not follow rest edits).

**Porting impact.** The builder must reproduce the twist-bone search and the chain-length/reference scale, or gizmo sizes will differ (cosmetic).

**Recommendation.** Add python/rigexec/hierarchy_query.py with children_by_tokens(prim, tokens, recursive, type_name), chain_length(joint_paths, use_rest=True) and ordered_union(). Optionally add rel guide:scaleReferenceChain plus float guide:scaleReferenceLength, resolved in libs/rigExecImaging/sceneIndices.cpp from live rest frames.

**Evidence:** `libs/rigExecSchema/schema.usda:201-215`; `libs/rigExecSchema/schema.usda:106-111`; `README.md:36-45`

**Verification (holds).** Confirmed:
- membership is static relationships (schema.usda:106-111);
- guide size is static guide:scaleX/Y/Z (schema.usda:201-215);
- TwoBoneIk re-measures rest lengths live (README.md:36-45).
The queries are construction-only in UE, so they belong in the builder.

Verifier evidence: `libs/rigExecSchema/schema.usda:106-111`; `libs/rigExecSchema/schema.usda:201-215`; `README.md:36-45`

### G2-item-metadata

**Typed item/module metadata as inter-element pointers and settings bus**

**Verdict:** Divergent-by-design · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D1

UE features: `UE4-metadata-namespace-convention`, `UE4-unit-metadata`

**UE rigs.** SetMetadata/GetMetadata are typed dispatches (bool, vector, element key, color, name arrays) with namespaces None/Self/Parent/Root. Module metadata forms a rig-wide settings bus. Libraries use metadata as typed pointers ('Gimbal Control', 'Null', 'Bottom Null', 'Secondary', 'Item') and tags ('Controls'). A missing key returns Default with Found=false. 'IsSet' is written at runtime on proxy buffers.

**usdRig today.** Wiring is static relationships and connections resolved at compile, with no membership or metadata lookups. Arbitrary data can sit in USD customData but evaluation never reads it. Construction-time lookups belong in the offline builder. Runtime metadata writes contradict the no-hidden-state and never-author rules.

**Gap.** No declared relationships for common element links (offset null, secondary, source item, gimbal), no tag attribute for tool queries, and no runtime-mutable per-element state (by design).

**Porting impact.** Pointer metadata becomes builder-authored relationships. Tools that locate a control's null or gimbal need a relationship convention. The IsSet runtime flag has to be redesigned (see G2-stateful-interaction).

**Recommendation.** Add optional declared relationships to RigExecControlAPI: rigExec:offsetNull, rigExec:secondary, rigExec:sourceItem, rigExec:gimbal. Add uniform token[] rigExec:tags. Keep everything else as builder-written customData. Do not add runtime metadata writes.

**Evidence:** `libs/rigExecSchema/schema.usda:106-111`; `docs/spec.md:59`; `README.md:22-23`; `libs/rigExecSchema/schema.usda:39-41`

**Verification (holds).** The cited non-goals exist:
- no hidden stateful simulation (spec.md:59);
- never authoring onto the stage (README.md:22-23);
- keyability, mirroring and presentation metadata deferred (schema.usda:39-41);
- RigExecRoot declares no membership lists (schema.usda:106-111).

grep finds no tag system (no rigExec:tags or similar) in libs, python or plugin. Construction-time pointer metadata maps naturally onto builder-authored relationships.

Verifier evidence: `docs/spec.md:59`; `README.md:22-23`; `libs/rigExecSchema/schema.usda:39-41`; `libs/rigExecSchema/schema.usda:106-111`

### G2-naming-helpers

**Construction-time naming functions (Has Side, Conform Name, Get Item Name, Rename Joint to Control)**

**Verdict:** Divergent-by-design · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D17

UE features: `UE4-has-side`, `UE4-conform-name`, `UE4-get-item-name`, `UE2-fk-naming`, `UE6-rename-joint-to-control`, `UE4-rigvm-name-string`, `UE8-local-shape-and-name-helpers`

**UE rigs.** String helpers run during construction:
- Has Side: case-insensitive side-token test.
- Conform Name: snake_case to 'Upperarm L', with optional side removal; empty tokens give double spaces.
- Get Item Name: 'FK 0' numbered names vs bone-derived names ('Spine 01 Sec FK'); the start index is 0 at the FkChain call site.
- Rename Joint to Control: 12 ordered NameReplace rules.
- Module-local Get Control Name / Get Control Shape (the latter cancels the control offset in the shape transform).
FName operations are case-insensitive; FString operations are case-sensitive.

**usdRig today.** There are no in-rig string operators (no general node graph, no construction at evaluation). Names are authored once by an offline builder, and the repo ships no naming utility: tools/biped/rig_names.py is referenced but absent.

**Gap.** No offline helper library that reproduces the UE control names and their case semantics.

**Porting impact.** Generated names are the keys for sequencer channel matching ('Arm L/FK 0'), so the importer must reproduce them exactly, including quirks.

**Recommendation.** Add python/rigexec/ue_naming.py with conform_name, get_item_name, has_side and rename_joint_to_control, mirroring RigVM semantics (IgnoreCase for FName operations, case-sensitive FString operations, Split keeps interior empty tokens). Unit-test it against runtime_hierarchy.txt names from the dump. The importer uses it to set displayName and the name table.

**Evidence:** `docs/spec.md:58-60`; `docs/biped-rig.md:24-27`; `libs/rigExecRigging/rigBuilder.h:1-24`

**Verification (holds).** Confirmed:
- spec.md:60 rules out a general-purpose node graph, so there are no in-rig string operators.
- python/rigexec contains only __init__.py, bake.py, curvenet.py and inverse.py, so no naming utility ships.
- tools/ has no biped/ folder (rig_names.py is referenced in docs only).
The port needs an offline helper; minor.

Verifier evidence: `docs/spec.md:58-60`; `docs/biped-rig.md:24-27`; `docs/biped-rig.md:229`; `libs/rigExecRigging/rigBuilder.h:1-24`

### G2-physics

**Physics assets (capsule/convex bodies, limited constraints)**

**Verdict:** Divergent-by-design · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D18

UE features: `UE-physics-assets`

**UE rigs.** There are four physics assets:
- SKM_Zebra_Physics (mesh default): 22 capsules, 21 constraints.
- PA_Zebra: 21 capsules, 20 swing/twist-limited constraints; used only by the retargeter's body-intersect op.
- PA_Zebra_Phys_Asset_Detailed: 27 convex bodies, unused.
- SKM_Monster_Physics: 1 capsule.

**usdRig today.** Simulation inside computations is a non-goal, and the repo has no UsdPhysics support (grep). Collision bodies are static data that stock UsdPhysics schemas can carry for other consumers.

**Gap.** No physics bodies or constraints in the rig model (by design).

**Porting impact.** None for the animation path. Ragdoll and contact-aware retargeting are unavailable.

**Recommendation.** Carry the bodies as UsdGeomCapsule + UsdPhysicsCollisionAPI prims nested under the matching RigExecJoint, in a separate physics layer for downstream simulators. Any future contact-aware solve goes into an explicit multipass stage (spec §6.2), not into the graph.

**Evidence:** `docs/spec.md:59`; `docs/spec.md:1136-1146`

**Verification (holds).** spec.md:59 exists: simulation may be integrated only through explicit state. No UsdPhysics usage exists in libs, python or rigExecUsdview; the usdNoodles generic prim library merely lists usdPhysics types for authoring. Not-applicable would also be defensible because the bodies are static engine data.

Verifier evidence: `docs/spec.md:59`; `docs/spec.md:1136-1146`; `plugin/usdNoodles/nodeLibs/usdPrimLibrary.py:20`

### G2-rigvm-control-flow

**RigVM control flow and generic dispatch (Sequence, Branch, If, loops, locals)**

**Verdict:** Divergent-by-design · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D17 · *(analyst said Divergent-by-design / major)*

UE features: `UE4-rigvm-control-flow-dispatch`

**UE rigs.** RigVM provides Sequence, Branch, If, ArrayIterator, ArrayGetAtIndex (negative index counts from the end), Find/Add/Reset/Make/GetNum, SelectInt32, CastEnumToInt, MakeStruct, Print, and function-local variables. These are used heavily both at construction and in per-frame solves (CRFL_Hierarchy alone: 18 Sequence, 17 Branch, 12 If).

**usdRig today.** There is no general node graph; topology and order are fixed per epoch. Per-frame choice is expressed through weights, inputs:enabled (value-only) and five linear FloatMathMover operations. There is no comparison, step or select operator and no integer/enum switch. Loops are unrolled by the builder.

**Gap.** No per-frame scalar conditional (step/select/min/max/abs) and no enum switch, so discrete runtime branches on channel values (e.g. Root 'Bake Root On') have no direct form.

**Porting impact.** Construction branches move to the builder. Runtime branches must become 0/1 envelopes, which works only where the selector is an authored float, not a solved pose.

**Recommendation.** Add FloatMathMover operations step (r = base >= value ? 1 : 0), select (r = base >= 0.5 ? max : min), min, max and abs in libs/rigExecMath/propertyMath.{h,cpp}. Extend the schema allowedTokens and the baked property step, keeping them stateless. Construction-time control flow stays in Python generators.

**Evidence:** `docs/spec.md:60`; `docs/spec.md:335`; `libs/rigExecSchema/schema.usda:69-78`; `libs/rigExecSchema/schema.usda:1208-1213`; `libs/rigExecMath/propertyMath.h:25-60`

**Verification (corrected).** The non-goal exists (spec.md:60), so the verdict holds, but major overstates the runtime impact.

The forward-solve branch census from the dump shows most runtime branches are expressible:
- Config variables and module metadata (Foot 3 metadata, IkFk2Bones and Body variables) are construction constants; the builder bakes them.
- Bool animation channels (Prop 4, IkFk2Bones 1) map to inputs:enabled on constraints and movers, which accepts a validated Bool connection (rigEvaluator.cpp:3625-3650).
- Float-channel '> 0' tests (Body graphs.txt:137-179) can be approximated with remap (tiny span) plus clamp.
- min and max already exist as clamp with one bound connected (propertyMath.cpp:28-61).
- IsInteracting branches belong to G2-stateful-interaction.

The row's own example is misplaced: 'Bake Root On' is an enum Switch in Root's BACKWARDS solve (CRM_FN_Root graphs.txt:212,225,266), i.e. G4.

Two real residual gaps:
- solvers have no inputs:enabled (only MoverAPI declares it, schema.usda:69-72), so a branch that selects a whole solve needs a BlendPointFrames weight;
- the pose-derived sign split in CR_Zebra_Deform (graphs.txt:192-195, 555-560) needs pose interpolators (G6).

Verifier evidence: `docs/spec.md:60`; `libs/rigExec/rigEvaluator.cpp:3625-3650`; `libs/rigExecMath/propertyMath.cpp:28-61`; `libs/rigExecSchema/schema.usda:69-72`; `ue/<dump>/FortniteRigs__Modules__FkSolves__Root__CRM_FN_Root/graphs.txt:212`; `ue/<dump>/FortniteRigs__Modules__FkSolves__Root__CRM_FN_Root/graphs.txt:266`; `ue/<dump>/FortniteRigs__Modules__FkSolves__Body__CRM_FN_Body/graphs.txt:137`; `ue/<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:555-560`

### G2-seq-embedded-snapshot

**Sequences embedding a (possibly stale) serialized rig instance**

**Verdict:** Divergent-by-design · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D18

UE features: `UE8-seq-embedded-rig-snapshot`

**UE rigs.** Each Control Rig track and section stores a serialized UControlRig/UModularRig (model, connections, overrides, bindings). The MR_Boombox snapshot in zebra_audition is stale relative to the asset (it lacks the AddControl_5/_6 connectors yet has their control channels). Ports should rebuild from the asset and match channels by full control name.

**usdRig today.** A shot references the rig asset live, so no embedded copy exists and cannot go stale; animation binds to channels by prim path. If controls are renamed without relocates, shot overs become orphaned silently.

**Gap.** No detector for orphaned animation opinions after rig renames.

**Porting impact.** The importer must resolve channels against the current asset and ignore the stale snapshot.

**Recommendation.** Add an 'orphaned animation opinions' check to plugin/rigExecUsdview/layerOpinionsModel.py that flags overs under a RigExecRoot with no defining spec, and report it in rigExecPose diagnostics.

**Evidence:** `examples/ArmShotAnim.usda:9-16`; `docs/spec.md:348-352`

**Verification (holds).** Shots reference the asset live (ArmShotAnim.usda:11-16), so no embedded rig copy exists. layerOpinionsModel.py exists to host the proposed orphan check.

Verifier evidence: `examples/ArmShotAnim.usda:11-16`; `docs/spec.md:348-352`

### G2-bone-axis-convention

**Bone-axis and side conventions, Euler aliasing on import**

**Verdict:** Implemented · **Severity:** minor · **Effort:** S · **Confidence:** medium · **Domain:** D1

UE features: `UE8-bone-axis-convention`

**UE rigs.** Manny, Zebra and Monster share a pelvis frame (X up, Y forward, Z = -X world). Monster's 'flipped' pelvis Euler angles are gimbal aliasing near pitch 90, not a different orientation. Left chains have +X child offsets; right chains have negated translations, with X also pointing +X in world space (legs have inverted signs). Importers must compare matrices or quaternions, never Euler triples.

**usdRig today.** Rest frames are matrices (rest:space), parent-relative and orthonormalized, so Euler aliasing never enters the rig. Mirrored right chains are simply different rest matrices; the biped right side authors mirrored rest:space. The rest:rx..rz channels use a fixed XYZ order, so importers should author rest:space matrices, with the stage unit and up axis declared (as in the biped).

**Gap.** No UE-to-USD basis/handedness conversion helper.

**Porting impact.** A wrong axis flip mirrors the character or breaks side detection. The negated-translation convention must be preserved for twist axes and mirroring.

**Recommendation.** Add python/rigexec/ue_convert.py: ue_transform_to_usd(...) using the same basis change as UE's USD exporter, validated against Monster.usd joint rests. Include quaternion-based comparison utilities and unit tests on the Manny, Zebra and Monster pelvis frames.

**Evidence:** `libs/rigExecSchema/schema.usda:292-300`; `docs/biped-rig.md:243-248`; `examples/biped/Biped.usda:1-6`

**Verification (holds).** Rest is a parent-relative, orthonormalized matrix (schema.usda:298-300). Every Zebra and Monster reference bone has unit scale (bones.txt), so orthonormalization is lossless. The biped declares upAxis and units (Biped.usda:1-6). A UE-to-USD basis helper is still needed.

Verifier evidence: `libs/rigExecSchema/schema.usda:292-300`; `docs/biped-rig.md:243-248`; `examples/biped/Biped.usda:1-6`

### G2-deformer-frame-nulls

**Construction-spawned deformer frame nulls under the head**

**Verdict:** Implemented · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D20 · *(analyst said Partial / minor)*

UE features: `UE-deformer-pivot-nulls`, `UE7-deformer-origin-nulls`

**UE rigs.** The construction event spawns 7 nulls (CR_Zebra_Deform) or 9 (CRM_Monster_Face) parented to bone 'head' at GlobalSpace initial transforms (e.g. Head Bend Null: 90 deg about Z at (0,0,90); Muzzle Bend Null: quat(-.7071,-.7071,0,0) at (0,20,110)). Their keys are stored in variables. Every frame GetTransform(global) feeds the matching deformer's origin, and the deformers act along the null's local +Z.

**usdRig today.** The frames can be static prims authored by the builder: a leaf RigExecJoint (purpose guide) with rest:space = nullGlobal * headRest^-1 (parent-relative rest). The head is spline-IK posed in the animator rig, so following it needs a ParentConstraint with baked offsets. Any computeMatrix provider can feed a mover (MatrixMover rigExec:transform), but no bend/twist/squash mover exists to consume the frame (frozen op set).

**Gap.** No construction step (the frames must be pre-authored), follow needs explicit constraints, and there are no parametric deformer consumers (G7).

**Porting impact.** The 16 nulls port trivially as static prims, but they serve no purpose until G7's parametric movers exist.

**Recommendation.** The importer emits <rig>/Joints/.../head/DeformFrames/<name> leaf joints plus one ParentConstraint per frame (or a RigExecSocket, see G2-socket-null-elements). When G7 adds bend/twist/squash movers, give them rel rigExec:originFrame, accepting any frame provider with rigExec:transformReadPhase=final. Add a builder helper AddFrameUnder(parentJoint, globalMatrix) that computes the parent-relative rest.

**Evidence:** `libs/rigExecSchema/schema.usda:298-300`; `libs/rigExecSchema/schema.usda:484-489`; `libs/rigExecSchema/schema.usda:1663-1679`; `libs/rigExec/moverGraph.h:59-73`; `libs/rigExec/moverGraph.cpp:193-196`

**Verification (corrected).** The claim that following the spline-IK-posed head 'needs a ParentConstraint with baked offsets' is false.
- A leaf RigExecJoint (purpose guide) nested under the head, with rest:space = nullGlobal * headRest^-1, follows any solver pose of the head. The solver commit propagates to unclaimed descendants (rigEvaluator.cpp:10690-10704, 10971-10973), confirmed by probes in all three evaluation modes.
- Its frame is a normal computePointFrame/computeMatrix provider, readable at a final phase.

The other two gap items belong to other rows:
- construction-time spawning is Divergent-by-design (G2-construction-event);
- the bend/twist/squash consumers are missing (G7, frozen op set at moverGraph.h:59-73).

Note for G7: existing movers take a rest-to-posed computeMatrix, while these kernels need the absolute frame (origin plus local +Z), so the new movers need a frame input. The 16 nulls themselves port as static nested joints.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:10690-10704`; `libs/rigExec/rigEvaluator.cpp:10971-10973`; `libs/rigExecSchema/schema.usda:298-300`; `libs/rigExecSchema/schema.usda:1663-1679`; `libs/rigExec/moverGraph.h:59-73`; `reports/ue-zebrasample/probes/g2review/probe_follow2.py`

### G2-helper-joint-layout

**Helper and twist joints as leaf children of their driving bone**

**Verdict:** Implemented · **Severity:** cosmetic · **Effort:** S · **Confidence:** medium · **Domain:** D1 · *(analyst said Implemented / minor)*

UE features: `UE-helper-joint-layout`

**UE rigs.** SK_Zebra has 20 def_* helper bones (strap, chest, back, elbow in/ot, knee in/ot, thigh in/ot) and 32 twist bones (4 per segment), all leaf children of the segment bone. The animator rig drives the twist bones, and CR_Zebra_Deform adds volume offsets on top. Right-side bones carry negated translations.

**usdRig today.** Nested leaf RigExecJoints with parent-relative rest express this layout exactly, and any RigExecJoint can be a skin influence. Driving is covered by G5 (twist) and G6 (correctives). A helper whose parent is solver-bound follows only if it is itself claimed by a solver or constrained.

**Gap.** Nothing structural is missing. Helpers under solver-posed bones need explicit follow drivers.

**Porting impact.** The 52 helper and twist joints port as joints. Each needs a follow driver (ParentConstraint or solver claim) before its corrective offset is applied.

**Recommendation.** The importer auto-emits a RigExecParentConstraint (source = parent bone, rest-relative offsets) for every unclaimed joint whose parent is solver-claimed. Alternatively, implement the socket-style follow from G2-socket-null-elements so plain nested joints follow solver-posed parents.

**Evidence:** `libs/rigExecSchema/schema.usda:360-371`; `libs/rigExecSchema/schema.usda:298-300`; `libs/rigExec/rigEvaluator.cpp:6981-6990`; `libs/rigExecSchema/schema.usda:484-489`

**Verification (corrected).** The verdict stands, but the stated gap and recommendation are wrong. Helpers do not need explicit follow drivers.
- Unclaimed nested joints ride the solved frame of their solver-posed parent (rigEvaluator.cpp:10690-10704, 10971-10973), verified at runtime by the nested-joint probes.
- Auto-emitting a ParentConstraint for every unclaimed helper under a solver-claimed parent would add about 52 redundant constraint steps for no change in result.
- Twist bones claimed by a TwistDistribution are independent owners, which is also fine.
- Any RigExecJoint can be a skin influence (rigEvaluator.cpp:6981-6990).

The real missing piece, the pose-driven corrective offsets on top, is carried by G2-postprocess-pass and G6. Nothing structural is missing here.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:10690-10704`; `libs/rigExec/rigEvaluator.cpp:10971-10973`; `libs/rigExec/rigEvaluator.cpp:6981-6990`; `tests/testRigExecConstraints.cpp:3068-3072`; `reports/ue-zebrasample/probes/g2review/probe_follow2.py`

### G2-lod

**Rig/deformation LOD**

**Verdict:** Implemented · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D18

UE features: `UE-skm-runtime-config`, `UE-postprocess-abp`

**UE rigs.** All meshes have 1 LOD, and the post-process ControlRig node has LODThreshold=-1 (runs at every LOD).

**usdRig today.** Always-on is the default. Authored LOD exists as the rigComplexity variant set with inputs:enabled overrides; a variant switch starts a new epoch.

**Gap.** No automatic distance/screen-size LOD. RigExecPartitionAPI LOD hints exist only in the spec, and rigExec:partition is inert.

**Porting impact.** None for these assets.

**Recommendation.** None required. If runtime LOD is needed, implement RigExecPartitionAPI hints consumed by libs/rigExecImaging/registry.cpp to toggle inputs:enabled without recompiling.

**Evidence:** `examples/ArmRig.usda:8-13`; `examples/ArmShotAnim.usda:11-16`; `docs/spec.md:354`; `docs/spec.md:263`

**Verification (holds).** Confirmed:
- the rigComplexity variant is used in ArmRig.usda:8-13 and selected in ArmShotAnim.usda:11-16;
- a variant switch is structural (spec.md:345);
- RigExecPartitionAPI does not exist in schema.usda (grep), and rigExec:partition is read by nothing outside the builder.

Verifier evidence: `examples/ArmRig.usda:8-13`; `examples/ArmShotAnim.usda:11-16`; `docs/spec.md:345`; `docs/spec.md:263`

### G2-reuse-composition

**Cross-character reuse (duplicated deform sections, skeleton copies)**

**Verdict:** Implemented · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D17

UE features: `UE-reuse-by-duplication`

**UE rigs.** There is no shared library between characters; assets were copied. CR_Monster_Deform's clavicle section is a copy of Zebra's (same node names, identical def_trap/def_strap rests). Zebra kernels were generated from Monster graphs. SK_ZebraHi is an unreferenced copy, and ZebraMuzzleTwist is never added.

**usdRig today.** USD references and sublayers share rig data without copying. The biped right side references the left layer and authors only the differences, and unused assets simply go unreferenced.

**Gap.** Nothing for reuse itself. Shared layers must keep relationships under a common ancestor (the rebasing limit).

**Porting impact.** The clavicle/trapezius corrective can be one shared component with per-character overrides (def_trap vs def_strap). Unused assets need not be ported.

**Recommendation.** Author a shared 'clavicle_helpers' corrective component with relationships relative to its own scope, and override the bone names per character.

**Evidence:** `docs/biped-rig.md:243-259`; `examples/biped/Biped_stack.usda:1-20`

**Verification (holds).** Confirmed: the right side references the left layer file and overrides the differences (biped-rig.md:243-259), and relationships must stay inside the arc (biped-rig.md:279-290).

Verifier evidence: `docs/biped-rig.md:243-259`; `docs/biped-rig.md:279-290`; `examples/biped/Biped_stack.usda:1-20`

### G2-seq-blend-layers

**Control Rig section blend type, weight and masks**

**Verdict:** Implemented · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D18

UE features: `UE8-seq-blend-weight-mask`

**UE rigs.** All saved sections are Absolute (the default), with weight 1.0, no weight keys, no control or transform masks, and no layered sections; AnimLayers user data is present but unused.

**usdRig today.** One shot animation layer gives exactly one resolved opinion per channel, which equals a single absolute, weight-1 section. Additive or weighted animation layers and masks do not exist, because there is no custom animation schema.

**Gap.** No additive/override animation layers with weights or masks. These sequences do not use them.

**Porting impact.** None for the shipped sequences.

**Recommendation.** None for this port. If animation layers are needed later, implement them as an authoring-time merge in the graph editor, not as an evaluation schema.

**Evidence:** `docs/spec.md:348-352`; `docs/spec.md:1104`

**Verification (holds).** A single absolute weight-1 section maps to one resolved USD opinion. There is no custom animation schema (spec.md:1104).

Verifier evidence: `docs/spec.md:348-352`; `docs/spec.md:1104`

### G2-seq-nonrig-tracks

**Non-rig sequencer tracks (actor transform/spawn, cameras, lights, material parameters, subsequences)**

**Verdict:** Implemented · **Severity:** cosmetic · **Effort:** S · **Confidence:** medium · **Domain:** D18 · *added by verifier*

UE features: `UE-level-sequences`

**UE rigs.** The shots animate more than the rigs:
- zebra_audition has Spawn and Transform tracks per actor, static props, lights, a cine camera and a camera-cut track.
- zebra_marketingPoseFaces animates camera focal length, focus, aperture and exposure.
- expression_demo_seq animates Eyes-slot material parameters (Highlight_Intensity, Cornea Roughness, Sclera Color Multiply RGBA, Highlight_Shape_Pos RGBA) and pulls in subsequences.

**usdRig today.** These are plain USD data that usdRig neither needs nor blocks:
- animated xformOps on the Xform that references the asset (ArmShotAnim.usda:9-16);
- UsdGeomCamera and UsdLux attribute samples;
- UsdShade input samples or splines;
- animated visibility for spawn;
- sublayers with layer offsets for subsequences.

The usdRig graph editor edits scalar splines only (graph-editor.md:18-27), so vector and color inputs are keyed as time samples.

**Gap.** No UE sequence importer covers these tracks. Spawn can map only to visibility, because 'active' is structural. USD has no standard camera-cut schema.

**Porting impact.** Shot context (cameras, lights, eye-material animation) must be converted separately from the rig channels.

**Recommendation.** Cover these tracks in the proposed python/rigexec/ue_sequence_import.py:
- spawn becomes animated visibility;
- camera cuts become per-shot camera relationships or separate shot layers;
- material parameters become UsdShade input time samples.

**Evidence:** `examples/ArmShotAnim.usda:9-20`; `docs/graph-editor.md:18-27`; `docs/spec.md:1094-1104`

### G2-asset-hygiene

**Unused/missing assets and plugin dependencies**

**Verdict:** Not-applicable · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D18

UE features: `UE8-unused-missing-assets`, `UE8-project-plugins`

**UE rigs.** Several assets are unused or dangling: ZebraMuzzleTwist_DeformerGraph, SK_ZebraHi and PA_Zebra_Phys_Asset_Detailed are unused, and the preview meshes /Game/Assets/Zebra/Geo/* are missing. The project enables DMC, AnimatorKit, RigMapperOp, CurveExpression and others. DeformerGraph arrives only transitively, and FortniteRigs is a content-only plugin.

**usdRig today.** USD composition opens stages with unresolved asset paths, reporting composition errors. usdRig's own runtime needs only the schema, imaging and Python plugin paths. The DeformerGraph dependency has no counterpart until G7.

**Gap.** None in usdRig.

**Porting impact.** The importer should skip unused assets and tolerate dangling references.

**Recommendation.** The importer ignores ZebraMuzzleTwist, SK_ZebraHi, PA_Zebra_Phys_Asset_Detailed and dangling preview meshes. Document the required PXR_PLUGINPATH_NAME entries for the ported asset.

**Evidence:** `README.md:471-493`

**Verification (holds).** Confirmed: this is engine plugin and asset bookkeeping. The UE ObjectRedirectors (SK_Mannequin to SK_Manny, SKM_Manny_Simple to SKM_Manny) likewise have no rigging meaning; USD asset paths or resolver remaps cover them.

Verifier evidence: `README.md:471-493`

### G2-procedural-element-limit

**ProceduralElementLimit**

**Verdict:** Not-applicable · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D20

UE features: `UE1-procedural-element-limit`

**UE rigs.** HierarchySettings.ProceduralElementLimit caps construction spawning, sized as static element count + 2000 (Zebra 3486, Biped 3106, Monster 3184, Boombox 2024, PropAim 2004).

**usdRig today.** Nothing is spawned during evaluation, so the element count is whatever the composed stage holds and compile validates it. Discovery cost scales at about 14 µs per prim.

**Gap.** None.

**Porting impact.** None.

**Recommendation.** None needed. Optionally report the rig prim count in compile diagnostics.

**Evidence:** `docs/spec.md:58`; `examples/biped/README.md:226-229`

**Verification (holds).** Nothing is spawned during evaluation (spec.md:58). The per-prim discovery cost is documented (examples/biped/README.md:226-229).

Verifier evidence: `docs/spec.md:58`; `examples/biped/README.md:226-229`

## G3 — Controls, animation channels and animator UX

usdRig can already express the basic control layer of the Zebra/Monster rigs. Controls nest by namespace. Custom animation channels are custom avars: attributes wired by same-type connections. Rotation orders are per prim. Side colours can be authored or connected to a shared palette. IK/FK set fading is built in: guide:displayOpacity can be connected, with an invert flag and a minimum floor. The animator UX those rigs depend on is mostly missing or only partial, for five reasons:
- A channel lives on exactly one prim and cannot be hosted on other controls.
- There are no per-channel limits or locks, although the schema text promises 'computational limits'.
- Visibility is an opacity fade driven only by float or double sources, not a bool-driven hide that also stops picking.
- Guide shapes are 6 unit primitives with per-axis scale only: no shape library, no shape offset or rotation, and no draw-on-top option.
- There is no proxy-control or driven-list concept.
Several UE behaviours depend on viewport gesture state or on state persisted between evaluations: movable pivots, delta proxies, Foot Rocker re-anchoring, the dormant IK/FK auto-switch and Prop change-pivot auto-key. These are Divergent-by-design. The usdRig-native alternative is tool-side behaviour in the usdview gizmo, previewed through SetInteractiveOverrides, with the evaluator kept stateless. Debug drawing is limited to fixed guide families (joint and solver spheres/cones, volume iso-surfaces, curvenets). Pole-vector lines, control-path rays, axis triads and stretch feedback are missing, but UE ships most of these switched off. Most gaps close with codeless, imaging-only or tooling-only additions that fit the non-destructive architecture:
- A ChannelHostAPI relationship.
- guide:visible with visibility drivers.
- guide:shapeSource and guide:shapeTransform.
- RigExecProxyControlAPI.
- A RigExecGuideLine prim.
- Registered channel metadata for type, range and enum labels.
Two changes do touch evaluation: fixed limit attributes clamped in computePointFrame (dynamic and baked paths), and lossless bool/float/double coercion in the input resolver. Both are needed to port the face pads, the lid sliders and the 69 bool channels faithfully.

| Row | Verdict | Severity | Effort | Summary |
|---|---|---|---|---|
| [`G3-channel-hosts`](#g3-channel-hosts) | Missing | major | S | No channel-host concept: Avar Editor shows only the focus prim's avars; picker buttons cycle defaults, never key |
| [`G3-control-limits`](#g3-control-limits) | Missing | major | M | No per-avar min/max, clamp or lock; FloatMathMover clamps floats only; slider ranges hard-coded; no limit drawing |
| [`G3-body-layout`](#g3-body-layout) | Partial | major | L | Maps to nested RigExecControls and avars; controls under solver-posed joints follow; relays only for Xform-null parents |
| [`G3-bool-vis-switches`](#g3-bool-vis-switches) | Partial | major | M | Only guide:displayOpacity is channel-drivable (float or double source; bools rejected); visibility is not |
| [`G3-shape-transform-offset`](#g3-shape-transform-offset) | Partial | major | S | Only positive guide:scaleX/Y/Z; no shape offset/rotation (guide:offsetX/Y/Z) or mirrored scale; co-pivot guides overlap |
| [`G3-switch-keying`](#g3-switch-keying) | Partial | major | S | No panel keys stepped switches: Avar Editor writes bool/token defaults, picker a float default, float dials get AutoEase |
| [`G3-display-toggles`](#g3-display-toggles) | Missing | minor | S | Guides draw at purpose default; no session control-guide show/hide, show-all-proxies override or controls-only picking |
| [`G3-mirror-metadata`](#g3-mirror-metadata) | Missing | minor | M | RigExecControlAPI mirror metadata is deferred; no Mirror Behavioral flag or Mirror Axis vector, no pose mirror/flip tool |
| [`G3-multi-edit-channels`](#g3-multi-edit-channels) | Missing | minor | S | Avar Editor edits only the focus prim and GroupTarget moves transforms only; no multi-prim channel or += / *= entry |
| [`G3-control-scale`](#g3-control-scale) | Partial | minor | S | No global/module control-scale knob; guide:scaleX/Y/Z read by plain Get (no connections), so scale is baked per control |
| [`G3-control-value-types`](#g3-control-value-types) | Partial | minor | S | All RigExecControls carry full tx..sz avars; no channel mask or keyable flag; rigExec:channelRole never read |
| [`G3-delta-proxy-profile`](#g3-delta-proxy-profile) | Partial | minor | M | Picker select plus GroupTarget individual pivots gives flat curl; no per-driven weights, per-member axis or snap-back |
| [`G3-dmc-shape-patches`](#g3-dmc-shape-patches) | Partial | minor | M | RigExecTouchRegions mesh regions only select their control; not drawn as its gizmo, no IK/FK layer switch, cannot pose |
| [`G3-enum-channel`](#g3-enum-channel) | Partial | minor | S | No operator consumes int/token channels; token combo writes default only (no keys); UsdUI valueLabels not used |
| [`G3-face-slider-controls`](#g3-face-slider-controls) | Partial | minor | M | rest:space + avars:unitScaleFactor cover offsets; lacks axis locks, non-uniform offset scale and shape offsets |
| [`G3-gizmo-materials`](#g3-gizmo-materials) | Partial | minor | M | Control guides are constant-colour Hydra prims; no hover highlight, no draw-on-top/X-ray mode, no unlit guarantee |
| [`G3-guide-lines`](#g3-guide-lines) | Partial | minor | S | PV line possible as 2-point BasisCurves driven by points-domain PositionConstraints; channel-driven path ray is not |
| [`G3-ikfk-vis-swap`](#g3-ikfk-vis-swap) | Partial | minor | S | Maps to guide:displayOpacity on float avars:ikfk (FK uses displayOpacityInvert); fades to 0.15 floor, not hidden |
| [`G3-lock-scale`](#g3-lock-scale) | Partial | minor | S | No declarative scale lock; workaround is a RigExecScaleConstraint per control on bool inputs:enabled; gizmo still edits |
| [`G3-module-vis-collapse`](#g3-module-vis-collapse) | Partial | minor | M | MoverAPI inputs:enabled can bypass constraints, but solvers lack enable, no module control set, avar scale floor 1e-4 |
| [`G3-proxy-driven-lists`](#g3-proxy-driven-lists) | Partial | minor | L | GroupTarget + rigExec:picker:controls allow multi-control drags; no proxy prim, driven list, selection vis, key redirect |
| [`G3-selection-sets`](#g3-selection-sets) | Partial | minor | S | RigExecPickerButton (ui:text + rigExec:picker:controls) is a persisted named set; no in-app creation or per-shot home |
| [`G3-shape-library`](#g3-shape-library) | Partial | minor | M | guide:shape has 6 primitives (sphere/circle/box/cube/diamond/pyramid), no library; ~66 of 243 Zebra shapes approximate |
| [`G3-typed-channels`](#g3-typed-channels) | Partial | minor | M | avars: custom attrs lack channel type/limits/keyable metadata; type-exact links; bools only drive inputs:enabled |
| [`G3-vis-logic-composition`](#g3-vis-logic-composition) | Partial | minor | S | AND/OR/NOT built as FloatMathMover float math into helper attrs, one mover per term; no bool ops or missing-default |
| [`G3-foot-rocker-anchor`](#g3-foot-rocker-anchor) | Divergent-by-design | minor | S | No interaction-gated offset feedback writes; parent rocker to pre-roll foot frame via nesting or relay; gizmo-only diff |
| [`G3-interaction-state`](#g3-interaction-state) | Divergent-by-design | minor | S | Evaluation is pure over values; gizmo drags arrive only as SetInteractiveOverrides, never as an IsInteracting flag |
| [`G3-movable-pivot`](#g3-movable-pivot) | Divergent-by-design | minor | M | Spec excludes gesture/persistent-state pivots; substitute is GroupTarget Last Selected pivot about a helper control |
| [`G3-ik-control-setup`](#g3-ik-control-setup) | Implemented | minor | S | Maps to RigExecControl + nested gimbal + rigExec:effectorControl; UE initial values go in rig-layer avar defaults |
| [`G3-rotation-orders`](#g3-rotation-orders) | Implemented | minor | S | Maps to avars:rotationOrder (names match UE, XYZ applies X first), baked too; rest:/default: stay XYZ; mind handedness |
| [`G3-shape-name-resolution`](#g3-shape-name-resolution) | Not-applicable | minor | S | UE construction-time name lookup; a port stores resolved shapes as static data; importer replays precedence/fallback |
| [`G3-debug-draw-primitives`](#g3-debug-draw-primitives) | Missing | cosmetic | M | Only fixed guide families (joint/solver spheres and cones, iso-surfaces); no triads, vectors, arcs or solver internals |
| [`G3-posereader-deformer-debug`](#g3-posereader-deformer-debug) | Missing | cosmetic | S | No cone pose reader or bend/twist/squash movers exist to visualise (G6/G7); only weight overlay and iso-surfaces |
| [`G3-stretch-feedback`](#g3-stretch-feedback) | Missing | cosmetic | S | No guide colour can depend on posed vs rest length; property movers run before the pose walk so nothing computes it |
| [`G3-display-names`](#g3-display-names) | Partial | cosmetic | S | RigExec panels ignore display names for prims and attributes, no builder helper; use UsdUIObjectHints displayName |
| [`G3-footprint-display-proxy`](#g3-footprint-display-proxy) | Partial | cosmetic | S | Nested RigExecControl, guide:shape box, linked colour/opacity; no rounded square or display-only flag; AND needs a mover |
| [`G3-runtime-color-feedback`](#g3-runtime-color-feedback) | Partial | cosmetic | S | guide:displayColor links to a helper revised by RigExecVec3fMathMover; one mover per colour; no event or auto-key hook |
| [`G3-side-colors`](#g3-side-colors) | Implemented | cosmetic | S | Maps to guide:displayColor, optionally linked (one hop) to a RigExecRoot palette color3f; importer does side/grey rule |
| [`G3-dangling-shape-refs`](#g3-dangling-shape-refs) | Not-applicable | cosmetic | S | UE asset-path bug; importer chooses to keep invisible Monster controls or remap /EpicControlRig to /FortniteRigs paths |

### G3-channel-hosts

**Channel hosts: one channel shown and keyed from many controls**

**Verdict:** Missing · **Severity:** major · **Effort:** S · **Confidence:** high · **Domain:** D3

UE features: `UE1-channel-hosts`, `UE4-channel-hosting`, `UE2-body-channels-hosts`, `UE2-prop-visibility-hosting`, `UE2-spine-channels-visibility`, `UE3-ikfk-switch-vis`, `UE3-sec-controls-vis`

**UE rigs.** RigUnit_SetChannelHosts appends host keys to a channel's Customization.AvailableSpaces. The host must be a non-channel control, must not be the channel's parent, and must not be a duplicate. The single channel then appears in the details panel/channel box of every host and is keyed once. MR_Zebra has 55 hosted channels, for example:
- Leg L/Ik Fk Switch is hosted on FK 0/1/2, the FK gimbals, Mid, PV, IK Gimbal and IK Base.
- Spine IK/FK/Sec FK Vis are hosted on 7 spine controls.
- Sec Controls Vis is hosted on the end FK.
- Body Aim Weight/Twist are hosted on Body and Body Orbit.
- Prop Global Vis is hosted on root/Global through the 'Control Vis Channel Host' connector.

**usdRig today.** A channel is an attribute on exactly one prim. The Avar Editor lists only the focus prim's own avars:/foot: attributes and merely counts other selected prims. A picker button can edit one attribute (rigExec:picker:attribute plus attributeLabels). A connection from a 'host' attribute to the owner does not help: connections point from consumer to source, so the evaluator ignores a value typed on the host.

**Gap.** There is no channel-host relationship, and no UI that shows, edits and keys an attribute owned by another prim from the host's channel box or Graph Editor.

**Porting impact.** The IK/FK switch, segment scales, visibility toggles and aim weights can be edited only after selecting the owning control, or through picker buttons. Animators lose the 'switch from any limb control' workflow on every limb, the spine, neck, body and prop.

**Recommendation.** (1) Add an applied RigExecChannelHostAPI to libs/rigExecSchema/schema.usda with 'rel rigExec:hostedChannels'. Its targets are exact attribute paths on other prims, and target order is display order. It is presentation-only and the evaluator never reads it.
(2) Extend plugin/rigExecUsdview/avarEditorModel.py DiscoverChannels to append the hosted channels, labelled owner/name and bound to the owner attribute, so WriteValue/ResetValue key the owner. graphModel.py lists them for the selected host.
(3) Add RigExecControlHandle::AddHostedChannel in libs/rigExecRigging/rigBuilder.h and the matching python/rigexec helper.
USD relationships remap through renames and references, which UE's key list does not.

**Evidence:** `plugin/rigExecUsdview/avarEditorModel.py:361-391`; `plugin/rigExecUsdview/avarEditorModel.py:394-418`; `libs/rigExecSchema/schema.usda:2495-2504`; `plugin/rigExecUsdview/pickerModel.py:130-149`; `libs/rigExec/rigEvaluator.cpp:523-569`

**Verification (holds).** Missing and major hold: nothing in libs, plugin or python contains a host, hosted-channel or channel-box concept.
- The Avar Editor discovers only the focus prim's own attributes (avarEditorModel.py:361-391, 394-413).
- A connection alias on the host does not help: the resolver follows the chain to its end source (moverGraph.h:313-317; rigEvaluator.cpp:567-568).
- Picker attribute buttons (schema.usda:2495-2505) are the only way to edit an owner's channel from elsewhere. They are discrete cycles only and write the default (pickerUI.py:979-982), so they cannot replace float hosted channels such as Segment Scale or Aim Weight, and they do not key.
Confirmed UE side: 55 hosted channels in the MR_Zebra runtime (AvailableSpaces entries).

Verifier evidence: `plugin/rigExecUsdview/avarEditorModel.py:361-413`; `libs/rigExec/moverGraph.h:313-317`; `libs/rigExecSchema/schema.usda:2495-2505`; `plugin/rigExecUsdview/pickerUI.py:968-982`

### G3-control-limits

**Transform limits, channel ranges and limit drawing**

**Verdict:** Missing · **Severity:** major · **Effort:** M · **Confidence:** high · **Domain:** D2

UE features: `UE1-control-limits`, `UE6-control-visual-conventions`, `UE6-corner-2d-slider`, `UE6-lid-main-sliders`

**UE rigs.** Each channel has min/max limit flags (order tx..sz) and values, applied whenever a value is set from the graph, the gizmo or Sequencer.
- Transform locks: IK Base 000111111 (translate only); spine/neck 000000111 (scale locked); face pads lock single axes (Corner tx; Lid Tp tx/ty/pitch/yaw).
- Channel ranges: Softness [0,1]; Segment Scale 1 [0.0001,2] with min only; Mid Blend 0.65 [0,1]; Aim Weight/Twist [0,1]; Control Path Distance min 0; Proxy Pivot Slide [0,1]; face channels -200..200 or -100..200.
- bDrawLimits draws the limit extents; the face sliders turn it off.
Zebra has 46 limited body controls/channels and Monster 17.

**usdRig today.** No limit, clamp or lock attribute exists on controls or avars. The RigExecControlAPI doc promises 'computational limits' but declares only channelRole. A FloatMathMover clamp can bound what consumers see of a float custom channel, but not the value the animator keys, and it cannot target double avars. Avar Editor slider ranges are hard-coded: ±1 m, ±180°, 0..2, 0..1 and 0..10.

**Gap.** Missing: per-avar min/max values with enabled flags, an evaluation-time clamp, UI clamping and data-driven slider ranges, and limit drawing.

**Porting impact.** Several UE constraints on posing are lost:
- IK Base can be rotated and scaled, and spine scale can be keyed.
- The face Corner pads and Lid sliders can leave their planes and feed unbounded values into the corner and blink logic (G6).
- Channel sliders show the wrong ranges.
- Animation keyed outside the UE ranges will not match UE playback, because UE clamps on set.

**Recommendation.** (1) Add fixed, statically registerable limit attributes to RigExecControl in libs/rigExecSchema/schema.usda: 'double3 limits:translateMin/translateMax', 'double3 limits:rotateMin/rotateMax', 'double3 limits:scaleMin/scaleMax' and 'uniform token[] limits:enabled' (for example txMin, txMax).
(2) Register them as AttributeValue inputs of computePointFrame in libs/rigExec/computations.cpp (RIGEXEC_REGISTER_XFORMABLE) and clamp before _ComposeAvars. Mirror this in libs/rigExec/bakedPose.cpp (the RigExecBakedComposeAvars callers).
(3) For custom channels, apply the proposed rigExecRange metadata in the RigExecResolvedInputs resolver in libs/rigExec/rigEvaluator.cpp.
(4) Tools: clamp on write in avarEditorModel.SliderRange/Coerce and plugin/rigExecUsdview/gizmoDrag.py. Add optional limit guides (guide:drawLimits) in libs/rigExecImaging/bridge.cpp.

**Evidence:** `libs/rigExecSchema/schema.usda:33-47`; `docs/spec.md:238`; `docs/spec.md:262`; `libs/rigExec/rigEvaluator.cpp:3568-3587`; `plugin/rigExecUsdview/avarEditorModel.py:274-310`; `docs/biped-rig.md:185-189`

**Verification (holds).** Missing holds: the schema has no clamp or limit attribute for avars (schema.usda grep); the only clamps are FloatMathMover (float only) and weight policies. Major holds: IK Base rotation and scale, spine scale, eye translation and spline overshoot past a clamped range all change the pose.
Corrections:
- The face logic reads only the free axes. Corner logic reads v.ty, v.tz and rotation X; lid logic reads tz and roll (G6 UE6-corner-logic, UE6-blink-logic). Missing locks there cause gizmo drift, not wrong logic values.
- The UE summary misses these face locks in the MR_Zebra runtime: Eye L/R are rotate-only (tx..tz and sx..sz locked), Squeeze has translation locked, Brow Main/In/Ot/Mid lock ty, and the Pupil Size, Iris Size and Convergence channels are ranged. In total 76 elements carry limit flags, body and face together.
- UE separates a displayed range from enforced limits (Rocker Ball Rotation 0..90 is not enforced). Stock OpenUSD 26.08 'limits' metadata with soft and hard sub-dictionaries (attribute.h:829-843; attributeLimits.h:31-60) models exactly that. Consider it instead of new double3 limits:* attributes, at least for custom channels.

Verifier evidence: `libs/rigExecSchema/schema.usda:39-47`; `libs/rigExecSchema/schema.usda:1197-1214`; `plugin/rigExecUsdview/avarEditorModel.py:274-309`; `<usd-install>/include/pxr/usd/usd/attributeLimits.h:31-60`; `<usd-install>/include/pxr/usd/usd/attribute.h:829-843`

### G3-body-layout

**Overall animator body control layout (Zebra)**

**Verdict:** Partial · **Severity:** major · **Effort:** L · **Confidence:** medium · **Domain:** D2 · *reconciled after probes*

UE features: `UE1-animator-body-layout`

**UE rigs.** The Zebra body has 252 controls: 134 EulerTransform, 14 Rotator and 4 Position animation controls, 4 proxies and 96 channels. The face adds 107 controls. Layout:
- root Global > Local and Root.
- Body Orbit > Body, Body Aim and a movable pivot.
- Spine FK, IK and Sec FK; neck; clavicles.
- Arms and legs: FK with gimbals, IK (9 spaces on arms, 4 on legs), PV, Mid, IK Base, IK Rotation.
- Feet: pivots, rocker, footprint.
- 16 twist offsets, fingers, props, tweakers, ears, mohawk.
- Per-limb Vis toggles on root/Global.

**usdRig today.** Each element has a counterpart:
- Hierarchy: namespace nesting of RigExecControls.
- Offset and null groups: plain Xforms or nested controls.
- Gimbals: nested controls.
- Channels: custom avars.
- Guides: one per control.
A control follows a solver-posed joint only through an explicit relay connection: default:space or parent:space connected to a relay's parent:space. Plain nesting under such a joint does not follow, and connected-space providers force the dynamic evaluation path.

**Gap.** The structure itself can be expressed. The animator-facing set depends on the pieces missing in the other G3 rows (channel hosts, limits, hide switches, proxies and pivots, shape library and offsets) and on space switching (G4). Controls parented under bone-following nulls need relay connections: twist offsets, fingers, ears, and face controls under the head.

**Porting impact.** All 359 controls can exist and evaluate, but the UX is degraded: no host fan-out, no hide switches and fewer shapes. Each bone-following control also needs relay wiring, which keeps the rig off the baked fast path.

**Recommendation.** (1) Close the component gaps: ChannelHostAPI, limits, guide:visible, guide:shapeSource/shapeTransform, ProxyControlAPI.
(2) Make 'follow a solver-posed provider' first-class with 'rel parent:provider' on RigExecXformable (libs/rigExecSchema/schema.usda). Resolve it in libs/rigExec/computations.cpp as a relationship-targeted computePointFrame input, and support it in the baked program (libs/rigExec/bakedPose.cpp) as part of the planned P0 connected-space work.

**Evidence:** `docs/biped-rig.md:136-141`; `tests/testRigExecConstraints.cpp:3093-3125`; `docs/plans/evaluation-engine-gaps-vs-premo-libee.md:303`; `libs/rigExec/rigEvaluator.cpp:10569-10600`

**Report reconciliation.** Probe g2review/probe_follow2.py shows controls nested under a solver-posed joint follow its solved frame, so 'plain nesting under such a joint does not follow' is wrong for unclaimed controls whose parent:space is unauthored. Relay parent:space links (which keep a rig off the baked path) are only needed for controls hanging from plain Xform nulls, from joints another solver owns, or with an authored parent:space.

**Verification (holds).** Confirmed:
- Namespace propagation stops at providers that own their pose (rigEvaluator.cpp:10569-10600).
- A relay connection is the tested route to follow a solver-posed joint (testRigExecConstraints.cpp:3091-3125).
- Connected-space providers are the open P0 baked gap (evaluation-engine-gaps-vs-premo-libee.md:303).
Many Zebra face controls are parented directly to BONES (eye_main_l, skull_tp), so the relay cost applies to them. This roll-up is proportionate as major.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:10569-10600`; `tests/testRigExecConstraints.cpp:3091-3125`; `docs/plans/evaluation-engine-gaps-vs-premo-libee.md:303`; `docs/biped-rig.md:136-141`

### G3-bool-vis-switches

**Bool-channel-driven control visibility (per-frame SetControlVisibility)**

**Verdict:** Partial · **Severity:** major · **Effort:** M · **Confidence:** high · **Domain:** D2

UE features: `UE1-visibility-channels`, `UE6-visibility-switches`, `UE8-spine-controls-visibility`, `UE2-spine-channels-visibility`, `UE2-body-channels-hosts`, `UE2-fkarray-visibility-and-spaces`, `UE4-unit-control-display`

**UE rigs.** Bool channels drive RigUnit_SetControlVisibility on every forward solve. About 120 Zebra controls use USER_DEFINED shape visibility. The switches are:
- Body Orbit, Body Aim and Body Offset Vis.
- Spine/Neck IK, FK and Sec FK Vis; the spine Start IK and the last Sec FK are always visible.
- FkArray per-control 'Visibility', with initial values from config.
- The Prop vis bools, Foot Pivot Control Vis and Gimbal Control Vis.
- Face: Micro Vis x2 (14 lid micro controls each), Brow Tweaker Vis x2 and Lip Tweaker Vis (20 lip controls). All face switches default to false, so micro controls start hidden.
A hidden gizmo is neither drawn nor clickable, but its control still evaluates.

**usdRig today.** guide:displayOpacity can be connected to a float or double attribute. The bridge follows the first connection, including property-mover results, and applies guide:displayOpacityInvert and guide:displayOpacityMin (default 0.15; 0 fades the guide out completely). The stock UsdGeomImageable 'visibility' hides a control's guide (it is inherited by hand onto the synthesized guide), but no channel can drive it and it inherits down the namespace. Picker buttons hide only by IK/FK mode.

**Gap.** (1) There is no true show/hide switch, only opacity:
- Bool sources are rejected (_HeldScalar accepts float/double only).
- The schema doc guarantees picking only above opacity 1e-4, and says nothing about a guide at exactly 0.
- Marquee selection ignores opacity and visibility.
(2) USD visibility cannot be driven by a channel, and it is hierarchical: hiding an FK parent control hides its nested children. UE visibility is per control.

**Porting impact.** Micro, tweaker and secondary controls either stay permanently visible, cluttering the face and spine, or must be ported as float dials with displayOpacityMin=0. In the second case they remain marquee-selectable while invisible.

**Recommendation.** (1) Add to RigExecControl in libs/rigExecSchema/schema.usda:
- 'bool guide:visible = true', which can be connected to a bool, float or double channel (non-zero means visible).
- 'rel guide:visibilityDrivers' (all targets must be true).
(2) Resolve both in libs/rigExecImaging/bridge.cpp _ReadGuideStyle, following movedProperties as opacity already does. Publish HdVisibilitySchema=false on the synthesized rigGuideCtrl prim in libs/rigExecImaging/sceneIndices.cpp, per control and not inherited by nested controls.
(3) Honour it in plugin/rigExecUsdview/gizmoMarquee.py Selectable, in TouchPose and in the picker's live filter.
The change is imaging-only: no evaluator state and nothing authored.

**Evidence:** `libs/rigExecSchema/schema.usda:223-249`; `libs/rigExecImaging/bridge.cpp:195-218`; `libs/rigExecImaging/bridge.cpp:222-235`; `libs/rigExecImaging/bridge.cpp:242-292`; `libs/rigExecImaging/sceneIndices.cpp:483-525`; `plugin/rigExecUsdview/gizmoMarquee.py:119-141`; `examples/biped/Biped.usda:4167-4173`

**Verification (holds).** Confirmed:
- Opacity follows only float or double sources (bridge.cpp:223-234, 260-293).
- Picking is guaranteed only above an opacity of 1e-4 (schema.usda:245).
- Marquee selection ignores opacity and visibility (gizmoMarquee.py:119-141).
- Guide visibility is inherited by hand from the flattened parent (sceneIndices.cpp:505-518).
Factual fix to the UE summary: ShapeVisibility (UserDefined vs BasedOnSelection) matters only for PROXY controls (RigHierarchyElements.h:1676-1683; RigUnit_SetControlVisibility.cpp:36-46). In the MR_Zebra runtime only the 4 proxies are USER_DEFINED, and all 239 animation controls are BASED_ON_SELECTION. SetControlVisibility still works on any non-proxy control, so the '~120 USER_DEFINED' count is wrong, but the behaviour and severity stand. There are about 42 face micro and tweaker controls plus the spine, body, prop and gimbal sets.

Verifier evidence: `libs/rigExecImaging/bridge.cpp:223-293`; `libs/rigExecSchema/schema.usda:239-249`; `plugin/rigExecUsdview/gizmoMarquee.py:119-141`; `libs/rigExecImaging/sceneIndices.cpp:505-518`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Rigs/RigHierarchyElements.h:1676-1683`

### G3-shape-transform-offset

**Shape transform (offset, rotation, mirrored scale, chain scale profile)**

**Verdict:** Partial · **Severity:** major · **Effort:** S · **Confidence:** high · **Domain:** D2

UE features: `UE2-shape-profile-and-offset`, `UE6-control-visual-conventions`, `UE6-lid-main-sliders`, `UE6-corner-2d-slider`, `UE4-unit-control-display`

**UE rigs.** Every control has a shape transform (translation, rotation and scale relative to the control).
- 'Get Control Shape' in FkChain, FkArray, Spine and Prop sets shape = Shape.Transform * inverse(Control Transform Offset).
- It then multiplies Scale3D by the 'Control Scale Factor Profile' curve at the chain ratio. Spine IK shapes sample it at 0, 0.5 and 1; the last FK gets Z*5; orient-offset rotations are applied.
- Face controls are drawn offset from the skin. Corner: T(±50,25,0), rotation 90°, scale 5. Lid slider: rotation quat(.5,.5,.5,.5), scale (20,±4,20).

**usdRig today.** guide:scaleX/Y/Z (positive, per axis) is the only shape transform. The guide is placed at the control's orthonormalized posed frame. A non-positive scale draws nothing. There is no offset or rotation: docs/biped-rig.md reports the head control sitting 13.4 cm low for this reason and suggests guide:offsetX/Y/Z.

**Gap.** Missing: shape translation and rotation, and mirrored (negative) shape scale. The per-chain profile is only possible at build time.

**Porting impact.** Face slider and pad gizmos would draw on the skin at their pivots, overlapping each other and the mesh. Spine and FK shapes lose their orientation offsets.

**Recommendation.** (1) Add 'matrix4d guide:shapeTransform' (identity default) to RigExecControl in libs/rigExecSchema/schema.usda. It holds translation, rotation and scale, including negative scale, applied in control-local space before placement.
(2) Apply it in libs/rigExecImaging/bridge.cpp _FillControlGuides and in the compute-extent callback.
(3) The 'non-positive draws nothing' rule stays on guide:scaleX/Y/Z only.
(4) Keep the profile curve a builder concern: a libs/rigExecRigging/rigBuilder.h helper that samples the curve.

**Evidence:** `libs/rigExecSchema/schema.usda:201-215`; `libs/rigExecImaging/bridge.cpp:924-1017`; `docs/biped-rig.md:181-184`

**Verification (holds).** Confirmed:
- Placement is the rigidized control frame, with shape and scale read by plain Get.
- A non-positive scale on any axis draws nothing (bridge.cpp:942-1017).
- There is no offset attribute (biped-rig.md:181-184).
Major holds. Several face controls have identity local offsets at shared pivots (Lid In/Ot at eye_main_l, brows at skull_tp), so without shape offsets their guides coincide inside the head.

Verifier evidence: `libs/rigExecImaging/bridge.cpp:942-1017`; `libs/rigExecSchema/schema.usda:201-215`; `docs/biped-rig.md:181-184`

### G3-switch-keying

**Keying switch and visibility channels from the animator panels**

**Verdict:** Partial · **Severity:** major · **Effort:** S · **Confidence:** high · **Domain:** D3 · *added by verifier*

UE features: `UE3-ikfk-switch-vis`, `UE1-visibility-channels`, `UE1-channel-hosts`, `UE4-channel-hosting`

**UE rigs.** Animators key bool and integer channels in Sequencer and the Anim Details panel from any host control, for example Ik Fk Switch and the Vis toggles (69 bool and 1 int on MR_Zebra). Bool and integer curves are stepped, so a switch changes on its key frame with no in-between blend.

**usdRig today.** The tools handle channel types differently:
- The Avar Editor keys float, double and half as Ts knots and int as held timeSamples.
- It always writes bool and token channels as the default and never keys them.
- The picker switch reads the attribute at the default time and writes a float default. On an animated dial the click changes the label but not the pose, and it never keys.
- A float dial standing in for a bool gets AutoEase knots from the Avar Editor and gizmo, so it blends between 0 and 1 between keys.
- The Graph Editor lists only double, float and half.

**Gap.** No RigExec panel can key a switch channel as a stepped key. The Avar Editor never keys bool or token channels. The picker switch writes only the default, and reads the default rather than the current frame's value. Float dials keyed from the tools get AutoEase knots instead of held ones.

**Porting impact.** An importer can author UE switch keys as held timeSamples or Held knots, but animators then lose control of them:
- A bool switch cannot be re-keyed in usdview.
- The picker IK/FK button does nothing to the pose on an animated limb.
- A float dial keyed from the Avar Editor interpolates between keys, giving a partial IK/FK blend and half-faded controls unless the tangents are changed to Step in the Graph Editor.

**Recommendation.** (1) avarEditorModel.WriteValue: in Animation mode, key bool, token and int channels as timeSamples with Set(value, time) after PromoteAnimationToEditTarget, instead of forcing the default.
(2) pickerUI._Switch: read the attribute at the current frame and write through avarEditorModel.WriteValue in the panel's write mode, inside one rigExecUndo EditScope.
(3) graphModel.AuthorKnot, called from gizmoMath.SetAnimated: author Held interpolation for channels declared as switches, via rigExec:channelRole = switch on the owner or the proposed channel-type metadata. A float dial then keys like a UE bool.

**Evidence:** `plugin/rigExecUsdview/avarEditorModel.py:516-542`; `plugin/rigExecUsdview/pickerUI.py:968-982`; `plugin/rigExecUsdview/gizmoMath.py:1291-1318`; `plugin/rigExecUsdview/graphModel.py:258-288`; `docs/graph-editor.md:20-23`

### G3-display-toggles

**Global control-shape display toggles**

**Verdict:** Missing · **Severity:** minor · **Effort:** S · **Confidence:** medium · **Domain:** D2 · *added by verifier*

UE features: `UE8-gizmo-materials`, `UE1-proxy-controls-driven-lists`

**UE rigs.** Control Rig edit-mode settings offer session-wide toggles: Hide Control Shapes, Show All Proxy Controls, Show Controls As Overlay and Only Select Rig Controls (ControlRigEditor/Private/EditMode/ControlRigEditModeSettings.h:35-41, 71-99). Animators use them to review a pose without gizmos, or to click only controls.

**usdRig today.** Control guides publish purpose default and draw whenever the rig draws. Hiding them all takes per-control authoring: purpose=guide, which usdview's guide toggle then also applies to joint guides, or UsdGeom visibility. The RigExec menu has panels plus viewport-tool and view-cube toggles, but no control-guide display toggle. Picking the mesh selects mesh prims unless TouchPose is enabled.

**Gap.** There is no session-level control-guide show/hide, no show-all-proxies override and no 'controls only' pick filter.

**Porting impact.** Pose review without controls needs stage edits or a different viewer. Clicks on the character select mesh prims instead of controls unless TouchPose is on. No evaluated result changes.

**Recommendation.** (1) Add a session-only RigExec menu toggle that filters the synthesized rigGuideCtrl children in RigExecResultsSceneIndex (libs/rigExecImaging/sceneIndices.cpp), or remaps their render tag, without authoring anything.
(2) Add a 'show all proxies' override once RigExecProxyControlAPI exists.
(3) Add an 'only select controls' option in plugin/rigExecUsdview/gizmoUI.py that drops non-control picks, the same way gizmoMarquee.Selectable already restricts marquee selection to RigExecControl.

**Evidence:** `docs/control-guides.md:250-263`; `libs/rigExecImaging/sceneIndices.cpp:534-552`; `plugin/rigExecUsdview/rigExecUsdview.py:230-242`

### G3-mirror-metadata

**Mirror-behaviour metadata on controls**

**Verdict:** Missing · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D2

UE features: `UE3-control-scale-color-shape`

**UE rigs.** 'Set Mirror Behavior' writes the bool item metadata 'Mirror Behavioral' on controls, for mirroring and pose tools to consume.

**usdRig today.** Mirroring metadata on RigExecControlAPI is explicitly deferred. The biped mirrors by composition (the right side references the left), and there is no pose-mirror tool.

**Gap.** There is no mirror-behaviour flag and no pose mirror or flip command.

**Porting impact.** Animators cannot mirror poses, either behaviourally or by orientation.

**Recommendation.** (1) Add 'uniform token rigExec:mirror:behavior = none|behavioral|orientation' and 'rel rigExec:mirror:counterpart' to RigExecControlAPI in libs/rigExecSchema/schema.usda.
(2) Add a mirror/flip pose command in plugin/rigExecUsdview (a picker command or an Avar Editor action) that reads them.

**Evidence:** `libs/rigExecSchema/schema.usda:37-41`; `docs/spec.md:262`

**Verification (holds).** Confirmed: mirroring metadata is deferred (schema.usda:39-41; spec.md:262), which makes this Missing, not rejected.
The UE summary omits the companion vector metadata 'Mirror Axis' (default (0,1,1)), set by Set Mirror Axis in FkChain and FkArray (CRFL_Control_v001 graphs.txt, 'Set Vector Metadata | Name=Mirror Axis'). 90 MR_Zebra controls carry 'Mirror Behavioral'. The recommendation should add an axis token or vector next to the behaviour flag.

Verifier evidence: `libs/rigExecSchema/schema.usda:33-47`; `docs/spec.md:262`

### G3-multi-edit-channels

**Editing a channel on many selected controls at once**

**Verdict:** Missing · **Severity:** minor · **Effort:** S · **Confidence:** medium · **Domain:** D2 · *added by verifier*

UE features: `UE1-animator-body-layout`, `UE1-channel-hosts`

**UE rigs.** The UE 5.8 Anim Details panel multi-edits. A value or a math expression typed into one property applies to that property on every selected control (FAnimDetailsMultiEditUtil::MultiEditSet and MultiEditMath, ControlRigEditor/Private/AnimDetails/AnimDetailsMultiEditUtil.h:14-50). Examples: flipping Ik Fk Switch on all limbs, or offsetting rx on a finger selection.

**usdRig today.** The Avar Editor edits only the focus prim and reports how many other prims are selected. The viewport GroupTarget moves several controls' transforms together, but custom channels and typed values can be edited on only one prim at a time.

**Gap.** There are no multi-prim channel edits and no relative math entry (+=, *=) in the channel box.

**Porting impact.** Setting a switch or visibility channel on all four limbs, or zeroing a channel across a finger selection, takes one edit per control. This is slower than the UE workflow, but the result is the same.

**Recommendation.** (1) Extend avarEditorModel.DiscoverChannels to the intersection of channel names across usdview's selection.
(2) Apply WriteValue and ResetValue to every selected prim inside one EditScope. rigExecUndo.EditRecorder already accepts many attribute paths (avarEditorModel.py:583-606).
(3) Accept relative expressions in the spin box (plugin/rigExecUsdview/avarEditorUI.py).
(4) Once G3-channel-hosts lands, dedupe hosted channels that resolve to the same owner attribute.

**Evidence:** `plugin/rigExecUsdview/avarEditorModel.py:394-413`; `plugin/rigExecUsdview/gizmoUI.py:1819-1833`

### G3-control-scale

**Global / module / auto control-shape scaling**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D2

UE features: `UE4-set-control-scale`, `UE2-set-control-scale`, `UE3-control-scale-color-shape`

**UE rigs.** Set Control Scale runs at construction with k = Root 'Global Control Scale' * module 'Control Scale' * an auto factor. The auto factor is chain length divided by a reference length: IkFk2Bones 55, Foot 17, LimbTwist 20, Spine 30, Root 96 (only when the length is over 0.5); it is 1 for Body, FkArray, FkChain and Prop. k multiplies both the shape scale and the shape translation; rotation is kept. The step is skipped when a CRSL library was found.

**usdRig today.** guide:scaleX/Y/Z are authored per control and multiplied by the evaluated frame-axis lengths. The bridge reads them with a plain Get, so connections are not followed. There is no rig-wide or module multiplier, and the builder has no guide setters.

**Gap.** There is no global or module control-scale knob, so k must be baked into each control by an importer or builder.

**Porting impact.** The importer bakes k into the guide scales. Resizing the rig's controls later means re-authoring every control.

**Recommendation.** (1) Add 'double guide:scale = 1' on RigExecRoot for rig-wide scaling, and an applied RigExecGuideScaleAPI for module scopes. Scales multiply down the namespace and apply to both the guide scale and the guide:shapeTransform translation, resolved in libs/rigExecImaging/bridge.cpp _FillControlGuides.
(2) Add RigExecControlHandle::SetGuide(shape, scale, color, shapeTransform) to libs/rigExecRigging/rigBuilder.h.

**Evidence:** `libs/rigExecSchema/schema.usda:201-215`; `libs/rigExecImaging/bridge.cpp:955-1003`; `libs/rigExecRigging/rigBuilder.h:119-138`

**Verification (holds).** Confirmed: guide:scaleX/Y/Z are read with a.Get(time) and no connection following (bridge.cpp:965-972). The builder has no guide setters, and SetAttr could author the declared guide attributes generically (rigBuilder.h:78-82, 119-138). Minor holds.

Verifier evidence: `libs/rigExecImaging/bridge.cpp:965-972`; `libs/rigExecRigging/rigBuilder.h:78-82`; `libs/rigExecRigging/rigBuilder.h:119-138`

### G3-control-value-types

**Control value types and channel filtering (EulerTransform / Rotator / Position)**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D2

UE features: `UE1-animator-body-layout`, `UE3-ik-controls-setup`

**UE rigs.** The control type decides which channels exist. MR_Zebra, face included, has 225 EULER_TRANSFORM, 14 ROTATOR (the gimbals, e.g. IK Gimbal) and 4 POSITION controls, and filtered_channels can narrow them further. The details panel, Sequencer tracks and viewport gizmo offer only the channels the type has.

**usdRig today.** Every RigExecControl carries the same double avars (tx/ty/tz, rx/ry/rz, rspin, sx/sy/sz) plus rotationOrder. A 'rotator' is simply a control whose translate and scale avars are left unauthored. RigExecControlAPI.rigExec:channelRole (pose, switch or tweak) exists, but only the builder writes it and nothing reads it.

**Gap.** There is no per-control channel mask or keyable flag. The Avar Editor, the gizmo's Move/Scale tools and the Graph Editor offer every channel on rotator and position controls. Keyability and presentation metadata are explicitly deferred (schema.usda:39-41; spec.md:262).

**Porting impact.** Animators can translate gimbal controls or rotate position controls. Those keys are real avars and change the pose, where UE would not offer the channel at all.

**Recommendation.** Add 'uniform token[] rigExec:channels' to RigExecControlAPI (libs/rigExecSchema/schema.usda). It lists a subset of tx..sz and rspin; empty means all, and it is the equivalent of UE's value type plus filtered channels.
- The evaluator treats unlisted avars as their fallback: a constant mask input to computePointFrame in libs/rigExec/computations.cpp, mirrored in libs/rigExec/bakedPose.cpp.
- plugin/rigExecUsdview/avarEditorModel.py, gizmoDrag.py/gizmoMath.py and graphModel.py hide the unlisted channels.
- Builder: RigExecControlHandle::SetChannels.

**Evidence:** `libs/rigExecSchema/schema.usda:158-166`; `libs/rigExecSchema/schema.usda:327-344`; `libs/rigExecSchema/schema.usda:33-47`; `libs/rigExecRigging/rigBuilder.cpp:384`; `docs/viewport-gizmos.md:44-50`

**Verification (holds).** Confirmed: every control carries the full avar set (schema.usda:158-166, 327-344), and channelRole is written only by builders and read by nothing (grep: rigBuilder.cpp:384, python/rigexec/__init__.py:381,483, tests only).
UE counts check out in the MR_Zebra runtime: 221 EULER_TRANSFORM controls plus 4 proxies, 14 ROTATOR and 4 POSITION. However, filtered_channels is empty on all 359 elements, so only the control type narrows the channel set.
Keyability and presentation metadata are deferred (schema.usda:39-41; spec.md:262), so this is Partial, not Divergent. Minor holds.

Verifier evidence: `libs/rigExecSchema/schema.usda:39-47`; `libs/rigExecSchema/schema.usda:158-166`; `libs/rigExecRigging/rigBuilder.cpp:384`; `docs/spec.md:262`

### G3-delta-proxy-profile

**Delta proxy control (ProxyControl module) with per-driven weight profile**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D2 · *(analyst said Missing / minor)*

UE features: `UE3-proxy-control-module`

**UE rigs.** CRM_FN_ProxyControl builds '<Proxy Name> Null' at Snap To and a proxy control under it (bIsProxy, DrivenControls, UserDefined visibility), plus a 'Pivot Vis' bool that nothing reads.
- While the proxy is being manipulated: D = L * Prev^-1. Each driven control k, at chain ratio r_k, gets OffsetTransformForItem(lerp(identity, D, profile(r_k))). Then Prev = L.
- Otherwise the proxy re-snaps: offset = ProxyOffset * Snap current, local = identity.
The 'Driven Control Profile' curve runs 1->0 for Inner and 0->1 for Outer. The biped uses it for Curl (12 finger FK controls) and Spread (4 FK 0 or metacarpal controls); Zebra has no instance.

**usdRig today.** The evaluator has no stateful delta operator: hidden previous-frame state is rejected by the spec. There is also no profile-curve evaluator. A rig-level alternative is a custom 'curl' dial that drives weighted RotationConstraints or nested offset controls. That result is persistent and keyed on the dial, rather than baked into the finger keys as in UE.

**Gap.** Missing: the tool-side distribution of the proxy delta over the driven controls with a weight profile, and the snap-back on release.

**Porting impact.** None for Zebra and Monster, which have no instances. The biped-template finger curl/spread workflow is lost.

**Recommendation.** Implement it as mode 'delta' of the proposed RigExecProxyControlAPI:
- drivenWeights replace the curve profile; the importer samples the UE FRuntimeFloatCurve at each r_k.
- During the drag, plugin/rigExecUsdview/gizmoMath.py applies lerp(identity, D, w_k) to each driven control's avars, previewed through SetInteractiveOverrides.
- The proxy re-snaps to snapTo on release.
No evaluator state is added.

**Evidence:** `docs/spec.md:1050`; `docs/spec.md:56-63`; `libs/rigExecSchema/schema.usda:999-1020`; `libs/rigExecSchema/schema.usda:1087-1100`

**Verification (corrected).** This is not wholly Missing. A picker button selecting the 12 finger FK controls, plus the GroupTarget in 'individual' pivot mode, rotates every member about its own origin by the same delta (gizmoMath.py:1284-1288, 2708-2766). That is the Curl with a flat profile.
Still missing:
- per-driven weights (UE's Driven Control Profile of 1 to 0 or 0 to 1);
- a per-member local rotation axis (GroupTarget uses the lead's frame for 'Local', gizmoMath.py:2720-2728);
- the proxy prim and its snap-back.
Zebra and Monster have no instance, so minor stands.

Verifier evidence: `plugin/rigExecUsdview/gizmoMath.py:1284-1288`; `plugin/rigExecUsdview/gizmoMath.py:2708-2766`; `docs/spec.md:1050`

### G3-dmc-shape-patches

**DMC per-bone mesh-patch gizmos (fk-layer / ik-layer shapes)**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D2

UE features: `UE2-dmc-shape-library-switch`, `UE3-control-scale-color-shape`, `UE3-ik-debug-draw`

**UE rigs.** When the Root metadata 'Direct Mesh Control' is on, modules use the namespace 'fk-layer' or 'ik-layer' and look up '<layer>.<bone>' shapes. SetupShapeLibraryFromLayer builds those shapes from the skeletal mesh's triangle label layers, so control gizmos become patches of the character mesh (DMC templates MR_ZebraDMC and MR_FN_BipedDMC). IkFk2Bones suppresses its PV line when DMC is found. In the headless dump, no control actually resolved to a layer shape.

**usdRig today.** TouchPose is the closest equivalent. RigExecTouchRegions (rel mesh, palette, alpha) holds RigExecTouchRegion children (face indices, rel control). A click ray-casts the deformed mesh and selects the bound control, and hovered or selected regions are highlighted on an overlay mesh. It only selects.

**Gap.** A region selects its control but is not that control's drawn gizmo. There is no per-control mesh-patch shape and no IK/FK layer switch, and dragging on the mesh does not pose it (DMC deformation itself is covered by G7).

**Porting impact.** Mesh-patch picking can be reproduced with TouchPose regions generated from the UE triangle label layers. The 'gizmo is the skin patch' look cannot.

**Recommendation.** (1) Generate one RigExecTouchRegions scope per UE label layer (fk-layer, ik-layer), with one RigExecTouchRegion per bone.
(2) Let the proposed guide:visible and IK/FK mode drivers choose which scope is active.
(3) Optionally let guide:shapeSource target a RigExecTouchRegion, so the region overlay (libs/rigExecImaging/touchOverlayAdapter.cpp) is drawn as the control's gizmo.

**Evidence:** `libs/rigExecSchema/schema.usda:2530-2569`; `libs/rigExecSchema/schema.usda:2610-2635`

**Verification (holds).** Confirmed: TouchPose regions (a face index list plus a control relationship) select only. They never draw as the control's gizmo and never pose (schema.usda:2530-2569, 2610-2635).
No Zebra or Monster control resolved to a layer shape in the dump, so minor holds.

Verifier evidence: `libs/rigExecSchema/schema.usda:2517-2635`

### G3-enum-channel

**Enum-typed integer channel (Bake Root On)**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D3

UE features: `UE2-root-matching-enum-channel`

**UE rigs.** CRM_FN_Root spawns the integer channel 'Bake Root On': initial 0, limited to [0,100], with ControlEnum set to the UserDefinedEnum CREnum_RootMatching. The UI shows it as a dropdown: Global Control, Local Control, Root Control. The channel is grouped with root/Global and its key is stored in the 'Root Matching Channel' variable.

**usdRig today.** A custom 'token avars:bakeRootOn' with authored allowedTokens appears as a combo box in the Avar Editor (VALUE_TOKEN plus allowedTokens) and animates as held timeSamples. An int avar gets a spin box with a hard-coded 0..10 slider. A picker attribute button can cycle labelled values. Nothing in the evaluator branches on a token or int input.

**Gap.** There is no int-plus-label-table channel type and no operator that consumes a discrete channel, for example to select one of N sources. Tokens cannot hold Ts splines, so the Graph Editor cannot show the channel.

**Porting impact.** The dropdown can be reproduced. Its downstream use, choosing the root-motion bake target, has no in-rig consumer, so the export/bake tool would have to read the channel instead.

**Recommendation.** (1) Represent enums as int channels carrying a registered 'rigExecEnumLabels' (token[]) SdfMetadata field (plugin/rigExecSchema/resources/plugInfo.json). avarEditorModel.Channel and pickerModel read it for combo labels and cycle labels.
(2) If a rig-side consumer is needed, add RigExecChoiceMover (an int input selects one of N float or matrix sources) to libs/rigExecSchema/schema.usda, evaluated in the property-chain phase of libs/rigExec/rigEvaluator.cpp.
(3) Let python/rigexec export_baked read the channel for root bakes.

**Evidence:** `plugin/rigExecUsdview/avarEditorModel.py:230-251`; `plugin/rigExecUsdview/avarEditorUI.py:93-97`; `plugin/rigExecUsdview/avarEditorModel.py:274-310`; `plugin/rigExecUsdview/pickerModel.py:130-149`; `docs/graph-editor.md:21`

**Verification (holds).** Partial and minor hold. Corrections:
- A token channel does not 'animate as held timeSamples' from the Avar Editor, which writes tokens as the default only (avarEditorModel.py:528-538). The int spin box and 0..10 slider are confirmed (avarEditorModel.py:295-296).
- Stock OpenUSD 26.08 already provides int enum labels: UsdUIAttributeHints valueLabels and valueLabelsOrder (attributeHints.h:56-71). Recommend that over a new 'rigExecEnumLabels' field. Nothing in the repo reads uiHints (grep finds nothing).
- The picker already has labelled cycling (rigExec:picker:attributeLabels, schema.usda:2500-2505; pickerModel.py:130-149). It writes a float default, though (pickerUI.py:979-982).

Verifier evidence: `plugin/rigExecUsdview/avarEditorModel.py:295-296`; `plugin/rigExecUsdview/avarEditorModel.py:528-538`; `libs/rigExecSchema/schema.usda:2500-2505`; `plugin/rigExecUsdview/pickerModel.py:130-149`; `<usd-install>/include/pxr/usd/usdUI/attributeHints.h:56-71`

### G3-face-slider-controls

**Face 2-D corner pads and lid sliders (locked axes, scaled/rotated offsets)**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D2 · *(analyst said Partial / major)*

UE features: `UE6-corner-2d-slider`, `UE6-lid-main-sliders`, `UE6-control-visual-conventions`

**UE rigs.** Corner L/R:
- The control sits under 'Corner L Null', a follower null constrained to skull/jaw/muzzle/mouth.
- Its offset keeps the bone translation, adds rotation Z ±60° and scale (0.075,-0.075,0.075), in Global offset space.
- TranslationX is locked (min=max=0), so it is a 2-D pad in the offset YZ plane.
- Shape: Triangle_Thick with a shape offset, bDrawLimits=false.
- It hosts Sneer Tp/Bt (-200..200); Monster adds Sticky (0..200).
Lid Tp L:
- The control sits under eye_main_l with a Local offset: rotation 180° about X, T(13,0,7), scale 0.05.
- tx, ty, pitch and yaw are locked; tz (open/close) and roll (tilt) are free.
Lid Bt has no limits, and the R side uses mirrored offsets.

**usdRig today.** The control's orthonormal rest:space carries the offset position and rotation. The uniform offset scale maps to avars:unitScaleFactor (translation avars × 0.075 or 0.05). Because tx is locked at 0, the corner's reflected Y can be replaced by an extra 180° rotation about Z. The follower null is a constrained parent control or Xform: constraints may write controls and native Xforms, and descendants follow the revision.

**Gap.** Several pieces are missing:
- Axis locks (see G3-control-limits).
- Offset scale in the rest: rest is orthonormalized, and unitScaleFactor is uniform and applies to translation only.
- Shape offsets (see G3-shape-transform-offset).
- The gizmo draws and edits a constrained control at its unconstrained frame. Whether a pad nested under a constrained null is edited at its propagated frame has not been verified.

**Porting impact.** Pads and lid sliders can be dragged off their planes, feeding wrong values into the corner and blink logic (G6). Gizmos sit on the skin, and a pad may be manipulated away from where the jaw has moved it.

**Recommendation.** Most of this is covered by the limits and guide:shapeTransform proposals. In addition, make plugin/rigExecUsdview/gizmoMath.py place and edit controls at their published revised frame (pose.controlFrames already carries it), and add a test for a pad nested under a constrained follower null.

**Evidence:** `libs/rigExecSchema/schema.usda:298-306`; `libs/rigExecSchema/schema.usda:342-344`; `libs/rigExec/rigEvaluator.cpp:10569-10600`; `docs/viewport-gizmos.md:371-373`

**Verification (corrected).** Partial holds, but major overstates the impact:
1. The face logic reads only the free channels: Corner Logic reads v.ty, v.tz and rotation X, and the lid functions read tz and roll (G6 UE6-corner-logic, UE6-blink-logic, UE6-blink-extend-open-rotate). Unlocked axes therefore move the gizmo but never feed wrong values.
2. The 'not verified' point is answered. Followers ported as RigExecParentConstraint targets are in the gizmo's overwriting set (gizmoMath.py:429-470). Their published frame is used, and nested controls compose off it (gizmoMath.py:1014-1071, 1146-1162), as measured on the biped's 32 nested finger controls. Only followers driven by Position/Rotation/Aim constraints keep the unconstrained-frame limitation (viewport-gizmos.md:371-373).
The remaining visible losses (locks, shape offsets) are already scored in G3-control-limits and G3-shape-transform-offset.

Verifier evidence: `plugin/rigExecUsdview/gizmoMath.py:429-470`; `plugin/rigExecUsdview/gizmoMath.py:1014-1071`; `plugin/rigExecUsdview/gizmoMath.py:1138-1162`; `docs/viewport-gizmos.md:371-373`; `libs/rigExecSchema/schema.usda:342-344`

### G3-gizmo-materials

**Gizmo materials: colour parameter, hover highlight, X-ray overlay**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D2

UE features: `UE8-gizmo-materials`

**UE rigs.** Each gizmo actor uses a dynamic material instance of the library's DefaultMaterial, with a 'Color' parameter set from ShapeColor. ModularRigGizmoMaterial is unlit, opaque and exposure-compensated. The DMC library's translucent material adds 'Hovered' and 'HoveredColor', so gizmos highlight on hover. An X-ray material with no depth test is used when the editor's 'Show Controls As Overlay' option is on.

**usdRig today.** Control guides are synthesized Hydra prims with constant displayColor and displayOpacity primvars under the default render purpose, and usdview draws its usual selection highlight. Hover highlighting exists only for gizmo manipulator handles and TouchPose regions. Control guides have no draw-on-top or X-ray option.

**Gap.** Missing: hover highlighting on control guides, a draw-on-top/X-ray mode, and an exposure-independent (unlit) guarantee across renderers.

**Porting impact.** Face and body controls inside or behind the mesh are hidden unless offset, and animators get less feedback before selecting.

**Recommendation.** (1) Add 'uniform bool guide:drawOnTop' to RigExecControl. libs/rigExecImaging/sceneIndices.cpp maps it to a render tag or material with depth testing disabled in Storm, or to usdview's overlay pass.
(2) Add hover highlighting in plugin/rigExecUsdview by reusing gizmoUI.py's WA_Hover path to tint the hovered control guide through a session-only override.

**Evidence:** `libs/rigExecSchema/schema.usda:216-249`; `plugin/rigExecUsdview/gizmoUI.py:278-288`; `docs/control-guides.md:250-263`

**Verification (holds).** Confirmed: hover states exist only for manipulator handles (gizmoUI.py:278-287) and TouchPose regions. Control guides have no hover tint and no draw-on-top option. Their render tag comes from purpose only (sceneIndices.cpp:534-552).

Verifier evidence: `plugin/rigExecUsdview/gizmoUI.py:270-287`; `libs/rigExecImaging/sceneIndices.cpp:534-552`; `docs/control-guides.md:250-263`

### G3-guide-lines

**Animator guide lines: pole-vector line and control-path ray**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D19 · *(analyst said Missing / minor)*

UE features: `UE2-root-control-path-debug`, `UE3-ik-debug-draw`

**UE rigs.** Root 'Generate Control Path' runs on Global and Local. It uses a bool 'Control Path Vis' (false) and a float 'Control Path Distance' d (50, min 0, max 500 without a limit), and draws a persistent foreground line:
- from the control origin to origin + d along local +Y
- thickness min(d, 1)
- coloured with the control's shape colour
IkFk2Bones always draws a grey (0.2) line from Mid to PV, unless DMC is active.

**usdRig today.** Guides exist only per provider: control shapes, joint sphere and cone (cones only toward child joints), solver element spheres and cones, and volume iso-surfaces. A CurveMover in emitGuidePoints mode can write ribbon frame origins into a points prim, but only origins.

**Gap.** There is no channel-controlled guide line, such as a line between two providers or along an axis.

**Porting impact.** Animators lose the pole-vector line and the root control-path ray. Posing still works.

**Recommendation.** (1) Add a typed prim RigExecGuideLine to libs/rigExecSchema/schema.usda (purpose guide by default) with:
- endpoints: 'rel rigExec:guide:from' and 'rel rigExec:guide:to' (providers), or 'double3 rigExec:guide:direction' plus a connectable 'double rigExec:guide:length'.
- style: 'double guide:wireWidth', a connectable 'color3f guide:displayColor' and a connectable 'bool guide:visible'.
(2) libs/rigExecImaging/bridge.cpp places it from pose.controlFrames and jointFramesFinal, and libs/rigExecImaging/sceneIndices.cpp synthesizes it as basisCurves.
The evaluator does not change.

**Evidence:** `libs/rigExecSchema/schema.usda:382-404`; `libs/rigExecSchema/schema.usda:507-521`; `libs/rigExecImaging/sceneIndices.cpp:345-352`

**Verification (corrected).** The always-on grey PV line is expressible today. Use a 2-point native BasisCurves whose .points are revised by two points-domain PositionConstraints, sourced from Mid and from PV. Each is enveloped by a dense static weight: [1,0] for the Mid constraint and [0,1] for the PV constraint. Supporting evidence:
- Points-domain targets are legal (rigEvaluator.cpp:3370-3395).
- Per-point envelopes use weight objects (examples/14_VolumeConstrainedSweep.usda:1-33).
- Offsets must be authored (no maintain-offset).
The Control Path ray is not expressible: its length comes from a float channel, and its visibility and colour are channel-driven. So Partial, minor.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:3370-3395`; `examples/14_VolumeConstrainedSweep.usda:1-33`; `libs/rigExecSchema/schema.usda:382-404`; `libs/rigExecImaging/sceneIndices.cpp:345-352`

### G3-ikfk-vis-swap

**IK/FK mode visibility swap**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D2

UE features: `UE4-switch-control-visibility`, `UE3-ikfk-switch-vis`

**UE rigs.** Switch Control Visibility(On, Off, Switch) makes the On controls visible when Switch is true and the Off controls visible when it is false; IkFk2Bones calls it once at construction. The forward solve, driven by the bool 'Ik Fk Switch' (legs default to IK, arms to FK), sets:
- FK controls visible = !IK.
- IK, PV and IK Rotation visible = IK.
- IK Base visible = IK && Sec Controls Vis.
Pre Forwards Solve also publishes the switch as module metadata 'IK Solve' for the Foot module.

**usdRig today.** This is designed in:
- IK controls connect guide:displayOpacity to <limb>_params.avars:ikfk.
- FK controls connect to the same dial with guide:displayOpacityInvert=1.
- The picker hides the inactive half of IK/FK button pairs by reading the dial (rigExec:picker:mode and modeDial).
- The same dial drives BlendPointFrames.inputs:weight.

**Gap.** It fades instead of hiding (floor 0.15 by default) and uses a float dial instead of a bool switch, because the opacity path rejects bools. The IK Base AND term needs a FloatMathMover multiply into a helper float. Cross-module 'IK Solve' metadata becomes direct connections.

**Porting impact.** The behaviour is reproducible with float dials and displayOpacityMin=0. The switch becomes a slider, so fractional values are possible where UE's switch is binary.

**Recommendation.** Use the proposed connectable, bool-aware guide:visible for hard swaps and keep displayOpacity for fades. Accept bool sources for BlendPointFrames.inputs:weight through the coercion proposed in G3-typed-channels, so the UE bool switch ports one-to-one.

**Evidence:** `libs/rigExecSchema/schema.usda:223-249`; `examples/biped/Biped.usda:4167-4173`; `plugin/rigExecUsdview/pickerModel.py:186-228`; `libs/rigExecSchema/schema.usda:2505-2515`

**Verification (holds).** Confirmed:
- IK controls connect their opacity to the ikfk dial, and FK controls connect with displayOpacityInvert (Biped.usda:4171-4172).
- The picker filters buttons by mode through the dial (pickerModel.py:224-226).
- The minimum-opacity floor applies only to connected opacity (bridge.cpp:277-289).
The float-dial workaround reproduces the UE result. Minor is proportionate. See G3-switch-keying for the keying limitation of that dial.

Verifier evidence: `examples/biped/Biped.usda:4167-4173`; `plugin/rigExecUsdview/pickerModel.py:186-228`; `libs/rigExecImaging/bridge.cpp:260-293`

### G3-lock-scale

**Lock Scale module option (per-frame local scale reset)**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** medium · **Domain:** D2

UE features: `UE2-lock-scale`

**UE rigs.** The public 'Lock Scale' bool (default false) exists on Root, Body and FkChain. When it is on, Forwards Solve sets local scale to (1,1,1), with propagation, every frame on these controls: Global and Local; Body Orbit, Body, Body Proxy and Body Offset; and all FkChain controls. This overrides any animated scale.

**usdRig today.** There is no lock metadata. Workaround: add a RigExecScaleConstraint per control, sourced from the control's parent, with affectScale enabled. Its MoverAPI inputs:enabled is connected to a bool 'lockScale' channel; bool connections are accepted on inputs:enabled.

**Gap.** There is no declarative scale lock. The workaround costs one mover per control, and the gizmo keeps editing and drawing the unconstrained avars.

**Porting impact.** This matters only when animators key scale on those controls with Lock Scale enabled, and the option defaults to false. The workaround reproduces the pose.

**Recommendation.** Express the lock through the proposed limits on RigExecControl (min = max = 1 on sx/sy/sz) or the rigExec:channels mask, not through a new operator. Add a builder helper SetLockScale(bool) in libs/rigExecRigging/rigBuilder.h.

**Evidence:** `libs/rigExecSchema/schema.usda:1100-1113`; `libs/rigExecSchema/schema.usda:49-93`; `libs/rigExec/rigEvaluator.cpp:3628-3645`; `docs/viewport-gizmos.md:371-373`

**Verification (holds).** The workaround is valid:
- ScaleConstraint exists with per-axis masks and an additive offset (schema.usda:1087-1100, 1101-1113).
- A constraint's inputs:enabled is resolved per frame through RigExecResolvedInputs, which follows connections (rigEvaluator.cpp:10989-10996).
- A bool connection is accepted on inputs:enabled (rigEvaluator.cpp:3632-3643).
ScaleConstraint targets are not in the gizmo's overwriting set (gizmoMath.py:429-470), so the gizmo still edits the scale avars under the lock, as the row says.
The option defaults to false, so minor is proportionate.

Verifier evidence: `libs/rigExecSchema/schema.usda:1087-1113`; `libs/rigExec/rigEvaluator.cpp:10989-10996`; `libs/rigExec/rigEvaluator.cpp:3632-3643`; `plugin/rigExecUsdview/gizmoMath.py:429-470`

### G3-module-vis-collapse

**Per-module Vis channel that hides controls and collapses the limb**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D2

UE features: `UE3-module-vis-hide`

**UE rigs.** IkFk2Bones spawns '<Module> Vis' (e.g. 'Leg L Vis', initial true) under root/Global, falling back to the IK control. When it is false, the forward solve hides every control in the module (GetItemsInModule), sets each bone's local scale to (0,0,0), and skips the rest of the module's solve.

**usdRig today.** Each part has a partial equivalent:
- Hiding: opacity connections from a float dial today, or the proposed guide:visible drivers.
- Skipping: the MoverAPI inputs:enabled of the limb's constraints and movers can be connected to a bool channel, giving a shape-preserving pass-through.
- Collapse: a ScaleConstraint or scale offset whose envelope comes from the dial.
Solver prims have no enable input, and there is no module grouping to address 'all controls in the module'.

**Gap.** There is no solver enable and no module-scoped control set, and avar scale is floored at 1e-4 instead of 0.

**Porting impact.** The limb hide-and-collapse convenience, which defaults to on (visible), can only be approximated, and the solve cost is not skipped.

**Recommendation.** (1) Add 'bool inputs:enabled' to the solver schemas (RigExecFkChain, TwoBoneIk, SplineIk, ...) in libs/rigExecSchema/schema.usda. A disabled solver passes the rest pose through; implement this in the pose-DAG solver batches (libs/rigExec/rigEvaluator.cpp) and in libs/rigExec/bakedPose.cpp.
(2) Express module membership with a UsdCollectionAPI instance on the module scope, so tools and visibility drivers can address the module's controls.

**Evidence:** `libs/rigExecSchema/schema.usda:49-93`; `libs/rigExec/rigEvaluator.cpp:3628-3645`; `libs/rigExecSchema/schema.usda:1100-1113`; `libs/rigExecSchema/schema.usda:158-166`

**Verification (holds).** Confirmed: inputs:enabled exists only on MoverAPI and on the Pose and PoseInterpolator schemas (schema.usda:69, 1921, 2059). No solver schema has one.
Constraints honour enabled per frame (rigEvaluator.cpp:10989-10996). The avar scale floor of 1e-4 is documented at schema.usda:158-166.
The UE channel defaults to true, so minor is proportionate.

Verifier evidence: `libs/rigExecSchema/schema.usda:69-72`; `libs/rigExecSchema/schema.usda:158-166`; `libs/rigExec/rigEvaluator.cpp:10989-10996`

### G3-proxy-driven-lists

**Proxy controls with driven-control lists and selection-based visibility**

**Verdict:** Partial · **Severity:** minor · **Effort:** L · **Confidence:** high · **Domain:** D2 · *(analyst said Missing / minor)*

UE features: `UE1-proxy-controls-driven-lists`

**UE rigs.** PROXY_CONTROL transform controls carry DrivenControls (RigUnit_SetControlDrivenList). The proxy itself is not keyed. When it is manipulated, the editor re-sets and keys the driven controls' resulting local transforms (FControlRigEditMode::NotifyDrivenControls). With ShapeVisibility=BasedOnSelection, a proxy is shown only while it or one of its driven controls is selected; UserDefined proxies are controlled by the graph.
- Zebra: the Body and Spine movable pivots (driving Body Orbit and End IK) and 2 Footprint Display proxies.
- Biped: 16 proxies, including finger Curl/Spread driving 4 to 12 FK controls.

**usdRig today.** Every control is a keyable transform provider, and there is no proxy or driven relationship. A picker button can select several controls (rigExec:picker:controls), but the gizmo edits exactly one prim. Nothing redirects keys to other controls or ties visibility to the selection.

**Gap.** Missing: a proxy flag, the driven list, selection-dependent visibility and key redirection.

**Porting impact.** Zebra and Monster lose the selection coupling of their pivot proxies (see G3-movable-pivot). The biped finger curl/spread proxies have no equivalent.

**Recommendation.** (1) Add an applied RigExecProxyControlAPI to libs/rigExecSchema/schema.usda:
- rel rigExec:proxy:drivenControls
- float[] rigExec:proxy:drivenWeights
- uniform token rigExec:proxy:visibility = userDefined|basedOnSelection
- uniform token rigExec:proxy:mode = delta|movablePivot|display
- rel rigExec:proxy:snapTo
(2) The evaluator never folds a proxy's avars into the pose.
(3) plugin/rigExecUsdview/gizmoDrag.py and gizmoPreview.py drive the driven controls through SetInteractiveOverrides during the drag and author only their avars on release, as one rigExecUndo.py step.
(4) libs/rigExecImaging/bridge.cpp hides basedOnSelection proxies unless the usdview selection contains the proxy or one of its driven controls.

**Evidence:** `libs/rigExecSchema/schema.usda:2490-2494`; `docs/viewport-gizmos.md:44-50`; `docs/spec.md:207`; `plugin/rigExecUsdview/gizmoMarquee.py:119-141`

**Verification (corrected).** The mapping's claim that 'the gizmo edits exactly one prim' is out of date. docs/viewport-gizmos.md:49-50 is stale.
- gizmoUI.RefreshTarget builds a gizmoMath.GroupTarget whenever several prims are selected (gizmoUI.py:1819-1829).
- GroupTarget pivots on the centre, the last-selected control or individual origins, and orbits members about the pivot with per-member channel inversion (gizmoMath.py:1284-1288, 2708-2766, 3176-3236).
- A picker button's rigExec:picker:controls can select a driven list in one click (schema.usda:2490-2494).
So 'manipulate several driven controls through one gizmo' exists. Still missing: a proxy prim with its own placement, the driven-list relationship, selection-based proxy visibility, and key redirection from the proxy to the driven controls. Minor stands.

Verifier evidence: `plugin/rigExecUsdview/gizmoUI.py:1793-1833`; `plugin/rigExecUsdview/gizmoMath.py:1284-1288`; `plugin/rigExecUsdview/gizmoMath.py:2708-2766`; `plugin/rigExecUsdview/gizmoMath.py:3176-3236`; `libs/rigExecSchema/schema.usda:2490-2494`

### G3-selection-sets

**Animator selection sets saved with sequences**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D2 · *(analyst said Missing / minor)*

UE features: `UE8-seq-selection-sets`

**UE rigs.** Animator Kit selection sets are stored as UAIESelectionSets asset user data on LevelSequences: about 40 named sets in zebra_audition and 12+ in zebra_marketingPoseFaces, covering Control Rig controls and actors. Face members use the stale prefix 'Zebra_Face_CtrlRig/', which no longer resolves because the module is now named 'Face'.

**usdRig today.** There is no selection-set schema or panel. The closest persistent multi-control selection is a picker button's rigExec:picker:controls relationship, which remaps on rename.

**Gap.** Named selection sets cannot be stored with a shot or animation layer, and there is no UI to create or apply them.

**Porting impact.** Sets must be converted to picker buttons or dropped. The stale face entries need a 'Zebra_Face_CtrlRig/' to 'Face/' remap.

**Recommendation.** (1) Store sets as UsdCollectionAPI instances (includes = control prims) on a 'SelectionSets' prim in the shot's animation layer: a stock schema that remaps through relationships.
(2) Add a small panel in plugin/rigExecUsdview with the same model/UI split as pickerModel.py. It applies sets with the shared replace/toggle/remove rules.
(3) The importer applies the prefix remap.

**Evidence:** `libs/rigExecSchema/schema.usda:2490-2494`; `plugin/rigExecUsdview/pickerModel.py:186-228`

**Verification (corrected).** This is not wholly Missing. A RigExecPickerButton is already a persisted, named multi-control selection: ui:text plus rel rigExec:picker:controls, remapped through renames (schema.usda:2390-2406, 2490-2494). The panel applies it with replace, toggle and remove rules (pickerModel.py:240-294). Pickers are discovered by type in any composed layer, so a shot layer can carry them (pickerScene.py:1-17).
Still missing: in-app creation from the current selection (edit mode was removed, pickerUI.py:444-451) and a conventional per-shot storage location. Minor holds.

Verifier evidence: `libs/rigExecSchema/schema.usda:2390-2406`; `libs/rigExecSchema/schema.usda:2490-2494`; `plugin/rigExecUsdview/pickerScene.py:1-17`; `plugin/rigExecUsdview/pickerUI.py:444-451`

### G3-shape-library

**Gizmo shape library (62 named mesh shapes, role-coded vocabulary)**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D2 · *(analyst said Partial / major)*

UE features: `UE1-gizmo-libraries`, `UE7-dmc-gizmo-library`, `UE6-control-visual-conventions`, `UE4-unit-control-display`

**UE rigs.** ModularRigGizmoLibrary and its _DMC twin are UControlRigShapeLibrary assets with 62 named static-mesh shapes, each with a per-shape transform (scale 0.1; Wedge 0.2; RoundedTriangle 1).
- Families, in Thin (1 mm), Thick (3 mm) and Solid variants: Arrow, Arrow2, Arrow4, Box, Circle, Diamond, HalfCircle, Hexagon, Octagon, Pyramid, QuarterCircle, RoundedSquare, RoundedTriangle, Sphere, Square, Star4, Triangle, Wedge.
- Custom: the Pins shapes, RoundedTrapeze(_ArrowTip) and RoundedSquare_ArrowTip.
Shape encodes role: hexagon body/IK, circle FK, pins fingers, triangle secondary FK and lid/corner sliders, diamond PV, half-circle Mid, box IK/attach/eye aim, sphere pivots/brows. Zebra usage: 188 Default, 40 Sphere_Solid, 25 Circle_Thick, 18 Circle_Pins_Thick, 13 Box_Thick, and so on. RigUnit_ShapeExists queries the libraries.

**usdRig today.** guide:shape picks one of 6 synthesized unit primitives: sphere, circle, box, cube, diamond or pyramid. They are drawn as wire curves (guide:wireWidth) or solid geometry (guide:drawMode), sized per axis by guide:scaleX/Y/Z times the evaluated axis length, and coloured and faded. There are no mesh or curve shape assets.

**Gap.** There is no shape library or custom shape source. Missing shapes: arrows, hexagon, octagon, half and quarter circles, triangle, square outline, star, wedge, and the rounded and pin shapes. Line thickness is a single width rather than the Thin/Thick families.

**Porting impact.** The role-coded vocabulary collapses to 6 primitives: triangles, half circles, pins and the arrow-tip root/prop shapes become circles or boxes. The change is visible to animators, and overlapping face and finger controls become hard to tell apart.

**Recommendation.** (1) Add 'rel guide:shapeSource' to RigExecControl in libs/rigExecSchema/schema.usda. It targets a UsdGeomBasisCurves or UsdGeomMesh prototype in a shape-library scope, for example a referenced ControlShapes.usda converted from the UE static meshes with the per-shape transform baked into the points.
(2) Keep guide:shape as the fallback when the target does not resolve.
(3) Publish the prototype's points, scaled and offset, as the synthesized rigGuideCtrl in libs/rigExecImaging/bridge.cpp _FillControlGuides and libs/rigExecImaging/sceneIndices.cpp, reusing the existing wire-width and picking path.
The source is epoch-constant data, so the evaluator does not change.

**Evidence:** `libs/rigExecSchema/schema.usda:167-222`; `libs/rigExecImaging/bridge.cpp:924-1017`; `docs/biped-rig.md:181-184`

**Verification (corrected).** Partial holds (6 tokens, schema.usda:167-179), but major overstates the loss. In the MR_Zebra runtime, 243 controls draw shapes (239 animation controls and 4 proxies):
- 160 are spheres or circles: 'Default' resolves to the library DefaultShape Sphere_solid, plus Sphere_* and Circle_* shapes (ModularRigGizmoLibrary asset.t3d:2).
- 17 are Box_Thick or Diamond_Solid, which also have exact tokens.
- Only 66 need approximation: 20 pins, 14 triangles, 10 hexagons, 10 half circles, 9 arrow, trapezoid or rounded shapes, and 3 others.
Per-axis guide scale can flatten a pyramid or diamond into triangle-like outlines. Exact shapes can already be drawn as native BasisCurves under a constraint-driven native Xform (rigEvaluator.cpp:3396-3410; bridge.cpp:772-787), though they are not picked as the control. This is minor readability loss, not lost workflow.

Verifier evidence: `libs/rigExecSchema/schema.usda:167-222`; `libs/rigExecImaging/bridge.cpp:958-976`; `libs/rigExec/rigEvaluator.cpp:3396-3410`; `libs/rigExecImaging/bridge.cpp:772-787`

### G3-typed-channels

**Typed animation channels (bool/float/scale-float/int) on owner controls, incl. face Jaw Attributes**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D3

UE features: `UE4-channel-hosting`, `UE6-jaw-channel-host`, `UE4-unit-interaction-channels`

**UE rigs.** Channels are child control elements (animation_type ANIMATION_CHANNEL) of type BOOL, FLOAT, SCALE_FLOAT or INTEGER. They are spawned at construction under an owner control with initial/min/max values and limit flags, grouped with the parent in Sequencer, and read at runtime with Get*AnimationChannelFromItem. MR_Zebra has 116 channels: 69 bool, 32 float, 14 scale-float, 1 int. The face 'Jaw Attributes' function adds these float channels on the jaw control: Roll Tp/Bt (-100..200), All/Mid Tp/Bt and Mouth/Muzzle Squetch (-200..200), plus the bool Lip Tweaker Vis. Monster adds Ch/Puff Tp/Bt (0..100).

**usdRig today.** A channel is a custom attribute in the avars: namespace on the owner RigExecControl. Examples are the biped's 'custom float avars:ikfk' and avars:footRoll, which carry doc strings. Consumers read a channel through a single same-typed connection: BlendPointFrames inputs:weight, FloatMathMover inputs:value/min/max, or, for bools only, MoverAPI inputs:enabled. The Avar Editor discovers avars:/foot: attributes and builds widgets by type: float/int spin box plus slider, bool checkbox, token combo box. Keys are Ts splines for double/float/half, otherwise held timeSamples. The attribute default is the initial value.

**Gap.** (1) Nothing declares an attribute to be a channel: no schema or metadata carries its type, a scale role, min/max, limit flags or keyability. (2) Connections must be type-exact with a single source. A bool channel can drive only bool inputs (inputs:enabled). It cannot drive float weights, guide opacity (which accepts float/double only) or FloatMathMover operands (float only). A float dial cannot drive a double avar. (3) No bool or int arithmetic exists, so bool logic has to be re-encoded as float 0/1. (4) The Graph Editor draws curves only for double/float/half; bool and int channels can only be keyed as timeSamples.

**Porting impact.** All channels can be authored as custom avars: 116 on Zebra, and the Monster face channels too. However, every bool channel that feeds a weight or a visibility must be ported as a float 0/1 dial. That changes the animator widget from a checkbox to a slider and allows fractional values UE forbids. Face channels with a -200..200 range get a 0..1 slider by default.

**Recommendation.** (1) Declare channels in metadata, with no new evaluation state. Register SdfMetadata fields in plugin/rigExecSchema/resources/plugInfo.json, next to rigExecReadPhase:
- rigExecChannel (token: bool|float|scaleFloat|int|enum)
- rigExecRange (double[] min,max)
- rigExecRangeEnabled (int[])
- rigExecKeyable (bool)
plugin/rigExecUsdview/avarEditorModel.py (Channel, SliderRange, Coerce) and graphModel.py read them.
(2) Accept lossless source types. In libs/rigExec/rigEvaluator.cpp, have _ValidateScalarConnection and the RigExecResolvedInputs resolver accept bool->float/double (as 0/1) and float<->double sources, and publish the coerced value as an exec value override (the same path property movers use) instead of rejecting it. Accept bool in libs/rigExecImaging/bridge.cpp _HeldScalar.
(3) Add builder helpers AddChannel(control, name, type, default, min, max) to libs/rigExecRigging/rigBuilder.h and python/rigexec/__init__.py.

**Evidence:** `libs/rigExecSchema/schema.usda:327-344`; `examples/biped/Biped.usda:5781-5822`; `libs/rigExec/rigEvaluator.cpp:523-569`; `libs/rigExec/rigEvaluator.cpp:3628-3645`; `plugin/rigExecUsdview/avarEditorModel.py:36-46`; `plugin/rigExecUsdview/avarEditorModel.py:161-178`; `plugin/rigExecUsdview/avarEditorUI.py:91-100`; `docs/graph-editor.md:21`; `plugin/rigExecSchema/resources/plugInfo.json:8-13`

**Verification (holds).** Partial and minor both hold, and the UE counts match the dump (69 bool, 32 float, 14 scale-float, 1 int). Three corrections to the mapping:
- 'Otherwise held timeSamples' is wrong for the tools. avarEditorModel.WriteValue always writes bool and token channels as the default and never keys them (avarEditorModel.py:528-538). Only int is keyed as timeSamples, through gizmoMath.SetAnimated (gizmoMath.py:1291-1312). The picker switch also writes a float default (pickerUI.py:979-982). See the missed row G3-switch-keying.
- The builder cannot author channels at all. RigExecHandleBase::SetAttr and both Python set_attribute bindings reject undeclared or custom attributes (rigBuilder.h:78-82; python/_rigexec.cpp:1255-1268).
- The recommendation should reuse stock OpenUSD 26.08 fields before inventing rigExecRange: attribute 'limits' soft/hard min/max (UsdAttribute::GetSoftLimits/GetHardLimits, attribute.h:829-843) and UsdUI hints for display name, hidden and value labels. Only the channel type and keyable flag need new metadata.
Confirmed: connections are type-exact with a single source (rigEvaluator.cpp:544-566), and _HeldScalar accepts only float or double (bridge.cpp:223-234).

Verifier evidence: `plugin/rigExecUsdview/avarEditorModel.py:528-538`; `plugin/rigExecUsdview/pickerUI.py:979-982`; `libs/rigExecRigging/rigBuilder.h:78-82`; `python/_rigexec.cpp:1255-1268`; `libs/rigExec/rigEvaluator.cpp:544-566`; `libs/rigExecImaging/bridge.cpp:223-234`; `<usd-install>/include/pxr/usd/usd/attribute.h:829-843`

### G3-vis-logic-composition

**Combined and cross-module visibility logic (AND terms, metadata sharing)**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D2

UE features: `UE2-prop-visibility-hosting`, `UE3-sec-controls-vis`, `UE3-ikfk-switch-vis`

**UE rigs.** Visibility terms are combined and shared across modules:
- Prop Global Vis (PGV) starts as NOT(connector connected) and is hosted on root/Global. Each other prop control is visible when PGV AND its own bool are true; PGV counts as true when the channel does not exist.
- 'Sec Controls Vis' (on IK, hosted on the end FK) sets Mid visibility and IK Base = IK && SecVis.
- The same value is published as module metadata 'Sec Controls Visibilty' (sic), which child LimbTwist modules read (default true) to show or hide their twist controls.

**usdRig today.** Property movers can build the logic. A FloatMathMover (multiply, inputs:value.connect to another dial) that targets a helper 'custom float' computes the AND of 0/1 dials. guide:displayOpacity then connects to the helper, and the bridge reads property-mover results from movedProperties. Cross-module sharing is a direct connection to the owning module's dial, so a USD path replaces the metadata lookup.

**Gap.** Missing pieces:
- Boolean ops (and/or/not) over bool channels.
- 'Default when missing' semantics (UE's ItemExists -> true).
Each combined term also costs one helper attribute and one mover, since a property mover has exactly one target.

**Porting impact.** About a dozen combined visibility terms per rig need hand-built float helper chains. This is workable but verbose.

**Recommendation.** (1) Let guide:visibilityDrivers (AND over targets, with 'uniform bool[] guide:visibilityDriverInvert') cover the common cases without movers.
(2) Where logic is needed elsewhere, add RigExecBoolMathMover (and/or/not/xor over bool inputs, exact bool target) to libs/rigExecSchema/schema.usda. Evaluate it in the property-chain code of libs/rigExec/rigEvaluator.cpp next to FloatMathMover.

**Evidence:** `libs/rigExecSchema/schema.usda:1192-1214`; `libs/rigExec/rigEvaluator.cpp:3568-3587`; `libs/rigExec/rigEvaluator.cpp:11659-11665`; `libs/rigExecImaging/bridge.cpp:195-218`

**Verification (holds).** Confirmed:
- FloatMathMover has add, multiply, clamp, remap and blend, but no bool type (schema.usda:1192-1214).
- It takes exactly one target (rigEvaluator.cpp:3543-3550).
- Property-mover results are published in movedProperties (rigEvaluator.cpp:11659-11665), and the bridge reads them through the connection (bridge.cpp:212-216).
Float boolean algebra is complete: AND is multiply, NOT is remap with min=1 and max=0 (remap is unclamped, schema.usda:1203-1205), and OR is add followed by clamp. The gap is therefore helper cost and missing bool-typed operations, not expressiveness. Partial and minor hold.

Verifier evidence: `libs/rigExecSchema/schema.usda:1192-1214`; `libs/rigExec/rigEvaluator.cpp:3543-3550`; `libs/rigExec/rigEvaluator.cpp:11659-11665`; `libs/rigExecImaging/bridge.cpp:195-218`

### G3-foot-rocker-anchor

**Foot Rocker control re-anchored to the foot joint while idle**

**Verdict:** Divergent-by-design · **Severity:** minor · **Effort:** S · **Confidence:** medium · **Domain:** D2

UE features: `UE3-foot-rocker-anchor`

**UE rigs.** While the rocker is not being manipulated, the Foot module sets its offset every frame to (inverse(rockerLocal) * Delta) * FootJoint_current, where Delta is the rocker's initial transform relative to the foot's. The rocker therefore keeps its bind placement relative to the moving foot joint. Offset writes change both the current and the initial transform. A nested Heel-interaction branch is unreachable.

**usdRig today.** A control cannot rewrite its own offset from a pose result: nothing is authored and there is no feedback. The non-cyclic equivalent is to parent the rocker to the pre-roll foot frame. Nest it under the leg IK control, or connect its default:space/parent:space to a relay nested under a provider that is upstream of the roll.

**Gap.** The rocker cannot follow the post-roll foot joint without creating a cycle. UE avoids the cycle only through the offset write gated by interaction state.

**Porting impact.** The rocker gizmo sits relative to the IK control rather than the rolled foot joint. Rocker values and keys evaluate the same.

**Recommendation.** (1) The builder authors the rocker under the IK control.
(2) If on-foot placement is wanted, add 'rel guide:followProvider' to RigExecControl (libs/rigExecSchema/schema.usda). libs/rigExecImaging/bridge.cpp _FillControlGuides uses it to place only the drawn guide relative to a posed provider, and plugin/rigExecUsdview/gizmoMath.py uses it for the manipulator frame. Evaluation is unchanged.

**Evidence:** `docs/spec.md:56-63`; `libs/rigExecSchema/schema.usda:307-357`; `tests/testRigExecConstraints.cpp:3093-3125`

**Verification (holds).** UE writes the control offset (both current and initial) every idle frame from the solved foot, which is a feedback write gated by interaction state. That conflicts with 'OpenExec cannot ... author values' (spec.md:58) and the rejection of hidden state (spec.md:1050).
The UE formula makes the rocker's global transform equal Delta*Foot while idle, independent of its own value, so only gizmo placement differs. Rocker values and poses are unaffected. Minor holds.

Verifier evidence: `docs/spec.md:56-63`; `docs/spec.md:1050`; `libs/rigExecSchema/schema.usda:307-357`

### G3-interaction-state

**Viewport interaction state inside rig evaluation (IsInteracting)**

**Verdict:** Divergent-by-design · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D3

UE features: `UE4-unit-interaction-channels`, `UE3-ikfk-switch-vis`, `UE8-prop-reset-controls-color`

**UE rigs.** RigUnit_IsInteracting exposes bIsInteracting/bIsTranslating/bIsRotating/bIsScaling and the items being manipulated, all taken from the execute context. Users:
- Forward Movable Proxy and ProxyControl.
- Foot Rocker re-anchoring.
- Body aim.
- The dormant IkFk2Bones auto-switch: grabbing an FK control sets IK Solve=false; grabbing IK or PV sets it true.
- Prop change-pivot, which fires the RequestAutoKey event.

**usdRig today.** Evaluation is a pure function of authored or overridden values at a given time. The evaluator sees a gizmo drag only as SetInteractiveOverrides values, never as a gesture flag.

**Gap.** Evaluation cannot depend on gestures: the rig cannot behave differently while a control is being dragged.

**Porting impact.** Every IsInteracting branch must be reimplemented as tool behaviour or dropped. Playback and offline results are unaffected, because UE bakes these effects into keys.

**Recommendation.** Keep the evaluator stateless. Put gesture behaviour in plugin/rigExecUsdview (gizmoDrag.py, gizmoPreview.py), keyed off the proposed RigExecProxyControlAPI modes. If a rig truly needs a gesture input, declare it explicitly: a session-layer custom attribute (e.g. rigExec:interacting as a numeric 0/1) supplied through SetInteractiveOverrides. spec.md:1050 requires any continuity state to be an explicit declared input.

**Evidence:** `docs/spec.md:207`; `docs/spec.md:56-63`; `docs/spec.md:1050`; `docs/viewport-gizmos.md:83-108`

**Verification (holds).** The cited lines exist:
- The event and direct-manipulation layer is deferred (spec.md:207).
- UI and manipulation are non-goals, and OpenExec cannot author values (spec.md:56-63).
- Continuity state must be an explicit input (spec.md:1050).
- Drags reach the evaluator only as overrides (viewport-gizmos.md:83-108).
UE applies these effects while the animator manipulates and then keys the resulting values, so recorded animation does not depend on the gesture. Minor holds.

Verifier evidence: `docs/spec.md:56-63`; `docs/spec.md:207`; `docs/spec.md:1050`; `docs/viewport-gizmos.md:83-108`

### G3-movable-pivot

**Movable pivot proxy (rotate a control about a relocatable pivot)**

**Verdict:** Divergent-by-design · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D2 · *(analyst said Divergent-by-design / major)*

UE features: `UE4-movable-proxy-construct`, `UE2-movable-pivot-proxy`, `UE3-movable-pivot-proxy`

**UE rigs.** Construct Movable Proxy v01 builds:
- '<Name> Null' under Proxy Parent, with the Driven control's initial rotation and Snap To's translation and scale.
- The proxy control '<Name>' (bIsProxy, DrivenControls=[Driven]).
- The bool 'Movable Pivot Vis' on Driven.
- '<Name> Bfr' null carrying metadata IsSet=false.
Forward Movable Proxy v01 runs only while the vis channel is on:
- Translating the proxy stores Driven's global in Bfr and sets IsSet.
- Rotating it stores Bfr on the first frame, then sets Driven global = Bfr global, so Driven orbits the pivot.
- When idle, IsSet=false, the pivot null = (Snap To's current T/S, Driven's current R) and the proxy's local rotation = identity.
Instances: Body Movable Pivot (drives Body Orbit, snaps to the pelvis) and Spine 'Chest Moveable Pivot' (drives End IK).

**usdRig today.** The behaviour depends on viewport gesture state and on state persisted across evaluations (Bfr and IsSet). The evaluator excludes both: the event/direct-manipulation layer is deferred, hidden previous-frame state is rejected, and OpenExec cannot author values. The gizmo's Pivot mode edits rest:t/r, which changes the bind rather than giving an animation pivot, and there is no temporary pose pivot.

**Gap.** There is no way to rotate Body Orbit or the chest IK about a relocatable pivot.

**Porting impact.** Animators lose the chest and body movable-pivot posing workflow. Existing keys play back identically, because UE bakes the result into the driven control's keys.

**Recommendation.** The usdRig-native alternative is tooling, not evaluation:
(1) Add a session-only 'Pose Pivot' to the viewport gizmo (plugin/rigExecUsdview/gizmoMath.py, gizmoDrag.py, gizmoSettings.py), toggled like D/Insert.
(2) The pivot snaps to the provider named by the proposed RigExecProxyControlAPI (mode=movablePivot, rel rigExec:proxy:snapTo).
(3) A rotation about the pivot becomes rotation plus compensating translation avars on the driven control, previewed through SetInteractiveOverrides and authored once on release.
The proxy prim stays an unevaluated guide, drawn at snapTo, whose visibility comes from the Movable Pivot Vis channel.

**Evidence:** `docs/spec.md:207`; `docs/spec.md:1050`; `docs/spec.md:56-63`; `docs/viewport-gizmos.md:52-60`; `docs/viewport-gizmos.md:44-50`

**Verification (corrected).** The verdict holds. The cited lines exist: hidden previous-frame state is rejected (spec.md:1050), stateful callbacks and viewport manipulation are v1 non-goals (spec.md:59, 63), and the event layer is deferred (spec.md:207). The UE mechanism needs the persistent Bfr/IsSet state and IsInteracting.
The severity is overstated, because the tool-side alternative largely exists:
1. Select the driven control, then a free 'pivot' helper control last.
2. Set the group pivot to Last Selected (gizmoMath.py:1284-1288; gizmoSettings.py:73-92).
3. Rotate. GroupTarget orbits the driven control about the helper's origin and authors both the rotation and the compensating translation on it (gizmoMath.py:2730-2766).
The only side effects are a spare rotation key on the helper and manual placement of the helper, so this is minor, not major.

Verifier evidence: `docs/spec.md:56-63`; `docs/spec.md:207`; `docs/spec.md:1050`; `plugin/rigExecUsdview/gizmoMath.py:1284-1288`; `plugin/rigExecUsdview/gizmoMath.py:2708-2766`; `plugin/rigExecUsdview/gizmoUI.py:1819-1829`

### G3-ik-control-setup

**IK control, IK gimbal, effector null and world-orient compensation**

**Verdict:** Implemented · **Severity:** minor · **Effort:** S · **Confidence:** medium · **Domain:** D2

UE features: `UE3-ik-controls-setup`

**UE rigs.** 'IK' is an EULER_TRANSFORM control: Box_Thick shape (or the 'ik-layer.<end>' DMC shape), parented to root/Local with InitialSpace=Global.
- Initial value = (VB[-1].rot * IK Rotation Offset, VB[-1].t).
- Offset translation = the end bone's position.
- Offset rotation = identity when 'IK Compensate World Orient' is on (hands), so the animated rotation equals the world orientation.
Children: the rotator 'IK Gimbal' (Sphere_Thin) and, under it, the null 'IK' (the solver effector). Module metadata 'IK Null', 'IK Control' and 'IK Driver' are published for child modules.

**usdRig today.** Each part maps directly:
- IK control: a top-level RigExecControl whose rest:space sits at the end joint, world-aligned when compensated.
- Zero pose different from rest: default:space or default:rx..rz.
- Gimbal: a nested RigExecControl.
- Effector null: a nested control or a plain Xform (intervening Xforms are composed), named by RigExecTwoBoneIk rigExec:effectorControl.
- Child-module lookup: a direct relationship or connection instead of metadata.

**Gap.** Only authoring differs. UE stores a non-zero initial value while usdRig uses default:space, and the two cannot both match: if default:space carries the bone orientation, avar values become relative to it and imported UE keys no longer match; if rest is world-aligned with an identity default, keys match but Reset returns to world-aligned rather than UE's initial value. The module metadata namespace is covered by G1.

**Porting impact.** To import UE keys one-to-one, author the rest world-aligned with an identity default. The Avar Editor's Reset then returns the control to world-aligned instead of UE's initial value (the bone orientation).

**Recommendation.** The importer authors the IK rest world-aligned and writes UE control values directly into the avars. Record the UE initial value as the reset target in a proposed 'rigExecResetValue' metadata field, honoured by plugin/rigExecUsdview/avarEditorModel.py FallbackValue, rather than moving it into default:space. Both key values and Reset then match UE.

**Evidence:** `libs/rigExecSchema/schema.usda:298-326`; `libs/rigExecSchema/schema.usda:546-549`; `libs/rigExecRigging/rigBuilder.h:1051-1053`; `docs/biped-rig.md:159-163`

**Verification (holds).** Implemented holds, but the stated conflict ('the two cannot both match') is overstated. The importer can:
1. author the rest world-aligned;
2. write the UE initial value as the avars' DEFAULT opinion in the rig layer;
3. write imported keys unchanged.
Default-mode Reset clears only the edit-target opinion, revealing the rig-layer default, which is the UE initial value (avarEditorModel.py:545-571).
One mismatch remains. Animation-mode Reset of a keyed channel keys FallbackValue, which prefers the schema fallback 0 over the authored default (avarEditorModel.py:201-214, 557-562). Minor is right.

Verifier evidence: `plugin/rigExecUsdview/avarEditorModel.py:201-227`; `plugin/rigExecUsdview/avarEditorModel.py:545-571`; `libs/rigExecSchema/schema.usda:298-326`; `libs/rigExecSchema/schema.usda:546-549`

### G3-rotation-orders

**Preferred rotation order per control family**

**Verdict:** Implemented · **Severity:** minor · **Effort:** S · **Confidence:** medium · **Domain:** D2

UE features: `UE1-rotation-orders`, `UE6-control-visual-conventions`

**UE rigs.** FRigControlSettings.PreferredRotationOrder is used when bUsePreferredRotationOrder is set, and selects the Euler order of the control's rotation channels and keys:
- XYZ: limb FK/IK/PV, spine, fingers, FkArray.
- XZY: clavicles.
- YXZ: Prop and Foot Rocker.
- ZYX: the Ball/Heel/Toe Tip pivots.
- YZX: channels, root/Body/twist/toe controls, and all face controls (where it is not enabled).
Zebra counts: XYZ 82, YZX 261, YXZ 8, ZYX 6, XZY 2. Only 43 settings in the modules enable the flag.

**usdRig today.** avars:rotationOrder (XYZ, XZY, YXZ, YZX, ZXY or ZYX) is a per-prim token read on every evaluation. Rotations are applied in that sequence (leftmost first, row-vector convention) and rspin is applied last. The baked program reads the same token.

**Gap.** No functional gap. Caveats:
- rest:/default: rotations are always XYZ.
- UE's 'order applies only when enabled' flag has no counterpart, so the token should be authored only where UE enables it.
- The mapping from UE EEulerRotationOrder names to usdRig's row-vector sequence must be verified numerically.

**Porting impact.** Imported Euler curves evaluate correctly only if the order token is mapped correctly per control. A wrong mapping silently changes the rotation of about 71 non-XYZ controls.

**Recommendation.** No schema change. Add an importer mapping table plus a parity test under tests/python that compares UE FRotator/EulerTransform quaternions with computations.cpp _ComposeAvars for all six orders.

**Evidence:** `libs/rigExecSchema/schema.usda:334-336`; `libs/rigExec/computations.cpp:189-222`; `libs/rigExec/computations.cpp:355-378`; `libs/rigExec/bakedPose.cpp:1952-1953`

**Verification (holds).** Semantics were verified on both sides. UE AnimationCore::QuatFromEuler with order XYZ expands to qZ*qY*qX, so X applies first (AnimationCoreLibrary.cpp:211-216). usdRig _ComposeAvars multiplies the rotations in token order with row vectors, so XYZ also applies X first (computations.cpp:189-222). The tokens therefore map by name.
The remaining risk is the handedness flip: UE negates X and Y when bUseUEHandyness is set (AnimationCoreLibrary.cpp:197-201), which belongs to the importer's left-handed to right-handed conversion.
The runtime counts match (YZX 261, XYZ 82, YXZ 8, ZYX 6, XZY 2). The baked program reads the same token (bakedPose.cpp:1952-1953).

Verifier evidence: `libs/rigExec/computations.cpp:189-222`; `libs/rigExec/computations.cpp:365-378`; `libs/rigExec/bakedPose.cpp:1952-1953`; `<UE>/Source/Runtime/AnimationCore/Private/AnimationCoreLibrary.cpp:191-216`

### G3-shape-name-resolution

**Shape-name resolution, library precedence and construction-time shape lookup**

**Verdict:** Not-applicable · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D2 · *(analyst said Divergent-by-design / minor)*

UE features: `UE8-asset-shape-library-resolution`, `UE4-shape-name-from-item-v01`, `UE2-dmc-shape-library-switch`

**UE rigs.** Each asset lists its ShapeLibraries in order: MR_Zebra [ModularRig]; MR_ZebraDMC [DMC, ModularRig]; MR_Monster [DMC] only; and so on. GetShapeByName splits 'Lib.Shape' and resolves in two passes:
- Pass 0 walks the libraries last to first and matches the namespace.
- Pass 1 ignores the namespace; the default-shape fallback applies only to library 0 in this pass.
- Libraries added at runtime (SetupShapeLibraryFromUserData with CRSL/'ShapeLibrary' mesh user data, or SetupShapeLibraryFromLayer) are appended and therefore win.
Construction helpers: Get Control Shape Name From Item v01 returns '<ns>.<resolved item>' if it exists, else 'Circle_Thick'; v02 adds the DMC-library and Default logic. FkChain and FkArray skip Set Control Scale when a CRSL library was found. MR_Monster's 'ModularRigGizmoLibrary.*' names resolve through the pass-1 fallback into the DMC library.

**usdRig today.** Name-based lookup has no counterpart. A port resolves every UE shape name once, at import. The usdRig-native form of library precedence is USD composition: shape-library layers are sublayered or referenced in order (the stronger layer wins), and relationships survive renames.

**Gap.** There is no runtime shape registry, no ShapeExists query and no injection of libraries from mesh user data. The UE precedence and fallback rules must be replayed by the importer.

**Porting impact.** To pick the same shapes, the importer must reproduce UE's exact resolution, including Monster's namespace fallback and the CRSL override. After that the result is static data.

**Recommendation.** (1) Implement the resolution in the UE-to-USD importer, outside the evaluator, and emit guide:shapeSource relationships into a shape-library layer.
(2) Order competing libraries as sublayers, so composition rather than code expresses 'the later library wins'.
(3) Emit a diagnostic from libs/rigExecImaging/bridge.cpp when a guide:shapeSource target does not resolve and guide:shape is used instead.

**Evidence:** `libs/rigExecSchema/schema.usda:167-179`; `libs/rigExecSchema/schema.usda:2395-2406`

**Verification (corrected).** Divergent-by-design requires a cited non-goal, and neither citation is one. schema.usda:167-179 is the guide:shape token doc. schema.usda:2395-2406 is the picker-button liveness doc (relationships over path strings for picker targets).
Name lookup, library precedence and ShapeExists are construction-time UE plumbing. A ported rig carries their result as static per-control data, and the shape vocabulary itself is scored in G3-shape-library. The row is therefore Not-applicable, with a minor importer obligation to replay the precedence and fallback rules (Monster's namespace fallback, the CRSL override).

Verifier evidence: `libs/rigExecSchema/schema.usda:167-179`; `libs/rigExecSchema/schema.usda:2395-2406`

### G3-debug-draw-primitives

**Generic and solver-internal debug drawing (axes, vectors, arcs, rectangles)**

**Verdict:** Missing · **Severity:** cosmetic · **Effort:** M · **Confidence:** high · **Domain:** D19

UE features: `UE4-draw-axis`, `UE4-embedded-debug-draw`, `UE4-rigvm-debug-draw`, `UE3-ik-debug-draw`

**UE rigs.** RigVM debug-draw nodes can sit inline in graphs:
- DebugTransformMutableNoSpace: axes, point or box, with scale, thickness and world offset.
- DebugLine / DebugLineStrip, DebugRectangle and DebugArc.
- VisualDebugVector: draws a vector and passes its value through.
Uses in the rigs:
- CRFL 'Draw Axis' (scale 10, thickness 0.2), exposed as LimbTwist 'Debug Axis'.
- Pole-vector helpers draw the projected mid, the current and reference PV vectors, and plane normals.
- IkFk2Bones 'Debug' draws virtual-bone axes, the Auto PV parent and soft-IK arcs.
All of this is gated by module Debug options, which are off by default.

**usdRig today.** Synthesized diagnostics exist only as fixed guide families: joint spheres and cones, solver element spheres and cones, volume iso-surfaces, and curvenet curves. The planned RestGuideAPI, which adds axis triads, is still 'awaiting review'. There are no triads, vectors, arcs or rectangles, and a computation cannot emit debug geometry.

**Gap.** There is no generic debug-draw channel and no way to show solver internals.

**Porting impact.** Developer-only visualisation, off by default, is lost. Animators are not affected.

**Recommendation.** (1) Build the RestGuideAPI spec (axis triads), and add 'uniform bool guide:drawAxes' and 'double guide:axesScale' to RigExecXformable and the solvers.
(2) For solver internals (virtual bones, pole vectors, soft-IK arcs), add an optional per-solver debug tap, like the existing solver guide taps in libs/rigExec/rigEvaluator.cpp. It publishes RigExecRigPose::debugPrimitives, which libs/rigExecImaging/bridge.cpp draws, and stays off unless 'rigExec:debugDraw' is true.

**Evidence:** `libs/rigExecSchema/schema.usda:382-404`; `libs/rigExecSchema/schema.usda:507-521`; `docs/superpowers/specs/2026-09-06-solver-rest-guides-design.md:3`; `libs/rigExecImaging/sceneIndices.cpp:345-352`

**Verification (holds).** Confirmed: RestGuideAPI is 'awaiting review' (2026-09-06-solver-rest-guides-design.md:3), and nothing in libs, plugin or python references it.
Axis tripods alone could be faked with native curves under constraint-driven Xforms. Solver internals (virtual bones, pole vectors, soft-IK arcs) cannot be shown. The draws are off by default, so cosmetic holds.

Verifier evidence: `docs/superpowers/specs/2026-09-06-solver-rest-guides-design.md:3`; `libs/rigExecSchema/schema.usda:382-404`; `libs/rigExecImaging/sceneIndices.cpp:345-352`

### G3-posereader-deformer-debug

**Pose-reader cone and GPU deformer debug visualisation**

**Verdict:** Missing · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D19

UE features: `UE-debug-draw`, `UE7-dg-debug-draw`

**UE rigs.** Spherical Pose Reader debug settings (inner and outer cones, driver vector coloured by output, 2D or 3D, scale 25, 20 segments) are off on all 49 readers. Every Optimus deformer graph carries OptimusDebugDrawDataInterface with bForceEnable. The kernels draw bend/twist/squash planes and axes only when EnableDebugDraw != 0, and that is a constant 0 in all 17 graphs. Retargeter ops enable bDebugDraw.

**usdRig today.** usdRig has no cone or spherical pose reader and no parametric bend/twist/squash movers, so there is nothing of this kind to visualise. Weight objects have their own overlays: the weight influence overlay and volume iso-surfaces.

**Gap.** The visualisation belongs to operators that do not exist yet (covered by G6 and G7).

**Porting impact.** None: all of it is disabled in the shipped assets.

**Recommendation.** When RigExecSphericalPoseReader and the parametric deformers are added (G6/G7), give them 'guide:drawDebug' (cones, limit planes), drawn by libs/rigExecImaging/bridge.cpp in the same way as the volume-weight guides.

**Evidence:** `libs/rigExecImaging/registry.h:292-306`

**Verification (holds).** Confirmed: the only field overlay in the imaging API is the weight overlay (registry.h:292-306). Every UE draw flag is off in the shipped assets.

Verifier evidence: `libs/rigExecImaging/registry.h:292-306`

### G3-stretch-feedback

**Squash/stretch colour-coded feedback lines**

**Verdict:** Missing · **Severity:** cosmetic · **Effort:** S · **Confidence:** medium · **Domain:** D19

UE features: `UE3-stretch-feedback`

**UE rigs.** CRM_FN_BipedStretchFeedback exists only in the biped templates. A bool 'Stretch FeedBack Vis' sits on root/Global. For each chain segment, r = current length / initial length, and the segment's DebugLine (thickness 1) is coloured:
- green if |r - 1| <= 0.01
- lerp(white, red, (r - 1) / (MaxStretch - 1)) when stretching
- lerp(blue, white, r) when squashing
Max Stretch is 5 (2 in the biped). The right arm and leg lists are derived by renaming _l to _r.

**usdRig today.** No guide can be coloured by a measurement. Joint guide colour is a constant; only controls can connect their guide colour.

**Gap.** There is no guide whose colour depends on posed versus rest segment length.

**Porting impact.** None for Zebra and Monster, which do not instance the module. The biped templates lose a QA aid.

**Recommendation.** Add a 'colorBy = stretchRatio' mode to the proposed RigExecGuideLine: 'rel rigExec:guide:chain' (ordered joints), rest/squash/stretch colours and maxStretch. libs/rigExecImaging/bridge.cpp computes the colours from jointFramesFinal and rest frames, which would be added to RigExecRigPose. The change is imaging-only.

**Evidence:** `libs/rigExecSchema/schema.usda:396-404`; `libs/rigExecImaging/bridge.cpp:242-292`

**Verification (holds).** The row's claim that joint guide colour 'is a constant' is inaccurate. _ReadGuideStyle, which follows connections, runs for joints too (bridge.cpp:878-882).
The verdict holds regardless. Property movers run before the pose walk, and pose interpolators read rotations, so nothing can compute a colour from posed versus rest segment length. The module is biped-template only, so cosmetic.

Verifier evidence: `libs/rigExecImaging/bridge.cpp:878-882`; `libs/rigExecSchema/schema.usda:396-404`

### G3-display-names

**Per-control display names from module config arrays**

**Verdict:** Partial · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D2

UE features: `UE2-display-names`, `UE8-display-name-lookup`

**UE rigs.** Settings.DisplayName = Array[i] when 0 <= i < Num(Array); otherwise it is None and the element name is shown. Sources:
- FkChain 'Display Names' and FkArray 'Display Names Sorted'.
- LimbTwist 'Display Names': Zebra twist controls show 'Offset 1' and 'Offset 2', then 'Twist 3' and 'Twist 4'.
- Spine: the FK primary and secondary arrays, plus IK Start/Mid/End and End Moveable Pivot names ('Pelvis FK', 'Chest', 'Chest Moveable Pivot').
The names appear in the Anim Outliner and in Sequencer as 'Module / DisplayName'.

**usdRig today.** Stock USD prim 'displayName' metadata (UsdObject::GetDisplayName) is available, and usdview's prim browser can show it (Show Prim DisplayName). RigExec panels (Avar Editor, Graph Editor, picker, Execution Stack) show prim and attribute names. Picker buttons have their own ui:text labels.

**Gap.** RigExec panels ignore displayName, there is no builder helper, and channel (attribute) display names are not shown either.

**Porting impact.** RigExec panels show element names ('FK 0', 'Twist 3') instead of 'Pelvis FK' or 'Offset 1', unless the animator switches usdview to display-name mode.

**Recommendation.** (1) The importer writes prim displayName, and attribute displayName for channels.
(2) plugin/rigExecUsdview/avarEditorModel.py (Channel label), graphModel.py and execStackUI.py prefer GetDisplayName().
(3) Add SetDisplayName to libs/rigExecRigging/rigBuilder.h.

**Evidence:** `<usd-install>/include/pxr/usd/usd/object.h:622`; `<usd-install>/lib/site-packages/pxr/Usdviewq/appController.py:985-986`; `libs/rigExecSchema/schema.usda:2446-2448`; `libs/rigExecRigging/rigBuilder.h:119-138`

**Verification (holds).** Confirmed: RigExec panels use prim and attribute names. The only GetDisplayName uses in the plugin are layer names (compositionArcsModel.py, layerOpinionsModel.py:575-576).
Recommendation fix: in 26.08, UsdObject::GetDisplayName is deprecated in favour of UsdUIObjectHints (object.h:617-622; objectHints.h:80-84). The importer and panels should use the uiHints displayName.

Verifier evidence: `plugin/rigExecUsdview/layerOpinionsModel.py:575-576`; `plugin/rigExecUsdview/avarEditorModel.py:236-240`; `<usd-install>/include/pxr/usd/usd/object.h:615-622`; `<usd-install>/include/pxr/usd/usdUI/objectHints.h:80-84`

### G3-footprint-display-proxy

**Footprint Display (display-only proxy shape)**

**Verdict:** Partial · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D2

UE features: `UE3-footprint-proxy`

**UE rigs.** Foot construction spawns 'Footprint Display' under the leg IK control:
- offset = lerp(Toe Tip Pivot, Heel Pivot, 0.5)
- a proxy with UserDefined visibility and shape RoundedSquare_Solid
- shape scale X = |Inner - Outer| / 8 and Y = |ToeTip - Heel| / 8
- colour copied from the IK control
A bool 'Footprint Vis' (false) sits on the IK control, and visibility = Footprint Vis && IK. The result is a non-keyable outline of the sole.

**usdRig today.** The usdRig equivalent is a RigExecControl nested under the IK control:
- rest:space at the midpoint.
- guide:shape 'box' (planar) with guide:scaleX/Y from the measured widths.
- guide:displayColor connected to the IK control's colour.
- guide:displayOpacity connected to a helper float equal to footprintVis * ikfk, computed by a FloatMathMover.

**Gap.** There is no rounded-square shape and no display-only (non-selectable, non-keyable) flag, and the AND term needs a helper mover.

**Porting impact.** Cosmetic: the footprint draws as a plain box and can be selected and keyed, with no effect on the pose.

**Recommendation.** Use RigExecProxyControlAPI mode=display, which excludes the prim from gizmo and marquee selection and from the Avar Editor. Use the proposed guide:shapeSource for the rounded square.

**Evidence:** `libs/rigExecSchema/schema.usda:167-222`; `libs/rigExecImaging/bridge.cpp:195-218`; `libs/rigExecSchema/schema.usda:1192-1214`

**Verification (holds).** Partial and cosmetic hold. Two notes:
1. The bridge follows only ONE connection hop and then calls source.Get (bridge.cpp:207-217). If the IK control's guide:displayColor is itself connected to a palette, a footprint connected to the IK colour draws the IK's local fallback, not the palette. Connect the footprint directly to the palette.
2. There is a closer display-only alternative today. Put an exact rounded-square BasisCurves under a native Xform that a ParentConstraint drives from the leg IK control: constraints may target any UsdGeomXformable (rigEvaluator.cpp:3396-3410), and the revised matrix is published for child geometry (bridge.cpp:772-787). The result is neither a control nor keyable, although it can still be picked as a mesh prim.

Verifier evidence: `libs/rigExecImaging/bridge.cpp:195-218`; `libs/rigExec/rigEvaluator.cpp:3396-3410`; `libs/rigExecImaging/bridge.cpp:772-787`; `libs/rigExecSchema/schema.usda:167-222`

### G3-runtime-color-feedback

**Runtime colour feedback (Prop Change Pivot, Reset Controls Color, copied colours)**

**Verdict:** Partial · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D2

UE features: `UE8-prop-reset-controls-color`, `UE4-unit-control-display`, `UE3-footprint-proxy`

**UE rigs.** Colours also change during the forward solve.
- When a Prop 'Change Pivot' bool is on, the prop controls are painted grey, Prop Global/Local red, and all of them are forced visible.
- When it is off, the prop's world transform is cached, RequestAutoKey is sent, and 'Reset Controls Color' restores the colours stored at construction in 'Color' metadata.
- Footprint copies the IK control's colour (GetShapeSettings, SetControlColor).

**usdRig today.** Connect guide:displayColor to a helper color3f. A RigExecVec3fMathMover (operation blend, value grey or red, defaultWeight connected to a float 'changePivot' dial) revises the helper. Property-mover results are published in movedProperties, and the bridge follows them through the connection. Copying another control's colour is a connection to its colour attribute. Auto-key events have no counterpart.

**Gap.** Each colour needs one helper attribute and one mover. A mover that targets guide:displayColor directly is not drawn, because the bridge consults movedProperties only through a connection. There is no event or auto-key hook, and Change Pivot itself depends on interaction (see G3-interaction-state).

**Porting impact.** The prop pivot-mode colour feedback can be reproduced; the auto-key side effect cannot.

**Recommendation.** In libs/rigExecImaging/bridge.cpp _ReadGuideStyle, also look up pose.movedProperties for the guide attribute's own path, so a Vec3fMathMover or FloatMathMover can target guide:displayColor or guide:displayOpacity directly. Leave auto-key to tooling, at gizmo commit.

**Evidence:** `libs/rigExecSchema/schema.usda:2098-2117`; `libs/rigExec/rigEvaluator.cpp:3568-3587`; `libs/rigExec/rigEvaluator.cpp:11659-11665`; `libs/rigExecImaging/bridge.cpp:195-218`

**Verification (holds).** Confirmed on both points:
- movedProperties is consulted only for the connection source (bridge.cpp:212-216). A Vec3fMathMover may target a color3f guide:displayColor (rigEvaluator.cpp:3573-3584), but the result would not be drawn because the unconnected path uses plain Get (bridge.cpp:257).
- inputs:defaultWeight accepts float connections (rigEvaluator.cpp:3632-3635).

Verifier evidence: `libs/rigExecImaging/bridge.cpp:212-216`; `libs/rigExecImaging/bridge.cpp:251-259`; `libs/rigExec/rigEvaluator.cpp:3573-3584`; `libs/rigExec/rigEvaluator.cpp:3632-3643`

### G3-side-colors

**Side-based control colours from Root metadata with grey-override rule**

**Verdict:** Implemented · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D2

UE features: `UE4-color-from-metadata-v01`, `UE4-color-from-metadata-v02`, `UE4-color-override-from-metadata`, `UE2-side-colors`, `UE1-control-colors`, `UE4-color-by-position`, `UE6-control-visual-conventions`, `UE3-control-scale-color-shape`

**UE rigs.** Colour is resolved at construction. An override colour with R==G==B (white or grey) means 'use the Root metadata side colour': 'Global Right/Left/Center Control Color' (red, blue, yellow).
- v01 picks the side from the module-name suffix ' R' or ' L' (case- and space-sensitive).
- v02 also calls HasSide(bone, 'r'/'l'), which matches an s_ prefix, _s_ infix or _s suffix; right wins.
- The fallback chain is authored colour, then module colour, then side colour.
Per-module overrides: Spine IK orange, Sec FK pink, LimbTwist red/blue, thumbs, proxies white; Spine Alt Color = Color*(0.5,0.2,0.2); Body Offset = Body*0.5. Face: L blue, R red, C yellow, lip magenta, skull orange, squash cyan. Set Contol Color By Position (colour by world-axis sign) exists but is unused.

**usdRig today.** guide:displayColor is authored per control. It can instead be connected to a shared color3f attribute, for example custom palette attributes on the RigExecRoot; the bridge follows the connection at the evaluated time, so the whole rig's side palette is one edit. The layered biped authors blue versus red on its mirrored right-side layer.

**Gap.** The side and grey-override decision is builder logic, and the repo has no builder or importer helper for it (rigBuilder.h has no guide or colour setters).

**Porting impact.** No visible difference once the importer bakes or connects the resolved colours.

**Recommendation.** (1) The importer applies the UE rule and connects each control's guide:displayColor to 'custom color3f rig:palette:left/right/center' on the RigExecRoot; explicit overrides stay local values.
(2) Add a RigExecControlHandle::SetGuideColor helper in libs/rigExecRigging/rigBuilder.h and python/rigexec.

**Evidence:** `libs/rigExecSchema/schema.usda:216-222`; `libs/rigExecImaging/bridge.cpp:195-218`; `libs/rigExecImaging/bridge.cpp:242-262`; `libs/rigExecRigging/rigBuilder.h:119-138`

**Verification (holds).** Confirmed: displayColor follows a connection at the evaluated time (bridge.cpp:251-259).
Caveat: the bridge follows exactly ONE hop (source.Get, bridge.cpp:207-217). A derived colour (Spine Alt Color = Color*(0.5,0.2,0.2), Body Offset = Body*0.5) or a copied colour must therefore connect straight to a palette attribute or to a Vec3fMathMover-revised helper. It cannot chain through another control's connected colour. Implemented and cosmetic hold.

Verifier evidence: `libs/rigExecImaging/bridge.cpp:195-218`; `libs/rigExecImaging/bridge.cpp:251-259`; `libs/rigExecSchema/schema.usda:216-222`

### G3-dangling-shape-refs

**Dangling /EpicControlRig shape and material references**

**Verdict:** Not-applicable · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D2

UE features: `UE8-gizmo-epiccontrolrig-dangling`

**UE rigs.** In ModularRigGizmoLibrary_DMC, shapes 54-61 (pin, rounded-trapeze and arrow-tip shapes) and the X-ray material point at a non-existent /EpicControlRig mount, so those shapes draw nothing.
- MR_FN_BipedDMC: 30 Circle_Pins_Thick controls plus 3 more are affected.
- MR_Monster: 4 RoundedTrapeze_ArrowTip_Thick and 1 RoundedSquare_ArrowTip_Thick are affected.
- MR_Zebra and MR_ZebraDMC are unaffected.
CRU_PropAim's preview mesh is also missing.

**usdRig today.** This is a UE asset-path bug. In usdRig, a missing guide:shapeSource target would resolve to an empty relationship, which is detectable, and guide:shape would be drawn instead.

**Gap.** No usdRig gap. The importer must choose between reproducing the invisible Monster controls and remapping /EpicControlRig/Controls/* to /FortniteRigs/Controls/*.

**Porting impact.** Monster's affected controls should be remapped to the FortniteRigs meshes rather than ported as invisible.

**Recommendation.** Add an importer remap table from /EpicControlRig/Controls/* to /FortniteRigs/Controls/*, and a libs/rigExecImaging/bridge.cpp diagnostic when guide:shapeSource does not resolve.

**Evidence:** `libs/rigExecSchema/schema.usda:2395-2406`

**Verification (holds).** This is a UE asset-path bug. The cited usdRig evidence (the picker liveness doc) is only loosely related, but the verdict does not depend on it.

Verifier evidence: `libs/rigExecSchema/schema.usda:2395-2406`

## G4 — Spaces, FK, constraints, inverse and matching

usdRig can already express most of the rigid-follow behaviour in the Zebra and Monster rigs. Controls driving bones, bones following virtual bones, the head attach null, the root/body/prop control stacks, orient-only space nulls and FK chains that run through driven bones all map to RigExecParentConstraint or PositionConstraint, plus namespace propagation. Propagation also works through solver-posed and constraint-posed joints: rigEvaluator.cpp:10661-10751 and testUsdviewParamNode.py confirm it, and docs/biped-rig.md:136-141 appears to be out of date. Every such mapping still depends on converter-baked offsets. usdRig has no maintain-offset option, per-source offsets carry no scale, rotation averaging works per Euler component rather than by quaternion, and offsets and masks can only be expressed in asset space. Space switching is the largest structural gap on the forward side. There is no labeled space list, no active-space channel, no default-parent designation and no switch compensation. inputs:sourceWeights is read raw and cannot be connected, so the Sequencer space keys and the 22 Zebra spaced controls need hand-built constraint stacks. Face mouth logic is weakened because per-parent weights cannot be driven by curves, avars (double) cannot feed float mover chains, and pose-interpolator outputs are published only after the pose walk. That same scheduling rule makes the CR_*_Deform ModifyTransforms layer (86 reader-weighted helper-joint offsets) impossible to reproduce today, which I rate the one blocker in this group. Everything inverse is missing: backwards solves / bake-to-controls, root-motion bake target, spine back-fit, IK/FK snap, Match Limb, and autokey on switch. Continuous IK/FK auto-matching in both directions cannot be expressed, because usdRig dependencies are structural and the pose DAG would contain a cycle. Interaction-state features (Movable Pivot, Change Pivot, Aim Twist sync, buffered aim blends) are divergent by design under spec non-goals docs/spec.md:59, 207 and 1050; the recommended alternatives are gizmo and picker tools that author compensated values, plus an optional avars pivot. The recommendations fall into five parts: a RigExecSpaceSwitch schema with an int activeSpace; rigExec:maintainOffset (rest/default/delta) and rotationBlend/offsetSpace tokens on source constraints; a RigExecLocalOffsetConstraint; reader steps scheduled inside the pose DAG; and a codeless RigExecMatchAPI consumed by a new python/rigexec/match.py and picker commands.

| Row | Verdict | Severity | Effort | Summary |
|---|---|---|---|---|
| [`G4-backsolve-analytic`](#g4-backsolve-analytic) | Missing | major | L | No inverse graph, per-operator inverse or bake-to-controls tool; rigexec.solve_parameters is only a generic LM solver |
| [`G4-backsolve-spine`](#g4-backsolve-spine) | Missing | major | M | No RigExecSplineIk back-fit or runtime control-offset rewrite (default:* is authoring-only); solve_parameters could fit |
| [`G4-ikfk-snap`](#g4-ikfk-snap) | Missing | major | M | No IK/FK match data, pole-vector helper or snap tool; picker has only zero_ctrls, gizmo avar inversion is reusable |
| [`G4-module-events-match-limb`](#g4-module-events-match-limb) | Missing | major | M | No module event or command system beyond picker zero_ctrls; the IK/FK dial is keyed by an external script |
| [`G4-available-spaces-switching`](#g4-available-spaces-switching) | Partial | major | L | No space-switch schema or switch compensation; sourceWeights not connectable; only a picker-cycled float dial |
| [`G4-expression-weighted-parents`](#g4-expression-weighted-parents) | Partial | major | L | sourceWeights not connectable, double avars can't feed float chains, pose-interpolator weights publish after pose walk |
| [`G4-modify-transforms`](#g4-modify-transforms) | Partial | major | L | Stacked local offsets exist (self-sourced ParentConstraint, probe C1a); pose-reader weights cannot reach it (probe C2) |
| [`G4-seq-space-keys`](#g4-seq-space-keys) | Partial | major | M | No space-channel type or switch-with-compensation/bake tool; stepped playback via held Ts knots on defaultWeight dials |
| [`G4-space-dependency-cycles`](#g4-space-dependency-cycles) | Partial | major | M | Prop-hand and arm-IK space constraints form a compile cycle; weights can't gate deps, no diagnostic names the pair |
| [`G4-backsolve-root-motion`](#g4-backsolve-root-motion) | Missing | minor | S | No root-motion bake mode; the 'Bake Root On' channel itself fits an int/token avar with picker attributeLabels |
| [`G4-maintain-offset`](#g4-maintain-offset) | Missing | minor | M | No maintain-offset: evaluator never computes offsets, converter must bake them (no scale, stale if rest:space changes) |
| [`G4-anim-baked-import`](#g4-anim-baked-import) | Partial | minor | M | No UE/FBX/UsdSkel ingest or quaternion channel; tracks become joint avars or time-sampled posed:space, curves floats |
| [`G4-compute-fk-aim-chain`](#g4-compute-fk-aim-chain) | Partial | minor | M | Per-joint RigExecAimConstraint (objectRotationUp, joint as its own up object) rebuilds the aim; no distance stretch |
| [`G4-envelope-blend-space`](#g4-envelope-blend-space) | Partial | minor | M | Envelope blends asset-space Euler per axis, not parent-local slerp; affect* masks use asset axes, no filter order |
| [`G4-eye-aim`](#g4-eye-aim) | Partial | minor | S | RigExecAimConstraint maps; rotationOffset is an asset-space Euler add, so nest the eye null under an aim provider |
| [`G4-fk-chain-construction`](#g4-fk-chain-construction) | Partial | minor | M | RigExecFkChain has no per-element control-to-joint offset; use inverse-offset child providers or ParentConstraints |
| [`G4-fk-drive-bones`](#g4-fk-drive-bones) | Partial | minor | M | Drive Bones maps to inputs:enabled; no additive control-delta operator (nested controls approximate), writes propagate |
| [`G4-gimbal-controls`](#g4-gimbal-controls) | Partial | minor | M | Nested RigExecControl with own avars:rotationOrder; no rotation-only lock, visibility not drivable by bool or IK/FK mode |
| [`G4-gizmo-constraint-lock`](#g4-gizmo-constraint-lock) | Partial | minor | S | Gizmo locks every RigExecParentConstraint target even if disabled or zero weight; Aim/Position/Rotation stay editable |
| [`G4-ikfk-auto-matching`](#g4-ikfk-auto-matching) | Partial | minor | M | No value-gated dependencies; an acyclic Position/Rotation constraint wiring may give two-way follow (not compiled) |
| [`G4-lid-multi-parent`](#g4-lid-multi-parent) | Partial | minor | M | 5-source ParentConstraint with static sourceWeights; rotation averaged per Euler component, not by quaternion |
| [`G4-new-parent-delta-follow`](#g4-new-parent-delta-follow) | Partial | minor | M | No delta-follow operator; carrier providers per item set ride propagation deltas, 0.5 weights blend per Euler axis |
| [`G4-position-local-offset`](#g4-position-local-offset) | Partial | minor | S | RigExecPositionConstraint with sourceWeights [w,1-w]; translationOffset and masks are asset-space, not parent-local |
| [`G4-rigvm-math`](#g4-rigvm-math) | Partial | minor | L | No pose-dependent math at eval time; property movers (add/multiply/clamp/remap/blend) run before the pose walk |
| [`G4-space-follow-nulls`](#g4-space-follow-nulls) | Partial | minor | M | One RigExecParentConstraint per null with converter-baked offsets (no scale, stale on rest edit); RigExecJoint as null |
| [`G4-weighted-aim-buffered`](#g4-weighted-aim-buffered) | Partial | minor | S | AimConstraint with defaultWeight dial blends from current rotation; UE's remembered buffer is rejected hidden state |
| [`G4-aim-twist-interaction`](#g4-aim-twist-interaction) | Divergent-by-design | minor | M | By design no interaction-aware eval or autokey; only a RotationConstraint on Body from Aim with asset-axis masks |
| [`G4-autokey-on-switch`](#g4-autokey-on-switch) | Divergent-by-design | minor | S | By design evaluation never authors values; no key-set definition, keys come from gizmo and Avar Editor tools |
| [`G4-change-pivot`](#g4-change-pivot) | Divergent-by-design | minor | M | By design no pose-mode tool pins a child in world while its parent moves; Preserve Children works only in Pivot mode |
| [`G4-movable-pivot-proxy`](#g4-movable-pivot-proxy) | Divergent-by-design | minor | M | Interaction proxy rejected by design, but a negated avar connection (unitScaleFactor=-1) builds a static pivot rig |
| [`G4-transform-get-set-units`](#g4-transform-get-set-units) | Divergent-by-design | minor | S | By design no imperative gets/sets; writes become rest:space, default:*, constraints, envelopes and baked offsets |
| [`G4-control-drives-bone`](#g4-control-drives-bone) | Implemented | minor | S | Maps to RigExecParentConstraint with baked offsets; no scaleOffset, non-uniform scale decomposes differently via SVD |
| [`G4-fk-through-driven-bones`](#g4-fk-through-driven-bones) | Implemented | minor | S | Maps to controls nested under constrained joints, re-posed by namespace propagation; mover order must be authored |
| [`G4-pelvis-txy-space`](#g4-pelvis-txy-space) | Implemented | minor | S | Maps to RigExecPositionConstraint with affectTranslationZ=false on a stand-in; pelvis drive must re-run after root |
| [`G4-root-body-prop-stacks`](#g4-root-body-prop-stacks) | Implemented | minor | S | Maps to nested RigExecControls with rest:space and no joint outputs, usable as constraint sources; shapes are G3 |
| [`G4-default-space-multiparent`](#g4-default-space-multiparent) | Partial | cosmetic | S | No separate default-space field or per-channel weights; an authored one-hot weight default covers Zebra (cosmetic) |
| [`G4-get-array-parents`](#g4-get-array-parents) | Implemented | cosmetic | S | Maps to the USD namespace parent, queryable by tools; a parent provider is an ordinary constraint target |
| [`G4-head-attach-null`](#g4-head-attach-null) | Implemented | cosmetic | S | Maps to the face root provider nested under the head joint via namespace propagation, or a baked-offset ParentConstraint |
| [`G4-mirror-transform`](#g4-mirror-transform) | Not-applicable | cosmetic | S | Not applicable: unused UE function; usdRig mirrors at composition time by referencing the left layer with mirrored rest |
| [`G4-seq-no-constraints`](#g4-seq-no-constraints) | Not-applicable | cosmetic | S | Not applicable: shots have no Sequencer constraint channels, and cross-rig constraint writes are rejected anyway |

### G4-backsolve-analytic

**Backwards Solve: set controls from bones (inverse execution used to bake animation onto the rig)**

**Verdict:** Missing · **Severity:** major · **Effort:** L · **Confidence:** high · **Domain:** D10

UE features: `UE2-fk-backward`, `UE2-body-backward`, `UE3-bone-follow-virtual`, `UE3-foot-matching`

**UE rigs.** Each module handles the RigUnit_InverseExecution event. FK and FkArray evaluate their space nulls, then set control = ControlTransformOffset * bone. Body sets its control = BodyCtrl_init*inv(BodyBone_init)*pelvis_cur and resets Body Offset's local transform to identity. IkFk2Bones sets virtual bones = project(bones), then runs Match FK and Match IK. Foot runs Match FK, Match IK, then sets the FK toes from the toe joints. Sequencer uses this to bake skeletal animation onto controls.

**usdRig today.** There is no inverse graph. rigexec.solve_parameters is a generic Levenberg-Marquardt solver over a caller-supplied forward callback, nothing maps joint poses back to control avars, and export_baked runs the other way (rig to cache).

**Gap.** No per-operator analytic inverse, no bake-to-controls tool, and no evaluation of space nulls during an inverse pass.

**Porting impact.** Retargeted, mocap or baked skeletal animation (Zeb_Face_Expressions, UEFN retargets) cannot be turned into editable control animation.

**Recommendation.** Add a codeless multiple-apply RigExecMatchAPI in libs/rigExecSchema/schema.usda for controls, with per-set properties: rel rigExec:match:<set>:source, matrix4d rigExec:match:<set>:offset, token rigExec:match:<set>:mode = full|rotate|translate|zero, and int rigExec:match:<set>:order. Example sets: backSolve, toFk, toIk. Implement python/rigexec/match.py with bake_to_controls(stage, rig, times, set='backSolve'). For each time it evaluates the sources, computes control avars from offset*source against the control's default and parent chain in dependency order, falls back to solve_parameters where no analytic inverse exists, and writes Ts knots into an edit layer.

**Evidence:** `python/rigexec/inverse.py:44-49`; `docs/python-bake-inverse.md:33-73`; `python/rigexec/bake.py:8-22`; `docs/spec.md:1084-1092`; `README.md:516-517`

**Verification (holds).** Verdict and severity hold. solve_parameters is callback-only (inverse.py:44-60), export_baked goes from rig to cache (bake.py:8-22), and the ExecIr inversion bridge is only an alignment note (spec.md:196-198). No match or bake-to-controls code exists (grep).

Verifier evidence: `python/rigexec/inverse.py:44-60`; `python/rigexec/bake.py:8-22`; `docs/spec.md:196-198`

### G4-backsolve-spine

**Spine backwards solve (FK / IK / Sec FK matching with tangent ray intersection)**

**Verdict:** Missing · **Severity:** major · **Effort:** M · **Confidence:** medium · **Domain:** D10

UE features: `UE2-spine-backward-matching`

**UE rigs.** FK controls = FkDeltaTransform * FK bones, or local identity when FK is hidden. IK Start and End = Offset * bones, and Mid IK takes its rotation from the middle bone. Mid IK's translation is the ray-ray intersection of the end tangents (bones[-2] along -X, bones[1] along +X) when 0 < RatioA < 20, otherwise the middle bone. For Sec FK, the spline is rebuilt and attached with Matching, SetControlOffset is applied from the match nulls, and the Sec FK globals are set. The solve also writes channels (Distribute Rotation=0, IK Vis=true).

**usdRig today.** There is no inverse and RigExecSplineIk has no back-fit helper. solve_parameters could fit the spline controls by least squares.

**Gap.** No analytic spline back-fit and no runtime rewrite of control offsets. UE calls SetControlOffset every frame, whereas usdRig default:* edits are authoring operations.

**Porting impact.** Spine and neck curve controls cannot be baked from skeletal animation, and IK spine matching is lost.

**Recommendation.** In python/rigexec/match.py, add a 'rayIntersectMidpoint' match mode (tangent-ray intersection with the ratio guard) and a solve_parameters fallback that fits the spline control avars to joint landmarks. The tool authors control-offset changes as default:tx..rz edits (libs/rigExecSchema/schema.usda:301-314), never at evaluation time.

**Evidence:** `python/rigexec/inverse.py:44-49`; `docs/python-bake-inverse.md:66-73`; `libs/rigExecSchema/schema.usda:301-314`

**Verification (holds).** Verdict and severity hold. There is no spline back-fit or match code, and default:* edits are authoring-only (schema.usda:301-314).

### G4-ikfk-snap

**IK/FK snap matching (Match FK / Match IK / foot follow-up)**

**Verdict:** Missing · **Severity:** major · **Effort:** M · **Confidence:** high · **Domain:** D10

UE features: `UE4-module-local-matching`, `UE3-ikfk-matching`, `UE3-foot-matching`

**UE rigs.** Match FK sets each FK control = (VB.rot*FK Rotation Offset, VB.t, VB.s), resets the segment-scale nulls and zeroes the gimbals. Match IK sets IK Base = VB0 and IK = VB2*IK Rotation Offset, zeroes the gimbal, applies the IK End Align offset and Auto PV parent, and places PV with Compute Pole Vector Location v02 (PV Distance Scale). Get Distances Between provides lengths. The Foot follows the parent module's 'Match FK/IK' metadata: Toes FK = ball joint; for IK it zeroes the toe, heel, ball and rocker pivots, reruns the pivots and sets Toes IK = ball joint.

**usdRig today.** Nothing exists; the biped lists the ikfk_switch snap as not built. A tool could compute these values from the Rig pose results exposed to Python (control and joint frames) and author avars.

**Gap.** No match data, no pole-vector placement helper, no snap tool, and no foot follow-up ordering.

**Porting impact.** Animators cannot switch a limb between IK and FK without the limb jumping.

**Recommendation.** Add 'toFk' and 'toIk' sets to RigExecMatchAPI (G4-backsolve-analytic) with modes full, zero (gimbals, foot pivots) and poleVector (rel root/mid/end plus a distance scale). Add a rel rigExec:match:<set>:after for ordering (foot after leg). Implement match(stage, rig, set, time) in python/rigexec/match.py.

**Evidence:** `docs/biped-rig.md:190-193`; `python/_rigexec.cpp:1075-1080`; `plugin/rigExecUsdview/pickerModel.py:25-28`

**Verification (holds).** Verdict and severity hold. The picker implements only zero_ctrls (pickerModel.py:28), and the biped lists the ikfk snap as not built (docs/biped-rig.md:190-193). The gizmo's world-delta-to-avar inversion (gizmoMath.py:1778-1817) could be reused by a snap tool.

Verifier evidence: `plugin/rigExecUsdview/pickerModel.py:28`; `docs/biped-rig.md:190-193`

### G4-module-events-match-limb

**Module user events (To IK / To FK / Key Controls) and AAU_Biped 'Match Limb'**

**Verdict:** Missing · **Severity:** major · **Effort:** M · **Confidence:** high · **Domain:** D10

UE features: `UE1-user-module-events`, `UE1-aau-match-limb`

**UE rigs.** IkFk2Bones defines user events 'To FK', 'To IK' and 'Key Controls', and Foot defines 'Key Controls'. ExecuteEventOnModuleByNameForBP runs one module's event, followed by a Forwards Solve. AAU_Biped 'Match Limb' is a Sequencer utility action on a selected limb control. It reads '<Limb>/Ik Fk Switch' at the current frame, runs To FK or To IK plus a forwards solve, keys the switch to the opposite value, selects FK 2 or IK, then runs Key Controls plus a forwards solve.

**usdRig today.** Modules have no events, since evaluation is event-free by design. The picker implements only the 'zero_ctrls' command, and the IK/FK dial is keyed by an external script.

**Gap.** No command system bound to rig data, and no match-key-select workflow.

**Porting impact.** The animators' one-click IK/FK switch in Sequencer is lost.

**Recommendation.** Add a 'match_limb' command to the picker and Avar Editor (plugin/rigExecUsdview/pickerUI.py and a new plugin/rigExecUsdview/matchTool.py). It is driven by a RigExecMatchAPI limb group (rel to the switch attribute, toFk/toIk sets, controls to key, controls to select), calls python/rigexec/match.py, and authors keys through the shared undo stack.

**Evidence:** `docs/spec.md:207`; `plugin/rigExecUsdview/pickerModel.py:25-28`; `plugin/rigExecUsdview/pickerUI.py:814-823`; `docs/biped-rig.md:99-107`

**Verification (holds).** Verdict and severity hold. No module event or command system exists beyond zero_ctrls (pickerModel.py:28, pickerUI.py:814-823), and the IK/FK dial is keyed by an external script (docs/biped-rig.md:99-107).

### G4-available-spaces-switching

**Per-control labeled space list and runtime parent switching (AvailableSpaces / SwitchParent / SetDefaultParent)**

**Verdict:** Partial · **Severity:** major · **Effort:** L · **Confidence:** high · **Domain:** D4

UE features: `UE1-space-switch-nulls`, `UE3-ik-spaces`, `UE4-unit-parent-space-api`, `UE2-root-control-stack`, `UE2-prop-control-stack`

**UE rigs.** Controls have several parents (a default parent plus per-space nulls, controls or bones) and a labeled AvailableSpaces menu. IK gets '<target> IK Null' under each target at the IK parent's bind; PV gets the same set plus the IK control. In Zebra, 22 body controls have spaces: Arm IK has 9 (clavicle, spine_05, pelvis, Local, Body, Global, Prop, Prop Attach 01/02), root/Root has Local plus Spine/Pelvis TXY, Body has Aim, and Prop has hand_r/hand_l/spine_05. SwitchParent/SwitchToParent sets one-hot parent weights and compensates to keep the global pose. SetDefaultParent, SetChannelHosts and SetControlDrivenList manage the lists.

**usdRig today.** This can only be built as RigExecParentConstraints on a space provider above the control: one constraint with inputs:sourceWeights, or one constraint per space with inputs:defaultWeight connected to a custom float dial. When all weights are zero the kernel passes the input through, which gives the namespace default parent. A static reparent via a parent:space connection is structural (a new epoch) and refuses the baked program.

**Gap.** There is no space-switch schema: no labeled list, no active-space index, no default-parent designation, no switch compensation, and no per-channel (location/rotation/scale) parent weights. inputs:sourceWeights is read raw: it cannot be connected or revised by movers, and float[] time samples interpolate linearly, so a stepped switch needs doubled keys. There is no space picker UI, and channel hosting and driven lists are also absent (G3).

**Porting impact.** Each of the 22 spaced Zebra controls and 10 spaced Monster controls needs a hand-built constraint stack and dial. Animators lose the space menu, and switching spaces makes the control jump.

**Recommendation.** Add RigExecSpaceSwitch (concrete, inherits RigExecSourceConstraint, applies MoverAPI) in libs/rigExecSchema/schema.usda with these properties: rigExec:moves = the control's space provider; rigExec:sources = the ordered spaces; uniform token[] rigExec:spaceLabels; int inputs:activeSpace = -1 (int values are held, and -1 means the namespace default); optional float[] inputs:spaceWeights; uniform token rigExec:spaceMode = full|orient|point; and rigExec:maintainOffset. The kernel is a one-hot RigExecApplyParentConstraint in libs/rigExecMath/solvers.cpp, with a handler row in libs/rigExec/rigEvaluator.cpp:249-271 and a baked op in libs/rigExec/bakedProgram.cpp. The labeled dropdown goes in plugin/rigExecUsdview/avarEditorModel.py; picker attribute buttons already cycle through labels.

**Evidence:** `libs/rigExecMath/solvers.cpp:1081-1083`; `libs/rigExec/rigEvaluator.cpp:9392-9427`; `libs/rigExecSchema/schema.usda:1014-1018`; `libs/rigExecSchema/schema.usda:73-78`; `libs/rigExec/moverGraph.h:314-318`; `libs/rigExecSchema/schema.usda:345-357`; `libs/rigExec/bakedProgram.cpp:360-368`; `docs/biped-rig.md:190-193`; `libs/rigExecSchema/schema.usda:2495-2504`

**Verification (holds).** Verdict and severity hold. inputs:sourceWeights is read raw (rigEvaluator.cpp:9392-9427), while inputs:defaultWeight is resolved and connectable (rigEvaluator.cpp:11015-11017). All-zero weights pass the input through (solvers.cpp:1081-1083). A connected parent:space refuses the bake (bakedProgram.cpp:250-252, 360-368). A grep for spaceSwitch, activeSpace and maintainOffset finds no switch or compensation code. The gap overstates one point: a labeled selector already exists. Picker attribute buttons cycle labeled values on any attribute (schema.usda:2495-2504, pickerModel.py:130-150), so a float space dial can get a cycling labeled selector (not a dropdown), with per-space one-hot envelopes built from FloatMathMover chains. There is still no compensation, so the severity stays major. Caveat for the recommendation: a single RigExecSpaceSwitch with all sources depends structurally on every source. Zebra's Prop and Arm IK spaces then form a compile cycle (see missed gap G4-space-dependency-cycles).

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:9392-9427`; `libs/rigExec/rigEvaluator.cpp:11015-11017`; `libs/rigExec/bakedProgram.cpp:250-252`; `libs/rigExecSchema/schema.usda:2495-2504`; `plugin/rigExecUsdview/pickerModel.py:130-150`

### G4-expression-weighted-parents

**Skull/jaw parent constraints with curve-driven per-parent weights (lips, corners, cheeks)**

**Verdict:** Partial · **Severity:** major · **Effort:** L · **Confidence:** high · **Domain:** D9

UE features: `UE6-top-low-lip-constraints`, `UE6-lip-corner-constraints`, `UE6-corner-height-lip-follow`

**UE rigs.** Lip, corner and cheek bones are ParentConstrained (maintain offset, Average, normalized) to [skull, jaw], with per-parent weights computed each frame from curves. For example, lip_tp_01_l gets (0.8, 0.2*clamp(1-smile_l)). Corners get skull 0.5 (times clamp(1-frown) on Monster) and jaw 0.5*clamp(1-smile). For corner height, 6 children per side are constrained to the skull with constraint weight = min(remap(Corner local tz, 0..100), 4.5*jaw_open).

**usdRig today.** The kernel does a normalized 2-source parent blend. However, inputs:sourceWeights is read raw (no connections, no property movers), so dynamic weights require stacking: one constraint to the skull at full weight, then one to the jaw with inputs:defaultWeight = b/(a+b). FloatMathMover chains can compute that, using remap with a connected max to divide. The drivers are the problem. smile, frown and jaw_open, and the control's local tz, must be float attributes available before the pose walk. Control avars are double and cannot connect to float chains, and pose-interpolator outputs are published only after the pose walk.

**Gap.** Per-source weights cannot be connected. FloatMathMover has no min op, although clamp with a connected max works. Avars cannot be read into float chains (double to float), and no scalar readers run in the pose phase (jaw_open).

**Porting impact.** The 22 face lip and corner constraints either lose smile/frown masking or need a large hand-built FloatMathMover network. The corner-height cheek follow cannot read jaw_open during the pose walk.

**Recommendation.** Four changes: (1) Resolve inputs:sourceWeights through RigExecResolvedInputs, and add rel rigExec:sourceWeightInputs (an ordered list of float attributes, one per source) in libs/rigExecSchema/schema.usda, read in libs/rigExec/rigEvaluator.cpp:9403-9427. (2) Allow float inputs to connect to double avars with read-time coercion, in _ValidateScalarConnection (libs/rigExec/rigEvaluator.cpp:560) and the resolver (libs/rigExec/moverGraph.h). (3) Add min, max and oneMinus operations to RigExecFloatMathMover (libs/rigExecMath/propertyMath.cpp). (4) Schedule scalar readers as pose-DAG steps (G6).

**Evidence:** `libs/rigExec/rigEvaluator.cpp:9392-9427`; `libs/rigExec/rigEvaluator.cpp:524-569`; `libs/rigExecSchema/schema.usda:1192-1214`; `libs/rigExec/rigEvaluator.cpp:10236-10238`; `libs/rigExec/rigEvaluator.cpp:2715-2723`; `libs/rigExec/rigEvaluator.cpp:11016-11017`

**Verification (holds).** Verdict and severity hold. sourceWeights is read raw (rigEvaluator.cpp:9403-9427). Scalar connections require an exact type match (rigEvaluator.cpp:560-566), so double avars cannot feed float chains. Pose-interpolator weights are published only after the pose walk (rigEvaluator.cpp:11667-11675), and the interpolator does not evaluate translation, so the Corner control's tz is unreadable. FloatMathMover offers only add, multiply, clamp, remap and blend (schema.usda:1208-1210). UE also applies the fractional constraint Weight with LerpTransform in parent-local space (RigUnit_TransformConstraint.cpp:348-352), which usdRig blends per Euler axis.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:560-566`; `libs/rigExec/rigEvaluator.cpp:11667-11675`; `libs/rigExecSchema/schema.usda:1208-1210`

### G4-modify-transforms

**ModifyTransforms AdditiveLocal: pose-reader-weighted helper-joint offsets**

**Verdict:** Partial · **Severity:** major · **Effort:** L · **Confidence:** medium · **Domain:** D9 · *(analyst said Missing / blocker)* · *reconciled after probes*

UE features: `UE-modifytransforms-op`

**UE rigs.** RigUnit_ModifyTransforms in AdditiveLocal mode computes T = clamp((w-min)/(max-min)), blends offset' = LerpTransform(I, offset, T), and sets the bone's local = offset' * current local, with propagation; it returns early when w <= min. CR_Zebra_Deform has 82 single-item nodes and Monster has 4. Weights come from spherical pose readers or remaps, and offsets on the same def_* helper joint accumulate in execution order.

**usdRig today.** The offset step could be approximated with a RigExecParentConstraint whose only source is the joint itself: per-source offset = the local offset, envelope = weight. This works because target = Offset * Source and sources read the current revision; the blend is per Euler axis. The weight is the blocker. It comes from a pose reader, but RigExecPose outputs are published only after the pose walk and property chains run before it, so no scalar derived from the current pose can drive a pose-domain envelope in the same evaluation.

**Gap.** No additive local-offset operator with weight remapping (min/max) and quaternion-lerp blending. No reader output is available during the pose phase to drive it. Constraints that use their own target as a source are untested.

**Porting impact.** The corrective helper-joint layer of CR_Zebra_Deform and CR_Monster_Deform (86 nodes) cannot be reproduced. Every helper-joint corrective (elbow, knee, shoulder and thigh volume) disappears from skinning.

**Recommendation.** Add RigExecLocalOffsetConstraint (inherits RigExecConstraint, applies MoverAPI) to libs/rigExecSchema/schema.usda with: uniform token rigExec:mode = additiveLocal|additiveGlobal|overrideLocal|overrideGlobal; double3 inputs:translationOffset; quatf inputs:rotationOffset; double3 inputs:scaleOffset (multiplicative); float inputs:weightMin and inputs:weightMax; and optional rel rigExec:offsetSource. The kernel goes in libs/rigExecMath/solvers.cpp (LerpTransform(I,O,T), then O*current), with a handler row in libs/rigExec/rigEvaluator.cpp:249-271 and a baked op in libs/rigExec/bakedProgram.cpp. Schedule the G6 scalar pose readers as pose-DAG steps that publish into _resolvedInputs before dependent envelopes are read (step builder around libs/rigExec/rigEvaluator.cpp:5926-6039, walk around 10894).

**Evidence:** `libs/rigExecMath/solvers.cpp:1047-1055`; `libs/rigExec/rigEvaluator.cpp:10474-10493`; `libs/rigExec/rigEvaluator.cpp:2715-2723`; `libs/rigExec/rigEvaluator.cpp:10236-10238`; `libs/rigExec/rigEvaluator.cpp:11016-11017`; `libs/rigExec/rigEvaluator.cpp:11724-11740`

**Report reconciliation.** Probe C1a (reports/ue-zebrasample/probes/offset-scheduling) shows that a self-sourced RigExecParentConstraint applies a weighted, stacked, local-frame offset, in dynamic and baked modes alike -- the same finding as G6-additive-local-offset. What this row really lacks is a pose-derived weight in the same evaluation: probe C2 shows a constraint weight connected to RigExecPose.outputs:weight silently reads the authored fallback. The verdict is therefore Partial, not Missing.

**Verification (corrected).** The timing gap is confirmed, so Missing holds. Property chains run before the pose walk (rigEvaluator.cpp:10236-10245), and pose-interpolator weights are published after it (rigEvaluator.cpp:11667-11675), so no pose-derived scalar can weight a transform operator in the same evaluation. Blocker overstates the impact, though: MR_Zebra and MR_Monster still evaluate and animate. What is lost is helper-joint volume correction in skinning, which is visible, i.e. major. Pose-reader-driven correction can still reach geometry in the same evaluation through RigExecPoseInterpolator -> BlendInput.inputs:weight (weights are published and asserted before the geometry chains, rigEvaluator.cpp:11724-11740). Baking the helper-joint offsets into corrective shapes is therefore an approximate port path. Baked playback (Zeb_Face_Expressions) carries the def_* tracks directly.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:10236-10245`; `libs/rigExec/rigEvaluator.cpp:11667-11675`; `libs/rigExec/rigEvaluator.cpp:11724-11740`

### G4-seq-space-keys

**Sequencer space-switch step keys (Parent / World / ControlRig) and key compensation**

**Verdict:** Partial · **Severity:** major · **Effort:** M · **Confidence:** high · **Domain:** D4

UE features: `UE8-seq-space-channels`

**UE rigs.** For each spaced control, Sequencer stores a step channel of {Parent | World | ControlRig element} keys. Evaluation calls SwitchToParent before the keyed control values are applied, and those values are read in the active parent's space. zebra_audition uses all three key types; its targets are Arm R/FK 0 Body/spine_05 Orient Space and Neck/End FK Global Orient Space. Sequencer's space tooling also rewrites the control keys so a switch does not move the control.

**usdRig today.** Playback can be replayed with the constraint-stack pattern. The space provider's constraints get time-sampled one-hot weights (or float dials keyed with held Ts knots). A static top-level provider serves as 'World', and all-zero weights select 'Parent'. The control is nested beneath the space provider so its keyed avars are read in the active space. Nothing compensates keys, because every edit is authored by tools and no tool exists for this.

**Gap.** No step-keyed space channel type: float[] weights interpolate linearly between samples. There is no switch-with-compensation or bake-space tool (keep the world pose and re-key avars over a range), and no converter mapping from UE space keys.

**Porting impact.** The two shots with space keys play back only after a converter emits weight keys, doubled to make the steps. In usdview, animators cannot switch space without the control jumping or re-keying by hand.

**Recommendation.** Use int inputs:activeSpace (G4-available-spaces-switching) as the channel. Add python/rigexec/spaces.py with switch_space(stage, control, space, time, compensate=True, frame_range=None). It evaluates Rig control frames (python/_rigexec.cpp controlFrames), computes new avars = world * inv(new space frame) at each sample, authors Ts knots and keys activeSpace. Expose it as a picker command in plugin/rigExecUsdview/pickerUI.py and as an Avar Editor action. The converter maps FMovieSceneControlRigSpaceBaseKey values to activeSpace keys: Parent->-1, World->the World source index, ControlRig->the element's index.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:9392-9427`; `libs/rigExecMath/solvers.cpp:1081-1083`; `libs/rigExec/rigEvaluator.cpp:10661-10751`; `docs/spec.md:1094-1104`; `plugin/rigExecUsdview/pickerModel.py:25-28`; `docs/biped-rig.md:99-107`

**Verification (holds).** Verdict and severity hold. There is no space-channel type and no switch-with-compensation or bake tool: the picker implements only zero_ctrls (pickerModel.py:28). One inaccuracy: stepped playback does not need doubled keys when each space envelope is a scalar float dial keyed with held Ts knots (spec.md:1094-1100) and connected to the resolved inputs:defaultWeight (rigEvaluator.cpp:11015-11017). Doubling is needed only for float[] sourceWeights time samples under the default linear stage profile (spec.md:1100). The major severity rests on the missing compensation workflow, not on playback.

Verifier evidence: `docs/spec.md:1094-1100`; `libs/rigExec/rigEvaluator.cpp:11015-11017`; `plugin/rigExecUsdview/pickerModel.py:28`

### G4-space-dependency-cycles

**Mutually referencing space lists (Prop follows hands, arm IK follows Prop) under a static, totally ordered pose DAG**

**Verdict:** Partial · **Severity:** major · **Effort:** M · **Confidence:** medium · **Domain:** D4 · *added by verifier*

UE features: `UE1-space-switch-nulls`, `UE3-ik-spaces`, `UE2-prop-control-stack`, `UE1-layout-prop`

**UE rigs.** Prop/Prop's spaces are the bones hand_r, hand_l and spine_05. Arm L/R IK and PV spaces include Prop, Prop Attach 01 and Prop Attach 02, through nulls parented under the Prop controls. UE only stores AvailableSpaces and resolves the single active parent lazily, in module execution order. Hand-to-prop and prop-to-hand setups therefore coexist: the direction that runs out of order reads a stale hand transform but never fails.

**usdRig today.** Each space becomes a constraint on a stand-in provider above the control. A constraint depends structurally on every source's owning solver, whatever its weights (rigEvaluator.cpp:5859-5872). Constraints form a total order, each depending on the previous one (rigEvaluator.cpp:5856-5857). Solvers depend on the constraints that target their input ancestry (rigEvaluator.cpp:5926-5944). A source written later is read at its base value (rigEvaluator.cpp:10477-10481).

**Gap.** Suppose the arms are TwoBoneIk solvers. One Prop space constraint sourcing hand_r and hand_l, plus arm IK space constraints sourcing the Prop, then forms a compile cycle, and the whole epoch is rejected. Splitting the Prop constraint per hand compiles only with a specific order. The two hand-offs (prop in the right hand with left IK in Prop space, and the reverse) need contradictory orders, so one direction silently reads the prop's unconstrained frame. Dependencies cannot be gated by weight, and no diagnostic names the space pair that cannot be ordered.

**Porting impact.** A naive conversion with one space-switch constraint per control (the G4-available-spaces-switching proposal) fails to compile MR_Zebra. The converter must split the Prop hand spaces and fix one hand-off direction to match UE module order. In the other direction, the hand follows a prop that has not followed its hand.

**Recommendation.** Lower a multi-source space switch to one pose step per source in the constraint dependency block (libs/rigExec/rigEvaluator.cpp:5851-5944), so that each source contributes its own dependency. Add ordering hints to RigExecSpaceSwitch (rel rigExec:spaceAfter or uniform token[] rigExec:spaceOrder) in libs/rigExecSchema/schema.usda. When a space pair cannot be ordered, emit a compile diagnostic naming it and read that pair at base, a documented fallback, instead of rejecting the epoch. The converter orders the Prop and hand steps to match the UE module order.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:5856-5857`; `libs/rigExec/rigEvaluator.cpp:5859-5872`; `libs/rigExec/rigEvaluator.cpp:5926-5944`; `libs/rigExec/rigEvaluator.cpp:10477-10481`; `ue/<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1889 (Prop spaces hand_r/hand_l/spine_05)`; `ue/<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1901,1903 (Arm L/R Prop IK Null under Prop/Prop)`

### G4-backsolve-root-motion

**Root backwards solve with selectable root-motion target ('Bake Root On')**

**Verdict:** Missing · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D10

UE features: `UE2-root-bake-root-matching`

**UE rigs.** On the first frame the root backwards solve snaps Global to the root bone; the Snapped flag is reset by forwards solves. The int enum 'Bake Root On' then chooses where root motion goes. On Root: Local = identity and Global = the stored transform. On Global: Global, Local and Root all equal the root. On Local: Local = Root = the root, and Global = the stored transform.

**usdRig today.** This would be a mode of the bake-to-controls tool. Schema avars are double plus the rotationOrder token, and there is no enum channel type.

**Gap.** No root-motion target selection and no reference-frame (first-frame snap) semantics.

**Porting impact.** Root motion cannot be baked onto the Global, Local and Root controls.

**Recommendation.** Add a variant token on the backSolve match set, evaluated by python/rigexec/match.py. Replace the hidden first-frame state with an explicit reference_time argument.

**Evidence:** `python/rigexec/inverse.py:44-49`; `libs/rigExecSchema/schema.usda:327-336`; `docs/biped-rig.md:48-53`

**Verification (holds).** Verdict and severity hold. The 'Bake Root On' channel itself is expressible: custom avars may be int or token, and picker attributeLabels give enum labels (schema.usda:2499-2504). Only the root-motion bake mode is missing.

Verifier evidence: `libs/rigExecSchema/schema.usda:2499-2504`

### G4-maintain-offset

**Maintain-offset computed from the initial / bind pose**

**Verdict:** Missing · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D9

UE features: `UE2-body-drive-pelvis`, `UE6-control-drives-bone`, `UE6-head-attach-null`, `UE6-lid-skin-constraints`, `UE4-evaluate-space-nulls`, `UE4-unit-position-constraint`, `UE6-eye-aim`

**UE rigs.** The following compute their offsets from INITIAL globals on every evaluation: bMaintainOffset on ParentConstraint, PositionConstraintLocalSpaceOffset and AimConstraintLocalSpaceOffset, and bChildInitial/bOldParentInitial on ProjectTransformToNewParent. For Parent the offset is the full transform including scale; for the LocalSpaceOffset variants it is a local-space position or rotation. Rest or construction edits therefore never require re-authoring.

**usdRig today.** Every offset is an authored number: translationOffset, Euler rotationOffset, and the per-source arrays. The builder only writes them.

**Gap.** The evaluator computes no offsets. Offsets have no scale component and no local-space option, and they go stale when rest:space (live, editable) changes.

**Porting impact.** The converter must bake roughly 200 offsets (116 lid evaluations, lips, bone drives, spaces). Pivot or rest edits in usdview then silently break them.

**Recommendation.** Add uniform token rigExec:maintainOffset = none|rest|default|delta to RigExecSourceConstraint (libs/rigExecSchema/schema.usda:999-1019). With rest or default, offset_i = F(target) * inv(F(source_i)) is computed each evaluation from computeRestFrame or computeDefaultFrame; restFrames is already available in the pose walk (libs/rigExec/rigEvaluator.cpp:11160-11170). With delta, the incoming frame is used instead (see G4-new-parent-delta-follow). Compose the result with any authored offsets in buildSources (libs/rigExec/rigEvaluator.cpp:10507-10547) and in the baked op, and add inputs:scaleOffsets[].

**Evidence:** `libs/rigExecRigging/rigBuilder.h:392-396`; `libs/rigExec/rigEvaluator.cpp:10507-10527`; `libs/rigExecSchema/schema.usda:980-991`; `libs/rigExecSchema/schema.usda:875`

**Verification (holds).** Verdict and severity hold. A repo-wide grep finds maintainOffset only in prose (schema.usda:875, splineIk.h:59), and the builder only writes offset arrays (rigBuilder.h:392-396). UE's ParentConstraint maintain-offset uses initial globals including scale (RigUnit_TransformConstraint.cpp:252-261). Baked offsets reproduce the results, so minor is right.

Verifier evidence: `libs/rigExecSchema/schema.usda:875`; `libs/rigExecRigging/rigBuilder.h:392-396`; `UE_5.8/.../RigUnit_TransformConstraint.cpp:252-261`

### G4-anim-baked-import

**Baked expression animation (bone tracks + corrective/face/deformer/material curves)**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D10

UE features: `UE-anim-baked-curves`

**UE rigs.** Zeb_Face_Expressions is a 163-frame bake with 371 bone tracks (including def_* helpers and twist bones) and 102 float curves: 39 body corrective, 54 face morph, 6 deformer-driver and 3 material. At playback the post-process rig overwrites the corrective curves and applies its AdditiveLocal helper offsets again on top of the baked helper tracks.

**usdRig today.** Bone tracks can be authored as joint avars (Ts splines) or as time-sampled posed:space matrices; the baked program supports animated authored posed:space. Curves become custom float attributes connected to BlendInput weights or dials. A variant or inputs:enabled switches off the live solvers for baked playback.

**Gap.** No ingest of UsdSkelAnimation, FBX or UE animation (a spec non-goal). There is no quaternion channel, so tracks need Euler conversion and filtering. Apart from constraints on posed joints, there is no way to layer post-process logic on top of baked tracks.

**Porting impact.** The expression demo needs an external converter. UE applies helper offsets twice here, and a port must decide whether to replicate or drop that.

**Recommendation.** Build this as tooling outside the evaluator: python/rigexec/import_anim.py converts UsdSkelAnimation (from UE's USD export) into joint avars splines (quaternion to Euler in avars:rotationOrder, with an Euler filter) and curves into float attributes. UsdSkel is never bound at evaluation, so the non-goal stays intact.

**Evidence:** `libs/rigExec/computations.cpp:345-354`; `libs/rigExec/bakedProgram.cpp:339-355`; `libs/rigExecSchema/schema.usda:327-344`; `docs/spec.md:56-63`; `python/rigexec/bake.py:8-22`

**Verification (holds).** Verdict and severity hold. The skeleton-binding bridge non-goal is at spec.md:61, and an animated authored posed:space bakes (bakedProgram.cpp:339-355). Note that matrix4d time samples interpolate component-wise, so per-frame posed:space bakes are exact only on sampled frames.

Verifier evidence: `docs/spec.md:61`; `libs/rigExec/bakedProgram.cpp:339-355`

### G4-compute-fk-aim-chain

**Compute FK: aim-chain rebuild with optional distance stretch**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D5

UE features: `UE3-compute-fk`

**UE rigs.** Compute FK rebuilds each FK bone so its X axis aims at the next joint while keeping twist from its own Z axis (y = cross(Zr,x), z = cross(x,y)). With Use Scale, X is also scaled by |d|/L. FK mode feeds it the FK driver globals, and it is reused after the IK solve for the Mid control.

**usdRig today.** The aim can be approximated per joint with RigExecAimConstraint toward the next control: aimVector +X, upVector +Z, worldUpType objectRotationUp with worldUpObject = the joint itself and worldUpVector +Z. No operator does the length-ratio stretch.

**Gap.** No aim-chain or stretch-to-target operator. A world-up object that is the constrained joint itself is untested, and no operator scales by distance.

**Porting impact.** Translating an FK control does not re-aim its parent bone. Zebra's default is Use Scale=false and FK translation is uncommon, so the difference shows only when animators translate FK controls.

**Recommendation.** Either add uniform token rigExec:stretchMode = none|scaleAim with double[] inputs:restLengths to RigExecAimConstraint (libs/rigExecSchema/schema.usda, libs/rigExecMath/solvers.cpp), or add a small RigExecAimChain solver in libs/rigExec/computations.cpp that publishes computePointFrameArray over rigExec:controls and rigExec:joints, with a baked op.

**Evidence:** `libs/rigExecSchema/schema.usda:1021-1071`; `libs/rigExecMath/solvers.cpp:1111-1218`; `libs/rigExec/rigEvaluator.cpp:11353-11385`

**Verification (holds).** Verdict and severity hold. objectRotationUp transforms worldUpVector by the up object's current rotation (rigEvaluator.cpp:11353-11384). With the constrained joint as its own up object, the desired up is its current Z projected off the aim (solvers.cpp:1174-1193), which equals UE's y=cross(Zr,x), z=cross(x,y). There is no length-ratio stretch operator. IkFk2Bones' Use Scale defaults to false.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:11353-11384`; `libs/rigExecMath/solvers.cpp:1174-1193`

### G4-envelope-blend-space

**Constraint weight and filter semantics: parent-local LerpTransform and Euler filters vs asset-space per-Euler-axis envelope**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D9 · *added by verifier*

UE features: `UE6-corner-height-lip-follow`, `UE6-new-parent-follow`, `UE2-prop-aim-solve`, `UE2-body-aim`, `UE6-head-attach-null`

**UE rigs.** RigUnit_ParentConstraint converts the mixed result into the child's parent-local space. It filters translation, rotation (Euler in RotationOrderForFilter) and scale there, then applies a fractional Weight with LerpTransform against the child's current local transform: translation and scale lerp, rotation slerps. SetTransform with Weight<1 (Lips Bt at 0.5) and the Aim Weight blends use the same LerpTransform.

**usdRig today.** inputs:defaultWeight envelopes and inputs:affect* masks on RigExecConstraint. The envelope moves each asset-space Euler component toward the target by its shortest delta, and masks address asset axes.

**Gap.** No slerp envelope, no parent-local mask space, and no filter rotation order separate from the operator's rotation order.

**Porting impact.** Results are identical at weights 0 and 1 and for unfiltered constraints; every Zebra/Monster ParentConstraint is all-channel. Results differ for fractional weights: Corner Height's min(...) weight, the Lips Bt 0.5 follow, intermediate Aim Weight, and partial space weights. The difference is a small orientation drift under large rotations.

**Recommendation.** Add uniform token rigExec:envelopeBlend = eulerAxis|slerp and uniform token rigExec:maskSpace = asset|parent to RigExecConstraint (libs/rigExecSchema/schema.usda:930-997). Implement both in RigExecApplyParentConstraint, RigExecApplyRotationConstraint and RigExecApplyAimConstraint (libs/rigExecMath/solvers.cpp:873-1218), using the target's namespace-parent frame from the pose walk, and add matching baked ops in libs/rigExec/bakedProgram.cpp.

**Evidence:** `libs/rigExecMath/solvers.cpp:779-800`; `libs/rigExecMath/solvers.cpp:1093-1107`; `libs/rigExecSchema/schema.usda:962-978`; `libs/rigExec/rigEvaluator.cpp:11232-11235`; `UE_5.8/.../RigUnit_TransformConstraint.cpp:298-360`

### G4-eye-aim

**Eye aim with local-space maintained offset and world-up in a space**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D9

UE features: `UE6-eye-aim`

**UE rigs.** The Eye L/R nulls aim +Y (up +Z) at 'Eye L/R aim' nulls, which sit 50 units ahead as children of the Eye Aim control, using AimConstraintLocalSpaceOffset with maintained local rotation offset. World-up is the direction (0,0,1) in Eye L aim space on Zebra, and the location eye+(0,0,10) on Monster. The Eye Aim control sits at the center of the box around both targets, under an inserted 'Eye Aim Null' parented to the skull, and has a Convergence channel.

**usdRig today.** RigExecAimConstraint with aimVector (0,1,0) and upVector (0,0,1). Zebra: worldUpType objectRotationUp, worldUpObject = the aim null, worldUpVector (0,0,1). Monster: worldUpType vector (0,0,1). The Eye Aim hierarchy becomes nested controls under the skull joint.

**Gap.** rotationOffset is an Euler addition in asset space, not UE's local-space maintained quaternion offset. Workaround: constrain an intermediate provider, and nest the eye null under it with the offset as its rest. Monster's location-based world-up equals a constant world direction only because the target is relative to the child.

**Porting impact.** Results match when the eye nulls point along +Y at rest; otherwise the nested-offset workaround is needed. Convergence logic is covered in G6.

**Recommendation.** Support rigExec:maintainOffset on Aim, computing a local rotation offset = inv(aimedInit_local)*childInit_local as RigUnit_AimBone.cpp does. Alternatively, add uniform token rigExec:offsetSpace = asset|local to RigExecConstraint (libs/rigExecSchema/schema.usda:980-996), applied in libs/rigExecMath/solvers.cpp:1205-1216.

**Evidence:** `libs/rigExecSchema/schema.usda:1021-1071`; `libs/rigExec/rigEvaluator.cpp:11325-11385`; `libs/rigExecMath/solvers.cpp:1111-1218`

**Verification (holds).** Verdict and severity hold. The objectRotationUp and vector world-up modes work as mapped (rigEvaluator.cpp:11325-11385). rotationOffset is added to the target Euler in asset space (solvers.cpp:1205-1209). Nesting the eye null under an unconstrained aim provider reproduces UE's local maintained offset exactly.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:11325-11385`; `libs/rigExecMath/solvers.cpp:1205-1209`

### G4-fk-chain-construction

**FK chain construction (null + control per bone, control transform offset, gimbal / segment-scale nesting)**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D5

UE features: `UE2-fkchain-construction`, `UE4-create-fk-chain-controls`, `UE1-layout-clavicle`, `UE1-layout-fingers`, `UE3-ikfk-fk-chain`

**UE rigs.** For each bone from Start to End: a null under the running parent at ControlTransformOffset*bone_init, a control under the null with identity offset, and the next null nested under that control. Each control gets mirror metadata, a display name and a rotation order. Zebra fingers are FK 0/1/2 (Base/Mid/Tip) under the hand bone with offset quat (-1,0,0,0), i.e. 180 degrees about X. The clavicle is a single 'FK' control under spine_05 with XZY order. IkFk2Bones FK nests Default FK Space > FK 0 > FK 0 Gimbal > Upper Segment Scale FK > FK 1 > Lower Segment Scale FK > FK 2 > FK 2 Gimbal, with control offset rotation VB.rot*FK Rotation Offset and a shape-scale profile.

**usdRig today.** Use nested RigExecControls whose rest:space = offset*bone_rest; a plain intervening Xform can stand in for the null. When control and joint rests coincide, drive the joints with RigExecFkChain (parentRelative, rigExec:startFrame = parent joint). When they differ, use one RigExecParentConstraint per bone with a baked rotation offset, as the biped does.

**Gap.** FkChain has no per-element control-to-joint offset (UE's Control Transform Offset / FK Rotation Offset), because each joint receives its control's frame directly. Rotated controls (Zebra fingers, limb FK) therefore need ParentConstraint stacks. FkChain is strictly linear, and intervening plain-Xform nulls cost the baked path.

**Porting impact.** 16 Zebra and 4 Monster FkChain modules, plus limb FK, port as ParentConstraint stacks instead of one solver each. That means more movers and more ordering pitfalls (docs/biped-rig.md:200-223). Keyed control values stay valid only if the UE control orientation is preserved.

**Recommendation.** Add matrix4d[] inputs:controlOffsets to RigExecFkChain in libs/rigExecSchema/schema.usda, parallel to rigExec:controls, with joint_i = inverse(offset_i) * W_i. Apply it in _ComputeFkChain (libs/rigExec/computations.cpp:614-697) and in the baked FkChain op (libs/rigExec/bakedProgram.cpp). The converter should emit nulls as control rest:space, not as intervening Xforms.

**Evidence:** `libs/rigExecSchema/schema.usda:418-522`; `libs/rigExec/computations.cpp:614-697`; `libs/rigExec/frameExtraction.h:67-123`; `libs/rigExec/rigEvaluator.cpp:9023-9045`; `libs/rigExec/bakedProgram.cpp:258-281`; `libs/rigExecSchema/schema.usda:1135-1140`

**Verification (holds).** Verdict and severity hold. Joint i receives the element's out-space, which is the control's posed frame (frameExtraction.h:61-121), and the control rests are published as the element rests (computations.cpp:676-713). So there is no per-element control-to-joint offset. Correction: ParentConstraint stacks are not the only route. Under each rotated control, a stand-in child provider with rest = inverse(ControlTransformOffset), i.e. aligned to the joint, can be listed in rigExec:controls with controlSpace=parentRelative. One FkChain then still drives rotated-control chains.

Verifier evidence: `libs/rigExec/frameExtraction.h:61-121`; `libs/rigExec/computations.cpp:676-713`; `libs/rigExecSchema/schema.usda:445-467`

### G4-fk-drive-bones

**FK drive bones with offset compensation and 'Use Active Skeleton' additive mode**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D5

UE features: `UE2-fk-drive-bones`

**UE rigs.** 'Drive Bones' toggles the bone writes, which set bone = inverse(ControlTransformOffset) * control global. With 'Use Active Skeleton', the incoming animated bone global is copied into the null, and the control's local value is applied as an additive offset on top of the animation without child propagation.

**usdRig today.** 'Drive Bones' maps to inputs:enabled on the driving constraints, which is a value edit. The absolute drive is a ParentConstraint with a baked offset, or FkChain. The additive mode has no equivalent: nothing applies a control's local delta on top of a joint's incoming (avars or posed:space) frame, because per-source offsets are static arrays.

**Gap.** No operator applies a control's delta additively on the incoming pose, and constraint writes always propagate to descendants.

**Porting impact.** The default config (Use Active Skeleton=false) ports. The layer-on-top-of-animation mode is lost.

**Recommendation.** Give RigExecLocalOffsetConstraint (G4-modify-transforms) a rel rigExec:offsetSource whose rest-to-pose local delta supplies the offset. Add uniform bool rigExec:propagate = true on RigExecConstraint, honored by commitConstraintFrames (libs/rigExec/rigEvaluator.cpp:10661-10751).

**Evidence:** `libs/rigExecSchema/schema.usda:69-72`; `libs/rigExec/rigEvaluator.cpp:10990-10997`; `libs/rigExec/rigEvaluator.cpp:10507-10547`; `libs/rigExec/rigEvaluator.cpp:10661-10751`

**Verification (holds).** Verdict and severity hold. inputs:enabled is resolved per frame (rigEvaluator.cpp:10990-10997), and propagation stops only at descendants that own their pose (rigEvaluator.cpp:10698-10704, 10629-10632). The additive mode has no operator, but a nested 'incoming animation' control with a child offset control that drives the joint approximates it. MR_Zebra has no Use Active Skeleton override, so the default (false) applies.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:10990-10997`; `libs/rigExec/rigEvaluator.cpp:10698-10704`

### G4-gimbal-controls

**Gimbal child controls (rotator, own rotation order, vis channel hosted on the source)**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D5

UE features: `UE4-gimbal-control`, `UE3-gimbal-control`, `UE1-gimbal-controls`

**UE rigs.** 'Create Gimbal Control' spawns a ROTATOR child '<Src> Gimbal' with identity offset and the preferred rotation order. Its shape is the source's scaled by 0.8 with color +0.2, unless an override is given. It writes metadata 'Gimbal Control' on the source and spawns a bool 'Gimbal Control Vis' hosted on both controls; visibility = channel AND IK/FK mode. IkFk2Bones gives FK 0, FK 2 and IK a gimbal, and downstream nulls hang under the gimbal. Zebra has 14 rotators.

**usdRig today.** Model the gimbal as a nested child RigExecControl with its own avars:rotationOrder. With FkChain rigExec:controlSpace=parentRelative the gimbal is listed as the chain driver, because its computePointFrame already includes the parent control. guide:scale and guide:displayColor express the smaller, lighter shape. Visibility can only be faded, through guide:displayOpacity connected to a float or double.

**Gap.** No rotation-only control type, so translate and scale avars stay editable. No bool visibility channel and no channel hosting: a bool cannot drive guide opacity, and nothing truly hides a control. No control-to-gimbal link that match and zero tools could read.

**Porting impact.** The gimbals themselves work. However, animators can translate them, which alters the chain, and the 'Gimbal' toggle shown on the main control cannot be reproduced.

**Recommendation.** Add a rigExec:controlType (transform|rotator|translator) or rigExec:lockedChannels token[] to RigExecControlAPI in libs/rigExecSchema/schema.usda, honored by plugin/rigExecUsdview/avarEditorModel.py and gizmoSettings.py. Add a connectable bool guide visibility in libs/rigExecImaging/bridge.cpp, and a rel rigExec:gimbal on RigExecControlAPI for match and zero tools. Channel types themselves belong to G3.

**Evidence:** `libs/rigExecSchema/schema.usda:334-336`; `libs/rigExec/computations.cpp:192-222`; `libs/rigExecSchema/schema.usda:445-467`; `libs/rigExecSchema/schema.usda:201-230`; `libs/rigExecSchema/schema.usda:33-47`

**Verification (holds).** Verdict and severity hold, but one claim is wrong: controls can be truly hidden. UsdGeomImageable visibility on a control is forwarded to its synthesized guide (sceneIndices.cpp:483-520), and visibility is an animatable token. What is actually missing is driving visibility from the bool channel AND the IK/FK mode. guide:displayOpacity only follows float or double connections (bridge.cpp:219-232, 259-292), and visibility is not rig-evaluated. No channel-lock or rotator concept exists in the Avar Editor or the gizmo (a grep for lock or keyable finds only gizmo axis dimming).

Verifier evidence: `libs/rigExecImaging/sceneIndices.cpp:483-520`; `libs/rigExecImaging/bridge.cpp:219-232`; `libs/rigExecImaging/bridge.cpp:259-292`; `plugin/rigExecUsdview/avarEditorModel.py:107`

### G4-gizmo-constraint-lock

**Viewport gizmo locks every ParentConstraint target, even while the constraint is disabled or at zero weight**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D2 · *added by verifier*

UE features: `UE1-space-switch-nulls`, `UE3-ikfk-matching`, `UE8-seq-space-channels`

**UE rigs.** UE controls stay manipulable whatever their active parent, parent weights or rig-side overwrites (Match FK/IK, space nulls). Space switching never locks a control in the viewport.

**usdRig today.** SolverPosedPaths treats every rigExec:moves target of a RigExecParentConstraint as avar-inert, whatever its inputs:enabled or envelope. Targets of Aim, Position and Rotation constraints stay editable.

**Gap.** The lock is structural, not value-aware. A follow or space ParentConstraint placed directly on a control makes that control undraggable in every mode.

**Porting impact.** The converter must put every space or follow ParentConstraint on a stand-in provider above the control, and use Position+Rotation pairs for control-level follows such as IK/FK auto-follow. Otherwise animators cannot drag those controls even when the follow is off.

**Recommendation.** Make SolverPosedPaths in plugin/rigExecUsdview/gizmoMath.py time-aware. It would consult the evaluator's per-evaluation constrained set (constrainedProviders, libs/rigExec/rigEvaluator.cpp:10656) or the resolved inputs:enabled/defaultWeight at the current frame, and treat a dormant ParentConstraint target as editable.

**Evidence:** `plugin/rigExecUsdview/gizmoMath.py:426-470`; `docs/viewport-gizmos.md:353-373`; `libs/rigExec/rigEvaluator.cpp:10656`

### G4-ikfk-auto-matching

**Continuous IK/FK auto-matching (inactive control set follows the limb every frame)**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D10 · *(analyst said Missing / major)*

UE features: `UE3-ikfk-matching`

**UE rigs.** With 'IK FK Auto Matching' on (the default), Match FK runs on every frame in IK mode and Match IK on every frame in FK mode, branching on the bool switch. The hidden control set therefore always sits on the current limb. Post Forwards Solve resets the Match metadata.

**usdRig today.** One direction could be built: ParentConstrain the FK controls to IK-only joints with envelope = switch. Doing both directions creates a structural pose-dependency cycle (FkChain -> FK controls -> IK joints -> IK controls -> FK joints). Solvers depend on every constraint targeting their inputs, and compile rejects the cycle whatever the weights are.

**Gap.** Dependencies cannot be gated by values (branching), and no display-only follow exists for inactive controls.

**Porting impact.** Inactive IK or FK controls stay where they were last keyed instead of tracking the limb. Without a snap tool (G4-ikfk-snap), switching makes the limb jump.

**Recommendation.** Keep evaluation acyclic. Add imaging-only rel guide:followFrame and a connectable float guide:followWeight to RigExecControl (libs/rigExecSchema/schema.usda:144-250, libs/rigExecImaging/bridge.cpp). The guide is then drawn at another provider's solved frame times the stored match offset, so inactive controls appear on the limb. The snap tool (G4-ikfk-snap) commits values when the switch is keyed.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:5926-5944`; `libs/rigExec/rigEvaluator.cpp:6092`; `libs/rigExec/rigEvaluator.cpp:11583-11596`

**Verification (corrected).** The claimed unavoidable cycle depends on the wiring chosen. Compile dependencies:
- Each constraint depends on the previous constraint and on the solvers that own its target and source closures (rigEvaluator.cpp:5851-5872).
- Each solver depends on the constraints that target its input ancestry (rigEvaluator.cpp:5926-5944).
An acyclic wiring:
- (B) Position+Rotation constraints on IK Base, IK and PV that source the FK controls (PV sources a locator nested under FK 1), with envelope 1-switch.
- (A), ordered after B: Position+Rotation constraints on the FK controls and gimbals that source IK shadow joints, with envelope = switch. A consumed TwoBoneIk still poses joints the blend does not claim (rigEvaluator.cpp:4299-4309, 4622-4631).
The dependency chain is B -> IK solver -> A -> FK solver -> blend. B reads the FK controls' base frames (implicit preceding, rigEvaluator.cpp:10477-10481), and FkChain passes control frames to the FK joints unchanged (frameExtraction.h:61-121).
Conditions:
- The IK root control must not be an FK control; example 03's ShoulderFK rootControl would recreate the cycle.
- B must precede every consumer of the limb's blended joints.
- Use Position/Rotation rather than Parent constraints, so the gizmo keeps the controls draggable (gizmoMath.py:426-470).
- The PV is a nested locator, not UE's Compute Pole Vector Location.
Inactive controls then track the limb. The jump at switch time is the separate snap/autokey gap. Not compiled; confidence medium.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:5851-5872`; `libs/rigExec/rigEvaluator.cpp:5926-5944`; `libs/rigExec/rigEvaluator.cpp:4622-4631`; `libs/rigExec/rigEvaluator.cpp:10477-10481`; `examples/03_IkFkBlendClamp.usda:110-128`; `plugin/rigExecUsdview/gizmoMath.py:426-470`

### G4-lid-multi-parent

**5-parent weighted lid skin constraints (static weight tables, quaternion averaging)**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D9

UE features: `UE6-lid-skin-constraints`

**UE rigs.** Eight loops, about 116 evaluations per frame. Each lid skin bone is ParentConstrained to [Lid In, Lid Ot, micro 01-03] with maintained offset and all channels including scale. Weights are static per-row values from Lid_Tp/Bt_Struct. Average interpolation sums the weighted rotations as quaternions with shortest-arc alignment and normalizes the result. Zebra has authoring quirks: Lid In R is listed twice, and the R bt-base constraint uses the L parents.

**usdRig today.** One RigExecParentConstraint per bone with 5 sources, static inputs:sourceWeights, baked per-source offsets and affectScale on. Translation and scale averaging match UE. Rotation is averaged per Euler component around the first nonzero-weight source, not by quaternion.

**Gap.** Euler-component rotation averaging depends on rotation order and diverges for widely spread sources, unlike UE's normalized quaternion averaging. Per-source offsets have no scale, and offsets are baked.

**Porting impact.** Lid skin bone orientations drift from UE when micro controls rotate far apart (blinks), which slightly changes lid skinning. Translations match.

**Recommendation.** Add uniform token rigExec:rotationBlend = eulerAnchored|quaternionAverage|quaternionShortest to RigExecSourceConstraint (libs/rigExecSchema/schema.usda:999-1019). Implement the hemisphere-aligned weighted quaternion sum (UE's AccumulateWithShortestRotation) and a slerp envelope in RigExecApplyParentConstraint and RotationConstraint (libs/rigExecMath/solvers.cpp:873-1109), plus the baked ops. Reproduce the Zebra source-list quirks exactly as authored.

**Evidence:** `libs/rigExecMath/solvers.cpp:997-1109`; `libs/rigExecSchema/schema.usda:1014-1018`; `libs/rigExecSchema/schema.usda:1132-1140`

**Verification (holds).** Verdict and severity hold. UE's Average interpolation uses AccumulateWithShortestRotation and then normalizes (RigUnit_TransformConstraint.cpp:264-269, 297). usdRig anchors the Euler deltas on the first nonzero source (solvers.cpp:1071-1087). Blink rotations are mostly single-axis, so the results stay close.

Verifier evidence: `libs/rigExecMath/solvers.cpp:1071-1087`; `UE_5.8/.../RigUnit_TransformConstraint.cpp:264-269`

### G4-new-parent-delta-follow

**'New Parent' delta follow: items ride a control's current local delta**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D9 · *(analyst said Partial / major)*

UE features: `UE6-new-parent-follow`

**UE rigs.** New Parent(Control, Children) takes ParentFrame = inv(ctrl local) * ctrl global, i.e. the control's current offset frame. Each child becomes childGlobal_current * inv(ParentFrame) * ctrlGlobal, so it rides the control's delta from wherever it currently is. The calls chain Muzzle -> Mouth -> Lips Tp -> Lips Bt over lip, nose, teeth and const items. Lips Bt applies weight 0.5 to lip_bt_02_l/r and Jaw Const.

**usdRig today.** When the controls are nested (e.g. Mouth under Muzzle), one ParentConstraint to the innermost control with a bind offset reproduces the chained result for items that every level touches. Items at different levels, and the weight-0.5 items, need separate constraints with envelopes, which blend per Euler axis rather than as UE's LerpTransform. usdRig has no operator that applies a provider's rest-to-pose delta to a transform's current frame; MatrixMover does this for points only.

**Gap.** No delta-follow constraint relative to the current frame. The envelope blends Euler angles per axis instead of slerping.

**Porting impact.** The group moves of Muzzle, Mouth and Lips (about 30 lip/nose/teeth/const items per face) must be re-derived as bind-offset constraints. Results diverge wherever earlier logic already moved an item.

**Recommendation.** Implement rigExec:maintainOffset='delta' on RigExecSourceConstraint: target = input * inv(computeDefaultFrame(source)) * computePointFrame(source), with the default frame already published by providers (libs/rigExec/computations.cpp:470-475). Resolve it in buildSources (libs/rigExec/rigEvaluator.cpp:10507-10547) and the baked op, with per-item weights via inputs:defaultWeight and a slerp envelope option (G4-lid-multi-parent).

**Evidence:** `libs/rigExecSchema/schema.usda:1663-1679`; `libs/rigExecMath/solvers.cpp:1047-1055`; `libs/rigExecMath/solvers.cpp:779-800`; `libs/rigExec/rigEvaluator.cpp:10661-10751`; `libs/rigExec/computations.cpp:470-475`

**Verification (corrected).** The mapping overlooks that namespace propagation already rides a provider's before->after delta from each item's current frame. commitConstraintFrames applies delta = inv(P_before)*P_after to the CURRENT frame of every non-owning descendant (rigEvaluator.cpp:10718-10737). That includes joints already written absolutely by earlier constraints, because they are not pose owners (rigEvaluator.cpp:10629-10632). The workaround:
- Nest the New Parent items (lip joints, Corner nulls, Skull/Jaw Const) under stand-in carrier providers whose rest is the driving control's offset frame.
- Parent-constrain each carrier to Muzzle, Mouth, Lips Tp or Lips Bt after the skull/jaw lip constraints. Items then get childCurrent*inv(offsetCur)*ctrlCur, exactly as UE.
- Overlapping sets (upper lips M/O/T, lower lips O/B, Jaw M) need one carrier per set combination. A carrier not nested under Muzzle needs a pre-snap to Mouth's offset frame before the absolute lip writes.
- Only the weight-0.5 items need an extra enveloped constraint (Euler blend instead of LerpTransform).
The cost is a joint namespace organised by item set instead of by skeleton, plus fragile ordering (docs/biped-rig.md:221). Because a workaround exists, minor rather than major.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:10718-10737`; `libs/rigExec/rigEvaluator.cpp:10629-10632`; `docs/biped-rig.md:221`

### G4-position-local-offset

**Position constraint with local-space maintained offset (LimbTwist Blend Translate)**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D9

UE features: `UE4-unit-position-constraint`, `UE3-limbtwist-translate`

**UE rigs.** PositionConstraintLocalSpaceOffset sets the child's position to the weighted parent positions plus a maintained offset expressed in the child's parent-local space (computed from initial poses), filtered per axis in local space. LimbTwist 'Blend Translate' constrains twist bones and nulls to [Start w_i, End 1-w_i], where w_i is the construction-time distance ratio. It runs in forward and backward solves when 'Use Translates' is on.

**usdRig today.** RigExecPositionConstraint with sources [start, end] and static inputs:sourceWeights [w, 1-w] reproduces the blend. inputs:translationOffset is additive in asset space, and masks use asset axes.

**Gap.** No offsets or masks in local (parent-frame) space, and no maintain-offset.

**Porting impact.** Twist bones on the limb axis have near-zero offset, so results match. Any off-axis residual offset does not rotate with the limb.

**Recommendation.** Add uniform token rigExec:offsetSpace = asset|parentLocal on RigExecConstraint (libs/rigExecSchema/schema.usda:980-996), applied in RigExecApplyPositionConstraint (libs/rigExecMath/solvers.cpp:804-871) with the target's final namespace-parent frame. Also support rigExec:maintainOffset.

**Evidence:** `libs/rigExecMath/solvers.cpp:804-871`; `libs/rigExecSchema/schema.usda:1073-1085`; `libs/rigExecSchema/schema.usda:980-982`; `libs/rigExec/rigEvaluator.cpp:377-386`

**Verification (holds).** Verdict and severity hold. The UE source computes the offset from initial transforms and applies it in parent-local space (RigUnit_TransformConstraint.cpp:829-869). usdRig adds translationOffset in asset space (solvers.cpp:839) and masks asset axes.

Verifier evidence: `UE_5.8/.../RigUnit_TransformConstraint.cpp:829-869`; `libs/rigExecMath/solvers.cpp:839`

### G4-rigvm-math

**RigVM transform / vector / quaternion math used at construction and during solves**

**Verdict:** Partial · **Severity:** minor · **Effort:** L · **Confidence:** high · **Domain:** D9

UE features: `UE4-rigvm-math`

**UE rigs.** The libraries use MakeRelative/MakeAbsolute, transform Mul/Lerp/TransformVector, IntersectPlane, RayIntersectRay (spine), VectorSetLength, QuaternionSwingTwist, slerp, inverse, axis-angle and Euler conversions, MatrixFromVectors, and vector and float arithmetic. They are used both at construction and in forward and backward solves.

**usdRig today.** Construction math belongs in the converter or builder (Python/C++). At evaluation time, math exists only inside fixed operators (constraints, solvers, internal swing-twist) and in property movers (add/multiply/clamp/remap/blend on float, float3 and matrix). Property movers run before the pose walk and cannot read pose results.

**Gap.** No math that depends on the current pose (distances, dot products, swing-twist outputs, ray intersections, matrix inverse or relative). A general node graph is excluded by design.

**Porting impact.** Any UE forward logic that computes values from the current pose needs a dedicated operator; see the readers, compute-FK and spine rows.

**Recommendation.** Add targeted reader operators scheduled in the pose DAG instead of a graph. For example, RigExecTransformReader: rel source, rel space, token output = translate|rotateEuler|swingAngle|twistAngle|distance, publishing float outputs:value into _resolvedInputs for FloatMathMover chains and envelopes (libs/rigExecSchema/schema.usda, the libs/rigExec/rigEvaluator.cpp pose steps, libs/rigExec/bakedProgram.cpp).

**Evidence:** `libs/rigExecSchema/schema.usda:1192-1214`; `libs/rigExecSchema/schema.usda:2119-2137`; `libs/rigExec/rigEvaluator.cpp:10236-10238`; `docs/spec.md:60`

**Verification (holds).** Verdict and severity hold. Property movers run before the pose walk (rigEvaluator.cpp:10236-10245), and a general node graph is a non-goal (spec.md:60).

### G4-space-follow-nulls

**Space / orient-space follow nulls (Construct + Evaluate Space Nulls v01)**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D4

UE features: `UE4-construct-space-nulls`, `UE4-evaluate-space-nulls`, `UE2-space-nulls`, `UE3-fk-orient-spaces`, `UE2-spine-orient-spaces`

**UE rigs.** At construction, each space target S gets a null '<Ctrl> <S> [Orient ]Space'. The null is spawned under a chosen parent at the control's offset frame and registered as a labeled available space. Every forward and backward solve then sets the null's global to N_init*inv(S_init)*S_cur. In orient-only mode it sets only the global rotation, and translation stays inherited from the null's hierarchy. There are 16 evaluate call sites: Spine builds 4 sets (Start/End x IK/FK), Neck adds a spine_05 space (16 nulls), and FK limbs, clavicles and FkArray use orient-only sets.

**usdRig today.** Author each space null as a stand-in RigExecControl (non-positive guide:scale draws nothing), nested where UE parents it. Drive it with a RigExecParentConstraint that has one source S and inputs:translationOffsets/rotationOffsets = N_rest relative to S_rest (target = Offset * Source). For orient-only, set inputs:affectTranslationX/Y/Z=false so the namespace-inherited translation survives. Nest the driven control under its space null so namespace propagation carries its avars. The result is deterministic and uses baked-program constraint operators.

**Gap.** (1) The converter must precompute offsets, because no offset relative to the bind or initial pose is computed. (2) Per-source offsets hold translation plus Euler rotation only, with no scale. (3) There is no dedicated null type. A RigExecControl stand-in is animator-visible in the Avar Editor and pickers, and plain UsdGeomXform nulls with non-identity transforms force the dynamic path. (4) A live rest edit makes the baked offsets stale.

**Porting impact.** About 40 orient-space nulls (Spine 12, Neck 16, limb FK, clavicles, FkArray) port as constraint-driven stand-in controls with baked offsets. Results match at full weight, but they clutter the animator's control set and must be re-baked after any rest edit.

**Recommendation.** Add a concrete RigExecSpace schema: a RigExecXformable with no avars and no guide, hidden from the Avar Editor and picker. It goes in libs/rigExecSchema/schema.usda, is registered in libs/rigExec/computations.cpp with RIGEXEC_REGISTER_XFORMABLE without scale inputs, and is added to the provider-type allowlist in libs/rigExec/bakedProgram.cpp:335-338. Pair it with rigExec:maintainOffset (see G4-maintain-offset) so space nulls need no baked numbers.

**Evidence:** `libs/rigExecSchema/schema.usda:1115-1142`; `libs/rigExecMath/solvers.cpp:997-1109`; `libs/rigExecSchema/schema.usda:962-978`; `libs/rigExecSchema/schema.usda:201-209`; `libs/rigExec/rigEvaluator.cpp:10661-10751`; `libs/rigExecRigging/rigBuilder.h:383-397`; `libs/rigExec/bakedProgram.cpp:258-281`; `libs/rigExec/bakedProgram.cpp:335-338`

**Verification (holds).** Verdict and severity hold. The ParentConstraint computes target = Offset*Source, with per-source translation and Euler offsets and no scale (solvers.cpp:1047-1055, schema.usda:1135-1140). A masked translation keeps the namespace-inherited input (solvers.cpp:1093-1098), which matches UE's orient-only SetRotation(global). No maintain-offset exists anywhere: the only grep hits are prose at schema.usda:875 and splineIk.h:59. A plain Xform null with a non-identity transform refuses the bake (bakedProgram.cpp:258-281). Correction to gap (3): a RigExecJoint stand-in avoids animator clutter today. It is an allowed baked provider type (bakedProgram.cpp:335-338), it is purpose=guide, guide:radius<=0 draws nothing (schema.usda:382-402), and it is not a RigExecControl, so marquee selection and the picker ignore it. A dedicated null type is therefore a convenience, not a need.

Verifier evidence: `libs/rigExecMath/solvers.cpp:1047-1055`; `libs/rigExecMath/solvers.cpp:1093-1098`; `libs/rigExec/bakedProgram.cpp:258-281`; `libs/rigExec/bakedProgram.cpp:335-338`; `libs/rigExecSchema/schema.usda:382-402`

### G4-weighted-aim-buffered

**Weighted aim (AimBoneMath / AimConstraintLocalSpaceOffset) blended against a remembered buffer**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D9

UE features: `UE4-unit-aim-bone-math`, `UE2-body-aim`, `UE2-prop-aim-solve`

**UE rigs.** AimBoneMath turns the primary axis toward a location or direction target, and turns the secondary axis toward its target projected off the primary. Body aims +Z at Body Aim with the secondary disabled; the result is slerped by 'Aim Weight' against a buffer captured every frame while the weight is 0 (persistent state). Prop chains 3 Aim Solves: AimConstraintLocalSpaceOffset on buffer nulls, aim +Y, up +X toward Aim.TransformLocation(10,0,0). Each is lerped by its own Aim Weight against a 'Previous Buffer' captured while the weight is 0, and a debug line is drawn from the control to Aim.

**usdRig today.** RigExecAimConstraint with inputs:defaultWeight connected to a custom float 'Aim Weight'. Body uses worldUpType none (minimum swing). Prop uses objectUp with worldUpObject = a provider nested under Aim at (10,0,0). The envelope blends from the control's current animated rotation, not from a remembered buffer.

**Gap.** The frozen-buffer blend is hidden previous-frame state, which the spec rejects. No aim debug-line guide exists. UE's secondary 'Location' target needs an extra helper provider.

**Porting impact.** Results match at weights 0 and 1. At intermediate Aim Weight they differ whenever the control was animated while aim was active. Scrubbing is deterministic, which improves on UE.

**Recommendation.** Keep the deterministic semantics, which the spec requires. Add rel rigExec:worldUpTarget, uniform token rigExec:worldUpTargetKind = location|direction and double3 inputs:worldUpTargetOffset to RigExecAimConstraint (libs/rigExecSchema/schema.usda, libs/rigExec/rigEvaluator.cpp:11309-11385) to remove the helper providers. Add aim-line solver guides in libs/rigExecImaging.

**Evidence:** `libs/rigExecSchema/schema.usda:1050-1052`; `libs/rigExec/rigEvaluator.cpp:11320-11352`; `libs/rigExecMath/solvers.cpp:1123-1194`; `libs/rigExecSchema/schema.usda:73-78`; `docs/spec.md:59`; `docs/spec.md:1050`

**Verification (holds).** Verdict and severity hold. With worldUpType none and authored sources, no roll correction runs, giving a minimum swing from the current rotation (solvers.cpp:1123-1124, 1152; rigEvaluator.cpp:11321). This matches AimBoneMath with a disabled secondary axis. objectUp computes upObject - child origin (rigEvaluator.cpp:11349-11350), which matches Prop's Location world-up. Aim-constraint targets stay draggable in the gizmo (gizmoMath.py:426). The buffer is hidden previous-frame state, which the spec rejects (spec.md:1050).

Verifier evidence: `libs/rigExecMath/solvers.cpp:1123-1124`; `libs/rigExec/rigEvaluator.cpp:11349-11350`; `plugin/rigExecUsdview/gizmoMath.py:426`; `docs/spec.md:1050`

### G4-aim-twist-interaction

**Interaction-aware Body/Aim twist sync (Drive Aim and Body Rotation)**

**Verdict:** Divergent-by-design · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D9

UE features: `UE2-body-aim-twist-interaction`

**UE rigs.** When Aim Weight > 0 and the animator rotates Body (IsInteracting), the Aim control is re-rotated to keep its swing while taking Body's twist, the buffer is updated, and RequestAutoKey is sent for Body and Aim. Otherwise Body's rotation = lerp(parent*buffer, Body projected under Aim, Aim Twist). The state persists across evaluations.

**usdRig today.** Evaluation has no interaction state and no event channel. The non-interactive part, Body taking Aim's twist by 'Aim Twist', maps to a RotationConstraint on Body from Aim with envelope = Aim Twist, without the buffer. Masks are asset-axis Euler components, not twist about a local axis.

**Gap.** No interaction-aware evaluation, no autokey events, and no swing/twist-split constraint.

**Porting impact.** Rotating Body while aiming does not counter-rotate the Aim control, and the twist sync is approximate.

**Recommendation.** Handle the interaction in tooling: a gizmo post-drag hook in plugin/rigExecUsdview authors Aim's compensating rotation and keys both controls. For a deterministic twist follow, add uniform token rigExec:rotationMask = euler|twist|swing and double3 inputs:twistAxis to RigExecRotationConstraint (libs/rigExecSchema/schema.usda:1087-1100, libs/rigExecMath/solvers.cpp:873-942).

**Evidence:** `docs/spec.md:56-63`; `docs/spec.md:207`; `docs/spec.md:1050`; `libs/rigExecSchema/schema.usda:1087-1100`; `libs/rigExecSchema/schema.usda:962-978`

**Verification (holds).** Verdict and severity hold. The cited non-goals exist: stateful callbacks (spec.md:59), the deferred event/direct-manipulation layer (spec.md:207), and rejected hidden previous-frame state (spec.md:1050).

Verifier evidence: `docs/spec.md:59`; `docs/spec.md:207`; `docs/spec.md:1050`

### G4-autokey-on-switch

**Key Controls event and autokey on IK/FK switch**

**Verdict:** Divergent-by-design · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D10

UE features: `UE3-autokey-on-switch`

**UE rigs.** When the IK/FK switch changes outside manipulation and auto-matching is on, the 'Key Controls' event sends RequestAutoKey (during interaction only) for the FK controls, gimbals, IK, PV, IK Base, IK Rotation and the switch. It also recomputes the segment-scale channels. The Foot keys its own controls when the parent module's metadata requests it.

**usdRig today.** Evaluation never authors values (non-destructive, and the event layer is deferred). Keying belongs to tools: the gizmo and Avar Editor author Ts knots when the user releases.

**Gap.** No key-set definition and no 'key these controls together' action.

**Porting impact.** After a switch, the dependent controls are not keyed automatically, so animation can drift between keys.

**Recommendation.** Reuse RigExecMatchAPI set membership as the key set, or add rel rigExec:keySet:members. The match_limb tool (G4-module-events-match-limb) keys every member at the current frame.

**Evidence:** `docs/spec.md:56-63`; `docs/spec.md:207`; `README.md:22-23`; `docs/viewport-gizmos.md:81-108`

**Verification (holds).** Verdict and severity hold. Evaluation is non-destructive (README.md:22-23), the event layer is deferred (spec.md:207), and the gizmo authors keys only on release (docs/viewport-gizmos.md:81-108).

Verifier evidence: `README.md:22-23`; `docs/spec.md:207`

### G4-change-pivot

**Prop 'Change Pivot' (pin the child in world while the parent moves)**

**Verdict:** Divergent-by-design · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D4

UE features: `UE2-prop-change-pivot`

**UE rigs.** Prop Global and Prop Local each have a 'Change Pivot' bool. Turning it on stores the child's world transform and recolors controls (grey and red). While it is on, the child is pinned in world, so moving the parent relocates the pivot. Turning it off autokeys the child and restores colors; visibility is forced while active. The behavior is stateful.

**usdRig today.** Hidden stored state and event-driven recoloring or autokeying are rejected by the spec. The equivalent is an authoring operation: move the parent, then author compensating child avars at the current key.

**Gap.** No tool keeps a child's world transform while its parent is moved in Pose mode. Preserve Children exists only for rest (Pivot mode) edits.

**Porting impact.** The prop pivot-relocation workflow is lost, and animators must counter-animate.

**Recommendation.** Extend the gizmo's Preserve Children option (docs/viewport-gizmos.md:55-59) to Pose mode in plugin/rigExecUsdview. It would evaluate the children's world frames through Rig before and after the drag and author compensated child avars, and be exposed as a picker command.

**Evidence:** `docs/spec.md:59`; `docs/spec.md:1050`; `docs/spec.md:207`; `python/_rigexec.cpp:902`; `docs/viewport-gizmos.md:52-63`

**Verification (holds).** Verdict and severity hold. Preserve Children applies only in Pivot mode (docs/viewport-gizmos.md:52-63), and the multi-selection group target declines it (gizmoMath.py:2766-2771). The stateful pin-and-recolor behavior falls under the spec's non-goals.

Verifier evidence: `docs/viewport-gizmos.md:52-63`; `plugin/rigExecUsdview/gizmoMath.py:2766-2771`

### G4-movable-pivot-proxy

**Movable Pivot proxy (rotate a control about an interactively placed pivot)**

**Verdict:** Divergent-by-design · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D9

UE features: `UE4-movable-proxy-forward`

**UE rigs.** Body and Spine have a 'Movable Pivot' proxy. Translating the proxy moves only the pivot. Rotating it orbits the driven control about the pivot, using a buffer captured at gesture start (IsSet metadata). When not interacting, the proxy null re-snaps to {driven rotation, SnapTo translation} and the proxy's rotation resets, while its translation offset is kept. A bool channel gates visibility.

**usdRig today.** Evaluation holds no interaction state. A static pivot rig (pivot control > rotate control > inverse-offset provider) would need a negated link between avars, which does not exist, and avars have no pivot term. Gizmo Pivot mode edits rest:t/r, which is a rig edit, not animation.

**Gap.** No rotate-about-pivot channel and no gesture-scoped temporary-pivot tool.

**Porting impact.** Animators lose rotating Body or Spine about an arbitrary point and must counter-animate translation by hand.

**Recommendation.** Add double avars:pivotX/Y/Z to RigExecXformable (libs/rigExecSchema/schema.usda), composed as T(-p)*S*R*T(p)*T(t) in _ComposeAvars (libs/rigExec/computations.cpp:192-222) and the baked compose. Add a 'temporary pivot' gizmo mode in plugin/rigExecUsdview (gizmoMath.py) that rotates about a picked point and writes compensated avars, with no evaluation state.

**Evidence:** `docs/viewport-gizmos.md:52-63`; `libs/rigExec/computations.cpp:192-222`; `docs/spec.md:207`; `docs/biped-rig.md:190-193`

**Verification (holds).** Verdict and severity hold, but the gap text is wrong in two places.
(a) A negated avar link does exist. Translation avars are AttributeValue inputs that follow a single same-typed connection, and they are multiplied by avars:unitScaleFactor before composing (computations.cpp:364-371). Avar connections are used in the biped (Biped.usda:4743-4745). So pivot control P (t=p) > rotate control R > inverse provider N (avars:t* connected to P, unitScaleFactor=-1) yields T(-p)*R*T(p), a static animatable rotate-about-pivot rig that also bakes (bakedProgramImpl.h:360-364).
(b) The gizmo already orbits a multi-selection about the 'Last Selected' control and writes compensating translations (gizmoMath.py:1264-1288, gizmoSettings.py:75).
Only UE's interaction-state proxy remains rejected (spec.md:59, 207, 1050).

Verifier evidence: `libs/rigExec/computations.cpp:364-371`; `examples/biped/Biped.usda:4743-4745`; `libs/rigExec/bakedProgramImpl.h:360-364`; `plugin/rigExecUsdview/gizmoMath.py:1264-1288`; `plugin/rigExecUsdview/gizmoSettings.py:75`

### G4-transform-get-set-units

**Imperative transform get/set units (GetTransform / SetTransform / SetRotation / OffsetTransformForItem)**

**Verdict:** Divergent-by-design · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D9

UE features: `UE4-unit-transform-get-set`

**UE rigs.** GetTransform and GetTransformItemArray read global or local, current or initial transforms. SetTransform and SetRotation write global or local, current or initial values with a weight and optional propagation. OffsetTransformForItem sets new = Offset * previous. Construction writes initial values to 'zero' controls.

**usdRig today.** There are no imperative gets or sets. Initial writes become authored rest:space and default:* (zero pose) at build time. Global sets become constraints, weighted sets become envelopes, and offsets become ParentConstraint offsets or default:* channels.

**Gap.** No ordered imperative writes to arbitrary items, no optional non-propagation, and no runtime rewrites of control offsets.

**Porting impact.** A converter has to re-express every module graph declaratively, and order-dependent overwrite patterns need explicit mover ordering.

**Recommendation.** Keep the declarative model. Add builder helpers in libs/rigExecRigging/rigBuilder.h and the python/rigexec Builder: 'zero control at a bone' (rest/default from a matrix) and 'drive X from Y with an offset'.

**Evidence:** `docs/spec.md:56-63`; `libs/rigExecSchema/schema.usda:292-326`; `docs/xformable-default-spaces.md:1-50`; `libs/rigExec/rigEvaluator.cpp:10661-10751`

**Verification (holds).** Verdict and severity hold. The spec's non-goals include no general node graph (spec.md:60) and no stateful callbacks (spec.md:59). Writes are declarative movers in a hierarchy order.

Verifier evidence: `docs/spec.md:59-60`

### G4-control-drives-bone

**Bone follows a control or virtual bone with maintained offset (ProjectTransformToNewParent / SetTransform copy)**

**Verdict:** Implemented · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D9

UE features: `UE6-control-drives-bone`, `UE2-body-drive-pelvis`, `UE2-root-drive-root-bone`, `UE3-bone-follow-virtual`, `UE2-pin-transform-copy`

**UE rigs.** Bone global = bone_init relative to driver_init, times driver_cur, with propagation (via ProjectTransformToNewParent). This covers 16 face pairs, Body/Body Offset->pelvis, and IkFk2Bones bones<->virtual bones in both directions. Root bone global = Root control global. Pin copies driver global to the driven bone without propagation (MR_FN_Biped only).

**usdRig today.** RigExecParentConstraint with one source and inputs:translationOffsets/rotationOffsets = bone_rest relative to driver_rest (target = Offset * Source). Enable affectScale when needed, and leave offsets at identity for the root and pin copies. 'Drive Body Joint' maps to inputs:enabled.

**Gap.** The converter must precompute offsets (no maintain-offset), offsets carry no scale, and writes always propagate. Pin's no-propagation matters only when an unpinned child exists.

**Porting impact.** Nothing material for Zebra or Monster.

**Recommendation.** rigExec:maintainOffset (G4-maintain-offset) removes the converter step, and rigExec:propagate (G4-fk-drive-bones) covers Pin.

**Evidence:** `libs/rigExecMath/solvers.cpp:1047-1055`; `libs/rigExecSchema/schema.usda:1115-1142`; `libs/rigExecRigging/rigBuilder.h:383-397`; `docs/biped-rig.md:185-189`; `libs/rigExec/rigEvaluator.cpp:10661-10751`

**Verification (holds).** Verdict and severity hold. A rigid bone_init*inv(ctrl_init) is representable as a per-source translation plus Euler offset. One small residual: UE's FTransform composes non-uniform scale component-wise, while usdRig multiplies offset*source matrices and SVD-decomposes the result (solvers.cpp:1054-1064). A non-uniformly scaled face control with a rotated offset therefore decomposes differently. The ParentConstraint also rejects inputs:scaleOffset (offsetGroup None, rigEvaluator.cpp:265-267).

Verifier evidence: `libs/rigExecMath/solvers.cpp:1054-1064`; `libs/rigExec/rigEvaluator.cpp:265-267`

### G4-fk-through-driven-bones

**Controls parented under driven bones (FkArray default-space nulls, tongue FK)**

**Verdict:** Implemented · **Severity:** minor · **Effort:** S · **Confidence:** medium · **Domain:** D5

UE features: `UE2-fkarray-construction`, `UE1-layout-fkarray-extras`, `UE6-tongue-fk`

**UE rigs.** FkArray gives each bone a '<Bone> Default Space' null under the bone's skeletal parent (or an override), then a null and a control. The chain relationship runs through the driven bones (ear_02's space follows bone ear_01), and missing bones are filtered out. Tongue controls 2-4 are children of the previous skeleton bone; because each bone is set in order by ProjectTransformToNewParent, the result behaves as an FK chain.

**usdRig today.** Nest each control under the joint UE parents it to, and drive each joint with a ParentConstraint that has a baked offset. When a constraint or solver revises a joint, its RigExec namespace descendants (nested controls included) are re-posed by the delta. A later constraint's source reads that propagated frame, and published control frames follow.

**Gap.** Mover order must be authored so each parent bone's constraint runs first (movers run in reverse-sibling order). Filtering missing bones is construction tooling (G1). docs/biped-rig.md:136-141 claims control guides do not follow solver-posed parents, which contradicts the current propagation code.

**Porting impact.** Ears, mohawk, tweakers and tongue port, provided the mover order is authored carefully.

**Recommendation.** Fix docs/biped-rig.md:136-141. Add a regression test to tests/testRigExecConstraints.cpp: a control nested under a constraint-driven joint feeds the next joint's constraint. Add a builder helper in libs/rigExecRigging/rigBuilder.h that emits the nesting and the ordered constraints.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:10661-10751`; `libs/rigExec/rigEvaluator.cpp:10474-10493`; `libs/rigExec/rigEvaluator.cpp:10971-10973`; `libs/rigExec/rigEvaluator.cpp:11583-11596`; `tests/testUsdviewParamNode.py:4-18`; `docs/biped-rig.md:136-141`

**Verification (holds).** Verdict and severity hold. commitConstraintFrames re-poses non-owning descendants of both constraint and solver revisions (rigEvaluator.cpp:10661-10751, 10971-10973). TestSolverOwnedJointBlocksNamespacePropagation shows a control nested under a solver-bound joint staying with that joint (testRigExecConstraints.cpp:2977-3074). testUsdviewParamNode.py:4-18 asserts that param controls nested under the wrist follow it in both IK and FK. docs/biped-rig.md:136-141, and the catalog's R1-namespace-propagation limitation, are stale. Minor reflects the ordering burden: constraints form a total order (rigEvaluator.cpp:5856-5857).

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:10661-10751`; `tests/testRigExecConstraints.cpp:2977-3074`; `tests/testUsdviewParamNode.py:4-18`; `libs/rigExec/rigEvaluator.cpp:5856-5857`

### G4-pelvis-txy-space

**Pelvis TXY ground-projected null injected as a space on root/Root**

**Verdict:** Implemented · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D4

UE features: `UE2-spine-pelvis-txy-root-space`

**UE rigs.** A parentless null, 'Pelvis TXY', copies the pelvis bone's global X and Y on every forward solve, with Z=0 and identity rotation. The Spine module registers it as an available space on root/Root through Root module metadata.

**usdRig today.** Use a RigExecPositionConstraint on a top-level stand-in provider at the origin, with sources=[pelvis joint] and inputs:affectTranslationZ=false; translation Z stays at the authored 0 and rotation is untouched. The cross-module injection is composition: the spine layer adds this provider as a source of root/Root's space constraint through an over.

**Gap.** Only the space registration is missing (G4-available-spaces-switching), along with the null type. Masks use asset axes: UE projects in Z-up world XY, so a Y-up stage has to mask Y instead.

**Porting impact.** None beyond the space-switch gap.

**Recommendation.** No new operator. The converter maps the up axis and emits the over that adds this source to root/Root's RigExecSpaceSwitch.

**Evidence:** `libs/rigExecSchema/schema.usda:1073-1085`; `libs/rigExecMath/solvers.cpp:804-871`; `libs/rigExecSchema/schema.usda:962-966`

**Verification (holds).** Verdict and severity hold. The PositionConstraint writes only the unmasked translation axes and keeps the input linear part (solvers.cpp:844-869), so a top-level stand-in keeps identity rotation and Z=0. The converter must handle an ordering subtlety. The root bone's constraint propagates its delta to the pelvis descendant (rigEvaluator.cpp:10661-10751), and TXY -> root/Root space -> root bone depends on the pelvis write. The pelvis drive therefore has to be applied again after the root drive (UE reads a stale TXY instead). A workaround exists, so minor is right.

Verifier evidence: `libs/rigExecMath/solvers.cpp:844-869`; `libs/rigExec/rigEvaluator.cpp:10661-10751`

### G4-root-body-prop-stacks

**Root / Body / Prop control stacks (nested transform controls, controls with no bone output used as spaces)**

**Verdict:** Implemented · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D4

UE features: `UE2-root-control-stack`, `UE1-layout-root`, `UE2-body-control-stack`, `UE1-layout-body`, `UE2-prop-control-stack`, `UE1-layout-prop`

**UE rigs.** Root: Global has no parent, and Local and Root are both children of Global. Body: Body Orbit (at the hip midpoint, under root/Local) > Body > optional Body Offset, with Body Aim under Orbit at +38 Z. Prop: Prop Global > Prop Local > Prop > Attach 01/02, with Aim placed 40 units along its parent's +Y; Prop drives no bones and serves as an arm IK space. The stacks carry shapes, colors and channels: Bake Root On (int enum), Control Path Vis/Distance, Aim Weight/Twist, Change Pivot and visibility bools.

**usdRig today.** Nested RigExecControls with rest:space (placement computed by the converter) and guide:shape, scale and color. Controls without joint outputs are valid, and constraints can use them as sources.

**Gap.** Remaining gaps belong to other rows: the UE shape library (usdRig has 6 primitives), int/enum/bool channels and channel hosting, and control-path debug lines are G3; spaces are G4-available-spaces-switching.

**Porting impact.** The hierarchy ports one-to-one.

**Recommendation.** Nothing new within G4. Channel types and guide offsets are tracked by G3.

**Evidence:** `libs/rigExecSchema/schema.usda:144-250`; `libs/rigExecSchema/schema.usda:252-358`; `libs/rigExec/computations.cpp:336-383`; `libs/rigExec/rigEvaluator.cpp:10474-10493`

**Verification (holds).** Verdict and severity hold. Nested RigExecControls with rest:space and no joint outputs are valid, and constraints can use them as sources (resolveBinding reads any provider, rigEvaluator.cpp:10474-10493). The remaining items (shapes, channels, spaces) belong to other rows.

### G4-default-space-multiparent

**Default active orient space and per-channel multi-parent weights at construction**

**Verdict:** Partial · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D4 · *(analyst said Partial / minor)*

UE features: `UE1-orient-spaces-multiparent`, `UE2-fkchain-default-orient-space`

**UE rigs.** When 'Default FK Space Index' >= 0, construction runs SwitchParent(maintain global). FK 0 then starts with parents [Default FK Space w=0, <idx> Orient Space w=1]; Arm L/R FK 0 default to spine_05. Each parent's weight is a separate Location/Rotation/Scale triple. Legs use -1, which keeps the default space.

**usdRig today.** The default is authoring-time data: the authored default of the space weights (or of an activeSpace value). Maintaining the global pose at construction amounts to baking the control's rest relative to the chosen space. A single weight per source covers all channels, so per-channel weights require separate masked constraints.

**Gap.** No per-channel source weights. The default space is not designated separately from the animated value: any animation overrides the default opinion.

**Porting impact.** The spine_05 default for arm FK ports as an authored default value. The impact is negligible.

**Recommendation.** Add uniform int rigExec:defaultSpace to RigExecSpaceSwitch (used when inputs:activeSpace is unauthored). Add optional float[] inputs:translationWeights/rotationWeights/scaleWeights parallel to the sources in libs/rigExecSchema/schema.usda, consumed per channel group in RigExecApplyParentConstraint (libs/rigExecMath/solvers.cpp:997-1109).

**Evidence:** `libs/rigExecSchema/schema.usda:1014-1018`; `libs/rigExecMath/solvers.cpp:1027-1103`; `libs/rigExecSchema/schema.usda:962-978`

**Verification (corrected).** The verdict holds, but the severity is too high. On the UE side, the only relevant override in MR_Zebra is 'Default FK Space Index=0' (4 hits in asset.t3d). The runtime multi-parent weights are uniform (0,0,0)/(1,1,1) Location/Rotation/Scale triples (runtime_hierarchy.txt:855, 919). An authored default one-hot envelope reproduces this exactly, and neither Zebra nor Monster uses per-channel weights. The analyst's own porting impact says 'negligible', which is cosmetic.

Verifier evidence: `ue/<dump>/Game__Assets__Zebra__Rig__MR_Zebra/asset.t3d (Default FK Space Index=0 x4)`; `ue/<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:855`

### G4-get-array-parents

**Get Array Parents (default-parent query)**

**Verdict:** Implemented · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D4

UE features: `UE4-get-array-parents`

**UE rigs.** Returns each item's default (first) parent. FkArray uses it at construction, and LimbTwist uses it in forward and backward solves to drive the parent nulls of the twist controls.

**usdRig today.** The default parent is the USD namespace parent, which authoring tools can query. Driving a parent provider is an ordinary constraint target.

**Gap.** None.

**Porting impact.** None.

**Recommendation.** None.

**Evidence:** `libs/rigExec/computations.cpp:452-454`; `libs/rigExec/rigEvaluator.cpp:10661-10751`

**Verification (holds).** Verdict and severity hold. The default parent is the namespace parent, and a parent provider is an ordinary constraint target.

### G4-head-attach-null

**Head Attach Null (face root follows the Parent connector)**

**Verdict:** Implemented · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D4

UE features: `UE6-head-attach-null`

**UE rigs.** A parentless null is placed at the head bone's global transform and ParentConstrained to the Parent connector (the head) with maintained offset, weight 1, Average. Jaw, Muzzle, Reverse Jaw, Head Squash and Cheek controls hang under it.

**usdRig today.** Nest the face root provider under the head joint, since namespace propagation passes through solver- and constraint-posed joints. Alternatively, use a ParentConstraint with a baked offset.

**Gap.** Only the null type, covered in G4-space-follow-nulls.

**Porting impact.** None.

**Recommendation.** None.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:10661-10751`; `tests/testUsdviewParamNode.py:4-18`; `libs/rigExecSchema/schema.usda:1115-1142`

**Verification (holds).** Verdict and severity hold. A provider nested under a solver- or constraint-posed joint follows that joint (the G4-fk-through-driven-bones evidence). A baked-offset ParentConstraint to the head is equally valid.

Verifier evidence: `tests/testRigExecConstraints.cpp:2977-3074`

### G4-mirror-transform

**Get Mirror Transform (unused library function)**

**Verdict:** Not-applicable · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D9

UE features: `UE4-get-mirror-transform`

**UE rigs.** Mirror = Transform * scale(-Axis). It has no callers, and with the default axis it degenerates (the Y and Z scales become zero).

**usdRig today.** Nothing uses it. usdRig mirrors at composition time: the right side references the left side's layer with a mirrored rest.

**Gap.** None for this port. Runtime pose mirroring belongs to G1/G3.

**Porting impact.** None.

**Recommendation.** None.

**Evidence:** `docs/biped-rig.md:237-252`

**Verification (holds).** Verdict and severity hold. The function has no callers, and mirroring happens at composition time (docs/biped-rig.md:237-252).

### G4-seq-no-constraints

**No Sequencer transform-constraint channels (Zebra and Boombox keyed separately)**

**Verdict:** Not-applicable · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D10

UE features: `UE8-seq-no-constraints`

**UE rigs.** No sequence carries ConstraintAndActiveChannel or tickable constraints. The Boombox is a separate spawnable actor with its own transform and MR_Boombox tracks, so the hand-prop relationship exists only as keys or through the Zebra Prop/IK spaces. CRU_PropAim is not referenced by any sequence.

**usdRig today.** There is nothing to port. The Prop module and IK spaces rows cover the relationship inside the rig. Cross-rig constraint writes would be rejected anyway.

**Gap.** None for these shots.

**Porting impact.** None. Both actors' keys port independently.

**Recommendation.** None.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:3308-3313`

**Verification (holds).** Verdict and severity hold. There is nothing to port, and cross-rig writes are rejected anyway (rigEvaluator.cpp:3308-3313).

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:3308-3313`

## G5 — IK, spine/neck, twist and foot

usdRig already has the core IK/spine/twist operators these rigs need: analytic TwoBoneIk with pole, softness and stretch, BlendPointFrames for IK/FK, FABRIK SingleChainIk, SplineIk, TwistDistribution, and Aim/Parent/Position/Rotation constraints. The biped already uses UE-like patterns: virtual spine_ik joints that drive bind joints, a mid-follow null, aim-based foot/ball solves, and no-twist aims. The main semantic gaps are in the solver models:
- **Soft IK with stretch:** UE scales both bone lengths smoothly by d/d_soft and caps stretch at 4x. usdRig's clampWithSoftness plus uniformSegments pops at dist == chain (solvers.cpp:146-157) and has no cap.
- **Spine:** UE uses a cubic Bezier through the control positions, a length-preserving fit plus a Stretch lerp, tangent frames with a slerped up vector, and end-control orientation overrides. SplineIk is a Maya-style degree-2 curve over rest-joint CVs with chord aim, linear twist and volume thinning.
- **Limb twist:** UE distributes only twist, keeps each bone's own rest position and orientation, and uses separate translate weights. TwistDistribution also distributes swing, puts the samples on the start-end line using the twist weights, and gives every sample the start's rest axes.

The second cross-cutting gap is scalar plumbing. Property movers are float-only and run before exec, while avars and many solver inputs are double. So UE logic that reads a control's local rotation, remaps or clamps it, and writes pivot, Distribute Rotation, Mid Blend or segment-scale channels cannot be expressed exactly. The biped works around this with float dials and twin-rotation constraints, which only approximate within ±180°. There is no pivot-translate channel for UE's Rotate Around Free Pivot. There is no post-pose scalar reader (the chain-length Squetch curve) and no post-pose joint-corrective pass (helper twist-bone volume slides).

Several patterns are expressible but only with extra relay prims and connected default/parent spaces, which forces dynamic evaluation because the bake refuses connected spaces: FK layered on a solved chain (Sec FK), free pivots, and Distribute Rotation. IK End Align and the Mid pin control would otherwise create solver cycles or need constraint-based IK/FK blending. Construction-time work (IK-plane helper joints, PV placement, twist-bone discovery, footprint pivots) is Divergent-by-design: build tooling does it, and none of that tooling is in the repo. IK/FK matching, backwards solves and foot resets are Missing. Recommended additions:
- a RigExecDoubleMathMover with a remapRange op, plus float↔double scalar connections
- avars:pivotX/Y/Z
- a RigExecSwingTwistConstraint
- TwoBoneIk options: softScale stretch, maxStretch, upper/lower scale, signed axes, end-orient and mid-offset controls
- SplineIk controlBezier / stretch / frame-model options
- FkChain baseChain layering
- TwistDistribution twistOnly / rest-preserving / position-weight options
- a post-pose RigExecChainReader and a joint-corrective pass

| Row | Verdict | Severity | Effort | Summary |
|---|---|---|---|---|
| [`G5-ikfk-match`](#g5-ikfk-match) | Missing | major | L | No IK/FK snap, PV snap, back-solves or auto-matching; evaluation cannot write control values, so tool-time only |
| [`G5-spine-squetch`](#g5-spine-squetch) | Missing | major | M | No operator publishes the SplineIk stretch ratio as a float; BlendInput silently clamps negative weights to 0 |
| [`G5-twist-volume-correctives`](#g5-twist-volume-correctives) | Missing | major | L | No post-pose joint corrective: RigExecPose weights feed only geometry; approximate via MatrixMover or baked shapes |
| [`G5-foot-roll-bank`](#g5-foot-roll-bank) | Partial | major | M | Rocker rotation cannot be read (no double math mover or clamped remap); roll needs float dial + twin RotationConstraints |
| [`G5-soft-ik-stretch`](#g5-soft-ik-stretch) | Partial | major | M | TwoBoneIk stretches only past full reach (pop at dist==chain when soft); no 4x cap, stretchPolicy token never read |
| [`G5-spine-curve`](#g5-spine-curve) | Partial | major | M | SplineIk is a degree-2 B-spline on rest joint origins, not a cubic Bezier on control positions; End rotation bends it |
| [`G5-spine-fit-orient`](#g5-spine-fit-orient) | Partial | major | M | SplineIk always stretches (no length-preserving fit or Stretch lerp) and uses chord aim + swing-twist, not slerped up |
| [`G5-twist-distribution`](#g5-twist-distribution) | Partial | major | M | RigExecTwistDistribution also spreads swing, lerps positions start-end and ignores joint rests; fixes cost constraints |
| [`G5-auto-pv-space`](#g5-auto-pv-space) | Partial | minor | M | Doable with Position+Aim constraints on a hidden Auto PV control; envelope blends Euler not slerp, offset fixed at rest |
| [`G5-fabrik-operator`](#g5-fabrik-operator) | Partial | minor | M | SingleChainIkConstraint re-aims joint X onto segments, no precision/iteration/set-effector inputs; Zebra use is an aim |
| [`G5-foot-controls-channels`](#g5-foot-controls-channels) | Partial | minor | S | Maps to RigExecControls with avars:rotationOrder and float dials; bool channels, limits, proxy type missing (G3) |
| [`G5-ik-end-align`](#g5-ik-end-align) | Partial | minor | M | TwoBoneIk always copies effector orientation to end; add post-blend RotationConstraint, envelope = End Align x ikfk |
| [`G5-ik-output-axes`](#g5-ik-output-axes) | Partial | minor | M | TwoBoneIk/SplineIk/SingleChainIk force +X aim with no signed axes; bind re-framed helpers, FK offset in FK control rest |
| [`G5-ik-solver-inherited-scale`](#g5-ik-solver-inherited-scale) | Partial | minor | M | TwoBoneIk root/mid outputs ignore parent scale and lengths never scale; keep stretch 1, add RigExecScaleConstraint |
| [`G5-limb-module-layout`](#g5-limb-module-layout) | Partial | minor | M | TwoBoneIk + FkChain + BlendPointFrames (avars:ikfk) topology exists as in the biped; no config-driven module builder |
| [`G5-mid-pin-control`](#g5-mid-pin-control) | Partial | minor | M | No TwoBoneIk mid-offset input; needs ~5 post-solve constraints per limb, and the Mid null blends Euler not slerp |
| [`G5-pole-vector-placement`](#g5-pole-vector-placement) | Partial | minor | S | Solve-time pole projection matches UE Location; no tool for v01/v02 flip-safe PV placement and no PV snap |
| [`G5-segment-scale`](#g5-segment-scale) | Partial | minor | M | No per-segment scale input; emulate with RigExecMatrixMathMover on helper rest:space and FK null default:space |
| [`G5-spine-distribute-rotation`](#g5-spine-distribute-rotation) | Partial | minor | S | Nothing can scale a double avar; approximate with a RotationConstraint toward a mirror control (per-axis Euler) |
| [`G5-spine-mid-blend`](#g5-spine-mid-blend) | Partial | minor | S | Mid follow is a [0.5,0.5] ParentConstraint; a float dial cannot drive sourceWeights or double midFollowWeight |
| [`G5-spine-module-layout`](#g5-spine-module-layout) | Partial | minor | M | Maps to nested RigExecControls plus two SplineIk on helper joints constrained to *_bind joints; no module builder |
| [`G5-spine-secfk-layer`](#g5-spine-secfk-layer) | Partial | minor | M | No FK-on-solved-chain operator; relays with connected defaultSpace work but force dynamic eval (baked program refused) |
| [`G5-twist-start-detwist`](#g5-twist-start-detwist) | Partial | minor | M | No swing-twist constraint or no-propagate flag; pose DAG runs de-twist before TwistDistribution, needs driver joints |
| [`G5-ik-plane-helper-chain`](#g5-ik-plane-helper-chain) | Divergent-by-design | minor | S | No construction event: tooling must pre-author planar RigExecJoint helpers; no IK-plane projection tool in repo |
| [`G5-limbtwist-setup`](#g5-limbtwist-setup) | Divergent-by-design | minor | S | No construction event or token-based discovery; tooling must author twist RigExecControls and ParentConstraints |
| [`G5-foot-ball-toe`](#g5-foot-ball-toe) | Implemented | minor | S | Maps to AimConstraints (foot worldUpType none, toe objectUp locator) with precomputed vectors; toes use FkChain |
| [`G5-foot-pivot-stack`](#g5-foot-pivot-stack) | Implemented | minor | S | Maps to nested RigExecControls under the leg IK control, innermost = TwoBoneIk effectorControl; pivot rests build-time |
| [`G5-free-pivot`](#g5-free-pivot) | Implemented | minor | M | No avars pivot channel; exact bakeable emulation via 3 nested nulls with connected avars, unitScaleFactor -1 (untested) |
| [`G5-ik-base-parent-buffer`](#g5-ik-base-parent-buffer) | Implemented | minor | S | Maps to RigExecControl as rigExec:rootControl under a scale-off ParentConstraint buffer; rot/scale lock missing (G3) |
| [`G5-twist-translate`](#g5-twist-translate) | Implemented | minor | S | Maps to RigExecParentConstraint (rotation masked) weights [w,1-w]; put offset on Start scaled 1/w, baked by tooling |
| [`G5-foot-pivot-sockets`](#g5-foot-pivot-sockets) | Implemented | cosmetic | S | Maps to RigExecJoint/RigExecControl under the ball joint with the socket offset in rest:space; no socket/tag schema |
| [`G5-spine-pelvis-local`](#g5-spine-pelvis-local) | Implemented | cosmetic | S | Maps to RigExecControl under Pelvis Sec FK driving pelvis via ParentConstraint; a later one re-imposes spine_01 |
| [`G5-spine-sliding-proxy`](#g5-spine-sliding-proxy) | Not-applicable | cosmetic | S | Not applicable: unused in UE; SplineIk has no curve-sample output and IsInteracting state is spec-rejected |
| [`G5-unused-ik-helpers`](#g5-unused-ik-helpers) | Not-applicable | cosmetic | S | Not applicable: the UE helpers have no callers, and the Snap Global Control latch is hidden state the spec rejects |

### G5-ikfk-match

**IK/FK matching and backwards solves (PV snap, segment-scale back-solve, foot reset, twist/spine inverse)**

**Verdict:** Missing · **Severity:** major · **Effort:** L · **Confidence:** high · **Domain:** D10

UE features: `UE8-foot-reset-rig-values`, `UE3-segment-scale`, `UE3-pv-location`, `UE3-limbtwist-forward`, `UE2-spine-attach-to-spline`, `UE2-spine-distribute-rotation`

**UE rigs.** Match IK / To IK / Key Controls:
- place the IK control at the end bone and the PV with Compute PV Location v02;
- set Upper/Lower Segment Scale = current length / default length;
- on the foot, zero the pivot controls, run Reset Foot Rig Values, rerun Set Foot Pivots and snap Toes IK to the ball.

Backwards solves:
- LimbTwist twist controls follow the bones;
- the spine Sec FK and match nulls are recomputed with the Matching branch;
- Distribute Rotation is set to 0.

**usdRig today.** No engine or tool support exists: docs/biped-rig.md:190-193 lists the ikfk_switch snap as not built. The Python inverse solver is a generic numeric boundary with no limb adapters, and interactive overrides only preview. Backwards (inverse) execution is not an engine concept. UI and tools are outside the core spec (docs/spec.md:63).

**Gap.** Missing: FK↔IK snap for limbs; the PV snap; the segment-scale back-solve; the foot pivot reset with Toes IK snap; inverse posing of twist and Sec FK controls from joints; and key-on-switch.

**Porting impact.** Animators cannot switch IK/FK mid-shot without a pop, nor bake IK to FK, and the Zebra Utils events (To IK / To FK / Key Controls) have no equivalent.

**Recommendation.** Add python/rigexec/matching.py:
- match_limb(rig, limb_cfg, to='ik'|'fk', time): uses Rig.evaluate joint frames; authors the FK control avars by decomposing joint-local × control-default⁻¹ for a parentRelative FkChain; places the IK effector at the end joint (with the IK Null offset); places the PV with limbs.pole_vector_location; sets segment ratios = current/rest lengths.
- match_foot(...): zeroes the pivot controls and snaps Toes IK to the ball.
- inverse helpers for twist and Sec FK controls.

Expose it as usdview actions in plugin/rigExecUsdview, authoring through the existing undo EditScope, with an optional key-on-switch. Keep it out of the evaluator, per spec.md:63.

**Evidence:** `docs/biped-rig.md:190-193`; `docs/biped-rig.md:99-107`; `docs/spec.md:56-63`; `libs/rigExecSchema/schema.usda:599-652`

**Verification (holds).** Verdict and severity hold. No matching code exists:
- python/rigexec has only bake.py, curvenet.py, inverse.py and __init__.py.
- A plugin/ grep for ikfk/pole finds only avar-editor and picker notes.
- docs/biped-rig.md:191 lists the ikfk_switch snap as not built.

The row understates the UE side in two ways:
1. With 'IK FK Auto Matching' = True (IkFk2Bones summary.json:58), Match FK runs every IK-mode forward frame and Match IK every FK-mode frame (graphs.txt:580-596). Inactive controls therefore always follow, including the foot, through the parent Match metadata (CRM_FN_Foot graphs.txt:100-111). usdRig evaluation cannot write control values (spec.md:58), so only a tool-time snap is possible. This part is tracked in G4 UE3-ikfk-matching.
2. The spine Backwards solve also matches the FK trio from bones, places Mid IK by ray intersection and resets Pelvis Local and Distribute Rotation. That belongs in the same matching tool.

Verifier evidence: `docs/biped-rig.md:186-193`; `python/rigexec/inverse.py:12-174`; `ue/<dump>/.../CRM_FN_IkFk2Bones/summary.json:58`; `ue/<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:580-596`; `ue/<dump>/FortniteRigs__Modules__Biped__Foot__CRM_FN_Foot/graphs.txt:100-111`; `docs/spec.md:56-63`

### G5-spine-squetch

**Spine Squetch: chain stretch ratio − 1 written to a morph curve**

**Verdict:** Missing · **Severity:** major · **Effort:** M · **Confidence:** high · **Domain:** D7

UE features: `UE-spine-squetch`

**UE rigs.** CR_Zebra_Deform: RigUnit_ChainInfo over [spine_05..spine_01], with bCalculateStretch and current lengths. ChainStretchFactor = current / initial chain length, and SetCurveValue('Squetch') = factor − 1, which drives the Squetch morph target. The value is negative under compression.

**usdRig today.** No operator publishes a chain length or stretch ratio as a float:
- SplineIk computes the ratio only internally for volume weights and publishes frames only (splineIk.h:244-252; computations.cpp:1171-1187).
- Property movers run before exec and cannot read poses (rigEvaluator.cpp:11667-11675); only RigExecPose.outputs:weight is published after the pose (rigEvaluator.cpp:2794-2802).
- BlendInput clamps weights to [0, lastActivation] and rejects activation <= 0 (moverGraph.cpp:1861-1871), so the compression half cannot drive a shape.

**Gap.** Missing: a post-pose scalar reader for chain length/stretch, an offset/scale on such a reader, and negative-weight drive of a blend shape.

**Porting impact.** The Squetch morph never fires, so torso squash/stretch shape correction is lost on the Zebra.

**Recommendation.** Add RigExecChainReader (new class in schema.usda near RigExecPoseInterpolator) with:
- rel rigExec:joints (ordered)
- uniform token rigExec:measure = stretchRatio|length|distance
- float inputs:scale and inputs:offset (UE: offset −1)
- float outputs:value, plus optional outputs:positive and outputs:negative

Evaluate it in the pose-interpolator phase (next to _EvaluatePoseInterpolators, rigEvaluator.cpp:11667-11675), publishing floats like RigExecPose.outputs:weight, and add a baked step. For compression, feed a second BlendInput from outputs:negative, or allow negative activations in the BlendInput kernel (moverGraph.cpp:1861-1871).

**Evidence:** `libs/rigExecMath/splineIk.h:244-252`; `libs/rigExec/computations.cpp:1171-1187`; `libs/rigExec/rigEvaluator.cpp:11659-11675`; `libs/rigExec/rigEvaluator.cpp:2794-2802`; `libs/rigExec/moverGraph.cpp:1861-1874`

**Verification (holds).** Verdict and severity hold. I checked for proxies:
- SplineIk's ratio is internal (splineIk.cpp:550), and it publishes frames only.
- RigExecPoseInterpolator declares enableTranslation, but evaluation never measures translation. It warns at compile (rigEvaluator.cpp:2552-2561) and calls Evaluate with a null translation (rigEvaluator.cpp:2783), so a translation-driven stretch reader is impossible.
- Property chains run before exec.
- BlendInput clamps weights to [0, last activation] and rejects activation <= 0 (moverGraph.cpp:1861-1871), so the compression half needs a precomputed negated-delta sample.
- A hack (a helper TwoBoneIk bend angle read by an RBF) would measure chord distance rather than summed segment length, so it is not faithful.

Verifier evidence: `libs/rigExecMath/splineIk.cpp:550`; `libs/rigExec/rigEvaluator.cpp:2552-2561`; `libs/rigExec/rigEvaluator.cpp:2760-2790`; `libs/rigExec/rigEvaluator.cpp:11659-11675`; `libs/rigExec/moverGraph.cpp:1861-1874`; `libs/rigExecSchema/schema.usda:1874-1889`

### G5-twist-volume-correctives

**Twist-bone volume slides driven by elbow/knee bend (post-process, AdditiveLocal)**

**Verdict:** Missing · **Severity:** major · **Effort:** L · **Confidence:** high · **Domain:** D7

UE features: `UE-helper-limb-volume`

**UE rigs.** CR_Zebra_Deform runs 26 ModifyTransforms (AdditiveLocal) on twist bones:
- Arms: local X translations (−8/−5/−4, +10/+4) once bend passes 40% (Remap 0.4..1).
- Legs: local Y translations and thigh_twist_01 rotated Z−20.
- Knee bend slides calf twist bones by 5 and 2 cm; shoulder-up slides upperarm twist bones by 2 and 1 cm.

The drivers are SphericalPoseReader outputs (G6). The offsets are applied on top of the incoming twist pose.

**usdRig today.** There is no stage in which pose-reader outputs drive joint offsets:
- Property movers run before exec and cannot read poses (rigEvaluator.cpp:11667-11675).
- RigExecPose.outputs:weight is published after the whole pose walk, and only geometry chains consume it (rigEvaluator.cpp:11667-11675, 2794-2802).
- Constraint offsets are asset-space additive (schema.usda:980-991), not local.

The usdRig-native alternative is PSD corrective blendshapes driven by RigExecPoseInterpolator. That needs sculpted shapes the UE asset does not have.

**Gap.** Missing: a post-pose joint-corrective pass, a local-space additive TRS offset operator on joints, and pose-reader outputs as its envelope.

**Porting impact.** Elbow, knee and shoulder volume preservation on the Zebra body is lost, so skin collapses visibly on deep bends.

**Recommendation.** Add RigExecJointOffsetMover with:
- rel rigExec:moves = one joint
- double3 inputs:translate, inputs:rotate and inputs:scale (local additive)
- rigExec:rotationOrder
- envelope inputs:defaultWeight, connectable to RigExecPose or pose-reader outputs

Evaluate it in a new corrective phase after _EvaluatePoseInterpolators (rigEvaluator.cpp:11667-11675) and before the geometry chains: re-commit the joint frames and propagate to non-owned descendants, and add a bake step. G6 supplies the SphericalPoseReader-equivalent driver. The Remap(0.4..1) should be a reader parameter or a post-pose remap.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:11659-11675`; `libs/rigExec/rigEvaluator.cpp:2794-2802`; `libs/rigExecSchema/schema.usda:980-991`; `libs/rigExecMath/solvers.cpp:804-871`

**Verification (holds).** Verdict and severity hold.

Why no joint-domain equivalent exists:
- Property chains run before exec, and pose-interpolator outputs are published only into resolved inputs for geometry chains (rigEvaluator.cpp:11659-11675, 2723-2731).
- No joint-domain post-pose offset operator exists. The generic ModifyTransforms operator is G4 UE-modifytransforms-op.

Approximations the row should list as interim alternatives:
(a) A pre-skin RigExecMatrixMover.
- Provider: a control at the twist joint's rest with a constant avar offset, whose rest-to-pose map equals the offset in joint space.
- Weight: a CombineWeight of the joint's skin weights × a DynamicWeight whose inputs:driver connects to RigExecPose.outputs:weight. The driver resolves through resolved inputs (rigEvaluator.cpp:7571), and the skin mover skins the preceding revision.
- Exact only for points fully weighted to that joint; clearly off where upper and lower twist weights mix at the elbow.
(b) Corrective shapes baked offline from the UE LBS result. No sculpting is needed, contrary to the row.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:11659-11675`; `libs/rigExec/rigEvaluator.cpp:2674-2800`; `libs/rigExec/rigEvaluator.cpp:7571`; `libs/rigExec/weightPackets.cpp:94-150`; `libs/rigExecSchema/schema.usda:1284-1301`; `libs/rigExecSchema/schema.usda:980-991`

### G5-foot-roll-bank

**Foot roll (heel/ball/toe rocker), banking and heel lift from control rotations (Set Foot Values / Set Foot Pivots)**

**Verdict:** Partial · **Severity:** major · **Effort:** M · **Confidence:** high · **Domain:** D8

UE features: `UE3-foot-roll-bank`

**UE rigs.** Set Foot Values reads these scalars:
- R = twist about X of the Foot Rocker local rotation;
- Bank = twist about Y;
- HeelBend/Twist/Side = Ball IK local Euler YZX.

Set Foot Pivots, in IK mode:
- ToeTipRocker X = Remap(R, [−360, S] → [−360, 0]) with S = −RBR·B, clamped;
- HeelRocker X = clamp(R, 0, 360);
- Inner and Outer Y = side-dependent clamp(Bank, 0..90 / −90..0);
- Heel Lift = Euler(HeelBend + clamp(R, −RBR, 0)·B, HeelTwist, HeelSide) · initial;
- free pivots (next row);
- the leg effector = the foot IK null.

**usdRig today.** The biped realizes roll and bank with RotationConstraints toward pre-rotated '_twin' prims. Their inputs:defaultWeight comes from float dials through FloatMathMover add/multiply/clamp chains (Biped.usda:3084-3157; docs/biped-rig.md:109-112). UE's piecewise remaps can be approximated this way, but:
- Property chains only revise float targets, and avars are double (rigEvaluator.cpp:560-566, 3570-3573). The rocker's and Ball IK's rotations therefore cannot be read, so the roll input must become a float dial.
- The angle equals weight × twin angle only within ±180°, because envelopes blend shortest per-axis Euler deltas (solvers.cpp:780-800).
- Every rotating pivot needs a twin prim and a constraint.

**Gap.** Missing pieces:
- a double-typed math mover that can write avars directly
- float↔double scalar conversion
- a remap-range operation (source range → target range with clamp); the current remap only normalizes and is unclamped
- a swing-twist rotation read of a control's local rotation

**Porting impact.** Animators lose 'rotate the Foot Rocker to roll or bank'; UE keys on the rocker rotation cannot be reused directly. The heel-lift coupling to Ball IK rotation, and rolls beyond ±180° or with non-linear remaps, deviate. Zebra walk and run feet look different.

**Recommendation.** Add RigExecDoubleMathMover (schema.usda next to RigExecFloatMathMover:1192-1214; kernel in libs/rigExecMath/propertyMath.h/.cpp; validation in rigEvaluator.cpp:3536-3597; property-chain bake in bakedProgram and bakedPose). It targets any double attribute, including avars:*. Add a 'remapRange' operation (inputs:min/max → inputs:targetMin/targetMax, uniform bool rigExec:clamp) to all scalar math movers, and allow float↔double connections in _ValidateScalarConnection (rigEvaluator.cpp:525-569).

Then python/rigexec/modules/foot.py can author Set Foot Pivots exactly: each pivot's avars:rx/ry = remapRange/clamp chains fed by the rocker's avars:rx/ry and Ball IK's avars. For exact twist reads, use RigExecSwingTwistConstraint (G5-twist-start-detwist).

**Evidence:** `examples/biped/Biped.usda:3084-3157`; `docs/biped-rig.md:109-112`; `libs/rigExec/rigEvaluator.cpp:560-566`; `libs/rigExec/rigEvaluator.cpp:3570-3597`; `libs/rigExecMath/propertyMath.cpp:11-61`; `libs/rigExecMath/solvers.cpp:780-800`

**Verification (holds).** Verdict and severity hold.

Why the rocker rotation cannot be read:
- FloatMathMover only targets float (rigEvaluator.cpp:3570-3573), and connections require the exact type (rigEvaluator.cpp:560-566).
- DynamicWeight's driver is also a float (schema.usda:1296-1300).
- MatrixMathMover blends componentwise (propertyMath.cpp:98-130), so an envelope cannot produce an exact rotation angle.
- So the rocker's rotation (roll and bank, with clamps) cannot be read. The twin + RotationConstraint route (exact within ±180°, solvers.cpp:780-800) driven by a float dial remains the only path.

What still works: the Heel Lift coupling to Ball IK rotation can use double-to-double avar connections on a separate nested null (Biped.usda:4743-4745). The truly lost part is therefore the 'rotate the rocker' interaction with its clamped piecewise reads, which is the primary foot-roll manipulation in UE.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:560-566`; `libs/rigExec/rigEvaluator.cpp:3570-3597`; `libs/rigExecMath/propertyMath.cpp:33-61`; `libs/rigExecMath/propertyMath.cpp:98-130`; `libs/rigExecSchema/schema.usda:1284-1301`; `libs/rigExecMath/solvers.cpp:780-800`; `examples/biped/Biped.usda:3084-3157`; `examples/biped/Biped.usda:4743-4745`

### G5-soft-ik-stretch

**Soft IK combined with stretch (UE soft-stretch, 4x cap, 0.95 softness distance)**

**Verdict:** Partial · **Severity:** major · **Effort:** M · **Confidence:** high · **Domain:** D6

UE features: `UE3-soft-ik`

**UE rigs.** Definitions:
- L = LA·Us + LB·Ls.
- ds = Remap(Softness, 0..1 → L..0.95L).
- d_soft = L - (L-ds)·exp(-(d-ds)/(L-ds)).

With Stretch off: the effector is moved to the soft effector along root→effector and the initial lengths are used, so the end lags the goal.

With Stretch on: the real effector is used, with lengths LA·Us·d/d_soft and LB·Ls·d/d_soft, so the chain always reaches the goal with a residual bend.

Engine stretch: StretchStartRatio=1 and StretchMaximumRatio=4, so hard stretch reaches up to 4x. The pole is the PV control position (Location kind). The effector rotation is multiplied by the inverse IK Rotation Offset. Stretch is a bool channel and Softness is a float in 0..1.

**usdRig today.** RigExecTwoBoneIk:
- Soft reach: soft = softness·chain; reach = chain - soft·exp(-(dist-(chain-soft))/soft). This is UE's formula with softness_usd = 0.05·Softness_UE, applied with a FloatMathMover multiply on the float inputs:softness.
- Stretch: factor = 1 + (dist/chain - 1)·stretch, applied only when factor > 1 (solvers.cpp:135-159).
  - Soft-only matches UE with Stretch off.
  - Hard stretch (softness 0, stretch 1) matches UE except that it has no 4x cap.
- Pole: the pole-control origin projected off the aim (solvers.cpp:161-168), which matches PoleVectorKind=Location.
- Effector offset: a nested 'IK Null' control under the IK control carries the rest rotation offset.
- stretchPolicy and unreachablePolicy each have a single token and are never read by the computation.

**Gap.** Soft plus stretch has different semantics. UE lengthens the bones continuously from ds onward and always reaches the goal. usdRig does not stretch while dist < chain because of the 'factor > 1' guard. At dist == chain the reach jumps from chain - 0.368·soft to chain and the limb snaps straight, so there is a visible pop. Also missing:
- a maximum-stretch cap (UE 4.0) and a stretch start ratio
- a softness option in UE units (0.95 soft-zone parameter)
- bool channel semantics

**Porting impact.** Zebra limbs keep Stretch=true by default. As soon as an animator adds Softness, the knee or elbow pops straight at full extension, and the soft pull-to-straight no longer matches UE poses. The pop is most visible in legs during walk cycles.

**Recommendation.** In libs/rigExecMath/solvers.cpp RigExecSolveTwoBoneIk, implement a 'softScale' stretch policy:
- When soft > 0 and dist > chain - soft, set s1,s2 *= lerp(1, dist/reach_soft, stretch) and reach = dist.
- Add double inputs:maxStretch (0 = uncapped; UE uses 4) and inputs:stretchStartRatio (UE uses 1).
- Clamp the uniformSegments factor to maxStretch.

Schema and plumbing:
- Extend rigExec:stretchPolicy allowedTokens to [uniformSegments, softScale] in schema.usda:564-575.
- Read the token in computations.cpp:_ComputeTwoBoneIk, where it is currently ignored, and mirror it in bakedPose.cpp.
- Optionally add uniform token rigExec:softnessUnits = chainFraction|softZone so the UE 0..1 Softness dial connects directly.
- Fix or document the discontinuity of the existing policy.

**Evidence:** `libs/rigExecMath/solvers.cpp:135-159`; `libs/rigExecMath/solvers.cpp:161-168`; `libs/rigExec/computations.cpp:742-757`; `libs/rigExec/computations.cpp:819-862`; `libs/rigExecSchema/schema.usda:564-576`

**Verification (holds).** Verdict and severity hold.

The discontinuity is confirmed in solvers.cpp:135-159:
- Stretch applies only when factor > 1, i.e. dist > chain.
- In the band chain-soft < dist < chain, reach stays at the soft value.
- Just past chain, s1+s2 = dist, so cosAlpha = 1 and the limb goes straight. The jump is about 0.368*soft.

The UE side is also confirmed. With Softness > 0, Soft IK passes explicit lengths L*d/d_soft, so the reach ratio d_soft/L < 1 and the engine's 1..4x stretch never engages. With Softness 0 (or Stretch off, where the lengths are 0) the engine path is used (RigUnit_TwoBoneIKSimple.cpp:151-178).

Other checks:
- stretchPolicy and unreachablePolicy are never read. computations.cpp:728-863 reads only the stretch, softness, offset and bend inputs. The tokens appear only in the builder (rigBuilder.cpp:518-525) and in comments (solvers.h:46-47).
- inputs:softness is a float (schema.usda:569-574), so the 0.05x mapping can be done with a FloatMathMover.
- UE's Stretch channel defaults to true, which keeps the severity at major.

Verifier evidence: `libs/rigExecMath/solvers.cpp:135-159`; `libs/rigExecMath/solvers.cpp:197-204`; `libs/rigExec/computations.cpp:728-822`; `libs/rigExecRigging/rigBuilder.cpp:518-525`; `libs/rigExecSchema/schema.usda:562-574`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Highlevel/Hierarchy/RigUnit_TwoBoneIKSimple.cpp:151-190`

### G5-spine-curve

**Spine curve construction: cubic Bezier through Start Driver / Mid IK / Mid IK / End IK**

**Verdict:** Partial · **Severity:** major · **Effort:** M · **Confidence:** high · **Domain:** D7

UE features: `UE2-spine-bezier-spline`

**UE rigs.** Build Spline From Items = ControlRigSplineFromPoints with the global positions of [Start Driver, Mid IK, Mid IK, End IK]:
- BSpline, open, 16 samples per segment. With 4 points this is a clamped cubic, i.e. a cubic Bezier whose two inner CVs are both the Mid IK position.
- Rebuilt every forward frame; the original length is stored at construction.
- At rest Mid IK sits at the start/end midpoint, so the rest curve is the straight start→end line.

**usdRig today.** RigExecSplineIk builds an open degree-2 four-CV B-spline with knots [0,0,1,2,2]. Its rest CVs are the rest origins of joints [0], [1], [N-2] and [N-1], carried by the root/end control rest-to-pose maps plus the mid control's translation offset from a blended follow point (splineIk.h:33-83; schema.usda:808-814). The curve interpolates only cv0 and cv3, not the control positions, and never uses control positions as CVs.

**Gap.** Missing options:
- CVs taken from control positions
- a degree-3 (Bezier) curve
- a repeated mid CV
- a curve-length output

The curve shape therefore differs for identical control poses; the biped also documents degree-2 artifacts (a 25° chest swing gives 53.65° on spine_5).

**Porting impact.** Spine and neck body lines driven by UE-authored control animation will not match. Retargeted Zebra shots show a different torso curvature.

**Recommendation.** Add uniform token rigExec:curveModel = restJointQuadratic|controlBezier to RigExecSplineIk (schema.usda:788-924). controlBezier:
- uses CVs [root origin, mid origin, mid origin, end origin] with a degree-3 arc-length table (extend RigExecSplineIkCurve in libs/rigExecMath/splineIk.h/.cpp);
- takes the rest length from the rest control origins.

Wire it through computations.cpp:1036-1228 and the SplineIk bake step (bakedPose.cpp), and add a setter to RigExecSplineIkHandle (rigBuilder.h:277-303).

**Evidence:** `libs/rigExecMath/splineIk.h:33-83`; `libs/rigExecMath/splineIk.h:105-144`; `libs/rigExecSchema/schema.usda:788-814`; `libs/rigExec/computations.cpp:1126-1172`; `docs/biped-rig.md:171-180`

**Verification (holds).** Verdict and severity hold.

The curve model is confirmed:
- Degree 2, four CVs, knots [0,0,1,2,2].
- The CVs are rest joint origins carried by the root/end control rest-to-pose maps plus the mid offset (splineIk.h:33-83; schema.usda:808-814).

An additional divergence the row should state: cv2 and cv3 ride the END control's full rest-to-pose map, so rotating the chest control bends the curve. docs/biped-rig.md:179 documents this: a 25° chest swing gives 53.65° on spine_5. UE's clamped cubic depends only on control positions; End IK rotation affects only the up-vector slerp and the last bone.

Mid control rotation is ignored in both (schema.usda:830-833).

UE samples 16 points per segment (and the fit uses precision 12), which adds small numeric differences.

Verifier evidence: `libs/rigExecMath/splineIk.h:33-83`; `libs/rigExecMath/splineIk.h:105-144`; `libs/rigExecSchema/schema.usda:788-833`; `libs/rigExec/computations.cpp:1126-1172`; `docs/biped-rig.md:179-180`

### G5-spine-fit-orient

**Attach Sec FKs To Spline: length-preserving fit, Stretch lerp, tangent + slerped-up frames, end overrides**

**Verdict:** Partial · **Severity:** major · **Effort:** M · **Confidence:** high · **Domain:** D7

UE features: `UE2-spine-attach-to-spline`

**UE rigs.** Step A: FitChainToSplineCurve (Alignment Front, precision 12, +X primary), then restore the rest local X lengths. That is a length-preserving fit.

Step B, per bone:
- T3 = frame at length-percentage p (the 'Bone Percentage' metadata: straight-line distance from the first bone divided by arc length).
- T2 = frame at p/r, with r = length / origLength.
- I = Lerp(A, T3, Stretch), where the Stretch channel defaults to 1.
- Frames: X = curve tangent, up = Slerp(StartIK.rot, EndIK.rot, p) · SecondaryAxis.
- Last bone = End IK rot · inverse(Offset). First bone = Start IK rot · inverse(Offset) on the spine, or I on the neck.
- Sec FKs Orient Offset is applied in matching.

**usdRig today.** SplineIk places joint i at ratio × the cumulative rest spacing, so it always stretches with the curve; minLengthRatio is only a floor. Each joint aims +X at the next joint (the chord; only the last joint uses the tangent). The rest up is carried by minimal rotation, plus a linear twist taken from the root/end controls' swing-twist about the rest chain axis and inputs:roll/twist. Linear volume thinning of Y/Z is added (splineIk.h:61-82; splineIk.cpp:571-604; schema.usda:852-908). End or start orientation overrides can be applied afterwards with ParentConstraints, as the biped does for the chest (docs/biped-rig.md:185-189; Biped.usda:2882-2900).

**Gap.** Missing or different:
- No length-preserving front-fit mode (UE Stretch = 0) and no Stretch lerp input.
- No tangent-plus-slerped-up frame model; the twist model differs (whole-rotation slerp vs swing-twist linear twist).
- No built-in first/last-joint-takes-control-orientation option.
- Placement uses cumulative arc spacing, not UE's straight-line 'Bone Percentage' (a UE open question).
- Volume thinning has no UE counterpart and must be disabled (preserveVolume = 0).

**Porting impact.** Spine and neck joint positions and rotations diverge from UE for the same control poses. The Stretch dial (keyed in UE shots) has no target, and compressing the spine cannot keep bone lengths.

**Recommendation.** Extend RigExecSplineIk (schema.usda:788-924; splineIk.h/.cpp; computations.cpp:1036-1228; bake) with:
- float inputs:stretch: 0 = front fit preserving rest segment lengths, 1 = percentage placement on the current curve, lerp between.
- uniform token rigExec:placement = cumulativeArc|chordPercent.
- uniform token rigExec:frameModel = chordMinimalTwist|tangentSlerpUp, with double3 inputs:upAxis.
- uniform token rigExec:startOrientation and rigExec:endOrientation (curve|control), with optional uniform matrix4d offsets.

All must be bakeable. For UE parity use preserveVolume = 0.

**Evidence:** `libs/rigExecMath/splineIk.h:61-82`; `libs/rigExecMath/splineIk.cpp:571-604`; `libs/rigExecSchema/schema.usda:852-908`; `examples/biped/Biped.usda:2882-2900`; `docs/biped-rig.md:185-189`

**Verification (holds).** Verdict and severity hold.

What SplineIk does:
- ratio = arcLength / restArcLength (splineIk.cpp:550).
- Joint i is always placed at ratio*cumulative (splineIk.cpp:577), so there is no length-preserving front fit and no Stretch lerp.
- It aims at the next joint (the chord) with a rotation-minimising rest-up transport and linear twist (splineIk.cpp:599-625).
- It applies a volume scale (splineIk.cpp:632-640); preserveVolume = 0 disables it (schema.usda:852-866).

Start and end orientation overrides are expressible after the solve with ParentConstraints, as the biped does for the chest (Biped.usda:2885-2893).

UE up = slerp(StartIK.rot, EndIK.rot, p) is a whole-rotation model with no usdRig equivalent.

Verifier evidence: `libs/rigExecMath/splineIk.cpp:550-640`; `libs/rigExecSchema/schema.usda:852-908`; `examples/biped/Biped.usda:2885-2900`

### G5-twist-distribution

**LimbTwist weighted twist distribution (Blend Twist + Get Node Twist Value)**

**Verdict:** Partial · **Severity:** major · **Effort:** M · **Confidence:** high · **Domain:** D7

UE features: `UE4-blend-twist`, `UE3-limbtwist-blend`

**UE rigs.** t = SwingTwist(initialLocal⁻¹ · currentLocal, Axis).Twist, taken from the driver: Start in normal mode, End in reverse mode.

For each twist null i with weight w:
- local = q · initialLocal_i;
- non-reverse: q = slerp(t⁻¹, identity, w);
- reverse: q = slerp(identity, t, w).

The nulls live under a Twist Parent Null that follows Start, so only twist is distributed; swing comes rigidly from Start. Each twist bone keeps its own rest position and rotation. Weights default to (0.25, 0.75, 1), with per-module overrides. The axis is ±X.

**usdRig today.** RigExecTwistDistribution:
- Takes the relative rotation start⁻¹·end (each measured against its own rest) and splits it into swing and twist about the start's X aim.
- Sample k = qs · slerp(identity, swing, w) · twist(angle·w).
- Sample origin = lerp(start, end, w), with the same w.
- Every sample carries the START rest landmarks (solvers.cpp:294-374; solverKernels.cpp:78-91).
- Joints bound to a sample take its axes as their world axes (frameExtraction.h:62-99).

Upper-twist workaround: start = a noTwist joint (biped twist_aims), end = the upperarm driver, then PositionConstraints fix the positions afterwards.

**Gap.** Differences from UE:
- Swing is distributed too, with no twist-only mode, so forearm twist bones partially follow wrist flexion.
- Positions are forced onto the start-end line at the twist weight (weight 1 lands on the end joint) instead of the bones' own rest positions.
- Samples ignore each bound joint's rest orientation, so joint rests must be re-framed to the start's.
- Twist is always relative to the start frame, with no bind-local reference to another provider; the axis is always the start's X.
- There is no per-sample Twist offset control layer (it needs an extra constraint per bone).

**Porting impact.** The Zebra has 8 twist modules × 4 bones. With stock TwistDistribution, twist bones slide to wrong positions and bend with the wrist, which visibly breaks forearm and shin deformation. Each bone needs 2-3 extra constraints to recover UE behavior.

**Recommendation.** Extend RigExecTwistDistribution (schema.usda:654-718; solvers.cpp:294-374; solverKernels.cpp; computations.cpp:969-1025; bakedPose.cpp; rigBuilder.h:243-257) with:
- float inputs:swingWeight (0 = twist-only, UE parity) or uniform token rigExec:distribute = twistOnly|swingTwist;
- uniform token rigExec:placement = lerp|boundJointRest, where boundJointRest keeps each bound joint's rest offset relative to the start and applies the sample delta;
- publish bound-joint rests instead of the start rests;
- rel rigExec:twistReference plus uniform token rigExec:twistReferenceMode = startRelative|bindLocal;
- uniform token rigExec:twistAxis (±X/±Y/±Z).

**Evidence:** `libs/rigExecMath/solvers.cpp:270-374`; `libs/rigExec/solverKernels.cpp:59-91`; `libs/rigExec/computations.cpp:969-1025`; `libs/rigExecSchema/schema.usda:654-718`; `libs/rigExec/frameExtraction.h:62-99`; `examples/biped/Biped.usda:2363-2385`

**Verification (holds).** Verdict and severity hold. All four cited differences are confirmed:
- The relative rotation is split into swing and twist, and swing is distributed as well: sample = qs·slerp(I, swing, w)·twist(w) (solvers.cpp:321-351).
- Sample origins are the lerp of start and end at the twist weight (solvers.cpp:365-370).
- Every sample is paired with the start rest (solverKernels.cpp:77-90), and extraction uses the posed axes (frameExtraction.h:62-99), so a bound twist joint's own rest orientation and position are ignored.
- The schema has no axis, mode or reference options (schema.usda:654-718).

With UE weights (0.25, 0.75, 1) the twist bones land at wrong positions even at rest, and forearm twist bones follow wrist flexion. The workarounds exist but cost several constraints per bone, so major is proportionate for 8 modules × 4 bones.

Verifier evidence: `libs/rigExecMath/solvers.cpp:294-374`; `libs/rigExec/solverKernels.cpp:59-91`; `libs/rigExec/frameExtraction.h:62-99`; `libs/rigExecSchema/schema.usda:654-718`

### G5-auto-pv-space

**Auto pole-vector parent with PV Twist Follow**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D6

UE features: `UE3-auto-pv-parent`

**UE rigs.** Every IK frame the 'Auto PV' null is set as follows:
- Position: P = lerp(PVTwistStart, PVTwistEnd, L0/ΣL).
- Frame A aims X at the end, with Z toward End.TransformPosition(0,0,±1); this follows the end's twist.
- Frame B aims X at the end, with Z along Start's ±Z direction.
- M = TransformLerp(B, A, w), where w = the 'PV Twist Follow' channel (legs only; its variable is > 0 only on legs).
- Result = (0, ∓max(L0,L1), 0)·M, using the scaled lengths.

The PV position control sits under 'Orient PV' under Auto PV.

**usdRig today.** Expressible with constraints on a hidden 'Auto PV' RigExecControl:
- A PositionConstraint with sources [twistStart, twistEnd] and sourceWeights [1-r, r], where r is a build-time constant.
- An AimConstraint aimed at twistEnd with worldUpType objectRotationUp, worldUpObject = start, worldUpVector (0,0,±1) (weight 1).
- A second AimConstraint with worldUpObject = end, whose inputs:defaultWeight is connected to the float 'PV Twist Follow' dial.
- Orient PV and the PV control nested under it with a rest offset (0, ∓maxL, 0).

The pose DAG runs these before the TwoBoneIk that reads the PV (rigEvaluator.cpp:5926-5943).

**Gap.** Differences from UE:
- The constraint envelope blends per-axis Euler with shortest deltas (solvers.cpp:780-800, 1205-1216), not the slerp of TransformLerp, so intermediate twist-follow values drift.
- The ∓max(L0,L1) offset is fixed at rest and does not follow the segment-scale channels.
- Offsets and weights must be precomputed.

**Porting impact.** The PV rides with the limb as in UE. At fractional PV Twist Follow the PV location differs slightly, and with segment scale ≠ 1 the PV distance differs.

**Recommendation.** Extend RigExecAimConstraint (schema.usda:1021-1071; solvers.cpp:1111-1218; the rigEvaluator.cpp aim dispatch) with:
- rel rigExec:worldUpObjects plus float[] inputs:worldUpWeights, giving a weighted, quaternion-averaged up direction, so a single constraint expresses twist follow;
- uniform token rigExec:envelopeBlend = euler|slerp.

Optionally let TwoBoneIk publish its measured lengths for length-scaled offsets.

**Evidence:** `libs/rigExecSchema/schema.usda:1021-1071`; `libs/rigExecMath/solvers.cpp:1111-1218`; `libs/rigExecMath/solvers.cpp:780-800`; `libs/rigExec/rigEvaluator.cpp:5926-5943`; `examples/biped/Biped.usda:3048-3074`

**Verification (holds).** Verdict and severity hold.

The constraint stack works:
- AimConstraint objectRotationUp exists in the schema (schema.usda:1041-1052) and in the evaluator (rigEvaluator.cpp:11353-11360).
- UE Frame A aims its secondary axis at End.TransformPosition(0,0,±1). After projection off the X aim this reduces to End's ±Z, so objectRotationUp with worldUpVector (0,0,±1) is exact.
- An envelope connected to a float dial is supported (Biped.usda:3055).
- The solver waits on constraints that target ancestors of the pole control (rigEvaluator.cpp:5926-5943).

The stated differences are also confirmed:
- Envelopes blend by per-axis Euler with shortest deltas (solvers.cpp:780-800, 1213), not TransformLerp's slerp.
- The offset is a fixed rest value.

Verifier evidence: `libs/rigExecSchema/schema.usda:1021-1071`; `libs/rigExec/rigEvaluator.cpp:11321-11360`; `libs/rigExecMath/solvers.cpp:780-800`; `libs/rigExecMath/solvers.cpp:1112-1216`; `libs/rigExec/rigEvaluator.cpp:5926-5943`; `examples/biped/Biped.usda:3048-3074`

### G5-fabrik-operator

**FABRIK chain operator semantics (orientation handling, precision/iterations)**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D6

UE features: `UE3-foot-ball-toe-solve`

**UE rigs.** RigUnit_FABRIKItemArray solves positions with the given Precision and MaxIterations. It updates each bone rotation by the shortest-arc delta applied to the bone's existing orientation, propagates to children, and optionally sets the effector transform (bSetEffectorTransform).

**usdRig today.** RigExecSingleChainIkConstraint is a deterministic planar FABRIK (up to 128 iterations, scale-relative tolerance). It sets each non-end joint's X axis exactly onto its solved segment (_AimedBasis, singleChainIk.cpp:212-229, 544-549). The end transports its input orientation, or takes the effector's orientation in singleChain mode (singleChainIk.cpp:550-563). It has no precision or iteration inputs.

**Gap.** Missing or different:
- Joints whose X is not along their child are re-oriented rather than rotated by a delta.
- No user precision or iteration count.
- No 'set effector transform' toggle.

**Porting impact.** Low for Zebra: the only FABRIK use is a 2-joint foot chain, which maps to an AimConstraint. It matters if other UE rigs use longer FABRIK chains with non-X-down bones.

**Recommendation.** Add uniform token rigExec:orientMode = aimX|preserveOffset to RigExecSingleChainIkConstraint; preserveOffset applies the minimal rotation from the current to the solved segment direction to the whole basis. Also add double inputs:tolerance and uniform int rigExec:maxIterations. Files: schema.usda:1144-1190, libs/rigExecMath/singleChainIk.h/.cpp, the rigEvaluator.cpp single-chain dispatch, and bakedPose.cpp.

**Evidence:** `libs/rigExecMath/singleChainIk.h:42-66`; `libs/rigExecMath/singleChainIk.cpp:212-251`; `libs/rigExecMath/singleChainIk.cpp:310-454`; `libs/rigExecMath/singleChainIk.cpp:541-563`; `libs/rigExecSchema/schema.usda:1144-1190`

**Verification (holds).** Verdict and severity hold.

- Non-end joints are re-aimed with X along the solved segment (singleChainIk.cpp:212-229, 544-549).
- The end joint is transported or takes the effector orientation (singleChainIk.cpp:550-563).
- The schema has no tolerance, iteration or set-effector inputs (schema.usda:1144-1190).

Zebra's only use is the 2-joint foot chain, which maps to an AimConstraint.

Verifier evidence: `libs/rigExecMath/singleChainIk.cpp:212-251`; `libs/rigExecMath/singleChainIk.cpp:536-570`; `libs/rigExecSchema/schema.usda:1144-1190`

### G5-foot-controls-channels

**Foot pivot/roll controls, channels, delta metadata and visibility**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** medium · **Domain:** D8

UE features: `UE3-foot-controls`

**UE rigs.** Controls:
- Toe Tip and Heel (ZYX), Ball (ZYX, HalfCircle), Foot Rocker (YXZ, at the heel), Ball IK (YZX).
- Toes FK and Toes IK.
- Footprint Display proxy.

Channels on the rocker: 'Rocker Blend' (0..1, default 1) and 'Rocker Ball Rotation' (default 45, range 0..90). 'Foot Pivot Control Vis' is a bool on the leg IK. Delta metadata is stored relative to the ball or foot joint. Visibility: pivot controls show when IK and PivotVis are both on.

**usdRig today.** Controls are RigExecControls with avars:rotationOrder ZYX/YXZ/YZX (schema.usda:334-336). The two rocker channels become custom float avars; the biped uses float dials such as avars:footRoll and toePlantAngle (Biped.usda:3116-3156). 'Foot Pivot Control Vis' becomes a float connected to guide:displayOpacity (schema.usda:223-249). Delta metadata becomes build-time constants or customData.

**Gap.** Missing pieces:
- bool channels, channel limits (0..90), hide/show visibility and the proxy control type (G3)
- the rocker control's rotation cannot feed the roll logic (G5-foot-roll-bank)

**Porting impact.** Controls exist, but the rocker becomes 'rotate plus dial' semantics only through the roll workaround, and the channel UX is weaker.

**Recommendation.** The foot builder authors these controls. Channel limits, bool channels and visibility are covered by the G3 recommendations.

**Evidence:** `libs/rigExecSchema/schema.usda:334-336`; `libs/rigExecSchema/schema.usda:223-249`; `examples/biped/Biped.usda:3097-3157`; `libs/rigExecSchema/schema.usda:33-47`

**Verification (holds).** Verdict and severity hold.

- avars:rotationOrder supports ZYX, YXZ and YZX (schema.usda:334-336).
- A float guide:displayOpacity connection gives the visibility fade (schema.usda:223-249).
- Custom float dials follow the biped pattern (Biped.usda:3100-3156).
- Limits, bool channels and the proxy control type are G3.
- The per-frame 'Keep Foot Rocker Control At Heel' SetControlOffset is covered by G3.

Verifier evidence: `libs/rigExecSchema/schema.usda:223-249`; `libs/rigExecSchema/schema.usda:327-344`; `examples/biped/Biped.usda:3084-3157`

### G5-ik-end-align

**IK End Align: separate end-rotation control oriented from the mid bone**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D6 · *(analyst said Partial / major)* · *reconciled after probes*

UE features: `UE3-ik-end-align`

**UE rigs.** Arms have an 'IK Rotation' rotator control and an 'IK End Align' bool. While the bool is on, SetControlOffset every frame puts IK Rotation at the IK control's translation with the rotation of 'IK Rotation Null' (a child of VB1, the forearm frame). The IK Null rotation is then IK Rotation · IK Rotation Offset, so the hand orientation follows the forearm. Toggling it is edge-detected: the offset is rewritten, controls are re-snapped and autokeyed. This runs after Soft IK, so the new rotation takes effect with a one-evaluation lag (open question in the UE analysis).

**usdRig today.** TwoBoneIk always copies the effector control's orientation to the end frame (solvers.cpp:211-217). A forearm-oriented effector would read the solved mid joint and create a solver cycle, which compile rejects. Workaround:
- The IK Rotation Null is a control positioned by a PositionConstraint to the IK control and oriented by a RotationConstraint to a relay nested under the solved mid helper joint (children of solver-posed joints ride solver commits, rigEvaluator.cpp:10953-10973, 10661-10751). The IK Rotation control nests under it.
- The wrist is then revised after the solve by a RotationConstraint whose envelope is a float 'IK End Align' dial.

Because BlendPointFrames consumes aggregates, not revised joints, the IK/FK blend must then be done with weighted constraints instead of BlendPointFrames. The edge-triggered offset rewrite and autokey are hidden state, rejected by the spec (docs/spec.md:59, 1050).

**Gap.** Missing pieces:
- an end-orientation input separate from the effector
- a way to express that orientation relative to the solved mid frame inside the solver
- a toggle with snap/autokey semantics

**Porting impact.** The arm 'hand follows forearm' mode is not available with the standard TwoBoneIk plus BlendPointFrames topology. The workaround rebuilds the IK/FK blend from constraints, which blend by Euler instead of slerp, and loses the switch snapping.

**Recommendation.** Add to RigExecTwoBoneIk (schema.usda:524-597):
- rel rigExec:endOrientControl
- float inputs:endOrientWeight, connectable to a dial
- uniform token rigExec:endOrientSpace = world|midFrame, where midFrame treats the control's local rest-to-pose delta as relative to the solved mid frame

The solver computes it after the mid frame (solvers.cpp:207-217), so there is no cycle. Implement in computations.cpp:727-863 and bakedPose.cpp. Implement the snap/autokey toggle as a usdview action (plugin/rigExecUsdview) in the matching tool, not in the evaluator.

**Evidence:** `libs/rigExecMath/solvers.cpp:211-218`; `libs/rigExec/rigEvaluator.cpp:10953-10973`; `libs/rigExec/rigEvaluator.cpp:10661-10751`; `libs/rigExec/computations.cpp:869-963`; `docs/spec.md:56-63`; `docs/spec.md:1050`

**Report reconciliation.** The review cites docs/biped-rig.md:136-141 and rigBuilder.h:200-206 as saying nesting under a solver-posed joint does not propagate. Probe g2review/probe_follow2.py shows unclaimed descendants do follow; that wording describes propagation THROUGH a joint that another solver owns. The relay-control construction still works, it is just not required.

**Verification (corrected).** Severity is overstated: the claim that the TwoBoneIk + BlendPointFrames topology must be replaced by constraint blending is wrong.

Why the standard topology still works:
- UE's Ik Fk Switch is a bool; only one solve runs per frame.
- So a post-blend RotationConstraint on the end helper, with envelope = IK End Align × ikfk, reproduces the mode exactly in both switch states. The envelope comes from a FloatMathMover multiply with a connected value (Biped.usda:3100-3156).
- Constraints may target solver-bound joints (testRigExecConstraints.cpp:3181, MoveJoint on the FkChain-bound Source).

How to build the source safely:
- The IK Rotation control is nested under a hidden relay control that is ParentConstrained to the solved mid helper.
- Constraint commits propagate to non-owned descendants (rigEvaluator.cpp:10661-10751).
- This avoids nesting directly under a solver-posed joint, which docs/biped-rig.md:136-141 and rigBuilder.h:200-206 say does not propagate.

IK Rotation Offset: author it as a child relay's rest rotation, not as RotationConstraint inputs:rotationOffset. That input is an additive Euler (solvers.cpp:933), not a quaternion product.

The edge-triggered offset rewrite and autokey are toggle-time matching/tool behaviour (G4 UE3-autokey-on-switch; hidden state is rejected by spec.md:59). What remains is authoring verbosity, so severity is minor.

Verifier evidence: `libs/rigExecMath/solvers.cpp:211-218`; `libs/rigExecMath/solvers.cpp:874-942`; `libs/rigExec/rigEvaluator.cpp:10661-10751`; `tests/testRigExecConstraints.cpp:3093-3248`; `examples/biped/Biped.usda:3100-3156`; `docs/biped-rig.md:136-141`; `libs/rigExecRigging/rigBuilder.h:200-206`; `docs/spec.md:56-63`

### G5-ik-output-axes

**Signed primary/secondary axis conventions of the IK output frames**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D6

UE features: `UE3-soft-ik`, `UE4-ik-plane-virtual-bones-v02`, `UE3-ik-plane-virtual-bones`, `UE1-layout-limb-ikfk`

**UE rigs.** TwoBoneIKSimplePerItem uses PrimaryAxis (±1,0,0) and SecondaryAxis (0,∓1,0) chosen per side (Negative Side), with SecondaryAxisWeight=1. The virtual bones use the module's Primary and Secondary Axis. Skin bones are written as VB rotation * inverse(FK Rotation Offset). Right-side Fortnite bones point down -X.

**usdRig today.** RigExecTwoBoneIk rebuilds the root and mid frames rigidly with +X along the bone and +Y toward the pole (solvers.cpp:58-93, 207-209). Extraction copies those posed axes as the joint's world axes (frameExtraction.h:62-99). Skinning consumes rest-to-pose deltas, so a joint whose rest is re-framed to +X down the bone and +Y toward the pole deforms correctly. Workaround: bind the IK to re-framed helper joints, then drive the real (e.g. -X-down-bone) joints through ParentConstraints whose rotationOffsets encode UE's FK Rotation Offset.

**Gap.** TwoBoneIk has no signed primary/secondary axis selection and no 'preserve rest orientation' mode. SingleChainIk and SplineIk also force +X aim. Binding a -X-down-bone joint directly rotates it 180 degrees.

**Porting impact.** Every limb needs helper joints plus one constraint per bone. Joint local axes on the helpers differ from UE's, which matters for anything that reads bone-local axes, such as pose readers in CR_*_Deform, unless the constraint layer restores them.

**Recommendation.** Add uniform token rigExec:primaryAxis (+X/-X/+Y/-Y/+Z/-Z, default +X) and rigExec:secondaryAxis (default +Y) to RigExecTwoBoneIk in libs/rigExecSchema/schema.usda:524-597. Carry them in RigExecTwoBoneIkParams (libs/rigExecMath/solvers.h) and use them in _FrameFromAxes and the end frame (solvers.cpp:58-93, 207-209). Read them in computations.cpp:_ComputeTwoBoneIk (727-863), bake them in bakedPose.cpp, and add setters to RigExecTwoBoneIkHandle (rigBuilder.h:210-228). The same token pair would suit SingleChainIk and SplineIk.

**Evidence:** `libs/rigExecMath/solvers.cpp:58-93`; `libs/rigExecMath/solvers.cpp:207-218`; `libs/rigExec/frameExtraction.h:62-99`; `libs/rigExecSchema/schema.usda:524-597`; `examples/biped/Biped.usda:2363-2385`

**Verification (holds).** Verdict and severity hold.

Forced +X aim confirmed in each solver:
- TwoBoneIk: _FrameFromAxes rebuilds the frame with ex = bone direction and ey = bendUp, keeping only the rest handle lengths (solvers.cpp:61-93, 208-209).
- SplineIk aims +X at the next joint with a transported rest up (splineIk.cpp:599-625).
- SingleChainIk also aims +X (_AimedBasis, singleChainIk.cpp:212-229, 544-549).

Extraction publishes the posed axes as the joint's world axes (frameExtraction.h:62-99), and the schema has no axis tokens (schema.usda:524-597). A -X-down-bone joint bound directly therefore gets a non-identity rest-to-pose delta at rest.

The helper-joint workaround is valid. UE itself follows the virtual bones with a bind-offset projection (IkFk2Bones graphs.txt:337-345), and the biped binds IK directly to re-framed joints (Biped.usda:5901-5935).

Correction to the mapping: UE's FK Rotation Offset relates the FK controls to the virtual bones (graphs.txt:366-373). In usdRig it belongs in the FK control rest orientation, not in the skin-follow constraint.

Verifier evidence: `libs/rigExecMath/solvers.cpp:61-93`; `libs/rigExecMath/solvers.cpp:208-217`; `libs/rigExec/frameExtraction.h:62-99`; `libs/rigExecMath/splineIk.cpp:599-625`; `libs/rigExecMath/singleChainIk.cpp:212-229`; `libs/rigExecSchema/schema.usda:524-597`; `examples/biped/Biped.usda:5901-5935`

### G5-ik-solver-inherited-scale

**Inherited (global/parent) scale in two-bone IK outputs and stretch-off lengths**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D6 · *added by verifier*

UE features: `UE3-soft-ik`, `UE3-ik-base-control`, `UE3-ik-plane-virtual-bones`

**UE rigs.** TwoBoneIKSimplePerItem starts from Item A's current global transform (virtual bone A, which sits under the limb's parent bone and inherits its scale) and writes it back with that scale (RigUnit_TwoBoneIKSimple.cpp:150-153, 209-214).

When ItemALength/ItemBLength are 0 (the Soft IK passes 0 when Stretch is off), the lengths are the initial distances times the item's current/initial scale (RigUnit_TwoBoneIKSimple.cpp:151-178). A scaled root/Global control or parent bone therefore scales the solved limb and its lengths. The root controls are EULER_TRANSFORM with no limits (MR_Zebra runtime_hierarchy.txt:2147).

The Parent Buffer drops parent scale only for the IK Base position.

**usdRig today.** How TwoBoneIk handles scale:
- It uses only the root and pole origins (solvers.cpp:105-107).
- It rebuilds the root/mid frames with the rest handle lengths (solvers.cpp:82-92, 208-209), so those outputs always extract with unit scale relative to rest (frameExtraction.h:62-99).
- The end frame copies the effector frame, including its scale (solvers.cpp:211-217).
- The FkChain carries affine scale (solvers.cpp:16-41).
- Bone lengths are rest measurements plus offsets and never scale (computations.cpp:819-822).

Workarounds:
- Keep inputs:stretch = 1 (matching UE's Stretch default) so a scaled goal is still reached.
- Re-apply the parent/root scale to the helper or skin joints with a RigExecScaleConstraint after the IK/FK blend, the pattern the biped uses for its spine (Biped.usda:2885-2900).

**Gap.** TwoBoneIk has no scale inheritance on its root/mid outputs and no scale-aware length measurement. Under a scaled rig root or limb parent:
- IK limb joints stay at unit scale while FK results are scaled, so switching IK/FK or blending shows a scale difference.
- With stretch 0, the chain cannot reach a goal that moved with the scale.

**Porting impact.** This matters only when the Zebra root/Global control or a limb parent (clavicle, pelvis) is scaled. IK arms and legs then keep their original thickness unless ScaleConstraints are added.

**Recommendation.** Add uniform token rigExec:scaleSource = none|rootControl|rootJointParent to RigExecTwoBoneIk (libs/rigExecSchema/schema.usda:524-597). When set:
- measure the rest-to-pose scale of that provider;
- multiply the measured lengths when stretch is 0;
- write the scale into the root/mid output handles in _FrameFromAxes (libs/rigExecMath/solvers.cpp:58-93).

Plumb it through libs/rigExec/computations.cpp:728-863 and bakedPose.cpp. The same option would suit RigExecSplineIk output handles (splineIk.cpp:630-640).

**Evidence:** `libs/rigExecMath/solvers.cpp:82-92`; `libs/rigExecMath/solvers.cpp:105-111`; `libs/rigExecMath/solvers.cpp:208-217`; `libs/rigExec/frameExtraction.h:62-99`; `libs/rigExec/computations.cpp:819-822`; `examples/biped/Biped.usda:2885-2900`

### G5-limb-module-layout

**IkFk2Bones limb module layout (Arm/Leg)**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D6

UE features: `UE1-layout-limb-ikfk`

**UE rigs.** Arm has 22 controls and leg 21. Contents:
- FK 0/1/2 with gimbals, and Upper/Lower Segment Scale FK nulls.
- IK Base on a Parent Buffer; IK with 4 (leg) or 9 (arm) spaces; PV (POSITION control) under Orient PV / Auto PV; Mid under a world Mid null.
- Arms only: IK Rotation and IK End Align.
- Channels: Ik Fk Switch (bool, hosted on all limb controls), Stretch, Softness, PV Twist Follow, segment scales, visibility bools.
- Virtual bones A/B/C.
- Arm Default IK=False; leg IK End Align=False.

**usdRig today.** The solver topology is what the biped ships per limb: TwoBoneIk plus a parentRelative FkChain plus BlendPointFrames, with the weight connected to a float avars:ikfk dial (Biped.usda:5901-5960). Gimbal and segment-scale nulls are nested RigExecControls. IK, PV, Mid and IK Rotation are RigExecControls. Helper joints and constraints cover virtual bones, the auto-PV space, the parent buffer and the mid pin (rows above). The remaining gaps belong to other groups: POSITION/ROTATOR control types, limits, bool channels with host lists, and animatable IK/PV space switching (G3/G4).

**Gap.** No in-repo module builder reproduces IkFk2Bones from config (Primary/Secondary Axis, Negative Side, PV Distance Scale, Default IK, IK End Align, Segment Scale Control). The row-specific solve gaps are listed separately.

**Porting impact.** All four Zebra limbs must be authored by new tooling. The layout itself is expressible.

**Recommendation.** Add python/rigexec/modules/limb.py, an IkFk2Bones-equivalent builder on rigexec.Builder. It authors helper joints, the FK/IK/PV/Mid controls, the solvers, the constraints (auto-PV, parent buffer, mid pin, skin follow) and the float dials from a module config dict mirroring the UE module variables.

**Evidence:** `examples/biped/Biped.usda:5901-5960`; `libs/rigExecSchema/schema.usda:418-499`; `libs/rigExecSchema/schema.usda:599-652`; `libs/rigExecSchema/schema.usda:33-47`; `docs/biped-rig.md:89-107`

**Verification (holds).** Verdict and severity hold.

The biped ships the same per-limb solver topology:
- TwoBoneIk (Biped.usda:5901).
- A parentRelative FkChain (5917-5933).
- BlendPointFrames with its weight connected to the float avars:ikfk (5935-5940).

Nested RigExecControls and constraints cover the remaining nulls (see the rows above). There is no module builder in python/rigexec or libs/rigExecRigging, and the channel, limit and host items belong to G3/G4.

Verifier evidence: `examples/biped/Biped.usda:5901-5960`; `libs/rigExecSchema/schema.usda:418-522`; `libs/rigExecSchema/schema.usda:599-652`; `libs/rigExecRigging/rigBuilder.h:209-228`

### G5-mid-pin-control

**Mid (elbow/knee) offset/pin control on a slerped Mid null**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D6

UE features: `UE3-mid-control`

**UE rigs.** After the IK or FK solve, every frame: Mid Null = (slerp(VB0.rot, VB1.rot, 0.5), VB1.t). The 'Mid' control underneath (HalfCircle, visible with Sec Controls Vis) is used by Compute FK: VB0 re-aims at the Mid control, VB1 moves to the Mid control and aims at VB2 with twist from the Mid control's Z axis, and VB2 keeps its transform. Backwards sets the Mid Null local to identity.

**usdRig today.** Expressible post-solve with constraints on solver-owned helper joints:
- The Mid Null is a control with a PositionConstraint to the mid helper and a RotationConstraint to [root helper, mid helper] weights [0.5, 0.5]. That blend is Euler, not slerp (solvers.cpp:873-942).
- AimConstraint on the root helper, aimed at the Mid control.
- PositionConstraint on the mid helper to the Mid control.
- AimConstraint on the mid helper, aimed at the end helper, with objectRotationUp from the Mid control.

The end helper does not move, because a solver-bound joint blocks namespace propagation (rigEvaluator.cpp:10629-10704). Constraints read the preceding revision, so there is no cycle.

**Gap.** No built-in mid-offset input on TwoBoneIk. The workaround takes about 5 constraints per limb, blends the Mid-null rotation by Euler instead of slerp, and must re-derive the aim vectors if helper axes change.

**Porting impact.** The elbow/knee pin works but is verbose. The Mid-null orientation differs slightly at large bends, which shifts the pin gizmo frame.

**Recommendation.** Add rel rigExec:midOffsetControl and float inputs:midOffsetWeight to RigExecTwoBoneIk. Inside the solver, apply the control's local rest-to-pose delta (read from its avars, not its posed frame, to avoid a cycle) in the frame slerp(root, mid) at the mid position. Then re-aim root→mid and mid→end, taking twist from the offset frame's up axis. Files: schema.usda:524-597, solvers.cpp:97-219, computations.cpp:727-863, bakedPose.cpp.

**Evidence:** `libs/rigExecMath/solvers.cpp:873-942`; `libs/rigExecMath/solvers.cpp:1111-1218`; `libs/rigExec/rigEvaluator.cpp:10629-10704`; `libs/rigExecSchema/schema.usda:1044-1052`

**Verification (holds).** Verdict and severity hold.

The workaround is sound:
- Constraints may target solver-bound helpers.
- Propagation stops at solver-owned descendants (tests/testRigExecConstraints.cpp:2990-3091; rigEvaluator.cpp:10629-10704), so the end helper holds.
- Constraint sources read the preceding revision in the pose walk, so there is no cycle.
- Mid-null rotation averaging is per-axis Euler (solvers.cpp:874-942), not slerp.

UE Compute FK takes VB twist from the Mid control Z axis (G4 UE3-compute-fk). AimConstraint objectRotationUp (schema.usda:1041-1052) covers that. The mid pin applies in both IK and FK modes and is expressible after BlendPointFrames.

Verifier evidence: `libs/rigExecMath/solvers.cpp:874-942`; `libs/rigExecMath/solvers.cpp:1112-1216`; `libs/rigExec/rigEvaluator.cpp:10629-10704`; `tests/testRigExecConstraints.cpp:2990-3091`; `libs/rigExecSchema/schema.usda:1041-1052`

### G5-pole-vector-placement

**Flip-safe pole vector location (Compute Pole Vector v01 / Location v02)**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D6

UE features: `UE4-compute-pole-vector-v01`, `UE4-pv-location-v02`, `UE3-pv-location`

**UE rigs.** v01: the pole vector is the perpendicular from the S-E line to the (optionally plane-projected) mid, and it also returns the upper length. v02: offset = unit(PV_cur) · upperLen · PV Offset (PV Distance Scale), negated when dot(PV_cur, R) <= 0, where R carries the initial pole direction in the current mid frame; result = elbow + offset. It is used at construction (placing the Orient PV null) and in Match IK (placing the PV control).

**usdRig today.** Solve-time pole semantics are Implemented: the pole is the control origin projected off the aim, with a deterministic fallback ladder (solvers.cpp:161-195). The construction use becomes build-time tooling that authors the PV control rest. The Match IK use has no counterpart; see G5-ikfk-match. preferredBendRadians only applies when the pole is degenerate.

**Gap.** No library or tool implements the v01/v02 PV location formula, including its flip test, and there is no runtime PV snap.

**Porting impact.** Initial PV placement must be computed offline. Snapping the PV during IK/FK switches is lost; see the matching row.

**Recommendation.** Add pole_vector_location(bones_initial, bones_current, secondary_axis, pv_offset) to python/rigexec/limbs.py, reproducing v01 and v02 exactly (including the dot<=0 negation). Use it from the limb builder and from the matching tool.

**Evidence:** `libs/rigExecMath/solvers.cpp:161-195`; `libs/rigExecSchema/schema.usda:547-549`; `libs/rigExecSchema/schema.usda:570`; `docs/biped-rig.md:190-193`

**Verification (holds).** Verdict and severity hold.

Solve-time semantics match. UE's Location kind is TransformPositionNoScale of the PV control (RigUnit_TwoBoneIKSimple.cpp:131-143), then projected off the aim. That equals usdRig's pole origin projection (solvers.cpp:161-195).

No v01/v02 PV formula exists anywhere:
- python/rigexec has none.
- A plugin/ grep for pole/ikfk hits only avar-editor and picker notes.

Note: 'IK FK Auto Matching' defaults to True (IkFk2Bones summary.json:58). Match IK, and therefore Compute PV Location v02, runs every FK-mode forward frame (graphs.txt:580-596), not only at construction and on explicit Match. That per-frame part is tracked by G4 UE3-ikfk-matching.

Verifier evidence: `libs/rigExecMath/solvers.cpp:161-195`; `libs/rigExecSchema/schema.usda:545-568`; `ue/<dump>/.../CRM_FN_IkFk2Bones/summary.json:58`; `ue/<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:580-596`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Highlevel/Hierarchy/RigUnit_TwoBoneIKSimple.cpp:131-143`

### G5-segment-scale

**Upper/Lower Segment Scale channels shared by FK and IK**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D6 · *(analyst said Partial / major)*

UE features: `UE3-segment-scale`

**UE rigs.** SCALE_FLOAT channels Upper/Lower Segment Scale (initial 1, min 1e-4, max 2) live on the IK control and are hosted on FK 0/1/2 and IK Base.
- In IK they scale the Soft IK lengths and the auto-PV offset.
- In FK the FK scales are locked to 1, and the parent null of FK1/FK2 gets local X = initialX·scale.
- Backwards solve and Key Controls set channel = current length / default length (IK stretch carried into FK).

**usdRig today.** IK: rigExec:upperLengthOffset and lowerLengthOffset are additive double deltas on the measured lengths (schema.usda:550-563; computations.cpp:819-822). A multiplicative ratio dial cannot drive them: FloatMathMover only targets float (rigEvaluator.cpp:3570-3573), and scalar connections must have the same type (rigEvaluator.cpp:560-566). Only a double custom attribute holding an offset in cm can be connected directly. FK: stretch requires translating the parentRelative FK controls (avars:tx), which likewise cannot be computed from a float dial. The back-solve (channel from the current length) belongs to matching, which is Missing.

**Gap.** Missing pieces:
- a multiplicative per-segment scale input on TwoBoneIk
- any way to derive FK control translations or IK length offsets from one shared ratio channel (no double math mover, no float→double conversion)
- channel-host sharing (G3)
- the IK→FK back-solve of the channel values

**Porting impact.** Animators lose the single limb-proportion dial that works in both FK and IK. Stretch keyed in IK cannot be carried over to FK, and existing UE keys on Upper/Lower Segment Scale have no direct target.

**Recommendation.** IK: add float inputs:upperScale = 1 and float inputs:lowerScale = 1 to RigExecTwoBoneIk (schema.usda:550-576), applied as (measured·scale + offset) in computations.cpp:819-822 and the bake. Being float, they can be connected to dials and revised by FloatMathMover. FK: add RigExecDoubleMathMover (see G5-foot-roll-bank) so a chain like add(-1)·multiply(restLocalX) can write the child FK control's avars:tx. The back-solve belongs in the matching tool (G5-ikfk-match).

**Evidence:** `libs/rigExecSchema/schema.usda:550-563`; `libs/rigExec/computations.cpp:819-822`; `libs/rigExec/rigEvaluator.cpp:560-566`; `libs/rigExec/rigEvaluator.cpp:3570-3597`; `libs/rigExecMath/propertyMath.cpp:33-61`

**Verification (corrected).** The claim that no shared ratio dial can derive FK translations or IK lengths is refuted: RigExecMatrixMathMover exists (schema.usda:2119-2137). It revises any matrix4d property, and property results feed exec as overrides (rigEvaluator.cpp:8772-8985, 11659-11665). Its inputs:defaultWeight envelope is read through connections and property chains (rigEvaluator.cpp:8880-8897), so a FloatMathMover chain can compute it from a custom float dial, as the biped foot does (Biped.usda:3100-3156).

IK side:
- Post-multiplying T(restLen*(s-1), 0, 0) onto the mid/end helper joints' rest:space changes the measured TwoBoneIk bone lengths (computations.cpp:808-822; spec.md:242).
- Chain-written rest:space evaluates and bakes (testRigExecEpochRests.cpp:507-540; bakedProgram.cpp:355-418; docs/baked-step-graph.md:813-818).

FK side:
- The same mover on an explicitly authored default:space of the 'Upper/Lower Segment Scale FK' null translates the nested FK control (computations.cpp:302-316, 337-383).
- An unconnected, chain-written default:space bakes (bakedProgram.cpp:398-418).
- A translation-only envelope blend is exact (propertyMath.cpp:98-130).

Costs and caveats:
- Two movers are needed per segment (s<1 and s>1), because envelopes must be in [0,1].
- default:space must be authored in full, because identity selects the computed fallback.
- UE keys can target the custom float dial directly.
- UE quirk the port must reproduce: with Stretch off, Soft IK passes lengths 0, so the engine uses initial distances times bone scale and segment scale does not lengthen the IK bones (RigUnit_TwoBoneIKSimple.cpp:151-178). Multiply the envelope by the stretch dial to match.

What remains is verbosity, the auto-PV offset scaling, the IK->FK back-solve (the matching row) and channel hosting (G3). Severity: minor.

Verifier evidence: `libs/rigExecSchema/schema.usda:2119-2137`; `libs/rigExec/rigEvaluator.cpp:3536-3597`; `libs/rigExec/rigEvaluator.cpp:8880-8897`; `libs/rigExecMath/propertyMath.cpp:98-130`; `libs/rigExec/computations.cpp:302-316`; `libs/rigExec/computations.cpp:337-383`; `libs/rigExec/computations.cpp:808-822`; `tests/testRigExecEpochRests.cpp:507-540`; `libs/rigExec/bakedProgram.cpp:355-418`; `examples/biped/Biped.usda:3100-3156`; `examples/09_PropertyMathMovers.usda:124-133`

### G5-spine-distribute-rotation

**Distribute Rotation (fraction of chest FK swing re-applied at spine base)**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** medium · **Domain:** D7

UE features: `UE2-spine-distribute-rotation`

**UE rigs.** Forward step:
- e = End FK local rotation as ZYX Euler.
- q = FromEuler(s·e.X, s·e.Y, 0), where s = the 'Distribute Rotation' channel (0..1, default 0).
- End IK Null local = q · its initial local.

End IK Null sits at the spine start between Mid FK and End FK's space, so End FK, End IK and Mid IK pivot about the spine base. Backwards sets s = 0.

**usdRig today.** End IK Null's avars:rx/ry can be connected to End FK's avars:rx/ry (a double→double connection is allowed, rigEvaluator.cpp:560-566), but nothing can scale a double: FloatMathMover targets float only (rigEvaluator.cpp:3570-3573; propertyMath.cpp:33-61). Approximation: a RotationConstraint from End IK Null toward a sibling 'mirror' control whose avars are connected to End FK, with inputs:defaultWeight = s. The envelope is per-axis asset-space Euler (solvers.cpp:780-800, 932-941), not a local fractional rotation.

**Gap.** No way to scale a double avar or to apply a fraction of another control's local rotation in local space. The backwards reset is a matching action.

**Porting impact.** Distribute Rotation (default 0) is available only approximately. Shots that key it get slightly different chest pivoting.

**Recommendation.** Use RigExecDoubleMathMover (G5-foot-roll-bank) so End IK Null avars:rx = multiply(connected End FK avars:rx, s), with matched rotation orders. Alternatively use the proposed RigExecSwingTwistConstraint with a parent-local reference and float inputs:swingWeight.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:560-566`; `libs/rigExec/rigEvaluator.cpp:3570-3573`; `libs/rigExecMath/propertyMath.cpp:33-61`; `libs/rigExecMath/solvers.cpp:780-800`; `libs/rigExecMath/solvers.cpp:873-942`

**Verification (holds).** Verdict and severity hold.

- Double-to-double avar connections exist in shipped, baking examples (Biped.usda:4743-4745; examples/11_VolumeWeights.usda:122).
- Nothing can scale them: FloatMathMover targets float only (rigEvaluator.cpp:3570-3573), and MatrixMathMover has no matrix source for another control's avars.
- The RotationConstraint-toward-mirror approximation (per-axis asset-space Euler, solvers.cpp:780-800) is the available path.
- The default of 0 limits the impact.

Verifier evidence: `examples/biped/Biped.usda:4743-4745`; `examples/11_VolumeWeights.usda:122`; `libs/rigExec/rigEvaluator.cpp:560-566`; `libs/rigExec/rigEvaluator.cpp:3570-3573`; `libs/rigExecMath/solvers.cpp:780-800`

### G5-spine-mid-blend

**Mid IK null blend between Start Driver and End IK ('Mid Blend' dial)**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D7

UE features: `UE2-spine-mid-blend`

**UE rigs.** ParentConstraint (maintain offset, Average interpolation) of the Mid IK Null to [Start Driver, End IK] with weights:
- w_start = clamp(remap(b, 0.5..1 → 1..0))
- w_end = clamp(remap(b, 0..0.5 → 0..1))

b = the 'Mid Blend' channel (default 0.65, so weights (0.7, 1)). The Mid IK control rides under the null.

**usdRig today.** The biped already has the pattern: 'spine_mid_follow' is ParentConstrained to the root and end controls with sourceWeights [0.5, 0.5] and baked per-source offsets, plus an aim, and the mid control nests under it (Biped.usda:3956-3985). SplineIk also has inputs:midFollowWeight (schema.usda:872-876).

The dial remap is the problem:
- inputs:sourceWeights is a float[] that no mover writes.
- midFollowWeight is a double, which FloatMathMover cannot target (rigEvaluator.cpp:3570-3573) and a float dial cannot connect to (rigEvaluator.cpp:560-566).

Workaround: two stacked ParentConstraints (start at weight 1, then end with envelope w_end/(w_start+w_end)). A FloatMathMover chain computes the envelope; remap with a connected max performs the division.

**Gap.** Missing pieces:
- no dial-driven per-source weights
- no float→double drive of midFollowWeight
- rotation averaging is per-axis Euler instead of quaternion Average
- offsets must be baked

**Porting impact.** The Mid Blend dial works only through a clunky FloatMathMover chain. Rotations of the mid null differ slightly from UE at intermediate weights.

**Recommendation.** Add float inputs:midBlend to RigExecSplineIk, implementing UE's two-sided remap and normalization for the follow point in splineIk.cpp and computations.cpp. Relax _ValidateScalarConnection (rigEvaluator.cpp:525-569) to accept float↔double scalar sources with numeric conversion (and mirror that in the bake's input classifier) so dials can drive double solver inputs.

**Evidence:** `examples/biped/Biped.usda:3956-3985`; `libs/rigExecSchema/schema.usda:872-876`; `libs/rigExecSchema/schema.usda:1009-1018`; `libs/rigExec/rigEvaluator.cpp:560-566`; `libs/rigExec/rigEvaluator.cpp:3570-3573`; `libs/rigExecMath/solvers.cpp:1066-1107`

**Verification (holds).** Verdict and severity hold.

What cannot be driven from a float dial:
- inputs:midFollowWeight is a double (schema.usda:872-876).
- inputs:sourceWeights is a float[] (schema.usda:1014-1018), and only Float, Vec3f and Matrix property movers exist (rigEvaluator.cpp:3536-3540).

The biped confirms both weights must be driven. Its follow null uses a translation-only ParentConstraint with weights [0.5, 0.5] (Biped.usda:3969-3983), and the SplineIk midFollowWeight is set to the same 0.5 (Biped.usda:5858) to avoid double-counting. A UE Mid Blend dial must therefore drive both the constraint and the double.

The two-stacked-constraint FloatMathMover workaround is valid:
- remap/clamp/add/multiply exist.
- A connected max works through resolved inputs.

Verifier evidence: `libs/rigExecSchema/schema.usda:872-876`; `libs/rigExecSchema/schema.usda:1009-1018`; `examples/biped/Biped.usda:3956-3985`; `examples/biped/Biped.usda:5854-5858`; `libs/rigExec/rigEvaluator.cpp:560-566`; `libs/rigExec/rigEvaluator.cpp:3536-3573`

### G5-spine-module-layout

**Spine/Neck module control layout (FK trio, IK trio, Sec FK chain, End IK Null)**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D7

UE features: `UE2-spine-control-layout`, `UE1-layout-spine-neck`

**UE rigs.** Spine has 21 controls and Neck has 15:
- FK trio Start/Mid/End FK is chained, with 'End IK Null' (at the spine start) inserted between Mid FK and End FK's space.
- IK trio: Start IK under Start FK (Start Driver null), End IK under End FK, Mid IK under 'Mid IK Null' under End FK.
- A Sec FK chain with orient offset 90° about Y.
- 'FkDeltaTransform' and 'Offset' metadata; 6 or 3 virtual bones; _reoriented and _match null chains.
- Channels Mid Blend, Stretch, Distribute Rotation and visibility bools.
- Neck uses Is Neck=true and has no Pelvis Local or movable pivot.

**usdRig today.** Nested RigExecControls reproduce the FK stack, including the End IK Null insertion, and the IK trio. Sec FK controls are RigExecControls. Spine and neck are two SplineIk instances over virtual spine_ik_/neck_ik_ joints that drive *_bind joints through Parent/Scale constraints, as the biped does (Biped.usda:5854-5900, 2882-2900). Offset and delta metadata become build-time rest offsets and constraint offsets. Channel hosts, limits and the Movable Pivot proxy belong to G3. The solve semantics differ; see the spine rows below.

**Gap.** The layout is expressible. The solve differences are in the rows below, and no module builder exists.

**Porting impact.** Spine and neck controls can be recreated one-to-one. Behavior parity depends on the spine solve rows.

**Recommendation.** Add python/rigexec/modules/spine.py, which generates the FK/IK/Sec FK controls, helper joints, SplineIk, the layered Sec FK (G5-spine-secfk-layer) and the skin-follow constraints from the UE config (Is Neck, orient offsets, display names).

**Evidence:** `examples/biped/Biped.usda:5854-5900`; `examples/biped/Biped.usda:2882-2900`; `libs/rigExecSchema/schema.usda:788-924`; `docs/biped-rig.md:75-83`

**Verification (holds).** Verdict and severity hold.

- Nested RigExecControls reproduce the FK/IK stack.
- The biped already uses two SplineIk instances over helper spine_ik/neck_ik joints (Biped.usda:5854-5900), with Parent and Scale constraints to the *_bind joints (Biped.usda:2885-2900).
- 'Pelvis TXY' is only a space target for root/Root (MR_Zebra runtime_hierarchy.txt:2147), which is G4.
- No spine module builder exists.

Verifier evidence: `examples/biped/Biped.usda:5854-5900`; `examples/biped/Biped.usda:2885-2900`; `libs/rigExecSchema/schema.usda:788-924`

### G5-spine-secfk-layer

**Secondary FK layer riding the spline result (per-frame SetControlOffset)**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D7

UE features: `UE2-spine-secfk-offset-layer`

**UE rigs.** Every forward frame, for each bone i:
- ReOrientedNull[i] = spline bone · Sec FKs Orient Offset.
- SetControlOffset(SecFK[i], local transform of ReOrientedNull[i]), so the Sec FK chain's offsets follow the spline while the animated locals add FK on top down the chain.
- Bones[i] = SecFK[i] · inverse(orient offset).

The last Sec FK is always visible and uses the chest-local shape.

**usdRig today.** Expressible with connected default spaces. SecFK[i] nests under SecFK[i-1]; its posed:defaultSpace connects to a relay control nested under spline helper joint i, and its parent:defaultSpace connects to the relay under helper i-1. That yields pose = avars · VJ_i · VJ_{i-1}⁻¹ · SecFK_{i-1} (computations.cpp:358-382). The relay's parent:space reads the solver-posed joint, a pattern exercised in tests/testRigExecConstraints.cpp:3117-3159. Skin joints then follow the Sec FK controls through ParentConstraints carrying the orient offset. Connected default/parent spaces refuse the baked program (bakedProgram.cpp:360-418; docs/baked-step-graph.md:837-838), so the spine forces dynamic evaluation.

**Gap.** No native 'FK layered on a solved chain' operator. The workaround needs two relay prims per bone plus matrix connections, and disables the baked program (biped: 0.71 ms baked vs 7.14 ms dynamic). Control guides may not follow solver-posed parents (docs/biped-rig.md:136-141).

**Porting impact.** The Sec FK workflow is reproducible, but the whole character falls back to dynamic evaluation, roughly a 10x slower frame, and the stage becomes cluttered with relay prims.

**Recommendation.** Add rel rigExec:baseChain (an aggregate provider such as the SplineIk) and uniform matrix4d rigExec:baseOffset to RigExecFkChain (schema.usda:418-522). The chain computes W_i = A_i · (B_i·O) · (B_{i-1}·O)⁻¹ · W_{i-1}. Implement it in RigExecSolveFkChain (solvers.cpp:16-41), computations.cpp:584-721 and the FkChain bake, and make control-guide imaging follow the layered frames.

**Evidence:** `libs/rigExec/computations.cpp:301-383`; `tests/testRigExecConstraints.cpp:3092-3248`; `libs/rigExec/rigEvaluator.cpp:10753-10871`; `libs/rigExec/bakedProgram.cpp:347-418`; `docs/baked-step-graph.md:837-838`; `docs/biped-rig.md:136-141`

**Verification (holds).** Verdict and severity hold.

The connected default/parent space pattern is tested (testRigExecConstraints.cpp:3093-3248). Its formula matches computations.cpp:337-383.

The bake refusal is confirmed:
- Authored or connected parent:defaultSpace/posed:defaultSpace, and connected default:space, all refuse (bakedProgram.cpp:355-418; docs/baked-step-graph.md:837-838).
- A constraint-only alternative cannot express the per-frame relative offset VJ_i·VJ_{i-1}⁻¹, because constraint offsets are static values.

The cost figures are confirmed (docs/plans/evaluation-engine-gaps-vs-premo-libee.md:77). This is a performance cost, not lost behaviour, so minor stands.

Verifier evidence: `tests/testRigExecConstraints.cpp:3093-3248`; `libs/rigExec/computations.cpp:337-383`; `libs/rigExec/bakedProgram.cpp:355-418`; `docs/baked-step-graph.md:837-838`; `docs/plans/evaluation-engine-gaps-vs-premo-libee.md:77`

### G5-twist-start-detwist

**LimbTwist forward order and 'Reset Twist on Start Bone' (world-referenced de-twist, no propagation)**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D7

UE features: `UE3-limbtwist-forward`

**UE rigs.** Forward order:
1. Twist Parent Null = Start global.
2. Blend Twist, then Blend Translate.
3. Twist bones = twist controls.
4. If not reversed: Start global rotation = swing(StartGlobal.rot, TwistAxis), times FromEuler(axis·180) when the axis is negative, written without propagating to children.

This removes the start bone's twist against world identity (an open question in the UE analysis). Backwards: the controls follow the bones.

**usdRig today.** The pose DAG makes a solver wait on constraints that target any of its inputs (rigEvaluator.cpp:5926-5943). A de-twist written on the same joint that feeds TwistDistribution would therefore run BEFORE the distribution, the opposite of UE. The port needs separate driver and skin joints, as in the biped's shoulderNoTwist aim plus twist_driver copy (Biped.usda:2363-2385, 2128-2150). World-referenced de-twist is a stack on the skin joint:
- RotationConstraint to an identity-oriented world locator;
- AimConstraint with worldUpType none toward the child joint (minimum swing, solvers.cpp:1142-1153);
- rotationOffset (180,0,0) for negative axes (an Euler add is a local pre-rotation in XYZ order).

'No propagation' only holds where descendants own their pose (rigEvaluator.cpp:10629-10704).

**Gap.** Missing pieces:
- no swing-twist constraint with a selectable reference (world, rest or parent) and weights
- no per-constraint 'do not propagate' flag
- ordering must be solved with duplicate driver joints
- the backwards solve is a matching feature

**Porting impact.** Reproducible with an extra driver joint per twisted segment and a 2-3 constraint stack. The biped's parent-relative noTwist differs from UE's world reference, so the start-bone orientation differs unless the world stack is used.

**Recommendation.** Add RigExecSwingTwistConstraint (schema.usda after RigExecAimConstraint) with:
- double3 inputs:twistAxis
- float inputs:twistWeight and inputs:swingWeight
- uniform token rigExec:reference = world|rest|parent|object, plus rel rigExec:referenceObject
- uniform bool rigExec:propagate

Put the kernel in solvers.cpp, reusing _SwingTwist (solvers.cpp:272-290); add it to the rigEvaluator.cpp constraint table and dispatch, bakedPose.cpp and a rigBuilder.h handle. It also serves Distribute Rotation and exact foot-rocker reads.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:5926-5943`; `libs/rigExec/rigEvaluator.cpp:10629-10704`; `libs/rigExecMath/solvers.cpp:1111-1216`; `examples/biped/Biped.usda:2363-2410`; `libs/rigExecSchema/schema.usda:983-996`

**Verification (holds).** Verdict and severity hold.

- The pose DAG makes solvers wait on constraints that target their input chains (rigEvaluator.cpp:5926-5943), so the ordering inversion is real.
- The biped uses separate noTwist/twist_driver joints (Biped.usda:2128-2150, 2363-2410).
- AimConstraint worldUpType none gives the minimum swing (solvers.cpp:1142-1153).
- No swing-twist node exists (R1-swing-twist-internal).

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:5926-5943`; `libs/rigExec/rigEvaluator.cpp:10629-10704`; `libs/rigExecMath/solvers.cpp:1112-1216`; `examples/biped/Biped.usda:2128-2150`; `examples/biped/Biped.usda:2363-2410`

### G5-ik-plane-helper-chain

**IK-plane virtual helper bones (mid projected to IK plane, planar A/B/C chain)**

**Verdict:** Divergent-by-design · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D6

UE features: `UE4-project-mid-to-ik-plane`, `UE4-ik-plane-virtual-bones-v02`, `UE3-ik-plane-virtual-bones`

**UE rigs.** At construction, from initial transforms: sec = the mid joint's secondary axis in world space, n = cross(sec, E-S), X = IntersectPlane(M, n, S, n), newMid = S + (X-S)*|M-S|/|X-S|. Spawns hidden bones Virtual Bone A/B/C under parent(Start): AimBoneMath with primary Axis down the chain and Z along the plane normal; C keeps End's own orientation and gets End Bone Rotation Offset on its initial local. The 2-bone solve and FK/IK matching run on the helpers. Skin bones then take the VB transforms with rotation * inverse(FK Rotation Offset).

**usdRig today.** usdRig has no construction event and never authors prims during evaluation (docs/spec.md:56-63; README.md:20-23). The equivalent is build-time:
- Tooling computes the planar positions and frames once.
- It authors unskinned RigExecJoint helpers with parent-relative rest:space.
- TwoBoneIk, FkChain and BlendPointFrames bind the helpers through rigExec:joints.
- Skin joints follow the helpers through ParentConstraints with baked per-source offsets, exactly as the biped's spine_ik_* joints drive *_bind joints.

TwoBoneIk re-measures bone lengths from the helper rests on every evaluation, so a rest edit needs no recompile.

**Gap.** No in-repo helper implements Project Middle Bone to IK Plane (including the IntersectPlane zero-guard) or the A/B/C frame construction. The biped's limb re-frame tooling (tools/biped/limb_frames.py) is referenced but not in the checkout. The unused, buggy 'Same As Source' output has no counterpart and needs none.

**Porting impact.** Helper joint rests must be precomputed offline for all 4 Zebra limbs and regenerated whenever the skeleton rest changes; UE recomputes them at every construction.

**Recommendation.** Add python/rigexec/limbs.py with project_mid_to_ik_plane(s, m, e, secondary_axis) and ik_plane_frames(s, m, e, primary_axis, secondary_axis, end_rotation_offset), mirroring the CRFL math. Add a builder helper, RigExecRigBuilder::AddIkPlaneHelperJoints(parent, start, mid, end, axes, offset), in libs/rigExecRigging/rigBuilder.h/.cpp. It authors the three RigExecJoint rests plus the skin-follow ParentConstraints with computed translationOffsets and rotationOffsets.

**Evidence:** `libs/rigExecSchema/schema.usda:292-300`; `libs/rigExec/computations.cpp:262-290`; `libs/rigExec/computations.cpp:805-822`; `examples/biped/Biped.usda:2882-2900`; `docs/spec.md:56-63`; `docs/biped-rig.md:296-306`

**Verification (holds).** Verdict and severity hold.

Non-goals confirmed: no dynamic topology and no authored values during evaluation (docs/spec.md:58), and non-destructive evaluation (README.md:22-23).

Rest-only helper chain confirmed:
- Helper rests are parent-relative (computations.cpp:269-290).
- TwoBoneIk re-measures bone lengths from the bound joints' rests on every evaluation (computations.cpp:808-822; docs/spec.md:242).
- A build-time helper chain is therefore equivalent for a fixed skeleton.

Tooling confirmed missing:
- tools/ has no biped/ directory, although docs/biped-rig.md:299 lists tools/biped/limb_frames.py.
- python/rigexec contains only __init__.py, bake.py, curvenet.py and inverse.py.

UE re-check:
- The forward 'Snap Actual skeleton to Virtual Bones' is ProjectTransformToNewParent(Bones[i], VB[i] initial, VB[i] current), an offset-preserving follow against bind (IkFk2Bones graphs.txt:337-345). The analyst's 'ParentConstraints with baked per-source offsets' is therefore the right mapping.
- The ue_summary misreads one step. 'rotation * inverse(FK Rotation Offset)' is written from FK controls/Compute FK into the Virtual Bones (graphs.txt:164, 366-373, 502), not from the VBs into the skin bones.

Verifier evidence: `docs/spec.md:56-63`; `README.md:22-23`; `libs/rigExec/computations.cpp:269-290`; `libs/rigExec/computations.cpp:808-822`; `docs/spec.md:242`; `docs/biped-rig.md:299`; `examples/biped/Biped.usda:2885-2900`; `ue/<dump>/FortniteRigs__Modules__IkSolves__IkFk2Bones__CRM_FN_IkFk2Bones/graphs.txt:337-345`; `ue/<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:366-373`

### G5-limbtwist-setup

**LimbTwist construction: twist-bone discovery by name tokens, twist nulls and offset controls**

**Verdict:** Divergent-by-design · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D7

UE features: `UE3-limbtwist-setup`, `UE1-layout-limb-twist`

**UE rigs.** Twist bones are Start's direct children whose names contain every '|'-token of Twist Search String ('upperarm|twist' etc., in hierarchy order). The module builds 'Twist Parent Null' (under Body), then 'Twist N Null' and a 'Twist N' offset control per bone. Controls use display names 'Offset 1/2', mirror metadata and auto scale |S−E|/20. Twist Weights and Twist Reverse come from config, and the twist axis is −X on some sides. Zebra has 4 bones per segment, 8 modules and 32 controls.

**usdRig today.** There is no construction event or procedural spawning (docs/spec.md:56-63; README.md:20-23). Build tooling discovers the twist joints and authors:
- a hidden RigExecControl per twist null, driven by TwistDistribution through relays or constraints;
- a 'Twist N' RigExecControl nested under it;
- a ParentConstraint from each twist joint to its control.

Namespace nesting under constrained nulls propagates the offset layer (rigEvaluator.cpp:10661-10751). Display names, mirror metadata and scale belong to G3.

**Gap.** No in-repo helper for token-based joint discovery or twist-module authoring. The UE Connector event also references stale connectors, which do not need porting.

**Porting impact.** The 32 Zebra twist controls must be generated offline, and re-generated when the skeleton changes.

**Recommendation.** Add python/rigexec/modules/twist.py with find_children_by_tokens(joint, 'upperarm|twist'), plus an author_limb_twist(cfg) that writes the TwistDistribution (with the extensions from G5-twist-distribution), offset controls and follow constraints.

**Evidence:** `docs/spec.md:56-63`; `libs/rigExec/rigEvaluator.cpp:10661-10751`; `libs/rigExecRigging/rigBuilder.h:242-257`; `libs/rigExecSchema/schema.usda:654-718`

**Verification (holds).** Verdict and severity hold.

- There is no construction event, and nothing is authored during evaluation (spec.md:58; README.md:22-23).
- There are no twist or discovery helpers in python/rigexec or rigBuilder; only the TwistDistribution handle exists (rigBuilder.h:242-257).
- Twist controls should be driven by constraints (constraint commits propagate to descendants, rigEvaluator.cpp:10661-10751), not by nesting under solver-posed joints.

Verifier evidence: `docs/spec.md:56-63`; `README.md:22-23`; `libs/rigExecRigging/rigBuilder.h:242-257`; `libs/rigExec/rigEvaluator.cpp:10661-10751`

### G5-foot-ball-toe

**Foot/ball placement: 2-joint FABRIK, toe aim with maintained offset, Toes IK/FK**

**Verdict:** Implemented · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D8

UE features: `UE3-foot-ball-toe-solve`

**UE rigs.** IK mode, after the leg solve:
1. FABRIKItemArray([Foot, Ball], effector = Ball IK null, precision 1e-4, 10 iterations, no effector set) rotates the foot so the ball reaches the target.
2. AimConstraintLocalSpaceOffset(Ball → Toe IK null, maintain offset, aim X, up Z, world-up point (0,0,1) in Toe IK null space).
3. Toes IK Null follows the Ball.
4. Ball = Toes IK control global.

FK mode: Ball = Toes FK control. Optional FK toe controls drive the toe joints.

**usdRig today.** A one-segment FABRIK is an aim. The foot uses an AimConstraint (source = Ball IK control, aimVector = rest foot→ball direction in foot-local space, worldUpType none = minimum swing, solvers.cpp:1142-1153); the biped does exactly this for ankle and ball (Biped.usda:3048-3074). The toe aim is an AimConstraint whose aim and up vectors are computed at build time so no offset is needed, with worldUpType objectUp using a locator child of the Toe IK null at (0,0,1). Toes IK Null and ball follow are ParentConstraints. The IK/FK choice is the constraint envelope connected to the limb blend weight (Biped.usda:3055, 3073). Toe joints use an FkChain with rigExec:startFrame (schema.usda:468-499).

**Gap.** Aim vectors and offsets must be computed by tooling (no maintain-offset). The IK/FK envelope blends Euler per axis instead of slerp.

**Porting impact.** Equivalent at weights 0 and 1; minor differences at intermediate IK/FK weights.

**Recommendation.** The foot builder computes the aim and up vectors from the rest pose and authors the constraints. Optionally add rigExec:envelopeBlend = slerp to constraints (see G5-auto-pv-space).

**Evidence:** `examples/biped/Biped.usda:3048-3074`; `libs/rigExecMath/solvers.cpp:1111-1216`; `libs/rigExecSchema/schema.usda:1021-1071`; `libs/rigExecSchema/schema.usda:468-499`

**Verification (holds).** Verdict and severity hold.

Foot aim:
- A 2-joint FABRIK is a minimum-swing aim, and AimConstraint worldUpType none gives exactly that (solvers.cpp:1142-1153). Children follow by propagation.

Toe aim with UE's maintained offset:
- Precomputed aimVector/upVector are equivalent, because the maintain offset is a constant local rotation.
- upVector is projected perpendicular to aimVector (solvers.cpp:1163-1170).
- objectUp toward a locator (rigEvaluator.cpp:11334-11352) reproduces the (0,0,1)-location world-up.

The envelope's Euler blending is irrelevant, because UE's Ik Fk Switch is a bool.

Verifier evidence: `libs/rigExecMath/solvers.cpp:1112-1216`; `libs/rigExec/rigEvaluator.cpp:11321-11360`; `examples/biped/Biped.usda:3048-3074`; `libs/rigExecSchema/schema.usda:468-499`

### G5-foot-pivot-stack

**Reverse-foot pivot null stack, footprint space and leg-effector hand-off**

**Verdict:** Implemented · **Severity:** minor · **Effort:** S · **Confidence:** medium · **Domain:** D8

UE features: `UE3-foot-pivot-hierarchy`, `UE1-layout-foot`

**UE rigs.** Construction: Footprint Space = (leg IK rotation, (Ball.x, Ball.y, 0)). Null chain under the leg IK gimbal:
IK Foot Space > Toe Tip Pivot > Heel Pivot > Ball Pivot > Toe Tip Rocker > Heel Rocker > Inner > Outer > {Heel Lift > IK, Ball IK, Toe IK}.
Each pivot sits at a connected socket, or at footprint·config offset (Zebra offsets are given). The foot's 'IK' null is the leg's Target Effector. The foot's Pre-Forwards runs before the leg's Forwards.

**usdRig today.** Nested RigExecControls (guides hidden) under the leg IK control reproduce the stack. The biped nests leg_l_ik/ankle_l_pivot/bankIn/bankOut/heel/ball/toe the same way and feeds solver and aim sources from its innermost prims (Biped.usda:3048-3074). Pivot rests are computed at build time from sockets or footprint offsets. The leg TwoBoneIk effectorControl is the innermost 'IK' control. Namespace propagation carries the stack (rigEvaluator.cpp:10661-10751). The foot-before-leg ordering falls out of the pose DAG, because the IK reads the effector chain (rigEvaluator.cpp:5926-5943).

**Gap.** Footprint and pivot placement is build-time only, with no construction event.

**Porting impact.** Equivalent once authored. Re-run tooling when sockets or config offsets change.

**Recommendation.** Add python/rigexec/modules/foot.py, which computes the footprint transform and pivot rests (socket or offset) and authors the nested pivot controls.

**Evidence:** `examples/biped/Biped.usda:3048-3074`; `libs/rigExec/rigEvaluator.cpp:10661-10751`; `libs/rigExec/rigEvaluator.cpp:5926-5943`; `libs/rigExecSchema/schema.usda:547-549`

**Verification (holds).** Verdict and severity hold.

- The biped nests its pivot chain under leg_l_ik and feeds its aim sources from the innermost prims (Biped.usda:3064-3074).
- The leg IK waits on constraints that rotate ancestors of its effector (rigEvaluator.cpp:5926-5943), which reproduces UE's foot-before-leg Pre Forwards order.

Verifier evidence: `examples/biped/Biped.usda:3048-3074`; `libs/rigExec/rigEvaluator.cpp:5926-5943`; `libs/rigExec/rigEvaluator.cpp:10661-10751`

### G5-free-pivot

**Rotate Around Free Pivot (translate control moves pivot only; rotation pivots about it)**

**Verdict:** Implemented · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D8 · *(analyst said Partial / minor)*

UE features: `UE3-free-pivot`

**UE rigs.** RAFP(Input, Pivot = control local (t_p, R_p)) = [x → R_p(x − t_p) + t_p] composed with Input. Translating the Toe Tip, Heel or Ball control moves only the pivot point, with no motion; rotating it rotates the pivot null about the translated position, in the null's initial-local frame.

**usdRig today.** There is no pivot channel: _ComposeAvars applies scale, ordered rotations, rspin and translation about the local origin only (computations.cpp:192-222). Exact emulation per pivot uses three helper prims:
- a translation-follower control (a PositionConstraint to the pivot control);
- a relay nested under it;
- the pivot null nested under the pivot control, with parent:defaultSpace connected to the relay's parent:space.

That gives N = N_d · T(−Δ) · (D⁻¹ R D) · T(Δ) (computations.cpp:358-382; connection pattern in tests/testRigExecConstraints.cpp:3117-3159). Connected spaces make it dynamic-only (bakedProgram.cpp:360-374).

**Gap.** No native animatable pivot (rotate-pivot) channel on RigExecXformable. The workaround costs three extra prims per pivot and disables the bake.

**Porting impact.** The 3 per-foot pivot controls (6 total) work only through the verbose relay workaround, which forces dynamic evaluation of the whole rig.

**Recommendation.** Add double avars:pivotX, avars:pivotY and avars:pivotZ (animatable and connectable) to RigExecXformable (schema.usda:327-344). Compose them as S · T(−p) · R · T(p) · T(t) in _ComposeAvars (computations.cpp:192-222) and in the baked ladder (bakedPose.cpp), and show the pivot in the usdview gizmo (plugin/rigExecUsdview). A UE pivot null then connects its avars:r* to the control's avars:r* and its avars:pivot* to the control's avars:t*; double→double connections already bake.

**Evidence:** `libs/rigExec/computations.cpp:192-222`; `libs/rigExec/computations.cpp:336-383`; `tests/testRigExecConstraints.cpp:3117-3159`; `libs/rigExec/bakedProgram.cpp:360-374`; `libs/rigExecSchema/schema.usda:327-344`

**Verification (corrected).** An exact, bakeable construction exists without connected spaces.

Construction: three nested hidden nulls P1 > P2 > P3 at the pivot null's rest.
- P1 avars:tx/ty/tz and P2 avars:rx/ry/rz are connected double-to-double to the pivot control's avars. P2 uses the same avars:rotationOrder as the control.
- P3's avars:t are connected to the same control translations, with avars:unitScaleFactor = -1.

The maths:
- Row-vector composition (computations.cpp:193-222) gives the local map T(-t)·R·T(t).
- For a child point x that is x -> R(x-t)+t, which is UE's RAFP.
- The rest of the pivot stack nests under P3.

Why it works and bakes:
- unitScaleFactor is a plain multiplier with no sign validation, in both the dynamic path (computations.cpp:364-369) and the baked path (bakedPose.cpp:2417-2429).
- Avar connections resolve per frame and bake: bakedProgramImpl.h:359-362 lists them, and the baking biped uses them (Biped.usda:4743-4745).
- The analyst's version with connected spaces is what forces dynamic evaluation (bakedProgram.cpp:355-418); this version avoids it.

Caveat: a negative unitScaleFactor is untested. The avars:pivot* recommendation remains a good ergonomic improvement.

Verifier evidence: `libs/rigExec/computations.cpp:193-222`; `libs/rigExec/computations.cpp:364-383`; `libs/rigExec/bakedPose.cpp:2417-2429`; `libs/rigExec/bakedProgramImpl.h:359-362`; `libs/rigExecSchema/schema.usda:327-344`; `examples/biped/Biped.usda:4743-4745`; `libs/rigExec/bakedProgram.cpp:355-418`

### G5-ik-base-parent-buffer

**IK Base root control on a scale-free Parent Buffer**

**Verdict:** Implemented · **Severity:** minor · **Effort:** S · **Confidence:** medium · **Domain:** D6

UE features: `UE3-ik-base-control`

**UE rigs.** 'Parent Buffer' null follows the limb parent every frame through ProjectTransformToNewParent, writing rotation and translation only (scale 1), so the limb can hang off a scaled bone. The 'IK Base' control underneath is made translate-only by rotation/scale limits (min = max). In IK mode Virtual Bone A's translation = IK Base translation. It follows VB[0] in backwards/match.

**usdRig today.** TwoBoneIk reads only the root control's origin (solvers.cpp:105; computations.cpp:732-739), so a RigExecControl used as rigExec:rootControl gives a translate-only IK root no matter how it is rotated. The Parent Buffer is a hidden RigExecControl targeted by a RigExecParentConstraint (sources = the parent joint, baked per-source offsets). Its scale mask defaults off (schema.usda:1132-1134; solvers.cpp:1093-1103), so the buffer ignores parent scale, and IK Base nests under it.

**Gap.** Remaining differences:
- The rotation/scale lock is not enforced because control limits do not exist (schema.usda:33-47; tracked in G3).
- Constraint offsets must be baked by tooling (no maintain-offset capture).
- Following VB[0] in the backwards solve is a matching feature.

**Porting impact.** The functional result is equivalent. Animators can rotate IK Base without effect instead of being blocked.

**Recommendation.** No new operator is needed. Control limits (G3) would restore the lock. The builder should author the ParentConstraint with computed offsets.

**Evidence:** `libs/rigExecMath/solvers.cpp:105-107`; `libs/rigExec/computations.cpp:732-739`; `libs/rigExecSchema/schema.usda:1115-1142`; `libs/rigExecMath/solvers.cpp:1093-1108`; `libs/rigExecSchema/schema.usda:33-47`

**Verification (holds).** Verdict and severity hold.

What the solve reads:
- TwoBoneIk uses only the root control's origin (solvers.cpp:105).
- The root control's X is read only in the coincident-goal fallback (solvers.cpp:123).

Parent Buffer mapping:
- ParentConstraint defaults to scale off (schema.usda:1128-1136).
- Its offset is applied before the source (solvers.cpp:1047-1064).
- UE spawns the buffer at the parent's global transform, so the offset is identity and parent scale cannot leak into the translation.

Ordering: the pose DAG runs a solver after constraints that target any ancestor of its inputs (rigEvaluator.cpp:5926-5943), so IK Base nested under a constrained buffer is ordered correctly.

Only the rotation/scale lock is missing (schema.usda:33-47, G3).

Verifier evidence: `libs/rigExecMath/solvers.cpp:105-133`; `libs/rigExec/computations.cpp:732-739`; `libs/rigExecSchema/schema.usda:1115-1142`; `libs/rigExecMath/solvers.cpp:1041-1108`; `libs/rigExec/rigEvaluator.cpp:5926-5943`

### G5-twist-translate

**LimbTwist translate distribution (Compute Translate Weights + Blend Translate); library Blend Position**

**Verdict:** Implemented · **Severity:** minor · **Effort:** S · **Confidence:** medium · **Domain:** D7

UE features: `UE8-limbtwist-translate-weights`, `UE4-blend-position`

**UE rigs.** At construction, w_i = |P_i − P_end| / |P_start − P_end| from initial globals. Every frame, PositionConstraintLocalSpaceOffset(child = twist null i, parents [Start w_i, End 1−w_i], maintain offset, all axes), so twist bones keep their rest ratio along the segment while the joints move or stretch. The library Blend Position does the same with caller weights and is unused.

**usdRig today.** Use a RigExecParentConstraint with rotation masked off (affectRotationX/Y/Z = false; scale is off by default), sources [start joint, end joint], sourceWeights [w_i, 1−w_i], and per-source translationOffsets expressed in source-local space (solvers.cpp:1047-1064). That reproduces a maintain-offset local-space translation blend. The weights are build-time constants. A PositionConstraint is not equivalent, because its translationOffset is added in asset space (solvers.cpp:839).

**Gap.** Offsets and weights must be baked by tooling (no maintain-offset capture), and the constraint must be ordered after the twist rotation writer.

**Porting impact.** Equivalent, apart from build-time authoring.

**Recommendation.** The twist module builder computes w_i and the local offsets and authors these constraints. Alternatively, rigExec:placement = boundJointRest on TwistDistribution (G5-twist-distribution) removes the need for them.

**Evidence:** `libs/rigExecMath/solvers.cpp:1041-1103`; `libs/rigExecMath/solvers.cpp:821-839`; `libs/rigExecSchema/schema.usda:1115-1142`; `libs/rigExecSchema/schema.usda:962-978`

**Verification (holds).** Verdict and severity hold; one authoring detail differs.

UE's local-space offset is expressed in the child's parent frame, i.e. the Twist Parent Null, which equals Start global (RigUnit_TransformConstraint.cpp:855-869, 894-911). usdRig ParentConstraint offsets are source-local, and sources are averaged (solvers.cpp:1047-1064).

To match UE exactly, put the whole offset on the Start source scaled by 1/w_i and use zero on End. For twist bones collinear with the segment the offset is 0 anyway.

Rotation masks are honoured (solvers.cpp:1104). The analyst is right that PositionConstraint adds its offset in asset space (solvers.cpp:839).

Verifier evidence: `libs/rigExecMath/solvers.cpp:998-1108`; `libs/rigExecMath/solvers.cpp:839`; `libs/rigExecSchema/schema.usda:1115-1142`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Highlevel/Hierarchy/RigUnit_TransformConstraint.cpp:855-935`

### G5-foot-pivot-sockets

**Foot pivot sockets (foot_<side>_inner/outer/heel/toe_tip) on the ball bones**

**Verdict:** Implemented · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D8

UE features: `UE-foot-pivot-sockets`, `UE8-foot-pivot-socket-values`

**UE rigs.** Eight skeleton sockets relative to ball_l/r are imported by modular rigs as static MeshSocket-tagged nulls and connected to the Foot pivot connectors. Examples: foot_l_heel (−18.8, 1, −1) vs foot_r_heel (18, −1, 1), which are asymmetric; toe_tip sits at (0,0,0), on the ball. Monster has no sockets, and the Biped leaves them unconnected.

**usdRig today.** A socket is a prim nested under the ball joint:
- either a RigExecControl or RigExecJoint (purpose guide) whose parent-relative rest:space is the socket offset (computations.cpp:268-290),
- or a plain Xform. Plain Xforms between providers are composed, but xformOps on RigExec prims are not an authority (schema.usda:283-289).

The values, including the heel asymmetry, are copied as data, and the 'MeshSocket' tag goes in customData.

**Gap.** No dedicated socket element or tag semantics.

**Porting impact.** None beyond authoring. Keep the authored asymmetry for parity.

**Recommendation.** The foot builder authors socket locators as hidden RigExecControls under ball_l/r, with customData tags.

**Evidence:** `libs/rigExecSchema/schema.usda:283-300`; `libs/rigExec/computations.cpp:262-290`; `libs/rigExecSchema/schema.usda:360-405`

**Verification (holds).** Verdict and severity hold. Sockets are only construction-time position sources, so build-time data is equivalent. Rests are parent-relative (computations.cpp:269-290), and xformOps are not an authority (schema.usda:283-289). The catalog notes there is no socket or tag schema, which is cosmetic.

Verifier evidence: `libs/rigExecSchema/schema.usda:283-300`; `libs/rigExec/computations.cpp:269-290`

### G5-spine-pelvis-local

**Pelvis Local control (pelvis-only rotation without carrying the spine)**

**Verdict:** Implemented · **Severity:** cosmetic · **Effort:** S · **Confidence:** medium · **Domain:** D7

UE features: `UE2-spine-pelvis-local`

**UE rigs.** 'Pelvis Local' sits under Pelvis Sec FK. Forward: the pelvis bone = Pelvis Local · inverse(orient offset), then spine_01 is re-imposed from Sec FK 1, so pelvis-local rotation does not carry the spine. The shape is swapped to the pink Local FK shape. Backwards resets it to identity.

**usdRig today.** Pelvis Local is a RigExecControl nested under the Pelvis Sec FK control, and a ParentConstraint (rotationOffsets = inverse orient offset) drives the pelvis joint. A later ParentConstraint re-imposes spine_01 from Sec FK 1. Constraint commits propagate to non-owned descendants, and later movers overwrite them (rigEvaluator.cpp:10661-10751; mover order per docs/biped-rig.md:220-223).

**Gap.** Only mover ordering must be authored correctly. The shape color and the reset-on-backwards behavior belong to G3 and matching.

**Porting impact.** Equivalent result.

**Recommendation.** Author it in the spine builder with explicit mover ordering.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:10661-10751`; `docs/biped-rig.md:220-223`; `libs/rigExecSchema/schema.usda:1115-1142`

**Verification (holds).** Verdict and severity hold.

Why the re-impose works:
- Constraint commits propagate deltas to non-owned descendants (rigEvaluator.cpp:10661-10751).
- A later ParentConstraint re-imposing spine_01 re-derives spine_02..05 by the correcting delta, which restores them.

Leg roots follow as in UE:
- Leg helper joints are solver-owned and follow through the constraint-driven Parent Buffer.
- The pose DAG orders them after the pelvis constraint (rigEvaluator.cpp:5926-5943).

Only mover ordering must be authored correctly (docs/biped-rig.md:218-222).

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:10661-10751`; `libs/rigExec/rigEvaluator.cpp:5926-5943`; `docs/biped-rig.md:196-222`

### G5-spine-sliding-proxy

**Sliding proxy pivot and position-at-spline-parameter (defined, unused)**

**Verdict:** Not-applicable · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D7

UE features: `UE8-spine-sliding-proxy-unused`

**UE rigs.** Construct/Forward Sliding Proxy and RigUnit_PositionFromControlRigSpline (Spline.PositionAtParam(U), with U from a 'Proxy Pivot Slide' channel) are defined in CRM_FN_Spine but never referenced. The design relies on IsInteracting-driven buffer state.

**usdRig today.** Unused, so there is nothing to port. There is no usdRig operator for 'position on the solver curve at parameter U': SplineIk publishes only joint frames (computations.cpp:1178-1187), and Ribbon samples a native curve at fixed counts. The IsInteracting-driven state is hidden state (docs/spec.md:59).

**Gap.** None for porting. A spline-sample output would be needed only if the feature were revived.

**Porting impact.** None.

**Recommendation.** Skip. If needed later, add float[] inputs:sampleParams to RigExecSplineIk to publish extra frame elements at those parameters.

**Evidence:** `libs/rigExec/computations.cpp:1178-1187`; `libs/rigExecSchema/schema.usda:720-786`; `docs/spec.md:56-63`

**Verification (holds).** Verdict and severity hold. The feature is unused (no FUNC references), and its IsInteracting state is hidden state (spec.md:59). SplineIk publishes joint frames only (computations.cpp:1178-1187).

Verifier evidence: `docs/spec.md:56-63`; `libs/rigExec/computations.cpp:1171-1187`

### G5-unused-ik-helpers

**Defined-but-unused IK/PV/twist library and module helpers**

**Verdict:** Not-applicable · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D6

UE features: `UE4-ik-plane-virtual-bones-v01`, `UE4-pv-location-v01`, `UE4-pv-from-plane-v01`, `UE4-auto-pole-vector-v02`, `UE4-math-compute-pole-vector`, `UE8-unused-module-helpers`

**UE rigs.** None of these have callers in the shipped rigs:
- Create IK Plane Virtual Bones v01.
- Compute PV Location v01 (slerped up offset).
- Compute PV From Plane v01, which has a debug draw left enabled.
- Construct/Compute Auto Pole Vector v02.
- CRFL_Math Compute Pole Vector, whose rotation is buggy because it dots positions.
- Root 'Snap Global Control'.
- IkFk2Bones Select Control Shape, Compute Twist Sockets, Compute Mid Bone Transform For IK Plane Align, and Offset Rotation On Transforms.

**usdRig today.** Nothing to port because nothing calls them. If ever needed:
- The PV and plane math belongs in python/rigexec/limbs.py.
- Auto PV v02 maps to the constraint stack in G5-auto-pv-space.
- Snap Global Control is a stateful latch, which the spec rejects (docs/spec.md:1050).
- Select Control Shape is G3 presentation.

**Gap.** None for porting. The buggy CRFL_Math rotation should not be reproduced.

**Porting impact.** None.

**Recommendation.** Skip. Implement them only as build-time helpers if a future module uses them.

**Evidence:** `docs/spec.md:56-63`; `docs/spec.md:1050`

**Verification (holds).** Verdict and severity hold.

- The feature data records 'No callers' for all of them.
- IkFk2Bones functions_used excludes Select Control Shape, Compute Twist Sockets, Compute Mid Bone Transform For IK Plane Align and Offset Rotation On Transforms. The graphs are defined at graphs.txt:2136-2665 and never referenced.
- Snap Global Control is a one-shot latch. docs/spec.md:1050 rejects hidden previous-frame state and spec.md:59 rejects hidden stateful computation.

Verifier evidence: `docs/spec.md:56-63`; `docs/spec.md:1050`; `ue/<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:2136-2665`

## G6 — Pose readers, correctives, shapes and face logic

usdRig already covers the output end of the UE corrective pipeline. It has sparse UsdSkelBlendShape samples, independently composed RigExecBlendInput channels, and connection-driven weights. Its pre-exec FloatMathMover chains already reproduce add, multiply, clamp, remap and min (via a connected max) on animator float channels. With different authoring, the morph inventories, curve-to-morph mapping, Expression Shape Logic, eye convergence and deformer-factor remaps port faithfully.

The input end of the pipeline is missing. There is no spherical or cone pose reader: a single-pose linear RigExecPoseInterpolator is only a circular approximation, with no ellipses, half-planes, active core or explicit reference parent. Pose-derived scalars are also published only after the whole pose walk. They can therefore reach BlendInput weights, but they cannot be thresholded, min'd or multiplied, and they cannot move joints in the same evaluation.

The largest single blocker is the lack of an operator for scalar-weighted additive local transform offsets (UE ModifyTransforms AdditiveLocal). CR_Zebra_Deform uses it 82 times and the face modules about 60 times, for blink and lid layers, soft eyes, lip roll/puff/CH, the jaw-open lip pull, brow micro helpers and the lip tweaker deltas.

The second face blocker is typing. Control avars are double while every scalar-logic input is float, so pad and slider translations and rotations cannot feed the Corner, Brow, Lid, Squeeze or Squash logic.

Smaller gaps:
- BlendInput clamps weights to [0, lastActivation], which loses the signed Squetch and squash shapes and the 1..2 overdrive.
- Scalar results are never published to materials or primvars (pupil and cornea).
- Controls have no limits and a channel cannot have more than one host.
- Constraint masks are asset-space only.
- Construction-time spawning becomes converter-authored static prims.
- UE's one-frame Jaw Normalize lag becomes same-frame evaluation, by design.

Recommended additions, in priority order:
1. RigExecTransformOffsetMover.
2. A double-to-float channel bridge or channel reader.
3. RigExecSphericalPoseReader and RigExecTwistReader, scheduled as a new pose-DAG step kind, together with 'late' property chains and remapRange/min/max ops.
4. BlendInput weightRange and combination modes.
5. Primvar publication of scalar results.

With these, both deform rigs and both face modules become expressible within usdRig's non-destructive, static-topology architecture.

| Row | Verdict | Severity | Effort | Summary |
|---|---|---|---|---|
| [`G6-control-channel-read`](#g6-control-channel-read) | Partial | blocker | M | No double avar to float bridge; a RigExecControl-driven interpolator reads rotation post-walk only, translation unread |
| [`G6-lid-layers`](#g6-lid-layers) | Partial | blocker | M | Layers map to self-sourced ParentConstraints per bone; Euler blend not slerp, quat offsets need Euler, no slider read |
| [`G6-jaw-open-logic`](#g6-jaw-open-logic) | Missing | major | M | Blocked: jaw value B is pose-derived and cannot feed later movers until the stack is unified (report 6.1) |
| [`G6-monster-trap`](#g6-monster-trap) | Missing | major | S | No path: offset weight is a clavicle pose measurement and nothing publishes a scalar inside the pose walk |
| [`G6-soft-eyes`](#g6-soft-eyes) | Missing | major | M | Eye-direction weights cannot be published inside the pose walk; only a constant-weight ParentConstraint approximation |
| [`G6-additive-local-offset`](#g6-additive-local-offset) | Partial | major | L | Self-sourced RigExecParentConstraint gives stacked local offsets; Euler not slerp; w outside [0,1] skips it |
| [`G6-brow-main`](#g6-brow-main) | Partial | major | S | Diagonal frame goes in rest rotation; no avar read, Y lock or bool vis; R mirror needs rotated frame and negated gains |
| [`G6-brow-micro`](#g6-brow-micro) | Partial | major | S | Helper slides map to self-sourced ParentConstraint translationOffsets; brow curve weights need the missing control read |
| [`G6-combo-min`](#g6-combo-min) | Partial | major | M | No min op or BlendInput min mode; pose-derived values cannot feed math movers until the stack is unified |
| [`G6-corner-logic`](#g6-corner-logic) | Partial | major | S | add/multiply/clamp chains are exact, but pad translation avars can't be read; curl works only as a post-walk twist pose |
| [`G6-deform-exec-order`](#g6-deform-exec-order) | Partial | major | L | No read-pose to scalar to modify-pose slot: walk holds only solvers/constraints, chains run before, interpolators after |
| [`G6-deformer-channel-mapping`](#g6-deformer-channel-mapping) | Partial | major | S | Remap chain blocked by double avar read; signed head_twist cannot come from an interpolator; deformer absent (G7) |
| [`G6-elbow-knee-readers`](#g6-elbow-knee-readers) | Partial | major | M | Hinge approximated by linear RBF pose; pose-derived Remap(0.4..1) cannot feed pre-exec FloatMathMover chains |
| [`G6-face-correctives`](#g6-face-correctives) | Partial | major | M | Products of control curves map to multiply chains; JN-based products blocked as JN is pose-derived; weights over 1 clamp |
| [`G6-foot-values`](#g6-foot-values) | Partial | major | M | Foot control rotation avars are double and cannot feed float chains; twist interpolator is post-walk; no Euler reorder |
| [`G6-jaw-open-reader`](#g6-jaw-open-reader) | Partial | major | S | 1-pose linear RBF on the jaw can drive the jaw_open BlendInput; no in-walk reader or elliptical factors for other users |
| [`G6-lip-roll`](#g6-lip-roll) | Partial | major | M | Pre-exec chains + self-sourced ParentConstraints work; needs a clamp mover; Euler blend diverges from slerp at 70-90 deg |
| [`G6-lip-tweakers`](#g6-lip-tweakers) | Partial | major | M | 4-source ParentConstraint with sourceWeights covers nulls; no offset from a control's live local transform for tweaks |
| [`G6-shoulder-clavicle-readers`](#g6-shoulder-clavicle-readers) | Partial | major | M | Curves map to approximated RBF readers; 35 helper-bone offsets blocked as pose weights publish only after the walk |
| [`G6-spherical-pose-reader`](#g6-spherical-pose-reader) | Partial | major | L | Only a 1-pose linear swing RigExecPoseInterpolator cone; no elliptical quadrant factors, one-sided or back-pole=0 rule |
| [`G6-thigh-readers`](#g6-thigh-readers) | Partial | major | S | Pelvis is the namespace parent and cross-side wiring works; skewed thigh-in reader and pose-derived weights block parity |
| [`G6-threshold-remap`](#g6-threshold-remap) | Partial | major | L | FloatMathMover remap lacks target range/clamp and runs pre-exec, so a chain fed by a pose output reads authored 0 |
| [`G6-material-curves`](#g6-material-curves) | Missing | minor | M | Scalar rig results never reach Hydra materials: bridge forwards only points/normals/extent; movedFloats reach hosts only |
| [`G6-blend-weight-range`](#g6-blend-weight-range) | Partial | minor | S | BlendInput clamps to [0, lastActivation]; exact workaround is an activation-2 sample plus a negated-offset twin input |
| [`G6-body-corrective-morphs`](#g6-body-corrective-morphs) | Partial | minor | S | BlendShapeMover + BlendInput per corrective works, but weight clamps to [0, lastActivation], so negative Squetch is lost |
| [`G6-brow-squeeze`](#g6-brow-squeeze) | Partial | minor | M | RigExecPositionConstraint masks use asset-space axes, so X-follow matches only if skull X aligns; no limits or avar read |
| [`G6-cross-rig-curve-handoff`](#g6-cross-rig-curve-handoff) | Partial | minor | S | No scalar channel between separate RigExecRoots; face logic and deform rig must merge into one root (no multipass) |
| [`G6-curve-only-readers`](#g6-curve-only-readers) | Partial | minor | S | 1-pose linear RigExecPoseInterpolator to BlendInput works; ellipses approximated, forearm twist diverges past 90 deg |
| [`G6-curve-registry`](#g6-curve-registry) | Partial | minor | S | Any float attr is an animatable channel, but no curve registry or discovery; Avar Editor lists only avars: and foot: |
| [`G6-eye-controls`](#g6-eye-controls) | Partial | minor | M | Static controls, float avars on Eye L and AimConstraint work; no rotation-only lock/limits or multi-host shared channel |
| [`G6-face-control-hierarchy`](#g6-face-control-hierarchy) | Partial | minor | M | Static nested RigExecControls + ParentConstraints work; no guide shape offsets, limits/locks or connected-space bake |
| [`G6-morph-normal-deltas`](#g6-morph-normal-deltas) | Partial | minor | M | Pre-skin BlendShapeMover before SkinMover works, but UsdSkelBlendShape normalOffsets are ignored; normals are recomputed |
| [`G6-postprocess-input-pose`](#g6-postprocess-input-pose) | Partial | minor | M | No input-pose slot for post-process mode; solver-bound joints ignore avars, and baked helper tracks would double-apply |
| [`G6-pupil-iris`](#g6-pupil-iris) | Partial | minor | S | FloatMathMover chains reproduce both formulas exactly, but results have no consumer and no shared channel host exists |
| [`G6-twist-reader`](#g6-twist-reader) | Partial | minor | M | No node publishes twist angle/quat; a twist-type linear RigExecPose gives a windowed ramp post-walk, folding at 180 |
| [`G6-face-function-reuse`](#g6-face-function-reuse) | Divergent-by-design | minor | M | No function/loop/table construct by design; ~300 offset movers and ~150 FloatMathMovers become generated static prims |
| [`G6-deformer-factor-remap`](#g6-deformer-factor-remap) | Implemented | minor | S | (c+1)/2 and (1-c)/2 map to exact unclamped FloatMathMover add/multiply chains; the consuming G7 deformer is missing |
| [`G6-expression-shape-logic`](#g6-expression-shape-logic) | Implemented | minor | S | Maps to two FloatMathMover chains per call on float avars; weights 1-2 need the extra-sample workaround; heavy authoring |
| [`G6-eye-convergence`](#g6-eye-convergence) | Implemented | minor | S | RigExecPositionConstraint with defaultWeight connected to avars:convergence gives P+c(A-P); c must stay within [0,1] |
| [`G6-face-attach`](#g6-face-attach) | Implemented | minor | S | Maps to RigExecParentConstraint to head with identity per-source offset; offsets precomputed, author affectScale on |
| [`G6-morph-inventory`](#g6-morph-inventory) | Implemented | minor | S | Maps to per-mesh UsdSkelBlendShape + RigExecBlendSample/BlendShapeMover; no importer, and normalOffsets are ignored |
| [`G6-inert-nodes`](#g6-inert-nodes) | Not-applicable | minor | S | No engine gap; dead UE nodes can be omitted or kept as inputs:enabled=false movers, but no RigVM importer exists |
| [`G6-curve-morph-mapping`](#g6-curve-morph-mapping) | Implemented | cosmetic | S | Curves become float attrs/BlendInput weights; last-writer-wins reproduces the bugs; converter replaces name binding |

### G6-control-channel-read

**Reading control translation/rotation as scalar channels (double avar -> float logic)**

**Verdict:** Partial · **Severity:** blocker · **Effort:** M · **Confidence:** high · **Domain:** D11 · *(analyst said Missing / blocker)*

UE features: `UE6-corner-logic`, `UE6-brow-main`, `UE6-brow-micro-curves`, `UE6-brow-squeeze`, `UE6-blink-logic`, `UE6-blink-extend-open-rotate`, `UE7-deformer-control-channel-mapping`, `UE3-foot-values`

**UE rigs.** Face logic reads GetTransform(Control, LocalSpace) every frame: the translation relative to the control's offset, Euler rotation and GetControlRotator Roll, for 2D pads and sliders. It converts these to curves and weights, with 100 local units = 1.0. Nearly every face shape, lid weight and squash factor is driven this way.

**usdRig today.** A control's local value is exactly its avars (relative to its default space). But avars are double while every scalar-logic input is float:
- _ValidateScalarConnection requires identical types.
- FloatMathMover targets must be float.
- The Avar Editor code documents that float dials exist only because of this split.

No computation publishes a control's avars, or its rotation in another Euler order, as float outputs.

**Gap.** Missing: a typed bridge from double avars (and derived Euler or twist values) to float logic.

**Porting impact.** Blocker for the face control scheme. Without the bridge, every pad and slider becomes a float slider: the 2D mouth-corner pads, diagonal brow pads, lid sliders, squeeze pad and squash controls all lose their direct-manipulation workflow.

**Recommendation.** Two options:
(a) Allow float inputs to connect to double sources with explicit narrowing. This covers FloatMathMover inputs:value/min/max, MoverAPI inputs:defaultWeight, BlendInput inputs:weight and DynamicWeight inputs. Accept Double in _ValidateScalarConnection (rigEvaluator.cpp:525-569) when Float is expected, and cast in RigExecResolvedInputs::GetAttribute (moverGraph.h:319-378) and _PinnedRead (rigEvaluator.cpp:8693-8720).
(b) Add a pre-exec RigExecChannelReader: rel rigExec:source (one RigExecControl), uniform token rigExec:channel = tx|ty|tz|rx|ry|rz|eulerX|eulerY|eulerZ|twistX|twistY|twistZ, uniform token rigExec:eulerOrder, float outputs:value. It would be evaluated as the head of a property chain.

Option (a) is the smallest change. Option (b) also covers UE's Euler re-order and twist reads (Lid Rotate roll, Foot Rocker).

**Evidence:** `libs/rigExec/rigEvaluator.cpp:525-569`; `libs/rigExec/rigEvaluator.cpp:3567-3597`; `libs/rigExecSchema/schema.usda:327-336`; `plugin/rigExecUsdview/avarEditorModel.py:37-46`; `libs/rigExec/moverGraph.h:319-378`

**Verification (corrected).** The verdict is overstated: a control's rotation can already be read into a float, but only for consumers after the pose walk.
- RigExecPoseInterpolator accepts a RigExecControl driver (rigEvaluator.cpp:2478-2507) and measures its local rotation against rest (2773-2775).
- A twist or swing pose with a linear kernel publishes an exact affine remap of the angle over a window (rbf.cpp:437-447; rbf.h:147-149). For example, Lid Rotate wPos = roll/90 for roll <= 90, or curl as a pose at -100 degrees with radius 100.

That covers rotation-driven morph weights only:
- Translation is not measured (2552-2562), so pads and sliders stay unreadable.
- The output exists only after the walk (11675).
- Every scalar input is float-typed and rejects double sources (560-566, 3570-3572, 3632-3650).

Most face drivers are translation pads feeding pre-exec logic, so blocker stands.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:2478-2507`; `libs/rigExec/rigEvaluator.cpp:2552-2562`; `libs/rigExec/rigEvaluator.cpp:2773-2775`; `libs/rigExecMath/rbf.cpp:437-447`; `libs/rigExec/rigEvaluator.cpp:560-566`; `libs/rigExec/rigEvaluator.cpp:3632-3650`; `libs/rigExecSchema/schema.usda:327-336`

### G6-lid-layers

**Lid slider layers: blink, extend, open, rotate, smile push, and lid curves**

**Verdict:** Partial · **Severity:** blocker · **Effort:** M · **Confidence:** high · **Domain:** D14 · *(analyst said Missing / blocker)*

UE features: `UE6-blink-logic`, `UE6-blink-extend-open-rotate`, `UE6-lid-curves`, `UE6-smile-lid-push`

**UE rigs.** Lid Tp/Bt slider weights:
- blink: w = remap(tz, 0..200 -> 0..1, clamp).
- extend: remap(tz, 100..200).
- open: remap(tz, 0..-200).
- rotate: roll mapped 0..+-90.

Each weight applies per-bone quaternion tables as AdditiveLocal rotations on 3 main and 3 base spawned lid bones per lid; the R side reuses the L tables. The layers stack after the Zebra smile-driven lower-lid push and before soft eyes. Curves written: lid_tp_blink_l/r (Zebra), plus Monster bt/extend curves, one of which is written twice.

**usdRig today.** The weights and lid curves can be pre-exec FloatMathMover chains (remap -> clamp) once the slider avars can be read. The layered per-bone weighted rotations have no operator. A nested-null plus RotationConstraint-to-twin workaround would need about 5 layers per lid bone (12 bones per eye), and it blends asset-space Euler angles, which diverges for the mixed-axis 60-70 deg blink quaternions.

**Gap.** Missing: the additive offset operator (with table-driven per-bone values) and the avar read.

**Porting impact.** Blocker: the eyes cannot blink, open wide or tilt from the lid sliders.

**Recommendation.** The converter authors one RigExecTransformOffsetMover per (bone, layer) from the tables. Use quatf inputs:orientation so the tables need no Euler conversion; the UE analysts flagged their degree conversions as approximate. Connect inputs:defaultWeight to the layer weight float. Read tz and roll through G6-control-channel-read. The lid curves are the same float attributes connected into BlendInputs, and the Monster double write is reproduced through mover order.

**Evidence:** `libs/rigExecMath/solvers.cpp:873-942`; `libs/rigExecSchema/schema.usda:1087-1100`; `libs/rigExec/rigEvaluator.cpp:525-569`; `libs/rigExecSchema/schema.usda:1192-1214`

**Verification (corrected).** REFUTED as Missing: the layered per-bone weighted rotations do have an operator. One self-sourced ParentConstraint per (bone, layer) takes rotationOffsets from the tables and inputs:defaultWeight from the layer weight. It applies Offset*current in the bone frame (solvers.cpp:1047-1055) and stacks in mover order (rigEvaluator.cpp:5856-5858). This needs about 12 constraints per layer per eye, not 5 nested nulls per bone.

What remains:
- Partial weights blend per-axis Euler rather than slerp (solvers.cpp:779-800), which matters for the mixed-axis 60-70 degree blink quaternions.
- rotationOffsets are Euler, so each quaternion needs an exact conversion.
- The slider tz and roll reads are missing (control-channel bridge).

The severity stays blocker only because of that read: the lid sliders still cannot drive blink.

Verifier evidence: `libs/rigExecMath/solvers.cpp:1047-1055`; `libs/rigExecMath/solvers.cpp:779-800`; `libs/rigExec/rigEvaluator.cpp:5856-5858`; `libs/rigExec/rigEvaluator.cpp:560-566`; `libs/rigExecSchema/schema.usda:1135-1140`

### G6-jaw-open-logic

**Jaw Open Logic: Jaw Normalize, jaw_open curve, smile-masked lip corner pull**

**Verdict:** Missing · **Severity:** major · **Effort:** M · **Confidence:** high · **Domain:** D14

UE features: `UE6-jaw-open-logic`

**UE rigs.** With B = the jaw-open reader value:
- Member variable Jaw Normalize = 4B; curve jaw_open = B.
- wL = B*(1 - smile_l) and wR = B*(1 - smile_r) weight AdditiveLocal X translations on lip_corner, lip_bt_01 and lip_tp_01/02, plus cheek_r (the Monster adds the 03 bones).

The mouth corners tuck in as the jaw opens, unless the character is smiling. Negative weights are skipped.

**usdRig today.** (1 - smile) and the products are expressible with FloatMathMover chains, but B is pose-derived. The translation offsets need the offset mover, or position-constraint twins. The Jaw Normalize member is just a float attribute written by a chain.

**Gap.** Missing: the late scalar phase, the offset mover, and a clamp weight policy. Today a negative weight fails the envelope with a diagnostic instead of being skipped silently.

**Porting impact.** The lip corners do not tuck when the jaw opens, and the JN-based open correctives are lost as well.

**Recommendation.** Chain: reader, or the jaw control channel read -> late chains (multiply 4 for JN; add(-smile) and add(1), then multiply B, for wL and wR) -> RigExecTransformOffsetMovers with rigExec:weightPolicy = clamp.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:10998-11028`; `libs/rigExecMath/propertyMath.cpp:33-61`; `libs/rigExec/rigEvaluator.cpp:10202-10216`

**Verification (holds).** The verdict holds because B is pose-derived, or would come from a control read that is missing.
- The translation offsets themselves are expressible with self-sourced ParentConstraints (translation lerp is exact; solvers.cpp:1093-1098).
- A negative weight passes through with a diagnostic (rigEvaluator.cpp:11018-11026). The pose result equals UE's skip; only the diagnostic noise differs.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:11018-11045`; `libs/rigExecMath/solvers.cpp:1093-1098`; `libs/rigExec/rigEvaluator.cpp:10214-10215`

### G6-monster-trap

**Monster clavicle trapezius helper correctives (joint-only)**

**Verdict:** Missing · **Severity:** major · **Effort:** S · **Confidence:** high · **Domain:** D11

UE features: `UE-monster-trap-correctives`

**UE rigs.** CR_Monster_Deform has 4 clavicle up/dn readers (+H excluded, reference spine_05). Each drives an AdditiveLocal Y+-30 deg rotation plus a 2-30 cm slide on def_trap_l/r. No curves are written.

**usdRig today.** No path exists. The output is joint-only, and usdRig has no way to move a joint from a pose measurement within the same evaluation: RigExecPoseInterpolator publishes floats only, after the pose walk, and the pose steps are solver batches and constraints only.

**Gap.** Missing: the reader and the additive offset operator.

**Porting impact.** The Monster's trapezius helpers never move, so shoulder shrug deformation is lost.

**Recommendation.** Use RigExecSphericalPoseReader plus RigExecTransformOffsetMover. A separate post-process rig is not needed; author them as a deform sublayer under the Monster rig's Movers.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:11666-11675`; `libs/rigExec/rigEvaluator.cpp:5945-6035`; `libs/rigExec/rigEvaluator.cpp:2720-2723`

**Verification (holds).** The verdict holds for the feature as a whole. The offset half now has a partial operator (a self-sourced ParentConstraint, solvers.cpp:1047-1055), but its weight is a clavicle pose measurement, and nothing publishes a scalar inside the pose walk:
- Pose steps are solver batches and constraints only (rigEvaluator.cpp:6022-6029).
- Interpolators run after the walk (11675).
- Constraint weights resolve through _resolvedInputs, which only property chains fill before the walk (11015-11017, 10214-10239).

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:6022-6029`; `libs/rigExec/rigEvaluator.cpp:11015-11017`; `libs/rigExec/rigEvaluator.cpp:11667-11675`

### G6-soft-eyes

**Soft eyes: eye-direction readers driving lid bones**

**Verdict:** Missing · **Severity:** major · **Effort:** M · **Confidence:** high · **Domain:** D14

UE features: `UE6-soft-eyes`

**UE rigs.** 4 SphericalPoseReaders per eye (dn, up, in, ot on eye_l/r, with skewed factors). Each reader's weight drives a loop of AdditiveLocal rotations over the 12 lid bones, using the Soft Eyes tables, so the lids follow gaze. These run after the blink layers.

**usdRig today.** Single-pose RBF readers on the eye joints could produce the weights, but only after the pose walk, and no operator applies weights to joints.

**Gap.** Missing: a reader scheduled in the pose DAG and the offset mover.

**Porting impact.** The lids do not follow eye direction, so the fleshy-eye look is lost.

**Recommendation.** Add RigExecSphericalPoseReaders on the eye joints, scheduled after the eye AimConstraints, plus 48 TransformOffsetMovers per eye, ordered after the lid-layer movers.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:11666-11675`; `libs/rigExec/rigEvaluator.cpp:5945-6035`; `libs/rigExecSchema/schema.usda:1852-1861`

**Verification (holds).** The verdict holds: the weights are eye-bone pose measurements, and nothing publishes a scalar inside the walk (rigEvaluator.cpp:6022-6029, 11675). The offset half is now partially covered by self-sourced ParentConstraints.

A usdRig-native approximation (not parity): constant-weight ParentConstraints on the lid bones sourced from the eye joint, with baked rest-relative offsets, for fractional follow.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:6022-6029`; `libs/rigExec/rigEvaluator.cpp:11667-11675`; `libs/rigExecMath/solvers.cpp:1047-1055`

### G6-additive-local-offset

**Scalar-weighted additive local transform offsets (ModifyTransforms AdditiveLocal)**

**Verdict:** Partial · **Severity:** major · **Effort:** L · **Confidence:** high · **Domain:** D11 · *(analyst said Missing / blocker)*

UE features: `UE-helper-shoulder-drive`

**UE rigs.** RigUnit_ModifyTransforms in AdditiveLocal mode:
- Skips when w <= 0.
- Otherwise builds T = LerpTransform(Identity, Offset, clamp(w,0,1)), a quaternion blend of rotation.
- Sets local := T * local, so the offset is applied in the bone's own frame.

Nodes on the same bone stack in execution order: def_strap gets 8 offsets (rotations such as Y+50, Y-80, Z+-45 and 0.5-4 cm slides), def_chest 3 and def_back 4. CR_Zebra_Deform uses 82 of these; the face modules use about 60 (lids, lips, brows, jaw).

**usdRig today.** There is no equivalent operator. Two partial workarounds exist:
(a) A Position/RotationConstraint toward a pre-offset 'twin' null, with inputs:defaultWeight = w (the biped foot-roll pattern). This blends toward an ABSOLUTE target in asset-space Euler, so a second layer overrides the first instead of adding to it, unless every layer gets its own nested null.
(b) A MatrixMathMover multiply on a matrix dial connected to the joint's posed:defaultSpace. This is a component-wise matrix lerp, which is non-rigid for large rotations, and it runs pre-exec only.

Constraint offsets are additive asset-space Euler values applied after the blend, not local offsets.

**Gap.** Missing: an operator that pre-multiplies a weighted (slerped) local SRT offset onto a transform, stacks by mover order, and clamps or skips out-of-range weights. Today a usdRig envelope outside [0,1] makes the mover pass through with a diagnostic.

**Porting impact.** Blocker for the face: blink, lid open/rotate, soft eyes, lip roll/puff/CH, the jaw-open lip pull and the lip tweaker deltas all depend on it. Major for the body helper bones (def_strap, def_chest, def_back, def_elbow, def_knee, def_thigh and the twist-bone slides). Twin-null workarounds would need about 5 nested nulls per lid bone and would still blend Euler angles.

**Recommendation.** Add RigExecTransformOffsetMover (applies RigExecMoverAPI) with:
- rigExec:moves = exactly one RigExecJoint, RigExecControl or Xformable.
- double3 inputs:translation; quatf inputs:orientation, or double3 inputs:rotation plus rigExec:rotationOrder; double3 inputs:scale = (1,1,1).
- uniform token rigExec:offsetSpace = local|asset (UE AdditiveLocal / AdditiveGlobal).
- optional rel rigExec:offsetSource: a provider whose local avar transform is used as the offset (lip tweakers).
- uniform token rigExec:weightPolicy = strict|clamp, where clamp reproduces UE (skip w<=0, clamp w>1).

Implementation:
1. Kernel RigExecApplyTransformOffset in libs/rigExecMath/solvers.cpp: slerp from identity, then offset * local in row-vector order.
2. Validate next to the constraints (rigEvaluator.cpp:3355-3428) and run it as a constraint-like pose step (rigEvaluator.cpp:10986-11060), so inputs:defaultWeight resolves through _resolvedInputs.
3. Add a baked step in bakedPose.cpp, plus builder and Python facade support.

Stacking order is the existing reverse-sibling post-order. The mover must be allowed to post-modify solver-bound joints (TwistDistribution outputs).

**Evidence:** `libs/rigExecSchema/schema.usda:980-996`; `libs/rigExecMath/solvers.cpp:873-942`; `libs/rigExecMath/propertyMath.cpp:97-127`; `libs/rigExecSchema/schema.usda:2119-2137`; `libs/rigExec/computations.cpp:298-314`; `libs/rigExec/rigEvaluator.cpp:10998-11028`; `examples/biped/Biped.usda:3097-3157`

**Verification (corrected).** REFUTED as Missing: a self-sourced RigExecParentConstraint already performs a weighted, stacked, pre-multiplied local offset inside the pose walk.

Setup: rigExec:moves = the bone, rigExec:sources = the same bone, inputs:translationOffsets/rotationOffsets = the UE offset, inputs:defaultWeight = w.
- The kernel builds target = Offset * Source, with the offset in source-local space (solvers.cpp:1047-1055). With source = the bone this is exactly UE AdditiveLocal: Transform * CurrentLocal/Global with children propagated (RigUnit_ModifyTransforms.cpp AdditiveLocal branch).
- Sources read the provider's current revision, which is implicit 'preceding' (rigEvaluator.cpp:10477-10486).
- Neither bindFrameSource nor source binding rejects a self-source (4958-4989, 5026-5051).
- Constraints run serially in mover order (5856-5858), so offsets on one bone accumulate like UE's chain.
- A constraint depends on the solver that owns its target (5859-5868), so it can post-modify TwistDistribution outputs.
- The weight resolves through _resolvedInputs. w<=0 is an exact dormant pass-through, matching UE's skip (11015-11045).

The biped already uses ParentConstraints with non-trivial per-source rotationOffsets (Biped.usda:2076-2085). A second, cruder path also exists: a MatrixMathMover chain (post-multiply, component-wise lerp; propertyMath.cpp:97-127) on a matrix dial connected to posed:defaultSpace (computations.cpp:425-436, 499-510).

What is really missing:
- Quaternion slerp at partial weight. usdRig blends per-axis asset-space Euler (solvers.cpp:779-800, 1104-1107); UE slerps (RigVMMathLibrary.cpp:253-260).
- A clamp policy. usdRig skips the whole constraint when w>1 (the offset drops to zero, probe C1b), with a diagnostic (rigEvaluator.cpp:11018-11026); UE clamps.
- An offset taken from a control's live transform (lip tweakers).
- A quaternion offset input.
- Documentation and tests for the self-source pattern (it is unexercised).

The blocker-level face consequences come from pose-derived and control-derived weights, which G6-control-channel-read and G6-deform-exec-order track. They do not come from this operator, so major, not blocker.

Verifier evidence: `libs/rigExecMath/solvers.cpp:1047-1055`; `libs/rigExecMath/solvers.cpp:1093-1107`; `libs/rigExecMath/solvers.cpp:779-800`; `libs/rigExec/rigEvaluator.cpp:10477-10486`; `libs/rigExec/rigEvaluator.cpp:4958-4989`; `libs/rigExec/rigEvaluator.cpp:5026-5051`; `libs/rigExec/rigEvaluator.cpp:5856-5868`; `libs/rigExec/rigEvaluator.cpp:11015-11045`; `examples/biped/Biped.usda:2076-2085`; `libs/rigExecMath/propertyMath.cpp:97-127`; `libs/rigExec/computations.cpp:425-436`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Highlevel/Hierarchy/RigUnit_ModifyTransforms.cpp (Weight<=Minimum return; T clamp; Transform * current)`; `<UE>/Plugins/Runtime/RigVM/Source/RigVM/Private/RigVMFunctions/Math/RigVMMathLibrary.cpp:253-260`

### G6-brow-main

**Brow Main diagonal pads to brow in/out up/down curves**

**Verdict:** Partial · **Severity:** major · **Effort:** S · **Confidence:** medium · **Domain:** D14

UE features: `UE6-brow-main`

**UE rigs.** The Brow Main L/R pads sit on skull_tp with an offset rotated +-45 deg about Y and a mirrored negative scale, and TranslationY is locked. brow_in_up/dn come from +-tz and brow_ot_up/dn from +-tx, each as clamp(0.01*v, 0, 200). The pads host the bool channel 'Brow Tweaker Vis'.

**usdRig today.** The diagonal frame goes in the control's rest/default rotation, so avars are measured in that frame. The curves are FloatMathMover chains, pending the avar read. The negative-scale mirror cannot live in rest:space, which is orthonormalized; the R side instead uses a rotated frame and negated multipliers. A tweaker visibility dial must be a float, since guide opacity connections accept float or double.

**Gap.** Missing: the avar read, the Y-axis lock (no limits), and a bool visibility channel.

**Porting impact.** The brow pads work only once the bridge exists, and animators can move a pad off its plane.

**Recommendation.** Depends on G6-control-channel-read. Add RigExecControlAPI limits (G3). Allow guide:displayOpacity to follow a bool channel, or use a float visibility channel.

**Evidence:** `libs/rigExecSchema/schema.usda:298-300`; `libs/rigExecSchema/schema.usda:223-249`; `libs/rigExecSchema/schema.usda:33-47`; `libs/rigExec/rigEvaluator.cpp:525-569`

**Verification (holds).** The verdict holds.
- rest:space is always orthonormalized (schema.usda:298-300).
- There are no limits (schema.usda:33-47).
- The pad translations cannot be read (the double-to-float rejection).

Verifier evidence: `libs/rigExecSchema/schema.usda:298-300`; `libs/rigExecSchema/schema.usda:33-47`; `libs/rigExec/rigEvaluator.cpp:560-566`

### G6-brow-micro

**Brow micro controls on curve-driven helper bones + micro pad curves**

**Verdict:** Partial · **Severity:** major · **Effort:** S · **Confidence:** medium · **Domain:** D14

UE features: `UE6-brow-micro-follow`, `UE6-brow-micro-curves`

**UE rigs.** Helper bones Brow In/Ot L/R under skull_tp slide Z+4 or Z-5 through AdditiveLocal offsets weighted by the brow_*_up/dn curves. A null Brow Mid is kept at the average of the two helpers by a maintain-offset position constraint; two further constraints target nulls that do not exist, so they do nothing. The micro controls ride these helpers. Brow Micro reads each micro pad's translation into up/dn/side curves (18 in total).

**usdRig today.** - Helper slides: two RigExecPositionConstraints per helper, toward 'up' and 'down' twin nulls placed at +offset and -offset, with envelopes connected to the curves. This works because up and down are mutually exclusive.
- Brow Mid: a RigExecPositionConstraint with two equal sources and an authored translationOffset (the precomputed maintain offset). The no-op constraints are dropped.
- Micro controls: nested under the helpers.
- Curves: FloatMathMover chains, pending the avar read.

**Gap.** The offset mover is missing, though a twin-null workaround exists. Maintain-offset capture and the avar read are also missing.

**Porting impact.** Micro controls follow the brow shape only through the workaround, and the micro curves need the bridge.

**Recommendation.** Use RigExecTransformOffsetMover (cleaner than twin nulls) and G6-control-channel-read. The converter computes the Brow Mid offset at rest.

**Evidence:** `libs/rigExecMath/solvers.cpp:805-871`; `libs/rigExecSchema/schema.usda:1073-1085`; `libs/rigExec/rigEvaluator.cpp:11015-11028`

**Verification (holds).** The verdict holds, but there is a better workaround than twin nulls: self-sourced ParentConstraints on Brow In/Ot with translationOffsets (0,0,4) and (0,0,-5). Translation is lerped exactly (solvers.cpp:1093-1098), and the offsets stack in order.

The weights (brow curves from the pads) remain blocked by the missing control read, so major stands.

Verifier evidence: `libs/rigExecMath/solvers.cpp:1047-1055`; `libs/rigExecMath/solvers.cpp:1093-1098`; `libs/rigExecMath/solvers.cpp:805-871`

### G6-combo-min

**Combination corrective as Min of two pose-reader outputs**

**Verdict:** Partial · **Severity:** major · **Effort:** M · **Confidence:** high · **Domain:** D12 · *(analyst said Partial / minor)*

UE features: `UE-combo-min-corrective`

**UE rigs.** thigh_up_c = min(thigh_up_l, thigh_up_r) and thigh_up_ext_c = min(Remap5, Remap6). Both drive centre morphs that fire only when both thighs are raised.

**usdRig today.** min(a,b) is writable today as a FloatMathMover clamp with a very low min and inputs:max.connect = b, applied over a base that carries a. This works only for pre-exec values, and reader outputs are post-pose. RigExecBlendInput has no combination (min or product) mode.

**Gap.** Missing: a late scalar chain or a BlendInput combination mode, and explicit min/max ops.

**Porting impact.** The two centre combination morphs stay off. More generally, the pattern 'combination of pose correctives' cannot be ported.

**Recommendation.** Add to RigExecBlendInput (schema.usda:1755-1766):
- uniform token rigExec:combination = none|product|minimum|maximum
- rel rigExec:combinationInputs (other BlendInputs, or float outputs such as reader or pose weights)

Resolve these in the geometry phase after readers and interpolators (rigEvaluator.cpp:12123-12129), so there is no phase problem. Also add 'min' and 'max' ops to propertyMath.cpp for pre-exec chains.

**Evidence:** `libs/rigExecMath/propertyMath.cpp:45-50`; `libs/rigExecSchema/schema.usda:1755-1766`; `libs/rigExec/rigEvaluator.cpp:12123-12129`; `libs/rigExec/moverGraph.cpp:1846-1874`

**Verification (corrected).** The verdict (Partial) holds; the severity is understated.

I looked for another route to min() and found none that works:
- RigExecCombineWeight has min/max/multiply modes (schema.usda:1644-1652), and a DynamicWeight driver is connectable (1294-1300).
- But geometry weight packets come from the exec snapshot, which is evaluated before the interpolator phase (rigEvaluator.cpp:11475-11478 vs 11675, 12026-12029).
- Pose weights are never exec overrides (2720-2723), so a pose-driven weight object reads authored values.

thigh_up_c and thigh_up_ext_c are the only correctives for both thighs raised, which covers sitting and crouching, common game poses. Losing them is a visible behaviour loss: major, consistent with G6-face-correctives.

Verifier evidence: `libs/rigExecSchema/schema.usda:1644-1652`; `libs/rigExec/rigEvaluator.cpp:11475-11478`; `libs/rigExec/rigEvaluator.cpp:12026-12029`; `libs/rigExec/rigEvaluator.cpp:2720-2723`; `libs/rigExecMath/propertyMath.cpp:45-50`

### G6-corner-logic

**Corner Logic: 2D mouth-corner pad to smile/frown/wide/narrow/curl curves**

**Verdict:** Partial · **Severity:** major · **Effort:** S · **Confidence:** high · **Domain:** D11

UE features: `UE6-corner-logic`

**UE rigs.** From the Corner L/R pad local values:
- smile = clamp(0.01*tz, 0, 100); frown = clamp(-0.01*tz, 0, 100).
- wide and narrow are the same functions of +ty and -ty.
- curl = -0.01 * EulerZYX(rotation).X, unclamped.

This produces 10 curves, which also feed the correctives, the lower-lid smile push and the jaw-open lip mask.

**usdRig today.** Each curve is a FloatMathMover chain on a float curve attribute: add(avars:tz) -> multiply(+-0.01) -> clamp(0,100). curl is add(avars:rx) -> multiply(-0.01). All of it runs pre-exec in dependency order, and downstream consumers connect to the curve attribute.

**Gap.** The avar read is missing (G6-control-channel-read). avars:rx equals UE's Euler ZYX X only for single-axis rotation or a ZYX rotation order. There is no reusable function abstraction, so each curve takes 3 authored movers.

**Porting impact.** Until the bridge exists, the corner pad cannot drive smile, frown, wide, narrow or curl, and everything downstream of those curves is affected.

**Recommendation.** Depends on G6-control-channel-read. Author Corner Logic once as a USD class prim and instantiate it per side through inherits or references; the converter can generate it.

**Evidence:** `libs/rigExecMath/propertyMath.cpp:33-61`; `examples/biped/Biped.usda:3097-3157`; `libs/rigExec/rigEvaluator.cpp:8829-8930`

**Verification (holds).** The verdict holds.
- smile, frown, wide and narrow are pad translations, which cannot be read (2552-2562 and the double-to-float rejection).
- curl alone could drive the curl_up morph exactly through a twist-type linear pose on the Corner control, but only as a BlendInput weight (11675).
- The add/multiply/clamp arithmetic is exact once a read exists (propertyMath.cpp:33-61).

A usage caveat for the recommendation: docs/biped-rig.md:246-252 warns that inherits also composes animation opinions, so class instancing should use references.

Verifier evidence: `libs/rigExecMath/propertyMath.cpp:33-61`; `libs/rigExec/rigEvaluator.cpp:2552-2562`; `docs/biped-rig.md:246-252`

### G6-deform-exec-order

**Deform-graph execution order: read pose -> compute scalar -> modify pose in one pass**

**Verdict:** Partial · **Severity:** major · **Effort:** L · **Confidence:** high · **Domain:** D11

UE features: `UE-deform-exec-order`

**UE rigs.** BeginExecution -> Sequence. Sequence.A runs one serial chain per region: reader -> SetCurveValue -> Remap/Min -> ModifyTransforms. Each reader sees the pose after earlier offsets, additive offsets accumulate in chain order, and SetCurveValue overwrites. Because bTransferInputCurves is on, the recomputed curves replace baked corrective curves from the animation. Sequence.B then adds the 7 Optimus deformers.

**usdRig today.** usdRig derives its order from the wiring:
1. Pre-exec property chains.
2. The pose DAG (Kahn levels of solver batches and constraints).
3. Pose interpolators, after the full walk.
4. Geometry chains.

Within that order:
- Movers on the same target stack in reverse-sibling post-order.
- A BlendInput connection outranks its authored (baked) spline, and a chain result overrides the authored base, so the 'rig recomputes baked curves' behaviour is reproduced.
- The deform rig can be a sublayer under <rig>/Movers.

**Gap.** There is no scheduling slot for read-pose -> scalar -> modify-pose inside the pose walk: pose steps are only solver batches and constraints, and property chains are evaluated before exec. The spec forbids hidden previous-frame state, so a one-frame-lag workaround is excluded too.

**Porting impact.** The CR_*_Deform topology (reader -> remap -> offset) has nowhere to run; only its curve -> morph tail survives.

**Recommendation.** Generalize _PoseStep (rigEvaluator.cpp:5945-6035, walked at 10894) to three kinds:
1. Solver batch.
2. Constraint or offset mover.
3. Scalar step: a pose reader or a late property chain.

Build pose dependencies from reader driver/parent providers and from connections into mover envelopes. Keep the pre-exec path for chains with no pose-derived inputs, so examples 03 and 09 are unchanged. Assert ordering the way rigEvaluator.cpp:11724-11755 already does, and mirror the new step kinds in the bakedPose.cpp step list.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:10202-10216`; `libs/rigExec/rigEvaluator.cpp:11658-11675`; `libs/rigExec/rigEvaluator.cpp:11724-11755`; `libs/rigExec/rigEvaluator.cpp:5945-6035`; `libs/rigExec/rigEvaluator.cpp:10986-11028`; `libs/rigExec/moverGraph.h:319-378`; `README.md:47-52`; `README.md:199-220`; `docs/spec.md:1042-1050`

**Verification (holds).** The verdict holds.
- Chains run first (rigEvaluator.cpp:10202-10239).
- The walk contains only solver batches and constraints (6022-6029, 10894-10976).
- Interpolators run after the walk and before geometry, with an asserted order (11667-11675, 11724-11755).
- A BlendInput connection resolves to the in-memory pose weight before any authored value (moverGraph.h:349-376).
- The rejection of hidden previous-frame state is confirmed at docs/spec.md:1042-1050 and 59.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:6022-6029`; `libs/rigExec/rigEvaluator.cpp:10894-10976`; `libs/rigExec/rigEvaluator.cpp:11724-11755`; `libs/rigExec/moverGraph.h:349-376`; `docs/spec.md:1042-1050`

### G6-deformer-channel-mapping

**Squash controls -> deformer factor curves**

**Verdict:** Partial · **Severity:** major · **Effort:** S · **Confidence:** high · **Domain:** D11

UE features: `UE7-deformer-control-channel-mapping`

**UE rigs.** Head, Muzzle, Skull Tp and Mouth Squash controls: local Tz and Tx (and the head's rotZ via Euler ZYX) go through unclamped linear Remaps into head_squash, head_bend, head_twist and the *_deformer curves. Zebra has 3 controls and 7 writes; Monster has 4 and 9. Each rig also has one dead duplicate write.

**usdRig today.** The remap arithmetic is a pre-exec FloatMathMover chain, because control avars are authored values: add(avar) -> multiply(scale) -> optional add. It is blocked by input typing (avars are double). The deformer consumer is absent (group G7).

**Gap.** Missing: the double-to-float control read, and the parametric deformer (G7).

**Porting impact.** The squash and bend controls cannot drive their factors; animators would key float dials instead.

**Recommendation.** Depends on G6-control-channel-read. The converter maps each Remap to the proposed remapRange op, or to add + multiply + add, and drops the dead duplicate writer.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:525-569`; `libs/rigExecSchema/schema.usda:327-336`; `libs/rigExecSchema/schema.usda:1192-1214`; `plugin/rigExecUsdview/avarEditorModel.py:37-46`

**Verification (holds).** The verdict holds. The translation reads need the missing bridge. head_twist = -rz/135 is signed and unclamped, which an interpolator cannot produce: the linear kernel is max(0,...) and there is no subtraction after the walk. The consuming deformer is absent (G7).

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:560-566`; `libs/rigExecMath/rbf.h:147-149`; `libs/rigExecSchema/schema.usda:327-336`

### G6-elbow-knee-readers

**Elbow/knee hinge readers with thresholded squash slides on twist bones**

**Verdict:** Partial · **Severity:** major · **Effort:** M · **Confidence:** high · **Domain:** D11

UE features: `UE-spr-zebra-elbow`, `UE-spr-zebra-knee`

**UE rigs.** The elbow and knee readers are centred on the back pole: outer extent 160-168 deg with one half-plane excluded, so they are 0 at rest and hyperextension is rejected. They write elbow_l/r and knee_l/r and drive def_elbow_in/ot, def_knee_in/ot and calf_twist_03/04 offsets.

A clamped Remap(0.4..1 -> 0..1) of the same output then:
- writes elbow_squash_* and knee_squash_*;
- slides upperarm_twist_02-04, lowerarm_twist_03/04 and thigh_twist_02-04 by 3-10 cm;
- rotates thigh_twist_01 by Z-20 on both sides (an asymmetry).

**usdRig today.** A hinge reader can be approximated with a single linear RBF pose placed at the full-bend direction and a radius close to the bend angle; this also rejects hyperextension. FloatMathMover remap+clamp exist, but property chains run before exec with _resolvedInputs cleared, so they cannot read a pose output. The twist bones are claimed by RigExecTwistDistribution, so any extra offset would have to post-modify solver outputs in the pose DAG, and no operator does that.

**Gap.** Missing: a stage that remaps pose-derived values, the additive offsets, and post-solve offsets on TwistDistribution-owned joints.

**Porting impact.** Elbow and knee pinch helpers and the squash volume slides are lost. The *_squash morphs can only be faked with a separate RBF pose tuned to start at 40% bend.

**Recommendation.** Use the reader, the offset mover and the late scalar chain from G6-deform-exec-order. Add a 'remapRange' op to libs/rigExecMath/propertyMath.cpp and the rigExec:operation tokens (schema.usda:1208-1210), with source min/max, new inputs:targetMin/targetMax and a clamp flag. Validate that a RigExecTransformOffsetMover targeting a TwistDistribution joint is scheduled after that solver's batch.

**Evidence:** `libs/rigExecSchema/schema.usda:654-693`; `libs/rigExecMath/propertyMath.cpp:45-55`; `libs/rigExecSchema/schema.usda:1204-1205`; `libs/rigExec/rigEvaluator.cpp:10202-10216`; `libs/rigExec/rigEvaluator.cpp:11666-11675`

**Verification (holds).** The verdict and severity hold, but the claim that 'no operator post-modifies solver outputs' is wrong.
- A constraint's pose dependencies include the solver that owns its target (rigEvaluator.cpp:5859-5868).
- The schedule code explicitly supports a constraint target that sits on a solver-bound joint (5905-5913).
- So a self-sourced ParentConstraint on upperarm_twist_0x or thigh_twist_0x runs after the TwistDistribution batch.

The real blocker remains: the Remap(0.4..1) input is a pose-derived value, and FloatMathMover chains run before exec with _resolvedInputs cleared (10214-10215). The remap is unclamped (propertyMath.cpp:51-55), so a clamp mover is needed as well.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:5859-5868`; `libs/rigExec/rigEvaluator.cpp:5905-5913`; `libs/rigExec/rigEvaluator.cpp:10214-10215`; `libs/rigExecMath/propertyMath.cpp:51-55`

### G6-face-correctives

**Face combination (product) correctives**

**Verdict:** Partial · **Severity:** major · **Effort:** M · **Confidence:** high · **Domain:** D12

UE features: `UE6-correctives`

**UE rigs.** 16 combination curves computed as products:
- JN*smile, JN*wide, JN*frown, where JN = 4 * the jaw reader, read from the PREVIOUS evaluation.
- wide*frown, brow_in_dn*brow_ot_dn, brow_in_dn*brow_squeeze, smile*wide.
- smile_open_c*wide, reading back an earlier product.

Nothing is clamped, so values can exceed 1. Bugs: wide_open_c_r is overwritten and frown_wide_c_r is never written. The Monster renames open_frown_c, and its smile_wide_open reads curves the Monster never writes.

**usdRig today.** Products of control-driven curves become FloatMathMover chains: base 0 -> add(a) -> multiply(b via connection), writing the combination BlendInput weight. These run pre-exec in dependency order, and read-backs of earlier products work. JN-based products have no path because JN is pose-derived. The one-evaluation lag cannot be reproduced, since the spec forbids hidden previous-frame state; usdRig would use the same-frame value.

**Gap.** Missing: a late scalar phase or a BlendInput combination mode (see G6-combo-min). Weights above 1 are clamped.

**Porting impact.** The 6 JN-based Zebra combination shapes (smile/wide/frown_open) cannot fire. The others work once the corner and brow curves are readable. The one-frame lag becomes same-frame, which is Divergent-by-design.

**Recommendation.** Add RigExecBlendInput rigExec:combination = product with rigExec:combinationInputs, resolved after readers and interpolators, or use late chains. If jaw_open is derived from the jaw control (see G6-jaw-open-reader), every product becomes a pre-exec chain. Document the removal of the lag.

**Evidence:** `libs/rigExecMath/propertyMath.cpp:39-44`; `docs/spec.md:1042-1050`; `libs/rigExec/rigEvaluator.cpp:10202-10216`; `libs/rigExec/rigEvaluator.cpp:11666-11675`; `libs/rigExec/moverGraph.cpp:1868-1874`

**Verification (holds).** The verdict holds.
- The multiply operation is exact, and connected values resolve through chains in dependency order (propertyMath.cpp:42-44; rigEvaluator.cpp:5265-5374).
- JN is pose-derived.
- A pose-driven CombineWeight is not a workaround, because weight packets are taken from the pre-interpolator exec snapshot (rigEvaluator.cpp:11475-11478).
- Removing the one-frame lag is consistent with spec.md:1050 and 59.

Verifier evidence: `libs/rigExecMath/propertyMath.cpp:39-44`; `libs/rigExec/rigEvaluator.cpp:5265-5374`; `libs/rigExec/rigEvaluator.cpp:11475-11478`; `docs/spec.md:1042-1050`

### G6-foot-values

**Reading foot control rotations into roll/bank/heel scalars**

**Verdict:** Partial · **Severity:** major · **Effort:** M · **Confidence:** medium · **Domain:** D11

UE features: `UE3-foot-values`

**UE rigs.** In CRM_FN_Foot Pre Forwards Solve:
- Ball IK local rotation -> Euler YZX gives heel bend, twist and side.
- Foot Rocker local rotation -> swing-twist about X and about Y; each twist -> Euler ZYX / ZXY gives the roll and bank scalars.

These scalars drive the pivot logic. The animator rotates controls rather than moving float sliders.

**usdRig today.** The biped expresses foot roll with custom FLOAT dials (foot:/avars:) feeding FloatMathMover chains that drive RotationConstraint envelopes. A control's rotation avars are double, and float chain inputs must connect to float sources, so a Foot Rocker control's rotation cannot feed those chains. There is no Euler re-order or twist extraction node.

**Gap.** Missing: a double-to-float channel read, and Euler re-order / twist extraction as a node.

**Porting impact.** UE's rotate-the-rocker and ball-control workflow must become float sliders. For rockers rotated on two axes, the values differ from UE's twist-extracted numbers.

**Recommendation.** Use G6-control-channel-read. Add a channel-reader variant of RigExecTwistReader that takes a control as source: uniform token rigExec:eulerOrder, float outputs eulerX/Y/Z and twist about an axis. Because it depends only on avars, it can run as the head of a pre-exec property chain and needs no pose step.

**Evidence:** `plugin/rigExecUsdview/avarEditorModel.py:37-46`; `libs/rigExec/rigEvaluator.cpp:525-569`; `examples/biped/Biped.usda:3097-3157`; `libs/rigExecSchema/schema.usda:327-336`

**Verification (holds).** The verdict holds.
- Custom float dials plus FloatMathMover chains drive RotationConstraint envelopes (Biped.usda:3097-3130).
- Avars are double (schema.usda:327-336), and float inputs reject double connections (rigEvaluator.cpp:560-566, 3632-3650).
- A twist-type interpolator on the Foot Rocker control could compute a roll ramp (driver may be a control, 2478-2483), but only after the walk (11675), while the pivots need it inside the walk.

I found no Euler re-order or twist node.

Verifier evidence: `examples/biped/Biped.usda:3097-3130`; `libs/rigExecSchema/schema.usda:327-336`; `libs/rigExec/rigEvaluator.cpp:560-566`; `libs/rigExec/rigEvaluator.cpp:2478-2483`; `plugin/rigExecUsdview/avarEditorModel.py:37-46`

### G6-jaw-open-reader

**Jaw-open cone reader**

**Verdict:** Partial · **Severity:** major · **Effort:** S · **Confidence:** medium · **Domain:** D11

UE features: `UE6-jaw-open-reader`

**UE rigs.** A SphericalPoseReader on the jaw bone (axis Y, offset (90,0,90), falloff factors 1.1/0.8/0.8/0.8) outputs a 0..1 jaw-open weight that feeds Jaw Open Logic.

**usdRig today.** A single-pose linear RBF on the jaw joint approximates the weight for driving blend shapes. Its consumers (Jaw Normalize, the lip-corner pull and the jaw_open-based correctives) need the value inside the pose walk or the scalar logic, which is not possible today.

**Gap.** Missing: a reader scheduled in the pose DAG, and the elliptical factors.

**Porting impact.** The jaw_open morph can be driven approximately, but the jaw-dependent logic cannot.

**Recommendation.** Use RigExecSphericalPoseReader. Alternatively, if the jaw bone is driven only by the jaw control, derive jaw_open from the control's rotation avar through G6-control-channel-read. That path is pre-exec and makes the whole jaw logic expressible with FloatMathMover chains.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:2494-2507`; `libs/rigExec/rigEvaluator.cpp:11666-11675`

**Verification (holds).** The verdict holds.
- A single-pose RBF reader on the jaw joint can feed the jaw_open BlendInput.
- The consumers inside the walk and the pre-exec consumers cannot use it (rigEvaluator.cpp:11675).
- The control-rotation alternative is also post-walk only, unless a double-to-float bridge is added.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:2494-2507`; `libs/rigExec/rigEvaluator.cpp:11667-11675`

### G6-lip-roll

**Lip Roll In/Out, Lip Puff and Lip CH (channel-weighted lip bone poses + curves)**

**Verdict:** Partial · **Severity:** major · **Effort:** M · **Confidence:** high · **Domain:** D14 · *(analyst said Missing / major)*

UE features: `UE6-lip-roll`, `UE6-lip-puff-ch`

**UE rigs.** The jaw channels Roll Tp/Bt (and on the Monster, Puff and Ch Tp/Bt) give w = +-0.005*channel, plus 0.5 times a dangling reader that always outputs 0. The weights drive per-lip-bone AdditiveLocal rotY and translate poses: about 29 nodes on the Zebra, more on the Monster. The same functions write curves lip_roll_*, lip_puff_* and ch_bt, including name bugs and an unclamped lip_roll_in_bt_l overwrite. Weights above 1 clamp and weights at or below 0 are skipped.

**usdRig today.** The weights and curves are exact pre-exec FloatMathMover chains on the float channels; the dangling reader term is the constant 0 and is dropped. The bone poses have no operator.

**Gap.** Missing: the offset mover, including a clamp weight policy.

**Porting impact.** The bone part of lip roll, puff and CH is lost; only the few matching morphs fire.

**Recommendation.** Add RigExecTransformOffsetMover per bone per function, with rigExec:weightPolicy = clamp. The converter drops the dangling 'Lip Main Tp/Bt' reader terms, which read a control that does not exist, and reproduces the curve-name bugs through mover order.

**Evidence:** `libs/rigExecSchema/schema.usda:1192-1214`; `libs/rigExec/rigEvaluator.cpp:10998-11028`

**Verification (corrected).** REFUTED as Missing.
- The weights come from float channels, so they are pre-exec chains that can connect to inputs:defaultWeight.
- The per-bone poses map to self-sourced ParentConstraints with the UE translation and rotation offsets (solvers.cpp:1047-1055), stacked after the lip bones' other drivers in mover order.
- UE's clamp (w>1 -> 1) needs a clamp FloatMathMover, because usdRig passes out-of-range weights through (rigEvaluator.cpp:11018-11026). w<=0 skips exactly as in UE.

The remaining divergence is per-axis Euler blending instead of slerp for the 70-90 degree rotY offsets at partial weight (solvers.cpp:779-800). That is visible, so major.

Verifier evidence: `libs/rigExecMath/solvers.cpp:1047-1055`; `libs/rigExecMath/solvers.cpp:779-800`; `libs/rigExec/rigEvaluator.cpp:11018-11045`; `libs/rigExecSchema/schema.usda:1192-1214`

### G6-lip-tweakers

**Lip tweaker system (struct-weighted follow nulls + additive tweaker delta)**

**Verdict:** Partial · **Severity:** major · **Effort:** M · **Confidence:** high · **Domain:** D14

UE features: `UE6-lip-tweakers`

**UE rigs.** 20 lip tweaker nulls under the jaw follow [Skull Const, Jaw Const, Corner L, Corner R] through ParentConstraints with per-lip weight rows (Lip_Null_Struct) and maintain offset. The tweaker controls sit under these nulls. Each lip bone then receives AdditiveLocal(Transform = the tweaker control's local transform, weight 1).

**usdRig today.** - Nulls: a RigExecParentConstraint with 4 sources, inputs:sourceWeights = the struct row, and per-source translation/rotation offsets precomputed at rest. This is exact except for rotation averaging: usdRig uses an Euler-anchor mean where UE averages quaternions, a small difference here.
- Tweaker controls: RigExecControls nested under the nulls.
- Tweaker delta on the already-solved lip bone: no operator.

**Gap.** Missing: an offset mover whose offset is a control's local transform, and maintain-offset capture. The construction loop is replaced by generated static prims.

**Porting impact.** The tweakers move the nulls correctly but do not deform the lips.

**Recommendation.** Add RigExecTransformOffsetMover with rel rigExec:offsetSource = the tweaker control (its local avar transform) and weight 1. The converter bakes the per-source offsets from rest and expands the 20-iteration loop.

**Evidence:** `libs/rigExecSchema/schema.usda:1115-1142`; `libs/rigExecMath/solvers.cpp:1027-1087`; `libs/rigExecSchema/schema.usda:1014-1018`

**Verification (holds).** The verdict holds. Constraint offsets are authored double3[] arrays read as values (schema.usda:1135-1140; rigEvaluator.cpp:10501-10523), so the offset cannot be a control's live local transform.

Partial workaround: a ParentConstraint sourced from the tweaker control with zero offset yields tweakerLocal*nullFrame. That equals UE only when the null tracks the bone's pre-tweak pose.

The Euler-anchor rotation averaging in multi-source constraints is confirmed (solvers.cpp:1066-1087).

Verifier evidence: `libs/rigExecSchema/schema.usda:1135-1140`; `libs/rigExec/rigEvaluator.cpp:10501-10523`; `libs/rigExecMath/solvers.cpp:1066-1087`

### G6-shoulder-clavicle-readers

**Zebra shoulder and clavicle correctives (curves + strap/chest/back helper bones)**

**Verdict:** Partial · **Severity:** major · **Effort:** M · **Confidence:** high · **Domain:** D11

UE features: `UE-spr-zebra-shoulder`, `UE-spr-zebra-clavicle`

**UE rigs.** 16 readers on upperarm_l/r and clavicle_l/r (up, fwd, bk, dn per side):
- Mixed reference parents (clavicle; spine_05 for shoulder_bk).
- Skewed and one-sided factors, e.g. shoulder_bk (2,0,1.25,1.25) and clavicle up/dn with +H = 0.
- Authored L/R asymmetries.

Each reader writes one curve (shoulder_*/clavicle_*) and weights 1-3 AdditiveLocal offsets on def_strap, def_chest, def_back and upperarm_twist_01/02. The offsets reach up to 80 deg of rotation and 25 cm of translation.

**usdRig today.** The curve half maps to one approximated RigExecPoseInterpolator per reader, with hidden driver joints for the spine_05-referenced readers. Its pose output connects to the matching BlendInput.inputs:weight (the biped PSD pattern). The joint half has no mapping: pose-interpolator outputs appear only after the pose walk, and no operator applies a weighted local offset.

**Gap.** The 35 helper-bone offsets cannot be expressed. The one-sided shoulder_bk and clavicle up/dn readers can only be approximated.

**Porting impact.** Strap, chest and back helpers stay at rest, so shoulder deformation during raise, shrug and reach is lost. The 16 morph correctives fire with different falloffs.

**Recommendation.** Depends on G6-spherical-pose-reader and G6-additive-local-offset. A new converter (tools/ueImport) would emit, per region, one reader plus N RigExecTransformOffsetMovers under <rig>/Movers/Deform/<region>, each with inputs:defaultWeight.connect = <reader>.outputs:weight. The converter should keep the authored asymmetries (def_strap_r +60 vs +30) for parity.

**Evidence:** `examples/biped/Biped_psd.usda:14-26`; `examples/biped/Biped_psd.usda:2783-2793`; `libs/rigExec/rigEvaluator.cpp:11666-11675`; `libs/rigExec/rigEvaluator.cpp:5945-6035`; `examples/biped/Biped.usda:2128-2149`

**Verification (holds).** The verdict and severity hold, but one sub-claim is wrong. 'No operator applies a weighted local offset' is refuted: a self-sourced ParentConstraint does (see G6-additive-local-offset; solvers.cpp:1047-1055).

The joint half is still unportable because its weights are pose measurements:
- Pose steps are only solver batches and constraints (rigEvaluator.cpp:6022-6029).
- Interpolator weights appear after the walk (11667-11675).
- Property chains run before exec (10202-10239).

The biped hidden-driver pattern is confirmed for the spine_05-referenced readers (Biped.usda:2076-2149).

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:6022-6029`; `libs/rigExec/rigEvaluator.cpp:11667-11675`; `libs/rigExec/rigEvaluator.cpp:10202-10239`; `libs/rigExecMath/solvers.cpp:1047-1055`; `examples/biped/Biped_psd.usda:14-26`

### G6-spherical-pose-reader

**Spherical (elliptical cone) pose reader operator**

**Verdict:** Partial · **Severity:** major · **Effort:** L · **Confidence:** high · **Domain:** D11

UE features: `UE-spr-operator`, `UE-spr-halfplane`

**UE rigs.** RigUnit_SphericalPoseReader inputs:
- A driver bone and a local DriverAxis.
- A RotationOffset (Euler) applied to the driver's INITIAL local frame, under an optional parent that defaults to the driver's parent.

The unit maps the current axis into an azimuthal-equidistant 2D plane around the cone centre. The inner (active) and outer (falloff) regions are ellipses with 4 per-quadrant scale factors (+W, -W, +H, -H). Output is 1 inside the inner ellipse, 0 outside the outer one and 1 - dIn/(dIn + dOut) between; it is 0 at the back pole.

A falloff factor of 0 makes the reader one-sided (23 of 49 body readers). Factors above 1 are extrapolated, widening the outer extent to as much as 176 deg (30 readers). Every reader uses active 0.1 with scale factors 0.1 (a 1.8 deg core) and falloff 0.45 (81 deg), so in practice the output is a near-linear angular ramp. Readers: 49 body (45 Zebra + 4 Monster) and 11 face.

**usdRig today.** Closest equivalent: a RigExecPoseInterpolator with one RigExecPose, set up as follows:
- poseType swing, twistAxis = the DriverAxis axis.
- kernel linear, normalize false.
- rotationRadius = the outer half-angle in radians.
- rotation = the target direction.

This yields w = max(0, 1 - angle/r), a circular linear-ramp cone. The optional parent is emulated with the biped's hidden driver-joint pattern: a RigExecJoint under the reference parent, ParentConstraint-copied from the real bone. Back-pole-centred readers must be re-parameterized with the pose placed at the full-bend direction, because a 180-degree swing has no defined axis.

**Gap.** Missing reader features:
- Elliptical inner/outer regions with 4 per-quadrant scale factors.
- One-sided readers (a zero factor).
- Extrapolated factors above 1.
- The active-region plateau and the Flip width/height options.
- The back-pole = 0 rule.
- Direction-based azimuthal distance (usdRig uses the quaternion swing-angle distance, which reads larger off-plane).
- An explicit reference-parent relationship (the driver must be a joint or control, measured against its nearest provider ancestor).

The output is also published only after the entire pose walk, so it can drive BlendInput weights but nothing inside the pose walk or the scalar chains.

**Porting impact.** The 60 readers cannot be ported 1:1. Circular readers (for example shoulder_up with factors 1x4, head with 1.1x4) approximate within a few percent. Skewed or one-sided readers need per-reader re-fitting and still leak on the excluded side: knee, elbow, forearm twist, spine fwd, clavicle up/dn, shoulder bk and thigh-in. Corrective activation timing will visibly differ.

**Recommendation.** Add RigExecSphericalPoseReader to libs/rigExecSchema/schema.usda. It is Typed, not a mover, with these properties:
- rel rigExec:driver; rel rigExec:referenceParent (optional, defaulting to the namespace provider ancestor).
- double3 inputs:driverAxis; double3 inputs:rotationOffset (Euler deg, UE roll/pitch/yaw).
- float inputs:activeRegionSize with 4 active scale factors; float inputs:falloffSize with 4 falloff scale factors (no [0,1] clamp).
- bool inputs:flipWidthScaling / flipHeightScaling; bool inputs:enabled; float outputs:weight.

Implementation:
1. Port RemapAndConvertInputs, the 2-iteration ellipse distance and CalcOutputParam into a new libs/rigExecMath/poseReaders.{h,cpp}, with a UE numeric parity fixture.
2. Discover readers by type, as _DiscoverPoseInterpolators does (rigEvaluator.cpp:985-1001).
3. Schedule each reader as a new _PoseStep kind in the Kahn walk (rigEvaluator.cpp:5945-6035, walked at 10894), depending on the producers of its driver and parent. Publish into _resolvedInputs so later constraints, offset movers and late chains in the same walk read it.
4. Add RigExecBakedStepKind::PoseReader in bakedPose.cpp / bakedSchedule.cpp.
5. Add the builder API (rigBuilder.h), the python/rigexec facade and a Shape Editor monitor.

**Evidence:** `libs/rigExecSchema/schema.usda:1809-1926`; `libs/rigExecSchema/schema.usda:1928-2075`; `libs/rigExec/rigEvaluator.cpp:2494-2529`; `libs/rigExec/rigEvaluator.cpp:2773-2784`; `libs/rigExecMath/rbf.h:103-122`; `libs/rigExecMath/rbf.h:193-207`; `libs/rigExec/rigEvaluator.cpp:11666-11675`; `examples/biped/Biped.usda:2076-2149`

**Verification (holds).** I checked the code and the verdict holds.
- The driver must be a single RigExecJoint or RigExecControl (rigEvaluator.cpp:2494-2507).
- It is measured against the nearest provider ancestor (2513-2529) as delta = restLocal^-1*local (2773-2775).
- A swing pose uses the angle between swing quaternions (rbf.cpp:437-447; rbf.h:193-207). The linear kernel is max(0,1-d/r) (rbf.h:147-149).
- Translation is not measured (2552-2562). Weights are published only after the pose walk (11667-11675).
- Each pose publishes its own float, and a BlendInput follows exactly one connection (moverGraph.h:319-378), so an ellipse cannot be assembled from several poses.
- No cone, ellipse or spherical reader exists under another name; catalog R2-no-cone-readers agrees and I found none.

Authoring notes the row should mention:
- rigExec:normalize must be false; with normalize on, a single pose always reads 1 (schema.usda:1896-1903).
- rotationRadius must be authored, because radius 0 gives a spike (rbf.h:142-149).
- The baked step enum lives in bakedProgramImpl.h:559-575, not bakedPose.cpp. The existing PoseInterpolator step is at bakedPose.cpp:1714-1740.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:2494-2529`; `libs/rigExec/rigEvaluator.cpp:2773-2784`; `libs/rigExecMath/rbf.cpp:437-447`; `libs/rigExecMath/rbf.h:142-149`; `libs/rigExecSchema/schema.usda:1896-1903`; `libs/rigExec/rigEvaluator.cpp:11667-11675`; `libs/rigExec/bakedProgramImpl.h:559-575`

### G6-thigh-readers

**Thigh readers: fwd/ot/bk/alternate/thigh-in with cross-side coupling**

**Verdict:** Partial · **Severity:** major · **Effort:** S · **Confidence:** medium · **Domain:** D11

UE features: `UE-spr-zebra-thigh`

**UE rigs.** 12 pelvis-referenced thigh readers:
- fwd: plus a Remap(0.5..1) that writes thigh_fwd_ext and adds def_thigh_in offsets.
- ot: curve only; its ModifyTransforms carry identity offsets.
- bk: thigh_bk_r copy-pastes thigh_ot_r's offset.
- alternate up: joint offsets only; its SetCurveValue never executes.
- thigh-in: a narrow reader (outer about 41-53 deg) with cross-side coupling, so each leg moves both def_thigh_in helpers.

def_thigh_in accumulates offsets from 6 readers.

**usdRig today.** The pelvis reference is the thigh's namespace parent, so no driver joint is needed. Curves go through approximated RBF readers. Once offset movers exist, cross-side coupling is plain wiring (one reader output connected to movers on both sides), and accumulation follows mover order.

**Gap.** Same as the shoulder/clavicle and elbow/knee rows. The narrow, skewed thigh-in reader (factors 0.5-0.65) is far from circular.

**Porting impact.** Hip crease and inner-thigh helper motion are lost. thigh_up/ot/bk/fwd_ext morph timing is approximate.

**Recommendation.** No new engine feature beyond the reader, offset mover and late chain. The converter must follow exec links: drop SetCurveValue_47/48 and the identity ModifyTransforms_3/4, and keep the as-authored thigh_bk_r offset for parity.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:2513-2529`; `README.md:199-220`; `libs/rigExec/rigEvaluator.cpp:11666-11675`

**Verification (holds).** The verdict holds.
- The pelvis is the namespace-parent provider used for measurement (rigEvaluator.cpp:2513-2529).
- Cross-side coupling is just two constraints whose weights connect to one float (defaultWeight accepts a single float connection; 3632-3650).
- Accumulation follows reverse-sibling post-order (README.md:199-220).

The narrow, skewed thigh-in reader and the pose-derived weights keep this at Partial/major.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:2513-2529`; `libs/rigExec/rigEvaluator.cpp:3632-3650`; `README.md:199-220`

### G6-threshold-remap

**Thresholded secondary correctives (clamped Remap of pose-reader output)**

**Verdict:** Partial · **Severity:** major · **Effort:** L · **Confidence:** high · **Domain:** D11

UE features: `UE-threshold-remap-secondary`

**UE rigs.** result = lerp(TMin, TMax, clamp((v - SMin)/(SMax - SMin), 0, 1)) applied to reader outputs:
- SMin 0.4 for elbow and knee: produces the *_squash curves.
- SMin 0.5 for thigh fwd: produces thigh_fwd_ext.

The same remapped value drives both the curve and 22 extra offsets.

**usdRig today.** FloatMathMover has remap (source range only, unclamped) and clamp, so a remap-then-clamp chain reproduces the arithmetic on a float. However, property chains run before exec and _resolvedInputs is cleared first, so a chain input connected to a pose output reads the authored 0.

**Gap.** Missing: a post-pose ('late') scalar chain, and a single remapRange op with a target range and a clamp. Today this takes remap, clamp, multiply and add movers.

**Porting impact.** Squash and extension secondary correctives, both morphs and joint slides, cannot be driven from the primary readers.

**Recommendation.** Classify property chains whose inputs transitively read reader or PoseInterpolator outputs as 'late' (dependency sort at rigEvaluator.cpp:5240-5374). Evaluate a late chain inside the pose walk right after its producer when a pose-domain op consumes it, or right after _EvaluatePoseInterpolators (rigEvaluator.cpp:11675) when only BlendInputs consume it.

Add a 'remapRange' op to propertyMath.{h,cpp} and schema.usda:1208-1213. Split the baked property-chain step (bakedProgram.cpp:2256-2263) into early and late steps.

**Evidence:** `libs/rigExecMath/propertyMath.cpp:33-61`; `libs/rigExecSchema/schema.usda:1197-1214`; `libs/rigExec/rigEvaluator.cpp:10202-10216`; `libs/rigExec/rigEvaluator.cpp:8829-8930`; `libs/rigExec/moverGraph.h:319-378`

**Verification (holds).** The verdict holds.
- Remap is (v-min)/(max-min) with no target range and no clamp (propertyMath.cpp:51-55).
- Chains evaluate before exec, after _resolvedInputs.Clear() (rigEvaluator.cpp:10214-10239).
- Interpolators publish only into _resolvedInputs and movedProperties after the walk (2720-2723, 11675).

So a chain whose input connects to a pose output reads the authored 0 through the connection fallback (moverGraph.h:369-376).

Verifier evidence: `libs/rigExecMath/propertyMath.cpp:51-55`; `libs/rigExec/rigEvaluator.cpp:10214-10239`; `libs/rigExec/rigEvaluator.cpp:2720-2723`; `libs/rigExec/moverGraph.h:369-376`

### G6-material-curves

**Curves routed to material parameters (eye material)**

**Verdict:** Missing · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D14

UE features: `UE-material-curves`, `UE8-curve-flags-material-routing`

**UE rigs.** Curves flagged bMaterial in AnimCurveMetaData set the same-named scalar parameter on all component materials each frame; a parameter not set in a frame resets to its default. pupil_dilation, cornea_size and highlight_* feed MI_eye_Zeb. Separately, a Sequencer material track animates Eyes-slot material parameters directly.

**usdRig today.** A property chain may target any float attribute under the asset, for example a UsdShadeShader input, and the result is published in movedProperties. The imaging bridge, however, forwards only points, normals and extent, so the renderer never sees the value. There is no name-based routing. Direct sequencer-style animation needs no rig: it is plain USD time samples on UsdShade inputs.

**Gap.** Missing: publication of scalar rig results as material parameters or primvars.

**Porting impact.** Pupil dilation, iris size and highlight controls have no visual effect.

**Recommendation.** Preferred: let float results that target a constant 'primvars:<name>' on a Gprim be published as Hydra primvars. That means extending the filter at rigExecImaging/bridge.cpp:1385-1400 and the primvar publication in sceneIndices.cpp; the eye material then reads the value through UsdPrimvarReader_float.

Alternative: publish UsdShadeShader input overrides through a material-network override in the results scene index. This is heavier because it dirties the materialNetwork.

**Evidence:** `libs/rigExecImaging/bridge.cpp:1385-1400`; `libs/rigExecImaging/snapshotStore.h:238-247`; `libs/rigExec/rigEvaluator.cpp:3308-3314`; `libs/rigExec/rigEvaluator.cpp:11658-11663`

**Verification (holds).** The verdict holds for rendering: only points, normals and extent become Hydra data (bridge.cpp:1396-1399).

One nuance: scalar results are kept in snapshot movedFloats and exposed through RigExecImaging_GetMovedFloats (bridge.cpp:1400-1410; registry.h:257). A host or tool could forward them to materials, but Hydra itself never sees them.

Verifier evidence: `libs/rigExecImaging/bridge.cpp:1385-1410`; `libs/rigExecImaging/registry.h:257`

### G6-blend-weight-range

**Signed and above-one morph weights (squash/Squetch, 0..2 face curves)**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D12 · *(analyst said Partial / major)*

UE features: `UE6-squetch-curves`

**UE rigs.** Signed and overdriven curve values:
- mouth_squash and muzzle_squash = clamp(0.01*channel, -2, 2), and the body Squetch curve, are signed.
- Expression Shape Logic and lip roll/puff curves reach 2; brow curves can reach 200.

UE applies morph weights linearly, so a negative weight inverts the shape and a weight above 1 extrapolates it.

**usdRig today.** FloatMathMover chains compute the signed value correctly, but BlendInput clamps w to [0, lastActivation] (spec v0.1): negative values vanish and values above 1 cap at the last sample.

Workaround:
- Author an extra BlendSample at activation 2 whose UsdSkelBlendShape carries doubled offsets.
- Split each signed channel into two BlendInputs (the shape and a negated-offset copy), driven by clamp(+v) and clamp(-v) chains.

**Gap.** RigExecBlendInput has no policy for extrapolated or negative weights.

**Porting impact.** Without duplicated shape data, the squash (negative) half of the mouth and muzzle Squetch shapes and every overdriven face pose above 1 are lost.

**Recommendation.** Add uniform token rigExec:weightRange = "clamp" | "extrapolate" to RigExecBlendInput (schema.usda:1755-1766). In extrapolate mode, RigExecSumBlendChannels (moverGraph.cpp:1868-1874, including the sparse path) would:
- for w < 0, use d = w * d_first / a_first (mirror through the implicit zero sample);
- for w > the last activation, extrapolate the last segment.

Update spec section 7.3 (docs/spec.md:1323) and the parity tests.

**Evidence:** `libs/rigExec/moverGraph.cpp:1868-1874`; `libs/rigExec/moverKernels.cpp:411-427`; `docs/spec.md:1313-1323`; `libs/rigExecSchema/schema.usda:1755-1807`

**Verification (corrected).** The verdict holds but the severity is overstated: an exact, converter-only workaround exists.
- Weights from 1 to 2: add a BlendSample at activation 2 that references a UsdSkelBlendShape with doubled offsets. Piecewise-linear in-betweens (moverGraph.cpp:1867-1880) then reproduce linear extrapolation exactly up to 2.
- Negative weights: add a twin BlendInput whose sample has negated offsets, driven by a clamp(-v,0,2) FloatMathMover chain. The squash channels are float animation channels, so they are available before exec.
- Sparse samples keep the duplicated data small (schema.usda:1783-1803).

What remains is authoring volume and UE's |w|<=5 range (SkeletalRender.cpp:26), which the rigs never reach. The body Squetch value is G5's gap. Minor per the 'workaround exists' rule.

Verifier evidence: `libs/rigExec/moverGraph.cpp:1867-1880`; `libs/rigExecSchema/schema.usda:1783-1803`; `libs/rigExecMath/propertyMath.cpp:33-61`; `<UE>/Source/Runtime/Engine/Private/SkeletalRender.cpp:24-26`

### G6-body-corrective-morphs

**Pose-driven body corrective morph targets (curve name == morph name)**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D12

UE features: `UE-body-corrective-morphs`

**UE rigs.** CR_Zebra_Deform writes 50 curves. 49 of them drive the same-named morph targets on SKM_Zebra with weight = curve value; head_side_dn_r has no morph. The Squetch curve can be negative.

**usdRig today.** The body mesh points get one RigExecBlendShapeMover. Each corrective gets one RigExecBlendInput whose RigExecBlendSample references a UsdSkelBlendShape (sparse offsets, the form UE's USD export produces). Each BlendInput.inputs:weight connects to its reader or pose output, following the biped PSD pattern. Explicit connections replace UE's implicit name matching.

**Gap.** Negative Squetch weights clamp to 0, and weights above the last activation clamp. The converter must author the name-to-channel connections itself.

**Porting impact.** The negative (compression) side of Squetch is lost. Everything else is faithful once the readers exist.

**Recommendation.** Use G6-blend-weight-range. The converter should author one BlendInput per curve, named after the curve.

**Evidence:** `libs/rigExecSchema/schema.usda:1737-1807`; `libs/rigExec/moverGraph.cpp:1345-1381`; `libs/rigExec/moverGraph.cpp:1868-1874`; `libs/rigExec/rigEvaluator.cpp:12123-12129`; `examples/biped/Biped_psd.usda:2783-2793`

**Verification (holds).** The verdict holds.
- The channel weight is clamped to [0, lastActivation] (moverGraph.cpp:1867-1870 in the working tree; spec.md:1323).
- On the UE side, negative morph weights really are applied: renderers test |w| against a small threshold and a maximum of 5 (SkeletalRender.cpp:24-26; SkeletalRenderGPUSkin.cpp:3071).

The Squetch value itself (chain stretch - 1) is G5's gap. The negative-side workaround is covered under G6-blend-weight-range.

Verifier evidence: `libs/rigExec/moverGraph.cpp:1867-1870`; `docs/spec.md:1323`; `<UE>/Source/Runtime/Engine/Private/SkeletalRender.cpp:24-26`; `<UE>/Source/Runtime/Engine/Private/SkeletalRenderGPUSkin.cpp:3071`

### G6-brow-squeeze

**Brow squeeze pad with skull-local X-follow null**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D14

UE features: `UE6-brow-squeeze`

**UE rigs.** Squeeze Null (under skull) follows the average X of Brow Main L and R in skull-local space, with maintain offset and an X-only filter. The Squeeze pad has a 45-deg offset and limits X, Z in [0,200] and Y = 0. brow_squeeze_r = clamp(0.01*tz) and brow_squeeze_l = clamp(0.01*tx): the wiring is crossed.

**usdRig today.** A RigExecPositionConstraint with two equal sources, an authored offset and affectTranslation X only. Its masks and offsets act on asset-space origins, so it matches UE only when the skull's local X is aligned with asset X. The curves are chains, and the crossed wiring is reproduced as authored.

**Gap.** Missing: local-space constraint masks, limits, and the avar read.

**Porting impact.** If the skull is rotated relative to asset axes, the squeeze pad can drift off the brow centre; otherwise the impact is small.

**Recommendation.** Add uniform token rigExec:constraintSpace = asset|parentLocal to RigExecConstraint (schema.usda:930-997), and apply masks and offsets in the target's parent frame in the solvers.cpp kernels. Limits come from G3 and the pad read from G6-control-channel-read.

**Evidence:** `libs/rigExecMath/solvers.cpp:833-850`; `libs/rigExecSchema/schema.usda:962-979`; `libs/rigExecSchema/schema.usda:33-47`

**Verification (holds).** The verdict holds: position constraint masks act on asset-space origin axes (solvers.cpp:844-850), and there are no local-space variants (catalog R1-no-local-space-constraints).

Verifier evidence: `libs/rigExecMath/solvers.cpp:833-850`; `libs/rigExecSchema/schema.usda:962-979`

### G6-cross-rig-curve-handoff

**Curve hand-off between rig instances (face module to post-process deform rig)**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** medium · **Domain:** D11 · *added by verifier*

UE features: `UE-deform-exec-order`, `UE7-deformer-curves-registry`, `UE8-curve-morph-name-bugs`

**UE rigs.** Two rig instances share values only through the animation curve buffer:
- The face module inside MR_Zebra (the animator/sequencer rig) writes head_* and *_deformer curves.
- AnimBP_Zebra runs CR_Zebra_Deform as a post-process with bTransferInputCurves=true. It reads those curves with GetCurveValue to drive the Optimus deformers and overwrites the corrective curves.

**usdRig today.** Works only if both rigs are merged into one RigExecRoot, via sublayer or reference under <rig>/Movers, so the float attributes flow through that evaluator's _resolvedInputs. Separate RigExecRoots each clear and fill their own _resolvedInputs, so a connection from rig B to rig A's chain output resolves to A's authored value, not its computed one. Cross-rig mover targets are rejected, and the spec's explicit multipass coordinator is not built.

**Gap.** There is no scalar channel between separately evaluated rigs. The port must fold MR_Zebra's face logic and CR_Zebra_Deform into a single rig.

**Porting impact.** This is a structural decision, not a loss. Keeping the UE split, with animator rig and post-process rig as separate RigExecRoots, silently zeroes every deformer and corrective input computed in the other rig.

**Recommendation.** Converter: emit the deform rig as a sublayer of the character's RigExecRoot. Engine: add a compile diagnostic when a scalar input's connection chain leaves the rig asset root (rigEvaluator.cpp:525-569 already walks the chain), so the silent authored-value fallback is reported.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:10214-10239`; `libs/rigExec/rigEvaluator.cpp:3308-3313`; `docs/spec.md:335`; `docs/spec.md:1136-1146`; `libs/rigExec/moverGraph.h:319-378`

### G6-curve-only-readers

**Curve-only body readers (ankle, head, forearm twist, spine forward)**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** medium · **Domain:** D11

UE features: `UE-spr-zebra-ankle`, `UE-spr-zebra-head`, `UE-spr-zebra-forearm-twist`, `UE-spr-zebra-spine-fwd`

**UE rigs.** 15 readers that only write curves:
- ankle up/dn: elliptical outer extents of about 73 deg by 41 or 89 deg.
- head up/dn/side_dn: factors 1.1x4; head_side_dn_r has no morph.
- forearm pos/neg: on lowerarm_twist_01, back-pole centre, opposite excluded half-planes.
- spine_01-03_fwd: one-sided, height extent about 61 deg.

**usdRig today.** A single-pose linear RigExecPoseInterpolator (normalize off) connected to BlendInput.inputs:weight covers these. Forearm twist is better expressed with poseType twist about X and poses at +theta and -theta, since usdRig's swing-twist split is exact. The one-sided spine reader is approximated by placing the pose at the forward-bend direction.

**Gap.** The ellipse aspect ratios and half-plane exclusions can only be approximated.

**Porting impact.** These correctives fire with slightly different falloffs and need per-reader tuning. The result is acceptable for review but not exact parity.

**Recommendation.** Use G6-spherical-pose-reader for exact parity. Until then, the converter can fit RigExecPose.rotationRadius per reader by sampling the UE kernel, using the rbf_evaluate / rbf_fit_width Python bindings.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:2604-2609`; `libs/rigExecMath/rbf.h:118-122`; `libs/rigExecSchema/schema.usda:1896-1903`; `libs/rigExecSchema/schema.usda:1911-1920`; `examples/biped/Biped_psd.usda:2783-2793`

**Verification (holds).** The verdict holds: single-pose linear interpolators feeding BlendInputs work (Biped_psd.usda:2783-2793), and pose types are per pose (rigEvaluator.cpp:2604-2609).

One correction for forearm twist: the angle is 2*acos(|dot|), which folds at 180 degrees (rbf.h:193-195).
- A pose at the UE back-pole-equivalent of about +176 degrees leaks onto negative twists.
- Poses must sit at about +/-90 degrees with a 90-degree radius. That matches UE up to 90 degrees of twist and diverges above it.

This is still a workaround-level difference.

Verifier evidence: `libs/rigExecMath/rbf.h:193-195`; `libs/rigExec/rigEvaluator.cpp:2604-2609`; `examples/biped/Biped_psd.usda:2783-2793`

### G6-curve-registry

**Curves as named, animatable rig scalar channels (incl. deformer factor curves)**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** medium · **Domain:** D12

UE features: `UE7-deformer-curves-registry`, `UE1-curve-set`

**UE rigs.** Curves are named float elements in the rig hierarchy. The skeleton import brings 958 (Zebra), 800 (Biped) and 993 (Monster), mostly unused: CTRL_expressions, pose-reader names, Pose_0-9, MoveData_Speed. The deformer factors (head_squash, head_bend, head_twist, *_deformer) are curves that the face rig computes or animations key; Zeb_Face_Expressions keys 6 of the 7.

**usdRig today.** Any float attribute is an animatable channel (splines or time samples) and can be a connection target. Deformer factors can be custom floats on a curves scope under the rig, written by FloatMathMover chains. Baked keys authored on the same attribute are superseded by a chain result or a connection whenever the rig computes the value.

**Gap.** There is no curve registry: no discovery or metadata, and the Avar Editor lists only avars: and foot: prefixes. The only way to switch between baked and rig-computed values is composition.

**Porting impact.** Only about 150 rig-relevant curves need porting, so bulk import is pointless. Animators cannot browse curves as a set in the UI.

**Recommendation.** Adopt a convention of custom float 'curves:<name>' attributes on a <rig>/Curves scope. Add 'curves:' to RIG_PREFIXES in plugin/rigExecUsdview/avarEditorModel.py. The converter should skip unused skeleton curves and offer a 'curveSource = rig|baked' variant that deactivates the curve-writing chains.

**Evidence:** `plugin/rigExecUsdview/avarEditorModel.py:37-54`; `libs/rigExec/rigEvaluator.cpp:8829-8930`; `libs/rigExec/moverGraph.h:319-378`

**Verification (holds).** The verdict holds.
- RIG_PREFIXES is ('avars:', 'foot:') (avarEditorModel.py:37-46).
- A chain result supersedes the authored or keyed base (rigEvaluator.cpp:8916-8927).
- There is no curve schema or discovery.

Verifier evidence: `plugin/rigExecUsdview/avarEditorModel.py:37-54`; `libs/rigExec/rigEvaluator.cpp:8916-8927`

### G6-eye-controls

**Eye Main / Eye controls with shared Pupil/Iris channels**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D14

UE features: `UE6-eye-controls`

**UE rigs.** Eye Main L/R drives the eye_main socket and hosts Micro Vis. Eye L null is skull-parented and translation-only. The Eye L control is rotation-only (translation limits 0, scale 1) and drives eye_l. Pupil Size and Irisl Size are float channels on Eye L, with Eye R as an extra host so the eyes share them.

**usdRig today.** Controls, joints and nulls are authored statically. The channels are custom float avars on Eye L. The aim null is driven by an AimConstraint, as in example 08.

**Gap.** There is no rotation-only lock (no limits) and no multi-host channel. A connected copy on Eye R is only a local opinion: editing it does not change the value that consumers of Eye L's attribute read.

**Porting impact.** Animators can translate the eyes, and the shared channels appear only on Eye L.

**Recommendation.** RigExecControlAPI limits and locks (G3). Add a channel-host mechanism, for example 'rigExec:channelSource' metadata on a proxy attribute that the Avar Editor (plugin/rigExecUsdview/avarEditorModel.py) honours by writing through to the source attribute.

**Evidence:** `libs/rigExecSchema/schema.usda:33-47`; `plugin/rigExecUsdview/avarEditorModel.py:37-54`; `examples/08_AimEyes.usda:104-123`; `libs/rigExecSchema/schema.usda:1021-1071`

**Verification (holds).** The verdict holds: ControlAPI has no limits or locks (schema.usda:33-47), and there is no channel-host concept. The Avar Editor discovers channels by prefix on the focus prim only (avarEditorModel.py:37-54).

Verifier evidence: `libs/rigExecSchema/schema.usda:33-47`; `plugin/rigExecUsdview/avarEditorModel.py:37-54`

### G6-face-control-hierarchy

**Face macro/direct controls (jaw, reverse jaw, skull tp, muzzle, mouth, lips, teeth, cheeks, nose, lid in/ot)**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D14

UE features: `UE6-jaw-reversejaw-skulltp`, `UE6-muzzle-mouth-lips-controls`, `UE6-teeth-cheek-nose`, `UE6-lid-in-ot-controls`

**UE rigs.** Construction spawns the face controls under Head Attach Null, bones (jaw, muzzle, nose, eye_main) or other controls, with offsets equal to bone globals. It also spawns helper bones (Jaw Const, Skull Const, Teeth Tp) that follow the controls. The controls drive bones, and Mouth/Lips feed New Parent. The Monster Nose hosts Sneer and Flare float channels. Controls use coloured shapes with shape offsets.

**usdRig today.** Everything is authored statically instead of spawned:
- RigExecControls nested under their controlling parent, with rest:space = bone global.
- Bone-parented controls use parent:space.connect to the joint (a static reparent).
- Joints are driven by ParentConstraints (biped pattern).
- Helper Const bones are RigExecJoints nested under the control.
- Float channels are custom float avars.

Procedural spawning becomes build-time authoring, consistent with the non-destructive, no-dynamic-topology design.

**Gap.** Missing:
- Guide shape offset/rotation and a custom shape library (only 6 primitives).
- Control limits and locks.
- Baking for connected-space providers: they refuse the baked program.

**Porting impact.** The rig works. Control shapes sit at their pivots and look different, and bone-parented controls fall back to dynamic evaluation.

**Recommendation.** Add guide:offset translate/rotate/scale to RigExecControl (schema.usda:167-222, noted in docs/biped-rig.md:181-184). Control limits belong to G3. Baking connected-space providers is the plans-doc P0 item.

**Evidence:** `libs/rigExecSchema/schema.usda:144-250`; `libs/rigExecSchema/schema.usda:345-357`; `libs/rigExec/computations.cpp:298-314`; `docs/biped-rig.md:180-184`; `docs/spec.md:56-58`; `docs/plans/evaluation-engine-gaps-vs-premo-libee.md:134`; `docs/plans/evaluation-engine-gaps-vs-premo-libee.md:303`

**Verification (holds).** The verdict holds.
- Guide offsets are not built (docs/biped-rig.md:180-184).
- ControlAPI has no limits (schema.usda:33-47).
- Connected-space providers refuse the bake (plans doc:134, :303).

Nesting controls under constraint-driven joints propagates through namespace (rigEvaluator.cpp:10690-10745); only solver-posed joints block propagation.

Verifier evidence: `docs/biped-rig.md:180-184`; `docs/plans/evaluation-engine-gaps-vs-premo-libee.md:134`; `libs/rigExec/rigEvaluator.cpp:10690-10745`

### G6-morph-normal-deltas

**Morph target normal deltas and pre-skin application**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D12 · *added by verifier*

UE features: `UE-body-corrective-morphs`, `UE-face-morph-inventory-zebra`, `UE-monster-morphs`

**UE rigs.** Every UE morph vertex carries a PositionDelta and a TangentZDelta (MorphTarget.h:22).
- Skinning adds both in bind pose before skinning. The normal delta is scaled by min(w,1) and renormalized (SkeletalRenderCPUSkin.cpp:647).
- Weights apply while |w| is between a small epsilon and 5 (SkeletalRender.cpp:24-26).
- The Monster USD export carries 79 BlendShapes.

**usdRig today.** Pre-skin application is expressible. A BlendShapeMover authored before the SkinMover sums deltas against the authored base, and the skin treats its incoming revision as the bind pose (catalog R2-skin-lbs).

Normal deltas are not. _ResolveBlendSampleLayout reads only UsdSkelBlendShape offsets and pointIndices; nothing in libs reads normalOffsets. Normals are recomputed from the final points, and only when the mesh authors normals.

**Gap.** Sculpted per-shape normal deltas (UsdSkelBlendShape normalOffsets) are ignored. Shading on correctives and face shapes comes only from recomputed geometric normals, or from none when normals are not authored.

**Porting impact.** Subtle shading differences on the 132/79 face and corrective shapes, for example creases whose normals were tuned in the sculpt. Positions are unaffected.

**Recommendation.** 1. Read normalOffsets into RigExecBlendSampleLayout (types.h; rigEvaluator.cpp:7091-7151).
2. Add uniform token rigExec:normalPolicy = recompute|blendOffsets on RigExecBlendShapeMover. blendOffsets would emit a normals revision with the same channel weights (clamped to 1 as UE does) ahead of the derived-normals pass (rigEvaluator.cpp:5521-5584).

**Evidence:** `libs/rigExec/rigEvaluator.cpp:7091-7151`; `libs/rigExec/rigEvaluator.cpp:5521-5540`; `libs/rigExecSchema/schema.usda:1783-1803`

### G6-postprocess-input-pose

**Deform rig running on arbitrary or baked input animation (post-process semantics)**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D11 · *added by verifier*

UE features: `UE-deform-exec-order`, `UE-body-corrective-morphs`, `UE-helper-shoulder-drive`

**UE rigs.** AnimBP_Zebra and Monster_PostAnimBP run CR_*_Deform on whatever pose the mesh receives (gameplay, retargeted or baked animation), with bResetInputPoseToInitial, bTransferInputPose and bTransferInputCurves all true.
- Zeb_Face_Expressions carries 371 bone tracks, including def_* helper bones, and 39 body-corrective curves.
- Readers measure the incoming pose.
- AdditiveLocal offsets add on top of the incoming helper tracks.
- SetCurveValue replaces the incoming corrective curves.

**usdRig today.** A control-less deform layer can pose RigExecJoints from keyed joint avars (joints carry avars), and interpolators measure final frames whatever posed them. There is no input-pose slot, however:
- Solver-bound joints ignore their avars, because their frames are supplied as overrides.
- Importing UsdSkel animation is a spec non-goal.
- Switching between rig-driven and baked-driven poses needs composition, such as a variant that deactivates the animator solvers and constraints.

**Gap.** There is no post-process mode that takes an external joint pose and runs only the corrective/deform layer. Baked helper-bone tracks and corrective curves in UE animation would double-apply or conflict with the rig's offsets and chains.

**Porting impact.** Playing the UE baked animations through the ported rig needs a converter pass: bake the body into joint avars, drop the def_* helper tracks and rig-computed curves, and select a baked-pose variant. Otherwise helper offsets are applied twice.

**Recommendation.** Document a 'poseSource = rig|baked' variant pattern next to R3-lod-variant (examples/ArmRig.usda). The baked variant deactivates animator solvers and constraints and keys joint avars. The converter strips def_* helper tracks and corrective/deformer curves from imported animations.

**Evidence:** `libs/rigExecSchema/schema.usda:327-336`; `docs/biped-rig.md:143-145`; `libs/rigExec/rigEvaluator.cpp:10189-10194`; `libs/rigExec/rigEvaluator.cpp:2741-2776`; `docs/spec.md:61`

### G6-pupil-iris

**Pupil / iris size channels to eye curves**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D14

UE features: `UE6-pupil-iris`

**UE rigs.** pupil_dilation = remap(Pupil Size, 0..200 -> 0..2) + 1. cornea_size = remap(Iris Size, 0..200 -> -0.1..0.2) + 0.225. Both are unclamped and are not morph targets. highlight_offset_x is an orphan write that never runs.

**usdRig today.** The channels are float, so FloatMathMover chains reproduce both formulas exactly: add, multiply 0.01, add 1 for pupil_dilation, and add, multiply 0.0015, add 0.125 for cornea_size. The results are float attributes.

**Gap.** The shared channel host is missing (G6-eye-controls), and the results have no consumer, because scalar rig results never reach Hydra or materials (G6-material-curves).

**Porting impact.** The values are computed but have no visible effect.

**Recommendation.** See G6-material-curves.

**Evidence:** `libs/rigExecMath/propertyMath.cpp:33-61`; `libs/rigExecImaging/bridge.cpp:1385-1400`

**Verification (holds).** The formulas are exact.
- pupil_dilation = 0.01*P + 1.
- cornea_size = 0.0015*I + 0.125.

The verdict remains Partial because no consumer receives the values (see G6-material-curves).

Verifier evidence: `libs/rigExecMath/propertyMath.cpp:33-61`; `libs/rigExecImaging/bridge.cpp:1385-1410`

### G6-twist-reader

**Twist / swing-twist reader node (twist value, signed twist split)**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D11

UE features: `UE4-get-node-twist-value`, `UE-swing-twist-reader-unused`

**UE rigs.** Get Node Twist Value returns the twist quaternion of (init^-1 * current local rotation) about an axis; it is used inside LimbTwist's Blend Twist.

CR_Zebra_Deform also contains an authored but never-executed subgraph: SwingTwist about X, then Euler ZYX .X, then an identity Remap, then a Greater>0 Branch that writes forearm_pos_l or forearm_neg_l.

**usdRig today.** Swing-twist decomposition exists only internally: TwistDistribution's _SwingTwist about the start X axis, and the RBF twist/swing metric. The LimbTwist use case is covered by RigExecTwistDistribution. No node publishes a twist angle, a twist quaternion or a signed positive/negative split.

**Gap.** Missing: a standalone twist/swing reader node with float outputs.

**Porting impact.** The dead subgraph should not be ported, and the LimbTwist-internal use maps to TwistDistribution, so impact is small. Custom twist-driven logic, and the signed-reader design as an alternative to half-plane readers, cannot be rebuilt.

**Recommendation.** Add a RigExecTwistReader schema with:
- rel rigExec:driver; rel rigExec:referenceParent.
- uniform token rigExec:twistAxis = X|Y|Z (or double3 inputs:twistAxis).
- float outputs:twistAngle (degrees, signed, in (-180,180]); outputs:swingAngle; outputs:twistPositive and outputs:twistNegative.

Share the pose-reader step kind and libs/rigExecMath/poseReaders.cpp, and reuse RigExecRbfSwingTwist (rbf.h:206-207).

**Evidence:** `libs/rigExecMath/solvers.cpp:272-290`; `libs/rigExecMath/rbf.h:197-207`; `libs/rigExecSchema/schema.usda:654-693`; `docs/biped-rig.md:190-193`

**Verification (holds).** The verdict holds, with one nuance. A twist-type RigExecPose with a linear kernel publishes max(0,1-|theta-theta0|/r), which is an affine remap of the twist angle over a window (rbf.cpp:437-447; rbf.h:147-149). Two poses at +/-theta0 give a signed positive/negative split.

However, it publishes only after the pose walk (rigEvaluator.cpp:11675), folds at 180 degrees (rbf.h:193-195), and no node exposes the raw angle or quaternion. Minor is right, since the UE use is dead code or covered by TwistDistribution.

Verifier evidence: `libs/rigExecMath/rbf.cpp:437-447`; `libs/rigExecMath/rbf.h:193-207`; `libs/rigExecMath/solvers.cpp:272-290`; `libs/rigExec/rigEvaluator.cpp:11667-11675`

### G6-face-function-reuse

**Face logic as reusable functions with table-driven loops**

**Verdict:** Divergent-by-design · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D17 · *added by verifier*

UE features: `UE6-blink-logic`, `UE6-blink-extend-open-rotate`, `UE6-soft-eyes`, `UE6-lip-tweakers`, `UE6-corner-logic`, `UE6-expression-shape-logic`, `UE6-brow-micro-curves`

**UE rigs.** CRM_Zebra_Face has 23 local RigVM functions called with parameters, for example Blink Logic x8, Corner Logic x2, Expression Shape Logic x8 (x14 on Monster) and Brow Micro x6. Inside them, ArrayIterator loops walk quaternion tables and bone arrays (Soft Eyes: 8 loops x 12 bones) and struct weight rows (20-lip tweaker loop).

Editing one table value (for example Lid Tp Blink Rotations[1]) changes every call site at once.

**usdRig today.** usdRig rejects a general-purpose node graph (spec non-goal). Reuse comes from USD composition:
- Class or reference instancing. The biped mirrors sides with references, because inherits also composes animation.
- Procedural authoring in the C++/Python builder.

Tables must be expanded into one static mover per (bone, layer) and one chain per call.

**Gap.** There is no function, loop or table construct. About 300 face offset movers and about 150 FloatMathMovers become static prims. A table edit means regenerating many prims unless the tables are kept as source data.

**Porting impact.** Authoring volume and maintainability only; results are unaffected. Tuning a lid table in usdRig requires the converter or builder to rerun.

**Recommendation.** Keep the UE tables as data (for example custom quatf[] attributes on a <rig>/Tables scope, or a JSON sidecar). Generate the per-bone movers with a python/rigexec builder helper, e.g. rigexec.face.build_layer(table, bones, weight_attr). Instance per-side logic with references to a component layer, not inherits.

**Evidence:** `docs/spec.md:60`; `docs/biped-rig.md:246-252`; `libs/rigExecRigging/rigBuilder.h:791-804`

### G6-deformer-factor-remap

**Curve -> deformer factor conversion**

**Verdict:** Implemented · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D11

UE features: `UE7-deformer-curve-factor-remap`

**UE rigs.** StretchFactor = (c+1)/2 for head and skull top, and (1-c)/2 for muzzle and mouth. Bend and twist factors pass the curve through unchanged. Nothing is clamped.

**usdRig today.** The inputs are control-derived curves, available pre-exec. A FloatMathMover chain writing a float deformer input reproduces the conversion exactly: add(1), multiply(0.5), or multiply(-1), add(1), multiply(0.5). The unclamped remap matches usdRig's no-clamp design.

**Gap.** No gap in the arithmetic. The consuming bend/squash/twist deformer is missing (group G7).

**Porting impact.** None beyond G7.

**Recommendation.** When G7 adds the deformer schema, make inputs:stretchFactor, bendFactor and twistFactor float so these chains can target them directly.

**Evidence:** `libs/rigExecMath/propertyMath.cpp:33-61`; `libs/rigExecSchema/schema.usda:1204-1205`; `libs/rigExec/rigEvaluator.cpp:3570-3574`

**Verification (holds).** The verdict holds: the (c+1)/2 and (1-c)/2 factors are exact add/multiply chains, with no clamp by design (schema.usda:1204-1205). Curves keyed by animation are float and available before exec.

Verifier evidence: `libs/rigExecMath/propertyMath.cpp:33-61`; `libs/rigExecSchema/schema.usda:1204-1205`

### G6-expression-shape-logic

**Expression Shape Logic: bipolar float channel to two clamped curves**

**Verdict:** Implemented · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D11

UE features: `UE6-expression-shape-logic`

**UE rigs.** A bipolar float animation channel c (range +-200) produces Pos = clamp(0.01*c, 0, 2) and Neg = clamp(-0.01*c, 0, 2). It is used for lip all/mid tp/bt and sneer tp/bt L/R. The Monster adds nose sneer, nose flare and sticky, which use only the Pos output.

**usdRig today.** The inputs are float animation channels, so they map to custom float avars on the host control. Each call becomes two FloatMathMover chains, add(channel) -> multiply(+-0.01) -> clamp(0,2), targeting the two curve or BlendInput weights. The chains run pre-exec and the arithmetic is exact.

**Gap.** Weights between 1 and 2 are clamped at the BlendInput (G6-blend-weight-range). Authoring volume is large: 16-22 calls x 2 chains x 3 movers, with no function abstraction.

**Porting impact.** Behaviour is identical up to weight 1.

**Recommendation.** Add a builder/Python helper that emits a remap-clamp chain for a channel. Blend-weight extrapolation covers the 1..2 range.

**Evidence:** `libs/rigExecSchema/schema.usda:1192-1214`; `libs/rigExecMath/propertyMath.cpp:33-61`; `examples/biped/Biped.usda:3097-3157`

**Verification (holds).** The verdict holds.
- Float custom avars connect to float chain inputs, as the biped does (Biped_layered_left.usda:1268).
- add, multiply(+/-0.01) and clamp(0,2) are exact (propertyMath.cpp:33-61).
- Weights from 1 to 2 are handled by the exact extra-sample workaround (see G6-blend-weight-range).

Verifier evidence: `examples/biped/Biped_layered_left.usda:1254-1268`; `libs/rigExecMath/propertyMath.cpp:33-61`

### G6-eye-convergence

**Eye convergence blend of aim targets**

**Verdict:** Implemented · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D14

UE features: `UE6-eye-convergence`

**UE rigs.** Each eye's aim null translation = Interpolate(A, P, 1 - Convergence), where A is the Eye Aim position and P is the parallel-offset target (the aim null's initial offset projected under the current Eye Aim). Convergence is a 0..1 channel on Eye Aim.

**usdRig today.** Each eye's target Xform is nested under Eye Aim at its rest offset, which gives P. A RigExecPositionConstraint with source Eye Aim and inputs:defaultWeight.connect = Eye Aim's custom float avars:convergence produces P + c(A - P), which equals UE's Interpolate(A, P, 1 - c). The eye AimConstraints then read these targets.

**Gap.** None. The convergence value must stay within [0,1]; out-of-range values make the constraint pass through with a diagnostic.

**Porting impact.** None.

**Recommendation.** Converter authoring only. Optionally clamp the channel with a FloatMathMover so the envelope never leaves [0,1].

**Evidence:** `libs/rigExecMath/solvers.cpp:805-871`; `libs/rigExec/rigEvaluator.cpp:11015-11028`; `libs/rigExecSchema/schema.usda:1073-1085`; `examples/08_AimEyes.usda:104-123`

**Verification (holds).** I checked the math and it matches.
- The position constraint gives input + c*(A - input) = P + c(A-P) (solvers.cpp:844-850).
- UE gives A + (1-c)(P-A) = P + c(A-P), the same value.
- The nested target reproduces ProjectTransformToNewParent through namespace propagation.
- The weight connects to a float avar.

Verifier evidence: `libs/rigExecMath/solvers.cpp:805-871`; `libs/rigExec/rigEvaluator.cpp:11015-11028`; `examples/08_AimEyes.usda:104-123`

### G6-face-attach

**Face module slot and Head Attach Null follow (Monster Parent unconnected)**

**Verdict:** Implemented · **Severity:** minor · **Effort:** S · **Confidence:** medium · **Domain:** D14

UE features: `UE1-layout-face-slot`, `UE8-monster-face-parent-missing`

**UE rigs.** The face is one module (the CRM_*_Face runtime asset) under root or Spine, with a Root connector and an optional secondary Parent connector. Its world-level Head Attach Null is spawned at the head's construction transform. A maintain-offset ParentConstraint makes the null follow the Parent connector:
- Zebra: Parent -> head.
- Monster: Parent is unconnected, so the null stays frozen and the Monster face controls do not follow the head.

**usdRig today.** The face is a referenced or sublayered component under the rig. Head Attach Null is a top-level null whose rest equals the head joint's rest, driven by a RigExecParentConstraint to the head joint with an identity per-source offset (the maintain-offset value at construction). The Monster no-op is reproduced by not authoring the constraint.

**Gap.** No optional-connector semantics and no maintain-offset capture; offsets are authored. Both belong to G1 and G4.

**Porting impact.** No loss if the converter bakes the offset. The port must decide explicitly whether to reproduce the Monster's frozen face.

**Recommendation.** The converter authors the follow constraint only when the connector resolves, and flags the Monster's unconnected Parent in the port report.

**Evidence:** `libs/rigExecSchema/schema.usda:1115-1142`; `libs/rigExecMath/solvers.cpp:1047-1055`; `libs/rigExecSchema/schema.usda:292-300`

**Verification (holds).** The verdict holds. A ParentConstraint with an identity per-source offset reproduces a maintain-offset follow when the null's rest equals the head's rest (solvers.cpp:1047-1055).

Note: RigExecParentConstraint defaults affectScale to off (schema.usda:1132-1134), while the UE constraint filters all channels. Author affectScale on for parity.

Verifier evidence: `libs/rigExecSchema/schema.usda:1115-1142`; `libs/rigExecMath/solvers.cpp:1047-1055`

### G6-morph-inventory

**Per-mesh morph target inventories (Zebra, Zebra_Hi, Monster)**

**Verdict:** Implemented · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D12

UE features: `UE-face-morph-inventory-zebra`, `UE-monster-morphs`, `UE8-morph-set-mismatch`

**UE rigs.** Morph target sets per mesh:
- SKM_Zebra: 132 morphs (49 body + 83 face), including 16 '_c_' combination shapes and 4 lip backups.
- SKM_Zebra_Hi: 130 morphs with different backups and reversed order.
- SKM_Monster: 79 face-only morphs, including open_frown_c, model_edits, nose/lip_stick/lid_bt/lip_puff/ch_bt, and all_up/dn and mid_up/dn that are never driven.

**usdRig today.** Each mesh keeps its own UsdSkelBlendShape prims (the Monster USD export already contains 79). Per-mesh RigExecBlendSamples reference them through rigExec:blendShape with sparse offsets and pointIndices. Each mesh has its own RigExecBlendShapeMover and BlendInputs, so differing sets and orders pose no problem. Hi-mesh BlendInput weights connect to the Lo-mesh weights; the connection walk follows the chain to the published source. Combination shapes are ordinary targets; their weight logic is covered in G6-face-correctives.

**Gap.** No importer creates the BlendShapeMover/BlendInput/BlendSample setup from a UsdSkel export. UsdSkel in-between attributes are not read, but UE morphs have none, so nothing is lost.

**Porting impact.** No functional loss; the work is bookkeeping in the converter.

**Recommendation.** Add a Python helper, e.g. rigexec.blendshapes.from_usdskel(mesh, blend_shapes) in python/rigexec, that authors one named BlendInput and one sparse BlendSample per UsdSkelBlendShape and optionally connects LOD-pair weights.

**Evidence:** `libs/rigExecSchema/schema.usda:1768-1807`; `libs/rigExec/moverGraph.cpp:1350-1373`; `libs/rigExec/moverGraph.h:349-368`; `examples/04_BlendShapeFace.usda:1-13`

**Verification (holds).** The verdict holds.
- Sparse UsdSkelBlendShape samples exist (schema.usda:1783-1803; rigEvaluator.cpp:7091-7151).
- A chained BlendInput-to-BlendInput connection resolves to the published pose weight through the connection walk (moverGraph.h:349-368).
- Per-mesh movers make the differing sets trivial.

One caveat belongs in the missed-gaps list, not here: only offsets and pointIndices are read (rigEvaluator.cpp:7104-7118), so UsdSkel normalOffsets from the UE export are ignored.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:7091-7151`; `libs/rigExec/moverGraph.h:349-368`; `libs/rigExecSchema/schema.usda:1783-1803`

### G6-inert-nodes

**Present-but-inert UE nodes must not be ported**

**Verdict:** Not-applicable · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D11

UE features: `UE-deform-nonexecuted-nodes`

**UE rigs.** Nodes that exist in CR_Zebra_Deform but never take effect:
- SetCurveValue_47/48 have no exec link.
- The Branch that would run SetCurveValue_37/38 has no exec input, so they never run.
- ModifyTransforms_3/4 execute but carry identity offsets.
- ArrayToSRT -> Average is orphaned.
- There is an unresolved Inverse_1 template node.
- The TwistFactor_Value variable is unused.

**usdRig today.** usdRig has no 'present but not executed' state: every authored mover with rigExec:moves runs, and readers would be discovered by type. The dead UE nodes should therefore simply not be authored.

**Gap.** No engine gap. The port needs a converter that follows exec and data links rather than node presence, and none exists.

**Porting impact.** A naive node-by-node converter would author duplicate thigh_up_l/r and forearm curve writers that change results through last-writer order.

**Recommendation.** In a new tools/ueImport converter (RigVM text dump -> USD):
- compute exec reachability from BeginExecution and data reachability from executed sinks;
- drop identity ModifyTransforms, unresolved template nodes and orphan array nodes;
- list what was dropped in a port report.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:985-1001`; `README.md:199-220`; `tools/ (only bakeGizmoIcons.py, diagnoseSolverBinding.py, migrateRestToLocal.py, rigExecPose.cpp; no UE/RigVM importer)`

**Verification (holds).** The verdict holds: there is no engine gap, and no UE/RigVM importer exists (tools/ has only bakeGizmoIcons.py, diagnoseSolverBinding.py, migrateRestToLocal.py and rigExecPose.cpp).

The mapping claim that usdRig has no present-but-not-executed state is wrong:
- inputs:enabled=false is an exact pass-through (schema.usda:950-953; rigEvaluator.cpp:10990-10996).
- A mover with empty rigExec:moves is inert.
- active=false removes a prim.

A converter could therefore keep dead UE nodes as disabled movers for provenance instead of dropping them.

Verifier evidence: `libs/rigExecSchema/schema.usda:950-953`; `libs/rigExec/rigEvaluator.cpp:10990-10996`; `tools/ (directory listing)`

### G6-curve-morph-mapping

**Face curve -> morph mapping, unmatched curves and copy-paste curve bugs**

**Verdict:** Implemented · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D12

UE features: `UE6-curve-morph-mapping`, `UE8-curve-morph-name-bugs`

**UE rigs.** Curve counts: the Zebra face writes 87 curve names (76 match morphs) and the Monster face writes 107 (75 match). The non-morph curves are eye-material and deformer inputs.

Authoring bugs and oddities:
- lip_roll_in_bt_l is written by both Ot Bt and In Bt; the last write wins.
- wide_open_c_r is written twice and frown_wide_c_r never.
- squint_l/r and Monster all_up/dn and mid_up/dn are never written.
- Monster writes Zebra-only combination curves that have no morph.
- model_edits is set to a constant 1.

**usdRig today.** A curve is a float attribute: either BlendInput.inputs:weight itself or a custom float on a curves scope, written by a FloatMathMover chain.
- Last-writer-wins per target reproduces the overwrite bugs.
- Unmatched curves are floats with no consumer.
- Never-written morphs are BlendInputs left at weight 0.
- model_edits is a BlendInput with an authored weight of 1.

**Gap.** None semantically: usdRig binds by explicit connection by design, so curve names are not a binding key and the converter has to resolve them.

**Porting impact.** The rig can be ported as authored, or the two copy-paste bugs can be fixed deliberately.

**Recommendation.** The converter should report names written without a morph and morphs without a writer. Offer an optional 'fix known bugs' switch (frown_wide_c_r, lip_roll_ot_bt).

**Evidence:** `libs/rigExec/rigEvaluator.cpp:8829-8930`; `README.md:199-220`; `libs/rigExecSchema/schema.usda:1755-1766`

**Verification (holds).** The verdict holds.
- Last writer wins per target, in mover order (rigEvaluator.cpp:8860-8927; README.md:199-220).
- An unconsumed float is harmless.
- An unwritten BlendInput stays at its authored weight (schema.usda:1764).

Name-based binding is replaced by explicit connections, which is a converter concern only.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:8860-8927`; `libs/rigExecSchema/schema.usda:1755-1766`

## G7 — GPU deformers and Direct Mesh Control

usdRig already has the plumbing for layering deformers. Movers are prims under <Rig>/Movers, ordered by namespace, and each one revises the preceding points buffer in place. They share one envelope (a per-point weight lerp, which is exactly UE's lerp(P, P', mask)), and their inputs can be connected or revised by property chains. Pose-derived frames are available to the geometry phase, and derived normals and extent are maintained automatically. So UE's phase/group/enqueue ordering and the 'variables vs constants' split both map cleanly onto native USD constructs.

The core gap is that there is no parametric deformer at all. The mover op set is frozen at Matrix/Skin/BlendShape/VolumeCorrect/Smooth/Lattice/SurfaceProject/Ribbon/Curvenet/derived ops (moverGraph.h:56-73). The 16 Barr bend/twist/squash-stretch instances on Zebra and Monster, which drive all head/muzzle/mouth/skull cartoon deformation, therefore cannot be expressed. The lattice is no substitute, because it binds from rest points and applies a global Bernstein delta.

Two other gaps are major:
- **Control-to-factor wiring.** Control translation cannot drive a float factor today. Avars are double, while FloatMathMover and scalar connections are exact-typed float, and the PoseInterpolator ignores translation. The 'translate the squash control' animator workflow cannot be wired.
- **Normals.** Derived normals are a full geometric recompute with vertex interpolation only. UE preserves the authored-normal offset and deforms tangents. usdRig has no tangents, and faceVarying (hard-edge) normals pass through stale.

Skin-weight masks can be precomputed into dense StaticWeights, but no weight object or helper derives them from the SkinMover layout with UE's descendant flood.

GPU execution and the generic deformer-graph authoring model diverge by design (spec.md:60, 1184-1195, 1398). They cost only performance, and a small mapping table, because all 17 graphs reduce to 3 kernels plus a normals pass.

Direct Mesh Control is mostly covered in spirit by TouchPose: face regions bound to controls, picked against deformed points, and lit on a session-layer overlay. What it lacks is automatic skin-weight polygroup generation, multiple named layers per mesh (ik/fk), always-on surface gizmos and a per-control shape fallback. These are all minor, because DMC is disabled or unresolved in the shipped rigs.

The recommended path:
1. An abstract RigExecNonlinearMover (handle rel plus length/limits/factor) with Bend, Twist and SquashStretch ops, a posed-handle-frame input, SIMD kernels with support-skip, baked lowering and builder/Python adders.
2. A RigExecSkinMaskWeight weight object.
3. A preserve-offset, faceVarying-aware normals policy.
4. A float/double narrowing rule for scalar connections.

| Row | Verdict | Severity | Effort | Summary |
|---|---|---|---|---|
| [`G7-bend-deformer`](#g7-bend-deformer) | Missing | major | L | No bend mover or posed handle-frame input in the frozen RigExecRevisionOp set; ribbon CurveMover only runs pre-skin |
| [`G7-squash-stretch-deformer`](#g7-squash-stretch-deformer) | Missing | major | M | No banded squash/stretch kernel (ZBias, XYBias, sine bulge); VolumeCorrectMover is only a global AABB scale |
| [`G7-twist-deformer`](#g7-twist-deformer) | Missing | major | S | No per-point twist op about a handle axis; TwistDistribution only publishes joint frames, CurveMover ribbon approximates |
| [`G7-control-to-factor-remap`](#g7-control-to-factor-remap) | Partial | major | S | Double avars:tx/tz/rz cannot feed a float FloatMathMover remap (exact-type connections, no double math mover) |
| [`G7-normals-keep-input`](#g7-normals-keep-input) | Partial | major | M | Derived RecomputeNormals replaces authored normals; no offset preservation, faceVarying, primvars:normals or tangents |
| [`G7-zebra-monster-stacks`](#g7-zebra-monster-stacks) | Partial | major | M | Nulls, remaps, order and masks are expressible except the kernels; double-avar drivers blocked, nulls need rest:space |
| [`G7-derived-normals-optout`](#g7-derived-normals-optout) | Missing | minor | S | Documented rigExec:derived = off is never read; faceVarying normals fail and raise MoverFailed every frame |
| [`G7-dmc-polygroup-generation`](#g7-dmc-polygroup-generation) | Missing | minor | M | No generation of RigExecTouchRegion faces from dominant skin influences; regions are only painted or .touch-imported |
| [`G7-pose-derived-scalar-squetch`](#g7-pose-derived-scalar-squetch) | Missing | minor | M | No chain-length or stretch-ratio reader publishes a float after the pose walk, so the Squetch morph driver is lost |
| [`G7-add-deformer-layering`](#g7-add-deformer-layering) | Partial | minor | S | Layering maps to mover prims under Movers with inputs:enabled; only the deformer movers and builder adders are missing |
| [`G7-curve-bus`](#g7-curve-bus) | Partial | minor | M | Curves work as custom float attrs written by FloatMathMover; no declared curve element, discovery or stage-free consumer |
| [`G7-deformer-child-components`](#g7-deformer-child-components) | Partial | minor | S | Point movers take one target, so fan-out needs a mover and mask per mesh (e.g. SKM_Zebra_Hi); no tag-based exclusion |
| [`G7-deformer-handle-frame`](#g7-deformer-handle-frame) | Partial | minor | S | Nulls map to RigExecJoint under the head, but no point mover reads the posed frame; UE global nulls need rest:space |
| [`G7-deformer-param-sets`](#g7-deformer-param-sets) | Partial | minor | S | Per-character values become mover attributes with class/inherit defaults once the nonlinear movers exist |
| [`G7-deformer-runtime`](#g7-deformer-runtime) | Partial | minor | L | export_baked bakes final moved points but movers need a USD stage; standalone rigpack rejects movers, no anim-graph hook |
| [`G7-dmc-layer-registration`](#g7-dmc-layer-registration) | Partial | minor | M | RigExecTouchRegions has no layer token or enable flag and regions cannot overlap, so IK and FK sets cannot coexist |
| [`G7-dmc-surface-gizmo-display`](#g7-dmc-surface-gizmo-display) | Partial | minor | M | TouchPose lights only hover/selected regions outside paint mode, is usdview-only, and blocks skin picks while on |
| [`G7-factor-conventions`](#g7-factor-conventions) | Partial | minor | S | Connections, time samples and property chains already resolve inputs; only a schema for factor/length/angle is missing |
| [`G7-skin-weight-mask`](#g7-skin-weight-mask) | Partial | minor | M | No skin-derived weight object; masks must be baked offline into sparse RigExecStaticWeight (clamp), stale on repaint |
| [`G7-skinned-tangent-frame`](#g7-skinned-tangent-frame) | Partial | minor | M | SkinMover and BlendShapeMover write only points; no normal/tangent transport or normalOffsets, just RecomputeNormals |
| [`G7-deformer-graph-authoring`](#g7-deformer-graph-authoring) | Divergent-by-design | minor | M | By design there is no general node graph or user kernel; each deform function needs a fixed-signature C++ mover |
| [`G7-gpu-execution`](#g7-gpu-execution) | Divergent-by-design | minor | M | By design CPU scalar/SSE2 only with no GPU path; SIMD covers only weighted matrix and LBS, a performance-only cost |
| [`G7-stack-ordering`](#g7-stack-ordering) | Implemented | minor | S | Maps to Movers reverse-sibling post-order (After/Before = siblings above/below SkinMover); builder lacks a reorder API |
| [`G7-deformer-handle-guides`](#g7-deformer-handle-guides) | Missing | cosmetic | S | No deformer handle or band guide; cosmetic since UE debug draw is off, and RigExecPlaneWeight planes can mark the band |
| [`G7-dmc-rundmc`](#g7-dmc-rundmc) | Partial | cosmetic | S | Binding a RigExecTouchRegion via rigExec:touch:control gives pick/highlight; no one-step convert-to-surface-gizmo |
| [`G7-dmc-shape-resolution`](#g7-dmc-shape-resolution) | Partial | cosmetic | S | guide:shape has no 'surface region else fallback' option; primitive guide can already hide via unknown token or scale<=0 |
| [`G7-dmc-submesh-generation`](#g7-dmc-submesh-generation) | Divergent-by-design | cosmetic | S | By design no proxy meshes; TouchPose builds the patch from deformed points into a session-layer RigExecTouchOverlay |
| [`G7-skinning-influences`](#g7-skinning-influences) | Implemented | cosmetic | S | Maps to RigExecSkinMover LBS with any elementSize and no renormalization; UE's skinned tangent frame is not carried |
| [`G7-cache-geometry`](#g7-cache-geometry) | Not-applicable | cosmetic | S | Not applicable: revisions read the preceding buffer in place and rigExecReadPhase exposes snapshots, so no copy node |

### G7-bend-deformer

**Barr bend deformer (no parametric bend mover)**

**Verdict:** Missing · **Severity:** major · **Effort:** L · **Confidence:** high · **Domain:** D13

UE features: `UE7-dg-barr-bend`, `UE-deformer-factor-conventions`

**UE rigs.** Runs per vertex on the current skinned position P (component space). Steps:
1. p = inverse_affine(M)*P, where M is the handle transform.
2. Lower = sanitize(lb)*L and Upper = max(Lower+1e-4, sanitize(1-lt)*L), with sanitize = clamp to [1e-4, 1-1e-4].
3. Radian = lerp(-A, A, (f+1)/2) = f*A.
4. If |Radian| > 1e-4: R = (Upper-Lower)/Radian and theta = (clamp(z, Lower, Upper) - Lower)/R.
   - x' = x
   - y' = cos(theta)*(y-R) + R + adjY
   - z' = -sin(theta)*(y-R) + Lower + adjZ
   - Outside the band, adj = (cos, sin)*(z - limit). The mesh is unchanged below Lower and continues rigidly along the end tangent above Upper.
5. P' = M*p'. Output = lerp(P, P', w); vertices with w < 1e-4 are skipped.
Bend axis is local X, bend direction local Y, deformation along local Z. Every graph uses L=90, A=180 deg and lb=lt=0. There are 7 graphs; the Zebra uses 3 and the Monster 4.

**usdRig today.** No mover computes a position-dependent nonlinear deformation in a handle frame. The revision op set is frozen and dispatch is compile-time only; an unhandled op is rejected. The envelope half exists: the MoverAPI common envelope (a dense per-point weight object or defaultWeight) blends the full-strength candidate against the preceding revision exactly as UE's lerp does.

Closest stand-ins are poor:
- LatticeMover binds from REST points, clamps to the rest-cage AABB and adds a global Bernstein delta, so it cannot follow the skinned head or produce a circular arc with rigid continuation.
- A TwistDistribution/SplineIk joint chain plus SkinMover weights would need re-skinning and gives a different look.

MatrixMover can apply only one affine map.

**Gap.** Four pieces are missing:
- a bend schema and kernel;
- a point-mover input carrying the POSED handle frame (MatrixMover only receives computeMatrix, the rest->posed delta);
- length, limit and max-angle parameters;
- a rotation-arc kernel with rigid continuation beyond the band.

**Porting impact.** Translate-X on the Head/Skull Tp/Muzzle(/Mouth) Squash controls would have no visible effect. Head Bend, Muzzle Bend and Skull Tp Bend are lost on the Zebra, plus Mouth Bend on the Monster, so the signature cartoon head bend disappears.

**Recommendation.** Schema, in libs/rigExecSchema/schema.usda (Geometry movers):
- Add an abstract RigExecNonlinearMover (Typed, used with RigExecMoverAPI) with:
  - rel rigExec:handle: exactly one RigExecControl/RigExecJoint. Its posed frame is the deformation space: origin = start, row 2 (local +Z) = deformation axis. The read phase comes from rigExecReadPhase metadata, default final.
  - float inputs:length
  - float inputs:lowBound and inputs:highBound (UE LimitFromBottom/LimitFromTop)
  - float inputs:factor
- Add a concrete RigExecBendMover with float inputs:maxAngle in degrees (radian = factor*maxAngle).

Evaluator:
- Add RigExecRevisionOp::Bend to moverGraph.h.
- Extend _RevisionKindToken, RigExecApplyRevisionKernel, RigExecRevisionOpForSchema and RigExecAssembleParameters in moverGraph.cpp.
- Add a handle SdfPath plus phase to RigExecRevisionBinding and 'const GfMatrix4d *handleFrame' (posed asset-space frame from computePointFrame) to RigExecProviderValues, wired from the frame chains the way the MatrixMover transform is.
- Add a 'GfMatrix4d handle' field to RigExecMoverParameters (types.h).

Kernel:
- RigExecApplyBend in libs/rigExecMath/geometryKernels.cpp, with an SSE2 twin in simdKernels.cpp.
- Reproduce the HLSL exactly, including the sanitize clamp and the |radian| <= 1e-4 identity case.
- Keep Gf's row-vector convention, so UE matrix rows map 1:1 (row0 = bend axis, row1 = bend direction, row2 = capture direction, row3 = origin).
- Points stay asset/target-local, following the docs/curvenet.md:447-451 convention.

Integration:
- Validation: add the type to the pointsMoverTypes block in rigEvaluator.cpp.
- Baked lowering: handle-frame slot in bakedGeometry.cpp.
- Builder: RigExecMoverChain::AddBendMover in rigBuilder.h/.cpp and python/_rigexec.cpp.
- Tests: parity against a Python transcription of DG_Function_Bend.

**Evidence:** `libs/rigExec/moverGraph.h:56-73`; `libs/rigExec/moverGraph.cpp:162-197`; `libs/rigExec/moverGraph.cpp:690-808`; `libs/rigExec/moverGraph.cpp:819-864`; `libs/rigExec/moverGraph.cpp:882-919`; `libs/rigExecSchema/schema.usda:49-89`; `libs/rigExecSchema/schema.usda:1663-1679`; `libs/rigExecSchema/schema.usda:2139-2159`; `libs/rigExecMath/geometryKernels.cpp:340-397`; `docs/spec.md:1355-1366`

**Verification (holds).** No bend mover exists. RigExecRevisionOp is a frozen list with no nonlinear op (moverGraph.h:59-73), and RigExecRevisionOpForSchema maps only the existing schemas (moverGraph.cpp:882-919). The lattice kernel binds from rest points inside the rest-cage AABB (geometryKernels.cpp:341-397). The HLSL matches the UE summary (ZebraHeadBend kernels.hlsl:54-104).

The analyst missed one closer stand-in: CurveMover ribbon mode (moverGraph.cpp:731-758; geometryKernels.cpp:631-670). It interpolates rest-to-posed maps of a driver frame array per point using a float2 u coordinate, so a bent joint chain plus u = z/L could approximate a bend without re-skinning.

The verdict still stands, because the stand-in is not equivalent:
- It applies rest-to-posed maps, so it would have to run before the SkinMover. That changes results for jaw and lip vertices.
- It interpolates matrices linearly.
- The chain angles cannot be driven from a float curve (see G7-control-to-factor-remap).

Severity major is right: 3 Zebra and 4 Monster bends would be lost.

Verifier evidence: `libs/rigExec/moverGraph.h:59-73`; `libs/rigExec/moverGraph.cpp:690-808`; `libs/rigExec/moverGraph.cpp:882-919`; `libs/rigExecMath/geometryKernels.cpp:341-397`; `libs/rigExecMath/geometryKernels.cpp:631-670`; `libs/rigExecSchema/schema.usda:2077-2096`; `ue/<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraHeadBend_DeformerGraph/kernels.hlsl:54-104`

### G7-squash-stretch-deformer

**Squash/stretch deformer with sine bulge and Z/XY biases**

**Verdict:** Missing · **Severity:** major · **Effort:** M · **Confidence:** high · **Domain:** D13

UE features: `UE7-dg-squash-stretch`

**UE rigs.** Setup:
- s = max(2F, 1e-4); F = 0.5 is the identity.
- t = normalized z in [Lower, Upper] (sanitized limits).
- ZBias remaps t through a quadratic Bezier with middle control point (ZBias, 1-ZBias).

XY bulge:
- b = sqrt(1/s).
- XYBias splits anisotropy between x and y (a = lerp(1/b, 1, wl), B = 1/a).
- Bulge = (pi/2)*sin(pi*t').
- x' = x*((bx-1)*Bulge + 1), and likewise for y.

Z stretch:
- z' = max(0, z-Upper) + (zc-Lower)*s + min(Lower, z).
- The band is scaled, the part above the band translates, and the part below is unchanged.

Then P' = M*p' and the result is lerped by the mask.

L is 55 for the Zebra head/muzzle/skull and the Monster muzzle/mouth, 75 for the Monster head and 40 for the Monster skull top. XYBias = ZBias = 0.5 everywhere. There are 7 graphs, all used.

**usdRig today.** No kernel exists. VolumeCorrectMover applies a global uniform AABB-volume scale, and MatrixMover with a scale provider applies one affine scale. Neither produces a banded stretch with a sine-profiled volume-preserving bulge. A DQS SkinMover on scaled joints keeps squash, but only through joint scale, which requires re-skinning.

**Gap.** There is no banded squash/stretch kernel, and no ZBias Bezier remap, XYBias anisotropy or sine bulge profile.

**Porting impact.** Translate-Z on every squash control would do nothing, so the Head, Muzzle, Mouth and Skull-top squash/stretch deformations are lost (7 instances).

**Recommendation.** Add RigExecSquashStretchMover, inheriting RigExecNonlinearMover, with:
- inputs:factor using the UE encoding (ratio = max(2*factor, 1e-4), neutral 0.5), documented on the attribute;
- float inputs:xyBias = 0.5 and inputs:zBias = 0.5.
Add RigExecRevisionOp::SquashStretch and kernel RigExecApplySquashStretch in geometryKernels.cpp. It must reproduce QuadraticBezierInterpolation, the XYBias split and the bulge formula exactly. Add golden tests over ZBias/XYBias != 0.5, even though the rigs only use 0.5.

**Evidence:** `libs/rigExec/moverGraph.cpp:690-808`; `libs/rigExecSchema/schema.usda:2303-2315`; `libs/rigExecSchema/schema.usda:1681-1735`; `libs/rigExecMath/solvers.cpp:376-388`

**Verification (holds).** The HLSL matches the analyst's summary (ZebraHead kernels.hlsl:24-173): Bezier ZBias, XYBias split, a (pi/2)*sin bulge and the banded Z stretch.

I tried to refute Missing with a composite:
- MatrixMovers (p' = q + w(Tq - q), solvers.cpp:376-388). A scale-XY mover with envelope sin(pi*t) reproduces x*((b-1)*Bulge+1) exactly when the matrix scale is 1+(b-1)*pi/2.
- PlaneWeight bands measured on current points (schema.usda:1435-1450, 1500-1576).
- A translate mover for the part above the band.

The composite cannot be driven. The scales and translations must come from Control/Joint computeMatrix, which is rest-to-posed and so double-applies head motion after skinning (computations.cpp:399-415). They must also be computed from a float curve, which is blocked (see G7-control-to-factor-remap).

VolumeCorrect is a global AABB centroid scale (geometryKernels.cpp:34-57). Missing and major hold.

Verifier evidence: `libs/rigExecMath/geometryKernels.cpp:34-57`; `libs/rigExecMath/solvers.cpp:376-388`; `libs/rigExec/computations.cpp:399-415`; `libs/rigExecSchema/schema.usda:1435-1450`; `libs/rigExecSchema/schema.usda:1500-1576`; `ue/<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraHead_DeformerGraph/kernels.hlsl:24-173`

### G7-twist-deformer

**Barr twist deformer**

**Verdict:** Missing · **Severity:** major · **Effort:** S · **Confidence:** high · **Domain:** D13

UE features: `UE7-dg-barr-twist`

**UE rigs.** Steps:
1. Radian = f*A with A=135.
2. Lower = lb*L and Upper = (1-lt)*L. These are NOT sanitized.
3. theta = (clamp(z, Lower, Upper) - Lower)/(Upper-Lower)*Radian.
4. xy' = mul(p.xy, [[c,-s],[s,c]]), with HLSL row-vector multiplication. This is a rotation by -theta about local Z. z is unchanged.
5. Output = lerp by mask; vertices with w < 1e-4 are skipped.
The twist ramps linearly up to Upper and stays constant above it. L is 60 for the Zebra head and 175 for the Monster head. The curve is -rotZ/135, so the net mesh rotation is +rotZ degrees. Two of the three graphs are used; ZebraMuzzleTwist is unused.

**usdRig today.** As for bend: there is no nonlinear op. RigExecTwistDistribution distributes twist over JOINTS, not mesh points, and it would require re-skinning the head region to a twist chain.

**Gap.** There is no per-point twist kernel about a handle axis with a linear ramp between the limits.

**Porting impact.** Rotating the Head Squash control (yaw) would do nothing, so the Zebra and Monster lose their head-twist deformation.

**Recommendation.** Add RigExecTwistMover (inherits RigExecNonlinearMover, adds float inputs:maxAngle) and RigExecRevisionOp::Twist, with kernel RigExecApplyTwist in geometryKernels.cpp. Keep UE's sign convention (rotation by -theta), and document that factor = -rotZ/maxAngle yields +rotZ. Keep the unsanitized limits for parity, but define Upper <= Lower as identity rather than NaN. This reuses all the handle and baked plumbing from G7-bend-deformer.

**Evidence:** `libs/rigExec/moverGraph.h:56-73`; `libs/rigExec/moverGraph.cpp:690-808`; `libs/rigExecSchema/schema.usda:654-718`

**Verification (holds).** The kernel matches the analyst's summary:
- Limits are unsanitized.
- theta ramps linearly.
- mul(p.xy, R) is a rotation by -theta (ZebraHeadTwist kernels.hlsl:49-86).

No per-point twist op exists (moverGraph.h:59-73). RigExecTwistDistribution only publishes a frame array (schema.usda:654-718).

A TwistDistribution driving a ribbon-mode CurveMover (moverGraph.cpp:731-758) could approximate a linear twist ramp without re-skinning. It has the same limits as the bend workaround:
- It must run before skinning.
- Matrices are interpolated linearly.
- Nothing can drive it from the float curve.

Missing and major both hold.

Verifier evidence: `libs/rigExec/moverGraph.h:59-73`; `libs/rigExecSchema/schema.usda:654-718`; `libs/rigExec/moverGraph.cpp:731-758`; `ue/<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraHeadTwist_DeformerGraph/kernels.hlsl:49-86`

### G7-control-to-factor-remap

**Squash/bend/twist controls: local translate/rotate channels remapped into deformer factors**

**Verdict:** Partial · **Severity:** major · **Effort:** S · **Confidence:** high · **Domain:** D11

UE features: `UE6-squash-controls-curves`, `UE-deformer-curve-bus`

**UE rigs.** Controls:
- Head Squash: Circle_Thick, cyan, scale 3, parent Head Attach Null, offset T(0,0,150) (Monster 100).
- Skull Tp Squash: child of Head Squash.
- Muzzle Squash: parent bone muzzle.
- Mouth Squash (Monster only): parent bone muzzle.
The face rig reads the controls' LOCAL T.Z, T.X and Euler(ZYX).Z and remaps them without clamping:
- head_squash = remap(tz, -100..100 -> -1..1)
- head_twist = remap(rz, -135..135 -> 1..-1)
- head_bend = remap(tx, -100..100 -> 1..-1)
- muzzle_squash = remap(tz, -30..30 -> -1..1)
- muzzle_bend = remap(tx, 20..-20 -> 1..-1)
- skull_tp_bend = remap(tx, 50..-50 -> -1..1)
Animators translate Z to squash, translate X to bend and rotate (yaw) to twist.

**usdRig today.** The controls themselves are Implemented: RigExecControl with guide:shape circle, guide:scale* and cyan displayColor, nested under the head/muzzle joints with rest offsets. The local channels ARE avars:tx/tz/rz, already Euler.

The remap math exists: FloatMathMover remap (v-min)/(max-min) is unclamped and accepts inverted ranges, and multiply/add give [-1,1].

The wiring is BLOCKED:
- Avars are double.
- FloatMathMover targets must be float, and scalar connections must match type exactly, so avars:tz cannot feed a float chain or a float factor.
- The PoseInterpolator ignores enableTranslation (rotation only) and outputs RBF weights, not a linear remap.
The biped works around this by using custom float 'foot:' dials instead of transform channels.

**Gap.** No path converts a double transform avar into a float driver; no double math mover exists and connections cannot narrow double to float.

**Porting impact.** Animators could not translate or rotate the squash controls to deform the head. They would need sliders (custom float avars) instead, which changes a key face workflow. Twist extraction from a ZYX Euler is only exact when the control's rotationOrder matches.

**Recommendation.** Allow float<-double (and double<-float) scalar connections:
- In _ValidateScalarConnection (rigEvaluator.cpp:525-568), accept exactly these pairs.
- In RigExecResolvedInputs::GetAttribute (moverGraph.h:318-374), convert through VtValue::Cast and apply a finite-range check.
Alternatively, add RigExecDoubleMathMover mirroring FloatMathMover, or give RigExecNonlinearMover a double inputs:factor plus inputs:factorInMin/InMax/OutMin/OutMax so it can connect straight to avars:tz. Set the control's avars:rotationOrder to ZYX to match UE's Euler extraction.

**Evidence:** `libs/rigExecSchema/schema.usda:158-166`; `libs/rigExec/rigEvaluator.cpp:525-568`; `libs/rigExec/rigEvaluator.cpp:3539-3594`; `plugin/rigExecUsdview/avarEditorModel.py:36-45`; `libs/rigExec/rigEvaluator.cpp:2545-2562`; `libs/rigExec/moverGraph.h:318-374`

**Verification (holds).** The blocker is confirmed on every path I could find:
- avars:tx/ty/tz/rx/ry/rz are double (schema.usda:327-332).
- FloatMathMover targets must be float (rigEvaluator.cpp:3570-3572), and Vec3f/Matrix math do not help.
- Scalar connections require an exact type match (rigEvaluator.cpp:560-566).
- RigExecResolvedInputs::GetAttribute calls a typed Get with no cast (moverGraph.h:319-378).
- The PoseInterpolator warns and ignores enableTranslation (rigEvaluator.cpp:2549-2560) and outputs RBF weights, not a linear remap.
- No double math mover exists (grep).

The avar editor comment documents the float 'foot:' dial workaround (avarEditorModel.py:39-46). Translate and rotate on the squash controls is the animator workflow, so major holds.

One note on the recommendation: check that UE's ZYX Euler extraction matches usdRig's rotationOrder token semantics before relying on 'ZYX' for off-axis rotations.

Verifier evidence: `libs/rigExecSchema/schema.usda:327-336`; `libs/rigExec/rigEvaluator.cpp:560-566`; `libs/rigExec/rigEvaluator.cpp:3567-3597`; `libs/rigExec/rigEvaluator.cpp:2549-2560`; `libs/rigExec/moverGraph.h:319-378`; `plugin/rigExecUsdview/avarEditorModel.py:39-46`

### G7-normals-keep-input

**Normal/tangent recompute that preserves authored normal offsets**

**Verdict:** Partial · **Severity:** major · **Effort:** M · **Confidence:** high · **Domain:** D13

UE features: `UE7-dg-recompute-normals-keep-input`

**UE rigs.** Every graph runs this pass after its positional deform. Per vertex:
1. Walk the half-edge one-ring (up to 32 steps) and sum normalized face normals twice: SumD for the deformed positions and SumO for the positions before this deformer.
2. TangentZ = QuatBetween(normalize(SumO), authored n0) applied to normalize(SumD). This carries the authored-normal offset onto the deformed geometric normal.
3. TangentX = QuatBetween(n0, TangentZ) applied to the input TangentX.
4. The bitangent sign comes from the UV-winding sum.
The pass needs BuildHalfEdgeBuffers, the index buffer and UV0. Because it runs per deformer, 'original' is the previous deformer's output.

**usdRig today.** Derived maintenance synthesizes a RecomputeNormals revision after the whole chain, but only when the Mesh authors 'normals'. It computes Newell face normals with interior-angle weights from the FINAL points, and it has these limits:
- The authored normals are replaced outright; no offset is preserved.
- The envelope is fixed at 1.
- It is mesh-only and vertex-interpolated.
- Recomputed values must match the authored cardinality, so faceVarying (hard-edge/UV-split) normals fail and pass through stale.
- There are no tangents or bitangent sign, and primvars:normals is not handled, although spec 7.6 describes normal and tangent recomputation.

**Gap.** Missing pieces:
- a preserve-authored-offset policy;
- faceVarying and primvars:normals support;
- tangent (and bitangent sign) maintenance;
- hard-edge preservation.

**Porting impact.** UE-exported meshes usually carry faceVarying normals with hard edges. In usdRig the deformed head, and in fact every skinned region, would shade with rest-pose normals. Converting to vertex normals loses custom smoothing and hard edges. Normal-mapped materials get no deformed tangents.

**Recommendation.** Extend derived maintenance (rigEvaluator.cpp:5521-5584, moverGraph.cpp:647-676, geometryKernels.cpp) in three ways:
1. Accumulate per face-vertex corner for faceVarying normals and primvars:normals, grouping corners by vertex and smoothing group. A corner whose authored normal differs keeps its own offset.
2. Add an applied RigExecDerivedNormalsAPI on the gprim with uniform token rigExec:normalsPolicy = recompute | preserveAuthoredOffset. The derived revision has no authored mover, so the policy must live on the gprim. preserveAuthoredOffset caches q = QuatBetween(geometric normal of the base points, authored normal) per epoch and applies q to the final geometric normal each frame. This is UE's rule with the authored rest as the reference.
3. Optionally maintain an authored primvars:tangents (float4 with sign) by the same rotation.
Also implement the spec 7.6 subdivision rule: publish no normals when subdivisionScheme != none.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:5521-5584`; `libs/rigExec/moverGraph.cpp:647-676`; `libs/rigExecMath/geometryKernels.cpp:218-280`; `libs/rigExec/rigEvaluator.cpp:12300-12335`; `docs/curvenet.md:452-456`; `docs/spec.md:1355-1375`

**Verification (holds).** The analyst's description is accurate:
- Derived maintenance is synthesized only for an authored `normals` attribute (not primvars:normals) and only on meshes (rigEvaluator.cpp:5521-5584).
- It uses Newell face normals with angle weights from the final points (geometryKernels.cpp:218-300).
- A cardinality mismatch, such as faceVarying normals, fails the revision (moverGraph.cpp:659-666). The stale authored normals are then published with a MoverFailed diagnostic every frame (rigEvaluator.cpp:12373-12390).
- The docs note this limitation (curvenet.md:452-456).

Major holds.

The recommendation has a flaw. Caching q = QuatBetween(restGeometricNormal, authoredNormal) per epoch and applying it to the posed geometric normal is not frame-invariant: a head rotated 90 degrees would receive a world-space rest rotation. UE recomputes q every frame against the incoming (skinned) positions and normals, so 'Original' is the deformer's input (kernels.hlsl:292-312). The fix must either carry the offset in a per-vertex local tangent frame, or compute q per frame from a transported normals chain (see missed gap G7-skinned-tangent-frame).

There are also minor algorithmic differences that are cosmetic:
- UE sums unweighted face normals.
- UE's half-edge walk stops at the first boundary edge.
- UE runs the pass once per deformer rather than once at the end of the chain.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:5521-5584`; `libs/rigExec/rigEvaluator.cpp:12373-12390`; `libs/rigExec/moverGraph.cpp:647-676`; `libs/rigExecMath/geometryKernels.cpp:218-300`; `docs/curvenet.md:452-456`; `docs/spec.md:1355-1375`; `ue/<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraHeadBend_DeformerGraph/kernels.hlsl:236-316`

### G7-zebra-monster-stacks

**Zebra post-process and Monster face deformer stacks (7 + 9 instances)**

**Verdict:** Partial · **Severity:** major · **Effort:** M · **Confidence:** high · **Domain:** D13

UE features: `UE-zebra-deformer-stack`, `UE6-monster-optimus-deformers`

**UE rigs.** Zebra: CR_Zebra_Deform Sequence.B adds 7 deformers (AfterDefault, group 1, child components). Each factor is GetCurveValue(curve); squash factors are remapped: head and skull (v+1)/2, muzzle (1-v)/2. Each Transform is GetTransform(null, Global) for one of 7 nulls spawned under 'head' at construction.

Monster: CRM_Monster_Face pin L adds 9 deformers with the same settings. Its nulls under head carry offsets such as Head Squash T(0,0,20), Head Bend T(0,0,25) rotZ 90 and Mouth Squash T(0,30,60) rot180Y. Squash factors are remapped: head/skull (v+1)/2, muzzle/mouth (1-v)/2. These deformers run only while the face module evaluates.

**usdRig today.** Everything except the kernels can be expressed:
- Nulls become RigExecJoint children of the head joint with authored rest:t/r, riding the head through namespace propagation. This replaces construction-time HierarchyAddNull, which is a non-goal (dynamic topology).
- Factors become connections into the mover factor from float 'curve' attributes, remapped by FloatMathMover (remap with min=-1, max=1 gives (v+1)/2; remap with min=1, max=-1 gives (1-v)/2; unclamped like UE).
- Transform becomes rel rigExec:handle.
- Order comes from the namespace, and masks from weight objects.
- Post-process vs module placement is irrelevant in one rig.
The deformer kernels themselves are Missing.

**Gap.** The stacks depend on the bend, twist, squash-stretch and skin-mask rows. The null/handle, remap and order parts are expressible today.

**Porting impact.** With those rows in place, the port is mechanical: 16 movers, 16 handle joints, 16 mask weights and the remap chains. Without them, all head, muzzle, mouth and skull squash/bend/twist is lost on both characters.

**Recommendation.** Generate a Movers/FaceDeformers scope per character in UE exec order, with handles as RigExecJoint under the head joint (mesh at asset identity). For squash factors, insert one FloatMathMover remap on the curve attribute's copy. Validate each deformer against UE vertex samples captured from the editor at a few factor values.

**Evidence:** `libs/rigExecSchema/schema.usda:1192-1214`; `libs/rigExec/rigEvaluator.cpp:8771-8830`; `libs/rigExec/rigEvaluator.cpp:10189-10194`; `libs/rigExec/rigEvaluator.cpp:11667-11675`; `docs/spec.md:1381-1390`

**Verification (holds).** Checks on the analyst's mapping:
- The FloatMathMover remap (v-min)/(max-min) is unclamped, so min=-1/max=1 gives (v+1)/2 and min=1/max=-1 gives (1-v)/2 (schema.usda:1197-1213).
- The remap writes its target in place, so it must target the mover's own factor attribute (a blend-copy of the curve), never the bus curve itself. The analyst does say 'copy'.

Two omissions:
1. The Monster stack and the live Zebra curves are written from double transform avars. That path is blocked by exact-type connections (rigEvaluator.cpp:560-566, 3570-3572). The claim that 'everything except the kernels can be expressed' therefore holds only for baked curves.
2. The nulls are GlobalSpace transforms and need conversion to head-relative rest:space (CR_Zebra_Deform graphs.txt:682-694).

The verdict and severity stand.

Verifier evidence: `libs/rigExecSchema/schema.usda:1192-1214`; `libs/rigExec/rigEvaluator.cpp:560-566`; `libs/rigExec/rigEvaluator.cpp:3570-3572`; `libs/rigExec/rigEvaluator.cpp:8771-8830`; `ue/<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:682-694`

### G7-derived-normals-optout

**Documented per-mesh opt-out of derived normal maintenance is not implemented**

**Verdict:** Missing · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D13 · *added by verifier*

UE features: `UE7-dg-recompute-normals-keep-input`, `UE7-dg-pipeline-structure`

**UE rigs.** In UE the normals/tangents pass is an explicit node in each deformer graph. A graph can omit it: Write Skinned Mesh then passes the input tangents through. Authors therefore choose per deformer whether normals are recomputed.

**usdRig today.** Derived maintenance is synthesized for every moved Mesh that authors `normals`. docs/curvenet.md tells authors with faceVarying normals to set `rigExec:derived = off`, but nothing in libs reads that attribute; the only escape is to delete the authored normals. A failed derived revision publishes the stale authored normals and pushes a MoverFailed diagnostic every evaluated frame.

**Gap.** There is no per-gprim switch to skip or force derived normal and extent maintenance. An unsupported interpolation is reported every frame instead of once at compile time.

**Porting impact.** UE-exported meshes usually carry faceVarying normals. On those meshes the documented opt-out does nothing: every frame logs MoverFailed and shading keeps rest normals. The only workaround is stripping the normals and letting Hydra compute smooth ones.

**Recommendation.** Implement uniform token rigExec:derived = auto|off on the gprim, or on the RigExecDerivedNormalsAPI proposed in G7-normals-keep-input. Read it in the SolverSchedule.DerivedMaintenance block of libs/rigExec/rigEvaluator.cpp and mirror it in the baked lowering in libs/rigExec/bakedGeometry.cpp. Turn the interpolation/cardinality mismatch into a single compile-time diagnostic. Until faceVarying support lands, either fix the curvenet.md text or implement the switch.

**Evidence:** `docs/curvenet.md:452-456`; `libs/rigExec/rigEvaluator.cpp:5521-5584`; `libs/rigExec/rigEvaluator.cpp:12373-12390`

### G7-dmc-polygroup-generation

**DMC polygroup authoring from skin weights (bone-dominant triangle labels)**

**Verdict:** Missing · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D15

UE features: `UE7-dmc-polygroup-tool`

**UE rigs.** The Modeling-mode tool 'Direct Mesh Polygroup' labels triangles by bone:
1. Blend each triangle's three vertex skin weights at 1/3 each. With bTryUsingQuads, pair triangles into source quads (respecting UV seams and hard normals) and blend 0.5/0.5.
2. The label is the strongest bone.
3. Bones whose names match a BonesToRemove substring collapse to their parent.
4. Groups smaller than MinGroupSize (2) merge into a neighbour.
On Accept it writes a triangle label layer (LayerName, default 'dmc-polygroup'; the templates expect 'ik-layer' and 'fk-layer') whose value is the bone name.

**usdRig today.** TouchPose regions (RigExecTouchRegion faces bound to a control) are painted by brush in the TouchPose panel or imported from studio .touch files. Nothing generates them from skin weights.

**Gap.** There is no automatic dominant-influence face labelling (quad pairing, ancestor collapse, minimum group size) and no per-layer output.

**Porting impact.** DMC layers must be hand-painted or scripted. The shipped rigs do not resolve DMC shapes anyway (DMC is off or has no layers in the dumps), so animator impact is small.

**Recommendation.** Add plugin/touchPose/touchpose/autoregions.py, which:
- reads the RigExecSkinMover layout for the mesh;
- averages weights per face (quads natively in USD, so no quad search is needed);
- picks the dominant influence, collapses names matching a remove list to the parent joint, and merges small groups by face adjacency;
- writes a RigExecTouchRegions scope per layer name, with regions bound to the controls selected by a user mapping (bone -> FK/IK control).
Expose it as a 'Generate from skin' button in touchPoseUI.py.

**Evidence:** `libs/rigExecSchema/schema.usda:2530-2635`; `plugin/touchPose/touchPoseModel.py:214-333`; `examples/biped/README.md:83-130`

**Verification (holds).** TouchPose regions come from brush painting or .touch import (touchPoseModel.py:867-938; touchpose/touchfile.py). A grep of the touchPose package found no skin-weight-based generation. The DMC layers do not resolve in the shipped dumps (ue_open_questions gpu-deformers-dmc), so minor, not major, is right.

Verifier evidence: `plugin/touchPose/touchPoseModel.py:214-333`; `plugin/touchPose/touchPoseModel.py:867-938`; `libs/rigExecSchema/schema.usda:2610-2635`

### G7-pose-derived-scalar-squetch

**CR_Zebra_Deform dead 'Inverse_1' node and the chain-stretch 'Squetch' curve**

**Verdict:** Missing · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D11

UE features: `UE8-deform-unresolved-inverse-squetch`

**UE rigs.** regen.py adds an unresolved wildcard template node 'Inverse_1' with no links, so it compiles to nothing and can be ignored. Next to it, Squetch = ChainInfo.ChainStretchFactor - 1 (current chain length / initial length - 1) is written by SetCurveValue 'Squetch' and drives a corrective morph. It can go negative.

**usdRig today.** The dead node needs nothing (Not-applicable). Squetch is a POSE-derived scalar, and nothing can produce it:
- Property chains run before exec and cannot read solved frames.
- The PoseInterpolator phase (after the pose walk, before geometry) measures driver rotation only and publishes RBF weights.
- There is no chain-length or distance reader.

**Gap.** There is no chain-length or stretch-ratio reader that publishes a float into the geometry phase.

**Porting impact.** The Squetch corrective (via the morph) would not fire. This is a small visible loss on stretch; the dead node is irrelevant.

**Recommendation.** Add a RigExecChainMeasure reader, evaluated in the pose-interpolator phase next to _EvaluatePoseInterpolators in rigEvaluator.cpp and lowered in bakedPose.cpp, with:
- rel rigExec:joints (ordered chain);
- uniform token rigExec:measure = length | stretchRatio | stretchRatioMinusOne;
- float outputs:value, connectable to BlendInput weights and nonlinear-mover factors.
The allowance for negative weights belongs to the G6 correctives rows.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:11667-11675`; `libs/rigExec/rigEvaluator.cpp:2545-2562`; `libs/rigExecSchema/schema.usda:1809-1830`

**Verification (holds).** Nothing publishes a chain-length float:
- The schema has only outputs:weight on poses (schema.usda:2065).
- SplineIk measures stretch internally but outputs only frames.
- Property chains cannot see the pose (rigEvaluator.cpp:11667-11675).

I confirmed on the UE side that 'Squetch' is a morph target on SKM_Zebra (summary.json:39), written by SetCurveValue_35 from ChainInfo stretch minus 1 (CR_Zebra_Deform graphs.txt:175-179). So a corrective shape would be lost. Minor holds.

Verifier evidence: `libs/rigExecSchema/schema.usda:2065-2074`; `libs/rigExec/rigEvaluator.cpp:11667-11675`; `ue/<dump>/Game__Assets__Zebra__Meshes__SKM_Zebra/summary.json:39`; `ue/<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:175-179`

### G7-add-deformer-layering

**Rig-driven deformer layering (AddOptimusDeformer unit)**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D13

UE features: `UE-optimus-add-deformer-op`, `UE7-add-optimus-deformer-unit`

**UE rigs.** A mutable rig unit carries three kinds of traits: DeformerGraphAsset, Settings (phase/group/child components/exclude tag) and one SetDeformer<Type>Variable per deformer variable (the trait name equals the variable name; 'Refresh Variables' generates them). On each execution it:
1. enqueues a persistent instance GUID for the phase and group;
2. on the game thread, lazily adds the deformer instance, calling SetAlwaysUseMeshDeformer(true) and async-loading the asset;
3. pushes the variable values.
The queue is cleared every frame, so a deformer exists only on frames where the unit ran; the Monster's 9 deformers exist only while the face module runs. There are 16 nodes in total.

**usdRig today.** Layering a deformer means authoring a mover prim under <Rig>/Movers with rel rigExec:moves = </Mesh.points>. The equivalents:
- Discovery by namespace, static per epoch, non-destructive.
- Variable traits become schema input attributes with single-source connections, resolved every frame.
- A reusable 'deformer asset' becomes a USD reference or inherit of a mover prim from a library layer.
- Per-frame conditional execution becomes an animatable or connectable inputs:enabled (shape-preserving pass-through) or inputs:defaultWeight.
- Lazy instantiation and async loading are not needed.
Missing: the deformer movers themselves.

**Gap.** There are no deformer movers to layer, and no builder or Python adders for them.

**Porting impact.** Once the movers exist, each AddOptimusDeformer node maps 1:1 to a mover prim with connected inputs. The Monster deformers become a permanent part of the rig and can be gated with inputs:enabled if needed.

**Recommendation.** Add RigExecMoverChain::AddBendMover, AddTwistMover and AddSquashStretchMover(name, handle, target, weightObject, defaultWeight) in rigBuilder.h/.cpp, with Python bindings in python/_rigexec.cpp. The importer maps each AddOptimusDeformer as follows:
- the node becomes a mover;
- Settings.ExecutionPhase becomes the sibling position relative to the SkinMover;
- ExecutionGroup becomes a nested Scope;
- variable-trait links become attribute connections.

**Evidence:** `libs/rigExecSchema/schema.usda:49-89`; `README.md:199-220`; `libs/rigExec/moverGraph.h:318-374`; `libs/rigExecRigging/rigBuilder.h:873`; `libs/rigExecRigging/rigBuilder.h:915`

**Verification (holds).** The layering mechanism is implemented:
- Namespace discovery and non-destructive ordering (README.md:199-220).
- inputs:enabled as a shape-preserving pass-through (schema.usda:69-72).
- Connectable inputs.

The builder has adders only for the existing movers (rigBuilder.h:854-935). The only thing missing is the deformer movers themselves, so Partial and minor hold.

Verifier evidence: `README.md:199-220`; `libs/rigExecSchema/schema.usda:66-88`; `libs/rigExecRigging/rigBuilder.h:854-935`

### G7-curve-bus

**Named curve bus between animator face rig and deform rig (baked curves)**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D11

UE features: `UE-deformer-curve-bus`

**UE rigs.** Named float curves in the pose are the only interface between the two rigs:
- Writers: the face module's SetCurveValue nodes.
- Readers: the post-process CR_Zebra_Deform's GetCurveValue nodes.
The curves are also baked into animation (Zeb_Face_Expressions contains head_bend, head_squash, muzzle_*_deformer and skull_tp_*_deformer), so the deformers work from live rig evaluation or from baked curves without the control rig. muzzle_squash_deformer has two writers, and the one that runs last wins. The Monster does not use a bus: its deformers read curves inside the face module.

**usdRig today.** A 'curve' can be a custom float attribute (e.g. curves:head_squash):
- Writers are FloatMathMover chains targeting it, ordered deterministically by namespace, so two writers are simply two ordered movers.
- Readers connect to it and see property-chain results through RigExecResolvedInputs.
- Baked curves are time samples or Ts splines on the same attribute, with the writer movers disabled or removed via a layer or variant.
- export_baked writes final moved attributes.
- Property chains run before exec, which is fine for control-driven curves.
What is missing:
- a declared curve element or list (UE hierarchy CURVE elements), and curve discovery;
- the double-avar source problem (see the control-to-factor row);
- a stage-free runtime: the standalone rigpack does not lower movers, so there is no game-style post-process consumer.

**Gap.** There is no first-class curve channel, no curve namespace convention in the tooling, and no runtime (stage-free) deform consumer.

**Porting impact.** It works with a naming convention. The UE curve tracks in animations must be mapped to attribute time samples. Splitting the 'deform rig' from the 'animator rig' becomes a layer/variant choice.

**Recommendation.** Adopt a 'curves:' attribute namespace. Add it to RIG_PREFIXES in plugin/rigExecUsdview/avarEditorModel.py so the curves are visible and keyable. Optionally add an applied RigExecCurveAPI declaring curve channels (default and soft range) for discovery. The importer maps UE float-curve tracks to time samples on those attributes, and a 'deformOnly' variant disables the face-rig writer movers.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:8771-8830`; `libs/rigExec/rigEvaluator.cpp:11667-11675`; `libs/rigExec/moverGraph.h:318-374`; `docs/python-bake-inverse.md:1-31`; `plugin/rigExecUsdview/avarEditorModel.py:36-45`; `README.md:521-523`

**Verification (holds).** The analyst's mapping checks out:
- Property chains resolve before exec (rigEvaluator.cpp:8771-8830, 11667-11675).
- RIG_PREFIXES covers only avars: and foot: (avarEditorModel.py:46).
- export_baked bakes the final moved properties (python-bake-inverse.md:1-31).
- Standalone mover lowering is out of scope (README.md:521-523).

A naming convention works, so minor holds.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:8771-8830`; `libs/rigExec/rigEvaluator.cpp:11667-11675`; `plugin/rigExecUsdview/avarEditorModel.py:36-46`; `docs/python-bake-inverse.md:1-31`; `README.md:521-523`

### G7-deformer-child-components

**DeformChildComponents / ExcludeChildComponentsWithTag fan-out**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** medium · **Domain:** D13

UE features: `UE7-add-optimus-deformer-unit`

**UE rigs.** With DeformChildComponents=True (set on all 16 nodes), the same deformer instance and variables apply to the owning SkeletalMeshComponent and, recursively, to every attached child SkeletalMeshComponent. Children tagged ExcludeChildComponentsWithTag are skipped (None here).

**usdRig today.** Parameterized point movers take exactly one points target: fan-out would alias mover-level parameters. MatrixMover requires one target. Multi-target movers may bind only a constant envelope whose weightTarget is the mover prim. The equivalent is one mover per mesh, sharing parameters through USD inherits or connections, each with its own mask weight object.

**Gap.** A single mover cannot fan out to several meshes with per-mesh masks, and there is no tag-based exclusion.

**Porting impact.** The dumps do not show whether the Zebra or Monster actors attach child skeletal meshes. If they do, each child needs its own mover and mask, which is extra authoring but gives the same result.

**Recommendation.** Allow RigExecNonlinearMover to list several points targets, compiling one revision per target; the parameters are target-independent. Add rel rigExec:weightObjects, whose members are matched to targets by their weightTarget, or keep a builder helper that stamps per-mesh copies. Exclusion is simply leaving a mesh out of rigExec:moves.

**Evidence:** `libs/rigExec/rigEvaluator.cpp:3505-3521`; `libs/rigExec/rigEvaluator.cpp:6865-6935`; `libs/rigExecSchema/schema.usda:79-88`

**Verification (holds).** The single-target rule is confirmed for the Smooth, VolumeCorrect, Lattice and Curvenet movers (rigEvaluator.cpp:3505-3522) and for MatrixMover (rigEvaluator.cpp:6871-6880). The multi-target constant-envelope rule is in schema.usda:79-88.

The analyst missed a stronger UE case with the same usdRig answer. SKM_Zebra (34k vertices) and SKM_Zebra_Hi (132k vertices) both run the same post-process AnimBP_Zebra and CR_Zebra_Deform (ue_asset_summaries). The bone-name masks resolve per mesh, whereas usdRig needs one mover set and one mask set per mesh. This is extra authoring for the same result, so minor holds.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:3505-3522`; `libs/rigExec/rigEvaluator.cpp:6871-6880`; `libs/rigExecSchema/schema.usda:79-88`

### G7-deformer-handle-frame

**Deformer local-frame (OriginTransform) input from a null that rides the head**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D13

UE features: `UE7-dg-deform-frame-convention`

**UE rigs.** Each kernel works in the frame of the 'Transform' variable. That variable is an FTransform turned into a row matrix, supplied each frame from GetTransform(<X Null>, GlobalSpace), i.e. component space. The null is spawned under the head bone. Axis roles:
- Origin = row _41.
- Capture direction = row _31 (+Z).
- Bend axis = row _11 (X).
- Bend direction = row _21 (Y).
The affine inverse is taken per vertex, so scale and shear are allowed.

**usdRig today.** The frame source exists. A 'null' becomes a RigExecJoint or RigExecControl nested under the head joint, with authored rest offsets. Unbound descendants follow a solver-posed or constrained ancestor through namespace propagation, and computePointFrame publishes the posed asset-space frame, including scale and shear.

Asset space equals UE component space when the mesh sits at identity under the asset root. That is already the rule for point movers: there is no local-to-asset conjugation (docs/curvenet.md).

What is missing is a point mover that consumes the POSED frame:
- MatrixMover reads computeMatrix, which is the rest->posed delta (computations.cpp).
- Its validator accepts only Control/Joint providers with a base or final phase.

There is also no dedicated null element type. Joints are purpose=guide and draw debug spheres.

**Gap.** There is no posed-frame (as opposed to delta-matrix) input for point movers, and no invisible 'null' provider type.

**Porting impact.** This is handled by the new mover plumbing. The 16 nulls port as RigExecJoints under the head (Monster offsets such as Head Bend Null T(0,0,25) rotZ 90, Mouth Bend Null quat(-.707,-.707,0,0)). Meshes must sit at identity relative to the rig asset root.

**Recommendation.** This is delivered by G7-bend-deformer: rel rigExec:handle is resolved through the frame chain (final phase) into a GfMatrix4d via RigExecPointsToMatrix of the posed landmarks. Keep Gf row-vector layout for a 1:1 UE mapping. For nulls, allow RigExecJoint with guide suppression, or reuse RigExecControl with a new guide:shape token 'none'.

**Evidence:** `libs/rigExec/computations.cpp:400-416`; `libs/rigExec/rigEvaluator.cpp:6865-6935`; `libs/rigExec/rigEvaluator.cpp:10189-10194`; `libs/rigExec/moverGraph.h:999-1038`; `docs/curvenet.md:445-451`; `docs/spec.md:1325-1333`

**Verification (holds).** The verdict and severity hold. MatrixMover accepts only a Control/Joint computeMatrix, which is the rest-to-posed delta (rigEvaluator.cpp:6906-6932; computations.cpp:399-415). RigExecProviderValues has no posed-frame slot (moverGraph.h:999-1038).

Two corrections to the gap text:

1. An invisible null already exists:
- A RigExecJoint with guide:radius <= 0 draws no guides (schema.usda:396-402).
- A RigExecControl with a non-positive guide:scale or an unrecognized guide:shape draws nothing (schema.usda:201-209; sceneIndices.cpp:890-905).
The 'no invisible null provider' part of the gap is therefore wrong.

2. The UE nulls are spawned with GlobalSpace transforms, not head-local offsets (CR_Zebra_Deform graphs.txt:682-694). For example, Head Squash Null sits at global Z=80 while the head bone is at global Z=92.6 (MR_Zebra runtime_hierarchy.txt:938). The importer must author rest:space = nullGlobal * inverse(headRestGlobal) (schema.usda:298-299), not copy the numbers as offsets. The same applies to the Monster T(0,0,25) values the analyst quotes.

Verifier evidence: `libs/rigExec/rigEvaluator.cpp:6906-6932`; `libs/rigExec/computations.cpp:399-415`; `libs/rigExec/moverGraph.h:999-1038`; `libs/rigExecSchema/schema.usda:396-402`; `libs/rigExecSchema/schema.usda:201-209`; `libs/rigExecSchema/schema.usda:298-299`; `libs/rigExecImaging/sceneIndices.cpp:890-905`; `ue/<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:682-694`; `ue/<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:938`

### G7-deformer-param-sets

**Per-character deformer parameter sets and Zebra/Monster differences**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D13

UE features: `UE7-dg-zebra-deformer-params`, `UE7-dg-monster-deformer-params`, `UE7-dg-zebra-vs-monster`

**UE rigs.** The kernels are shared; parameters and usage differ.

Zebra (8 graphs, ZebraMuzzleTwist unused): squash L=55, twist L=60/135 deg, bend L=90/180 deg. Masks: head, 25-bone muzzle, 5-bone skull.

Monster (9 graphs): head squash 75, bend 90/180, twist 175/135, muzzle/mouth squash 55, skull squash 40. Muzzle and mouth masks include the cheeks (27/26/26/25 bones); the skull mask omits the ears.

Other differences: the stack order, the curve names (head_* vs head_*_deformer), and the integration point (post-process rig vs face module). The Zebra graphs also depend on a missing Monster function GUID.

**usdRig today.** Per-character values are plain attribute values on the mover prims. Shared kernel defaults can live on class prims (inherits) in a deformer library layer, with per-character layers overriding length, angle and mask, the same composition pattern as the layered biped. Asset-graph problems (missing GUIDs, cross-character function references) disappear because nothing references a graph asset.

**Gap.** The parameters have no schema until the nonlinear movers exist; nothing else is missing.

**Porting impact.** Data-only once the movers exist. Skip ZebraMuzzleTwist. Keep both orders and the length differences exactly, since they are visible.

**Recommendation.** Ship a deformer library layer with class prims _SquashStretchDefaults, _BendDefaults and _TwistDefaults. Per-character layers set length, maxAngle, the mask weight and the curve connections, generated by the importer from the asset.t3d constants and BoneNames lists.

**Evidence:** `libs/rigExecSchema/schema.usda:1-11`; `README.md:199-220`; `libs/rigExecSchema/schema.usda:1269-1282`

**Verification (holds).** The values are plain data once the movers exist, and class/inherit composition works as the analyst describes. The parameter table matches the feature file and the asset summaries: for example ZebraHead L=55 and Monster head L=75, Monster head twist L=175 and Monster skull top L=40. Partial and minor hold.

Verifier evidence: `libs/rigExecSchema/schema.usda:1-11`; `README.md:199-220`

### G7-deformer-runtime

**Deformers at game runtime and in baked output**

**Verdict:** Partial · **Severity:** minor · **Effort:** L · **Confidence:** medium · **Domain:** D18

UE features: `UE7-dg-zebra-vs-monster`, `UE-deformer-curve-bus`

**UE rigs.** The Zebra deformers run in the post-process AnimBP (CR_Zebra_Deform) at game runtime on any animation, driven only by curves, so they survive baked animation. The Monster deformers run only while the MR_Monster face module evaluates (CR_Monster_Deform adds none), so baked Monster animation loses them.

**usdRig today.** Movers run in the dynamic and baked-program evaluators, which need a USD stage. export_baked writes final moved points and attributes into a plugin-free USD cache, so all deformation is baked into the geometry; this is better than the Monster behaviour. The standalone rigpack backend executes providers without a stage, but whole-rig mover lowering is out of its scope. There is no post-process anim-graph integration.

**Gap.** There is no stage-free runtime that applies skin and nonlinear movers from baked joint and curve animation.

**Porting impact.** Offline and DCC use is fine through the evaluator or export_baked. A game-style runtime consumer of the deform rig does not exist.

**Recommendation.** Extend libs/rigExecStandalone to lower point-mover chains (Skin, BlendShape and the new nonlinear ops) and their weight packets into the rigpack, reusing RigExecRunRevisionKernel and the baked geometry loop. The pack can then deform from joint and curve inputs without a stage.

**Evidence:** `docs/python-bake-inverse.md:1-31`; `README.md:521-523`; `libs/rigExec/bakedGeometry.cpp:68-120`

**Verification (holds).** export_baked writes the final moved points (python-bake-inverse.md:1-31). The baked program reuses the same revision kernels (bakedGeometry.cpp:68-120), but it needs a stage. The standalone pack rejects mover applications (README.md:521-523; standalone-pack.md:57). Partial and minor hold for an offline/DCC target.

Verifier evidence: `docs/python-bake-inverse.md:1-31`; `libs/rigExec/bakedGeometry.cpp:68-120`; `README.md:521-523`; `docs/standalone-pack.md:57`

### G7-dmc-layer-registration

**DMC shape-library registration from mesh label layers (CRM_FN_DMC, ik-layer/fk-layer)**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D15

UE features: `UE7-dmc-shape-library-from-layer`, `UE1-dmc-module`, `UE8-seq-dmc-layer-names-not-animlayers`

**UE rigs.** SetupShapeLibraryFromLayer runs only in the construction event and needs an editor delegate. It:
1. reads a triangle label layer from the mesh;
2. collects the unique labels (bone names) as groups;
3. builds a proxy sub-mesh per group;
4. registers a ControlRigShapeLibrary named after the layer, with DMC materials and hover parameters, so shapes resolve as '<layer>.<bone>'.
CRM_FN_DMC, when 'Direct Mesh Control' is true, registers 'ik-layer' and 'fk-layer' and writes Root metadata 'Direct Mesh Control Libraries' (the non-empty layers) and 'Direct Mesh Control'. The 'fk-layer'/'ik-layer' strings in MR_Zebra_Take1 are these layers, not animation layers.

**usdRig today.** RigExecTouchRegions (rel mesh, palette, alpha) plus RigExecTouchRegion (faces, rel control) is a declarative, authored equivalent: named face sets bound to controls, found by type, with no construction step (a non-goal) and no sub-mesh assets. Limitations:
- TouchModel.FromStage uses the first scope with regions per mesh.
- Regions must not overlap.
- There is no layer name, so IK and FK sets covering the same faces cannot coexist.
- There is no enable flag or 'libraries present' metadata.
- Binding is explicit (region -> control) rather than resolved by bone name.

**Gap.** There are no multiple named layers per mesh, no active-layer selection by limb mode, and no DMC-enabled switch.

**Porting impact.** One region set per mesh can reproduce either the FK or the IK DMC layer, not both. An importer must not create animation layers from these strings.

**Recommendation.** Add uniform token rigExec:touch:layer to RigExecTouchRegions (default 'default'). Let TouchModel.FromStage load every scope for a mesh keyed by layer. Choose the active layer per region from the limb's IK/FK dial, reusing the RigExecPickerButton rigExec:picker:mode/modeDial pattern (schema.usda:2505-2514) on each region. An 'enabled' variant or visibility on the scope replaces the DMC bool. The importer treats ik-layer and fk-layer as touch layers.

**Evidence:** `libs/rigExecSchema/schema.usda:2530-2570`; `libs/rigExecSchema/schema.usda:2610-2635`; `plugin/touchPose/touchPoseModel.py:214-333`; `docs/spec.md:56-63`

**Verification (holds).** FromStage stops at the first scope that holds regions (touchPoseModel.py:294-300). RigExecTouchRegions has no layer token (schema.usda:2530-2570), and a face-to-region table assumes no overlap (touchPoseModel.py:236-242). The claims hold.

Verifier evidence: `plugin/touchPose/touchPoseModel.py:236-242`; `plugin/touchPose/touchPoseModel.py:258-333`; `libs/rigExecSchema/schema.usda:2530-2570`; `libs/rigExecSchema/schema.usda:2505-2514`

### G7-dmc-surface-gizmo-display

**DMC surface patches as always-on, pickable control gizmos that track the deformed skin**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D15

UE features: `UE7-dmc-proxy-component`

**UE rigs.** For each DMC-shaped control, a UDirectMeshControlComponent on the shape actor renders the group sub-mesh through the DG_DirectMeshControl deformer. The deformer reads the bound source mesh's deformed vertices through SubToSource (forcing LOD0 and SetAlwaysUseMeshDeformer), so the patch exactly overlays the current skin. It is:
- tinted per group (OverlayColor);
- hover-highlighted through the Hovered/HoveredColor material parameters;
- never culled;
- selecting the control when clicked, after which the standard gizmo manipulates it.

**usdRig today.** TouchPose picks with an exact ray cast against the deformed points and lights the hover, lead and selected regions on a lifted overlay mesh. Selection anywhere lights the region, and the viewport gizmo then manipulates the selected control; marquee and toggle rules match the picker. Edit/paint mode draws all regions with per-region colours.

It is usdview plugin tooling, outside the core spec (UI and hit testing are non-goals). Limitations:
- In animate mode the patches are not the control's persistent gizmo; only hover and selection are lit.
- The mesh cannot be picked while TouchPose is on.

**Gap.** Missing: always-on per-control surface gizmos, per-control hover colour parameters, and publication without stage authoring. The tooling is usdview-only.

**Porting impact.** Pick-on-skin works. The DMC look, where every control is a visible tinted skin patch, is not reproduced. The impact is minor because DMC is not active in the shipped rigs.

**Recommendation.** Add a filtering scene index in libs/rigExecImaging, next to RigExecResultsSceneIndex. For each RigExecTouchRegion with a control, it synthesizes a guide-purpose overlay rprim:
- faces = region faces;
- points = the moved points array of the mesh (shared, re-indexed);
- displayColor from the palette;
- primOrigin pointing at the control, so the stock pick path selects the control as it does for rigGuideCtrl.
Add a TouchRegions uniform bool rigExec:touch:alwaysVisible.

**Evidence:** `plugin/touchPose/touchPoseModel.py:1-60`; `examples/biped/README.md:83-130`; `libs/rigExecSchema/schema.usda:2572-2608`; `libs/rigExecImaging/sceneIndices.cpp:638-660`; `docs/spec.md:56-63`

**Verification (holds).** _ShowAllRegions draws every region only in paint mode, and clears them otherwise (touchPoseUI.py:1392-1420). Control mode lights only the hover and lead regions. While TouchPose is on, clicks on the skin are swallowed (touchPoseUI.py:106-113). This is tooling outside the core spec (spec.md:63). Partial and minor hold.

Verifier evidence: `plugin/touchPose/touchPoseUI.py:1392-1420`; `plugin/touchPose/touchPoseUI.py:106-113`; `plugin/touchPose/touchPoseModel.py:1-60`; `docs/spec.md:63`

### G7-factor-conventions

**Deformer variables vs baked constants and factor encodings**

**Verdict:** Partial · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D13

UE features: `UE7-dg-variables-and-constants`, `UE-deformer-factor-conventions`

**UE rigs.** Each graph exposes exactly two animatable variables: Transform, and one of StretchFactor/BendFactor/TwistFactor (double). Everything else is compiled in as a constant: LengthTo* (float), MaxBendAngle/MaxTwistAngle (int), XYBias/ZBias 0.5, limits 0, debug 0.

Factor encodings:
- StretchFactor: neutral 0.5, range [0, inf).
- Bend/Twist: range [-1, 1], neutral 0.

The BendFactor variable default is 0.5, which is NOT the identity; an unbound bend would bend the mesh 90 deg. The variable is routed through a ConstantValue node whose Value pin is linked, so the stored constant (e.g. 0.163201) is ignored.

**usdRig today.** USD already expresses this split. Uniform vs varying attributes, time samples/splines, and single-source exact-typed connections are resolved each frame by RigExecResolvedInputs, with property-chain results taking precedence. Every schema input can be animated or connected unless declared uniform. What is missing is only a deformer schema to carry these parameters (see the bend, twist and squash rows).

**Gap.** No schema declares the factor, length or angle parameters, or their neutral values.

**Porting impact.** An importer that copies UE variable defaults verbatim would bend unbound heads by 90 degrees. Otherwise all per-graph constants map to plain attribute values.

**Recommendation.** Declare inputs:factor and rel rigExec:handle as the connectable per-frame inputs of RigExecNonlinearMover. Make length, maxAngle, xyBias, zBias, lowBound and highBound ordinary (non-uniform) floats, which is a superset of UE. Use identity defaults: factor 0 for bend/twist and 0.5 for squash under the UE encoding. Document the encodings on the attributes. The importer should drop the UE BendFactor default of 0.5 and the ConstantValue shadow values.

**Evidence:** `libs/rigExec/moverGraph.h:318-374`; `libs/rigExec/rigEvaluator.cpp:525-568`; `libs/rigExecSchema/schema.usda:1192-1214`

**Verification (holds).** Scalar inputs resolve as follows:
- A property-chain result first, then a single exact-typed connection, then the authored or time-sampled value (moverGraph.h:319-378).
- Compile checks connection cardinality, type and cycles (rigEvaluator.cpp:525-569).

The only missing piece is a schema carrying the parameters. The BendFactor default 0.5 is not the identity, and that is accurately flagged. Partial and minor hold.

Verifier evidence: `libs/rigExec/moverGraph.h:319-378`; `libs/rigExec/rigEvaluator.cpp:525-569`; `libs/rigExecSchema/schema.usda:1192-1214`

### G7-skin-weight-mask

**Skin-weights-as-vertex-mask weight maps**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D13

UE features: `UE7-dg-skinweight-vertex-mask`, `UE-deformer-factor-conventions`

**UE rigs.** OptimusSkinWeightsAsVertexMask computes Mask(v) = sum over influences of w_i * isSelected(bone_i). Selection starts from BoneNames, expands ExpandTowardsRoot levels up and floods ExpandTowardsLeaf levels down (effective defaults 0 and 999, so every descendant is included).

Mask bone lists:
- head: head plus 277 descendants on the Zebra, 151 on the Monster.
- Zebra muzzle: 25 lip/teeth/nose/jaw bones.
- Zebra skull top: skull_tp, eye_main_l/r and ear_base_l/r.
- Monster muzzle/mouth: these lists add cheek_l/r.
- Monster skull top: omits the ears.

The mask feeds lerp(P, P', mask) in each kernel, with w < 1e-4 skipped. There are 17 masks, one per graph.

**usdRig today.** The consuming side is implemented. A dense RigExecStaticWeight whose weightTarget is <mesh>.points is bound through rigExec:weightObject and supplies the per-point envelope. The blend is exact at 0 and 1; it has a dense-default-0 rule and a strict/clamp range policy.

The skin layout needed to derive the mask exists on RigExecSkinMover (influences, jointIndices, jointWeights, elementSize), and joint descendants are just namespace nesting.

Nothing derives the field, though. There is no weight object that reads a skin layout plus a bone list, and no authoring helper; only the curvenet weight has a helper.

**Gap.** There is no skin-mask weight object or helper, and no root/leaf expansion. Masks must be baked offline into 17 dense float arrays and go stale when skin weights are re-painted.

**Porting impact.** Without masks every deformer moves the entire mesh, so the body would bend with the head. The workaround is an import-time bake into dense StaticWeights (34k/108k/132k floats each).

**Recommendation.** Add RigExecSkinMaskWeight (inherits RigExecWeightObject, representation dense only) to schema.usda:
- rel rigExec:skinMover
- rel rigExec:bones, which must be a subset of that mover's influences
- uniform int rigExec:expandTowardsRoot = 0
- uniform int rigExec:expandTowardsLeaf = 999 (descendants follow RigExecJoint namespace)
Build the packet in libs/rigExec/weightPackets.cpp from the epoch-cached RigExecSkinTopology, cache it per epoch, add baked lowering, and add a rigBuilder AddSkinMaskWeight. For static bakes, also ship a Python helper rigexec.create_skin_mask_weight() in python/rigexec.

**Evidence:** `libs/rigExecSchema/schema.usda:1220-1241`; `libs/rigExecSchema/schema.usda:1269-1282`; `libs/rigExec/weightPackets.cpp:29-92`; `libs/rigExecSchema/schema.usda:1681-1735`; `libs/rigExec/moverGraph.cpp:819-864`; `libs/rigExec/types.h:219-255`

**Verification (holds).** No weight object derives values from skin weights. There is no skin-mask type in the schema (the weight classes are at schema.usda:1220-1657), and a grep found nothing.

The consuming side works: a bound weight object replaces defaultWeight (schema.usda:79-88). The skin layout is cached per epoch (types.h:223-255).

One correction: the offline bake does not need dense arrays. RigExecStaticWeight supports a sparse representation with a sorted, deduplicated support set (weightPackets.cpp:48-77), so only head-region vertices need to be stored. Use rangePolicy clamp, because summed float weights can exceed 1 slightly and strict rejects that (weightPackets.cpp:14-27). A static bake is exact, so minor holds.

Verifier evidence: `libs/rigExecSchema/schema.usda:1220-1282`; `libs/rigExec/weightPackets.cpp:14-27`; `libs/rigExec/weightPackets.cpp:48-92`; `libs/rigExec/types.h:223-255`; `libs/rigExecSchema/schema.usda:79-88`

### G7-skinned-tangent-frame

**Normal/tangent transport through default skinning and morphs**

**Verdict:** Partial · **Severity:** minor · **Effort:** M · **Confidence:** medium · **Domain:** D13 · *added by verifier*

UE features: `UE8-skinning-cvars`, `UE7-deformer-stack-ordering`, `UE7-dg-recompute-normals-keep-input`

**UE rigs.** The meshes have default_mesh_deformer=None, so the base deformer is DG_LinearBlendSkin_Morph_Cloth. The engine ships separate DG_Function_LinearBlendSkin*_PositionOnly functions, so this default is the variant that also skins TangentX/TangentZ with the bone matrices and applies morph tangent deltas. The per-deformer KeepInputNormals pass then treats these skinned authored normals as 'Original'. Result: authored normals (custom smoothing, hard edges) survive skinning across the whole body, and full recompute runs only for sections that opt into DefaultRecomputeTangentDeformer.

**usdRig today.** SkinMover and BlendShapeMover write only the points property. The only normal maintenance is the derived RecomputeNormals revision, which replaces authored normals with angle-weighted geometric normals from the final points. There is no transport of normals through influence matrices, and BlendSample reads only UsdSkelBlendShape offsets, not normalOffsets. Normals are maintained, but not with UE's semantics.

**Gap.** Missing pieces:
- a normal/tangent transport operator (per-point weighted cofactor/inverse-transpose for LBS, rotation part for DQS);
- blend-shape normal offsets;
- maintenance of a tangent property (float4 with sign).

**Porting impact.** Even with every head deformer idle, the posed Zebra and Monster shade with recomputed geometric normals instead of skinned authored ones, so custom smoothing is lost across the whole body. It also leaves the preserve-offset normals fix with no correctly transported input to measure against.

**Recommendation.** Schema (libs/rigExecSchema/schema.usda, next to SkinMover): add a normals transport step, either:
- a policy value 'transport' on the gprim-level RigExecDerivedNormalsAPI proposed in G7-normals-keep-input, or
- an explicit RigExecSkinNormalsMover targeting <mesh>.normals or the winning primvars:normals.

Kernel: n' = normalize(sum_i w_i * cof(M_i) * n). Reuse the epoch-cached RigExecSkinTopology (types.h), and add an SSE twin in simdKernels.cpp. Handle faceVarying normals by mapping corners through faceVertexIndices.

Blend shapes: let RigExecBlendSample read UsdSkelBlendShape normalOffsets.

The head deformers' preserve-offset rule can then compute its quaternion per frame against these transported normals, which is UE's 'Original = input' rule.

**Evidence:** `libs/rigExecSchema/schema.usda:1681-1735`; `libs/rigExecSchema/schema.usda:1768-1807`; `libs/rigExec/rigEvaluator.cpp:5521-5584`; `libs/rigExec/moverGraph.cpp:647-676`; `libs/rigExecMath/geometryKernels.cpp:218-300`; `docs/spec.md:1355-1375`

### G7-deformer-graph-authoring

**Generic deformer-graph (compute graph + data interfaces + custom HLSL) authoring**

**Verdict:** Divergent-by-design · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D13

UE features: `UE7-dg-pipeline-structure`

**UE rigs.** Each of the 17 graphs is one UpdateGraph with a 'Primary' skeletal-mesh binding. The pipeline is:
Read Skinned Mesh (Position/TangentX/TangentZ/Color, i.e. the previous deformer's output)
  -> [CacheGeometry]
  -> DG_Function_Bend|Twist|SquashStretch
  -> ComputeNormalsTangentsAndKeepInputNormals
  -> Write Skinned Mesh.
The graphs use 14 data interfaces (transient buffers, a graph parameter buffer, skin-weight mask, half-edge, debug draw) and custom HLSL kernels. They contain no morph, skeleton or skin-cache nodes and rely on the default deformer that runs earlier.

**usdRig today.** usdRig rejects a general-purpose node graph (spec non-goal) and runtime operation dispatch. Every op is a fixed-signature mover, and although the spec lists 'user native kernels with exact predeclared inputs/outputs', no such extension point exists.

The native alternative is one statically typed mover prim per deform function, ordered in Movers. Its read is the preceding revision and its write is in place; normals and extent come from derived maintenance.

**Gap.** No user-authorable deformation graph or kernel exists. Every new UE graph topology needs C++ work in the evaluator.

**Porting impact.** All 17 ZebraSample graphs decompose into 3 kernels plus the normals pass, so a mapping table suffices. Custom graphs from other projects would not port.

**Recommendation.** Short term: a Python importer (e.g. python/rigexec/ue_deformers.py) keyed by the DG_Function_* reference and the bone-mask list, emitting RigExecBend/Twist/SquashStretch movers plus RigExecSkinMaskWeight.

Longer term: implement the spec 7.6 'user native kernel' extension point. That would be a plugin-registered C++ kernel resolved at compile time from a uniform token rigExec:kernel on a RigExecNativeKernelMover, with typed inputs:* attributes declared in the plugin's plugInfo. It stays statically typed, with no per-node runtime dispatch.

**Evidence:** `docs/spec.md:56-63`; `libs/rigExec/moverGraph.cpp:193-196`; `libs/rigExec/moverGraph.cpp:805-808`; `docs/spec.md:1355-1366`; `README.md:199-220`

**Verification (holds).** The non-goal exists: 'A new general-purpose node graph or renderer' (spec.md:60). There is no runtime dispatch beyond the frozen op set (moverGraph.cpp:193-196, 805-808). The 'user native kernels' item is listed only as a planned post-operation (spec.md:1364), and a grep found no implementation.

The 16 used graphs reduce to 3 kernels plus a normals pass, so minor is right.

Verifier evidence: `docs/spec.md:60`; `docs/spec.md:1364`; `libs/rigExec/moverGraph.cpp:193-196`; `libs/rigExec/moverGraph.cpp:805-808`

### G7-gpu-execution

**GPU compute execution of deformers and skin-cache settings**

**Verdict:** Divergent-by-design · **Severity:** minor · **Effort:** M · **Confidence:** high · **Domain:** D18

UE features: `UE7-dg-pipeline-structure`, `UE8-skinning-cvars`

**UE rigs.** Every kernel is a GPU compute shader (numthreads(64,1,1), one invocation per render section) operating on GPU skin/skin-cache buffers after default skinning. Project settings are GPU/renderer plumbing: r.SkinCache.DefaultBehavior=0, CompileShaders, Support16BitBoneIndex, UnlimitedBoneInfluences, ray tracing and experimental chunking. The kernels skip vertices whose mask is below 1e-4.

**usdRig today.** CPU scalar and SSE2 SIMD are the mandatory paths. GPU is optional and only for whole resident mover-chain segments, and none is implemented. Point-range parallelism (grain 512, threshold 4096) exists for skin and envelope blends, and SIMD exists only for the weighted matrix and LBS. Kernels touch every point of the target. Checkpoint memory grows with points x revisions.

**Gap.** There is no GPU path, no SIMD for new kernels, and no support-skip for points with zero weight.

**Porting impact.** Results are identical; the cost is performance only. Each frame adds 7-9 full-mesh CPU passes (34k Zebra, 132k Zebra_Hi, 108k Monster points) plus 7-9 checkpoints per mesh, even though only head vertices move.

**Recommendation.** Write the nonlinear kernels with SSE2 and use WorkParallelForN range splitting (parallel.h). Add an epoch-cached support index list from the bound weight packet (w > 0; sparse packets already sorted) so kernels run only on masked points, matching UE's w < 1e-4 skip. Consider dropping checkpoints for revisions that no read phase names. GPU remains the spec 7.8 whole-segment future work.

**Evidence:** `docs/spec.md:1184-1195`; `docs/spec.md:1398`; `libs/rigExec/parallel.h:21-32`; `libs/rigExecMath/simdKernels.h:26-56`; `README.md:510-515`

**Verification (holds).** Spec 7.8 makes CPU scalar and SIMD mandatory and GPU optional (spec.md:1398), and 6.5 has no GPU promise (spec.md:1184-1195). The analyst's inventory is accurate:
- Parallel grain and threshold: parallel.h:23-32.
- SIMD exists only for the weighted matrix and LBS kernels (simdKernels.h:26-56).
- Checkpoint memory grows with point count times revisions (README.md:510-512).

A weight of 0 passes through bit-for-bit, so results match UE's w<1e-4 skip up to trivial values. The cost is performance only, so minor holds.

Verifier evidence: `docs/spec.md:1184-1195`; `docs/spec.md:1398`; `libs/rigExec/parallel.h:23-32`; `libs/rigExecMath/simdKernels.h:26-56`; `README.md:510-512`

### G7-stack-ordering

**Deformer stack order on top of default skinning (phase/group/enqueue order)**

**Verdict:** Implemented · **Severity:** minor · **Effort:** S · **Confidence:** high · **Domain:** D13

UE features: `UE7-deformer-stack-ordering`

**UE rigs.** Dispatch order is:
1. BeforeDefaultDeformer instances.
2. The OverrideDefaultDeformer slot: the last-enqueued instance in the highest group, else the default DG_LinearBlendSkin_Morph_Cloth.
3. AfterDefaultDeformer instances.
Groups run ascending and instances in enqueue order; re-enqueuing blanks the earlier entry. Each instance reads the previous one's output. Order matters:
- Zebra: HeadSquash, HeadTwist, HeadBend, MuzzleSquash, MuzzleBend, SkullTpSquash, SkullTpBend.
- Monster: HeadSquash, HeadBend, HeadTwist, MuzzleBend, MuzzleSquash, MouthBend, MouthSquash, SkullTpBend, SkullTpSquash.

**usdRig today.** Order comes from the composed Movers namespace: reverse-sibling post-order, bottom sibling first, children before their parent. Under() nests groups. Each revision consumes the preceding revision, and read phases expose explicit snapshots. The UE phases map as follows:
- The default deformer is explicit: BlendShapeMover below SkinMover gives morph then LBS.
- 'After' means siblings above the SkinMover and 'Before' means below it.
- 'Override' means disabling or replacing the SkinMover (inputs:enabled or a variant).
- Groups are nested Scopes, and dedupe is prim identity.
Cloth has no equivalent (out of this group's scope).

**Gap.** None semantically. Authoring pitfalls remain: the bottom sibling fires first, composed multi-layer rigs need explicit nameChildren reorder statements, and there is no API to move a mover after creation.

**Porting impact.** The two distinct Zebra and Monster orders are reproducible exactly. An importer that writes siblings in UE exec order without reversing them would invert the stack, and since the deformers do not commute, the shapes would be wrong.

**Recommendation.** The importer writes an explicit 'reorder nameChildren' in UE exec order, reversed for bottom-first execution, and a test asserts rig.mover_order(). Add RigExecMoverChain::MoveBefore/MoveAfter in rigBuilder.h.

**Evidence:** `README.md:199-236`; `libs/rigExec/moverGraph.cpp:199-262`; `libs/rigExecSchema/schema.usda:1681-1735`; `libs/rigExecSchema/schema.usda:49-72`

**Verification (holds).** Reverse-sibling post-order and children-first execution are documented (README.md:203-218) and confirmed in the builder: appending makes the new mover run first (rigBuilder.h:1008-1017). Under() nests movers (rigBuilder.h:990-996). The builder has no reorder or move API. Each revision reads the preceding one.

The verdict holds. The severity could reasonably be cosmetic, since a correct importer reproduces both orders exactly; minor is defensible for the missing reorder API.

Verifier evidence: `README.md:199-236`; `libs/rigExecRigging/rigBuilder.h:836-843`; `libs/rigExecRigging/rigBuilder.h:990-1017`; `libs/rigExec/moverGraph.cpp:199-262`

### G7-deformer-handle-guides

**Deformer handle debug draw (capture line, limit planes, axes)**

**Verdict:** Missing · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D19 · *(analyst said Missing / minor)*

UE features: `UE7-dg-deform-frame-convention`

**UE rigs.** With Optional_EnableDebugDraw set, the kernels draw:
- a blue line from the origin along the capture direction for L;
- blue quads at the lower and upper limits;
- a red line along the bend axis and a green line along the bend direction;
- for twist, a rotated upper plane.
The constant is 0 in all graphs; it is a rigger aid.

**usdRig today.** Synthesized guides exist for controls, joints, solvers and volume weights, through the same scene-index synthesis. Geometry movers have none.

**Gap.** There is no guide for a deformer handle, its length or its limit band.

**Porting impact.** Riggers cannot see where the deformation band starts and ends when placing handle nulls. There is no effect on animation results.

**Recommendation.** Add guide:drawMode (wire|none, default none) and guide:displayColor to RigExecNonlinearMover. In libs/rigExecImaging/sceneIndices.cpp, synthesize a 'rigGuideDeformer' child using the same announcement and dirtying machinery as rigGuideCtrl: a line along +Z for inputs:length, rectangles at the low and high bounds, X/Y axis lines, and for twist a rotated top rectangle.

**Evidence:** `libs/rigExecImaging/sceneIndices.cpp:638-660`; `libs/rigExecImaging/sceneIndices.cpp:885-905`; `libs/rigExecSchema/schema.usda:167-222`

**Verification (corrected).** No deformer guide exists, so Missing is fine. The severity should be cosmetic, for two reasons:
- EnableDebugDraw is a compiled constant 0 in every graph (the kernels only draw when it is non-zero, ZebraHeadBend kernels.hlsl:112-140), so a port loses nothing animators see.
- A usable band marker already exists. A RigExecPlaneWeight (planeAxis z, bounded) nested under the handle joint draws wire iso-planes at falloffMin and falloffMax with an extentU x extentV rectangle (schema.usda:1459-1475, 1500-1576). Together with the handle joint's own sphere guide, that gives a rigger the band start and end today.

Verifier evidence: `libs/rigExecSchema/schema.usda:1459-1475`; `libs/rigExecSchema/schema.usda:1500-1576`; `libs/rigExecImaging/sceneIndices.cpp:638-660`; `ue/<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraHeadBend_DeformerGraph/kernels.hlsl:112-140`

### G7-dmc-rundmc

**RunDMC bonus module: convert any control to a DMC surface shape**

**Verdict:** Partial · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D15

UE features: `UE7-dmc-rundmc-module`

**UE rigs.** A RigModule with a Control connector and variables Name (label, e.g. hand_l) and LayerName (default 'dmc-polygroup'). When the connector is connected and Name is non-empty, construction runs SetupShapeLibraryFromLayer(LayerName) and sets the control's shape to Name, coloured red and visible. No template in the project uses it.

**usdRig today.** Binding an existing or painted RigExecTouchRegion to any control through rel rigExec:touch:control gives the pick and highlight part. Replacing the drawn shape needs the 'touchRegion' guide token proposed in G7-dmc-shape-resolution.

**Gap.** There is no one-step 'use surface label X as this control's gizmo'.

**Porting impact.** None; the module is unused in the rigs.

**Recommendation.** Once the touch layer and touchRegion guide exist, add a TouchPose panel action 'Use region as gizmo' that sets rigExec:touch:control on the named region and guide:shape='touchRegion' on the control.

**Evidence:** `libs/rigExecSchema/schema.usda:2610-2635`; `libs/rigExecSchema/schema.usda:167-222`

**Verification (holds).** The module is unused in every rig. Region-to-control binding exists (schema.usda:2630-2634), and a one-step conversion does not. Partial and cosmetic are correct.

Verifier evidence: `libs/rigExecSchema/schema.usda:2610-2635`; `libs/rigExecSchema/schema.usda:167-222`

### G7-dmc-shape-resolution

**Per-control DMC shape selection with fallback (Get Control Shape Name From Item v02, Get DMC Shape)**

**Verdict:** Partial · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D2 · *(analyst said Partial / minor)*

UE features: `UE4-shape-name-from-item-v02-dmc`, `UE6-get-dmc-shape`

**UE rigs.** Get Control Shape Name From Item v02 (9 call sites: IkFk2Bones, Foot, Spine, FkArray, FkChain):
- If DMC Libraries contains the namespace and ShapeExists('<ns>.<bone>'), use that shape; otherwise use the module's Default Shape.
- IK controls use 'ik-layer' and FK controls use 'fk-layer'.

Zebra face Get DMC Shape (23 call sites):
- DMC Found ? 'fk-layer.'+name : default.
- If that shape does not exist, fall back to Circle_Thick.
- DMC Found is never true in the shipped rigs.

**usdRig today.** guide:shape is a uniform token over 6 primitives, with no shape library, name resolution or conditional fallback; an unknown token draws no guide. The region->control binding makes 'this control has a surface patch' an authored fact, but the control keeps drawing its primitive guide as well.

**Gap.** No shape can mean 'use my surface region, else fall back to X', and a region-bound control's primitive guide cannot be suppressed.

**Porting impact.** There is no difference in the shipped state (DMC off). Reproducing DMC mode would show both the primitive guide and the region.

**Recommendation.** Add a guide:shape token 'touchRegion' plus uniform token guide:fallbackShape. In _FindControlGuideShape (sceneIndices.cpp), draw nothing when a RigExecTouchRegion in the active layer binds the control, so its surface patch is the gizmo; otherwise draw guide:fallbackShape. The importer maps UE Default Shape names (Circle_Thick, etc.) to fallback tokens.

**Evidence:** `libs/rigExecSchema/schema.usda:167-222`; `libs/rigExecImaging/sceneIndices.cpp:885-905`; `libs/rigExecSchema/schema.usda:2610-2635`

**Verification (corrected).** The gap text is partly wrong. A region-bound control's primitive guide can already be suppressed:
- An unrecognized guide:shape token draws no guide, by documented design (sceneIndices.cpp:890-905).
- A non-positive guide:scale also draws nothing (schema.usda:201-209).

Only the conditional fallback ('surface patch, else X') is missing. The analyst's own porting impact says there is no difference in the shipped state: DMC Found is never true on the Zebra, and no ik-layer/fk-layer shapes resolve in the DMC templates. By the rubric, the severity is therefore cosmetic.

Verifier evidence: `libs/rigExecImaging/sceneIndices.cpp:890-905`; `libs/rigExecSchema/schema.usda:167-222`; `libs/rigExecSchema/schema.usda:2610-2635`

### G7-dmc-submesh-generation

**Per-polygroup transient proxy skeletal meshes (SubToSource)**

**Verdict:** Divergent-by-design · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D15

UE features: `UE7-dmc-submesh-generation`

**UE rigs.** For each label group, the editor subsystem builds a transient one-bone USkeletalMesh sub-mesh:
- a 'SubToSource' float vertex attribute = source render vertex index + 0.5, via GetRawPointIndices;
- per-group vertex colours and the M_DirectMeshControl material.
Results are cached per (mesh, layer) using a derived-data hash, rebuilt when the hash changes, and never saved.

**usdRig today.** usdRig never generates prims or assets (a non-destructive non-goal). TouchPose builds the highlight patch on the fly from the painted face list and the DEFORMED points read back from Hydra. Derived arrays are rebuilt only when the rig generation changes, and the patch is written to a session-layer RigExecTouchOverlay mesh. Face indices play the role of SubToSource.

**Gap.** None functionally. The overlay is authored in the session layer (about 8.9 ms per region crossing) rather than published directly to Hydra.

**Porting impact.** No sub-mesh assets need porting.

**Recommendation.** Keep the approach. See G7-dmc-surface-gizmo-display for moving the overlay into a scene-index filter.

**Evidence:** `libs/rigExecSchema/schema.usda:2572-2608`; `plugin/touchPose/touchPoseModel.py:1-60`; `docs/spec.md:56-63`

**Verification (holds).** No functional gap: TouchModel.OverlayGeometry builds the patch from the deformed points, and the derived arrays are rebuilt only when the rig generation changes (touchPoseModel.py:10-60, 675-757).

The citation should change. spec.md:58 concerns evaluation-time topology, not assets. The deliberate design is the pre-declared RigExecTouchOverlay prim that ships empty and is filled in a session layer (schema.usda:2572-2608). The label could arguably be Implemented in tooling. Cosmetic holds.

Verifier evidence: `libs/rigExecSchema/schema.usda:2572-2608`; `plugin/touchPose/touchPoseModel.py:1-60`; `plugin/touchPose/touchPoseModel.py:675-757`; `docs/spec.md:58`

### G7-skinning-influences

**Default skinning, influence count and skin-cache project settings**

**Verdict:** Implemented · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D13

UE features: `UE8-skinning-cvars`

**UE rigs.** The meshes have no default_mesh_deformer, so the base is DG_LinearBlendSkin_Morph_Cloth: morph targets, then LBS, then cloth. Unlimited bone influences are enabled with 16-bit bone indices. SKM_Zebra_Hi was imported with boneInfluenceLimit 0 (no limit). The Monster USD export shows 12 influences per vertex; the Zebra count is unknown. The skin-cache, ray-tracing and chunking settings are renderer plumbing.

**usdRig today.** RigExecSkinMover uses the UsdSkel layout (jointIndices/jointWeights with any elementSize >= 1, influences = RigExecJoint/Control providers) with classicLinear LBS; weights are not renormalized. An upstream BlendShapeMover is skinned, reproducing morph-before-LBS. The biped uses elementSize 10, so 12 or more is fine.

The cvars are Not-applicable.

By design there is no UsdSkel import bridge and no geomBindTransform: bind = provider rest frames.

**Gap.** None for influences. Conversion from the UE USD export's primvars:skel:* is an importer task. There is no recompute-tangents skinning variant (see G7-normals-keep-input).

**Porting impact.** LBS matches UE when the imported weights sum to 1. The importer must copy UsdSkel primvars into rigExec:* and create joint rest frames equal to the bind transforms.

**Recommendation.** The Python importer reads UsdSkelBindingAPI primvars from the UE USD export and writes SkinMover attributes, with elementSize = max influences (12 for SKM_Monster). It warns on a non-identity geomBindTransform and on per-vertex weight sums different from 1.

**Evidence:** `libs/rigExecSchema/schema.usda:1681-1735`; `libs/rigExecMath/solvers.cpp:390-441`; `libs/rigExec/rigEvaluator.cpp:6981-7013`; `docs/spec.md:56-63`

**Verification (holds).** The SkinMover layout follows UsdSkel conventions with any elementSize >= 1 and no renormalization (schema.usda:1681-1735). Validation is in rigEvaluator.cpp:6975-7060 and solvers.cpp:390-441. The UsdSkel import bridge is a non-goal (spec.md:61).

For positions this is Implemented. The analyst did not cover that UE's default DG_LinearBlendSkin_Morph_Cloth also skins the tangent frame (the engine ships separate *_PositionOnly LBS functions). usdRig has no equivalent operator; I list it as missed gap G7-skinned-tangent-frame rather than downgrading this position-skinning row.

Verifier evidence: `libs/rigExecSchema/schema.usda:1681-1735`; `libs/rigExecMath/solvers.cpp:390-441`; `libs/rigExec/rigEvaluator.cpp:6975-7060`; `docs/spec.md:61`

### G7-cache-geometry

**CacheGeometry pass-through snapshot**

**Verdict:** Not-applicable · **Severity:** cosmetic · **Effort:** S · **Confidence:** high · **Domain:** D13

UE features: `UE7-dg-cache-geometry-passthrough`

**UE rigs.** A vertex kernel copies Position, TangentX and TangentZ into transient buffers used as the 'Original*' inputs of the normals pass. It is mathematically the identity. The Zebra graphs reference a missing function GUID in Monster_Head_DeforerGraph ('<graph missing>'), and the Modified ZebraHead graph compiled without the pass.

**usdRig today.** Every revision already reads the preceding buffer through the VDF READWRITE connector. Named snapshots of any (target, mover) revision are available through rigExecReadPhase (preceding, final, or an absolute prim path), recorded copy-on-write, so no copy node is needed.

**Gap.** None. It is an implementation artifact of the Optimus graph.

**Porting impact.** Drop the node. The cross-character Zebra->Monster asset dependency and the dangling GUID disappear.

**Recommendation.** The importer ignores CacheGeometry subgraphs and function references whose kernel is an identity copy. If the new normals policy needs the pre-deform positions, it reads them through a read phase rather than a copy.

**Evidence:** `libs/rigExec/moverGraph.h:75-140`; `libs/rigExec/moverGraph.cpp:199-262`; `README.md:222-236`

**Verification (holds).** The kernel is a pure copy (Monster_Head / ZebraHeadBend kernels.hlsl:1-12). ZebraHead compiled without it, with no change in result. usdRig revisions read the preceding revision in place, and read phases expose named snapshots (moverGraph.h:75-114; moverGraph.cpp:199-262). Not-applicable holds.

Verifier evidence: `libs/rigExec/moverGraph.h:75-114`; `libs/rigExec/moverGraph.cpp:199-262`; `ue/<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraHeadBend_DeformerGraph/kernels.hlsl:1-12`

