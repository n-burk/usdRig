> **Naming note (added after the fact, 2026-08-16).** This document is the dated
> record of the v0.1 architecture spec and is left verbatim. One name has since
> changed in the implementation: the rig root prim type **`RigExecRig` is now
> `RigExecRoot`**. Every occurrence of `RigExecRig` below refers to it. No
> compatibility alias exists -- a stage still authoring `RigExecRig` has an
> unknown prim type and no rig is discovered on it.

Character Rig Execution Engine — Architecture Spec & Implementation Plan (v0.1)

**Codename:** RigExec  
 **Status:** Proposed architecture, research baseline 24 July 2026  
 **Audience:** DCC architecture, character technology, USD, imaging, rendering, and runtime teams

# Executive summary

RigExec is a rigging object model and character-computation product **on top of OpenExec**, not a competing graph engine. OpenExec provides compilation, request scheduling, pull evaluation, caching, sparse invalidation, typed values, and parallel execution. It is value-driven, stateless at computation boundaries, unable to author or change scene topology, and not itself a rigging or event system. RigExec owns the schema, compilation policy, evaluation, packing, extraction, and Hydra-publication layers defined here; an event/UI product is deferred.

The canonical pose value is RigExecPointFrame: four points representing an affine origin and transformed X, Y, and Z basis endpoints. It round-trips exactly to a nonsingular affine matrix, including scale and shear. Joint/control policies derive orientation and twist deterministically from origin/aim/up landmarks. Every semantically addressable scalar transform provider publishes paired computePointFrame/computeMatrix; packed solver arrays surface semantically addressable elements through explicit scalar providers.

All durable authoring remains OpenUSD. The rig object model is deliberately granular and composition-native: joints, transform providers, one-transform matrix movers, reusable weight objects, independent blend inputs/samples, and output bindings receive independent layer opinions, and the compiler packs them only after composition. Geometry operations use the term **movers** throughout. Every mover—or IK, aim, constraint, or math operation that modifies an existing result—authors rel rigExec:moves to the exact prim or property it affects. The compiler walks the composed mover namespace in post-order: descendants first, then ancestors. It lowers each write into an immutable value revision, so the last operation in logical hierarchy order supplies the final result; worker completion order is irrelevant. Namespace, target, or operation-kind edits rebuild an evaluation epoch, while evaluation itself never changes graph topology.

One execution core serves two deployments. USD uses ExecUsdSystem/EsfUsd; standalone supplies compact scene objects behind the same Esf interfaces and constructs the same scene-independent exec core. OpenUSD 26.08 is consumed unchanged: RigExec does not fork, patch, or modify vdf, exec, Esf, their compiler, or scheduler. Because OpenUSD labels Esf non-public, Phase 0 must qualify the required interfaces from a pinned stock build behind one RigExec-owned compatibility boundary. If those shipped interfaces cannot be compiled, linked, loaded, and supported on a target platform, the standalone path stops for review rather than changing OpenUSD.

Geometry is a first-class immutable typed value with shared SoA points, full primvar descriptors, topology, coordinate-space metadata, and provenance. Vectorized movers implement weighted matrix movement, blend shapes, lattice, curve/ribbon, surface, and post stages. Meshes, points, and basis curves may be inputs or moved results; topology remains fixed within an evaluation epoch.

Hydra receives complete immutable generations through a filtering scene index that publishes only standard local xforms, points, primvars, normals, and extents with precise dirtiness. Rig execution always finishes before Hydra pulls; render delegates require no RigExec-specific code.

Stock OpenUSD owns animation value resolution. Authored TsSpline data on supported scalar attributes is evaluated natively by timed UsdAttribute::Get()/UsdAttributeQuery::Get() through Ts; ordinary sparse timeSamples remain the native held/linear path for vectors, matrices, arrays, non-interpolatable values, and any channel not authored as a spline. RigExec defines no custom animation source or curve sampler.

A typed RigExecTapSet maps stable provider/computation/type addresses to batched backend keys and immutable snapshots. The evaluator, parity harnesses, exporters, and Hydra publication use this extraction boundary. UI, direct-manipulation workflows, animation editors, and general tooling are deliberately deferred.

The C++17-compatible core targets Windows, macOS, and Linux. Production computations remain native. A static, no-Python iPadOS runtime loads mobile LODs for headless time-sequenced evaluation and Hydra-publication validation from .rigpack; an on-device renderer and product playback surface are not v0.1 deliverables. Six evidence-gated phases progress from the stock-OpenUSD seam proof through schemas/points, native animation, movers/Hydra, standalone/mobile runtime, and production hardening.

# 1\. Scope, evidence rules, and status vocabulary

This document uses three mandatory status labels:

  - EXISTS — verified in the tagged OpenUSD 26.08 release documentation or source; RigExec uses it.
  - PLANNED — explicitly signaled future capability or source TODO; shipping experimental code remains EXISTS with its maturity stated separately.
  - BUILD — absent from the verified OpenExec product boundary, or deliberately application-specific; the RigExec team owns it.

The sole normative upstream baseline for v0.1 is the unchanged OpenUSD 26.08 release dated 20 July 2026. The [tagged changelog](https://github.com/PixarAnimationStudios/OpenUSD/blob/v26.08/CHANGELOG.md), 26.08 release documentation, and [tagged pxr/exec source](https://github.com/PixarAnimationStudios/OpenUSD/tree/v26.08/pxr/exec) are authoritative for capability claims. /dev documentation links are explanatory navigation aids only and must be corroborated against that tag; later releases are non-binding candidates for a future qualification.

Where a behavior is undocumented, this specification says “OpenExec is silent” and makes a RigExec decision. Proposed class and schema names beginning with RigExec do not claim to be upstream APIs. Signatures beginning with Exec, Vdf, Esf, Hd, or Usd are either current upstream names or are explicitly marked as sketches.

## Goals

1.  Film-quality character evaluation and render publication with predictable latency.
2.  One OpenExec-based evaluator behind USD and standalone scene backends.
3.  OpenUSD-native rig authoring, composition, animation value resolution, and interchange.
4.  Point-native transforms and hierarchy-ordered movers, including one-transform matrix movers and reusable static/dynamic weight objects for every catalogued prim/property domain.
5.  Render-delegate-neutral publication through Hydra 2.0.
6.  Testable, typed extraction of every semantically addressable result.
7.  A cross-platform path from workstation DCCs to tablet-class headless time-sequenced evaluation and Hydra-publication validation.

## Non-goals for v1

  - Dynamic topology creation during evaluation. OpenExec observes scene description and cannot add or remove stage objects or author values, as the OpenExec boundary documentation states.
  - Stateful simulation hidden inside computation callbacks. Simulation may be integrated later only by making state and time-step inputs explicit or by consuming baked caches.
  - A new general-purpose node graph or renderer.
  - Import/export bridges to monolithic skeleton-binding schemas. RigExec v1 defines its authoring and execution contract only through granular, independently composable controls, joints, transform providers, matrix movers, weight objects, other operations, and result bindings.
  - Production Python callbacks in the evaluation hot path.
  - UI, viewport manipulation, hit testing, animation editors, undo/transaction systems, pose libraries, graph browsers, and other end-user tooling. The current plan defines only data, evaluation, packing, extraction, Hydra publication, and rendering infrastructure.

# 2\. Prior art analysis

## 2.1 DreamWorks Premo and LibEE

**Adopt.** LibEE combines concurrency inside expensive nodes and between ready nodes, caches repeated control-to-output task plans, and separates rich authoring from optimized dependencies ([DigiPro ’12 paper](https://doi.org/10.1145/2370919.2370930), [parallel-evaluation chapter](https://www.oreilly.com/library/view/multithreading-for-visual/9781482243567/chapter-27.html), [LibEE 2](https://research.dreamworks.com/wp-content/uploads/2018/08/talk_libee2-Edited.pdf)). RigExec obtains those properties through OpenExec requests rather than rebuilding LibEE.

Premo emphasizes low-latency direct manipulation ([Premo](https://dblp.org/rec/conf/siggraph/GongNPRWBPD14)); its [hierarchy model](https://research.dreamworks.com/wp-content/uploads/2019/10/talk_hierModel_stage2.pdf) carries joints atomically. RigExec adopts only the underlying separation of authoring, evaluation, and display plus packed-frame execution, with membership frozen per v1 epoch. Premo’s UI and manipulation model is not a v0.1 deliverable.

**Reject.** We do not recreate LibEE, copy a proprietary Premo object model, or let arbitrary stateful nodes opt out of dependency declaration. OpenExec callbacks must receive all inputs explicitly and be stateless; those constraints are essential to safe caching and parallel evaluation ([OpenExec System Design](https://openusd.org/dev/api/page__execution__system__design.html#computations)).

## 2.2 Pixar Presto and OpenExec

**Adopt.** Pixar presents OpenExec as bringing the execution architecture used by Presto to OpenUSD through a scene-adapter/compiler boundary ([OpenExec ASWF architecture talk](https://openusd.org/files/OpenExecASWF.pdf)). Its verified public model is a strongly typed, vectorized dataflow network compiled from scene description plus registered computations; requests produce reusable schedules; pull evaluation stops at cache hits and runs invalid nodes with the parallel executor enabled by default. Non-structural value changes invalidate data without recompiling topology ([OpenExec System Design](https://openusd.org/dev/api/page__execution__system__design.html)).

Presto material is precedent, not current API. Its [2015](https://www.multithreadingandvfx.org/course_notes/2015/presto_threading.pdf) and [2017](https://www.multithreadingandvfx.org/course_notes/2017/florian_presentation_2017.pdf) courses cover graph/model/frame parallelism, background caches, stale-result discard, buffer masks, strip mining, and isolated time values; [PCF](https://research.pixar.com/docs/2022.SiggraphTalks.LKNHMLLK.pdf) adds aggregate/cache and GPU limits. RigExec adopts those ideas without attributing cancellation, concurrent time contexts, or residency controls to OpenExec.

The actual source stack is important:

  - vdf — vectorized dataflow/scheduling/executors; ef — execution/cache utilities.
  - esf — read-only scene interfaces; esfUsd — USD adapters.
  - exec — scene-independent compiler/runtime/registry/requests/invalidation.
  - execUsd — USD system, keys, requests, cache views, time/change processing.
  - execGeom — UsdGeom computations; execIr — limited invertible-rig schemas/controllers.

These are visible in the [pxr/exec source tree](https://github.com/PixarAnimationStudios/OpenUSD/tree/v26.08/pxr/exec) and their library READMEs. execUsd explicitly depends on exec and esfUsd; ExecSystem accepts an EsfStage. This is the seam required for dual deployment, not an inferred replacement point.

The 25.08–26.08 [changelog](https://github.com/PixarAnimationStudios/OpenUSD/blob/v26.08/CHANGELOG.md#openexec) records requests, sparse invalidation, additional providers, cycle checks, overrides, preliminary ExecIr, and exec imaging; it also warns that rig APIs remain limited and unstable.

**Reject.** We do not attribute a geometry-mover library, complete IK system, interaction loop, arbitrary Python registration, public multi-time prefetch/residency policy, production rig schemas, or supported standalone adapter SPI to OpenExec. An internal time-indexed cache exists, but it is not that product contract. The [OpenExec introduction](https://openusd.org/release/intro_to_openexec.html#what-openexec-is-not) says the framework is not a rigging system, is value-driven rather than event-driven, and cannot modify a stage. Current [ExecIr documentation](https://openusd.org/dev/api/page__exec_ir_readme.html) also says it has no IK solver schemas.

## 2.3 Hydra 2.0 and composition-native publication

Hydra 2.0 separates scene transformation from rendering through scene indices. A filtering scene index observes an input scene index, presents a transformed view, and forwards added/removed/dirtied notices; HdSingleInputFilteringSceneIndexBase is the standard base. Renderers consume the terminal scene without needing to understand upstream filters ([Hydra 2.0 Getting Started Guide](https://openusd.org/dev/api/_page__hydra__getting__started__guide.html)).

RigExec deliberately does not adopt a monolithic skeleton/binding resolver as an architecture or conformance reference. Its required authoring grain is smaller: controls, joints, transform providers, one-transform matrix movers, reusable static or dynamic weight objects, mover applications, and output mappings are independent typed prims, properties, and relationships that compose through ordinary USD layers, references, variants, and list editing. The compiler may pack those composed values for execution, but no packed skeleton aggregate is authoritative authoring data.

OpenUSD 26.08 includes an environment-gated usdExecImaging scene index that overlays OpenExec-computed xforms for ExecIrJointScope guides and UsdGeomXformable prims ([26.08 changelog](https://github.com/PixarAnimationStudios/OpenUSD/blob/v26.08/CHANGELOG.md#usd-imaging)).

**Adopt.** RigExec uses this filter-and-overlay pattern, Hydra data-source locators, and precise notices. **Build.** The current exec imaging path does not publish RigExec’s moved points, curves, and arbitrary computed primvars, so the RigExecInternalPrimPruningSceneIndex / RigExecBindingResolvingSceneIndex / RigExecResultsSceneIndex chain extends the pattern. **Reject.** No render delegate receives RigExec-specific APIs or pulls the rig directly.

## 2.4 Secondary prior art (bounded survey)

This survey informs product choices but does not override the primary OpenExec design:

  - Houdini APEX: adopt graph assets/subgraphs/geometry-as-data, not its DCC evaluator (APEX basics).
  - Maya EM/GPU override: adopt capability/fallback telemetry, not a second evaluator (EM, GPU override).
  - Blender depsgraph: adopt authored/evaluated separation, with OpenExec still authoritative (design).
  - ozz-animation: adopt portable SoA/offline packs, not its runtime as a DCC graph (repository).
  - Interactive rigging systems: useful future consumers of typed overrides and taps, but their UI, transaction, and manipulation designs are outside this infrastructure plan.

# 3\. System architecture overview

## 3.1 Layered architecture

**Status:** OpenExec core **EXISTS**; RigExec layers **BUILD**; selected upstream rig/imaging seeds **EXISTS, experimental**.

**Pipeline:** OpenUSD authoring · schemas, mover hierarchy, animation layers → RigExec schema compiler · post-order mover lowering + validation  
Standalone .rigpack · derived from composed USD → RigExec Esf standalone adapter  
OpenUSD authoring · schemas, mover hierarchy, animation layers → EsfUsd scene adapter  
EsfUsd scene adapter → OpenExec ExecSystem · compile → schedule → evaluate  
RigExec Esf standalone adapter → OpenExec ExecSystem · compile → schedule → evaluate  
RigExec schema compiler · post-order mover lowering + validation → OpenExec ExecSystem · compile → schedule → evaluate  
OpenExec ExecSystem · compile → schedule → evaluate → VDF parallel dataflow · typed values, caches, invalidation  
VDF parallel dataflow · typed values, caches, invalidation → RigExec Tap/Request API · immutable generation snapshots  
RigExec Tap/Request API · immutable generation snapshots → Hydra 2.0 filtering scene index  
RigExec Tap/Request API · immutable generation snapshots → Parity tests, cache export, · standalone pack validation  
Hydra 2.0 filtering scene index → Any Hydra renderer

Four contracts govern the system:

1.  Authoring: composed USD plus registered schemas fully describes intent; callbacks neither discover inputs nor author.
2.  Compilation: stable base/final addresses, partitions, post-order mover lowering, paired transform views, and fixed epoch topology.
3.  Evaluation: pure callbacks/immutable results; OpenExec owns schedule/cache, RigExec deadlines/generations/residency.
4.  Publication: consumers see complete RigExecSnapshot generations only.

## 3.2 One core, two scene backends

**Pipeline:** UsdStage → ExecUsdSystem · notice, time, request façade  
ExecUsdSystem · notice, time, request façade → internal EsfUsdSceneAdapter  
internal EsfUsdSceneAdapter → adapted EsfStage view  
RigExecSceneDb or .rigpack → RigExecStandaloneSystem · delta, time, request façade  
RigExecStandaloneSystem · delta, time, request façade → RigExecEsfSceneAdapter  
RigExecEsfSceneAdapter → adapted EsfStage view  
adapted EsfStage view → shared/inherited ExecSystem internals · compiler + runtime  
adapted EsfStage view → shared/inherited ExecSystem internals · compiler + runtime  
shared/inherited ExecSystem internals · compiler + runtime → ef / vdf scheduler, cache, parallel executor

EsfStageInterface and companion object/property/query interfaces supply the read-only seam; EsfUsdSceneAdapter::AdaptStage() implements USD and protected ExecSystem(EsfStage&&) is scene-independent ([Esf](https://github.com/PixarAnimationStudios/OpenUSD/blob/v26.08/pxr/exec/esf/stage.h), [adapter](https://github.com/PixarAnimationStudios/OpenUSD/blob/v26.08/pxr/exec/esfUsd/sceneAdapter.h), [system](https://github.com/PixarAnimationStudios/OpenUSD/blob/v26.08/pxr/exec/exec/system.h)). Because the [Esf README](https://github.com/PixarAnimationStudios/OpenUSD/blob/v26.08/pxr/exec/esf/README.md) marks it non-public, Phase 0 must prove lifecycle/change/request parity against an unchanged, version-pinned stock build behind one RigExec compatibility library. A future public SPI may replace that boundary, but the plan neither requires nor modifies OpenUSD to create it.

## 3.3 Component boundaries

|  |  |  |
| :-: | :-: | :-: |
| \*\*Library\*\* | \*\*Status\*\* | \*\*Responsibility\*\* |
| rigExecSchema | BUILD | Generated USD typed/applied schemas, tokens, validators, composition conventions. |
| rigExecMath | BUILD | RigExecPointFrame, frame reconstruction, SRT/affine conversion, IK, solver, and value-blend kernels; temporal interpolation remains stock USD. |
| rigExecCompute | BUILD | Backend-neutral layer on exec: registrations, controllers, solvers, transform adapters, tap metadata; no execUsd dependency. |
| rigExecMove | BUILD | Mover discovery, target-specific writer chains, immutable geometry and weight packets, and matrix/blend/lattice/curve/surface/post kernels. |
| rigExecUsd | BUILD | USD-facing system façade, native resolved-value bindings, rig compiler, .rigpack exporter. |
| rigExecEsfStandalone | BUILD | Standalone scene database and system behind the existing internal Esf seam. |
| rigExecImaging | BUILD | Hydra scene-index filters/data sources aligned with existing experimental usdExecImaging. |
| rigExecValidation | BUILD | Structural validators, deterministic parity/golden harnesses, benchmark trace schema, and render-preflight checks. |

  

## 3.4 OpenExec gap matrix

The status column uses only the required factual buckets. Shipping code is **EXISTS** even when experimental; **PLANNED** is reserved for explicitly signaled future work. Maturity and product reliance stay in the evidence/action columns.

|  |  |  |  |
| :-: | :-: | :-: | :-: |
| \*\*Subsystem / capability\*\* | \*\*Status\*\* | \*\*Verified basis\*\* | \*\*RigExec action\*\* |
| Vectorized dataflow network | EXISTS | vdf and system-design docs | Use unchanged. |
| Execution data/cache utilities | EXISTS | ef library | Use unchanged. |
| Scene-independent compiler/runtime | EXISTS | exec, ExecSystem(EsfStage&&) | Use unchanged behind a pinned RigExec compatibility boundary. |
| USD scene adapter | EXISTS | esfUsd, ExecUsdSystem | Native USD deployment. |
| Non-USD scene-access seam | EXISTS | esf interfaces exist but README disclaims public use | BUILD adapter outside OpenUSD; qualify stock binaries/source visibility per platform and stop if unavailable. |
| Compile → schedule → evaluate phases | EXISTS | OpenExec System Design | Prepare stable batched requests; do not rebuild a scheduler. |
| Pull evaluation, automatic caching, sparse invalidation | EXISTS | Intro, design, 25.08 release notes | Use unchanged. |
| Parallel executor enabled by default | EXISTS | System Design; VDF\\\_ENABLE\\\_PARALLEL\\\_  &#10;EVALUATION\\\_ENGINE | Compose with outer work budget; do not create nested unbounded pools. |
| Strongly typed arbitrary computation values | EXISTS | Intro and VDF API | Register point-frame, geometry-packet, and diagnostics value types. |
| Public element extraction from plugin vector results | BUILD | Current extraction type equals result type; value keys have no element selector | Use aggregate result types plus scalar semantic providers; element selection is post-cache. |
| Built-ins such as computeValue and computePath | EXISTS | Intro; 26.05 release notes | Reuse for attributes and provider identity. |
| Schema-bound C++ computation DSL | EXISTS | EXEC\\\_REGISTER\\\_  &#10;COMPUTATIONS\\\_  &#10;FOR\\\_SCHEMA tutorial | Native extension mechanism. |
| Python computation registration | BUILD | Docs say registration is currently C++ only | Deferred; production and conformance registrations are native. |
| Batched request/cache-view extraction | EXISTS | ExecUsdRequest, ExecUsdCacheView | Wrap in typed RigExecTapSet. |
| Request invalidation callbacks/time ranges | EXISTS | OpenExec client API | Callback enqueues work only; no re-entry. |
| Change-time invalidation and prepared requests | EXISTS | ExecUsdSystem::ChangeTime, PrepareRequest | Time-sequenced evaluation foundation. |
| Internal time-indexed cache | EXISTS | EfPageCacheStorage\\\<EfTime\\\> in runtime | Use; do not promise controls the public API lacks. |
| Public multi-time prefetch/residency/prediction | BUILD | No public batch-time, eviction, or prediction contract | Bounded snapshot cache for frame and render-sample residency above exec. |
| Transient value overrides | EXISTS | ComputeWithOverrides in 26.05+ | Retain in the low-level runtime and parity suite; interactive consumers are deferred. |
| Experimental ExecIr inversion bridge | EXISTS | 26.08 ships limited FK/switch controller plumbing and marks it unstable | Align behind adapter; do not serialize its types. |
| Native/general inversion API | PLANNED | ExecIr documentation describes future native inversion work | Keep controller seam replaceable and parity-tested. |
| General RigExec inverse solvers | BUILD | No general/native public inversion facility | Deferred beyond v0.1; preserve a pure-computation boundary without UI commitments. |
| Connection dataflow | EXISTS | 26.08 supports only one valid same-typed attribute connection | Use one-to-one connections; relationships for variable arity; validate limitations. |
| Computation composition across applied/concrete schemas | BUILD | Current source marks cross-registration definition composition TBD | One authoritative schema registration owns each computation name. |
| Dynamic property/primvar discovery in callbacks | BUILD | Registration inputs are static; property/namespace-child inputs are TODO | Finite build-time manifests and plugin-declared primvar adapters; reject undeclared entries. |
| IK solvers, constraints, ribbons, twist distribution | BUILD | ExecIr docs explicitly omit IK; no complete rig library | Implement as RigExec schemas/computations. |
| Production rig object model | BUILD | Intro says OpenExec alone is not a rigging system | Implement schemas, compiler invariants, versioning. |
| Point-native affine transform model | BUILD | No upstream schema/API contract | Implement RigExecPointFrame and adapters. |
| Mover/write-chain object model | BUILD | No rig operation ordering/target-write schema | Implement mover APIs, typed generated applications/finalizers, post-order lowering, phase checks, diagnostics, and taps. |
| Mesh/curve mover library | BUILD | No OpenExec geometry-operation library | Implement vectorized immutable geometry computations. |
| Event/direct-manipulation layer | BUILD | Intro says OpenExec is not event-driven | Deferred; not part of the data/rendering infrastructure plan. |
| Native TsSpline attribute value resolution | EXISTS | OpenUSD 26.08 timed UsdAttribute::Get and UsdAttributeQuery::Get evaluate resolved spline data | Use .spline knots for scalar half, float, double, and timecode attributes; pass an explicit numeric or PreTime time code. |
| Native sparse timeSamples interpolation | EXISTS | OpenUSD 26.08 resolves sparse samples with built-in linear/held interpolation | Use for vectors, matrices, quaternions, arrays, and non-interpolatable values; accept preexisting scalar samples unchanged. |
| Custom animation source or sampler | BUILD | Not required by stock OpenUSD 26.08 value resolution | Reject for v0.1; consume only resolved standard USD attribute values and test parity through OpenExec. |
| Hydra filtering scene-index framework | EXISTS | Hydra 2.0 guide and hdsi | Use unchanged. |
| Exec-backed transform imaging | EXISTS | 26.08 implementation is xform-only, environment-gated, and experimental | Align; build production points/primvars filter. |
| Uniform “tap any computation output” façade | BUILD | OpenExec requests named computation values, but schedules are opaque | Build on keys/requests; enforce addressable outputs and typed snapshots. |
| Topology/cycle product policy | BUILD | OpenExec forbids scene mutation and detects graph cycles | Structural authoring creates epochs; validator rejects cycles; multipass sits above graphs. |
| Cross-platform desktop product | BUILD | OpenUSD is the dependency foundation | CI, ABI policy, packaging, determinism tests. |
| iPadOS runtime | BUILD | Embedded-Apple build support is only a dependency lead, not an OpenExec product promise | Phase 0/4 proof for headless runtime and Hydra data-source publication validation; static runtime, no Python, mobile LOD, and no on-device renderer. |

  

## 3.5 Significant architecture choice: extend OpenExec or wrap it

**Alternative A — call OpenExec only through** **ExecUsdSystem****.** This minimizes contact with internal APIs but makes the standalone requirement impossible without constructing a UsdStage.

**Alternative B — implement a scene backend behind the Esf interfaces shipped in stock OpenUSD and expose a thin RigExec façade. Chosen.** This shares the compiler, network, scheduler, executor, and computation registrations without changing OpenUSD. The cost is managing a non-public seam in one version-pinned RigExec library. Phase 0 is an exit gate: if a standalone Esf proof cannot compile/link/load or pass parity against ExecUsdSystem on a qualified stock build, the program stops and revisits standalone scope; it does not patch OpenUSD or silently create a second engine.

# 4\. Data model and OpenUSD schemas

## 4.1 Authoring rules

**Status:** USD schemas/composition and limited experimental ExecIr schemas **EXISTS**; RigExec schema domain **BUILD**.

The schema domain is named RigExec. These proposed product names do not claim to be upstream OpenUSD:

|  |  |  |
| :-: | :-: | :-: |
| \*\*Schema\*\* | \*\*Kind\*\* | \*\*Purpose and published computations\*\* |
| RigExecRig | Typed prim | Character root, schema/version, partition, controls, outputs, mover discovery, and LOD policy. Publishes computeRigBounds, computeDiagnostics. |
| RigExecControl | Typed prim | Animator-authored control value with canonical pose points, channel semantics, and limits. Publishes computePointFrame, computeMatrix. |
| RigExecJoint | Typed prim | Bind/rest identity and posed result. Publishes computePointFrame, computeMatrix. |
| RigExecFkChain | Typed solver | Applies control frames/offsets to a rest hierarchy. Publishes aggregate computePointFrameArray; addressable joint providers publish scalar frames. |
| RigExecTwoBoneIk | Typed solver/operation | Analytic two-bone IK with pole, stretch, softness, and preferred bend. It may publish frames as a provider or atomically move named joint targets. |
| RigExecBlendPointFrames | Typed solver/operation | IK/FK or general frame blend. It may publish an array or move declared frame targets. |
| RigExecAimConstraint | Typed operation | Moves a transform target by replacing aim/up orientation while preserving declared position/scale components. |
| RigExecFloatMathMover, RigExecVec3fMathMover, RigExecMatrixMathMover | Typed operations | Statically typed add, multiply, clamp, remap, or blend over an exact property target. |
| RigExecTwistDistribution | Typed solver | Distributes unwrapped twist over N frames. Publishes computePointFrameArray. |
| RigExecRibbon | Typed solver | Samples a driver curve, constructs transported frames, and optionally conforms to a surface. Publishes computePointFrameArray, computeCurvePoints. |
| RigExecPointFrameView | Typed adapter | Selects one declared element of an aggregate frame result and publishes scalar computePointFrame/computeMatrix for stable extraction and downstream computation. |
| RigExecBlendShapeMover | Typed mover | Applies independently composed blend inputs and samples to a geometry target. |
| RigExecBlendInput | Typed prim | One independently composable blend channel with an authored/connected weight, target-space policy, and relationships to samples. |
| RigExecBlendSample | Typed prim | One target or in-between activation with exact delta and optional sparse-index property relationships. |
| RigExecMatrixMover | Typed mover | Moves the preceding points through one declared affine transform and blends each point with exactly one compatible RigExecWeightObject; it contains no joint set, bind table, or packed transform list. |
| RigExecWeightObject | Abstract typed prim | Common contract for a total scalar weight field over the logical elements of one exact prim or property target. Publishes computeWeightDescriptor and computeWeightPacket. |
| RigExecStaticWeight | Concrete RigExecWeightObject | Provides a time-invariant constant, dense, or sparse authored weight field; every target element resolves to an explicit value or the authored sparse default. |
| RigExecDynamicWeight | Concrete RigExecWeightObject | Provides an epoch-shape-stable field whose values are recomputed from statically declared inputs, optionally modulating another weight object. |
| RigExecLatticeMover | Typed mover | Cage/lattice input and point output. |
| RigExecCurveMover | Typed mover | Wire, spline-IK, ribbon, and curve-coordinate movement. |
| RigExecSurfaceMover | Typed mover | Surface attachment/projection/sliding and surface-driven correction. |
| RigExecPostMover | Typed mover | Normal/tangent rebuild, smoothing, volume correction, clamps, and user kernels. |
| RigExecMoverAPI | Applied API | Adds uniform rel rigExec:moves and inputs:enabled (fallback true); marks an operation for hierarchy-ordered lowering. |
| RigExecMoveTargetAPI | Applied API | Marks a geometry prim as writable and owns its public base/final result boundary; property writability is declared by the owning schema’s catalog metadata. |
| Generated mover application schemas (canonical spellings below) | Generated typed prims | Pre-registered session-layer/pack nodes and typed finalizers that realize one writer revision. Each publishes value and status views from one atomic application result; MatrixValue means a mover that writes a matrix-valued target and is distinct from the point-moving RigExecMatrixMover. Atomic solver bundles use a registered bundle application. |
| RigExecPointTransformAPI | Applied API | Adds rigExec:restPoints, rigExec:posePoints, frame policy, twist, scale/shear limits and parent relationship. |
| RigExecControlAPI | Applied API | Channel semantics and computational limits; keyability, mirroring, presentation, and manipulation metadata are deferred. |
| RigExecGeometryDriverAPI | Applied API | Adapts authored mesh/points/curves into computeSourceGeometry from a finite declared primvar manifest. |
| RigExecPartitionAPI | Applied API | Stable character/region partition ID and deadline/LOD hints. |
| RigExecTapAPI | Multiple-apply API | Gives a stable alias and semantic role to a published result. |

  

The canonical generated schema names are RigExecPointFrameMoverApplication, RigExecFloatMoverApplication, RigExecVec3fMoverApplication, RigExecMatrixValueMoverApplication, and RigExecGeometryMoverApplication.

Granularity is an authoring contract, not merely an execution detail. No aggregate skeleton object owns joint membership, bind state, transform-weight tables, solver order, or output binding. Each RigExecJoint, transform provider, RigExecMatrixMover, weight object, other mover, source property, and relationship is independently addressable and may receive a composed opinion from a reference, inherit, variant, or stronger layer. Composition resolves first; only then does the compiler validate the complete rig, freeze a BindingEpoch, and pack coherent buffers for vectorized execution. Packing may optimize evaluation but never becomes the durable identity or override boundary.

RigExecMatrixMover is intentionally one operation, not a combined multi-transform aggregate. rel rigExec:moves names the exact points result it changes, rel rigExec:transform resolves one GfMatrix4d provider, and rel rigExec:weightObject resolves exactly one compatible weight provider. The transform is already a rig-common-to-rig-common affine map; the mover never discovers joints or constructs bind transforms. A rest-to-pose matrix can come directly from a point-frame provider (§5) or from a separate, independently composable matrix-math provider. Multiple matrix movers are ordinary hierarchy-ordered writers, so each may be added, deleted, retargeted, reordered, or overridden without rebuilding an aggregate transform table.

Every weight object has one rel rigExec:weightTarget naming a prim or exact property. A prim target canonicalizes through RigExecComputationCatalog to that prim’s primary weightable value; scalar targets have one logical element, while arrays and geometry point results have their catalogued element count. The descriptor domain is compiler-derived, not freely authored: native v1 tokens are scalar for one scalar value, point for the point domain of mesh/points/curves or a registered point array, and element for another registered array’s logical elements. A plugin domain token is legal only when the catalog registers its target types, cardinality rule, matrix/action semantics, blend operation, coordinate space, and output mapping. Thus the same weight contract applies to controls, joints, arbitrary catalogued properties, meshes, points, curves, and plugin prims rather than being restricted to a specialized geometry schema.

The field is total and has one canonical encoding. constant stores no rigExec:values elements and broadcasts rigExec:defaultWeight; dense stores exactly one value per logical element and requires defaultWeight = 0 as ignored canonical padding; sparse stores equal-sized index/value arrays and uses defaultWeight everywhere else. Indices are unique non-negative logical element indices. The compiler sorts sparse (index,value) pairs numerically; authored pair order is non-semantic. An atomic order-only permutation that preserves the same index/value mapping produces the same descriptor, pack, and result and does not rebuild the epoch.

Any concrete mover may opt into this catalogued weight contract, but it must register the matrix/action and blend operation for its target type and role. RigExecMatrixMover requires one weight object; RigExecBlendShapeMover accepts one optional per-point object; scalar/property movers may consume a one-element field. Unknown types or domains are rejected rather than inferred from equal array lengths.

RigExecStaticWeight owns time-invariant float\[\] rigExec:values, optional int\[\] rigExec:indices, and float rigExec:defaultWeight; it rejects time samples and value connections. RigExecDynamicWeight keeps the same target, representation, and cardinality for a BindingEpoch but publishes a current immutable RigExecWeightPacket from registered inputs. For the built-in multiply operation, let bi be the optional base packet and let d, s, and a be inputs:driver, inputs:scale, and inputs:bias:

ri=(bid)s+a,wi=RangePolicy(ri).

When a base object exists, its canonical target, domain, representation, cardinality, and sparse indices must exactly match the dynamic descriptor. Without a base, only constant representation is legal and bi=1 for every target element. For constant, the computed broadcast is stored as the packet default with no values. For sparse, the equation transforms every explicit value and the sparse default. For dense, it transforms all logicalCount values and retains canonical packet default zero even when bias is nonzero. RangePolicy first rejects non-finite input; strict rejects values outside \[0,1\], while clamp clamps them, with strict the release default. Plugin dynamic-weight schemas may add operations only by statically registering every input. A target, representation, domain, cardinality, canonical sparse support, relationship or attribute-connection identity, operation-kind, or policy edit is structural because it changes compiled dependencies; compatible static values/default, values from an unchanged connected provider, dynamic output values, and order-only sparse pair permutations are value-only.

Blend channels are likewise not parallel name/weight/target arrays. RigExecBlendShapeMover.rel rigExec:blendInputs targets independently composed RigExecBlendInput prims. Each input owns or connects one weight and targets one or more RigExecBlendSample prims. Sample target order is non-semantic; activations must be unique and compile in (activation, canonicalPath) order. Sparse indices and deltas are exact property relationships. Adding or removing an input/sample is structural; changing a compatible weight or delta value is value-only.

Generated schema classes provide the production C++ authoring API; generated language bindings are allowed only in build and conformance tests. Evaluation uses EXEC\_REGISTER\_COMPUTATIONS\_FOR\_SCHEMA. Because partial definition composition across schemas is a source TODO, one schema owns each complete computation. Drivers own computeSourceGeometry; move targets expose distinct base/final computations; pre-registered application schemas own typed writer callbacks. Every concrete mover/operation schema also owns a statically registered computeMoverParameters computation: it declares every schema-specific attribute/relationship input and returns a registered immutable RigExecMoverParameters packet. The generic result-typed application schemas therefore need only fixed previous, moverParameters, and target-descriptor inputs; callbacks never discover properties or read the stage. Point-transform and mover APIs are data-only. No callback code appears in authored .usda.

Before OpenExec compilation, the RigExec compiler authors deterministic hidden application prims under each rig’s \_\_RigExecGenerated scope in an anonymous derived session layer. Native v0.1 first finds the nearest instanceable ancestor (normally the referenced asset root) and authors instanceable=false there in the derived layer; a rig in a prototype or otherwise unable to deinstance is rejected. Each generated prim then has one pre-registered result-typed schema and links to the previous revision, the mover’s computeMoverParameters, and the target descriptor; typed target finalizers receive the chain head. Built-ins use a closed packet variant; plugins register an immutable payload type with equality/hash and a schema/version tag. RigExecInternalPrimPruningSceneIndex removes only that reserved scope from imaging. Standalone packs store equivalent application/dependency records, not generated USD prims or evaluated parameter packets.

RigExecUsdSystem exclusively owns one anonymous generated sublayer per stage. Before constructing ExecUsdSystem, it inserts that layer into the editable session layer, verifies its opinions are strong enough to deinstance, and compiles under the stage coordinator. For a local structural transaction, the compiler prepares and validates replacement records off-stage, then applies only the affected generated-spec diff to the persistent owned layer inside one SdfChangeBlock while evaluation is quiescent. The existing ExecUsdSystem processes those resync scopes and incrementally rebuilds affected providers and requests; the old published snapshot remains visible until the new binding epoch succeeds. The system is recreated only for stage replacement or an explicitly unsupported full reset, not for an ordinary epoch. Teardown removes only the owned sublayer. Ordinary root-layer save never includes it, and RigExec publish/flatten/export operates on an authoring-stage view with the generated layer removed; reserved generated paths in a durable layer fail validation. Failure to isolate, update, or exclude this static-input lowering on both backends is a Phase 1 stop gate.

Move-addressable providers expose computeBasePointFrame/computePointFrame, computeSourceGeometry/computeGeometry, or property-specific base/final computations. Registrations for catalogued movable inputs use project helpers RigExecBaseValue\<T\> or RigExecResolvedValue\<T\>. Upstream AttributeValue\<T\> binds attribute computeValue and may follow a valid connection, so it is allowed only for non-move-addressable inputs; it is ambiguous and forbidden wherever a base/final move phase is promised. RigExecBaseValue explicitly binds the catalogued base/resolved-authored provider, while generated relationships/connections bind RigExecResolvedValue to the chain head.

Values are wired in two ways:

  - Attributes and attribute connections carry scalar or built-in-array values where there is exactly one same-typed producer. In OpenUSD 26.08, OpenExec’s computeValue follows exactly one valid same-type connection; invalid, multiple, or wrong-type connections fall back to the owning attribute’s resolved value. Rig validation rejects ambiguous connections before evaluation.
  - Ordinary relationships target provider prims for variable-arity inputs such as joints, curve drivers, surfaces, or frame producers. Registration resolves the named computation/type expected from each target.
  - rel rigExec:moves is the reserved write-set relationship. Its targets are exact prim or property paths; target-list order is deliberately non-semantic. The mover prim’s position in the composed USD namespace supplies operation order.

An IK, aim, constraint, math, or geometry computation without moves is only a value provider; once it modifies an existing result it must apply RigExecMoverAPI and declare the complete write set. This distinction works with current OpenExec rather than assuming future multi-connection behavior.

## 4.2 Mover targeting, hierarchy order, and last-writer semantics

**Status:** **BUILD**. OpenUSD supplies relationships and a composed child order; RigExec defines their execution meaning. A [UsdRelationship](https://openusd.org/release/api/class_usd_relationship.html) can target prims, attributes, or relationships and is uniform over time. RigExec narrows moves to existing prims or non-structural, computation-catalogued attributes. Relationship-valued and topology properties plus undeclared or dangling paths are rejected; child-order metadata is not a targetable scenegraph property.

Every RigExecRig discovers mover-bearing prims beneath its composed Movers child. The compiler iterates [UsdPrimRange::PreAndPostVisit()](https://openusd.org/release/api/class_usd_prim_range.html) and processes only IsPostVisit() entries, yielding a deterministic post-order depth-first walk for a fixed composed stage: descendants before ancestors and branches in the composed order returned by [UsdPrim::GetChildrenNames()](https://openusd.org/release/api/class_usd_prim.html). v0.1 uses active, loaded, defined, non-abstract prims on deinstanced rig roots and never traverses instance proxies; load, activation, population, or instancing-policy changes are structural. GetChildrenReorder() alone is not the final composed order. Same-target movers should be nested; release validation otherwise requires a parent-authored child-reorder opinion—reorder nameChildren = \[...\] in USDA, authored through SetChildrenReorder()—listing every competing direct-child name, then still reads effective order from GetChildrenNames(). No stack list, relationship-target order, plugin order, request order, or worker completion time participates.

Target resolution is exact:

  - A prim target maps through RigExecComputationCatalog to one schema-declared primary writable result: normally computePointFrame for a transform prim or computeGeometry for a geometry prim. If the prim has no unique primary result, authoring must name a property.
  - A property target maps to exact owner-schema catalog metadata and a statically typed application; applied APIs attach only to prims. Prim and property spellings that resolve to one result canonicalize to one address.
  - Default multi-target fan-out creates independent applications/failures per target at one ordinal. A schema-declared atomic bundle instead returns all named targets together and fails/passes through as a unit. IK maps root/mid/end with schema-specific role relationships whose target set must equal moves; it never infers roles from list position. Target-list permutation changes neither mode, mapping, order, nor result.

RigExecMatrixMover narrows the general rule: moves, transform, and weightObject each have cardinality one. Its move target must resolve to a catalogued point-array value; a mesh, points, or basis-curves prim target canonicalizes to its points result. Its weight target must canonicalize to that same value and logical point count. Reusing one weight object is legal only for movers on that same canonical target; fan-out is authored as one matrix-mover/weight-object pair per target.

For the post-order movers m1,…,mn, let S0 map every writable target to its base value. Compilation lowers:

Si={Apply(mi,Si−1) if mi is enabled, Si−1 otherwise, Resolved(a)=Sn\[a\].

The implementation is static single-assignment, not mutable shared state: base(a) → after(m1,a) → … → final(a). Each declared read carries phase base, preceding, or final: preceding reads the head before the reader ordinal; final is legal only when every writer of that target precedes the reader and no cycle results. Self/later-final reads fail compilation. Overlapping writes serialize; disjoint chains remain parallel. Every transform revision publishes paired point-frame and matrix views derived from the same value/reference. A cached node is still logically applied, so “fired last” means **last in compiled logical order**, never last callback to finish.

Disabled or zero-weight operations pass through. Independent fan-out fails per target; an atomic IK failure passes through its whole bundle. Aim/math failures pass through with diagnostics unless an explicit finite fallback is authored. “Hold last valid” is publication policy, not callback state.

Adding, removing, activating, reparenting, or renaming a mover; editing moves; changing child order, operation kind, target type/cardinality, read phase, or composition/variant selection begins a new epoch. Numeric inputs, weights, goals, time samples, compatible spline-knot/tangent/extrapolation edits, and direct inputs:enabled edits are value-only **only when pass-through preserves type, cardinality, primvar manifest, and output set**; otherwise enablement is structural or prohibited. Switching an attribute between time-sample, spline, default, or clip value-source forms is validated under standard USD resolution and invalidates its dependent value ranges; it does not create a RigExec curve object. active=false is structural and removes descendants. Cross-rig targets are rejected in v1; cross-character effects use §6.2’s multipass.

## 4.3 Point-frame storage versus runtime values

USD-authored frames use built-in point3d\[\] attributes with exactly four elements in order \[O, X, Y, Z\]; validators enforce cardinality and finiteness. Runtime scalar providers return registered RigExecPointFrame. Packed solver boundaries return a registered aggregate scalar RigExecPointFrameArray with an immutable shared span. Although VDF is vectorized internally, current plugin-computation extraction fixes extraction type to result type and ExecUsdValueKey has no element selector; RigExec therefore does not claim native client indexing of an arbitrary VDF vector or use VtArray\<RigExecPointFrame\> as a result.

Every semantically addressable element also has a scene-addressable scalar provider—for example each RigExecJoint or ribbon-sample provider consumes the aggregate and publishes computePointFrame. A tap’s optional element selector indexes a project-owned aggregate *after* extraction; it is not part of the upstream key. This preserves packed execution without hiding semantic transforms.

RigExecGeometryPacket, by contrast, is one registered scalar result holding shared immutable point/primvar buffers. Its internal arrays are the data-parallel payload; VDF vectorization and SIMD are separate layers. This avoids incorrectly equating the word “vectorized” in OpenExec with automatic SIMD or GPU deformation.

## 4.4 Composition and layer ownership

Rig assets use four conceptual layers:

1.  Model layer — rest mesh/curves, topology, blend-sample deltas, and immutable source geometry.
2.  Rig-definition layer — controls, joints, solvers, relationships, computation-bearing schemas, independently addressable rest/pose transforms, and default values.
3.  Binding/customization layer — matrix movers, exact transform and weight-object relationships, static/dynamic weight prims, blend inputs/samples, mover targets, character-specific corrective targets, hierarchy/enable variants, and asset overrides.
4.  Animation layer — native .spline opinions for eligible scalar channels; ordinary sparse .timeSamples for point arrays, matrices, booleans/tokens, arrays, and other supported values; plus deliberate baked standard-USD results.

The asset layer references the model/rig; a shot references the asset. Selecting a rigComplexity variant is structural and starts a new epoch even when it changes only authored enable values; direct runtime LOD overrides may change shape-preserving inputs:enabled without recompilation. Payloads may defer high-resolution geometry, but evaluation never changes loaded topology. Validation and chain hashing run after composition.

Animation authors durable control/avar values, not solver outputs. Standard USD composition and value resolution complete before execution consumes those values; solver and mover results remain derived. Editing workflows and transaction policy are outside the current scope.

## 4.5 Complete rig-definition example: ArmRig.usda

The following is complete scene description for a compact but nontrivial arm. The schema plugin supplies fallbacks and computation definitions; the asset supplies authored values and wiring. It contains FK and two-bone IK, an IK/FK blend, twist distribution, a curve-driven ribbon, three independently composable weighted matrix movers, and blend-shape, curve, and post stages.

\#usda 1.0  
(  
    defaultPrim = "ArmAsset"  
    metersPerUnit = 0.01  
    upAxis = "Y"  
)  
  
def Xform "ArmAsset" (  
    prepend variantSets = "rigComplexity"  
    variants = {  
        string rigComplexity = "film"  
    }  
)  
{  
    def RigExecRig "Rig"  
    {  
        uniform token rigExec:schemaVersion = "0.1"  
        uniform token rigExec:partition = "ArmAsset"  
        rel rigExec:controls = \[  
            \</ArmAsset/Rig/Controls/ShoulderFK\>,  
            \</ArmAsset/Rig/Controls/ElbowFK\>,  
            \</ArmAsset/Rig/Controls/WristFK\>,

            \</ArmAsset/Rig/Controls/HandIK\>,  
            \</ArmAsset/Rig/Controls/ElbowPole\>  
        \]  
        rel rigExec:jointOutputs = \[  
            \</ArmAsset/Rig/Joints/Shoulder\>,  
            \</ArmAsset/Rig/Joints/Elbow\>,  
            \</ArmAsset/Rig/Joints/Wrist\>  
        \]  
  
        def Scope "Controls"  
        {  
            def RigExecControl "ShoulderFK" (  
                prepend apiSchemas = \["RigExecPointTransformAPI", "RigExecControlAPI"\]  
            )  
            {  
                point3d\[\] rigExec:restPoints = \[  
                    (0, 10, 0), (4, 10, 0), (0, 11, 0), (0, 10, 1)  
                \]  
                point3d\[\] rigExec:posePoints = \[  
                    (0, 10, 0), (4, 10, 0), (0, 11, 0), (0, 10, 1)  
                \]  
                uniform token rigExec:framePolicy = "orthogonal"

                uniform token rigExec:aimAxis = "x"  
                uniform token rigExec:upAxis = "y"  
                double rigExec:twist = 0  
            }  
  
            def RigExecControl "ElbowFK" (  
                prepend apiSchemas = \["RigExecPointTransformAPI", "RigExecControlAPI"\]  
            )  
            {  
                point3d\[\] rigExec:restPoints = \[  
                    (4, 10, 0), (8, 10, 0), (4, 11, 0), (4, 10, 1)  
                \]  
                point3d\[\] rigExec:posePoints = \[  
                    (4, 10, 0), (8, 10, 0), (4, 11, 0), (4, 10, 1)  
                \]  
                uniform token rigExec:framePolicy = "orthogonal"  
                rel rigExec:parent = \</ArmAsset/Rig/Controls/ShoulderFK\>  
                double rigExec:twist = 0  
            }  
  
            def RigExecControl "WristFK" (  
                prepend apiSchemas = \["RigExecPointTransformAPI", "RigExecControlAPI"\]

            )  
            {  
                point3d\[\] rigExec:restPoints = \[  
                    (8, 10, 0), (10, 10, 0), (8, 11, 0), (8, 10, 1)  
                \]  
                point3d\[\] rigExec:posePoints = \[  
                    (8, 10, 0), (10, 10, 0), (8, 11, 0), (8, 10, 1)  
                \]  
                uniform token rigExec:framePolicy = "orthogonal"  
                rel rigExec:parent = \</ArmAsset/Rig/Controls/ElbowFK\>  
                double rigExec:twist = 0  
            }  
  
            def RigExecControl "HandIK" (  
                prepend apiSchemas = \["RigExecPointTransformAPI", "RigExecControlAPI"\]  
            )  
            {  
                point3d\[\] rigExec:restPoints = \[  
                    (8, 10, 0), (10, 10, 0), (8, 11, 0), (8, 10, 1)  
                \]  
                point3d\[\] rigExec:posePoints = \[  
                    (8, 10, 0), (10, 10, 0), (8, 11, 0), (8, 10, 1)

                \]  
                uniform token rigExec:framePolicy = "orthogonal"  
                float inputs:stretch = 1  
                float inputs:softness = 0.15  
            }  
  
            def RigExecControl "ElbowPole" (  
                prepend apiSchemas = \["RigExecPointTransformAPI", "RigExecControlAPI"\]  
            )  
            {  
                point3d\[\] rigExec:restPoints = \[  
                    (4, 10, -4), (5, 10, -4), (4, 11, -4), (4, 10, -3)  
                \]  
                point3d\[\] rigExec:posePoints = \[  
                    (4, 10, -4), (5, 10, -4), (4, 11, -4), (4, 10, -3)  
                \]  
                uniform token rigExec:framePolicy = "orthogonal"  
            }  
        }  
  
        def Scope "Solvers"  
        {

            def RigExecFkChain "FK"  
            {  
                rel rigExec:controls = \[  
                    \</ArmAsset/Rig/Controls/ShoulderFK\>,  
                    \</ArmAsset/Rig/Controls/ElbowFK\>,  
                    \</ArmAsset/Rig/Controls/WristFK\>  
                \]  
            }  
  
            def RigExecTwoBoneIk "IK"  
            {  
                rel rigExec:rootControl = \</ArmAsset/Rig/Controls/ShoulderFK\>  
                rel rigExec:effectorControl = \</ArmAsset/Rig/Controls/HandIK\>  
                rel rigExec:poleControl = \</ArmAsset/Rig/Controls/ElbowPole\>  
                double rigExec:upperLength = 4  
                double rigExec:lowerLength = 4  
                uniform token rigExec:stretchPolicy = "uniformSegments"  
                uniform token rigExec:unreachablePolicy = "clampWithSoftness"  
                double rigExec:preferredBendRadians = -0.35  
            }  
  
            def RigExecBlendPointFrames "IKFKBlend"

            {  
                rel rigExec:inputA = \</ArmAsset/Rig/Solvers/FK\>  
                rel rigExec:inputB = \</ArmAsset/Rig/Solvers/IK\>  
                float inputs:weight = 0  
                uniform token rigExec:rotationBlend = "shortestArc"  
                uniform token rigExec:scaleBlend = "log"  
            }  
  
            def RigExecTwistDistribution "ForearmTwist"  
            {  
                rel rigExec:start = \</ArmAsset/Rig/Joints/Elbow\>  
                rel rigExec:end = \</ArmAsset/Rig/Joints/Wrist\>  
                int rigExec:count = 5  
                float\[\] rigExec:weights = \[0, 0.2, 0.5, 0.8, 1\]  
                uniform token rigExec:distribution = "minimumEnergy"  
            }  
  
            def RigExecRibbon "ArmRibbon"  
            {  
                rel rigExec:driverCurve = \</ArmAsset/Geom/RibbonDriver\>  
                rel rigExec:startFrame = \</ArmAsset/Rig/Joints/Shoulder\>  
                rel rigExec:endFrame = \</ArmAsset/Rig/Joints/Wrist\>

                int rigExec:sampleCount = 5  
                uniform token rigExec:parameterization = "arcLength"  
                uniform token rigExec:frameTransport = "rotationMinimizing"  
                rel rigExec:twistFrames = \</ArmAsset/Rig/Solvers/ForearmTwist\>  
            }  
        }  
  
        def Scope "Joints"  
        {  
            def RigExecJoint "Shoulder" (  
                prepend apiSchemas = \["RigExecPointTransformAPI"\]  
            )  
            {  
                point3d\[\] rigExec:restPoints = \[  
                    (0, 10, 0), (4, 10, 0), (0, 11, 0), (0, 10, 1)  
                \]  
                rel rigExec:frameSource = \</ArmAsset/Rig/Solvers/IKFKBlend\>  
                int rigExec:sourceElement = 0  
            }  
            def RigExecJoint "Elbow" (  
                prepend apiSchemas = \["RigExecPointTransformAPI"\]  
            )

            {  
                point3d\[\] rigExec:restPoints = \[  
                    (4, 10, 0), (8, 10, 0), (4, 11, 0), (4, 10, 1)  
                \]  
                rel rigExec:frameSource = \</ArmAsset/Rig/Solvers/IKFKBlend\>  
                int rigExec:sourceElement = 1  
                rel rigExec:parent = \</ArmAsset/Rig/Joints/Shoulder\>  
            }  
            def RigExecJoint "Wrist" (  
                prepend apiSchemas = \["RigExecPointTransformAPI"\]  
            )  
            {  
                point3d\[\] rigExec:restPoints = \[  
                    (8, 10, 0), (10, 10, 0), (8, 11, 0), (8, 10, 1)  
                \]  
                rel rigExec:frameSource = \</ArmAsset/Rig/Solvers/IKFKBlend\>  
                int rigExec:sourceElement = 2  
                rel rigExec:parent = \</ArmAsset/Rig/Joints/Elbow\>  
            }  
        }  
  
        def Scope "Weights"

        {  
            def RigExecStaticWeight "ShoulderPoints"  
            {  
                rel rigExec:weightTarget = \</ArmAsset/Geom/ArmBody.points\>  
                uniform token rigExec:representation = "dense"  
                uniform float\[\] rigExec:values = \[1, 0.1, 0, 1\]  
                uniform float rigExec:defaultWeight = 0  
                uniform token rigExec:rangePolicy = "strict"  
            }  
  
            def RigExecStaticWeight "ElbowPaint"  
            {  
                rel rigExec:weightTarget = \</ArmAsset/Geom/ArmBody.points\>  
                uniform token rigExec:representation = "dense"  
                uniform float\[\] rigExec:values = \[0, 0.8, 0.3, 0\]  
                uniform float rigExec:defaultWeight = 0  
                uniform token rigExec:rangePolicy = "strict"  
            }  
  
            def RigExecDynamicWeight "ElbowDriven"  
            {  
                rel rigExec:weightTarget = \</ArmAsset/Geom/ArmBody.points\>

                rel rigExec:baseWeight = \</ArmAsset/Rig/Weights/ElbowPaint\>  
                uniform token rigExec:representation = "dense"  
                uniform token rigExec:operation = "multiply"  
                float inputs:driver = 1  
                float inputs:scale = 1  
                float inputs:bias = 0  
                uniform token rigExec:rangePolicy = "strict"  
            }  
  
            def RigExecStaticWeight "WristPoints"  
            {  
                rel rigExec:weightTarget = \</ArmAsset/Geom/ArmBody.points\>  
                uniform token rigExec:representation = "sparse"  
                uniform int\[\] rigExec:indices = \[1, 2\]  
                uniform float\[\] rigExec:values = \[0.1, 0.7\]  
                uniform float rigExec:defaultWeight = 0  
                uniform token rigExec:rangePolicy = "strict"  
            }  
  
            def RigExecStaticWeight "BicepMask"  
            {  
                rel rigExec:weightTarget = \</ArmAsset/Geom/ArmBody.points\>

                uniform token rigExec:representation = "dense"  
                uniform float\[\] rigExec:values = \[0, 1, 1, 0\]  
                uniform float rigExec:defaultWeight = 0  
                uniform token rigExec:rangePolicy = "strict"  
            }  
        }  
  
        def Scope "BlendInputs"  
        {  
            def RigExecBlendInput "BicepFlex"  
            {  
                float inputs:weight = 0.35  
                uniform token rigExec:targetSpace = "restLocal"  
                rel rigExec:samples = \</ArmAsset/Rig/BlendInputs/BicepFlex/Full\>  
  
                def RigExecBlendSample "Full"  
                {  
                    float rigExec:activation = 1  
                    rel rigExec:pointDeltas = \</ArmAsset/Targets/BicepFlexTarget.rigExec:delta\>  
                }  
            }  
        }

  
        def Scope "Movers"  
        {  
            reorder nameChildren = \["Pose", "Geometry"\]  
  
            def Scope "Pose"  
            {  
                def RigExecAimConstraint "WristAim" (  
                    prepend apiSchemas = \["RigExecMoverAPI"\]  
                )  
                {  
                    rel rigExec:moves = \</ArmAsset/Rig/Joints/Wrist\>  
                    rel rigExec:aimTarget = \</ArmAsset/Rig/Controls/ElbowPole\>  
                    float inputs:weight = 0.2  
                    uniform token rigExec:aimAxis = "x"  
                    uniform token rigExec:upPolicy = "preserveInputUp"  
                    uniform token\[\] rigExec:preserve = \["origin", "scale"\]  
  
                    def RigExecFloatMathMover "ClampIKFKWeight" (  
                        prepend apiSchemas = \["RigExecMoverAPI"\]  
                    )  
                    {

                        rel rigExec:moves = \</ArmAsset/Rig/Solvers/IKFKBlend.inputs:weight\>  
                        uniform token rigExec:operation = "clamp"  
                        float inputs:min = 0  
                        float inputs:max = 1  
                    }  
                }  
            }  
  
            def Scope "Geometry"  
            {  
                def RigExecPostMover "RecomputeNormals" (  
                    prepend apiSchemas = \["RigExecMoverAPI"\]  
                )  
                {  
                    rel rigExec:moves = \</ArmAsset/Geom/ArmBody\>  
                    uniform token rigExec:operation = "recomputeNormals"  
  
                    def RigExecPostMover "VolumeCorrect" (  
                        prepend apiSchemas = \["RigExecMoverAPI"\]  
                    )  
                    {  
                        rel rigExec:moves = \</ArmAsset/Geom/ArmBody\>

                        uniform token rigExec:operation = "volumeCorrect"  
                        float inputs:volumeStrength = 0.25  
  
                        def RigExecCurveMover "RibbonWrap" (  
                            prepend apiSchemas = \["RigExecMoverAPI"\]  
                        )  
                        {  
                            rel rigExec:moves = \</ArmAsset/Geom/ArmBody\>  
                            rel rigExec:driverCurve = \</ArmAsset/Geom/RibbonDriver\>  
                            rel rigExec:driverFrames = \</ArmAsset/Rig/Solvers/ArmRibbon\>  
                            rel rigExec:bindCoordinates =  
                                \</ArmAsset/Geom/ArmBody.primvars:rigExec:ribbonST\>  
                            uniform token rigExec:mode = "ribbon"  
  
                            def RigExecMatrixMover "ShoulderMatrix" (  
                                prepend apiSchemas = \["RigExecMoverAPI"\]  
                            )  
                            {  
                                rel rigExec:moves = \</ArmAsset/Geom/ArmBody.points\>  
                                rel rigExec:transform = \</ArmAsset/Rig/Joints/Shoulder\>  
                                rel rigExec:weightObject =  
                                    \</ArmAsset/Rig/Weights/ShoulderPoints\>

                                uniform token rigExec:transformReadPhase = "final"  
  
                                def RigExecMatrixMover "ElbowMatrix" (  
                                    prepend apiSchemas = \["RigExecMoverAPI"\]  
                                )  
                                {  
                                    rel rigExec:moves = \</ArmAsset/Geom/ArmBody.points\>  
                                    rel rigExec:transform = \</ArmAsset/Rig/Joints/Elbow\>  
                                    rel rigExec:weightObject =  
                                        \</ArmAsset/Rig/Weights/ElbowDriven\>  
                                    uniform token rigExec:transformReadPhase = "final"  
  
                                    def RigExecMatrixMover "WristMatrix" (  
                                        prepend apiSchemas = \["RigExecMoverAPI"\]  
                                    )  
                                    {  
                                        rel rigExec:moves = \</ArmAsset/Geom/ArmBody.points\>  
                                        rel rigExec:transform = \</ArmAsset/Rig/Joints/Wrist\>  
                                        rel rigExec:weightObject =  
                                            \</ArmAsset/Rig/Weights/WristPoints\>  
                                        uniform token rigExec:transformReadPhase = "final"  
  

                                        def RigExecBlendShapeMover "BicepFlex" (  
                                            prepend apiSchemas = \["RigExecMoverAPI"\]  
                                        )  
                                        {  
                                            rel rigExec:moves = \</ArmAsset/Geom/ArmBody.points\>  
                                            rel rigExec:blendInputs =  
                                                \</ArmAsset/Rig/BlendInputs/BicepFlex\>  
                                            rel rigExec:weightObject =  
                                                \</ArmAsset/Rig/Weights/BicepMask\>  
                                        }  
                                    }  
                                }  
                            }  
                        }  
                    }  
                }  
  
                def RigExecCurveMover "GuideFromRibbon" (  
                    prepend apiSchemas = \["RigExecMoverAPI"\]  
                )  
                {  
                    rel rigExec:moves = \</ArmAsset/Geom/RibbonGuides\>

                    rel rigExec:driverFrames = \</ArmAsset/Rig/Solvers/ArmRibbon\>  
                    uniform token rigExec:mode = "emitGuidePoints"  
                }  
            }  
        }  
    }  
  
    def Scope "Geom"  
    {  
        def BasisCurves "RibbonDriver" (  
            prepend apiSchemas = \["RigExecGeometryDriverAPI"\]  
        )  
        {  
            uniform token type = "cubic"  
            uniform token basis = "bspline"  
            uniform token wrap = "nonperiodic"  
            int\[\] curveVertexCounts = \[4\]  
            point3f\[\] points = \[  
                (0,10,0), (2.7,10,0), (5.3,10,0), (8,10,0)  
            \]  
            uniform token\[\] rigExec:sourcePrimvars = \["points", "widths"\]  
            float\[\] widths = \[0.1, 0.1, 0.1, 0.1\] (

                interpolation = "vertex"  
            )  
        }  
  
        def BasisCurves "RibbonGuides" (  
            prepend apiSchemas = \["RigExecGeometryDriverAPI", "RigExecMoveTargetAPI"\]  
        )  
        {  
            uniform token type = "linear"  
            uniform token wrap = "nonperiodic"  
            int\[\] curveVertexCounts = \[5\]  
            point3f\[\] points = \[  
                (0,10,0), (2,10,0), (4,10,0), (6,10,0), (8,10,0)  
            \]  
            uniform token\[\] rigExec:sourcePrimvars = \["points", "widths"\]  
            float\[\] widths (  
                interpolation = "vertex"  
            )  
            uniform token\[\] rigExec:publishedPrimvars = \["points", "widths"\]  
        }  
  
        def Mesh "ArmBody" (

            prepend apiSchemas = \["RigExecGeometryDriverAPI", "RigExecMoveTargetAPI"\]  
        )  
        {  
            int\[\] faceVertexCounts = \[4\]  
            int\[\] faceVertexIndices = \[0, 1, 2, 3\]  
            point3f\[\] points = \[  
                (0,9.5,0), (8,9.5,0), (8,10.5,0), (0,10.5,0)  
            \]  
            uniform token\[\] rigExec:sourcePrimvars = \[  
                "points", "rigExec:ribbonST"  
            \]  
            float2\[\] primvars:rigExec:ribbonST = \[  
                (0,0), (1,0), (1,1), (0,1)  
            \] (interpolation = "vertex")  
            uniform token\[\] rigExec:publishedPrimvars = \["points", "normals"\]  
        }  
    }  
  
    def Scope "Targets"  
    {  
        def Scope "BicepFlexTarget"  
        {

            point3f\[\] rigExec:delta = \[  
                (0,0,0), (0,0.25,-0.15), (0,0.25,0.15), (0,0,0)  
            \]  
        }  
    }  
  
    variantSet "rigComplexity" = {  
        "preview" {  
            over "Rig" {  
                over "Movers" {  
                    over "Geometry" {  
                        over "RecomputeNormals" {  
                            over "VolumeCorrect" {  
                                bool inputs:enabled = false  
                                over "RibbonWrap" {  
                                    bool inputs:enabled = false  
                                }  
                            }  
                        }  
                        over "GuideFromRibbon" {  
                            bool inputs:enabled = false  
                        }

                    }  
                }  
            }  
            over "Geom" {  
                over "RibbonGuides" {  
                    float\[\] widths = \[0, 0, 0, 0, 0\] (  
                        interpolation = "vertex"  
                    )  
                }  
            }  
        }  
        "animation" {  
            over "Rig" {  
                over "Movers" {  
                    over "Geometry" {  
                        over "RecomputeNormals" {  
                            over "VolumeCorrect" {  
                                bool inputs:enabled = false  
                            }  
                        }  
                    }  
                }

            }  
            over "Geom" {  
                over "RibbonGuides" {  
                    float\[\] widths = \[0.06, 0.06, 0.06, 0.06, 0.06\]  
                }  
            }  
        }  
        "film" {  
            over "Geom" {  
                over "RibbonGuides" {  
                    float\[\] widths = \[0.06, 0.06, 0.06, 0.06, 0.06\]  
                }  
            }  
        }  
    }  
}

The parent-authored child reorder makes Pose precede Geometry; inside geometry, post-order gives BicepFlex → WristMatrix → ElbowMatrix → ShoulderMatrix → RibbonWrap → VolumeCorrect → RecomputeNormals. Each matrix mover consumes one transform and one total weight field; no operation owns a transform list. Normals always publish, while preview keeps RibbonGuides targetable and hides it with zero widths. ClampIKFKWeight is a typed property mover before ancestor WristAim. Selecting these variants starts a new epoch; direct shape-preserving enable or weight-value edits need not. The compiler generates scalar views for all twist/ribbon frames; no stack or authored stage links exist.

## 4.6 Animation/reference example: ArmShotAnim.usda

This shot layer references the reusable asset, selects an animation LOD, and authors only animator values using stock OpenUSD 26.08 value sources. point3d\[\] pose arrays use sparse timeSamples; OpenUSD resolves them with its built-in element-wise linear interpolation when bracketing arrays have equal length, or held interpolation when the explicit stage policy/type requires it. Eligible scalar half, float, double, and timecode animation uses sparse .spline knots; a timed UsdAttribute::Get() or UsdAttributeQuery::Get() resolves the opinion and evaluates it through Ts. Ordinary timeSamples are not converted to Ts, and Ts splines do not support vectors, matrices, quaternions, or arrays.

RigExec authors no RigExecAnimationSource, curve schema, interpolation mode, or sampled bridge. OpenExec inputs consume the standard resolved attribute value at the requested explicit UsdTimeCode, including PreTime; Phase 0 goldens compare that result directly with UsdAttribute::Get(time). A no-argument Get() is not used for animated evaluation because it requests the default time.

\#usda 1.0  
(  
    defaultPrim = "Shot"  
    startTimeCode = 1001  
    endTimeCode = 1048  
    timeCodesPerSecond = 24  
)  
  
def Xform "Shot"  
{  
    def Xform "HeroArm" (  
        prepend references = @./ArmRig.usda@\</ArmAsset\>  
        variants = {  
            string rigComplexity = "animation"  
        }  
    )  
    {  
        over "Rig"  
        {  
            over "Controls"  
            {  
                over "HandIK"

                {  
                    point3d\[\] rigExec:posePoints.timeSamples = {  
                        1001: \[(8,10,0), (10,10,0), (8,11,0), (8,10,1)\],  
                        1024: \[(6,13,2), (7.8,13.5,2), (5.8,14,2), (6,13,3)\],  
                        1048: \[(9,11,-1), (11,11,-1), (9,12,-1), (9,11,0)\]  
                    }  
                }  
                over "ElbowPole"  
                {  
                    point3d\[\] rigExec:posePoints.timeSamples = {  
                        1001: \[(4,10,-4), (5,10,-4), (4,11,-4), (4,10,-3)\],  
                        1048: \[(5,12,-5), (6,12,-5), (5,13,-5), (5,12,-4)\]  
                    }  
                }  
            }  
            over "Solvers"  
            {  
                over "IKFKBlend"  
                {  
                    float inputs:weight.spline = {  
                        bezier,  
                        1001: 0; post held,

                        1012: 0; post held,  
                        1013: 1; post held,  
                        1048: 1; post held,  
                    }  
                }  
            }  
            over "Weights"  
            {  
                over "ElbowDriven"  
                {  
                    float inputs:driver.spline = {  
                        bezier,  
                        1001: 0.5; pre (0, 0); post curve (8, 0),  
                        1024: 1; pre (8, 0); post curve (8, 0),  
                        1048: 0.75; pre (8, 0); post curve (0, 0)  
                    }  
                }  
            }  
        }  
    }  
}

## 4.7 Schema decision alternatives

**Solver wiring: generic node/port schema versus domain schemas.** A generic RigExecNode with token-typed ports is flexible but weakens USD validation, discoverability, and plugin loading. Concrete typed schemas are chosen for production behavior; a generic experimental node may exist for prototyping, but published assets must resolve to typed schemas.

**Frames in one array versus four attributes.** Four attributes make single-point edits smaller but can be observed in inconsistent combinations during separate edits and create more graph inputs. One validated four-point array is chosen as the atomic authored value. Any authoring client replaces the complete array as one value opinion.

**Relationships versus multi-connections.** Multi-connection attributes would look graph-like, but current OpenExec supports only one valid connection for computed value flow. Relationships plus named computation provider resolution are chosen for variable-arity frame/geometry inputs. The schema can migrate to richer upstream connection semantics later without changing solver results.

**Explicit list versus composed hierarchy.** A stack list is locally obvious but duplicates namespace intent. The chosen post-order moves hierarchy works through composition; same-target movers are nested or fully covered by a parent child-reorder opinion. The compiler records/diffs ordinals and chain digests.

# 5\. Point-based transform system

**Status:** **BUILD**. OpenExec can carry the custom type and cache it (**EXISTS**), but defines no point-transform semantics.

## 5.1 Canonical value and reference frame

A transform is represented by four posed landmark points:

P=(po,px,py,pz),piℝ3

and four immutable reference/rest points:

Q=(qo,qx,qy,qz).

q\_o is origin; q\_x is normally the visible tip; q\_y and q\_z are up/side landmarks. The points encode both joint ends and the transformed basis. Q may be a unit frame or carry real bone length.

Define:

BQ=\[qx−qoqy−qoqz−qo\],BP=\[px−popy−popz−po\].

If det(B\_Q) \!= 0, the affine map from reference space to posed space is:

L=BPBQ−1,t=po−Lqo,f(q)=Lq+t.

The homogeneous transform in the mathematical column-vector convention is:

M=\[L t 0 1 \].

Then f(q\_i)=p\_i; when det(B\_P) \!= 0, the inverse is:

qi=L−1(pi−t).

These equations define round-trip behavior. A singular posed frame retains its points/forward map and a degenerate status but has no inverse. GfMatrix4d conformance tests transform each q\_i and compare p\_i, avoiding row/column assumptions.

The runtime scalar is:

struct RigExecPointFrame {  
    std::array\<GfVec3d, 4\> points;  // O, X/tip, Y/up, Z/side  
    uint32\_t flags;                 // valid, degenerate, reflected, affine  
};

Reference points are explicit inputs, keeping callbacks pure and import/export unambiguous.

## 5.2 Reconstruction policies

rigExec:framePolicy selects one deterministic policy:

1.  affine — use all four points directly. Scale, reflection, and shear are retained. Reject only a singular reference frame; flag a singular posed frame but keep the points extractable.
2.  orthogonal — chosen default for controls and joints. Treat p\_x as aim, p\_y as up/twist, and p\_z as side-scale/handedness; remove shear.
3.  axial — orthogonal frame with scale only along the aim axis. Useful for ordinary bones.
4.  rigid — orthogonal unit scale. Useful for controls whose landmark distance is display-only.

For the orthogonal policy, let:

a=px−po,ex=a/‖a‖.

Project the up landmark:

u=(py−po)−ex(ex·(py−po)),ey=u/‖u‖,ez=exey,ey=ezex.

The final re-cross removes numerical drift. Unwrapped twist θ rotates the transverse axes:

ey=cosey+sinez,ez=−siney+cosez.

Let the reference handle lengths be l\_x=||q\_x-q\_o||, l\_y=||q\_y-q\_o||, and l\_z=||q\_z-q\_o||. The scale magnitudes are:

sx=‖a‖/lx,sy=‖u‖/ly,dz=(pz−po)·ez,sz=|dz|/lz,z=sign(dz).

σ\_z selects permitted handedness; zero follows the authored reflection policy. rigid sets all magnitudes to one, axial sets s\_y=s\_z=1, and orthogonal uses all three. Reconstructed landmarks are p\_o, p\_o+l\_xs\_xe\_x, p\_o+l\_ys\_ye\_y^θ, and p\_o+l\_zσ\_zs\_ze\_z^θ; import pins any negative sign to reflectionAxis.

## 5.3 Degeneracy, twist, scale, and shear policy

Use scale-relative tolerances, not a hard world-unit epsilon:

ϵ=10−10max(1,lx,ly,lz)

in double-precision authoring/evaluation math. If ||a|| \< ε, the frame is invalid for orientation. If ||u|| \< ε, aim is valid but twist is underdetermined.

Fallbacks must be explicit inputs because OpenExec callbacks cannot depend on hidden previous-frame state:

1.  Project the parent’s computed up axis onto the new aim plane.
2.  If degenerate, project the authored rest up axis.
3.  If still degenerate, select the world basis axis least parallel to e\_x and project it.

The chosen fallback is deterministic at any isolated frame and produces the same result on every run.

**Continuity alternatives.** Hidden previous-frame orientation is smooth but order-dependent and rejected. The chosen parent/rest/world fallback is reproducible. Any future continuity state must be an explicit declared input and is not part of the v0.1 data contract.

Controls/joints default to orthogonal, no shear, positive transverse scale, and an authored mirror reflectionAxis; auxiliary frames may be affine. Validation rejects arbitrary control shear.

**Alternative A — three points plus a quaternion/twist value.** Compact and familiar, but not a point-only affine representation and cannot encode shear or three independent scales without extra channels.

**Alternative B — four landmarks. Chosen.** It maps exactly to any nonsingular affine transform, exposes all frame directions as points, and makes import/export algebraic. The extra point and validation cost are small relative to the gain in uniformity.

## 5.4 Matrix/SRT interop

For export or a conventional plugin, points decode to (L,t) as above. For SRT inspection/import, compute the singular value decomposition:

L=UΣVT,δ=det(UVT).

Select singular index k whose hemisphere-canonicalized right-singular direction v\_k is most aligned with the authored, clip-pinned reflectionAxis; when δ=+1, k is immaterial. Define:

Dii=1 (i≠k),Dkk=δ,

R=UDVT,H=RTL.

R is proper; diag(H) gives scale and normalized off-diagonals give shear. The pinned policy selects any negative principal stretch while preserving RH=L. If import lacks reflectionAxis, choose the largest-magnitude local scale axis at the first sample and pin it for the clip.

The parameter tuple is:

TransformParams = (translation t,  
                   unit quaternion q derived from R,  
                   scale s = diag(H),  
                   shear h = offDiagonal(H),  
                   reflectionAxis)

ParamsToMatrix builds L=R(q)H(s,h)/t; MatrixToPoints applies it to the rest landmarks; PointsToParams is PointsToMatrix → SVD. Quaternions are adapters, never canonical authoring.

At structural import, the USD adapter reads the full UsdGeomXformable local transform, normalizes declared inputs, and converts to points; callbacks never query the stage. Current execGeom covers limited xformOp:transform behavior, not arbitrary op/reset stacks ([source](https://github.com/PixarAnimationStudios/OpenUSD/blob/v26.08/pxr/exec/execGeom/xformable.cpp)), so this adapter is **BUILD**.

## 5.5 Deferred inverse parameter solve

Inverse solving is documented only as a future pure-computation boundary; it is not a v0.1 phase deliverable, performance gate, UI, or manipulation feature. Let authored avars be vector a, forward rig evaluation be F(a), selected landmark projection be S, and desired points be d. An analytic controller inverse would be preferred. A future numeric fallback could solve:

minΔa‖W(SF(a+Δa)−d)‖22+λ‖CΔa‖22+μ‖Δa‖22

subject to channel limits. W weights selected points/axes, C regularizes semantic channels or distance from a declared reference pose, and μ is Levenberg damping. The solver uses analytic Jacobians for built-in IK/frame kernels and finite differences only for explicitly declared prototype nodes. Termination is (point error \< 10^-5 × characterScale) or (relative improvement \< 10^-6) or the profile’s fixed iteration limit.

Desired landmarks enter through declared typed inputs or low-level evaluation overrides. Forward/inverse callbacks remain pure. Hit testing, gesture state, commits, and undo are deferred consumers.

## 5.6 Native USD time evaluation and interpolation

OpenUSD 26.08 is the only temporal-interpolation authority:

  - TsSpline .spline opinions provide sparse curve knots for scalar half, float, double, and timecode attributes. Timed UsdAttribute::Get(time) and UsdAttributeQuery::Get(time) resolve the authored source and evaluate its Bezier/Hermite, knot, looping, and extrapolation behavior through Ts.
  - RigExec-authored animation uses .spline for every eligible scalar channel. Ordinary sparse timeSamples use stock USD held/linear value resolution and are required for non-scalar tuples, matrices, quaternions, arrays, and non-interpolatable values. Preexisting standard-USD scalar timeSamples remain valid inputs and are resolved unchanged; RigExec does not convert them to splines.
  - UsdStage interpolation type is stage-local state, not authored scene data. Every evaluation/build profile sets it explicitly— linear by default, with a separately named held profile when required—and includes it in the generation key, pack manifest, and semantic hash. It affects ordinary timeSamples, not spline curve semantics.
  - If a site or value-clip manifest presents both forms for one attribute, standard USD precedence applies; RigExec neither merges nor reinterprets the sources.
  - Timed SdfValueBlock intervals can make Get(time) return no value. BlockAnimation() suppresses weaker animation but may expose a weaker/default/fallback value under standard resolution. RigExec preserves exactly the timed Get outcome—typed value or no value—and never invents a preceding sample or custom hold.

For rigExec:posePoints, USD first resolves the complete four-element point3d\[\] value at the requested time, then RigExec validates cardinality/finiteness and reconstructs the frame. Degenerate resolved landmarks follow §5.3’s deterministic policy. Imported standard USD transform properties remain in their native authored representation. There is no RigExec track-level point/transform interpolation mode, no custom animation schema, and no curve-sampling bridge.

## 5.7 Universal point extraction invariant

OpenExec’s public request API addresses published computations on prims or attributes; schedules are opaque, and it has no public “read any anonymous VDF node output” handle. RigExec meets the extraction requirement with a compiler invariant:

1.  Every semantically meaningful transform stage is a scene-addressable RigExec prim or attribute provider.
2.  RigExecTransformComputation(...) publishes paired point-frame/matrix results for every base, post-mover, and final transform revision; packed arrays are intermediates.
3.  A solver with multiple semantically addressable transforms creates scalar scene-addressable providers for them; it may not make a client depend on an opaque VDF element or private node identity.
4.  Every published result has a stable RigExecValueAddress and optional RigExecTapAPI alias.
5.  The validator fails a rig when a transform-semantic output lacks the point computation, reference points, or type registration.

OpenExec’s existing ExecSystem::Diagnostics::GraphNetwork() can produce a DOT topology export. Value inspection is the RigExec address/tap layer. Production features never depend on private node identities or a modified/instrumented OpenUSD build.

# 6\. Execution model

## 6.1 Compilation, scheduling, and evaluation

**Status:** OpenExec phases **EXISTS**; RigExec compiler policy and request profiles **BUILD**.

RigExec preserves OpenExec’s phase boundaries:

1.  Compilation: composed scene description plus registered computations become the execution network. RigExec validates and canonicalizes moves targets, performs the fixed-policy post-order walk, and lowers each target’s writers into base/intermediate/final providers before OpenExec compiles them. Structural changes incrementally rebuild affected areas.
2.  Scheduling: a batch of requested value keys becomes a reusable request schedule. PrepareRequest() front-loads this work.
3.  Evaluation: invalid requested values are pulled, callbacks run, and valid results are reused from caches.

Stable pose, proxyGeometry, fullGeometry, and renderMotion profiles are prepared in batches; the OpenExec tutorial explicitly recommends batched ExecUsdRequest use ([system design](https://openusd.org/dev/api/page__execution__system__design.html), [request tutorial](https://openusd.org/dev/api/md_pxr_exec_exec_usd_docs_tutorial1_computing_values.html)).

The mover compiler stores, per target, the traversal policy, mover paths, post-order ordinals, canonical targets, operation kinds, and chain hash. A mover that reads another moved target binds the latest version preceding its ordinal. The compiler rejects self-final reads, dependency cycles, cross-rig writes, structural property targets, duplicate targets after canonicalization, and incompatible multi-target types. A structural transaction is atomic: the previous epoch remains publishable until all affected chains, taps, prepared requests, and Hydra reverse bindings compile successfully.

ExecUsdCacheView cannot outlive its system or request. RigExecSnapshot therefore copies small values and retains RigExec-owned immutable geometry buffers before crossing threads.

## 6.2 Character and region partitioning

**Status:** partition policy **BUILD** on the existing request-pruned network. OpenExec does not mandate one graph or system per character.

Each RigExecRig declares one character partition. Optional regions—body, face, cloth, hair, proxy/render geometry—form dependency-constrained subpartitions. Validation forbids a direct dependency cycle across character roots. A shot-level coordinator implements deliberate cross-character constraints as an explicit acyclic multipass:

pass 0: independent character poses  
pass 1: cross-character contact targets  
pass 2: affected character correction and deformation

LibEE’s [six-character example](https://www.oreilly.com/library/view/multithreading-for-visual/9781482243567/chapter-33.html) and Presto’s [per-model strategy](https://www.multithreadingandvfx.org/course_notes/2015/presto_threading.pdf) are precedent, not proof of one required graph shape. Native USD uses one serialized ExecUsdSystem per stage; standalone may use a system per instance under a global budget. Packed frame ranges cross solver boundaries, while semantically addressable intermediates remain published.

## 6.3 Caching, invalidation, and time

**Status:** computation caches/time interval invalidation **EXISTS**; publication residency, prediction, and eviction **BUILD**.

OpenExec owns value validity. ChangeTime() re-resolves time-dependent inputs, detects which actually changed, invalidates dependents, and notifies interested requests. In the USD deployment, stock OpenUSD 26.08 evaluates eligible authored scalar splines through Ts and ordinary time samples through USD’s held/linear interpolation before computations consume the resolved value. The runtime contains an EfPageCacheStorage\<EfTime\> for time-varying values ([runtime source](https://github.com/PixarAnimationStudios/OpenUSD/blob/v26.08/pxr/exec/exec/runtime.cpp), [cache API](https://openusd.org/dev/api/class_ef_page_cache_storage.html)). RigExec replaces neither USD value resolution nor the OpenExec cache.

RigExecFrameResidencyCache stores published immutable snapshots under (definition hash, instance, dependency-version digest, RigExecTimeKey, stage interpolation type, profile, complexity, override digest). RigExecTimeKey is the full numeric/PreTime identity; equal numeric payloads with different tags never alias. Affected-time intervals evict only intersecting entries; a separate global revision rejects stale publication. A segmented LRU protects explicitly requested frame/render samples and evicts film geometry before pose snapshots. Phase 0 calibrates provisional limits of 2 GiB per desktop hero, 25% of physical memory globally, and 256 MiB mobile; no cache repeats materialization, while an unbounded cache is unsafe.

Concurrent invalidation callbacks translate request indices/time intervals and only enqueue coalesced work; host, publication, and Hydra work runs on owner threads. Because ordinary Compute() renews request interest in v26.08 but ComputeWithOverrides() does not, the low-level override path retains an independently prepared baseline subscription; a pinned conformance regression guards the difference even though interactive consumers are deferred.

Structural edits invalidate compiled nodes/schedules and changed-definition snapshots. Reparenting, moves, child order, operation/read kind, activation, output manifests, and variant selection are structural; numeric parameters, weights, goals, and direct shape-preserving inputs:enabled edits invalidate values only. Old snapshots cannot publish into a new epoch.

Native USD serializes ChangeTime/Compute because concurrent calls are undocumented. Only standalone immutable snapshots may use isolated multi-time systems until a public context API exists.

## 6.4 Threading and work ownership

**Status:** VDF parallel evaluation **EXISTS**; application concurrency model **BUILD**.

|  |  |  |
| :-: | :-: | :-: |
| \*\*Thread/arena\*\* | \*\*Owns\*\* | \*\*Must not do\*\* |
| Stage/change owner | complete external USD edit batches and hand them to the coordinator | race mutation against evaluation or publish partial edits |
| Stage coordinator (one per ExecUsdSystem) | apply USD edit batches, notice epochs, ChangeTime, prepare/compute, generations | issue Hydra notices or mutate renderer state |
| VDF worker arena | pure computation callbacks and internal parallel tasks | read undeclared scene/app state |
| Snapshot publisher | atomic immutable-generation swap | evaluate the rig |
| Hydra notice owner | coalesced \\\_SendPrimsDirtied calls | wait on workers inside GetPrim() |
| Batch/render-sample scheduler | submits explicitly requested times/profiles under the global work budget | bypass the stage coordinator or invent sample times |

  

One stage gate serializes every USD mutation/notice epoch with prepare, time change, and compute; host integrations hand it complete edit batches. OpenExec’s executor parallelizes declared work underneath. A global governor prevents outer character jobs from spawning unconstrained inner pools and reserves render/publication capacity.

OpenExec exposes no public async/cancellation call. RigExec coalesces unstarted duplicate work and discards completed generations whose stage revision, full tagged RigExecTimeKey, stage interpolation type, profile, or binding epoch is stale. Explicit batch/render-sample work uses isolated standalone systems only where the source snapshot is immutable. CPU callbacks target 2 ms p95, and long custom callbacks fail validation.

## 6.5 Vectorization and kernel execution

OpenExec/VDF vectorization means values can contain multiple uniquely identifiable elements flowing over one connection, with masks/ranges and contiguous schedule access. It does not promise CPU SIMD, a GPU backend, or device residency. RigExec movers separately implement:

  - SoA point, weight, delta and primvar buffers;
  - 64-byte-aligned blocks and 256–2,048-point strip sizes selected by benchmark;
  - scalar reference kernels;
  - platform SIMD kernels (AVX2/AVX-512 where available, ARM NEON);
  - optional GPU kernels only after an entire compatible mover-chain segment can stay resident.

Transfer boundaries can erase kernel gains, so GPU eligibility is a mover-chain property, not a per-node badge.

## 6.6 Determinism and failure behavior

  - Composed child order and canonical mover/target addresses are part of the compiled asset hash; moves target-list order is not.
  - Every compiled matrix-mover hash includes the canonical mover and points-target paths, exact transform-provider/read-phase identity, weight-object path and descriptor, and post-order ordinal. Every structural weight-descriptor hash includes its canonical target, domain, representation, logical count, sorted sparse support, range policy, operation kind, and declared provider identities; current values and sparse default are value generations, not layout identity. Relationship target order is excluded.
  - Reductions use a fixed tree; no result depends on unordered-container iteration.
  - CPU reference math uses double for frame/solver state and float for bulk geometry unless a schema requests double.
  - Degenerate frames return points plus a status flag; they do not silently emit identity.
  - Cycles are schema validation errors. Current OpenExec cycle detection reports an error and affected results may be wrong until the cycle is removed, so RigExec never treats a cycle as a solver.
  - A disabled mover returns its preceding revision without error. A mover with a non-finite/non-affine matrix, descriptor mismatch, non-finite weight, or strict range violation fails that application atomically, returns the preceding revision with MoverFailed status, and never publishes a partial point update. NaN/Inf in a non-mover solver/provider instead poisons only its dependent branch, records the first provider address, and uses the configured last-valid-generation or error-guide publication policy.
  - A debug mode hashes outputs across repeated evaluation and worker counts to detect hidden state/races.

## 6.7 Execution granularity alternatives

**Alternative A — one computation per scalar channel/joint operation.** Maximum raw visibility, but excessive topology, scheduling overhead and weak vectorization.

**Alternative B — one callback for the entire character.** Low graph overhead, but destroys sparse invalidation, critical-path parallelism, reuse and intermediate extraction.

**Chosen hybrid.** Pack coherent hierarchy/geometry values across a solver or mover application; expose each semantically meaningful stage as a named computation. Same-target writers form explicit serial edges, while unrelated targets remain parallel. Benchmarks, not aesthetic node counts, set the lower boundary. This matches the authoring/evaluation separation documented by [LibEE 2](https://research.dreamworks.com/wp-content/uploads/2018/08/talk_libee2-Edited.pdf) and OpenExec’s compilation model.

# 7\. Mover pipeline: mesh and curve I/O

**Status:** **BUILD**. OpenExec transports/caches typed results; it does not ship a production geometry-mover library. This hierarchy-ordered pipeline is the vectorized geometry-operation facility required by R4.

## 7.1 Geometry value

struct RigExecPrimvarDescriptor {  
    TfToken name, valueType, role, interpolation, colorSpace;  
    int tupleWidth, elementSize;  
    bool indexed, timeVarying;  
    RigExecSharedBufferBase values;  
    RigExecSharedBuffer\<int\> indices; // empty when flat  
};  
  
struct RigExecGeometryPacket {  
    RigExecTopologyHandle topology;      // immutable for the epoch  
    RigExecSharedBuffer\<GfVec3f\> points;  
    RigExecPrimvarTable primvars;        // normals are full descriptors here  
    RigExecSpace space;  
    GfMatrix4d sourceLocalToRigCommon;  
    GfMatrix4d rigCommonToOutputLocal;  
    RigExecDirtyRanges dirtyPoints;  
    uint64\_t provenanceHash;  
};  
  
struct RigExecWeightDescriptor {  
    RigExecValueAddress target;           // canonical prim/property value  
    TfToken domain;                       // scalar, point, element, or plugin

    TfToken representation;               // constant, dense, or sparse  
    TfToken rangePolicy;  
    size\_t logicalCount;  
    RigExecSharedBuffer\<int\> indices;      // sorted structural sparse layout  
    uint64\_t descriptorHash;  
};  
  
struct RigExecWeightBindingRecord {  
    RigExecValueAddress baseWeight, driver, scale, bias;  
    uint64\_t bindingHash;                  // identities/types, never values  
};  
  
struct RigExecWeightPacket {  
    RigExecWeightDescriptor descriptor;  
    float defaultWeight;                  // constant broadcast or sparse fallback  
    RigExecSharedBuffer\<float\> values;    // 0, logicalCount, or indices.size()  
    RigExecDirtyRanges dirtyElements;  
};  
  
struct RigExecGeometryMoverResult {  
    RigExecGeometryPacket value;  
    RigExecSolveStatus status;            // success, disabled, or mover failure

};

RigExecTopologyHandle describes mesh face counts/indices and subdivision tags, curve counts/basis/wrap, or point-cloud cardinality. It is immutable within an epoch. A primvar descriptor preserves flat versus indexed values, indices, interpolation, role, element size, color space, exact type/tuple width, and variability; representation or output-set changes are structural.

RigExecWeightDescriptor is shape-only and frozen for the epoch, including the canonical sorted sparse-index layout but excluding numeric values and the sparse default. RigExecWeightPacket is the common immutable value returned by both static and dynamic weight prims. constant stores zero packet values and broadcasts the packet default; dense stores exactly logicalCount values and requires canonical default zero; sparse stores one packet value per descriptor index plus the packet default for all other elements. Logical elements are target elements, never tuple components. dirtyElements is conservative: a constant/default edit or any dynamic driver/scale/bias change dirties the full logical domain; a dense edit may carry known changed ranges; a sparse explicit-value-only edit may carry the ranges named by its fixed support. Internal ranges may reduce evaluation work, but Hydra still dirties the complete published points leaf.

Every generated mover application computes one immutable typed {value,status} result. Geometry, point-frame, float, Vec3f, and matrix-value applications use separately registered concrete result types with this layout; atomic bundles register their own concrete result. Its computeGeometry (or scalar-equivalent) view returns value, while computeStatus returns the status from that same result without rerunning the kernel. Disabled and failed applications therefore remain addressable/tappable even though their value view is the preceding revision. Downstream value chains consume only the value view; diagnostics and parity requests consume the paired status view.

Callbacks cannot enumerate arbitrary prim properties through today’s static input DSL. On RigExecGeometryDriverAPI, rigExec:sourcePrimvars may select only from a pre-registered v0.1 union (points, normals, widths, topology, and ribbon coordinates); it never adds a computation input. Static and dynamic weights enter through their own declared computeWeightPacket computations rather than by expanding this geometry manifest. A custom input vocabulary requires a plugin-owned concrete driver schema whose complete computeSourceGeometry registration declares every input before system construction. Undeclared primvars are rejected. “Arbitrary computed primvar” means a plugin-declared/catalogued output, not runtime namespace discovery.

Three spaces are explicit: **source local** is the authored driver prim’s local space; **rig common** is the RigExecRig root space used between native stages; and **output local** is the final Hydra prim’s local space. RigExecGeometryDriverAPI owns or targets one exact sourceLocalToRigCommon property and always applies it exactly once before the first mover, storing the matrix in RigExecGeometryPacket.sourceLocalToRigCommon. Matrix movers consume transforms that already map rig-common to rig-common and never conditionally apply a geometry-bind transform. Final points/extent are output-local; normals use the inverse transpose of the applicable linear map. Transform point frames are rig-common; Hydra derives a local-to-parent matrix against the evaluated parent.

All callbacks are pure: inputs are immutable; outputs are new or copy-on-write; pool reuse cannot change logical results.

## 7.2 Target-chain compilation

RigExecGeometryDriverAPI.computeSourceGeometry adapts authored Mesh, Points, or BasisCurves and maps its points into rig-common space once; RigExecMoveTargetAPI.computeGeometry maps the final points to output-local and publishes the target’s result. The source and final addresses are distinct so a mover can consume preceding geometry without reading its own result.

At structural compile, RigExec discovers each exact moves target, performs §4.2’s composed post-order traversal, and emits a pure target application for every (mover,target). The example arm lowers to:

ArmBody.computeSourceGeometry  
  → BicepFlex@ArmBody.computeGeometry  
  → WristMatrix@ArmBody.computeGeometry  
  → ElbowMatrix@ArmBody.computeGeometry  
  → ShoulderMatrix@ArmBody.computeGeometry  
  → RibbonWrap@ArmBody.computeGeometry  
  → VolumeCorrect@ArmBody.computeGeometry  
  → RecomputeNormals@ArmBody.computeGeometry  
  → ArmBody.computeGeometry

Each adapter receives the preceding immutable packet, declared inputs, and inputs:enabled; the target API alone converts the final rig-common points to output-local. §4.2 defines fan-out, cycles, and conflicts. Base, every post-mover revision, and final are cacheable/addressable. Fusion must materialize every public address through virtual results or an unfused profile.

## 7.3 Blend shapes

For dense targets:

pi′=pi+kk(wk)dk,i.

dk,i is a rest-local delta and α\_k the target/in-between curve. This is activation-space, non-temporal interpolation; USD remains the sole time-interpolation authority. Activations are finite, strictly positive, and unique; an implicit zero-delta sample exists at activation 0. Samples compile in (activation, canonicalPath) order, interpolate piecewise linearly, and clamp the channel weight to \[0,lastActivation\] in v1. rigExec:targetSpace must be restLocal in v1; the source-local-to-rig-common linear map converts the resulting delta exactly once. Sparse pairs are index-sorted; active inputs accumulate in canonical input-path order. A compatible optional weight object multiplies the final per-point delta after sample interpolation. Channel weights may be authored, connected, or pose-computed. Normal deltas follow activation or are rebuilt. Imported absolute points compile to deltas. Input/sample membership, activation, target-space, sparse/dense representation, index target, or delta target edits are structural; compatible channel-weight, delta-value, and weight-packet changes are value-only.

## 7.4 Weighted matrix movement

RigExecMatrixMover consumes one affine matrix T(t) and one total weight field wi(t) for the exact points target. If qi is point i from the preceding mover revision in rig-common space, the normative column-vector reference result is:

pi′=qi+wi(t)T(t)qi−qi,0≤wi(t)≤1.

The GfMatrix4d implementation is accepted only when transforming conformance points matches that equation, avoiding row/column storage assumptions. A weight of exactly zero returns the preceding point bit-for-bit; one applies the complete affine transform. Intermediate weights linearly blend the input and transformed point, including translation. T(t) must be finite and affine and must already map rig-common to rig-common; singular affine maps are legal because collapsing points can be intentional. A transform-provider prim resolves through its catalogued computeMatrix; an exact matrix property resolves through its declared base, preceding, or final phase. Point-frame providers use §5’s rest-to-pose matrix directly, while more complex deltas are built by separate matrix-math providers before they reach the mover. RigExecMatrixMover writes points only; normals and tangents require an explicitly ordered later transform or rebuild operation, and extent derives from final points.

There is deliberately no cross-mover normalization, transform palette, inverse-bind table, simultaneous multi-transform reduction, or atomic binding aggregate. Multiple matrix movers apply the equation sequentially in §4.2’s bottom-up hierarchy order, so in general they are order-dependent and do not claim equivalence to a simultaneous weighted-transform blend. Each mover independently consumes its own weight object; adding or replacing one transform/weight pair changes only that authored operation and its downstream chain. RigExecMoveTargetAPI.computeGeometry alone applies rigCommonToOutputLocal after the final mover.

The weight-object descriptor is epoch-static and contains the canonical weight target, domain, representation, cardinality, sorted sparse indices, and policy. Its hash covers the full provider identities retained collision-authoritatively in RigExecWeightBindingRecord and the BindingEpoch; the digest never replaces those records for equality or routing. Its RigExecWeightPacket contains the current immutable scalar values, sparse default, and dirty ranges. The compiler requires the canonical weight target and matrix-mover points target to match and requires exactly one finite effective weight per point. Static dense/sparse fields, scalar broadcast, dynamic modulation, values at zero/one, non-identity matrices, reflections, scale/shear, and sequential non-commuting transforms all have scalar-reference, SIMD, USD, and standalone goldens.

## 7.5 Lattice, curve/ribbon, and surface deformation

**Lattice.** Bind coordinates (u,v,w) address a regular B-spline or Bernstein cage. Evaluation is the tensor product:

p′(u,v,w)=a,b,cNa(u)Nb(v)Nc(w)Cabc.

C may come from geometry or point frames; basis/order/domain policy is authored.

**Curve/wire/ribbon.** At bind time, each driven point stores curve coordinate u, transverse coordinates (v,w), and optionally a scale reference. Evaluation computes curve position c(u) and a rotation-minimizing frame (t,n,b), then:

p′=c(u)+vn(u)+wb(u).

Twist is unwrapped along arc length; closed curves store seam/holonomy correction. The same mover can output BasisCurves guides, and its driver may be another solver result.

**Surface.** v0.1 binds polygon triangle ID plus barycentrics; runtime derivatives form the frame. Sliding closest-point is separate. Subdivision/NURBS drivers require explicit patch evaluators and are deferred.

Meshes, curves, and frames can drive one another. The same source/final split applies to pre-authored Points and BasisCurves; §4.5 supplies the curve result and Phase 3 a Points fixture.

## 7.6 Post-deformation and derived data

Post operations include:

  - volume/segment-length correction;
  - Laplacian or delta-mush smoothing with fixed adjacency;
  - collision/projection against an input proxy surface;
  - normal and tangent recomputation;
  - extent calculation;
  - user native kernels with declared primvar inputs/outputs.

Normals use inverse-transpose for pure transforms and topology-aware rebuild after nonlinear movement; extents deterministically reduce final local points. computePoints, computeNormals, computeExtent, and declared primvar computations are zero-copy post-cache views of computeGeometry, not claims of upstream pruning.

## 7.7 Topology and dynamic-output policy

OpenExec computations cannot add stage prims/properties or change scene topology. RigExec therefore supports:

  - fixed-topology point deformation;
  - pre-authored curve/mesh topology selected by variants between epochs;
  - a fixed maximum guide set whose inactive curves use standard zero widths; any custom active mask is diagnostic only;
  - baked topology-changing simulation imported as time-sampled geometry outside the live rig.

Strand birth/death, remeshing, and fracture stay outside the live evaluator; a future procedural scene index may produce topology before RigExec, never inside a mover callback.

## 7.8 Mover architecture alternatives

**Alternative A — an explicit stack relationship or one monolithic target callback.** A list is locally obvious and a monolith maximizes fusion, but both duplicate namespace intent, broaden invalidation, and hide intermediate values.

**Alternative B — composed bottom-up mover hierarchy with one immutable target application per mover. Chosen.** It makes standard USD hierarchy the order authority and preserves sparse invalidation, taps, and plugin composition. A compiler may fuse proven-compatible adjacent applications behind the same public result contract.

**CPU versus GPU.** Scalar reference kernels and CPU SIMD are mandatory v0.1 correctness and shipping paths. GPU mover-chain computation is optional and profile-driven; it is never required to pass the CPU release gates. When enabled, an entire compatible chain segment must remain resident, then read back once before renderer-neutral Hydra publication; otherwise execution falls back to CPU with telemetry. Hydra render delegates may use their normal GPU rendering paths independently. Zero-readback RigExec publication is future renderer-specific work and cannot satisfy R6 by itself.

# 8\. Evaluation and publication coordination

**Status:** synchronous OpenExec evaluation/invalidation **EXISTS**; RigExec queueing, generation fencing, residency, and publication coordination **BUILD**.

## 8.1 Scope boundary

This layer coordinates data and rendering infrastructure only. It does not define hit testing, manipulators, gesture state, viewport controls, animation editing, undo, pose libraries, or tablet editing. Those products may later consume the same value addresses, overrides, and snapshots without changing the contracts below.

## 8.2 Generation-fenced flow

**Sequence:** USD change/time or pack sample request → Evaluation queue: complete revision + tagged RigExecTimeKey + interpolation type + profile

Evaluation queue → Evaluation queue: coalesce equivalent unstarted work

Evaluation queue → Exec coordinator + VDF: serialized ChangeTime/Compute

Exec coordinator + VDF → Snapshot publisher: immutable snapshot + revision/time-key/epoch

Snapshot publisher → Snapshot publisher: reject stale or incomplete generation

Snapshot publisher → Hydra notice owner: dirty precise standard output locators

Hydra notice owner → Hydra notice owner: observers pull cached data sources

Evaluation always completes before publication. A snapshot becomes visible only when its stage/source revision, full tagged RigExecTimeKey, explicit stage interpolation type, request profile, and BindingEpoch still match. Hydra pulls never compute or wait.

## 8.3 Queue and work ownership

Current ExecUsdSystem is synchronous and has no public cancellation contract. RigExecEvaluationQueue therefore:

  - replaces an unstarted equivalent item for the same (source revision, character, RigExecTimeKey, stage interpolation type, profile, epoch);
  - allows in-flight work to finish, then rejects it if any generation key is stale;
  - batches compatible character/profile requests while leaving node parallelism to VDF;
  - admits only explicitly requested frame or render-motion samples; it does not predict user behavior;
  - yields outer scheduling slots under a global work budget so render/publication work remains bounded.

## 8.4 Explicit-time evaluation and residency

For native USD, a cache miss reconstructs the exact UsdTimeCode—numeric or PreTime—from RigExecTimeKey, performs one serialized ExecUsdSystem::ChangeTime(usdTimeCode), evaluates the prepared profiles, and publishes one complete generation fence. Timed attribute inputs use stock USD value resolution, including native Ts spline evaluation where authored. For standalone packs, the full key must be exported unless the host supplies an explicit complete resolved-state override; the pack performs no curve interpolation.

The residency cache may return an exact matching generation immediately. Misses evaluate only the requested profiles. Structural/value invalidation evicts intersecting generations using OpenExec’s affected-time information and the RigExec dependency digest.

## 8.5 Deferred consumers

Direct manipulation, undo/commit transactions, playback controls, scrubbing policy, animation-editor curve UX, and product-facing diagnostics are deferred. The atomic change-delivery batch in §11 is an evaluator input mechanism with no editing-workflow semantics. No current schema, performance gate, phase exit criterion, or renderer contract depends on these deferred consumers.

# 9\. Introspection and extraction API

**Status:** OpenExec keys/requests/cache views and DOT graphing **EXISTS**; the uniform typed tap/snapshot API **BUILD**.

## 9.1 Backend-neutral addresses

struct RigExecValueAddress {  
    SdfPath provider;             // target for base/final; mover for an intermediate  
    std::optional\<SdfPath\> target;// present for a generated (mover,target) application  
    TfToken computation;         // e.g. computePointFrame, computeGeometry  
    TfToken resultType;           // runtime registered type  
    TfToken phase = RigExecTokens-\>final; // base, afterMover, or final  
    std::optional\<int\> element;   // index into a RigExec-owned aggregate after extraction  
    std::optional\<TfToken\> field; // named primvar/field adapter  
};  
  
template\<class T\>  
class RigExecTap {  
public:  
    const RigExecValueAddress& GetAddress() const;  
};  
  
class RigExecTapSet {  
public:  
    template\<class T\>  
    RigExecTap\<T\> Add(const RigExecValueAddress&);  
  
    void Prepare();

    RigExecSnapshot Evaluate(  
        RigTime,  
        TfSpan\<const RigExecOverride\> = {});  
    RigExecSubscription Subscribe(InvalidationFn);  
};

The USD implementation resolves the address through RigExecComputationCatalog, converts the generated provider/computation to ExecUsdValueKey, and reads VtValue results; upstream keys carry no expected type. The **BUILD** catalog keys owner schema, canonical target, phase, property, and computation and validates every adapter. Shorthand addresses default to final; base/afterMover are explicit. Standalone uses the same catalog/IDs.

Snapshot access is immutable and generation-aware:

const RigExecPointFrame& frame = snapshot.Get(frameTap);  
GfMatrix4d matrix = snapshot.Get(matrixTap);  
RigExecGeometryView body = snapshot.Get(bodyTap);  
VtValue custom = snapshot.Get(untypedTap);

## 9.2 Infrastructure consumers

|  |  |
| :-: | :-: |
| \*\*Consumer\*\* | \*\*Tap set\*\* |
| Hydra scene index | visible xforms, points, primvars, extent |
| Cache/export service | matrices, point frames, moved points, normals, arbitrary computed attrs |
| Standalone pack/compiler | resolved authored inputs, manifests, addresses, parity samples |
| Automated validation | golden result addresses, tolerances, invalidation expectations |
| Headless trace export | provider/computation metadata, DOT topology, dependency/timing records |

  

Interactive consumers such as manipulators, motion trails, pose libraries, and graph browsers are deferred. A headless trace bundle may combine OpenExec’s Diagnostics::GraphNetwork() DOT output with RigExec schema/provider metadata and timings; it does not pretend the opaque schedule or every internal VDF output is a stable public object.

## 9.3 Invalidation subscription

A subscription maps OpenExec request indices back to taps and carries:

struct RigExecInvalidationHint {  
    std::optional\<EfTimeInterval\> affectedTimes; // value callback only  
    TfSmallVector\<RigExecTapId, 8\> taps; // owning; safe to queue  
};

Value callbacks supply indices plus an interval; time-change callbacks supply indices only, while RigExec’s ChangeTime wrapper records old/new times. Revision/kind come later from the stage-write gate plus USD notices or standalone journal. The coordinator joins hints after the notice epoch closes, and publication rechecks the epoch before/after compute; no listener ordering is assumed.

On removal, schema replacement, or output-set change, the tap set expires its request, resolves catalog addresses again, builds/prepares a replacement, then atomically swaps subscription/index maps. A removed key is never assumed to revive inside the old request. Missing taps surface explicitly; remove→re-add-at-same-path and schema-change tests are mandatory.

## 9.4 Language-binding boundary

The v0.1 infrastructure contract is the C++ typed address/tap/snapshot API. Generated schema bindings may exist for build and conformance tests, but a Python authoring, extraction, or product-tool surface is not a current deliverable. No Python callback runs on a VDF worker or ships in standalone/mobile evaluation profiles.

# 10\. Hydra 2.0 scene-index integration

**Status:** Hydra filtering scene indices and 26.08’s experimental xform overlay **EXISTS**; full RigExec result publication **BUILD**.

## 10.1 Filter placement and shape

The application installs RigExec explicitly around the completed UsdImaging branch; it never relies on RigExec plugin discovery order and never puts RigExec in UsdImagingSceneIndexAppendCallback. That callback runs inside the USD construction sequence, before all instancing, material-binding, and application/plugin resolution is guaranteed complete. The v26.08 construction order is:

1.  Acquire the selected HdRendererPlugin; read GetSceneIndexCreateArgs() and overlay stronger application policy.
2.  Register any application/renderer filters and ordering with HdSceneIndexPluginRegistry before graph construction.
3.  Create an unpopulated UsdImagingSceneIndex::New(createArgs, /\* no RigExec callback \*/ {}). The encapsulated chain installs the complete UsdImaging resolution stack. The deprecated UsdImagingCreateSceneIndices(...).finalSceneIndex remains the compatibility equivalent.
4.  Wrap that completed branch with the three RigExec filters below.
5.  Add the filtered branch to HdMergingSceneIndex with its activeInputSceneRoot.
6.  Call HdSceneIndexPluginRegistry::AppendSceneIndicesForRenderer(rendererDisplayName, merging, renderInstanceId, appName) to produce the terminal chain. appName is nonempty so application-specific plugin libraries can load; loading alone never replaces explicit registration/order.
7.  Call rendererPlugin-\>CreateRenderer(terminal, rendererCreateArgs), where the distinct second argument is the v26.08 HdRendererCreateArgsSchema container; do not reuse the scene-index create-args container returned by GetSceneIndexCreateArgs().
8.  Only then call UsdImagingSceneIndex::SetStage(stage), so initial PrimsAdded traverses RigExec and reaches attached observers.

If an already populated source must be wrapped, every RigExec filter initializes from one explicit HdSceneIndexPrimView traversal before forwarding a full resync. A transition host uses the same filtered branch through the renderer plugin’s back-end emulation or HdRenderIndex::InsertSceneIndex; that is a compatibility transport, not a second RigExec integration. Optional registration with HdSceneIndexNameRegistry exists only for the Hydra Scene Browser/debugger and is not how a renderer discovers results ([26.08 create-args schema](https://github.com/PixarAnimationStudios/OpenUSD/blob/v26.08/pxr/imaging/hd/sceneIndexCreateArgsSchema.h), [UsdImagingSceneIndex](https://openusd.org/dev/api/class_usd_imaging_scene_index.html), [Hydra guide](https://openusd.org/dev/api/_page__hydra__getting__started__guide.html)).

It composes three filters:

1.  RigExecInternalPrimPruningSceneIndex removes only derived \_\_RigExecGenerated application paths owned by the current RigExecUsdSystem.
2.  RigExecBindingResolvingSceneIndex atomically receives the compiler-produced immutable BindingEpoch and maps its compiled output addresses to upstream Hydra prim shells. The epoch contains rig/output paths, canonical mover-chain signatures, matrix-provider and weight-object descriptors, blend descriptors, tap mappings, instancer dependencies, and reverse input-to-output routes. The scene index does not inspect Hydra to reconstruct joint, weight, blend, or mover semantics.
3.  RigExecResultsSceneIndex overlays cached xform, primvar and extent leaves from RigExecSnapshotStore.

The pruning filter has a deliberately narrow predicate: the reserved scope must come from RigExec’s owned anonymous generated layer. It never infers ownership from a third-party rig/binding schema and never removes unrelated authored prims, computations, or renderer data. It applies the same predicate to traversal and live notices: GetChildPrimPaths() omits owned generated paths, and incoming PrimsAdded, PrimsRemoved, and PrimsDirtied entries for those paths are filtered before forwarding; unrelated notice entries pass through unchanged. Output ownership comes only from RigExecMoveTargetAPI plus the compiler’s exact output manifest.

**Pipeline:** UsdImaging final chain → RigExec internal-prim pruning · (owned generated paths only)  
RigExec internal-prim pruning · (owned generated paths only) → RigExecBindingResolvingSceneIndex · (compiler-produced output bindings)  
RigExecBindingResolvingSceneIndex · (compiler-produced output bindings) → RigExecResultsSceneIndex  
RigExecResultsSceneIndex → HdMergingSceneIndex  
HdMergingSceneIndex → application filters  
application filters → renderer filters / HdRenderer

RigExec builds reverse-mapped mesh, points, curves, xform, and property routing from its own compiled records: exact mover targets, transform providers/read phases, weight-object targets/descriptors, static or dynamic weight dependencies, blend inputs/samples, tap addresses, output paths, and conversion policies. Native v0.1 deinstances every live RigExec character before generated providers are authored; prototypes remain untouched and are not computation providers. Sharing is limited to immutable source buffers/definition hashes, and identical final results may be instanced only after evaluation.

## 10.2 Standard published schemas

The public boundary contains only standard Hydra data:

  - HdXformSchema local-to-parent matrix and preserved resetXformStack;
  - HdPrimvarsSchema flat/indexed entries for points, normals, tangents, widths, and arbitrary computed primvars;
  - HdExtentSchema min/max from final prim-local points;
  - unchanged mesh/curves/points topology from the input scene.

Frames convert against evaluated parents; packets convert to output-local and preserve indexed-primvar metadata. Renderers never call RigExec. Neutral publication is host-resident; GPU work reads back, while zero-readback is future and not R6.

## 10.3 Cached data-source contract

Hydra expects GetPrim() and GetChildPrimPaths() to be thread-safe, while notice delivery must be serialized for observers that require one thread ([HdSceneIndexBase](https://openusd.org/dev/api/class_hd_scene_index_base.html)). RigExec obeys a strict rule:

GetPrim() only reads an atomic immutable snapshot and constructs/returns cached data-source overlays. It never calls Compute, waits for evaluation, changes time, or locks the authoring stage.

Overlay containers always exist; availability changes leaves. Sampled sources retain one immutable generation, so v0.1 rebuilds the sparse root/intermediate overlay handles for each accepted snapshot instead of mutating objects a consumer may hold. GetChildPrimPaths() forwards input topology minus the exact owned \_\_RigExecGenerated application paths and synthesizes nothing. The pruning filter also removes those exact paths from every upstream added/removed/dirtied notice batch while preserving the order and locators of all unrelated entries. Structural notices atomically swap a new BindingEpoch/snapshot before owner-thread notices.

Because those container handles change, a leaf dirty alone is insufficient. The publisher starts with the logical changed leaves and calls HdContainerDataSourceEditor::ComputeDirtyLocators(). The resulting set includes each leaf plus HdDataSourceLocatorSentinelTokens-\>container (token value \_\_containerDataSource) at the prim root and every rebuilt ancestor container, forcing consumers to re-fetch the handle chain ([v26.08 editor contract](https://github.com/PixarAnimationStudios/OpenUSD/blob/v26.08/pxr/imaging/hd/containerDataSourceEditor.h)). A future implementation may use stable epoch-long containers with atomically retargeted leaves, but it may claim leaf-only dirtiness only after a tearing/lifetime proof; that is not the v0.1 contract.

### 10.3.1 Required Hydra-side inputs and published leaves

RigExec does not scrape Hydra to discover undeclared rig inputs, and it never reads source geometry during a renderer pull. Authored geometry, rig values, and mover inputs enter evaluation through the declared USD/OpenExec adapters in §§6–7. On the Hydra side, RigExecResultsSceneIndex requires only the upstream HdSceneIndexPrim shell and the standard containers it must preserve or overlay:

|  |  |  |  |
| :-: | :-: | :-: | :-: |
| \*\*Need\*\* | \*\*Locator / object\*\* | \*\*Required data-source type and value\*\* | \*\*Ownership\*\* |
| Prim identity and root | HdSceneIndexPrim | TfToken primType plus non-null HdContainerDataSourceHandle dataSource | Forward the upstream primType; a null root means no prim and therefore no RigExec publication. |
| Posed local transform | xform / matrix | HdMatrixDataSource = HdTypedSampledDataSource\\\<GfMatrix4d\\\> | RigExec snapshot; required for each published transform result and expressed local-to-parent. |
| Transform inheritance | xform / resetXformStack | HdBoolDataSource | Forward the upstream leaf unless RigExec explicitly owns a changed value; default false only when the upstream schema has no leaf. |
| Flat primvar | primvars / \\\<name\\\> / primvarValue | HdSampledDataSource; use the exact typed alias when registered, normally HdVec3fArrayDataSource for points/normals and HdFloatArrayDataSource for widths | RigExec snapshot for every declared published primvar. |
| Indexed primvar | primvars / \\\<name\\\> / indexedPrimvarValue and / indices | Exact typed sampled value plus HdIntArrayDataSource | RigExec snapshot; publish the indexed representation rather than silently flattening it. |
| Primvar descriptor | / interpolation, / role, / colorSpace, / elementSize under the same primvar | HdTokenDataSource, HdTokenDataSource, HdTokenDataSource, HdIntDataSource | Preserve the complete RigExecPrimvarDescriptor; interpolation is mandatory, while role, color space, and element size are emitted when declared or inherited. |
| Final local bound | extent / min, extent / max | two HdVec3dDataSource leaves carrying GfVec3d | RigExec snapshot, derived from final output-local points. |
| Representation mask | losing flat/indexed leaves and intentionally absent descriptor leaves | HdBlockDataSource | Conditional but required wherever recursive overlay would otherwise expose an incompatible weaker representation. |
| Fixed geometry contract | mesh, basisCurves, topology/subdivision/geom-subset children; point prim type | Existing standard upstream containers | Forward unchanged because v0.1 prohibits topology changes during evaluation. |
| Other render state | visibility, purpose, material bindings, display style, double-sided state, instancing, and unowned primvars | Existing standard upstream containers | Forward recursively; RigExec must not replace or omit unrelated renderer inputs. |

  

The only renderer-to-scene-index side channel RigExec interprets is HdSceneIndexCreateArgsSchema. motionBlurSupport is an HdBoolDataSource capability bit: false permits one sample, while true allows the explicit render-preflight sample set in §10.5. It does not communicate shutter offsets. cameraMotionBlurSupport, legacyRenderDelegateInfo, and renderer-specific extension leaves are forwarded unchanged and are not RigExec evaluation inputs. If motionBlurSupport is absent, the application’s explicit render profile is authoritative; a motion-blurred render still fails preflight when its required offsets are unavailable.

RigExecResultsSceneIndex::GetPrim(path) first fetches the upstream prim, retains its primType, builds standard containers with HdXformSchema::Builder, HdPrimvarSchema::Builder plus HdPrimvarsSchema::BuildRetained, and HdExtentSchema::Builder, and makes that sparse root the stronger first input to HdOverlayContainerDataSource::OverlayedContainerDataSources(rigExecStrongRoot, upstreamRoot). Container-on-container overlap composes recursively, so an owned value leaf wins while topology, descriptors not replaced by RigExec, and unrelated schemas continue to come from upstream.

Flat versus indexed representation is fixed for a BindingEpoch. An indexed result places HdBlockDataSource::New() at a weaker upstream primvarValue; a flat result blocks weaker indexedPrimvarValue and indices. Replaced or intentionally absent descriptor leaves are likewise masked when necessary. A representation change rebuilds the epoch, masks the losing leaves, and sends universal/resync invalidation; it never exposes incompatible flat and indexed representations simultaneously. No custom RigExec schema crosses the renderer boundary. RigExecResultsSceneIndex creates no computation prims and publishes only standard final-value leaves; unrelated upstream schemas and computations pass through unchanged.

The result leaves are snapshot-backed subclasses of the appropriate HdTypedSampledDataSource\<T\> (or HdSampledDataSource only for a catalogued Hydra-compatible type without a standard alias). Constant descriptor leaves may use retained typed data sources. Unsupported or type-changing primvars fail binding/preflight instead of exposing an unstable VtValue type to a renderer.

### 10.3.2 How Hydra discovers and pulls the results

Hydra’s contract is observer-notified and consumer-pulled; RigExec never pushes arrays into a render delegate:

1.  The application follows §10.1’s eight-step construction sequence, wrapping the completed unpopulated UsdImagingSceneIndex (or compatibility finalSceneIndex) with the three RigExec filters before stage population.
2.  Initial population produces PrimsAdded, or a newly attached consumer traverses from / with GetChildPrimPaths(). The renderer, compatibility adapter, or Hydra Scene Browser then calls the terminal chain’s GetPrim(path).
3.  GetPrim() returns the original primType and the stronger RigExec-over-upstream root container. Consumers use standard schema accessors such as HdXformSchema::GetFromParent, HdPrimvarsSchema::GetFromParent, and HdExtentSchema::GetFromParent, or navigate the same canonical locator paths through container GetNames() / Get().
4.  A consumer obtains the sampled leaf and calls GetTypedValue(shutterOffset) or GetValue(shutterOffset). For motion blur it first calls GetContributingSampleTimesForInterval(start, end, \&times) and then requests those frame-relative offsets. Every call reads the retained immutable generation; none can trigger RigExec evaluation.
5.  After a newer evaluation generation is complete, RigExec atomically swaps the snapshot, expands the logical leaf set with HdContainerDataSourceEditor::ComputeDirtyLocators(), and sends \_SendPrimsDirtied. Observers re-fetch the rebuilt container handles and leaves through the same GetPrim() /data-source path.
6.  A dirty locator is hierarchical, but its prim path is not: dirtying primvars covers its descendants on that one prim, not descendant prims. Ordinary value changes therefore begin with narrow logical leaves and add only their required container-handle chains; PrimsAdded, PrimsRemoved, or an add-again resync is reserved for a real prim/type/output-set structural change.

With a native Hydra 2.0 renderer, HdRendererPlugin::CreateRenderer(terminalSceneIndex, ...) supplies the observed terminal chain directly. With a legacy render delegate, Hydra’s adapter/render-index emulation observes the same terminal chain and translates standard schemas into delegate calls. Both paths therefore pick up identical RigExec results without renderer-specific RigExec code.

## 10.4 Invalidation and publication flow

The BindingEpoch owns the authoritative explicit reverse map:

(request profile, request index/tap)  
  → \[(Hydra prim path, Hydra data-source locator, conversion policy)\]

Publication:

1.  A USD value edit, mover hierarchy/moves edit, other relationship/topology edit, override evaluation, or time change alters a RigExec result generation.
2.  For normal OpenExec invalidation, the callback maps affected request indices and time interval into a coalesced evaluation item; it does not re-enter execution.
3.  The worker/coordinator evaluates and builds an immutable snapshot.
4.  The publisher atomically replaces the partition/profile generation if its revision is current.
5.  One notice-owner thread coalesces logical output leaves, expands their rebuilt handle chains with ComputeDirtyLocators(), and calls \_SendPrimsDirtied.
6.  Downstream filters/renderers receive PrimsDirtied and pull the new cached leaves through §10.3.2’s standard schema path.

Every weight value/default/driver edit follows the compiler’s reverse edges from that weight object through each registered consumer to the consumer’s exact outputs. For matrix/blend point consumers this reaches points and, when declared, downstream normals/tangents and extent; for a transform consumer it reaches xform/matrix; for scalar or plugin-property consumers it reaches the exact tap and any catalogued Hydra output locator, with no Hydra notice when the value is not published. Matrix-provider, delta, and blend-channel values follow the same consumer-output routing. Matrix-mover transform/weight relationships, weight targets, static↔dynamic schema changes, dynamic relationship or attribute-connection identity, operation kinds, weight domains, canonical sparse support, range policies, representation, cardinality, blend input/sample membership, activation/target-space policy, or output-manifest edits rebuild the BindingEpoch and use universal/resync invalidation. An order-only sparse pair permutation with the same canonical support and values leaves the descriptor/epoch unchanged. Graph topology never changes during evaluation; only a completed structural recompile publishes a replacement epoch.

Typical locator mapping:

|  |  |
| :-: | :-: |
| \*\*Result\*\* | \*\*Dirty locator\*\* |
| posed transform | xform / matrix; preserve/dirty xform / resetXformStack only when it changes |
| deformed points | primvars / points / primvarValue |
| normals/tangents | primvars / \\\<name\\\> / primvarValue, or primvars / \\\<name\\\> / indexedPrimvarValue plus primvars / \\\<name\\\> / indices |
| computed custom primvar | matching full flat/indexed value path and full indices path when changed |
| final bound | extent / min and extent / max (or the extent container) |
| flat/indexed representation change | block the losing leaves; HdDataSourceLocatorSet::UniversalSet() or add-again resync |
| output-set/binding/topology change | HdDataSourceLocatorSet::UniversalSet(); \\\_SendPrimsAdded again for existing-path resync, or \\\_SendPrimsAdded/\\\_SendPrimsRemoved for real existence changes |

  

Start with the narrowest logical leaf for value changes, then expand it with ComputeDirtyLocators() because v0.1 replaces retained container handles. Use universal dirtiness/resync for representation, topology, binding, or output-set changes. The reverse map is authoritative; HdDependenciesSchema is advisory unless dependency forwarding is installed.

## 10.5 Motion samples

A non-motion evaluation profile publishes one sample. HdSceneIndexCreateArgsSchema.motionBlurSupport advertises renderer capability but does not supply sample times; RigExecRenderPreflight supplies the explicit render sample offsets. Automatic shutter-distribution negotiation remains deferred **BUILD**, not an upstream assumption.

1.  Preflight validates configured shutter offsets for visible outputs and checks them against the renderer capability bit.
2.  The evaluator batches characters at each absolute sample time.
3.  RigExecSnapshotStore retains those samples under one render-generation fence.
4.  Each snapshot-backed sampled source implements GetContributingSampleTimesForInterval() and GetTypedValue() / GetValue(): it reports the retained frame-relative offsets and returns only cached values.

Missing required samples fail render preflight; GetPrim() never evaluates.

## 10.6 Frame and render-profile publication

  - Ordinary time evaluation: one full numeric/ PreTime UsdTimeCode ChangeTime plus prepared requests produces a complete immutable frame generation. The notice owner releases all affected prim dirties only after the generation fence is accepted.
  - Exact cached generation: an exact revision/ RigExecTimeKey /interpolation-type/profile/epoch cache hit may publish immediately; a miss follows §8.4. Stale completed generations fail that complete identity test.
  - Render motion samples: preflight supplies the explicit sample set, evaluation completes every required time, and the sampled data sources retain all values under one render-generation fence.

## 10.7 Imaging alternatives

**Alternative A — make each render delegate call RigExec.** Reject: duplicated integration, no ecosystem portability, and the engine leaks into renderer code.

**Alternative B — publish Hydra ExtComputations and require delegate execution.** Useful as an optional GPU/deferred path, but not universally supported.

**Alternative C — chosen default: filtering scene index overlays final standard values.** It mirrors Hydra’s intended transformation pipeline and the current exec-imaging pattern without inheriting another rig object model. Release qualification still tests Storm plus at least two production delegates; “schema-neutral” is an architecture guarantee, not an excuse to skip compatibility tests during Hydra’s ongoing 2.0 transition.

# 11\. Standalone runtime and scene backend

**Status:** Esf seam **EXISTS** (internal/non-public); standalone backend, façade, and pack **BUILD**.

## 11.1 RigExecSceneDb

The standalone backend is not a miniature UsdStage and does not implement composition. Composition happens in the USD build/export process. It is an immutable evaluated-description snapshot containing exactly the semantics the OpenExec compiler queries. RigExecDeltaBatch atomically delivers already-authorized host changes to the evaluator; it has no undo, commit, save, or editing-workflow policy:

class RigExecSceneDb {  
public:  
    RigExecSceneSnapshot GetSnapshot() const;  
    RigExecDeltaBatch BeginDeltaBatch();  
    uint64\_t GetRevision() const;  
};  
  
struct RigExecSceneSnapshot {  
    InternedPathTable paths;  
    SchemaTable concreteAndAppliedSchemas;  
    PropertyTable attributesAndRelationships;  
    TypedValueTable staticDefaults;  
    ResolvedStateTable exportedResolvedStates; // typed value or explicit blocked/no-value  
    ConnectionTable incomingAndOutgoingConnections;  
    MetadataTable metadata;  
    GeometryBlobTable geometry;

};

The adapter implements the 26.08 Esf queries required by the qualified computation corpus: stage/object/prim/property lookup and schemas; identity/metadata/connections; attribute types and exact resolved states; and relationship/forwarded targets. It does not expose or evaluate spline/time-sample curve data. exportedResolvedStates is an exact lookup table keyed by the pack’s declared RigExecTimeKey, produced by stock USD resolution during compilation. Each entry is either a typed value or an explicit blocked/no-value state; standalone never substitutes a default or preceding value for a block. Compact holders reference snapshot state that outlives ExecSystem; Initialize() rebinds retained queries. Every virtual, buffer/alignment rule, lifetime path, change/resync, and query revival has conformance tests.

## 11.2 System and change delivery

RigExecStandaloneSystem exposes the same request, prepare, compute, override, and cache-view façade as §9 plus exact exported-key selection. Internally it passes RigExecEsfSceneAdapter::Adapt(scene) to ExecSystem(EsfStage&&). Delta batches mirror execUsd: expire request indices, discard fully expired requests outside tracker locks, scope a local \_ChangeProcessor across DidResync/DidChangeInfoOnly/DidChangeIncomingConnections, select the adapter’s resolved-state row by the complete RigExecTimeKey, and call \_ChangeTime(EfTime) reconstructed with the same numeric/PreTime identity. EfTime preserves and hash-compares that tag, so exact and pre-time requests at the same numeric payload invalidate and cache separately. Only a declared exported key or a host-supplied complete resolved-state override set is legal.

RigExec\_RequestImpl adapts Esf objects into keys, expires providers, prepares schedules, binds cache-view lifetime, and renews ordinary-compute interest. These internal contacts stay in one compatibility library; deriving ExecSystem alone is insufficient.

Because those hooks are non-public and may lack shared-library exports, v1 qualifies an unchanged monolithic/static or same-visibility-boundary stock OpenUSD build. Shared packages require per-OS compile/link/load probes. Only rigExecEsfStandalone sees the shipped headers/symbols; the rest depends on the project façade. No copied private implementation, source patch, fork, export-symbol modification, or upstream acceptance is a fallback.

## 11.3 Derived .rigpack

rigexec-compile opens the authoritative USD asset and independently composes and validates every requested (variant, profile, population policy) before writing one deterministic multi-profile pack. sourceAssetId is the pipeline’s logical asset/version identity and does not encode nonsemantic authoring order, while each composedLayerHash is a schema-normalized semantic composition hash rather than a raw layer-byte hash:

Header:  
  magic, packVersion, endian, sourceAssetId, requestedProfileSetHash  
  OpenUSD/Exec compatibility SHA, RigExec schema ABI, plugin registry hash  
  timeCodesPerSecond, framesPerSecond, stageInterpolationType  
Shared tables:  
  interned paths/tokens, schema sets, properties, relationships/connections  
  rest/pose defaults, declared external value-input slots, exported RigExecTimeKey set  
  resolved value-or-block states, content-deduplication map, tap addresses  
  topology, immutable geometry blobs, blend-input/sample descriptors  
  weight-object descriptors, static constant/dense/sparse packet values/defaults  
  dynamic source bindings and fixed layouts (never evaluated dynamic values)  
Profile records:  
  profileId, composedLayerHash, population manifest  
  canonical (mover,target) applications, post-order ordinals, chain hashes  
  matrix-provider/read-phase and weight-object addresses for each matrix mover  
  partition/request manifests, schema/property overrides, static plugin IDs

Chunks per profile:  
  preview, animation or film data; optional platform-compressed geometry

The pack contains data, not code; plugin IDs resolve trusted registrations. Every native evaluation request explicitly selects stock UsdStage interpolation type linear or held. One pack pins exactly one such policy across all of its profiles; a different policy requires a separately built pack. That stage-local policy, timeCodesPerSecond, framesPerSecond, and the render-offset unit (stage time codes) are stored in the header and semantic hash. A RigExecTimeKey encodes the canonical IEEE-754 bits of the composed-stage time code plus an exact or preTime tag; NaN and infinity are illegal, and signed zero canonicalizes to positive zero. This prevents recomputing frame + offset and losing exact lookup identity.

For every exported key, rigexec-compile asks the unchanged OpenUSD 26.08 stage for **every registered external value-input slot**, including slots that currently appear static or have only one authored time sample. It calls timed UsdAttribute::Get(time) (or the equivalent qualified UsdAttributeQuery) and stores either the resolved typed value or explicit blocked/no-value state; identical states may be content-deduplicated only after resolution. This rule never relies on ValueMightBeTimeVarying(). The pack serializes no spline knots, tangents, time-sample maps, curve interpolation modes, clips, or custom animation tracks. Standalone evaluation therefore performs exact tagged-key lookup only. An unexported exact/PreTime key fails explicitly unless the host supplies a complete already-resolved state override set; arbitrary-time native interpolation remains available only in the USD deployment.

Schema normalization happens before serialization: sparse weight properties are stored only as canonical sorted (index,value) packets, raw authored pair order is omitted, and generic property records do not duplicate the raw weight arrays. On load, RigExecSceneDb exposes canonical logical rigExec:indices, rigExec:values, and rigExec:defaultWeight properties from those packet tables through Esf, so the shared StaticWeight registrations receive the same typed inputs without reconstructing raw author order. Consequently an order-only pair permutation with the same mapping produces the same semantic hash and pack bytes. Content is shared only when byte-identical across composed profiles. Identical normalized composition, ordered profile set, stage interpolation policy, time-code units, tagged sample-key set, and options produce identical bytes, with timestamps only in an unhashed sidecar.

All authoring remains USD. A DCC embedding the standalone runtime can:

  - compile USD in a pipeline process and load .rigpack;
  - translate its host scene into a transient RigExecSceneDb evaluation snapshot while USD remains the saved asset;
  - apply live typed deltas as ephemeral overlays; any durable commit must write through a USD layer/export service before save.

Editable packs and independently persisted SceneDb authoring are prohibited. Standalone constructs no UsdStage and uses no esfUsd/execUsd, though current exec/Esf types may still link core USD libraries.

## 11.4 Parity contract

Every reference asset runs through both backends:

1.  Compose USD and create ExecUsdSystem.
2.  Compile the same composition to RigExecSceneDb /pack.
3.  Build identical semantic tap sets.
4.  Evaluate the same exported exact/ PreTime keys, configured stage-time-code render-motion offsets, explicit stage interpolation policy, profiles, and complete resolved-state overrides.
5.  Compare canonical mover applications/order, matrix-mover/weight/blend descriptors, every requested intermediate revision, point frames, matrices, weight packets, statuses, final geometry, and invalidation sets.

Frame/matrix tolerances are 1e-10 in double math; bulk float points use 1e-6 × characterScale unless a kernel declares a tighter bound. Native-USD animation goldens cover spline knots/in-betweens/extrapolation, ordinary held/linear samples under both explicit stage policies, single-sample attributes, layer strength, clips, PreTime, SdfValueBlock, BlockAnimation(), and time-sample-versus-spline precedence. Pack parity compares only the explicit exported tagged-key set and configured shutter offsets, including value-versus-block state for every registered external input slot. Structural edit tests cover mover reparenting, composed child reorder, moves retargeting, matrix-transform or weight-object retargeting, static↔dynamic replacement, weight-target/representation/cardinality/canonical-support/source changes, one blend input/sample add/delete/retarget, and remove/re-add. Value-only tests cover compatible matrix, static-field/default, dynamic-driver/output, blend-channel, and delta changes plus unsorted sparse authoring and order-only paired index/value permutations; they compare the exact chains/schedules/taps invalidated. Weight descriptor/packet parity fixtures cover scalar, point, element-domain array, and registered plugin domains. Multi-target fixtures contrast independent per-target fan-out with a schema-declared atomic bundle: one bundle failure returns every named target's preceding revision under the same MoverFailed status, and a role-relationship target set that differs from moves fails composition validation. Backend parity is a Phase 0 and continuous-release gate.

## 11.5 Standalone alternatives

**Alternative A — create an in-memory** **UsdStage** **in every runtime.** It reuses ExecUsdSystem but fails the explicit no-UsdStage deployment requirement and carries composition/plugin costs onto constrained devices.

**Alternative B — fork** **exec** **with a custom scene compiler.** It meets independence but creates two engines and permanent divergence.

**Alternative C — chosen: implement Esf objects behind one compatibility boundary.** This is the seam visible in source and in Pixar’s 2024 [OpenExec architecture talk](https://openusd.org/files/OpenExecASWF.pdf), where Presto and USD scenes feed one compiler through adapters. The open-source seam remains internal, so upstream stabilization is a real milestone rather than an assumed guarantee.

# 12\. Extension and plugin API

## 12.1 Native OpenExec registration

**Status:** C++ schema registration **EXISTS**; RigExec helper conventions **BUILD**.

Custom values are registered exactly once with stable fallbacks and equality:

TF\_REGISTRY\_FUNCTION(ExecTypeRegistry)  
{  
    ExecTypeRegistry::RegisterType(RigExecPointFrame{});  
    ExecTypeRegistry::RegisterType(RigExecPointFrameArray{});  
    ExecTypeRegistry::RegisterType(RigExecGeometryPacket{});  
    ExecTypeRegistry::RegisterType(RigExecGeometryMoverResult{});  
    ExecTypeRegistry::RegisterType(RigExecPointFrameMoverResult{});  
    ExecTypeRegistry::RegisterType(RigExecFloatMoverResult{});  
    ExecTypeRegistry::RegisterType(RigExecVec3fMoverResult{});  
    ExecTypeRegistry::RegisterType(RigExecMatrixValueMoverResult{});  
    ExecTypeRegistry::RegisterType(RigExecMoverParameters{});  
    ExecTypeRegistry::RegisterType(RigExecWeightDescriptor{});  
    ExecTypeRegistry::RegisterType(RigExecWeightBindingRecord{});  
    ExecTypeRegistry::RegisterType(RigExecWeightPacket{});  
    ExecTypeRegistry::RegisterType(RigExecMatrixMoverParameters{});  
    ExecTypeRegistry::RegisterType(RigExecBlendInputDescriptor{});

    ExecTypeRegistry::RegisterType(RigExecBlendSampleDescriptor{});  
    ExecTypeRegistry::RegisterType(RigExecBlendValuePacket{});  
    ExecTypeRegistry::RegisterType(RigExecSolveStatus{});  
}

A current-style control registration lists every value used by canonical frame reconstruction:

EXEC\_REGISTER\_COMPUTATIONS\_FOR\_SCHEMA(RigExecControl)  
{  
    self.PrimComputation(RigExecTokens-\>computePointFrame)  
        .Callback\<RigExecPointFrame\>(  
            \&RigExecComputeControlPointFrame)  
        .Inputs(  
            AttributeValue\<GfVec3d\>(  
                RigExecTokens-\>rigExecPosePoints),  
            AttributeValue\<GfVec3d\>(  
                RigExecTokens-\>rigExecRestPoints),  
            AttributeValue\<TfToken\>(  
                RigExecTokens-\>rigExecFramePolicy),  
            AttributeValue\<double\>(  
                RigExecTokens-\>rigExecTwist),  
            AttributeValue\<TfToken\>(  
                RigExecTokens-\>rigExecReflectionAxis),  
            Relationship(RigExecTokens-\>rigExecParent)  
                .TargetedObjects\<RigExecPointFrame\>(  
                    RigExecTokens-\>computePointFrame));  
  
    self.PrimComputation(RigExecTokens-\>computeMatrix)  
        .Callback\<GfMatrix4d\>(\&RigExecComputeMatrix)

        .Inputs(  
            Computation\<RigExecPointFrame\>(  
                RigExecTokens-\>computePointFrame),  
            AttributeValue\<GfVec3d\>(  
                RigExecTokens-\>rigExecRestPoints));  
}

RigExecComputeControlPointFrame collects exactly four posed and four rest GfVec3d elements with VdfReadIterator\<GfVec3d\>, then reads policy, twist, reflection policy, and optional parent frame; OpenExec vectorizes array attributes by element type, so the registration intentionally uses AttributeValue\<GfVec3d\>. The deterministic world-axis fallback is a compiled constant. Nothing used by reconstruction is hidden application state. This uses upstream concepts and signatures: EXEC\_REGISTER\_COMPUTATIONS\_FOR\_SCHEMA, PrimComputation, Callback\<ResultType\>(callable), Inputs, AttributeValue\<T\> and Computation\<T\>. Relationship-targeted solvers use Relationship(...).TargetedObjects\<T\>(...)/registered provider resolution. Exact generated property tokens are produced by usdGenSchema.

The two concrete weight schemas register the same public result names and fixed result types. The project helper RigExecCompiledTargetDescriptor(...) expands during lowering to the compiler-produced static descriptor input; it is not an upstream OpenExec symbol:

EXEC\_REGISTER\_COMPUTATIONS\_FOR\_SCHEMA(RigExecStaticWeight)  
{  
    self.PrimComputation(RigExecTokens-\>computeWeightDescriptor)  
        .Callback\<RigExecWeightDescriptor\>(  
            \&RigExecBuildStaticWeightDescriptor)  
        .Inputs(  
            RigExecCompiledTargetDescriptor(  
                RigExecTokens-\>rigExecWeightTarget),  
            AttributeValue\<TfToken\>(  
                RigExecTokens-\>rigExecRepresentation),  
            AttributeValue\<int\>(RigExecTokens-\>rigExecIndices),  
            AttributeValue\<TfToken\>(  
                RigExecTokens-\>rigExecRangePolicy));  
  
    self.PrimComputation(RigExecTokens-\>computeWeightPacket)  
        .Callback\<RigExecWeightPacket\>(  
            \&RigExecBuildStaticWeightPacket)  
        .Inputs(  
            Computation\<RigExecWeightDescriptor\>(  
                RigExecTokens-\>computeWeightDescriptor),  
            AttributeValue\<int\>(RigExecTokens-\>rigExecIndices),  
            AttributeValue\<float\>(

                RigExecTokens-\>rigExecDefaultWeight),  
            AttributeValue\<float\>(RigExecTokens-\>rigExecValues));  
}  
  
EXEC\_REGISTER\_COMPUTATIONS\_FOR\_SCHEMA(RigExecDynamicWeight)  
{  
    self.PrimComputation(RigExecTokens-\>computeWeightDescriptor)  
        .Callback\<RigExecWeightDescriptor\>(  
            \&RigExecBuildDynamicWeightDescriptor)  
        .Inputs(  
            RigExecCompiledTargetDescriptor(  
                RigExecTokens-\>rigExecWeightTarget),  
            Relationship(RigExecTokens-\>rigExecBaseWeight)  
                .TargetedObjects\<RigExecWeightDescriptor\>(  
                    RigExecTokens-\>computeWeightDescriptor),  
            RigExecCompiledWeightBindings(  
                RigExecTokens-\>rigExecBaseWeight,  
                RigExecTokens-\>inputsDriver,  
                RigExecTokens-\>inputsScale,  
                RigExecTokens-\>inputsBias),  
            AttributeValue\<TfToken\>(  
                RigExecTokens-\>rigExecRepresentation),

            AttributeValue\<TfToken\>(  
                RigExecTokens-\>rigExecOperation),  
            AttributeValue\<TfToken\>(  
                RigExecTokens-\>rigExecRangePolicy));  
  
    self.PrimComputation(RigExecTokens-\>computeWeightPacket)  
        .Callback\<RigExecWeightPacket\>(  
            \&RigExecBuildDynamicWeightPacket)  
        .Inputs(  
            Computation\<RigExecWeightDescriptor\>(  
                RigExecTokens-\>computeWeightDescriptor),  
            Relationship(RigExecTokens-\>rigExecBaseWeight)  
                .TargetedObjects\<RigExecWeightPacket\>(  
                    RigExecTokens-\>computeWeightPacket),  
            AttributeValue\<float\>(RigExecTokens-\>inputsDriver),  
            AttributeValue\<float\>(RigExecTokens-\>inputsScale),  
            AttributeValue\<float\>(RigExecTokens-\>inputsBias));  
}

Each concrete mover first builds its common parameter packet with fixed schema inputs:

self.PrimComputation(RigExecTokens-\>computeMoverParameters)  
    .Callback\<RigExecMoverParameters\>(  
        \&RigExecBuildMatrixMoverParameters)  
    .Inputs(  
        AttributeValue\<bool\>(RigExecTokens-\>inputsEnabled),  
        RigExecResolvedMatrix(  
            RigExecTokens-\>rigExecTransform,  
            RigExecTokens-\>rigExecTransformReadPhase),  
        Relationship(RigExecTokens-\>rigExecWeightObject)  
            .TargetedObjects\<RigExecWeightPacket\>(  
                RigExecTokens-\>computeWeightPacket));

Every mover’s parameter computation includes inputs:enabled; disabled evaluation is therefore an ordinary invalidated pass-through of the preceding revision. RigExecResolvedMatrix(...) is a project lowering helper, not an upstream runtime selector: the compiler resolves rigExec:transform plus the structural read-phase token to exactly one catalogued base/preceding/final computeMatrix provider or exact matrix4d property adapter, then authors the generated relationship/input to that fixed provider before OpenExec compilation. It also resolves rigExec:weightObject to exactly one computeWeightPacket and verifies that the packet descriptor’s canonical target and logical count match the exact points move target. These provider bindings are static for the epoch; matrix and packet values remain ordinary dynamic computation results.

RigExecStaticWeight.computeWeightDescriptor consumes the compiler-supplied exact target descriptor plus representation, raw sparse indices, and range policy and returns the canonical sorted support. computeWeightPacket also consumes the raw indices, so it preserves authored (index,value) pairing while permuting values into descriptor order; an order-only pair permutation invalidates the packet but compares equal at the descriptor/epoch boundary. It also consumes the authored current values and sparse default. RigExecDynamicWeight.computeWeightDescriptor freezes the same target/shape contract. RigExecCompiledWeightBindings(...) is a compiler-owned static record containing the canonical base-object path plus driver/scale/bias connection/provider addresses and types; it makes those identities part of descriptor hashing without making current provider values structural. The dynamic computeWeightPacket consumes the optional base object’s packet and the declared driver, scale, and bias values. It transforms the current default only for constant/sparse packets and preserves canonical default zero for dense packets. No weight callback enumerates a stage or changes representation/cardinality. A runtime descriptor mismatch, non-finite value, or strict range violation returns one atomic RigExecGeometryMoverResult{previous, MoverFailed(...)}; the status view names the first bad provider and the value view is exact pass-through.

Blend wiring follows the same static/dynamic split. RigExecBlendSample.computeBlendSampleDescriptor consumes activation plus exact delta/index descriptors; RigExecBlendInput.computeBlendInputDescriptor consumes target-space policy and the descriptors from every related sample; RigExecBlendInput.computeBlendValuePacket adds the current channel weight and current compatible delta buffers. RigExecBlendShapeMover.computeMoverParameters statically targets every input descriptor and consumes the corresponding value packets plus its optional RigExecWeightPacket. Membership and descriptor changes rebuild the epoch; compatible values only invalidate the computation.

The generated RigExecGeometryMoverApplication.computeApplicationResult has a fixed registration that consumes the preceding RigExecGeometryPacket, one targeted RigExecMoverParameters value, and its static target descriptor before dispatching the registered kernel. Its registered computeGeometry and computeStatus views consume that one result. Point-frame/float/Vec3f/matrix-value applications follow the same {value,status} shape; transform applications additionally publish paired point-frame/matrix value views. Project target-chain helpers expand to upstream static inputs and are not claimed upstream calls. RigExecRegisterMover rejects a schema lacking the result/view computations or a registered payload type.

A mover plugin registers a typed kernel, not its own ordering system:

using RigExecGeometryMoverFn = RigExecGeometryMoverResult (\*)(  
    const RigExecGeometryPacket& previous,  
    const RigExecMoverApplication& application,  
    const VdfContext& declaredInputs);  
  
RigExecRegisterMover\<RigExecGeometryPacket\>(  
    TfType::Find\<RigExecMatrixMover\>(),  
    \&ComputeWeightedMatrixMove,  
    {.writeMode = RigExecWriteMode::Compose,  
     .targetKind = RigExecTargetKind::Geometry});

The compiler supplies one immutable application per (mover,target). A plugin may fan out through moves, but its kernel sees one exact target at a time unless its schema declares an atomic multi-result type.

Computation libraries may be separate from schema libraries; generated plugInfo.json must list every registration-owning schema under Info.Exec.Schemas.

## 12.2 RigExec plugin descriptor

Every native plugin also exports:

struct RigExecPluginDescriptor {  
    TfToken pluginId;  
    SemVer apiVersion;  
    Hash computationRegistryHash;  
    DeterminismClass determinism;  
    ThreadSafetyClass threadSafety;  
    DeviceCapabilities devices; // cpuScalar, cpuSimd, metal, cuda...  
    TfTokenVector schemas;  
};

Release assets reject an unknown ABI, nondeterministic hot-path plugin, missing fallback, unregistered result type, callback over the latency limit, or undeclared topology-changing output. Desktop may discover signed dynamic libraries; mobile uses a generated static registration table.

## 12.3 C++ client façade

auto system = RigExecSystem::FromUsdStage(stage); // or FromPack(path)  
auto taps = system-\>CreateTapSet({  
    RigExecAddress("/Char/Rig/Joints/Wrist", "computePointFrame"),  
    RigExecAddress("/Char/Geom/Body", "computeGeometry")});  
taps.Prepare();  
auto snapshot = taps.Evaluate(RigTime(1001.0));

The façade hides whether the keys/cache view are execUsd or standalone. Advanced USD clients may still use upstream APIs directly.

## 12.4 Deferred language authoring surfaces

Generated schema bindings may be used by the build and conformance harness, but Python authoring APIs, Python graph-generation tools, and Python computation registration are outside the current infrastructure scope. Production registrations are native C++ and load before system construction. No runtime decorator dynamically extends OpenExec, and no Python callback runs in film, performance, standalone, or iPadOS profiles.

## 12.5 Plugin judgment

**Alternative A — arbitrary Python in VDF workers.** Maximum flexibility, but violates practical parallel performance and risks hidden state/GIL serialization.

**Alternative B — native callbacks with generated schema registration. Chosen.** It keeps production execution consistent with OpenExec’s purity and thread-safety contract; language-level authoring ergonomics are deferred.

# 13\. Performance targets and benchmarking plan

**Status:** targets/benchmarks **BUILD**. The numbers below are provisional, unvalidated engineering targets until Phase 0 fixes hardware/assets/hashes and architecture review approves release gates. Published studio measurements are precedent, not transferable guarantees.

## 13.1 Reference workloads

|  |  |
| :-: | :-: |
| \*\*Workload\*\* | \*\*Rig/evaluation content\*\* |
| ArmReference | The document example; 5 controls, IK/FK, 5 ribbon/twist frames, 4-vertex correctness mesh. |
| Hero-A | 2,500 transform frames, 60k addressable computation/provider stages after vector packing, 250k preview/render vertices, 6 weighted matrix movers, 8 active blend targets, 12 curves, normals and extent. |
| Hero-B | 5,000 controls/avars, 150k computation stages, 1M render vertices, face/body/cloth/hair regions, 16 active targets; stress/film LOD. |
| Crowd-Lite | 40 instances, 300 frames and 50k vertices each, shared immutable definition/topology, varied animation. |
| Tablet-Hero | 400 frames, 50k vertices, 3 weighted matrix movers, 4 blend targets, one ribbon, static plugin table. |

  

The 5,000-control/150,000-node scale is informed by DreamWorks’ published Premo article, but RigExec’s workload and hardware are different. Hero-A is the warm pose/proxy evaluation-and-publication fixture; Hero-B is the stress/film-render fixture. Historical results are context only.

Desktop reference hardware is a documented 32-physical-core workstation with 128 GiB RAM; tests reserve two cores for host/Hydra/render coordination and record exact CPU, clocks, memory channels, OS, compiler and OpenUSD SHA. A 12-core laptop and 8-core Apple-silicon tablet class are secondary baselines. GPU mover results are optional and reported separately; they never substitute for scalar-reference and CPU-SIMD gates. Normal renderer GPU use is measured independently on desktop render delegates only; no on-device tablet renderer is in scope.

## 13.2 Product budgets

|  |  |
| :-: | :-: |
| \*\*Metric\*\* | \*\*Provisional target (Phase 0 calibrates)\*\* |
| Compatible authored-value or numeric-time request → coherent pose+proxy snapshot, warm Hero-A | p50 ≤ 8 ms; p95 ≤ 14 ms |
| Compatible authored-value or numeric-time request → Hydra-published standard-value generation, warm Hero-A | p50 ≤ 16.7 ms; p95 ≤ 25 ms |
| Compatible authored-value or numeric-time request → full Hero-A animation-LOD geometry generation | p95 ≤ 33 ms |
| Single callback in evaluator/publication critical path | p95 ≤ 2 ms |
| Exact cache-hit time-key request → published pose generation | p95 ≤ 2 ms |
| Cold exact-time-key request → pose+proxy generation | p95 ≤ 16 ms |
| One Hero-B film-LOD evaluation at 24 fps | p95 evaluator ≤ 30 ms; total frame ≤ 41.7 ms |
| Eight Hero-A characters at 24 fps, animation LOD | p95 evaluator+Hydra publication ≤ 25 ms; total render frame ≤ 41.7 ms |
| Forty Crowd-Lite characters at 24 fps | p95 evaluator+publication ≤ 25 ms |
| Hydra notice/publication work per 24 fps frame | p95 ≤ 2 ms and ≤ 5,000 coalesced entries |
| Warm compatible value/time-update memory growth after 10,000 evaluations | zero unbounded growth; ≤ 1% steady-state drift |
| Hero-A initial rig compile+profile prepare, stage already composed | p95 ≤ 2 s |
| Local structural recompile affecting one limb | p95 ≤ 100 ms |
| Headless tablet proxy evaluation/publication | 60 Hz target; p95 evaluator+publish ≤ 14 ms |
| Tablet full mobile-LOD time-sequenced evaluation | 30 fps; p95 evaluator+publish ≤ 25 ms |
| USD/standalone numeric parity | frames ≤ 1e-10 double; points ≤ 1e-6 × character scale |

  

Latency target order is deliberate: renderer-visible freshness is governed by worst-case evaluation-and-publication delay, not only average throughput. Batch work may use additional cores only after required frame and publication budgets are protected.

## 13.3 Instrumentation

Each headless trace spans source revision/full RigExecTimeKey/interpolation-policy request, queue, compile/schedule, callbacks/occupancy, exec completion, snapshot materialization/acceptance, Hydra notice, and render consumption.

The trace schema records:

  - critical path and ready-but-not-running time;
  - concurrency across nodes and SIMD/work within nodes;
  - cache hit/miss and invalidation fan-out by address;
  - compilation/schedule/evaluation time separately;
  - mover count, maximum target-chain depth, target fan-out, and fused/unfused applications;
  - hierarchy/moves structural recompile scope and chain-hash changes;
  - stale-generation work;
  - buffer allocation/copy/device-transfer bytes;
  - character/region deadline misses;
  - plugin quarantine or thread-unsafe serialization lanes.

DreamWorks’ LibEE material repeatedly emphasizes that rig topology and critical-path measurement are necessary to realize engine parallelism; see [“Building Highly Parallel Character Rigs”](https://research.dreamworks.com/wp-content/uploads/2018/07/highly_parallel_characters-Edited.pdf) and the [LibEE profiling chapter](https://www.oreilly.com/library/view/multithreading-for-visual/9781482243567/chapter-31.html). RigExec emits machine-readable traces; a product profiler UI is deferred.

## 13.4 Benchmark method and gates

1.  Separate cold composition/compile, warm evaluation, value/structural edits, forward/reverse/random time access.
2.  Run at least 500 warm samples; report p50/p95/p99, variance, and worst traces with asset/OpenUSD/compiler/plugin hashes pinned.
3.  Sweep worker limits and request profiles; measure partial-evaluation overhead rather than assuming it wins.
4.  Accept speed only after output/invalidation parity; run sanitizers and 10,000 randomized edit/evaluate sequences.
5.  Measure ten-minute laptop/tablet power and thermal steady state.
6.  Ledger every run; \>5% p95 or \>10% memory regression needs an approved traced waiver.

# 14\. Phased implementation roadmap

## 14.1 Upstream evolution baseline

|  |  |
| :-: | :-: |
| \*\*OpenUSD release\*\* | \*\*Verified OpenExec change relevant to RigExec\*\* |
| 25.08 | Preview: computation DSL, client request API, compile/schedule/evaluate, caching and sparse invalidation. |
| 25.11 | Additional computation/input forms, metadata/constants/connection-targeted providers, preliminary cycle detection. |
| 26.03 | Very limited ExecIrFkController; improved nonfatal cycle validation. |
| 26.05 | ExecIrControllerBuilder, incoming connections, computePath, ComputeWithOverrides, attribute expressions. |
| 26.08 | Experimental joint/FK/two-rig switch package and authoring demo; general callable callbacks; single-connection dataflow; xform-only usdExecImaging; timed UsdAttribute/UsdAttributeQuery resolution of authored scalar TsSpline values; spline-form value clips. |

  

The authoritative ledger is the [tagged OpenUSD 26.08 changelog](https://github.com/PixarAnimationStudios/OpenUSD/blob/v26.08/CHANGELOG.md). This roadmap is normatively pinned to unchanged 26.08 and hides experimental upstream rig types behind adapters. Qualification of any newer release is a separate future program and cannot silently change v0.1 evidence or behavior.

## 14.2 Phase 0 — evidence and seam proof (8–10 weeks)

**Deliverables**

  - reproducible OpenUSD 26.08 source/build manifest and primary-source capability ledger;
  - approved reference machines, frozen workload hashes, and calibrated p50/p95 thresholds;
  - minimal RigExecControl schema/computation and typed tap;
  - RigExecSceneDb + Esf adapter proof with value/time/connection/relationship queries;
  - RigExecStandaloneSystem proof and change bridge;
  - USD/standalone parity harness;
  - cached xform-only Hydra filter proof that never computes in GetPrim();
  - trace IDs and compile/schedule/evaluate timing.

**Exit criteria**

  - same point/matrix results and invalidation sets for 1,000 randomized edits, native-USD times, exported pack times, and exact-typed overrides;
  - override-vs-ordinary interest renewal and remove→re-add request rebinding pass version-pinned regressions;
  - standalone process constructs no UsdStage;
  - no source patch, fork, copied private implementation, export-symbol modification, or other change to stock OpenUSD;
  - Esf contact surface isolated in one library and version-pinned;
  - every required internal Esf/ exec entry point compiles, links, loads, and passes a smoke compute on Windows, macOS, and Linux against the pinned unchanged build; the qualified standalone distribution uses only symbols available from that stock monolithic/static or same-visibility-boundary build;
  - architecture review accepts the pinned stock-build compatibility boundary or reduces standalone scope; changing OpenUSD is not an alternative.
  - performance council accepts or revises the provisional targets with captured baseline traces.

## 14.3 Phase 1 — schema, point frames, taps and FK (10–12 weeks)

**Deliverables**

  - full schema v0.1 and usdGenSchema C++ classes;
  - mover/target APIs, pre-registered typed application/finalizer schemas, generated session-layer/pack lowering, exact target catalog, post-order compiler, and conflict/cycle validator;
  - point-frame math, degeneracy/SRT/import-export suite;
  - controls, joints, FK chain, hierarchy packing and point/matrix adapters;
  - compiler invariants, validators, profile preparation;
  - typed tap/snapshot API, machine-readable DOT/value trace export, and parity fixtures;
  - composed reference/variant/animation sample assets.

**Exit criteria**

  - all transform providers pass Points→Matrix→Points and matrix import tests, including reflection/shear;
  - every transform base/intermediate/final has paired point-frame/matrix taps;
  - references, variants, layer child-reorder opinions, and standalone packs produce identical canonical mover orders;
  - target-list permutation is inert; ambiguous siblings, structural targets, unsatisfied final reads, and cycles fail validation;
  - numeric/goal/time/shape-preserving enable edits do not recompile; variant, operation, manifest, phase, and target edits do;
  - both backends compile/evaluate the generated typed application chain, including schema-specific parameter packets; Phase 1 stops if static-input lowering or exact private-scope pruning cannot be made reliable;
  - native referenced/instanceable assets deinstance deterministically before generation, while prototype-hosted rigs fail validation;
  - ArmReference works identically in both backends and the headless parity harness.

## 14.4 Phase 2 — native USD animation and solver runtime (12–16 weeks)

**Deliverables**

  - forward analytic two-bone IK, IK/FK blend, aim/math mover operations, and constraints;
  - twist distribution and rotation-minimizing ribbon frames;
  - stock-26.08 resolved-value ingestion and conformance for scalar .spline, ordinary sparse .timeSamples, explicit stage interpolation policy, clips, layer strength, PreTime, value blocks/ BlockAnimation(), and source precedence, with no RigExec animation schema or sampler;
  - explicit-time queueing, generation checks, snapshot residency, and render-motion sample preflight;
  - exported resolved-sample-grid generation and exact-time standalone lookup.

**Exit criteria**

  - 10,000 randomized eligible spline/time-sample values and exact/paired- PreTime keys match direct timed UsdAttribute::Get() results through the OpenExec computation path;
  - every exported pack tagged key and configured render-motion offset matches native USD, while unexported standalone key requests fail explicitly;
  - Hero-A pose/proxy evaluation-and-publication gates pass on all desktop OSes;
  - stale results never publish under randomized edit/time races.

## 14.5 Phase 3 — geometry movers and Hydra publication (16–20 weeks)

**Deliverables**

  - geometry packets and CPU reference kernels;
  - weighted matrix, blend-shape, lattice, curve/ribbon, surface, and post movers plus target-chain compiler;
  - matrix-mover, static/dynamic weight, and blend descriptor parity between USD and standalone, including exact transform providers/read phases, exact weight targets, constant/dense/sparse fields, and independently layered blend inputs/samples;
  - baked standard OpenUSD xform, points, primvar, and geometry-cache export with no execution-only binding dependency;
  - RigExecInternalPrimPruningSceneIndex, RigExecBindingResolvingSceneIndex, and RigExecResultsSceneIndex;
  - motion samples, precise dependency/locator routing, Storm and two production-delegate test adapters;
  - SIMD kernels after reference correctness.

**Exit criteria**

  - automated arm goldens prove the bottom-up mover hierarchy numerically, expose every (mover,target) tap, and match the expected standard Hydra data sources;
  - multi-target fan-out, nested last-writer behavior, inputs:enabled pass-through, hierarchy reparenting, and affected-chain-only rebuild tests pass; atomic-bundle goldens contrast independent fan-out failures with one two-bone-IK bundle failure that returns every named target's preceding revision under one MoverFailed status, and reject any role-relationship target set that differs from moves;
  - stronger-layer/reference/variant tests can independently add, delete, replace, reparent, reorder, or retarget one matrix mover, static/dynamic weight object, blend input, or blend sample; only the composed mover hierarchy determines operation order, while unsorted and order-only-permuted sparse weight pairs canonicalize identically without an epoch rebuild;
  - mismatched move/weight targets, wrong-type transform or points targets, non-affine/non-finite matrices, dense/sparse cardinality errors, duplicate/out-of-range sparse indices, invalid weights, dynamic descriptor drift, duplicate activations, and sparse-delta index errors fail deterministically;
  - weight goldens cover constant empty-values/default broadcast, dense canonical default-zero enforcement, sparse defaults and pair canonicalization, no-base dynamic constant evaluation plus rejection of no-base dense/sparse forms, strict versus clamp, rejection of StaticWeight time samples/connections, identical packs/no epoch for order-only sparse pair permutations, and one scalar, one element-domain array, and one plugin-catalogued property consumer;
  - scalar-reference, SIMD, USD, and standalone weighted-matrix results match for identity and non-identity transforms, zero/one/fractional weights, static/dynamic fields, sparse defaults, reflections, scale/shear, singular affine maps, and non-commuting sequential operations;
  - exact result/status goldens prove zero-weight, disabled, and failed mover applications return the preceding typed value bit-for-bit; disabled applications report disabled without error, failures report MoverFailed with the first bad provider, and neither path can publish a torn or partial update;
  - mesh, points and basis curves work as inputs and outputs;
  - render delegates require no RigExec code and receive standard final data;
  - no compute/wait occurs in GetPrim();
  - Hydra construction and pull goldens cover pre-population attachment, already-populated wrapping with traversal/full resync, late-consumer traversal, native renderer and legacy adapter pickup, losing flat/indexed or intentionally absent leaves masked by HdBlockDataSource, and ComputeDirtyLocators() expansion through every rebuilt \_\_containerDataSource ancestor; motion cases cover motionBlurSupport true/false/absent, a one-sample non-motion profile, exact retained frame-relative offsets and cached values under one generation fence, and missing-sample preflight failure without evaluation from GetPrim() or sampled-value pulls;
  - traversal plus live PrimsAdded / PrimsRemoved / PrimsDirtied streams expose no owned \_\_RigExecGenerated path, while unrelated prims, computations, notices, and standard data sources pass through unchanged;
  - exact transform, static/dynamic weight, delta, and blend-value edits produce the expected narrow locator sets; structural mover/weight/blend edits resync, and terminal enumeration contains no custom RigExec data source;
  - Storm and two production delegates observe identical standard values and invalidations;
  - Hero-A, eight-character and invalidation-fan-out gates pass.

## 14.6 Phase 4 — standalone portability and mobile-runtime validation (12–16 weeks, overlaps Phase 3)

**Deliverables**

  - deterministic .rigpack compiler/loader and definition instancing;
  - DCC host-scene adapter example with no UsdStage;
  - Windows/macOS/Linux packages and ABI/plugin policy;
  - static monolithic iPadOS build, no Python/dynamic plugin dependency;
  - mobile LOD, ARM NEON kernels, memory/thermal telemetry;
  - headless tap/parity and Hydra-compatible publication validation.

**Exit criteria**

  - standalone execution constructs/composes no UsdStage and links no esfUsd / execUsd; residual core USD-library links are documented;
  - bit-identical pack builds and corrupted-pack validation;
  - backend parity across every registered external value-input slot in the exported exact/ PreTime resolved-state corpus;
  - headless tablet proxy and 30 fps mobile-LOD evaluation/publication gates pass for ten-minute thermal runs.

## 14.7 Phase 5 — production hardening and optional acceleration (16–24 weeks)

**Deliverables**

  - production machine-readable critical-path/invalidation trace bundle and headless asset validation;
  - plugin certification and signed package workflow;
  - crash recovery, last-valid-generation policy, metrics and support bundle;
  - deterministic multi-platform golden farm;
  - optional GPU mover-chain segments with capability/fallback telemetry;
  - RigExec schema/pack version-compatibility rules, adapters, and conformance tests.

**Exit criteria**

  - three production characters and one sequence complete a four-week pilot;
  - zero severity-1 correctness races; all sanitizer/random-edit campaigns clean;
  - measured evaluator/Hydra-publication p95 meets the approved thresholds; remaining worst-case traces have identified asset/plugin causes;
  - release/rollback/migration runbooks rehearsed;
  - architecture review accepts every remaining pinned stock-OpenUSD internal dependency.

## 14.8 Stock OpenUSD qualification and optional replacement policy

The v0.1 implementation must work against the qualified unchanged OpenUSD 26.08 build. Optional future upstream APIs may replace isolated RigExec adapters only through a conformance/deletion test; their acceptance or delivery is not a program dependency:

  - a future public Esf backend SPI may replace the compatibility include layer;
  - stable upstream invertible-rig primitives may implement RigExec controllers behind unchanged schemas;
  - a production upstream exec scene index may replace portions of RigExec imaging if it publishes points/primvars/motion samples with the same cached contract;
  - native OpenExec inversion or cancellation may replace wrapper behavior, never silently change asset semantics.

# 15\. Risks, resolved decisions, and acceptance map

## 15.1 Principal risks

|  |  |  |
| :-: | :-: | :-: |
| \*\*Risk\*\* | \*\*Exposure\*\* | \*\*Mitigation and stop condition\*\* |
| Esf is source-visible but explicitly not public API, and internal symbols may not be exported by shared builds. | High: a release may break the adapter or a Windows/macOS/Linux package may compile but fail to link/load. | Isolate all contact in rigExecEsfStandalone, pin the OpenUSD SHA, qualify an unchanged monolithic/static or same-visibility-boundary stock build, and run per-OS compile/link/load probes plus backend parity. Phase 0 stops or reduces standalone scope if any required contact is unavailable; no OpenUSD patch, fork, copied private implementation, or export change is permitted. |
| OpenExec and ExecIr are evolving rapidly. | High: asset/runtime coupling to experimental types would create migrations. | Keep RigExec\\\* schemas and taps as the stable product contract; qualify only the unchanged 26.08 baseline for v0.1. Any later-release qualification is outside v0.1. Never serialize experimental C++ types into .rigpack. |
| Public OpenExec compute is synchronous and has no documented cancellation/concurrent-call contract. | High: obsolete frame/render-sample work may consume latency or race publication. | One coordinator owns each system; the queue coalesces unstarted duplicates and generation fences reject stale completion. Do not rely on cancellation or concurrent Compute() until upstream documents it. |
| Point landmarks become coincident or nearly collinear. | High: orientation, scale, or solver results can become discontinuous. | Character-scaled thresholds, deterministic parent/rest/world fallback, solve-status taps, and headless asset validation. Release assets may not contain unresolved degeneracy; hidden previous-frame state is prohibited. |
| Solver/mover packing hides useful intermediate values. | Medium: correctness and invalidation defects become hard to isolate. | The compiler enforces a named computePointFrame for every transform-semantic stage and an address for every (mover,target) result. Packing may remove scalar plumbing, never semantic taps or golden-test visibility. |
| A weight object silently targets the wrong value or changes shape during evaluation. | High: a plausible mask could move the wrong points or corrupt a later mover chain. | The compiler canonicalizes and hashes each weight target against its consuming mover, freezes domain/representation/cardinality/source bindings in the epoch, and rejects target mismatch or dynamic descriptor drift. Layer/reference/variant, static↔dynamic, sparse/default, and non-commuting-order goldens are mandatory. |
| Namespace composition silently changes mover precedence. | High: a reference, variant, reparent, or child reorder could change the final pose or geometry. | Validate after composition; record/hash every chain; require nesting or an authored composed sibling order for overlapping writers; diff chain ordinals on publish; rebuild atomically; run reference/variant/layer golden tests. |
| Geometry evaluation becomes memory-bandwidth bound. | High: parallel nodes alone will not meet film or tablet budgets. | Immutable shared buffers, SoA layout, dirty regions, LOD profiles, fused kernels only after reference parity, allocation/copy counters, and benchmark-driven CPU/GPU partitioning. |
| Hydra pulls race execution or trigger hidden work. | High: deadlock, tearing, or unpredictable renderer latency. | Workers publish immutable snapshots first; one notice owner sends coalesced dirties; GetPrim() only reads an atomic snapshot pointer and never computes or waits. |
| Native animation source precedence or type limits are misunderstood. | High: scalar splines, ordinary samples, clips, or standalone packs may evaluate differently. | Pin stock OpenUSD 26.08 and compare timed UsdAttribute::Get() with OpenExec for knots, in-betweens, extrapolation, paired exact/PreTime keys, held/linear modes, layer strength, clips, blocks, and time-sample-versus-spline precedence. Packs contain only export-key resolved value-or-no-value states and fail unexported-key requests; no RigExec curve sampler exists. |
| Native plugins violate purity, determinism, or thread safety. | High: cache corruption and nondeterministic frames. | Mandatory descriptor, ABI/hash validation, sanitizer and randomized-order tests, certified fallbacks, time limits, and a serialized quarantine lane for non-release diagnostics only. |
| Tablet memory, thermal limits, or platform rules invalidate desktop assumptions. | High: headless time-sequenced evaluation or Hydra-publication validation misses its memory/thermal budget. | Static registration, no interpreter, compressed read-only packs, mobile LODs, NEON kernels, bounded snapshot residency, and ten-minute thermal gates. No on-device renderer, playback product, authoring, or editor surface is in scope. |

  

## 15.2 Resolved architecture decisions

1.  Stock OpenUSD only: the sole v0.1 baseline is the unchanged OpenUSD 26.08 release. All schemas, adapters, scene-index filters, and build glue live in RigExec. If the shipped Esf/ exec seam cannot be used from that qualified unchanged build, Phase 0 stops or reduces standalone scope. A newer OpenUSD release requires a separate future qualification and does not alter this contract.
2.  Provisional performance contract: retain §13’s 32-physical-core/128-GiB primary workstation, 12-core laptop, 8-core Apple-silicon tablet class, named workloads, and p50/p95 thresholds as current gates. Phase 0 records exact hardware and may calibrate them through evidence, not informal substitution.
3.  Infrastructure-only scope: v0.1 covers the composable data model, native USD animation values, compilation/evaluation, packing, extraction, desktop Hydra publication/rendering, parity, and headless mobile evaluation/publication validation. UI, manipulation, editing, on-device rendering/playback, and product tooling are deferred.
4.  Acceleration scope: scalar reference and CPU SIMD paths are mandatory and deterministic. GPU mover-chain computation is optional/profile-driven and cannot replace CPU gates; Hydra render delegates may use their normal GPU rendering paths.
5.  Native animation ownership: RigExec-authored scalar half / float / double / timecode animation uses native .spline knots evaluated through timed stock-USD resolution; other supported values use ordinary sparse .timeSamples with an explicit stage held/linear policy. Preexisting scalar time samples remain valid standard-USD inputs. RigExec has no custom animation source, curve sampler, or interpolation mode. Standalone packs store only stock-USD-resolved value-or-block states on an explicit tagged-key grid.

## 15.3 Requirement-to-evidence map

|  |  |  |
| :-: | :-: | :-: |
| \*\*Requirement\*\* | \*\*Primary design sections\*\* | \*\*Release evidence\*\* |
| R1 — dual deployment, one engine | 3.2, 11 | Same exported tagged-key corpus, results, and invalidations through USD and standalone; no UsdStage or interpolation engine in standalone process. |
| R2 — all authoring in OpenUSD | 4, 5.6 | Generated schemas validate; arm definition/reference/animation compose; timed resolved values match stock USD. |
| R3 — point-based transforms | 5, 9 | Landmark/frame/SRT property tests; every transform-semantic output passes compiler tap audit. |
| R4 — geometry in and out | 4.2, 7 | Mesh, points, and basis-curves driver/result goldens through complete bottom-up mover chains, including all intermediate applications. |
| R5 — OpenExec maximalism | 3.3–3.5 | Gap ledger reviewed against the pinned tagged OpenUSD 26.08 source. |
| R6 — Hydra 2.0 scene index | 10 | Standard xform/primvar results in three delegates; precise dirties; no compute in pulls. |
| R7 — generation-safe evaluation/publication | 6, 8, 10, 13 | Edit/time races, latency, scale, motion-sample, and thermal benchmarks meet p95 gates with no torn publication. |
| R8 — uniform extraction | 9 | Hydra, export, pack, and validation clients use one typed batch/snapshot API; no private evaluator reads. |
| R9 — portability | 11–14 | CI/package/exported-key parity matrix for Windows, macOS, Linux, and static headless iPadOS runtime. |

  

# 16\. Glossary

  - Avar: animator-facing scalar or small vector parameter, such as IK blend or elbow roll.
  - Evaluation epoch: interval during which rig and geometry topology are fixed; structural edits begin a new epoch.
  - Esf: OpenExec’s source-level, currently non-public read-only scene interface.
  - Geometry packet: RigExec’s immutable typed topology, points, primvars, provenance, and shared-buffer value.
  - Move target: canonical prim or property result named by rel rigExec:moves, with distinct base and final values.
  - Mover: a geometry or rig operation that writes one or more move targets through the common RigExecMoverAPI.
  - Mover application: compiler-generated pure computation for one (mover,target) pair and one logical post-order ordinal.
  - Bottom-up order: post-order traversal of the composed Movers namespace: descendants before ancestors and composed child order for branches.
  - Binding epoch: immutable compiled interval containing exact mover chains, matrix-provider/read-phase bindings, weight and blend descriptors, output manifests, reverse dependency routes, and snapshot-compatible Hydra publication metadata.
  - Weight object: independently composable static or dynamic prim that publishes one total scalar field over the logical elements of one canonical prim/property target.
  - Point frame: four points (o, x, y, z) representing an affine transform as origin plus transformed basis endpoints.
  - Rig pack (.rigpack): deterministic, derived standalone deployment artifact compiled from composed USD.
  - Tap: typed, public extraction address included in a RigExecTapSet.
  - Transient override: non-authored typed input supplied to one evaluation for conformance, host integration, or a future consumer; it is never durable scene state.

# 17\. Annotated references

## OpenUSD, OpenExec, and Hydra primary sources

  - Introduction to OpenExec — product boundary, caching/invalidation, and non-goals.
  - OpenExec System Design — phases, purity, inputs, and parallelism.
  - OpenExec source tree, 26.08 — definitive library/API inventory.
  - OpenUSD 26.08 changelog — release evidence and maturity.
  - OpenUSD 26.08 Time and Animated Values guide — native sparse time samples, scalar spline types, timed attribute evaluation through Ts, interpolation, value clips, and precedence.
  - UsdAttribute spline API and UsdAttributeQuery — native authored-spline access and resolved timed-value queries.
  - UsdRelationship — composed prim/property targets, list editing, forwarding, and reference/prototype restrictions.
  - UsdPrim child-order API and UsdPrimRange — true composed child order and depth-first pre/post traversal used by mover lowering.
  - Computing values with ExecUsd — batching, cache views, and time.
  - OpenExec computation registration tutorial — schema-bound C++ registration and typed input patterns.
  - Esf README and Esf stage interface — proof of the scene seam and its current non-public warning.
  - ExecIr README — current experimental invertible-rig scope, including the explicit absence of IK schemas.
  - OpenExec ASWF architecture talk — adapter/compiler architecture and relationship to Presto.
  - Hydra 2.0 Getting Started Guide — scene-index chains, filter placement, retained data, and observers.
  - HdSceneIndexCreateArgsSchema, UsdImagingSceneIndex, and HdSceneIndexPluginRegistry — renderer capability side channel and exact originating/merging/renderer-filter construction sequence.
  - HdContainerDataSourceEditor and HdOverlayContainerDataSource — sparse overlay strength, container-handle sentinel invalidation, and retained-data-source replacement rules.
  - HdSceneIndexBase — query and observer/threading contract used by the snapshot publication design.

## Production execution precedent

  - Watt et al., “LibEE: A Multithreaded Dependency Graph for Character Animation” — DigiPro ’12 evidence for node/inter-node parallelism, isolation, and profiling.
  - Watt, “LibEE: Parallel Evaluation of Character Rigs” — cached task lists, production requirements, profiling, and scale.
  - “LibEE 2: Rich Authoring and Fast Evaluation” — separation of rich authoring from optimized evaluation and incremental graph maintenance.
  - “Hierarchy Models: Building Blocks for Procedural Rigging” — packed hierarchy values and procedural rig building blocks.
  - “Premo: DreamWorks Animation’s New Approach to Animation” — precedent for decoupled edit/evaluation/display architecture; its UI and direct-manipulation product are out of scope.
  - “Building Highly Parallel Character Rigs” — practical rig topology, critical paths, and concurrency guidance.
  - “Presto Execution System: An Asynchronous Computation Engine for Animation” — background evaluation, cache filling, interruption, and stale-result precedent.
  - “Building a Scalable Evaluation Engine for Presto” — vectorized work, time as data, strip mining, and executor scaling.
  - “From Procedural Panda-monium to Fast Vectorized Execution using PCF Crowd Primitives” — aggregate vector values, cache reuse, and GPU/history-free limits.