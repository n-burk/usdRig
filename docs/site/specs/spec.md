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

Geometry remains native OpenUSD end to end. Authored and published shapes are ordinary UsdGeomMesh, UsdGeomPoints, and UsdGeomBasisCurves prims; computations, taps, snapshots, packs, and Hydra bindings carry exact native property values such as point3f\[\] points, normal3f\[\] normals, float3\[\] extent, widths, topology arrays, and standard primvars. RigExec defines no parallel geometry schema, aggregate value carrier, primvar descriptor, or serialized geometry format. A mover changes one exact native property chain, most commonly points, while topology remains fixed within an evaluation epoch. A callback may use transient aligned/SoA scratch internally, but that layout is never registered, authored, cached as a public result, serialized, or exposed through an extension surface.

Hydra receives complete immutable generations through a filtering scene index that publishes only standard local xforms, points, primvars, normals, and extents with precise dirtiness. Rig execution always finishes before Hydra pulls; render delegates require no RigExec-specific code.

Stock OpenUSD owns animation value resolution. Authored TsSpline data on supported scalar attributes is evaluated natively by timed UsdAttribute::Get()/UsdAttributeQuery::Get() through Ts; ordinary sparse timeSamples remain the native held/linear path for vectors, matrices, arrays, non-interpolatable values, and any channel not authored as a spline. RigExec defines no custom animation source or curve sampler.

A typed RigExecTapSet maps stable canonical target/type/phase addresses—plus a mover ordinal only for an intermediate revision—to private batched backend keys and immutable snapshots. Generated provider paths never enter a public or durable tap address. The evaluator, parity harnesses, exporters, and Hydra publication use this extraction boundary. UI, direct-manipulation workflows, animation editors, and general tooling are deliberately deferred.

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
4.  Point-native transforms and hierarchy-ordered movers, including one-transform matrix movers and reusable static/dynamic weight objects for exact native USD prim/property domains.
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

**Pipeline:** OpenUSD authoring · rig schemas + native UsdGeom properties · mover hierarchy + animation layers → RigExec schema compiler · post-order exact-property lowering + validation  
Standalone .rigpack · generic flattened source.usdc · + derived bindings/states → RigExec Esf standalone adapter  
OpenUSD authoring · rig schemas + native UsdGeom properties · mover hierarchy + animation layers → EsfUsd scene adapter  
EsfUsd scene adapter → OpenExec ExecSystem · compile → schedule → evaluate  
RigExec Esf standalone adapter → OpenExec ExecSystem · compile → schedule → evaluate  
RigExec schema compiler · post-order exact-property lowering + validation → OpenExec ExecSystem · compile → schedule → evaluate  
OpenExec ExecSystem · compile → schedule → evaluate → VDF parallel dataflow · typed values, caches, invalidation  
VDF parallel dataflow · typed values, caches, invalidation → RigExec Tap/Request API · native property values + rig values · immutable generation snapshots  
RigExec Tap/Request API · native property values + rig values · immutable generation snapshots → Hydra 2.0 filtering scene index  
RigExec Tap/Request API · native property values + rig values · immutable generation snapshots → Parity tests, cache export, · standalone pack validation  
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
| rigExecMove | BUILD | Mover discovery, exact-property writer chains over native USD value types, immutable weight packets, and matrix/blend/lattice/curve/surface/post kernels. |
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
| Strongly typed arbitrary computation values | EXISTS | Intro, VDF API, and OpenExec value-specifier registry | Use stock USD array attributes through scalar-element VDF execution and passive native-array extraction; register only genuinely RigExec-specific non-geometry results. |
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
| General RigExec inverse solvers | BUILD | No general/native public inversion facility | `rigexec.solve_parameters` implements a bounded pure callback solver with analytic-Jacobian or finite-difference input; UI and authored-channel adapters remain consumers. |
| Connection dataflow | EXISTS | 26.08 supports only one valid same-typed attribute connection | Use one-to-one connections; relationships for variable arity; validate limitations. |
| Computation composition across applied/concrete schemas | BUILD | Current source marks cross-registration definition composition TBD | One authoritative schema registration owns each computation name. |
| Dynamic property/primvar discovery in callbacks | BUILD | Registration inputs are static; property/namespace-child inputs are TODO | Each mover schema statically declares the exact native properties it reads; the compiler binds those paths and rejects undeclared runtime discovery. |
| IK solvers, constraints, ribbons, twist distribution | BUILD | ExecIr docs explicitly omit IK; no complete rig library | Implement as RigExec schemas/computations. |
| Production rig object model | BUILD | Intro says OpenExec alone is not a rigging system | Implement schemas, compiler invariants, versioning. |
| Point-native affine transform model | BUILD | No upstream schema/API contract | Implement RigExecPointFrame and adapters. |
| Mover/write-chain object model | BUILD | No rig operation ordering/target-write schema | Implement mover APIs, typed generated applications/finalizers, post-order lowering, phase checks, diagnostics, and taps. |
| Mesh/curve mover library | BUILD | No OpenExec geometry-operation library | Implement property-typed kernels over native UsdGeom values without introducing a parallel geometry object model. |
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
| RigExecRig | Typed prim | Character root: partition, and the namespace root for control/joint/mover discovery — it declares no membership lists, no schema-version stamp, and no derived-maintenance policy (maintenance is unconditional). Publishes computeDiagnostics; renderable bounds remain standard per-prim extent values or baked UsdGeomModelAPI extent hints. |
| RigExecControl | Typed prim | Animator-authored control value with canonical pose points, channel semantics, and limits. Publishes computePointFrame, computeMatrix. |
| RigExecJoint | Typed prim | Bind/rest identity and posed result. Publishes computePointFrame, computeMatrix. |
| RigExecFkChain | Typed solver | Applies control frames/offsets to a rest hierarchy. Publishes aggregate computePointFrameArray; addressable joint providers publish scalar frames. |
| RigExecTwoBoneIk | Typed solver/operation | Analytic two-bone IK with pole, stretch, softness, and preferred bend. It may publish frames as a provider or atomically move named joint targets. There is no absolute bone-length attribute. Each bone is MEASURED by the kernel from the frames the joints named in rigExec:joints carry ON ENTRY to this solver (root to mid, mid to end) -- their rest positions unless an earlier step of the pose stack wrote them -- plus rigExec:upperLengthOffset/rigExec:lowerLengthOffset -- the only author-time control over limb proportion. The measurement is re-taken on every evaluation, so editing a bound joint's rest re-proportions the limb immediately and needs no recompile. rigExec:joints therefore carries two meanings: the chain a solver POSES, and the chain it MEASURES. A solver whose aggregate is read by another solver (an IK feeding an IK/FK blend) does not pose -- the consumer does -- so its rigExec:joints is a rest reference by that relaxation, not by exclusivity: rigExec:joints is an ordered write and several solvers may name one joint (see “Solvers stack” in §4.2). Compile rejects a stale rigExec:upperLength/rigExec:lowerLength opinion left by an asset saved against the old schema rather than letting it sit inert. |
| RigExecBlendPointFrames | Typed solver/operation | IK/FK or general frame blend. It may publish an array or move declared frame targets. |
| RigExecAimConstraint | Typed operation | Moves a transform target by replacing aim/up orientation while preserving declared position/scale components. |
| RigExecFloatMathMover, RigExecVec3fMathMover, RigExecMatrixMathMover | Typed operations | Statically typed add, multiply, clamp, remap, or blend over an exact property target. |
| RigExecTwistDistribution | Typed solver | Distributes unwrapped twist over N frames. Publishes computePointFrameArray. |
| RigExecRibbon | Typed solver | Samples a native UsdGeomBasisCurves.points driver, constructs transported frames, and optionally conforms to a native mesh surface. Publishes computePointFrameArray; a separately ordered mover may write a target BasisCurves.points property. |
| RigExecPointFrameView | Typed adapter | Selects one declared element of an aggregate frame result and publishes scalar computePointFrame/computeMatrix for stable extraction and downstream computation. |
| RigExecBlendShapeMover | Typed mover | Applies independently composed blend inputs and native target-shape points samples to an exact standard UsdGeomPointBased.points property. |
| RigExecBlendInput | Typed prim | One independently composable blend channel with an authored/connected weight, target-space policy, and relationships to samples. |
| RigExecBlendSample | Typed prim | One target or in-between activation related to an exact native target-shape points property; dense deltas are derived against the base points. |
| RigExecMatrixMover | Typed mover | Moves the preceding native point3f\\\[\\\] points value through one declared target-local affine transform and blends each point with the common RigExecMoverAPI envelope; it contains no joint set, bind table, or packed transform list. |
| RigExecWeightObject | Abstract typed prim | Common contract for a total scalar weight field over one exact application target, or a constant operation envelope over one atomic multi-target mover. Publishes computeWeightDescriptor and computeWeightPacket. |
| RigExecStaticWeight | Concrete RigExecWeightObject | Provides a time-invariant constant, dense, or sparse authored weight field; every target element resolves to an explicit value or the authored sparse default. |
| RigExecDynamicWeight | Concrete RigExecWeightObject | Provides an epoch-shape-stable field whose values are recomputed from statically declared inputs, optionally modulating another weight object. |
| RigExecLatticeMover | Typed mover | Reads cage points from a native mesh/points prim and writes an exact native points property. |
| RigExecCurveMover | Typed mover | Reads standard basis-curve properties for wire, spline-IK, ribbon, and curve-coordinate movement and writes an exact native property. |
| RigExecSurfaceMover | Typed mover | Reads standard mesh properties for attachment/projection/sliding and surface-driven correction and writes an exact native property. |
| RigExecPostMover | Typed mover | Writes an exact standard points, normals, extent, or predeclared primvar property for smoothing, volume correction, rebuilds, clamps, and user kernels. |
| RigExecMoverAPI | Applied API | Adds rel rigExec:moves, inputs:enabled (fallback true), the common inputs:defaultWeight envelope (fallback 1), and optional rel rigExec:weightObject; marks an operation for hierarchy-ordered lowering. |
| Generated property application schemas (canonical spellings below) | Generated typed prims | Pre-registered hidden session-layer/pack nodes that realize one writer revision for the exact native or RigExec property type. Native geometry applications publish the same built-in USD-authorable value type as the target; diagnostics are a separate status computation under the same generation fence. MatrixValue means a mover that writes a matrix-valued target and is distinct from the point-moving RigExecMatrixMover. Atomic solver bundles use a registered bundle application. |
| RigExecPointTransformAPI | Applied API | Adds rigExec:restPoints, rigExec:posePoints, frame policy, twist, scale/shear limits and parent relationship. |
| RigExecControlAPI | Applied API | Channel semantics and computational limits; keyability, mirroring, presentation, and manipulation metadata are deferred. |
| RigExecPartitionAPI | Applied API | Stable character/region partition ID and deadline/LOD hints. |
| RigExecTapAPI | Multiple-apply API | Gives a stable alias and semantic role to a published result. |

  

The canonical scalar families are RigExecPointFrameMoverApplication, RigExecFloatMoverApplication, RigExecVec3fMoverApplication, and RigExecMatrixValueMoverApplication. Array behavior hosts are fixed by both operation signature and exact property type—for example RigExecMatrixPoint3fArrayMoverApplication, RigExecBlendPoint3fArrayMoverApplication, and RigExecRecomputeNormal3fArrayMoverApplication. There is deliberately no universal array host that discovers inputs at runtime: stock OpenExec freezes every plugin computation’s .Inputs(...) at static schema registration, so a plugin that needs a different side-property signature supplies a predeclared application schema and input manifest. Each host is also fixed to one exact SdfValueTypeName; point3f\[\], normal3f\[\], and float3\[\] therefore cannot collapse merely because their scalar execution element is GfVec3f. These compiler-owned prims are hidden behavior hosts, not authorable geometry schemas, and their attributes use only stock Sdf value types.

Granularity is an authoring contract, not merely an execution detail. No aggregate skeleton object owns joint membership, bind state, transform-weight tables, solver order, or output binding. Each RigExecJoint, transform provider, RigExecMatrixMover, weight object, other mover, source property, and relationship is independently addressable and may receive a composed opinion from a reference, inherit, variant, or stronger layer. Composition resolves first; only then does the compiler validate the complete rig, freeze a BindingEpoch, and pack coherent buffers for vectorized execution. Packing may optimize evaluation but never becomes the durable identity or override boundary.

Every prim that writes through RigExecMoverAPI has exactly one common envelope. Let qi be logical target element i from the preceding revision and let ri be the mover's full-strength candidate after all operation-specific inputs have been evaluated. If inputs:enabled is false, the application returns qi bit-for-bit and does not apply an envelope. Otherwise the effective envelope wi is selected by this strict precedence:

  1.  If rel rigExec:weightObject has one target, wi comes from that compatible total RigExecWeightPacket. The packet's own constant or sparse fallback remains part of the weight object. inputs:defaultWeight is ignored; it is not multiplied into or used to fill the packet.
  2.  If rel rigExec:weightObject has no target, wi is inputs:defaultWeight broadcast over every logical element of every application produced by that mover.

The relationship is optional and has cardinality zero or one; multiple, dangling, unsupported, wrong-target, wrong-domain, or wrong-cardinality bindings fail compilation rather than falling back to inputs:defaultWeight. A mover with one application target binds a field whose weightTarget canonicalizes to that target. An atomic mover with multiple applications binds only a constant, one-element operation envelope whose weightTarget is the mover prim itself; that scalar broadcasts to every application (and every joint of an atomic multi-joint solve). Dense or sparse multi-target fields are rejected because one weightTarget cannot unambiguously name several independent domains. Both sources are normalized to \[0,1\]. The application then returns EnvelopeTarget(qi,ri,wi), whose registered target semantics must preserve both endpoints exactly: zero returns qi bit-for-bit and one returns ri. Scalar, vector, matrix-value, and native point applications use component-wise qi + wi(ri - qi); transform-domain operations use their registered pose blend for translation, rotation, scale, and shear. The common envelope is the only mover-level amplitude. Concrete mover schemas do not redeclare inputs:weight or inputs:strength. RigExecBlendInput.inputs:weight remains a blend-channel activation evaluated while constructing ri, before the enclosing BlendShapeMover envelope is applied; weighting controls on non-mover weight providers likewise remain operator-specific.

RigExecMatrixMover is intentionally one operation, not a combined multi-transform aggregate. rel rigExec:moves names the exact points result it changes, rel rigExec:transform resolves one GfMatrix4d provider, and the common MoverAPI envelope resolves either one compatible weight provider or the inputs:defaultWeight broadcast. The transform is already a rig-common-to-rig-common affine map; the mover never discovers joints or constructs bind transforms. A rest-to-pose matrix can come directly from a point-frame provider (§5) or from a separate, independently composable matrix-math provider. Multiple matrix movers are ordinary hierarchy-ordered writers, so each may be added, deleted, retargeted, reordered, or overridden without rebuilding an aggregate transform table.

Every weight object has one rel rigExec:weightTarget naming a prim or exact property. For stock UsdGeomMesh, UsdGeomPoints, or UsdGeomBasisCurves, a prim target canonicalizes by the standard UsdGeomPointBased rule to its native .points property; scalar targets have one logical element and array targets use their native element count. The exception is the constant atomic-operation domain above: weightTarget names the multi-target mover prim, logicalCount is one, and the resolved value broadcasts to all of that mover's applications. The descriptor domain is compiler-derived, not freely authored: native v0.1 tokens are scalar, point for an exact UsdGeomPointBased.points target, and element for another registered array property. Plugins may add non-geometry domains for their own property schemas, but may not redefine geometry cardinality, topology, coordinate semantics, or output mapping. Thus weights remain granular and reusable without inventing a specialized geometry schema.

The field is total and has one canonical encoding. constant stores no rigExec:values elements and broadcasts rigExec:defaultWeight; dense stores exactly one value per logical element and requires rigExec:defaultWeight = 0 as ignored canonical padding; sparse stores equal-sized index/value arrays and uses rigExec:defaultWeight everywhere else. Indices are unique non-negative logical element indices. The compiler sorts sparse (index,value) pairs numerically; authored pair order is non-semantic. An atomic order-only permutation that preserves the same index/value mapping produces the same descriptor, pack, and result and does not rebuild the epoch.

Every concrete mover uses this catalogued envelope contract through RigExecMoverAPI. A bound weight object must register the action and blend operation for its target type and role; MatrixMover and BlendShapeMover accept a compatible per-point field, single-target transform movers accept a compatible one-element scalar field, scalar/property movers accept a compatible field for their exact logical domain, and multi-target movers accept only the constant atomic-operation domain. With no bound object, the common scalar inputs:defaultWeight supplies the total broadcast field. Unknown types or domains are rejected rather than inferred from equal array lengths.

RigExecStaticWeight owns time-invariant float\[\] rigExec:values, optional int\[\] rigExec:indices, and float rigExec:defaultWeight; it rejects time samples and value connections. RigExecDynamicWeight keeps the same target, representation, and cardinality for a BindingEpoch but publishes a current immutable RigExecWeightPacket from registered inputs. For the built-in multiply operation, let bi be the optional base packet and let d, s, and a be inputs:driver, inputs:scale, and inputs:bias:

ri=(bid)s+a,wi=RangePolicy(ri).

When a base object exists, its canonical target, domain, representation, cardinality, and sparse indices must exactly match the dynamic descriptor. Without a base, only constant representation is legal and bi=1 for every target element. For constant, the computed broadcast is stored as the packet default with no values. For sparse, the equation transforms every explicit value and the sparse default. For dense, it transforms all logicalCount values and retains canonical packet default zero even when bias is nonzero. RangePolicy first rejects non-finite input; strict rejects values outside \[0,1\], while clamp clamps them, with strict the release default. Plugin dynamic-weight schemas may add operations only by statically registering every input. A target, representation, domain, cardinality, canonical sparse support, relationship or attribute-connection identity, operation-kind, or policy edit is structural because it changes compiled dependencies; compatible static values/default, values from an unchanged connected provider, dynamic output values, and order-only sparse pair permutations are value-only.

Blend channels are likewise not parallel name/weight/target arrays. RigExecBlendShapeMover.rel rigExec:blendInputs targets independently composed RigExecBlendInput prims. Each input owns or connects one weight and targets one or more RigExecBlendSample prims. Each sample relates to a native UsdGeomPointBased.points property with the same point cardinality and declared index correspondence as the destination; the destination’s standard topology remains authoritative, and the compiler derives a dense delta against its base points. Sample target order is non-semantic; activations must be unique and compile in (activation, canonicalPath) order. Adding or removing an input/sample is structural; changing compatible native target points is value-only.

Every relationship that reads a UsdGeomPointBased.points dependency has an explicit uniform read-phase field on the relationship owner. The built-in mover-specialized fields are RigExecBlendSample.rigExec:pointsReadPhase, RigExecLatticeMover.rigExec:cageReadPhase, RigExecCurveMover.rigExec:driverCurveReadPhase, and RigExecSurfaceMover.rigExec:surfaceReadPhase; they accept base, application-relative preceding, or acyclic final and have schema fallback base. Every operation-specific RigExecPostMover signature similarly declares a named rigExec:\<role\>ReadPhase for each PointBased proxy, collision surface, or other side input, with fallback base. The reusable non-mover RigExecRibbon fields rigExec:driverCurveReadPhase and rigExec:surfaceReadPhase accept only base or acyclic final, also defaulting to base. A mover’s own destination input remains the immediately preceding target-chain revision unless a separately named original-base role is part of its frozen signature. A plugin PointBased side input must declare its role, phase-field token, fallback, allowed phases, and whether the dependency is specialized to one consuming mover ordinal; preceding is rejected for an unspecialized provider. Omission therefore means authored base, never an inferred current value; a value edit is ordinary, while changing a read-phase token is structural.

Generated schema classes provide the production C++ authoring API; generated language bindings are allowed only in build and conformance tests. Evaluation uses EXEC\_REGISTER\_COMPUTATIONS\_FOR\_SCHEMA. Because partial definition composition across schemas is a source TODO, one schema owns each complete computation. Base geometry values normally use the stock computeValue results of exact native attributes; the sole exception is §7.2’s stock-UsdGeomPointBased materialization for statically declared authored-base points dependencies. Pre-registered hidden property applications own typed writer callbacks and final chain mappings. Every concrete mover/operation schema also owns a statically registered computeMoverParameters computation: it declares every schema-specific attribute/relationship input and returns a registered immutable RigExecMoverParameters packet. A property application consumes only the preceding native value, mover parameters, a fixed target descriptor, and any exact native side properties declared by that mover. That compiler-private descriptor contains only canonical property identity, exact Sdf type/role, logical cardinality, and epoch-validation hashes derived from stock properties; it contains no points, topology, primvar layout, geometry buffer, or public/serialized geometry contract. Callbacks never enumerate properties or read the stage. Point-transform and mover APIs are data-only. No callback code appears in authored .usda.

Before OpenExec compilation, the RigExec compiler authors deterministic hidden application prims under each rig’s \_\_RigExecGenerated scope in an anonymous derived session layer. Native v0.1 first finds the nearest instanceable ancestor (normally the referenced asset root) and authors instanceable=false there in the derived layer; a rig in a prototype or otherwise unable to deinstance is rejected. Each generated prim then has one pre-registered property-typed schema and links to the previous revision, the mover’s computeMoverParameters, and the target descriptor; the catalog maps the chain head back to the exact property address. Built-in geometry targets use stock USD-authorable value types directly. Plugins may register immutable rig or diagnostic payloads, but no custom geometry prim, aggregate, topology carrier, or geometry result type. RigExecInternalPrimPruningSceneIndex removes only that reserved scope from imaging. Standalone packs store equivalent application/dependency records, not generated USD prims or evaluated parameter packets.

RigExecUsdSystem exclusively owns one anonymous generated sublayer per stage. Before constructing ExecUsdSystem, it inserts that layer into the editable session layer, verifies its opinions are strong enough to deinstance, and compiles under the stage coordinator. For a local structural transaction, the compiler prepares and validates replacement records off-stage, then applies only the affected generated-spec diff to the persistent owned layer inside one SdfChangeBlock while evaluation is quiescent. The existing ExecUsdSystem processes those resync scopes and incrementally rebuilds affected providers and requests; the old published snapshot remains visible until the new binding epoch succeeds. The system is recreated only for stage replacement or an explicitly unsupported full reset, not for an ordinary epoch. Teardown removes only the owned sublayer. Ordinary root-layer save never includes it, and RigExec publish/flatten/export operates on an authoring-stage view with the generated layer removed; reserved generated paths in a durable layer fail validation. Failure to isolate, update, or exclude this static-input lowering on both backends is a Phase 1 stop gate.

Move-addressable providers expose computeBasePointFrame/computePointFrame for RigExec transform values or property-specific base/intermediate/final values. A native property’s base is its stock resolved attribute computeValue, except that every statically declared authored-base PointBased points dependency uses §7.2’s predeclared exact native slot after stock velocity/acceleration materialization. Later phases are property-typed generated applications that return the identical USD-authorable value type. Registrations for catalogued movable inputs use project lowering helpers RigExecBaseValue\<T\> or RigExecResolvedValue\<T\>. Upstream AttributeValue\<T\> may follow a valid connection where allowed, so the compiler binds exact base or chain-head providers wherever a move phase is promised instead of leaving phase selection ambiguous.

Values are wired in two ways:

  - Attributes and attribute connections carry scalar or built-in-array values where there is exactly one same-typed producer. In OpenUSD 26.08, OpenExec’s computeValue follows exactly one valid same-type connection; invalid, multiple, or wrong-type connections fall back to the owning attribute’s resolved value. Rig validation rejects ambiguous connections before evaluation. For every UsdGeomPointBased.points dependency compiled as a destination base, blend sample, lattice cage, curve/ribbon driver, surface driver, or other native side input, incoming connections on that source points, velocities, or accelerations are rejected: stock UsdGeomPointBased::ComputePointsAtTime(s) resolves authored attributes but does not execute OpenExec connections, so silently accepting them would change source-motion semantics. Procedural point changes use explicit movers/providers instead.
  - Ordinary relationships target provider prims for variable-arity inputs such as joints, curve drivers, surfaces, or frame producers. Registration resolves the named computation/type expected from each target.
  - rel rigExec:moves is the reserved write-set relationship. Its targets are exact prim or property paths; target-list order is deliberately non-semantic. The mover prim’s position in the composed USD namespace supplies operation order.

An IK, aim, constraint, math, or geometry computation without moves is only a value provider; once it modifies an existing result it must apply RigExecMoverAPI and declare the complete write set. This distinction works with current OpenExec rather than assuming future multi-connection behavior.

## 4.2 Mover targeting, hierarchy order, and last-writer semantics

**Status:** **BUILD**. OpenUSD supplies relationships and a composed child order; RigExec defines their execution meaning. A [UsdRelationship](https://openusd.org/release/api/class_usd_relationship.html) can target prims, attributes, or relationships and is uniform over time. RigExec narrows moves to existing prims or non-structural, computation-catalogued attributes. Relationship-valued and topology properties plus undeclared or dangling paths are rejected; child-order metadata is not a targetable scenegraph property.

Every RigExecRig discovers mover-bearing prims beneath its composed Movers child; aggregate solvers are discovered by type anywhere beneath the rig. ONE walk numbers both, and it is taken over the WHOLE RIG: the compiler recursively walks the final composed namespace in **reverse-sibling post-order**: descendants execute before their mover parent, and sibling branches execute in reverse of the composed order returned by [UsdPrim::GetChildrenNames()](https://openusd.org/release/api/class_usd_prim.html). This matches the usdview stack presentation: the bottom sibling executes first and the top sibling executes last. v0.1 uses active, loaded, defined, non-abstract prims on deinstanced rig roots and never traverses instance proxies; load, activation, population, or instancing-policy changes are structural. GetChildrenReorder() alone is not the final composed order, and the compiler never reads it: order comes from the FINAL COMPOSED STAGE ONLY, with any parent child-order instruction already folded into GetChildrenNames() before RigExec reverses that sibling order for execution. Multiple movers writing one target are an ordinary stack whose order is therefore always well defined; nesting and reorder nameChildren = \[...\] are both conveniences for arranging that order, not preconditions for having one. The compiler does not reason about how the composed order arose—which layer or arc contributed a sibling, or whether a reorder opinion exists. No relationship-target order, plugin order, request order, or worker completion time participates.

Target resolution is exact:

  - A prim target maps through RigExecComputationCatalog to one unambiguous writable value: normally computePointFrame for a RigExec transform prim; for a stock UsdGeomMesh, UsdGeomPoints, or UsdGeomBasisCurves, the standard UsdGeomPointBased rule canonicalizes the prim to its .points property. If no unique standard or schema-declared value exists, authoring must name a property.
  - A property target retains its exact SdfPath, SdfValueTypeName, owner schema, and role in a statically typed application. Standard geometry properties require no applied RigExec API. Prim and property spellings that resolve to one value canonicalize to one address.
  - Default multi-target fan-out creates independent applications/failures per target at one ordinal. A schema-declared atomic bundle instead returns all named targets together and fails/passes through as a unit. IK maps root/mid/end with schema-specific role relationships whose target set must equal moves; it never infers roles from list position. Target-list permutation changes neither mode, mapping, order, nor result.

RigExecMatrixMover narrows the general rule: moves and transform each have cardinality one, while the common MoverAPI weightObject relationship has cardinality zero or one. Its move target must resolve to the native point3f\[\] points property of a stock mesh, points, or basis-curves prim; a prim spelling canonicalizes to that exact property. When a weight object is bound, its target must canonicalize to the same property and logical point count. Reusing one weight object is legal only for movers on that same canonical target. With no bound object, inputs:defaultWeight broadcasts to that mover's complete point target.

For the post-order movers m1,…,mn, let S0 map every writable target to its base value. Compilation lowers:

Si={Apply(mi,Si−1) if mi is enabled, Si−1 otherwise, Resolved(a)=Sn\[a\].

The implementation is static single-assignment, not mutable shared state: base(a) → after(m1,a) → … → final(a). Each declared read carries phase base, preceding, or final. preceding is context-sensitive: it reads the head before one specific consuming mover application’s ordinal and is legal only for a dependency specialized to that application. A reusable non-mover provider has no ordinal and may read only base or acyclic final; an unspecialized provider or plugin declaration using preceding fails compilation. final is legal only when every writer of that target precedes the reader and no cycle results. A shared descriptor such as RigExecBlendSample may request preceding because the compiler resolves it separately for each consuming RigExecBlendShapeMover application; it does not publish one shared preceding value. Self/later-final reads fail compilation. Overlapping writes serialize; disjoint chains remain parallel. Every transform revision publishes paired point-frame and matrix views derived from the same value/reference. A cached node is still logically applied, so “fired last” means **last in compiled logical order**, never last callback to finish.

Disabled or zero-weight operations pass through. Independent fan-out fails per target; an atomic IK failure passes through its whole bundle. Aim/math failures pass through with diagnostics unless an explicit finite fallback is authored. “Hold last valid” is publication policy, not callback state.

**One pose stack over solvers and constraints.** rigExec:joints is a write, not an exclusive claim: any number of aggregate solvers may name one joint, exactly as any number of movers may name one target. Aggregate solvers and pose-domain frame constraints are steps of ONE KIND in ONE STACK. The order of that stack is the REVERSE COMPOSED PRE-ORDER OF THE WHOLE RIG -- the bottom composed sibling first, a parent after all of its descendants, the same rule this section gives the mover stack -- and NOTHING ELSE breaks a tie. A nested step runs before the step it sits under, two steps in different scopes are ordered by where their scopes sit, and layer strength participates in that composed order exactly as reorder nameChildren does. There is no separate solver phase and no separate constraint phase: a constraint BELOW a solver runs before it and FEEDS it, and a constraint ABOVE it revises its output. Put Solvers at the bottom of the rig root to get the classic "solve, then revise" shape; that is what every shipped rig authors.

Not every solver has a position in this stack. A PRODUCER has none, and is scheduled by DATA FLOW alone -- before every consumer of its aggregate, and wherever that puts it. Two kinds of solver are producers: one whose AGGREGATE another solver reads, and one that writes no joint at all. Being READ is what takes the first out of the stack, not being joint-less: the consumed-solver relaxation is per JOINT, so a solver feeding a blend still writes any joint the blend does not name, and it is a producer all the same. This is what keeps a blend schedulable at all, because a blend is authored last and the reversed sibling order therefore puts it BEFORE the solvers it blends.

A FRAME read is POSITIONAL: a step that reads a provider reads the version standing at the reader's own place in the stack, so a reader below a writer sees the earlier version and that is not a contradiction. An AGGREGATE read is ABSOLUTE -- an aggregate is a dataflow value, not a stacked per-joint frame, and there is no earlier version of it to read -- so a hierarchy that orders an aggregate consumer before its producer, with both of them stack steps, is a COMPILE ERROR naming the pair. Under the producer rule above that pair cannot arise from authoring -- a solver whose aggregate is read is never a stack step -- so the check is a guard against the day a consumed solver is allowed to hold a position, not a migration hazard. A cycle is still a compile error (§6.6), and one solver naming one joint twice is a compile error because its two entries have no order at all.

A later writer replaces ONLY the joints it names. A joint an earlier writer wrote and this one does not name keeps its absolute out-space frame and does not follow its moved ancestor -- namespace propagation stops at every joint a solver writes -- so a partial re-write detaches the chain at that point. That is defined behaviour, not a failure. Stacking is not reported: it is ordinary authoring, the compile is silent about it, and the per-joint chain the compiler built -- solvers and constraints in execution order -- is what a tool reads to show the order.

A solver's write is visible to every later solver and every later pose constraint: a solver that reads a joint an earlier writer wrote sees the earlier version, and a solver that reads a joint it writes itself sees the version standing before its own commit. final on a joint means after its LAST writer, where the last writer is the last pose-walk commit that writes it -- the highest-ordinal constraint targeting it if there is one, otherwise the last solver in its stack. A reader whose read phase names a solver sees the joint as that solver left it; such solver checkpoints are addressable from the geometry domain only (a mover's rigExec:transform phase), because a pose constraint's frame sources carry no read phase and always observe the top of the stack. A solver MEASURES each joint it names from the frame the preceding steps of the pose stack left there: the incoming frame replaces the authored rest as that solver's rest reference. A joint with no earlier writer still hands the solver its authored rest:space rest, which is why a rig whose constraints all sit above its solvers is unchanged by this rule. EVERY aggregate solver answers to this, and a step below one is never discarded. RigExecTwoBoneIk and RigExecSplineIk MEASURE from the joint rests, so a step below one of them re-proportions the chain -- the bone lengths and the rest spline are re-measured from the incoming frames -- and does not merely re-orient it. The four that measure from somewhere else instead COMPOSE onto the joint rest: RigExecFkChain applies its control deltas to it rather than to the control's own rest, RigExecBlendPointFrames applies the blended map to it while still measuring both inputs against the A aggregate's rests, and RigExecRibbon and RigExecTwistDistribution apply each element's own rest-to-pose map to it. RigExecSingleChainIkConstraint, the one CONSTRAINT that measures from joint rests, takes the incoming frame the same way. A joint no earlier step wrote keeps its authored rest as that reference, so every one of the seven is bit-identical to what it was on a rig that does not stack. A solver whose aggregate another solver reads still does not write the joints that consumer writes: its rigExec:joints remains a rest reference, so the IK/FK idiom is a single-writer picture and is unchanged.

base(J) is the frame after the LAST SOLVER write in J's chain, or, for a joint no solver writes, the frame the pose walk seeded it with. It is no longer "before every constraint": a constraint that fell below the last solver is folded into base, through that solver's rest reference and then through the solver's own write. A reader that wants a joint before ANY pose step must name a phase rather than rely on base. AtPrim(P) resolves against the INTERLEAVED chain -- solvers and constraints in one order -- so naming the Solvers scope means "after the last solver", which is exactly base, and naming the Movers scope means "after the last constraint".

Adding, removing, activating, reparenting, or renaming a mover; editing moves; changing child order, operation kind, exact target type/cardinality, read phase, or composition/variant selection begins a new epoch. Numeric inputs, weights, goals, time samples, compatible spline-knot/tangent/extrapolation edits, native array values, and direct inputs:enabled edits are value-only **only when pass-through preserves** **SdfValueTypeName****, cardinality, standard primvar metadata/indexing, and the declared property set**; otherwise enablement is structural or prohibited. Switching an attribute between time-sample, spline, default, or clip value-source forms is validated under standard USD resolution and invalidates its dependent value ranges; it does not create a RigExec curve object. active=false is structural and removes descendants. Cross-rig targets are rejected in v1; cross-character effects use §6.2’s multipass.

## 4.3 Point-frame storage versus runtime values

USD-authored frames use built-in point3d\[\] attributes with exactly four elements in order \[O, X, Y, Z\]; validators enforce cardinality and finiteness. Runtime scalar providers return registered RigExecPointFrame. Packed solver boundaries return a registered aggregate scalar RigExecPointFrameArray with an immutable shared span. For a project-defined computation, stock 26.08 fixes extraction type to the registered result type and ExecUsdValueKey has no element selector; RigExec therefore does not claim native client indexing of an arbitrary VDF vector or use VtArray\<RigExecPointFrame\> as a result. USD array attributes are a separate stock case: their built-in resolved/connected value definitions execute as vectors of the scalar element type and can extract the exact VtArray named by the attribute.

Every semantically addressable element also has a scene-addressable scalar provider—for example each RigExecJoint or ribbon-sample provider consumes the aggregate and publishes computePointFrame. A tap’s optional element selector indexes a project-owned aggregate *after* extraction; it is not part of the upstream key. This preserves packed execution without hiding semantic transforms.

Geometry deliberately has no comparable RigExec aggregate. Each standard property remains its own exact USD attribute value—typically point3f\[\], normal3f\[\], or float3\[\]—with VtArray\<GfVec3f\> appearing only at the ordinary USD/extraction/snapshot boundary. Inside OpenExec, the same array is a VDF vector of GfVec3f elements. No VtArray is registered or used as a plugin-computation input/result type, and no geometry type registration is required. Stock VDF vectors, C++ extraction storage, optional transient SIMD scratch, and GPU execution are separate implementation layers and never create a RigExec geometry contract.

## 4.4 Composition and layer ownership

Rig assets use four conceptual layers:

1.  Model layer — native rest mesh/points/curves, their standard topology/properties, and native target shapes for blend samples.
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
        uniform token rigExec:partition = "ArmAsset"  
        # No membership lists: controls, joints, and movers are discovered  
        # from the namespace beneath the rig, and operator wiring (a  
        # solver's ordered rigExec:joints / rigExec:controls) is the graph.  
  
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
            }  
            def RigExecJoint "Elbow" (  
                prepend apiSchemas = \["RigExecPointTransformAPI"\]  
            )

            {  
                point3d\[\] rigExec:restPoints = \[  
                    (4, 10, 0), (8, 10, 0), (4, 11, 0), (4, 10, 1)  
                \]  
                rel rigExec:parent = \</ArmAsset/Rig/Joints/Shoulder\>  
            }  
            def RigExecJoint "Wrist" (  
                prepend apiSchemas = \["RigExecPointTransformAPI"\]  
            )  
            {  
                point3d\[\] rigExec:restPoints = \[  
                    (8, 10, 0), (10, 10, 0), (8, 11, 0), (8, 10, 1)  
                \]  
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
                rel rigExec:samples = \</ArmAsset/Rig/BlendInputs/BicepFlex/Full\>  
  
                def RigExecBlendSample "Full"  
                {  
                    float rigExec:activation = 1  
                    rel rigExec:targetPoints = \</ArmAsset/Targets/BicepFlexTarget.points\>  
                }  
            }  
        }  
  

        def Scope "Movers"  
        {  
            # Display Geometry above Pose; the stack executes bottom-to-top.
            reorder nameChildren = \["Geometry", "Pose"\]
  
            def Scope "Pose"  
            {  
                def RigExecAimConstraint "WristAim" (  
                    prepend apiSchemas = \["RigExecMoverAPI"\]  
                )  
                {  
                    rel rigExec:moves = \</ArmAsset/Rig/Joints/Wrist\>  
                    rel rigExec:aimTarget = \</ArmAsset/Rig/Controls/ElbowPole\>  
                    float inputs:defaultWeight = 0.2
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
                def RigExecPostMover "RecomputeExtent" (  
                    prepend apiSchemas = \["RigExecMoverAPI"\]  
                )  
                {  
                    rel rigExec:moves = \</ArmAsset/Geom/ArmBody.extent\>  
                    uniform token rigExec:operation = "recomputeExtent"  
  
                    def RigExecPostMover "RecomputeNormals" (  
                        prepend apiSchemas = \["RigExecMoverAPI"\]  
                    )  
                    {  
                        rel rigExec:moves = \</ArmAsset/Geom/ArmBody.normals\>  
                        uniform token rigExec:operation = "recomputeNormals"

  
                        def RigExecPostMover "VolumeCorrect" (  
                            prepend apiSchemas = \["RigExecMoverAPI"\]  
                        )  
                        {  
                            rel rigExec:moves = \</ArmAsset/Geom/ArmBody.points\>  
                            uniform token rigExec:operation = "volumeCorrect"  
                            float inputs:defaultWeight = 0.25
  
                            def RigExecCurveMover "RibbonWrap" (  
                                prepend apiSchemas = \["RigExecMoverAPI"\]  
                            )  
                            {  
                                rel rigExec:moves = \</ArmAsset/Geom/ArmBody.points\>  
                                rel rigExec:driverCurve = \</ArmAsset/Geom/RibbonDriver\>  
                                rel rigExec:driverFrames = \</ArmAsset/Rig/Solvers/ArmRibbon\>  
                                rel rigExec:bindCoordinates = \</ArmAsset/Geom/ArmBody.primvars:st\>  
                                uniform token rigExec:mode = "ribbon"  
  
                                def RigExecMatrixMover "ShoulderMatrix" (  
                                    prepend apiSchemas = \["RigExecMoverAPI"\]  
                                )

                                {  
                                    rel rigExec:moves = \</ArmAsset/Geom/ArmBody.points\>  
                                    rel rigExec:transform = \</ArmAsset/Rig/Joints/Shoulder\>  
                                    rel rigExec:weightObject = \</ArmAsset/Rig/Weights/ShoulderPoints\>  
                                    uniform token rigExec:transformReadPhase = "final"  
  
                                    def RigExecMatrixMover "ElbowMatrix" (  
                                        prepend apiSchemas = \["RigExecMoverAPI"\]  
                                    )  
                                    {  
                                        rel rigExec:moves = \</ArmAsset/Geom/ArmBody.points\>  
                                        rel rigExec:transform = \</ArmAsset/Rig/Joints/Elbow\>  
                                        rel rigExec:weightObject = \</ArmAsset/Rig/Weights/ElbowDriven\>  
                                        uniform token rigExec:transformReadPhase = "final"  
  
                                        def RigExecMatrixMover "WristMatrix" (  
                                            prepend apiSchemas = \["RigExecMoverAPI"\]  
                                        )  
                                        {  
                                            rel rigExec:moves = \</ArmAsset/Geom/ArmBody.points\>  
                                            rel rigExec:transform = \</ArmAsset/Rig/Joints/Wrist\>  
                                            rel rigExec:weightObject = \</ArmAsset/Rig/Weights/WristPoints\>

                                            uniform token rigExec:transformReadPhase = "final"  
  
                                            def RigExecBlendShapeMover "BicepFlex" (  
                                                prepend apiSchemas = \["RigExecMoverAPI"\]  
                                            )  
                                            {  
                                                rel rigExec:moves = \</ArmAsset/Geom/ArmBody.points\>  
                                                rel rigExec:blendInputs = \</ArmAsset/Rig/BlendInputs/BicepFlex\>  
                                                rel rigExec:weightObject = \</ArmAsset/Rig/Weights/BicepMask\>  
                                            }  
                                        }  
                                    }  
                                }  
                            }  
                        }  
                    }  
                }  
  
                def RigExecPostMover "RecomputeGuideExtent" (  
                    prepend apiSchemas = \["RigExecMoverAPI"\]  
                )  
                {

                    rel rigExec:moves = \</ArmAsset/Geom/RibbonGuides.extent\>  
                    uniform token rigExec:operation = "recomputeExtent"  
  
                    def RigExecCurveMover "GuideFromRibbon" (  
                        prepend apiSchemas = \["RigExecMoverAPI"\]  
                    )  
                    {  
                        rel rigExec:moves = \</ArmAsset/Geom/RibbonGuides.points\>  
                        rel rigExec:driverFrames = \</ArmAsset/Rig/Solvers/ArmRibbon\>  
                        uniform token rigExec:mode = "emitGuidePoints"  
                    }  
                }  
            }  
        }  
    }  
  
    def Scope "Geom"  
    {  
        def BasisCurves "RibbonDriver"  
        {  
            token visibility = "invisible"  
            uniform token type = "cubic"

            uniform token basis = "bspline"  
            uniform token wrap = "nonperiodic"  
            int\[\] curveVertexCounts = \[4\]  
            point3f\[\] points = \[  
                (0,10,0), (2.7,10,0), (5.3,10,0), (8,10,0)  
            \]  
            float\[\] widths = \[0.1, 0.1, 0.1, 0.1\] (  
                interpolation = "vertex"  
            )  
        }  
  
        def BasisCurves "RibbonGuides"  
        {  
            uniform token type = "linear"  
            uniform token wrap = "nonperiodic"  
            int\[\] curveVertexCounts = \[5\]  
            point3f\[\] points = \[  
                (0,10,0), (2,10,0), (4,10,0), (6,10,0), (8,10,0)  
            \]  
            float3\[\] extent = \[(-0.03,9.97,-0.03), (8.03,10.03,0.03)\]  
            float\[\] widths (  
                interpolation = "vertex"

            )  
        }  
  
        def Mesh "ArmBody"  
        {  
            uniform token subdivisionScheme = "none"  
            int\[\] faceVertexCounts = \[4\]  
            int\[\] faceVertexIndices = \[0, 1, 2, 3\]  
            point3f\[\] points = \[  
                (0,9.5,0), (8,9.5,0), (8,10.5,0), (0,10.5,0)  
            \]  
            normal3f\[\] normals = \[  
                (0,0,1), (0,0,1), (0,0,1), (0,0,1)  
            \] (interpolation = "vertex")  
            float3\[\] extent = \[(0,9.5,0), (8,10.5,0)\]  
            texCoord2f\[\] primvars:st = \[  
                (0,0), (1,0), (1,1), (0,1)  
            \] (interpolation = "vertex")  
        }  
    }  
  
    def Scope "Targets"

    {  
        def Points "BicepFlexTarget"  
        {  
            token visibility = "invisible"  
            point3f\[\] points = \[  
                (0,9.5,0), (8,9.75,-0.15), (8,10.75,0.15), (0,10.5,0)  
            \]  
        }  
    }  
  
    variantSet "rigComplexity" = {  
        "preview" {  
            over "Rig" {  
                over "Movers" {  
                    over "Geometry" {  
                        over "RecomputeExtent" {  
                            over "RecomputeNormals" {  
                                over "VolumeCorrect" {  
                                    bool inputs:enabled = false  
                                    over "RibbonWrap" {  
                                        bool inputs:enabled = false  
                                    }

                                }  
                            }  
                        }  
                        over "RecomputeGuideExtent" {  
                            over "GuideFromRibbon" {  
                                bool inputs:enabled = false  
                            }  
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
                        over "RecomputeExtent" {  
                            over "RecomputeNormals" {  
                                over "VolumeCorrect" {  
                                    bool inputs:enabled = false  
                                }  
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

The parent-authored child reorder displays Geometry above Pose, so reverse-sibling stack execution makes Pose precede Geometry. Inside geometry, the lower guide branch resolves GuideFromRibbon → RecomputeGuideExtent before the upper ArmBody branch resolves BicepFlex → WristMatrix → ElbowMatrix → ShoulderMatrix → RibbonWrap → VolumeCorrect → RecomputeNormals → RecomputeExtent. Each matrix mover consumes one transform and one total common envelope, supplied here by a weight object and otherwise supplied by inputs:defaultWeight; no operation owns a transform list. Every geometry prim is an unadorned stock Mesh, Points, or BasisCurves, and every write target is an exact standard property. The blend sample is a native target shape whose standard points are compared with the base shape. This polygon-mesh fixture explicitly targets final native normals and extent, and the ribbon guide explicitly targets final native points and extent; subdivision and point-cloud fixtures exercise §7.6’s other stock policies and author their own exact extent movers. Preview keeps RibbonGuides.points targetable and hides the curves with standard zero widths. ClampIKFKWeight is a typed property mover before ancestor WristAim. Selecting these variants starts a new epoch; direct shape-preserving enable or weight-value edits need not. The compiler generates scalar views for all twist/ribbon frames; no separate authored stack list or stage links exist.

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

**Explicit list versus composed hierarchy.** A stack list is locally obvious but duplicates namespace intent. The chosen reverse-sibling post-order moves hierarchy works through composition; same-target movers are ordered by that hierarchy, arranged by nesting or by a parent child-order instruction when the authored order is not the wanted one. The compiler records/diffs ordinals and chain digests.

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

## 5.5 Pure inverse parameter solve

`rigexec.solve_parameters` provides the pure-computation boundary without an editing UI. Let authored avars be vector a, forward rig evaluation be F(a), selected landmark projection be S, and desired points be d. An analytic controller inverse is preferred. The numeric fallback solves:

minΔa‖W(SF(a+Δa)−d)‖22+λ‖CΔa‖22+μ‖Δa‖22

subject to channel limits. W weights selected points/axes, C regularizes semantic channels or distance from a declared reference pose, and μ is Levenberg damping. The Python API accepts an analytic Jacobian or uses bounded central finite differences on an explicitly supplied prototype callback. Its current reference regularization uses C=I; damping stabilizes the normal-equation step. Termination is (weighted point error \< 10^-5 × characterScale) or (relative improvement \< 10^-6) or the profile’s fixed iteration limit. Only reaching the target reports `converged`; bounded/unreachable, stalled, iteration-limit and non-finite outcomes report a reason and retain the best parameters. The dense implementation is intended for small channel subsets, with no full-rig scaling promise.

Desired landmarks enter through the callback's explicit arguments, declared typed inputs or low-level evaluation overrides. Forward/inverse callbacks remain pure. Hit testing, gesture state, commits, and undo are separate consumers. See [Python bake and inverse APIs](python-bake-inverse.md).

## 5.6 Native USD time evaluation and interpolation

OpenUSD 26.08 is the only temporal-interpolation authority:

  - TsSpline .spline opinions provide sparse curve knots for scalar half, float, double, and timecode attributes. Timed UsdAttribute::Get(time) and UsdAttributeQuery::Get(time) resolve the authored source and evaluate its Bezier/Hermite, knot, looping, and extrapolation behavior through Ts.
  - RigExec-authored animation uses .spline for every eligible scalar channel. Ordinary sparse timeSamples use stock USD held/linear value resolution and are required for non-scalar tuples, matrices, quaternions, arrays, and non-interpolatable values. Preexisting standard-USD scalar timeSamples remain valid inputs and are resolved unchanged; RigExec does not convert them to splines.
  - UsdStage interpolation type is stage-local state, not authored scene data. Every evaluation/build profile sets it explicitly— linear by default, with a separately named held profile when required—and includes it in the generation key, pack manifest, and semantic hash. It affects ordinary timeSamples, not spline curve semantics.
  - If a site or value-clip manifest presents both forms for one attribute, standard USD precedence applies; RigExec neither merges nor reinterprets the sources.
  - Timed SdfValueBlock intervals can make Get(time) return no value. BlockAnimation() suppresses weaker animation but may expose a weaker/default/fallback value under standard resolution. RigExec preserves exactly the timed Get outcome—typed value or no value—and never invents a preceding sample or custom hold.

For rigExec:posePoints, USD first resolves the complete four-element point3d\[\] value at the requested time, then RigExec validates cardinality/finiteness and reconstructs the frame. Degenerate resolved landmarks follow §5.3’s deterministic policy. Imported standard USD transform properties remain in their native authored representation. There is no RigExec track-level point/transform interpolation mode, no custom animation schema, and no curve-sampling bridge. §7.2’s pre-evaluation PointBased dependency materialization is distinct: it calls the stock geometry API for standard authored velocity/acceleration semantics and stores each exact native result; it defines no interpolation algorithm or animation source.

## 5.7 Universal point extraction invariant

OpenExec’s public request API addresses published computations on prims or attributes; schedules are opaque, and it has no public “read any anonymous VDF node output” handle. RigExec meets the extraction requirement with a compiler invariant:

1.  Every semantically meaningful transform stage is a scene-addressable RigExec prim or attribute provider.
2.  RigExecTransformComputation(...) publishes paired point-frame/matrix results for every base, post-mover, and final transform revision; packed arrays are intermediates.
3.  A solver with multiple semantically addressable transforms creates scalar scene-addressable providers for them; it may not make a client depend on an opaque VDF element or private node identity.
4.  Every published result has a stable RigExecValueAddress and optional RigExecTapAPI alias.
5.  The validator fails a rig when a transform-semantic output lacks the point computation, reference points, or type registration.

OpenExec’s existing ExecSystem::Diagnostics::GraphNetwork() can produce a raw DOT topology dump for transient internal diagnosis. A persisted or externally delivered trace canonicalizes every result to the public target/type/phase(/mover) address and redacts generated paths and opaque private node identities. Value inspection is the RigExec address/tap layer. Production features never depend on private node identities or a modified/instrumented OpenUSD build.

# 6\. Execution model

## 6.1 Compilation, scheduling, and evaluation

**Status:** OpenExec phases **EXISTS**; RigExec compiler policy and request profiles **BUILD**.

RigExec preserves OpenExec’s phase boundaries:

1.  Compilation: composed scene description plus registered computations become the execution network. RigExec validates and canonicalizes moves targets, performs the fixed-policy post-order walk, and lowers each target’s writers into base/intermediate/final providers before OpenExec compiles them. Structural changes incrementally rebuild affected areas.
2.  Scheduling: a batch of requested value keys becomes a reusable request schedule. PrepareRequest() front-loads this work.
3.  Evaluation: invalid requested values are pulled, callbacks run, and valid results are reused from caches.

Stable pose, proxyGeometry, fullGeometry, and renderMotion profiles are prepared in batches; the OpenExec tutorial explicitly recommends batched ExecUsdRequest use ([system design](https://openusd.org/dev/api/page__execution__system__design.html), [request tutorial](https://openusd.org/dev/api/md_pxr_exec_exec_usd_docs_tutorial1_computing_values.html)).

The mover compiler stores, per target, the traversal policy, mover paths, post-order ordinals, canonical targets, operation kinds, and chain hash. A mover that reads another moved target binds the latest version preceding its ordinal. The compiler rejects self-final reads, dependency cycles, cross-rig writes, structural property targets, duplicate targets after canonicalization, and incompatible multi-target types. A structural transaction is atomic: the previous epoch remains publishable until all affected chains, taps, prepared requests, and Hydra reverse bindings compile successfully.

ExecUsdCacheView cannot outlive its system or request. RigExecSnapshot therefore copies small values and retains immutable native typed values such as VtVec3fArray, VtFloatArray, and VtIntArray before crossing threads; it never converts them into a public RigExec geometry carrier.

## 6.2 Character and region partitioning

**Status:** partition policy **BUILD** on the existing request-pruned network. OpenExec does not mandate one graph or system per character.

Each RigExecRig declares one character partition. Optional regions—body, face, cloth, hair, proxy/render geometry—form dependency-constrained subpartitions. Validation forbids a direct dependency cycle across character roots. A shot-level coordinator implements deliberate cross-character constraints as an explicit acyclic multipass:

pass 0: independent character poses  
pass 1: cross-character contact targets  
pass 2: affected character correction and movement

LibEE’s [six-character example](https://www.oreilly.com/library/view/multithreading-for-visual/9781482243567/chapter-33.html) and Presto’s [per-model strategy](https://www.multithreadingandvfx.org/course_notes/2015/presto_threading.pdf) are precedent, not proof of one required graph shape. Native USD uses one serialized ExecUsdSystem per stage; standalone may use a system per instance under a global budget. Packed frame ranges cross solver boundaries, while semantically addressable intermediates remain published.

## 6.3 Caching, invalidation, and time

**Status:** computation caches/time interval invalidation **EXISTS**; publication residency, prediction, and eviction **BUILD**.

OpenExec owns value validity. ChangeTime() re-resolves time-dependent inputs, detects which actually changed, invalidates dependents, and notifies interested requests. In the USD deployment, stock OpenUSD 26.08 evaluates eligible authored scalar splines through Ts and ordinary time samples through USD’s held/linear interpolation before computations consume the resolved value. The runtime contains an EfPageCacheStorage\<EfTime\> for time-varying values ([runtime source](https://github.com/PixarAnimationStudios/OpenUSD/blob/v26.08/pxr/exec/exec/runtime.cpp), [cache API](https://openusd.org/dev/api/class_ef_page_cache_storage.html)). RigExec replaces neither USD value resolution nor the OpenExec cache.

RigExecFrameResidencyCache stores published immutable snapshots under (definition hash, instance, external dependency-version digest, internal PointBased-provider-set generation/value digest, complete RigExecEvaluationIdentity, BindingEpoch). RigExecTimeKey inside the evaluation identity is the full numeric/PreTime identity; equal numeric payloads with different tags never alias. The identity also distinguishes every transient override set and any shape-preserving runtime complexity selector not already fixed by the profile or epoch. Point-motion dependencies include every source path/role/read phase, points, velocities, accelerations, and timeCodesPerSecond. Affected-time intervals evict only intersecting entries; the external source revision rejects stale publication without being advanced by an owned provider update. A segmented LRU protects explicitly requested frame/render samples and evicts film geometry before pose snapshots. Phase 0 calibrates provisional limits of 2 GiB per desktop hero, 25% of physical memory globally, and 256 MiB mobile; no cache repeats materialization, while an unbounded cache is unsafe.

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

The coordinator distinguishes externalSourceRevision from internalProviderGeneration. Authored/root/session edits outside the owned generated layer advance the external revision. A §7.2 inputs:resolvedPointBased:\<role\> write advances only the internal provider generation; its expected notice is recognized by owned layer/path/property identity, delivered to ExecUsdSystem so dependents invalidate, and suppressed from recursively enqueueing external work or making its own request stale. Render preflight first computes an immutable pointBasedInputBatchDigest over two complete ordered sets: for each authored-base leaf, (canonical native PointBased property address, role, base phase, exact Sdf type, absolute sample time, stock-materialized native value-or-block); for each moved dependency, (canonical native source property address, role, declared preceding/final phase, resolved passive chain-head public address, consuming ordinal when applicable, chain-definition hash). A moved result value is not available or hashed during preflight. The digest also includes the complete render-request RigExecEvaluationIdentity context—including profile, common base time/sample set, interpolation policy, canonical override set/digest, and any independent shape-preserving complexity selector—source dependency versions, and timeCodesPerSecond. Generated slot paths are runtime wiring and never enter this durable identity. Each serialized sample work item records that batch digest and the exact provider subgeneration/value digest installed for every authored-base dependency used by its compute; moved dependencies are validated by their compiled chain key and the sample generation. The sample is accepted only while those subgenerations and chain keys are current; after capture, later base-slot installs may replace the slots without invalidating the already retained sample. Final multi-sample publication checks completeness plus the unchanged external revision, binding epoch, and batch digest—not equality with the last current provider slots. If any real external edit arrives before publication, the external revision changes and rejects the entire batch. Standalone uses the same separation between host-scene revision, selected generic provider-state rows, moved chain keys, override context, and immutable batch digest.

OpenExec exposes no public async/cancellation call. RigExec coalesces unstarted duplicate work and discards a single-sample compute whose externalSourceRevision, expected provider subgenerations/value digest, complete RigExecEvaluationIdentity, or BindingEpoch is stale. A completed render batch instead uses the external revision/epoch/pointBasedInputBatchDigest completeness rule above. Explicit batch/render-sample work uses isolated standalone systems only where the source snapshot is immutable. CPU callbacks target 2 ms p95, and long custom callbacks fail validation.

## 6.5 Vectorization and kernel execution

OpenExec/VDF vectorization means values can contain multiple uniquely identifiable elements flowing over one connection, with masks/ranges and contiguous schedule access. It does not promise CPU SIMD, a GPU backend, or device residency. For a USD array property, RigExec attribute expressions consume and write stock VDF vectors of the property’s scalar element type; an ordinary attribute extraction later materializes the exact native VtArray. Their private implementation may use:

  - direct SIMD over the stock contiguous VDF element storage, preferred when it meets the target;
  - optional ephemeral SoA scratch for a fused kernel segment, written back to the stock VDF element vector before the callback ends;
  - 64-byte-aligned scratch blocks and 256–2,048-point strip sizes selected by benchmark;
  - scalar reference kernels;
  - platform SIMD kernels (AVX2/AVX-512 where available, ARM NEON);
  - optional GPU kernels only after an entire compatible mover-chain segment can stay resident.

No private layout may appear in ExecTypeRegistry, taps, snapshots, .rigpack, exported headers, extension manifests, or Hydra. Transfer boundaries can erase callback gains, so GPU eligibility is a mover-chain property, not a per-node badge. A steady-state benchmark rejects any implementation that retains a second full geometry representation.

## 6.6 Determinism and failure behavior

  - Composed child order and canonical mover/target addresses are part of the compiled asset hash; moves target-list order is not.
  - Every compiled matrix-mover hash includes the canonical mover and points-target paths, exact transform-provider/read-phase identity, weight-object path and descriptor, and post-order ordinal. Every structural weight-descriptor hash includes its canonical target, domain, representation, logical count, sorted sparse support, range policy, operation kind, and declared provider identities; current values and sparse default are value generations, not layout identity. Relationship target order is excluded.
  - Reductions use a fixed tree; no result depends on unordered-container iteration.
  - CPU reference math uses double for frame/solver state and float for bulk geometry unless a schema requests double.
  - Degenerate frames return points plus a status flag; they do not silently emit identity.
  - Cycles are schema validation errors. Current OpenExec cycle detection reports an error and affected results may be wrong until the cycle is removed, so RigExec never treats a cycle as a solver.
  - A disabled mover returns its preceding revision without error. A mover with a non-finite/non-affine matrix, descriptor mismatch, non-finite weight, or strict range violation fails that application atomically, returns the preceding revision with MoverFailed status, and never publishes a partial point update. NaN/Inf in a non-mover solver/provider instead poisons only its dependent branch, records the first canonical public value address, and uses the configured last-valid-generation or error-guide publication policy.
  - A debug mode hashes outputs across repeated evaluation and worker counts to detect hidden state/races.

## 6.7 Execution granularity alternatives

**Alternative A — one computation per scalar channel/joint operation.** Maximum raw visibility, but excessive topology, scheduling overhead and weak vectorization.

**Alternative B — one callback for the entire character.** Low graph overhead, but destroys sparse invalidation, critical-path parallelism, reuse and intermediate extraction.

**Chosen hybrid.** Pack coherent rig values across a solver and process one exact native property value per mover application; expose each semantically meaningful property revision as an addressable result. Same-target writers form explicit serial edges, while unrelated targets remain parallel. Benchmarks, not aesthetic node counts, set the lower boundary. This matches the authoring/evaluation separation documented by [LibEE 2](https://research.dreamworks.com/wp-content/uploads/2018/08/talk_libee2-Edited.pdf) and OpenExec’s compilation model.

# 7\. Mover pipeline: mesh and curve I/O

**Status:** **BUILD**. OpenExec transports/caches typed results; it does not ship a production geometry-mover library. This hierarchy-ordered pipeline is the vectorized geometry-operation facility required by R4.

## 7.1 Native geometry property contract

There is no RigExec geometry value. Stock OpenUSD schemas and properties are authoritative at every durable and public boundary. “Execution type” below means the scalar type carried in stock VDF data flow; “extraction type” is the normal native value returned to USD/RigExec clients:

|  |  |  |  |
| :-: | :-: | :-: | :-: |
| \*\*Native schema/property\*\* | \*\*Exact Sdf type\*\* | \*\*Execution / extraction type\*\* | \*\*v0.1 policy\*\* |
| UsdGeomPointBased.points | point3f\\\[\\\] | GfVec3f elements / VtVec3fArray | Primary movable local-space point property for Mesh, Points, and BasisCurves; public cardinality is fixed for the epoch. |
| UsdGeomPointBased.velocities | vector3f\\\[\\\] | GfVec3f elements / VtVec3fArray | Standard source-motion input only. Whenever a BindingEpoch owns/publishes final points, v0.1 blocks the complete upstream derivative entry regardless of pass-through or numeric equality. |
| UsdGeomPointBased.accelerations | vector3f\\\[\\\] | GfVec3f elements / VtVec3fArray | Standard source-motion input only and meaningful only with valid velocities; structurally blocked with velocities for every RigExec-owned final points leaf. |
| UsdGeomPointBased.normals | normal3f\\\[\\\] | GfVec3f elements / VtVec3fArray | Optional flat property with standard interpolation metadata. It is not a generic primvar; primvars:normals takes precedence when both exist. |
| UsdGeomBoundable.extent | float3\\\[\\\], exactly two values | GfVec3f elements / VtVec3fArray | Separate derived local-space property, recomputed from final points and authoritative widths where applicable. |
| UsdGeomPoints.widths, ids | float\\\[\\\], int64\\\[\\\] | float / VtFloatArray; int64\\\_t / VtInt64Array | Independent standard built-ins. Built-in width is flat object-space diameter; IDs and width cardinality follow the stock Points contract. A composed primvars:widths takes stock precedence. |
| UsdGeomCurves.curveVertexCounts, widths | int\\\[\\\], float\\\[\\\] | int / VtIntArray; float / VtFloatArray | Standard curve topology and flat built-in width. Topology is input-only; a width writer targets the winning width property separately and passes stock interpolation-size validation. A composed primvars:widths takes stock precedence. |
| primvars:widths on Points/Curves | float\\\[\\\] plus optional int\\\[\\\] :indices | float elements / VtFloatArray | An actual UsdGeomPrimvar; it may use constant or other stock interpolation and flat/indexed representation, including inherited resolution, and wins over built-in widths under stock UsdImaging precedence. Its public representation is never flattened by RigExec. |
| UsdGeomBasisCurves.type, basis, wrap | token, token, token | TfToken | Input-only curve policy; never changed during evaluation. |
| UsdGeomPrimvar value and optional :indices | exact authored SdfValueTypeName; int\\\[\\\] indices | scalar or scalar-element VDF type / corresponding native scalar or VtArray; VtIntArray indices | The primvar must already exist with fixed type, interpolation, element size, role, color space, and flat/indexed representation for the epoch. |

  

The complete Mesh structure consumed and forwarded by v0.1 is equally explicit:

|  |  |  |
| :-: | :-: | :-: |
| \*\*Stock owner\*\* | \*\*Exact properties and Sdf types\*\* | \*\*Policy\*\* |
| UsdGeomMesh topology | faceVertexCounts int\\\[\\\], faceVertexIndices int\\\[\\\], holeIndices int\\\[\\\] | Input/forward only; never mover targets. |
| UsdGeomMesh subdivision tags | cornerIndices int\\\[\\\], cornerSharpnesses float\\\[\\\], creaseIndices int\\\[\\\], creaseLengths int\\\[\\\], creaseSharpnesses float\\\[\\\] | Input/forward only; cardinality validated by stock Mesh rules. |
| UsdGeomMesh subdivision policy | subdivisionScheme token, interpolateBoundary token, faceVaryingLinearInterpolation token, triangleSubdivisionRule token | Input/forward only and epoch-structural. |
| UsdGeomGprim render policy inherited by Mesh, Points, and BasisCurves | orientation token, doubleSided bool | Forward unchanged; never inferred from movement. Standard UsdGeomSubset prims and their properties also pass through unchanged. |

  

Stock 26.08 deliberately rejects VtArray\<T\> as a plugin-computation input or result type ([computation builders](https://github.com/PixarAnimationStudios/OpenUSD/blob/v26.08/pxr/exec/exec/computationBuilders.h), [type registry](https://github.com/PixarAnimationStudios/OpenUSD/blob/v26.08/pxr/exec/exec/typeRegistry.h)). Its built-in array-attribute definitions instead use SdfValueTypeName::GetScalarType() for VDF data flow and the exact array TfType for extraction. RigExec follows that contract exactly: point3f\[\], normal3f\[\], and float3\[\] all flow as GfVec3f element vectors, while the target catalog and passive extraction attribute retain their distinct exact Sdf roles. No native-geometry or VtArray computation type is project-registered.

Native geometry prims carry no RigExec applied geometry API, custom property manifest, custom topology, or project geometry metadata. Each mover schema statically declares every exact standard property it reads; the compiler binds those property providers before OpenExec system construction. A computed primvar is an already-authored ordinary primvars:\<name\> property, including its standard optional indices property, not a dynamically created RigExec field.

Each point chain stays in the target prim’s local space. A cross-prim driver supplies a separately compiled GfMatrix4d input derived from standard UsdGeomXformable transforms; for RigExecMatrixMover, the final declared transform already maps target-local points to target-local points. No geometry value carries source/output-space matrices or provenance.

Array-property attribute expressions write only the stock scalar-element VDF vector. A separate computeMoverStatus validates parameters and reports success, disabled, or MoverFailed; the expression consumes that scalar status and uses VdfContext::SetOutputToReferenceInput() to preserve the preceding vector exactly when disabled or invalid. Kernels are required to be total after validation. An unexpected infrastructure exception rejects the whole generation rather than manufacturing a custom {geometry,status} result.

Weighting remains independently typed rig data:

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

RigExecWeightDescriptor is shape-only and frozen for the epoch, including the canonical sorted sparse-index layout but excluding numeric values and the sparse default. RigExecWeightPacket is the common immutable value returned by both static and dynamic weight prims. constant stores zero packet values and broadcasts the packet default; dense stores exactly logicalCount values and requires canonical default zero; sparse stores one packet value per descriptor index plus the packet default for all other elements. Logical elements are target elements, never tuple components. dirtyElements is conservative: a constant/default edit or any dynamic driver/scale/bias change dirties the full logical domain; a dense edit may carry known changed ranges; a sparse explicit-value-only edit may carry the ranges named by its fixed support. Internal ranges may reduce evaluation work, but Hydra still dirties the complete published points leaf.

All callbacks are pure: inputs are immutable; outputs are new or copy-on-write; pool reuse cannot change logical results.

## 7.2 Target-chain compilation

At structural compile, RigExec discovers each exact moves target, performs §4.2’s composed post-order traversal, and emits a pure exact-property application for every (mover,target). For ordinary properties, the base is the unchanged stock attribute computeValue. The compiler also resolves the declared read phase of every UsdGeomPointBased.points dependency reachable from the prepared graph—destination bases, native blend samples, lattice cages, curve/ribbon drivers, surface drivers, and any plugin-declared PointBased side input. Each distinct **authored base** source leaf gets one predeclared exact point3f\[\] inputs:resolvedPointBased:\<role\> slot. A dependency bound to a moved preceding or final phase connects directly to that phase’s existing passive chain output at each shutter time; it is never stock-materialized or double-sampled. That moved chain’s own authored base leaf is materialized once through its base slot.

Before any requested frame or render batch executes, the coordinator calls stock UsdGeomPointBased::ComputePointsAtTime() or ComputePointsAtTimes() outside OpenExec for every authored-base source leaf and retains the returned exact native arrays. Immediately before each serialized ChangeTime/compute, it writes that sample’s complete base-slot set into the current default/value slots inside one owned change block and delivers ordinary value invalidation. It never authors a generated time-sample map, so exact and PreTime requests at the same numeric payload cannot collide. This is the concrete injection path for authored velocities/accelerations on every native PointBased base input; a VtArray override is never passed to ComputeWithOverrides. The request identity includes every source path/role/read phase, points, velocities, accelerations, stage timeCodesPerSecond, common base time, and complete sample set. Every intermediate and final result retains its exact Sdf type. The public address is the exact property path plus phase and optional mover ordinal, while hidden provider paths remain compiler details. The example arm’s point chain uses its base role and lowers to:

/ArmAsset/Geom/ArmBody.points \[stock-resolved base point3f\[\]; VDF GfVec3f elements\]  
  → BicepFlex@/ArmAsset/Geom/ArmBody.points \[point3f\[\]; VDF GfVec3f elements\]  
  → WristMatrix@/ArmAsset/Geom/ArmBody.points \[point3f\[\]; VDF GfVec3f elements\]  
  → ElbowMatrix@/ArmAsset/Geom/ArmBody.points \[point3f\[\]; VDF GfVec3f elements\]  
  → ShoulderMatrix@/ArmAsset/Geom/ArmBody.points \[point3f\[\]; VDF GfVec3f elements\]  
  → RibbonWrap@/ArmAsset/Geom/ArmBody.points \[point3f\[\]; VDF GfVec3f elements\]  
  → VolumeCorrect@/ArmAsset/Geom/ArmBody.points \[point3f\[\]; VDF GfVec3f elements\]  
  → /ArmAsset/Geom/ArmBody.points \[final extraction VtVec3fArray\]

Each array application owns exactly two hidden **chain-output** attributes of the exact target SdfValueTypeName, both generated only in the pruned session namespace or standalone scene database. An operation-specific host may also own the side-input attributes frozen by its registered signature:

1.  outputs:expression owns exactly one connection whose target is the preceding exact-typed attribute and has a schema-registered AttributeExpression. Its registration result is the scalar element type— GfVec3f for point3f\[\] —and it declares Connections\<ElementT\>(computeValue).InputName(previous).Required(). The callback reads that named vector with VdfReadIterator\<ElementT\>, allocates/writes the result with stock VDF vector APIs, and references the prior input directly for disabled/failure pass-through.
2.  outputs:value has exactly one same-typed connection to outputs:expression and no expression. Clients request this passive attribute’s built-in computeValue. Stock computeConnectedValue keeps the VDF result type at ElementT but selects the full array type from the provider attribute for extraction, yielding the exact native VtArray\<ElementT\>.

The passive bridge is mandatory in unchanged 26.08. A direct request of the plugin expression would use the plugin result type for extraction, because stock Exec\_PluginComputationDefinition does not let a plugin declare a different extraction type ([definition contract](https://github.com/PixarAnimationStudios/OpenUSD/blob/v26.08/pxr/exec/exec/computationDefinition.cpp), [connected-value implementation](https://github.com/PixarAnimationStudios/OpenUSD/blob/v26.08/pxr/exec/exec/builtinAttributeComputations.cpp)). RigExec neither patches that behavior nor calls private definitions. It uses only the public AttributeExpression, Connections, ordinary single USD attribute connections, ExecUsdValueKey(outputs:value), and built-in computeValue surface. Prepared internal Exec keys for the next application and final extractor address {outputs:value, computeValue}; a tap retains only the canonical target/type/phase(/mover) address, which the private catalog resolves to that bridge. outputs:expression is internal and never tap-addressable. Before OpenExec compilation, RigExec rejects a missing, multiple, wrong-type, or wrong-role connection on either chain-output attribute rather than accepting the built-in resolved-value fallback. The exact Sdf role is checked in the catalog because stock connection compatibility alone cannot distinguish point3f\[\], normal3f\[\], and float3\[\], whose C++ element/container types coincide. The same output attributes and connections are exposed by the standalone Esf adapter, so both deployments compile the same stock behavior.

Normals and extent are independent chains. The explicitly authored RecomputeNormals mover consumes the latest preceding point element vector plus standard topology and writes an exact normal3f\[\] application chain; the explicitly authored RecomputeExtent mover targets the existing float3\[\] extent, consumes final local points and the standard winning-width policy where applicable, and writes its exact chain. Mesh/curve topology and unrelated properties bypass the point chain unchanged. Each application receives the preceding vector, its statically declared side properties, computeMoverStatus, and inputs:enabled. §4.2 defines fan-out, cycles, and conflicts. Base, every post-mover revision, and final are cacheable/addressable only through their canonical target/type/phase(/mover) identities; the catalog alone maps them to private passive bridges. Fusion must preserve each requested semantic revision, and a private bridge can reference the expression output without copying.

## 7.3 Blend shapes

For dense targets, the independently composed channel weights first produce the full-strength candidate:

ri=pi+Σk κk(wk)dk,i.

The common MoverAPI envelope then produces the mover result:

pi′=pi+ei(ri−pi).

dk,i is derived from a native target shape as targetPoints\[k\]\[i\] - basePoints\[i\], α\_k is the target/in-between activation curve used by κk, and ei is the common envelope selected by §4.1: a bound compatible weight object's value for point i, otherwise inputs:defaultWeight broadcast. The bound object supersedes rather than multiplies the fallback attribute. This is activation-space, non-temporal interpolation; USD remains the sole time-interpolation authority. Activations are finite, strictly positive, and unique; an implicit zero-delta sample exists at activation 0. Every sample is a stock mesh/points/basis-curves points property with matching cardinality and target-local coordinates. Samples compile in (activation, canonicalPath) order, interpolate piecewise linearly, and clamp the channel weight to \[0,lastActivation\] in v0.1. Active inputs accumulate in canonical input-path order. RigExecBlendInput.inputs:weight is the channel amplitude wk, not a second mover envelope. Channel weights may be authored, connected, or pose-computed. Normals are handled by their separate native property chain. Input/sample membership, activation, sample property path, or target-shape cardinality edits are structural; compatible channel-weight, native sample-points, inputs:defaultWeight, and weight-packet changes are value-only.

## 7.4 Weighted matrix movement

RigExecMatrixMover consumes one affine matrix T(t) and the common total envelope wi(t) for the exact native points target. wi(t) comes from a bound compatible weight object's packet when present and otherwise from inputs:defaultWeight broadcast; the two sources are never multiplied. If qi is point i from the preceding mover revision in target-local space, the normative column-vector reference result is:

pi′=qi+wi(t)(T(t)qi−qi),0≤wi(t)≤1.

The GfMatrix4d implementation is accepted only when transforming conformance points matches that equation, avoiding row/column storage assumptions. A weight of exactly zero returns the preceding point bit-for-bit; one applies the complete affine transform. Intermediate weights linearly blend the input and transformed point, including translation. T(t) must be finite, affine, and already map the target prim’s local space to that same local space; singular affine maps are legal because collapsing points can be intentional. A transform-provider prim resolves through its catalogued computeMatrix; an exact matrix property resolves through its declared base, preceding, or final phase. When a joint or driver is expressed in rig-common space, a separate matrix-math provider combines its result with standard UsdGeomXformable transforms before the matrix reaches the mover. RigExecMatrixMover writes only the native points value; normals and tangents require an explicitly ordered later native-property operation, and extent derives separately from final local points.

There is deliberately no cross-mover normalization, transform palette, inverse-bind table, simultaneous multi-transform reduction, or atomic binding aggregate. Multiple matrix movers apply the equation sequentially in §4.2’s bottom-up hierarchy order, so in general they are order-dependent and do not claim equivalence to a simultaneous weighted-transform blend. Each mover independently resolves its common envelope; adding or replacing one transform, changing its defaultWeight, or binding/retargeting its weight object changes only that authored operation and its downstream chain. Because all point results stay target-local, there is no geometry finalizer or hidden bind-space conversion.

The weight-object descriptor is epoch-static and contains the canonical weight target, domain, representation, cardinality, sorted sparse indices, and policy. Its hash covers the full provider identities retained collision-authoritatively in RigExecWeightBindingRecord and the BindingEpoch; the digest never replaces those records for equality or routing. Its RigExecWeightPacket contains the current immutable scalar values, sparse default, and dirty ranges. When an object is bound, the compiler requires its canonical weight target and the matrix-mover points target to match and requires exactly one finite effective weight per point. When none is bound, the compiler synthesizes that total field by broadcasting the finite normalized inputs:defaultWeight. Static dense/sparse fields, both scalar broadcast paths, dynamic modulation, values at zero/one, non-identity matrices, reflections, scale/shear, and sequential non-commuting transforms all have scalar-reference, SIMD, USD, and standalone goldens.

## 7.5 Lattice, curve/ribbon, and surface movement

**Lattice.** Bind coordinates (u,v,w) address a regular B-spline or Bernstein cage. Evaluation is the tensor product:

p′(u,v,w)=a,b,cNa(u)Nb(v)Nc(w)Cabc.

C comes from a native mesh/points points property or from point frames; basis/order/domain policy is authored on the mover.

**Curve/wire/ribbon.** At bind time, each driven point stores curve coordinate u, transverse coordinates (v,w), and optionally a scale reference. The built-in ribbon mode used by §4.5 accepts an ordinary two-component UsdGeomPrimvar st=(u,v) and defines the missing third coordinate as w=0; a three-component ordinary primvar may provide (u,v,w) explicitly. Evaluation computes curve position c(u) and a rotation-minimizing frame (t,n,b), then:

p′=c(u)+vn(u)+wb(u).

Twist is unwrapped along arc length; closed curves store seam/holonomy correction. The same mover can write a native BasisCurves.points property, and its driver may be another solver result.

**Surface.** v0.1 reads the native mesh points, faceVertexCounts, and faceVertexIndices, binds polygon triangle ID plus barycentrics, and forms runtime derivative frames. Sliding closest-point is separate. Subdivision/NURBS drivers require explicit patch evaluators and are deferred.

Meshes, curves, and frames can drive one another. Every source, intermediate, and final geometry value remains an exact native property value; §4.5 supplies the curve result and Phase 3 a Points fixture.

## 7.6 Post-movement and derived native properties

Post operations include:

  - volume/segment-length correction;
  - Laplacian or delta-mush smoothing with fixed adjacency;
  - collision/projection against an input proxy surface;
  - normal and tangent recomputation;
  - extent calculation;
  - user native kernels with exact predeclared standard-property inputs/outputs.

Every post operation that changes a value is an ordinary mover with an exact rel rigExec:moves target and the common MoverAPI envelope from §4.1. Smoothing and volume-restoration kernels compute a full-strength candidate and do not own a separate strength attribute; inputs:defaultWeight therefore defaults them to full effect, while zero is exact pass-through. RigExec never injects an implicit geometry finalizer: a Mesh, Points, or BasisCurves output that needs recomputed normals, widths, or extent must pre-author the native output property and explicitly order the corresponding mover in the composed hierarchy.

All derived geometry remains ordinary stock properties. Built-in points, normals, velocities, accelerations, and widths are flat arrays and are never treated as indexed values; only a real UsdGeomPrimvar may carry a separate :indices property. If both a built-in and composed local/inherited primvars:normals or primvars:widths property exist, stock UsdImaging primvar precedence is preserved and RigExec compiles/publishes only the winning representation ([Points contract](https://github.com/PixarAnimationStudios/OpenUSD/blob/v26.08/pxr/usd/usdGeom/points.h), [Curves contract](https://github.com/PixarAnimationStudios/OpenUSD/blob/v26.08/pxr/usd/usdGeom/curves.h)).

Normal publication is representation-specific and conservative:

  - On a UsdGeomMesh whose composed subdivisionScheme is not none, moved output publishes neither terminal normals nor primvars:normals; the renderer derives subdivision normals from final points. Export blocks/omits a stale authored terminal normal opinion.
  - On a polygonal mesh, absence of authored normals preserves stock faceted behavior. If explicit normals are required, an explicitly ordered native-normal operation must produce the final normal3f\[\] property or existing normals primvar with its stock interpolation/cardinality. Inverse-transpose is valid only when the complete point operation is one spatially constant, nonsingular, positive-determinant affine mapping. Reflection, a singular map, varying per-point weight, blending, lattice/surface/curve movement, smoothing, or any other nonlinear operation invalidates that shortcut and requires a topology-aware rebuild. Failure to produce finite correctly sized normals prevents that result generation from publishing.
  - UsdGeomPoints normals are optional orientation data. A qualified proper-affine operation may transform them; otherwise a point mover must have an explicit operation-specific orientation rule or the stale normals are blocked/omitted. There is no topology-derived fallback for a point cloud.
  - UsdGeomBasisCurves without normals retains the stock tube-style interpretation. Curves with normals retain their ribbon interpretation only when an explicitly ordered transported-normal operation produces finite native normals with the required stock interpolation/cardinality. A nonlinear point move or a degenerate transported frame cannot silently change a ribbon into a tube; the invalid generation is rejected or retains the preceding accepted generation under the publication policy.

Width is a separate exact native property chain: changing points never implicitly scales it. The compiler selects the stock-precedence winner between built-in widths and primvars:widths; a renderer-facing width mover that targets the shadowed representation fails binding instead of appearing to succeed invisibly. A winning indexed primvar remains indexed at taps/export/Hydra. Only the extent computation uses stock UsdGeomPrimvar::ComputeFlattened() plus standard interpolation-size expansion in transient scratch: constant widths broadcast transiently to point count for UsdGeomPoints::ComputeExtent, while curves use their stock interpolation/cardinality rules. Invalid indices, unsupported interpolation, or a flattened/expanded count mismatch rejects the generation; no persistent flattened width property is created.

extent is an independent two-element float3\[\] result recomputed from final prim-local points and the authoritative winning widths where the stock UsdGeomPoints or UsdGeomBasisCurves extent rule uses them; when neither width representation resolves, the stock no-width fallback is preserved. Points, widths, normals, and extent for a render profile are evaluated at the same sample offsets and accepted under the same generation fence. Every computed primvar is likewise its own exact native property computation and tap; none is a view into an aggregate.

## 7.7 Topology and dynamic-output policy

OpenExec computations cannot add stage prims/properties or change scene topology. RigExec therefore supports:

  - fixed-topology movement of preexisting native property values;
  - pre-authored curve/mesh topology selected by variants between epochs;
  - a fixed maximum guide set whose inactive curves use only standard zero widths;
  - baked topology-changing simulation imported as time-sampled geometry outside the live rig.

Mesh or curve topology, point cardinality/identity layout, subdivision policy, primvar type/interpolation/element size/indexing, or output-property membership changes begin a new epoch. UsdGeomMesh::ValidateTopology() and stock basis-curve interpolation-size helpers validate the composed inputs. No mover may target topology or descriptor metadata. Strand birth/death, remeshing, and fracture stay outside the live evaluator; a future procedural scene index may produce topology before RigExec, never inside a mover callback.

## 7.8 Mover architecture alternatives

**Alternative A — an explicit stack relationship or one monolithic target callback.** A list is locally obvious and a monolith maximizes fusion, but both duplicate namespace intent, broaden invalidation, and hide intermediate values.

**Alternative B — composed bottom-up mover hierarchy with one immutable target application per mover. Chosen.** It makes standard USD hierarchy the order authority and preserves sparse invalidation, taps, and plugin composition. A compiler may fuse proven-compatible adjacent applications behind the same public result contract.

**CPU versus GPU.** Scalar reference kernels and CPU SIMD are mandatory v0.1 correctness and shipping paths. GPU mover-chain computation is optional and profile-driven; it is never required to pass the CPU release gates. When enabled, an entire compatible chain segment must remain resident, then read back once before renderer-neutral Hydra publication; otherwise execution falls back to CPU with telemetry. Hydra render delegates may use their normal GPU rendering paths independently. Zero-readback RigExec publication is future renderer-specific work and cannot satisfy R6 by itself.

# 8\. Evaluation and publication coordination

**Status:** synchronous OpenExec evaluation/invalidation **EXISTS**; RigExec queueing, generation fencing, residency, and publication coordination **BUILD**.

## 8.1 Scope boundary

This layer coordinates data and rendering infrastructure only. It does not define hit testing, manipulators, gesture state, viewport controls, animation editing, undo, pose libraries, or tablet editing. Those products may later consume the same value addresses, overrides, and snapshots without changing the contracts below.

## 8.2 Generation-fenced flow

**Sequence:** USD change/time or pack sample request → Evaluation queue: external revision + complete evaluation identity

Evaluation queue → Evaluation queue: coalesce equivalent unstarted work

Evaluation queue → Exec coordinator + VDF: install owned provider generation; serialized ChangeTime/Compute

Exec coordinator + VDF → Snapshot publisher: snapshot + external/provider revisions + identity/epoch

Snapshot publisher → Snapshot publisher: reject stale or incomplete generation

Snapshot publisher → Hydra notice owner: dirty precise standard output locators

Hydra notice owner → Hydra notice owner: observers pull cached data sources

Evaluation always completes before publication. A one-sample snapshot becomes visible only when its externalSourceRevision, expected internal PointBased-provider subgenerations/value digest, complete RigExecEvaluationIdentity, and BindingEpoch match at capture. A render collection becomes visible only when every required offset was captured under the same immutable pointBasedInputBatchDigest—which includes the complete override/complexity context—and the external revision/epoch remain current; it does not compare earlier samples with provider slots subsequently installed for a later offset. Hydra pulls never compute or wait.

## 8.3 Queue and work ownership

Current ExecUsdSystem is synchronous and has no public cancellation contract. RigExecEvaluationQueue therefore:

  - replaces an unstarted equivalent item only for the same (externalSourceRevision, character, complete RigExecEvaluationIdentity, BindingEpoch);
  - allows in-flight work to finish, then rejects it if any generation key is stale;
  - batches compatible character/profile requests while leaving node parallelism to VDF;
  - admits only explicitly requested frame or render-motion samples; it does not predict user behavior;
  - yields outer scheduling slots under a global work budget so render/publication work remains bounded.

## 8.4 Explicit-time evaluation and residency

For native USD, a cache miss captures externalSourceRevision, materializes every required §7.2 stock-computed PointBased dependency for the complete renderBaseTime/sample-set identity, installs the sample’s complete slot set under new owned internalProviderGenerations, reconstructs the exact UsdTimeCode—numeric or PreTime—from RigExecTimeKey, performs one serialized ExecUsdSystem::ChangeTime(usdTimeCode), evaluates the prepared profiles, and publishes one complete generation fence only if the captured external revision remains current. Timed attribute inputs use stock USD value resolution, including native Ts spline evaluation where authored. For standalone packs, the corresponding complete empty-override RigExecEvaluationIdentity and every required generic resolved slot must be exported unless the host supplies an explicit complete resolved-state override under its own nonempty identity; the pack performs no curve interpolation.

The residency cache may return an exact matching generation immediately. Misses evaluate only the requested profiles. Structural/value invalidation evicts intersecting generations using OpenExec’s affected-time information and the RigExec dependency digest.

## 8.5 Deferred consumers

Direct manipulation, undo/commit transactions, playback controls, scrubbing policy, animation-editor curve UX, and product-facing diagnostics are deferred. The atomic change-delivery batch in §11 is an evaluator input mechanism with no editing-workflow semantics. No current schema, performance gate, phase exit criterion, or renderer contract depends on these deferred consumers.

# 9\. Introspection and extraction API

**Status:** OpenExec keys/requests/cache views and DOT graphing **EXISTS**; the uniform typed tap/snapshot API **BUILD**.

## 9.1 Backend-neutral addresses

struct RigExecValueAddress {  
    SdfPath target;               // canonical prim or exact property path  
    SdfValueTypeName usdType;     // exact for properties; empty for non-USD rig values  
    TfToken publicComputation;    // non-property rig value only; empty for exact USD properties  
    TfType resultType;            // expected C++ extraction type  
    TfToken phase = RigExecTokens-\>final; // base, afterMover, or final  
    std::optional\<SdfPath\> mover; // required for an afterMover revision  
    std::optional\<int\> element;   // index into a RigExec-owned aggregate after extraction  
};  
  
// Compiler-private; never returned, serialized, traced externally, or hashed as a public address.  
struct RigExecResolvedExecKey {  
    SdfPath provider;  
    TfToken computation;  
    uint64\_t bindingEpoch;  
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

The USD implementation resolves the public canonical target/type/phase(/mover) address through RigExecComputationCatalog, obtains a private RigExecResolvedExecKey, converts that to ExecUsdValueKey, and reads VtValue results; upstream keys carry no expected type. publicComputation is present only for non-property rig values such as computePointFrame; an exact USD property leaves it empty and the catalog privately selects the required stock computeValue bridge. The **BUILD** catalog keys owner schema, canonical target, exact SdfValueTypeName, phase, and public computation where applicable and validates every adapter. Neither provider nor the private computation token is returned by RigExecTap::GetAddress(), serialized in a pack, or emitted in an external trace. A geometry prim shorthand may canonicalize only to stock UsdGeomPointBased.points; primvars and every other geometry value use an exact property path. A primvar’s value and optional :indices are separate addresses. Shorthand addresses default to final; base/afterMover are explicit. Standalone uses the same public catalog IDs and its own private resolved keys.

Snapshot access is immutable and generation-aware:

const RigExecPointFrame& frame = snapshot.Get(frameTap);  
GfMatrix4d matrix = snapshot.Get(matrixTap);  
VtVec3fArray bodyPoints = snapshot.Get(bodyPointsTap);  
VtVec3fArray bodyNormals = snapshot.Get(bodyNormalsTap);  
VtVec3fArray bodyExtent = snapshot.Get(bodyExtentTap);  
VtValue custom = snapshot.Get(untypedTap);

## 9.2 Infrastructure consumers

|  |  |
| :-: | :-: |
| \*\*Consumer\*\* | \*\*Tap set\*\* |
| Hydra scene index | visible xforms, points, primvars, extent |
| Cache/export service | matrices, point frames, moved points, normals, arbitrary computed attrs |
| Standalone pack/compiler | resolved authored inputs, manifests, addresses, parity samples |
| Automated validation | golden result addresses, tolerances, invalidation expectations |
| Headless trace export | canonical public value addresses, redacted/canonicalized DOT topology, dependency/timing records |

  

Interactive consumers such as manipulators, motion trails, pose libraries, and graph browsers are deferred. A headless trace bundle may derive topology from OpenExec’s Diagnostics::GraphNetwork() output, but before persistence it maps known values to canonical public addresses and redacts generated paths, private computation tokens, and opaque node IDs; the unredacted DOT exists only in-process as transient internal diagnostics. The bundle does not pretend the opaque schedule or every internal VDF output is a stable public object.

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

The pruning filter has a deliberately narrow predicate: the reserved scope must come from RigExec’s owned anonymous generated layer. It never infers ownership from a third-party rig/binding schema and never removes unrelated authored prims, computations, or renderer data. It applies the same predicate to traversal and live notices: GetChildPrimPaths() omits owned generated paths, and incoming PrimsAdded, PrimsRemoved, and PrimsDirtied entries for those paths are filtered before forwarding; unrelated notice entries pass through unchanged. Output ownership comes only from the compiler’s canonicalized exact rigExec:moves property targets and derived native-property dependencies; geometry prims carry no ownership API.

**Pipeline:** UsdImaging final chain → RigExec internal-prim pruning · (owned generated paths only)  
RigExec internal-prim pruning · (owned generated paths only) → RigExecBindingResolvingSceneIndex · (compiler-produced output bindings)  
RigExecBindingResolvingSceneIndex · (compiler-produced output bindings) → RigExecResultsSceneIndex  
RigExecResultsSceneIndex → HdMergingSceneIndex  
HdMergingSceneIndex → application filters  
application filters → renderer filters / HdRenderer

RigExec builds reverse-mapped mesh, points, curves, xform, and property routing from its compiled records: exact standard property paths and SdfValueTypeNames, mover targets, transform providers/read phases, weight-object targets/descriptors, static or dynamic weight dependencies, native blend-sample points, canonical public tap addresses, output paths, and locator mappings. It never reconstructs geometry semantics from Hydra. Native v0.1 deinstances every live RigExec character before generated providers are authored; prototypes remain untouched and are not computation providers. Sharing is limited to immutable native values/definition hashes, and identical final results may be instanced only after evaluation.

## 10.2 Standard published schemas

The public boundary contains only standard Hydra data:

  - HdXformSchema local-to-parent matrix and preserved resetXformStack;
  - HdPrimvarsSchema flat entries for stock built-ins such as points, normals, widths, velocities, and accelerations, plus flat or indexed entries only for actual UsdGeomPrimvar properties;
  - HdExtentSchema min/max from final prim-local points;
  - unchanged mesh/curves/points topology from the input scene.

Frames convert against evaluated parents. Native geometry property results are already target-local; exact standard primvar metadata and optional indices are preserved. Renderers never call RigExec. Neutral publication is host-resident; GPU work reads back, while zero-readback is future and not R6.

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
| Flat stock value | primvars / \\\<name\\\> / primvarValue | HdSampledDataSource; use the exact typed alias when registered, normally HdVec3fArrayDataSource for points/normals and HdFloatArrayDataSource for widths | RigExec snapshot for each owned stock built-in. points, built-in normals, widths, velocities, and accelerations are never published with indexedPrimvarValue or indices. |
| Actual flat/indexed UsdGeomPrimvar | flat: / primvarValue; indexed: / indexedPrimvarValue and / indices | Exact typed sampled value and, only for indexed primvars, HdIntArrayDataSource | Preserve the authored representation rather than silently flattening or indexing it. |
| Primvar descriptor | / interpolation, / role, / colorSpace, / elementSize under the same primvar | HdTokenDataSource, HdTokenDataSource, HdTokenDataSource, HdIntDataSource | Preserve the standard authored/upstream UsdGeomPrimvar interpolation and element size, the exact Sdf role, and resolved color-space metadata; there is no RigExec descriptor. Stock built-ins retain their standard UsdImaging descriptors. |
| Final local bound | extent / min, extent / max | two HdVec3dDataSource leaves carrying GfVec3d | RigExec snapshot mapped from the final native two-element extent value derived from final local points and standard widths policy. |
| Representation mask | losing flat/indexed leaves, intentionally absent descriptor leaves, stale normals, and stale motion derivatives | HdBlockDataSource | Conditional but required wherever recursive overlay would otherwise expose an incompatible weaker value. Block the complete invalid primvar entry when a weaker child leaf could survive. |
| RigExec-owned-points derivative mask | complete primvars / velocities and primvars / accelerations entries | HdBlockDataSource | Required whenever the BindingEpoch owns/publishes final points, including disabled, zero-weight, failed pass-through, or numerically unchanged frames. Final sampled points are authoritative. |
| Fixed geometry contract | mesh, basisCurves, topology/subdivision/geom-subset children; point prim type | Existing standard upstream containers | Forward unchanged because v0.1 prohibits topology changes during evaluation. |
| Other render state | visibility, purpose, material bindings, display style, double-sided state, instancing, and unowned primvars | Existing standard upstream containers | Forward recursively; RigExec must not replace or omit unrelated renderer inputs. |

  

The only renderer-to-scene-index side channel RigExec interprets is HdSceneIndexCreateArgsSchema. motionBlurSupport is an HdBoolDataSource capability bit: false permits one sample, while true allows the explicit render-preflight sample set in §10.5. It does not communicate shutter offsets. cameraMotionBlurSupport, legacyRenderDelegateInfo, and renderer-specific extension leaves are forwarded unchanged and are not RigExec evaluation inputs. If motionBlurSupport is absent, the application’s explicit render profile is authoritative; a motion-blurred render still fails preflight when its required offsets are unavailable.

RigExecResultsSceneIndex::GetPrim(path) first fetches the upstream prim, retains its primType, maps exact native property results to standard Hydra locators, builds standard containers with HdXformSchema::Builder, HdPrimvarSchema::Builder plus HdPrimvarsSchema::BuildRetained, and HdExtentSchema::Builder, and makes that sparse root the stronger first input to HdOverlayContainerDataSource::OverlayedContainerDataSources(rigExecStrongRoot, upstreamRoot). points maps to flat primvars/points/primvarValue; built-in or primvar normals and widths follow stock UsdImaging precedence; built-in widths remain flat while a winning actual primvars:widths retains its authored flat/indexed representation; only actual indexed UsdGeomPrimvar properties receive indexedPrimvarValue plus indices; and extent maps to extent/min and extent/max. Whenever the epoch owns the final points leaf, the stronger root places blocks at the complete primvars/velocities and primvars/accelerations entries. Container-on-container overlap composes recursively, so an owned value leaf wins while topology, descriptors not replaced by RigExec, and unrelated schemas continue to come from upstream.

Flat versus indexed representation is fixed for a BindingEpoch. For an actual indexed primvar, the overlay blocks a weaker upstream primvarValue; for an actual flat primvar, it blocks weaker indexedPrimvarValue and indices. A stock built-in is always flat. Replaced or intentionally absent descriptor leaves are likewise masked when necessary. A representation change rebuilds the epoch, masks the losing leaves, and sends universal/resync invalidation; it never exposes incompatible flat and indexed representations simultaneously. A stale semantic value is blocked at the strongest complete entry that can prevent weaker descendants from reappearing—for moved points this is the entire velocities and accelerations primvar entries, and for invalid normals it is the complete winning normals entry. No custom RigExec schema crosses the renderer boundary. RigExecResultsSceneIndex creates no computation prims and publishes only standard final-value leaves; unrelated upstream schemas and computations pass through unchanged.

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

Every weight-object value/default/driver edit follows the compiler’s reverse edges from that object through each registered consumer to the consumer’s exact outputs. An inputs:defaultWeight value edit follows the owning mover's application edges to the same outputs; it remains an ordinary value edit even while ignored behind a bound weight object, so removing that binding later exposes the already-composed fallback without changing its authored value. For matrix/blend point consumers this reaches points and, when declared, downstream normals/tangents and extent; for a transform consumer it reaches xform/matrix; for scalar or plugin-property consumers it reaches the exact tap and any catalogued Hydra output locator, with no Hydra notice when the value is not published. Matrix-provider, native target-shape points, and blend-channel values follow the same consumer-output routing. Adding, removing, or retargeting any mover's common rigExec:weightObject relationship; changing a matrix-mover transform relationship; or changing weight targets, static↔dynamic schema, dynamic relationship or attribute-connection identity, operation kinds, weight domains, canonical sparse support, range policies, representation, cardinality, blend input/sample membership, activation or target-property identity, or output-property sets rebuilds the BindingEpoch and uses universal/resync invalidation. An order-only sparse pair permutation with the same canonical support and values leaves the descriptor/epoch unchanged. Graph topology never changes during evaluation; only a completed structural recompile publishes a replacement epoch.

Typical locator mapping:

|  |  |
| :-: | :-: |
| \*\*Result\*\* | \*\*Dirty locator\*\* |
| posed transform | xform / matrix; preserve/dirty xform / resetXformStack only when it changes |
| RigExec-owned final points | primvars / points / primvarValue; also the complete primvars / velocities and primvars / accelerations entry locators for the epoch-owned structural blocks |
| normals/tangents | primvars / \\\<name\\\> / primvarValue, or primvars / \\\<name\\\> / indexedPrimvarValue plus primvars / \\\<name\\\> / indices |
| computed custom primvar | matching full flat/indexed value path and full indices path when changed |
| final bound | extent / min and extent / max (or the extent container) |
| flat/indexed representation change | block the losing leaves; HdDataSourceLocatorSet::UniversalSet() or add-again resync |
| output-set/binding/topology change | HdDataSourceLocatorSet::UniversalSet(); \\\_SendPrimsAdded again for existing-path resync, or \\\_SendPrimsAdded/\\\_SendPrimsRemoved for real existence changes |

  

Start with the narrowest logical leaf for value changes, then expand it with ComputeDirtyLocators() because v0.1 replaces retained container handles. Use universal dirtiness/resync for representation, topology, binding, or output-set changes. The reverse map is authoritative; HdDependenciesSchema is advisory unless dependency forwarding is installed.

## 10.5 Motion samples

A non-motion evaluation profile publishes one sample. In the v0.1 motion profile, **point motion is explicit-samples-only**: final points are retained at every shutter offset and no final point derivatives are published. Every owned dependent native result needed by the render graph—including explicit normals, widths, extent, and declared computed primvars—is evaluated and retained at the same offsets under the same fence. Whenever a BindingEpoch owns and publishes the final points leaf, the overlay blocks the complete upstream velocities and accelerations primvar entries for every frame—including disabled, zero-weight, failed pass-through, or numerically unchanged results. Ownership, not a value comparison, prevents a renderer from applying derivative motion on top of RigExec’s authoritative point samples. A future qualified derivative-recomputation profile is outside v0.1.

RigExecRenderPreflight defines one numeric absolute renderBaseTime for the render frame and a vector of absolute sample times renderBaseTime + shutterOffset, all in stage time codes. For every authored-base PointBased leaf enumerated by §7.2, the resolver calls stock UsdGeomPointBased::ComputePointsAtTimes(sampleTimes, renderBaseTime), retains the exact returned point3f\[\] values, and feeds the complete matching role-slot set immediately before each serialized OpenExec sample evaluation. Ordinary one-sample evaluation uses ComputePointsAtTime(time, time) for every authored-base leaf. Moved preceding/final driver dependencies evaluate their existing chain heads after those base slots are installed. Velocity is interpreted in position units/second and acceleration in position units/second²; the stock implementation applies UsdStage::GetTimeCodesPerSecond() conversion. Each derivative array must be empty or match its source points count, accelerations require valid velocities, and all contributing samples must satisfy stock alignment rules ([26.08 PointBased contract](https://github.com/PixarAnimationStudios/OpenUSD/blob/v26.08/pxr/usd/usdGeom/pointBased.h)). RigExec validates those conditions before the call and fails the complete frame/render preflight deterministically on mismatch, non-finite values, API failure, or an incomplete returned slot/sample set; it does not silently discard a bad derivative and fall back to a different motion rule.

All materialized PointBased base-provider values/roles, moved-phase bindings, source dependency versions, renderBaseTime, absolute sample times, and timeCodesPerSecond participate in the request/generation identity and batch digest. The native provider updates are complete before ChangeTime; standalone packs store the same base values as ordinary generic resolved-state rows under their explicit evaluation identities. This preserves stock source-motion behavior for destination bases, blend samples, and lattice/curve/surface drivers without bypassing a moved driver chain or forwarding source derivatives as final output. HdSceneIndexCreateArgsSchema.motionBlurSupport advertises renderer capability but does not supply sample times; render preflight supplies the offsets. Automatic shutter-distribution negotiation remains deferred **BUILD**, not an upstream assumption.

1.  Preflight validates configured shutter offsets for visible outputs and checks them against the renderer capability bit.
2.  The evaluator computes one immutable pointBasedInputBatchDigest, then batches characters at each absolute sample time and validates every compute against all installed provider subgenerations.
3.  RigExecSnapshotStore retains all captured offsets under that batch digest and publishes one render-generation fence only after completeness and the external-revision/epoch check; later provider-slot writes do not stale earlier retained offsets.
4.  Each snapshot-backed sampled source implements GetContributingSampleTimesForInterval() and GetTypedValue() / GetValue(): it reports the retained frame-relative offsets and returns only cached values.

Missing required samples fail render preflight; GetPrim() never evaluates.

## 10.6 Frame and render-profile publication

  - Ordinary time evaluation: one full numeric/ PreTime UsdTimeCode ChangeTime plus prepared requests produces a complete immutable frame generation. The notice owner releases all affected prim dirties only after the generation fence is accepted.
  - Exact cached generation: a cache hit may publish immediately only when the complete RigExecEvaluationIdentity, externalSourceRevision, BindingEpoch, and selected provider-state digest all match; a render collection additionally matches its immutable pointBasedInputBatchDigest. A miss follows §8.4, and stale completed generations fail the same complete identity test.
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
    ResolvedStateTable exportedResolvedStates; // provider + complete evaluation identity  
    ConnectionTable incomingAndOutgoingConnections;  
    MetadataTable metadata;  
    std::string sourceLayerIdentifier; // Ar-resolvable identifier for standard source.usdc

};

The adapter implements the 26.08 Esf queries required by the qualified computation corpus: stage/object/prim/property lookup and schemas; identity/metadata/connections; attribute types and exact resolved states; and relationship/forwarded targets. It does not expose or evaluate spline/time-sample curve data. The composed authored description is available as one generic flattened standard USDC layer without constructing a UsdStage; the generic tables bind exact prim/property paths and SdfValueTypeNames to those standard specs.

sourceLayerIdentifier is always a normal ArResolver identifier accepted by SdfLayer::FindOrOpen(). A referenced payload uses its resolved asset identifier. For an embedded payload, the loader either uses a separately qualified generic package resolver identifier such as asset.rigpack\[source.usdc\], or extracts the byte-identical source.usdc entry to a managed temporary file and opens that ordinary path. The temporary asset and opened SdfLayerRefPtr are retained for the complete adapter/system lifetime and removed only after every Esf holder is gone. v0.1 never asks SdfLayer to interpret an arbitrary pack byte span and never introduces a geometry-specific file-format plugin or resolver.

Geometry is not split into a special partition or reconstructed object. exportedResolvedStates is one generic exact lookup table keyed by (canonical public provider-value address, RigExecEvaluationIdentity). The complete evaluation identity contains profile, full tagged RigExecTimeKey, stage interpolation policy, optional renderBaseTime/absolute-sample-set hash, the canonical transient-override set hashed as overrideDigest, and any shape-preserving runtime complexity selector not already fixed by profile or BindingEpoch. Canonical override entries are ordered by canonical public value address and exact type; identity equality verifies that retained typed list after the digest comparison, so a hash collision cannot alias requests. For a geometry array, the provider-value address is always the stock owner prim’s native property plus the declared role/read phase—never a generated application or slot path. The adapter maps that native address to private static OpenExec wiring only after load. Pack-exported identities always use the canonical empty override set and the profile/epoch’s default complexity context; a future host override need not be predicted or pre-exported. A standalone request with nonempty overrides instead supplies a complete resolved-state override set for every registered external provider, and that transient set is selected, cached, fenced, and traced under its own complete identity. The table is produced by stock USD resolution during compilation. Any time-resolved geometry entry is simply the property’s native typed value. Each entry is either a typed value or an explicit blocked/no-value state; standalone never substitutes a default or preceding value for a block. Compact holders reference snapshot/layer state that outlives ExecSystem; Initialize() rebinds retained queries. Every virtual, buffer/alignment rule, lifetime path, resolver/open failure, managed-temporary lifetime, change/resync, and query revival has conformance tests.

## 11.2 System and change delivery

RigExecStandaloneSystem exposes the same request, prepare, compute, override, and cache-view façade as §9 plus exact exported-identity selection. Internally it passes RigExecEsfSceneAdapter::Adapt(scene) to ExecSystem(EsfStage&&). Delta batches mirror execUsd: expire request indices, discard fully expired requests outside tracker locks, scope a local \_ChangeProcessor across DidResync/DidChangeInfoOnly/DidChangeIncomingConnections, select every adapter row by canonical public provider-value address plus the complete RigExecEvaluationIdentity, privately resolve the selected rows to Esf/OpenExec providers, install the identity’s exact retained override set, and call \_ChangeTime(EfTime) reconstructed with the same numeric/PreTime identity. EfTime preserves and hash-compares that tag, so exact and pre-time requests at the same numeric payload invalidate and cache separately; ordinary (time,time) point resolution and a motion sample at the same absolute time under another renderBaseTime, or requests with different override/complexity contexts, also select distinct rows. Only a declared empty-override exported identity or a host-supplied complete resolved-state override set is legal; the latter forms and retains its own canonical identity and may not borrow rows from an exported or transient identity with a different override digest.

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
  rest/pose defaults, declared external value-input slots, exported RigExecEvaluationIdentity set  
  resolved value-or-block states, content-deduplication map, canonical public tap addresses  
  exact native property paths/Sdf types, blend-input/native-target-shape descriptors  
  weight-object descriptors, static constant/dense/sparse packet values/defaults  
  dynamic source bindings and fixed layouts (never evaluated dynamic values)  
Standard USD payload:  
  referenced or embedded flattened authored source.usdc, byte-identical standard layer data  
Profile records:  
  profileId, composedLayerHash, population manifest  
  canonical (mover,target) applications, post-order ordinals, chain hashes

  matrix-provider/read-phase and weight-object addresses for each matrix mover  
  partition/request manifests, schema/property overrides, static plugin IDs  
Chunks per profile:  
  preview, animation or film computation/binding data; no geometry-specific encoding

The pack contains data, not code; plugin IDs resolve trusted registrations. One generic flattened authored-USDC payload contains all retained standard authored specs, including native geometry, without a geometry partition. Generic typed-state entries retain exact native USD types. No pack-native topology, primvar, aggregate, SoA, or blob structure exists. Byte compression/content deduplication may apply generically to any payload but creates no separately versioned geometry format or SDK type. Every native evaluation request explicitly selects stock UsdStage interpolation type linear or held. One pack pins exactly one such policy across all of its profiles; a different policy requires a separately built pack. That stage-local policy, timeCodesPerSecond, framesPerSecond, the render-offset unit (stage time codes), and each renderBaseTime/absolute-sample mapping are stored in the manifest and semantic hash. A RigExecTimeKey encodes the canonical IEEE-754 bits of the composed-stage time code plus an exact or preTime tag; NaN and infinity are illegal, and signed zero canonicalizes to positive zero. This prevents recomputing frame + offset and losing exact lookup identity.

For every exported empty-override RigExecEvaluationIdentity, rigexec-compile asks the unchanged OpenUSD 26.08 stage for **every registered external value-input slot**, including slots that currently appear static or have only one authored time sample. It calls timed UsdAttribute::Get(time) (or the equivalent qualified UsdAttributeQuery) and stores either the resolved typed value or explicit blocked/no-value state; identical states may be content-deduplicated only after resolution. Every statically declared authored-base PointBased dependency also contributes an ordinary generic row keyed by its canonical native source .points address, role, declared read phase, and complete identity. The compiler fills that row with stock ComputePointsAtTime(time, time) for an ordinary exported identity or the corresponding result from ComputePointsAtTimes(sampleTimes, renderBaseTime) for a motion-profile identity; the standalone adapter maps it to the private inputs:resolvedPointBased:\<role\> execution slot after load. A dependency bound to a moved preceding/final phase stores no duplicate materialized row; it consumes the existing chain result after that chain’s own base slots are supplied. No generated application or slot path is serialized as a geometry value address, and there is no geometry-specific record kind, table, or encoding. Source paths/roles/read phases, points, velocities, accelerations, time-code rate, base time, and sample set are semantic dependencies. This rule never relies on ValueMightBeTimeVarying(). The pack serializes no spline knots, tangents, time-sample maps, curve interpolation modes, clips, or custom animation tracks. Standalone evaluation therefore performs exact identity lookup only. An unexported or nonempty-override identity fails explicitly unless the host supplies a complete already-resolved state override set, including every required provider; that set is retained under its canonical overrideDigest, and arbitrary-time native interpolation remains available only in the USD deployment.

Schema normalization happens before serialization: sparse weight properties are stored only as canonical sorted (index,value) packets, raw authored pair order is omitted, and generic property records do not duplicate the raw weight arrays. On load, RigExecSceneDb exposes canonical logical rigExec:indices, rigExec:values, and rigExec:defaultWeight properties from those packet tables through Esf, so the shared StaticWeight registrations receive the same typed inputs without reconstructing raw author order. Consequently an order-only pair permutation with the same mapping produces the same semantic hash and pack bytes. Content is shared only when byte-identical across composed profiles. Identical normalized composition, ordered profile set, stage interpolation policy, time-code units, exported evaluation-identity set, render-base/sample mapping, generic resolved-state rows, and options produce identical bytes, with timestamps only in an unhashed sidecar.

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
5.  Compare canonical mover applications/order, matrix-mover/weight/blend descriptors, every requested intermediate revision, point frames, matrices, weight packets, statuses, and every exact native geometry property value/type/metadata plus invalidation sets.

Frame/matrix tolerances are 1e-10 in double math; bulk float points use 1e-6 × characterScale unless a callback declares a tighter bound. Native-USD animation goldens cover spline knots/in-betweens/extrapolation, ordinary held/linear samples under both explicit stage policies, single-sample attributes, layer strength, clips, PreTime, SdfValueBlock, BlockAnimation(), and time-sample-versus-spline precedence. Pack parity compares only the explicit empty-override exported evaluation-identity set, including value-versus-block state for every registered external input slot; nonempty standalone override tests provide a complete resolved-state set. Collision goldens use the same absolute time for ordinary (time,time) resolution and a motion profile with a different renderBaseTime, and separately use identical time/profile/render context with two different canonical override sets and runtime complexity selectors; they prove distinct queue items, generic rows or transient state sets, cache entries, generation fences, traces, and results. An injected equal-digest test proves retained typed override entries remain collision-authoritative. Native-geometry parity is property-by-property across USD, every intermediate tap, snapshot, embedded/referenced USDC, generic resolved-state entry, standalone result, export, and Hydra mapping; it includes prim type, exact SdfValueTypeName, value/cardinality, topology/subdivision properties, interpolation, role, element size, color-space metadata, and flat/indexed representation. Structural edit tests cover mover reparenting, composed child reorder, moves retargeting, matrix-transform or weight-object retargeting, static↔dynamic replacement, weight-target/representation/cardinality/canonical-support/source changes, one blend input/sample add/delete/retarget, native geometry cardinality/descriptor changes, and remove/re-add. Value-only tests cover compatible matrix, static-field/default, dynamic-driver/output, blend-channel, native target-shape points, source points/velocities/accelerations/time-code rate, generated base-provider values, and ordinary point/normal/extent/primvar value changes plus unsorted sparse weight authoring and order-only paired index/value permutations; they compare the exact chains/schedules/taps invalidated. Weight descriptor/packet parity fixtures cover scalar, native point, element-domain array, and registered non-geometry plugin domains. Multi-target fixtures contrast independent per-target fan-out with a schema-declared atomic bundle: one bundle failure returns every named target's preceding revision under the same MoverFailed status, and a role-relationship target set that differs from moves fails composition validation. Backend parity is a Phase 0 and continuous-release gate.

## 11.5 Standalone alternatives

**Alternative A — create an in-memory** **UsdStage** **in every runtime.** It reuses ExecUsdSystem but fails the explicit no-UsdStage deployment requirement and carries composition/plugin costs onto constrained devices.

**Alternative B — fork** **exec** **with a custom scene compiler.** It meets independence but creates two engines and permanent divergence.

**Alternative C — chosen: implement Esf objects behind one compatibility boundary.** This is the seam visible in source and in Pixar’s 2024 [OpenExec architecture talk](https://openusd.org/files/OpenExecASWF.pdf), where Presto and USD scenes feed one compiler through adapters. The open-source seam remains internal, so upstream stabilization is a real milestone rather than an assumed guarantee.

# 12\. Extension and plugin API

## 12.1 Native OpenExec registration

**Status:** C++ schema registration **EXISTS**; RigExec helper conventions **BUILD**.

Only genuinely RigExec-specific non-geometry rig, weight, parameter, and diagnostic result types—scalar or immutable aggregate—are registered. A stock USD array property is not a VtArray\<T\> OpenExec computation: its expression result/input type is the scalar element type and its VDF value is an element vector; the passive stock attribute bridge supplies the exact native array extraction. VtArray\<T\> is therefore forbidden in Computation\<T\>, AttributeValue\<T\>, Callback\<T\>, plugin computation results, and project type registration. No native geometry value is wrapped or re-registered:

TF\_REGISTRY\_FUNCTION(ExecTypeRegistry)  
{  
    ExecTypeRegistry::RegisterType(RigExecPointFrame{});  
    ExecTypeRegistry::RegisterType(RigExecPointFrameArray{});  
    ExecTypeRegistry::RegisterType(RigExecPointFrameMoverResult{});  
    ExecTypeRegistry::RegisterType(RigExecFloatMoverResult{});  
    ExecTypeRegistry::RegisterType(RigExecVec3fMoverResult{});  
    ExecTypeRegistry::RegisterType(RigExecMatrixValueMoverResult{});  
    ExecTypeRegistry::RegisterType(RigExecMoverParameters{});  
    ExecTypeRegistry::RegisterType(RigExecMoverStatus{});  
    ExecTypeRegistry::RegisterType(RigExecWeightDescriptor{});  
    ExecTypeRegistry::RegisterType(RigExecWeightBindingRecord{});  
    ExecTypeRegistry::RegisterType(RigExecWeightPacket{});  
    ExecTypeRegistry::RegisterType(RigExecBlendInputDescriptor{});  
    ExecTypeRegistry::RegisterType(RigExecBlendSampleDescriptor{});  
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
        AttributeValue\<float\>(RigExecTokens-\>inputsDefaultWeight),\
        RigExecResolvedMatrix(  
            RigExecTokens-\>rigExecTransform,  
            RigExecTokens-\>rigExecTransformReadPhase),  
        Relationship(RigExecTokens-\>rigExecWeightObject)  
            .TargetedObjects\<RigExecWeightPacket\>(  
                RigExecTokens-\>computeWeightPacket));

Every mover’s parameter computation includes inputs:enabled, inputs:defaultWeight, and the optional rigExec:weightObject binding. Disabled evaluation is therefore an ordinary invalidated pass-through of the preceding revision. RigExecResolvedMatrix(...) is a project lowering helper, not an upstream runtime selector: the compiler resolves rigExec:transform plus the structural read-phase token to exactly one catalogued base/preceding/final computeMatrix provider or exact matrix4d property adapter, then authors the generated relationship/input to that fixed provider before OpenExec compilation. When a single-target mover's rigExec:weightObject has one target, the compiler resolves it to exactly one computeWeightPacket, verifies that the packet descriptor’s canonical target and logical count match the exact move target, and marks the packet as the authoritative envelope. For an atomic multi-target mover, it instead requires the constant one-element operation domain from §4.1: the packet target is the mover prim and its scalar broadcasts across every application. When the relationship has no target, the parameter computation broadcasts inputs:defaultWeight. The typed mover has no second weight or strength parameter. Provider bindings are static for the epoch; defaultWeight, matrix, and packet values remain ordinary dynamic computation inputs.

RigExecStaticWeight.computeWeightDescriptor consumes the compiler-supplied exact target descriptor plus representation, raw sparse indices, and range policy and returns the canonical sorted support. computeWeightPacket also consumes the raw indices, so it preserves authored (index,value) pairing while permuting values into descriptor order; an order-only pair permutation invalidates the packet but compares equal at the descriptor/epoch boundary. It also consumes the authored current values and sparse default. RigExecDynamicWeight.computeWeightDescriptor freezes the same target/shape contract. RigExecCompiledWeightBindings(...) is a compiler-owned static record containing the canonical base-object path plus canonical public driver/scale/bias connection/value addresses and types; it makes those identities part of descriptor hashing without making current provider values structural. Backend adapters resolve those addresses to private providers only at runtime. The dynamic computeWeightPacket consumes the optional base object’s packet and the declared driver, scale, and bias values. It transforms the current default only for constant/sparse packets and preserves canonical default zero for dense packets. No weight callback enumerates a stage or changes representation/cardinality. A descriptor mismatch, non-finite value, or strict range violation makes the generated application’s computeMoverStatus return MoverFailed(firstBadAddress) using the canonical public value address; the exact-property outputs:expression callback then uses SetOutputToReferenceInput(previous) so the preceding VDF vector passes through bit-for-bit.

Blend wiring follows the same static/dynamic split. RigExecBlendSample.computeBlendSampleDescriptor consumes activation plus the exact native target-shape points path, type, and cardinality descriptor; RigExecBlendInput.computeBlendInputDescriptor consumes descriptors from every related sample. In canonical descriptor order, the generated point-array application’s hidden point3f\[\] inputs bind only to the §7.2-resolved providers: an authored-base destination or sample leaf uses its exact inputs:resolvedPointBased:\<role\> slot, while a dependency declared at moved preceding or final phase uses that existing passive chain output. The original native paths remain descriptor/dependency identity and are not directly re-read by the expression. The registrations use scalar-element AttributeValue\<GfVec3f\>/Connections\<GfVec3f\> inputs, so the callback receives stock VDF vectors, derives deltas, and writes the destination VDF vector. The passive exact-typed outputs:value attribute is the only internal OpenExec extraction bridge and yields VtVec3fArray; public taps retain only canonical native target/type/phase(/mover) addresses. No target, base, delta, or result becomes a RigExec array type. Membership and descriptor changes rebuild the epoch; compatible channel-weight or target-points values only invalidate the expression.

Each hidden operation/type-specific exact-property application—such as RigExecMatrixPoint3fArrayMoverApplication, RigExecBlendPoint3fArrayMoverApplication, or RigExecRecomputeNormal3fArrayMoverApplication—is a behavior host whose authored attributes use only exact stock Sdf types. It has a scalar computeMoverStatus, a fixed scalar-element AttributeExpression\<ElementT\> on outputs:expression, and the passive same-typed outputs:value bridge from §7.2. The expression’s statically registered .Inputs(...) list includes the preceding VDF vector, status, parameters, and every allowed side-property vector; no runtime registry can add a dependency. It then dispatches its compatible kernel or references the preceding vector. The catalog retains the exact SdfValueTypeName, so points, normals, and extent cannot be rebound merely because all three execute as GfVec3f elements. Point-frame/float/Vec3f/matrix-value applications retain their RigExec-specific scalar result conventions; geometry properties do not. Project target-chain helpers expand to upstream static inputs and are not claimed upstream calls.

The stock 26.08 callback form for a point3f\[\] application is:

self.AttributeExpression(RigExecTokens-\>outputsExpression)  
    .Callback\<GfVec3f\>(+\[\](const VdfContext& ctx) -\> void {  
        VdfReadIterator\<GfVec3f\> previous(  
            ctx, RigExecTokens-\>previous);  
        VdfReadIterator\<RigExecMoverParameters\> parameters(  
            ctx, RigExecTokens-\>parameters);  
        VdfReadIterator\<RigExecMoverStatus\> status(  
            ctx, RigExecTokens-\>status);  
        if (\!RigExecStatusAllowsApply(\*status)) {  
            ctx.SetOutputToReferenceInput(RigExecTokens-\>previous);  
            return;  
        }  
        auto result = VdfReadWriteIterator\<GfVec3f\>::Allocate(  
            ctx, previous.ComputeSize());  
        size\_t element = 0;  
        for (; \!previous.IsAtEnd();  
             ++previous, ++result, ++element) {  
            \*result = RigExecApplyPointElement(  
                \*previous, \*parameters, element);  
        }  
    })  
    .Inputs(

        Connections\<GfVec3f\>(ExecBuiltinComputations-\>computeValue)  
            .InputName(RigExecTokens-\>previous)  
            .Required(),  
        Prim().Computation\<RigExecMoverParameters\>(  
            RigExecTokens-\>computeMoverParameters)  
            .InputName(RigExecTokens-\>parameters)  
            .Required(),  
        Prim().Computation\<RigExecMoverStatus\>(  
            RigExecTokens-\>computeMoverStatus)  
            .InputName(RigExecTokens-\>status)  
            .Required());

The compiler has already proven that previous resolves to exactly one source before the reference pass-through is legal. This representative matrix application obtains its transform and complete weight packet from the declared immutable parameters input; an operation needing native topology or another array lists those explicit inputs in its own application schema. RigExecApplyPointElement may read only previous, parameters, and status. outputs:value is not in this registration; its ordinary single connection to outputs:expression selects stock connected-value computation and full point3f\[\] extraction.

There is **no public RigExec array or geometry kernel registration ABI**. Core applications link their callbacks directly in their schema libraries through stock EXEC\_REGISTER\_COMPUTATIONS\_FOR\_SCHEMA and AttributeExpression\<ElementT\>. A third-party native-property mover follows the same path: it supplies a generated mover schema, an operation/type-specific hidden application schema whose attributes use exact stock Sdf types, and a complete static OpenExec expression registration in that schema library. Its compiler-lowering manifest contains only schema tokens, supported stock owner kinds, exact input/output property paths and SdfValueTypeNames, fixed topology/output-set policy, and the application schema name. The manifest contains no callback pointer, array interface, geometry value, buffer layout, topology carrier, or extraction wrapper. OpenExec discovers the callback through the application schema’s Info.Exec.Schemas entry.

The compiler accepts that manifest only when the application’s frozen signature covers every declared dependency, the output role/type matches the move target, topology and output membership are fixed, and the two-attribute passive extraction contract is present. A plugin that needs different native side arrays or topology supplies a different predeclared application schema; it cannot add inputs at runtime. The callback itself reads/writes stock VDF element vectors and must finish every VdfContext, iterator, reference, private span, SoA, or SIMD scratch use before returning. When one declared input flattens several source arrays, descriptor-checked VdfSubrangeView\<VdfReadIteratorRange\<ElementT\>\> boundaries recover each source; equal C++ element type never infers semantic grouping. Validation completes before allocation, and a post-validation callback is total, non-throwing, and cannot fail after partially writing output. It cannot return VtValue, VtArray, a geometry aggregate, a topology carrier, an output-set change, or an arbitrary property. Scalar-property extensions likewise register directly through typed stock OpenExec callbacks; RigExec exposes no common erased value/callback ABI. Native extension binaries remain pinned to the qualified OpenUSD/compiler/CRT build.

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

Release assets reject an unknown ABI, nondeterministic hot-path plugin, missing fallback, unregistered RigExec result type, callback over the latency limit, undeclared property input/output, custom geometry type, or topology-changing output. Desktop may discover signed dynamic libraries; mobile uses a generated static registration table.

## 12.3 C++ client façade

auto system = RigExecSystem::FromUsdStage(stage); // or FromPack(path)  
auto taps = system-\>CreateTapSet({  
    RigExecAddress("/Char/Rig/Joints/Wrist", "computePointFrame"),  
    RigExecPropertyAddress(  
        "/Char/Geom/Body.points",  
        SdfValueTypeNames-\>Point3fArray,  
        RigExecTokens-\>final)});  
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
| Compatible authored-value or numeric-time request → full Hero-A animation-LOD native points/normals/extent/declared-primvar generation | p95 ≤ 33 ms |
| Single callback in evaluator/publication critical path | p95 ≤ 2 ms |
| Exact cache-hit time-key request → published pose generation | p95 ≤ 2 ms |
| Cold exact-time-key request → pose+proxy generation | p95 ≤ 16 ms |
| One Hero-B film-LOD evaluation at 24 fps | p95 evaluator ≤ 30 ms; total frame ≤ 41.7 ms |
| Eight Hero-A characters at 24 fps, animation LOD | p95 evaluator+Hydra publication ≤ 25 ms; total render frame ≤ 41.7 ms |
| Forty Crowd-Lite characters at 24 fps | p95 evaluator+publication ≤ 25 ms |
| Hydra notice/publication work per 24 fps frame | p95 ≤ 2 ms and ≤ 5,000 coalesced entries |
| Warm compatible value/time-update memory growth after 10,000 evaluations | zero unbounded growth; ≤ 1% steady-state drift; no persistent second full-geometry representation |
| Hero-A initial rig compile+profile prepare, stage already composed | p95 ≤ 2 s |
| Local structural recompile affecting one limb | p95 ≤ 100 ms |
| Headless tablet proxy evaluation/publication | 60 Hz target; p95 evaluator+publish ≤ 14 ms |
| Tablet full mobile-LOD time-sequenced evaluation | 30 fps; p95 evaluator+publish ≤ 25 ms |
| USD/standalone numeric parity | frames ≤ 1e-10 double; points ≤ 1e-6 × character scale |

  

Latency target order is deliberate: renderer-visible freshness is governed by worst-case evaluation-and-publication delay, not only average throughput. Batch work may use additional cores only after required frame and publication budgets are protected.

## 13.3 Instrumentation

Each headless trace spans externalSourceRevision, complete RigExecEvaluationIdentity (including the exact/PreTime tag, render-base/sample-set context, canonical override set/digest, and independent runtime complexity selector), provider-state or batch digest, queue, compile/schedule, callbacks/occupancy, exec completion, snapshot materialization/acceptance, Hydra notice, and render consumption.

The trace schema records:

  - critical path and ready-but-not-running time;
  - concurrency across nodes and SIMD/work within nodes;
  - cache hit/miss and invalidation fan-out by address;
  - compilation/schedule/evaluation time separately;
  - mover count, maximum target-chain depth, target fan-out, and fused/unfused applications;
  - hierarchy/moves structural recompile scope and chain-hash changes;
  - stale-generation work;
  - native-array bytes read/written, retained standard-property snapshot bytes, private scratch bytes, native↔scratch conversions/materializations, and device-transfer bytes;
  - character/region deadline misses;
  - plugin quarantine or thread-unsafe serialization lanes.

DreamWorks’ LibEE material repeatedly emphasizes that rig topology and critical-path measurement are necessary to realize engine parallelism; see [“Building Highly Parallel Character Rigs”](https://research.dreamworks.com/wp-content/uploads/2018/07/highly_parallel_characters-Edited.pdf) and the [LibEE profiling chapter](https://www.oreilly.com/library/view/multithreading-for-visual/9781482243567/chapter-31.html). RigExec emits machine-readable traces; a product profiler UI is deferred.

## 13.4 Benchmark method and gates

1.  Separate cold composition/compile, warm evaluation, value/structural edits, forward/reverse/random time access.
2.  Run at least 500 warm samples; report p50/p95/p99, variance, and worst traces with asset/OpenUSD/compiler/plugin hashes pinned.
3.  Sweep worker limits and request profiles; measure partial-evaluation overhead rather than assuming it wins.
4.  Benchmark direct SIMD over native arrays against optional fused private scratch; reject scratch unless its measured gain exceeds conversion/materialization cost and it leaves no retained duplicate representation.
5.  Accept speed only after output/invalidation parity; run sanitizers and 10,000 randomized edit/evaluate sequences.
6.  Measure ten-minute laptop/tablet power and thermal steady state.
7.  Ledger every run; \>5% p95 or \>10% memory regression needs an approved traced waiver.

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
  - stock array-expression proof for exact point3f\[\], normal3f\[\], and float3\[\] roles using scalar-element VDF execution plus the passive same-typed extraction bridge;
  - cached xform-only Hydra filter proof that never computes in GetPrim();
  - trace IDs and compile/schedule/evaluate timing.

**Exit criteria**

  - same point/matrix results and invalidation sets for 1,000 randomized edits, native-USD times, exported pack times, and exact-typed overrides; same-time/profile requests with different canonical override sets or runtime complexity selectors never coalesce, cache-hit, accept, or publish as equivalent;
  - override-vs-ordinary interest renewal and remove→re-add request rebinding pass version-pinned regressions;
  - standalone process constructs no UsdStage;
  - no source patch, fork, copied private implementation, export-symbol modification, or other change to stock OpenUSD;
  - compile-time and source scans reject VtArray in Computation\<\>, AttributeValue\<\>, Callback\<\>, project type registration, and plugin computation results; array probes cover empty, singleton, multi-element, time-varying, chained, exact-role mismatch, and disabled/failure reference pass-through;
  - every array probe proves that outputs:expression has one exact-role preceding connection and a scalar-element result, outputs:value is a passive same-typed one-connection bridge, {outputs:value, computeValue} extracts the exact complete native array, and a direct expression request is rejected by the catalog;
  - Esf contact surface isolated in one library and version-pinned;
  - every required internal Esf/ exec entry point compiles, links, loads, and passes a smoke compute on Windows, macOS, and Linux against the pinned unchanged build; the qualified standalone distribution uses only symbols available from that stock monolithic/static or same-visibility-boundary build;
  - architecture review accepts the pinned stock-build compatibility boundary or reduces standalone scope; changing OpenUSD is not an alternative.
  - performance council accepts or revises the provisional targets with captured baseline traces.

## 14.3 Phase 1 — schema, point frames, taps and FK (10–12 weeks)

**Deliverables**

  - full schema v0.1 and usdGenSchema C++ classes;
  - mover API, native-property/RigExec-value catalog, pre-registered property-typed hidden applications, generated session-layer/pack lowering, post-order compiler, and conflict/cycle validator;
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
  - numeric/goal/time/shape-preserving enable edits do not recompile; variant, operation, exact native property set/type/descriptor, phase, and target edits do;
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
  - every empty-override exported RigExecEvaluationIdentity and configured render-motion offset matches native USD, same-time ordinary/motion identities remain distinct, nonempty overrides require complete host-resolved states under their own identity, and unexported standalone identities fail explicitly;
  - for one character with identical time/profile/render context, two different canonical override sets or independent runtime complexity selectors never coalesce, hit the same cache/transient-state row, accept one another’s generation, or cross-publish; render requests also produce distinct pointBasedInputBatchDigest values;
  - Hero-A pose/proxy evaluation-and-publication gates pass on all desktop OSes;
  - stale results never publish under randomized edit/time races.

## 14.5 Phase 3 — geometry movers and Hydra publication (16–20 weeks)

**Deliverables**

  - property-typed mover chains over native UsdGeomMesh, UsdGeomPoints, and UsdGeomBasisCurves values plus CPU reference kernels;
  - weighted matrix, native target-shape blend, lattice, curve/ribbon, surface, and post movers plus exact-property target-chain compiler;
  - matrix-mover, static/dynamic weight, and blend descriptor parity between USD and standalone, including exact transform providers/read phases, exact weight targets, constant/dense/sparse fields, and independently layered blend inputs/samples;
  - baked standard OpenUSD xform, points, primvar, and geometry-cache export with no execution-only binding dependency;
  - RigExecInternalPrimPruningSceneIndex, RigExecBindingResolvingSceneIndex, and RigExecResultsSceneIndex;
  - motion samples, precise dependency/locator routing, Storm and two production-delegate test adapters;
  - SIMD kernels after reference correctness.

**Exit criteria**

  - automated arm goldens prove the bottom-up mover hierarchy numerically, expose every (mover,target) tap, and match the expected standard Hydra data sources;
  - multi-target fan-out, nested last-writer behavior, inputs:enabled pass-through, hierarchy reparenting, and affected-chain-only rebuild tests pass; atomic-bundle goldens contrast independent fan-out failures with one two-bone-IK bundle failure that returns every named target's preceding revision under one MoverFailed status, and reject any role-relationship target set that differs from moves;
  - stronger-layer/reference/variant tests can independently add, delete, replace, reparent, reorder, or retarget one matrix mover, static/dynamic weight object, blend input, or blend sample; only the composed mover hierarchy determines operation order, while unsorted and order-only-permuted sparse weight pairs canonicalize identically without an epoch rebuild;
  - mismatched move/weight targets, wrong-type transform or points targets, non-affine/non-finite matrices, dense/sparse weight cardinality errors, duplicate/out-of-range sparse weight indices, invalid weights, dynamic descriptor drift, duplicate activations, and native target-shape type/cardinality mismatches fail deterministically;
  - common-envelope goldens cover inputs:defaultWeight broadcast across every built-in mover family, the fallback value 1, exact zero pass-through, bound-weight-object precedence without multiplication, object removal exposing the already-authored fallback, and rejection of multiple or incompatible bindings; schema/API scans prove that concrete movers declare no duplicate envelope weight/strength properties while RigExecBlendInput.inputs:weight remains a channel amplitude;
  - weight-object goldens cover constant empty-values/rigExec:defaultWeight broadcast, dense canonical default-zero enforcement, sparse defaults and pair canonicalization, no-base dynamic constant evaluation plus rejection of no-base dense/sparse forms, strict versus clamp, rejection of StaticWeight time samples/connections, identical packs/no epoch for order-only sparse pair permutations, and one scalar, one element-domain array, and one plugin-catalogued property consumer;
  - scalar-reference, SIMD, USD, and standalone weighted-matrix results match for identity and non-identity transforms, zero/one/fractional weights, static/dynamic fields, sparse defaults, reflections, scale/shear, singular affine maps, and non-commuting sequential operations;
  - exact result/status goldens prove zero-weight, disabled, and failed mover applications return the preceding typed value bit-for-bit; disabled applications report disabled without error, failures report MoverFailed with the first bad canonical public value address, and neither path can publish a torn or partial update;
  - mesh, points, and basis curves work as inputs and outputs with only their standard schemas/properties; each fixture pre-authors its native extent and an explicitly ordered exact-target RecomputeExtent mover, while every base/intermediate/final points, normals, extent, widths, topology, and declared primvar value preserves its exact native type and metadata;
  - native-property matrices enumerate and validate Mesh topology/subdivision/Gprim policy, Points widths/ids, BasisCurves topology/type/basis/wrap/widths, built-in flat arrays, and real flat/indexed primvars; no built-in points/normals/widths/velocity/acceleration value is represented as indexed;
  - normal goldens cover polygon meshes with absent/authored/rebuilt normals, subdivision meshes with terminal normals blocked, constant proper affine transforms, reflections, singular maps, varying weights, nonlinear point movement, point-cloud orientation data, ribbon/tube curves, transported-frame degeneracy, and both built-in/ primvars:normals precedence;
  - motion goldens materialize stock ComputePointsAtTime(s) results through every exact authored-base PointBased role slot, reject incoming source points/velocities/accelerations connections, cover destination, blend-sample, lattice-cage, curve/ribbon-driver, and surface-driver roles, and prove that a driver read at final consumes its moved chain head while only that chain’s authored base is materialized; they use one common base time with samples on both sides, cover non-24 timeCodesPerSecond, empty and points-sized derivatives plus mismatched cardinality/alignment/non-finite failures, prove an owned provider notice invalidates dependents without advancing externalSourceRevision or recursively enqueuing work, prove a racing real source edit rejects the generation, and use at least three shutter offsets to prove later provider subgenerations do not stale earlier captured samples under one immutable batch digest; they retain final sampled points and dependent native outputs at every offset, structurally block complete upstream velocities/accelerations entries whenever the epoch owns final points, and prove that no delegate receives both authoritative point samples and derivative motion;
  - extent/width goldens prove that widths change only through their own native property chain; local/inherited primvars:widths precedence, built-in fallback, constant broadcast, indexed flattening, stock interpolation cardinality, and invalid indices/counts are covered; Mesh, Points, and BasisCurves extents derive from final same-generation points and the authoritative winning width at every retained sample without changing its public representation;
  - native example assets carry no project geometry applied API, property manifest, custom target-shape delta property, or geometry-specific serialized record; prim and explicit .points targets canonicalize identically, topology targets are rejected, and unrelated topology/render properties remain bit-for-bit unchanged;
  - type-registry, exported-header, SDK, tap, snapshot, pack, plugin-descriptor, lowering-manifest, generated-session-layer, and standalone-adapter scans find no custom geometry value type, carrier, topology handle, geometry view, primvar descriptor, array callback ABI, or geometry-specific record. Generated behavior-host array slots must have the exact Sdf type/role of one native property revision, must be absent from durable native value addresses, source USDC, pack row keys, export, Hydra, and public taps, and must disappear with the owned session layer; any private SoA scratch is released before the exact native result crosses the direct OpenExec callback boundary;
  - .rigpack round trips one referenced/embedded generic flattened source.usdc —including its native geometry—plus exact generic property states, and contains no geometry-specific partition, type ID, or blob format;
  - referenced pack layers open through their normal resolved ArResolver identifiers; embedded source.usdc opens through a separately qualified generic package resolver or a lifetime-managed byte-identical temporary file, never through an arbitrary byte-span SdfLayer API;
  - render delegates require no RigExec code and receive standard final data;
  - no compute/wait occurs in GetPrim();
  - Hydra construction and pull goldens cover pre-population attachment, already-populated wrapping with traversal/full resync, late-consumer traversal, native renderer and legacy adapter pickup, losing flat/indexed or intentionally absent leaves plus stale normal/derivative entries masked at the complete strongest entry by HdBlockDataSource, and ComputeDirtyLocators() expansion through every rebuilt \_\_containerDataSource ancestor; motion cases cover motionBlurSupport true/false/absent, a one-sample non-motion profile, exact retained frame-relative offsets and cached final-point values under one generation fence, blocked derivative entries, and missing-sample preflight failure without evaluation from GetPrim() or sampled-value pulls;
  - traversal plus live PrimsAdded / PrimsRemoved / PrimsDirtied streams expose no owned \_\_RigExecGenerated path, while unrelated prims, computations, notices, and standard data sources pass through unchanged;
  - exact transform, static/dynamic weight, native target-shape points, and blend-value edits produce the expected narrow locator sets; structural mover/weight/blend edits resync, and terminal enumeration contains no RigExec schema, locator, or value-type contract—private snapshot-backed subclasses of stock HdTypedSampledDataSource\<T\> remain an allowed implementation detail;
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
| Namespace composition silently changes mover precedence. | High: a reference, variant, reparent, or child reorder could change the final pose or geometry. | Validate after composition; record/hash every chain; diff chain ordinals on publish; rebuild atomically; run reference/variant/layer golden tests. Measured behaviour, which the earlier "require an authored sibling order" mitigation did not address: a stronger layer can never reorder existing siblings, it only appends; the real movers are weak-side insertion displacing a sibling and reference/arc order deciding composed branch placement. RigExec then executes that displayed sibling order bottom-to-top. Ordinal diffing on publish is what catches these; a reorder opinion does not. |
| Native geometry evaluation becomes memory-bandwidth bound, or a private optimized layout becomes a de facto second geometry ABI. | High: parallel nodes alone may miss film/tablet budgets, while retained scratch would violate the native boundary and double memory. | Prefer direct SIMD over stock VDF scalar-element vectors; native VtArray exists only at extraction/snapshot boundaries. Allow optional SoA only as private ephemeral fused-kernel scratch; measure VDF/native-array bytes, scratch bytes, conversions, materializations, and snapshot residency. Release gates prohibit a persistent second full representation and scan registries, taps, packs, headers, plugins, and Hydra for custom geometry types. |
| Hydra pulls race execution or trigger hidden work. | High: deadlock, tearing, or unpredictable renderer latency. | Workers publish immutable snapshots first; one notice owner sends coalesced dirties; GetPrim() only reads an atomic snapshot pointer and never computes or waits. |
| Native animation source precedence or type limits are misunderstood. | High: scalar splines, ordinary samples, clips, or standalone packs may evaluate differently. | Pin stock OpenUSD 26.08 and compare timed UsdAttribute::Get() with OpenExec for knots, in-betweens, extrapolation, paired exact/PreTime keys, held/linear modes, layer strength, clips, blocks, and time-sample-versus-spline precedence. Packs contain only exported-evaluation-identity resolved value-or-no-value states and fail unexported identities; no RigExec curve sampler exists. |
| Native plugins violate purity, determinism, or thread safety. | High: cache corruption and nondeterministic frames. | Mandatory descriptor, ABI/hash validation, sanitizer and randomized-order tests, certified fallbacks, time limits, and a serialized quarantine lane for non-release diagnostics only. |
| Tablet memory, thermal limits, or platform rules invalidate desktop assumptions. | High: headless time-sequenced evaluation or Hydra-publication validation misses its memory/thermal budget. | Static registration, no interpreter, compressed read-only packs, mobile LODs, NEON kernels, bounded snapshot residency, and ten-minute thermal gates. No on-device renderer, playback product, authoring, or editor surface is in scope. |

  

## 15.2 Resolved architecture decisions

1.  Stock OpenUSD only: the sole v0.1 baseline is the unchanged OpenUSD 26.08 release. All schemas, adapters, scene-index filters, and build glue live in RigExec. If the shipped Esf/ exec seam cannot be used from that qualified unchanged build, Phase 0 stops or reduces standalone scope. A newer OpenUSD release requires a separate future qualification and does not alter this contract.
2.  Provisional performance contract: retain §13’s 32-physical-core/128-GiB primary workstation, 12-core laptop, 8-core Apple-silicon tablet class, named workloads, and p50/p95 thresholds as current gates. Phase 0 records exact hardware and may calibrate them through evidence, not informal substitution.
3.  Infrastructure-only scope: v0.1 covers the composable data model, native USD animation values, compilation/evaluation, packing, extraction, desktop Hydra publication/rendering, parity, and headless mobile evaluation/publication validation. UI, manipulation, editing, on-device rendering/playback, and product tooling are deferred.
4.  Acceleration scope: scalar reference and CPU SIMD paths are mandatory and deterministic. GPU mover-chain computation is optional/profile-driven and cannot replace CPU gates; Hydra render delegates may use their normal GPU rendering paths.
5.  Native animation ownership: RigExec-authored scalar half / float / double / timecode animation uses native .spline knots evaluated through timed stock-USD resolution; other supported values use ordinary sparse .timeSamples with an explicit stage held/linear policy. Preexisting scalar time samples remain valid standard-USD inputs. RigExec has no custom animation source, curve sampler, or interpolation mode. Standalone packs store only stock-USD-resolved value-or-block states on an explicit empty-override RigExecEvaluationIdentity grid whose time component retains the exact/ PreTime tag; host overrides supply complete states under a distinct identity.
6.  Native geometry ownership: authored, composed, computed, tapped, snapshotted, packed, exported, and Hydra-published geometry remains stock UsdGeomMesh, UsdGeomPoints, or UsdGeomBasisCurves properties with exact native USD types. RigExec owns mover behavior and ordering only. Private execution layouts are disposable kernel details and never cross a public/cache/serialization boundary.

## 15.3 Requirement-to-evidence map

|  |  |  |
| :-: | :-: | :-: |
| \*\*Requirement\*\* | \*\*Primary design sections\*\* | \*\*Release evidence\*\* |
| R1 — dual deployment, one engine | 3.2, 11 | Same exported RigExecEvaluationIdentity corpus, results, and invalidations through USD and standalone; no UsdStage or interpolation engine in standalone process. |
| R2 — all authoring in OpenUSD | 4, 5.6 | Generated schemas validate; arm definition/reference/animation compose; timed resolved values match stock USD. |
| R3 — point-based transforms | 5, 9 | Landmark/frame/SRT property tests; every transform-semantic output passes compiler tap audit. |
| R4 — geometry in and out | 4.2, 7, 10–12 | Property-by-property Mesh, Points, and BasisCurves goldens across authored base values, scalar-element expression/passive native-array bridges, every bottom-up mover revision, snapshots, standard-USDC pack data, export, and Hydra; exact types/metadata/topology preserved and no custom geometry carrier exists. |
| R5 — OpenExec maximalism | 3.3–3.5 | Gap ledger reviewed against the pinned tagged OpenUSD 26.08 source. |
| R6 — Hydra 2.0 scene index | 10 | Standard xform/primvar/extent results in three delegates; flat/indexed and complete stale-normal/derivative masks, retained final motion samples, precise dirties, and no compute in pulls. |
| R7 — generation-safe evaluation/publication | 6, 8, 10, 13 | Edit/time races, latency, scale, motion-sample, and thermal benchmarks meet p95 gates with no torn publication. |
| R8 — uniform extraction | 9 | Hydra, export, pack, and validation clients use one typed batch/snapshot API; geometry taps return exact native property types and no private evaluator/carrier is exposed. |
| R9 — portability | 11–14 | CI/package/exported-evaluation-identity parity matrix for Windows, macOS, Linux, and static headless iPadOS runtime. |

  

# 16\. Glossary

  - Avar: animator-facing scalar or small vector parameter, such as IK blend or elbow roll.
  - Evaluation epoch: interval during which rig and geometry topology are fixed; structural edits begin a new epoch.
  - Esf: OpenExec’s source-level, currently non-public read-only scene interface.
  - Native geometry boundary: geometry is represented only by stock UsdGeomMesh, UsdGeomPoints, or UsdGeomBasisCurves prims and their exact standard property values from authoring through Hydra.
  - Private kernel layout: optional ephemeral aligned/SoA scratch derived from a native property value and materialized back before a callback returns; never an authored, cached public, tapped, serialized, SDK, plugin, or Hydra type.
  - Move target: canonical prim or property result named by rel rigExec:moves, with distinct base and final values.
  - Mover: a geometry or rig operation that writes one or more move targets through the common RigExecMoverAPI.
  - Mover application: compiler-generated pure computation for one (mover,target) pair and one logical post-order ordinal.
  - Bottom-up order: reverse-sibling post-order traversal of the composed Movers namespace: descendants before ancestors, with the bottom composed sibling branch first and the top branch last.
  - Binding epoch: immutable compiled interval containing exact mover chains, matrix-provider/read-phase bindings, weight and blend descriptors, output manifests, reverse dependency routes, and snapshot-compatible Hydra publication metadata.
  - Weight object: independently composable static or dynamic prim that publishes one total scalar field over the logical elements of one canonical prim/property target.
  - Point frame: four points (o, x, y, z) representing an affine transform as origin plus transformed basis endpoints.
  - Rig pack (.rigpack): deterministic, derived standalone deployment artifact compiled from composed USD.
  - Tap: typed canonical target/type/phase(/mover) extraction address included in a RigExecTapSet; it never exposes a generated provider path.
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
  - UsdPrim child-order API — true composed child order consumed in reverse by RigExec's recursive post-order mover lowering.
  - Computing values with ExecUsd — batching, cache views, and time.
  - OpenExec computation registration tutorial — schema-bound C++ registration and typed input patterns.
  - ExecTypeRegistry and value specifiers — all USD-authorable attribute and metadata types are known to OpenExec by default; only genuinely new application values require registration.
  - UsdGeomPointBased, UsdGeomMesh, UsdGeomPoints, and UsdGeomBasisCurves — native points, normals, extent, widths/IDs, topology, and curve policy used by every geometry path.
  - UsdGeomPrimvarsAPI and UsdGeomPrimvar — standard primvar value, interpolation, element-size, role, and optional indexed representation.
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
