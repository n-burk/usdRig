# Implementation plan: fixed-topology geometry, the exec-keeping tier, and the baked-rig mode

Date: 2026-09-13. Builds on `reports/biped_eval_trace_analysis.md` (sections 5, 6, 10) and on
`reports/rigexec-perf.patch`, which is the base for everything below.

## 0. Base and landing

- Base tree: the checkout as of 2026-09-12 21:30 plus `rigexec-perf.patch` (the tree every number in
  the analysis was measured on). Work happens in copies of that tree; each track lands in its own copy,
  a merge step joins them, three adversarial reviews and a fix-up follow, then the result is measured.
- Landing into the working tree is a three-way merge (base snapshot, this work, the tree as it is then),
  because another session is editing `rigEvaluator.cpp`, `rigEvaluator.h` and two test files at the
  same time (its edits so far: pose-seed and guide snapshot caches, `SetSolverGuidesEnabled`, a
  solver-level accessor, and two tests; captured for the merge). Two items from the analysis are
  therefore deliberately **not** in this plan to avoid duplicate work: guides gated on a consumer, and
  one exec request per solver level.
- Acceptance for every track: dynamic-mode output byte-identical to the base on `Biped.usda`,
  `Biped_layered.usda` and `Biped_anim.usda` (`--joints --targets` stdout and `--joints-out`); epoch
  digest unchanged; ctest failure set unchanged (`testRigExecCurvenet`, `testUsdNoodles` pre-existing);
  new behaviour covered by new tests; no environment-gated code paths except the documented kill
  switches; measured numbers reported as min of 5 or more paired runs.

## 1. Track G: geometry under a fixed-topology assumption

Target: animated geometry 1.5-1.6 ms to ~0.55 ms per frame on the biped; frame ~9 to ~7.6 ms.

- **G1. Per-epoch skin topology.** New `RigExecSkinTopology` (indices, weights, element size, point
  count, validated flag) held by `shared_ptr` on `RigExecMoverParameters`; identity compare in
  `operator==`; the assembler resolves it through a cache owned by the evaluator (member, keyed by mover
  path; cleared from `_OnObjectsChanged` on every notice; never a static). Compile refuses the cache
  when `rigExec:jointIndices`, `jointWeights` or `elementSize` are time-varying
  (`ValueMightBeTimeVarying`), connected, or resolved through a property chain; then the packet carries
  the arrays as today. The kernel validates once per topology and does an O(1) shape check per frame.
  Authored base points are read and pushed once per epoch when the attribute is not time-varying,
  skipping the 26k compare. Expected 0.62-0.70 ms per skinned mesh.
- **G2. Constant full-strength envelope fast path.** When the common envelope packet is the synthesized
  constant with `defaultWeight == 1` and a supported range policy, skip the preceding copy, `ResolveAll`
  and the blend (exact per `envelope.h`). Expected 0.17-0.19 ms.
- **G3. In-kernel parallelism.** `WorkParallelForN` over point ranges inside the LBS kernel and the
  envelope blend (grain 512-2048, measured; no dispatch below ~4k points), bit-identical. One
  `TfEnvSetting` `RIGEXEC_ENABLE_PARALLEL_EVAL` (default true) checked at every rigExec parallel site
  (this one, G4, and the two compile tasks already landed), so a regression can be bisected without
  `PXR_WORK_THREAD_LIMIT`. Expected 0.15-0.23 ms.
- **G4. Level-parallel chain walk, gated.** Partition `_chainOrder` into dependency levels at Compile
  from the existing `dependsOn` map; run a level under a `WorkDispatcher` only when it holds at least
  three chains and Compile classified the level as safe (no phased reads via `_chainSnapshots` across
  chains in the level, no curvenet or profile movers, no shared weight objects). Per-task diagnostics
  buffers concatenated in `_chainOrder` order; `_liveGraphs` pre-created at Compile; one
  `UsdGeomXformCache` per task; exec calls stay outside the region; `_chainSnapshots` records applied at
  the level barrier. Worth 0 on the biped, 0.9-2.3 ms per frame on 5-9 independent meshes; a synthetic
  multi-mesh stage is added under `tests/` as the fixture.
- **G5 (stretch).** A NEON path for `RigExecApplyLinearBlendSkinSimd` (the SSE2-only kernel falls back
  to scalar on ARM), parity-gated against the scalar reference. Estimated 0.13 ms.

## 2. Track T: the tier that keeps exec

- **T1. Epoch-constant rest taps.** Evaluate `computeRestFrame` for every provider once at Compile and
  drop the 326 taps from the pose-seed request and the rest taps from the authoritative snapshot; a rest
  edit already recompiles through the digest, asserted by a test. Measured 0.06-0.16 ms.
- **T2. Missed guard and value cache.** Add the `HasAuthoredConnections` guard at `moverGraph.h:190`
  (~0.2 ms, byte-identical). Add a per-evaluator cache for `_ResolvedRead` values admitted only when
  `!ValueMightBeTimeVarying() && !HasAuthoredConnections()`, cleared on every notice and on
  `SetInteractiveOverrides` (0.2-0.3 ms). The process-lifetime variant is explicitly rejected.
- **T3. First-frame premium at Compile.** Warm the pose-seed request once at the end of Compile (at the
  stage's start time code, or Default) so the 7-8 ms first warm compute and the first `ChangeTime` are
  paid before the first frame. No throwaway Evaluate.

## 3. Track B: the baked-rig mode (optional toggle)

Target: a complete animated frame in 0.5-1.5 ms, bit-exact with the dynamic path on every stage the
bake accepts; dynamic path untouched when the mode is off.

- **B1. `RigExecBakedProgram`.** Built at Compile from the compiled epoch: dense provider slots in
  namespace DFS order; an input binding table (attribute to slot) where every input that is
  time-varying, connected, or produced by a property chain is read per frame through a retained
  `UsdAttributeQuery` or computed in-program, and everything else is captured as an epoch constant; an
  ordered op list: property-chain ops, provider frame compose (rest and default-space ladder resolved
  once), the aggregate solver kernels in compiled batch order (SplineIK rest description and two-bone
  lengths baked), the constraint kernels in `_poseSteps` order with descendant propagation precomputed
  as (descendant, closest) pairs and the blocking rule applied at bake, `PointsToMatrix` for joints and
  skin influences, the skin kernel over G1's topology with the envelope applied, derived extent, and
  publication into `RigExecRigPose` (same maps, same contents, including `solverFrames` from the
  in-program aggregates). Starting point: the measured prototype (`bakedFloor.cpp`, 1,237 lines, plus
  `RunBakedFrameAt`), which is bit-exact for the biped but captures property-chain results and ignores
  interactive overrides.
- **B2. Bakeability.** Compile records a reason per feature the program cannot express and stays
  Dynamic when any is present: connected-space providers, intervening Xforms, weight objects and volume
  weights, geometry-domain constraints, SingleChainIK, ribbons, curvenets, blend shapes, profile movers,
  twist distributions, and any mover type outside skin and matrix. `IsBakeable(&reasons)` reports them.
- **B3. Invalidation.** Structural notice: rebuild with the epoch. Changed-info notice on any prim that
  contributed a captured constant: rebuild (a generated changed-property to captured-constant index,
  produced by the same pass that captures; the epoch digest alone is **not** a sufficient key). Value
  notice on a bound input: nothing to do, the query reads it. `SetInteractiveOverrides`: overrides on
  bound inputs become slot writes; an override the program cannot place forces Dynamic for that
  generation.
- **B4. Toggle.** `SetEvaluationMode(Dynamic | Baked | BakedWithParityCheck)`, `GetEvaluationMode`,
  `IsBakeable`. `BakedWithParityCheck` runs both paths in one generation, compares joint frames,
  matrices, provider transforms and moved properties with exact equality, and reports mismatches in
  `pose.diagnostics` and a counter. Python binding (`evaluation_mode`, `is_bakeable`), and
  `rigExecPose --mode dynamic|baked|parity`.
- **B5. Tests.** A ctest that evaluates the three biped stages plus a foot-roll/IK-FK keyed overlay in
  Baked and Dynamic over frames 1-8 and asserts exact equality; a test that keys the IK/FK weight,
  toggles `inputs:enabled`, and edits a rest after the bake and asserts the baked result follows; an
  interactive-override test; a fallback test on a rig with a curvenet; and the existing suites run once
  more with the mode forced to Baked where the rig is bakeable.

## 4. Order, merge, review

Tracks G, T and B run in parallel on separate copies (B in two stages: core program and mode, then
invalidation, overrides, property chains and tests). A merge step joins them into one tree, resolves
conflicts, builds every target, runs the full suite and the identity gates. Three reviewers attack the
merged tree (dynamic-path semantics and thread-safety; bake fidelity and invalidation; test coverage by
mutation), a fix-up applies blocker and must-fix items, then the combined tree is measured paired
against the base on the static, animated and multi-mesh stages. The result is a unified patch against
the base plus a changelog, and it is landed into the working tree by three-way merge.

## 5. Numbers to hit (animated biped, steady state, this machine)

| | base (patched) | after G + T | baked mode |
|---|---|---|---|
| frame | 9.0 ms | ≤ 7.6 ms | ≤ 1.5 ms |
| geometry | 1.5 ms | ≤ 0.6 ms | ≤ 0.3 ms |
| compile | ~100 ms | ≤ 110 ms | ≤ 120 ms including the bake |
| first frame of a session | 20 ms | ≤ 12 ms | |
