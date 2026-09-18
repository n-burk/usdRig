# `.rigexec` Binary + Bake/Runtime APIs (M1/M2 design)

Companion plan: `../../../godot_rigExec/PLAN.md`. Decisions D1–D7 locked
there; this doc is the serialization design they imply.

## 1. What gets serialized

The baked program is already declarative, so the binary is a flat
encoding of `RigExecBakedProgramImpl` plus the build tables the steps
index into — not a trace, not a frame cache:

- **Steps** (`RigExecBakedStep`, 18 `RigExecBakedStepKind`s): kind,
  object/part indices, declared read/write slot ranges, preds/succs,
  dirty-set flags (`isSource`, `externalReads`, `varyingInputs`,
  `resolvedInputReads`, `overrideInputs`), cluster/level/cost.
- **Schedule**: `RigExecBakedCluster` membership + edges, cone
  closures, grain. Cluster members stay in increasing program index,
  so a cluster still needs no internal schedule (D2: the runtime
  replays the same clusters on `WorkerThreadPool` / the USD
  dispatcher / serial fallback with zero re-derivation).
- **Domain tables** the steps index: compose groups, solvers, commits,
  pose interpolators, weight objects, revision index, chains, derived
  index, plus epoch constants and the input binding table.
- **Manifest**: rig identity, format version, joint/skeleton table
  (names, parents, rest matrices row-major), mesh bindings, baked
  frame list, feature flags — plus `compileDiagnostics`, the compile
  notices (inert movers, purpose warnings) a fresh evaluator seeds
  its first generation with. The runtime replays them ahead of the
  program lines on its first `Execute`, exactly once, then drains
  them; binaries baked before the key existed replay nothing.
- **Diagnostics section (optional, embedded)**: schedule report text,
  per-step labels, cone shadow-verify data. The loader skips it (D6:
  one file, no sidecar).

Per D1 every table is laid out data-oriented (SoA, 16B-aligned,
contiguous per-cluster working sets); the landmark-vs-matrix choice
is made per op for interpreter throughput. `double` where the baked
path computes in double (pose), `float` where it computes in float
(points/weights) — no silent narrowing.

## 2. Container layout (little-endian, v1)

```
header            magic "REXB", format_version u32 (=1),
                  section_count u32, flags u32
section_table[]   { tag u32, offset u64, size u64 } × section_count
sections...       string table, manifest, slot metadata, constants,
                  input table, domain tables (one tag each), steps,
                  clusters, cones, diagnostics
```

- All offsets u64, counts u32; unknown section tags are skipped, so
  minor versions can add tables without breaking old loaders.
- Loader rejects major mismatch, warns on minor. No forward-compat
  execution: a v1 runtime never runs a v2 program.
- String table first: every name (joints, meshes, override slots) is
  an index; no USD paths anywhere past the manifest's debug copies.

## 3. Input model (bake-time vs runtime)

- **Epoch constants** (rest ladder, solver rest descriptions,
  composed wiring): embedded verbatim; the binary never re-reads USD.
- **Varying inputs** (animated attributes, chain-driven values):
  sampled at bake for every requested frame into the input table
  (frame → dense values). The runtime indexes by frame; no USD, no
  `UsdAttributeQuery` in the player.
- **Overrides**: named input slots (`RigExecRoot`-relative names from
  the existing override index) settable per frame before `Execute()`.
  Same placement semantics as `SetOverrides`; unplaceable-on-baked
  overrides are a bake-time error listing the slot.
- Time model: the binary carries its baked frame list; evaluating an
  unbaked frame is a loader error, not an extrapolation.

## 4. APIs

Bake side (new `rigExecBake` SHARED lib, full USD linkage):

```cpp
struct RigExecBakeOpts {
    std::vector<double> frames;   // required, non-empty
    uint32_t targetReaderVersion = 1;
};
struct RigExecBakeResult {
    std::vector<uint8_t> bytes;   // the .rigexec file
    std::string manifestJson;     // human-readable manifest copy
};
RigExecBakeResult RigExecBakeToBinary(RigExecRigEvaluator &evaluator,
                                      const RigExecBakeOpts &opts);
// Throws RigExecBakeError naming the feature when the epoch cannot
// bake. Per D4 every such feature is then implemented (see §6),
// never kept as a refusal.
```

Runtime side (new `rigExecRuntime` STATIC lib, zero USD headers —
only `<cstdint>`, `<cmath>`, `<vector>`, vendored scalar math):

```cpp
class RigExecRuntimeReader {
public:
    static std::unique_ptr<RigExecRuntimeReader> Open(
        const uint8_t *bytes, size_t size, std::string *error);
    // Frame + override inputs, then Execute, then outputs:
    bool SetFrame(double frame, std::string *error);
    bool SetOverride(uint32_t slot, const double *values, size_t count);
    bool Execute();  // runs the cluster DAG, serial or threaded
    // Outputs: row-major 4x4 per joint, then deformed points (D3):
    const float *GetJointMatrices(size_t *count) const;
    const float *GetPoints(size_t *count) const;
};
```

Plus a C ABI shim (`rigexec_runtime_c.h`, no STL across the DLL
boundary) for the GDExtension, and a `RigExecBakedPlayback` C++
adapter in `rigExecImaging` (M2b) that feeds runtime outputs into
the existing Hydra pose-publish path.

CLI (extends the `rigExecPose` pattern, same flag style):

```
rigExecBake <stage> --rig <prim> --frames 1001,1002 -o char.rigexec
rigExecPose ... --verify-binary char.rigexec   # dynamic==baked==binary
```

## 5. Interpreter fidelity rules

- The runtime replays step bodies in clusterDAG order; floating-point
  results must be bit-identical to the baked path for the same frame
  (same kernels, same order, same operand values — the property the
  step graph already guarantees across orders).
- Cone re-execution (§7 of the baked design) is preserved: the binary
  carries cones + source descriptors, so unchanged frames skip
  clusters exactly as the baked path does.
- Diagnostics section is never on the execution path.

## 6. D4 worklist (refusals to implement, not keep)

Known `Refuse` sites in the bake path today:

1. `bakedProgram.cpp`: pose interpolator driver / driver-parent with
   no provider slot.
2. `bakedWeights.cpp`: weight-object composition cycle, missing
   weight prim, volume weight that is not a pose provider.
3. `bakedProgramImpl.h`: unsupported `rigExec:rotationBlend`.
4. `rigExecStandalone/system.cpp`: reverse solver-joint bindings need
   evaluator lowering (only if the baked path can produce them —
   verify, else document why out of reach).
5. Weight fields + volume placements publish empty on both paths
   today; the comparator already covers them, so the first
   generation that fills them is measured, not waved through.

Each gets implemented + a dedicated parity case; `--require-baked`
semantics extend to bake ("any refusal fails the bake naming the
feature") until the list is empty.

## 7. M1 slices (in order, each green before the next)

1. **Container + manifest + CLI**: header/section/string-table
   writer+reader, `BakeToBinary` returning manifest-only bytes,
   `rigExecBake` CLI skeleton, round-trip + golden tests. Proves the
   evaluator → binary → reader path end to end.
2. **Pose tables + steps + schedule**: serialize compose/solver/
   commit/interpolator tables, the step list, clusters, cones.
3a. **Geometry tables** (done): revisions, chains, derived, weights,
   packets, bindings, phases, chunks, partitions, topologies-by-value,
   falloff LUTs, delta descriptors. Proven over 6 fixtures incl.
   blendshape, volume weights and multi-chain.
3b. **Curvenet bind reconstitution** (done): the profile bind owns a
   factorization (no by-value form), so M2 re-binds from inputs; the
   weight side already retains its (bound*), the profile side needs new
   retention state filled at bind time, plus a curvenet fixture
   (12_CurvenetProfile.usda) in ctest. Per-frame layout streams for
   cache-refused cases ride the InputTable (slice 4).
4. **Inputs/overrides**: varying-input sampling, override slots,
   multi-frame parity.
5. **D4 gaps + full parity**: §6 worklist, then
   dynamic == baked == binary on every example stage.
