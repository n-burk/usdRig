# Per-Frame Cache Measurements (Stream 0)

The numbers the per-frame caching plan's defaults are made from, measured
2026-09-18 with `tests/benchFrameCache.cpp` (built by CMake as
`benchFrameCache`, deliberately not registered with ctest: it prints
numbers and asserts nothing):

    benchFrameCache <examplesDir> [biped|9mesh|all]

Serial numbers run under `RIGEXEC_ENABLE_PARALLEL_EVAL=0`, which is the
mode the background workers will use (plan D4: workers run the baked
serial executor end to end and never call `Work*`). `RIGEXEC_BAKED_STEP_TIMING=N`
adds the library's own prologue/region/epilogue thirds on stderr.

Workstation: 13th Gen Intel i9-13900KF (24 cores / 32 threads), 63.7 GB RAM,
Windows, Release + Ninja. Timings are workstation numbers, not CI promises;
the ratios between them are what size the defaults.

Rigs:

- **biped**: `examples/biped/Biped_anim.usda`, frames 1..8 (378 providers,
  76 clusters, 11,500 bound inputs of which 24 varying).
- **9mesh**: the `MakeMultiMeshRig` construction from
  `testRigExecChainLevels` with 9 skinned meshes over two shared controls,
  animated (time samples at 1..40 on both control avars so consecutive
  frames genuinely re-evaluate instead of cone-skipping a static rig) and
  valid envelopes throughout, timed at frames 1..40 (2 providers,
  19 clusters).

## 1. Per-frame stored bytes (pose maps vs slot arenas, separately)

| rig   | pose maps (payload) | slot arena (per-frame) | stored total |
|-------|---------------------|------------------------|--------------|
| biped | 430,258 B (420.2 KiB) | 2,767,505 B (2.64 MiB) | 3,197,763 B (3.05 MiB) |
| 9mesh | 447,038 B (436.6 KiB) | 1,796,703 B (1.71 MiB) | 2,243,741 B (2.14 MiB) |

Pose bytes are stable frame to frame (two evaluated frames compared:
430,258 vs 430,258 on biped, 447,038 vs 447,038 on 9mesh), so the
per-frame stored-bytes number is a number, not a range.

What is counted, and what is not:

- **Pose**: every published map of a real evaluated pose (joint frames
  base/final, joint matrices, control frames, solver frames, moved
  properties, weight fields/frames, diagnostics). SdfPath keys count
  `sizeof(SdfPath)` (8); path strings are interned and shared, so they
  are excluded and said so. `std::map` node overhead is estimated at
  32 bytes/entry and reported beside the payload, never folded in
  (biped: 29,440 B over 920 nodes; 9mesh: 352 B over 11 nodes).
- **Arena**: the baked program's per-frame working state through
  `GetStepGraph` -- SSA frame versions, avars, matrices, property
  results, solver outputs and scratch, geometry outputs and packets,
  weight packets, chain buffers, last-run comparison buffers, cone
  state. Epoch structure (steps, clustering, cones, walk/commits,
  ladder tables when epoch-constant, invalidation index, topology and
  bind caches) is excluded: it is the program, not the arena a cached
  frame would retain.
- Two known imprecisions, both conservative except where noted:
  `lastAuxPoints` is a copy-on-write handle to a chain-published array
  whose buffer is also counted under the chain result, so one buffer
  per derived revision is double-counted (overstates the arena);
  `runSnapshots` was non-empty at measurement time but its revision
  inputs are unsized (understates the arena; Stream D sizes them when
  it retains arenas for real).

Dominant domains: on the biped the pose is 315 KiB of moved properties
plus ~100 KiB of joint/control frames, and the arena is 2.07 MiB of
geometry chains plus ~0.5 MiB of SSA versions and matrices. On the 9mesh
rig the pose is 436 KiB of moved points and the arena is 1.71 MiB of
geometry chains over 9 revisions.

**D2 go/no-go (read here): GO.** A full (pose + arena) biped frame is
3.05 MiB; the proposed 256 MiB cap below holds ~80 of them -- the +-8
neighborhood plus a +-32 sweep with headroom -- and ~119 full 9mesh
frames. Arena retention for cluster-level reuse fits the cap at a useful
frame count on both rigs, so Stream D ships cross-frame cone reuse, not
whole-pose memo only.

## 2. Baked serial frame cost (the warming budget)

Min-of-5 pass means over 40 animated frames each (200 frames per rig),
with the minimum clusters-run across the 200 frames proving every timed
frame ran the whole program rather than an empty cone:

| rig   | serial min | serial median | parallel min | parallel median | min clusters run |
|-------|------------|---------------|--------------|-----------------|------------------|
| biped | 3,093 us   | 3,269 us      | 1,082 us     | 1,143 us        | 75 of 76 |
| 9mesh | 165 us     | 169 us        | 187 us       | 205 us          | 16 of 19 |

Cold first frame out of a fresh program (serial): biped 5,677 us,
9mesh 785 us. Compile: biped 206 ms, 9mesh 3.3 ms. All 202 baked
generations per rig ran on the baked path (no dynamic fallback).

Notes:

- The serial number is the warming budget: one below-normal worker
  warms one biped frame in ~3.1 ms, one 9mesh frame in ~0.17 ms.
- The 9mesh rig is *faster* serial than parallel (165 vs 187 us):
  kernel-level parallelism costs more than it buys on two providers.
  This is the baked serial-default lesson the plan cites (D4), and it
  confirms workers must run the serial executor rather than `Work*`.
- The biped parallel number (1.1 ms) is the live-UI comparison point
  for Stream F's warming-on-vs-off latency bench, not a worker mode.

## 3. UI-thread sampling cost (bounds the neighbor radius)

The cost of sampling one frame's input vector the way the UI thread
will at enqueue (D3): every varying binding through its retained
`UsdAttributeQuery`, or through its resolved attribute when a property
chain stands on the walk -- the route the baked frame path reads -- at
a fresh time code, plus the blend-channel, operator-array, ribbon, and
chain-base reads the prologue makes outside the binding table.
Mean over 200 repeats:

| rig   | query | chain | blend | op-array | ribbon | override-reg | sample | base pts | base cost | sample+base |
|-------|-------|-------|-------|----------|--------|--------------|--------|----------|-----------|-------------|
| biped | 12 | 12 | 0 | 222 | 0 | 11,278 | 74.3 us | 315,336 B over 2 reads | 0.2 us | **74.4 us/frame** |
| 9mesh | 2 | 0 | 0 | 0 | 0 | 54 | 0.4 us | 446,364 B over 9 reads | 0.7 us | **1.1 us/frame** |

Per-read cost: biped 301.9 ns, 9mesh 177.3 ns.

Notes:

- The biped's 222 operator-array reads dominate its read count; the
  11,278 override-registered inputs are standing registrations the
  control digest covers, not per-frame reads.
- Chain base points are the bulk of the sampled *bytes* (315 KiB biped,
  446 KiB 9mesh) but cost ~1 us to read: the sampling budget is reads,
  not bytes.
- The enqueue burst for a +-N neighborhood is N*2 frames of sampling
  on the UI thread: +-8 costs ~1.2 ms on the biped, ~18 us on 9mesh.

## 4. Proposed defaults (from the numbers above)

| default | value | in code | derived as |
|---------|-------|---------|------------|
| per-rig byte cap | 256 MiB | `kRigExecFrameCacheDefaultByteCap` (`libs/rigExec/frameCache.h`) | ~80 full biped frames (3.05 MiB each) -- the +-8 neighborhood plus a +-32 sweep -- or ~119 full 9mesh frames, or 600+ pose-only frames (420 KiB each) if a rig ever retains poses without arenas |
| background workers | 2 | `kRigExecBackgroundSchedulerDefaultWorkers` (`libs/rigExec/backgroundScheduler.h`) | two below-normal workers drain the biped +-8 neighborhood (16 x 3.1 ms / 2) in ~25 ms without contending with the UI thread; the 9mesh neighborhood drains in ~1.4 ms |
| neighbor radius | +-8 (16 frames) | `kRigExecFrameCacheDefaultNeighborRadius` (`libs/rigExec/backgroundScheduler.h`) | one 1.2 ms sampling burst at enqueue on the biped (~18 us on 9mesh) and ~25 ms of background drain on two workers -- a scrub step's neighborhood is warm before the next drag tick |

Rationale notes:

- The cap is sized in *full* (pose + arena) frames because D2 went GO:
  the cache retains what sparse reuse needs, not just what scrub reads.
  Pose-only capacity (600+ frames) is the fallback arithmetic if a rig
  ever disqualifies from arena retention.
- Two workers, not four: the biped neighborhood drains in a single
  frame budget (25 ms) on two, and every worker past the second is
  contention with the UI thread for no interactive gain. Revisit only
  if Stream F's warming-on-vs-off bench shows the UI never notices and
  a sweep benchmark wants the throughput.
- Radius 8 is the sampling bound made concrete: the UI-thread enqueue
  burst stays ~1 ms on the heaviest measured rig. A wider radius is
  cheap in background time (3.1 ms/frame serial) but spends UI time
  linearly (74 us/frame); radius lives where the UI cost stays under a
  millisecond.

## 5. Reproducing

From `D:/work/usdRig-framecache` with the Release build configured
(see `bin/build_rigexec.bat` / CI `usdrig.yml`):

    # serial (warming budget + sampling; workers run serial)
    $env:RIGEXEC_ENABLE_PARALLEL_EVAL=0
    ./build/benchFrameCache ./examples all

    # parallel (live-UI comparison point)
    Remove-Item Env:RIGEXEC_ENABLE_PARALLEL_EVAL
    ./build/benchFrameCache ./examples all

Byte totals are deterministic across runs (same stage, same program);
timings vary run to run -- rerun and take min-of-5 pass means as the
bench does. `RIGEXEC_BAKED_STEP_TIMING=1` on stderr splits a frame into
prologue/region/epilogue thirds when a timing needs explaining.

## 6. Stream F warming benches (measured 2026-09-19)

What warming does with the defaults, measured with
`tests/benchFrameCacheWarm.cpp` (built by CMake as `benchFrameCacheWarm`,
deliberately not registered with ctest: it prints numbers and asserts
nothing, because wall-clock comparisons flake on shared CI runners):

    benchFrameCacheWarm <examplesDir> [all|scrub|edit|latency|memory|stress|trace] [seconds]

Same workstation as sections 1-4. The primary rig is a tiny in-memory rig
(one 64-point skinned mesh over two animated controls, frames 1..16, 2
providers, 3 clusters), which samples, so every key below is a real
control-state digest over real sampled inputs. The scrub bench also runs
the biped (frames 1..8) when `examplesDir` is given, and the biped samples
too. Numbers are medians; p95 where the plan asks for it. Cross-checks
against section 2 first: the bench's live-eval reference reproduces it
(parallel biped 1088 us vs 1082-1143 us; serial 3398 us vs 3093-3269 us),
so the cold/warm ratios below divide honest denominators.

### 6.1 Cold-vs-warm scrub

Cold is live eval + sample + digest + publish per frame (the first scrub
over a cold range); warm is sample + digest + lookup per frame (the scrub
back), five reversed passes. The warm anatomy splits sampling from
digest+lookup over 40 repetitions each.

| rig | mode | live eval | cold | warm | warm = sample + digest/lookup | cold/warm | live/warm |
|-----|------|-----------|------|------|-------------------------------|-----------|-----------|
| tiny | parallel | 5.2 us | 10.0 us (p95 41) | 3.0 us (p95 3) | 1.0 + 2.0 us | 3.3x | 1.7x |
| biped | parallel | 1088 us | 2130 us (p95 2669) | 583 us (p95 1054) | 159 + 410 us | 3.7x | 1.9x |
| biped | serial | 3398 us | 4090 us (p95 4504) | 560 us (p95 776) | 164 + 403 us | 7.3x | 6.1x |

Zero misses and zero invalid serves in every warm pass (tiny: 120 hits;
biped: 80 hits per mode; cold published every frame).

**The >=10x target is missed: 3.7x parallel, 7.3x serial (cold/warm).**
The anatomy says why: the warm path is digest-dominated by design. D1
recomputes the digest from the stage at every lookup, and on the biped
that re-hash over 11,500 inputs costs ~410 us of the ~583 us warm frame --
70% of the cost of the thing meant to avoid the 1088-3398 us evaluation.
Sampling itself is ~160 us here (vs 74 us in section 3, which times raw
reads rather than the vector-building sampler). The ratio is still a real
speedup, and it grows where evaluation is dearer than hashing (serial more
than parallel), but a warmed scrub will not reach 10x until the digest
stops costing more than a third of an evaluation. The follow-up is a
digest that memoizes per-input hashes and re-folds only what the notice
touched; it is not in this branch.

### 6.2 Edit recompute cost

One control-sample edit (`avars:tx = 999` at frame 8) across the 16 tiny
frames, then sparse planning for what moved:

- Affected-frame scan: 60 us total, 3.8 us/frame -- 1 of 16 affected,
  the edited frame only.
- Affected-index build: 8 us, one-time per epoch (3 clusters).
- Sparse plan for the affected frame: 0.8 us/plan -- Partial, run 3 of 3
  clusters (the tiny rig's cone is the whole graph; the memo still
  applies), memoized selection used, +1 index walk across 200 plans
  (memo: 199 hits, 1 miss, 1 store).
- A second edit of the same control re-runs the memoized selection with
  +0 walks.
- An override drag (all-frames-moved endpoint): 16 of 16 affected, plan
  Partial over all 3 clusters -- the conservative answer for an unmapped
  control, by contract.

### 6.3 UI-eval latency, warming on/off

200 live evals per series on the tiny rig, A/B/A (warmup, off, on, off
again), with the pool draining throughout the on-series: plumbing (trivial
work, 80 jobs) and loaded (3.1 ms synthetic serial jobs, 32 jobs over two
workers -- every sample taken under load).

| series | median | p95 | max |
|--------|--------|-----|-----|
| warming off | 6.0 us | 9.0 us | 47 us |
| warming on (plumbing) | 6.0 us | 7.0 us | 15 us |
| warming on (loaded) | 5.0 us | 6.0 us | 67 us |
| warming off (repeat) | 5.0 us | 6.0 us | 70 us |

No warming effect at any percentile: the on-series match off within a
microsecond, and the repeat matches the first off. The maxima (47-70 us)
appear in every series including off -- machine noise, not warming. Two
caveats, both stated: tiny-rig evals (~6 us) are too fast to contend with
anything, so this answers the plumbing question, not the biped-drag one;
and the loaded jobs are synthetic spins, labeled as such in the bench
output. The biped-drag comparison is the manual validation-plan item, not
a number this bench claims.

### 6.4 Memory under the cap

- Tiny frame: 1,378 bytes of pose maps, measured.
- 64 frames at the default 256 MiB cap: 64 entries, 88,192 bytes held,
  0 evictions.
- Squeezed to a 64 KiB cap with 64 more frames published: 47 entries,
  64,766 bytes held (under the cap), 81 evictions counted.
- The Stream 0 capacity arithmetic, recomputed from the constants in
  code: GO, 83 full frames at the cap (3,197,763 B each) -- the report's
  "~80" (§1), confirmed rather than copied.

### 6.5 UI-vs-warming stress (the TSAN bench)

Two seconds of UI-vs-warming churn: the UI thread evaluates live,
publishes, commits, and cancels while a second thread hammers
enqueue/cancel; a TSAN-instrumented Linux build of the same bench is the
race check (`benchFrameCacheWarm <examples> stress <seconds>` runs it
standalone -- it is not a ctest gate on any platform).

2 s run: 268,585 UI evals, 33,574 publishes, 4,196 commits, 0 invalid;
1,077,116 churn enqueues; scheduler 1,055,466 completed, 2,297
dropped-stale, 68,676 canceled, 2,794 declined, 0 shed, queue drained to
idle at the end. The Windows run above exercises the same surface without
the instrumenter; the TSAN verdict belongs to a Linux TSAN build and is
recorded when one runs it.

### 6.6 Profiler lanes demo

`benchFrameCacheWarm <examples> trace` scripts a cold pass, a warm pass,
and a commit/cancel pair on the manual pool while recording the Stream F
profiler lanes, then writes `frame-cache-warm.trace` (Chrome Trace Event
JSON, opens in Perfetto or chrome://tracing). This run: 10 profiled
events -- 3 complete scopes (`benchCold`, `benchWarm`, `benchCommit`),
4 `cacheHit` instants on the `frameCache` lane, 2 `warmQueue` counter
samples and 1 `warmCancel` instant on the `scheduler` lane.

### 6.7 Final defaults (confirmed in code, 2026-09-19)

The section 4 proposal stands unchanged; every value below was re-read
from its header and printed by the bench (`memory` mode) on the
measurement date:

| default | value | in code | status |
|---------|-------|---------|--------|
| per-rig byte cap | 256 MiB | `kRigExecFrameCacheDefaultByteCap` (`libs/rigExec/frameCache.h` = 268435456) | confirmed; holds 83 full biped frames |
| background workers | 2 | `kRigExecBackgroundSchedulerDefaultWorkers` (`libs/rigExec/backgroundScheduler.h` = 2) | confirmed |
| neighbor radius | +-8 (16 frames) | `kRigExecFrameCacheDefaultNeighborRadius` (`libs/rigExec/backgroundScheduler.h` = 8) | confirmed |

Reproducing the Stream F numbers (from `D:/work/usdRig-framecache` with
the Release build configured):

    ./build/benchFrameCacheWarm ./examples all        # everything, 5 s stress
    ./build/benchFrameCacheWarm ./examples scrub      # section 6.1
    ./build/benchFrameCacheWarm ./examples latency    # section 6.3 (+p95)
    ./build/benchFrameCacheWarm ./examples stress 10  # section 6.5 (TSAN: Linux TSAN build)

## 7. Production warming repair (measured 2026-09-19)

Synthesis found the production triggers enqueuing zero jobs: the C-API
`RigExecImaging_OnEditCommitted/OnIdle` called the registry triggers with no
runner, the runner defaulted to null, and `BuildWarmWork` declined every
frame -- only injected test kernels ever warmed. The repair, in
`libs/rigExec/frozenContext.{h,cpp}` (production runner factory
`RigExecMakeProductionStepRunner`, chain-sample marking),
`libs/rigExecImaging/registry.{h,cpp}` (null runner now means production;
chain-carrying vectors decline until the chain hook lands), and
`libs/rigExecImaging/bridge.{h,cpp}` (`NoteWarmingEnqueued`: the enqueue-time
freshness proof, without which a warmed entry is unreachable -- proofs were
recorded only after live evaluations, which memoize their own result anyway):

- `OnEditCommitted()` enqueues 80 jobs (16 neighbors +-1..8 plus the 64-frame
  sweep +-9..40, never the playhead); `OnIdle()` enqueues the 64-frame sweep;
  playback rigs still enqueue nothing. Proven by
  `TestProductionTriggerPathEnqueuesAndFences`, which also proves stale
  production work drops on a generation bump without publishing.
- A background completion plus its proof serves with zero evaluator pulls,
  proven by `TestWarmedCompletionServesWithoutEvaluating` (fails without the
  proof: the lookup misses and evaluates live).
- Two handoffs remain, both specified at their call sites and both outside
  this repair's files: the program-side frozen executor (the production
  runner declines until the region can run serially from sampled inputs
  against worker-owned slots -- running the live program on a worker would
  race the UI thread, read the stage mid-author, and leak `Work*` past
  unscoped launch sites), and the evaluator-side chain-sampling entry (run
  the property-chain prologue for the job's time on the UI thread).

Re-measured warm scrub (`benchFrameCacheWarm ./examples scrub`, same
workstation as section 6; medians, p95 in parentheses):

| rig | mode | live eval | cold | warm | warm = sample + digest/lookup | cold/warm |
|-----|------|-----------|------|------|-------------------------------|-----------|
| tiny | parallel | 5.5 us | 9.0 us (p95 36) | 3.0 us (p95 3) | 1.0 + 2.0 us | 3.0x |
| biped | parallel | 1049 us | 1802 us (p95 1910) | 564 us (p95 658) | 161 + 405 us | 3.2x |
| biped | serial | 3199 us | 4112 us (p95 5205) | 561 us (p95 646) | 156 + 394 us | 7.3x |

Section 6.1 read 3.3x / 3.7x / 7.3x: unchanged within run-to-run variance,
as expected -- this bench drives a manual scheduler with trivial work, so it
measures the UI-memoization lookup path, which the repair does not touch
(no regression there), rather than background-worker warming. The >=10x
target is still missed for the section 6.1 reason (the warm frame is
digest-dominated: ~405 us of ~564 us on the biped). End-to-end
background-warmed scrub speedups stay unmeasurable until the frozen executor
above lands; what is measured instead is the pipeline that will serve them
(zero-pull served completion, above).

## 8. Digest mixer follow-up (measured 2026-09-19)

The byte-at-a-time FNV-1a digest fold is now a word-at-a-time splitmix64
finalizer with FNV-style chaining (`_MixWord` in `frameCache.cpp` and
`frozenContext.cpp`) over the identical logical stream -- same call order,
same tags, same counts. The chaining multiply is load-bearing, not style:
an xor-only fold is commutative, so duplicate words cancel anywhere in the
stream (`[W,W]` folds to the seed whatever `W` is) and permutations
collide; `TestDigestSeesDuplicateValues` pins both, and the avar-region
shape that caught it (`TestPatchFrozenAvarConstants`, whose constant lives
in two digested locations) stays green. No test hardcodes a digest value;
the sensitivity suites assert ==/!= only, so the new values needed no
updates.

Re-measured warm scrub (`benchFrameCacheWarm ./examples scrub`, same
workstation; medians over two runs):

| rig | warm digest+lookup before | after |
|-----|---------------------------|-------|
| tiny | 3.0 us | 2.0 us |
| biped | ~475 us | ~243 us |

The digest half roughly halved, but the warm frame is now
sample-dominated, and sampling itself measures ~10x its section 7 value on
this workstation (~1600 us vs 161 us on the biped, stable across runs with
untouched binaries). That gap predates this pass and is unexplained --
either the workstation or the report's environment differs in a way that
moves USD read cost -- so the >=10x target stays missed and the memoized
per-input digest from section 6.1 remains the prescribed fix, now joined
by a sampling investigation.

## 9. Release-commit cost (measured 2026-09-19)

`tests/benchCommitLag.cpp` (built, not registered, like the other benches)
simulates a manipulation release at a held playhead on the biped: one
avar time-sample edit plus the release flush's `OnEditCommitted` burst
(section 7's 80 jobs, all sampled on the calling thread). Medians over
five drained rounds, `RIGEXEC_FRAME_CACHE=on`:

| half | cost | of which |
|------|------|----------|
| edit (`Set` + notices + live re-eval) | ~10 ms | -- |
| commit (`OnEditCommitted`, 80 enqueued) | ~136 ms | ~121 ms sampling (1.52 ms/frame) + ~11 ms control digests (134 us each) + ~4 ms per-frame overhead (snapshot refresh, chain-currency checks, epoch digests at 33 us each) |

A second commit with no new edit still costs ~55 ms (37 re-sampled
frames): completions from the drained burst re-warm rather than skip. The
native notice addition from the gap-closure pass (`NoteCaptureIndex`) is
~33 us of the edit half -- negligible. The commit half is what the usdview
release trigger used to run on the next event-loop turn; it now runs from
a restarting 120 ms idle timer (plan follow-up record), so the gesture
stays responsive and the ~136 ms lands in the post-edit pause. The
per-frame sampling was 89% of that pause as measured -- but section 10
re-attributes it: most of the burst was re-verification and table walks,
not reads.

## 10. Burst preparation (measured 2026-09-19)

Moving the burst off the main thread was considered and refused: sampling
reads the live USD stage, the UI thread can author at any moment, and USD
forbids concurrent read-during-write, so workers cannot sample and no
cheap snapshot or safe lock placement exists. Instead the bench gained a
pinned-route breakdown and a static/varying composition probe, which
re-attribute the ~141 ms biped burst per frame (~1.77 ms):

| part | cost/frame | share |
|------|------------|-------|
| chain-currency re-verification (x3: refresh, explicit, sampler entry) | 936 us | 53% |
| binding-table walks over epoch constants (~11.5k visits) | ~90 us | 5% |
| static array re-reads (843 KB, 241 of 253 samples static) | ~110 us | 6% |
| control digest (843 KB fold + sort) | 141 us | 8% |
| chain hook (12 varying values) | 63 us | 4% |
| epoch digest (x2: key + proof scope check) | 66 us | 4% |
| packets, Adds, proof record, enqueue, contention | ~364 us | 20% |

A burst is a synchronous UI-thread span, so everything except the hook,
the varying reads, and the per-frame enqueue is burst-fixed. Preparing it
once (verified bindings, per-table visit sets, epoch digest, static
sample maps, two-level digest with memoized level-1s and a pre-sorted
order -- `RigExecBurstSampleCache`) moves the commit from ~141 ms to
~20 ms (7.1x) with the same 80 jobs, and the no-edit re-commit from
~55 ms to ~2 ms. The residual is fundamental per-frame work. The section
8 sampling "anomaly" is retired as a mystery: per-frame reads were never
the whole story, and the memoized-digest future work is now half done
(burst-scoped; the lookup-path half remains).
