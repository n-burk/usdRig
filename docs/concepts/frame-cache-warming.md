---
title: What warming does
summary: The per-frame cache in one page: what warms, what you see, what it costs, and the switches.
order: 25
---

Scrubbing and playback read cached frames; the frame under your cursor
always evaluates live. That sentence is the whole contract. Warming is the
background work that keeps the cache full around the playhead, so a scrub
back over a range you already visited is instant instead of a
re-evaluation.

## What you see

- **First scrub over a cold range** computes each frame live, exactly as
  today, and keeps the poses it computed.
- **Scrub back** over a warmed range serves frames from the cache with no
  re-evaluation.
- **Drag a control** and the frame under the cursor evaluates live on every
  tick, as today; warming never serves a neighboring frame's pose in place
  of the requested one, and it never makes a drag laggier. When you release
  (edit-commit), the neighbors re-warm in the background.
- **Edit a mover parameter** and the strip shows affected frames as dirty
  while the edit is in progress. Once the edit settles, warming samples
  the changed inputs and refills those frames automatically.
- **During playback** warming gets out of the way: the sweep skips frames
  the playhead already passed.

## What warming never does

- **Never shows the wrong frame.** A cached pose is bit-identical to a live
  evaluation of the same inputs, or it is not served at all — an unevaluated
  result is recoverable, a plausible wrong one is not. A new edit cancels
  in-flight warming for that rig, and stale results are dropped, never
  published.
- **Background workers handle supported frozen programs.** Other operations
  warm through a privately owned stage and evaluator. The idle tick queues
  one missing frame when prior jobs have drained; it does not evaluate the
  live rig. Warming pauses during edits and playback and never changes the
  displayed frame.
- **Never serves the playhead from the pool.** A miss at the requested frame
  evaluates live on the calling thread and is never slower than with the
  cache off.

## What it costs

- **Memory, bounded per rig:** 256 MiB by default — about 80 full biped
  frames (the ±8 neighborhood plus a ±32 sweep) — with least-recently-used
  eviction past the cap. Long sessions cannot grow it.
- **CPU:** two background workers at below-normal priority. They run hot
  while the UI is idle and yield while it is busy.
- **Dynamic fallback:** one captured stage per context and one private
  evaluator per warmed rig. Capture runs on the stage's owner thread when
  fallback is needed. Value edits copy only the changed, fully composed
  properties; retained patches are bounded by the number of edited properties.
  Structural edits recapture the stage on the owner thread; they can cost
  more than a value edit.
- **One short burst per edit:** sampling the ±8 neighborhood's inputs at
  enqueue costs about 1.2 ms on the heaviest measured rig, then the pool
  drains it in about 25 ms.

## Viewport evaluation

usdview prepares an executable program by default so the background workers
can fill the animation range before playback. Explicit evaluation modes and
an authored `rigExec:baked = false` retain their requested behavior. Rigs that
cannot prepare or freeze a program queue the requested range one frame per
idle tick against a private stage. Every valid live result can also be cached as you visit frames.
These fallback entries use frame time and the stage edit serial, so adding
an operation does not require adding a separate cache input sampler.
Background sampling also discards its saved static inputs after a stage
edit, including edits that keep the same executable program and bindings.

Set `RIGEXEC_DYNAMIC_RUNS_PROGRAM=0` to opt out of the viewport default.
This leaves visited-frame caching and private-stage background warming
enabled, and preserves dynamic evaluation inside those workers.

## Switches

| variable | default | what it does |
|---|---|---|
| `RIGEXEC_FRAME_CACHE` | `on` | `on`, `off`, or `warm-off` (serve cached frames, but fill nothing in the background). `off` restores exact current behavior — the rollback is one variable. |
| `RIGEXEC_ENABLE_PARALLEL_EVAL=0` | unset | Cache reads are still served; background fill is disabled. |
| `RIGEXEC_FRAME_CACHE_VERIFY=1` | off | Shadow mode: every cache hit is also live-evaluated and compared, and mismatches are reported as diagnostics. For debugging, not for production. |

## Limits worth knowing

- **Rigs that decline the bake** (see [Baked and dynamic
  evaluation](baked-vs-dynamic.md)) use the private-stage fallback. Frozen
  programs remain the preferred route because they avoid a second stage and
  evaluator. Failed fallback frames retry after a new source revision or
  an explicit range/cache reset.
- **`.rigexec` playback sessions** are unchanged and never consult the
  frame cache.
- The cache is **in memory only** — there is no on-disk persistence, so
  restarting the session starts cold.

## Implementation

See [architecture](../specs/spec.md), `libs/rigExec/frameCache.cpp`, and
`libs/rigExec/backgroundScheduler.cpp` for cache ownership and scheduling.
`libs/rigExecImaging/fallbackStage.cpp` captures immutable property snapshots
on the owner thread and applies them only inside worker-owned stages. Worker
results use the live enqueue-time cache key and generation/time fences; edits,
undo, stage replacement, and deactivation cannot publish an older revision.

## Verification

`bin/test/run_testusdview_framecache.bat` (or `.sh`) opens the animated arm in
usdview, waits for automatic range warming, and replays the range in both
directions. Pass another animated stage as its first argument. The native
`testRigExecImagingFrameCacheDefault` suite checks default-mode cache hits,
zero evaluator pulls on warmed frames, edit invalidation, and mode overrides.
`bin/test/run_testusdview_wrinkle_framecache.bat` (or `.sh`) checks Wrinkle
parameter edits, immediate dirty display, automatic rebuilding, and cached
geometry against a fresh evaluation in usdview.
