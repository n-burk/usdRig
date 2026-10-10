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
- **Background workers run frozen programs.** Every operation, including
  affine frame providers and external movers, warms from captured inputs on
  the workers; no worker opens or evaluates a stage. A rig whose inputs
  cannot all be frozen still caches each frame you visit, and once the
  workers drain the idle tick fills at most one missing frame of the
  requested range through the same compiled graph on the owning thread.
  That fill pauses during edits, drags and playback, and does not change
  the displayed frame.
- **Never serves the playhead from the pool.** A miss at the requested frame
  evaluates live on the calling thread and is never slower than with the
  cache off.

## What it costs

- **Memory, bounded per rig:** 256 MiB by default — about 80 full biped
  frames (the ±8 neighborhood plus a ±32 sweep) — with least-recently-used
  eviction past the cap. Long sessions cannot grow it.
- **CPU:** two background workers at below-normal priority. They run hot
  while the UI is idle and yield while it is busy.
- **One short burst per edit:** sampling the ±8 neighborhood's inputs at
  enqueue costs about 1.2 ms on the heaviest measured rig, then the pool
  drains it in about 25 ms.

## Viewport evaluation

usdview evaluates the compiled operation graph and can sample detached inputs
for background workers to fill the animation range. Every job executes frozen
state without querying the stage. Source edits invalidate incompatible entries
and discard saved static samples, including edits that retain the same program
and bindings. Visited-frame caching uses the same published pose values.

A rig that cannot compile or provide a complete frozen input vector reports the
reason. It does not select a different evaluator.

## Switches

| variable | default | what it does |
|---|---|---|
| `RIGEXEC_FRAME_CACHE` | `on` | `on`, `off`, or `warm-off` (serve cached frames, but fill nothing in the background). `off` restores exact current behavior — the rollback is one variable. |
| `RIGEXEC_ENABLE_PARALLEL_EVAL=0` | unset | Cache reads are still served; background fill is disabled. |
| `RIGEXEC_FRAME_CACHE_VERIFY=1` | off | Shadow mode: every cache hit is also live-evaluated and compared, and mismatches are reported as diagnostics. For debugging, not for production. |

## Limits worth knowing

- **Incomplete frozen inputs** prevent worker-thread warming; the host can
  still evaluate a valid scene program on its owning thread. An owning-thread
  fill that fails is retried only after a stage edit, a new range request or
  a cache clear. Compilation failures remain invalid poses with diagnostics.
- **`.rigexec` playback sessions** are unchanged and never consult the
  frame cache.
- The cache is **in memory only** — there is no on-disk persistence, so
  restarting the session starts cold.

## Implementation

See [architecture](../specs/spec.md), `libs/rigExec/frameCache.cpp`, and
`libs/rigExec/backgroundScheduler.cpp` for cache ownership and scheduling.
The owning thread samples a job's inputs before dispatch; the job runs on a
worker-owned arena and publishes only while its generation and time fence are
current. Value edits retire the affected times, structural edits retire every
time, and stage replacement or deactivation cancels the rig's outstanding
work.

## Verification

`bin/test/run_testusdview_framecache.bat` (or `.sh`) opens the animated arm in
usdview, waits for automatic range warming, and replays the range in both
directions. Pass another animated stage as its first argument. The native
`testRigExecImagingFrameCacheDefault` suite checks default-mode cache hits,
zero evaluator pulls on warmed frames, edit invalidation, and mode overrides.
`bin/test/run_testusdview_wrinkle_framecache.bat` (or `.sh`) checks Wrinkle
parameter edits, immediate dirty display, automatic rebuilding, and cached
geometry against a fresh evaluation in usdview.
