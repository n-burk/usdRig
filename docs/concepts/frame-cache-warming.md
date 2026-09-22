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
  re-evaluation — that is the speedup (about 4× a cold scrub on the biped
  in the default mode; `reports/frame-cache-measurements.md` §6 has the
  numbers).
- **Drag a control** and the frame under the cursor evaluates live on every
  tick, as today; warming never serves a neighboring frame's pose in place
  of the requested one, and it never makes a drag laggier. When you release
  (edit-commit), the neighbors re-warm in the background.
- **During playback** warming gets out of the way: the sweep skips frames
  the playhead already passed.

## What warming never does

- **Never shows the wrong frame.** A cached pose is bit-identical to a live
  evaluation of the same inputs, or it is not served at all — an unevaluated
  result is recoverable, a plausible wrong one is not. A new edit cancels
  in-flight warming for that rig, and stale results are dropped, never
  published.
- **Never blocks the viewport.** Cache fill and warming run on low-priority
  background threads; the UI thread never waits on them.
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

## Switches

| variable | default | what it does |
|---|---|---|
| `RIGEXEC_FRAME_CACHE` | `on` | `on`, `off`, or `warm-off` (serve cached frames, but fill nothing in the background). `off` restores exact current behavior — the rollback is one variable. |
| `RIGEXEC_ENABLE_PARALLEL_EVAL=0` | unset | Cache reads are still served; background fill is disabled. |
| `RIGEXEC_FRAME_CACHE_VERIFY=1` | off | Shadow mode: every cache hit is also live-evaluated and compared, and mismatches are reported as diagnostics. For debugging, not for production. |

## Limits worth knowing

- **Rigs that decline the bake** (see [Baked and dynamic
  evaluation](baked-vs-dynamic.md)) still benefit: frames you
  scrubbed yourself are remembered and served back. But there is no
  background warming for them, because the dynamic path cannot run off the
  UI thread.
- **`.rigexec` playback sessions** are unchanged and never consult the
  frame cache.
- The cache is **in memory only** — there is no on-disk persistence, so
  restarting the session starts cold.

## Sources

`reports/frame-cache-measurements.md` (the measured numbers behind the
defaults above); `docs/plans/per-frame-caching-system.md` (the full plan);
`docs/specs/baked-step-graph.md` (the cache-interaction appendix, for the
engineering detail).
