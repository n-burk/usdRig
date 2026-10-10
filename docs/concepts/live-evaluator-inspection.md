---
title: Live evaluator inspection
summary: Attach the optional rigExec_viz tool to usdview and read a session's operation graph and per-operation thread timings.
order: 60
---

The optional sibling `rigExec_viz` tool attaches to usdview's active imaging
session. Set `RIGEXEC_VIZ_ROOT` to its package root when using the normal
`bin/launch_usdview.bat` or `bin/usdview.sh` helper. Set
`RIGEXEC_VIZ_AUTOOPEN=1` to open its graph and thread trace panel on startup.

`RigExecImaging_LiveDebugJsonForStage` in `profilerApi.h` returns a versioned JSON
snapshot of an already-active session identified by a USD stage-cache ID and
rig path. Call it only on the stage owner thread, between evaluations. It does
not create a context, compile, evaluate, author, or change the current frame.
An empty rig path lists available roots. Playback-only sessions are not exposed
as live evaluators.

Pass the last `graph_key` and `generation` to omit unchanged data. Graph keys
identify the session's compiled program and binding epoch; a changed key requires
replacing the graph and discarding old trace history. Graph rows contain canonical
operation IDs, declared predecessor/successor edges, read/write ranges, and
cluster IDs. Range endpoints are inclusive, matching `Rig.op_graph()`.
Trace rows contain `[step, start_us, duration_us, thread, completion_sequence]`.
Timestamps use the engine's monotonic clock. Zero start and empty thread mean
that timing was disabled. `trace_phases` holds one `[memo_us, publish_us]` row
per trace row: the operation's memo before its body and its value publication
after it, so the operation occupied its thread from `start_us - memo_us` to
`start_us + duration_us + publish_us`. Both are zero when timing was disabled.
Do not infer scheduling overlap from completion order.
`concurrency_limit` reports OpenUSD Work's current limit, not observed occupancy.

The C API uses a size query followed by a copy with capacity greater than the
returned size (for NUL). A too-small buffer is untouched. A thread-local pending
reply keeps both calls on the same snapshot; consume it on the same owner thread.
An unknown context returns -1. Reads use the registry's existing coordination
boundary and introduce no new mutex or worker synchronization primitive.

`RigExecImaging_SetLiveOpTimingForStage` enables bounded, last-run stamps in the
existing operation storage. The unified scheduler and kernels remain unchanged.
It does not enable the accumulating profiler. Reading a trace formats thread IDs
and serializes records after execution, outside operation bodies. Disable timing
when the tool closes. Frozen snapshots do not inherit the timing opt-in.

The snapshot is the last live evaluator run, which can differ from the displayed
frame; compare `time` with `viewport_time`. Cache hits do not run operations.
Detached frame warming is not captured. A polling UI may skip intermediate runs
and must describe its trace as the last observed generation, not a complete
record of every frame or of end-to-end UI latency.

Verification: `testRigExecLiveDebug_serial` and `_parallel` exercise active-session
routing, incremental snapshots, timestamps and phases, recording shutdown,
unchanged source layers, and evaluation/profiler counters.
`testRigExecOpTracePython` and `_serial` cover Python trace timing, and
`testRigExecOpTrace_*` checks the memo and publication stamps against each
body, with op timing alone and with the profiler. The sibling's
`tests/test_usdview.py` checks the real Qt panel against the host's imaging
evaluator through `testusdview`.
