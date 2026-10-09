# Compiling rigExec programs to native code

This note asks whether the rigExec evaluator should be replaced, or
complemented, by compiling a baked rig to native code (LLVM or a similar
backend) and running that code in parallel. It is an investigation. It
does not change the evaluator, the runtime, the schema, or the tests.

The short answer: keep the interpreter. The `.rigexec` program is already
a static dataflow whose bodies are native C++ kernels. A compiler does
not make the hot kernel faster, and it fights the bit-identical contract,
plugin movers, and edit-time updates. Parallelism that pays is coarser:
several characters or a frame range, each with its own reader, and a
threaded split of a large skin. Dispatching every pose operation is
slower.

Measurements below were taken on a 4-thread Xeon guest with g++ 13.3
`-O3 -fno-fast-math -ffp-contract=off`, the same floating-point flags the
runtime uses. OpenUSD is not installed in the environment that produced
them, so the biped and ball stages were not baked or played through
`Execute`. The Godot tutorial's `rolling_ball.rigexec` is format 17; this
branch's reader is format 18 and refuses it. The 26,276-point figure is
the authored body mesh in `examples/biped/Biped_body_model.usda`, which
matches the calibration comment in `libs/rigExec/bakedSchedule.cpp`.

The probe that produced the tables lives in
[experiments/native-eval](../../experiments/native-eval/README.md).

## How evaluation works today

A rig is discovered under one `RigExecRoot` and compiled into one
operation graph. Native evaluation samples scene inputs on the owning
thread, then runs that graph. Frozen evaluation samples first and runs
the same graph on detached state. A `.rigexec` file is that compiled
program plus its static tables, as one FlatBuffer (`REXB`, format
version 18). Nothing in the file varies with time. An input's default is
its value at bake time. See
[Evaluation and independent checks](../concepts/baked-vs-dynamic.md) and
`libs/rigExecBinary/rigexec.fbs`.

### Program

Each operation declares the values it reads and writes. The shared
compiler in `libs/rigExecGraph/opGraph.cpp` binds each read to its
producer, rejects or sets aside cycles, and emits one canonical order.
Clusters are a coarsening of that order: a linear chain may share a
cluster only when the intermediate values have no outside readers, so a
cluster boundary adds no cross-branch dependency. Zero grain, which is
what `RigExecCompileOpGraph` lowers with, makes one cluster per
operation. The baker then re-lowers with a cost model
(`RigExecBakedScheduleGrainUs`): about four clusters per thread, clamped
to 5–50 µs. Those costs and the critical path are stored on the file.

Step kinds cover pose composition, solvers, constraints, propagation,
pose interpolators, property revisions, space expressions, weight fields,
and geometry (static assembly, influence fold, per-chunk deformation,
fuse, chain status, derived normals and extent). Geometry revision
opcodes include matrix, skin, blend shape, lattice, ribbon, wire, delta
mush, wrinkle, and external. Playback walks steps in an order the file's
own edges already satisfy; `Open` refuses a graph that order does not
match.

### Dispatch

`RigExecExecuteOpGraph` is the one readiness loop. It seeds operations
whose inputs changed, expands the successor cone, and runs a cluster when
its candidate predecessors have finished. With no `dispatch` callback the
loop is serial. The USD baked path installs a `WorkDispatcher` when
parallel evaluation is on and `RIGEXEC_BAKED_SCHEDULE` is `parallel`
(the default). The zero-USD runtime does not install a dispatcher:
`RigExecRuntimeReader::Execute` calls the serial loop. The reader holds
no locks; the header's contract is one reader per thread. The Godot
player in `docs/examples/godot_rolling_ball.zip` calls `Execute` once.

Inside a geometry step the USD mover graph splits point ranges with
`WorkParallelForN` when the count is at least 4,096 and the grain is 512
(`libs/rigExec/parallel.h`, `moverGraph.cpp`). The runtime port of the
same kernels runs the range in one loop. Skin and matrix take an SSE2
path when `RIGEXEC_ENABLE_SIMD` is on, which is the default. Dual
quaternion skinning on the USD side is split the same way; a comment in
`moverGraph.cpp` records that, before that split, skinning the 26,276-point
body on one thread was the largest cost in a drag.

### Data

Points are arrays of 3-float vectors. Skin indices and weights are
interleaved, `point * elementSize + slot`. Influence matrices are
row-major, row-vector, and the SIMD skin narrows them to float rows of
16. Slot domains (avars, pose versions, matrices, property results,
chain points, and the rest of the `SlotDomain` enum) are dense tables
addressed by integer ids. Input memos are strings of those bits, rebuilt
for every operation on every `Execute` and compared to the previous run.
Unchanged outputs stay. A clean retained output stops the wave.

### Where time goes

The baker's cost table, fitted on example rigs including the biped, is
the quantitative model the schedule already trusts
(`kStepCosts` in `bakedSchedule.cpp`). Pose operations are small:
compose is a few hundredths of a microsecond per joint, a constraint's
fixed cost is about 1.2 µs, a solver is a few tenths of a microsecond
per control. Geometry is different. A skin chunk pays about 11 µs before
it touches a vertex, because it gathers and narrows its influence
matrices, then about 0.0007 µs per vertex-influence. The comment on that
row says the biped's chunks carry 30 to 58 influences, and that copying
26,276 points is about 9 µs. An earlier fit that treated each parallel
chunk as if it skinned the whole mesh estimated 1972 µs of serial work
for a program whose wall time on a 20-core machine was 531 µs. Those
are that machine's calibration notes, not timings from this run.

On this 4-thread guest, a faithful SSE2 linear-blend of 26,276 points,
four influences, and a 137-joint palette (one weight left at zero so the
kernel's zero-weight skip stays live) takes about 102 µs. That is the
body the runtime already runs.

## Can the program be compiled?

Mostly the structure can. The part that would have to stay interpreted,
or stay a call, is large enough that a whole-program compiler is a poor
trade.

**Static structure.** After bake, the operation graph, cluster edges,
skin topology, blend-shape deltas, and wrinkle connectivity are fixed.
Property operations are an enum chosen at bake. A compiler could turn
each step kind into a direct call, and a skin with a known element size
into a monomorphic loop. Doing that in C++ changed the 26,276-point skin
from 103 µs to 102 µs.

**Data-dependent control.** Input values change per edit and per sample.
Space switches select a source from an authored index. Movers honor
enable and envelope. IK and aim take geometric branches. Wrinkle's
iteration count and stiffness are inputs. The wrinkle kernel is a
reversing sweep over constraints: iteration *n* walks the edges forward
and iteration *n+1* walks them backward, then projects points
(`libs/rigExecMath/wrinkleKernel.h`). That answer depends on the serial
order. A parallel or reassociated wrinkle is a different solver.

**Plugins.** External movers are opcode 16. The file stores opaque epoch
and frame bytes. The host installs a `prepare`/`apply` function pair
(`libs/rigExecBinary/external.h`). `apply` revises points in place and
must not keep state between calls. A compiler can emit a call. It cannot
inline a library that is not present at bake time, and a Godot build
that never links USD still has to load that plugin the same way. See
[External mover plugins](../concepts/external-movers.md).

**Per frame versus per edit.** A structural scene edit rebuilds
declarations on the USD side. Playback of a `.rigexec` does not. The
closure then reruns the cone of changed inputs. The first `Execute`
seeds the graph; later ones skip bodies whose memos match. A compiled
image that always runs every operation does more work on a held frame
than the interpreter. `TouchAnimatedInputs` sets `animatedTouched`, and
the successful run clears it. No expression in the runtime reads the
flag. A time change reruns work when the sampled input bits change,
which is what the input sampler writes.

**Bit-identical results.** The runtime header requires floating-point
results to be bit-identical to the baked path for the same inputs.
`rigExecPose --verify-binary` checks dynamic, baked, and binary.
Comparisons are `memcmp`, which includes NaN payloads and the sign of
zero. The format refuses scalar float fields in tables because a default
of `+0` would erase `-0`. Tests such as `testRigExecRuntimeGeometry` and
`testRigExecRuntimeInputs` keep `-0` weights distinct from `+0`. The
skin loop skips a weight of zero so that `0 * NaN` does not poison the
sum. Global flags are `-ffp-contract=off` and, on the runtime,
`-fno-fast-math`, because contraction breaks those comparisons. Any
backend has to preserve that operation order. The ahead-of-time skin
below matched the in-process kernel with `memcmp` under those flags. That
is the same algorithm compiled twice, which is a weak test of a different
IR pipeline.

## Options

### LLVM ORC JIT

ORC would lower the graph at load or after a structural edit and jump to
the result. The skin measurement says the generated code is the same
speed as the kernel already in the binary: 101.9 µs ahead of time versus
101.8 µs in process, after a 54 ms `g++ -shared` of that one function.
A rig module is larger, and an interactive edit already pays for USD
compilation. Fifty milliseconds on top of that is several frames of a
100 µs skin.

The dependency is heavy. Debian's `libllvm18` package metadata reports
about 118 MB installed. The tutorial's Windows release addon DLL is 2.6 MB.
Godot playback must not need USD; it also should not need a compiler.
Debugging a fault means mapping JIT addresses back to a step, on top of
the FlatBuffer the interpreter can already name with
`RigExecFormatStepLabel`. Plugins remain indirect calls.

### Ahead-of-time compile at bake

Bake could emit C or object code next to the FlatBuffer. That moves the
54 ms (and much more, for a whole rig) off the interactive path and onto
export. The runtime would still need the interpreter for rigs that have
not been exported, for plugins, and for any host that does not ship the
matching object format. Linux, Windows, and macOS each need a binary,
and the Godot extension would grow a loader for them. The measured skin
does not get faster. The interesting AOT result is the compile latency
and the proof that a second compilation can be bit-identical when the
flags match.

### MLIR or a Halide-style scheduler

Those tools earn their keep on regular grids with explicit schedules.
This graph mixes tiny pose operations, indexed gathers (skin), topology
(smooth, delta mush), and a serial constraint sweep (wrinkle). A new IR
would have to re-prove the bit-identical order the C++ kernels already
encode, including the zero-weight skip and the wrinkle reversal. The
existing cluster lowering is already a scheduler: it coarsens chains up
to a grain and refuses edges that would add a false dependency.

### ISPC or more SIMD

Linear-blend skin and the rigid matrix mover are already SSE2, with a
scalar fallback so a non-x86 build matches the reference kernel. A
further SIMD dialect helps a kernel that is still scalar and hot. Dual
quaternion skinning was that kernel on the USD side until the point
range was split; the runtime still has its own port. ISPC does not
remove the indexed gather, and it must keep the zero-weight skip if the
`memcmp` tests are to pass. It is a per-kernel choice, made after a
profile of a format-18 character, and it is independent of compiling the
graph.

### A more parallel interpreter

This is the option the code is already shaped for.

The shared executor accepts a dispatcher. The USD baked path uses it.
The runtime's `Execute` does not. Clusters in the file already carry a
cost and a critical path. Point ranges above 4,096 are independent
subproblems in the USD kernels; the runtime runs them serially and says
so in the geometry comments ("the serial loop is the parallel loop's
answer"). Several characters are several readers. Wrinkle and the rest
of evaluation are stateless across frames: each `Execute` starts from
the incoming points, so a frame range is independent readers too
([Compression-driven wrinkles](../concepts/wrinkle-deformation.md)).

What that parallelism is worth was measured directly.

## Measured parallelism

### One large skin

26,276 points, 4 influences, 137 joints, SSE2, zero-weight skip live.
Median of 20 runs after warmup.

| Loop | Median |
| --- | ---: |
| Dynamic element size | 103 µs |
| C++ loop fixed at 4 influences | 102 µs |
| Same loop, `g++ -shared`, `dlopen` | 102 µs |
| Persistent pool, 4 threads, split the points | 63 µs |

The ahead-of-time image matched both C++ loops with `memcmp`. Compile
of that one function was 54 ms. Four threads on one skin are 1.6×. A
one-thread trip through the same pool was slower than the direct call
(median 190 µs), so the win is the split, and only once the range is
large. Two threads landed at 116 µs, still behind the direct 102 µs.

### Several characters, and a pose-sized graph

`RigExecExecuteOpGraph` on synthetic graphs, same flags. "Light" work is
eight dependent multiply-adds per operation. The straight loop is those
multiply-adds with no graph.

| Graph | Serial executor | With a 4-thread pool | Straight loop |
| --- | ---: | ---: | ---: |
| One chain of 2,000 light ops (2,000 clusters) | 55 µs | 1,681 µs | 28 µs |
| 64 chains × 32 light ops, one cluster per op | 55 µs | 1,940 µs | |
| Same graph packed to 64 clusters | 41 µs | 170 µs | |
| 8 independent skins of 1,024 points | 34 µs | 26 µs | |

| Work | Serial | 4 persistent threads |
| --- | ---: | ---: |
| 4 skins of 26,276 points | 887 µs | 223 µs |

A chain has nothing to parallelize, and the dispatcher made it about
30× slower. Packing the wide light graph into 64 clusters was still
slower in parallel than in serial, because each cluster is a few dozen
multiply-adds. Eight small skins were 1.3×. Four full bodies were 4.0×,
which is the machine. The executor on the long chain was about 2× a
straight loop of the same arithmetic: that is memo, virtual-style
callbacks, and cluster bookkeeping, on top of work that is already too
small to schedule.

### Fusing a stack of matrix movers

Four rigid blends of 4,096 points. Separate SSE2 passes were 56 µs. A
scalar fused pass of the same math was 77 µs and bit-identical. An SSE2
fused pass, keeping the point in registers across the four transforms,
was 44 µs and bit-identical: 1.3×. Fusion helps when it keeps the SIMD
the interpreter already has. A compiler that scalarizes the stack loses.

### What a realistic frame can gain

On one character the wide part of the graph is the set of skin chunks
whose influence sets have both landed, plus independent branches
(another mesh, a property chain that shares no value). Pose stacks on
one joint are a chain. The cost model already estimates
`serialCost / criticalPathCost` for that shape and stores it on the
file; the runtime does not use it to dispatch.

A single 26k skin at 102 µs, split 4 ways, saves about 40 µs. If the
rest of a biped-scale frame is a few hundred microseconds of serial
pose and chunk setup, the frame speedup is well under 1.5× on four
cores. Four such characters, or four frames of an export, approach 4×
because the readers do not share a chain. That is the realistic
speedup. A JIT of the same kernels does not add to it.

## Recommendation

Leave the interpreter as the evaluator. Complement it in three stages.
Stop when a profile of a re-exported character says the remaining time
is inside a kernel the C++ already specializes.

**Stage 1. Dispatch expensive clusters in the runtime.** Reuse
`RigExecExecuteOpGraph`'s `dispatch`/`wait` pair, which the baked USD
path already drives with a work dispatcher. Submit a cluster only when
its stored cost is large enough that a wakeup is cheaper than running
it inline; the light-graph numbers put that threshold in the tens of
microseconds, in the same range as the baker's 5–50 µs grain. Publish
and sort outputs after the join, as `Execute` already does, so
diagnostic order stays the order the slots were written. Godot can pass
`WorkerThreadPool` as the dispatcher; a host with one character and no
pool keeps today's serial `Execute`.

**Stage 2. Split large point ranges, and batch readers.** Inside a skin
or matrix chunk bigger than a few thousand points, use a persistent
pool the way `moverGraph.cpp` uses `WorkParallelForN`. One 26k skin was
1.6× on four threads; the threshold of 4,096 already in `parallel.h` is
in the right region, and a lower one will lose to the wakeup. For a
crowd or an export, run one `RigExecRuntimeReader` per character or per
frame. Four independent 26k skins were 4.0×. Evaluation does not read
the previous frame, so the frames do not have to be in order.

**Stage 3. Specialize a kernel that a profile still shows on one
thread.** The candidate the USD comments already name is a large dual
quaternion skin, or a stacked run of matrix movers if those dominate a
particular rig. Write that loop in the existing runtime TU, under the
same flags, and keep the scalar fallback for the parity tests. The
element-size monomorph of linear blend is not worth a change: it was
1 µs on 26k points. Wrinkle stays a serial sweep.

LLVM ORC, a whole-graph AOT image, MLIR, and Halide are not on this
path. An AOT image of a single kernel is redundant with stage 3's C++.

## Risks

Parallel cluster execution has to preserve each operation's arithmetic.
Point ranges of a skin are independent, so splitting them is
bit-identical, and the probe's 4-thread skin matched the serial buffer
with `memcmp`. A reduction that reassociates a sum, or a wrinkle sweep
that runs edges concurrently, will not match.

`apply` on an external mover is specified not to keep state between
calls. It is not specified to run on two revisions at once. Stage 1
should call a given plugin revision from one thread, or the plugin
contract needs that sentence before clusters overlap.

Diagnostic text is gathered after `Execute` returns. Workers must write
into the step's own slot, which the baked path already does, and the
runtime must keep assembling in a stable order. Tests that compare a
serial trace's completion order will see a different order under a
dispatcher; tests that compare values should not.

A dispatcher that runs a task inline when its pool is saturated can
deadlock: the executor submits the successors of a cluster from inside
the cluster task. The pool has to have a queue, and `wait` has to be
called from the thread that started the root clusters.

The cost model was fitted on another machine. Using it as a dispatch
threshold can pack too finely on a small guest. The synthetic graphs
show the failure mode: parallel singleton clusters were 30× slower. The
threshold wants to be conservative, and a serial `Execute` remains the
reference for values.

`animatedTouched` does not currently select work. A host that signals a
time change without writing new input bits will keep the previous cone.
That is existing behavior. A compiled evaluator must not "fix" it by
rerunning the world every frame.

This environment could not time a format-18 biped or ball. Stage 1
should be checked on a re-exported character with
`rigExecPose --verify-binary` before it changes the default. The probe
is the place to extend that measurement; it is not part of the product
build.
