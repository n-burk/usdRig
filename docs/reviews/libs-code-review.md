# Review of `libs/` on `rigexec-format-v4`

Review only. No product code, schema, or test was changed. The tree
reviewed is the branch tip that contains the constraint-lowering fix
from draft PR #7 (`constraintSceneLowering.cpp` no longer walks a
dangling `initializer_list`). Draft PR #9's ctest consolidation is not
in this tree: the fine-cluster and auto-clavicle cases are still
separate `add_test` entries.

Line counts below are `*.h` plus `*.cpp` unless noted. Generated
FlatBuffers headers are called out separately so they are not treated
as handwritten bulk.

## What was checked

- Every library directory under `libs/`, the root `CMakeLists.txt`
  target graph, and the public headers those targets install.
- A search for the dangling-`initializer_list` pattern from PR #7 and
  for the nearby lifetime shapes (`string_view` bound to a temporary,
  `c_str()` on a temporary, range-for over a braced list).
- A compile of the USD-free slice with no OpenUSD include path:
  `g++ -std=c++17 -O2 -ffp-contract=off -fno-fast-math`, include paths
  `libs/`, `thirdparty/flatbuffers/include`, and (for transport only)
  `third_party/lzma/C`.
- The seven `tests/bench*.cpp` drivers were read. They were not run.
  This environment has no OpenUSD prefix, and every bench opens a
  stage.

`nm -u` on the twenty resulting objects (`17` runtime translation
units, `opGraph.cpp`, `format.cpp`, `transport.cpp`) shows libc and
libstdc++ symbols only. No undefined `pxr`, `Gf`, `Usd`, `Sdf`, or
`Vt` symbol.

## Executive summary

The playback stack really is free of USD. `rigExecRuntime` (31,392
lines) and `rigExecBinary` (38,861 lines, most of them generated) have
zero `#include` of `pxr/`, and they compile and link that way. The
name `rigExecGraph` does not. The CMake comment says the graph target
is a USD-free compiler, and the target is one file, `opGraph.cpp`.
The other 34 `libs/rigExecGraph/*.cpp` files are compiled into the
USD-linked `rigExec` shared library, and several graph headers include
USD types and `rigExec/` headers. That is the layering bug agents will
trip on.

`Evaluate` does not fall back to OpenExec when a program will not
build. It returns an invalid pose. The banner in `bakedProgram.h`
still says an unsupported rig stays on the dynamic path. That comment
is stale, and it is the comment an agent will trust.

The largest speed gap that does not threaten the bit-exact contract is
inside one skin. The baked evaluator parallelizes linear and
dual-quaternion skin at 4,096 points and hoists the palette once. The
runtime runs the same kernels on one thread for the whole mesh.
Cluster dispatch (`SetTaskDispatch`) does not split a mesh.

Targeted testing pays off at the library boundary and stops at
`rigExec`. A change to `libs/rigExecMath/solvers.cpp` has 9 direct
tests and 118 transitive ones, because `rigExec` links math publicly.
A change to `constraintSceneLowering.cpp` is a change to `rigExec`
itself: 80 direct tests, 107 transitive. Details and the selector are
in `docs/reviews/agent-navigation-and-targeted-testing.md`.

## Priority

Effort is the size of the edit, not a schedule. Small is a localized
change. Medium is one subsystem and the parity tests that already
cover it. Large crosses the `rigExec` shared library or splits a
multi-thousand-line function. None of these are safe to do by deleting
a runner: the bit-exact contract compares dynamic, baked, and binary
results, including NaN sign, under `-ffp-contract=off`.

| Pri | Finding | Where | Effort |
|-----|---------|-------|--------|
| High | `rigExecGraph` the target and `libs/rigExecGraph/` the directory are different libraries | `CMakeLists.txt:52-53`, `:108-138`, `:175-177` | Large to split; small to stop the comment lying |
| High | USD-free runtime claim holds, and should be kept that way | `runtime.h:1-16`, compile below | Small to guard (a CI compile with no pxr include path) |
| High | `Evaluate` does not take the dynamic path the banner describes | `bakedProgram.h:12-15` vs `rigEvaluator.cpp:297-303` | Small |
| High | Runtime skin is serial; baked skin is parallel past 4,096 points | `geometry.cpp:2499-2501`, `moverGraph.cpp:871-930` | Medium |
| High | Every successful `Execute` copies moved points and weights out of the retained buffer | `exec.cpp:671-707`, `store.h:160-162` | Medium |
| Medium | Oracle, golden, and reference code ship inside `librigExec` | `CMakeLists.txt:12`, `:100-107` | Medium |
| Medium | Bake's public header includes the 6,070-line evaluator impl | `revisionReads.h:21`, `bakedProgramImpl.h` | Medium |
| Medium | Imaging's public playback header includes runtime and sampler; the link is private | `playback.h:46-47`, `CMakeLists.txt:272` | Small |
| Medium | `_CompileEpochAttempt` is one function, lines 172-3600 | `rigEvaluatorCompile.cpp:172` | Large |
| Medium | Volume-field validation is copied between the reference and the runtime | `weightReference.cpp:272-304`, `weights.cpp:1787-1818` | Medium |
| Low | `initializer_list` footgun is fixed; one `.c_str()` call site is safe only while `read` stays synchronous | `constraintSceneLowering.cpp:106-122`, `:212-215` | Small |
| Low | Smooth copies the whole point vector even when adjacency is cached | `geometry.cpp:1857` | Small |
| Low | `RrProloguePose` discards both of its diagnostic parameters | `pose.cpp:592-593` | Small |
| Low | Historical comment in the touch-pose shader wrapper | `touchPoseHighlight.cpp:297-318` | Small |

## Per library

### `rigExecSchema`

`schema.usda` only. There is no C++ target. Generated plug-in resources
live under the build tree, not beside the source schema. Nothing here
links backward into the evaluator. Agents should edit the schema, then
regenerate with `bin/gen_schema.sh`, and not look for a library named
`rigExecSchema`.

### `rigExecGraph`

9,696 lines, 53 `#include` lines that name `pxr/`.

The CMake target is the thing the comment describes:

```cmake
# CMakeLists.txt:52-53
# Shared descriptor compiler and executor, with no USD dependency.
add_library(rigExecGraph STATIC libs/rigExecGraph/opGraph.cpp)
```

`opGraph.h` includes only the standard library. `opValues.h`,
`providerRecords.h`, `poseArithmetic.h`, `providerArithmetic.h`, and
`blendLayout.h` are also free of `pxr/` and are what the runtime
includes. That slice compiled in the check above.

The rest of the directory is the USD scene compiler, and it is
compiled into `rigExec`, not into `rigExecGraph`. The parser counted
34 `libs/rigExecGraph/*.cpp` files on the `rigExec` source list
(`CMakeLists.txt:108-138` and `:175-177`) and one file on the graph
target. Headers in that USD half include the stage:

```cpp
// libs/rigExecGraph/sceneAccess.h:4-9
#include "pxr/base/tf/token.h"
#include "pxr/base/vt/dictionary.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/sdf/valueTypeName.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/timeCode.h"
```

They also include the evaluator. `weightProgram.h:3` includes
`rigExec/weightPackets.h`. `geometryProgram.h` includes mover-graph
headers. `bakedProgramImpl.h` includes the scene-lowering headers back.
The dependency is a cycle between the directory and `rigExec`, not
between the static archive and `rigExec`. The archive itself links
nothing.

`testRigExecOpGraph` is the direct test of the archive. It links
OpenUSD `work` (`CMakeLists.txt` near the op-graph test, and
`tests/testRigExecOpGraph.cpp:4-5`), so the unit test is not USD-free
even though the library is. The dispatcher test is the only reason.

A real split is large: the USD lowering has to move to a target that
is allowed to link USD, and the headers that the runtime includes have
to stay on the USD-free target. Renaming, or moving the USD `.cpp`
files under `libs/rigExec/`, is the small step that makes the
directory match the link line. Leaving the comment as it stands will
send an agent to "fix" a USD include in a file that is supposed to be
compiled into `rigExec`.

### `rigExecMath`

15,758 lines, 45 `pxr/` includes, all `gf`, `vt`, or `tf`. No
`UsdStage` and no `SdfPath` in the math headers that were searched.
`CMakeLists.txt:80` links the archive `PUBLIC` to `arch`, `tf`, `gf`,
and `vt`. Math is independent of the stage, which is the rule in
`AGENTS.md`. It is not independent of OpenUSD's base libraries.

The runtime does not link this archive. It instantiates header-only
kernels that do not include `pxr/`:

- `propertyMathKernel.h` (Gf instantiations live in `propertyMath.cpp`)
- `latticeKernel.h`, `deltaMushKernel.h`, `wrinkleKernel.h`
- `affineFrameKernel.h` (the singular header; `affineFrameKernels.h`
  is the Gf one)
- `autoClavicleKernel.h`, `limbStretchKernel.h`, `envelope.h`
- `pointBlocks.h`, `pointRanges.h`, the surface and wire caches

That split is the right one, and it is why the runtime compile
succeeded. The hazard is adding a `#include "pxr/..."` to one of those
headers. `simdKernels.h` and `geometryKernels.h` already include `gf`;
they are the Gf side and the runtime does not include them.

Direct tests, from the link lines: `testRigExecMath`,
`testRigExecSingleChainIk`, `testRigExecDualQuat`,
`testRigExecSplineIk`, `testRigExecRbf`, `testRigExecWeightFields`,
`testRigExecWrinkleMath`, `testRigExecSurfaceKernelCache`, and
`testRigExecPoseInterpolator` (which also links `rigExec`). Transitive
closure is 118 tests because `rigExec` links math publicly.

### `rigExec`

88,746 lines, 517 `pxr/` includes. This is the evaluator, the mover
graph, the baked program, scheduling, the frame cache, and the frozen
worker. It also absorbs the USD half of `rigExecGraph` and, privately,
`rigExecInputValues`.

`Evaluate` (`rigEvaluator.cpp:181-198`) settles the epoch and runs
`_bakedProgram->Run`. When the program is missing it records
`"native program unavailable for compiled epoch"` and returns
(`:297-303`). It does not call `RigExecTapSet::Evaluate`. The tap set
still exists. Tests and `bakedExecCrossCheck.cpp` construct one, and
`tapSet.cpp:277` still computes an `ExecUsdRequest`. That is the
cross-check and the OpenExec registration path, not the pose
`Evaluate` returns.

The file banner disagrees:

```cpp
// libs/rigExec/bakedProgram.h:12-15
// It is a SECOND implementation of the evaluation semantics, so it is a
// request rather than a promise: a rig using any feature the program cannot
// express stays on the dynamic path, with a reason per feature. See
// IsBakeable.
```

`IsBakeable`'s own comment already says a silent fallback is the wrong
outcome (`bakedProgram.h:223-225`).
`_RebuildBakedProgram` still calls the diagnostic "the fallback"
(`rigEvaluator.cpp:401-404`). An agent that believes the banner will
look for a live OpenExec evaluate path and will not find it on
`Evaluate`. Updating the banner is small. The dozens of "dynamic path"
comments inside `bakedPose.cpp`, `bakedGeometry.cpp`, and
`bakedProgramImpl.h` mostly mean "the mover-graph implementation this
step must match." Those are parity notes, not a second `Evaluate`.
They should stay until a given one is actually wrong.

`_CompileEpochAttempt` (`rigEvaluatorCompile.cpp:172-3600`) is one
function of about 3,429 lines. The next function starts at line 3602.
Splitting it along the profile scopes already in the body is a large
edit and does not change behavior if the scopes stay in order. It is
the single worst function in the tree for an agent to edit safely.

These translation units are compiled into the shipping shared library
and are test oracles, not evaluation:

- `goldenPose.cpp`, `goldenSuite.cpp`
- `oracleInputs.cpp`
- `weightReference.cpp`, `scalarReference.cpp`, `scalarReferenceAdapter.cpp`
- `bakedExecCrossCheck.cpp`, `bakedExecCrossCheckRows.cpp`

`CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS` is on (`CMakeLists.txt:12`), so on
Windows every one of those symbols is exported. On other platforms
default visibility does the same for a shared library. Moving them to
a static archive that only the tests link is a medium change. The
evaluator does call some of them (`rigEvaluator.h` holds a golden-suite
observer; the scoped-clear shadow compares poses), so the split has to
keep the observer hook and drop the oracle bodies.

`bakedProgramImpl.h` is 6,070 lines and is included by the bake
library, by imaging (`registry.cpp`, `bridge.cpp`, `liveDebug.cpp`),
and by most baked translation units. It is the private definition of
the program, installed as a public header because the install rule
ships `libs/*/*.h`.

Parallel skin lives here and is in good shape. See the speed section.

`rigExecInputValues` is a separate static library
(`libs/rigExec/inputReplayValues.cpp`) linked `PRIVATE` into
`rigExec`, specifically so the value-codec templates are not exported.
That boundary is doing its job.

### `rigExecRuntime`

31,392 lines, zero `pxr/` includes. `CMakeLists.txt:324` links
`rigExecBinary` and `rigExecGraph` and nothing from OpenUSD.
`runtime.h:1` calls it zero-USD playback. `runtime.h:8-16` states the
fidelity rule (bit-identical to the baked path, gated by
`rigExecPose --verify-binary`) and the threading rule: `Execute` is
serial over clusters unless the caller passes a dispatcher. The
OpenUSD host passes a `WorkDispatcher`. A Godot host passes its
`WorkerThreadPool`. The reader holds no locks.

`runtimeMath.h` is a mirror of the Gf operations the kernels use, with
the same order, including Gf's short-vector normalize. The header's
own comment says the comparison test links USD and this header must
not (`runtimeMath.h:13-16`). `testRigExecRuntimeMath` links `gf` for
that comparison only (`CMakeLists.txt:824`).

The compile check covered every runtime `.cpp`, including
`geometry.cpp` (9,062 lines) and `inputs.cpp` (2,912). They compiled
without a pxr include path.

What the runtime includes from other libraries is the USD-free header
slice: `store.h:21-22` pulls `opValues.h` and `providerRecords.h`;
`geometry.cpp:18-27` pulls the header-only kernels; `poseInternal.h:7-8`
pulls `autoClavicleKernel.h` and `poseArithmetic.h`. None of those
headers include `pxr/`. A future include of `affineFrameKernels.h`
(plural, Gf) or `simdKernels.h` would break the claim at compile time.
Worth a one-file CI compile with an empty include path outside `libs/`
and the two third-party include dirs.

The runtime is a second implementation of the baked step bodies, on
purpose. `weights.cpp:1-10` says so, and the parity gate is
`rigExecPose --verify-binary`. Sharing the header-only kernels is how
the two stay honest. Merging `RrRunPoseStep` (`poseSteps.cpp`, from
line 530) with `RigExecBakedRunPoseStep` (`bakedPose.cpp`, from line
4394) would put USD types back in the runtime or abstract them again.
The kernels are already the abstraction. Leave the runners.

`RrProloguePose` (`pose.cpp:588-593`) takes `poseDiagnostics` and
`error` and immediately discards both. The parameters cannot report a
prologue failure. Either fill them or remove them. Removing them
touches every caller, so it is a small edit with a wide diff.

Direct linkers of the runtime: 20 executables, 27 tests, including the
imaging test (imaging links the runtime privately) and the runtime
domain suites. Transitive closure is 37 tests. That is the useful
targeted set. See the speed section for the skin and the output copy.

### `rigExecBinary`

Handwritten `format.cpp` is 7,533 lines and `format.h` is 753.
`generated/rigexec_generated.h` is 28,554 lines and
`presentation_generated.h` is 1,190. The generated files are FlatBuffers
output; they are not review targets.

`format.h:9` says the format is USD-free. The compile agrees, once
`transport.cpp` is given `third_party/lzma/C` (it includes `LzmaEnc.h`
directly; CMake already adds that private include). No pxr include in
the library. `format.cpp` does include `providerRecords.h` and
`affineFrameKernel.h`, both USD-free.

`testRigExecFormat` links only `rigExecBinary`
(`CMakeLists.txt:911-912`). Its sources do not include `pxr/`. It is
the one test in the tree that can build and run with no OpenUSD at
all. The two runtime playback tests (`testRigExecWrinkleRuntime`,
`testRigExecDeltaMushRuntime`) also link only the runtime, but their
ctest fixtures are `.rigexec` files produced by the bake tool, so a
clean run still needs USD one step earlier.

A format change is still expensive in the transitive closure: 42
tests, because bake and the runtime both link the archive.

### `rigExecSampler`

743 lines, 17 `pxr/` includes. Two sources: `inputSampler.cpp` and
`runtimePoseProjection.cpp`. The job is to read a stage's animated
values into a `.rigexec` input block. `CMakeLists.txt:348` links
`rigExecRuntime` plus `usd`, `sdf`, `tf`, and `gf`. The runtime stays
free of USD because this library exists. That boundary is correct.

Imaging includes the sampler from a public header. See imaging.

### `rigExecBake`

7,902 lines, 69 `pxr/` includes. Links `rigExec` and `rigExecBinary`
(`CMakeLists.txt:365-367`). Thirty-one tests name it on their link
line, and the transitive set is the same size: nothing higher up links
bake except tests and the bake tool.

The public header undoes the layering:

```cpp
// libs/rigExecBake/revisionReads.h:21
#include "rigExec/bakedProgramImpl.h"
```

Any consumer of the revision-read API parses the evaluator's private
program. `bake.cpp`, `serialize.cpp`, `staticCapture.cpp`,
`computedCapture.cpp`, and `arrayReads.cpp` include the impl header
too. A medium fix is a narrow struct for the keys the enumerator
walks, defined in the bake library, with the impl header staying
private to `rigExec`.

### `rigExecImaging`

21,383 lines, 163 `pxr/` includes. Public link is `rigExec` plus Hydra
and `usdImaging` (`CMakeLists.txt:262-271`). Private link is
`rigExecRuntime` and `rigExecSampler` (`:272`).

`playback.h` is a public header and it includes both private
libraries:

```cpp
// libs/rigExecImaging/playback.h:46-47
#include "rigExecRuntime/runtime.h"
#include "rigExecSampler/inputSampler.h"
```

The install rule ships headers from `libs/`, so an installed
`playback.h` names libraries the imaging target does not propagate.
Callers inside this repo get away with it because they also link the
evaluator. A downstream plugin that links only `rigExecImaging` and
includes `playback.h` will fail at link time, or compile only because
the runtime headers happen to be on the same include root. Making the
runtime and sampler `PUBLIC` dependencies matches the header. Hiding
playback behind an internal header matches the link line. Either is
small.

`_EvaluateSessions` walks sessions in one loop
(`registry.cpp:1184-1190`). A stage with several rigs evaluates them
one after another. The frame-cache hit path in the bridge is what
keeps a scrub off that loop; the benches exist to measure it and were
not run here.

`touchPoseHighlight.cpp:297-318` is a postmortem of a shader-wrapper
bug (an assignment that dropped the wrapped shader's primvar
requests, measured on the biped eye shader). The constraint it
records is real. The twenty-line narrative belongs in the commit that
fixed it. A four-line comment that the wrapper must append primvars,
not replace them, carries the same contract.

`registry.cpp` is 5,736 lines and `sceneIndices.cpp` is 3,595.
`bridge.cpp` is 3,199. The legacy registry handle (`registry.cpp:507-513`)
is a strong pointer because `GetStore` hands out references into it.
That comment is load-bearing, not decoration.

Thirteen tests link imaging directly. The closure is the same thirteen.
Imaging is one of the few areas where "build this target and run its
tests" is already a tight set.

### `rigExecRigging`

4,811 lines, 32 `pxr/` includes. `rigBuilder.cpp` is 3,108 of them.
Links `rigExecMath` and USD (`sdf`, `tf`, `gf`, `vt`, `usd`) and does
not link `rigExec` (`CMakeLists.txt:235-237`). Authoring does not
depend on the evaluator. Eleven tests. This is the direction the graph
directory does not have.

### `rigExecStandalone`

1,558 lines, 34 `pxr/` includes. Links `rigExec`, `exec`, `esf`,
`usd`, and `sdf` (`CMakeLists.txt:227`). Two tests
(`testRigExecStandalone`, `testRigExecPack`). The specs
(`docs/specs/standalone-runtime.md`, `standalone-pack.md`) describe it
as the experimental scene adapter, distinct from `.rigexec` playback.
The link line matches that. It is not on the runtime's include path.
No change recommended beyond leaving it off the playback link.

### Third-party archives

`rigExecFlatBuffers` is an interface target over
`thirdparty/flatbuffers/include`. `rigExecLzma` compiles four C files
from `third_party/lzma/C` with `Z7_ST` and
`RIGEXEC_LZMA_PORTABLE_SCALAR`. Both are private details of
`rigExecBinary`. They showed up in the link closure (42 tests) only
because the binary archive links them. Not review targets.

## Lifetime

PR #7's bug was a range-for over a ternary of
`std::initializer_list` temporaries. The range-for keeps the list
object. The backing array belongs to the temporary that was copied,
and that temporary is dead before the first iteration. On the CI host
the pointer was null, and building `inputs:affect…` called `strlen`
on it. The fix stores the channel names in a local array for the whole
loop (`constraintSceneLowering.cpp:106-122`).

Searched the libraries for the same shape.

No remaining range-for over a ternary of `initializer_list`s. The
patterns that look similar and are defined:

- `for (const float e : {extentU, extentV})` in `weights.cpp:1790` and
  `weightReference.cpp:275`. The braced list initializes the
  range-for's own array, which lives for the loop. The elements are
  scalars, copied in.
- `static const std::initializer_list<const char *> eulerOrders`
  (`rigEvaluatorValidation.cpp:1034-1038`). Initializing a static
  `initializer_list` extends the backing array to the life of the
  static, the same way a static reference extends a temporary. It is
  then passed straight into `validateToken`, which walks it before
  returning (`:1019-1032`).
- `tests/testRigExecFormat.cpp:891` takes an `initializer_list` and
  copies it into a `std::vector` on the first line. The array only
  has to live for the call.
- The `for (auto id : {hop.raw, hop.overlay})` loops copy integers.

The one site that becomes the PR #7 bug if someone "cleans it up":

```cpp
// libs/rigExecGraph/constraintSceneLowering.cpp:212-215
auto mask=[&](const char *channel,bool fallback,RigExecConstraintAxisMask *value) {
    const std::string prefix=std::string("inputs:affect")+channel;
    return read((prefix+"X").c_str(),fallback,&value->x) &&
        read((prefix+"Y").c_str(),fallback,&value->y) &&
        read((prefix+"Z").c_str(),fallback,&value->z);
```

`read` uses the pointer before it returns (`readValue` at the top of
the same function). The temporary `std::string` from `prefix+"X"`
lives for the full call. Storing that pointer, or splitting the
expression so the temporary dies first, is the dangling bug again.
Passing `std::string` into `read` removes the footgun. Small.

`providerProgram.cpp:537` binds `string_view` to `TfToken::GetString()`,
which returns a reference into the token. Safe for the loop that owns
the token.

`moverRegistry.cpp` stores `schemaType.c_str()` on a handler kept in a
deque. The comment and the deque stability note say the entry owns the
string. That is a contract, not a dangling pointer, as long as the
deque element is the owner. Worth knowing if that storage changes.

No null-check-after-reference showed up. `if (!prim)` on a `UsdPrim`
is a real invalid-prim test, not a dead check.

## Duplication and comments

The baked program and the runtime are two runners over one set of
kernels. That is the design in `runtime.h:8-11`, not an accident to
delete. The kernels already shared, with a citation in
`docs/references.md` where the math has a published source
(`deltaMushKernel.h`, `latticeKernel.h`):

| Kernel header | Who instantiates it |
|---------------|---------------------|
| `deltaMushKernel.h`, `latticeKernel.h`, `wrinkleKernel.h` | mover graph (Gf) and `geometry.cpp` (Rr) |
| `propertyMathKernel.h` | `propertyMath.cpp` (Gf) and `properties.cpp` (Rr) |
| `affineFrameKernel.h` | Gf evaluators and `affineMath.h` |
| `autoClavicleKernel.h`, `limbStretchKernel.h` | baked pose, runtime pose solvers |
| `envelope.h` | both runners |

What is still copied, and is not the arithmetic, is the volume-field
validation. `weightReference.cpp:272-304` and `weights.cpp:1787-1818`
use the same error strings for a non-finite extent and a non-finite
scale, and the same `for (... : {a, b})` shape. The transforms under
those checks are deliberately not the same function: one calls
`GfMatrix4d::SetScale`, the other calls `_RrApplyAxisScales`, and the
parity test requires those bits. Extracting the two string checks is
medium and safe. Extracting the matrix is how the bits drift.

Comments that should change:

- `bakedProgram.h:12-15`, the stale dynamic-path banner. High, because
  it describes a control flow that `Evaluate` does not have.
- `touchPoseHighlight.cpp:297-318`, the eye-shader postmortem. The
  contract is "append the wrapped shader's primvar requests." The
  incident report can go.

Comments that should stay, even though they are long:

- `weights.cpp:1-30`. It records which bake enumeration the runtime
  trusts, and the residual risk for a connected weight target. That is
  a contract an agent will get wrong without it.
- `runtimeMath.h:5-16` and `CMakeLists.txt:21-31`. They are why
  `-ffp-contract=off` is not optional. A fast-math cleanup fails
  `SameBits`, including the NaN-sign case PR #7 fixed in the vec3
  kernel by keeping it `noinline`.
- `moverGraph.cpp:2763-2769`. The dense blend stays in the original
  lerp form because `dLo*(1-t) + dHi*t` and `dLo + (dHi-dLo)*t` are
  not bit-identical. `examples/04_BlendShapeFace.usda` gates it.
- `registry.cpp:507-513`. The legacy handle is strong because callers
  hold references into it.

`RrProloguePose`'s discarded parameters are the closest thing to a
dead API in the runtime. There is no `#if 0` block in `libs/*.cpp`.
A link-time dead-symbol pass was not run; the golden and oracle
symbols are live in the sense that tests and the observer call them,
and dead in the sense that a playback host does not need them in
`librigExec`.

Naming is consistent with the layer once you know the rule: `RigExec*`
on the USD side, `Rr*` and `RigExecWire*` on the runtime and the file,
`fb::` for generated FlatBuffers. It looks inconsistent and it is the
boundary marker. Do not rename it.

Magic numbers that already have names: `RigExecGeometryGrainSize` is
512 and `RigExecGeometryParallelThreshold` is 4,096 (`parallel.h:34-39`).
The runtime skin path does not consult them. Unnamed tolerances that
are part of the numeric contract (`1e-10` in `runtimeMath.h:28` as
`kRrMinVectorLength`, the avar scale floor in `avarScale.h`) are
documented. A few local cutoffs (`1e-12` in the weight placement, the
spline IK iteration cap) are local and low priority.

## Execution speed

Bit-exactness constrains every suggestion here. The root CMake file
adds `-ffp-contract=off` for every non-MSVC translation unit
(`CMakeLists.txt:21-31`) and repeats it on the runtime
(`:329-336`). Change detection uses `memcmp` on point and weight bytes
(`pointBlocks.h` documents NaN payload and signed zero). Parallel
results have to match the serial loop. Do not turn on fast-math, FMA
contraction, or a reciprocal-divide rewrite.

### Measured

The USD-free objects were compiled at `-O2` with contraction off. That
is a compile check, not a timing. `geometry.cpp` compiled cleanly in
that set; it is the hot runtime translation unit.

The benches were not run. Each of `tests/benchFrameCache.cpp`,
`benchFrameCacheWarm.cpp`, `benchPlayback.cpp`, `benchCommitLag.cpp`,
`benchEditLatency.cpp`, `benchPathLookup.cpp`, and `benchStageReads.cpp`
opens a USD stage (the biped, or a rig built through the evaluator).
None is registered with CTest. There is no OpenUSD prefix in this
environment, so there is no new number to put next to the 26,276-point
figure already in the source.

### High: runtime skin stays on one thread

```cpp
// libs/rigExecRuntime/geometry.cpp:2493-2501
if (!p.skinTopology &&
    !RrGeoSkinTransformsAreUsable(transforms.transforms,
                                  transforms.transformCount)) {
    return false;
}
// The runtime runs serially; a point range is an independent
// sub-problem, so the serial call is the parallel loop's answer.
return RrGeoApplySkinKernelRange(p, transforms, 0, count, pts, useSimd);
```

The baked side treats that independence as permission to split, for
both methods. The comment at `moverGraph.cpp:871-878` records the
measurement: only the linear path used to split, so every
dual-quaternion character skinned its whole mesh on one thread, and
on a 26,276-point body that was the largest cost in a drag.

The split is gated on `RigExecParallelEvaluationEnabled`, the frozen
serial flag, and `count >= RigExecGeometryParallelThreshold` (4,096).
The palette and the narrowed rows are built once and handed to every
chunk (`moverGraph.cpp:889-915`). A chunk that rebuilds them repeats
the same pure function; the comment counts that waste at 137
influences and a grain of 512.

`SetTaskDispatch` (`runtime.h:12-16`, `closure.cpp:982-985`) runs
independent clusters on the caller's workers. One skin of one mesh is
one cluster. Dispatch does not enter `RrGeoApplySkinKernelRange`.

The port is medium: the same threshold, the same grain, the same hoist,
and the serial path left in place below 4,096 points. The runtime has
no `WorkParallelForN`; the dispatcher callback is the stand-in, which
is what a Godot host already passes for clusters. A point-range
callback on that same hook keeps USD out of the runtime. The result
has to match the serial loop byte for byte, which the baked blend path
already requires of its own split (`moverGraph.cpp:2772-2774`).

Per-point cost is `O(points × elementSize)` for a fixed influence
count. That is the algorithm, not a stray quadratic. Layout validation
without a topology table still scans every index (`geometry.cpp`
around the `RrGeoSkinLayoutIsUsable` call). The baked path treats a
validated topology as `O(1)`. Caching that on the runtime topology
epoch is small and belongs in the same change only if it does not
skip a failure the serial check would have caught.

### High: the API copies the mesh it just computed

`RrRetainedArray::Read` returns a const reference
(`store.h:160-162`). The copy-on-write spare means a later `Write`
clones only when the buffer is shared, and `swap` recycles a unique
buffer (`:168-178`). `Execute` then throws that away for the caller:

```cpp
// libs/rigExecRuntime/exec.cpp:671-675
for (const auto &entry : store.movedProperties) {
    RigExecRuntimePoints moved;
    moved.path = program.TextOrEmpty(entry.first);
    moved.points = entry.second.Read();
    points.push_back(std::move(moved));
}
```

`field.weights = entry.second.weights.Read()` at `:707` is the same
copy for every weight field. `_SortByPath` then sorts the output
vectors (`:714-718`). The comment at `:710-711` says the API holds its
copies so the producers' buffers become unique and the next frame can
swap. The copy is intentional and it is a full mesh per moved property
per successful frame. Returning `shared_ptr<const vector>` or moving
out when `unique()` is true removes it. Both are API changes. Medium,
and behavior-preserving only for callers that do not mutate the
returned vectors. Imaging's playback path is the caller to read before
changing the signature.

### Medium, not re-measured

- `closure.cpp:949-980`. `inputsChanged` clears a scratch string, asks
  `RrEffectiveInputMemo`, sorts the covered-slot vectors, and appends
  a key per read, for each candidate op. The memo bytes decide whether
  the op runs. An incremental memo is medium and has to produce the
  same bytes.
- `geometry.cpp:1857`. Laplacian smooth copies every point into
  `source` even when `RigExecSurfaceKernelCache` already held the
  adjacency (`:1848-1853`). The older copy/move note in
  `docs/copy-move-review.md` said adjacency was rebuilt on every call.
  The cache path now skips that rebuild. The point copy remains. A
  scratch buffer on the revision removes it. Small.
- Delta mush uses `surfaceCache->MushRest` when a cache is passed
  (`geometry.cpp:3159-3171`) and rebuilds rest adjacency when it is
  not. The no-cache arm is the cold path.
- Imaging sessions are serial (`registry.cpp:1184`). The frame-cache
  hit path is the mitigation. Locking around the warm index and the
  frame-cache shards is real and was not timed here; the bench drivers
  exist for that and need USD.
- `inputs.cpp` and `publish.cpp` allocate on the diagnostic and refusal
  paths. Those are not the successful frame.

### What not to speed up

Reordering a lerp, contracting a multiply-add, or replacing `memcmp`
with an epsilon compare will fail the gates named in `runtime.h` and
in `moverGraph.cpp:2763-2769`. The vec3 NaN-sign failure in PR #7 was
this class of bug: at `-O3` an inlined `(-x) + (-2)` did not preserve
the sign of a NaN, and `noinline` on the kernel put the bits back.
Treat any "the compiler will clean this up" change in a kernel as a
`SameBits` change.

## Simplifications that remove a concept

These do not change results:

1. Delete the "stays on the dynamic path" sentence. `Evaluate` has one
   success path, the baked program, and one failure path, an invalid
   pose plus the `IsBakeable` reasons. The OpenExec tap set remains for
   cross-check. It is not a fallback evaluate.
2. Stop compiling oracles into `librigExec`. One test archive.
3. Make `libs/rigExecGraph/` mean the USD-free archive. Move the USD
   lowering sources next to the evaluator that already compiles them,
   or give them a target whose link line includes USD. Do not add a
   third wrapper.
4. Do not unify the baked and runtime runners. Keep extracting
   header-only kernels when a third copy appears, as the volume
   validation strings are about to.

## Checks not run

No OpenUSD prefix was available, so `bin/build_rigexec.sh`, CTest, and
`tests/bench*.cpp` were not run. The USD-free compile and `nm -u`
check above is the substitute for the runtime and the binary. The
link-graph counts come from `tools/select_affected_tests.py` against
this `CMakeLists.txt`, not from a configured build.
