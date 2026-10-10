# Navigating the repo and testing only what changed

Research and a dry-run proof of concept. Nothing here is wired into
CI. No product code, schema, or test was changed.

The C++ gate on draft PR #7 took about 21 minutes (163 tests, ctest
real time 1,252 seconds on a 4-core runner). Draft PR #9, which is not
merged here, got that to about 10 minutes by running the serial,
fine-cluster, and scoped-clear variants inside the primary process
(150 tests, 593 seconds real on one green run, 735 on another). This
note is the next cut: do not launch the suite an edit cannot affect.

## What an agent can see today

`AGENTS.md` names `rigExec`, math, imaging, bake, binary, runtime,
rigging, and standalone. It does not name `rigExecGraph` or
`rigExecSampler`. Both are load-bearing, and the graph name is the
easy one to get wrong (see the library review). There is no
per-directory README under `libs/`. There is no `compile_commands.json`
and `bin/build_rigexec.sh` does not pass
`-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`. There is no `.clangd`. CI
(`.github/workflows/usdrig.yml`) caches the OpenUSD prefix and uploads
the whole build tree between jobs. It does not use ccache or sccache.
The workflow comment says CTest has no fast/full labels. That is still
true. The labels that exist are `golden`, `exec`, `one-loop`, and
`input-protocol`, on a handful of tests.

The root `CMakeLists.txt` is the build. It has 191 `add_test()` lines.
`foreach` and `rigexec_add_graph_test()` multiply some of them. The
parser in `tools/select_affected_tests.py` sees 209 test names and 102
executables, including the bench drivers, which are not registered
with CTest. A configured tree was not available here, so this is the
CMake text, not `ctest -N`.

Almost every test has a 120 second timeout (`CMakeLists.txt:2168`).
Three names are raised to 300 seconds and, for the frame-cache pair,
`PROCESSORS 4` (`:2193-2202`): `testRigExecImagingFrameCache`,
`testRigExecImagingFrameCacheDefault`, and
`testRigExecBakedModeScopedClears`. PR #7's green run still spent
221 seconds in scoped clears and over 120 seconds in several others
before the cap moved. An agent that runs the full gate on a loaded
4-core machine will see timeouts that are the cap, not a failed
assertion.

`JOBS` in `bin/build_rigexec.sh` defaults to 8 because each `rigExec`
translation unit is large enough to OOM a compiler when Ninja uses
every core. That is the other reason a full rebuild is the wrong
inner loop.

## Where selection works, and where it does not

`rigExec` is one shared library. It compiles the evaluator, the baked
program, the mover graph, the frame cache, the oracles, and 34 of the
35 `libs/rigExecGraph/*.cpp` files. Anything that links `rigExec`
relinks when any of those objects change. The parser's transitive
counts:

| Library | Tests in the link closure | What a direct link line selects |
|---------|---------------------------|----------------------------------|
| `rigExecMath` | 118 | 9 executables (`testRigExecMath`, the IK/RBF/weight/wrinkle math tests, `testRigExecPoseInterpolator`) |
| `rigExecGraph` (the archive, `opGraph.cpp` only) | 111 | `testRigExecOpGraph` |
| `rigExec` | 107 | 80 tests. A change to `constraintSceneLowering.cpp` lands here, because that file is compiled into `rigExec` |
| `rigExecBinary` | 42 | `testRigExecFormat` |
| `rigExecRuntime` | 37 | 27 tests, 20 executables |
| `rigExecBake` | 31 | 31 |
| `rigExecSampler` | 30 | (reached through runtime consumers and imaging) |
| `rigExecImaging` | 13 | 13 |
| `rigExecRigging` | 11 | 11 |
| `rigExecStandalone` | 2 | 2 |

Reading the table: a math `.cpp` change is a 9-test direct guess and a
118-test safe rebuild, because `rigExec` links math `PUBLIC`. An
imaging `.cpp` change is 13 tests either way. A scene-lowering change
is an `rigExec` change, and the direct set is already most of the gate.
Selection does not make `libs/rigExec/` cheap. It makes math, binary,
runtime, imaging, rigging, and standalone cheap, which is where a
kernel or a format fix actually lives.

Header edits are wider than `.cpp` edits. `opGraph.h` is included by
the runtime and the evaluator. The selector's include scan gives that
header 28 direct tests and the same 111 transitive tests as the
archive. The scan is one level of `#include` plus the link closure, not
a full preprocessor.

Three executables have a project-library closure that does not reach
an OpenUSD target:

- `testRigExecFormat` links only `rigExecBinary`. Its sources do not
  include `pxr/`. This test can build and run with no OpenUSD.
- `testRigExecWrinkleRuntime` and `testRigExecDeltaMushRuntime` link
  only `rigExecRuntime`. Their ctest fixtures are `.rigexec` files
  produced by `rigExecBake` (`FIXTURES_REQUIRED`). The executable is
  USD-free; a clean ctest run is not, until a fixture file is checked
  in.

`testRigExecOpGraph` links the USD-free archive and then OpenUSD
`work` (`tests/testRigExecOpGraph.cpp:4-5`). It is one include away
from being a USD-free unit test. The math tests need `gf` because
`rigExecMath` links it publicly; they do not need `exec`, `usd`, or
Hydra.

The small stages already exist and most runtime tests do not have to
open the biped. `docs/examples/*.usda` is one node per file (the delta
mush runtime fixture is baked from `docs/examples/delta_mush_mover.usda`).
`examples/01_FkChainTail.usda` through `examples/15_TransformMatrixMover.usda`
are the feature rigs. `examples/biped/` is the stack the benches and
the slow playback tests open. `tests/fixtures/` is 32 files, all
`.usda`, `.json`, or reference headers. There is no checked-in
`.rigexec`.

## Proof of concept

`tools/select_affected_tests.py` reads `CMakeLists.txt`. It does not
configure, compile, or run anything.

- It expands literal `foreach` loops and synthesizes the extra names
  `rigexec_add_graph_test()` registers (graph serial, and cone verifies).
- It records each `add_library` / `add_executable` source list and each
  `target_link_libraries` edge. `PUBLIC` and `PRIVATE` are not
  distinguished, so a private edge still counts as a link. That
  over-selects slightly and does not miss a relink.
- **Direct** means the executable's own link line names the changed
  library. **Transitive** means the link closure does. Direct is the
  fast guess. Transitive is the safe rebuild. For a `.cpp` compiled
  into `rigExec`, direct is already the large set.
- A changed header is followed one `#include` deep, then mapped through
  the translation unit that included it.
- `bin/test_changed.sh` is a thin wrapper. Default is dry-run. `--run`
  executes the printed `cmake --build` and `ctest -R` lines and was
  not executed here (no OpenUSD prefix, no build tree).

Snapshot of this branch: `experiments/source_to_test_map.json` (209
tests, the per-library lists, the three USD-free executables).
Transcript: `experiments/select_affected_dry_run.txt`.

Dry-run, `libs/rigExecMath/solvers.cpp`:

```text
direct tests (9):
  testRigExecDualQuat testRigExecMath testRigExecPoseInterpolator
  testRigExecRbf testRigExecSingleChainIk testRigExecSplineIk
  testRigExecSurfaceKernelCache testRigExecWeightFields
  testRigExecWrinkleMath
transitive tests (118)
cmake --build build --target testRigExecMath ... (those nine)
```

Dry-run, `libs/rigExecBinary/format.cpp`: direct test
`testRigExecFormat`, transitive 42.

Dry-run, `libs/rigExecGraph/constraintSceneLowering.cpp`: the file is
on the `rigExec` source list, so direct is 80 tests and transitive is
107. That is the honest answer, and it is why the graph/evaluator
split in the library review is also a testing change.

Known gaps in the parser, all printed by `--explain` and recorded in
the JSON notes:

- Example-fixture loops (`foreach(... IN LISTS RIGEXEC_EXAMPLE_FIXTURES)`)
  are not expanded. `verify_binary_*` and `example_graph_*` are missing
  from the map. They are outside the CI `-E 'Cones_|ExampleParity'`
  gate anyway.
- A test launched through `cmake -P` has an empty executable in the
  map when the command is `${CMAKE_COMMAND}` rather than the driver
  binary. `testRigExecInputActions` is the one that showed up that way.
- The map is stale the moment `CMakeLists.txt` changes. Regenerate
  with `python3 tools/select_affected_tests.py --json experiments/source_to_test_map.json`.
  Do not hand-edit the JSON.

## Prior art, and what fits this tree

**Bazel (and Buck, Please, GN).** `bazel query 'rdeps(//..., //libs/rigExecMath:rigExecMath)'`
and `bazel-diff` answer affected targets because each library is a
real target with a declared source list. This repo has that
information, flattened into one `CMakeLists.txt`. Adopting Bazel would
replace `find_package(pxr)` and the plug-in layout OpenUSD expects.
The query we want is a hundred lines of Python over the file we
already have. Not worth a second build system.

**CMake itself.** There is no upstream "tests affected by this diff."
The pieces that do exist are the ones to use:

- `cmake --build build --target <exe>` builds one executable and the
  libraries it links. Ninja already prunes up-to-date objects.
- `ctest -R` and `ctest -L` select by name and by label.
  `FIXTURES_SETUP` / `FIXTURES_REQUIRED` are already used for the
  wrinkle and delta-mush binaries. Labels are the missing half.
- `CMAKE_EXPORT_COMPILE_COMMANDS` writes `compile_commands.json`.
  clangd, clang-tidy, and an include-what-you-use pass all read it.
  One flag in `bin/build_rigexec.sh`.
- File sets (`target_sources` with `BASE_DIRS`) would make the
  selector a CMake script instead of a parser. Not required for the
  dry-run. Worth it if the `foreach` gaps start to lie.

**Compiler caches.** ccache and sccache both wrap the compiler
(`CMAKE_CXX_COMPILER_LAUNCHER`). Direct mode hashes the preprocessed
source, so a touched comment in a popular header still misses, and an
unchanged `.cpp` hits. sccache adds a remote bucket, which is how a
cloud agent and a laptop share objects. OpenUSD is already cached as
a prefix tarball in GitHub Actions. The uncached cost is compiling
`rigExec` and `rigExecImaging` on every push. A launcher on the
`build-release` job is the highest-leverage CI change that does not
alter a test. It does not replace selection: a cache miss on
`geometry.cpp` is still a long compile, and a cache hit still runs
whatever ctest you ask for.

**Test impact analysis.** Azure DevOps "test impact," Google's TAP,
and coverage-based selectors (lcov per test, then invert the map) answer
a question the link closure cannot: which tests execute this function
inside `rigExec`. PR #7 recorded one gcov run of the gate at 85.6%
library line coverage (66,293 of 77,417). That is one number for the
whole suite, not a per-test map. Producing the map means running the
gate under gcov once, splitting `.gcda` per test (or using
`gcovr --json` with a process-per-test), and inverting it. It is the
right second phase for edits inside `libs/rigExec/`. It is the wrong
first phase: it needs a green full run, it rots as tests change, and
the link map already answers math, binary, runtime, imaging, and
rigging.

**clangd.** `compile_commands.json` at the build root, and a `.clangd`
that points `CompilationDatabase` at `build`, gives an agent
go-to-definition across the `rigExec` / `rigExecGraph` cycle. Without
it, an agent greps and follows the wrong `opGraph.h`. Godot's editor
build publishes the same file for the same reason. No new tool.

**What not to copy.** A custom test runner, a second ctest dashboard,
or a hand-maintained spreadsheet of tests. The CMake file is already
the spreadsheet, and it is the one contributors update.

## Recommended plan

Do these in order. Each one is useful alone. None of them belongs in
the CI gate until the dry-run has been wrong in public at least once
and the miss was fixed. This PR stops at step 0.

### 0. Land the dry-run (this change)

- `tools/select_affected_tests.py`
- `bin/test_changed.sh` (dry-run unless `--run`)
- `experiments/source_to_test_map.json` and
  `experiments/select_affected_dry_run.txt`, regenerated, not edited
- This note, and the library review beside it

An agent edits `libs/rigExecMath/rbf.cpp` and runs
`bin/test_changed.sh libs/rigExecMath/rbf.cpp`. It builds the nine
direct test executables, not the imaging library.

### 1. Labels and a fast tier, in `CMakeLists.txt`

Add a label at each `add_test`, from the executable's link line, so
`ctest -L` works without the Python parser:

| Label | Membership |
|-------|------------|
| `usd-free` | `testRigExecFormat`, and the runtime playback tests once they have a checked-in fixture |
| `math` | the nine direct math executables |
| `runtime` | tests that link `rigExecRuntime` and not `rigExecImaging` |
| `binary` | `testRigExecFormat` |
| `graph` | `testRigExecOpGraph` |
| `imaging` | the 13 imaging tests |
| `bake` | tests that link `rigExecBake` |
| `rigging` | the 11 rigging tests |
| `fast` | `usd-free` + `math` + `graph` + any test whose stage is a `docs/examples/` node or a `tests/fixtures/` file |
| `slow` | anything that opens `examples/biped/`, the frame-cache suites, and the example-parity sweep |

`fast` is an allow-list, not "everything except biped," so a new test
is slow until someone labels it. That is the safe default.

Check in one small `.rigexec` (the delta-mush doc example, or a
hand-built file of the kind `testRigExecFormat` already constructs)
and drop `FIXTURES_REQUIRED` on `testRigExecDeltaMushRuntime` for the
`usd-free` label. The bake-produced fixture can stay as a second test
that does need USD.

Point `testRigExecOpGraph`'s dispatcher at `std::thread` when the test
is not checking `work` itself, or split the `work` cases into
`testRigExecOpGraphWork`. The archive test then joins `usd-free`.

### 2. Teach the build scripts what clangd and a cache need

In `bin/build_rigexec.sh` and the CI configure step:

```text
-DCMAKE_EXPORT_COMPILE_COMMANDS=ON
-DCMAKE_CXX_COMPILER_LAUNCHER=sccache   # or ccache, if the wrapper is installed
```

Add a four-line `.clangd`:

```yaml
CompileFlags:
  CompilationDatabase: build
```

Document both in `AGENTS.md`, and add the two missing libraries to the
code map there:

- `libs/rigExecGraph`: USD-free op graph (`opGraph.cpp` only). The
  other sources in that directory compile into `rigExec`.
- `libs/rigExecSampler`: stage values into `.rigexec` inputs. The
  runtime does not include it.

Add one paragraph to `AGENTS.md`: `Evaluate` runs the baked program or
returns an invalid pose. It does not fall back to OpenExec. The banner
in `bakedProgram.h` is stale until that edit lands. And: for a change
under `libs/`, run `bin/test_changed.sh` on the paths and treat the
direct set as the inner loop, the transitive set before push.

Do not add a README per library. The code map plus this note is the
ownership map. A second copy will drift.

Link both review notes from `docs/index.md` so they are findable. They
are not node catalog pages.

### 3. Per-test coverage, only for `libs/rigExec/`

When the link map says "107 tests," run a gcov (or clang source-based)
pass that isolates `.gcda` per ctest invocation and invert it to
`path -> tests`. Store the inversion next to the JSON snapshot and
regenerate it when the gate's test list changes, not on every commit.
Use it as a narrower suggestion inside `rigExec`, and still build the
`rigExec` target. Coverage can miss a test that would have failed.
The link closure cannot.

PR #9's consolidation should land before that measurement. Otherwise
the map records four processes that are about to become one, and the
slow numbers in this note go stale on purpose.

### 4. Leave the CI gate whole

The pull-request job keeps `ctest -R '^testRigExec' -E 'Cones_|ExampleParity'`.
Selection is for the edit loop. The gate is what catches a `PUBLIC`
link the selector under-counted and a bit-exact drift a math test does
not instantiate. Caching the compiler and exporting
`compile_commands.json` are the CI changes. A `ctest -L fast` job can
be a second, optional check later. It should not replace the gate.

## What this environment did not verify

`bin/test_changed.sh --run` was not executed. There is no OpenUSD
prefix and no `build/` tree here. The dry-run was executed against
this `CMakeLists.txt` and the include scan; the transcript is
`experiments/select_affected_dry_run.txt`. The USD-free compile that
backs the "no OpenUSD" claim is in the library review, not repeated
here.
