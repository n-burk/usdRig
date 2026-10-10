# Working in RigExec

## Project

RigExec evaluates USD character rigs without changing their source stages.
`RigExecRoot` marks a rig. Controls and solvers produce transforms; movers
revise transforms, properties, or geometry in a defined dependency order.
Hydra scene indices publish evaluated data to `usdview` and other consumers.
The root README is the build entry point; `docs/index.md` is the node catalog.

## Code map

- `libs/rigExecMath`: numerical kernels. Keep math independent of stage access.
  The archive links `gf`, `vt`, and `tf`; it does not read a stage.
- `libs/rigExecGraph`: the USD-free op scheduler (`opGraph.cpp` only).
  Runtime and the binary format include these headers. Do not add a
  stage or `pxr/` include here.
- `libs/rigExecScene`: USD scene lowering. It links `tf`, `gf`, `vt`,
  `sdf`, `ts`, `usd`, and `usdGeom`, plus the graph archive and math.
  Its objects are absorbed into `librigExec.so`. Installed headers live
  under `include/rigExecScene/`.
- `libs/rigExecOracle`: test oracles and golden capture. Executables that
  link `rigExec`, and the Python module, link this object library. It is
  not part of `librigExec.so`. A cpu-reference or golden run aborts when
  the library is missing.
- `libs/rigExecSampler`: copies stage values into runtime inputs. This is
  the USD boundary in front of playback.
- `libs/rigExecSchema/schema.usda`: source schema. Resources under
  `plugin/rigExecSchema/resources` are generated; regenerate with
  `bin/gen_schema.sh` or `bin/gen_schema.bat` after schema edits.
- `libs/rigExec`: evaluator, mover graph, baked program, scheduling,
  invalidation, and frame caching. Preserve dynamic/baked parity.
  `Evaluate` runs that baked program, or returns an invalid pose and the
  `IsBakeable` reasons. It does not evaluate through OpenExec. The tap set
  remains for cross-check and tests.
- `libs/rigExecRigging` and `python`: authoring APIs and Python bindings.
- `libs/rigExecImaging`: scene indices, activation, and display state.
- `libs/rigExecBake`, `libs/rigExecBinary`, `libs/rigExecRuntime`: export,
  binary layout, and playback. Check version compatibility when changing records.
  `rigExecRuntime` links `rigExecBinary` and `rigExecGraph` and no OpenUSD
  library. Do not include a `pxr/` header from a runtime translation unit or
  from a header-only kernel the runtime instantiates.
- `libs/rigExecStandalone`: experimental scene adapter and rigpack path;
  distinct from `.rigexec` binary playback.
- `plugin/rigExecUsdview`: editor UI plus Qt-free interaction models.
- `plugin/usdNoodles`: bundled third-party node editor; preserve its license.
- `examples`: sample stages and procedural generators. The biped's eye
  shader keeps its own EULA; see `THIRD_PARTY_NOTICES.md`.
- `tests`: C++ suites, Python model tests, and graphical viewer tests.

## Unified evaluation loop

Native, frozen, and binary runtime evaluation must use the shared operation
graph, dependency compiler, readiness rules, and kernels. Add behavior through
declared inputs, outputs, and producers in that graph. Do not introduce a
second evaluator, fallback walk, private-stage evaluation path, or separate
pre/post pass that computes rig results outside the unified loop. Independent
reference checks remain optional judges, never production fallbacks.

Reuse compiled dependency edges and cluster closures for invalidation and
cache queries; do not rebuild a parallel dependency model or expand all
transitive dependencies on each edit. Preserve exact producer/version binding,
cycle handling, and native/frozen/runtime parity. See
[evaluation and checks](docs/concepts/baked-vs-dynamic.md).

## No-mutex zones and thread ownership

Operation bodies, numerical kernels, and the frozen worker's private arena
are no-mutex zones. Do not add mutexes, lock-taking helpers, shared mutable
caches, or blocking synchronization there. This includes indirect locks from
profiling and logging: collect per-step data locally and publish it at the
existing boundary. Respect the contracts in
[bodyPurity.h](libs/rigExec/bodyPurity.h),
[bakedSchedule.h](libs/rigExec/bakedSchedule.h), and
[frozenContext.h](libs/rigExec/frozenContext.h).

Sample live USD inputs on the owning thread before dispatch. Computation
bodies consume captured facts and declared typed values; workers own their
mutable arenas and must not access the live stage, evaluator, or imaging
registry. Solve sharing through immutable snapshots and explicit ownership,
not by wrapping a live evaluator or stage in a new mutex.

Existing scheduler, cache-publication, and registry locks belong to their
documented coordination boundaries. Preserve their lock order and generation
fences; do not extend their scope into computation or add locks to hide an
ownership violation. Keep authoring and UI callbacks on their owner thread.
A queued UI callback still blocks that thread while it runs: offload only
detached work through the existing worker path, and measure both dispatch and
completion latency when fixing interaction stalls.

## Build and verify

Use an unchanged OpenUSD 26.08 install with OpenExec, CMake 3.26+, Ninja,
C++17, and a compatible Python interpreter. `USD` and `PY` configure helper
scripts; their defaults look for a sibling dependency install.

1. Read relevant code and current changes before editing. Preserve user work.
2. Make the whole set of related changes, then build once. Do not alternate
   one edit, a rebuild and a test run.
3. Rebuild only the targets you need, e.g.
   `cmake --build build --target testRigExecOpValues rigExecPose`.
   `bin/build_rigexec.sh` / `bin\build_rigexec.bat` build everything; use them
   for a fresh checkout only.
4. Run the smallest set of tests that covers the change, capped at 4 minutes
   of wall clock. `bin/test_changed.sh <paths>` prints that set.
   `bin/test_changed.sh --run <paths>` builds the direct executables and
   runs them. The direct set is the inner loop; run the transitive set
   before pushing. The ctest line uses labels: `usd-free` when the direct
   executables link no OpenUSD library, otherwise the project libraries
   the change sits in.
   `ctest --test-dir build -L fast` is the short tier. `ctest -L usd-free`
   runs without a stage. `tests/fixtures/minimal.rigexec` is the checked-in
   program `testRigExecRuntimeFixture` opens. The whole `ctest` gate still
   runs in CI.
   `bin/build_rigexec.sh` writes `build/compile_commands.json` and uses
   ccache when it is on `PATH`. `.clangd` reads that database.
   `bin/check_runtime_usd_free.sh` compiles the runtime with no `pxr`
   include path.
5. Use `bin/test/run_python_tests.sh` or `.bat` only for Qt-free interaction
   changes, and the relevant `bin/test/run_testusdview*` helper only for
   viewport changes.
6. Report checks actually run and any unavailable dependency or graphics test.

Do not change tolerances just to make failures disappear. Evaluation must
remain non-authoring; scene edits belong in the authoring layer. Keep USD
row-vector conventions explicit at math boundaries and avoid stage access
inside pure computation callbacks.

## Documentation and release hygiene

Edit node prose in `docs/operator_notes.py` and schema docs, then regenerate
with `python docs/build_pages.py`. Handwritten guides live in `docs/concepts`
and `docs/specs`. `docs/build_html.py` produces ignored `docs/site` output.
Keep Markdown links relative and verify their targets after moving files.

Use short functional comments for contracts, numerical conventions, ownership,
and non-obvious constraints. Omit change diaries, issue transcripts, product
comparisons, and comments that repeat the code. Preserve copyright, license,
source attribution, and tooling directives.

Describe behavior in project terms. Cite public algorithm sources in
`docs/references.md` and link them from affected kernels or guides. Never
replace missing provenance with a claim of originality. Root MIT terms apply
to original project work; third-party material retains its existing terms.

Do not commit credentials, local paths, build trees, research-paper copies,
assistant state, generated HTML, or `docs/sizzle` content. Keep useful current
docs; do not reintroduce archived plans or performance diaries as specifications.

Keep one-off investigation scripts, image prompts, capture manifests, and trial
implementations outside this checkout. A new utility belongs here only if it
has a supported purpose, documented inputs, and dependencies available to a
fresh checkout. Retain real test fixtures and example generators. Check the
Git candidate file list and packaged archives, not only `.gitignore`: ignore
rules do not remove already tracked files or archive members.
