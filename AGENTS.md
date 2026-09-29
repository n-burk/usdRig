# Working in RigExec

## Project

RigExec evaluates USD character rigs without changing their source stages.
`RigExecRoot` marks a rig. Controls and solvers produce transforms; movers
revise transforms, properties, or geometry in a defined dependency order.
Hydra scene indices publish evaluated data to `usdview` and other consumers.
The root README is the build entry point; `docs/index.md` is the node catalog.

## Code map

- `libs/rigExecMath`: numerical kernels. Keep math independent of stage access.
- `libs/rigExecSchema/schema.usda`: source schema. Resources under
  `plugin/rigExecSchema/resources` are generated; regenerate with
  `bin/gen_schema.sh` or `bin/gen_schema.bat` after schema edits.
- `libs/rigExec`: dynamic evaluator, mover graph, baked program, scheduling,
  invalidation, and frame caching. Preserve dynamic/baked parity.
- `libs/rigExecRigging` and `python`: authoring APIs and Python bindings.
- `libs/rigExecImaging`: scene indices, activation, and display state.
- `libs/rigExecBake`, `libs/rigExecBinary`, `libs/rigExecRuntime`: export,
  binary layout, and playback. Check version compatibility when changing records.
- `libs/rigExecStandalone`: experimental scene adapter and rigpack path;
  distinct from `.rigexec` binary playback.
- `plugin/rigExecUsdview`: editor UI plus Qt-free interaction models.
- `plugin/usdNoodles`: bundled third-party node editor; preserve its license.
- `examples`: sample stages and procedural generators. The biped has an
  unresolved redistribution/provenance record; see `THIRD_PARTY_NOTICES.md`.
- `tests`: C++ suites, Python model tests, and graphical viewer tests.

## Build and verify

Use an unchanged OpenUSD 26.08 install with OpenExec, CMake 3.26+, Ninja,
C++17, and a compatible Python interpreter. `USD` and `PY` configure helper
scripts; their defaults look for a sibling dependency install.

1. Read relevant code and current changes before editing. Preserve user work.
2. Build with `bin/build_rigexec.sh` or `bin\build_rigexec.bat`; these also run CTest.
3. Use `bin/run_python_tests.sh` or `.bat` for Qt-free interaction changes.
4. Use the relevant `bin/run_testusdview*` helper for viewport changes.
5. Report checks actually run and any unavailable dependency or graphics test.

Use the existing tests to verify dependency invalidation, numerical conventions,
dynamic/baked parity, and binary round trips. Do not change tolerances just to
make failures disappear. Evaluation must remain non-authoring; scene edits
belong in the authoring layer. Keep USD row-vector conventions explicit at
math boundaries and avoid stage access inside pure computation callbacks.

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
