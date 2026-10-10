# Source input replay

The optional source recorder logs caller inputs and authoring operations for
`RigExecRigEvaluator`. It never receives a pose, computed property, weight
packet, resolved input pool or numerical output. Its normal disabled path
allocates nothing, registers no notices and reads no source stage.

Set `RIGEXEC_INPUT_REPLAY=capture:<new-file>` and
`RIGEXEC_INPUT_REPLAY_PROVENANCE=<source-and-plugin-manifest>` before running a
fixture. The transcript requires a new file; it does not overwrite an existing
input artifact. The provenance file's exact bytes are retained and checked
again during replay. Keep the source manifest, suite source hashes, original
library and plugin source provenance, loader paths, exit status and transcript
SHA together. An incomplete or refused capture is not numerical evidence.

The recorder tracks non-owning stage/layer identities and evaluator actions. It captures
initial authored specs and typed fields, source-file bytes, session layers,
resolver search paths, population masks, payload load rules, muted layers and
identity edit targets. Subsequent field, dictionary, time-sample, namespace
and child-list operations retain their exact types and order. Sdf notice
batches seal the operation sequence. Replay applies each batch in one
`SdfChangeBlock` and checks the same layer/path notice summary before the next
caller action. Layer collections use stable IDs rather than memory addresses.
Initial anonymous layer aliases bind before any evaluator exists; ordinary
strings and customData strings keep their original bytes.

Known `RigExecInputReplayImportFromString` callers record their actual authored
text and layer identity before invoking the same USD import. Replay uses that
same input operation inside its original notice batch. The adapter suppresses
individual delegate records only during that known import, and allows its
replacement notice only when the batch contains that recorded operation.
It never generates text from computed or captured numeric values. Failed
imports, unadapted replacement calls and notice differences are refused.

Caller events record explicit Compile boundaries, requested interactive and
upstream vectors before admission, vector order and duplicates, clears,
guide/weight-field publication options, exact Default versus numeric-time
identity, repeated visits and destruction. Each evaluator survives its whole
recorded history. Internal validation evaluators and Compile calls inside
Evaluate do not manufacture extra caller events. Profiling and CPU reference
validation options are observational and are excluded.

The input-action wire writes version 5 and still reads version 2, 3 and 4 histories.
Version 5 records event 24, the actual process-global upstream array-admission
flag as one canonical boolean byte. Its initial state is the first event, even
when a setter runs before any evaluator exists. Each actual setter call records
its requested flag in caller order before mutation, including restoration. Replay
uses the same public setter and restores its entry flag on success or refusal.
Versions 2, 3 and 4 reject this new event; malformed booleans, missing initial
state and controls inside unfinished edit/comparison scopes are refused.

The maintained held-program adapter records real Build, Run, value-edit router
activation/deactivation and destruction on the same evaluator. Build retains
its guide/publication options and whether a reasons output was requested. Run
records requested time and a fresh-output time-seed policy. Actual Build
bool/reasons, Run bool/bail and untouched pose/time are separate passive golden
observations, never source inputs. Replay retains program identity and notice
order; the router closes before the program and the program before its owner.
Unadapted raw program mutations remain outside this contract. Completion checks
the exact program/run inventory, including units with no held programs.

Version 4 additionally records actual source stage/layer expiry at safe caller
boundaries. The observer holds weak source identities; evaluator destruction
alone is not source retirement. Replay releases only explicitly retired IDs,
rejects live-owner or live-stage retirement and rejects later use of retired
identities. Historical IDs and end counts remain retained.

Explicit Compile actions preserve whether a diagnostic pointer was supplied and
its initial ordered string buffer. Actual return booleans and full post-call
diagnostics are separate passive outputs under `RIGEXEC_COMPILE_GOLDEN`, never
expected-result action inputs. Null pointers remain null. Each recorded call
requires one result; exceptions leave incomplete evidence. Raw replay refusal
exit codes remain unchanged. A complete ordered Current/Original result-ledger
comparison may separately establish that negative Compile calls matched; it
never changes the native exit or excuses malformed/unrecorded actions.

Numeric values use a versioned binary codec with explicit scalar/component
types and raw bits. Float, double, half, vectors, matrices, quaternions and
typed arrays preserve signed zero and NaN payloads. Structural values,
dictionaries, references, payloads, list operations, time maps and supported
TsSpline components have explicit encodings. Reconstruction must re-encode
identically. Unknown types or setter canonicalization fail explicitly. Empty
values, erased samples, blocks and successful empty arrays remain distinct.
USDA and USDC are not numeric transport substitutes; crate floating-array
compression can lose signed zero.

The supported replay tool is:

```text
rigExecReplayInputs <input-actions> <core-schema-resources> [mover-plugin-resources ...]
```

Disable input capture in the replay process. The current tool requires
`RIGEXEC_GOLDEN_SUITE=check:<original-judge>` and checks the original numerical
judge. A separately compiled original host requires
`RIGEXEC_GOLDEN_SUITE=capture:<fresh-judge>` and uses the unchanged original
kernels and original execution policy. Both require the same suite name and
immutable input transcript. Source and plugin SDKs must match each host.
The original observer verifies evaluator identity, visit order, time, typed
publication values and complete evaluator/generation inventory. Current
output capture is never an original judge.

Capture fails for custom layer delegates or resolver contexts, mapped/variant
edit targets, unadapted bulk import/reload/replacement or layer identity changes,
permission/delegate mutation, concurrent or reentrant source histories,
caller actions inside unfinished authoring batches, and source authoring
during numerical Compile, Evaluate or adapted held-program Run. Post-construction population-mask, payload-load,
mute or resolver changes require an entry adapter; a stage notice refuses the
first changed configuration, including a later change-and-revert sequence.
A layer newly reachable from an existing stage or
newly chosen as its edit target requires an entry adapter and is refused;
reconstructing it late could introduce extra invalidations. Unsupported cases name the source/action rather
than silently falling back. Identity edit targets among already recorded
layers are sampled at caller boundaries. Explicit composition API calls that
leave their configuration unchanged are outside the recorder contract;
their entry chronology needs an adapter. Arbitrary application callback
behavior is not serialized.

This protocol currently covers native evaluator caller histories. Direct
frozen/runtime object setters, execute/clone APIs, application callbacks and
plugin registration histories require explicit adapters and source
provenance. A native transcript's strict end index does not establish that
those other APIs were recorded. Full suite coverage requires a maintained
inventory of actual recorded histories and unsupported cases, then successful
original capture and current replay. Changed suite constructors or visit
counts do not justify replacing the original judge. Intentional numerical
changes require exact separately recorded surfaces and independent proofs.

For the maintained suite input harness, set
`RIGEXEC_INPUT_REPLAY_NATIVE_HISTORIES=1` in both input capture and the actual
current suite's original-judge check. Named Build/Freeze/frozen-comparison
scopes execute the existing current calls unchanged and record balanced
boundaries. Source changes and nested native calls inside those scopes are
refused. Original native replay validates those boundaries without running
current comparison code. These boundaries establish native caller chronology;
they do not claim an original frozen/runtime numerical judge.

The PropertyOps direct successful program run has an explicit adapter: the
actual run is followed by a real owning native Evaluate with the same authored
inputs, options and time; exact publication bytes must agree. Original replay
independently judges that added native visit. No produced value is passed as
input. This changes that harness's visit count and is recorded in its source
provenance. The later epoch-binding refusal retains its false/empty status
assertions and the unchanged original negative test; it has no mathematical
native counterpart. Distinct direct overrides or frozen/runtime histories
without such a counterpart need their own input adapter and coverage entry.


Observation hosts must complete their observers before Windows process teardown.
The build tree adds `observerHostExit.cpp` to executable consumers only; DLLs and
static archives do not compile that shim. Installed/custom opt-in hosts explicitly
compile the maintained shim or call `RigExecFinalizeInputReplay()` and
`RigExecFinalizeGoldenSuite()` after all owners close and before returning from
main. Both completion APIs are idempotent. Relying only on a DLL-owned `atexit`
callback can report a late failure without changing Windows' process exit status.
The installed target exports no absolute source path or automatic consumer shim.

The exact `ImportFromString` adapter retains its caller text, layer identity and
notice boundary. USD computes that action's replacement diff through
`SdfAbstractData::VisitSpecs`, whose spec order is undefined. The recorder and
replayer sort that known action's notice entries by path; flags, typed old/new
values, field order and sublayer changes remain exact. Ordinary intercepted
edit batches retain their original entry order. A supported regression changes
a replacement notice flag and requires the strict notice audit to refuse it.
The known import must be the sole action in its source notice batch; mixed
ordinary edits and imports, or multiple imports in one batch, are explicitly
refused. This prevents import normalization from relaxing ordinary edit order.

Exact source replacement adapters also cover `Clear` and `TransferContent`.
Transfer captures its source layer's authored typed state and shared identity;
replay invokes the original SDK method. Each replacement must be the sole
action in its source notice batch. Unsupported unadapted replacements are
refused at the source delegate using USD 26.08's official
`Setting layer data` scope description. This opt-in caller-thread check uses
an internal SDK registry spin lock; it runs after the numerical-authoring
refusal and exact-wrapper suppression, and never in a parallel numerical body.
The disabled recorder does not install the delegate or query this registry.
