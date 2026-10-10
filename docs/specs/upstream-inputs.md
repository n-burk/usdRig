# Upstream rig inputs

An upstream Hydra scene index can supply authored-level rig inputs without
editing the USD stage. On the prim that owns an attribute, publish a
`rigExecInputs` container with an `HdSampledDataSource` child named by that
attribute. For example, the `avars:rz` child on a joint supplies that joint's
`avars:rz` attribute. Install the source before the RigExec results scene index
through the [overrides callback](hydra-integration-notes.md#62-b-usdimagingsceneindexappendcallback--createsceneindicesinfo).
The evaluator API is `SetUpstreamInputs`; its list replaces the previous list,
and an empty list lifts all standing values.

## Admission and precedence

Each key must name an existing, unconnected attribute with a stage value.
Its value must hold exactly the attribute's supported input type, and the rig
must have a listed read of that attribute. This includes admitted mover and
weight-object reads, including oracle reads. Upstream values replace the
attribute's authored value; interactive overrides take precedence, and connected
attributes are rejected. The stage remains unchanged. Admitted paths are sorted
by `GetUpstreamInputPaths`.

Unsupported keys are ignored with a diagnostic:

```text
upstream input <path>: <reason>; ignored
```

Array admission additionally requires the same element count as the stage value
at the evaluated time. A constant stage count is memoized until a relevant stage
edit; varying counts are checked at the time of admission. The supported array
pool types are int, float, double, vec2f and vec3f. The exact admitted paths,
types, time kinds and consumers come from
`RigExecBakedUpstreamAdmissibleArrays(evaluator)`.

Array admission currently defaults off and is available through the test hook
`RigExecSetUpstreamArrayAdmissionForTesting`. Structural arrays are excluded:
painted values and indices read per element by weight packets, folded solver
volume weights, ribbon structural points and Derived bases do not become
upstream inputs. A geometry chain's base points can be admitted. A path that a
weight packet reads per element is structural even when it is also a chain base.

## Time and source changes

`T0` is the input scene index's time from the latest RigExec time trigger.
Data-source sample offsets are relative to `T0`; stored sample times are absolute
(`T0 + offset`). Pulls run on time triggers, `rigExecInputs` additions or dirties,
and warm-range changes, without holding the registry lock.

For each source, RigExec asks `GetContributingSampleTimesForInterval` once for
the pull window. A uniform source is read at offset zero. A varying source is
read only at the returned sample offsets. Values at `T0` and warm frames are
reconstructed from those samples using the stage's interpolation setting.
Linear interpolation applies to float, double, vec3f, vec3d and matrix4d values,
and to equal-length float, double, vec2f and vec3f arrays. Other types, unequal
array lengths and Held interpolation hold the preceding sample. Boundary samples
hold beyond the returned sample times within the pull window.

An in-window time move with unchanged sources reanchors the stored table without
calling the source again. A varying source has no pulled value outside that
window. A source must dirty `rigExecInputs` when its samples or source set change;
a time trigger alone does not announce a source change. A dirty during a pull
permits one deferred pull. Another dirty during that deferred pull is dropped
with a diagnostic. With several scene-index chains, the triggering chain supplies
the session's table; conflicting keys are diagnosed.

These rules require sources to answer offsets relative to the trigger's `T0`.
A lagging trigger produces values anchored to the wrong time. Long warm windows
or large varying arrays can make the pull expensive, even though workers never
call the source.

## Warming and cache reuse

Live evaluation and warm jobs consume the pulled table. Warm jobs reconstruct
arrays from shared sample buffers and carry a per-frame fold hash; the table does
not store an array copy for every frame. Uniform and held values share buffers.
Frame-cache keys include the frame's admitted upstream values, and sparse reuse
misses when an upstream value changes. A missing varying value skips that warm
frame with `upstream not pulled for this time`.

A source-set, window or frame-value change advances the session's upstream serial
and cancels the live session's warm generation. Moving only `T0` through the same
signal does not. The lookup fast path relies on a completed warm row's generation
postdating the last upstream source change. A burst is available only when no
upstream source varies, and its pins include the upstream serial. Preview updates
do not change upstream inputs. Playback sessions consume the table but do not
warm frames.

## Binary playback

Current format-7 playback admits scalar keys only when `FindInput` finds the
attribute, the file's input tag matches the value type, and the stage attribute
is unconnected and has a value. Drop diagnostics follow live admission.

Playback applies upstream values after the stage sampler's `Apply`, without
invalidating the sampler. Numeric keys use `SetSampledInputAt`; Token keys use
`SetInputToken` with their text, including text absent from the file. Animated
keys are reapplied whenever `Apply` sampled the stage. Lifting a non-Animated key
uses `ResetInput`. Lifting an Animated key restores the sampler's value when it
just sampled, otherwise it rereads that one stage attribute at the playback time;
a failed read uses `ClearInputAt`.

Playback applies arrays with the authored `SetInputArray` API. Admission checks
against the stage count at evaluation time; the runtime additionally requires
the stored default's count. A refused set lifts the key and reports the refusal.
Animated arrays with actual AtTime consumers restore the current stage sample
when lifted, including a same-time lift or a first refused value. Default-only
arrays use `ResetInput` even when numeric samples mark their slots Animated.
Standing upstream arrays are reapplied after stage sampling.

The stage sampler copies Animated AtTime arrays of any element count. Each
consumer validates the resulting count as live evaluation does. An oracle's
failed `sampleSource` Get falls back to the raw canonical `weightTarget`;
successful empty samples take precedence and undergo the normal count check.
A failed Get clears HasValue; a successful empty array remains a value. Both leaves retain
separate captured Default data, so stage samples affect only AtTime reads.
Eligibility comes from the file's actual chain base, layout, dense sample,
AtTime path-read and oracle consumers. Structural and Default-only leaves are
excluded; wire knots (`double[]`) are currently Default-only. Animated skin
layouts retain the exporter's existing refusal. Static reports omit only arrays
covered by this same sampler inventory.

Only the evaluator's admitted array paths are public file inputs. Structural
painted arrays and excluded point/oracle arrays remain captured as private typed
storage slots and are absent from `GetInputInfo`, `FindInput` and public set
APIs. An internal stage-only bridge samples eligible private oracle arrays,
without granting authored overrides or changing public input counts. Static
private arrays retain their captured defaults.

An authored upstream array replaces both AtDefault and AtTime reads. With a
varying upstream source, each evaluated frame's upstream value also feeds its
AtDefault leaves; it does not freeze those leaves at the bake time. If one cage
attribute supplies both rest and live lattice reads, both see the standing value.
A stage time sample alone reaches AtTime reads only. Format-8 playback must keep
this distinction when applying upstream arrays.

## Baking

A bake accepts admitted upstream scalars and arrays. It suspends the complete
requested list while evaluating and capturing the authored stage, then restores
that list on success or failure. Every standing admitted path must be a listed
file input; a missing slot reports an exporter invariant defect. File defaults
and static data remain authored-stage values, so the bytes equal a bake with
upstream values lifted. `RigExecBakeResult::upstreamInputs` reports the sorted
admitted names on success; callers retain and reapply their values through the
scalar or authored array input API. Authored array sets still require the file's
default element count. Interactive overrides still refuse a bake.
