# Layer readiness

RigExec is work in progress. Every layer below is usable but unfinished and could be volatile:
APIs, behavior, and file formats may change without notice.

Readiness levels used here:

- **Prototype.** Works end to end for the covered cases; expect gaps,
  churn, and breaking changes.
- **Beta.** More mature and broader in coverage, but still pre-stable;
  details may change.
- **Stable.** Not claimed by any layer yet.

| Layer | Covers | Readiness | Notes |
|---|---|---|---|
| Math kernels | `libs/rigExecMath`: solvers, deformation, interpolation, weight math | Prototype | Conventions are pinned by regression suites; coverage keeps growing. |
| Schema and authoring | `libs/rigExecSchema`, `libs/rigExecRigging`, `python`: scene description and authoring APIs | Prototype | Schema and authoring surface may change. |
| Dynamic evaluator | `libs/rigExec`: evaluator, mover graph, scheduling, invalidation, frame cache | Prototype | Being revisited; expect active revision and churn here first. |
| Baked program | `libs/rigExec`: baked program and parity with the dynamic path | Prototype | Must keep matching dynamic results; see [evaluation modes](docs/concepts/baked-vs-dynamic.md). |
| Display | `libs/rigExecImaging`: Hydra scene indices and viewport publication | Prototype | Display state stays separate from authoring state. |
| Touch Pose | `plugin/touchPose` with [touch regions](docs/nodes/touch_regions.md) | Beta | More mature than the surrounding prototype layers; details may still change. |
| Picker | `plugin/rigExecUsdview` picker UI with [picker panels](docs/nodes/picker.md) | Beta | More mature than the surrounding prototype layers; details may still change. |
| Export and runtime | `libs/rigExecBake`, `libs/rigExecBinary`, `libs/rigExecRuntime`: experimental `.rigexec` export and playback | Prototype | Format changes require explicit compatibility testing. |
| Standalone adapter | `libs/rigExecStandalone`: experimental Esf adapter and rigpack backend | Prototype | Narrow supported surface; check its guide before integration. |

The [architecture guide](docs/specs/spec.md) explains how these layers relate.
Current behavior is defined by source and tests rather than by this page;
treat any mismatch as a prompt to update the page.
