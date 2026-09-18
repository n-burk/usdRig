# Runtime probes for the ZebraSample gap analysis

These small stages and scripts settle the claims that
[`docs/plans/ue-zebrasample-rig-gaps.md`](../../../docs/plans/ue-zebrasample-rig-gaps.md)
rests on hardest. Appendix B of that report lists each claim and the result. They
evaluate against the build in `build/`; nothing here is part of the test suite.

| Directory | Claims | How to run |
|---|---|---|
| `offset-scheduling/` | C1a–c: a self-sourced `RigExecParentConstraint` is a stacked local offset; out-of-range weights; float-chain-driven weights. C2: a pose-interpolator weight wired into the pose walk is silently read as its authored value | `gen.py` writes the stages; `run.py <stage>...` prints J/C/K frames in dynamic, baked and parity modes (`MODES=` selects). Or `build/rigExecPose.exe <stage> --joints --targets --mode parity` |
| `typing-blend/` | C3a: double avar feeding a float math-mover input. C3b: `BlendInput` weight clamp. C3c: pose interpolator ignores control translation | `build/rigExecPose.exe <stage> --frames 1 --joints --targets --mode dynamic` (and `baked` / `parity`); use `--frames 1,6,11` for `c3a_value_animated_unauthored` and `--frames 1,2,3,4,5` for the `c3c_*` stages |
| `solver-semantics/` | C4a: two-bone soft IK + stretch discontinuity and missing stretch cap. C4b: `SplineIk` with a fourth control. C4c: `TwistDistribution` under a pure bend | `gen_twobone.py` / `gen_spline.py` write stages; `run_twobone.py`, `run_twobone_fine.py`, `run_twist.py` drive `rigExecPose` |
| `order/` | Mover order vs solver order: a constraint reading an IK joint always sees the post-IK value, and a stack that reads before the IK and then moves the IK goal is rejected as a dependency cycle (spec §4.2 would give the pre-IK read) | `gen_order.py` writes the stages; `run_order.py` compiles each in dynamic and baked mode and prints Tracker / FootIK / Ankle, or the compile error |
| `g2review/` | Nested joints/controls follow a solver-posed joint (`probe_follow*.py`); double-to-float fallback and the curve "animated unless rig-driven" blend (`probe_double.py`) | Run with the interpreter from `bin/_env.sh` but with `PXR_PLUGINPATH_NAME` **unset** -- the scripts register the plugins themselves through `tests/python/rigexec_test_env.py`, and a second registration floods the output with duplicate-type errors |
| `g1/` | USD de-duplicates relationship targets; `GetForwardedTargets` resolves a connector relationship, and remaps it through a reference | Plain `pxr`; run under `bin/_env.sh` |

Python scripts expect `bin/_env.sh` to have been sourced (for `$PY` and the USD paths),
except where noted above.
