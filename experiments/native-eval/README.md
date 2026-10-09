# Native-evaluation investigation probe

Standalone timings for the design note in
[docs/investigations/native-code-evaluation.md](../../docs/investigations/native-code-evaluation.md).
This directory is not wired into the product CMake build and does not
change the runtime.

It needs a C++17 compiler. It does not need OpenUSD: it compiles the
existing USD-free runtime sources.

```sh
cmake -S experiments/native-eval -B /tmp/rigexec-native-eval \
  -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/rigexec-native-eval -j 2
```

`probe_program` opens a `.rigexec` file, prints the baked schedule, and
times `Execute`. The rolling-ball program inside
`docs/examples/godot_rolling_ball.zip` is format 17. This branch's reader
is format 18 and refuses it, so the probe prints the refusal and skips
`Execute` until that asset is re-exported.

```sh
unzip -p docs/examples/godot_rolling_ball.zip \
  godot_rigExec/demo/rolling_ball.rigexec > /tmp/rolling_ball.rigexec
/tmp/rigexec-native-eval/probe_program /tmp/rolling_ball.rigexec
```

`bench_schedule` times `RigExecExecuteOpGraph` on synthetic graphs.
`bench_skin` times a linear-blend skin loop, the same loop compiled
ahead of time with `g++`, a persistent thread split, and a fused matrix
chain. Both print median microseconds on the machine that runs them.
