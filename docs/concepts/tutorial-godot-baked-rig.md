---
title: "Tutorial: Godot and baked rigs"
summary: Play the packaged rolling-ball game, and see how its baked file relates to the current striped stage.
order: 40
---

The packaged Godot game loads one `rolling_ball.rigexec` file. That file
holds a baked rig program plus an extra presentation section the tutorial
exporter appends. OpenUSD is not required to play it.

[Download the Godot project](../examples/godot_rolling_ball.zip). It includes
the scenes, the baked asset, the addon source, and tested Windows x86-64
libraries. Open `demo/project.godot` in **Godot 4.7+** and press **F5**.
WASD or the arrow keys move, Space hops, Shift brakes, and R resets. Collect
the six rings and reach the exit.

The game script owns collision and input. It drives two public controllers,
`Move.tx`, `Move.ty`, `Move.tz`, `Roll.rx`, `Roll.ry`, and `Roll.rz`, through
`set_control()`. `evaluate()` runs the baked program and refreshes the
render mesh. Joints stay inside the program. The legacy skeleton adapter is
still in the player and is unused by this scene.

## What the packaged file contains

`rigExecBake` writes the program. It does not write render geometry. The
tutorial exporter then appends binary section tag 13, `Presentation`, as
JSON: version 1, public controls, subdivided mesh indices, UVs, subdivision
stencils, and a base64 PNG. `libs/rigExecBinary` names that tag and the core
runtime never decodes it, so playback of the program still succeeds. The
Godot player reads the section itself.

`set_control(name, value)` looks up the USD property path stored for that
public name and calls `RigExecRuntimeReader::SetAvar`. With a presentation
section present, `set_avar()` is refused. `get_controls()` lists the public
names and hides the USD paths. `reset_controls()` clears the overrides.

The file shipped in the zip was baked from an earlier textured stage. Its
embedded presentation still carries that texture, face-varying UVs, and a
UsdPreviewSurface-style material (diffuse scale 0.75, emission scale 0.5,
roughness 0.5, metallic 0, specular 0.35, repeat on S, clamp on T). Playing
the zip shows that embedded data. It does not read
`docs/examples/tutorial_rolling_ball.usda`.

## The stage in this checkout

The stage checked in now is a yellow sphere of radius 1 with a black
equatorial stripe in per-point `displayColor`. It has no UVs and no bound
material. Scrub 1001–1049:

```
bin/usdview.sh docs/examples/tutorial_rolling_ball.usda
bin\launch_usdview.bat docs\examples\tutorial_rolling_ball.usda
```

`/BallAsset/Rig` is a `RigExecRoot`. Nested controls `Move`, `Squash`,
`Roll`, and `Spin` feed a `RigExecFkChain`. `RollFromTravelX`, a
`RigExecFloatMathMover`, multiplies `Move.avars:tx` by the authored degrees
per unit on `Roll.avars:rz`, so the straight-line shot rolls without a
separate spin key. `BallSkin`, a `RigExecMatrixMover`, carries
`/BallAsset/Geom/Ball.points` from the Spin joint.

[`tutorial_rolling_ball_free.usda`](../examples/tutorial_rolling_ball_free.usda)
references that asset and selects the `free` variant. The variant deactivates
`RollFromTravelX` and zeros `Roll.avars:rz`, so gameplay can set rotation
itself. The wrapper also publishes the game interface:

```usda
over "Move" {
    custom string rigExec:publicName = "Move"
    custom token[] rigExec:exposedAvars = ["tx", "ty", "tz"]
    over "Squash" {
        over "Roll" {
            custom string rigExec:publicName = "Roll"
            custom token[] rigExec:exposedAvars = ["rx", "ry", "rz"]
        }
    }
}
```

The exporter turns those declarations into `Move.tx` and `Roll.rx` style
names. Rotations are degrees. The wrapper's `metersPerUnit` is 1, so one
authored unit is one metre and the radius-one ball matches the game.

The [usdview package](../examples/usdview_rolling_ball.zip) is this striped
stage plus the icons it references. Extract the whole archive so
`docs/examples` and `icons` stay beside each other.

## Baking

`demo/setup_rolling.py` expects this layout, and the directory name `usdRig`
is what the script looks up:

```text
workspace/
    usdRig/
    godot_rigExec/
        addons/rigexec/
        demo/
        tools/
    usd-install/
```

Close the Godot editor before setup. On Windows it holds the addon DLL open.
From `godot_rigExec` the script runs the bake without `--poseable`:

```text
rigExecBake docs/examples/tutorial_rolling_ball_free.usda --frames 1001 -o demo/rolling_ball.rigexec
```

That is enough for `SetAvar` on a compiled control's translate, rotate, or
scale channel. `--poseable` is a separate switch: it also records every
overridable input and the property chains as programs. This tutorial does
not pass it. Frame 1001 is the only sample. The free variant is the one on
the wrapper, so the travel-driven roll mover stays off.

`python tools/export_ball_assets.py` is the second phase. It opens the same
free stage and requires all of the following on `/BallAsset/Geom/Ball`:

- `subdivisionScheme` of `catmullClark`
- face-varying primvar `st`
- a bound `UsdPreviewSurface` whose diffuse and emissive colors are
  `UsdUVTexture` nodes, wrap S `repeat`, wrap T `clamp`, sharing one PNG

The current ball is `catmullClark` and has vertex `displayColor`. It has no
`st` primvar and no material, so those asserts fail. `python demo/setup_rolling.py`
therefore cannot rebuild the packaged asset against this checkout. The
Windows libraries and `demo/rolling_ball.rigexec` already in the zip are
what the game plays.

Native rebuilds of the addon, once a stage satisfies the exporter, also need
SCons, a C++ toolchain, OpenSubdiv, and this pin of godot-cpp:

```text
git clone https://github.com/godotengine/godot-cpp.git thirdparty/godot-cpp
git -C thirdparty/godot-cpp checkout 507ed9d840c01a3c5b2a39af8bb4000bfac30bf5
git -C thirdparty/godot-cpp submodule update --init --recursive
python demo/setup_rolling.py --build
```

This environment has no OpenUSD install and no Godot, so the playable zip
was inspected and not launched, and the bake was not re-run.
