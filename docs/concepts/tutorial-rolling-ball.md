---
title: "Tutorial: a rolling ball rig"
summary: Build the classic bouncing-ball rig in usdview, node by node, and make the roll a consequence of the travel instead of a channel to key.
order: 30
---

The oldest rigging exercise there is: a ball that rolls along the ground
without sliding. It is small enough to build in one sitting and it uses
most of what UsdRig is — a control hierarchy, an FK solver, joints, a
weight object and a mover — plus one thing that is easy to get wrong and
worth getting right, which is making the **spin follow the travel** so
there is only one channel to animate.

This page explains how to build it in `usdview`, through the **usdNoodles**
node graph editor. The GIFs are captioned inspection checkpoints of the
completed asset in the real application, with the corrected top-centred star
and continuous equatorial stripe. They pause for 4.5 seconds and retain the
native UI resolution; open them directly to read the attributes. The finished stage is
[`docs/examples/tutorial_rolling_ball.usda`](../examples/tutorial_rolling_ball.usda)
and the snippets below describe its composed `travelX` mode. The asset also
has a `free` mode for the interactive Godot game described below.

[Download the USD source, texture and icons together](../examples/usdview_rolling_ball.zip).
Extract the entire archive, then open `docs/examples/tutorial_rolling_ball.usda`.
The `tutorial_rolling_ball_free.usda` wrapper uses the same textured mesh for Godot.

## What you start with

The geometry, and a rig root with nothing in it. Open
`docs/examples/tutorial_rolling_ball.usda` and delete everything under
`/BallAsset/Rig`'s five scopes to get back to this, or point the same
steps at a stage of your own — the pieces are:

* `/BallAsset/Geom/Ball` — a 32 × 16 lat/long sphere of radius 1, centred
  at `(0, 1, 0)` so it sits ON the ground and the contact point is the
  origin. It carries `primvars:st` and a `UsdPreviewSurface` reading
  `textures/pixar_ball.png`: yellow, one broad blue band, one red star.
  The graphic is not decoration — an untextured sphere spinning about its
  own centre looks perfectly still, and you cannot see whether a rolling
  rig is rolling or sliding.
* `/BallAsset/Geom/Ground` — a checkered plane.
* `/BallAsset/Rig` — a [Rig Root](../nodes/rig_root.md) with empty
  `Controls`, `Joints`, `Solvers`, `Weights` and `Movers` scopes.
* `/BallAsset/MainCam` — an authored camera for inspecting the scene.

Open it with `bin/launch_usdview.bat docs/examples/tutorial_rolling_ball.usda`
(or `bin/usdview.sh`), then **Window ▸ Noodles Editor**. The editor opens
docked; it is scoped to the rig, so the graph starts on
`/BallAsset/Rig` rather than on the asset container.

## Step 1 — point your edits at the file

![Source stage and the layer editor's edit-target choices](../gifs/concepts/ball_step01_edit_target.gif)

Do this first, every time. usdview opens with the **session layer** as the
edit target, and the session layer is thrown away when the application
closes. Everything you are about to build would go with it.

**In the UI:** **Window ▸ Layer Editor** opens the Noodles layer editor
beside the graph. It lists the stage's layers with a ● on the current edit
target, and a warning strip under them: *"Authoring to the session layer,
changes will not be preserved on save"*. Click the **Root Layer** row. The
● moves, the warning clears, and the graph's own banner goes with it.

From here on every prim, relationship and value lands in
`tutorial_rolling_ball.usda`, and **Ctrl+S** in the graph saves it.

## Step 2 — put the stage in the graph

![Inspecting the geometry, rig scopes and textured material](../gifs/concepts/ball_step02_stage.gif)

The editor draws the prims you ask it to. Select `/BallAsset/Rig` in
usdview's prim browser and press **A** over the graph canvas ("add from
prim tree"); do the same for `Geom/Ball` and `Geom/Ground`.

A mesh node arrives with a row per property — over a hundred of them for a
sphere — so click the **caret in its title bar** to collapse it, then drag
each node by its title into a tidy left-to-right layout. This is not
housekeeping: relationships are authored by dragging from a pin onto a
node, and a node under another node is a relationship pointed at the wrong
prim.

## Step 3 — the Move control

![Inspecting the completed Move control](../gifs/concepts/ball_step03_move_control.gif)

A [Control](../nodes/control.md) is the animator-facing prim: an
animatable coordinate space whose animation is authored on its `avars`.
`Move` is the one that travels along the ground.

**In the UI:** select the `Controls` scope in the prim browser (that is
where the new prim goes), hover the graph, press **Tab** to open the node
creation hotbox, type `RigExecControl`, press **Return**. The prim is
created as `RigExecControl1`; **double-click its title**, clear it, type
`Move`, **Return**. Then, with the node still selected, press **Tab**
again and type `RigExecControlAPI` — the same hotbox lists every applied
API schema in the build beside the concrete prim types, and choosing one
applies it to the selection.

```usda
def RigExecControl "Move" (
    prepend apiSchemas = ["RigExecControlAPI", "NodeGraphNodeAPI"]
)
{
    double avars:tx = 0
    double avars:tz = 0
    matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
}
```

Nothing else is authored. `guide:shape`, `guide:drawMode`,
`guide:scaleX/Y/Z` and `guide:wireWidth` all have schema fallbacks that
draw exactly the wire circle you want, and a rig should not carry an
opinion that says what the schema already says.

## Step 4 — the Squash control

![Inspecting Squash under Move](../gifs/concepts/ball_step04_squash_control.gif)

Same three actions, with `Move` selected so the new control lands
underneath it. `Squash` carries the scale avars.

```usda
def RigExecControl "Squash" (
    prepend apiSchemas = ["RigExecControlAPI", "NodeGraphNodeAPI"]
)
{
    double avars:sx = 1
    double avars:sy = 1
    double avars:sz = 1
    matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
}
```

Its rest space is the **ground**, not the ball's centre, and it sits
**above** the roll in the hierarchy. Both of those are the whole trick of
a squashing ball: scaling about the ground keeps the contact point on the
ground, and squashing above the spin keeps the squash vertical while the
ball turns inside it. Swap the two and the ball squashes sideways as it
rolls.

## Step 5 — the Roll and Spin controls

![Inspecting the Roll and Spin controls](../gifs/concepts/ball_step05_roll_control.gif)

`Roll` is the control the rig drives for you; `Spin` is the one you can
still key by hand when a shot needs the ball to scuff or spin on the spot.

```usda
def RigExecControl "Roll" (
    prepend apiSchemas = ["RigExecControlAPI", "NodeGraphNodeAPI"]
)
{
    double avars:rz = -57.29578
    matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 1, 0, 1) )

    def RigExecControl "Spin" (
        prepend apiSchemas = ["RigExecControlAPI", "NodeGraphNodeAPI"]
    )
    {
        double avars:rz = 0
        matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
    }
}
```

Two values here are not cosmetic.

`rest:space` puts `Roll` at `(0, 1, 0)` — the ball's centre. A rotation
turns about its own frame's origin, and a roll about the ground would swing
the ball through the floor.

`avars:rz = -57.29578` is not a pose; it is the **gain**, the degrees of
roll per unit of travel, and Step 9 multiplies it by the travel. It is an
avar, so the Avar Editor types it — and
[Step 9](#step-9--make-the-roll-a-consequence-of-the-travel) is where
this page sets it, next to the mover that reads it.

## Step 6 — the joints

![Inspecting the four-joint hierarchy](../gifs/concepts/ball_step06_joints.gif)

Controls are what an animator touches; [Joints](../nodes/joint.md) are
what a solver writes and a mover reads. This rig needs one per control, in
the same shape: `Root ▸ Squash ▸ Roll ▸ Spin`. Create them the same way —
**Tab**, `RigExecJoint`, **Return**, rename — with the parent joint
selected each time. No API schema: a joint needs none.

```usda
def RigExecJoint "Root"
{
    matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )

    def RigExecJoint "Squash"
    {
        matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )

        def RigExecJoint "Roll"
        {
            matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 1, 0, 1) )

            def RigExecJoint "Spin"
            {
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
            }
        }
    }
}
```

`Roll`'s bind frame is the ball's centre, for the same reason the
control's is: the mover divides the posed frame by the rest frame, and the
two have to agree about where the ball turns.

## Step 7 — the FK chain

![Inspecting the FK chain and its control and joint relationships](../gifs/concepts/ball_step07_fk_chain.gif)

An [FK Chain](../nodes/fk_chain.md) is the solver that maps controls onto
joints, one to one, in order.

**In the UI:** select the `Solvers` scope, **Tab**, `RigExecFkChain`,
**Return**, rename to `BallChain`. The new node opens with its rows
showing: **drag from its `rigExec:controls` pin onto each control node**
in turn — `Move`,
`Squash`, `Roll`, `Spin` — and from `rigExec:joints` onto `Root`,
`Squash`, `Roll`, `Spin`. The order you drop them in is the order the
chain pairs them in.

```usda
def RigExecFkChain "BallChain"
{
    rel rigExec:controls = [
        </BallAsset/Rig/Controls/Move>,
        </BallAsset/Rig/Controls/Move/Squash>,
        </BallAsset/Rig/Controls/Move/Squash/Roll>,
        </BallAsset/Rig/Controls/Move/Squash/Roll/Spin>,
    ]
    uniform token rigExec:controlSpace = "parentRelative"
    rel rigExec:joints = [
        </BallAsset/Rig/Joints/Root>,
        </BallAsset/Rig/Joints/Root/Squash>,
        </BallAsset/Rig/Joints/Root/Squash/Roll>,
        </BallAsset/Rig/Joints/Root/Squash/Roll/Spin>,
    ]
}
```

`rigExec:controlSpace = "parentRelative"` is required and is **not** the
fallback. The fallback is `world`, which reads each control's whole
world transform and applies it to the joint; with a nested control
hierarchy that composes every ancestor's motion twice and the ball leaves
the ground within a few frames. See
[What no editor can author](#what-no-editor-can-author) — this is one of
the five values you have to type.

## Step 8 — the skin

![Inspecting the weight and matrix mover binding the ball to the last joint](../gifs/concepts/ball_step08_skin.gif)

One [Matrix Mover](../nodes/matrix_mover.md) carries the whole ball from
the last joint, under a [Static Weight](../nodes/static_weight.md) of 1.

**In the UI:** create `RigExecStaticWeight` under `Weights` and drag its
`rigExec:weightTarget` pin onto the **`points` row** of the `Ball` node —
a weight object is a field over a property's elements, not over a prim.
Create `RigExecMatrixMover` under `Movers`, apply `RigExecMoverAPI` from
the hotbox, then drag `rigExec:moves` onto `Ball.points`,
`rigExec:transform` onto the `Spin` joint and `rigExec:weightObject` onto
the weight.

```usda
def RigExecStaticWeight "BallWeight"
{
    uniform float rigExec:defaultWeight = 1
    rel rigExec:weightTarget = </BallAsset/Geom/Ball.points>
}

def RigExecMatrixMover "BallSkin" (
    prepend apiSchemas = ["RigExecMoverAPI", "NodeGraphNodeAPI"]
)
{
    rel rigExec:moves = </BallAsset/Geom/Ball.points>
    rel rigExec:transform = </BallAsset/Rig/Joints/Root/Squash/Roll/Spin>
    rel rigExec:weightObject = </BallAsset/Rig/Weights/BallWeight>
}
```

The weight's representation stays at its `constant` fallback and its
envelope is `defaultWeight = 1`: every point of the ball is carried
whole. A `dense` array of per-point weights is what you reach for when
part of a mesh follows a joint and part of it does not — here nothing
does.

Select `Move`, pick **Move** on the viewport toolbar (or press **W**) and
drag the red X handle. The ball follows the control. It also **slides**,
which is the problem the next step solves.

## Step 9 — make the roll a consequence of the travel

![Inspecting the roll gain and float math mover](../gifs/concepts/ball_step09_auto_roll.gif)

This is the step the whole tutorial is for.

A ball of radius *r* that rolls without slipping turns by the arc it
covers: rolling forward a distance *d* turns it *d / r* radians. Travelling
along **+X** turns it about **−Z**, so

```text
rz(radians) = -tx / r
rz(degrees) = -tx * 180 / (pi * r)
            = -57.29578 * tx          for r = 1
```

One full turn is 360° and carries the ball `2 * pi * r` = 6.2832 units.
Over the 8 units this shot travels the ball turns −458.37°, one and a
quarter revolutions.

You could key that by hand on `Roll.avars:rz`, and it would be wrong the
first time anyone changes the timing. Instead, derive it. A
[Float Math Mover](../nodes/float_math_mover.md) revises one exact float
property; its `inputs:value` accepts a **connection**, so it can read a
number from somewhere else on the stage. Point it at the travel, multiply,
and write the answer into the roll:

```usda
def RigExecFloatMathMover "RollFromTravelX" (
    prepend apiSchemas = ["RigExecMoverAPI", "NodeGraphNodeAPI"]
)
{
    float inputs:value = 0
    float inputs:value.connect = </BallAsset/Rig/Controls/Move.avars:tx>
    rel rigExec:moves = </BallAsset/Rig/Controls/Move/Squash/Roll.avars:rz>
    uniform token rigExec:operation = "multiply"
}
```

`multiply` computes `r = incoming * inputs:value`, where *incoming* is the
target's own authored value — the −57.29578 gain from Step 5 — and
`inputs:value` is the travel arriving down the connection. So

```text
Roll.avars:rz  =  -57.29578  *  Move.avars:tx
```

and the roll is recomputed every frame from wherever the ball happens to
be. There is nothing to key on it and nothing to keep in sync.

**In the UI:** select `Roll`, open the Avar Editor
(**RigExec ▸ General Editors ▸ Avar Editor**), set its **Write** mode to
**Default (the rest value)** — a gain is not a pose and has no business
being a key — and type the number into the `rz` box. The box carries
three decimals, so what you type is `-57.296`; over the whole eight-unit
shot that is two thousandths of a degree away from the exact figure.
Commit it with **Tab**.

Then create `RigExecFloatMathMover` under `Movers`, apply
`RigExecMoverAPI`, drag `rigExec:moves` onto the **`avars:rz` row** of the
`Roll` node, and drag from the **right-hand edge of the `avars:tx` row on
the `Move` node** onto the mover's **`value`** row (the row `inputs:value`
is drawn as, under the node's `inputs` header). A property row draws no
output dot until something is connected to it; the drag starts from the
row's edge all the same. That second drag is an attribute
connection rather than a relationship — a value travelling, not a
reference.

Property chains resolve *before* exec runs, so the revised `rz` is what
the FK chain sees in the same evaluation; nothing is a frame late.

Drag `Move`'s X handle again and the ball rolls, star and band sweeping
over the top, with no keys on any rotation.

For a straight shot along Z, the analogous mover reads `Move.avars:tz`,
targets `Roll.avars:rx`, and uses a gain of **+57.29578**. Do not combine
independent position-to-Euler mappings for free movement: rotations about
different axes do not commute. A ball that turns corners needs accumulated
orientation, as in the free-rolling mode below.

## Step 10 — key the travel, and only the travel

![The Move control and automatic roll relationship used by the two-key animation](../gifs/concepts/ball_step10_keys.gif)

Select `Move`. Put the gizmo on **Animation** on the viewport toolbar and
the Avar Editor's own **Write** box on **Animation (key at current
frame)** — that is what makes an edit author a key at the current frame
instead of a default. Go to frame 1001, type `0` into the Avar Editor's
`tx` box and press **Tab**; go to 1049, type `8`, **Tab** again. (Tab is
the commit key here: Return leaves the box showing the old number.)

```usda
double avars:tx.spline = {
    1001: 0; pre (0, 0); post linear,
    1049: 8; post linear,
}
```

Two keys on one channel, and the ball rolls eight units without slipping.
Move them, retime them, add an ease — the roll follows, because it is not
a channel.

## Playback

![The finished rig evaluated live by usdview's Hydra Storm renderer](../gifs/concepts/ball_final.gif)

This preview uses usdview's Storm renderer and the live rigExec imaging plugin
at **1280 × 800, 30 fps**, with 8× antialiasing. It plays the authored travel
forward and backward at inspection speed, with a following camera and hidden
rig guides. The material and mesh come directly from the downloadable USD.

Drag usdview's frame slider from 1001 to 1049, or press **Play**. The rig
evaluates live: no bake, no export. The star and the band are what tell
you it is rolling — watch the contact point stay put under the ball as it
crosses the ground.

## What no editor can author

Everything above happens with the pointer except five values, and the
reason is worth knowing: **usdNoodles has no attribute-value editor**. It
creates prims, renames them, applies API schemas, draws relationships and
connections, and moves nodes — but nothing in it sets a value. usdview's
own property panel is read-only, and the Layer Opinions panel
(**RigExec ▸ General Editors ▸ Layer Opinions**) edits and deletes
opinions that already exist rather than creating them.

The Avar Editor covers the `avars` and the viewport manipulators cover
poses and rest offsets, which is most of a rig. What is left here is
three `uniform` properties — and `uniform` is exactly the kind no
animation UI touches — plus the two `rest:space` matrices from Steps 5
and 6, because a `matrix4d` is not a number a spin box takes:

```usda
# /BallAsset/Rig/Solvers/BallChain
uniform token rigExec:controlSpace = "parentRelative"

# /BallAsset/Rig/Weights/BallWeight
uniform float rigExec:defaultWeight = 1

# /BallAsset/Rig/Movers/RollFromTravelX
uniform token rigExec:operation = "multiply"

# /BallAsset/Rig/Controls/Move/Squash/Roll and
# /BallAsset/Rig/Joints/Root/Squash/Roll
matrix4d rest:space = ( (1,0,0,0), (0,1,0,0), (0,0,1,0), (0,1,0,1) )
```

usdview does have somewhere to type them, and it is not a text editor:
**Window ▸ Interpreter**. The recordings for Steps 7, 8 and 9 each open
it, type three lines and close it again, so you can see exactly where the
value goes:

```python
from pxr import Sdf
p = usdviewApi.stage.GetPrimAtPath('/BallAsset/Rig/Solvers/BallChain')
p.CreateAttribute('rigExec:controlSpace', Sdf.ValueTypeNames.Token, False, Sdf.VariabilityUniform).Set('parentRelative')
```

The others are the same three lines with a different path, name, type and
value — `Sdf.ValueTypeNames.Float` and `1.0` for the weight's envelope,
`Sdf.ValueTypeNames.Token` and `'multiply'` for the mover's operation,
and `Sdf.ValueTypeNames.Matrix4d` with
`Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, 1, 0))` (and
`Sdf.VariabilityVarying` rather than uniform) for the two `rest:space`
frames. Editing the `.usda` and reloading (**File ▸ Reload All Layers**)
does the same job, and `docs/examples/tutorial_rolling_ball.usda` already
has every one of them.

## Free rolling in the Godot game

The tutorial asset now exposes **`rollMode`** on `/BallAsset`:

* **`travelX`** (default) preserves the recorded lesson: the variant supplies
  the `-57.29578` gain and the float math mover drives roll from X travel.
* **`free`** disables `RollFromTravelX` and supplies a neutral `rz = 0`.
  Both Roll and Spin expose independent `rx`, `ry`, and `rz` controls.
  The centre pivot and `Move → Squash → Roll → Spin` hierarchy stay intact.

Open [`tutorial_rolling_ball_free.usda`](../examples/tutorial_rolling_ball_free.usda)
for the game-ready variant. It references the same tutorial rig, uses one
unit as one metre (radius one), and is baked at frame 1001. Changing the
variant in usdview also lets you pose all three rotation channels manually.
Selecting `free` does not automatically integrate travel inside USD; its
rotation channels are driven by the application or authored animation.

The sibling `godot_rigExec/demo/project.godot` now opens **Roll / Collect**,
a small course with six rings, obstacles, a ramp, hopping, a finish pad and
fall recovery. Use WASD / arrows to move, Space to hop, Shift to brake and
R to restart. Run `python demo/setup_rolling.py --build` from that plugin
checkout to rebuild the native plugin and bake this asset, then run
`godot --path demo`.

The game stores orientation as a normalized quaternion. On contact with a
surface, project the **actual collision-constrained displacement** into its
tangent plane, then accumulate a world-space increment:

```text
tangent = displacement - normal * dot(normal, displacement)
axis = normalize(cross(normal, tangent))
angle = length(tangent) / radius
orientation = normalize(quaternion(axis, angle) * orientation)
```

Skip the increment for zero displacement. This handles diagonal travel,
reversals and changing surface normals. It retains path history: a square
lap can return the ball to its starting position with a different orientation.
In the air the game preserves angular velocity, and resumes contact rolling
on landing. The no-slip calculation assumes a spherical ball of fixed radius;
squash is left at one in the game.

Only at the rig boundary is the quaternion decomposed to degree avars.
USD's row-vector XYZ composition corresponds to Godot's `EULER_ORDER_ZYX`.
`RigExecPlayer.set_avar()` drives Move translation and Roll rotation, then
`evaluate()` executes the compiled FK program and `apply_to_skeleton()`
updates the skeleton. `rolling_ball.tscn` is a reusable Godot object containing
the collision body, skeleton, mesh, material and `rolling_ball.gd` controller.
The level instantiates it and manages the course. Other controllers can feed
`drive(direction, delta, braking)` instead of keyboard input.

Its mesh is exported from `/BallAsset/Geom/Ball` with the original face-varying
UVs, refined by OpenSubdiv and attached to Spin. The exported shader preserves
the bound USD material's texture, diffuse and emission scales, roughness,
metallic and specular values. The export corrects the source's inward winding
for Godot culling while preserving corner UVs. No generic Godot sphere or
substitute UV mapping is used. The visible mesh has no separately animated
rotation. Godot lighting and tone mapping can differ from usdview.

The plugin distinguishes skinning deltas from bone poses: the existing
`get_joint_transforms()` returns rest-to-posed deltas, while
`get_joint_pose_transforms()` returns posed frames and
`get_joint_rest_transforms()` supplies skeleton rests. Applying a delta as
a bone pose would make this ball orbit instead of turning about its centre.

Run `godot --headless --path demo --script verify_rolling.gd` in the plugin
checkout to verify multi-axis FK, no-slip signs, blocked movement, resets
and the game loop against the native extension.

## Where to go next

* [Godot and baked rigs](../concepts/tutorial-godot-baked-rig.md) — package this
  ball as a reusable game object and drive its controls in Godot, with gameplay GIFs.
* [Baked and dynamic evaluation](baked-vs-dynamic.md) — the same rig, two
  ways to compute it.
* [Two-Bone IK](../nodes/two_bone_ik.md) — the next solver up, and the one
  that makes a limb out of the chain you just built.
* [Lattice Mover](../nodes/lattice_mover.md) — squash and stretch as a
  deformation rather than a scale, for when scaling the whole ball is too
  blunt.
