"""Hand-written prose for the UsdRig node pages (see build_pages.py).

Each entry supplies the user-facing description, wiring table, example
story, tips, and cross-links for one operator. Parameter reference comes
straight from libs/rigExecSchema/schema.usda at generation time.

Two optional keys steer the media: "example_key" points a page at another
operator's stage in docs/examples/ instead of its own, and "no_gif" marks
a page that documents an annotation rather than a motion, so no GIF is
rendered for it (such a page still links a stage if it sets "example_key").
"""

CATEGORIES = [
    ("Rig", ["rig_root"]),
    ("Transform providers", ["control", "joint"]),
    ("Solvers", ["fk_chain", "two_bone_ik", "spline_ik",
                 "blend_point_frames", "twist_distribution", "ribbon"]),
    ("Constraints", ["aim_constraint", "position_constraint",
                     "rotation_constraint", "scale_constraint",
                     "parent_constraint", "single_chain_ik_constraint"]),
    ("Geometry movers", ["matrix_mover", "skin_mover", "blendshape_mover",
                         "curve_mover", "lattice_mover", "surface_mover",
                         "smooth_mover", "volume_correct_mover"]),
    ("Curvenet", ["curvenet", "curvenet_adjustment",
                  "curvenet_adjuster_mover", "curvenet_mover"]),
    ("Blend channels", ["blend_input", "blend_sample"]),
    ("Pose space", ["pose_interpolator", "pose"]),
    ("Weights", ["static_weight", "dynamic_weight", "sphere_weight",
                 "plane_weight", "curve_weight", "combine_weight",
                 "curvenet_weight"]),
    ("Property math", ["float_math_mover", "vec3f_math_mover",
                       "matrix_math_mover"]),
    ("Interface", ["picker", "picker_panel", "picker_button",
                   "touch_regions", "touch_region"]),
]

OPERATORS = {
    "rig_root": {
        "title": "Rig Root",
        "schema": "RigExecRoot",
        "no_gif": True,
        "example_key": "two_bone_ik",
        "summary": "The prim that makes a namespace a rig: partition, discovery root, evaluation unit.",
        "description": """Every rig is one `RigExecRoot` prim and everything composed
beneath it. The root declares no membership lists: a `RigExecControl` under it
is a control, a `RigExecJoint` is a joint output, a placed volume prim is a
weight field, and a prim carrying `rigExec:moves` under the root's `Movers`
child is a mover. It is what a host activates — one rig, or every
`RigExecRoot` on the stage — and it is the unit that compiles, publishes a
generation, and carries the pose diagnostics. It is Imageable but deliberately
not Xformable, so guide bounds propagate up to the enclosing asset for camera
framing while the rig itself never adds a transform.""",
        "how_it_works": """Nothing on the root evaluates per frame; it is read during the
evaluator's compile phase, in the discover-and-validate pass. Compile walks the
composed namespace beneath the root and collects controls, joints, pose
interpolators, placed volume weights, and aggregate solvers by prim type
anywhere under it, then walks the WHOLE RIG in reverse-sibling post-order —
descendants before their parent, and the *bottom* sibling branch in usdview
first — to number ONE pose stack of joint-writing solvers and frame
constraints, of which the `<rig>/Movers` mover stack is a restriction; the walk uses the standard
`UsdPrimRange` predicate, so a deactivated or unloaded branch is simply not
part of the rig and changing that is a structural (epoch-rebuilding) edit
rather than a value edit. A rig that finds no controls, joints, volume weights,
and no movers at all is a compile error ("Rig publishes no outputs"), and every
mover target is checked against the root's *parent* prim, which is the rig
asset and the boundary of what the rig may write. `uniform bool rigExec:baked`
is re-read at the tail of each compile and only asks for the baked program: an
explicit `SetEvaluationMode` call or a non-empty `RIGEXEC_EVALUATION_MODE`
outranks it, an epoch the program cannot express falls back to the dynamic path
with a note on the published pose, and both paths publish the same values.""",
        "wiring": [
            # Type-based discovery over the whole subtree: rigEvaluator.cpp:1014
            # (_DiscoverControls), :939 (_DiscoverJointOutputs), :986 (pose
            # interpolators), :1037 (volume weights), :1071 (aggregate solvers).
            # Each is a plain UsdPrimRange(rig) type-name walk; no manifest and no
            # relationship on the root itself.
            ("(namespace)", "Everything composed beneath the root is the rig; controls, "
             "joints, solvers, and weight volumes are discovered by prim type.", "-"),
            # The one child NAME the compiler looks up: rigEvaluator.cpp:3540 (and
            # the matching digest walk at :2559). Absent Movers scope is legal --
            # a rig of controls and joints alone compiles (:4436 only fails when
            # nothing at all was found).
            ("`Movers` child", "The scope the mover stack is walked from: only prims under "
             "`<rig>/Movers` are compiled as movers.", "no"),
            # assetRoot = _rigPath.GetParentPath() (rigEvaluator.cpp:3596) and the
            # HasPrefix check that rejects anything outside it (:3609).
            ("(parent prim)", "The rig root's parent is the asset: movers may only target "
             "prims under it, and geometry lives there too.", "-"),
        ],
        "param_groups": [],
        "example": """Every shipped example is one of these: `two_bone_ik.usda` puts a
`RigExecRoot` named `Rig` inside the `IkAsset` Xform, with `Controls`,
`Solvers`, `Joints`, `Weights`, and `Movers` scopes beneath it and the deformed
cards in a sibling `Geom` scope. Only `Movers` is a name the compiler knows —
the rest are ordinary `Scope` prims kept for readability — and the geometry sits
under `IkAsset` because that parent is what bounds the rig's write set.""",
        "tips": [
            "Keep the deformed geometry inside the same asset prim as the rig: a "
            "mover whose target is outside the rig root's parent fails compile "
            "with \"targets outside the rig asset\".",
            "Order two movers that write the same target — or a solver against a "
            "constraint, or two solvers against each other, which are all steps "
            "of ONE pose stack — by arranging them in namespace: nesting, or "
            "`reorder nameChildren` on their parent. The bottom composed sibling "
            "executes first, the compiler reads the final composed order and "
            "nothing about how it arose, and nothing else breaks a tie. Put "
            "`Solvers` at the bottom of the rig root for the classic \"solve, "
            "then revise\" shape.",
            "`rigExec:baked` has to be *authored* to be heard (the check is "
            "`HasAuthoredValue`), it is only a request, and it is the weakest of "
            "the three ways the mode is chosen.",
        ],
        "see_also": [
            ("control", "Control"),
            ("joint", "Joint"),
            ("matrix_mover", "Matrix Mover"),
        ],
    },
    "control": {
        "title": "Control",
        "schema": "RigExecControl",
        "summary": "The animator's handle: animation is authored on its avars.",
        "description": """A control is the rig's interaction surface. An animator keys its
translation, rotation, scale, and spin avars, and solvers and movers read
the resulting posed frame. Controls draw a viewport guide at the posed
origin — `sphere`, `circle`, `box`, `cube`, `diamond` or `pyramid`, drawn
as curves (`wire`) or as solids (`geometry`) — so the handle can be picked
directly; unlike joints and solvers, a control is not a diagnostic, so it
keeps the default render purpose.""",
        "how_it_works": """A control is evaluated in the base phase only — it is an input, so
nothing in the pose domain revises it and its base frame IS its posed
frame. The frame comes from `rest:space` plus the avars each frame:
translations and Euler rotations compose over the rest offset inside the
selected parent and default spaces (`avars * posed:defaultSpace *
inverse(parent:defaultSpace) * parent:space`), and scale avars apply
before rotation and translation. Anything downstream — an FK chain
listing the control, a constraint naming it as a source, a matrix mover
reading its frame — follows the animated result with no further
wiring.""",
        "wiring": [
            # A control declares no input relationships of its own (it adds only
            # avars:s* and guide:* to RigExecXformable, schema.usda:144-260);
            # consumers name it. FK chain: libs/rigExec/computations.cpp:704-707
            # registers rigExec:controls with .Required(). Matrix mover frames:
            # libs/rigExec/rigEvaluator.cpp:7203 rejects a matrix mover whose
            # rigExec:transform does not have exactly one target, and the kernel
            # reads the compiled rigExec:resolvedTransform
            # (libs/rigExec/moverKernels.cpp:1108).
            ("(none)", "Controls are read by solvers, constraints, and movers; "
             "they take no input relationships.", "-"),
        ],
        "param_groups": [
            ("Control channel", "RigExecControlAPI"),
            ("Transform provider", "RigExecXformable"),
        ],
        "example": """Twelve controls in a 6x2 grid draw every `guide:shape` in every
`guide:drawMode`. Left to right in both rows: sphere, circle, box, cube,
diamond, pyramid — `box` is the planar square lying flat in XZ, `cube` the
solid six-sided one. The top row is `wire` (the guide is drawn as curves),
the bottom row `geometry` (the same shape drawn as a solid). All twelve
carry the same animation — `avars:ry` 0 → 90°, `avars:rz` 0 → 22° and
`avars:ty` 0 → 0.25 over 1001-1024 — and all twelve do real work: each
feeds an FK chain that poses a joint resting at the control, and a matrix
mover carries the grey card a unit below it. Watch either row against the
blue rest outline the cards leave behind: `ry` spins the card about the
handle's own origin, `rz` tips it out of the ground plane, and `ty` lifts
it clear of the outline. The rotation is what the round shapes could not
show on their own — a sphere or a circle turned about its symmetry axis
looks identical — so the card is where the frame's motion becomes
visible.""",
        "tips": [
            "Keep `rest:space` at the handle's bind pose and animate only the "
            "avars; rest edits re-proportion every downstream solve.",
            # No guide:* token is read anywhere in libs/rigExec (the evaluator);
            # only the imaging plugin consumes them, and an unrecognized pair
            # resolves to no guide at all in _FindControlGuideShape
            # (libs/rigExecImaging/sceneIndices.cpp:968).
            "`guide:shape` and `guide:drawMode` change the pickable viewport "
            "guide without touching evaluation; a pair the build does not draw "
            "synthesizes no guide at all rather than falling back to one.",
            # guide:wireWidth is only consulted on the basisCurves path
            # (libs/rigExecImaging/sceneIndices.cpp:520), and it is authored in
            # the unit shape's pre-scale units, so guide:scaleX/Y/Z multiply it.
            "`guide:wireWidth` is in the unit shape's local units — the drawn "
            "width is scaled by `guide:scaleX/Y/Z` along with the shape — and "
            "`geometry` draw mode ignores it entirely.",
        ],
        "see_also": [
            ("joint", "Joint"),
            ("fk_chain", "FK Chain"),
            ("matrix_mover", "Matrix Mover"),
        ],
    },
    "joint": {
        "title": "Joint",
        "schema": "RigExecJoint",
        "summary": "A posed output of the rig: solvers write it, movers read it.",
        "description": """A joint is where solved posing becomes readable data.
`rigExec:joints` is an ordered write and not an exclusive claim: any number of
aggregate solvers may name one joint, and any number of pose constraints may
name it on `rigExec:moves`. All of them are steps of one kind in ONE
hierarchical stack, ordered by the composed namespace of the whole rig — the
bottom sibling first — and by nothing else. A constraint that sits BELOW a
solver runs before it and feeds it: the frame it leaves becomes that solver's
rest reference. A constraint ABOVE it revises the solver's output, which is the
classic shape and the one you get by putting `Solvers` at the bottom of the
rig. Joints nest in the namespace to form the hierarchy, and
usdview draws each joint as a guide sphere with a cone to every nested
child.""",
        "how_it_works": """The compiler builds one chain per joint out of every step that
writes it — the solvers that name it and the constraints that move it — in the
rig's hierarchical order, and the pose phase runs that chain. A solver
extracts the joint's element from its frame array and REPLACES whatever stood
there, measuring the joint from the frame the preceding steps left; a
constraint reads that same incoming frame and writes a revised one over it. A
joint no solver names is still evaluated — it follows its namespace parent's
posed space with its own rest offset and avars — and a joint with no step
before a solver hands that solver its authored `rest:space` rest, which is why
a rig whose constraints all sit above its solvers behaves exactly as it always
did. A mover reads the `base` frame — the joint after the LAST SOLVER in the
chain — unless it asks for `final`, which is the top of the chain, or names a
prim, which is the joint as of when the walk finished with it.""",
        "wiring": [
            # rigEvaluator.cpp records every solver that names a joint and
            # serializes them into one ordered stack per joint (pose-graph
            # dependency order first, the solver stack ordinal breaking ties); a
            # joint no solver names at all is legal (every joint is tapped for a
            # frame regardless, rigEvaluator.cpp:5513), which is why this is "-".
            ("(posed by)", "Any number of solvers' `rigExec:joints` name this joint; "
             "the writes stack in hierarchical order and the last one supplies "
             "its base frame.", "-"),
            # rigEvaluator.cpp:5478 pushes each constraint onto a per-target LIST,
            # so the count is unbounded; in examples/biped/Biped.usda 27 of the
            # 252 joints carry more than one (22 with two, 5 with three).
            # Ordering is the mover stack (constraints are collected in
            # _GetMoverExecutionOrder order, rigEvaluator.cpp:149-163 / 3543 /
            # 5290) and sequenced at rigEvaluator.cpp:6156-6158.
            ("(revised by)", "Any number of pose constraints name this joint on "
             "`rigExec:moves`. They occupy the SAME hierarchical stack as the "
             "solvers: one above a solver revises its output, one below feeds "
             "it.", "-"),
        ],
        "param_groups": [
            ("Transform provider", "RigExecXformable"),
        ],
        "example": """A three-joint arm whose wrist is written twice. The FK chain poses
shoulder, elbow, and wrist from three controls as the elbow control curls 70
degrees, then an aim constraint re-aims the wrist at a fixed anchor above and
beyond the hand in the pose phase. Each panel is skinned rigidly to its own
joint (Upper to Shoulder, Fore to Elbow, Hand to Wrist) and reads the `final`
phase, so the long hand panel keeps pointing at the anchor while the forearm
swings out from under it. In the GIF the pale spheres at the shoulder, elbow
and wrist are the joints themselves and the tapered wire between each pair is
the bone that the namespace nesting creates; the dashed line from the wrist to
`Anchor` is the pose-phase aim revision, not a control link. `ShoulderCtl` is
deliberately left unanimated, so the Upper panel sits on its own rest ghost
for the whole loop and the only motion in frame comes from the one driver.""",
        "tips": [
            # Hierarchy is namespace nesting and the guide cones follow it:
            # schema.usda RigExecJoint doc (libs/rigExecSchema/schema.usda:369-379).
            # rest:space is "relative to the namespace frame provider's rest
            # frame" (libs/rigExecSchema/schema.usda:307-308).
            "Nest joints (Elbow inside Shoulder) so hierarchy, guides, and FK "
            "composition all agree — and remember `rest:space` is measured from "
            "the parent joint's rest, not from the world.",
            # The solver stack: the writer order is settled against the finished
            # pose graph. Unlimited constraints per joint: rigEvaluator.cpp:5478,
            # ordered by rigEvaluator.cpp:6157.
            "Several solvers may pose one joint, and pose constraints are steps "
            "of the same kind in the same stack: the order is the composed "
            "namespace of the whole rig and nothing else, bottom sibling first. "
            "Put `Solvers` at the bottom of the rig to get the classic \"solve, "
            "then revise\" shape; a constraint that ends up BELOW a solver feeds "
            "it instead — every solver kind composes over the frame the steps "
            "below it left, so nothing is discarded either way.",
            # The consumed-solver relaxation: rigEvaluator.cpp:4915-4923, which is
            # how examples/biped/Biped.usda lets leg_l_twoBoneIk, leg_l_fkChain
            # and leg_l_ikfk all list ankle_l_bind.
            "A solver whose output another solver reads is naming joints for "
            "their rests, not claiming them, so an IK and an FK chain can both "
            "list the chain that their IK/FK blend actually poses.",
        ],
        "see_also": [
            ("fk_chain", "FK Chain"),
            ("aim_constraint", "Aim Constraint"),
            ("matrix_mover", "Matrix Mover"),
        ],
    },
    "fk_chain": {
        "title": "FK Chain",
        "schema": "RigExecFkChain",
        "summary": "Composes per-control local animation down a joint hierarchy.",
        "description": """Forward kinematics in its simplest form: one control per joint, each
authoring a local rotation (or translation), composed down the chain so
every joint inherits its ancestors' motion. Use it for tails, spines,
fingers — anywhere the animator wants direct control of every link. The
controls are normally nested one under the next so each handle rides the
one above it, which is what `rigExec:controlSpace = "parentRelative"`
declares.""",
        "how_it_works": """The solver runs in the pose phase: it reads each targeted control's
`computePointFrame` and `computeRestFrame`, forms that control's
rest-to-pose delta, and publishes one aggregate `computePointFrameArray`
whose element N is written to `rigExec:joints`[N] — the two lists are
parallel, so control N poses joint N and joint N inherits the composed
frames above it. `rigExec:controlSpace` decides how the deltas compose:
`world` chains them (W_i = W_(i-1) . A_i) for sibling controls, while
`parentRelative` takes each delta as-is because a nested control's frame
already travels with its parent. The chain MEASURES its deltas from its
*controls'* rests, but the basis it composes them onto is the joint's rest
reference — which the pose stack replaces with the frame the steps below the
chain left (spec section 4.2). So a constraint below the chain moves the
joint and the chain carries that displacement through its solve instead of
replacing it; a joint no step below it wrote keeps its authored rest and the
chain answers exactly as it always did. Rest offsets between joints set the bone
lengths and pivots — nothing is measured in absolute numbers — but the
solve itself is absolute: unless `rigExec:startFrame` names the provider
the chain hangs from, the chain ignores whatever its joints sit under.
Skinning movers then read the posed joints at the `final` phase.""",
        "wiring": [
            # Both control inputs are .Required() on the computation
            # (libs/rigExec/computations.cpp:704-711), and the compiler enforces
            # it from the other end: a bound joint's element index is checked
            # against the control count (libs/rigExec/rigEvaluator.cpp:4773-4779
            # takes knownCount = rigExec:controls size; 4853-4858 rejects an
            # element >= it). Measured: deleting this relationship from the
            # stage fails compile with "element 0 for .../Seg1 is out of range
            # (solver produces 0 frames)".
            ("`rigExec:controls`", "Ordered animator controls, one per joint.", "yes"),
            # NOT enforced: an FkChain with no joints compiles and evaluates
            # (empty targets are skipped at libs/rigExec/rigEvaluator.cpp:4726-
            # 4728), but nothing then extracts its elements and no joint moves
            # (measured on this stage with the relationship deleted: Seg2 stays
            # at its rest x = 2 at frame 1006 instead of x = 1.88). The ordered
            # list IS the solver->joint binding:
            # libs/rigExec/rigEvaluator.cpp:2114-2118, schema.usda:509-515.
            ("`rigExec:joints`", "Ordered nested joints the chain poses; with "
             "none, the solver publishes frames nothing reads.", "no"),
            # Plain AttributeValue input, no .Required(), default "world":
            # libs/rigExec/computations.cpp:720 and schema.usda:454. It must be
            # "parentRelative" whenever the controls are nested, or the parent's
            # motion is composed in twice (libs/rigExec/computations.cpp:680).
            ("`rigExec:controlSpace`", "`parentRelative` when the controls are "
             "nested under each other; `world` for sibling controls.", "no"),
        ],
        "param_groups": [],
        # The strip is 10 quads at 0.5 units (x = 0 .. 5, 22 points) and the
        # three RigExecStaticWeight fields are linear ramps, not step columns:
        #   x:     0     0.5   1     1.5   2     2.5   3     3.5   4    4.5  5
        #   Seg1:  1     1     1     0.75  0.5   0.25  0     0     0    0    0
        #   Seg2:  0     0     0     0.25  0.5   0.5   0.5   0.25  0    0    0
        #   Seg3:  0     0     0     0     0     0.25  0.5   0.75  1    1    1
        # Every column sums to 1; the joint columns (x = 2, x = 4) are 0.5/0.5
        # between the two joints that meet there and the midline between Seg2
        # and Seg3 (x = 3) is 0.5/0.5 as well. That is what makes both bends
        # draw as arcs of the same kind in the GIF instead of one hard crease
        # at Seg1-Seg2 and one two-facet blend at Seg2-Seg3.
        "example": """Three controls curl a ten-quad strip. C2 is a namespace child of C1
and C3 of C2, each with the same local rest offset as its joint (2 units),
so the rings sit on Seg1/Seg2/Seg3 at x = 0, 2, 4 and each one rides the
one above it — the chain reads them with `rigExec:controlSpace =
"parentRelative"`. One matrix mover per joint skins the strip through a
linear weight ramp, so each joint hands its influence to the next across a
two-unit span: the column on Seg2 is split 0.5 with Seg1, the midline
between Seg2 and Seg3 is split 0.5 to each, and the column on Seg3 is split
0.5 with Seg2. Both bends therefore draw as arcs rather than creases, and
the same handover is happening at every joint — the strip ends at x = 5 so
no part of it is a rigid slab hanging off the last one.""",
        "tips": [
            "Nesting the controls only works with `rigExec:controlSpace = "
            "\"parentRelative\"`: a nested control's frame already carries its "
            "parent's motion, so the default `world` composes it in a second "
            "time. On this stage, dropping the token sends Seg3 to x = -0.45 at "
            "frame 1006 instead of x = 3.03.",
            "Point `rigExec:startFrame` at the joint the chain hangs from when "
            "that joint is posed by another solver; without it the chain is an "
            "absolute solve and the limb detaches from its parent.",
            "FK pairs well with IK: blend the two aggregates through a Blend "
            "Point Frames node, or stack both solvers on the same joints — "
            "`rigExec:joints` is an ordered write, so the later writer simply "
            "replaces the frames the earlier one committed.",
        ],
        "see_also": [
            ("two_bone_ik", "Two-Bone IK"),
            ("blend_point_frames", "Blend Point Frames"),
            ("joint", "Joint"),
        ],
    },
    "two_bone_ik": {
        "title": "Two-Bone IK",
        "schema": "RigExecTwoBoneIk",
        "summary": "Aims a two-segment limb at an effector with pole-vector control.",
        "description": """Analytic two-bone inverse kinematics for arms and legs. The root
stays planted on its control, the end joint reaches for the effector
control, and the pole control picks which way the middle joint bends.
Bone lengths are measured from the bound joints' rests on every
evaluation — there is nothing absolute to author or keep in sync.""",
        "how_it_works": """Each evaluation measures root-to-mid and mid-to-end from the
frames the joints carry on entry to this solver — their `rest:space` rests
unless a step below it in the pose stack already wrote them — plus the length
offsets, then solves the two-bone chain in the plane through the pole. `inputs:stretch` lets the chain elongate toward
out-of-reach goals under `rigExec:stretchPolicy`, and
`rigExec:unreachablePolicy` with `inputs:softness` shapes the lock-up as
the goal leaves reach.""",
        "wiring": [
            ("`rigExec:effectorControl`", "Control supplying the end-goal position.", "yes"),
            ("`rigExec:poleControl`", "Control defining the bend plane.", "yes"),
            ("`rigExec:rootControl`", "Control planting the chain root.", "yes"),
            ("`rigExec:joints`", "Three nested joints: root, mid, end — the chain "
             "this solver measures and writes. It is an ordered write, not an "
             "exclusive claim: another step may write the same joints, and the "
             "last writer in the stack supplies their base frame. The solver "
             "measures the two bones from the frames the joints carry ON ENTRY, "
             "so a step BELOW it that moves one of them re-proportions the limb "
             "rather than only re-orienting it.", "yes"),
        ],
        "example": """A two-card arm bends as its hand effector swings in and lifts. The
effector rests just inside full reach so the arm holds a slight natural
bend; the pole above the elbow keeps the bend plane facing the camera.""",
        "tips": [
            "Keep the pole on the side you want the joint to favor; the "
            "solver builds a right-handed bend basis, so pole-above bends "
            "keep +Z-facing cards front-facing.",
            "Rest the effector just inside full reach: exactly straight is a "
            "singularity where the bend plane is undefined.",
            "Move a joint rest and the limb re-proportions itself — IK never "
            "caches bone lengths.",
        ],
        "see_also": [
            ("fk_chain", "FK Chain"),
            ("blend_point_frames", "Blend Point Frames"),
            ("aim_constraint", "Aim Constraint"),
        ],
    },
    "spline_ik": {
        "title": "Spline IK",
        "schema": "RigExecSplineIk",
        "summary": "Lays a joint chain along a curve built from three controls.",
        "description": """The spine and neck solver: three controls shape an open degree-2
four-CV B-spline, and the ordered `rigExec:joints` chain is laid along it
by arc length. The curve is built from the posed control frames rather
than read from the stage, so unlike a Ribbon it needs no native driver
curve and a control-driven pose reaches it directly. Each joint aims +X
at its successor, a twist linear in arc position is added, and
`rigExec:volumeWeights` thins the off-axis handles as the curve
stretches.""",
        "how_it_works": """Everything happens in the pose phase. Each evaluation reads the posed
and rest frames of the three controls and the rest frames of every joint
on `rigExec:joints`, rebuilds the rest curve from the rest origins of
joints [0], [1], [N-2], [N-1], then carries cv0/cv1 by the root control's
rest→pose map and cv2/cv3 by the end control's, adding the mid control's
translation offset from its follow point to cv1 and cv2. Joint *i* is
placed at arc distance `ratio * (rest spacing before i)` with `ratio =
posed arc length / rest arc length` (`rigExec:restLength` picks whether
that reference length is the rest curve or the rest chain), gets
`roll + twist * t_i` about its aim axis, and carries scale `s_y = s_z =
1 - w_i * preserveVolume * (ratio - 1)` on its Y/Z handles. The result is
published as `computePointFrameArray`, one frame per `rigExec:joints`
entry, which the bound joints extract view-free.""",
        "wiring": [
            # computations.cpp:1195-1198 -- computePointFrame input .Required()
            ("`rigExec:rootControl`", "Provider carrying cv0 and cv1 and the root roll.", "yes"),
            # computations.cpp:1202-1205 -- .Required(); translation only, schema.usda:843-846
            ("`rigExec:midControl`", "Provider whose translation offset bends cv1 and cv2.", "yes"),
            # computations.cpp:1209-1212 -- .Required()
            ("`rigExec:endControl`", "Provider carrying cv2 and cv3 and the end twist.", "yes"),
            # computations.cpp:1073 -- an empty list warns and produces no frames
            # (compile simply skips a solver with no joints, rigEvaluator.cpp:4726)
            ("`rigExec:joints`", "Ordered chain, root to tip: the solve's cardinality "
             "and its rest CVs. Naming a joint another step also writes stacks the "
             "two, and the last writer in that stack supplies the joint's base "
             "frame. The rest CVs and rest spacing come from the frames the joints "
             "carry ON ENTRY, so a step below this one that moves a joint changes "
             "the rest curve it solves against.", "yes"),
        ],
        # RigExecSplineIk inherits Boundable directly (schema.usda:797-798): no
        # RigExecXformable, RigExecConstraint or RigExecMoverAPI attributes apply,
        # and its guide:* knobs are declared on the class itself.
        "param_groups": [],
        "example": """A six-joint chain rests straight along a 6-unit tube with controls at
its root, middle and tip. The mid control lifts 2.4 units and comes back:
the curve arches, the chain stretches from 1.2 to 1.6 units per segment
to keep pace with the longer arc, and the volume weights pinch the tube
by about 26 percent at the crown against the straight rest cage beneath
it. Nothing but that one control is keyed.""",
        "tips": [
            "The mid control contributes translation only — its rotation and "
            "scale are ignored. It offsets from a follow point that "
            "`inputs:midFollowWeight` blends between its rest origin carried by "
            "the root control (0) and by the end control (1), so an unkeyed mid "
            "still rides between the two ends.",
            "`rigExec:restLength` = `curve` makes the stretch ratio exactly 1 at "
            "rest; `chain` makes the chain span the curve exactly. The two only "
            "differ when the rest chain is not itself the rest curve — a "
            "straight rest chain (like the docs example) gives identical "
            "results, a curved rest spine does not.",
            "`inputs:minLengthRatio` floors the chain: below that fraction of "
            "the rest root→end chord the end CVs are held that far ahead of cv0 "
            "along the root's posed chain axis, so an end control driven down "
            "onto its root cannot fold the curve back through it. The biped "
            "spine and neck both run at 0.5; the default 0 switches it off.",
        ],
        "see_also": [
            ("ribbon", "Ribbon"),
            ("two_bone_ik", "Two-Bone IK"),
            ("twist_distribution", "Twist Distribution"),
        ],
    },
    "blend_point_frames": {
        "title": "Blend Point Frames",
        "schema": "RigExecBlendPointFrames",
        "summary": "Blends two solver poses per joint under one weight.",
        "description": """The IK/FK switch and everything like it: two solvers describe the same
skeleton, and the blend mixes their aggregates element by element under
`inputs:weight`. At 0 the A pose wins, at 1 the B pose wins, and between
them rotations take the shortest arc while scales blend logarithmically.
The two inputs only have to agree on element count — what each of them
poses, its own joints or nothing at all, is its own business. Blending is how
to *mix* two solvers on one skeleton; stacking them — naming the same joints
on both `rigExec:joints` lists — is how to have the later writer replace the
earlier one's frames outright.""",
        "how_it_works": """The blend runs in the pose phase, in a batch after both inputs (an
`inputA`/`inputB` solver is a dependency, so the compiler's levels put it
in an earlier batch), and writes one blended frame per joint listed in
`rigExec:joints`; matrix movers then read those joints normally at the
`final` phase. Each element is blended as a rest-to-pose transform measured
against the A aggregate's rest landmarks — translation lerps, rotation
slerps shortest-arc, scale follows `rigExec:scaleBlend` — so a mid-weight
pose is the interpolation of the two transforms, not the midpoint of the
two skeletons' joints, and an intermediate chain can sit a little off the
average of the poses it is between. The blend still MEASURES both inputs
against the rests carried inside the A aggregate, but it APPLIES the blended
map to the joint's own rest reference — the frame a pose step below the blend
left there (spec section 4.2) — so a constrained joint carries its
displacement through the blend rather than losing it.""",
        "wiring": [
            # Both are declared .Required() in the exec registration
            # (libs/rigExec/computations.cpp:950-959), but nothing enforces
            # either one: a stage with inputB deleted, and one with BOTH
            # deleted, compiles and evaluates (verify_examples.py ok in both
            # dynamic and baked modes). A missing target is the computation's
            # null pointer, and a null pointer returns the OTHER input by
            # value (libs/rigExec/computations.cpp:876-881,
            # libs/rigExec/bakedPose.cpp:2514-2523); with neither wired the
            # blend publishes nothing and its joints fall back to their rest
            # chain (libs/rigExec/bakedPose.cpp:2589-2591, 3340-3352).
            ("`rigExec:inputA`", "Aggregate solver selected at weight 0; "
             "unwired, inputB's pose passes through at every weight.", "no"),
            ("`rigExec:inputB`", "Aggregate solver selected at weight 1; "
             "unwired, inputA's pose passes through at every weight.", "no"),
            # Not enforced: a guide-only blend with no joints is a supported
            # shape (libs/rigExec/bakedProgram.cpp:1943-1944) and compiles --
            # the stage with this relationship deleted still verifies. List
            # position is the element index
            # (libs/rigExecSchema/schema.usda:640-644).
            ("`rigExec:joints`", "Ordered joints the blend poses, one per "
             "aggregate element; omit only for a guide-only blend another "
             "solver reads.", "no"),
        ],
        # RigExecBlendPointFrames inherits Boundable directly
        # (libs/rigExecSchema/schema.usda:608-609), so there is no RigExec base
        # class carrying parameters: every knob is on the node itself.
        "param_groups": [],
        "example": """Two FK chains hold very different poses of one three-joint arm, four
units apart so neither hides the other. `FkUp` — the `UpCtl` rings in the
near lane, posing `PoseUp1/2/3` — is input **A**, a tight upward curl;
`FkDown` — the `DownCtl` rings in the far lane, posing `PoseDown1/2/3` —
is input **B**, a long low hook. `PoseBlend` writes the mix into
`Arm1/Arm2/Arm3`, and three matrix movers skin a 21×3 strip to those
three joints, so the strip is the only thing in frame that shows what the
blend produced. The weight sweeps 0 → 1 and back: at 0 the strip lies on
the A chain, at 1 on the B chain, and every value between is the
interpolation of the two *transforms* — a shape neither input has, and
not the average of the two skeletons' joint positions.""",
        "tips": [
            "The two inputs must publish the same number of elements: a "
            "cardinality mismatch warns and publishes nothing, and the joints "
            "then fall back to their rest chain rather than to either pose.",
            "An unwired `rigExec:inputA` or `rigExec:inputB` is not an error — "
            "the other input passes through unchanged at every weight, so a "
            "half-wired switch looks like a rig that simply ignores its "
            "weight.",
            "The kernel clamps `inputs:weight` to [0, 1], but shape it with a "
            "Float Math mover (as in `float_math_mover.usda`) when you want "
            "the extremes to dwell instead of relying on that bound.",
        ],
        "see_also": [
            ("fk_chain", "FK Chain"),
            ("two_bone_ik", "Two-Bone IK"),
            ("float_math_mover", "Float Math Mover"),
        ],
    },
    "twist_distribution": {
        "title": "Twist Distribution",
        "schema": "RigExecTwistDistribution",
        "summary": "Unwinds roll between two frames across N interpolated frames.",
        "description": """Takes the twist between a start and an end provider and spreads it
over a graduated run of frames — the standard forearm/spine setup where
the middle of a limb should turn half as far as the end. Output joints
can index any single frame of the run without listing all of them.""",
        "how_it_works": """The solver unwraps the endpoint twist, distributes it with minimum
energy over `rigExec:count` frames weighted by `rigExec:weights`, and
optionally adds `inputs:twistTurns` of extra aim-axis winding (integers
preserve the endpoints; fractions animate). Each claimed joint rides the
frame picked by its `rigExec:jointElements` entry.""",
        "wiring": [
            ("`rigExec:start`", "Provider holding the zero-twist end.", "yes"),
            ("`rigExec:end`", "Provider holding the full-twist end.", "yes"),
            ("`rigExec:joints`", "Joints posed by picked distributed frames.", "no"),
        ],
        "example": """The chest spins 90 degrees about the spine axis while the root stays
put. A mid-spine joint rides the middle of three distributed frames and
turns half as far, swinging the fin card it skins.""",
        "tips": [
            "Drive the end with `avars:rspin` (roll about the bone axis), not "
            "a tilt: only axial roll becomes distributed twist.",
            "Feed the run to a Ribbon as `twistFrames` so a spine strip "
            "inherits the same unwind.",
        ],
        "see_also": [
            ("ribbon", "Ribbon"),
            ("joint", "Joint"),
            ("fk_chain", "FK Chain"),
        ],
    },
    "ribbon": {
        "title": "Ribbon",
        "schema": "RigExecRibbon",
        "summary": "Samples a driver curve into transported frames for wrap deformers.",
        "description": """Turns a native B-spline driver into a run of evenly spaced,
rotation-minimizing frames. A curve mover then rides geometry along those
frames, so a strip keeps its length and cross-section while the driver
bends — the classic spine/tentacle rig. The mover reads the frame array
off this prim directly; naming a joint per sample on `rigExec:joints` is
optional, and buys a posed chain that constraints and matrix movers can
read like any other joint.""",
        "how_it_works": """All of it is pose phase: the solve is one branch of the
pose bake (`libs/rigExec/bakedPose.cpp:2552`), and `RigExecRibbon` is one
of the six baked solver types (`libs/rigExec/bakedProgram.cpp:124`). The
compiler resolves `rigExec:driverCurve` to the target's native `points`
attribute and the solve reads two values of it — the live one and the
bind-time (default) one — then samples both at equal arc length
(`libs/rigExecMath/geometryKernels.cpp:549`) and transports a
rotation-minimizing frame along each, publishing the posed frames paired
with their rests as one `computePointFrameArray`. Each
`rigExec:joints` entry takes one element of that array as its whole posed
frame, handed to exec as an override on the joint
(`libs/rigExec/rigEvaluator.cpp:10512-10515`). Because the driver is scene
data rather than a control-driven curve, the solve cannot see mover output
(`libs/rigExec/computations.cpp:1032`); what a wrap measures against is
the bind-time curve.""",
        "wiring": [
            # rigEvaluator.cpp:3468-3479 resolves the target to its `.points`;
            # with no target nothing is resolved and solverKernels.cpp:28
            # returns an empty array, so the mover downstream passes points
            # through untouched. NOT enforced: no compile error names this
            # relationship (grep rigEvaluator.cpp for driverCurve -- every hit
            # is a digest, a resolve or a read-phase check). "yes" here means
            # the node does nothing without it, the way surface_mover's
            # rigExec:surface is written up.
            ("`rigExec:driverCurve`", "Native BasisCurves supplying the spine path; "
             "without it the solver publishes no frames rather than erroring.", "yes"),
            # bakedPose.cpp:475-481: "rigExec:parameterization, frameTransport,
            # startFrame, endFrame, twistFrames and driverCurveReadPhase are
            # deliberately NOT read: the computation does not read them
            # either". The exec computation's inputs are sampleCount and the two
            # point packets and nothing else (moverKernels.cpp:1203-1208). The
            # only place in libs/rigExec that names these three tokens at all is
            # _ValidateAdjustmentPoseConsumers (rigEvaluator.cpp:1193-1196), a
            # walk that returns early unless the stage carries a
            # RigExecCurvenetAdjustment. Deleting all three from the docs
            # example leaves every posed point bit-identical (measured).
            ("`rigExec:startFrame`", "Joint provider naming the run's base; declared "
             "for authoring intent, read by no computation.", "no"),
            ("`rigExec:endFrame`", "Joint provider naming the run's tip; the same.", "no"),
            ("`rigExec:twistFrames`", "Twist solver named as the run's unwind; the "
             "same.", "no"),
            # rigEvaluator.cpp:4795-4803: cardinality is sampleCount (zero
            # below two, so no element binds), and jointElements defaults to
            # list position (schema.usda:776-779). A solver with an empty
            # joints list is simply skipped by the claim pass
            # (rigEvaluator.cpp:4724-4727).
            ("`rigExec:joints`", "Ordered joints posed along the run, one per sample.", "no"),
        ],
        # RigExecRibbon inherits Boundable directly (schema.usda:729-730): it
        # is not a RigExecXformable and applies no RigExec API schema, so every
        # knob it has is already on its own page.
        "param_groups": [],
        "example": """A 24-quad strip stands seven units tall beside an invisible
four-CV B-spline driver — offset a half-width along the bind binormal, so
the driver run and the nine joint guides on it sit clear of the mesh
instead of inside it — and the driver's control points swing on a
staggered cycle. The strip curls up one side until its tip has travelled
2.2 units across, dropped 2.2 and swung 1.0 toward camera around frame
1006; it sweeps back through upright near frame 1012, at its deepest
point toward camera (2.8 units, frame 1013); then it whips 4.0 units out
the other way by frame 1018 and recovers, closing the loop exactly at
frame 1024. The out-of-plane half of that swing is the part worth
watching: it is authored a quarter cycle out of phase with the sideways
sweep, so the driver is never planar and the rotation-minimizing frames
visibly roll as the strip travels — a planar driver would bend without
ever twisting. Only the two upper control points carry that Z, and the
upper one carries nearly all of it: the first tangent is set by CV0 and
CV2, and the seed normal is the world axis least parallel to it, so a Z
component there large enough to beat the X component would pick a
different axis and roll the whole wrap 90 degrees.
`rigExec:sampleCount` is nine, a curve mover in `ribbon` mode rides the
strip along the run through its `primvars:st` bind coordinates, and a
nine-joint chain named on `rigExec:joints` takes the same nine samples so
the run reads as a dense spine running up the edge of the bending
ribbon.""",
        "tips": [
            "Never bind the driver perfectly straight. The seed normal is the "
            "world axis least parallel to the first tangent, picked once per "
            "curve, and a tie is broken by declaration order X, Y, Z "
            "(`libs/rigExecMath/geometryKernels.cpp:583-592`), so a straight "
            "bind against bent poses picks a different axis for the rest than "
            "for the pose and rolls the whole wrap 90 degrees. Rest it at a "
            "lean and keep the leaning component of the first tangent larger "
            "than the other one all the way through the animation.",
            "`rigExec:startFrame`, `rigExec:endFrame` and `rigExec:twistFrames` "
            "are declared but never read: the solve's only inputs are the "
            "driver points and `rigExec:sampleCount` "
            "(`libs/rigExec/moverKernels.cpp:1203-1208`, "
            "`libs/rigExec/bakedPose.cpp:475`). What actually pins the run is "
            "the bind-time driver curve.",
            "`rigExec:sampleCount` is the run's cardinality. Below two the "
            "solver publishes no frames at all "
            "(`libs/rigExec/solverKernels.cpp:28`) and no joint element can "
            "bind, and time-sampling it is rejected at compile "
            "(`libs/rigExec/rigEvaluator.cpp:4589`).",
        ],
        "see_also": [
            ("curve_mover", "Curve Mover"),
            ("twist_distribution", "Twist Distribution"),
            ("joint", "Joint"),
        ],
    },
    "aim_constraint": {
        "title": "Aim Constraint",
        "schema": "RigExecAimConstraint",
        "summary": "Rotates targets so a local axis points at blended sources.",
        "description": """FBX-style aim: the transform named by `rigExec:moves` rotates so its
aim vector points at the weighted source position, stabilized by the
world-up policy. The everyday use is eyes tracking a look-at control,
but the same node aims turrets, spotlights, and driven props.""",
        "how_it_works": """In the pose phase the constraint blends the ordered `rigExec:sources`
into one goal position, builds the aim rotation from `inputs:aimVector`,
`inputs:upVector`, and the world-up mode, then mixes the result over the
incoming pose through the common mover envelope. Axis masks and the
rotation offset shape the result per channel, and only rotation is
written. New assets use `sources`; `aimTarget`/`aimAxis`/`upPolicy`/
`preserve` are the legacy contract kept for existing stages.""",
        "wiring": [
            # rigEvaluator.cpp:5333-5339 -- empty sources is a compile error
            # ("has no constraint sources"), but for this schema type alone an
            # authored rigExec:aimTarget is read first as the legacy spelling
            # (rigEvaluator.cpp:5327-5332).
            ("`rigExec:sources`", "Ordered transform sources to track.", "yes"),
            # rigEvaluator.cpp:3661-3668 -- a source constraint "must move exactly
            # one transform-provider prim"; that target must be a UsdGeomXformable
            # (rigEvaluator.cpp:3705-3713).
            ("`rigExec:moves`", "Exactly one joint or provider to re-aim (pose phase).", "yes"),
            # rigEvaluator.cpp:5353-5371 -- at most one target, and only bound at
            # all for RigExecAimConstraint; an empty binding is legal and the
            # object world-up modes fall back at rigEvaluator.cpp:11657-11682.
            ("`rigExec:worldUpObject`", "Provider for object world-up modes.", "no"),
        ],
        "param_groups": [
            ("Common mover envelope", "RigExecMoverAPI"),
            ("Constraint base", "RigExecConstraint"),
            ("Ordered sources", "RigExecSourceConstraint"),
        ],
        "example": """Two eye joints track a look-at control that sweeps across the front of
the face. The control carries its own solid sphere guide, so the thing
being aimed at is a visible orange ball rather than an implied point: it
slides side to side and rises and falls, and each eye swings through
about 70 degrees horizontally and another 20 vertically following it.
Each eye is a small sphere with a bright iris cone on its front, both
skinned rigidly to the aimed joint, so the iris points wherever the eye
is aimed at every angle. Both eyes converge on the same target, so the
closer the ball comes the more their lines of sight cross.""",
        "tips": [
            "Put pose-phase constraints in a scope ordered below Geometry "
            "(bottom executes first) so skinning reads their final output.",
            "`rigExec:worldUpType = \"vector\"` with `inputs:worldUpVector` +Y "
            "keeps eyes level while they swing; `none` applies no roll "
            "correction at all (FBX minimum swing) -- but only once "
            "`rigExec:sources` is authored, because the legacy "
            "`rigExec:aimTarget` spelling still preserves the input up.",
            "Make the look-at handle visible: it is an ordinary control, so on "
            "the RigExecControl prim `guide:shape = \"sphere\"` with "
            "`guide:drawMode = \"geometry\"` draws a solid ball at its posed "
            "origin instead of the default wire circle, sized by "
            "`guide:scaleX/Y/Z` times the evaluated frame scale. Seeing the "
            "target is what makes an aim rig readable -- an eye pointing at "
            "nothing is just a rotating eye.",
        ],
        "see_also": [
            ("matrix_mover", "Matrix Mover"),
            ("control", "Control"),
            ("joint", "Joint"),
        ],
    },
    "position_constraint": {
        "title": "Position Constraint",
        "schema": "RigExecPositionConstraint",
        "summary": "Moves one provider to the weighted average of its sources' origins.",
        "description": """FBX-style position: the constraint takes the origins of the ordered
`rigExec:sources`, averages them by `inputs:sourceWeights`, adds
`inputs:translationOffset`, and writes that point onto the single provider
named by `rigExec:moves`. Rotation and scale are untouched — this operator
owns the translation channel only. It is how a prop is pinned between two
hands, how a hip rides between two feet, and — with an animated weight
array — how either of those hands off to the other.""",
        "how_it_works": """Every source-blending constraint runs in the pose phase, in the
composed order of the `Movers` namespace, so it revises a provider that
earlier solvers and constraints have already posed. Each evaluation it
resolves the current frame of every `rigExec:sources` target, reads
`inputs:sourceWeights` raw off the attribute at that frame's time, and
accumulates `sum(origin * weight) / sum(weight)` — the weights are
normalized, so they are ratios, not percentages. `inputs:translationOffset`
is added to that blended point, the `inputs:affectTranslation*` mask selects
which axes are claimed, and the common `RigExecMoverAPI` envelope
(`inputs:defaultWeight`, or a bound `rigExec:weightObject`) lerps the result
over the incoming origin. Only the origin changes; the frame's other
landmarks are carried along with it.""",
        "wiring": [
            # libs/rigExec/rigEvaluator.cpp:5333 -- empty sources is reported and the
            # compile fails.
            ("`rigExec:sources`", "Ordered transform providers whose origins are "
             "blended; the order pairs with `inputs:sourceWeights`.", "yes"),
            # libs/rigExec/rigEvaluator.cpp:3544-3551 -- a constraint with no
            # rigExec:moves at all is an error; rigEvaluator.cpp:3662-3667 -- a source
            # constraint must move exactly one.
            ("`rigExec:moves`", "Exactly one provider to reposition, or one "
             "`<mesh>.points` property for the geometry domain.", "yes"),
            # libs/rigExec/rigEvaluator.cpp:9724 -- empty is legal (all sources weight
            # 1.0 at rigEvaluator.cpp:9731); any other length than the source count
            # returns false at 9729 and rigEvaluator.cpp:11537-11542 passes the
            # constraint through with a diagnostic.
            ("`inputs:sourceWeights`", "One float per source. Empty means every "
             "source at equal, full weight.", "no"),
            # libs/rigExec/rigEvaluator.cpp:11322-11336 -- resolved to one scalar in
            # the transform domain; rigEvaluator.cpp:11369-11376 -- per point in the
            # geometry domain, where the solve runs unweighted.
            ("`rigExec:weightObject`", "Optional envelope field; replaces "
             "`inputs:defaultWeight` when bound.", "no"),
        ],
        "param_groups": [
            ("Common mover envelope", "RigExecMoverAPI"),
            ("Constraint base", "RigExecConstraint"),
            ("Ordered sources", "RigExecSourceConstraint"),
        ],
        "example": """Two posts stand at different heights and a plate hangs between them.
The joint's rest sits exactly at the posts' midpoint, so an even blend leaves
it there and every bit of motion you see is the blend: `inputs:sourceWeights`
starts even at (1, 1) — the blue ghost is that even-blend frame, the plate
parked over the midpoint — then swings to (1, 0), to (0, 1), and back to even,
so the plate departs from its rest in both directions with nothing else in the
rig animated. The pale line between the two post tops is the segment the source
origins define; `inputs:translationOffset` of (0, 0.9, 0) is the visible gap
between that line and the joint, which is the whole of "maintain offset" here
— there is no such switch.""",
        "tips": [
            # libs/rigExecMath/solvers.cpp:833-839 -- target accumulates origin*w,
            # then divides by the running total before the offset is added.
            "Weights are ratios, not percentages: the blend divides by their "
            "total, so (1, 0) and (5, 0) give the same answer. Only their "
            "relative size matters, and nothing needs to sum to one.",
            # libs/rigExecMath/solvers.cpp:836-838 -- totalWeight == 0 returns the
            # input unchanged; solvers.cpp:775-776, checked at 824-825 -- a negative
            # or non-finite source weight fails the solve atomically rather than
            # being clamped.
            "All-zero weights are an exact pass-through, not a collapse to the "
            "origin — but a negative weight invalidates the solve instead of "
            "being clamped, so keep animated weights at or above zero.",
            # libs/rigExec/rigEvaluator.cpp:4282-4302 (masks and offsets) and
            # 4304-4310 (rotation order) -- authoring a channel property the operator
            # does not honor is reported and the compile fails; the table row at
            # rigEvaluator.cpp:255-257 gives Position translation only, no rotation
            # order.
            "There is no maintain-offset switch: author "
            "`inputs:translationOffset`, which is added after the blend. "
            "`inputs:rotationOffset`, the `affectRotation*`/`affectScale*` masks "
            "and `rigExec:rotationOrder` are compile errors here, not silent "
            "no-ops — this operator writes translation only.",
        ],
        "see_also": [
            ("aim_constraint", "Aim Constraint"),
            ("parent_constraint", "Parent Constraint"),
            ("matrix_mover", "Matrix Mover"),
        ],
    },
    "rotation_constraint": {
        "title": "Rotation Constraint",
        "schema": "RigExecRotationConstraint",
        "summary": "Copies orientation from blended sources, leaving position alone.",
        "description": """FBX-style rotation copy: the prim named by `rigExec:moves` takes its
orientation from the blended `rigExec:sources`, while its translation,
scale, and shear pass through untouched. That split is the whole point —
a panel bolted to a post can turn with a distant handle without drifting
off the post. Per-axis masks and a degrees offset shape which part of the
source orientation is actually copied.""",
        "how_it_works": """It runs in the pose phase, on the single composed mover walk, after the
constrained provider's incoming frame is known. The kernel decomposes
that incoming frame, converts each source's orientation to Euler degrees
in `rigExec:rotationOrder`, and accumulates weighted *shortest* per-axis
deltas against the first positive-weight source, so the fully constrained
result depends only on the sources. `inputs:rotationOffset` is added in
degrees, the `inputs:affectRotation*` mask picks which axes are written,
and the common mover envelope (`inputs:defaultWeight`, or a bound
`rigExec:weightObject`) blends per axis between the incoming Euler and the
target one. Only the rotation of the decomposed frame is replaced before
the frame is rebuilt, which is why the origin and scale survive.""",
        "wiring": [
            # rigEvaluator.cpp:5333-5339 -- empty sources is a hard compile error
            ("`rigExec:sources`", "Ordered transform providers whose orientations are blended.", "yes"),
            # rigEvaluator.cpp:3661-3667 -- source constraints must move exactly one prim
            ("`rigExec:moves`", "Exactly one transform-provider prim to re-orient.", "yes"),
            # rigEvaluator.cpp:9720-9735 -- empty array is legal (weights assign 1.0 per
            # source); a wrong-length one fails the read, and rigEvaluator.cpp:11537-11542
            # turns that into a per-frame diagnostic and pass-through, not a compile error
            ("`inputs:sourceWeights`", "Per-source weights parallel to `rigExec:sources`; empty means every source at full weight.", "no"),
            # rigEvaluator.cpp:11321-11337 -- optional; when bound it replaces inputs:defaultWeight
            ("`rigExec:weightObject`", "Weight field supplying the envelope instead of `inputs:defaultWeight`.", "no"),
        ],
        "param_groups": [
            ("Common mover envelope", "RigExecMoverAPI"),
            ("Constraint base", "RigExecConstraint"),
            ("Ordered sources", "RigExecSourceConstraint"),
        ],
        "example": """A Swing control rolls about Z and, at the same time, slides up and to
the right; a rotation constraint copies its orientation onto the Panel
joint, and a matrix mover skins a twelve-quad card (`PanelCard`) to that
joint. `inputs:rotationOffset` stays at zero and the
`inputs:affectRotationX/Y/Z` masks stay at their all-on defaults here, so
what you see is the unshaped copy — the source orientation and nothing
else. Watch the handle slide up and to the right while the card's base
never leaves the post — only the roll crosses the constraint.""",
        "tips": [
            "Rotation is the only channel this operator writes: authoring "
            "`inputs:affectTranslation*`, `inputs:affectScale*`, "
            "`inputs:translationOffset`, or `inputs:scaleOffset` on it is a "
            "compile error rather than a silent no-op.",
            "A zero envelope is an exact pass-through — `inputs:enabled = false` "
            "or `inputs:defaultWeight = 0` leaves the incoming pose bit-for-bit, "
            "and the sources are not even resolved.",
            "Blending two sources works per Euler component against the first "
            "positive-weight source, so keep `rigExec:rotationOrder` matched to "
            "how the sources are keyed and their per-axis values within half a "
            "turn of each other.",
        ],
        "see_also": [
            ("aim_constraint", "Aim Constraint"),
            ("position_constraint", "Position Constraint"),
            ("matrix_mover", "Matrix Mover"),
        ],
    },
    "scale_constraint": {
        "title": "Scale Constraint",
        "schema": "RigExecScaleConstraint",
        "summary": "Copies blended source scale onto one target, per axis.",
        "description": """FBX-style scale: the transform named by `rigExec:moves` takes its
scale from the weighted average of the ordered `rigExec:sources`, and
nothing else about it changes. It is how a prop inherits a character's
size, and -- one constraint per moved transform, every one of them naming
the same source -- how a single master "size" handle scales several parts
of a rig in lock-step.
Because only the scale channel is written, the target swells and shrinks
about its own pivot instead of drifting.""",
        "how_it_works": """The constraint runs in the pose phase, on the transform provider
named by `rigExec:moves`. Each evaluation decomposes every source frame,
accumulates its scale weighted by the parallel `inputs:sourceWeights`
(an empty array weights every source fully), divides by the total weight,
adds the ADDITIVE `inputs:scaleOffset`, and writes that value into the
decomposed scale of the incoming frame — one axis at a time, only where
`inputs:affectScaleX/Y/Z` is on. The common mover envelope
(`inputs:defaultWeight`, or a bound `rigExec:weightObject`) then mixes the
candidate over the incoming scale, so a zero envelope is an exact
pass-through. Translation, rotation and shear of the moved frame are
carried through untouched, and downstream skinning reading the joint at
`final` picks the new scale up automatically.""",
        "wiring": [
            # rigEvaluator.cpp:5326-5340 -- empty rigExec:sources (no aimTarget
            # fallback outside Aim) is "has no constraint sources", compile fails.
            ("`rigExec:sources`", "Ordered transform providers whose scales are "
             "blended into the goal.", "yes"),
            # rigEvaluator.cpp:3661-3667 -- a source-frame constraint whose
            # rigExec:moves is not exactly one target is rejected:
            # "must move exactly one transform-provider prim".
            ("`rigExec:moves`", "Exactly one transform provider to rescale "
             "(pose phase).", "yes"),
            # rigEvaluator.cpp:9719-9734 -- an empty (or absent) array is legal
            # and means every source at 1.0; a non-empty array of the wrong
            # length is a diagnostic and the constraint is passed through
            # unapplied (rigEvaluator.cpp:11537-11542).
            ("`inputs:sourceWeights`", "Per-source weights parallel to "
             "`rigExec:sources`; empty means all-equal.", "no"),
            # rigEvaluator.cpp:5302-5309 -- at most one target, validated in
            # phase A; schema.usda:79-88 -- when bound it replaces
            # inputs:defaultWeight rather than multiplying into it.
            ("`rigExec:weightObject`", "Weight field supplying the envelope "
             "instead of `inputs:defaultWeight`.", "no"),
        ],
        "param_groups": [
            ("Common mover envelope", "RigExecMoverAPI"),
            ("Constraint base", "RigExecConstraint"),
            ("Ordered sources", "RigExecSourceConstraint"),
        ],
        "example": """A 6x4 quad card is skinned rigidly to one joint sitting at its
centre, and a Sizer control off to the left animates only `avars:sx` and
`avars:sy`. The scale constraint copies that scale onto the joint with
`affectScaleZ` off, so the card stretches to 1.8x wide and 1.3x tall,
collapses to about half size, and returns — growing and shrinking in
place while its centre never moves.""",
        "tips": [
            "`inputs:scaleOffset` is ADDITIVE: its identity is (0, 0, 0), not "
            "(1, 1, 1), because the kernel computes blended scale + offset "
            "(`RigExecApplyScaleConstraint`, libs/rigExecMath/solvers.cpp:945).",
            "Only the scale channel is honored — authoring "
            "`inputs:translationOffset`, `inputs:rotationOffset`, any "
            "`inputs:affectTranslation*`/`inputs:affectRotation*`, or "
            "`rigExec:rotationOrder` on a scale constraint is a compile error, "
            "never a silent no-op.",
            "Several sources average, they do not multiply: two equally weighted "
            "sources at scale 2 and 1 land the target at 1.5. Use "
            "`inputs:sourceWeights` to bias between them.",
        ],
        "see_also": [
            ("aim_constraint", "Aim Constraint"),
            ("control", "Control"),
            ("matrix_mover", "Matrix Mover"),
        ],
    },
    "parent_constraint": {
        "title": "Parent Constraint",
        "schema": "RigExecParentConstraint",
        "summary": "Carries a target with its sources — position and rotation — under a per-source offset; weight attaches and releases.",
        "description": """FBX-style parenting expressed as a constraint rather than as namespace
nesting: the prim named by `rigExec:moves` is carried by the blended
`rigExec:sources`, keeping the offset authored for each source. It is the
one constraint that writes all three channel groups, and the one whose
scale group is **off** by default, matching the FBX runtime — a parented
prop inherits where its parent is and which way it faces, not how big it
is. Because it is an ordinary mover, `inputs:defaultWeight` is the
attach/release channel: key it down and the prop settles back into its own
space.""",
        "how_it_works": """In the pose phase the constraint resolves each entry of the ordered
`rigExec:sources` to that source's current revision, composes the matching
`inputs:translationOffsets[i]` / `inputs:rotationOffsets[i]` entry *before*
the source transform (`targetMatrix = offset * source`), and accumulates a
weighted average of the resulting translations and scales plus a
shortest-arc Euler average of the rotations, anchored on the first
contributing source and read in `rigExec:rotationOrder`. That candidate is
then written per axis through the three `inputs:affect*` mask triples and
blended over the target's incoming frame by the common mover envelope, so
the single `rigExec:moves` target is revised in place and anything that reads
that provider afterwards -- a skinning mover with
`rigExec:transformReadPhase = "final"`, for instance -- sees the parented
result. A zero envelope is an exact pass-through: the target keeps whatever
posed it before the constraint ran.""",
        "wiring": [
            # rigEvaluator.cpp:5333-5339 -- empty sources is a compile error
            # ("... has no constraint sources"); the aimTarget fallback there is
            # Aim-only.
            ("`rigExec:sources`", "Ordered transform providers that carry the target; "
             "order is shared with every parallel array.", "yes"),
            # rigEvaluator.cpp:3661-3667 -- a source constraint "must move exactly
            # one transform-provider prim"; the target must be a UsdGeomXformable
            # (rigEvaluator.cpp:3702-3712).
            ("`rigExec:moves`", "The one transform provider this constraint revises "
             "(pose phase).", "yes"),
            # rigEvaluator.cpp:3951-3961 -- more than one target is an error; empty
            # is fine and inputs:defaultWeight supplies the envelope instead
            # (rigEvaluator.cpp:11337-11340).
            ("`rigExec:weightObject`", "Optional constant weight field supplying the "
             "attach envelope instead of `inputs:defaultWeight`.", "no"),
            # rigEvaluator.cpp:9740-9763 -- an empty array is the neutral value and
            # a non-empty one must have exactly one entry per source; read for this
            # schema type only at rigEvaluator.cpp:10839-10846.
            ("`inputs:translationOffsets`", "Per-source translation offsets, parallel "
             "to `rigExec:sources` (empty = zero).", "no"),
            # rigEvaluator.cpp:9740-9763, same rule.
            ("`inputs:rotationOffsets`", "Per-source Euler offsets in degrees, parallel "
             "to `rigExec:sources` (empty = zero).", "no"),
            # rigEvaluator.cpp:9712-9735 -- empty means every source at full weight;
            # non-empty must be exactly one entry per source.
            ("`inputs:sourceWeights`", "Per-source blend weights (empty = every source "
             "at full weight).", "no"),
        ],
        "param_groups": [
            ("Common mover envelope", "RigExecMoverAPI"),
            ("Constraint base", "RigExecConstraint"),
            ("Ordered sources", "RigExecSourceConstraint"),
        ],
        "example": """A Prop joint is parented to the Hand control with a
`(1.6, 0, 0)` translation offset and a 25-degree Z rotation offset, and a flat
card is skinned rigidly to that joint, so the card rides out and rotates with
the hand while holding that exact distance and tilt.
Mid-shot `inputs:defaultWeight` fades 1 → 0 and the card slides back to its
own rest space while the hand stays out; the weight returns to 1 and the
card re-attaches at the same offset it left with, then the hand carries it
home.""",
        "tips": [
            "Scale is off on every axis by default — this is the only constraint "
            "that overrides the base class's all-on default, to match the FBX "
            "runtime — so a parented prop keeps its own scale until you author "
            "`inputs:affectScale*`.",
            "The offsets are per-source arrays. Authoring the scalar "
            "`inputs:translationOffset` / `inputs:rotationOffset` / "
            "`inputs:scaleOffset` inherited from the constraint base is a compile "
            "error here, because this operator composes the arrays instead.",
            "`inputs:defaultWeight` is re-read every frame, so keying it is the "
            "attach/release channel; at 0 the constraint is a bit-for-bit "
            "pass-through and the target falls back to whatever posed it earlier.",
        ],
        "see_also": [
            ("aim_constraint", "Aim Constraint"),
            ("control", "Control"),
            ("matrix_mover", "Matrix Mover"),
        ],
    },
    "single_chain_ik_constraint": {
        "title": "Single-Chain IK Constraint",
        "schema": "RigExecSingleChainIkConstraint",
        "summary": "Re-poses an existing joint chain of any length onto an effector goal.",
        "description": """FBX-style single-chain IK, and the only IK in UsdRig that is a
**constraint** rather than a solver: it does not publish a frame array that
joints extract from, it revises the joint frames that are already there —
whatever the last solver in each joint's stack committed.
Name the two endpoints — `rigExec:firstJoint` and `rigExec:endJoint` — and the
chain between them is inferred from namespace nesting, so the same node drives
a two-joint chain or a ten-joint one. The first joint's origin stays planted,
every segment keeps its length, and the end joint lands on the effector
whenever the goal is in reach.""",
        "how_it_works": """The constraint runs in the pose phase, ordered BELOW the
geometry movers in the Movers stack — the hierarchy runs bottom-up, so the
chain is solved before the skin movers read it at their `final` transform
read phase. At compile time it walks `endJoint`'s ancestors up to
`firstJoint` to infer the ordered chain, and `rigExec:moves` must restate that
complete set. Each evaluation it reads the chain's current frames and the
effector's frame, solves the positions with deterministic FABRIK inside the
plane chosen by `rigExec:solverMode` (`rotatePlane` uses the pole point and
`inputs:twistDegrees`; `singleChain` takes the plane from the effector's own
orientation and ignores both), then rebuilds each non-end frame with its X axis
aimed at the next solved joint and writes the whole chain back atomically — a
failed solve passes every joint through untouched rather than half-posing the
limb. `rigExec:evaluationMode` decides whether the segment lengths come from
the joints' rests (`neverTS`, the default) or from their animated translation
and scale.""",
        "wiring": [
            # rigEvaluator.cpp:5384-5392 -- exactly one prim target, or compile fails.
            ("`rigExec:firstJoint`", "Joint that anchors the chain; its origin never moves.", "yes"),
            # rigEvaluator.cpp:5393-5419 -- must be a namespace descendant of firstJoint,
            # and every prim between them must be a RigExecJoint.
            ("`rigExec:endJoint`", "Final joint of the chain, nested under the first.", "yes"),
            # rigEvaluator.cpp:5384-5392 (exactly one) and 5443-5450 (bound as a frame source).
            ("`rigExec:effector`", "Single transform provider supplying the IK goal.", "yes"),
            # rigEvaluator.cpp:5430-5441 -- moves must EQUAL the inferred chain, no more,
            # no less; rigEvaluator.cpp:3715-3727 additionally requires >= 2 prim targets.
            ("`rigExec:moves`", "Every joint of the inferred chain, restated exactly.", "yes"),
            # rigEvaluator.cpp:5451-5474 -- bound only in rotatePlane mode, and never
            # required at compile time; rigEvaluator.cpp:11428-11435 downgrades an empty
            # list in object pole mode to a pass-through diagnostic.
            ("`rigExec:poleVectorObjects`", "Providers blended into the pole point when "
             "`poleVectorMode` is `object`; unused by `singleChain`.", "no"),
            # schema.usda:79-89 -- optional on every mover; when bound it replaces
            # inputs:defaultWeight as the envelope. rigEvaluator.cpp:858-865 and
            # 5305-5310: this operator is always multi-target, so the only legal
            # binding is a CONSTANT one-element field whose rigExec:weightTarget is
            # the constraint prim itself.
            ("`rigExec:weightObject`", "Constant one-element weight field targeting the "
             "constraint prim, supplying the envelope instead of "
             "`inputs:defaultWeight`.", "no"),
        ],
        "param_groups": [
            ("Common mover envelope", "RigExecMoverAPI"),
            ("Constraint base", "RigExecConstraint"),
        ],
        "example": """A box arm — three joints, two segments — lies along +X; the diamond
effector control swings up and back over the frame range and the wrist tracks
it exactly, folding the arm at the elbow. The pyramid control above the elbow
is the pole-vector object, and it swings through z: the bend plane follows it,
so the elbow leaves the XY plane and comes back. Two matrix movers skin the one
mesh from the two bones through dense static weights that partition the points
at the elbow ring.""",
        "tips": [
            "`solverMode = \"singleChain\"` ignores the pole and `inputs:twistDegrees` "
            "entirely — the effector's orientation picks the bend plane — and the "
            "pole relationships are not even bound in that mode, so leftover pole "
            "wiring cannot break the compile.",
            "There is no stretch or softness dial: segment lengths are preserved "
            "exactly, and a goal past full reach returns the chain straightened "
            "down the root-to-goal ray. Reach for Two-Bone IK when you want stretch.",
            "The chain, the effector and the pole are named by their own "
            "relationships: this operator does not read the generic "
            "`rigExec:sources` list at all.",
            "Do not author the inherited `inputs:affect*` masks or the "
            "`inputs:translationOffset`/`rotationOffset`/`scaleOffset` values here, "
            "and do not author `rigExec:rotationOrder`: this operator honors no "
            "channel group, and any authored opinion — even one equal to the "
            "default — is a compile error.",
        ],
        "see_also": [
            ("two_bone_ik", "Two-Bone IK"),
            ("aim_constraint", "Aim Constraint"),
            ("joint", "Joint"),
        ],
    },
    "matrix_mover": {
        "title": "Matrix Mover",
        "schema": "RigExecMatrixMover",
        "summary": "Carries points by a provider's rigid delta under a weight field.",
        "description": """The workhorse deformer: it reads one transform provider's
rest-relative delta and applies it to the moved points, scaled per point
by a weight object. One mover at constant weight is a rigid attachment;
several stacked on one target are applied in sequence, each from the
preceding revision, which is exact wherever a point has a single
influence and is not linear blend skinning where weights overlap — that
is the Skin Mover's job.""",
        "how_it_works": """The provider publishes `computeMatrix`, the rest-to-posed map of its
own frame (computations.cpp:401-415), and the mover blends it over the
incoming points as `p' = q + w (T q - q)` (schema.usda:1677-1679), where
`w` is the bound weight field or, with none bound,
`inputs:defaultWeight`. `rigExec:transformReadPhase` chooses which
revision of the provider is read: the default `base` binds the provider
itself, `final` binds the head of its frame chain
(moverGraph.cpp:1366-1379). The result is passed down the point chain,
and the compiler synthesizes the recompute revisions that keep authored
`normals` and `extent` on that gprim current (rigEvaluator.cpp:5821-5863).""",
        "wiring": [
            # rigEvaluator.cpp:7203 rejects anything but exactly one target, and
            # :7221-7231 rejects a target that is not a RigExecControl or
            # RigExecJoint ("not a catalogued matrix provider").
            ("`rigExec:transform`", "Exactly one control or joint to follow.", "yes"),
            # rigEvaluator.cpp:3956-3960: more than one binding is an error, none
            # is fine -- the envelope then comes from inputs:defaultWeight.
            ("`rigExec:weightObject`", "Weight field scaling the follow (optional; "
             "overrides the envelope when bound).", "no"),
            # rigEvaluator.cpp:7172-7196: one native point3f[] `points` property on
            # a stock PointBased prim, nothing else. (Omitting the relationship
            # entirely is not an error: the prim is then not a mover at all --
            # skipped as a grouping scope, :3543-3557 -- and an authored but empty
            # one is reported inert, :3561-3578.)
            ("`rigExec:moves`", "Exact points property to deform.", "yes"),
        ],
        "param_groups": [
            ("Common mover envelope", "RigExecMoverAPI"),
        ],
        "example": """A solid box control sits on a flat card, and the mover's
`rigExec:transform` points straight at that control — no joint in between —
so the card is rigidly bolted to the handle. The box slides 2.2 units along
X and yaws 24 degrees and back, and the card goes with it corner for corner,
which is what "rigid delta" means. No weight object is bound and the envelope
stays at 1, so the follow is full strength over every point of the card.""",
        "tips": [
            "The provider may be a control as readily as a joint: those are the "
            "only two types `rigExec:transform` accepts "
            "(rigEvaluator.cpp:7221-7231).",
            "`final` binds the provider's frame-chain head, so every pose step "
            "above it is included; the default `base` is the joint after the LAST "
            "SOLVER wrote it, which is not the same as \"before every "
            "constraint\" — a constraint that sits below the last solver is "
            "folded into `base` through that solver. Name a prim if you want a "
            "specific moment: the `Solvers` scope means \"after the last "
            "solver\", the `Movers` scope \"after the last constraint\" "
            "(moverGraph.cpp:1366-1379, schema.usda:1685).",
            "Same-target movers are an ordinary stack ordered by the composed "
            "namespace: reverse-sibling post-order, so descendants run before "
            "their parent and the bottom sibling before the top (spec section "
            "4.2). Stacking is how you layer rigid follows, not how you blend "
            "influences on one point — use the Skin Mover for that "
            "(schema.usda:1700-1705).",
        ],
        "see_also": [
            ("skin_mover", "Skin Mover"),
            ("static_weight", "Static Weight"),
            ("joint", "Joint"),
        ],
    },
    "skin_mover": {
        "title": "Skin Mover",
        "schema": "RigExecSkinMover",
        "summary": "Blends many influences per point in one pass, UsdSkel-style.",
        "description": """Character skinning in a single node: `rigExec:influences` lists the
joints, and the UsdSkel-shaped `rigExec:jointIndices` /
`rigExec:jointWeights` arrays say which of them move each point and by
how much. A stack of matrix movers applies one influence at a time over
the preceding revision, which matches a skinCluster only where a point
has exactly one influence; the skin mover gathers every influence of a
point and blends them together, so shared points bend smoothly instead
of hinging. `rigExec:skinningMethod` picks the blend: `classicLinear`
for linear blend skinning, `dualQuaternion` for the volume-preserving
one.""",
        "how_it_works": """The mover is one revision in its target's point chain, so it runs in
the mover-application walk after solving: it reads
every influence's `computeMatrix` (the rest-to-posed map) at
`rigExec:transformReadPhase`, gathers `rigExec:elementSize` index/weight
slots per point in point order, and accumulates them — `classicLinear`
sums `w_k T_k p` and leaves the weight shortfall `1 - sum w_k` on the
rest point, while `dualQuaternion` splits each influence once per
evaluation into a pre-rotation stretch and a unit dual quaternion,
blends along the shortest arc with one normalisation, and enters the
same shortfall as an identity influence. The resulting points are then
mixed against the incoming revision by the common mover envelope —
`inputs:defaultWeight`, or a bound `rigExec:weightObject` — so the
envelope fades the whole skin, not one influence. The layout's element
shape, index range and non-negative finite weights are checked at
compile and again whenever the cached layout is resolved, and the
influence matrices are checked every frame (finite, affine); a mismatch
fails the application rather than skinning a truncated array.""",
        "wiring": [
            # rigEvaluator.cpp:7276-7280 -- empty rigExec:influences is a compile error.
            # rigEvaluator.cpp:7281-7291 -- each target must be a RigExecControl or RigExecJoint.
            ("`rigExec:influences`", "Ordered joints or controls the indices "
             "address; at least one, and each must be a catalogued provider.", "yes"),
            # rigEvaluator.cpp:7247-7270 -- exactly one exact point3f[] points
            # property on a stock PointBased prim.
            ("`rigExec:moves`", "The one exact `points` property to skin.", "yes"),
            # rigEvaluator.cpp:7335-7350 -- lengths must match each other and, for a
            # non-empty target mesh, points * elementSize.
            ("`rigExec:jointIndices` / `rigExec:jointWeights`",
             "Parallel per-point arrays, `elementSize` slots per point in point "
             "order.", "yes"),
            # moverGraph.cpp:1746-1749 -- no weight object means inputs:defaultWeight
            # broadcasts as the envelope; a bound packet supplies it per point.
            ("`rigExec:weightObject`", "Optional field supplying the envelope "
             "instead of `inputs:defaultWeight`; fades the whole skin.", "no"),
        ],
        "param_groups": [
            ("Common mover envelope", "RigExecMoverAPI"),
        ],
        "example": """Two identical 24-quad ribbons lie flat in the floor on either side of
one three-joint chain, and each is skinned by ONE skin mover. Both
movers name the same `rigExec:influences` and carry the same
`rigExec:jointIndices` / `rigExec:jointWeights` — two influence slots
per point, ramping 1 → 0 linearly across each span, so the bend draws as
a curve rather than a crease at the joints. The only difference between
them is `rigExec:skinningMethod`: the bright ribbon is
`dualQuaternion`, the dim one `classicLinear`. Where the wrist rolls 45°
the linear ribbon's cross-section pinches from its rest width of 1.500
to 1.386 and its tip falls short of the arc; the dual-quaternion ribbon
holds 1.500 at every column.""",
        "tips": [
            "`rigExec:elementSize` is the same for every point: pad a point that "
            "needs fewer influences with a zero weight. Weights are never "
            "renormalized, and a sum below one holds the point toward its rest "
            "position in proportion rather than dragging it to the origin.",
            "Influences must be RigExec provider prims (controls or joints), not "
            "arbitrary matrix attributes, and the read phase accepts only `base` "
            "or `final` even though the schema lists `preceding`; read `final` so "
            "constraints that revise a joint are included.",
            "`classicLinear` has the SIMD path and is the cheaper blend; "
            "`dualQuaternion` is scalar but keeps volume through twists, which is "
            "what a candy-wrapped wrist or the pinch at a bent elbow is missing.",
        ],
        "see_also": [
            ("matrix_mover", "Matrix Mover"),
            ("fk_chain", "FK Chain"),
            ("joint", "Joint"),
        ],
    },
    "blendshape_mover": {
        "title": "Blendshape Mover",
        "schema": "RigExecBlendShapeMover",
        "summary": "Sums sculpted blend channels into one delta pass.",
        "description": """Applies facial-style blend shapes: each bound blend input contributes
its weighted channel delta, the mover sums them all, scales the total by
its weight object, and adds it to the moved points. Channels compose
independently, so a smile and a brow raise layer without interfering.""",
        "how_it_works": """Every `rigExec:blendInputs` channel evaluates its samples against
its weight (interpolating in-betweens by activation), producing one
delta array. The mover sums the channel deltas, multiplies by the common
envelope and the bound mask field, and revises the points. Target
topology must match the moved mesh point for point.""",
        "wiring": [
            ("`rigExec:blendInputs`", "Blend channels to sum.", "yes"),
            ("`rigExec:weightObject`", "Mask scaling the summed delta.", "no"),
            ("`rigExec:moves`", "Exact points property to deform.", "yes"),
        ],
        "param_groups": [
            ("Common mover envelope", "RigExecMoverAPI"),
        ],
        "example": """A face card plays two channels in sequence: Smile sweeps through an
in-between (corners widen, then lift) and BrowRaise pulses the top row
up, all under a constant mask.""",
        "tips": [
            "Keep targets as invisible Points prims beside the geometry so the "
            "stage stays self-contained.",
            "A painted mask lets one mover shape the whole face while "
            "protecting the ears, neck seam, or scalp.",
        ],
        "see_also": [
            ("blend_input", "Blend Input"),
            ("blend_sample", "Blend Sample"),
            ("static_weight", "Static Weight"),
        ],
    },
    "curve_mover": {
        "title": "Curve Mover",
        "schema": "RigExecCurveMover",
        "summary": "Transports points through a solver's frame array, or emits its frame origins.",
        "description": """Frame transport for geometry. In `ribbon` mode each moved point
carries a bind parameter that picks a spot in the driver frame array, and
the point is mapped by the rest-relative rigid transform read there — so a
dense mesh spread across the parameter bends, while a prop whose every
vertex shares one parameter rides rigidly. In `emitGuidePoints` mode the
mover skips the bind and writes the frame origins straight out as points.
Nothing in either mode samples a curve: the frames are the input, and any
solver that publishes a frame array — a ribbon, an FK chain, spline IK, a
twist distribution — can supply them.""",
        "how_it_works": """The mover runs as one revision on its `rigExec:moves` target's point
chain, reading the driver solver's frames after the pose phase has solved
them. It builds one rigid map per element from that element's rest
landmarks to its posed landmarks, then places each point by clamping its
bind u to [0, 1], scaling it across the array, and blending the two
neighbouring maps' results — so the array's cardinality, not a curve, sets
the resolution of the transport. Guide emission needs no bind at all: it
copies frame origin `i` to point `i`, and fails the revision if the counts
differ. The compiler maintains the moved mesh's normals and extent
afterwards.""",
        "wiring": [
            # libs/rigExec/moverKernels.cpp:816-821 (exec) and
            # libs/rigExec/moverGraph.cpp:2262-2267 (lowered graph): empty frames,
            # or rests that do not match the frame count, leave the parameters
            # invalid. Not a compile error: the revision passes through and the
            # chain sweep reports "MoverFailed <path>: execution rejected its
            # inputs; revision passed through" (libs/rigExec/bakedGeometry.cpp:
            # 2094-2097) with the pose still valid.
            ("`rigExec:driverFrames`", "Any solver publishing a frame array "
             "(ribbon, FK chain, spline IK, twist distribution).", "yes"),
            # libs/rigExec/moverKernels.cpp:823-827 and moverGraph.cpp:2269-2276:
            # `params.valid = !params.bindCoords.empty()` in ribbon mode only;
            # emitGuidePoints sets valid unconditionally.
            ("`rigExec:bindCoordinates`", "Per-point `primvars:st`; u picks the "
             "spot in the frame array.", "ribbon mode"),
            # libs/rigExec/rigEvaluator.cpp:3766-3800: a typed points mover's
            # target must be a point3f[] `points` property on a UsdGeomPointBased
            # prim, or the compile is rejected.
            ("`rigExec:moves`", "Exact native points property to write.", "yes"),
            # libs/rigExec/moverGraph.cpp:1489-1503 builds this mover's binding
            # from bindCoordinates and driverFrames only; rigEvaluator.cpp:2704
            # merely hashes driverCurve into the epoch digest, and the only prim
            # whose rigExec:driverCurve is resolved is RigExecRibbon
            # (rigEvaluator.cpp:3468-3479). No kernel reads it here.
            ("`rigExec:driverCurve`", "Unread on this node — the frames already "
             "carry the curve's shape.", "no"),
        ],
        "param_groups": [
            ("Common mover envelope", "RigExecMoverAPI"),
        ],
        "example": """Three controls stacked up Y are the only animation, and an FK chain
that claims no joints turns them into a three-element frame array. One
curve mover spreads a 45-point streamer -- five columns across, nine rows
up -- over the whole u range so it curls and twists between the frames, a
second pins all four corners of each card to a single u so the cards ride
rigidly at 0.15, 0.5 and 0.85, and a third in `emitGuidePoints` mode
writes the same three frame origins out as a gold guide curve. The
streamer sits wholly at negative x and the cards at positive x, so the
gold line up the middle is the third mover's output on its own -- the
frame origins themselves, one point per element. There is no ribbon, no
joint and no driver curve in the file.""",
        "tips": [
            "Any frame-publishing solver drives this mover — an FK chain that "
            "claims no joints is a legal frame source, and the array's element "
            "count is the transport resolution.",
            "`emitGuidePoints` writes one point per frame: give the target "
            "exactly as many points as the solver has elements or the revision "
            "fails and passes through unchanged.",
            "Bind u is clamped and scaled across the array, so equal u on every "
            "vertex of a prop rides one spot rigidly and a spread of u bends; "
            "u = 0 rides the first element's map exactly, u = 1 the last.",
        ],
        "see_also": [
            ("ribbon", "Ribbon"),
            ("fk_chain", "FK Chain"),
            ("matrix_mover", "Matrix Mover"),
        ],
    },
    "lattice_mover": {
        "title": "Lattice Mover",
        "schema": "RigExecLatticeMover",
        "summary": "Deforms points through an animated Bernstein or B-spline cage.",
        "description": """Free-form deformation: a native Points or mesh cage surrounds the
geometry, and posing the cage (default time is the bind, timeSamples are
the posed cage) carries the moved points through tensor-product basis
evaluation. Fewer cage points than mesh vertices drive broad, smooth
shaping — bulges, bends, squash and stretch.""",
        "how_it_works": """Each moved point is located in the bind cage's lattice coordinates,
then re-evaluated in the posed cage under the `bernstein` or `bspline`
basis. `rigExec:divisions` sets the cage resolution per axis with
x-fastest point ordering; the cage is read at `rigExec:cageReadPhase`
(usually `base`, the authored animation).""",
        "wiring": [
            ("`rigExec:cage`", "Native Points/mesh prim supplying cage points.", "yes"),
            ("`rigExec:moves`", "Exact points property to deform.", "yes"),
        ],
        "param_groups": [
            ("Common mover envelope", "RigExecMoverAPI"),
        ],
        "example": """A 2×3×2 Bernstein cage bulges its middle layer outward and back,
widening the middle of a vertical strip. Bind and posed cage share the
same point ordering.""",
        "tips": [
            "Hide the cage (`visibility = invisible`): it is scaffolding, not "
            "geometry.",
            "Follow a bulge with a Smooth mover to settle the lattice falloff "
            "or a Volume Correct mover to hold girth.",
        ],
        "see_also": [
            ("smooth_mover", "Smooth Mover"),
            ("volume_correct_mover", "Volume Correct Mover"),
            ("matrix_mover", "Matrix Mover"),
        ],
    },
    "surface_mover": {
        "title": "Surface Mover",
        "schema": "RigExecSurfaceMover",
        "summary": "Drapes points onto an animated driver surface.",
        "description": """Conforms sticker patches to a moving mesh: every moved point is pulled
onto the closest point of the driver's triangulated surface, and the
common envelope blends that contact back over the incoming points. Use it
for decals, clothing patches, and anything that must finish flush against
deforming skin. In v0.1 `rigExec:mode` selects no numeric difference —
`attach` and `project` both project at full strength — so the mover is a
contact pass, not a slide-along-the-surface follow.""",
        "how_it_works": """The mover is one revision in its target's point chain, so it runs in
the mover-application walk after solving and after every earlier revision
on that chain. It reads the driver prim's `points` at
`rigExec:surfaceReadPhase` plus its `faceVertexCounts` /
`faceVertexIndices`, fans every face into a triangle fan, and takes the
closest point over all of them per moved point; the envelope then blends
that candidate over the incoming revision, and the compiler re-synthesizes
authored `normals` and `extent` afterwards. The search carries no
frame-to-frame state, so a point roughly a facet away from the driver
tracks it smoothly while a point far away can flip between near-tied
facets and pop.""",
        "wiring": [
            # rigEvaluator.cpp:8795-8797 (`if (surfaces.empty()) { continue; }`)
            # and moverKernels.cpp:795-796 (params.valid false without surface
            # points): an unwired surface is never a compile error, the mover
            # just passes its incoming points through (verified: the stage still
            # compiles and evaluates with the relationship deleted).
            ("`rigExec:surface`", "Native mesh prim supplying the driver "
             "surface; without it the mover is inert, not an error.", "yes"),
            # rigEvaluator.cpp:3545-3557: a mover prim with no rigExec:moves is
            # reclassified as a grouping scope; 3560-3577: authored but empty
            # targets report a notice and go inert for the generation.
            ("`rigExec:moves`", "Exact points property to drape.", "yes"),
            # moverGraph.cpp:1450-1460: only a non-base phase binds the driver
            # to a moved chain output. moverGraph.cpp:1101-1135: read-phase
            # metadata on the rigExec:surface relationship wins, and this uniform
            # token is the role-named fallback. rigEvaluator.cpp:4174-4202
            # rejects an unparseable token at compile.
            ("`rigExec:surfaceReadPhase`", "`base` for the driver's authored "
             "points, `final` when the driver is itself rigged.", "no"),
        ],
        "param_groups": [
            ("Common mover envelope", "RigExecMoverAPI"),
        ],
        "example": """A 6x6 quad sheet hinges up about its camera-right edge and back under
one `avars:rz` control, and two sticker patches sharing one mesh ride it,
floating 1.4 above the sheet at rest. `FollowSheet` carries the patches
roughly along with the same joint through a per-point weight, and
`DrapeStickers` then projects them onto the sheet read at `final`, which
is why the drape stays smooth: every sticker point starts within a facet
of the surface. The 0.75 envelope stops each patch a steady distance
above the sheet rather than landing it coplanar, which is what keeps it
out of a z-fight with the surface it landed on — and what you see moving
is that standoff staying parallel to the sheet as the sheet bends away
underneath it. `FollowSheet` is nested inside `DrapeStickers` and
`BendSheet` is authored last, because movers run in reverse composed
namespace order with descendants ahead of their parent.""",
        "tips": [
            "Keep the moved points close to the driver — roughly a facet or "
            "less. The closest-point search runs per point per frame with no "
            "continuity, so distant points sit on near-ties between facets and "
            "jump when the winner changes; a rough matrix-mover follow ahead of "
            "the drape is the fix.",
            "Read the driver at `final` when the driver mesh is itself moved by "
            "the rig, and at `base` when it carries authored point animation; "
            "`base` on a rigged driver drapes onto the undeformed surface.",
            "`rigExec:mode` is inert in v0.1: the kernel pins the operation to "
            "full-strength projection for both tokens, so `attach` does not yet "
            "transport points along the surface. Shape the result with the "
            "common envelope instead — below 1 it leaves a standoff and stops "
            "the patch z-fighting with the surface it landed on.",
        ],
        "see_also": [
            ("matrix_mover", "Matrix Mover"),
            ("lattice_mover", "Lattice Mover"),
            ("static_weight", "Static Weight"),
        ],
    },
    "smooth_mover": {
        "title": "Smooth Mover",
        "schema": "RigExecSmoothMover",
        "summary": "Relaxes points with uniform Laplacian smoothing.",
        "description": """One full Laplacian step over the mesh's own edge adjacency, blended
through the common envelope. It settles lattice falloff, melts sculpt
pops, and rounds low-poly cages — anywhere high-frequency shape needs
taking down without leaving the mover stack.""",
        "how_it_works": """It runs as one point revision in the mover stack, its place in the
order taken from its position in the composed namespace like every other
mover. Each point moves toward the average of its edge neighbors
(uniform weights, fixed adjacency built from the destination prim's own
`faceVertexCounts` / `faceVertexIndices`); the envelope mixes that full
step over the incoming points. One iteration is a low-pass filter, so
detail at the sampling limit of the cage goes first and broad shape
survives nearly untouched — at envelope 1 a symmetric spike collapses
exactly to its neighbors' centroid, so partial envelopes are the normal
working range. Points with no neighbors are left alone, and invalid
topology passes straight through.""",
        "wiring": [
            # rigEvaluator.cpp:3544 a mover with no rigExec:moves is skipped as a
            # grouping scope (it would silently do nothing), :3779 the target must
            # be a native UsdGeomPointBased point3f[] points property, and :3810
            # a smooth mover must have exactly one such target in v0.1.
            ("`rigExec:moves`", "Exact points property to relax.", "yes"),
        ],
        "param_groups": [
            ("Common mover envelope", "RigExecMoverAPI"),
        ],
        "example": """A 13×9 sheet is authored crumpled: an egg-crate quilt riding on a
broad swell. The quilt's period is exactly four grid steps in both axes,
which is the one frequency a uniform Laplacian annihilates — its
eigenvalue is (cos π/2 + cos π/2)/2 = 0 — while the swell's is
(cos π/12 + cos π/8)/2 = 0.9449. So as the mover's `inputs:defaultWeight`
spline ramps 0 → 1 → 0 the single step irons the quilt away to
*nothing* and lets it crumple back, while the swell keeps 94.5% of its
height: the sheet's centre goes 0.610 → 0.425, which is 94.5% of the
0.45 swell alone. The frequency split, made exact.
The border ring lifts as it relaxes — its mid-edge point rises 0.16 →
0.04 above the flat and pulls 0.17 inward — because boundary points have
fewer neighbors to average.""",
        "tips": [
            "Chain smooth after lattice or blendshape passes to settle their "
            "high frequencies.",
            "The mover has no parameters of its own: drive strength with the "
            "envelope or a weight object.",
            "Adjacency is the destination's own topology, so open borders drift "
            "inward toward their neighbors; bind a per-point weight object "
            "holding the border at 0 to pass those points through untouched.",
        ],
        "see_also": [
            ("lattice_mover", "Lattice Mover"),
            ("volume_correct_mover", "Volume Correct Mover"),
            ("blendshape_mover", "Blendshape Mover"),
        ],
    },
    "volume_correct_mover": {
        "title": "Volume Correct Mover",
        "schema": "RigExecVolumeCorrectMover",
        "summary": "Pulls a deformation back toward its rest bound volume.",
        "description": """Volume preservation as a post-pass: after a bulge, stretch, or squash
deforms the points, this mover pulls them back toward the rest bound
volume so bellies don't gain girth they shouldn't. It reacts to whatever
ran before it in the chain — it has no driver of its own.""",
        "how_it_works": """The mover compares the incoming (already deformed) points against
the rest bound volume and revises them toward it, mixed through the
common envelope. Nest it outside the deformer it corrects (deepest runs
first) so the correction sees the full deformation.""",
        "wiring": [
            ("`rigExec:moves`", "Exact points property to correct.", "yes"),
        ],
        "param_groups": [
            ("Common mover envelope", "RigExecMoverAPI"),
        ],
        "example": """One cage bulges two slabs side by side: the grey slab shows the raw
lattice bulge while the orange slab runs the same bulge through the
corrector, so the pair shows exactly what the correction takes away.""",
        "tips": [
            "Partial envelopes (0.3–0.6) usually look more organic than full "
            "correction.",
            "Order matters: the corrector must run after the deformer it "
            "tames — nest the deformer inside it.",
        ],
        "see_also": [
            ("lattice_mover", "Lattice Mover"),
            ("smooth_mover", "Smooth Mover"),
            ("matrix_mover", "Matrix Mover"),
        ],
    },
    "curvenet": {
        "title": "Curvenet",
        "schema": "RigExecCurvenet",
        "summary": "A net of cubic profile curves that articulates a surface "
                   "independently of its tessellation.",
        "description": """The rigging primitive of de Goes, Sheffler & Fleischer,
*Character Articulation through Profile Curves* (SIGGRAPH 2022): instead of
painting influences per vertex, a rigger traces a handful of profile curves
over the form — rings around a limb, rails along it, creases around a mouth —
and articulates those.""",
        "how_it_works": """A curvenet authors no behavior of its own — it is
geometry that other nodes read. Its `points` pool is posed like any other
points array, in the geometry phase, by ordinary movers writing
`Net.points`; the compiler puts that chain ahead of every chain that reads
the net, so a Profile Mover always sees the finished pool
(`libs/rigExec/rigEvaluator.cpp:6817-6821`, `libs/rigExec/rigEvaluator.cpp:12083-12090`).
Everything structural is *derived* from `rigExec:splineIndices` and never
authored: `RigExecBuildCurvenetTopology` classifies an endpoint shared by
three or more splines as an intersection and one incident to a single spline
as an anchor, and chains the splines between them into curves
(`libs/rigExecMath/curvenet.cpp:305-333`, called from
`libs/rigExecMath/curvenetAdjustments.cpp:195` and
`libs/rigExec/curvenetWeightComputations.cpp:28-31`); the orientation and
non-uniform scale along every curve are then derived too, the Profile Mover's
bind re-orienting each intersection fan against the projection surface's
normals (`libs/rigExecMath/profileMover.cpp:58-70`). Readers take
`rigExec:basis` and `rigExec:samplesPerSpline` off the net prim itself
(`libs/rigExec/moverGraph.cpp:2120-2126`; the adjuster reads the basis the
same way at `libs/rigExec/curvenetAdjuster.cpp:102-104`).""",
        "wiring": [
            # moverGraph.cpp:2112,2138: an empty pool makes the Profile Mover's
            # parameters invalid, which is a MoverFailed pass-through; the
            # adjuster's copy of the same check (curvenetAdjuster.cpp:97,127) is
            # promoted to a compile error at rigEvaluator.cpp:3825.
            ("`points`", "Inherited control-point pool in the projection pose: "
             "knots and tangent handles in one array.", "yes"),
            # moverGraph.cpp:2138 (empty splineIndices -> invalid params);
            # curvenetAdjuster.cpp:100,127 + rigEvaluator.cpp:3825 (compile error).
            ("`rigExec:splineIndices`", "Four pool indices per cubic spline; for "
             "the bezier basis they are p0, h0, h1, p1.", "yes"),
            # rigEvaluator.cpp:3825 -> curvenetAdjuster.cpp:113: an adjustment
            # whose rigExec:curvenet is not exactly this net fails compilation.
            ("(read by)", "A Curvenet Mover's `rigExec:curvenet`, a Curvenet "
             "Adjustment's `rigExec:curvenet`, and a Curvenet Weight's "
             "`rigExec:curvenetPoints` / `rigExec:curvenetSplineIndices`.", "-"),
            # moverGraph.cpp:1481 binds the net's points; moverGraph.cpp:2147-2150
            # + rigEvaluator.cpp:12435-12444: the posed pool arrives from the net's
            # own chain when it has one, from the authored value otherwise.
            ("(posed by)", "Any mover whose `rigExec:moves` names this net's "
             "`points` — a matrix mover, a curve mover, an adjuster.", "-"),
        ],
        "param_groups": [],
        "example": """A tube of 120 vertices profiled by three rings and four
rails — 28 cubic splines over 76 pooled control points, none of which mentions
a tube vertex. An FK-driven matrix mover bends the upper pool with a painted
field, a Curvenet Adjustment (`RingPush`, on pool knot 4) pushes one
middle-ring knot straight out through the Adjuster Mover, and the Profile
Mover carries both onto the surface under a `RigExecCurvenetWeight` envelope
painted on the same 76 pool points, which pins the tube's base row.
In the picture: the cyan wire cage is the rest pose, the green cables floating
clear of it are the posed net, the orange ring is the Bend control you animate,
and the small yellow diamond off to the side is `RingPush` on pool knot 4 —
the bulge in the silhouette under it is the Adjuster Mover's output reaching
the surface through the net.""",
        "tips": [
            "Index sharing is the whole connectivity model: give two splines the "
            "same pool entry and they join. Valence is then derived — three or "
            "more incident splines make an intersection, one makes an anchor — "
            "so there is nothing else to declare, and no normal, twist or tangent "
            "frame is ever authored.",
            "Readers take the basis off the net prim, but `RigExecCurvenetWeight` "
            "carries its own `rigExec:basis`, which defaults to `catmullRom` "
            "while the net defaults to `bezier`. Author it to match the net, or "
            "connect it to the net's attribute, or the parametrization is built "
            "against a different curve than the deformation.",
            "Pose the pool with the rig you already have. `points` is an exact "
            "native `point3f[]`, so a matrix mover plus a weight object works "
            "exactly as it does on a mesh, and the evaluator hands the Profile "
            "Mover the result of the net's own chain rather than the authored "
            "value.",
        ],
        "see_also": [
            ("curvenet_mover", "Curvenet Mover"),
            ("curvenet_adjustment", "Curvenet Adjustment"),
            ("curvenet_adjuster_mover", "Curvenet Adjuster Mover"),
        ],
    },
    "curvenet_adjustment": {
        "title": "Curvenet Adjustment",
        "schema": "RigExecCurvenetAdjustment",
        "example_key": "curvenet",
        "summary": "A handle on one curvenet knot, posed in the deformed frame.",
        "description": """A curvenet adjustment is an ordinary animator control — the same
avar channels as a `RigExecControl` — bound to one entry of a curvenet's
shared control-point pool. It is how a face rig tweaks a profile curve on
top of whatever already moved it: the handle's local delta is applied in a
frame deduced from the curvenet's own deformed shape, so the same key reads
as "lift this knot away from the surface" whether the head is at rest or
mid-turn. Nothing places the control: there is no rest position to author
and no offset to keep in sync with the net, because the frame is recomputed
from the incoming points every evaluation.""",
        "how_it_works": """The adjustment is not evaluated in the pose phase at all — it is read
in the geometry (mover-graph) phase by the `RigExecCurvenetAdjusterMover`
whose `rigExec:adjustments` names it, while that mover revises the curvenet's
own `points`. The adjuster reads the prim's `rest:*`/`default:*`/`avars:*`
channels and space matrices into one local matrix, and the kernel builds a
deformation-relative frame per pool point by sampling the rest and incoming
nets at 16 intervals per spline: an intersection knot takes a best-fit
rotation from its incident tangents, and knots along a curve take the
neighbouring intersections' rotations transported along the curve and slerped
by the sample's fractional arc length between them. The local matrix is then
conjugated by that frame and applied to the knot (and, with `rigExec:includeTangents`, its incident Bezier
handles), the mover envelope blends the result over the incoming revision, and
the adjusted frames are published with the evaluated geometry as control
frames in asset space — which is what draws the handle's guide and what the
usdview manipulator edits.""",
        "wiring": [
            # curvenetAdjuster.cpp:111-113 - exactly one target and it must be the
            # net the adjuster mover writes, or the mover parameters stay invalid
            # and the compile fails (rigEvaluator.cpp:3823-3827).
            ("`rigExec:curvenet`", "The RigExecCurvenet this handle adjusts; must be the "
             "same net the adjuster mover targets.", "yes"),
            # curvenetAdjuster.cpp:105-106 + :78-80 - the mover collects each
            # listed prim and every RigExecCurvenetAdjustment beneath it, and :110
            # rejects a listed prim that is not itself an adjustment (rigBuilder.cpp
            # :2350-2351 refuses to author such a target). Listing a Scope of
            # adjustments therefore fails the compile; listing a knot adjustment
            # picks up its tangent children. An adjustment no mover reaches is
            # simply never evaluated.
            ("(listed by the adjuster)", "The RigExecCurvenetAdjusterMover's "
             "`rigExec:adjustments` names this prim itself, or -- for a tangent -- "
             "the knot adjustment it is a child of.", "yes"),
            # curvenetAdjuster.cpp:115 reads it; curvenetAdjustments.cpp:204-205
            # rejects <0 (the schema fallback), out of range and duplicates, and
            # :220-221 rejects a pool entry that is not a curve endpoint.
            ("`rigExec:knotIndex`", "Index into the curvenet's shared `points` pool — the "
             "knot this handle moves, or, on a tangent child, the handle entry.", "yes"),
            # curvenetAdjuster.cpp:117-122 - default "knot"; any token other than
            # "knot"/"tangent" invalidates the mover.
            ("`rigExec:pointKind`", "\"knot\" (default) or \"tangent\".", "no"),
            # curvenetAdjuster.cpp:118-121 requires the namespace parent to be an
            # already-collected adjustment; curvenetAdjustments.cpp:213-216 requires
            # that parent to be a knot command whose incident handles include this
            # point index.
            ("(namespace parent)", "A `pointKind = \"tangent\"` adjustment must be a child "
             "of the knot adjustment whose Bezier handle it names.", "yes, for tangents"),
            # curvenetAdjuster.cpp:116 reads it; curvenetAdjustments.cpp:222-225
            # adds the knot's incident handles to the affected set when true.
            ("`rigExec:includeTangents`", "Carry the knot's incident Bezier handles with it "
             "(default true).", "no"),
        ],
        "param_groups": [
            ("Control channel", "RigExecControl"),
            ("Transform provider", "RigExecXformable"),
        ],
        "example": """The shared curvenet stage profiles its tube with a small net and poses
that net with ordinary rig machinery; `RingPush` is the extra handle layered on
top, bound to one knot of the middle profile ring through `rigExec:curvenet`
and `rigExec:knotIndex = 4`. Watch the **diamond** on the right of the net, not
the ring on the tube: the chip in the corner names `Bend`, the FK control that
swings the whole thing, while the knot the page is about is the one the diamond
rides. The same key is authored twice with the same value — `avars:tx = 1.8` at
frame 1005 and again at 1022 — first with the rig at rest, then under a bend
held at 45 degrees from 1018 to 1030, and the knot leaves the surface the same
way both times because the adjuster rebuilds its frame from the incoming net.
Between the two pushes (1010-1018) only the bend moves, so the two deltas can be
told apart. The adjuster mover writes that knot and its incident Bezier handles
into the net's points, and the Profile Mover carries the tube along.""",
        "tips": [
            "An adjustment's frame is an output of the point graph, so nothing in "
            "the pose phase may read it: naming one in a solver's "
            "`rigExec:controls`, a constraint's `rigExec:sources`, a matrix mover's "
            "transform provider — or nesting a RigExecControl under it, which would "
            "reach it through the default-space namespace fallback — is a compile "
            "error. Connecting a single avar scalar from it stays legal.",
            "On a tangent child, `rigExec:knotIndex` names the HANDLE's pool entry, "
            "not the knot's, and it must be one of the parent knot's incident "
            "handles; the tangent's delta then applies in the parent's already "
            "adjusted frame.",
            "`rigExec:includeTangents` only does anything on a `bezier` net — a "
            "`catmullRom` net has no separate handles, so a knot adjustment moves "
            "only its own pool entry and tangent children cannot be bound at all.",
        ],
        "see_also": [
            ("curvenet", "Curvenet"),
            ("curvenet_adjuster_mover", "Curvenet Adjuster Mover"),
            ("control", "Control"),
        ],
    },
    "curvenet_adjuster_mover": {
        "title": "Curvenet Adjuster Mover",
        "schema": "RigExecCurvenetAdjusterMover",
        "summary": "Applies knot and tangent controls in the frame of the already-deformed net.",
        "description": """The animation-facing half of the curvenet technique (2023 talk): a
deformer that writes the curvenet's *own* `points`, so an animator can push a
knot after every earlier deformer has fired. Each listed
`RigExecCurvenetAdjustment` contributes a local translate/rotate/scale delta
that is interpreted in a frame computed from the incoming deformation, not in
asset space — the same dial means "out along the curve" whether the net is at
rest or fully bent. Tangent adjustments parented under a knot control come
along automatically, so the relationship only ever names the knot controls.""",
        "how_it_works": """The adjuster is a revision in the point graph, on the curvenet's own
`points` property, so it runs after the pose pass and after whatever movers
precede it in the chain. It reads the net's authored (default-time) `points`
as the rest pose plus `rigExec:splineIndices` and `rigExec:basis`, samples
both the rest and the incoming configurations, and builds one frame per
control point: a best-fit rotation of the incident tangents at every
intersection, parallel transport elsewhere, and between two intersections a
slerp weighted by inverse arc distance. Each adjustment's local channel
matrix is then conjugated into its frame and applied to the knot (and, with
`rigExec:includeTangents`, its incident Bezier handles); the adjusted points
go through the common mover envelope, and the adjusted frames are published
per control prim in asset space for guides and viewport manipulation.""",
        "wiring": [
            # required, and must be exactly one native points attribute:
            # rigEvaluator.cpp:3777 (points-target set), :3814 (exactly one target
            # in v0.1); the owner must be typed RigExecCurvenet:
            # curvenetAdjuster.cpp:97.
            ("`rigExec:moves`", "The RigExecCurvenet's own exact `points` property — exactly one.", "yes"),
            # empty list -> parameters stay invalid (curvenetAdjuster.cpp:106) ->
            # RigExecValidateCurvenetAdjuster fails the compile
            # (curvenetAdjuster.cpp:141-144, called from rigEvaluator.cpp:3825).
            ("`rigExec:adjustments`", "The knot RigExecCurvenetAdjustment controls; their tangent children are collected automatically (curvenetAdjuster.cpp:70-85).", "yes"),
            # each adjustment must name this same net, or assembly bails:
            # curvenetAdjuster.cpp:111-113.
            ("`rigExec:curvenet` (on each adjustment)", "The same net the mover targets; a knot naming a different net fails the compile.", "yes"),
            # weights ? *weights : Constant(inputs:defaultWeight) --
            # curvenetAdjuster.cpp:93-94; the wrapper resolves the envelope per
            # point and blends against the preceding revision
            # (moverGraph.cpp:905-912), and the published frames are weighted
            # per control point at moverGraph.cpp:822-827.
            ("`rigExec:weightObject`", "Optional per-point envelope field; without one `inputs:defaultWeight` broadcasts.", "no"),
        ],
        "param_groups": [
            ("Common mover envelope", "RigExecMoverAPI"),
        ],
        "example": """This page has its own stage, and the loop plays three beats, one
motion at a time. First the rig is at rest and `RingPush` spends
`avars:tx = 1.8` on pool knot 8: the knot leaves along world **+X** and the
Profile Mover carries the surface with it. Then the push comes off and the
`Bend` control alone swings the net 45 degrees — that bulge is the *Matrix
Mover's* work, not the adjuster's. Then, inside the held bend, the same
`avars:tx = 1.8` fires again: the knot travels the same 1.8 units, but now
**along the bent net normal**, about 46 degrees off where it went the first
time (knot 8 carries `NetBend` weight 1.0, so its frame rides the whole
bend). Same dial, same number, a frame that moved — which is the sentence at
the top of this page, on screen.""",
        "tips": [
            "Chain order is the whole point: the adjuster belongs last on the net's "
            "points so its frames follow earlier deformation. In "
            "`tests/testRigExecCurvenetAdjuster.cpp:193-209` a 90 degree warp ahead "
            "of it turns a knot's `avars:tx` into motion along world +Y.",
            "Tangent children need `rigExec:basis = \"bezier\"`: a Catmull-Rom net "
            "has no separate handles, so `rigExec:includeTangents` moves nothing and "
            "a tangent adjustment fails the mover outright "
            "(rigExecMath/curvenetAdjustments.cpp:115-123, 213-218).",
            "Bindings fail atomically and at compile time — a duplicate knot index, "
            "a knot of valence 0, a tangent that is not a direct child of its knot, "
            "or a nonfinite transform is a compile error, and `inputs:enabled = "
            "false` does not excuse it: validation re-reads the mover with enabled "
            "forced true (curvenetAdjuster.cpp:119-121, 134-147; "
            "rigExecMath/curvenetAdjustments.cpp:204-221).",
        ],
        "see_also": [
            ("curvenet", "Curvenet"),
            ("curvenet_adjustment", "Curvenet Adjustment"),
            ("curvenet_mover", "Curvenet Mover"),
        ],
    },
    "curvenet_mover": {
        "title": "Curvenet Mover",
        "schema": "RigExecCurvenetMover",
        "example_key": "curvenet",
        "summary": "The Profile Mover: propagates a posed curvenet onto a surface.",
        "description": """The deformer half of curvenet rigging (de Goes, Sheffler & Fleischer,
*Character Articulation through Profile Curves*, SIGGRAPH 2022): a net of
profile splines is articulated like any other geometry, and this mover
carries that articulation onto one mesh's `points`. The surface reproduces
the curves while keeping its own detail, and because the mesh is cut along
the net, each side of a curve deforms independently — a crease can fold
without dragging the other side with it. Nothing in the wiring mentions the
target's tessellation, so re-meshing the surface only re-cuts and re-binds:
no wiring changes.""",
        "how_it_works": """It runs as one revision of the target's point chain, after the solve
and after every earlier revision on that chain; same-target movers are
ordered by the reverse-sibling post-order walk of `<rig>/Movers`, so
descendants run before their mover parent and sibling branches run
bottom-to-top in usdview (`libs/rigExec/rigEvaluator.cpp:149-163, 3531-3533`).
Every frame it reads the *projection pose* — the curvenet's `points`,
`rigExec:splineIndices`, `rigExec:basis` and `rigExec:samplesPerSpline`,
plus the target's `faceVertexCounts`, `faceVertexIndices` and `points`, all
at **default** time (`libs/rigExec/moverGraph.cpp:2109-2137`) — and hashes
them into a bind digest; the expensive half, cutting the mesh along the net
and factorizing the cut-aware Laplacian, is done once and cached under that
digest (`libs/rigExecMath/profileMover.h:1-11`,
`libs/rigExec/moverGraph.cpp:2167-2199`). It then reads the net's *posed*
points — the result of the curvenet's own mover chain, which the evaluator
guarantees has already run by recording the net's points as a dependency
edge (`libs/rigExec/rigEvaluator.cpp:6821`) — harmonically interpolates the
per-side deformation gradients over the cut mesh, and Poisson-reconstructs
vertex positions from the incoming point revision, which is the surface it
deforms FROM (`libs/rigExec/moverGraph.cpp:830-848`). The solved points are
then blended over that incoming revision by the common MoverAPI envelope
(`libs/rigExec/moverGraph.cpp:888-913`) and written back to the target's
`points`.""",
        "wiring": [
            # Not rejected at compile: an absent or unresolvable net leaves the
            # packet invalid, which is the MoverFailed pass-through
            # (libs/rigExec/moverGraph.cpp:2106, 2138-2141).
            ("`rigExec:curvenet`", "The `RigExecCurvenet` supplying the control "
             "points; the first target is the one used. Omitting it is not a "
             "compile error — the mover then fails and passes its incoming "
             "points through unchanged.", "yes"),
            # Exactly one target, and it must be a native point3f[] `points`
            # attribute on a UsdGeomPointBased
            # (libs/rigExec/rigEvaluator.cpp:3770-3802 native points target,
            # 3810-3823 exactly one target for RigExecCurvenetMover).
            ("`rigExec:moves`", "Exactly one exact native `point3f[] points` "
             "property — the surface this deforms. Multi-target fan-out is "
             "rejected at compile.", "yes"),
            # Common envelope, at most one target
            # (libs/rigExecSchema/schema.usda:79-88; rel rigExec:curvenet at 2307).
            ("`rigExec:weightObject`", "Optional weight field over the target's "
             "points, supplying the envelope instead of `inputs:defaultWeight`.",
             "no"),
        ],
        "param_groups": [
            ("Common mover envelope", "RigExecMoverAPI"),
            ("Curvenet source (read from `rigExec:curvenet`)", "RigExecCurvenet"),
        ],
        "example": """Shares the Curvenet page's stage: a profile net drawn over a
144-quad capped tube, its knots posed by ordinary rig machinery — a matrix
mover under an FK-driven joint, then a Curvenet Adjustment through the
Adjuster Mover — and `ProfileMover` propagating that posed net onto
`Tube.points` under a `RigExecCurvenetWeight` envelope. In the picture the
**green** curves are the posed net, the **cyan** wireframe is the rest pose
the whole thing departs from, and the small **yellow diamond** off the
middle ring is the `RingPush` knot handle. The wide swing is the bend; the
local lobe pushed out beside that diamond is *one* knot moved through the
Adjuster, and the surface reproducing it — with the rest of the tube left
alone — is the thing a skin cluster cannot do. The net's 76 pooled control
points are the only thing the rig names; the tube's 168 vertices appear in
no relationship anywhere — the whole point of the representation.""",
        "tips": [
            "The projection pose is read at **default** time on both the net and "
            "the target, never at the evaluated frame: author the drawn pose as "
            "the default value and keep animation in time samples. A target whose "
            "`points` exist only as time samples binds nothing and passes "
            "through.",
            "Everything the cut depends on — `rigExec:basis`, "
            "`rigExec:samplesPerSpline`, `rigExec:splineIndices`, the net's "
            "default points and the target's default points and topology — is "
            "hashed into the bind key, so editing any of it re-cuts the mesh and "
            "re-factorizes. Animating the net's posed points changes none of "
            "those, which is why every frame after the first is cheap.",
            "The incoming point revision is the surface it deforms from, so a "
            "curvenet stacked after skinning, after another curvenet or over "
            "simulated points needs no extra setup — put the mover later in the "
            "chain and it layers.",
        ],
        "see_also": [
            ("curvenet", "Curvenet"),
            ("curvenet_adjuster_mover", "Curvenet Adjuster Mover"),
            ("matrix_mover", "Matrix Mover"),
        ],
    },
    "blend_input": {
        "title": "Blend Input",
        "schema": "RigExecBlendInput",
        "summary": "One weighted channel of sculpted targets inside a blendshape pass.",
        "description": """A single channel — a smile, a brow raise, a phoneme. It carries one
animated `inputs:weight` and an ordered list of target samples; the
blendshape mover sums every bound channel into the final delta. Channels
are independent, so weights layer without interfering.""",
        "how_it_works": """The channel evaluates its samples against the weight: below the
first activation the delta fades in, between samples it interpolates,
past the last it holds (or extrapolates, per the mover). The result is
one delta array the mover adds to its sum.""",
        "wiring": [
            ("`rigExec:samples`", "Ordered target samples for this channel.", "yes"),
            ("(bound by)", "A blendshape mover's `rigExec:blendInputs` sums this channel.", "-"),
        ],
        "example": """One channel, one target: the Smirk weight sweeps 0 → 1 → 0 and the
card's right column slides sideways with it — the smallest possible
blend channel.""",
        "tips": [
            "One input per facial action keeps weights directable; combine "
            "them in the mover, not in the sculpts.",
            "Nest samples under their input so the channel reads as one unit "
            "in the graph.",
        ],
        "see_also": [
            ("blendshape_mover", "Blendshape Mover"),
            ("blend_sample", "Blend Sample"),
            ("float_math_mover", "Float Math Mover"),
        ],
    },
    "blend_sample": {
        "title": "Blend Sample",
        "schema": "RigExecBlendSample",
        "summary": "One sculpted target at a fixed channel activation.",
        "description": """A single pose on a channel's travel: full targets at activation 1,
in-betweens anywhere between. In-betweens shape the path — corners that
widen before they lift, lids that lag then catch up — instead of the
straight line one target would give.""",
        "how_it_works": """Each sample names its sculpt through `rigExec:targetPoints` and its
position on the channel through `rigExec:activation`. The channel
interpolates between neighboring samples by weight, so a 0.5 sample is
the pose the channel passes through halfway up.""",
        "wiring": [
            ("`rigExec:targetPoints`", "Native points prim holding the sculpt.", "yes"),
            ("(listed by)", "The parent blend input's `rigExec:samples`.", "-"),
        ],
        "example": """One channel with two samples: at activation 0.5 the card shifts
right, at 1.0 it sits right and up, so sweeping the weight draws a
curved path instead of a straight slide.""",
        "tips": [
            "Activations need not be uniform: cluster in-betweens where the "
            "path curves hardest.",
            "Keep every target's point count identical to the moved mesh — "
            "topology must match exactly.",
        ],
        "see_also": [
            ("blend_input", "Blend Input"),
            ("blendshape_mover", "Blendshape Mover"),
            ("static_weight", "Static Weight"),
        ],
    },
    "pose_interpolator": {
        "title": "Pose Interpolator",
        "schema": "RigExecPoseInterpolator",
        "summary": "Turns a driver's rotation into one float per authored pose.",
        "description": """A pose interpolator is pose-space deformation's measuring device: one
driver joint in, one weight per authored `RigExecPose` child out. Each pose
records where the driver stands — a quaternion measured in the driver's own
parent frame, relative to the driver's rest — plus how wide its influence
reaches, and the interpolator publishes each pose's share of wherever the
driver is now. It writes no transform and no points, which is why it lives
in its own scope rather than under `Movers`; what makes it useful is that a
`RigExecBlendInput` connects its `inputs:weight` to a pose's
`outputs:weight`, so a corrective shape can be retargeted or layered without
touching the interpolator that fires it.""",
        "how_it_works": """The maths is a radial basis function (`libs/rigExecMath/rbf.h`). Compile
reads the driver, the kernel, the channel switches and every enabled pose's rotation,
type and radius, and inverts the kernel matrix once — it is a pure function
of the authored poses, so nothing on the prim stores it. Evaluation happens
in its own phase, after the whole pose walk and before the geometry chains
(`rigEvaluator.cpp:11990-11998`): it takes the driver's final and rest
frames, forms `delta = restLocal⁻¹ · (parent⁻¹ · world)`
(`rigEvaluator.cpp:3073-3076`), runs one kernel row through the inverted
matrix, and publishes a `float` on each pose's `outputs:weight` into both the
resolved-input table a consumer reads through and the pose's moved-property
map (`rigEvaluator.cpp:3019-3026`). A driver with no published final frame
is diagnosed and the interpolator publishes zeros rather than quietly
measuring a rest (`rigEvaluator.cpp:3048-3056`).""",
        "wiring": [
            # rigEvaluator.cpp:2794 -- driverTargets.size() != 1 is a compile error
            ("`rigExec:driver`", "Exactly one `RigExecJoint` or `RigExecControl` "
             "whose local rotation every pose is measured against.", "yes"),
            # rigEvaluator.cpp:2929-2932 -- "has no RigExecPose children" is a compile
            # error; rigEvaluator.cpp:2866-2874 -- any other child type is an error too
            ("`RigExecPose` children", "One prim per pose, directly under the "
             "interpolator; any other child type is a compile error.", "yes"),
            # rigEvaluator.cpp:2878-2886 -- an authored connection ON outputs:weight is
            # an error; the consumer side connects INTO it (testRigExecPoseInterpolator
            # .cpp:172-178) and nothing requires that a consumer exist
            ("`<pose>.outputs:weight`", "Read by a consumer — a "
             "`RigExecBlendInput`'s `inputs:weight` connects to it. The pose's own "
             "`outputs:weight` must carry no authored connection.", "-"),
            # schema.usda RigExecPose.rigExec:poseControls -- "nothing in evaluation
            # reads it, and a pose with none is still a valid pose"
            ("`rigExec:poseControls`", "Authoring provenance on each pose: the "
             "control properties that put the rig into it. Never read at "
             "evaluation.", "no"),
        ],
        "param_groups": [
            ("Pose", "RigExecPose"),
        ],
        "example": """An Elbow control closes a skinned 30-quad strip to 40 degrees, holds
there, closes to 80, holds again and opens; two matrix movers skin the strip
to the Upper and Fore joints. The interpolator watches the **Fore joint** —
not the Elbow control the animator keys — and carries three poses, `neutral`
at identity, `half` at 40 degrees and `bent` at 80, each 0.7 radians wide,
which is the spacing between them. Watch the three floats trade: at rest
`neutral` reads 1.000 and `half` and `bent` 0.000; a third of the way in they
split 0.529 / 0.601 / 0.000; on the first hold `half` reads 1.000 alone; on
the second `bent` reads 1.000 and the others 0.000. Only `bent.outputs:weight`
is connected to the blend channel, so the corrective swells on the outside of
the elbow exactly when that one float does — nothing at the `half` hold, full
at the `bent` hold.""",
        "tips": [
            "Author a non-zero `rigExec:rotationRadius` on every pose, roughly the "
            "angle between neighbouring poses. A per-pose zero is used literally "
            "rather than fitted (`rbf.cpp:992-1001`, `rbf.cpp:407-413`), and an "
            "interpolator whose radii are all zero reads 1 only when the driver "
            "stands exactly on a pose and 0 everywhere else.",
            "Drive from a joint whose parent already carries the motion you do not "
            "want measured: the phase measures the driver's rotation relative to "
            "its rest in its nearest frame-publishing ancestor's frame, so the "
            "parent subtracts its own share back out. Measuring against anything "
            "other than the immediate namespace parent is reported as a warning.",
            "`rigExec:enableTranslation` is not measured by this phase — it warns "
            "and judges the poses on rotation alone "
            "(`rigEvaluator.cpp:2852-2862`). Disabling a whole interpolator "
            "publishes zeros, while disabling one pose drops it out of the solve "
            "entirely so the other poses' weights change.",
            "A radius wider than the pose spacing makes the Gaussian rows overlap "
            "enough that the inverse pushes a far pose negative between two near "
            "ones. `rigExec:allowNegativeWeights = 0` clamps that lobe; leave it on "
            "only when a consumer wants the extrapolation.",
        ],
        "see_also": [
            ("blend_input", "Blend Input"),
            ("blendshape_mover", "Blend Shape Mover"),
            ("joint", "Joint"),
        ],
    },
    "pose": {
        "title": "Pose",
        "schema": "RigExecPose",
        "example_key": "pose_interpolator",
        "summary": "One place the driver can be, and the float it publishes when the driver gets there.",
        "description": """A pose is a single authored sample of its interpolator's pose space:
where the driver stands, how wide that pose's influence reaches, and the
`outputs:weight` a blend channel connects to. Its weight is 1 when the
driver stands exactly on the pose and, after the interpolator's
normalization, a share of the total elsewhere — negative where the solve
says lean *away* from this pose. Poses are prims parented under their
interpolator rather than parallel arrays on it, so a layer can retune one
pose's radius, or switch it off, without restating the other seven.""",
        "how_it_works": """Six of the pose's attributes are read at compile time, when the
interpolator builds and inverts its RBF matrix once: `rigExec:rotation`
(the driver's local rotation relative to its own rest, in the driver's
parent frame), `rigExec:translation`, `rigExec:poseType`, the two radii,
and `inputs:enabled` all feed that solve, and those same six are the
pose's share of the epoch digest, so editing any of them recompiles the
interpolator. The other three — `rigExec:falloff`,
`rigExec:poseControls` and `rigExec:poseControlValues` — are provenance
that evaluation never reads, and the translation pair is dropped unless
the interpolator sets `rigExec:enableTranslation`, which the evaluation
phase warns it does not measure. Evaluation happens in the pose-interpolator
phase — after the complete pose walk, every constraint included, and
before the geometry chains that consume the weights — where the
interpolator measures its driver's final local rotation against every
pose and publishes one float per pose into both the resolved-input map a
consumer reads through and the rig pose's `movedProperties` map. The metric is
the pose's own: a `swing` or `twist` pose is judged on that half of the
driver's rotation, split about the interpolator's `rigExec:twistAxis`,
while a `whole` pose is judged on the entire rotation.""",
        "wiring": [
            # Poses are reached ONLY as children of an interpolator
            # (rigEvaluator.cpp:986-1001 discovers interpolators; 2866-2873 walks
            # their children), any non-RigExecPose child is a compile error
            # (rigEvaluator.cpp:2867-2874), and an interpolator with no pose
            # children is a compile error (rigEvaluator.cpp:2929-2933).
            ("(child of)", "The `RigExecPoseInterpolator` that solves it. A pose "
             "anywhere else is never read, and a non-`RigExecPose` child of an "
             "interpolator is a compile error.", "yes"),
            # Authored only by rigBuilder.cpp:1674-1688; grepping libs/rigExec for
            # poseControls / poseControlValues returns no reader at all, and the
            # schema says so outright (schema.usda rigExec:poseControls).
            ("`rigExec:poseControls`", "The exact control *properties* that put "
             "the rig into this pose, parallel to `rigExec:poseControlValues`. "
             "Authoring data for a shape editor; nothing in evaluation reads it.",
             "no"),
            # Published by the interpolator (rigEvaluator.cpp:3020-3021, 3100);
            # an authored connection ON it is refused at compile
            # (rigEvaluator.cpp:2877-2887).
            ("`outputs:weight`", "Written by the interpolator every evaluation; a "
             "`RigExecBlendInput`'s `inputs:weight` connects *to* it. An authored "
             "connection on it is a compile error.", "-"),
        ],
        "param_groups": [
            ("Interpolator settings", "RigExecPoseInterpolator"),
        ],
        "example": """The `ElbowSwing` interpolator carries two poses — `neutral` at
identity and `bent` at 80 degrees about Z — and the bent pose's
`outputs:weight` is the only thing wired into the corrective blend
channel. Measured on the stage, the bent pose publishes 0.000 with the
arm straight, 0.423 at frame 1004 three frames into the bend, and 1.000
at frame 1008 when the elbow reaches the pose exactly.""",
        "tips": [
            "Author a non-zero `rigExec:rotationRadius` by hand. The evaluator "
            "hands the per-pose radii straight to the solver, and a width of "
            "zero makes every non-zero distance infinite, so the pose's kernel "
            "is dead: on a two-pose test rig the zero-radius pose published "
            "0.000 at every frame, including the one where the driver reached "
            "it.",
            "`inputs:enabled = false` drops the pose out of the solve entirely "
            "rather than solving it and silencing the result — the remaining "
            "poses re-solve as if it had never been authored — and the disabled "
            "pose publishes a hard zero whatever else the interpolator does.",
            "`rigExec:poseType` defaults to `swing`, which measures the driver's "
            "rotation with its twist about `rigExec:twistAxis` removed. Use "
            "`whole` when one pose should account for the entire rotation. A "
            "swing pose captured from a driver that only twists measures as "
            "identical to neutral, and an interpolator whose poses are all "
            "coincident under its enabled channels is reported degenerate and "
            "pegs every weight at 1/n.",
        ],
        "see_also": [
            ("pose_interpolator", "Pose Interpolator"),
            ("blend_input", "Blend Input"),
            ("blendshape_mover", "Blend Shape Mover"),
        ],
    },
    "static_weight": {
        "title": "Static Weight",
        "schema": "RigExecStaticWeight",
        "summary": "A painted, time-invariant weight field over moved points.",
        "description": """The paint layer of rigging: one scalar per moved point, authored once
and held for the shot. Constant representation broadcasts a single value
to a whole target; dense lists every element; sparse lists painted
indices over a default. Bound through a mover's `rigExec:weightObject`,
it scales that mover's effect per point.""",
        "how_it_works": """The field resolves inside the mover walk, in the pass that assembles
the revision consuming it: every target element gets an explicit value
(or the sparse default), and that array is published as the mover's
envelope in place of `inputs:defaultWeight` for that application. The
fields are time-invariant by contract — time samples or connections on
`rigExec:values`, `rigExec:indices`, `rigExec:defaultWeight`,
`rigExec:representation`, or `rigExec:rangePolicy` fail the pose — and
`strict` rejects a value outside [0, 1] where `clamp` bounds it.""",
        "wiring": [
            # rigEvaluator.cpp:874 "must have exactly one target"; :882 also
            # requires it to resolve to the binding mover's own target.
            ("`rigExec:weightTarget`", "Exact points property this field covers; "
             "must match the target of the mover that binds it.", "yes"),
            # rigEvaluator.cpp:12406 keys pose.weightFields by the mover's
            # BOUND weight object, so only a bound field is ever resolved.
            ("(bound by)", "A mover's `rigExec:weightObject` applies this field.",
             "-"),
        ],
        "param_groups": [
            ("Weight field", "RigExecWeightObject"),
        ],
        "example": """One joint yaws 0 → −32 → 0 degrees while a dense field over a
24 × 4 strip holds the root columns at 0 and ramps to 1 at the tip, so
the bend grows along the strip and the root never leaves its rest
position — paint, not animation, shaping the deformation. The sweep
stays in the ground plane, so the strip keeps its face to the camera and
the ramp reads the same at the extreme as it does at rest. The docs
renderer tints the strip by the field the mover actually consumed, grey
at weight 0 and red at weight 1, so the fixed ramp is visible while the
pose swings through it.""",
        "tips": [
            "Dense suits small ordered targets; sparse suits hero meshes "
            "where most verts sit at the default.",
            # rigEvaluator.cpp:7924 "dense weight cardinality mismatch";
            # :7930 "dense weight requires canonical defaultWeight 0".
            "A dense field must author exactly one value per target element and "
            "leave `rigExec:defaultWeight` at 0 — the canonical encoding — or "
            "the pose fails rather than padding.",
            "A constant field of 1.0 is the rigid-attachment idiom: full "
            "follow, no paint.",
        ],
        "see_also": [
            ("dynamic_weight", "Dynamic Weight"),
            ("matrix_mover", "Matrix Mover"),
            ("blendshape_mover", "Blendshape Mover"),
        ],
    },
    "dynamic_weight": {
        "title": "Dynamic Weight",
        "schema": "RigExecDynamicWeight",
        "summary": "The joint is frozen; only the painted field is animated.",
        "description": """Paint that moves: a dynamic weight takes a static base field and
reshapes it every frame from an animated driver — a muscle that engages
with effort, a corrective that fades with a pose, a knee influence that
breathes with the stride. Same target contract as the field it wraps: it
declares the same `rigExec:weightTarget` and is bound in the mover's
place of the field it modulates.""",
        "how_it_works": """The compiler binds the mover's `rigExec:weightObject` to one
`computeWeightPacket` for the whole epoch, and every generation
re-evaluates that packet from `inputs:driver`, `inputs:scale`, and
`inputs:bias` as ordinary dynamic inputs before the mover walk reads it
at the consuming revision. The base field resolves first — the
`rigExec:baseWeight` relationship pulls that object's own packet —
then each element becomes `(b × inputs:driver) × inputs:scale +
inputs:bias` — `inputs:scale` and `inputs:bias` apply *after* the driver
is combined in, not to the driver — and `rigExec:rangePolicy` either
rejects a result outside [0, 1] (`strict`) or clamps it (`clamp`). The
resolved envelope is handed to the mover and, when the influence overlay
is armed, published as the field a rigger is shown; nothing is written
back to the stage.""",
        "wiring": [
            # rigEvaluator.cpp:794 rejects more than one target; :799 makes the
            # relationship mandatory for every non-constant representation
            # ("a DynamicWeight without a base must be constant"), :7786/:7793
            # repeat both in the CPU resolver, and weightPackets.cpp:114-118 is
            # the same rule in the packet kernel exec and the baked path share
            # (moverKernels.cpp:238, bakedWeights.cpp:335).
            ("`rigExec:baseWeight`", "Weight field this one modulates; required "
             "unless the representation is `constant`, and at most one target.",
             "yes"),
            # rigEvaluator.cpp:874 "must have exactly one target"; :882 also
            # requires it to resolve to the binding mover's own target.
            ("`rigExec:weightTarget`", "Exact points property this field covers; "
             "must match the target of the mover that binds it.", "yes"),
            # rigEvaluator.cpp:12406 keys pose.weightFields by the mover's
            # BOUND weight object, so only a bound field is ever resolved.
            ("(bound by)", "A mover's `rigExec:weightObject` applies this field.",
             "-"),
        ],
        "param_groups": [
            ("Weight field", "RigExecWeightObject"),
        ],
        "example": """A 24 × 6 strip is skinned to one joint that holds a fixed 35-degree
bend, and the only animation in the file is `inputs:driver` sweeping
0.15 → 1 → 0.15. The base paint is a straight ramp from 0 at the
one-sixth mark to 1 at the tip, and with `inputs:scale = 1` the driver
simply scales that whole field at once: every painted point follows the
same fraction of the bend at every moment, so the grey-to-red ramp
brightens and dims — and the strip curls further and relaxes — while the
control and the joint never move. The docs renderer tints the strip by
the field the mover actually consumed — grey at weight 0, red at
weight 1 — so the ramp and the curl that follows it are the same
event.""",
        "tips": [
            # rigEvaluator.cpp:7883-7889: out-of-range only survives under
            # `clamp`; `strict` fails the pose with "strict range violation".
            "`rangePolicy: clamp` is what lets `inputs:scale` overdrive the "
            "paint: a result outside [0, 1] is clamped instead of failing the "
            "pose, so a driver can push part of a painted ramp to fully "
            "followed while the rest of it still fades.",
            # rigEvaluator.cpp:12406 publishes pose.weightFields under the
            # BOUND weight object; bridge.cpp:1521 looks the overlay up there.
            "The influence overlay paints the field a mover consumed, so point "
            "it at the dynamic weight, not the base it wraps: an unbound base "
            "has no resolved field of its own to show.",
            # rigEvaluator.cpp:776-786 validates at most one float connection per
            # input ("scalar input must have at most one connection"); and
            # rigEvaluator.cpp:10561-10570 runs the property chains BEFORE exec
            # and feeds their results to packet assembly, so a
            # RigExecFloatMathMover can move `.inputs:driver` the same way
            # float_math_mover.usda moves a blend input's `.inputs:weight`.
            "Drive the driver from another channel (a float math mover or a "
            "connection) to tie corrective strength to posing; each of driver, "
            "scale, and bias takes at most one float connection.",
        ],
        "see_also": [
            ("static_weight", "Static Weight"),
            ("matrix_mover", "Matrix Mover"),
            ("float_math_mover", "Float Math Mover"),
        ],
    },
    "sphere_weight": {
        "title": "Sphere Weight",
        "schema": "RigExecSphereWeight",
        "summary": "A ball of influence: radial falloff generated from a placed volume.",
        "description": """The weight nobody paints. A sphere weight is a placed volume that
*generates* a scalar field instead of storing one: every point of the
target gets the weight its distance from the volume's origin earns,
ramped between `inputs:falloffMin` (fully on) and `inputs:falloffMax`
(fully off). Because it is a `RigExecXformable`, it is one selectable,
framable prim positioned by the same avars as a control or joint — so a
sphere authored inside a joint rides that joint with nothing wired, and
the region a mover grabs can be animated by moving the ball. Per-axis
`inputs:scaleX/Y/Z` turn the iso-surfaces into ellipsoids.""",
        "how_it_works": """The prim publishes a weight packet that the bound mover consumes as
its envelope during the geometry (point-chain) phase. Each evaluation it
takes its own posed frame from `computePointFrame`, strips scale and
shear so the field matches the rigid guide that is drawn, divides local
coordinates by `inputs:scaleX/Y/Z`, and measures
`d = |(px/sx, py/sy, pz/sz)|` for every element of
`rigExec:weightTarget` — or of `rigExec:sampleSource`, when authored,
which changes *what is measured* without changing what is weighted.
`d` is normalized across the falloff band, exchanged end-for-end by
`inputs:invert`, remapped through the falloff lookup table, multiplied by
`inputs:strength`, and bounded by `rigExec:rangePolicy` (`clamp` by
default, so scrubbing strength saturates instead of invalidating the
rig). `rigExec:samplePhase` chooses the points measured: `reference`
(the default) uses the STATIC authored base points, so a point keeps
the weight its bind pose earned and a mover's own output cannot feed
back into its own weights; `current` re-measures the points as they
stand at that position in the mover stack. The band, invert and strength
are live per-frame inputs; `rigExec:falloffProfile` and `rigExec:falloffCurve` are
structural and are baked to one lookup table per binding epoch.""",
        "wiring": [
            # Exactly one target: libs/rigExec/rigEvaluator.cpp:872; and it must be
            # the same property the bound mover moves: rigEvaluator.cpp:882.
            ("`rigExec:weightTarget`", "Exact points property this generated field "
             "covers; must be the same target the bound mover moves.", "yes"),
            # Optional: an empty sampleSource simply falls back to the weightTarget
            # points (libs/rigExec/weightPackets.cpp:277,
            # libs/rigExec/rigEvaluator.cpp:7562), and only its cardinality is
            # checked when it is authored (weightPackets.cpp:269).
            ("`rigExec:sampleSource`", "Static points to measure against instead of "
             "the target — typically an unposed reference copy. The weighted "
             "domain stays the target.", "no"),
            # No relationship places the volume: the packet reads the prim's own
            # posed point frame (libs/rigExec/moverKernels.cpp:294), so an unwired
            # volume follows its namespace-parent xformable (schema.usda:1344).
            ("(placement)", "No wiring: the volume is posed by its own `rest:space` "
             "and avars, or follows its namespace-parent xformable when unwired.", "-"),
            # The packet is only ever consumed through a mover's weightObject;
            # nothing requires a volume weight to be bound at all.
            ("(bound by)", "A mover's `rigExec:weightObject` applies this field.", "-"),
        ],
        "param_groups": [
            ("Volume weight", "RigExecVolumeWeight"),
            ("Transform provider", "RigExecXformable"),
        ],
        "example": """A 24 x 14 quad plank is lifted by one matrix mover whose joint holds a
static one-unit rise — no animation on the deformation at all. The only
spline in the file slides the Probe control along the plank, and the
sphere authored inside the probe joint rides it, so the region the mover
grabs travels and a bump walks back and forth. The plank is wider than
the ball and the travel stops short of both ends, so the field never
runs off an edge: it stays a complete red disc ringed by grey. The two
concentric red wire rings are the band's own iso-surfaces — the inner
one is `inputs:falloffMin` 0.6, where the field is fully on and the
plank is lifted the whole unit, and the outer one is
`inputs:falloffMax` 1.6, where it is fully off; `linear` between them
so the ramp reads as an even slope rather than a plateau with an
edge.""",
        "tips": [
            "Author the volume *inside* the joint or control it should follow: an "
            "unwired xformable takes its namespace parent's posed space, so a ball "
            "of influence needs no constraint and no wiring.",
            "Size the ball with `inputs:falloffMin`/`inputs:falloffMax` and "
            "`inputs:scaleX/Y/Z`, never with a scale in the transform — the "
            "placement has its scale and shear removed before the field is "
            "built, so a scale inherited from the volume's parent changes "
            "neither the field nor the drawn guide and simply does nothing. A "
            "non-positive or non-finite axis scale invalidates the packet "
            "outright.",
            "`falloffMax` *below* `falloffMin` flips the ramp with no special case — "
            "that is the hole-instead-of-ball idiom. The two exactly equal is the "
            "one degenerate case and means a hard step at that radius.",
        ],
        "see_also": [
            ("static_weight", "Static Weight"),
            ("dynamic_weight", "Dynamic Weight"),
            ("matrix_mover", "Matrix Mover"),
        ],
    },
    "plane_weight": {
        "title": "Plane Weight",
        "schema": "RigExecPlaneWeight",
        "summary": "A half-space gradient: everything past the placed plane is weighted in.",
        "description": """A generated weight field whose distance is the **signed** local
coordinate along `rigExec:planeAxis`, so a band straddling zero authors a
gradient *across* the plane rather than a band mirrored on both sides of
it — "everything above this height", "everything past this line". The
plane is a `RigExecXformable`, so it is placed by the same avars a control
is and can be driven by one, and the drawn guide is two rectangles, one at
`inputs:falloffMin` and one at `inputs:falloffMax`. It is infinite by
default; `rigExec:planeBounds = "bounded"` clips the field to the
`inputs:extentU` x `inputs:extentV` rectangle, which is the difference
between a half-space and a patch.""",
        "how_it_works": """The packet is built in the geometry (point-chain) phase, as the
envelope of whichever mover binds it through `rigExec:weightObject`: the
volume's posed frame comes in as its `computePointFrame`, gets its scale
and shear removed so the field matches the rigid guide, and is inverted to
carry each sampled point into plane-local space. The weight is then the
shared remap of the signed axis coordinate `d` —
`u = clamp01((d - falloffMin) / (falloffMax - falloffMin))`, lerped by
`inputs:invert`, run through the baked `rigExec:falloffProfile` table as
`1 - u`, and scaled by `inputs:strength` — so the field is fully ON at
`falloffMin` and fully OFF at `falloffMax`. With `bounded`, a point outside
the in-plane rectangle gets exactly zero instead, with no edge ramp. Which
points are measured is `rigExec:samplePhase`: `reference` (the default)
measures the authored base points, `current` the points as they stand at
that mover's position in the stack.""",
        "wiring": [
            # rigEvaluator.cpp:874 ("rigExec:weightTarget must have exactly one
            # target") and :882 (must equal the bound mover's target).
            ("`rigExec:weightTarget`", "Exact points property this field covers; "
             "must match the target of the mover that binds it.", "yes"),
            # Nothing to author: computePointFrame is a .Required() input of the
            # packet computation (moverKernels.cpp:965-967), but every
            # RigExecXformable publishes one (computations.cpp:516), and an
            # unplaced volume simply resolves to identity
            # (rigEvaluator.cpp:9941-9948). rigEvaluator.cpp:7530-7532 is the CPU
            # oracle's "no resolved placement for this volume weight".
            ("(placement)", "No wiring: the volume's own posed frame, from its "
             "avars (optionally connected to a control's, as in the example) or "
             "from an xformable namespace parent it rides.", "-"),
            # rigEvaluator.cpp:7563 - sampleSource is tried first and falls back
            # to weightTarget, so authoring it is optional.
            ("`rigExec:sampleSource`", "Optional static points to measure against "
             "instead of the weighted domain; ignored when `rigExec:samplePhase` "
             "is `current`.", "no"),
            # rigEvaluator.cpp:4041 - the mover's weightObject binding is what
            # runs _ValidateWeightObjectDomain, so an unbound volume weight is
            # never validated or compiled at all.
            ("(bound by)", "A mover's `rigExec:weightObject` applies this field.",
             "-"),
        ],
        "param_groups": [
            ("Volume weight", "RigExecVolumeWeight"),
            ("Transform provider", "RigExecXformable"),
        ],
        "example": """A 13-column, 5-row strip is moved by one matrix mover reading a
joint at its root, and the
Bend control holds a constant 28 degrees for the whole shot — the only
animation in the file is the PlaneSlide control's `avars:tx`, which the
plane weight's own `avars:tx` is connected to. `rigExec:planeAxis = "x"`
with `falloffMin = 1.5` / `falloffMax = -1.5` hands the joint everything
past the plane, so sliding the handle from x = 3.0 out to 5.4 and back
walks the fold along the strip while the hinge angle never changes. The
two drawn rectangles are the ends of that band: the one at `falloffMin`,
1.5 units *past* the handle, is the fully-ON iso-surface, the one at
`falloffMax`, 1.5 units *before* it, is fully OFF — grey on the strip is
w = 0, red is w = 1, and the ramp between them is the fold.""",
        "tips": [
            "The band and the extents are different directions: "
            "`inputs:falloffMin`/`falloffMax` are distances ALONG "
            "`rigExec:planeAxis`, `inputs:extentU`/`extentV` are half-sizes "
            "ACROSS it (U = axis+1, V = axis+2, so Z and X for the default `y`). "
            "Scrubbing the band slides the two drawn rectangles apart without "
            "resizing them; only the extents resize them.",
            "`bounded` is hard — a point one epsilon outside the rectangle gets "
            "zero, not a ramped-down weight. Soften the border by multiplying the "
            "bounded plane with a sphere in a RigExecCombineWeight; the edge then "
            "inherits that volume's falloff.",
            "`rigExec:planeAxis` and `rigExec:planeBounds` are structural — they "
            "select which field function runs and are hashed into the binding "
            "epoch — while `inputs:extentU`/`extentV` are ordinary per-frame "
            "floats you can animate without recompiling.",
        ],
        "see_also": [
            ("sphere_weight", "Sphere Weight"),
            ("combine_weight", "Combine Weight"),
            ("matrix_mover", "Matrix Mover"),
        ],
    },
    "curve_weight": {
        "title": "Curve Weight",
        "schema": "RigExecCurveWeight",
        "summary": "A tube of influence around a curve's control polygon.",
        "description": """A placed volume that generates its field from distance to a curve
instead of storing one: everything within `inputs:falloffMin` of the
curve is fully weighted, everything past `inputs:falloffMax` is not, and
the band between them ramps. The curve is ordinary scene geometry named
through `rigExec:curve` — a weight object never grows its own points —
so a spline an artist already has becomes the shape of an influence.
Distance is measured to the **polyline through the control points**, not
to the evaluated basis, so the field never reaches anywhere the drawn
guide does not.""",
        "how_it_works": """Compile registers the volume's placement tap (`computePointFrame`,
not `computeMatrix`, so an unanimated volume still lands where it is
placed) and bakes the falloff into a lookup table; the structural digest
hashes `rigExec:curve` together with the *type* its target resolves to,
so repairing or retyping the points source re-enters Compile while
merely moving the curve does not. Then, each frame, `computeWeightPacket`
reads the curve's points and the weighted domain's points in the same
space, carries both into rigid volume-local space, divides by
`inputs:scaleX/Y/Z`, and remaps the nearest-segment distance through the
baked profile plus `inputs:invert` and `inputs:strength`. The result is a
dense float per target element, published as the envelope of whichever
mover binds it through `rigExec:weightObject` — a matrix mover then
applies `p' = q + w (T q - q)`.""",
        "wiring": [
            # rigEvaluator.cpp:5135-5159 - exactly one target, and it must resolve
            # to a point3f[] attribute, or compile fails.
            ("`rigExec:curve`", "Exactly one native points source — a BasisCurves prim "
             "or an exact `point3f[]` property — whose control polygon the distance "
             "is measured to.", "yes"),
            # rigEvaluator.cpp:872-884 - "rigExec:weightTarget must have exactly
            # one target", and it must equal the bound mover's target, or compile
            # fails; weightPackets.cpp:266 - empty target points invalidates the
            # packet.
            ("`rigExec:weightTarget`", "Exact points property this field covers.", "yes"),
            # rigEvaluator.cpp:5133 - requireTargets(..., 1, "at most one points
            # source"); weightPackets.cpp:270 - must match the target's cardinality.
            ("`rigExec:sampleSource`", "Optional unposed points to measure *instead of* "
             "the target, at the same element count; the weighted domain stays the "
             "target.", "no"),
            # schema.usda:79-88 (RigExecMoverAPI) - optional, at most one target.
            ("(bound by)", "A mover's `rigExec:weightObject` applies this field.", "-"),
        ],
        "param_groups": [
            ("Falloff band, sampling and guide", "RigExecVolumeWeight"),
            ("Placement", "RigExecXformable"),
        ],
        "example": """A flat 13x13 sheet is weighted by a tube around a shallow
V-shaped curve, and a matrix mover lifts the weighted points 0.8 units.
Nothing in the rig animates — the Lift control holds `avars:ty` for the
whole shot — but the curve's own points sweep across the sheet and back,
so a curved ridge travels with it. What moves is not the transform but
which points the field grabs. Two different things are drawn: the amber
wire tube is the pair of iso-surfaces at `inputs:falloffMin` and
`inputs:falloffMax`, and the red band painted on the sheet is the field
those two distances produce — the tube is the rule, the band is the
result. The curve itself rides at `y = 0.3`, just clear of the sheet, so
the control polygon the distance is measured to stays visible above the
geometry it weights.""",
        "tips": [
            "The field uses the control polygon, not the evaluated basis, so a "
            "cubic curve influences the region around its hull rather than around "
            "the smooth curve you see; add control points where you need the tube "
            "to bend.",
            "`inputs:scaleX/Y/Z` divide the local coordinate before the distance, "
            "turning the tube elliptical — but any axis that is non-positive or "
            "non-finite invalidates the whole packet rather than collapsing the "
            "volume.",
            "Leave `rigExec:samplePhase` at `reference` and the tube measures the "
            "static bind points, so a point keeps the weight its rest position "
            "earned; `current` measures the points as they stand at this mover's "
            "place in the stack, which makes the field order dependent by design.",
        ],
        "see_also": [
            ("sphere_weight", "Sphere Weight"),
            ("matrix_mover", "Matrix Mover"),
            ("static_weight", "Static Weight"),
        ],
    },
    "combine_weight": {
        "title": "Combine Weight",
        "schema": "RigExecCombineWeight",
        "summary": "Folds several weight fields into one under a single mode.",
        "description": """The operator that makes weight objects composable: a painted static
field masked by a sphere volume, two volumes unioned with `max`, a driven
dynamic weight subtracted out. Every input is resolved to a dense field
over the same elements and folded with one `rigExec:combineMode`, so the
result is an ordinary weight object that any mover can bind. One combine
carries one mode; mixed compositions nest combines inside combines rather
than tagging individual targets.""",
        "how_it_works": """The fold runs when the bound mover evaluates, in the geometry
(point-chain) phase, after the pose phase has placed every volumetric input — a volume weight is a
`RigExecXformable`, so the pose walk gives it a provider slot exactly
like a joint's. It resolves each `rigExec:inputWeights` target to a dense
field of the mover's element count (its own `rigExec:weightTarget` is
read only for that count, never for its points), folds them under the
mode — `multiply`, `add`, `subtract`, `max`, `min`, `average`, or
`overlay` — then applies `inputs:invert` and `inputs:strength` as
`w = (w + (1 - 2w) * invert) * strength` and bounds the result under
`rigExec:rangePolicy`. One invalid or differently sized input fails the
whole packet rather than folding a truncated field, and the mover passes
its points through unchanged.""",
        "wiring": [
            # Not enforced: an empty list folds to the mode's identity (0 for
            # add/subtract/max/average/overlay, 1 for multiply/min) and compiles
            # cleanly -- libs/rigExecMath/weightFields.cpp:288-291 and
            # libs/rigExecMath/weightFields.cpp:236-250. Verified by compiling
            # the example stage with the relationship deleted: it compiles, the
            # pose is valid, and the panel stays flat (max identity 0).
            ("`rigExec:inputWeights`", "Ordered weight objects to fold; every one "
             "must cover the same target as this combine.", "no"),
            # libs/rigExec/rigEvaluator.cpp:872 (exactly one target) and
            # libs/rigExec/rigEvaluator.cpp:880 (must equal the mover's target),
            # applied recursively to every input at rigEvaluator.cpp:887-899.
            # A bare PointBased prim is accepted and canonicalized to .points
            # for a point domain (rigEvaluator.cpp:877-879).
            ("`rigExec:weightTarget`", "The weighted domain, matching the bound "
             "mover's exact target; read only for its element count.", "yes"),
            # No relationship of its own: the mover names the combine, and the
            # binding is validated from the mover side in
            # libs/rigExec/rigEvaluator.cpp:584-593.
            ("(bound by)", "A mover's `rigExec:weightObject` applies the folded "
             "field.", "-"),
        ],
        "param_groups": [
            ("Weight field", "RigExecWeightObject"),
        ],
        "example": """Two sphere volumes over one flat 192-quad panel, folded with `max`.
A static joint holds a 1.0-unit lift, so the panel's height is literally
the combined field. The Anchor volume sits still at x = -1.2; the Slider
volume rides an animated control from x = 4.8 — parked clear of the panel,
where it contributes nothing, so the first frame is an honest picture of
the Anchor field alone — in to x = -0.2, holds there, and leaves again.
A second dome arrives, the two stand apart with a flat valley between
them, then merge into one wide ridge. `max` is the union, so neither
volume can ever dim the other; swap it for `min` and only the lens where
both volumes agree is left standing, and `multiply` softens that same
intersection.""",
        "tips": [
            "`multiply`, `add`, `max`, `min`, and `average` are order "
            "independent; `subtract` and `overlay` seed from the first target "
            "and consume the authored order, so reordering the relationship "
            "changes the result for those two alone.",
            "A combine must publish `dense` — it is the schema fallback and the "
            "only value its kernel accepts — but its inputs may be constant, "
            "sparse, dense, or generated; each is resolved to dense before the "
            "fold.",
            "`rigExec:rangePolicy` is inherited unchanged from "
            "`RigExecWeightObject`, so a combine falls back to `strict` -- "
            "unlike the volume weights, which redeclare it as `clamp`. Author "
            "`clamp`, as the example does, or an `add` that overlaps past 1 (or "
            "an overdriven `inputs:strength`) invalidates the packet and the "
            "mover silently passes its points through.",
        ],
        "see_also": [
            ("sphere_weight", "Sphere Weight"),
            ("static_weight", "Static Weight"),
            ("dynamic_weight", "Dynamic Weight"),
        ],
    },
    "curvenet_weight": {
        "title": "Curvenet Weight",
        "schema": "RigExecCurvenetWeight",
        "summary": "Paints a weight field on a curvenet and solves it onto a mesh.",
        "description": """Weight painting that survives a re-mesh. Instead of one scalar per
vertex, the values live on a curvenet's control-point pool — a few dozen
numbers on curves traced over the surface — and the field is *solved*
onto whatever mesh the net is pointed at. Bound through a mover's
`rigExec:weightObject`, it is an ordinary dense envelope: the mover never
learns that the falloff came from curves. Retessellate the mesh and the
same painted net produces the same falloff.""",
        "how_it_works": """The prim publishes `computeWeightPacket`, so the field is solved in
the weight computation that feeds its bound mover, before that mover's
application runs in the geometry phase. It reads the five native arrays
its relationships name — the mesh `points`, `faceVertexCounts` and
`faceVertexIndices`, and the net's `points` and `rigExec:splineIndices` —
at the requested time, walks each spline with `rigExec:basis` and
`rigExec:samplesPerSpline`, projects every sample onto the surface, and
minimizes `xᵀLx + κ‖Bx − Sw‖²` with `κ = 100 × mean edge length`
(`libs/rigExecMath/curvenetWeights.cpp:34`), where `S` interpolates `inputs:weights` at the
samples. Indices listed in `rigExec:autoSmooth` are solved harmonically
along the net's own connectivity first and their authored values ignored;
mesh components no sample reaches keep `rigExec:unreachedValue`
(`libs/rigExecMath/curvenetWeights.cpp:78-121,137-145,156`). `L + κBᵀB` is factorized once per
geometry/layout/basis/sampling/auto-smooth key and kept in a 32-entry
cache, so re-painting or animating `inputs:weights` re-solves against the
existing factors instead of re-cutting the mesh.""",
        "wiring": [
            # rigEvaluator.cpp:651-687 -- all five must resolve to exactly ONE
            # native property of the expected type, and each is then checked
            # against the matching mesh/net owner; a bare prim path fails.
            ("`rigExec:weightTarget`", "The moved mesh's exact `points` property — "
             "the same property the bound mover moves.", "yes"),  # rigEvaluator.cpp:659, 681
            ("`rigExec:curvenetPoints`", "One RigExecCurvenet's `points` control pool.",
             "yes"),  # rigEvaluator.cpp:660, 684
            ("`rigExec:curvenetSplineIndices`", "That same curvenet's "
             "`rigExec:splineIndices`.", "yes"),  # rigEvaluator.cpp:661, 685
            ("`rigExec:meshFaceCounts`", "The target mesh's `faceVertexCounts`.",
             "yes"),  # rigEvaluator.cpp:662, 682
            ("`rigExec:meshFaceIndices`", "The target mesh's `faceVertexIndices`.",
             "yes"),  # rigEvaluator.cpp:663, 683
            ("`inputs:weights`", "One float per control-pool point, including the "
             "tangent handles; auto-smoothed entries are ignored.",
             "yes"),  # libs/rigExecMath/curvenetWeights.cpp:131 rejects any other cardinality
            ("(bound by)", "A mover's `rigExec:weightObject` applies this field.",
             "-"),
        ],
        "param_groups": [
            ("Weight field", "RigExecWeightObject"),
        ],
        "example": """A flat slab is crossed by a curvenet: one rail down its length and
two profile curves meeting the rail at shared knots. The green splines are
that curvenet itself — it holds the numbers and stays at rest while the slab
bends, because the parametrization reads the authored pool, not a mover's
output. Eight knot values are painted and the solve turns them into a field
over all 65 slab vertices, which a matrix mover uses as its envelope: grey at
the Lift end, saturating to red at the far end, and the far edge stays paler
than the near edge, so the single `avars:rz` rotation lands as a graded bend
that also twists. Re-meshing the slab from three rows to five changed nothing
on the net — the same eight numbers re-solve onto whatever vertices are
there.""",
        "tips": [
            "Paint knots, auto-smooth handles: listing every tangent handle in "
            "`rigExec:autoSmooth` lets the net interpolate them harmonically, but "
            "each unknown run must reach at least one painted point or the bind "
            "fails with `auto-smooth component has no authored weight anchor` "
            "(libs/rigExecMath/curvenetWeights.cpp:104-105).",
            "Editing `inputs:weights` is a value edit — the factorization is keyed "
            "by geometry, layout, basis, sample count and auto-smooth membership "
            "(curvenetWeightComputations.cpp:88-90), and the arrays are read through "
            "the generation's resolved inputs every frame (bakedWeights.cpp:233-236), so an animated "
            "field costs one re-solve, not a re-cut.",
            "The least-squares fit can land just outside [0, 1]; the type's "
            "`clamp` default bounds it, while `strict` invalidates the packet "
            "(curvenetWeightComputations.cpp:48-55) and a mover handed an "
            "invalid envelope passes its preceding revision through unchanged "
            "(moverKernels.cpp:446).",
        ],
        "see_also": [
            ("curvenet", "Curvenet"),
            ("static_weight", "Static Weight"),
            ("matrix_mover", "Matrix Mover"),
        ],
    },
    "float_math_mover": {
        "title": "Float Math Mover",
        "schema": "RigExecFloatMathMover",
        "summary": "Arithmetic on one scalar channel: add, clamp, remap, blend.",
        "description": """The property domain's calculator: it revises an exact float attribute
— a blendshape weight, a blend factor, an envelope — instead of geometry.
The everyday job is unit plumbing. A channel arrives in whatever range the
department upstream authored it in, and `remap` divides it down into the
0 → 1 a weight expects; `clamp`, `add`, `multiply` and `blend` cover the
rest. `remap` deliberately does not bound its result, so a clamp mover
after it is what keeps an overshoot from driving a shape past its target.""",
        "how_it_works": """A math mover has no phase inside exec at all. Its inputs are all
authored on itself and the chain's base is the target attribute's own
authored value, so property chains are evaluated BEFORE exec runs and the
result is supplied to exec as a value override — which is how a normalized
weight reaches the blendshape mover or solver that reads it instead of
being recomputed inside that kernel. Each revision computes
`r = op(incoming)` and mixes it back through the common envelope
(`incoming + defaultWeight × (r − incoming)`), so zero passes the incoming
value through and one applies the operation outright. Movers sharing one
target revise it in mover-stack order: a reversed namespace walk, each
parent after its children, so the last sibling listed runs first.""",
        "wiring": [
            # rigEvaluator.cpp:3847 rejects anything but exactly one target
            # ("its parameters are mover-level, so a fan-out would alias them");
            # :3864 requires an exact property path, :3895 requires type float.
            # An authored-but-empty rigExec:moves is not an error -- the mover is
            # inert for that generation and says so (rigEvaluator.cpp:3577) -- so
            # "yes" here means "required for the mover to revise anything".
            ("`rigExec:moves`", "Exact float property to revise; exactly one.", "yes"),
        ],
        "param_groups": [
            ("Common mover envelope", "RigExecMoverAPI"),
        ],
        "example": """A hinged panel curls open from a single blendshape. The incoming
channel ramps straight up and back down, 0 → 4.8 → 0 in its own units, on
the blend input's `inputs:weight`. Two movers sit on that one property:
`Normalize` (remap, min 0 max 4) turns the raw number into a weight, and
`Bound` (clamp, min 0 max 1) catches the overshoot. At the peak the chain
reads **4.80 raw → 1.20 remapped → 1.00 bounded**, and 1.00 is all the
blendshape ever sees.

Watch for the stall. There is no hold anywhere in the animation, yet the
panel stops dead for about half a second near the top: the chip climbs
past 4.00 to 4.80 and back down to 4.00 while the geometry does not move a
pixel. That stall is `Bound` — everything above raw 4.0 remaps past 1.0
and is clamped back to it. The chip shows the raw number going in; the
panel shows what came out. The wire sphere at the origin is the `Hinge`
joint, a landmark on the line the panel curls about: nothing targets it
and it drives nothing, the blendshape does all the work.""",
        "tips": [
            # rigEvaluator.cpp:5540-5548: "the chain's base is the target
            # attribute's own authored value ... evaluable BEFORE exec runs".
            "Property chains resolve before exec runs, so a revised weight "
            "reaches solvers and movers in the same evaluation.",
            # rigEvaluator.cpp:149-163 (_GetMoverExecutionOrder reverses the
            # composed pre-order), confirmed by evaluating the example stage:
            # listing Bound first is what makes Normalize run first.
            "Same-target math movers run in mover-stack order — the reversed "
            "namespace walk — so the LAST sibling listed executes FIRST. "
            "`reorder nameChildren` is how the example puts remap before clamp.",
            # libs/rigExecMath/propertyMath.cpp:51-55: (base - min) / span, span == 0 -> 0, with
            # no min/max pinning afterwards.
            "`remap` only normalizes: `(v − min) / (max − min)`, with a "
            "zero-width range returning 0 rather than dividing. Chain a `clamp` "
            "after it whenever the incoming channel can overshoot.",
        ],
        "see_also": [
            ("blend_input", "Blend Input"),
            ("blendshape_mover", "Blendshape Mover"),
            ("vec3f_math_mover", "Vec3f Math Mover"),
        ],
    },
    "vec3f_math_mover": {
        "title": "Vec3f Math Mover",
        "schema": "RigExecVec3fMathMover",
        "summary": "Component-wise arithmetic on one vector-valued property.",
        "description": """Float math lifted to three components: add, multiply, clamp, remap,
or blend over an exact vector-valued attribute. The target does not have
to be typed `float3` — every GfVec3f-backed scalar role is accepted
(`float3`, `vector3f`, `point3f`, `normal3f`, `color3f`), because a mover
offsetting a vector and one offsetting a colour are doing the identical
arithmetic. Native transform ops are the natural targets: revising an
Xform's `xformOp:scale` carries its whole subtree without touching a
point. Watch the precision — UsdGeom authors a scale op as `float3` but
a translate op as `double3` by default, and a `double3` target is a
compile error, so a translate this mover drives has to be authored at
float precision.""",
        "how_it_works": """Each evaluation reads the mover's authored inputs, computes
`r = op(incoming)` per component, and mixes the result back over the
incoming value through the common envelope,
`incoming + defaultWeight × (r − incoming)`, one component at a time — so
a zero envelope is an exact pass-through and one applies the operation
outright. `add` and `multiply` use `inputs:value`, `blend` replaces the
incoming value with it, and `clamp`/`remap` use the `inputs:min` and
`inputs:max` bounds, which are `float3` precisely so the three components
can be bounded differently. `remap` normalizes `[min, max]` to `[0, 1]`
without clamping (a degenerate `min == max` yields 0).

Inputs may be CONNECTED rather than authored locally: the read follows
the connection chain and resolves the source at the evaluated time, which
is what lets an animator channel published on a control drive the mover.
The whole property chain still owes exec nothing, so it resolves BEFORE
exec runs and its result is handed back as the attribute's own value; a
chain whose input is produced by another property chain is ordered after
its producer.""",
        "wiring": [
            # rigEvaluator.cpp:3842-3866 -- a property mover's parameters are
            # mover-level, so a fan-out would alias them across targets: exactly
            # one exact property path, never a prim path (which canonicalizes to
            # .points).
            ("`rigExec:moves`", "Exactly one exact vector-valued property to "
             "revise (`float3`, `vector3f`, `point3f`, `normal3f`, `color3f`).",
             "yes"),
            # rigEvaluator.cpp:8995-9020 (_PinnedRead) and 9063 -- a connected
            # input is handed to RigExecResolvedInputs::GetAttribute, the only
            # code that follows a connection chain, and read at the frame time.
            ("`inputs:value.connect`", "Optional connection supplying the "
             "operand from another `float3` attribute — a control's animated "
             "channel, say — instead of an authored constant.", "no"),
            # schema.usda:79-88 -- when bound, the weight object's field is
            # authoritative and inputs:defaultWeight is ignored;
            # rigEvaluator.cpp:9175-9188 -- a property chain resolves it as a
            # single-element field.
            ("`rigExec:weightObject`", "Optional one-element weight field "
             "supplying the envelope instead of `inputs:defaultWeight`.", "no"),
        ],
        "param_groups": [
            ("Common mover envelope", "RigExecMoverAPI"),
        ],
        "example": """A Squash control sits on the card as a solid disc guide in the
card's own plane, and publishes one animator channel — a `float3`
`inputs:squash` keyed (1, 1, 1) → (1.45, 0.6, 1) → (0.72, 1.5, 1) → back.
The mover's `inputs:value` is connected to that channel and multiplies it
onto the card Xform's `xformOp:scale`, so the card squashes wide and
stretches tall around the handle while its authored scale stays
(1, 1, 1) and no point is ever touched.""",
        "tips": [
            "Connect `inputs:value` to a channel on a control to make the "
            "handle, not the mover, the thing an animator keys — the read "
            "follows the connection at the evaluated time.",
            "A vector channel carries time SAMPLES, not a spline: USD splines "
            "are scalar-valued, so `.spline` is for the avars and a `float3` "
            "channel is keyed with `.timeSamples`.",
            "Clamp and remap bounds are per component, so one mover can bound "
            "an asymmetric 3D range — and `remap` does not clamp, so compose a "
            "clamp mover after it when you want the result bounded.",
            "`multiply` at full envelope hands the operand through outright "
            "when the incoming value is (1, 1, 1); switch the operation to "
            "`add` and the same channel becomes an offset on a "
            "float-precision translate op (the default `xformOp:translate` is "
            "`double3`, which the mover rejects).",
        ],
        "see_also": [
            ("float_math_mover", "Float Math Mover"),
            ("matrix_math_mover", "Matrix Math Mover"),
            ("control", "Control"),
        ],
    },
    "matrix_math_mover": {
        "title": "Matrix Math Mover",
        "schema": "RigExecMatrixMathMover",
        "summary": "Multiplies or blends one matrix channel.",
        "description": """Matrix arithmetic on an exact matrix4d attribute — composing spaces,
post-multiplying offsets, blending between alternative frames. Distinct
from the point-moving Matrix Mover: this one revises the matrix itself,
and whatever reads that matrix follows. The operand is an ordinary
matrix4d input, so it can be a constant or a connection to another matrix
channel — a control's `posed:space` makes an animator handle the second
matrix in the product.""",
        "how_it_works": """`multiply` post-applies `inputs:value` after the incoming matrix
(row-vector convention: points meet the incoming matrix first, the value
second); `blend` replaces it. The envelope mixes component-wise and is
exact at both endpoints, so a `blend` at weight 1 is a straight
substitution and a partial weight is a crossfade between two frames.
`inputs:value` may be CONNECTED, which is how a control drives the
arithmetic: the mover reads the same `posed:space` matrix the control is
posed and drawn at. The whole property chain resolves BEFORE exec runs —
that is what lets its result be handed back as the attribute's value —
so the operand has to be a matrix that already stands on the stage,
authored or connected.""",
        "wiring": [
            ("`rigExec:moves`", "Exact matrix4d property to revise.", "yes"),
            ("`inputs:value.connect`", "Optional matrix4d source for the operand. "
             "Pointed at a control's `posed:space`, the mover follows that "
             "handle; unconnected, it uses the authored constant.", "no"),
        ],
        "param_groups": [
            ("Common mover envelope", "RigExecMoverAPI"),
        ],
        "example": """Two handles and one card, with no solver in between, and each
handle is drawn as one of the two operands. The **diamond** swinging on a
stalk above the card's rest centre is the Spin dial: it is the `blend`
operand, and the `blend` mover takes the card Xform's authored identity
straight to that frame (envelope 1 is a substitution, not a mix). The
**box** riding the card's centre is the Slide handle, the `multiply`
operand, post-applied afterwards. Because the multiply comes *after*, its
translation acts in the spun frame's parent: the card turns about its own
centre through 70° and *then* slides along the grid's X, not along its
own tilted X — which is the whole difference between post- and
pre-multiplication, visible in one picture. The blue wireframe card is
the rest pose the pair departs from.""",
        "tips": [
            "Post-multiplication order matters: the value applies AFTER the "
            "incoming matrix, so it acts in the incoming frame's parent — a "
            "translation runs along the parent axes whatever rotation came "
            "before it, and the incoming transform happens first.",
            "Connect `inputs:value` to a control's `posed:space` to make the "
            "operand animatable. A joint's posed frame is not available here: "
            "the chain resolves before the solve, so what it would read is the "
            "authored identity.",
            "Two movers on one matrix are a stack, and composed children "
            "execute bottom-up — author the `blend` that establishes the frame "
            "BELOW the `multiply` that offsets it.",
        ],
        "see_also": [
            ("matrix_mover", "Matrix Mover"),
            ("control", "Control"),
            ("float_math_mover", "Float Math Mover"),
        ],
    },
    "picker": {
        "title": "Picker",
        "schema": "RigExecPicker",
        "no_gif": True,
        "summary": "A character's control picker panel, shipped as scene data.",
        "description": """The animator's button board: a 2D panel of clickable shapes that
select the rig's controls, flip its switches, and zero its pose. A picker
is not a sidecar file beside the rig — it is `RigExecPicker` prims in the
rig's own layer stack, so it composes, it layers, and an animator can
override one button with an `over` instead of re-authoring anything.
Open it from usdview's **RigExec ▸ Control Picker** menu.""",
        "how_it_works": """Nothing here is evaluated and nothing here is drawn in the viewport:
the picker runs in no rig phase, the compiler never reads it, and the
schema classes are not imageable. The usdview panel finds pickers BY TYPE
— `Usd.PrimRange` over the stage, `IsA(RigExecPicker)`, pruning any prim
whose type starts with `RigExec` and is not the root or a picker — and
builds one character tab per picker, sorted on `ui:order`
(`plugin/rigExecUsdview/pickerScene.py:181-222`). Each picker reads its
`RigExecPickerPanel` children as sub-tabs and their `RigExecPickerButton`
grandchildren as items, then resolves every button target against the rig
named by `rigExec:picker:rig` — controls and joints only — to decide which
buttons are live (`pickerScene.py:152-178`, `:235-266`). Clicks drive
usdview's own selection; the stage is only written when a switch button
edits its attribute, or the `zero_ctrls` command clears authored pose
avars.""",
        "wiring": [
            # pickerScene.py:225-232 reads the relationship, and :246-266 falls
            # back to pickerModel.live_control_paths(stage) when it is absent --
            # nothing errors, the buttons just resolve against the whole stage.
            ("`rigExec:picker:rig`", "The `RigExecRoot` whose controls and joints "
             "the buttons resolve against. Unauthored means the whole stage, "
             "which is right until a shot holds two characters.", "no"),
            # pickerScene.py:157-171: panels are GetChildren() filtered on
            # IsA(RigExecPickerPanel), buttons are the panels' own children. A
            # picker with no panel children simply produces an empty tab.
            ("(child prims)", "`RigExecPickerPanel` children — direct children "
             "only — become the sub-tabs, in `ui:order`.", "no"),
        ],
        "param_groups": [],
        "example": """`picker.usda` is a two-bone arm with an IK/FK blend and the panel that
drives it: one `RigExecPicker` holding a `Body` panel with a backdrop
decoration, select buttons for the shoulder, elbow, wrist, pole and hand,
an `All FK` button that selects three controls at once, an IK/FK switch on
`ArmParams.avars:ikfk`, and a `Zero Ctrls` command. Nothing in the stage
is animated — the picker is interface, not motion — so open it in usdview
and click rather than scrubbing.""",
        "tips": [
            "A picker can be parented anywhere except inside rig graph: "
            "discovery prunes joints, controls, solvers, movers and weights, "
            "but not the `RigExecRoot`, so beside the rig or inside it both "
            "work (`pickerScene.py:213-222`).",
            "Layer the picker rather than editing it: an `over` on one button "
            "moves, recolours, relabels or retargets it, and `active = false` "
            "on a button, a panel or the whole picker removes it — all asserted "
            "in `tests/python/test_picker_scene.py:189-256`.",
            "Two characters on a stage give two tabs with nothing configured, "
            "but name each picker's rig anyway: without it liveness is "
            "computed against every control and joint on the whole stage "
            "(`pickerScene.py:246-266`, `pickerModel.py:311-322`).",
        ],
        "see_also": [
            ("picker_panel", "Picker Panel"),
            ("picker_button", "Picker Button"),
            ("control", "Control"),
        ],
        "example_key": "picker",
    },
    "picker_panel": {
        "title": "Picker Panel",
        "schema": "RigExecPickerPanel",
        "no_gif": True,
        "example_key": "picker",
        "summary": "One sub-tab of a picker: a 2D canvas of buttons.",
        "description": """A picker's sub-tab — Body, Face, Hands — and the coordinate space its
buttons are placed in. The origin is the TOP LEFT with y increasing
downward, matching every picker authoring tool and the Qt widget the panel
is drawn into, so positions copied out of a conventional picker land where
they did there. A panel holds nothing but `RigExecPickerButton` children
and a background colour.""",
        "how_it_works": """Read in usdview only, in no rig phase: `pickerScene.read()` takes the
picker's children that are `RigExecPickerPanel`, sorts them on `ui:order`
with namespace order breaking ties, and makes one sub-tab each; its own
children that are `RigExecPickerButton` become that tab's items
(`plugin/rigExecUsdview/pickerScene.py:152-178`). `ui:background` is
converted from the schema's 0-1 colour to 8-bit for Qt, and `ui:label`
falls back to the prim name. `ui:size` is the declared extent, but the
view fits the tab to the BUTTONS' bounding box plus a 12-unit margin when
the panel has any, and only falls back to `ui:size` for an empty panel
(`pickerModel.py:325-349`).""",
        "wiring": [
            # pickerScene.py:169-171 -- buttons are the panel's own children,
            # filtered on IsA(RigExecPickerButton); a grandchild is not found.
            # Nothing enforces this: an empty panel is a blank tab, not an error.
            ("(child prims)", "`RigExecPickerButton` children — direct children "
             "only — are this tab's items.", "no"),
            # pickerScene.py:157-158: GetChildren() uses the default predicate,
            # so `active = false` on the panel drops the whole sub-tab
            # (tests/python/test_picker_scene.py:247-250).
            ("(none)", "A panel names nothing else; it is placed by being a "
             "child of its `RigExecPicker`.", "-"),
        ],
        "param_groups": [],
        "example": """`picker.usda` carries a single `Body` panel, 200 by 260 picker units on
a dark grey ground, holding nine buttons: a backdrop, two mode-filtered
pairs, a multi-control `All FK` button, an IK/FK switch and a `Zero Ctrls`
command. Its buttons happen to fill the declared `ui:size` exactly, so the
fitted view and the declaration agree.""",
        "tips": [
            "Author button positions as if `ui:size` were the frame, but do not "
            "rely on it: the tab is fitted to the buttons, so a panel split out "
            "of a shared canvas still reads (`pickerModel.py:325-349`).",
            "y grows DOWNWARD in panel units — a button at y=24 is near the top "
            "of the tab, not the bottom (`schema.usda:2379-2382`).",
            "`ui:label` is the tab caption and empty means the prim name, so a "
            "panel named `Body` needs no label at all "
            "(`pickerScene.py:164`).",
        ],
        "see_also": [
            ("picker", "Picker"),
            ("picker_button", "Picker Button"),
        ],
    },
    "picker_button": {
        "title": "Picker Button",
        "schema": "RigExecPickerButton",
        "no_gif": True,
        "example_key": "picker",
        "summary": "One clickable shape: selects controls, flips a switch, or decorates.",
        "description": """A button is one shape in a panel and one of four things, decided by
what it names rather than by a flag: a SELECT button lists prims in
`rigExec:picker:controls`, a SWITCH edits the one attribute in
`rigExec:picker:attribute`, a COMMAND runs the named action in
`rigExec:picker:command`, and a button naming none of them is DECORATION —
a backdrop, a silhouette, a label. Everything else on the class is
appearance and visibility: outline, colours, text, draw order, and
which half of an IK/FK pair a button belongs to.""",
        "how_it_works": """usdview reads buttons when the panel opens and after any change to
the prims; no rig phase touches them. Liveness is the resolve result, not
a flag: `pickerScene` turns the relationships into target paths and the
model marks a selecting or switching button live only when every target —
or, for a switch, the prim owning the attribute — is a `RigExecControl` or
`RigExecJoint` inside the rig named by `rigExec:picker:rig`, or anywhere on
the stage when the picker names none (`pickerModel.py:89-98`,
`pickerScene.py:235-243`, `:246-266`); a command button the panel
implements is live on its own (`pickerModel.py:89-90`). A button that
resolves to nothing is not drawn at all, though `coverage()` still counts
it for the panel's status line (`pickerModel.py:222-223`, `:296-306`,
`pickerUI.py:804-817`). A click selects the targets in order, or cycles the
switch's attribute one step and writes it back as a float
(`pickerUI.py:819-838`, `:984-998`); the hit test is the button's bounding
box rather than its drawn outline, and `ui:size` is clamped up to 4 units
so a fingertip button is still catchable (`pickerModel.py:23`, `:41-42`,
`:240-262`).""",
        "wiring": [
            # pickerScene.py:62-70 reads the targets; pickerModel.py:96-98 makes
            # the button live only if ALL of them resolve. Nothing requires it:
            # a button with no targets and no attribute is a decoration
            # (pickerModel.py:100-104).
            ("`rigExec:picker:controls`", "The `RigExecControl` or `RigExecJoint` "
             "prims this button selects, in order.", "no"),
            # pickerScene.py:72-86 takes the first target only, splitting it into
            # prim path + property name; pickerModel.py:91-93 tests that PRIM for
            # liveness, so the attribute must live on a control or a joint.
            ("`rigExec:picker:attribute`", "The single attribute a switch edits, "
             "as a property path on a control or joint. Mutually exclusive with "
             "`rigExec:picker:controls`; on a plain click the attribute wins.",
             "no"),
            # pickerScene.py:88-92, :140-142: an authored rigExec:picker:mode is
            # DROPPED unless this relationship resolves, because a mode with no
            # dial cannot be tested.
            ("`rigExec:picker:modeDial`", "The attribute `rigExec:picker:mode` is "
             "tested against — the limb's IK/FK dial.",
             "only with `rigExec:picker:mode`"),
        ],
        "param_groups": [],
        "example": """`picker.usda` has one of each kind in a single panel: `b_Shoulder`
selects one control, `b_Arm` selects three at once, `b_ArmIkFk` switches
`ArmParams.avars:ikfk` between the labels `FK` and `IK`, `b_Zero` runs the
`zero_ctrls` command, and `backdrop` names nothing and is decoration. The
elbow/wrist pair carries `rigExec:picker:mode = "fk"` and the pole/hand
pair `"ik"`, both dialled off `ArmParams.avars:ikfk`, so half the limb's
buttons swap out when the switch is clicked.""",
        "tips": [
            "Leave `ui:text` unauthored on a command button. The panel falls "
            "back to the command's own title (`Zero Ctrls`) and then dispatches "
            "on that LABEL, so a friendlier caption makes the button inert "
            "(`pickerScene.py:130-131`, `pickerModel.py:28`, `:77-80`).",
            "Author a switch's attribute on a control or a joint — a custom "
            "`avars:ikfk` on a params control, connected onward to the blend — "
            "because an attribute on a solver prim fails the liveness test and "
            "the button is never drawn (`pickerModel.py:91-93`, `:222-223`).",
            "`rigExec:picker:mode` is ignored without `rigExec:picker:modeDial`, "
            "and a dial resting between 0.001 and 0.999 counts as neither mode, "
            "so both halves stay reachable mid-handover "
            "(`pickerScene.py:140-142`, `pickerUI.py:950-956`).",
        ],
        "see_also": [
            ("picker", "Picker"),
            ("picker_panel", "Picker Panel"),
            ("blend_point_frames", "Blend Point Frames"),
        ],
    },
    "touch_regions": {
        "title": "Touch Regions",
        "schema": "RigExecTouchRegions",
        "no_gif": True,
        "summary": "Named face sets that turn the model itself into the control picker.",
        "description": """Touch regions make the skin clickable: a named set of faces on one
mesh carries the control a click inside it should select, so picking the
forearm selects the forearm control instead of hunting for a wire shape in
a crowded viewport. A `RigExecTouchRegions` scope names the mesh it
annotates and holds one `RigExecTouchRegion` child per set, plus the one
colour ramp declared for all of them — `rigExec:touch:palette`, indexed by
the region's own order — and the opacity to draw them at. The scope is
found by type, so it
may be parked beside the geometry it annotates or inside the `RigExecRoot`
for a studio that ships one prim holding the whole rig.

Regions are annotation, not rig: a stage evaluates identically with them,
without them, or with half of them unpainted. They are deliberately *not*
`GeomSubset`s — hdSt collects every face subset under a mesh whatever its
`familyName`, so a touch set collided with the `materialBind` subset owning
the same face (16,739 warnings on open), and the sets were moved into a
scope of their own.""",
        "how_it_works": """Touch regions belong to no compile or evaluation phase: `rigExec:touch:*`
appears nowhere under `libs/rigExec`, so they add nothing to the pose walk.
They are read at UI time by the usdview TouchPose plugin, which traverses
the stage for prims typed `RigExecTouchRegions`, keeps the scopes whose
`rigExec:touch:mesh` targets the mesh being touched (a scope naming no
mesh matches whichever mesh is asked for), takes the first of them that
has region children, and flattens those into a single `face -> region`
int array (−1 for unpainted skin).
That table goes to the native mesh in `rigExecImaging`, which keeps a BVH
over the *posed* triangles and refits it whenever the rig publishes a new
pose, so a pick ray hits the deformed skin and resolves to a face, a
region, and the control that region names. Lighting a region creates,
deletes and authors nothing: the imaging plugin publishes a per-face slot
(the region index + 1, `0` for unpainted skin) as a uniform primvar and
the current colours as a constant `vec4[]` table, and swaps the mesh's
surface terminal for a generated shader that runs the original one and
then mixes `table[slot]` into the lit colour — a hover is one
constant-primvar upload, not a resync.""",
        "wiring": [
            # touchPoseModel.py:88-91 -- FindRegionScopes keeps a scope only when
            # its rigExec:touch:mesh targets include the mesh being touched. A
            # scope with NO targets is not filtered out (it then matches whatever
            # mesh is asked for), and nothing under libs/rigExec reads it at all.
            ("`rigExec:touch:mesh`", "The `UsdGeomMesh` whose faces every region "
             "under this scope indexes. Nothing enforces it: a scope with no "
             "target is kept for whichever mesh is asked for, which is only safe "
             "while the stage holds one character.", "no"),
            # touchPoseModel.py:219-227 -- the reader takes the first scope whose
            # children are typed RigExecTouchRegion (or carry the legacy
            # touchpose:faces attribute); a scope with none is skipped entirely.
            ("(children)", "One `RigExecTouchRegion` per named set. A scope with "
             "no region children is passed over.", "yes"),
            # touchPoseModel.py:196 + 240-241 -- FromStage defaults
            # require_control=True and `continue`s past a region that binds
            # nothing: "a region the click cannot act on must not swallow the
            # click that would otherwise reach usdview's own picking".
            ("`rigExec:touch:control`", "Per region: the control a click inside it "
             "selects. Unenforced, but a region without one is dropped by the "
             "reader's default rather than drawn inert.", "no"),
            # touchPoseModel.py:167-172 -- the region's faces are scattered into
            # the face->region table; an absent or empty array leaves the region
            # owning no faces, so it is never hit.
            ("`rigExec:touch:faces`", "Per region: the face indices it owns on the "
             "parent's mesh. Unenforced: a region with none is read, owns no "
             "face, and can never be hit.", "no"),
        ],
        "param_groups": [
            ("Touch region (per child prim)", "RigExecTouchRegion"),
        ],
        "example": """A three-segment strip skinned by an FK chain, its twelve faces divided
into three named regions — `base_touch`, `mid_touch`, `tip_touch`, four
faces each — every one naming the control that poses that segment. The
`TouchPose` scope targets the single `Skin` mesh and carries a three-colour
ramp and the empty, invisible `Overlay` prim the TouchPose exporter ships
beside the regions. Nothing in the file drives the regions: the FK controls
curl the strip and back, and the face sets ride along on the deformed
skin.""",
        "tips": [
            "Keep regions disjoint. The lookup is one flat `face -> region` array "
            "built by scattering each region's faces in namespace order, so a "
            "face claimed twice silently belongs to whichever region is written "
            "last; the painting tools maintain the partition for you.",
            "A region with no `rigExec:touch:control` is skipped by the reader "
            "rather than drawn dead — an unbound set must not swallow the click "
            "that would otherwise reach usdview's own picking.",
            "Face indices are scattered against the mesh's live face count and "
            "anything out of range is dropped without a word, so a region set "
            "exported against different topology fails quietly: re-export the "
            "regions whenever the mesh's face count changes.",
        ],
        "see_also": [
            ("touch_region", "Touch Region"),
            ("control", "Control"),
            ("fk_chain", "FK Chain"),
        ],
        "example_key": "touch_regions",
    },
    "touch_region": {
        "title": "Touch Region",
        "schema": "RigExecTouchRegion",
        "no_gif": True,
        "summary": "A named set of mesh faces that selects the control posing them.",
        "description": """A touch region is the annotation that makes a character
clickable: a named set of face indices on one mesh, plus the control a click
inside that set should select. Touch a shoulder in the viewport and the
shoulder's control is selected, with no picker window and no knowledge of where
the rig parked its controls. Regions are *not* `GeomSubset`s — hdSt collects
every face subset under a mesh whatever its `familyName`, so touch sets living
there collided with the material-bind sets sharing the same faces
(schema.usda:2532-2536); they sit in their own `RigExecTouchRegions` scope
instead, found by type rather than by position.""",
        "how_it_works": """Nothing about a touch region runs in an evaluation
phase — the rig evaluator never reads a `rigExec:touch:*` token at all (no
match for `rigExec:touch` anywhere under `libs/rigExec/`), so a region adds no
compile record, no step and no cost to a pose. It is read once at *attach* time
by the TouchPose viewport tool, which finds every `RigExecTouchRegions` scope by
type, filters them by the mesh they name, and reads each typed
`RigExecTouchRegion` child in namespace order
(touchPoseModel.py:73-93, 217-244). From those it builds a flat
face → region-index array — `-1` for unpainted skin — and hands it to the
native mesh once (touchPoseModel.py:167-189); a pick is then a BVH ray cast to a
face plus one array index, and the first target of `rigExec:touch:control`
becomes the prim the selection is replaced with (touchPoseModel.py:239,
touchPoseUI.py:1011-1029). The
region writes nothing to the stage: the highlight is a Storm shader tint fed by
two synthetic primvars, `rigExecTouchRegion` (face → slot) and
`rigExecTouchTable` (slot → colour), added by a scene-index filter
(touchPoseHighlight.h:5-31).""",
        "wiring": [
            # Nothing enforces this: the evaluator never reads it, and an empty
            # face set is simply skipped when the face -> region table is built
            # (plugin/touchPose/touchPoseModel.py:169). A region with no faces
            # can never be picked.
            ("`rigExec:touch:faces`", "Face indices on the mesh named by the parent "
             "scope's `rigExec:touch:mesh`. Indices outside the mesh's face range "
             "are dropped silently.", "no"),
            # schema.usda:2639-2643: unauthored means "drawn but drives nothing".
            # The reader's default drops such a region entirely so the click falls
            # through to usdview's own picking
            # (plugin/touchPose/touchPoseModel.py:196, 238-241).
            ("`rigExec:touch:control`", "The prim a click inside this region "
             "selects. Only the first target is used; unauthored means the region "
             "is skipped by the default reader.", "no"),
            # The parent is what names the mesh and carries the palette; a region
            # is located by walking the children of a RigExecTouchRegions scope
            # (plugin/touchPose/touchPoseModel.py:216-224).
            ("(parent scope)", "A `RigExecTouchRegions` prim, which names the mesh "
             "and holds the shared palette. Regions are only found as its "
             "children.", "yes"),
        ],
        "param_groups": [
            ("Region scope", "RigExecTouchRegions"),
        ],
        "example": """A two-segment limb, twelve quads of skin, painted into two
regions: `Upper` owns the six faces over the first bone and selects the Upper
control, `Fore` owns the six over the second and selects the Fore control. The
FK chain folds the limb and unfolds it over frames 1001-1016, and the regions
follow the deformation for free because they index faces, not points.""",
        "tips": [
            "Regions must not overlap: the face → region table is built by "
            "assignment, so a face claimed twice ends up owned by whichever region "
            "comes LAST in namespace order (touchPoseModel.py:172).",
            "`rigExec:touch:elementType` only allows `face` today, and nothing "
            "reads it yet — the exporter writes it and the reader ignores it. It is "
            "there so a point or edge set does not need a second attribute later.",
            "A region binds by path, not by type: the picker just resolves the "
            "target and selects it, so an unfinished paint can point at a joint, a "
            "solver, anything — but a region with no target is dropped, not drawn "
            "inert, unless the host asks for `require_control=False`.",
        ],
        "see_also": [
            ("touch_regions", "Touch Regions"),
            ("control", "Control"),
            ("matrix_mover", "Matrix Mover"),
        ],
        "example_key": "touch_region",
    },
}
