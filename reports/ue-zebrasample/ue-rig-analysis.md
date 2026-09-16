# ZebraSample Unreal character rigs: operator and setup analysis

Companion to [`docs/plans/ue-zebrasample-rig-gaps.md`](../../docs/plans/ue-zebrasample-rig-gaps.md). This is the UE-side evidence base: what each rig asset is, and every operator and setup pattern the Zebra and Monster rigs rely on, described precisely enough to re-implement. It was written by eight analysts from the text dump produced by [`tools/ueDumpRigs.py`](../../tools/ueDumpRigs.py) and from the UE 5.8 engine sources; nothing here was checked by running Unreal beyond that dump.

Path conventions: `<dump>/` is the dump directory (one directory per asset, `/` → `__`), `<UE>/` the engine install, `<ZebraSample>/` the project. Line numbers refer to the dump files.

Errata applied to the analysts' text: MR_Zebra has **34** module instances and MR_FN_Biped **48** (counted from `modular_rig_model.txt` and the instance sub-objects in `asset.t3d`); where a sentence still says 36 or 50, read 34 or 48.

## 1. Assets

### UE1 modular body rigs

- **MR_Zebra (/Game/Assets/Zebra/Rig/MR_Zebra)** — *Production modular body rig for SKM_Zebra (ControlRigBlueprint, runtime class ModularRig)*. 34 modules: root, Prop, Body, Spine, Neck, Clavicle L/R, Arm L/R, Leg L/R, Foot L/R, 8 LimbTwist modules, Thumb/Index/Pinky L/R, Tweakers, Face (the CRM_Zebra_Face runtime asset), Ear Base L/R, Ear L/R and Mohawk. The rig's own graph is a single empty Forwards Solve node (RigUnit_BeginExecution). Everything else comes from the modules. Static hierarchy: 371 imported bones, 958 curves, 8 socket nulls and 149 connectors. Runtime hierarchy: 423 bones, 326 nulls, 359 controls (252 body, 107 face), 149 connectors. It has 2 rig variables (Arm_L_RotateOrder and Arm_R_RotateOrder, both XYZ), each bound into an Arm module. Settings: bAutoResolve=False, ProceduralElementLimit=3486. Events: only Forwards Solve on the host. Construction, Backwards Solve, Connector and user events run inside the modules.
- **MR_ZebraDMC** — *MR_Zebra plus the Direct Mesh Control module*. Same module graph as MR_Zebra, plus module 'CRM_FN_DMC' (the /FortniteRigs/Modules/Miscellaneous/CRM_FN_DMC runtime asset, parent root, connector CRM_FN_DMC/Root -> bone root, Binding Direct_Mesh_Control <- rig var Direct_Mesh_Control=True). It has 150 connectors and an otherwise identical runtime hierarchy (359 controls). The only other runtime differences: the Mohawk controls' shape resolves to 'Default' instead of 'DefaultGizmoLibraryNormalized.Circle_Thick', and root/RootJoint carries extra module metadata 'Direct Mesh Control Libraries' and 'Direct Mesh Control'. ProceduralElementLimit=3487.
- **MR_FN_Biped (template)** — *Fortnite/Manny biped template that MR_Zebra and MR_Monster were derived from*. 48 modules on SKM_Manny (106 bones, 800 curves, 8 foot-socket nulls, 192 connectors). Adds the following over Zebra: Meta L/R (FkArray over 4 metacarpals); Index/Middle/Ring/Pinky chains parented to Meta; 12 CRM_FN_ProxyControl modules (Finger Spread Inner/Outer, Finger Inner/Outer Curl, Meta Spread Inner/Outer, per side); Attach (FkArray over attach/cape/backpack/weapon/hand_attach bones); Stretch Feedback (CRM_FN_BipedStretchFeedback); IK Bone Pins (CRM_FN_Pin). Runtime: 127 bones, 329 nulls, 283 controls. PreviousModulePaths is identical to Zebra's, which shows the lineage.
- **MR_FN_BipedDMC (template)** — *Biped template plus the DMC module*. MR_FN_Biped plus CRM_FN_DMC (parent root, ConfigOverrides=(), Binding Direct_Mesh_Control). 193 connectors and 283 runtime controls.
- **MR_Monster** — *Bust creature (no arms or legs) modular rig*. 7 modules: root, Prop, Body, Spine, Clavicle L/R and Face. Face is the CRM_Monster_Face runtime asset with parent module Spine; its Face/Parent connector is unconnected. There are no Neck, Arm, Leg, Foot, finger or ear modules, even though bones neck_01, neck_02 and head exist. Static: 165 bones, 993 curves, 26 connectors. Runtime: 202 bones, 85 nulls, 177 controls (57 body, 120 face). Body-module config values are identical to Zebra's. Vestigial variables Arm_L_RotateOrder and Arm_R_RotateOrder remain. Prop/Prop only gets the spine_05 space because hand_l and hand_r don't exist.
- **MR_Boombox** — *Prop modular rig (ControlRigRuntimeAsset) built from the engine's generic modules*. Module chain: Root (/ControlRig/Modules/Modules58/Root) -> AddControl -> AddControl_1 -> AddControl_2..6 (/ControlRigModules/Modules58/AddControl). Each AddControl primary connector ('Add Control Primary', rule Type=Socket) connects to a socket. 'Parent Control' is secondary and optional, with rule Or(Type=Null, Type=Control, Type=Bone), and connects to the previous module's control. Static: 8 bones, 1 user socket (handle_socket on bone handle), 15 connectors. Config lives in struct variables (Module Settings ControlSize, Control Settings Shape). No runtime_hierarchy dump.
- **CRU_PropAim** — *Utility modular rig with a single Prop module on a one-bone skeleton*. Prop module with no parent. Prop/Parent -> bone root, Prop/Spaces -> [root], Control Vis Channel Host unconnected. Runtime: 17 controls (6 transform, 8 bool, 3 float) and 3 nulls. bAutoResolve=False, ProceduralElementLimit=2004, preview mesh /EpicControlRig/Meshes/Dummy/SKM_Dummy.
- **AAU_Biped** — *Editor ActorActionUtility (Blutility) for animators: IK/FK snap-and-switch on a selected limb module in Sequencer*. ParentClass ActorActionUtility. Variables: 'Control Rig' (ControlRig) and 'Modular Rig' (ModularRig). Functions: 'Match Limb(InModule)' (bCallInEditor) and 'Select Control(ControlRig, ControlName)'. The EventGraph is empty. Match Limb finds the Sequencer control rig with an active section and a non-empty selection, reads '<Module>/Ik Fk Switch' at the current frame, then runs module user event 'To FK' or 'To IK' plus a Forwards Solve. It then keys the switch to the opposite value, selects '<Module>/FK 2' or '<Module>/IK', and runs module event 'Key Controls' plus a Forwards Solve. The asset does not mirror or build rigs.
- **ModularRigGizmoLibrary / ModularRigGizmoLibrary_DMC** — *Control shape libraries (ControlRigShapeLibrary) used by the Fortnite modules*. Both libraries have the same 62 shape names: 18 families x {Thin, Thick, Solid}, plus Circle_Pins_Thick, Square_Pins_Thick, Sphere_Pins_Small, Stick_Pins_Thick, Cross_Pins_Thick, RoundedTrapeze_Thick, RoundedTrapeze_ArrowTip_Thick and RoundedSquare_ArrowTip_Thick. Every shape has scale 0.1. The default shape is Sphere_solid, MaterialColorParameter='Color'. The DMC variant uses ModularRigGizmoMaterial_DMC, a different XRay material, and adds MaterialHoveredParameter='Hovered' and MaterialHoveredColorParameter='HoveredColor'.

### UE2 FK-family modules

- **CRM_FN_Root (/FortniteRigs/Modules/FkSolves/Root/CRM_FN_Root)** — *Top-level root module of every Fortnite modular rig (module name 'root'/'Root' in MR_FN_Biped, MR_Zebra, MR_ZebraDMC, MR_Monster). It publishes rig-wide settings (global control scale, left/right/center colors) as module metadata.*. Connectors: RootJoint (primary), plus 'Body' (secondary, optional, RigChildOfPrimaryConnectionRule); socket 'root'. Config: Control Scale=1, Global Control Scale=1, Global Center/Left/Right Control Color=(1,1,0)/(0,0,1)/(1,0,0), Lock Scale=false. Construction (67 nodes): spawns the controls Global (no parent), Local (child of Global) and Root (child of Global, with 'Local Control' as an available space). It spawns channels: 'Bake Root On' (integer, uses the CREnum_RootMatching enum) on Global, and 'Control Path Vis' (bool) plus 'Control Path Distance' (float, default 50) on both Global and Local. It writes 7 module metadata values (NameSpace=Self) and runs Set Control Scale with factor ChainLength(root, Body)/96. Forwards (33 nodes): Lock Scale, root bone = Root control, control-path debug lines, and it clears the Snapped flag. Backwards (44 nodes): on the first frame it stores the Global transform, then a switch on 'Bake Root On' decides whether root motion goes to the Root, Global or Local control. Local functions: Generate Control Path, Set Up Control Path, and Snap Global Control (not used). Library calls: Set Control Scale, Get Chain Length. Runtime in MR_Zebra: 3 transform controls and 5 channels.
- **CREnum_RootMatching** — *UserDefinedEnum that the 'Bake Root On' integer channel uses for display.*. DisplayNameMap: NewEnumerator0='Global Control', NewEnumerator1='Local Control', NewEnumerator4='Root Control'. The t3d does not record the order of the enumerator values. In the Root backward solve, comment boxes place case 0 under 'Root motion on Root Control', case 1 under 'Global' and case 2 under 'Local'.
- **CRM_FN_Body (/FortniteRigs/Modules/FkSolves/Body/CRM_FN_Body)** — *COG/body module: Body Orbit, Body, optional Body Offset, a Body Aim target and a movable pivot proxy. Drives the pelvis ('Body' connector).*. Connectors: Parent (primary), plus optional secondaries Body, Right Hip and Left Hip; socket 'root'. Config: Drive Body Joint=true, Control Scale=1, Lock Scale=false, Body Aim Control Offset=(0,0,38), Body Aim Axis=(0,0,1), Create Body Offset Control=false. Private shape variables: Body Control Shape (Hexagon_Thin), Body Proxy Control Shape (Sphere_Solid), Aim Control Shape (RoundedTrapeze_ArrowTip_Thick). Construction (131 nodes): Body Orbit is placed at the hip midpoint (or at the Body bone), then Body, optional Body Offset, the Body Movable Pivot proxy, and Body Aim. Channels: Body Orbit Vis, Body Aim Vis, Aim Weight, Aim Twist, Body Offset Vis, Movable Pivot Vis. It stores the 'Body Delta Transform' metadata and writes Body Control into the Root module's metadata namespace. Forwards (97 nodes): visibility, lock scale, aim/twist logic (stateful and aware of editor interaction), pelvis drive via Project To New Parent, and the movable proxy. Backwards (17 nodes): Body = inverse(delta) * pelvis, and Body Offset is reset. Local function: Drive Aim and Body Rotation (42 nodes). Library calls: Construct/Forward Movable Proxy v01, Set Control Scale, Get Control Color From Metadata. Runtime: 3 animation controls, 1 proxy control, 5 channels and 2 nulls.
- **CRM_FN_Spine (/FortniteRigs/Modules/Biped/Spine/CRM_FN_Spine)** — *Spline IK/FK spine module. The same asset is used for both 'Spine' (pelvis..spine_05) and 'Neck' (neck_01..head, with Is Neck=true) in MR_FN_Biped and MR_Zebra (4 references); MR_Monster has only Spine.*. Connectors: Start (primary); End (secondary, ChildOfPrimary rule); End Parent; Start Parent; Start Orient Spaces and End Orient Spaces (arrays, optional); Start Snap To (optional). Construction (439 nodes) creates: virtual bones '<bone>_virtual'; nulls '<bone>_reoriented'; the FK stack 'Start FK', 'Mid FK', 'End FK' with 'Default X FK Space' nulls and an 'End IK Null'; 'Start IK' + 'Default Start IK Space' + 'Start Snap To'; 'End IK' + 'Default End IK Space'; 'Mid IK' + 'Mid IK Null'; the proxy 'End Movable Pivot' (spine only); '<Bone> Sec FK' controls and '<bone>_match' nulls; 'Pelvis Local' and the 'Pelvis TXY' null (spine only); scale-float channels Stretch, Distribute Rotation and Mid Blend; bool channels IK Vis, FK Vis and Sec FK Vis; orient-space nulls; and the 'Bone Percentage' metadata. Forwards (115 nodes): visibility, orient spaces, distribute rotation, mid-null constraint, cubic spline build, 'Attach Sec FKs To Spline' (Fit Chain + stretch), Sec FK offset update, bone write, pelvis local, and the movable proxy. Backwards (125 nodes): full IK/FK/Sec FK matching, including a ray intersection for Mid IK. Local functions: Build Spline From Items, Solve Mid Control Null, Controls Visibility, Get Optional Display Name, Control Color Override From Metadata, Attach Sec FKs To Spline (76 nodes), and Construct/Forward Sliding Proxy (defined but never referenced). Runtime in MR_Zebra: Spine has 13 animation controls, 1 proxy, 7 channels, 35 nulls and 6 virtual bones; Neck has 9 controls, 6 channels, 30 nulls and 3 virtual bones.
- **CRM_FN_FkChain (/FortniteRigs/Modules/FkSolves/Fk/CRM_FN_FkChain)** — *Generic FK chain module (clavicles, fingers, thumbs). 24 references in MR_FN_Biped, 16 in MR_Zebra, 4 in MR_Monster.*. Connectors: Start (primary); Parent (optional); End (optional, ChildOfPrimary rule); Orient Spaces (optional array). Config: Control Names As FK=true, Drive Bones=true, Lock Scale=false, Default Orient Space Index=-1, Control Scale=1, Rotation Order=XYZ, Mirror Behavior=false, Mirror Axis=(0,1,1), Display Names[], Use Active Skeleton=false. Private: Control Transform Offset, Shape Settings (Circle_Thin), Control Scale Factor Profile (flat curve at 1). Construction (105 nodes): shape library setup; chain resolution; for each bone a null plus a control named 'FK'/'FK n' or a conformed bone name; mirror metadata; orient-space nulls on the first control; optional default orient space via Switch Parent; control scale. Forwards (42 nodes): lock scale, drive bones (optionally additive on the active skeleton), evaluate orient nulls. Backwards (17 nodes): evaluate orient nulls, then control = offset * bone. The Connector event sets default matches for Parent and End. Local functions: Get Control Shape, Get Control Name (only Get Control Shape is called).
- **CRM_FN_FkArray (/FortniteRigs/Modules/FkSolves/Fk/CRM_FN_FkArray)** — *Array of independent FK controls (metacarpals, attach bones, zebra ears/mohawk/tweakers). 6 references in MR_FN_Biped, 12 in MR_Zebra.*. Connectors: Parent (primary); Bones (array, required); Spaces, Orient Spaces and Override Parents (optional arrays). Config: Standard FK Names=false, Drive Bones=true, Rotation Order=XYZ, Use Active Skeleton=false, Mirror Axis=(0,1,1), Mirror Behavior=true, Control Scale=1, Display Names[], Controls Visibility Initials[]. Construction (221 nodes): filters out bones that do not exist; parents come from each bone's hierarchy parent, with optional overrides. Per bone it creates '<Bone> Default Space', then a null and a control named after the conformed bone, plus a 'Visibility' bool channel. Space nulls come in full and orient-only variants; control scale is applied last. Forwards (53 nodes): per-control visibility from its channel, space-null evaluation, bone drive. Backwards (22 nodes): evaluate space nulls, then control = offset * bone. The Connector event is empty.
- **CRM_FN_Prop (/FortniteRigs/Modules/FkSolves/Prop/CRM_FN_Prop)** — *Prop/weapon control rig (no bones driven). Its controls serve as IK spaces for the arms. Used by MR_FN_Biped, MR_Zebra, MR_Monster and CRU_PropAim.*. Connectors: Parent (primary); Spaces (optional array); Control Vis Channel Host (optional, RigTypeConnectionRule ElementType=Control). Config: Control Scale=1, Rotation Order=YXZ. Construction (139 nodes): the Aim control; the chain Prop Global > Prop Local > Prop > Prop Attach 01/02; per-level 'Change Pivot' bool and 'Aim Weight' float channels; buffer nulls Aim Global/Local/Prop Buffer; visibility channels hosted on the controls (or on root/Global); control colors stored in metadata; spaces. Forwards (146 nodes): gated visibility, change-pivot compensation (stateful, with recolor and autokey), three chained Aim Solve calls, and forced visibility while a pivot change is active. No backwards solve. Local functions: Get Control Shape, Reset Controls Color, Aim Solve (45 nodes). Runtime: 6 controls, 11 channels, 3 nulls.
- **CRM_FN_Pin (/FortniteRigs/Modules/Miscellaneous/CRM_FN_Pin)** — *Utility that copies global transforms from bone to bone ('IK Bone Pins' in MR_FN_Biped: hand_r->ik_hand_gun, hand_l->ik_hand_l, foot_l->ik_foot_l, foot_r->ik_foot_r, hand_r->ik_hand_r).*. Connectors: Root (primary); Drivers and Driven (optional arrays). Construction (18 nodes) keeps the pairs (Drivers[i], Driven[i]) whose driver exists. Forwards (7 nodes): Driven[i] global = Drivers[i] global, without propagation. No controls, no backwards solve. Not used in MR_Zebra or MR_Monster.
- **CRFL_Hierarchy_v001 / CRFL_Control_v001 / CRFL_Math_v001 / CRFL_Module_v001 (functions used by the FK-family modules)** — *Shared function libraries*. CRFL_Hierarchy: Set Control Scale (graphs.txt:596), Get Array Parents (651), Get Control Color From Metadata (961), Get Control Color From Metadata v02 (1565), Has Side (1639), Construct Movable Proxy v01 (1010), Forward Movable Proxy v01 (1084). CRFL_Control: Set Mirror Axis (24), Set Mirror Behavior (35), Get Control Shape Name From Item v01 (70) and v02 (430), Control Color Override From Metadata v01 (116) and v02 (404), Evaluate Space Nulls v01 (268), Construct Space Nulls v01 (326); Create Gimbal Control (195) is used only by IkFk2Bones, not by the FK modules. CRFL_Math: Get Chain Length (16). CRFL_Module: Set Default Match To Connector v01 (361), Conform Name v01 (378), Get Item Name v01 (510).

### UE3 IK-family modules

- **CRM_FN_IkFk2Bones (/FortniteRigs/Modules/IkSolves/IkFk2Bones)** — *Two-bone IK/FK limb module (arms and legs). Used 4x in MR_FN_Biped, MR_FN_BipedDMC, MR_Zebra and MR_ZebraDMC.*. Connectors: Start (primary), Mid and End (ChildOfPrimary), Parent (optional), IK Spaces and FK Spaces (optional arrays). Construction: resolves Bones, builds 3 IK-plane virtual bones ('Virtual Bone A/B/C'), a Default FK Space null, an FK chain 'FK 0/1/2' with gimbals on FK 0 and FK 2 plus 'Upper/Lower Segment Scale FK' nulls, FK orient-space nulls, a Parent Buffer null and a translate-only 'IK Base' control, the 'IK' control with 'IK Gimbal' and an 'IK' effector null, an optional 'IK Rotation' (end-align) rotator and 'IK Rotation Null', 'PV Twist Start/End Null', 'Auto PV ' and 'Orient PV ' nulls, the 'PV' position control, per-space '<space> IK Null' and '<space> PV Null' nulls, and the 'Mid' null and 'Mid' control. It also adds the channels Ik Fk Switch, IK End Align, Stretch, Softness, PV Twist Follow, Upper/Lower Segment Scale, Sec Controls Vis and '<Module> Vis'. Pre Forwards Solve: reads IK Solve from the channel and publishes it as module metadata. Forwards Solve (292 nodes): module visibility gate, parent buffer, orient spaces, IK/FK visibility, then either IK (auto PV parent -> Soft IK -> IK end align -> FK auto-match) or FK (segment-scale nulls -> Compute FK -> IK auto-match). After that come the Mid-control re-aim, snapping bones to the virtual bones, auto-key on switch, debug drawing, and the IsInteracting metadata. Post Forwards Solve clears the Match/Key metadata flags. Backwards Solve (38 nodes): virtual bones follow the skeleton, then Match FK, Match IK and the segment-scale channels are computed from the pose. Utils graph has user events To IK, To FK and Key Controls. The local library holds 14 functions (Soft IK has 128 nodes; IK End Align Solve has 67).
- **CRM_FN_Foot (/FortniteRigs/Modules/Biped/Foot)** — *Reverse-foot / foot-roll module parented under an IkFk2Bones leg (Foot L and Foot R in the Biped and Zebra rigs).*. Connectors: Foot Joint (primary), Ball Joint (ChildOfPrimary), Toe Joints (optional array), and optional Toe Tip, Heel, Inner Bank and Outer Bank pivots. Construction reads the parent leg's module metadata (IK Null, IK Control, IK Driver, IK Solve). It builds a pivot null stack under the leg's IK gimbal: IK Foot Space > Toe Tip Pivot > Heel Pivot > Ball Pivot > Toe Tip Rocker Pivot > Heel Rocker Pivot > Inner Pivot > Outer Pivot > {Heel Lift > IK, Ball IK null, Toe IK null, Ball IK control}. Controls: Toe Tip, Heel, Ball, Foot Rocker, Ball IK, Toes FK, Toes IK, 'FK <i>' toes, and a Footprint Display proxy. Channels: Rocker Blend, Rocker Ball Rotation, Foot Pivot Control Vis, Footprint Vis. Pre Forwards Solve (runs before the leg's solve): reads the parent's IK Solve, sets visibility, reads control rotations into scalars (Set Foot Values), and in IK mode runs Set Foot Pivots, which ends by writing the leg's IK effector null. Forwards Solve: in IK mode FABRIK moves the foot so the ball reaches the Ball IK null, then aims the ball at the Toe IK null and applies the Toes IK control; in FK mode the ball follows Toes FK. It also re-anchors the foot rocker, drives the toe joints, and runs match/key when the parent's metadata flags are set. Backwards Solve: Match FK, Match IK, then toe FK controls follow the toe joints.
- **CRM_FN_LimbTwist (/FortniteRigs/Modules/Biped/LimbTwist)** — *Twist-bone distribution module (Arm/Leg Upper/Lower Twist L/R: 8 per biped rig).*. Connectors: Start, Parent (optional), End (ChildOfPrimary). Construction finds twist bones among Start's direct children whose names contain all '|'-separated tokens of 'Twist Search String'. It computes translate weights, then builds 'Twist Parent Null' (under Parent) plus 'Twist <n> Null' and a 'Twist <n>' offset control per twist bone. Forwards Solve: snaps the twist parent null to Start. It then extracts the swing-twist of the driver (Start, or End when reversed) relative to bind and slerps a weighted fraction of it onto each twist null. When Use Translates is on, it position-constrains the nulls between Start and End. Twist bones then follow the twist controls, and the Start bone's own twist is removed when not reversed. Backwards Solve re-derives the nulls and controls from the bones. The Connector event sets the default matches.
- **CRM_FN_BipedStretchFeedback (/FortniteRigs/Modules/Biped/StretchFeedback)** — *Visual squash/stretch feedback module (1 instance in the MR_FN_Biped and BipedDMC templates, none in the Zebra rigs).*. Connectors: Root, Spine Elements, Arm Elements, Leg Elements, Vis Channel Control. Construction resolves the element arrays and derives the right-side arrays by renaming '_l' to '_r'. If the vis connector is connected, it spawns the bool channel 'Stretch FeedBack Vis' under it. Forwards Solve calls the local 'Stretch Feedback' function 5 times: spine, arm L, arm R, leg L, leg R. For each consecutive element pair it compares the current segment length to the initial length and draws a colored debug line. It writes nothing back to the rig.
- **CRM_FN_ProxyControl (/FortniteRigs/Modules/Miscellaneous/CRM_FN_ProxyControl)** — *Relative 'delta' proxy control driving several controls (Meta/Finger Spread and Curl: 12 in the Biped templates).*. Connectors: Parent (primary), Driven Controls (optional array), Snap To (optional). Construction spawns '<Proxy Name> Null' under Parent at the Snap To (or Parent) initial transform. Under it goes a transform control '<Proxy Name>' with bIsProxy=true, DrivenControls set to the driven list, and ShapeVisibility=UserDefined, plus a bool channel 'Pivot Vis', which no solve reads. Forwards Solve: while this control is being interacted with, it computes the per-evaluation local delta against the previous local value. That delta, weighted by a float curve over the driven-list ratio, is applied as an offset to each driven control. Otherwise the proxy re-snaps to Snap To (offset = Proxy Offset Transform * SnapTo) and resets to identity. The Backwards Solve is empty.

### UE4 CRFL libraries

- **CRFL_Control_v001 (/FortniteRigs/Libraries/CRFL_Control_v001)** — *Shared function library: control creation and display helpers (mirror metadata, shape name/colour resolution, shape scaling, gimbal controls, space nulls)*. A ControlRigBlueprint function library. It is not a module. Its hierarchy holds only the Manny preview skeleton (89 bones, 126 curves). It has 11 public functions: Set Mirror Axis, Set Mirror Behavior, Add Mirror Tag, Get Control Shape Name From Item v01/v02, Control Color Override From Metadata v01/v02, Scale Control Shape v01, Create Gimbal Control, Evaluate Space Nulls v01 and Construct Space Nulls v01. Its RigVMModel (Forwards Solve) and RigVMModel Rig (Construction) graphs are only test stubs: Print nodes and an Item Array of clavicle_l/upperarm_l/lowerarm_l. The heaviest callers are Evaluate Space Nulls v01 (16 call sites in 4 modules), Get Control Shape Name From Item v02 (9 sites), Set Mirror Behavior (10 sites) and Construct Space Nulls v01 (8 sites). Add Mirror Tag and Scale Control Shape v01 have no callers. Most functions run in the Construction event. Evaluate Space Nulls runs in Forwards and Backwards Solve. Evidence: dump FortniteRigs__Libraries__CRFL_Control_v001/summary.json 'local_functions'; graphs.txt line 11 '### GRAPH ...RigVMFunctionLibrary'.
- **CRFL_Debug_v001 (/FortniteRigs/Libraries/CRFL_Debug_v001)** — *Shared function library: debug visualisation*. Holds a single function, Draw Axis (13 nodes). For each item in the input array, it calls GetTransform (global) and then DebugTransformMutableNoSpace in Axes mode, gated by a Branch on the Enable input. CRM_FN_LimbTwist calls it once (inside its local Blend Twist) and CRFL_Module_v001 calls it once (inside its own Blend Twist). Its event graphs are empty stubs. Evidence: graphs.txt lines 11-46.
- **CRFL_Hierarchy_v001 (/FortniteRigs/Libraries/CRFL_Hierarchy_v001)** — *Shared function library: procedural hierarchy construction, colour/visibility, movable pivots, IK-plane and pole-vector helpers*. The largest library, with 23 public functions: Add Null Above, Add Null Below, Control Stack at Position, Control Stack at Item, Set Contol Color By Position, Set Control Scale, Get Array Parents, Create IK Plane Virtual Bones (v01 and v02), Get Mirror Transform, Get Children by Contained Strings, Switch Control Visibility, Get Control Color From Metadata (v01 and v02), Construct/Forward Movable Proxy v01, Compute/Construct Auto Pole Vector v02, Compute Pole Vector From Plane v01, Compute Pole Vector Location v01/v02, Has Side and Compute Pole Vector v01. The most-used are Get Control Color From Metadata (16 call sites across 7 modules plus CRFL_Control) and Set Control Scale (9 modules, one call each). Movable Proxy is used by Spine and Body. Create IK Plane Virtual Bones v02, Compute Pole Vector Location v02 (2 calls) and Switch Control Visibility are used by IkFk2Bones. Get Array Parents is used by LimbTwist (2) and FkArray (1). The internal call graph is: Control Stack at Item -> Control Stack at Position -> Add Null Above; IK Plane Virtual Bones -> CRFL_Math Project Middle Bone to IK Plane; Compute PV Location v02 -> Compute Pole Vector v01 (x2); Get Control Color v02 -> Has Side (x2); Construct Auto PV v02 -> Compute PV From Plane v01. Its unit usage is dominated by GetTransform (26), NameConcat (21), Sequence (18), Branch (17), VectorSub (12) and AimBoneMath (10). The Forwards Solve test graph calls Get Children by Contained Strings(upperarm_l, 'lowerarm|twist'). Evidence: summary.json; graphs.txt lines 10-33.
- **CRFL_Math_v001 (/FortniteRigs/Libraries/CRFL_Math_v001)** — *Shared function library: chain measurement, twist extraction and IK-plane math*. Holds 4 functions: Get Chain Length (used by Foot, LimbTwist, Spine, Root and IkFk2Bones to derive shape auto-scale factors), Compute Pole Vector (no callers), Project Middle Bone to IK Plane (called by both CRFL_Hierarchy Create IK Plane Virtual Bones versions) and Get Node Twist Value (called by the LimbTwist-local Blend Twist and by CRFL_Module Blend Twist). Its Forwards Solve test graph calls Project Middle Bone to IK Plane on root_ctrl, root_ctrl_2 and root_ctrl_2_2 with Debug=true. Evidence: graphs.txt lines 4, 11-14.
- **CRFL_Module_v001 (/FortniteRigs/Libraries/CRFL_Module_v001)** — *Shared function library: modular-rig helpers (connector default match, naming, twist/position blending)*. Holds 7 functions: Blend Twist, Connect to Module Metadata, Blend Position, Create FK Chain Controls, Set Default Match To Connector v01, Conform Name v01 and Get Item Name v01. The used ones are Set Default Match To Connector v01 (LimbTwist x3, FkChain x2; connector event only), Conform Name v01 (FkArray x2, FkChain x1, plus Get Item Name) and Get Item Name v01 (Spine, FkChain and IkFk2Bones, once each). Blend Twist, Blend Position, Connect to Module Metadata and Create FK Chain Controls have no module callers; LimbTwist carries its own local 'Blend Twist' and 'Blend Translate'. Blend Twist calls CRFL_Math Get Node Twist Value and CRFL_Debug Draw Axis. Create FK Chain Controls calls CRFL_Hierarchy Control Stack at Item. Evidence: graphs.txt lines 11-18; functions_used.json per_rig.

### UE5 body deformation and runtime

- **CR_Zebra_Deform** — *Post-process deformation rig (ControlRigRuntimeAsset) for SKM_Zebra / SKM_Zebra_Hi, run by AnimBP_Zebra*. Path convention: '<dump>/' = <dump>. Has no controls: 50 static CURVE elements (<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/hierarchy.txt) and 8 private variables (7 FRigElementKey null handles plus an unused double TwistFactor_Value). Construction event (graphs.txt:679-717): PrepareForExecution, then HierarchyImportFromSkeleton (bIncludeCurves=true, bIncludeVirtualBones=True, bIncludeMeshSockets=False), then 7 HierarchyAddNull calls under bone 'head' (GlobalSpace) whose keys are stored in variables. Forward Solve graph (284 nodes, 413 links): 45 SphericalPoseReader, 82 ModifyTransforms (all single-item, AdditiveLocal, weight range [0,1]), 54 SetCurveValue (50 on the execution path, so each of the 50 curves is written once), 10 Remap, 2 Min, 1 ChainInfo, 7 AddOptimusDeformer, 7 GetCurveValue, 9 GetTransform, 2 Sequence. Sequence.A runs the whole pose-reader/corrective chain (187 executed items). Sequence.B then adds the 7 head/muzzle/skull Optimus deformers. There is no backward solve.
- **CR_Monster_Deform** — *Post-process deformation rig for SKM_Monster, run by Monster_PostAnimBP*. ControlRigRuntimeAsset with a static hierarchy (165 BONE, 74 CURVE) and no construction event (<dump>/Game__Assets__Monster__Rig__CR_Monster_Deform/summary.json). Forward Solve has 10 nodes: 4 SphericalPoseReaders on clavicle_l/r (clavicle up and down) driving 4 AdditiveLocal ModifyTransforms on the helper bones def_trap_l/def_trap_r. It sets no curves and adds no deformers. Node names (ModifyTransforms_2_2_2_x, SphericalPoseReader_1_1_1_1_x) and the def_trap rest transforms match Zebra's clavicle section and def_strap bones, so it was copied from CR_Zebra_Deform.
- **AnimBP_Zebra** — *Post-process Animation Blueprint assigned to SKM_Zebra and SKM_Zebra_Hi*. AnimGraph: LinkedInputPose_0 -> AnimGraphNode_ControlRig_1(CR_Zebra_Deform) -> Root (<dump>/Game__Assets__Zebra__Rig__AnimBP_Zebra/asset.t3d:62-106). The only non-default node property is ControlRigAssetReference (line 67), so every other setting is an engine default: Alpha=1, bResetInputPoseToInitial/bTransferInputPose/bTransferInputCurves=true, LODThreshold=-1, bSetRefPoseFromSkeleton=false. The event graph (BlueprintUpdateAnimation, TryGetPawnOwner) is disabled. TargetSkeleton is SK_Zebra.
- **Monster_PostAnimBP** — *Post-process Animation Blueprint for SKM_Monster*. Same topology as AnimBP_Zebra: LinkedInputPose -> ControlRig(CR_Monster_Deform) -> Root (<dump>/Game__Assets__Monster__Rig__Monster_PostAnimBP/asset.t3d:74). All node settings are defaults and the event graph is disabled. TargetSkeleton is Monster_Skeleton. The ControlRig node has a stale ErrorType=1 serialized (line 97).
- **SKM_Zebra / SKM_Zebra_Hi** — *Runtime skeletal meshes*. Both use skeleton SK_Zebra (371 bones), post-process ABP AnimBP_Zebra and physics asset SKM_Zebra_Physics, with default_mesh_deformer=None, 1 LOD and 8 foot pivot sockets on ball_l/ball_r. SKM_Zebra: 34,459 verts, 132 morph targets (49 body correctives + 83 face shapes), 8 named materials. SKM_Zebra_Hi: 132,333 verts, 130 morph targets (lacks lip_all_up_backup/_2/_3 and lip_all_dn_backup; adds brow_in_up_l_backup and brow_in_dn_l_backup), materials named Material..Material_6.
- **SKM_Monster** — *Runtime skeletal mesh plus UE->USD export*. Skeleton Monster_Skeleton (165 bones: root/pelvis/spine_01-05/neck/head/clavicles/def_trap plus 152 head-subtree bones; no arms or legs). 107,867 verts, 1 LOD, 79 face-only morph targets. Post-process ABP Monster_PostAnimBP, physics SKM_Monster_Physics, default_mesh_deformer=None. USD export (monster_usd_tree.txt): SkelRoot/Skeleton(165 joints)/Mesh with 5 GeomSubsets+UsdPreviewSurface-style materials, 79 BlendShapes, 12 joint influences per vertex, no animation.
- **SK_Zebra / SK_ZebraHi / Monster_Skeleton** — *Skeleton assets*. SK_Zebra: 371-bone tree plus 8 foot sockets (foot_{l,r}_{inner,outer,heel,toe_tip} on ball_{l,r}), with an empty AnimCurveMetaData. SK_ZebraHi is an identical copy except that it has no PreviewSkeletalMesh, and no mesh references it. Monster_Skeleton: 164-entry BoneTree plus CurveMetaData flagging smile_l, brow_dn_l, stitches and brow_up_l as bMorphtarget.
- **IK_Zebra** — *IK Rig used as the retarget target*. One IKRigFullBodyIKSolver (root pelvis, 20 iterations, 10 sub-iterations, no stretch). 4 effector goals (hand_l/r, foot_l/r, ChainDepth 2). Bone settings: clavicles and pelvis have RotationStiffness 0.95; lowerarm and calf have X/Y locked with PreferredAngles Z=90. 14 retarget chains (Spine, Neck, L/R Clav, L/R Arm, L/R Pinky, L/R Index, L/R Thumb, L/R Leg); PelvisBone=pelvis.
- **RTG_UEFN_to_Zebra** — *IK Retargeter from the UEFN mannequin to Zebra*. 9 retarget ops in this order: Pelvis Motion, FK Chains (14 chains), Run IK Rig (arms and legs), Blend to Source and Offset Goals (both children of Run IK Rig), Body Intersect IK (uses PA_Zebra), Root Motion, Remap Curves (copy all source curves), Filter Bones (neck_01, neck_02, head). TargetMeshOffset X=85.39. The retarget pose has rotation offsets on legs, arms, fingers, spine and pelvis.
- **PA_Zebra / PA_Zebra_Phys_Asset_Detailed / SKM_Zebra_Physics / SKM_Monster_Physics** — *Physics assets*. SKM_Zebra_Physics is assigned to both Zebra meshes: 22 capsule bodies (including skull and skull_tp) and 21 constraints. PA_Zebra has 21 capsules and 20 constraints, all with ACM_Limited swing1/swing2/twist; RTG Body Intersect uses it. PA_Zebra_Phys_Asset_Detailed has 27 convex bodies and 26 constraints, and its preview mesh is Geo/Test/Zebra. SKM_Monster_Physics has a single capsule 'spine_05_capsule' (radius 37.0, length 22.4).
- **Zeb_Face_Expressions** — *Baked AnimSequence on SK_Zebra*. 163 frames, 6.79 s. 371 bone tracks, including every def_* and twist helper bone, and 102 float curves: 39 body-corrective curves (the CR_Zebra_Deform names; 'squetch' in lower case), 54 face morph curves, 6 deformer-driver curves (head_bend, head_squash, muzzle_bend_deformer, muzzle_squash_deformer, skull_tp_bend_deformer, skull_tp_squash_deformer) and material/other curves (cornea_size, pupil_dilation, lip_roll_in_bt_l). There are no vector or transform curves.
- **Level sequences (MR_Zebra_Take1, zebra_audition, zebra_marketingPoseFaces, expression_demo_seq) + MRG configs** — *Cinematic runtime integration*. Characters are animated with MovieSceneControlRigParameterTrack named MR_Zebra (and MR_Boombox for the prop), next to empty SkeletalAnimation tracks. The zebra_audition binding 'SKM Zebra' also has Spawn and Transform tracks. expression_demo_seq only animates camera and eye-material parameters (Highlight_Intensity, Cornea Roughness, Sclera Color Multiply RGBA, Highlight_Shape_Pos RGBA) and pulls in subsequences. The MovieRenderGraph assets are render-pipeline configs only.

### UE6 face rigs

- **CRM_Zebra_Face (/Game/Assets/Zebra/Rig/CRM_Zebra_Face, ControlRigRuntimeAsset)** — *Face module of MR_Zebra and MR_ZebraDMC. Module name 'Face', parent module root, Face/Root->bone root, Face/Parent->bone head. Parent module of the Ear Base L/R and Mohawk FK modules.*. Static hierarchy: CONNECTOR Root (primary), CONNECTOR Parent (Secondary, optional, rule RigChildOfPrimaryConnectionRule), SOCKET Root. No authored controls; everything is spawned by the Construction event. 88 member variables (element-key handles, lid quaternion tables, shape-offset vectors, three user-struct weight tables, Jaw Normalize, DMC flags). 23 local library functions. ConstructionGraph has 274 nodes: PrepareForExecution, then Sequence A (head attach null, squash controls, jaw/muzzle/mouth/nose/lips/reverse-jaw/skull-tp/cheeks), B (corners, brows, squeeze, teeth, tongue loop, 20-lip tweaker loop), C (8 Spawn Lid Bones and Controls calls plus Lid In/Ot), E (eye main, lid sliders, eye nulls/aim/controls, pupil/iris channels, Eye Aim, Add Null Above). The Forwards Solve graph has 565 nodes: bone drivers, visibility switches and the squeeze null run first, then aggregate Sequence pins A..K (A brows/corners/sneer/lip curves, B lip constraints, C lip rolls, D correctives, E New Parent, F jaw-open reader, G lip tweakers, H lids, I soft eyes, J lid skin constraints, K squash curves, pupil/cornea curves, eye aim and convergence). There is no Backwards Solve. In MR_Zebra the module creates 107 controls (20 of them animation channels), 32 nulls and 31 bones. Unit counts: SetCurveValue 90, GetCurveValue 42, ModifyTransforms 60, ParentConstraint 34, SphericalPoseReader 11, Remap 28, DoubleMul 53, ProjectTransformToNewParent 16, SetTransform 17. It writes 87 distinct curve names, 76 of which are SKM_Zebra morph targets. It adds no Optimus deformers; CR_Zebra_Deform reads its head_/muzzle_/skull_tp_ deformer curves.
- **CRM_Monster_Face (/Game/Assets/Monster/Rig/CRM_Monster_Face, ControlRigRuntimeAsset)** — *Face module of MR_Monster. Parent module Spine; only Face/Root is connected (to bone root); Face/Parent is left unconnected.*. A fork of the Zebra face. The static hierarchy adds 96 CURVE elements. It has 108 member variables: the Zebra set plus Mouth Squash, the *_Null deformer handles, Ch/Puff Lip Tp/Bt, Nose Sneer/Flare L/R, Squint L/R and Sticky L/R; the DMC variables are removed. It has 26 local functions: Get DMC Shape is removed, and Lip Puff Tp/Bt and Lip Ch Tp/Bt are added. The ConstructionGraph has 285 nodes. Sequence A adds Mouth Squash, Nose channels and Squint channels, B adds Sticky channels, D covers the eyes, and E spawns 9 deformer nulls under the head bone. The Forwards Solve graph has 670 nodes. It first sets curve model_edits=1, adds nose/sticky curves on pin A, lip puff/ch on pin C, and lid_bt blink/extend curves on pin H. Pin K is the same as Zebra, and pin L adds *_deformer curves and 9 RigUnit_AddOptimusDeformer (head/headbend/headtwist/muzzlebend/muzzlesquash/mouthbend/mouthsquash/skulltpbend/skulltpsquash). In MR_Monster the module creates 120 controls (32 animation channels), 41 nulls and 31 bones. It uses SetCurveValue 117 times and writes 107 distinct curve names, 75 of which are SKM_Monster morph targets.
- **Lid_Tp_Struct / Lid_Bt_Struct (UserDefinedStruct)** — *Per-skin-bone parent-weight rows for eyelid parent constraints*. Each struct has five doubles, Value1..Value5 (default 0). Lid_Tp_Struct (guid 5C9296D0...) is the element type of the member variables 'Lid Tp Struct' AND 'Lid Bt Struct'. Lid_Bt_Struct (guid 33818006...) has an identical layout but is unused by the face rigs. Value1..5 map to the parents [Lid In, Lid Ot, micro 01, micro 02, micro 03] of ParentConstraint_158..165. The row values are only fully visible in the MR_*/asset.t3d module property bags.
- **Lip_Null_Struct (UserDefinedStruct)** — *Per-lip-tweaker-null parent-weight rows*. Four doubles, Value1..Value4, mapping to the parents [Skull Const, Jaw Const, Corner L, Corner R] of ParentConstraint_22. There are 20 rows, one per Lip Control Null.
- **MR_Zebra runtime_hierarchy Face/* (and MR_Monster)** — *Post-construction result of the face module*. Zebra has 172 Face/ lines: 107 CONTROL, 32 NULL, 31 BONE, 2 CONNECTOR. Monster has 120 CONTROL, 41 NULL, 31 BONE, 2 CONNECTOR. Element names are prefixed Face/, and duplicate channel names get suffixes (Sneer Tp_2, Micro Vis_2, Brow Tweaker Vis_2, Sticky_2). All transform controls are EULER_TRANSFORM ANIMATION_CONTROL with preferred rotation order YZX. Animation channels are FLOAT/BOOL ANIMATION_CHANNEL children of their host control.
- **SKM_Zebra / SKM_Monster (morph targets)** — *Curve consumers*. SKM_Zebra has 132 morphs (body correctives plus about 80 face shapes, including squint_l/r, which the face never drives, and frown_wide_c_r, which is never written because of a bug). SKM_Monster has 79 morphs: nose_*, lip_puff_*, lip_stick_*, ch_bt, open_frown_c_*, model_edits, lid_bt_blink_*, lid_tp_blink_extend_*, plus all_up/all_dn/mid_up/mid_dn, which are not driven. Monster has no smile_wide/wide_open/brow_dn_c combination morphs even though the face still writes those curves.

### UE7 deformer graphs and DMC

- **ZebraHead_DeformerGraph (/Game/Assets/Zebra/Rig/Deformers)** — *GPU deformer: head squash/stretch*. UpdateGraph: Read Skinned Mesh -> DG_Function_SquashStretch (LengthToDeform=55, XYBias=0.5, ZBias=0.5, LimitFromBottom/Top=0, EnableDebugDraw=0) -> DG_Function_ComputeNormalsTangentsAndKeepInputNormals -> Write Skinned Mesh. Vertex mask: Skin Weights as Vertex Mask with BoneNames=[head]. Variables: Transform (FTransform, default T(0,0,80)) and StretchFactor (double, 0.5). It has a dangling FunctionReferenceNode_0 to Monster_Head_DeforerGraph shown as '<graph missing>'. The compiled graph has 2 kernels (no CacheGeometry pass), so OriginalPosition/TangentX/TangentZ are read straight from Read Skinned Mesh. The T3D has no Status= line, so the asset is in the default Modified state. It is used 1st in the CR_Zebra_Deform chain.
- **ZebraHeadTwist_DeformerGraph** — *GPU deformer: head twist*. Read Skinned Mesh -> CacheGeometry (dangling reference into Monster_Head_DeforerGraph; the compiled pass-through kernel is present) -> DG_Function_Twist (LengthToTwist=60, MaxTwistAngle=135 int, limits 0) -> ComputeNormalsKeepInput -> Write. Mask is [head]. Variables: Transform (T(0,0,80)) and TwistFactor (0). The compiled graph has 3 kernels and Status=Compiled. It is used 2nd in CR_Zebra_Deform.
- **ZebraHeadBend_DeformerGraph** — *GPU deformer: head bend*. Read -> CacheGeometry -> DG_Function_Bend (LengthToBend=90, MaxBendAngle=180 int, limits 0, debug 0) -> ComputeNormalsKeepInput -> Write. Mask is [head]. Variables: Transform (rotation quat(0,0,0.7071,0.7071) = 90 deg about Z, T(0,0,90)) and BendFactor (0.5). The const node default 0.163201 is overridden because the variable is linked into its Value pin. The graph has 3 kernels (<dump>/.../kernels.hlsl) and 14 compute data interfaces (SkinnedMeshRead, 4 TransientBuffer, 3 CustomComputeKernel, GraphDI, SkinWeightsAsVertexMask, DebugDraw, SkinnedMesh, HalfEdge, SkinnedMeshWrite). It is used 3rd in CR_Zebra_Deform.
- **ZebraMuzzleSquash_DeformerGraph** — *GPU deformer: muzzle squash*. Same topology as ZebraHead plus CacheGeometry, with SquashStretch L=55 and XY/ZBias=0.5. Mask has 25 bones: muzzle, lip_bt, lip_bt_01..04_l/r, lip_tp, lip_tp_01..04_l/r, nose, teeth_tp, teeth_bt, jaw, lip_corner_l, lip_corner_r. Variable defaults: Transform T(0,0,80) and StretchFactor 0.5. The const default Transform is rot(0,-1,0,0) T(0,0,110). It is used 4th in CR_Zebra_Deform.
- **ZebraMuzzleBend_DeformerGraph** — *GPU deformer: muzzle bend*. Bend with L=90 and MaxBendAngle=180, using the same 25-bone muzzle mask. Variable Transform default is rot(-0.7071,-0.7071,0,0) T(0,20,110), and BendFactor default is 0. It is used 5th in CR_Zebra_Deform.
- **ZebraMuzzleTwist_DeformerGraph** — *GPU deformer: muzzle twist (unused)*. Twist with L=60 and MaxTwistAngle=135, using the same 25-bone muzzle mask. Variables: Transform rot(0,-1,0,0) T(0,0,80) and TwistFactor 0. No RigUnit_AddOptimusDeformer in any rig references it.
- **ZebraSkullTpSquash_DeformerGraph** — *GPU deformer: skull-top squash*. SquashStretch with L=55 and biases 0.5. Mask is [skull_tp, eye_main_l, eye_main_r, ear_base_l, ear_base_r]. Variables: Transform T(0,0,100) and StretchFactor 0.5. It is used 6th in CR_Zebra_Deform.
- **ZebraSkullTpBend_DeformerGraph** — *GPU deformer: skull-top bend*. Bend with L=90 and 180 deg, using the same 5-bone skull mask. Variables: Transform (90 deg about Z, T(0,0,100)) and BendFactor 0.5. Preview mesh is SKM_Zebra. It is used 7th (last) in CR_Zebra_Deform.
- **Monster_Head_DeforerGraph (/Game/Assets/Monster/Deformers)** — *GPU deformer: head squash (reference implementation)*. Read -> SubGraphNode 'CacheGeometry' (local SubGraph: a CustomComputeKernel 'MyKernel' that copies Position/TangentX/TangentZ to OutPosition/OutTangentX/OutTangentZ) -> SquashStretch (L=75, biases 0.5) -> ComputeNormalsKeepInput -> Write. Mask is [head]. Variables: Transform (default T(0,0,80); const default T(0,0,20)) and StretchFactor 0.5. It also contains an unused Function graph 'NewFunction' with the same pass-through kernel; its GUID EE5E6AF5 does not match the GUID 1ED32A22 that the Zebra graphs reference. Preview mesh is SKM_Monster. It is used 1st in CRM_Monster_Face.
- **Monster_HeadBend_DeforerGraph** — *GPU deformer*. CacheGeometry subgraph -> Bend (L=90, 180 deg) -> normals. Mask is [head]. Variables: Transform (90 deg Z, T(0,0,20)) and BendFactor 0. It is used 2nd in CRM_Monster_Face.
- **Monster_HeadTwist_DeformerGraph** — *GPU deformer*. CacheGeometry -> Twist (L=175, 135 deg) -> normals. Mask is [head]. Variables: Transform T(0,0,20) and TwistFactor 0. It is used 3rd.
- **Monster_MuzzleBend_DeforerGraph** — *GPU deformer*. Bend with L=90 and 180 deg. Mask has 27 bones: lip_bt(+01..04 l/r), lip_tp(+01..04 l/r), nose, teeth_tp, teeth_bt, lip_corner_l/r, cheek_l/r, muzzle, jaw. Variable Transform default is rot(-0.7071,-0.7071,0,0) T(0,20,110). It is used 4th.
- **Monster_MuzzleSquash_DeforerGraph** — *GPU deformer*. SquashStretch with L=55. Mask has 26 bones (the muzzle-bend list minus 'muzzle'). It is used 5th.
- **Monster_MouthBend_DeforerGraph** — *GPU deformer (Monster only)*. Bend with L=90 and 180 deg. Mask has 26 bones (the muzzle-bend list minus 'jaw'). Its CacheGeometry TangentZ output is unconnected, so the compiled kernel stubs WriteOutTangentZ and the normals pass reads OriginalTangentZ directly from Read Skinned Mesh. It is used 6th.
- **Monster_MouthSquash_DeforerGraph** — *GPU deformer (Monster only)*. SquashStretch with L=55. Mask has 25 bones (no muzzle and no jaw). It is used 7th.
- **Monster_SkullTpBend_DeforerGraph** — *GPU deformer*. Bend with L=90 and 180 deg. Mask is [skull_tp, eye_main_l, eye_main_r], without the ear bones that Zebra includes. It is used 8th.
- **Monster_SkullTpSquash_DeforerGraph** — *GPU deformer*. SquashStretch with L=40. Mask is [skull_tp, eye_main_l, eye_main_r]. Variable Transform default is T(0,0,45) and StretchFactor 0.5 (const default 0.55). It is used 9th (last).
- **CR_Zebra_Deform (ControlRigRuntimeAsset, post-process rig in AnimBP_Zebra)** — *deformer driver at runtime*. Construction event: ImportSkeleton (curves included), then 7 HierarchyAddNull calls under bone 'head' with Space=GlobalSpace, each stored in an FRigElementKey variable. Forwards Solve: Sequence A runs the pose-reader correctives (outside this scope). Sequence B runs a chain of 7 RigUnit_AddOptimusDeformer nodes in this order: Head(squash) -> HeadTwist -> HeadBend -> MuzzleSquash -> MuzzleBend -> SkullTpSquash -> SkullTpBend. Each node's Transform trait comes from GetTransform(null, GlobalSpace), and its factor comes from GetCurveValue, remapped for squash. All settings are AfterDefaultDeformer, ExecutionGroup=1, DeformChildComponents=True. units_used shows 7 AddOptimusDeformer nodes.
- **CRM_Monster_Face (face module inside MR_Monster)** — *deformer driver inside the modular rig*. Construction event spawns 9 deformer-origin nulls under 'head' (GlobalSpace) and 4 squash controls (Head Squash, Skull Tp Squash, Muzzle Squash, Mouth Squash). Forwards Solve: each control's local transform is remapped and written to *_deformer curves (SetCurveValue_39..79). The same graph then reads those curves and runs 9 AddOptimusDeformer nodes: Head -> HeadBend -> HeadTwist -> MuzzleBend -> MuzzleSquash -> MouthBend -> MouthSquash -> SkullTpBend -> SkullTpSquash. CR_Monster_Deform, the post-process rig, has no AddOptimusDeformer.
- **CRM_Zebra_Face (Face module of MR_Zebra / MR_ZebraDMC)** — *control -> deformer-curve driver*. Construction spawns the Head Squash, Muzzle Squash and Skull Tp Squash controls. Forwards Solve writes the curves head_squash, head_twist, head_bend, muzzle_squash_deformer, muzzle_bend_deformer, skull_tp_squash_deformer and skull_tp_bend_deformer from those controls' local transforms. The Zebra face rig adds no deformers itself; CR_Zebra_Deform consumes the curves in the post-process ABP.
- **RigUnit_AddOptimusDeformer (<UE>/Plugins/Animation/DeformerGraph/Source/OptimusCore/Private/ControlRig/RigUnit_Optimus.h/.cpp)** — *engine rig unit*. 'Add Deformer' mutable unit. It has trait-based pins: a DeformerGraphAsset trait (soft UOptimusDeformer), a Settings trait (ExecutionPhase, ExecutionGroup, DeformChildComponents, ExcludeChildComponentsWithTag), and one SetDeformer<Type>Variable trait per deformer variable, named after the variable. Every evaluation it enqueues the instance into the component's UOptimusDeformerDynamicInstanceManager and pushes variable values. On the game thread it adds the deformer instance once per component.
- **CRM_FN_DMC (/FortniteRigs/Modules/Miscellaneous)** — *DMC enabling module*. Experimental RigModule ('Add directly below the CRM_FN_Root module to enable DMC FK and IK layers'). It has a Root connector and a Root socket. Construction: if var 'Direct Mesh Control' (default True) is set, it runs SetupShapeLibraryFromLayer('ik-layer') and SetupShapeLibraryFromLayer('fk-layer'). Each layer name that yields groups is added to 'Direct Mesh Control Libraries'. The module then sets module metadata in the Root namespace: 'Direct Mesh Control Libraries' (FName array) and 'Direct Mesh Control' (bool). Its forward solve is empty.
- **RunDMC (/Game/BonusContent/Modules)** — *bonus DMC module (unused by any rig)*. RigModule with a single connector 'Control' (rule: ElementType=Control). Construction: if the connector is connected and var Name != '', it runs SetupShapeLibraryFromLayer(LayerName, default 'dmc-polygroup'). It then calls HierarchySetShapeSettings on the resolved control with shape Name=<Name>, color red and visible. The module description reads: 'Assign to any control, that control will be converted to use a DMC set'.
- **MR_ZebraDMC / MR_FN_BipedDMC** — *DMC template variants*. Identical to MR_Zebra (34 modules) and MR_FN_Biped (48 modules) except for three things. Each adds a 'CRM_FN_DMC' module (parent 'root', connector CRM_FN_DMC/Root -> bone root, binding ("Direct_Mesh_Control","Direct_Mesh_Control")). Each adds a rig variable Direct_Mesh_Control (bool, True). Each adds ModularRigGizmoLibrary_DMC to ShapeLibraries: at index 0 in MR_ZebraDMC, at index 1 in MR_FN_BipedDMC. Runtime control counts match the non-DMC rigs (359 and 283). In the headless dump no control resolved to an ik-layer/fk-layer shape.
- **ModularRigGizmoLibrary_DMC (/FortniteRigs/Controls)** — *gizmo library*. The same 62 shape names and proxies as ModularRigGizmoLibrary (pins shapes point to /EpicControlRig instead of /FortniteRigs). Differences: DefaultMaterial=ModularRigGizmoMaterial_DMC, XRayMaterial=/EpicControlRig ModularRigXRayMaterial, and it adds MaterialHoveredParameter='Hovered' and MaterialHoveredColorParameter='HoveredColor'. DefaultShape is Sphere_solid scaled 0.1.
- **DirectMeshControl engine plugin (<UE>/Plugins/Experimental/Animation/DirectMeshControl)** — *engine plugin (experimental, editor-only modules)*. Two Editor-type modules. DirectMeshControl contains the polygroup modeling tool, sub-mesh generation, DMC component and Optimus component source. DirectMeshControlRig contains RigUnit_SetupShapeLibraryFromLayer and the UDirectMeshControlProxy -> UDirectMeshControlComponent provider registration. Content: DG_DirectMeshControl(+NoColor) deformers, M_DirectMeshControl(+Hover) materials, and a Cardbox sample rig. Dependencies: GeometryProcessing, MeshModelingToolset, SkeletalMeshModelingTools, ControlRig, RigVM, ComputeFramework, DeformerGraph.

### UE8 coverage pass

- **/Game/Sequences/zebra_audition (LevelSequence, 7.9 MB)** — *main animated shot: Zebra + Boombox*. created 2026.06.09. Spawnables: 'SKM Zebra' (MR_Zebra Control Rig track, Transform track, and an empty SkeletalAnimation track with 0 sections), 'MR_Boombox' (MR_Boombox Control Rig track, Transform, Spawn), static-mesh props stool and Boombox_proxy (Transform + Spawn each), lights and a cine camera, plus a Camera Cut master track. Each Control Rig track has 1 section. The name table contains ControlChannelMap/ChannelMapInfo, ControlsRotationOrder with EEulerRotationOrder XYZ/XZY/YXZ/YZX/ZYX, SpaceChannels with EMovieSceneControlRigSpaceType Parent/World/ControlRig, and Bool/Enum/Scalar/Vector/Transform parameter curves. It has no constraint channels, no IntegerParameterNamesAndCurves, no Vector2D curves, and no EMovieSceneBlendType value names. It also stores AIESelectionSets, AnimLayers and a serialized ModularRig snapshot (ModularRigModel, ConnectionList, ConfigOverrides, VariableBindings).
- **/Game/MR_Zebra_Take1 (LevelSequence, 392 KB)** — *take on a possessed SkeletalMeshActor_1*. created 2026.06.02. Binding MR_Zebra: Transform track, an empty SkeletalAnimation track, and an MR_Zebra Control Rig track with 1 section. The name table has no space channel struct names (only the SpaceChannelIndex field), so no space keys. It references ModularRigGizmoLibrary_DMC and the string 'Direct Mesh Control'. The strings 'fk-layer' and 'ik-layer' match the DMC polygroup layer names used by RigUnit_SetupShapeLibraryFromLayer, not animation layers.
- **/Game/Sequences/zebra_marketingPoseFaces (LevelSequence, 2.4 MB)** — *face pose shots*. created 2026.04.23. Spawnable 'Zebra' with an MR_Zebra Control Rig track (1 section) plus a Transform track. The name table has space channels with Parent and ControlRig key types only (target 'Neck/End FK Global Orient Space'). It has 12+ AIESelectionSets, including stale 'Zebra_Face_CtrlRig/...' control names. Camera tracks include focal length, focus, aperture and exposure compensation. expression_demo_seq (49 KB) instead animates Eyes-slot material parameters directly: Highlight_Intensity, Cornea Roughness, Sclera Color Multiply RGBA and Highlight_Shape_Pos RGBA.
- **/Game/Assets/Environment/Rig/MR_Boombox (ControlRigRuntimeAsset, modular)** — *prop rig built from engine stock modules*. 8 modules: Root (/ControlRig/Modules/Modules58/Root) and AddControl, AddControl_1 ... AddControl_6 (/ControlRigModules/Modules58/AddControl), forming a chain handle -> boombox -> {button, button2, antenna, tape1, tape2}. Static hierarchy: 8 bones plus socket handle_socket (on bone handle) plus 15 connectors. The forward-solve graph is empty (only BeginExecution), there is no Backwards Solve, and no SupportedEventNames are saved. ShapeLibraries = [DefaultGizmoLibraryNormalized]. SourceHierarchyImport is SK_Boombox; the preview mesh is SKM_Boombox. Per-module settings live only in the editor ModularRig_0 module objects of asset.t3d; ConfigOverrides export as '()'. No runtime hierarchy was dumped because instantiation failed.
- **/ControlRigModules/Modules58/AddControl and /ControlRig/Modules/Modules58/Root (engine modules, not dumped)** — *stock engine modules used only by MR_Boombox*. Only a read-only name-table scan was possible. AddControl: connectors 'Add Control Primary' and 'Parent Control'. Construction functions: Control Stack at Item (Control Suffix '_ctrl', Null Suffix '_null'/'_ctrl_null', optional Bottom Null, optional Secondary Control, Offset Control, Orient to World) and Create Sockets (HierarchyAddSocket named '<child>_socket'). Forward: Attach Bone to Control. Also ColorizeControls (side colors by X/Y/Z Plane and Negative), Scale Module Controls (Global Scale x Local Scale), and a ModuleSettings struct. Root: Root Control Name, Global Control Name, Body Offset Control Name (producing root_ctrl, global_ctrl, body_offset_ctrl), RootModuleSettings, Generate Biped Skeleton Data, Map Skeleton Tree, Detect Scale, Get BBox of Skeleton, sockets pelvis_socket/spine_socket/spine_01_socket/biped_physics_socket, and a Backwards Solve function 'INV Root'.
- **/FortniteRigs/Meshes/Manny/SKM_Manny + SK_Manny (+ redirectors SK_Mannequin, SKM_Manny_Simple)** — *reference/preview mannequin for all FortniteRigs assets*. SKM_Manny: 106 bones, 3 LODs, 48705 LOD0 vertices, materials M_HeadLegs and M_Torso, no morph targets, no post-process ABP, no physics asset, AnimCurveMetaData as asset user data. SK_Manny has 8 foot pivot sockets on ball_l and ball_r. SK_Mannequin and SKM_Manny_Simple are ObjectRedirectors (asset_index class) that resolve to SK_Manny and SKM_Manny, which is why their dumps show SK_Manny/SKM_Manny content.
- **/FortniteRigs/Controls/ModularRigGizmoLibrary(_DMC) + ModularRigGizmoMaterial(_DMC) + ModularRigXRayMaterial + ControlRig_*_Pins/Rounded* meshes** — *control gizmo display*. Both libraries define the same 62 shape names (0-53 from /ControlRig/Controls, 54-61 custom pin and trapeze meshes) and a DefaultShape (sphere, scale 0.1). The non-DMC library points to /FortniteRigs/Controls/* and uses ModularRigGizmoMaterial (unlit, opaque). The DMC library points shapes 54-61 and its XRayMaterial to the non-existent /EpicControlRig/ mount; it uses ModularRigGizmoMaterial_DMC (unlit, translucent, with Hovered/HoveredColor parameters). ModularRigXRayMaterial is unlit, translucent and has bDisableDepthTest.
- **/Game/Assets/Zebra/Rig/RTG_UEFN_to_Zebra (IKRetargeter) + IK_Zebra (IKRigDefinition)** — *retargeting from UEFN mannequin to Zebra*. The retargeter has no SourceIKRigAsset. Its source preview mesh /Game/Characters/UEFN_Mannequin/... does not exist in the project, and the target preview mesh /Game/Assets/Zebra/Geo/Zebra_transfered3 does not exist either. Op stack: Pelvis Motion, FK Chains, Blend to Source, Body Intersect Goals (PA_Zebra), Offset Goals, Run IK Rig, Root Motion, Remap Curves, Filter Bones. In the Run IK Rig op, the ChainMapping maps RightClav->RightLeg and LeftClav->LeftLeg. IK_Zebra: a FullBodyIK solver with 4 goals and chains Spine, Neck, RightClav, ... ; its preview mesh is the same missing Zebra_transfered3.
- **/FortniteRigs/UtilityRigs/CRU_PropAim** — *Sequencer 'utility' rig for the constraint system*. A modular rig with one Prop module whose Parent and Spaces connectors point to bone 'root'. bAllowMultipleInstances=True. AssetVariant tag AnimatorKit_Utility. ModularRigSettings bAutoResolve=False. Its preview mesh /EpicControlRig/Meshes/Dummy/SKM_Dummy does not exist (the cause of 'ERR pm'). The runtime hierarchy has 17 controls, 3 nulls and 3 connectors.
- **ZebraMuzzleTwist_DeformerGraph, SK_ZebraHi** — *unused assets*. ZebraMuzzleTwist is the only one of the 8 Zebra deformer graphs that CR_Zebra_Deform does not reference (7 AddOptimusDeformer nodes). SK_ZebraHi is a copy of SK_Zebra (same 8 sockets, same VirtualBoneGuid, 370-entry BoneTree) that no mesh uses: both SKM_Zebra and SKM_Zebra_Hi point to SK_Zebra.
- **ZebraSample project (uproject, DefaultEngine.ini, Plugins/FortniteRigs)** — *project-level requirements*. The project enables DMC, AnimatorKit, RigMapperOp, RelativeIKOp, CurveExpression, SkeletalMeshMorphTargetEditingTools, PerformanceCaptureWorkflow, MovieRenderPipeline, GeometryCacheLevelSequenceBaker and others. DeformerGraph is not listed but comes in through plugin dependencies. The renderer uses unlimited GPU-skin bone influences, 16-bit bone indices, skin cache in Exclusive mode, experimental chunking and ray tracing. FortniteRigs is a content-only project plugin with no plugin dependencies declared, and there are no CoreRedirects for /EpicControlRig.

## 2. Features by domain

Importance: **core** = the rig does not work without it; **important** = visible behaviour or a key animator workflow; **nice-to-have** = secondary or unused in the shipped rigs.

### D1 Rig element model

#### UE-helper-joint-layout — Corrective helper-joint and twist-joint layout on SK_Zebra

*core* · assets: SK_Zebra, SKM_Zebra, SKM_Zebra_Hi

The Zebra skeleton has 371 bones: root, pelvis, spine_01-05, neck_01-02, head, and a 278-bone head subtree (232 lid bones plus lips, ears, tongue, teeth, mohawk). There are no IK or virtual bones.

Deformation helpers are leaf bones parented directly to the driving body bone:
- def_strap_l/r under clavicle.
- def_chest_l/r and def_back_l/r under spine_04.
- def_elbow_in/ot_l/r under upperarm, at the elbow.
- def_knee_in/ot_l/r under thigh, at the knee.
- def_thigh_in/ot_l/r under pelvis.

Twist bones come in 4 per segment, all children of the segment bone:
- upperarm_twist_01..04: 01 at the shoulder, offsets 0 to 14.1 cm.
- lowerarm_twist_04..01: 04 near the elbow at 2.6 cm, 01 near the wrist at 17.4 cm.
- thigh_twist_01..04: 3.2 to 19 cm.
- calf_twist_04..01: 4.1 to 24.8 cm.

The animator rig drives the twist bones, and CR_Zebra_Deform adds volume offsets on top. Hands have pinky, index and thumb only.

**Setup.** Naming: def_<region>_<in|ot>_<side> for helpers, <segment>_twist_0N_<side> for twist bones. Mirrored right-side bones carry negated translations (the X axis points down the chain on the left and up the chain on the right).

**Scale:** 20 def_* helpers + 32 twist bones

**Evidence:** `<dump>/Game__Assets__Zebra__Meshes__SKM_Zebra/bones.txt (def_strap_l parent=clavicle_l; def_back_l parent=spine_04; def_elbow_in_l parent=upperarm_l; def_knee_in_l parent=thigh_l; def_thigh_in_l parent=pelvis)`; `<dump>/Game__Assets__Zebra__Meshes__SKM_Zebra/bones.txt (lowerarm_twist_01_l local=T(17.37,...); calf_twist_01_l local=T(-24.81,...))`; `<dump>/Game__Assets__Zebra__Meshes__SKM_Zebra/summary.json bone_count 371`

#### UE1-module-namespacing — Module namespace naming, spawn metadata and name de-duplication

*core* · assets: MR_Zebra, MR_Monster, MR_FN_Biped, CRU_PropAim

Every element a module creates is named '<ModuleName>/<ShortName>', for example 'Leg L/IK' or 'Spine/Pelvis TXY'. It carries metadata DesiredName (NAME), DesiredKey (RIG_ELEMENT_KEY) and Module (NAME). Connectors are named '<Module>/<ConnectorName>'. When a module creates two elements with the same short name, the later ones get _2 or _3 suffixes, while display_name keeps the clean label (e.g. 'Arm L/Gimbal Control Vis_2' with display 'Gimbal Control Vis'). Connection targets are matched case-insensitively as FNames: connections to 'Control:Root/Local' resolve to the control 'root/Local' of module 'root'. Controls without an explicit display name show the full namespaced name ('Arm L/IK Base').

**Setup.** The animator sees the module prefix on every control. Display names set by module config override the visible label.

**Operators:** `URigHierarchy::DesiredNameMetadataName`, `URigHierarchy::DesiredKeyMetadataName`, `URigHierarchy::ModuleMetadataName`, `URigHierarchy::GetModuleFName`, `FRigModuleInstance::GetModulePrefix`

**Evidence:** `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Rigs/RigHierarchy.h:175-177 ('DesiredName', 'DesiredKey', 'Module')`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:863 'CONTROL Arm L/Gimbal Control Vis_2 ... display_name=Gimbal Control Vis'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:868 'display_name=Arm L/IK Base'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/modular_rig_model.txt: 'Name="Body/Parent"),Targets=((Type=Control,Name="Root/Local")' vs runtime 'CONTROL root/Local'`

#### UE1-runtime-hierarchy-census — Final runtime hierarchy composition (control/animation types per rig and module)

*core* · assets: MR_Zebra, MR_Monster, MR_FN_Biped, CRU_PropAim

Runtime (post-construction) totals:
- Zebra: 359 controls = 221 EULER_TRANSFORM + 14 ROTATOR + 4 POSITION animation controls; 69 BOOL + 32 FLOAT + 14 SCALE_FLOAT + 1 INTEGER channels; 4 PROXY_CONTROL transforms. Also 326 nulls and 423 bones (52 procedural: Arm/Leg 'Virtual Bone A/B/C', Spine '*_virtual' x6, Neck '*_virtual' x3, 31 Face bones).
- Zebra body only (face excluded): 252 controls.
- Per module, Zebra (controls/nulls/bones): root 8/0/0; Prop 17/3/0; Body 9/2/0; Spine 21/35/6; Neck 15/30/3; Clavicle 1/2/0 each; Arm 22/33/3 each; Leg 21/21/3 each; Foot 12/13/0 each; each LimbTwist 4/5/0; Thumb/Index/Pinky 3/3/0 each; Tweakers 4/4/0; Ear Base 2/2/0; Ear 4/4/0; Mohawk 4/4/0; Face 107/32/31.
- Monster: 177 controls (body 57 + face 120), 85 nulls, 202 bones.
- Biped: 283 controls (135 transform, 14 rotator, 4 position, 82 bool, 17 float, 14 scale-float, 1 int, 16 proxy), 329 nulls, 127 bones. LimbTwist has 2 controls each; Meta 8; Attach 14; finger proxies 2 each; Stretch Feedback 1; IK Bone Pins 0.
- PropAim: 17 controls.

**Operators:** `ERigControlType (EulerTransform, Rotator, Position, Bool, Float, ScaleFloat, Integer)`, `ERigControlAnimationType (AnimationControl, AnimationChannel, ProxyControl)`

**Scale:** 359, 177, 283 and 17 controls respectively.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/summary.json: 'runtime_hierarchy_counts': {'CURVE': 958, 'BONE': 423, 'NULL': 326, 'CONTROL': 359, 'CONNECTOR': 149}`; `<dump>/Game__Assets__Monster__Rig__MR_Monster/summary.json: 'CONTROL': 177`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/summary.json: 'CONTROL': 283`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:850 'BONE Arm L/Virtual Bone A parents=['BONE:clavicle_l']'`

#### UE4-unit-metadata — Engine units: item/module metadata dispatch

*core* · assets: CRFL_Control_v001, CRFL_Hierarchy_v001, CRFL_Module_v001

DISPATCH_RigDispatch_SetMetadata(Item, Name, NameSpace, Value) and GetMetadata(Item, Name, NameSpace, Default) are typed dispatches for bool, vector, item key, LinearColor, FName arrays and so on. DISPATCH_RigDispatch_GetModuleMetadata and SetModuleMetadata(Name, NameSpace, Value/Default) store values on a module rather than an item. When the value is missing, Get returns Default with Found=false. RigUnit_SetMetadataTag adds a name tag. The NameSpace enum is None (store raw on the item), Self (relative to the calling module), Parent (relative to the parent module) or Root (under the root module).

**Operators:** `DISPATCH_RigDispatch_SetMetadata`, `DISPATCH_RigDispatch_GetMetadata`, `DISPATCH_RigDispatch_GetModuleMetadata`, `DISPATCH_RigDispatch_SetModuleMetadata`, `RigUnit_SetMetadataTag`

**Scale:** SetMetadata: Hierarchy 8, Control 3. GetModuleMetadata: Hierarchy 6, Module 1. GetMetadata 1. SetMetadataTag 1.

**Evidence:** `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/Hierarchy/RigUnit_Metadata.h:189 (GetMetadata), :263 (SetMetadata), :553 (SetMetadataTag), :894-950 (GetModuleMetadata: Value=Default, Found=false when missing)`; `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/RigUnitContext.h:35-46`

#### UE8-bone-axis-convention — Bone-axis convention shared by Manny, Zebra and Monster (the Euler 'flip' is gimbal aliasing)

*core* · assets: SKM_Manny, SKM_Zebra, SKM_Monster, MR_Zebra, MR_Monster

bones.txt prints rotations as R(roll, pitch, yaw) (dump.py fmt_xf). Composing with UE's FRotationMatrix (X=(cp·cy, cp·sy, sp), ...) gives these global frames. Pelvis: X ≈ +Z world (up), Y ≈ +Y world (forward), Z = -X world, for all three meshes: Manny X=(0,-0.06,1), Zebra X=(0,-0.06,1), Monster X=(0,0.04,1); Z=(-1,0,0) in all. The critic's 'flipped' Monster pelvis R(90,87.97,90) vs Manny/Zebra R(-90,86.37,-90) is the same orientation written differently near pitch 90° (gimbal lock); the tilt differs by about 3.6°. Importers must compare matrices or quaternions, never Euler triples. The real Monster difference is its layout: 165 bones, upper body only (no arms past the clavicles, no legs), root at the origin, pelvis at z=-88.2, head at z≈39.2. Side convention (Manny, Zebra, Monster): left side is +X world (Zebra upperarm_l at (10.52,1.98,79.99)). Left-chain bones have +X child offsets and X axes pointing outward (+X). Right-chain bones use negated child translations (upperarm_r local T(-17.81…) on Manny, T(-9.126…) on Zebra) and X axes that also point +X world, i.e. toward the body. Leg chains follow the same rule with the signs inverted (calf_l T(-43.34), calf_r T(+43.34)). Mirroring and negative-side detection should use this 'negated translation' convention. The computed frames match the engine-built runtime nulls ('Arm L/FK 0 spine_05 Orient Space' T(10.52,1.978,79.99); 'Clavicle L/FK Body Orient Space' T(1.428,2.413,79.32)).

**Setup.** none

**Operators:** `FRotator -> FRotationMatrix`

**Scale:** 3 skeletons

**Evidence:** `<dump>.py:72-77 (R(roll,pitch,yaw))`; `<dump>/FortniteRigs__Meshes__Manny__SKM_Manny/bones.txt:2 (pelvis R(-90,86.37,-90)), upperarm_r local=T(-17.81,...)`; `<dump>/Game__Assets__Zebra__Meshes__SKM_Zebra/bones.txt :: 'pelvis parent=root local=T(-3.431e-17,5.345,50.87) R(-90,86.37,-90)'`; `<dump>/Game__Assets__Monster__Meshes__SKM_Monster/bones.txt:2 (pelvis local=T(1.418e-14,-14.78,-88.2) R(90,87.97,90))`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:864, 1389`

#### UE4-children-by-contained-strings — Get Children by Contained Strings (token-filtered hierarchy search)

*important* · assets: CRFL_Hierarchy_v001, CRM_FN_LimbTwist

Inputs: Item, Include Item, Recursive (library default true), Type to Search (ERigElementType), Search String and Seperator. Output: Results. children = CollectionChildrenArray(Item, bIncludeParent, bRecursive, bDefaultChildren=true, TypeToSearch). tokens = Split(SearchString, Seperator); empty tokens are kept, but a trailing empty token is not. A child is kept only if its name contains ALL tokens. StringContains is case-sensitive. Example: LimbTwist uses Item=Start Bone, Recursive=false, Type=Bone, Separator='|', and the module variable 'Twist Search String' (default 'upperarm|twist') to find twist bones.

**Setup.** Module option 'Twist Search String' (pipe-separated tokens).

**Operators:** `RigUnit_CollectionChildrenArray`, `RigVMFunction_StringSplit`, `RigVMFunction_StringContains`, `DISPATCH_RigDispatch_ToString`, `DISPATCH_RigVMDispatch_ArrayIterator`

**Scale:** LimbTwist 1 call (x4 limb-twist module instances).

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:854-916 'Get Children by Contained Strings'`; `ue/<dump>/FortniteRigs__Modules__Biped__LimbTwist__CRM_FN_LimbTwist/graphs.txt:135 'Get Children by Contained Strings | Recursive=false; Type to Search=Bone; Seperator=|'`; `ue/<dump>/FortniteRigs__Modules__Biped__LimbTwist__CRM_FN_LimbTwist/summary.json variable 'Twist Search String' DefaultValue="upperarm|twist"`; `UE_5.8/Engine/Plugins/Runtime/RigVM/Source/RigVM/Private/RigVMFunctions/RigVMFunction_String.cpp:53-56 (CaseSensitive), :118-140 Split`

#### UE4-metadata-namespace-convention — Item metadata namespaces (None vs Self vs Root) as a linking mechanism

*important* · assets: CRFL_Control_v001, CRFL_Hierarchy_v001, MR_Zebra

The libraries use item metadata as typed pointers between elements. NameSpace=Self stores the key relative to the calling module, so runtime keys are prefixed, e.g. 'Arm L/Gimbal Control:RIG_ELEMENT_KEY'. Examples: 'Gimbal Control' on the source control; 'Null', 'Bottom Null', 'Secondary' and 'Item' on stack controls; the bool 'IsSet' on proxy buffers. NameSpace=None is used for rig-global tags ('Mirror Axis', 'Mirror Behavioral'). NameSpace=Root is used to read rig-wide module metadata. RigUnit_SetMetadataTag with NameSpace=Self adds the tag 'Controls'.

**Operators:** `DISPATCH_RigDispatch_SetMetadata`, `DISPATCH_RigDispatch_GetMetadata`, `RigUnit_SetMetadataTag`

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Control_v001/graphs.txt:206 'Set Item Metadata | Name=Gimbal Control; NameSpace=Self'`; `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:187-189 'Name=Null / Bottom Null / Secondary; NameSpace=Self', :214 'Add Tag | Tag=Controls; NameSpace=Self', :420 'Name=Item; NameSpace=Self'`; `ue/<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:855 'Arm L/Gimbal Control:RIG_ELEMENT_KEY'`; `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/RigUnitContext.h:35`

#### UE4-mirror-metadata — Mirror metadata tagging (Set Mirror Axis / Set Mirror Behavior / Add Mirror Tag)

*important* · assets: CRFL_Control_v001, CRM_FN_Foot, CRM_FN_IkFk2Bones, CRM_FN_LimbTwist, CRM_FN_FkArray, CRM_FN_FkChain

Set Mirror Axis(Item, Value: FVector) writes item metadata 'Mirror Axis' (vector) with NameSpace=None. The library default is (0,1,1). Set Mirror Behavior(Item, Value: bool) writes item metadata 'Mirror Behavioral' (bool) with NameSpace=None. Add Mirror Tag(Item, Axis, Behavioral) calls both. Nothing in these libraries computes mirroring; the metadata is data for external mirror/pose tools, which read it to decide per-control mirror axes and behavioural vs orientation mirroring. Because NameSpace is None, the key has no module prefix (runtime shows 'Mirror Behavioral:BOOL').

**Setup.** The per-module variables that feed these values are Mirror Behavior (bool) and Mirror Axis (vector).

**Operators:** `DISPATCH_RigDispatch_SetMetadata (Vector / Bool, NameSpace=None)`

**Scale:** Set Mirror Behavior: 10 sites (Foot 3, IkFk2Bones 3, LimbTwist 1, FkArray 1, FkChain 1, Add Mirror Tag 1). Set Mirror Axis: 3 sites (FkArray, FkChain, Add Mirror Tag). Add Mirror Tag: unused.

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Control_v001/graphs.txt:29 'Set Vector Metadata | Name=Mirror Axis; NameSpace=None'`; `ue/<dump>/FortniteRigs__Libraries__CRFL_Control_v001/graphs.txt:40 'Set Bool Metadata | Name=Mirror Behavioral; NameSpace=None'`; `ue/<dump>/FortniteRigs__Libraries__CRFL_Control_v001/graphs.txt:12 library node default 'Value=(X=0.000000,Y=1.000000,Z=1.000000)'`; `ue/<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:855 metadata=['...','Mirror Behavioral:BOOL',...]`

#### UE8-manny-reference-mesh — SKM_Manny reference mannequin, redirectors and its foot pivot sockets

*important* · assets: SKM_Manny, SK_Manny, SK_Mannequin, SKM_Manny_Simple, all FortniteRigs modules/libraries/templates

SKM_Manny (skeleton SK_Manny) is the preview mesh of every FortniteRigs asset: the 5 CRFL libraries, all 13 modules including CRM_FN_DMC, and both templates. It has 106 bones in UE5 mannequin layout (root, pelvis, spine_01-05, neck_01-02, head, clavicle/upperarm/lowerarm/hand, metacarpal plus 3 phalanges for index/middle/ring/pinky, 3-bone thumb, upperarm_twist_01/02, lowerarm_twist_01/02, thigh/calf/foot/ball and twist bones), 3 LODs, 48705 LOD0 vertices, no morph targets, no physics asset and no post-process ABP. SK_Mannequin and SKM_Manny_Simple are ObjectRedirectors to SK_Manny and SKM_Manny, which explains their identical dump content. SK_Manny sockets on the ball bones (RelativeLocation, no rotation): foot_l_toe_tip (7.5,1.2,0), foot_l_heel (-20,1.2,0), foot_l_outer (0,1.2,-6.5), foot_l_inner (0,1.2,5); foot_r_toe_tip (-7.5,0,0), foot_r_heel (20,-1.2,0), foot_r_outer (0,0,6.5), foot_r_inner (0,0,-5). The Y offsets are not mirror-consistent: +1.2 on all left sockets, -1.2 on the right heel only. The template connection targets (metacarpals, 5 fingers, twist bones) use these bone names. CRU_PropAim is the exception: its preview mesh /EpicControlRig/Meshes/Dummy/SKM_Dummy is missing.

**Setup.** none

**Operators:** `USkeletalMesh`, `USkeleton`, `USkeletalMeshSocket`, `UObjectRedirector`

**Scale:** 20 assets use it as preview

**Evidence:** `<dump>/FortniteRigs__Meshes__Manny__SKM_Manny/summary.json :: 'bone_count': 106, 'lods': 3, 'num_verts_lod0': 48705`; `<dump>/FortniteRigs__Meshes__Manny__SK_Manny/asset.t3d :: SocketName="foot_l_heel" RelativeLocation=(X=-20.000000,Y=1.200000`; `<dump>/asset_index.json :: '/FortniteRigs/Meshes/Manny/SK_Mannequin' class 'ObjectRedirector'; '/FortniteRigs/Meshes/Manny/SKM_Manny_Simple' class 'ObjectRedirector'`; `<dump>/FortniteRigs__Meshes__Manny__SKM_Manny_Simple/summary.json :: asset_user_data '/FortniteRigs/Meshes/Manny/SKM_Manny.SKM_Manny:AnimCurveMetaData_0'`; `grep set_preview_mesh <dump>/FortniteRigs__*/regen.py -> '/FortniteRigs/Meshes/Manny/SKM_Manny.SKM_Manny' (20 assets)`

#### UE-curve-metadata — Curve metadata and curve/morph name matching

*nice-to-have* · assets: Monster_Skeleton, SK_Zebra, SKM_Zebra, SKM_Monster

Skeletons and meshes carry AnimCurveMetaData. On Monster_Skeleton it explicitly flags smile_l, brow_dn_l, stitches and brow_up_l as bMorphtarget=True. SK_Zebra's metadata object is empty in the export. Rig CURVE elements are plain name/value floats, and curve names are FNames matched case-insensitively ('squetch' in the anim equals 'Squetch' in the rig and mesh). Curves with no morph (e.g., head_side_dn_r, cornea_size, pupil_dilation, *_deformer) are consumed by rig logic or materials instead.

**Setup.** None.

**Scale:** 4 flagged curves on Monster

**Evidence:** `<dump>/Game__Assets__Monster__Meshes__Monster_Skeleton/asset.t3d CurveMetaData=(("smile_l", (Type=(bMorphtarget=True))),...)`; `<dump>/Game__Assets__Zebra__Meshes__SK_Zebra/asset.t3d AnimCurveMetaData_0 (no CurveMetaData line)`; `<dump>/Game__Assets__Zebra__Anims__Zeb_Face_Expressions/summary.json float_curves 'squetch'`

### D2 Control UX

#### UE1-animator-body-layout — Overall body control layout the animator gets (Zebra)

*core* · assets: MR_Zebra

Top-down control layout:
- root/Global > root/Local and root/Root (root motion; Bake Root On enum).
- root/Local > Body/Body Orbit > Body/Body (pelvis) + Body Aim; plus a movable-pivot proxy.
- Body/Body > Spine FK chain (Pelvis FK, Waist FK, Chest FK) with IK controls pelvis/Waist/Chest and a Chest Moveable Pivot; secondary FK Pelvis/Spine01-05; Pelvis Local.
- Neck: Neck Base FK, Neck Mid FK, Head FK, IK Neck Base/Neck Mid/head, secondary Neck 01/02/Head.
- Clavicle L/R 'Clavicle' FK.
- Arms: FK UpperArm/LowerArm/Hand (+gimbals); IK (9 spaces incl. prop); PV; Elbow; IK Base; IK Rotation; channels Ik Fk Switch/Stretch/Softness/Segment Scales/IK End Align.
- Legs: FK UpperLeg/LowerLeg/Foot; IK (4 spaces); PV; Knee; IK Base.
- Feet: Heel, Toe Tip, Ball, Ball IK, Foot Rocker, Toes FK/IK, Footprint.
- 16 twist offset controls (4 per segment).
- Fingers: Thumb/Index/Pinky Base/Mid/Tip.
- Props: Prop Global > Local > Prop > Attach 01/02, plus Aim.
- Tweakers: Def Thigh In L/R.
- Ears: Ear Base L/R > Ear 01/02.
- Mohawk: Bk and Fr.
- Face: 107 controls.
- Per-limb visibility toggles on root/Global ('Arm L Vis', 'Leg L Vis', ...).

**Setup.** Body: 252 controls (134 EulerTransform + 14 Rotator + 4 Position animation controls, 4 proxies, 96 channels). Face: 107.

**Scale:** 359 controls total.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt (all CONTROL lines; e.g. 1875, 1914, 1918, 1404, 1388, 2115, 2041, 2062, 1997, 822, 1883, 1537, 2150)`

#### UE3-ik-controls-setup — IK control, IK gimbal, IK effector null and world-orient compensation

*core* · assets: CRM_FN_IkFk2Bones

Construction:
- Spawns transform control 'IK':
  - parent = root module metadata 'Local Control' (root/Local); InitialSpace = GlobalSpace
  - InitialValue = (VB[-1].initialRot * IK Rotation Offset, VB[-1].t)
  - OffsetTransform translation = VB[-1].t; offset rotation = IK Compensate World Orient ? identity : VB[-1].rot * IK Rotation Offset (global)
  - preferred rotation order = Rotation Order; shape = IK Control Shape (or 'ik-layer.<End bone>' DMC shape); color from metadata; default gizmo Box_Thick
- Creates the gimbal 'IK Gimbal'.
- Spawns null 'IK' (local identity) under the gimbal. This is the solver effector.
- Sets module metadata (namespace Self): 'IK Null', 'IK Control', 'IK Driver' (= IK gimbal). Child modules such as the Foot read these.

With compensation on, the control's animated rotation equals its world orientation (tooltip: used for hands).

**Setup.** 'IK' is a Box_Thick EULER_TRANSFORM control parented to root/Local, with a Sphere_Thin rotator 'IK Gimbal' child. Its space list comes from the IK Spaces feature.

**Operators:** `RigUnit_HierarchyAddControlTransform`, `DISPATCH_RigDispatch_GetModuleMetadata`, `DISPATCH_RigDispatch_SetModuleMetadata`, `RigUnit_HierarchyAddNull`, `RigVMFunction_MathQuaternionMul`

**Scale:** 1 per limb

**Evidence:** `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:610 'Spawn Transform Control | Name=IK'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:666 'Get Module Item Metadata | Name=Local Control; NameSpace=Root'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:726 'If | True=(X=0...W=1)' + 1162-1176`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:633-635,909 'Spawn Null | Name=IK' / 'Name=IK Null' / 'Name=IK Control' / 'Name=IK Driver'`; `<dump>/.../CRM_FN_IkFk2Bones/regen.py 'SpawnControl_1_1_1_1_2_1.Settings' InitialSpace=GlobalSpace`

#### UE3-ikfk-switch-vis — IK/FK switch channel, host fan-out and mode visibility

*core* · assets: CRM_FN_IkFk2Bones, CRFL_Hierarchy_v001

Construction:
- Bool channel 'Ik Fk Switch' on IK, initial = Default IK (legs True, arms False).
- Hosts = FK Controls, FK Gimbals, Mid, PV, IK Gimbal, IK Base.
- Previous IK Solve = Default IK.
- Switch Control Visibility(IK FK Manipulation) runs once.

Pre Forwards Solve: IK Solve = channel; module metadata 'IK Solve' (Self) = the same value, for child modules.

Forward visibility (when IK FK Manipulation = false):
- FK controls = !IK.
- IK, PV, IK Rotation = IK.
- IK Base = IK && SecVis.
- Gimbals as described in the gimbal feature.

Dormant mode IK FK Manipulation (private, default false): while interacting, grabbing an FK control sets IK Solve = false and grabbing IK or PV sets it true. SetControlDrivenList then cross-links FK <-> [IK, PV].

**Setup.** 'Ik Fk Switch' bool channel (true = IK), visible on every IK/FK control of the limb.

**Operators:** `RigUnit_HierarchyAddAnimationChannelBool`, `RigUnit_SetChannelHosts`, `RigUnit_PreBeginExecution`, `DISPATCH_RigDispatch_SetModuleMetadata`, `RigUnit_SetControlVisibility`, `RigUnit_IsInteracting`, `RigUnit_SetControlDrivenList`, `FUNC Switch Control Visibility`

**Scale:** 1 per limb

**Evidence:** `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:655 'Name=Ik Fk Switch; MinimumValue=false'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:135-138 'Pre Forwards Solve' + COMMENT 'Storing Ik Solve into metadata in Pre Forward Solve for the Foot Module'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:1856,1415-1418 SetChannelHosts`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:295-307 interaction auto-switch`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:1089-1100 SetControlDrivenList`

#### UE4-color-from-metadata-v01 — Get Control Color From Metadata (module-side colour convention)

*core* · assets: CRFL_Hierarchy_v001, CRM_FN_Foot, CRM_FN_Spine, CRM_FN_Body, CRM_FN_IkFk2Bones, CRM_FN_LimbTwist, CRM_FN_FkChain, CRM_FN_ProxyControl, CRFL_Control_v001

Construction-time helper. Input: Color Override (LinearColor; the library node defaults to white 1,1,1,1). Output: Result (LinearColor). Steps: (1) module = GetModuleName(), the module prefix with its trailing separator removed. (2) isR = module.EndsWith(' R') and isL = module.EndsWith(' L'); both are case-sensitive string tests and the space is significant. (3) The side colour is read with GetModuleMetadata(NameSpace=Root, Default=(1,1,0,1)). If isR or isL, it reads 'Global Right Control Color' when isR, otherwise 'Global Left Control Color'. If neither, it reads 'Global Center Control Color'. (4) If Override.R==Override.G AND Override.G==Override.B (any grey, white or black), it returns the metadata colour; otherwise it returns the Override. Any non-grey authored colour therefore wins, and grey or white means 'use the rig-wide side colour'. Callers feed the Color from a module's shape-settings variable (default white) and write the Result into the colour of SpawnControl Settings.Shape.

**Setup.** The colour applies to every procedurally spawned control's shape. Modules are named with a side suffix ('Arm L', 'Leg R', 'Clavicle L'), as in MR_Zebra modular_rig_model.txt. CRM_FN_Root publishes the defaults: Center=(1,1,0) yellow, Left=(0,0,1) blue, Right=(1,0,0) red.

**Operators:** `RigUnit_GetModuleName`, `DISPATCH_RigDispatch_GetModuleMetadata (LinearColor, NameSpace=Root)`, `DISPATCH_RigDispatch_FromString`, `RigVMFunction_StringEndsWith`, `RigVMFunction_MathBoolOr`, `RigVMFunction_MathBoolAnd`, `DISPATCH_RigVMDispatch_CoreEquals`, `DISPATCH_RigVMDispatch_If`

**Scale:** 16 call sites: Foot 6, Spine 2, Body 2, IkFk2Bones 2, LimbTwist 1, FkChain 1, ProxyControl 1, CRFL_Control 1 (via Control Color Override v01).

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:961 '### GRAPH ...Get Control Color From Metadata.New Function_ContainedGraph' (nodes 966-984, links 985-1008)`; `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/regen.py:1855 set_pin_default_value('EndsWith.Ending', ' R'); :1858 ' L'`; `UE_5.8/Engine/Plugins/Runtime/RigVM/Source/RigVM/Private/RigVMFunctions/RigVMFunction_String.cpp:43-46 StringEndsWith uses ESearchCase::CaseSensitive`; `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Execution/RigUnit_RigModules.cpp:163-180 GetModuleName = module prefix minus last char`; `ue/<dump>/FortniteRigs__Modules__Biped__Foot__CRM_FN_Foot/graphs.txt:268 'FUNC Get Control Color From Metadata' -> SpawnControl_1.Settings.Shape.Color`

#### UE4-set-control-scale — Set Control Scale / Scale Control Shape v01 (uniform gizmo scaling)

*core* · assets: CRFL_Hierarchy_v001, CRFL_Control_v001, CRM_FN_Foot, CRM_FN_LimbTwist, CRM_FN_Spine, CRM_FN_Body, CRM_FN_FkArray, CRM_FN_FkChain, CRM_FN_Prop, CRM_FN_Root, CRM_FN_IkFk2Bones

Construction only. Inputs: Controls (RigElementKey[]), Global Scale, Scale and Auto Scale Factor (all double, default 1). s = GlobalScale * Scale * AutoScaleFactor. For each control: T = GetShapeTransform(control), which reads the CurrentLocal shape transform. It then calls SetShapeTransform(control, {Rotation=T.R, Translation=T.T * s, Scale=T.S * s}), which writes the InitialLocal shape transform. Both shape scale and shape offset are multiplied, so offset gizmos move proportionally. Scale Control Shape v01 in CRFL_Control is an identical duplicate with no callers.

**Setup.** Global Scale = Root metadata 'Global Control Scale'. Scale = the module's own 'Control Scale' variable. Auto Scale Factor = (NumChainItems > 1) ? GetChainLength(initial)/RefLength : 1.0. RefLength is 55 in IkFk2Bones, 17 in Foot, 20 in LimbTwist, 30 in Spine and 96 in Root. Root applies it only when length > 0.5. Body, FkArray, FkChain and Prop pass 1.0.

**Operators:** `RigUnit_GetShapeTransform`, `RigUnit_SetShapeTransform`, `RigVMFunction_MathVectorFromDouble`, `RigVMFunction_MathVectorMul`, `RigVMFunction_MathDoubleMul`, `DISPATCH_RigVMDispatch_ArrayIterator`

**Scale:** 9 modules, 1 call each.

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:596-649 'Set Control Scale'`; `ue/<dump>/FortniteRigs__Libraries__CRFL_Control_v001/graphs.txt:140 'Scale Control Shape v01'`; `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Hierarchy/RigUnit_SetControlOffset.cpp:97 (GetShapeTransform CurrentLocal) and :115 (SetShapeTransform InitialLocal)`; `ue/<dump>/FortniteRigs__Modules__IkSolves__IkFk2Bones__CRM_FN_IkFk2Bones/graphs.txt:703 'Set Control Scale' (Global Scale <- Get_Module_Metadata_1 'Global Control Scale'; Auto Scale Factor <- If_1_1)`

#### UE4-unit-control-display — Engine units: control shape, colour, visibility and shape-library query

*core* · assets: CRFL_Control_v001, CRFL_Hierarchy_v001

RigUnit_GetShapeTransform(Control) reads the CurrentLocal gizmo transform. RigUnit_SetShapeTransform(Control, Transform) writes the InitialLocal gizmo transform. RigUnit_ShapeExists(ShapeName) returns true if any of the rig's shape libraries contains that name; library-qualified names 'Lib.Shape' are resolved through the library name map (UControlRig::OnShapeExists -> UControlRigShapeLibrary::GetShapeByName). RigUnit_SetControlColor(Control, Color) sets the gizmo colour. RigUnit_SetControlVisibility(Item, bVisible) sets gizmo visibility.

**Operators:** `RigUnit_GetShapeTransform`, `RigUnit_SetShapeTransform`, `RigUnit_ShapeExists`, `RigUnit_SetControlColor`, `RigUnit_SetControlVisibility`

**Scale:** SetControlColor 3, SetControlVisibility 3, Get/SetShapeTransform 2 each, ShapeExists 2.

**Evidence:** `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Hierarchy/RigUnit_SetControlOffset.cpp:83-117`; `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/Core/RigUnit_UserData.h:98 and Private/Units/Core/RigUnit_UserData.cpp:268-275`; `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Private/ControlRig.cpp:508-517`; `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/Hierarchy/RigUnit_SetControlColor.h:48; RigUnit_SetControlVisibility.h:47`

#### UE6-corner-2d-slider — Lip corner 2-D in-viewport slider (Corner L/R)

*core* · assets: CRM_Zebra_Face, CRM_Monster_Face

Construction spawns 'Corner L Null' (no parent, global = lip_corner_l global) and control 'Corner L' under it. The control offset keeps only the bone translation, with rotation Z=+60 degrees (quat Z .5 W .866) and scale (0.075,-0.075,0.075) in Global offset space. Settings lock TranslationX (min=max=0) and leave Y/Z free, so the control is a 2-D pad in its offset YZ plane. Corner R uses rotation Z=-60 degrees. Corner L Null is a real follower: Lip Corner Constraints and New Parent constrain and move it with skull/jaw/muzzle/mouth. The control values feed Corner Logic (UE6-corner-logic).

**Setup.** Triangle_Thick; blue (L) or red (R); shape translation (+-50,25,0) (Monster (+-80,0,0)), shape rotation 90 degrees, scale 5; bDrawLimits=false. Hosts float channels Sneer Tp and Sneer Bt (-200..200); Monster also hosts Sticky (0..200). Duplicate names on the R side get _2 suffixes.

**Operators:** `RigUnit_HierarchyAddNull`, `RigUnit_HierarchyAddControlTransform`

**Scale:** 2 controls, 2 nulls

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:13 SpawnControl_4 Name=Corner L OffsetTransform Z=0.5 W=0.866 Scale3D 0.075/-0.075`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/regen.py SpawnControl_4.Settings LimitTranslationX=(bMinimum=true,bMaximum=true)`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:2205 CONTROL Face/Corner L LIM[0]`

#### UE6-lid-main-sliders — Lid Tp/Bt main sliders (per eye)

*core* · assets: CRM_Zebra_Face, CRM_Monster_Face

'Lid Tp L' is parented to bone eye_main_l with OffsetSpace Local: offset rotation 180 degrees about X, T(13,0,7), scale 0.05. It locks TranslationX, TranslationY, Pitch and Yaw at 0, leaving TranslationZ (open/close) and Roll (lid tilt) free. 'Lid Bt L' uses an identity-rotation offset at T(13,0,-5) with no limits. The R versions use offset T(-13,0,-7) (Tp, identity rotation) and T(-13,0,5) (Bt, 180 degrees about X). The Tp slider's value.tz and Roll drive the blink/extend/open/rotate functions. The Bt slider drives the same functions with the bottom-lid tables.

**Setup.** Triangle_Thick, blue (L) or red (R); shape rotation quat (.5,.5,.5,.5), scale (20,4,20); Bt shape scale (20,-4,20).

**Operators:** `RigUnit_HierarchyAddControlTransform`

**Scale:** 4 controls

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:23 SpawnControl_17 Name=Lid Tp L OffsetSpace=LocalSpace`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/regen.py SpawnControl_17.Settings LimitTranslationX..LimitYaw true, LimitRoll false`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1146 Face/Lid Tp L LIM[0, 1, 3, 4]`

#### UE1-control-limits — Transform limit flags and channel ranges

*important* · assets: MR_Zebra, MR_Monster, MR_FN_Biped

Limit-enabled flags use the order [tx, ty, tz, rx, ry, rz, sx, sy, sz]:
- IK Base: 000111111 (translate-only; min/max limit rotation and scale).
- Spine and Neck FK/IK: 000000111 (scale locked).
- All other body transform controls: none.

Channel ranges (initial, min, max, limits):
- Softness: 0 [0, 1].
- PV Twist Follow: [0, 1] (config sets 1 for legs).
- Upper/Lower Segment Scale (SCALE_FLOAT): 1 [0.0001, 2], min only.
- Spine/Neck Stretch (SCALE_FLOAT): 1 [0, 1].
- Mid Blend (SCALE_FLOAT): 0.65 [0, 1].
- Distribute Rotation (SCALE_FLOAT): [0, 1].
- Rocker Blend: 1 [0, 1].
- Rocker Ball Rotation: 45, range 0-90, no limits enabled.
- Aim Weight and Aim Twist: 0 [0, 1].
- Control Path Distance: 50, min 0 only.
- Bake Root On (INTEGER): 0 [0, 100], enum CREnum_RootMatching {Global Control, Local Control, Root Control}.
- Proxy Pivot Slide: 0.5 [0, 1].
- Limb 'Stretch': bool, initial true.

**Operators:** `RigUnit_HierarchyAddAnimationChannelFloat`, `RigUnit_HierarchyAddAnimationChannelScaleFloat`, `RigUnit_HierarchyAddAnimationChannelInteger`, `FRigControlLimitEnabled`

**Scale:** 46 limit-enabled body controls/channels in Zebra and Biped. 17 in Monster.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:868 'CONTROL Arm L/IK Base ... limit_enabled=[...{minimum: True, maximum: True} x6]'`; `<dump>/FortniteRigs__Modules__FkSolves__Root__CRM_FN_Root/graphs.txt: 'Bake Root On; InitialValue=0; MinimumValue=0; MaximumValue=100; ... ControlEnum=/FortniteRigs/Modules/FkSolves/Root/CREnum_RootMatching.CREnum_RootMatching'`; `<dump>/FortniteRigs__Modules__FkSolves__Root__CREnum_RootMatching/asset.t3d: 'Global Control' 'Local Control' 'Root Control'`; `<dump>/FortniteRigs__Modules__IkSolves__IkFk2Bones__CRM_FN_IkFk2Bones/graphs.txt: 'Name=Upper Segment Scale; InitialValue=1.000000; MinimumValue=0.000100; MaximumValue=2.000000'`

#### UE1-gizmo-libraries — Modular rig gizmo shape libraries and shape naming

*important* · assets: ModularRigGizmoLibrary, ModularRigGizmoLibrary_DMC, MR_Zebra, MR_FN_Biped, MR_Monster

ControlRigShapeLibrary with 62 named shapes, each an FControlRigShapeDefinition{ShapeName, Transform (scale 0.1), ShapeProxy static mesh}. Families (Thin 1mm, Thick 3mm, Solid): Arrow2, Arrow4, Arrow, Box, Circle, Diamond, HalfCircle, Hexagon, Octagon, Pyramid, QuarterCircle, RoundedSquare, RoundedTriangle, Sphere, Square, Star4, Triangle, Wedge. Custom Fortnite shapes: Circle_Pins_Thick, Square_Pins_Thick, Sphere_Pins_Small, Stick_Pins_Thick, Cross_Pins_Thick, RoundedTrapeze_Thick, RoundedTrapeze_ArrowTip_Thick, RoundedSquare_ArrowTip_Thick. Default shape: Sphere_solid. The gizmo material has a 'Color' parameter; the DMC library adds hover parameters.

Modules register libraries at construction (RigUnit_SetupShapeLibraryFromUserData, NameSpace 'CRSL', Path 'ShapeLibrary'). Control shape names may be library-qualified ('ModularRigGizmoLibrary.RoundedSquare_ArrowTip_Thick', 'DefaultGizmoLibraryNormalized.Sphere_Thin') or bare.

Zebra body usage: Default 134, Circle_Thick 20, Circle_Pins_Thick 18, Box_Thick 12, Hexagon_Thick 8, Triangle_Thin 8, Circle_Thin 6; root Global uses RoundedSquare_ArrowTip_Thick, Local uses RoundedSquare_Thick, Root uses Arrow_Thick; Prop uses RoundedTrapeze_ArrowTip_Thick; PV uses Diamond_Solid; Mid uses HalfCircle_Solid.

**Setup.** Shape families encode role: hexagon = body/IK, circle = FK, pins = fingers, triangle = secondary FK, diamond = PV, box = IK/attach, sphere = pivots.

**Operators:** `UControlRigShapeLibrary`, `RigUnit_SetupShapeLibraryFromUserData`, `RigUnit_SetupShapeLibraryFromLayer`, `RigUnit_ShapeExists`

**Evidence:** `<dump>/FortniteRigs__Controls__ModularRigGizmoLibrary/asset.t3d: 'Shapes(60)=(ShapeName="RoundedTrapeze_ArrowTip_Thick"'`; `<dump>/FortniteRigs__Controls__ModularRigGizmoLibrary_DMC/asset.t3d: 'MaterialHoveredParameter="Hovered"'`; `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkArray/graphs.txt:319 'RigUnit_SetupShapeLibraryFromUserData | NameSpace=CRSL; Path=ShapeLibrary'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1875 'CONTROL root/Global ... shape_name=ModularRigGizmoLibrary.RoundedSquare_ArrowTip_Thick'`

#### UE1-proxy-controls-driven-lists — Proxy controls with driven-control lists (movable pivots, footprint, finger curl/spread)

*important* · assets: MR_Zebra, MR_Monster, MR_FN_Biped

PROXY_CONTROL transform controls select or drive other controls through driven_controls (RigUnit_SetControlDrivenList):
- 'Body/Body Movable Pivot' (Sphere_Solid) drives Body/Body Orbit.
- 'Spine/End Movable Pivot' (display 'Chest Moveable Pivot') drives Spine/End IK. Each has a '... Movable Pivot Null' parent and a '... Movable Pivot Bfr' child null with metadata IsSet:BOOL.
- 'Foot L/Footprint Display' (RoundedSquare_Solid, under Leg L/IK) is a display-only proxy.

Biped template, CRM_FN_ProxyControl instances: 'Spread' or 'Curl' proxies parented to '<Proxy> Null' under a finger bone. 'Snap To' positions them at a finger control. The Driven Controls array fills driven_controls:
- Finger Spread Inner/Outer drive FK 0 of Index/Middle/Ring/Pinky.
- Finger Inner/Outer Curl drive FK 0-2 of all 4 fingers (12 controls).
- Meta Spread drives the 4 metacarpal controls.

Config: 'Driven Control Profile' curve (1->0 for Inner, 0->1 for Outer) weights the drive along the list. Each proxy has a 'Pivot Vis' channel.

**Setup.** Zebra: 4 proxies (2 movable pivots, 2 footprint displays). Monster: 2. Biped: 16.

**Operators:** `RigUnit_SetControlDrivenList`, `ERigControlAnimationType::ProxyControl`, `CRM_FN_ProxyControl`

**Scale:** Driven lists: 2 in Zebra, 14 in Biped.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:2039 'CONTROL Body/Body Movable Pivot ... animation_type=PROXY_CONTROL ... driven_controls=[{type: Control, name: "Body/Body Orbit"}]'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1945 'CONTROL Spine/End Movable Pivot ... driven_controls=[{type: Control, name: "Spine/End IK"}]'`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/runtime_hierarchy.txt:817 'CONTROL Finger Inner Curl L/Curl ... PROXY_CONTROL ... driven_controls=[{type: Control, name: "Index L/FK 0"}...'`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/asset.t3d: 'Proxy Name="Curl"' 'Driven Control Profile=(EditorCurveData=(Keys=((Value=1.000000),(Time=1.000000))))'`; `<dump>/units_used.json: RigUnit_SetControlDrivenList 4`

#### UE1-visibility-channels — Visibility-switch channels and shape-visibility modes

*important* · assets: MR_Zebra, MR_Monster, MR_FN_Biped, CRU_PropAim

Bool animation channels toggle the shape visibility of groups of controls during forward solve. Controls driven this way use shape_visibility=USER_DEFINED; ordinary controls use BASED_ON_SELECTION. Defaults come from module construction:
- root: Control Path Vis (False) on root/Global and root/Local (with Control Path Distance, float 50 in [0, 500]).
- Body: Body Orbit Vis (False), Movable Pivot Vis (default unset), Body Aim Vis (false), Body Offset Vis (true, only when an offset control exists).
- Spine/Neck: IK Vis (false), FK Vis (true), Sec FK Vis (false); Spine also Movable Pivot Vis (true).
- IkFk2Bones: Gimbal Control Vis x3 (False), Sec Controls Vis (false), plus '<Limb> Vis' bools parented to root/Global ('Leg L/Leg L Vis', 'Arm L/Arm L Vis').
- Foot: Footprint Vis (false), Foot Pivot Control Vis (False).
- Prop: Prop Global Vis (on root/Global), Prop Local Vis, Prop Control Vis, Aim Control Vis, Prop Attach 01/02 Vis (all false).
- FkArray: one 'Visibility' channel per control, with initials from config ('Controls Visibility Initials'; Attach shows only 'Attach' by default).
- Proxy modules: Pivot Vis (true).
- StretchFeedback: 'Stretch FeedBack Vis' on root/Global.

IK/PV controls of limbs in FK mode report shape_visible=False.

**Setup.** Zebra body has 64 BOOL channels (69 including face). About 100 USER_DEFINED shape-visibility controls versus 152 BASED_ON_SELECTION.

**Operators:** `RigUnit_HierarchyAddAnimationChannelBool`, `RigUnit_GetBoolAnimationChannelFromItem`, `RigUnit_HierarchySetShapeSettings`, `RigUnit_SetChannelHosts`

**Scale:** Zebra: 64 body bool channels.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:2150 "CONTROL Leg L/Leg L Vis parents=['CONTROL:root/Global']"`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:2041 'CONTROL Leg L/IK ... shape_visible=False ... shape_visibility=BASED_ON_SELECTION'`; `<dump>/FortniteRigs__Modules__FkSolves__Prop__CRM_FN_Prop/graphs.txt: 'Name=Prop Attach 01 Vis; InitialValue=false'`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt: 'Name=FK Vis; InitialValue=true'`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/asset.t3d: Attach 'Controls Visibility Initials(1)=False'`; `<dump>/units_used.json: RigUnit_HierarchyAddAnimationChannelBool 38, RigUnit_GetBoolAnimationChannelFromItem 49`

#### UE2-movable-pivot-proxy — Movable pivot proxy control (rotate a control around a relocatable pivot)

*important* · assets: CRM_FN_Body, CRM_FN_Spine, CRFL_Hierarchy_v001

Construct Movable Proxy v01(Driven, ProxyParent, SnapTo, Name, ShapeSettings, ProxyVis):
1. Spawn null '<Name> Null' under ProxyParent. Its global transform uses the rotation of Driven's initial global transform and the translation and scale of SnapTo's initial global transform.
2. Spawn control '<Name>' under that null. Its settings are the proxy settings (bIsProxy=true, ShapeVisibility=UserDefined).
3. Spawn bool channel 'Movable Pivot Vis' (initial = ProxyVis) under Driven.
4. SetControlDrivenList(proxy, [Driven]).
5. Spawn null '<Name> Bfr' under the proxy at the global identity transform, with bool metadata 'IsSet' = false.

Forward Movable Proxy v01, each frame:
- Proxy visibility = the channel value. The rest runs only while that value is true.
- (a) While the proxy is being translated: Bfr global = Driven global; IsSet = true.
- (b) While the proxy is being rotated: if IsSet is not yet true, Bfr = Driven global and IsSet = true. Then Driven global = Bfr global, so Driven orbits the pivot together with the proxy's rotation.
- When there is no rotate interaction: IsSet = false; the pivot null global = (T and S of SnapTo current, R of Driven current); proxy local rotation = identity.

The behavior depends on editor interaction state (IsInteracting).

**Setup.** Two instances.
- Body: 'Body Movable Pivot' (Sphere_Solid, parent = Parent connector i.e. root/Local, SnapTo = pelvis bone, Driven = Body Orbit); its 'Movable Pivot Vis' channel is hosted on Body.
- Spine: 'End Movable Pivot' (DefaultGizmoLibraryNormalized.Default, color (1,0.05,0), scale 0.4, parent = Start FK, SnapTo = last spine bone, Driven = End IK, display name from 'End Moveable Pivot Display Name'; runtime display name 'Chest Moveable Pivot'). It is created only when Is Neck is false.

**Operators:** `RigUnit_HierarchyAddNull`, `RigUnit_HierarchyAddControlTransform`, `RigUnit_HierarchyAddAnimationChannelBool`, `RigUnit_SetControlDrivenList`, `DISPATCH_RigDispatch_SetMetadata`, `DISPATCH_RigDispatch_GetMetadata`, `RigUnit_IsInteracting`, `RigUnit_SetControlVisibility`, `RigUnit_SetTransform`, `RigUnit_SetRotation`

**Scale:** 2 per biped rig

**Evidence:** `<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:1010-1083 'Construct Movable Proxy v01'`; `<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:1084-1197 'Forward Movable Proxy v01'`; `<dump>/FortniteRigs__Modules__FkSolves__Body__CRM_FN_Body/graphs.txt:237 'Proxy Name=Body Movable Pivot'`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:383 'Construct Movable Proxy v01'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt 'Body/Body Movable Pivot ... animation_type=PROXY_CONTROL ... driven_controls=[{type: Control, name: "Body/Body Orbit"}]'`

#### UE2-set-control-scale — Global/module/auto control-shape scaling

*important* · assets: CRM_FN_Root, CRM_FN_Body, CRM_FN_Spine, CRM_FN_FkChain, CRM_FN_FkArray, CRM_FN_Prop

Library 'Set Control Scale' (CRFL_Hierarchy). For each control, with k = GlobalScale * Scale * AutoScaleFactor:
- shape Scale3D *= k
- shape Translation *= k
- rotation is kept

Inputs per module:
- Root: GlobalScale = its own 'Global Control Scale' variable; Scale = Control Scale; Auto = ChainLength_initial([RootJoint bone, Body bone]) / 96. It runs only when length > 0.5.
- Spine: Auto = ChainLength_initial([Start IK, End IK]) / 30.
- Body, FkChain, FkArray, Prop: Auto = 1; GlobalScale = Root metadata 'Global Control Scale' (default 1).
- FkChain and FkArray skip scaling when a CRSL user-data shape library was found.

Get Chain Length = sum of distances between consecutive items' global translations (initial pose when Initial=true).

**Setup.** Public 'Control Scale' (double, default 1) on every module, plus 'Global Control Scale' on Root.

**Operators:** `RigUnit_GetShapeTransform`, `RigUnit_SetShapeTransform`, `RigVMFunction_MathVectorMul`, `RigVMFunction_MathVectorDistance`, `DISPATCH_RigDispatch_GetModuleMetadata`

**Scale:** 9 calls across 6 modules

**Evidence:** `<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:596-650 'Set Control Scale'`; `<dump>/FortniteRigs__Libraries__CRFL_Math_v001/graphs.txt:16-82 'Get Chain Length'`; `<dump>/FortniteRigs__Modules__FkSolves__Root__CRM_FN_Root/graphs.txt:98 'B=96.000000' and :133 'Greater | B=0.500000'`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:310 'B=30.000000'`; `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkChain/graphs.txt (construction) 'RigVMFunction_ControlFlowBranch_2.False -> Set Control Scale.ExecuteContext'`

#### UE2-side-colors — Side-based control colors from Root metadata

*important* · assets: CRM_FN_Body, CRM_FN_FkChain, CRM_FN_FkArray, CRM_FN_Spine, CRM_FN_Prop, CRFL_Hierarchy_v001, CRFL_Control_v001

Get Control Color From Metadata(Override): if Override.R == G == B (white or grey), pick a Root module metadata color:
- module name ends with ' R': 'Global Right Control Color'
- ends with ' L': 'Global Left Control Color'
- otherwise: 'Global Center Control Color' (default yellow)

Otherwise Override is returned unchanged.

v02 adds a bone test: right if (bone name is valid AND HasSide(bone, 'r')) OR the module name ends with ' R'; the same for left. HasSide(name, s) = name starts with 's_' OR contains '_s_' OR ends with '_s'.

The 'Control Color Override From Metadata' v01/v02 wrappers apply the grey test to the per-control override and pass the module color on.

Other uses:
- Spine: 'Color' = GetControlColor(Color); 'Alt Color' = Color * (0.5,0.2,0.2).
- Body Offset: color = Body color * 0.5.

**Setup.** Default shape colors are white (1,1,1) in the shape variables, so they resolve to the side colors: blue on the left, red on the right, yellow in the center.

**Operators:** `RigUnit_GetModuleName`, `RigVMFunction_StringEndsWith`, `RigVMFunction_StartsWith`, `RigVMFunction_Contains`, `RigVMFunction_EndsWith`, `DISPATCH_RigDispatch_GetModuleMetadata`, `DISPATCH_RigVMDispatch_If`, `RigVMFunction_MathColorMul`

**Scale:** 16 + 3 + 10 calls

**Evidence:** `<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:961-1009`; `<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:1565-1638 (v02)`; `<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:1639-1702 'Has Side'`; `<dump>/FortniteRigs__Libraries__CRFL_Control_v001/graphs.txt:116,404 'Control Color Override From Metadata v01/v02'`

#### UE3-control-scale-color-shape — Shared control scale, side color and DMC shape lookup

*important* · assets: CRM_FN_IkFk2Bones, CRM_FN_Foot, CRM_FN_LimbTwist, CRFL_Hierarchy_v001, CRFL_Control_v001

Set Control Scale(Controls, Global Scale, Scale, Auto): for each control, shape scale *= G*S*A and shape translation *= G*S*A.
- G = root 'Global Control Scale'; S = module 'Control Scale'.
- A = ChainLength/k when the chain has >1 item, else 1. Get Chain Length sums consecutive initial distances. k = 55 for IkFk2Bones over Bones, 17 for Foot over [foot, ball], 20 for LimbTwist over [Start, End].

Get Control Color From Metadata(Override):
- if Override.R == G == B (white/grey), use root metadata 'Global Right Control Color' if the module name ends in ' R', 'Global Left Control Color' if it ends in ' L', else 'Global Center Control Color' (fallback yellow);
- otherwise use Override.

Get Control Shape Name From Item v02(Item, Namespace 'ik-layer'/'fk-layer', Default, DMC libs):
- if Namespace is in root 'Direct Mesh Control Libraries' and shape '<ns>.<resolved item name>' exists, use it;
- else use Default.

Set Mirror Behavior writes bool item metadata 'Mirror Behavioral'.

**Setup.** Blue left, red right, from root metadata or the module Color override.

**Operators:** `RigUnit_GetShapeTransform`, `RigUnit_SetShapeTransform`, `RigUnit_ShapeExists`, `RigUnit_GetModuleName`, `RigVMFunction_StringEndsWith`, `DISPATCH_RigDispatch_SetMetadata`

**Scale:** every IK-family module

**Evidence:** `<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:596-634 'Set Control Scale'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:711 'Divide | B=55.000000'`; `<dump>/.../CRM_FN_Foot/graphs.txt:258 'Divide | B=17.000000'`; `<dump>/.../CRM_FN_LimbTwist/graphs.txt:160 'Divide | B=20.000000'`; `<dump>/FortniteRigs__Libraries__CRFL_Control_v001/graphs.txt:430-473 'Get Control Shape Name From Item v02'`; `<dump>/FortniteRigs__Libraries__CRFL_Control_v001/graphs.txt:35-43 'Name=Mirror Behavioral'`

#### UE3-proxy-control-module — Delta proxy control (ProxyControl module) with per-driven weight profile

*important* · assets: CRM_FN_ProxyControl

Construction:
- Driven controls = resolve 'Driven Controls'; Parent = resolve 'Parent'; Snap = 'Snap To' if connected, else Parent.
- Null '<Proxy Name> Null' under Parent at Snap's initial global.
- Control '<Proxy Name>' under it:
  - OffsetTransform = Proxy Offset Transform (local);
  - Settings.Proxy = (bIsProxy = true, DrivenControls = driven list, ShapeVisibility = UserDefined), so the engine treats it as an indirect control whose selection or keying targets the driven list;
  - shape = Proxy Control Shape Settings (default gizmo, orange, scale 0.1), color from metadata.
- Bool 'Pivot Vis' (initial true) on it, which nothing reads.

Forward, when interacting AND the proxy is among the interacted items:
1. L = proxy local transform.
2. D = L relative to PreviousL (L * Prev^-1).
3. For each driven control k with iterator ratio r_k in [0, 1]: OffsetTransformForItem(driven, TransformLerp(identity, D, curve(r_k))), i.e. global = offset * current.
4. PreviousL = L.

Otherwise:
- SetControlOffset(proxy, global = ProxyOffset * Snap current);
- proxy local = identity; PreviousL = identity.

The proxy thus re-centres on its snap target when released.

Biped usage: 'Curl' (Sphere_Solid) drives 12 finger FK controls. 'Spread' (HalfCircle_Solid) drives 4 FK 0 or metacarpal controls. The Driven Control Profile is 1 to 0 for 'Inner' and 0 to 1 for 'Outer'.

**Setup.** PROXY_CONTROL shapes 'Curl' and 'Spread' on the hand, snapped to a finger FK control or metacarpal.

**Operators:** `RigUnit_HierarchyAddControlTransform (bIsProxy)`, `RigUnit_IsInteracting`, `RigVMFunction_MathTransformMakeRelative`, `RigVMFunction_MathTransformLerp`, `RigVMFunction_AnimEvalRichCurve`, `RigUnit_OffsetTransformForItem`, `RigUnit_SetControlOffset`, `RigVMFunction_MathTransformMakeAbsolute`

**Scale:** 12 in MR_FN_Biped / BipedDMC; 0 in Zebra

**Evidence:** `<dump>/FortniteRigs__Modules__Miscellaneous__CRM_FN_ProxyControl/graphs.txt:2-88 RigVMModel (COMMENT 'Get Proxy Transform Delta', 'Reset Proxy Control')`; `<dump>/.../CRM_FN_ProxyControl/graphs.txt:90-157 Construction ('Name=Pivot Vis', 'Settings.Proxy.DrivenControls')`; `<dump>/.../CRM_FN_ProxyControl/regen.py:234 'Proxy=(bIsProxy=true,DrivenControls=...,ShapeVisibility=UserDefined)'`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/asset.t3d 'Finger Inner Curl L | Proxy Name="Curl"' / 'Driven Control Profile=(EditorCurveData=(Keys=((Value=1.000000),(Time=1.000000))))'`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/modular_rig_model.txt 'Finger Inner Curl L/Driven Controls'`

#### UE3-sec-controls-vis — Secondary-controls visibility channel shared across modules via metadata

*important* · assets: CRM_FN_IkFk2Bones, CRM_FN_LimbTwist

Construction spawns bool 'Sec Controls Vis' (initial false) on IK, hosted on FK Controls[-1] (the hand/foot FK).

Forward:
- Mid control visible = value.
- IK Base visible = IK && value.
- Module metadata 'Sec Controls Visibilty' (sic, namespace Self) = value.

LimbTwist reads 'Sec Controls Visibilty' from its Parent module (default true) and sets its twist controls' visibility to it.

**Setup.** 'Sec Controls Vis' on the IK control, also shown on the end FK control.

**Operators:** `RigUnit_GetBoolAnimationChannelFromItem`, `DISPATCH_RigDispatch_SetModuleMetadata`, `DISPATCH_RigDispatch_GetModuleMetadata`, `RigUnit_SetControlVisibility`

**Scale:** 1 per limb, consumed by 2 twist modules each

**Evidence:** `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:772 'Name=Sec Controls Vis; InitialValue=false'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:122 'Set Module Bool Metadata | Name=Sec Controls Visibilty; NameSpace=Self'`; `<dump>/FortniteRigs__Modules__Biped__LimbTwist__CRM_FN_LimbTwist/graphs.txt:57 'Name=Sec Controls Visibilty; NameSpace=Parent; Default=true'`

#### UE4-color-from-metadata-v02 — Get Control Color From Metadata v02 (bone-name side detection)

*important* · assets: CRFL_Hierarchy_v001, CRM_FN_Foot, CRFL_Control_v001

Same contract as v01 with an extra Bone (RigElementKey) input. boneName = ItemToName(Bone) and valid = IsNameValid(boneName). useRight = (valid AND HasSide(boneName,'r')) OR module.EndsWith(' R'). useLeft = (valid AND HasSide(boneName,'l')) OR module.EndsWith(' L'). If useRight or useLeft, the colour is Root metadata 'Global Right Control Color' when useRight, else 'Global Left Control Color'. Otherwise it is 'Global Center Control Color'. The default is (1,1,0,1). The grey-override rule is the same as v01 (R==G==B means use metadata). Right wins when both sides match.

**Setup.** Lets controls in a centre module still be coloured by side when their driving bone is named with an _l/_r token.

**Operators:** `RigUnit_ItemToName`, `RigVMFunction_IsNameValid`, `FUNC Has Side (CRFL_Hierarchy)`, `RigUnit_GetModuleName`, `DISPATCH_RigDispatch_GetModuleMetadata`, `RigVMFunction_StringEndsWith`, `DISPATCH_RigVMDispatch_If`

**Scale:** 3 call sites: Foot 2, CRFL_Control Control Color Override v02 1.

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:1565 'Get Control Color From Metadata v02' (Has Side nodes at 1597 'Side to Check=r' and 1601 'Side to Check=l')`; `ue/<dump>/FortniteRigs__Modules__Biped__Foot__CRM_FN_Foot/graphs.txt:545 'Get Control Color From Metadata v02' with Bone <- VariableNode_21`

#### UE4-color-override-from-metadata — Control Color Override From Metadata v01/v02 (and module-local unversioned copies)

*important* · assets: CRFL_Control_v001, CRM_FN_Prop, CRM_FN_FkArray, CRM_FN_Spine, CRM_FN_IkFk2Bones

Inputs: Color Override and Module Color (both LinearColor; v02 adds Bone). Output: Result. If Override.R==G==B, it returns GetControlColorFromMetadata(ColorOverride = Module Color); v02 calls the v02 function and forwards Bone. Otherwise it returns Override. Callers pass Module Color = (0,0,0,0), which is grey, so the metadata side colour is used. The net effect is a two-level fallback: authored colour, then module colour, then Root side colour. CRM_FN_Spine and CRM_FN_IkFk2Bones each carry an unversioned local copy that reads a 'Color' variable instead of a Module Color pin.

**Operators:** `DISPATCH_RigVMDispatch_CoreEquals`, `RigVMFunction_MathBoolAnd`, `DISPATCH_RigVMDispatch_If`, `FUNC Get Control Color From Metadata (v01/v02)`

**Scale:** v01: Prop 1. v02: FkArray 1. Local unversioned copies: Spine 5 and IkFk2Bones 5 call sites.

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Control_v001/graphs.txt:116 'Control Color Override From Metadata v01'`; `ue/<dump>/FortniteRigs__Libraries__CRFL_Control_v001/graphs.txt:404 'Control Color Override From Metadata v02'`; `ue/<dump>/FortniteRigs__Modules__FkSolves__Prop__CRM_FN_Prop/graphs.txt:694 'Control Color Override From Metadata v01_1 | Module Color=(R=0...A=0)'`; `ue/<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkArray/graphs.txt:719 'Control Color Override From Metadata v02'`; `ue/<dump>/FortniteRigs__Modules__IkSolves__IkFk2Bones__CRM_FN_IkFk2Bones/graphs.txt:2820 local 'Control Color Override From Metadata'`

#### UE4-movable-proxy-construct — Construct Movable Proxy v01 (movable pivot control)

*important* · assets: CRFL_Hierarchy_v001, CRM_FN_Spine, CRM_FN_Body, MR_FN_Biped

Construction. Inputs: Driven Control, Proxy Parent Control, Proxy Snap To, Proxy Name, Proxy Shape Settings (FRigUnit_HierarchyAddControlTransform_Settings) and Proxy Vis (bool). Outputs: Proxy Control, Proxy Buffer and Proxy Vis Channel. Steps: (1) Spawn Null '<Name> Null' under Proxy Parent Control, with global rotation from the Driven Control's initial global and translation/scale from Proxy Snap To's initial global. (2) Spawn a transform control '<Name>' under that null with identity offset and value; its settings come from the input (callers pass bIsProxy=True, DrivenControls=[...]). (3) Spawn the bool channel 'Movable Pivot Vis' under the Driven Control, with initial = Proxy Vis. (4) SetControlDrivenList(proxy, [Driven Control]). (5) Spawn Null '<Name> Bfr' under the proxy control (identity global; it is repositioned at runtime). (6) SetMetadata(Bfr, 'IsSet', Self) = false.

**Setup.** This gives the animator a proxy 'pivot' control (Spine: 'End Movable Pivot', shape DefaultGizmoLibraryNormalized.Default at 0.4 scale; Body: 'Body Movable Pivot', Sphere_Solid in purple). It is shown when the 'Movable Pivot Vis' bool on the driven control is on, and selecting it also affects the driven control through the driven list.

**Operators:** `RigUnit_HierarchyAddNull`, `RigUnit_HierarchyAddControlTransform`, `RigUnit_HierarchyAddAnimationChannelBool`, `RigUnit_SetControlDrivenList`, `DISPATCH_RigDispatch_SetMetadata`, `RigUnit_GetTransform (initial)`, `RigVMFunction_NameConcat`

**Scale:** 2 modules (Spine End IK, Body).

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:1010-1082 'Construct Movable Proxy v01' (Concat 'B= Bfr', Concat_1 'B= Null', 'Name=IsSet ... Value=False')`; `ue/<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:383 'Proxy Name=End Movable Pivot'; ue/<dump>/FortniteRigs__Modules__FkSolves__Body__CRM_FN_Body/graphs.txt:237 'Proxy Name=Body Movable Pivot'`; `ue/<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/regen.py 'Construct Movable Proxy v01.Proxy Shape Settings' contains 'Proxy=(bIsProxy=True,DrivenControls=...)'`; `ue/<dump>/FortniteRigs__Templates__MR_FN_Biped/runtime_hierarchy.txt:1522-1524 'Spine/End Movable Pivot Null' -> 'Spine/End Movable Pivot' -> 'Spine/End Movable Pivot Bfr'; :1584-1586 Body equivalents (Null parent root/Local)`

#### UE4-switch-control-visibility — Switch Control Visibility (IK/FK visibility swap)

*important* · assets: CRFL_Hierarchy_v001, CRM_FN_IkFk2Bones

Inputs: On Controls, Off Controls and Switch (bool). For each On control it calls SetControlVisibility(Switch); for each Off control it calls SetControlVisibility(!Switch). IkFk2Bones calls it once in construction, with Switch from a module variable and two On controls, to set the initial IK/FK control visibility.

**Operators:** `RigUnit_SetControlVisibility`, `RigVMFunction_MathBoolNot`, `DISPATCH_RigVMDispatch_ArrayIterator`

**Scale:** IkFk2Bones 1 call.

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:918-948 'Switch Control Visibility'`; `ue/<dump>/FortniteRigs__Modules__IkSolves__IkFk2Bones__CRM_FN_IkFk2Bones/graphs.txt:689 'Switch Control Visibility | On Controls=((...),(...))'`

#### UE6-visibility-switches — Bool channels toggling micro-control visibility

*important* · assets: CRM_Zebra_Face, CRM_Monster_Face

Before the sequence, each switch loops SetControlVisibility(item, bVisible = GetBoolAnimationChannelFromItem(channel)): 'Micro Vis' on Eye Main L controls the 14 left lid micro controls (Lid Bt/Tp 01-03 L, Lid Bt/Tp Base 01-03 L, Lid In L, Lid Ot L); 'Micro Vis_2' on Eye Main R controls the right-side equivalents; 'Lip Tweaker Vis' on jaw controls the 20 Lip Controls; 'Brow Tweaker Vis' on Brow Main L controls Brow In/Mid/Ot L; 'Brow Tweaker Vis_2' on Brow Main R controls Brow Ot/In/Mid R. All channels default to false, so micro controls start hidden.

**Setup.** Five bool channels: 2x Micro Vis, 2x Brow Tweaker Vis, 1x Lip Tweaker Vis.

**Operators:** `RigUnit_HierarchyAddAnimationChannelBool`, `RigUnit_GetBoolAnimationChannelFromItem`, `RigUnit_SetControlVisibility`, `RigVMDispatch_ArrayIterator`

**Scale:** 5 SetControlVisibility loops, about 42 controls

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:681 SetControlVisibility, :683 ItemArray_3`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:770 GetAnimationChannelFromItem_13 (Lip Tweaker Vis)`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:44 SpawnAnimationChannel Name=Micro Vis`

#### UE8-asset-shape-library-resolution — Shape library lists and gizmo-name resolution rules

*important* · assets: MR_Zebra, MR_ZebraDMC, MR_Monster, MR_FN_Biped, MR_FN_BipedDMC, MR_Boombox, CRU_PropAim, CRM_FN_DMC, RunDMC, CR_Monster_Deform

Saved ShapeLibraries per rig: MR_Zebra [ModularRigGizmoLibrary]; MR_ZebraDMC [ModularRigGizmoLibrary_DMC, ModularRigGizmoLibrary]; MR_FN_Biped [ModularRigGizmoLibrary]; MR_FN_BipedDMC [ModularRigGizmoLibrary, ModularRigGizmoLibrary_DMC]; MR_Monster [ModularRigGizmoLibrary_DMC] only; CRU_PropAim [Default(implicit), ModularRigGizmoLibrary]; MR_Boombox, RunDMC and CR_Monster_Deform [DefaultGizmoLibraryNormalized]; CRM_FN_DMC [DefaultGizmoLibraryNormalized, ModularRigGizmoLibrary_DMC]. When there is more than one library, shape names are namespaced as '<Library>.<Shape>'. GetShapeByName(name) splits at '.' and runs two passes. Pass 0 walks the libraries last to first (later libraries win) and only checks the library whose name equals the namespace. Pass 1 drops the namespace. The default-shape fallback is used only in pass 1 for library index 0. Libraries added at runtime by RigUnit_SetupShapeLibraryFromUserData (NameSpace CRSL; used by FkArray, FkChain and Prop) or RigUnit_SetupShapeLibraryFromLayer (DMC) are appended and so take precedence. Consequence: MR_Monster's controls reference 'ModularRigGizmoLibrary.RoundedSquare_Thick' and 'DefaultGizmoLibraryNormalized.Default'. No library by those names exists, so they resolve in pass 1 to same-named shapes in the DMC library. MR_Zebra shape usage: 188 'Default', 40 Sphere_Solid, 25 Circle_Thick, 18 Circle_Pins_Thick, 13 Box_Thick, ...

**Setup.** The gizmo shape name is set per control in Settings.Shape.Name; the library order is set per asset.

**Operators:** `UControlRigShapeLibrary::GetShapeByName`, `UControlRigShapeLibrary::GetShapeName`, `UControlRig::OnAddShapeLibrary`, `RigUnit_SetupShapeLibraryFromUserData`, `RigUnit_SetupShapeLibraryFromLayer`

**Scale:** 10 assets; 359 MR_Zebra controls

**Evidence:** `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/ControlRigGizmoLibrary.cpp:111-203`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/ControlRig.cpp:400-412, 416-455, 486`; `<dump>/Game__Assets__Monster__Rig__MR_Monster/asset.t3d:473`; `<dump>/Game__Assets__Zebra__Rig__MR_ZebraDMC/asset.t3d:2129-2130`; `<dump>/FortniteRigs__Templates__MR_FN_BipedDMC/asset.t3d:2765-2766`; `<dump>/FortniteRigs__UtilityRigs__CRU_PropAim/asset.t3d:163`; `<dump>/Game__Assets__Monster__Rig__MR_Monster/runtime_hierarchy.txt :: 'shape_name=ModularRigGizmoLibrary.RoundedSquare_Thick'`; `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkArray/graphs.txt :: 'SetupShapeLibraryFromUserData | ... NameSpace=CRSL; Path=ShapeLibrary'`

#### UE8-gizmo-epiccontrolrig-dangling — Dangling /EpicControlRig/ references (DMC pin shapes, X-ray material, PropAim preview)

*important* · assets: ModularRigGizmoLibrary_DMC, MR_FN_BipedDMC, MR_Monster, MR_ZebraDMC, CRU_PropAim

ModularRigGizmoLibrary_DMC shapes 54-61 (Circle_Pins_Thick, Square_Pins_Thick, Sphere_Pins_Small, Stick_Pins_Thick, Cross_Pins_Thick, RoundedTrapeze_Thick, RoundedTrapeze_ArrowTip_Thick, RoundedSquare_ArrowTip_Thick) and its XRayMaterial point to /EpicControlRig/Controls/... The package binary confirms these paths. The mount does not exist: the plugin is FortniteRigs, the engine has no EpicControlRig plugin, and DefaultEngine.ini has no CoreRedirects. Those shapes therefore have no mesh and draw nothing, and X-ray falls back to DefaultMaterial. The resolution order decides who is affected. MR_FN_BipedDMC [ModularRig, DMC]: un-namespaced 'Circle_Pins_Thick' (30 controls) and RoundedTrapeze_ArrowTip_Thick (3) resolve to the broken DMC entries. MR_Monster [DMC only]: RoundedTrapeze_ArrowTip_Thick (3+1) and RoundedSquare_ArrowTip_Thick (1) are broken. MR_ZebraDMC [DMC, ModularRig]: the later non-DMC library wins, so it is fine. MR_Zebra is fine. This holds unless a module-added CRSL library supplies the shape. CRU_PropAim's PreviewSkeletalMesh is /EpicControlRig/Meshes/Dummy/SKM_Dummy, also missing (source of 'ERR pm ... NoneType'). A USD port should map /EpicControlRig/Controls/* to /FortniteRigs/Controls/*.

**Setup.** Affected controls look invisible in the viewport but stay selectable through the outliner.

**Operators:** `TSoftObjectPtr<UStaticMesh> ShapeProxy`

**Scale:** 9 dangling references; about 38 controls affected across 2 rigs

**Evidence:** `<dump>/FortniteRigs__Controls__ModularRigGizmoLibrary_DMC/asset.t3d:4, 62-69`; `<ZebraSample>/Plugins/FortniteRigs/Content/Controls/ModularRigGizmoLibrary_DMC.uasset name table: '/EpicControlRig/Controls/ControlRig_Circle_Pins_6mm', '/EpicControlRig/Controls/ModularRigXRayMaterial'`; `<dump>/FortniteRigs__Controls__ModularRigGizmoLibrary/asset.t3d:60-67 (/FortniteRigs/Controls/... correct)`; `<dump>/FortniteRigs__UtilityRigs__CRU_PropAim/asset.t3d:168`; `<dump>/_log.txt:138 (ERR pm AttributeError NoneType)`; `<ZebraSample>/Config/DefaultEngine.ini (no CoreRedirects section)`; `<dump>/FortniteRigs__Templates__MR_FN_BipedDMC/runtime_hierarchy.txt :: 'shape_name=Circle_Pins_Thick' (30)`

#### UE1-control-colors — Side colors and per-module color configuration

*nice-to-have* · assets: MR_Zebra, MR_FN_Biped, MR_Monster, CRU_PropAim

The Root module defines Global Left Control Color blue (0, 0, 1, 1), Right red (1, 0, 0, 1) and Center yellow (1, 1, 0, 1), and publishes them as module metadata for other modules to colorize by side. Per-module overrides:
- Spine/Neck: IK shapes orange (1, 0.51, 0.15); Sec FK pink (0.98, 0.65, 1); Neck Color yellow.
- LimbTwist: L red (1, 0.26, 0.26); R blue (0.31, 0.19, 1).
- Thumb L: blue (0.21, 0.27, 1). Thumb R: red (1, 0.18, 0.22).
- Proxy shapes: white.
- StretchFeedback: Rest green, Squash blue, Stretch red, Max Stretch Factor 5 (template config 2).

Prop controls carry 'Prop/Color' LINEAR_COLOR metadata. The runtime dump does not include the final control colors.

**Operators:** `FRigControlSettings.ShapeColor`, `DISPATCH_RigDispatch_SetModuleMetadata`

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Root__CRM_FN_Root/summary.json: 'Global Left Control Color:FLinearColor=(R=0.000000,G=0.000000,B=1.000000'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/asset.t3d: Leg Upper Twist L 'Shape Settings=(Color=(R=1.000000,G=0.262251,B=0.262251'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1883 "'Prop/Color:LINEAR_COLOR'"`

#### UE1-rotation-orders — Preferred rotation orders per control family

*nice-to-have* · assets: MR_Zebra, MR_FN_Biped, MR_Monster, CRU_PropAim

Preferred Euler rotation order per control:
- XYZ: limb FK, IK and PV controls (from the bound rig variable or config XYZ); spine/neck FK and IK; finger chains; FkArray controls.
- XZY: clavicles (config).
- YXZ: Prop controls (class default) and Foot Rocker.
- ZYX: foot pivot controls Ball, Heel and Toe Tip.
- YZX: channels, root/Body/twist/foot-toe controls, and Prop channels.

Zebra census: XYZ 82, YZX 154, YXZ 8, ZYX 6, XZY 2.

**Operators:** `FRigControlSettings.PreferredRotationOrder`, `EEulerRotationOrder`

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1388 'CONTROL Clavicle L/FK ... preferred_rotation_order=XZY'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1883 'CONTROL Prop/Prop ... preferred_rotation_order=YXZ'`; `<dump>/FortniteRigs__Modules__FkSolves__Prop__CRM_FN_Prop/summary.json: 'Rotation Order:EEulerRotationOrder=YXZ'`

#### UE2-display-names — Per-index display names from config arrays

*nice-to-have* · assets: CRM_FN_FkChain, CRM_FN_FkArray, CRM_FN_Spine

Settings.DisplayName of a spawned control = Array[i] when 0 <= i < Num(Array), otherwise None (Spine: 'Get Optional Display Name').

Sources per module:
- FkChain: 'Display Names' (FString array).
- FkArray: 'Display Names Sorted', i.e. the config array with entries of removed (missing) bones filtered out.
- Spine: 'FK Primary Display Names' for the Start/Mid/End FK; 'FK Secondary Display Names' for the Sec FKs; the single names 'IK Start/Mid/End Display Name' and 'End Moveable Pivot Display Name'.

**Setup.** Category 'Display Name Options'. Display names appear in the Anim Outliner (runtime examples: 'Pelvis FK', 'Waist FK', 'Chest FK', 'Neck Base FK', 'Head FK', 'Chest Moveable Pivot').

**Operators:** `DISPATCH_RigVMDispatch_ArrayGetNum`, `DISPATCH_RigVMDispatch_ArrayGetAtIndex`, `DISPATCH_RigVMDispatch_If`, `DISPATCH_RigDispatch_FromString`

**Scale:** 3 modules

**Evidence:** `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/summary.json 'FK Primary Display Names' 'IK Start Display Name'`; `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkChain/graphs.txt 'If.Result -> SpawnControl.Settings.DisplayName' (construction)`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt 'Spine/Start FK ... display_name=Pelvis FK' 'Spine/End IK ... display_name=Chest'`

#### UE2-dmc-shape-library-switch — Shape library from user data and DMC per-bone shape names

*nice-to-have* · assets: CRM_FN_FkChain, CRM_FN_FkArray, CRM_FN_Prop, CRM_FN_Spine, CRFL_Control_v001

Construction:
1. RigUnit_SetupShapeLibraryFromUserData(NameSpace='CRSL', Path='ShapeLibrary', LibraryName = the 'CRSL Namespace' variable; Prop uses 'CRSL') registers a shape library from user data. The 'bFound' output is stored in 'CRSL Found'.
2. If Root metadata 'Direct Mesh Control' is true, CRSL Namespace = 'fk-layer' (FkChain/FkArray; Spine passes the constant 'fk-layer').

Get Control Shape Name From Item v02(Item, ns, Default, DMCLibraries):
- candidate = (ns in DMCLibraries) ? '<ns>.<resolved item name>' : Default
- lookup = (ns not in DMCLibraries AND Default != 'Default') ? Default : candidate
- result = ShapeExists(lookup) ? candidate : Default

v01 (Prop) always tries '<ns>.<item>' and falls back to 'Circle_Thick'.

When CRSL Found is true, FkChain and FkArray skip Set Control Scale. FkArray forwards also contains a disconnected branch that calls SetupShapeLibraryFromLayer 'ik-layer' / 'fk-layer'.

**Setup.** The Root-level DMC switch comes from module metadata. The shape namespace 'fk-layer' is used by the DMC templates (MR_FN_BipedDMC, MR_ZebraDMC).

**Operators:** `RigUnit_SetupShapeLibraryFromUserData`, `RigUnit_SetupShapeLibraryFromLayer`, `RigUnit_ShapeExists`, `RigUnit_ResolveConnector`, `DISPATCH_RigVMDispatch_ArrayFind`, `DISPATCH_RigDispatch_GetModuleMetadata`

**Scale:** 9 v02 calls and 1 v01 call

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkChain/graphs.txt:188 'NameSpace=CRSL; Path=ShapeLibrary'`; `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkChain/graphs.txt:191,196 'Name=Direct Mesh Control' 'Value=fk-layer'`; `<dump>/FortniteRigs__Libraries__CRFL_Control_v001/graphs.txt:430-474 'Get Control Shape Name From Item v02'`; `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkArray/graphs.txt:51,54 'LayerName=ik-layer' 'LayerName=fk-layer'`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/Core/RigUnit_UserData.h:44-48`

#### UE2-lock-scale — Lock Scale option (per-frame scale reset)

*nice-to-have* · assets: CRM_FN_Root, CRM_FN_Body, CRM_FN_FkChain

If the public bool 'Lock Scale' is true, Forwards Solve sets local scale to (1,1,1) with propagation on each module control:
- Root: Global, Local
- Body: Body Orbit, Body, Body Proxy, plus Body Offset when it exists
- FkChain: all Controls

This is an evaluation-time override. Transform limits are not used for it.

**Setup.** Public 'Lock Scale' (bool, default false), category 'Module Options'.

**Operators:** `RigUnit_SetScale`, `DISPATCH_RigVMDispatch_ArrayIterator`

**Scale:** 3 modules

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Root__CRM_FN_Root/graphs.txt:14 'SetScale ... Space=LocalSpace ... Scale=(X=1.000000'`; `<dump>/FortniteRigs__Modules__FkSolves__Body__CRM_FN_Body/graphs.txt:21 'Set Scale'`; `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkChain/graphs.txt:35 'SetScale'`

#### UE2-prop-visibility-hosting — Prop visibility channels hosted on external/root control

*nice-to-have* · assets: CRM_FN_Prop

Construction:
- Bool 'Prop Global Vis' is created under Prop Global. Its initial value = NOT (the Control Vis Channel Host connector is connected). If the host is connected, the channel is re-hosted on it; in the templates the host is root/Global.
- Bools 'Prop Local Vis', 'Prop Control Vis', 'Aim Control Vis', 'Prop Attach 01 Vis' and 'Prop Attach 02 Vis' (all default false) are created under Prop Global and hosted on [Prop Local, Prop, Aim, Prop Attach 01, Prop Attach 02].

Forwards:
- PGV = the channel value if the channel exists, otherwise true.
- Prop Global visible = PGV.
- Each other control is visible when PGV AND its own channel is true.

**Setup.** The connector 'Control Vis Channel Host' only accepts Control elements (connection rule).

**Operators:** `RigUnit_HierarchyAddAnimationChannelBool`, `RigUnit_SetChannelHosts`, `RigUnit_ItemExists`, `RigUnit_SetControlVisibility`, `RigVMFunction_MathBoolAnd`, `RigVMFunction_MathBoolNot`

**Scale:** 6 bool channels per Prop

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Prop__CRM_FN_Prop/graphs.txt:438 'Name=Prop Global Vis'`; `<dump>/FortniteRigs__Modules__FkSolves__Prop__summary.json 'Control Vis Channel Host ... RigTypeConnectionRule ... (ElementType=Control)'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt 'Prop/Prop Global Vis ... AvailableSpaces=((Key=(Type=Control,Name="root/Global")))'`

#### UE2-shape-profile-and-offset — Shape transform compensation and chain scale profile

*nice-to-have* · assets: CRM_FN_FkChain, CRM_FN_FkArray, CRM_FN_Spine, CRM_FN_Prop

Local function 'Get Control Shape':
- shape.Transform = Shape.Transform * inverse(Control Transform Offset), so the gizmo stays aligned with the bone when an offset is applied
- shape.Name comes from Get Control Shape Name From Item (DMC lookup)
- the color override comes from the metadata color helpers

The shape Scale3D is then multiplied by RigVMFunction_AnimEvalRichCurve('Control Scale Factor Profile', ratio in [0,1]), a flat curve at 1 by default, evaluated at the iterator ratio (bone i of n).

Spine specifics:
- IK Start/End/Mid shapes use the profile at 0, 1 and 0.5.
- The last FK shape gets Z scale * 5.
- Shape rotations are multiplied by 'Controls Orient Offset' (FK/IK) or 'Sec FKs Orient Offset' (Sec FK and Local).

**Setup.** Private 'Control Scale Factor Profile' (FRuntimeFloatCurve with keys (0,1) and (1,1)).

**Operators:** `RigVMFunction_AnimEvalRichCurve`, `RigVMFunction_MathVectorScale`, `RigVMFunction_MathTransformMul`, `RigVMFunction_MathTransformInverse`, `RigVMFunction_MathQuaternionMul`

**Scale:** 4 modules

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkChain/graphs.txt:418-443 'Get Control Shape'`; `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkChain/graphs.txt 'RigVMFunction_AnimEvalRichCurve.Result -> Scale.Factor' / 'DISPATCH_RigVMDispatch_ArrayIterator.Ratio -> RerouteNode_10.Value'`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt 'Multiply_6 | B=5.000000'`

#### UE3-foot-rocker-anchor — Foot Rocker control re-anchored at the heel while idle

*nice-to-have* · assets: CRM_FN_Foot

Forward, when not interacting: SetControlOffset(Foot Rocker, global = (inverse(rockerLocal) * Delta) * FootJoint_current). Delta = rocker initial global relative to foot initial.

The control's global then equals Delta * Foot, so it keeps its bind placement relative to the moving foot joint. Offset writes set both current and initial.

A nested branch that copies the Toe IK null onto the Toe Tip Pivot null (no propagate) while the Heel control is being interacted with is unreachable: it sits behind the not-interacting branch.

**Setup.** Foot Rocker stays at the heel.

**Operators:** `RigUnit_IsInteracting`, `RigUnit_SetControlOffset`, `DISPATCH_RigDispatch_GetMetadata`, `RigVMFunction_MathTransformInverse`, `RigVMFunction_MathTransformMakeAbsolute`

**Scale:** per foot

**Evidence:** `<dump>/.../CRM_FN_Foot/graphs.txt:45 COMMENT 'Keep Foot Rocker Control At Heel'`; `<dump>/.../CRM_FN_Foot/graphs.txt:54-59,147-166`

#### UE3-footprint-proxy — Footprint display proxy control

*nice-to-have* · assets: CRM_FN_Foot

Construction spawns a transform control 'Footprint Display':
- parent = leg IK control; offset = lerp(Toe Tip Pivot, Heel Pivot, 0.5) at initial transforms;
- Settings: Proxy bIsProxy = true, ShapeVisibility = UserDefined, shape RoundedSquare_Solid;
- shape scale X = |Inner - Outer|/8, Y = |ToeTip - Heel|/8;
- color = the leg IK control's shape color.

If the IK control exists, also spawns bool channel 'Footprint Vis' (initial false) on it.

Forward: visible = Footprint Vis && IK. It is a non-keyable sole outline.

**Setup.** 'Footprint Vis' toggle on the leg IK control.

**Operators:** `RigUnit_HierarchyAddControlTransform (proxy)`, `RigVMFunction_MathTransformLerp`, `RigVMFunction_MathVectorDistance`, `RigUnit_SetControlColor`, `RigUnit_HierarchyGetShapeSettings`

**Scale:** per foot

**Evidence:** `<dump>/.../CRM_FN_Foot/graphs.txt:455,457 'Name=Footprint Display' / 'Name=Footprint Vis'`; `<dump>/.../CRM_FN_Foot/graphs.txt:464-465 'Divide | B=8.000000' + 834-843`; `<dump>/.../CRM_FN_Foot/regen.py SpawnControl_6.Settings Proxy=(bIsProxy=true,ShapeVisibility=UserDefined)`

#### UE3-module-vis-hide — Per-module visibility channel that hides controls and collapses the limb

*nice-to-have* · assets: CRM_FN_IkFk2Bones

Construction spawns a bool channel '<ModuleName> Vis' (e.g. 'Leg L Vis', initial true). Its parent is the root module metadata 'Global Control' (root/Global), falling back to the IK control.

Forward: if the channel is false:
- all controls in the module (GetItemsInModule type Control) are hidden;
- every Bones[i] gets local scale (0,0,0);
- the rest of the solve is skipped.

If true, the full solve runs.

**Setup.** '<Module> Vis' bool channel on root/Global.

**Operators:** `RigUnit_GetModuleName`, `RigVMFunction_StringConcat`, `RigUnit_HierarchyAddAnimationChannelBool`, `RigUnit_GetItemsInModule`, `RigUnit_SetControlVisibility`, `RigUnit_SetScale`

**Scale:** 1 per limb

**Evidence:** `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:1045-1049,1606-1613 'Concat | B= Vis' / 'Get Module Item Metadata | Name=Global Control'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:271-279,567-578 'Get Items In Module' / 'Set Scale ... Scale=(X=0,Y=0,Z=0)' / COMMENT 'Module Vis'`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/runtime_hierarchy.txt 'CONTROL Leg L/Leg L Vis <- CONTROL:root/Global'`

#### UE3-movable-pivot-proxy — Movable pivot proxy (Construct/Forward Movable Proxy v01 library)

*nice-to-have* · assets: CRFL_Hierarchy_v001, CRM_FN_Spine, CRM_FN_Body

This library is not used by the scoped modules; functions_used lists it for CRM_FN_Spine and CRM_FN_Body.

Construct Movable Proxy v01:
- null '<Name> Null' under Proxy Parent, with the Driven control's initial rotation and the Snap To initial translation/scale;
- control '<Name>' with the given settings;
- bool 'Movable Pivot Vis' (initial = Proxy Vis) on the Driven control;
- SetControlDrivenList(proxy, [Driven]);
- null '<Name> Bfr' under the proxy with metadata IsSet = false.

Forward Movable Proxy v01:
- proxy visibility = vis channel; if visible:
  - (a) translating the proxy: buffer global = driven global, IsSet = true;
  - (b) rotating the proxy: if !IsSet, buffer = driven and IsSet = true; then driven global = buffer global, so the driven rotates about the proxy pivot;
  - otherwise: IsSet = false, proxy parent null global = (Snap To t/s, driven rotation), proxy local rotation = identity.

This is a user-relocatable rotation pivot for a control.

**Setup.** 'Movable Pivot Vis' channel on the driven control.

**Operators:** `RigUnit_IsInteracting (bIsTranslating/bIsRotating)`, `DISPATCH_RigDispatch_GetMetadata`, `DISPATCH_RigDispatch_SetMetadata`, `RigUnit_SetControlDrivenList`, `RigUnit_SetTransform`, `RigUnit_SetControlVisibility`

**Scale:** 2 uses (Spine, Body)

**Evidence:** `<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:1010-1082 'Construct Movable Proxy v01'`; `<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:1084-1196 'Forward Movable Proxy v01'`; `<dump>/functions_used.json 'Forward Movable Proxy v01': 2`

#### UE4-color-by-position — Set Contol Color By Position (world-side colouring, unused)

*nice-to-have* · assets: CRFL_Hierarchy_v001

Inputs: Controls, Center Color (default yellow 1,1,0), Left Color (green 0,1,0), Right Color (red 1,0,0), Axis (ERigControlAxis 0=X, 1=Y, 2=Z) and Flip. For each control: p = the current global translation component on Axis, negated when Flip. If |p| is nearly zero (tolerance 0.001) or p == 0, it uses Center; else if p > 0 it uses Left; else Right. Each control is coloured with SetControlColor.

**Operators:** `RigUnit_GetTransform`, `DISPATCH_RigVMDispatch_CastEnumToInt`, `RigVMFunction_MathFloatIsNearlyZero`, `RigVMFunction_MathDoubleGreater/Less`, `RigVMFunction_MathDoubleMul`, `RigUnit_SetControlColor`

**Scale:** No callers.

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:460-572 'Set Contol Color By Position' (IsNearlyZero 'Tolerance=0.001000', Multiply 'B=-1.000000')`; `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:15 library defaults Center/Left/Right colours`

#### UE4-shape-name-from-item-v01 — Get Control Shape Name From Item v01 (per-item shape lookup with fallback)

*nice-to-have* · assets: CRFL_Control_v001, CRM_FN_Prop

Inputs: Item (RigElementKey), ShapeLib Namespace (FName, default 'CRSL') and Default Shape (FName, default 'Sphere_Solid'). Output: Result. The item is resolved with ResolveConnector(Item, SkipSocket=false); a non-connector resolves to itself. candidate = NS + '.' + resolved.Name. If Default Shape == 'Default', name = candidate; otherwise name = Default Shape. Result = ShapeExists(name) ? name : 'Circle_Thick'. ShapeExists asks the rig's shape libraries through UControlRig::OnShapeExists, and names are 'Library.Shape' qualified.

**Operators:** `RigUnit_ResolveConnector`, `RigUnit_ShapeExists`, `RigVMFunction_NameConcat`, `DISPATCH_RigVMDispatch_CoreEquals`, `DISPATCH_RigVMDispatch_If`

**Scale:** 1 call site (Prop).

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Control_v001/graphs.txt:70-99 (If_1 'False=Circle_Thick', Equals 'B=Default', Concat 'B=.')`; `ue/<dump>/FortniteRigs__Modules__FkSolves__Prop__CRM_FN_Prop/graphs.txt:693 'Get Control Shape Name From Item v01_1 | ... ShapeLib Namespace=CRSL'`; `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Private/ControlRig.cpp:508-517 OnShapeExists -> UControlRigShapeLibrary::GetShapeByName`; `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Execution/RigUnit_RigModules.cpp:10-22 ResolveConnector returns input when unresolved`

#### UE6-control-visual-conventions — Face gizmo conventions (shapes, colors, limits drawing)

*nice-to-have* · assets: CRM_Zebra_Face, CRM_Monster_Face

Color code: left side blue (0,0,1), right side red (1,0,0), center yellow (1,1,0), lip macro and tweaker controls magenta (1,0,1), skull/reverse-jaw orange (1,0.5,0), squash controls cyan (0,1,1). Shape vocabulary: Default (mouth/jaw/muzzle/lid micro), Triangle_Thick (corners, lid sliders), Sphere_Solid (brows, squeeze, eye main, tongue, lip tweakers), Circle_Thick (eyes, head squash, Zebra nose), Circle_Solid (lid in/out), Box_Thick (eye aim), RoundedTriangle_Thin (teeth). Shapes are mostly offset away from the face with shape transforms. Most slider controls set bDrawLimits=false. Every control uses preferred rotation order YZX, which is not enabled.

**Setup.** See each control feature for its shape transform and scale.

**Operators:** `FRigControlSettings Shape/Limits`

**Scale:** 87 transform controls (Zebra)

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_ZebraDMC/runtime_hierarchy.txt shape_name counts (54 Default, 35 Sphere_Solid, 6 Triangle_Thick, 5 Circle_Thick, 4 Circle_Solid, 2 RoundedTriangle_Thin, 1 Box_Thick)`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/regen.py SpawnControl_*.Settings Shape=(...Color=...)`

#### UE7-dmc-gizmo-library — ModularRigGizmoLibrary_DMC and shape-library precedence

*nice-to-have* · assets: ModularRigGizmoLibrary_DMC, MR_ZebraDMC, MR_FN_BipedDMC

A shape library with the same 62 named shapes and scales as ModularRigGizmoLibrary. Examples: Arrow/Box/Circle/Diamond/HalfCircle/Hexagon/Octagon/Pyramid/QuarterCircle/RoundedSquare/RoundedTriangle/Sphere/Square/Star4/Triangle/Wedge in Thin/Thick/Solid variants, plus Circle_Pins_Thick, Square_Pins_Thick, Sphere_Pins_Small, Stick_Pins_Thick, Cross_Pins_Thick and RoundedTrapeze variants. Most use scale 0.1; Wedge uses 0.2 and RoundedTriangle 1.

Differences from the regular library:
- DefaultMaterial = ModularRigGizmoMaterial_DMC.
- XRayMaterial from /EpicControlRig.
- MaterialHoveredParameter 'Hovered' and MaterialHoveredColorParameter 'HoveredColor', which enable hover feedback.
- Pins-shape proxies come from /EpicControlRig.

Shape-name resolution (GetShapeByName): split 'Lib.Shape' at the dot. Pass 1 matches the library name (after remapping). Pass 2 ignores the namespace. Libraries are searched from last to first, and the default-shape fallback applies only to the first library (index 0) in pass 2.

**Setup.** Shape names used by the rigs, e.g. Circle_Thick, Sphere_Solid, Box_Thick, Circle_Pins_Thick.

**Operators:** `UControlRigShapeLibrary`, `UControlRigShapeLibrary::GetShapeByName`

**Scale:** 1 library, 62 shapes.

**Evidence:** `<dump>/FortniteRigs__Controls__ModularRigGizmoLibrary_DMC/asset.t3d:2-69`; `diff against <dump>/FortniteRigs__Controls__ModularRigGizmoLibrary/asset.t3d (lines 3-7 material and hover params)`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/ControlRigGizmoLibrary.cpp:139-191`

#### UE8-display-name-lookup — Per-control display names from module config arrays (Get Display Name / Get Optional Display Name)

*nice-to-have* · assets: CRM_FN_LimbTwist, CRM_FN_Spine, MR_Zebra

Both functions return DisplayName = (Num(Array) >= 1 && Index < Num(Array)) ? Array[Index] : None. The result feeds SpawnControl.Settings.DisplayName, so an empty or short array falls back to the element name. LimbTwist reads the public member 'Display Names' (TArray<FName>) with the twist index. Spine reads config arrays such as 'FK Primary Display Names' (MR_Zebra Spine: 'Pelvis FK', 'Waist FK', 'Chest FK'), and scalar display names (IK Start/Mid/End Display Name 'pelvis', 'Waist', 'Chest'; 'End Moveable Pivot Display Name'). MR_Zebra config examples: Leg Upper/Lower Twist R Display Names = ['Offset 1', 'Offset 2'], so only the first two twist controls are renamed and the rest keep 'Twist 3' and 'Twist 4'. Clavicle ['Clavicle']; finger chains ['Base', 'Mid', ...]. Sequencer shows these as 'Module / DisplayName' groups ('Leg Upper Twist R / Offset 1', 'Leg Upper Twist R / Twist 3').

**Setup.** Module config arrays of FName; the displayed label is separate from the element name.

**Operators:** `DISPATCH_RigVMDispatch_ArrayGetNum`, `DISPATCH_RigVMDispatch_ArrayGetAtIndex`, `DISPATCH_RigVMDispatch_If`, `RigUnit_HierarchyAddControlTransform.Settings.DisplayName`

**Scale:** 8 LimbTwist + 2 Spine modules

**Evidence:** `<dump>/FortniteRigs__Modules__Biped__LimbTwist__CRM_FN_LimbTwist/graphs.txt:605-640, 280-283`; `<dump>/FortniteRigs__Modules__Biped__LimbTwist__CRM_FN_LimbTwist/regen.py:28 (add_member_variable('Display Names', 'TArray<FName>', True, False))`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:1972-1998, 1131, 1229`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/asset.t3d:399-405, 510-511, 526-527, 537, 563-564`; `zebra_audition.uasset name table: '...MovieSceneControlRigParameterTrack_0.Leg Upper Twist R / Offset 1'`

#### UE8-gizmo-materials — Gizmo materials: color parameter, hover state, X-ray overlay

*nice-to-have* · assets: ModularRigGizmoLibrary, ModularRigGizmoLibrary_DMC, ModularRigGizmoMaterial, ModularRigGizmoMaterial_DMC, ModularRigXRayMaterial

Each gizmo is a static-mesh actor using a dynamic material instance of Library.DefaultMaterial. Its vector parameter Library.MaterialColorParameter ('Color' in both libraries) is set to the control's Settings.ShapeColor. ModularRigGizmoMaterial: MSM_Unlit, BLEND_Opaque, VectorParameter Color; the graph has a Divide by EyeAdaptation, which cancels exposure, into EmissiveColor. ModularRigGizmoMaterial_DMC: MSM_Unlit, BLEND_Translucent, VectorParameter Color plus ScalarParameter 'Hovered' and a 'HoveredColor' vector, with an If node feeding Opacity (comments COLOR/OPACITY). Only the DMC library sets MaterialHoveredParameter=Hovered and MaterialHoveredColorParameter=HoveredColor; the gizmo actor sets Hovered to 1 or 0 on hover and HoveredColor from the edit-mode settings. ModularRigXRayMaterial: MSM_Unlit, BLEND_Translucent, bDisableDepthTest=True, with scalar and vector parameters. It is used instead of DefaultMaterial only when the editor setting 'Show Controls As Overlay' (bShowControlsAsOverlay) is on and the X-ray material loads; otherwise DefaultMaterial is used. Custom meshes ControlRig_{Circle,Square,Cross,Stick}_Pins_6mm, Small_Sphere_Pins_Solid, RoundedTrapeze(_ArrowTip)_3mm and RoundedSquare_ArrowTip_3mm are plain StaticMeshes (slot material WorldGridMaterial, overridden at draw; Circle_Pins bounds 145x137x83). Every library shape has transform scale 0.1, except Wedge (0.2) and RoundedTriangle (no transform, so scale 1.0).

**Setup.** Control color comes from Settings.ShapeColor (side colors set by modules); the overlay is toggled from the Control Rig edit-mode toolbar.

**Operators:** `AControlRigShapeActor`, `UControlRigShapeLibrary (DefaultMaterial, XRayMaterial, MaterialColorParameter, MaterialHoveredParameter, MaterialHoveredColorParameter)`

**Scale:** 2 libraries, 3 materials, 8 custom meshes

**Evidence:** `<dump>/FortniteRigs__Controls__ModularRigGizmoLibrary/asset.t3d:2-5, 42-44 (RoundedTriangle without Transform), 57-59 (Wedge 0.2)`; `<dump>/FortniteRigs__Controls__ModularRigGizmoLibrary_DMC/asset.t3d:3-7`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/ControlRigGizmoLibrary.h:73-82`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRigEditor/Private/EditMode/ControlRigEditMode.cpp:5019-5043`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/ControlRigGizmoActor.cpp:175-196, 279, 411-420`; `<ZebraSample>/Plugins/FortniteRigs/Content/Controls/ModularRigGizmoMaterial.uasset name table: 'MSM_Unlit','BLEND_Opaque','MaterialExpressionVectorParameter','MaterialExpressionEyeAdaptation'`; `<ZebraSample>/Plugins/FortniteRigs/Content/Controls/ModularRigGizmoMaterial_DMC.uasset name table: 'BLEND_Translucent','Hovered','HoveredColor','MaterialExpressionIf'`; `<ZebraSample>/Plugins/FortniteRigs/Content/Controls/ModularRigXRayMaterial.uasset name table: 'bDisableDepthTest','BLEND_Translucent'`

#### UE8-prop-reset-controls-color — Prop 'Change Pivot' color feedback and 'Reset Controls Color'

*nice-to-have* · assets: CRM_FN_Prop, MR_Zebra, CRU_PropAim

Construction stores each Prop control's color in LinearColor metadata 'Color' (namespace Self) with GetControlColor and SetMetadata ('Store Control Color in Metadata'). Forward solve: when the Prop Global or Prop Local pivot bool channel is on ('Change Pivot'), SetControlColor paints the controls gray (0.5,0.5,0.5) and Prop Global/Local red (1,0,0), and SetControlVisibility(true) forces all controls visible ('Make all the controls visible when Change Pivot is On'). When it is off, it caches 'Prop Control World Transform' and sends RigUnit_SendEvent 'RequestAutoKey' on the Prop control. 'Reset Controls Color' then iterates [Prop Global, Prop Local, Prop Control, Aim Control, Prop Attach 01, Prop Attach 02] and calls SetControlColor(control, GetMetadata(item, 'Color', Self, default white)), restoring the stored colors. It is called in 2 places, after the SendEvent nodes.

**Setup.** Bool channels 'Change Pivot' on Prop Global and Prop Local (Prop/Change Pivot, Prop/Change Pivot_2).

**Operators:** `RigUnit_GetControlColor`, `DISPATCH_RigDispatch_SetMetadata`, `DISPATCH_RigDispatch_GetMetadata`, `RigUnit_SetControlColor`, `RigUnit_SendEvent (RequestAutoKey)`, `RigUnit_SetControlVisibility`

**Scale:** 1 Prop module per rig (MR_Zebra, MR_Monster, MR_FN_Biped, CRU_PropAim)

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Prop__CRM_FN_Prop/graphs.txt:707-731 (Reset Controls Color), 66-81 (gray/red SetControlColor), 88-94, 101, 241, 246, 367-374 (Store Control Color in Metadata)`

#### UE8-seq-selection-sets — Animator selection sets saved on sequences (with stale control names)

*nice-to-have* · assets: zebra_audition, zebra_marketingPoseFaces

UAIESelectionSets is asset user data on the LevelSequence. It holds named sets of FAIESelectionSetItemName entries (type 0 = Control Rig control, type 1 = actor) plus view data. zebra_audition defines sets such as Right_upperArm, RightElbow, RWrist, Left_PV, RightIK_HAND, Pelvis, Spine1-4, thumb1-3, fingerA_1..fingerB_3, neck1/neck2/head, UpperTeeth, earRT1-3, eyeRT/eyeLF, eyeTpRT, browRt/browCt/browLf, cornerRT, eyeSocketRT, AllBody. Their members use names such as 'Zebra_Face_CtrlRig/Teeth Tp', 'Zebra_Face_CtrlRig/Eye R', 'Zebra_Face_CtrlRig/Lid Tp R', 'Arm R/Gimbal Control Vis_2'. The face module is now named 'Face', so the 'Zebra_Face_CtrlRig/…' members no longer resolve. An importer should map 'Zebra_Face_CtrlRig/' to 'Face/' or drop those entries.

**Setup.** Selection sets panel in Sequencer (Animator Kit)

**Operators:** `UAIESelectionSets`, `FAIESelectionSetItem`, `FAIESelectionSetItemName`

**Scale:** about 40 sets in zebra_audition, 12+ in marketingPoseFaces

**Evidence:** `<UE>/Plugins/Animation/ControlRig/Source/ControlRigEditor/Private/Sequencer/SelectionSets/SelectionSets.h:19-95, 171`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRigEditor/Public/ControlRigSequencerEditorLibrary.h:1503 (GetSelectionSets)`; `zebra_audition.uasset name table: 'AIESelectionSets','Right_upperArm','Zebra_Face_CtrlRig/Teeth Tp','Arm R/Gimbal Control Vis_2'`; `zebra_marketingPoseFaces.uasset name table: 'AIESelectionSets_0.SelectionSets(11 - Value)', 'Zebra_Face_CtrlRig/Lid Bt 01 L'`

#### UE8-spine-controls-visibility — Spine/Neck 'Controls Visibility' (bool channels drive control visibility each frame)

*nice-to-have* · assets: CRM_FN_Spine, MR_Zebra

Runs in Forwards Solve. (1) IK visibility: the bool channel 'IKs Visibility Channel' (Spine/IK Vis, Neck/IK Vis) sets SetControlVisibility on [Start, Mid, End] IK controls when 'Is Neck' is true, otherwise on [Mid, End] only, so the spine Start IK (hips) is never hidden. (2) FK visibility: 'FKs Visibility Channel' (FK Vis) applies to all 'FK Controls'. (3) Secondary FK visibility: 'Sec FKs Visibility Channel' (Sec FK Vis) applies to each 'Sec FK Controls' element whose iterator Ratio != 1.0, i.e. every element except the last (e.g. Head Sec FK / Spine 05 Sec FK stay visible). The Vis channels are bool animation channels hosted on the Start IK control (Spine/IK Vis parent CONTROL Spine/Start IK).

**Setup.** Bool channels FK Vis, IK Vis and Sec FK Vis on the Start IK control of Spine and Neck.

**Operators:** `RigUnit_GetBoolAnimationChannelFromItem`, `RigUnit_SetControlVisibility`, `DISPATCH_RigVMDispatch_ArrayIterator (Ratio)`, `DISPATCH_RigVMDispatch_CoreNotEquals`

**Scale:** 2 modules (Spine, Neck), 6 channels

**Evidence:** `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:1694-1770, 28 (called from Forwards Solve graph)`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt :: 'CONTROL Spine/IK Vis parents=[\'CONTROL:Spine/Start IK\']', 'CONTROL Neck/Sec FK Vis'`

### D3 Animation channels

#### UE1-channel-hosts — Animation channels hosted on multiple controls (channel hosts)

*core* · assets: MR_Zebra, MR_Monster, MR_FN_Biped, CRU_PropAim

An animation channel (a control with animation_type ANIMATION_CHANNEL, parented under its owner control) can be shown on extra 'host' controls. RigUnit_SetChannelHosts calls AddChannelHost(channel, host), which appends the host key to the channel's Settings.Customization.AvailableSpaces. Rules: the channel must be an animation channel; the host must be a non-channel control, must not be the channel's parent, and must not be a duplicate.

Zebra examples:
- 'Leg L/Ik Fk Switch' (on Leg L/IK) is also hosted on FK 0, FK 1, FK 2, FK 0 Gimbal, FK 2 Gimbal, Mid, PV, IK Gimbal and IK Base.
- 'Leg L/Upper Segment Scale' and 'Lower Segment Scale' are hosted on FK 0/1/2 and IK Base.
- 'Arm L/IK End Align' is hosted on 'Arm L/IK Rotation'.
- 'Leg L/PV Twist Follow' is hosted on PV.
- 'Spine/IK Vis', 'FK Vis' and 'Sec FK Vis' are hosted on all 7 main spine controls.
- 'Spine/Mid Blend' is hosted on Mid FK, End FK and End IK.
- 'Prop/Prop Global Vis' is hosted on root/Global (via connector 'Control Vis Channel Host').
- The Gimbal Control Vis channels are hosted on their FK control.

**Setup.** Channels appear in the details/channel box of every host control, so the IK/FK switch can be edited from any limb control.

**Operators:** `RigUnit_SetChannelHosts`, `URigHierarchyController::AddChannelHost`, `FRigControlSettings.Customization.AvailableSpaces`

**Scale:** Channels with extra hosts: 49 in Zebra body, 16 in Monster body, 49 in Biped, 5 in PropAim.

**Evidence:** `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/Execution/RigUnit_DynamicHierarchy.h:133-160`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Rigs/RigHierarchyController.cpp:4593-4653`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:2066 'CONTROL Leg L/Ik Fk Switch' customization=(AvailableSpaces=((Key=(Type=Control,Name="Leg L/FK 0"))...`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1907 'CONTROL Prop/Prop Global Vis ... AvailableSpaces=((Key=(Type=Control,Name="root/Global")))'`; `<dump>/units_used.json: RigUnit_SetChannelHosts 22`

#### UE6-jaw-channel-host — Jaw Attributes: channel host for lip/mouth sliders

*core* · assets: CRM_Zebra_Face, CRM_Monster_Face

Construction function Jaw Attributes spawns animation channels on the jaw control and stores their keys in member variables. The float channels are Roll Tp (-100..200), Roll Bt (-100..200), All Tp, All Bt, Mid Tp, Mid Bt, Mouth Squetch and Muzzle Squetch (all -200..200, init 0, limits enabled), plus the bool Lip Tweaker Vis (false). Monster adds Ch Tp, Ch Bt, Puff Tp and Puff Bt (0..100). Zebra's input is 'Jaw Control' and the body reads the Jaw Control variable; Monster's input is 'Parent'.

**Setup.** The channels appear as float/bool attributes on the jaw control, with display names exactly as listed.

**Operators:** `RigUnit_HierarchyAddAnimationChannelFloat`, `RigUnit_HierarchyAddAnimationChannelBool`, `RigVMVariableNode (setter)`

**Scale:** Zebra 9 channels, Monster 13

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:2848-2914 Jaw Attributes`; `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/graphs.txt:3163 SpawnAnimationChannel_1 Name=Ch Tp MaximumValue=100`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:2185-2193 Face/Roll Tp..Lip Tweaker Vis par=C:Face/jaw`

#### UE2-body-channels-hosts — Body visibility/aim channels with multi-host

*important* · assets: CRM_FN_Body

Channels spawned under their natural parent and then re-hosted with RigUnit_SetChannelHosts so they appear on other controls:
- Body Orbit Vis (on Body Orbit; hosts [Body])
- Body Aim Vis, Aim Weight, Aim Twist (on Body Aim; hosts [Body, Body Orbit])
- Body Offset Vis (on Body Offset; hosts [Body])
- Movable Pivot Vis (hosts [Body])

Forwards step A:
- Body Orbit visible = Body Orbit Vis
- Body Aim visible = Body Aim Vis
- Body Offset visible = Body Offset Vis (only when the offset control exists)

**Setup.** Defaults and limits:
- Body Orbit Vis: false
- Body Aim Vis: false
- Aim Weight: 0, range [0,1] limited
- Aim Twist: 0, range [0,1] limited
- Body Offset Vis: true

The runtime stores channel hosts in the control's customization AvailableSpaces list.

**Operators:** `RigUnit_HierarchyAddAnimationChannelBool`, `RigUnit_HierarchyAddAnimationChannelFloat`, `RigUnit_SetChannelHosts`, `RigUnit_GetBoolAnimationChannelFromItem`, `RigUnit_SetControlVisibility`

**Scale:** 5-6 channels per rig

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Body__CRM_FN_Body/graphs.txt:250,270,274,286,325 channel spawns`; `<dump>/FortniteRigs__Modules__FkSolves__Body__CRM_FN_Body/graphs.txt:255,264,272,276,281 'Set Channel Hosts'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt 'Body/Aim Weight ... customization=(AvailableSpaces=((Key=(Type=Control,Name="Body/Body")),(Key=(Type=Control,Name="Body/Body Orbit")))'`

#### UE2-fkarray-visibility-and-spaces — FK array per-control Visibility channel and full/orient spaces

*important* · assets: CRM_FN_FkArray

Forwards:
- A: for each control i, visible = the bool channel VisChannels[i] ('Visibility', parented to that control).
- B: Evaluate Space Nulls(Orient Space Nulls, Combined Orient Spaces, OrientOnly = true), then Evaluate Space Nulls(Space Nulls, Combined Spaces, OrientOnly = false).

Construction builds the space nulls for every control i:
- Construct Space Nulls(Controls[i], Spaces, SpaceNullParent = hierarchy parent of bone i, OrientOnly = false), appending the Spaces list to Combined Spaces once per control so the null and space lists stay index-aligned.
- The same for Orient Spaces with OrientOnly = true.

**Setup.** Public 'Controls Visibility Initials' (bool array) gives the channel defaults. The runtime channel is named 'Visibility' (BOOL, group_with_parent_control).

**Operators:** `RigUnit_GetBoolAnimationChannelFromItem`, `RigUnit_SetControlVisibility`, `DISPATCH_RigVMDispatch_ArrayAppend`

**Scale:** 1 channel per FkArray control

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkArray/graphs.txt:39-40 'Orient Only=true' / 'Orient Only=false'`; `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkArray/regen.py:749,773 'Construct Space Nulls v01.Orient Only', 'false'/'true'`

#### UE2-spine-channels-visibility — Spine mode visibility channels (IK/FK/Sec FK Vis) with 7 hosts

*important* · assets: CRM_FN_Spine

Construction spawns three bools under Start IK: 'IK Vis' (false), 'FK Vis' (true) and 'Sec FK Vis' (false). Each is hosted on [Start FK, End FK, End IK, SecFK[0], Mid FK, SecFK[-1], Mid IK].

Forwards ('Controls Visibility'):
- IK controls visible = IK Vis. The IK set is [Mid, End] for the spine and [Start, Mid, End] for the neck; the spine's Start IK is always visible because it hosts the channels.
- FK controls visible = FK Vis.
- Sec FK controls visible = Sec FK Vis, except the last one (ratio == 1), which stays visible.

The backwards solve reads FK Vis and Sec FK Vis as mode switches and sets IK Vis to true.

**Operators:** `RigUnit_HierarchyAddAnimationChannelBool`, `RigUnit_SetChannelHosts`, `RigUnit_SetControlVisibility`, `RigUnit_GetBoolAnimationChannelFromItem`, `DISPATCH_RigVMDispatch_CoreNotEquals`

**Scale:** 3 channels per spine/neck

**Evidence:** `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:347-349 'Name=IK Vis' 'Name=FK Vis' 'Name=Sec FK Vis'`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt (RigVMFunctionLibrary.Controls Visibility) 'Not_Equals B=1.000000' on Ratio`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt 'Spine/IK Vis ... AvailableSpaces=((Key=(Type=Control,Name="Spine/Start FK"))'`

#### UE4-channel-hosting — Animation channels as child controls with extra hosts

*important* · assets: CRFL_Control_v001, CRFL_Hierarchy_v001

Bool animation channels are spawned with RigUnit_HierarchyAddAnimationChannelBool under a parent control (min/max limits, initial value). RigUnit_SetChannelHosts then adds more host controls, and each host exposes the same channel element. Two library channels follow this pattern: 'Gimbal Control Vis' (parent = gimbal, host = source control) and 'Movable Pivot Vis' (parent = driven control, initial = the Proxy Vis input). The runtime reads them with RigUnit_GetBoolAnimationChannelFromItem(bInitial=false).

**Setup.** The channels show on the host controls as bool attributes: 'Gimbal Control Vis' and 'Movable Pivot Vis'.

**Operators:** `RigUnit_HierarchyAddAnimationChannelBool`, `RigUnit_SetChannelHosts`, `RigUnit_GetBoolAnimationChannelFromItem`

**Evidence:** `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Execution/RigUnit_DynamicHierarchy.cpp:137-152 SetChannelHosts -> Controller->AddChannelHost`; `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Execution/RigUnit_DynamicHierarchy.cpp:1099-1129 AddAnimationChannelBool creates a Bool control with limits`; `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:1043 'Spawn Bool Animation Channel | Name=Movable Pivot Vis'`; `ue/<dump>/FortniteRigs__Templates__MR_FN_Biped/runtime_hierarchy.txt:1504 'CONTROL Spine/Movable Pivot Vis parents=[CONTROL:Spine/End IK]'`

#### UE4-unit-interaction-channels — Engine units: viewport interaction state and channel reads

*important* · assets: CRFL_Hierarchy_v001

RigUnit_IsInteracting outputs bIsInteracting, bIsTranslating, bIsRotating and bIsScaling, taken from the UnitContext.InteractionType bit flags, plus Items (the elements currently being manipulated). RigUnit_GetBoolAnimationChannelFromItem(Item, bInitial) reads the value of a bool channel control. Together these enable the gesture-aware movable pivot.

**Operators:** `RigUnit_IsInteracting`, `RigUnit_GetBoolAnimationChannelFromItem`

**Scale:** 1 node each (Forward Movable Proxy).

**Evidence:** `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Execution/RigUnit_IsInteracting.cpp:8-17`; `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/Hierarchy/RigUnit_ControlChannelFromItem.h:41-60`

#### UE2-root-matching-enum-channel — Enum-typed integer animation channel

*nice-to-have* · assets: CRM_FN_Root, CREnum_RootMatching

Construction calls RigUnit_HierarchyAddAnimationChannelInteger with Name='Bake Root On', InitialValue=0, Min=0, Max=100, limits enabled, and ControlEnum pointing at the UserDefinedEnum CREnum_RootMatching. The UI shows the integer as a dropdown ('Global Control' / 'Local Control' / 'Root Control'). The channel key is stored in the 'Root Matching Channel' variable.

**Setup.** Channel parented to Global; group_with_parent_control=True.

**Operators:** `RigUnit_HierarchyAddAnimationChannelInteger`, `UserDefinedEnum`

**Scale:** 1 per rig

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Root__CRM_FN_Root/graphs.txt:128 'Name=Bake Root On; ... ControlEnum=/FortniteRigs/Modules/FkSolves/Root/CREnum_RootMatching'`; `<dump>/FortniteRigs__Modules__FkSolves__Root__CREnum_RootMatching/asset.t3d 'DisplayNameMap'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt 'root/Bake Root On ... control_type=INTEGER'`

### D4 Spaces

#### UE1-layout-root — Root module layout (Global / Local / Root motion)

*core* · assets: MR_Zebra, MR_Monster, MR_FN_Biped

Controls:
- root/Global: parentless EulerTransform, shape RoundedSquare_ArrowTip_Thick, scaled by 'Global Control Scale' (1.1 on Zebra).
- root/Local: child of Global, RoundedSquare_Thick.
- root/Root: child of Global, Arrow_Thick; the root-motion control for bone 'root', with spaces 'Local Control' (root/Local) and 'Spine/Pelvis TXY'.

Channels:
- On Global: Bake Root On (INTEGER enum Global/Local/Root Control), Control Path Vis (bool), Control Path Distance (float, 50).
- On Local: Control Path Vis_2 and Control Path Distance_2.

Connectors: RootJoint -> bone root (primary); Body -> pelvis (ChildOfPrimary). Config: Control Scale 1.8, Lock Scale True. Metadata published: Global/Local/Root/Body Control keys and side colors. root/Local is the default parent for Body, Prop, the limb IK controls and the Arm 'IK Rotation'.

**Setup.** 8 controls (3 transforms, 5 channels).

**Operators:** `CRM_FN_Root`

**Scale:** Identical in Zebra, Monster and Biped (Biped keeps default scales).

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1875 'CONTROL root/Global parents=[]'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1876 'CONTROL root/Bake Root On ... control_type=INTEGER'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:2147 'CONTROL root/Root'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/asset.t3d:374-376 'Control Scale=1.800000' 'Global Control Scale=1.100000' 'Lock Scale=True'`

#### UE1-space-switch-nulls — Space switching via per-space nulls with labeled AvailableSpaces

*core* · assets: MR_Zebra, MR_Monster, MR_FN_Biped, CRU_PropAim

For each target in a module's space array connector, the module spawns a null parented directly under that target. IkFk2Bones creates '<Module>/<TargetShortName> IK Null' and '<Module>/<TargetShortName> PV Null'. It registers them on the control as Customization.AvailableSpaces entries {Key=null, Label=<target display>}. The control's default parent is a separate 'Parent Buffer' or 'Default ... Space' null, or the connected Parent.

Zebra Leg L/IK: parent root/Local; spaces spine_05, Body/Body, Root/Local and Root/Global. The nulls are 'Leg L/spine_05 IK Null' (under bone spine_05), 'Leg L/Body IK Null' (under Body/Body), 'Leg L/Local IK Null' (under root/Local) and 'Leg L/Global IK Null' (under root/Global).

Arm L/IK: 9 spaces (clavicle_l, spine_05, pelvis, Root/Local, Body/Body, Root/Global, Prop, Prop Attach 01, Prop Attach 02). The prop space nulls are parented under the Prop module controls ('Arm L/Prop IK Null' under CONTROL Prop/Prop).

PV controls get the same list plus the IK control ('Leg L/PV' spaces: spine_05, Body/Body, Root/Local, Root/Global, Leg L/IK). Other examples:
- root/Root: spaces root/Local ('Local Control') and 'Spine/Pelvis TXY', a parentless null.
- Body/Body: space Body/Body Aim ('Aim').
- Prop/Prop: bone keys hand_r, hand_l, spine_05 used directly.
- Prop Attach 02: space Prop Attach 01.

Switching uses RigUnit_SwitchParent / SetDefaultParent with space-switch compensation from the engine.

**Setup.** Space menu entries per control, labeled by target name. Controls with spaces: 22 in Zebra body, 10 in Monster body, 39 in Biped.

**Operators:** `RigUnit_AddAvailableSpaces`, `RigUnit_SetDefaultParent`, `RigUnit_SwitchParent`, `RigUnit_HierarchyAddNull`, `FRigControlElementCustomization.AvailableSpaces`

**Scale:** About 2 nulls per IK space per limb: Zebra Arm L has 9 IK and 9 PV nulls.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:2041 'CONTROL Leg L/IK parents=['CONTROL:root/Local']' customization=(AvailableSpaces=((Key=(Type=Null,Name="Leg L/spine_05 IK Null"),Label="spine_05"),...(Key=(Type=Null,Name="Leg L/Global IK Null"),Label="Root/Global"))`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:2148 "NULL Leg L/Global IK Null parents=['CONTROL:root/Global']"`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1901 "NULL Arm L/Prop IK Null parents=['CONTROL:Prop/Prop']"`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:2147 'CONTROL root/Root ... AvailableSpaces=((Key=(Type=Control,Name="root/Local"),Label="Local Control"),(Key=(Type=Null,Name="Spine/Pelvis TXY")))'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:2160 "NULL Spine/Pelvis TXY parents=[]"`; `<dump>/units_used.json: RigUnit_AddAvailableSpaces 8, RigUnit_SetDefaultParent 8, RigUnit_SwitchParent 2`

#### UE2-root-control-stack — Root module Global/Local/Root control stack

*core* · assets: CRM_FN_Root, MR_FN_Biped, MR_Zebra, MR_Monster

Construction spawns three EULER_TRANSFORM animation controls at the identity pose.
- 'Global' has no parent.
- 'Local' and 'Root' are both children of Global. Root is not a child of Local.
- Root gets Local as an available space with label 'Local Control'.

Other modules use Global and Local as world/character spaces through connectors. Examples: Body/Parent -> root/Local; Prop/Parent -> root/Local; Spine Start/End Orient Spaces -> [Root/Global, Root/Local, Body/Body].

**Setup.** Shapes and scales:
- Global: ModularRigGizmoLibrary.RoundedSquare_ArrowTip_Thick, color (0.434,0.028,0.050), shape scale 8.
- Local: RoundedSquare_Thick, color (0.839,0.478,0), scale 6.
- Root: Arrow_Thick, color (0.031,0.768,0.002), scale 3, shape rotated 180 degrees about Z (quat (0,0,1,0)).

Preferred rotation order YZX (not enabled); no limits. At runtime Root's available spaces are 'Local Control' plus 'Spine/Pelvis TXY' (added by the Spine module).

**Operators:** `RigUnit_HierarchyAddControlTransform`, `RigUnit_AddAvailableSpaces`

**Scale:** 1 per rig (3 controls). Used in MR_FN_Biped, MR_FN_BipedDMC, MR_Zebra, MR_ZebraDMC and MR_Monster.

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Root__CRM_FN_Root/graphs.txt:69-71 'Name=Global' 'Name=Local' 'Name=Root'`; `<dump>/FortniteRigs__Modules__FkSolves__Root__CRM_FN_Root/graphs.txt:127 'Label="Local Control"'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt 'CONTROL root/Root parents=['CONTROL:root/Global']'`

#### UE2-space-nulls — Space / Orient-space nulls (Construct + Evaluate Space Nulls v01)

*core* · assets: CRM_FN_FkChain, CRM_FN_FkArray, CRM_FN_Spine, CRFL_Control_v001

Construct Space Nulls v01(Driven, Spaces[], SpaceNullParent, OrientOnly). For each space s, with short = short name of s:
1. Spawn null '<Driven> <short> Orient Space' when OrientOnly, else '<Driven> <short> Space'.
2. Its parent is SpaceNullParent. Its global transform is the current global of parent(Driven), i.e. the driven control's offset null.
3. AddAvailableSpaces(Driven, [{null, label = OrientOnly ? '<short> Orient' : '<short>'}]).
4. The null is appended to the output list.

Evaluate Space Nulls v01(Spaces, SpaceNulls, OrientOnly), each frame, for each i (it prints an error if there are more spaces than nulls):

M = MakeRelative(null[i] initial global, space[i] initial global) * space[i] current global

- If OrientOnly, only the null's global rotation is set to M.rotation, so the null keeps its hierarchy position but follows the space's rotation.
- Otherwise the full global transform is set to M.

The animator switches the control's active space to one of these nulls through UE's space switching. Evaluation runs in both forward and backward solves.

**Setup.** Space targets come from array connectors: 'Orient Spaces' (FkChain, FkArray), 'Spaces' (FkArray, full spaces), and 'Start/End Orient Spaces' (Spine; e.g. [Root/Global, Root/Local, Body/Body, spine_05]). The runtime labels are 'Global Orient', 'Local Orient' and 'Body Orient'.

**Operators:** `RigUnit_HierarchyAddNull`, `RigUnit_GetItemShortName`, `RigUnit_AddAvailableSpaces`, `RigVMFunction_NameConcat`, `RigVMFunction_MathTransformMakeRelative`, `RigVMFunction_MathTransformMul`, `RigUnit_SetRotation`, `RigUnit_SetTransform`, `DISPATCH_RigVMDispatch_Print`

**Scale:** 16 Evaluate and 8 Construct calls; Spine creates 4 null sets (Start/End × IK/FK), Neck has 4 spaces each

**Evidence:** `<dump>/FortniteRigs__Libraries__CRFL_Control_v001/graphs.txt:326-403 'Construct Space Nulls v01' ('B= Space', 'B= Orient')`; `<dump>/FortniteRigs__Libraries__CRFL_Control_v001/graphs.txt:268-325 'Evaluate Space Nulls v01'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt 'Clavicle L/FK ... AvailableSpaces=((Key=(Type=Null,Name="Clavicle L/FK Body Orient Space"),Label="Body Orient"))'`

#### UE3-fk-orient-spaces — FK orient-only space switching (Construct and Evaluate Space Nulls v01) with default space index

*core* · assets: CRM_FN_IkFk2Bones, CRFL_Control_v001

Construction runs Construct Space Nulls v01(Driven = FK Controls[0], Spaces = FK Spaces, Space Null Parent = parent of parent of FK 0 (= limb Parent), Orient Only = true). For each space it:
- spawns null '<Driven> <ShortName> Orient Space' under Space Null Parent, at the current global transform of the driven control's parent;
- calls AddAvailableSpaces(driven, [null, label '<ShortName> Orient']).

If 0 <= Default FK Space Index < Num(space nulls), it calls SwitchParent(FK 0 -> SpaceNulls[idx], ParentItem, maintain global). Arms use index 0, which is spine_05.

Forward and Backwards Solve run Evaluate Space Nulls v01(Orient Only). For each i:
- G = (SpaceNull_i initial global relative to Space_i initial global) * Space_i current global;
- orient-only mode sets only the null's global rotation (a Print warns if there are more spaces than nulls).

The null therefore follows the limb parent's position but the space target's rotation, as an orient space.

**Setup.** FK 0 space list: Default FK Space (the default parent) plus '<target> Orient' entries (Arm: spine_05, Local, Body; Leg: Body, Local).

**Operators:** `RigUnit_HierarchyAddNull`, `RigUnit_GetItemShortName`, `RigUnit_AddAvailableSpaces`, `RigUnit_SwitchParent`, `RigVMFunction_MathTransformMakeRelative`, `RigVMFunction_MathTransformMul`, `RigUnit_SetRotation`

**Scale:** 2-3 orient spaces per limb

**Evidence:** `<dump>/FortniteRigs__Libraries__CRFL_Control_v001/graphs.txt:326-387 'Construct Space Nulls v01'`; `<dump>/FortniteRigs__Libraries__CRFL_Control_v001/graphs.txt:268-324 'Evaluate Space Nulls v01'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:980 'SwitchParent | Mode=ParentItem; bMaintainGlobal=True' + 1471-1483`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:237 'Evaluate Space Nulls v01 | Orient Only=true'`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/runtime_hierarchy.txt 'CONTROL Arm L/FK 0 <- NULL:Arm L/Default FK Space, NULL:Arm L/FK 0 spine_05 Orient Space'`

#### UE3-ik-spaces — IK and PV space switching via per-space nulls

*core* · assets: CRM_FN_IkFk2Bones

Construction: SpaceList = IK Spaces + [IK Control]. For each item not equal to the resolved End bone:
- If item != IK Control: spawn null '<item> IK Null' parented to item, at the IK control parent's initial global transform, and call AddAvailableSpaces(IK Control, [(null, label = item name)]).
- Always: spawn null '<item> PV Null' parented to item, at the PV control's initial global transform, and call AddAvailableSpaces(PV Control, [(null, label = item name)]).

The PV control can follow the IK control ('IK PV Null'), but IK does not get itself as a space. Space switching and its compensation are the engine's multi-parent spaces. Nulls are placed so each space starts at the bind pose.

**Setup.** IK space list: default root/Local plus clavicle, spine_05, pelvis, Body, Global, Prop... PV spaces: the same plus IK.

**Operators:** `RigUnit_HierarchyAddNull`, `RigUnit_AddAvailableSpaces`, `RigVMFunction_NameConcat`, `DISPATCH_RigVMDispatch_CoreNotEquals`

**Scale:** 4-9 spaces per IK control

**Evidence:** `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:798 COMMENT 'IK Spaces - Applied to IK and PV Control'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:832,835 'Concat | B= IK Null' / 'B= PV Null'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:1275-1284,1332-1345,1495-1500`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/runtime_hierarchy.txt 'NULL Arm L/Prop IK Null' / 'NULL Leg L/IK PV Null'`

#### UE4-construct-space-nulls — Construct Space Nulls v01 (space-switch targets per control)

*core* · assets: CRFL_Control_v001, CRM_FN_Spine, CRM_FN_FkArray, CRM_FN_FkChain, CRM_FN_IkFk2Bones, MR_Zebra

Construction. Inputs: Driven Control, Spaces (RigElementKey[]), Space Null Parent and Orient Only (bool). Output: Output Space Nulls (RigElementKey[]), stored in the local variable 'Space Nulls'. For each space S: label = ShortName(S), with ' Orient' appended when Orient Only; nullName = Driven.Name + ' ' + label + ' Space'. It spawns a Null named nullName under Space Null Parent. The null's global transform is the current global transform of the driven control's default (first) parent, which equals initial during construction, so the null coincides with the control's offset frame. It then calls AddAvailableSpaces(Driven, [{Key=null, Label=label}]) to register the null in the control's space-switch list, and appends the null to the output.

**Setup.** The animator sees labels such as 'spine_05 Orient', 'Local Orient' and 'Body Orient' in the control's space-switch menu. The multi-parent control has its default FK space null at weight 0 and the chosen space null at weight 1 (weights=(...0...),(...1...)).

**Operators:** `RigUnit_GetItemShortName`, `RigUnit_HierarchyGetParent (bDefaultParent=true)`, `RigUnit_GetTransform`, `RigUnit_HierarchyAddNull (GlobalSpace)`, `RigUnit_AddAvailableSpaces`, `RigVMFunction_NameConcat`, `DISPATCH_RigVMDispatch_ArrayAdd`, `DISPATCH_RigVMDispatch_ArrayIterator`

**Scale:** 8 call sites (Spine 4, FkArray 2, FkChain 1, IkFk2Bones 1). Orient Only=true everywhere except one FkArray call.

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Control_v001/graphs.txt:326-387 'Construct Space Nulls v01' (Concat_1 'B= Space', Concat_3 'B= Orient', AddAvailableSpaces)`; `ue/<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:855 AvailableSpaces=((Key=(Type=Null,Name="Arm L/FK 0 spine_05 Orient Space"),Label="spine_05 Orient"),(..."Arm L/FK 0 Local Orient Space"...),(..."Arm L/FK 0 Body Orient Space"...))`; `ue/<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:864 'NULL Arm L/FK 0 spine_05 Orient Space parents=[BONE:clavicle_l]'`; `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Execution/RigUnit_DynamicHierarchy.cpp:106-135 AddAvailableSpaces -> Controller->AddAvailableSpace`

#### UE4-evaluate-space-nulls — Evaluate Space Nulls v01 (space nulls follow their spaces, full or orient-only)

*core* · assets: CRFL_Control_v001, CRM_FN_Spine, CRM_FN_FkArray, CRM_FN_FkChain, CRM_FN_IkFk2Bones

Runs in Forwards Solve and again in Backwards Solve. Inputs: Spaces, Space Nulls (parallel arrays) and Orient Only. For each index i: if i >= Num(SpaceNulls), it prints 'Error: Space array is larger than Space Nulls array'. Otherwise N = SpaceNulls[i] and S = Spaces[i]. local = MakeRelative(Global=InitialGlobal(N), Parent=InitialGlobal(S)), i.e. N_init relative to S_init. target = local * CurrentGlobal(S) (UE order: local first, then S_cur); in column-vector form M_N = M_Scur * inv(M_Sinit) * M_Ninit. If Orient Only, it calls SetRotation(N, target.Rotation, global) and the null keeps its hierarchy-inherited translation. Otherwise it calls SetTransform(N, target, global). Both use weight 1 and propagate to children. The result is a rigid, offset-preserving follow of the space element, run every frame before the controls are evaluated.

**Operators:** `RigUnit_GetTransform (initial and current, global)`, `RigVMFunction_MathTransformMakeRelative`, `RigVMFunction_MathTransformMul`, `RigUnit_SetRotation`, `RigUnit_SetTransform`, `RigVMFunction_MathIntLess`, `DISPATCH_RigVMDispatch_ArrayGetNum`, `DISPATCH_RigVMDispatch_Print`, `RigVMFunction_ControlFlowBranch`

**Scale:** 16 call sites: Spine 8 (4 forward + 4 backward), FkArray 4, FkChain 2, IkFk2Bones 2.

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Control_v001/graphs.txt:268-324 'Evaluate Space Nulls v01' (GetTransform_7/_11 bInitial=true, GetTransform_10 bInitial=False)`; `ue/<dump>/FortniteRigs__Libraries__CRFL_Control_v001/graphs.txt:294 Print 'Space array is larger than Space Nulls array'`; `UE_5.8/Engine/Plugins/Runtime/RigVM/Source/RigVM/Private/RigVMFunctions/Math/RigVMFunction_MathTransform.cpp:66-76 MakeRelative = Global.GetRelativeTransform(Parent)`; `ue/<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:51-54 (forward, Orient Only=true) and :1257-1304 ('RigVMModel Backward Solve')`

#### UE4-unit-parent-space-api — Engine units: parent/space/driven-list management

*core* · assets: CRFL_Control_v001, CRFL_Hierarchy_v001

RigUnit_SetDefaultParent(Child, Parent) calls AddParent with weight 1, maintain global and remove-all-other-parents, making Parent the sole default parent. RigUnit_AddAvailableSpaces(Control, [{Key, Label}]) appends to the control's space-switch list (customization.AvailableSpaces) and warns on missing elements. RigUnit_HierarchyGetParent returns GetFirstParent (cached) when bDefaultParent, else the active parent. RigUnit_SetChannelHosts(Channel, Hosts) adds channel hosts. RigUnit_SetControlDrivenList(Control, Driven) swaps in a new DrivenControls list, keeps the previous list, and notifies ControlDrivenListChanged. RigUnit_CollectionChildrenArray(Parent, IncludeParent, Recursive, DefaultChildren, TypeToSearch) collects children.

**Operators:** `RigUnit_SetDefaultParent`, `RigUnit_AddAvailableSpaces`, `RigUnit_HierarchyGetParent`, `RigUnit_SetChannelHosts`, `RigUnit_SetControlDrivenList`, `RigUnit_CollectionChildrenArray`

**Scale:** HierarchyGetParent 10, SetDefaultParent 2, AddAvailableSpaces 1, SetChannelHosts 1, SetControlDrivenList 1, CollectionChildrenArray 1.

**Evidence:** `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Execution/RigUnit_DynamicHierarchy.cpp:78-152`; `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Public/Rigs/RigHierarchyController.h:730 AddParent(child,parent,weight,bMaintainGlobalTransform,bRemoveAllParents,label)`; `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Execution/RigUnit_Hierarchy.cpp:10-42`; `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Hierarchy/RigUnit_SetControlDrivenList.cpp:27-44`; `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/Execution/RigUnit_Collection.h:203-245`

#### UE6-head-attach-null — Head Attach Null (face root space following the Parent connector)

*core* · assets: CRM_Zebra_Face, CRM_Monster_Face

Construction spawns the null 'Head Attach Null' with no parent at the global transform of bone 'head' (HierarchyAddNull_2 <- GetTransform_3 head GlobalSpace). As the first forward step (after model_edits on Monster), ParentConstraint_37 sets Child=Null Head Attach Null with parent Connector 'Parent' at weight 1, bMaintainOffset=True, all channels, Average interpolation. The null therefore rigidly follows the resolved connector target (the head bone). The jaw, Muzzle, Reverse Jaw, Head Squash and (Zebra) Cheek controls are parented to it.

**Setup.** Null only; there is no animator control.

**Operators:** `RigUnit_HierarchyAddNull`, `RigUnit_ParentConstraint`

**Scale:** 1 null, 1 constraint

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:663 ParentConstraint_37 Child=(Type=Null,Name=Head Attach Null) Parents=Connector Parent`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:2181 NULL Face/Head Attach Null par=[]`

#### UE8-seq-space-channels — Sequencer space-switch channels (Parent / World / ControlRig keys)

*core* · assets: zebra_audition, zebra_marketingPoseFaces, MR_Zebra

For controls that have spaces, the section keeps an FSpaceControlNameAndChannel {ControlName, FMovieSceneControlRigSpaceChannel}. The channel's step keys are FMovieSceneControlRigSpaceBaseKey {SpaceType Parent|World|ControlRig, ControlRigElement}. At evaluation: Parent calls SwitchToParent(control, GetDefaultParent); World calls SwitchToParent(control, WorldSpaceReferenceKey); ControlRig calls SwitchToParent(control, ControlRigElement). URigHierarchy::SwitchToParent adds the target as an extra parent with weight 0 if it is not already one, then sets a one-hot weight array (1 on the chosen parent). The keyed local transform values are interpreted in that parent. zebra_audition contains all three key types. Its space targets are the Orient Space nulls 'Arm R/FK 0 Body Orient Space' and 'Arm R/FK 0 spine_05 Orient Space' (multi-parent nulls under clavicle_r) and 'Neck/End FK Global Orient Space'. zebra_marketingPoseFaces uses only Parent and ControlRig keys (target 'Neck/End FK Global Orient Space'). MR_Zebra_Take1 has no space channel structs. A port must replay these step keys as parent-weight switches evaluated before the control values are applied.

**Setup.** Space keys are shown per control in Sequencer, with targets such as Body/spine_05/Global/Local Orient Space nulls created by the modules.

**Operators:** `FMovieSceneControlRigSpaceChannel`, `FMovieSceneControlRigSpaceBaseKey`, `UControlRig::SwitchToParent`, `URigHierarchy::SwitchToParent`

**Scale:** 2 of 3 sequences; at least 3 distinct space targets referenced

**Evidence:** `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Sequencer/MovieSceneControlRigSpaceChannel.h:26-55`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Sequencer/MovieSceneControlRigParameterSection.h:138-151, 342-344`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Sequencer/MovieSceneControlRigParameterBuffer.cpp:987-994`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Rigs/RigHierarchy.cpp:4187-4247`; `zebra_audition.uasset name table: 'EMovieSceneControlRigSpaceType::World', 'EMovieSceneControlRigSpaceType::ControlRig', 'Arm R/FK 0 Body Orient Space', 'Neck/End FK Global Orient Space', 'SpaceControlNameAndChannel'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:919 (CONTROL Arm R/FK 0 parents=['NULL:Arm R/Default FK Space','NULL:Arm R/FK 0 spine_05 Orient Space'] weights=...), :928-930`

#### UE1-orient-spaces-multiparent — Orient spaces and weighted multi-parent default FK space

*important* · assets: MR_Zebra, MR_FN_Biped, MR_Monster

'Orient Spaces' / 'FK Spaces' connectors create '<Ctrl> <Target> Orient Space' nulls, sibling to the control's default space null. They are exposed as AvailableSpaces labeled '<Target> Orient', e.g. 'Arm L/FK 0 spine_05 Orient Space', 'Local Orient Space' and 'Body Orient Space'. They switch orientation only.

When the module config 'Default FK Space Index' is at least 0, the FK root control is given multiple parents with weights: Arm L/FK 0 has parents ['Arm L/Default FK Space' (w=0), 'Arm L/FK 0 spine_05 Orient Space' (w=1)], making the first listed orient space the active default. Legs use index -1 and stay on 'Default FK Space' (under pelvis).

- Spine: Start/End FK have Global/Local/Body Orient; End IK and Start IK too.
- Neck: adds spine_05 Orient.
- Clavicle: Body Orient.
- Biped finger chains: 'hand_l Orient'. The Zebra finger chains have Orient Spaces unconnected and no orient null.

**Setup.** Space menu entries end with ' Orient'. Arm FK defaults to spine_05 orientation.

**Operators:** `RigUnit_AddAvailableSpaces`, `FRigMultiParentElement parent weights (Location/Rotation/Scale)`

**Scale:** Zebra: 2 weighted multi-parent controls (Arm L/R FK 0).

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:855 "CONTROL Arm L/FK 0 parents=['NULL:Arm L/Default FK Space', 'NULL:Arm L/FK 0 spine_05 Orient Space'] weights=(Location=0.000000,Rotation=0.000000,Scale=0.000000),(Location=1.000000,Rotation=1.000000,Scale=1.000000)" and AvailableSpaces labels 'spine_05 Orient','Local Orient','Body Orient'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/asset.t3d: Arm L 'Default FK Space Index=0'`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/runtime_hierarchy.txt:825 'NULL Index L/FK 0 hand_l Orient Space'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1404 'CONTROL Neck/Start FK' labels 'Global Orient','Local Orient','Body Orient','spine_05 Orient'`

#### UE2-spine-orient-spaces — Spine start/end orient spaces for IK and FK

*important* · assets: CRM_FN_Spine

Construction, when the 'Start Orient Spaces' array connector is connected:
- Start IK Space Nulls = ConstructSpaceNulls(Start IK, spaces, parent = Start FK, OrientOnly)
- Start FK Space Nulls = ConstructSpaceNulls(Start FK, spaces, parent = grandparent of Start FK, OrientOnly)

For End (when 'End Orient Spaces' is connected):
- End IK Space Nulls: parent = End Parent (= End FK)
- End FK Space Nulls: parent = grandparent of End FK, i.e. End IK Null

Forwards evaluates all four sets every frame (orient-only). Backwards evaluates Start FK, then End FK (inside the FK loop at the last control), then the Start and End IK sets.

**Setup.** Spine orient targets: [Root/Global, Root/Local, Body/Body]. Neck adds spine_05. Labels: 'Global Orient', 'Local Orient', 'Body Orient', 'spine_05 Orient'.

**Operators:** `RigUnit_ResolveArrayConnector`, `RigUnit_HierarchyGetParent`, `DISPATCH_RigVMDispatch_ArrayAppend`

**Scale:** Spine: 12 orient nulls. Neck: 16.

**Evidence:** `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt (ConstructionGraph) 'Constuct Space Nulls v01_2 ... Driven Control=$Start Control'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt 'NULL Spine/End FK Global Orient Space parents=['NULL:Spine/End IK Null']'`

#### UE2-spine-pelvis-txy-root-space — Pelvis TXY null injected as a space on the Root control

*important* · assets: CRM_FN_Spine, CRM_FN_Root

When Is Neck is false:

Construction:
- Spawn null 'Pelvis TXY' with no parent at identity.
- AddAvailableSpaces(Control = Root module metadata 'Root Control', i.e. root/Root, Spaces = [Pelvis TXY]).

Forwards step E: Pelvis TXY global translation X,Y = pelvis bone global X,Y; Z = 0 and rotation = identity (other fields keep their defaults).

The animator can space-switch root/Root to follow the pelvis ground projection. This is a cross-module space injection that uses metadata.

**Setup.** The space appears on root/Root without a label.

**Operators:** `RigUnit_HierarchyAddNull`, `DISPATCH_RigDispatch_GetModuleMetadata`, `RigUnit_AddAvailableSpaces`, `RigUnit_SetTransform`

**Scale:** 1 per rig

**Evidence:** `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:675 'Name=Pelvis TXY'`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:673 'Get Module Item Metadata' (Name=Root Control; NameSpace=Root)`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt 'root/Root ... (Key=(Type=Null,Name="Spine/Pelvis TXY"))'`

#### UE4-get-array-parents — Get Array Parents

*important* · assets: CRFL_Hierarchy_v001, CRM_FN_LimbTwist, CRM_FN_FkArray

Input: Items. Output: Parents. For each item it returns the default (first) parent from HierarchyGetParent(bDefaultParent=true), which caches and uses GetFirstParent, into an array parallel to Items. The active multi-parent is not used. LimbTwist uses this to drive the offset/parent nulls of twist controls rather than the controls themselves.

**Operators:** `RigUnit_HierarchyGetParent (bDefaultParent=true)`, `DISPATCH_RigVMDispatch_ArrayReset/ArrayAdd/ArrayIterator`

**Scale:** 3 call sites (LimbTwist forward and backward, FkArray construction).

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:651-676 'Get Array Parents'`; `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Execution/RigUnit_Hierarchy.cpp:10-42 (bDefaultParent -> GetFirstParent; else GetActiveParent)`; `ue/<dump>/FortniteRigs__Modules__Biped__LimbTwist__CRM_FN_LimbTwist/graphs.txt:318 'Get Array Parents.Parents -> Blend Twist_0.Drivens' (Backwards Solve)`

#### UE2-fkchain-default-orient-space — Default orient space selection at construction

*nice-to-have* · assets: CRM_FN_FkChain

After the orient space nulls are constructed: if 'Default Orient Space Index' != -1 and index < Num(Orient Spaces), call SwitchParent(Child = Controls[0], Parent = OrientSpaceNulls[index], Mode = ParentItem, bMaintainGlobal = true). This sets the first FK control's initial active space.

**Setup.** Public 'Default Orient Space Index' (int32, default -1).

**Operators:** `RigUnit_SwitchParent`, `DISPATCH_RigVMDispatch_CoreNotEquals`, `RigVMFunction_MathIntLess`

**Scale:** FkChain only

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkChain/graphs.txt:107 'RigUnit_SwitchParent | Mode=ParentItem; bMaintainGlobal=True'`; `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkChain/graphs.txt:177 'Not Equals | B=-1'`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/Execution/RigUnit_DynamicHierarchy.h:162-210`

#### UE2-prop-change-pivot — Change Pivot (hold child in world while re-positioning parent)

*nice-to-have* · assets: CRM_FN_Prop

'Change Pivot' bool channels exist on Prop Global and on Prop Local. Forwards, for the Global level (the Local level works the same):
1. When the channel differs from 'Prop Global Pivot Previous':
- turning on: store Local Control World Transform = Prop Local global; set Prop Local, Prop, Aim, Attach 01 and Attach 02 to grey (0.5); set Prop Global to red;
- turning off: SendEvent RequestAutoKey on Prop Local; Reset Controls Color (restores the colors stored in metadata 'Color').
2. Previous = current.
3. While the channel is on: Prop Local global = the stored transform. The child is pinned in world space while the animator moves the parent, which relocates the pivot.

The Local level pins Prop (stores Prop Control World Transform), greys Prop Global, Prop, Aim and the attach controls, and sets Prop Local red.

Step E: while either pivot is on, force Prop Global, Prop, Aim and Prop Local visible.

**Setup.** Channels 'Change Pivot' (bool, default false) under Prop Global and Prop Local (runtime names 'Change Pivot', 'Change Pivot_2').

**Operators:** `RigUnit_GetBoolAnimationChannelFromItem`, `DISPATCH_RigVMDispatch_CoreNotEquals`, `RigUnit_SetTransform`, `RigUnit_SetControlColor`, `RigUnit_GetControlColor`, `DISPATCH_RigDispatch_SetMetadata`, `DISPATCH_RigDispatch_GetMetadata`, `RigUnit_SendEvent`, `RigUnit_SetControlVisibility`

**Scale:** 2 per Prop

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Prop__CRM_FN_Prop/graphs.txt:2-303 Forwards (Sequence B/C/E)`; `<dump>/FortniteRigs__Modules__FkSolves__Prop__CRM_FN_Prop/graphs.txt:347,349 'Name=Change Pivot'`; `<dump>/FortniteRigs__Modules__FkSolves__Prop__CRM_FN_Prop/graphs.txt:367 'Name=Color; NameSpace=Self'`; `<dump>/FortniteRigs__Modules__FkSolves__Prop__CRM_FN_Prop/graphs.txt:707-731 'Reset Controls Color'`

### D5 FK

#### UE2-body-control-stack — Body Orbit / Body / Body Offset COG controls

*core* · assets: CRM_FN_Body

Construction:

1. 'Body Orbit' is created under the Parent connector (runtime: root/Local). Its offset (global) = Body Offset Transform * P, where P is:
- if both Left Hip and Right Hip connectors are connected: identity rotation with translation = lerp(LeftHip.initialT, RightHip.initialT, 0.5);
- otherwise, if Body is connected: the Body bone's initial global transform;
- otherwise identity.
2. 'Body Orbit Vis' bool (default false) is spawned under Body Orbit.
3. 'Body' control is created under Body Orbit with identity local offset. The 'Body Driver' variable is set to Body.
4. Metadata 'Body Delta Transform' is written on Body = MakeRelative(BodyBone_init, BodyCtrl_init), i.e. BodyBone_init * inverse(BodyCtrl_init).
5. If 'Create Body Offset Control' is true: 'Body Offset' is created under Body, and Body Driver becomes Body Offset. A 'Body Offset Vis' bool (default true) is spawned under it, hosted on Body.

**Setup.** Shapes:
- Body: 'Body Control Shape' (Hexagon_Thin; color override resolved via Get Control Color From Metadata).
- Body Orbit: the same shape scaled 1.2x.
- Body Offset: the Body shape scaled 0.85x, color * 0.5.

Connections in the templates: Body/Body -> pelvis, Left Hip -> thigh_l, Right Hip -> thigh_r.

**Operators:** `RigUnit_HierarchyAddControlTransform`, `RigUnit_ResolveConnector`, `RigVMFunction_MathTransformLerp`, `RigVMFunction_MathTransformMakeAbsolute`, `RigVMFunction_MathTransformMakeRelative`, `DISPATCH_RigDispatch_SetMetadata`, `RigUnit_HierarchyGetShapeSettings`

**Scale:** 1 per biped rig (3 transform controls)

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Body__CRM_FN_Body/graphs.txt:201,247,315 'Name=Body' 'Name=Body Orbit' 'Name=Body Offset'`; `<dump>/FortniteRigs__Modules__FkSolves__Body__CRM_FN_Body/graphs.txt:230 'Interpolate ... T=0.500000' (hip midpoint)`; `<dump>/FortniteRigs__Modules__FkSolves__Body__CRM_FN_Body/graphs.txt:231 'Name=Body Delta Transform'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt 'CONTROL Body/Body Orbit parents=['CONTROL:root/Local']'`

#### UE2-fk-drive-bones — FK drive bones (with offset compensation and Use Active Skeleton additive mode)

*core* · assets: CRM_FN_FkChain, CRM_FN_FkArray

Forwards, when Drive Bones is true, for each bone i:
- If Use Active Skeleton is true: Null[i] global = bone[i] current global (the incoming animated pose), and T = control[i] local * Null[i] global. The control acts as an additive local offset on top of the existing animation.
- Otherwise: T = control[i] global.

Then bone[i] global = inverse(Control Transform Offset) * T, with bPropagateToChildren = NOT UseActiveSkeleton.

FkArray runs the same code. It is ordered after its visibility and space-null steps.

**Setup.** Public config: Drive Bones = true, Use Active Skeleton = false. Private: Control Transform Offset = identity.

**Operators:** `RigUnit_GetTransform`, `RigUnit_SetTransform`, `RigVMFunction_MathTransformMakeAbsolute`, `RigVMFunction_MathTransformInverse`, `RigVMFunction_MathBoolNot`, `DISPATCH_RigVMDispatch_ArrayIterator`, `DISPATCH_RigVMDispatch_ArrayGetAtIndex`

**Scale:** All FkChain and FkArray instances

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkChain/graphs.txt:2-90 Forwards`; `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkChain/graphs.txt:31 'When using Active Skeleton we copy the transform to a Null and use the local transform of the Control'`; `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkArray/graphs.txt:2-112 Forwards`

#### UE2-fkarray-construction — FK array construction (independent per-bone FK with parent overrides)

*core* · assets: CRM_FN_FkArray, CRFL_Hierarchy_v001

Construction:

1. For each Bones connector target: if it exists, add it to Bones; otherwise record its index in 'Indices to Remove'. This lets the same template work on meshes that lack some bones.
2. Fk Parents = Get Array Parents(Bones), the hierarchy parent of each bone.
3. For each Override Parents[i]: if it exists and i is not in Indices to Remove, Fk Parents[i] = Override Parents[i].
4. Spaces and Orient Spaces are filtered to existing items. Display Names and Controls Visibility Initials are filtered by Indices to Remove, giving the '...Sorted' arrays.
5. For each bone j: spawn null '<Conform(bone)> Default Space' under FkParents[j] at the bone's initial global transform, then set FkParents[j] = that null.
6. For each bone i:
- null <name> under FkParents[i]: global = Control Transform Offset * bone initial
- control <name> under that null: rotation order from config, display name from the sorted list, shape via Get Control Shape
- set FkParents[i] = the control; write mirror metadata; append to Controls
- spawn bool 'Visibility' under the control. Its initial value is Sorted[i] when Num != 0 and Num >= i, otherwise true. Append it to Vis Channels.

**Setup.** Connections:
- MR_Zebra: Ear Base L bones=[ear_base_l], Override Parents=[skull_tp]; Ear L bones=[ear_01_l, ear_02_l]; Mohawk bones=[mohawk_bk, mohawk_fr] with Parent=skull_tp.
- MR_FN_Biped: Attach bones=[attach, attach_cape, attach_backpack, weapon_l, hand_attach_l, weapon_r, hand_attach_r], Override Parents=[root/Local, spine_05, spine_05], Spaces=[Prop/Prop].

Private 'FK Control Shape' is Default (white).

**Operators:** `RigUnit_ResolveArrayConnector`, `RigUnit_ItemExists`, `RigUnit_HierarchyGetParent`, `DISPATCH_RigVMDispatch_ArraySetAtIndex`, `DISPATCH_RigVMDispatch_ArrayFind`, `RigUnit_HierarchyAddNull`, `RigUnit_HierarchyAddControlTransform`, `RigUnit_HierarchyAddAnimationChannelBool`

**Scale:** 6 instances in MR_FN_Biped, 12 in MR_Zebra; 1-7 controls each

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkArray/graphs.txt:113-608 Construction`; `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkArray/graphs.txt:172 'B= Default Space'`; `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkArray/graphs.txt:250 'Name=Visibility'`; `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkArray/graphs.txt 'Making sure all the bones exists, different SKM can be used with MR Biped'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt 'NULL Ear L/Ear 02 L Default Space parents=['BONE:ear_01_l']'`

#### UE2-fkchain-construction — FK chain construction (null + control per bone)

*core* · assets: CRM_FN_FkChain

Construction:

1. Bones: if the End connector is connected, Bones = GetChainItemArray(Start..End, inclusive); otherwise Bones = [Start].
2. Parents: 'Parrent' and 'Fk Parent' are both set to the literal connector key 'Parent', which resolves to its target. Orient Spaces = ResolveArrayConnector('Orient Spaces').
3. For each bone i (Count n, Ratio r = i/(n-1)):
- name = Get Item Name v01(bone, i, n, Prefix = ControlNamesAsFK ? 'FK' : '', Suffix = ControlNamesAsFK ? '' : 'FK', StandardNumerical = ControlNamesAsFK, RemoveSideSuffix = true, start index 0).
- null <name>: parent = Fk Parent; global transform = Control Transform Offset * bone initial global.
- control <name>: parent = that null, identity local offset.
- Fk Parent = this control, so the next bone's null nests under it.
- The control is appended to Controls and gets mirror metadata.

Result hierarchy: Parent > FK 0 null > FK 0 > FK 1 null > FK 1 ... (runtime Thumb L: 'FK 0/1/2'; Clavicle L: 'FK').

**Setup.** Public config:
- Control Names As FK = true
- Rotation Order = XYZ (sets PreferredRotationOrder, bUsePreferredRotationOrder = true)
- Display Names[] (per index)
- Mirror Behavior = false
- Mirror Axis = (0,1,1)
- Default Orient Space Index = -1

Private: Shape Settings = Circle_Thin with shape rotation (0,-0.707,0,0.707).

Connections: Start -> first bone, End -> last bone, Parent -> parent bone (e.g. hand_l). Runtime display names come from config, e.g. 'Base', 'Mid', 'Tip'.

**Operators:** `RigUnit_HierarchyGetChainItemArray`, `RigUnit_ResolveConnector`, `RigUnit_ResolveArrayConnector`, `RigUnit_HierarchyAddNull`, `RigUnit_HierarchyAddControlTransform`, `RigVMFunction_MathTransformMakeAbsolute`, `DISPATCH_RigVMDispatch_ArrayAdd`

**Scale:** 24 instances in MR_FN_Biped, 16 in MR_Zebra, 4 in MR_Monster; 1-3 controls each

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkChain/graphs.txt:98 'RigUnit_HierarchyGetChainItemArray ... Start=(Type=Connector,Name="Start"); End=(Type=Connector,Name="End")'`; `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkChain/graphs.txt:144 'Get Item Name v01 | Remove Side Suffix=true'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt 'NULL Thumb L/FK 1 parents=['CONTROL:Thumb L/FK 0']'`

#### UE2-root-drive-root-bone — Root bone follows the Root control

*core* · assets: CRM_FN_Root

Forwards Solve, sequence step B: the global transform of the bone resolved from the RootJoint connector is set to the current global transform of the Root control (bPropagateToChildren=true).

**Setup.** RootJoint connector -> bone 'root'.

**Operators:** `RigUnit_GetTransform`, `RigUnit_SetTransform`, `RigUnit_ResolveConnector`

**Scale:** 1 per rig

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Root__CRM_FN_Root/graphs.txt:36-41 'VariableNode_1.Value -> Set Transform.Item'`; `<dump>/FortniteRigs__Modules__FkSolves__Root__CRM_FN_Root/graphs.txt:77 'Connector=(Type=Connector,Name="RootJoint")'`

#### UE3-compute-fk — Compute FK: aim-chain rebuild with optional scale stretch

*core* · assets: CRM_FN_IkFk2Bones

Compute FK(Transforms T[0..2], Length L[0..1], Negative Side, Apply Scale). Output = copy of T; for each i < len(L), Output[i] = FKBone(T[i], T[i].t, T[i+1].t, L[i]). T[2] passes through unchanged.

FKBone steps:
1. d = target - root; x = unit(NegSide ? -d : d).
2. From root matrix axes Yr, Zr: y = unit(cross(Zr, x)).
3. With Apply Scale: X = x*(|d|/L), Y = y*|Yr|, Z = unit(cross(X, Y))*|Zr|. Without it: X = x, Y = y, Z = unit(cross(x, y)).
4. Build the matrix with origin = root.

The bone therefore aims at the next joint while keeping its twist from its own Z axis.

FK mode: Transforms = global transforms of FK Drivers; Length = Default Lengths; Apply Scale = Use Scale. Each result goes to VB[i] global as (rot * inverse(FK Rotation Offset), t, s).

IK and FK modes (after the solve): Transforms = [VB0, Mid Control global, VB2] → Set Transform Array on the Virtual Bones (see Mid control).

**Setup.** FK mode stretch is expressed through the segment-scale nulls; Use Scale (default False) also scales the bones along X.

**Operators:** `RigVMFunction_MathMatrixFromTransformV2`, `RigVMFunction_MathMatrixToVectors`, `RigVMFunction_MathMatrixFromVectors`, `RigVMFunction_MathMatrixToTransform`, `RigVMFunction_MathVectorCross`, `RigVMFunction_MathVectorUnit`, `RigUnit_GetTransformItemArray`, `RigUnit_SetTransformItemArray`

**Scale:** 1-2 calls per limb per frame

**Evidence:** `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:2978-3103 'Compute FK' + 'FK Bone.ContainedGraph'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:164,429-434,365-373 Compute FK_1 -> For_Each_1 -> Set Transform_8 (rot * Inverse(FK Rotation Offset))`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:420-428 Compute FK (Mid) -> SetTransformItemArray_2`

#### UE3-ikfk-fk-chain — FK chain with gimbals, segment-scale nulls, scale profile and naming

*core* · assets: CRM_FN_IkFk2Bones, CRFL_Module_v001, CRFL_Control_v001

Construction first spawns the null 'Default FK Space' under Parent at Bones[0]'s initial global transform. Then, for each virtual bone i of Count:
- If i > 0, spawn a null named ['','Upper Segment Scale FK','Lower Segment Scale FK'][i] under the current FK parent. Its rotation is the FK parent's initial rotation; its translation and scale come from VB[i].
- Spawn the transform control 'FK <i>' (Get Item Name v01, which yields 'FK 0/1/2'):
  - offset = (VB[i].rot * FK Rotation Offset, VB[i].t, VB[i].s), global space
  - preferred rotation order = Rotation Order
  - display name = FK Start/Mid/End Display Name
  - shape name from 'fk-layer' DMC lookup (default FK Control Shape.Name), color from metadata
  - shape transform = (NegSide ? FKShape.Transform * Scale(-1,1,1) : FKShape.Transform); rotation *= FK Rotation Offset; scale *= FK Control Scale Factor Profile(i/(Count-1))
- Set metadata 'Mirror Behavioral'.
- If i != 1: Create Gimbal Control, and the gimbal becomes the next FK parent and an FK Driver. For i == 1 the FK control itself is the driver.

Result for the biped: FK Drivers = [FK 0 Gimbal, FK 1, FK 2 Gimbal]. Hierarchy: Default FK Space > FK 0 > FK 0 Gimbal > Upper Segment Scale FK > FK 1 > Lower Segment Scale FK > FK 2 > FK 2 Gimbal.

**Setup.** FK 0/1/2 are EULER_TRANSFORM controls with Circle_Thick shapes, display names like 'UpperArm FK', rotation order XYZ (arms can bind it). FK 0 and FK 2 each have a ROTATOR gimbal child '<ctrl> Gimbal'.

**Operators:** `RigUnit_HierarchyAddNull`, `RigUnit_HierarchyAddControlTransform`, `RigVMFunction_AnimEvalRichCurve`, `RigVMFunction_MathTransformMul`, `FUNC Get Item Name v01`, `FUNC Create Gimbal Control`, `FUNC Get Control Shape Name From Item v02`, `FUNC Set Mirror Behavior`

**Scale:** 3 FK + 2 gimbal controls + 2 scale nulls per limb

**Evidence:** `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:818 'Spawn Null | Name=Default FK Space'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:959 'Make Array | Values=("None","Upper Segment Scale FK","Lower Segment Scale FK")'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:895 'Get Item Name v01 | Standard Numerical Names=true; Prefix=FK'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:1290,1297,1434 Not_Equals_1(B=1) -> Create Gimbal Control`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/runtime_hierarchy.txt 'CONTROL Arm L/FK 0 Gimbal' / 'NULL Arm L/Upper Segment Scale FK'`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/asset.t3d 'FK Drivers(0)=(Type=Control,Name="Leg L/FK 0 Gimbal")'`

#### UE6-control-drives-bone — Control-to-bone drive via ProjectTransformToNewParent + SetTransform

*core* · assets: CRM_Zebra_Face, CRM_Monster_Face

For each (bone, control) pair the forward solve computes T = ProjectTransformToNewParent(Child=bone initial global, OldParent=control initial global, NewParent=control current global), i.e. bone_init * inv(ctrl_init) * ctrl_cur. It then applies SetTransform(bone, GlobalSpace, weight 1, bPropagateToChildren=True). Pairs, in execution order: jaw<-jaw ctrl; skull<-Reverse Jaw; skull_tp<-Skull Tp; teeth_tp<-Teeth Tp; teeth_bt<-Teeth Bt; cheek_l/r<-Cheek L/R; nose<-Nose; tongue_01..04<-tongue_01..04 (loop); eye_l<-Eye L; eye_main_l<-Eye Main L; eye_r<-Eye R; eye_main_r<-Eye Main R. A separate unexecuted copy (Set Transform_10) targets null 'Teeth Tp' from bone Skull.

**Setup.** Each control's offset equals its bone's global rest transform (OffsetSpace Global, OffsetTransform linked from GetTransform of the bone), so the control value is zero at rest.

**Operators:** `RigUnit_ProjectTransformToNewParent`, `RigUnit_SetTransform`, `RigVMDispatch_ArrayIterator`

**Scale:** 16 ProjectTransformToNewParent / 17 SetTransform per face

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:628 ProjectTransformToNewParent Child=jaw bChildInitial=True`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:709 ProjectTransformToNewParent_6 Child=teeth_tp`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Hierarchy/RigUnit_ProjectTransformToNewParent.cpp (MakeRelative/MakeAbsolute)`

#### UE1-gimbal-controls — Gimbal child controls with element-key metadata

*important* · assets: MR_Zebra, MR_FN_Biped

IkFk2Bones adds a ROTATOR child control '<X> Gimbal' under FK 0, FK 2 and IK (display '<Name> Gimbal'). These are 14 rotators in Zebra: 3 per limb plus the 2 Arm 'IK Rotation' controls. The parent control carries metadata '<Module>/Gimbal Control' (RIG_ELEMENT_KEY) pointing to its gimbal. Gimbal shapes are shown via the 'Gimbal Control Vis' channel. The IK gimbal uses 'DefaultGizmoLibraryNormalized.Sphere_Thin'. Downstream nulls hang under the gimbal ('Upper Segment Scale FK' under FK 0 Gimbal; 'IK' null under IK Gimbal).

**Setup.** Circle_Thick shape for FK gimbals and Sphere_Thin for the IK gimbal. Rotation-only, no limits.

**Operators:** `RigUnit_HierarchyAddControlRotator`, `DISPATCH_RigDispatch_SetMetadata`

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:856 "CONTROL Arm L/FK 0 Gimbal parents=['CONTROL:Arm L/FK 0'] ... control_type=ROTATOR"`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:855 "'Arm L/Gimbal Control:RIG_ELEMENT_KEY'"`

#### UE1-layout-clavicle — Clavicle FK chain layout

*important* · assets: MR_Zebra, MR_Monster, MR_FN_Biped

A CRM_FN_FkChain with Start=clavicle_l and End unconnected produces one control 'Clavicle L/FK'. Display 'Clavicle', shape Default offset (12, 0, 8) at scale 0.25, rotation order XZY, Lock Scale. It is parented to null 'Clavicle L/FK' under bone spine_05 (connector Parent). Space: Body Orient ('Clavicle L/FK Body Orient Space' under spine_05). Carries mirror metadata.

**Setup.** 1 control per side.

**Operators:** `CRM_FN_FkChain`

**Scale:** 2 modules in each of Zebra, Monster and Biped.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1388 "CONTROL Clavicle L/FK parents=['NULL:Clavicle L/FK'] ... display_name=Clavicle ... preferred_rotation_order=XZY"`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/asset.t3d: Clavicle L 'Display Names(0)="Clavicle"'`

#### UE1-layout-fingers — Finger FK chain layout (Zebra 3-finger hand vs Biped 5-finger with metacarpals)

*important* · assets: MR_Zebra, MR_FN_Biped

CRM_FN_FkChain per finger produces controls FK 0/1/2 with display names Base/Mid/Tip. Shape Circle_Pins_Thick with Control Transform Offset quat (-1, 0, 0, 0). Each control sits under a same-named null, and the chain nests (FK 1 null under FK 0 control).

Zebra has Thumb (thumb_01..03), Index (index_metacarpal..index_02) and Pinky (pinky_metacarpal..pinky_02) per side, all with Parent = hand bone. The FK 0 null is parented directly to hand_l. Orient Spaces are unconnected.

The Biped template has Thumb plus Index/Middle/Ring/Pinky (01..03) parented to the metacarpal bones. It adds the Meta L/R FkArray ('Index/Middle/Ring/Pinky Meta', Sphere_Solid, one Visibility channel each) and the 'hand_l Orient' space on every finger.

**Setup.** Zebra: 3 controls per finger, 18 total. Biped: 30 finger + 8 meta controls.

**Operators:** `CRM_FN_FkChain`, `CRM_FN_FkArray`

**Scale:** Zebra: 6 finger modules. Biped: 10 finger + 2 meta modules.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:822 'CONTROL Thumb L/FK 0 ... shape_name=Circle_Pins_Thick display_name=Base'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/modular_rig_model.txt: 'Index L/Start' -> 'index_metacarpal_l', 'Index L/End' -> 'index_02_l'`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/runtime_hierarchy.txt:875 'CONTROL Meta L/Index Metacarpal L ... display_name=Index Meta'`

#### UE1-layout-fkarray-extras — FkArray modules: tweakers, ears, mohawk (per-bone FK with default-space nulls)

*important* · assets: MR_Zebra, MR_FN_Biped

CRM_FN_FkArray creates the following for each bone in 'Bones':
- a null '<Bone Display> Default Space', parented to Override Parents[i] if given, otherwise to the bone's skeletal parent
- an offset null '<Bone Display>'
- a control (name conformed from the bone: 'Def Thigh In L', 'Ear 01 L', 'Mohawk Bk'), with mirror metadata
- a 'Visibility' bool channel

Zebra instances:
- Tweakers: def_thigh_in_l/r under pelvis; Control Scale 0.4; shape Default.
- Ear Base L/R: ear_base_x with override parent skull_tp; shape Default offset (+/-16, 0, +/-7) at 0.3.
- Ear L/R: ear_01_x under ear_base_x, ear_02_x under ear_01_x; shape DefaultGizmoLibraryNormalized.Circle_Thick.
- Mohawk: mohawk_bk and mohawk_fr under skull_tp; Circle_Thick at scale (1.25, 0.5, 1).

The chain relationship goes through the driven bones (ear_02's default space follows bone ear_01), not through control parenting. The Biped's Attach module is the same class with 7 bones and display names Attach/Cape/Backpack/Weapon Attach L/Hand Attach L/Weapon Attach R/Hand Attach R, plus 'Prop Space' nulls from Spaces -> Prop/Prop.

**Setup.** Tweakers 4 (2 controls + 2 vis). Ear Base 2 each. Ear 4 each. Mohawk 4. Biped Attach 14.

**Operators:** `CRM_FN_FkArray`, `FUNC Conform Name v01 @ CRFL_Module_v001`

**Scale:** Zebra: 6 FkArray modules (16 controls). Biped: 3 (Meta L/R and Attach).

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1345 "NULL Ear Base L/Ear Base L Default Space parents=['BONE:skull_tp']"`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1537 'CONTROL Tweakers/Def Thigh In L'`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/runtime_hierarchy.txt:1695 'CONTROL Attach/Attach'`

#### UE2-prop-control-stack — Prop control hierarchy and spaces (no bone output)

*important* · assets: CRM_FN_Prop

Construction.

Hierarchy:
- 'Prop Global' under the Parent connector
- 'Prop Local' under Prop Global
- 'Prop' under Prop Local
- 'Prop Attach 01' and 'Prop Attach 02' under Prop

All are identity offsets, rotation order = 'Rotation Order' (YXZ).

Other elements:
- 'Aim' control under Parent. Its offset = Parent's current global transform, with translation moved to Parent.TransformLocation((0,40,0)), i.e. 40 units along the parent's +Y.
- Prop gets every 'Spaces' connector target as an available space.
- Prop Attach 02 gets Prop Attach 01 as an available space.

The module drives no bones. Other modules consume its controls as spaces, e.g. Arm IK Spaces include Prop/Prop, Prop Attach 01 and 02; FkArray Attach Spaces includes Prop/Prop.

**Setup.** Shapes:
- Prop Global, Prop Local, Prop: RoundedTrapeze_ArrowTip_Thick; Prop Local and Prop use Square_Thick overrides (color (1,0.5,0) / (1,0.2,0), scale (2,2,1), rotated -90 degrees about Y).
- Aim: Arrow_Solid, green, scale (0.8,0.8,5).
- Attach 01: Box_Thick, cyan, scale 0.5. Attach 02: Box_Thick, blue, scale 0.4.

Connections: Prop/Parent -> root/Local; Spaces = [hand_r, hand_l, spine_05].

**Operators:** `RigUnit_ResolveConnector`, `RigUnit_ResolveArrayConnector`, `RigUnit_HierarchyAddControlTransform`, `RigUnit_AddAvailableSpaces`, `RigVMFunction_MathTransformTransformVector`

**Scale:** 1 per rig; 6 controls

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Prop__CRM_FN_Prop/graphs.txt:320,336,341,351,404,405 control spawns`; `<dump>/FortniteRigs__Modules__FkSolves__Prop__CRM_FN_Prop/graphs.txt:353 'Location=(X=0.000000,Y=40.000000'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/modular_rig_model.txt 'Arm L/IK Spaces ... (Type=Control,Name="Prop/Prop"),(Type=Control,Name="Prop/Prop Attach 01")'`

#### UE3-gimbal-control — Gimbal control helper (Create Gimbal Control)

*important* · assets: CRFL_Control_v001, CRM_FN_IkFk2Bones

Create Gimbal Control(Source, Shape Settings Override, Rotation Order, Display Name) does the following:
1. Spawns a ROTATOR control '<Source> Gimbal' as a child of Source, with local identity offset, preferred rotation order, and display name = Display Name.
2. Shape = the override if its Name != None. Otherwise Source's shape with scale *0.8 and color +(0.2,0.2,0.2).
3. Sets item metadata 'Gimbal Control' on Source pointing to the gimbal.
4. Spawns the bool channel 'Gimbal Control Vis' (initial False) under the gimbal and adds Source as a channel host.

The IK gimbal uses override shape 'DefaultGizmoLibraryNormalized.Sphere_Thin' with the IK control's color +0.2.

Forward visibility:
- FK gimbal visible = !IKSolve && channel 'Gimbal Control Vis' on that gimbal.
- IK gimbal visible = IKSolve && its channel.

Match functions zero the gimbal locals.

**Setup.** Each gimbal has a bool channel 'Gimbal Control Vis' (display 'Gimbal') hosted on its source control.

**Operators:** `RigUnit_HierarchyAddControlRotator`, `RigUnit_HierarchyGetShapeSettings`, `DISPATCH_RigDispatch_SetMetadata`, `RigUnit_HierarchyAddAnimationChannelBool`, `RigUnit_SetChannelHosts`, `RigUnit_GetBoolAnimationChannel`, `RigUnit_SetControlVisibility`

**Scale:** 3 gimbals per limb

**Evidence:** `<dump>/FortniteRigs__Libraries__CRFL_Control_v001/graphs.txt:195-266 'Create Gimbal Control' / 'Name=Gimbal Control Vis'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:918 'Create Gimbal Control_1 | Shape Settings Override=(...Sphere_Thin'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:199 'From String | String=Gimbal Control Vis' + 470-473`

#### UE4-gimbal-control — Create Gimbal Control

*important* · assets: CRFL_Control_v001, CRM_FN_IkFk2Bones, MR_Zebra, MR_FN_Biped

Construction. Inputs: Source Control, Shape Settings Override (FRigUnit_HierarchyAddControl_ShapeSettings), Rotation Order (EEulerRotationOrder, default YZX) and Display Name. Outputs: Gimbal Control and Visibility Channel. Steps: (1) Spawn a Rotator control named '<Source.Name> Gimbal', parented to Source Control, with identity local offset (LocalSpace), initial value (0,0,0), no limits, bUsePreferredRotationOrder=true, PreferredRotationOrder=input and InitialSpace=LocalSpace. Its display name is '<Display Name> Gimbal', or None if Display Name is None. Its shape is the Override when Override.Name != None; otherwise it copies the Source's shape settings with Scale3D multiplied by 0.8 and Color + (0.2,0.2,0.2,1). (2) SetMetadata(Source, 'Gimbal Control', Self) = gimbal key. (3) Spawn a bool animation channel 'Gimbal Control Vis' under the gimbal (initial False, min False, max True). (4) SetChannelHosts(channel, [Source Control]) so the toggle also appears on the main control. Nothing in the library drives visibility; the module reads the channel.

**Setup.** A rotation-only child control ('X Gimbal') gives the animator a second rotation layer in a different Euler order. It is toggled by the bool channel 'Gimbal Control Vis', which is hosted on both the gimbal and the source control.

**Operators:** `RigUnit_HierarchyAddControlRotator`, `RigUnit_HierarchyGetShapeSettings`, `RigVMFunction_MathVectorMul`, `RigVMFunction_MathColorAdd`, `DISPATCH_RigDispatch_SetMetadata`, `RigUnit_HierarchyAddAnimationChannelBool`, `RigUnit_SetChannelHosts`, `RigVMFunction_NameConcat`, `DISPATCH_RigVMDispatch_If`

**Scale:** IkFk2Bones 2 calls per limb (FK and IK). About 4 gimbal pairs per biped (Arm/Leg L/R); the Zebra runtime has 54 'Gimbal' lines.

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Control_v001/graphs.txt:195-266 'Create Gimbal Control' (Concat 'B= Gimbal', Multiply_6 'B=(0.8,0.8,0.8)', Add_8 'B=(0.2,0.2,0.2,1)', SpawnAnimationChannel 'Name=Gimbal Control Vis')`; `ue/<dump>/FortniteRigs__Libraries__CRFL_Control_v001/regen.py:603 SpawnControl_2.Settings '(...bUsePreferredRotationOrder=true,PreferredRotationOrder=YZX...)'`; `ue/<dump>/FortniteRigs__Modules__IkSolves__IkFk2Bones__CRM_FN_IkFk2Bones/graphs.txt:917-918 two calls (second with Shape Settings Override Name='DefaultGizmoLibraryNormalized.Sphere_Thin')`; `ue/<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:856 'CONTROL Arm L/FK 0 Gimbal ... control_type=ROTATOR ... display_name=UpperArm FK Gimbal'; :857 'CONTROL Arm L/Gimbal Control Vis parents=[CONTROL:Arm L/FK 0 Gimbal]'`

#### UE6-tongue-fk — Tongue FK chain with controls parented under driven bones

*important* · assets: CRM_Zebra_Face, CRM_Monster_Face

Construction loops over bones [tongue_01..04] and spawns a control with the same name at each bone's global transform. The parents are [Jaw Control, bone tongue_01, bone tongue_02, bone tongue_03] by index, so controls 2-4 are children of the previous skeleton bone rather than the previous control. In the forward solve, each tongue bone i receives SetTransform(global) = ProjectTransformToNewParent(bone i initial, control i initial -> current), propagated. Because each bone is written before the next control is read, this gives a regular FK chain.

**Setup.** Sphere_Solid, yellow, scale 0.025, shape offset (0,0,1) (Monster (0,0,3)), shape rotated -90 degrees about Y. No limits.

**Operators:** `RigVMDispatch_ArrayIterator`, `RigVMDispatch_ArrayGetAtIndex`, `RigUnit_HierarchyAddControlTransform`, `RigUnit_ProjectTransformToNewParent`, `RigUnit_SetTransform`

**Scale:** 4 controls

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:66 SpawnControl_46 (loop), :70 ItemArray_1 (jaw,tongue_01,tongue_02,tongue_03)`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:760 Set Transform_9, :762 ItemArray_5`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:955 CONTROL Face/tongue_03 par=B:tongue_02`

#### UE4-create-fk-chain-controls — Create FK Chain Controls (library, unused)

*nice-to-have* · assets: CRFL_Module_v001

Inputs: Name, Items, Parent, Control Settings, Add Nul Above, Nul Suffix, Control Suffix and Orient to World. Output: Controls. It resets the list and sets runningParent = Parent. For each item i it calls ControlStackAtItem(Item, Parent=runningParent, Name = Name + 'FK ' + str(i+1), Settings, AddNull=AddNulAbove, suffixes, OrientToWorld, no bottom null, no secondary). The new control becomes runningParent and is appended. The result is a simple parented FK chain matched to the bones' initial transforms.

**Operators:** `FUNC Control Stack at Item`, `RigVMFunction_MathIntToName`, `RigVMFunction_NameConcat`, `DISPATCH_RigVMDispatch_ArrayIterator/ArrayAdd/ArrayReset`

**Scale:** No callers; the FK modules build their own chains.

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Module_v001/graphs.txt:289-359 'Create FK Chain Controls ' (Concat_1 'A=FK ')`

### D6 IK

#### UE1-layout-limb-ikfk — Arm/Leg IkFk2Bones module layout

*core* · assets: MR_Zebra, MR_FN_Biped

Per limb (Arm: 22 controls; Leg: 21):
- FK 0/1/2 (Circle_Thick): Arm 'UpperArm FK'/'LowerArm FK'/'Hand FK'; Leg 'UpperLeg FK'/'LowerLeg FK'/'Foot FK'. FK 1 hangs under null 'Upper Segment Scale FK' (under FK 0 Gimbal); FK 2 under 'Lower Segment Scale FK' (under FK 1).
- Gimbals: FK 0 Gimbal, FK 2 Gimbal, IK Gimbal (rotators).
- IK Base: Box_Thick, translate-only, under 'Parent Buffer' (under Parent bone).
- IK: Box_Thick, under root/Local, with 4 (Leg) or 9 (Arm) spaces.
- PV: POSITION, Diamond_Solid, under 'Orient PV' (under world null 'Auto PV'), with the IK spaces plus IK.
- Mid: HalfCircle_Solid, 'Knee'/'Elbow', under world null 'Mid'.
- Arm only: 'IK Rotation' (ROTATOR, Circle_Pins, under root/Local) and 'IK End Align' bool (hosted on IK Rotation).
- Channels on IK: Ik Fk Switch (bool, hosted on all limb controls), Stretch (bool true), Softness [0, 1], PV Twist Follow [0, 1] (hosted on PV), Upper/Lower Segment Scale [0.0001, 2], Sec Controls Vis (hosted on FK 2), 3 Gimbal Control Vis, and '<Limb> Vis' (under root/Global).

Helper elements: virtual bones A/B/C (under Parent bone); nulls for Default FK Space, orient spaces, IK/PV nulls per space, PV Twist Start/End Null, IK PV Null, IK Rotation Null.

Primary connector Start carries match/key metadata. Default IK=False on arms (so IK/PV shapes are hidden initially); legs default IK True per the class default but IK End Align False.

**Setup.** Arm L/R: 22 controls each. Leg L/R: 21 each. Transform controls per limb: 11 (Arm) or 10 (Leg), including 4 POSITION PVs in total across the 4 limbs.

**Operators:** `CRM_FN_IkFk2Bones`

**Scale:** 4 limb modules in Zebra and Biped.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:2115 "CONTROL Arm L/IK parents=['CONTROL:root/Local']"`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:2069 "CONTROL Leg L/PV parents=['NULL:Leg L/Orient PV '] ... control_type=POSITION ... shape_name=Diamond_Solid"`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:2128 'CONTROL Arm L/IK Rotation ... control_type=ROTATOR'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:2121 'CONTROL Arm L/Ik Fk Switch'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:2070 'CONTROL Leg L/Upper Segment Scale ... SCALE_FLOAT'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/asset.t3d: Arm L 'Default IK=False'; Leg L 'IK End Align=False'`

#### UE3-auto-pv-parent — Auto pole-vector parent with PV twist follow (Compute Pole Vector Parent)

*core* · assets: CRM_FN_IkFk2Bones

Compute Pole Vector Parent(Start, End, Initial, Lengths, PV Twist Follow w, NegSide):
1. P = lerp(Start.t, End.t, L0/sum(L)).
2. A = AimMath at P: primary X aims at End (location), secondary Z aims at End.TransformPosition((0,0,+/-1)) (location). This frame follows the end's twist.
3. B = AimMath at P: primary X aims at End, secondary Z along Start.TransformVector((0,0,+/-1)) (direction). This frame has no twist follow.
   (+/- = NegSide ? -1 : 1.)
4. M = TransformLerp(B, A, w).
5. Result = (0, NegSide ? +max(L0,L1) : -max(L0,L1), 0) * M, i.e. offset along local Y.

Construction:
- Null 'PV Twist End Null' under the IK gimbal (translation/scale of VB2, rotation of VB1).
- Result with Start = VB0, Initial = true, Lengths = Default Lengths gives the placement of root null 'Auto PV '.
- Child null 'Orient PV ' at the PV location.
- Position control 'PV' (bIsPosition, Diamond_Solid, scale .4) under it.
- 'PV Twist Start Null' under Parent at VB0.

Forward (IK mode):
- w = (PV Twist Follow var > 0) ? channel 'PV Twist Follow' : 0.
- Start = PV Twist Start Null, End = PV Twist End Null, Lengths = Scaled Lengths.
- Auto PV global = Result. The PV control rides along with the limb root-to-IK direction and optionally with IK twist.

The channel 'PV Twist Follow' (0..1, initial = var) exists only when the var > 0 (legs), hosted on the PV control.

**Setup.** 'PV' is a POSITION control (Diamond_Solid). Legs get a 'PV Twist Follow' float channel on IK, hosted on PV. A grey debug line is drawn Mid->PV unless DMC is active.

**Operators:** `RigUnit_AimBoneMath`, `RigVMFunction_MathVectorLerp`, `RigVMFunction_MathTransformLerp`, `RigVMFunction_MathTransformMakeAbsolute`, `RigVMFunction_MathDoubleArraySum`, `RigVMFunction_MathDoubleMax`, `RigUnit_HierarchyAddControlVector`, `RigUnit_HierarchyAddAnimationChannelFloat`

**Scale:** 1 per limb; twist follow on legs only

**Evidence:** `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:1972-2068 'Compute Pole Vector Parent.ContainedGraph'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:637-638,732,748,766 'Auto PV ' / 'Orient PV ' / 'PV Twist End Null' / 'PV Twist Start Null'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:612 'Spawn Vector Control | Name=PV'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:267-269,560-563 PV Twist Follow var>0 gate`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:729 'Name=PV Twist Follow; MinimumValue=0; MaximumValue=1'`

#### UE3-ik-plane-virtual-bones — IK-plane virtual bones (Create IK Plane Virtual Bones v02 + Project Middle Bone to IK Plane)

*core* · assets: CRM_FN_IkFk2Bones, CRFL_Hierarchy_v001, CRFL_Math_v001

Construction builds a clean, planar proxy chain of 3 real rig bones named '<Name>Bone A/B/C' (Name='Virtual ', parents A->parent(Start), B->A, C->B). The solve runs on these, and the skin bones follow them.

Projecting the mid joint (initial global transforms s, m, e):
- a = m.TransformPosition(SecondaryAxis) - m.t
- N = cross(a, e.t - s.t)
- P = IntersectPlane(start = m.t, dir = N, plane point = s.t, normal = N)
- m' = s.t + (P - s.t) * |m.t - s.t| / |P - s.t|, which keeps the upper length.

Building the bones (n = cross(m' - s, e - m')):
- A = AimMath at s: primary Axis aims at m' (Location), secondary Z aims along direction n.
- B = AimMath at m': primary Axis aims at e, secondary Z aims along n.
- C = AimMath at e: primary Axis aims at End.TransformPosition(Axis), secondary (0,-1,0) aims at End.TransformPosition((0,0,2)), so it keeps the End bone's own orientation.

Axis = module Primary Axis, SecondaryAxis = module Secondary Axis. Afterwards the construction sets Virtual Bone C's initial local rotation *= End Bone Rotation Offset (bInitial=true, no propagation).

**Setup.** Hidden helper bones named '<Module>/Virtual Bone A/B/C'.

**Operators:** `RigUnit_HierarchyAddBone`, `RigUnit_AimBoneMath`, `RigVMFunction_MathIntersectPlane`, `RigVMFunction_MathVectorCross`, `RigVMFunction_MathTransformArrayToSRT`, `RigUnit_SetRotation(Local,bInitial)`

**Scale:** 3 extra bones per limb (12 per biped)

**Evidence:** `<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:1421-1563 'Create IK Plane Virtual Bones v02'`; `<dump>/FortniteRigs__Libraries__CRFL_Math_v001/graphs.txt:298-439 'Project Middle Bone to IK Plane'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:677 'Create IK Plane Virtual Bones v02 | Name=Virtual '`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:844 'Set Rotation | Space=LocalSpace; bInitial=true' + 1319-1321 End Bone Rotation Offset`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/runtime_hierarchy.txt 'BONE Arm L/Virtual Bone A'`

#### UE3-pv-location — Pole vector location (Compute Pole Vector v01 / Location v02)

*core* · assets: CRFL_Hierarchy_v001, CRM_FN_IkFk2Bones

Compute Pole Vector v01(T[s,m,e], SecondaryAxis, ProjectToPlane):
1. a = m.TransformPosition(SecAxis) - m.t; N = cross(a, e - s).
2. m' = ProjectToPlane ? IntersectPlane(m, N, s, N) : m.
3. proj = s + (e - s) * dot(m' - s, e - s)/|e - s|^2.
4. PoleVector = m' - proj; Elbow = m'; UpperLen = |m' - s|.

Compute Pole Vector Location v02(Bones, SecAxis, PV Offset):
1. Compute cur = v01(current globals) and init = v01(initial globals), both unprojected.
2. Re-express the initial pole direction in the current mid frame: ref = mid_cur.TransformPosition((init + mid0.t) relative to mid0) - mid_cur.t.
3. dir = unit(cur) * UpperLen * PV Offset; negate it if dot(cur, ref) <= 0, which keeps the pole on the bind-pose side.
4. Result = Elbow + dir.

Usage: construction sets the 'Orient PV ' null translation to this. Match IK sets the PV control global translation from it, with PV Offset = PV Distance Scale.

**Setup.** PV Distance Scale is the pole distance as a multiple of the upper segment length.

**Operators:** `RigVMFunction_MathIntersectPlane`, `RigVMFunction_MathVectorDot`, `RigVMFunction_MathVectorLengthSquared`, `RigVMFunction_MathVectorSetLength`, `RigVMFunction_MathTransformMakeRelative`, `RigVMFunction_MathTransformTransformVector`, `RigVMFunction_VisualDebugVectorNoSpace`

**Scale:** construction plus every Match IK

**Evidence:** `<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:1703-1806 'Compute Pole Vector Location v02'`; `<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:1808-1898 'Compute Pole Vector v01'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:1052,1615-1618 construction use`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:2765,2811-2814 Match IK use`

#### UE3-soft-ik — Soft IK with stretch and per-segment scale (Soft IK local function)

*core* · assets: CRM_FN_IkFk2Bones

Inputs: Bone A/B/C = Virtual Bones 0/1/2; Effector = IK Null global; Softness = float channel 'Softness' on the IK control; EnableStretch = bool channel 'Stretch'; Upper/Lower Segment Scale; Softness Distance = 0.95.

Step 1, lengths and softness start:
- LA = |A0 - B0|, LB = |B0 - C0| using initial globals.
- L = LA*Us + LB*Ls, where Us/Ls are the upper/lower segment scales.
- d = |Effector.t - A.t| using current A.
- ds = Remap(Softness, 0..1 -> L..L*0.95), unclamped.

Step 2, soft distance:
- num = d - ds; den = L - ds (each replaced by 1 if nearly zero, tolerance 1e-3).
- f = 1 - exp(-num/den).
- d_soft = ds + den*f.
- If Softness <= 1e-4 or f <= 0: SoftLenA = LA*Us, SoftLenB = LB*Ls, SoftEffector = Effector.
- Otherwise: SoftLenA = LA*Us*(d/d_soft), SoftLenB = LB*Ls*(d/d_soft), SoftEffector.t = A.t + setLength(Effector.t - A.t, d_soft), keeping the effector's rotation and scale.

Step 3, solve with TwoBoneIKSimplePerItem:
- ItemA = A, ItemB = B, EffectorItem = C.
- Effector = (Stretch ? Effector : SoftEffector), with rotation = rot * inverse(IK Rotation Offset).
- ItemALength/ItemBLength = Stretch ? SoftLen : 0. Zero makes the unit use the initial lengths scaled by current/initial bone scale.
- PoleVector (0,0,0), Kind = Location, PoleVectorSpace = PV Control, so the pole is the PV control's world position.
- PrimaryAxis = (+/-1,0,0), SecondaryAxis = (0,-/+1,0), SecondaryAxisWeight = 1.
- bEnableStretch = Stretch, StretchStartRatio = 1.0, StretchMaximumRatio = 4.0, Weight = 1.
- The unit then writes C = the full effector transform.

Engine stretch: scaling = (Max - 1) * clamp((d/L - Start)/(Max - Start), 0, 1); both lengths are multiplied by (1 + scaling).

Debug draws arcs of radius ds (green) and L (red) at A, plus the soft effector axes.

**Setup.** On the IK control: 'Stretch' bool channel (initial true) and 'Softness' float channel (0..1, limits on, initial 0).

**Operators:** `RigUnit_TwoBoneIKSimplePerItem`, `RigVMFunction_MathDoubleExponential`, `RigVMFunction_MathDoubleRemap`, `RigVMFunction_MathVectorSetLength`, `RigVMFunction_MathDoubleIsNearlyZero`, `RigVMFunction_DebugArcNoSpace`, `RigUnit_GetFloatAnimationChannel`, `RigUnit_GetBoolAnimationChannel`

**Scale:** 1 solve per limb per frame

**Evidence:** `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:2247-2539 'Soft IK.New Function_ContainedGraph'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:31 'FUNC Soft IK ... PoleVectorKind=Location; StretchStartRatio=1.000000; StretchMaximumRatio=4.000000; ... Softness Distance=0.950000'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:32,37 'Get Float Channel | Channel=Softness' / 'Channel=Stretch'`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Highlevel/Hierarchy/RigUnit_TwoBoneIKSimple.cpp:111-220`; `<UE>/Source/Runtime/AnimationCore/Private/TwoBoneIK.cpp:127-140`

#### UE4-pv-location-v02 — Compute Pole Vector Location v02 (flip-safe PV placement)

*core* · assets: CRFL_Hierarchy_v001, CRM_FN_IkFk2Bones

Inputs: Bones[3] (the virtual bones), Secondary Axis, PV Offset (library default 10) and Debug. Output: Pole Vector Location. (1) PVi = ComputePoleVector(initial global transforms, SecAxis, ProjectToPlane=false).PoleVector. (2) {PVc, elbowC, upperC} = ComputePoleVector(current transforms, ...). (3) Reference: R = T1_cur.TransformLocation(MakeRelative({pos = PVi + P1_init}, T1_init).pos) - P1_cur, which carries the initial pole direction along with the current mid bone. (4) off = normalize(PVc) * (upperC * PVOffset). (5) If dot(PVc, R) <= 0, off = -off; this prevents a flip when the limb straightens or hyperextends. (6) Result = elbowC + off. When Debug is on it draws PVc (green) and R (blue) at the origin, scaled by PV Offset.

**Setup.** PV Offset is driven by the IkFk2Bones variable 'PV Distance Scale'. The result places the pole-vector control or null at construction and during IK matching.

**Operators:** `RigUnit_GetTransformItemArray (initial and current)`, `FUNC Compute Pole Vector v01 (x2)`, `RigVMFunction_MathTransformMakeRelative`, `RigVMFunction_MathTransformTransformVector`, `RigVMFunction_MathVectorSetLength`, `RigVMFunction_MathVectorScale`, `RigVMFunction_MathDoubleMul`, `RigVMFunction_MathVectorDot`, `RigVMFunction_MathDoubleLessEqual`, `RigVMFunction_MathVectorNegate`, `RigVMFunction_VisualDebugVectorNoSpace`, `DISPATCH_RigVMDispatch_If`

**Scale:** IkFk2Bones 2 calls (construction + Match IK).

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:1703-1806 'Compute Pole Vector Location v02' (comments 'Negate vector if computed to a flipped vector from the initial ref compute', 'Scale Projected mid bone > Mid Bone location vector based on Upper bone length * PV Offset')`; `ue/<dump>/FortniteRigs__Modules__IkSolves__IkFk2Bones__CRM_FN_IkFk2Bones/graphs.txt:1052 construction call -> HierarchyAddNull_2_1.Transform.Translation; :2765 inside Match IK -> Set Transform_1 (PV control)`

#### UE3-ik-base-control — IK Base (root-position) control on a scale-free Parent Buffer

*important* · assets: CRM_FN_IkFk2Bones

Construction:
- Spawns null 'Parent Buffer' under Parent at Parent's global transform.
- Spawns control 'IK Base' under it with offset translation = VB[0] initial translation.
  - Rotation limits min = max = 0 on all axes; scale limits min = max = 1, so it is effectively translate-only.
  - Shape = IK Control Shape with scale *1.5.

Forward Solve, every frame: Parent Buffer global = ProjectTransformToNewParent(child = buffer initial, old parent = Parent initial, new parent = Parent current). Only rotation and translation are written, scale stays at 1, so the limb can hang off a scaled bone (per the graph comment).

IK mode: Virtual Bone A global translation = IK Base global translation, so the animator can move the IK root (e.g. shoulder).

Backwards and Match IK: IK Base follows VB[0]. It is visible only in IK mode and when 'Sec Controls Vis' is on.

**Setup.** 'IK Base' is a Box_Thick control (display name Arm/Leg), translate-only via limits.

**Operators:** `RigUnit_ProjectTransformToNewParent`, `RigUnit_SetTransform`, `RigUnit_SetTranslation`, `RigUnit_HierarchyAddControlTransform (limits)`

**Scale:** 1 per limb

**Evidence:** `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:986 'Spawn Null | Name=Parent Buffer'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:233 COMMENT 'The Parent buffer allows us to connect the limb to a bone that is being scaled'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:211,482-485,495 Set Transform_7 (VB[0] translation = IK Base)`; `<dump>/.../CRM_FN_IkFk2Bones/regen.py 'SpawnControl_1_1_1_1_2_2.Settings' LimitPitch=(bMinimum=true,bMaximum=true)...MinValue Rotation=0`

#### UE3-ik-end-align — IK End Align: separate end-rotation control

*important* · assets: CRM_FN_IkFk2Bones

Construction, when IK End Align = True (default; arms):
- Rotator control 'IK Rotation' under root 'Local Control', with global offset (VB2.rot * IK Rotation Offset, VB2.t) and shape ModularRigGizmoLibrary.Circle_Pins_Thick, color = IK color * (0.8, 0.8, 0.5).
- Null 'IK Rotation Null' under VB1 at that transform.
- Bool channel 'IK End Align' (initial false) on IK, hosted on IK Rotation.

Forward IK path: IK Rotation visibility = channel value. Then IK End Align Solve runs.

Edge detection (channel != Previous Ik End Align, then store the new value):
- Switched on: SetControlOffset(IK Rotation, global = (rot of IK Rotation Null, t/s of IK Gimbal)); IK Rotation global rotation = IK Gimbal rotation; autokey.
- Switched off: IK Control global rotation = IK Rotation rotation; autokey; IK Gimbal local rotation = identity; autokey; IK Null global rotation = IK Rotation rot * IK Rotation Offset.

Every frame while on:
- SetControlOffset(IK Rotation, global = (IK Rotation Null rot, IK Control t/s)). The offset sets both current and initial, so the rotation control rides with IK position but orients from the mid bone frame.
- VB2 rotation = IK Rotation rot * IK Rotation Offset, with Weight = 0 (a no-op).
- IK Null global rotation = IK Rotation rot * IK Rotation Offset.

Note this runs after Soft IK.

**Setup.** Arms: 'IK Rotation' rotator (Circle_Pins_Thick) plus 'IK End Align' bool on IK. Legs override IK End Align = False.

**Operators:** `RigUnit_HierarchyAddControlRotator`, `RigUnit_SetControlOffset`, `RigUnit_SetRotation`, `RigUnit_SendEvent(RequestAutoKey)`, `RigUnit_GetBoolAnimationChannelFromItem`, `DISPATCH_RigVMDispatch_CoreNotEquals`

**Scale:** 2 arms per biped

**Evidence:** `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:2844-2976 'IK End Align Solve'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:2897 'Set Rotation | ... Weight=0.000000'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:780,792,808 'Name=IK End Align' / 'IK Rotation Null' / 'Spawn Rotator Control | Name=IK Rotation'`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Hierarchy/RigUnit_SetControlOffset.cpp:31-35`

#### UE3-mid-control — Mid (elbow/knee) offset control with slerped Mid null

*important* · assets: CRM_FN_IkFk2Bones

Construction:
- Spawns root-level null 'Mid' at Compute Mid Null Transform(VB) = (slerp(VB0.rot, VB1.rot, 0.5), VB1.t).
- Spawns control 'Mid' under it with local identity offset and display name IK Mid Display Name ('Elbow'/'Knee').
  - Shape HalfCircle_Solid (rot (-.5,-.5,.5,.5), translation (0,10,0), scale (.5,.5,10) by default), overridden per side (see negative side).
  - Color = IK color.
- Sets its mirror behavior.

Forward, every frame after the IK or FK solve:
1. Mid Null global = the slerped transform from the current VBs.
2. Compute FK([VB0, Mid Control global, VB2]) sets the Virtual Bones: VB0 re-aims at the Mid control position, VB1 moves to the Mid control and aims at VB2, with twist from the Mid control's Z axis.

The Mid control is therefore a secondary pin/offset control on the elbow or knee in both modes.

Backwards sets Mid Null local = identity. It is visible when 'Sec Controls Vis' is on.

**Setup.** 'Mid' is a HalfCircle_Solid EULER_TRANSFORM control, visibility tied to 'Sec Controls Vis'.

**Operators:** `RigVMFunction_MathQuaternionSlerp`, `DISPATCH_RigVMDispatch_MakeStruct`, `RigUnit_HierarchyAddNull`, `RigUnit_HierarchyAddControlTransform`, `RigUnit_SetTransform`

**Scale:** 1 per limb

**Evidence:** `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:2070-2088 'Compute Mid Null Transform' (Interpolate T=0.5)`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:642,644 'Spawn Null | Name=Mid' / 'Spawn Transform Control | Name=Mid'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:314-315,546,420-424 Set Transform_4 -> Compute FK Transforms.1 = Mid Control`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:1724-1725,1764 Backwards Set Transform_3 Mid Null LocalSpace identity`

#### UE3-segment-scale — Upper/Lower segment scale channels (FK and IK stretch)

*important* · assets: CRM_FN_IkFk2Bones

When Segment Scale Control is on (default), construction spawns SCALE_FLOAT channels 'Upper Segment Scale' and 'Lower Segment Scale' on the IK control. Each has initial 1, min 0.0001 (min limit only), max 2, and hosts = FK Controls + IK Base.

Forward:
- Us/Ls = channel values.
- Scaled Lengths = [Default[0]*Us, Default[1]*Ls]; if the option is off, Scaled Lengths = Default Lengths.
- IK mode: the scales feed Soft IK lengths and the PV parent offset.
- FK mode ('Locking FK Control Channels'): FK 0 and FK 1 local scale are forced to 1. Then the parent null of FK 1 gets local translation X = initialLocal.X * Us, and the parent null of FK 2 gets X = initialLocal.X * Ls.

Backwards, and in Key Controls while interacting: Calculate And Set Segment Scale Channel Values sets channel[i] = CurrentLength[i]/DefaultLength[i], where current lengths are distances between the virtual bones. This carries IK stretch into FK and keys.

Default Lengths = Get Distances Between(Bones, Initial = true), i.e. consecutive distances [|B0-B1|, |B1-B2|].

**Setup.** Scale-float channels 'Upper Segment Scale' and 'Lower Segment Scale' live on IK and are shared (channel hosts) with FK 0/1/2 and IK Base.

**Operators:** `RigUnit_HierarchyAddAnimationChannelScaleFloat`, `RigUnit_GetFloatAnimationChannelFromItem`, `RigUnit_SetFloatAnimationChannelFromItem`, `RigUnit_SetTranslation(Local)`, `RigUnit_SetScale`, `RigUnit_SetChannelHosts`, `RigVMFunction_MathVectorDistance`

**Scale:** 2 channels per limb

**Evidence:** `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:922-923 'Spawn Scale Float Animation Channel | Name=Upper Segment Scale; InitialValue=1.000000; MinimumValue=0.000100; MaximumValue=2.000000'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:175-183,440-452 GetTransform_9 LocalSpace bInitial -> Multiply_2 -> Set Translation X`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:223 COMMENT 'Locking FK Control Channels' + 220-222 SetScale`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:3105-3132 'Calculate And Set Segment Scale Channel  Values'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:2090-2134 'Get Distances Between'`

#### UE4-compute-pole-vector-v01 — Compute Pole Vector v01 (elbow offset from the start-end line)

*important* · assets: CRFL_Hierarchy_v001

Inputs: Transforms[3], Secondary Axis (library default (0,1,0)) and Project To Plane. Outputs: Pole Vector (vector), Elbow Position and Upper Segment Length. With P0, P1, P2 the three translations and SE = P2 - P0: if Project To Plane, elbow = IntersectPlane(start=P1, dir=n, plane point=P0, normal=n) with n = cross(T1.TransformLocation(SecAxis) - P1, SE); else elbow = P1. SM = elbow - P0. foot = P0 + SE * dot(SM,SE)/|SE|^2. Pole Vector = elbow - foot, the unnormalised perpendicular from the start-end line to the elbow. Upper Segment Length = |SM|.

**Operators:** `RigVMFunction_MathVectorSub`, `RigVMFunction_MathVectorDot`, `RigVMFunction_MathVectorLengthSquared`, `RigVMFunction_MathDoubleDiv`, `RigVMFunction_MathVectorScale`, `RigVMFunction_MathVectorAdd`, `RigVMFunction_MathVectorCross`, `RigVMFunction_MathIntersectPlane`, `RigVMFunction_MathTransformTransformVector`, `RigVMFunction_MathVectorLength`

**Scale:** Called twice by Compute Pole Vector Location v02.

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:1808-1897 'Compute Pole Vector v01' (comments 'Project mid bone location to vector between start and end', 'Project Knee to Leg plane')`; `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/regen.py:3349 add_link('RigVMFunction_MathVectorLength.Result', 'Return.Upper Segment Length')`

#### UE4-ik-plane-virtual-bones-v02 — Create IK Plane Virtual Bones v02 (planar helper bone chain for 2-bone IK)

*important* · assets: CRFL_Hierarchy_v001, CRM_FN_IkFk2Bones, MR_FN_Biped

Construction. Inputs: Start, Mid, End, Name, Axis (primary, default X) and Secondary Axis (default Y). Output: Bones[3]. [P0,P1,P2] = translations of ProjectMiddleBoneToIKPlane(Start, Mid, End, SecondaryAxis, Debug=false). n = cross(P1-P0, P2-P1). Bone A: AimBoneMath(pos=P0, identity rotation, Primary{Axis, target P1 as a location}, Secondary{Axis (0,0,1), target n as a direction}); spawned GlobalSpace as Name+'Bone A' under Start's default parent. Bone B: the same aim at P2 from P1; parent = Bone A. Bone C: at P2, Primary{Axis, target = E_cur.TransformLocation(Axis)}, Secondary{Axis (0,-1,0), target = E_cur.TransformLocation((0,0,2)), as a location}; parent = Bone B. E_cur is End's current global, which equals initial during construction. The virtual chain lies in one plane with the primary axis down the chain and local Z along the plane normal; the IK solve runs on it, and FK/IK matching reads it.

**Operators:** `FUNC Project Middle Bone to IK Plane`, `RigVMFunction_MathTransformArrayToSRT`, `RigVMFunction_MathVectorSub`, `RigVMFunction_MathVectorCross`, `RigUnit_AimBoneMath`, `RigUnit_HierarchyAddBone (GlobalSpace)`, `RigUnit_HierarchyGetParent (default)`, `RigVMFunction_MathTransformTransformVector`, `RigVMFunction_NameConcat`

**Scale:** IkFk2Bones 1 call per limb module (4 limbs in the biped).

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:1421-1563 'Create IK Plane Virtual Bones v02' (Concat 'B=Bone A/B/C', AimBoneMath_2 Secondary 'Axis=(X=0,Y=-1,Z=0)', TransformVector_1 'Location=(0,0,2)')`; `ue/<dump>/FortniteRigs__Modules__IkSolves__IkFk2Bones__CRM_FN_IkFk2Bones/graphs.txt:677 'Create IK Plane Virtual Bones v02 | Name=Virtual '`; `ue/<dump>/FortniteRigs__Templates__MR_FN_Biped/runtime_hierarchy.txt:916-918 'BONE Arm L/Virtual Bone A parents=[BONE:clavicle_l]', 'Virtual Bone B', 'Virtual Bone C'`

#### UE4-project-mid-to-ik-plane — Project Middle Bone to IK Plane (planarise a 3-joint chain)

*important* · assets: CRFL_Math_v001, CRFL_Hierarchy_v001

Always uses INITIAL global transforms S, M and E. Inputs: Start, Mid, End, Debug and Secondary Axis (library default (0,1,0)). Outputs: outputs (Transform[3]) and Same As Source (bool). sec = M.TransformLocation(SecondaryAxis) - M.pos, the world direction of the mid joint's secondary axis. SE = E.pos - S.pos. n = cross(sec, SE). X = RayPlaneIntersection(start=M.pos, dir=n, plane through S.pos with normal n), the mid joint projected onto the plane through S that contains sec and SE. IntersectPlane returns zero when dot(n,dir) is about 0. v = X - S.pos. newMid = S.pos + v * (|S.pos - M.pos| / |v|), which preserves the upper segment length. out = {rot=M.rot, pos=newMid, scale=M.scale}. outputs = [S, out, E]. Same As Source = |S.pos - out.pos| < 0.001; as wired this compares Start with the new mid, which looks like an authoring slip. When Debug is on it draws a red line S->newMid (thickness 0.2) and a rectangle at out.

**Operators:** `RigUnit_GetTransform (initial)`, `RigVMFunction_MathTransformTransformVector`, `RigVMFunction_MathVectorSub`, `RigVMFunction_MathVectorCross`, `RigVMFunction_MathIntersectPlane`, `RigVMFunction_MathVectorDistance`, `RigVMFunction_MathVectorLength`, `RigVMFunction_MathDoubleDiv`, `RigVMFunction_MathVectorMul`, `RigVMFunction_MathTransformMake`, `RigVMFunction_DebugLineNoSpace`, `RigVMFunction_DebugRectangleNoSpace`, `DISPATCH_RigVMDispatch_ArrayMake`, `RigVMFunction_MathDoubleLess`

**Scale:** Called by Create IK Plane Virtual Bones v01 and v02 (and by the library test graph).

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Math_v001/graphs.txt:298-439 'Project Middle Bone to IK Plane' (comments 'Cross knee axis with root to tip vector...', 'Find point along new vector root to intersection point that is at same length as original upper segment')`; `UE_5.8/Engine/Plugins/Runtime/RigVM/Source/RigVM/Private/RigVMFunctions/Math/RigVMFunction_MathVector.cpp:708-720 IntersectPlane`

#### UE4-auto-pole-vector-v02 — Construct / Compute Auto Pole Vector v02 (IK-following PV space, unused)

*nice-to-have* · assets: CRFL_Hierarchy_v001

Construct (Name, Shape, PV Root Space, Item A/B/C, Parent, IK Control, Offset Multiplier, Initial) spawns four elements. (1) Null '<Name>RootSpaceNull' under PV Root Space's default parent. (2) Null '<Name>RootNull' under Parent. Both use T = AimBoneMath(PVRootSpace global, Primary X at the IK Control position, Secondary Z toward the IK Control's X direction). (3) Null '<Name>Null' under RootNull at the translation from ComputePoleVectorFromPlane(A,B,C). (4) A position Vector control '<Name>' (shape default Diamond_Solid) under that null. Outputs: Pv Control and Pv Root Space Null. Compute (PV Control, IK Control, PV Root Space, PV Orient Space, PV Twist Follow, Initial) calculates two aims from PV Root Space. A aims X at the IK control with Y toward the root space's Y. B aims X at the IK control with Z toward the IK control's X. It sets the PV control's grandparent global to Lerp(A, B, PVTwistFollow), using slerped rotation, and the PV control's parent global rotation to PV Orient Space's rotation. A graph comment notes that the root space should be the parent joint of item A to avoid cycles in off-plane IK.

**Setup.** Would expose a float 'PV Twist Follow' that blends between the root-space up direction and the IK-control twist.

**Operators:** `RigUnit_AimBoneMath`, `RigVMFunction_MathTransformLerp`, `RigUnit_HierarchyGetParent`, `RigUnit_SetTransform`, `RigUnit_SetRotation`, `RigUnit_HierarchyAddNull`, `RigUnit_HierarchyAddControlVector`, `FUNC Compute Pole Vector From Plane v01`

**Scale:** No callers; IkFk2Bones uses its own 'Compute Pole Vector Parent'.

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:1198-1245 'Compute Auto Pole Vector v02'`; `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:1247-1319 'Construct Auto Pole Vector v02' (Concat 'B=RootNull', 'B=Null', 'B=RootSpaceNull')`; `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/regen.py:3393 SpawnControl_2.Settings '(...bIsPosition=True...Shape=(...Name="Diamond_Solid"...)'`; `UE_5.8/Engine/Plugins/Runtime/RigVM/Source/RigVM/Private/RigVMFunctions/Math/RigVMMathLibrary.cpp:253-260 LerpTransform (lerp T/S, slerp R)`

#### UE4-ik-plane-virtual-bones-v01 — Create IK Plane Virtual Bones (v01, unused)

*nice-to-have* · assets: CRFL_Hierarchy_v001

Like v02, but Secondary Axis is fixed at (0,1,0) for the projection, and a Negative Side bool negates the primary Axis (axis var = NegativeSide ? -Axis : Axis). Bone C's primary target is E.TransformLocation((±2,0,0)): -2 when Negative Side, else +2.

**Operators:** `FUNC Project Middle Bone to IK Plane`, `RigUnit_AimBoneMath`, `RigUnit_HierarchyAddBone`, `RigVMFunction_MathVectorNegate`, `DISPATCH_RigVMDispatch_If`

**Scale:** No callers.

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:678-838 (If_1 'True=-2.000000; False=2.000000', Project node 'Secondary Axis=(0,1,0)')`

#### UE4-math-compute-pole-vector — Compute Pole Vector (CRFL_Math, unused, buggy rotation)

*nice-to-have* · assets: CRFL_Math_v001

Inputs: Bone A, Bone B, Bone C, Initial, Origin Scale (vector), OffsetFactor (default 2) and Draw. Output: Transform. The translation is correct: ab = B-A, ac = C-A, proj = (dot(ab,ac)/|ac|) * unit(ac), translation = B + (ab - proj) * OffsetFactor. The rotation is built from X = 0.5*(ab - (dot(A.pos, C.pos)/|ac|)*unit(ac)), which dots the positions rather than the vectors and so is wrong, Y = unit(cross(ac, ab)) and Z = unit(cross(Y, X)). It is assembled with MatrixFromVectors(X, Y, Z, Origin=OriginScale) and then round-tripped through Euler XYZ. When Draw is on it draws axes at the result (scale 20).

**Operators:** `RigVMFunction_MathVectorSub/Dot/Length/Unit/Scale/Add/Cross`, `RigVMFunction_MathFloatDiv`, `RigVMFunction_MathMatrixFromVectors`, `RigVMFunction_MathMatrixToTransform`, `RigVMFunction_MathQuaternionToEuler/FromEuler`, `RigVMFunction_DebugTransformMutableNoSpace`

**Scale:** No callers.

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Math_v001/graphs.txt:83-282 'Compute Pole Vector' (links 233-238: Dot_1.A <- boneA translation, Dot_1.B <- boneC translation)`

#### UE4-pv-from-plane-v01 — Compute Pole Vector From Plane v01

*nice-to-have* · assets: CRFL_Hierarchy_v001

Inputs: Item A, Item B, Item C, OffsetMultiplier and Initial. Output: Result (Transform). AB = B - A and AC = C - A. perp = (AC x AB) x AC, the in-plane direction perpendicular to AC pointing toward B. pos = B + normalize(perp) * |AB| * OffsetMultiplier. Result = AimBoneMath({identity rotation, pos}, Primary{X axis at B.pos, as a location}, Secondary{Z axis to world (0,0,1), as a direction}). A VisualDebugVector node on the plane normal is left enabled (bEnabled=True), so it draws whenever the function runs.

**Operators:** `RigUnit_GetTransform`, `RigVMFunction_MathVectorSub`, `RigVMFunction_MathVectorCross`, `RigVMFunction_MathVectorSetLength`, `RigVMFunction_MathVectorLength`, `RigVMFunction_MathDoubleMul`, `RigVMFunction_MathVectorScale`, `RigVMFunction_MathVectorAdd`, `RigUnit_AimBoneMath`, `RigVMFunction_VisualDebugVectorNoSpace`

**Scale:** Used only by Construct Auto Pole Vector v02, which has no callers.

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:1321-1376 'Compute Pole Vector From Plane v01' (VisualDebugVectorNoSpace_1 'bEnabled=True', comment 'Flip Here')`

#### UE4-pv-location-v01 — Compute Pole Vector Location v01 (slerped up-vector offset, unused)

*nice-to-have* · assets: CRFL_Hierarchy_v001

Inputs: Item A, Item B, Up Vector, OffsetMultiplier and Initial. Output: Result (vector). len = |B.pos - A.pos| * OffsetMultiplier. q = Slerp(A.rot, B.rot, 0.5). Result = FTransform(q, B.pos).TransformLocation(UpVector * len), i.e. B.pos + q * (Up * len).

**Operators:** `RigUnit_GetTransform`, `RigVMFunction_MathQuaternionSlerp (T=0.5)`, `DISPATCH_RigVMDispatch_MakeStruct`, `RigVMFunction_MathVectorMul`, `RigVMFunction_MathTransformTransformVector`, `RigVMFunction_MathVectorLength`

**Scale:** No callers.

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:1378-1419 'Compute Pole Vector Location v01' (MathQuaternionSlerp_1 'T=0.500000')`

#### UE8-unused-module-helpers — Defined-but-unused helpers: Root 'Snap Global Control' and IkFk2Bones utilities

*nice-to-have* · assets: CRM_FN_Root, CRM_FN_IkFk2Bones

None of these are referenced. Snap Global Control (Root, commented 'Used in Backward Solve to snap the Global Control to the root on the first frame only'): if the 'Global Control Snapped' latch is false, set Global Control's global transform to Root Bone's global transform, then set the latch to true. Select Control Shape (IkFk2Bones): HierarchyGetShapeSettings(item), then HierarchySetShapeSettings with the same color, transform and visibility and Name = Value ? True Name : False Name (defaults Sphere_Solid / Box_Solid). Compute Twist Sockets: cache start/end locations (from Location Items) and orientations (from Orientation Items). Start = AimBoneMath(input at start location, Primary target = end location, Secondary target = StartOrientation.Rotate(StartTargetAxis), Secondary axis = StartTargetAxis negated if Condition && StartTargetAxis == (0,1,0)). End = AimBoneMath(input at end location, Primary target = start location, Secondary target = EndOrientation.Rotate(EndTargetAxis), Secondary axis = Condition ? -EndTargetAxis : EndTargetAxis). Compute Mid Bone Transform For IK Plane Align: with root = T[0], mid = T[1], tip = T[2], n = cross(midMatrix.Y, tip - root). I = IntersectPlane(start = mid, direction = n, plane point = root, plane normal = n). newMid.translation = root + (I - root) * |mid - root| / |I - root|; rotation and scale are kept; T[1] is replaced; optional debug line and rectangle. Offset Rotation On Transforms: for each transform, rotation = rotation * RotationOffset (quaternion multiply).

**Setup.** none

**Operators:** `RigUnit_AimBoneMath`, `RigVMFunction_MathIntersectPlane`, `RigVMFunction_MathVectorCross`, `RigVMFunction_MathMatrixToVectors`, `RigUnit_HierarchyGetShapeSettings`, `RigUnit_HierarchySetShapeSettings`, `DISPATCH_RigVMDispatch_SelectInt32`

**Scale:** 0 uses (5 functions)

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Root__CRM_FN_Root/graphs.txt:390-410`; `<dump>/FortniteRigs__Modules__IkSolves__IkFk2Bones__CRM_FN_IkFk2Bones/graphs.txt:2136-2159, 2160-2246, 2558-2644, 2645-2665`; `<dump>/FortniteRigs__Modules__IkSolves__IkFk2Bones__CRM_FN_IkFk2Bones/summary.json :: functions_used (none of these four)`; `<dump>/FortniteRigs__Modules__FkSolves__Root__CRM_FN_Root/summary.json :: functions_used (no 'Snap Global Control')`

### D7 Spine, neck and twist

#### UE1-layout-spine-neck — Spine and Neck module layout (FK, IK, secondary FK, blend channels)

*core* · assets: MR_Zebra, MR_Monster, MR_FN_Biped

CRM_FN_Spine is instanced as Spine (pelvis->spine_05) and as Neck (neck_01->head, config Is Neck=True).

Spine (21 controls):
- FK: Start/Mid/End FK = 'Pelvis FK'/'Waist FK'/'Chest FK' (Circle_Thin), chained: Start FK under 'Default Start FK Space' (under Body/Body); Mid under Start; End under 'End IK Null' under Mid.
- IK: Start/Mid/End IK = 'pelvis'/'Waist'/'Chest' (Hexagon_Thick). Start IK under Start FK; End IK under End FK; Mid IK under 'Mid IK Null' under End FK.
- Secondary FK: Pelvis Sec FK (under bone root), Spine 01-04 Sec FK (Triangle_Thin chain), Spine 05 Sec FK (Hexagon_Thick), and 'Pelvis Local' (Hexagon_Thick under Pelvis Sec FK).
- End Movable Pivot: proxy, 'Chest Moveable Pivot'.
- Channels:
  - Mid Blend (SCALE_FLOAT 0.65, hosted on Mid FK, End FK and End IK)
  - Stretch (SCALE_FLOAT 1, on End FK; hosted on Start FK, End IK and Start IK)
  - Distribute Rotation (SCALE_FLOAT, on End FK)
  - IK Vis, FK Vis and Sec FK Vis (hosted on the 7 main controls)
  - Movable Pivot Vis

Helper elements: 6 virtual bones '<bone>_virtual', plus '_reoriented' and '_match' null chains and a 'Spine/Pelvis TXY' world null.

Neck (15 controls): the same main controls with names Neck Base FK / Neck Mid FK / Head FK and Neck Base / Neck Mid / head; Sec FK chain Neck 01, Neck 02 and Head Sec FK (under bone spine_05); no movable pivot; 3 virtual bones.

Spaces: Start/End FK and IK get Global/Local/Body Orient; Neck adds spine_05 Orient.

**Setup.** Spine: 21 controls (14 transforms, 1 proxy, 6 channels). Neck: 15 (9 transforms, 6 channels). Spine config Control Scale 1.2; Neck 2.0.

**Operators:** `CRM_FN_Spine`

**Scale:** Zebra and Biped have Spine and Neck. Monster has Spine only.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1918 'CONTROL Spine/Start FK ... display_name=Pelvis FK'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1414 'CONTROL Neck/Mid Blend ... control_type=SCALE_FLOAT'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1548 'BONE Spine/spine_05_virtual'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/asset.t3d:713 'Is Neck=True'`

#### UE2-spine-attach-to-spline — Attach Sec FKs To Spline (length-preserving fit + stretch + twist up-vector)

*core* · assets: CRM_FN_Spine

Local function with input Matching (false in forwards, true in backwards).

Step A:
1. FitChainToSplineCurveItemArray(Items = Virtual Bones, Spline, Alignment = Front, Min 0, Max 1, SamplingPrecision 12, PrimaryAxis +X, SecondaryAxis 0, RotationEase Linear).
2. For each virtual bone except the first: local translation = (initial local X, 0, 0). This restores the original segment lengths.

Step B, for each bone i:
- p = metadata 'Bone Percentage'
- r = SplineLength / SplineOrigLength
- up = Slerp(StartIK.rot, EndIK.rot, p).RotateVector(Secondary Axis)
- T3 = TransformFromSpline(U = ParamAtLengthPercentage(p), up)
- T2 = TransformFromSpline(U = ParamAtLengthPercentage(p / r), up). TransformFromSpline builds X = tangent, Z = tangent x up.
- A = (translation = VirtualBones[i] global translation, rotation and scale from r > 1 ? T2 : T3)
- I = Lerp(A, T3, Stretch). With Stretch = 0 the bones keep their original lengths; with Stretch = 1 they sit at the original percentages of the stretched curve.
- rotation:
  - last bone (ratio == 1): EndIK.rot * inverse(EndIK meta 'Offset'.rot)
  - first bone (ratio == 0): IsNeck ? I.rot : StartIK.rot * inverse(StartIK 'Offset'.rot)
  - otherwise: I.rot
- If Matching: rotation *= Sec FKs Orient Offset and the target is MatchNulls[i]; otherwise the target is Bones[i].
- target global = (I.T, rotation, scale 1).

**Setup.** 'Stretch': SCALE_FLOAT channel on End FK, default 1, range [0,1], hosts [Start FK, End IK, Start IK].

'Secondary Axis' (construction) = cross(EndIK_init.T - StartIK_init.T, lerp(StartIK_init X axis, EndIK_init X axis, 0.5)).

**Operators:** `RigUnit_FitChainToSplineCurveItemArray`, `RigUnit_ParameterAtPercentage`, `RigUnit_TransformFromControlRigSpline`, `RigUnit_GetLengthControlRigSpline`, `RigVMFunction_MathQuaternionSlerp`, `RigVMFunction_MathQuaternionRotateVector`, `RigVMFunction_MathTransformLerp`, `RigVMFunction_MathQuaternionInverse`, `RigUnit_SetTranslation`, `RigUnit_SetTransform`, `DISPATCH_RigDispatch_GetMetadata`

**Scale:** 2 calls per spine/neck

**Evidence:** `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:2046 'FitChainToSplineCurveItemArray | Alignment=Front; ... SamplingPrecision=12'`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:2005 'Name=Bone Percentage'`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:2067,2072 'Name=Offset'`; `<UE>/Plugins/Animation/ControlRigSpline/Source/ControlRigSpline/Private/ControlRigSplineUnits.cpp:150-183 (TransformFromControlRigSpline)`

#### UE2-spine-bezier-spline — Cubic Bezier spline from Start driver / Mid IK / End IK

*core* · assets: CRM_FN_Spine

'Build Spline From Items'(Start, Mid, End) = ControlRigSplineFromPoints(Points = global translations of [Start Driver, Mid IK, Mid IK, End IK], SplineMode = BSpline, bClosed = false, SamplesPerSegment = 16, Compression = 0, Stretch = 0).

The engine builds a clamped degree-3 BSpline. With 4 points that is exactly a cubic Bezier whose two inner control points are both the Mid IK position.

When the spline is used:
- Construction: build it once and store 'Spline Orig Length' = GetLengthControlRigSpline.
- Forwards: rebuild every frame into the 'Spline' variable.
- Backwards: rebuild only when Sec FK Vis is on.
- Debug (private bool): DrawControlRigSpline (color (0.5,0,0), thickness 0.1, detail 16).

**Operators:** `RigUnit_GetTransformItemArray`, `RigVMFunction_MathTransformArrayToSRT`, `RigUnit_ControlRigSplineFromPoints`, `RigUnit_GetLengthControlRigSpline`, `RigUnit_DrawControlRigSpline`

**Scale:** 3 calls (construction, forward, backward)

**Evidence:** `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:1636 'SplineMode=BSpline; bClosed=False; SamplesPerSegment=16'`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/regen.py 'GetTransformItemArray.Items' 4 items (links Items.0=Start, Items.1/2=Mid, Items.3=End)`; `<UE>/Plugins/Animation/ControlRigSpline/Source/ControlRigSpline/Private/ControlRigSplineTypes.cpp:93-123,439 'new ControlRigBSpline(ControlPoints, 3, bInClosed, true)'`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:4 'RigUnit_DrawControlRigSpline'`

#### UE2-spine-control-layout — Spine/neck control layout (FK trio, IK trio, Sec FK chain)

*core* · assets: CRM_FN_Spine

Construction.

Setup: Bones = chain(Start..End). N = number of bones. StartPos = initial translation of Start Snap To (or of Bones[0] if that connector is not connected). EndPos = Bones[-1] initial translation.

FK stack. FK Bones = [Bones[0], Bones[N/2] (integer division), Bones[-1]]. Starting with FKParent = Start Parent, for each i:
- Before the last one only, spawn null 'End IK Null' at StartPos under FKParent and set FKParent to it.
- Spawn null 'Default <Start|Mid|End> FK Space' under FKParent at global (R = Controls Orient Offset, T = StartPos for i = 0, otherwise the FK bone's initial translation).
- Spawn control '<Start|Mid|End> FK' at the same offset. Store metadata 'FkDeltaTransform' = MakeRelative(control init, FK bone init).
- FKParent = End Parent = the new control.

IK controls:
- 'Default Start IK Space' null under Start FK at Bones[0] init; 'Start IK' control under it at StartPos; metadata 'Offset' = relative(StartIK init, Bones[0] init).
- 'Start Snap To' null under Start IK at Bones[0]; this null becomes 'Start Driver'.
- 'Default End IK Space' null under End FK at Bones[-1]; 'End IK' at EndPos; 'Offset' relative to Bones[-1].
- 'Mid IK Null' under End FK at lerp(StartPos, EndPos, 0.5); 'Mid IK' under it with a local offset (R = Controls Orient Offset); 'Offset' relative to Bones[N/2].

Sec FK chain: for each bone, '<Bone> Sec FK' is a child of the previous one (the first is under the parent of Bones[0]), with global offset (bone T, bone R * Sec FKs Orient Offset, bone S). A parallel null chain '<bone>_match' is created.

Runtime hierarchy: Body/Body > Default Start FK Space > Start FK > Default Mid FK Space > Mid FK > End IK Null > Default End FK Space > End FK > {Default End IK Space > End IK, Mid IK Null > Mid IK, ...}.

**Setup.** Public config:
- Control Scale = 1
- Color = white (resolved to a side color)
- Default Distribute Rotation = 0
- Is Neck = false
- Rotation Order = XYZ
- Controls Orient Offset = identity
- Sec FKs Orient Offset = quat(0,0.7071,0,0.7071), i.e. 90 degrees about Y
- Display name options

Private shapes: IKs = Hexagon_Thick; FKs = Circle_Thin; Sec FKs = Triangle_Thin; Local FKs = Hexagon_Thick with pink color (1,0.107,0.523).

Spine connections: Start = pelvis, End = spine_05, Start/End Parent = Body/Body, Start Snap To = Body/Body.
Neck connections: Start = neck_01, End = head, Parents = spine_05.

**Operators:** `RigUnit_HierarchyGetChainItemArray`, `RigUnit_HierarchyAddNull`, `RigUnit_HierarchyAddControlTransform`, `RigUnit_GetRelativeTransformForItem`, `DISPATCH_RigDispatch_SetMetadata`, `RigVMFunction_MathVectorLerp`, `RigVMFunction_MathIntDiv`, `RigUnit_ResolveConnector`

**Scale:** Spine: 13 controls, 35 nulls. Neck: 9 controls, 30 nulls. 2 instances per biped rig (Spine and Neck).

**Evidence:** `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:546 'Values=("Start FK","Mid FK","End FK")'`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:533 'Name=End IK Null'`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:245-248,398,403,560 IK controls/nulls`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:295,299,303 'Name=Offset' ; :338 'Name=FkDeltaTransform'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt 'NULL Spine/End IK Null parents=['CONTROL:Spine/Mid FK']'`

#### UE2-spine-secfk-offset-layer — Secondary FK layer riding on the spline result (runtime control offsets)

*core* · assets: CRM_FN_Spine

Forwards, after Attach Sec FKs To Spline has written the spline pose into Bones.

Step C, for each bone i:
- ReOrientedNull[i] global = (Bones[i].T, Bones[i].R * Sec FKs Orient Offset, Bones[i].S)
- SetControlOffset(SecFK[i], offset = ReOrientedNull[i] LOCAL transform, LocalSpace)

The Sec FK controls' offset transforms follow the IK spline every frame, while their animated local values add FK on top.

Step D, for each bone i: Bones[i] global = (SecFK[i].T, SecFK[i].R * inverse(Sec FKs Orient Offset), SecFK[i].S).

**Setup.** Sec FK controls: Triangle_Thin, 'Sec FKs Control Shapes'; display names from 'FK Secondary Display Names'. Visibility follows 'Sec FK Vis' except for the last Sec FK, which is always visible and gets the Local FK (chest-local) shape.

**Operators:** `RigUnit_SetControlOffset`, `RigUnit_SetTransform`, `RigVMFunction_MathQuaternionMul`, `RigVMFunction_MathQuaternionInverse`, `RigUnit_GetTransform`

**Scale:** 6 (spine) + 3 (neck) Sec FK controls

**Evidence:** `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt (RigVMModel) 'SetControlOffset_2 ... Space=LocalSpace' with 'GetTransform_8 Space=LocalSpace' on ReOriented Nulls`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/Hierarchy/RigUnit_SetControlOffset.h:10-15`

#### UE3-limbtwist-blend — Weighted twist distribution (Blend Twist + Get Node Twist Value)

*core* · assets: CRM_FN_LimbTwist, CRFL_Math_v001

Get Node Twist Value(Item, Axis):
- delta = currentLocal relative to initialLocal (rotation = initial^-1 * current);
- twist = SwingTwist(delta, Axis).Twist.

Blend Twist(Driver, Drivens, Weights, Axis, Reverse):
1. t = twist of Driver.
2. For each driven i with weight w:
   - q = Reverse ? slerp(identity, t, w) : slerp(inverse(t), identity, w);
   - set driven local rotation = q;
   - OffsetTransformForItem(driven, offset rotation = driven's initial local rotation), giving local = offset * current, i.e. quaternion q * initialRot.
3. Optional Draw Axis debug.

Forward:
- Driver = Reverse ? End : Start.
- Drivens = Twist Control ? parents of the twist controls (the Twist n Nulls) : Twist Bones.
- Weights default (0.25, 0.75, 1.0); biped overrides remove index 2 and set legs upper[0] = 0.33.
- Axis default (1,0,0); some right-side instances use (-1,0,0).

Effect: in non-reverse (upper twist) mode, twist bone i ends with fraction w_i of the start bone's twist (the child undoes 1 - w). In reverse (lower twist) mode, bone i receives fraction w_i of the hand/foot twist.

**Setup.** Config: Twist Axis, Twist Weights array, Twist Reverse (true for lower arm/leg, where the End drives).

**Operators:** `RigVMFunction_MathQuaternionSwingTwist`, `RigVMFunction_MathTransformMakeRelative`, `RigVMFunction_MathQuaternionSlerp`, `RigVMFunction_MathQuaternionInverse`, `RigUnit_SetRotation(Local)`, `RigUnit_OffsetTransformForItem`

**Scale:** 8 per biped per frame

**Evidence:** `<dump>/.../CRM_FN_LimbTwist/graphs.txt:435-522 'Blend Twist'`; `<dump>/FortniteRigs__Libraries__CRFL_Math_v001/graphs.txt:455-473 'Get Node Twist Value'`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Hierarchy/RigUnit_OffsetTransform.cpp:34-43`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/asset.t3d 'Twist Weights(0)=0.330000' / 'Twist Reverse=True'`

#### UE3-limbtwist-forward — LimbTwist forward/backward evaluation order, start-bone de-twist and inverse

*core* · assets: CRM_FN_LimbTwist

Forward sequence:
A) Twist Parent Null global = Start Bone global.
B) Blend Twist, then Blend Translate (if enabled).
C) If Twist Control: TwistBone[i] global = TwistControl[i] global; control visibility = parent metadata 'Sec Controls Visibilty'.
D) If !Twist Reverse: 'Reset Twist on Start Bone'.
   - (swing, twist) = SwingTwist(StartGlobal.rot, TwistAxis);
   - Start global rotation = (TwistAxis.x + TwistAxis.y + TwistAxis.z < 0) ? swing * FromEuler(TwistAxis*180, ZYX) : swing;
   - no propagation, so children keep their globals.
   This removes the start bone's own twist against the world-identity reference, with a 180-degree flip for negative axes.

Backwards:
A) Twist Parent Null = Start.
B) If Use Translates: Blend Twist and Blend Translate on the twist nulls.
C) TwistControl[i] global = TwistBone[i] global.

Connector event: Parent defaults to the resolved Start. It also contains stale calls for 'Start Socket' and 'Clavicle' connectors.

**Setup.** None.

**Operators:** `RigVMFunction_MathQuaternionSwingTwist`, `RigVMFunction_MathQuaternionFromEuler`, `RigUnit_SetRotation(Global, no propagate)`, `RigUnit_InverseExecution`, `FUNC Set Default Match To Connector v01`

**Scale:** 8 per biped

**Evidence:** `<dump>/.../CRM_FN_LimbTwist/graphs.txt:2-131 RigVMModel (COMMENT 'Reset Twist on Start Bone', 'Multiply | B=(X=180,Y=180,Z=180)', 'Less | B=0')`; `<dump>/.../CRM_FN_LimbTwist/graphs.txt:305-373 Backwards Solve`; `<dump>/.../CRM_FN_LimbTwist/graphs.txt:286-303 Connector Event Graph`

#### UE3-limbtwist-setup — LimbTwist construction: twist bone discovery, twist nulls and offset controls

*core* · assets: CRM_FN_LimbTwist, CRFL_Hierarchy_v001

Connectors: Start (primary), Parent (optional), End (ChildOfPrimary). Start/End/Parent vars hold the connector keys.

Twist bones:
- Twist Bones = direct Bone children of Start whose names contain every token of Twist Search String split by '|'. The default is 'upperarm|twist'; overrides are 'lowerarm|twist', 'thigh|twist', 'calf|twist'.
- Order is the hierarchy child order.

When Use Translates is on (default): Translate Weights[i] = |twist_i - End| / |Start - End| at initial globals.

When Twist Control is on (default):
- null 'Twist Parent Null' under Parent (Body/Body in the biped) at Start's initial global;
- per bone i: null 'Twist <i+1> Null' under it at the bone's initial global;
- control 'Twist <i+1>' under that null: identity offset, Shape Settings (Tube_Solid default), color from metadata, display name from Display Names[i] ('Offset 1/2'), mirror metadata = FK Mirror Behavior;
- Set Control Scale with auto = |Start - End|/20.

The module identifier is 'CRM_Epic_LimbTwist_v02'.

**Setup.** 'Twist 1..N' offset controls (display 'Offset 1/2'), shown via the parent limb's 'Sec Controls Vis'.

**Operators:** `RigUnit_CollectionChildrenArray`, `RigVMFunction_StringSplit`, `RigVMFunction_StringContains`, `RigUnit_HierarchyAddNull`, `RigUnit_HierarchyAddControlTransform`, `RigVMFunction_MathIntToName`

**Scale:** 8 modules per biped, 2 twist bones each (UE mannequin)

**Evidence:** `<dump>/FortniteRigs__Modules__Biped__LimbTwist__CRM_FN_LimbTwist/graphs.txt:133-284 ConstructionGraph`; `<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:854-916 'Get Children by Contained Strings'`; `<dump>/.../CRM_FN_LimbTwist/graphs.txt:535-575 'Compute Translate Weights'`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/runtime_hierarchy.txt 'CONTROL Arm Upper Twist L/Twist 1 <- NULL:Arm Upper Twist L/Twist 1 Null'`

#### UE-helper-limb-volume — Twist-bone volume slides on deep elbow/knee bend

*important* · assets: CR_Zebra_Deform

Beyond 40% of elbow or knee bend (Remap 0.4..1), the rig adds local-axis translations to the twist bones to push or pull skin volume. Arm offsets are along twist-bone X: upperarm_twist_04 -8, 03 -5, 02 -4 and lowerarm_twist_04 +10, 03 +4 on the left, mirrored on the right. Leg offsets are along twist-bone Y: thigh_twist_04 +5, 03 +3.5, 02 +3 and thigh_twist_01 rotated Z-20, again mirrored. Knee bend also slides calf_twist_04/03 by 5 and 2 cm linearly. Shoulder-up slides upperarm_twist_01 by 2 cm and upperarm_twist_02 by 1 cm. These offsets sit on top of the twist distribution from the animation or animator rig because they are AdditiveLocal.

**Setup.** None.

**Operators:** `RigUnit_ModifyTransforms`, `RigVMFunction_MathDoubleRemap`

**Scale:** 26 ModifyTransforms on twist bones

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:146-151,154-159 (upperarm/lowerarm twist MTs)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:164-167,182-185 (thigh_twist MTs)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:119-122 (calf_twist MTs)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:19-23 (upperarm_twist_01/02 from shoulder-up)`

#### UE-spine-squetch — Spine squash/stretch curve from chain length ratio (Squetch)

*important* · assets: CR_Zebra_Deform, SKM_Zebra

RigUnit_ChainInfo runs with Items = [spine_05, spine_04, spine_03, spine_02, spine_01] (a Constant array), Param=0, bCalculateStretch=True and bInitial=False. ChainStretchFactor = (sum of current segment lengths) / (sum of initial segment lengths). Subtract(B=1) gives stretch-1, which SetCurveValue_35 writes to curve 'Squetch'. The value is positive when stretched and negative when compressed, and it drives the 'Squetch' morph target. The pure ChainInfo node is pulled lazily on the execution path after head_up.

**Setup.** None. Comment box: Spine Squetch. The animation stores the curve as lower-case 'squetch'; UE curve names are case-insensitive FNames.

**Operators:** `RigUnit_ChainInfo`, `DISPATCH_RigVMDispatch_Constant`, `RigVMFunction_MathDoubleSub`, `RigUnit_SetCurveValue`

**Scale:** 1 chain, 1 curve

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:175-178 (ChainInfo bCalculateStretch=True; ItemArray; SetCurveValue_35 Curve=Squetch; Subtract B=1)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:535-538 (ItemArray.Value -> ChainInfo.Items; ChainStretchFactor -> Subtract.A)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/regen.py:1666 ItemArray.Value ((spine_05),(spine_04),(spine_03),(spine_02),(spine_01))`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Highlevel/Hierarchy/RigUnit_ChainInfo.cpp:144-146 (ChainStretchFactor = ChainLength / InitialChainLength)`

#### UE1-layout-limb-twist — LimbTwist module layout (auto-found twist bones)

*important* · assets: MR_Zebra, MR_FN_Biped

Each twist module (Start, End, Parent=Body/Body) finds twist bones by 'Twist Search String' ('upperarm|twist', 'lowerarm|twist', 'thigh|twist' or 'calf|twist'). For each bone found it creates one offset control 'Twist N' (shape Default, YZX, mirror metadata) under 'Twist N Null' under 'Twist Parent Null' (under Body/Body). Twist distribution weights come from 'Twist Weights' per bone; 'Twist Reverse' is used on lower segments; Twist Axis is (-1, 0, 0) on R and leg-upper L. Display names 'Offset 1'/'Offset 2' are applied to the first two controls where configured (arms and R legs in Zebra).

Zebra has 4 twist bones per segment, giving 4 controls. Manny has 2, giving 2 controls.

**Setup.** 8 modules with 4 controls each in Zebra (32 total). 2 each in Biped (16).

**Operators:** `CRM_FN_LimbTwist`

**Scale:** 8 modules.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1997 "CONTROL Arm Upper Twist L/Twist 1 parents=['NULL:Arm Upper Twist L/Twist 1 Null'] ... display_name=Offset 1"`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/asset.t3d: Arm Lower Twist L 'Twist Weights(0)=0.750000' ... 'Twist Reverse=True'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/hierarchy.txt: 'BONE upperarm_twist_04_l'`

#### UE2-spine-distribute-rotation — Distribute Rotation (propagate part of chest FK swing to spine base)

*important* · assets: CRM_FN_Spine

Forwards step A.C:
1. e = ToEuler(End FK local rotation, order ZYX).
2. s = float channel 'Distribute Rotation' on End FK.
3. q = FromEuler((s*e.X, s*e.Y, 0), ZYX).
4. End IK Null local rotation = MakeAbsoluteQuat(Local = q, Parent = End IK Null initial local rotation).

End IK Null sits at the spine start position under Mid FK, and End FK's space null is its child. So a fraction s of the chest FK's X/Y rotation is applied again about the spine base, which carries End FK, End IK and Mid IK with it.

Backwards sets the channel to 0.

**Setup.** 'Distribute Rotation': SCALE_FLOAT channel on End FK, range [0,1] limited. Its initial value comes from public 'Default Distribute Rotation' (0).

**Operators:** `RigUnit_GetFloatAnimationChannel`, `RigVMFunction_MathQuaternionToEuler`, `RigVMFunction_MathVectorScale`, `RigVMFunction_MathQuaternionFromEuler`, `RigVMFunction_MathQuaternionMakeAbsolute`, `RigUnit_SetRotation`, `RigUnit_SetFloatAnimationChannel`

**Scale:** 1 per spine/neck

**Evidence:** `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:13 'Channel=Distribute Rotation'`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/regen.py 'RigVMFunction_MathQuaternionToEuler.RotationOrder', 'ZYX'`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:483 'Name=Distribute Rotation'`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:1287 'Set Float Channel | Channel=Distribute Rotation; ... Value=0.000'`

#### UE2-spine-mid-blend — Mid IK null blend between Start driver and End IK

*important* · assets: CRM_FN_Spine

'Solve Mid Control Null', run in forwards and backwards. Let b = scale-float channel 'Mid Blend' (on Mid IK, default 0.65).

ParentConstraint(Child = Mid IK Null, bMaintainOffset = true, Filter all, Interpolation = Average) with parents:
- [Start Driver, weight = clamp(remap(b, 0.5..1 -> 1..0))]
- [End IK, weight = clamp(remap(b, 0..0.5 -> 0..1))]

At b = 0 only the start is used, at b = 1 only the end, at b = 0.5 both equally. The default 0.65 gives weights (0.7, 1).

The Mid IK control rides on this null, so it floats between the ends.

**Setup.** 'Mid Blend': SCALE_FLOAT channel on Mid IK, range [0,1] limited, hosts [Mid FK, End FK, End IK].

**Operators:** `RigUnit_ParentConstraint`, `RigVMFunction_MathDoubleRemap`, `RigUnit_GetFloatAnimationChannelFromItem`

**Scale:** 1 per spine/neck

**Evidence:** `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:1652 'RigUnit_ParentConstraint | bMaintainOffset=True'`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/regen.py 'Remap.SourceMinimum', '0.500000' ... 'Remap.TargetMinimum', '1.000000' / 'Remap_1.SourceMaximum', '0.500000'`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:484 'Name=Mid Blend; InitialValue=0.650000'`

#### UE2-spine-pelvis-local — Pelvis Local and chest-local controls (spine only)

*important* · assets: CRM_FN_Spine

When Is Neck is false:

Construction:
- Control 'Pelvis Local' is created under SecFK[0] with identity local offset and the Local FK shape (shape rotation * Sec FKs Orient Offset).
- SecFK[-1] gets its shape replaced by the Local FK shape: transform = SecShape * inverse(SecShape) * LocalShape, rotation * orient offset.

Forwards step E:
- Bones[0] (pelvis) global = PelvisLocal global with rotation * inverse(Sec FKs Orient Offset); this propagates.
- Then Bones[1] global = SecFK[1] global * inverse(offset), which re-imposes spine_01 so the pelvis-local rotation does not carry the spine.

Backwards: Pelvis Local local = identity (only after the Sec FK match branch).

**Setup.** Shape: 'Local FKs Control Shapes' (Hexagon_Thick, pink).

**Operators:** `RigUnit_HierarchyAddControlTransform`, `RigUnit_HierarchySetShapeSettings`, `RigUnit_SetTransform`

**Scale:** 1 per spine

**Evidence:** `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:626 'Name=Pelvis Local'`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/regen.py 'At_4.Index', '0' / 'At_5.Index', '1' (forward Set Transform / Set Transform_4)`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt 'CONTROL Spine/Pelvis Local parents=['CONTROL:Spine/Pelvis Sec FK']'`

#### UE4-blend-twist — Blend Twist (library version; LimbTwist carries a local copy)

*important* · assets: CRFL_Module_v001, CRM_FN_LimbTwist

Inputs: Driver, Drivens[], Weights[] (parallel), TwistAxis, Reverse and Debug Axis. Output: Twist Ouput. (A) twist = GetNodeTwistValue(Driver, TwistAxis). (B) For each driven i: SetRotation(driven, LocalSpace, Slerp(identity, Reverse ? inverse(twist) : twist, Weights[i])). Then OffsetTransformForItem(driven, Rotation = the driven's initial local rotation), which post-multiplies (new global = Offset * previous global), so final local rotation = Slerp(...) * q_initLocal. (C) DrawAxis(Drivens, Enable=Debug Axis, Scale 10, Thickness 0.2). A second For_Each with angle x flip-multiplier logic (ToAxisAndAngle/FromAxisAndAngle) has no input exec link and is dead code.

**Setup.** The drivens are the default parents of the twist controls (from Get Array Parents); per-twist-bone weights come from module configuration.

**Operators:** `FUNC Get Node Twist Value`, `RigVMFunction_MathQuaternionSlerp`, `RigVMFunction_MathQuaternionInverse`, `DISPATCH_RigVMDispatch_SelectInt32`, `RigVMFunction_MathBoolToInteger`, `RigUnit_SetRotation (local)`, `RigUnit_OffsetTransformForItem`, `RigUnit_GetTransform (initial local)`, `FUNC Draw Axis`

**Scale:** The CRFL version has no callers. The LimbTwist-local 'Blend Twist' is called twice (forward and backward).

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Module_v001/graphs.txt:20-155 'Blend Twist' (comments 'Connect the twist weights to drivens', 'Restore the initial rotations')`; `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Hierarchy/RigUnit_OffsetTransform.cpp:33-44 (MakeAbsolute(Offset, Previous) then SetTransform)`; `ue/<dump>/FortniteRigs__Modules__Biped__LimbTwist__CRM_FN_LimbTwist/graphs.txt:455 'Draw Axis' inside 'RigVMFunctionLibrary.Blend Twist' (local copy)`

#### UE8-limbtwist-translate-weights — LimbTwist 'Compute Translate Weights' + 'Blend Translate' (translation distribution)

*important* · assets: CRM_FN_LimbTwist, MR_Zebra, MR_FN_Biped

Compute Translate Weights (construction). Clear the local 'Weights Array'. For each twist bone i, using initial global translations (bInitial=true): w_i = |P_i - P_end| / |P_start - P_end|. A twist bone near the start bone gets w close to 1, and one near the end bone gets w close to 0. Output: 'Translate Weights' (TArray<double>). Blend Translate (forward): for each driven i, run RigUnit_PositionConstraintLocalSpaceOffset with Child=driven_i, Parents=[Start Bone (weight w_i), End Bone (weight 1-w_i)], bMaintainOffset=true, and all axes. Each twist bone thus stays on the segment between the two limb joints, keeping its rest-pose ratio, while the joints translate or stretch.

**Setup.** none (automatic); operates on the module's twist bones

**Operators:** `RigUnit_GetTransform (initial)`, `RigVMFunction_MathVectorDistance`, `RigVMFunction_MathDoubleDiv`, `RigUnit_PositionConstraintLocalSpaceOffset`, `DISPATCH_RigVMDispatch_ArrayIterator`

**Scale:** 8 LimbTwist modules in MR_Zebra

**Evidence:** `<dump>/FortniteRigs__Modules__Biped__LimbTwist__CRM_FN_LimbTwist/graphs.txt:535-576 (Compute Translate Weights), 577-604 (Blend Translate)`; `<dump>/FortniteRigs__Modules__Biped__LimbTwist__CRM_FN_LimbTwist/summary.json :: functions_used 'Compute Translate Weights': 1, 'Blend Translate': 2`

#### UE4-blend-position — Blend Position (two-parent weighted position constraint, unused)

*nice-to-have* · assets: CRFL_Module_v001

Inputs: Start, End, items[] and Weights[]. For each item i it calls PositionConstraintLocalSpaceOffset(Child=item, Parents=[{Start, w=Weights[i]}, {End, w=1-Weights[i]}], bMaintainOffset=true, XYZ filter, Weight 1). This distributes positions between two endpoints while keeping the initial offsets.

**Operators:** `RigUnit_PositionConstraintLocalSpaceOffset`, `RigVMFunction_MathDoubleSub (1-w)`, `DISPATCH_RigVMDispatch_ArrayIterator/GetAtIndex`

**Scale:** No callers; LimbTwist uses a local 'Blend Translate'.

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Module_v001/graphs.txt:257-287 'Blend Position' (Subtract 'A=1.000000')`; `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/Highlevel/Hierarchy/RigUnit_TransformConstraint.h:358`

#### UE8-spine-sliding-proxy-unused — Spine 'Construct/Forward Sliding Proxy' (defined, unused) and RigUnit_PositionFromControlRigSpline

*nice-to-have* · assets: CRM_FN_Spine

Not referenced by any graph. For a port it is optional; the design is recorded here. Construct: spawn '<Proxy Name> Null' under 'Proxy Parent Control' (rotation from Driven Control, translation and scale from 'Proxy Snap To', initial values). Spawn a proxy transform control '<Proxy Name>' under that null and set its driven list to [Driven Control]. Spawn a buffer null '<Proxy Name> Bfr' with bool metadata IsSet=false. Add a bool channel 'Movable Pivot Vis' (default true) on the driven control and a float channel 'Proxy Pivot Slide' (default 0.5, range 0-1, limited) on the proxy. Forward: if the proxy or its slide channel is being translated (RigUnit_IsInteracting), set the buffer's global transform to the driven control's and set IsSet=true. While rotating, set the buffer once when IsSet is false. When not interacting, set IsSet=false, move the proxy's parent to translation = PositionFromControlRigSpline(Spline, U = slide channel) with rotation = the driven control's global rotation, and reset the proxy's local rotation to identity. RigUnit_PositionFromControlRigSpline returns Spline.PositionAtParam(U), with U clamped to 0-1 by metadata.

**Setup.** (unused) proxy control plus 'Proxy Pivot Slide' float and 'Movable Pivot Vis' bool

**Operators:** `RigUnit_PositionFromControlRigSpline`, `RigUnit_IsInteracting`, `RigUnit_SetControlDrivenList`, `RigUnit_HierarchyAddAnimationChannelBool/Float`, `RigUnit_SetRotation`, `RigUnit_HierarchyGetParent`

**Scale:** 0 uses

**Evidence:** `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:1771-1853, 1854-1971`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/summary.json :: functions_used (no 'Construct Sliding Proxy'/'Forward Sliding Proxy')`; `<UE>/Plugins/Animation/ControlRigSpline/Source/ControlRigSpline/Public/ControlRigSplineUnits.h:185-213`; `<UE>/Plugins/Animation/ControlRigSpline/Source/ControlRigSpline/Private/ControlRigSplineUnits.cpp:134-148`; `<dump>/units_used.json :: 'RigUnit_PositionFromControlRigSpline': 1 (CRM_FN_Spine only)`

### D8 Foot

#### UE1-layout-foot — Foot module layout (reverse-foot pivot stack)

*core* · assets: MR_Zebra, MR_FN_Biped

Pivot null stack under Leg IK Gimbal: 'Foot L/IK Foot Space' > Toe Tip Pivot > Heel Pivot > Ball Pivot > Toe Tip Rocker Pivot > Heel Rocker Pivot > Inner Pivot > Outer Pivot > Heel Lift > IK. 'Ball IK' and 'Toe IK' nulls sit under Outer Pivot.

Pivot positions come from the socket nulls foot_l_toe_tip/heel/inner/outer (connectors) plus config offsets (Zebra: Inner Bank (-5, 0, -3), Outer Bank (6, 0, -3), Toe Tip (0, 6.8, -2.5), Heel (0, -20, -3)).

Controls (12):
- Toe Tip and Heel: Sphere_Solid, ZYX.
- Ball: HalfCircle_Thin, ZYX.
- Ball IK: Box_Thick.
- Foot Rocker: Sphere_Thick, YXZ, with channels Rocker Blend 1 [0, 1] and Rocker Ball Rotation 45.
- Toes FK (under bone foot_l) and Toes IK (under 'Toes IK Null'): HalfCircle_Thick.
- Footprint Display: proxy, RoundedSquare_Solid.
- Footprint Vis and Foot Pivot Control Vis channels (on Leg IK).

The foot reads the Leg module metadata (IK Null/Control) through the Parent namespace. Config 'Negative Side' True on the R side.

**Setup.** 12 controls per foot: 7 transforms, 1 proxy, 4 channels.

**Operators:** `CRM_FN_Foot`

**Scale:** 2 modules in Zebra and Biped. The Biped template leaves the pivot connectors unconnected.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:2062 "CONTROL Foot L/Foot Rocker parents=['NULL:Foot L/IK Foot Space']"`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:2073 'CONTROL Foot L/Footprint Display ... PROXY_CONTROL'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/asset.t3d: Foot L 'Heel Pivot Offset=(X=0.000000,Y=-20.000000,Z=-3.000000)'`

#### UE3-foot-ball-toe-solve — Foot/ball placement: FABRIK plus local-space aim for the toe, Toes IK/FK

*core* · assets: CRM_FN_Foot

Forward (after the leg solve).

IK mode:
1. FABRIKItemArray([Foot Joint, Ball Joint], effector = Ball IK null global, precision 1e-4, 10 iterations, bSetEffectorTransform = false, propagate). This rotates the foot so the ball reaches the Ball IK null.
2. AimConstraintLocalSpaceOffset(child Ball Joint, parent Toe IK null weight 1, maintain offset, AimAxis X, UpAxis Z, WorldUp = (0,0,1) location in Toe IK null space, filter XYZ). This rotates the toe toward the toe-tip null.
3. Toes IK Null (identity child of the foot) = project(Toes IK Null, Ball Joint, Ball Joint).
4. Ball Joint global = Toes IK control global, for toe offsets on top of the aim.

FK mode: Ball Joint global = Toes FK control global. The 'Toes FK' control sits under Foot Joint with offset = ball initial; 'Toes IK' sits under 'Toes IK Null'. Toes IK is visible in IK mode, Toes FK in FK mode.

If Toe Joints are connected: 'FK <i>' controls under Toes FK at each toe's initial transform, and the toe joints follow them.

**Setup.** 'Toes FK' and 'Toes IK' are HalfCircle_Thick shapes (Tube_Pins_Hollow node default); optional 'FK <i>' per toe.

**Operators:** `RigUnit_FABRIKItemArray`, `RigUnit_AimConstraintLocalSpaceOffset`, `RigUnit_ProjectTransformToNewParent`, `RigUnit_SetTransform`, `RigUnit_HierarchyAddControlTransform`

**Scale:** per foot per frame

**Evidence:** `<dump>/.../CRM_FN_Foot/graphs.txt:26 'Basic FABRIK | Precision=0.000100; ... MaxIterations=10; bSetEffectorTransform=false'`; `<dump>/.../CRM_FN_Foot/graphs.txt:23 'Aim Constraint | bMaintainOffset=True; ... AimAxis=(X=1..); UpAxis=(... Z=1)'`; `<dump>/.../CRM_FN_Foot/graphs.txt:141-146,177,204-210 chain`; `<dump>/.../CRM_FN_Foot/graphs.txt:360,547,557 'Toes FK' / 'Toes IK' / 'Toes IK Null'`

#### UE3-foot-controls — Foot pivot/roll controls, channels and visibility

*core* · assets: CRM_FN_Foot

Controls. Delta metadata = the control's initial global transform relative to the joint named in brackets.
- 'Toe Tip': under IK Foot Space at the toe-tip pivot; rotation order ZYX; Pivot shape; Delta metadata vs Ball Joint.
- 'Heel': under Toe Tip Pivot at the heel pivot; ZYX; Delta vs Foot Joint.
- 'Ball': under Heel Pivot; offset = Ball Pivot null with Z = 0; ZYX; HalfCircle_Thin; uniform scale = BallIK.scale.X * 1.2.
- 'Foot Rocker': under IK Foot Space; rotation from Ball Pivot, translation at the heel; YXZ; Sphere_Thick; Delta vs Foot Joint.
- 'Ball IK': under Outer Pivot; translation from the ball, rotation from the foot IK null; YZX; Box_Thick.

Channels:
- 'Rocker Blend' (float 0..1, initial 1) and 'Rocker Ball Rotation' (float, initial 45, range 0..90, limits off), both on Foot Rocker.
- 'Foot Pivot Control Vis' (bool, initial false) on the leg IK control, or on Foot Rocker if that is missing.

IK Controls = [Toe Tip, Heel, Ball, Ball IK, Foot Rocker]; Pivot Controls = [Toe Tip, Heel, Ball].

Visibility: pivot controls = IK && PivotVis; other IK controls = IK.

**Setup.** 5 foot IK controls plus 3 channels. Pivot controls are hidden until 'Foot Pivot Control Vis' is on.

**Operators:** `RigUnit_HierarchyAddControlTransform`, `RigUnit_HierarchyAddAnimationChannelFloat`, `RigUnit_HierarchyAddAnimationChannelBool`, `DISPATCH_RigDispatch_SetMetadata`, `RigUnit_SetControlVisibility`

**Scale:** per foot

**Evidence:** `<dump>/.../CRM_FN_Foot/graphs.txt:272,275,506,373,228 'Name=Toe Tip' / 'Heel' / 'Ball' / 'Foot Rocker' / 'Ball IK'`; `<dump>/.../CRM_FN_Foot/graphs.txt:384,386 'Name=Rocker Blend; InitialValue=1' / 'Name=Rocker Ball Rotation; InitialValue=45.000000'`; `<dump>/.../CRM_FN_Foot/graphs.txt:425 'Name=Foot Pivot Control Vis'`; `<dump>/.../CRM_FN_Foot/graphs.txt:346,353,421 'Set Transform Metadata | Name=Delta'`; `<dump>/.../CRM_FN_Foot/graphs.txt:129-176 pivot visibility`

#### UE3-foot-pivot-hierarchy — Reverse-foot pivot null stack and footprint space

*core* · assets: CRM_FN_Foot

Construction:
- Target IK Orientation = the leg IK control's initial global rotation.
- Footprint Space Transform = (rot = Target IK Orientation, t = (Ball.x, Ball.y, 0), s = ball scale): ball position projected to the ground.

Null chain, each pivot position = connected pivot's initial translation, or FootprintSpace * offset, with rotation = Target IK Orientation:
'IK Foot Space' (under the leg IK Driver/gimbal, identity) > 'Toe Tip Pivot' > 'Heel Pivot' > 'Ball Pivot' (= footprint transform) > 'Toe Tip Rocker Pivot' (at the toe tip) > 'Heel Rocker Pivot' (at the heel) > 'Inner Pivot' > 'Outer Pivot' > { 'Heel Lift' (at the ball), 'Ball IK' null (ball), 'Toe IK' null (toe tip), 'Ball IK' control }.

'IK' null under Heel Lift is placed at the leg's IK null initial transform. It is the foot's output effector. Target Effector = leg 'IK Null'.

**Setup.** The pivot controls are listed in the next feature.

**Operators:** `RigUnit_HierarchyAddNull`, `RigVMFunction_MathTransformMakeAbsolute`, `DISPATCH_RigVMDispatch_If`, `RigUnit_ResolveConnector.bIsConnected`

**Scale:** 12 nulls per foot

**Evidence:** `<dump>/.../CRM_FN_Foot/graphs.txt:563 COMMENT 'Footprint Space Transform (Ball Bone Position projectd to floor and oriented to ik space'`; `<dump>/.../CRM_FN_Foot/graphs.txt:866-893 If_6/If_4/If_5/If_3 connector-or-offset`; `<dump>/.../CRM_FN_Foot/graphs.txt:232,241 'Spawn Null | Name=IK' / COMMENT 'This Null will be used to Parent Constraint the Target Effector'`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/runtime_hierarchy.txt 'NULL Foot L/Heel Lift <- NULL:Foot L/Outer Pivot' / 'NULL Foot L/IK <- NULL:Foot L/Heel Lift'`

#### UE3-foot-roll-bank — Foot roll (heel/ball/toe rocker), banking and heel lift (Set Foot Pivots)

*core* · assets: CRM_FN_Foot

Runs in Pre Forwards Solve when the leg is in IK. R = Foot Rocker, B = Rocker Blend channel, RBR = Rocker Ball Rotation channel.

A) Free pivots and rocker:
- ToeTipPivot local = RAFP(its initial local, Toe Tip control local).
- HeelPivot local = RAFP(initial, Heel control local).
- BallPivot local = RAFP(initial, Ball control local).
- ToeTipRocker local rotation = Euler XYZ (X = Remap(R, [-360, S] -> [-360, 0], clamped), 0, 0), where S = -RBR*B. The toe rotates only once the roll passes -RBR*B.
- HeelRocker local rotation = Euler X = clamp(R, 0, 360). Positive roll pivots on the heel.

B) Bank:
- Inner Pivot local rotation Y = NegSide ? clamp(Bank, 0, 90) : clamp(Bank, -90, 0).
- Outer Pivot Y = NegSide ? clamp(Bank, -90, 0) : clamp(Bank, 0, 90).

C) Heel Lift local = Euler XYZ(HeelBend + clamp(R, -RBR, 0)*B, HeelTwist, HeelSide) * initial local. Rolling between 0 and -RBR lifts the heel about the ball.

D) Leg Target Effector global = the foot's 'IK' null global, so the leg IK follows the whole pivot stack.

**Setup.** Foot Rocker rotation: X = roll, Y = bank. 'Rocker Ball Rotation' (degrees) and 'Rocker Blend' shape the roll.

**Operators:** `RigVMFunction_MathQuaternionFromEuler`, `RigVMFunction_MathDoubleClamp`, `RigVMFunction_MathDoubleRemap`, `RigVMFunction_MathDoubleNegate`, `RigUnit_SetRotation(Local)`, `RigUnit_SetTransform`, `FUNC Rotate Aroound Free Pivot`, `FUNC Set Null Local Transform`

**Scale:** per foot per frame

**Evidence:** `<dump>/.../CRM_FN_Foot/graphs.txt:1131-1320 'Set Foot Pivots'`; `<dump>/.../CRM_FN_Foot/graphs.txt:1213-1214 'Remap_2 | SourceMinimum=-360; TargetMinimum=-360; TargetMaximum=0; bClamp=true' / 'Remap_3 | SourceMinimum=1; SourceMaximum=0'`; `<dump>/.../CRM_FN_Foot/graphs.txt:1142,1153,1160,1162 bank clamps`; `<dump>/.../CRM_FN_Foot/graphs.txt:1135 'Get Transform | Item=(Type=Null,Name="IK")' -> Set Transform Target Effector`; `<dump>/.../CRM_FN_Foot/graphs.txt:1115-1129 'Set Null Local Transform' (Local * initial local)`

#### UE3-free-pivot — Rotate Around Free Pivot

*core* · assets: CRM_FN_Foot

RAFP(Input, Pivot, Rot) with Pivot = Rot = the control's local transform (t_p, R_p).
1. M = inverse(Pivot) * (R_p, 0).
2. P = (Rot.rot, t_p, Pivot.scale).
3. abs = M * P, which maps x to Rot(x - t_p) + t_p.
4. Output = abs * Input (FTransform order: abs is applied first, then Input).

Translating the control only moves the pivot point, with no motion. Rotating it rotates the pivot null about the control's translated position, in the null's own initial-local frame. The control offset equals the null's initial transform, so the frames coincide.

**Setup.** Toe Tip, Heel and Ball are translate-to-place, rotate-to-pivot controls.

**Operators:** `RigVMFunction_MathTransformMakeRelative`, `RigVMFunction_MathTransformMakeAbsolute`, `RigVMFunction_MathTransformMul`, `DISPATCH_RigVMDispatch_MakeStruct`

**Scale:** 3 per foot

**Evidence:** `<dump>/.../CRM_FN_Foot/graphs.txt:1418-1446 'Rotate Aroound Free Pivot'`; `<UE>/Plugins/Runtime/RigVM/Source/RigVM/Private/RigVMFunctions/Math/RigVMFunction_MathTransform.cpp:66-76`

#### UE-foot-pivot-sockets — Foot-roll pivot sockets on the ball bones

*important* · assets: SK_Zebra, SKM_Zebra

Eight mesh/skeleton sockets define the foot-roll and bank pivots used by the animator rig's foot module, relative to ball_l/r:
- foot_l_inner (-8,0,5.5)
- foot_l_outer (-8,0,-7.75)
- foot_l_heel (-18.8,1,-1)
- foot_l_toe_tip (0,0,0)
- foot_r_inner (8,0,-5.5)
- foot_r_outer (8,0,7.75)
- foot_r_heel (18,-1,1); unlike the left heel, the right heel is not an exact mirror (18 vs 18.8).
- foot_r_toe_tip (0,0,0)

**Setup.** Socket naming: foot_<side>_<inner|outer|heel|toe_tip>.

**Operators:** `SkeletalMeshSocket`

**Scale:** 8 sockets

**Evidence:** `<dump>/Game__Assets__Zebra__Meshes__SK_Zebra/asset.t3d SocketName="foot_r_heel" BoneName="ball_r" RelativeLocation=(X=18.000000,Y=-1.000000,Z=1.000000)`; `<dump>/Game__Assets__Zebra__Meshes__SK_Zebra/asset.t3d SocketName="foot_l_heel" RelativeLocation=(X=-18.800000,Y=1.000000,Z=-1.000000)`; `<dump>/Game__Assets__Zebra__Meshes__SKM_Zebra/summary.json sockets`

#### UE8-foot-pivot-socket-values — Per-skeleton foot pivot sockets imported as MeshSocket nulls

*important* · assets: SK_Zebra, SK_ZebraHi, SK_Manny, MR_Zebra, MR_FN_Biped

Modular rigs import skeleton sockets as static NULL elements parented to the ball bones, tagged 'MeshSocket' with metadata 'Tags:NAME_ARRAY': foot_{l,r}_{inner, outer, heel, toe_tip}. Zebra socket offsets (relative to ball_l / ball_r): foot_l_inner (-8,0,5.5), foot_l_outer (-8,0,-7.75), foot_l_heel (-18.8,1,-1), foot_l_toe_tip (0,0,0); foot_r_inner (8,0,-5.5), foot_r_outer (8,0,7.75), foot_r_heel (18,-1,1), foot_r_toe_tip (0,0,0). The Zebra toe-tip pivot therefore sits exactly on the ball joint (null foot_l_toe_tip global (10.32,14.4,-0.097) equals ball_l), and the heel is asymmetric (-18.8 vs 18), giving null foot_l_heel (8.50,-4.36,-0.21) vs foot_r_heel (-8.62,-3.57,-0.25). MR_Zebra connects these nulls to the Foot pivot connectors. MR_FN_Biped has the Manny equivalents (feature UE8-manny-reference-mesh) but leaves them unconnected. SK_Monster has no sockets.

**Setup.** Foot module pivot controls (Heel, Toe Tip, bank pivots) are placed at these nulls.

**Operators:** `USkeletalMeshSocket`, `hierarchy NULL import with tag MeshSocket`

**Scale:** 8 sockets per biped skeleton

**Evidence:** `<dump>/Game__Assets__Zebra__Meshes__SK_Zebra/asset.t3d :: SocketName="foot_l_heel" RelativeLocation=(X=-18.800000,Y=1.000000,Z=-1.000000); foot_l_toe_tip has no RelativeLocation line`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/hierarchy.txt :: 'NULL foot_l_toe_tip parents=[\'BONE:ball_l\'] init_global=T(10.32,14.4,-0.09704)'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/modular_rig_model.txt :: 'Name="Foot L/Toe Tip Pivot"),Targets=((Type=Null,Name="foot_l_toe_tip"'`; `<dump>/Game__Assets__Monster__Meshes__SKM_Monster/summary.json :: 'sockets': []`

#### UE8-foot-reset-rig-values — Foot 'Reset Foot Rig Values' during Match IK (FK-to-IK snap)

*important* · assets: CRM_FN_Foot, MR_Zebra, MR_FN_Biped

'Reset Foot Rig Values' sets the module's cached float state variables to 0: Foot Rocker, Foot Bank, Ball Pivot, Ball Pivot Heel Bend, Ball Pivot Heel Twist and Ball Pivot Heel Side. These are the values 'Set Foot Values' and 'Set Foot Pivots' read to rebuild the roll/bank/pivot nulls. It is called only from the local 'Match IK' function (fired by the IkFk2Bones To IK / Key Controls flow). Match IK does the following. Sequence A: set the local transforms of Toe Tip Control, Heel Control, Ball IK Control, Foot Rocker Control and Ball Pivot Control to identity (bPropagateToChildren=false), then Toes IK Null to identity (propagate=true); then Reset Foot Rig Values; then rerun 'Set Foot Pivots' with the zeroed values ('Rerun Foot Pivot logic with zeroed out values'). Sequence B: set Toes IK Control's global transform to Ball Joint's global transform. After matching, the IK foot therefore has no residual roll, bank or pivot offset, and the IK toe matches the FK ball.

**Setup.** Affects Foot L/R Heel, Toe Tip, Ball IK, Foot Rocker, Ball (pivot) controls and the Toes IK control.

**Operators:** `RigUnit_SetTransform`, `RigVMFunction_Sequence`, `VariableNode setters`

**Scale:** 1 call per Foot module (2 in MR_Zebra)

**Evidence:** `<dump>/FortniteRigs__Modules__Biped__Foot__CRM_FN_Foot/graphs.txt:1339-1368 (Reset Foot Rig Values), 1464-1512 (Match IK)`; `<dump>/FortniteRigs__Modules__Biped__Foot__CRM_FN_Foot/regen.py:26-30 (member float vars), 2135, 2143-2144`; `<dump>/FortniteRigs__Modules__Biped__Foot__CRM_FN_Foot/summary.json :: functions_used 'Reset Foot Rig Values': 1`

### D9 Constraints

#### UE-modifytransforms-op — Weighted additive-local helper-joint offset (Modify Transforms)

*core* · assets: CR_Zebra_Deform, CR_Monster_Deform

Pins: ItemToModify[] {Item, Transform}, Weight, WeightMinimum, WeightMaximum, Mode.
1. The unit returns early if Weight <= min+eps or min == max.
2. T = clamp((Weight-min)/(max-min), 0, 1).
3. In AdditiveLocal mode, when T<1 the offset is blended as LerpTransform(Identity, Offset, T): translation and scale lerp, rotation slerps.
4. The new local transform is Offset * CurrentLocal (UE order), so the offset applies in the bone's own local frame. If the local transform is dirty, the global one is used instead.
5. The result is set with children propagation.
Every instance here holds exactly one item with scale 1, WeightMin 0, WeightMax 1 and Mode AdditiveLocal, and its Weight is linked to a reader output or a Remap. Offsets on the same bone accumulate in execution order.

**Setup.** None. Offsets are node pin defaults in cm and quaternions.

**Operators:** `RigUnit_ModifyTransforms`

**Scale:** 86 nodes (82 Zebra, all single-item; 4 Monster)

**Evidence:** `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Highlevel/Hierarchy/RigUnit_ModifyTransforms.cpp:9-111`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/Highlevel/Hierarchy/RigUnit_ModifyTransforms.h:11-131`; `<UE>/Plugins/Runtime/RigVM/Source/RigVM/Private/RigVMFunctions/Math/RigVMMathLibrary.cpp:253-260 (LerpTransform)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:6 ModifyTransforms ... Mode=AdditiveLocal`

#### UE1-layout-body — Body module layout (orbit, body, aim, movable pivot)

*core* · assets: MR_Zebra, MR_Monster, MR_FN_Biped

Controls:
- Body/Body Orbit: Hexagon_Thin, parent root/Local.
- Body/Body: Hexagon_Thin, child of Orbit, drives bone pelvis ('Drive Body Joint' True). Space 'Aim' = Body/Body Aim. Metadata 'Body Delta Transform'.
- Body/Body Aim: RoundedTrapeze_ArrowTip_Thick, child of Orbit, offset (0, 0, 38) along 'Body Aim Axis' (0, 0, 1).
- Body/Body Movable Pivot: proxy, driving Orbit.

Channels: Body Orbit Vis (hosted on Body); Movable Pivot Vis (hosted on Body); Body Aim Vis, Aim Weight [0, 1] and Aim Twist [0, 1] (hosted on Body and Body Orbit).

Nulls: 'Body Movable Pivot Null' (under root/Local) and 'Body Movable Pivot Bfr' (IsSet metadata).

Connectors: Parent -> root/Local (primary); Body -> pelvis; Right/Left Hip -> thigh_r/thigh_l. Body/Body is the parent or space target for Spine, the twist modules, the limb IK/FK spaces and the clavicle orient space. Config: Control Scale 0.8.

**Setup.** 9 controls: 3 transforms, 1 proxy, 5 channels.

**Operators:** `CRM_FN_Body`, `RigUnit_AimConstraintLocalSpaceOffset (module internals)`

**Scale:** Same in Zebra, Monster and Biped.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1914 'CONTROL Body/Body ... AvailableSpaces=((Key=(Type=Control,Name="Body/Body Aim"),Label="Aim"))' metadata 'Body/Body Delta Transform:TRANSFORM'`; `<dump>/FortniteRigs__Modules__FkSolves__Body__CRM_FN_Body/summary.json: 'Body Aim Control Offset:FVector=(X=0.000000,Y=0.000000,Z=38.000000)'`

#### UE2-body-drive-pelvis — Pelvis driven by Body with maintained offset (Project to new Parent)

*core* · assets: CRM_FN_Body

Forwards step D runs when 'Drive Body Joint' is true and the Body connector is connected. It sets the global transform of the Body bone (propagate=true) to:

ProjectTransformToNewParent(Child=bone, OldParent=BodyDriver, NewParent=BodyDriver, bChildInitial=true, bOldParentInitial=true, bNewParentInitial=false)

= BoneInit * inverse(DriverInit) * DriverCurrent (UE order child*parent). This is a parent constraint with maintained offset. Body Driver is the Body control, or Body Offset when that control exists.

**Setup.** Public 'Drive Body Joint' (bool, default true).

**Operators:** `RigUnit_ProjectTransformToNewParent`, `RigUnit_SetTransform`, `RigVMFunction_MathBoolAnd`

**Scale:** 1 per rig

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Body__CRM_FN_Body/graphs.txt:11 'bChildInitial=True; bOldParentInitial=True; bNewParentInitial=False'`; `<dump>/FortniteRigs__Modules__FkSolves__Body__CRM_FN_Body/graphs.txt:185-186 'VariableNode_32.Value -> ProjectTransformToNewParent.OldParent/NewParent'`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/Hierarchy/RigUnit_ProjectTransformToNewParent.h:11`

#### UE3-bone-follow-virtual — Offset-preserving follow (ProjectTransformToNewParent) between skeleton and virtual bones

*core* · assets: CRM_FN_IkFk2Bones, CRM_FN_Foot

ProjectTransformToNewParent(Child, OldParent, NewParent, childInitial = T, oldInitial = T, newInitial = F) computes Global = (ChildInit relative to OldParentInit) * NewParentCurrent. This is a maintain-offset parent constraint against the bind pose.

Uses:
- Forward (IkFk2Bones): Bones[i] = project(Bones[i], VB[i], VB[i]).
- Backward: VB[i] = project(VB[i], Bones[i], Bones[i]).
- Foot: toe joints follow FK toe controls. Toes IK Null follows the Ball Joint. FK toe controls get the toe joint transforms on backwards (Set Transform Array).

**Setup.** None.

**Operators:** `RigUnit_ProjectTransformToNewParent`, `RigUnit_SetTransform`, `DISPATCH_RigVMDispatch_ArrayIterator`

**Scale:** 3 bones per limb per frame

**Evidence:** `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Hierarchy/RigUnit_ProjectTransformToNewParent.cpp:10-24`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:80,337-345 COMMENT 'Snap Actual skeleton to Virtual Bones'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:1733-1741 'Snap Virtual Bones to Skeleton'`; `<dump>/.../CRM_FN_Foot/graphs.txt:113-126 toe joints`

#### UE4-rigvm-math — RigVM math functions used by the libraries

*core* · assets: CRFL_Control_v001, CRFL_Hierarchy_v001, CRFL_Math_v001, CRFL_Module_v001

MathTransformMakeRelative: Local = Global.GetRelativeTransform(Parent), normalised. MakeAbsolute: Local * Parent. MathTransformMul: A * B (UE order, A applied first). MathTransformLerp: lerp translation and scale, slerp rotation. MathTransformTransformVector: TransformLocation. MathTransformArrayToSRT splits a transform array. MathIntersectPlane: ray-plane intersection; zero when the ray is parallel; the normal is normalised with a Z fallback. MathVectorSetLength: normalise * Length, zero with a warning for a near-zero input. MathQuaternionSwingTwist: FQuat::ToSwingTwist about a normalised axis; identity for a zero axis. Also used: Slerp, Inverse, Mul, To/FromAxisAndAngle, To/FromEuler (XYZ), MatrixFromVectors/ToTransform, and vector Sub/Add/Cross/Dot/Length/LengthSquared/Distance/Scale/Unit/Negate/Mul/FromDouble, double/int/float arithmetic and comparisons, IsNearlyZero, and ColorAdd.

**Operators:** `RigVMFunction_MathTransformMakeRelative`, `RigVMFunction_MathTransformMul`, `RigVMFunction_MathTransformLerp`, `RigVMFunction_MathTransformTransformVector`, `RigVMFunction_MathTransformArrayToSRT`, `RigVMFunction_MathTransformMake`, `RigVMFunction_MathIntersectPlane`, `RigVMFunction_MathVectorSetLength`, `RigVMFunction_MathQuaternionSwingTwist`, `RigVMFunction_MathQuaternionSlerp`, `RigVMFunction_MathQuaternionInverse`, `RigVMFunction_MathQuaternionToAxisAndAngle`, `RigVMFunction_MathQuaternionFromAxisAndAngle`, `RigVMFunction_MathQuaternionToEuler`, `RigVMFunction_MathQuaternionFromEuler`, `RigVMFunction_MathMatrixFromVectors`, `RigVMFunction_MathMatrixToTransform`, `RigVMFunction_MathVectorCross`, `RigVMFunction_MathVectorDot`, `RigVMFunction_MathVectorDistance`, `RigVMFunction_MathColorAdd`, `RigVMFunction_MathFloatIsNearlyZero`

**Scale:** CRFL_Hierarchy: VectorSub 12, TransformVector 7, VectorCross 5. CRFL_Math: VectorSub 7, VectorUnit 4, VectorScale 4.

**Evidence:** `UE_5.8/Engine/Plugins/Runtime/RigVM/Source/RigVM/Private/RigVMFunctions/Math/RigVMFunction_MathTransform.cpp:66-76, :145-148`; `UE_5.8/Engine/Plugins/Runtime/RigVM/Source/RigVM/Private/RigVMFunctions/Math/RigVMFunction_MathVector.cpp:297-306, :708-720`; `UE_5.8/Engine/Plugins/Runtime/RigVM/Source/RigVM/Private/RigVMFunctions/Math/RigVMFunction_MathQuaternion.cpp:219-229`; `UE_5.8/Engine/Plugins/Runtime/RigVM/Source/RigVM/Public/RigVMFunctions/Math/RigVMFunction_MathVector.h:1141; RigVMFunction_MathMatrix.h:216`

#### UE4-unit-aim-bone-math — Engine unit: AimBoneMath (two-axis aim math)

*core* · assets: CRFL_Hierarchy_v001

Inputs: InputTransform, Primary{Weight, Axis, Target, Kind Location/Direction, Space}, Secondary{...} and Weight. If a target's Space is valid, the target is first transformed from that space (TransformPosition or TransformVector, both ignoring scale). Primary: for a Location target, dir = Target - Result.pos; the rotation is then premultiplied by FindQuatBetweenVectors(axisWorld, lerp(axisWorld, dir, w)). Secondary: the target direction is projected onto the plane perpendicular to the already-aimed primary axis and aligned the same way. The output Result keeps the input translation and scale. Debug drawing is available when enabled.

**Operators:** `RigUnit_AimBoneMath`

**Scale:** CRFL_Hierarchy 10 nodes (IK-plane bones and auto pole vectors).

**Evidence:** `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/Highlevel/Hierarchy/RigUnit_AimBone.h:139`; `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Highlevel/Hierarchy/RigUnit_AimBone.cpp:10-140`

#### UE4-unit-transform-get-set — Engine units: transform get/set/offset

*core* · assets: CRFL_Control_v001, CRFL_Hierarchy_v001, CRFL_Math_v001, CRFL_Module_v001, CRFL_Debug_v001

RigUnit_GetTransform(Item, Space Global/Local, bInitial) reads a transform. RigUnit_GetTransformItemArray does the same for an array. RigUnit_SetTransform and SetRotation(Item, Space, bInitial, Value, Weight, bPropagateToChildren) write a transform or rotation; with bInitial=true they write the initial (bind) pose, which the construction helpers use to 'zero' controls. RigUnit_OffsetTransformForItem(Item, OffsetTransform, Weight) reads the current transform (local if global is dirty), computes new = Offset * Previous (post-multiplied) and sets it back.

**Operators:** `RigUnit_GetTransform`, `RigUnit_GetTransformItemArray`, `RigUnit_SetTransform`, `RigUnit_SetRotation`, `RigUnit_OffsetTransformForItem`

**Scale:** GetTransform: Hierarchy 26, Math 10, Control 4, Module 4, Debug 1. SetTransform: Hierarchy 8. SetRotation: Hierarchy 2, Module 2, Control 1.

**Evidence:** `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/Hierarchy/RigUnit_GetTransform.h:14,113`; `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/Hierarchy/RigUnit_SetTransform.h:18,148`; `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Hierarchy/RigUnit_OffsetTransform.cpp:11-44`

#### UE6-eye-aim — Eye aim target with local-space-offset aim constraints

*core* · assets: CRM_Zebra_Face, CRM_Monster_Face

Construction: nulls 'Eye L aim' and 'Eye R aim' are placed under skull at eye position + (0,50,0), i.e. 50 units forward along +Y. 'Eye Aim' is spawned with parent key (Type None, 'Skull'), so it has no parent. Its InitialValue translation is the center of the box enclosing the two aim nulls' initial positions (MathBoxFromArray.Center). SetDefaultParent(Eye Aim -> bone Skull), a Convergence channel (0..1) on Eye Aim, and SetDefaultParent for both aim nulls -> Eye Aim follow. Then StandardFunctionLibrary 'Add Null Above'(Eye Aim, suffix ' Null') inserts 'Eye Aim Null' between skull and Eye Aim. Forward (pin K): AimConstraintLocalSpaceOffset(child = Eye L null, parent = Eye L aim, weight 1, maintain offset, AimAxis +Y, UpAxis +Z). WorldUp is Kind=Direction (0,0,1) in the space of Eye L aim (Zebra). Monster uses Kind=Location with target = Eye L null global position + (0,0,10). The R side is the same.

**Setup.** Eye Aim: Box_Thick, yellow, scale 0.3, parent space Eye Aim Null under skull. Channel Convergence (0..1).

**Operators:** `RigUnit_HierarchyAddNull`, `RigVMFunction_MathVectorAdd`, `RigVMFunction_MathBoxFromArray`, `RigUnit_SetDefaultParent`, `StandardFunctionLibrary Add Null Above`, `RigUnit_AimConstraintLocalSpaceOffset`

**Scale:** 1 control, 3 nulls, 2 aim constraints

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:205 HierarchyAddNull_7 Eye L aim; :215 RigVMFunction_MathBoxFromArray; :216 SpawnControl_10 Eye Aim; :221 SetDefaultParent_1; :228 Add Null Above New Suffix= Null`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:1138 AimConstraintLocalSpaceOffset AimAxis Y UpAxis Z`; `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/graphs.txt:1182 AimConstraintLocalSpaceOffset Kind=Location; :1338 Add_2 B=(0,0,10)`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1374-1378 Eye Aim Null / Eye Aim / Eye L aim par=C:Face/Eye Aim`

#### UE6-lid-skin-constraints — Weighted parent constraints from micro controls to lid skin bones

*core* · assets: CRM_Zebra_Face, CRM_Monster_Face, Lid_Tp_Struct, Lid_Bt_Struct

Pin J runs 8 loops. For each skin bone i in a list: ParentConstraint(child = bone i, maintain offset, all channels, Average) with parents [Lid In, Lid Ot, micro 01, micro 02, micro 03] and weights = struct row i (Value1..Value5), normalized by their sum. Lists: L tp = lid_in_l, lid_tp_01..14_l, lid_ot_l (Monster adds 15), parents Lid In L / Lid Ot L / Lid Tp 01-03 L. L tp base = lid_in_base_l, lid_tp_base_01..14_l, lid_ot_base_l, with the Lid Tp Base 01-03 L controls. L bt = lid_bt_01..13_l (Monster 01..11) with Lid Bt 01-03 L. L bt base = lid_bt_base_01..13_l with Lid Bt Base 01-03 L. The R side is the same. Top lists use 'Lid Tp Struct' and bottom lists use 'Lid Bt Struct'. Zebra tp rows: [1,0,0,0,0], [.775,0,.225,0,0], [.4,0,.55,.05,0], [.2,0,.5,.3,0], [.1,0,.4,.5,0], [.05,0,.3,.65,0], [0,0,.2,.8,0], [0,0,.1,.9,0], [0,0,.02,.96,.02], [0,0,0,.9,.1], [0,0,0,.73,.27], [0,.1,0,.45,.45], [0,.3,0,.25,.45], [0,.53,0,.12,.35], [0,.75,0,.05,.2], [0,1,0,0,0]. Because the micro controls live under the rotating spawned bones, the lid skin bones inherit the blink as a blended rotation about the eye center. Zebra quirks: the R bt constraint lists 'Lid In R' twice, and the R bt-base constraint uses Lid In L/Lid Ot L; Monster fixes both.

**Setup.** Micro controls Lid Tp/Bt(/Base) 01-03 and the corner controls Lid In/Ot L/R.

**Operators:** `RigUnit_ParentConstraint`, `RigVMDispatch_ArrayIterator`, `RigVMDispatch_ArrayGetAtIndex`, `UserDefinedStruct break`

**Scale:** 8 loops, about 116 constraint evaluations per frame (Zebra)

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:1053 ParentConstraint_158 Parents Lid In L, Lid Ot L, Lid Tp 01-03 L; IN Parents.N.Weight<-At_10.Element.ValueN`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:1081 ParentConstraint_164 (Lid In R twice)`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/asset.t3d:1900 Lid_Tp_Struct=((Value1=1.0),(Value1=0.775,Value3=0.225),...)`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Highlevel/Hierarchy/RigUnit_TransformConstraint.cpp FRigUnit_ParentConstraint_Execute (normalized, maintain offset from initial globals)`

#### UE6-lip-corner-constraints — Lip corner constraints (corner bones and Corner nulls)

*core* · assets: CRM_Zebra_Face, CRM_Monster_Face

Pin B, after Top/Low. The children are lip_corner_l and null Corner L Null (and the R equivalents), with parents [Skull, Jaw], maintain offset. Zebra weights: skull 0.5, jaw 0.5*clamp(1-smile_l,0,1). Monster: skull 0.5*clamp(1-frown_l,0,1), jaw 0.5*clamp(1-smile_l,0,1). So smiling sticks the corner to the skull and frowning sticks it to the jaw. Because the Corner L control is parented under Corner L Null, the corner pad follows the resulting mouth corner.

**Setup.** Corner L/R through the smile/frown curves.

**Operators:** `RigUnit_ParentConstraint`, `RigUnit_GetCurveValue`, `RigVMFunction_MathDoubleOneMinus`, `RigVMFunction_MathDoubleClamp`, `RigVMFunction_MathDoubleMul`

**Scale:** 4 constraints

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:2635-2664 Lip Corner Constraints (Child=Null Corner L Null)`; `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/graphs.txt:2932 Lip Corner Constraints (22 nodes)`

#### UE6-new-parent-follow — New Parent function (rigid follow of item groups by a control's delta)

*core* · assets: CRM_Zebra_Face, CRM_Monster_Face

Local function New Parent(Control, Children[]). ParentFrame = inverse(GetTransform(Control, LocalSpace)) * GetTransform(Control, Global). For controls, LocalSpace is the value relative to the offset frame, so this equals the control's offset/rest frame in world space. For each child: SetTransform(child, Global, w=1, propagate) = MakeAbsolute(MakeRelative(childGlobal, ParentFrame), ControlGlobal). Each child is thus moved rigidly by the control's current delta about its rest frame. Calls on pin E, in order: (1) Muzzle -> [lip_tp_01..04_l, lip_tp, lip_tp_04..01_r, lip_corner_l/r, Jaw, muzzle, Corner L/R Null, Skull Const, Jaw Const, nose, bone Teeth Tp, teeth_tp] (Monster adds cheek_l, cheek_r). (2) Mouth -> all 9 upper and 9 lower lip bones, corners, corner nulls, Jaw Const, Skull Const (Zebra also teeth_tp, teeth_bt, nose, Teeth Tp). (3) Lips Tp -> [lip_tp_02..04_r, lip_tp, lip_tp_04..02_l, Skull Const]. (4) An inline copy for Lips Bt over [lip_bt_04_r, lip_bt_03_r, lip_bt, lip_bt_03_l, lip_bt_04_l, lip_bt_02_l, lip_bt_02_r, Jaw Const]: index<5 uses SetTransform weight 1.0, otherwise weight 0.5, so lip_bt_02_l/r and Jaw Const get half follow.

**Setup.** Driven by Muzzle, Mouth, Lips Tp and Lips Bt.

**Operators:** `RigUnit_GetTransform`, `RigVMFunction_MathTransformInverse`, `RigVMFunction_MathTransformMul`, `RigVMFunction_MathTransformMakeRelative`, `RigVMFunction_MathTransformMakeAbsolute`, `RigUnit_SetTransform`, `RigVMFunction_MathIntLess`, `RigVMFunction_ControlFlowBranch`

**Scale:** 3 function calls + 1 inline 8-item loop

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:2125-2160 New Parent body`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:1187-1191 Less_1 B=5, Set Transform_15 Weight 1, Set Transform_17 Weight=0.5`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Rigs/RigHierarchy.cpp ComputeLocalControlValue (control local = value relative to offset)`

#### UE6-top-low-lip-constraints — Top/Low lip skull-jaw weighted parent constraints (smile/frown masked)

*core* · assets: CRM_Zebra_Face, CRM_Monster_Face

Pin B. Parent constraints use maintain offset, all channels and Average interpolation, with parents [Skull (bone skull), Jaw (bone jaw)] and normalized weights. Top: lip_tp (1,0); lip_tp_01_l (0.8, 0.2*clamp(1-smile_l,0,1)); lip_tp_01_r (0.8, 0.2*clamp(1-smile_r)); lip_tp_02_l (0.9, 0.1*clamp(1-smile_l)); lip_tp_02_r (0.9, 0.1*clamp(1-smile_r)); lip_tp_03/04_l/r (1,0). Low: lip_bt (0,1); lip_bt_01_l/r (0.2,0.8); lip_bt_02_l/r (0.1,0.9); lip_bt_03/04 (0,1). Monster makes the Low skull weights frown-masked: 01 = 0.2*clamp(1-frown), 02 = 0.1*clamp(1-frown). Skull and jaw are the control-driven bones, so the lips blend between upper and lower head.

**Setup.** None directly (jaw, Reverse Jaw, Corner smile/frown).

**Operators:** `RigUnit_ParentConstraint`, `RigUnit_GetCurveValue`, `RigVMFunction_MathDoubleOneMinus`, `RigVMFunction_MathDoubleClamp`, `RigVMFunction_MathDoubleMul`

**Scale:** 18 constraints

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:2567-2610 Top Lip Constraints`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:2612-2633 Low Lip Constraints`; `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/graphs.txt:2887 Low Lip Constraints (21 nodes, GetCurveValue frown_l)`

#### UE1-layout-prop — Prop module layout and prop spaces

*important* · assets: MR_Zebra, MR_Monster, MR_FN_Biped, CRU_PropAim

Control chain:
- Prop Global (under root/Local)
- Prop Local (under Prop Global)
- Prop (under Prop Local), with spaces from the 'Spaces' connector (hand_r, hand_l, spine_05)
- Prop Attach 01 and Prop Attach 02 (under Prop); Attach 02 has space Attach 01
- Aim (Arrow_Solid, under root/Local)

Shapes: RoundedTrapeze_ArrowTip_Thick for Global/Local/Prop, Box_Thick for the attaches. Each carries 'Prop/Color' metadata, rotation order YXZ.

Channels: Change Pivot (bool) on Global and on Local; Aim Weight [0, 1] on Global, Local and Prop; Prop Global Vis (hosted on root/Global via 'Control Vis Channel Host'); Prop Local Vis, Prop Control Vis, Aim Control Vis and Prop Attach 01/02 Vis (hosted on the 5 prop controls).

Nulls: Aim Global/Local/Prop Buffer.

The Arm IK spaces include Prop, Prop Attach 01 and Prop Attach 02, so hands can follow the prop. CRU_PropAim is a stand-alone rig containing just this module on a single root bone.

**Setup.** 17 controls: 6 transforms, 8 bool and 3 float channels.

**Operators:** `CRM_FN_Prop`

**Scale:** 1 module in each rig.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1883 'CONTROL Prop/Prop ... AvailableSpaces=((Key=(Type=Bone,Name="hand_r")),(Key=(Type=Bone,Name="hand_l")),(Key=(Type=Bone,Name="spine_05")))'`; `<dump>/FortniteRigs__UtilityRigs__CRU_PropAim/runtime_hierarchy.txt: 'CONTROL Prop/Aim parents=['BONE:root']'`; `<dump>/FortniteRigs__UtilityRigs__CRU_PropAim/modular_rig_model.txt: 'Prop/Parent' -> 'root'`

#### UE2-body-aim — Body Aim control (weighted aim of the Body's Z axis)

*important* · assets: CRM_FN_Body

Construction: 'Body Aim' is created under Body Orbit with local offset translation = Body Aim Control Offset (0,0,38), so it sits 38 units above the COG. Body gets Body Aim as available space labelled 'Aim'. Initial buffers: 'Aim Buffer Transform' = Body global; 'Aim Rotate Buffer Quat' = Body global rotation.

Forwards, step C.B, with w = channel 'Aim Weight':
- If w > 0, Body global rotation = Slerp/Lerp(A = MakeAbsolute(AimBufferTransform, globalOf(parent(Body))), B = AimBoneMath(...), T = w).rotation.
  - AimBoneMath: InputTransform = Body global; Primary {Axis = 'Body Aim Axis' (0,0,1), Target = BodyAim global translation, Kind = Location, Weight 1}; Secondary {Axis = (0,0,0), so it is disabled; Target = BodyAim.rot * (0,0,1), Kind = Direction}.
  - AimBoneMath rotates Body so that its primary axis points at the target (FindQuatBetweenVectors).
- Otherwise, Aim Buffer Transform = Body local transform (stored every frame while aim is off).

**Setup.** Body Aim: shape 'Aim Control Shape' (RoundedTrapeze_ArrowTip_Thick, color (1,0.5,1)).

Channels on Body Aim, each hosted on [Body, Body Orbit]:
- 'Body Aim Vis' (bool, default false)
- 'Aim Weight' (float 0..1, default 0)
- 'Aim Twist' (float 0..1, default 0)

**Operators:** `RigUnit_AimBoneMath`, `RigVMFunction_MathQuaternionRotateVector`, `RigVMFunction_MathTransformLerp`, `RigVMFunction_MathTransformMakeAbsolute`, `RigUnit_HierarchyGetParent`, `RigUnit_SetRotation`, `RigUnit_AddAvailableSpaces`

**Scale:** 1 per rig

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Body__CRM_FN_Body/graphs.txt:40 'RigUnit_AimBoneMath ... Primary=(Weight=1.000000,Axis=(X=0.000000,Y=0.000000,Z=1.000000)'`; `<dump>/FortniteRigs__Modules__FkSolves__Body__regen.py:1267 'AimBoneMath.Secondary ... Axis=(X=0.000000,Y=0.000000,Z=0.000000)'`; `<dump>/FortniteRigs__Modules__FkSolves__Body__CRM_FN_Body/graphs.txt:293 'Label="Aim"'`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Highlevel/Hierarchy/RigUnit_AimBone.cpp:10-80`

#### UE2-prop-aim-solve — Prop weighted aim (Aim Solve with buffer and stored rest transform)

*important* · assets: CRM_FN_Prop

'Aim Solve' is called three times in a chain: (Prop Global, Aim Global Buffer), (Prop Local, Aim Local Buffer), (Prop, Aim Prop Buffer). Each call has its own 'Aim Weight' channel and its own persistent 'Previous * Buffer Transform' variable.

Per call:
1. Buffer global translation = Control global translation.
2. AimConstraintLocalSpaceOffset(Child = Buffer, Parents = [Aim] with weight 1, AimAxis = +Y, UpAxis = +X, WorldUp = {Target = Aim.TransformLocation((10,0,0)), Kind = Location}, bMaintainOffset = false, filter XYZ, RotationOrderForFilter XZY).
3. If w > 0: Control global rotation = Lerp(PrevBuffer, Buffer global, w).rotation.
4. Otherwise: PrevBuffer = Control global transform (captured while aim is off).
5. Draw a debug line Control -> Aim (foreground, thickness 0.35), enabled when PropGlobalVis AND AimControlVis AND w > 0 AND ControlVis. Line colors: yellow (global), orange (local), red-orange (prop).

Buffer nulls: Aim Global Buffer under Parent; Aim Local Buffer under Prop Global; Aim Prop Buffer under Prop Local.

**Setup.** Three 'Aim Weight' floats (0..1, default 0), on Prop Global, Prop Local and Prop (runtime 'Aim Weight', '_2', '_3').

**Operators:** `RigUnit_AimConstraintLocalSpaceOffset`, `RigUnit_SetTranslation`, `RigUnit_SetRotation`, `RigVMFunction_MathTransformLerp`, `RigVMFunction_MathTransformTransformVector`, `RigVMFunction_DebugLineNoSpace`, `RigUnit_GetFloatAnimationChannelFromItem`, `RigUnit_HierarchyAddNull`

**Scale:** 3 solves per Prop

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Prop__CRM_FN_Prop/graphs.txt:732-834 'Aim Solve'`; `<dump>/FortniteRigs__Modules__FkSolves__Prop__CRM_FN_Prop/graphs.txt:753 'AimAxis=(X=0.000000,Y=1.000000,Z=0.000000); UpAxis=(X=1.000000'`; `<dump>/FortniteRigs__Modules__FkSolves__Prop__CRM_FN_Prop/graphs.txt:389,394,398 'Aim Global/Local/Prop Buffer'`; `<dump>/FortniteRigs__Modules__FkSolves__Prop__CRM_FN_Prop/regen.py:686,716,746 'Aim Line Color'`

#### UE3-limbtwist-translate — Twist-bone translation blending via weighted position constraint (Blend Translate)

*important* · assets: CRM_FN_LimbTwist

For each driven i, PositionConstraintLocalSpaceOffset:
- child = driven;
- parents = [Start with weight w_i, End with weight 1 - w_i];
- maintain offset, filter XYZ, weight 1.

w_i is the construction-time distance ratio, so twist nulls and bones slide proportionally along the limb when Start or End translate or stretch. It runs in forward and backward when Use Translates is on.

**Setup.** 'Use Translates' option (default True).

**Operators:** `RigUnit_PositionConstraintLocalSpaceOffset`, `RigVMFunction_MathDoubleSub`

**Scale:** 8 per biped

**Evidence:** `<dump>/.../CRM_FN_LimbTwist/graphs.txt:577-603 'Blend Translate' (Parents 0 weight = w, 1 weight = 1-w)`; `<dump>/.../CRM_FN_LimbTwist/graphs.txt:124-131 Use Translates branch`

#### UE4-movable-proxy-forward — Forward Movable Proxy v01 (interaction-driven rotate-about-pivot)

*important* · assets: CRFL_Hierarchy_v001, CRM_FN_Spine, CRM_FN_Body

Forwards Solve. Inputs: Proxy Control, Proxy Buffer, Proxy Snap To, Driven Control and Proxy Vis Channel. vis = GetBoolChannel(ProxyVisChannel), then SetControlVisibility(Proxy, vis). If vis: (A) if the viewport is translating the proxy (IsInteracting.bIsTranslating and the proxy is in Items), set Buffer global = Driven current global and IsSet=true, so the pivot can move without moving the driven control. (B) If the proxy is being rotated: when IsSet is false, first capture Buffer global = Driven global and set IsSet=true; then set Driven Control global = Buffer global. The buffer is a child of the rotating proxy, so the driven control orbits the proxy pivot. When the proxy is not being rotated: set IsSet=false; set the proxy's default-parent null global = {Rotation = Driven global rotation, Translation/Scale = Snap To global}; set the proxy's local rotation to identity. The pivot therefore re-snaps and its rotation resets after each rotate gesture, while its local translation (the pivot offset) persists.

**Setup.** The behaviour depends on editor gizmo interaction state, so it is only meaningful during live manipulation. It is a viewport-interaction feature, not animatable data.

**Operators:** `RigUnit_GetBoolAnimationChannelFromItem`, `RigUnit_SetControlVisibility`, `RigUnit_IsInteracting`, `DISPATCH_RigVMDispatch_ArrayFind`, `DISPATCH_RigDispatch_GetMetadata / SetMetadata ('IsSet', Self)`, `RigUnit_GetTransform`, `RigUnit_SetTransform`, `RigUnit_SetRotation (local)`, `RigUnit_HierarchyGetParent (default)`

**Scale:** 2 modules.

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:1084-1196 'Forward Movable Proxy v01' (comments 'When proxy control is translating - Set buffer to driven control position', 'When there's no interaction with the proxy control - Reset the Null position to Driven Control and proxy rotations to zero')`; `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Execution/RigUnit_IsInteracting.cpp:8-17 (bIsTranslating/bIsRotating from UnitContext.InteractionType; Items = ElementsBeingInteracted)`; `ue/<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:29 (Driven Control and Snap To both VariableNode_14)`

#### UE2-body-aim-twist-interaction — Drive Aim and Body Rotation (interaction-aware twist sync)

*nice-to-have* · assets: CRM_FN_Body

Local function, run in forwards step C.A only when Aim Weight > 0. Inputs: Interp = 'Aim Twist' channel, InBuf = 'Aim Rotate Buffer Quat', Aim, Body. Local variable buf = InBuf.

When the animator is rotating the Body control (IsInteracting.bIsRotating AND Body is in the interacted items):
1. P = ProjectTransformToNewParent(Child=Aim, Old=Body, New=Body) (initial child and old parent).
2. twist = the twist of P.rot about Z; swing = the swing of Aim's local rotation about Z.
3. Aim global rotation = swing * twist.
4. buf = P.rot.
5. Send RequestAutoKey events for Body and Aim, only during interaction.

Otherwise, Body global rotation = Lerp(A.rot = parentGlobal(Body).rot * buf, B = ProjectTransformToNewParent(Child=Body, Old=Aim, New=Aim), T = Interp).rotation. With Aim Twist = 1, Body inherits the Aim control's rotation delta.

Output: buf, which is written back to 'Aim Rotate Buffer Quat' and to AimBufferTransform.rotation. When the weight is 0 the buffer is set to Body's local rotation. This state is kept across evaluations.

**Setup.** Driven by the 'Aim Twist' channel (0..1) on Body Aim.

**Operators:** `RigUnit_IsInteracting`, `DISPATCH_RigVMDispatch_ArrayFind`, `RigUnit_ProjectTransformToNewParent`, `RigVMFunction_MathQuaternionSwingTwist`, `RigVMFunction_MathQuaternionMul`, `RigVMFunction_MathTransformLerp`, `RigUnit_SetRotation`, `RigUnit_SendEvent`

**Scale:** 1 per rig

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Body__CRM_FN_Body/graphs.txt:576-676 'Drive Aim and Body Rotation'`; `<dump>/FortniteRigs__Modules__FkSolves__Body__CRM_FN_Body/graphs.txt:594 'TwistAxis=(X=0.000000,Y=0.000000,Z=1.000000)'`; `<dump>/FortniteRigs__Modules__FkSolves__Body__CRM_FN_Body/graphs.txt:616 'Event=RequestAutoKey ... bOnlyDuringInteraction=true'`

#### UE2-pin-transform-copy — Pin module: copy global transform driver->driven bone

*nice-to-have* · assets: CRM_FN_Pin

Construction: resolve the Drivers and Driven array connectors; for each driver i that exists, append Drivers[i] and Driven[i] to the internal arrays. The driven bone's existence is not checked.

Forwards: for each i, Driven[i] global = Drivers[i] current global, with Weight 1 and bPropagateToChildren = false.

No controls, no backwards solve. This keeps the UE mannequin ik_* bones matching the deformed hands and feet.

**Setup.** MR_FN_Biped 'IK Bone Pins' pairs: hand_r->ik_hand_gun, hand_l->ik_hand_l, foot_l->ik_foot_l, foot_r->ik_foot_r, hand_r->ik_hand_r.

**Operators:** `RigUnit_ResolveArrayConnector`, `RigUnit_ItemExists`, `DISPATCH_RigVMDispatch_ArrayAdd`, `RigUnit_GetTransform`, `RigUnit_SetTransform`

**Scale:** 1 instance (MR_FN_Biped and MR_FN_BipedDMC only)

**Evidence:** `<dump>/FortniteRigs__Modules__Miscellaneous__CRM_FN_Pin/graphs.txt (Forwards 7 nodes, Construction 18 nodes)`; `<dump>/FortniteRigs__Modules__Miscellaneous__CRM_FN_Pin/regen.py:31 'Set Transform.bPropagateToChildren', 'false'`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/modular_rig_model.txt 'IK Bone Pins/Drivers' ... 'IK Bone Pins/Driven'`

#### UE4-get-mirror-transform — Get Mirror Transform (unused, degenerate)

*nice-to-have* · assets: CRFL_Hierarchy_v001

Inputs: Transform and Axis (default (1,0,0)). Output: Mirror Transform = Transform * FTransform(identity rotation, zero translation, Scale3D = -Axis), i.e. a world reflection by scale. With the default axis the scale is (-1,0,0), which zeroes Y and Z, so the function is only meaningful for axis inputs such as (1,-1,-1).

**Operators:** `RigVMFunction_MathVectorNegate`, `RigVMFunction_MathTransformMul`

**Scale:** No callers.

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:840-852 'Get Mirror Transform' (Negate.Result -> Multiply.B.Scale3D)`

#### UE4-unit-position-constraint — Engine unit: PositionConstraintLocalSpaceOffset

*nice-to-have* · assets: CRFL_Module_v001

Child, Parents[{Item, Weight}], bMaintainOffset (the offset is computed from the current child-parent delta), per-axis Filter and Weight. It sets the child's position to the weighted blend of the parents' positions plus the maintained offset.

**Operators:** `RigUnit_PositionConstraintLocalSpaceOffset`

**Scale:** 1 node (Blend Position, unused).

**Evidence:** `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/Highlevel/Hierarchy/RigUnit_TransformConstraint.h:358-395`

#### UE6-corner-height-lip-follow — Corner height: upper lip/cheek stick to skull when corner raised and jaw open

*nice-to-have* · assets: CRM_Zebra_Face, CRM_Monster_Face

Pin A, for L: w = min(remap(Corner L local tz, 0..100 -> 0..1, clamp), 4.5*jaw_open), where jaw_open is the curve value from the previous write. Then ParentConstraint(child, parent Skull weight 1, maintain offset, constraint Weight = w) for each child in [lip_tp_01..04_l, cheek_l, lip_corner_l]. R uses Corner R with [lip_tp_04_r, 01_r, 02_r, 03_r, cheek_r, lip_corner_r]. The Top/Lip Corner constraints on pin B later fully re-solve the lip bones, so this effect only survives on cheek_l/r.

**Setup.** Corner L/R vertical travel.

**Operators:** `RigUnit_GetTransform`, `RigVMFunction_MathDoubleRemap`, `RigUnit_GetCurveValue`, `RigVMFunction_MathDoubleMul`, `RigVMFunction_MathDoubleMin`, `RigUnit_ParentConstraint`

**Scale:** 2 loops x 6

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:640 ParentConstraint_23 IN Weight<-Minimum.Result; :642 Minimum; Multiply_5 B=4.5`

### D10 Inverse solves, matching and baking

#### UE2-spine-backward-matching — Spine backwards solve (FK/IK/Sec FK matching incl. mid-point ray intersection)

*core* · assets: CRM_FN_Spine

Backwards Solve.

C:
1. Evaluate the Start FK orient nulls.
2. Set 'Distribute Rotation' = 0.
3. If FK Vis: for each FK control i (at the last one, first evaluate the End FK orient nulls), control global = FkDeltaTransform(meta) * FKBone[i] current. Otherwise each FK control's local = identity.

D: evaluate the Start and End IK orient nulls.

E: Start IK global (T and R only) = Offset(meta) * Bones[0].

F:
1. Set IK Vis = true.
2. End IK = Offset * Bones[-1].
3. Run Solve Mid Control Null.
4. Mid IK global rotation = (Offset * Bones[N/2]).R.
5. Mid IK translation = RayIntersectRay(A = {origin Bones[-2].T, dir Bones[-2].R * (-1,0,0)}, B = {origin Bones[1].T, dir Bones[1].R * (+1,0,0)}) when 0 < RatioA < 20; otherwise (Offset * Bones[N/2]).T. The intersection of the end tangents recovers the Bezier control point.

G:
- If Sec FK Vis: rebuild the spline; run Attach Sec FKs To Spline(Matching = true), which writes the match nulls; for each i, SetControlOffset(SecFK[i], MatchNull[i] local); then SecFK[i] global = (Bones[i].T, Bones[i].R * Sec FKs Orient Offset, S); then Pelvis Local local = identity.
- Otherwise: all Sec FK local = identity.

**Operators:** `RigUnit_InverseExecution`, `DISPATCH_RigDispatch_GetMetadata`, `RigVMFunction_MathTransformMul`, `RigVMFunction_MathRayIntersectRay`, `RigVMFunction_MathQuaternionRotateVector`, `RigUnit_SetControlOffset`, `RigUnit_SetBoolAnimationChannelFromItem`, `RigUnit_SetFloatAnimationChannel`, `RigUnit_SetTransform`

**Scale:** 1 per spine/neck

**Evidence:** `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:1242 'Get Transform Metadata' (FkDeltaTransform)`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:1308,1419-1454 'RigVMFunction_MathRayIntersectRay' and 'RatioA -> Greater_1/Less_1' (B=0 / B=20)`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/regen.py 'Rotate Vector_2' Vector=(X=-1.000000 ...) / 'At_1_3.Index', '-2'`

#### UE3-ikfk-matching — Continuous IK<->FK auto matching (Match FK / Match IK)

*core* · assets: CRM_FN_IkFk2Bones

Match FK (sets module metadata 'Match FK' = true). For each virtual-bone global transform i:
- if i != 0: the default parent of FK[i] (the segment-scale null) gets global (identity rotation, VB.t, VB.s), then local rotation = identity;
- FK[i] global = (VB.rot * FK Rotation Offset, VB.t, VB.s).
All FK gimbals then get local = identity.

Match IK (sets 'Match IK' = true):
1. IK Base global = VB0.
2. IK Control global = (VB2.rot * IK Rotation Offset, VB2.t, VB2.s); IK Gimbal local = identity.
3. If IK End Align is on: IK Rotation offset = (IK Rotation Null rot, IK t/s) and IK Rotation global rotation = VB2.rot * IK Rotation Offset.
4. Auto PV global = Compute Pole Vector Parent(channel twist follow).
5. PV global translation = Compute Pole Vector Location v02(VB, Secondary Axis, PV Distance Scale).

When IK FK Auto Matching (default True) is on, Match FK runs every IK-mode frame and Match IK every FK-mode frame, so the inactive side always follows.

The Backwards Solve runs both matches. User events 'To IK' and 'To FK' call Match IK and Match FK.

Post Forwards Solve resets the metadata 'Match IK' / 'Match FK' / 'Key Controls' to false.

**Setup.** Right-click or event entries 'To IK' and 'To FK'; the IK FK Auto Matching option.

**Operators:** `RigUnit_SetTransform`, `RigUnit_SetRotation`, `RigUnit_SetTranslation`, `RigUnit_SetControlOffset`, `RigUnit_HierarchyGetParent`, `RigVMFunction_UserDefinedEvent`, `RigUnit_PostBeginExecution`, `DISPATCH_RigDispatch_SetModuleMetadata`

**Scale:** every frame per limb

**Evidence:** `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:2666-2723 'Match FK'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:2725-2818 'Match IK'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:281-288,580-583 IK FK Auto Matching branches`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:1772-1773 'EventName=To IK' / 'EventName=To FK'`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:289,590 'Post Forwards Solve' -> Set Module Bool Metadata Match IK=false`

#### UE-anim-baked-curves — Baked expression animation carrying bone tracks plus corrective, face, deformer and material curves

*important* · assets: Zeb_Face_Expressions, CR_Zebra_Deform

Zeb_Face_Expressions (163 frames, 6.79 s, skeleton SK_Zebra) is a bake of the full rig output. It has 371 bone tracks, including every def_* helper and twist bone, and 102 float curves:
- 39 body corrective curves: all deform curves except shoulder_up_l/r, shoulder_bk_l/r, knee_l, ankle_up_l, knee_squash_l/r, thigh_fwd_ext_l/r and thigh_up_ext_c.
- 54 face morph curves.
- 6 deformer-driver curves.
- cornea_size, pupil_dilation and lip_roll_in_bt_l.
There are no vector or transform curves. At playback the post-process rig overwrites the corrective curves and re-adds helper offsets on top of the baked helper tracks, because those offsets are AdditiveLocal. A project-level AnimCurveCompressionSettings uses the CompressedRichCurve codec.

**Setup.** None.

**Operators:** `AnimSequence`, `AnimCurveCompressionCodec_CompressedRichCurve`

**Scale:** 1 anim, 102 curves, 371 tracks

**Evidence:** `<dump>/Game__Assets__Zebra__Anims__Zeb_Face_Expressions/summary.json frames 163; float_curves [...]; bone_tracks [... def_elbow_in_r ... thigh_twist_04_r ...]`; `<dump>/Game__Misc__SandboxAnimCurveCompressionSettings/asset.t3d Codec=...AnimCurveCompressionCodec_CompressedRichCurve`

#### UE1-aau-match-limb — AAU_Biped 'Match Limb': Sequencer IK/FK snap-switch-and-key utility action

*important* · assets: AAU_Biped, MR_Zebra, MR_FN_Biped

Editor-only Blutility (ActorActionUtility). The Control Rig edit mode lists it for a selected control that belongs to a module rig of a supported class (only when the utility-actions setting is enabled; the graph comment says 'Requires: AnimMode.SupportUtilityActions 1'). The selected module's rig instance is passed as 'InModule'.

Match Limb(InModule):
1. ModuleName = GetObjectName(InModule); switch = ModuleName + '/Ik Fk Switch'.
2. Seq = GetFocusedLevelSequence. For each ControlRigSequencerBindingProxy in GetControlRigs(Seq), for each section of its track that IsActive: if proxy.ControlRig.CurrentControlSelection is non-empty, store 'Control Rig' and break. Cast it to ModularRig and store it.
3. frame = GetLocalPosition (DisplayRate). v = GetLocalControlRigBool(Seq, rig, switch, frame).
4. If v (IK): ExecuteEventOnModuleByNameForBP('To FK', ModuleName); Execute('Forwards Solve'); SetInteraction(true); SetLocalControlRigBool(switch, false, frame, bSetKey=true); SetInteraction(false); Select Control(ModuleName + '/FK 2').
   Else (FK): 'To IK'; Forwards Solve; key switch=true; select ModuleName + '/IK'.
5. ExecuteEventOnModuleByNameForBP('Key Controls', ModuleName); Execute('Forwards Solve').

Select Control(rig, name): EditorActorSubsystem.SelectNothing, then ClearControlSelection on every Sequencer control rig, then SelectControl(name, true).

The local variables 'Foot Module Name' and 'Match Foot' are declared but unused. The comments describe the Foot follow-up that relies on the forced full evaluation. This utility does not mirror or build rigs.

**Setup.** The animator selects any control of a limb module in Sequencer and runs 'Match Limb' from the right-click utility actions. The tool toggles and keys '<Limb>/Ik Fk Switch' and reselects FK 2 or IK.

**Operators:** `UActorActionUtility`, `ControlRigSequencerEditorLibrary.GetControlRigs`, `ControlRigSequencerEditorLibrary.GetLocalControlRigBool`, `ControlRigSequencerEditorLibrary.SetLocalControlRigBool`, `ControlRigSequencerEditorLibrary.SetInteraction`, `LevelSequenceEditorBlueprintLibrary.GetFocusedLevelSequence`, `LevelSequenceEditorBlueprintLibrary.GetLocalPosition`, `MovieSceneSection.IsActive`, `ControlRig.CurrentControlSelection`, `ControlRig.SelectControl`, `ControlRig.ClearControlSelection`, `ModularRig.ExecuteEventOnModuleByNameForBP`, `RigVMHost.Execute`

**Evidence:** `<dump>/FortniteRigs__ActorActionUtilities__AAU_Biped/asset.t3d (UTF-16): 'ParentClass="/Script/CoreUObject.Class'/Script/Blutility.ActorActionUtility'"'`; `<dump>/FortniteRigs__ActorActionUtilities__AAU_Biped/asset.t3d: 'FunctionGraphs(0)="/Script/Engine.EdGraph'Match Limb'"', 'bCallInEditor=True'`; `<dump>/FortniteRigs__ActorActionUtilities__AAU_Biped/asset.t3d: 'DefaultValue="To IK"', 'DefaultValue="To FK"', 'DefaultValue="Key Controls"', 'DefaultValue="/Ik Fk Switch"', 'DefaultValue="/FK 2"', 'DefaultValue="/IK"', 'DefaultValue="Forwards Solve"'`; `<dump>/FortniteRigs__ActorActionUtilities__AAU_Biped/asset.t3d: NodeComment '**Running the Match Event**' and 'Requires:\r\n\r\nAnimMode.SupportUtilityActions 1'`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRigEditor/Private/EditMode/ControlRigUtilityActions.cpp:151-252 (module-rig matching of ActorActionUtility supported classes)`

#### UE1-user-module-events — User-defined module events (To IK / To FK / Key Controls) triggered per module

*important* · assets: MR_Zebra, MR_FN_Biped, AAU_Biped

CRM_FN_IkFk2Bones defines 3 user events:
- 'To FK' runs Match FK: snap FK controls to the current IK pose.
- 'To IK' runs Match IK: snap IK/PV controls to the FK pose.
- 'Key Controls' sets module metadata.

CRM_FN_Foot defines 'Key Controls', which appends its controls to the key list. UModularRig::ExecuteEventOnModuleByNameForBP(Event, ModuleName) queues the event for a single module, runs it, and returns whether it executed. A following host 'Forwards Solve' is needed so dependent modules (Foot) see the metadata the Leg set.

**Operators:** `RigVMFunction_UserDefinedEvent`, `UModularRig::ExecuteEventOnModuleByNameForBP`, `UModularRig::ExecuteEventOnModule`

**Evidence:** `<dump>/FortniteRigs__Modules__IkSolves__IkFk2Bones__CRM_FN_IkFk2Bones/graphs.txt: 'EventName=To IK', 'EventName=To FK', 'EventName=Key Controls'`; `<dump>/FortniteRigs__Modules__Biped__Foot__CRM_FN_Foot/graphs.txt: 'EventName=Key Controls'`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/ModularRig.cpp:1504 'ExecuteEventOnModuleByNameForBP'`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/ModularRig.cpp:1446-1533`

#### UE2-body-backward — Body backwards solve (pelvis -> Body control)

*important* · assets: CRM_FN_Body

Backwards Solve:
- A: Body control global = MakeAbsolute(Local = inverse(meta 'Body Delta Transform'), Parent = Body bone current global) = BodyCtrl_init * inverse(BodyBone_init) * BodyBone_current.
- B: if Create Body Offset Control is true, Body Offset local = identity.

Body Orbit and Aim are not touched.

**Operators:** `RigUnit_InverseExecution`, `DISPATCH_RigDispatch_GetMetadata`, `RigVMFunction_MathTransformInverse`, `RigVMFunction_MathTransformMakeAbsolute`, `RigUnit_SetTransform`

**Scale:** 1 per rig

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Body__CRM_FN_Body/graphs.txt:499-531`; `<dump>/FortniteRigs__Modules__FkSolves__Body__CRM_FN_Body/graphs.txt:509 'Name=Body Delta Transform'`

#### UE2-fk-backward — FK backwards solve (controls from bones)

*important* · assets: CRM_FN_FkChain, CRM_FN_FkArray

Backwards Solve:
- A: evaluate the orient space nulls (FkArray also evaluates the full space nulls, using the non-combined Spaces and Orient Spaces lists).
- B: for each bone i, control[i] global = Control Transform Offset * bone[i] current global.

Controls keep their parent chain; no local reset is done.

**Operators:** `RigUnit_InverseExecution`, `RigUnit_SetTransform`, `RigVMFunction_MathTransformMakeAbsolute`

**Scale:** All FK instances

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkChain/graphs.txt:326-359`; `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkArray/graphs.txt:609-651`

#### UE2-root-bake-root-matching — Root backwards solve with selectable root-motion target (Bake Root On)

*important* · assets: CRM_FN_Root, CREnum_RootMatching

Backwards Solve (inversion, used for baking). R = current global of the root bone.

1. If the private bool 'Global Control Snapped' is false, store R in 'Global Control Transform' and set Snapped=true. This snaps Global only on the first frame; every Forwards Solve resets Snapped to false.
2. Switch on the integer channel 'Bake Root On':
- Case 0 ('Root motion on Root Control'): Root = R globally, Local local = identity, then Global = stored transform (global, propagating).
- Case 1 ('Root motion on Global Control'): Global, Local and Root are each set to R globally, in that order.
- Case 2 ('Root motion on Local Control'): Local = R, Root = R, then Global = stored transform.

**Setup.** The 'Bake Root On' channel (INTEGER, default 0, range 0..100 with limits on, ControlEnum=CREnum_RootMatching) lives on the Global control.

**Operators:** `RigUnit_InverseExecution`, `RigUnit_GetIntAnimationChannelFromItem`, `DISPATCH_RigVMDispatch_SwitchInt32`, `RigUnit_SetTransform`, `RigVMFunction_ControlFlowBranch`, `RigVMVariableNode (state)`

**Scale:** 1 per rig

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Root__CRM_FN_Root/graphs.txt:212-297`; `<dump>/FortniteRigs__Modules__FkSolves__Root__CRM_FN_Root/graphs.txt:256 'To Snap the GlobalControl only on the first frame'`; `<dump>/FortniteRigs__Modules__FkSolves__Root__CRM_FN_Root/regen.py:57,101,102 comment boxes 'Root motion on Global/Root/Local Control' (their positions map cases 1/0/2)`; `<dump>/FortniteRigs__Modules__FkSolves__Root__CRM_FN_Root/graphs.txt:35 'Set Global Control Snapped | Value=false'`

#### UE3-autokey-on-switch — Key Controls event and autokey on IK/FK switch

*important* · assets: CRM_FN_IkFk2Bones, CRM_FN_Foot

Forward: if IK FK Auto Matching && !IK FK Manipulation && IKSolve != Previous IK Solve, invoke entry 'Key Controls', then set Previous = IKSolve.

The 'Key Controls' event:
1. Sets metadata 'Key Controls' = true.
2. Sends RequestAutoKey (bOnlyDuringInteraction = true) for FK Controls + FK Gimbals + [IK, PV, IK Gimbal, IK Base] + IK Rotation (if it exists), and for the Ik Fk Switch channel.
3. If Segment Scale Control && interacting: recompute the segment-scale channels from current/default lengths and autokey them.

The Foot runs its own 'Key Controls' when the parent metadata 'Key Controls' is true. It autokeys FK toes + IK controls + Toes FK/IK with bOnlyDuringInteraction = false.

**Setup.** None; this is automatic on the channel toggle.

**Operators:** `RigVMInvokeEntryNode`, `RigVMFunction_UserDefinedEvent`, `RigUnit_SendEvent`, `RigUnit_ItemExists`, `RigUnit_IsInteracting`

**Scale:** per switch

**Evidence:** `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:259 'Run Key Controls | Entry=Key Controls' + 591-595`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:1778 'Send Event | Event=RequestAutoKey; ... bOnlyDuringInteraction=true'`; `<dump>/FortniteRigs__Modules__Biped__Foot__CRM_FN_Foot/graphs.txt:103-105,212-213 'Get Module Bool Metadata | Name=Key Controls; NameSpace=Parent'`; `<dump>/.../CRM_FN_Foot/graphs.txt:1024-1047 'Key Controls'`

#### UE3-foot-matching — Foot IK/FK matching driven by the parent leg

*important* · assets: CRM_FN_Foot

Match FK: Toes FK global = Ball Joint global.

Match IK:
1. Toe Tip, Heel, Ball IK, Foot Rocker and Ball controls get local = identity (no propagate); Toes IK Null local = identity.
2. Reset Foot Rig Values, then re-run Set Foot Pivots with zeros.
3. Toes IK global = Ball Joint global.

Forward runs Match FK when the parent's 'Match FK' metadata is true (after IK solve) and Match IK when the parent's 'Match IK' is true (in FK mode). Backwards runs Match FK, then Match IK, then FK toe controls = toe joints.

**Setup.** None; this follows the leg switch.

**Operators:** `RigUnit_SetTransform(Local)`, `DISPATCH_RigDispatch_GetModuleMetadata`, `RigUnit_GetTransformItemArray`, `RigUnit_SetTransformItemArray`

**Scale:** per foot

**Evidence:** `<dump>/.../CRM_FN_Foot/graphs.txt:1448-1511 'Match FK' / 'Match IK' / COMMENT 'Rerun Foot Pivot logic with zeroed out values'`; `<dump>/.../CRM_FN_Foot/graphs.txt:104,108,214-220 'Name=Match FK; NameSpace=Parent'`; `<dump>/.../CRM_FN_Foot/graphs.txt:1004-1022 Backwards`

#### UE4-module-local-matching — IK/FK matching conventions (Match FK / Match IK / Get Distances Between; module-local, not in CRFL)

*important* · assets: CRM_FN_IkFk2Bones, CRM_FN_Foot, CRFL_Hierarchy_v001

These functions are local to modules. The IkFk2Bones versions are described here because they rely on CRFL outputs. Match FK: sets module metadata 'Match FK'=true (Self). For each virtual-bone global transform i, FK control[i] global = {virtual bone position and scale, rotation = vb.rot * 'FK Rotation Offset'}. For i != 0, the FK control's default parent is first set to the virtual-bone position and scale with identity global rotation and local rotation zeroed. All FK gimbal controls are reset to local identity. Match IK: sets 'Match IK'=true. IK Base Control global = VirtualBones[0]. IK Control global = {VirtualBones[-1] position and scale, rotation = vb.rot * 'IK Rotation Offset'}. IK Gimbal is set to local identity. If the 'IK End Align' channel is on, it calls SetControlOffset(IK Rotation Control, {IK Rotation Null rotation, IK control translation}) and a global SetRotation. It then sets the Auto PV Parent from the local 'Compute Pole Vector Parent' and places the PV Control via CRFL Compute Pole Vector Location v02 (PV Distance Scale). Get Distances Between(Items, Initial) returns [|P_i - P_{i-1}|] for i >= 1. CRM_FN_Foot has its own smaller Match FK/IK. Compute FK (IkFk2Bones local) was not analysed here.

**Operators:** `DISPATCH_RigDispatch_SetModuleMetadata`, `RigUnit_GetTransformItemArray`, `RigVMFunction_MathQuaternionMul`, `RigUnit_SetTransform`, `RigUnit_SetRotation`, `RigUnit_SetTranslation`, `RigUnit_SetControlOffset`, `RigUnit_GetBoolAnimationChannelFromItem`, `RigUnit_GetFloatAnimationChannelFromItem`, `FUNC Compute Pole Vector Location v02`

**Scale:** Match FK/IK: IkFk2Bones 3 each, Foot 2 each. Get Distances Between: IkFk2Bones 3.

**Evidence:** `ue/<dump>/FortniteRigs__Modules__IkSolves__IkFk2Bones__CRM_FN_IkFk2Bones/graphs.txt:2666-2723 'Match FK' ('Set Module Bool Metadata | Name=Match FK; NameSpace=Self; Value=true')`; `ue/<dump>/FortniteRigs__Modules__IkSolves__IkFk2Bones__CRM_FN_IkFk2Bones/graphs.txt:2725-2818 'Match IK' ('Name=Match IK')`; `ue/<dump>/FortniteRigs__Modules__IkSolves__IkFk2Bones__CRM_FN_IkFk2Bones/graphs.txt:2090-2130 'Get Distances Between'`; `ue/<dump>/FortniteRigs__Modules__Biped__Foot__CRM_FN_Foot/graphs.txt:1448/1464 local 'Match FK' / 'Match IK'`

#### UE8-seq-no-constraints — No transform-constraint channels: the Zebra-Boombox relationship is not a live constraint

*important* · assets: zebra_audition, MR_Boombox, MR_Zebra, Main.umap

FChannelMapInfo always serializes ConstraintsIndex. However, no sequence's name table contains ConstraintAndActiveChannel, TickableTransformConstraint, TransformableControlHandle or ConstraintsManager, and Main.umap has no constraint names either. The Boombox is a separate spawnable actor in zebra_audition: it has its own 3D Transform track, its own MR_Boombox Control Rig track (channels include tape1_ctrl and tape2_ctrl), and its own Spawn track. Any hand/prop relationship is therefore baked into keys: Boombox actor transform and MR_Boombox control keys, and/or Zebra Arm IK space keys. MR_Zebra's own Prop module (Prop/Spaces -> hand_r, hand_l, spine_05; Arm IK Spaces include Prop/Prop and Prop Attach 01/02) can drive a Zebra-internal prop control, but nothing links it to the Boombox actor. CRU_PropAim, the rig tagged for Sequencer constraint-system use, is not referenced by any sequence.

**Setup.** The animator must key the boombox actor/rig and the Zebra hands separately, or use IK spaces on the Zebra Prop control.

**Operators:** `FConstraintAndActiveChannel (unused)`, `UMovieScene3DTransformTrack`

**Scale:** 0 constraint channels in 3 sequences

**Evidence:** `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Sequencer/MovieSceneControlRigParameterSection.h:212-213, 346-348`; `<dump>/Game__Sequences__zebra_audition/sequence.txt:36-41 (BINDING MR_Boombox: Transform, Animation(0), MR_Boombox CR track, Spawned)`; `name-table scan of zebra_audition, zebra_marketingPoseFaces, MR_Zebra_Take1 and Main.umap: only 'ConstraintsIndex' matches 'Constraint'`; `zebra_audition.uasset name table: 'Props.973FDE664EE3A9B0DC811CA933869126.MovieSceneControlRigParameterTrack_0.tape1_ctrl'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/modular_rig_model.txt :: 'Name="Prop/Spaces"' and 'Name="Arm R/IK Spaces"'`

### D11 Pose readers and drivers

#### UE-deform-exec-order — Execution order and curve-write semantics of the deform graph

*core* · assets: CR_Zebra_Deform, CR_Monster_Deform

BeginExecution -> Sequence. Sequence.A runs a single serial chain in this region order:
1. thigh fwd (L, R), then thigh_up_c, fwd_ext and thigh_up_ext_c
2. thigh ot
3. thigh bk
4. alternate thigh up
5. thigh in
6. knees, then knee_squash
7. shoulder up, fwd, bk, dn
8. elbows, then elbow_squash
9. clavicle up, dn, fwd, bk
10. ankles
11. head side and up/down
12. Squetch
13. forearm twist
14. spine fwd
Sequence.B then adds the 7 deformers.

All 45 readers and all 82 ModifyTransforms execute. 50 of the 54 SetCurveValue nodes execute, one per curve. SetCurveValue overwrites the curve value; it does not add. The rig receives the input pose and curves (AnimNode bTransferInputCurves=true), so baked corrective curves in animation are replaced by the recomputed values. Additive offsets on the same bone accumulate in chain order, and each reader reads the pose after all previous modifications. The helpers are leaves, so earlier offsets do not affect driver bones.

Monster order: clavicle_l up, clavicle_r up, clavicle_l dn, clavicle_r dn.

**Setup.** None.

**Operators:** `RigUnit_BeginExecution`, `RigVMFunction_Sequence`, `RigUnit_SetCurveValue`

**Scale:** 187 executed items in the Zebra chain

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:644-646 (RigUnit_BeginExecution -> RigVMFunction_Sequence; Sequence.A -> SphericalPoseReader_1_1_1_1_1_1_2; Sequence.B -> RigVMFunction_Sequence_1)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:338-378,608-636 (execute-context reroutes chaining regions)`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Hierarchy/RigUnit_SetCurveValue.cpp:8-20 (SetCurveValueByIndex)`; `<dump>/Game__Assets__Monster__Rig__CR_Monster_Deform/graphs.txt (L RigUnit_BeginExecution.ExecutePin -> SphericalPoseReader_1_1_1_1_1.ExecutePin ...)`

#### UE-spr-operator — Spherical Pose Reader (elliptical cone reader, 0..1 output)

*core* · assets: CR_Zebra_Deform, CR_Monster_Deform

Inputs: DriverItem, DriverAxis (local vector), RotationOffset (Euler degrees X=roll, Y=pitch, Z=yaw via FQuat::MakeFromEuler), ActiveRegionSize a, ActiveRegionScaleFactors (+W,-W,+H,-H), FalloffSize f, FalloffRegionScaleFactors, Flip{Width,Height}Scaling, and OptionalParentItem (defaults to the driver's first parent).

(1) Regions: innerAngle = clamp(a*180, 0.5, 178) deg. outerAngle = clamp(f*180, innerAngle+1, 179) deg. Each outer scale factor is lerp(inner_rad*activeSF/outer_rad, 1, falloffSF). Factors above 1 extrapolate.

(2) Reference frame, cached once from INITIAL transforms: LocalInit = DriverInitGlobal * ParentInitGlobal^-1, with its rotation replaced by LocalInitRot*Quat(RotationOffset). Each frame, WorldOffset = LocalInit * ParentCurrentGlobal, with its location set to the driver's current position.

(3) DriverNormal = WorldOffset^-1 rotation applied to (DriverCurrentRot*DriverAxis). The cone center is WorldOffset +Z, which equals RotationOffset*Z expressed in the driver's rest-local frame.

(4) theta = acos(DriverNormal.z). If DriverNormal.z == -1 the output is 0 (back-pole singularity). mag = theta/pi. If mag is about 0 the output is 1. Driver2D = normalize(DriverNormal.xy)*mag.

(5) Each ellipse's half-width is (x>0 ? +W : -W)*angle/pi and its half-height is (y>0 ? +H : -H)*angle/pi. The unit takes a 2-iteration closest-point-on-ellipse distance and an inside test for the inner and outer ellipses.

(6) Output: 1 if inside the inner ellipse, 0 if outside the outer ellipse, otherwise 1 - dInner/(dInner+dOuter).

The unit runs on the forward event as a mutable node. The output float OutputParam drives ModifyTransforms.Weight, SetCurveValue.Value or Remap. Every reader in both rigs uses ActiveRegionSize=0.1 (18 deg) with all ActiveRegionScaleFactors=0.1, so the full-value core is only about 1.8 deg, and FalloffSize=0.45 (81 deg). The practical result is a near-linear ramp from the outer ellipse edge to the cone center.

**Setup.** No controls. Per-reader parameters are node pin defaults (regen.py set_pin_default_value '<node>.RotationOffset' etc.). FlipWidthScaling/FlipHeightScaling do not appear in the dump, so they default to false.

**Operators:** `RigUnit_SphericalPoseReader`

**Scale:** 49 readers total (45 Zebra, 4 Monster)

**Evidence:** `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Highlevel/Hierarchy/RigUnit_SphericalPoseReader.cpp:9-133 (Execute), 135-183 (RemapAndConvertInputs), 185-209 (CalcOutputParam), 211-273 (DistanceToEllipse)`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/Highlevel/Hierarchy/RigUnit_SphericalPoseReader.h:280-421 (pins/defaults), 64-68 (GetEllipseWidthAndHeight)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:4 SphericalPoseReader_1_1_1_1_1_1_1_1 ActiveRegionSize=0.100000 ... FalloffSize=0.450000`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/summary.json units_used RigUnit_SphericalPoseReader 45`

#### UE-spr-zebra-clavicle — Zebra clavicle pose readers: up / dn / fwd / bk driving strap, chest and back helpers

*core* · assets: CR_Zebra_Deform

Eight readers on clavicle_l/r. Parent is None, so the reference is the parent spine_05. Left side uses axis (1,0,0); right side uses (-1,0,0). All targets are 90 deg from the rest axis except fwd (120 deg). Fall SF order is (+W,-W,+H,-H).

clavicle_up:
- L42 left: off(0,0,90), fallSF(1,1,0,1.1). Writes clavicle_up_l. Drives def_strap_l R(Y+50) T(4,0,-1), def_chest_l T(0,-2,10), def_back_l T(0,0,15).
- L45 right: off(0,180,-90), same SF. Writes clavicle_up_r. Drives def_strap_r R(Y+50) T(-4,0,1), def_chest_r T(0,2,-10), def_back_r T(0,0,-15).

clavicle_dn:
- L65 left: off(180,0,-90), fallSF(1.1,1.1,0,1.1). Writes clavicle_dn_l. Drives def_strap_l R(Y-80) T(-4,0,0) and def_chest_l T(0,5,-10).
- L68 right: off(0,0,-90), same SF. Writes clavicle_dn_r. Drives def_strap_r R(Y-80) T(4,0,0) and def_chest_r T(0,-5,10).

clavicle_fwd:
- L84 left: off(90,90,30), fallSF(1.1,1.1,1.46,1.0). Writes clavicle_fwd_l. Drives def_strap_l R(Z-45), def_chest_l T(-15,25,0), def_back_l R(X-30) T(0,5,0).
- L89 right: off(-90,90,30), same SF. Writes clavicle_fwd_r. Drives def_strap_r R(Z-45), def_chest_r T(15,-25,0), def_back_r R(X-30) T(0,-5,0).

clavicle_bk:
- L92 left: off(90,0,180), fallSF(1.0,1.1,0.8,0.8). Writes clavicle_bk_l. Drives def_strap_l R(Z+45) and def_back_l T(-10,-10,0).
- L95 right: off(90,0,0), same SF. Writes clavicle_bk_r. Drives def_strap_r R(Z+45) and def_back_r T(10,10,0).

**Setup.** None. Comment boxes: clavicle up, clavicle dn, clavicle fwd, clavicle bk.

**Operators:** `RigUnit_SphericalPoseReader`, `RigUnit_SetCurveValue`, `RigUnit_ModifyTransforms`

**Scale:** 8 readers, 8 curves, 22 ModifyTransforms

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:42,45,65,68,84,89,92,95 (SphericalPoseReader_1_1_1_1_1/_2/_4/_5/_8/_9/_10/_11)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:43,44,66,67,85,87,88,90,91,93,94,99,102-105,110,111,130,131 (ModifyTransforms def_strap/def_chest/def_back)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:62-70,96-101 (SetCurveValue clavicle_*)`

#### UE-spr-zebra-elbow — Zebra elbow bend readers with thresholded elbow-squash volume correction

*core* · assets: CR_Zebra_Deform

Left elbow, L27 lowerarm_l: axis(1,0,0), off(90,0,90), fallSF(2.0,0,2.0,2.0), parent None (upperarm_l). The center is -X, so the reader sits at the back pole at rest (output 0) and ramps toward full fold. Outer extent is 160 deg except the excluded -W half-plane.
- Writes elbow_l.
- Drives def_elbow_in_l T(-20,5,0) and def_elbow_ot_l T(6,0,0).
- Remap_1 (source 0.4..1 -> 0..1, clamped) of the same output writes elbow_squash_l and weights upperarm_twist_04_l T(-8,0,0), upperarm_twist_03_l T(-5,0,0), upperarm_twist_02_l T(-4,0,0), lowerarm_twist_04_l T(10,0,0), lowerarm_twist_03_l T(4,0,0), and def_elbow_ot_l T(6,0,0) again.

Right elbow, L28 lowerarm_r: axis(-1,0,0), off(0,-90,0), fallSF(2.1,2.0,0,2.0). It uses a different frame convention and excludes +H, but the center is still opposite the rest axis.
- Writes elbow_r.
- Drives def_elbow_in_r T(20,-5,0) and def_elbow_ot_r T(-6,0,0).
- Through 3 data reroutes, Remap_2 (0.4..1) writes elbow_squash_r and weights the mirrored twist and elbow translations (upperarm_twist_04/03/02_r at +8/+5/+4, lowerarm_twist_04/03_r at -10/-4, def_elbow_ot_r at -6).

**Setup.** None. Comment box: Elbows. Curves: elbow_l/r and elbow_squash_l/r.

**Operators:** `RigUnit_SphericalPoseReader`, `RigVMFunction_MathDoubleRemap`, `RigUnit_SetCurveValue`, `RigUnit_ModifyTransforms`

**Scale:** 2 readers, 2 remaps, 4 curves, 16 ModifyTransforms

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:27-32 (SphericalPoseReader_1_1_1_1_3 / _1_1_1_1_1_3, def_elbow MTs)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:144 Remap_1 SourceMinimum=0.400000 ... bClamp=true`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:482-495, 498-517 (Remap_1/Remap_2 links, RerouteNode_10-12)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:145-159 (SetCurveValue_30/31, arm-twist ModifyTransforms)`; `<UE>/Plugins/Runtime/RigVM/Source/RigVM/Private/RigVMFunctions/Math/RigVMFunction_MathDouble.cpp:164-180 (Remap)`

#### UE-spr-zebra-knee — Zebra knee bend readers with thresholded knee-squash thigh/calf twist-bone slides

*core* · assets: CR_Zebra_Deform

Left knee, L8 calf_l: axis(-1,0,0), off(-90,0,90), fallSF(0,2.1,2.1,2.1), parent thigh_l. The center is +X, the back pole at rest; outer extent is 168 deg with the +W half-plane excluded.
- Writes knee_l.
- Drives def_knee_in_l T(0,3,0), def_knee_ot_l R(Z-60) T(0,0,8), calf_twist_04_l T(0,5,0) and calf_twist_03_l T(0,2,0).
- Remap_3 (0.4..1 clamped) writes knee_squash_l and weights thigh_twist_04_l T(0,5,0), thigh_twist_03_l T(0,3.5,0), thigh_twist_02_l T(0,3,0) and thigh_twist_01_l R(Z-20).

Right knee, L11 calf_r: axis(1,0,0), off(-90,0,-90), same SF, parent thigh_r.
- Writes knee_r.
- Drives def_knee_in_r T(0,-3,0), def_knee_ot_r R(Z+60) T(0,0,-8), calf_twist_04_r T(0,-5,0) and calf_twist_03_r T(0,-2,0).
- Remap_4 writes knee_squash_r and drives thigh_twist_04/03/02_r at Y -5/-3.5/-3 plus thigh_twist_01_r R(Z-20). The -20 has the same sign as the left side, which is an asymmetry.

**Setup.** None. Comment boxes: Knee, Knee Extend.

**Operators:** `RigUnit_SphericalPoseReader`, `RigVMFunction_MathDoubleRemap`, `RigUnit_SetCurveValue`, `RigUnit_ModifyTransforms`

**Scale:** 2 readers, 2 remaps, 4 curves, 16 ModifyTransforms

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:8-13 (SphericalPoseReader_1_1_1_1_1_1_2_1 / _2_1_1 and def_knee MTs)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:119-122 (calf_twist MTs), 164-167, 182-185 (thigh_twist MTs)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:169 Remap_3, 186 Remap_4 (0.4..1 clamp); links 520-526, 542-554`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:123,168,187,188 SetCurveValue knee_l/knee_squash_l/knee_r/knee_squash_r`

#### UE-spr-zebra-shoulder — Zebra shoulder (upperarm) pose readers: up / fwd / bk / dn

*core* · assets: CR_Zebra_Deform

Eight readers on upperarm_l/r. Common settings: active 0.1 with SF 0.1x4, falloff 0.45. Fall SF order is (+W,-W,+H,-H); cone center = RotationOffset*Z in driver-rest-local.

shoulder_up:
- L16 upperarm_l: axis(1,0,0), off(0,0,0), fallSF(1,1,1,1), parent clavicle_l, center +Z, 90 deg away at rest. Writes shoulder_up_l. Drives def_strap_l T(-2,0,0), upperarm_twist_01_l T(2,0,0), upperarm_twist_02_l T(1,0,0).
- L18 upperarm_r: axis(-1,0,0), off(180,0,0), fallSF(1,1,1,1), parent clavicle_r. Writes shoulder_up_r. Drives def_strap_r T(2,0,0), upperarm_twist_01_r T(-2,0,0), upperarm_twist_02_r T(-1,0,0).

shoulder_fwd:
- L24 upperarm_l: off(90,0,45), fallSF(1.6,1,1.3,1.3), parent clavicle_l, 135 deg away. Writes shoulder_fwd_l. Drives def_strap_l R(Z+30) T(-2,0,0).
- L25 upperarm_r: off(-90,0,45), fallSF(1,1.6,1.3,1.3). Writes shoulder_fwd_r. Drives def_strap_r R(Z+60) T(2,0,0). This is asymmetric with the left side.

shoulder_bk:
- L34 upperarm_l: off(-90,0,-90), fallSF(2,0,1.25,1.25), parent spine_05, 180 deg away (back pole). Writes shoulder_bk_l. Drives def_strap_l R(Z-30) and def_back_l R(Z-30) T(-4,0,0).
- L35 upperarm_r: off(90,0,-90), fallSF(0,2,1.25,1.25), parent spine_05. Writes shoulder_bk_r. Drives def_strap_r R(Z+30) T(4,0,0) and def_back_r R(Z+30) T(4,0,0).

shoulder_dn:
- L48 upperarm_l: off(180,0,0), fallSF(1.1,0.5,1,1), parent clavicle_l. Writes shoulder_dn_l. Drives def_strap_l T(0.5,0,-0.5).
- L49 upperarm_r: off(0,0,0), fallSF(0.5,1.1,1,1), parent clavicle_r. Writes shoulder_dn_r. Drives def_strap_r T(-0.5,0,0.5).

All ModifyTransforms are AdditiveLocal and weighted directly by the reader output.

**Setup.** None. Corrective curves are named <joint>_<direction>_<side>. Comment boxes: Arm Up, Arm Fwd, Arm Bk, Arm Dn.

**Operators:** `RigUnit_SphericalPoseReader`, `RigUnit_SetCurveValue`, `RigUnit_ModifyTransforms`

**Scale:** 8 readers, 8 curves, 13 ModifyTransforms

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:16,18,24,25,34,35,48,49 (SphericalPoseReader_1_1_1_1_2_1_2_*)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:17,19-23,26,33,36,37,50,51,108,109 (ModifyTransforms def_strap/upperarm_twist/def_back)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:299-325 (OutputParam -> Weight links)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:46-56,106-107 (SetCurveValue shoulder_*)`

#### UE-spr-zebra-thigh — Zebra hip/thigh readers: up(fwd), ot, bk, alternate-up and thigh-in (cross-coupled)

*core* · assets: CR_Zebra_Deform

Twelve readers on thigh_l/r. All use OptionalParent pelvis; left axis is (-1,0,0), right is (1,0,0).

(a) thigh up/fwd:
- L5 left: off(90,0,-30), fallSF(1,1.4,1.2,1.2), center 120 deg from rest. Writes thigh_up_l. Drives def_thigh_in_l R(Z-15) T(8,1,3).
- L4 right: off(-90,0,-30), fallSF(1.4,1,1.2,1.2). Writes thigh_up_r. Drives def_thigh_in_r R(Z-15) T(-8,-1,0).
- Remap_5/6 (0.5..1 clamped) write thigh_fwd_ext_l/r and drive def_thigh_in_l T(3,2,3) (no rotation) and def_thigh_in_r R(Z-15) T(-3,-2,-3).

(b) thigh ot:
- L113 left: off(180,0,-30), fallSF(0.8,1.1,1,1). Writes thigh_ot_l.
- L112 right: off(0,0,-30), fallSF(1.1,0.8,1,1). Writes thigh_ot_r.
- Their ModifyTransforms (ModifyTransforms_3/4) carry identity offsets, so they do nothing.

(c) thigh bk:
- L206 left: off(-90,0,0), fallSF(0.8,1.1,1,1). Writes thigh_bk_l. Drives def_thigh_in_l T(0,-2,0) and def_thigh_ot_l T(10,-6,0).
- L205 right: off(0,0,-30), which is identical to thigh_ot_r. Writes thigh_bk_r. Drives def_thigh_in_r T(0,-2,0) and def_thigh_ot_r T(-10,6,0).

(d) alternate up:
- L218 left: off(135,20,-30), SF 1x4. Drives def_thigh_in_l T(8,0,-8).
- L217 right: off(-135,-160,-30). Drives def_thigh_in_r T(-8,0,8).
- Their SetCurveValue_47/48 (thigh_up_l/r) are not on the execution path, so they never run.

(e) thigh in, a narrow reader (outer about 41-53 deg):
- L225 left: off(4,50,-100), SF(0.5,0.65,0.5,0.6). Drives def_thigh_in_l T(3,10,6) and also def_thigh_in_r T(0,0,3).
- L224 right: off(-4,-130,-100), SF(0.5,0.65,0.6,0.5). Drives def_thigh_in_r T(-3,-10,-6) and def_thigh_in_l T(0,0,-3).
- This is cross-side coupling: one leg's pose moves both inner-thigh helpers.

**Setup.** None. Comment boxes: Thigh Fwd, Thigh Ot, Thigh In.

**Operators:** `RigUnit_SphericalPoseReader`, `RigVMFunction_MathDoubleRemap`, `RigUnit_SetCurveValue`, `RigUnit_ModifyTransforms`

**Scale:** 12 readers, 10 executed curve writes (thigh_up/ot/bk/fwd_ext x2 plus the thigh_up_c and thigh_up_ext_c min combos), 16 ModifyTransforms

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:4-7 (thigh fwd readers + ModifyTransforms/_1)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:112-117 (thigh ot readers, ModifyTransforms_3/_4 identity)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:205-231 (SphericalPoseReader_1_1_1_1_1_1_1_3.._5, ModifyTransforms_7.._18)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:211,214 Remap_5/Remap_6 SourceMinimum=0.500000 bClamp=true`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:587-592 (SphericalPoseReader_1_1_1_1_1_1_4 -> SetCurveValue_47 with no exec link)`

#### UE3-foot-values — Reading foot control rotations into roll/bank/heel scalars (Set Foot Values)

*core* · assets: CRM_FN_Foot

Runs in Pre Forwards Solve every frame.
- Ball IK local rotation to Euler (order YZX): X = Ball Pivot Heel Bend, Y = Heel Twist, Z = Heel Side.
- Foot Rocker local rotation:
  - swing-twist about X, twist to Euler ZYX, .X = Foot Rocker;
  - swing-twist about Y, twist to Euler ZXY, .Y = Foot Bank.
- Ball Pivot control local rotation to Euler YXZ .Z = Ball Pivot (not used downstream).

Reset Foot Rig Values zeroes all of these.

**Setup.** The animator rotates Foot Rocker (X = roll, Y = bank) and Ball IK (heel lift) instead of using float sliders.

**Operators:** `RigVMFunction_MathQuaternionToEuler`, `RigVMFunction_MathQuaternionSwingTwist`, `RigUnit_GetTransform(Local)`

**Scale:** per foot per frame

**Evidence:** `<dump>/.../CRM_FN_Foot/graphs.txt:1369-1416 'Set Foot Values' (RotationOrder=YZX/ZYX/ZXY/YXZ, TwistAxis X/Y)`; `<dump>/.../CRM_FN_Foot/graphs.txt:1339-1356 'Reset Foot Rig Values'`; `<UE>/Plugins/Runtime/RigVM/Source/RigVM/Private/RigVMFunctions/Math/RigVMFunction_MathQuaternion.cpp:219-229`

#### UE6-corner-logic — Corner Logic: 2-D slider to smile/frown/wide/narrow/curl curves

*core* · assets: CRM_Zebra_Face, CRM_Monster_Face

Let v = GetTransform(Corner, LocalSpace), the value relative to the offset. smile = clamp(0.01*v.tz, 0, 100); frown = clamp(-0.01*v.tz, 0, 100); wide = clamp(0.01*v.ty, 0, 100); narrow = clamp(-0.01*v.ty, 0, 100). These are Remap nodes (0..100 -> 0..100 and 0..-100 -> 0..100, bClamp) applied after multiplying by 0.01. curl = -0.01 * QuaternionToEuler(v.rotation, order ZYX).X, unclamped; the parallel Clamp(0..180) output is unused. In pin A the forward writes smile_l, frown_l, narrow_l, wide_l, curl_up_l from Corner L, then smile_r, frown_r, narrow_r, wide_r, curl_up_r from Corner R. The function body is pure (reads only) and has no wired exec pins; it is pulled as a data dependency.

**Setup.** Corner L/R. 100 local units = curve 1.0.

**Operators:** `RigUnit_GetTransform`, `RigVMFunction_MathDoubleMul`, `RigVMFunction_MathDoubleRemap`, `RigVMFunction_MathQuaternionToEuler`, `RigUnit_SetCurveValue`

**Scale:** 2 calls, 10 curves

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:2471-2509 Corner Logic`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:646 SetCurveValue_2 Curve=smile_l IN Corner Logic.smile`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:806 SetCurveValue_59 Curve=curl_up_l`

#### UE6-expression-shape-logic — Expression Shape Logic: bipolar channel to two clamped curves

*core* · assets: CRM_Zebra_Face, CRM_Monster_Face

Expression Shape Logic(Attribute channel key, Clamp Neg, Clamp Pos) reads c = GetFloatAnimationChannelFromItem(Attribute). It returns Pos = clamp(0.01*c, ClampNeg, ClampPos) and Neg = clamp(-0.01*c, ClampNeg, ClampPos). All call sites pass ClampNeg=0, ClampPos=2. Uses: All Tp -> lip_all_up_tp / lip_all_dn_tp; All Bt -> lip_all_up_bt / lip_all_dn_bt; Mid Tp -> lip_mid_up_tp / lip_mid_dn_tp; Mid Bt -> lip_mid_up_bt / lip_mid_dn_bt; Sneer Tp L -> sneer_tp_up_l / sneer_tp_dn_l; Sneer Bt L -> sneer_bt_up_l / sneer_bt_dn_l; the R side likewise. Monster adds Pos only for Nose Sneer L/R -> nose_sneer_l/r, Nose Flare L/R -> nose_flare_l/r, and Sticky L/R -> lip_stick_l/r.

**Setup.** Channels: All/Mid Tp/Bt on jaw (-200..200); Sneer Tp/Bt on Corner L/R (-200..200); Monster Nose Sneer/Flare L/R on Nose (0..200) and Sticky on Corner L/R (0..200). A channel value of 100 gives curve 1; the curve is capped at 2.

**Operators:** `RigUnit_GetFloatAnimationChannelFromItem`, `RigVMFunction_MathDoubleMul`, `RigVMFunction_MathDoubleClamp`, `RigUnit_SetCurveValue`

**Scale:** Zebra 8 calls / 16 curves; Monster 14 calls / 22 curves

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:2425-2449 Expression Shape Logic (B=0.01, B=-0.01)`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:978 Expression Shape Logic Clamp Pos=2`; `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/graphs.txt:1319 SetCurveValue_86 Curve=nose_sneer_l`

#### UE6-jaw-open-reader — Jaw-open cone reader (SphericalPoseReader on jaw)

*core* · assets: CRM_Zebra_Face, CRM_Monster_Face

Pin F: SphericalPoseReader with DriverItem bone Jaw, DriverAxis (0,1,0), RotationOffset (90,0,90), ActiveRegionSize 0.1 (scale factors 0.1), FalloffSize 0.45, FalloffRegionScaleFactors PW 1.1, NW 0.8, PH 0.8, NH 0.8, parent = the driver's parent. It produces a 0..1 weight from the jaw bone's direction relative to its rest frame. The output feeds Jaw Open Logic (Zebra input 'B', Monster input 'Jaw Curve').

**Setup.** Driven by the jaw control through the jaw bone.

**Operators:** `RigUnit_SphericalPoseReader`

**Scale:** 1 reader

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:637 SphericalPoseReader_1_1_1_1_2_1_2_2_1 DriverItem=Jaw DriverAxis Y RotationOffset (90,0,90)`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Highlevel/Hierarchy/RigUnit_SphericalPoseReader.cpp`

#### UE7-deformer-control-channel-mapping — Squash controls -> deformer curves (translate/rotate to curve remap)

*core* · assets: CRM_Zebra_Face, CRM_Monster_Face

Each squash control's LocalSpace transform is decomposed, remapped (RigVMFunction_MathDoubleRemap, bClamp=False, linear) and written to curves. The curve names below are Zebra's; Monster appends '_deformer' to the head curves (e.g. head_squash_deformer).

Head Squash control (the local rotation is converted with QuaternionToEuler, order ZYX):
- Tz -> Remap(-100..100 -> -1..1) -> head_squash
- rotZ -> Remap(-135..135 -> 1..-1) -> head_twist, i.e. -rz/135
- Tx -> Remap(-100..100 -> 1..-1) -> head_bend, i.e. -Tx/100

Muzzle Squash control:
- Tz -> Remap(-30..30 -> -1..1) -> muzzle_squash_deformer
- Tx -> Remap(20..-20 -> 1..-1) -> muzzle_bend_deformer, i.e. Tx/20

Skull Tp Squash control:
- Tz -> Remap(-100..100 -> -1..1) -> skull_tp_squash_deformer
- Tx -> Remap(50..-50 -> -1..1) -> skull_tp_bend_deformer, i.e. -Tx/50

Mouth Squash control (Monster only):
- Tz/30 -> mouth_squash_deformer
- Tx/20 -> mouth_bend_deformer

Net effect with the deformer remaps:
- head stretch ratio s = 1 + Tz/100
- muzzle and mouth s = 1 - Tz/30
- skull top s = 1 + Tz/100
- head bend angle = -(Tx/100)*180 deg
- muzzle bend = (Tx/20)*180 deg
- skull bend = -(Tx/50)*180 deg
- head twist: mesh rotates +rz about the Head Twist Null's Z

Dead writes: Zebra SetCurveValue_65 (muzzle_squash_deformer, fed from the head remap) and Monster SetCurveValue_42 have no exec input.

**Setup.** Head Squash: EULER_TRANSFORM control, parent NULL 'Head Attach Null', shape Circle_Thick, init_global T(0,0,150) (Zebra) or T(0,0,100) (Monster). Skull Tp Squash: parent control Head Squash, Circle_Thick. Muzzle Squash: parent bone muzzle, shape Default, T(0,20,89) (Zebra) or (0,28,20) (Monster). Mouth Squash (Monster): parent muzzle, T(0,28,23). All four have no limits, preferred rotation order YZX, and shape visibility BASED_ON_SELECTION. Channels used: tx, tz, and rz for the head.

**Operators:** `RigUnit_GetTransform (LocalSpace)`, `RigVMFunction_MathQuaternionToEuler (RotationOrder=ZYX)`, `RigVMFunction_MathDoubleRemap`, `RigUnit_SetCurveValue`

**Scale:** Zebra: 3 controls, 7 curve writes. Monster: 4 controls, 9 curve writes.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:1103-1124 (GetTransform_1 LocalSpace, Remap_1/2/3/4/5/7/8, SetCurveValue Curve=head_squash/head_twist/head_bend/...)`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:1723-1750 (links Translation.Z/X, Result.Z)`; `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/graphs.txt:1236-1303 and links 'GetTransform_13.Transform.Translation.Z -> Remap_11.Value' etc.`

#### UE-helper-shoulder-drive — Shoulder/clavicle helper drive: def_strap, def_chest, def_back

*important* · assets: CR_Zebra_Deform

def_strap_l/r receive 8 additive offsets each, from shoulder up/fwd/bk/dn and clavicle up/dn/fwd/bk. These include rotations of Y+50 (clavicle up), Y-80 (clavicle dn), Z-45 (clavicle fwd), Z+45 (clavicle bk), Z+30 or +60 (shoulder fwd) and Z-/+30 (shoulder bk), plus 0.5 to 4 cm slides. def_chest gets 3 offsets (clavicle up, dn, fwd), with the largest being T(-15,25,0) on clavicle fwd. def_back gets 4 offsets (clavicle up T(0,0,15), fwd R(X-30), bk T(-10,-10,0), shoulder bk R(Z-30) T(-4,0,0)). Right-side values are sign-mirrored. See UE-spr-zebra-shoulder and UE-spr-zebra-clavicle for the per-reader values.

**Setup.** None.

**Operators:** `RigUnit_ModifyTransforms`

**Scale:** def_strap 16, def_chest 6, def_back 8 ModifyTransforms

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:43,66,85,93 def_strap_l rotations (Y=0.422618 / Y=-0.642788 / Z=-0.382683 / Z=0.382683)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:87 def_chest_l T(X=-15,Y=25)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:104 def_back_l Rotation X=-0.258819`

#### UE-monster-trap-correctives — Monster clavicle up/down trapezius helper correctives

*important* · assets: CR_Monster_Deform, Monster_Skeleton

Four readers with active 0.1/0.1x4, falloff 0.45, fallSF(1,1,0,1.1) and parent None (spine_05):
1. clavicle_l up: axis(1,0,0), off(0,0,90). Drives def_trap_l R(Y+30) T(4,-30,-1).
2. clavicle_r up: axis(-1,0,0), off(0,180,-90). Drives def_trap_r R(Y+30) T(-4,20,1).
3. clavicle_l dn: axis(1,0,0), off(0,180,90), whose center is -Z. Drives def_trap_l R(Y-30) T(-2,10,1).
4. clavicle_r dn: axis(-1,0,0), off(0,0,-90). Drives def_trap_r R(Y-30) T(4,-10,-1).
All four are AdditiveLocal. No curves are written. The def_trap_l/r bones are children of clavicle_l/r, with rest local T(±5.309,±0.822,±4.113) R(-0.2358,-4.45,21.72), identical to Zebra def_strap.

**Setup.** None. Comment box: Clavicle Deform.

**Operators:** `RigUnit_SphericalPoseReader`, `RigUnit_ModifyTransforms`

**Scale:** 4 readers, 4 ModifyTransforms, 2 helper bones

**Evidence:** `<dump>/Game__Assets__Monster__Rig__CR_Monster_Deform/graphs.txt (SphericalPoseReader_1_1_1_1_1.._4; ModifyTransforms_2_2_2_1.._4 def_trap_*)`; `<dump>/Game__Assets__Monster__Meshes__SKM_Monster/bones.txt def_trap_l parent=clavicle_l local=T(5.309,0.822,4.113)`; `<dump>/Game__Assets__Monster__Rig__CR_Monster_Deform/summary.json units_used`

#### UE-spr-halfplane — One-sided (half-plane) pose readers via zero falloff scale factor and >1 extrapolated factors

*important* · assets: CR_Zebra_Deform, CR_Monster_Deform

The reader picks the ellipse quadrant from the sign of the 2D point. Setting one FalloffRegionScaleFactor to 0 shrinks the outer ellipse to about 1.8 deg in that half-plane (lerp(min,1,0) = min = inner*0.1/outer), so the output is effectively 0 whenever the driver moves to that side. This turns a symmetric cone into a one-directional reader, for example knee bend on one side only, or positive versus negative forearm twist. Factors above 1 (1.1, 1.25, 1.4, 1.46, 1.6, 2.0, 2.1, 2.2) are extrapolated rather than clamped by the math, which widens the outer ellipse past the 81-degree FalloffSize. Derived outer half-extents: 1.4 -> 112.7 deg; 2.0 -> 160.2 deg; 2.1 -> 168.1 deg; 2.2 -> 176 deg. The header metadata declares ClampMin=0/ClampMax=1 for these fields, but the stored values exceed it.

**Setup.** Authored per node. Readers with a zero factor: calf x2, lowerarm x2, upperarm back x2, clavicle up/dn x4 (+H=0), spine fwd x3, forearm twist x4, Monster clavicle x4.

**Operators:** `RigUnit_SphericalPoseReader`

**Scale:** 23 of 49 readers use a 0 factor; 30 use factors >1

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:8 calf_l FalloffRegionScaleFactors=(PositiveWidth=0.000000,NegativeWidth=2.100000,...)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:196 lowerarm_twist_01_l (PositiveWidth=2.2,NegativeWidth=0,...)`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Highlevel/Hierarchy/RigUnit_SphericalPoseReader.cpp:158-170 (Lerp(Min,1,factor))`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/Highlevel/Hierarchy/RigUnit_SphericalPoseReader.h:40-51 (ClampMax=1 meta)`

#### UE-spr-zebra-ankle — Zebra ankle up/down readers (curve-only correctives)

*important* · assets: CR_Zebra_Deform

Four readers on foot_l/r with parent None (calf). DriverAxis is (0,1,0) for left and (0,-1,0) for right. Each center is 90 deg from rest; the outer extent is about 73 deg in width and 41 or 89 deg in height. None of them drive helper bones.
- ankle_dn_l: L80 foot_l, off(90,90,90), fallSF(0.9,0.9,1.1,0.5).
- ankle_dn_r: L82 foot_r, off(-90,90,90), same SF.
- ankle_up_l: L124 foot_l, off(-90,90,90), fallSF(0.9,0.9,0.5,1.1).
- ankle_up_r: L126 foot_r, off(90,90,90), fallSF(0.9,0.9,0.5,1.1).

**Setup.** None.

**Operators:** `RigUnit_SphericalPoseReader`, `RigUnit_SetCurveValue`

**Scale:** 4 readers, 4 curves

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:80-83 (SphericalPoseReader_1_1_1_1_6/_7, SetCurveValue_14/15)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:124-127 (SphericalPoseReader_1_1_1_1_12/_13, SetCurveValue_26/27)`

#### UE-spr-zebra-forearm-twist — Forearm twist positive/negative readers on the twist bone (half-plane split)

*important* · assets: CR_Zebra_Deform

Four readers measure the twist of lowerarm_twist_01_l/r, whose pose comes from the animator rig's LimbTwist or the baked animation. Parent is None (lowerarm). DriverAxis is (0,1,0) for left and (0,-1,0) for right. The center is opposite the rest axis (180 deg, the back pole), and fallSF is (2.2,0,2.2,2.2), so the outer extent is about 176 deg and the -W half-plane is excluded.

The two readers on each side use RotationOffsets that give the same center but mirrored sphere X/Y axes, so the excluded half-plane falls on opposite twist directions:
- forearm_pos_l: L196, off(0,90,90).
- forearm_neg_l: L199, off(0,-90,-90).
- forearm_neg_r: L201, off(0,90,-90).
- forearm_pos_r: L202, off(0,-90,90).
The only outputs are the curves.

**Setup.** None. Comment box: Forearm Twists.

**Operators:** `RigUnit_SphericalPoseReader`, `RigUnit_SetCurveValue`

**Scale:** 4 readers, 4 curves

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:196-203 (SphericalPoseReader_1_1_1_1_18.._21, SetCurveValue_39-42)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:564-572 (reader -> SetCurveValue exec/data links)`

#### UE-spr-zebra-head — Zebra head up/down/side-down readers (curve-only)

*important* · assets: CR_Zebra_Deform

Four readers on bone 'head' with parent None (neck_02). All use fallSF 1.1x4 (outer about 89 deg).
- head_side_dn_l: L132, axis(0,0,-1), off(90,90,90), center -X.
- head_side_dn_r: L134, axis(0,0,1), off(90,90,90). No morph target named head_side_dn_r exists, so this output has no visible effect.
- head_dn: L171, axis(0,1,0), off(90,90,90).
- head_up: L173, axis(0,1,0), off(-90,90,90), center +X.
The only outputs are the curves.

**Setup.** None. Comment boxes: Head Side, Head Up/Down.

**Operators:** `RigUnit_SphericalPoseReader`, `RigUnit_SetCurveValue`

**Scale:** 4 readers, 4 curves

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:132-135 (SphericalPoseReader_1_1_1_1_14/_15, SetCurveValue_28/29)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:171-174 (SphericalPoseReader_1_1_1_1_16/_17, SetCurveValue_33/34)`; `<dump>/Game__Assets__Zebra__Meshes__SKM_Zebra/summary.json morph_targets (head_side_dn_l present, head_side_dn_r absent)`

#### UE-spr-zebra-spine-fwd — Per-vertebra spine forward-bend readers

*important* · assets: CR_Zebra_Deform

Three identical readers on spine_01, spine_02 and spine_03 with parent None (the previous spine bone). Each uses axis(0,1,0), off(90,180,90) and fallSF(0,1.1,0.75,0.75), with center -X at 90 deg from rest. The +W half-plane is excluded (one-sided forward bend), and the height extent is about 61 deg. They write spine_01_fwd, spine_02_fwd and spine_03_fwd.

**Setup.** None. Comment box: Spine.

**Operators:** `RigUnit_SphericalPoseReader`, `RigUnit_SetCurveValue`

**Scale:** 3 readers, 3 curves

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:247-252 (SphericalPoseReader_1_1_1_1_22/_23/_24, SetCurveValue_44/52/53)`

#### UE-threshold-remap-secondary — Thresholded secondary correctives (Remap of reader output with clamp)

*important* · assets: CR_Zebra_Deform

A second-stage corrective feeds the reader output through Remap: result = lerp(TMin, TMax, clamp((v-SMin)/(SMax-SMin), 0, 1)). It only activates after the primary pose is partly reached.
- Elbow and knee use SMin=0.4, SMax=1 into 0..1, producing elbow_squash_l/r and knee_squash_l/r and weighting the extra twist-bone and helper slides.
- Thigh fwd uses SMin=0.5, producing thigh_fwd_ext_l/r plus extra def_thigh_in offsets.
The same remapped value drives both the curve (morph) and the joint offsets.

**Setup.** None.

**Operators:** `RigVMFunction_MathDoubleRemap`

**Scale:** 6 remaps, 6 curves, 22 ModifyTransforms

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:144,152,169,186 (Remap_1..4 SourceMinimum=0.400000 bClamp=true)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:211,214 (Remap_5/6 SourceMinimum=0.500000 bClamp=true)`; `<UE>/Plugins/Runtime/RigVM/Source/RigVM/Private/RigVMFunctions/Math/RigVMFunction_MathDouble.cpp:164-180`

#### UE4-get-node-twist-value — Get Node Twist Value (twist about an axis relative to the rest pose)

*important* · assets: CRFL_Math_v001, CRM_FN_LimbTwist, CRFL_Module_v001

Inputs: Item and Axis (default X). Output: Twist (quaternion). delta = MakeRelative(Global=CurrentLocal(Item), Parent=InitialLocal(Item)).Rotation, which is init^-1 * current in the initial local frame. Twist = the twist component of delta about normalize(Axis) (FQuat::ToSwingTwist). A zero axis returns identity.

**Operators:** `RigUnit_GetTransform (LocalSpace, initial and current)`, `RigVMFunction_MathTransformMakeRelative`, `RigVMFunction_MathQuaternionSwingTwist`

**Scale:** LimbTwist 1 call (inside its local Blend Twist); CRFL_Module Blend Twist 1.

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Math_v001/graphs.txt:455-473 'Get Node Twist Value'`; `UE_5.8/Engine/Plugins/Runtime/RigVM/Source/RigVM/Private/RigVMFunctions/Math/RigVMFunction_MathQuaternion.cpp:219-229 SwingTwist`; `ue/<dump>/FortniteRigs__Modules__Biped__LimbTwist__CRM_FN_LimbTwist/graphs.txt:470 (inside the local Blend Twist)`

#### UE6-soft-eyes — Soft eyes: spherical pose readers on eye bones driving lid bones

*important* · assets: CRM_Zebra_Face, CRM_Monster_Face

Pin I runs 4 SphericalPoseReaders per eye. Common settings: DriverItem bone eye_l (eye_r), DriverAxis X (eye_r: -X), ActiveRegionSize 0.1, ActiveRegionScaleFactors all 0.1, FalloffSize 0.45, OptionalParentItem none (the reader uses the driver's parent). Left RotationOffsets: Dn (180,0,0), Up (0,0,0), In (90,-90,0), Ot (90,-90,180). Right: Dn (0,0,0), Up (180,0,0), In (-90,-90,0), Ot (-90,-90,180). The falloff scale factors are skewed, e.g. Dn/Up PW 1.1 NW 1.0 PH .8 NH .8. Each output weight w drives an AdditiveLocal ModifyTransforms loop over the 12 spawned lid bones [Bt 01-03, Bt Base 01-03, Tp 01-03, Tp Base 01-03] using the quaternion tables 'Soft Eyes Dn/Up/In/Ot'. Zebra degrees: Dn Y+[5,15,5,2.5,7.5,2.5,7.5,7.5,2.5,1.25,3.75,1.25]; Up Y-[2.5,7.5,2.5,1.25,2.5,1.25,5,15,5,2.5,5,2.5]; In Z+[10,10,5]x4; Ot Z-[5,10,10]x4. Monster uses a larger Dn/Up set. The lids follow the eyeball gaze.

**Setup.** Automatic, driven by eye rotation (Eye L/R controls and the aim).

**Operators:** `RigUnit_SphericalPoseReader`, `RigUnit_ModifyTransforms`, `RigVMDispatch_ArrayIterator`, `RigVMDispatch_ArrayGetAtIndex`

**Scale:** 8 pose readers, 8 loops x 12 bones

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:993 SphericalPoseReader_1_1_1_1_2_1_2_2_10 DriverItem eye_l RotationOffset=(X=180)`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:1020 SphericalPoseReader_..._14 DriverItem eye_r DriverAxis X=-1`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/asset.t3d:1900 Soft_Eyes_Dn=...`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/Highlevel/Hierarchy/RigUnit_SphericalPoseReader.h`

#### UE7-deformer-curve-factor-remap — Curve -> deformer factor conversion

*important* · assets: CR_Zebra_Deform, CRM_Monster_Face

Squash factors: StretchFactor = (c+1)/2 for head and skull top, so s = 2*StretchFactor = 1 + c. For muzzle and mouth the remap is inverted: StretchFactor = (1-c)/2, so s = 1 - c. The kernel clamps s to at least 1e-4, so c = -1 (head) or c = +1 (muzzle) gives a full squash.

Bend and twist factors take the curve value directly, and the kernel maps it to an angle of f * MaxAngle (180 deg for bend, 135 deg for twist). No clamping happens anywhere, so factors beyond ±1 over-rotate.

**Setup.** None.

**Operators:** `RigVMFunction_MathDoubleRemap`

**Scale:** 7 remaps (3 Zebra, 4 Monster).

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt: 'Remap_7 ... SourceMinimum=-1.000000; SourceMaximum=1.000000; TargetMinimum=0.000000; TargetMaximum=1.000000; bClamp=False'`; `same file: 'Remap_8 ... SourceMinimum=1.000000; SourceMaximum=-1.000000; TargetMinimum=0.000000; TargetMaximum=1.000000'`; `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/graphs.txt:1260,1264,1283,1291 (Remap_18/19/20/21)`

#### UE-deform-nonexecuted-nodes — Present-but-inert nodes (importer must follow execute links)

*nice-to-have* · assets: CR_Zebra_Deform

Several nodes exist but have no effect, so an importer should treat execution and data links as authoritative rather than node presence:
- SetCurveValue_47/48 (thigh_up_l/r from the alternate readers) have no execute link.
- SetCurveValue_37/38 hang off the unexecuted Branch.
- ModifyTransforms_3/4 are executed but carry identity offsets.
- RigVMFunction_MathTransformArrayToSRT -> ArrayAverage is orphaned.
- 'Inverse_1' is an unresolved template node (?::None).
- The TwistFactor_Value variable is unused.

**Setup.** None.

**Operators:** `RigUnit_SetCurveValue`, `RigUnit_ModifyTransforms`, `RigVMFunction_MathTransformArrayToSRT`, `RigVMFunction_MathVectorArrayAverage`

**Scale:** ~8 inert nodes

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:221-222 SetCurveValue_47/48 (only data links at 591-592)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:114-115 ModifyTransforms_3/_4 Translation=(0,0,0) identity`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:179-181 Inverse_1 ?::None; ArrayToSRT; ArrayAverage`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/regen.py:1680 add_template_node('Inverse::Execute(in Value,out Result)'...)`

#### UE-swing-twist-reader-unused — Swing-twist forearm twist reader with sign branch (authored but never executed)

*nice-to-have* · assets: CR_Zebra_Deform

Alternative forearm-twist reader, and dead code:
1. delta = GetTransform(lowerarm_twist_01_l, LocalSpace, current).Rotation * inverse(GetTransform(same, LocalSpace, bInitial=true).Rotation).
2. SwingTwist(TwistAxis=(1,0,0)) is applied to delta and the twist is converted with ToEuler(ZYX); the X component goes through an identity Remap (0..1 -> 0..1, no clamp).
3. Greater(>0) drives a Branch: True writes forearm_pos_l = value, False writes forearm_neg_l = value.

The Branch's execute input is not linked, so this path never runs. The forearm curves come from the spherical readers on the twist bone instead. A reimplementation should NOT evaluate it, but the same design works as a signed twist-reader pattern.

**Setup.** None.

**Operators:** `RigUnit_GetTransform`, `RigVMFunction_MathQuaternionInverse`, `RigVMFunction_MathQuaternionMul`, `RigVMFunction_MathQuaternionSwingTwist`, `RigVMFunction_MathQuaternionToEuler`, `RigVMFunction_MathDoubleRemap`, `RigVMFunction_MathDoubleGreater`, `RigVMFunction_ControlFlowBranch`

**Scale:** 1 dead subgraph (8 nodes)

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:137-143 (GetTransform/GetTransform_1 bInitial=true, Inverse, Multiply, SwingTwist TwistAxis=(X=1), ToEuler ZYX, Remap)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:476-481, 555-560 (data links; Branch.True/False -> SetCurveValue_37/38; no link into RigVMFunction_ControlFlowBranch.ExecuteContext)`

### D12 Shapes and curves

#### UE-body-corrective-morphs — Pose-driven body corrective morph targets (curve name == morph name)

*core* · assets: SKM_Zebra, SKM_Zebra_Hi, CR_Zebra_Deform

Each deform-rig curve drives the morph target of the same name, with weight equal to the curve value. SetCurveValue writes to the animation curve buffer, and UE applies curves that match morph target names as morph weights. The 49 body correctives on SKM_Zebra are:
- shoulder_up/dn/fwd/bk_l/r
- elbow_l/r and elbow_squash_l/r
- forearm_pos/neg_l/r
- clavicle_up/dn/fwd/bk_l/r
- thigh_up/ot/bk/fwd_ext_l/r, thigh_up_c, thigh_up_ext_c
- knee_l/r and knee_squash_l/r
- ankle_up/dn_l/r
- head_up, head_dn, head_side_dn_l
- Squetch
- spine_01/02/03_fwd
Of the 50 deform curves, only head_side_dn_r has no morph. Weights are usually 0..1, but Squetch can go negative.

**Setup.** None. The naming contract is <joint>_<dir>[_<qualifier>]_<side|c>.

**Operators:** `RigUnit_SetCurveValue`

**Scale:** 49 corrective morphs

**Evidence:** `<dump>/Game__Assets__Zebra__Meshes__SKM_Zebra/summary.json morph_targets (first 49 entries shoulder_up_l .. spine_03_fwd)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/hierarchy.txt (50 CURVE lines)`

#### UE6-correctives — Correctives: product (combination) shapes

*core* · assets: CRM_Zebra_Face, CRM_Monster_Face

Pin D, each curve written in execution order as a product. Zebra: smile_open_c_l = JN*smile_l; smile_open_c_r = JN*smile_r; wide_open_c_l = JN*wide_l; wide_open_c_r = JN*wide_r; wide_open_c_r = frown_r*wide_r (bug: overwrites the previous value; frown_wide_c_r is never written); frown_wide_c_l = wide_l*frown_l; frown_open_c_r = JN*frown_r; frown_open_c_l = JN*frown_l; brow_dn_c_l = brow_in_dn_l*brow_ot_dn_l; brow_dn_c_r likewise; brow_squeeze_in_dn_c_l = brow_in_dn_l*brow_squeeze_l; brow_squeeze_in_dn_c_r likewise; smile_wide_c_r = smile_r*wide_r; smile_wide_c_l = smile_l*wide_l; smile_wide_open_c_r = smile_open_c_r(read back)*wide_r; smile_wide_open_c_l likewise. JN is the member 'Jaw Normalize' = 4*jawOpenReader, which pin F sets after pin D, so the value comes from the previous evaluation. Monster changes the first pair to open_frown_c_l/r = JN*frown_l/r, so smile_wide_open_c_* reads a smile_open_c_* that Monster never writes. No min/max or clamp is used; products can exceed 1 because JN can reach 4.

**Setup.** None; this is driven by other curves.

**Operators:** `RigUnit_GetCurveValue`, `RigVMFunction_MathDoubleMul`, `RigUnit_SetCurveValue`, `RigVMVariableNode getter`

**Scale:** 16 SetCurveValue

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:1972-2123 Correctives (SetCurveValue_34 Curve=wide_open_c_r fed by frown_r*wide_r)`; `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/graphs.txt:2251 Curve=open_frown_c_l`

#### UE6-curve-morph-mapping — Face curve name to morph target mapping

*core* · assets: CRM_Zebra_Face, CRM_Monster_Face, SKM_Zebra, SKM_Monster

Curve names equal morph target names, so they drive morphs directly. Zebra writes 87 curves, and 76 match SKM_Zebra morphs. The rest are cornea_size, pupil_dilation and highlight_offset_x (eye/material), head_* and *_deformer (deformer inputs), and lip_roll_in_bt_l (no such morph). Zebra morphs never driven by the face: squint_l/r, frown_wide_c_r and the lip_all_*_backup shapes. Monster writes 107 curves, 75 of which are SKM_Monster morphs. The unmatched ones are the Zebra-only combination curves (smile_wide_c_*, wide_open_c_*, frown_open_c_*, frown_wide_c_*, brow_dn_c_*, brow_squeeze_in_dn_c_*), lip_roll_* and the deformer curves. Monster morphs never driven: all_up, all_dn, mid_up, mid_dn. Monster writes model_edits = 1.0 first thing each frame, a constant sculpt-fix morph.

**Setup.** The curves are also CURVE elements in the MR hierarchies (the Monster face also declares 96 static CURVE elements).

**Operators:** `RigUnit_SetCurveValue`, `morph target curves`

**Scale:** Zebra 76 / Monster 75 driven morphs

**Evidence:** `<dump>/Game__Assets__Zebra__Meshes__SKM_Zebra/summary.json morph_targets (132)`; `<dump>/Game__Assets__Monster__Meshes__SKM_Monster/summary.json morph_targets (79)`; `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/graphs.txt:1284 SetCurveValue_77 Curve=model_edits`

#### UE8-curve-flags-material-routing — How curves become morph weights or material parameters (curve metadata flags)

*core* · assets: SKM_Zebra, SK_Zebra, SKM_Monster, Monster_Skeleton, M_eye_eyeball_updated / MI_eye_Zeb

At evaluation, FBoneContainer builds CurveFlags from Skeleton->ForEachCurveMetaData (bMaterial gives Material, bMorphtarget gives MorphTarget) and unions them with the skeletal mesh's UAnimCurveMetaData asset user data. FAnimInstanceProxy::UpdateCurvesToEvaluationContext sends a curve to the MorphTargetCurve list only if it is flagged MorphTarget, and to the MaterialCurve list (scalar material parameter of the same name on all component materials) if it is flagged Material. Material parameters that were set last frame but not this frame are reset to the material's default. SK_Zebra's own AnimCurveMetaData has no entries (the curve_meta_data property no longer exists, hence 'ERR sc'). Monster_Skeleton flags only 4 curves (smile_l, brow_dn_l, stitches, brow_up_l) as bMorphtarget. The SKM_Zebra package name table does contain curve names that are not morphs (cornea_size, pupil_dilation, highlight_offset_x, highlight_radius, highlight_softness, head_side_dn_r, lip_roll_in_bt_l, deformer curves) along with 'bMaterial' and 'bMorphtarget'. So the mesh-level AnimCurveMetaData likely carries the flags: morph flags for the 132 morphs and material flags for the eye curves, which feed the Eyes-slot material MI_eye_Zeb. This is not dumped and needs a re-dump of SkeletalMesh AnimCurveMetaData. Separately, the expression_demo_seq sequence animates Eyes-slot material parameters directly (Highlight_Intensity, Cornea Roughness, Sclera Color Multiply, Highlight_Shape_Pos) with a ComponentMaterialTrack, outside the rig.

**Setup.** Eye controls (Pupil Size, Irisl Size, highlight) write the material curves.

**Operators:** `FBoneContainer::CacheRequiredAnimCurves`, `FAnimInstanceProxy::UpdateCurvesToEvaluationContext`, `USkeletalMeshComponent::ApplyAnimationCurvesToComponent`, `UAnimCurveMetaData`, `UMovieSceneComponentMaterialTrack`

**Scale:** 132 Zebra morphs, 5 eye material curves; Monster 79 morphs

**Evidence:** `<UE>/Source/Runtime/Engine/Private/BoneContainer.cpp:285-305, 358-385`; `<UE>/Source/Runtime/Engine/Private/Animation/AnimInstanceProxy.cpp:3575-3622`; `<dump>/Game__Assets__Monster__Meshes__Monster_Skeleton/asset.t3d :: 'CurveMetaData=(("smile_l", (Type=(bMorphtarget=True))),...,("stitches", ...'`; `<ZebraSample>/Content/Assets/Zebra/Meshes/SKM_Zebra.uasset name table: 'bMaterial','bMorphtarget','cornea_size','highlight_radius','highlight_softness','MI_eye_Zeb'`; `<dump>/Game__Sequences__expression_demo_seq/sequence.txt :: 'MovieSceneComponentMaterialTrack name=Material Slot: Eyes'`; `<dump>/_log.txt:22, 45, 47, 110, 112 (ERR sc)`

#### UE-combo-min-corrective — Combination (both-legs) corrective via Min of two reader outputs

*important* · assets: CR_Zebra_Deform, SKM_Zebra

thigh_up_c = min(thigh_up_l reader L5, thigh_up_r reader L4), computed by Minimum and written by SetCurveValue_50. thigh_up_ext_c = min(Remap_5, Remap_6) of the thresholded fwd readers, computed by Minimum_1 and written by SetCurveValue_49. Both curves drive center morph targets (thigh_up_c, thigh_up_ext_c) that only activate when both thighs are raised. This is a set-driven combination shape built from the minimum of the two inputs.

**Setup.** None.

**Operators:** `RigVMFunction_MathDoubleMin`, `RigUnit_SetCurveValue`

**Scale:** 2 Min nodes, 2 combo curves

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:232-240 (Minimum, Minimum_1, SetCurveValue_49/50)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:603-613 (OutputParam -> Minimum.A/B; Remap_5/6 -> Minimum_1.A/B)`; `<dump>/Game__Assets__Zebra__Meshes__SKM_Zebra/summary.json morph_targets thigh_up_c, thigh_up_ext_c`

#### UE-face-morph-inventory-zebra — Zebra face shape library incl. combination and micro shapes

*important* · assets: SKM_Zebra, SKM_Zebra_Hi

SKM_Zebra has 83 non-body morphs, which the face rig drives through curves.
- Primaries: jaw_open, smile, wide, frown, narrow, curl_up, squint, lid_tp_blink, brow_in/ot_up/dn, brow_squeeze, brow_mid_up/dn/side, sneer_tp/bt_up/dn, lip_all/mid up/dn tp/bt, lip_roll_in/ot_tp_l, mouth_squash, muzzle_squash.
- Micro shapes: brow_*_micro, brow_side_micro, brow_ot_side.
- 16 combination shapes with a '_c_' infix: smile_wide_c, smile_open_c, wide_open_c, frown_wide_c, frown_open_c, smile_wide_open_c, brow_dn_c, brow_squeeze_in_dn_c, each as _l/_r.
- Leftover 'backup' sculpts: lip_all_up_backup, _2, _3 and lip_all_dn_backup.
SKM_Zebra_Hi has 130 morphs: the same set without the 4 lip backups, plus brow_in_up_l_backup and brow_in_dn_l_backup. Its morph order is reversed.

**Setup.** Combination shapes use the <a>_<b>[_<c>]_c_<side> naming.

**Scale:** 83 (Zebra) / 81 (Zebra_Hi) face morphs; 16 combination shapes

**Evidence:** `<dump>/Game__Assets__Zebra__Meshes__SKM_Zebra/summary.json morph_targets (jaw_open ... squint_r; smile_wide_c_l etc.)`; `<dump>/Game__Assets__Zebra__Meshes__SKM_Zebra_Hi/summary.json morph_targets (brow_in_up_l_backup, brow_in_dn_l_backup)`

#### UE-monster-morphs — Monster face-only morph inventory (79) and naming

*important* · assets: SKM_Monster

SKM_Monster has 79 morphs and no body correctives. Most names are shared with Zebra (brows incl. micro, smile/wide/narrow/frown, sneer, curl, lip_all/mid, muzzle_squash, mouth_squash, jaw_open). Monster-only shapes: lip_stick_l/r, nose_flare_l/r, nose_sneer_l/r, lid_bt_blink_l/r, lid_tp_blink_extend_l/r, lip_puff_tp/bt, mid_up/dn, all_up/dn, ch_bt, model_edits, and open_frown_c_l/r (combination order reversed relative to Zebra's frown_open_c).

**Setup.** None.

**Scale:** 79 morphs

**Evidence:** `<dump>/Game__Assets__Monster__Meshes__SKM_Monster/summary.json morph_targets`; `<dump>/Game__Assets__Monster__Meshes__SKM_Monster/monster_usd_tree.txt BINDING /SKM_Monster/SKM_Monster blendShapes=79`

#### UE6-brow-micro-curves — Brow Micro function: micro pad to up/down/side curves

*important* · assets: CRM_Zebra_Face, CRM_Monster_Face

Brow Micro(Item) takes v = local translation of the micro control and returns brow_in_up = clamp(0.01*v.z, 0, 200), brow_in_dn = clamp(-0.01*v.z, 0, 200) and brow_side = 0.01*v.x (unclamped). Six calls write: In R -> brow_in_up_micro_r, brow_in_dn_micro_r, brow_side_micro_r; Mid R -> brow_mid_up_r, brow_mid_dn_r, brow_mid_side_r; Ot R -> brow_ot_up_micro_r, brow_ot_dn_micro_r, brow_ot_side_r; and the same for L.

**Setup.** Brow In/Mid/Ot L/R micro controls.

**Operators:** `RigUnit_GetTransform`, `RigVMFunction_MathDoubleMul`, `RigVMFunction_MathDoubleRemap`, `RigUnit_SetCurveValue`

**Scale:** 6 calls, 18 curves

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:2190-2212 Brow Micro`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:854 SetCurveValue_23 Curve=brow_in_up_micro_r`

#### UE6-lid-curves — Lid blink curve outputs

*important* · assets: CRM_Zebra_Face, CRM_Monster_Face

Zebra writes lid_tp_blink_l = Blink Logic_2 result (Lid Tp L) and lid_tp_blink_r = Blink Logic result (Lid Tp R). Monster additionally writes lid_bt_blink_l / lid_bt_blink_r (Bt blink weights), lid_tp_blink_extend_l / lid_tp_blink_extend_r (extend weights), and lid_bt_blink_extend_l, which is set twice (L, then the R weight overwrites it). As a result lid_bt_blink_extend_r is never written, and the Monster mesh lacks lid_bt_blink_extend morphs anyway. The curve equals the clamped remap weight (0..1).

**Setup.** Lid Tp/Bt sliders.

**Operators:** `RigUnit_SetCurveValue`

**Scale:** Zebra 2 curves, Monster 7 writes

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:895 SetCurveValue_63 Curve=lid_tp_blink_l`; `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/graphs.txt:1314-1315 SetCurveValue_82/_83 Curve=lid_bt_blink_extend_l`

#### UE6-squetch-curves — Mouth/Muzzle Squetch channels to squash morph curves

*important* · assets: CRM_Zebra_Face, CRM_Monster_Face

mouth_squash = clamp(0.01*MouthSquetch, -2, 2) and muzzle_squash = clamp(0.01*MuzzleSquetch, -2, 2). Both are signed and drive morphs mouth_squash and muzzle_squash (present on both meshes). These run at the end of pin A.

**Setup.** jaw channels Mouth Squetch and Muzzle Squetch (-200..200).

**Operators:** `RigUnit_GetFloatAnimationChannelFromItem`, `RigVMFunction_MathDoubleMul`, `RigVMFunction_MathDoubleClamp`, `RigUnit_SetCurveValue`

**Scale:** 2 curves

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:784 Clamp_24 Minimum=-2 Maximum=2`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:785 SetCurveValue_57 Curve=mouth_squash`

#### UE7-deformer-curves-registry — Deformer driver curves as skeleton or rig curves

*important* · assets: MR_Zebra, MR_ZebraDMC, CR_Zebra_Deform, CR_Monster_Deform, Zeb_Face_Expressions

The deformer factors travel as named float curves in the rig hierarchy and the animation. They are not morph targets.
- Zebra curves: head_squash, head_twist, head_bend, muzzle_squash_deformer, muzzle_bend_deformer, skull_tp_squash_deformer, skull_tp_bend_deformer. They sit in the MR_Zebra hierarchy (imported from the skeleton) next to unrelated corrective curves such as muzzle_squash and mouth_squash.
- Monster curves: head_bend_deformer, head_squash_deformer, head_twist_deformer, mouth_bend_deformer, mouth_squash_deformer, muzzle_bend_deformer, and so on (CR_Monster_Deform hierarchy).
- Zeb_Face_Expressions (163 frames) keys head_bend, head_squash, muzzle_bend_deformer, muzzle_squash_deformer, skull_tp_bend_deformer and skull_tp_squash_deformer. It does not key head_twist.

A usdRig port needs these as animatable scalar attributes that feed the deformer operator.

**Setup.** The curves are channels on the rig; they can be animated directly or computed from the squash controls.

**Operators:** `FRigCurveElement`, `AnimSequence float curves`

**Scale:** 7 Zebra and 9 Monster curves.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/hierarchy.txt:1319-1330 (CURVE head_bend ... skull_tp_bend_deformer)`; `<dump>/Game__Assets__Monster__Rig__CR_Monster_Deform/hierarchy.txt:187-216`; `<dump>/Game__Assets__Zebra__Anims__Zeb_Face_Expressions/summary.json 'float_curves'`

#### UE8-curve-morph-name-bugs — Rig-written curves without morphs, morphs never written, and copy-paste curve bugs

*important* · assets: CRM_Zebra_Face, CR_Zebra_Deform, CRM_Monster_Face, SKM_Zebra, SKM_Monster

Zebra writes 137 SetCurveValue curves (87 names in the face module, 50 in the deform rig). Written but not a morph on SKM_Zebra: cornea_size, pupil_dilation, highlight_offset_x (eye material curves); head_bend, head_squash, head_twist and the muzzle/skull_tp *_deformer curves (read back by CR_Zebra_Deform for the Optimus deformers); head_side_dn_r (a SphericalPoseReader on head writes both head_side_dn_l and head_side_dn_r, but only head_side_dn_l exists as a morph); lip_roll_in_bt_l. Bug: the 'Lip Roll Ot Bt Funct' function writes 'lip_roll_in_bt_l', the same name as 'Lip Roll In Bt Funct', and neither bottom lip-roll morph exists (only lip_roll_in_tp_l and lip_roll_ot_tp_l). Bug: the 'Correctives' function writes wide_open_c_r twice and never frown_wide_c_r, although frown_wide_c_l is written and a frown_wide_c_r morph exists; the second node was probably meant to be frown_wide_c_r. squint_l and squint_r are morphs that nothing writes. Monster: the face module writes 107 curve names, 32 of them not morphs on SKM_Monster (correctives such as smile_wide_c_l/r, wide_open_c_l/r, frown_open_c_l/r, lip_roll_*, eye material and deformer curves), and never writes the morphs all_up, all_dn, mid_up, mid_dn. CR_Monster_Deform writes no curves. The MR_Zebra static curve set (958 curves, including 247 MetaHuman CTRL_expressions_*, Male_Med_Head_*, Pose_0-9, bone-named curves such as 'pelvis' and 'root', and odd duplicates clavicle_r_back_310 / clavicle_r_up_401 next to clavicle_r_back_30 / clavicle_r_up_40) comes from skeleton import and is mostly unused by the rig.

**Setup.** Face controls (Lip Roll, Corner correctives) drive these curves.

**Operators:** `RigUnit_SetCurveValue`, `RigUnit_GetCurveValue`, `RigUnit_SphericalPoseReader`

**Scale:** Zebra: 12 non-morph written names, 7 unwritten morphs (lo); Monster: 32 / 4

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:2700-2710 (Lip Roll Ot Bt Funct: SetCurveValue_41 Curve=lip_roll_in_bt_l), 2783-2790 (Lip Roll In Bt Funct: Curve=lip_roll_in_bt_l), 2007 (frown_wide_c_l), 2008 and 2048 (wide_open_c_r x2)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:133-136 (SetCurveValue_28 head_side_dn_l, SetCurveValue_29 head_side_dn_r, SphericalPoseReader_1_1_1_1_15 DriverItem=head)`; `<dump>/Game__Assets__Zebra__Meshes__SKM_Zebra/summary.json:33 (head_side_dn_l only), :85 (frown_wide_c_r), :135 (squint_l)`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/hierarchy.txt:19-20 (clavicle_r_back_310, clavicle_r_up_401), :1214, :1331-1332`; `<dump>/Game__Assets__Monster__Meshes__SKM_Monster/summary.json :: morph_targets (all_up, all_dn, mid_up, mid_dn)`

#### UE1-curve-set — Imported curve set (Fortnite standard + character-specific)

*nice-to-have* · assets: MR_Zebra, MR_FN_Biped, MR_Monster

The curves imported into each rig are the skeleton's curve list. The Biped template's 800 curves are a strict subset of Zebra's 958. They include 247 'CTRL_*' (MetaHuman-style expression controls), 32 'WM*', corrective pose-reader curve names such as calf_l_back_50/90/120/150, clavicle_l_fwd_30, foot_l_up_35 and hand_l_up_90, Pose_0..9, and MoveData_Speed / base_pose. Zebra adds 158 character curves (brow_*, lip_*, sneer_*, smile_*, frown_*, head_*, ...). Monster has 993 (brow 42, lip 22, lid 9, ...). The modular host itself does not read these; face/deform modules and ABPs do.

**Operators:** `CURVE elements`

**Scale:** 958 in Zebra, 800 in Biped, 993 in Monster. All 800 Biped curve names also appear in Zebra.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/hierarchy.txt:1 'CURVE MoveData_Speed'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/hierarchy.txt:3 'CURVE calf_l_back_50'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/summary.json: 'CURVE': 958`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/summary.json: 'CURVE': 800`

#### UE8-morph-set-mismatch — Low-res vs high-res Zebra morph sets differ

*nice-to-have* · assets: SKM_Zebra, SKM_Zebra_Hi

SKM_Zebra has 132 morph targets and SKM_Zebra_Hi has 130. Only in the low-res mesh: lip_all_dn_backup, lip_all_up_backup, lip_all_up_backup_2, lip_all_up_backup_3. Only in the high-res mesh: brow_in_dn_l_backup, brow_in_up_l_backup. The rig never writes any of these 'backup' shapes, so both meshes deform the same from the rig. A USD port that exports blend shapes per mesh must still carry the per-mesh differences, and should not assume one blendShape list for both.

**Setup.** none

**Operators:** `UMorphTarget`

**Scale:** 6 differing morphs

**Evidence:** `<dump>/Game__Assets__Zebra__Meshes__SKM_Zebra/summary.json:93 ('lip_all_up_backup')`; `<dump>/Game__Assets__Zebra__Meshes__SKM_Zebra_Hi/summary.json:77 ('brow_in_dn_l_backup')`

### D13 GPU deformers

#### UE-deformer-curve-bus — Curve bus between animator face rig and post-process deform rig for deformers

*core* · assets: CRM_Zebra_Face, CR_Zebra_Deform, Zeb_Face_Expressions

The animator face module (CRM_Zebra_Face) converts deformer-control local transforms into normalized curves with Remap (no clamp):
- head_squash = T.Z mapped -100..100 -> -1..1.
- head_twist = Euler Z mapped -135..135 -> 1..-1.
- head_bend = T.X mapped -100..100 -> 1..-1.
- muzzle_squash_deformer has two writers: SetCurveValue_65 (from the head-squash remap) and SetCurveValue_66 (muzzle control T.Z -30..30 -> -1..1).
- muzzle_bend_deformer = T.X mapped 20..-20 -> 1..-1.
- skull_tp_squash_deformer = T.Z mapped -100..100 -> -1..1.
- skull_tp_bend_deformer = T.X mapped 50..-50 -> -1..1.

These curves are baked into animation (Zeb_Face_Expressions contains head_bend, head_squash, muzzle_*_deformer and skull_tp_*_deformer). The post-process rig reads them with GetCurveValue. So the deformers work both from live rig evaluation and from baked animation, with curves as the only interface.

**Setup.** Face-rig deformer controls (variables 'Head Squash', 'Muzzle Squash', 'Skull Tp Squash' ...) drive the curves through local translation ranges of about ±100 cm (head/skull), ±30/±20 cm (muzzle) and ±135 deg (head twist).

**Operators:** `RigUnit_SetCurveValue`, `RigUnit_GetCurveValue`, `RigVMFunction_MathDoubleRemap`, `RigUnit_GetTransform`

**Scale:** 7 deformer curves

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:1104-1126 (SetCurveValue head_squash/head_twist/head_bend/muzzle_*_deformer/skull_tp_*_deformer; Remap_1..8 ranges)`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:1724-1747 (GetTransform_1.Transform.Translation.Z -> Remap_1.Value, ...)`; `<dump>/Game__Assets__Zebra__Anims__Zeb_Face_Expressions/summary.json float_curves (head_bend, head_squash, muzzle_squash_deformer, skull_tp_bend_deformer ...)`

#### UE-optimus-add-deformer-op — Add Deformer rig unit: runtime layering of Optimus deformer graphs on skinning

*core* · assets: CR_Zebra_Deform, CRM_Monster_Face

RigUnit_AddOptimusDeformer is a mutable unit configured through traits:
- DeformerGraphAsset: the UOptimusDeformer soft reference.
- Settings: ExecutionPhase (AfterDefaultDeformer=0, OverrideDefaultDeformer=1, BeforeDefaultDeformer=2), ExecutionGroup (int), DeformChildComponents (bool) and ExcludeChildComponentsWithTag.
- One SetDeformer<Type>Variable trait per deformer variable, named after the variable, holding a Value.

Each execution does three things:
1. It enqueues the node's instance GUID into the component's UOptimusDeformerDynamicInstanceManager (LOD0) for the given phase and group.
2. On the game thread it adds the producer deformer instance if missing. If the component has no mesh deformer it calls SetAlwaysUseMeshDeformer(true), which falls back to the project default deformer. When DeformChildComponents is set, this also applies to child skeletal mesh components.
3. It pushes every variable-trait value into that instance.

The manager runs phases in the order Before, Override (only the last instance, else the default), After. Within a phase, groups are sorted ascending and instances keep enqueue order. The queue is cleared after each frame, so a deformer only runs on frames where the unit executed.

**Setup.** The variable traits are generated by the 'Refresh Variables' workflow from the deformer's variable list.

**Operators:** `RigUnit_AddOptimusDeformer`, `RigVMTrait_OptimusDeformer`, `RigVMTrait_OptimusDeformerSettings`, `RigVMTrait_SetDeformerFloatVariable`, `RigVMTrait_SetDeformerTransformVariable`

**Scale:** 16 Add Deformer nodes (7 Zebra, 9 Monster)

**Evidence:** `<UE>/Plugins/Animation/DeformerGraph/Source/OptimusCore/Private/ControlRig/RigUnit_Optimus.h:16-79`; `<UE>/Plugins/Animation/DeformerGraph/Source/OptimusCore/Private/ControlRig/RigUnit_Optimus.cpp:237-420`; `<UE>/Plugins/Animation/DeformerGraph/Source/OptimusCore/Private/OptimusDeformerDynamicInstanceManager.cpp:46-145 (EnqueueWork phase/group ordering)`; `<UE>/Plugins/Animation/DeformerGraph/Source/OptimusCore/Public/OptimusDeformerDynamicInstanceManager.h:19-24`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/regen.py:2228-2231 add_trait(... RigVMTrait_OptimusDeformer / Settings / SetDeformerTransformVariable 'Transform' / SetDeformerFloatVariable 'StretchFactor')`

#### UE-zebra-deformer-stack — Zebra head/muzzle/skull GPU deformer stack driven by curves and pivot nulls

*core* · assets: CR_Zebra_Deform

Sequence.B adds 7 deformers in this order. All use ExecutionPhase=AfterDefaultDeformer, ExecutionGroup=1, DeformChildComponents=True and no exclude tag, so they run after linear skinning in add order.
1. ZebraHead_DeformerGraph (squash/stretch): StretchFactor = Remap_7(curve head_squash, -1..1 -> 0..1) = (v+1)/2. Transform = global transform of 'Head Squash Null'.
2. ZebraHeadTwist_DeformerGraph: TwistFactor = head_twist, used directly. Transform = 'Head Twist Null'.
3. ZebraHeadBend_DeformerGraph: BendFactor = head_bend. Transform = 'Head Bend Null'.
4. ZebraMuzzleSquash_DeformerGraph: StretchFactor = Remap_8(muzzle_squash_deformer, 1..-1 -> 0..1) = (1-v)/2, which is inverted. Transform = 'Muzzle Squash Null'.
5. ZebraMuzzleBend_DeformerGraph: BendFactor = muzzle_bend_deformer. Transform = 'Muzzle bend Null'.
6. ZebraSkullTpSquash_DeformerGraph: StretchFactor = Remap_9(skull_tp_squash_deformer, -1..1 -> 0..1). Transform = 'Skull Tp Squash Null'.
7. ZebraSkullTpBend_DeformerGraph: BendFactor = skull_tp_bend_deformer. Transform = 'Skull Tp Bend Null'.

Each Transform comes from GetTransform(null key variable, GlobalSpace, current), so the deformer frame follows the animated head. The unlinked defaults (StretchFactor 0.5, Bend/Twist 0, Transform = null init) only apply if a link is missing.

**Setup.** There are no controls in this rig. The animator drives the curves from face controls (see UE-deformer-curve-bus).

**Operators:** `RigUnit_AddOptimusDeformer`, `RigUnit_GetCurveValue`, `RigVMFunction_MathDoubleRemap`, `RigUnit_GetTransform`, `RigVMFunction_Sequence`

**Scale:** 7 deformers, 7 curves, 7 nulls

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:256-286 (AddOptimusDeformer.._6, GetCurveValue head_squash/head_twist/head_bend/muzzle_squash_deformer/muzzle_bend_deformer/skull_tp_squash_deformer/skull_tp_bend_deformer, Remap_7/8/9)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:647-677 (Sequence_1.A -> AddOptimusDeformer -> _1 -> _3 -> _2 -> _4 -> _5 -> _6; GetCurveValue -> Remap_7 -> StretchFactor.Value; GetTransform_2.Transform -> AddOptimusDeformer.Transform.Value)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:271 Remap_8 SourceMinimum=1.000000; SourceMaximum=-1.000000`

#### UE7-add-optimus-deformer-unit — RigUnit_AddOptimusDeformer (rig-driven deformer layering)

*core* · assets: CR_Zebra_Deform, CRM_Monster_Face

Registers and drives a deformer instance from Control Rig. The node has traits 'DeformerGraphAsset' (FRigVMTrait_OptimusDeformer), 'Settings' (FRigVMTrait_OptimusDeformerSettings) and one FRigVMTrait_SetDeformer<Type>Variable per deformer variable. The trait name must equal the variable name, and the trait's Value pin is animatable.

Execute, on every evaluation:
1. Create DeformerInstanceGuid once (hidden, persistent).
2. Build the component list: the owning SkeletalMeshComponent, plus all child SkeletalMeshComponents (recursive) if DeformChildComponents is set. Children with tag ExcludeChildComponentsWithTag are skipped, and so are editor-preview actors that belong to another actor.
3. On the anim thread, for each component: LOD0 MeshDeformerInstance must be a UOptimusDeformerDynamicInstanceManager; call EnqueueProducerDeformer(guid, ExecutionPhase, ExecutionGroup).
4. On the game thread (async-loading the asset if needed): if the component has no mesh deformer instance, wait for parallel anim and call SetAlwaysUseMeshDeformer(true), which falls back to the project default deformer. Then AddProducerDeformer(rig, guid, graph) if no instance exists for that guid.
5. For each variable trait: if the instance exists, call instance->Set<Type>Variable(traitName, Value).

One deformer instance is shared by all LODs.

Editor workflow 'Refresh Variables' rebuilds the variable traits from the deformer's variable list, mapping each Optimus data type to a trait struct. It supports int, int2/3/4, float/double, vector2/3/4, linear colour, quat, rotator, transform, name and bool, plus arrays. The trait defaults come from the deformer variable defaults.

**Setup.** Every node here uses Settings=(ExecutionPhase=AfterDefaultDeformer, ExecutionGroup=1, DeformChildComponents=True, ExcludeChildComponentsWithTag=None). The Transform trait is linked from GetTransform(null, GlobalSpace); the factor trait is linked from a curve (remapped for squash).

**Operators:** `RigUnit_AddOptimusDeformer`, `RigVMTrait_OptimusDeformer`, `RigVMTrait_OptimusDeformerSettings`, `RigVMTrait_SetDeformerTransformVariable`, `RigVMTrait_SetDeformerFloatVariable`, `UOptimusDeformerDynamicInstanceManager`

**Scale:** 16 nodes (7 in CR_Zebra_Deform, 9 in CRM_Monster_Face).

**Evidence:** `<UE>/Plugins/Animation/DeformerGraph/Source/OptimusCore/Private/ControlRig/RigUnit_Optimus.h:15-79`; `<UE>/Plugins/Animation/DeformerGraph/Source/OptimusCore/Private/ControlRig/RigUnit_Optimus.cpp:237 (Execute), :319 EnqueueProducerDeformer, :358 SetAlwaysUseMeshDeformer(true), :366 AddProducerDeformer, :411 VariableTrait->SetValue`; `RigUnit_Optimus.cpp:58-229 (Refresh Variables workflow)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/regen.py:2227-2231 (add_unit_node + add_trait ... 'RigVMTrait_SetDeformerTransformVariable','Transform' / 'RigVMTrait_SetDeformerFloatVariable','StretchFactor')`; `<dump>/units_used.json:56 '"RigUnit_AddOptimusDeformer": 16'`

#### UE7-deformer-stack-ordering — Deformer stack execution order (phase/group/enqueue order) on top of default skinning

*core* · assets: CR_Zebra_Deform, CRM_Monster_Face

The dynamic instance manager builds the dispatch order each frame:
1. BeforeDefaultDeformer instances.
2. The OverrideDefaultDeformer slot. Only the last-enqueued instance in the highest group is used; if there is none, the DefaultInstance runs.
3. AfterDefaultDeformer instances.
Within a phase, groups are sorted ascending and instances run in enqueue order. Re-enqueuing the same guid within a frame blanks the earlier entry. The queue is reset after each frame, so a deformer only runs on frames where its rig unit executed.

Each instance's Read Skinned Mesh sees the output of the previous one. The meshes have no default_mesh_deformer, so the default comes from DeformerGraph settings:
- DefaultDeformer = DG_LinearBlendSkin_Morph_Cloth (linear-blend skinning, morph targets and cloth run first).
- DefaultRecomputeTangentDeformer = DG_LinearBlendSkin_Morph_Cloth_RecomputeNormals.

Deformer order is significant because bend, twist and squash do not commute:
- Zebra: HeadSquash, HeadTwist, HeadBend, MuzzleSquash, MuzzleBend, SkullTpSquash, SkullTpBend.
- Monster: HeadSquash, HeadBend, HeadTwist, MuzzleBend, MuzzleSquash, MouthBend, MouthSquash, SkullTpBend, SkullTpSquash.
In words, Zebra twists before bending and squashes before bending, while Monster bends before twisting and bends before squashing.

**Setup.** None for the animator; the order is fixed by the exec wiring.

**Operators:** `UOptimusDeformerDynamicInstanceManager::EnqueueWork/EnqueueProducerDeformer`, `EOptimusDeformerExecutionPhase {AfterDefaultDeformer=0, OverrideDefaultDeformer=1, BeforeDefaultDeformer=2}`, `UOptimusSettings DefaultDeformer`

**Scale:** 2 chains (7 and 9 deformers).

**Evidence:** `<UE>/Plugins/Animation/DeformerGraph/Source/OptimusCore/Private/OptimusDeformerDynamicInstanceManager.cpp:65-111 (phase loop), :139 'ExecutionQueueMap.Reset();', :256-270 (EnqueueProducerDeformer dedupe)`; `<UE>/Plugins/Animation/DeformerGraph/Source/OptimusCore/Public/OptimusDeformerDynamicInstanceManager.h:19-24`; `<UE>/Plugins/Animation/DeformerGraph/Config/DefaultDeformerGraph.ini:45-47 '+DefaultDeformer=/DeformerGraph/Deformers/DG_LinearBlendSkin_Morph_Cloth...'`; `<dump>/Game__Assets__Zebra__Meshes__SKM_Zebra/summary.json: 'default_mesh_deformer': 'None'`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:649-674 (ExecutePin chain)`; `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/graphs.txt:2012-2051 (ExecutePin chain)`

#### UE7-dg-barr-bend — Barr bend deformer kernel (DG_Function_Bend) with region limits

*core* · assets: ZebraHeadBend_DeformerGraph, ZebraMuzzleBend_DeformerGraph, ZebraSkullTpBend_DeformerGraph, Monster_HeadBend_DeforerGraph, Monster_MuzzleBend_DeforerGraph, Monster_MouthBend_DeforerGraph, Monster_SkullTpBend_DeforerGraph

Runs per vertex on skinned positions P (component space). Inputs: OriginTransform M (float4x4 from an FTransform), LengthToBend L, MaxBendAngle A (int degrees), BendFactor f in [-1,1], WeightMap w, Optional_LimitFromBottom lb, Optional_LimitFromTop lt, Optional_EnableDebugDraw.
1. Radian = lerp(-A, A, (f+1)/2) in radians, which equals f*A.
2. Transform to the local frame: p = inverse_affine(M)*P.
3. Limits: Lower = clamp(lb, 1e-4, 1-1e-4)*L and Upper = max(Lower+1e-4, clamp(1-lt, 1e-4, 1-1e-4)*L).
4. zc = clamp(p.z, Lower, Upper).
5. If |Radian| > 1e-4: R = (Upper-Lower)/Radian and theta = (zc-Lower)/R.
   - Outside the limits: if p.z < Lower, d = p.z-Lower; if p.z > Upper, d = p.z-Upper. Then adjZ = cos(theta)*d and adjY = sin(theta)*d.
   - x' = p.x
   - y' = cos(theta)*(p.y-R) + R + adjY
   - z' = -sin(theta)*(p.y-R) + Lower + adjZ
   - P' = M*p'.
6. Final position = lerp(P, P', w). Vertices with w < 1e-4 are skipped.
Bend axis = local X (the rotation axis); bend direction = local Y (the centre of curvature is at y=R); the deformation runs along local Z. Below Lower the mesh is unchanged. Above Upper the mesh follows as a rigid body, continuing along the end tangent. In every use here lb=lt=0, so Lower = 0.0001*L and Upper = 0.9999*L. Output is OutPosition only (Vertex domain).

**Setup.** The animator drives this through BendFactor, which comes from the '*_bend*' curve. That curve is driven by the squash control's local translate X (see UE7-deformer-control-channel-mapping). The deform frame comes from a 'Bend Null' parented under the head. Constants baked in the graph: L=90 and MaxBendAngle=180 in all bend graphs.

**Operators:** `OptimusNode_FunctionReference -> /DeformerGraph/DeformerFunctions/DG_Function_Bend`, `OptimusNode_CustomComputeKernel 'Bend' (entry Bend_OptimusNode_CustomComputeKernel_2)`, `DSL_Matrix (MatrixInverse_Affine, MatrixTransformPosition)`

**Scale:** 7 graphs (3 Zebra, 4 Monster). 6 AddOptimusDeformer instances use them (3 Zebra, 3 Monster... Monster has 4 bend: head/muzzle/mouth/skull).

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraHeadBend_DeformerGraph/kernels.hlsl:54-166 (BendDeform)`; `kernels.hlsl:62 'float Radian = lerp(-MaxRadian, MaxRadian, (BendFactor + 1) / 2);'`; `kernels.hlsl:84 'float R = (UpperLimit-LowerLimit) / Radian;'`; `kernels.hlsl:161 'NewPosition = lerp(Position, NewPosition, Weight);'`; `<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraHeadBend_DeformerGraph/asset.t3d: AdditionalSources(0)=...DSL_Matrix`; `kernel source hash identical across all 7 bend graphs (md5 50624192 after normalizing names)`

#### UE7-dg-barr-twist — Barr twist deformer kernel (DG_Function_Twist)

*core* · assets: ZebraHeadTwist_DeformerGraph, ZebraMuzzleTwist_DeformerGraph, Monster_HeadTwist_DeformerGraph

Inputs: OriginTransform M, LengthToTwist L, MaxTwistAngle A (int degrees), TwistFactor f in [-1,1], WeightMap w, limits lb and lt.
1. Radian = lerp(-A, A, (f+1)/2).
2. p = inverse_affine(M)*P.
3. Lower = lb*L and Upper = (1-lt)*L. Unlike Bend and Squash, these values are not sanitized.
4. zc = clamp(p.z, Lower, Upper) and theta = (zc-Lower)/(Upper-Lower)*Radian.
5. RotationMatrix = float2x2(cos, -sin, sin, cos) and xy' = mul(p.xy, RotationMatrix). With HLSL row-vector multiplication this gives (x*cos + y*sin, -x*sin + y*cos), a rotation by -theta about local Z.
6. z' = p.z and P' = M*p'.
7. Output = lerp(P, P', w), with w < 1e-4 skipped.
The twist ramps linearly from 0 at z=Lower to the full Radian at z=Upper and stays constant above Upper. Below Lower there is no change. Debug draw shows the lower plane and a rotated upper plane.

**Setup.** TwistFactor comes from curve head_twist (Zebra) or head_twist_deformer (Monster). That curve equals -rotZ/135, where rotZ is the Head Squash control's local Euler Z (ZYX order). With A=135 the kernel applies a mesh rotation of +rotZ degrees. Constants: Zebra head L=60; Monster head L=175; A=135 everywhere.

**Operators:** `DG_Function_Twist (kernel Twist_OptimusNode_CustomComputeKernel_2)`, `DSL_Matrix`

**Scale:** 3 graphs. 2 are used (ZebraHeadTwist, Monster_HeadTwist); ZebraMuzzleTwist is unused.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraHeadTwist_DeformerGraph/kernels.hlsl:50-146`; `kernels.hlsl:58 Radian lerp`; `kernels.hlsl:77 'float Theta = (ClampedZ - LowerLimit) / (UpperLimit - LowerLimit) * Radian;'`; `kernels.hlsl:85 'NewLocalPosition.xy = mul(LocalPosition.xy , RotationMatrix);'`

#### UE7-dg-deform-frame-convention — Deformer local frame convention (OriginTransform)

*core* · assets: all 17 deformer graphs

Each kernel works in the local frame of the 'Transform' variable. The FTransform is converted to a float4x4 row matrix (the GraphDataInterface packs it at buffer offset 32). Positions from Read Skinned Mesh are in component (mesh) space, so the transform must also be in component space; Control Rig GlobalSpace equals component space.

Frame layout:
- The deformation axis is local +Z, starting at the origin (row _41_42_43) and extending L units along row _31_32_33.
- Bend: local X (row _11) is the bend axis and local Y (row _21) is the bend direction.
- Squash: local X and Y are the bulge axes.
- The inverse is computed per vertex with MatrixInverse_Affine, so scale and shear are supported.

Debug-draw colour convention: a blue line along the capture direction, blue quads at the limits, a red line for X, and a green line for Y.

**Setup.** The rig passes GetTransform(<X Null>, GlobalSpace) into the Transform trait every frame, so each deformer frame follows the head bone.

**Operators:** `OptimusNode_GetVariable (FTransform)`, `OptimusNode_ConstantValue_FTransform (Value pin linked from variable)`, `OptimusGraphDataInterface`

**Scale:** 17 graphs; 16 runtime instances.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraHeadBend_DeformerGraph/kernels.hlsl:113-116 (CaptureOrigin=_41_42_43, CaptureDirection=_31_32_33, BendAxis=_11_12_13, BendDirection=_21_22_23)`; `<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraHeadBend_DeformerGraph/asset.t3d: 'Variables(5)=(Name="Transform"...Offset=32)', 'ParameterBufferSize=112'`; `<UE>/Plugins/Animation/DeformerGraph/Source/OptimusCore/Private/DataInterfaces/OptimusDataInterfaceSkinnedMeshRead.h:23`

#### UE7-dg-pipeline-structure — Common deformer-graph pipeline and data interfaces

*core* · assets: all 17 deformer graphs

Every graph has a single UpdateGraph with one primary component binding, 'Primary' (OptimusSkeletalMeshComponentSource). The pipeline is:

Read Skinned Mesh (Position, TangentX, TangentZ, Color; the current state of the mesh, possibly already deformed by earlier deformers)
  -> [CacheGeometry]
  -> Deform function (Bend / Twist / SquashStretch), whose OutPosition goes to a transient buffer
  -> ComputeNormalsTangentsAndKeepInputNormals (OutPosition, OutTangentX, OutTangentZ)
  -> Write Skinned Mesh (Position, TangentX, TangentZ; Color not written).

Execution details:
- Every kernel uses NumThreadsExpression='Vertex', numthreads(64,1,1), and a per-invocation thread offset (one invocation per render section).
- Typical compute data interfaces: SkinnedMeshRead, TransientBuffer x4, CustomComputeKernel x3, GraphDataInterface (constants and variables in a 112-byte parameter buffer), SkinWeightsAsVertexMask, DebugDraw, SkinnedMesh, HalfEdge, SkinnedMeshWrite.
- The graphs contain no morph-target, skeleton or skin-cache nodes. They rely on the default deformer, run earlier in the stack, for LBS, morphs and cloth.

**Setup.** None.

**Operators:** `OptimusSkinnedMeshReadDataInterface`, `OptimusSkinnedMeshWriteDataInterface`, `OptimusTransientBufferDataInterface`, `OptimusCustomComputeKernelDataInterface`, `OptimusGraphDataInterface`, `OptimusComputeGraph`

**Scale:** 17 graphs.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraHeadBend_DeformerGraph/asset.t3d: 'DataInterfaces(0)..DataInterfaces(13)', 'NumThreadsExpression="Vertex"' (lines 944-952), 'ComponentType=...OptimusSkeletalMeshComponentSource'`; `<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraHeadBend_DeformerGraph/kernels.hlsl:172-180 ([numthreads(64,1,1)], ReadThreadIndexOffset)`; `<UE>/Plugins/Animation/DeformerGraph/Source/OptimusCore/Private/DataInterfaces/OptimusDataInterfaceSkinnedMeshRead.h:23`

#### UE7-dg-skinweight-vertex-mask — Skin-weights-as-vertex-mask (bone-list weight map)

*core* · assets: all 17 deformer graphs

OptimusSkinWeightsAsVertexMaskDataInterface builds a per-vertex float mask.

Bone selection:
- Start from BoneNames and walk up the parent chain while |distance| <= ExpandTowardsRoot.
- Flood to descendants: a child is added if its parent's distance is >= 0 and < ExpandTowardsLeaf, or if the parent's distance is negative and |distance| < ExpandTowardsRoot.
- Build a per-render-section BoneIsSelected table.

Shader: Mask(v) = sum over the vertex's influences of weight_i * isSelected(bone_i). The result is in [0,1] and supports unlimited-influence buffers.

Optional SkinWeightProfile support exists. In all graphs only BoneNames and bDebugDrawIncludedBones=False are authored; SkinWeightProfile/ExpandTowardsRoot/ExpandTowardsLeaf are neither set nor pin-linked. The effective values are therefore ExpandTowardsRoot=0 and ExpandTowardsLeaf=999 (the header default), so each listed bone includes ALL its descendants. On Zebra, [head] covers head plus 277 descendants (skull, eyes, lids, jaw, tongue, teeth, ears, mohawk); on Monster it covers head plus 151.

The mask feeds the kernel's WeightMap, and the kernel computes lerp(P, P', mask). The kernel comment notes that a painted vertex-attribute map could be used instead.

**Setup.** Mask bone lists (full list per graph in the asset summaries): head = [head]; Zebra muzzle = 25 lip/teeth/nose/jaw/muzzle bones; Zebra skull top = [skull_tp, eye_main_l/r, ear_base_l/r]; Monster muzzle/mouth lists add cheek_l/r; Monster skull top = [skull_tp, eye_main_l/r].

**Operators:** `OptimusSkinWeightsAsVertexMaskDataInterface ('Skin Weights as Vertex Mask' node)`, `ReadMask_* shader (DataInterfaceSkinWeightsAsVertexMask.ush)`

**Scale:** 17 graphs, one mask DI each.

**Evidence:** `<UE>/Plugins/Animation/DeformerGraph/Shaders/Private/DataInterfaceSkinWeightsAsVertexMask.ush: ReadMask_ sums BoneIsSelected ? BoneWeight : 0`; `<UE>/Plugins/Animation/DeformerGraph/Source/OptimusCore/Private/DataInterfaces/OptimusDataInterfaceSkinWeightsAsVertexMask.h:53-66 (BoneNames default {Root}, ExpandTowardsRoot=0, ExpandTowardsLeaf=999)`; `OptimusDataInterfaceSkinWeightsAsVertexMask.cpp:300-363 (distance flood)`; `<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraHeadBend_DeformerGraph/asset.t3d:462 'BoneNames(0)="head"' (only BoneNames and bDebugDrawIncludedBones present)`; `<dump>/Game__Assets__Zebra__Meshes__SKM_Zebra/bones.txt (head has 277 descendants; computed)`

#### UE7-dg-squash-stretch — Squash/stretch with sine bulge (DG_Function_SquashStretch)

*core* · assets: ZebraHead_DeformerGraph, ZebraMuzzleSquash_DeformerGraph, ZebraSkullTpSquash_DeformerGraph, Monster_Head_DeforerGraph, Monster_MuzzleSquash_DeforerGraph, Monster_MouthSquash_DeforerGraph, Monster_SkullTpSquash_DeforerGraph

Inputs: OriginTransform M, LengthToDeform L, StretchFactor F in [0, inf), XYBias, ZBias, WeightMap w, limits lb and lt.

Setup:
- s = max(2F, 1e-4). F=0.5 gives the identity.
- p = inverse_affine(M)*P.
- Lower = sanitize(lb)*L and Upper = sanitize(1-lt)*L, where sanitize clamps to [1e-4, 1-1e-4].
- zc = clamp(p.z, Lower, Upper) and t = (zc-Lower)/(Upper-Lower).

ZBias remaps t with a quadratic Bezier from (0,0) to (1,1) with middle control point (ZBias, 1-ZBias):
- If ZBias != 0.5: u = (ZBias - sqrt(t - 2*t*ZBias + ZBias^2)) / (2*ZBias - 1) and t' = 2(1-u)u(1-ZBias) + u^2.
- Otherwise t' = t.

XY bulge:
- b = sqrt(1/s), the base volume-preserving cross-section scale for x and y.
- XYBias is sanitized. wl = 1 - |XYBias-0.5|/0.5.
- a = lerp(1/b, 1, wl) and B = 1/a.
- xm = XYBias<0.5 ? B : a and ym = XYBias<0.5 ? a : B.
- bx = b*xm and by = b*ym.
- Bulge = (pi/2)*sin(pi*t'). It peaks at pi/2 in the middle and averages 1 over [0,1].
- x' = x*((bx-1)*Bulge + 1) and y' = y*((by-1)*Bulge + 1).

Z stretch:
- z' = max(0, p.z-Upper) + (zc-Lower)*s + min(Lower, p.z). The region is scaled by s, the mesh above Upper is translated by (Upper-Lower)(s-1), and the mesh below Lower is unchanged.
- XY is untouched outside the region because sin(0) = sin(pi) = 0.

Finally P' = M*p' and output = lerp(P, P', w).

Volume preservation is approximate: the cross-section scales by b on average, not at every point. With XYBias=0.5 the bulge is isotropic; all rig graphs use XYBias=ZBias=0.5.

**Setup.** StretchFactor = remap(curve, [-1,1] -> [0,1]); for muzzle and mouth the remap is inverted. The curve comes from the squash control's local translate Z. Constants: L=55 for Zebra head/muzzle/skull and Monster muzzle/mouth; L=75 for Monster head; L=40 for Monster skull top. XYBias=ZBias=0.5 everywhere.

**Operators:** `DG_Function_SquashStretch (kernel SquashStretch_OptimusNode_CustomComputeKernel_2)`, `DSL_Matrix`

**Scale:** 7 graphs, all used (3 Zebra, 4 Monster).

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraHead_DeformerGraph/kernels.hlsl:24-173`; `kernels.hlsl:31 QuadraticBezierInterpolation`; `kernels.hlsl:60 'float StretchRatioZ = max(Factor * 2, KINDA_SMALL_NUMBER);'`; `kernels.hlsl:80 'float2 BulgeRatio = sqrt(1/(StretchRatioZ));'`; `kernels.hlsl:166 lerp by Weight`

#### UE-deformer-factor-conventions — Deformer variable conventions and skin-weight masks for the head deformers

*important* · assets: ZebraHead_DeformerGraph, ZebraHeadBend_DeformerGraph, ZebraHeadTwist_DeformerGraph, ZebraMuzzle*_DeformerGraph, ZebraSkullTp*_DeformerGraph

Each graph exposes 2 variables: Transform, plus one of StretchFactor, BendFactor or TwistFactor.
- Squash/stretch: StretchRatioZ = max(2*Factor, eps), so 0.5 is neutral and the range is [0, inf). The mesh is scaled along local Z within the [LimitFromBottom, 1-LimitFromTop]*LengthToDeform band, with volume-preserving XY bulge sqrt(1/ratio) shaped by ZBias/XYBias.
- Bend and twist: Radian = lerp(-MaxAngle, +MaxAngle, (Factor+1)/2), so Factor lies in [-1, 1] and 0 is neutral. The mesh is bent or twisted about local Z over LengthToBend or LengthToTwist (Barr 1984).
- Blending: results are blended by a skin-weights-as-vertex-mask weight. The mask bones are: head for Head, HeadBend and HeadTwist; muzzle + lip_bt/lip_tp chains (18+ bones) for Muzzle*; skull_tp, eye_main_l/r and ear_base_l/r for SkullTp*.
- Normals and tangents are recomputed afterwards.

**Setup.** None. Default variable values: StretchFactor 0.5, Bend/Twist 0 (Zebra), Transform = pivot pose.

**Operators:** `OptimusSkinWeightsAsVertexMaskDataInterface`, `DG_Function_SquashStretch`, `DG_Function_Bend`, `DG_Function_Twist`

**Scale:** 8 Zebra deformer assets (7 used)

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraHead_DeformerGraph/kernels.hlsl:55-105 (StretchRatioZ = max(Factor * 2...))`; `<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraHeadBend_DeformerGraph/kernels.hlsl:53-60 (BendFactor range [-1, 1])`; `<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraHeadTwist_DeformerGraph/kernels.hlsl:50-56`; `<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraMuzzleBend_DeformerGraph/asset.t3d BoneNames(0)="muzzle" BoneNames(1)="lip_bt"`; `<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraSkullTpBend_DeformerGraph/asset.t3d BoneNames(0)="skull_tp"`

#### UE6-monster-optimus-deformers — Monster face layers 9 Optimus deformers positioned by nulls

*important* · assets: CRM_Monster_Face

Construction pin E spawns 9 nulls under bone head (global): Head Squash Null T(0,0,20); Head Bend Null T(0,0,25) rotZ 90; Head Twist Null T(0,0,25); Mouth Squash Null T(0,30,60) rot 180 about Y; Mouth Bend Null T(0,20,60) quat(-.707,-.707,0,0); Skull Tp Squash Null T(0,5,45) rotZ 90; Skull Tp Bend Null T(0,0,40) rotZ 90; Muzzle Squash Null T(0,30,60); Muzzle Bend Null T(0,20,60). Forward pin L, in order, each with Settings ExecutionPhase=AfterDefaultDeformer, ExecutionGroup=1, DeformChildComponents=True and Transform = the null's global transform: AddOptimusDeformer Monster_Head_DeforerGraph StretchFactor = remap(head_squash_deformer, -1..1 -> 0..1); Monster_HeadBend BendFactor = head_bend_deformer; Monster_HeadTwist TwistFactor = head_twist_deformer; Monster_MuzzleBend BendFactor = muzzle_bend_deformer; Monster_MuzzleSquash StretchFactor = remap(muzzle_squash_deformer, 1..-1 -> 0..1); Monster_MouthBend BendFactor = mouth_bend_deformer; Monster_MouthSquash StretchFactor = remap(mouth_squash_deformer, 1..-1 -> 0..1); Monster_SkullTpBend BendFactor = skull_tp_bend_deformer; Monster_SkullTpSquash StretchFactor = remap(skull_tp_squash_deformer, -1..1 -> 0..1). The graphs use engine functions DG_Function_SquashStretch, DG_Function_Bend and DG_Function_Twist (Barr global deformations), and their exposed variables are StretchFactor/BendFactor/TwistFactor plus Transform. CR_Monster_Deform does not add these deformers, so they exist only while the face module runs.

**Setup.** Driven by the Head/Skull Tp/Muzzle/Mouth Squash controls through the *_deformer curves.

**Operators:** `RigUnit_AddOptimusDeformer`, `RigUnit_HierarchyAddNull`, `RigUnit_GetCurveValue`, `RigVMFunction_MathDoubleRemap`

**Scale:** 9 deformers, 9 nulls

**Evidence:** `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/graphs.txt:1265-1295 AddOptimusDeformer_* (ExecutionPhase=AfterDefaultDeformer,ExecutionGroup=1)`; `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/graphs.txt:254 HierarchyAddNull_11 Head Squash Null`; `<dump>/Game__Assets__Monster__Deformers__Monster_Head_DeforerGraph/kernels.hlsl DG_Function_SquashStretch`; `<dump>/units_used.json per_rig CRM_Monster_Face RigUnit_AddOptimusDeformer 9`

#### UE6-squash-controls-curves — Squash/bend/twist deformer controls and their curve remaps

*important* · assets: CRM_Zebra_Face, CRM_Monster_Face

Controls: 'Head Squash' (parent Head Attach Null, offset T(0,0,150) global, Monster 100); 'Skull Tp Squash' (child of Head Squash, same offset); 'Muzzle Squash' (parent bone muzzle, T(0,20,89), Monster T(0,28,20)); Monster adds 'Mouth Squash' (parent bone muzzle, T(0,28,23)). Pin K reads local values: head_squash = remap(HeadSquash.tz, -100..100 -> -1..1); head_twist = remap(Euler(ZYX).Z of HeadSquash rotation, -135..135 -> 1..-1); head_bend = remap(HeadSquash.tx, -100..100 -> 1..-1); muzzle_squash_deformer = remap(MuzzleSquash.tz, -30..30 -> -1..1); muzzle_bend_deformer = remap(MuzzleSquash.tx, 20..-20 -> 1..-1); skull_tp_squash_deformer = remap(SkullTpSquash.tz, -100..100 -> -1..1); skull_tp_bend_deformer = remap(SkullTpSquash.tx, 50..-50 -> -1..1). None of the remaps clamp. Monster pin L also writes head_squash_deformer, head_twist_deformer and head_bend_deformer (same math), plus mouth_squash_deformer = remap(MouthSquash.tz, -30..30 -> -1..1) and mouth_bend_deformer = remap(MouthSquash.tx, 20..-20 -> 1..-1). An unexecuted node would write muzzle_squash_deformer from the head squash value.

**Setup.** Head Squash: Circle_Thick, cyan, scale 3. Skull Tp Squash: Circle_Thick, cyan, scale 1.5. Muzzle Squash: Default, cyan, scale 0.2. Mouth Squash: Default, cyan, scale 0.1. No limits. Translate Z to squash, translate X to bend, rotate (yaw) to twist.

**Operators:** `RigUnit_GetTransform`, `RigVMFunction_MathDoubleRemap`, `RigVMFunction_MathQuaternionToEuler`, `RigUnit_SetCurveValue`

**Scale:** 3-4 controls, 7 (Zebra) / 16 (Monster) curve writes

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:1104 SetCurveValue Curve=head_squash; :1105 Remap_1; :1118 Remap_5; :1123 Remap_8`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:184 SpawnControl_6 Head Squash; :189 SpawnControl_8; :191 SpawnControl_9`; `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/graphs.txt:1238 SetCurveValue_39 head_squash_deformer; :270 SpawnControl_11 Mouth Squash`

#### UE7-dg-monster-deformer-params — Monster deformer set (9 graphs) parameter table

*important* · assets: Monster_*_DeforerGraph / DeformerGraph (9)

| Graph | Function | Length | MaxAngle | Mask |
| Head | Squash | 75 | - | head |
| HeadBend | Bend | 90 | 180 | head |
| HeadTwist | Twist | 175 | 135 | head |
| MuzzleBend | Bend | 90 | 180 | 27: lips, nose, teeth, corners, cheeks, muzzle, jaw |
| MuzzleSquash | Squash | 55 | - | 26: MuzzleBend list minus muzzle |
| MouthBend | Bend | 90 | 180 | 26: MuzzleBend list minus jaw |
| MouthSquash | Squash | 55 | - | 25: no muzzle, no jaw |
| SkullTpBend | Bend | 90 | 180 | skull_tp, eye_main_l, eye_main_r |
| SkullTpSquash | Squash | 40 | - | skull_tp, eye_main_l, eye_main_r |

All squash graphs use XYBias=ZBias=0.5. Every graph uses a local CacheGeometry subgraph and preview mesh SKM_Monster, and all are Status=Compiled.

**Setup.** See UE7-deformer-control-channel-mapping.

**Operators:** `DG_Function_Bend`, `DG_Function_Twist`, `DG_Function_SquashStretch`

**Scale:** 9 graphs, all used.

**Evidence:** `(analysis helper table, not kept) (from <dump>/Game__Assets__Monster__Deformers__*/asset.t3d)`; `<dump>/Game__Assets__Monster__Deformers__Monster_HeadTwist_DeformerGraph/asset.t3d (LengthToTwist const 175)`; `<dump>/Game__Assets__Monster__Deformers__Monster_Head_DeforerGraph/asset.t3d:1423 'Status=Compiled'`

#### UE7-dg-recompute-normals-keep-input — Recompute normals/tangents while keeping authored normals (half-edge 1-ring)

*important* · assets: all 17 deformer graphs

Runs after the positional deform (DG_Function_ComputeNormalsTangentsAndKeepInputNormals).

Per vertex:
1. Walk the one-ring with half-edges. Start at Edge(v); for each triangle t = e/3 take its other two vertices. Stop when TwinEdge = -1 or the walk returns to the start, capped at 32 iterations. The next edge is twin_tri*3 + (twin_sub+1)%3.
2. Sum the normalized face normals cross(e2, e1) for the deformed positions (SumD) and for the original, pre-deform positions (SumO).
3. Sum sign(UV cross product) over triangles where v is corner 0, ignoring |cp| < 1e-6.
4. Let nD = normalize(SumD), nO = normalize(SumO), and n0 = the authored input TangentZ.
5. q1 = QuatBetween(nO, n0). DeformedTangentZ = q1 * nD, which carries the authored-normal offset onto the deformed geometric normal.
6. q2 = QuatBetween(n0, DeformedTangentZ). DeformedTangentX = q2 * input TangentX.
7. Write Position (passthrough), TangentX (w=0) and TangentZ (w = SumOrientation <= 0 ? -1 : 1).

QuatBetweenVectors handles antiparallel vectors by rotating pi about a perpendicular axis.

Requirements: the half-edge DI (the mesh needs BuildHalfEdgeBuffers), plus the index buffer and UV0 from OptimusSkinnedMeshDataInterface. 'Original' = the input to this deformer, i.e. the output of the previous deformer in the stack.

**Setup.** None; the pass is automatic in every graph.

**Operators:** `DG_Function_ComputeNormalsTangentsAndKeepInputNormals`, `OptimusHalfEdgeDataInterface (ReadEdge, ReadTwinEdge)`, `OptimusSkinnedMeshDataInterface (ReadIndexBuffer, ReadUV)`, `DSL_Quaternion`

**Scale:** 17 graphs (15 share an identical kernel hash; Monster_Head differs only by whitespace).

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraHeadBend_DeformerGraph/kernels.hlsl:183-327`; `kernels.hlsl:241 'int MaxIterationIndex = 32;'`; `kernels.hlsl:308 'QuatBetweenVectors(FinalComputedOriginalTangentZ, OriginalTangentZ)'`; `kernels.hlsl:316 'SumOrientation <= 0 ? -1 : 1'`; `<UE>/Plugins/Animation/DeformerGraph/Source/OptimusCore/Private/DataInterfaces/OptimusDataInterfaceHalfEdge.h:26-27 (BuildHalfEdgeBuffers note)`

#### UE7-dg-variables-and-constants — Deformer variables vs baked constants

*important* · assets: all 17 deformer graphs

Each graph exposes exactly two variables: 'Transform' (FTransform) and one factor ('BendFactor', 'TwistFactor' or 'StretchFactor', DoubleProperty). Each variable is routed through a ConstantValue node whose Value pin is linked from the GetVariable node, so the constant's stored value is ignored at runtime.

Everything else is a baked constant: LengthTo* (float), MaxBendAngle/MaxTwistAngle (IntProperty), XYBias/ZBias (0.5), LimitFromBottom/Top (0), EnableDebugDraw (0). The values are compiled into the ValueMap (for example, FloatProperty 90.0 stored as bytes (0,0,180,66)).

Variable defaults:
- Bend: 0.5 or 0.
- Twist: 0.
- Stretch: 0.5 (identity).
Note: BendFactor 0.5 is NOT the identity (0 is), so a missing variable binding would bend the mesh by 90 degrees.

**Setup.** Only Transform and the factor are animatable, through AddOptimusDeformer variable traits.

**Operators:** `OptimusVariableDescription`, `OptimusNode_GetVariable`, `OptimusNode_ConstantValue_*`

**Scale:** 34 variables across 17 graphs.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraHeadBend_DeformerGraph/asset.t3d: 'CustomProperties VariableDefinition Name="BendFactor" Type=DoubleProperty DefaultValue="0.500000"'`; `same file: 'ValueMap=(((Type=Constant,Name="UpdateGraph_OptimusNode_ConstantValue_FloatProperty")...ShaderValue=(0,0,180,66)'`; `(analysis helper table, not kept) (per-graph pin->source table)`

#### UE7-dg-zebra-deformer-params — Zebra deformer set (8 graphs) parameter table

*important* · assets: Zebra*_DeformerGraph (8)

Kernel math is shared (see UE7-dg-barr-bend, UE7-dg-barr-twist, UE7-dg-squash-stretch); only parameters, masks and usage differ.

| Graph | Function | Length | MaxAngle / biases | Mask | Used |
| ZebraHead | Squash | 55 | XYBias/ZBias 0.5 | head | yes |
| ZebraHeadTwist | Twist | 60 | 135 | head | yes |
| ZebraHeadBend | Bend | 90 | 180 | head | yes |
| ZebraMuzzleSquash | Squash | 55 | 0.5 / 0.5 | 25-bone muzzle | yes |
| ZebraMuzzleBend | Bend | 90 | 180 | 25-bone muzzle | yes |
| ZebraMuzzleTwist | Twist | 60 | 135 | 25-bone muzzle | no |
| ZebraSkullTpSquash | Squash | 55 | 0.5 / 0.5 | 5-bone skull | yes |
| ZebraSkullTpBend | Bend | 90 | 180 | 5-bone skull | yes |

All limits are 0 and debug draw is off.

**Setup.** See UE7-deformer-control-channel-mapping.

**Operators:** `DG_Function_Bend`, `DG_Function_Twist`, `DG_Function_SquashStretch`

**Scale:** 8 graphs; 7 used.

**Evidence:** `(analysis helper table, not kept) (generated from <dump>/Game__Assets__Zebra__Rig__Deformers__*/asset.t3d)`; `<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraMuzzleBend_DeformerGraph/asset.t3d: BoneNames(0)="muzzle" ... BoneNames(24)`; `<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraSkullTpBend_DeformerGraph/asset.t3d: BoneNames(0)="skull_tp"..."ear_base_r"`

#### UE7-dg-zebra-vs-monster — Zebra vs Monster deformer differences

*important* · assets: Zebra*_DeformerGraph, Monster_*Graph, CR_Zebra_Deform, CRM_Monster_Face

1. Integration: Zebra runs its deformers from the post-process rig CR_Zebra_Deform, driven by curves, so they also work at runtime and with baked animation. Monster runs them from the Face module inside MR_Monster, so they only run while the control rig evaluates.
2. CacheGeometry: Monster uses a local SubGraph. Zebra references a function in Monster_Head_DeforerGraph whose GUID (1ED32A22) no longer exists; the node shows '<graph missing>'. The compiled Zebra graphs still contain the pass-through kernel, except ZebraHead, which is Modified and compiled without it. The result is a cross-character asset dependency from Zebra to Monster.
3. Set: Monster adds MouthBend and MouthSquash plus a Mouth Squash control. Zebra has MuzzleTwist, which is unused.
4. Order: Zebra runs twist before bend and squash before bend; Monster runs bend before twist and bend before squash (see UE7-deformer-stack-ordering).
5. Lengths: head squash 55 vs 75; head twist 60 vs 175; skull-top squash 55 vs 40.
6. Masks: Monster muzzle and mouth masks add cheek_l/r. The Monster skull-top mask omits the ear bones. The Zebra head mask covers 277 descendants, the Monster one 151.
7. Curve names: Zebra uses head_squash/head_bend/head_twist; Monster uses head_*_deformer.

**Setup.** n/a

**Operators:** `OptimusNode_FunctionReference`, `OptimusNode_SubGraphReference`

**Scale:** 17 graphs.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraHeadBend_DeformerGraph/asset.t3d: 'FunctionGraphIdentifier=(Asset=...Monster_Head_DeforerGraph...,Guid=1ED32A224E388296B87554A8F4657949)' and 'DisplayName=..."<graph missing>"'`; `<dump>/Game__Assets__Monster__Deformers__Monster_Head_DeforerGraph/asset.t3d:1360 'Guid=EE5E6AF54B624361866B9E8B3AD39597' (NewFunction)`; `<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraHead_DeformerGraph/asset.t3d (no 'Status=' line; KernelToNode has only SquashStretch and ComputeNormals)`

#### UE8-skinning-cvars — Skinning and skin-cache project settings, and the unknown influence count

*important* · assets: DefaultEngine.ini, SKM_Zebra, SKM_Zebra_Hi, SKM_Monster

DefaultEngine.ini lines 10-14 and 21 set the following. r.GPUSkin.Support16BitBoneIndex=True. r.GPUSkin.UnlimitedBoneInfluences=True: read-only; meshes whose max influences exceed r.GPUSkin.UnlimitedBoneInfluencesThreshold (default EXTRA_BONE_INFLUENCES = 8) use the unlimited-influence buffer instead of the fixed 4/8 layout. r.SkinCache.CompileShaders=True. r.SkinCache.DefaultBehavior=0 (Exclusive): meshes are excluded from the skin cache unless they opt in, but a mesh with Support Ray Tracing is forced in, and the project sets r.RayTracing=True. SkeletalMesh.UseExperimentalChunking=1. The mesh packages store LOD SkinCacheUsage = ESkinCacheUsage::Auto, and 'MaxBoneInfluences'/'BoneInfluenceLimit' properties. SKM_Zebra_Hi was imported through Interchange with boneInfluenceLimit 0 (use the project default, i.e. no limit), bRecomputeTangents, bAddCurveMetadataToSkeleton and material instances. The actual per-vertex influence count, which sets the UsdSkel jointIndices/jointWeights elementSize, is in none of the dumps, and monster_usd_tree.txt lists only prim types. The Optimus deformers run after skinning regardless of skin-cache opt-in.

**Setup.** none

**Operators:** `FGPUSkinCache`, `GPUSkinVertexFactory (unlimited bone influences)`

**Scale:** project-wide; 3 character meshes

**Evidence:** `<ZebraSample>/Config/DefaultEngine.ini:10-14, 21`; `<UE>/Source/Runtime/Engine/Private/GPUSkinCache.cpp:118-125`; `<UE>/Source/Runtime/Engine/Private/GPUSkinVertexFactory.cpp:52-63`; `<ZebraSample>/Content/Assets/Zebra/Meshes/SKM_Zebra_Hi.uasset name table: '"boneInfluenceLimit": 0,', '"bAddCurveMetadataToSkeleton": true,', 'ESkinCacheUsage::Auto'`; `<ZebraSample>/Content/Assets/Zebra/Meshes/SKM_Zebra.uasset name table: 'MaxBoneInfluences','BoneInfluenceLimit','ESkinCacheUsage::Auto'`; `<dump>/Game__Assets__Monster__Meshes__SKM_Monster/monster_usd_tree.txt (no elementSize info)`

#### UE7-dg-cache-geometry-passthrough — CacheGeometry pass-through (snapshot of input geometry)

*nice-to-have* · assets: Monster_* (local SubGraph 'CacheGeometry'), Zebra*_DeformerGraph (FunctionReference to Monster_Head_DeforerGraph)

A Vertex-domain custom kernel ('MyKernel') that copies ReadPosition, ReadTangentX and ReadTangentZ to OutPosition, OutTangentX and OutTangentZ in 3 transient buffers. Those buffers are the 'OriginalPosition', 'OriginalTangentX' and 'OriginalTangentZ' inputs of the normals function, while the raw Read Skinned Mesh position goes into the deform function. The copy is mathematically the identity, so Original = input geometry. The two Monster exceptions noted in the asset summaries (Monster_MouthBend and the Modified ZebraHead) show that the copy can be skipped.

**Setup.** None.

**Operators:** `OptimusNode_CustomComputeKernel`, `OptimusNode_SubGraphReference (SubGraphName=CacheGeometry)`, `OptimusNode_FunctionReference`, `OptimusTransientBufferDataInterface`

**Scale:** 16 of 17 compiled graphs contain the pass; ZebraHead does not.

**Evidence:** `<dump>/Game__Assets__Monster__Deformers__Monster_Head_DeforerGraph/kernels.hlsl:1-21 (ShaderText KERNEL copies Position/TangentX/TangentZ)`; `<dump>/Game__Assets__Monster__Deformers__Monster_Head_DeforerGraph/asset.t3d:917 'SubGraphName="CacheGeometry"'`; `<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraHeadBend_DeformerGraph/asset.t3d:869 'KernelToNode(0)=...Monster_Head_DeforerGraph:CacheGeometry...'`; `<dump>/Game__Assets__Monster__Deformers__Monster_MouthBend_DeforerGraph/kernels.hlsl:26 'void WriteOutTangentZ(uint VertexIndex, float4 Value) { }'`

#### UE8-deform-unresolved-inverse-squetch — CR_Zebra_Deform: unresolved 'Inverse_1' template node and the 'Squetch' curve

*nice-to-have* · assets: CR_Zebra_Deform

regen.py adds 'Inverse::Execute(in Value,out Result)' as template node Inverse_1 without resolving its wildcard type. The dump shows it as '?::None' (the single '?::None' entry in units_used.json). It has no links, so it compiles to nothing and can be ignored. The neighbouring nodes compute Squetch = ChainInfo.ChainStretchFactor - 1 (Subtract with A linked from ChainInfo, B=1; ChainInfo bCalculateStretch=True) and write it with SetCurveValue 'Squetch', which executes between SetCurveValue_34 and SphericalPoseReader_1_1_1_1_18.

**Setup.** none

**Operators:** `RigVMFunction_MathDoubleSub`, `RigUnit_ChainInfo`, `RigUnit_SetCurveValue`, `template 'Inverse' (unresolved)`

**Scale:** 1 dead node

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:175-179, 536-538, 561`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/regen.py :: "add_template_node('Inverse::Execute(in Value,out Result)', ... 'Inverse_1')"`; `<dump>/units_used.json:180, 369 ('?::None': 1)`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/hierarchy.txt :: 'CURVE Squetch'`

### D14 Face logic

#### UE6-blink-extend-open-rotate — Blink Extend, Lid Open and Lid Rotate logic

*core* · assets: CRM_Zebra_Face, CRM_Monster_Face

These functions share Blink Logic's structure with different weights. Blink Extend: w = remap(tz, 100..200 -> 0..1, clamp); it applies 'Lid Tp/Bt Extend Blink Rotations' to the main (non-base) Tp/Bt bones only, 4 calls. Zebra Tp extend is about [-22,-25,-5] degrees and Bt about [-30,-28,-21]. Lid Open: w = remap(tz, 0..-200 -> 0..1, clamp); it applies 'Lid Tp/Bt (Base) Open Rotations' to both main and base bones, 8 calls, Tp about [-30,-28,~-21 mixed] and Bt about [+20,+20,+20]. Lid Rotate: roll = GetControlRotator(Control, Local).Roll; wPos = remap(roll, 0..90 -> 0..1) applies 'Lid Tp/Bt Rot Pos' and wNeg = remap(roll, 0..-90 -> 0..1) applies 'Rot Neg', on the main Tp/Bt bones, 4 calls. The Rot tables mix about 20-degree Y/Z rotations with a ±35-degree X rotation on the middle bone. All are additive local on the same spawned bones after Blink, so the effects stack.

**Setup.** The upper half of the Lid slider range (100..200) is the extended squeeze; negative travel opens the lid wide; Roll tilts the lid.

**Operators:** `RigUnit_GetTransform`, `RigUnit_GetControlRotator`, `RigVMFunction_MathDoubleRemap`, `RigUnit_ModifyTransforms`

**Scale:** 16 calls

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:2328 Blink Extend Remap_28 SourceMinimum=100`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:2358 Lid Open Remap_28 SourceMaximum=-200`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:2387-2413 Lid Rotate GetControlRotator.Rotator.Roll, Remap 0..90 / 0..-90`

#### UE6-blink-logic — Blink Logic (slider-weighted additive rotations on spawned lid bones)

*core* · assets: CRM_Zebra_Face, CRM_Monster_Face

Blink Logic(Rotations[] quat, Lid Joints[], Control) computes w = remap(GetTransform(Control, Local).tz, 0..200 -> 0..1, clamp). For each joint i it applies ModifyTransforms(joint i, AdditiveLocal, Transform = rotation Rotations[i], weight w), which lerps identity to R by w and pre-multiplies the local transform. It returns w. Eight calls on pin H: Lid Tp L with 'Lid Tp Blink Rotations' on the Tp bones and 'Lid Tp Base Blink Rotations' on the Tp Base bones; Lid Bt L with 'Lid Bt Blink Rotations' and 'Lid Bt Base Blink Rotations'; the same for R, reusing the L quaternion tables (mirroring comes from the eye_main_r frame). Zebra values: Tp blink about [60,70,41] degrees mostly about Y; Tp base [5,10,5]; Bt [-15,-25,-20]; Bt base [-15,-20,-20]. Monster values: Tp [60,60,36]; Bt [-30,-35,-30].

**Setup.** Lid Tp/Bt sliders along +Z of the offset frame (downward for Tp because of the 180-degree offset).

**Operators:** `RigUnit_GetTransform`, `RigVMFunction_MathDoubleRemap`, `RigUnit_ModifyTransforms`, `RigVMDispatch_ArrayIterator`, `RigVMDispatch_ArrayGetAtIndex`

**Scale:** 8 calls x 3 bones

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:2289-2317 Blink Logic Remap_28 0..200 -> 0..1`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:905 Blink Logic Control=Lid Tp R`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/asset.t3d:1900 Lid_Tp_Blink_Rotations=...`

#### UE6-brow-main — Brow Main L/R diagonal pads to brow in/out up/down curves

*core* · assets: CRM_Zebra_Face, CRM_Monster_Face

Control 'Brow Main L' is parented to bone skull_tp, with offset rotation Y=-45 degrees (quat Y -0.3827), T(10,20,120) (Monster T(10,30,70)) and scale (0.05,-0.05,0.05) (Monster 0.075). TranslationY is locked at 0, so the control moves in its XZ plane, which is diagonal on screen. Brow Main R is mirrored (Y=+45 degrees, X negative, scale X negative). Brow Main(Item) reads v = local translation: brow_in_dn = clamp(-0.01*v.z, 0, 200); brow_in_up = clamp(0.01*v.z, 0, 200); brow_ot_dn = clamp(-0.01*v.x, 0, 200); brow_ot_up = clamp(0.01*v.x, 0, 200). Pin A writes brow_in_dn/up and brow_ot_dn/up for R and then L. Moving straight up raises both inner and outer brows.

**Setup.** Sphere_Solid, blue (L) or red (R), shape scale 3, bDrawLimits=false. Hosts the bool channel 'Brow Tweaker Vis'.

**Operators:** `RigUnit_GetTransform`, `RigVMFunction_MathDoubleMul`, `RigVMFunction_MathDoubleRemap`, `RigUnit_SetCurveValue`

**Scale:** 2 controls, 8 curves

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:2162-2188 Brow Main`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:48 SpawnControl_42 Brow Main L Rotation Y=-0.382683`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1329 Face/Brow Main L LIM[1]`

#### UE6-eye-controls — Eye Main and Eye rotation controls

*core* · assets: CRM_Zebra_Face, CRM_Monster_Face

'Eye Main L' is parented to skull_tp with offset = eye_l global transform, and drives bone eye_main_l (the eyeball socket, parent of eye_l and all lid bones). It hosts the bool channel 'Micro Vis'. 'Eye L null' is parented to skull at the eye_l global translation (no rotation). 'Eye L' is parented to that null, with offset translation = eye_l translation. Its translation limits are all 0 and its scale limits are all 1, so it is rotation only. It drives bone eye_l. It hosts float channels 'Pupil Size' and 'Irisl Size' (-100..100). SetChannelHosts adds Eye R as an extra host of both channels, so the two eyes share them. The eye null is aimed each frame (UE6-eye-aim).

**Setup.** Eye L/R: Circle_Thick, blue or red, shape (1,15,0) rotated -90 degrees about X, scale 0.25. Eye Main L/R: Sphere_Solid, scale 0.15, shape (+-10,-+10,0).

**Operators:** `RigUnit_HierarchyAddNull`, `RigUnit_HierarchyAddControlTransform`, `RigUnit_HierarchyAddAnimationChannelFloat`, `RigUnit_SetChannelHosts`

**Scale:** 4 controls, 2 nulls, 2 shared channels

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:30 SpawnControl_3 Name=Eye L`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:197 SetChannelHosts Hosts=Eye R`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1369-1371 Face/Eye L LIM[0,1,2,6,7,8]; Pupil Size AvailableSpaces Eye R`

#### UE6-jaw-open-logic — Jaw Open Logic: jaw_open curve, Jaw Normalize and smile-masked lip corner pull

*core* · assets: CRM_Zebra_Face, CRM_Monster_Face

Input B (0..1). Set member Jaw Normalize = 4*B, set curve jaw_open = B. wL = B*(1 - smile_l) and wR = B*(1 - smile_r), unclamped; negative weights are skipped by ModifyTransforms. AdditiveLocal translations: lip_corner_l X-2, lip_bt_01_l X-2.5, lip_tp_01_l X-2.5, lip_tp_02_l X-1.5, cheek_l identity (no-op), all weighted by wL; lip_corner_r X+2, lip_bt_01_r X+2.5, lip_tp_01_r X+2.5, lip_tp_02_r X+1.5, cheek_r X+1, weighted by wR. Monster adds lip_tp_03_l X-1.5 (wL) and lip_bt_03_r X+1.5 (wR). The effect is that the mouth corners pull inward as the jaw opens unless smiling.

**Setup.** jaw control, via the reader.

**Operators:** `RigUnit_GetCurveValue`, `RigVMFunction_MathDoubleOneMinus`, `RigVMFunction_MathDoubleMul`, `RigUnit_ModifyTransforms`, `RigUnit_SetCurveValue`, `RigVMVariableNode setter`

**Scale:** Zebra 10 ModifyTransforms, Monster 12

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:2511-2565 Jaw Open Logic (Multiply_26 B=4, SetCurveValue Curve=jaw_open)`; `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/graphs.txt:2780 Jaw Open Logic (23 nodes)`

#### UE6-jaw-reversejaw-skulltp — Jaw, Reverse Jaw (skull) and Skull Tp controls

*core* · assets: CRM_Zebra_Face, CRM_Monster_Face

Three controls are parented to Head Attach Null: 'jaw' (offset = jaw bone global, drives bone jaw), 'Reverse Jaw' (offset = skull bone global, drives bone skull, which carries the upper lip, eyes and muzzle) and 'Skull Tp' (child of Reverse Jaw, offset = skull_tp global, drives skull_tp, the parent of brows and Eye Main). Construction spawns two helper bones: 'Jaw Const' (parent = jaw control, local identity) and 'Skull Const' (parent = Reverse Jaw control). The lip tweaker nulls use them as parents because they follow the control rather than the bone.

**Setup.** jaw: shape Default, yellow, shape translation (-12,20,0), scale 0.2 (Monster (-20,30,-0.5)). Reverse Jaw: orange (1,.5,0), shape (33,20,0) (Monster (45,25,0)). Skull Tp: orange, shape (23,21,0), scale 0.1. No limits. jaw also hosts the lip/mouth animation channels.

**Operators:** `RigUnit_HierarchyAddControlTransform`, `RigUnit_HierarchyAddBone`

**Scale:** 3 controls, 2 helper bones

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:6 SpawnControl Name=jaw Parent=Head Attach Null`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:114 HierarchyAddBone_30 Name=Skull Const`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:2194 BONE Face/Jaw Const par=C:Face/jaw`

#### UE6-lip-tweakers — Lip tweaker system (20 nulls + controls, struct-weighted follow, additive offset)

*core* · assets: CRM_Zebra_Face, CRM_Monster_Face, Lip_Null_Struct

Construction resets the 'Lip Control Nulls' and 'Lip Controls' arrays, then loops over the 20 lip bones [lip_tp, tp_01..04_l, bt_01..04_l, corner_l, lip_bt, tp_01..04_r, bt_01..04_r, corner_r]. For each: name = Rename Joint to Control; spawn null <name> under bone Jaw at the bone's global transform; spawn control <name> under that null at the same transform. The control shape X offset is -2 when index>10 and +2 otherwise. Both keys are appended to the arrays. Forward pin G: (1) for each null i, ParentConstraint(null i, parents [Skull Const, Jaw Const, Corner L, Corner R], weights = Lip Null Struct row i, maintain offset). Rows: lip_tp [1,0,0,0]; tp_01_l [.5,.1,.4,0]; tp_02_l [.7,0,.3,0]; tp_03_l [.9,0,.1,0]; tp_04_l [1,0,0,0]; bt_01_l [0,.6,.4,0]; bt_02_l [0,.7,.3,0]; bt_03_l [0,.9,.1,0]; bt_04_l [0,1,0,0]; corner_l [.2,.2,.6,0]; lip_bt [0,1,0,0]; the R rows are mirrored using the Corner R column. (2) For each lip bone i, ModifyTransforms(bone i, AdditiveLocal, Transform = GetTransform(Lip Controls[i], LocalSpace), weight 1), which adds the tweaker's value delta on top of the solved bone.

**Setup.** 20 controls named Lip Tp, Lip Tp 01-04 L/R, Lip Bt, Lip Bt 01-04 L/R, Lip Corner L/R. Sphere_Solid, magenta, scale 0.05, shape rotated -90 degrees about X, bDrawLimits=false. Visibility is controlled by jaw.Lip Tweaker Vis. Each control's parent space is its same-named null.

**Operators:** `RigVMFunction_ControlFlowBranch`, `RigVMFunction_MathIntGreater`, `RigUnit_HierarchyAddNull`, `RigUnit_HierarchyAddControlTransform`, `RigVMDispatch_ArrayAdd`, `RigVMDispatch_ArrayReset`, `RigUnit_ParentConstraint`, `RigUnit_ModifyTransforms`, `RigUnit_GetTransform`

**Scale:** 20 nulls, 20 controls, 20 constraints, 20 ModifyTransforms

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:74 SpawnControl_49, :80 Greater B=10, :97 HierarchyAddNull_29 Parent=Jaw, :246 ItemArray_11`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:1094-1097 For_Each_26 / ParentConstraint_22 / At_18 (Lip Null Struct)`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:766 ModifyTransforms_204`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/asset.t3d:1900 Lip_Null_Struct=...`

#### UE6-muzzle-mouth-lips-controls — Muzzle / Mouth / Lips Tp / Lips Bt macro controls

*core* · assets: CRM_Zebra_Face, CRM_Monster_Face

Muzzle: parent Head Attach Null, offset = muzzle bone global. Mouth: Zebra parent = Muzzle control, offset T(0,15,105). Monster spawns it as a new node with parent = jaw bone, offset T(0,15,41), magenta, scale 0.15. Lips Tp: Zebra parent = Mouth control, offset T(0,15,96); Monster parent = muzzle bone, offset T(0,15,40). Lips Bt: parent jaw bone, offset T(0,15,95) (Monster T(0,15,37)). These controls do not drive bones directly. They feed the New Parent operator (UE6-new-parent-follow), which moves groups of lip/mouth bones by each control's delta.

**Setup.** Shape Default. Muzzle and Mouth are yellow; Lips Tp/Bt are magenta (1,0,1), scale 0.1, with shape offsets (0,13,8) and (0,10,-1).

**Operators:** `RigUnit_HierarchyAddControlTransform`

**Scale:** 4 controls

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:107 SpawnControl_77 Name=Mouth`; `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/graphs.txt:272 SpawnControl_12 Name=Mouth Parent=Jaw`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:2197 CONTROL Face/Mouth par=C:Face/Muzzle`

#### UE1-layout-face-slot — Face module slot in the body rig

*important* · assets: MR_Zebra, MR_ZebraDMC, MR_Monster

The face is a single module referencing a ControlRigRuntimeAsset (CRM_Zebra_Face or CRM_Monster_Face). Its connectors are Root -> bone root (primary) and Parent (secondary, optional, ChildOfPrimary).
- Zebra: parent module root, Parent -> head. The face spawns 107 controls, 32 nulls and 31 bones, including a parentless 'Face/Head Attach Null' and constraint bones 'Face/Jaw Const' and 'Face/Skull Const'.
- Monster: parent module Spine, Parent unconnected. 120 face controls and 41 nulls.

Zebra's ear and mohawk modules are children of Face so they evaluate after it. The face internals belong to another analysis scope.

**Operators:** `CRM_Zebra_Face (runtime asset)`, `CRM_Monster_Face (runtime asset)`

**Scale:** Face controls: Zebra 107, Monster 120.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/modular_rig_model.txt: 'Face/Parent' -> 'head'`; `<dump>/Game__Assets__Monster__Rig__MR_Monster/modular_rig_model.txt: 'Name="Face",ParentModuleName="Spine"'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:2194 'BONE Face/Jaw Const'`

#### UE6-brow-micro-follow — Brow micro controls riding on curve-driven helper bones

*important* · assets: CRM_Zebra_Face, CRM_Monster_Face

Construction spawns helper bones 'Brow In L' at (5,20,118) and 'Brow Ot L' at (15,20,118) under skull_tp (Monster z=68, y=30), with the micro controls 'Brow In L' and 'Brow Ot L' parented to them. It also spawns a null 'Brow Mid L' at (10,20,118) with no parent (Monster: parent skull_tp) and control 'Brow Mid L' under that null. Brow Mid R's null is also placed at X=+10 (bug); the maintained offset hides it. In pin A: ModifyTransforms AdditiveLocal on bone Brow In L applies translation Z+4 weighted by the brow_in_up_l curve and Z-5 weighted by brow_in_dn_l; bone Brow Ot L gets the same with brow_ot_up_l / brow_ot_dn_l. The weight is clamped to [0,1] by ModifyTransforms. Then PositionConstraintLocalSpaceOffset puts null Brow Mid L at the weighted average (1,1) of bones Brow In L and Brow Ot L, maintain offset, XYZ. Two further position constraints target nulls 'Brow In L'/'Brow Ot L' that do not exist, so they are no-ops. The R side is mirrored. The micro controls therefore follow the main brow shape.

**Setup.** Brow In/Mid/Ot L/R: Sphere_Solid, scale 0.05 (R has negative X scale), TranslationY locked, bDrawLimits=false. Visibility is toggled by Brow Tweaker Vis.

**Operators:** `RigUnit_HierarchyAddBone`, `RigUnit_HierarchyAddNull`, `RigUnit_ModifyTransforms (AdditiveLocal)`, `RigUnit_GetCurveValue`, `RigUnit_PositionConstraintLocalSpaceOffset`

**Scale:** 4 helper bones, 2 nulls, 6 micro controls, 8 ModifyTransforms, 6 position constraints

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:794 ModifyTransforms_229 Brow In L Translation Z=4`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:798 PositionConstraintLocalSpaceOffset_1 Child=Null Brow Mid L Parents Brow In L, Brow Ot L`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:93 HierarchyAddBone_25 Brow In L`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Highlevel/Hierarchy/RigUnit_ModifyTransforms.cpp (AdditiveLocal lerp identity->T by clamped weight)`

#### UE6-brow-squeeze — Brow squeeze pad with X-follow null

*important* · assets: CRM_Zebra_Face, CRM_Monster_Face

'Squeeze Null' sits under skull at (0,20,120) (Monster (0,30,70)). The 'Squeeze' control sits under the null with offset rotation Y=+45 degrees, T(0,20,118) and scale (-0.05,0.05,0.05). Its limits are TranslationX [0,200], TranslationY [0,0] and TranslationZ [0,200]. Before the sequence, PositionConstraintLocalSpaceOffset keeps the null X at the average of Brow Main L and Brow Main R (Filter X only, maintain offset). Brow Squeeze Logic outputs Left = clamp(0.01*v.z, 0, 200) and Right = clamp(0.01*v.x, 0, 200). The forward wiring is crossed: brow_squeeze_r <- Left (Z) and brow_squeeze_l <- Right (X).

**Setup.** Squeeze: Sphere_Solid, yellow, scale 2.

**Operators:** `RigUnit_PositionConstraintLocalSpaceOffset`, `RigUnit_GetTransform`, `RigVMFunction_MathDoubleRemap`, `RigUnit_SetCurveValue`

**Scale:** 1 control, 1 null, 2 curves

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:705 PositionConstraintLocalSpaceOffset Child=Squeeze Null Filter=(bX=true,bY=false,bZ=false)`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:2451-2469 Brow Squeeze Logic`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/regen.py SpawnControl_43.Settings MaxValue=(Location=(X=200,Y=0,Z=200))`

#### UE6-eye-convergence — Eye convergence blend of aim targets

*important* · assets: CRM_Zebra_Face, CRM_Monster_Face

Pin K, for each aim null: P = ProjectTransformToNewParent(child = Eye L aim initial, old parent = Eye Aim initial, new parent = Eye Aim current), which gives the parallel-offset target. A = Eye Aim global. T = 1 - Convergence (channel on Eye Aim, read by name). The node sets Eye L aim global translation = Interpolate(A, P, T).translation (SetTransform translation only, propagate). Convergence 0 keeps the targets parallel (offset by the eye spacing); Convergence 1 collapses both targets onto the Eye Aim control, crossing the eyes on it.

**Setup.** Eye Aim.Convergence float channel, 0..1, default 0.

**Operators:** `RigUnit_GetFloatAnimationChannel`, `RigVMFunction_MathDoubleOneMinus`, `RigUnit_ProjectTransformToNewParent`, `RigVMFunction_MathTransformLerp`, `RigUnit_SetTranslation`

**Scale:** 2 blends

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:1142 RigUnit_GetFloatAnimationChannel Control=Eye Aim Channel=Convergence`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:1143 Set Transform_1, :1146 Interpolate`

#### UE6-lid-in-ot-controls — Lid inner/outer corner controls

*important* · assets: CRM_Zebra_Face, CRM_Monster_Face

Lid In L and Lid Ot L are parented to bone eye_main_l with local identity offset, so they pivot at the eye center; the R side uses eye_main_r. Their rotation drives lid_in/lid_ot and the corner end of the lid skin chains through the weighted parent constraints (Value1 and Value2 columns).

**Setup.** Circle_Solid, scale 0.05 (Monster 0.03); shape offsets (10,7,-1.5) and (9,-7,1) (Monster (7,3.5,0) and (7,-3,0)); shape rotated 90 degrees about Y.

**Operators:** `RigUnit_HierarchyAddControlTransform`

**Scale:** 4 controls

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:144 SpawnControl_54 Name=Lid In L`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1144 Face/Lid In L par=B:eye_main_l`

#### UE6-lip-roll — Lip Roll Out/In (channel-weighted additive lip bone poses + curves)

*important* · assets: CRM_Zebra_Face, CRM_Monster_Face

Pin C. Roll Tp and Roll Bt are jaw channels (-100..200). Ot Tp: w = 0.005*RollTp; curve lip_roll_ot_tp_l = clamp(0.01*v, 0, 2). AdditiveLocal: lip_tp rotY -70 degrees (Monster -35), T.z +1.5; lip_tp_04_l/r about rotY -90 (Monster -50) with X -2/+2; lip_tp_03_l/r quat (0.067,-0.25,0.25,0.933). Ot Bt: w = 0.005*RollBt; the curve is named lip_roll_in_bt_l (sic) = clamp(0.01*v, 0, 2); lip_bt rotY +50 (Monster +60) with T.z +0.25, 04_l/r rotY +90 (Monster +55), 03_l/r +20. In Tp: w = -0.005*RollTp + 0.5*SphericalPoseReader('Lip Main Tp'); curve lip_roll_in_tp_l = clamp(-0.01*v, 0, 2). lip_tp rotY +70 with T.z -1.2 (Monster T.x +6); 04_l rotY +90 T.z -3; 04_r rotY +90 T.z +3; 03_r/03_l rotY +60 T.z +1/-1; 02_l/r and 01_l/r rotY +30. In Bt: w = -0.005*RollBt + 0.5*reader('Lip Main Bt'); lip_roll_in_bt_l = -0.01*v (no clamp; this overwrites the Ot Bt write). lip_bt rotY -70, T(2,0,-3); 04_l/r rotY -45, T.z -+2; 03_l/r rotY -20, T.z -+0.75. Monster: lip_bt rotY -90 T(2,0,1); 04 rotY -90; 03 rotY -90 T.z -+2; plus 02_l/r rotY -30. No face control is named 'Lip Main Tp'/'Lip Main Bt', so the pose readers never update and output their default 0. Weights above 1 clamp to 1, and weights at or below 0 are skipped.

**Setup.** jaw.Roll Tp and jaw.Roll Bt: positive values roll the lip out, negative values roll it in.

**Operators:** `RigUnit_GetFloatAnimationChannelFromItem`, `RigVMFunction_MathDoubleMul`, `RigVMFunction_MathDoubleClamp`, `RigVMFunction_MathDoubleAdd`, `RigUnit_SphericalPoseReader`, `RigUnit_ModifyTransforms`, `RigUnit_SetCurveValue`

**Scale:** 4 functions, about 29 ModifyTransforms

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:2666-2818 Lip Roll Ot/In Tp/Bt Funct`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:2710 SetCurveValue_41 Curve=lip_roll_in_bt_l (in Ot Bt)`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:2750 SphericalPoseReader DriverItem=(Type=Control,Name=Lip Main Tp)`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/Highlevel/Hierarchy/RigUnit_SphericalPoseReader.h OutputParam(0.0f)`

#### UE6-teeth-cheek-nose — Teeth, cheek and nose direct controls

*important* · assets: CRM_Zebra_Face, CRM_Monster_Face

Teeth Tp: construction spawns bone 'Teeth Tp' (parent Skull, global = teeth_tp global), a null 'Teeth Tp' under it (unused), and control 'Teeth Tp' under that bone with offset = teeth_tp global. The bone 'Teeth Tp' is also in the New Parent lists, so upper teeth follow Muzzle/Mouth. Teeth Bt: control under jaw bone, offset = teeth_bt global. Cheek L/R: parent Head Attach Null (Monster: bone Skull), offset = cheek_l/r global. Nose: parent bone nose, offset T(0,24,102); Monster uses the nose bone global transform. All of these drive their bones through UE6-control-drives-bone.

**Setup.** Teeth: RoundedTriangle_Thin, yellow, scale 0.1, shape offsets (-5,0,1.5) and (-5,0,-1). Cheek L blue, Cheek R red, shape Default, scale 0.1. Nose: Zebra Circle_Thick yellow with shape rotated 45 degrees about X, scale (0.5,0.5,1); Monster Sphere_Solid, scale 0.05. Monster Nose hosts float channels Sneer L/R and Flare L/R (0..200); Monster Cheek L/R host Squint L/R (0..100, unused).

**Operators:** `RigUnit_HierarchyAddBone`, `RigUnit_HierarchyAddNull`, `RigUnit_HierarchyAddControlTransform`

**Scale:** 5 controls + 1 helper bone + 1 null

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:187 HierarchyAddBone Name=Teeth Tp, :56 SpawnControl_44, :57 SpawnControl_45`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:124 SpawnControl_47 Cheek L`; `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/graphs.txt:133 SpawnControl_81 Nose`

#### UE8-monster-face-parent-missing — Monster face attach null has no driver (Face/Parent unconnected)

*important* · assets: MR_Monster, CRM_Monster_Face, MR_Zebra, CRM_Zebra_Face

In construction, both face modules spawn 'Head Attach Null' with no parent (world level) at the current global transform of the hard-coded bone 'head' (GetTransform_3 Item=Bone head). jaw, Muzzle, Reverse Jaw, Head Squash, Cheek L/R and the other face controls are parented under it. In forward solve, ParentConstraint_37 (Child=Head Attach Null, Parents=[Connector 'Parent', weight 1], bMaintainOffset=True, all axes, Average interpolation) makes the null follow whatever the Parent connector resolves to. Face/Parent is a Secondary, optional connector with the ChildOfPrimary rule. Zebra connects it to bone head. Monster leaves it unconnected. In that case the parent cache is invalid, OverallWeight stays 0, and the constraint does nothing. The Monster null keeps its construction pose, T(0.068,2.118,39.2), so Monster face controls do not follow neck or head animation. (MR_Monster also has no Neck module.) A port should either connect Parent to head or replicate the no-op.

**Setup.** Face module connectors: Root (primary, bone root) and Parent (secondary, optional).

**Operators:** `RigUnit_HierarchyAddNull`, `RigUnit_GetTransform`, `RigUnit_ParentConstraint`

**Scale:** 1 rig affected (MR_Monster); about 70 face controls ride on the null

**Evidence:** `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/graphs.txt:30-31 (HierarchyAddNull_2 'Head Attach Null'; GetTransform_3 Item=Bone head), :711 (ParentConstraint_37 Parents=((Item=(Type=Connector,Name="Parent"),Weight=1.0)))`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:27-28, 663`; `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/hierarchy_export.txt :: 'Key=(Type=Connector,Name="Parent"),Content="(Settings=(Type=Secondary,bOptional=True,Rules=((ScriptStructPath=...RigChildOfPrimaryConnectionRule'`; `<dump>/Game__Assets__Monster__Rig__MR_Monster/runtime_hierarchy.txt:1374 (CONNECTOR Face/Parent), :1451 (NULL Face/Head Attach Null parents=[])`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/modular_rig_model.txt :: 'Name="Face/Parent"),Targets=((Type=Bone,Name="head"'`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Highlevel/Hierarchy/RigUnit_TransformConstraint.cpp:178-220`

#### UE-material-curves — Eye material parameter drive (curves and sequencer material tracks)

*nice-to-have* · assets: CRM_Zebra_Face, Zeb_Face_Expressions, expression_demo_seq

The face rig writes the non-morph curves pupil_dilation and cornea_size, which the animation also stores. They are presumably consumed by the eye material as material curves. expression_demo_seq animates the 'Eyes' material slot directly with a MovieSceneComponentMaterialTrack whose channels are Highlight_Intensity, Cornea Roughness, Sclera Color Multiply R/G/B/A and Highlight_Shape_Pos R/G/B/A.

**Setup.** The face rig's eye controls drive the curves; sequencer drives the material scalars and colors.

**Operators:** `RigUnit_SetCurveValue`, `MovieSceneComponentMaterialTrack`

**Scale:** 2 curves, 10 material channels

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:1125-1126 SetCurveValue_70 Curve=pupil_dilation; SetCurveValue_71 Curve=cornea_size`; `<dump>/Game__Sequences__expression_demo_seq/sequence.txt:12-13 TRACK MovieSceneComponentMaterialTrack name=Material Slot: Eyes`

#### UE6-lip-puff-ch — Lip Puff and Lip CH shapes (Monster only)

*nice-to-have* · assets: CRM_Monster_Face

Pin C, after the rolls. Puff Tp: w = 0.005*PuffTp + 0.5*reader('Lip Main Tp', dangling); curve lip_puff_tp = clamp(0.01*v, 0, 2). It applies the Monster Roll-In-Tp bone deltas (lip_tp rotY +70 T.x +6; 04 rotY +90 T(+-5,0,+-2); 03 rotY +60 T.z +-1; 02/01 rotY +30). Puff Bt: w = 0.005*PuffBt + 0.5*reader('Lip Main Bt'); lip_puff_bt = 0.01*v, unclamped; it applies the Monster Roll-In-Bt deltas. Ch Tp: w = 0.005*ChTp; the curve is named lip_roll_ot_tp_l (sic) = clamp(0.01*v, 0, 2). It applies lip_tp rotY -30 T(2,0,1.5); 04_l/r quat (0.037,-0.421,0.079,0.903); 03_r rotY -60 T.z -3; 03_l rotY -60 T.z +3; 02_l rotY -10 T.z -2; 02_r rotY -10 T.z +2. Ch Bt: w = 0.005*ChBt; curve ch_bt = clamp(0.01*v, 0, 2). It applies lip_bt rotY +40 T.z -1; 04_l rotY +55 T(0,-3,-3); 04_r rotY +55 T(0,3,3); 03_l/r rotY +20. The channels range 0..100, so w reaches at most 0.5.

**Setup.** jaw channels Puff Tp, Puff Bt, Ch Tp, Ch Bt (0..100).

**Operators:** `RigUnit_GetFloatAnimationChannelFromItem`, `RigUnit_ModifyTransforms`, `RigUnit_SphericalPoseReader`, `RigUnit_SetCurveValue`, `RigVMFunction_MathDoubleClamp`

**Scale:** 4 functions

**Evidence:** `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/graphs.txt:3225-3400 Lip Puff Tp/Bt, Lip Ch Tp/Bt Funct`; `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/graphs.txt:3241 SetCurveValue_40 Curve=lip_puff_tp; :3368 Curve=ch_bt`

#### UE6-pupil-iris — Pupil / iris size channels to material-style curves

*nice-to-have* · assets: CRM_Zebra_Face, CRM_Monster_Face

pupil_dilation = remap(GetFloatAnimationChannel(Eye L, Pupil Size), 0..200 -> 0..2, no clamp) + 1, giving 1 at rest and 0..2 over the range -100..100. cornea_size = remap(Iris Size, 0..200 -> -0.1..0.2, no clamp) + 0.225, giving 0.125 at rest (the SetCurveValue default is also 0.125). The two eyes share the channels (Eye R is an extra host). A disconnected SetCurveValue highlight_offset_x = 0.125 is never executed. Neither curve is a morph target.

**Setup.** Pupil Size and Irisl Size (sic) on Eye L/Eye R, range -100..100.

**Operators:** `RigUnit_GetFloatAnimationChannel`, `RigVMFunction_MathDoubleRemap`, `RigVMFunction_MathDoubleAdd`, `RigUnit_SetCurveValue`

**Scale:** 2 curves

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:1125 SetCurveValue_70 Curve=pupil_dilation Value=1`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:1127 Remap_9, :1132 Remap_10 TargetMinimum=-0.1 TargetMaximum=0.2`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:1137 SetCurveValue_72 highlight_offset_x (orphan)`

#### UE6-smile-lid-push — Smile pushes lower lid (Zebra only)

*nice-to-have* · assets: CRM_Zebra_Face

In pin A, AdditiveLocal ModifyTransforms applies to bone 'Lid Bt 02 L' a rotation of Y -7 degrees (quat Y -0.061049) and to 'Lid Bt Base 02 L' Y -9 degrees (quat Y -0.078459), both weighted by Corner Logic(Corner L).smile. The R side does the same with Corner R. Because these precede the blink/soft-eye additive layers, the smile raises the middle lower-lid bone. Monster removed these nodes.

**Setup.** Corner L/R.

**Operators:** `RigUnit_ModifyTransforms`

**Scale:** 4 ModifyTransforms

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:699 ModifyTransforms_2_2_2_1_2_1_3_2_11 Lid Bt 02 L Y=-0.061049 IN Weight<-Corner Logic.smile`

### D15 Direct Mesh Control

#### UE7-dmc-proxy-component — DMC shape rendering: proxy DMC component + DG_DirectMeshControl deformer bound to the source mesh

*core* · assets: DirectMeshControl plugin, MR_ZebraDMC, MR_FN_BipedDMC

At OnPostEngineInit, DirectMeshControlRig registers a provider in FShapeProxyComponentProviderRegistry: UDirectMeshControlProxy -> UDirectMeshControlComponent. For each control using a DMC shape, the provider creates 'DirectMeshControlComponent_CtrlProxy' on the AControlRigShapeActor, set to the proxy's sub-mesh. The component is attached to the shape actor root with no collision, HitProxyPriority=HPP_Wireframe and translucent select.

PostSetupFunction then:
1. Disables culling (bNeverDistanceCull, BoundsScale=1000).
2. On the rig's bound source SkeletalMeshComponent, calls SetAlwaysUseMeshDeformer(true) and SetForcedLOD(1), which forces LOD0.
3. Calls DMCComponent->SetMeshDeformer(/DirectMeshControl/Deformers/DG_DirectMeshControl).
4. Binds the deformer instance settings' ComponentResolver to return the bound source SKM component. If that is gone, it returns the first SkeletalMeshComponent up the attach-parent actor chain.
5. Calls SetupFromDeformer and SetUpdateAnimationInEditor(true).

UDirectMeshControlComponent overrides SendRenderDynamicData_Concurrent. It skips bone-transform processing and only calls MeshObject->Update with no morphs, then enqueues the deformer (ExecutionGroup_BeginInitViews) with a fallback that resets the vertex-factory overrides.

Optimus component source UOptimusDirectMeshControlComponentSource: binding name 'DirectMeshControl', execution domains Vertex and Triangle, one invocation per render section.

The deformer exposes an 'OverlayColor' variable (GetColorVarName). Inference (the DG asset is binary): the deformer looks up each proxy vertex's SubToSource index in the source component's deformed vertex buffer. The DMC shape then overlays the current deformed surface exactly, tinted per group, and clicking that surface selects the control. Hover highlighting uses the material parameters Hovered and HoveredColor.

**Setup.** Nothing to set up per control. A control whose shape resolves to a DMC shape is displayed and picked as a patch of the character's own deformed skin.

**Operators:** `UDirectMeshControlProxy`, `UDirectMeshControlComponent`, `UOptimusDirectMeshControlComponentSource`, `FShapeProxyComponentProviderRegistry`, `AControlRigShapeActor::EnsureProxyComponent`, `DG_DirectMeshControl / DG_DirectMeshControlNoColor`

**Scale:** One DMC component per DMC-shaped control.

**Evidence:** `<UE>/Plugins/Experimental/Animation/DirectMeshControl/Source/DirectMeshControlRig/Private/DirectMeshControlRigModule.cpp: RegisterProvider<UDirectMeshControlProxy, UDirectMeshControlComponent>`; `<UE>/Plugins/Experimental/Animation/DirectMeshControl/Source/DirectMeshControlRig/Private/Units/RigUnit_DirectMeshControl.cpp:169-238 (:188 SetForcedLOD(1), :192 SetMeshDeformer, :203 ComponentResolver)`; `<UE>/Plugins/Experimental/Animation/DirectMeshControl/Source/DirectMeshControl/Private/DirectMeshControlComponent.cpp:36-105`; `<UE>/Plugins/Experimental/Animation/DirectMeshControl/Source/DirectMeshControl/Private/OptimusDirectMeshControlComponentSource.cpp: GetBindingName 'DirectMeshControl', GetExecutionDomains {Vertex, Triangle}`; `<UE>/Plugins/Experimental/Animation/DirectMeshControl/Source/DirectMeshControl/Private/DirectMeshControlUtilities.cpp:36-40 ('OverlayColor'), :79 DG_DirectMeshControl path`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/ControlRigGizmoActor.cpp:43-101`; `<UE>/Plugins/Experimental/Animation/DirectMeshControl/Content/Deformers/DG_DirectMeshControl.uasset`

#### UE7-dmc-shape-library-from-layer — RigUnit_SetupShapeLibraryFromLayer (mesh polygroups -> control shape library)

*core* · assets: CRM_FN_DMC, RunDMC, CRM_FN_FkArray (dead forward-graph copy)

Runs in the construction event only. It needs OnAddShapeLibraryDelegate to be bound, which is editor-side.

Steps:
1. Take the owner SkeletalMeshComponent's mesh and convert its LOD0 MeshDescription to an FDynamicMesh3 (with tangents).
2. Find the triangle label layer named LayerName; if it is missing or LayerName is None, return nothing.
3. GroupNames = the unique label values, i.e. bone names.
4. Get or build the per-group sub-meshes (see UE7-dmc-submesh-generation).
5. Find or create a UControlRigShapeLibrary named after the layer, outer'ed to the source mesh. It is reset when its shape count or sub-mesh identities change.
6. Library settings: DefaultShape name None; DefaultMaterial and XRayMaterial = /DirectMeshControl/Materials/M_DirectMeshControl; MaterialHoveredParameter 'Hovered'; MaterialHoveredColorParameter 'HoveredColor'.
7. One shape per group: ShapeName = label (bone name) and ShapeProxy = a transient UDirectMeshControlProxy outer'ed to the sub-mesh. A PostSetupFunction configures the shape actor's proxy component (see UE7-dmc-proxy-component).
8. Sort the shapes by name and register the library under LibraryName = <mesh sub-path> + LayerName. For an asset this is just the layer name, so shapes resolve as '<layer>.<bone>'.

Output: GroupNames (TArray<FName>).

**Setup.** A shape named '<layer>.<bone>' becomes available to any control whose shape name references it.

**Operators:** `RigUnit_SetupShapeLibraryFromLayer (FRigUnit_DynamicHierarchyBaseMutable)`, `UControlRigShapeLibrary`, `FExecuteContext::OnAddShapeLibraryDelegate`

**Scale:** CRM_FN_DMC: 2 calls. RunDMC: 1 call. FkArray: 2 dead calls in its forward graph.

**Evidence:** `<UE>/Plugins/Experimental/Animation/DirectMeshControl/Source/DirectMeshControlRig/Public/Units/RigUnit_DirectMeshControl.h:10-40 (LayerName default "dmc-polygroup", output GroupNames)`; `<UE>/Plugins/Experimental/Animation/DirectMeshControl/Source/DirectMeshControlRig/Private/Units/RigUnit_DirectMeshControl.cpp:50-253 (:59 bAllowOnlyConstructionEvent = true; :108-109 LibraryName; :167-168 ShapeName/ShapeProxy; :250 OnAddShapeLibraryDelegate.Execute)`; `<dump>/FortniteRigs__Modules__Miscellaneous__CRM_FN_DMC/graphs.txt:12,19 (LayerName=ik-layer / fk-layer)`

#### UE4-shape-name-from-item-v02-dmc — Get Control Shape Name From Item v02 (Direct Mesh Control shape per bone)

*important* · assets: CRFL_Control_v001, CRM_FN_IkFk2Bones, CRM_FN_Foot, CRM_FN_Spine, CRM_FN_FkArray, CRM_FN_FkChain, CRM_FN_DMC

Inputs: Item, ShapeLib Namespace, Default Shape and Direct Mesh Control Libraries (FName[]). Output: Result. found = DMCLibraries.Find(NS).Success. candidate = NS + '.' + ResolveConnector(Item).Name. If found, Result = ShapeExists(candidate) ? candidate : Default Shape. If not found, Result = Default Shape. The If_3 branch (!found AND Default != 'Default') also returns Default. The effect: when the DMC module has registered a polygroup shape library called e.g. 'ik-layer', a control built for bone X uses the mesh-derived gizmo 'ik-layer.X'; otherwise it uses the module's authored shape.

**Setup.** IK controls use namespace 'ik-layer' and FK controls use 'fk-layer'. The Default Shape comes from the module's shape-settings variable Name.

**Operators:** `RigUnit_ResolveConnector`, `RigUnit_ShapeExists`, `DISPATCH_RigVMDispatch_ArrayFind`, `RigVMFunction_MathBoolNot`, `RigVMFunction_MathBoolAnd`, `DISPATCH_RigVMDispatch_CoreNotEquals`, `RigVMFunction_NameConcat`, `DISPATCH_RigVMDispatch_If`

**Scale:** 9 call sites: IkFk2Bones 4, Foot 2, Spine 1, FkArray 1 (inside its local Get Control Shape), FkChain 1 (inside its local Get Control Shape).

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Control_v001/graphs.txt:430-473 'Get Control Shape Name From Item v02' (INARG Direct Mesh Control Libraries : TArray<FName>)`; `ue/<dump>/FortniteRigs__Modules__IkSolves__IkFk2Bones__CRM_FN_IkFk2Bones/graphs.txt:1064-1070 (ShapeLib Namespace=ik-layer x3, fk-layer x1; Direct Mesh Control Libraries <- Get_Module_Metadata 'Direct Mesh Control Libraries' NameSpace=Root)`; `ue/<dump>/FortniteRigs__Modules__Biped__Foot__CRM_FN_Foot/graphs.txt:570-571 (ik-layer / fk-layer)`

#### UE7-dmc-polygroup-tool — DMC polygroup authoring: bone-dominant triangle labels

*important* · assets: SKM (source mesh of MR_ZebraDMC / MR_FN_BipedDMC), DirectMeshControl plugin

The Modeling Mode extension 'DMC' provides the 'Direct Mesh Polygroup' tool (UDirectMeshPolygroupTool), which works on one skeletal mesh, optionally limited to a triangle selection.

Algorithm (FDirectMeshPolygroupsGenerator::FindPolygroupsFromSkinWeights):
1. Per triangle, blend the 3 vertex skin weights with 1/3 each (FBoneWeights::Blend).
2. If bTryUsingQuads is set (default): find source quads with FindSourceMeshPolygonPolygroups (QuadAdjacencyWeight=1, QuadMetricClamp=1, QuadSearchRounds=1, bRespectUVSeams, bRespectHardNormals) and blend the two triangles 0.5/0.5 so both halves of a quad get the same bone.
3. The group bone is the strongest influence (Weights[0]).
4. Bones whose name contains any BonesToRemove substring are replaced by their parent, repeatedly.
5. Group triangles by bone index. Groups smaller than MinGroupSize (default 2) are merged with a neighbour.

On Accept, the tool writes a FDynamicMeshTriangleLabelAttribute layer (LayerName, default 'dmc-polygroup') whose per-triangle value is the bone NAME, and commits the result to the mesh description. The Fortnite templates expect two layers: 'ik-layer' and 'fk-layer'.

**Setup.** Artist-facing tool options: QuadAdjacencyWeight, QuadMetricClamp, QuadSearchRounds, bRespectUVSeams, bRespectHardNormals, MinGroupSize, bShowGroupColors, bTryUsingQuads, BonesToRemove, LayerName.

**Operators:** `UDirectMeshPolygroupTool`, `FDirectMeshPolygroupOp`, `FPolygroupsGenerator`, `FDynamicMeshTriangleLabelAttribute`

**Scale:** Layers expected by the rigs: ik-layer, fk-layer, dmc-polygroup (RunDMC default).

**Evidence:** `<UE>/Plugins/Experimental/Animation/DirectMeshControl/Source/DirectMeshControl/Public/Tools/DirectMeshPolygroupTool.h:54-100 (properties, LayerName="dmc-polygroup")`; `<UE>/Plugins/Experimental/Animation/DirectMeshControl/Source/DirectMeshControl/Private/Tools/DirectMeshPolygroupTool.cpp:82-239 (FindPolygroupsFromSkinWeights)`; `DirectMeshPolygroupTool.cpp:510 (BonesToRemove substring match), :572 'LabelAttribute->SetValue(TriangleID, ... BoneNames[BoneIndex] ...)'`; `<UE>/Plugins/Experimental/Animation/DirectMeshControl/Source/DirectMeshControl/Private/DirectMeshControlModule.cpp (GetExtensionTools 'Direct Mesh Polygroup')`

#### UE7-dmc-submesh-generation — Per-polygroup proxy sub-mesh generation (SubToSource mapping)

*important* · assets: DirectMeshControl plugin

FGroupSubMeshes::Rebuild. For each group ID in the label layer:
1. Extract a FDynamicSubmesh3 of the group's triangles with material IDs and bones disabled, and convert it to a MeshDescription.
2. Replace the skeleton with a single 'Root' bone (identity pose) and bind every vertex to Root with weight 1.
3. Add a float vertex attribute 'SubToSource' holding (source render vertex index + 0.5). The index is found through LOD0 GetRawPointIndices, which maps imported vertices to render vertices. The attribute is enabled for rendering.
4. Set every vertex-instance colour to LinearColors::SelectFColor(groupID).
5. Build a transient USkeletalMesh named '<Mesh>_<Layer>_<GroupID>' with material M_DirectMeshControl and a transient one-bone USkeleton; normals and tangents are not recomputed.
6. Batch-build and finish compilation behind a 'Finalizing Direct Mesh Controls...' dialog.

The results live in UDMCMeshGenerationManager, owned by the editor subsystem UDMCMeshGenerationSubsystem. They are cached per (mesh, layer) and rebuilt only when HashCombine(mesh, SHA1(GetDerivedDataKey())) changes. Nothing is saved to disk.

**Setup.** None.

**Operators:** `FGroupSubMeshes::Rebuild`, `UDMCMeshGenerationManager::GetSubMeshes`, `UDMCMeshGenerationSubsystem (UEditorSubsystem)`, `FStaticToSkeletalMeshConverter`

**Scale:** One sub-mesh per bone group per layer.

**Evidence:** `<UE>/Plugins/Experimental/Animation/DirectMeshControl/Source/DirectMeshControl/Private/DirectMeshControlUtilities.cpp:108-269 (:202 RegisterAttribute SubToSource, :211 RenderIndex + 0.5f, :216 SelectFColor, :224 name format)`; `<UE>/Plugins/Experimental/Animation/DirectMeshControl/Source/DirectMeshControl/Private/DMCMeshGenerationSubsystem.cpp:95-144 (GetDDCHash, rebuild on hash change)`; `<UE>/Plugins/Experimental/Animation/DirectMeshControl/Source/DirectMeshControl/Public/DirectMeshControlUtilities.h:21-59`

#### UE1-dmc-module — CRM_FN_DMC module: Direct Mesh Control layer registration

*nice-to-have* · assets: MR_ZebraDMC, MR_FN_BipedDMC

Runtime-asset module with only a Root connector (-> bone root), parented to root. Its Construction event, when variable 'Direct Mesh Control' is true (bound from the host bool Direct_Mesh_Control=True):
1. Runs RigUnit_SetupShapeLibraryFromLayer for LayerName 'ik-layer' and then 'fk-layer'. These load polygroup layers from the mesh as DMC shape libraries.
2. For each layer whose returned GroupNames is non-empty, adds the layer name to 'Direct Mesh Control Libraries'.
3. Writes module metadata 'Direct Mesh Control Libraries' (FName array) and 'Direct Mesh Control' (bool) in the Root namespace (onto root/RootJoint).

Other modules (FkArray, FkChain) read this to pick on-mesh shapes. In the headless dump no layers were found, so the runtime hierarchy is unchanged. The module needs the DirectMeshControl plugin and uses ModularRigGizmoLibrary_DMC materials.

**Operators:** `RigUnit_SetupShapeLibraryFromLayer`, `DISPATCH_RigDispatch_SetModuleMetadata`, `DISPATCH_RigVMDispatch_ArrayIsEmpty`, `DISPATCH_RigVMDispatch_ArrayAdd`

**Evidence:** `<dump>/FortniteRigs__Modules__Miscellaneous__CRM_FN_DMC/graphs.txt: 'RigUnit_SetupShapeLibraryFromLayer | title=Set Shape Library from Layer | LayerName=ik-layer'`; `<dump>/FortniteRigs__Modules__Miscellaneous__CRM_FN_DMC/graphs.txt: 'Set Module Bool Metadata | Name=Direct Mesh Control; NameSpace=Root'`; `<dump>/Game__Assets__Zebra__Rig__MR_ZebraDMC/runtime_hierarchy.txt:1726 "'Direct Mesh Control Libraries:NAME_ARRAY', 'Direct Mesh Control:BOOL'"`; `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkArray/graphs.txt:51 'SetupShapeLibraryFromLayer_1 ... LayerName=ik-layer'`

#### UE6-get-dmc-shape — Get DMC Shape: conditional direct-mesh-control gizmo names (Zebra)

*nice-to-have* · assets: CRM_Zebra_Face, MR_ZebraDMC

Get DMC Shape(DMC Shape, Default Shape) returns DMC Found ? 'fk-layer.'+DMCShape : DefaultShape. If that shape name does not exist (RigUnit_ShapeExists), it returns 'Circle_Thick' instead. The module-metadata read 'Direct Mesh Control' (namespace Root, bool) is AND-ed with DMC Found, but the result is unused. The Construction graph sets DMC Found from the member 'Direct Mesh Control', which defaults to false and is unbound even in MR_ZebraDMC, so the default shapes are always used. 23 call sites cover jaw, muzzle, mouth, Nose, lip_tp, lip_bt, reverse_jaw, skull, cheek_l/r, lip_corner_l/r, brow_l/r, lid_tp/bt_l/r, eye_l/r, eye_aim and squash controls. A deprecated leftover reads metadata 'Direct Mesh Control Libraries' and runs ArrayFind 'fk-layer'. Monster removes the whole feature.

**Setup.** Would swap face gizmos to DMC library shapes named fk-layer.<part>.

**Operators:** `RigDispatch_GetModuleMetadata`, `RigVMDispatch_If`, `RigUnit_ShapeExists`, `RigVMFunction_NameConcat`, `RigVMFunction_MathBoolAnd`, `RigVMDispatch_ArrayFind`

**Scale:** 23 calls

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:2820-2846 Get DMC Shape (Concat A=fk-layer., If False=Circle_Thick)`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:248 VariableNode_16 Set DMC Found; :276 Find Element=fk-layer`; `<dump>/Game__Assets__Zebra__Rig__MR_ZebraDMC/runtime_hierarchy.txt (Face shapes all default: no fk-layer.* names)`

#### UE7-dmc-rundmc-module — RunDMC: convert any control to a DMC surface shape

*nice-to-have* · assets: RunDMC

RigModule with connector 'Control' (rule ElementType=Control) and socket Root. Variables: Name (FString, tooltip 'The label name that relates to the DMC set (eg: hand_l)') and LayerName (FName, default 'dmc-polygroup').

Construction:
1. If ResolveConnector('Control', SkipSocket=true).bIsConnected AND Name != '', run SetupShapeLibraryFromLayer(LayerName).
2. HierarchySetShapeSettings on the resolved control: bVisible=True, Name=FName(Name), Color=(1,0,0,1), identity transform.

The name is unqualified, so shape lookup falls back to searching every library newest-first, which finds the layer library registered just before. No template in the project uses this module.

**Setup.** Module options: Name (polygroup label, e.g. hand_l) and LayerName. Connect the Control connector to the control to convert.

**Operators:** `RigUnit_ResolveConnector`, `RigUnit_SetupShapeLibraryFromLayer`, `RigUnit_HierarchySetShapeSettings`, `DISPATCH_RigDispatch_FromString`, `DISPATCH_RigVMDispatch_CoreNotEquals`

**Scale:** 0 uses in the project's rigs.

**Evidence:** `<dump>/Game__BonusContent__Modules__RunDMC/graphs.txt:4-26`; `<dump>/Game__BonusContent__Modules__RunDMC/summary.json variables (Name tooltip, LayerName default dmc-polygroup)`; `<dump>/Game__BonusContent__Modules__RunDMC/asset.t3d:945 (Description 'Assign to any control, that control will be converted to use a DMC set.')`; `<dump>/Game__BonusContent__Modules__RunDMC/regen.py:11-36`

#### UE8-seq-dmc-layer-names-not-animlayers — 'fk-layer'/'ik-layer' strings in MR_Zebra_Take1 are DMC polygroup layers, not animation layers

*nice-to-have* · assets: MR_Zebra_Take1, CRM_FN_DMC, CRM_FN_FkArray, RunDMC

RigUnit_SetupShapeLibraryFromLayer reads a polygroup triangle-label layer (default 'dmc-polygroup') from the source skeletal mesh, builds a sub-mesh per polygroup, and registers the result as a UControlRigShapeLibrary. CRM_FN_DMC and CRM_FN_FkArray call it with LayerName=ik-layer and fk-layer. MR_Zebra_Take1 contains 'fk-layer', 'ik-layer', 'Direct Mesh Control' and '/FortniteRigs/Controls/ModularRigGizmoLibrary_DMC', so the rig instance serialized with the take was DMC-enabled. The current MR_Zebra uses only ModularRigGizmoLibrary, and its sequence track shows 1 section, so there are no animation layers. An importer should not create animation layers from these strings.

**Setup.** none

**Operators:** `RigUnit_SetupShapeLibraryFromLayer`

**Scale:** 1 sequence

**Evidence:** `<UE>/Plugins/Experimental/Animation/DirectMeshControl/Source/DirectMeshControlRig/Public/Units/RigUnit_DirectMeshControl.h:10-35`; `<dump>/FortniteRigs__Modules__Miscellaneous__CRM_FN_DMC/graphs.txt :: 'SetupShapeLibraryFromLayer | ... LayerName=ik-layer' / 'LayerName=fk-layer'`; `<dump>/Game__BonusContent__Modules__RunDMC/summary.json :: LayerName default 'dmc-polygroup'`; `MR_Zebra_Take1.uasset name table: 'fk-layer','ik-layer','ModularRigGizmoLibrary_DMC','Direct Mesh Control'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/asset.t3d:2087 (ShapeLibraries(0)=ModularRigGizmoLibrary)`

### D16 Modular system

#### UE1-connection-rules — Connection rule semantics (Type, Tag, ChildOfPrimary, And, Or, ArraySize)

*core* · assets: MR_Zebra, MR_FN_Biped, MR_Monster, MR_Boombox

Each rule's Resolve(target, input) returns PossibleTarget or InvalidTarget(message).
- FRigTypeConnectionRule{ElementType}: valid iff target.key.IsTypeOf(ElementType). Default type is Socket.
- FRigTagConnectionRule{Tag}: valid iff hierarchy.HasTag(target, Tag).
- FRigChildOfPrimaryConnectionRule: resolve the module's primary connector. Invalid if the primary is unresolved, or if target == primary target ('already used for the primary'). If the primary target is a Socket, its first parent stands in for it. Valid iff target is parented (recursively) under the primary target.
- FRigAndConnectionRule{ChildRules}: evaluate children in order and return the first invalid result, otherwise valid.
- FRigOrConnectionRule{ChildRules}: return the first valid child result, otherwise the last result.
- FRigArraySizeConnectionRule{MinNumConnections (bMinEnabled), MaxNumConnections (bMaxEnabled)}: never filters targets; it only bounds the number of connections for array connectors.

Used in these assets: ChildOfPrimary (Mid/End/Ball/Toe/Arm/Leg elements/Face Parent/Root Body), Type=Control (Prop 'Control Vis Channel Host'), Type=Socket (AddControl primary), and Or(Type Null|Control|Bone) (AddControl 'Parent Control'). Tag, And and ArraySize are available but unused in these rigs.

**Operators:** `FRigConnectionRule`, `FRigTypeConnectionRule`, `FRigTagConnectionRule`, `FRigChildOfPrimaryConnectionRule`, `FRigAndConnectionRule`, `FRigOrConnectionRule`, `FRigArraySizeConnectionRule`, `FRigConnectionRuleInput`

**Scale:** ChildOfPrimary on 32 connectors in Zebra and 37 in Biped. Boombox: 7x Or(Type) rules and 7x Type=Socket rules.

**Evidence:** `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Rigs/RigConnectionRules.h:111-254`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Rigs/RigConnectionRules.cpp:231-282 (And/Or)`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Rigs/RigConnectionRules.cpp:289-320 (Type/Tag)`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Rigs/RigConnectionRules.cpp:326-372 (ChildOfPrimary incl. socket-parent substitution)`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Rigs/RigConnectionRules.cpp:378-404 (ArraySize does not restrict)`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/hierarchy_export.txt: 32x 'RigChildOfPrimaryConnectionRule'`

#### UE1-connector-settings — Connectors: primary/secondary, optional, array, post-construction, rules

*core* · assets: MR_Zebra, MR_FN_Biped, MR_Monster, MR_Boombox, CRU_PropAim

A connector is a CONNECTOR element with FRigConnectorSettings{Description, Type (Primary|Secondary), bOptional, bIsArray (accepts several targets), bPostConstruction (resolved against elements spawned after construction), Rules[] (FRigConnectionRuleStash = ScriptStructPath + ExportedText)}.

Each module must have exactly one primary connector; it gets default settings. Inside a module graph a connector name resolves through the element-key redirector to its connected target(s).

Exposed connectors per module class:
- Root: RootJoint(P), Body(S, opt, ChildOfPrimary)
- Body: Parent(P), Body/Right Hip/Left Hip (S, opt)
- Spine: Start(P), End(S, ChildOfPrimary), End Parent(S), Start Parent(S), Start Orient Spaces/End Orient Spaces (S, opt, array), Start Snap To(S, opt)
- IkFk2Bones: Start(P), Mid(S, ChildOfPrimary), End(S, ChildOfPrimary), Parent(S, opt), IK Spaces/FK Spaces (S, opt, array)
- Foot: Foot Joint(P), Ball Joint(S, ChildOfPrimary), Toe Joints(S, opt, array, ChildOfPrimary), Toe Tip/Heel/Inner Bank/Outer Bank Pivot (S, opt)
- LimbTwist: Start(P), Parent(S, opt), End(S, ChildOfPrimary)
- FkChain: Start(P), Parent(S, opt), End(S, opt, ChildOfPrimary), Orient Spaces(S, opt, array)
- FkArray: Parent(P), Bones(S, array), Spaces/Orient Spaces/Override Parents (S, opt, array)
- Prop: Parent(P), Spaces(S, opt, array), Control Vis Channel Host(S, opt, Type=Control)
- StretchFeedback: Root(P), Spine Elements(S, opt, array), Arm Elements/Leg Elements (S, opt, array, ChildOfPrimary), Vis Channel Control(S, opt)
- Pin: Root(P), Drivers/Driven (S, opt, array)
- ProxyControl: Parent(P), Driven Controls(S, opt, array), Snap To(S, opt)
- DMC: Root(P)
- Zebra/Monster Face: Root(P), Parent(S, opt, ChildOfPrimary)
- Engine AddControl: Add Control Primary(P, Type=Socket), Parent Control(S, opt, Or(Null, Control, Bone))

**Operators:** `FRigConnectorElement`, `FRigConnectorSettings`, `EConnectorType`, `FRigModuleConnector`, `FRigModuleSettings.ExposedConnectors`, `RigUnit_ResolveConnector`, `RigUnit_ResolveArrayConnector`

**Scale:** Zebra 149 connectors. Biped 192. Monster 26. Boombox 15. PropAim 3.

**Evidence:** `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Rigs/RigHierarchyElements.h:1956-1973 (Description, Type, bOptional, bIsArray, bPostConstruction, Rules)`; `<dump>/FortniteRigs__Modules__IkSolves__IkFk2Bones__CRM_FN_IkFk2Bones/summary.json: '(Name="IK Spaces",Settings=(Type=Secondary,bOptional=True,bIsArray=True))'`; `<dump>/FortniteRigs__Modules__FkSolves__Prop__CRM_FN_Prop/summary.json: 'RigTypeConnectionRule\",ExportedText=\"(ElementType=Control)'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/asset.t3d:735 'RigModuleSettings=(Identifier=(Name="CRM_Zebra_Face")...ExposedConnectors=((Name="Root"),(Name="Parent"'`; `<dump>/Game__Assets__Environment__Rig__MR_Boombox/hierarchy_export.txt: 'RigOrConnectionRule' ... '(ElementType=Null)' '(ElementType=Control)' '(ElementType=Bone)'`; `<dump>/units_used.json: RigUnit_ResolveConnector 33, RigUnit_ResolveArrayConnector 17`

#### UE1-modular-rig-model — Modular rig model: module list, parent module, asset reference, overrides, bindings, connections

*core* · assets: MR_Zebra, MR_ZebraDMC, MR_FN_Biped, MR_FN_BipedDMC, MR_Monster, MR_Boombox, CRU_PropAim

A modular rig is a host ControlRig (class ModularRig) whose content is an FModularRigModel. The model holds: (1) Modules[]: FRigModuleReference{Name, ParentModuleName, ControlRigAssetReference (either BlueprintRigClass=<module BP class> or ControlRigAsset=<ControlRigRuntimeAsset>), ConfigOverrides (FControlRigOverrideContainer of per-module variable values), Bindings (TMap moduleVarName -> source var path)}; (2) Connections.ConnectionList[]: {Connector key 'Module/ConnectorName', Targets[] element keys (Bone/Null/Control/Socket)}; (3) PreviousModulePaths (old path -> current name, used for recovery after renames or reparents). The host graph itself is empty (a single BeginExecution). All controls are produced by module construction events at runtime.

**Setup.** Zebra: 34 modules. Biped template: 48. Monster: 7. Boombox: 8. PropAim: 1. The host rig exposes only plain member variables (Arm_L_RotateOrder, Arm_R_RotateOrder, and Direct_Mesh_Control in the DMC variants).

**Operators:** `UModularRig`, `FModularRigModel`, `FRigModuleReference`, `FModularRigConnections`, `FModularRigSingleConnection`, `FControlRigOverrideContainer`, `UModularRigController`

**Scale:** Zebra: 34 modules, 149 connectors, 180 connection entries (20 of them stale). Biped: 48 modules, 192 connectors.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/modular_rig_model.txt: '(Modules=((Name="root",ControlRigAssetReference=(BlueprintRigClass="/FortniteRigs/Modules/FkSolves/Root/CRM_FN_Root.CRM_FN_Root_C")'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/graphs.txt: 'nodes=1 | events=['Forwards Solve']'`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/ModularRigModel.h:58-121 (FRigModuleReference: Name, ParentModuleName, ControlRigAssetReference, ConfigOverrides, Bindings)`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/ModularRigModel.h:393-406 (Modules, Connections, PreviousModulePaths)`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/summary.json: 'runtime_instance_class': 'ModularRig'`

#### UE1-module-config-values — Per-module configuration values (ConfigOverrides) and their effective values

*core* · assets: MR_Zebra, MR_ZebraDMC, MR_FN_Biped, MR_Monster, MR_Boombox

Each module instance overrides public variables of its module class. ExecuteQueue re-applies them to module memory after a VM init (ConfigOverrides.CopyToSubject). The dumped modular_rig_model.txt prints 'ConfigOverrides=()' for every module. The effective per-instance values appear as property diffs on the preview instance sub-objects in asset.t3d (MR_Zebra_C_0.<Module>).

Zebra values:
- root: Control Scale 1.8, Global Control Scale 1.1, Lock Scale True.
- Body: Control Scale 0.8.
- Spine: Control Scale 1.2; IK shapes color (1, 0.51, 0.15); Sec FK color (0.98, 0.65, 1); FK Primary Display Names [Pelvis FK, Waist FK, Chest FK]; IK Start/Mid/End names [pelvis, Waist, Chest]; End Moveable Pivot Display Name 'Chest Moveable Pivot'; Control Scale Factor Profile curve.
- Neck: Is Neck True; Control Scale 2.0; Color yellow; names [Neck Base FK, Neck Mid FK, Head FK] / [Neck Base, Neck Mid, head].
- Leg L: IK Rotation Offset quat (-0.707, 0, 0.707, 0); PV Distance Scale 1.25; PV Twist Follow 1; IK End Align False; FK Rotation Offset (-1, 0, 0, 0); End Bone Rotation Offset; Rotation Order XYZ; names UpperLeg FK / LowerLeg FK / Foot FK / Knee.
- Leg R: adds Primary Axis (-1, 0, 0).
- Arm L: PV Distance Scale 2; Default IK False; Rotation Order XZY (overridden by binding, see UE1-variable-bindings); Default FK Space Index 0; names UpperArm FK / LowerArm FK / Hand FK / Elbow; PV shape scale 0.5.
- Arm R: IK Rotation Offset (-1, 0, 0, 0); Primary Axis (-1, 0, 0).
- Foot L: Inner Bank (-5, 0, -3); Outer Bank (6, 0, -3); Toe Tip (0, 6.8, -2.5); Heel (0, -20, -3).
- Foot R: mirrored X values; Negative Side True.
- Twist: Twist Search String 'thigh|twist' / 'calf|twist' / 'lowerarm|twist' (arm upper keeps the default 'upperarm|twist'). Twist Weights: arm upper [0.1, 0.5, 0.75, 1]; arm lower L [0.75, 0.52, 0, 0.1] (R [0.75, 0.5, 0.25, 0.1]); leg upper [0.1, 0.65, 0.75, 1]; leg lower [0.85, 0.65, 0.1, 0.45]. Lower twists have Twist Reverse True. Twist Axis (-1, 0, 0) on R and leg-upper L.
- Clavicles: Lock Scale; shape offset (+/-12, 0, +/-8) at scale 0.25; Rotation Order XZY; Mirror Behavior; name 'Clavicle'.
- Fingers: Control Transform Offset quat (-1, 0, 0, 0); Circle_Pins_Thick at scale (0.45, 0.45, 0.2); names Base / Mid / Tip.
- Tweakers: Control Scale 0.4; both visibility initials True.
- Ear/Mohawk: FK Control Shape transforms.

Boombox (engine modules): 'Root Module Settings' ControlSize 0.5; AddControl 'Control Settings' Shape Box_Thick / Box_Thin / Sphere_Thick / Square_Thick with offsets; 'Module Settings' ControlSize 5 / 0.5.

**Setup.** The config values set control scale and shape transforms, display names, default IK/FK state, default space index, pivot offsets and twist weights.

**Operators:** `FControlRigOverrideContainer::CopyToSubject`, `UModularRigController::SetConfigValueInModule`, `UModularRigController::ResetConfigValueInModule`

**Scale:** Zebra: 32 of 34 modules have non-default values (all except Prop and Face). Biped: most of its 48.

**Evidence:** `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/ModularRig.cpp:685-689 (ConfigOverrides.CopyToSubject after InitializeVM)`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/asset.t3d:373-768 (MR_Zebra_C_0 module blocks, e.g. 'FK Start Display Name="UpperArm FK"', 'Is Neck=True', 'Twist Search String="thigh|twist"')`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/modular_rig_model.txt: 'ConfigOverrides=()' (dump shows empty)`; `<dump>/Game__Assets__Environment__Rig__MR_Boombox/asset.t3d: 'Control Settings=(Shape=(Name="Box_Thin"'`; `<dump>/FortniteRigs__Modules__Biped__LimbTwist__CRM_FN_LimbTwist/summary.json: default 'Twist Search String:FString=upperarm|twist', 'Twist Weights=(0.25,0.75,1.0)'`

#### UE1-module-execution-order — Depth-first per-module event execution queue

*core* · assets: MR_Zebra, MR_Monster, MR_FN_Biped, MR_Boombox

Each module is a separate UControlRig sub-object of the host, named after the module. For every event the host runs (Construction/PrepareForExecution, Forwards Solve, Backwards Solve, Connector, Interaction, user events), UModularRig::Execute_Internal walks the module tree depth-first pre-order (RootModules, then CachedChildren recursively) and queues each module that supports the event. Interaction events are queued only for modules that own an interacted element.

ExecuteQueue then does the following for each queued module:
- sets the element-key redirector (connectors resolve to their targets)
- sets the execute-context namespace
- copies the host unit context, including the ElementsBeingInteracted filtered to the module
- re-applies ConfigOverrides after a VM re-init
- runs UpdateModuleVariables (bindings)
- runs the module VM
- records construction spawn indices

The observed Zebra order (module sub-object order): root, Prop, Body, Spine, Leg L, Foot L, Leg Upper Twist L, Leg Lower Twist L, Leg R, Foot R, (R twists), Clavicle L, Arm L, Thumb L, Index L, Pinky L, Arm Upper/Lower Twist L, Clavicle R, Arm R, (R fingers and twists), Neck, Tweakers, Face, Ear Base L, Ear L, Ear Base R, Ear R, Mohawk. Parent/child module relationships therefore define evaluation order, not transform parenting.

**Operators:** `UModularRig::Execute_Internal`, `UModularRig::ExecuteQueue`, `UModularRig::ForEachModule(bDepthFirst=true)`, `UModularRig::TraverseModules`, `FRigModuleExecutionElement`

**Scale:** Every rig. Zebra: 35 module VMs per event (36 with DMC).

**Evidence:** `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/ModularRig.cpp:450-486 (queue modules per event)`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/ModularRig.cpp:581-756 (ExecuteQueue)`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/ModularRig.cpp:1312-1339 (ForEachModule depth-first)`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/ModularRig.h:285 (bDepthFirst = true default)`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/asset.t3d:35-763 (module sub-objects root|Prop|Body|Spine|Leg L|Foot L|...|Neck|Tweakers|Face|Ear Base L|Ear L|Ear Base R|Ear R|Mohawk)`

#### UE1-module-instances-zebra — MR_Zebra module instance table (name, parent, class)

*core* · assets: MR_Zebra, MR_ZebraDMC

Module tree (child <- parent : class):
- root (no parent): CRM_FN_Root
- Prop <- root : CRM_FN_Prop
- Body <- root : CRM_FN_Body
- Tweakers <- root : CRM_FN_FkArray
- Face <- root : CRM_Zebra_Face (runtime asset)
- Spine <- Body : CRM_FN_Spine
- Neck <- Spine : CRM_FN_Spine
- Leg L/R <- Spine : CRM_FN_IkFk2Bones
- Clavicle L/R <- Spine : CRM_FN_FkChain
- Foot L/R <- Leg L/R : CRM_FN_Foot
- Leg Upper/Lower Twist L/R <- Leg L/R : CRM_FN_LimbTwist
- Arm L/R <- Clavicle L/R : CRM_FN_IkFk2Bones
- Thumb, Index, Pinky L/R <- Arm L/R : CRM_FN_FkChain
- Arm Upper/Lower Twist L/R <- Arm L/R : CRM_FN_LimbTwist
- Ear Base L/R <- Face : CRM_FN_FkArray
- Ear L/R <- Ear Base L/R : CRM_FN_FkArray
- Mohawk <- Face : CRM_FN_FkArray

MR_ZebraDMC adds CRM_FN_DMC <- root (the CRM_FN_DMC runtime asset). Ears and Mohawk are parented to Face so they run after the face module; they depend on bone skull_tp, which Face drives.

**Setup.** Module names carry the side suffix (' L' or ' R') and become the namespace prefix of every spawned element, e.g. 'Arm L/FK 0'.

**Operators:** `FRigModuleReference`, `CRM_FN_Root`, `CRM_FN_Body`, `CRM_FN_Spine`, `CRM_FN_IkFk2Bones`, `CRM_FN_Foot`, `CRM_FN_LimbTwist`, `CRM_FN_FkChain`, `CRM_FN_FkArray`, `CRM_FN_Prop`

**Scale:** 34 modules. Classes: 8x LimbTwist, 6x FkChain fingers + 2x FkChain clavicles, 6x FkArray, 4x IkFk2Bones, 2x Foot, 2x Spine, and one each of Root, Body, Prop and Face.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/modular_rig_model.txt: 'Name="Arm L",ParentModuleName="Clavicle L"'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/modular_rig_model.txt: 'Name="Ear L",ParentModuleName="Ear Base L"'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/modular_rig_model.txt: 'Name="Face",ParentModuleName="root",ControlRigAssetReference=(ControlRigAsset="/Game/Assets/Zebra/Rig/CRM_Zebra_Face.CRM_Zebra_Face")'`; `<dump>/Game__Assets__Zebra__Rig__MR_ZebraDMC/modular_rig_model.txt: 'Name="CRM_FN_DMC",ParentModuleName="root"'`

#### UE1-module-metadata-bus — Module metadata stored on primary connectors (inter-module communication)

*core* · assets: MR_Zebra, MR_ZebraDMC, MR_FN_Biped, MR_Monster

Modules publish values with SetModuleMetadata(Name, NameSpace, Value). The values are stored as metadata on the module's primary connector. Other modules read them with GetModuleMetadata using NameSpace Self, Parent or Root.
- Root module publishes on root/RootJoint: 'Global Control', 'Root Control', 'Local Control' and 'Body Control' (RIG_ELEMENT_KEY), 'Global Control Scale' (FLOAT), and 'Global Left/Right/Center Control Color' (LINEAR_COLOR).
- IkFk2Bones publishes on '<Leg|Arm>/Start': 'Match IK', 'Match FK', 'Key Controls' (BOOL) and 'IK Null', 'IK Control', 'IK Driver' (RIG_ELEMENT_KEY). The Foot module (a child of Leg) reads these through the Parent namespace to attach its pivots under the leg IK and to follow match/key requests.
- DMC module publishes 'Direct Mesh Control Libraries' (NAME_ARRAY) and 'Direct Mesh Control' (BOOL) in the Root namespace.

**Operators:** `DISPATCH_RigDispatch_SetModuleMetadata`, `DISPATCH_RigDispatch_GetModuleMetadata`, `DISPATCH_RigDispatch_SetMetadata`, `DISPATCH_RigDispatch_GetMetadata`

**Scale:** GetModuleMetadata 44 and SetModuleMetadata 25 across all modules.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1726 'CONNECTOR root/RootJoint ... 'Global Control:RIG_ELEMENT_KEY', 'Root Control:RIG_ELEMENT_KEY', 'Local Control:RIG_ELEMENT_KEY', 'Global Control Scale:FLOAT', 'Global Left Control Color:LINEAR_COLOR'...'Body Control:RIG_ELEMENT_KEY''`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1742 'CONNECTOR Leg L/Start ... 'Match IK:BOOL', 'Match FK:BOOL', 'Key Controls:BOOL', 'IK Null:RIG_ELEMENT_KEY', 'IK Control:RIG_ELEMENT_KEY', 'IK Driver:RIG_ELEMENT_KEY''`; `<dump>/FortniteRigs__Modules__Miscellaneous__CRM_FN_DMC/graphs.txt: 'Set Module FName Array Metadata | Name=Direct Mesh Control Libraries; NameSpace=Root'`; `<dump>/units_used.json: DISPATCH_RigDispatch_GetModuleMetadata 44, SetModuleMetadata 25`

#### UE1-zebra-connections — MR_Zebra connector -> target map (primary/secondary/array)

*core* · assets: MR_Zebra, MR_ZebraDMC

Live connections (connector -> target):
- root/RootJoint -> bone root; root/Body -> pelvis.
- Body/Parent -> Control root/Local; Body/Body -> pelvis; Body/Right Hip -> thigh_r; Body/Left Hip -> thigh_l.
- Spine: Start -> pelvis; End -> spine_05; End Parent and Start Parent -> Body/Body; Start Orient Spaces and End Orient Spaces -> [root/Global, root/Local, Body/Body]; Start Snap To -> Body/Body.
- Neck: Start -> neck_01; End -> head; Start Parent and End Parent -> spine_05; Start/End Orient Spaces -> [root/Global, root/Local, Body/Body, spine_05]; Start Snap To unconnected.
- Clavicle L: Start -> clavicle_l; Parent -> spine_05; Orient Spaces -> [Body/Body]; End unconnected.
- Arm L: Start/Mid/End -> upperarm_l/lowerarm_l/hand_l; Parent -> clavicle_l; IK Spaces -> [clavicle_l, spine_05, pelvis, root/Local, Body/Body, root/Global, Prop/Prop, Prop/Prop Attach 01, Prop/Prop Attach 02]; FK Spaces -> [spine_05, root/Local, Body/Body].
- Leg L: Start/Mid/End -> thigh_l/calf_l/foot_l; Parent -> pelvis; IK Spaces -> [spine_05, Body/Body, root/Local, root/Global]; FK Spaces -> [Body/Body, root/Local].
- Foot L: Foot Joint -> foot_l; Ball Joint -> ball_l; Toe Tip/Heel/Inner Bank/Outer Bank Pivot -> nulls foot_l_toe_tip/foot_l_heel/foot_l_inner/foot_l_outer; Toe Joints unconnected.
- LimbTwist (Start, End, Parent): Arm Upper (upperarm, lowerarm, Body/Body); Arm Lower (lowerarm, hand, Body/Body); Leg Upper (thigh, calf, Body/Body); Leg Lower (calf, foot, Body/Body).
- Fingers (Start, End, Parent): Thumb (thumb_01, thumb_03, hand); Index (index_metacarpal, index_02, hand); Pinky (pinky_metacarpal, pinky_02, hand). Orient Spaces unconnected.
- Prop: Parent -> root/Local; Spaces -> [hand_r, hand_l, spine_05]; Control Vis Channel Host -> root/Global.
- FkArray (Parent, Bones, Override Parents): Tweakers (root, [def_thigh_in_l, def_thigh_in_r], none); Ear Base L/R (root, [ear_base_x], [skull_tp]); Ear L/R (root, [ear_01_x, ear_02_x], none); Mohawk (skull_tp, [mohawk_bk, mohawk_fr], none).
- Face: Root -> root; Parent -> head.
- ZebraDMC only: CRM_FN_DMC/Root -> root.

**Operators:** `FModularRigConnections`, `FRigElementKeyRedirector`

**Scale:** Zebra: 129 live connector entries plus 20 stale ones. Array connectors with more than one target: IK Spaces (9/4), FK Spaces (3/2), Orient Spaces (3-4), Prop Spaces (3), FkArray Bones (1-2).

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/modular_rig_model.txt: 'Name="Arm L/IK Spaces"),Targets=((Type=Bone,Name="clavicle_l"),...(Type=Control,Name="Prop/Prop Attach 02"))'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/modular_rig_model.txt: 'Name="Foot L/Heel Pivot"),Targets=((Type=Null,Name="foot_l_heel"))'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/modular_rig_model.txt: 'Name="Ear Base L/Override Parents"),Targets=((Type=Bone,Name="skull_tp"))'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/asset.t3d:2086 'ArrayConnectionMap=...'`

#### UE2-module-config-variables — Public member variables as module configuration

*core* · assets: CRM_FN_Root, CRM_FN_Body, CRM_FN_Spine, CRM_FN_FkChain, CRM_FN_FkArray, CRM_FN_Prop

Variables with bPublic=True are the per-instance configuration (ModularRig ConfigOverrides), grouped by category ('Module Options', 'Control Shapes Options', 'Mirroring', 'Display Name Options'). Private variables hold element keys and state and are rebuilt during construction.

Public variables per module:
- Root: Control Scale, Global Control Scale, Global Center/Left/Right Control Color, Lock Scale
- Body: Drive Body Joint, Control Scale, Lock Scale, Body Aim Control Offset, Body Aim Axis, Create Body Offset Control
- Spine: Control Scale, Color, Default Distribute Rotation, Is Neck, Rotation Order, Controls Orient Offset, Sec FKs Orient Offset, FK Primary/Secondary Display Names, IK Start/Mid/End Display Name, End Moveable Pivot Display Name
- FkChain: Control Names As FK, Drive Bones, Lock Scale, Default Orient Space Index, Control Scale, Rotation Order, Mirror Behavior, Display Names, Mirror Axis, Use Active Skeleton
- FkArray: Standard FK Names, Drive Bones, Rotation Order, Use Active Skeleton, Mirror Axis, Mirror Behavior, Control Scale, Display Names, Controls Visibility Initials
- Prop: Control Scale, Rotation Order (YXZ)

State variables persist between evaluations and are part of the solve: Root 'Global Control Snapped' and 'Global Control Transform'; Body aim buffers; Prop pivot-previous and previous-buffer transforms.

**Operators:** `ControlRigBlueprint member variables`, `ModularRig ConfigOverrides`, `Bindings (e.g. Arm 'Rotation Order' -> 'Arm_L_RotateOrder' in the MR_Zebra model)`

**Scale:** 6-26 variables per module

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkChain/summary.json variables (bPublic=True)`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/summary.json variables`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt 'Clavicle L/FK ... preferred_rotation_order=XZY' (an override differing from the default XYZ)`

#### UE2-module-connectors — Exposed connectors, rules and primary sockets of FK-family modules

*core* · assets: CRM_FN_Root, CRM_FN_Body, CRM_FN_Spine, CRM_FN_FkChain, CRM_FN_FkArray, CRM_FN_Prop, CRM_FN_Pin

Each module declares RigModuleSettings.ExposedConnectors. The first connector is primary; the others are Secondary, optionally with bOptional, bIsArray and Rules. Each module also has a default SOCKET for its primary connection (metadata SocketDesiredParent).

- Root: RootJoint; Body [opt, ChildOfPrimary rule]
- Body: Parent; Body, Right Hip, Left Hip [opt]
- Spine: Start; End [ChildOfPrimary]; End Parent; Start Parent; Start/End Orient Spaces [opt, array]; Start Snap To [opt]
- FkChain: Start; Parent [opt]; End [opt, ChildOfPrimary]; Orient Spaces [opt, array]
- FkArray: Parent; Bones [array]; Spaces, Orient Spaces, Override Parents [opt, array]
- Prop: Parent; Spaces [opt, array]; Control Vis Channel Host [opt, RigTypeConnectionRule ElementType=Control]
- Pin: Root; Drivers, Driven [opt, array]

Construction resolves them with RigUnit_ResolveConnector / RigUnit_ResolveArrayConnector (bIsConnected gates optional logic). Connector element keys can also be used directly as parents: FkChain and Spine set Parent / Start Parent / End Parent to the literal connector key.

**Setup.** Template wiring (MR_FN_Biped):
- Body/Parent -> Root/Local
- Spine Start/End Parent -> Body/Body
- Clavicle Orient Spaces -> Body/Body
- Prop/Parent -> root/Local
- Prop/Control Vis Channel Host -> Root/Global

In MR_Zebra, Ear Base L/R get Override Parents -> skull_tp.

**Operators:** `RigUnit_ResolveConnector`, `RigUnit_ResolveArrayConnector`, `RigChildOfPrimaryConnectionRule`, `RigTypeConnectionRule`

**Scale:** 7 module types; 2-7 connectors each

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Root__CRM_FN_Root/summary.json 'rig_module_settings'`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/summary.json 'ExposedConnectors=((Name="Start"),(Name="End",Settings=(Type=Secondary,Rules=((ScriptStructPath="/Script/ControlRig.RigChildOfPrimaryConnectionRule"'`; `<dump>/FortniteRigs__Modules__FkSolves__Prop__CRM_FN_Prop/summary.json 'RigTypeConnectionRule",ExportedText="(ElementType=Control)'`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt (ConstructionGraph) 'Set End Parent ... Value=(Type=Connector,Name="End Parent")'`

#### UE3-foot-connectors-config — Foot module connectors and config

*core* · assets: CRM_FN_Foot

Connectors:
- Foot Joint (primary).
- Ball Joint and Toe Joints (array, optional), both ChildOfPrimary.
- Optional Toe Tip Pivot, Heel Pivot, Inner Bank Pivot, Outer Bank Pivot. Zebra connects them to authored nulls foot_l_toe_tip, foot_l_heel, foot_l_inner, foot_l_outer.

Public defaults:
- Inner Bank Offset (5,0,0), Outer Bank Offset (-6,0,0), Toe Tip Pivot Offset (0,8,0), Heel Pivot Offset (0,-22,0). These are used only when the pivot connector is unconnected and are expressed in the footprint space.
- Control Scale 1, Negative Side False, FK Control Mirror Behavior True.
- Shapes: Ball IK Box_Thick, Pivot Sphere_Solid, Toe HalfCircle_Thick, Foot Rocker Sphere_Thick.

Biped overrides: Inner (-5,0,0), Outer (6.5,0,0) (R: -6.5), Toe (0,7,0), Heel (0,-20,0), Foot R Negative Side = True. Zebra: offsets with Z = -3/-2.5 and pivot shape scale .35.

**Setup.** The module settings panel.

**Operators:** `RigUnit_ResolveConnector`, `RigUnit_ResolveArrayConnector`

**Scale:** 2 per biped rig (4 rigs)

**Evidence:** `<dump>/FortniteRigs__Modules__Biped__Foot__CRM_FN_Foot/summary.json rig_module_settings + variables`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/asset.t3d 'Foot R | Negative Side=True'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/modular_rig_model.txt 'Foot L/Heel Pivot' -> foot_l_heel`

#### UE3-ikfk-config — IkFk2Bones public config variables and per-instance overrides

*core* · assets: CRM_FN_IkFk2Bones, MR_FN_Biped, MR_Zebra

Public variables and defaults: Control Scale=1; Color=white (white/grey means take the side color from metadata); IK Rotation Offset=identity quat; PV Distance Scale=1; PV Twist Follow=0; IK End Align=True; Default IK=True; IK Compensate World Orient=True; FK Rotation Offset=identity; Rotation Order=ZXY (bindable); FK Mirror Behavior=True; Use Scale=False; Segment Scale Control=True; Primary Axis=(1,0,0); Secondary Axis=(0,1,0); IK FK Auto Matching=True; Debug=False; End Bone Rotation Offset=identity; FK Start/Mid/End and IK Mid Display Names; Default FK Space Index=-1. Shape structs: FK Control Shape (Circle_Thick, rot (0,-.707,0,.707), translation (8,...)), IK Control Shape (Box_Thick), PV Control Shape (Diamond_Solid), plus an FK Control Scale Factor Profile curve (flat 1). Biped template legs: IK Rotation Offset=(-.707,0,.707,0) on L and (0,.707,0,.707) on R; PV Twist Follow=1; IK End Align=False; FK Rotation Offset=(-1,0,0,0); End Bone Rotation Offset=(0,-.707,.707,0); Rotation Order=XYZ; Leg R Primary Axis=(-1,0,0). Arms: Default IK=False, Default FK Space Index=0, Arm R IK Rotation Offset=(-1,0,0,0). Zebra adds PV Distance Scale 1.25 (leg) and 2.0 (arm), plus IK and PV shape scale overrides. Rotation Order is bound to rig variables Arm_L_RotateOrder and Arm_R_RotateOrder.

**Setup.** Module settings panel groups: IK Options, FK Options, Module Options, Control Shapes Options, Display Name Options.

**Operators:** `ModularRig module config / Bindings`

**Scale:** about 25 public options

**Evidence:** `<dump>/.../CRM_FN_IkFk2Bones/summary.json variables`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/asset.t3d:519-531 'Begin Object Name="Leg L"' IK Rotation Offset / PV Twist Follow=1.000000 / IK End Align=False`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/asset.t3d 'PV Distance Scale=2.000000'`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/modular_rig_model.txt Bindings=(("Rotation Order", "Arm_L_RotateOrder"))`

#### UE3-ikfk-connectors — IkFk2Bones connector set and auto Parent default

*core* · assets: CRM_FN_IkFk2Bones

Exposed connectors: Start (primary), Mid and End (Secondary, rule RigChildOfPrimaryConnectionRule), Parent (Secondary, optional), IK Spaces and FK Spaces (Secondary, optional, array). Construction sets Bones=[resolve(Start), resolve(Mid), resolve(End)], IK Spaces and FK Spaces from ResolveArrayConnector, and Parent = default parent of Bones[0]. The Connector event runs when the candidate connector is 'Parent': it calls SetDefaultMatch(parent of resolved Start), so an arm defaults to clavicle_l and a leg to pelvis. Zebra wiring: Leg L Start/Mid/End = thigh_l/calf_l/foot_l; Arm L IK Spaces has 9 targets (clavicle_l, spine_05, pelvis, Root/Local, Body/Body, Root/Global, Prop, Prop Attach 01/02).

**Setup.** The animator sees none of this; it is module wiring only.

**Operators:** `RigUnit_ResolveConnector`, `RigUnit_ResolveArrayConnector`, `RigUnit_ConnectorExecution`, `RigUnit_GetCandidates`, `RigUnit_SetDefaultMatch`, `RigUnit_HierarchyGetParent`

**Scale:** 4 instances per biped rig (Arm L/R, Leg L/R) in 4 modular rigs

**Evidence:** `<dump>/FortniteRigs__Modules__IkSolves__IkFk2Bones__CRM_FN_IkFk2Bones/summary.json rig_module_settings ExposedConnectors`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:1678-1692 'RigVMModel Connector Event Graph' SetDefaultMatch`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:1352-1354 ResolveConnector_1/2/3 -> VariableNode_3.Value`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/modular_rig_model.txt 'Arm L/IK Spaces'`

#### UE3-module-metadata-bus — Parent/child module metadata contract (IK leg to foot/twist)

*core* · assets: CRM_FN_IkFk2Bones, CRM_FN_Foot, CRM_FN_LimbTwist

The leg writes module metadata (namespace Self):
- items: 'IK Null', 'IK Control', 'IK Driver' (construction);
- bools: 'IK Solve' (Pre Forwards Solve), 'Match FK' / 'Match IK' (true inside the match functions), 'Key Controls', 'IsInteracting', 'Sec Controls Visibilty' (forward). Post Forwards Solve clears Match and Key.

The Foot reads the same names from namespace Parent: construction reads IK Null, IK Control, IK Driver, IK Solve (default true); Pre Forwards Solve reads IK Solve; Forwards Solve reads Match FK, Match IK, Key Controls.

The root module supplies (namespace Root): 'Local Control', 'Global Control', 'Global Control Scale', 'Direct Mesh Control', 'Direct Mesh Control Libraries', and the 'Global Left/Right/Center Control Color' colors.

Evaluation order this relies on: parent Pre Forwards Solve, then the Foot's Pre Forwards Solve (which drives the leg effector), then the leg's Forwards Solve, then the Foot's Forwards Solve.

**Setup.** None.

**Operators:** `DISPATCH_RigDispatch_SetModuleMetadata`, `DISPATCH_RigDispatch_GetModuleMetadata`, `RigUnit_PreBeginExecution`, `RigUnit_PostBeginExecution`

**Scale:** all IK-family modules

**Evidence:** `<dump>/.../CRM_FN_Foot/graphs.txt:5,7 'Name=IK Solve; NameSpace=Parent' / COMMENT 'The Foot as a Pre Forward solve that runs before the Leg'`; `<dump>/.../CRM_FN_Foot/graphs.txt:243-244,473,504 construction metadata reads`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:225 'Name=IsInteracting; NameSpace=Self'`; `<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:961-1008 'Global Right Control Color'`

#### UE4-root-global-metadata — Rig-wide settings published as Root-module metadata

*core* · assets: CRM_FN_Root, CRM_FN_DMC, CRFL_Hierarchy_v001, CRFL_Control_v001, CRM_FN_FkChain, CRM_FN_IkFk2Bones, CRM_FN_Spine

This is how module variables become rig-wide settings. During construction, CRM_FN_Root runs SetModuleMetadata(NameSpace=Self) for 'Global Control Scale' (double, default 1.0), 'Global Left/Right/Center Control Color' (LinearColor; defaults blue/red/yellow) and the item keys 'Global Control', 'Local Control' and 'Root Control'. CRM_FN_DMC sets 'Direct Mesh Control' (bool) and 'Direct Mesh Control Libraries' (FName array, e.g. ['ik-layer','fk-layer']) with NameSpace=Root, and only when the corresponding mesh polygroup layers exist. Every other module and the CRFL colour, scale and shape helpers read these values with GetModuleMetadata(NameSpace=Root), with defaults of 1.0, yellow or an empty array when the value is absent.

**Setup.** The animator or TD sets these on the root module: Global Control Scale and the three Global side colours (category 'Control Shapes Options'). Adding the DMC module switches shape lookup to mesh-layer shapes.

**Operators:** `DISPATCH_RigDispatch_SetModuleMetadata`, `DISPATCH_RigDispatch_GetModuleMetadata`, `RigUnit_SetupShapeLibraryFromLayer (DMC)`

**Scale:** Read in essentially every biped module: Set Control Scale Global Scale in 9 modules, and DMC libraries in 5 modules (9 lookups).

**Evidence:** `ue/<dump>/FortniteRigs__Modules__FkSolves__Root__CRM_FN_Root/graphs.txt:79-115 'Set_Module_Metadata ... Name=Global Control Scale / Global Left Control Color / Global Center Control Color / Global Right Control Color'`; `ue/<dump>/FortniteRigs__Modules__FkSolves__Root__CRM_FN_Root/summary.json variables 'Global Center Control Color' DefaultValue (R=1,G=1,B=0), 'Global Left Control Color' (0,0,1), 'Global Right Control Color' (1,0,0)`; `ue/<dump>/FortniteRigs__Modules__Miscellaneous__CRM_FN_DMC/graphs.txt:10 'Name=Direct Mesh Control Libraries; NameSpace=Root'; :12/:19 'SetupShapeLibraryFromLayer LayerName=ik-layer / fk-layer'`; `ue/<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkChain/graphs.txt:135 'Get Module Float Metadata | Name=Global Control Scale; NameSpace=Root; Default=1.000000'`; `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/RigUnitContext.h:35-46 ERigMetaDataNameSpace {None, Self, Parent, Root}`

#### UE4-unit-module-connector — Engine units: module identity and connector resolution

*core* · assets: CRFL_Control_v001, CRFL_Hierarchy_v001, CRFL_Module_v001

RigUnit_GetModuleName returns the calling module's prefix without its trailing separator, and warns outside modules. RigUnit_GetItemShortName returns the UI short display name (ForceShort) and falls back to Item.Name. RigUnit_ItemToName converts an element key to its name. RigUnit_ResolveConnector(Connector, SkipSocket) returns the first resolved target, or the input when unresolved or not a connector; post-construction connectors raise an error during Construction. RigUnit_GetCandidates (connector event only) returns the connector being resolved and its candidate matches. RigUnit_SetDefaultMatch (connector event only) sets the default match.

**Operators:** `RigUnit_GetModuleName`, `RigUnit_GetItemShortName`, `RigUnit_ItemToName`, `RigUnit_ResolveConnector`, `RigUnit_GetCandidates`, `RigUnit_SetDefaultMatch`

**Scale:** GetModuleName 2, ResolveConnector 2, GetCandidates 2, SetDefaultMatch 2, GetItemShortName 1, ItemToName 1.

**Evidence:** `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Execution/RigUnit_RigModules.cpp:10-50 (ResolveConnector), :94-107 (GetItemShortName), :163-180 (GetModuleName)`; `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Modules/RigUnit_ConnectionCandidates.cpp:9-26, :64-86`; `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/Execution/RigUnit_Item.h:218 (ItemToName)`

#### UE6-face-module-shell — Face module shell: connectors, socket, placement in modular rigs

*core* · assets: CRM_Zebra_Face, CRM_Monster_Face, MR_Zebra, MR_ZebraDMC, MR_Monster

The module declares a primary connector Root, a secondary optional connector Parent (connection rule ChildOfPrimary) and a default Root socket. MR_Zebra and MR_ZebraDMC wire Face/Root->bone root and Face/Parent->bone head, with parent module root. MR_Monster parents Face under module Spine and wires only Face/Root->root, so Face/Parent stays unresolved. The Zebra Ear Base L/R and Mohawk FK-array modules use Face as their parent module. The module has no config overrides or variable bindings; the face 'Direct Mesh Control' variable is not bound, even in MR_ZebraDMC.

**Setup.** There are no authored controls; all elements are spawned in Construction and named Face/<name>.

**Operators:** `ModularRig module/connector model`, `RigChildOfPrimaryConnectionRule`

**Scale:** 2 connectors, 1 socket; 1 module instance per rig

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/hierarchy_export.txt (Type=Secondary,bOptional=True,RigChildOfPrimaryConnectionRule)`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/modular_rig_model.txt Connector Face/Parent Targets bone head`; `<dump>/Game__Assets__Monster__Rig__MR_Monster/modular_rig_model.txt Name=Face,ParentModuleName=Spine`

#### UE6-forward-order — Face forward solve ordering and state carry-over

*core* · assets: CRM_Zebra_Face, CRM_Monster_Face

BeginExecution runs, then (Monster only) model_edits=1. Next come the Head Attach Null constraint and the control-to-bone SetTransforms (jaw, skull, skull_tp, teeth, cheeks, nose, tongue, eyes), then the 5 visibility loops and the squeeze-null constraint. The aggregate Sequence then runs A: brow curves, brow helper bones, micro brow curves, squeeze, corner curves, (Zebra) smile lid push, corner-height constraints, lip all/mid/sneer, squetch, (Monster) nose/sticky. B: top/low/corner lip constraints. C: lip rolls, (Monster) puff/ch. D: Correctives. E: New Parent x3 plus the Lips Bt loop. F: jaw reader and Jaw Open Logic. G: lip tweaker constraints and additive offsets. H: blink, extend, open, rotate. I: soft eyes. J: lid skin constraints. K: squash curves, pupil/cornea, eye aim constraints, convergence. L (Monster): *_deformer curves and 9 AddOptimusDeformer. Consequences: (1) Correctives use the Jaw Normalize from the previous evaluation, because F runs after D. (2) eye_l/eye_r bones are written from the Eye controls before the aim constraints update Eye L/R null, which can lag by one evaluation. (3) Pure library functions (Brow Main, Brow Micro, Corner Logic, Expression Shape Logic, Brow Squeeze Logic) have no wired exec pins and are evaluated as data dependencies of the SetCurveValue nodes. The AnimNode ControlRig resets bones, including spawned ones, to initial before input by default (bResetInputPoseToInitial=true), so the AdditiveLocal stacks do not accumulate across frames.

**Setup.** None.

**Operators:** `RigUnit_BeginExecution`, `RigVMFunction_Sequence (aggregate A..L)`

**Scale:** 565 / 670 nodes

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:1879-1913 RigVMFunction_Sequence_2_ContainedGraph (pins A..K)`; `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/regen.py add_link RigVMFunction_Sequence_1.L -> SetCurveValue_39.ExecutePin`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/AnimNode_ControlRigBase.cpp:35 bResetInputPoseToInitial(true)`

#### UE6-member-config-tables — Member-variable configuration and handle caching

*core* · assets: CRM_Zebra_Face, CRM_Monster_Face

The Construction graph stores the key of every spawned control, channel or null in a private RigElementKey member variable (for example 'Corner L', 'Sneer Tp L', 'Lip Roll Ot Tp' = Roll Tp channel, 'Eye Main Micro L' = Micro Vis channel). The forward solve reads the keys back instead of hard-coding names. Array members collect spawned keys: Lid Spawn Tp/Bt (Base) Bones L/R, Lip Control Nulls and Lip Controls. Authored tuning tables are also member defaults, filled by the modular rig with the asset defaults: 13 TArray<FQuat> lid tables (Blink, Base Blink, Extend, Open, Base Open, Rot Pos/Neg for Tp/Bt), 4 Soft Eyes quaternion arrays of 12, 4 TArray<FVector> lid micro shape offsets, and 3 user-struct arrays (Lid Tp Struct 16 rows, Lid Bt Struct 13 rows, Lip Null Struct 20 rows; Monster 17/11/20). The doubles are Jaw Normalize (a runtime state carried between frames) and Smile Open (unused). Zebra also has the bools DMC Found and Direct Mesh Control. Monster adds keys for the 9 deformer nulls, Mouth Squash, Ch/Puff, Nose Sneer/Flare, Squint and Sticky.

**Setup.** None; the variables are private (bPrivate=True) and exposed to cinematics.

**Operators:** `RigVMVariableNode`, `UserDefinedStruct`

**Scale:** 88 / 108 variables

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/summary.json variables (88)`; `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/summary.json variables (108)`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/asset.t3d:1900 PropertyBag Face variables (runtime values)`; `<dump>/Game__Assets__Monster__Rig__MR_Monster/asset.t3d:442 PropertyBag Face variables`

#### UE8-conn-resolution-dead-entries — Connection list resolution: case-insensitive matching, dead entries, legacy mirrors

*core* · assets: MR_Zebra, MR_ZebraDMC, MR_Monster, MR_FN_Biped, MR_FN_BipedDMC, MR_Boombox

FModularRigConnections::FindConnectionIndex does a linear search with FRigElementKey::operator==, which compares Name (FName, case-insensitive) and Type. Connection entries therefore match connectors that differ only in case: 'Root/RootJoint' matches 'root/RootJoint', and 'Root/body' matches 'root/Body'. Entries whose connector no longer exists are kept but never used. Counts (entries / runtime connectors / exact / case-only / dead): MR_Zebra 142/149/120/2/20; MR_ZebraDMC 143/150/121/2/20; MR_Monster 34/26/21/2/11; MR_FN_Biped and MR_FN_BipedDMC 210-211/192-193/170-171/2/38. MR_Zebra's dead entries: Root/Root, Root/WorldOffsetPosition, Root/Right Hip, Root/Left Hip, Foot L/Root, Foot L/End, Foot R/Root, Foot R/End, Thumb L/OrientSpace, Neck/EndParent, Neck/StartParent, Spine/EndParent, Spine/StartParent, Clavicle L/OrientSpace, Clavicle R/OrientSpace, Spine/StartSnapTo, Spine/Start Spaces, Spine/End Spaces, Neck/Start Spaces, Neck/End Spaces. The Biped templates also have dead entries for Meta L/R Bone A-D, finger OrientSpace, Index L/Mid and Attach/Parents. ArrayConnectionMap in asset.t3d is identical to ConnectionList (0 differences in all 5 rigs); the editor rebuilds it from the model connections. PreviousModulePaths maps old path-style module names ('Root:Spine:Clavicle L:Arm L') to current names ('Arm L'). The MR_Zebra map still lists removed modules: Meta L/R, Middle L/R, Ring L/R. Importer rule: compare connector keys case-insensitively, ignore entries whose connector is missing, and ignore ArrayConnectionMap and PreviousModulePaths.

**Setup.** The module connectors are the rig-building UI; stale entries are invisible to the user.

**Operators:** `FModularRigConnections::FindConnectionIndex`, `FRigElementKey::operator==`, `UControlRigEditorAsset (ArrayConnectionMap rebuild)`

**Scale:** 91 dead + 10 case-only entries across 5 rigs

**Evidence:** `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/ModularRigModel.h:284-290`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Rigs/RigHierarchyDefines.h:1848-1851`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRigDeveloper/Private/ControlRigEditorAsset.cpp:1447-1451`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/modular_rig_model.txt :: 'Name="Root/RootJoint"' and 'PreviousModulePaths='`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1726-1727 (CONNECTOR root/RootJoint, root/Body)`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/asset.t3d :: 'ArrayConnectionMap='`

#### UE-monster-deformers-in-face-module — Monster adds its 9 deformers from the animator face module, not post-process

*important* · assets: CRM_Monster_Face, CR_Monster_Deform, MR_Monster

For Monster, the Add Deformer nodes live in the face module's forward solve (CRM_Monster_Face, inside MR_Monster), chained after SetCurveValue_76. The order is:
1. Head (stretch default 0.5, pivot z80)
2. HeadBend (bend default 0.5, rotZ90 at z90)
3. HeadTwist (z20)
4. MuzzleBend (quat(-0.707,-0.707,0,0) at (0,20,110))
5. MuzzleSquash (z80)
6. MouthBend
7. MouthSquash
8. SkullTpBend
9. SkullTpSquash
All use After/group 1/child components. Factors come from GetCurveValue, some via Remap, and transforms from GetTransform. CR_Monster_Deform, the post-process rig, adds none. As a result, Monster deformers only run while the animator modular rig is evaluated (editor or sequencer). Zebra's deformers run in post-process for any animation source.

**Setup.** The Monster face module owns the deformer curves (head_bend_deformer, head_squash_deformer, head_twist_deformer, mouth_bend_deformer, mouth_squash_deformer, muzzle_*_deformer, skull_tp_*_deformer are in CR_Monster_Deform's curve list).

**Operators:** `RigUnit_AddOptimusDeformer`, `RigUnit_GetCurveValue`, `RigUnit_GetTransform`

**Scale:** 9 deformers

**Evidence:** `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/graphs.txt:1265-1295 (AddOptimusDeformer_3/_2/_4/_5/_6/AddOptimusDeformer/_1/_7/_8)`; `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/graphs.txt:2009-2051 (exec chain SetCurveValue_76 -> AddOptimusDeformer_3 -> _2 -> _4 -> AddOptimusDeformer -> _1 -> _7 -> _8 -> _5 -> _6)`; `<dump>/Game__Assets__Monster__Rig__CR_Monster_Deform/summary.json units_used (no AddOptimusDeformer)`

#### UE1-array-connector-semantics — Array connectors: ordered multi-target, index pairing, silent drop of missing targets

*important* · assets: MR_FN_Biped, MR_Monster, MR_Zebra

An array connector stores an ordered Targets[] list. Modules read it with RigUnit_ResolveArrayConnector. Modules use ordering in these ways:
- Pin: Drivers[i] drives Driven[i] (hand_r->ik_hand_gun, hand_l->ik_hand_l, foot_l->ik_foot_l, foot_r->ik_foot_r, hand_r->ik_hand_r).
- FkArray: Override Parents[i] applies to Bones[i]. Attach bones [attach, attach_cape, attach_backpack, ...] get override parents [root/Local, spine_05, spine_05]; the rest use their skeletal parent.
- Space lists: order equals the order of the space-switch menu.

Targets that don't exist in the rig are dropped silently. In MR_Monster, Prop/Spaces [hand_r, hand_l, spine_05] yields only the spine_05 space because the hand bones are absent. The same list can contain duplicates (Pin Drivers has hand_r twice).

**Operators:** `RigUnit_ResolveArrayConnector`, `FModularRigSingleConnection.Targets`

**Evidence:** `<dump>/FortniteRigs__Templates__MR_FN_Biped/modular_rig_model.txt: 'IK Bone Pins/Drivers' 'IK Bone Pins/Driven'`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/modular_rig_model.txt: 'Attach/Override Parents'`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/runtime_hierarchy.txt: 'NULL Attach/Attach Default Space parents=['CONTROL:root/Local']' and 'NULL Attach/Attach Cape Default Space parents=['BONE:spine_05']'`; `<dump>/Game__Assets__Monster__Rig__MR_Monster/runtime_hierarchy.txt:1389 'CONTROL Prop/Prop' customization=(AvailableSpaces=((Key=(Type=Bone,Name="spine_05")))`

#### UE1-connector-event-default-match — Module 'Connector' event: default match / auto-suggest

*important* · assets: MR_Zebra, MR_FN_Biped

Modules can implement a 'Connector' event (RigUnit_ConnectorExecution) that runs during candidate resolution. It uses RigUnit_GetCandidates to learn which connector is being resolved and RigUnit_SetDefaultMatch to propose a default target.
- CRM_FN_FkChain: 'End' defaults to the last recursive bone child of Start (ArrayGetAtIndex Index=-1) when children exist; 'Parent' defaults to the default parent of Start.
- CRM_FN_IkFk2Bones: when the connector is 'Parent', the default is the parent of the resolved Start.
- CRM_FN_LimbTwist: 'Parent' and 'Start ...' default to the resolved Start; another connector defaults to Start's second direct child (At index 1).
- CRM_FN_FkArray: has a stub Connector graph.

Library helpers:
- CRFL_Module 'Set Default Match To Connector v01' (Connector, Default Match): if GetCandidates.Connector == Connector, then SetDefaultMatch(Default).
- 'Connect to Module Metadata' (Connector, Module Metadata Name, NameSpace): for each candidate, if GetModuleMetadata(Name, NameSpace) is found and equals the candidate, set it as the default match.

**Operators:** `RigUnit_ConnectorExecution`, `RigUnit_GetCandidates`, `RigUnit_SetDefaultMatch`, `RigUnit_ResolveConnector`, `RigUnit_CollectionChildrenArray`, `RigUnit_HierarchyGetParent`, `FUNC Set Default Match To Connector v01 @ CRFL_Module_v001`, `FUNC Connect to Module Metadata @ CRFL_Module_v001`

**Scale:** 4 module classes. RigUnit_ConnectorExecution total 4.

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkChain/graphs.txt:361 '### GRAPH ... RigVMModel Connector Graph'`; `<dump>/FortniteRigs__Modules__IkSolves__IkFk2Bones__CRM_FN_IkFk2Bones/graphs.txt:1678 'Connector Event Graph' ... 'B=(Type=Connector,Name="Parent")'`; `<dump>/FortniteRigs__Modules__Biped__LimbTwist__CRM_FN_LimbTwist/graphs.txt:286`; `<dump>/FortniteRigs__Libraries__CRFL_Module_v001/graphs.txt:361 'Set Default Match To Connector v01'`; `<dump>/FortniteRigs__Libraries__CRFL_Module_v001/graphs.txt:183 'Connect to Module Metadata'`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/ModularRig.cpp:1365-1404 (ExecuteConnectorEvent)`

#### UE1-module-mirroring — MirrorModule (editor) and mirrored L/R configuration

*important* · assets: MR_Zebra, MR_FN_Biped

UModularRigController::MirrorModule(Module, FRigVMMirrorSettings{MirrorAxis, AxisToFlip, SearchString, ReplaceString}):
- creates a new module of the same class under the same parent, with the name search-replaced (case-sensitive) and made unique
- re-creates every connection with the target names search-replaced (e.g. _l -> _r)
- search-replaces binding source paths
- for each public, instance-editable FVector or FTransform config variable, writes the mirrored value (MirrorVector/MirrorTransform) of the original override or class default
- copies the other config values

The R-side modules in these rigs show such sign-flipped values: Foot R Inner Bank (5, 0, -3) vs L (-5, 0, -3); Clavicle R shape translation (-12, 0, -8) vs (12, 0, 8); Twist shape Z +5 vs -5.

Separately, controls carry per-control mirroring metadata ('Mirror Behavioral' BOOL, 'Mirror Axis' VECTOR) set by CRFL_Control 'Set Mirror Axis/Behavior'. Pose mirroring tools read it.

**Setup.** FK/finger/FkArray/twist controls carry Mirror Behavioral and Mirror Axis metadata. The FkArray and FkChain default Mirror Axis is (0, 1, 1).

**Operators:** `UModularRigController::MirrorModule`, `FRigVMMirrorSettings`, `FUNC Set Mirror Axis @ CRFL_Control_v001`

**Evidence:** `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/ModularRigController.cpp:1689-1816`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/asset.t3d: Foot L 'Inner Bank Offset=(X=-5.000000,Y=0.000000,Z=-3.000000)' vs Foot R 'Inner Bank Offset=(X=5.000000'`; `<dump>/FortniteRigs__Libraries__CRFL_Control_v001/graphs.txt:29 'Name=Mirror Axis'`; `<dump>/FortniteRigs__Libraries__CRFL_Control_v001/graphs.txt:40 'Name=Mirror Behavioral'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:822 'Mirror Behavioral:BOOL', 'Mirror Axis:VECTOR'`

#### UE1-rule-manager-matching — Connector candidate filtering pipeline (rule manager)

*important* · assets: MR_Zebra, MR_FN_Biped, MR_Monster, MR_Boombox

FindMatches(connector, module, resolvedConnectors):
1. If construction is required, run PrepareForExecution first.
2. Start from every hierarchy element as a PossibleTarget.
3. FilterIncompatibleTypes: reject Curves and Connectors.
4. FilterInvalidModules: reject elements in the connector's own module namespace. Reject elements spawned at or after the module's ConstructionSpawnStartIndex (PostConstructionSpawnStartIndex for post-construction connectors); if no spawn index is known, reject elements of child modules instead. Result: a module can only connect to things that exist before it (skeleton, imported nulls/sockets, or elements of earlier modules).
5. FilterByConnectorRules: apply each rule in sequence.
6. FilterByConnectorEvent: run the module's 'Connector' event, which can prune candidates or mark a DefaultTarget, and move the default match to the front.

If no matches remain, the state is Error; otherwise Success. Excluded results keep their reason text.

**Operators:** `UModularRigRuleManager::FindMatches`, `FilterIncompatibleTypes`, `FilterInvalidModules`, `FilterByConnectorRules`, `FilterByConnectorEvent`, `FModularRigResolveResult`, `FRigElementResolveResult`

**Scale:** Editor/connect time only. Every connector of every module.

**Evidence:** `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/ModularRigRuleManager.cpp:11-58`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/ModularRigRuleManager.cpp:174-189`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/ModularRigRuleManager.cpp:208-256 (namespace and spawn-index filtering)`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/ModularRigRuleManager.cpp:288-327`

#### UE1-variable-bindings — Module variable bindings to host-rig (or other module) variables

*important* · assets: MR_Zebra, MR_ZebraDMC, MR_FN_Biped, MR_FN_BipedDMC

FRigModuleReference.Bindings maps a module public variable name to a source path. The path is either '<HostVar>' or '<Module>.<Var>'; FRigHierarchyModulePath.Split separates the module part. SetModuleVariableBindings resolves the source FRigVMExternalVariable. Before each module execution, UpdateModuleVariables copies the source value into the module variable when the types are compatible. Bound values therefore override config values on every run.

Zebra/Biped: Arm L 'Rotation Order' <- 'Arm_L_RotateOrder' and Arm R 'Rotation Order' <- 'Arm_R_RotateOrder'. Both are host EEulerRotationOrder variables with default XYZ, bPublic, not exposed to cinematics. The runtime Arm FK controls show preferred_rotation_order=XYZ even though the Arm L config says XZY and the Arm R class default is ZXY.

DMC variants: CRM_FN_DMC 'Direct_Mesh_Control' <- host bool 'Direct_Mesh_Control' (True).

**Setup.** The rig-level variable acts as a user-facing setting that feeds per-module parameters.

**Operators:** `UModularRig::SetModuleVariableBindings`, `UModularRig::UpdateModuleVariables`, `UModularRigController::BindModuleVariable`

**Scale:** 2 bindings in Zebra and Biped. 3 in the DMC variants.

**Evidence:** `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/ModularRig.cpp:1089-1142`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/ModularRig.cpp:726-727 ('Copy variable bindings')`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/modular_rig_model.txt: 'Bindings=(("Rotation Order", "Arm_L_RotateOrder"))'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/summary.json: 'Name=\"Arm_L_RotateOrder\",CPPType=\"EEulerRotationOrder\"...DefaultValue=\"XYZ\"'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:855 'CONTROL Arm L/FK 0 ... preferred_rotation_order=XYZ'`; `<dump>/Game__Assets__Zebra__Rig__MR_ZebraDMC/modular_rig_model.txt: 'Bindings=(("Direct_Mesh_Control", "Direct_Mesh_Control"))'`

#### UE1-zebra-vs-monster — Difference MR_Zebra vs MR_Monster

*important* · assets: MR_Zebra, MR_Monster

Monster is a head/bust rig built with the same Fortnite modules and identical config values for root, Body, Spine and Clavicles (values copied from Zebra). It has no Neck, Arms, Legs, Feet, LimbTwist, fingers, Tweakers, ears or mohawk. Its skeleton has no limbs; neck_01, neck_02 and head exist but get no Neck module.

Face (CRM_Monster_Face) is parented under module Spine instead of root, and its Parent connector is unconnected. Prop/Prop keeps only the spine_05 space. Monster has no mesh sockets, so no foot nulls, and 993 curves.

Body controls 57 (root 8, Prop 17, Body 9, Spine 21, Clavicles 2) versus Zebra 252. Face controls 120 versus 107. Monster still declares the unused Arm_L/R_RotateOrder variables.

**Evidence:** `<dump>/Game__Assets__Monster__Rig__MR_Monster/modular_rig_model.txt (7 modules)`; `<dump>/Game__Assets__Monster__Rig__MR_Monster/hierarchy.txt: 'BONE neck_01' 'BONE head' (165 bones, no arm/leg bones)`; `<dump>/Game__Assets__Monster__Rig__MR_Monster/asset.t3d: Spine 'FK Primary Display Names(0)="Pelvis FK"' (same as Zebra)`; `<dump>/Game__Assets__Monster__Rig__MR_Monster/summary.json: 'variables' Arm_L_RotateOrder, Arm_R_RotateOrder`

#### UE1-zebra-vs-zebradmc — Difference MR_Zebra vs MR_ZebraDMC

*important* · assets: MR_Zebra, MR_ZebraDMC

The two rigs are identical except for:
1. The extra module CRM_FN_DMC (parent root, connector CRM_FN_DMC/Root -> root, binding Direct_Mesh_Control).
2. The extra host bool variable Direct_Mesh_Control (default True).
3. Static connector count 150 vs 149; ProceduralElementLimit 3487 vs 3486.
4. Mohawk 'FK Control Shape' config lacks Name='DefaultGizmoLibraryNormalized.Circle_Thick', so its runtime shape is 'Default'.
5. Extra DMC metadata on root/RootJoint.

Control, null and bone counts are identical (359/326/423). Module order in the model list differs, but tree membership does not.

**Operators:** `CRM_FN_DMC`

**Evidence:** `diff of <dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt vs <dump>/Game__Assets__Zebra__Rig__MR_ZebraDMC/runtime_hierarchy.txt: lines 1355, 1359, 1726, 1728 only`; `<dump>/Game__Assets__Zebra__Rig__MR_ZebraDMC/summary.json: 'Name=\"Direct_Mesh_Control\",CPPType=\"bool\"...DefaultValue=\"True\"'`; `<dump>/Game__Assets__Zebra__Rig__MR_ZebraDMC/asset.t3d: 'HierarchySettings=(ProceduralElementLimit=3487)'`

#### UE2-root-published-metadata — Rig-wide settings published as Root module metadata

*important* · assets: CRM_FN_Root, CRM_FN_Body, CRM_FN_Spine, CRM_FN_FkChain, CRM_FN_FkArray, CRM_FN_Prop

Root construction calls SetModuleMetadata (NameSpace=Self) for:
- 'Global Control', 'Root Control', 'Local Control' (element keys)
- 'Global Control Scale' (float)
- 'Global Left/Right/Center Control Color' (linear color)

Other modules read these with GetModuleMetadata NameSpace=Root:
- Global Control Scale: Body, FkChain, FkArray, Prop, Spine.
- Root Control: Spine, which adds a space to it.
- Colors: the Get Control Color From Metadata functions.

The Body module writes 'Body Control' into the Root namespace. The metadata is physically stored on the root/RootJoint connector (runtime metadata list). Root also exposes 'Direct Mesh Control' (bool) and 'Direct Mesh Control Libraries' (name array) keys, which the FK modules read (NameSpace=Root) to switch to DMC shape libraries.

**Setup.** Public Root config variables drive these: Global Control Scale=1; colors Center=yellow, Left=blue, Right=red.

**Operators:** `DISPATCH_RigDispatch_SetModuleMetadata`, `DISPATCH_RigDispatch_GetModuleMetadata`

**Scale:** 7 keys written by Root, 1 by Body; read by all FK-family modules

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Root__CRM_FN_Root/graphs.txt:79-81,90,110,113,115 'Set Module ... Metadata | Name=...; NameSpace=Self'`; `<dump>/FortniteRigs__Modules__FkSolves__Body__CRM_FN_Body/graphs.txt:203 'Name=Body Control; NameSpace=Root'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt 'CONNECTOR root/RootJoint ... Global Control Scale:FLOAT ... Body Control:RIG_ELEMENT_KEY'`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:673 'Get Module Item Metadata' (Root Control)`

#### UE2-spine-neck-mode — Is Neck configuration switch (one module asset for spine and neck)

*important* · assets: CRM_FN_Spine, MR_FN_Biped, MR_Zebra

The public bool 'Is Neck' (default false) changes the module's behavior.

When false (spine), the module also:
- creates End Movable Pivot and runs Forward Movable Proxy;
- creates Pelvis Local and the Pelvis TXY root space;
- in forwards, writes the pelvis from Pelvis Local.

When true (neck):
- the first bone's spline rotation uses the interpolated spline rotation instead of the Start IK rotation;
- the visibility IK set also includes Start IK.

Runtime Neck instances have no 'Pelvis Local', 'End Movable Pivot' or 'Movable Pivot Vis', which shows that Is Neck is overridden to true for the 'Neck' module.

**Setup.** Config category 'Modules Options'.

**Operators:** `RigVMFunction_MathBoolNot`, `RigVMFunction_ControlFlowBranch`, `DISPATCH_RigVMDispatch_If`

**Scale:** 2 instances per biped (Spine + Neck)

**Evidence:** `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/summary.json 'Name=\"Is Neck\"'`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:34,72,388,628,1722,2042 'VAR Is Neck'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt (Neck: 9 controls, no proxy; Spine: 13 controls + PROXY_CONTROL)`

#### UE3-negative-side-detect — Automatic negative-side (mirrored chain) detection

*important* · assets: CRM_FN_IkFk2Bones

Construction computes Negative Side = dot(VB[1].t - VB[0].t, VB[0].rot * (1,0,0)) < 0, using current global transforms after the virtual bones are created. It stores the result in the private variable 'Negative Side'. That flag then drives:
- Soft IK primary axis X = NegSide ? -1 : 1, and secondary axis Y = NegSide ? +1 : -1.
- Compute FK aim direction negation.
- Pole-vector parent offset sign.
- FK shape mirror: shape transform * Scale(-1,1,1).
- Mid control shape: rotation (.707,0,-.707,0) vs (0,-.707,0,.707), and translation Y +6 vs -6.

**Setup.** Nothing to set; the side is inferred from bone orientation.

**Operators:** `RigVMFunction_MathVectorDot`, `RigVMFunction_MathTransformRotateVector`, `RigVMFunction_MathDoubleLess`, `DISPATCH_RigVMDispatch_If`

**Scale:** all 4 limbs

**Evidence:** `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:1004-1014 'Calculate Negative Side Value' + links 1526-1538`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:19 'If | True=-1.000000; False=1.000000' + 393-394 Soft IK.PrimaryAxis.X / SecondaryAxis.Y`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:661 DISPATCH_RigVMDispatch_If_1 and 816 If_1 (Mid shape)`

#### UE4-default-match-to-connector — Set Default Match To Connector v01 (connector auto-resolve defaults)

*important* · assets: CRFL_Module_v001, CRM_FN_LimbTwist, CRM_FN_FkChain

Valid only in the connector-resolution event. Inputs: Connector and Default Match. If GetCandidates().Connector == Connector (the connector currently being resolved), it calls SetDefaultMatch(Default Match). The engine then demotes the previous default to 'possible target' and marks the given element as the default, which must already be among the matches. Modules chain several calls, one per connector. For example, LimbTwist sets 'Parent' and 'Start Socket' to ResolveConnector results and sets an array element's default to connector 'Clavicle'. FkChain sets 'End' to an array element and 'Parent' to a hierarchy parent.

**Setup.** When the module is dropped onto a rig, its connectors auto-suggest these targets.

**Operators:** `RigUnit_GetCandidates`, `RigUnit_SetDefaultMatch`, `DISPATCH_RigVMDispatch_CoreEquals`, `RigVMFunction_ControlFlowBranch`

**Scale:** 5 call sites (LimbTwist 3, FkChain 2).

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Module_v001/graphs.txt:361-376 'Set Default Match To Connector v01'`; `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Modules/RigUnit_ConnectionCandidates.cpp:9-26 (GetCandidates only in the ConnectorExecution event) and :64-86 (SetDefaultMatch)`; `ue/<dump>/FortniteRigs__Modules__Biped__LimbTwist__CRM_FN_LimbTwist/graphs.txt:289-294 ('RigVMModel Connector Event Graph', Connector=Parent / Start Socket)`; `ue/<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkChain/graphs.txt:371-372 (Connector=End / Parent)`

#### UE6-zebra-vs-monster — Zebra vs Monster face differences (fork deltas)

*important* · assets: CRM_Zebra_Face, CRM_Monster_Face

Monster changes relative to Zebra. Layout is rescaled for the Monster head: brows at z=68-70, y=30; squash offsets at 100; Muzzle Squash at T(0,28,20); lid micro shapes (7,x,x), scale 0.03. Mouth is re-parented to the jaw bone; Lips Tp is under the muzzle bone; Cheeks are under the skull bone; Nose is Sphere_Solid at the nose bone transform with Sneer/Flare channels. Monster adds Squint (unused), Sticky -> lip_stick, Mouth Squash, Puff/Ch channels and functions, 9 Optimus deformers with nulls, the extra lid bt/extend curves, frown masking in the Low/Corner lip constraints, 2 extra jaw-open lip offsets, stronger or different Lip Roll poses, and fixed R lid bt constraint parents. Eye aim uses a location world-up. Correctives use open_frown_c instead of smile_open_c, and model_edits=1. Lid chains have 15 top / 11 bottom skin bones (Zebra 14/13) with retuned struct weights and larger Soft Eyes Dn/Up. Monster removes the DMC shape switching, the smile lid push and the orphan Eye Aim OnOff read. Monster keeps the Zebra lip_roll curve names, which have no Monster morphs.

**Setup.** See the individual features.

**Operators:** `(structural diff)`

**Scale:** Monster: +39/-28 construction nodes, +110/-5 forward nodes

**Evidence:** `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/summary.json local_functions (26) vs Zebra (23)`; `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/graphs.txt:272 SpawnControl_12 Mouth Parent=Jaw`; `<dump>/Game__Assets__Monster__Rig__MR_Monster/asset.t3d:442 Lid_Tp_Struct (17 rows) Lid_Bt_Struct (11 rows)`

#### UE7-dmc-module-consumers — Fortnite modules consuming DMC metadata (ik-layer / fk-layer per bone)

*important* · assets: CRM_FN_IkFk2Bones, CRM_FN_FkChain, CRM_FN_FkArray, CRM_FN_Spine, CRM_FN_Foot

During construction, each module reads GetModuleMetadata('Direct Mesh Control Libraries', Root) and feeds it to 'Get Control Shape Name From Item v02' with a fixed namespace per control.

Per-module mapping:
- IkFk2Bones:
  - IK control -> 'ik-layer.<Bones[-1]>' (default: IK Control Shape)
  - IK Base -> 'ik-layer.<Bones[0]>'
  - PV (vector control) -> 'ik-layer.<Bones[1]>' (default: PV Control Shape)
  - FK controls -> 'fk-layer.<Bones[i]>' (default: FK Control Shape)
  - Also sets 'DMC Found' = Find(libs, 'ik-layer'). In Forwards Solve, if metadata 'Direct Mesh Control' and DMC Found are both true, it skips the grey Mid-control-to-PV debug line.
- FkChain and FkArray: if metadata 'Direct Mesh Control' (default False) is true, set var 'CRSL Namespace' = 'fk-layer'. That var is also the namespace for SetupShapeLibraryFromUserData. Their local 'Get Control Shape' function returns shape Name = the DMC-aware name, visibility from input, and Transform = ShapeTransform * inverse(ControlOffset).
- Spine: FK controls -> 'fk-layer.<bone>'.
- Foot: 'Toes IK' -> 'ik-layer.<Ball Joint>' and 'Toes FK' -> 'fk-layer.<Ball Joint>'.

FkArray also contains a forward-graph Branch with SetupShapeLibraryFromLayer('ik-layer'/'fk-layer') that has no exec input, so it never runs.

**Setup.** These are the animator-facing DMC controls. The IK hand/foot, IK base, PV and FK controls, spine FK controls and toe controls appear as patches of the mesh surface, grouped by dominant bone.

**Operators:** `DISPATCH_RigDispatch_GetModuleMetadata`, `FUNC Get Control Shape Name From Item v02`, `RigUnit_HierarchyAddControlTransform / AddControlVector (Settings.Shape.Name)`, `RigUnit_SetupShapeLibraryFromUserData`

**Scale:** 5 module types; in the templates this covers all arms, legs, feet, spine, neck and FK chains.

**Evidence:** `<dump>/FortniteRigs__Modules__IkSolves__IkFk2Bones__CRM_FN_IkFk2Bones/graphs.txt:1059-1083,1624-1672 (At_14 Index=-1, At_17 Index=0, At_5 Index=1; Find Element=ik-layer; ... -> SpawnControl_*.Settings.Shape.Name)`; `<dump>/FortniteRigs__Modules__IkSolves__IkFk2Bones__CRM_FN_IkFk2Bones/regen.py:4073-4095 (ShapeLib Namespace ik-layer x3, fk-layer x1)`; `<dump>/FortniteRigs__Modules__IkSolves__IkFk2Bones__CRM_FN_IkFk2Bones/graphs.txt:291-294,597-602 (Branch_9 on DMC && DMC Found -> DebugLineNoSpace on False)`; `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkChain/graphs.txt:191-196,307-313,420-440`; `<dump>/FortniteRigs__Modules__Biped__Foot__CRM_FN_Foot/graphs.txt:360-363,547-549,993-1000; regen.py:1588,1593`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/regen.py:3322`; `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkArray/graphs.txt:51-54,108-110`

#### UE7-dmc-module-crm-fn-dmc — CRM_FN_DMC: enable DMC layers and publish module metadata

*important* · assets: CRM_FN_DMC, MR_ZebraDMC, MR_FN_BipedDMC

Construction:
1. If 'Direct Mesh Control' is true: Sequence A runs SetupShapeLibraryFromLayer('ik-layer') and, if its GroupNames are non-empty, adds 'ik-layer' to the 'Direct Mesh Control Libraries' array. Sequence B does the same for 'fk-layer'.
2. On Completed, SetModuleMetadata(Name='Direct Mesh Control Libraries', NameSpace=Root, Value=array).
3. SetModuleMetadata(Name='Direct Mesh Control', NameSpace=Root, Value=<bool var>).

Forwards solve: empty.

The Root namespace makes the metadata readable by every module in the modular rig. The module must be added before the modules that consume it; its description says 'directly below the CRM_FN_Root module'.

Variables: 'Direct Mesh Control' (bool, public, default True), 'Direct Mesh Control Libraries' (TArray<FName>), and 'NewVar' (unused).

**Setup.** Module option 'Direct Mesh Control' (bool). The DMC templates bind it to the rig variable Direct_Mesh_Control (True). The module has one connector, 'Root', which the templates connect to bone root.

**Operators:** `RigUnit_SetupShapeLibraryFromLayer`, `DISPATCH_RigDispatch_SetModuleMetadata`, `DISPATCH_RigVMDispatch_ArrayIsEmpty`, `DISPATCH_RigVMDispatch_ArrayAdd`, `RigVMFunction_ControlFlowBranch`, `RigVMFunction_Sequence`

**Scale:** 1 module in each of the 2 DMC templates.

**Evidence:** `<dump>/FortniteRigs__Modules__Miscellaneous__CRM_FN_DMC/graphs.txt:4-54`; `<dump>/FortniteRigs__Modules__Miscellaneous__CRM_FN_DMC/regen.py:17-18,52-54 (Set_Module_Metadata_5.Name 'Direct Mesh Control Libraries', NameSpace 'Root'; Set_Module_Metadata_4 'Direct Mesh Control')`; `<dump>/FortniteRigs__Modules__Miscellaneous__CRM_FN_DMC/asset.t3d:1521 (RigModuleSettings Description 'EXPERIMENTAL ... MUST HAVE DIRECT MESH CONTROL PLUGIN ENABLED')`; `<dump>/FortniteRigs__Modules__Miscellaneous__CRM_FN_DMC/summary.json variables`

#### UE7-dmc-template-variants — DMC template variants (MR_ZebraDMC, MR_FN_BipedDMC)

*important* · assets: MR_ZebraDMC, MR_FN_BipedDMC, MR_Zebra, MR_FN_Biped

Diffing the module lists shows the only structural change is one extra module:
(Name='CRM_FN_DMC', ParentModuleName='root', ControlRigAsset=/FortniteRigs/Modules/Miscellaneous/CRM_FN_DMC, Bindings=(("Direct_Mesh_Control","Direct_Mesh_Control")))
Its connection is CRM_FN_DMC/Root -> bone root.

Other changes:
- A rig variable Direct_Mesh_Control (bool, True).
- ModularRigGizmoLibrary_DMC is added to ShapeLibraries. MR_ZebraDMC lists [DMC, Regular]; MR_FN_BipedDMC lists [Regular, DMC]. Shape lookup searches later libraries first, so for unqualified shape names the Biped template prefers the DMC-material gizmos and Zebra prefers the regular ones.

Control counts are unchanged (359 and 283). The only per-control shape difference in the headless dump is that Mohawk Bk/Fr show 'Default' instead of 'DefaultGizmoLibraryNormalized.Circle_Thick'. No control resolved to an ik/fk-layer shape in the headless dump.

**Setup.** Template-level toggle: Direct_Mesh_Control (bool).

**Operators:** `ModularRig module list`, `ControlRigBlueprint ShapeLibraries`, `module variable bindings`

**Scale:** 2 templates (35 and 49 modules).

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_ZebraDMC/modular_rig_model.txt: '(Name="CRM_FN_DMC",ParentModuleName="root",...Bindings=(("Direct_Mesh_Control", "Direct_Mesh_Control")))' and 'Name="CRM_FN_DMC/Root"),Targets=((Type=Bone,Name="root")'`; `<dump>/Game__Assets__Zebra__Rig__MR_ZebraDMC/asset.t3d:2129-2130 (ShapeLibraries order)`; `<dump>/FortniteRigs__Templates__MR_FN_BipedDMC/asset.t3d:2765-2766`; `<dump>/Game__Assets__Zebra__Rig__MR_ZebraDMC/regen.py:1641 add_member_variable('Direct_Mesh_Control', 'bool', True, False)`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/ControlRigGizmoLibrary.cpp:139-191 (backwards library search, two-pass namespace)`; `<dump>/Game__Assets__Zebra__Rig__MR_ZebraDMC/summary.json runtime_hierarchy_counts CONTROL 359`

#### UE7-monster-face-inline-chain — Monster: deformers added inside the face module of the modular rig

*important* · assets: CRM_Monster_Face, MR_Monster, CR_Monster_Deform

In MR_Monster the module 'Face' (CRM_Monster_Face, parent module Spine) adds all 9 deformers inside its own Forwards Solve. The solve first computes the *_deformer curves from the controls (SetCurveValue_39 -> 40 -> 41 -> 73 -> 74 -> 78 -> 79 -> 75 -> 76), then runs the AddOptimusDeformer chain.

Curve-to-trait remaps:
- head_squash_deformer -> Remap_19 (-1..1 -> 0..1) -> StretchFactor
- muzzle_squash_deformer -> Remap_18 (1..-1 -> 0..1)
- mouth_squash_deformer -> Remap_21 (1..-1 -> 0..1)
- skull_tp_squash_deformer -> Remap_20 (-1..1 -> 0..1)
- The bend and twist curves pass through directly.

The Monster post-process rig, CR_Monster_Deform, adds no deformers. The Monster deformers therefore only run while the modular control rig evaluates (editor, Sequencer), unlike Zebra's post-process path.

**Setup.** See UE7-deformer-control-channel-mapping. Monster adds a Mouth Squash control that Zebra does not have.

**Operators:** `RigUnit_AddOptimusDeformer`, `RigUnit_SetCurveValue`, `RigUnit_GetCurveValue`, `RigVMFunction_MathDoubleRemap`

**Scale:** 1 module, 9 deformers, 9 curves.

**Evidence:** `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/graphs.txt:1238-1303 (SetCurveValue/GetCurveValue/Remap nodes)`; `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/graphs.txt:1265-1295 (9 AddOptimusDeformer)`; `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/graphs.txt:2009-2051`; `<dump>/Game__Assets__Monster__Rig__MR_Monster/modular_rig_model.txt '(Name="Face",ParentModuleName="Spine",...CRM_Monster_Face)'`; `grep 'AddOptimusDeformer' finds no hit in <dump>/Game__Assets__Monster__Rig__CR_Monster_Deform/*`

#### UE8-asset-supported-events — Saved SupportedEventNames vs. runtime event union (layered-rig picker, bake)

*important* · assets: MR_Zebra, MR_ZebraDMC, MR_Monster, MR_FN_Biped, MR_FN_BipedDMC, MR_Boombox, CRM_Zebra_Face, CRM_Monster_Face

At runtime, UModularRig::GetSupportedEvents rebuilds SupportedEvents from its modules when the list is empty, and UControlRigRuntimeAsset::UpdateSupportedEventNames uses ModularRigModel.GetSupportedEvents(). Module events, taken from the event graphs: Foot (Construction, Forwards, Pre Forwards, Backwards, Key Controls); IkFk2Bones (adds Post Forwards, Connector, To IK, To FK); LimbTwist, FkChain and FkArray (Connector, Backwards); Spine, Body, Root and ProxyControl (Backwards); Prop, DMC, Pin, StretchFeedback and both Face modules (Construction and Forwards only). The union for MR_Zebra is the 9 events saved on MR_FN_Biped and MR_ZebraDMC, but MR_Zebra's saved list is only [Construction, Forwards Solve], which is stale. MR_Monster's saved list [Forwards, Construction, Backwards, Connector] equals its real union (it has no IkFk2Bones or Foot module), so it is not stale. MR_Boombox saves no list, and its modules are unknown. Effects: (1) The Sequencer layered-rig picker reads the asset-registry tag SupportedEventNames and rejects an asset whose tag exists without 'Backwards Solve' or 'Inverse', so MR_Zebra is hidden there. (2) LoadAnimSequenceIntoThisSection (bake to control rig) needs ControlRig->SupportsEvent('Backwards Solve') at runtime, which the MR_Zebra instance does support. (3) ModularRig runs an event only on modules that support it, so face modules (no Backwards Solve) are never inverted during a bake and face controls stay at their current values.

**Setup.** Sequencer: 'Add Layered Control Rig' picker and 'Bake to Control Rig'.

**Operators:** `UModularRig::GetSupportedEvents`, `UControlRigRuntimeAsset::UpdateSupportedEventNames`, `FRigUnit_InverseExecution::EventName`

**Scale:** 1 stale list (MR_Zebra); 2 face modules without inverse

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/asset.t3d:2102-2103`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/asset.t3d:2737-2745`; `<dump>/Game__Assets__Monster__Rig__MR_Monster/asset.t3d:488-491`; `<dump>/Game__Assets__Zebra__Rig__MR_ZebraDMC/asset.t3d:2145-2153`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/ModularRig.cpp:450-454, 514-526`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/ControlRigRuntimeAsset.cpp:37-45`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRigEditor/Private/Sequencer/ControlRigParameterTrackEditor.cpp:2180-2201`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Sequencer/MovieSceneControlRigParameterSection.cpp:4257-4261`; `<dump>/FortniteRigs__Modules__IkSolves__IkFk2Bones__CRM_FN_IkFk2Bones/graphs.txt :: "events=['To IK', 'To FK', 'Key Controls']"`

#### UE8-boombox-prop-rig — MR_Boombox: prop rig made of stock Root + chained AddControl modules

*important* · assets: MR_Boombox, SK_Boombox, SKM_Boombox

Static hierarchy: Root (origin) -> handle T(-0.47,-0.26,63.8) R(0,0,40.63) -> boombox (+31.68 Z) -> button, button2, antenna, tape1, tape2. Socket handle_socket sits on handle with an identity offset. Module chain and connections: Root/Root -> bone Root. AddControl: Primary=socket handle_socket, Parent Control=Root/body_offset_ctrl. AddControl_1: Primary=socket AddControl/boombox_socket, Parent=AddControl/handle_ctrl. AddControl_2, _3, _4, _5, _6: Primary = AddControl_1/{button, button2, antenna, tape1, tape2}_socket, Parent=AddControl_1/boombox_ctrl. The resulting controls are Root/root_ctrl, Root/global_ctrl, Root/body_offset_ctrl, AddControl/handle_ctrl, AddControl_1/boombox_ctrl, AddControl_2/button_ctrl, AddControl_3/button2_ctrl, AddControl_4/antenna_ctrl, AddControl_5/tape1_ctrl and AddControl_6/tape2_ctrl. Each AddControl creates '<child>_socket' sockets for the child bones of its target, and the next module in the chain connects to them. Per-instance settings (stored in the editor ModularRig_0 module objects, since ConfigOverrides export as '()'): Root 'Root Module Settings' ControlSize=0.5. AddControl Shape Box_Thick. AddControl_1 ControlSize=5.0, shape Box_Thin with Translation X=5 and Scale3D (0.5,2,0.5). AddControl_2 and _3 Sphere_Thick. AddControl_4 ControlSize=0.5, Box_Thick at X=10. AddControl_5 CharacterFacingDownAxis=(1,1,0), Square_Thick at Z=1. AddControl_6 Square_Thick at Z=1. The rig's own graph is only an empty Forwards Solve. ShapeLibraries=[DefaultGizmoLibraryNormalized]; there are no saved SupportedEventNames. No runtime hierarchy was dumped.

**Setup.** 10 transform controls (handle, boombox body, two buttons, antenna, two tape reels, plus the root/global/body-offset stack); gizmo sizes set via the ControlSize module setting; shapes as listed.

**Operators:** `UModularRig`, `FRigModuleReference`, `HierarchyAddSocket (inside AddControl)`

**Scale:** 8 modules, 15 connections, 8 bones

**Evidence:** `<dump>/Game__Assets__Environment__Rig__MR_Boombox/modular_rig_model.txt :: 'Connector=(Type=Connector,Name="AddControl/Add Control Primary"),Targets=((Type=Socket,Name="handle_socket"'`; `<dump>/Game__Assets__Environment__Rig__MR_Boombox/hierarchy.txt:1-9`; `<dump>/Game__Assets__Environment__Rig__MR_Boombox/asset.t3d :: 'Root Module Settings=(ControlSize_12_1266EFC440EAC0A0D120DC82EC6819EA=0.500000)', 'Control Settings=(Shape=(Name="Box_Thin",Transform=(Translation=(X=5.000000'`; `<dump>/Game__Assets__Environment__Rig__MR_Boombox/graphs.txt :: 'nodes=1 | events=[\'Forwards Solve\']'`; `<dump>/_log.txt:2-8 (ERR ... create_control_rig)`; `zebra_audition.uasset name table: 'Root/body_offset_ctrl','Root/global_ctrl','Root/root_ctrl','AddControl/handle_ctrl','AddControl_1/boombox_ctrl'`

#### UE8-conn-optional-unconnected — Optional connectors left unconnected (the module takes its fallback path)

*important* · assets: MR_Zebra, MR_ZebraDMC, MR_Monster, MR_FN_Biped, MR_FN_BipedDMC

Every connector left unconnected in these rigs is declared bOptional=True in its module's RigModuleSettings.ExposedConnectors: FkArray Spaces/Orient Spaces/Override Parents (Secondary, array); FkChain Parent, End (with ChildOfPrimary rule) and Orient Spaces; Foot Toe Joints and the Toe Tip/Heel/Inner Bank/Outer Bank Pivots; Spine Start Snap To and Start/End Orient Spaces; Face Parent. The module graph resolves each connector and branches on whether it is connected (ResolveConnector bIsConnected, 'Set Default Match To Connector v01'). Unconnected in MR_Zebra (27): Foot L/R Toe Joints; Clavicle L/R End; Thumb/Index/Pinky L/R Orient Spaces; Neck/Start Snap To; Tweakers/Ear Base L/R/Ear L/R/Mohawk Spaces and Orient Spaces; Tweakers/Ear L/Ear R/Mohawk Override Parents. MR_Monster (3): Clavicle L/R End and Face/Parent. MR_FN_Biped (20): includes all 8 foot pivot connectors, even though the template's static hierarchy has the 8 imported MeshSocket nulls (foot_l_heel …), plus Meta L/R Spaces/Orient Spaces/Override Parents and Attach/Orient Spaces. MR_Zebra connects its pivot connectors to those nulls. An importer must support an 'absent' state for optional connectors and run the module's fallback, for example Foot pivots from 'Heel Pivot Offset'/'Toe Tip Pivot Offset' variables instead of socket nulls.

**Setup.** In the rig editor these connectors show as unassigned; nothing warns the user.

**Operators:** `FRigConnectorSettings (bOptional, bIsArray, Type Secondary, Rules)`, `RigChildOfPrimaryConnectionRule`

**Scale:** 27 + 3 + 20 (+20 DMC template) unconnected connectors

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkArray/summary.json :: 'Name=\"Spaces\",Settings=(Type=Secondary,bOptional=True,bIsArray=True)'`; `<dump>/FortniteRigs__Modules__Biped__Foot__CRM_FN_Foot/summary.json :: 'Name=\"Toe Tip Pivot\",Settings=(Type=Secondary,bOptional=True)'`; `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkChain/summary.json :: 'Name=\"End\",Settings=(Type=Secondary,bOptional=True,Rules=((ScriptStructPath=\"/Script/ControlRig.RigChildOfPrimaryConnectionRule\"'`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/summary.json :: 'Start Snap To'`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/hierarchy.txt :: 'NULL foot_l_heel parents=[\'BONE:ball_l\'] ... tags=[\'MeshSocket\']'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/modular_rig_model.txt :: 'Name="Foot L/Heel Pivot"),Targets=((Type=Null,Name="foot_l_heel"'`

#### UE1-auto-resolve — Auto-connect of secondary connectors (disabled in these rigs)

*nice-to-have* · assets: MR_Zebra, MR_ZebraDMC, MR_FN_Biped, MR_Monster, CRU_PropAim

FModularRigSettings.bAutoResolve (engine default true) auto-resolves secondary connectors once the primary is connected. AutoConnectSecondaryConnectors(keys, bReplace):
- skips primaries
- requires the primary to be connected
- runs FindMatches
- connects if there is exactly 1 match, otherwise connects the first DefaultTarget match
- otherwise leaves the connector unresolved

All the Fortnite-based rigs here set bAutoResolve=False, so every connection is stored explicitly in ConnectionList.

**Operators:** `FModularRigSettings::bAutoResolve`, `UModularRigController::AutoConnectSecondaryConnectors`, `UModularRigController::AutoConnectModules`

**Evidence:** `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Rigs/RigModuleDefines.h:16-27 (bAutoResolve = true default)`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/ModularRigController.cpp:698-838`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/asset.t3d:2051 'ModularRigSettings=(bAutoResolve=False)'`; `<dump>/FortniteRigs__UtilityRigs__CRU_PropAim/asset.t3d: 'ModularRigSettings=(bAutoResolve=False)'`

#### UE1-biped-template-extras — Biped template-only modules: Stretch Feedback, IK Bone Pins, Attach, finger/meta proxies

*nice-to-have* · assets: MR_FN_Biped, MR_FN_BipedDMC

- Stretch Feedback (CRM_FN_BipedStretchFeedback): Root -> root. Spine Elements = spine_01..05. Arm Elements and Leg Elements list only the left chains (upperarm_l, its twists, lowerarm_l, its twists, hand_l / thigh_l ... foot_l). Vis Channel Control -> root/Global. Creates the 'Stretch FeedBack Vis' bool on root/Global. Colors: Rest green, Squash blue, Stretch red; Max Stretch Factor 2.
- IK Bone Pins (CRM_FN_Pin): no controls. Pins the IK virtual bones to the deform bones index-wise: ik_hand_gun <- hand_r, ik_hand_l <- hand_l, ik_foot_l <- foot_l, ik_foot_r <- foot_r, ik_hand_r <- hand_r.
- Attach: an FkArray over the attach/weapon bones.
- 12 ProxyControl modules: finger and meta curl/spread.

Zebra drops all of these.

**Operators:** `CRM_FN_BipedStretchFeedback`, `CRM_FN_Pin`, `CRM_FN_ProxyControl`, `CRM_FN_FkArray`

**Scale:** 15 extra modules and 31 extra controls compared with Zebra's body.

**Evidence:** `<dump>/FortniteRigs__Templates__MR_FN_Biped/modular_rig_model.txt: 'Stretch Feedback/Arm Elements' -> upperarm_l ... hand_l`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/modular_rig_model.txt: 'IK Bone Pins/Drivers' / 'IK Bone Pins/Driven'`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/runtime_hierarchy.txt:1710 'CONTROL Stretch Feedback/Stretch FeedBack Vis'`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/asset.t3d: Stretch Feedback 'Max Stretch Factor=2.000000'`

#### UE1-boombox-addcontrol-chain — Prop rig from generic engine modules: Root + chained AddControl via sockets

*nice-to-have* · assets: MR_Boombox

Uses the engine's generic Modules58.
- Root: 'Root/Root' -> bone Root; provides control 'Root/body_offset_ctrl'.
- AddControl: primary 'Add Control Primary' requires a Socket (Type rule). Secondary optional 'Parent Control' accepts Null, Control or Bone (Or rule).

Chain:
- AddControl -> user socket handle_socket (on bone handle); parent control Root/body_offset_ctrl.
- AddControl_1 -> Socket 'AddControl/boombox_socket'; parent AddControl/handle_ctrl.
- AddControl_2 and AddControl_3 -> 'AddControl_1/button_socket' and 'AddControl_1/button2_socket'.
- AddControl_4 -> antenna_socket. AddControl_5 and AddControl_6 -> tape1_socket and tape2_socket.
- All of AddControl_2..6 use parent AddControl_1/boombox_ctrl.

Each AddControl therefore creates '<bone>_ctrl' for its socket's bone and spawns '<child>_socket' sockets on child bones for downstream modules. Per-instance struct config: Control Settings.Shape (Box_Thick, Box_Thin offset (5, 0, 0) at scale (0.5, 2, 0.5), Sphere_Thick, Square_Thick offset z = 1), Module Settings ControlSize (5 or 0.5), CharacterFacingDownAxis (1, 1, 0). bAutoResolve is not set, so the engine default applies.

**Operators:** `/ControlRig/Modules/Modules58/Root`, `/ControlRigModules/Modules58/AddControl`, `FRigTypeConnectionRule(Socket)`, `FRigOrConnectionRule`

**Scale:** 8 modules, 15 connectors, 7 controls expected (runtime not dumped).

**Evidence:** `<dump>/Game__Assets__Environment__Rig__MR_Boombox/modular_rig_model.txt: 'Name="AddControl_1/Add Control Primary"),Targets=((Type=Socket,Name="AddControl/boombox_socket"))'`; `<dump>/Game__Assets__Environment__Rig__MR_Boombox/modular_rig_model.txt: 'Name="AddControl/Parent Control"),Targets=((Type=Control,Name="Root/body_offset_ctrl"))'`; `<dump>/Game__Assets__Environment__Rig__MR_Boombox/asset.t3d: 'Root Module Settings=(ControlSize_12_1266EFC440EAC0A0D120DC82EC6819EA=0.500000)'`

#### UE1-module-asset-kinds — Blueprint-class modules vs runtime-asset modules (property-bag variables)

*nice-to-have* · assets: MR_Zebra, MR_Monster, MR_ZebraDMC, MR_Boombox

A module reference is one of two kinds:
- a Blueprint-generated class (BlueprintRigClass=/…/CRM_FN_*.CRM_FN_*_C). The instance is a sub-object of that class, with variables as UProperties.
- a ControlRigRuntimeAsset (ControlRigAsset=/…/CRM_Zebra_Face, CRM_Monster_Face, CRM_FN_DMC). The instance is a plain UControlRig ('/Script/ControlRig.ControlRig') with GeneratedBy=<runtime asset>, a copy of its RigModuleSettings, and variables held in a transient PropertyBag.

MR_Boombox is itself a ControlRigRuntimeAsset hosting a ModularRig_0 editor instance. Its modules come from the engine plugins (/ControlRig/Modules/Modules58/Root and /ControlRigModules/Modules58/AddControl).

**Operators:** `FControlRigAssetSoftReference`, `UControlRigRuntimeAsset`, `UPropertyBag`

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/asset.t3d:731-737 'Begin Object Name="Face" ExportPath="/Script/ControlRig.ControlRig'...GeneratedBy=ControlRigRuntimeAsset'/Game/Assets/Zebra/Rig/CRM_Zebra_Face.CRM_Zebra_Face''`; `<dump>/Game__Assets__Environment__Rig__MR_Boombox/modular_rig_model.txt: 'BlueprintRigClass="/ControlRig/Modules/Modules58/Root.Root_C"'`; `<dump>/FortniteRigs__Modules__Miscellaneous__CRM_FN_DMC/summary.json: 'class': 'ControlRigRuntimeAsset'`

#### UE1-module-settings-identity — Module identity and library metadata (RigModuleSettings)

*nice-to-have* · assets: MR_Zebra, MR_ZebraDMC, MR_Monster

Each module asset declares RigModuleSettings{Identifier(Name, Type='Module'), Icon (texture), Category, Keywords, Description, ExposedConnectors}. The Fortnite modules use Category 'Fortnite Modules' with per-module icons (FortIconFoot, FortIcon2Bones, ...). The LimbTwist identifier is 'CRM_Epic_LimbTwist_v02'. The DMC description tells users to add it 'directly below the CRM_FN_Root module to enable DMC FK and IK layers' and that it requires the Direct Mesh Control plugin. Host modular rigs have an empty identifier; they are not modules themselves.

**Operators:** `FRigModuleSettings`, `FRigModuleIdentifier`

**Evidence:** `<dump>/FortniteRigs__Modules__Biped__LimbTwist__CRM_FN_LimbTwist/summary.json: 'Identifier=(Name=\"CRM_Epic_LimbTwist_v02\"'`; `<dump>/Game__Assets__Zebra__Rig__MR_ZebraDMC/asset.t3d: 'Description="\r\nEXPERIMENTAL \r\nMUST HAVE DIRECT MESH CONTROL PLUGIN ENABLED\r\n\r\nAdd directly below the CRM_FN_Root module'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/summary.json: 'rig_module_settings': '(Identifier=(Name=\"\",Type=\"Module\")...ExposedConnectors=)'`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Rigs/RigModuleDefines.h:29-78`

#### UE1-zebra-vs-biped-template — Lineage: MR_Zebra derived from MR_FN_Biped template

*nice-to-have* · assets: MR_Zebra, MR_FN_Biped, MR_Monster

PreviousModulePaths is identical in all three rigs (Root:Spine:Clavicle L:Arm L:Meta L:Middle L, ...), which shows Zebra and Monster started from the Biped template.

Zebra removed: Meta L/R, Middle/Ring L/R, the 12 proxy modules, Attach, Stretch Feedback and IK Bone Pins. It re-parented Index/Pinky from Meta to Arm, with Start = the metacarpal bone.

Zebra added: Tweakers, Face, Ear Base L/R, Ear L/R and Mohawk.

Zebra re-tuned: control scales (root 1.8/1.1, Body 0.8, Spine 1.2, Neck 2 vs 3); spine Local FK shape scale; leg PV Distance Scale 1.25; arm PV Distance Scale 2 and PV shape 0.5; FK shape Z scale (8 vs 1.47); twist weights (4 entries vs 2); foot pivot Z offsets (-3); finger shape scales.

Zebra connected the foot pivot connectors to the socket nulls; the template leaves them unconnected. The 20 (Zebra) and 38 (Biped) stale ConnectionList entries use older connector names ('Root/Root', 'Spine/StartParent', 'Clavicle L/OrientSpace', 'Meta L/Bone A', 'Attach/Parents', ...) that match no existing connector. They are kept alongside the current names and are harmless.

**Operators:** `FModularRigModel::PreviousModulePaths`

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/modular_rig_model.txt: 'PreviousModulePaths=(((ModulePath="Root:Spine:Clavicle L:Arm L:Meta L:Middle L"), "Middle L")'`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/modular_rig_model.txt: 'Name="Index L",ParentModuleName="Meta L"'`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/asset.t3d: Neck 'Control Scale=3.000000' vs <dump>/Game__Assets__Zebra__Rig__MR_Zebra/asset.t3d Neck 'Control Scale=2.000000'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/modular_rig_model.txt: stale 'Name="Root/Root"' (no CONNECTOR Root/Root in hierarchy.txt; live one is 'root/RootJoint')`

#### UE2-fkchain-connector-event — Connector event default matches (FkChain)

*nice-to-have* · assets: CRM_FN_FkChain, CRFL_Module_v001

Connector event, run while connectors are being resolved:
- A: Set Default Match To Connector v01(Connector = 'Parent', Default = hierarchy parent of the resolved Start connector).
- B: if Start has any recursive bone children, Set Default Match(Connector = 'End', Default = the last recursive bone child, index -1).

The library function calls SetDefaultMatch only when GetCandidates().Connector equals the given connector.

**Setup.** Connector UX only.

**Operators:** `RigUnit_ConnectorExecution`, `RigUnit_Item`, `RigUnit_HierarchyGetParent`, `RigUnit_CollectionChildrenArray`, `RigUnit_GetCandidates`, `RigUnit_SetDefaultMatch`

**Scale:** FkChain (5 calls total in the libraries)

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkChain/graphs.txt:361-385`; `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkChain/regen.py:301 'Item.Item', '(Type=Connector,Name="Start")'`; `<dump>/FortniteRigs__Libraries__CRFL_Module_v001/graphs.txt:361-377`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/Modules/RigUnit_ConnectionCandidates.h:51-55`

#### UE2-mirror-metadata — Mirror axis / behavior metadata on FK controls

*nice-to-have* · assets: CRM_FN_FkChain, CRM_FN_FkArray, CRFL_Control_v001

For each spawned FK control:
- Set Mirror Axis: SetMetadata(control, 'Mirror Axis', FVector, NameSpace None)
- Set Mirror Behavior: SetMetadata(control, 'Mirror Behavioral', bool)

The graphs never consume these values. External mirroring and pose tools use them.

**Setup.** Public 'Mirror Axis' (default (0,1,1)) and 'Mirror Behavior' (FkChain default false, FkArray default true), category 'Mirroring'.

**Operators:** `DISPATCH_RigDispatch_SetMetadata`

**Scale:** 10 + 3 calls

**Evidence:** `<dump>/FortniteRigs__Libraries__CRFL_Control_v001/graphs.txt:24-45 'Name=Mirror Axis' 'Name=Mirror Behavioral'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt 'Thumb L/FK 0 ... Mirror Behavioral:BOOL', 'Mirror Axis:VECTOR'`

#### UE4-connect-to-module-metadata — Connect to Module Metadata (unused)

*nice-to-have* · assets: CRFL_Module_v001

Connector event only. Inputs: Connector, Module Metadata Name and Module Metadata NameSpace (default 'Parent Control' / Parent). Outputs: Found and Result. If the connector being resolved equals Connector, then for each candidate c: (value, found) = GetModuleMetadata(Name, NameSpace) as an item key; found_metadata = found; if found and c == value, it calls SetDefaultMatch(c) and sets Result = c. This lets a parent module publish a preferred attachment element as metadata.

**Operators:** `RigUnit_GetCandidates`, `DISPATCH_RigDispatch_GetModuleMetadata (Item)`, `RigUnit_SetDefaultMatch`, `DISPATCH_RigVMDispatch_ArrayIterator`

**Scale:** No callers.

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Module_v001/graphs.txt:183-255 'Connect to Module Metadata' (comment '(modules only) (connector event only)')`; `ue/<dump>/FortniteRigs__Libraries__CRFL_Module_v001/graphs.txt:13 defaults 'Module Metadata Name=Parent Control; Module Metadata NameSpace=Parent'`

#### UE6-authoring-quirks — Authoring defects to reproduce or fix when porting

*nice-to-have* · assets: CRM_Zebra_Face, CRM_Monster_Face

Verified quirks that change results. (1) Correctives write wide_open_c_r twice (the second time as frown_r*wide_r), so frown_wide_c_r is never set. (2) Lip Roll Ot Bt writes curve lip_roll_in_bt_l, which In Bt then overwrites with an unclamped negative value. Monster Ch Tp writes lip_roll_ot_tp_l. (3) Lip Roll In and Lip Puff pose readers reference nonexistent controls 'Lip Main Tp'/'Lip Main Bt', so their output is always 0. (4) Zebra ParentConstraint_164 lists Lid In R twice, and _165 uses the L In/Ot controls for the R bt-base chain (fixed in Monster). (5) The Brow Mid R null is placed at +X. (6) Position constraints target nonexistent nulls 'Brow In/Ot L/R', so they do nothing. (7) Brow squeeze L/R outputs are crossed. (8) Monster writes lid_bt_blink_extend_l twice and never writes the _r curve. (9) Orphan nodes: highlight_offset_x, Set Transform_10 (null Teeth Tp) and an extra muzzle_squash_deformer write. (10) The corner-height constraints on the lip bones are overwritten by pin B. (11) Jaw Normalize lags one evaluation. (12) The Squint L/R (Monster) and Smile Open members are unused.

**Setup.** None.

**Operators:** `(diagnostic)`

**Scale:** 12 items

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:2008 SetCurveValue_34 Curve=wide_open_c_r`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:2710 SetCurveValue_41 Curve=lip_roll_in_bt_l`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:1081 ParentConstraint_164`; `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/graphs.txt:1315 SetCurveValue_83 Curve=lid_bt_blink_extend_l`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:2214 NULL Face/Brow Mid R T(10,20,118)`

#### UE8-asset-variant-validator-limits — Asset-level metadata: AssetVariant tags, validator, auto-resolve, procedural element limit

*nice-to-have* · assets: CRM_FN_BipedStretchFeedback, CRM_FN_Pin, CRM_FN_ProxyControl, CRU_PropAim, MR_Zebra, MR_Monster, MR_FN_Biped, MR_Boombox

AssetVariant tags: 'Stable' on CRM_FN_BipedStretchFeedback, CRM_FN_Pin and CRM_FN_ProxyControl; 'AnimatorKit_Utility' (label Utility: 'Utility rigs can be used with the Constraint System for on-demand rigging in Sequencer') on CRU_PropAim together with bAllowMultipleInstances=True. The other rigs have a variant GUID only. Every rig has Validator=ControlRigValidator with no Passes array, so no validation passes run. All 8 modular rigs set ModularRigSettings.bAutoResolve=False, so connectors are not auto-resolved on module add. HierarchySettings.ProceduralElementLimit saves 2000 + the static element count: MR_Zebra 3486 = 2000 + 1486 static elements (958 curves, 371 bones, 8 nulls, 149 connectors); MR_Monster 3184; MR_FN_Biped 3106; MR_Boombox 2024. At runtime UControlRig adds the CDO hierarchy count to the 2000 default. Any DynamicHierarchy node fails with 'Node has hit the Procedural Element Limit' once Hierarchy->Num() reaches the limit. MR_Zebra's runtime total of 2215 elements is below it.

**Setup.** Class Settings > Hierarchy > Procedural Element Limit

**Operators:** `FRigHierarchySettings.ProceduralElementLimit`, `FRigUnit_DynamicHierarchyBase`, `UControlRigValidator`, `FModularRigSettings.bAutoResolve`

**Scale:** all rigs

**Evidence:** `<dump>/FortniteRigs__Modules__Biped__StretchFeedback__CRM_FN_BipedStretchFeedback/asset.t3d:10906`; `<dump>/FortniteRigs__UtilityRigs__CRU_PropAim/asset.t3d:160, 169-170, 176`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/asset.t3d:805, 2051, 2093`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/ControlRig.cpp:850-854`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Execution/RigUnit_DynamicHierarchy.cpp:37-44`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/RigUnitContext.h:47-72`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/summary.json :: hierarchy_counts / runtime_hierarchy_counts`

### D17 Libraries and reuse

#### UE2-fk-naming — Metadata-free control naming helpers (Get Item Name v01 / Conform Name v01 / Get Control Name)

*important* · assets: CRM_FN_FkChain, CRM_FN_FkArray, CRM_FN_Spine, CRFL_Module_v001

Conform Name v01(Name, RemoveSideSuffix):
1. Replace ' ' with '_' and split on '_'.
2. If RemoveSideSuffix, drop tokens equal to 'l' or 'r' (case-insensitive).
3. Capitalize each remaining token and join them with spaces, then trim.
4. If no tokens remain, return the input capitalized.

Get Item Name v01(Name, StdNumerical, Index, Count, Prefix, Suffix, RemoveSide, StartIdx):
- mid = StdNumerical ? (Count > 1 OR (Prefix == '' AND Suffix == '') ? str(Index + StartIdx) : '') : Name
- result = Conform(join([Prefix, mid, Suffix], ' '))

Examples:
- FkChain: 'FK', 'FK 0', 'FK 1'; or 'Clavicle FK' when names are not standard.
- Spine Sec FK (Suffix 'Sec FK', start 1): 'spine_01' -> 'Spine 01 Sec FK'.
- FkArray 'Get Control Name': StandardFKNames ? (Count > 1 ? 'FK <i>' : 'FK') : Conform(bone), keeping side suffixes (e.g. 'Ear 02 L', 'Def Thigh In L').

**Setup.** FkChain 'Control Names As FK' (default true); FkArray 'Standard FK Names' (default false). Element names are namespaced by the module ('Thumb L/FK 0').

**Operators:** `RigVMFunction_StringSplit`, `RigVMFunction_StringReplace`, `RigVMFunction_StringToUppercase`, `RigVMFunction_StringToLowercase`, `RigVMFunction_StringJoin`, `RigVMFunction_MathIntToName`, `RigVMFunction_NameConcat`

**Scale:** 4 Conform calls, 3 Get Item Name calls

**Evidence:** `<dump>/FortniteRigs__Libraries__CRFL_Module_v001/graphs.txt:378-490 'Conform Name v01'`; `<dump>/FortniteRigs__Libraries__CRFL_Module_v001/graphs.txt:510-573 'Get Item Name v01'`; `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkArray/graphs.txt:737-760 'Get Control Name'`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:621 'Suffix=Sec FK; Remove Side Suffix=true; Numerical Name Start Index=1'`

#### UE4-conform-name — Conform Name v01 (bone name to display/control name)

*important* · assets: CRFL_Module_v001, CRM_FN_FkArray, CRM_FN_FkChain

Inputs: Name and Remove Side Suffix. Output: Conformed Name. s = ToString(Name) with ' ' replaced by '_'. tokens = Split(s, '_'), keeping empty tokens. Tokens equal to 'l' or 'r' (case-insensitive after lowercasing) are dropped when Remove Side Suffix is true. If no tokens remain, out = Upper(s[0]) + s[1:], using the original name. Otherwise out = concatenation over tokens of Upper(tok[0]) + tok[1:] + ' '. Result = FName(TrimStartAndEnd(out)). Examples: 'upperarm_l' gives 'Upperarm L', or 'Upperarm' with removal; 'spine_01' gives 'Spine 01'. Interior empty tokens produce double spaces.

**Operators:** `DISPATCH_RigDispatch_ToString / FromString`, `RigVMFunction_StringReplace`, `RigVMFunction_StringSplit`, `RigVMFunction_StringToLowercase / ToUppercase`, `RigVMFunction_StringLeft / Right / Length`, `RigVMFunction_StringConcat`, `RigVMFunction_StringTrimWhitespace`, `DISPATCH_RigVMDispatch_ArrayAdd/GetNum`

**Scale:** FkArray 2, FkChain 1 (inside local Get Control Name), plus every Get Item Name v01 call.

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Module_v001/graphs.txt:378-489 'Conform Name v01'`; `ue/<dump>/FortniteRigs__Libraries__CRFL_Module_v001/regen.py:902 'RigVMFunction_StringSplit.Separator' '_' ; :963-964 Replace.Old ' ' / New '_' ; :969/:975 Equals.B 'l' / 'r' ; :914 'Concat.D' ' '`; `ue/<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkArray/graphs.txt:171 'Conform Name v01' -> Concat.A (null naming)`

#### UE4-get-chain-length — Get Chain Length

*important* · assets: CRFL_Math_v001, CRM_FN_Foot, CRM_FN_LimbTwist, CRM_FN_Spine, CRM_FN_Root, CRM_FN_IkFk2Bones

Inputs: Items (RigElementKey[]) and Initial (bool). Output: Length (float). The result is the sum over i=0..n-2 of |P_i - P_{i+1}|, where P is the global translation read with bInitial=Initial. The accumulator is the local variable 'distance'. Callers use Initial=true and divide the result by a per-module reference length to get a shape auto-scale factor.

**Operators:** `RigUnit_GetTransform`, `RigVMFunction_MathVectorDistance`, `RigVMFunction_MathFloatAdd`, `DISPATCH_RigVMDispatch_ArrayIterator`, `DISPATCH_RigVMDispatch_ArrayGetAtIndex`, `RigVMFunction_MathIntAdd/Sub`, `DISPATCH_RigVMDispatch_CoreEquals`

**Scale:** 5 modules, 1 call each.

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Math_v001/graphs.txt:16-81 'Get Chain Length' (comment 'Return a float distance representing the length of a chain')`; `ue/<dump>/FortniteRigs__Modules__IkSolves__IkFk2Bones__CRM_FN_IkFk2Bones/graphs.txt:711 'Divide_1 ... B=55.000000' and :1148 'Get Chain Length.Length -> Divide_1.A'`; `ue/<dump>/FortniteRigs__Modules__FkSolves__Root__CRM_FN_Root/graphs.txt:98 'Divide_1 ... B=96.000000'`

#### UE4-get-item-name — Get Item Name v01 (numbered or bone-derived control naming)

*important* · assets: CRFL_Module_v001, CRM_FN_Spine, CRM_FN_FkChain, CRM_FN_IkFk2Bones

Inputs: Name, Standard Numerical Names, Index, Count, Prefix, Suffix, Remove Side Suffix and Numerical Name Start Index (library default 1). Output: Result. num = str(Index + StartIndex). useNum = (Count > 1) OR (Prefix == '' AND Suffix == ''). core = StandardNumericalNames ? (useNum ? num : '') : str(Name). Result = ConformName(Join([Prefix, core, Suffix], ' '), RemoveSideSuffix). Call sites: FkChain uses Prefix='FK', Suffix='' when numerical, else Prefix='' and Suffix='FK', StartIndex 0, giving names like 'FK 0' or 'Upperarm FK'. Spine uses Suffix='Sec FK' with side removal, e.g. 'Spine 01 Sec FK'. IkFk2Bones uses Standard=true, Prefix='FK' and side removal.

**Setup.** Module options such as 'Standard Numerical Names' control whether FK controls are named 'FK 0..n' or after their bones. The module prefix ('Arm L/') is added by the modular rig.

**Operators:** `RigVMFunction_MathIntAdd`, `RigVMFunction_MathIntToName`, `RigVMFunction_MathIntGreater`, `DISPATCH_RigVMDispatch_CoreEquals`, `RigVMFunction_StringJoin (Separator ' ')`, `DISPATCH_RigVMDispatch_If`, `FUNC Conform Name v01`

**Scale:** 3 call sites (Spine, FkChain, IkFk2Bones).

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Module_v001/graphs.txt:510-573 'Get Item Name v01'`; `ue/<dump>/FortniteRigs__Libraries__CRFL_Module_v001/regen.py:1085 'RigVMFunction_StringJoin.Separator' ' '`; `ue/<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkChain/graphs.txt:148 'Get Item Name v01 | Remove Side Suffix=true; Numerical Name Start Index=0' and :145-146 If 'True=FK' / If_1 'False=FK'`; `ue/<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:621 'Suffix=Sec FK; Remove Side Suffix=true'`; `ue/<dump>/FortniteRigs__Modules__IkSolves__IkFk2Bones__CRM_FN_IkFk2Bones/graphs.txt:895 'Standard Numerical Names=true; Prefix=FK'`; `ue/<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:855 'CONTROL Arm L/FK 0'`

#### UE4-library-structure-versioning — CRFL library structure, versioning and reuse pattern

*important* · assets: CRFL_Control_v001, CRFL_Debug_v001, CRFL_Hierarchy_v001, CRFL_Math_v001, CRFL_Module_v001

Every CRFL asset is a plain ControlRigBlueprint whose public RigVMFunctionLibrary is referenced by modules through FunctionReference nodes ('FUNC <name> @ /FortniteRigs/Libraries/CRFL_*'). Changed behaviour ships as a new function name with a version suffix (v01/v02: Get Control Shape Name From Item, Control Color Override, Get Control Color, Create IK Plane Virtual Bones, Compute PV Location), and the old version is kept. Some helpers are duplicated as module-local copies, e.g. 'Control Color Override From Metadata' in Spine and IkFk2Bones, and 'Blend Twist' in LimbTwist. Unused public functions: Add Mirror Tag, Scale Control Shape v01, Add Null Below, Set Contol Color By Position, Create IK Plane Virtual Bones v01, Get Mirror Transform, Compute/Construct Auto Pole Vector v02, Compute Pole Vector Location v01, CRFL_Math Compute Pole Vector, Blend Twist, Blend Position, Connect to Module Metadata, Create FK Chain Controls, and the Control Stack chain. Each library's event graphs contain ad-hoc test nodes (Print, Item Array, test calls on Manny bones).

**Operators:** `RigVMFunctionReferenceNode`, `RigVMCollapseNode (function definitions)`, `RigVMAggregateNode (Concat/Multiply/Sequence aggregates)`

**Scale:** 5 libraries, 46 public functions. 26 distinct functions have external callers (counting indirect use).

**Evidence:** `ue/<dump>/functions_used.json per_rig (CRFL_* entries and module entries)`; `ue/<dump>/FortniteRigs__Libraries__CRFL_Control_v001/graphs.txt:3-6 test stubs (Print 'Prefix=a:', Item Array clavicle_l/upperarm_l/lowerarm_l)`; `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:4 test call 'Search String=lowerarm|twist'`; `ue/<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:1675 local 'Control Color Override From Metadata' graph`

#### UE4-rigvm-control-flow-dispatch — RigVM control flow and generic dispatch nodes

*important* · assets: CRFL_Control_v001, CRFL_Debug_v001, CRFL_Hierarchy_v001, CRFL_Math_v001, CRFL_Module_v001

RigVMFunction_Sequence runs its A, B, C... outputs in order; aggregate versions add more outputs. RigVMFunction_ControlFlowBranch runs its True or False output, then Completed. Generic dispatch nodes: If (select by type), CoreEquals/NotEquals (typed comparison, including LinearColor components), ArrayIterator (Element, Index, Count, Ratio; loop body, then Completed), ArrayGetAtIndex (negative index = from the end, e.g. -1 = last), ArrayAdd, ArrayFind, ArrayReset, ArrayMake, ArrayGetNum, SelectInt32, CastEnumToInt, MakeStruct and Print (on-screen error text). Function-local variables (LOCALVAR) hold accumulators and results.

**Operators:** `RigVMFunction_Sequence`, `RigVMFunction_ControlFlowBranch`, `DISPATCH_RigVMDispatch_If`, `DISPATCH_RigVMDispatch_CoreEquals`, `DISPATCH_RigVMDispatch_CoreNotEquals`, `DISPATCH_RigVMDispatch_ArrayIterator`, `DISPATCH_RigVMDispatch_ArrayGetAtIndex`, `DISPATCH_RigVMDispatch_ArrayAdd`, `DISPATCH_RigVMDispatch_ArrayFind`, `DISPATCH_RigVMDispatch_ArrayReset`, `DISPATCH_RigVMDispatch_ArrayMake`, `DISPATCH_RigVMDispatch_ArrayGetNum`, `DISPATCH_RigVMDispatch_SelectInt32`, `DISPATCH_RigVMDispatch_CastEnumToInt`, `DISPATCH_RigVMDispatch_MakeStruct`, `DISPATCH_RigVMDispatch_Print`

**Scale:** Hierarchy: Sequence 18, Branch 17, If 12, ArrayGetAtIndex 11.

**Evidence:** `ue/<dump>/units_used.json (CRFL_* per-rig entries)`; `ue/<dump>/FortniteRigs__Modules__IkSolves__IkFk2Bones__CRM_FN_IkFk2Bones/graphs.txt:2728 'At ... Index=-1' (last virtual bone)`

#### UE4-rigvm-name-string — RigVM name vs string functions (case-sensitivity matters)

*important* · assets: CRFL_Control_v001, CRFL_Hierarchy_v001, CRFL_Module_v001

The FName functions go through the name cache. NameConcat treats a None part as empty. NameReplace is case-insensitive; a None Old returns the input and a None New means ''. Name StartsWith, EndsWith and Contains are case-insensitive. The FString functions (StringEndsWith, StartsWith, Contains, Split) are case-sensitive. Split keeps interior empty tokens. Join inserts the separator between all values, including empty ones. TrimWhitespace trims both ends. Left, Right, Upper, Lower and Replace behave as their FString counterparts. The libraries therefore detect a module's side case-sensitively (' L' / ' R') and a bone's side case-insensitively (Has Side).

**Operators:** `RigVMFunction_NameConcat`, `RigVMFunction_NameReplace`, `RigVMFunction_StartsWith`, `RigVMFunction_EndsWith`, `RigVMFunction_Contains`, `RigVMFunction_IsNameValid`, `RigVMFunction_StringEndsWith`, `RigVMFunction_StringContains`, `RigVMFunction_StringSplit`, `RigVMFunction_StringJoin`, `RigVMFunction_StringConcat`, `RigVMFunction_StringReplace`, `RigVMFunction_StringTrimWhitespace`, `RigVMFunction_StringLeft`, `RigVMFunction_StringRight`, `RigVMFunction_StringLength`, `RigVMFunction_StringToUppercase`, `RigVMFunction_StringToLowercase`, `RigVMFunction_MathIntToName`, `DISPATCH_RigDispatch_ToString`, `DISPATCH_RigDispatch_FromString`

**Scale:** NameConcat: Hierarchy 21, Control 10, Module 2. String functions: Module about 20 nodes.

**Evidence:** `UE_5.8/Engine/Plugins/Runtime/RigVM/Source/RigVM/Private/RigVMFunctions/RigVMFunction_Name.cpp:11-16, :117-122, :163-227`; `UE_5.8/Engine/Plugins/Runtime/RigVM/Source/RigVM/Private/RigVMCore/RigVMNameCache.cpp:101-107, :225-251`; `UE_5.8/Engine/Plugins/Runtime/RigVM/Source/RigVM/Private/RigVMFunctions/RigVMFunction_String.cpp:43-56, :63-92, :118-152`

#### UE7-dmc-shape-name-resolution — Get Control Shape Name From Item v02 (DMC-aware shape picking)

*important* · assets: CRFL_Control_v001, CRM_FN_IkFk2Bones, CRM_FN_FkChain, CRM_FN_FkArray, CRM_FN_Spine, CRM_FN_Foot

Library function. Inputs: Item (FRigElementKey, usually a bone or connector), ShapeLib Namespace (FName), Default Shape (FName), Direct Mesh Control Libraries (FName array). Output: Result (FName).

Steps:
1. found = Find(Libraries, Namespace).Success.
2. bone = ResolveConnector(Item).Name, with SkipSocket=False.
3. candidate = found ? Namespace + '.' + bone : DefaultShape.
4. probe = (!found AND DefaultShape != 'Default') ? DefaultShape : candidate.
5. Result = ShapeExists(probe) ? candidate : DefaultShape.

Net effect: if the DMC layer was loaded and a polygroup exists for that bone, the control uses shape '<layer>.<bone>'; otherwise it keeps its normal gizmo shape.

**Setup.** The module decides which layer each control uses (see UE7-dmc-module-consumers).

**Operators:** `RigVMFunctionReferenceNode 'Get Control Shape Name From Item v02'`, `RigUnit_ResolveConnector`, `RigUnit_ShapeExists`, `DISPATCH_RigVMDispatch_ArrayFind`, `DISPATCH_RigVMDispatch_If`, `RigVMFunction_NameConcat`

**Scale:** Used by 5 Fortnite modules, 9 call sites.

**Evidence:** `<dump>/FortniteRigs__Libraries__CRFL_Control_v001/graphs.txt:430-490 (function body and Concat_1_1 contained graph, 'Not_Equals ... B=Default', 'Concat_1_1 ... B=.')`

#### UE-reuse-by-duplication — Asset reuse by duplication between characters

*nice-to-have* · assets: CR_Monster_Deform, CR_Zebra_Deform, Zebra deformer graphs, SK_ZebraHi, ZebraMuzzleTwist_DeformerGraph

Neither character uses a shared library for these assets; they were copied:
- CR_Monster_Deform's clavicle section is a copy of Zebra's: same node names ModifyTransforms_2_2_2_x and SphericalPoseReader_1_1_1_1_x, and def_trap rest transforms identical to def_strap.
- The Zebra deformer kernels were generated from Monster graphs (the '#line' source is Monster_Head_DeforerGraph).
- SK_ZebraHi is an unreferenced copy of SK_Zebra.
- ZebraMuzzleTwist_DeformerGraph exists but CR_Zebra_Deform never adds it.
No function libraries (CRFL) are used by either deform rig (functions_used is empty).

**Setup.** None.

**Scale:** 4 duplicated/unused assets

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraHeadBend_DeformerGraph/kernels.hlsl:4 #line 1 "/Engine/Generated/UObject/Game/Assets/Monster/Deformers/Monster_Head_DeforerGraph..."`; `<dump>/Game__Assets__Monster__Meshes__SKM_Monster/bones.txt def_trap_l local=T(5.309,0.822,4.113) R(-0.2358,-4.45,21.72) vs <dump>/Game__Assets__Zebra__Meshes__SKM_Zebra/bones.txt def_strap_l (same)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/summary.json functions_used {}`; `<dump>/asset_index.json OptimusDeformer /Game/Assets/Zebra/Rig/Deformers/ZebraMuzzleTwist_DeformerGraph (no references in CR_Zebra_Deform graphs.txt)`

#### UE4-has-side — Has Side (side token detection in names)

*nice-to-have* · assets: CRFL_Hierarchy_v001

Inputs: Name (FName) and Side to Check (FName). Output: Result (bool). Result = StartsWith(Name, Side+'_') OR Contains(Name, '_'+Side+'_') OR EndsWith(Name, '_'+Side). The Name-type RigVM functions compare case-insensitively (ESearchCase::IgnoreCase), so 'upperarm_l', 'L_hand' and 'foot_L' all match 'l'. The regen default 'EndsWith_2.Ending'='_R' is overridden by the linked Concat.

**Operators:** `RigVMFunction_StartsWith`, `RigVMFunction_Contains`, `RigVMFunction_EndsWith`, `RigVMFunction_NameConcat`, `RigVMFunction_MathBoolOr`

**Scale:** Called only by Get Control Color From Metadata v02 (2 calls).

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:1639 'Has Side.New Function_ContainedGraph' (Concat B=_ ; Concat_1 A=_ C=_ ; Concat_2 A=_)`; `UE_5.8/Engine/Plugins/Runtime/RigVM/Source/RigVM/Private/RigVMFunctions/RigVMFunction_Name.cpp:163-227 EndsWith/StartsWith/Contains use ESearchCase::IgnoreCase`

#### UE6-rename-joint-to-control — Rename Joint to Control (snake_case bone to display-name control)

*nice-to-have* · assets: CRM_Zebra_Face, CRM_Monster_Face

This is a chain of NameReplace calls applied in order: lid_bt_base_ -> 'Lid Bt Base ', lid_tp_base_ -> 'Lid Tp Base ', lid_bt -> 'Lid Bt', lid_tp -> 'Lid Tp', lip_corner -> 'Lip Corner', lip_bt -> 'Lip Bt', lip_tp -> 'Lip Tp', lip_ -> 'Lip ', tp_ -> 'Tp ', _l -> ' L', _r -> ' R', bt_ -> 'Bt '. Examples: lid_tp_base_01_l -> 'Lid Tp Base 01 L', lip_corner_r -> 'Lip Corner R'. It is used for the lid bones and controls and for the lip tweaker nulls and controls.

**Setup.** Naming convention: capitalized words, with side suffix ' L'/' R' and Tp/Bt abbreviations.

**Operators:** `RigVMFunction_NameReplace`

**Scale:** 12 NameReplace; called 9x per build

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:1939-1971 Rename Joint to Control`

#### UE7-dg-shared-function-library — Engine deformer function and source libraries reused by all graphs

*nice-to-have* · assets: all 17 deformer graphs

The graphs author no custom deform math of their own. They reference engine function assets under /DeformerGraph/DeformerFunctions/: DG_Function_Bend, DG_Function_Twist, DG_Function_SquashStretch and DG_Function_ComputeNormalsTangentsAndKeepInputNormals. Kernels include HLSL source libraries: DSL_Matrix for Bend/Twist/Squash and DSL_Quaternion for normals.

The same engine folder also ships these functions, which the rigs do not use:
- DG_Function_Flare, DG_Function_LatticeDeform, DG_Function_BlendPositions
- the LBS and DQS skinning functions
- ComputeNormalsTangents and ...KeepImportedNormals
The base deformer DG_LinearBlendSkin_Morph_Cloth comes from the same plugin.

The only custom kernel is the CacheGeometry copy.

**Setup.** n/a

**Operators:** `OptimusNode_FunctionReference`, `OptimusSource (AdditionalSources)`

**Scale:** 4 functions and 2 source libraries.

**Evidence:** `<UE>/Plugins/Animation/DeformerGraph/Content/DeformerFunctions/ (DG_Function_Bend.uasset, DG_Function_Twist.uasset, DG_Function_SquashStretch.uasset, ...)`; `<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraHeadBend_DeformerGraph/asset.t3d: 'AdditionalSources(0)="/Script/OptimusCore.OptimusSource'/DeformerGraph/SourceLibraries/DSL_Quaternion.DSL_Quaternion'"'`

#### UE8-array-union-clone — ArrayUnion / ArrayClone used to build combined element lists

*nice-to-have* · assets: CRM_FN_Foot, CRM_FN_Spine, CRM_FN_IkFk2Bones, CRM_FN_BipedStretchFeedback

ArrayClone copies an array so a member variable is not changed in place. ArrayUnion(io Array, Other) rebuilds Array as its unique elements (deduplicated by value hash, first occurrence kept) and then appends each element of Other not already present. Foot construction: Clone(member controls), Union(Make_Array[...]), Union(another member list), then feed 'Get Chain Length' and 'Set Control Scale'.Controls, so every foot control gets the same size scale exactly once. Spine does the same in construction. IkFk2Bones: Clone(list) then Append, then Union(Make_Array) for the control list in construction and utility graphs; Clone also inside 'Compute FK'. StretchFeedback: 2 Clones in construction.

**Setup.** none

**Operators:** `DISPATCH_RigVMDispatch_ArrayUnion`, `DISPATCH_RigVMDispatch_ArrayClone`, `DISPATCH_RigVMDispatch_ArrayAppend`

**Scale:** 5 Union + 10 Clone nodes

**Evidence:** `<UE>/Plugins/Runtime/RigVM/Source/RigVM/Private/RigVMFunctions/RigVMDispatch_Array.cpp:1272-1340`; `<UE>/Plugins/Runtime/RigVM/Source/RigVM/Public/RigVMFunctions/RigVMDispatch_Array.h:422`; `<dump>/FortniteRigs__Modules__Biped__Foot__CRM_FN_Foot/graphs.txt:487-493, 852-871`; `<dump>/FortniteRigs__Modules__IkSolves__IkFk2Bones__CRM_FN_IkFk2Bones/graphs.txt:698-721, 1303-1314`; `<dump>/units_used.json :: 'DISPATCH_RigVMDispatch_ArrayUnion': 5, 'DISPATCH_RigVMDispatch_ArrayClone': 10`

#### UE8-crfl-blend-twist-dead — CRFL_Module 'Blend Twist' (with the axis-angle path) is unreferenced; LimbTwist uses its own copy

*nice-to-have* · assets: CRFL_Module_v001, CRM_FN_LimbTwist

All 'FUNC Blend Twist' references point to CRM_FN_LimbTwist's local library, so the CRFL_Module version is dead content. Inside that version, the only graph using MathQuaternionToAxisAndAngle/FromAxisAndAngle is: MakeRelative(driver local, driver initial local), then SwingTwist(axis), then twist, then (axis, angle), then angle x (-1) x (weight_i x flip mult), then FromAxisAndAngle, then SetRotation. This path has no execution input, so it is dead even there. The active path in the CRFL copy is: twist = CRFL_Math 'Get Node Twist Value'(Driver, TwistAxis); for each driven i, rot_i = Slerp(identity, Reverse ? twist : twist^-1, w_i); SetRotation(local), then OffsetTransformForItem with the initial local rotation. The LimbTwist local copy that rigs actually run differs: q = Reverse ? twist : twist^-1; rot_i = Reverse ? Slerp(identity, q, w_i) : Slerp(q, identity, w_i). A port must implement the LimbTwist variant; the axis-angle units appear only in dead code.

**Setup.** LimbTwist config: Reverse, TwistAxis, Debug Axis

**Operators:** `RigVMFunction_MathQuaternionToAxisAndAngle (dead)`, `RigVMFunction_MathQuaternionFromAxisAndAngle (dead)`, `RigVMFunction_MathQuaternionSlerp`, `RigVMFunction_MathQuaternionInverse`, `RigUnit_OffsetTransformForItem`, `FUNC Get Node Twist Value`

**Scale:** 0 references to the CRFL version; 2 per LimbTwist module to the local version

**Evidence:** `<dump>/FortniteRigs__Libraries__CRFL_Module_v001/graphs.txt:20-156 (Blend Twist), links 'RigVMFunction_MathQuaternionToAxisAndAngle.Angle -> Multiply_1.A' and no link into For_Each.ExecuteContext`; `<dump>/FortniteRigs__Modules__Biped__LimbTwist__CRM_FN_LimbTwist/graphs.txt:40, 331 (LibraryNodePath=...CRM_FN_LimbTwist...:RigVMFunctionLibrary.Blend Twist), 435-534 (local version: If/Interpolate/Interpolate_1 links)`; `<dump>/units_used.json :: 'RigVMFunction_MathQuaternionToAxisAndAngle': 1 (CRFL_Module_v001 only)`

#### UE8-local-shape-and-name-helpers — Module-local 'Get Control Shape' and 'Get Control Name' helpers (FkChain, FkArray, Prop)

*nice-to-have* · assets: CRM_FN_FkChain, CRM_FN_FkArray, CRM_FN_Prop

Get Control Shape returns FRigUnit_HierarchyAddControl_ShapeSettings with bVisible=Shape.bVisible and Transform = Shape.Transform * inverse(Transform Offset), which cancels the control offset so the gizmo stays where it was authored. FkChain/FkArray set Name = 'Get Control Shape Name From Item v02'(Item=Bone, Default Shape=Shape.Name, ShapeLib Namespace = the 'CRSL Namespace' variable, Direct Mesh Control Libraries = module metadata 'Direct Mesh Control Libraries' in the Root namespace) and Color=Shape.Color. The Prop version uses 'Get Control Shape Name From Item v01'(Item.Name = Bone Name) and Color = 'Control Color Override From Metadata v01'(Shape.Color). Get Control Name (defined in FkChain and FkArray, used only by FkArray) returns: if FK Standard Names, then (Count > 1 ? 'FK ' + IntToName(Index) : 'FK'); otherwise Conform Name v01(Name). This produces control names like 'Clavicle L/FK' and 'Index L/FK 0'.

**Setup.** FK Standard Names (bool), shape settings and transform offset per module

**Operators:** `RigVMFunction_MathTransformInverse`, `RigVMFunction_MathTransformMul`, `RigVMFunction_MathIntToName`, `RigVMFunction_NameConcat`, `DISPATCH_RigDispatch_GetModuleMetadata`, `FUNC Get Control Shape Name From Item v01/v02`, `FUNC Conform Name v01`

**Scale:** FkChain x9, FkArray x6, Prop x1 in MR_Zebra

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkChain/graphs.txt:418-443, 444-470`; `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkArray/graphs.txt:708-736, 737`; `<dump>/FortniteRigs__Modules__FkSolves__Prop__CRM_FN_Prop/graphs.txt:682-706`; `<dump>/FortniteRigs__Modules__FkSolves__Fk__CRM_FN_FkChain/summary.json :: functions_used lacks 'Get Control Name'`

### D18 Runtime integration

#### UE-postprocess-abp — Post-process Anim Blueprint running a Control Rig runtime asset

*core* · assets: AnimBP_Zebra, Monster_PostAnimBP

The AnimGraph is: Input Pose (LinkedInputPose) -> Control Rig node (ControlRigAssetReference = CR_*_Deform) -> Output Pose. Everything else is an engine default:
- Alpha=1, AlphaInputType=Float, bAlphaBoolEnabled=true, no AlphaCurveName.
- LODThreshold=-1, so it runs at all LODs.
- bSetRefPoseFromSkeleton=false.
- bResetInputPoseToInitial=true, bTransferInputPose=true, bTransferInputCurves=true.
- bTransferPoseInGlobalSpace depends on the performance cvar.
- No I/O bone filters and no variable mappings.
The event graph is disabled. Output bones and curves from the rig, including the corrective curves, go back into the pose. Because it is a post-process ABP, it runs after the main animation instance (animation, sequencer Control Rig track or retargeter) on every evaluation of the mesh.

**Setup.** None.

**Operators:** `AnimGraphNode_ControlRig`, `FAnimNode_ControlRig`, `AnimGraphNode_LinkedInputPose`

**Scale:** 2 ABPs, 1 Control Rig node each

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__AnimBP_Zebra/asset.t3d:67 Node=(ControlRigAssetReference=(...CR_Zebra_Deform...))`; `<dump>/Game__Assets__Zebra__Rig__AnimBP_Zebra/asset.t3d:62,91,95,102 LinkedTo=(AnimGraphNode_ControlRig_1 ...)/(AnimGraphNode_LinkedInputPose_0 ...)`; `<dump>/Game__Assets__Monster__Rig__Monster_PostAnimBP/asset.t3d:74 Node=(ControlRigAssetReference=(...CR_Monster_Deform...))`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/AnimNode_ControlRigBase.cpp:33-46 (defaults)`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/AnimNode_ControlRig.cpp:25-35 (Alpha=1, LODThreshold=INDEX_NONE)`

#### UE-runtime-layering — Runtime evaluation layering: animator rig -> post-process correctives -> morphs -> GPU deformers

*core* · assets: MR_Zebra, AnimBP_Zebra, CR_Zebra_Deform, SKM_Zebra, zebra_audition

Per frame:
1. The main pose comes from a baked AnimSequence, the retargeter, or a Level Sequence MovieSceneControlRigParameterTrack evaluating MR_Zebra, which includes the face module that writes face and deformer curves.
2. The post-process ABP runs CR_Zebra_Deform. It recomputes the 50 body corrective curves from the final pose (overwriting any baked values), adds helper and twist-bone offsets, and enqueues 7 GPU deformers.
3. Skinning uses the modified helper bones, and morph targets take their weights from the same-named curves (body correctives and face shapes).
4. Optimus deformers run after default skinning, in group-1 add order.
The animator never sees the correctives. They are a pure function of pose, so a reimplementation should evaluate them after all animator rigs.

**Setup.** None.

**Operators:** `FAnimNode_ControlRig`, `RigUnit_AddOptimusDeformer`, `MovieSceneControlRigParameterTrack`

**Scale:** all Zebra playback paths

**Evidence:** `<dump>/Game__Sequences__zebra_audition/sequence.txt BINDING SKM Zebra ... TRACK MovieSceneControlRigParameterTrack name=MR_Zebra`; `<dump>/Game__Assets__Zebra__Meshes__SKM_Zebra/summary.json post_process_anim_blueprint`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:644-646 (Sequence A correctives, B deformers)`

#### UE7-zebra-deform-postprocess-chain — Zebra: deformers driven from the post-process control rig (curve-driven, works with baked anim)

*core* · assets: CR_Zebra_Deform, AnimBP_Zebra, SKM_Zebra, SKM_Zebra_Hi, Zeb_Face_Expressions

SKM_Zebra and SKM_Zebra_Hi use post-process AnimBP_Zebra, which runs a ControlRig anim node with CR_Zebra_Deform. Its Forwards Solve runs the pose-reader correctives first (Sequence A), then the 7-deformer chain (Sequence B).

Curve-to-trait wiring:
- GetCurveValue(head_squash) -> Remap(-1..1 -> 0..1) -> StretchFactor
- head_twist -> TwistFactor
- head_bend -> BendFactor
- muzzle_squash_deformer -> Remap(1..-1 -> 0..1) -> StretchFactor
- muzzle_bend_deformer -> BendFactor
- skull_tp_squash_deformer -> Remap(-1..1 -> 0..1) -> StretchFactor
- skull_tp_bend_deformer -> BendFactor
All remaps use bClamp=False.

Because the drivers are plain animation curves, any animation carrying them drives the deformers at runtime without the authoring rig. Zeb_Face_Expressions contains head_bend, head_squash, muzzle_bend_deformer, muzzle_squash_deformer, skull_tp_bend_deformer and skull_tp_squash_deformer. Each deformer's Transform = GetTransform(<Null var>, GlobalSpace, bInitial=False).

**Setup.** The curves are written by the Zebra face module (see UE7-deformer-control-channel-mapping) or by the animation.

**Operators:** `RigUnit_GetCurveValue`, `RigVMFunction_MathDoubleRemap`, `RigUnit_GetTransform`, `RigUnit_AddOptimusDeformer`, `RigVMFunction_Sequence`, `AnimNode_ControlRig (post-process)`

**Scale:** 1 rig, 7 deformers, 7 curves, 3 remaps.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:256-290 (AddOptimusDeformer*, GetCurveValue Curve=head_squash/head_twist/head_bend/..., Remap_7/8/9)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:644-677 (links)`; `<dump>/Game__Assets__Zebra__Rig__AnimBP_Zebra/asset.t3d:67 'ControlRigAsset=...CR_Zebra_Deform'`; `<dump>/Game__Assets__Zebra__Meshes__SKM_Zebra/summary.json 'post_process_anim_blueprint': ...AnimBP_Zebra_C`; `<dump>/Game__Assets__Zebra__Anims__Zeb_Face_Expressions/summary.json float_curves`

#### UE8-seq-cr-channel-layout — Control Rig Sequencer section: per-control channel layout (Euler, not quaternion)

*core* · assets: zebra_audition, MR_Zebra_Take1, zebra_marketingPoseFaces, MR_Zebra, MR_Boombox

Each MovieSceneControlRigParameterSection builds channels only for animatable controls, and the channel type follows the control type. Float and ScaleFloat use a scalar FMovieSceneFloatChannel. Bool uses FMovieSceneBoolChannel. Integer with a ControlEnum uses an enum channel (FMovieSceneByteChannel, uint8); integer without an enum uses FMovieSceneIntegerChannel. Vector2D uses X/Y float curves. Position, Scale and Rotator use X/Y/Z float curves. Transform, TransformNoScale and EulerTransform use FTransformParameterNameAndCurves: 9 FMovieSceneFloatChannels (Translation[3], Rotation[3] as Euler degrees, Scale[3]). There are no quaternion channels. ControlChannelMap maps each control name to FChannelMapInfo {ControlIndex, TotalChannelIndex, ChannelIndex, ParentControlIndex, ChannelTypeName, bDoesHaveSpace, SpaceChannelIndex, MaskIndex, CategoryIndex, ConstraintsIndex[]}. The track stores ControlsRotationOrder: a map from control name to FControlRotationOrder {EEulerRotationOrder RotationOrder, bOverrideSetting}. The Rotation[3] Euler values are read in that order; ChangeControlRotationOrder re-keys the values through quaternions. The saved sequences use rotation orders XYZ, XZY, YXZ, YZX and ZYX, consistent with the rig's preferred_rotation_order counts (MR_Zebra: 261 YZX, 82 XYZ, 8 YXZ, 6 ZYX, 2 XZY). MR_Zebra has 221 EULER_TRANSFORM animation controls, 4 proxy controls, 14 ROTATOR, 4 POSITION, 32 FLOAT channels, 14 SCALE_FLOAT channels, 69 BOOL channels and 1 INTEGER (enum) channel, so a port needs all of: float, bool, byte/enum, vector3 and 9-channel Euler transform curves. The name tables of all three sequences contain Bool/Enum/Scalar/Vector/TransformParameterNamesAndCurves with XCurve/YCurve/ZCurve. None contains IntegerParameterNamesAndCurves or a Vector2D curve name. Which controls actually carry keys is not recoverable from the name table: FChannelMapInfo lists every animatable control.

**Setup.** Sequencer groups channels by module and display name, for example 'Leg L / UpperLeg FK', 'Arm L / Elbow', 'Thumb L / Base', 'Leg Upper Twist R / Offset 1'. Channel keys use the full control FName, for example 'Arm L/FK 0'.

**Operators:** `UMovieSceneControlRigParameterSection`, `UMovieSceneControlRigParameterTrack`, `FChannelMapInfo`, `FControlRotationOrder`, `FTransformParameterNameAndCurves`, `FMovieSceneFloatChannel`, `FMovieSceneBoolChannel`, `FMovieSceneByteChannel`

**Scale:** 4 Control Rig tracks in 3 sequences (MR_Zebra x3, MR_Boombox x1), 1 section each; MR_Zebra exposes 359 controls

**Evidence:** `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Sequencer/MovieSceneControlRigParameterSection.h:181-214 (FChannelMapInfo), 328-340 (ControlChannelMap, Enum/Integer curves)`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Sequencer/MovieSceneControlRigParameterSection.cpp:3380-3486 (Float/ScaleFloat->AddScalarParameter, Bool, Integer+ControlEnum->AddEnumParameter, Vector2D, Position/Scale/Rotator->AddVectorParameter, Transform/EulerTransform->AddTransformParameter)`; `<UE>/Source/Runtime/MovieSceneTracks/Public/Sections/MovieSceneParameterSection.h:310-331 (Translation[3], Rotation[3], Scale[3])`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Sequencer/MovieSceneControlRigParameterTrack.h:24-37, 264-265 (ControlsRotationOrder)`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Sequencer/MovieSceneControlRigParameterSection.cpp:3540-3614 (ChangeControlRotationOrder via QuatFromEuler/EulerFromQuat)`; `<ZebraSample>/Content/Sequences/zebra_audition.uasset name table: 'ControlChannelMap','ChannelMapInfo','ControlsRotationOrder','EEulerRotationOrder::YZX','TransformParameterNamesAndCurves','EnumParameterNamesAndCurves','MovieSceneByteChannel'`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt :: grep -o 'control_type=[A-Z_]* animation_type=[A-Z_]*' | sort | uniq -c`

#### UE-ikrig-zebra — IK Rig: full-body IK solver, goals, bone settings and retarget chains

*important* · assets: IK_Zebra

Solver stack contents:
- One IKRigFullBodyIKSolver: RootBone=pelvis, Iterations=20, SubIterations=10, MassMultiplier=1, bAllowStretch=False, RootBehavior=Free, PrePull RotationAlpha=0 and PositionAlpha=1, GlobalPullChainAlpha=0, MaxAngle=30, OverRelaxation=1.3.
- Goals RightArm_Goal (hand_r), LeftArm_Goal (hand_l), LeftLeg_Goal (foot_l) and RightLeg_Goal (foot_r), each with ChainDepth=2.
- Bone settings: clavicle_l/r and pelvis RotationStiffness=0.95. lowerarm_l/r and calf_l/r lock X and Y (a hinge about Z) with PreferredAngles Z=90.

Retarget definition: PelvisBone=pelvis and 14 chains (Spine spine_01-spine_05, Neck neck_01-head, Right/LeftClav, Right/LeftArm upperarm-hand with IK goals, Right/Left Pinky, Index and Thumb, Left/RightLeg thigh-foot with IK goals). Goal initial transforms are stored in component space.

**Setup.** Goal names follow <Side><Limb>_Goal.

**Operators:** `IKRigFullBodyIKSolver`, `IKRigEffectorGoal`

**Scale:** 1 solver, 4 goals, 7 bone settings, 14 chains

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__IK_Zebra/asset.t3d:40 SolverStack(0)=/Script/IKRig.IKRigFullBodyIKSolver(Settings=(RootBone="pelvis",Iterations=20,...))`; `<dump>/Game__Assets__Zebra__Rig__IK_Zebra/asset.t3d:11-30 GoalName=RightArm_Goal BoneName=hand_r ...`; `<dump>/Game__Assets__Zebra__Rig__IK_Zebra/asset.t3d:41 RetargetDefinition=(PelvisBone="pelvis",BoneChains=(...))`

#### UE-level-sequences — Sequencer Control Rig tracks and shot setup

*important* · assets: MR_Zebra_Take1, zebra_audition, zebra_marketingPoseFaces, expression_demo_seq, MRG_All, MRG_expression_demo_graph

Characters are keyed through MovieSceneControlRigParameterTrack, which holds the animator control channels:
- MR_Zebra_Take1: binding MR_Zebra with Transform, SkeletalAnimation (0 sections) and CR track MR_Zebra.
- zebra_audition: binding 'SKM Zebra' with Spawn, Transform, empty SkeletalAnimation and CR track MR_Zebra. MR_Boombox has a CR track MR_Boombox. The shot also has cine camera, light and skylight tracks.
- zebra_marketingPoseFaces: binding Zebra with CR track MR_Zebra.
- expression_demo_seq: camera, eye-material tracks and Subsequences.
The MovieRenderGraph assets are render configs (render layer, sampling, warm-up and output nodes) only. The dump does not include per-channel CR key data.

**Setup.** The CR track exposes MR_Zebra's controls as sequencer channels. The deform rig has no controls, so it never appears in sequencer.

**Operators:** `MovieSceneControlRigParameterTrack`, `MovieSceneSkeletalAnimationTrack`, `MovieScene3DTransformTrack`, `MovieSceneSpawnTrack`

**Scale:** 4 level sequences, 4 CR tracks

**Evidence:** `<dump>/Game__MR_Zebra_Take1/sequence.txt TRACK MovieSceneControlRigParameterTrack name=MR_Zebra sections=1`; `<dump>/Game__Sequences__zebra_audition/sequence.txt BINDING MR_Boombox ... TRACK MovieSceneControlRigParameterTrack name=MR_Boombox`; `<dump>/Game__Sequences__zebra_marketingPoseFaces/sequence.txt TRACK MovieSceneControlRigParameterTrack name=MR_Zebra`; `<dump>/Game__Sequences__MovieRenderGraph__MRG_expression_demo_graph/asset.t3d MovieGraphRenderLayerNode`

#### UE-monster-usd-export — UE->USD export of Monster (UsdSkel with blendshapes)

*important* · assets: SKM_Monster

The export Monster.usd contains:
- /SKM_Monster: SkelRoot with SkelBindingAPI.
- /SKM_Monster/Skel: Skeleton with 165 joints using full-path joint tokens (root, root/pelvis, ...).
- /SKM_Monster/SKM_Monster: Mesh with SkelBindingAPI, 79 BlendShape prims and blendShapes bound with the same names as the morph targets, and 12 joint influences per vertex.
- 5 GeomSubsets, Section0-4, each with an UnrealMaterial containing UnrealShader, SurfaceShader, PrimvarReader and texture shader nodes.
There is no SkelAnimation, so no animation was exported. The rig logic is not exported: no correctives, no deform rig, and no Optimus deformers.

**Setup.** None.

**Operators:** `UsdSkelRoot`, `UsdSkelSkeleton`, `UsdSkelBlendShape`, `UsdGeomSubset`

**Scale:** 1 USD file, 165 joints, 79 blendshapes

**Evidence:** `<dump>/Game__Assets__Monster__Meshes__SKM_Monster/monster_usd_tree.txt TYPES {'SkelRoot': 1, 'Skeleton': 1, 'Mesh': 1, 'GeomSubset': 5, 'Material': 5, 'Shader': 36, 'BlendShape': 79}`; `<dump>/Game__Assets__Monster__Meshes__SKM_Monster/monster_usd_tree.txt BINDING /SKM_Monster/SKM_Monster blendShapes=79 ... joints/influences: 12`; `<dump>/Game__Assets__Monster__Meshes__SKM_Monster/monster_usd_tree.txt ANIMS []`

#### UE-retargeter-ops — IK Retargeter op stack (UEFN mannequin -> Zebra)

*important* · assets: RTG_UEFN_to_Zebra, IK_Zebra, PA_Zebra

The op stack runs in this order:
0. Pelvis Motion: pelvis to pelvis, translation and rotation alpha 1, ScaleHorizontal/Vertical 1, AffectIKHorizontal 1, AffectIKVertical 0.
1. FK Chains: 14 target chains mapped by name; RightClav/LeftClav map to source RightClavicle/LeftClavicle.
2. Run IK Rig: IK_Zebra on the 4 limb chains. Its chain map oddly maps RightClav->RightLeg and LeftClav->LeftLeg.
3. Blend to Source (child of Run IK Rig): arms and legs, alpha 1.
4. Body Intersect IK: physics override PA_Zebra, goal intersect for both arm goals, intersect bodies pelvis, spine_02, spine_04, spine_05, head, jaw, skull_tp, thighs and calves, and pole-vector intersect on lowerarm_l/r.
5. Offset Goals (child of Run IK Rig): alpha 1.
6. Root Motion: CopyFromSourceRoot and CopyHeightFromSource, source pelvis to target root, bMaintainOffsetFromPelvis, propagate to non-retargeted children.
7. Remap Curves: bCopyAllSourceCurves=True.
8. Filter Bones: neck_01, neck_02 and head, with Responsiveness 0.5, CutoffFrequency 1.5, VelocityCutoff 20.
TargetMeshOffset is X=85.39. The target retarget pose stores rotation offsets for the legs, arms, fingers, spine_01-04 and pelvis.

**Setup.** None.

**Operators:** `IKRetargetPelvisMotionOp`, `IKRetargetFKChainsOp`, `IKRetargetRunIKRigOp`, `IKRetargetBlendToSourceOp`, `IKRetargetBodyIntersectIKOp`, `IKRetargetOffsetGoalsOp`, `IKRetargetRootMotionOp`, `IKRetargetCurveRemapOp`, `IKRetargetFilterBoneOp`

**Scale:** 9 ops, 14 chains

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__RTG_UEFN_to_Zebra/asset.t3d RetargetOps(0)=/Script/IKRig.IKRetargetPelvisMotionOp ... RetargetOps(8)=/Script/IKRig.IKRetargetFilterBoneOp`; `<dump>/Game__Assets__Zebra__Rig__RTG_UEFN_to_Zebra/asset.t3d RetargetOps(3)=/Script/BodyIntersectIKOp.IKRetargetBodyIntersectIKOp(Settings=(TargetPhysicsAssetOverride=...PA_Zebra...`; `<dump>/Game__Assets__Zebra__Rig__RTG_UEFN_to_Zebra/asset.t3d TargetRetargetPoses=(("Default Pose", (BoneRotationOffsets=(("calf_r", ...`

#### UE-skm-runtime-config — Skeletal mesh runtime wiring (PP ABP, physics, deformer, LOD, sockets)

*important* · assets: SKM_Zebra, SKM_Zebra_Hi, SKM_Monster

Each skeletal mesh names its post_process_anim_blueprint (AnimBP_Zebra or Monster_PostAnimBP) and its physics_asset (SKM_Zebra_Physics or SKM_Monster_Physics). The meshes set default_mesh_deformer=None and target_mesh_deformers=None, so GPU deformation is only switched on by the rig's Add Deformer unit. Each mesh has 1 LOD and no skin weight profiles or clothing.
- SKM_Zebra: 34,459 verts, 371 bones, 8 materials (Mohawk, Teeth, Gums, Clothing, Body, Nails, Lashes, Eyes).
- SKM_Zebra_Hi: 132,333 verts, same skeleton and ABP.
- SKM_Monster: 107,867 verts, 165 bones, 5 materials.
The Zebra meshes carry 8 sockets on ball_l/r.

**Setup.** None.

**Scale:** 3 meshes

**Evidence:** `<dump>/Game__Assets__Zebra__Meshes__SKM_Zebra/summary.json post_process_anim_blueprint AnimBP_Zebra_C; physics_asset SKM_Zebra_Physics; default_mesh_deformer None; lods 1; num_verts_lod0 34459`; `<dump>/Game__Assets__Zebra__Meshes__SKM_Zebra_Hi/summary.json num_verts_lod0 132333`; `<dump>/Game__Assets__Monster__Meshes__SKM_Monster/summary.json num_verts_lod0 107867; bone_count 165`

#### UE2-stateful-interaction-and-autokey — Interaction-dependent and stateful solves (editor semantics)

*important* · assets: CRM_FN_Body, CRM_FN_Prop, CRM_FN_Spine, CRM_FN_Root

Several solves rely on host interaction queries, events and persistent state, not only on the current pose:
- RigUnit_IsInteracting (bIsRotating, bIsTranslating, Items) in the Movable Proxy and in Drive Aim and Body Rotation.
- RigUnit_SendEvent 'RequestAutoKey': Body aim/twist sync (bOnlyDuringInteraction=true); Prop pivot release (bOnlyDuringInteraction=false).
- Transform and bool variables carried across evaluations:
  - Body: 'Aim Rotate Buffer Quat', 'Aim Buffer Transform'
  - Prop: 'Prop Global/Local Pivot Previous', 'Local/Prop Control World Transform', 'Previous Global/Local/Prop Buffer Transform'
  - Root: 'Global Control Snapped', 'Global Control Transform'
  - Proxy: 'IsSet' element metadata
- RigUnit_SetControlOffset and RigUnit_SetControlColor are called at runtime, in the Spine and Prop forward solves.

A re-implementation needs per-instance persistent state, an interaction or manipulation signal, and an auto-key callback.

**Operators:** `RigUnit_IsInteracting`, `RigUnit_SendEvent`, `RigUnit_SetControlOffset`, `RigUnit_SetControlColor`, `RigVMVariableNode setters`

**Scale:** 4 modules

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Body__CRM_FN_Body/graphs.txt:587,616,618`; `<dump>/FortniteRigs__Modules__FkSolves__Prop__CRM_FN_Prop/graphs.txt (Forwards) 'SendEvent_1 ... Event=RequestAutoKey ... bOnlyDuringInteraction=false'`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/Execution/RigUnit_IsInteracting.h:11-15`

#### UE6-zebra-deformer-curve-interface — Zebra face to CR_Zebra_Deform curve interface for Optimus deformers

*important* · assets: CRM_Zebra_Face, CR_Zebra_Deform, AnimBP_Zebra

The Zebra face adds no deformers. It writes the curves head_squash, head_twist, head_bend, muzzle_squash_deformer, muzzle_bend_deformer, skull_tp_squash_deformer and skull_tp_bend_deformer. CR_Zebra_Deform, the runtime post-process rig run by AnimBP_Zebra, reads them and adds 7 deformers: ZebraHead Stretch = remap(head_squash, -1..1 -> 0..1); ZebraHeadTwist Twist = head_twist; ZebraHeadBend Bend = head_bend; ZebraMuzzleSquash Stretch = remap(muzzle_squash_deformer, 1..-1 -> 0..1); ZebraMuzzleBend Bend = muzzle_bend_deformer; ZebraSkullTpSquash Stretch = remap(skull_tp_squash_deformer, -1..1 -> 0..1); ZebraSkullTpBend Bend = skull_tp_bend_deformer. Their transforms come from that rig's own *_Null variables. ZebraMuzzleTwist exists as an asset but is not added.

**Setup.** Same squash controls as UE6-squash-controls-curves.

**Operators:** `RigUnit_SetCurveValue`, `RigUnit_GetCurveValue`, `RigUnit_AddOptimusDeformer`, `AnimGraphNode_ControlRig`

**Scale:** 7 curves -> 7 deformers

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:258-286 GetCurveValue head_squash/head_twist/head_bend/muzzle_squash_deformer...`; `<dump>/Game__Assets__Zebra__Rig__AnimBP_Zebra/asset.t3d ControlRigAsset=CR_Zebra_Deform`

#### UE7-dmc-editor-only-scope — DMC is editor-only and experimental (no runtime or cooked equivalent)

*important* · assets: DirectMeshControl plugin, CRM_FN_DMC, RunDMC, MR_ZebraDMC, MR_FN_BipedDMC

Where DMC can run:
- Both plugin modules are 'Type': 'Editor', and the plugin is Experimental and not enabled by default. The ZebraSample .uproject enables it.
- Mesh-description access, the generation subsystem (UEditorSubsystem) and the modeling tool are editor-only.
- The rig unit returns early outside the Construction event or when OnAddShapeLibraryDelegate is unbound.

Consequences:
- DMC changes only the animator's picking surface and gizmo display. It never changes the pose or deformation.
- DMC-shaped controls animate exactly like the same controls in the non-DMC template, which is why the DMC templates are otherwise identical.
- Side effect while active: the bound source mesh is forced to LOD0 and to use a mesh deformer.

**Setup.** n/a

**Operators:** `DirectMeshControl.uplugin`

**Scale:** n/a

**Evidence:** `<UE>/Plugins/Experimental/Animation/DirectMeshControl/DirectMeshControl.uplugin ('EnabledByDefault' : false, 'IsExperimentalVersion': true, Modules Type 'Editor')`; `<ZebraSample>/ZebraSample.uproject: '"Name": "DirectMeshControl", "Enabled": true'`; `<UE>/Plugins/Experimental/Animation/DirectMeshControl/Source/DirectMeshControlRig/Private/Units/RigUnit_DirectMeshControl.cpp:59-73`

#### UE8-dump-gaps-runtime-assets — Extraction gaps: runtime-asset rigs, truncated variable defaults, curve metadata, vector curves

*important* · assets: CRM_Zebra_Face, CRM_Monster_Face, CR_Zebra_Deform, CR_Monster_Deform, RunDMC, CRM_FN_DMC, MR_Boombox, Zeb_Face_Expressions, all skeletal meshes

For the 7 ControlRigRuntimeAsset rigs, is_control_rig_module, get_preview_mesh, rig_module_settings, modular_rig_model and create_control_rig all failed, so summary.json has is_module/preview_mesh/rig_module_settings = null and there is no runtime hierarchy. The workarounds are: preview meshes are in regen.py set_preview_mesh (face and deform rigs: SKM_Zebra / SKM_Monster; RunDMC: None). Connector settings are in hierarchy_export.txt (Face: Root default primary; Parent Secondary + bOptional + ChildOfPrimary rule). CRM_FN_DMC and RunDMC ExposedConnectors are in asset.t3d. Member variables were captured through the get_asset_variables fallback (88 in the Zebra face module, 108 in the Monster face module, 8 in CR_Zebra_Deform), but dump.py truncates each variable record to 500 characters. 69 records across 12 assets are cut off, including 7 in each face module (for example 'Soft Eyes Dn' TArray<FQuat> defaults, and the Lid* shape and rotation arrays) and many module config transforms and shape settings. regen.py add_member_variable calls carry no defaults, and the face modules have no asset.t3d (over 1.5 MB), so full defaults for those arrays are currently unavailable. ConfigOverrides export as '()' for every module; the real values are in the editor module instances inside asset.t3d. Skeleton curve_meta_data reads failed 5 times because the property was replaced by UAnimCurveMetaData user data. AnimSequence vector curves were not dumped (RCT_VECTOR missing), so Zeb_Face_Expressions 'vector_curves': 0 is unreliable (102 float curves, 163 frames, 371 bone tracks). get_notation failed on RigVMInvokeEntryNode in Foot and IkFk2Bones. The log begins at MR_Zebra_Take1; earlier lines are not in _log.txt.

**Setup.** none

**Scale:** 7 runtime assets; 69 truncated variable records

**Evidence:** `<dump>/_log.txt:2-8, 24-37, 52-65, 84-90, 126-132 (ERR ismod/pm/rms/mv, mrm, inst)`; `<dump>/_log.txt:43 (ERR vc), 114, 125 (ERR not2), 138 (ERR pm CRU_PropAim)`; `<dump>.py:25-31 (S() truncation), 204-208 (mv -> av fallback, S(v, 500))`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/summary.json :: '...<trunc 896>' ('Soft Eyes Dn')`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/regen.py:1-7 (add_connector with default settings, set_preview_mesh SKM_Zebra), :53 (add_member_variable without default)`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/hierarchy_export.txt:1`; `<dump>/Game__Assets__Zebra__Anims__Zeb_Face_Expressions/summary.json`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/ControlRigRuntimeAsset.h:140-150`

#### UE8-project-plugins — Required plugins (including transitive DeformerGraph)

*important* · assets: ZebraSample.uproject, FortniteRigs.uplugin

Explicitly enabled: SkeletalMeshMorphTargetEditingTools (experimental), AnimatorKit (beta), MovieRenderPipeline, GeometryCacheLevelSequenceBaker, MediaViewer, AlembicHairImporter, DirectMeshControl (experimental), RelativeIKOp (experimental), Chooser, BlendStack, AnimationWarping, PoseSearch, AnimationLocomotionLibrary, Locomotor, MotionWarping, Mover, DrawDebugLibrary, NetworkPrediction, SmartObjects, CurveExpression (experimental), GameplayInteractions, GameFeatures, RigMapperOp (experimental), PerformanceCaptureWorkflow (beta). Solaris is disabled. DeformerGraph (Optimus, beta, EnabledByDefault=false) is not listed; it is enabled through the dependencies of AnimatorKit, DirectMeshControl and GeometryCacheLevelSequenceBaker, and all the *DeformerGraph assets need it. ControlRig, ControlRigSpline (beta), ControlRigModules (beta, provides AddControl) and IKRig are enabled by default. FortniteRigs is a content-only project plugin (CanContainContent, no Modules, no Plugins dependency list) mounted at /FortniteRigs. Nothing redirects its former /EpicControlRig mount.

**Setup.** none

**Scale:** project-wide

**Evidence:** `<ZebraSample>/ZebraSample.uproject :: Plugins list`; `<ZebraSample>/Plugins/FortniteRigs/FortniteRigs.uplugin`; `<UE>/Plugins/Animation/DeformerGraph/DeformerGraph.uplugin :: "EnabledByDefault" : false`; `<UE>/Plugins/Animation/AnimatorKit/AnimatorKit.uplugin :: Plugins includes DeformerGraph`; `<UE>/Plugins/Experimental/Animation/DirectMeshControl/DirectMeshControl.uplugin :: Plugins includes DeformerGraph, ComputeFramework, SkeletalMeshModelingTools`; `<UE>/Plugins/Animation/ControlRigModules/ControlRigModules.uplugin :: EnabledByDefault true, IsBetaVersion true`

#### UE8-rtg-broken — RTG_UEFN_to_Zebra retargeter cannot run as shipped

*important* · assets: RTG_UEFN_to_Zebra, IK_Zebra, PA_Zebra

The retargeter has no SourceIKRigAsset property (so None), and both FK Chains and Run IK Rig ops have ChainMapping SourceIKRig=None. SourcePreviewMesh /Game/Characters/UEFN_Mannequin/Meshes/SKM_UEFN_Mannequin is not in the project. TargetIKRigAsset=IK_Zebra; TargetPreviewMesh is missing; TargetMeshOffset X=85.39. Op stack in order: (0) Pelvis Motion (pelvis to pelvis). (1) FK Chains (IK_Zebra; 14 target chains; mapping RightClav->RightClavicle, LeftClav->LeftClavicle, others by same name). (2) Blend to Source (Right/Left Arm and Leg, alpha 1, parent Run IK Rig). (3) Body Intersect Goals (TargetPhysicsAssetOverride PA_Zebra, goals LeftArm_Goal/RightArm_Goal, pole-vector intersect on lowerarm_l/r, parent Run IK Rig). (4) Offset Goals (4 limbs, parent Run IK Rig). (5) Run IK Rig (4 limb chains). Its own ChainMapping maps RightClav->RightLeg and LeftClav->LeftLeg, a copy error that differs from op 1; it is only harmless if the IK op ignores clavicle chains. (6) Root Motion (source pelvis, target root, CopyFromSourceRoot, CopyHeightFromSource, bMaintainOffsetFromPelvis, bPropagateToNonRetargetedChildren). (7) Remap Curves (bCopyAllSourceCurves, no explicit remaps). (8) Filter Bones on neck_01, neck_02, head (Responsiveness 0.5, CutoffFrequency 1.5, VelocityCutoffFrequency 20, bResetPlayback).

**Setup.** none (retarget pipeline)

**Operators:** `UIKRetargeter`, `IKRetargetPelvisMotionOp`, `IKRetargetFKChainsOp`, `IKRetargetBlendToSourceOp`, `IKRetargetBodyIntersectIKOp`, `IKRetargetOffsetGoalsOp`, `IKRetargetRunIKRigOp`, `IKRetargetRootMotionOp`, `IKRetargetCurveRemapOp`, `IKRetargetFilterBoneOp`

**Scale:** 1 retargeter, 9 ops

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__RTG_UEFN_to_Zebra/asset.t3d:14-17 (SourcePreviewMesh, TargetIKRigAsset, TargetPreviewMesh, TargetMeshOffset); grep -c SourceIKRigAsset = 0`; `<dump>/Game__Assets__Zebra__Rig__RTG_UEFN_to_Zebra/asset.t3d:20 ('(TargetChainName="RightClav",SourceChainName="RightClavicle")' ... 'SourceIKRig=None')`; `<dump>/Game__Assets__Zebra__Rig__RTG_UEFN_to_Zebra/asset.t3d:24 ('(TargetChainName="RightClav",SourceChainName="RightLeg")', '(TargetChainName="LeftClav",SourceChainName="LeftLeg")')`; `<dump>/Game__Assets__Zebra__Rig__RTG_UEFN_to_Zebra/asset.t3d:19-29 (op list)`; `<dump>/asset_index.json (no /Game/Characters entries)`

#### UE8-seq-blend-weight-mask — Control Rig section blend mode, weight and masks as saved

*important* · assets: zebra_audition, MR_Zebra_Take1, zebra_marketingPoseFaces

UE saves only non-default tagged properties. FOptionalMovieSceneBlendType defaults to BlendType=Absolute with bIsValid=false. All three sequences store 'BlendType'/'OptionalMovieSceneBlendType'/'bIsValid', but none contains an 'EMovieSceneBlendType' name, so every section in them is Absolute (not Additive, Override, Relative or AdditiveFromBase). The section constructor sets TransformMask=AllTransform and Weight.SetDefault(1.0). The name tables contain no 'Weight' (apart from control names), 'ControlNameMask', 'TransformMask', 'ControlsMask', 'PlayRate' or 'OverrideAssets' property. So every section is weight 1.0 with no weight keys, has no masked controls and keeps all transform channels enabled. The UAnimLayers asset user data (Base/Additive/Override layers with weights) is present as 'AnimLayers', but each Control Rig track has only one section, so no layered sections exist. To port: treat each section as a single absolute layer with weight 1.

**Setup.** none (section-level settings)

**Operators:** `FOptionalMovieSceneBlendType`, `EMovieSceneBlendType`, `FMovieSceneTransformMask`, `UAnimLayers`

**Scale:** 4 sections, all Absolute, weight 1

**Evidence:** `<UE>/Source/Runtime/MovieScene/Public/Evaluation/Blending/MovieSceneBlendType.h:18-31, 39-46 (default Absolute, bIsValid=false)`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Private/Sequencer/MovieSceneControlRigParameterSection.cpp:929-935 (TransformMask=AllTransform; Weight.SetDefault(1.0f))`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Sequencer/MovieSceneControlRigParameterSection.h:312-326, 350-357 (ControlsMask, ControlNameMask, TransformMask, Weight, OverrideAssets, PlayRate)`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRigEditor/Private/Sequencer/AnimLayers/AnimLayers.h:134-175, 334 (EAnimLayerType Base/Additive/Override; UAnimLayers : UAssetUserData)`; `<dump>/Game__Sequences__zebra_audition/sequence.txt :: 'MovieSceneControlRigParameterTrack name=MR_Zebra sections=1'`; `name-table scan of zebra_audition / zebra_marketingPoseFaces / MR_Zebra_Take1: 'OptionalMovieSceneBlendType' present, 'EMovieSceneBlendType' absent`

#### UE8-seq-embedded-rig-snapshot — Sequences embed a serialized rig instance that can be older than the asset

*important* · assets: zebra_audition, MR_Boombox, MR_Zebra

Each Control Rig track and section keeps a UControlRig/UModularRig object (UPROPERTY ControlRig) plus ControlRigAssetReference, ControlRigSettingsOverrides and PriorityOrder. The zebra_audition name table therefore contains a copy of the modular model: ModularRigModel, ConnectionList, ModularRigSingleConnection, ConfigOverrides, ControlRigOverrideContainer, VariableBindings, ShapeLibraries, DynamicHierarchy, and override paths 'Root Module Settings->ControlSize_12_1266…', 'Module Settings->ControlSize_12_…', 'Control Settings->Shape'. This MR_Boombox snapshot has connectors for AddControl..AddControl_4 but none for AddControl_5/_6, and no tape1_socket/tape2_socket, yet it has control channels AddControl_5/tape1_ctrl and AddControl_6/tape2_ctrl. The current asset has 15 connections, including tape sockets. The snapshot is stale relative to the asset. A USD port should rebuild the rig from the asset and match section channels to controls by full control name ('Module/Control', FName case-insensitive). It should not trust the embedded rig copy.

**Setup.** none

**Operators:** `UMovieSceneControlRigParameterTrack::ControlRig`, `FControlRigAssetStrongReference`, `FControlRigOverrideContainer`

**Scale:** 1 stale snapshot confirmed (MR_Boombox); MR_Zebra snapshots not checked in detail

**Evidence:** `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Sequencer/MovieSceneControlRigParameterTrack.h:247-273`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Sequencer/MovieSceneControlRigParameterSection.h:298-310`; `zebra_audition.uasset name table: 'ModularRigModel','ConnectionList','Root Module Settings->ControlSize_12_1266EFC440EAC0A0D120DC82EC6819EA','Control Settings->Shape','AddControl_4/Parent Control','AddControl_5/tape1_ctrl' (no 'AddControl_5/Add Control Primary')`; `<dump>/Game__Assets__Environment__Rig__MR_Boombox/modular_rig_model.txt :: 'AddControl_5/Add Control Primary' -> 'AddControl_1/tape1_socket'`

#### UE-physics-assets — Physics assets (capsule/convex bodies with limited constraints)

*nice-to-have* · assets: SKM_Zebra_Physics, PA_Zebra, PA_Zebra_Phys_Asset_Detailed, SKM_Monster_Physics

- SKM_Zebra_Physics (the mesh default): 22 SkeletalBodySetups with capsule (sphyl) geometry on pelvis, spine_02/04/05, head, skull, skull_tp, jaw, ear_base_l/r, eye_main_l/r, upper and lower arms, hands, thighs and calves, plus 21 constraints.
- PA_Zebra: 21 capsule bodies (no skull) and 20 constraints, all with Swing1/Swing2/Twist ACM_Limited; the retargeter uses it for body intersection.
- PA_Zebra_Phys_Asset_Detailed: 27 convex bodies (adds clavicles, feet, neck_01/02, spine_01/03, ear_01/02) and 26 constraints, not referenced by any mesh.
- SKM_Monster_Physics: 1 capsule 'spine_05_capsule' (radius 37.0, length 22.4).

**Setup.** None.

**Operators:** `PhysicsAsset`, `SkeletalBodySetup`, `PhysicsConstraintTemplate`

**Scale:** 4 physics assets, 71 bodies total

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__SKM_Zebra_Physics/asset.t3d:152-168 AggGeom=(SphylElems=...) BoneName="pelvis"/"skull_tp"/"jaw"`; `<dump>/Game__Assets__Zebra__Rig__PA_Zebra/asset.t3d:85-118 DefaultInstance=(JointName=... ConeLimit=(Swing1Motion=ACM_Limited ...`; `<dump>/Game__Assets__Zebra__Rig__PA_Zebra_Phys_Asset_Detailed/asset.t3d AggGeom=(ConvexElems (27)`; `<dump>/Game__Assets__Monster__Rig__SKM_Monster_Physics/asset.t3d:5 Name="spine_05_capsule"`

#### UE7-deformer-child-components — Deformer propagation to child skeletal mesh components

*nice-to-have* · assets: CR_Zebra_Deform, CRM_Monster_Face

With DeformChildComponents=True (all 16 nodes), the same deformer instance guid is enqueued and added on every child USkeletalMeshComponent of the rig's mesh component (recursive GetChildrenComponents). Each child gets its own Optimus instance, and the variable values are pushed to each one. A child's own skin weights drive its mask; the mask uses bone names, so the child needs a matching skeleton. No component tags exclude children (ExcludeChildComponentsWithTag=None). This lets attached meshes such as hair or clothing follow the same bend/twist/squash.

**Setup.** None.

**Operators:** `RigUnit_AddOptimusDeformer (GetComponentsToProcess)`

**Scale:** 16 nodes.

**Evidence:** `<UE>/Plugins/Animation/DeformerGraph/Source/OptimusCore/Private/ControlRig/RigUnit_Optimus.cpp:271-305`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:256 'DeformChildComponents=True,ExcludeChildComponentsWithTag="None"'`

#### UE8-unused-missing-assets — Unused assets and missing references (deformer, skeleton, preview meshes)

*nice-to-have* · assets: ZebraMuzzleTwist_DeformerGraph, CR_Zebra_Deform, SK_ZebraHi, SK_Zebra, PA_Zebra_Phys_Asset_Detailed, IK_Zebra, RTG_UEFN_to_Zebra

(1) CR_Zebra_Deform has 7 AddOptimusDeformer nodes: ZebraHead, ZebraHeadTwist, ZebraHeadBend, ZebraMuzzleSquash, ZebraMuzzleBend, ZebraSkullTpSquash, ZebraSkullTpBend. The 8th graph, ZebraMuzzleTwist_DeformerGraph, is unused; no muzzle_twist curve or node references it. (2) SK_ZebraHi duplicates SK_Zebra (same sockets, VirtualBoneGuid 1B0B9536…, BoneTree 370) but no mesh uses it: SKM_Zebra and SKM_Zebra_Hi both use SK_Zebra. SK_ZebraHi also has no preview mesh. (3) SK_Zebra.PreviewSkeletalMesh and PA_Zebra_Phys_Asset_Detailed.PreviewSkeletalMesh = /Game/Assets/Zebra/Geo/Test/Zebra, which is missing; there is no Content/Assets/Zebra/Geo folder. (4) IK_Zebra.PreviewSkeletalMesh and RTG TargetPreviewMesh = /Game/Assets/Zebra/Geo/Zebra_transfered3, also missing. None of these affects rig evaluation, but an asset-graph importer must tolerate unresolved soft references.

**Setup.** none

**Operators:** `RigUnit_AddOptimusDeformer`

**Scale:** 1 unused deformer, 1 unused skeleton, 4 dangling preview references

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:256-283 (7 'DeformerGraph=' entries)`; `<dump>/asset_index.json :: '/Game/Assets/Zebra/Rig/Deformers/ZebraMuzzleTwist_DeformerGraph'`; `<dump>/Game__Assets__Zebra__Meshes__SKM_Zebra_Hi/summary.json :: 'skeleton': '/Game/Assets/Zebra/Meshes/SK_Zebra.SK_Zebra'`; `<dump>/Game__Assets__Zebra__Meshes__SK_ZebraHi/asset.t3d :: 'VirtualBoneGuid=1B0B95364210D9FEA49448AE45A6E174' (same as SK_Zebra), no PreviewSkeletalMesh`; `<dump>/Game__Assets__Zebra__Meshes__SK_Zebra/asset.t3d:70`; `<dump>/Game__Assets__Zebra__Rig__PA_Zebra_Phys_Asset_Detailed/asset.t3d:345`; `<dump>/Game__Assets__Zebra__Rig__IK_Zebra/asset.t3d:34`; `ls <ZebraSample>/Content/Assets/Zebra -> Anims Icons Materials Meshes Rig (no Geo)`

### D19 Debug visualization

#### UE-debug-draw — Pose-reader and deformer debug visualization settings

*nice-to-have* · assets: CR_Zebra_Deform, CR_Monster_Deform, Zebra deformer graphs, RTG_UEFN_to_Zebra

The Spherical Pose Reader can draw its inner cone (green) and outer cone (yellow), connector lines, and the driver vector (red when the output is 0, blue otherwise). It draws in 3D or as a flat 2D projection, with scale 25, 20 segments and thickness 0.25. Every reader here sets bDrawDebug=false. Each deformer graph includes an OptimusDebugDrawDataInterface; the squash kernel can draw its capture axis, limit planes and bulge plane when bDebugDraw is set. The retargeter ops enable bDebugDraw on Pelvis Motion, FK Chains, Offset Goals, Root Motion, Remap Curves and Filter Bones.

**Setup.** None.

**Operators:** `FSphericalPoseReaderDebugSettings`, `OptimusDebugDrawDataInterface`

**Scale:** 49 readers (all off), 8 deformer graphs

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/regen.py:76 SphericalPoseReader_1_1_1_1_1_1_1_1.Debug '(bDrawDebug=false,bDraw2D=false,DebugScale=25.000000,DebugSegments=20,...)'`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/Highlevel/Hierarchy/RigUnit_SphericalPoseReader.h:72-275`; `<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraHead_DeformerGraph/kernels.hlsl:108-137 (if (bDebugDraw) ... DrawPlane)`

#### UE2-root-control-path-debug — Control path debug line on Global/Local

*nice-to-have* · assets: CRM_FN_Root

Construction ('Set Up Control Path', called for both Global and Local) spawns two channels:
- bool 'Control Path Vis' (default false)
- float 'Control Path Distance' (default 50, min 0 with limit, max 500 without limit)

Forwards ('Generate Control Path'): for control C with channel values d and vis, draw a foreground line strip with lifetime -1:
- points = [C.globalTranslation, C.global.TransformPosition((0, d, 0))], i.e. a segment of length d along the control's local +Y axis
- thickness = min(d, 1)
- color = C's shape color
- enabled = vis

**Setup.** The channels appear under root/Global ('Control Path Vis', 'Control Path Distance') and under root/Local (runtime names '..._2').

**Operators:** `RigUnit_HierarchyAddAnimationChannelFloat`, `RigUnit_HierarchyAddAnimationChannelBool`, `RigVMFunction_DebugLineStripNoSpace`, `RigVMFunction_MathTransformTransformVector`, `RigVMFunction_MathDoubleMin`, `RigUnit_HierarchyGetShapeSettings`

**Scale:** 2 per rig (4 channels)

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Root__CRM_FN_Root/graphs.txt:335-367 'Generate Control Path'`; `<dump>/FortniteRigs__Modules__FkSolves__Root__CRM_FN_Root/graphs.txt:375-376 'Name=Control Path Distance; InitialValue=50.000000'`

#### UE3-ik-debug-draw — IK debug drawing (PV line, virtual-bone axes, soft IK arcs)

*nice-to-have* · assets: CRM_FN_IkFk2Bones

Always, unless (root 'Direct Mesh Control' && DMC Found):
- DebugLine from the Mid control to the PV control, color (0.2,0.2,0.2), lifetime -1.

When Debug is on:
- DebugTransformArray axes (scale 10) for the virtual bones and the Auto PV parent (red).
- Soft IK arcs.
- Compute Pole Vector Parent axes (the construction call uses Debug = true).
- Pole-vector debug vectors.

DMC Found = 'ik-layer' is in root 'Direct Mesh Control Libraries'.

**Setup.** The 'Debug' module option.

**Operators:** `RigVMFunction_DebugLineNoSpace`, `RigVMFunction_DebugTransformArrayMutableNoSpace`, `RigVMFunction_DebugTransformMutableNoSpace`, `RigVMFunction_DebugArcNoSpace`, `RigVMFunction_DebugRectangleNoSpace`

**Scale:** per limb

**Evidence:** `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:71 'Draw Line | ... Color=(R=0.2,G=0.2,B=0.2' + 598-602`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:1083 'Find | Element=ik-layer' + 1654-1655`; `<dump>/.../CRM_FN_IkFk2Bones/graphs.txt:145,168 debug transform draws`

#### UE3-stretch-feedback — Squash/stretch feedback lines

*nice-to-have* · assets: CRM_FN_BipedStretchFeedback

Construction:
- Spine Elements = resolve 'Spine Elements'.
- Arm L = resolve 'Arm Elements'; Arm R = the Arm L keys with names renamed '_l' to '_r' (CollectionReplaceItemsArray, keep invalid, no duplicates).
- Legs are handled the same way.
- If 'Vis Channel Control' is connected (root/Global in the biped), spawn bool channel 'Stretch FeedBack Vis' (initial False) under it.

Forward: Stretch Feedback(Elements, Enabled = channel) for spine, arm L/R, leg L/R. For i >= 1:
- r = |p_i - p_{i-1}|_current / |p_i - p_{i-1}|_initial.
- color = |r - 1| <= 0.01 ? RestColor (green) : (r > 1 ? lerp(StartColor (white), StretchColor (red), (r - 1)/(MaxStretch - 1), unclamped) : lerp(SquashColor (blue), StartColor, r)).
- DebugLine(p_{i-1}, p_i, color, Line Thickness).

Defaults: Max Stretch Factor 5 (biped override 2), Line Thickness 1. It is purely visual and does not write to the rig. Element lists include the twist bones, e.g. upperarm_l, twist_01, twist_02, lowerarm_l, ...

**Setup.** 'Stretch FeedBack Vis' toggle on root/Global.

**Operators:** `RigUnit_CollectionReplaceItemsArray`, `RigUnit_ResolveArrayConnector`, `RigUnit_HierarchyAddAnimationChannelBool`, `RigVMFunction_MathVectorDistance`, `RigVMFunction_MathColorLerp`, `RigVMFunction_MathDoubleRemap`, `RigVMFunction_MathDoubleIsNearlyEqual`, `RigVMFunction_DebugLineNoSpace`

**Scale:** 1 module, 5 chains (biped templates only)

**Evidence:** `<dump>/FortniteRigs__Modules__Biped__StretchFeedback__CRM_FN_BipedStretchFeedback/graphs.txt:116-167 ConstructionGraph ('Replace Items | Old=_l; New=_r', 'Name=Stretch FeedBack Vis')`; `<dump>/.../CRM_FN_BipedStretchFeedback/graphs.txt:197-290 'Stretch Feedback' function`; `<dump>/.../CRM_FN_BipedStretchFeedback/summary.json variables (Max Stretch Factor 5, colors)`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/modular_rig_model.txt 'Stretch Feedback/Arm Elements'`

#### UE4-draw-axis — Draw Axis

*nice-to-have* · assets: CRFL_Debug_v001, CRM_FN_LimbTwist, CRFL_Module_v001

Inputs: Item (RigElementKey[]), Scale (default 10), Thickness (default 0.2) and Enable (default true). If Enable, it draws an axis tripod at each item's current global transform (DebugTransformMutableNoSpace, Mode=Axes, foreground depth, persistent lifetime -1, identity world offset). The graph comment documents the arguments as Items, Scale, Thickness and Active.

**Setup.** Exposed through the LimbTwist 'Debug Axis' module option.

**Operators:** `RigUnit_GetTransform (global)`, `RigVMFunction_DebugTransformMutableNoSpace`, `RigVMFunction_ControlFlowBranch`, `DISPATCH_RigVMDispatch_ArrayIterator`

**Scale:** 2 call sites.

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Debug_v001/graphs.txt:11-46 'Draw Axis' (DebugTransformMutableItemSpace 'Mode=Axes')`; `UE_5.8/Engine/Plugins/Runtime/RigVM/Source/RigVM/Public/RigVMFunctions/Debug/RigVMFunction_DebugTransform.h:28`; `ue/<dump>/FortniteRigs__Modules__Biped__LimbTwist__CRM_FN_LimbTwist/graphs.txt:455 'Draw Axis | Scale=10.000000; Thickness=0.200000' (Enable <- Debug Axis)`

#### UE4-embedded-debug-draw — Debug drawing embedded in IK/pole-vector helpers

*nice-to-have* · assets: CRFL_Math_v001, CRFL_Hierarchy_v001

Project Middle Bone to IK Plane draws a red line from start to the projected mid (thickness 0.2) and a red rectangle at the projected transform when Debug is on. Compute Pole Vector Location v02 draws the current pole vector (green) and the reference pole vector (blue) from the origin when Debug is on, scaled by PV Offset. Compute Pole Vector From Plane v01 has one VisualDebugVector node hard-enabled (red plane normal). CRFL_Math Compute Pole Vector draws axes (scale 20) when Draw is on.

**Operators:** `RigVMFunction_DebugLineNoSpace`, `RigVMFunction_DebugRectangleNoSpace`, `RigVMFunction_VisualDebugVectorNoSpace`, `RigVMFunction_DebugTransformMutableNoSpace`

**Scale:** Active in IkFk2Bones when its 'Debug' variable is set.

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Math_v001/graphs.txt:327 'Draw Line ... Thickness=0.200000', :329 'Draw Rectangle ... Thickness=2.000000'`; `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:1719 'Visual Debug Vector ... Color=(R=0,G=0,B=1)', :1728 'Color=(R=0,G=1,B=0)'`; `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:1346 'Visual Debug Vector ... bEnabled=True'`; `UE_5.8/Engine/Plugins/Runtime/RigVM/Source/RigVM/Public/RigVMFunctions/Debug/RigVMFunction_VisualDebug.h:69`

#### UE4-rigvm-debug-draw — RigVM debug-draw functions

*nice-to-have* · assets: CRFL_Debug_v001, CRFL_Math_v001, CRFL_Hierarchy_v001

DebugTransformMutableNoSpace(Transform, Mode Axes/Point/Box, Color, Thickness, Scale, WorldOffset, bEnabled) draws a transform. DebugLineNoSpace(A, B, Color, Thickness) draws a line. DebugRectangleNoSpace(Transform, Color, Scale, Thickness) draws a rectangle. VisualDebugVectorNoSpace(Value, Mode Vector, Color, Thickness, Scale, bEnabled) draws a vector from the origin and passes Value through, so it can sit inline in math chains. All use DebugDrawSettings (depth priority, lifetime).

**Operators:** `RigVMFunction_DebugTransformMutableNoSpace`, `RigVMFunction_DebugLineNoSpace`, `RigVMFunction_DebugRectangleNoSpace`, `RigVMFunction_VisualDebugVectorNoSpace`

**Scale:** VisualDebugVector 4, DebugTransform 2, DebugLine 1, DebugRectangle 1.

**Evidence:** `UE_5.8/Engine/Plugins/Runtime/RigVM/Source/RigVM/Public/RigVMFunctions/Debug/RigVMFunction_DebugTransform.h:28`; `UE_5.8/Engine/Plugins/Runtime/RigVM/Source/RigVM/Public/RigVMFunctions/Debug/RigVMFunction_DebugLine.h:14`; `UE_5.8/Engine/Plugins/Runtime/RigVM/Source/RigVM/Public/RigVMFunctions/Debug/RigVMFunction_DebugPrimitives.h:63`; `UE_5.8/Engine/Plugins/Runtime/RigVM/Source/RigVM/Public/RigVMFunctions/Debug/RigVMFunction_VisualDebug.h:69`; `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:1365-1366 (VisualDebugVector_1.Value passes through to Cross)`

#### UE7-dg-debug-draw — Deformer debug drawing (GPU debug draw DI)

*nice-to-have* · assets: all 17 deformer graphs

OptimusDebugDrawDataInterface is present with bForceEnable=True, but the kernels only draw when Optional_EnableDebugDraw != 0, and only from vertex index 0 (thread 0).

What gets drawn (via AddLine and AddQuad):
- Bend: the capture line, 30-unit quads at Lower and Upper, and a red bend-axis line and green bend-direction line (length 100) at Lower.
- Twist: the lower plane and an upper plane rotated by Radian, with red/green axis lines of length 30.
- Squash: the lower plane, the stretched upper plane, and a bulge plane scaled by BulgeRatio at the biased mid point.

In every rig graph EnableDebugDraw is a constant 0, so nothing is drawn.

**Setup.** None; the switch is a baked constant.

**Operators:** `OptimusDebugDrawDataInterface (ReadDebugDraw().AddLine/AddQuad)`

**Scale:** 17 graphs.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraHeadBend_DeformerGraph/asset.t3d:955 'DebugDrawParameters=(bForceEnable=True)'`; `<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraHeadBend_DeformerGraph/kernels.hlsl:111-135,159`; `<UE>/Plugins/Animation/DeformerGraph/Source/OptimusCore/Private/DataInterfaces/OptimusDataInterfaceDebugDraw.h:26-39`

### D20 Procedural construction

#### UE-deform-runtime-asset — Control-less post-process deform rig built at construction from the skeleton

*core* · assets: CR_Zebra_Deform, CR_Monster_Deform

CR_Zebra_Deform is a ControlRigRuntimeAsset with no controls. Its construction event runs HierarchyImportFromSkeleton (NameSpace=None, bIncludeCurves=true, bIncludeMeshSockets=False, bIncludeVirtualBones=True), which pulls every bone and skeleton curve into the rig hierarchy, including the deformer-driver curves the face rig writes. The asset also stores 50 CURVE elements statically, one per output corrective curve, so the curve setters always resolve. It then spawns 7 nulls under 'head' (see UE-deformer-pivot-nulls). Only a Forward Solve event exists, with no Backward Solve and no user events. CR_Monster_Deform has no construction graph; its hierarchy (165 bones, 74 curves) is stored statically.

**Setup.** No animator-facing controls. The rig is driven purely by the incoming animation pose and curves.

**Operators:** `RigUnit_PrepareForExecution`, `RigUnit_HierarchyImportFromSkeleton`, `RigUnit_HierarchyAddNull`, `RigUnit_BeginExecution`

**Scale:** 2 deform rigs; Zebra: 50 static curves + imported 371 bones; Monster: 165 bones/74 curves static

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:681 HierarchyImportFromSkeleton bIncludeCurves=true; bIncludeVirtualBones=True`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:696-697 PrepareForExecution.ExecutePin -> HierarchyImportFromSkeleton -> HierarchyAddNull`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/summary.json hierarchy_counts CURVE 50; class ControlRigRuntimeAsset`; `<dump>/Game__Assets__Monster__Rig__CR_Monster_Deform/summary.json hierarchy_counts BONE 165 CURVE 74`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/Execution/RigUnit_DynamicHierarchy.h:354-366 (ImportFromSkeleton flags)`

#### UE1-static-hierarchy-import — Static hierarchy assembled from the skeletal mesh: imported bones, curves and mesh sockets as nulls

*core* · assets: MR_Zebra, MR_ZebraDMC, MR_FN_Biped, MR_Monster, CRU_PropAim, MR_Boombox

The authored (static) hierarchy of each modular rig holds only skeleton data plus the module connectors:
- all skeleton bones as BoneType=Imported with the reference pose
- all skeleton anim curves as CURVE elements (value 0)
- skeletal-mesh sockets as NULL elements parented to the socket bone, carrying the socket's local offset, tag 'MeshSocket' and metadata 'Tags' (NAME_ARRAY)
- one CONNECTOR per exposed connector of each module

Zebra: 371 bones, 958 curves, 8 nulls (foot_[l|r]_inner/outer/heel/toe_tip on ball_[l|r]), 149 connectors. Biped (SKM_Manny): 106 bones, 800 curves, 8 nulls, 192 connectors. Monster: 165 bones, 993 curves, no sockets, 26 connectors. PropAim: 1 bone (root). Boombox: 8 bones plus 1 user Socket element 'handle_socket' (metadata SocketDesiredParent).

The foot socket nulls are what the Foot module's pivot connectors target.

**Operators:** `URigHierarchyController::AddBone(ERigBoneType::Imported)`, `URigHierarchyController::AddCurve`, `URigHierarchyController::AddNull`, `URigHierarchyController::AddConnector`, `URigHierarchyController::AddSocket`

**Scale:** Zebra: 1486 static elements. Biped: 1106. Monster: 1184.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/regen.py:1146 "add_null('foot_l_heel', unreal.RigElementKey(type=unreal.RigElementType.BONE, name='ball_l'), unreal.Transform(location=[-18.799999,1.000000,-1.000000]"`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/hierarchy.txt:1146 "NULL foot_l_heel parents=['BONE:ball_l'] ... tags=['MeshSocket']"`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/hierarchy_export.txt: 958x add_curve (regen.py) and 'BoneType=Imported' x371`; `<dump>/Game__Assets__Zebra__Meshes__SKM_Zebra/summary.json: 'sockets': 8, 'morph_targets': 132, 'bone_count': 371`; `<dump>/Game__Assets__Environment__Rig__MR_Boombox/hierarchy.txt: "SOCKET handle_socket parents=['BONE:handle'] ... metadata=['SocketDesiredParent:RIG_ELEMENT_KEY']"`

#### UE2-construction-spawn-pattern — Construction-event procedural spawning pattern

*core* · assets: CRM_FN_Root, CRM_FN_Body, CRM_FN_Spine, CRM_FN_FkChain, CRM_FN_FkArray, CRM_FN_Prop, CRM_FN_Pin

All of these modules build their rig procedurally in the Construction event (RigUnit_PrepareForExecution). The static hierarchy contains only connectors and one socket.

Pattern, run as a Sequence of steps:
1. Reset the array variables.
2. Resolve connectors into variables.
3. Spawn nulls, controls, bones and channels (HierarchyAdd*) with GlobalSpace or LocalSpace offsets computed from INITIAL bone transforms.
4. Store returned element keys in variables or arrays for the solve events.
5. Attach metadata (Offset, FkDeltaTransform, Body Delta Transform, Bone Percentage, Color, IsSet).
6. Set channel hosts and available spaces.
7. Apply the control scale.

In a modular rig, spawned element names are namespaced '<Module>/<Name>'. Duplicate short names get a '_2' suffix (e.g. 'Control Path Vis_2', 'Aim Weight_3').

**Operators:** `RigUnit_PrepareForExecution`, `RigUnit_HierarchyAddControlTransform`, `RigUnit_HierarchyAddNull`, `RigUnit_HierarchyAddBone`, `RigUnit_HierarchyAddAnimationChannelBool`, `RigUnit_HierarchyAddAnimationChannelFloat`, `RigUnit_HierarchyAddAnimationChannelScaleFloat`, `RigUnit_HierarchyAddAnimationChannelInteger`, `RigVMFunction_Sequence`, `DISPATCH_RigVMDispatch_ArrayReset`

**Scale:** 7 modules; the Spine construction graph alone has 439 nodes

**Evidence:** `<dump>/FortniteRigs__Modules__FkSolves__Root__CRM_FN_Root/hierarchy.txt (only CONNECTOR/SOCKET)`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/summary.json 'hierarchy_counts': CONNECTOR 7, SOCKET 1`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt 'root/Control Path Vis_2'`

#### UE4-unit-dynamic-hierarchy-spawn — Engine units: dynamic hierarchy spawning (construction event)

*core* · assets: CRFL_Control_v001, CRFL_Hierarchy_v001

RigUnit_HierarchyAddNull(Parent, Name, Transform, Space Local/Global) creates a null. RigUnit_HierarchyAddBone does the same for bones. RigUnit_HierarchyAddControlTransform, AddControlRotator and AddControlVector(Parent, Name, OffsetTransform, OffsetSpace, InitialValue, Settings) create controls; the settings cover shape (visible, name, colour, transform), limits, proxy/driven list, preferred rotation order, display name, bIsPosition for vectors and InitialSpace. RigUnit_HierarchyAddAnimationChannelBool(Parent, Name, Initial, Min, Max) creates a bool channel control; names may not contain ':' or '/'. RigUnit_HierarchyGetShapeSettings reads a control's shape settings. All are valid only during construction (IsValidToRunInContext) and return the new element key.

**Operators:** `RigUnit_HierarchyAddNull`, `RigUnit_HierarchyAddBone`, `RigUnit_HierarchyAddControlTransform`, `RigUnit_HierarchyAddControlRotator`, `RigUnit_HierarchyAddControlVector`, `RigUnit_HierarchyAddAnimationChannelBool`, `RigUnit_HierarchyGetShapeSettings`

**Scale:** CRFL_Hierarchy: AddNull 8, AddBone 6, AddControlTransform 3, AddControlVector 1, AddAnimationChannelBool 1. CRFL_Control: AddNull 1, AddControlRotator 1, AddAnimationChannelBool 1.

**Evidence:** `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/Execution/RigUnit_DynamicHierarchy.h:478 (AddNull), :444 (AddBone), :1213 (AddControlTransform), :1079 (AddControlRotator), :968 (AddControlVector), :1254 (AddAnimationChannelBool), :1692 (GetShapeSettings)`; `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Execution/RigUnit_DynamicHierarchy.cpp:1099-1129`

#### UE6-lid-spawn — Spawn Lid Bones and Controls (procedural lid rig bones + micro controls)

*core* · assets: CRM_Zebra_Face, CRM_Monster_Face

Spawn Lid Bones and Controls(Lid Bones[], Control Shape[] vectors, Spawn Bones[] out-array var, Is Right, Parent Bone). Neg = Is Right ? -1 : +1. The local shape color is R = IsRight, G = 0, B = 1-IsRight (red for right, blue for left). The function resets Spawn Bones, then for each input bone name: newName = Rename Joint to Control(name); spawn bone newName under Parent Bone with local identity (so it pivots at the eye_main center); append it to Spawn Bones; spawn control newName under that bone with identity offset, shape Default, scale 0.05 (Monster 0.03), shape translation = ControlShape[i]*Neg. There are 8 calls: L/R x {lid_tp_01..03, lid_tp_base_01..03, lid_bt_base_01..03, lid_bt_01..03}, with parent eye_main_l or eye_main_r. The shape vectors are the 'Lid Micro * Control Shape L' variables: Zebra Tp (11,6,3),(12,1,6),(10,-6,5); Monster (7,3,1),(7,.5,2),(7,-2,1). The R side reuses the L vectors, negated.

**Setup.** 24 micro controls: Lid Tp/Tp Base/Bt/Bt Base 01-03 L/R, hidden by default (Micro Vis).

**Operators:** `RigUnit_HierarchyAddBone`, `RigUnit_HierarchyAddControlTransform`, `RigVMFunction_NameReplace`, `RigVMDispatch_ArrayReset`, `RigVMDispatch_ArrayAdd`, `RigVMFunction_MathBoolToFloat`, `RigVMFunction_MathVectorMul`, `RigVMFunction_ControlFlowBranch`

**Scale:** 8 calls, 24 bones, 24 controls

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:2214-2287 Spawn Lid Bones and Controls`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:151 Spawn Lid Bones and Controls Is Right=false Parent Bone=eye_main_l`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt:1120 BONE Face/Lid Tp 01 L par=B:eye_main_l`

#### UE-deformer-pivot-nulls — Deformer frame nulls spawned under head at construction

*important* · assets: CR_Zebra_Deform

The construction event spawns 7 nulls, all parented to bone 'head' with Space=GlobalSpace, so the transform is the initial global pose and the null follows the head afterwards. Each resulting key is stored in a private variable.
- Head Squash Null: identity rotation, T(0,0,80).
- Head Bend Null: 90 deg about Z, T(0,0,90).
- Head Twist Null: identity, T(0,0,80).
- Muzzle Squash Null: quat(0,-1,0,0), i.e. 180 deg about Y so +Z points down, T(0,0,110).
- Muzzle Bend Null: quat(-0.7071,-0.7071,0,0), T(0,20,110).
- Skull Tp Squash Null: identity, T(0,0,100).
- Skull Tp Bend Null: 90 deg about Z, T(0,0,100).
The deformer kernels deform along the frame's local +Z starting at the frame origin, so these nulls define each deformer's capture axis and pivot.

**Setup.** Null names contain spaces. Variables: 'Head Squash Null', 'Head Bend Null', 'Head Twist Null', 'Muzzle Squash Null', 'Muzzle bend Null', 'Skull Tp Squash Null', 'Skull Tp Bend Null'.

**Operators:** `RigUnit_HierarchyAddNull`, `RigVMVariableNode`

**Scale:** 7 nulls

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:682-695 (HierarchyAddNull.._6 Parent=(Type=Bone,Name="head") ... Space=GlobalSpace; VariableNode setters)`; `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:698-717 (Item -> variable links)`; `<UE>/Plugins/Animation/ControlRig/Source/ControlRig/Public/Units/Execution/RigUnit_DynamicHierarchy.h:478-505`; `<dump>/Game__Assets__Zebra__Rig__Deformers__ZebraHead_DeformerGraph/kernels.hlsl:62-66 (ReadOriginTransform; deform along local z)`

#### UE2-spine-virtual-bones-percentages — Spline helper elements: virtual bones, reoriented/match nulls, bone percentages

*important* · assets: CRM_FN_Spine

Construction spawns:
- A BONE chain '<bone>_virtual' (the first under the parent of Bones[0], each next under the previous one) at the current global of each bone. These are the fit-chain targets.
- A NULL chain '<bone>_reoriented' with Transform (bone T, bone R * Sec FKs Orient Offset, S). The transform is given in LocalSpace even though the values are global; forwards overwrites them anyway.
- A NULL chain '<bone>_match' under the parent of Bones[0] at the Sec FK offset, used by the backwards match.

Float metadata 'Bone Percentage' on each bone = |Bones[i].initT - Bones[0].initT| / SplineOrigLength. This is a straight-line distance from the first bone, not arc length.

**Operators:** `RigUnit_HierarchyAddBone`, `RigUnit_HierarchyAddNull`, `RigVMFunction_NameConcat`, `RigVMFunction_MathVectorDistance`, `RigVMFunction_MathDoubleDiv`, `DISPATCH_RigDispatch_SetMetadata`

**Scale:** Spine: 6 virtual bones, 6 reoriented and 6 match nulls. Neck: 3 each.

**Evidence:** `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:580 'B=_virtual'; :657 'B=_reoriented'; :606 'B=_match'`; `<dump>/FortniteRigs__Modules__Biped__Spine__CRM_FN_Spine/graphs.txt:257 'Set Float Metadata' (Bone Percentage)`; `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/runtime_hierarchy.txt 'BONE Spine/pelvis_virtual parents=['BONE:root']'`

#### UE6-construction-order — Face construction event ordering

*important* · assets: CRM_Zebra_Face, CRM_Monster_Face

The Construction event (PrepareForExecution) runs a Sequence. Zebra first sets DMC Found. Pin A: Head Attach Null (head global), Head Squash, Skull Tp Squash, Muzzle Squash, (Monster: Mouth Squash), jaw, Jaw Attributes, Jaw Const, Muzzle, Mouth, Nose (plus Monster nose channels), Lips Tp, Lips Bt, Reverse Jaw, Skull Const, Skull Tp, Cheek L/R (plus Monster squint). Pin B: corners and sneer (plus Monster sticky), brows (main, tweaker vis, mid null/control, in/out bones/controls), Squeeze Null/Squeeze, Teeth, tongue loop, reset of the lip arrays, lip tweaker loop. Pin C: 8 lid spawns plus Lid In/Ot L/R. Pin E (Monster: D): Eye Main L/R with Micro Vis, Lid Tp/Bt sliders, eye nulls, aim nulls, Eye L/R with Pupil/Iris, SetChannelHosts, Eye Aim, Convergence, SetDefaultParent x3, Add Null Above. Monster pin E: 9 deformer nulls. Controls are addressed by name across modules, and names are case-insensitive: 'Skull'/'Jaw'/'Muzzle' match bones skull/jaw/muzzle.

**Setup.** None.

**Operators:** `RigUnit_PrepareForExecution`, `RigVMFunction_Sequence`

**Scale:** 274 / 285 nodes

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/graphs.txt:4 GRAPH ConstructionGraph nodes=274`; `<dump>/Game__Assets__Zebra__Rig__CRM_Zebra_Face/regen.py:5849 add_link VariableNode_48.ExecuteContext -> SpawnControl.ExecutePin`

#### UE7-deformer-origin-nulls — Construction-spawned deformer origin nulls (head-parented, global placement)

*important* · assets: CR_Zebra_Deform, CRM_Monster_Face

The construction event spawns one null per deformer with RigUnit_HierarchyAddNull (Parent = bone 'head', Space=GlobalSpace) and stores each key in an FRigElementKey variable.

Zebra (CR_Zebra_Deform):
| Null | Translation | Rotation |
| Head Squash Null | (0,0,80) | identity |
| Head Bend Null | (0,0,90) | 90 deg about Z (quat 0,0,.7071,.7071) |
| Head Twist Null | (0,0,80) | identity |
| Muzzle Squash Null | (0,0,110) | 180 deg about Y (quat 0,-1,0,0) |
| Muzzle Bend Null | (0,20,110) | quat(-.7071,-.7071,0,0) = 180 deg about (1,1,0)/sqrt2: X->Y, Y->X, Z->-Z |
| Skull Tp Squash Null | (0,0,100) | identity |
| Skull Tp Bend Null | (0,0,100) | 90 deg about Z |

Monster (CRM_Monster_Face, runtime names Face/...):
| Null | Translation | Rotation |
| Head Squash Null | (0,0,20) | identity |
| Head Bend Null | (0,0,25) | 90 deg Z |
| Head Twist Null | (0,0,25) | identity |
| Skull Tp Squash Null | (0,5,45) | 90 deg Z |
| Skull Tp Bend Null | (0,0,40) | 90 deg Z |
| Muzzle Squash Null | (0,30,60) | 180 deg Y |
| Mouth Squash Null | (0,30,60) | 180 deg Y |
| Muzzle Bend Null | (0,20,60) | 180 deg about (1,1,0) |
| Mouth Bend Null | (0,20,60) | 180 deg about (1,1,0) |

Every frame each null's GlobalSpace transform (so it follows the head) is fed to the deformer's Transform variable.

**Setup.** The nulls are not animator-facing; they only define each deformer's frame.

**Operators:** `RigUnit_HierarchyAddNull`, `RigUnit_GetTransform`, `RigVMVariableNode (FRigElementKey)`

**Scale:** 7 Zebra and 9 Monster nulls.

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__CR_Zebra_Deform/graphs.txt:679-717 (HierarchyAddNull..._6, Space=GlobalSpace)`; `<dump>/Game__Assets__Monster__Rig__CRM_Monster_Face/graphs.txt:252-269 (HierarchyAddNull_9.._17)`; `<dump>/Game__Assets__Monster__Rig__MR_Monster/runtime_hierarchy.txt:1116-1124 (NULL Face/Head Squash Null ... init_global)`

#### UE8-engine-addcontrol-root-modules — Engine stock modules Root and AddControl (behaviour inferred from package name tables)

*important* · assets: /ControlRig/Modules/Modules58/Root, /ControlRigModules/Modules58/AddControl, MR_Boombox

AddControl (inferred from function and pin names; its graph was not dumped). Connectors: 'Add Control Primary' (primary) and 'Parent Control'. Construction: 'Create Controls' uses 'Control Stack at Item', which spawns a null '<name><Null Suffix>' (default suffixes '_null'/'_ctrl_null'), a control '<name><Control Suffix>' ('_ctrl'), an optional 'Add Bottom Null', and an optional 'Add Secondary Control' with Secondary Settings. It applies 'Offset Control' (offset transform), 'Orient to World', and 'Offset Initial Transform' (GlobalSpace or LocalSpace). 'Create Sockets' collects children of the connected item and runs HierarchyAddSocket named Concat(child, '_socket'). ResolveConnector bIsConnected gates the Parent Control branch. Forward solve: 'Attach Bone to Control' / 'Attach Control to Skeleton' use ParentConstraint with bMaintainOffset. Colors come from 'Color Module Controls' / ColorizeControls: side is chosen by the control position against the X/Y/Z Plane and the Negative flag, giving LeftColor/RightColor/CenterColor, with an Override from module metadata. 'Scale Module Controls' uses Global Scale x Local Scale. ModuleSettings struct: CenterColor, CharacterFacingDownAxis, ControlSize, ControlSuffix, LeftColor, LeftSideSuffix, MirrorAxis, OverrideColor, Prefix, PrimaryBoneAxis, RightColor, RightSideSuffix, SecondaryAxis. Root: a single 'Root (Primary): The root bone' connector and a RootModuleSettings struct. Construction creates the root stack from Root Control Name, Global Control Name and Body Offset Control Name, each with a '_ctrl' suffix (producing root_ctrl, global_ctrl, body_offset_ctrl), plus Root/Global/Body Offset Control Settings. It computes skeleton data (Generate Biped Skeleton Data, Map Skeleton Tree, Detect Scale, Get BBox of Skeleton with Use Max Box Size and Offset) to size controls, sets 'Global Metadata for the child modules', and creates sockets from metadata (pelvis_socket, spine_socket, spine_01_socket, biped_physics_socket). Backwards Solve runs 'INV Root'. These packages need a real dump (graphs, regen) before re-implementation.

**Setup.** Module Settings, Control Settings (shape name/transform), Secondary Control Settings, Control Suffix, Null Suffix, Offset Control; Root: control names and shapes.

**Operators:** `HierarchyAddSocket`, `RigUnit_HierarchyAddControlTransform`, `RigUnit_ParentConstraint`, `RigUnit_SetControlColor`, `RigVMFunction_MathBoxFromArray`

**Scale:** 1 Root + 7 AddControl instances (MR_Boombox only)

**Evidence:** `<UE>/Plugins/Animation/ControlRigModules/Content/Modules58/AddControl.uasset name table: 'Add Control Primary Connector','Control Suffix','_ctrl','_ctrl_null','_socket','ConstructionGraph___Create_Sockets_HierarchyAddSocket_Color__Const','Attach Bone to Control','ColorizeControls.X Plane','Entry.Module Settings.CharacterFacingDownAxis_27_...'`; `<UE>/Plugins/Animation/ControlRig/Content/Modules/Modules58/Root.uasset name table: 'Root Control Name','Global Control Name','Body Offset Control Name','body_offset','pelvis_socket','spine_socket','biped_physics_socket','INV Root','Get BBox of Skeleton.Use Max Box Size','- Root (Primary) : The root bone.'`; `<UE>/Plugins/Animation/ControlRigModules/ControlRigModules.uplugin (Beta, EnabledByDefault, depends on ControlRig, ControlRigSpline, RigVM, ControlRigPhysics, PhysicsControl, FullBodyIK)`

#### UE1-procedural-element-limit — Procedural element limit per modular rig

*nice-to-have* · assets: MR_Zebra, MR_ZebraDMC, MR_FN_Biped, MR_FN_BipedDMC, MR_Monster, MR_Boombox, CRU_PropAim

HierarchySettings.ProceduralElementLimit caps how many elements construction events may spawn. It appears to be sized as the static element count plus a 2000 margin: Zebra 1486 + 2000 = 3486; ZebraDMC 3487; Biped 1106 + 2000 = 3106; BipedDMC 3107; Monster 1184 + 2000 = 3184; Boombox 24 + 2000 = 2024; PropAim 4 + 2000 = 2004.

**Operators:** `FRigHierarchySettings::ProceduralElementLimit`

**Evidence:** `<dump>/Game__Assets__Zebra__Rig__MR_Zebra/asset.t3d:805 'HierarchySettings=(ProceduralElementLimit=3486)'`; `<dump>/FortniteRigs__Templates__MR_FN_Biped/asset.t3d: 'HierarchySettings=(ProceduralElementLimit=3106)'`

#### UE4-add-null-above — Add Null Above (insert offset null between an item and its parent)

*nice-to-have* · assets: CRFL_Hierarchy_v001

Construction. Inputs: Item, Old Suffix and New Suffix. Output: Null. name = Replace(Item.Name, OldSuffix -> '') + NewSuffix. NameReplace is case-insensitive; a None Old leaves the name unchanged and a None New means empty. It spawns a Null with that name under the item's default parent, at the item's initial global transform (GlobalSpace). It then calls SetDefaultParent(Item, Null), which re-parents with maintain-global and removes all other parents. The face rigs call a same-named function from the engine StandardFunctionLibrary, not this copy.

**Operators:** `RigUnit_HierarchyGetParent (bDefaultParent)`, `RigUnit_GetTransform (initial global)`, `RigUnit_HierarchyAddNull`, `RigUnit_SetDefaultParent`, `RigVMFunction_NameReplace`, `RigVMFunction_NameConcat`

**Scale:** Used only by Control Stack at Position.

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:35-85 'Add Null Above' (comment 'Create a null above the given item in the same space.')`; `UE_5.8/Engine/Plugins/Runtime/RigVM/Source/RigVM/Private/RigVMCore/RigVMNameCache.cpp:225-251 Replace (None handling, IgnoreCase)`; `UE_5.8/Engine/Plugins/Animation/ControlRig/Source/ControlRig/Private/Units/Execution/RigUnit_DynamicHierarchy.cpp:78-104 SetDefaultParent -> AddParent(child,parent,1.0,true,true)`; `functions_used.json: CRM_Zebra_Face 'Add Null Above' resolves to /ControlRig/StandardFunctionLibrary (FUNC ... @ StandardFunctionLibrary)`

#### UE4-add-null-below — Add Null Below

*nice-to-have* · assets: CRFL_Hierarchy_v001

Inputs: Item, Replace Suffix and New Suffix. Output: Null. It spawns a Null named Replace(Item.Name, ReplaceSuffix -> '') + NewSuffix, parented directly under Item, at Item's initial global transform (GlobalSpace). No re-parenting is done.

**Operators:** `RigUnit_HierarchyAddNull`, `RigUnit_GetTransform (initial)`, `RigVMFunction_NameReplace`, `RigVMFunction_NameConcat`

**Scale:** No callers.

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:87-125 'Add Null Below'`

#### UE4-control-stack — Control Stack at Position / at Item (null + control + secondary + bottom null)

*nice-to-have* · assets: CRFL_Hierarchy_v001, CRFL_Module_v001

At Position inputs: Global Position, Parent, Name, Settings, Add Null, Add Bottom Null, Add Secondary Control, Secondary Settings, Control Suffix (default '_ctrl'), Null Suffix (default '_null') and Orient to World. Outputs: Control, Top Null, Secondary and Bottom Null. The sequence runs A to E. (A) Spawn a transform control Name+ControlSuffix with OffsetTransform=GlobalPosition, value identity and Settings. SetDefaultParent(control, Parent) keeping global. Re-set the control's initial global to itself, which zeroes it. If Orient to World, set its initial global rotation to identity (keeping T/S). Tag the control 'Controls'. (B) If Add Secondary: spawn control Name+'_sec'+ControlSuffix under the control with Secondary Settings, and set metadata 'Secondary' on the control. (C) If Add Null: AddNullAbove(control, Old=ControlSuffix, New=NullSuffix), set metadata 'Null', then set the control's initial global to the null's initial global. (D) If Add Bottom Null: spawn Null Name+'_ctrl_null' (hard-coded suffix) under the secondary if it exists, else under the control, with identity local; set metadata 'Bottom Null'. At Item: Name = (Name != None) ? Name : Item.Name and Global Position = Item's initial global; it calls At Position and then sets metadata 'Item' (Self) on the control = Item. Library node default for At Item: Add Null=true.

**Operators:** `RigUnit_HierarchyAddControlTransform`, `RigUnit_SetDefaultParent`, `RigUnit_SetTransform (initial global)`, `RigUnit_GetTransform (initial)`, `RigUnit_HierarchyAddNull`, `RigUnit_SetMetadataTag`, `DISPATCH_RigDispatch_SetMetadata`, `RigVMFunction_MathTransformMakeRelative`, `FUNC Add Null Above`

**Scale:** No module callers. The only reference chain is Create FK Chain Controls (itself unused) -> At Item -> At Position.

**Evidence:** `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:127-351 'Control Stack at Position' (Concat_1_1 'B=_ctrl_null', Concat_2 'B=_sec', comments 'This will Zero the NUL', 'Orient to world')`; `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:373-458 'Control Stack at Item' (comment 'If user provides a Name other than None, use that instead.')`; `ue/<dump>/FortniteRigs__Libraries__CRFL_Hierarchy_v001/graphs.txt:13-14 library defaults 'Control Suffix=_ctrl; Null Suffix=_null'`

## 3. Open questions and suspected authoring defects

Questions the analysts could not settle from the dump alone.

### UE4 CRFL libraries

- RigVM function LOCALVAR lifetime: Conform Name v01 appends to local 'Name Tokens' without an explicit Reset, and Get Chain Length accumulates into local 'distance'. If RigVM does not reset function locals on each call, repeated calls would accumulate. The engine's local-variable reset semantics were not checked.
- Project Middle Bone to IK Plane 'Same As Source' compares Start with the projected mid (|S - out| < 0.001). It is probably meant to compare the original and projected mid. No caller reads this output.
- Get Item Name v01 at the IkFk2Bones call site does not override 'Numerical Name Start Index'. The library node declares 1, but runtime names are 'FK 0' and 'FK 2', so the call's result (RerouteNode_43 -> RerouteNode_54) may feed something other than the element name (display name or null naming). This was not traced.
- Compute FK, Compute Pole Vector Parent, Compute Mid Null Transform and Calculate And Set Segment Scale Channel Values (IkFk2Bones-local) and the Foot-local Match FK/IK were not analysed in detail. They are out of CRFL scope and should be covered by the module analysis.
- Control Stack at Position leaves SpawnControl.Parent unlinked (Bone 'None') and relies on SetDefaultParent with maintain-global. The exact initial-offset result after re-parenting depends on engine AddParent behaviour; the graph then explicitly re-sets the initial global to 'zero' the control.
- Construct Space Nulls v01 reads the driven control's default-parent transform with bInitial=False. During construction current equals initial, so this is assumed to be the bind pose.

### UE2 FK-family modules

- modular_rig_model.txt shows ConfigOverrides=() for every module, yet the runtime hierarchies clearly reflect overrides: Neck has no Pelvis Local or End Movable Pivot (so Is Neck=true), display names such as 'Pelvis FK', 'Chest', 'Base/Mid/Tip' are set, and Clavicle L has preferred_rotation_order=XZY. The dump does not capture per-module config values, so they must be recovered from runtime_hierarchy.txt or re-exported.
- CREnum_RootMatching: the t3d only contains the DisplayNameMap (NewEnumerator0/1/4). The integer value order of the enumerators is not dumped. The Root backwards comment boxes imply 0 = Root Control, 1 = Global Control, 2 = Local Control, which conflicts with the display-map ordering (Global, Local, Root). This needs verification in-editor.
- Root backwards 'first frame' Global snap depends on Forwards resetting 'Global Control Snapped' to false. During a bake the host's call order (forward/backward interleaving) determines whether the Global control is re-snapped every frame or only on the first frame.
- Body 'Drive Aim and Body Rotation', the Movable Proxy and Prop Change Pivot depend on IsInteracting, RequestAutoKey and variable state that persists between evaluations. Offline or USD evaluation cannot reproduce these without an interaction model, so decide whether usdRig should support or ignore them.
- FkArray construction initializes the Visibility channel with 'Num >= index' instead of 'Num > index', which is a potential out-of-range read. The Override Parents loop checks 'Indices to Remove' against the override index instead of the compacted bone index. The backwards solve evaluates the raw Spaces/Orient Spaces lists instead of the Combined lists used in forwards. These look like bugs, and the intended behavior is unclear.
- FkArray forwards contains a DMC branch (SetupShapeLibraryFromLayer 'ik-layer' / 'fk-layer') with no incoming execute link, so it is dead code. The intended runtime shape-layer switching is unknown.
- Spine construction spawns the '<bone>_reoriented' nulls with Space=LocalSpace but feeds them global bone values, which gives odd initial transforms at runtime (e.g. T(-81.1,...)). Forwards overwrites them every frame, but the backwards solve never updates them.
- Spine 'Bone Percentage' is the straight-line distance from the first bone divided by the Bezier arc length, not a cumulative arc length. Confirm whether usdRig should replicate this exactly.
- Spine local functions 'Construct Sliding Proxy' and 'Forward Sliding Proxy' (with a 'Proxy Slide Channel' output) are defined but never referenced (0 FUNC references), and were not analyzed in detail.
- modular_rig_model.txt connection lists contain stale connector names (e.g. 'Root/WorldOffsetPosition', 'Spine/StartSnapTo', 'Clavicle L/OrientSpace') left over from older module versions, next to the current names. Only the current connector names resolve.
- RigUnit_SetControlOffset is documented as 'typically only used during the Construction Event', but the Spine uses it every Forwards and Backwards frame. An evaluator must support runtime offset changes, ideally without invalidating caches.

### UE5 body deformation and runtime

- SPR scale factors above 1 (1.1-2.2) exceed the header's ClampMax=1 metadata. RemapAndConvertInputs uses the raw values, so the outer ellipse is extrapolated, but it is unverified whether RigVM clamps pin values at runtime; derived outer extents such as 168 deg assume no clamp.
- The dump shows no FlipWidthScaling/FlipHeightScaling pins for any reader, so false is assumed. The nodes may predate these pins.
- Several left/right asymmetries may be authoring bugs: - thigh_bk_r (graphs.txt:205) uses the same RotationOffset (0,0,-30) as thigh_ot_r (graphs.txt:112), while thigh_bk_l uses (-90,0,0). - def_strap_r shoulder-fwd rotation is 60 deg vs 30 deg on the left. - The left thigh_fwd_ext offset has no rotation while the right has -15 deg. - thigh_twist_01_l and thigh_twist_01_r both rotate Z-20. - The right elbow reader uses a different offset/half-plane convention (0,-90,0 with +H=0) from the left (90,0,90 with -W=0). For parity, replicate them as authored.
- SKM meshes have default_mesh_deformer=None, so AddOptimusDeformer forces SetAlwaysUseMeshDeformer(true) and the project's default mesh deformer (likely a linear-blend-skin + morph graph) is used as the base. The project setting is not in the dump.
- CRM_Zebra_Face has two writers of muzzle_squash_deformer (SetCurveValue_65 fed by the head-squash remap, SetCurveValue_66 fed by the muzzle control). Which one executes last, and so wins, was not traced.
- The 'Squetch' curve (chain stretch - 1) can be negative. Whether UE applies negative morph weights for this shape, or clamps them, depends on renderer and morph settings not in the dump.
- Monster_PostAnimBP's ControlRig node serializes ErrorType=1 (asset.t3d:97) with no message. This may be a stale compile error; its runtime effect is unknown.
- The dump does not contain MovieSceneControlRigParameterTrack channel lists or keys, nor Zeb_Face_Expressions curve key values; only names are available.
- Morph target application relies on UE's implicit curve-name == morph-name matching. SK_Zebra's AnimCurveMetaData export is empty, so explicit bMorphtarget/material flags for Zebra curves (e.g., cornea_size, pupil_dilation as material curves) are unknown.
- Whether the twist bones in the post-process input already carry LimbTwist distribution depends on the animation source (baked animation includes the twist tracks). CR_Zebra_Deform only adds offsets and never computes twist distribution itself.

### UE7 deformer graphs and DMC

- The DirectMeshControl deformer DG_DirectMeshControl (<UE>/Plugins/Experimental/Animation/DirectMeshControl/Content/Deformers/DG_DirectMeshControl.uasset) is binary and was not dumped. It is not confirmed that it reads the source positions through the SubToSource attribute and applies OverlayColor. The C++ only shows the attribute name 'SubToSource', the variable name 'OverlayColor', the 'DirectMeshControl' component binding and the ComponentResolver to the source mesh.
- No control in the headless runtime_hierarchy of MR_ZebraDMC or MR_FN_BipedDMC resolved to an 'ik-layer.*' or 'fk-layer.*' shape. Possible reasons: SKM_Zebra or SKM_Manny lack those triangle-label layers, or the editor delegate/subsystem was unavailable in the commandlet. The mesh summaries do not expose triangle label layers, so which bones actually have DMC patches is unknown.
- Binding key mismatch: the DMC templates bind ("Direct_Mesh_Control","Direct_Mesh_Control"), but the CRM_FN_DMC module variable is named 'Direct Mesh Control' (with spaces). It is unclear whether the binding resolves. The module default is True, so DMC is enabled either way.
- Effective ExpandTowardsLeaf=999 for every Skin Weights as Vertex Mask is inferred from the property being absent in the T3D (equal to the CDO default in the 5.8 header). If the 5.8 CDO differs, the head masks would cover only the head bone's own weights, not the whole face hierarchy.
- The Zebra graphs reference Monster_Head_DeforerGraph with a non-existent function GUID ('<graph missing>'). It is unclear whether recompiling in 5.8 would drop the CacheGeometry pass (as in the Modified ZebraHead) or fail. Either way the pass is an identity copy, so the deformation result should not change.
- ZebraHead_DeformerGraph has no Status line, so it is in the default Modified state. It is unconfirmed whether the engine uses its stale compiled graph or recompiles it at load.
- Whether SKM_Zebra, SKM_Zebra_Hi and SKM_Monster enable BuildHalfEdgeBuffers (required by the normals pass) is not in the mesh summaries. Without it the normal recompute may be skipped or fall back.
- ZebraMuzzleTwist_DeformerGraph is authored but no rig adds it. Zebra has no control or curve for muzzle twist; the Monster rig has no muzzle twist either.
- Kernel-stage numeric behaviour (float precision, per-section invocation offsets) was not validated. The math above is read directly from the extracted HLSL.

### UE1 modular body rigs

- modular_rig_model.txt prints ConfigOverrides=() for every module. FControlRigOverrideContainer apparently doesn't text-export. The per-module values listed here were recovered from the preview instance sub-objects (MR_*_C_0.<Module>) in asset.t3d. Those should equal the overrides, but that equality is inferred and not verified against the serialized container. A consolidated dump is in (analysis helper table, not kept).
- Runtime control colors and limit min/max values (only the enabled flags) are missing from runtime_hierarchy.txt. Channel ranges were taken from module graph pin defaults instead. Final per-control shape colors after the side-color logic are unknown.
- runtime_hierarchy.txt truncates some long lines ('<trunc N>'), e.g. Arm L/IK and Arm L/PV AvailableSpaces. The 9-entry Arm IK space list is inferred from the connection list and the existing '<Target> IK Null' nulls.
- AAU_Biped's SupportedClasses (the CDO property) is not in the t3d export. It presumably targets CRM_FN_IkFk2Bones_C, since the events To IK/To FK/Key Controls exist only there, but this is unconfirmed. The 'Match Foot'/'Foot Module Name' locals are unused, so the Foot follow-up the comments describe is not implemented in this version.
- The DMC binding key is 'Direct_Mesh_Control', while the CRM_FN_DMC variable is named 'Direct Mesh Control' (with spaces). It is unclear whether the runtime-asset property bag sanitizes names so the binding resolves. SetModuleVariableBindings returns false on a missing source/target.
- Engine Modules58 Root/AddControl are binary-only (no text dump), so the Boombox control and socket spawning behavior is inferred from connector and socket names. MR_Boombox has no runtime_hierarchy dump.
- The Stretch Feedback module in the Biped template only lists left-side arm and leg elements. Whether it derives the right side by name mirroring was not checked in its graph (other scope).
- Monster rig has neck_01, neck_02 and head bones but no Neck module. Whether the head is driven only by CRM_Monster_Face (Head Squash / Head Attach Null) belongs to the face-rig analysis.

### UE6 face rigs

- Where do pupil_dilation / cornea_size / highlight_offset_x curves get consumed? They are not morph targets and no dumped ABP or material references them; presumably material curves through AnimCurveMetaData on SKM_Zebra/SKM_Monster (not dumped).
- Library functions with no wired execute pins (Brow Main, Brow Micro, Corner Logic, Expression Shape Logic, Brow Squeeze Logic) are assumed to run as data dependencies of the SetCurveValue consumers. This was not confirmed in the RigVM compiler source.
- Does the forward solve see Jaw Normalize with a one-frame lag, and do Eye L/R null aim updates lag? This depends on whether member variables and null transforms persist between evaluations. The analysis assumes they do; UE was not run.
- The exact SphericalPoseReader rest output for the jaw reader (RotationOffset 90,0,90 with DriverAxis Y) was not evaluated numerically. The analysis assumes it goes from 0 when the jaw is closed toward 1 as the jaw opens.
- The contents of StandardFunctionLibrary 'Add Null Above' (engine content asset) were not dumped. Its behaviour is inferred from the runtime result: an Eye Aim Null is inserted between skull and Eye Aim at the same transform.
- Whether RigUnit_SetControlVisibility and the float/bool channel reads behave identically outside the editor, for example in sequencer or at runtime with MR_Zebra driving AnimBP_Zebra/CR_Zebra_Deform, was not checked.
- The lid Rot/Blink quaternion tables were converted to approximate degrees by eye (2*asin of the dominant component). Mixed-axis quaternions (for example Lid Tp Blink [0]) need exact conversion when re-implementing.

### UE3 IK-family modules

- IK End Align Solve writes the IK Null global rotation after Soft IK has already consumed the IK Null (the Soft IK -> IK End Align order is at graphs.txt:597,551). The effector rotation therefore likely takes effect through the persisted null local on the next evaluation. Is this one-evaluation lag intended? Also, the Virtual Bone C rotation write uses Weight=0 (a no-op).
- Backwards Solve sets the root-level 'Mid' null's local transform to identity (graphs.txt:1725,1764), which appears to move it to the origin until the next forward solve. Bug or intended?
- Forward Solve contains an orphan node 'Set Transform_11' (sets the PV parent rotation from the root 'Global Control'). It is never executed because no exec link reaches it.
- In the Foot, the 'Heel control interacted -> copy Toe IK null onto Toe Tip Pivot null' branch sits only under the not-interacting branch, so it looks unreachable. Two Euler nodes in Set Foot Pivots have unused outputs, and the 'Ball Pivot' scalar is never consumed.
- LimbTwist 'Reset Twist on Start Bone' removes twist against the world-identity reference (SwingTwist of the global rotation) rather than the parent frame. Confirm whether usdRig should replicate this exactly, since it depends on the bone's world axis alignment.
- The LimbTwist Connector event references non-existent connectors ('Start Socket', 'Clavicle'); these look like leftovers.
- The ProxyControl 'Pivot Vis' channel is spawned but never read. The native bIsProxy / DrivenControls behaviour (selection and keying redirection) is editor-side and not visible in the graph.
- Module evaluation order (the Foot's Pre Forwards Solve running before the leg's Forwards Solve, and child modules after parents) comes from the ModularRig runtime, not from these graphs. It should be confirmed against ModularRig.cpp before re-implementing the leg/foot coupling.
- The ConfigOverrides=() lists in modular_rig_model.txt are empty. The per-instance config values were taken from the module instance objects in asset.t3d.

### UE8 coverage pass

- Which Control Rig channels actually carry keys in zebra_audition, zebra_marketingPoseFaces and MR_Zebra_Take1, and what are their key times, values, interpolation and tangents? The name table lists every animatable control. A re-dump is needed: for each section, iterate get_channel_proxy / ControlRigSequencerLibrary.get_local_control_rig_* and get_control_rig_space_keys, and read FMovieSceneControlRigSpaceChannel key times and values.
- Which exact controls own the space channels, and what are the key times? Known: zebra_audition has Parent, World and ControlRig keys with targets 'Arm R/FK 0 Body Orient Space', 'Arm R/FK 0 spine_05 Orient Space' and 'Neck/End FK Global Orient Space'.
- How is the Boombox kept in the Zebra's hand in zebra_audition? No constraint channels exist, so check whether the MR_Boombox actor Transform track or the MR_Zebra arm IK space keys (Prop/Prop Attach spaces) carry the relationship.
- What does the Zebra mesh-level AnimCurveMetaData contain (bMaterial and bMorphtarget per curve)? This decides whether cornea_size, pupil_dilation, highlight_offset_x, highlight_radius and highlight_softness reach MI_eye_Zeb / M_eye_eyeball_updated as material parameters. Re-dump SkeletalMesh.get_asset_user_data(AnimCurveMetaData) and the material parameter names.
- What is the per-vertex bone influence count of SKM_Zebra, SKM_Zebra_Hi and SKM_Monster (the UsdSkel elementSize)? Given UnlimitedBoneInfluences=True, it needs a skin-weight dump (SkeletalMeshLODModel / get_skin_weights).
- What are the full default values of the 69 variable records truncated at 500 characters, especially the face-module arrays (Soft Eyes Dn/Up, Lid Tp/Bt Blink Rotations, Lid Micro Control Shapes)? Re-dump without truncation.
- How do the engine AddControl and Root modules (Modules58) actually behave (graphs, pin defaults, ModuleSettings/RootModuleSettings defaults)? The description here is inferred from name tables only. Dump /ControlRigModules/Modules58/AddControl and /ControlRig/Modules/Modules58/Root with the same dump.py, and instantiate MR_Boombox with the ControlRigRuntimeAsset API.
- Does any module add a CRSL shape library at runtime (SetupShapeLibraryFromUserData reads mesh user data 'ShapeLibrary') that would supply the pin shapes on MR_FN_BipedDMC and MR_Monster, making the dangling /EpicControlRig DMC entries harmless?
- Does the Run IK Rig op of RTG_UEFN_to_Zebra ever use its RightClav->RightLeg / LeftClav->LeftLeg mapping (its Chains list contains only the 4 limb chains)?

