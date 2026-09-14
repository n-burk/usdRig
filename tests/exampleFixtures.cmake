# ---------------------------------------------------------------------------
# The per-example fixture table: one source of truth.
#
# Three harnesses ask the same three questions of every shipped example -- at
# which frames, on which control, and on which operator input -- and they must
# never disagree about the answer. The ctest entries built by
# rigexec_add_example_parity_test read this table; so does the generated
# header tests/rigExecExampleFixtures.h, which is what the C++ drag suite
# walks. A heuristic search would have made each of them right about a
# different set of prims.
#
# Each entry is one '|'-separated record, with no spaces around the bars:
#
#   stage|frames|controlPrim|controlAvar|operatorPrim|operatorInput|bakes|blocker
#
#   stage         path under examples/, the file the rig is opened from
#   frames        comma-separated frame list, passed to rigExecPose --frames
#                 and split by the C++ suite. Every authored sample time of
#                 the stage, plus points BETWEEN them: the numbered examples
#                 all run 1001-1048, so a sweep at frames 1, 2, 3 reads the
#                 first key held and compares the same static pose three
#                 times. Verified against the authored timeSamples in each
#                 file, not assumed from the time-code range.
#   controlPrim   a RigExecControl a gizmo could land on, '-' when the
#                 example authors no controls at all (04, 06, 07 and the
#                 constraint stages drive their rigs from plain Xforms)
#   controlAvar   the avar on it to drag; double-valued
#                 ('-' is the empty field throughout: CMake's list commands
#                 drop a genuinely empty element, so a record written with
#                 '||' would silently lose a column)
#   operatorPrim  a mover, weight or constraint prim -- the OTHER kind of
#                 drag, the one that lands on an operator's own input and
#                 needs the prim to be routed through the resolved inputs.
#                 '-' only for a stage that authors no operator at all
#                 (components/spider_leg.usd is joints and nothing else);
#                 the pair must be empty together, which the loop below
#                 enforces so a half-filled record cannot skip a drag
#   operatorInput the attribute on it to drag; float- or double-valued
#   bakes         YES when RigExecBakedProgram::Build accepts the rig today
#   blocker       when it does not, the operator group whose refusal is the
#                 FIRST one IsBakeable reports; '-' when it bakes. A stage
#                 may need more than one group; the name is where to start,
#                 not a promise that the entry goes green the moment that
#                 group lands.
#
# Not listed, deliberately -- rigExecPose exits 2 on the first five and 1 on
# the last two, so an entry for any of them could never pass:
#   aimtest_xform_flattened.usd, spider_legs_assembly.usd, arcTestAsset.usda,
#   biped/Biped_layered_left.usda, biped/Biped_layered_right.usda
#       carry no RigExecRoot -- they are geometry or layer fragments
#   simple_rig_flattened.usd, biped/Biped_layered_center.usda
#       are sublayer arms that do not compile when opened on their own
# Every other stage under examples/, examples/biped/ and examples/components/
# has an entry, so a new example that nobody wired up is a visible omission
# rather than a silently skipped test.
# ---------------------------------------------------------------------------
set(RIGEXEC_EXAMPLE_FIXTURES
    # -- the numbered tour -------------------------------------------------
    "01_FkChainTail.usda|1001,1012,1024,1036,1048|/TailAsset/Rig/Controls/Tail2|avars:rz|/TailAsset/Rig/Weights/Seg2W|rigExec:defaultWeight|YES|-"
    "02_TwoBoneIkLeg.usda|1001,1016,1024,1032,1048|/LegAsset/Rig/Controls/FootIK|avars:ty|/LegAsset/Rig/Weights/KneeDriven|inputs:driver|YES|-"
    "03_IkFkBlendClamp.usda|1001,1020,1024,1028,1048|/BlendArmAsset/Rig/Controls/HandIK|avars:tx|/BlendArmAsset/Rig/Solvers/IKFKBlend|inputs:weight|YES|-"
    "04_BlendShapeFace.usda|1001,1016,1024,1040,1048|-|-|/FaceAsset/Rig/BlendInputs/Smile|inputs:weight|NO|provisional"
    "05_TwistRibbonSpine.usda|1001,1012,1024,1036,1048|/SpineAsset/Rig/Controls/ChestCtl|avars:rz|/SpineAsset/Rig/Weights/FinW|rigExec:defaultWeight|NO|provisional"
    "06_LatticeBulge.usda|1001,1012,1024,1036,1048|-|-|/LatticeAsset/Rig/Movers/Geometry/VolumeCorrect/Smooth|inputs:defaultWeight|YES|-"
    "07_SurfaceDrape.usda|1001,1012,1024,1036,1048|-|-|/DrapeAsset/Rig/Movers/Geometry/AttachToGround|inputs:defaultWeight|YES|-"
    "08_AimEyes.usda|1001,1012,1016,1032,1048|/EyesAsset/Rig/Controls/LookAt|avars:tx|/EyesAsset/Rig/Movers/Pose/AimL|inputs:defaultWeight|YES|-"
    "09_PropertyMathMovers.usda|1001,1012,1024,1036,1048|/PropMathAsset/Rig/Controls/RootCtl|avars:ry|/PropMathAsset/Rig/Movers/ClampGain|inputs:max|YES|-"
    "10_AimXformTurret.usda|1001,1012,1024,1036,1048|/TurretAsset/Rig/Controls/TrackTarget|avars:tx|/TurretAsset/Rig/Movers/AimBarrel|inputs:defaultWeight|YES|-"
    "11_VolumeWeights.usda|1001,1012,1024,1036,1048|/VolumeAsset/Rig/Controls/Root|avars:rz|/VolumeAsset/Rig/Joints/Shoulder/ShoulderVolume|inputs:falloffMax|YES|-"
    "12_CurvenetProfile.usda|1001,1012,1024,1036,1048|/CurvenetAsset/Rig/Controls/BendCtl|avars:rz|/CurvenetAsset/Rig/Movers/Geometry/ProfileMover|inputs:defaultWeight|NO|provisional"
    "13_ReadPhases.usda|1001,1012,1024,1036,1048|/ReadPhaseAsset/Rig/Controls/LiftCtl|avars:ty|/ReadPhaseAsset/Rig/Weights/CageW|rigExec:defaultWeight|NO|provisional"
    # -- the constraint stages: every one aims at a plain UsdGeomXformable --
    "aimtest.usda|1,25,50,75,100|-|-|/World/RigRoot/Movers/RigExecAimConstraint1|inputs:defaultWeight|YES|-"
    "aimtest_points.usda|1,25,50,75,100|-|-|/World/RigRoot/Movers/RigExecAimConstraint1|inputs:defaultWeight|YES|-"
    "rotateConstraint.usda|1,25,50,75,100|-|-|/World/RigRoot/Movers/RigExecRotationConstraint1|inputs:defaultWeight|YES|-"
    "rigexec_flat.usda|0,25,50,75,100|-|-|/World/RigRoot/Movers/RigExecAimConstraint1|inputs:defaultWeight|YES|-"
    "par_rot_aim.usd|0,25,50,75,100|-|-|/World/RigRoot/Movers/RigExecAimConstraint1|inputs:defaultWeight|YES|-"
    "par_rot_aim_redorder.usd|0,25,50,75,100|-|-|/World/RigRoot/Movers/RigExecAimConstraint1|inputs:defaultWeight|YES|-"
    "rot_par_combo.usd|0,25,50,75,100|-|-|/World/RigRoot/Movers/RigExecRotationConstraint1|inputs:defaultWeight|YES|-"
    "aim_par_combo_flattened.usd|0,25,50,75,100|-|-|/World/RigRoot/Movers/RigExecAimConstraint1|inputs:defaultWeight|YES|-"
    # -- the arm, asset and shot ------------------------------------------
    "ArmRig.usda|1001,1024,1048|/ArmAsset/Rig/Controls/HandIK|avars:tx|/ArmAsset/Rig/Solvers/IK|inputs:softness|NO|solvers"
    "ArmShotAnim.usda|1001,1012,1013,1024,1048|/Shot/HeroArm/Rig/Controls/HandIK|avars:ty|/Shot/HeroArm/Rig/Solvers/IK|inputs:softness|NO|solvers"
    # -- the stages that bake today ----------------------------------------
    # The two component layers the spider assembly is built from are rigs in
    # their own right and open on their own; spider_leg.usd is the one stage
    # in the tour with no operator of any kind, so it exercises the frame
    # sweep and nothing else.
    "components/spider_leg.usd|1,2,3|-|-|-|-|YES|-"
    "components/spider_leg_ik.usd|1,2,3|/RigRoot/Controller/hip|avars:ry|/RigRoot/Solvers/RigExecTwoBoneIk1|inputs:softness|YES|-"
    "simple_rig.usd|1,2,3|/World/RigExecRoot/Controllers/Root|avars:tx|/World/RigExecRoot/Movers/RigExecMatrixMover1|inputs:defaultWeight|YES|-"
    "simple_rig_anim.usd|0,12,40,51,79,100|/World/RigExecRoot/Controllers/Root|avars:tx|/World/RigExecRoot/Movers/RigExecMatrixMover1|inputs:defaultWeight|YES|-"
    "spider_legs_assembly_ref.usda|1,2,3|/World/RigExecRoot1/Xform1/Controller/hip|avars:ry|/World/RigExecRoot1/Xform1/Solvers/RigExecTwoBoneIk1|inputs:softness|YES|-"
    "biped/Biped.usda|1,2,3|/Biped/Rig/Controls/hips_ctl|avars:ty|/Biped/Rig/Movers/twist_aims/elbowTwist_l_bind_aim|inputs:defaultWeight|YES|-"
    "biped/Biped_layered.usda|1,2,3|/Biped/Rig/Controls/hips_ctl|avars:ty|/Biped/Rig/Movers/twist_aims/elbowTwist_l_bind_aim|inputs:defaultWeight|YES|-"
    "biped/Biped_anim.usda|1,2,3,4,5,6,7,8|/Biped/Rig/Controls/hips_ctl|avars:ty|/Biped/Rig/Movers/twist_aims/elbowTwist_l_bind_aim|inputs:defaultWeight|YES|-"
)

# The C++ half of the single source of truth: the same records, as the rows
# of a static array, so the drag suite cannot fall behind the ctest entries.
# Built here rather than in CMakeLists.txt because the record layout and the
# struct it fills are the same decision and belong in the same file.
set(RIGEXEC_EXAMPLE_FIXTURE_ROWS "")
foreach(_fixture IN LISTS RIGEXEC_EXAMPLE_FIXTURES)
    string(REPLACE "|" ";" _field_list "${_fixture}")
    list(LENGTH _field_list _field_count)
    if (NOT _field_count EQUAL 8)
        message(FATAL_ERROR
            "example fixture '${_fixture}' has ${_field_count} fields, "
            "expected 8 (see tests/exampleFixtures.cmake)")
    endif()
    list(GET _field_list 0 _stage)
    list(GET _field_list 1 _frames)
    list(GET _field_list 2 _control_prim)
    list(GET _field_list 3 _control_avar)
    list(GET _field_list 4 _operator_prim)
    list(GET _field_list 5 _operator_input)
    list(GET _field_list 6 _bakes)
    list(GET _field_list 7 _blocker)
    if (_bakes STREQUAL "YES")
        set(_bakes_cxx "true")
    else()
        set(_bakes_cxx "false")
    endif()
    # '-' is the table's empty field; the header carries a real empty string
    # so the C++ side tests one thing rather than two.
    foreach(_optional IN ITEMS _control_prim _control_avar _operator_prim
                              _operator_input _blocker)
        if ("${${_optional}}" STREQUAL "-")
            set(${_optional} "")
        endif()
    endforeach()
    # A prim without its attribute, or the reverse, would make the C++ suite
    # skip that drag silently -- exactly the vacuous pass this table exists
    # to prevent -- so a half-filled pair is an error at configure time.
    string(COMPARE EQUAL "${_control_prim}" "" _no_control_prim)
    string(COMPARE EQUAL "${_control_avar}" "" _no_control_avar)
    if (NOT _no_control_prim STREQUAL _no_control_avar)
        message(FATAL_ERROR
            "example fixture '${_fixture}': the control prim and its avar "
            "have to be given together, or both left as '-'")
    endif()
    string(COMPARE EQUAL "${_operator_prim}" "" _no_operator_prim)
    string(COMPARE EQUAL "${_operator_input}" "" _no_operator_input)
    if (NOT _no_operator_prim STREQUAL _no_operator_input)
        message(FATAL_ERROR
            "example fixture '${_fixture}': the operator prim and its input "
            "have to be given together, or both left as '-'")
    endif()
    string(APPEND RIGEXEC_EXAMPLE_FIXTURE_ROWS
        "    {\"${_stage}\", \"${_frames}\", \"${_control_prim}\","
        " \"${_control_avar}\", \"${_operator_prim}\","
        " \"${_operator_input}\", ${_bakes_cxx}, \"${_blocker}\"},\n")
endforeach()
