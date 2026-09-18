cmake_minimum_required(VERSION 3.26)
project(rigExec VERSION 0.1.0 LANGUAGES CXX)

# RigExec builds against an unchanged installed OpenUSD v26.08
# (spec §3.5: no fork, patch, or modification of OpenUSD).
set(USD_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../usd-install"
    CACHE PATH "OpenUSD installation prefix")

find_package(pxr REQUIRED CONFIG PATHS "${USD_INSTALL_DIR}" NO_DEFAULT_PATH)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

if (MSVC)
    add_compile_options(/permissive- /Zc:__cplusplus /bigobj /W3
        /wd4244 /wd4305 /wd4267)
    add_compile_definitions(NOMINMAX _USE_MATH_DEFINES
        WIN32_LEAN_AND_MEAN)
endif()

# ---------------------------------------------------------------------------
# Install layout.
#
# Mirrors OpenUSD's own tree so an install can be dropped beside a USD install
# (or straight into it) and be found by the same PXR_PLUGINPATH_NAME idiom:
# shared libraries in lib/ next to their import libraries -- NOT bin/, which
# is what USD does on Windows too -- plugins in lib/usd/<name>/resources/, and
# the usdview Python plugin in lib/python/.
# ---------------------------------------------------------------------------
set(RIGEXEC_INSTALL_LIBDIR     "lib"
    CACHE STRING "Install directory for libraries, relative to the prefix")
set(RIGEXEC_INSTALL_INCLUDEDIR "include"
    CACHE STRING "Install directory for headers, relative to the prefix")
set(RIGEXEC_INSTALL_PLUGINDIR  "lib/usd"
    CACHE STRING "Install directory for USD plugins, relative to the prefix")
set(RIGEXEC_INSTALL_PYTHONDIR  "lib/python"
    CACHE STRING "Install directory for Python modules, relative to the prefix")
set(RIGEXEC_INSTALL_CMAKEDIR   "lib/cmake/rigExec"
    CACHE STRING "Install directory for the CMake package, relative to the prefix")

# Runtime library resolution on ELF/Mach-O. rigExecImaging is dlopen'd by USD's
# Plug, but the library itself sits in lib/ beside rigExec, so a loader-relative
# rpath is what lets it find its sibling once loaded from an arbitrary working
# directory; USE_LINK_PATH adds the USD prefix's lib directory on top. Windows
# has no rpath and resolves DLLs through PATH instead, which is why the README
# puts %RIG%\lib there.
if (APPLE)
    set(CMAKE_INSTALL_NAME_DIR "@rpath")
    set(CMAKE_INSTALL_RPATH "@loader_path")
elseif (UNIX)
    set(CMAKE_INSTALL_RPATH "$ORIGIN")
endif()
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)

# ---------------------------------------------------------------------------
# rigExecMath: pure point-frame and solver kernels (spec §5).
# ---------------------------------------------------------------------------
add_library(rigExecMath STATIC
    libs/rigExecMath/pointFrame.cpp
    libs/rigExecMath/solvers.cpp
    libs/rigExecMath/singleChainIk.cpp
    libs/rigExecMath/propertyMath.cpp
    libs/rigExecMath/geometryKernels.cpp
    libs/rigExecMath/weightFields.cpp
    libs/rigExecMath/simdKernels.cpp
    libs/rigExecMath/curvenet.cpp
    libs/rigExecMath/curvenetWeights.cpp
    libs/rigExecMath/curvenetAdjustments.cpp
    libs/rigExecMath/sparseSolve.cpp
    libs/rigExecMath/cutMesh.cpp
    libs/rigExecMath/profileMover.cpp
    libs/rigExecMath/dualQuat.cpp
    libs/rigExecMath/splineIk.cpp
    libs/rigExecMath/rbf.cpp
)
target_include_directories(rigExecMath PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
target_link_libraries(rigExecMath PUBLIC arch tf gf vt)
set_target_properties(rigExecMath PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------------
# rigExec: exec type registrations, schema computations, tap set, and the
# staged rig evaluator (spec §§9, 12).
# ---------------------------------------------------------------------------
add_library(rigExec SHARED
    libs/rigExec/types.cpp
    libs/rigExec/parallel.cpp
    libs/rigExec/computations.cpp
    libs/rigExec/moverKernels.cpp
    libs/rigExec/solverKernels.cpp
    libs/rigExec/weightPackets.cpp
    libs/rigExec/curvenetWeightComputations.cpp
    libs/rigExec/curvenetAdjuster.cpp
    libs/rigExec/moverGraph.cpp
    libs/rigExec/tapSet.cpp
    libs/rigExec/rigEvaluator.cpp
    libs/rigExec/bakedProgram.cpp
    libs/rigExec/bakedPose.cpp
    libs/rigExec/bakedSchedule.cpp
    libs/rigExec/bakedGeometry.cpp
    libs/rigExec/bakedWeights.cpp
    libs/rigExec/bakedVerify.cpp
)
target_include_directories(rigExec PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
target_link_libraries(rigExec PUBLIC
    rigExecMath
    # usdSkel is here for UsdSkelBlendShape only: a sparse blend sample names
    # one, and its offsets + pointIndices ARE the sample's shape.
    plug sdf usd usdGeom usdSkel work
    exec execUsd ef vdf
)

# ---------------------------------------------------------------------------
# rigExecRigging: the authoring side of spec section 4 -- a fluent C++ API
# that creates and wires controls, joints, solvers, constraints, weight
# objects, and mover chains on a UsdStage. The engine itself authors
# nothing; this library is what tools (and the Python bindings) use to build
# rigs programmatically.
# ---------------------------------------------------------------------------
add_library(rigExecStandalone SHARED
    libs/rigExecStandalone/sceneDb.cpp
    libs/rigExecStandalone/adapter.cpp
    libs/rigExecStandalone/system.cpp
    libs/rigExecStandalone/pack.cpp)
target_include_directories(rigExecStandalone PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
target_link_libraries(rigExecStandalone PUBLIC rigExec exec esf usd sdf)

add_library(rigExecRigging SHARED
    libs/rigExecRigging/rigBuilder.cpp
    libs/rigExecRigging/schemaAuthoring.cpp)
target_include_directories(rigExecRigging PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
target_link_libraries(rigExecRigging PUBLIC
    rigExecMath
    sdf tf gf vt usd)

# ---------------------------------------------------------------------------
# rigExecImaging: Hydra 2.0 scene-index publication (spec §10).
# ---------------------------------------------------------------------------
add_library(rigExecImaging SHARED
    libs/rigExecImaging/sceneIndices.cpp
    libs/rigExecImaging/bridge.cpp
    libs/rigExecImaging/registry.cpp
    libs/rigExecImaging/playback.cpp
    libs/rigExecImaging/sceneIndexPlugin.cpp
    libs/rigExecImaging/rigAdapter.cpp
    libs/rigExecImaging/touchOverlayAdapter.cpp
    # TouchPose: posed-mesh picking and the Storm shader highlight.
    libs/rigExecImaging/touchPoseMesh.cpp
    libs/rigExecImaging/touchPoseHighlight.cpp
    libs/rigExecImaging/touchPoseApi.cpp
)
target_include_directories(rigExecImaging PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
target_link_libraries(rigExecImaging PUBLIC
    rigExec
    hd
    usdImaging
    usdUtils
    # TouchPose: the wrapped terminal shader is generated through Sdr from
    # the base terminal's glslfx, and the BVH refit runs on work.
    hio
    sdr
    work
)
# Selects dllexport over dllimport in registry.h's C surface while building
# this library; consumers including the installed header get dllimport.
# Baked playback (M2b) executes binaries through the zero-USD runtime,
# statically linked here like every other runtime consumer.
target_link_libraries(rigExecImaging PRIVATE rigExecRuntime)
target_compile_definitions(rigExecImaging PRIVATE RIGEXEC_IMAGING_EXPORTS)

# ---------------------------------------------------------------------------
# rigExecBinary: the .rigexec sectioned container, shared by every bake
# producer and runtime consumer. Zero dependencies by design -- it is the
# foundation of the zero-USD runtime (M2), so it must never link USD.
# ---------------------------------------------------------------------------
add_library(rigExecBinary STATIC
    libs/rigExecBinary/container.cpp
    libs/rigExecBinary/program.cpp
    libs/rigExecBinary/pose.cpp
    libs/rigExecBinary/geometry.cpp
    libs/rigExecBinary/inputTable.cpp)
target_include_directories(rigExecBinary PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
set_target_properties(rigExecBinary PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------------
# rigExecRuntime: zero-USD .rigexec playback (M2).
#
# The wire decode comes from rigExecBinary (already USD-free); the math is
# vendored (runtimeMath.h); the step bodies are ports. This target must
# never link or include USD: consumers without an OpenUSD install (Godot)
# compile these sources directly. The M4 import check enforces it.
# ---------------------------------------------------------------------------
add_library(rigExecRuntime STATIC
    libs/rigExecRuntime/open.cpp
    libs/rigExecRuntime/exec.cpp
    libs/rigExecRuntime/closure.cpp
    libs/rigExecRuntime/publish.cpp
    libs/rigExecRuntime/kernels.cpp
    libs/rigExecRuntime/pose.cpp
    libs/rigExecRuntime/geometry.cpp
    libs/rigExecRuntime/weights.cpp)
target_include_directories(rigExecRuntime PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
target_link_libraries(rigExecRuntime PUBLIC rigExecBinary)
set_target_properties(rigExecRuntime PROPERTIES POSITION_INDEPENDENT_CODE ON)
if (MSVC)
    target_compile_options(rigExecRuntime PRIVATE /fp:precise)
else ()
    target_compile_options(rigExecRuntime PRIVATE -fno-fast-math)
endif ()

# ---------------------------------------------------------------------------
# rigExecBake: .rigexec baking from a compiled evaluator (M1).
# ---------------------------------------------------------------------------
add_library(rigExecBake SHARED
    libs/rigExecBake/bake.cpp
    libs/rigExecBake/serialize.cpp
    libs/rigExecBake/capture.cpp)
target_include_directories(rigExecBake PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
target_link_libraries(rigExecBake PUBLIC
    rigExec
    rigExecBinary)

# ---------------------------------------------------------------------------
# Tools
#
# rigExecPose evaluates an arbitrary rig stage and prints the result. Unlike
# the tests it asserts nothing about a fixture it owns, which is what makes it
# usable on an asset being authored -- see tools/rigExecPose.cpp.
# ---------------------------------------------------------------------------
add_executable(rigExecBakeTool tools/rigExecBake.cpp)
set_target_properties(rigExecBakeTool PROPERTIES OUTPUT_NAME rigExecBake)
target_link_libraries(rigExecBakeTool PRIVATE rigExecBake)
target_compile_definitions(rigExecBakeTool PRIVATE
    RIGEXEC_SCHEMA_RESOURCE_DIR="${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")

add_executable(rigExecPose tools/rigExecPose.cpp)
target_link_libraries(rigExecPose PRIVATE rigExec rigExecRuntime)
target_compile_definitions(rigExecPose PRIVATE
    RIGEXEC_SCHEMA_RESOURCE_DIR="${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")

# ---------------------------------------------------------------------------
# Tests
#
# Off by default when cross-compiling: these are host command-line executables
# driven by ctest, which cannot run on an iOS/Android device target.
# ---------------------------------------------------------------------------
if (CMAKE_CROSSCOMPILING)
    set(_rigexec_tests_default OFF)
else()
    set(_rigexec_tests_default ON)
endif()
option(RIGEXEC_BUILD_TESTS "Build the RigExec test and probe executables"
       ${_rigexec_tests_default})

if (RIGEXEC_BUILD_TESTS)
enable_testing()

# The tests load the codeless schema from the GENERATED resource directory,
# not the source one: only the generated plugInfo carries the LibraryPath
# that lets Plug load the compute-extent registration on demand.
set(_rigexec_schema_resource_define
    RIGEXEC_SCHEMA_RESOURCE_DIR="${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
# The generated imaging plugInfo, for the test that drives the real
# UsdImaging chain (the rig adapter's keyless registration lives there).
set(_rigexec_imaging_resource_define
    RIGEXEC_IMAGING_RESOURCE_DIR="${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecImaging/resources")

add_executable(testRigExecMath tests/testRigExecMath.cpp)
target_link_libraries(testRigExecMath PRIVATE rigExecMath)
add_test(NAME testRigExecMath COMMAND testRigExecMath)

add_executable(testRigExecSingleChainIk tests/testRigExecSingleChainIk.cpp)
target_link_libraries(testRigExecSingleChainIk PRIVATE rigExecMath)
add_test(NAME testRigExecSingleChainIk COMMAND testRigExecSingleChainIk)

add_executable(testRigExecDualQuat tests/testRigExecDualQuat.cpp)
target_link_libraries(testRigExecDualQuat PRIVATE rigExecMath)
add_test(NAME testRigExecDualQuat COMMAND testRigExecDualQuat)

add_executable(testRigExecSplineIk tests/testRigExecSplineIk.cpp)
target_link_libraries(testRigExecSplineIk PRIVATE rigExecMath)
add_test(NAME testRigExecSplineIk COMMAND testRigExecSplineIk)

# The RBF pose solver against an independent reference implementation,
# which stays the oracle as a CHECKED-IN FIXTURE rather than a live
# import: the reference ran once and every answer it gave is in
# tests/fixtures/psd_parity.json. The fixture path is compiled in so `ctest`
# works from any working directory; argv[1] still overrides it, which is what
# a regenerated fixture is checked with before it is copied over.
add_executable(testRigExecRbf tests/testRigExecRbf.cpp)
target_link_libraries(testRigExecRbf PRIVATE rigExecMath)
target_compile_definitions(testRigExecRbf PRIVATE
    RIGEXEC_RBF_FIXTURE="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/psd_parity.json")
add_test(NAME testRigExecRbf COMMAND testRigExecRbf)

add_executable(testRigExecConstraints tests/testRigExecConstraints.cpp)
target_link_libraries(testRigExecConstraints PRIVATE rigExec)
target_compile_definitions(testRigExecConstraints PRIVATE
    ${_rigexec_schema_resource_define})
add_test(NAME testRigExecConstraints COMMAND testRigExecConstraints)

# The suites this one and two below are re-run under the baked mode's parity
# check.
#
# Nothing in them knows the mode exists: RIGEXEC_EVALUATION_MODE selects the
# initial mode of every evaluator they build, and each generation then runs
# BOTH paths and compares them with exact equality. The dynamic answer is
# still what gets published, so every assertion those suites make is
# unchanged and a disagreement cannot make one of them pass; the disagreement
# is reported on stderr instead, and FAIL_REGULAR_EXPRESSION is what turns it
# into a failure here. This is the only way to hold the program against the
# several hundred rigs these suites already build.
#
# REQUIRE_BAKE, anywhere in the argument list, additionally puts
# RIGEXEC_BAKE_REQUIRED=1 in the environment, which turns a generation that
# fell back to the dynamic path into a reported mismatch instead of a silent
# success. Without it a suite whose fixtures all DECLINE the bake passes
# here having compared nothing. It is opt-in per suite because turning it on
# for a suite whose operators do not bake yet is red by construction: add it
# in the same commit that removes that suite's refusal.
#
# CONE_VERIFY, likewise anywhere in the argument list, registers TWO further
# entries beside the parity one -- <name>Cones_serial and <name>Cones_parallel
# -- which run the same binary with RIGEXEC_BAKED_VERIFY_CONES=1 and each
# schedule in turn. That mode makes every generation run twice: once as the
# cone decided, once whole, from the state the prologue left, comparing every
# slot, counter and diagnostic. It is what holds "a skipped step's values are
# still this frame's values" to account on a suite that drags, which is the
# only place the claim can be wrong. Opt-in, because the mode roughly doubles
# a suite's runtime and only a suite whose generations come from the program
# proves anything under it.
function(rigexec_add_baked_parity_test name)
    set(_args "${ARGN}")
    set(_require_bake OFF)
    list(FIND _args "REQUIRE_BAKE" _require_bake_at)
    if (NOT _require_bake_at EQUAL -1)
        list(REMOVE_AT _args ${_require_bake_at})
        set(_require_bake ON)
    endif()
    set(_cone_verify OFF)
    list(FIND _args "CONE_VERIFY" _cone_verify_at)
    if (NOT _cone_verify_at EQUAL -1)
        list(REMOVE_AT _args ${_cone_verify_at})
        set(_cone_verify ON)
    endif()
    add_test(NAME ${name}BakedParity COMMAND ${name} ${_args})
    set_tests_properties(${name}BakedParity PROPERTIES
        ENVIRONMENT "RIGEXEC_EVALUATION_MODE=parity"
        FAIL_REGULAR_EXPRESSION "baked parity mismatch")
    if (_require_bake)
        # APPEND, because ENVIRONMENT holds ONE ';'-separated value and
        # setting it again would drop the mode above.
        set_property(TEST ${name}BakedParity APPEND PROPERTY ENVIRONMENT
            "RIGEXEC_BAKE_REQUIRED=1")
    endif()
    if (_cone_verify)
        foreach(_schedule serial parallel)
            add_test(NAME ${name}Cones_${_schedule}
                COMMAND ${name} ${_args})
            set_tests_properties(${name}Cones_${_schedule} PROPERTIES
                ENVIRONMENT "RIGEXEC_EVALUATION_MODE=parity"
                FAIL_REGULAR_EXPRESSION "baked parity mismatch")
            set_property(TEST ${name}Cones_${_schedule} APPEND
                PROPERTY ENVIRONMENT
                "RIGEXEC_BAKED_VERIFY_CONES=1"
                "RIGEXEC_BAKED_SCHEDULE=${_schedule}")
            if (_require_bake)
                set_property(TEST ${name}Cones_${_schedule} APPEND
                    PROPERTY ENVIRONMENT "RIGEXEC_BAKE_REQUIRED=1")
            endif()
        endforeach()
    endif()
endfunction()
# CONE_VERIFY, not REQUIRE_BAKE, and now for ONE reason rather than three.
# The provider-ladder work took the animated rest:tx and the animated
# posed:space this comment used to name: both are per-frame inputs of the
# compose, and the suite's generations that carry them bake. What is left is
# eight generations of TestConnectedParentSpaceSolverInputs, which connects
# a provider's default:space to another provider's parent:space -- a CONNECTED
# SPACE, whose value is a posed frame the dynamic path re-evaluates through
# exec at defined points in the walk, carrying a base and a final phase into
# namespace descendants. That is the connected-space-provider feature, not a
# ladder value read off the stage, and the flag goes on with it.
#
# The WARNING such a run prints names "connected default:space", not
# "connected-space provider": IsBakeable sorts its reasons and the evaluator
# reports the first one. Both come from the same rig and the same feature --
# read the whole reasons vector, not the warning, when chasing this.
rigexec_add_baked_parity_test(testRigExecConstraints CONE_VERIFY)

# The per-example parity entries: one named red/green signal per shipped rig.
#
# rigExecPose runs both paths in one generation and compares them exactly;
# --require-baked is what stops the entry passing vacuously when the rig
# declines the bake, and --guides adds the solver-guide frames, which are a
# compared domain the tool otherwise leaves empty on both sides. The frames,
# and which rigs bake today, come from tests/exampleFixtures.cmake -- the
# same table the C++ drag suite reads.
#
#   NAME      the ctest entry is example_parity_<name>
#   RIG       stage path under examples/
#   FRAMES    comma-separated, passed straight to --frames
#   GUIDES    ask for the solver guides
#   DISABLED  the rig does not bake yet; registered so it is visible as
#             "Not Run" rather than missing, and enabled by deleting the
#             keyword once its operator group lands
function(rigexec_add_example_parity_test)
    cmake_parse_arguments(A "GUIDES;DISABLED" "NAME;RIG;FRAMES" "" ${ARGN})
    # DEFINED, not truthiness: a fixture whose frame list is the single
    # frame 0 is a perfectly good entry, and CMake reads "0" as false.
    if (NOT DEFINED A_NAME OR NOT DEFINED A_RIG OR NOT DEFINED A_FRAMES)
        message(FATAL_ERROR "rigexec_add_example_parity_test: "
                            "NAME, RIG and FRAMES are all required")
    endif()
    # A misspelled keyword is otherwise dropped in silence, and the entry
    # quietly registers as enabled -- the one mistake this function must not
    # swallow, since DISABLED is how a non-baking rig stays green.
    if (A_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "rigexec_add_example_parity_test: unknown "
                            "argument(s): ${A_UNPARSED_ARGUMENTS}")
    endif()
    set(_guides "")
    if (A_GUIDES)
        set(_guides "--guides")
    endif()
    add_test(NAME example_parity_${A_NAME} COMMAND rigExecPose
        "${CMAKE_CURRENT_SOURCE_DIR}/examples/${A_RIG}"
        --frames "${A_FRAMES}" --mode parity --require-baked ${_guides})
    # Two regexes, not one: ';' is CMake's list separator. The first is the
    # evaluator's own warning, the second this tool's own two gates.
    #
    # The mode is on the command line as well as in the environment, and both
    # say the same thing; the variable is what fixes the mode of the evaluator
    # from its construction, which is the state the sibling suites run in.
    set_tests_properties(example_parity_${A_NAME} PROPERTIES
        ENVIRONMENT "RIGEXEC_EVALUATION_MODE=parity"
        FAIL_REGULAR_EXPRESSION "baked parity mismatch;FAIL:")
    # The tool finds the codeless schema through its compiled-in resource
    # directory; this is for a run that inherits a different PXR_PLUGINPATH.
    # APPEND, because ENVIRONMENT is one value.
    set(_schema_resources
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    set_property(TEST example_parity_${A_NAME} APPEND PROPERTY ENVIRONMENT
        "PXR_PLUGINPATH_NAME=${_schema_resources}")
    if (A_DISABLED)
        set_tests_properties(example_parity_${A_NAME} PROPERTIES
            DISABLED TRUE)
    endif()
endfunction()

include("${CMAKE_CURRENT_SOURCE_DIR}/tests/exampleFixtures.cmake")
foreach(_fixture IN LISTS RIGEXEC_EXAMPLE_FIXTURES)
    string(REPLACE "|" ";" _fixture_fields "${_fixture}")
    list(GET _fixture_fields 0 _fixture_stage)
    list(GET _fixture_fields 1 _fixture_frames)
    list(GET _fixture_fields 6 _fixture_bakes)
    # The entry name is the stage path with the separators CMake test names
    # cannot carry replaced: biped/Biped_anim.usda -> biped_Biped_anim.
    string(REGEX REPLACE "\\.(usda|usd)$" "" _fixture_name "${_fixture_stage}")
    string(REPLACE "/" "_" _fixture_name "${_fixture_name}")
    if (_fixture_bakes STREQUAL "YES")
        rigexec_add_example_parity_test(NAME ${_fixture_name}
            RIG ${_fixture_stage} FRAMES ${_fixture_frames} GUIDES)
    else()
        # Registered but not run: the rig refuses the bake today, so
        # --require-baked would fail it. The table's blocker column names the
        # operator group that turns each one on; deleting DISABLED is that
        # group's job, in the commit that removes the refusal.
        rigexec_add_example_parity_test(NAME ${_fixture_name}
            RIG ${_fixture_stage} FRAMES ${_fixture_frames} GUIDES DISABLED)
    endif()
endforeach()

# The other half of the same table, in process: every fixture that bakes is
# compiled under the parity check, evaluated across its frames, then dragged
# on its control avar and on an operator input. A drag the program cannot
# place is not wrong, it is slow -- the generation falls back and every value
# still agrees -- so the assertion that matters is that the generations came
# from the program.
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/tests/exampleFixtures.h.in"
               "${CMAKE_CURRENT_BINARY_DIR}/tests/rigExecExampleFixtures.h"
               @ONLY)
add_executable(testRigExecExampleParity tests/testRigExecExampleParity.cpp)
target_link_libraries(testRigExecExampleParity PRIVATE rigExec)
target_include_directories(testRigExecExampleParity PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/tests")
target_compile_definitions(testRigExecExampleParity PRIVATE
    ${_rigexec_schema_resource_define})
add_test(NAME testRigExecExampleParity COMMAND testRigExecExampleParity
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
# It sets the mode itself, so RIGEXEC_EVALUATION_MODE must stay unset here --
# but a fallback inside it must still be loud.
set_tests_properties(testRigExecExampleParity PROPERTIES
    ENVIRONMENT "RIGEXEC_BAKE_REQUIRED=1"
    FAIL_REGULAR_EXPRESSION "baked parity mismatch")
# The same sweep with the cone verifier on, in both schedules. Every fixture
# that bakes is evaluated across its frames and then dragged twice, so this
# is where "the run skipped the right half" is measured against "the run
# skipped nothing" on several hundred generations rather than on one.
foreach(_schedule serial parallel)
    add_test(NAME testRigExecExampleParityCones_${_schedule}
        COMMAND testRigExecExampleParity
        "${CMAKE_CURRENT_SOURCE_DIR}/examples")
    set_tests_properties(testRigExecExampleParityCones_${_schedule} PROPERTIES
        ENVIRONMENT "RIGEXEC_BAKE_REQUIRED=1"
        FAIL_REGULAR_EXPRESSION "baked parity mismatch")
    set_property(TEST testRigExecExampleParityCones_${_schedule} APPEND
        PROPERTY ENVIRONMENT
        "RIGEXEC_BAKED_VERIFY_CONES=1"
        "RIGEXEC_BAKED_SCHEDULE=${_schedule}")
endforeach()

# The geometry operations that bake with no shipped rig to say so:
# blendShape, curvenet, emitGuidePoints, ribbon and the two read phases that
# come out of the run's snapshot store. Every rig that authors them is held
# on the dynamic path by a refusal belonging to another group, so this suite
# unbinds that refusal in memory and holds the operations themselves against
# the dynamic path; see the file's own header. It is registered by hand
# rather than through rigexec_add_baked_parity_test because, like the
# example sweep above, it fixes its own evaluation mode.
add_executable(testRigExecGeometryOpsBakedParity
    tests/testRigExecGeometryOpsBakedParity.cpp)
target_link_libraries(testRigExecGeometryOpsBakedParity PRIVATE rigExec)
target_compile_definitions(testRigExecGeometryOpsBakedParity PRIVATE
    ${_rigexec_schema_resource_define})
add_test(NAME testRigExecGeometryOpsBakedParity
    COMMAND testRigExecGeometryOpsBakedParity
    "${CMAKE_CURRENT_SOURCE_DIR}/examples"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/rigExecRibbonProbe.usda")
# The mode is the suite's own, so RIGEXEC_EVALUATION_MODE stays unset here --
# but a generation that fell back must still be loud.
set_tests_properties(testRigExecGeometryOpsBakedParity PROPERTIES
    ENVIRONMENT "RIGEXEC_BAKE_REQUIRED=1"
    FAIL_REGULAR_EXPRESSION "baked parity mismatch")
foreach(_schedule serial parallel)
    add_test(NAME testRigExecGeometryOpsBakedParityCones_${_schedule}
        COMMAND testRigExecGeometryOpsBakedParity
        "${CMAKE_CURRENT_SOURCE_DIR}/examples"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/rigExecRibbonProbe.usda")
    set_tests_properties(testRigExecGeometryOpsBakedParityCones_${_schedule}
        PROPERTIES ENVIRONMENT "RIGEXEC_BAKE_REQUIRED=1"
        FAIL_REGULAR_EXPRESSION "baked parity mismatch")
    set_property(TEST testRigExecGeometryOpsBakedParityCones_${_schedule}
        APPEND PROPERTY ENVIRONMENT
        "RIGEXEC_BAKED_VERIFY_CONES=1"
        "RIGEXEC_BAKED_SCHEDULE=${_schedule}")
endforeach()

add_executable(testRigExecStandalone tests/testRigExecStandalone.cpp)
target_link_libraries(testRigExecStandalone PRIVATE rigExecStandalone)
target_compile_definitions(testRigExecStandalone PRIVATE ${_rigexec_schema_resource_define})
add_test(NAME testRigExecStandalone COMMAND testRigExecStandalone)

# The .rigexec container + bake conformance. The walk covers every fixture
# in tests/exampleFixtures.cmake (same table as the parity entries); argv[1]
# is either the examples directory (table walk) or a stage (ad-hoc bake).
add_executable(testRigExecRuntimeMath tests/testRigExecRuntimeMath.cpp)
target_link_libraries(testRigExecRuntimeMath PRIVATE rigExecRuntime gf)
add_test(NAME testRigExecRuntimeMath COMMAND testRigExecRuntimeMath)

add_executable(testRigExecRuntimeLoader tests/testRigExecRuntimeLoader.cpp)
target_link_libraries(testRigExecRuntimeLoader PRIVATE rigExecRuntime rigExecBake)
target_include_directories(testRigExecRuntimeLoader PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/tests")
target_compile_definitions(testRigExecRuntimeLoader PRIVATE
    ${_rigexec_schema_resource_define}
    RIGEXEC_EXAMPLES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/examples")
add_test(NAME testRigExecRuntimeLoader COMMAND testRigExecRuntimeLoader
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

# Family parity tests: baked program vs runtime, per family. Each is
# owned by that family's port worker; the bodies land with the ports.
add_executable(testRigExecRuntimePose tests/testRigExecRuntimePose.cpp)
target_link_libraries(testRigExecRuntimePose PRIVATE rigExecRuntime rigExecBake)
target_include_directories(testRigExecRuntimePose PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/tests")
target_compile_definitions(testRigExecRuntimePose PRIVATE
    ${_rigexec_schema_resource_define}
    RIGEXEC_EXAMPLES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/examples")
add_test(NAME testRigExecRuntimePose COMMAND testRigExecRuntimePose
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecRuntimeWeights tests/testRigExecRuntimeWeights.cpp)
target_link_libraries(testRigExecRuntimeWeights PRIVATE rigExecRuntime rigExecBake)
target_include_directories(testRigExecRuntimeWeights PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/tests")
target_compile_definitions(testRigExecRuntimeWeights PRIVATE
    ${_rigexec_schema_resource_define}
    RIGEXEC_EXAMPLES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/examples")
add_test(NAME testRigExecRuntimeWeights COMMAND testRigExecRuntimeWeights
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecRuntimeGeometry tests/testRigExecRuntimeGeometry.cpp)
target_link_libraries(testRigExecRuntimeGeometry PRIVATE rigExecRuntime rigExecBake)
target_include_directories(testRigExecRuntimeGeometry PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/tests")
target_compile_definitions(testRigExecRuntimeGeometry PRIVATE
    ${_rigexec_schema_resource_define}
    RIGEXEC_EXAMPLES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/examples")
add_test(NAME testRigExecRuntimeGeometry COMMAND testRigExecRuntimeGeometry
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecBinary tests/testRigExecBinary.cpp)
target_link_libraries(testRigExecBinary PRIVATE rigExecBake)
target_include_directories(testRigExecBinary PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/tests")
target_compile_definitions(testRigExecBinary PRIVATE
    ${_rigexec_schema_resource_define}
    RIGEXEC_EXAMPLES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/examples")
add_test(NAME testRigExecBinary COMMAND testRigExecBinary
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

# The M2 end-to-end gate, one entry per baking fixture: bake the stage to a
# .rigexec with the CLI, then prove the zero-USD runtime replays it bit for
# bit (rigExecPose --verify-binary runs the evaluator in parity mode, so a
# dynamic==baked disagreement fails the frame before the binary is compared).
# The stages and frame lists come from tests/exampleFixtures.cmake -- the
# same table as every other example sweep. Enabled by M2 integrate once the
# family ports (pose, weights, geometry) and the compile-diagnostics seed
# replay landed and the first fixture verified.
function(rigexec_add_verify_binary_test)
    cmake_parse_arguments(A "" "NAME;RIG;FRAMES" "" ${ARGN})
    if (NOT DEFINED A_NAME OR NOT DEFINED A_RIG OR NOT DEFINED A_FRAMES)
        message(FATAL_ERROR "rigexec_add_verify_binary_test: "
                            "NAME, RIG and FRAMES are all required")
    endif()
    if (A_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "rigexec_add_verify_binary_test: unknown "
                            "argument(s): ${A_UNPARSED_ARGUMENTS}")
    endif()
    set(_out "${CMAKE_CURRENT_BINARY_DIR}/verify-binary/${A_NAME}.rigexec")
    add_test(NAME verify_binary_${A_NAME} COMMAND "${CMAKE_COMMAND}"
        "-DBAKE=$<TARGET_FILE:rigExecBakeTool>" "-DPOSE=$<TARGET_FILE:rigExecPose>"
        "-DSTAGE=${CMAKE_CURRENT_SOURCE_DIR}/examples/${A_RIG}"
        "-DFRAMES=${A_FRAMES}" "-DOUT=${_out}"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/runVerifyBinary.cmake")
    set(_schema_resources
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    set_property(TEST verify_binary_${A_NAME} APPEND PROPERTY ENVIRONMENT
        "PXR_PLUGINPATH_NAME=${_schema_resources}")
endfunction()

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/verify-binary")
foreach(_fixture IN LISTS RIGEXEC_EXAMPLE_FIXTURES)
    string(REPLACE "|" ";" _fixture_fields "${_fixture}")
    list(GET _fixture_fields 0 _fixture_stage)
    list(GET _fixture_fields 1 _fixture_frames)
    list(GET _fixture_fields 6 _fixture_bakes)
    string(REGEX REPLACE "\\.(usda|usd)$" "" _fixture_name "${_fixture_stage}")
    string(REPLACE "/" "_" _fixture_name "${_fixture_name}")
    if (_fixture_bakes STREQUAL "YES")
        rigexec_add_verify_binary_test(NAME ${_fixture_name}
            RIG ${_fixture_stage} FRAMES ${_fixture_frames})
    endif()
endforeach()

add_executable(testRigExecPack tests/testRigExecPack.cpp)
target_link_libraries(testRigExecPack PRIVATE rigExecStandalone)
target_compile_definitions(testRigExecPack PRIVATE ${_rigexec_schema_resource_define})
add_test(NAME testRigExecPack COMMAND testRigExecPack)

add_executable(testRigExecCurvenet tests/testRigExecCurvenet.cpp)
target_link_libraries(testRigExecCurvenet PRIVATE rigExecMath)
add_test(NAME testRigExecCurvenet COMMAND testRigExecCurvenet)

add_executable(testRigExecCurvenetAdjustments tests/testRigExecCurvenetAdjustments.cpp)
target_link_libraries(testRigExecCurvenetAdjustments PRIVATE rigExecMath)
add_test(NAME testRigExecCurvenetAdjustments COMMAND testRigExecCurvenetAdjustments)

add_executable(testRigExecCurvenetAdjuster tests/testRigExecCurvenetAdjuster.cpp)
target_link_libraries(testRigExecCurvenetAdjuster PRIVATE rigExec rigExecRigging)
target_compile_definitions(testRigExecCurvenetAdjuster PRIVATE ${_rigexec_schema_resource_define})
add_test(NAME testRigExecCurvenetAdjuster COMMAND testRigExecCurvenetAdjuster)
# The curvenet adjuster's only baked coverage. Its own fixtures pin
# cpuParityMode, which asks for the dynamic path, so the suite grew one
# generation that does not -- over a net carrying an xform op, which is what
# exercises the net -> asset ladder the adjusted control frames publish
# through. REQUIRE_BAKE is what stops that generation passing vacuously: it
# asserts only that the pose is valid and agreed, which a generation the
# program never ran would satisfy too. The suite's LATER fixtures connect
# parent:space on a provider, which the program still refuses -- but they
# reach it on the cpuParityMode evaluator, which asks for the dynamic path,
# and a bake required is not reported against a run that asked for it. So
# the flag guards the one generation it should and is silent on the rest.
# CONE_VERIFY because this is the only place an adjuster runs at all, and
# the control frames it leaves on the revision are per-revision state no
# declared slot covers -- the verifier re-running the whole generation
# beside the cone is what holds that to account.
rigexec_add_baked_parity_test(testRigExecCurvenetAdjuster
    REQUIRE_BAKE CONE_VERIFY)

add_executable(testRigExecWeightFields tests/testRigExecWeightFields.cpp)
target_link_libraries(testRigExecWeightFields PRIVATE rigExecMath)
add_test(NAME testRigExecWeightFields COMMAND testRigExecWeightFields)

# The baked evaluation mode against the dynamic path, on the shipped biped.
add_executable(testRigExecBakedMode tests/testRigExecBakedMode.cpp)
target_link_libraries(testRigExecBakedMode PRIVATE rigExec)
target_compile_definitions(testRigExecBakedMode PRIVATE ${_rigexec_schema_resource_define})
add_test(NAME testRigExecBakedMode COMMAND testRigExecBakedMode
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

# `uniform bool rigExec:baked` on the rig root, and the precedence ladder
# around it. Registered TWICE: the second entry is the only way to test that
# RIGEXEC_EVALUATION_MODE outranks the attribute, since the variable is read
# once per process and a second evaluator in the same binary cannot see a
# different answer.
add_executable(testRigExecBakedAttribute tests/testRigExecBakedAttribute.cpp)
target_link_libraries(testRigExecBakedAttribute PRIVATE rigExec)
target_compile_definitions(testRigExecBakedAttribute
    PRIVATE ${_rigexec_schema_resource_define})
add_test(NAME testRigExecBakedAttribute COMMAND testRigExecBakedAttribute
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
add_test(NAME testRigExecBakedAttributeEnvironment
    COMMAND testRigExecBakedAttribute
    "${CMAKE_CURRENT_SOURCE_DIR}/examples" --expect-environment)
set_tests_properties(testRigExecBakedAttributeEnvironment PROPERTIES
    ENVIRONMENT "RIGEXEC_EVALUATION_MODE=dynamic")

# The step graph behind the baked program: the edges, not the values.
add_executable(testRigExecBakedSchedule tests/testRigExecBakedSchedule.cpp)
target_link_libraries(testRigExecBakedSchedule PRIVATE rigExec)
target_compile_definitions(testRigExecBakedSchedule PRIVATE ${_rigexec_schema_resource_define})
add_test(NAME testRigExecBakedSchedule COMMAND testRigExecBakedSchedule
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
# The same suite with the cone verifier on, in both schedules. This is the
# only place the verifier's own comparison is exercised: the suite drives a
# NaN into a published packet on purpose, and a comparison that called a NaN
# different from itself reported mismatches on a generation whose cone had
# skipped nothing. Registered by hand rather than through
# rigexec_add_baked_parity_test because this suite is not a parity suite --
# it asserts on the graph, not on two paths agreeing.
foreach(_schedule serial parallel)
    add_test(NAME testRigExecBakedScheduleCones_${_schedule}
        COMMAND testRigExecBakedSchedule
        "${CMAKE_CURRENT_SOURCE_DIR}/examples")
    set_tests_properties(testRigExecBakedScheduleCones_${_schedule} PROPERTIES
        ENVIRONMENT "RIGEXEC_BAKED_VERIFY_CONES=1"
        FAIL_REGULAR_EXPRESSION "baked parity mismatch")
    set_property(TEST testRigExecBakedScheduleCones_${_schedule} APPEND
        PROPERTY ENVIRONMENT "RIGEXEC_BAKED_SCHEDULE=${_schedule}")
endforeach()

# The solver half of the baked program, on rigs the examples do not have:
# every shipped ribbon and twist stage also carries geometry the program
# still declines, so without these the solvers would be compared with the
# dynamic path only through a generation neither of them baked. Registered
# under the cone verifier as well, because the fixtures are small enough
# that a run skips most of the program.
add_executable(testRigExecSolverBake tests/testRigExecSolverBake.cpp)
target_link_libraries(testRigExecSolverBake PRIVATE rigExec)
target_compile_definitions(testRigExecSolverBake
    PRIVATE ${_rigexec_schema_resource_define})
add_test(NAME testRigExecSolverBake COMMAND testRigExecSolverBake
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
rigexec_add_baked_parity_test(testRigExecSolverBake REQUIRE_BAKE CONE_VERIFY
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecSolverStacking tests/testRigExecSolverStacking.cpp)
target_link_libraries(testRigExecSolverStacking PRIVATE rigExec)
target_compile_definitions(testRigExecSolverStacking
    PRIVATE ${_rigexec_schema_resource_define})
add_test(NAME testRigExecSolverStacking COMMAND testRigExecSolverStacking
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
# No REQUIRE_BAKE: the checkpoint case deliberately authors a read phase that
# names a solver, which refuses the bake by design (there is no solver
# equivalent of the baked phased-read store). Every other fixture bakes, so
# parity and the cone verifier still run over two solver commits on one slot.
rigexec_add_baked_parity_test(testRigExecSolverStacking CONE_VERIFY
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecArm tests/testRigExecArm.cpp)
target_link_libraries(testRigExecArm PRIVATE rigExec)
target_compile_definitions(testRigExecArm PRIVATE ${_rigexec_schema_resource_define})
add_test(NAME testRigExecArm COMMAND testRigExecArm
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
# REQUIRE_BAKE and CONE_VERIFY as of the Phase 3 merge: this is the suite
# that exercises blendShape, volumeCorrect, ribbon and twist TOGETHER, and
# every one of those now comes from the program. It was registered bare while
# the arm's geometry and its two solver types belonged to different groups;
# with both landed it runs clean under the bake requirement and under the
# cone verifier in each schedule.
rigexec_add_baked_parity_test(testRigExecArm REQUIRE_BAKE CONE_VERIFY
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecImaging tests/testRigExecImaging.cpp)
target_link_libraries(testRigExecImaging PRIVATE rigExecImaging)
target_compile_definitions(testRigExecImaging PRIVATE
    ${_rigexec_schema_resource_define} ${_rigexec_imaging_resource_define})
add_test(NAME testRigExecImaging COMMAND testRigExecImaging
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecImagingPlayback tests/testRigExecImagingPlayback.cpp)
target_link_libraries(testRigExecImagingPlayback PRIVATE rigExecImaging rigExecBake)
target_include_directories(testRigExecImagingPlayback PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/tests")
target_compile_definitions(testRigExecImagingPlayback PRIVATE
    ${_rigexec_schema_resource_define} ${_rigexec_imaging_resource_define})
add_test(NAME testRigExecImagingPlayback COMMAND testRigExecImagingPlayback
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecTouchPose tests/testRigExecTouchPose.cpp)
target_link_libraries(testRigExecTouchPose PRIVATE rigExecImaging)
add_test(NAME testRigExecTouchPose COMMAND testRigExecTouchPose)

add_executable(testRigExecVolumeWeights tests/testRigExecVolumeWeights.cpp)
target_link_libraries(testRigExecVolumeWeights PRIVATE rigExec)
target_compile_definitions(testRigExecVolumeWeights PRIVATE ${_rigexec_schema_resource_define})
add_test(NAME testRigExecVolumeWeights COMMAND testRigExecVolumeWeights
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
# And again through the BAKED program. The suite's default turns the baked
# path off, because cpuParityMode is what it was written for -- an
# independently written CPU resolver checked against exec on every shape,
# both plane bounds, invert, strength, sampleSource, every combine mode and
# both sample phases. That is also the widest volumetric coverage in the
# tree, so it is worth running a second time with the oracle off and the
# program on: same assertions, both paths compared exactly in every
# generation, and RIGEXEC_BAKE_REQUIRED so a rig that quietly declined
# cannot pass. RIGEXEC_TEST_BAKED_PATH is what the suite reads to choose.
rigexec_add_baked_parity_test(testRigExecVolumeWeights REQUIRE_BAKE
    CONE_VERIFY "${CMAKE_CURRENT_SOURCE_DIR}/examples")
foreach(_entry testRigExecVolumeWeightsBakedParity
               testRigExecVolumeWeightsCones_serial
               testRigExecVolumeWeightsCones_parallel)
    set_property(TEST ${_entry} APPEND PROPERTY ENVIRONMENT
        "RIGEXEC_TEST_BAKED_PATH=1")
endforeach()

add_executable(testRigExecWeightOverlay tests/testRigExecWeightOverlay.cpp)
target_link_libraries(testRigExecWeightOverlay PRIVATE rigExecImaging)
target_compile_definitions(testRigExecWeightOverlay PRIVATE ${_rigexec_schema_resource_define})
add_test(NAME testRigExecWeightOverlay COMMAND testRigExecWeightOverlay
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecMoverGraph tests/testRigExecMoverGraph.cpp)
target_link_libraries(testRigExecMoverGraph PRIVATE rigExec)
add_test(NAME testRigExecMoverGraph COMMAND testRigExecMoverGraph)

# The epoch-fixed skin layout: what has to invalidate it, and what Compile
# has to refuse to cache in the first place.
add_executable(testRigExecSkinTopology tests/testRigExecSkinTopology.cpp)
target_link_libraries(testRigExecSkinTopology PRIVATE rigExec)
target_compile_definitions(testRigExecSkinTopology PRIVATE
    ${_rigexec_schema_resource_define})
add_test(NAME testRigExecSkinTopology COMMAND testRigExecSkinTopology)

# The static input cache's three rules -- admission, typing, and the
# worker-thread bypass -- asserted directly, because each one's failure mode
# is either order-dependent or probabilistic through a rig.
add_executable(testRigExecStaticInputCache
    tests/testRigExecStaticInputCache.cpp)
target_link_libraries(testRigExecStaticInputCache PRIVATE rigExec)
add_test(NAME testRigExecStaticInputCache COMMAND testRigExecStaticInputCache)

# When the epoch may hold the rests once, and what has to put them back on
# the per-frame path: animated, connected and chain-written rest channels,
# before and after Compile, and a drag standing on one.
# The pose-interpolator phase, evaluated: the weights, the LOCAL delta, the
# connection that carries a weight into a blend channel, the two enables and
# parity with the solver driven directly. Self-contained -- it builds its own
# stage -- so it needs no examples directory.
add_executable(testRigExecPoseInterpolator
    tests/testRigExecPoseInterpolator.cpp)
target_link_libraries(testRigExecPoseInterpolator PRIVATE rigExec rigExecMath)
target_compile_definitions(testRigExecPoseInterpolator PRIVATE
    ${_rigexec_schema_resource_define})
add_test(NAME testRigExecPoseInterpolator COMMAND testRigExecPoseInterpolator)

add_executable(testRigExecEpochRests tests/testRigExecEpochRests.cpp)
target_link_libraries(testRigExecEpochRests PRIVATE rigExec)
target_compile_definitions(testRigExecEpochRests PRIVATE
    ${_rigexec_schema_resource_define})
add_test(NAME testRigExecEpochRests COMMAND testRigExecEpochRests
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
# And again under the parity check. A rest that moves on one path and not the
# other is exactly what the comparator is for, and this is the suite that
# moves rests on purpose.
#
# REQUIRE_BAKE as of the provider-ladder work, which is the day the comment
# here said the flag would go on. The three things this suite authors on
# purpose -- an animated or connected rest channel, a property chain writing
# rest:space, and a drag standing on a rest -- are all per-frame inputs of
# the compose now, so every generation of this suite comes from the program
# and a fallback here is a regression rather than a missing feature. It is
# the sharpest entry in the tree for that work: nothing else moves a rest on
# purpose, and a program that quietly went back to folding one would show up
# here as a fallback and nowhere else.
#
# The Phase 4 deferrals branch authored two DELIBERATE negatives against this
# same suite -- an intervening Xform above a provider and an animated one,
# both still refused, both comparing their fallback with a plain dynamic
# evaluator. A declining fixture cannot live under a bake requirement (the
# requirement reports a fallback with the same "baked parity mismatch"
# prefix a real disagreement carries), so at the merge they moved to
# testRigExecBakedMode, which is where this tree keeps its deliberate
# negatives and is deliberately not a parity entry. Both intents survive:
# the refusals still have live fixtures that assert them by name, and this
# suite stays the scoreboard that has to read zero.
rigexec_add_baked_parity_test(testRigExecEpochRests REQUIRE_BAKE CONE_VERIFY
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

# Bit-identical results from the parallel per-point kernels, both ways round:
# against the scalar reference, and against the same binary run serially.
add_executable(testRigExecParallelKernels tests/testRigExecParallelKernels.cpp)
target_link_libraries(testRigExecParallelKernels PRIVATE rigExec work)
add_test(NAME testRigExecParallelKernels COMMAND testRigExecParallelKernels)
add_test(NAME testRigExecParallelKernelsSerial
    COMMAND testRigExecParallelKernels)
set_tests_properties(testRigExecParallelKernelsSerial PROPERTIES
    ENVIRONMENT "RIGEXEC_ENABLE_PARALLEL_EVAL=0")

# One task per chain inside a dependency level: the same values as the serial
# walk, the same diagnostics order, and the same classification. Run twice,
# the second time down the serial branch.
add_executable(testRigExecChainLevels tests/testRigExecChainLevels.cpp)
target_link_libraries(testRigExecChainLevels PRIVATE rigExec work)
target_compile_definitions(testRigExecChainLevels PRIVATE
    ${_rigexec_schema_resource_define})
add_test(NAME testRigExecChainLevels COMMAND testRigExecChainLevels)
add_test(NAME testRigExecChainLevelsSerial COMMAND testRigExecChainLevels)
set_tests_properties(testRigExecChainLevelsSerial PROPERTIES
    ENVIRONMENT "RIGEXEC_ENABLE_PARALLEL_EVAL=0")
# And again, repeatedly. The two guards that make the level-parallel walk
# memory-safe -- the pre-created graph nodes and the static cache's owning
# thread -- fail probabilistically when they are broken, not deterministically:
# a broken build passes a single run roughly three times in four. Each entry
# above asserts the invariant directly; this one runs the whole file until it
# fails, so a regression that slips past the invariants still lands red rather
# than one CI run in twenty. The file costs well under a tenth of a second.
add_test(NAME testRigExecChainLevelsRepeat
    COMMAND testRigExecChainLevels 20)

add_executable(testRigExecProfiler tests/testRigExecProfiler.cpp)
target_link_libraries(testRigExecProfiler PRIVATE rigExec)
add_test(NAME testRigExecProfiler COMMAND testRigExecProfiler)

# Measures what the compile's path bookkeeping costs; asserts nothing, so it
# is built but deliberately NOT registered with ctest.
add_executable(benchPathLookup tests/benchPathLookup.cpp)
target_link_libraries(benchPathLookup PRIVATE rigExec)

add_executable(testRigExecInteractive tests/testRigExecInteractive.cpp)
target_link_libraries(testRigExecInteractive PRIVATE rigExec)
target_compile_definitions(testRigExecInteractive PRIVATE
    ${_rigexec_schema_resource_define})
add_test(NAME testRigExecInteractive COMMAND testRigExecInteractive)
# REQUIRE_BAKE: every generation this suite runs comes from the program
# today, so a fallback to the dynamic path is a regression rather than a
# refusal still waiting on an operator group. The other three baked-parity
# suites still fall back (plain-Xformable constraint targets, SingleChainIK,
# weight objects on movers, authored posed:space and rest overrides), so
# they get REQUIRE_BAKE in the commit that bakes their operators.
# The drag suite, and the one whose every generation comes from the program:
# the natural home for the cone verifier, because an interactive override is
# what makes two consecutive frames differ in one value and nothing else.
rigexec_add_baked_parity_test(testRigExecInteractive REQUIRE_BAKE CONE_VERIFY)

add_executable(testRigExecDefaultSpaces tests/testRigExecDefaultSpaces.cpp)
target_link_libraries(testRigExecDefaultSpaces PRIVATE rigExec)
target_compile_definitions(testRigExecDefaultSpaces PRIVATE
    ${_rigexec_schema_resource_define})
add_test(NAME testRigExecDefaultSpaces COMMAND testRigExecDefaultSpaces)

add_executable(testRigExecSchemaAuthoring
    tests/testRigExecSchemaAuthoring.cpp)
target_link_libraries(testRigExecSchemaAuthoring PRIVATE rigExecRigging)
target_compile_definitions(testRigExecSchemaAuthoring PRIVATE
    ${_rigexec_schema_resource_define})
add_test(NAME testRigExecSchemaAuthoring COMMAND testRigExecSchemaAuthoring)
add_test(NAME testRigExecSchemaAuthoringMissing
    COMMAND testRigExecSchemaAuthoring --missing-schema)
set_tests_properties(testRigExecSchemaAuthoringMissing PROPERTIES
    ENVIRONMENT "PXR_PLUGINPATH_NAME=")

add_executable(testRigExecRigBuilder tests/testRigExecRigBuilder.cpp)
target_link_libraries(testRigExecRigBuilder PRIVATE rigExecRigging)
target_compile_definitions(testRigExecRigBuilder PRIVATE
    ${_rigexec_schema_resource_define})
add_test(NAME testRigExecRigBuilder COMMAND testRigExecRigBuilder)

# The engine authors nothing: asserted by whole-scene equality rather than by
# probing for the specific artifacts the deleted compiler used to leave.
add_executable(testRigExecNoAuthoring tests/testRigExecNoAuthoring.cpp)
target_link_libraries(testRigExecNoAuthoring PRIVATE rigExecImaging)
target_compile_definitions(testRigExecNoAuthoring PRIVATE ${_rigexec_schema_resource_define})
add_test(NAME testRigExecNoAuthoring COMMAND testRigExecNoAuthoring
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

# Native bounds in a host that runs NO RigExec code: deliberately links
# usdGeom and plug only, never rigExecImaging, so the compute-extent
# function can only arrive through Plug's load-on-demand path.
add_executable(testRigExecBounds tests/testRigExecBounds.cpp)
target_link_libraries(testRigExecBounds PRIVATE plug tf gf usd usdGeom)
target_compile_definitions(testRigExecBounds PRIVATE ${_rigexec_schema_resource_define})
add_dependencies(testRigExecBounds rigExecImaging)
add_test(NAME testRigExecBounds COMMAND testRigExecBounds
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(probeCodingError tests/probeCodingError.cpp)
target_link_libraries(probeCodingError PRIVATE rigExec)
target_compile_definitions(probeCodingError PRIVATE ${_rigexec_schema_resource_define})

# End-to-end against the real UsdImagingCreateSceneIndices chain. Not a unit
# test: it needs the RigExec scene index plugin discoverable through Plug, so
# it runs from run_probe-style environments rather than bare ctest.
add_executable(probeImagingPipeline tests/probeImagingPipeline.cpp)
target_link_libraries(probeImagingPipeline PRIVATE rigExecImaging usdImaging usdGeom)
target_compile_definitions(probeImagingPipeline PRIVATE ${_rigexec_schema_resource_define})

# The non-public Esf/Exec seam must qualify separately on each stock USD
# package. A failed link is a compatibility gate, not a reason to change USD.
add_executable(probeEsfCompatibility EXCLUDE_FROM_ALL tests/probeEsfCompatibility.cpp)
target_link_libraries(probeEsfCompatibility PRIVATE exec esfUsd usd)

endif()  # RIGEXEC_BUILD_TESTS

# ---------------------------------------------------------------------------
# Python bindings: the _rigexec pybind11 module and the rigexec package.
#
# Requires a CPython with development headers plus pybind11 in that
# interpreter (pip install pybind11). The default is ON only when both are
# actually available, so a plain configure works on machines without Python
# dev dependencies; an explicit -DRIGEXEC_BUILD_PYTHON=ON still fails loudly
# inside python/ with installation guidance.
# ---------------------------------------------------------------------------
if (CMAKE_CROSSCOMPILING)
    set(_rigexec_python_default OFF)
else()
    # Probe quietly: the interpreter must be found with dev headers, and the
    # pip-installed pybind11 package must report its CMake directory.
    find_package(Python3 COMPONENTS Interpreter Development.Module QUIET)
    if (Python3_FOUND AND Python3_Development.Module_FOUND)
        execute_process(
            COMMAND "${Python3_EXECUTABLE}" -m pybind11 --cmakedir
            OUTPUT_VARIABLE _rigexec_pybind11_probe
            OUTPUT_STRIP_TRAILING_WHITESPACE
            RESULT_VARIABLE _rigexec_pybind11_probe_result)
        if (_rigexec_pybind11_probe_result EQUAL 0 AND EXISTS "${_rigexec_pybind11_probe}")
            set(_rigexec_python_default ON)
        else()
            set(_rigexec_python_default OFF)
        endif()
    else()
        set(_rigexec_python_default OFF)
    endif()
endif()
option(RIGEXEC_BUILD_PYTHON "Build the RigExec Python bindings (_rigexec + rigexec)"
       ${_rigexec_python_default})

if (RIGEXEC_BUILD_PYTHON)
    add_subdirectory(python)
endif()

# ---------------------------------------------------------------------------
# usdNoodles: the Noodles node-graph editor for usdview, localized from
# OpenUSD PR #4156 -- see plugin/usdNoodles/CMakeLists.txt.
#
# It wraps its C++ with the Boost.Python that ships inside OpenUSD, not with
# pybind11, so what it needs is an OpenUSD built with Python and with imaging
# (garch, glf) -- not RIGEXEC_BUILD_PYTHON. The default follows whether this
# USD install has them.
# ---------------------------------------------------------------------------
if (NOT CMAKE_CROSSCOMPILING AND TARGET python AND TARGET garch AND TARGET glf)
    set(_rigexec_usdnoodles_default ON)
else()
    set(_rigexec_usdnoodles_default OFF)
endif()
option(RIGEXEC_BUILD_USDNOODLES
       "Build the usdNoodles node-graph editor plugin for usdview"
       ${_rigexec_usdnoodles_default})

if (RIGEXEC_BUILD_USDNOODLES)
    # garch and glf name OpenGL::GL in their link interface and pxrConfig
    # leaves finding it to the consumer. It is found HERE, not in the
    # subdirectory: an imported target's link interface resolves in the
    # directory that imported it, and that is this one.
    find_package(OpenGL REQUIRED)
    add_subdirectory(plugin/usdNoodles)
endif()

# End-to-end Python binding test: authors a rig through the package and
# evaluates it in-process. The interpreter was already located by the probe
# above (the python/ subdirectory's find_package result does not propagate to
# this scope).
if (RIGEXEC_BUILD_PYTHON AND RIGEXEC_BUILD_TESTS AND Python3_FOUND)
    add_test(NAME testRigExecPython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_python.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRigExecPythonSchemaAuthoring
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_schema_authoring.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRigExecSplineIkPython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_spline_ik.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRigExecFkControlSpacePython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_fk_control_space.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRigExecFkStartFramePython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_fk_start_frame.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRigExecPoseCyclePython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_pose_cycle.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRigExecBake COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_bake.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRigExecInverse COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_inverse.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    # Holds the interactive-override invalidation contract in both
    # directions: a layout an avar drag cannot reach must survive it and stay
    # bit-identical, and one a layout-naming override CAN reach must still be
    # dropped. Runs against the shipped biped, which is the only example with
    # a skinned mesh large enough for the difference to be observable.
    add_test(NAME testRigExecSkinLayoutOverrides COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_skin_layout_overrides.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    # Cone re-execution on the biped, from Python: parity at zero across a
    # drag, bit-exactness against a rig that never saw one, and -- the half
    # that matters -- that a leaf drag runs FEWER clusters than the root,
    # which genuinely reaches every step through its propagation sets.
    add_test(NAME testRigExecBakedCone COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_baked_cone.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    # The stacked biped -- rig, pose interpolators and sparse blend shapes
    # composed together -- through the program, at parity: the interpolator
    # is a step and a sparse sample's shape is a prologue-resolved layout, and
    # this is the only rig at scale that exercises both.
    add_test(NAME testRigExecBakedPsd COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_baked_psd.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRestLocal COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rest_local.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRestMigration COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rest_migration.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testInterveningXform COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_intervening_xform.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRigExecEpochConstants COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_epoch_constants.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRigExecWireMover COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_wire_mover.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRigExecFloatCurveOp COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_float_curve_op.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRigExecMatrixSpace COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_matrix_space.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRigExecBakedAvarPatch COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_baked_avar_patch.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    # The schema half of rigExec:baked (the registry's own answer for its
    # type, variability and fallback -- which is what a stale
    # generatedSchema.usda would get wrong) and the path usdview takes to it.
    add_test(NAME testRigExecBakedAttributePython
        COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_baked_attribute.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")

    # Headless usdview-plugin tests (Qt-free modules under plugin/).
    # testGizmoMath and testGizmoScreen are written by later tasks; they
    # are registered here so CMakeLists.txt is touched by one task only.
    add_test(NAME testRigExecUndo
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_undo.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testGizmoMath
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_gizmo_math.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testLayerOpinionsModel
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_layer_opinions_model.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testAvarEditorModel
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_avar_editor_model.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testCompositionArcsModel
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_composition_arcs_model.py")
    add_test(NAME testGizmoPreview
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_gizmo_preview.py")
    add_test(NAME testGizmoScreen
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_gizmo_screen.py")
    add_test(NAME testGizmoSettings
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_gizmo_settings.py")
    add_test(NAME testGraphModel
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_graph_model.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testGraphScreen
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_graph_screen.py")
    add_test(NAME testRigExecStageEdits
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_stage_edits.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testGizmoDrag
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_gizmo_drag.py")
    add_test(NAME testGizmoSnap
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_gizmo_snap.py")
    add_test(NAME testViewCubeMath
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_viewcube_math.py")
    add_test(NAME testGizmoMarquee
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_gizmo_marquee.py")

    # set_tests_properties takes exactly ONE value per property, so extra
    # assignments go through APPEND; ctest then splits the resulting list on
    # ';' into individual name=value entries (each entry below is itself
    # semicolon-free). pxr resolution and the native DLL directories are
    # handled by the test itself (_setup_environment); RIGEXEC_USD_INSTALL
    # pins the configured USD install so it does not have to guess.
    foreach(_python_test testRigExecPython testRigExecPythonSchemaAuthoring
                         testRigExecSplineIkPython
                         testRigExecFkControlSpacePython
                         testRigExecBake testRigExecInverse
                         testRestLocal testRestMigration
                         testInterveningXform testRigExecEpochConstants
                         testRigExecBakedAvarPatch testRigExecWireMover
                         testRigExecMatrixSpace
                         testRigExecFloatCurveOp
                         testRigExecUndo testGizmoMath testGizmoScreen
                         testLayerOpinionsModel testCompositionArcsModel
                         testAvarEditorModel
                         testGizmoPreview
                         testGizmoSettings testGizmoDrag testGizmoSnap
                         testViewCubeMath testRigExecStageEdits
                         testGraphModel testGraphScreen)
        set_tests_properties(${_python_test} PROPERTIES ENVIRONMENT
            "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/python")
        set_property(TEST ${_python_test} APPEND PROPERTY ENVIRONMENT
            "PXR_PLUGINPATH_NAME=${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
        set_property(TEST ${_python_test} APPEND PROPERTY ENVIRONMENT
            "RIGEXEC_USD_INSTALL=${USD_INSTALL_DIR}")
    endforeach()
endif()

# The usdNoodles suite: plain unittest modules inside the package, run from
# the staged build copy so they exercise what this build produced.
#
# pxr goes on PYTHONPATH here rather than being resolved inside the test, as
# tests/python/rigexec_test_env.py does for the tests above: these modules are
# upstream code, and three of them start child interpreters that have to
# import pxr and UsdNoodles too -- an environment variable is the one thing
# that reaches those. ENVIRONMENT_MODIFICATION because a Windows PATH-style
# list cannot be written through ENVIRONMENT at all (see below). The site
# layouts are the ones rigexec_test_env.py knows.
if (RIGEXEC_BUILD_USDNOODLES AND RIGEXEC_BUILD_TESTS AND Python3_FOUND)
    add_test(NAME testUsdNoodles
        COMMAND ${Python3_EXECUTABLE} -m unittest discover -s . -p "test*.py"
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/python/UsdNoodles/testenv")
    file(GLOB _rigexec_usd_site_dirs LIST_DIRECTORIES true
        "${USD_INSTALL_DIR}/Lib/site-packages"
        "${USD_INSTALL_DIR}/lib/python*/site-packages"
        "${USD_INSTALL_DIR}/lib/python")
    foreach(_rigexec_usd_site_dir IN LISTS _rigexec_usd_site_dirs)
        if (IS_DIRECTORY "${_rigexec_usd_site_dir}/pxr")
            set_property(TEST testUsdNoodles APPEND PROPERTY
                ENVIRONMENT_MODIFICATION
                "PYTHONPATH=path_list_prepend:${_rigexec_usd_site_dir}")
        endif()
    endforeach()
    set_property(TEST testUsdNoodles APPEND PROPERTY ENVIRONMENT_MODIFICATION
        "PYTHONPATH=path_list_prepend:${CMAKE_CURRENT_BINARY_DIR}/python")
endif()

# ---------------------------------------------------------------------------
# Every test, C++ and Python alike, carries the path its loader needs.
#
# Without this, `ctest --test-dir build` -- which is what the README tells a
# reader to run -- works only from a shell that already has OpenUSD on the
# loader path: on Windows the C++ tests die at startup with 0xc0000135 (DLL
# not found) and on Linux with a missing .so, while the same tests pass
# through bin/build_rigexec.* because those helpers export it first. Tests
# that only pass when launched a particular way are tests nobody else can run.
#
# ENVIRONMENT_MODIFICATION rather than ENVIRONMENT: ctest splits an
# ENVIRONMENT value on ';', which is exactly the separator a Windows PATH
# uses, so a multi-entry PATH cannot be written that way at all.
# path_list_prepend appends with the platform's own separator and leaves the
# inherited value in place.
#
# Read from the directory's own TESTS property so a test added later is
# covered by having been added, not by being remembered here.
get_property(_rigexec_tests DIRECTORY PROPERTY TESTS)
foreach(_rigexec_test IN LISTS _rigexec_tests)
    # The slowest entry here runs in a few seconds, so anything still going
    # after two minutes is wedged, not slow. ctest's default is 1500 s, which
    # is long enough for one hung python test -- a usdview harness waiting on
    # a window that will never appear -- to hold a whole queue for 25 minutes
    # and then report the same failure it would have reported at once.
    set_tests_properties(${_rigexec_test} PROPERTIES TIMEOUT 120)
    if (WIN32)
        set_property(TEST ${_rigexec_test} APPEND PROPERTY
            ENVIRONMENT_MODIFICATION
            "PATH=path_list_prepend:${USD_INSTALL_DIR}/bin"
            "PATH=path_list_prepend:${USD_INSTALL_DIR}/lib"
            "PATH=path_list_prepend:${CMAKE_CURRENT_BINARY_DIR}")
    elseif (APPLE)
        # Mach-O reads DYLD_LIBRARY_PATH and ELF reads LD_LIBRARY_PATH;
        # setting only the other platform's is the same as setting none.
        set_property(TEST ${_rigexec_test} APPEND PROPERTY
            ENVIRONMENT_MODIFICATION
            "DYLD_LIBRARY_PATH=path_list_prepend:${USD_INSTALL_DIR}/lib"
            "DYLD_LIBRARY_PATH=path_list_prepend:${CMAKE_CURRENT_BINARY_DIR}")
    else()
        set_property(TEST ${_rigexec_test} APPEND PROPERTY
            ENVIRONMENT_MODIFICATION
            "LD_LIBRARY_PATH=path_list_prepend:${USD_INSTALL_DIR}/lib"
            "LD_LIBRARY_PATH=path_list_prepend:${CMAKE_CURRENT_BINARY_DIR}")
    endif()
endforeach()

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
include(CMakePackageConfigHelpers)

install(TARGETS rigExecMath rigExec rigExecStandalone rigExecRigging rigExecImaging rigExecBinary rigExecBake
    EXPORT rigExecTargets
    RUNTIME DESTINATION "${RIGEXEC_INSTALL_LIBDIR}"
    LIBRARY DESTINATION "${RIGEXEC_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION "${RIGEXEC_INSTALL_LIBDIR}")

# FILES_MATCHING still walks -- and recreates -- every subdirectory it passes
# through, so dot-directories and local tooling state are excluded explicitly.
foreach(_lib rigExecMath rigExec rigExecStandalone rigExecRigging rigExecImaging rigExecBinary rigExecBake)
    install(DIRECTORY "libs/${_lib}/"
        DESTINATION "${RIGEXEC_INSTALL_INCLUDEDIR}/${_lib}"
        FILES_MATCHING
            PATTERN ".*" EXCLUDE
            PATTERN "__pycache__" EXCLUDE
            PATTERN "*.h")
endforeach()

# The codeless schema domain is no longer location-independent: it declares
# implementsComputeExtent, so Plug has to be able to LOAD it on demand to get
# the registering code (UsdGeomBoundable compute-extent) into the process.
# That needs a LibraryPath naming the imaging library -- and a library-type
# plugin, because Plug treats a "resource" plugin as already loaded and never
# dlopens it (plug/plugin.cpp: _isLoaded(type == ResourceType)).
#
# So the schema plugInfo is generated rather than installed verbatim, exactly
# like rigExecImaging's and for the same reason: the filename is platform
# dependent, and only $<TARGET_FILE_NAME:> knows it. Generating it under
# build/usd/rigExecSchema/resources/ is what lets ONE file serve both trees --
# from there "../../../<library>" lands on build/, and from
# lib/usd/rigExecSchema/resources/ the identical hop lands on lib/.
#
# The schema DECLARATIONS stay data-only: generatedSchema.usda is copied
# beside it untouched, and no schema C++ class exists or is generated.
set(_rigexec_schema_resources
    "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
set(_rigexec_schema_pluginfo "${_rigexec_schema_resources}/plugInfo.json")
# The generated copy is produced at CONFIGURE time by reading the checked-in
# one, so editing the checked-in one has to re-run configure. Without this a
# schema edit (a new SdfMetadata field, say) builds clean and then fails at
# stage open, because the plugInfo the runtime loads is the stale generated
# copy and nothing rebuilt it.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/plugin/rigExecSchema/resources/plugInfo.json"
    "${CMAKE_CURRENT_SOURCE_DIR}/plugin/rigExecSchema/resources/generatedSchema.usda")
file(READ
    "${CMAKE_CURRENT_SOURCE_DIR}/plugin/rigExecSchema/resources/plugInfo.json"
    _rigexec_schema_pluginfo_content)
string(REPLACE "\"Type\": \"resource\"" "\"Type\": \"library\""
    _rigexec_schema_pluginfo_content "${_rigexec_schema_pluginfo_content}")

# implementsComputeExtent is added HERE, not left in the checked-in copy.
#
# The flag is a promise that Plug can load code which registers a
# compute-extent function. Only this generated copy can keep that promise:
# it is the one with a LibraryPath. Advertising it from the source-tree
# resources -- which are a data-only "resource" plugin Plug never dlopens --
# promises an extent that never arrives, and a host registering those
# resources gets Boundable prims whose bounds are silently empty. Better to
# be honestly non-Boundable-with-extents there than dishonestly Boundable.
foreach(_rigexec_boundable
        RigExecXformable RigExecControl RigExecJoint RigExecFkChain
        RigExecTwoBoneIk RigExecBlendPointFrames RigExecTwistDistribution
        RigExecRibbon RigExecSplineIk RigExecVolumeWeight RigExecSphereWeight
        RigExecPlaneWeight RigExecCurveWeight)
    string(REPLACE
        "\"schemaIdentifier\": \"${_rigexec_boundable}\""
        "\"implementsComputeExtent\": true, \"schemaIdentifier\": \"${_rigexec_boundable}\""
        _rigexec_schema_pluginfo_content "${_rigexec_schema_pluginfo_content}")
endforeach()
string(REPLACE "\"Name\": \"rigExecSchema\""
    "\"LibraryPath\": \"../../../$<TARGET_FILE_NAME:rigExecImaging>\", \"Name\": \"rigExecSchema\""
    _rigexec_schema_pluginfo_content "${_rigexec_schema_pluginfo_content}")
file(GENERATE OUTPUT "${_rigexec_schema_pluginfo}"
    CONTENT "${_rigexec_schema_pluginfo_content}")

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/plugin/rigExecSchema/resources/generatedSchema.usda"
    "${_rigexec_schema_resources}/generatedSchema.usda" COPYONLY)

install(FILES
        "${_rigexec_schema_pluginfo}"
        "${_rigexec_schema_resources}/generatedSchema.usda"
    DESTINATION "${RIGEXEC_INSTALL_PLUGINDIR}/rigExecSchema/resources")

# The usdview plugins ship as DIRECTORIES, not as lists of files.
#
# Each is a plugInfo.json naming the container plus a flat set of modules that
# import each other as siblings -- rigExecUsdview.py puts its own directory on
# sys.path and the panels take it from there -- so what has to arrive is the
# directory's contents, whatever they currently are.
#
# It was a list, and the list went stale. The gizmo toolbar and its maths, the
# graph editor, the view cube, the undo stack, and the layer-opinion and
# composition-arc panels were all added after that list was written and none of
# them were ever added to it, so an installed plugin could open two of its
# panels and warned at runtime about the rest. Nothing failed at build time,
# because a list of filenames that must agree with a directory is a second copy
# of that directory, and a copy drifts in silence.
#
# The artwork travels the same way, for the same reason: gizmoIcons tints it at
# draw time and falls back to drawing each glyph when a file is missing, so an
# install that dropped it would look almost right.
#
# FILES_MATCHING still walks -- and recreates -- every subdirectory it passes
# through, so dot-directories and bytecode caches are excluded explicitly: a
# stale .pyc for a module that has since changed is worse than no .pyc.
install(DIRECTORY plugin/rigExecUsdview/
    DESTINATION "${RIGEXEC_INSTALL_PYTHONDIR}/rigExecUsdview"
    FILES_MATCHING
        PATTERN ".*" EXCLUDE
        PATTERN "__pycache__" EXCLUDE
        PATTERN "*.py"
        PATTERN "*.json"
        PATTERN "*.png")

# museAgent.py carries the agent core (message shaping, tool loop, image
# metadata) and museAssistant imports it, so shipping one without the other
# installs a plugin that fails at import. The directory install is what keeps
# that true of whatever the plugin grows next.
install(DIRECTORY plugin/museAssistant/
    DESTINATION "${RIGEXEC_INSTALL_PYTHONDIR}/museAssistant"
    FILES_MATCHING
        PATTERN ".*" EXCLUDE
        PATTERN "__pycache__" EXCLUDE
        PATTERN "*.py"
        PATTERN "*.json")

# rigExecImaging is the one plugin whose plugInfo.json names a library, so it
# is generated from a template rather than checked in: the filename is
# .dll/.so/.dylib depending on the platform, and $<TARGET_FILE_NAME:> is the
# only thing that actually knows it.
#
# One generated file serves both trees. The build copy sits under
# build/usd/rigExecImaging/resources/ precisely so that its "../../<library>"
# hop lands on the build directory the same way the installed copy's hop lands
# on lib/ -- identical content, identical relative shape, nothing to drift.
#
# Note this assumes the library name does not vary by configuration; it does
# not here, since no debug postfix is set. If one is ever added, file(GENERATE)
# fails loudly at generate time rather than emitting a wrong file, and the
# output path then needs a $<CONFIG> component.
# Editing the template has to re-run configure, for the same reason the
# schema plugInfo above says: without this the runtime loads the stale
# generated copy and nothing rebuilds it.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/plugin/rigExecImaging/resources/plugInfo.json.in")
set(_rigexec_pluginfo
    "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecImaging/resources/plugInfo.json")
file(READ
    "${CMAKE_CURRENT_SOURCE_DIR}/plugin/rigExecImaging/resources/plugInfo.json.in"
    _rigexec_pluginfo_content)
string(REPLACE "@RIGEXEC_IMAGING_LIBRARY_FILENAME@"
    "$<TARGET_FILE_NAME:rigExecImaging>"
    _rigexec_pluginfo_content "${_rigexec_pluginfo_content}")
file(GENERATE OUTPUT "${_rigexec_pluginfo}"
    CONTENT "${_rigexec_pluginfo_content}")

install(FILES "${_rigexec_pluginfo}"
    DESTINATION "${RIGEXEC_INSTALL_PLUGINDIR}/rigExecImaging/resources")

# CMake package, so consumers can find_package(rigExec) and link
# rigExec::rigExec rather than reproducing the include and link lines.
install(EXPORT rigExecTargets
    FILE rigExecTargets.cmake
    NAMESPACE rigExec::
    DESTINATION "${RIGEXEC_INSTALL_CMAKEDIR}")

configure_package_config_file(
    cmake/rigExecConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/rigExecConfig.cmake"
    INSTALL_DESTINATION "${RIGEXEC_INSTALL_CMAKEDIR}"
    PATH_VARS RIGEXEC_INSTALL_PLUGINDIR RIGEXEC_INSTALL_PYTHONDIR
              RIGEXEC_INSTALL_LIBDIR)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/rigExecConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMinorVersion)

install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/rigExecConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/rigExecConfigVersion.cmake"
    DESTINATION "${RIGEXEC_INSTALL_CMAKEDIR}")
