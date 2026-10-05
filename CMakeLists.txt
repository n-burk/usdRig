cmake_minimum_required(VERSION 3.26)
project(rigExec VERSION 0.1.0 LANGUAGES CXX)

# RigExec builds against an unchanged installed OpenUSD v26.08
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

# Bitwise parity with Gf and the frozen oracles needs OpenUSD's own
# floating-point contraction: USD builds with -ffp-contract=off on Apple
# (cmake/defaults/clangdefaults.cmake) so a*b+c is never fused into FMA.
# Clang on ARM64 contracts by default, which drifts every memcmp/bitwise
# test (RuntimeMath, RuntimePose, RuntimeGeometry, ImagingPlayback,
# verify_binary, the Rbf oracle) by an ulp. -fno-fast-math does not cover
# this; only -ffp-contract=off does. MSVC needs nothing: /fp:precise is
# its default and it never auto-contracts.
if (NOT MSVC)
    add_compile_options(-ffp-contract=off)
endif()

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

if (APPLE)
    set(CMAKE_INSTALL_NAME_DIR "@rpath")
    set(CMAKE_INSTALL_RPATH "@loader_path")
elseif (UNIX)
    set(CMAKE_INSTALL_RPATH "$ORIGIN")
endif()
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)

# rigExecMath: pure point-frame and solver kernels (spec §5).
add_library(rigExecMath STATIC
    libs/rigExecMath/pointFrame.cpp
    libs/rigExecMath/solvers.cpp
    libs/rigExecMath/singleChainIk.cpp
    libs/rigExecMath/propertyMath.cpp
    libs/rigExecMath/geometryKernels.cpp
    libs/rigExecMath/weightFields.cpp
    libs/rigExecMath/simdKernels.cpp
    libs/rigExecMath/dualQuat.cpp
    libs/rigExecMath/splineIk.cpp
    libs/rigExecMath/rbf.cpp
)
target_include_directories(rigExecMath PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
target_link_libraries(rigExecMath PUBLIC arch tf gf vt)
set_target_properties(rigExecMath PROPERTIES POSITION_INDEPENDENT_CODE ON)

# rigExec: exec type registrations, schema computations, tap set, and the
add_library(rigExec SHARED
    libs/rigExec/types.cpp
    libs/rigExec/parallel.cpp
    libs/rigExec/computations.cpp
    libs/rigExec/moverKernels.cpp
    libs/rigExec/solverKernels.cpp
    libs/rigExec/weightPackets.cpp
    libs/rigExec/movers/moverRegistry.cpp
    libs/rigExec/movers/matrixMover.cpp
    libs/rigExec/movers/skinMover.cpp
    libs/rigExec/movers/blendShapeMover.cpp
    libs/rigExec/movers/smoothMover.cpp
    libs/rigExec/movers/deltaMushMover.cpp
    libs/rigExec/movers/wrinkleMover.cpp
    libs/rigExec/movers/volumeCorrectMover.cpp
    libs/rigExec/movers/latticeMover.cpp
    libs/rigExec/movers/surfaceMover.cpp
    libs/rigExec/movers/curveMover.cpp
    libs/rigExec/movers/mathMovers.cpp
    libs/rigExec/moverGraph.cpp
    libs/rigExec/tapSet.cpp
    libs/rigExec/rigEvaluator.cpp
    libs/rigExec/rigEvaluatorChainPlan.cpp
    libs/rigExec/rigEvaluatorCompile.cpp
    libs/rigExec/rigEvaluatorDependencies.cpp
    libs/rigExec/rigEvaluatorDigest.cpp
    libs/rigExec/rigEvaluatorDynamic.cpp
    libs/rigExec/rigEvaluatorGeometry.cpp
    libs/rigExec/rigEvaluatorGeometryEvaluation.cpp
    libs/rigExec/rigEvaluatorInputs.cpp
    libs/rigExec/rigEvaluatorLiveGraph.cpp
    libs/rigExec/rigEvaluatorNotices.cpp
    libs/rigExec/rigEvaluatorPose.cpp
    libs/rigExec/rigEvaluatorProperties.cpp
    libs/rigExec/rigEvaluatorValidation.cpp
    libs/rigExec/bakedProgram.cpp
    libs/rigExec/bakedPose.cpp
    libs/rigExec/bakedSchedule.cpp
    libs/rigExec/bakedTrace.cpp
    libs/rigExec/bakedGeometry.cpp
    libs/rigExec/bakedWeights.cpp
    libs/rigExec/bakedVerify.cpp
    libs/rigExec/frameCache.cpp
    libs/rigExec/frozenContext.cpp
    libs/rigExec/frozenDigest.cpp
    libs/rigExec/frozenGeometry.cpp
    libs/rigExec/frozenPropertySampling.cpp
    libs/rigExec/frozenSampling.cpp
    libs/rigExec/frozenSnapshot.cpp
    libs/rigExec/frozenWorker.cpp
    libs/rigExec/backgroundScheduler.cpp
    libs/rigExec/outputAffectedIndex.cpp
    libs/rigExec/taskListCache.cpp
    libs/rigExec/frameCacheSparsity.cpp
)
target_include_directories(rigExec PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
target_link_libraries(rigExec PUBLIC
    rigExecMath
    # usdSkel is here for UsdSkelBlendShape only: a sparse blend sample names
    plug sdf usd usdGeom usdSkel work
    exec execUsd ef vdf
)

add_library(rigExec::rigExec ALIAS rigExec)
include(cmake/rigExecMoverPlugin.cmake)

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

# rigExecImaging: Hydra 2.0 scene-index publication (spec §10).
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
    # The 1.4 warm-frame index: shared-owned per-frame completions.
    libs/rigExecImaging/warmIndex.cpp
    libs/rigExecImaging/profilerApi.cpp
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
    hio
    sdr
    work
)
target_link_libraries(rigExecImaging PRIVATE rigExecRuntime rigExecSampler)
target_compile_definitions(rigExecImaging PRIVATE RIGEXEC_IMAGING_EXPORTS)

# The vendored FlatBuffers C++ runtime headers (Apache-2.0, unmodified; see
# thirdparty/flatbuffers/README.md). The generated headers under
# libs/rigExecBinary/generated static_assert this exact version.
add_library(rigExecFlatBuffers INTERFACE)
target_include_directories(rigExecFlatBuffers INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/flatbuffers/include>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
add_library(rigExec::rigExecFlatBuffers ALIAS rigExecFlatBuffers)

add_library(rigExecBinary STATIC
    libs/rigExecBinary/container.cpp
    libs/rigExecBinary/program.cpp
    libs/rigExecBinary/pose.cpp
    libs/rigExecBinary/geometry.cpp
    libs/rigExecBinary/inputTable.cpp
    libs/rigExecBinary/propertyChains.cpp
    libs/rigExecBinary/external.cpp
    libs/rigExecBinary/computed.cpp
    libs/rigExecBinary/format.cpp)
target_include_directories(rigExecBinary PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
target_link_libraries(rigExecBinary PUBLIC rigExecFlatBuffers)
set_target_properties(rigExecBinary PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(rigExecRuntime STATIC
    libs/rigExecRuntime/open.cpp
    libs/rigExecRuntime/exec.cpp
    libs/rigExecRuntime/closure.cpp
    libs/rigExecRuntime/publish.cpp
    libs/rigExecRuntime/kernels.cpp
    libs/rigExecRuntime/pose.cpp
    libs/rigExecRuntime/poseConstraints.cpp
    libs/rigExecRuntime/poseInterpolation.cpp
    libs/rigExecRuntime/poseMath.cpp
    libs/rigExecRuntime/poseSolvers.cpp
    libs/rigExecRuntime/poseSteps.cpp
    libs/rigExecRuntime/geometry.cpp
    libs/rigExecRuntime/weights.cpp
    libs/rigExecRuntime/inputs.cpp
    libs/rigExecRuntime/properties.cpp)
target_include_directories(rigExecRuntime PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
target_link_libraries(rigExecRuntime PUBLIC rigExecBinary)
set_target_properties(rigExecRuntime PROPERTIES POSITION_INDEPENDENT_CODE ON)
# Installed names, so a mover plugin's tests can export and replay its rigs
# the same way inside this build and against an installed SDK.
add_library(rigExec::rigExecRuntime ALIAS rigExecRuntime)
# -ffp-contract=off repeats the global option so the runtime's own flags
# name everything its bitwise operation order needs; other builds of these
# sources (the Godot extension) copy this list.
if (MSVC)
    target_compile_options(rigExecRuntime PRIVATE /fp:precise)
else ()
    target_compile_options(rigExecRuntime PRIVATE -fno-fast-math
        -ffp-contract=off)
endif ()

# rigExecSampler: hands a .rigexec's Animated inputs the values a stage holds
# at each new time (RigExecInputSampler), for every consumer that plays the
# runtime over a stage. The runtime itself stays USD-free.
add_library(rigExecSampler STATIC libs/rigExecSampler/inputSampler.cpp)
target_include_directories(rigExecSampler PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
target_link_libraries(rigExecSampler PUBLIC rigExecRuntime usd sdf tf gf)
set_target_properties(rigExecSampler PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(rigExec::rigExecSampler ALIAS rigExecSampler)

# rigExecBake: .rigexec baking from a compiled evaluator (M1).
add_library(rigExecBake SHARED
    libs/rigExecBake/bake.cpp
    libs/rigExecBake/serialize.cpp
    libs/rigExecBake/capture.cpp
    libs/rigExecBake/computedCapture.cpp
    libs/rigExecBake/revisionReads.cpp
    libs/rigExecBake/staticReport.cpp)
target_include_directories(rigExecBake PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
target_link_libraries(rigExecBake PUBLIC
    rigExec
    rigExecBinary)
add_library(rigExec::rigExecBake ALIAS rigExecBake)

add_executable(rigExecBakeTool tools/rigExecBake.cpp)
set_target_properties(rigExecBakeTool PROPERTIES OUTPUT_NAME rigExecBake)
target_link_libraries(rigExecBakeTool PRIVATE rigExecBake)
target_compile_definitions(rigExecBakeTool PRIVATE
    RIGEXEC_SCHEMA_RESOURCE_DIR="${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")

add_executable(rigExecPose tools/rigExecPose.cpp)
target_link_libraries(rigExecPose PRIVATE rigExec rigExecRuntime rigExecSampler)
target_compile_definitions(rigExecPose PRIVATE
    RIGEXEC_SCHEMA_RESOURCE_DIR="${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")

if (CMAKE_CROSSCOMPILING)
    set(_rigexec_tests_default OFF)
else()
    set(_rigexec_tests_default ON)
endif()
option(RIGEXEC_BUILD_TESTS "Build the RigExec test and probe executables"
       ${_rigexec_tests_default})

# The USD install the tests RUN against: its bin/lib go on the test
# PATH, and RIGEXEC_USD_INSTALL points the Python helpers at its
# site-packages. This is USD_INSTALL_DIR unless that is a CMake shim
# rather than a real install (as with the Houdini build, where the
# shim lives in cmake/houdini but the binaries come from HOUDINI_ROOT).
set(RIGEXEC_TEST_USD_DIR "${USD_INSTALL_DIR}"
    CACHE PATH "USD install whose binaries the tests run against")


if (RIGEXEC_BUILD_TESTS)
enable_testing()

set(_rigexec_schema_resource_define
    RIGEXEC_SCHEMA_RESOURCE_DIR="${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
# The generated imaging plugInfo, for the test that drives the real
set(_rigexec_imaging_resource_define
    RIGEXEC_IMAGING_RESOURCE_DIR="${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecImaging/resources")

add_subdirectory(tests/externalMovers)
add_executable(testRigExecExternalMovers tests/testRigExecExternalMovers.cpp)
target_link_libraries(testRigExecExternalMovers PRIVATE
    rigExec rigExecRigging rigExecBake rigExecRuntime usd usdGeom plug)
add_dependencies(testRigExecExternalMovers testExternalMover)
add_test(NAME testRigExecExternalMovers COMMAND testRigExecExternalMovers
    "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources"
    "$<TARGET_GENEX_EVAL:testExternalMover,$<TARGET_PROPERTY:testExternalMover,RIGEXEC_PLUGIN_RESOURCE_DIR>>")

add_executable(testRigExecMath tests/testRigExecMath.cpp)
add_executable(testRigExecWrinkleMath tests/testRigExecWrinkleMath.cpp)
target_link_libraries(testRigExecWrinkleMath PRIVATE rigExecMath)
add_test(NAME testRigExecWrinkleMath COMMAND testRigExecWrinkleMath)
add_executable(testRigExecWrinkle tests/testRigExecWrinkle.cpp)
target_link_libraries(testRigExecWrinkle PRIVATE rigExec rigExecRigging rigExecBake rigExecRuntime rigExecSampler)
add_test(NAME testRigExecWrinkle COMMAND testRigExecWrinkle
    "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources"
    "${CMAKE_CURRENT_BINARY_DIR}/wrinkle_test.rigexec")
set_tests_properties(testRigExecWrinkle PROPERTIES FIXTURES_SETUP WrinkleBinary)
add_executable(testRigExecWrinkleRuntime tests/testRigExecWrinkleRuntime.cpp)
target_link_libraries(testRigExecWrinkleRuntime PRIVATE rigExecRuntime)
add_test(NAME testRigExecWrinkleRuntime COMMAND testRigExecWrinkleRuntime
    "${CMAKE_CURRENT_BINARY_DIR}/wrinkle_test.rigexec")
set_tests_properties(testRigExecWrinkleRuntime PROPERTIES FIXTURES_REQUIRED WrinkleBinary)
add_executable(testRigExecDeltaMush tests/testRigExecDeltaMush.cpp)
target_link_libraries(testRigExecDeltaMush PRIVATE rigExec rigExecRigging rigExecBake rigExecRuntime rigExecSampler)
add_test(NAME testRigExecDeltaMush COMMAND testRigExecDeltaMush "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
add_executable(testRigExecRuntimeLiveFace tests/testRigExecRuntimeLiveFace.cpp)
target_link_libraries(testRigExecRuntimeLiveFace PRIVATE rigExec rigExecBake rigExecRuntime)
add_test(NAME testRigExecRuntimeLiveFace COMMAND testRigExecRuntimeLiveFace
    "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources"
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/biped/Biped_stack.usda")
add_executable(testRigExecDeltaMushRuntime tests/testRigExecDeltaMushRuntime.cpp)
target_link_libraries(testRigExecDeltaMushRuntime PRIVATE rigExecRuntime)
add_test(NAME testRigExecDeltaMushBakeFile COMMAND rigExecBakeTool
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/delta_mush_mover.usda"
    --rig /Rig --time 1
    -o "${CMAKE_CURRENT_BINARY_DIR}/delta_mush_test.rigexec")
set_tests_properties(testRigExecDeltaMushBakeFile PROPERTIES FIXTURES_SETUP DeltaMushBinary)
add_test(NAME testRigExecDeltaMushRuntime COMMAND testRigExecDeltaMushRuntime
    "${CMAKE_CURRENT_BINARY_DIR}/delta_mush_test.rigexec")
set_tests_properties(testRigExecDeltaMushRuntime PROPERTIES FIXTURES_REQUIRED DeltaMushBinary)
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
            add_test(NAME ${name}Cones_grain0_${_schedule}
                COMMAND ${name} ${_args})
            set_tests_properties(${name}Cones_grain0_${_schedule} PROPERTIES
                ENVIRONMENT "RIGEXEC_EVALUATION_MODE=parity"
                FAIL_REGULAR_EXPRESSION "baked parity mismatch")
            set_property(TEST ${name}Cones_grain0_${_schedule} APPEND
                PROPERTY ENVIRONMENT
                "RIGEXEC_BAKED_VERIFY_CONES=1"
                "RIGEXEC_BAKED_SCHEDULE=${_schedule}"
                "RIGEXEC_BAKED_GRAIN_US=0")
            if (_require_bake)
                set_property(TEST ${name}Cones_${_schedule}
                    ${name}Cones_grain0_${_schedule} APPEND
                    PROPERTY ENVIRONMENT "RIGEXEC_BAKE_REQUIRED=1")
            endif()
        endforeach()
    endif()
endfunction()
rigexec_add_baked_parity_test(testRigExecConstraints CONE_VERIFY)

function(rigexec_add_example_parity_test)
    cmake_parse_arguments(A "GUIDES;DISABLED" "NAME;RIG;STAGE;FRAMES" "" ${ARGN})
    # DEFINED, not truthiness: a frame list that is the single frame 0 is
    # false to if().
    if (NOT DEFINED A_NAME OR NOT DEFINED A_FRAMES OR
            (NOT DEFINED A_RIG AND NOT DEFINED A_STAGE) OR
            (DEFINED A_RIG AND DEFINED A_STAGE))
        message(FATAL_ERROR "rigexec_add_example_parity_test: "
                            "NAME, FRAMES and one of RIG or STAGE are required")
    endif()
    if (A_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "rigexec_add_example_parity_test: unknown "
                            "argument(s): ${A_UNPARSED_ARGUMENTS}")
    endif()
    set(_guides "")
    if (A_GUIDES)
        set(_guides "--guides")
    endif()
    if (DEFINED A_RIG)
        set(A_STAGE "${CMAKE_CURRENT_SOURCE_DIR}/examples/${A_RIG}")
    endif()
    add_test(NAME example_parity_${A_NAME} COMMAND rigExecPose
        "${A_STAGE}"
        --frames "${A_FRAMES}" --mode parity --require-baked ${_guides})
    set_tests_properties(example_parity_${A_NAME} PROPERTIES
        ENVIRONMENT "RIGEXEC_EVALUATION_MODE=parity"
        FAIL_REGULAR_EXPRESSION "baked parity mismatch;FAIL:")
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
    string(REGEX REPLACE "\\.(usda|usd)$" "" _fixture_name "${_fixture_stage}")
    string(REPLACE "/" "_" _fixture_name "${_fixture_name}")
    if (_fixture_bakes STREQUAL "YES")
        rigexec_add_example_parity_test(NAME ${_fixture_name}
            RIG ${_fixture_stage} FRAMES ${_fixture_frames} GUIDES)
    else()
        rigexec_add_example_parity_test(NAME ${_fixture_name}
            RIG ${_fixture_stage} FRAMES ${_fixture_frames} GUIDES DISABLED)
    endif()
endforeach()

rigexec_add_example_parity_test(NAME biped_Biped_stack_anim
    RIG biped/Biped_stack_anim.usda FRAMES 1,50,100,150,200 GUIDES)

# Surface projectors whose rigExec:space, rigExec:sources and
# rigExec:sourceSpace are constrained controls that nothing else reads: the
# projector frames are built from base and final of all three.
rigexec_add_example_parity_test(NAME projector_spaces
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/projector_spaces.usda"
    FRAMES 1,4,7,10 GUIDES)

# Two volumes with one placement step each, one of them placed differently
# at base (a mover's packet) and at final (pose.weightFrames, a
# current-phase field) because a constraint revises its joint after the
# solver.
rigexec_add_example_parity_test(NAME volume_placements
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/volume_placements.usda"
    FRAMES 1,5,10 GUIDES)

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
set_tests_properties(testRigExecExampleParity PROPERTIES
    ENVIRONMENT "RIGEXEC_BAKE_REQUIRED=1"
    FAIL_REGULAR_EXPRESSION "baked parity mismatch")
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
    # And at grain zero, one step per cluster (see CONE_VERIFY above).
    add_test(NAME testRigExecExampleParityCones_grain0_${_schedule}
        COMMAND testRigExecExampleParity
        "${CMAKE_CURRENT_SOURCE_DIR}/examples")
    set_tests_properties(testRigExecExampleParityCones_grain0_${_schedule}
        PROPERTIES ENVIRONMENT "RIGEXEC_BAKE_REQUIRED=1"
        FAIL_REGULAR_EXPRESSION "baked parity mismatch")
    set_property(TEST testRigExecExampleParityCones_grain0_${_schedule} APPEND
        PROPERTY ENVIRONMENT
        "RIGEXEC_BAKED_VERIFY_CONES=1"
        "RIGEXEC_BAKED_SCHEDULE=${_schedule}"
        "RIGEXEC_BAKED_GRAIN_US=0")
endforeach()

add_executable(testRigExecSurfaceProjector
    tests/testRigExecSurfaceProjector.cpp)
target_link_libraries(testRigExecSurfaceProjector PRIVATE rigExec)
target_compile_definitions(testRigExecSurfaceProjector PRIVATE
    ${_rigexec_schema_resource_define})
add_test(NAME testRigExecSurfaceProjector
    COMMAND testRigExecSurfaceProjector "${CMAKE_CURRENT_SOURCE_DIR}/examples")
set_tests_properties(testRigExecSurfaceProjector PROPERTIES
    ENVIRONMENT "RIGEXEC_BAKE_REQUIRED=1;RIGEXEC_DYNAMIC_RUNS_PROGRAM=0"
    FAIL_REGULAR_EXPRESSION "baked parity mismatch")

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
    # And at grain zero, one step per cluster (see CONE_VERIFY above).
    add_test(NAME testRigExecGeometryOpsBakedParityCones_grain0_${_schedule}
        COMMAND testRigExecGeometryOpsBakedParity
        "${CMAKE_CURRENT_SOURCE_DIR}/examples"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/rigExecRibbonProbe.usda")
    set_tests_properties(
        testRigExecGeometryOpsBakedParityCones_grain0_${_schedule}
        PROPERTIES ENVIRONMENT "RIGEXEC_BAKE_REQUIRED=1"
        FAIL_REGULAR_EXPRESSION "baked parity mismatch")
    set_property(
        TEST testRigExecGeometryOpsBakedParityCones_grain0_${_schedule}
        APPEND PROPERTY ENVIRONMENT
        "RIGEXEC_BAKED_VERIFY_CONES=1"
        "RIGEXEC_BAKED_SCHEDULE=${_schedule}"
        "RIGEXEC_BAKED_GRAIN_US=0")
endforeach()

add_executable(testRigExecStandalone tests/testRigExecStandalone.cpp)
target_link_libraries(testRigExecStandalone PRIVATE rigExecStandalone)
target_compile_definitions(testRigExecStandalone PRIVATE ${_rigexec_schema_resource_define})
add_test(NAME testRigExecStandalone COMMAND testRigExecStandalone)

add_executable(testRigExecRuntimeMath tests/testRigExecRuntimeMath.cpp)
target_link_libraries(testRigExecRuntimeMath PRIVATE rigExecRuntime gf)
add_test(NAME testRigExecRuntimeMath COMMAND testRigExecRuntimeMath)

add_executable(testRigExecRuntimeLoader tests/testRigExecRuntimeLoader.cpp)
target_link_libraries(testRigExecRuntimeLoader PRIVATE rigExecRuntime rigExecBake rigExecSampler)
target_include_directories(testRigExecRuntimeLoader PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/tests")
target_compile_definitions(testRigExecRuntimeLoader PRIVATE
    ${_rigexec_schema_resource_define}
    RIGEXEC_EXAMPLES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/examples")
add_test(NAME testRigExecRuntimeLoader COMMAND testRigExecRuntimeLoader
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

# Family parity tests: baked program vs runtime, per family. Each is
add_executable(testRigExecRuntimePose tests/testRigExecRuntimePose.cpp)
target_link_libraries(testRigExecRuntimePose PRIVATE rigExecRuntime rigExecBake rigExecSampler)
target_include_directories(testRigExecRuntimePose PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/tests")
target_compile_definitions(testRigExecRuntimePose PRIVATE
    ${_rigexec_schema_resource_define}
    RIGEXEC_EXAMPLES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/examples")
add_test(NAME testRigExecRuntimePose COMMAND testRigExecRuntimePose
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
# The same suite baked with every step in its own cluster, so a drag at a
# fixed frame reaches each step only through the dirtying owed to it (a
# moved avar, an override flag, a moved chain result) and never through a
# cluster it shares with another step.
add_test(NAME testRigExecRuntimePoseFineClusters COMMAND testRigExecRuntimePose
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
set_tests_properties(testRigExecRuntimePoseFineClusters PROPERTIES
    ENVIRONMENT "RIGEXEC_BAKED_GRAIN_US=0")

add_executable(testRigExecRuntimeWeights tests/testRigExecRuntimeWeights.cpp)
target_link_libraries(testRigExecRuntimeWeights PRIVATE rigExecRuntime rigExecBake rigExecSampler)
target_include_directories(testRigExecRuntimeWeights PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/tests")
target_compile_definitions(testRigExecRuntimeWeights PRIVATE
    ${_rigexec_schema_resource_define}
    RIGEXEC_EXAMPLES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/examples")
add_test(NAME testRigExecRuntimeWeights COMMAND testRigExecRuntimeWeights
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecRuntimeGeometry tests/testRigExecRuntimeGeometry.cpp)
target_link_libraries(testRigExecRuntimeGeometry PRIVATE rigExecRuntime rigExecBake rigExecSampler)
target_include_directories(testRigExecRuntimeGeometry PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/tests")
target_compile_definitions(testRigExecRuntimeGeometry PRIVATE
    ${_rigexec_schema_resource_define}
    RIGEXEC_EXAMPLES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/examples")
add_test(NAME testRigExecRuntimeGeometry COMMAND testRigExecRuntimeGeometry
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

# Inputs set on a binary against the same values authored in the session
# layer, in the dynamic and baked evaluators: example inputs, held
# space-switch indices and a ladder input a switch recomposes from.
add_executable(testRigExecRuntimeInputs tests/testRigExecRuntimeInputs.cpp)
target_link_libraries(testRigExecRuntimeInputs PRIVATE
    rigExec rigExecBake rigExecRuntime rigExecSampler)
target_compile_definitions(testRigExecRuntimeInputs PRIVATE
    ${_rigexec_schema_resource_define})
add_test(NAME testRigExecRuntimeInputs COMMAND testRigExecRuntimeInputs
    "${CMAKE_CURRENT_SOURCE_DIR}/examples"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures")

# The single-FlatBuffer format on hand-built files: bit exactness, the
# refusals before and in verification, determinism, validation rules.
add_executable(testRigExecFormat tests/testRigExecFormat.cpp)
target_link_libraries(testRigExecFormat PRIVATE rigExecBinary)
add_test(NAME testRigExecFormat COMMAND testRigExecFormat)

add_executable(testRigExecBinary tests/testRigExecBinary.cpp)
target_link_libraries(testRigExecBinary PRIVATE rigExecBake rigExecRigging)
target_include_directories(testRigExecBinary PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/tests")
target_compile_definitions(testRigExecBinary PRIVATE
    ${_rigexec_schema_resource_define}
    RIGEXEC_EXAMPLES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/examples")
add_test(NAME testRigExecBinary COMMAND testRigExecBinary
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

# RIG names a stage under examples/, STAGE any other stage by full path.
# The test bakes the stage once, at TIME (default: the first of FRAMES), and
# replays the binary against the parity evaluator: its defaults at TIME, then
# every frame of FRAMES with the binary's Animated inputs sampled from the
# stage, then two drags of each input DRAGS names (a comma-separated list of
# <prim>.<attr>) against the same value authored in the session layer.
# CLASS is `inputs` (every animated source is an input, so the binary
# follows the stage at every frame) or `static` (an animated source is
# static data the binary holds at TIME, so it verifies at TIME alone, plus
# the drags); see tests/exampleFixtures.cmake.
function(rigexec_add_verify_binary_test)
    cmake_parse_arguments(A ""
        "NAME;RIG;STAGE;FRAMES;TIME;CLASS;DRAGS"
        "" ${ARGN})
    if (NOT DEFINED A_NAME OR NOT DEFINED A_FRAMES OR
            (NOT DEFINED A_RIG AND NOT DEFINED A_STAGE) OR
            (DEFINED A_RIG AND DEFINED A_STAGE))
        message(FATAL_ERROR "rigexec_add_verify_binary_test: NAME, FRAMES "
                            "and one of RIG or STAGE are required")
    endif()
    if (A_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "rigexec_add_verify_binary_test: unknown "
                            "argument(s): ${A_UNPARSED_ARGUMENTS}")
    endif()
    if (DEFINED A_CLASS AND NOT A_CLASS STREQUAL "inputs" AND
            NOT A_CLASS STREQUAL "static")
        message(FATAL_ERROR "rigexec_add_verify_binary_test: CLASS is "
                            "'${A_CLASS}', expected inputs or static")
    endif()
    if (DEFINED A_RIG)
        set(A_STAGE "${CMAKE_CURRENT_SOURCE_DIR}/examples/${A_RIG}")
    endif()
    set(_options)
    foreach(_option IN ITEMS TIME CLASS DRAGS)
        if (DEFINED A_${_option})
            list(APPEND _options "-D${_option}=${A_${_option}}")
        endif()
    endforeach()
    set(_out "${CMAKE_CURRENT_BINARY_DIR}/verify-binary/${A_NAME}.rigexec")
    add_test(NAME verify_binary_${A_NAME} COMMAND "${CMAKE_COMMAND}"
        "-DBAKE=$<TARGET_FILE:rigExecBakeTool>" "-DPOSE=$<TARGET_FILE:rigExecPose>"
        "-DSTAGE=${A_STAGE}"
        "-DFRAMES=${A_FRAMES}" "-DOUT=${_out}" ${_options}
        -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/runVerifyBinary.cmake")
    set(_schema_resources
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    set_property(TEST verify_binary_${A_NAME} APPEND PROPERTY ENVIRONMENT
        "PXR_PLUGINPATH_NAME=${_schema_resources}")
endfunction()

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/verify-binary")
# Every baking example, in its fixture class, dragging the table's control
# avar and operator input (fields 2-5) where the row names them.
foreach(_fixture IN LISTS RIGEXEC_EXAMPLE_FIXTURES)
    string(REPLACE "|" ";" _fixture_fields "${_fixture}")
    list(GET _fixture_fields 0 _fixture_stage)
    list(GET _fixture_fields 1 _fixture_frames)
    list(GET _fixture_fields 2 _fixture_control_prim)
    list(GET _fixture_fields 3 _fixture_control_avar)
    list(GET _fixture_fields 4 _fixture_operator_prim)
    list(GET _fixture_fields 5 _fixture_operator_input)
    list(GET _fixture_fields 6 _fixture_bakes)
    list(GET _fixture_fields 8 _fixture_animation)
    string(REGEX REPLACE "\\.(usda|usd)$" "" _fixture_name "${_fixture_stage}")
    string(REPLACE "/" "_" _fixture_name "${_fixture_name}")
    set(_fixture_drags)
    if (NOT _fixture_control_prim STREQUAL "-")
        list(APPEND _fixture_drags
            "${_fixture_control_prim}.${_fixture_control_avar}")
    endif()
    if (NOT _fixture_operator_prim STREQUAL "-")
        list(APPEND _fixture_drags
            "${_fixture_operator_prim}.${_fixture_operator_input}")
    endif()
    set(_fixture_drag_args)
    if (_fixture_drags)
        string(REPLACE ";" "," _fixture_drag_list "${_fixture_drags}")
        set(_fixture_drag_args DRAGS "${_fixture_drag_list}")
    endif()
    if (_fixture_bakes STREQUAL "YES")
        rigexec_add_verify_binary_test(NAME ${_fixture_name}
            RIG ${_fixture_stage} FRAMES ${_fixture_frames}
            CLASS ${_fixture_animation} ${_fixture_drag_args})
    endif()
endforeach()

# The stack's binary round trip; see the parity entry above for why the
rigexec_add_verify_binary_test(NAME biped_Biped_stack_anim
    RIG biped/Biped_stack_anim.usda FRAMES 1,50,100,150,200 CLASS inputs)

# The results the runtime computes rather than replays (constraint
# envelopes, current-phase weight fields), which no example exercises:
# the binary, baked at the first frame, must compute them at every frame.
rigexec_add_verify_binary_test(NAME computed_weights
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/computed_weights.usda"
    FRAMES 1,3,5,7,10 CLASS inputs)

# Property chains the runtime computes in the cases no example covers
# together: envelopes on property movers, a double chain, vec3f and matrix
# chains, the chains' diagnostics, phased consumers, and a constraint
# weight read through a chain at a declared `final`.
rigexec_add_verify_binary_test(NAME computed_chains
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/computed_chains.usda"
    FRAMES 1,3,5,7,10 CLASS inputs)

# Two-bone IK measured in an animated rigExec:space and a keyed
# rigExec:spaceMatrix, one with a keyed spaceMatrix alone, and a spline IK
# whose rest is rebuilt in the moving space: the solvers' spaceMatrix is a
# registered read the runtime both consumes and computes.
rigexec_add_verify_binary_test(NAME computed_ik_space
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/computed_ik_space.usda"
    FRAMES 1,3,5,7,10 CLASS inputs)

# The assemblers' connection-following scalar reads of every type (int,
# bool, float, a float widened to double, double3, matrix4d), including a
# float whose connection ends on a double with no value, which reads the
# head's own value. The mesh's keyed points are a chain base the binary
# holds at the bake time, so the class is static; the drags show the reads
# are computed: the keyed avar a mover input and a shader dial read through
# their connections, a keyed float dial, and the two heads whose
# connections end on the valueless double. Baked at frame 10, where the
# points have left the mush's rest points, so a drag of the mush's inputs
# moves its output (a mush of its own rest returns it). The same stage with
# the points held by a stronger default is of the inputs class: its binary
# follows the keyed double3, matrix4d and float reads at every frame.
rigexec_add_verify_binary_test(NAME computed_path_reads
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/computed_path_reads.usda"
    FRAMES 1,4,7,10 TIME 10 CLASS static
    DRAGS "/PathReadAsset/Rig/Controls/Dial.avars:amount,/PathReadAsset/Rig/Dials.gain,/PathReadAsset/Rig/Dials.held,/PathReadAsset/Rig/Movers/Mush.inputs:step")
rigexec_add_verify_binary_test(NAME computed_path_reads_held_points
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/rigExecPathReadsHeldPoints.usda"
    FRAMES 1,4,7,10 CLASS inputs)

# Space switches nested under switched controls, whose compose reads an
# ancestor recomposed before that ancestor's own switch: the nested pair,
# a pair resolved in one round, the nested pair reading keyed dials that
# hold no default, and a switch whose parent and rigExec:space are
# recomposed on a control above them. The nested pair is baked at frame 2,
# where its controls have left their rests (at rest a switch holds its
# target whatever its index), and drags each switch's inputs:activeSpace
# there.
set(_switch_drags_space_switch_nested
    TIME 2
    DRAGS "/Rig/Movers/sSpaces.inputs:activeSpace,/Rig/Movers/pSpaces.inputs:activeSpace")
foreach(_switch_fixture IN ITEMS space_switch_nested space_switch_same_round
        space_switch_dial space_switch_carry)
    rigexec_add_verify_binary_test(NAME ${_switch_fixture}
        STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/${_switch_fixture}.usda"
        FRAMES 0,1,2,3 CLASS inputs ${_switch_drags_${_switch_fixture}})
endforeach()

# Projector providers that only a projector reads: guards the export and
# its ProviderMatrix producers, not the projector values (matrix primvars
# are outside the runtime output contract verify-binary compares).
rigexec_add_verify_binary_test(NAME projector_spaces
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/projector_spaces.usda"
    FRAMES 1,4,7,10)

# One VolumePlacements step per volume (part 1), each writing its own
# WeightFrames slot, and a current-phase field reading both placements.
rigexec_add_verify_binary_test(NAME volume_placements
    STAGE "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/volume_placements.usda"
    FRAMES 1,5,10)

add_executable(testRigExecPack tests/testRigExecPack.cpp)
target_link_libraries(testRigExecPack PRIVATE rigExecStandalone)
target_compile_definitions(testRigExecPack PRIVATE ${_rigexec_schema_resource_define})
add_test(NAME testRigExecPack COMMAND testRigExecPack)

add_executable(testRigExecWeightFields tests/testRigExecWeightFields.cpp)
target_link_libraries(testRigExecWeightFields PRIVATE rigExecMath)
add_test(NAME testRigExecWeightFields COMMAND testRigExecWeightFields)

# The baked evaluation mode against the dynamic path, on the shipped biped.
add_executable(testRigExecBakedMode tests/testRigExecBakedMode.cpp)
target_link_libraries(testRigExecBakedMode PRIVATE rigExec)
target_compile_definitions(testRigExecBakedMode PRIVATE ${_rigexec_schema_resource_define})
add_test(NAME testRigExecBakedMode COMMAND testRigExecBakedMode
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

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

add_executable(testRigExecDynamicDispatch tests/testRigExecDynamicDispatch.cpp)
target_link_libraries(testRigExecDynamicDispatch PRIVATE rigExec)
target_compile_definitions(testRigExecDynamicDispatch
    PRIVATE ${_rigexec_schema_resource_define})
add_test(NAME testRigExecDynamicDispatch COMMAND testRigExecDynamicDispatch
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
add_test(NAME testRigExecDynamicDispatchProgram
    COMMAND testRigExecDynamicDispatch "${CMAKE_CURRENT_SOURCE_DIR}/examples")
set_tests_properties(testRigExecDynamicDispatchProgram PROPERTIES
    ENVIRONMENT "RIGEXEC_DYNAMIC_RUNS_PROGRAM=1")

# The step graph behind the baked program: the edges, not the values.
add_executable(testRigExecBakedSchedule tests/testRigExecBakedSchedule.cpp)
target_link_libraries(testRigExecBakedSchedule PRIVATE rigExec)
target_compile_definitions(testRigExecBakedSchedule PRIVATE ${_rigexec_schema_resource_define})
add_test(NAME testRigExecBakedSchedule COMMAND testRigExecBakedSchedule
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
foreach(_schedule serial parallel)
    add_test(NAME testRigExecBakedScheduleCones_${_schedule}
        COMMAND testRigExecBakedSchedule
        "${CMAKE_CURRENT_SOURCE_DIR}/examples")
    set_tests_properties(testRigExecBakedScheduleCones_${_schedule} PROPERTIES
        ENVIRONMENT "RIGEXEC_BAKED_VERIFY_CONES=1"
        FAIL_REGULAR_EXPRESSION "baked parity mismatch")
    set_property(TEST testRigExecBakedScheduleCones_${_schedule} APPEND
        PROPERTY ENVIRONMENT "RIGEXEC_BAKED_SCHEDULE=${_schedule}")
    # And at grain zero, one step per cluster (see CONE_VERIFY above).
    add_test(NAME testRigExecBakedScheduleCones_grain0_${_schedule}
        COMMAND testRigExecBakedSchedule
        "${CMAKE_CURRENT_SOURCE_DIR}/examples")
    set_tests_properties(testRigExecBakedScheduleCones_grain0_${_schedule}
        PROPERTIES ENVIRONMENT "RIGEXEC_BAKED_VERIFY_CONES=1"
        FAIL_REGULAR_EXPRESSION "baked parity mismatch")
    set_property(TEST testRigExecBakedScheduleCones_grain0_${_schedule} APPEND
        PROPERTY ENVIRONMENT "RIGEXEC_BAKED_SCHEDULE=${_schedule}"
        "RIGEXEC_BAKED_GRAIN_US=0")
endforeach()

# The baked op trace: completion order against the step graph, under both
# executors.
add_executable(testRigExecOpTrace tests/testRigExecOpTrace.cpp)
target_link_libraries(testRigExecOpTrace PRIVATE rigExec)
target_compile_definitions(testRigExecOpTrace PRIVATE ${_rigexec_schema_resource_define})
foreach(_schedule serial parallel)
    add_test(NAME testRigExecOpTrace_${_schedule}
        COMMAND testRigExecOpTrace "${CMAKE_CURRENT_SOURCE_DIR}/examples")
    set_tests_properties(testRigExecOpTrace_${_schedule} PROPERTIES
        ENVIRONMENT "RIGEXEC_BAKED_SCHEDULE=${_schedule}")
    set_property(TEST testRigExecOpTrace_${_schedule} APPEND
        PROPERTY ENVIRONMENT "RIGEXEC_ENABLE_PARALLEL_EVAL=1")
endforeach()

# The one-loop acceptance criteria on their fixtures, under both executors.
add_executable(testRigExecOneLoopAcceptance tests/testRigExecOneLoopAcceptance.cpp)
target_link_libraries(testRigExecOneLoopAcceptance PRIVATE rigExec)
target_compile_definitions(testRigExecOneLoopAcceptance PRIVATE ${_rigexec_schema_resource_define})
foreach(_schedule serial parallel)
    add_test(NAME testRigExecOneLoopAcceptance_${_schedule}
        COMMAND testRigExecOneLoopAcceptance "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures")
    set_tests_properties(testRigExecOneLoopAcceptance_${_schedule} PROPERTIES
        ENVIRONMENT "RIGEXEC_BAKED_SCHEDULE=${_schedule}")
    set_property(TEST testRigExecOneLoopAcceptance_${_schedule} APPEND
        PROPERTY ENVIRONMENT "RIGEXEC_ENABLE_PARALLEL_EVAL=1")
endforeach()

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
rigexec_add_baked_parity_test(testRigExecSolverStacking CONE_VERIFY
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecArm tests/testRigExecArm.cpp)
target_link_libraries(testRigExecArm PRIVATE rigExec)
target_compile_definitions(testRigExecArm PRIVATE ${_rigexec_schema_resource_define})
add_test(NAME testRigExecArm COMMAND testRigExecArm
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
rigexec_add_baked_parity_test(testRigExecArm REQUIRE_BAKE CONE_VERIFY
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecImaging tests/testRigExecImaging.cpp)
target_link_libraries(testRigExecImaging PRIVATE rigExecImaging)
target_compile_definitions(testRigExecImaging PRIVATE
    ${_rigexec_schema_resource_define} ${_rigexec_imaging_resource_define})
add_test(NAME testRigExecImaging COMMAND testRigExecImaging
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecProfilerApi tests/testRigExecProfilerApi.cpp)
target_link_libraries(testRigExecProfilerApi PRIVATE rigExecImaging js)
target_compile_definitions(testRigExecProfilerApi PRIVATE ${_rigexec_schema_resource_define})
add_test(NAME testRigExecProfilerApi COMMAND testRigExecProfilerApi
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecImagingMultiStage
    tests/testRigExecImagingMultiStage.cpp)
target_link_libraries(testRigExecImagingMultiStage PRIVATE rigExecImaging)
target_compile_definitions(testRigExecImagingMultiStage PRIVATE
    ${_rigexec_schema_resource_define} ${_rigexec_imaging_resource_define})
add_test(NAME testRigExecImagingMultiStage COMMAND testRigExecImagingMultiStage
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
rigexec_add_baked_parity_test(testRigExecVolumeWeights REQUIRE_BAKE
    CONE_VERIFY "${CMAKE_CURRENT_SOURCE_DIR}/examples")
foreach(_entry testRigExecVolumeWeightsBakedParity
               testRigExecVolumeWeightsCones_serial
               testRigExecVolumeWeightsCones_parallel
               testRigExecVolumeWeightsCones_grain0_serial
               testRigExecVolumeWeightsCones_grain0_parallel)
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
target_compile_definitions(testRigExecMoverGraph PRIVATE
    ${_rigexec_schema_resource_define})
add_test(NAME testRigExecMoverGraph COMMAND testRigExecMoverGraph)

# The epoch-fixed skin layout: what has to invalidate it, and what Compile
add_executable(testRigExecSkinTopology tests/testRigExecSkinTopology.cpp)
target_link_libraries(testRigExecSkinTopology PRIVATE rigExec)
target_compile_definitions(testRigExecSkinTopology PRIVATE
    ${_rigexec_schema_resource_define})
add_test(NAME testRigExecSkinTopology COMMAND testRigExecSkinTopology)
set_tests_properties(testRigExecSkinTopology PROPERTIES
    ENVIRONMENT "RIGEXEC_EVALUATION_MODE=reference")

add_executable(testRigExecStaticInputCache
    tests/testRigExecStaticInputCache.cpp)
target_link_libraries(testRigExecStaticInputCache PRIVATE rigExec)
add_test(NAME testRigExecStaticInputCache COMMAND testRigExecStaticInputCache)

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
set_tests_properties(testRigExecEpochRests PROPERTIES
    ENVIRONMENT "RIGEXEC_EVALUATION_MODE=reference")
rigexec_add_baked_parity_test(testRigExecEpochRests REQUIRE_BAKE CONE_VERIFY
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

# Bit-identical results from the parallel per-point kernels, both ways round:
add_executable(testRigExecParallelKernels tests/testRigExecParallelKernels.cpp)
target_link_libraries(testRigExecParallelKernels PRIVATE rigExec work)
add_test(NAME testRigExecParallelKernels COMMAND testRigExecParallelKernels)
add_test(NAME testRigExecParallelKernelsSerial
    COMMAND testRigExecParallelKernels)
set_tests_properties(testRigExecParallelKernelsSerial PROPERTIES
    ENVIRONMENT "RIGEXEC_ENABLE_PARALLEL_EVAL=0")

add_executable(testRigExecChainLevels tests/testRigExecChainLevels.cpp)
target_link_libraries(testRigExecChainLevels PRIVATE rigExec work)
target_compile_definitions(testRigExecChainLevels PRIVATE
    ${_rigexec_schema_resource_define})
add_test(NAME testRigExecChainLevels COMMAND testRigExecChainLevels)
add_test(NAME testRigExecChainLevelsSerial COMMAND testRigExecChainLevels)
set_tests_properties(testRigExecChainLevelsSerial PROPERTIES
    ENVIRONMENT "RIGEXEC_ENABLE_PARALLEL_EVAL=0")
set_property(TEST testRigExecChainLevels testRigExecChainLevelsSerial APPEND
    PROPERTY ENVIRONMENT "RIGEXEC_EVALUATION_MODE=reference")
add_test(NAME testRigExecChainLevelsRepeat
    COMMAND testRigExecChainLevels 20)
set_tests_properties(testRigExecChainLevelsRepeat PROPERTIES
    ENVIRONMENT "RIGEXEC_EVALUATION_MODE=reference")

add_executable(testRigExecProfiler tests/testRigExecProfiler.cpp)
target_link_libraries(testRigExecProfiler PRIVATE rigExec)
add_test(NAME testRigExecProfiler COMMAND testRigExecProfiler)

# Measures what the compile's path bookkeeping costs; asserts nothing, so it
add_executable(benchPathLookup tests/benchPathLookup.cpp)
target_link_libraries(benchPathLookup PRIVATE rigExec)

# Scratch: which USD reads stop scaling across threads (see the file header).
add_executable(benchStageReads tests/benchStageReads.cpp)
target_link_libraries(benchStageReads PRIVATE rigExec)

add_executable(testRigExecFrameCache tests/testRigExecFrameCache.cpp)
target_link_libraries(testRigExecFrameCache PRIVATE rigExec)
add_test(NAME testRigExecFrameCache COMMAND testRigExecFrameCache)

add_executable(testRigExecFrozenContext tests/testRigExecFrozenContext.cpp)
target_link_libraries(testRigExecFrozenContext PRIVATE rigExec rigExecRigging)
add_test(NAME testRigExecFrozenContext COMMAND testRigExecFrozenContext "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecBackgroundScheduler tests/testRigExecBackgroundScheduler.cpp)
target_link_libraries(testRigExecBackgroundScheduler PRIVATE rigExec)
add_test(NAME testRigExecBackgroundScheduler COMMAND testRigExecBackgroundScheduler)

add_executable(testRigExecFrameCacheSparsity tests/testRigExecFrameCacheSparsity.cpp)
target_link_libraries(testRigExecFrameCacheSparsity PRIVATE rigExec)
target_compile_definitions(testRigExecFrameCacheSparsity PRIVATE
    ${_rigexec_schema_resource_define})
add_test(NAME testRigExecFrameCacheSparsity COMMAND testRigExecFrameCacheSparsity)
# One step per cluster, so that a switched group and the provider it
# recomposes land in different clusters (TestRecomposedVersionSeedsSparseReuse).
set_tests_properties(testRigExecFrameCacheSparsity PROPERTIES
    ENVIRONMENT "RIGEXEC_BAKED_GRAIN_US=0")

add_executable(testRigExecFrameCacheCones_Verify tests/testRigExecFrameCacheCones_Verify.cpp)
target_link_libraries(testRigExecFrameCacheCones_Verify PRIVATE rigExec)
add_test(NAME testRigExecFrameCacheCones_Verify COMMAND testRigExecFrameCacheCones_Verify "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecImagingFrameCache tests/testRigExecImagingFrameCache.cpp)
target_link_libraries(testRigExecImagingFrameCache PRIVATE rigExecImaging rigExecBake)
target_include_directories(testRigExecImagingFrameCache PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/tests")
target_compile_definitions(testRigExecImagingFrameCache PRIVATE
    ${_rigexec_schema_resource_define} ${_rigexec_imaging_resource_define})
add_test(NAME testRigExecImagingFrameCache COMMAND testRigExecImagingFrameCache
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
add_test(NAME testRigExecImagingFrameCacheDefault COMMAND "${CMAKE_COMMAND}" -E env
    --unset=RIGEXEC_EVALUATION_MODE --unset=RIGEXEC_DYNAMIC_RUNS_PROGRAM
    RIGEXEC_FRAME_CACHE=on RIGEXEC_ENABLE_PARALLEL_EVAL=1
    "$<TARGET_FILE:testRigExecImagingFrameCache>"
    "${CMAKE_CURRENT_SOURCE_DIR}/examples" --default-mode)

foreach(_rigexec_plugin_test testRigExecFrameCacheCones_Verify
        testRigExecFrozenContext
        testRigExecImagingFrameCache
        testRigExecImagingFrameCacheDefault)
    set_property(TEST ${_rigexec_plugin_test} APPEND PROPERTY
        ENVIRONMENT_MODIFICATION
        "PXR_PLUGINPATH_NAME=path_list_prepend:${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources"
        "PXR_PLUGINPATH_NAME=path_list_prepend:${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecImaging/resources")
endforeach()

add_executable(benchFrameCache tests/benchFrameCache.cpp)
target_link_libraries(benchFrameCache PRIVATE rigExec)
target_compile_definitions(benchFrameCache PRIVATE
    ${_rigexec_schema_resource_define})

add_executable(benchFrameCacheWarm tests/benchFrameCacheWarm.cpp)
target_link_libraries(benchFrameCacheWarm PRIVATE rigExec)
target_compile_definitions(benchFrameCacheWarm PRIVATE
    ${_rigexec_schema_resource_define})

add_executable(benchCommitLag tests/benchCommitLag.cpp)
target_link_libraries(benchCommitLag PRIVATE rigExecImaging rigExecBake)
target_include_directories(benchCommitLag PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/tests")
target_compile_definitions(benchCommitLag PRIVATE
    ${_rigexec_schema_resource_define} ${_rigexec_imaging_resource_define})

add_executable(benchEditLatency tests/benchEditLatency.cpp)
target_link_libraries(benchEditLatency PRIVATE rigExec)
target_compile_definitions(benchEditLatency PRIVATE
    ${_rigexec_schema_resource_define})

add_executable(benchPlayback tests/benchPlayback.cpp)
target_link_libraries(benchPlayback PRIVATE rigExecImaging rigExecBake)
target_include_directories(benchPlayback PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/tests")
target_compile_definitions(benchPlayback PRIVATE
    ${_rigexec_schema_resource_define} ${_rigexec_imaging_resource_define})

add_executable(testRigExecInteractive tests/testRigExecInteractive.cpp)
target_link_libraries(testRigExecInteractive PRIVATE rigExec)
target_compile_definitions(testRigExecInteractive PRIVATE
    ${_rigexec_schema_resource_define})
add_test(NAME testRigExecInteractive COMMAND testRigExecInteractive)
rigexec_add_baked_parity_test(testRigExecInteractive REQUIRE_BAKE CONE_VERIFY)

add_executable(testRigExecValueEdits tests/testRigExecValueEdits.cpp)
target_link_libraries(testRigExecValueEdits PRIVATE rigExec)
target_compile_definitions(testRigExecValueEdits PRIVATE
    ${_rigexec_schema_resource_define})
add_test(NAME testRigExecValueEdits COMMAND testRigExecValueEdits
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")
rigexec_add_baked_parity_test(testRigExecValueEdits REQUIRE_BAKE CONE_VERIFY
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecScopedClears tests/testRigExecScopedClears.cpp)
target_link_libraries(testRigExecScopedClears PRIVATE rigExec)
target_compile_definitions(testRigExecScopedClears PRIVATE
    ${_rigexec_schema_resource_define})
add_test(NAME testRigExecScopedClears COMMAND testRigExecScopedClears)
rigexec_add_baked_parity_test(testRigExecScopedClears CONE_VERIFY)

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
add_executable(testRigExecNoAuthoring tests/testRigExecNoAuthoring.cpp)
target_link_libraries(testRigExecNoAuthoring PRIVATE rigExecImaging)
target_compile_definitions(testRigExecNoAuthoring PRIVATE ${_rigexec_schema_resource_define})
add_test(NAME testRigExecNoAuthoring COMMAND testRigExecNoAuthoring
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecBounds tests/testRigExecBounds.cpp)
target_link_libraries(testRigExecBounds PRIVATE plug tf gf usd usdGeom)
target_compile_definitions(testRigExecBounds PRIVATE ${_rigexec_schema_resource_define})
add_dependencies(testRigExecBounds rigExecImaging)
add_test(NAME testRigExecBounds COMMAND testRigExecBounds
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(probeCodingError tests/probeCodingError.cpp)
target_link_libraries(probeCodingError PRIVATE rigExec)
target_compile_definitions(probeCodingError PRIVATE ${_rigexec_schema_resource_define})

add_executable(probeImagingPipeline tests/probeImagingPipeline.cpp)
target_link_libraries(probeImagingPipeline PRIVATE rigExecImaging usdImaging usdGeom)
target_compile_definitions(probeImagingPipeline PRIVATE ${_rigexec_schema_resource_define})

# The non-public Esf/Exec seam must qualify separately on each stock USD
add_executable(probeEsfCompatibility EXCLUDE_FROM_ALL tests/probeEsfCompatibility.cpp)
target_link_libraries(probeEsfCompatibility PRIVATE exec esfUsd usd)

function(rigexec_add_reference_test name)
    add_test(NAME ${name}Reference COMMAND ${name} ${ARGN})
    set(_env "RIGEXEC_EVALUATION_MODE=reference")
    get_test_property(${name} ENVIRONMENT _plain_env)
    if (_plain_env)
        list(APPEND _env ${_plain_env})
    endif()
    set_tests_properties(${name}Reference PROPERTIES ENVIRONMENT "${_env}")
    get_test_property(${name} ENVIRONMENT_MODIFICATION _plain_modification)
    if (_plain_modification)
        set_tests_properties(${name}Reference PROPERTIES
            ENVIRONMENT_MODIFICATION "${_plain_modification}")
    endif()
    get_test_property(${name} FAIL_REGULAR_EXPRESSION _plain_fail)
    if (_plain_fail)
        set_tests_properties(${name}Reference PROPERTIES
            FAIL_REGULAR_EXPRESSION "${_plain_fail}")
    endif()
endfunction()
foreach(_suite testRigExecBackgroundScheduler testRigExecConstraints
               testRigExecFrameCacheSparsity
               testRigExecInteractive testRigExecPoseInterpolator)
    rigexec_add_reference_test(${_suite})
endforeach()

function(rigexec_add_digest_gate_test name)
    add_test(NAME ${name}DigestGate COMMAND ${ARGN})
    set(_env "RIGEXEC_VERIFY_DIGEST_GATE=1")
    get_test_property(${name} ENVIRONMENT _plain_env)
    if (_plain_env)
        list(APPEND _env ${_plain_env})
    endif()
    set_tests_properties(${name}DigestGate PROPERTIES ENVIRONMENT "${_env}")
    get_test_property(${name} ENVIRONMENT_MODIFICATION _plain_modification)
    if (_plain_modification)
        set_tests_properties(${name}DigestGate PROPERTIES
            ENVIRONMENT_MODIFICATION "${_plain_modification}")
    endif()
endfunction()
foreach(_suite testRigExecConstraints testRigExecInteractive)
    rigexec_add_digest_gate_test(${_suite} ${_suite})
endforeach()

function(rigexec_add_scoped_clear_test name)
    add_test(NAME ${name}ScopedClears COMMAND ${ARGN})
    set(_env "RIGEXEC_VERIFY_SCOPED_CLEARS=1")
    get_test_property(${name} ENVIRONMENT _plain_env)
    if (_plain_env)
        list(APPEND _env ${_plain_env})
    endif()
    set_tests_properties(${name}ScopedClears PROPERTIES
        ENVIRONMENT "${_env}"
        FAIL_REGULAR_EXPRESSION "baked parity mismatch")
    get_test_property(${name} ENVIRONMENT_MODIFICATION _plain_modification)
    if (_plain_modification)
        set_tests_properties(${name}ScopedClears PROPERTIES
            ENVIRONMENT_MODIFICATION "${_plain_modification}")
    endif()
endfunction()
foreach(_suite testRigExecInteractive testRigExecScopedClears
               testRigExecSkinTopology)
    rigexec_add_scoped_clear_test(${_suite} ${_suite})
endforeach()
foreach(_suite testRigExecBakedMode testRigExecValueEdits)
    rigexec_add_scoped_clear_test(${_suite} ${_suite}
        "${CMAKE_CURRENT_SOURCE_DIR}/examples")
endforeach()
foreach(_suite testRigExecEpochRests testRigExecNoAuthoring)
    rigexec_add_digest_gate_test(${_suite} ${_suite}
        "${CMAKE_CURRENT_SOURCE_DIR}/examples")
endforeach()
foreach(_suite testRigExecArm testRigExecBakedMode testRigExecBakedSchedule
               testRigExecBinary
               testRigExecFrameCacheCones_Verify testRigExecFrozenContext
               testRigExecImaging testRigExecImagingPlayback
               testRigExecNoAuthoring testRigExecRuntimeGeometry
               testRigExecRuntimeLoader testRigExecRuntimePose
               testRigExecRuntimeWeights testRigExecSolverBake
               testRigExecSolverStacking testRigExecVolumeWeights
               testRigExecWeightOverlay)
    rigexec_add_reference_test(${_suite}
        "${CMAKE_CURRENT_SOURCE_DIR}/examples")
endforeach()

endif()  # RIGEXEC_BUILD_TESTS

if (CMAKE_CROSSCOMPILING)
    set(_rigexec_python_default OFF)
else()
    # Probe quietly: the interpreter must be found with dev headers, and the
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

if (NOT CMAKE_CROSSCOMPILING AND TARGET python AND TARGET garch AND TARGET glf)
    set(_rigexec_usdnoodles_default ON)
else()
    set(_rigexec_usdnoodles_default OFF)
endif()
option(RIGEXEC_BUILD_USDNOODLES
       "Build the usdNoodles node-graph editor plugin for usdview"
       ${_rigexec_usdnoodles_default})

if (RIGEXEC_BUILD_USDNOODLES)
    find_package(OpenGL REQUIRED)
    add_subdirectory(plugin/usdNoodles)
endif()

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
    add_test(NAME testRigExecBipedHandPython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_biped_hand.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRigExecTrsMastersPython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_trs_masters.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRigExecWireInvariantPython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_wire_invariant.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    # The deformation stack must keep running in its authored order: the head
    # wires are the first four deformers on the face and everything after
    # them is shaped by what they leave behind.
    add_test(NAME testRigExecDeformerOrderPython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_deformer_order.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    # A pose sample sitting on top of its neutral makes the RBF degenerate
    # and the weights run away -- the right deltoid ballooned this way.
    add_test(NAME testRigExecPoseRotationsPython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_pose_rotations.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    # The projector's ray is the SOURCE FRAME's Z axis, and the two eye
    # bind joints are 180 degrees apart rather than mirrored, so the right
    # eye once cast into the back of its own eyeball.
    add_test(NAME testRigExecEyeSymmetryPython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_eye_symmetry.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    # A drag is a SWEEP: a deformer correct at both ends of one can
    # still jump in the middle, and every other check here sets one
    # value and looks once.
    add_test(NAME testRigExecFaceSmoothPython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_face_smooth.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    # The spine's volume preservation must stay in the spine: an
    # inherited shear rode through the head's scale constraints and grew
    # the cranium 6.56% when the hips moved.
    add_test(NAME testRigExecRigidHeadPython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_rigid_head.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    # A radial weight turns a fraction of the rotation about the pivot the
    # transform turns about; a linear one follows the chord, which falls
    # inside the arc. On a blink that walks the lid into the eyeball.
    add_test(NAME testRigExecBlinkClearancePython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_blink_clearance.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources"
            "$<TARGET_FILE:rigExecBakeTool>" "$<TARGET_FILE:rigExecPose>")
    add_test(NAME testRigExecFaceDrivesPython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_face_drives.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    # inputs:enabled must actually switch a deformer off: the Deformer
    # Toggles panel is only a bisecting tool if the switch it writes is
    # one the evaluator obeys.
    add_test(NAME testRigExecDeformerEnabledPython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_deformer_enabled.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRigExecPoseCyclePython
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_pose_cycle.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRigExecPsdNumeric COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_psd_numeric.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRigExecSpaceSwitch COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_space_switch.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRigExecLiveOperationGraph COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_live_operation_graph.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    # Switches nested under switched controls: bakeable, and the program,
    # the walk and the drag path agree bit for bit; a switch reading under
    # its own target is refused.
    add_test(NAME testRigExecSpaceSwitchVersions COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_space_switch_versions.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources"
        "${CMAKE_CURRENT_SOURCE_DIR}/examples")
    # Again with every frame's cone checked against the whole program, one
    # step per cluster: a recomposed version reads avars a cone must reach.
    add_test(NAME testRigExecSpaceSwitchVersionsCones
        COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_space_switch_versions.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources"
        "${CMAKE_CURRENT_SOURCE_DIR}/examples")
    add_test(NAME testRigExecBake COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_bake.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRigExecInverse COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_inverse.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRigExecSkinLayoutOverrides COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_skin_layout_overrides.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRigExecBakedCone COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_baked_cone.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRigExecOpTracePython COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_op_trace.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRigExecGil COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_gil.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
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
    add_test(NAME testRigExecMoverDiscovery COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_mover_discovery.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRigExecSkippedOperations COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_skipped_operations.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRigExecBakedAvarPatch COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_baked_avar_patch.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRigExecPxrHygiene COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_pxr_import_hygiene.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRigExecBakedAttributePython
        COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_baked_attribute.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")

    add_test(NAME testRigExecUndo
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_undo.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testGizmoMath
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_gizmo_math.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testPoseReaderViz
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_pose_reader_viz.py"
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
    add_test(NAME testPathSpelling
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_path_spelling.py")
    add_test(NAME testMoverChainOrder
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_mover_chain_order.py")
    add_test(NAME testIkFkMatch
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_ikfk_match.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testSpaceMatch
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_space_match.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testAutoClavicle
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_auto_clavicle.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
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
    add_test(NAME testCacheStripModel
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_cache_strip_model.py")
    add_test(NAME testCacheStripPanel
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_cache_strip_panel.py")
    add_test(NAME testUsdviewFrameCacheDriver
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_usdview_frame_cache.py")
    add_test(NAME testImagingHandle
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_imaging_handle.py")
    add_test(NAME testSessionState
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_session_state.py")

    foreach(_python_test testRigExecPython testRigExecPythonSchemaAuthoring
                         testRigExecSplineIkPython
                         testRigExecFkControlSpacePython
                         testRigExecBake testRigExecInverse
                         testRigExecPsdNumeric testRigExecSpaceSwitch
                         testRigExecLiveOperationGraph
                         testRigExecSpaceSwitchVersions
                         testRigExecSpaceSwitchVersionsCones
                         testRestLocal testRestMigration
                         testInterveningXform testRigExecEpochConstants
                         testRigExecBakedAvarPatch testRigExecWireMover
                         testRigExecPxrHygiene
                         testRigExecMatrixSpace
                         testRigExecMoverDiscovery
                         testRigExecSkippedOperations
                         testRigExecFloatCurveOp
                         testRigExecOpTracePython
                         testRigExecUndo testGizmoMath testGizmoScreen
                         testPoseReaderViz
                         testLayerOpinionsModel testCompositionArcsModel
                         testPathSpelling
                         testMoverChainOrder testIkFkMatch testSpaceMatch
                         testAutoClavicle
                         testAvarEditorModel
                         testGizmoPreview
                         testGizmoSettings testGizmoDrag testGizmoSnap
                         testViewCubeMath testRigExecStageEdits
                         testGraphModel testGraphScreen testCacheStripModel
                         testCacheStripPanel testUsdviewFrameCacheDriver
                         testImagingHandle
                         testSessionState)
        set_tests_properties(${_python_test} PROPERTIES ENVIRONMENT
            "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/python")
        set_property(TEST ${_python_test} APPEND PROPERTY ENVIRONMENT
            "PXR_PLUGINPATH_NAME=${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
        set_property(TEST ${_python_test} APPEND PROPERTY ENVIRONMENT
            "RIGEXEC_USD_INSTALL=${RIGEXEC_TEST_USD_DIR}")
    endforeach()
    set_property(TEST testRigExecSpaceSwitchVersionsCones APPEND PROPERTY
        ENVIRONMENT "RIGEXEC_BAKED_VERIFY_CONES=1" "RIGEXEC_BAKED_GRAIN_US=0")
    rigexec_add_digest_gate_test(testRigExecStageEdits
        ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_stage_edits.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    rigexec_add_scoped_clear_test(testRigExecStageEdits
        ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_stage_edits.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
endif()

if (RIGEXEC_BUILD_USDNOODLES AND RIGEXEC_BUILD_TESTS AND Python3_FOUND)
    add_test(NAME testUsdNoodles
        COMMAND ${Python3_EXECUTABLE} -m unittest discover -s . -p "test*.py"
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/python/UsdNoodles/testenv")
    set_tests_properties(testUsdNoodles PROPERTIES ENVIRONMENT
        "RIGEXEC_USD_INSTALL=${RIGEXEC_TEST_USD_DIR}")
    file(GLOB _rigexec_usd_site_dirs LIST_DIRECTORIES true
        "${RIGEXEC_TEST_USD_DIR}/Lib/site-packages"
        "${RIGEXEC_TEST_USD_DIR}/lib/python*/site-packages"
        "${RIGEXEC_TEST_USD_DIR}/lib/python")
    foreach(_rigexec_usd_site_dir IN LISTS _rigexec_usd_site_dirs)
        if (IS_DIRECTORY "${_rigexec_usd_site_dir}/pxr")
            set_property(TEST testUsdNoodles APPEND PROPERTY
                ENVIRONMENT_MODIFICATION
                "PYTHONPATH=path_list_prepend:${_rigexec_usd_site_dir}")
        endif()
    endforeach()
    set_property(TEST testUsdNoodles APPEND PROPERTY ENVIRONMENT_MODIFICATION
        "PYTHONPATH=path_list_prepend:${CMAKE_CURRENT_BINARY_DIR}/python")
    set_property(TEST testUsdNoodles APPEND PROPERTY ENVIRONMENT_MODIFICATION
        "PYTHONPATH=path_list_prepend:${CMAKE_CURRENT_BINARY_DIR}/python/UsdNoodles/testenv")
endif()

# External repositories supply their own CMakeLists and register plugin targets.
set(RIGEXEC_MOVER_PLUGIN_DIRS "" CACHE STRING
    "Semicolon-separated paths to external mover plugin source directories")
foreach(_mover_dir IN LISTS RIGEXEC_MOVER_PLUGIN_DIRS)
    get_filename_component(_mover_dir "${_mover_dir}" ABSOLUTE
        BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
    if (NOT EXISTS "${_mover_dir}/CMakeLists.txt")
        message(FATAL_ERROR "Mover plugin directory has no CMakeLists.txt: ${_mover_dir}")
    endif()
    string(SHA256 _mover_id "${_mover_dir}")
    add_subdirectory("${_mover_dir}" "${CMAKE_CURRENT_BINARY_DIR}/mover-plugins/${_mover_id}")
endforeach()

get_property(_rigexec_tests DIRECTORY PROPERTY TESTS)
foreach(_rigexec_test IN LISTS _rigexec_tests)
    set_tests_properties(${_rigexec_test} PROPERTIES TIMEOUT 120)
    if (WIN32)
        set_property(TEST ${_rigexec_test} APPEND PROPERTY
            ENVIRONMENT_MODIFICATION
            "PATH=path_list_prepend:${RIGEXEC_TEST_USD_DIR}/bin"
            "PATH=path_list_prepend:${RIGEXEC_TEST_USD_DIR}/lib"
            "PATH=path_list_prepend:${CMAKE_CURRENT_BINARY_DIR}")
    elseif (APPLE)
        # Mach-O reads DYLD_LIBRARY_PATH and ELF reads LD_LIBRARY_PATH;
        set_property(TEST ${_rigexec_test} APPEND PROPERTY
            ENVIRONMENT_MODIFICATION
            "DYLD_LIBRARY_PATH=path_list_prepend:${RIGEXEC_TEST_USD_DIR}/lib"
            "DYLD_LIBRARY_PATH=path_list_prepend:${CMAKE_CURRENT_BINARY_DIR}")
    else()
        set_property(TEST ${_rigexec_test} APPEND PROPERTY
            ENVIRONMENT_MODIFICATION
            "LD_LIBRARY_PATH=path_list_prepend:${RIGEXEC_TEST_USD_DIR}/lib"
            "LD_LIBRARY_PATH=path_list_prepend:${CMAKE_CURRENT_BINARY_DIR}")
    endif()
endforeach()

# Install
include(CMakePackageConfigHelpers)

install(TARGETS rigExecMath rigExec rigExecStandalone rigExecRigging rigExecImaging rigExecFlatBuffers rigExecBinary rigExecBake rigExecRuntime rigExecSampler
    EXPORT rigExecTargets
    RUNTIME DESTINATION "${RIGEXEC_INSTALL_LIBDIR}"
    LIBRARY DESTINATION "${RIGEXEC_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION "${RIGEXEC_INSTALL_LIBDIR}")

# FILES_MATCHING still walks -- and recreates -- every subdirectory it passes
foreach(_lib rigExecMath rigExec rigExecStandalone rigExecRigging rigExecImaging rigExecBinary rigExecBake rigExecRuntime rigExecSampler)
    install(DIRECTORY "libs/${_lib}/"
        DESTINATION "${RIGEXEC_INSTALL_INCLUDEDIR}/${_lib}"
        FILES_MATCHING
            PATTERN ".*" EXCLUDE
            PATTERN "__pycache__" EXCLUDE
            PATTERN "*.h")
endforeach()
# The FlatBuffers runtime headers the generated rigExecBinary headers
# include; their license installs with the others below.
install(DIRECTORY thirdparty/flatbuffers/include/flatbuffers
    DESTINATION "${RIGEXEC_INSTALL_INCLUDEDIR}")

set(_rigexec_schema_resources
    "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
set(_rigexec_schema_pluginfo "${_rigexec_schema_resources}/plugInfo.json")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/plugin/rigExecSchema/resources/plugInfo.json"
    "${CMAKE_CURRENT_SOURCE_DIR}/plugin/rigExecSchema/resources/generatedSchema.usda")
file(READ
    "${CMAKE_CURRENT_SOURCE_DIR}/plugin/rigExecSchema/resources/plugInfo.json"
    _rigexec_schema_pluginfo_content)
string(REPLACE "\"Type\": \"resource\"" "\"Type\": \"library\""
    _rigexec_schema_pluginfo_content "${_rigexec_schema_pluginfo_content}")

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

# The node-graph icons the plugInfo type entries name (a class's
# extraPlugInfo nodeGraphIcon, relative to these resources): exactly those,
# so the plugin ships no art it does not declare.
file(STRINGS
    "${CMAKE_CURRENT_SOURCE_DIR}/plugin/rigExecSchema/resources/plugInfo.json"
    _rigexec_icon_lines REGEX "\"nodeGraphIcon\"")
set(_rigexec_schema_icons "")
foreach(_rigexec_icon_line IN LISTS _rigexec_icon_lines)
    string(REGEX REPLACE ".*\"nodeGraphIcon\": \"icons/([^\"]+)\".*" "\\1"
        _rigexec_icon "${_rigexec_icon_line}")
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/icons/${_rigexec_icon}"
        "${_rigexec_schema_resources}/icons/${_rigexec_icon}" COPYONLY)
    list(APPEND _rigexec_schema_icons
        "${CMAKE_CURRENT_SOURCE_DIR}/icons/${_rigexec_icon}")
endforeach()

install(FILES
        "${_rigexec_schema_pluginfo}"
        "${_rigexec_schema_resources}/generatedSchema.usda"
    DESTINATION "${RIGEXEC_INSTALL_PLUGINDIR}/rigExecSchema/resources")
install(FILES ${_rigexec_schema_icons}
    DESTINATION "${RIGEXEC_INSTALL_PLUGINDIR}/rigExecSchema/resources/icons")

install(DIRECTORY plugin/rigExecUsdview/
    DESTINATION "${RIGEXEC_INSTALL_PYTHONDIR}/rigExecUsdview"
    FILES_MATCHING
        PATTERN ".*" EXCLUDE
        PATTERN "__pycache__" EXCLUDE
        PATTERN "*.py"
        PATTERN "*.json"
        PATTERN "*.png")


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
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/rigExecMoverPlugin.cmake"
    DESTINATION "${RIGEXEC_INSTALL_CMAKEDIR}")

install(FILES LICENSE NOTICE THIRD_PARTY_NOTICES.md
    DESTINATION "share/rigExec")
install(FILES plugin/usdNoodles/LICENSE.txt
    DESTINATION "share/rigExec/licenses" RENAME OpenUSD-LICENSE.txt)
install(FILES thirdparty/flatbuffers/LICENSE
    DESTINATION "share/rigExec/licenses" RENAME FlatBuffers-LICENSE.txt)
