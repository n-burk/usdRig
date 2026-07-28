cmake_minimum_required(VERSION 3.26)
project(rigExec LANGUAGES CXX)

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
# rigExecMath: pure point-frame and solver kernels (spec §5).
# ---------------------------------------------------------------------------
add_library(rigExecMath STATIC
    libs/rigExecMath/pointFrame.cpp
    libs/rigExecMath/solvers.cpp
    libs/rigExecMath/geometryKernels.cpp
    libs/rigExecMath/simdKernels.cpp
)
target_include_directories(rigExecMath PUBLIC libs)
target_link_libraries(rigExecMath PUBLIC arch tf gf vt)

# ---------------------------------------------------------------------------
# rigExec: exec type registrations, schema computations, tap set, and the
# staged rig evaluator (spec §§9, 12).
# ---------------------------------------------------------------------------
add_library(rigExec SHARED
    libs/rigExec/types.cpp
    libs/rigExec/computations.cpp
    libs/rigExec/moverKernels.cpp
    libs/rigExec/moverGraph.cpp
    libs/rigExec/tapSet.cpp
    libs/rigExec/rigEvaluator.cpp
)
target_include_directories(rigExec PUBLIC libs)
target_link_libraries(rigExec PUBLIC
    rigExecMath
    plug sdf usd usdGeom
    exec execUsd ef vdf
)

# ---------------------------------------------------------------------------
# rigExecImaging: Hydra 2.0 scene-index publication (spec §10).
# ---------------------------------------------------------------------------
add_library(rigExecImaging SHARED
    libs/rigExecImaging/sceneIndices.cpp
    libs/rigExecImaging/bridge.cpp
    libs/rigExecImaging/registry.cpp
    libs/rigExecImaging/sceneIndexPlugin.cpp
)
target_include_directories(rigExecImaging PUBLIC libs)
target_link_libraries(rigExecImaging PUBLIC
    rigExec
    hd
    usdImaging
    usdUtils
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
enable_testing()

add_executable(testRigExecMath tests/testRigExecMath.cpp)
target_link_libraries(testRigExecMath PRIVATE rigExecMath)
add_test(NAME testRigExecMath COMMAND testRigExecMath)

add_executable(testRigExecArm tests/testRigExecArm.cpp)
target_link_libraries(testRigExecArm PRIVATE rigExec)
add_test(NAME testRigExecArm COMMAND testRigExecArm
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecImaging tests/testRigExecImaging.cpp)
target_link_libraries(testRigExecImaging PRIVATE rigExecImaging)
add_test(NAME testRigExecImaging COMMAND testRigExecImaging
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecMoverGraph tests/testRigExecMoverGraph.cpp)
target_link_libraries(testRigExecMoverGraph PRIVATE rigExec)
add_test(NAME testRigExecMoverGraph COMMAND testRigExecMoverGraph)

add_executable(probeCodingError tests/probeCodingError.cpp)
target_link_libraries(probeCodingError PRIVATE rigExec)

# End-to-end against the real UsdImagingCreateSceneIndices chain. Not a unit
# test: it needs the RigExec scene index plugin discoverable through Plug, so
# it runs from run_probe-style environments rather than bare ctest.
add_executable(probeImagingPipeline tests/probeImagingPipeline.cpp)
target_link_libraries(probeImagingPipeline PRIVATE rigExecImaging usdImaging usdGeom)
