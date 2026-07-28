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
    libs/rigExecMath/geometryKernels.cpp
    libs/rigExecMath/simdKernels.cpp
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
    libs/rigExec/computations.cpp
    libs/rigExec/moverKernels.cpp
    libs/rigExec/moverGraph.cpp
    libs/rigExec/tapSet.cpp
    libs/rigExec/rigEvaluator.cpp
)
target_include_directories(rigExec PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
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
target_include_directories(rigExecImaging PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs>
    $<INSTALL_INTERFACE:${RIGEXEC_INSTALL_INCLUDEDIR}>)
target_link_libraries(rigExecImaging PUBLIC
    rigExec
    hd
    usdImaging
    usdUtils
)
# Selects dllexport over dllimport in registry.h's C surface while building
# this library; consumers including the installed header get dllimport.
target_compile_definitions(rigExecImaging PRIVATE RIGEXEC_IMAGING_EXPORTS)

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

# The engine authors nothing: asserted by whole-scene equality rather than by
# probing for the specific artifacts the deleted compiler used to leave.
add_executable(testRigExecNoAuthoring tests/testRigExecNoAuthoring.cpp)
target_link_libraries(testRigExecNoAuthoring PRIVATE rigExecImaging)
add_test(NAME testRigExecNoAuthoring COMMAND testRigExecNoAuthoring
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(probeCodingError tests/probeCodingError.cpp)
target_link_libraries(probeCodingError PRIVATE rigExec)

# End-to-end against the real UsdImagingCreateSceneIndices chain. Not a unit
# test: it needs the RigExec scene index plugin discoverable through Plug, so
# it runs from run_probe-style environments rather than bare ctest.
add_executable(probeImagingPipeline tests/probeImagingPipeline.cpp)
target_link_libraries(probeImagingPipeline PRIVATE rigExecImaging usdImaging usdGeom)

endif()  # RIGEXEC_BUILD_TESTS

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
include(CMakePackageConfigHelpers)

install(TARGETS rigExecMath rigExec rigExecImaging
    EXPORT rigExecTargets
    RUNTIME DESTINATION "${RIGEXEC_INSTALL_LIBDIR}"
    LIBRARY DESTINATION "${RIGEXEC_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION "${RIGEXEC_INSTALL_LIBDIR}")

# FILES_MATCHING still walks -- and recreates -- every subdirectory it passes
# through, so dot-directories are excluded explicitly. Without this, local
# assistant state under libs/rigExec/.omc reappears as empty directories in
# the install tree, session ids and all.
foreach(_lib rigExecMath rigExec rigExecImaging)
    install(DIRECTORY "libs/${_lib}/"
        DESTINATION "${RIGEXEC_INSTALL_INCLUDEDIR}/${_lib}"
        FILES_MATCHING
            PATTERN ".*" EXCLUDE
            PATTERN "__pycache__" EXCLUDE
            PATTERN "*.h")
endforeach()

# The codeless schema domain and the usdview plugin are location-independent:
# their plugInfo.json carries no LibraryPath, so both install verbatim. Files
# are listed explicitly rather than globbed so that local tooling state
# (.omc/, __pycache__/) cannot follow them into the install tree.
install(FILES
        plugin/rigExecSchema/resources/plugInfo.json
        plugin/rigExecSchema/resources/generatedSchema.usda
    DESTINATION "${RIGEXEC_INSTALL_PLUGINDIR}/rigExecSchema/resources")

install(FILES
        plugin/rigExecUsdview/plugInfo.json
        plugin/rigExecUsdview/rigExecUsdview.py
    DESTINATION "${RIGEXEC_INSTALL_PYTHONDIR}/rigExecUsdview")

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
