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
    libs/rigExec/curvenetWeightComputations.cpp
    libs/rigExec/curvenetAdjuster.cpp
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
# Tools
#
# rigExecPose evaluates an arbitrary rig stage and prints the result. Unlike
# the tests it asserts nothing about a fixture it owns, which is what makes it
# usable on an asset being authored -- see tools/rigExecPose.cpp.
# ---------------------------------------------------------------------------
add_executable(rigExecPose tools/rigExecPose.cpp)
target_link_libraries(rigExecPose PRIVATE rigExec)
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

add_executable(testRigExecConstraints tests/testRigExecConstraints.cpp)
target_link_libraries(testRigExecConstraints PRIVATE rigExec)
target_compile_definitions(testRigExecConstraints PRIVATE
    ${_rigexec_schema_resource_define})
add_test(NAME testRigExecConstraints COMMAND testRigExecConstraints)

add_executable(testRigExecStandalone tests/testRigExecStandalone.cpp)
target_link_libraries(testRigExecStandalone PRIVATE rigExecStandalone)
target_compile_definitions(testRigExecStandalone PRIVATE ${_rigexec_schema_resource_define})
add_test(NAME testRigExecStandalone COMMAND testRigExecStandalone)

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

add_executable(testRigExecWeightFields tests/testRigExecWeightFields.cpp)
target_link_libraries(testRigExecWeightFields PRIVATE rigExecMath)
add_test(NAME testRigExecWeightFields COMMAND testRigExecWeightFields)

add_executable(testRigExecArm tests/testRigExecArm.cpp)
target_link_libraries(testRigExecArm PRIVATE rigExec)
target_compile_definitions(testRigExecArm PRIVATE ${_rigexec_schema_resource_define})
add_test(NAME testRigExecArm COMMAND testRigExecArm
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecImaging tests/testRigExecImaging.cpp)
target_link_libraries(testRigExecImaging PRIVATE rigExecImaging)
target_compile_definitions(testRigExecImaging PRIVATE ${_rigexec_schema_resource_define})
add_test(NAME testRigExecImaging COMMAND testRigExecImaging
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecVolumeWeights tests/testRigExecVolumeWeights.cpp)
target_link_libraries(testRigExecVolumeWeights PRIVATE rigExec)
target_compile_definitions(testRigExecVolumeWeights PRIVATE ${_rigexec_schema_resource_define})
add_test(NAME testRigExecVolumeWeights COMMAND testRigExecVolumeWeights
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecWeightOverlay tests/testRigExecWeightOverlay.cpp)
target_link_libraries(testRigExecWeightOverlay PRIVATE rigExecImaging)
target_compile_definitions(testRigExecWeightOverlay PRIVATE ${_rigexec_schema_resource_define})
add_test(NAME testRigExecWeightOverlay COMMAND testRigExecWeightOverlay
    "${CMAKE_CURRENT_SOURCE_DIR}/examples")

add_executable(testRigExecMoverGraph tests/testRigExecMoverGraph.cpp)
target_link_libraries(testRigExecMoverGraph PRIVATE rigExec)
add_test(NAME testRigExecMoverGraph COMMAND testRigExecMoverGraph)

add_executable(testRigExecInteractive tests/testRigExecInteractive.cpp)
target_link_libraries(testRigExecInteractive PRIVATE rigExec)
target_compile_definitions(testRigExecInteractive PRIVATE
    ${_rigexec_schema_resource_define})
add_test(NAME testRigExecInteractive COMMAND testRigExecInteractive)

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
    add_test(NAME testRestLocal COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rest_local.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testRestMigration COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rest_migration.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testInterveningXform COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_intervening_xform.py"
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
                         testInterveningXform
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

install(TARGETS rigExecMath rigExec rigExecStandalone rigExecRigging rigExecImaging
    EXPORT rigExecTargets
    RUNTIME DESTINATION "${RIGEXEC_INSTALL_LIBDIR}"
    LIBRARY DESTINATION "${RIGEXEC_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION "${RIGEXEC_INSTALL_LIBDIR}")

# FILES_MATCHING still walks -- and recreates -- every subdirectory it passes
# through, so dot-directories and local tooling state are excluded explicitly.
foreach(_lib rigExecMath rigExec rigExecStandalone rigExecRigging rigExecImaging)
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
