# ---------------------------------------------------------------------------
# usdNoodles: the Noodles node-graph editor for usdview.
#
# Three parts: the usdNoodles library (UsdPrimRegistry, the schema discovery
# the graph's USD-prim nodes are built from), the _usdNoodles Boost.Python
# module that wraps it together with the noodles core renderer, and the
# UsdNoodles Python package -- the editor itself, its assets, its plugInfo.json
# and its tests.
#
# Localized from OpenUSD PR #4156 (pxr/usdImaging/usdNoodles), where
# pxr_library() built it into the USD install as pxr.UsdNoodles. Here it is
# built against the installed OpenUSD like the rest of this repository, and
# that is why the package is the top-level UsdNoodles: pxr/__init__.py is a
# plain package, not a namespace one, so a module under pxr can only come from
# the USD install's own pxr directory.
# ---------------------------------------------------------------------------

# Every binary here lands beside the RigExec libraries, which is the one build
# directory the helpers put on PATH -- all Windows has for finding a DLL.
# _usdNoodles overrides this below: a Python module has to sit in its package.
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

set(_usdnoodles_package_dir "${CMAKE_BINARY_DIR}/python/UsdNoodles")
set(_usdnoodles_install_dir "${RIGEXEC_INSTALL_PYTHONDIR}/UsdNoodles")

# ---------------------------------------------------------------------------
# The noodles core library (github.com/facebookexperimental/noodles) does the
# layout, the link geometry and the GL drawing. The bindings/wrap*.cpp files
# are coupled to its API, so it is pinned to the commit PR #4156's
# build_usd.py pins.
#
# A prefix that already has it is used as is -- build_usd.py --build-noodles
# installs it into the USD prefix, which is why that is where NOODLES_ROOT
# looks by default. Otherwise it is fetched and built here, so a stock OpenUSD
# install needs nothing extra.
# ---------------------------------------------------------------------------
set(NOODLES_ROOT "${USD_INSTALL_DIR}" CACHE PATH
    "Prefix holding include/noodles/ and the noodles library; fetched when absent")
find_path(NOODLES_INCLUDE_DIR core/api.h
    PATHS "${NOODLES_ROOT}/include/noodles" NO_DEFAULT_PATH)
find_library(NOODLES_LIBRARY noodles
    PATHS "${NOODLES_ROOT}/lib" NO_DEFAULT_PATH)
# The noodles render headers include <GL/glew.h>. noodles installs GLEW's
# headers beside its own when it built GLEW itself, and uses a system GLEW
# otherwise, so the prefix is searched first and the system after it.
find_path(NOODLES_GLEW_INCLUDE_DIR GL/glew.h
    PATHS "${NOODLES_ROOT}/include")

if (NOODLES_INCLUDE_DIR AND NOODLES_LIBRARY AND NOODLES_GLEW_INCLUDE_DIR)
    message(STATUS "usdNoodles: using noodles from ${NOODLES_LIBRARY}")
    add_library(noodles UNKNOWN IMPORTED)
    set_target_properties(noodles PROPERTIES
        IMPORTED_LOCATION "${NOODLES_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES
            "${NOODLES_INCLUDE_DIR};${NOODLES_GLEW_INCLUDE_DIR}")
else()
    message(STATUS "usdNoodles: noodles not found under NOODLES_ROOT "
                   "(${NOODLES_ROOT}); fetching it")
    include(FetchContent)
    FetchContent_Declare(noodles
        URL "https://github.com/facebookexperimental/noodles/archive/ff5d473f10e8c37ceaf0da11ea7cb80805bc8314.zip"
        URL_HASH SHA256=2968168266009fa5e4f233bfa1432690b3c48196673654371162908ca401f671
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    # noodles builds its own GoogleTest suite whenever GTest is findable; that
    # suite belongs to noodles' CI, not to this build's ctest.
    set(CMAKE_DISABLE_FIND_PACKAGE_GTest ON)
    FetchContent_MakeAvailable(noodles)
endif()

# Both targets define NOODLES_EXPORTS, as PR #4156's build did: UsdPrimRegistry
# is declared with noodles' own NOODLES_API, so this is what exports it from
# the library on Windows. The MFB_* names are the ones pxr_library() defines
# for every library and Python module -- Tf's registry and TF_WRAP_MODULE are
# keyed on them.
set(_usdnoodles_definitions
    NOODLES_EXPORTS
    MFB_PACKAGE_NAME=usdNoodles
    MFB_ALT_PACKAGE_NAME=usdNoodles
    MFB_PACKAGE_MODULE=UsdNoodles)

# The target is not called usdNoodles because an OpenUSD built from PR #4156
# already exports an imported target by that name through pxrConfig. The file
# keeps the name; PR #4156's is usd_-prefixed, so the two cannot overwrite
# each other either.
add_library(usdNoodlesLib SHARED
    usd/UsdPrimRegistry.cpp
    moduleDeps.cpp)
set_target_properties(usdNoodlesLib PROPERTIES OUTPUT_NAME usdNoodles)
target_include_directories(usdNoodlesLib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
target_link_libraries(usdNoodlesLib PUBLIC
    noodles
    garch glf hio js tf gf vt sdf usd plug
    python)
target_compile_definitions(usdNoodlesLib PRIVATE ${_usdnoodles_definitions})

add_library(_usdNoodles MODULE
    bindings/module.cpp
    bindings/wrapGraphLayout.cpp
    bindings/wrapGraphRenderer.cpp
    bindings/wrapLinkGeometry.cpp
    bindings/wrapNodeData.cpp
    bindings/wrapRenderManagers.cpp
    bindings/wrapRendering.cpp
    bindings/wrapSpatial.cpp
    bindings/wrapTextLayout.cpp
    bindings/wrapUndo.cpp
    bindings/wrapUsdPrimRegistry.cpp
    bindings/wrapVertexCache.cpp)
target_link_libraries(_usdNoodles PRIVATE usdNoodlesLib)
target_compile_definitions(_usdNoodles PRIVATE ${_usdnoodles_definitions})
# Python names the file after the module and nothing else: no lib prefix, and
# .pyd on Windows. macOS takes .so, as OpenUSD's own modules do, rather than
# CMake's .dylib-flavoured MODULE default.
set_target_properties(_usdNoodles PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY "${_usdnoodles_package_dir}")
if (WIN32)
    set_target_properties(_usdNoodles PROPERTIES SUFFIX ".pyd")
elseif (APPLE)
    set_target_properties(_usdNoodles PROPERTIES SUFFIX ".so")
endif()
# Installed in lib/python/UsdNoodles while usdNoodles lands in lib, so the
# loader-relative rpath needs two hops up to find it.
if (APPLE)
    set_property(TARGET _usdNoodles APPEND PROPERTY INSTALL_RPATH "@loader_path/../..")
elseif (UNIX)
    set_property(TARGET _usdNoodles APPEND PROPERTY INSTALL_RPATH "$ORIGIN/../..")
endif()

# ---------------------------------------------------------------------------
# The package: every module, asset and test, staged beside _usdNoodles so the
# build tree has one importable UsdNoodles (PYTHONPATH=build/python) whose
# plugInfo.json can be handed to PXR_PLUGINPATH_NAME as is.
#
# A directory, not a list of files. PR #4156 kept a list, and it had already
# drifted: testUsdNoodlesSaveLoadRoundTrip.py was in the source and not in it,
# so no installed tree ever carried that test. The C++ under bindings/ and
# usd/ is compiled, not shipped, and is left out by name so the package does
# not carry empty directories for it.
# ---------------------------------------------------------------------------
set(_usdnoodles_matching
    PATTERN ".*" EXCLUDE
    PATTERN "__pycache__" EXCLUDE
    PATTERN "bindings" EXCLUDE
    PATTERN "usd" EXCLUDE
    PATTERN "*.py"
    PATTERN "*.json"
    PATTERN "*.png"
    PATTERN "*.glsl"
    PATTERN "LICENSE.txt")

# A standalone target (not POST_BUILD) so editing a .py file restages it even
# when nothing relinks.
add_custom_target(usdNoodles_python_package ALL
    COMMAND "${CMAKE_COMMAND}"
        "-DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
        "-DPACKAGE_DIR=${_usdnoodles_package_dir}"
        "-DMATCHING=${_usdnoodles_matching}"
        "-DKEEP_PREFIX=_usdNoodles"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/stagePackage.cmake"
    COMMENT "Staging the UsdNoodles package"
    VERBATIM)

install(TARGETS usdNoodlesLib
    RUNTIME DESTINATION "${RIGEXEC_INSTALL_LIBDIR}"
    LIBRARY DESTINATION "${RIGEXEC_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION "${RIGEXEC_INSTALL_LIBDIR}")
install(TARGETS _usdNoodles
    LIBRARY DESTINATION "${_usdnoodles_install_dir}")
# The source directory by its full path, never "./": install() treats a
# relative "." as a directory named ".", which the ".*" exclusion then drops
# together with everything in it.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/"
    DESTINATION "${_usdnoodles_install_dir}"
    FILES_MATCHING ${_usdnoodles_matching})
