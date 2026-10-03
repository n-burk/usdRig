
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

set(_usdnoodles_package_dir "${CMAKE_BINARY_DIR}/python/UsdNoodles")
set(_usdnoodles_install_dir "${RIGEXEC_INSTALL_PYTHONDIR}/UsdNoodles")

set(NOODLES_ROOT "${USD_INSTALL_DIR}" CACHE PATH
    "Prefix holding include/noodles/ and the noodles library; fetched when absent")
find_path(NOODLES_INCLUDE_DIR core/api.h
    PATHS "${NOODLES_ROOT}/include/noodles" NO_DEFAULT_PATH)
find_library(NOODLES_LIBRARY noodles
    PATHS "${NOODLES_ROOT}/lib" NO_DEFAULT_PATH)
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
    set(CMAKE_DISABLE_FIND_PACKAGE_GTest ON)
    FetchContent_MakeAvailable(noodles)
endif()

set(_usdnoodles_definitions
    NOODLES_EXPORTS
    MFB_PACKAGE_NAME=usdNoodles
    MFB_ALT_PACKAGE_NAME=usdNoodles
    MFB_PACKAGE_MODULE=UsdNoodles)

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
set_target_properties(_usdNoodles PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY "${_usdnoodles_package_dir}")
if (WIN32)
    set_target_properties(_usdNoodles PROPERTIES SUFFIX ".pyd")
elseif (APPLE)
    set_target_properties(_usdNoodles PROPERTIES SUFFIX ".so")
endif()
# Installed in lib/python/UsdNoodles while usdNoodles lands in lib, so the
if (APPLE)
    set_property(TARGET _usdNoodles APPEND PROPERTY INSTALL_RPATH "@loader_path/../..")
elseif (UNIX)
    set_property(TARGET _usdNoodles APPEND PROPERTY INSTALL_RPATH "$ORIGIN/../..")
endif()

set(_usdnoodles_matching
    PATTERN ".*" EXCLUDE
    PATTERN "__pycache__" EXCLUDE
    PATTERN "bindings" EXCLUDE
    PATTERN "usd" EXCLUDE
    PATTERN "*.py"
    PATTERN "*.json"
    PATTERN "*.png"
    PATTERN "*.glsl"
    PATTERN "LICENSE.txt"
    PATTERN "NOODLES_LICENSE.txt"
    PATTERN "THIRD_PARTY_LICENSES.txt")

# A standalone target (not POST_BUILD) so editing a .py file restages it even
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
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/"
    DESTINATION "${_usdnoodles_install_dir}"
    FILES_MATCHING ${_usdnoodles_matching})
