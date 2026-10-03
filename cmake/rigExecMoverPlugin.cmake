include_guard(GLOBAL)
include(CMakeParseArguments)

# Available both inside the usdRig build and from find_package(rigExec).
function(rigexec_add_mover_plugin name)
    cmake_parse_arguments(P "NO_INSTALL" "PLUGIN_ROOT" "SOURCES;LIBRARIES" ${ARGN})
    if (P_UNPARSED_ARGUMENTS OR P_KEYWORDS_MISSING_VALUES OR NOT P_SOURCES)
        message(FATAL_ERROR
            "rigexec_add_mover_plugin(${name}) requires SOURCES; optional LIBRARIES, PLUGIN_ROOT and NO_INSTALL")
    endif()
    if (NOT name MATCHES "^[A-Za-z_][A-Za-z0-9_]*$")
        message(FATAL_ERROR "Mover plugin names must be C/C++ identifiers: ${name}")
    endif()
    add_library(${name} SHARED ${P_SOURCES})
    target_compile_features(${name} PRIVATE cxx_std_17)
    target_link_libraries(${name} PRIVATE rigExec::rigExec ${P_LIBRARIES})
    if (MSVC)
        target_compile_options(${name} PRIVATE /permissive- /Zc:__cplusplus /bigobj)
        target_compile_definitions(${name} PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
    else()
        target_compile_options(${name} PRIVATE -ffp-contract=off)
    endif()

    if (P_PLUGIN_ROOT)
        get_filename_component(_plugin_root "${P_PLUGIN_ROOT}" ABSOLUTE
            BASE_DIR "${CMAKE_CURRENT_BINARY_DIR}")
    else()
        set(_plugin_root "${CMAKE_BINARY_DIR}/usd/rigExecMoverPlugins")
    endif()
    set(_resources "${_plugin_root}/${name}/$<CONFIG>/resources")
    set_property(TARGET ${name} PROPERTY RIGEXEC_PLUGIN_RESOURCE_DIR "${_resources}")
    file(WRITE "${_plugin_root}/plugInfo.json"
        "{\"Includes\": [\"*/resources/\", \"*/*/resources/\"]}\n")
    set(_metadata "{\"Plugins\": [{\"Type\": \"library\", \"Name\": \"${name}\", \"Root\": \".\", \"ResourcePath\": \".\", \"LibraryPath\": \"@LIBRARY@\", \"Info\": {\"RigExecMoverPlugin\": 1}}]}\n")
    string(REPLACE "@LIBRARY@" "$<TARGET_FILE:${name}>" _build_metadata "${_metadata}")
    file(GENERATE OUTPUT "${_resources}/plugInfo.json" CONTENT "${_build_metadata}")

    if (NOT P_NO_INSTALL)
        if (NOT DEFINED RIGEXEC_INSTALL_LIBDIR)
            set(RIGEXEC_INSTALL_LIBDIR "lib")
        endif()
        if (NOT DEFINED RIGEXEC_INSTALL_PLUGINDIR)
            set(RIGEXEC_INSTALL_PLUGINDIR "lib/usd")
        endif()
        set(_install_resources "${RIGEXEC_INSTALL_PLUGINDIR}/rigExecMoverPlugins/${name}/resources")
        file(RELATIVE_PATH _relative_lib
            "${CMAKE_INSTALL_PREFIX}/${_install_resources}"
            "${CMAKE_INSTALL_PREFIX}/${RIGEXEC_INSTALL_LIBDIR}")
        string(REPLACE "@LIBRARY@" "${_relative_lib}/$<TARGET_FILE_NAME:${name}>"
            _install_metadata "${_metadata}")
        file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${name}-install/$<CONFIG>/plugInfo.json"
            CONTENT "${_install_metadata}")
        if (APPLE)
            set_target_properties(${name} PROPERTIES INSTALL_RPATH "@loader_path;${rigExec_LIBRARY_DIR}")
        elseif (UNIX)
            set_target_properties(${name} PROPERTIES INSTALL_RPATH "$ORIGIN;${rigExec_LIBRARY_DIR}")
        endif()
        install(TARGETS ${name}
            RUNTIME DESTINATION "${RIGEXEC_INSTALL_LIBDIR}"
            LIBRARY DESTINATION "${RIGEXEC_INSTALL_LIBDIR}"
            ARCHIVE DESTINATION "${RIGEXEC_INSTALL_LIBDIR}")
        install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${name}-install/$<CONFIG>/plugInfo.json"
            DESTINATION "${_install_resources}")
        install(FILES "${_plugin_root}/plugInfo.json"
            DESTINATION "${RIGEXEC_INSTALL_PLUGINDIR}/rigExecMoverPlugins")
    endif()
endfunction()
