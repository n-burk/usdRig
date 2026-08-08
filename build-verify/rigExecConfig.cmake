
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was rigExecConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

# Resolve our own paths BEFORE looking up any dependency. 
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was rigExecConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

#################################################################################### sets
# PACKAGE_PREFIX_DIR, and a nested config package's own 
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was rigExecConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

#################################################################################### can
# overwrite it (CMake <= 3.29 does not save and restore it around
# find_dependency) -- which would silently re-root these three at the USD
# prefix instead of ours.
set_and_check(rigExec_PLUGIN_DIR "${PACKAGE_PREFIX_DIR}/lib/usd")
set_and_check(rigExec_PYTHON_DIR "${PACKAGE_PREFIX_DIR}/lib/python")
set_and_check(rigExec_LIBRARY_DIR "${PACKAGE_PREFIX_DIR}/lib")

# Plugin locations, so a consuming build can compose PXR_PLUGINPATH_NAME
# without hardcoding the layout.
set(rigExec_PLUGINPATHS
    "${rigExec_PLUGIN_DIR}/rigExecSchema/resources"
    "${rigExec_PLUGIN_DIR}/rigExecImaging/resources"
    "${rigExec_PYTHON_DIR}/rigExecUsdview")

# RigExec's public link interface exposes OpenUSD imported targets (usd, sdf,
# exec, vdf, ...), so consumers need the same pxr package this was built
# against. The build-time prefix is recorded as a default; override it with
# rigExec_USD_INSTALL_DIR or the usual CMAKE_PREFIX_PATH / pxr_DIR.
set(rigExec_USD_INSTALL_DIR "D:/work/usdRig/usdRig/../usd-install"
    CACHE PATH "OpenUSD prefix RigExec was built against")

include(CMakeFindDependencyMacro)

# pxrConfig.cmake resolves its own transitive config packages (OpenSubdiv, and
# MaterialX where that build enabled it) with a bare find_dependency, which
# does not inherit the PATHS given here -- so the USD prefix has to be on
# CMAKE_PREFIX_PATH or those lookups fail even though pxr itself is found.
#
# Scoped in a block() so the append cannot leak into the consumer: on failure
# find_dependency() calls return(), which would skip a plain restore. block()
# needs CMake 3.25, hence the fallback for older consumers -- there the leak
# only occurs on the already-fatal not-found path.
if (COMMAND block)
    block(SCOPE_FOR VARIABLES PROPAGATE pxr_FOUND)
        list(APPEND CMAKE_PREFIX_PATH "${rigExec_USD_INSTALL_DIR}")
        find_dependency(pxr CONFIG PATHS "${rigExec_USD_INSTALL_DIR}")
    endblock()
else()
    set(_rigExec_saved_prefix_path "${CMAKE_PREFIX_PATH}")
    list(APPEND CMAKE_PREFIX_PATH "${rigExec_USD_INSTALL_DIR}")
    find_dependency(pxr CONFIG PATHS "${rigExec_USD_INSTALL_DIR}")
    set(CMAKE_PREFIX_PATH "${_rigExec_saved_prefix_path}")
    unset(_rigExec_saved_prefix_path)
endif()

include("${CMAKE_CURRENT_LIST_DIR}/rigExecTargets.cmake")

check_required_components(rigExec)
