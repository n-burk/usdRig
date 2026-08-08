#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rigExec::rigExecMath" for configuration "Release"
set_property(TARGET rigExec::rigExecMath APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rigExec::rigExecMath PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/rigExecMath.lib"
  )

list(APPEND _cmake_import_check_targets rigExec::rigExecMath )
list(APPEND _cmake_import_check_files_for_rigExec::rigExecMath "${_IMPORT_PREFIX}/lib/rigExecMath.lib" )

# Import target "rigExec::rigExec" for configuration "Release"
set_property(TARGET rigExec::rigExec APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rigExec::rigExec PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/rigExec.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/rigExec.dll"
  )

list(APPEND _cmake_import_check_targets rigExec::rigExec )
list(APPEND _cmake_import_check_files_for_rigExec::rigExec "${_IMPORT_PREFIX}/lib/rigExec.lib" "${_IMPORT_PREFIX}/lib/rigExec.dll" )

# Import target "rigExec::rigExecImaging" for configuration "Release"
set_property(TARGET rigExec::rigExecImaging APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rigExec::rigExecImaging PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/rigExecImaging.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/rigExecImaging.dll"
  )

list(APPEND _cmake_import_check_targets rigExec::rigExecImaging )
list(APPEND _cmake_import_check_files_for_rigExec::rigExecImaging "${_IMPORT_PREFIX}/lib/rigExecImaging.lib" "${_IMPORT_PREFIX}/lib/rigExecImaging.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
