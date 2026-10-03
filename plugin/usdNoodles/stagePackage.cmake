# Mirrors the UsdNoodles package from SOURCE_DIR into PACKAGE_DIR, the
# directory _usdNoodles is built into, so that one directory is the whole
# importable package.
#
#   cmake -DSOURCE_DIR=<dir> -DPACKAGE_DIR=<dir> -DMATCHING=<spec>
#         -DKEEP_PREFIX=<name> -P stagePackage.cmake
#
# MATCHING is the file(COPY) FILES_MATCHING spec that says what the package is
# made of; CMakeLists.txt hands the same spec to its install(), so the staged
# and the installed package cannot disagree. KEEP_PREFIX names the files the
# build itself puts at the top of PACKAGE_DIR -- the extension module -- which
# have no source counterpart and must survive the pruning below.
#
# A mirror rather than a copy: a file deleted or renamed in the source is
# deleted here too. A copy would leave the old module importable from the
# build tree, and the tests -- which run against this directory -- would go on
# passing against code that no longer exists.

foreach(_var SOURCE_DIR PACKAGE_DIR MATCHING KEEP_PREFIX)
    if (NOT DEFINED ${_var})
        message(FATAL_ERROR "stagePackage.cmake: ${_var} is not set")
    endif()
endforeach()

file(GLOB_RECURSE _staged RELATIVE "${PACKAGE_DIR}" "${PACKAGE_DIR}/*")
foreach(_file IN LISTS _staged)
    if (_file MATCHES "(^|/)__pycache__/" OR _file MATCHES "^${KEEP_PREFIX}\\.")
        continue()
    endif()
    if (NOT EXISTS "${SOURCE_DIR}/${_file}")
        file(REMOVE "${PACKAGE_DIR}/${_file}")
    endif()
endforeach()

# file(COPY) skips a file whose destination has the same timestamp, so an
# unchanged package costs a directory walk, not a rewrite.
file(COPY "${SOURCE_DIR}/" DESTINATION "${PACKAGE_DIR}"
    FILES_MATCHING ${MATCHING})
