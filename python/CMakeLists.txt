# ---------------------------------------------------------------------------
# _rigexec: pybind11 extension module + rigexec pure-Python package.
#
# The native module binds the spec section 4 authoring surface (Builder and
# its handles) and the staged evaluator, so a Python tool can create and wire
# an entire rig on a pxr.Usd.Stage and evaluate it in-process -- the same way
# other OpenUSD python bindings are used. The pxr modules of this USD build
# are Boost.Python; _rigexec is pybind11. Both coexist because they load the
# SAME USD shared libraries (Windows DLL caching), and a pxr object's C++
# owner is recovered through its "refptr" capsule -- see _rigexec.cpp.
# ---------------------------------------------------------------------------

find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)

# pybind11: an explicit -Dpybind11_DIR wins; otherwise ask the interpreter for
# the pip-installed package's CMake directory (the layout this repo builds
# against).
if (NOT pybind11_FOUND AND NOT pybind11_DIR)
    execute_process(
        COMMAND "${Python3_EXECUTABLE}" -m pybind11 --cmakedir
        OUTPUT_VARIABLE _rigexec_pybind11_dir
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE _rigexec_pybind11_result)
    if (NOT _rigexec_pybind11_result EQUAL 0 OR NOT EXISTS "${_rigexec_pybind11_dir}")
        message(FATAL_ERROR
            "pybind11 not found. Install it into the target interpreter "
            "(pip install pybind11) or pass -Dpybind11_DIR=<dir>.")
    endif()
    list(APPEND CMAKE_PREFIX_PATH "${_rigexec_pybind11_dir}")
endif()
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(_rigexec _rigexec.cpp)
target_link_libraries(_rigexec PRIVATE
    rigExecRigging
    rigExec
    usd sdf tf gf vt)

# The installed extension lands in lib/python while rigExec/rigExecRigging land
# in lib, so the loader-relative rpath inherited from CMAKE_INSTALL_RPATH
# ($ORIGIN / @loader_path) needs one extra hop up to find its siblings.
if (APPLE)
    set_property(TARGET _rigexec APPEND PROPERTY INSTALL_RPATH "@loader_path/../")
elseif (UNIX)
    set_property(TARGET _rigexec APPEND PROPERTY INSTALL_RPATH "$ORIGIN/..")
endif()

# Keep the pure-Python package importable from ONE build-tree PYTHONPATH entry:
# copy it next to the extension module. A standalone target (not POST_BUILD)
# so editing a .py file re-copies even when no C++ target relinks. The
# installed layout is handled by install() below, where both land in
# RIGEXEC_INSTALL_PYTHONDIR.
add_custom_target(rigexec_python_package ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/rigexec"
        "$<TARGET_FILE_DIR:_rigexec>/rigexec"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/rigexec/__init__.py")

install(TARGETS _rigexec LIBRARY DESTINATION "${RIGEXEC_INSTALL_PYTHONDIR}")
# No trailing slash: install the rigexec/ directory itself so the package
# lands in lib/python/rigexec/, not its contents directly in lib/python/.
install(DIRECTORY "rigexec"
    DESTINATION "${RIGEXEC_INSTALL_PYTHONDIR}"
    FILES_MATCHING PATTERN "*.py")
