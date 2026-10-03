cmake_minimum_required(VERSION 3.26)
project(RigExecExternalMoverFixture LANGUAGES CXX)

# This directory is also a standalone SDK consumer for installed-build tests.
if(NOT TARGET rigExec::rigExec)
    find_package(rigExec CONFIG REQUIRED)
endif()

rigexec_add_mover_plugin(testExternalMover NO_INSTALL
    PLUGIN_ROOT "${CMAKE_BINARY_DIR}/test-plugins"
    SOURCES mover.cpp)
