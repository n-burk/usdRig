# Investigation probe. Not part of the product build.
# Builds the USD-free runtime from the existing sources and two
# microbenchmarks. Does not modify those sources.
cmake_minimum_required(VERSION 3.26)
project(rigexec_native_eval_probe LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

get_filename_component(RIGEXEC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

add_library(probe_lzma STATIC
    ${RIGEXEC_ROOT}/third_party/lzma/C/LzmaEnc.c
    ${RIGEXEC_ROOT}/third_party/lzma/C/LzmaDec.c
    ${RIGEXEC_ROOT}/third_party/lzma/C/LzFind.c
    ${RIGEXEC_ROOT}/third_party/lzma/C/CpuArch.c)
target_include_directories(probe_lzma PRIVATE ${RIGEXEC_ROOT}/third_party/lzma/C)
target_compile_definitions(probe_lzma PRIVATE Z7_ST RIGEXEC_LZMA_PORTABLE_SCALAR)

add_library(probe_binary STATIC
    ${RIGEXEC_ROOT}/libs/rigExecBinary/format.cpp
    ${RIGEXEC_ROOT}/libs/rigExecBinary/transport.cpp)
target_include_directories(probe_binary PUBLIC ${RIGEXEC_ROOT}/libs)
target_include_directories(probe_binary PRIVATE ${RIGEXEC_ROOT}/third_party/lzma/C)
target_include_directories(probe_binary PUBLIC
    ${RIGEXEC_ROOT}/thirdparty/flatbuffers/include)
target_link_libraries(probe_binary PUBLIC probe_lzma)

add_library(probe_graph STATIC ${RIGEXEC_ROOT}/libs/rigExecGraph/opGraph.cpp)
target_include_directories(probe_graph PUBLIC ${RIGEXEC_ROOT}/libs)

add_library(probe_runtime STATIC
    ${RIGEXEC_ROOT}/libs/rigExecRuntime/open.cpp
    ${RIGEXEC_ROOT}/libs/rigExecRuntime/exec.cpp
    ${RIGEXEC_ROOT}/libs/rigExecRuntime/closure.cpp
    ${RIGEXEC_ROOT}/libs/rigExecRuntime/publish.cpp
    ${RIGEXEC_ROOT}/libs/rigExecRuntime/kernels.cpp
    ${RIGEXEC_ROOT}/libs/rigExecRuntime/pose.cpp
    ${RIGEXEC_ROOT}/libs/rigExecRuntime/poseConstraints.cpp
    ${RIGEXEC_ROOT}/libs/rigExecRuntime/poseInterpolation.cpp
    ${RIGEXEC_ROOT}/libs/rigExecRuntime/poseMath.cpp
    ${RIGEXEC_ROOT}/libs/rigExecRuntime/poseSolvers.cpp
    ${RIGEXEC_ROOT}/libs/rigExecRuntime/poseSteps.cpp
    ${RIGEXEC_ROOT}/libs/rigExecRuntime/geometry.cpp
    ${RIGEXEC_ROOT}/libs/rigExecRuntime/weights.cpp
    ${RIGEXEC_ROOT}/libs/rigExecRuntime/inputs.cpp
    ${RIGEXEC_ROOT}/libs/rigExecRuntime/labels.cpp
    ${RIGEXEC_ROOT}/libs/rigExecRuntime/properties.cpp
    ${RIGEXEC_ROOT}/libs/rigExecRuntime/spaces.cpp)
target_include_directories(probe_runtime PUBLIC ${RIGEXEC_ROOT}/libs)
target_link_libraries(probe_runtime PUBLIC probe_binary probe_graph)
if(NOT MSVC)
    target_compile_options(probe_runtime PRIVATE -fno-fast-math -ffp-contract=off)
    target_compile_options(probe_graph PRIVATE -fno-fast-math -ffp-contract=off)
    target_compile_options(probe_binary PRIVATE -fno-fast-math -ffp-contract=off)
endif()

function(probe_exe name)
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} PRIVATE ${ARGN})
    if(NOT MSVC)
        target_compile_options(${name} PRIVATE -fno-fast-math -ffp-contract=off)
    endif()
    target_compile_definitions(${name} PRIVATE RIGEXEC_ROOT="${RIGEXEC_ROOT}")
endfunction()

probe_exe(probe_program probe_runtime)
probe_exe(bench_schedule probe_graph)
target_link_libraries(bench_schedule PRIVATE pthread)
probe_exe(bench_skin)
target_link_libraries(bench_skin PRIVATE pthread ${CMAKE_DL_LIBS})
