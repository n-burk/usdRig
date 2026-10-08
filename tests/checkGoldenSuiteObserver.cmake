# Supported protocol regression. DRIVER is the observer-only fixture binary;
# BINARY_DIR is writable test output. No repository judge is created or edited.
if(NOT DEFINED DRIVER OR NOT DEFINED BINARY_DIR)
    message(FATAL_ERROR "DRIVER and BINARY_DIR are required")
endif()
string(RANDOM LENGTH 16 ALPHABET 0123456789abcdef _suffix)
set(_directory "${BINARY_DIR}/golden-suite-observer-${_suffix}")
set(_raw 0)
function(run_observer mode scenario should_pass)
    execute_process(COMMAND "${CMAKE_COMMAND}" -E env
        "RIGEXEC_GOLDEN_SUITE=${mode}:${_directory}"
        "RIGEXEC_GOLDEN_SUITE_NAME=protocol"
        "RIGEXEC_GOLDEN_SUITE_RAW=${_raw}"
        "${DRIVER}" "${scenario}"
        RESULT_VARIABLE _result OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
    if(should_pass AND NOT _result STREQUAL "0")
        message(FATAL_ERROR "${mode}/${scenario} failed: ${_result}\n${_out}\n${_err}")
    elseif(NOT should_pass AND _result STREQUAL "0")
        message(FATAL_ERROR "${mode}/${scenario} incorrectly accepted a changed capture")
    elseif(NOT should_pass AND NOT _err MATCHES "golden suite")
        message(FATAL_ERROR "${mode}/${scenario} failed without an observer verdict: ${_err}")
    endif()
endfunction()
run_observer(capture baseline TRUE)
run_observer(check baseline TRUE)
file(READ "${_directory}/protocol/0.golden" _digest)
if(NOT _digest MATCHES "encoding raw-bits domain-digests" OR
        NOT _digest MATCHES "h movedProperties 1 " OR
        _digest MATCHES "v movedProperties ")
    message(FATAL_ERROR "default mode no longer emits the established domain digest")
endif()
run_observer(capture baseline FALSE)
foreach(_scenario empty missing-evaluator extra-evaluator missing-generation
        extra-generation time rig bits unclosed)
    run_observer(check "${_scenario}" FALSE)
endforeach()
file(WRITE "${_directory}/protocol/99.golden" "unexpected capture\n")
run_observer(check baseline FALSE)
file(RENAME "${_directory}/protocol/99.golden" "${_directory}/protocol/99.extra")
file(RENAME "${_directory}/protocol/index.golden" "${_directory}/protocol/index.missing")
run_observer(check baseline FALSE)
file(RENAME "${_directory}/protocol/index.missing" "${_directory}/protocol/index.golden")
file(RENAME "${_directory}/protocol/1.golden" "${_directory}/protocol/1.missing")
run_observer(check baseline FALSE)

# Full raw observations retain the same strict owner/generation inventory.
set(_directory "${BINARY_DIR}/golden-suite-observer-raw-${_suffix}")
set(_raw 1)
run_observer(capture baseline TRUE)
run_observer(check baseline TRUE)
file(READ "${_directory}/protocol/0.golden" _raw_capture)
string(FIND "${_raw_capture}" "v movedProperties \"/A.value\" double 8000000000000000" _typed_row)
if(NOT _raw_capture MATCHES "encoding raw-bits full-values" OR
        _typed_row EQUAL -1 OR _raw_capture MATCHES "h movedProperties ")
    message(FATAL_ERROR "raw mode omitted the exact typed signed-zero payload")
endif()
file(SHA256 "${_directory}/protocol/index.golden" _index_before)
string(REPLACE "double 8000000000000000" "double 0000000000000000"
    _tampered "${_raw_capture}")
file(WRITE "${_directory}/protocol/0.golden" "${_tampered}")
run_observer(check baseline FALSE)
file(SHA256 "${_directory}/protocol/index.golden" _index_after)
if(NOT _index_before STREQUAL _index_after)
    message(FATAL_ERROR "payload-only mutation changed the inventory")
endif()
