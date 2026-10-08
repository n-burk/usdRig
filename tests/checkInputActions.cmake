if(NOT DEFINED DRIVER OR NOT DEFINED BINARY_DIR OR NOT DEFINED SCHEMA)
    message(FATAL_ERROR "DRIVER, BINARY_DIR and SCHEMA are required")
endif()
string(RANDOM LENGTH 12 ALPHABET 0123456789abcdef token)
set(directory "${BINARY_DIR}/input-actions-${token}")
file(MAKE_DIRECTORY "${directory}")
set(provenance "${directory}/source.json")
file(WRITE "${provenance}" "{\"purpose\":\"input protocol regression only; no numerical judge\"}\n")
set(transcript "${directory}/fixture.actions")
execute_process(COMMAND "${CMAKE_COMMAND}" -E env
    "RIGEXEC_INPUT_REPLAY=capture:${transcript}" "RIGEXEC_INPUT_REPLAY_PROVENANCE=${provenance}"
    "RIGEXEC_GOLDEN_SUITE=" "RIGEXEC_INPUT_REPLAY_NATIVE_HISTORIES=" "${DRIVER}" fixture normal "${SCHEMA}"
    RESULT_VARIABLE status ERROR_VARIABLE error)
if(NOT status STREQUAL "0")
    message(FATAL_ERROR "Source fixture capture refused: ${error}")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" -E env "RIGEXEC_INPUT_REPLAY=" "RIGEXEC_GOLDEN_SUITE="
    "${DRIVER}" replay "${transcript}" "${SCHEMA}" RESULT_VARIABLE status ERROR_VARIABLE error)
if(NOT status STREQUAL "0")
    message(FATAL_ERROR "Exact source history replay refused: ${error}")
endif()
# Known import diff enumeration is unordered; its payload remains exact.
set(reordered "${directory}/notice-order.actions")
execute_process(COMMAND "${DRIVER}" mutate "${transcript}" "${reordered}" notice-order RESULT_VARIABLE status)
if(NOT status STREQUAL "0")
    message(FATAL_ERROR "Cannot permute known-import spec diff enumeration")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" -E env "RIGEXEC_INPUT_REPLAY=" "RIGEXEC_GOLDEN_SUITE="
    "${DRIVER}" replay "${reordered}" "${SCHEMA}" RESULT_VARIABLE status ERROR_VARIABLE error)
if(NOT status STREQUAL "0")
    message(FATAL_ERROR "Equivalent known-import spec diff enumeration was refused: ${error}")
endif()
foreach(kind IN ITEMS truncate extra sequence notice notice-value notice-path notice-field notice-boundary)
    set(damaged "${directory}/${kind}.actions")
    execute_process(COMMAND "${DRIVER}" mutate "${transcript}" "${damaged}" "${kind}"
        RESULT_VARIABLE status)
    if(NOT status STREQUAL "0")
        message(FATAL_ERROR "Cannot prepare ${kind} protocol regression")
    endif()
    execute_process(COMMAND "${CMAKE_COMMAND}" -E env "RIGEXEC_INPUT_REPLAY=" "RIGEXEC_GOLDEN_SUITE="
        "${DRIVER}" replay "${damaged}" "${SCHEMA}" RESULT_VARIABLE status ERROR_VARIABLE error)
    if(status STREQUAL "0" OR error STREQUAL "")
        message(FATAL_ERROR "Damaged ${kind} transcript was not refused with a reason")
    endif()
    if(kind MATCHES "^notice" AND NOT error MATCHES "source (notice batch audit differs|batch operation count/adapter differs)")
        message(FATAL_ERROR "Changed known-import notice content/boundary did not fail the exact notice audit: ${error}")
    endif()
endforeach()
foreach(kind IN ITEMS custom bulk bulk-clear bulk-transfer mixed-import during unclosed late population load mute freeze held-owner-close held-unclosed held-router-live)
    execute_process(COMMAND "${CMAKE_COMMAND}" -E env
        "RIGEXEC_INPUT_REPLAY=capture:${directory}/${kind}.actions" "RIGEXEC_INPUT_REPLAY_PROVENANCE=${provenance}"
        "RIGEXEC_GOLDEN_SUITE=" "RIGEXEC_INPUT_REPLAY_NATIVE_HISTORIES=" "${DRIVER}" fixture "${kind}" "${SCHEMA}"
        RESULT_VARIABLE status ERROR_VARIABLE error)
    if(status STREQUAL "0" OR NOT error MATCHES "input replay capture refused:")
        message(FATAL_ERROR "Unsupported/incomplete ${kind} input history was not explicitly refused: ${error}")
    endif()
endforeach()
set(comparison "${directory}/comparison.actions")
execute_process(COMMAND "${CMAKE_COMMAND}" -E env
    "RIGEXEC_INPUT_REPLAY=capture:${comparison}" "RIGEXEC_INPUT_REPLAY_PROVENANCE=${provenance}"
    "RIGEXEC_INPUT_REPLAY_NATIVE_HISTORIES=1" "RIGEXEC_GOLDEN_SUITE="
    "${DRIVER}" fixture comparison "${SCHEMA}" RESULT_VARIABLE status ERROR_VARIABLE error)
if(NOT status STREQUAL "0")
    message(FATAL_ERROR "Opt-in native comparison chronology refused: ${error}")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" -E env "RIGEXEC_INPUT_REPLAY=" "RIGEXEC_GOLDEN_SUITE="
    "${DRIVER}" replay "${comparison}" "${SCHEMA}" RESULT_VARIABLE status ERROR_VARIABLE error)
if(NOT status STREQUAL "0")
    message(FATAL_ERROR "Balanced native comparison chronology replay refused: ${error}")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" -E env
    "RIGEXEC_INPUT_REPLAY=capture:${directory}/nested.actions" "RIGEXEC_INPUT_REPLAY_PROVENANCE=${provenance}"
    "RIGEXEC_INPUT_REPLAY_NATIVE_HISTORIES=1" "RIGEXEC_GOLDEN_SUITE="
    "${DRIVER}" fixture nested "${SCHEMA}" RESULT_VARIABLE status ERROR_VARIABLE error)
if(status STREQUAL "0" OR NOT error MATCHES "input replay capture refused:")
    message(FATAL_ERROR "Native Evaluate inside comparison scope was not refused: ${error}")
endif()

# Exact Clear/Transfer caller adapters preserve shared session history.
set(bulk_adapters "${directory}/bulk-adapters.actions")
execute_process(COMMAND "${CMAKE_COMMAND}" -E env
    "RIGEXEC_INPUT_REPLAY=capture:${bulk_adapters}" "RIGEXEC_INPUT_REPLAY_PROVENANCE=${provenance}"
    "RIGEXEC_INPUT_REPLAY_NATIVE_HISTORIES=" "RIGEXEC_GOLDEN_SUITE="
    "${DRIVER}" fixture bulk-adapters "${SCHEMA}" RESULT_VARIABLE status ERROR_VARIABLE error)
if(NOT status STREQUAL "0")
    message(FATAL_ERROR "Exact Clear/Transfer source actions refused: ${error}")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" -E env "RIGEXEC_INPUT_REPLAY=" "RIGEXEC_GOLDEN_SUITE="
    "${DRIVER}" replay "${bulk_adapters}" "${SCHEMA}" RESULT_VARIABLE status ERROR_VARIABLE error)
if(NOT status STREQUAL "0")
    message(FATAL_ERROR "Exact Clear/Transfer source history replay refused: ${error}")
endif()

# Real held identity, repeated visit and explicit fresh-pose seed transport.
set(held_actions "${directory}/held.actions")
execute_process(COMMAND "${CMAKE_COMMAND}" -E env
    "RIGEXEC_INPUT_REPLAY=capture:${held_actions}" "RIGEXEC_INPUT_REPLAY_PROVENANCE=${provenance}"
    "RIGEXEC_GOLDEN_SUITE=" "RIGEXEC_INPUT_REPLAY_NATIVE_HISTORIES="
    "${DRIVER}" fixture held "${SCHEMA}" RESULT_VARIABLE status ERROR_VARIABLE error)
if(NOT status STREQUAL "0")
    message(FATAL_ERROR "Held public Build/Run capture refused: ${error}")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" -E env "RIGEXEC_INPUT_REPLAY=" "RIGEXEC_GOLDEN_SUITE="
    "${DRIVER}" replay "${held_actions}" "${SCHEMA}" RESULT_VARIABLE status ERROR_VARIABLE error)
if(NOT status STREQUAL "0")
    message(FATAL_ERROR "Held public Build/Run replay refused: ${error}")
endif()
