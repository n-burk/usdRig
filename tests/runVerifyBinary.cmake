# runVerifyBinary.cmake: one fixture's bake-then-verify gate (M2).
#
# Bakes STAGE at FRAMES with the rigExecBake CLI, then gates the zero-USD
# runtime against the baked path with rigExecPose --verify-binary. Run as a
# ctest entry (see rigexec_add_verify_binary_test in CMakeLists.txt); fails
# loudly on either half. Required -D arguments: BAKE, POSE, STAGE, FRAMES,
# OUT; optional: MIN_ENVELOPES, MIN_PHASE_PACKETS, MIN_PROPERTY_VALUES,
# MIN_CHAIN_READS, MIN_REGISTERED_READS, MIN_BLEND_WEIGHTS,
# MIN_DEFAULT_WEIGHTS, MIN_PATH_READS, MIN_BLEND_ACTIVATIONS.
if (NOT DEFINED BAKE OR NOT DEFINED POSE OR NOT DEFINED STAGE
        OR NOT DEFINED FRAMES OR NOT DEFINED OUT)
    message(FATAL_ERROR "runVerifyBinary.cmake: BAKE, POSE, STAGE, FRAMES "
                        "and OUT are all required")
endif()
execute_process(
    COMMAND "${BAKE}" "${STAGE}" --frames "${FRAMES}" -o "${OUT}"
    RESULT_VARIABLE _bake_rc
    OUTPUT_VARIABLE _bake_out
    ERROR_VARIABLE _bake_err)
if (NOT _bake_rc EQUAL 0)
    message(FATAL_ERROR
        "bake failed (${_bake_rc}):\n${_bake_out}\n${_bake_err}")
endif()
# The runtime compares what it computes (property-chain results,
# constraint envelopes, current-phase weight packets, and every input read
# it evaluates over the slots) bit for bit against the frame records the
# bake wrote.
set(ENV{RIGEXEC_RUNTIME_CROSSCHECK} 1)
execute_process(
    COMMAND "${POSE}" "${STAGE}" --verify-binary "${OUT}"
    RESULT_VARIABLE _pose_rc
    OUTPUT_VARIABLE _pose_out
    ERROR_VARIABLE _pose_err)
# Always show the per-frame ledger: a passing gate with no ledger is a gate
# that cannot be audited.
message(STATUS "${_pose_out}")
if (NOT _pose_rc EQUAL 0)
    message(FATAL_ERROR
        "verify-binary failed (${_pose_rc}):\n${_pose_err}")
endif()
# The ledger every run prints with the cross-check on, one count per kind
# (the total is not captured: a CMake regex keeps nine groups).
if (NOT _pose_out MATCHES
        "cross-check: [0-9]+ computed value\\(s\\) matched the frame records \\(([0-9]+) envelope\\(s\\), ([0-9]+) current-phase packet\\(s\\), ([0-9]+) property value\\(s\\), ([0-9]+) chain read\\(s\\), ([0-9]+) registered read\\(s\\), ([0-9]+) blend weight\\(s\\), ([0-9]+) default weight\\(s\\), ([0-9]+) path read\\(s\\), ([0-9]+) blend activation\\(s\\)\\)")
    message(FATAL_ERROR "verify-binary printed no cross-check ledger")
endif()
set(_envelopes "${CMAKE_MATCH_1}")
set(_packets "${CMAKE_MATCH_2}")
set(_properties "${CMAKE_MATCH_3}")
set(_chainReads "${CMAKE_MATCH_4}")
set(_registeredReads "${CMAKE_MATCH_5}")
set(_blendWeights "${CMAKE_MATCH_6}")
set(_defaultWeights "${CMAKE_MATCH_7}")
set(_pathReads "${CMAKE_MATCH_8}")
set(_blendActivations "${CMAKE_MATCH_9}")
# Optional MIN_ENVELOPES / MIN_PHASE_PACKETS / MIN_PROPERTY_VALUES /
# MIN_CHAIN_READS / MIN_REGISTERED_READS / MIN_BLEND_WEIGHTS /
# MIN_DEFAULT_WEIGHTS / MIN_PATH_READS / MIN_BLEND_ACTIVATIONS: a fixture
# that exists to reach the computed paths fails when the cross-check
# compared fewer values.
foreach(_kind IN ITEMS
        "MIN_ENVELOPES;_envelopes;envelope(s)"
        "MIN_PHASE_PACKETS;_packets;current-phase packet(s)"
        "MIN_PROPERTY_VALUES;_properties;property value(s)"
        "MIN_CHAIN_READS;_chainReads;chain read(s)"
        "MIN_REGISTERED_READS;_registeredReads;registered read(s)"
        "MIN_BLEND_WEIGHTS;_blendWeights;blend weight(s)"
        "MIN_DEFAULT_WEIGHTS;_defaultWeights;default weight(s)"
        "MIN_PATH_READS;_pathReads;path read(s)"
        "MIN_BLEND_ACTIVATIONS;_blendActivations;blend activation(s)")
    list(GET _kind 0 _option)
    list(GET _kind 1 _count)
    list(GET _kind 2 _label)
    if (DEFINED ${_option})
        if (${${_count}} LESS ${${_option}})
            message(FATAL_ERROR "the cross-check compared ${${_count}} "
                                "${_label}, expected at least "
                                "${${_option}}")
        endif()
    endif()
endforeach()
