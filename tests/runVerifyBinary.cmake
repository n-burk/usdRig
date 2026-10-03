# runVerifyBinary.cmake: one fixture's bake-then-verify gate (M2).
#
# Bakes STAGE at FRAMES with the rigExecBake CLI, then gates the zero-USD
# runtime against the baked path with rigExecPose --verify-binary. Run as a
# ctest entry (see rigexec_add_verify_binary_test in CMakeLists.txt); fails
# loudly on either half. Required -D arguments: BAKE, POSE, STAGE, FRAMES,
# OUT.
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
