# Removed evaluator policies and scheduling mechanisms, checked in production.
set(RIGEXEC_RETIRED_SCAN_PATHS
    libs python tools plugin/rigExecUsdview)
set(RIGEXEC_RETIRED_SYMBOLS
    RigExecEvaluationMode RigExecEvaluationModeSource
    ExecReference BakedWithParityCheck
    _EvaluateDynamic _EvaluateGeometry _EvaluateChain
    _poseSteps _chainPlan _chainSnapshots RigExecChainSnapshots
    RigExecMoverGraph
    RunHeadTier RunRestTier RunLayoutTier
    _FrozenRunSteps _MakeProductionClusterRunner
    RigExecRunPartialCone RigExecPartialRunResult
    SetRunMaskForTesting _RrFamilyBit
    cpuParityMode bakedParityMismatches
    moverGraphParityAgreements moverGraphParityMismatches
    solverOverrideRounds solverOverridesConverged
    moverGraphRevisionsCreated moverGraphRevisionsExecuted moverGraphSchedulesBuilt
    GetSolverBatchLevels GetChainLevels)
