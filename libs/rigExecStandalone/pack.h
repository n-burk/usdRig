#ifndef RIGEXEC_STANDALONE_PACK_H
#define RIGEXEC_STANDALONE_PACK_H
#include "sceneDb.h"
#include "pxr/usd/usd/stage.h"

namespace rigExec {
/// Experimental provider-only pack: standard flattened source.usdc plus a
/// generic typed manifest.usda. Export resolves every discovered attribute at
/// each exact Default/numeric/PreTime identity with stock USD. Unsupported
/// evaluator-side mover/solver-output lowering fails before writing the pack.
bool RigExecExportRigPack(const UsdStageRefPtr &stage,
    const std::vector<UsdTimeCode> &times, const std::string &packPath,
    const std::string &sourceAssetId, std::string *error = nullptr);

/// Loads the resolved database through Sdf only; constructs no UsdStage and
/// retains no layer or archive object. Unexported times never interpolate.
std::shared_ptr<RigExecSceneDb> RigExecLoadRigPack(
    const std::string &packPath, std::string *error = nullptr);
}
#endif
