#ifndef RIGEXEC_STANDALONE_PACK_H
#define RIGEXEC_STANDALONE_PACK_H
#include "sceneDb.h"
#include "pxr/usd/usd/stage.h"

namespace rigExec {
/// Scene pack: flattened source.usdc plus exact typed binary values carried
/// as manifest.usda hex strings. source.usdc
/// is retained for inspection and never supplies semantic runtime values.
/// Export resolves every discovered attribute at
/// each exact Default/numeric/PreTime identity with stock USD. The shared scene
/// compiler validates all supported producer domains before writing the pack.
/// Version 3 retains exact value bits, recursive metadata, splines and authored
/// Default opinions separately; older packs require regeneration.
/// Default is always captured alongside requested numeric identities because
/// production descriptors read static arrays and operation tokens there.
bool RigExecExportRigPack(const UsdStageRefPtr &stage,
    const std::vector<UsdTimeCode> &times, const std::string &packPath,
    const std::string &sourceAssetId, std::string *error = nullptr);

/// Loads the resolved database through Sdf only; constructs no UsdStage and
/// retains no layer or archive object. Unexported times never interpolate.
std::shared_ptr<RigExecSceneDb> RigExecLoadRigPack(
    const std::string &packPath, std::string *error = nullptr);
}
#endif
