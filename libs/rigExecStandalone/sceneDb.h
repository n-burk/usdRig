#ifndef RIGEXEC_STANDALONE_SCENE_DB_H
#define RIGEXEC_STANDALONE_SCENE_DB_H

#include "pxr/base/tf/token.h"
#include "pxr/base/vt/dictionary.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/sdf/valueTypeName.h"
#include "pxr/usd/usd/timeCode.h"
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace rigExec {
PXR_NAMESPACE_USING_DIRECTIVE

/// Canonical, exact numeric/PreTime identity; rejects non-finite times.
std::string RigExecStandaloneTimeKey(UsdTimeCode time);

/// Shared export/runtime capability policy. Non-RigExec types are retained as
/// source data; only self-contained RigExec provider schemas are supported.
bool RigExecStandaloneSupportsPrimType(const TfToken &type);

struct RigExecStandaloneAttribute {
    SdfValueTypeName type;
    VtDictionary metadata;
    SdfPathVector connections;
    /// An empty value is an explicit no-value/block state, never a fallback.
    std::map<std::string, VtValue> resolved;
};
struct RigExecStandaloneRelationship {
    VtDictionary metadata;
    SdfPathVector targets;
};
struct RigExecStandalonePrim {
    TfToken type;
    TfTokenVector appliedSchemas;
    VtDictionary metadata;
    bool active = true;
};

/// Compact resolved scene data. No UsdStage or EsfUsd object is retained.
/// All durable authoring stays in USD; runtime edits are ephemeral overlays.
class RigExecSceneDb {
public:
    std::map<SdfPath, RigExecStandalonePrim> prims;
    std::map<SdfPath, RigExecStandaloneAttribute> attributes;
    std::map<SdfPath, RigExecStandaloneRelationship> relationships;
    std::set<std::string> identities;
    double timeCodesPerSecond = 24;
    double framesPerSecond = 24;
    TfToken interpolation = TfToken("linear");
    std::string sourceAssetId;

    bool Validate(std::string *error = nullptr) const;
    /// Reject scene features whose values require RigExecRigEvaluator lowering.
    bool ValidateCapabilities(std::string *error = nullptr) const;
    bool HasObject(const SdfPath &path) const;
    bool IsActive(const SdfPath &path) const;
    bool Get(const SdfPath &path, UsdTimeCode time, VtValue *value) const;
    const void *SchemaKey(const SdfPath &prim) const;
    SdfPathVector IncomingConnections(const SdfPath &path) const;
private:
    /// Keys are globally interned: Exec's process-wide registry caches them
    /// beyond a database lifetime, so database-owned addresses are unsafe.
    mutable std::map<std::pair<TfToken, TfTokenVector>, const void *> _schemaKeys;
    mutable std::map<SdfPath, SdfPathVector> _incomingConnections;
};

} // namespace rigExec
#endif
