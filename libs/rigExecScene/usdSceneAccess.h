#ifndef RIGEXEC_GRAPH_USD_SCENE_ACCESS_H
#define RIGEXEC_GRAPH_USD_SCENE_ACCESS_H
#include "sceneAccess.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/attribute.h"
namespace rigExec {
/// USD frontend for the same composed descriptor input used by scene databases.
/// Captures values only; compiled graph callbacks must not retain this adapter.
class RigExecUsdSceneAccess final : public RigExecSceneAccess {
public:
    explicit RigExecUsdSceneAccess(const UsdStageRefPtr &stage) : _stage(stage) {}
    SdfPathVector Prims(const SdfPath &root) const override;
    bool Prim(const SdfPath &, RigExecScenePrim *) const override;
    SdfPathVector Attributes(const SdfPath &prim) const override;
    SdfPathVector Relationships(const SdfPath &prim) const override;
    bool Attribute(const SdfPath &, RigExecSceneAttribute *) const override;
    // Host-only, per-property capture binding. No handle enters descriptors.
    UsdAttribute BindAttribute(const SdfPath &) const;
    bool Attribute(const UsdAttribute &, RigExecSceneAttribute *) const;
    bool Resolve(const UsdAttribute &, UsdTimeCode, VtValue *) const;
    bool ValueBlocked(const UsdAttribute &, UsdTimeCode) const;
    bool Relationship(const SdfPath &, RigExecSceneRelationship *) const override;
    bool Resolve(const SdfPath &, UsdTimeCode, VtValue *) const override;
    bool ValueBlocked(const SdfPath &, UsdTimeCode) const override;
    bool HasIdentity(UsdTimeCode time) const override;
    double TimeCodesPerSecond() const override;
    double FramesPerSecond() const override;
    TfToken Interpolation() const override;
    TfToken UpAxis() const override;
private:
    UsdStageRefPtr _stage;
};
}
#endif
