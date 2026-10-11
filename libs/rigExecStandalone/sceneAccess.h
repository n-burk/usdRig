#ifndef RIGEXEC_STANDALONE_SCENE_ACCESS_H
#define RIGEXEC_STANDALONE_SCENE_ACCESS_H
#include "sceneDb.h"
#include "rigExecScene/sceneAccess.h"

namespace rigExec {
/// Non-owning compiler view. The database must outlive descriptor capture.
class RigExecSceneDbAccess final : public RigExecSceneAccess {
public:
    explicit RigExecSceneDbAccess(const RigExecSceneDb &database) : _db(database) {}
    SdfPathVector Prims(const SdfPath &root) const override;
    bool Prim(const SdfPath &, RigExecScenePrim *) const override;
    SdfPathVector Attributes(const SdfPath &prim) const override;
    SdfPathVector Relationships(const SdfPath &prim) const override;
    bool Attribute(const SdfPath &, RigExecSceneAttribute *) const override;
    bool Relationship(const SdfPath &, RigExecSceneRelationship *) const override;
    bool Resolve(const SdfPath &, UsdTimeCode, VtValue *) const override;
    bool ValueBlocked(const SdfPath &, UsdTimeCode) const override;
    bool HasIdentity(UsdTimeCode time) const override;
    double TimeCodesPerSecond() const override { return _db.timeCodesPerSecond; }
    double FramesPerSecond() const override { return _db.framesPerSecond; }
    TfToken Interpolation() const override { return _db.interpolation; }
    TfToken UpAxis() const override { return _db.upAxis; }
private:
    const RigExecSceneDb &_db;
};
} // namespace rigExec
#endif
