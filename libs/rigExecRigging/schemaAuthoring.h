//
// Strict, schema-backed authoring for codeless RigExec schemas.
//
// Unlike the coarse RigExecRigBuilder, this low-level API never invents a
// property or accepts a caller-declared Sdf type. Every operation resolves its
// contract from OpenUSD's registered/composed UsdPrimDefinition and fails
// before authoring when that contract is unavailable.
//
#ifndef RIGEXEC_RIGGING_SCHEMA_AUTHORING_H
#define RIGEXEC_RIGGING_SCHEMA_AUTHORING_H

#include "pxr/pxr.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <string>
#include <utility>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// A strict authoring view of one concrete, registered schema prim.
///
/// The wrapper retains the expected concrete schema type and revalidates it on
/// every operation, so replacing the prim's type after Define/Get cannot turn
/// a typed setter into generic USD authoring. All failures throw either
/// std::invalid_argument (invalid request) or std::runtime_error (missing
/// schema/runtime authoring failure).
class RigExecSchemaPrim final {
public:
    /// Define `path` as the registered concrete `schemaType`. Reuses an
    /// existing prim only when its authored type is exactly `schemaType`.
    static RigExecSchemaPrim Define(
        UsdStageRefPtr stage, const SdfPath &path,
        const TfToken &schemaType);

    /// Get an existing prim and require its authored type to exactly equal
    /// `expectedSchemaType`.
    static RigExecSchemaPrim Get(
        UsdStageRefPtr stage, const SdfPath &path,
        const TfToken &expectedSchemaType);

    /// True only while the stage still contains the expected registered type.
    bool IsValid() const;

    /// Return the revalidated prim. Throws if it was removed or retyped.
    UsdPrim GetPrim() const;
    UsdStageRefPtr GetStage() const { return _stage; }
    const SdfPath &GetPath() const { return _path; }
    const TfToken &GetSchemaTypeName() const { return _schemaType; }

    /// Query/apply a registered API by schema identifier (not C++ TfType
    /// name). ApplyAPI checks CanApplyAPI, ApplyAPI, and HasAPI in sequence.
    bool HasAPI(const TfToken &schemaIdentifier) const;
    void ApplyAPI(const TfToken &schemaIdentifier) const;

    /// Set one declared attribute. Its Sdf type and variability come from the
    /// prim's composed definition; an undeclared name or non-exact Vt payload
    /// is rejected without creating a property spec. Control/joint avar scale
    /// channels reject non-finite values and apply the shared signed 1e-4
    /// magnitude floor before authoring.
    void SetAttribute(
        const TfToken &name, const VtValue &value,
        UsdTimeCode time = UsdTimeCode::Default()) const;

    /// Replace the targets of one declared relationship. Attribute names and
    /// undeclared names are rejected before any relationship spec is authored.
    void SetRelationship(
        const TfToken &name, const SdfPathVector &targets) const;

    /// Clear all locally-authored values, samples, and spline data from one
    /// declared attribute. The property declaration itself remains available
    /// from the schema.
    void ClearAttribute(const TfToken &name) const;

    /// Author the registered string metadata field `rigExecReadPhase` on one
    /// declared attribute or relationship.
    void SetReadPhase(
        const TfToken &propertyName, const std::string &phase) const;

private:
    RigExecSchemaPrim(
        UsdStageRefPtr stage, SdfPath path, TfToken schemaType)
        : _stage(std::move(stage)),
          _path(std::move(path)),
          _schemaType(std::move(schemaType)) {}

    UsdPrim _RequirePrim() const;

    UsdStageRefPtr _stage;
    SdfPath _path;
    TfToken _schemaType;
};

}  // namespace rigExec

#endif  // RIGEXEC_RIGGING_SCHEMA_AUTHORING_H
