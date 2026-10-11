#ifndef RIGEXEC_GRAPH_SCENE_ACCESS_H
#define RIGEXEC_GRAPH_SCENE_ACCESS_H

#include "pxr/base/tf/token.h"
#include "pxr/base/vt/dictionary.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/sdf/valueTypeName.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/timeCode.h"
#include <vector>

namespace rigExec {
PXR_NAMESPACE_USING_DIRECTIVE

struct RigExecScenePrim {
    SdfPath path;
    TfToken type;
    TfTokenVector appliedSchemas;
    VtDictionary metadata;
    bool active = false;
    SdfPathVector children;
};
struct RigExecSceneAttribute {
    SdfPath path;
    SdfValueTypeName type;
    VtDictionary metadata;
    SdfPathVector connections;
    bool hasValue = false;
    bool hasAuthoredValue = false;
    bool hasAuthoredReadableValue = false;
    bool hasAuthoredConnections = false;
    bool mightBeTimeVarying = false;
    bool hasAuthoredDefault = false;
    bool defaultBlocked = false;
    std::vector<double> sampleTimes;
    SdfVariability variability = SdfVariabilityVarying;
    /// Owned composed spline for compile-time LUT lowering, empty when absent.
    VtValue spline;
};
struct RigExecSceneRelationship {
    SdfPath path;
    VtDictionary metadata;
    SdfPathVector targets;
};

/// Composed compiler input. Enumeration is lexical and includes inactive
/// objects; lowering chooses active objects and its own canonical graph order.
/// Values are resolved at an exact identity. A successful empty value means
/// an explicit block/no-value state; false means the identity is unavailable.
/// No stage, schema handle, or source query belongs in compiled descriptors.
class RigExecSceneAccess {
public:
    virtual ~RigExecSceneAccess() = default;
    virtual SdfPathVector Prims(const SdfPath &root) const = 0;
    virtual bool Prim(const SdfPath &, RigExecScenePrim *) const = 0;
    virtual SdfPathVector Attributes(const SdfPath &prim) const = 0;
    virtual SdfPathVector Relationships(const SdfPath &prim) const = 0;
    virtual bool Attribute(const SdfPath &, RigExecSceneAttribute *) const = 0;
    virtual bool Relationship(const SdfPath &, RigExecSceneRelationship *) const = 0;
    virtual bool Resolve(const SdfPath &, UsdTimeCode, VtValue *) const = 0;
    virtual bool ValueBlocked(const SdfPath &, UsdTimeCode) const = 0;
    virtual bool HasIdentity(UsdTimeCode) const = 0;
    virtual double TimeCodesPerSecond() const = 0;
    virtual double FramesPerSecond() const = 0;
    virtual TfToken Interpolation() const = 0;
    virtual TfToken UpAxis() const = 0;
};
} // namespace rigExec
#endif
