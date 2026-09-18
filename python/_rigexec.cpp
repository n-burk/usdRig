//
// _rigexec: native Python bindings for the RigExec rigging API (spec §4)
// and the staged rig evaluator.
//
// Interop note: this OpenUSD build's pxr modules are Boost.Python, while
// this module is pybind11. Both coexist in one interpreter because they load
// the SAME USD shared libraries through Windows DLL caching. A pxr object
// (e.g. a Usd.Stage) carries its C++ owner as a capsule named "refptr" under
// the instance attribute "__owner" (pxr/base/tf/pyIdentity.h: PyCapsule_New(
// new TfRefPtr<T>(ptr), "refptr", dtor)). _ExtractStage recovers the stage
// through that capsule and copies the refcounted pointer properly.
//
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3i.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/tf/refPtr.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include "rigExec/rigEvaluator.h"
#include "rigExecMath/pointFrame.h"
#include "rigExecMath/rbf.h"
#include "rigExecMath/geometryKernels.h"
#include "rigExecRigging/rigBuilder.h"
#include "rigExecRigging/schemaAuthoring.h"

#include <array>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

namespace py = pybind11;
using namespace py::literals;

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

// ---------------------------------------------------------------------------
// pxr interop: recover a UsdStage* from a Boost.Python-wrapped pxr object.
// ---------------------------------------------------------------------------

/// True if \p obj's MRO contains the named pxr class (e.g. "pxr.Usd.Stage").
bool
_IsPxrType(const py::object &obj, const char *className)
{
    try {
        py::sequence mro = obj.attr("__class__").attr("__mro__");
        for (const auto &base : mro) {
            if (std::string(py::str(base)).find(className) != std::string::npos) {
                return true;
            }
        }
    } catch (py::error_already_set &) {
        return false;
    }
    return false;
}

/// Extract the C++ stage owned by a pxr.Usd.Stage instance. Returns an empty
/// RefPtr on failure (with the Python exception cleared) so callers can raise
/// a precise error.
UsdStageRefPtr
_ExtractStage(const py::object &obj)
{
    if (!_IsPxrType(obj, "pxr.Usd.Stage")) {
        return UsdStageRefPtr();
    }
    PyObject *owner = PyObject_GetAttrString(obj.ptr(), "__owner");
    if (!owner) {
        PyErr_Clear();
        return UsdStageRefPtr();
    }
    UsdStageRefPtr result;
    if (PyCapsule_CheckExact(owner)) {
        void *held = PyCapsule_GetPointer(owner, "refptr");
        if (held && !PyErr_Occurred()) {
            // The capsule owns a heap TfRefPtr<UsdStage>; copy it so the
            // refcount stays balanced on both sides.
            result = UsdStageRefPtr(*static_cast<TfRefPtr<UsdStage> *>(held));
        } else if (PyErr_Occurred()) {
            PyErr_Clear();
        }
    }
    Py_DECREF(owner);
    return result;
}

// ---------------------------------------------------------------------------
// Value conversion helpers.
// ---------------------------------------------------------------------------

std::vector<double>
_Mat4ToVec(const GfMatrix4d &m)
{
    std::vector<double> out(16);
    for (int i = 0; i < 4; ++i) {
        const GfVec4d row = m.GetRow(i);
        for (int j = 0; j < 4; ++j) {
            out[i * 4 + j] = row[j];
        }
    }
    return out;
}

GfMatrix4d
_VecToMat4(const std::vector<double> &v)
{
    if (v.size() != 16) {
        throw py::value_error("matrix4d needs exactly 16 numbers, row-major");
    }
    return GfMatrix4d(
        v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7],
        v[8], v[9], v[10], v[11], v[12], v[13], v[14], v[15]);
}

std::vector<double>
_Vec3ToVec(const GfVec3d &v) { return { v[0], v[1], v[2] }; }

GfMatrix4d
_FrameToMatrix(const rigExec::RigExecPointFrame &f)
{
    const GfVec3d o = f.Origin();
    // Basis rows are the handle vectors (landmark minus origin), not the
    // absolute landmark positions: a translated frame must keep its linear
    // part unchanged.
    const GfVec3d ex = f.X() - o;
    const GfVec3d ey = f.Y() - o;
    const GfVec3d ez = f.Z() - o;
    return GfMatrix4d(
        ex[0], ex[1], ex[2], 0,
        ey[0], ey[1], ey[2], 0,
        ez[0], ez[1], ez[2], 0,
        o[0], o[1], o[2], 1);
}

/// Convert a VtValue to the most natural Python value by held type.
py::object
_VtToPython(const VtValue &v)
{
    if (v.IsHolding<bool>()) {
        return py::cast(v.Get<bool>());
    }
    if (v.IsHolding<int>()) {
        return py::cast(v.Get<int>());
    }
    if (v.IsHolding<float>()) {
        return py::cast(double(v.Get<float>()));
    }
    if (v.IsHolding<double>()) {
        return py::cast(v.Get<double>());
    }
    if (v.IsHolding<GfVec3f>()) {
        const GfVec3f p = v.Get<GfVec3f>();
        return py::make_tuple(double(p[0]), double(p[1]), double(p[2]));
    }
    if (v.IsHolding<GfMatrix4d>()) {
        return py::cast(_Mat4ToVec(v.Get<GfMatrix4d>()));
    }
    if (v.IsHolding<VtVec3fArray>()) {
        const VtVec3fArray a = v.Get<VtVec3fArray>();
        std::vector<std::array<double, 3>> out;
        out.reserve(a.size());
        for (const auto &p : a) {
            out.push_back({ double(p[0]), double(p[1]), double(p[2]) });
        }
        return py::cast(out);
    }
    if (v.IsHolding<TfToken>()) {
        return py::cast(v.Get<TfToken>().GetString());
    }
    // Fallback: the value's string form, never a silent loss.
    std::ostringstream oss;
    oss << v;
    return py::cast(oss.str());
}

std::string
_PathStr(const SdfPath &p) { return p.IsEmpty() ? "" : p.GetString(); }

SdfPath
_StrToPath(const std::string &s) { return s.empty() ? SdfPath() : SdfPath(s); }

GfVec3f
_PythonToVec3f(const py::handle &value, const std::string &context)
{
    if (py::isinstance<py::str>(value) ||
        py::isinstance<py::bytes>(value) || !PySequence_Check(value.ptr())) {
        throw py::type_error(context + " needs a sequence of 3 numbers");
    }
    py::sequence seq = py::reinterpret_borrow<py::sequence>(value);
    if (seq.size() != 3) {
        throw py::type_error(context + " needs exactly 3 numbers");
    }
    return GfVec3f(
        float(seq[0].cast<double>()), float(seq[1].cast<double>()),
        float(seq[2].cast<double>()));
}

GfVec3d
_PythonToVec3d(const py::handle &value, const std::string &context)
{
    if (py::isinstance<py::str>(value) ||
        py::isinstance<py::bytes>(value) || !PySequence_Check(value.ptr())) {
        throw py::type_error(context + " needs a sequence of 3 numbers");
    }
    py::sequence seq = py::reinterpret_borrow<py::sequence>(value);
    if (seq.size() != 3) {
        throw py::type_error(context + " needs exactly 3 numbers");
    }
    return GfVec3d(
        seq[0].cast<double>(), seq[1].cast<double>(),
        seq[2].cast<double>());
}

GfVec3i
_PythonToVec3i(const py::handle &value, const std::string &context)
{
    if (py::isinstance<py::str>(value) ||
        py::isinstance<py::bytes>(value) || !PySequence_Check(value.ptr())) {
        throw py::type_error(context + " needs a sequence of 3 integers");
    }
    py::sequence seq = py::reinterpret_borrow<py::sequence>(value);
    if (seq.size() != 3) {
        throw py::type_error(context + " needs exactly 3 integers");
    }
    return GfVec3i(
        seq[0].cast<int>(), seq[1].cast<int>(), seq[2].cast<int>());
}

/// Convert a Python value using the attribute's actual composed schema type.
/// No type name is supplied by Python: that avoids duplicating the schema and
/// makes misspelled/off-schema properties fail before authoring anything.
VtValue
_PythonToSchemaValue(
    const UsdPrim &prim, const TfToken &attributeName, const py::object &value)
{
    if (!prim) {
        throw py::value_error("invalid prim for schema attribute");
    }
    const UsdAttribute attr = prim.GetAttribute(attributeName);
    if (!attr) {
        throw py::key_error(
            "attribute '" + attributeName.GetString() +
            "' is not declared by the prim's concrete or applied schemas");
    }

    const std::string typeName = attr.GetTypeName().GetAsToken().GetString();
    const std::string context =
        "attribute '" + attributeName.GetString() + "' (" + typeName + ")";

    if (typeName == "bool") {
        if (!PyBool_Check(value.ptr())) {
            throw py::type_error(context + " needs a bool");
        }
        return VtValue(value.cast<bool>());
    }
    if (typeName == "int") {
        if (PyBool_Check(value.ptr())) {
            throw py::type_error(context + " needs an integer, not bool");
        }
        return VtValue(value.cast<int>());
    }
    if (typeName == "float") {
        if (PyBool_Check(value.ptr())) {
            throw py::type_error(context + " needs a number, not bool");
        }
        return VtValue(float(value.cast<double>()));
    }
    if (typeName == "double") {
        if (PyBool_Check(value.ptr())) {
            throw py::type_error(context + " needs a number, not bool");
        }
        return VtValue(value.cast<double>());
    }
    if (typeName == "token") {
        if (!py::isinstance<py::str>(value)) {
            throw py::type_error(context + " needs a string token");
        }
        return VtValue(TfToken(value.cast<std::string>()));
    }
    if (typeName == "float3" || typeName == "color3f" ||
        typeName == "point3f" || typeName == "normal3f" ||
        typeName == "vector3f") {
        return VtValue(_PythonToVec3f(value, context));
    }
    if (typeName == "double3" || typeName == "point3d" ||
        typeName == "vector3d") {
        return VtValue(_PythonToVec3d(value, context));
    }
    if (typeName == "int3") {
        return VtValue(_PythonToVec3i(value, context));
    }
    if (typeName == "matrix4d") {
        if (py::isinstance<py::str>(value) ||
            py::isinstance<py::bytes>(value) ||
            !PySequence_Check(value.ptr())) {
            throw py::type_error(
                context + " needs a sequence of 16 row-major numbers");
        }
        py::sequence seq = value;
        if (seq.size() != 16) {
            throw py::type_error(context + " needs 16 row-major numbers");
        }
        std::vector<double> numbers(16);
        for (size_t i = 0; i < numbers.size(); ++i) {
            numbers[i] = seq[i].cast<double>();
        }
        return VtValue(_VecToMat4(numbers));
    }

    if (py::isinstance<py::str>(value) ||
        py::isinstance<py::bytes>(value) || !PySequence_Check(value.ptr())) {
        throw py::type_error(context + " needs a non-string sequence");
    }
    py::sequence seq = value;
    if (typeName == "token[]") {
        VtTokenArray out(seq.size());
        for (size_t i = 0; i < out.size(); ++i) {
            out[i] = TfToken(seq[i].cast<std::string>());
        }
        return VtValue(out);
    }
    if (typeName == "float[]") {
        VtFloatArray out(seq.size());
        for (size_t i = 0; i < out.size(); ++i) {
            out[i] = float(seq[i].cast<double>());
        }
        return VtValue(out);
    }
    if (typeName == "double[]") {
        VtDoubleArray out(seq.size());
        for (size_t i = 0; i < out.size(); ++i) {
            out[i] = seq[i].cast<double>();
        }
        return VtValue(out);
    }
    if (typeName == "int[]") {
        VtIntArray out(seq.size());
        for (size_t i = 0; i < out.size(); ++i) {
            out[i] = seq[i].cast<int>();
        }
        return VtValue(out);
    }
    if (typeName == "int64[]") {
        VtInt64Array out(seq.size());
        for (size_t i = 0; i < out.size(); ++i) {
            out[i] = seq[i].cast<int64_t>();
        }
        return VtValue(out);
    }
    if (typeName == "float3[]" || typeName == "color3f[]" ||
        typeName == "point3f[]" || typeName == "normal3f[]" ||
        typeName == "vector3f[]") {
        VtArray<GfVec3f> out(seq.size());
        for (size_t i = 0; i < out.size(); ++i) {
            out[i] = _PythonToVec3f(seq[i], context + " row");
        }
        return VtValue(out);
    }
    if (typeName == "double3[]") {
        VtArray<GfVec3d> out(seq.size());
        for (size_t i = 0; i < out.size(); ++i) {
            out[i] = _PythonToVec3d(seq[i], context + " row");
        }
        return VtValue(out);
    }

    throw py::type_error(
        "unsupported schema value type '" + typeName + "' for " +
        attributeName.GetString());
}

SdfPath
_PathFromText(const std::string &text, bool allowEmpty)
{
    if (text.empty()) {
        if (allowEmpty) {
            return SdfPath();
        }
        throw py::value_error("path must not be empty");
    }
    std::string whyNot;
    if (!SdfPath::IsValidPathString(text, &whyNot)) {
        throw py::value_error(
            "invalid USD path '" + text + "'" +
            (whyNot.empty() ? std::string() : ": " + whyNot));
    }
    return SdfPath(text);
}

SdfPath
_PythonToPath(const py::handle &value, bool allowEmpty = true)
{
    if (value.is_none()) {
        if (allowEmpty) {
            return SdfPath();
        }
        throw py::value_error("path must not be None");
    }
    if (py::isinstance<py::str>(value)) {
        return _PathFromText(value.cast<std::string>(), allowEmpty);
    }
    if (py::hasattr(value, "pathString")) {
        return _PathFromText(
            py::str(value.attr("pathString")).cast<std::string>(),
            allowEmpty);
    }
    if (py::hasattr(value, "path")) {
        return _PathFromText(
            py::str(value.attr("path")).cast<std::string>(), allowEmpty);
    }
    if (py::hasattr(value, "GetPath")) {
        return _PathFromText(
            py::str(value.attr("GetPath")()).cast<std::string>(),
            allowEmpty);
    }
    throw py::type_error("expected a path string, RigExec handle, or Usd.Prim");
}

SdfPath
_PythonToDependencyPath(
    const py::handle &value, const UsdStageRefPtr &ownerStage,
    const TfToken &expectedType = TfToken(), bool allowEmpty = true)
{
    const SdfPath path = _PythonToPath(value, allowEmpty);
    TfToken actualType;
    UsdStageRefPtr dependencyStage;

    if (py::isinstance<rigExec::RigExecHandleBase>(value)) {
        const auto &handle = value.cast<const rigExec::RigExecHandleBase &>();
        if (!handle.IsValid()) {
            throw py::value_error("dependency handle is no longer valid");
        }
        dependencyStage = handle.GetStage();
        actualType = handle.GetSchemaTypeName();
    } else if (py::isinstance<rigExec::RigExecSchemaPrim>(value)) {
        const auto &schemaPrim =
            value.cast<const rigExec::RigExecSchemaPrim &>();
        if (!schemaPrim.IsValid()) {
            throw py::value_error("dependency SchemaPrim is no longer valid");
        }
        dependencyStage = schemaPrim.GetStage();
        actualType = schemaPrim.GetSchemaTypeName();
    } else if (_IsPxrType(
                   py::reinterpret_borrow<py::object>(value),
                   "pxr.Usd.Prim")) {
        const int isValid = PyObject_IsTrue(value.ptr());
        if (isValid < 0) {
            throw py::error_already_set();
        }
        if (!isValid) {
            throw py::value_error("dependency Usd.Prim is no longer valid");
        }
        dependencyStage = _ExtractStage(value.attr("GetStage")());
        actualType = TfToken(
            py::str(value.attr("GetTypeName")()).cast<std::string>());
    } else if (py::hasattr(value, "GetPrim")) {
        const py::object prim = value.attr("GetPrim")();
        const int isValid = prim.is_none() ? 0 : PyObject_IsTrue(prim.ptr());
        if (isValid < 0) {
            throw py::error_already_set();
        }
        if (!isValid) {
            throw py::value_error("dependency Usd schema object is no longer valid");
        }
        if (py::hasattr(prim, "GetStage") &&
            py::hasattr(prim, "GetTypeName")) {
            dependencyStage = _ExtractStage(prim.attr("GetStage")());
            actualType = TfToken(
                py::str(prim.attr("GetTypeName")()).cast<std::string>());
        }
    }

    if (dependencyStage && ownerStage && dependencyStage != ownerStage) {
        throw py::value_error(
            "dependency handle/prim belongs to a different UsdStage");
    }
    if (!expectedType.IsEmpty()) {
        if (actualType.IsEmpty()) {
            if (!ownerStage || !path.IsPrimPath()) {
                throw py::type_error(
                    "dependency must name a " + expectedType.GetString() +
                    " prim");
            }
            const UsdPrim prim = ownerStage->GetPrimAtPath(path);
            if (!prim) {
                throw py::value_error(
                    "typed dependency does not exist: " + path.GetString());
            }
            actualType = prim.GetTypeName();
        }
        if (actualType != expectedType) {
            throw py::type_error(
                "dependency must be a " + expectedType.GetString() +
                ", got " + actualType.GetString());
        }
    }
    return path;
}

SdfPathVector
_PythonToDependencyPaths(
    const py::iterable &values, const UsdStageRefPtr &ownerStage,
    const TfToken &expectedType = TfToken())
{
    if (py::isinstance<py::str>(values) ||
        py::isinstance<py::bytes>(values)) {
        throw py::type_error(
            "expected an iterable of path-like values, not a string");
    }
    SdfPathVector out;
    for (const py::handle &value : values) {
        out.push_back(_PythonToDependencyPath(
            value, ownerStage, expectedType, false));
    }
    return out;
}

// ---------------------------------------------------------------------------
// Rig: the evaluator wrapper.
// ---------------------------------------------------------------------------

const char *
_ModeName(rigExec::RigExecEvaluationMode mode)
{
    switch (mode) {
    case rigExec::RigExecEvaluationMode::Baked: return "baked";
    case rigExec::RigExecEvaluationMode::BakedWithParityCheck: return "parity";
    case rigExec::RigExecEvaluationMode::Dynamic: break;
    }
    return "dynamic";
}

// Who chose the mode. Spelled as the thing a reader would go and look at:
// the attribute by its property name, the variable by its own name.
const char *
_ModeSourceName(rigExec::RigExecEvaluationModeSource source)
{
    switch (source) {
    case rigExec::RigExecEvaluationModeSource::Explicit: return "explicit";
    case rigExec::RigExecEvaluationModeSource::Environment:
        return "environment";
    case rigExec::RigExecEvaluationModeSource::Attribute: return "attribute";
    case rigExec::RigExecEvaluationModeSource::Default: break;
    }
    return "default";
}

rigExec::RigExecEvaluationMode
_ParseMode(const std::string &name)
{
    if (name == "baked") return rigExec::RigExecEvaluationMode::Baked;
    if (name == "parity") {
        return rigExec::RigExecEvaluationMode::BakedWithParityCheck;
    }
    if (name != "dynamic") {
        throw py::value_error(
            "evaluation_mode must be 'dynamic', 'baked' or 'parity' (got '" +
            name + "')");
    }
    return rigExec::RigExecEvaluationMode::Dynamic;
}

struct _Rig {
    UsdStageRefPtr stage;  ///< keeps the stage alive for the rig's lifetime
    SdfPath rigPath;
    std::unique_ptr<rigExec::RigExecRigEvaluator> evaluator;

    explicit _Rig(UsdStageRefPtr s, const SdfPath &p) : stage(std::move(s)) {
        if (!stage) {
            throw py::value_error("could not extract a UsdStage from the given object");
        }
        rigPath = p;
        evaluator.reset(new rigExec::RigExecRigEvaluator(stage, rigPath));
    }

    // Compile and Evaluate run WITHOUT the GIL, and that is load-bearing.
    //
    // Both dispatch work to TBB and wait for it. The first task to ask
    // OpenExec for a computation definition makes Exec_DefinitionRegistry
    // load the plugin that defines it, and TfDlopen finishes a load by
    // importing the library's Python module, which takes the GIL
    // (TfScriptModuleLoader::LoadModules -> TfPyLock). When that first
    // task lands on a worker thread while this thread holds the GIL and
    // spins in WorkDispatcher::Wait for the worker, neither can move --
    // one in five runs of the evaluator tests hung exactly there, and
    // PXR_WORK_THREAD_LIMIT=1 made it go away because the task then ran
    // on this thread. usdview never saw it: ctypes drops the GIL around
    // every foreign call. Nothing the evaluator does needs the GIL held,
    // so the two entry points that dispatch work give it up for the
    // duration; the exception below is thrown with it held again.
    void Compile() {
        std::vector<std::string> errors;
        bool ok = false;
        {
            py::gil_scoped_release release;
            ok = evaluator->Compile(&errors);
        }
        if (!ok) {
            std::string msg = "rig compile failed:";
            for (const auto &e : errors) {
                msg += "\n  - " + e;
            }
            throw py::value_error(msg);
        }
    }

    rigExec::RigExecRigPose Evaluate(double timeFrames) const {
        UsdTimeCode t = timeFrames < 0 ? UsdTimeCode::Default() : UsdTimeCode(timeFrames);
        py::gil_scoped_release release;
        return evaluator->Evaluate(t);
    }
};

// ---------------------------------------------------------------------------
// RBF pose interpolators (libs/rigExecMath/rbf.h).
//
// Enough surface for the converter to stop importing a Python
// solver: a fitter that hands back a solved TABLE as a plain dict,
// and an evaluator that takes that same dict back. The dict is exactly the
// record a converted rig writes out, so it round-trips through JSON and
// through the schema without a second shape to keep in step.
//
// Free functions rather than a class on purpose. The solve is a build-time
// step whose product is data; a live object would invite the per-frame path
// to reach back through Python, which is the thing this port exists to stop.
// ---------------------------------------------------------------------------

std::vector<double>
_SeqToDoubles(const py::object &obj)
{
    std::vector<double> out;
    if (obj.is_none()) {
        return out;
    }
    for (const py::handle &item : py::iterable(obj)) {
        out.push_back(item.cast<double>());
    }
    return out;
}

GfVec3d
_SeqToVec3d(const py::object &obj)
{
    const std::vector<double> values = _SeqToDoubles(obj);
    return GfVec3d(values.size() > 0 ? values[0] : 0.0,
                   values.size() > 1 ? values[1] : 0.0,
                   values.size() > 2 ? values[2] : 0.0);
}

std::vector<GfVec3d>
_SeqToVec3ds(const py::object &obj)
{
    std::vector<GfVec3d> out;
    if (obj.is_none()) {
        return out;
    }
    for (const py::handle &item : py::iterable(obj)) {
        out.push_back(_SeqToVec3d(py::reinterpret_borrow<py::object>(item)));
    }
    return out;
}

/// Fill the parts of a desc that the fitter and the evaluator share.
rigExec::RigExecRbfSolverDesc
_RbfDescFrom(const py::object &poses, const py::object &translations,
             const std::string &kernel, const py::object &poseTypes,
             const py::object &twistAxis, double regularization,
             bool enableRotation, bool enableTranslation)
{
    rigExec::RigExecRbfSolverDesc desc;
    desc.poses = _SeqToVec3ds(poses);
    desc.translations = _SeqToVec3ds(translations);
    // Anything that is not "linear" is the gaussian, which is the Python's
    // own fallback for an unknown kernel name (rbf.py:117-120).
    desc.kernel = kernel == "linear" ? rigExec::RigExecRbfKernel::Linear
                                     : rigExec::RigExecRbfKernel::Gaussian;
    for (double value : _SeqToDoubles(poseTypes)) {
        const int kind = int(value);
        desc.poseTypes.push_back(
            kind == 1   ? rigExec::RigExecRbfPoseType::Swing
            : kind == 2 ? rigExec::RigExecRbfPoseType::Twist
                        : rigExec::RigExecRbfPoseType::Whole);
    }
    if (!twistAxis.is_none()) {
        desc.twistAxis = _SeqToVec3d(twistAxis);
    }
    desc.regularization = regularization;
    desc.enableRotation = enableRotation;
    desc.enableTranslation = enableTranslation;
    return desc;
}

/// The solved interpolator as the dict the converter writes out.
py::dict
_RbfTable(const rigExec::RigExecRbfSolver &solver,
          const rigExec::RigExecRbfSolverDesc &desc,
          const std::string &kernel)
{
    py::list poses, translations, weights;
    for (const GfVec3d &p : solver.GetPoses()) {
        poses.append(py::make_tuple(p[0], p[1], p[2]));
    }
    for (const GfVec3d &t : solver.GetTranslations()) {
        translations.append(py::make_tuple(t[0], t[1], t[2]));
    }
    for (const std::vector<double> &row : solver.GetWeights()) {
        weights.append(py::cast(row));
    }
    py::list poseTypes;
    for (rigExec::RigExecRbfPoseType kind : desc.poseTypes) {
        poseTypes.append(int(kind));
    }
    py::dict table;
    table["poses"] = poses;
    table["translations"] = translations;
    table["kernel"] = kernel;
    table["radius"] = solver.GetRadius();
    table["radii"] = py::cast(solver.GetRadii());
    table["translation_radius"] = solver.GetTranslationRadius();
    table["translation_radii"] = py::cast(solver.GetTranslationRadii());
    table["pose_types"] = poseTypes;
    table["twist_axis"] = py::make_tuple(desc.twistAxis[0], desc.twistAxis[1],
                                         desc.twistAxis[2]);
    table["regularization"] = desc.regularization;
    table["normalize"] = solver.GetNormalize();
    table["enable_rotation"] = solver.GetEnableRotation();
    table["enable_translation"] = solver.GetEnableTranslation();
    table["degenerate"] = solver.Degenerate();
    table["singular"] = solver.GetRegularizedSingular();
    table["weights"] = weights;
    return table;
}

/// Rebuild a solved interpolator from such a dict.
rigExec::RigExecRbfSolver
_RbfSolverFromTable(const py::dict &table)
{
    auto get = [&table](const char *key) -> py::object {
        return table.contains(key)
                   ? py::reinterpret_borrow<py::object>(table[key])
                   : py::none();
    };
    const py::object kernel = get("kernel");
    rigExec::RigExecRbfSolverDesc desc = _RbfDescFrom(
        get("poses"), get("translations"),
        kernel.is_none() ? std::string("gaussian")
                         : kernel.cast<std::string>(),
        get("pose_types"), get("twist_axis"),
        get("regularization").is_none()
            ? 0.0
            : get("regularization").cast<double>(),
        get("enable_rotation").is_none()
            ? true
            : get("enable_rotation").cast<bool>(),
        get("enable_translation").is_none()
            ? false
            : get("enable_translation").cast<bool>());
    // The shared widths go in through the desc so the constructor does not
    // measure its own; the per-pose ones and the inverted matrix are adopted
    // wholesale, because a shipped table's widths may carry a painted
    // poseFalloff that no falloff vector would reproduce.
    desc.radius = get("radius").is_none() ? 0.0
                                          : get("radius").cast<double>();
    desc.translationRadius = get("translation_radius").is_none()
                                 ? 0.0
                                 : get("translation_radius").cast<double>();
    desc.normalize = get("normalize").is_none()
                         ? true
                         : get("normalize").cast<bool>();

    std::vector<std::vector<double>> weights;
    if (!get("weights").is_none()) {
        for (const py::handle &row : py::iterable(get("weights"))) {
            weights.push_back(
                _SeqToDoubles(py::reinterpret_borrow<py::object>(row)));
        }
    }
    rigExec::RigExecRbfSolver solver(desc);
    solver.SetSolvedTable(_SeqToDoubles(get("radii")),
                          _SeqToDoubles(get("translation_radii")), weights);
    if (weights.empty()) {
        solver.Solve();
    }
    return solver;
}

}  // namespace

// ---------------------------------------------------------------------------
// Module definition.
// ---------------------------------------------------------------------------

PYBIND11_MODULE(_rigexec, m) {
    m.doc() = "RigExec native bindings: rigging API (spec section 4) and the staged rig evaluator.";
    m.attr("__version__") = "0.1.0";

    // ---- PointFrame -------------------------------------------------------

    py::class_<rigExec::RigExecPointFrame>(m, "PointFrame",
        "One point frame: origin O plus orthonormal-ish axes X, Y, Z (USD row-vector convention).")
        .def_property_readonly("origin", [](const rigExec::RigExecPointFrame &f) {
            return _Vec3ToVec(f.Origin());
        })
        .def_property_readonly("x_axis", [](const rigExec::RigExecPointFrame &f) {
            return _Vec3ToVec(f.X());
        })
        .def_property_readonly("y_axis", [](const rigExec::RigExecPointFrame &f) {
            return _Vec3ToVec(f.Y());
        })
        .def_property_readonly("z_axis", [](const rigExec::RigExecPointFrame &f) {
            return _Vec3ToVec(f.Z());
        })
        .def_property_readonly("valid", &rigExec::RigExecPointFrame::IsValid)
        .def_property_readonly("degenerate", &rigExec::RigExecPointFrame::IsDegenerate)
        .def("to_matrix4", [](const rigExec::RigExecPointFrame &f) {
            return _Mat4ToVec(_FrameToMatrix(f));
        }, "The 16-number row-major matrix mapping local to frame space.")
        .def("__repr__", [](const rigExec::RigExecPointFrame &f) {
            auto o = f.Origin();
            return "<PointFrame origin=(" + std::to_string(o[0]) + ", " +
                   std::to_string(o[1]) + ", " + std::to_string(o[2]) + ")>";
        });

    // ---- Rig (evaluator) ---------------------------------------------------

    py::class_<_Rig>(m, "Rig",
        "Compiles and evaluates one RigExecRoot on a pxr.Usd.Stage.\n\n"
        ">>> rig = _rigexec.Rig(stage, '/Model/Rig')\n"
        ">>> rig.compile()\n"
        ">>> pose = rig.evaluate(1024)")
        .def(py::init([](py::object stageObj, std::string rigPath) {
                 auto s = _ExtractStage(stageObj);
                 if (!s) {
                     throw py::type_error(
                         "Rig() needs a pxr.Usd.Stage instance (got an object without a usable __owner capsule)");
                 }
                 return std::make_unique<_Rig>(std::move(s), SdfPath(rigPath));
             }),
             py::arg("stage"), py::arg("rig_path") = "/Rig")
        .def_property_readonly("rig_path", [](const _Rig &r) { return _PathStr(r.rigPath); })
        .def("compile", py::overload_cast<>(&_Rig::Compile),
             "Discovers joints and movers, validates targets. Raises ValueError with all messages on failure.")
        .def("evaluate", [](const _Rig &r, double time) { return r.Evaluate(time); },
             py::arg("time") = -1.0,
             "Evaluate one generation; negative time means the stage default.")
        .def_property("cpu_parity_mode",
            [](_Rig &r) { return r.evaluator->cpuParityMode; },
            [](_Rig &r, bool v) { r.evaluator->cpuParityMode = v; })
        .def_property("evaluation_mode",
            [](_Rig &r) { return _ModeName(r.evaluator->GetEvaluationMode()); },
            [](_Rig &r, std::string v) {
                r.evaluator->SetEvaluationMode(_ParseMode(v));
            },
            "'dynamic' (OpenExec plus the pose walk), 'baked' (the flattened\n"
            "epoch when the rig allows it, dynamic otherwise), or 'parity'\n"
            "(both, compared with exact equality; see\n"
            "Pose.baked_parity_mismatches). Baked is a request: setting it\n"
            "can never change an answer, only how fast it arrives.\n"
            "Setting it also takes the decision away from the rig's own\n"
            "rigExec:baked for good; see evaluation_mode_source.")
        .def_property("publish_weight_fields",
            [](_Rig &r) { return r.evaluator->GetPublishWeightFields(); },
            [](_Rig &r, bool v) { r.evaluator->SetPublishWeightFields(v); },
            "Whether each evaluation resolves the per-point weight fields\n"
            "Pose.weight_field reads. On by default; the viewer turns it off\n"
            "until a weight overlay is shown.")
        .def_property_readonly("evaluation_mode_source",
            [](_Rig &r) {
                return _ModeSourceName(r.evaluator->GetEvaluationModeSource());
            },
            "Who chose evaluation_mode: 'explicit' (this property was set),\n"
            "'environment' (a non-empty RIGEXEC_EVALUATION_MODE),\n"
            "'attribute' (the rig's own uniform bool rigExec:baked) or\n"
            "'default' (nobody asked). That is also the precedence, highest\n"
            "first -- an interactive host leaves the mode alone so a rig\n"
            "authored rigExec:baked = true opens through the program.")
        .def_property_readonly("baked_cluster_count", [](const _Rig &r) {
                return r.evaluator->GetBakedClusterCount();
            },
            "How many clusters the standing baked program holds; zero with\n"
            "no program.")
        .def_property_readonly("baked_clusters_run_last_generation",
            [](const _Rig &r) {
                return r.evaluator->GetBakedClustersRunLastGeneration();
            },
            "How many of those clusters the last generation ran. Cone\n"
            "re-execution is invisible on a published pose -- a frame that\n"
            "re-ran everything publishes the same numbers as one that\n"
            "skipped the right half -- so this is what makes a skipped\n"
            "cone observable, with baked_cluster_count beside it.")
        .def_property_readonly("skin_topology_cache_size", [](const _Rig &r) {
                return r.evaluator->GetSkinTopologyCacheSize();
            },
            "How many skin layouts the epoch's topology cache holds answers\n"
            "for. Dropping and re-reading a layout publishes the same\n"
            "deformation as keeping it, so only the cache's occupancy says\n"
            "whether an interactive override paid for the re-read.")
        .def("set_interactive_overrides",
             [](_Rig &r, const std::vector<std::tuple<std::string, std::string,
                                                     py::object>> &entries) {
                 // The shape the manipulation path actually pushes: an
                 // ATTRIBUTE override per dragged avar, with no computation
                 // named (rigExecImaging/registry.cpp _ResolvePreviewSample).
                 std::vector<rigExec::RigExecValueOverride> overrides;
                 overrides.reserve(entries.size());
                 for (const auto &entry : entries) {
                     const py::object &value = std::get<2>(entry);
                     if (!py::isinstance<py::float_>(value) &&
                         !py::isinstance<py::int_>(value)) {
                         throw py::type_error(
                             "interactive override values must be numeric");
                     }
                     // Typed like the attribute, as the viewport's preview
                     // channel types it: a float dial overridden with a
                     // double VtValue is a different value to every reader
                     // that asks for a float.
                     const SdfPath primPath(std::get<0>(entry));
                     const TfToken attrName(std::get<1>(entry));
                     VtValue typed(value.cast<double>());
                     if (const UsdStageRefPtr &s =
                             r.evaluator->GetEvaluationStage()) {
                         const UsdAttribute a = s->GetAttributeAtPath(
                             primPath.AppendProperty(attrName));
                         if (a && a.GetTypeName() == SdfValueTypeNames->Float) {
                             typed = VtValue(static_cast<float>(value.cast<double>()));
                         } else if (a && a.GetTypeName() ==
                                             SdfValueTypeNames->Int) {
                             typed = VtValue(static_cast<int>(value.cast<double>()));
                         }
                     }
                     overrides.push_back(rigExec::RigExecValueOverride{
                         primPath, TfToken(), attrName, typed});
                 }
                 r.evaluator->SetInteractiveOverrides(std::move(overrides));
             },
             py::arg("overrides"),
             "Uncommitted manipulation values as (prim_path, attribute, value)\ntriples -- the route a gizmo drag takes, with NOTHING authored.\nSetting them does not evaluate.")
        .def("clear_interactive_overrides", [](_Rig &r) {
                 r.evaluator->ClearInteractiveOverrides();
             },
             "Drops every interactive override; the next evaluate is the\nauthored rig again.")
        .def_property_readonly("has_interactive_overrides", [](const _Rig &r) {
                 return r.evaluator->HasInteractiveOverrides();
             })
        .def("solver_batch_levels", [](const _Rig &r) {
                 std::map<std::string, size_t> out;
                 for (const auto &entry : r.evaluator->GetSolverBatchLevels()) {
                     out[_PathStr(entry.first)] = entry.second;
                 }
                 return out;
             },
             "Aggregate solver path -> its dependency level in the compiled\npose schedule. Diagnostic: evaluation order comes from the\ninterleaved pose steps, not from this map.")
        .def("chain_levels", [](const _Rig &r) {
                 std::vector<py::dict> out;
                 for (size_t i = 0; i < r.evaluator->GetChainLevelCount(); ++i) {
                     py::dict d;
                     std::vector<std::string> targets;
                     for (const SdfPath &t :
                          r.evaluator->GetChainLevelTargets(i)) {
                         targets.push_back(_PathStr(t));
                     }
                     d["targets"] = targets;
                     d["parallel"] = r.evaluator->IsChainLevelParallel(i);
                     out.push_back(d);
                 }
                 return out;
             },
             "The compiled geometry-chain levels, in walk order.")
        .def("is_bakeable", [](const _Rig &r) {
            return r.evaluator->IsBakeable(nullptr);
        }, "Whether the compiled epoch can be expressed as a baked program.")
        .def("bakeability_reasons", [](const _Rig &r) {
            std::vector<std::string> reasons;
            r.evaluator->IsBakeable(&reasons);
            return reasons;
        }, "One reason per feature that stops the rig from baking; empty\n"
           "when is_bakeable() is true.")
        .def("binding_epoch_digest", [](const _Rig &r) {
            return r.evaluator->GetBindingEpochDigest();
        })
        .def("mover_order", [](const _Rig &r) {
            std::vector<py::dict> out;
            for (const auto &rec : r.evaluator->GetMoverOrder()) {
                py::dict d;
                d["path"] = _PathStr(rec.moverPath);
                d["type"] = rec.schemaType.GetString();
                std::vector<std::string> targets;
                for (const auto &t : rec.targets) {
                    targets.push_back(_PathStr(t));
                }
                d["targets"] = targets;
                d["ordinal"] = rec.ordinal;
                out.push_back(d);
            }
            return out;
        }, "The composed movers in execution order (reverse-sibling\npost-order: bottom-to-top stack walk, spec section 4.2).")
        .def_property("profiling_enabled",
            [](_Rig &r) { return r.evaluator->GetProfilingEnabled(); },
            [](_Rig &r, bool v) { r.evaluator->SetProfilingEnabled(v); },
            "When set, compile and evaluate record scoped phase timings.")
        .def_property("solver_guides_enabled",
            [](_Rig &r) { return r.evaluator->GetSolverGuidesEnabled(); },
            [](_Rig &r, bool v) { r.evaluator->SetSolverGuidesEnabled(v); },
            "When set (the default), evaluate fills pose solver guide frames. "
            "Clear it when nothing reads solver_frames to skip the guide request.")
        .def("clear_profile", [](_Rig &r) { r.evaluator->ClearProfile(); },
             "Drops every recorded profile event.")
        .def("write_profile_trace", [](_Rig &r, std::string path) {
                 std::string error;
                 if (!r.evaluator->WriteProfileTrace(path, &error)) {
                     throw py::value_error(error);
                 }
             },
             py::arg("path"),
             "Writes the accumulated phase timings as Chrome Trace Event JSON.")
        .def("profile_summary", [](const _Rig &r) {
            std::vector<py::dict> out;
            for (const rigExec::RigExecProfileSummaryRow &row :
                 r.evaluator->GetProfiler().Summarize()) {
                py::dict d;
                d["name"] = row.name;
                d["category"] = row.category;
                d["count"] = row.count;
                d["total_us"] = row.totalUs;
                d["max_us"] = row.maxUs;
                out.push_back(d);
            }
            return out;
        }, "Per-phase timing totals, sorted by total cost descending.")
        .def("profile_events", [](const _Rig &r) {
            // WHICH THREAD RAN WHAT, which the summary aggregates away and
            // is the only direct evidence that a parallel region actually
            // ran in parallel rather than merely being allowed to. The
            // Chrome trace carries it too, but reading a trace file is not
            // something a report can do in line.
            std::vector<py::dict> out;
            for (const rigExec::RigExecProfileEvent &event :
                 r.evaluator->GetProfiler().GetEvents()) {
                py::dict d;
                d["name"] = event.name;
                d["category"] = event.category;
                d["start_us"] = event.startUs;
                d["duration_us"] = event.durationUs;
                d["thread"] = event.threadIndex;
                out.push_back(d);
            }
            return out;
        }, "Every recorded scope in completion order, with the index of the\n"
           "thread that ran it. Use profile_summary for totals.");

    // ---- Pose ---------------------------------------------------------------

    py::class_<rigExec::RigExecRigPose>(m, "Pose",
        "One evaluated generation of a rig.")
        .def_property_readonly("time", [](const rigExec::RigExecRigPose &p) {
            return p.time.IsDefault() ? -1.0 : p.time.GetValue();
        })
        .def_readonly("valid", &rigExec::RigExecRigPose::valid)
        .def_property_readonly("diagnostics", [](const rigExec::RigExecRigPose &p) {
            return py::cast(p.diagnostics);
        })
        .def_property_readonly("mover_graph_parity_mismatches",
            [](const rigExec::RigExecRigPose &p) { return p.moverGraphParityMismatches; })
        .def_property_readonly("baked_parity_mismatches",
            [](const rigExec::RigExecRigPose &p) { return p.bakedParityMismatches; })
        .def_readonly("mover_graph_revisions_created",
            &rigExec::RigExecRigPose::moverGraphRevisionsCreated)
        .def_readonly("mover_graph_revisions_executed",
            &rigExec::RigExecRigPose::moverGraphRevisionsExecuted)
        .def_readonly("mover_graph_schedules_built",
            &rigExec::RigExecRigPose::moverGraphSchedulesBuilt)
        .def_property_readonly("solver_override_rounds",
            [](const rigExec::RigExecRigPose &p) { return p.solverOverrideRounds; })
        .def_readonly("solver_evaluations", &rigExec::RigExecRigPose::solverEvaluations)
        .def_property_readonly("solver_overrides_converged",
            [](const rigExec::RigExecRigPose &p) { return p.solverOverridesConverged; })

        // Joint frames.
        .def("joint_frame", [](const rigExec::RigExecRigPose &p, std::string path, bool final) {
                const auto &map = final ? p.jointFramesFinal : p.jointFramesBase;
                auto it = map.find(SdfPath(path));
                if (it == map.end()) {
                    throw py::key_error("no joint frame for " + path);
                }
                return it->second;
            }, py::arg("path"), py::arg("final") = true,
           "The point frame of one joint ('base' before pose movers, 'final' after).")
        .def("joint_paths", [](const rigExec::RigExecRigPose &p) {
                std::vector<std::string> out;
                for (const auto &kv : p.jointFramesFinal) {
                    out.push_back(_PathStr(kv.first));
                }
                return out;
            })
        .def("joint_matrix", [](const rigExec::RigExecRigPose &p, std::string path) {
                auto it = p.jointMatricesFinal.find(SdfPath(path));
                if (it == p.jointMatricesFinal.end()) {
                    throw py::key_error("no joint matrix for " + path);
                }
                return _Mat4ToVec(it->second);
            }, py::arg("path"),
           "The final rest-to-pose affine map of one joint (16 numbers, row-major).")

        // Control frames.
        .def("control_paths", [](const rigExec::RigExecRigPose &p) {
                std::vector<std::string> out;
                for (const auto &kv : p.controlFrames) out.push_back(_PathStr(kv.first));
                return out;
            })
        .def("control_frame", [](const rigExec::RigExecRigPose &p, std::string path) {
                auto it = p.controlFrames.find(SdfPath(path));
                if (it == p.controlFrames.end()) {
                    throw py::key_error("no control frame for " + path);
                }
                return it->second;
            }, py::arg("path"))

        // Provider xforms.
        .def("provider_paths", [](const rigExec::RigExecRigPose &p) {
                std::vector<std::string> out;
                for (const auto &kv : p.providerXforms) out.push_back(_PathStr(kv.first));
                return out;
            })
        .def("provider_xform", [](const rigExec::RigExecRigPose &p, std::string path) {
                auto it = p.providerXforms.find(SdfPath(path));
                if (it == p.providerXforms.end()) {
                    throw py::key_error("no provider xform for " + path);
                }
                return _Mat4ToVec(it->second);
            }, py::arg("path"))

        // Solver frames.
        .def("solver_frames", [](const rigExec::RigExecRigPose &p, std::string path) {
                auto it = p.solverFrames.find(SdfPath(path));
                if (it == p.solverFrames.end()) {
                    throw py::key_error("no solver frames for " + path);
                }
                return py::cast(it->second);
            }, py::arg("path"))

        // Moved properties.
        .def("moved_property", [](const rigExec::RigExecRigPose &p, std::string path) {
                auto it = p.movedProperties.find(SdfPath(path));
                if (it == p.movedProperties.end()) {
                    throw py::key_error("no moved property for " + path);
                }
                return _VtToPython(it->second);
            }, py::arg("path"),
           "The final computed value of one moved property, converted by held type.")
        .def("moved_properties", [](const rigExec::RigExecRigPose &p) {
                py::dict out;
                for (const auto &kv : p.movedProperties) {
                    out[py::cast(_PathStr(kv.first))] = _VtToPython(kv.second);
                }
                return out;
            })

        // Weight fields.
        .def("weight_field", [](const rigExec::RigExecRigPose &p, std::string path) {
                auto it = p.weightFields.find(SdfPath(path));
                if (it == p.weightFields.end()) {
                    throw py::key_error("no weight field for " + path);
                }
                return py::cast(it->second.weights);
            }, py::arg("path"),
           "The resolved dense weights of one consumed weight object.")
        .def("weight_frame", [](const rigExec::RigExecRigPose &p, std::string path) {
                auto it = p.weightFrames.find(SdfPath(path));
                if (it == p.weightFrames.end()) {
                    throw py::key_error("no weight frame for " + path);
                }
                return _Mat4ToVec(it->second);
            }, py::arg("path"));

    // ---- Strict low-level schema authoring ---------------------------------

    py::class_<rigExec::RigExecSchemaPrim>(m, "SchemaPrim",
        "A strict authoring view of one registered concrete schema prim.\n\n"
        "Properties are resolved from OpenUSD's composed prim definition; "
        "undeclared names and caller-invented types are rejected.")
        .def_static("define",
            [](py::object stageObj, py::object path,
               const std::string &schemaType) {
                auto stage = _ExtractStage(stageObj);
                if (!stage) {
                    throw py::type_error(
                        "SchemaPrim.define() needs a pxr.Usd.Stage instance");
                }
                return rigExec::RigExecSchemaPrim::Define(
                    std::move(stage), _PythonToPath(path, false),
                    TfToken(schemaType));
            }, py::arg("stage"), py::arg("path"), py::arg("schema_type"))
        .def_static("get",
            [](py::object stageObj, py::object path,
               const std::string &expectedSchemaType) {
                auto stage = _ExtractStage(stageObj);
                if (!stage) {
                    throw py::type_error(
                        "SchemaPrim.get() needs a pxr.Usd.Stage instance");
                }
                return rigExec::RigExecSchemaPrim::Get(
                    std::move(stage), _PythonToPath(path, false),
                    TfToken(expectedSchemaType));
            }, py::arg("stage"), py::arg("path"),
               py::arg("expected_schema_type"))
        .def_property_readonly("valid", &rigExec::RigExecSchemaPrim::IsValid)
        .def_property_readonly("path",
            [](const rigExec::RigExecSchemaPrim &prim) {
                return _PathStr(prim.GetPath());
            })
        .def_property_readonly("schema_type",
            [](const rigExec::RigExecSchemaPrim &prim) {
                return prim.GetSchemaTypeName().GetString();
            })
        .def("has_api",
            [](const rigExec::RigExecSchemaPrim &prim,
               const std::string &schemaIdentifier) {
                return prim.HasAPI(TfToken(schemaIdentifier));
            }, py::arg("schema_identifier"))
        .def("apply_api",
            [](const rigExec::RigExecSchemaPrim &prim,
               const std::string &schemaIdentifier) {
                prim.ApplyAPI(TfToken(schemaIdentifier));
            }, py::arg("schema_identifier"))
        .def("set_attribute",
            [](const rigExec::RigExecSchemaPrim &prim, const std::string &name,
               const py::object &value, const py::object &time) {
                const TfToken token(name);
                UsdTimeCode timeCode = UsdTimeCode::Default();
                if (!time.is_none()) {
                    if (py::hasattr(time, "IsDefault") &&
                        time.attr("IsDefault")().cast<bool>()) {
                        timeCode = UsdTimeCode::Default();
                    } else if (py::hasattr(time, "GetValue")) {
                        timeCode = UsdTimeCode(
                            time.attr("GetValue")().cast<double>());
                    } else {
                        timeCode = UsdTimeCode(time.cast<double>());
                    }
                }
                prim.SetAttribute(
                    token, _PythonToSchemaValue(prim.GetPrim(), token, value),
                    timeCode);
            }, py::arg("name"), py::arg("value"), py::arg("time") = py::none(),
            "Set a declared attribute; its type comes from the composed schema.")
        .def("set_relationship",
            [](const rigExec::RigExecSchemaPrim &prim, const std::string &name,
               const py::iterable &targets) {
                prim.SetRelationship(
                    TfToken(name),
                    _PythonToDependencyPaths(targets, prim.GetStage()));
            }, py::arg("name"), py::arg("targets"),
            "Replace targets on a declared relationship.")
        .def("_validate_relationship_targets",
            [](const rigExec::RigExecSchemaPrim &prim,
               const py::iterable &targets) {
                std::vector<std::string> result;
                for (const SdfPath &path : _PythonToDependencyPaths(
                         targets, prim.GetStage())) {
                    result.push_back(path.GetString());
                }
                return result;
            }, py::arg("targets"))
        .def("clear_attribute",
            [](const rigExec::RigExecSchemaPrim &prim, const std::string &name) {
                prim.ClearAttribute(TfToken(name));
            }, py::arg("name"))
        .def("set_read_phase",
            [](const rigExec::RigExecSchemaPrim &prim,
               const std::string &propertyName, const std::string &phase) {
                prim.SetReadPhase(TfToken(propertyName), phase);
            }, py::arg("property_name"), py::arg("phase"))
        .def("__repr__", [](const rigExec::RigExecSchemaPrim &prim) {
            return "<SchemaPrim " + prim.GetSchemaTypeName().GetString() +
                   " '" + _PathStr(prim.GetPath()) + "'>";
        });

    // ---- Rigging API: handles ----------------------------------------------

    using rigExec::RigExecHandleBase;

    py::class_<RigExecHandleBase>(m, "Handle",
        "Base of every rig object handle. Handles are cheap values naming one prim.")
        .def_property_readonly("valid", &RigExecHandleBase::IsValid)
        .def_property_readonly("path", [](const RigExecHandleBase &h) { return _PathStr(h.GetPath()); })
        .def_property_readonly("name", &RigExecHandleBase::GetName)
        .def("set_attribute",
            [](RigExecHandleBase &h, const std::string &name,
               const py::object &value) {
                const TfToken token(name);
                const UsdPrim prim = h.GetPrim();
                const UsdAttribute attribute = prim.GetAttribute(token);
                if (!attribute) {
                    throw py::key_error(
                        "attribute '" + name +
                        "' is not declared by this handle's composed schema");
                }
                h.SetAttr(
                    name.c_str(), attribute.GetTypeName().GetAsToken(),
                    _PythonToSchemaValue(prim, token, value));
            }, py::arg("name"), py::arg("value"),
            "Set a declared attribute using its OpenUSD schema type."
            " Undeclared names are rejected and never materialized.")
        .def("set_relationship",
            [](RigExecHandleBase &h, const std::string &name,
               const py::iterable &targets) {
                rigExec::RigExecSchemaPrim::Get(
                    h.GetStage(), h.GetPath(), h.GetSchemaTypeName())
                    .SetRelationship(
                        TfToken(name),
                        _PythonToDependencyPaths(targets, h.GetStage()));
            }, py::arg("name"), py::arg("targets"),
            "Replace targets on a declared relationship. Undeclared names "
            "are rejected and never materialized.")
        .def("__repr__", [](const RigExecHandleBase &h) {
            return "<" + std::string(h.GetPrim().GetTypeName().GetString()) + " '" +
                   _PathStr(h.GetPath()) + "'>";
        });

    py::class_<rigExec::RigExecMoverHandle, RigExecHandleBase>(m, "Mover",
        "Common schema-backed handle for operations carrying RigExecMoverAPI.")
        .def("set_enabled", &rigExec::RigExecMoverHandle::SetEnabled,
             py::arg("enabled"))
        .def("set_default_weight",
             &rigExec::RigExecMoverHandle::SetDefaultWeight,
             py::arg("weight"),
             "Set the common normalized mover envelope used without a weight object.")
        .def("set_weight_object",
            [](rigExec::RigExecMoverHandle &h, py::object p) {
                h.SetWeightObject(_PythonToDependencyPath(p, h.GetStage()));
            }, py::arg("path") = py::none(),
            "Bind a target-compatible weight object, or clear it with None.")
        .def("set_moves",
            [](rigExec::RigExecMoverHandle &h, const py::iterable &targets) {
                h.SetMoves(_PythonToDependencyPaths(
                    targets, h.GetStage()));
            }, py::arg("targets"))
        .def("set_read_phase",
            [](rigExec::RigExecMoverHandle &h,
               const std::string &propertyName, const std::string &phase) {
                h.SetReadPhase(TfToken(propertyName), phase);
            }, py::arg("property_name"), py::arg("phase"),
            "Set canonical rigExecReadPhase metadata on a declared input.");

    // Control / joint.
    py::class_<rigExec::RigExecControlHandle, RigExecHandleBase>(m, "Control")
        .def("set_rest_space", [](rigExec::RigExecControlHandle &h, std::vector<double> m) {
            h.SetRestSpace(_VecToMat4(m));
        }, py::arg("matrix"))
        .def("set_avar_translation", &rigExec::RigExecControlHandle::SetAvarTranslation,
             py::arg("tx"), py::arg("ty"), py::arg("tz"))
        .def("set_avar_rotation", [](rigExec::RigExecControlHandle &h, double rx, double ry, double rz, std::string order) {
            h.SetAvarRotation(rx, ry, rz, TfToken(order));
        }, py::arg("rx"), py::arg("ry"), py::arg("rz"), py::arg("order") = "XYZ")
        .def("set_avar_scale", &rigExec::RigExecControlHandle::SetAvarScale,
             py::arg("sx"), py::arg("sy"), py::arg("sz"),
             "Author finite local scale; magnitudes below 1e-4 keep their "
             "sign and are raised to 1e-4.")
        .def("set_avar_spin", &rigExec::RigExecControlHandle::SetAvarSpin,
             py::arg("degrees"))
        .def("set_channel_role", [](rigExec::RigExecControlHandle &h, std::string role) {
            h.SetChannelRole(TfToken(role));
        }, py::arg("role"));

    py::class_<rigExec::RigExecJointHandle, RigExecHandleBase>(m, "Joint")
        .def("set_rest_space", [](rigExec::RigExecJointHandle &h, std::vector<double> m) {
            h.SetRestSpace(_VecToMat4(m));
        }, py::arg("matrix"))
        .def("set_avar_translation", &rigExec::RigExecJointHandle::SetAvarTranslation,
             py::arg("tx"), py::arg("ty"), py::arg("tz"))
        .def("set_avar_rotation", [](rigExec::RigExecJointHandle &h, double rx, double ry, double rz, std::string order) {
            h.SetAvarRotation(rx, ry, rz, TfToken(order));
        }, py::arg("rx"), py::arg("ry"), py::arg("rz"), py::arg("order") = "XYZ")
        .def("set_avar_scale", &rigExec::RigExecJointHandle::SetAvarScale,
             py::arg("sx"), py::arg("sy"), py::arg("sz"),
             "Author finite local scale; magnitudes below 1e-4 keep their "
             "sign and are raised to 1e-4.")
        .def("set_avar_spin", &rigExec::RigExecJointHandle::SetAvarSpin,
             py::arg("degrees"));

    // Solvers.
    py::class_<rigExec::RigExecSolverHandle, RigExecHandleBase>(m, "Solver")
        .def("set_joints", [](rigExec::RigExecSolverHandle &h,
                              const py::iterable &joints) {
            h.SetJoints(_PythonToDependencyPaths(
                joints, h.GetStage(), TfToken("RigExecJoint")));
        }, py::arg("paths"));

    py::class_<rigExec::RigExecFkChainHandle, rigExec::RigExecSolverHandle>(m, "FkChain")
        .def("set_controls", [](rigExec::RigExecFkChainHandle &h,
                                const py::iterable &controls) {
            h.SetControls(_PythonToDependencyPaths(
                controls, h.GetStage(), TfToken("RigExecControl")));
        }, py::arg("paths"))
        .def("set_control_space", [](rigExec::RigExecFkChainHandle &h, std::string v) { h.SetControlSpace(TfToken(v)); }, py::arg("space"),
             "'world' (sibling controls; the solver composes the chain) or "
             "'parentRelative' (controls nested one under the next already "
             "travel with their parent; the solver takes each delta as-is). "
             "Same joints either way.")
        .def("set_start_frame", [](rigExec::RigExecFkChainHandle &h,
                                   py::object p) {
            h.SetStartFrame(_PythonToDependencyPath(p, h.GetStage()));
        }, py::arg("path"),
             "The joint or control the chain HANGS FROM (rigExec:startFrame). "
             "Every element is composed onto that provider's rest-to-pose "
             "delta, so the chain rides it -- in whatever poses the provider, "
             "including another solver. Unauthored, or None/empty to clear, "
             "keeps the historical absolute solve, where a chain whose joints "
             "sit under a solver-posed joint simply does not follow it.")
        .def("set_start_frame_policy", [](rigExec::RigExecFkChainHandle &h, std::string v) { h.SetStartFramePolicy(TfToken(v)); }, py::arg("policy"),
             "'none' (only an authored rigExec:startFrame applies) or "
             "'parent' (the compiler derives the provider from the joint "
             "hierarchy: nearest namespace ancestor of the chain's joints "
             "that is a joint or control). Authored targets always win.");

    py::class_<rigExec::RigExecTwoBoneIkHandle, rigExec::RigExecSolverHandle>(m, "TwoBoneIk")
        .def("set_root_control", [](rigExec::RigExecTwoBoneIkHandle &h, py::object p) {
            h.SetRootControl(_PythonToDependencyPath(p, h.GetStage()));
        }, py::arg("path"))
        .def("set_effector_control", [](rigExec::RigExecTwoBoneIkHandle &h, py::object p) {
            h.SetEffectorControl(_PythonToDependencyPath(p, h.GetStage()));
        }, py::arg("path"))
        .def("set_pole_control", [](rigExec::RigExecTwoBoneIkHandle &h, py::object p) {
            h.SetPoleControl(_PythonToDependencyPath(p, h.GetStage()));
        }, py::arg("path"))
        .def("set_upper_length_offset",
             &rigExec::RigExecTwoBoneIkHandle::SetUpperLengthOffset,
             py::arg("offset"))
        .def("set_lower_length_offset",
             &rigExec::RigExecTwoBoneIkHandle::SetLowerLengthOffset,
             py::arg("offset"))
        .def("set_preferred_bend_radians",
             &rigExec::RigExecTwoBoneIkHandle::SetPreferredBendRadians,
             py::arg("radians"))
        .def("set_stretch", &rigExec::RigExecTwoBoneIkHandle::SetStretch,
             py::arg("stretch"))
        .def("set_softness", &rigExec::RigExecTwoBoneIkHandle::SetSoftness,
             py::arg("softness"))
        .def("set_stretch_policy", [](rigExec::RigExecTwoBoneIkHandle &h, std::string v) { h.SetStretchPolicy(TfToken(v)); }, py::arg("policy"))
        .def("set_unreachable_policy", [](rigExec::RigExecTwoBoneIkHandle &h, std::string v) { h.SetUnreachablePolicy(TfToken(v)); }, py::arg("policy"));

    py::class_<rigExec::RigExecBlendPointFramesHandle, rigExec::RigExecSolverHandle>(m, "BlendPointFrames")
        .def("set_input_a", [](rigExec::RigExecBlendPointFramesHandle &h, py::object p) {
            h.SetInputA(_PythonToDependencyPath(p, h.GetStage(), TfToken(), false));
        }, py::arg("path"))
        .def("set_input_b", [](rigExec::RigExecBlendPointFramesHandle &h, py::object p) {
            h.SetInputB(_PythonToDependencyPath(p, h.GetStage(), TfToken(), false));
        }, py::arg("path"))
        .def("set_weight", &rigExec::RigExecBlendPointFramesHandle::SetWeight, py::arg("weight"))
        .def("set_rotation_blend", [](rigExec::RigExecBlendPointFramesHandle &h, std::string v) { h.SetRotationBlend(TfToken(v)); }, py::arg("mode"))
        .def("set_scale_blend", [](rigExec::RigExecBlendPointFramesHandle &h, std::string v) { h.SetScaleBlend(TfToken(v)); }, py::arg("mode"));

    py::class_<rigExec::RigExecTwistDistributionHandle, rigExec::RigExecSolverHandle>(m, "TwistDistribution")
        .def("set_start", [](rigExec::RigExecTwistDistributionHandle &h, py::object p) {
            h.SetStart(_PythonToDependencyPath(p, h.GetStage(), TfToken(), false));
        }, py::arg("path"))
        .def("set_end", [](rigExec::RigExecTwistDistributionHandle &h, py::object p) {
            h.SetEnd(_PythonToDependencyPath(p, h.GetStage(), TfToken(), false));
        }, py::arg("path"))
        .def("set_count", &rigExec::RigExecTwistDistributionHandle::SetCount, py::arg("count"))
        .def("set_twist_turns", &rigExec::RigExecTwistDistributionHandle::SetTwistTurns, py::arg("turns"))
        .def("set_weights", [](rigExec::RigExecTwistDistributionHandle &h, std::vector<float> w) { h.SetWeights(w); }, py::arg("weights"))
        .def("set_distribution", [](rigExec::RigExecTwistDistributionHandle &h, std::string v) { h.SetDistribution(TfToken(v)); }, py::arg("mode"))
        .def("set_joint_elements", [](rigExec::RigExecTwistDistributionHandle &h, std::vector<int> e) { h.SetJointElements(e); }, py::arg("elements"));

    py::class_<rigExec::RigExecRibbonHandle, rigExec::RigExecSolverHandle>(m, "Ribbon")
        .def("set_driver_curve", [](rigExec::RigExecRibbonHandle &h, py::object p) {
            h.SetDriverCurve(_PythonToDependencyPath(p, h.GetStage(), TfToken(), false));
        }, py::arg("path"))
        .def("set_start_frame", [](rigExec::RigExecRibbonHandle &h, py::object p) {
            h.SetStartFrame(_PythonToDependencyPath(p, h.GetStage()));
        }, py::arg("path"))
        .def("set_end_frame", [](rigExec::RigExecRibbonHandle &h, py::object p) {
            h.SetEndFrame(_PythonToDependencyPath(p, h.GetStage()));
        }, py::arg("path"))
        .def("set_twist_frames", [](rigExec::RigExecRibbonHandle &h,
                                      const py::iterable &paths) {
            h.SetTwistFrames(_PythonToDependencyPaths(paths, h.GetStage()));
        }, py::arg("paths"))
        .def("set_sample_count", &rigExec::RigExecRibbonHandle::SetSampleCount, py::arg("count"))
        .def("set_parameterization", [](rigExec::RigExecRibbonHandle &h, std::string v) { h.SetParameterization(TfToken(v)); }, py::arg("mode"))
        .def("set_driver_curve_read_phase", [](rigExec::RigExecRibbonHandle &h, std::string v) { h.SetDriverCurveReadPhase(TfToken(v)); }, py::arg("phase"))
        .def("set_surface_read_phase", [](rigExec::RigExecRibbonHandle &h, std::string v) { h.SetSurfaceReadPhase(TfToken(v)); }, py::arg("phase"))
        .def("set_joint_elements", [](rigExec::RigExecRibbonHandle &h, std::vector<int> e) { h.SetJointElements(e); }, py::arg("elements"));

    py::class_<rigExec::RigExecSplineIkHandle, rigExec::RigExecSolverHandle>(m, "SplineIk",
        "Control-driven spline IK: root/mid/end controls shape a degree-2 "
        "B-spline and the ordered joint chain is laid along it by arc length.")
        .def("set_root_control", [](rigExec::RigExecSplineIkHandle &h, py::object p) {
            h.SetRootControl(_PythonToDependencyPath(p, h.GetStage(), TfToken("RigExecControl"), false));
        }, py::arg("path"))
        .def("set_mid_control", [](rigExec::RigExecSplineIkHandle &h, py::object p) {
            h.SetMidControl(_PythonToDependencyPath(p, h.GetStage(), TfToken("RigExecControl"), false));
        }, py::arg("path"))
        .def("set_end_control", [](rigExec::RigExecSplineIkHandle &h, py::object p) {
            h.SetEndControl(_PythonToDependencyPath(p, h.GetStage(), TfToken("RigExecControl"), false));
        }, py::arg("path"))
        .def("set_volume_weights", [](rigExec::RigExecSplineIkHandle &h, std::vector<float> w) { h.SetVolumeWeights(w); }, py::arg("weights"),
             "Per-joint squash/stretch weights parallel to the joints; empty means no thinning.")
        .def("set_rest_length", [](rigExec::RigExecSplineIkHandle &h, std::string v) { h.SetRestLength(TfToken(v)); }, py::arg("mode"),
             "'curve' (ratio == 1 at rest) or 'chain' (chain spans the curve exactly).")
        .def("set_preserve_volume", &rigExec::RigExecSplineIkHandle::SetPreserveVolume, py::arg("amount"))
        .def("set_mid_follow_weight", &rigExec::RigExecSplineIkHandle::SetMidFollowWeight, py::arg("weight"))
        .def("set_roll", &rigExec::RigExecSplineIkHandle::SetRoll, py::arg("degrees"))
        .def("set_twist", &rigExec::RigExecSplineIkHandle::SetTwist, py::arg("degrees"))
        .def("set_min_length_ratio", &rigExec::RigExecSplineIkHandle::SetMinLengthRatio, py::arg("ratio"),
             "Length floor as a fraction of the rest root->end chord (inputs:minLengthRatio); 0 = off.")
        .def("set_root_tangent", [](rigExec::RigExecSplineIkHandle &h, std::string v) { h.SetRootTangent(TfToken(v)); }, py::arg("mode"),
             "'rigid' carries cv1 with the root control; 'aim' turns it onto the chord to the (floored) end (rigExec:rootTangent).")
        .def("set_joint_elements", [](rigExec::RigExecSplineIkHandle &h, std::vector<int> e) { h.SetJointElements(e); }, py::arg("elements"));

    // Constraints.
    py::class_<rigExec::RigExecConstraintHandle, rigExec::RigExecMoverHandle>(m, "Constraint")
        .def("set_target", [](rigExec::RigExecConstraintHandle &h, py::object p) {
            h.SetTarget(_PythonToDependencyPath(p, h.GetStage(), TfToken(), false));
        }, py::arg("path"))
        .def("set_locked", &rigExec::RigExecConstraintHandle::SetLocked,
             py::arg("locked"));

    py::class_<rigExec::RigExecSourceConstraintHandle, rigExec::RigExecConstraintHandle>(m, "SourceConstraint")
        .def("set_sources", [](rigExec::RigExecSourceConstraintHandle &h,
                               const py::iterable &sources,
                               const py::object &weights) {
            const SdfPathVector paths =
                _PythonToDependencyPaths(sources, h.GetStage());
            if (weights.is_none()) {
                h.SetSources(paths);
            } else {
                h.SetSources(paths, weights.cast<std::vector<float>>());
            }
        }, py::arg("paths"), py::arg("weights") = py::none(),
           "Replace ordered sources and, optionally, their parallel weights. "
           "Omitting weights clears any previously authored source weights.")
        .def("set_source_weights",
             &rigExec::RigExecSourceConstraintHandle::SetSourceWeights,
             py::arg("weights"));

    py::class_<rigExec::RigExecAimConstraintHandle, rigExec::RigExecSourceConstraintHandle>(m, "AimConstraint")
        .def("set_affect_rotation", &rigExec::RigExecAimConstraintHandle::SetAffectRotation,
             py::arg("x"), py::arg("y"), py::arg("z"))
        .def("set_rotation_offset", &rigExec::RigExecAimConstraintHandle::SetRotationOffset,
             py::arg("x"), py::arg("y"), py::arg("z"))
        .def("set_rotation_order", [](rigExec::RigExecAimConstraintHandle &h, std::string v) {
            h.SetRotationOrder(TfToken(v));
        }, py::arg("order"))
        .def("set_aim_vector", &rigExec::RigExecAimConstraintHandle::SetAimVector,
             py::arg("x"), py::arg("y"), py::arg("z"))
        .def("set_up_vector", &rigExec::RigExecAimConstraintHandle::SetUpVector,
             py::arg("x"), py::arg("y"), py::arg("z"))
        .def("set_world_up_vector", &rigExec::RigExecAimConstraintHandle::SetWorldUpVector,
             py::arg("x"), py::arg("y"), py::arg("z"))
        .def("set_aim_target", [](rigExec::RigExecAimConstraintHandle &h, py::object p) {
            h.SetAimTarget(_PythonToDependencyPath(p, h.GetStage()));
        }, py::arg("path"))
        .def("set_world_up_object", [](rigExec::RigExecAimConstraintHandle &h, py::object p) {
            h.SetWorldUpObject(_PythonToDependencyPath(p, h.GetStage()));
        }, py::arg("path"))
        .def("set_world_up_type", [](rigExec::RigExecAimConstraintHandle &h, std::string v) { h.SetWorldUpType(TfToken(v)); }, py::arg("type"));

    py::class_<rigExec::RigExecPositionConstraintHandle, rigExec::RigExecSourceConstraintHandle>(m, "PositionConstraint")
        .def("set_affect_translation", &rigExec::RigExecPositionConstraintHandle::SetAffectTranslation,
             py::arg("x"), py::arg("y"), py::arg("z"))
        .def("set_translation_offset", &rigExec::RigExecPositionConstraintHandle::SetTranslationOffset,
             py::arg("x"), py::arg("y"), py::arg("z"));
    py::class_<rigExec::RigExecRotationConstraintHandle, rigExec::RigExecSourceConstraintHandle>(m, "RotationConstraint")
        .def("set_affect_rotation", &rigExec::RigExecRotationConstraintHandle::SetAffectRotation,
             py::arg("x"), py::arg("y"), py::arg("z"))
        .def("set_rotation_offset", &rigExec::RigExecRotationConstraintHandle::SetRotationOffset,
             py::arg("x"), py::arg("y"), py::arg("z"))
        .def("set_rotation_order", [](rigExec::RigExecRotationConstraintHandle &h, std::string v) {
            h.SetRotationOrder(TfToken(v));
        }, py::arg("order"));
    py::class_<rigExec::RigExecScaleConstraintHandle, rigExec::RigExecSourceConstraintHandle>(m, "ScaleConstraint")
        .def("set_affect_scale", &rigExec::RigExecScaleConstraintHandle::SetAffectScale,
             py::arg("x"), py::arg("y"), py::arg("z"))
        .def("set_scale_offset", &rigExec::RigExecScaleConstraintHandle::SetScaleOffset,
             py::arg("x"), py::arg("y"), py::arg("z"));
    py::class_<rigExec::RigExecParentConstraintHandle, rigExec::RigExecSourceConstraintHandle>(m, "ParentConstraint")
        .def("set_affect_translation", &rigExec::RigExecParentConstraintHandle::SetAffectTranslation,
             py::arg("x"), py::arg("y"), py::arg("z"))
        .def("set_affect_rotation", &rigExec::RigExecParentConstraintHandle::SetAffectRotation,
             py::arg("x"), py::arg("y"), py::arg("z"))
        .def("set_affect_scale", &rigExec::RigExecParentConstraintHandle::SetAffectScale,
             py::arg("x"), py::arg("y"), py::arg("z"))
        .def("set_rotation_order", [](rigExec::RigExecParentConstraintHandle &h, std::string v) {
            h.SetRotationOrder(TfToken(v));
        }, py::arg("order"))
        .def("set_translation_offsets",
             [](rigExec::RigExecParentConstraintHandle &h,
                std::vector<std::array<double, 3>> v) {
                 std::vector<GfVec3d> o;
                 for (const auto &t : v) {
                     o.emplace_back(t[0], t[1], t[2]);
                 }
                 h.SetTranslationOffsets(o);
             }, py::arg("offsets"))
        .def("set_rotation_offsets",
             [](rigExec::RigExecParentConstraintHandle &h,
                std::vector<std::array<double, 3>> v) {
                 std::vector<GfVec3d> o;
                 for (const auto &t : v) {
                     o.emplace_back(t[0], t[1], t[2]);
                 }
                 h.SetRotationOffsets(o);
             }, py::arg("degrees"));

    py::class_<rigExec::RigExecSingleChainIkConstraintHandle, rigExec::RigExecConstraintHandle>(m, "SingleChainIkConstraint")
        .def("set_first_joint", [](rigExec::RigExecSingleChainIkConstraintHandle &h, py::object p) {
            h.SetFirstJoint(_PythonToDependencyPath(p, h.GetStage(), TfToken("RigExecJoint"), false));
        }, py::arg("path"))
        .def("set_end_joint", [](rigExec::RigExecSingleChainIkConstraintHandle &h, py::object p) {
            h.SetEndJoint(_PythonToDependencyPath(p, h.GetStage(), TfToken("RigExecJoint"), false));
        }, py::arg("path"))
        .def("set_effector", [](rigExec::RigExecSingleChainIkConstraintHandle &h, py::object p) {
            h.SetEffector(_PythonToDependencyPath(p, h.GetStage(), TfToken(), false));
        }, py::arg("path"))
        .def("set_moves", [](rigExec::RigExecSingleChainIkConstraintHandle &h,
                              const py::iterable &paths) {
            h.SetMoves(_PythonToDependencyPaths(
                paths, h.GetStage(), TfToken("RigExecJoint")));
        }, py::arg("paths"))
        .def("set_pole_vector_objects", [](rigExec::RigExecSingleChainIkConstraintHandle &h,
                                            const py::iterable &paths) {
            h.SetPoleVectorObjects(_PythonToDependencyPaths(paths, h.GetStage()));
        }, py::arg("paths"))
        .def("set_pole_vector_weights", &rigExec::RigExecSingleChainIkConstraintHandle::SetPoleVectorWeights,
             py::arg("weights"))
        .def("set_pole_vector", &rigExec::RigExecSingleChainIkConstraintHandle::SetPoleVector,
             py::arg("x"), py::arg("y"), py::arg("z"))
        .def("set_twist_degrees", &rigExec::RigExecSingleChainIkConstraintHandle::SetTwistDegrees,
             py::arg("degrees"))
        .def("set_solver_mode", [](rigExec::RigExecSingleChainIkConstraintHandle &h, std::string v) { h.SetSolverMode(TfToken(v)); }, py::arg("mode"))
        .def("set_pole_vector_mode", [](rigExec::RigExecSingleChainIkConstraintHandle &h, std::string v) { h.SetPoleVectorMode(TfToken(v)); }, py::arg("mode"))
        .def("set_evaluation_mode", [](rigExec::RigExecSingleChainIkConstraintHandle &h, std::string v) { h.SetEvaluationMode(TfToken(v)); }, py::arg("mode"));

    // Weight objects.
    py::class_<rigExec::RigExecWeightHandle, RigExecHandleBase>(m, "Weight")
        .def("set_target", [](rigExec::RigExecWeightHandle &h, py::object p) {
            h.SetTarget(_PythonToDependencyPath(p, h.GetStage(), TfToken(), false));
        }, py::arg("path"))
        .def("set_representation", [](rigExec::RigExecWeightHandle &h, std::string v) { h.SetRepresentation(TfToken(v)); }, py::arg("rep"))
        .def("set_range_policy", [](rigExec::RigExecWeightHandle &h, std::string v) { h.SetRangePolicy(TfToken(v)); }, py::arg("policy"));

    py::class_<rigExec::RigExecStaticWeightHandle, rigExec::RigExecWeightHandle>(m, "StaticWeight")
        .def("set_values", [](rigExec::RigExecStaticWeightHandle &h, std::vector<float> v) { h.SetValues(v); }, py::arg("values"))
        .def("set_indices", [](rigExec::RigExecStaticWeightHandle &h, std::vector<int> i) { h.SetIndices(i); }, py::arg("indices"))
        .def("set_sparse_values", &rigExec::RigExecStaticWeightHandle::SetSparseValues,
             py::arg("values"), py::arg("indices"))
        .def("set_default_weight", &rigExec::RigExecStaticWeightHandle::SetDefaultWeight, py::arg("weight"));

    py::class_<rigExec::RigExecDynamicWeightHandle, rigExec::RigExecWeightHandle>(m, "DynamicWeight")
        .def("set_base_weight", [](rigExec::RigExecDynamicWeightHandle &h, py::object p) {
            h.SetBaseWeight(_PythonToDependencyPath(p, h.GetStage()));
        }, py::arg("path"))
        .def("set_driver", &rigExec::RigExecDynamicWeightHandle::SetDriver, py::arg("driver"))
        .def("set_scale", &rigExec::RigExecDynamicWeightHandle::SetScale, py::arg("scale"))
        .def("set_bias", &rigExec::RigExecDynamicWeightHandle::SetBias, py::arg("bias"));

    py::class_<rigExec::RigExecVolumeWeightHandle, rigExec::RigExecWeightHandle>(m, "VolumeWeight")
        .def("set_rest_space", [](rigExec::RigExecVolumeWeightHandle &h, std::vector<double> m) { h.SetRestSpace(_VecToMat4(m)); }, py::arg("matrix"))
        .def("set_avar_translation", &rigExec::RigExecVolumeWeightHandle::SetAvarTranslation,
             py::arg("tx"), py::arg("ty"), py::arg("tz"))
        .def("set_avar_rotation", [](rigExec::RigExecVolumeWeightHandle &h, double rx, double ry, double rz, std::string order) {
            h.SetAvarRotation(rx, ry, rz, TfToken(order));
        }, py::arg("rx"), py::arg("ry"), py::arg("rz"), py::arg("order") = "XYZ")
        .def("set_avar_spin", &rigExec::RigExecVolumeWeightHandle::SetAvarSpin,
             py::arg("degrees"))
        .def("set_falloff", &rigExec::RigExecVolumeWeightHandle::SetFalloff,
             py::arg("falloff_min"), py::arg("falloff_max"))
        .def("set_invert", &rigExec::RigExecVolumeWeightHandle::SetInvert, py::arg("invert"))
        .def("set_strength", &rigExec::RigExecVolumeWeightHandle::SetStrength, py::arg("strength"))
        .def("set_falloff_profile", [](rigExec::RigExecVolumeWeightHandle &h, std::string v) { h.SetFalloffProfile(TfToken(v)); }, py::arg("profile"))
        .def("set_falloff_curve", [](rigExec::RigExecVolumeWeightHandle &h, std::vector<std::pair<double, double>> knots) {
            h.SetFalloffCurve(knots);
        }, py::arg("knots"), "A list of (x, y) pairs over x in [0, 1].")
        .def("set_sample_phase", [](rigExec::RigExecVolumeWeightHandle &h, std::string v) { h.SetSamplePhase(TfToken(v)); }, py::arg("phase"))
        .def("set_sample_source", [](rigExec::RigExecVolumeWeightHandle &h, py::object p) {
            h.SetSampleSource(_PythonToDependencyPath(p, h.GetStage()));
        }, py::arg("path"));

    py::class_<rigExec::RigExecSphereWeightHandle, rigExec::RigExecVolumeWeightHandle>(m, "SphereWeight")
        .def("set_scales", &rigExec::RigExecSphereWeightHandle::SetScales,
             py::arg("sx"), py::arg("sy"), py::arg("sz"));

    py::class_<rigExec::RigExecPlaneWeightHandle, rigExec::RigExecVolumeWeightHandle>(m, "PlaneWeight")
        .def("set_axis", [](rigExec::RigExecPlaneWeightHandle &h, std::string v) { h.SetAxis(TfToken(v)); }, py::arg("axis"))
        .def("set_bounds", [](rigExec::RigExecPlaneWeightHandle &h, std::string v) { h.SetBounds(TfToken(v)); }, py::arg("bounds"))
        .def("set_extents", &rigExec::RigExecPlaneWeightHandle::SetExtents,
             py::arg("extent_u"), py::arg("extent_v"));

    py::class_<rigExec::RigExecCurveWeightHandle, rigExec::RigExecVolumeWeightHandle>(m, "CurveWeight")
        .def("set_curve", [](rigExec::RigExecCurveWeightHandle &h, py::object p) {
            h.SetCurve(_PythonToDependencyPath(p, h.GetStage(), TfToken(), false));
        }, py::arg("path"))
        .def("set_scales", &rigExec::RigExecCurveWeightHandle::SetScales,
             py::arg("sx"), py::arg("sy"), py::arg("sz"));

    py::class_<rigExec::RigExecCombineWeightHandle, rigExec::RigExecWeightHandle>(m, "CombineWeight")
        .def("set_input_weights", [](rigExec::RigExecCombineWeightHandle &h,
                                      const py::iterable &paths) {
            h.SetInputWeights(_PythonToDependencyPaths(paths, h.GetStage()));
        }, py::arg("paths"))
        .def("set_combine_mode", [](rigExec::RigExecCombineWeightHandle &h, std::string v) { h.SetCombineMode(TfToken(v)); }, py::arg("mode"))
        .def("set_strength", &rigExec::RigExecCombineWeightHandle::SetStrength, py::arg("strength"))
        .def("set_invert", &rigExec::RigExecCombineWeightHandle::SetInvert, py::arg("invert"));

    // Blend inputs / samples.
    py::class_<rigExec::RigExecBlendSampleHandle, RigExecHandleBase>(m, "BlendSample")
        .def("set_activation", &rigExec::RigExecBlendSampleHandle::SetActivation, py::arg("activation"))
        .def("set_target_points", [](rigExec::RigExecBlendSampleHandle &h, py::object p) {
            h.SetTargetPoints(_PythonToDependencyPath(p, h.GetStage(), TfToken(), false));
        }, py::arg("path"))
        .def("set_blend_shape", [](rigExec::RigExecBlendSampleHandle &h, py::object p) {
            h.SetBlendShape(_PythonToDependencyPath(p, h.GetStage(), TfToken("BlendShape"), false));
        }, py::arg("path"),
           "Point this sample at a UsdSkelBlendShape whose offsets (and\n"
           "pointIndices, when non-empty) ARE its deltas. The sparse\n"
           "alternative to set_target_points, and mutually exclusive\n"
           "with it.")
        .def("set_read_phase", [](rigExec::RigExecBlendSampleHandle &h, std::string v) { h.SetReadPhase(TfToken(v)); }, py::arg("phase"));

    py::class_<rigExec::RigExecBlendInputHandle, RigExecHandleBase>(m, "BlendInput")
        .def("set_weight", &rigExec::RigExecBlendInputHandle::SetWeight, py::arg("weight"))
        .def("connect_weight", [](rigExec::RigExecBlendInputHandle &h, py::object p) {
            h.ConnectWeight(_PythonToPath(p, true));
        }, py::arg("output"),
           "Drive inputs:weight from a Pose's outputs:weight instead of an\n"
           "authored number. Takes the property path pose.weight_output()\n"
           "returns; None removes the connection.")
        .def("add_sample", [](rigExec::RigExecBlendInputHandle &h, std::string name, float activation) {
            return h.AddSample(name, activation);
        }, py::arg("name"), py::arg("activation") = 1.0f);

    // Pose interpolators (the conventional poseInterpolator; the maths is
    // libs/rigExecMath/rbf.h). Authoring only: the solved matrix is not
    // stored, because it is a function of the poses, the radii, the kernel
    // and the regularization, every one of which is authored.
    // "InterpolatorPose" and not "Pose": the module already binds Pose for
    // an evaluated rig pose, and pybind answers a duplicate type name by
    // refusing to initialize the module at all.
    py::class_<rigExec::RigExecPoseHandle, RigExecHandleBase>(
        m, "InterpolatorPose")
        .def("set_pose_type", [](rigExec::RigExecPoseHandle &h, std::string v) {
            h.SetPoseType(TfToken(v));
        }, py::arg("pose_type"),
           "swing | twist | whole -- WHICH PART of the driver's rotation this\n"
           "pose is measured against. poseType, and a property of the\n"
           "pose rather than the interpolator: a neck that has twisted has\n"
           "not bent, and its bend shapes should stay at zero.")
        .def("set_rotation", [](rigExec::RigExecPoseHandle &h,
                                std::array<double, 4> q) {
            // (real, imaginary), the order GfQuatf takes -- NOT the conventional tool's
            // [x, y, z, w], which the converter reorders on the way in.
            h.SetRotation(GfQuatf(float(q[0]),
                                  GfVec3f(float(q[1]), float(q[2]),
                                          float(q[3]))));
        }, py::arg("quaternion"),
           "The driver's local rotation in this pose as (real, i, j, k),\n"
           "REBASED with the neutral taken out.")
        .def("set_translation", [](rigExec::RigExecPoseHandle &h,
                                   std::array<double, 3> t) {
            h.SetTranslation(GfVec3f(float(t[0]), float(t[1]), float(t[2])));
        }, py::arg("translation"))
        .def("set_radii", &rigExec::RigExecPoseHandle::SetRadii,
             py::arg("rotation_radius"), py::arg("translation_radius") = 0.0f,
             "The pose's own falloff widths: radians and centimetres. Zero\n"
             "means measure one from the poses.")
        .def("set_falloff", &rigExec::RigExecPoseHandle::SetFalloff,
             py::arg("falloff"))
        .def("set_pose_controls", [](rigExec::RigExecPoseHandle &h,
                                     py::object properties,
                                     std::vector<double> values) {
            std::vector<SdfPath> paths;
            for (const py::handle &item : py::iterable(properties)) {
                paths.push_back(_PythonToPath(item, false));
            }
            h.SetPoseControls(paths, values);
        }, py::arg("properties"), py::arg("values"),
           "The exact control PROPERTIES that put the rig into this pose and\n"
           "their values, in OUR units and from OUR zero (degrees,\n"
           "centimetres). Parallel arrays; authoring data only.")
        .def("set_enabled", &rigExec::RigExecPoseHandle::SetEnabled,
             py::arg("enabled"))
        .def("weight_output", [](const rigExec::RigExecPoseHandle &h) {
            return h.GetWeightOutput().GetString();
        }, "The outputs:weight property path, which is what a blend input's\n"
           "inputs:weight connects to.");

    py::class_<rigExec::RigExecPoseInterpolatorHandle, RigExecHandleBase>(
        m, "PoseInterpolator")
        .def("set_driver", [](rigExec::RigExecPoseInterpolatorHandle &h,
                              py::object p) {
            h.SetDriver(_PythonToDependencyPath(p, h.GetStage(), TfToken(),
                                                false));
        }, py::arg("driver"))
        .def("set_kernel", [](rigExec::RigExecPoseInterpolatorHandle &h,
                              std::string v) {
            h.SetKernel(TfToken(v));
        }, py::arg("kernel"), "gaussian | linear (interpolation).")
        .def("set_channels", &rigExec::RigExecPoseInterpolatorHandle::SetChannels,
             py::arg("enable_rotation"), py::arg("enable_translation"))
        .def("set_allow_negative_weights",
             &rigExec::RigExecPoseInterpolatorHandle::SetAllowNegativeWeights,
             py::arg("allow"))
        .def("set_normalize",
             &rigExec::RigExecPoseInterpolatorHandle::SetNormalize,
             py::arg("normalize"))
        .def("set_regularization",
             &rigExec::RigExecPoseInterpolatorHandle::SetRegularization,
             py::arg("regularization"))
        .def("set_twist_axis", [](rigExec::RigExecPoseInterpolatorHandle &h,
                                  std::string v) {
            h.SetTwistAxis(TfToken(v));
        }, py::arg("axis"), "X | Y | Z, in the driver's own frame.")
        .def("set_enabled",
             &rigExec::RigExecPoseInterpolatorHandle::SetEnabled,
             py::arg("enabled"))
        .def("add_pose", [](rigExec::RigExecPoseInterpolatorHandle &h,
                            std::string name) {
            return h.AddPose(name);
        }, py::arg("name"));

    py::class_<rigExec::RigExecCurvenetAdjustmentHandle, rigExec::RigExecControlHandle>(m, "CurvenetAdjustment")
        .def("set_curvenet", [](rigExec::RigExecCurvenetAdjustmentHandle &h, py::object p) {
            h.SetCurvenet(_PythonToDependencyPath(p,h.GetStage(),TfToken("RigExecCurvenet"),false));
        }, py::arg("curvenet"))
        .def("set_knot_index", &rigExec::RigExecCurvenetAdjustmentHandle::SetKnotIndex, py::arg("index"))
        .def("set_include_tangents", &rigExec::RigExecCurvenetAdjustmentHandle::SetIncludeTangents, py::arg("include"))
        .def("add_tangent", &rigExec::RigExecCurvenetAdjustmentHandle::AddTangent, py::arg("name"), py::arg("index"));

    // Curvenet.
    py::class_<rigExec::RigExecCurvenetHandle, RigExecHandleBase>(m, "Curvenet")
        .def("set_points", [](rigExec::RigExecCurvenetHandle &h, std::vector<std::array<double, 3>> pts) {
            std::vector<GfVec3f> v;
            for (const auto &p : pts) {
                v.push_back(GfVec3f(float(p[0]), float(p[1]), float(p[2])));
            }
            h.SetPoints(v);
        }, py::arg("points"))
        .def("add_spline", &rigExec::RigExecCurvenetHandle::AddSpline,
             py::arg("p0"), py::arg("h0"), py::arg("h1"), py::arg("p1"))
        .def("set_basis", [](rigExec::RigExecCurvenetHandle &h, std::string v) { h.SetBasis(TfToken(v)); }, py::arg("basis"))
        .def("set_samples_per_spline", &rigExec::RigExecCurvenetHandle::SetSamplesPerSpline, py::arg("count"));

    // Mover handles.
    py::class_<rigExec::RigExecMatrixMoverHandle, rigExec::RigExecMoverHandle>(m, "MatrixMover")
        .def("set_transform_provider", [](rigExec::RigExecMatrixMoverHandle &h, py::object p) { h.SetTransformProvider(_PythonToDependencyPath(p, h.GetStage(), TfToken(), false)); }, py::arg("path"))
        .def("set_read_phase", [](rigExec::RigExecMatrixMoverHandle &h,
                                    std::string phase) {
            h.SetReadPhase(TfToken(phase));
        }, py::arg("phase"), "Legacy transform read-phase attribute setter.")
        .def("set_read_phase", [](rigExec::RigExecMatrixMoverHandle &h,
                                    std::string property, std::string phase) {
            h.RigExecMoverHandle::SetReadPhase(
                TfToken(property), phase);
        }, py::arg("property_name"), py::arg("phase"))
        .def("set_transform_read_phase", [](rigExec::RigExecMatrixMoverHandle &h, std::string v) { h.SetReadPhase(TfToken(v)); }, py::arg("phase"));

    py::class_<rigExec::RigExecSkinMoverHandle, rigExec::RigExecMoverHandle>(m, "SkinMover",
        "Multi-influence skinning in one pass over an ordered influence list\n"
        "(UsdSkel's jointIndices / jointWeights layout).")
        .def("set_influences", [](rigExec::RigExecSkinMoverHandle &h, py::iterable providers) {
            std::vector<SdfPath> paths;
            for (const auto &p : providers) paths.push_back(_PythonToDependencyPath(
                py::reinterpret_borrow<py::object>(p), h.GetStage(), TfToken(), false));
            h.SetInfluences(paths);
        }, py::arg("providers"))
        .def("set_joint_influences", [](rigExec::RigExecSkinMoverHandle &h,
                                         std::vector<int> indices,
                                         std::vector<float> weights, int elementSize) {
            h.SetJointInfluences(indices, weights, elementSize);
        }, py::arg("joint_indices"), py::arg("joint_weights"), py::arg("element_size"))
        .def("set_skinning_method", [](rigExec::RigExecSkinMoverHandle &h, std::string v) { h.SetSkinningMethod(TfToken(v)); }, py::arg("method"))
        .def("set_transform_read_phase", [](rigExec::RigExecSkinMoverHandle &h, std::string v) { h.SetReadPhase(TfToken(v)); }, py::arg("phase"));

    py::class_<rigExec::RigExecLatticeMoverHandle, rigExec::RigExecMoverHandle>(m, "LatticeMover")
        .def("set_cage", [](rigExec::RigExecLatticeMoverHandle &h, py::object p) { h.SetCage(_PythonToDependencyPath(p, h.GetStage(), TfToken(), false)); }, py::arg("path"))
        .def("set_basis", [](rigExec::RigExecLatticeMoverHandle &h, std::string v) { h.SetBasis(TfToken(v)); }, py::arg("basis"))
        .def("set_divisions", &rigExec::RigExecLatticeMoverHandle::SetDivisions,
             py::arg("x"), py::arg("y"), py::arg("z"))
        .def("set_read_phase", [](rigExec::RigExecLatticeMoverHandle &h,
                                    std::string phase) {
            h.SetReadPhase(TfToken(phase));
        }, py::arg("phase"), "Legacy cage read-phase attribute setter.")
        .def("set_read_phase", [](rigExec::RigExecLatticeMoverHandle &h,
                                    std::string property, std::string phase) {
            h.RigExecMoverHandle::SetReadPhase(
                TfToken(property), phase);
        }, py::arg("property_name"), py::arg("phase"))
        .def("set_cage_read_phase", [](rigExec::RigExecLatticeMoverHandle &h, std::string v) { h.SetReadPhase(TfToken(v)); }, py::arg("phase"));

    py::class_<rigExec::RigExecBlendShapeMoverHandle, rigExec::RigExecMoverHandle>(m, "BlendShapeMover")
        .def("add_blend_input", [](rigExec::RigExecBlendShapeMoverHandle &h, std::string name, float weight) {
            return h.AddBlendInput(name, weight);
        }, py::arg("name"), py::arg("weight") = 0.0f)
        .def("set_blend_inputs", [](rigExec::RigExecBlendShapeMoverHandle &h,
                                     const py::iterable &inputs) {
            h.SetBlendInputs(_PythonToDependencyPaths(
                inputs, h.GetStage(), TfToken("RigExecBlendInput")));
        }, py::arg("inputs"));

    py::class_<rigExec::RigExecCurveMoverHandle, rigExec::RigExecMoverHandle>(m, "CurveMover")
        .def("set_driver_curve", [](rigExec::RigExecCurveMoverHandle &h, py::object p) { h.SetDriverCurve(_PythonToDependencyPath(p, h.GetStage(), TfToken(), false)); }, py::arg("path"))
        .def("set_driver_frames", [](rigExec::RigExecCurveMoverHandle &h,
                                      const py::iterable &paths) {
            h.SetDriverFrames(_PythonToDependencyPaths(paths, h.GetStage()));
        }, py::arg("paths"))
        .def("set_bind_coordinates", [](rigExec::RigExecCurveMoverHandle &h, py::object p) { h.SetBindCoordinates(_PythonToDependencyPath(p, h.GetStage())); }, py::arg("path"))
        .def("set_mode", [](rigExec::RigExecCurveMoverHandle &h, std::string v) { h.SetMode(TfToken(v)); }, py::arg("mode"))
        .def("set_read_phase", [](rigExec::RigExecCurveMoverHandle &h,
                                    std::string phase) {
            h.SetReadPhase(TfToken(phase));
        }, py::arg("phase"), "Legacy driver-curve read-phase attribute setter.")
        .def("set_read_phase", [](rigExec::RigExecCurveMoverHandle &h,
                                    std::string property, std::string phase) {
            h.RigExecMoverHandle::SetReadPhase(
                TfToken(property), phase);
        }, py::arg("property_name"), py::arg("phase"))
        .def("set_driver_curve_read_phase", [](rigExec::RigExecCurveMoverHandle &h, std::string v) { h.SetReadPhase(TfToken(v)); }, py::arg("phase"));

    py::class_<rigExec::RigExecSurfaceMoverHandle, rigExec::RigExecMoverHandle>(m, "SurfaceMover")
        .def("set_surface", [](rigExec::RigExecSurfaceMoverHandle &h, py::object p) { h.SetSurface(_PythonToDependencyPath(p, h.GetStage(), TfToken(), false)); }, py::arg("path"))
        .def("set_mode", [](rigExec::RigExecSurfaceMoverHandle &h, std::string v) { h.SetMode(TfToken(v)); }, py::arg("mode"))
        .def("set_read_phase", [](rigExec::RigExecSurfaceMoverHandle &h,
                                    std::string phase) {
            h.SetReadPhase(TfToken(phase));
        }, py::arg("phase"), "Legacy surface read-phase attribute setter.")
        .def("set_read_phase", [](rigExec::RigExecSurfaceMoverHandle &h,
                                    std::string property, std::string phase) {
            h.RigExecMoverHandle::SetReadPhase(
                TfToken(property), phase);
        }, py::arg("property_name"), py::arg("phase"))
        .def("set_surface_read_phase", [](rigExec::RigExecSurfaceMoverHandle &h, std::string v) { h.SetReadPhase(TfToken(v)); }, py::arg("phase"));

    py::class_<rigExec::RigExecSmoothMoverHandle, rigExec::RigExecMoverHandle>(m, "SmoothMover")
        .def("set_strength", &rigExec::RigExecSmoothMoverHandle::SetStrength, py::arg("strength"));

    py::class_<rigExec::RigExecVolumeCorrectMoverHandle, rigExec::RigExecMoverHandle>(m, "VolumeCorrectMover")
        .def("set_strength", &rigExec::RigExecVolumeCorrectMoverHandle::SetStrength, py::arg("strength"));

    py::class_<rigExec::RigExecCurvenetAdjusterMoverHandle, rigExec::RigExecMoverHandle>(m, "CurvenetAdjusterMover")
        .def("set_adjustments", [](rigExec::RigExecCurvenetAdjusterMoverHandle &h, py::iterable inputs) {
            std::vector<SdfPath> paths;
            for (const auto &p:inputs) paths.push_back(_PythonToDependencyPath(
                py::reinterpret_borrow<py::object>(p),h.GetStage(),TfToken("RigExecCurvenetAdjustment"),false));
            h.SetAdjustments(paths);
        },py::arg("adjustments"));

    py::class_<rigExec::RigExecCurvenetMoverHandle, rigExec::RigExecMoverHandle>(m, "CurvenetMover")
        .def("set_curvenet", [](rigExec::RigExecCurvenetMoverHandle &h, py::object p) { h.SetCurvenet(_PythonToDependencyPath(p, h.GetStage(), TfToken("RigExecCurvenet"), false)); }, py::arg("path"))
        .def("set_strength", &rigExec::RigExecCurvenetMoverHandle::SetStrength, py::arg("strength"));

    py::class_<rigExec::RigExecFloatMathMoverHandle, rigExec::RigExecMoverHandle>(m, "FloatMathMover")
        .def("set_operation", [](rigExec::RigExecFloatMathMoverHandle &h, std::string v) { h.SetOperation(TfToken(v)); }, py::arg("op"))
        .def("set_value", &rigExec::RigExecFloatMathMoverHandle::SetValue, py::arg("value"))
        .def("set_bounds", &rigExec::RigExecFloatMathMoverHandle::SetBounds, py::arg("min"), py::arg("max"))
        .def("set_weight", &rigExec::RigExecFloatMathMoverHandle::SetWeight, py::arg("weight"));

    py::class_<rigExec::RigExecVec3fMathMoverHandle, rigExec::RigExecMoverHandle>(m, "Vec3fMathMover")
        .def("set_operation", [](rigExec::RigExecVec3fMathMoverHandle &h, std::string v) { h.SetOperation(TfToken(v)); }, py::arg("op"))
        .def("set_value", [](rigExec::RigExecVec3fMathMoverHandle &h, std::array<double, 3> v) {
            h.SetValue(GfVec3f(float(v[0]), float(v[1]), float(v[2])));
        }, py::arg("value"))
        .def("set_bounds", [](rigExec::RigExecVec3fMathMoverHandle &h, std::array<double, 3> mn, std::array<double, 3> mx) {
            h.SetBounds(GfVec3f(float(mn[0]), float(mn[1]), float(mn[2])),
                        GfVec3f(float(mx[0]), float(mx[1]), float(mx[2])));
        }, py::arg("min"), py::arg("max"))
        .def("set_weight", &rigExec::RigExecVec3fMathMoverHandle::SetWeight, py::arg("weight"));

    py::class_<rigExec::RigExecMatrixMathMoverHandle, rigExec::RigExecMoverHandle>(m, "MatrixMathMover")
        .def("set_operation", [](rigExec::RigExecMatrixMathMoverHandle &h, std::string v) { h.SetOperation(TfToken(v)); }, py::arg("op"))
        .def("set_value", [](rigExec::RigExecMatrixMathMoverHandle &h, std::vector<double> m) {
            h.SetValue(_VecToMat4(m));
        }, py::arg("matrix"))
        .def("set_weight", &rigExec::RigExecMatrixMathMoverHandle::SetWeight, py::arg("weight"));

    // Mover chain.
    py::class_<rigExec::RigExecMoverChain>(m, "MoverChain",
        "One mover chain: operations added here apply in REVERSE add order\n(last added runs first); add outermost passes first.")
        .def_property_readonly("scope_path", [](const rigExec::RigExecMoverChain &c) { return _PathStr(c.GetScopePath()); })
        .def("add_matrix_mover", [](rigExec::RigExecMoverChain &c, std::string name,
                                     py::object transformProvider,
                                     py::object weightObject, py::object target,
                                     std::string readPhase) {
            return c.AddMatrixMover(
                name,
                _PythonToDependencyPath(transformProvider, c.GetStage(), TfToken(), false),
                _PythonToDependencyPath(weightObject, c.GetStage()),
                _PythonToDependencyPath(target, c.GetStage()), TfToken(readPhase));
        }, py::arg("name"), py::arg("transform_provider"),
           py::arg("weight_object") = py::none(),
           py::arg("target") = py::none(), py::arg("read_phase") = "base")
        .def("add_skin_mover", [](rigExec::RigExecMoverChain &c, std::string name,
                                   py::iterable influences, py::object weightObject,
                                   py::object target, std::string readPhase) {
            std::vector<SdfPath> paths;
            for (const auto &p : influences) paths.push_back(_PythonToDependencyPath(
                py::reinterpret_borrow<py::object>(p), c.GetStage(), TfToken(), false));
            return c.AddSkinMover(
                name, paths,
                _PythonToDependencyPath(weightObject, c.GetStage()),
                _PythonToDependencyPath(target, c.GetStage()), TfToken(readPhase));
        }, py::arg("name"), py::arg("influences"),
           py::arg("weight_object") = py::none(),
           py::arg("target") = py::none(), py::arg("read_phase") = "base")
        .def("add_lattice_mover", [](rigExec::RigExecMoverChain &c, std::string name,
                                      py::object cagePrim, int dx, int dy, int dz,
                                      std::string basis, py::object target,
                                      std::string readPhase) {
            return c.AddLatticeMover(
                name, _PythonToDependencyPath(cagePrim, c.GetStage(), TfToken(), false),
                dx, dy, dz, TfToken(basis),
                _PythonToDependencyPath(target, c.GetStage()), TfToken(readPhase));
        }, py::arg("name"), py::arg("cage_prim"), py::arg("div_x"), py::arg("div_y"), py::arg("div_z"),
           py::arg("basis") = "bspline", py::arg("target") = py::none(), py::arg("read_phase") = "base")
        .def("add_blend_shape_mover", [](rigExec::RigExecMoverChain &c,
                                          std::string name, py::object weightObject,
                                          py::object target) {
            return c.AddBlendShapeMover(
                name, _PythonToDependencyPath(weightObject, c.GetStage()),
                _PythonToDependencyPath(target, c.GetStage()));
        }, py::arg("name"), py::arg("weight_object") = py::none(),
           py::arg("target") = py::none())
        .def("add_curve_mover", [](rigExec::RigExecMoverChain &c, std::string name,
                                    py::object driverCurve,
                                    const py::object &driverFrames,
                                    py::object bindCoordinates, std::string mode,
                                    py::object target, std::string readPhase) {
            SdfPathVector frames;
            if (!driverFrames.is_none()) {
                const bool isSinglePath =
                    py::isinstance<py::str>(driverFrames) ||
                    py::hasattr(driverFrames, "pathString") ||
                    py::hasattr(driverFrames, "path") ||
                    py::hasattr(driverFrames, "GetPath");
                if (isSinglePath) {
                    frames.push_back(_PythonToDependencyPath(
                        driverFrames, c.GetStage(), TfToken(), false));
                } else {
                    frames = _PythonToDependencyPaths(
                        driverFrames.cast<py::iterable>(), c.GetStage());
                }
            }
            return c.AddCurveMover(
                name, _PythonToDependencyPath(driverCurve, c.GetStage(), TfToken(), false),
                frames,
                _PythonToDependencyPath(bindCoordinates, c.GetStage()), TfToken(mode),
                _PythonToDependencyPath(target, c.GetStage()), TfToken(readPhase));
        }, py::arg("name"), py::arg("driver_curve"),
           py::arg("driver_frames") = py::none(),
           py::arg("bind_coordinates") = py::none(), py::arg("mode") = "ribbon",
           py::arg("target") = py::none(), py::arg("read_phase") = "base")
        .def("add_surface_mover", [](rigExec::RigExecMoverChain &c, std::string name,
                                      py::object surfacePrim, std::string mode,
                                      py::object target, std::string readPhase) {
            return c.AddSurfaceMover(
                name, _PythonToDependencyPath(surfacePrim, c.GetStage(), TfToken(), false),
                TfToken(mode), _PythonToDependencyPath(target, c.GetStage()),
                TfToken(readPhase));
        }, py::arg("name"), py::arg("surface_prim"), py::arg("mode") = "attach",
           py::arg("target") = py::none(), py::arg("read_phase") = "base")
        .def("add_smooth_mover", [](rigExec::RigExecMoverChain &c, std::string name,
                                     float defaultWeight, py::object target) {
            return c.AddSmoothMover(name, defaultWeight,
                                    _PythonToDependencyPath(target, c.GetStage()));
        }, py::arg("name"), py::arg("default_weight") = 1.0f,
           py::arg("target") = py::none())
        .def("add_volume_correct_mover", [](rigExec::RigExecMoverChain &c,
                                             std::string name, float defaultWeight,
                                             py::object target) {
            return c.AddVolumeCorrectMover(
                name, defaultWeight,
                _PythonToDependencyPath(target, c.GetStage()));
        }, py::arg("name"), py::arg("default_weight") = 1.0f,
           py::arg("target") = py::none())
        .def("add_curvenet_adjuster_mover", [](rigExec::RigExecMoverChain &c, std::string name,
                py::iterable inputs, float defaultWeight, py::object target) {
            std::vector<SdfPath> paths;
            for (const auto &p:inputs) paths.push_back(_PythonToDependencyPath(
                py::reinterpret_borrow<py::object>(p),c.GetStage(),TfToken("RigExecCurvenetAdjustment"),false));
            return c.AddCurvenetAdjusterMover(name,paths,defaultWeight,
                _PythonToDependencyPath(target,c.GetStage()));
        },py::arg("name"),py::arg("adjustments"),py::arg("default_weight")=1.0f,py::arg("target")=py::none())
        .def("add_curvenet_mover", [](rigExec::RigExecMoverChain &c, std::string name,
                                       py::object curvenetPrim, float defaultWeight,
                                       py::object target) {
            return c.AddCurvenetMover(
                name, _PythonToDependencyPath(curvenetPrim, c.GetStage(), TfToken("RigExecCurvenet"), false),
                defaultWeight, _PythonToDependencyPath(target, c.GetStage()));
        }, py::arg("name"), py::arg("curvenet_prim"),
           py::arg("default_weight") = 1.0f,
           py::arg("target") = py::none())
        .def("add_float_math_mover", [](rigExec::RigExecMoverChain &c, std::string name,
                                         std::string operation, float value,
                                         py::object target, float defaultWeight) {
            return c.AddFloatMathMover(
                name, TfToken(operation), value,
                _PythonToDependencyPath(target, c.GetStage()), defaultWeight);
        }, py::arg("name"), py::arg("operation"), py::arg("value"),
           py::arg("target") = py::none(), py::arg("default_weight") = 1.0f)
        .def("add_vec3f_math_mover", [](rigExec::RigExecMoverChain &c, std::string name,
                                         std::string operation,
                                         std::array<double, 3> value,
                                         py::object target, float defaultWeight) {
            return c.AddVec3fMathMover(
                name, TfToken(operation),
                GfVec3f(float(value[0]), float(value[1]), float(value[2])),
                _PythonToDependencyPath(target, c.GetStage()), defaultWeight);
        }, py::arg("name"), py::arg("operation"), py::arg("value"),
           py::arg("target") = py::none(), py::arg("default_weight") = 1.0f)
        .def("add_matrix_math_mover", [](rigExec::RigExecMoverChain &c, std::string name,
                                          std::string operation,
                                          std::vector<double> value,
                                          py::object target, float defaultWeight) {
            return c.AddMatrixMathMover(
                name, TfToken(operation), _VecToMat4(value),
                _PythonToDependencyPath(target, c.GetStage()), defaultWeight);
        }, py::arg("name"), py::arg("operation"), py::arg("matrix"),
           py::arg("target") = py::none(), py::arg("default_weight") = 1.0f)
        .def("add_aim_constraint", [](rigExec::RigExecMoverChain &c, std::string name,
                                       py::object target, const py::iterable &sources,
                                       std::vector<float> weights) {
            const SdfPath t = _PythonToDependencyPath(target, c.GetStage());
            const SdfPathVector s = _PythonToDependencyPaths(sources, c.GetStage());
            if (!weights.empty() && weights.size() != s.size()) throw py::value_error("weights must match sources");
            auto h = c.AddAimConstraint(name, t);
            if (!s.empty()) h.SetSources(s, weights);
            return h;
        }, py::arg("name"), py::arg("target") = py::none(),
           py::arg("sources") = py::tuple(), py::arg("weights") = py::list())
        .def("add_position_constraint", [](rigExec::RigExecMoverChain &c, std::string name,
                                            py::object target, const py::iterable &sources,
                                            std::vector<float> weights) {
            const SdfPath t = _PythonToDependencyPath(target, c.GetStage());
            const SdfPathVector s = _PythonToDependencyPaths(sources, c.GetStage());
            if (!weights.empty() && weights.size() != s.size()) throw py::value_error("weights must match sources");
            auto h = c.AddPositionConstraint(name, t);
            if (!s.empty()) h.SetSources(s, weights);
            return h;
        }, py::arg("name"), py::arg("target") = py::none(),
           py::arg("sources") = py::tuple(), py::arg("weights") = py::list())
        .def("add_rotation_constraint", [](rigExec::RigExecMoverChain &c, std::string name,
                                            py::object target, const py::iterable &sources,
                                            std::vector<float> weights) {
            const SdfPath t = _PythonToDependencyPath(target, c.GetStage());
            const SdfPathVector s = _PythonToDependencyPaths(sources, c.GetStage());
            if (!weights.empty() && weights.size() != s.size()) throw py::value_error("weights must match sources");
            auto h = c.AddRotationConstraint(name, t);
            if (!s.empty()) h.SetSources(s, weights);
            return h;
        }, py::arg("name"), py::arg("target") = py::none(),
           py::arg("sources") = py::tuple(), py::arg("weights") = py::list())
        .def("add_scale_constraint", [](rigExec::RigExecMoverChain &c, std::string name,
                                         py::object target, const py::iterable &sources,
                                         std::vector<float> weights) {
            const SdfPath t = _PythonToDependencyPath(target, c.GetStage());
            const SdfPathVector s = _PythonToDependencyPaths(sources, c.GetStage());
            if (!weights.empty() && weights.size() != s.size()) throw py::value_error("weights must match sources");
            auto h = c.AddScaleConstraint(name, t);
            if (!s.empty()) h.SetSources(s, weights);
            return h;
        }, py::arg("name"), py::arg("target") = py::none(),
           py::arg("sources") = py::tuple(), py::arg("weights") = py::list())
        .def("add_parent_constraint", [](rigExec::RigExecMoverChain &c, std::string name,
                                          py::object target, const py::iterable &sources,
                                          std::vector<float> weights) {
            const SdfPath t = _PythonToDependencyPath(target, c.GetStage());
            const SdfPathVector s = _PythonToDependencyPaths(sources, c.GetStage());
            if (!weights.empty() && weights.size() != s.size()) throw py::value_error("weights must match sources");
            auto h = c.AddParentConstraint(name, t);
            if (!s.empty()) h.SetSources(s, weights);
            return h;
        }, py::arg("name"), py::arg("target") = py::none(),
           py::arg("sources") = py::tuple(), py::arg("weights") = py::list())
        .def("add_single_chain_ik_constraint",
            [](rigExec::RigExecMoverChain &c, std::string name,
               py::object firstJoint, py::object endJoint, py::object effector,
               const py::iterable &poleObjects, const py::object &moves) {
                const SdfPath first = _PythonToDependencyPath(
                    firstJoint, c.GetStage(), TfToken("RigExecJoint"), false);
                const SdfPath end = _PythonToDependencyPath(
                    endJoint, c.GetStage(), TfToken("RigExecJoint"), false);
                const SdfPath effectorPath = _PythonToDependencyPath(
                    effector, c.GetStage(), TfToken(), false);
                const SdfPathVector poles =
                    _PythonToDependencyPaths(poleObjects, c.GetStage());
                if (moves.is_none()) {
                    return c.AddSingleChainIkConstraint(
                        name, first, end, effectorPath, poles);
                }
                return c.AddSingleChainIkConstraint(
                    name,
                    _PythonToDependencyPaths(
                        moves.cast<py::iterable>(), c.GetStage(),
                        TfToken("RigExecJoint")),
                    first, end, effectorPath, poles);
            }, py::arg("name"), py::arg("first_joint"), py::arg("end_joint"),
               py::arg("effector"), py::arg("pole_vector_objects") = py::tuple(),
               py::kw_only(), py::arg("moves") = py::none(),
               "Create a complete joint-chain IK constraint. The coarse form "
               "infers its write set; moves= enables strict importer parity "
               "validation.")
        .def("under", [](const rigExec::RigExecMoverChain &c,
                          const RigExecHandleBase &mover, py::object defaultTarget) {
            return c.Under(
                mover, _PythonToDependencyPath(defaultTarget, c.GetStage()));
        }, py::arg("mover"), py::arg("default_target") = py::none());

    // Builder.
    py::class_<rigExec::RigExecRigBuilder>(m, "Builder",
        "The top-level rig builder: one per (stage, rig root). Creates and wires the prims a RigExec rig is made of.")
        .def_static("create", [](py::object stageObj, py::object rigRoot,
                                  std::string partition) {
            auto s = _ExtractStage(stageObj);
            if (!s) {
                throw py::type_error(
                    "Builder.create() needs a pxr.Usd.Stage instance");
            }
            return rigExec::RigExecRigBuilder::Create(
                s, _PythonToPath(rigRoot, false), TfToken(partition));
        }, py::arg("stage"), py::arg("rig_root") = "/Rig", py::arg("partition") = "")
        .def_property_readonly("root_path", [](const rigExec::RigExecRigBuilder &b) { return _PathStr(b.GetRootPath()); })

        // Transform providers.
        .def("add_control", [](rigExec::RigExecRigBuilder &b, std::string name,
                                std::vector<double> restSpace,
                                const py::object &parentObject) {
            rigExec::RigExecControlHandle parent;
            const rigExec::RigExecControlHandle *parentPtr = nullptr;
            if (!parentObject.is_none()) {
                parent = rigExec::RigExecControlHandle(
                    b.GetStage(), _PythonToDependencyPath(
                        parentObject, b.GetStage(),
                        TfToken("RigExecControl"), false));
                parentPtr = &parent;
            }
            return b.AddControl(
                name, restSpace.empty() ? GfMatrix4d() : _VecToMat4(restSpace),
                parentPtr);
        }, py::arg("name"), py::arg("rest_space") = py::list(),
           py::arg("parent") = py::none(),
           "Create <rig>/Controls/<name>, or nest it under `parent` (another "
           "control), in which case rest_space is relative to that parent's "
           "rest and the control travels with it.")
        .def("add_joint", [](rigExec::RigExecRigBuilder &b, std::string name,
                              std::vector<double> restSpace,
                              const py::object &parentObject) {
            rigExec::RigExecJointHandle parent;
            const rigExec::RigExecJointHandle *parentPtr = nullptr;
            if (!parentObject.is_none()) {
                parent = rigExec::RigExecJointHandle(
                    b.GetStage(), _PythonToDependencyPath(
                        parentObject, b.GetStage(),
                        TfToken("RigExecJoint"), false));
                parentPtr = &parent;
            }
            return b.AddJoint(
                name, restSpace.empty() ? GfMatrix4d() : _VecToMat4(restSpace),
                parentPtr);
        }, py::arg("name"), py::arg("rest_space") = py::list(),
           py::arg("parent_joint") = py::none())

        // Solvers.
        .def("add_fk_chain", [](rigExec::RigExecRigBuilder &b,
                                 std::string name, const py::object &controls,
                                 const py::object &joints) {
            SdfPathVector controlPaths;
            SdfPathVector jointPaths;
            if (!controls.is_none()) {
                controlPaths = _PythonToDependencyPaths(
                    controls.cast<py::iterable>(), b.GetStage(),
                    TfToken("RigExecControl"));
            }
            if (!joints.is_none()) {
                jointPaths = _PythonToDependencyPaths(
                    joints.cast<py::iterable>(), b.GetStage(),
                    TfToken("RigExecJoint"));
            }
            auto handle = b.AddFkChain(name);
            if (!controls.is_none()) {
                handle.SetControls(controlPaths);
            }
            if (!joints.is_none()) {
                handle.SetJoints(jointPaths);
            }
            return handle;
        }, py::arg("name"), py::arg("controls") = py::none(),
           py::arg("joints") = py::none(),
           "Create an FK solver and optionally wire its ordered controls and joints.")
        .def("add_two_bone_ik", [](rigExec::RigExecRigBuilder &b,
                                     std::string name, py::object rootControl,
                                     py::object effectorControl,
                                     py::object poleControl) {
            return b.AddTwoBoneIk(
                name,
                _PythonToDependencyPath(rootControl, b.GetStage(),
                    TfToken("RigExecControl"), false),
                _PythonToDependencyPath(effectorControl, b.GetStage(),
                    TfToken("RigExecControl"), false),
                _PythonToDependencyPath(poleControl, b.GetStage(),
                    TfToken("RigExecControl")));
        }, py::arg("name"), py::arg("root_control"),
           py::arg("effector_control"), py::arg("pole_control") = py::none())
        .def("add_blend_point_frames", [](rigExec::RigExecRigBuilder &b,
                                            std::string name, py::object inputA,
                                            py::object inputB, float weight) {
            return b.AddBlendPointFrames(
                name,
                _PythonToDependencyPath(inputA, b.GetStage(), TfToken(), false),
                _PythonToDependencyPath(inputB, b.GetStage(), TfToken(), false),
                weight);
        }, py::arg("name"), py::arg("input_a"), py::arg("input_b"),
           py::arg("weight") = 0.0f)
        .def("add_twist_distribution", [](rigExec::RigExecRigBuilder &b,
                                            std::string name, py::object start,
                                            py::object end, int count) {
            return b.AddTwistDistribution(
                name,
                _PythonToDependencyPath(start, b.GetStage(), TfToken(), false),
                _PythonToDependencyPath(end, b.GetStage(), TfToken(), false),
                count);
        }, py::arg("name"), py::arg("start"), py::arg("end"),
           py::arg("count") = 1)
        .def("add_ribbon", [](rigExec::RigExecRigBuilder &b,
                                std::string name, py::object driverCurve,
                                int sampleCount) {
            return b.AddRibbon(
                name, _PythonToDependencyPath(
                    driverCurve, b.GetStage(), TfToken(), false),
                sampleCount);
        }, py::arg("name"), py::arg("driver_curve"),
           py::arg("sample_count") = 5)
        .def("add_spline_ik", [](rigExec::RigExecRigBuilder &b,
                                   std::string name, py::object rootControl,
                                   py::object midControl, py::object endControl,
                                   const py::object &joints) {
            auto handle = b.AddSplineIk(
                name,
                _PythonToDependencyPath(rootControl, b.GetStage(),
                    TfToken("RigExecControl"), false),
                _PythonToDependencyPath(midControl, b.GetStage(),
                    TfToken("RigExecControl"), false),
                _PythonToDependencyPath(endControl, b.GetStage(),
                    TfToken("RigExecControl"), false));
            if (!joints.is_none()) {
                handle.SetJoints(_PythonToDependencyPaths(
                    joints, b.GetStage(), TfToken("RigExecJoint")));
            }
            return handle;
        }, py::arg("name"), py::arg("root_control"), py::arg("mid_control"),
           py::arg("end_control"), py::arg("joints") = py::none(),
           "Create a spline IK solver from three controls and optionally wire "
           "its ordered joint chain (root to tip).")

        // Weight objects.
        .def("add_static_weight", [](rigExec::RigExecRigBuilder &b,
                                      std::string name, py::object target,
                                      std::vector<float> values,
                                      std::vector<int> indices,
                                      float defaultWeight) {
            return b.AddStaticWeight(
                name, _PythonToDependencyPath(
                    target, b.GetStage(), TfToken(), false),
                values, indices, defaultWeight);
        }, py::arg("name"), py::arg("target"), py::arg("values") = py::list(),
           py::arg("indices") = py::list(), py::arg("default_weight") = 0.0f)
        .def("add_dynamic_weight", [](rigExec::RigExecRigBuilder &b,
                                       std::string name, py::object target,
                                       py::object baseWeight) {
            return b.AddDynamicWeight(
                name, _PythonToDependencyPath(
                    target, b.GetStage(), TfToken(), false),
                _PythonToDependencyPath(baseWeight, b.GetStage()));
        }, py::arg("name"), py::arg("target"),
           py::arg("base_weight") = py::none())
        .def("add_sphere_weight", [](rigExec::RigExecRigBuilder &b,
                                      std::string name, py::object target,
                                      float falloffMin, float falloffMax) {
            return b.AddSphereWeight(
                name, _PythonToDependencyPath(
                    target, b.GetStage(), TfToken(), false),
                falloffMin, falloffMax);
        }, py::arg("name"), py::arg("target"),
           py::arg("falloff_min") = 0.0f, py::arg("falloff_max") = 1.0f)
        .def("add_plane_weight", [](rigExec::RigExecRigBuilder &b,
                                     std::string name, py::object target,
                                     float falloffMin, float falloffMax) {
            return b.AddPlaneWeight(
                name, _PythonToDependencyPath(
                    target, b.GetStage(), TfToken(), false),
                falloffMin, falloffMax);
        }, py::arg("name"), py::arg("target"),
           py::arg("falloff_min") = 0.0f, py::arg("falloff_max") = 1.0f)
        .def("add_curve_weight", [](rigExec::RigExecRigBuilder &b,
                                     std::string name, py::object target,
                                     py::object curve, float falloffMin,
                                     float falloffMax) {
            return b.AddCurveWeight(
                name, _PythonToDependencyPath(
                    target, b.GetStage(), TfToken(), false),
                _PythonToDependencyPath(
                    curve, b.GetStage(), TfToken(), false),
                falloffMin, falloffMax);
        }, py::arg("name"), py::arg("target"), py::arg("curve"),
           py::arg("falloff_min") = 0.0f, py::arg("falloff_max") = 1.0f)
        .def("define_sphere_weight", [](rigExec::RigExecRigBuilder &b,
                                         py::object path, py::object target,
                                         float falloffMin, float falloffMax) {
            return b.DefineSphereWeight(
                _PythonToPath(path, false),
                _PythonToDependencyPath(
                    target, b.GetStage(), TfToken(), false),
                falloffMin, falloffMax);
        }, py::arg("path"), py::arg("target"),
           py::arg("falloff_min") = 0.0f, py::arg("falloff_max") = 1.0f)
        .def("define_plane_weight", [](rigExec::RigExecRigBuilder &b,
                                        py::object path, py::object target,
                                        float falloffMin, float falloffMax) {
            return b.DefinePlaneWeight(
                _PythonToPath(path, false),
                _PythonToDependencyPath(
                    target, b.GetStage(), TfToken(), false),
                falloffMin, falloffMax);
        }, py::arg("path"), py::arg("target"),
           py::arg("falloff_min") = 0.0f, py::arg("falloff_max") = 1.0f)
        .def("define_curve_weight", [](rigExec::RigExecRigBuilder &b,
                                        py::object path, py::object target,
                                        py::object curve, float falloffMin,
                                        float falloffMax) {
            return b.DefineCurveWeight(
                _PythonToPath(path, false),
                _PythonToDependencyPath(
                    target, b.GetStage(), TfToken(), false),
                _PythonToDependencyPath(
                    curve, b.GetStage(), TfToken(), false),
                falloffMin, falloffMax);
        }, py::arg("path"), py::arg("target"), py::arg("curve"),
           py::arg("falloff_min") = 0.0f, py::arg("falloff_max") = 1.0f)
        .def("add_combine_weight", [](rigExec::RigExecRigBuilder &b,
                                       std::string name, py::object target,
                                       const py::iterable &inputWeights,
                                       std::string mode) {
            return b.AddCombineWeight(
                name, _PythonToDependencyPath(
                    target, b.GetStage(), TfToken(), false),
                _PythonToDependencyPaths(inputWeights, b.GetStage()),
                TfToken(mode));
        }, py::arg("name"), py::arg("target"),
           py::arg("input_weights") = py::tuple(),
           py::arg("mode") = "multiply")

        // Independently composable blend inputs.
        .def("add_blend_input", &rigExec::RigExecRigBuilder::AddBlendInput,
             py::arg("name"), py::arg("weight") = 0.0f)

        // A handle onto a blend input that already exists -- the ones
        // build_shapes.py authored, which build_psd.py then connects to a
        // pose weight. Handles are cheap values naming one prim and the
        // C++ side constructs them freely; this is the one way python has
        // to name a prim it did not itself create, and it type-checks.
        .def("blend_input", [](rigExec::RigExecRigBuilder &b,
                               py::object path) {
            return rigExec::RigExecBlendInputHandle(
                b.GetStage(), _PythonToDependencyPath(
                    path, b.GetStage(), TfToken("RigExecBlendInput"),
                    false));
        }, py::arg("path"))

        // Pose interpolators, at <rig>/PoseInterpolators/<name>. Its own
        // scope and not /Movers: an interpolator writes no transform and no
        // points, so it has no place in an order that exists to say which
        // write lands on top of which.
        .def("add_pose_interpolator", [](rigExec::RigExecRigBuilder &b,
                std::string name, py::object driver) {
            return b.AddPoseInterpolator(name, _PythonToDependencyPath(
                driver, b.GetStage(), TfToken(), false));
        }, py::arg("name"), py::arg("driver"))

        .def("add_curvenet_adjustment", [](rigExec::RigExecRigBuilder &b, std::string name,
                py::object curvenet, int index) {
            return b.AddCurvenetAdjustment(name,_PythonToDependencyPath(curvenet,
                b.GetStage(),TfToken("RigExecCurvenet"),false),index);
        },py::arg("name"),py::arg("curvenet"),py::arg("knot_index"))
        // Curvenets.
        .def("add_curvenet", [](rigExec::RigExecRigBuilder &b, std::string name, std::vector<std::array<double, 3>> points) {
            std::vector<GfVec3f> v;
            for (const auto &p : points) {
                v.push_back(GfVec3f(float(p[0]), float(p[1]), float(p[2])));
            }
            return b.AddCurvenet(name, v);
        }, py::arg("name"), py::arg("points") = py::list())

        // Mover chains.
        .def("new_mover_chain",
             [](rigExec::RigExecRigBuilder &b, std::string name,
                py::object defaultTarget) {
                 return b.NewMoverChain(
                     name, _PythonToDependencyPath(
                         defaultTarget, b.GetStage()));
             }, py::arg("name"), py::arg("default_target") = py::none(),
             "Start a mover chain; operations added without an explicit target"
             " reuse default_target.");

    // ---- RBF pose interpolators -------------------------------------------
    //
    // See the note above _RbfDescFrom. These two entries are what let the
    // converter drop its Python solver import.

    m.def("bind_wire",
          [](const std::vector<std::array<float, 3>> &points,
             const std::vector<std::array<float, 3>> &controlPoints,
             int order, const std::vector<double> &knots) {
              std::vector<GfVec3f> pts, cvs;
              pts.reserve(points.size());
              for (const auto &p : points) pts.emplace_back(p[0], p[1], p[2]);
              for (const auto &c : controlPoints) cvs.emplace_back(c[0], c[1], c[2]);
              const rigExec::RigExecNurbsCurve curve{&cvs, order, &knots};
              if (!curve.IsValid()) {
                  throw py::value_error(
                      "bind_wire: control points, order and knots do not "
                      "describe a NURBS curve (knots must number points + order)");
              }
              std::vector<std::pair<float, float>> out;
              for (const GfVec2f &b : rigExec::RigExecBindWire(pts, curve)) {
                  out.emplace_back(b[0], b[1]);
              }
              return out;
          },
          py::arg("points"), py::arg("control_points"), py::arg("order"),
          py::arg("knots"),
          "Wire bind coordinates for RigExecCurveMover mode \"wire\": per "
          "point, (u, d) with u the parameter of the closest point on the "
          "rest NURBS curve and d the distance to it.");
    m.def("rbf_fit_width", [](py::object poses, py::object translations,
                              std::string kernel, py::object pose_types,
                              py::object twist_axis, double regularization,
                              bool enable_rotation, bool enable_translation,
                              double pose_falloff) {
        rigExec::RigExecRbfSolverDesc desc = _RbfDescFrom(
            poses, translations, kernel, pose_types, twist_axis,
            regularization, enable_rotation, enable_translation);
        rigExec::RigExecRbfFitReport report;
        rigExec::RigExecRbfSolver solver =
            rigExec::RigExecRbfFitWidth(desc, &report);
        // the conventional painted poseFalloff, on top of the fitted width. 0.3 is
        // the conventional default and leaves the fit alone; the floor at 0.05 is the
        // the reference implementation's.
        if (std::abs(pose_falloff - 0.3) > 1.0e-6) {
            solver.ScaleWidths(std::max(pose_falloff / 0.3, 0.05));
            solver.Solve();
        }
        py::dict table = _RbfTable(solver, desc, kernel);
        py::dict fit;
        fit["width"] = report.width;
        fit["translation_width"] = report.translationWidth;
        fit["per_pose"] = report.perPose;
        fit["scale"] = report.scale;
        fit["coverage"] = report.coverage;
        fit["overshoot"] = report.overshoot;
        table["fit"] = fit;
        return table;
    }, py::arg("poses"), py::arg("translations") = py::none(),
       py::arg("kernel") = "gaussian", py::arg("pose_types") = py::none(),
       py::arg("twist_axis") = py::none(), py::arg("regularization") = 0.0,
       py::arg("enable_rotation") = true,
       py::arg("enable_translation") = false,
       py::arg("pose_falloff") = 0.3,
       "Fit an interpolator's falloff width, solve it, and return the table.\n"
       "\n"
       "poses are XYZ eulers in radians, translations are metres in the\n"
       "driver's own frame, pose_types are poseType (0 whole, 1 swing,\n"
       "2 twist). The returned dict is the solved interpolator and is what\n"
       "rbf_evaluate takes back.");

    // The converter stores a pose as a QUATERNION (the schema's
    // rigExec:rotation) and the solver takes XYZ eulers, so the conversion has
    // to happen somewhere. Here, rather than as a second copy of
    // rbf.py:1130-1146 in python: the closed form is not interchangeable with
    // GfRotation's decomposition at the precision the parity fixture is
    // compared at, and two spellings of it would drift.
    m.def("rbf_euler_from_quaternion", [](std::array<double, 4> q) {
        const GfVec3d e = rigExec::RigExecRbfEulerFromQuaternion(
            GfQuatd(q[0], GfVec3d(q[1], q[2], q[3])));
        return py::make_tuple(e[0], e[1], e[2]);
    }, py::arg("quaternion"),
       "A (real, i, j, k) quaternion as an XYZ euler in radians -- the exact\n"
       "inverse of the closed form the solver's poses are built with.");

    // The gate tools/biped/verify_psd.py holds every interpolator to:
    // the kernels must still sum to at least RigExecRbfCoverageFloor
    // everywhere BETWEEN the poses. Zero there is a dead zone -- every
    // shape switches off and snaps back as the driver leaves -- and it
    // is invisible to any check made AT the poses, which is why it needs
    // its own entry point rather than a loop over rbf_evaluate.
    m.def("rbf_coverage", [](py::dict table, int steps) {
        return _RbfSolverFromTable(table).Coverage(steps);
    }, py::arg("table"), py::arg("steps") = 8,
       "The lowest total kernel value anywhere between the poses,\n"
       "sampled along every ordered pair.");

    m.def("rbf_evaluate", [](py::dict table, py::object rotation,
                             py::object translation, bool allow_negative) {
        rigExec::RigExecRbfSolver solver = _RbfSolverFromTable(table);
        const GfVec3d euler = _SeqToVec3d(rotation);
        GfVec3d moved(0.0);
        const bool haveTranslation = !translation.is_none();
        if (haveTranslation) {
            moved = _SeqToVec3d(translation);
        }
        std::vector<double> out;
        solver.Evaluate(euler, haveTranslation ? &moved : nullptr, &out,
                        allow_negative);
        return out;
    }, py::arg("table"), py::arg("rotation"),
       py::arg("translation") = py::none(),
       py::arg("allow_negative") = true,
       "How much each authored pose counts, for one driver pose.\n"
       "\n"
       "table is what rbf_fit_width returned (or the same dict read back from\n"
       "a converted rig). allow_negative is allowNegativeWeights: with\n"
       "it off the weights are clamped at zero AFTER normalisation, never\n"
       "before.");
}
