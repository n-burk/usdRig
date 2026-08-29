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
#include "pxr/base/vt/array.h"
#include "pxr/base/tf/refPtr.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include "rigExec/rigEvaluator.h"
#include "rigExecMath/pointFrame.h"
#include "rigExecRigging/rigBuilder.h"

#include <array>
#include <memory>
#include <sstream>
#include <string>
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

// ---------------------------------------------------------------------------
// Rig: the evaluator wrapper.
// ---------------------------------------------------------------------------

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

    void Compile() {
        std::vector<std::string> errors;
        if (!evaluator->Compile(&errors)) {
            std::string msg = "rig compile failed:";
            for (const auto &e : errors) {
                msg += "\n  - " + e;
            }
            throw py::value_error(msg);
        }
    }

    rigExec::RigExecRigPose Evaluate(double timeFrames) const {
        UsdTimeCode t = timeFrames < 0 ? UsdTimeCode::Default() : UsdTimeCode(timeFrames);
        return evaluator->Evaluate(t);
    }
};

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
        }, "The composed movers in execution order (reverse-sibling\npost-order: bottom-to-top stack walk, spec section 4.2).");

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
        .def_property_readonly("solver_override_rounds",
            [](const rigExec::RigExecRigPose &p) { return p.solverOverrideRounds; })
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
        .def("control_frame", [](const rigExec::RigExecRigPose &p, std::string path) {
                auto it = p.controlFrames.find(SdfPath(path));
                if (it == p.controlFrames.end()) {
                    throw py::key_error("no control frame for " + path);
                }
                return it->second;
            }, py::arg("path"))

        // Provider xforms.
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

    // ---- Rigging API: handles ----------------------------------------------

    using rigExec::RigExecHandleBase;

    py::class_<RigExecHandleBase>(m, "Handle",
        "Base of every rig object handle. Handles are cheap values naming one prim.")
        .def_property_readonly("valid", &RigExecHandleBase::IsValid)
        .def_property_readonly("path", [](const RigExecHandleBase &h) { return _PathStr(h.GetPath()); })
        .def_property_readonly("name", &RigExecHandleBase::GetName)
        .def("set_attr", [](RigExecHandleBase &h, std::string name,
                            std::string typeName, py::object value) {
                // Generic escape hatch: dispatch on the DECLARED Sdf type so
                // the payload matches exactly what this USD build's VtValue
                // accepts (raw std::vector payloads are rejected for array
                // attributes). A Python list is a sequence, so type-name
                // dispatch must come first.
                auto set = [&h, &name, &typeName](VtValue v) {
                    h.SetAttr(name.c_str(), TfToken(typeName), std::move(v));
                };
                if (typeName == "float") {
                    set(VtValue(float(value.cast<double>())));
                } else if (typeName == "double") {
                    set(VtValue(value.cast<double>()));
                } else if (typeName == "int") {
                    set(VtValue(value.cast<int>()));
                } else if (typeName == "bool") {
                    set(VtValue(value.cast<bool>()));
                } else if (typeName == "token" || typeName == "string") {
                    const std::string s = value.cast<std::string>();
                    set(typeName == "token" ? VtValue(TfToken(s)) : VtValue(s));
                } else if (typeName == "point3f" || typeName == "vec3f") {
                    py::sequence seq = value;
                    if (seq.size() != 3) {
                        throw py::type_error("set_attr: point3f needs 3 values");
                    }
                    set(VtValue(GfVec3f(float(seq[0].cast<double>()),
                                        float(seq[1].cast<double>()),
                                        float(seq[2].cast<double>()))));
                } else if (typeName == "point3d" || typeName == "vec3d" ||
                           typeName == "double3") {
                    py::sequence seq = value;
                    if (seq.size() != 3) {
                        throw py::type_error("set_attr: point3d needs 3 values");
                    }
                    set(VtValue(GfVec3d(seq[0].cast<double>(),
                                        seq[1].cast<double>(),
                                        seq[2].cast<double>())));
                } else if (typeName == "matrix4d") {
                    py::sequence seq = value;
                    if (seq.size() != 16) {
                        throw py::type_error(
                            "set_attr: matrix4d needs a flat list of 16 values");
                    }
                    std::vector<double> v(16);
                    for (size_t i = 0; i < 16; ++i) {
                        v[i] = seq[i].cast<double>();
                    }
                    set(VtValue(_VecToMat4(v)));
                } else if (typeName == "float[]") {
                    py::sequence seq = value;
                    VtFloatArray arr(seq.size());
                    for (size_t i = 0; i < seq.size(); ++i) {
                        arr[i] = float(seq[i].cast<double>());
                    }
                    set(VtValue(arr));
                } else if (typeName == "double[]") {
                    py::sequence seq = value;
                    VtDoubleArray arr(seq.size());
                    for (size_t i = 0; i < seq.size(); ++i) {
                        arr[i] = seq[i].cast<double>();
                    }
                    set(VtValue(arr));
                } else if (typeName == "int[]") {
                    py::sequence seq = value;
                    VtIntArray arr(seq.size());
                    for (size_t i = 0; i < seq.size(); ++i) {
                        arr[i] = seq[i].cast<int>();
                    }
                    set(VtValue(arr));
                } else if (typeName == "point3f[]" || typeName == "vec3f[]") {
                    py::sequence rows = value;
                    VtArray<GfVec3f> arr(rows.size());
                    for (size_t i = 0; i < rows.size(); ++i) {
                        py::sequence row = rows[i];
                        if (row.size() != 3) {
                            throw py::type_error(
                                "set_attr: point3f[] rows need 3 values");
                        }
                        arr[i] = GfVec3f(float(row[0].cast<double>()),
                                         float(row[1].cast<double>()),
                                         float(row[2].cast<double>()));
                    }
                    set(VtValue(arr));
                } else if (typeName == "point3d[]" || typeName == "vec3d[]" ||
                           typeName == "double3[]") {
                    py::sequence rows = value;
                    VtArray<GfVec3d> arr(rows.size());
                    for (size_t i = 0; i < rows.size(); ++i) {
                        py::sequence row = rows[i];
                        if (row.size() != 3) {
                            throw py::type_error(
                                "set_attr: point3d[] rows need 3 values");
                        }
                        arr[i] = GfVec3d(row[0].cast<double>(),
                                         row[1].cast<double>(),
                                         row[2].cast<double>());
                    }
                    set(VtValue(arr));
                } else {
                    throw py::type_error("set_attr: unsupported type name: " + typeName);
                }
            }, py::arg("name"), py::arg("type_name"), py::arg("value"),
           "Author one attribute with an explicit Sdf type name (e.g. 'float',"
           " 'token', 'point3f[]'). The value is converted to the exact payload"
           " for that declared type.")
        .def("__repr__", [](const RigExecHandleBase &h) {
            return "<" + std::string(h.GetPrim().GetTypeName().GetString()) + " '" +
                   _PathStr(h.GetPath()) + "'>";
        });

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
        }, py::arg("rx"), py::arg("ry"), py::arg("rz"), py::arg("order") = "XYZ");

    // Solvers.
    py::class_<rigExec::RigExecSolverHandle, RigExecHandleBase>(m, "Solver")
        .def("set_joints", [](rigExec::RigExecSolverHandle &h, std::vector<std::string> paths) {
            std::vector<SdfPath> p;
            for (const auto &s : paths) {
                p.push_back(_StrToPath(s));
            }
            h.SetJoints(p);
        }, py::arg("paths"));

    py::class_<rigExec::RigExecFkChainHandle, rigExec::RigExecSolverHandle>(m, "FkChain")
        .def("set_controls", [](rigExec::RigExecFkChainHandle &h, std::vector<std::string> paths) {
            std::vector<SdfPath> p;
            for (const auto &s : paths) {
                p.push_back(_StrToPath(s));
            }
            h.SetControls(p);
        }, py::arg("paths"));

    py::class_<rigExec::RigExecTwoBoneIkHandle, rigExec::RigExecSolverHandle>(m, "TwoBoneIk")
        .def("set_root_control", [](rigExec::RigExecTwoBoneIkHandle &h, std::string p) { h.SetRootControl(_StrToPath(p)); }, py::arg("path"))
        .def("set_effector_control", [](rigExec::RigExecTwoBoneIkHandle &h, std::string p) { h.SetEffectorControl(_StrToPath(p)); }, py::arg("path"))
        .def("set_pole_control", [](rigExec::RigExecTwoBoneIkHandle &h, std::string p) { h.SetPoleControl(_StrToPath(p)); }, py::arg("path"))
        .def("set_stretch_policy", [](rigExec::RigExecTwoBoneIkHandle &h, std::string v) { h.SetStretchPolicy(TfToken(v)); }, py::arg("policy"))
        .def("set_unreachable_policy", [](rigExec::RigExecTwoBoneIkHandle &h, std::string v) { h.SetUnreachablePolicy(TfToken(v)); }, py::arg("policy"));

    py::class_<rigExec::RigExecBlendPointFramesHandle, rigExec::RigExecSolverHandle>(m, "BlendPointFrames")
        .def("set_input_a", [](rigExec::RigExecBlendPointFramesHandle &h, std::string p) { h.SetInputA(_StrToPath(p)); }, py::arg("path"))
        .def("set_input_b", [](rigExec::RigExecBlendPointFramesHandle &h, std::string p) { h.SetInputB(_StrToPath(p)); }, py::arg("path"))
        .def("set_weight", &rigExec::RigExecBlendPointFramesHandle::SetWeight, py::arg("weight"))
        .def("set_rotation_blend", [](rigExec::RigExecBlendPointFramesHandle &h, std::string v) { h.SetRotationBlend(TfToken(v)); }, py::arg("mode"))
        .def("set_scale_blend", [](rigExec::RigExecBlendPointFramesHandle &h, std::string v) { h.SetScaleBlend(TfToken(v)); }, py::arg("mode"));

    py::class_<rigExec::RigExecTwistDistributionHandle, rigExec::RigExecSolverHandle>(m, "TwistDistribution")
        .def("set_start", [](rigExec::RigExecTwistDistributionHandle &h, std::string p) { h.SetStart(_StrToPath(p)); }, py::arg("path"))
        .def("set_end", [](rigExec::RigExecTwistDistributionHandle &h, std::string p) { h.SetEnd(_StrToPath(p)); }, py::arg("path"))
        .def("set_count", &rigExec::RigExecTwistDistributionHandle::SetCount, py::arg("count"))
        .def("set_weights", [](rigExec::RigExecTwistDistributionHandle &h, std::vector<float> w) { h.SetWeights(w); }, py::arg("weights"))
        .def("set_distribution", [](rigExec::RigExecTwistDistributionHandle &h, std::string v) { h.SetDistribution(TfToken(v)); }, py::arg("mode"))
        .def("set_joint_elements", [](rigExec::RigExecTwistDistributionHandle &h, std::vector<int> e) { h.SetJointElements(e); }, py::arg("elements"));

    py::class_<rigExec::RigExecRibbonHandle, rigExec::RigExecSolverHandle>(m, "Ribbon")
        .def("set_driver_curve", [](rigExec::RigExecRibbonHandle &h, std::string p) { h.SetDriverCurve(_StrToPath(p)); }, py::arg("path"))
        .def("set_start_frame", [](rigExec::RigExecRibbonHandle &h, std::string p) { h.SetStartFrame(_StrToPath(p)); }, py::arg("path"))
        .def("set_end_frame", [](rigExec::RigExecRibbonHandle &h, std::string p) { h.SetEndFrame(_StrToPath(p)); }, py::arg("path"))
        .def("set_twist_frames", [](rigExec::RigExecRibbonHandle &h, std::vector<std::string> paths) {
            std::vector<SdfPath> p;
            for (const auto &s : paths) {
                p.push_back(_StrToPath(s));
            }
            h.SetTwistFrames(p);
        }, py::arg("paths"))
        .def("set_sample_count", &rigExec::RigExecRibbonHandle::SetSampleCount, py::arg("count"))
        .def("set_parameterization", [](rigExec::RigExecRibbonHandle &h, std::string v) { h.SetParameterization(TfToken(v)); }, py::arg("mode"))
        .def("set_driver_curve_read_phase", [](rigExec::RigExecRibbonHandle &h, std::string v) { h.SetDriverCurveReadPhase(TfToken(v)); }, py::arg("phase"))
        .def("set_surface_read_phase", [](rigExec::RigExecRibbonHandle &h, std::string v) { h.SetSurfaceReadPhase(TfToken(v)); }, py::arg("phase"))
        .def("set_joint_elements", [](rigExec::RigExecRibbonHandle &h, std::vector<int> e) { h.SetJointElements(e); }, py::arg("elements"));

    // Constraints.
    py::class_<rigExec::RigExecConstraintHandle, RigExecHandleBase>(m, "Constraint")
        .def("set_target", [](rigExec::RigExecConstraintHandle &h, std::string p) { h.SetTarget(_StrToPath(p)); }, py::arg("path"))
        .def("set_default_weight", &rigExec::RigExecConstraintHandle::SetDefaultWeight, py::arg("weight"))
        .def("set_weight_object", [](rigExec::RigExecConstraintHandle &h, std::string p) { h.SetWeightObject(_StrToPath(p)); }, py::arg("path"))
        .def("set_translation_offset", &rigExec::RigExecConstraintHandle::SetTranslationOffset,
             py::arg("x"), py::arg("y"), py::arg("z"))
        .def("set_rotation_offset", &rigExec::RigExecConstraintHandle::SetRotationOffset,
             py::arg("x"), py::arg("y"), py::arg("z"))
        .def("set_scale_offset", &rigExec::RigExecConstraintHandle::SetScaleOffset,
             py::arg("x"), py::arg("y"), py::arg("z"));

    py::class_<rigExec::RigExecSourceConstraintHandle, rigExec::RigExecConstraintHandle>(m, "SourceConstraint")
        .def("set_sources", [](rigExec::RigExecSourceConstraintHandle &h, std::vector<std::string> paths) {
            std::vector<SdfPath> p;
            for (const auto &s : paths) {
                p.push_back(_StrToPath(s));
            }
            h.SetSources(p);
        }, py::arg("paths"))
        .def("set_sources_weighted", [](rigExec::RigExecSourceConstraintHandle &h, std::vector<std::string> paths, std::vector<float> weights) {
            std::vector<SdfPath> p;
            for (const auto &s : paths) {
                p.push_back(_StrToPath(s));
            }
            h.SetSources(p, weights);
        }, py::arg("paths"), py::arg("weights"));

    py::class_<rigExec::RigExecAimConstraintHandle, rigExec::RigExecSourceConstraintHandle>(m, "AimConstraint")
        .def("set_aim_vector", &rigExec::RigExecAimConstraintHandle::SetAimVector,
             py::arg("x"), py::arg("y"), py::arg("z"))
        .def("set_up_vector", &rigExec::RigExecAimConstraintHandle::SetUpVector,
             py::arg("x"), py::arg("y"), py::arg("z"))
        .def("set_aim_target", [](rigExec::RigExecAimConstraintHandle &h, std::string p) { h.SetAimTarget(_StrToPath(p)); }, py::arg("path"))
        .def("set_world_up_object", [](rigExec::RigExecAimConstraintHandle &h, std::string p) { h.SetWorldUpObject(_StrToPath(p)); }, py::arg("path"))
        .def("set_world_up_type", [](rigExec::RigExecAimConstraintHandle &h, std::string v) { h.SetWorldUpType(TfToken(v)); }, py::arg("type"));

    py::class_<rigExec::RigExecPositionConstraintHandle, rigExec::RigExecSourceConstraintHandle>(m, "PositionConstraint");
    py::class_<rigExec::RigExecRotationConstraintHandle, rigExec::RigExecSourceConstraintHandle>(m, "RotationConstraint");
    py::class_<rigExec::RigExecScaleConstraintHandle, rigExec::RigExecSourceConstraintHandle>(m, "ScaleConstraint");
    py::class_<rigExec::RigExecParentConstraintHandle, rigExec::RigExecSourceConstraintHandle>(m, "ParentConstraint")
        // The parent constraint reads PER-SOURCE offset arrays (double3[],
        // parallel to set_sources); the inherited scalar setters do not apply.
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
        .def("set_first_joint", [](rigExec::RigExecSingleChainIkConstraintHandle &h, std::string p) { h.SetFirstJoint(_StrToPath(p)); }, py::arg("path"))
        .def("set_end_joint", [](rigExec::RigExecSingleChainIkConstraintHandle &h, std::string p) { h.SetEndJoint(_StrToPath(p)); }, py::arg("path"))
        .def("set_effector", [](rigExec::RigExecSingleChainIkConstraintHandle &h, std::string p) { h.SetEffector(_StrToPath(p)); }, py::arg("path"))
        .def("set_moves", [](rigExec::RigExecSingleChainIkConstraintHandle &h, std::vector<std::string> paths) {
            std::vector<SdfPath> out;
            for (const auto &p : paths) { out.push_back(_StrToPath(p)); }
            h.SetMoves(out);
        }, py::arg("paths"))
        .def("set_pole_vector_objects", [](rigExec::RigExecSingleChainIkConstraintHandle &h, std::vector<std::string> paths) {
            std::vector<SdfPath> p;
            for (const auto &s : paths) {
                p.push_back(_StrToPath(s));
            }
            h.SetPoleVectorObjects(p);
        }, py::arg("paths"))
        .def("set_solver_mode", [](rigExec::RigExecSingleChainIkConstraintHandle &h, std::string v) { h.SetSolverMode(TfToken(v)); }, py::arg("mode"))
        .def("set_pole_vector_mode", [](rigExec::RigExecSingleChainIkConstraintHandle &h, std::string v) { h.SetPoleVectorMode(TfToken(v)); }, py::arg("mode"));

    // Weight objects.
    py::class_<rigExec::RigExecWeightHandle, RigExecHandleBase>(m, "Weight")
        .def("set_target", [](rigExec::RigExecWeightHandle &h, std::string p) { h.SetTarget(_StrToPath(p)); }, py::arg("path"))
        .def("set_representation", [](rigExec::RigExecWeightHandle &h, std::string v) { h.SetRepresentation(TfToken(v)); }, py::arg("rep"))
        .def("set_range_policy", [](rigExec::RigExecWeightHandle &h, std::string v) { h.SetRangePolicy(TfToken(v)); }, py::arg("policy"));

    py::class_<rigExec::RigExecStaticWeightHandle, rigExec::RigExecWeightHandle>(m, "StaticWeight")
        .def("set_values", [](rigExec::RigExecStaticWeightHandle &h, std::vector<float> v) { h.SetValues(v); }, py::arg("values"))
        .def("set_indices", [](rigExec::RigExecStaticWeightHandle &h, std::vector<int> i) { h.SetIndices(i); }, py::arg("indices"))
        .def("set_default_weight", &rigExec::RigExecStaticWeightHandle::SetDefaultWeight, py::arg("weight"));

    py::class_<rigExec::RigExecDynamicWeightHandle, rigExec::RigExecWeightHandle>(m, "DynamicWeight")
        .def("set_base_weight", [](rigExec::RigExecDynamicWeightHandle &h, std::string p) { h.SetBaseWeight(_StrToPath(p)); }, py::arg("path"))
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
        .def("set_falloff", &rigExec::RigExecVolumeWeightHandle::SetFalloff,
             py::arg("falloff_min"), py::arg("falloff_max"))
        .def("set_invert", &rigExec::RigExecVolumeWeightHandle::SetInvert, py::arg("invert"))
        .def("set_strength", &rigExec::RigExecVolumeWeightHandle::SetStrength, py::arg("strength"))
        .def("set_falloff_profile", [](rigExec::RigExecVolumeWeightHandle &h, std::string v) { h.SetFalloffProfile(TfToken(v)); }, py::arg("profile"))
        .def("set_falloff_curve", [](rigExec::RigExecVolumeWeightHandle &h, std::vector<std::pair<double, double>> knots) {
            h.SetFalloffCurve(knots);
        }, py::arg("knots"), "A list of (x, y) pairs over x in [0, 1].")
        .def("set_sample_phase", [](rigExec::RigExecVolumeWeightHandle &h, std::string v) { h.SetSamplePhase(TfToken(v)); }, py::arg("phase"))
        .def("set_sample_source", [](rigExec::RigExecVolumeWeightHandle &h, std::string p) { h.SetSampleSource(_StrToPath(p)); }, py::arg("path"));

    py::class_<rigExec::RigExecSphereWeightHandle, rigExec::RigExecVolumeWeightHandle>(m, "SphereWeight")
        .def("set_scales", &rigExec::RigExecSphereWeightHandle::SetScales,
             py::arg("sx"), py::arg("sy"), py::arg("sz"));

    py::class_<rigExec::RigExecPlaneWeightHandle, rigExec::RigExecVolumeWeightHandle>(m, "PlaneWeight")
        .def("set_axis", [](rigExec::RigExecPlaneWeightHandle &h, std::string v) { h.SetAxis(TfToken(v)); }, py::arg("axis"))
        .def("set_bounds", [](rigExec::RigExecPlaneWeightHandle &h, std::string v) { h.SetBounds(TfToken(v)); }, py::arg("bounds"))
        .def("set_extents", &rigExec::RigExecPlaneWeightHandle::SetExtents,
             py::arg("extent_u"), py::arg("extent_v"));

    py::class_<rigExec::RigExecCurveWeightHandle, rigExec::RigExecVolumeWeightHandle>(m, "CurveWeight")
        .def("set_curve", [](rigExec::RigExecCurveWeightHandle &h, std::string p) { h.SetCurve(_StrToPath(p)); }, py::arg("path"))
        .def("set_scales", &rigExec::RigExecCurveWeightHandle::SetScales,
             py::arg("sx"), py::arg("sy"), py::arg("sz"));

    py::class_<rigExec::RigExecCombineWeightHandle, rigExec::RigExecWeightHandle>(m, "CombineWeight")
        .def("set_input_weights", [](rigExec::RigExecCombineWeightHandle &h, std::vector<std::string> paths) {
            std::vector<SdfPath> p;
            for (const auto &s : paths) {
                p.push_back(_StrToPath(s));
            }
            h.SetInputWeights(p);
        }, py::arg("paths"))
        .def("set_combine_mode", [](rigExec::RigExecCombineWeightHandle &h, std::string v) { h.SetCombineMode(TfToken(v)); }, py::arg("mode"))
        .def("set_strength", &rigExec::RigExecCombineWeightHandle::SetStrength, py::arg("strength"))
        .def("set_invert", &rigExec::RigExecCombineWeightHandle::SetInvert, py::arg("invert"));

    // Blend inputs / samples.
    py::class_<rigExec::RigExecBlendSampleHandle, RigExecHandleBase>(m, "BlendSample")
        .def("set_activation", &rigExec::RigExecBlendSampleHandle::SetActivation, py::arg("activation"))
        .def("set_target_points", [](rigExec::RigExecBlendSampleHandle &h, std::string p) { h.SetTargetPoints(_StrToPath(p)); }, py::arg("path"))
        .def("set_read_phase", [](rigExec::RigExecBlendSampleHandle &h, std::string v) { h.SetReadPhase(TfToken(v)); }, py::arg("phase"));

    py::class_<rigExec::RigExecBlendInputHandle, RigExecHandleBase>(m, "BlendInput")
        .def("set_weight", &rigExec::RigExecBlendInputHandle::SetWeight, py::arg("weight"))
        .def("add_sample", [](rigExec::RigExecBlendInputHandle &h, std::string name, float activation) {
            return h.AddSample(name, activation);
        }, py::arg("name"), py::arg("activation") = 1.0f);

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
    py::class_<rigExec::RigExecMatrixMoverHandle, RigExecHandleBase>(m, "MatrixMover")
        .def("set_transform_provider", [](rigExec::RigExecMatrixMoverHandle &h, std::string p) { h.SetTransformProvider(_StrToPath(p)); }, py::arg("path"))
        .def("set_weight_object", [](rigExec::RigExecMatrixMoverHandle &h, std::string p) { h.SetWeightObject(_StrToPath(p)); }, py::arg("path"))
        .def("set_read_phase", [](rigExec::RigExecMatrixMoverHandle &h, std::string v) { h.SetReadPhase(TfToken(v)); }, py::arg("phase"));

    py::class_<rigExec::RigExecLatticeMoverHandle, RigExecHandleBase>(m, "LatticeMover")
        .def("set_cage", [](rigExec::RigExecLatticeMoverHandle &h, std::string p) { h.SetCage(_StrToPath(p)); }, py::arg("path"))
        .def("set_basis", [](rigExec::RigExecLatticeMoverHandle &h, std::string v) { h.SetBasis(TfToken(v)); }, py::arg("basis"))
        .def("set_divisions", &rigExec::RigExecLatticeMoverHandle::SetDivisions,
             py::arg("x"), py::arg("y"), py::arg("z"))
        .def("set_read_phase", [](rigExec::RigExecLatticeMoverHandle &h, std::string v) { h.SetReadPhase(TfToken(v)); }, py::arg("phase"));

    py::class_<rigExec::RigExecBlendShapeMoverHandle, RigExecHandleBase>(m, "BlendShapeMover")
        .def("add_blend_input", [](rigExec::RigExecBlendShapeMoverHandle &h, std::string name, float weight) {
            return h.AddBlendInput(name, weight);
        }, py::arg("name"), py::arg("weight") = 0.0f)
        .def("set_weight_object", [](rigExec::RigExecBlendShapeMoverHandle &h, std::string p) { h.SetWeightObject(_StrToPath(p)); }, py::arg("path"));

    py::class_<rigExec::RigExecCurveMoverHandle, RigExecHandleBase>(m, "CurveMover")
        .def("set_driver_curve", [](rigExec::RigExecCurveMoverHandle &h, std::string p) { h.SetDriverCurve(_StrToPath(p)); }, py::arg("path"))
        .def("set_driver_frames", [](rigExec::RigExecCurveMoverHandle &h, std::vector<std::string> paths) {
            std::vector<SdfPath> ps;
            ps.reserve(paths.size());
            for (const auto &p : paths) {
                ps.push_back(_StrToPath(p));
            }
            h.SetDriverFrames(ps);
        }, py::arg("paths"))
        .def("set_bind_coordinates", [](rigExec::RigExecCurveMoverHandle &h, std::string p) { h.SetBindCoordinates(_StrToPath(p)); }, py::arg("path"))
        .def("set_mode", [](rigExec::RigExecCurveMoverHandle &h, std::string v) { h.SetMode(TfToken(v)); }, py::arg("mode"))
        .def("set_read_phase", [](rigExec::RigExecCurveMoverHandle &h, std::string v) { h.SetReadPhase(TfToken(v)); }, py::arg("phase"));

    py::class_<rigExec::RigExecSurfaceMoverHandle, RigExecHandleBase>(m, "SurfaceMover")
        .def("set_surface", [](rigExec::RigExecSurfaceMoverHandle &h, std::string p) { h.SetSurface(_StrToPath(p)); }, py::arg("path"))
        .def("set_mode", [](rigExec::RigExecSurfaceMoverHandle &h, std::string v) { h.SetMode(TfToken(v)); }, py::arg("mode"))
        .def("set_read_phase", [](rigExec::RigExecSurfaceMoverHandle &h, std::string v) { h.SetReadPhase(TfToken(v)); }, py::arg("phase"));

    py::class_<rigExec::RigExecSmoothMoverHandle, RigExecHandleBase>(m, "SmoothMover")
        .def("set_strength", &rigExec::RigExecSmoothMoverHandle::SetStrength, py::arg("strength"));

    py::class_<rigExec::RigExecVolumeCorrectMoverHandle, RigExecHandleBase>(m, "VolumeCorrectMover")
        .def("set_strength", &rigExec::RigExecVolumeCorrectMoverHandle::SetStrength, py::arg("strength"));

    py::class_<rigExec::RigExecCurvenetMoverHandle, RigExecHandleBase>(m, "CurvenetMover")
        .def("set_curvenet", [](rigExec::RigExecCurvenetMoverHandle &h, std::string p) { h.SetCurvenet(_StrToPath(p)); }, py::arg("path"))
        .def("set_strength", &rigExec::RigExecCurvenetMoverHandle::SetStrength, py::arg("strength"));

    py::class_<rigExec::RigExecFloatMathMoverHandle, RigExecHandleBase>(m, "FloatMathMover")
        .def("set_operation", [](rigExec::RigExecFloatMathMoverHandle &h, std::string v) { h.SetOperation(TfToken(v)); }, py::arg("op"))
        .def("set_value", &rigExec::RigExecFloatMathMoverHandle::SetValue, py::arg("value"))
        .def("set_bounds", &rigExec::RigExecFloatMathMoverHandle::SetBounds, py::arg("min"), py::arg("max"))
        .def("set_weight", &rigExec::RigExecFloatMathMoverHandle::SetWeight, py::arg("weight"));

    py::class_<rigExec::RigExecVec3fMathMoverHandle, RigExecHandleBase>(m, "Vec3fMathMover")
        .def("set_operation", [](rigExec::RigExecVec3fMathMoverHandle &h, std::string v) { h.SetOperation(TfToken(v)); }, py::arg("op"))
        .def("set_value", [](rigExec::RigExecVec3fMathMoverHandle &h, std::array<double, 3> v) {
            h.SetValue(GfVec3f(float(v[0]), float(v[1]), float(v[2])));
        }, py::arg("value"))
        .def("set_bounds", [](rigExec::RigExecVec3fMathMoverHandle &h, std::array<double, 3> mn, std::array<double, 3> mx) {
            h.SetBounds(GfVec3f(float(mn[0]), float(mn[1]), float(mn[2])),
                        GfVec3f(float(mx[0]), float(mx[1]), float(mx[2])));
        }, py::arg("min"), py::arg("max"))
        .def("set_weight", &rigExec::RigExecVec3fMathMoverHandle::SetWeight, py::arg("weight"));

    py::class_<rigExec::RigExecMatrixMathMoverHandle, RigExecHandleBase>(m, "MatrixMathMover")
        .def("set_operation", [](rigExec::RigExecMatrixMathMoverHandle &h, std::string v) { h.SetOperation(TfToken(v)); }, py::arg("op"))
        .def("set_value", [](rigExec::RigExecMatrixMathMoverHandle &h, std::vector<double> m) {
            h.SetValue(_VecToMat4(m));
        }, py::arg("matrix"))
        .def("set_weight", &rigExec::RigExecMatrixMathMoverHandle::SetWeight, py::arg("weight"));

    // Mover chain.
    py::class_<rigExec::RigExecMoverChain>(m, "MoverChain",
        "One mover chain: operations added here apply in REVERSE add order\n(last added runs first); add outermost passes first.")
        .def_property_readonly("scope_path", [](const rigExec::RigExecMoverChain &c) { return _PathStr(c.GetScopePath()); })
        .def("add_matrix_mover", [](rigExec::RigExecMoverChain &c, std::string name, std::string transformProvider, std::string weightObject, std::string target, std::string readPhase) {
            return c.AddMatrixMover(name, _StrToPath(transformProvider), _StrToPath(weightObject), _StrToPath(target), TfToken(readPhase));
        }, py::arg("name"), py::arg("transform_provider"), py::arg("weight_object"),
           py::arg("target") = "", py::arg("read_phase") = "base")
        .def("add_lattice_mover", [](rigExec::RigExecMoverChain &c, std::string name, std::string cagePrim, int dx, int dy, int dz, std::string basis, std::string target, std::string readPhase) {
            return c.AddLatticeMover(name, _StrToPath(cagePrim), dx, dy, dz, TfToken(basis), _StrToPath(target), TfToken(readPhase));
        }, py::arg("name"), py::arg("cage_prim"), py::arg("div_x"), py::arg("div_y"), py::arg("div_z"),
           py::arg("basis") = "bspline", py::arg("target") = "", py::arg("read_phase") = "base")
        .def("add_blend_shape_mover", [](rigExec::RigExecMoverChain &c, std::string name, std::string weightObject, std::string target) {
            return c.AddBlendShapeMover(name, _StrToPath(weightObject), _StrToPath(target));
        }, py::arg("name"), py::arg("weight_object") = "", py::arg("target") = "")
        .def("add_curve_mover", [](rigExec::RigExecMoverChain &c, std::string name, std::string driverCurve, std::string driverFrames, std::string bindCoordinates, std::string mode, std::string target, std::string readPhase) {
            return c.AddCurveMover(name, _StrToPath(driverCurve), _StrToPath(driverFrames), _StrToPath(bindCoordinates), TfToken(mode), _StrToPath(target), TfToken(readPhase));
        }, py::arg("name"), py::arg("driver_curve"), py::arg("driver_frames") = "",
           py::arg("bind_coordinates") = "", py::arg("mode") = "ribbon",
           py::arg("target") = "", py::arg("read_phase") = "base")
        .def("add_surface_mover", [](rigExec::RigExecMoverChain &c, std::string name, std::string surfacePrim, std::string mode, std::string target, std::string readPhase) {
            return c.AddSurfaceMover(name, _StrToPath(surfacePrim), TfToken(mode), _StrToPath(target), TfToken(readPhase));
        }, py::arg("name"), py::arg("surface_prim"), py::arg("mode") = "attach",
           py::arg("target") = "", py::arg("read_phase") = "base")
        .def("add_smooth_mover", [](rigExec::RigExecMoverChain &c, std::string name, float strength, std::string target) {
            return c.AddSmoothMover(name, strength, _StrToPath(target));
        }, py::arg("name"), py::arg("strength"), py::arg("target") = "")
        .def("add_volume_correct_mover", [](rigExec::RigExecMoverChain &c, std::string name, float strength, std::string target) {
            return c.AddVolumeCorrectMover(name, strength, _StrToPath(target));
        }, py::arg("name"), py::arg("strength"), py::arg("target") = "")
        .def("add_curvenet_mover", [](rigExec::RigExecMoverChain &c, std::string name, std::string curvenetPrim, float strength, std::string target) {
            return c.AddCurvenetMover(name, _StrToPath(curvenetPrim), strength, _StrToPath(target));
        }, py::arg("name"), py::arg("curvenet_prim"), py::arg("strength"), py::arg("target") = "")
        .def("add_float_math_mover", [](rigExec::RigExecMoverChain &c, std::string name, std::string operation, float value, std::string target, float weight) {
            return c.AddFloatMathMover(name, TfToken(operation), value, _StrToPath(target), weight);
        }, py::arg("name"), py::arg("operation"), py::arg("value"), py::arg("target") = "", py::arg("weight") = 1.0f)
        .def("add_vec3f_math_mover", [](rigExec::RigExecMoverChain &c, std::string name, std::string operation, std::array<double, 3> value, std::string target, float weight) {
            return c.AddVec3fMathMover(name, TfToken(operation), GfVec3f(float(value[0]), float(value[1]), float(value[2])), _StrToPath(target), weight);
        }, py::arg("name"), py::arg("operation"), py::arg("value"), py::arg("target") = "", py::arg("weight") = 1.0f)
        .def("add_matrix_math_mover", [](rigExec::RigExecMoverChain &c, std::string name, std::string operation, std::vector<double> value, std::string target, float weight) {
            return c.AddMatrixMathMover(name, TfToken(operation), _VecToMat4(value), _StrToPath(target), weight);
        }, py::arg("name"), py::arg("operation"), py::arg("matrix"), py::arg("target") = "", py::arg("weight") = 1.0f)
        .def("add_aim_constraint", [](rigExec::RigExecMoverChain &c, std::string name, std::string target) {
            return c.AddAimConstraint(name, _StrToPath(target));
        }, py::arg("name"), py::arg("target") = "")
        .def("add_position_constraint", [](rigExec::RigExecMoverChain &c, std::string name, std::string target) {
            return c.AddPositionConstraint(name, _StrToPath(target));
        }, py::arg("name"), py::arg("target") = "")
        .def("add_rotation_constraint", [](rigExec::RigExecMoverChain &c, std::string name, std::string target) {
            return c.AddRotationConstraint(name, _StrToPath(target));
        }, py::arg("name"), py::arg("target") = "")
        .def("add_scale_constraint", [](rigExec::RigExecMoverChain &c, std::string name, std::string target) {
            return c.AddScaleConstraint(name, _StrToPath(target));
        }, py::arg("name"), py::arg("target") = "")
        .def("add_parent_constraint", [](rigExec::RigExecMoverChain &c, std::string name, std::string target) {
            return c.AddParentConstraint(name, _StrToPath(target));
        }, py::arg("name"), py::arg("target") = "")
        .def("add_single_chain_ik_constraint", [](rigExec::RigExecMoverChain &c, std::string name, std::string target) {
            return c.AddSingleChainIkConstraint(name, _StrToPath(target));
        }, py::arg("name"), py::arg("target") = "");

    // Builder.
    py::class_<rigExec::RigExecRigBuilder>(m, "Builder",
        "The top-level rig builder: one per (stage, rig root). Creates and wires the prims a RigExec rig is made of.")
        .def_static("create", [](py::object stageObj, std::string rigRoot, std::string partition) {
            auto s = _ExtractStage(stageObj);
            if (!s) {
                throw py::type_error(
                    "Builder.create() needs a pxr.Usd.Stage instance");
            }
            return rigExec::RigExecRigBuilder::Create(s, SdfPath(rigRoot), TfToken(partition));
        }, py::arg("stage"), py::arg("rig_root") = "/Rig", py::arg("partition") = "")
        .def_property_readonly("root_path", [](const rigExec::RigExecRigBuilder &b) { return _PathStr(b.GetRootPath()); })

        // Transform providers.
        .def("add_control", [](rigExec::RigExecRigBuilder &b, std::string name, std::vector<double> restSpace) {
            return b.AddControl(name, restSpace.empty() ? GfMatrix4d() : _VecToMat4(restSpace));
        }, py::arg("name"), py::arg("rest_space") = py::list())
        .def("add_joint", [](rigExec::RigExecRigBuilder &b, std::string name, std::vector<double> restSpace, const rigExec::RigExecJointHandle *parent) {
            return b.AddJoint(name, restSpace.empty() ? GfMatrix4d() : _VecToMat4(restSpace), parent);
        }, py::arg("name"), py::arg("rest_space") = py::list(), py::arg("parent_joint") = nullptr)

        // Solvers.
        .def("add_fk_chain", &rigExec::RigExecRigBuilder::AddFkChain, py::arg("name"))
        .def("add_two_bone_ik", [](rigExec::RigExecRigBuilder &b, std::string name, std::string rootControl, std::string effectorControl, std::string poleControl) {
            return b.AddTwoBoneIk(name, _StrToPath(rootControl), _StrToPath(effectorControl), _StrToPath(poleControl));
        }, py::arg("name"), py::arg("root_control") = "", py::arg("effector_control") = "", py::arg("pole_control") = "")
        .def("add_blend_point_frames", [](rigExec::RigExecRigBuilder &b, std::string name, std::string inputA, std::string inputB, float weight) {
            return b.AddBlendPointFrames(name, _StrToPath(inputA), _StrToPath(inputB), weight);
        }, py::arg("name"), py::arg("input_a") = "", py::arg("input_b") = "", py::arg("weight") = 0.0f)
        .def("add_twist_distribution", [](rigExec::RigExecRigBuilder &b, std::string name, std::string start, std::string end, int count) {
            return b.AddTwistDistribution(name, _StrToPath(start), _StrToPath(end), count);
        }, py::arg("name"), py::arg("start") = "", py::arg("end") = "", py::arg("count") = 1)
        .def("add_ribbon", [](rigExec::RigExecRigBuilder &b, std::string name, std::string driverCurve, int sampleCount) {
            return b.AddRibbon(name, _StrToPath(driverCurve), sampleCount);
        }, py::arg("name"), py::arg("driver_curve") = "", py::arg("sample_count") = 5)

        // Weight objects.
        .def("add_static_weight", [](rigExec::RigExecRigBuilder &b, std::string name, std::string target, std::vector<float> values, std::vector<int> indices, float defaultWeight) {
            return b.AddStaticWeight(name, _StrToPath(target), values, indices, defaultWeight);
        }, py::arg("name"), py::arg("target") = "", py::arg("values") = py::list(),
           py::arg("indices") = py::list(), py::arg("default_weight") = 0.0f)
        .def("add_dynamic_weight", [](rigExec::RigExecRigBuilder &b, std::string name, std::string target, std::string baseWeight) {
            return b.AddDynamicWeight(name, _StrToPath(target), _StrToPath(baseWeight));
        }, py::arg("name"), py::arg("target") = "", py::arg("base_weight") = "")
        .def("add_sphere_weight", [](rigExec::RigExecRigBuilder &b, std::string name, std::string target, float falloffMin, float falloffMax) {
            return b.AddSphereWeight(name, _StrToPath(target), falloffMin, falloffMax);
        }, py::arg("name"), py::arg("target") = "", py::arg("falloff_min") = 0.0f, py::arg("falloff_max") = 1.0f)
        .def("add_plane_weight", [](rigExec::RigExecRigBuilder &b, std::string name, std::string target, float falloffMin, float falloffMax) {
            return b.AddPlaneWeight(name, _StrToPath(target), falloffMin, falloffMax);
        }, py::arg("name"), py::arg("target") = "", py::arg("falloff_min") = 0.0f, py::arg("falloff_max") = 1.0f)
        .def("add_curve_weight", [](rigExec::RigExecRigBuilder &b, std::string name, std::string target, std::string curve, float falloffMin, float falloffMax) {
            return b.AddCurveWeight(name, _StrToPath(target), _StrToPath(curve), falloffMin, falloffMax);
        }, py::arg("name"), py::arg("target") = "", py::arg("curve") = "", py::arg("falloff_min") = 0.0f, py::arg("falloff_max") = 1.0f)
        .def("add_combine_weight", [](rigExec::RigExecRigBuilder &b, std::string name, std::string target, std::vector<std::string> inputWeights, std::string mode) {
            std::vector<SdfPath> p;
            for (const auto &s : inputWeights) {
                p.push_back(_StrToPath(s));
            }
            return b.AddCombineWeight(name, _StrToPath(target), p, TfToken(mode));
        }, py::arg("name"), py::arg("target") = "", py::arg("input_weights") = py::list(), py::arg("mode") = "multiply")

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
                std::string defaultTarget) {
                 return b.NewMoverChain(name, _StrToPath(defaultTarget));
             }, py::arg("name"), py::arg("default_target") = "",
             "Start a mover chain; operations added without an explicit target"
             " reuse default_target.");
}
