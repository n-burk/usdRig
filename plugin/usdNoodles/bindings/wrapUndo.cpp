//
// Copyright (c) Meta Platforms, Inc. and affiliates.
//
// Licensed under the terms set forth in the LICENSE.txt file available
// at the root of this repository.
//

#include "undo/CompoundCommand.h"
#include "undo/LambdaCommand.h"
#include "undo/NoodlesUndoManager.h"

#include "pxr/pxr.h"

// usd24
  #include "pxr/external/boost/python.hpp"
  #include "pxr/external/boost/python/noncopyable.hpp"

#include <memory>
#include <utility>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace pxr_boost::python;
using namespace noodles;

namespace {

/// Python wrapper for LambdaCommand that holds boost::python callable objects.
///
/// These are the only Python objects that reach the undo manager, and the
/// manager outlives the interpreter -- see the destructor.
class PyLambdaCommand : public Command {
 public:
  PyLambdaCommand(std::string description, object doFunc, object undoFunc)
      : _description(std::move(description)),
        _callables(new Callables{std::move(doFunc), std::move(undoFunc)}) {}

  ~PyLambdaCommand() override {
    // NoodlesUndoManager::instance() is a function-local static, so its stacks
    // are destroyed during __cxa_finalize -- which runs AFTER Py_Finalize has
    // torn the interpreter down. Dropping a Python reference at that point
    // calls PyThreadState_Get with no thread state, and the process dies on
    // the way out of an otherwise clean quit. It reproduces in two lines:
    // push one command from Python, exit.
    //
    // Once the interpreter is gone there is nothing left to release -- it has
    // already freed every object these handles name -- so abandon them
    // instead of decref'ing into freed memory. This leaks only at process
    // exit, and only for commands still on the stack.
    //
    // The check has to be here rather than in noodles: the undo manager is
    // plain C++ that knows nothing about Python, and this wrapper is what
    // couples a Python lifetime to it.
    if (!Py_IsInitialized()) {
      (void)_callables.release();
    }
  }

  void execute() override {
    if (_callables && _callables->doFunc && !_callables->doFunc.is_none()) {
      _callables->doFunc();
    }
  }

  void undo() override {
    if (_callables && _callables->undoFunc && !_callables->undoFunc.is_none()) {
      _callables->undoFunc();
    }
  }

  std::string description() const override {
    return _description;
  }

 private:
  // Held indirectly so the destructor can abandon the references without
  // destroying them; an `object` member would decref unconditionally.
  struct Callables {
    object doFunc;
    object undoFunc;
  };

  std::string _description;
  std::unique_ptr<Callables> _callables;
};

/// Python wrapper for CompoundCommand that accepts Python Command objects.
class PyCompoundCommand {
 public:
  explicit PyCompoundCommand(const std::string& description)
      : _cmd(std::make_shared<CompoundCommand>(description)) {}

  void addCommand(object pyDoFunc, object pyUndoFunc, const std::string& desc) {
    _cmd->addCommand(
        std::make_unique<PyLambdaCommand>(desc, std::move(pyDoFunc), std::move(pyUndoFunc)));
  }

  void pushToManager() {
    // Swap ownership: Python retains _cmd (now empty), while we transfer the
    // original to the undo manager.  boost::python objects can't transfer
    // unique_ptr ownership, so we wrap the shared_ptr in a thin forwarding
    // Command that the manager owns via unique_ptr.
    auto cmd = std::make_shared<CompoundCommand>(_cmd->description());
    std::swap(cmd, _cmd);

    // Wrap in a CommandPtr via a thin forwarding command
    struct SharedCommand : public Command {
      std::shared_ptr<CompoundCommand> inner;
      explicit SharedCommand(std::shared_ptr<CompoundCommand> c) : inner(std::move(c)) {}
      void execute() override {
        inner->execute();
      }
      void undo() override {
        inner->undo();
      }
      std::string description() const override {
        return inner->description();
      }
    };

    NoodlesUndoManager::instance().pushCommand(std::make_unique<SharedCommand>(std::move(cmd)));
  }

  std::string description() const {
    return _cmd->description();
  }

  bool empty() const {
    return _cmd->empty();
  }

  size_t size() const {
    return _cmd->size();
  }

 private:
  std::shared_ptr<CompoundCommand> _cmd;
};

/// Helper to push a LambdaCommand directly from Python.
void pushLambdaCommand(const std::string& description, object doFunc, object undoFunc) {
  NoodlesUndoManager::instance().pushCommand(
      std::make_unique<PyLambdaCommand>(description, std::move(doFunc), std::move(undoFunc)));
}

} // namespace

void wrapUndo() {
  class_<NoodlesUndoManager, noncopyable>("NoodlesUndoManager", no_init)
      .def(
          "instance",
          &NoodlesUndoManager::instance,
          return_value_policy<reference_existing_object>())
      .staticmethod("instance")
      .def("undo", &NoodlesUndoManager::undo)
      .def("redo", &NoodlesUndoManager::redo)
      .def("canUndo", &NoodlesUndoManager::canUndo)
      .def("canRedo", &NoodlesUndoManager::canRedo)
      .def("clear", &NoodlesUndoManager::clear)
      .def("undoDescription", &NoodlesUndoManager::undoDescription)
      .def("redoDescription", &NoodlesUndoManager::redoDescription)
      .def("setMaxStackDepth", &NoodlesUndoManager::setMaxStackDepth);

  class_<PyCompoundCommand, noncopyable>("CompoundCommand", init<std::string>())
      .def("addCommand", &PyCompoundCommand::addCommand)
      .def("pushToManager", &PyCompoundCommand::pushToManager)
      .def("description", &PyCompoundCommand::description)
      .def("empty", &PyCompoundCommand::empty)
      .def("size", &PyCompoundCommand::size);

  def("pushLambdaCommand", &pushLambdaCommand);
}
