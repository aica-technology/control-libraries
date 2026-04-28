#include "state_representation_bindings.hpp"

#include <state_representation/exceptions.hpp>

void bind_exceptions(py::module_& m) {
  py::register_exception<exceptions::Exception>(m, "Error", PyExc_RuntimeError);
  py::object error = m.attr("Error");
  py::register_exception<exceptions::EmptyStateException>(m, "EmptyStateError", error.ptr());
  py::register_exception<exceptions::IncompatibleReferenceFramesException>(m, "IncompatibleReferenceFramesError", error.ptr());
  py::register_exception<exceptions::IncompatibleSizeException>(m, "IncompatibleSizeError", error.ptr());
  py::register_exception<exceptions::IncompatibleStatesException>(m, "IncompatibleStatesError", error.ptr());
  py::register_exception<exceptions::InvalidCastException>(m, "InvalidCastError", error.ptr());
  py::register_exception<exceptions::InvalidParameterException>(m, "InvalidParameterError", error.ptr());
  py::register_exception<exceptions::InvalidStateVariableException>(m, "InvalidStateVariableError", error.ptr());
  py::register_exception<exceptions::IONotFoundException>(m, "IONotFoundError", error.ptr());
  py::register_exception<exceptions::JointNotFoundException>(m, "JointNotFoundError", error.ptr());
  py::register_exception<exceptions::NotImplementedException>(m, "NotImplementedError", error.ptr());
}
