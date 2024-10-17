#include "state_representation_bindings.hpp"

#include <state_representation/exceptions/EmptyStateException.hpp>
#include <state_representation/exceptions/InvalidStateVariableException.hpp>
#include <state_representation/exceptions/IncompatibleReferenceFramesException.hpp>
#include <state_representation/exceptions/IncompatibleSizeException.hpp>
#include <state_representation/exceptions/IncompatibleStatesException.hpp>
#include <state_representation/exceptions/InvalidCastException.hpp>
#include <state_representation/exceptions/InvalidParameterException.hpp>
#include <state_representation/exceptions/JointNotFoundException.hpp>
#include <state_representation/exceptions/NotImplementedException.hpp>

void bind_exceptions(py::module_& m) {
  py::register_exception<exceptions::EmptyStateException>(m, "EmptyStateError", PyExc_RuntimeError);
  py::register_exception<exceptions::EmptyStateException>(m, "InvalidStateVariableException", PyExc_RuntimeError);
  py::register_exception<exceptions::IncompatibleReferenceFramesException>(m, "IncompatibleReferenceFramesError", PyExc_RuntimeError);
  py::register_exception<exceptions::IncompatibleSizeException>(m, "IncompatibleSizeError", PyExc_RuntimeError);
  py::register_exception<exceptions::IncompatibleStatesException>(m, "IncompatibleStatesError", PyExc_RuntimeError);
  py::register_exception<exceptions::InvalidCastException>(m, "InvalidCastError", PyExc_RuntimeError);
  py::register_exception<exceptions::InvalidParameterException>(m, "InvalidParameterError", PyExc_RuntimeError);
  py::register_exception<exceptions::JointNotFoundException>(m, "JointNotFoundError", PyExc_RuntimeError);
  py::register_exception<exceptions::NotImplementedException>(m, "NotImplementedError", PyExc_RuntimeError);
}
