#include "dynamical_systems_bindings.hpp"

#include <dynamical_systems/exceptions/EmptyAttractorException.hpp>
#include <dynamical_systems/exceptions/EmptyBaseFrameException.hpp>
#include <dynamical_systems/exceptions/IncompatibleSizeException.hpp>
#include <dynamical_systems/exceptions/InvalidDynamicalSystemException.hpp>

void bind_exceptions(py::module_& m) {
  py::register_exception<dynamical_systems::exceptions::EmptyAttractorException>(m, "EmptyAttractorError", PyExc_RuntimeError);
  py::register_exception<dynamical_systems::exceptions::EmptyBaseFrameException>(m, "EmptyBaseFrameError", PyExc_RuntimeError);
  py::register_exception<dynamical_systems::exceptions::IncompatibleSizeException>(m, "IncompatibleSizeError", PyExc_RuntimeError);
  py::register_exception<dynamical_systems::exceptions::InvalidDynamicalSystemException>(m, "InvalidDynamicalSystemError", PyExc_RuntimeError);
}
