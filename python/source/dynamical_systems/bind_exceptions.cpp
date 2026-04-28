#include "dynamical_systems_bindings.hpp"

#include <dynamical_systems/exceptions.hpp>

void bind_exceptions(py::module_& m) {
  py::register_exception<dynamical_systems::exceptions::EmptyAttractorException>(m, "EmptyAttractorError", PyExc_RuntimeError);
  py::register_exception<dynamical_systems::exceptions::EmptyBaseFrameException>(m, "EmptyBaseFrameError", PyExc_RuntimeError);
}
