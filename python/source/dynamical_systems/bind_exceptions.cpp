#include "dynamical_systems_bindings.hpp"

#include <dynamical_systems/exceptions.hpp>

void bind_exceptions(py::module_& m) {
  py::object error = py::module_::import("state_representation.exceptions").attr("Error");
  py::register_exception<dynamical_systems::exceptions::EmptyAttractorException>(m, "EmptyAttractorError", error.ptr());
  py::register_exception<dynamical_systems::exceptions::EmptyBaseFrameException>(m, "EmptyBaseFrameError", error.ptr());
}
