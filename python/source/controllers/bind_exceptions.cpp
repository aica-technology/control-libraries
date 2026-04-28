#include "controllers_bindings.hpp"

#include <controllers/exceptions.hpp>

void bind_exceptions(py::module_& m) {
  py::object error = py::module_::import("state_representation.exceptions").attr("Error");
  py::register_exception<controllers::exceptions::InvalidControllerException>(m, "InvalidControllerError", error.ptr());
  py::register_exception<controllers::exceptions::NoRobotModelException>(m, "NoRobotModelError", error.ptr());
}
