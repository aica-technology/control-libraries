#include "controllers_bindings.hpp"

#include <controllers/exceptions/InvalidControllerException.hpp>
#include <controllers/exceptions/NoRobotModelException.hpp>

void bind_exceptions(py::module_& m) {
  py::register_exception<controllers::exceptions::InvalidControllerException>(m, "InvalidControllerError", PyExc_RuntimeError);
  py::register_exception<controllers::exceptions::NoRobotModelException>(m, "NoRobotModelError", PyExc_RuntimeError);
}
