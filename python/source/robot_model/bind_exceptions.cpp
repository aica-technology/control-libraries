#include "robot_model_bindings.hpp"

#include <robot_model/exceptions.hpp>

void bind_exceptions(py::module_& m) {
  py::object error = py::module_::import("state_representation.exceptions").attr("Error");
  py::register_exception<robot_model::exceptions::FrameNotFoundException>(m, "FrameNotFoundError", error.ptr());
  py::register_exception<robot_model::exceptions::InvalidJointStateSizeException>(m, "InvalidJointStateSizeError", error.ptr());
  py::register_exception<robot_model::exceptions::InverseKinematicsNotConvergingException>(m, "InverseKinematicsNotConvergingErrors", error.ptr());
  py::register_exception<robot_model::exceptions::CollisionGeometryException>(m, "CollisionGeometryError", error.ptr());
}
