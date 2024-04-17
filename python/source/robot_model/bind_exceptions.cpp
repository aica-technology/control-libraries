#include "robot_model_bindings.hpp"

#include <robot_model/exceptions/FrameNotFoundException.hpp>
#include <robot_model/exceptions/InvalidJointStateSizeException.hpp>
#include <robot_model/exceptions/InverseKinematicsNotConvergingException.hpp>

void bind_exceptions(py::module_& m) {
  py::register_exception<robot_model::exceptions::FrameNotFoundException>(m, "FrameNotFoundError", PyExc_RuntimeError);
  py::register_exception<robot_model::exceptions::InvalidJointStateSizeException>(m, "InvalidJointStateSizeError", PyExc_RuntimeError);
  py::register_exception<robot_model::exceptions::InverseKinematicsNotConvergingException>(m, "InverseKinematicsNotConvergingErrors", PyExc_RuntimeError);
}
