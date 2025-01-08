#include "state_representation_bindings.hpp"

#include <state_representation/State.hpp>
#include <state_representation/space/SpatialState.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianTwist.hpp>
#include <state_representation/space/cartesian/CartesianAcceleration.hpp>
#include <state_representation/space/cartesian/CartesianWrench.hpp>


void spatial_state(py::module_& m) {
  py::class_<SpatialState, std::shared_ptr<SpatialState>, State> c(m, "SpatialState");

  c.def(py::init(), "Empty constructor.");
  c.def(py::init<const std::string&, const std::string&>(), "Constructor with name and reference frame specification.", "name"_a, "reference_frame"_a=std::string("world"));
  c.def(py::init<const SpatialState&>(), "Copy constructor from another SpatialState.", "state"_a);

  c.def("get_reference_frame", &SpatialState::get_reference_frame, "Getter of the reference frame.");
  c.def("set_reference_frame", &SpatialState::set_reference_frame, "Setter of the reference frame.", "reference_frame"_a);

  c.def("__copy__", [](const SpatialState &state) {
    return SpatialState(state);
  });
  c.def("__deepcopy__", [](const SpatialState &state, py::dict) {
    return SpatialState(state);
  }, "memo"_a);
  c.def("__repr__", [](const SpatialState& state) {
    std::stringstream buffer;
    buffer << state;
    return buffer.str();
  });
}

void cartesian_state_variable(py::module_& m) {
  py::enum_<CartesianStateVariable>(m, "CartesianStateVariable")
      .value("POSITION", CartesianStateVariable::POSITION)
      .value("ORIENTATION", CartesianStateVariable::ORIENTATION)
      .value("POSE", CartesianStateVariable::POSE)
      .value("LINEAR_VELOCITY", CartesianStateVariable::LINEAR_VELOCITY)
      .value("ANGULAR_VELOCITY", CartesianStateVariable::ANGULAR_VELOCITY)
      .value("TWIST", CartesianStateVariable::TWIST)
      .value("LINEAR_ACCELERATION", CartesianStateVariable::LINEAR_ACCELERATION)
      .value("ANGULAR_ACCELERATION", CartesianStateVariable::ANGULAR_ACCELERATION)
      .value("ACCELERATION", CartesianStateVariable::ACCELERATION)
      .value("FORCE", CartesianStateVariable::FORCE)
      .value("TORQUE", CartesianStateVariable::TORQUE)
      .value("WRENCH", CartesianStateVariable::WRENCH)
      .value("ALL", CartesianStateVariable::ALL)
      .export_values();

  m.def("string_to_cartesian_state_variable", &state_representation::string_to_cartesian_state_variable, "Convert a string to a CartesianStateVariable enum (case insensitive)", "variable"_a);
  m.def("cartesian_state_variable_to_string", &state_representation::cartesian_state_variable_to_string, "Convert CartesianStateVariable to a string", "variable"_a);
}

void cartesian_state(py::module_& m) {
  m.def("dist", py::overload_cast<const CartesianState&, const CartesianState&, const CartesianStateVariable&>(&state_representation::dist), "Compute the distance between two CartesianStates", "s1"_a, "s2"_a, "state_variable_type"_a=CartesianStateVariable::ALL);

  py::class_<CartesianState, std::shared_ptr<CartesianState>, SpatialState> c(m, "CartesianState");
  c.def(py::init(), "Empty constructor");
  c.def(py::init<const std::string&, const std::string&>(), "Constructor with name and reference frame provided", "name"_a, "reference_frame"_a=std::string("world"));
  c.def(py::init<const CartesianState&>(), "Copy constructor of a CartesianState", "state"_a);

  c.def_static("Identity", &CartesianState::Identity, "Constructor for the identity CartesianState (identity pose and 0 for the rest)", "name"_a, "reference_frame"_a=std::string("world"));
  c.def_static("Random", &CartesianState::Random, "Constructor for a random state", "name"_a, "reference_frame"_a=std::string("world"));

  c.def("get_position", &CartesianState::get_position, "Getter of the position attribute");
  c.def("get_orientation", [](const CartesianState &state) -> py::object {
    py::object PyQuaternion = py::module_::import("pyquaternion").attr("Quaternion");
    return PyQuaternion(state.get_orientation_coefficients());;
  }, "Getter of the orientation attribute as pyquaternion object");
  c.def("get_orientation_coefficients", &CartesianState::get_orientation_coefficients, "Getter of the orientation attribute as quaternion coefficients (w, x, y, z)");
  c.def("get_pose", &CartesianState::get_pose, "Getter of a 7d pose vector from position and orientation coefficients");
  c.def("get_transformation_matrix", &CartesianState::get_transformation_matrix, "Getter of a 4x4 transformation matrix of the pose");

  c.def("get_linear_velocity", &CartesianState::get_linear_velocity, "Getter of the linear velocity attribute");
  c.def("get_angular_velocity", &CartesianState::get_angular_velocity, "Getter of the angular velocity attribute");
  c.def("get_twist", &CartesianState::get_twist, "Getter of the 6d twist from linear and angular velocity attributes");

  c.def("get_linear_acceleration", &CartesianState::get_linear_acceleration, "Getter of the linear acceleration attribute");
  c.def("get_angular_acceleration", &CartesianState::get_angular_acceleration, "Getter of the angular acceleration attribute");
  c.def("get_acceleration", &CartesianState::get_acceleration, "Getter of the 6d acceleration from linear and angular acceleration attributes");

  c.def("get_force", &CartesianState::get_force, "Getter of the force attribute");
  c.def("get_torque", &CartesianState::get_torque, "Getter of the torque attribute");
  c.def("get_wrench", &CartesianState::get_wrench, "Getter of the 6d wrench from force and torque attributes");

  c.def("set_position", py::overload_cast<const Eigen::Vector3d&>(&CartesianState::set_position), "Setter of the position");
  c.def("set_position", py::overload_cast<const std::vector<double>&>(&CartesianState::set_position), "Setter of the position from a list");
  c.def("set_position", py::overload_cast<const double&, const double&, const double&>(&CartesianState::set_position), "Setter of the position from three scalar coordinates", "x"_a, "y"_a, "z"_a);
  c.def("set_orientation", [](CartesianState &state, const py::object& quaternion) {
    py::object PyQuaternion = py::module_::import("pyquaternion").attr("Quaternion");
    if (py::isinstance<py::list>(quaternion)) {
      state.set_orientation(py::cast<std::vector<double>>(quaternion));
    } else if (py::isinstance<py::array>(quaternion)) {
      state.set_orientation(py::cast<Eigen::Vector4d>(quaternion));
    } else if (py::isinstance(quaternion, PyQuaternion)) {
      state.set_orientation(py::cast<Eigen::Vector4d>(quaternion.attr("elements")));
    } else {
      throw std::invalid_argument("Type of input argument quaternion is not supported. "
                                  "Supported types are: pyquaternion.Quaternion, numpy.array, list");
    }
  }, "Setter of the orientation attribute from a pyquaternion.Quaternion, numpy.array(w, x, y, z), or list(w, x, y, z)");
  c.def("set_pose", py::overload_cast<const Eigen::Matrix<double, 7, 1>&>(&CartesianState::set_pose), "Setter of the pose from a 7d vector of position and orientation coefficients (x, y, z, qw, qx, qy, qz)");
  c.def("set_pose", py::overload_cast<const std::vector<double>&>(&CartesianState::set_pose), "Setter of the pose from a 7d list of position and orientation coefficients (x, y, z, qw, qx, qy, qz)");

  c.def("set_linear_velocity", py::overload_cast<const Eigen::Vector3d&>(&CartesianState::set_linear_velocity), "Setter of the linear velocity attribute");
  c.def("set_linear_velocity", py::overload_cast<const std::vector<double>&>(&CartesianState::set_linear_velocity), "Setter of the linear velocity from a list");
  c.def("set_linear_velocity", py::overload_cast<const double&, const double&, const double&>(&CartesianState::set_linear_velocity), "Setter of the linear velocity from three scalar coordinates", "x"_a, "y"_a, "z"_a);
  c.def("set_angular_velocity", py::overload_cast<const Eigen::Vector3d&>(&CartesianState::set_angular_velocity), "Setter of the angular velocity attribute");
  c.def("set_angular_velocity", py::overload_cast<const std::vector<double>&>(&CartesianState::set_angular_velocity), "Setter of the angular velocity from a list");
  c.def("set_angular_velocity", py::overload_cast<const double&, const double&, const double&>(&CartesianState::set_angular_velocity), "Setter of the angular velocity from three scalar coordinates", "x"_a, "y"_a, "z"_a);
  c.def("set_twist", py::overload_cast<const Eigen::Matrix<double, 6, 1>&>(&CartesianState::set_twist), "Setter of the linear and angular velocities from a 6d twist vector");
  c.def("set_twist", py::overload_cast<const std::vector<double>&>(&CartesianState::set_twist), "Setter of the linear and angular velocities from a list");

  c.def("set_linear_acceleration", py::overload_cast<const Eigen::Vector3d&>(&CartesianState::set_linear_acceleration), "Setter of the linear acceleration attribute");
  c.def("set_linear_acceleration", py::overload_cast<const std::vector<double>&>(&CartesianState::set_linear_acceleration), "Setter of the linear acceleration from a list");
  c.def("set_linear_acceleration", py::overload_cast<const double&, const double&, const double&>(&CartesianState::set_linear_acceleration), "Setter of the linear acceleration from three scalar coordinates", "x"_a, "y"_a, "z"_a);
  c.def("set_angular_acceleration", py::overload_cast<const Eigen::Vector3d&>(&CartesianState::set_angular_acceleration), "Setter of the angular acceleration attribute");
  c.def("set_angular_acceleration", py::overload_cast<const std::vector<double>&>(&CartesianState::set_angular_acceleration), "Setter of the angular acceleration from a list");
  c.def("set_angular_acceleration", py::overload_cast<const double&, const double&, const double&>(&CartesianState::set_angular_acceleration), "Setter of the angular acceleration from three scalar coordinates", "x"_a, "y"_a, "z"_a);
  c.def("set_acceleration", py::overload_cast<const Eigen::Matrix<double, 6, 1>&>(&CartesianState::set_acceleration), "Setter of the linear and angular accelerations from a 6d acceleration vector");
  c.def("set_acceleration", py::overload_cast<const std::vector<double>&>(&CartesianState::set_acceleration), "Setter of the linear and angular accelerations from a list");

  c.def("set_force", py::overload_cast<const Eigen::Vector3d&>(&CartesianState::set_force), "Setter of the force attribute");
  c.def("set_force", py::overload_cast<const std::vector<double>&>(&CartesianState::set_force), "Setter of the force from a list");
  c.def("set_force", py::overload_cast<const double&, const double&, const double&>(&CartesianState::set_force), "Setter of the force from three scalar coordinates", "x"_a, "y"_a, "z"_a);
  c.def("set_torque", py::overload_cast<const Eigen::Vector3d&>(&CartesianState::set_torque), "Setter of the torque attribute");
  c.def("set_torque", py::overload_cast<const std::vector<double>&>(&CartesianState::set_torque), "Setter of the torque from a list");
  c.def("set_torque", py::overload_cast<const double&, const double&, const double&>(&CartesianState::set_torque), "Setter of the torque from three scalar coordinates", "x"_a, "y"_a, "z"_a);
  c.def("set_wrench", py::overload_cast<const Eigen::Matrix<double, 6, 1>&>(&CartesianState::set_wrench), "Setter of the force and torque from a 6d wrench vector");
  c.def("set_wrench", py::overload_cast<const std::vector<double>&>(&CartesianState::set_wrench), "Setter of the force and torque velocities from a list");

  c.def("set_zero", &CartesianState::set_zero, "Set the CartesianState to a zero value");
  c.def("clamp_state_variable", &CartesianState::clamp_state_variable, "Clamp inplace the magnitude of the a specific state variable (velocity, acceleration or force)", "max_value"_a, "state_variable_type"_a, "noise_ratio"_a=double(0));
  c.def("copy", &CartesianState::copy, "Return a copy of the CartesianState");
  c.def("data", &CartesianState::data, "Returns the data as the concatenation of all the state variables in a single vector");
  c.def("array", &CartesianState::array, "Returns the data vector as an array");
  c.def("set_data", py::overload_cast<const Eigen::VectorXd&>(&CartesianState::set_data), "Set the data of the state from all the state variables in a single vector", "data"_a);
  c.def("set_data", py::overload_cast<const std::vector<double>&>(&CartesianState::set_data), "Set the data of the state from all the state variables in a single list", "data"_a);

  c.def(py::self *= py::self);
  c.def(py::self * py::self);
  c.def(py::self *= double());
  c.def(py::self * double());
  c.def(double() * py::self);
  c.def(py::self * Eigen::Vector3d());
  c.def("__mul__", [](const CartesianState& self, const Eigen::MatrixXd& other) -> void { throw py::type_error("unsupported operand type(s) for *: 'state_representation.CartesianState' and 'np.ndarray'"); });
  c.def("__imul__", [](const CartesianState& self, const Eigen::MatrixXd& other) -> void { throw py::type_error("unsupported operand type(s) for *=: 'state_representation.CartesianState' and 'np.ndarray'"); });
  c.def(py::self /= double());
  c.def(py::self / double());
  c.def("__truediv__", [](const CartesianState& self, const Eigen::MatrixXd& other) -> void { throw py::type_error("unsupported operand type(s) for /: 'state_representation.CartesianState' and 'np.ndarray'"); });

  c.def(py::self += py::self);
  c.def(py::self + py::self);
  c.def("__neg__", [](const CartesianState& self) -> CartesianState { return -self; });
  c.def(py::self -= py::self);
  c.def(py::self - py::self);

  c.def("inverse", &CartesianState::inverse, "Compute the inverse of the current CartesianState");
  c.def("dist", &CartesianState::dist, "Compute the distance to another state as the sum of distances between each features", "state"_a, "state_variable_type"_a=CartesianStateVariable::ALL);
  c.def("norms", &CartesianState::norms, "Compute the norms of the state variable specified by the input type (default is full state)", "state_variable_type"_a=CartesianStateVariable::ALL);
  c.def("normalize", &CartesianState::normalize, "Normalize inplace the state at the state variable given in argument (default is full state)", "state_variable_type"_a=CartesianStateVariable::ALL);
  c.def("normalized", &CartesianState::normalized, "Compute the normalized state at the state variable given in argument (default is full state)", "state_variable_type"_a=CartesianStateVariable::ALL);

  c.def("to_list", &CartesianState::to_std_vector, "Return the state as a list");

  c.def("__copy__", [](const CartesianState &state) {
    return CartesianState(state);
  });
  c.def("__deepcopy__", [](const CartesianState &state, py::dict) {
    return CartesianState(state);
  }, "memo"_a);
  c.def("__repr__", [](const CartesianState& state) {
    std::stringstream buffer;
    buffer << state;
    return buffer.str();
  });

  c.def("get_state_variable", &CartesianState::get_state_variable, "Getter of the variable value corresponding to the input", "state_variable_type"_a);
  c.def("set_state_variable", py::overload_cast<const Eigen::VectorXd&, const CartesianStateVariable&>(&CartesianState::set_state_variable), "Setter of the variable value corresponding to the input", "new_value"_a, "state_variable_type"_a);
  c.def("set_state_variable", py::overload_cast<const std::vector<double>&, const CartesianStateVariable&>(&CartesianState::set_state_variable), "Setter of the variable value corresponding to the input", "new_value"_a, "state_variable_type"_a);
}

void cartesian_pose(py::module_& m) {
  py::class_<CartesianPose, std::shared_ptr<CartesianPose>, CartesianState> c(m, "CartesianPose");

  c.def(py::init(), "Empty constructor");
  c.def(py::init<const std::string&, const std::string&>(), "Constructor with name and reference frame provided", "name"_a, "reference_frame"_a=std::string("world"));
  c.def(py::init<const CartesianPose&>(), "Copy constructor of a CartesianPose", "pose"_a);
  c.def(py::init<const CartesianState&>(), "Copy constructor from a CartesianState", "state"_a);
  c.def(py::init<const CartesianTwist&>(), "Copy constructor from a CartesianTwist by considering that it is a displacement over 1 second", "twist"_a);

  c.def(py::init<const std::string&, const Eigen::Vector3d&, const std::string&>(), "Constructor of a CartesianPose from a position given as a vector of coordinates", "name"_a, "position"_a, "reference_frame"_a=std::string("world"));
  c.def(py::init<const std::string&, double, double, double, const std::string&>(), "Constructor of a CartesianPose from a position given as three scalar coordinates", "name"_a, "x"_a, "y"_a, "z"_a, "reference_frame"_a=std::string("world"));
  c.def(py::init([](const std::string& name, const Eigen::Vector4d& orientation, const std::string& reference) {
    Eigen::Quaterniond q(orientation(0), orientation(1), orientation(2), orientation(3));
    return new CartesianPose(name, q, reference);
  }), "Constructor of a CartesianPose from a quaternion", "name"_a, "orientation"_a, "reference_frame"_a=std::string("world"));
  c.def(py::init([](const std::string& name, const Eigen::Vector3d& position, const Eigen::Vector4d& orientation, const std::string& reference) {
    Eigen::Quaterniond q(orientation(0), orientation(1), orientation(2), orientation(3));
    return new CartesianPose(name, position, q, reference);
  }), "Constructor of a CartesianPose from a position given as a vector of coordinates and a quaternion", "name"_a, "position"_a, "orientation"_a, "reference_frame"_a=std::string("world"));

  c.def_static("Identity", &CartesianPose::Identity, "Constructor for the identity pose", "name"_a, "reference_frame"_a=std::string("world"));
  c.def_static("Random", &CartesianPose::Random, "Constructor for a random pose", "name"_a, "reference_frame"_a=std::string("world"));

  std::vector<std::string> deleted_attributes = {
      "linear_velocity",
      "angular_velocity",
      "twist",
      "linear_acceleration",
      "angular_acceleration",
      "acceleration",
      "force",
      "torque",
      "wrench"
  };

  for (const std::string& attr : deleted_attributes) {
    c.def(std::string("get_" + attr).c_str(), [](const CartesianPose&) -> void {}, "Deleted method from parent class.");
    c.def(std::string("set_" + attr).c_str(), [](const CartesianPose& pose) -> CartesianPose { return pose; }, "Deleted method from parent class.");
  }

  c.def(py::self *= py::self);
  c.def("__imul__", [](const CartesianPose& self, const CartesianTwist& other) -> void { throw py::type_error("unsupported operand type(s) for *=: 'state_representation.CartesianPose' and 'state_representation.CartesianTwist'"); });
  c.def("__imul__", [](const CartesianPose& self, const CartesianAcceleration& other) -> void { throw py::type_error("unsupported operand type(s) for *=: 'state_representation.CartesianPose' and 'state_representation.CartesianAcceleration'"); });
  c.def("__imul__", [](const CartesianPose& self, const CartesianWrench& other) -> void { throw py::type_error("unsupported operand type(s) for *=: 'state_representation.CartesianPose' and 'state_representation.CartesianWrench'"); });
  c.def(py::self *= CartesianState());
  c.def(py::self * py::self);
  c.def(py::self * CartesianTwist());
  c.def(py::self * CartesianAcceleration());
  c.def(py::self * CartesianWrench());
  c.def(py::self * CartesianState());
  c.def(py::self *= double());
  c.def(py::self * double());
  c.def(double() * py::self);
  c.def("__mul__", [](const CartesianPose& self, const Eigen::MatrixXd& other) -> void { throw py::type_error("unsupported operand type(s) for *: 'state_representation.CartesianPose' and 'np.ndarray'"); });
  c.def("__imul__", [](const CartesianPose& self, const Eigen::MatrixXd& other) -> void { throw py::type_error("unsupported operand type(s) for *=: 'state_representation.CartesianPose' and 'np.ndarray'"); });
  c.def(py::self /= double());
  c.def(py::self / double());
  c.def("__truediv__", [](const CartesianPose& self, const Eigen::MatrixXd& other) -> void { throw py::type_error("unsupported operand type(s) for /: 'state_representation.CartesianPose' and 'np.ndarray'"); });

  c.def(py::self / std::chrono::nanoseconds());

  c.def(py::self += py::self);
  c.def("__iadd__", [](const CartesianPose& self, const CartesianTwist& other) -> void { throw py::type_error("unsupported operand type(s) for +=: 'state_representation.CartesianPose' and 'state_representation.CartesianTwist'"); });
  c.def("__iadd__", [](const CartesianPose& self, const CartesianAcceleration& other) -> void { throw py::type_error("unsupported operand type(s) for +=: 'state_representation.CartesianPose' and 'state_representation.CartesianAcceleration'"); });
  c.def("__iadd__", [](const CartesianPose& self, const CartesianWrench& other) -> void { throw py::type_error("unsupported operand type(s) for +=: 'state_representation.CartesianPose' and 'state_representation.CartesianWrench'"); });
  c.def(py::self += CartesianState());
  c.def(py::self + py::self);
  c.def("__add__", [](const CartesianPose& self, const CartesianTwist& other) -> void { throw py::type_error("unsupported operand type(s) for +: 'state_representation.CartesianPose' and 'state_representation.CartesianTwist'"); });
  c.def("__add__", [](const CartesianPose& self, const CartesianAcceleration& other) -> void { throw py::type_error("unsupported operand type(s) for +: 'state_representation.CartesianPose' and 'state_representation.CartesianAcceleration'"); });
  c.def("__add__", [](const CartesianPose& self, const CartesianWrench& other) -> void { throw py::type_error("unsupported operand type(s) for +: 'state_representation.CartesianPose' and 'state_representation.CartesianWrench'"); });
  c.def(py::self + CartesianState());
  c.def("__neg__", [](const CartesianPose& self) -> CartesianPose { return -self; });
  c.def(py::self -= py::self);
  c.def("__isub__", [](const CartesianPose& self, const CartesianTwist& other) -> void { throw py::type_error("unsupported operand type(s) for -=: 'state_representation.CartesianPose' and 'state_representation.CartesianTwist'"); });
  c.def("__isub__", [](const CartesianPose& self, const CartesianAcceleration& other) -> void { throw py::type_error("unsupported operand type(s) for -=: 'state_representation.CartesianPose' and 'state_representation.CartesianAcceleration'"); });
  c.def("__isub__", [](const CartesianPose& self, const CartesianWrench& other) -> void { throw py::type_error("unsupported operand type(s) for -=: 'state_representation.CartesianPose' and 'state_representation.CartesianWrench'"); });
  c.def(py::self -= CartesianState());
  c.def(py::self - py::self);
  c.def("__sub__", [](const CartesianPose& self, const CartesianTwist& other) -> void { throw py::type_error("unsupported operand type(s) for -: 'state_representation.CartesianPose' and 'state_representation.CartesianTwist'"); });
  c.def("__sub__", [](const CartesianPose& self, const CartesianAcceleration& other) -> void { throw py::type_error("unsupported operand type(s) for -: 'state_representation.CartesianPose' and 'state_representation.CartesianAcceleration'"); });
  c.def("__sub__", [](const CartesianPose& self, const CartesianWrench& other) -> void { throw py::type_error("unsupported operand type(s) for -: 'state_representation.CartesianPose' and 'state_representation.CartesianWrench'"); });

  c.def(py::self - CartesianState());

  c.def("copy", &CartesianPose::copy, "Return a copy of the CartesianPose");
  c.def("data", &CartesianPose::data, "Returns the pose data as a vector");
  c.def("differentiate", [](const CartesianPose &pose, double dt) -> CartesianTwist {
    return pose.differentiate(dt);
  }, "Differentiate a Cartesian pose over a time period in seconds", "dt"_a);
  c.def("differentiate", [](const CartesianPose &pose, const std::chrono::nanoseconds& dt) -> CartesianTwist {
    return pose.differentiate(dt);
  }, "Differentiate a Cartesian pose over a time period", "dt"_a);
  c.def("inverse", &CartesianPose::inverse, "Compute the inverse of the current CartesianPose");
  c.def("set_data", py::overload_cast<const Eigen::VectorXd&>(&CartesianPose::set_data), "Set the pose data from a vector", "data"_a);
  c.def("set_data", py::overload_cast<const std::vector<double>&>(&CartesianPose::set_data), "Set the pose data from a list", "data"_a);
  c.def("norms", &CartesianPose::norms, "Compute the norms of the state variable specified by the input type (default is full pose)", "state_variable_type"_a=CartesianStateVariable::POSE);
  c.def("normalized", &CartesianPose::normalized, "Compute the normalized pose at the state variable given in argument (default is full pose)", "state_variable_type"_a=CartesianStateVariable::POSE);

  c.def("__copy__", [](const CartesianPose &pose) {
    return CartesianPose(pose);
  });
  c.def("__deepcopy__", [](const CartesianPose &pose, py::dict) {
    return CartesianPose(pose);
  }, "memo"_a);
  c.def("__repr__", [](const CartesianPose& pose) {
    std::stringstream buffer;
    buffer << pose;
    return buffer.str();
  });
}

void cartesian_twist(py::module_& m) {
  py::class_<CartesianTwist, std::shared_ptr<CartesianTwist>, CartesianState> c(m, "CartesianTwist");

  c.def(py::init(), "Empty constructor");
  c.def(py::init<const std::string&, const std::string&>(), "Constructor with name and reference frame provided", "name"_a, "reference_frame"_a=std::string("world"));
  c.def(py::init<const CartesianTwist&>(), "Copy constructor", "twist"_a);
  c.def(py::init<const CartesianState&>(), "Copy constructor from a CartesianState", "state"_a);
  c.def(py::init<const CartesianPose&>(), "Copy constructor from a CartesianPose by considering that it is equivalent to dividing the pose by 1 second", "pose"_a);

  c.def(py::init<const std::string&, const Eigen::Vector3d&, const std::string&>(), "Construct a CartesianTwist from a linear velocity given as a vector.", "name"_a, "linear_velocity"_a, "reference_frame"_a=std::string("world"));
  c.def(py::init<const std::string&, const Eigen::Vector3d&, const Eigen::Vector3d&, const std::string&>(), "Construct a CartesianTwist from a linear velocity and angular velocity given as vectors.", "name"_a, "linear_velocity"_a, "angular_velocity"_a, "reference_frame"_a=std::string("world"));
  c.def(py::init<const std::string&, const Eigen::Matrix<double, 6, 1>&, const std::string&>(), "Construct a CartesianTwist from a single 6d twist vector", "name"_a, "twist"_a, "reference_frame"_a=std::string("world"));

  c.def_static("Zero", &CartesianTwist::Zero, "Constructor for the zero twist", "name"_a, "reference_frame"_a=std::string("world"));
  c.def_static("Random", &CartesianTwist::Random, "Constructor for a random twist", "name"_a, "reference_frame"_a=std::string("world"));

  std::vector<std::string> deleted_attributes = {
      "position",
      "orientation",
      "pose",
      "linear_acceleration",
      "angular_acceleration",
      "acceleration",
      "force",
      "torque",
      "wrench"
  };

  for (const std::string& attr : deleted_attributes) {
    c.def(std::string("get_" + attr).c_str(), [](const CartesianTwist&) -> void {}, "Deleted method from parent class.");
    c.def(std::string("set_" + attr).c_str(), [](const CartesianTwist& twist) -> CartesianTwist { return twist; }, "Deleted method from parent class.");
  }
  c.def(std::string("get_orientation_coefficients").c_str(), [](const CartesianTwist&) -> void {}, "Deleted method from parent class.");

  c.def(py::self *= double());
  c.def(py::self * double());
  c.def(double() * py::self);
  c.def(Eigen::Matrix<double, 6, 6>() * py::self);
  c.def("__mul__", [](const CartesianTwist& self, const Eigen::MatrixXd& other) -> void { throw py::type_error("unsupported operand type(s) for *: 'state_representation.CartesianTwist' and 'numpy.ndarray'"); });
  c.def(py::self * std::chrono::nanoseconds());
  c.def(std::chrono::nanoseconds() * py::self);
  c.def(py::self /= double());
  c.def(py::self / double());
  c.def("__truediv__", [](const CartesianTwist& self, const Eigen::MatrixXd& other) -> void { throw py::type_error("unsupported operand type(s) for /: 'state_representation.CartesianTwist' and 'np.ndarray'"); });
  c.def(py::self / std::chrono::nanoseconds());

  c.def(py::self += py::self);
  c.def("__iadd__", [](const CartesianTwist& self, const CartesianPose& other) -> void { throw py::type_error("unsupported operand type(s) for +=: 'state_representation.CartesianTwist' and 'state_representation.CartesianPose'"); });
  c.def("__iadd__", [](const CartesianTwist& self, const CartesianAcceleration& other) -> void { throw py::type_error("unsupported operand type(s) for +=: 'state_representation.CartesianTwist' and 'state_representation.CartesianAcceleration'"); });
  c.def("__iadd__", [](const CartesianTwist& self, const CartesianWrench& other) -> void { throw py::type_error("unsupported operand type(s) for +=: 'state_representation.CartesianTwist' and 'state_representation.CartesianWrench'"); });
  c.def(py::self += CartesianState());
  c.def(py::self + py::self);
  c.def("__add__", [](const CartesianTwist& self, const CartesianPose& other) -> void { throw py::type_error("unsupported operand type(s) for +: 'state_representation.CartesianTwist' and 'state_representation.CartesianPose'"); });
  c.def("__add__", [](const CartesianTwist& self, const CartesianAcceleration& other) -> void { throw py::type_error("unsupported operand type(s) for +: 'state_representation.CartesianTwist' and 'state_representation.CartesianAcceleration'"); });
  c.def("__add__", [](const CartesianTwist& self, const CartesianWrench& other) -> void { throw py::type_error("unsupported operand type(s) for +: 'state_representation.CartesianTwist' and 'state_representation.CartesianWrench'"); });
  c.def(py::self + CartesianState());
  c.def("__neg__", [](const CartesianTwist& self) -> CartesianTwist { return -self; });
  c.def(py::self -= py::self);
  c.def("__isub__", [](const CartesianTwist& self, const CartesianPose& other) -> void { throw py::type_error("unsupported operand type(s) for -=: 'state_representation.CartesianTwist' and 'state_representation.CartesianPose'"); });
  c.def("__isub__", [](const CartesianTwist& self, const CartesianAcceleration& other) -> void { throw py::type_error("unsupported operand type(s) for -=: 'state_representation.CartesianTwist' and 'state_representation.CartesianAcceleration'"); });
  c.def("__isub__", [](const CartesianTwist& self, const CartesianWrench& other) -> void { throw py::type_error("unsupported operand type(s) for -=: 'state_representation.CartesianTwist' and 'state_representation.CartesianWrench'"); });
  c.def(py::self -= CartesianState());
  c.def(py::self - py::self);
  c.def("__sub__", [](const CartesianTwist& self, const CartesianPose& other) -> void { throw py::type_error("unsupported operand type(s) for -: 'state_representation.CartesianTwist' and 'state_representation.CartesianPose'"); });
  c.def("__sub__", [](const CartesianTwist& self, const CartesianAcceleration& other) -> void { throw py::type_error("unsupported operand type(s) for -: 'state_representation.CartesianTwist' and 'state_representation.CartesianAcceleration'"); });
  c.def("__sub__", [](const CartesianTwist& self, const CartesianWrench& other) -> void { throw py::type_error("unsupported operand type(s) for -: 'state_representation.CartesianTwist' and 'state_representation.CartesianWrench'"); });
  c.def(py::self - CartesianState());

  c.def("clamp", &CartesianTwist::clamp, "Clamp inplace the magnitude of the twist to the values in argument", "max_linear"_a, "max_angular"_a, "linear_noise_ratio"_a=0, "angular_noise_ratio"_a=0);
  c.def("clamped", &CartesianTwist::clamped, "Return the clamped twist", "max_linear"_a, "max_angular"_a, "linear_noise_ratio"_a=0, "angular_noise_ratio"_a=0);

  c.def("copy", &CartesianTwist::copy, "Return a copy of the CartesianTwist");
  c.def("data", &CartesianTwist::data, "Returns the twist data as a vector");
  c.def("differentiate", [](const CartesianTwist &twist, double dt) -> CartesianAcceleration {
    return twist.differentiate(dt);
  }, "Differentiate a Cartesian twist over a time period in seconds", "dt"_a);
  c.def("differentiate", [](const CartesianTwist &twist, const std::chrono::nanoseconds& dt) -> CartesianAcceleration {
    return twist.differentiate(dt);
  }, "Differentiate a Cartesian twist over a time period", "dt"_a);
  c.def("integrate", [](const CartesianTwist &twist, double dt) -> CartesianPose {
    return twist.integrate(dt);
  }, "Integrate a Cartesian twist over a time period in seconds", "dt"_a);
  c.def("integrate", [](const CartesianTwist &twist, const std::chrono::nanoseconds& dt) -> CartesianPose {
    return twist.integrate(dt);
  }, "Integrate a Cartesian twist over a time period", "dt"_a);
  c.def("inverse", &CartesianTwist::inverse, "Compute the inverse of the current CartesianTwist");
  c.def("set_data", py::overload_cast<const Eigen::VectorXd&>(&CartesianTwist::set_data), "Set the twist data from a vector", "data"_a);
  c.def("set_data", py::overload_cast<const std::vector<double>&>(&CartesianTwist::set_data), "Set the twist data from a list", "data"_a);
  c.def("norms", &CartesianTwist::norms, "Compute the norms of the state variable specified by the input type (default is full twist)", "state_variable_type"_a=CartesianStateVariable::TWIST);
  c.def("normalized", &CartesianTwist::normalized, "Compute the normalized twist at the state variable given in argument (default is full twist)", "state_variable_type"_a=CartesianStateVariable::TWIST);

  c.def("__copy__", [](const CartesianTwist &twist) {
    return CartesianTwist(twist);
  });
  c.def("__deepcopy__", [](const CartesianTwist &twist, py::dict) {
    return CartesianTwist(twist);
  }, "memo"_a);
  c.def("__repr__", [](const CartesianTwist& twist) {
    std::stringstream buffer;
    buffer << twist;
    return buffer.str();
  });
}

void cartesian_acceleration(py::module_& m) {
  py::class_<CartesianAcceleration, std::shared_ptr<CartesianAcceleration>, CartesianState> c(m, "CartesianAcceleration");

  c.def(py::init(), "Empty constructor");
  c.def(py::init<const std::string&, const std::string&>(), "Constructor with name and reference frame provided", "name"_a, "reference_frame"_a=std::string("world"));
  c.def(py::init<const CartesianAcceleration&>(), "Copy constructor", "acceleration"_a);
  c.def(py::init<const CartesianState&>(), "Copy constructor from a CartesianState", "state"_a);
  c.def(py::init<const CartesianTwist&>(), "Copy constructor from a CartesianTwist by considering that it is equivalent to dividing the twist by 1 second", "twist"_a);

  c.def(py::init<const std::string&, const Eigen::Vector3d&, const std::string&>(), "Construct a CartesianAcceleration from a linear acceleration given as a vector.", "name"_a, "linear_acceleration"_a, "reference_frame"_a=std::string("world"));
  c.def(py::init<const std::string&, const Eigen::Vector3d&, const Eigen::Vector3d&, const std::string&>(), "Construct a CartesianAcceleration from a linear acceleration and angular acceleration given as vectors.", "name"_a, "linear_acceleration"_a, "angular_acceleration"_a, "reference_frame"_a=std::string("world"));
  c.def(py::init<const std::string&, const Eigen::Matrix<double, 6, 1>&, const std::string&>(), "Construct a CartesianAcceleration from a single 6d acceleration vector", "name"_a, "acceleration"_a, "reference_frame"_a=std::string("world"));

  c.def_static("Zero", &CartesianAcceleration::Zero, "Constructor for the zero acceleration", "name"_a, "reference_frame"_a=std::string("world"));
  c.def_static("Random", &CartesianAcceleration::Random, "Constructor for a random acceleration", "name"_a, "reference_frame"_a=std::string("world"));

  std::vector<std::string> deleted_attributes = {
      "position",
      "orientation",
      "pose",
      "linear_velocity",
      "angular_velocity",
      "twist",
      "force",
      "torque",
      "wrench"
  };

  for (const std::string& attr : deleted_attributes) {
    c.def(std::string("get_" + attr).c_str(), [](const CartesianAcceleration&) -> void {}, "Deleted method from parent class.");
    c.def(std::string("set_" + attr).c_str(), [](const CartesianAcceleration& acceleration) -> CartesianAcceleration { return acceleration; }, "Deleted method from parent class.");
  }
  c.def(std::string("get_orientation_coefficients").c_str(), [](const CartesianAcceleration&) -> void {}, "Deleted method from parent class.");

  c.def(py::self *= double());
  c.def(py::self * double());
  c.def(double() * py::self);
  c.def(Eigen::Matrix<double, 6, 6>() * py::self);
  c.def("__mul__", [](const CartesianAcceleration& self, const Eigen::MatrixXd& other) -> void { throw py::type_error("unsupported operand type(s) for *: 'state_representation.CartesianAcceleration' and 'numpy.ndarray'"); });
  c.def(py::self * std::chrono::nanoseconds());
  c.def(std::chrono::nanoseconds() * py::self);
  c.def(py::self /= double());
  c.def(py::self / double());
  c.def("__truediv__", [](const CartesianAcceleration& self, const Eigen::MatrixXd& other) -> void { throw py::type_error("unsupported operand type(s) for /: 'state_representation.CartesianAcceleration' and 'np.ndarray'"); });

  c.def(py::self += py::self);
  c.def("__iadd__", [](const CartesianAcceleration& self, const CartesianPose& other) -> void { throw py::type_error("unsupported operand type(s) for +=: 'state_representation.CartesianAcceleration' and 'state_representation.CartesianPose'"); });
  c.def("__iadd__", [](const CartesianAcceleration& self, const CartesianTwist& other) -> void { throw py::type_error("unsupported operand type(s) for +=: 'state_representation.CartesianAcceleration' and 'state_representation.CartesianTwist'"); });
  c.def("__iadd__", [](const CartesianAcceleration& self, const CartesianWrench& other) -> void { throw py::type_error("unsupported operand type(s) for +=: 'state_representation.CartesianAcceleration' and 'state_representation.CartesianWrench'"); });
  c.def(py::self += CartesianState());
  c.def(py::self + py::self);
  c.def("__add__", [](const CartesianAcceleration& self, const CartesianPose& other) -> void { throw py::type_error("unsupported operand type(s) for +: 'state_representation.CartesianAcceleration' and 'state_representation.CartesianPose'"); });
  c.def("__add__", [](const CartesianAcceleration& self, const CartesianTwist& other) -> void { throw py::type_error("unsupported operand type(s) for +: 'state_representation.CartesianAcceleration' and 'state_representation.CartesianTwist'"); });
  c.def("__add__", [](const CartesianAcceleration& self, const CartesianWrench& other) -> void { throw py::type_error("unsupported operand type(s) for +: 'state_representation.CartesianAcceleration' and 'state_representation.CartesianWrench'"); });
  c.def(py::self + CartesianState());
  c.def("__neg__", [](const CartesianAcceleration& self) -> CartesianAcceleration { return -self; });
  c.def(py::self -= py::self);
  c.def("__isub__", [](const CartesianAcceleration& self, const CartesianPose& other) -> void { throw py::type_error("unsupported operand type(s) for -=: 'state_representation.CartesianAcceleration' and 'state_representation.CartesianPose'"); });
  c.def("__isub__", [](const CartesianAcceleration& self, const CartesianTwist& other) -> void { throw py::type_error("unsupported operand type(s) for -=: 'state_representation.CartesianAcceleration' and 'state_representation.CartesianTwist'"); });
  c.def("__isub__", [](const CartesianAcceleration& self, const CartesianWrench& other) -> void { throw py::type_error("unsupported operand type(s) for -=: 'state_representation.CartesianAcceleration' and 'state_representation.CartesianWrench'"); });
  c.def(py::self -= CartesianState());
  c.def(py::self - py::self);
  c.def("__sub__", [](const CartesianAcceleration& self, const CartesianPose& other) -> void { throw py::type_error("unsupported operand type(s) for -: 'state_representation.CartesianAcceleration' and 'state_representation.CartesianPose'"); });
  c.def("__sub__", [](const CartesianAcceleration& self, const CartesianTwist& other) -> void { throw py::type_error("unsupported operand type(s) for -: 'state_representation.CartesianAcceleration' and 'state_representation.CartesianTwist'"); });
  c.def("__sub__", [](const CartesianAcceleration& self, const CartesianWrench& other) -> void { throw py::type_error("unsupported operand type(s) for -: 'state_representation.CartesianAcceleration' and 'state_representation.CartesianWrench'"); });
  c.def(py::self - CartesianState());

  c.def("clamp", &CartesianAcceleration::clamp, "Clamp inplace the magnitude of the acceleration to the values in argument", "max_linear"_a, "max_angular"_a, "linear_noise_ratio"_a=0, "angular_noise_ratio"_a=0);
  c.def("clamped", &CartesianAcceleration::clamped, "Return the clamped acceleration", "max_linear"_a, "max_angular"_a, "linear_noise_ratio"_a=0, "angular_noise_ratio"_a=0);

  c.def("copy", &CartesianAcceleration::copy, "Return a copy of the CartesianAcceleration");
  c.def("data", &CartesianAcceleration::data, "Returns the acceleration data as a vector");
  c.def("integrate", [](const CartesianAcceleration &acceleration, double dt) -> CartesianTwist {
    return acceleration.integrate(dt);
  }, "Integrate a Cartesian acceleration over a time period in seconds", "dt"_a);
  c.def("integrate", [](const CartesianAcceleration &acceleration, const std::chrono::nanoseconds& dt) -> CartesianTwist {
    return acceleration.integrate(dt);
  }, "Integrate a Cartesian acceleration over a time period", "dt"_a);
  c.def("inverse", &CartesianAcceleration::inverse, "Compute the inverse of the current CartesianPose");
  c.def("set_data", py::overload_cast<const Eigen::VectorXd&>(&CartesianAcceleration::set_data), "Set the acceleration data from a vector", "data"_a);
  c.def("set_data", py::overload_cast<const std::vector<double>&>(&CartesianAcceleration::set_data), "Set the acceleration data from a list", "data"_a);
  c.def("norms", &CartesianAcceleration::norms, "Compute the norms of the state variable specified by the input type (default is full acceleration)", "state_variable_type"_a=CartesianStateVariable::ACCELERATION);
  c.def("normalized", &CartesianAcceleration::normalized, "Compute the normalized acceleration at the state variable given in argument (default is full acceleration)", "state_variable_type"_a=CartesianStateVariable::ACCELERATION);

  c.def("__copy__", [](const CartesianAcceleration &acceleration) {
    return CartesianAcceleration(acceleration);
  });
  c.def("__deepcopy__", [](const CartesianAcceleration &acceleration, py::dict) {
    return CartesianAcceleration(acceleration);
  }, "memo"_a);
  c.def("__repr__", [](const CartesianAcceleration& acceleration) {
    std::stringstream buffer;
    buffer << acceleration;
    return buffer.str();
  });
}

void cartesian_wrench(py::module_& m) {
  py::class_<CartesianWrench, std::shared_ptr<CartesianWrench>, CartesianState> c(m, "CartesianWrench");

  c.def(py::init(), "Empty constructor");
  c.def(py::init<const std::string&, const std::string&>(), "Constructor with name and reference frame provided", "name"_a, "reference_frame"_a=std::string("world"));
  c.def(py::init<const CartesianWrench&>(), "Copy constructor", "twist"_a);
  c.def(py::init<const CartesianState&>(), "Copy constructor from a CartesianState", "state"_a);

  c.def(py::init<const std::string&, const Eigen::Vector3d&, const std::string&>(), "Construct a CartesianWrench from a force given as a vector.", "name"_a, "force"_a, "reference_frame"_a=std::string("world"));
  c.def(py::init<const std::string&, const Eigen::Vector3d&, const Eigen::Vector3d&, const std::string&>(), "Construct a CartesianWrench from a force and torque given as vectors.", "name"_a, "force"_a, "torque"_a, "reference_frame"_a=std::string("world"));
  c.def(py::init<const std::string&, const Eigen::Matrix<double, 6, 1>&, const std::string&>(), "Construct a CartesianTwist from a single 6d wrench vector", "name"_a, "wrench"_a, "reference_frame"_a=std::string("world"));

  c.def_static("Zero", &CartesianWrench::Zero, "Constructor for the zero wrench", "name"_a, "reference_frame"_a=std::string("world"));
  c.def_static("Random", &CartesianWrench::Random, "Constructor for a random wrench", "name"_a, "reference_frame"_a=std::string("world"));

  std::vector<std::string> deleted_attributes = {
      "position",
      "orientation",
      "pose",
      "linear_velocity",
      "angular_velocity",
      "twist",
      "linear_acceleration",
      "angular_acceleration",
      "acceleration",
  };

  for (const std::string& attr : deleted_attributes) {
    c.def(std::string("get_" + attr).c_str(), [](const CartesianWrench&) -> void {}, "Deleted method from parent class.");
    c.def(std::string("set_" + attr).c_str(), [](const CartesianWrench& wrench) -> CartesianWrench { return wrench; }, "Deleted method from parent class.");
  }
  c.def(std::string("get_orientation_coefficients").c_str(), [](const CartesianWrench&) -> void {}, "Deleted method from parent class.");

  c.def(py::self *= double());
  c.def(py::self * double());
  c.def(double() * py::self);
  c.def(Eigen::Matrix<double, 6, 6>() * py::self);
  c.def("__mul__", [](const CartesianWrench& self, const Eigen::MatrixXd& other) -> void { throw py::type_error("unsupported operand type(s) for *: 'state_representation.CartesianWrench' and 'numpy.ndarray'"); });
  c.def(py::self /= double());
  c.def(py::self / double());
  c.def("__truediv__", [](const CartesianWrench& self, const Eigen::MatrixXd& other) -> void { throw py::type_error("unsupported operand type(s) for /: 'state_representation.CartesianWrench' and 'np.ndarray'"); });

  c.def(py::self += py::self);
  c.def("__iadd__", [](const CartesianWrench& self, const CartesianPose& other) -> void { throw py::type_error("unsupported operand type(s) for +=: 'state_representation.CartesianWrench' and 'state_representation.CartesianPose'"); });
  c.def("__iadd__", [](const CartesianWrench& self, const CartesianTwist& other) -> void { throw py::type_error("unsupported operand type(s) for +=: 'state_representation.CartesianWrench' and 'state_representation.CartesianTwist'"); });
  c.def("__iadd__", [](const CartesianWrench& self, const CartesianAcceleration& other) -> void { throw py::type_error("unsupported operand type(s) for +=: 'state_representation.CartesianWrench' and 'state_representation.CartesianAcceleration'"); });
  c.def(py::self += CartesianState());
  c.def(py::self + py::self);
  c.def("__add__", [](const CartesianWrench& self, const CartesianPose& other) -> void { throw py::type_error("unsupported operand type(s) for +: 'state_representation.CartesianWrench' and 'state_representation.CartesianPose'"); });
  c.def("__add__", [](const CartesianWrench& self, const CartesianTwist& other) -> void { throw py::type_error("unsupported operand type(s) for +: 'state_representation.CartesianWrench' and 'state_representation.CartesianTwist'"); });
  c.def("__add__", [](const CartesianWrench& self, const CartesianAcceleration& other) -> void { throw py::type_error("unsupported operand type(s) for +: 'state_representation.CartesianWrench' and 'state_representation.CartesianAcceleration'"); });
  c.def(py::self + CartesianState());
  c.def("__neg__", [](const CartesianWrench& self) -> CartesianWrench { return -self; });
  c.def(py::self -= py::self);
  c.def("__isub__", [](const CartesianWrench& self, const CartesianPose& other) -> void { throw py::type_error("unsupported operand type(s) for -=: 'state_representation.CartesianWrench' and 'state_representation.CartesianPose'"); });
  c.def("__isub__", [](const CartesianWrench& self, const CartesianTwist& other) -> void { throw py::type_error("unsupported operand type(s) for -=: 'state_representation.CartesianWrench' and 'state_representation.CartesianTwist'"); });
  c.def("__isub__", [](const CartesianWrench& self, const CartesianAcceleration& other) -> void { throw py::type_error("unsupported operand type(s) for -=: 'state_representation.CartesianWrench' and 'state_representation.CartesianAcceleration'"); });
  c.def(py::self -= CartesianState());
  c.def(py::self - py::self);
  c.def("__sub__", [](const CartesianWrench& self, const CartesianPose& other) -> void { throw py::type_error("unsupported operand type(s) for -: 'state_representation.CartesianWrench' and 'state_representation.CartesianPose'"); });
  c.def("__sub__", [](const CartesianWrench& self, const CartesianTwist& other) -> void { throw py::type_error("unsupported operand type(s) for -: 'state_representation.CartesianWrench' and 'state_representation.CartesianTwist'"); });
  c.def("__sub__", [](const CartesianWrench& self, const CartesianAcceleration& other) -> void { throw py::type_error("unsupported operand type(s) for -: 'state_representation.CartesianWrench' and 'state_representation.CartesianAcceleration'"); });
  c.def(py::self - CartesianState());

  c.def("clamp", &CartesianWrench::clamp, "Clamp inplace the magnitude of the wrench to the values in argument", "max_force"_a, "max_torque"_a, "force_noise_ratio"_a=0, "torque_noise_ratio"_a=0);
  c.def("clamped", &CartesianWrench::clamped, "Return the clamped wrench", "max_force"_a, "max_torque"_a, "force_noise_ratio"_a=0, "torque_noise_ratio"_a=0);

  c.def("copy", &CartesianWrench::copy, "Return a copy of the CartesianWrench");
  c.def("data", &CartesianWrench::data, "Returns the wrench data as a vector");
  c.def("set_data", py::overload_cast<const Eigen::VectorXd&>(&CartesianWrench::set_data), "Set the wrench data from a vector", "data"_a);
  c.def("set_data", py::overload_cast<const std::vector<double>&>(&CartesianWrench::set_data), "Set the wrench data from a list", "data"_a);
  c.def("norms", &CartesianWrench::norms, "Compute the norms of the state variable specified by the input type (default is full wrench)", "state_variable_type"_a=CartesianStateVariable::WRENCH);
  c.def("normalized", &CartesianWrench::normalized, "Compute the normalized twist at the state variable given in argument (default is full wrench)", "state_variable_type"_a=CartesianStateVariable::WRENCH);

  c.def("__copy__", [](const CartesianWrench &wrench) {
    return CartesianWrench(wrench);
  });
  c.def("__deepcopy__", [](const CartesianWrench &wrench, py::dict) {
    return CartesianWrench(wrench);
  }, "memo"_a);
  c.def("__repr__", [](const CartesianWrench& wrench) {
    std::stringstream buffer;
    buffer << wrench;
    return buffer.str();
  });
}

void bind_cartesian_space(py::module_& m) {
  spatial_state(m);
  cartesian_state_variable(m);
  cartesian_state(m);
  cartesian_pose(m);
  cartesian_twist(m);
  cartesian_acceleration(m);
  cartesian_wrench(m);
}
