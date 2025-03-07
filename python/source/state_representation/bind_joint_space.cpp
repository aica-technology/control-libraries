#include "state_representation_bindings.hpp"

#include <state_representation/State.hpp>
#include <state_representation/space/joint/JointState.hpp>
#include <state_representation/space/joint/JointPositions.hpp>
#include <state_representation/space/joint/JointVelocities.hpp>
#include <state_representation/space/joint/JointTorques.hpp>

void joint_state_variable(py::module_& m) {
  py::enum_<JointStateVariable>(m, "JointStateVariable")
      .value("POSITIONS", JointStateVariable::POSITIONS)
      .value("VELOCITIES", JointStateVariable::VELOCITIES)
      .value("ACCELERATIONS", JointStateVariable::ACCELERATIONS)
      .value("TORQUES", JointStateVariable::TORQUES)
      .value("ALL", JointStateVariable::ALL)
      .export_values();

  m.def("string_to_joint_state_variable", &state_representation::string_to_joint_state_variable, "Convert a string to a JointStateVariable enum (case insensitive)", "variable"_a);
  m.def("joint_state_variable_to_string", &state_representation::joint_state_variable_to_string, "Convert JointStateVariable to a string", "variable"_a);
}

void joint_state(py::module_& m) {
  m.def("dist", py::overload_cast<const JointState&, const JointState&, const JointStateVariable&>(&state_representation::dist), "Compute the distance between two JointState.", "s1"_a, "s2"_a, "state_variable_type"_a=JointStateVariable::ALL);

  py::class_<JointState, std::shared_ptr<JointState>, State> c(m, "JointState");
  c.def(py::init(), "Empty constructor for a JointState.");
  c.def(py::init<const std::string&, unsigned int>(), "Constructor with name and number of joints provided.", "robot_name"_a, "nb_joints"_a=0);
  c.def(py::init<const std::string&, const std::vector<std::string>&>(), "Constructor with name and list of joint names provided.", "robot_name"_a, "joint_names"_a);
  c.def(py::init<const JointState&>(), "Copy constructor of a JointState.", "state"_a);

  c.def_static("Zero", py::overload_cast<const std::string&, unsigned int>(&JointState::Zero), "Constructor for a zero JointState.", "robot_name"_a, "nb_joints"_a);
  c.def_static("Zero", py::overload_cast<const std::string&, const std::vector<std::string>&>(&JointState::Zero), "Constructor for a zero JointState.", "robot_name"_a, "joint_names"_a);
  c.def_static("Random", py::overload_cast<const std::string&, unsigned int>(&JointState::Random), "Constructor for a random JointState.", "robot_name"_a, "nb_joints"_a);
  c.def_static("Random", py::overload_cast<const std::string&, const std::vector<std::string>&>(&JointState::Random), "Constructor for a random JointState.", "robot_name"_a, "joint_names"_a);

  c.def("get_size", &JointState::get_size, "Getter of the size from the attributes.");
  c.def("get_names", &JointState::get_names, "Getter of the names attribute.");
  c.def("get_joint_index", &JointState::get_joint_index, "Get joint index by the name of the joint, if it exists.", "joint_name"_a);
  c.def("get_positions", &JointState::get_positions, "Getter of the positions attribute.");
  c.def("get_position", [](const JointState& joint_state, const std::string& joint_name) { return joint_state.get_position(joint_name); }, "Get the position of a joint by its name, if it exists.", "joint_name"_a);
  c.def("get_position", [](const JointState& joint_state, unsigned int joint_index) { return joint_state.get_position(joint_index); }, "Get the position of a joint by its name, if it exists.", "joint_index"_a);
  c.def("get_velocities", &JointState::get_velocities, "Getter of the velocities attribute");
  c.def("get_velocity", [](const JointState& joint_state, const std::string& joint_name) { return joint_state.get_velocity(joint_name); }, "Get the velocity of a joint by its name, if it exists.", "joint_name"_a);
  c.def("get_velocity", [](const JointState& joint_state, unsigned int joint_index) { return joint_state.get_velocity(joint_index); }, "Get the velocity of a joint by its name, if it exists.", "joint_index"_a);
  c.def("get_accelerations", &JointState::get_accelerations, "Getter of the accelerations attribute");
  c.def("get_acceleration", [](const JointState& joint_state, const std::string& joint_name) { return joint_state.get_acceleration(joint_name); }, "Get the acceleration of a joint by its name, if it exists.", "joint_name"_a);
  c.def("get_acceleration", [](const JointState& joint_state, unsigned int joint_index) { return joint_state.get_acceleration(joint_index); }, "Get the acceleration of a joint by its name, if it exists.", "joint_index"_a);
  c.def("get_torques", &JointState::get_torques, "Getter of the torques attribute");
  c.def("get_torque", [](const JointState& joint_state, const std::string& joint_name) { return joint_state.get_torque(joint_name); }, "Get the torque of a joint by its name, if it exists.", "joint_name"_a);
  c.def("get_torque", [](const JointState& joint_state, unsigned int joint_index) { return joint_state.get_torque(joint_index); }, "Get the torque of a joint by its name, if it exists.", "joint_index"_a);

  c.def("set_names", py::overload_cast<unsigned int>(&JointState::set_names), "Setter of the names attribute from the number of joints.", "nb_joints"_a);
  c.def("set_names", py::overload_cast<const std::vector<std::string>&>(&JointState::set_names), "Setter of the names attribute.", "names"_a);
  c.def("set_positions", py::overload_cast<const Eigen::VectorXd&>(&JointState::set_positions), "Setter of the positions attribute from a vector.", "positions"_a);
  c.def("set_positions", py::overload_cast<const std::vector<double>&>(&JointState::set_positions), "Setter of the positions attribute from a list.", "positions"_a);
  c.def("set_position", py::overload_cast<double, const std::string&>(&JointState::set_position), "Set the position of a joint by its name.", "position"_a, "joint_name"_a);
  c.def("set_position", py::overload_cast<double, unsigned int>(&JointState::set_position), "Set the position of a joint by its index.", "position"_a, "joint_index"_a);
  c.def("set_velocities", py::overload_cast<const Eigen::VectorXd&>(&JointState::set_velocities), "Setter of the velocities attribute from a vector.", "velocities"_a);
  c.def("set_velocities", py::overload_cast<const std::vector<double>&>(&JointState::set_velocities), "Setter of the velocities attribute from a list.", "velocities"_a);
  c.def("set_velocity", py::overload_cast<double, const std::string&>(&JointState::set_velocity), "Set the velocity of a joint by its name.", "velocity"_a, "joint_name"_a);
  c.def("set_velocity", py::overload_cast<double, unsigned int>(&JointState::set_velocity), "Set the velocity of a joint by its index.", "velocity"_a, "joint_index"_a);
  c.def("set_accelerations", py::overload_cast<const Eigen::VectorXd&>(&JointState::set_accelerations), "Setter of the accelerations attribute from a vector.", "accelerations"_a);
  c.def("set_accelerations", py::overload_cast<const std::vector<double>&>(&JointState::set_accelerations), "Setter of the accelerations attribute from a list.", "accelerations"_a);
  c.def("set_acceleration", py::overload_cast<double, const std::string&>(&JointState::set_acceleration), "Set the acceleration of a joint by its name.", "acceleration"_a, "joint_name"_a);
  c.def("set_acceleration", py::overload_cast<double, unsigned int>(&JointState::set_acceleration), "Set the acceleration of a joint by its index.", "acceleration"_a, "joint_index"_a);
  c.def("set_torques", py::overload_cast<const Eigen::VectorXd&>(&JointState::set_torques), "Setter of the torques attribute from a vector.", "torques"_a);
  c.def("set_torques", py::overload_cast<const std::vector<double>&>(&JointState::set_torques), "Setter of the torques attribute from a list.", "torques"_a);
  c.def("set_torque", py::overload_cast<double, const std::string&>(&JointState::set_torque), "Set the torque of a joint by its name.", "torque"_a, "joint_name"_a);
  c.def("set_torque", py::overload_cast<double, unsigned int>(&JointState::set_torque), "Set the torque of a joint by its index.", "torque"_a, "joint_index"_a);

  c.def("set_zero", &JointState::set_zero, "Set the JointState to a zero value.");
  c.def("clamp_state_variable", py::overload_cast<double, const JointStateVariable&, double>(&JointState::clamp_state_variable), "Clamp inplace the magnitude of the a specific state variable.", "value"_a, "state_variable_type"_a, "noise_ratio"_a=double(0));
  c.def("clamp_state_variable", py::overload_cast<const Eigen::ArrayXd&, const JointStateVariable&, const Eigen::ArrayXd&>(&JointState::clamp_state_variable), "Clamp inplace the magnitude of the a specific state variable.", "max_absolute_value_array"_a, "state_variable_type"_a, "noise_ratio_array"_a);
  c.def("copy", &JointState::copy, "Return a copy of the JointState.");
  c.def("data", &JointState::data, "Returns the data as the concatenation of all the state variables in a single vector.");
  c.def("array", &JointState::array, "Returns the data vector as an array.");
  c.def("set_data", py::overload_cast<const Eigen::VectorXd&>(&JointState::set_data), "Set the data of the state from all the state variables in a single vector.", "data"_a);
  c.def("set_data", py::overload_cast<const std::vector<double>&>(&JointState::set_data), "Set the data of the state from all the state variables in a single list.", "data"_a);

  c.def(py::self *= double());
  c.def(py::self * double());
  c.def(double() * py::self);
  c.def("__mul__", [](const JointState& self, const Eigen::MatrixXd& other) -> void { throw py::type_error("unsupported operand type(s) for *: 'state_representation.JointState' and 'np.ndarray'"); });
  c.def("__imul__", [](const JointState& self, const Eigen::MatrixXd& other) -> void { throw py::type_error("unsupported operand type(s) for *=: 'state_representation.JointState' and 'np.ndarray'"); });
  c.def(Eigen::MatrixXd() * py::self);
  c.def(py::self /= double());
  c.def(py::self / double());
  c.def("__truediv__", [](const JointState& self, const Eigen::MatrixXd& other) -> void { throw py::type_error("unsupported operand type(s) for /: 'state_representation.JointState' and 'np.ndarray'"); });

  c.def(py::self += py::self);
  c.def(py::self + py::self);
  c.def("__neg__", [](const JointState& self) -> JointState { return -self; });
  c.def(py::self -= py::self);
  c.def(py::self - py::self);

  c.def("dist", &JointState::dist, "Compute the distance to another state as the sum of distances between each attribute.", "state"_a, "state_variable_type"_a=JointStateVariable::ALL);

  c.def("to_list", &JointState::to_std_vector, "Return the state as a list.");

  c.def("__copy__", [](const JointState &state) {
    return JointState(state);
  });
  c.def("__deepcopy__", [](const JointState &state, py::dict) {
    return JointState(state);
  }, "memo"_a);
  c.def("__repr__", [](const JointState& state) {
    std::stringstream buffer;
    buffer << state;
    return buffer.str();
  });

  c.def("multiply_state_variable", py::overload_cast<const Eigen::MatrixXd&, const JointStateVariable&>(&JointState::multiply_state_variable), "Proxy function that scale the specified state variable by a matrix", "matrix"_a, "state_variable_type"_a);
  c.def("get_state_variable", &JointState::get_state_variable, "Getter of the variable value corresponding to the input", "state_variable_type"_a);
  c.def("set_state_variable", py::overload_cast<const Eigen::VectorXd&, const JointStateVariable&>(&JointState::set_state_variable), "Setter of the variable value corresponding to the input", "new_value"_a, "state_variable_type"_a);
  c.def("set_state_variable", py::overload_cast<const std::vector<double>&, const JointStateVariable&>(&JointState::set_state_variable), "Setter of the variable value corresponding to the input", "new_value"_a, "state_variable_type"_a);
  c.def("set_state_variable", py::overload_cast<double, unsigned int, const JointStateVariable&>(&JointState::set_state_variable), "Setter of the variable value corresponding to the input", "new_value"_a, "joint_index"_a, "state_variable_type"_a);
}

void joint_positions(py::module_& m) {
  py::class_<JointPositions, std::shared_ptr<JointPositions>, JointState> c(m, "JointPositions");

  c.def(py::init(), "Empty constructor");
  c.def(py::init<const std::string&, unsigned int>(), "Constructor with name and number of joints provided", "robot_name"_a, "nb_joints"_a=0);
  c.def(py::init<const std::string&, const std::vector<std::string>&>(), "Constructor with name and list of joint names provided", "robot_name"_a, "joint_names"_a);
  c.def(py::init<const std::string&, const Eigen::VectorXd&>(), "Constructor with name and position values provided", "robot_name"_a, "positions"_a);
  c.def(py::init<const std::string&, const std::vector<std::string>&, const Eigen::VectorXd&>(), "Constructor with name, a list of joint names and position values provided", "robot_name"_a, "joint_names"_a, "positions"_a);
  c.def(py::init<const JointPositions&>(), "Copy constructor", "positions"_a);
  c.def(py::init<const JointState&>(), "Copy constructor from a JointState", "state"_a);
  c.def(py::init<const JointVelocities&>(), "Integration constructor from a JointVelocities by considering that it is equivalent to multiplying the velocities by 1 second", "velocities"_a);

  c.def_static("Zero", py::overload_cast<const std::string&, unsigned int>(&JointPositions::Zero), "Constructor for the zero JointPositions", "robot_name"_a, "nb_joints"_a);
  c.def_static("Zero", py::overload_cast<const std::string&, const std::vector<std::string>&>(&JointPositions::Zero), "Constructor for the zero JointPositions", "robot_name"_a, "joint_names"_a);
  c.def_static("Random", py::overload_cast<const std::string&, unsigned int>(&JointPositions::Random), "Constructor for the random JointPositions", "robot_name"_a, "nb_joints"_a);
  c.def_static("Random", py::overload_cast<const std::string&, const std::vector<std::string>&>(&JointPositions::Random), "Constructor for the random JointPositions", "robot_name"_a, "joint_names"_a);

  std::vector<std::string> deleted_attributes = {
      "velocities",
      "velocity",
      "accelerations",
      "acceleration",
      "torques",
      "torque",
  };

  for (const std::string& attr : deleted_attributes) {
    c.def(std::string("get_" + attr).c_str(), [](const JointPositions&) -> void {}, "Deleted method from parent class.");
    c.def(std::string("set_" + attr).c_str(), [](const JointPositions& positions) -> JointPositions { return positions; }, "Deleted method from parent class.");
  }

  c.def(py::self *= double());
  c.def(py::self * double());
  c.def(double() * py::self);
  c.def("__mul__", [](const JointPositions& self, const Eigen::MatrixXd& other) -> void { throw py::type_error("unsupported operand type(s) for *: 'state_representation.JointPositions' and 'np.ndarray'"); });
  c.def("__imul__", [](const JointPositions& self, const Eigen::MatrixXd& other) -> void { throw py::type_error("unsupported operand type(s) for *=: 'state_representation.JointPositions' and 'np.ndarray'"); });
  c.def(Eigen::MatrixXd() * py::self);
  c.def(py::self /= double());
  c.def(py::self / double());
  c.def("__truediv__", [](const JointPositions& self, const Eigen::MatrixXd& other) -> void { throw py::type_error("unsupported operand type(s) for /: 'state_representation.JointPositions' and 'np.ndarray'"); });
  c.def(py::self / std::chrono::nanoseconds());

  c.def(py::self += py::self);
  c.def("__iadd__", [](const JointPositions& self, const JointVelocities& other) -> void { throw py::type_error("unsupported operand type(s) for +=: 'state_representation.JointPositions' and 'state_representation.JointVelocities'"); });
  c.def("__iadd__", [](const JointPositions& self, const JointAccelerations& other) -> void { throw py::type_error("unsupported operand type(s) for +=: 'state_representation.JointPositions' and 'state_representation.JointAccelerations'"); });
  c.def("__iadd__", [](const JointPositions& self, const JointTorques& other) -> void { throw py::type_error("unsupported operand type(s) for +=: 'state_representation.JointPositions' and 'state_representation.JointTorques'"); });
  c.def(py::self += JointState());
  c.def(py::self + py::self);
  c.def("__add__", [](const JointPositions& self, const JointVelocities& other) -> void { throw py::type_error("unsupported operand type(s) for +: 'state_representation.JointPositions' and 'state_representation.JointVelocities'"); });
  c.def("__add__", [](const JointPositions& self, const JointAccelerations& other) -> void { throw py::type_error("unsupported operand type(s) for +: 'state_representation.JointPositions' and 'state_representation.JointAccelerations'"); });
  c.def("__add__", [](const JointPositions& self, const JointTorques& other) -> void { throw py::type_error("unsupported operand type(s) for +: 'state_representation.JointPositions' and 'state_representation.JointTorques'"); });
  c.def(py::self + JointState());
  c.def("__neg__", [](const JointPositions& self) -> JointPositions { return -self; });
  c.def(py::self -= py::self);
  c.def("__isub__", [](const JointPositions& self, const JointVelocities& other) -> void { throw py::type_error("unsupported operand type(s) for -=: 'state_representation.JointPositions' and 'state_representation.JointVelocities'"); });
  c.def("__isub__", [](const JointPositions& self, const JointAccelerations& other) -> void { throw py::type_error("unsupported operand type(s) for -=: 'state_representation.JointPositions' and 'state_representation.JointAccelerations'"); });
  c.def("__isub__", [](const JointPositions& self, const JointTorques& other) -> void { throw py::type_error("unsupported operand type(s) for -=: 'state_representation.JointPositions' and 'state_representation.JointTorques'"); });
  c.def(py::self -= JointState());
  c.def(py::self - py::self);
  c.def("__sub__", [](const JointPositions& self, const JointVelocities& other) -> void { throw py::type_error("unsupported operand type(s) for -: 'state_representation.JointPositions' and 'state_representation.JointVelocities'"); });
  c.def("__sub__", [](const JointPositions& self, const JointAccelerations& other) -> void { throw py::type_error("unsupported operand type(s) for -: 'state_representation.JointPositions' and 'state_representation.JointAccelerations'"); });
  c.def("__sub__", [](const JointPositions& self, const JointTorques& other) -> void { throw py::type_error("unsupported operand type(s) for -: 'state_representation.JointPositions' and 'state_representation.JointTorques'"); });
  c.def(py::self - JointState());

  c.def("copy", &JointPositions::copy, "Return a copy of the JointPositions");
  c.def("data", &JointPositions::data, "Returns the positions data as a vector");
  c.def("set_data", py::overload_cast<const Eigen::VectorXd&>(&JointPositions::set_data), "Set the positions data from a vector", "data"_a);
  c.def("set_data", py::overload_cast<const std::vector<double>&>(&JointPositions::set_data), "Set the positions data from a list", "data"_a);

  c.def("differentiate", [](const JointPositions &positions, double dt) -> JointVelocities {
    return positions.differentiate(dt);
  }, "Differentiate joint positions over a time period in seconds", "dt"_a);
  c.def("differentiate", [](const JointPositions &positions, const std::chrono::nanoseconds& dt) -> JointVelocities {
    return positions.differentiate(dt);
  }, "Differentiate joint positions over a time period", "dt"_a);

  c.def("__copy__", [](const JointPositions &positions) {
    return JointPositions(positions);
  });
  c.def("__deepcopy__", [](const JointPositions &positions, py::dict) {
    return JointPositions(positions);
  }, "memo"_a);
  c.def("__repr__", [](const JointPositions& positions) {
    std::stringstream buffer;
    buffer << positions;
    return buffer.str();
  });
}

void joint_velocities(py::module_& m) {
  py::class_<JointVelocities, std::shared_ptr<JointVelocities>, JointState> c(m, "JointVelocities");

  c.def(py::init(), "Empty constructor");
  c.def(py::init<const std::string&, unsigned int>(), "Constructor with name and number of joints provided", "robot_name"_a, "nb_joints"_a=0);
  c.def(py::init<const std::string&, const std::vector<std::string>&>(), "Constructor with name and list of joint names provided", "robot_name"_a, "joint_names"_a);
  c.def(py::init<const std::string&, const Eigen::VectorXd&>(), "Constructor with name and velocity values provided", "robot_name"_a, "velocities"_a);
  c.def(py::init<const std::string&, const std::vector<std::string>&, const Eigen::VectorXd&>(), "Constructor with name, a list of joint names and velocity values provided", "robot_name"_a, "joint_names"_a, "velocities"_a);
  c.def(py::init<const JointVelocities&>(), "Copy constructor", "velocities"_a);
  c.def(py::init<const JointState&>(), "Copy constructor from a JointState", "state"_a);
  c.def(py::init<const JointPositions&>(), "Differentiation constructor from a JointPositions by considering that it is equivalent to dividing the positions by 1 second", "positions"_a);
  c.def(py::init<const JointAccelerations&>(), "Integration constructor from a JointAccelerations by considering that it is equivalent to multiplying the accelerations by 1 second", "accelerations"_a);

  c.def_static("Zero", py::overload_cast<const std::string&, unsigned int>(&JointVelocities::Zero), "Constructor for the zero JointVelocities", "robot_name"_a, "nb_joints"_a);
  c.def_static("Zero", py::overload_cast<const std::string&, const std::vector<std::string>&>(&JointVelocities::Zero), "Constructor for the zero JointVelocities", "robot_name"_a, "joint_names"_a);
  c.def_static("Random", py::overload_cast<const std::string&, unsigned int>(&JointVelocities::Random), "Constructor for the random JointVelocities", "robot_name"_a, "nb_joints"_a);
  c.def_static("Random", py::overload_cast<const std::string&, const std::vector<std::string>&>(&JointVelocities::Random), "Constructor for the random JointVelocities", "robot_name"_a, "joint_names"_a);

  std::vector<std::string> deleted_attributes = {
      "positions",
      "position",
      "accelerations",
      "acceleration",
      "torques",
      "torque",
  };

  for (const std::string& attr : deleted_attributes) {
    c.def(std::string("get_" + attr).c_str(), [](const JointVelocities&) -> void {}, "Deleted method from parent class.");
    c.def(std::string("set_" + attr).c_str(), [](const JointVelocities& velocities) -> JointVelocities { return velocities; }, "Deleted method from parent class.");
  }

  c.def(py::self *= double());
  c.def(py::self * double());
  c.def(double() * py::self);
  c.def("__mul__", [](const JointVelocities& self, const Eigen::MatrixXd& other) -> void { throw py::type_error("unsupported operand type(s) for *: 'state_representation.JointVelocities' and 'np.ndarray'"); });
  c.def("__imul__", [](const JointVelocities& self, const Eigen::MatrixXd& other) -> void { throw py::type_error("unsupported operand type(s) for *=: 'state_representation.JointVelocities' and 'np.ndarray'"); });
  c.def(Eigen::MatrixXd() * py::self);
  c.def(py::self * std::chrono::nanoseconds());
  c.def(std::chrono::nanoseconds() * py::self);
  c.def(py::self /= double());
  c.def(py::self / double());
  c.def("__truediv__", [](const JointVelocities& self, const Eigen::MatrixXd& other) -> void { throw py::type_error("unsupported operand type(s) for /: 'state_representation.JointVelocities' and 'np.ndarray'"); });
  c.def(py::self / std::chrono::nanoseconds());

  c.def(py::self += py::self);
  c.def("__iadd__", [](const JointVelocities& self, const JointPositions& other) -> void { throw py::type_error("unsupported operand type(s) for +=: 'state_representation.JointVelocities' and 'state_representation.JointPositions'"); });
  c.def("__iadd__", [](const JointVelocities& self, const JointAccelerations& other) -> void { throw py::type_error("unsupported operand type(s) for +=: 'state_representation.JointVelocities' and 'state_representation.JointAccelerations'"); });
  c.def("__iadd__", [](const JointVelocities& self, const JointTorques& other) -> void { throw py::type_error("unsupported operand type(s) for +=: 'state_representation.JointVelocities' and 'state_representation.JointTorques'"); });
  c.def(py::self += JointState());
  c.def(py::self + py::self);
  c.def("__add__", [](const JointVelocities& self, const JointPositions& other) -> void { throw py::type_error("unsupported operand type(s) for +: 'state_representation.JointVelocities' and 'state_representation.JointPositions'"); });
  c.def("__add__", [](const JointVelocities& self, const JointAccelerations& other) -> void { throw py::type_error("unsupported operand type(s) for +: 'state_representation.JointVelocities' and 'state_representation.JointAccelerations'"); });
  c.def("__add__", [](const JointVelocities& self, const JointTorques& other) -> void { throw py::type_error("unsupported operand type(s) for +: 'state_representation.JointVelocities' and 'state_representation.JointTorques'"); });
  c.def(py::self + JointState());
  c.def("__neg__", [](const JointVelocities& self) -> JointVelocities { return -self; });
  c.def(py::self -= py::self);
  c.def("__isub__", [](const JointVelocities& self, const JointPositions& other) -> void { throw py::type_error("unsupported operand type(s) for -=: 'state_representation.JointVelocities' and 'state_representation.JointPositions'"); });
  c.def("__isub__", [](const JointVelocities& self, const JointAccelerations& other) -> void { throw py::type_error("unsupported operand type(s) for -=: 'state_representation.JointVelocities' and 'state_representation.JointAccelerations'"); });
  c.def("__isub__", [](const JointVelocities& self, const JointTorques& other) -> void { throw py::type_error("unsupported operand type(s) for -=: 'state_representation.JointVelocities' and 'state_representation.JointTorques'"); });
  c.def(py::self -= JointState());
  c.def(py::self - py::self);
  c.def("__sub__", [](const JointVelocities& self, const JointPositions& other) -> void { throw py::type_error("unsupported operand type(s) for -: 'state_representation.JointVelocities' and 'state_representation.JointPositions'"); });
  c.def("__sub__", [](const JointVelocities& self, const JointAccelerations& other) -> void { throw py::type_error("unsupported operand type(s) for -: 'state_representation.JointVelocities' and 'state_representation.JointAccelerations'"); });
  c.def("__sub__", [](const JointVelocities& self, const JointTorques& other) -> void { throw py::type_error("unsupported operand type(s) for -: 'state_representation.JointVelocities' and 'state_representation.JointTorques'"); });
  c.def(py::self - JointState());

  c.def("copy", &JointVelocities::copy, "Return a copy of the JointVelocities");
  c.def("data", &JointVelocities::data, "Returns the velocities data as a vector");
  c.def("set_data", py::overload_cast<const Eigen::VectorXd&>(&JointVelocities::set_data), "Set the velocities data from a vector", "data"_a);
  c.def("set_data", py::overload_cast<const std::vector<double>&>(&JointVelocities::set_data), "Set the velocities data from a list", "data"_a);

  c.def("differentiate", [](const JointVelocities &velocities, double dt) -> JointAccelerations {
    return velocities.differentiate(dt);
  }, "Differentiate joint velocities over a time period in seconds", "dt"_a);
  c.def("differentiate", [](const JointVelocities &velocities, const std::chrono::nanoseconds& dt) -> JointAccelerations {
    return velocities.differentiate(dt);
  }, "Differentiate joint velocities over a time period", "dt"_a);
  c.def("integrate", [](const JointVelocities &velocities, double dt) -> JointPositions {
    return velocities.integrate(dt);
  }, "Integrate joint velocities over a time period in seconds", "dt"_a);
  c.def("integrate", [](const JointVelocities &velocities, const std::chrono::nanoseconds& dt) -> JointPositions {
    return velocities.integrate(dt);
  }, "Integrate joint velocities over a time period", "dt"_a);

  c.def("clamp", py::overload_cast<double, double>(&JointVelocities::clamp), "Clamp inplace the magnitude of the velocity to the values in argument", "max_absolute_value"_a, "noise_ratio"_a=0.0);
  c.def("clamped", py::overload_cast<double, double>(&JointVelocities::clamp), "Return the velocity clamped to the values in argument", "max_absolute_value"_a, "noise_ratio"_a=0.0);
  c.def("clamp", py::overload_cast<const Eigen::ArrayXd&, const Eigen::ArrayXd&>(&JointVelocities::clamp), "Clamp inplace the magnitude of the velocity to the values in argument", "max_absolute_value_array"_a, "noise_ratio_array"_a);
  c.def("clamped", py::overload_cast<const Eigen::ArrayXd&, const Eigen::ArrayXd&>(&JointVelocities::clamp), "Return the velocity clamped to the values in argument", "max_absolute_value_array"_a, "noise_ratio_array"_a);

  c.def("__copy__", [](const JointVelocities &velocities) {
    return JointVelocities(velocities);
  });
  c.def("__deepcopy__", [](const JointVelocities &velocities, py::dict) {
    return JointVelocities(velocities);
  }, "memo"_a);
  c.def("__repr__", [](const JointVelocities& velocities) {
    std::stringstream buffer;
    buffer << velocities;
    return buffer.str();
  });
}

void joint_accelerations(py::module_& m) {
  py::class_<JointAccelerations, std::shared_ptr<JointAccelerations>, JointState> c(m, "JointAccelerations");

  c.def(py::init(), "Empty constructor");
  c.def(py::init<const std::string&, unsigned int>(), "Constructor with name and number of joints provided", "robot_name"_a, "nb_joints"_a=0);
  c.def(py::init<const std::string&, const std::vector<std::string>&>(), "Constructor with name and list of joint names provided", "robot_name"_a, "joint_names"_a);
  c.def(py::init<const std::string&, const Eigen::VectorXd&>(), "Constructor with name and acceleration values provided", "robot_name"_a, "accelerations"_a);
  c.def(py::init<const std::string&, const std::vector<std::string>&, const Eigen::VectorXd&>(), "Constructor with name, a list of joint names and acceleration values provided", "robot_name"_a, "joint_names"_a, "accelerations"_a);
  c.def(py::init<const JointAccelerations&>(), "Copy constructor", "accelerations"_a);
  c.def(py::init<const JointState&>(), "Copy constructor from a JointState", "state"_a);
  c.def(py::init<const JointVelocities&>(), "Differentiation constructor from a JointVelocities by considering that it is equivalent to dividing the velocities by 1 second", "velocities"_a);

  c.def_static("Zero", py::overload_cast<const std::string&, unsigned int>(&JointAccelerations::Zero), "Constructor for the zero JointAccelerations", "robot_name"_a, "nb_joints"_a);
  c.def_static("Zero", py::overload_cast<const std::string&, const std::vector<std::string>&>(&JointAccelerations::Zero), "Constructor for the zero JointAccelerations", "robot_name"_a, "joint_names"_a);
  c.def_static("Random", py::overload_cast<const std::string&, unsigned int>(&JointAccelerations::Random), "Constructor for the random JointAccelerations", "robot_name"_a, "nb_joints"_a);
  c.def_static("Random", py::overload_cast<const std::string&, const std::vector<std::string>&>(&JointAccelerations::Random), "Constructor for the random JointAccelerations", "robot_name"_a, "joint_names"_a);

  std::vector<std::string> deleted_attributes = {
      "positions",
      "position",
      "velocities",
      "velocity",
      "torques",
      "torque",
  };

  for (const std::string& attr : deleted_attributes) {
    c.def(std::string("get_" + attr).c_str(), [](const JointAccelerations&) -> void {}, "Deleted method from parent class.");
    c.def(std::string("set_" + attr).c_str(), [](const JointAccelerations& accelerations) -> JointAccelerations { return accelerations; }, "Deleted method from parent class.");
  }

  c.def(py::self *= double());
  c.def(py::self * double());
  c.def(double() * py::self);
  c.def("__mul__", [](const JointAccelerations& self, const Eigen::MatrixXd& other) -> void { throw py::type_error("unsupported operand type(s) for *: 'state_representation.JointAccelerations' and 'np.ndarray'"); });
  c.def("__imul__", [](const JointAccelerations& self, const Eigen::MatrixXd& other) -> void { throw py::type_error("unsupported operand type(s) for *=: 'state_representation.JointAccelerations' and 'np.ndarray'"); });
  c.def(Eigen::MatrixXd() * py::self);
  c.def(py::self * std::chrono::nanoseconds());
  c.def(std::chrono::nanoseconds() * py::self);
  c.def(py::self /= double());
  c.def(py::self / double());
  c.def("__truediv__", [](const JointAccelerations& self, const Eigen::MatrixXd& other) -> void { throw py::type_error("unsupported operand type(s) for /: 'state_representation.JointAccelerations' and 'np.ndarray'"); });

  c.def(py::self += py::self);
  c.def("__iadd__", [](const JointAccelerations& self, const JointPositions& other) -> void { throw py::type_error("unsupported operand type(s) for +=: 'state_representation.JointAccelerations' and 'state_representation.JointPositions'"); });
  c.def("__iadd__", [](const JointAccelerations& self, const JointVelocities& other) -> void { throw py::type_error("unsupported operand type(s) for +=: 'state_representation.JointAccelerations' and 'state_representation.JointVelocities'"); });
  c.def("__iadd__", [](const JointAccelerations& self, const JointTorques& other) -> void { throw py::type_error("unsupported operand type(s) for +=: 'state_representation.JointAccelerations' and 'state_representation.JointTorques'"); });
  c.def(py::self += JointState());
  c.def(py::self + py::self);
  c.def("__add__", [](const JointAccelerations& self, const JointPositions& other) -> void { throw py::type_error("unsupported operand type(s) for +: 'state_representation.JointAccelerations' and 'state_representation.JointPositions'"); });
  c.def("__add__", [](const JointAccelerations& self, const JointVelocities& other) -> void { throw py::type_error("unsupported operand type(s) for +: 'state_representation.JointAccelerations' and 'state_representation.JointVelocities'"); });
  c.def("__add__", [](const JointAccelerations& self, const JointTorques& other) -> void { throw py::type_error("unsupported operand type(s) for +: 'state_representation.JointAccelerations' and 'state_representation.JointTorques'"); });
  c.def(py::self + JointState());
  c.def("__neg__", [](const JointAccelerations& self) -> JointAccelerations { return -self; });
  c.def(py::self -= py::self);
  c.def("__isub__", [](const JointAccelerations& self, const JointPositions& other) -> void { throw py::type_error("unsupported operand type(s) for -=: 'state_representation.JointAccelerations' and 'state_representation.JointPositions'"); });
  c.def("__isub__", [](const JointAccelerations& self, const JointVelocities& other) -> void { throw py::type_error("unsupported operand type(s) for -=: 'state_representation.JointAccelerations' and 'state_representation.JointVelocities'"); });
  c.def("__isub__", [](const JointAccelerations& self, const JointTorques& other) -> void { throw py::type_error("unsupported operand type(s) for -=: 'state_representation.JointAccelerations' and 'state_representation.JointTorques'"); });
  c.def(py::self -= JointState());
  c.def(py::self - py::self);
  c.def("__sub__", [](const JointAccelerations& self, const JointPositions& other) -> void { throw py::type_error("unsupported operand type(s) for -: 'state_representation.JointAccelerations' and 'state_representation.JointPositions'"); });
  c.def("__sub__", [](const JointAccelerations& self, const JointVelocities& other) -> void { throw py::type_error("unsupported operand type(s) for -: 'state_representation.JointAccelerations' and 'state_representation.JointVelocities'"); });
  c.def("__sub__", [](const JointAccelerations& self, const JointTorques& other) -> void { throw py::type_error("unsupported operand type(s) for -: 'state_representation.JointAccelerations' and 'state_representation.JointTorques'"); });
  c.def(py::self - JointState());

  c.def("copy", &JointAccelerations::copy, "Return a copy of the JointAccelerations");
  c.def("data", &JointAccelerations::data, "Returns the accelerations data as a vector");
  c.def("set_data", py::overload_cast<const Eigen::VectorXd&>(&JointAccelerations::set_data), "Set the accelerations data from a vector", "data"_a);
  c.def("set_data", py::overload_cast<const std::vector<double>&>(&JointAccelerations::set_data), "Set the accelerations data from a list", "data"_a);

  c.def("integrate", [](const JointAccelerations &accelerations, double dt) -> JointVelocities {
    return accelerations.integrate(dt);
  }, "Integrate joint accelerations over a time period in seconds", "dt"_a);
  c.def("integrate", [](const JointAccelerations &accelerations, const std::chrono::nanoseconds& dt) -> JointVelocities {
    return accelerations.integrate(dt);
  }, "Integrate joint accelerations over a time period", "dt"_a);

  c.def("clamp", py::overload_cast<double, double>(&JointAccelerations::clamp), "Clamp inplace the magnitude of the accelerations to the values in argument", "max_absolute_value"_a, "noise_ratio"_a=0.0);
  c.def("clamped", py::overload_cast<double, double>(&JointAccelerations::clamp), "Return the accelerations clamped to the values in argument", "max_absolute_value"_a, "noise_ratio"_a=0.0);
  c.def("clamp", py::overload_cast<const Eigen::ArrayXd&, const Eigen::ArrayXd&>(&JointAccelerations::clamp), "Clamp inplace the magnitude of the accelerations to the values in argument", "max_absolute_value_array"_a, "noise_ratio_array"_a);
  c.def("clamped", py::overload_cast<const Eigen::ArrayXd&, const Eigen::ArrayXd&>(&JointAccelerations::clamp), "Return the accelerations clamped to the values in argument", "max_absolute_value_array"_a, "noise_ratio_array"_a);

  c.def("__copy__", [](const JointAccelerations &accelerations) {
    return JointAccelerations(accelerations);
  });
  c.def("__deepcopy__", [](const JointAccelerations &accelerations, py::dict) {
    return JointAccelerations(accelerations);
  }, "memo"_a);
  c.def("__repr__", [](const JointAccelerations& accelerations) {
    std::stringstream buffer;
    buffer << accelerations;
    return buffer.str();
  });
}

void joint_torques(py::module_& m) {
  py::class_<JointTorques, std::shared_ptr<JointTorques>, JointState> c(m, "JointTorques");

  c.def(py::init(), "Empty constructor");
  c.def(py::init<const std::string&, unsigned int>(), "Constructor with name and number of joints provided", "robot_name"_a, "nb_joints"_a=0);
  c.def(py::init<const std::string&, const std::vector<std::string>&>(), "Constructor with name and list of joint names provided", "robot_name"_a, "joint_names"_a);
  c.def(py::init<const std::string&, const Eigen::VectorXd&>(), "Constructor with name and torque values provided", "robot_name"_a, "velocities"_a);
  c.def(py::init<const std::string&, const std::vector<std::string>&, const Eigen::VectorXd&>(), "Constructor with name, a list of joint names and torque values provided", "robot_name"_a, "joint_names"_a, "velocities"_a);
  c.def(py::init<const JointTorques&>(), "Copy constructor", "torques"_a);
  c.def(py::init<const JointState&>(), "Copy constructor from a JointState", "state"_a);

  c.def_static("Zero", py::overload_cast<const std::string&, unsigned int>(&JointTorques::Zero), "Constructor for the zero JointTorques", "robot_name"_a, "nb_joints"_a);
  c.def_static("Zero", py::overload_cast<const std::string&, const std::vector<std::string>&>(&JointTorques::Zero), "Constructor for the zero JointTorques", "robot_name"_a, "joint_names"_a);
  c.def_static("Random", py::overload_cast<const std::string&, unsigned int>(&JointTorques::Random), "Constructor for the random JointTorques", "robot_name"_a, "nb_joints"_a);
  c.def_static("Random", py::overload_cast<const std::string&, const std::vector<std::string>&>(&JointTorques::Random), "Constructor for the random JointTorques", "robot_name"_a, "joint_names"_a);

  std::vector<std::string> deleted_attributes = {
      "positions",
      "position",
      "velocities",
      "velocity",
      "accelerations",
      "acceleration",
  };

  for (const std::string& attr : deleted_attributes) {
    c.def(std::string("get_" + attr).c_str(), [](const JointTorques&) -> void {}, "Deleted method from parent class.");
    c.def(std::string("set_" + attr).c_str(), [](const JointTorques& torques) -> JointTorques { return torques; }, "Deleted method from parent class.");
  }

  c.def(py::self *= double());
  c.def(py::self * double());
  c.def(double() * py::self);
  c.def("__mul__", [](const JointTorques& self, const Eigen::MatrixXd& other) -> void { throw py::type_error("unsupported operand type(s) for *: 'state_representation.JointTorques' and 'np.ndarray'"); });
  c.def("__imul__", [](const JointTorques& self, const Eigen::MatrixXd& other) -> void { throw py::type_error("unsupported operand type(s) for *=: 'state_representation.JointTorques' and 'np.ndarray'"); });
  c.def(Eigen::MatrixXd() * py::self);
  c.def(py::self /= double());
  c.def(py::self / double());
  c.def("__truediv__", [](const JointTorques& self, const Eigen::MatrixXd& other) -> void { throw py::type_error("unsupported operand type(s) for /: 'state_representation.JointTorques' and 'np.ndarray'"); });

  c.def(py::self += py::self);
  c.def("__iadd__", [](const JointTorques& self, const JointPositions& other) -> void { throw py::type_error("unsupported operand type(s) for +=: 'state_representation.JointTorques' and 'state_representation.JointPositions'"); });
  c.def("__iadd__", [](const JointTorques& self, const JointVelocities& other) -> void { throw py::type_error("unsupported operand type(s) for +=: 'state_representation.JointTorques' and 'state_representation.JointVelocities'"); });
  c.def("__iadd__", [](const JointTorques& self, const JointAccelerations& other) -> void { throw py::type_error("unsupported operand type(s) for +=: 'state_representation.JointTorques' and 'state_representation.JointAccelerations'"); });
  c.def(py::self += JointState());
  c.def(py::self + py::self);
  c.def("__add__", [](const JointTorques& self, const JointPositions& other) -> void { throw py::type_error("unsupported operand type(s) for +: 'state_representation.JointTorques' and 'state_representation.JointPositions'"); });
  c.def("__add__", [](const JointTorques& self, const JointVelocities& other) -> void { throw py::type_error("unsupported operand type(s) for +: 'state_representation.JointTorques' and 'state_representation.JointVelocities'"); });
  c.def("__add__", [](const JointTorques& self, const JointAccelerations& other) -> void { throw py::type_error("unsupported operand type(s) for +: 'state_representation.JointTorques' and 'state_representation.JointAccelerations'"); });
  c.def(py::self + JointState());
  c.def("__neg__", [](const JointTorques& self) -> JointTorques { return -self; });
  c.def(py::self -= py::self);
  c.def("__isub__", [](const JointTorques& self, const JointPositions& other) -> void { throw py::type_error("unsupported operand type(s) for -=: 'state_representation.JointTorques' and 'state_representation.JointPositions'"); });
  c.def("__isub__", [](const JointTorques& self, const JointVelocities& other) -> void { throw py::type_error("unsupported operand type(s) for -=: 'state_representation.JointTorques' and 'state_representation.JointVelocities'"); });
  c.def("__isub__", [](const JointTorques& self, const JointAccelerations& other) -> void { throw py::type_error("unsupported operand type(s) for -=: 'state_representation.JointTorques' and 'state_representation.JointAccelerations'"); });
  c.def(py::self -= JointState());
  c.def(py::self - py::self);
  c.def("__sub__", [](const JointTorques& self, const JointPositions& other) -> void { throw py::type_error("unsupported operand type(s) for -: 'state_representation.JointTorques' and 'state_representation.JointPositions'"); });
  c.def("__sub__", [](const JointTorques& self, const JointVelocities& other) -> void { throw py::type_error("unsupported operand type(s) for -: 'state_representation.JointTorques' and 'state_representation.JointVelocities'"); });
  c.def("__sub__", [](const JointTorques& self, const JointAccelerations& other) -> void { throw py::type_error("unsupported operand type(s) for -: 'state_representation.JointTorques' and 'state_representation.JointAccelerations'"); });
  c.def(py::self - JointState());

  c.def("copy", &JointTorques::copy, "Return a copy of the JointTorques");
  c.def("data", &JointTorques::data, "Returns the torques data as a vector");
  c.def("set_data", py::overload_cast<const Eigen::VectorXd&>(&JointTorques::set_data), "Set the torques data from a vector", "data"_a);
  c.def("set_data", py::overload_cast<const std::vector<double>&>(&JointTorques::set_data), "Set the torques data from a list", "data"_a);

  c.def("clamp", py::overload_cast<double, double>(&JointTorques::clamp), "Clamp inplace the magnitude of the torque to the values in argument", "max_absolute_value"_a, "noise_ratio"_a=0.0);
  c.def("clamped", py::overload_cast<double, double>(&JointTorques::clamp), "Return the torque clamped to the values in argument", "max_absolute_value"_a, "noise_ratio"_a=0.0);
  c.def("clamp", py::overload_cast<const Eigen::ArrayXd&, const Eigen::ArrayXd&>(&JointTorques::clamp), "Clamp inplace the magnitude of the torque to the values in argument", "max_absolute_value_array"_a, "noise_ratio_array"_a);
  c.def("clamped", py::overload_cast<const Eigen::ArrayXd&, const Eigen::ArrayXd&>(&JointTorques::clamp), "Return the torque clamped to the values in argument", "max_absolute_value_array"_a, "noise_ratio_array"_a);

  c.def("__copy__", [](const JointTorques &torques) {
    return JointTorques(torques);
  });
  c.def("__deepcopy__", [](const JointTorques &torques, py::dict) {
    return JointTorques(torques);
  }, "memo"_a);
  c.def("__repr__", [](const JointTorques& torques) {
    std::stringstream buffer;
    buffer << torques;
    return buffer.str();
  });
}

void bind_joint_space(py::module_& m) {
  joint_state_variable(m);
  joint_state(m);
  joint_positions(m);
  joint_velocities(m);
  joint_accelerations(m);
  joint_torques(m);
}