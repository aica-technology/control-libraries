#include "state_representation_bindings.h"

#include <state_representation/State.hpp>


void state_type(py::module_& m) {
  py::enum_<StateType>(m, "StateType")
      .value("NONE", StateType::NONE)
      .value("STATE", StateType::STATE)
      .value("SPATIAL_STATE", StateType::SPATIAL_STATE)
      .value("CARTESIAN_STATE", StateType::CARTESIAN_STATE)
      .value("CARTESIAN_POSE", StateType::CARTESIAN_POSE)
      .value("CARTESIAN_TWIST", StateType::CARTESIAN_TWIST)
      .value("CARTESIAN_ACCELERATION", StateType::CARTESIAN_ACCELERATION)
      .value("CARTESIAN_WRENCH", StateType::CARTESIAN_WRENCH)
      .value("JOINT_STATE", StateType::JOINT_STATE)
      .value("JOINT_POSITIONS", StateType::JOINT_POSITIONS)
      .value("JOINT_VELOCITIES", StateType::JOINT_VELOCITIES)
      .value("JOINT_ACCELERATIONS", StateType::JOINT_ACCELERATIONS)
      .value("JOINT_TORQUES", StateType::JOINT_TORQUES)
      .value("JACOBIAN", StateType::JACOBIAN)
      .value("PARAMETER", StateType::PARAMETER)
      .value("GEOMETRY_SHAPE", StateType::GEOMETRY_SHAPE)
      .value("GEOMETRY_ELLIPSOID", StateType::GEOMETRY_ELLIPSOID)
      .value("TRAJECTORY", StateType::TRAJECTORY)
      .export_values();
}

void state(py::module_& m) {
  py::class_<State, std::shared_ptr<State>> c(m, "State");
  c.def_property_readonly_static("__array_priority__", [](py::object) { return 10000; });

  c.def(py::init(), "Empty constructor");
  c.def(py::init<const std::string&>(), "Constructor with name specification", "name"_a);
  c.def(py::init<const State&>(), "Copy constructor from another State", "state"_a);

  c.def("get_type", &State::get_type, "Getter of the type attribute");
  c.def("is_empty", &State::is_empty, "Getter of the empty attribute");
  c.def("get_age", &State::get_age, "Get the age of the state, i.e. the time since last modification");
  c.def("get_timestamp", &State::get_timestamp, "Getter of the timestamp attribute");
  c.def("reset_timestamp", &State::reset_timestamp, "Reset the timestamp attribute to the current time");
  c.def("get_name", &State::get_name, "Getter of the name");
  c.def("set_name", &State::set_name, "Setter of the name");

  c.def("is_deprecated", &State::is_deprecated<std::micro>, "Check if the state is deprecated given a certain time delay with microsecond precision");

  c.def("is_incompatible", &State::is_incompatible, "Check if the state is compatible for operations with the state given as argument", "state"_a);
  c.def("initialize", &State::initialize, "Initialize the State to a zero value");

  c.def("__copy__", [](const State &state) {
    return State(state);
  });
  c.def("__deepcopy__", [](const State &state, py::dict) {
    return State(state);
  }, "memo"_a);
  c.def("__repr__", [](const State& state) {
    std::stringstream buffer;
    buffer << state;
    return buffer.str();
  });
  c.def("__bool__", [](const State& state) {
    return bool(state);
  }, py::is_operator());
}

void bind_state(py::module_& m) {
  state_type(m);
  state(m);
}