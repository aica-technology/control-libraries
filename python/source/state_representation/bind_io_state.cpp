#include "state_representation_bindings.hpp"

#include <state_representation/DigitalIOState.hpp>
#include <state_representation/AnalogIOState.hpp>


void digital_io_state(py::module_& m) {
  py::class_<DigitalIOState, std::shared_ptr<DigitalIOState>, State> c(m, "DigitalIOState");

  c.def(py::init(), "Empty constructor for an digital IO state");
  c.def(py::init<const std::string&, unsigned int>(), "Constructor with name and number of digital IOs provided", "name"_a, "nb_ios"_a=0);
  c.def(py::init<const std::string&, const std::vector<std::string>&>(), "onstructor with name and list of digital IO names provided", "name"_a, "io_names"_a);
  c.def(py::init<const DigitalIOState&>(), "Copy constructor of an digital IO state", "state"_a);

  c.def_static("Zero", py::overload_cast<const std::string&, unsigned int>(&DigitalIOState::Zero), "Constructor for a zero digital IO state", "name"_a, "nb_ios"_a);
  c.def_static("Zero", py::overload_cast<const std::string&, const std::vector<std::string>&>(&DigitalIOState::Zero), "Constructor for a zero digital IO state", "name"_a, "io_names"_a);
  c.def_static("Random", py::overload_cast<const std::string&, unsigned int>(&DigitalIOState::Random), "Constructor for a random digital IO state", "name"_a, "nb_ios"_a);
  c.def_static("Random", py::overload_cast<const std::string&, const std::vector<std::string>&>(&DigitalIOState::Random), "Constructor for a random digital IO state", "name"_a, "io_names"_a);
  
  c.def("get_size", &DigitalIOState::get_size, "Getter of the size from the attributes.");
  c.def("get_names", &DigitalIOState::get_names, "Getter of the names attribute.");
  c.def("get_joint_index", &DigitalIOState::get_io_index, "Get IO index by the name of the IO, if it exists", "io_name"_a);
  c.def("set_names", py::overload_cast<unsigned int>(&DigitalIOState::set_names), "Setter of the names from the number of IOs", "nb_ios"_a);
  c.def("set_names", py::overload_cast<const std::vector<std::string>&>(&DigitalIOState::set_names), "Setter of the names from a list of IO names", "names"_a);

  c.def("get_value", [](const DigitalIOState& state, const std::string& name) { return state.get_value(name); }, "Get the value of a digital IO by its name, if it exists", "name"_a);
  c.def("get_value", [](const DigitalIOState& state, unsigned int io_index) { return state.get_value(io_index); }, "Get the value of a digital IO by its index, if it exists", "io_index"_a);
  c.def("set_value", py::overload_cast<bool, const std::string&>(&DigitalIOState::set_value), "Set the value of a digital IO by its name", "value"_a, "name"_a);
  c.def("set_value", py::overload_cast<bool, unsigned int>(&DigitalIOState::set_value), "Set the value of a digital IO by its index", "value"_a, "io_index"_a);
  
  c.def("is_true", [](const DigitalIOState& state, const std::string& name) { return state.is_true(name); }, "Check if a digital IO is true by its name, if it exists", "name"_a);
  c.def("is_true", [](const DigitalIOState& state, unsigned int io_index) { return state.is_true(io_index); }, "Check if a digital IO is true by its index, if it exists", "io_index"_a);
  c.def("is_false", [](const DigitalIOState& state, const std::string& name) { return state.is_false(name); }, "Check if a digital IO is false by its name, if it exists", "name"_a);
  c.def("is_false", [](const DigitalIOState& state, unsigned int io_index) { return state.is_false(io_index); }, "Check if a digital IO is false by its index, if it exists", "io_index"_a);
  c.def("set_true", py::overload_cast<const std::string&>(&DigitalIOState::set_true), "Set the a digital IO to true by its name", "name"_a);
  c.def("set_true", py::overload_cast<unsigned int>(&DigitalIOState::set_true), "Set the a digital IO to true by its index", "io_index"_a);
  c.def("set_false", py::overload_cast<const std::string&>(&DigitalIOState::set_false), "Set the a digital IO to false by its name", "name"_a);
  c.def("set_false", py::overload_cast<unsigned int>(&DigitalIOState::set_false), "Set the a digital IO to false by its index", "io_index"_a);
  
  c.def("copy", &DigitalIOState::copy, "Return a copy of the digital IO state");
  c.def("set_false", py::overload_cast<>(&DigitalIOState::set_false), "Set all digital IOs false");
  c.def("data", &DigitalIOState::data, "Returns the values of the IO state as an Eigen vector");
  c.def("array", &DigitalIOState::array, "Returns the values of the IO state an Eigen array");
  c.def("set_data", py::overload_cast<const Eigen::Vector<bool, Eigen::Dynamic>&>(&DigitalIOState::set_data), "Set the values of the IO state from a single Eigen vector", "data"_a);
  c.def("set_data", py::overload_cast<const std::vector<bool>&>(&DigitalIOState::set_data), "Set the values of the IO state from a single list", "data"_a);

  c.def("to_list", &DigitalIOState::to_std_vector, "Return the IO values as a list");

  c.def("__copy__", [](const DigitalIOState &state) {
    return DigitalIOState(state);
  });
  c.def("__deepcopy__", [](const DigitalIOState &state, py::dict) {
    return DigitalIOState(state);
  }, "memo"_a);
  c.def("__repr__", [](const DigitalIOState& state) {
    std::stringstream buffer;
    buffer << state;
    return buffer.str();
  });
}

void analog_io_state(py::module_& m) {
  py::class_<AnalogIOState, std::shared_ptr<AnalogIOState>, State> c(m, "AnalogIOState");

  c.def(py::init(), "Empty constructor for an analog IO state");
  c.def(py::init<const std::string&, unsigned int>(), "Constructor with name and number of analog IOs provided", "name"_a, "nb_ios"_a=0);
  c.def(py::init<const std::string&, const std::vector<std::string>&>(), "onstructor with name and list of analog IO names provided", "name"_a, "io_names"_a);
  c.def(py::init<const AnalogIOState&>(), "Copy constructor of an analog IO state", "state"_a);

  c.def_static("Zero", py::overload_cast<const std::string&, unsigned int>(&AnalogIOState::Zero), "Constructor for a zero analog IO state", "name"_a, "nb_ios"_a);
  c.def_static("Zero", py::overload_cast<const std::string&, const std::vector<std::string>&>(&AnalogIOState::Zero), "Constructor for a zero analog IO state", "name"_a, "io_names"_a);
  c.def_static("Random", py::overload_cast<const std::string&, unsigned int>(&AnalogIOState::Random), "Constructor for a random analog IO state", "name"_a, "nb_ios"_a);
  c.def_static("Random", py::overload_cast<const std::string&, const std::vector<std::string>&>(&AnalogIOState::Random), "Constructor for a random analog IO state", "name"_a, "io_names"_a);
  
  c.def("get_size", &AnalogIOState::get_size, "Getter of the size from the attributes.");
  c.def("get_names", &AnalogIOState::get_names, "Getter of the names attribute.");
  c.def("get_joint_index", &AnalogIOState::get_io_index, "Get IO index by the name of the IO, if it exists", "io_name"_a);
  c.def("set_names", py::overload_cast<unsigned int>(&AnalogIOState::set_names), "Setter of the names from the number of IOs", "nb_ios"_a);
  c.def("set_names", py::overload_cast<const std::vector<std::string>&>(&AnalogIOState::set_names), "Setter of the names from a list of IO names", "names"_a);

  c.def("get_value", [](const AnalogIOState& state, const std::string& name) { return state.get_value(name); }, "Get the value of an analog IO by its name, if it exists", "name"_a);
  c.def("get_value", [](const AnalogIOState& state, unsigned int io_index) { return state.get_value(io_index); }, "Get the value of an analog IO by its index, if it exists", "io_index"_a);
  c.def("set_value", py::overload_cast<double, const std::string&>(&AnalogIOState::set_value), "Set the value of an analog IO by its name", "value"_a, "name"_a);
  c.def("set_value", py::overload_cast<double, unsigned int>(&AnalogIOState::set_value), "Set the value of an analog IO by its index", "value"_a, "io_index"_a);
  
  c.def("copy", &AnalogIOState::copy, "Return a copy of the analog IO state");
  c.def("set_zero", &AnalogIOState::set_zero, "Set the analog IO state to zero data");
  c.def("data", &AnalogIOState::data, "Returns the values of the IO state as an Eigen vector");
  c.def("array", &AnalogIOState::array, "Returns the values of the IO state an Eigen array");
  c.def("set_data", py::overload_cast<const Eigen::VectorXd&>(&AnalogIOState::set_data), "Set the values of the IO state from a single Eigen vector", "data"_a);
  c.def("set_data", py::overload_cast<const std::vector<double>&>(&AnalogIOState::set_data), "Set the values of the IO state from a single list", "data"_a);

  c.def("to_list", &AnalogIOState::to_std_vector, "Return the IO values as a list");

  c.def("__copy__", [](const AnalogIOState &state) {
    return AnalogIOState(state);
  });
  c.def("__deepcopy__", [](const AnalogIOState &state, py::dict) {
    return AnalogIOState(state);
  }, "memo"_a);
  c.def("__repr__", [](const AnalogIOState& state) {
    std::stringstream buffer;
    buffer << state;
    return buffer.str();
  });
}

void bind_io_state(py::module_& m) {
  digital_io_state(m);
  analog_io_state(m);
}