#include "state_representation_bindings.hpp"

#include <state_representation/parameters/ParameterType.hpp>
#include <state_representation/parameters/Parameter.hpp>
#include <state_representation/parameters/ParameterMap.hpp>

#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/joint/JointState.hpp>
#include <state_representation/space/joint/JointPositions.hpp>
#include <state_representation/geometry/Ellipsoid.hpp>

#include "parameter_container.hpp"
#include "py_parameter_map.hpp"

using namespace py_parameter;

void parameter_type(py::module_& m) {
  py::enum_<ParameterType>(m, "ParameterType")
      .value("INT", ParameterType::INT)
      .value("INT_ARRAY", ParameterType::INT_ARRAY)
      .value("DOUBLE", ParameterType::DOUBLE)
      .value("DOUBLE_ARRAY", ParameterType::DOUBLE_ARRAY)
      .value("BOOL", ParameterType::BOOL)
      .value("BOOL_ARRAY", ParameterType::BOOL_ARRAY)
      .value("STRING", ParameterType::STRING)
      .value("STRING_ARRAY", ParameterType::STRING_ARRAY)
      .value("STATE", ParameterType::STATE)
      .value("VECTOR", ParameterType::VECTOR)
      .value("MATRIX", ParameterType::MATRIX)
      .export_values();
}

void parameter_interface(py::module_& m) {
  py::class_<ParameterInterface, std::shared_ptr<ParameterInterface>, State> c(m, "ParameterInterface");

  c.def(py::init<const std::string&, const ParameterType&, const StateType&>(), "Constructor of a ParameterInterface with name, parameter type and parameter state type", "name"_a, "type"_a, "parameter_state_type"_a=StateType::NONE);
  c.def(py::init<const ParameterInterface&>(), "Copy constructor from another ParameterInterface", "parameter"_a);

  c.def("get_parameter_type", &ParameterInterface::get_parameter_type, "Get the parameter type.");
  c.def("get_parameter_state_type", &ParameterInterface::get_parameter_state_type, "Get the state type of the parameter.");
}

void parameter(py::module_& m) {
  py::class_<ParameterContainer, std::shared_ptr<ParameterContainer>, ParameterInterface> c(m, "Parameter");

  c.def(py::init<const std::string&, const ParameterType&, const StateType&>(), "Constructor of a parameter with name, parameter type and parameter state type", "name"_a, "type"_a, "parameter_state_type"_a=StateType::NONE);
  c.def(py::init<const std::string&, const py::object&, const ParameterType&, const StateType&>(), "Constructor of a parameter with name, value, parameter type and parameter state type", "name"_a, "value"_a, "type"_a, "parameter_state_type"_a=StateType::NONE);
  c.def(py::init<const ParameterContainer&>(), "Copy constructor from another Parameter", "parameter"_a);
  c.def(py::init([](const std::shared_ptr<ParameterInterface>& parameter) { return interface_ptr_to_container(parameter); }), "Constructor from a parameter interface pointer", "parameter"_a);

  c.def("get_value", &ParameterContainer::get_value, "Getter of the value attribute.");
  c.def("set_value", &ParameterContainer::set_value, "Setter of the value attribute.", py::arg("value"));

  c.def("__copy__", [](const ParameterContainer& parameter) {
    return ParameterContainer(parameter);
  });
  c.def("__deepcopy__", [](const ParameterContainer& parameter, py::dict) {
    return ParameterContainer(parameter);
  }, "memo"_a);
  c.def("__repr__", [](const ParameterContainer& parameter) {
    std::stringstream buffer;
    switch (parameter.get_parameter_type()) {
      case ParameterType::INT:
        buffer << container_to_parameter<int>(parameter);
        break;
      case ParameterType::INT_ARRAY:
        buffer << container_to_parameter<std::vector<int>>(parameter);
        break;
      case ParameterType::DOUBLE:
        buffer << container_to_parameter<double>(parameter);
        break;
      case ParameterType::DOUBLE_ARRAY:
        buffer << container_to_parameter<std::vector<double>>(parameter);
        break;
      case ParameterType::BOOL:
        buffer << container_to_parameter<bool>(parameter);
        break;
      case ParameterType::BOOL_ARRAY:
        buffer << container_to_parameter<std::vector<bool>>(parameter);
        break;
      case ParameterType::STRING:
        buffer << container_to_parameter<std::string>(parameter);
        break;
      case ParameterType::STRING_ARRAY:
        buffer << container_to_parameter<std::vector<std::string>>(parameter);
        break;
      case ParameterType::STATE: {
        try {
          switch (parameter.get_parameter_state_type()) {
            case StateType::CARTESIAN_STATE:
              buffer << container_to_parameter<CartesianState>(parameter);
              break;
            case StateType::CARTESIAN_POSE:
              buffer << container_to_parameter<CartesianPose>(parameter);
              break;
            case StateType::JOINT_STATE:
              buffer << container_to_parameter<JointState>(parameter);
              break;
            case StateType::JOINT_POSITIONS:
              buffer << container_to_parameter<JointPositions>(parameter);
              break;
            case StateType::GEOMETRY_ELLIPSOID:
              buffer << container_to_parameter<Ellipsoid>(parameter);
              break;
            default:
              buffer << "Parameter '" << parameter.get_name() << "' contains an unsupported state type" << std::endl;
              break;
          }
        } catch (const std::exception&) {
          buffer << "Parameter '" << parameter.get_name() << "' is invalid" << std::endl;
          break;
        }
        break;
      }
      case ParameterType::MATRIX:
        buffer << container_to_parameter<Eigen::MatrixXd>(parameter);
        break;
      case ParameterType::VECTOR:
        buffer << container_to_parameter<Eigen::VectorXd>(parameter);
        break;
      default:
        buffer << "Parameter '" << parameter.get_name() << "' has an invalid parameter type" << std::endl;
        break;
    }
    return buffer.str();
  });
}

void parameter_map(py::module_& m) {
  py::class_<ParameterMap, std::shared_ptr<ParameterMap>, PyParameterMap> c(m, "ParameterMap");

  c.def(py::init(), "Empty constructor");
  c.def(
      py::init([](const std::map<std::string, ParameterContainer>& parameters) {
        auto parameter_map = container_to_interface_ptr_map(parameters);
        return ParameterMap(parameter_map);
      }), "Construct the parameter map with an initial list of parameters", "parameters"_a
  );
  c.def(
      py::init([](const std::list<ParameterContainer>& parameters) {
        auto parameter_list = container_to_interface_ptr_list(parameters);
        return ParameterMap(parameter_list);
      }), "Construct the parameter map with an initial map of parameters", "parameters"_a);

  c.def(
      "get_parameter", [](ParameterMap& self, const std::string& name) -> ParameterContainer {
        return interface_ptr_to_container(self.get_parameter(name));
       } , "Get a parameter by its name", "name"_a
  );
  c.def(
      "get_parameters", [](ParameterMap& self) {
        return interface_ptr_to_container_map(self.get_parameters());
      } , "Get a map of all the <name, parameter> pairs"
  );
  c.def(
      "get_parameter_value", [](ParameterMap& self, const std::string& name) -> py::object {
        return interface_ptr_to_container(self.get_parameter(name)).get_value();
      }, "Get a parameter value by its name", "name"_a
  );
  c.def(
      "get_parameter_list", [](ParameterMap& self) {
        return interface_ptr_to_container_list(self.get_parameter_list());
      } , "Get a list of all the parameters"
  );

  c.def("set_parameter", [](ParameterMap& self, const ParameterContainer& parameter) {
    self.set_parameter(container_to_interface_ptr(parameter));
  }, "Set a parameter", "parameter"_a);
  c.def("set_parameters", [](ParameterMap& self, const std::list<ParameterContainer>& parameters) {
    self.set_parameters(container_to_interface_ptr_list(parameters));
  }, "Set parameters from a list of parameters", "parameters"_a);
  c.def("set_parameters", [](ParameterMap& self, const std::map<std::string, ParameterContainer>& parameters) {
    self.set_parameters(container_to_interface_ptr_map(parameters));
  }, "Set parameters from a map with <name, parameter> pairs", "parameters"_a);
  c.def(
      "set_parameter_value", [](ParameterMap& self, const std::string& name, const py::object& value, const ParameterType& type, const StateType& parameter_state_type) -> void {
        auto param = ParameterContainer(name, value, type, parameter_state_type);
        self.set_parameter(container_to_interface_ptr(param));
      }, "Set a parameter value by its name", "name"_a, "value"_a, "type"_a, "parameter_state_type"_a=StateType::NONE
  );
  c.def("remove_parameter", &ParameterMap::remove_parameter, "Remove a parameter from the parameter map.", "name"_a);
}

void bind_parameters(py::module_& m) {
  parameter_type(m);
  parameter_interface(m);
  parameter(m);
  parameter_map(m);
}