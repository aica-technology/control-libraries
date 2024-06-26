#pragma once

#include "state_representation_bindings.hpp"

#include <state_representation/State.hpp>
#include <state_representation/geometry/Ellipsoid.hpp>
#include <state_representation/parameters/Parameter.hpp>

namespace py_parameter {

struct ParameterValues {
  int int_value = int();
  std::vector<int> int_array_value = std::vector<int>();
  double double_value = double();
  std::vector<double> double_array_value = std::vector<double>();
  bool bool_value = bool();
  std::vector<bool> bool_array_value= std::vector<bool>();
  std::string string_value = std::string();
  std::vector<std::string> string_array_value = std::vector<std::string>();
  std::shared_ptr<State> state_pointer;
  Eigen::MatrixXd matrix_value = Eigen::MatrixXd();
  Eigen::VectorXd vector_value = Eigen::VectorXd();
};

class ParameterContainer : public ParameterInterface {
public:
  ParameterContainer(
      const std::string& name, const ParameterType& type, const StateType& parameter_state_type = StateType::NONE
  );
  ParameterContainer(
      const std::string& name, const py::object& value, const ParameterType& type,
      const StateType& parameter_state_type = StateType::NONE
  );
  ParameterContainer(const ParameterContainer& parameter);

  void set_value(py::object value);

  py::object get_value() const;

  void reset();

  ParameterValues values;
};

ParameterContainer interface_ptr_to_container(const std::shared_ptr<ParameterInterface>& parameter);

std::shared_ptr<ParameterInterface> container_to_interface_ptr(const ParameterContainer& parameter);

template<typename T>
inline Parameter<T> container_to_parameter(const ParameterContainer& container) {
  if (container.is_empty()) {
    return Parameter<T>(container.get_name());
  } else {
    return *container_to_interface_ptr(container)->get_parameter<T>();
  }
}

std::map<std::string, ParameterContainer>
interface_ptr_to_container_map(const std::map<std::string, std::shared_ptr<ParameterInterface>>& parameters);

std::map<std::string, std::shared_ptr<ParameterInterface>>
container_to_interface_ptr_map(const std::map<std::string, ParameterContainer>& parameters);

std::list<ParameterContainer>
interface_ptr_to_container_list(const std::list<std::shared_ptr<ParameterInterface>>& parameters);

std::list<std::shared_ptr<ParameterInterface>>
container_to_interface_ptr_list(const std::list<ParameterContainer>& parameters);

}// namespace py_parameter