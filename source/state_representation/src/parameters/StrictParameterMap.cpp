#include "state_representation/parameters/StrictParameterMap.hpp"

namespace state_representation {

StrictParameterMap::StrictParameterMap(const ParameterInterfaceList& parameters) : ParameterMap(parameters) {}

StrictParameterMap::StrictParameterMap(const ParameterInterfaceMap& parameters) : ParameterMap(parameters) {}
void StrictParameterMap::validate_and_set_parameter(const std::shared_ptr<ParameterInterface>& parameter) {
  if (this->parameters_.find(parameter->get_name()) != this->parameters_.cend()) {
    this->assert_parameter_valid(parameter);
  }
  this->parameters_.insert_or_assign(parameter->get_name(), parameter);
}

}// namespace state_representation
