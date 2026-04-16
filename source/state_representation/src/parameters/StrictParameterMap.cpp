#include "state_representation/parameters/StrictParameterMap.hpp"

namespace state_representation {

StrictParameterMap::StrictParameterMap(const ParameterInterfaceList& parameters) : ParameterMap(parameters) {}

StrictParameterMap::StrictParameterMap(const ParameterInterfaceMap& parameters) : ParameterMap(parameters) {}

void StrictParameterMap::validate_and_set_parameter(const std::shared_ptr<ParameterInterface>& parameter) {
  if (auto param_it = this->parameters_.find(parameter->get_name()); param_it != this->parameters_.cend()) {
    try {
      copy_parameter_value(parameter, param_it->second);
      return;
    } catch (const std::exception& ex) {
      throw exceptions::InvalidParameterException(ex.what());
    }
  }
  this->parameters_.insert({parameter->get_name(), parameter});
}

}// namespace state_representation
