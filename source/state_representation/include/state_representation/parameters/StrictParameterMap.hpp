#pragma once

#include "state_representation/parameters/ParameterMap.hpp"

namespace state_representation {

/**
 * @class StrictParameterMap
 * @brief A strict version of ParameterMap that enforces type safety for parameters.
 */
class StrictParameterMap : public ParameterMap {
public:
  /**
   * @brief Empty constructor
   */
  StrictParameterMap() = default;

  /**
   * @brief Construct the parameter map with an initial list of parameters
   * @param parameters A list of Parameter pointers
   */
  explicit StrictParameterMap(const ParameterInterfaceList& parameters);

  /**
   * @brief Construct the parameter map with an initial map of parameters
   * @param parameters A map of Parameter pointers
   */
  explicit StrictParameterMap(const ParameterInterfaceMap& parameters);

protected:
  /**
   * @brief Validate and set a parameter in the map.
   * @param parameter The parameter to be validated
   */
  void validate_and_set_parameter(const std::shared_ptr<ParameterInterface>& parameter) override;
};

}// namespace state_representation
