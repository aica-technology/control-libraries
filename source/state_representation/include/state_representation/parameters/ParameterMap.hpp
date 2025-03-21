#pragma once

#include <list>
#include <map>
#include <memory>

#include "state_representation/exceptions/InvalidParameterException.hpp"
#include "state_representation/parameters/Parameter.hpp"

namespace state_representation {

typedef std::list<std::shared_ptr<ParameterInterface>> ParameterInterfaceList;
typedef std::map<std::string, std::shared_ptr<ParameterInterface>> ParameterInterfaceMap;

/**
 * @class ParameterMap
 * @brief A wrapper class to contain a map of Parameter pointers by name and provide robust access methods
 */
class ParameterMap {
public:
  /**
   * @brief Empty constructor
   */
  ParameterMap() = default;

  /**
   * @brief Construct the parameter map with an initial list of parameters
   * @param parameters A list of Parameter pointers
   */
  explicit ParameterMap(const ParameterInterfaceList& parameters);

  /**
   * @brief Construct the parameter map with an initial map of parameters
   * @param parameters A amp of Parameter pointers
   */
  explicit ParameterMap(const ParameterInterfaceMap& parameters);

  /**
   * @brief Get a parameter by its name.
   * @param name The name of the parameter
   * @return The parameter, if it exists
   */
  [[nodiscard]] std::shared_ptr<ParameterInterface> get_parameter(const std::string& name) const;

  /**
   * @brief Get a map of all the <name, parameter> pairs.
   * @return The map of parameters
   */
  [[nodiscard]] ParameterInterfaceMap get_parameters() const;

  /**
   * @brief Get a parameter value by its name.
   * @tparam T Type of the parameter value
   * @param name The name of the parameter
   * @return The value of the parameter, if the parameter exists
   */
  template<typename T>
  [[nodiscard]] T get_parameter_value(const std::string& name) const;

  /**
   * @brief Get a list of all the parameters.
   * @return The list of parameters
   */
  [[nodiscard]] ParameterInterfaceList get_parameter_list() const;

  /**
   * @brief Set a parameter.
   * @param parameter The new parameter
   */
  void set_parameter(const std::shared_ptr<ParameterInterface>& parameter);

  /**
   * @brief Set parameters from a list of parameters.
   * @param parameters The list of parameters
   */
  void set_parameters(const ParameterInterfaceList& parameters);

  /**
   * @brief Set parameters from a map with <name, parameter> pairs.
   * @param parameters The map of parameters
   */
  void set_parameters(const ParameterInterfaceMap& parameters);

  /**
   * @brief Set a parameter value by its name.
   * @tparam T Type of the parameter value
   * @param name The name of the parameter
   * @param value The new value of the parameter
   */
  template<typename T>
  void set_parameter_value(const std::string& name, const T& value);

  /**
   * @brief Remove a parameter from the parameter map.
   * @param name The name of the parameter that should be removed
   * @raise InvalidParameterException if the parameter does not exist
   */
  void remove_parameter(const std::string& name);

protected:
  /**
   * @brief Validate and set a parameter in the map.
   * @details This virtual function should be redefined in any base class to provide proper validation of parameters
   * by name, value and type. In the base form, it allows any type of parameter name to be added to the map.
   * @param parameter The parameter to be validated
   */
  virtual void validate_and_set_parameter(const std::shared_ptr<ParameterInterface>& parameter);

  /**
   * @brief Check if a parameter exists and has the expected type, throw an exception otherwise.
   * @param parameter The parameter to be validated
   * @throws InvalidParameterException if the parameter doesn't exist or has unexpected type
   */
  void assert_parameter_valid(const std::shared_ptr<ParameterInterface>& parameter);

  ParameterInterfaceMap parameters_;///< map of parameters by name
};

template<typename T>
inline T ParameterMap::get_parameter_value(const std::string& name) const {
  return this->get_parameter(name)->get_parameter_value<T>();
}

template<typename T>
inline void ParameterMap::set_parameter_value(const std::string& name, const T& value) {
  this->validate_and_set_parameter(make_shared_parameter<T>(name, value));
}

}// namespace state_representation
