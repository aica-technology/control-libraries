#pragma once

#include <memory>

#include "state_representation/parameters/ParameterInterface.hpp"
#include "state_representation/geometry/Ellipsoid.hpp"
#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/space/joint/JointPositions.hpp"

namespace state_representation {

/**
 * @class Parameter
 * @brief Class to represent name-value pairs of different types
 * @tparam T The type of the value stored in the parameter
 */
template<typename T>
class Parameter : public ParameterInterface {
public:
  /**
   * @brief Constructor with the name of the parameter
   * @param name The name of the parameter (default is empty)
   */
  explicit Parameter(const std::string& name = "");

  /**
   * @brief Constructor with a name and a value
   * @param name The name of the parameter
   * @param value The value of the parameter
   */
  Parameter(const std::string& name, const T& value);

  /**
   * @brief Copy constructor
   * @param parameter The parameter to copy
   */
  template<typename U>
  explicit Parameter(const Parameter<U>& parameter);

  /**
   * @brief Default virtual destructor
   */
  virtual ~Parameter() = default;

  /**
   * @brief Conversion equality
   */
  template<typename U>
  Parameter<T>& operator=(const Parameter<U>& parameter);

  /**
   * @brief Getter of the value attribute
   * @tparam U The expected type of the parameter
   * @return The value attribute
   */
  template<typename U>
  U get_value() const;

  /**
   * @brief Getter of the value attribute
   * @return The value attribute
   */
  const T& get_value() const;

  /**
   * @brief Getter of the value attribute
   * @return The value attribute
   */
  T& get_value();

  /**
   * @brief Setter of the value attribute
   * @param value The new value attribute
   */
  virtual void set_value(const T& value);

  /**
   * @copybrief State::reset
   */
  void reset() override;

  /**
   * @brief Overload the ostream operator for printing
   * @param os The ostream to append the string representing the state to
   * @param parameter The parameter to print
   * @return The appended ostream
   */
  template<typename U>
  friend std::ostream& operator<<(std::ostream& os, const Parameter<U>& parameter);

private:
  T value_;///< Value of the parameter
};

template<typename T>
template<typename U>
Parameter<T>::Parameter(const Parameter<U>& parameter) : Parameter<T>(parameter.get_name()) {
  if (parameter) {
    this->set_value(static_cast<T>(parameter.get_value()));
  }
}

template<typename T>
template<typename U>
Parameter<T>& Parameter<T>::operator=(const Parameter<U>& parameter) {
  Parameter<T> temp(parameter);
  *this = temp;
  return *this;
}

template<typename T>
template<typename U>
inline U Parameter<T>::get_value() const {
  this->assert_not_empty();
  return static_cast<U>(this->value_);
}

template<typename T>
inline const T& Parameter<T>::get_value() const {
  this->assert_not_empty();
  return this->value_;
}

template<typename T>
inline T& Parameter<T>::get_value() {
  this->assert_not_empty();
  return this->value_;
}

template<typename T>
inline void Parameter<T>::set_value(const T& value) {
  this->value_ = value;
  this->set_empty(false);
}

template<typename T>
inline void Parameter<T>::reset() {
  this->State::reset();
  this->value_ = T();
}

/**
 * @brief Create a Parameter object that is owned by a shared_ptr
 * @tparam T The type of the value stored in the parameter
 * @param name The name of the parameter
 * @param param_value The value of the parameter
 * @return A shared_ptr that owns the newly created Parameter object
 */
template<typename T>
static std::shared_ptr<Parameter<T>> make_shared_parameter(const std::string& name, const T& param_value) {
  return std::make_shared<Parameter<T>>(name, param_value);
}

/**
 * @brief Encapsulate a parameter in a ParameterInterface pointer
 * @param name The name of the parameter
 * @param type The type of the parameter
 * @param parameter_state_type The state type of the parameter, if applicable
 * @return A ParameterInterface pointer holding a reference to a Parameter object
 */
[[maybe_unused]] static std::shared_ptr<ParameterInterface> make_shared_parameter_interface(
    const std::string& name, const ParameterType& type, const StateType& parameter_state_type = StateType::NONE
) {
  switch (type) {
    case ParameterType::BOOL:
      return std::make_shared<Parameter<bool>>(name);
    case ParameterType::BOOL_ARRAY:
      return std::make_shared<Parameter<std::vector<bool>>>(name);
    case ParameterType::INT:
      return std::make_shared<Parameter<int>>(name);
    case ParameterType::INT_ARRAY:
      return std::make_shared<Parameter<std::vector<int>>>(name);
    case ParameterType::DOUBLE:
      return std::make_shared<Parameter<double>>(name);
    case ParameterType::DOUBLE_ARRAY:
      return std::make_shared<Parameter<std::vector<double>>>(name);
    case ParameterType::STRING:
      return std::make_shared<Parameter<std::string>>(name);
    case ParameterType::STRING_ARRAY:
      return std::make_shared<Parameter<std::vector<std::string>>>(name);
    case ParameterType::STATE: {
      switch (parameter_state_type) {
        case StateType::CARTESIAN_STATE:
          return std::make_shared<Parameter<CartesianState>>(name);
        case StateType::CARTESIAN_POSE:
          return std::make_shared<Parameter<CartesianPose>>(name);
        case StateType::JOINT_STATE:
          return std::make_shared<Parameter<JointState>>(name);
        case StateType::JOINT_POSITIONS:
          return std::make_shared<Parameter<JointPositions>>(name);
        case StateType::GEOMETRY_ELLIPSOID:
          return std::make_shared<Parameter<Ellipsoid>>(name);
        case StateType::NONE:
          throw exceptions::InvalidParameterException("No StateType provided.");
        default:
          throw exceptions::InvalidParameterException("This StateType is not supported for parameters.");
      }
    }
    case ParameterType::VECTOR:
      return std::make_shared<Parameter<Eigen::VectorXd>>(name);
    case ParameterType::MATRIX:
      return std::make_shared<Parameter<Eigen::MatrixXd>>(name);
    default:
      throw exceptions::InvalidParameterException("This ParameterType is not supported for parameters.");
  }
}
}// namespace state_representation
