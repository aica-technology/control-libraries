#pragma once

#include <cassert>
#include <chrono>
#include <iostream>
#include <memory>
#include <typeinfo>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "state_representation/StateType.hpp"
#include "state_representation/MathTools.hpp"

/**
 * @namespace state_representation
 * @brief Core state variables and objects
 */
namespace state_representation {

/**
 * @class State
 * @brief Abstract class to represent a state
 */
class State : public std::enable_shared_from_this<State> {
public:
  /**
   * @brief Empty constructor
   */
  State();

  /**
   * @brief Constructor with name specification
   * @param name The name of the state
   */
  explicit State(const std::string& name);

  /**
   * @brief Copy constructor from another state
   */
  State(const State& state);

  /**
   * @brief Virtual destructor
   */
  virtual ~State() = default;

  /**
   * @brief Swap the values of the two states
   * @param state1 State to be swapped with 2
   * @param state2 State to be swapped with 1
   */
  friend void swap(State& state1, State& state2);

  /**
   * @brief Copy assignment operator that has to be defined to the custom assignment operator
   * @param state The state with value to assign
   * @return Reference to the current state with new values
   */
  State& operator=(const State& state);

  /**
   * @brief Getter of the type attribute
   */
  const StateType& get_type() const;

  /**
   * @brief Getter of the name attribute
   */
  const std::string& get_name() const;

  /**
   * @brief Getter of the empty attribute
   */
  bool is_empty() const;

  /**
   * @brief Getter of the timestamp attribute
   */
  const std::chrono::time_point<std::chrono::steady_clock>& get_timestamp() const;

  /**
   * @brief Setter of the name attribute
   */
  virtual void set_name(const std::string& name);

  /**
   * @brief Reset the timestamp attribute to now
   */
  void reset_timestamp();

  /**
   * @brief Set the data of the state from an Eigen vector
   */
  virtual void set_data(const Eigen::VectorXd& data);

  /**
   * @brief Set the data of the state from a std vector
   */
  virtual void set_data(const std::vector<double>& data);

  /**
   * @brief Set the data of the state from an Eigen matrix
   */
  virtual void set_data(const Eigen::MatrixXd& data);

  /**
   * @brief Get the age of the state, i.e. the time since the last modification
   */
  double get_age() const;

  /**
   * @brief Check if the state is incompatible for operations with the state given as argument
   * @param state The state to check compatibility with
   */
  virtual bool is_incompatible(const State& state) const;

  /**
   * @brief Check if the state is deprecated given a certain time delay
   * @param time_delay The time after which to consider the state as deprecated
   */
  bool is_deprecated(double time_delay) const;

  /**
   * @brief Check if the state is deprecated given a certain time delay
   * @param time_delay The time after which to consider the state as deprecated
   */
  template<typename DurationT>
  bool is_deprecated(const std::chrono::duration<int64_t, DurationT>& time_delay) const;

  /**
   * @brief Reset the object to a post-construction state
   */
  virtual void reset();

  /**
   * @brief Boolean operator for the truthiness of a state
   * @return False if the state is empty, true otherwise
   */
  explicit operator bool() const noexcept;

  /**
   * @brief Overload the ostream operator for printing
   * @param os The ostream to append the string representing the state to
   * @param state The state to print
   * @return The appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const State& state);

protected:
  /**
   * @brief Setter of the state type attribute
   */
  void set_type(const StateType& type);

  /**
   * @brief Setter of the empty attribute
   * @param empty Flag to specify if the state should be empty or not, default true
   */
  void set_empty(bool empty = true);

  /**
   * @brief Throw an exception if the state is empty
   * @throws exceptions::EmptyStateException
   */
  void assert_not_empty() const;

  /**
   * @brief Convert the state to its string representation
   */
  virtual std::string to_string() const;

private:
  StateType type_;                                              ///< type of the State
  std::string name_;                                            ///< name of the state
  bool empty_;                                                  ///< indicate if the state is empty
  std::chrono::time_point<std::chrono::steady_clock> timestamp_;///< time since last modification made to the state
};

inline void swap(State& state1, State& state2) {
  std::swap(state1.name_, state2.name_);
  std::swap(state1.empty_, state2.empty_);
  std::swap(state1.timestamp_, state2.timestamp_);
}

template<typename DurationT>
inline bool State::is_deprecated(const std::chrono::duration<int64_t, DurationT>& time_delay) const {
  return ((std::chrono::steady_clock::now() - this->timestamp_) > time_delay);
}

template<typename T>
std::shared_ptr<State> make_shared_state(const T& state) {
  return std::make_shared<T>(state);
}
}// namespace state_representation
