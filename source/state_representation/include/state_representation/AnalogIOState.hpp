#pragma once

#include "state_representation/IOState.hpp"

namespace state_representation {

class AnalogIOState : public IOState<double> {
public:
  /**
   * @brief Empty constructor for an analog IO state
   */
  AnalogIOState();

  /**
   * @brief Constructor with name and number of analog IOs provided
   * @param name The name of the associated IO state
   * @param nb_ios The number of IOs for initialization
   */
  explicit AnalogIOState(const std::string& name, unsigned int nb_ios = 0);

  /**
   * @brief Constructor with name and list of analog IO names provided
   * @param name The name of the associated analog IO state
   * @param io_name List of IO names
   */
  AnalogIOState(const std::string& name, const std::vector<std::string>& io_names);

  /**
   * @brief Copy constructor of an analog IO state
   * @param state The analog IO state to copy from
   */
  AnalogIOState(const AnalogIOState& state);

  /**
   * @brief Constructor for a zero analog IO state
   * @param name The name of the associated analog IO state
   * @param nb_ios The number of analog IOs for initialization
   * @return Analog IO state with zero data
   */
  static AnalogIOState Zero(const std::string& name, unsigned int nb_ios);

  /**
   * @brief Constructor for a zero analog IO state
   * @param name The name of the associated analog IO state
   * @param names List of IO names
   * @return Analog IO state with zero data
   */
  static AnalogIOState Zero(const std::string& name, const std::vector<std::string>& names);

  /**
   * @brief Constructor for an analog IO state with random data
   * @param name The name of the associated analog IO state
   * @param nb_ios The number of analog IOs for initialization
   * @return Analog IO state with random data
   */
  static AnalogIOState Random(const std::string& name, unsigned int nb_ios);

  /**
   * @brief Constructor for an analog IO state with random data
   * @param name The name of the associated analog IO state
   * @param names List of IO names
   * @return Analog IO state with random data
   */
  static AnalogIOState Random(const std::string& name, const std::vector<std::string>& names);

  /**
   * @brief Copy assignment operator that has to be defined to the custom assignment operator
   * @param state The state with value to assign
   * @return Reference to the current state with new values
   */
  AnalogIOState& operator=(const AnalogIOState& state);

  /**
   * @brief Return a copy of the analog IO state
   */
  AnalogIOState copy() const;

  /**
   * @copybrief State::reset
   */
  void reset() override;

  /**
   * @brief Set the analog IO state to zero data
   */
  void set_zero();

  /**
   * @brief Overload the ostream operator for printing
   * @param os The ostream to append the string representing the state to
   * @param state The spatial state to print
   * @return The appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const AnalogIOState& state);

protected:
  /**
   * @brief Swap the values of the IO states
   * @param state1 IO state to be swapped with 2
   * @param state2 IO state to be swapped with 1
   */
  friend void swap(AnalogIOState& state1, AnalogIOState& state2);

  /**
   * @copydoc State::to_string
   */
  std::string to_string() const override;
};

inline void swap(AnalogIOState& state1, AnalogIOState& state2) {
  swap(static_cast<IOState<double>&>(state1), static_cast<IOState<double>&>(state2));
}
}// namespace state_representation
