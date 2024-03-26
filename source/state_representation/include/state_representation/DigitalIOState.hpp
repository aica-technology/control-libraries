#pragma once

#include "state_representation/IOState.hpp"

namespace state_representation {

class DigitalIOState : public IOState<bool> {
public:
  /**
   * @brief Empty constructor for a digital IO state
   */
  DigitalIOState();

  /**
   * @brief Constructor with name and number of digital IOs provided
   * @param name The name of the associated IO state
   * @param nb_ios The number of IOs for initialization
   */
  explicit DigitalIOState(const std::string& name, unsigned int nb_ios = 0);

  /**
   * @brief Constructor with name and list of digital IO names provided
   * @param name The name of the associated digital IO state
   * @param io_name List of IO names
   */
  DigitalIOState(const std::string& name, const std::vector<std::string>& io_names);

  /**
   * @brief Copy constructor of a digital IO state
   * @param state The digital IO state to copy from
   */
  DigitalIOState(const DigitalIOState& state);

  /**
   * @brief Constructor for a zero digital IO state
   * @param name The name of the associated digital IO state
   * @param nb_ios The number of digital IOs for initialization
   * @return Digital IO state with zero data
   */
  static DigitalIOState Zero(const std::string& name, unsigned int nb_ios);

  /**
   * @brief Constructor for a zero digital IO state
   * @param name The name of the associated digital IO state
   * @param names List of IO names
   * @return Digital IO state with zero data
   */
  static DigitalIOState Zero(const std::string& name, const std::vector<std::string>& names);

  /**
   * @brief Constructor for a digital IO state with random data
   * @param name The name of the associated digital IO state
   * @param nb_ios The number of digital IOs for initialization
   * @return Digital IO state with random data
   */
  static DigitalIOState Random(const std::string& name, unsigned int nb_ios);

  /**
   * @brief Constructor for a digital IO state with random data
   * @param name The name of the associated digital IO state
   * @param names List of IO names
   * @return Digital IO state with random data
   */
  static DigitalIOState Random(const std::string& name, const std::vector<std::string>& names);

  /**
   * @brief Copy assignment operator that has to be defined to the custom assignment operator
   * @param state The state with value to assign
   * @return Reference to the current state with new values
   */
  DigitalIOState& operator=(const DigitalIOState& state);

  /**
   * @brief Check if a digital IO is true by its name, if it exists
   * @param name The name of the IO
   * @throws IONotFoundException if the desired IO doesn't exist
   * @return The value of the IO, if it exists
   */
  bool is_true(const std::string& name) const;

  /**
   * @brief Check if a digital IO is true by its index, if it exists
   * @param io_index The index of the IO
   * @throws IONotFoundException if the desired IO doesn't exist
   * @return The value of the IO, if it exists
   */
  bool is_true(unsigned int io_index) const;

  /**
   * @brief Check if a digital IO is false by its name, if it exists
   * @param name The name of the IO
   * @throws IONotFoundException if the desired IO doesn't exist
   * @return The value of the IO, if it exists
   */
  bool is_false(const std::string& name) const;

  /**
   * @brief Check if a digital IO is false by its index, if it exists
   * @param io_index The index of the IO
   * @throws IONotFoundException if the desired IO doesn't exist
   * @return The value of the IO, if it exists
   */
  bool is_false(unsigned int io_index) const;

  /**
   * @brief Set the a digital IO to true by its name
   * @param name The name of the IO
   * @throws IONotFoundException if the desired IO doesn't exist
   */
  void set_true(const std::string& name);

  /**
   * @brief Set the a digital IO to true by its index
   * @param io_index The index of the IO
   * @throws IONotFoundException if the desired IO doesn't exist
   */
  void set_true(unsigned int io_index);

  /**
   * @brief Set the a digital IO to false by its name
   * @param name The name of the IO
   * @throws IONotFoundException if the desired IO doesn't exist
   */
  void set_false(const std::string& name);

  /**
   * @brief Set the a digital IO to false by its index
   * @param io_index The index of the IO
   * @throws IONotFoundException if the desired IO doesn't exist
   */
  void set_false(unsigned int io_index);

  /**
   * @brief Return a copy of the digital IO state
   */
  DigitalIOState copy() const;

  /**
   * @copybrief State::reset
   */
  void reset() override;

  /**
   * @brief Set all digital IOs false
   */
  void set_false();

  /**
   * @brief Overload the ostream operator for printing
   * @param os The ostream to append the string representing the state to
   * @param state The spatial state to print
   * @return The appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const DigitalIOState& state);

protected:
  /**
   * @brief Swap the values of the IO states
   * @param state1 IO state to be swapped with 2
   * @param state2 IO state to be swapped with 1
   */
  friend void swap(DigitalIOState& state1, DigitalIOState& state2);

  /**
   * @copydoc State::to_string
   */
  std::string to_string() const override;
};

inline void swap(DigitalIOState& state1, DigitalIOState& state2) {
  swap(static_cast<IOState<bool>&>(state1), static_cast<IOState<bool>&>(state2));
}

}// namespace state_representation
