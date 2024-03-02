#pragma once

#include "state_representation/State.hpp"

#include "state_representation/exceptions/IONotFoundException.hpp"
#include "state_representation/exceptions/IncompatibleSizeException.hpp"
#include "state_representation/exceptions/InvalidCastException.hpp"

namespace state_representation {

template<typename T>
class IOState : public State {
public:
  /**
   * @brief Getter of the size
   */
  unsigned int get_size() const;

  /**
   * @brief Getter of the names
   * @return The vector of strings containing the IO names
   */
  const std::vector<std::string>& get_names() const;

  /**
   * @brief Get IO index by the name of the IO, if it exists
   * @param io_name The name of the desired IO
   * @throws IONotFoundException if the desired IO doesn't exist
   * @return The index of the IO, if it exists
   */
  unsigned int get_io_index(const std::string& io_name) const;

  /**
   * @brief Returns the values of the IO state as an Eigen vector
   */
  Eigen::Vector<T, Eigen::Dynamic> data() const;

  /**
   * @brief Returns the values of the IO state an Eigen array
   */
  Eigen::Array<T, Eigen::Dynamic, 1> array() const;

  /**
   * @brief Setter of the names from the number of IOs
   * @param nb_ios The number of IOs of the IO state
   */
  void set_names(unsigned int nb_ios);

  /**
   * @brief Setter of the names from a list of IO names
   * @param names The vector of strings containing the IO names
   */
  void set_names(const std::vector<std::string>& names);

  /**
   * @brief Set the values of the IO state from a single Eigen vector
   * @param The data vector
   */
  void set_data(const Eigen::Vector<T, Eigen::Dynamic>& data);

  /**
   * @brief Set the values of the IO state from a single std vector
   * @param The data vector
   */
  void set_data(const std::vector<T>& data);

  /**
   * @brief Check if the IO group is incompatible for operations with the state given as argument
   * @param state The state to check compatibility with
   */
  bool is_incompatible(const State& state) const override;

  /**
   * @brief Return the IO values as a std vector
   * @return The IO values as a std vector
   */
  std::vector<T> to_std_vector() const;

protected:
  /**
   * @brief Empty constructor for an IO state
   */
  IOState() = default;

  /**
   * @brief Constructor with name and number of IOs provided
   * @param name The name of the associated IO state
   * @param nb_ios The number of IOs for initialization
   */
  explicit IOState(const std::string& name, unsigned int nb_ios);

  /**
   * @brief Constructor with name and list of IO names provided
   * @param name The name of the associated IO state
   * @param io_name List of IO names
   */
  IOState(const std::string& name, const std::vector<std::string>& io_names);

  /**
   * @brief Swap the values of the IO states
   * @param state1 IO state to be swapped with 2
   * @param state2 IO state to be swapped with 1
   */
  friend void swap(IOState& state1, IOState& state2) {
    swap(static_cast<State&>(state1), static_cast<State&>(state2));
    std::swap(state1.names_, state2.names_);
    std::swap(state1.data_, state2.data_);
  }

  static void assert_index_in_range(unsigned int io_index, unsigned int size);

  std::vector<std::string> names_;///< names of the IOs
  Eigen::Vector<T, Eigen::Dynamic> data_;///< IO values
};

template<typename T>
IOState<T>::IOState(const std::string& name, unsigned int nb_ios) :
    State(name),
    names_(nb_ios) {
  this->set_names(nb_ios);
}

template<typename T>
IOState<T>::IOState(const std::string& name, const std::vector<std::string>& io_names) :
  IOState<T>(name, io_names.size()) {
  this->set_names(io_names);
}

template<typename T>
unsigned int IOState<T>::get_size() const {
  return this->names_.size();
}

template<typename T>
const std::vector<std::string>& IOState<T>::get_names() const {
  return this->names_;
}

template<typename T>
unsigned int IOState<T>::get_io_index(const std::string& io_name) const {
  auto finder = std::find(this->names_.begin(), this->names_.end(), io_name);
  if (finder == this->names_.end()) {
    throw exceptions::IONotFoundException("The IO with name '" + io_name + "' could not be found in the IO state.");
  }
  return std::distance(this->names_.begin(), finder);
}

template<typename T>
Eigen::Vector<T, Eigen::Dynamic> IOState<T>::data() const {
  this->assert_not_empty();
  return this->data_;
}

template<typename T>
Eigen::Array<T, Eigen::Dynamic, 1> IOState<T>::array() const {
  this->assert_not_empty();
  return this->data_.array();
}
template<typename T>
void IOState<T>::set_names(unsigned int nb_ios) {
  if (this->get_size() != nb_ios) {
    throw exceptions::IncompatibleSizeException(
        "Input number of IOs is of incorrect size, expected " + std::to_string(this->get_size()) + " got "
            + std::to_string(nb_ios));
  }
  for (unsigned int i = 0; i < nb_ios; ++i) {
    this->names_[i] = "io" + std::to_string(i);
  }
  this->reset_timestamp();
}

template<typename T>
void IOState<T>::set_names(const std::vector<std::string>& names) {
  if (this->get_size() != names.size()) {
    throw exceptions::IncompatibleSizeException(
        "Input number of IOs is of incorrect size, expected " + std::to_string(this->get_size()) + " got "
            + std::to_string(names.size()));
  }
  this->names_ = names;
  this->reset_timestamp();
}

template<typename T>
void IOState<T>::set_data(const Eigen::Vector<T, Eigen::Dynamic>& data) {
  if (data.size() != this->get_size()) {
    throw exceptions::IncompatibleSizeException(
        "Input is of incorrect size, expected " + std::to_string(this->get_size()) + ", got "
            + std::to_string(data.size()));
  }
  this->data_ = data;
  this->set_empty(false);
}

template<typename T>
bool IOState<T>::is_incompatible(const State& state) const {
  try {
    auto other = dynamic_cast<const IOState<T>&>(state);
    if (this->names_.size() != other.names_.size()) {
      return true;
    }
    for (unsigned int i = 0; i < this->names_.size(); ++i) {
      if (this->names_[i] != other.names_[i]) {
        return true;
      }
    }
    return false;
  } catch (const std::bad_cast& ex) {
    throw exceptions::InvalidCastException(std::string("Could not cast the given object to an IOState: ") + ex.what());
  }
}

template<typename T>
void IOState<T>::assert_index_in_range(unsigned int io_index, unsigned int size) {
  if (io_index > size) {
    throw exceptions::IONotFoundException(
        "Index '" + std::to_string(io_index) + "' is out of range for IO state with size" + std::to_string(size));
  }
}

}// namespace state_representation
