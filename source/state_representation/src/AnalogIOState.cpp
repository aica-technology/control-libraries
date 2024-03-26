#include "state_representation/AnalogIOState.hpp"

#include "state_representation/exceptions/IONotFoundException.hpp"

using namespace state_representation::exceptions;

namespace state_representation {

AnalogIOState::AnalogIOState() : IOState<double>() {
  this->set_type(StateType::ANALOG_IO_STATE);
}

AnalogIOState::AnalogIOState(const std::string& name, unsigned int nb_ios) : IOState<double>(name, nb_ios) {
  this->set_type(StateType::ANALOG_IO_STATE);
  this->data_ = Eigen::VectorXd::Zero(nb_ios);
}

AnalogIOState::AnalogIOState(const std::string& name, const std::vector<std::string>& io_names) :
  AnalogIOState(name, io_names.size()) {
  this->set_names(io_names);
}

AnalogIOState::AnalogIOState(const AnalogIOState& state) : AnalogIOState(state.get_name(), state.get_names()) {
  if (state) {
    this->set_data(state.data());
  }
}

AnalogIOState AnalogIOState::Zero(const std::string& name, unsigned int nb_ios) {
  AnalogIOState zero = AnalogIOState(name, nb_ios);
  // specify that the default constructed zero state is non-empty
  zero.set_empty(false);
  return zero;
}

AnalogIOState AnalogIOState::Zero(const std::string& name, const std::vector<std::string>& io_names) {
  AnalogIOState zero = AnalogIOState(name, io_names);
  // specify that the default constructed zero state is non-empty
  zero.set_empty(false);
  return zero;
}

AnalogIOState AnalogIOState::Random(const std::string& name, unsigned int nb_ios) {
  AnalogIOState random = AnalogIOState(name, nb_ios);
  // set all the state variables to random
  random.set_data(Eigen::VectorXd::Random(random.get_size()));
  return random;
}

AnalogIOState AnalogIOState::Random(const std::string& name, const std::vector<std::string>& io_names) {
  AnalogIOState random = AnalogIOState(name, io_names);
  // set all the state variables to random
  random.set_data(Eigen::VectorXd::Random(random.get_size()));
  return random;
}

AnalogIOState& AnalogIOState::operator=(const AnalogIOState& state) {
  AnalogIOState tmp(state);
  swap(*this, tmp);
  return *this;
}

AnalogIOState AnalogIOState::copy() const {
  AnalogIOState result(*this);
  return result;
}

void AnalogIOState::reset() {
  this->set_zero();
  this->State::reset();
}

void AnalogIOState::set_zero() {
  if (this->get_size() > 0) {
    this->data_.setZero();
    this->set_empty(false);
  }
}

std::string AnalogIOState::to_string() const {
  std::stringstream s;
  s << this->State::to_string();
  s << std::endl << "analog io names: [";
  for (auto& n : this->get_names()) { s << n << ", "; }
  s << "]";
  if (this->is_empty()) {
    return s.str();
  }
  s << std::endl << "values: [";
  for (auto& p : this->data()) { s << p << ", "; }
  s << "]";
  return s.str();
}

std::ostream& operator<<(std::ostream& os, const AnalogIOState& state) {
  os << state.to_string();
  return os;
}
}// namespace state_representation
