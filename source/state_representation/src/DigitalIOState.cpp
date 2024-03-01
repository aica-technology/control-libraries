#include "state_representation/DigitalIOState.hpp"

#include "state_representation/exceptions/IONotFoundException.hpp"

using namespace state_representation::exceptions;

namespace state_representation {

DigitalIOState::DigitalIOState() : IOState<double>() {
  this->set_type(StateType::DIGITAL_IO_STATE);
}

DigitalIOState::DigitalIOState(const std::string& name, unsigned int nb_ios) : IOState<double>(name, nb_ios) {
  this->set_type(StateType::DIGITAL_IO_STATE);
  this->data_ = Eigen::VectorXd::Zero(nb_ios);
}

DigitalIOState::DigitalIOState(const std::string& name, const std::vector<std::string>& io_names) :
  DigitalIOState(name, io_names.size()) {
  this->set_names(io_names);
}

DigitalIOState::DigitalIOState(const DigitalIOState& state) : DigitalIOState(state.get_name(), state.get_names()) {
  if (state) {
    this->set_data(state.data());
  }
}

DigitalIOState DigitalIOState::Zero(const std::string& name, unsigned int nb_ios) {
  DigitalIOState zero = DigitalIOState(name, nb_ios);
  // as opposed to the constructor specify this state to be filled
  zero.set_empty(false);
  return zero;
}

DigitalIOState DigitalIOState::Zero(const std::string& name, const std::vector<std::string>& io_names) {
  DigitalIOState zero = DigitalIOState(name, io_names);
  // as opposed to the constructor specify this state to be filled
  zero.set_empty(false);
  return zero;
}

DigitalIOState DigitalIOState::Random(const std::string& name, unsigned int nb_ios) {
  DigitalIOState random = DigitalIOState(name, nb_ios);
  // set all the state variables to random
  random.set_data(Eigen::VectorXd::Random(random.get_size()));
  return random;
}

DigitalIOState DigitalIOState::Random(const std::string& name, const std::vector<std::string>& io_names) {
  DigitalIOState random = DigitalIOState(name, io_names);
  // set all the state variables to random
  random.set_data(Eigen::VectorXd::Random(random.get_size()));
  return random;
}

DigitalIOState& DigitalIOState::operator=(const DigitalIOState& state) {
  DigitalIOState tmp(state);
  swap(*this, tmp);
  return *this;
}

double DigitalIOState::get_value(const std::string& io_name) const {
  return this->get_value(this->get_io_index(io_name));
}

double DigitalIOState::get_value(unsigned int io_index) const {
  this->assert_not_empty();
  IOState<double>::assert_index_in_range(io_index, this->get_size());
  return this->data_(io_index);
}

void DigitalIOState::set_value(double value, const std::string& io_name) {
  this->set_value(value, this->get_io_index(io_name));
}

void DigitalIOState::set_value(double value, unsigned int io_index) {
  IOState<double>::assert_index_in_range(io_index, this->get_size());
  this->data_(io_index) = value;
  this->set_empty(false);
}

DigitalIOState DigitalIOState::copy() const {
  DigitalIOState result(*this);
  return result;
}

void DigitalIOState::reset() {
  this->set_zero();
  this->State::reset();
}

void DigitalIOState::set_zero() {
  if (this->get_size() > 0) {
    this->data_.setZero();
    this->set_empty(false);
  }
}

std::string DigitalIOState::to_string() const {
  std::stringstream s;
  s << this->State::to_string();
  s << std::endl << "digital io names: [";
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

std::ostream& operator<<(std::ostream& os, const DigitalIOState& state) {
  os << state.to_string();
  return os;
}
}// namespace state_representation
