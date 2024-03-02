#include "state_representation/DigitalIOState.hpp"

#include "state_representation/exceptions/IONotFoundException.hpp"

using namespace state_representation::exceptions;

namespace state_representation {

DigitalIOState::DigitalIOState() : IOState<bool>() {
  this->set_type(StateType::DIGITAL_IO_STATE);
}

DigitalIOState::DigitalIOState(const std::string& name, unsigned int nb_ios) : IOState<bool>(name, nb_ios) {
  this->set_type(StateType::DIGITAL_IO_STATE);
  this->data_ = Eigen::Vector<bool, Eigen::Dynamic>::Zero(nb_ios);
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
  random.set_data(Eigen::Vector<bool, -1>::Random(random.get_size()));
  return random;
}

DigitalIOState DigitalIOState::Random(const std::string& name, const std::vector<std::string>& io_names) {
  DigitalIOState random = DigitalIOState(name, io_names);
  // set all the state variables to random
  random.set_data(Eigen::Vector<bool, -1>::Random(random.get_size()));
  return random;
}

DigitalIOState& DigitalIOState::operator=(const DigitalIOState& state) {
  DigitalIOState tmp(state);
  swap(*this, tmp);
  return *this;
}

bool DigitalIOState::is_true(const std::string& io_name) const {
  return this->is_true(this->get_io_index(io_name));
}

bool DigitalIOState::is_true(unsigned int io_index) const {
  this->assert_not_empty();
  IOState<bool>::assert_index_in_range(io_index, this->get_size());
  return this->data_(io_index) == true;
}

bool DigitalIOState::is_false(const std::string& io_name) const {
  return !this->is_true(io_name);
}

bool DigitalIOState::is_false(unsigned int io_index) const {
  return !this->is_true(io_index);
}

void DigitalIOState::set_value(bool value, unsigned int io_index) {
  IOState<bool>::assert_index_in_range(io_index, this->get_size());
  this->data_(io_index) = value;
  this->set_empty(false);
}

void DigitalIOState::set_true(const std::string& io_name) {
  this->set_true(this->get_io_index(io_name));
}

void DigitalIOState::set_true(unsigned int io_index) {
  this->set_value(true, io_index);
}

void DigitalIOState::set_false(const std::string& io_name) {
  this->set_false(this->get_io_index(io_name));
}

void DigitalIOState::set_false(unsigned int io_index) {
  this->set_value(false, io_index);
}

DigitalIOState DigitalIOState::copy() const {
  DigitalIOState result(*this);
  return result;
}

void DigitalIOState::reset() {
  this->set_false();
  this->State::reset();
}

void DigitalIOState::set_false() {
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
