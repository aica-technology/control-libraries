#include "state_representation/space/cartesian/CartesianAcceleration.hpp"
#include "state_representation/exceptions/EmptyStateException.hpp"

using namespace state_representation::exceptions;

namespace state_representation {

CartesianAcceleration::CartesianAcceleration() {
  this->set_type(StateType::CARTESIAN_ACCELERATION);
}

CartesianAcceleration::CartesianAcceleration(const std::string& name, const std::string& reference) :
    CartesianState(name, reference) {
  this->set_type(StateType::CARTESIAN_ACCELERATION);
}

CartesianAcceleration::CartesianAcceleration(
    const std::string& name, const Eigen::Vector3d& linear_acceleration, const std::string& reference
) : CartesianState(name, reference) {
  this->set_type(StateType::CARTESIAN_ACCELERATION);
  this->set_linear_acceleration(linear_acceleration);
}

CartesianAcceleration::CartesianAcceleration(
    const std::string& name, const Eigen::Vector3d& linear_acceleration, const Eigen::Vector3d& angular_acceleration,
    const std::string& reference
) : CartesianState(name, reference) {
  this->set_type(StateType::CARTESIAN_ACCELERATION);
  this->set_linear_acceleration(linear_acceleration);
  this->set_angular_acceleration(angular_acceleration);
}

CartesianAcceleration::CartesianAcceleration(
    const std::string& name, const Eigen::Matrix<double, 6, 1>& acceleration, const std::string& reference
) : CartesianState(name, reference) {
  this->set_type(StateType::CARTESIAN_ACCELERATION);
  this->set_acceleration(acceleration);
}

CartesianAcceleration::CartesianAcceleration(const CartesianState& state) : CartesianState(state) {
  this->set_type(StateType::CARTESIAN_ACCELERATION);
  if (state) {
    this->set_zero();
    this->set_acceleration(state.get_acceleration());
  }
}

CartesianAcceleration::CartesianAcceleration(const CartesianAcceleration& acceleration) :
    CartesianAcceleration(static_cast<const CartesianState&>(acceleration)) {}

CartesianAcceleration::CartesianAcceleration(const CartesianTwist& twist) :
    CartesianAcceleration(twist.differentiate(1.0)) {}

CartesianAcceleration CartesianAcceleration::Zero(const std::string& name, const std::string& reference) {
  return CartesianState::Identity(name, reference);
}

CartesianAcceleration CartesianAcceleration::Random(const std::string& name, const std::string& reference) {
  // separating in the two lines in needed to avoid compilation error due to ambiguous constructor call
  Eigen::Matrix<double, 6, 1> random = Eigen::Matrix<double, 6, 1>::Random();
  return CartesianAcceleration(name, random, reference);
}

Eigen::VectorXd CartesianAcceleration::data() const {
  return this->get_acceleration();
}

void CartesianAcceleration::set_data(const Eigen::VectorXd& data) {
  if (data.size() != 6) {
    throw IncompatibleSizeException(
        "Input is of incorrect size: expected 6, given " + std::to_string(data.size()));
  }
  this->set_acceleration(data);
}

void CartesianAcceleration::set_data(const std::vector<double>& data) {
  this->set_data(Eigen::VectorXd::Map(data.data(), data.size()));
}

void CartesianAcceleration::clamp(
    double max_linear, double max_angular, double linear_noise_ratio, double angular_noise_ratio
) {
  // clamp linear
  this->clamp_state_variable(max_linear, CartesianStateVariable::LINEAR_ACCELERATION, linear_noise_ratio);
  // clamp angular
  this->clamp_state_variable(max_angular, CartesianStateVariable::ANGULAR_ACCELERATION, angular_noise_ratio);
}

CartesianAcceleration CartesianAcceleration::clamped(
    double max_linear, double max_angular, double linear_noise_ratio, double angular_noise_ratio
) const {
  CartesianAcceleration result(*this);
  result.clamp(max_linear, max_angular, linear_noise_ratio, angular_noise_ratio);
  return result;
}

CartesianAcceleration CartesianAcceleration::copy() const {
  CartesianAcceleration result(*this);
  return result;
}

CartesianTwist CartesianAcceleration::integrate(double dt) const {
  CartesianTwist twist(this->get_name(), this->get_reference_frame());
  twist.set_twist(dt * this->get_acceleration());
  return twist;
}

CartesianTwist CartesianAcceleration::integrate(const std::chrono::nanoseconds& dt) const {
  // convert the dt to a double with the second as reference
  return this->integrate(dt.count() / 1e9);
}

CartesianAcceleration CartesianAcceleration::inverse() const {
  return this->CartesianState::inverse();
}

CartesianAcceleration CartesianAcceleration::normalized(const CartesianStateVariable& state_variable_type) const {
  return CartesianState::normalized(state_variable_type);
}

std::vector<double> CartesianAcceleration::norms(const CartesianStateVariable& state_variable_type) const {
  return CartesianState::norms(state_variable_type);
}

CartesianAcceleration& CartesianAcceleration::operator*=(double lambda) {
  this->CartesianState::operator*=(lambda);
  return (*this);
}

CartesianAcceleration CartesianAcceleration::operator*(double lambda) const {
  return this->CartesianState::operator*(lambda);
}

CartesianAcceleration operator*(double lambda, const CartesianAcceleration& acceleration) {
  return acceleration * lambda;
}

CartesianAcceleration operator*(const Eigen::Matrix<double, 6, 6>& lambda, const CartesianAcceleration& acceleration) {
  CartesianAcceleration result(acceleration);
  result.set_acceleration(lambda * result.get_acceleration());
  return result;
}

CartesianTwist CartesianAcceleration::operator*(const std::chrono::nanoseconds& dt) const {
  return this->integrate(dt);
}

CartesianTwist operator*(const std::chrono::nanoseconds& dt, const CartesianAcceleration& acceleration) {
  return acceleration.integrate(dt);
}

CartesianAcceleration& CartesianAcceleration::operator/=(double lambda) {
  this->CartesianState::operator/=(lambda);
  return (*this);
}

CartesianAcceleration CartesianAcceleration::operator/(double lambda) const {
  return this->CartesianState::operator/(lambda);
}

CartesianAcceleration& CartesianAcceleration::operator+=(const CartesianAcceleration& acceleration) {
  this->CartesianState::operator+=(acceleration);
  return (*this);
}

CartesianAcceleration& CartesianAcceleration::operator+=(const CartesianState& state) {
  this->CartesianState::operator+=(state);
  return (*this);
}

CartesianAcceleration CartesianAcceleration::operator+(const CartesianAcceleration& acceleration) const {
  return this->CartesianState::operator+(acceleration);
}

CartesianState CartesianAcceleration::operator+(const CartesianState& state) const {
  return this->CartesianState::operator+(state);
}

CartesianAcceleration CartesianAcceleration::operator-() const {
  return this->CartesianState::operator-();
}

CartesianAcceleration& CartesianAcceleration::operator-=(const CartesianAcceleration& acceleration) {
  this->CartesianState::operator-=(acceleration);
  return (*this);
}

CartesianAcceleration& CartesianAcceleration::operator-=(const CartesianState& state) {
  this->CartesianState::operator-=(state);
  return (*this);
}

CartesianAcceleration CartesianAcceleration::operator-(const CartesianAcceleration& acceleration) const {
  return this->CartesianState::operator-(acceleration);
}

CartesianState CartesianAcceleration::operator-(const CartesianState& state) const {
  return this->CartesianState::operator-(state);
}

std::ostream& operator<<(std::ostream& os, const CartesianAcceleration& acceleration) {
  os << acceleration.to_string();
  return os;
}

}// namespace state_representation
