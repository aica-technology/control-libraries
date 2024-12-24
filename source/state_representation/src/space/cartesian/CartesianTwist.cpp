#include "state_representation/space/cartesian/CartesianTwist.hpp"
#include "state_representation/exceptions/EmptyStateException.hpp"

namespace state_representation {

using namespace exceptions;

CartesianTwist::CartesianTwist() {
  this->set_type(StateType::CARTESIAN_TWIST);
}

CartesianTwist::CartesianTwist(const std::string& name, const std::string& reference) :
    CartesianState(name, reference) {
  this->set_type(StateType::CARTESIAN_TWIST);
}

CartesianTwist::CartesianTwist(
    const std::string& name, const Eigen::Vector3d& linear_velocity, const std::string& reference
) : CartesianState(name, reference) {
  this->set_type(StateType::CARTESIAN_TWIST);
  this->set_linear_velocity(linear_velocity);
}

CartesianTwist::CartesianTwist(
    const std::string& name, const Eigen::Vector3d& linear_velocity, const Eigen::Vector3d& angular_velocity,
    const std::string& reference
) : CartesianState(name, reference) {
  this->set_type(StateType::CARTESIAN_TWIST);
  this->set_linear_velocity(linear_velocity);
  this->set_angular_velocity(angular_velocity);
}

CartesianTwist::CartesianTwist(
    const std::string& name, const Eigen::Matrix<double, 6, 1>& twist, const std::string& reference
) : CartesianState(name, reference) {
  this->set_type(StateType::CARTESIAN_TWIST);
  this->set_twist(twist);
}

CartesianTwist::CartesianTwist(const CartesianState& state) : CartesianState(state) {
  this->set_type(StateType::CARTESIAN_TWIST);
  if (state) {
    this->set_zero();
    this->set_twist(state.get_twist());
  }
}

CartesianTwist::CartesianTwist(const CartesianTwist& twist) :
    CartesianTwist(static_cast<const CartesianState&>(twist)) {}

CartesianTwist::CartesianTwist(const CartesianPose& pose) : CartesianTwist(pose.differentiate(1.0)) {}

CartesianTwist::CartesianTwist(const CartesianAcceleration& acceleration) :
    CartesianTwist(acceleration.integrate(1.0)) {}

CartesianTwist CartesianTwist::Zero(const std::string& name, const std::string& reference) {
  return CartesianState::Identity(name, reference);
}

CartesianTwist CartesianTwist::Random(const std::string& name, const std::string& reference) {
  // separating in the two lines in needed to avoid compilation error due to ambiguous constructor call
  Eigen::Matrix<double, 6, 1> random = Eigen::Matrix<double, 6, 1>::Random();
  return CartesianTwist(name, random, reference);
}

Eigen::VectorXd CartesianTwist::data() const {
  return this->get_twist();
}

void CartesianTwist::set_data(const Eigen::VectorXd& data) {
  if (data.size() != 6) {
    throw IncompatibleSizeException(
        "Input is of incorrect size: expected 6, given " + std::to_string(data.size()));
  }
  this->set_twist(data);
}

void CartesianTwist::set_data(const std::vector<double>& data) {
  this->set_data(Eigen::VectorXd::Map(data.data(), data.size()));
}

void CartesianTwist::clamp(
    double max_linear, double max_angular, double linear_noise_ratio, double angular_noise_ratio
) {
  // clamp linear
  this->clamp_state_variable(max_linear, CartesianStateVariable::LINEAR_VELOCITY, linear_noise_ratio);
  // clamp angular
  this->clamp_state_variable(max_angular, CartesianStateVariable::ANGULAR_VELOCITY, angular_noise_ratio);
}

CartesianTwist CartesianTwist::clamped(
    double max_linear, double max_angular, double linear_noise_ratio, double angular_noise_ratio
) const {
  CartesianTwist result(*this);
  result.clamp(max_linear, max_angular, linear_noise_ratio, angular_noise_ratio);
  return result;
}

CartesianTwist CartesianTwist::copy() const {
  CartesianTwist result(*this);
  return result;
}

CartesianAcceleration CartesianTwist::differentiate(double dt) const {
  CartesianAcceleration acceleration(this->get_name(), this->get_reference_frame());
  acceleration.set_acceleration(this->get_twist() / dt);
  return acceleration;
}

CartesianAcceleration CartesianTwist::differentiate(const std::chrono::nanoseconds& dt) const {
  return this->differentiate(dt.count() / 1e9);
}

CartesianPose CartesianTwist::integrate(double dt) const {
  CartesianPose displacement(this->get_name(), this->get_reference_frame());
  displacement.set_position(dt * this->get_linear_velocity());
  Eigen::Quaterniond angular_displacement = Eigen::Quaterniond::Identity();
  double angular_norm = this->get_angular_velocity().norm();
  if (angular_norm > 1e-4) {
    double theta = angular_norm * dt * 0.5;
    angular_displacement.w() = cos(theta);
    angular_displacement.vec() = this->get_angular_velocity() / angular_norm * sin(theta);
  }
  displacement.set_orientation(angular_displacement);
  return displacement;
}

CartesianPose CartesianTwist::integrate(const std::chrono::nanoseconds& dt) const {
  // convert the dt to a double with the second as reference
  return this->integrate(dt.count() / 1e9);
}

CartesianTwist CartesianTwist::inverse() const {
  return this->CartesianState::inverse();
}

CartesianTwist CartesianTwist::normalized(const CartesianStateVariable& state_variable_type) const {
  return CartesianState::normalized(state_variable_type);
}

std::vector<double> CartesianTwist::norms(const CartesianStateVariable& state_variable_type) const {
  return CartesianState::norms(state_variable_type);
}

CartesianTwist& CartesianTwist::operator*=(double lambda) {
  this->CartesianState::operator*=(lambda);
  return (*this);
}

CartesianTwist CartesianTwist::operator*(double lambda) const {
  return this->CartesianState::operator*(lambda);
}

CartesianTwist operator*(double lambda, const CartesianTwist& twist) {
  return twist * lambda;
}

CartesianTwist operator*(const Eigen::Matrix<double, 6, 6>& lambda, const CartesianTwist& twist) {
  CartesianTwist result(twist);
  result.set_twist(lambda * result.get_twist());
  return result;
}

CartesianPose CartesianTwist::operator*(const std::chrono::nanoseconds& dt) const {
  return this->integrate(dt);
}

CartesianPose operator*(const std::chrono::nanoseconds& dt, const CartesianTwist& twist) {
  return twist.integrate(dt);
}

CartesianTwist& CartesianTwist::operator/=(double lambda) {
  this->CartesianState::operator/=(lambda);
  return (*this);
}

CartesianTwist CartesianTwist::operator/(double lambda) const {
  return this->CartesianState::operator/(lambda);
}

CartesianAcceleration CartesianTwist::operator/(const std::chrono::nanoseconds& dt) const {
  return this->differentiate(dt);
}

CartesianTwist& CartesianTwist::operator+=(const CartesianTwist& twist) {
  this->CartesianState::operator+=(twist);
  return (*this);
}

CartesianTwist& CartesianTwist::operator+=(const CartesianState& state) {
  this->CartesianState::operator+=(state);
  return (*this);
}

CartesianTwist CartesianTwist::operator+(const CartesianTwist& twist) const {
  return this->CartesianState::operator+(twist);
}

CartesianState CartesianTwist::operator+(const CartesianState& state) const {
  return this->CartesianState::operator+(state);
}

CartesianTwist CartesianTwist::operator-() const {
  return this->CartesianState::operator-();
}

CartesianTwist& CartesianTwist::operator-=(const CartesianTwist& twist) {
  this->CartesianState::operator-=(twist);
  return (*this);
}

CartesianTwist& CartesianTwist::operator-=(const CartesianState& state) {
  this->CartesianState::operator-=(state);
  return (*this);
}

CartesianTwist CartesianTwist::operator-(const CartesianTwist& twist) const {
  return this->CartesianState::operator-(twist);
}

CartesianState CartesianTwist::operator-(const CartesianState& state) const {
  return this->CartesianState::operator-(state);
}

std::ostream& operator<<(std::ostream& os, const CartesianTwist& twist) {
  os << twist.to_string();
  return os;
}

}// namespace state_representation
