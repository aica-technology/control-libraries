#include "state_representation/space/cartesian/CartesianWrench.hpp"
#include "state_representation/exceptions/EmptyStateException.hpp"

namespace state_representation {

using namespace exceptions;

CartesianWrench::CartesianWrench() {
  this->set_type(StateType::CARTESIAN_WRENCH);
}

CartesianWrench::CartesianWrench(const std::string& name, const std::string& reference) :
    CartesianState(name, reference) {
  this->set_type(StateType::CARTESIAN_WRENCH);
}

CartesianWrench::CartesianWrench(const std::string& name, const Eigen::Vector3d& force, const std::string& reference) :
    CartesianState(name, reference) {
  this->set_type(StateType::CARTESIAN_WRENCH);
  this->set_force(force);
}

CartesianWrench::CartesianWrench(
    const std::string& name, const Eigen::Vector3d& force, const Eigen::Vector3d& torque, const std::string& reference
) : CartesianState(name, reference) {
  this->set_type(StateType::CARTESIAN_WRENCH);
  this->set_force(force);
  this->set_torque(torque);
}

CartesianWrench::CartesianWrench(
    const std::string& name, const Eigen::Matrix<double, 6, 1>& wrench, const std::string& reference
) : CartesianState(name, reference) {
  this->set_type(StateType::CARTESIAN_WRENCH);
  this->set_wrench(wrench);
}

CartesianWrench::CartesianWrench(const CartesianState& state) : CartesianState(state) {
  this->set_type(StateType::CARTESIAN_WRENCH);
  if (state) {
    this->set_zero();
    this->set_wrench(state.get_wrench());
  }
}

CartesianWrench::CartesianWrench(const CartesianWrench& wrench) :
    CartesianWrench(static_cast<const CartesianState&>(wrench)) {}

CartesianWrench CartesianWrench::Zero(const std::string& name, const std::string& reference) {
  return CartesianState::Identity(name, reference);
}

CartesianWrench CartesianWrench::Random(const std::string& name, const std::string& reference) {
  // separating in the two lines in needed to avoid compilation error due to ambiguous constructor call
  Eigen::Matrix<double, 6, 1> random = Eigen::Matrix<double, 6, 1>::Random();
  return CartesianWrench(name, random, reference);
}

Eigen::VectorXd CartesianWrench::data() const {
  return this->get_wrench();
}

void CartesianWrench::set_data(const Eigen::VectorXd& data) {
  if (data.size() != 6) {
    throw IncompatibleSizeException(
        "Input is of incorrect size: expected 6, given " + std::to_string(data.size()));
  }
  this->set_wrench(data);
}

void CartesianWrench::set_data(const std::vector<double>& data) {
  this->set_data(Eigen::VectorXd::Map(data.data(), data.size()));
}

void CartesianWrench::clamp(double max_force, double max_torque, double force_noise_ratio, double torque_noise_ratio) {
  // clamp force
  this->clamp_state_variable(max_force, CartesianStateVariable::FORCE, force_noise_ratio);
  // clamp torque
  this->clamp_state_variable(max_torque, CartesianStateVariable::TORQUE, torque_noise_ratio);
}

CartesianWrench CartesianWrench::clamped(
    double max_force, double max_torque, double force_noise_ratio, double torque_noise_ratio
) const {
  CartesianWrench result(*this);
  result.clamp(max_force, max_torque, force_noise_ratio, torque_noise_ratio);
  return result;
}

CartesianWrench CartesianWrench::copy() const {
  CartesianWrench result(*this);
  return result;
}

CartesianWrench CartesianWrench::inverse() const {
  return this->CartesianState::inverse();
}

CartesianWrench CartesianWrench::normalized(const CartesianStateVariable& state_variable_type) const {
  return CartesianState::normalized(state_variable_type);
}

std::vector<double> CartesianWrench::norms(const CartesianStateVariable& state_variable_type) const {
  return CartesianState::norms(state_variable_type);
}

CartesianWrench& CartesianWrench::operator*=(double lambda) {
  this->CartesianState::operator*=(lambda);
  return (*this);
}

CartesianWrench operator*(double lambda, const CartesianWrench& wrench) {
  return wrench * lambda;
}

CartesianWrench CartesianWrench::operator*(double lambda) const {
  return this->CartesianState::operator*(lambda);
}

CartesianWrench operator*(const Eigen::Matrix<double, 6, 6>& lambda, const CartesianWrench& wrench) {
  // sanity check
  if (wrench.is_empty()) {
    throw EmptyStateException(wrench.get_name() + " state is empty");
  }
  CartesianWrench result(wrench);
  result.set_wrench(lambda * result.get_wrench());
  return result;
}

CartesianWrench& CartesianWrench::operator/=(double lambda) {
  this->CartesianState::operator/=(lambda);
  return (*this);
}

CartesianWrench CartesianWrench::operator/(double lambda) const {
  return this->CartesianState::operator/(lambda);
}

CartesianWrench& CartesianWrench::operator+=(const CartesianWrench& wrench) {
  this->CartesianState::operator+=(wrench);
  return (*this);
}

CartesianWrench& CartesianWrench::operator+=(const CartesianState& state) {
  this->CartesianState::operator+=(state);
  return (*this);
}

CartesianWrench CartesianWrench::operator+(const CartesianWrench& wrench) const {
  return this->CartesianState::operator+(wrench);
}

CartesianState CartesianWrench::operator+(const CartesianState& state) const {
  return this->CartesianState::operator+(state);
}

CartesianWrench CartesianWrench::operator-() const {
  return this->CartesianState::operator-();
}

CartesianWrench& CartesianWrench::operator-=(const CartesianWrench& wrench) {
  this->CartesianState::operator-=(wrench);
  return (*this);
}

CartesianWrench& CartesianWrench::operator-=(const CartesianState& state) {
  this->CartesianState::operator-=(state);
  return (*this);
}

CartesianWrench CartesianWrench::operator-(const CartesianWrench& wrench) const {
  return this->CartesianState::operator-(wrench);
}

CartesianState CartesianWrench::operator-(const CartesianState& state) const {
  return this->CartesianState::operator-(state);
}

std::ostream& operator<<(std::ostream& os, const CartesianWrench& wrench) {
  os << wrench.to_string();
  return os;
}

}// namespace state_representation
