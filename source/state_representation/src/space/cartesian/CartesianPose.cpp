#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/IncompatibleSizeException.hpp"

using namespace state_representation::exceptions;

namespace state_representation {

CartesianPose::CartesianPose() {
  this->set_type(StateType::CARTESIAN_POSE);
}

CartesianPose::CartesianPose(const std::string& name, const std::string& reference) : CartesianState(name, reference) {
  this->set_type(StateType::CARTESIAN_POSE);
}

CartesianPose::CartesianPose(const std::string& name, const Eigen::Vector3d& position, const std::string& reference) :
    CartesianState(name, reference) {
  this->set_type(StateType::CARTESIAN_POSE);
  this->set_position(position);
}

CartesianPose::CartesianPose(
    const std::string& name, double x, double y, double z, const std::string& reference
) : CartesianState(name, reference) {
  this->set_type(StateType::CARTESIAN_POSE);
  this->set_position(x, y, z);
}

CartesianPose::CartesianPose(
    const std::string& name, const Eigen::Quaterniond& orientation, const std::string& reference
) : CartesianState(name, reference) {
  this->set_type(StateType::CARTESIAN_POSE);
  this->set_orientation(orientation);
}

CartesianPose::CartesianPose(
    const std::string& name, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
    const std::string& reference
) : CartesianState(name, reference) {
  this->set_type(StateType::CARTESIAN_POSE);
  this->set_position(position);
  this->set_orientation(orientation);
}

CartesianPose::CartesianPose(const CartesianState& state) : CartesianState(state) {
  this->set_type(StateType::CARTESIAN_POSE);
  if (state) {
    this->set_zero();
    this->set_pose(state.get_pose());
  }
}

CartesianPose::CartesianPose(const CartesianPose& pose) : CartesianPose(static_cast<const CartesianState&>(pose)) {}

CartesianPose::CartesianPose(const CartesianTwist& twist) : CartesianPose(twist.integrate(1.0)) {}

CartesianPose CartesianPose::Identity(const std::string& name, const std::string& reference) {
  return CartesianState::Identity(name, reference);
}

CartesianPose CartesianPose::Random(const std::string& name, const std::string& reference) {
  return CartesianPose(name, Eigen::Vector3d::Random(), Eigen::Quaterniond::UnitRandom(), reference);
}

CartesianPose CartesianPose::from_transformation_matrix(
    const std::string& name, const Eigen::Matrix4d& transformation_matrix, const std::string& reference) {
  auto pose = CartesianPose(name, reference);
  pose.set_pose_from_transformation_matrix(transformation_matrix);
  return pose;
}

Eigen::VectorXd CartesianPose::data() const {
  return this->get_pose();
}

void CartesianPose::set_data(const Eigen::VectorXd& data) {
  if (data.size() != 7) {
    throw IncompatibleSizeException(
        "Input is of incorrect size: expected 7, given " + std::to_string(data.size()));
  }
  this->set_pose(data);
}

void CartesianPose::set_data(const std::vector<double>& data) {
  this->set_data(Eigen::VectorXd::Map(data.data(), data.size()));
}

CartesianPose CartesianPose::copy() const {
  CartesianPose result(*this);
  return result;
}

CartesianTwist CartesianPose::differentiate(double dt) const {
  CartesianTwist twist(this->get_name(), this->get_reference_frame());
  twist.set_linear_velocity(this->get_position() / dt);
  // set angular velocity from the log of the quaternion error
  Eigen::Quaterniond log_q = math_tools::log(this->get_orientation());
  twist.set_angular_velocity(2 * log_q.vec() / dt);
  return twist;
}

CartesianTwist CartesianPose::differentiate(const std::chrono::nanoseconds& dt) const {
  return this->differentiate(dt.count() / 1e9);
}

CartesianPose CartesianPose::inverse() const {
  return this->CartesianState::inverse();
}

CartesianPose CartesianPose::normalized(const CartesianStateVariable& state_variable_type) const {
  return CartesianState::normalized(state_variable_type);
}

std::vector<double> CartesianPose::norms(const CartesianStateVariable& state_variable_type) const {
  return CartesianState::norms(state_variable_type);
}

CartesianPose& CartesianPose::operator*=(const CartesianState& state) {
  this->CartesianState::operator*=(state);
  return (*this);
}

CartesianPose& CartesianPose::operator*=(const CartesianPose& pose) {
  this->CartesianState::operator*=(pose);
  return (*this);
}

CartesianState CartesianPose::operator*(const CartesianState& state) const {
  return this->CartesianState::operator*(state);
}

CartesianPose CartesianPose::operator*(const CartesianPose& pose) const {
  return this->CartesianState::operator*(pose);
}

CartesianTwist CartesianPose::operator*(const CartesianTwist& twist) const {
  return this->CartesianState::operator*(twist);
}

CartesianAcceleration CartesianPose::operator*(const CartesianAcceleration& acceleration) const {
  return this->CartesianState::operator*(acceleration);
}

CartesianWrench CartesianPose::operator*(const CartesianWrench& wrench) const {
  return this->CartesianState::operator*(wrench);
}

CartesianPose& CartesianPose::operator*=(double lambda) {
  this->CartesianState::operator*=(lambda);
  return (*this);
}

CartesianPose CartesianPose::operator*(double lambda) const {
  return this->CartesianState::operator*(lambda);
}

CartesianPose operator*(double lambda, const CartesianPose& pose) {
  return pose * lambda;
}

CartesianPose& CartesianPose::operator/=(double lambda) {
  this->CartesianState::operator/=(lambda);
  return (*this);
}

CartesianPose CartesianPose::operator/(double lambda) const {
  return this->CartesianState::operator/(lambda);
}

CartesianTwist CartesianPose::operator/(const std::chrono::nanoseconds& dt) const {
  return this->differentiate(dt);
}

CartesianPose& CartesianPose::operator+=(const CartesianPose& pose) {
  this->CartesianState::operator+=(pose);
  return (*this);
}

CartesianPose& CartesianPose::operator+=(const CartesianState& state) {
    this->CartesianState::operator+=(state);
    return (*this);
}

CartesianPose CartesianPose::operator+(const CartesianPose& pose) const {
  return this->CartesianState::operator+(pose);
}

CartesianState CartesianPose::operator+(const CartesianState& state) const {
  return this->CartesianState::operator+(state);
}

CartesianPose CartesianPose::operator-() const {
  return this->CartesianState::operator-();
}

CartesianPose& CartesianPose::operator-=(const CartesianPose& pose) {
  this->CartesianState::operator-=(pose);
  return (*this);
}

CartesianPose& CartesianPose::operator-=(const CartesianState& state) {
  this->CartesianState::operator-=(state);
  return (*this);
}

CartesianPose CartesianPose::operator-(const CartesianPose& pose) const {
  return this->CartesianState::operator-(pose);
}

CartesianState CartesianPose::operator-(const CartesianState& state) const {
  return this->CartesianState::operator-(state);
}

std::ostream& operator<<(std::ostream& os, const CartesianPose& pose) {
  os << pose.to_string();
  return os;
}

}// namespace state_representation
