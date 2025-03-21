#include "state_representation/space/joint/JointVelocities.hpp"

namespace state_representation {

using namespace exceptions;

JointVelocities::JointVelocities() {
  this->set_type(StateType::JOINT_VELOCITIES);
}

JointVelocities::JointVelocities(const std::string& robot_name, unsigned int nb_joints)
    : JointState(robot_name, nb_joints) {
  this->set_type(StateType::JOINT_VELOCITIES);
}

JointVelocities::JointVelocities(const std::string& robot_name, const Eigen::VectorXd& velocities)
    : JointState(robot_name, velocities.size()) {
  this->set_type(StateType::JOINT_VELOCITIES);
  this->set_velocities(velocities);
}

JointVelocities::JointVelocities(const std::string& robot_name, const std::vector<std::string>& joint_names)
    : JointState(robot_name, joint_names) {
  this->set_type(StateType::JOINT_VELOCITIES);
}

JointVelocities::JointVelocities(
    const std::string& robot_name, const std::vector<std::string>& joint_names, const Eigen::VectorXd& velocities
)
    : JointState(robot_name, joint_names) {
  this->set_type(StateType::JOINT_VELOCITIES);
  this->set_velocities(velocities);
}

JointVelocities::JointVelocities(const JointState& state) : JointState(state) {
  this->set_type(StateType::JOINT_VELOCITIES);
  if (state) {
    this->set_zero();
    this->set_velocities(state.get_velocities());
  }
}

JointVelocities::JointVelocities(const JointVelocities& velocities)
    : JointVelocities(static_cast<const JointState&>(velocities)) {}

JointVelocities::JointVelocities(const JointAccelerations& accelerations)
    : JointVelocities(accelerations.integrate(1.0)) {}

JointVelocities::JointVelocities(const JointPositions& positions) : JointVelocities(positions.differentiate(1.0)) {}

JointVelocities JointVelocities::Zero(const std::string& robot_name, unsigned int nb_joints) {
  return JointState::Zero(robot_name, nb_joints);
}

JointVelocities JointVelocities::Zero(const std::string& robot_name, const std::vector<std::string>& joint_names) {
  return JointState::Zero(robot_name, joint_names);
}

JointVelocities JointVelocities::Random(const std::string& robot_name, unsigned int nb_joints) {
  return JointVelocities(robot_name, Eigen::VectorXd::Random(nb_joints));
}

JointVelocities JointVelocities::Random(const std::string& robot_name, const std::vector<std::string>& joint_names) {
  return JointVelocities(robot_name, joint_names, Eigen::VectorXd::Random(joint_names.size()));
}

Eigen::VectorXd JointVelocities::data() const {
  return this->get_velocities();
}

void JointVelocities::set_data(const Eigen::VectorXd& data) {
  this->set_velocities(data);
}

void JointVelocities::set_data(const std::vector<double>& data) {
  this->set_velocities(Eigen::VectorXd::Map(data.data(), data.size()));
}

void JointVelocities::clamp(double max_absolute_value, double noise_ratio) {
  this->clamp_state_variable(max_absolute_value, JointStateVariable::VELOCITIES, noise_ratio);
}

void JointVelocities::clamp(const Eigen::ArrayXd& max_absolute_value_array, const Eigen::ArrayXd& noise_ratio_array) {
  this->clamp_state_variable(max_absolute_value_array, JointStateVariable::VELOCITIES, noise_ratio_array);
}

JointVelocities JointVelocities::clamped(double max_absolute_value, double noise_ratio) const {
  JointVelocities result(*this);
  result.clamp(max_absolute_value, noise_ratio);
  return result;
}

JointVelocities JointVelocities::clamped(
    const Eigen::ArrayXd& max_absolute_value_array, const Eigen::ArrayXd& noise_ratio_array
) const {
  JointVelocities result(*this);
  result.clamp(max_absolute_value_array, noise_ratio_array);
  return result;
}

JointVelocities JointVelocities::copy() const {
  JointVelocities result(*this);
  return result;
}

JointAccelerations JointVelocities::differentiate(double dt) const {
  return JointAccelerations(this->get_name(), this->get_names(), this->get_velocities() / dt);
}

JointAccelerations JointVelocities::differentiate(const std::chrono::nanoseconds& dt) const {
  // convert the period to a double with the second as reference
  return this->differentiate(dt.count() / 1e9);
}

JointPositions JointVelocities::integrate(double dt) const {
  return JointPositions(this->get_name(), this->get_names(), dt * this->get_velocities());
}

JointPositions JointVelocities::integrate(const std::chrono::nanoseconds& dt) const {
  // convert the period to a double with the second as reference
  return this->integrate(dt.count() / 1e9);
}

JointVelocities& JointVelocities::operator*=(double lambda) {
  this->JointState::operator*=(lambda);
  return (*this);
}

JointVelocities JointVelocities::operator*(double lambda) const {
  return this->JointState::operator*(lambda);
}

JointVelocities operator*(double lambda, const JointVelocities& velocities) {
  JointVelocities result(velocities);
  result *= lambda;
  return result;
}

JointVelocities operator*(const Eigen::MatrixXd& lambda, const JointVelocities& velocities) {
  JointVelocities result(velocities);
  result.multiply_state_variable(lambda, JointStateVariable::VELOCITIES);
  return result;
}

JointPositions JointVelocities::operator*(const std::chrono::nanoseconds& dt) const {
  return this->integrate(dt);
}

JointPositions operator*(const std::chrono::nanoseconds& dt, const JointVelocities& velocities) {
  return velocities.integrate(dt);
}

JointVelocities& JointVelocities::operator/=(double lambda) {
  this->JointState::operator/=(lambda);
  return (*this);
}

JointVelocities JointVelocities::operator/(double lambda) const {
  return this->JointState::operator/(lambda);
}

JointAccelerations JointVelocities::operator/(const std::chrono::nanoseconds& dt) const {
  return this->differentiate(dt);
}

JointVelocities& JointVelocities::operator+=(const JointVelocities& velocities) {
  this->JointState::operator+=(velocities);
  return (*this);
}

JointVelocities& JointVelocities::operator+=(const JointState& state) {
  this->JointState::operator+=(state);
  return (*this);
}

JointVelocities JointVelocities::operator+(const JointVelocities& velocities) const {
  return this->JointState::operator+(velocities);
}

JointState JointVelocities::operator+(const JointState& state) const {
  return this->JointState::operator+(state);
}

JointVelocities JointVelocities::operator-() const {
  return this->JointState::operator-();
}

JointVelocities& JointVelocities::operator-=(const JointVelocities& velocities) {
  this->JointState::operator-=(velocities);
  return (*this);
}

JointVelocities& JointVelocities::operator-=(const JointState& state) {
  this->JointState::operator-=(state);
  return (*this);
}

JointVelocities JointVelocities::operator-(const JointVelocities& velocities) const {
  return this->JointState::operator-(velocities);
}

JointState JointVelocities::operator-(const JointState& state) const {
  return this->JointState::operator-(state);
}

std::ostream& operator<<(std::ostream& os, const JointVelocities& velocities) {
  os << velocities.to_string();
  return os;
}
}// namespace state_representation
