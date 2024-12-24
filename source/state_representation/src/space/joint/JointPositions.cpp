#include "state_representation/space/joint/JointPositions.hpp"

#include "state_representation/exceptions/EmptyStateException.hpp"

namespace state_representation {

using namespace exceptions;

JointPositions::JointPositions() {
  this->set_type(StateType::JOINT_POSITIONS);
}

JointPositions::JointPositions(const std::string& robot_name, unsigned int nb_joints) :
    JointState(robot_name, nb_joints) {
  this->set_type(StateType::JOINT_POSITIONS);
}

JointPositions::JointPositions(const std::string& robot_name, const Eigen::VectorXd& positions) :
    JointState(robot_name, positions.size()) {
  this->set_type(StateType::JOINT_POSITIONS);
  this->set_positions(positions);
}

JointPositions::JointPositions(const std::string& robot_name, const std::vector<std::string>& joint_names) :
    JointState(robot_name, joint_names) {
  this->set_type(StateType::JOINT_POSITIONS);
}

JointPositions::JointPositions(
    const std::string& robot_name, const std::vector<std::string>& joint_names, const Eigen::VectorXd& positions
) : JointState(robot_name, joint_names) {
  this->set_type(StateType::JOINT_POSITIONS);
  this->set_positions(positions);
}

JointPositions::JointPositions(const JointState& state) : JointState(state) {
  this->set_type(StateType::JOINT_POSITIONS);
  if (state) {
    this->set_zero();
    this->set_positions(state.get_positions());
  }
}

JointPositions::JointPositions(const JointPositions& positions) :
    JointPositions(static_cast<const JointState&>(positions)) {}

JointPositions::JointPositions(const JointVelocities& velocities) : JointPositions(velocities.integrate(1.0)) {}

JointPositions JointPositions::Zero(const std::string& robot_name, unsigned int nb_joints) {
  return JointState::Zero(robot_name, nb_joints);
}

JointPositions JointPositions::Zero(const std::string& robot_name, const std::vector<std::string>& joint_names) {
  return JointState::Zero(robot_name, joint_names);
}

JointPositions JointPositions::Random(const std::string& robot_name, unsigned int nb_joints) {
  return JointPositions(robot_name, Eigen::VectorXd::Random(nb_joints));
}

JointPositions JointPositions::Random(const std::string& robot_name, const std::vector<std::string>& joint_names) {
  return JointPositions(robot_name, joint_names, Eigen::VectorXd::Random(joint_names.size()));
}

Eigen::VectorXd JointPositions::data() const {
  return this->get_positions();
}

void JointPositions::set_data(const Eigen::VectorXd& data) {
  this->set_positions(data);
}

void JointPositions::set_data(const std::vector<double>& data) {
  this->set_positions(Eigen::VectorXd::Map(data.data(), data.size()));
}

void JointPositions::clamp(double max_absolute_value, double noise_ratio) {
  this->clamp_state_variable(max_absolute_value, JointStateVariable::POSITIONS, noise_ratio);
}

void JointPositions::clamp(const Eigen::ArrayXd& max_absolute_value_array, const Eigen::ArrayXd& noise_ratio_array) {
  this->clamp_state_variable(max_absolute_value_array, JointStateVariable::POSITIONS, noise_ratio_array);
}

JointPositions JointPositions::clamped(double max_absolute_value, double noise_ratio) const {
  JointPositions result(*this);
  result.clamp(max_absolute_value, noise_ratio);
  return result;
}

JointPositions JointPositions::clamped(
    const Eigen::ArrayXd& max_absolute_value_array, const Eigen::ArrayXd& noise_ratio_array
) const {
  JointPositions result(*this);
  result.clamp(max_absolute_value_array, noise_ratio_array);
  return result;
}

JointPositions JointPositions::copy() const {
  JointPositions result(*this);
  return result;
}

JointVelocities JointPositions::differentiate(double dt) const {
  return JointVelocities(this->get_name(), this->get_names(), this->get_positions() / dt);
}

JointVelocities JointPositions::differentiate(const std::chrono::nanoseconds& dt) const {
  return this->differentiate(dt.count() / 1e9);
}

JointPositions& JointPositions::operator*=(double lambda) {
  this->JointState::operator*=(lambda);
  return (*this);
}

JointPositions JointPositions::operator*(double lambda) const {
  return this->JointState::operator*(lambda);
}

JointPositions operator*(double lambda, const JointPositions& positions) {
  JointPositions result(positions);
  result *= lambda;
  return result;
}

JointPositions operator*(const Eigen::MatrixXd& lambda, const JointPositions& positions) {
  JointPositions result(positions);
  result.multiply_state_variable(lambda, JointStateVariable::POSITIONS);
  return result;
}

JointPositions& JointPositions::operator/=(double lambda) {
  this->JointState::operator/=(lambda);
  return (*this);
}

JointPositions JointPositions::operator/(double lambda) const {
  return this->JointState::operator/(lambda);
}

JointVelocities JointPositions::operator/(const std::chrono::nanoseconds& dt) const {
  return this->differentiate(dt);
}

JointPositions& JointPositions::operator+=(const JointPositions& positions) {
  this->JointState::operator+=(positions);
  return (*this);
}

JointPositions& JointPositions::operator+=(const JointState& state) {
  this->JointState::operator+=(state);
  return (*this);
}

JointPositions JointPositions::operator+(const JointPositions& positions) const {
  return this->JointState::operator+(positions);
}

JointState JointPositions::operator+(const JointState& state) const {
  return this->JointState::operator+(state);
}

JointPositions JointPositions::operator-() const {
  return this->JointState::operator-();
}

JointPositions& JointPositions::operator-=(const JointPositions& positions) {
  this->JointState::operator-=(positions);
  return (*this);
}

JointPositions& JointPositions::operator-=(const JointState& state) {
  this->JointState::operator-=(state);
  return (*this);
}

JointPositions JointPositions::operator-(const JointPositions& positions) const {
  return this->JointState::operator-(positions);
}

JointState JointPositions::operator-(const JointState& state) const {
  return this->JointState::operator-(state);
}

std::ostream& operator<<(std::ostream& os, const JointPositions& positions) {
  os << positions.to_string();
  return os;
}
}// namespace state_representation
