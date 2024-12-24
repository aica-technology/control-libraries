#include "state_representation/space/joint/JointAccelerations.hpp"

#include "state_representation/exceptions/EmptyStateException.hpp"

namespace state_representation {

using namespace exceptions;

JointAccelerations::JointAccelerations() {
  this->set_type(StateType::JOINT_ACCELERATIONS);
}

JointAccelerations::JointAccelerations(const std::string& robot_name, unsigned int nb_joints) :
    JointState(robot_name, nb_joints) {
  this->set_type(StateType::JOINT_ACCELERATIONS);
}

JointAccelerations::JointAccelerations(const std::string& robot_name, const Eigen::VectorXd& accelerations) :
    JointState(robot_name, accelerations.size()) {
  this->set_type(StateType::JOINT_ACCELERATIONS);
  this->set_accelerations(accelerations);
}

JointAccelerations::JointAccelerations(const std::string& robot_name, const std::vector<std::string>& joint_names) :
    JointState(robot_name, joint_names) {
  this->set_type(StateType::JOINT_ACCELERATIONS);}

JointAccelerations::JointAccelerations(const std::string& robot_name,
                                       const std::vector<std::string>& joint_names,
                                       const Eigen::VectorXd& accelerations) : JointState(robot_name, joint_names) {
  this->set_type(StateType::JOINT_ACCELERATIONS);
  this->set_accelerations(accelerations);
}

JointAccelerations::JointAccelerations(const JointState& state) : JointState(state) {
  this->set_type(StateType::JOINT_ACCELERATIONS);
  if (state) {
    this->set_zero();
    this->set_accelerations(state.get_accelerations());
  }
}

JointAccelerations::JointAccelerations(const JointAccelerations& accelerations) :
    JointAccelerations(static_cast<const JointState&>(accelerations)) {}

JointAccelerations::JointAccelerations(const JointVelocities& velocities) :
    JointAccelerations(velocities.differentiate(1.0)) {}

JointAccelerations JointAccelerations::Zero(const std::string& robot_name, unsigned int nb_joints) {
  return JointState::Zero(robot_name, nb_joints);
}

JointAccelerations
JointAccelerations::Zero(const std::string& robot_name, const std::vector<std::string>& joint_names) {
  return JointState::Zero(robot_name, joint_names);
}

JointAccelerations JointAccelerations::Random(const std::string& robot_name, unsigned int nb_joints) {
  return JointAccelerations(robot_name, Eigen::VectorXd::Random(nb_joints));
}

JointAccelerations
JointAccelerations::Random(const std::string& robot_name, const std::vector<std::string>& joint_names) {
  return JointAccelerations(robot_name, joint_names, Eigen::VectorXd::Random(joint_names.size()));
}

Eigen::VectorXd JointAccelerations::data() const {
  return this->get_accelerations();
}

void JointAccelerations::set_data(const Eigen::VectorXd& data) {
  this->set_accelerations(data);
}

void JointAccelerations::set_data(const std::vector<double>& data) {
  this->set_accelerations(Eigen::VectorXd::Map(data.data(), data.size()));
}

void JointAccelerations::clamp(double max_absolute_value, double noise_ratio) {
  this->clamp_state_variable(max_absolute_value, JointStateVariable::ACCELERATIONS, noise_ratio);
}

void
JointAccelerations::clamp(const Eigen::ArrayXd& max_absolute_value_array, const Eigen::ArrayXd& noise_ratio_array) {
  this->clamp_state_variable(max_absolute_value_array, JointStateVariable::ACCELERATIONS, noise_ratio_array);
}

JointAccelerations JointAccelerations::clamped(double max_absolute_value, double noise_ratio) const {
  JointAccelerations result(*this);
  result.clamp(max_absolute_value, noise_ratio);
  return result;
}

JointAccelerations JointAccelerations::clamped(const Eigen::ArrayXd& max_absolute_value_array,
                                               const Eigen::ArrayXd& noise_ratio_array) const {
  JointAccelerations result(*this);
  result.clamp(max_absolute_value_array, noise_ratio_array);
  return result;
}

JointAccelerations JointAccelerations::copy() const {
  JointAccelerations result(*this);
  return result;
}

JointVelocities JointAccelerations::integrate(double dt) const {
  return JointVelocities(this->get_name(), this->get_names(), dt * this->get_accelerations());
}

JointVelocities JointAccelerations::integrate(const std::chrono::nanoseconds& dt) const {
  // convert the period to a double with the second as reference
  return this->integrate(dt.count() / 1e9);  
}

JointAccelerations& JointAccelerations::operator*=(double lambda) {
  this->JointState::operator*=(lambda);
  return (*this);
}

JointAccelerations JointAccelerations::operator*(double lambda) const {
  return this->JointState::operator*(lambda);
}

JointAccelerations operator*(double lambda, const JointAccelerations& accelerations) {
  JointAccelerations result(accelerations);
  result *= lambda;
  return result;
}

JointAccelerations operator*(const Eigen::MatrixXd& lambda, const JointAccelerations& accelerations) {
  JointAccelerations result(accelerations);
  result.multiply_state_variable(lambda, JointStateVariable::ACCELERATIONS);
  return result;
}

JointVelocities JointAccelerations::operator*(const std::chrono::nanoseconds& dt) const {
  return this->integrate(dt);
}

JointVelocities operator*(const std::chrono::nanoseconds& dt, const JointAccelerations& accelerations) {
  return accelerations.integrate(dt);
}

JointAccelerations& JointAccelerations::operator/=(double lambda) {
  this->JointState::operator/=(lambda);
  return (*this);
}

JointAccelerations JointAccelerations::operator/(double lambda) const {
  return this->JointState::operator/(lambda);
}

JointAccelerations& JointAccelerations::operator+=(const JointAccelerations& accelerations) {
  this->JointState::operator+=(accelerations);
  return (*this);
}

JointAccelerations& JointAccelerations::operator+=(const JointState& state) {
  this->JointState::operator+=(state);
  return (*this);
}

JointAccelerations JointAccelerations::operator+(const JointAccelerations& accelerations) const {
  return this->JointState::operator+(accelerations);
}

JointState JointAccelerations::operator+(const JointState& state) const {
  return this->JointState::operator+(state);
}

JointAccelerations JointAccelerations::operator-() const {
  return this->JointState::operator-();
}

JointAccelerations& JointAccelerations::operator-=(const JointAccelerations& accelerations) {
  this->JointState::operator-=(accelerations);
  return (*this);
}

JointAccelerations& JointAccelerations::operator-=(const JointState& state) {
  this->JointState::operator-=(state);
  return (*this);
}

JointAccelerations JointAccelerations::operator-(const JointAccelerations& accelerations) const {
  return this->JointState::operator-(accelerations);
}

JointState JointAccelerations::operator-(const JointState& state) const {
  return this->JointState::operator-(state);
}

std::ostream& operator<<(std::ostream& os, const JointAccelerations& accelerations) {
  os << accelerations.to_string();
  return os;
}
}// namespace state_representation
