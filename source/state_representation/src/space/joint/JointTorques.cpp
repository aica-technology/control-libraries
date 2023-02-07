#include "state_representation/space/joint/JointTorques.hpp"

namespace state_representation {

using namespace exceptions;

JointTorques::JointTorques() {
  this->set_type(StateType::JOINT_TORQUES);
}

JointTorques::JointTorques(const std::string& robot_name, unsigned int nb_joints) : JointState(robot_name, nb_joints) {
  this->set_type(StateType::JOINT_TORQUES);
}

JointTorques::JointTorques(const std::string& robot_name, const Eigen::VectorXd& torques) :
    JointState(robot_name, torques.size()) {
  this->set_type(StateType::JOINT_TORQUES);
  this->set_torques(torques);
}

JointTorques::JointTorques(const std::string& robot_name, const std::vector<std::string>& joint_names) :
    JointState(robot_name, joint_names) {
  this->set_type(StateType::JOINT_TORQUES);
}

JointTorques::JointTorques(const std::string& robot_name, const std::vector<std::string>& joint_names,
                           const Eigen::VectorXd& torques) : JointState(robot_name, joint_names) {
  this->set_type(StateType::JOINT_TORQUES);
  this->set_torques(torques);
}

JointTorques::JointTorques(const JointState& state) : JointState(state) {
  // set all the state variables to 0 except torques
  this->set_type(StateType::JOINT_TORQUES);
  this->set_zero();
  this->set_torques(state.get_torques());
  this->set_empty(state.is_empty());
}

JointTorques::JointTorques(const JointTorques& torques) : JointTorques(static_cast<const JointState&>(torques)) {}

JointTorques JointTorques::Zero(const std::string& robot_name, unsigned int nb_joints) {
  return JointState::Zero(robot_name, nb_joints);
}

JointTorques JointTorques::Zero(const std::string& robot_name, const std::vector<std::string>& joint_names) {
  return JointState::Zero(robot_name, joint_names);
}

JointTorques JointTorques::Random(const std::string& robot_name, unsigned int nb_joints) {
  return JointTorques(robot_name, Eigen::VectorXd::Random(nb_joints));
}

JointTorques JointTorques::Random(const std::string& robot_name, const std::vector<std::string>& joint_names) {
  return JointTorques(robot_name, joint_names, Eigen::VectorXd::Random(joint_names.size()));
}

Eigen::VectorXd JointTorques::data() const {
  return this->get_torques();
}

void JointTorques::set_data(const Eigen::VectorXd& data) {
  this->set_torques(data);
}

void JointTorques::set_data(const std::vector<double>& data) {
  this->set_torques(Eigen::VectorXd::Map(data.data(), data.size()));
}

void JointTorques::clamp(double max_absolute_value, double noise_ratio) {
  this->clamp_state_variable(max_absolute_value, JointStateVariable::TORQUES, noise_ratio);
}

void JointTorques::clamp(const Eigen::ArrayXd& max_absolute_value_array, const Eigen::ArrayXd& noise_ratio_array) {
  this->clamp_state_variable(max_absolute_value_array, JointStateVariable::TORQUES, noise_ratio_array);
}

JointTorques JointTorques::clamped(double max_absolute_value, double noise_ratio) const {
  JointTorques result(*this);
  result.clamp(max_absolute_value, noise_ratio);
  return result;
}

JointTorques JointTorques::clamped(const Eigen::ArrayXd& max_absolute_value_array,
                                   const Eigen::ArrayXd& noise_ratio_array) const {
  JointTorques result(*this);
  result.clamp(max_absolute_value_array, noise_ratio_array);
  return result;
}

JointTorques JointTorques::copy() const {
  JointTorques result(*this);
  return result;
}

JointTorques& JointTorques::operator*=(double lambda) {
  this->JointState::operator*=(lambda);
  return (*this);
}

JointTorques JointTorques::operator*(double lambda) const {
  return this->JointState::operator*(lambda);
}

JointTorques operator*(double lambda, const JointTorques& torques) {
  JointTorques result(torques);
  result *= lambda;
  return result;
}

JointTorques operator*(const Eigen::MatrixXd& lambda, const JointTorques& torques) {
  JointTorques result(torques);
  result.multiply_state_variable(lambda, JointStateVariable::TORQUES);
  return result;
}

JointTorques& JointTorques::operator/=(double lambda) {
  this->JointState::operator/=(lambda);
  return (*this);
}

JointTorques JointTorques::operator/(double lambda) const {
  return this->JointState::operator/(lambda);
}

JointTorques& JointTorques::operator+=(const JointTorques& torques) {
  this->JointState::operator+=(torques);
  return (*this);
}

JointTorques& JointTorques::operator+=(const JointState& state) {
  this->JointState::operator+=(state);
  return (*this);
}

JointTorques JointTorques::operator+(const JointTorques& torques) const {
  return this->JointState::operator+(torques);
}

JointState JointTorques::operator+(const JointState& state) const {
  return this->JointState::operator+(state);
}

JointTorques JointTorques::operator-() const {
  return this->JointState::operator-();
}

JointTorques& JointTorques::operator-=(const JointTorques& torques) {
  this->JointState::operator-=(torques);
  return (*this);
}

JointTorques& JointTorques::operator-=(const JointState& state) {
  this->JointState::operator-=(state);
  return (*this);
}

JointTorques JointTorques::operator-(const JointTorques& torques) const {
  return this->JointState::operator-(torques);
}

JointState JointTorques::operator-(const JointState& state) const {
  return this->JointState::operator-(state);
}

std::ostream& operator<<(std::ostream& os, const JointTorques& torques) {
  os << torques.to_string();
  return os;
}
}// namespace state_representation
