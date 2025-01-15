#include "state_representation/trajectory/JointTrajectory.hpp"
#include "state_representation/space/joint/JointState.hpp"

namespace state_representation {

JointTrajectory::JointTrajectory() : TrajectoryBase<Eigen::VectorXd>() {
  this->set_type(StateType::JOINT_TRAJECTORY);
  this->reset();
}

JointTrajectory::JointTrajectory(const std::string& name) : TrajectoryBase<Eigen::VectorXd>(name) {
  this->set_type(StateType::JOINT_TRAJECTORY);
  this->reset();
}

const std::vector<std::string>& JointTrajectory::get_joint_names() const {
  return this->joint_names_;
}

const std::deque<JointState> JointTrajectory::get_points() const {
  std::deque<JointState> points;
  for (const auto& point : this->TrajectoryBase<Eigen::VectorXd>::get_points()) {
    JointState state(this->robot_name_, point.size() / 4);
    state.set_data(point);
    points.push_back(state);
  }
  return points;
}

const JointState JointTrajectory::get_point(unsigned int index) const {
  auto point = this->TrajectoryBase<Eigen::VectorXd>::get_point(index);
  JointState state(this->robot_name_, point.size() / 4);
  state.set_data(point);
  return state;
}

std::pair<JointState, std::chrono::nanoseconds> JointTrajectory::operator[](unsigned int idx) {
  auto pair = this->TrajectoryBase<Eigen::VectorXd>::operator[](idx);
  JointState state(this->robot_name_, pair.first.size() / 4);
  state.set_data(pair.first);
  return std::make_pair(state, pair.second);
}

void JointTrajectory::reset() {
  this->TrajectoryBase<Eigen::VectorXd>::reset();
  this->robot_name_ = "";
  this->joint_names_.clear();
}

void JointTrajectory::delete_point() {
  this->TrajectoryBase<Eigen::VectorXd>::delete_point();
  if (this->get_size() == 0) {
    this->joint_names_.clear();
  }
}

void JointTrajectory::clear() {
  this->TrajectoryBase<Eigen::VectorXd>::clear();
  this->joint_names_.clear();
}
}// namespace state_representation
