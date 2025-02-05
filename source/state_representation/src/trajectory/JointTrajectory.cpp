#include <algorithm>

#include "state_representation/space/joint/JointState.hpp"
#include "state_representation/trajectory/JointTrajectory.hpp"

#include "state_representation/exceptions/IncompatibleStatesException.hpp"

namespace state_representation {

JointTrajectory::JointTrajectory(const std::string& name) : TrajectoryBase<JointTrajectoryPoint>(name) {
  this->set_type(StateType::JOINT_TRAJECTORY);
}

JointTrajectory::JointTrajectory(
    const std::string& name, const JointState& point, const std::chrono::nanoseconds& duration
)
    : JointTrajectory(name) {
  this->assert_points_not_empty<JointState>({point});
  this->joint_names_ = point.get_names();
  this->add_point(point, duration);
}

JointTrajectory::JointTrajectory(
    const std::string& name, const std::vector<JointState>& points,
    const std::vector<std::chrono::nanoseconds>& durations
)
    : JointTrajectory(name) {
  this->assert_points_not_empty(points);
  this->joint_names_ = points[0].get_names();
  this->add_points(points, durations);
}

const std::vector<std::string>& JointTrajectory::get_joint_names() const {
  return this->joint_names_;
}

void JointTrajectory::set_joint_names(const std::vector<std::string>& joint_names) {
  this->joint_names_ = joint_names;
}

void JointTrajectory::add_point(const JointState& point, const std::chrono::nanoseconds& duration) {
  this->add_points({point}, {duration});
}

void JointTrajectory::add_points(
    const std::vector<JointState>& points, const std::vector<std::chrono::nanoseconds>& durations
) {
  this->assert_points_not_empty(points);
  this->assert_points_durations_sizes_equal(points, durations);
  this->assert_not_contains_empty_state(points);
  this->assert_compatible_joint_names(points, this->joint_names_);
  for (unsigned int i = 0; i < points.size(); ++i) {
    this->TrajectoryBase<JointTrajectoryPoint>::add_point(JointTrajectoryPoint(points[i], durations[i]));
  }
}

void JointTrajectory::insert_point(
    const JointState& point, const std::chrono::nanoseconds& duration, unsigned int index
) {
  this->assert_not_contains_empty_state<JointState>({point});
  this->assert_compatible_joint_names({point}, this->joint_names_);
  this->TrajectoryBase<JointTrajectoryPoint>::insert_point(JointTrajectoryPoint(point, duration), index);
}

void JointTrajectory::set_point(const JointState& point, const std::chrono::nanoseconds& duration, unsigned int index) {
  this->assert_not_contains_empty_state<JointState>({point});
  this->assert_compatible_joint_names({point}, this->joint_names_);
  this->TrajectoryBase<JointTrajectoryPoint>::set_point(JointTrajectoryPoint(point, duration), index);
}

void JointTrajectory::set_points(
    const std::vector<JointState>& points, const std::vector<std::chrono::nanoseconds>& durations
) {
  this->assert_points_not_empty(points);
  this->assert_points_size(points);
  this->assert_points_durations_sizes_equal(points, durations);
  for (unsigned int i = 0; i < points.size(); ++i) {
    this->set_point(points[i], durations[i], i);
  }
}

const std::vector<JointState> JointTrajectory::get_points() const {
  std::vector<JointState> points;
  auto queue = this->TrajectoryBase<JointTrajectoryPoint>::get_points();
  std::transform(queue.begin(), queue.end(), std::back_inserter(points), [&](const auto& point) {
    return point.to_joint_state(this->joint_names_);
  });
  return points;
}

const JointState JointTrajectory::get_point(unsigned int index) const {
  return this->TrajectoryBase<JointTrajectoryPoint>::get_point(index).to_joint_state(this->joint_names_);
}

std::pair<JointState, const std::chrono::nanoseconds> JointTrajectory::operator[](unsigned int idx) const {
  auto point = this->TrajectoryBase<JointTrajectoryPoint>::operator[](idx);
  return std::make_pair(point.to_joint_state(this->joint_names_), point.duration);
}

void JointTrajectory::assert_compatible_joint_names(const std::vector<JointState>& states) const {
  if (!states.empty()) {
    this->assert_compatible_joint_names(states, states[0].get_names());
  }
}

void JointTrajectory::assert_compatible_joint_names(
    const std::vector<JointState>& states, const std::vector<std::string>& joint_names
) const {
  if (!std::ranges::all_of(states, [&](const auto& state) { return state.get_names() == joint_names; })) {
    throw exceptions::IncompatibleStatesException("Incompatible joint names");
  }
}
}// namespace state_representation
