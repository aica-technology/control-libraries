#include "state_representation/trajectory/JointTrajectory.hpp"
#include "state_representation/space/joint/JointState.hpp"

#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/IncompatibleStatesException.hpp"

namespace state_representation {

JointTrajectory::JointTrajectory(const std::string& name) : TrajectoryBase<JointTrajectoryPoint>(name) {
  this->set_type(StateType::JOINT_TRAJECTORY);
  this->reset();
}

JointTrajectory::JointTrajectory(
    const JointState& point, const std::chrono::nanoseconds& duration, const std::string& name
)
    : TrajectoryBase<JointTrajectoryPoint>(name) {
  this->set_type(StateType::JOINT_TRAJECTORY);
  this->reset();
  this->add_point(point, duration);
}

JointTrajectory::JointTrajectory(
    const std::vector<JointState>& points, const std::vector<std::chrono::nanoseconds>& durations,
    const std::string& name
)
    : TrajectoryBase<JointTrajectoryPoint>(name) {
  this->set_type(StateType::JOINT_TRAJECTORY);
  this->reset();
  this->set_points(points, durations);
}

void JointTrajectory::add_point(const JointState& new_point, const std::chrono::nanoseconds& duration) {
  if (new_point.is_empty()) {
    throw exceptions::EmptyStateException("Point is empty");
  }
  if (this->get_size() == 0) {
    this->joint_names_ = new_point.get_names();
    this->robot_name_ = new_point.get_name();
  } else if (this->joint_names_ != new_point.get_names()) {
    throw exceptions::IncompatibleStatesException(
        "Incompatible joint names between the new point and current trajectory"
    );
  } else if (this->robot_name_ != new_point.get_name()) {
    throw exceptions::IncompatibleStatesException("Incompatible robot name between the new point and current trajectory"
    );
  }
  JointTrajectoryPoint trajectory_point;
  trajectory_point.data = new_point.data();
  trajectory_point.duration = duration;
  this->TrajectoryBase<JointTrajectoryPoint>::add_point(trajectory_point);
}

void JointTrajectory::add_points(
    const std::vector<JointState>& new_points, const std::vector<std::chrono::nanoseconds>& durations
) {
  if (new_points.size() != durations.size()) {
    throw exceptions::IncompatibleSizeException("The size of the points and durations vectors are not equal");
  }
  for (unsigned int i = 0; i < new_points.size(); ++i) {
    add_point(new_points[i], durations[i]);
  }
}

void JointTrajectory::insert_point(
    const JointState& new_point, const std::chrono::nanoseconds& duration, unsigned int index
) {
  if (new_point.is_empty()) {
    throw exceptions::EmptyStateException("Point is empty");
  }
  if (this->get_size() == 0) {
    this->joint_names_ = new_point.get_names();
    this->robot_name_ = new_point.get_name();
  } else if (this->joint_names_ != new_point.get_names()) {
    throw exceptions::IncompatibleStatesException(
        "Incompatible joint names between the new point and current trajectory"
    );
  } else if (this->robot_name_ != new_point.get_name()) {
    throw exceptions::IncompatibleStatesException("Incompatible robot name between the new point and current trajectory"
    );
  }
  try {
    JointTrajectoryPoint trajectory_point;
    trajectory_point.data = new_point.data();
    trajectory_point.duration = duration;
    this->TrajectoryBase<JointTrajectoryPoint>::insert_point(trajectory_point, index);
  } catch (...) {
    throw;
  }
}

void JointTrajectory::set_point(const JointState& point, const std::chrono::nanoseconds& duration, unsigned int index) {
  if (point.is_empty()) {
    throw exceptions::EmptyStateException("Point is empty");
  } else if (point.get_names() != this->joint_names_) {
    throw exceptions::IncompatibleStatesException(
        "Incompatible joint names between the new point and current trajectory"
    );
  } else if (point.get_name() != this->robot_name_) {
    throw exceptions::IncompatibleStatesException("Incompatible robot name between the new point and current trajectory"
    );
  }
  JointTrajectoryPoint trajectory_point;
  trajectory_point.data = point.data();
  trajectory_point.duration = duration;
  try {
    this->TrajectoryBase<JointTrajectoryPoint>::set_point(trajectory_point, index);
  } catch (...) {
    throw;
  }
}

void JointTrajectory::set_points(
    const std::vector<JointState>& points, const std::vector<std::chrono::nanoseconds>& durations
) {
  if (points.size() != durations.size()) {
    throw exceptions::IncompatibleSizeException("The size of the points and durations vectors are not equal");
  }
  if (points.empty()) {
    throw exceptions::EmptyStateException("Points vector is empty");
  }
  auto candidate_robot_name = points[0].get_name();
  auto candidate_joint_names = points[0].get_names();

  std::vector<JointTrajectoryPoint> trajectory_points;
  for (unsigned int i = 0; i < points.size(); ++i) {
    if (points[i].is_empty()) {
      throw exceptions::EmptyStateException("Vector contains at least one point that is empty");
    } else if (points[i].get_names() != candidate_joint_names) {
      throw exceptions::IncompatibleStatesException(
          "Incompatible joint names between the new point and current trajectory"
      );
    } else if (points[i].get_name() != candidate_robot_name) {
      throw exceptions::IncompatibleStatesException(
          "Incompatible robot name between the new point and current trajectory"
      );
    }
    JointTrajectoryPoint trajectory_point;
    trajectory_point.data = points[i].data();
    trajectory_point.duration = durations[i];
    trajectory_points.push_back(trajectory_point);
  }
  try {
    this->TrajectoryBase<JointTrajectoryPoint>::set_points(trajectory_points);
    this->joint_names_ = candidate_joint_names;
    this->robot_name_ = candidate_robot_name;
  } catch (...) {
    throw;
  }
}

const std::vector<std::string>& JointTrajectory::get_joint_names() const {
  return this->joint_names_;
}

const std::deque<JointState> JointTrajectory::get_points() const {
  std::deque<JointState> points;
  for (unsigned int i = 0; i < this->get_size(); ++i) {
    auto state = this->operator[](i);
    points.push_back(state.first);
  }
  return points;
}

const JointState JointTrajectory::get_point(unsigned int index) const {
  try {
    return this->operator[](index).first;
  } catch (...) {
    throw;
  }
}

std::pair<JointState, const std::chrono::nanoseconds> JointTrajectory::operator[](unsigned int idx) const {
  try {
    auto point = this->TrajectoryBase<JointTrajectoryPoint>::operator[](idx);
    JointState state(this->robot_name_, this->joint_names_);
    state.set_data(point.data);
    auto duration = point.duration;
    return std::make_pair(state, duration);
  } catch (...) {
    throw;
  }
}

void JointTrajectory::reset() {
  this->TrajectoryBase<JointTrajectoryPoint>::reset();
  this->robot_name_ = "";
  this->joint_names_.clear();
}
}// namespace state_representation
