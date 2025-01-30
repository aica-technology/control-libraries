#include "state_representation/trajectory/JointTrajectory.hpp"
#include "state_representation/space/joint/JointState.hpp"

#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/IncompatibleStatesException.hpp"

namespace state_representation {

JointTrajectory::JointTrajectory(const std::string& name) : TrajectoryBase<JointTrajectoryPoint>(name) {
  this->set_type(StateType::JOINT_TRAJECTORY);
}

JointTrajectory::JointTrajectory(
    const std::string& name, const JointState& point, const std::chrono::nanoseconds& duration
)
    : TrajectoryBase<JointTrajectoryPoint>(name) {
  this->set_type(StateType::JOINT_TRAJECTORY);
  if (point.is_empty()) {
    throw exceptions::EmptyStateException("Point is empty");
  }
  this->joint_names_ = point.get_names();
  this->add_point(point, duration);
}

JointTrajectory::JointTrajectory(
    const std::string& name, const std::vector<JointState>& points,
    const std::vector<std::chrono::nanoseconds>& durations
)
    : TrajectoryBase<JointTrajectoryPoint>(name) {
  this->set_type(StateType::JOINT_TRAJECTORY);
  for (unsigned int i = 1; i < points.size(); ++i) {
    if (points[i - 1].is_empty()) {
      throw exceptions::EmptyStateException("Vector contains at least one point that is empty");
    } else if (points[i - 1].get_names() != points[i].get_names()) {
      throw exceptions::IncompatibleStatesException("Incompatible joint names within the new points vector");
    }
  }
  if (points.size() > 0) {
    this->joint_names_ = points[0].get_names();
  }
  this->add_points(points, durations);
}

void JointTrajectory::add_point(const JointState& new_point, const std::chrono::nanoseconds& duration) {
  if (new_point.is_empty()) {
    throw exceptions::EmptyStateException("Point is empty");
  } else if (this->joint_names_ != new_point.get_names()) {
    throw exceptions::IncompatibleStatesException(
        "Incompatible joint names between the new point and current trajectory"
    );
  }
  JointTrajectoryPoint trajectory_point;
  trajectory_point.data = new_point.data();
  trajectory_point.duration = duration;
  trajectory_point.name = new_point.get_name();
  this->TrajectoryBase<JointTrajectoryPoint>::add_point(trajectory_point);
}

void JointTrajectory::add_points(
    const std::vector<JointState>& new_points, const std::vector<std::chrono::nanoseconds>& durations
) {
  if (new_points.size() != durations.size()) {
    throw exceptions::IncompatibleSizeException("The size of the points and durations vectors are not equal");
  }
  for (unsigned int i = 1; i < new_points.size(); ++i) {
    if (new_points[i - 1].is_empty()) {
      throw exceptions::EmptyStateException("Vector contains at least one point that is empty");
    } else if (new_points[i - 1].get_names() != new_points[i].get_names()) {
      throw exceptions::IncompatibleStatesException("Incompatible joint names within the new points vector");
    }
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
  } else if (this->joint_names_ != new_point.get_names()) {
    throw exceptions::IncompatibleStatesException(
        "Incompatible joint names between the new point and current trajectory"
    );
  }
  try {
    JointTrajectoryPoint trajectory_point;
    trajectory_point.data = new_point.data();
    trajectory_point.duration = duration;
    trajectory_point.name = new_point.get_name();
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
  }
  JointTrajectoryPoint trajectory_point;
  trajectory_point.data = point.data();
  trajectory_point.duration = duration;
  trajectory_point.name = point.get_name();
  try {
    this->TrajectoryBase<JointTrajectoryPoint>::set_point(trajectory_point, index);
  } catch (...) {
    throw;
  }
}

void JointTrajectory::set_points(
    const std::vector<JointState>& points, const std::vector<std::chrono::nanoseconds>& durations
) {
  if (points.empty()) {
    throw exceptions::EmptyStateException("Points vector is empty");
  } else if (points.size() != this->get_size()) {
    throw exceptions::IncompatibleSizeException("The size of the current vector and the new vector are not equal");
  } else if (points.size() != durations.size()) {
    throw exceptions::IncompatibleSizeException("The size of the points and durations vectors are not equal");
  }

  std::vector<JointTrajectoryPoint> trajectory_points;
  for (unsigned int i = 0; i < points.size(); ++i) {
    if (points[i].is_empty()) {
      throw exceptions::EmptyStateException("Vector contains at least one point that is empty");
    } else if (points[i].get_names() != this->joint_names_) {
      throw exceptions::IncompatibleStatesException(
          "Incompatible joint names between the new point and current trajectory"
      );
    }
    JointTrajectoryPoint trajectory_point;
    trajectory_point.data = points[i].data();
    trajectory_point.duration = durations[i];
    trajectory_point.name = points[i].get_name();
    trajectory_points.push_back(trajectory_point);
  }
  try {
    this->TrajectoryBase<JointTrajectoryPoint>::set_points(trajectory_points);
  } catch (...) {
    throw;
  }
}

const std::vector<std::string>& JointTrajectory::get_joint_names() const {
  return this->joint_names_;
}

void JointTrajectory::set_joint_names(const std::vector<std::string>& joint_names) {
  this->joint_names_ = joint_names;
}

const std::vector<JointState> JointTrajectory::get_points() const {
  std::vector<JointState> points;
  for (unsigned int i = 0; i < this->get_size(); ++i) {
    points.push_back(this->operator[](i).first);
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
    JointState state(point.name, this->joint_names_);
    state.set_data(point.data);
    auto duration = point.duration;
    return std::make_pair(state, duration);
  } catch (...) {
    throw;
  }
}
}// namespace state_representation
