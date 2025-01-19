#include "state_representation/trajectory/CartesianTrajectory.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"

#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/IncompatibleReferenceFramesException.hpp"
#include "state_representation/exceptions/IncompatibleSizeException.hpp"

#include <eigen3/Eigen/src/Core/Matrix.h>

namespace state_representation {
CartesianTrajectory::CartesianTrajectory(const std::string& name, const std::string& reference_frame)
    : TrajectoryBase<CartesianTrajectoryPoint>(name), reference_frame_(reference_frame) {
  this->set_type(StateType::CARTESIAN_TRAJECTORY);
  this->reset();
}

const std::string CartesianTrajectory::get_reference_frame() const {
  return this->reference_frame_;
}

CartesianTrajectory::CartesianTrajectory(
    const CartesianState& point, const std::chrono::nanoseconds& duration, const std::string& name,
    const std::string& reference_frame)
    : TrajectoryBase<CartesianTrajectoryPoint>(name), reference_frame_(reference_frame) {
  this->set_type(StateType::CARTESIAN_TRAJECTORY);
  this->reset();
  CartesianTrajectoryPoint trajectory_point;
  trajectory_point.data = point.data();
  trajectory_point.duration = duration;
  trajectory_point.name = point.get_name();
  this->TrajectoryBase<CartesianTrajectoryPoint>::add_point(trajectory_point);
}

CartesianTrajectory::CartesianTrajectory(
    const std::vector<CartesianState>& points, const std::vector<std::chrono::nanoseconds>& durations,
    const std::string& name, const std::string& reference_frame)
    : TrajectoryBase<CartesianTrajectoryPoint>(name), reference_frame_(reference_frame) {
  this->set_type(StateType::CARTESIAN_TRAJECTORY);
  this->reset();
  try {
    this->set_points(points, durations);
  } catch (...) {
    throw;
  }
}

void CartesianTrajectory::add_point(const CartesianState& new_point, const std::chrono::nanoseconds& duration) {
  if (new_point.is_empty()) {
    throw exceptions::EmptyStateException("The Cartesian state provided is empty");
  }
  if (this->get_size() > 0) {
    if (new_point.get_reference_frame() != reference_frame_) {
      throw exceptions::IncompatibleReferenceFramesException(
          "Incompatible reference frames: " + new_point.get_reference_frame() + " and " + reference_frame_);
    }
  } else {
    reference_frame_ = new_point.get_reference_frame();
  }
  CartesianTrajectoryPoint trajectory_point;
  trajectory_point.data = new_point.data();
  trajectory_point.duration = duration;
  trajectory_point.name = new_point.get_name();
  this->TrajectoryBase<CartesianTrajectoryPoint>::add_point(trajectory_point);
}

void CartesianTrajectory::insert_point(
    const CartesianState& new_point, const std::chrono::nanoseconds& duration, unsigned int pos) {
  if (new_point.is_empty()) {
    throw exceptions::EmptyStateException("Point is empty");
  }
  if (this->get_size() > 0) {
    if (new_point.get_reference_frame() != reference_frame_) {
      throw exceptions::IncompatibleReferenceFramesException(
          "Incompatible reference frames: " + new_point.get_reference_frame() + " and " + reference_frame_);
    }
  } else if (reference_frame_.empty()) {
    reference_frame_ = new_point.get_reference_frame();
  }
  CartesianTrajectoryPoint trajectory_point;
  trajectory_point.data = new_point.data();
  trajectory_point.duration = duration;
  trajectory_point.name = new_point.get_name();
  try {
    this->TrajectoryBase<CartesianTrajectoryPoint>::insert_point(trajectory_point, pos);
  } catch (...) {
    throw;
  }
}

void CartesianTrajectory::set_point(
    const CartesianState& point, const std::chrono::nanoseconds& duration, unsigned int index) {
  if (point.is_empty()) {
    throw exceptions::EmptyStateException("Point is empty");
  }
  if (point.get_reference_frame() != reference_frame_) {
    throw exceptions::IncompatibleReferenceFramesException(
        "Incompatible reference frames: " + point.get_reference_frame() + " and " + reference_frame_);
  }
  CartesianTrajectoryPoint trajectory_point;
  trajectory_point.data = point.data();
  trajectory_point.duration = duration;
  trajectory_point.name = point.get_name();
  try {
    this->TrajectoryBase<CartesianTrajectoryPoint>::set_point(trajectory_point, index);
  } catch (...) {
    throw;
  }
}

void CartesianTrajectory::set_points(
    const std::vector<CartesianState>& points, const std::vector<std::chrono::nanoseconds>& durations) {
  if (points.size() != durations.size()) {
    throw exceptions::IncompatibleSizeException("The size of the points and durations vectors are not equal");
  }
  if (points.empty()) {
    this->reset();
    return;
  }
  std::string candidate_reference_frame = points[0].get_reference_frame();

  std::vector<CartesianTrajectoryPoint> trajectory_points;
  for (unsigned int i = 0; i < points.size(); ++i) {
    if (points[i].is_empty()) {
      throw exceptions::EmptyStateException("Vector contains at least one point that is empty");
    } else if (points[i].get_reference_frame() != candidate_reference_frame) {
      throw exceptions::IncompatibleReferenceFramesException(
          "Incompatible reference frames: " + points[i].get_reference_frame() + " and " + candidate_reference_frame);
    }
    CartesianTrajectoryPoint trajectory_point;
    trajectory_point.data = points[i].data();
    trajectory_point.duration = durations[i];
    trajectory_point.name = points[i].get_name();
    trajectory_points.push_back(trajectory_point);
  }
  try {
    this->TrajectoryBase<CartesianTrajectoryPoint>::set_points(trajectory_points);
    reference_frame_ = candidate_reference_frame;
  } catch (...) {
    throw;
  }
}

const std::deque<CartesianState> CartesianTrajectory::get_points() const {
  std::deque<CartesianState> points;
  for (unsigned int i = 0; i < this->get_size(); ++i) {
    auto state = this->operator[](i);
    points.push_back(state.first);
  }
  return points;
}

CartesianState CartesianTrajectory::get_point(unsigned int index) const {
  try {
    return this->operator[](index).first;
  } catch (...) {
    throw;
  }
}

std::pair<CartesianState, const std::chrono::nanoseconds> CartesianTrajectory::operator[](unsigned int idx) const {
  try {
    auto point = this->TrajectoryBase<CartesianTrajectoryPoint>::operator[](idx);
    CartesianState state;
    state.set_data(point.data);
    state.set_name(point.name);
    state.set_reference_frame(this->reference_frame_);
    auto duration = point.duration;
    return std::make_pair(state, duration);
  } catch (...) {
    throw;
  }
}

void CartesianTrajectory::reset() {
  this->TrajectoryBase<CartesianTrajectoryPoint>::reset();
  this->reference_frame_ = "world";
}

void CartesianTrajectory::delete_point() {
  this->TrajectoryBase<CartesianTrajectoryPoint>::delete_point();
  if (this->get_size() == 0) {
    this->clear();
  }
}

void CartesianTrajectory::clear() {
  this->TrajectoryBase<CartesianTrajectoryPoint>::clear();
  this->reference_frame_ = "";
}
}// namespace state_representation
