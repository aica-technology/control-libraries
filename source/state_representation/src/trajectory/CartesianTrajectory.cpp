#include "state_representation/trajectory/CartesianTrajectory.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"

#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/IncompatibleReferenceFramesException.hpp"
#include "state_representation/exceptions/IncompatibleSizeException.hpp"

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
    const std::string& reference_frame
)
    : TrajectoryBase<CartesianTrajectoryPoint>(name), reference_frame_(reference_frame) {
  this->set_type(StateType::CARTESIAN_TRAJECTORY);
  this->reset();
  this->add_point(point, duration);
}

CartesianTrajectory::CartesianTrajectory(
    const std::vector<CartesianState>& points, const std::vector<std::chrono::nanoseconds>& durations,
    const std::string& name, const std::string& reference_frame
)
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
    if (new_point.get_reference_frame() != this->reference_frame_) {
      throw exceptions::IncompatibleReferenceFramesException(
          "Incompatible reference frames: " + new_point.get_reference_frame() + " and " + this->reference_frame_
      );
    }
  } else {
    this->reference_frame_ = new_point.get_reference_frame();
  }
  CartesianTrajectoryPoint trajectory_point;
  trajectory_point.data = new_point.data();
  trajectory_point.duration = duration;
  trajectory_point.name = new_point.get_name();
  this->TrajectoryBase<CartesianTrajectoryPoint>::add_point(trajectory_point);
}

void CartesianTrajectory::add_points(
    const std::vector<CartesianState>& new_points, const std::vector<std::chrono::nanoseconds>& durations
) {
  if (new_points.size() != durations.size()) {
    throw exceptions::IncompatibleSizeException("The size of the points and durations vectors are not equal");
  }
  for (unsigned int i = 0; i < new_points.size(); ++i) {
    add_point(new_points[i], durations[i]);
  }
}

void CartesianTrajectory::insert_point(
    const CartesianState& new_point, const std::chrono::nanoseconds& duration, unsigned int index
) {
  if (new_point.is_empty()) {
    throw exceptions::EmptyStateException("Point is empty");
  }
  if (this->get_size() > 0) {
    if (new_point.get_reference_frame() != this->reference_frame_) {
      throw exceptions::IncompatibleReferenceFramesException(
          "Incompatible reference frames: " + new_point.get_reference_frame() + " and " + this->reference_frame_
      );
    }
  } else if (this->reference_frame_.empty()) {
    this->reference_frame_ = new_point.get_reference_frame();
  }
  CartesianTrajectoryPoint trajectory_point;
  trajectory_point.data = new_point.data();
  trajectory_point.duration = duration;
  trajectory_point.name = new_point.get_name();
  try {
    this->TrajectoryBase<CartesianTrajectoryPoint>::insert_point(trajectory_point, index);
  } catch (...) {
    throw;
  }
}

void CartesianTrajectory::set_point(
    const CartesianState& point, const std::chrono::nanoseconds& duration, unsigned int index
) {
  if (point.is_empty()) {
    throw exceptions::EmptyStateException("Point is empty");
  }
  if (point.get_reference_frame() != this->reference_frame_) {
    throw exceptions::IncompatibleReferenceFramesException(
        "Incompatible reference frames: " + point.get_reference_frame() + " and " + this->reference_frame_
    );
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
    const std::vector<CartesianState>& points, const std::vector<std::chrono::nanoseconds>& durations
) {
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
          "Incompatible reference frames: " + points[i].get_reference_frame() + " and " + candidate_reference_frame
      );
    }
    CartesianTrajectoryPoint trajectory_point;
    trajectory_point.data = points[i].data();
    trajectory_point.duration = durations[i];
    trajectory_point.name = points[i].get_name();
    trajectory_points.push_back(trajectory_point);
  }
  try {
    this->TrajectoryBase<CartesianTrajectoryPoint>::set_points(trajectory_points);
    this->reference_frame_ = candidate_reference_frame;
  } catch (...) {
    throw;
  }
}

const std::vector<CartesianState> CartesianTrajectory::get_points() const {
  std::vector<CartesianState> points;
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
    CartesianState state(point.name, this->reference_frame_);
    state.set_data(point.data);
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
}// namespace state_representation
