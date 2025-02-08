#include "state_representation/trajectory/CartesianTrajectory.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"

#include "state_representation/exceptions/IncompatibleReferenceFramesException.hpp"
#include <algorithm>

namespace state_representation {
CartesianTrajectory::CartesianTrajectory() : TrajectoryBase<CartesianTrajectoryPoint>() {
  this->set_type(StateType::CARTESIAN_TRAJECTORY);
}

CartesianTrajectory::CartesianTrajectory(const std::string& name, const std::string& reference_frame)
    : TrajectoryBase<CartesianTrajectoryPoint>(name), reference_frame_(reference_frame) {
  this->set_type(StateType::CARTESIAN_TRAJECTORY);
}

CartesianTrajectory::CartesianTrajectory(
    const std::string& name, const CartesianState& point, const std::chrono::nanoseconds& duration)
    : CartesianTrajectory(name, point.get_reference_frame()) {
  this->add_point(point, duration);
}

CartesianTrajectory::CartesianTrajectory(
    const std::string& name, const std::vector<CartesianState>& points,
    const std::vector<std::chrono::nanoseconds>& durations)
    : CartesianTrajectory(name) {
  this->assert_points_not_empty(points);
  this->reference_frame_ = points[0].get_reference_frame();
  this->add_points(points, durations);
}

CartesianTrajectory::CartesianTrajectory(const CartesianTrajectory& state)
    : CartesianTrajectory(state.get_name(), state.get_reference_frame()) {
  if (state) {
    this->add_points(state.get_points(), state.get_durations());
  }
}

const std::string& CartesianTrajectory::get_reference_frame() const {
  return this->reference_frame_;
}

void CartesianTrajectory::set_reference_frame(const CartesianPose& pose) {
  auto points = this->get_points();
  this->reference_frame_ = pose.get_reference_frame();
  std::transform(points.begin(), points.end(), points.begin(), [&](const auto& point) { return point * pose; });
  this->set_points(points, this->get_durations());
}

void CartesianTrajectory::set_reference_frame(const std::string& reference_frame) {
  this->reference_frame_ = reference_frame;
}

const std::vector<CartesianState> CartesianTrajectory::get_points() const {
  std::vector<CartesianState> points;
  auto queue = this->TrajectoryBase<CartesianTrajectoryPoint>::get_points();
  std::transform(queue.begin(), queue.end(), std::back_inserter(points), [&](const auto& point) {
    return point.to_cartesian_state(this->reference_frame_);
  });
  return points;
}

CartesianState CartesianTrajectory::get_point(unsigned int index) const {
  return this->TrajectoryBase<CartesianTrajectoryPoint>::get_point(index).to_cartesian_state(this->reference_frame_);
}

void CartesianTrajectory::add_point(const CartesianState& point, const std::chrono::nanoseconds& duration) {
  this->add_points({point}, {duration});
}

void CartesianTrajectory::add_points(
    const std::vector<CartesianState>& points, const std::vector<std::chrono::nanoseconds>& durations) {
  this->assert_points_not_empty(points);
  this->assert_points_durations_sizes_equal(points, durations);
  this->assert_not_contains_empty_state(points);
  this->assert_same_reference_frame(points, this->reference_frame_);
  for (unsigned int i = 0; i < points.size(); ++i) {
    this->TrajectoryBase<CartesianTrajectoryPoint>::add_point(CartesianTrajectoryPoint(points[i], durations[i]));
  }
}

void CartesianTrajectory::insert_point(
    const CartesianState& point, const std::chrono::nanoseconds& duration, unsigned int index) {
  this->assert_not_contains_empty_state<CartesianState>({point});
  this->assert_same_reference_frame({point}, this->reference_frame_);
  this->TrajectoryBase<CartesianTrajectoryPoint>::insert_point(CartesianTrajectoryPoint(point, duration), index);
}

void CartesianTrajectory::set_point(
    const CartesianState& point, const std::chrono::nanoseconds& duration, unsigned int index) {
  this->assert_not_contains_empty_state<CartesianState>({point});
  this->assert_same_reference_frame({point}, this->reference_frame_);
  this->TrajectoryBase<CartesianTrajectoryPoint>::set_point(CartesianTrajectoryPoint(point, duration), index);
}

void CartesianTrajectory::set_points(
    const std::vector<CartesianState>& points, const std::vector<std::chrono::nanoseconds>& durations) {
  this->assert_points_not_empty(points);
  this->assert_points_size(points);
  this->assert_points_durations_sizes_equal(points, durations);
  for (unsigned int i = 0; i < points.size(); ++i) {
    this->set_point(points[i], durations[i], i);
  }
}

std::pair<CartesianState, const std::chrono::nanoseconds> CartesianTrajectory::operator[](unsigned int idx) const {
  auto point = this->TrajectoryBase<CartesianTrajectoryPoint>::operator[](idx);
  return std::make_pair(point.to_cartesian_state(this->reference_frame_), point.duration);
}

CartesianTrajectory& CartesianTrajectory::operator=(const CartesianTrajectory& trajectory) {
  if (this != &trajectory) {
    this->reset();
    CartesianTrajectory tmp(trajectory);
    swap(*this, tmp);
  }
  return *this;
}

void CartesianTrajectory::assert_same_reference_frame(const std::vector<CartesianState>& states) const {
  if (!states.empty()) {
    assert_same_reference_frame(states, states[0].get_reference_frame());
  }
}

void CartesianTrajectory::assert_same_reference_frame(
    const std::vector<CartesianState>& states, const std::string& reference_frame) const {
  if (!std::ranges::all_of(states, [&](const auto& state) { return state.get_reference_frame() == reference_frame; })) {
    throw exceptions::IncompatibleReferenceFramesException(
        "Incompatible reference frame " + states.front().get_reference_frame() + " and " + reference_frame);
  }
}
}// namespace state_representation
