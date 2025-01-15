#include "state_representation/trajectories/CartesianTrajectory.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"

#include <eigen3/Eigen/src/Core/Matrix.h>

namespace state_representation {
CartesianTrajectory::CartesianTrajectory(const std::string& reference_frame, const std::string& name)
    : TrajectoryBase<Eigen::VectorXd>(name), reference_frame_(reference_frame) {
  this->set_type(StateType::CARTESIAN_TRAJECTORY);
  this->reset();
}

const std::string CartesianTrajectory::get_reference_frame() const {
  return this->reference_frame_;
}

const std::deque<CartesianState> CartesianTrajectory::get_points() const {
  std::deque<CartesianState> points;
  for (const auto& point : this->TrajectoryBase<Eigen::VectorXd>::get_points()) {
    CartesianState state;
    state.set_data(point);
    points.push_back(state);
  }
  return points;
}

CartesianState CartesianTrajectory::get_point(unsigned int index) const {
  auto point = this->TrajectoryBase<Eigen::VectorXd>::get_point(index);
  CartesianState state;
  state.set_data(point);
  return state;
}

std::pair<CartesianState, std::chrono::nanoseconds> CartesianTrajectory::operator[](unsigned int idx) {
  auto pair = this->TrajectoryBase<Eigen::VectorXd>::operator[](idx);
  CartesianState state;
  state.set_data(pair.first);
  return std::make_pair(state, pair.second);
}
}// namespace state_representation
