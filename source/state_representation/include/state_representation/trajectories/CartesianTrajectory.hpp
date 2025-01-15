#pragma once

#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/trajectories/TrajectoryBase.hpp"

namespace state_representation {
class CartesianTrajectory : public TrajectoryBase<Eigen::VectorXd> {
private:
  std::string reference_frame_;///< name of the reference frame

public:
  /**
   * @brief Constructor with name and reference frame provided
   * @param reference_frame reference frame of the trajectory points
   * @param name the name of the state
   */
  explicit CartesianTrajectory(const std::string& reference_frame = "world", const std::string& name = "");

  /**
   * @brief Getter of the reference frame as const reference
   */
  const std::string get_reference_frame() const;

  /**
   * @brief Add new point and corresponding time to trajectory
   * @return Success of the operation
   */
  template<typename DurationT>
  bool add_point(const CartesianState& new_point, const std::chrono::duration<int64_t, DurationT>& new_time);

  /**
   * @brief Insert new point and corresponding time to trajectory between two
   * already existing points
   * @return Success of the operation
   */
  template<typename DurationT>
  bool
  insert_point(const CartesianState& new_point, const std::chrono::duration<int64_t, DurationT>& new_time, int pos);

  /**
   * @brief Get attribute list of trajectory points
   */
  const std::deque<CartesianState> get_points() const;

  /**
   * @brief Get the trajectory point at given index
   * @param index the index
   */
  CartesianState get_point(unsigned int index) const;

  /**
   * @brief Set the trajectory point at given index
   * @param index the index
   * @param point the new point
   * @param new_time the new time
   * @return Success of the operation
   */
  template<typename DurationT>
  bool
  set_point(unsigned int index, const CartesianState& point, const std::chrono::duration<int64_t, DurationT>& new_time);

  /**
   * @brief Set the trajectory point at given index
   * @param points vector of new points
   * @param new_time vector of new times
   * @return Success of the operation
   */
  template<typename DurationT>
  bool set_points(
      const std::vector<CartesianState>& points,
      const std::vector<std::chrono::duration<int64_t, DurationT>>& new_times);

  /**
   * @brief Operator overload for returning a single trajectory point and
   * corresponding time
   */
  std::pair<CartesianState, std::chrono::nanoseconds> operator[](unsigned int idx);

  /**
   * @brief Reset trajectory
   */
  virtual void reset() override;

  /**
   * @brief Delete last point and corresponding time from trajectory
   */
  virtual void delete_point() override;

  /**
   * @brief Clear trajectory
   */
  virtual void clear() override;
};

template<typename DurationT>
inline bool CartesianTrajectory::add_point(
    const CartesianState& new_point, const std::chrono::duration<int64_t, DurationT>& new_time) {
  if (new_point.is_empty()) {
    return false;
  }
  if (this->get_size() > 0) {
    if (new_point.get_reference_frame() != reference_frame_) {
      return false;
    }
  } else if (reference_frame_.empty()) {
    reference_frame_ = new_point.get_reference_frame();
  }
  this->TrajectoryBase<Eigen::VectorXd>::add_point(new_point.data(), new_time);
  return true;
}

template<typename DurationT>
inline bool CartesianTrajectory::insert_point(
    const CartesianState& new_point, const std::chrono::duration<int64_t, DurationT>& new_time, int pos) {
  if (new_point.is_empty()) {
    return false;
  }
  if (this->get_size() > 0) {
    if (new_point.get_reference_frame() != reference_frame_) {
      return false;
    }
  } else if (reference_frame_.empty()) {
    reference_frame_ = new_point.get_reference_frame();
  }
  this->TrajectoryBase<Eigen::VectorXd>::insert_point(new_point.data(), new_time, pos);
  return true;
}

template<typename DurationT>
inline bool CartesianTrajectory::set_point(
    unsigned int index, const CartesianState& point, const std::chrono::duration<int64_t, DurationT>& new_time) {
  if (point.is_empty()) {
    return false;
  }
  if (point.get_reference_frame() != reference_frame_) {
    return false;
  }
  return this->TrajectoryBase<Eigen::VectorXd>::set_point(index, point.data(), new_time);
}

template<typename DurationT>
inline bool CartesianTrajectory::set_points(
    const std::vector<CartesianState>& points,
    const std::vector<std::chrono::duration<int64_t, DurationT>>& new_times) {
  if (points.empty()) {
    return false;
  }
  std::string candidate_reference_frame = points[0].get_reference_frame();

  std::vector<Eigen::VectorXd> data;
  for (const auto& point : points) {
    if (point.is_empty() || (point.get_reference_frame() != candidate_reference_frame)) {
      return false;
    }
    data.push_back(point.data());
  }
  if (this->TrajectoryBase<Eigen::VectorXd>::set_points(data, new_times)) {
    reference_frame_ = candidate_reference_frame;
    return true;
  } else {
    return false;
  }
}

}// namespace state_representation
