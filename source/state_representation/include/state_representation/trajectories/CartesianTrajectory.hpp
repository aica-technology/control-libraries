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

}// namespace state_representation
