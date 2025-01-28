#pragma once

#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/trajectory/TrajectoryBase.hpp"

namespace state_representation {

/**
 * @class CartesianTrajectoryPoint
 * @brief Struct to represent a Cartesian trajectory point
 */
struct CartesianTrajectoryPoint : public TrajectoryPoint {
  std::string name;
};

/**
 * @class CartesianTrajectory
 * @brief Class to represent a trajectory of Cartesian points and corresponding durations
 */
class CartesianTrajectory : public TrajectoryBase<CartesianTrajectoryPoint> {
public:
  /**
   * @brief Constructor with name and reference frame provided
   * @param name the name of the state
   * @param reference_frame reference frame of the trajectory points
   */
  explicit CartesianTrajectory(const std::string& name = "", const std::string& reference_frame = "world");

  /**
   * @brief Constructor with initial point, duration, name, and reference frame provided
   * @param point the initial point
   * @param duration the initial duration
   * @param name the name of the state
   * @param reference_frame reference frame of the trajectory points
   */
  explicit CartesianTrajectory(
      const CartesianState& point, const std::chrono::nanoseconds& duration, const std::string& name = "",
      const std::string& reference_frame = "world"
  );

  /**
   * @brief Constructor with intial points, durations, name and reference frame provided
   * @param points vector of initial points
   * @param durations vector of initial durations
   * @param name the name of the state
   * @param reference_frame reference frame of the trajectory points
   */
  explicit CartesianTrajectory(
      const std::vector<CartesianState>& points, const std::vector<std::chrono::nanoseconds>& durations,
      const std::string& name = "", const std::string& reference_frame = "world"
  );

  /**
   * @brief Getter of the reference frame as const reference
   * @return the reference frame associated with the trajectory
   */
  const std::string get_reference_frame() const;

  /**
   * @brief Add new point and corresponding time to trajectory
   * @param new_point the new trajectory point
   * @param duration the duration for the new point
   */
  void add_point(const CartesianState& new_point, const std::chrono::nanoseconds& duration);

  /**
   * @brief Insert new point and corresponding time to trajectory between two
   * already existing points
   * @param new_point the new trajectory point
   * @param duration the duration for the new point
   * @param pos the desired position of the new point in the queue
   */
  void insert_point(const CartesianState& new_point, const std::chrono::nanoseconds& duration, unsigned int pos);

  /**
   * @brief Get list of trajectory points
   * @return queue of the Cartesian states of the trajectory
   */
  const std::deque<CartesianState> get_points() const;

  /**
   * @brief Get the trajectory point at given index
   * @param index the index
   * @return the Cartesian state that corresponds to the index
   */
  CartesianState get_point(unsigned int index) const;

  /**
   * @brief Set the trajectory point at given index
   * @param point the new point
   * @param duration the new time
   * @param index the index
   * @throw std::out_of_range if index is out of range
   * @throw EmptyStateException if point is empty
   * @throw IncompatibleReferenceFramesException if point has different reference frame
   */
  void set_point(const CartesianState& point, const std::chrono::nanoseconds& duration, unsigned int index);

  /**
   * @brief Set the trajectory point at given index
   * @param points vector of new points
   * @param durations vector of new durations
   * @throw IncompatibleSizeException if points and durations have different sizes
   * @throw EmptyStateException if point is empty
   * @throw IncompatibleReferenceFramesException if point has different reference frame
   */
  void set_points(const std::vector<CartesianState>& points, const std::vector<std::chrono::nanoseconds>& durations);

  /**
   * @brief Operator overload for returning a single trajectory point and
   * corresponding time
   * @return the Cartesian state and duration pair that corresponds to the index
   */
  std::pair<CartesianState, const std::chrono::nanoseconds> operator[](unsigned int idx) const;

  /**
   * @brief Reset trajectory
   */
  virtual void reset() override;

private:
  std::string reference_frame_;///< name of the reference frame
};
}// namespace state_representation
