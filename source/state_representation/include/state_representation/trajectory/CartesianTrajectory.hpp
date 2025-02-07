#pragma once

#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/trajectory/TrajectoryBase.hpp"

namespace state_representation {

/**
 * @class CartesianTrajectoryPoint
 * @brief A Cartesian trajectory point representation
 */
struct CartesianTrajectoryPoint : public TrajectoryPoint {
  /**
   * @brief Empty constructor
   */
  CartesianTrajectoryPoint() = default;

  /**
   * @brief Constructor from Cartesian state and duration
   * @param state the Cartesian state used to initialize the trajectory point
   * @param duration the intended duration for the trajectory point 
   */
  CartesianTrajectoryPoint(const CartesianState& state, const std::chrono::nanoseconds& duration)
      : TrajectoryPoint(state.get_name(), state.data(), duration) {}

  /**
   * @brief Convert the trajectory point to a Cartesian state
   * @param reference_frame the underlying reference frame of the trajectory point 
   * @return the Cartesian state representation of the trajectory point 
   */
  CartesianState to_cartesian_state(const std::string& reference_frame) const {
    CartesianState state(name, reference_frame);
    state.set_data(data);
    return state;
  }
};

/**
 * @class CartesianTrajectory
 * @brief Class to represent a trajectory of Cartesian points and corresponding durations
 */
class CartesianTrajectory : public TrajectoryBase<CartesianTrajectoryPoint> {
public:
  /**
   * @brief Empty constructor
   */
  explicit CartesianTrajectory();

  /**
   * @brief Constructor with name and reference frame provided
   * @param name the name of the state
   * @param reference_frame reference frame of the trajectory points
   */
  explicit CartesianTrajectory(const std::string& name, const std::string& reference_frame = "world");

  /**
   * @brief Constructor with name, initial point, and duration provided
   * @param name the name of the state
   * @param point the initial point
   * @param duration the initial duration
   * @throw EmptyStateException if point is empty
   */
  explicit CartesianTrajectory(
      const std::string& name, const CartesianState& point, const std::chrono::nanoseconds& duration
  );

  /**
   * @brief Constructor with name, intial points, and durations provided
   * @param name the name of the state
   * @param points vector of initial points
   * @param durations vector of initial durations
   * @throw EmptyStateException if any point is empty
   * @throw IncompatibleReferenceFramesException if any point has different reference frame from others
   * @throw IncompatibleSizeException if points and durations have different sizes
   */
  explicit CartesianTrajectory(
      const std::string& name, const std::vector<CartesianState>& points,
      const std::vector<std::chrono::nanoseconds>& durations
  );

  /**
   * @brief Copy constructor of a CartesianTrajectory
   * @param state the Cartesian trajectory to copy from
   */
  CartesianTrajectory(const CartesianTrajectory& state);

  /**
   * @brief Get the reference frame
   * @return the reference frame associated with the trajectory
   */
  const std::string& get_reference_frame() const;

  /**
   * @brief Set the reference frame by applying a transformation to all existing points to change the reference frame
   * @param pose the new pose that needs to be applied to existing points to change the reference frame
   * @throws EmptyStateException if pose is empty
   */
  void set_reference_frame(const CartesianPose& pose);

  /**
   * @brief Set the reference frame by simply changing the reference frame name
   * @param reference_frame the new reference frame
   */
  void set_reference_frame(const std::string& reference_frame);

  /**
   * @brief Get list of trajectory points
   * @return queue of the Cartesian states of the trajectory
   */
  const std::vector<CartesianState> get_points() const;

  /**
   * @brief Get the trajectory point at given index
   * @param index the index
   * @return the Cartesian state that corresponds to the index
   */
  CartesianState get_point(unsigned int index) const;

  /**
   * @brief Add new point and corresponding duration to trajectory
   * @param point the new trajectory point
   * @param duration the duration for the new point
   * @throw EmptyStateException if point is empty
   * @throw IncompatibleReferenceFramesException if point has different reference frame
   */
  void add_point(const CartesianState& point, const std::chrono::nanoseconds& duration);

  /**
   * @brief Add new points and corresponding durations to trajectory
   * @param points the new trajectory point
   * @param durations the duration for the new point
   * @throw IncompatibleSizeException if points and durations have different sizes
   * @throw EmptyStateException if point is empty
   * @throw IncompatibleReferenceFramesException if point has different reference frame
   */
  void add_points(const std::vector<CartesianState>& points, const std::vector<std::chrono::nanoseconds>& durations);

  /**
   * @brief Insert new point and corresponding duration to trajectory between two
   * already existing points
   * @param point the new trajectory point
   * @param duration the duration for the new point
   * @param index the desired position of the new point in the queue
   */
  void insert_point(const CartesianState& point, const std::chrono::nanoseconds& duration, unsigned int index);

  /**
   * @brief Set the trajectory point at given index
   * @param point the new point
   * @param duration the new duration
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
   * corresponding duration
   * @return the Cartesian state and duration pair that corresponds to the index
   */
  std::pair<CartesianState, const std::chrono::nanoseconds> operator[](unsigned int idx) const;

  /**
   * @brief Copy assignment operator that has to be defined to the custom assignment operator
   * @param trajectory the trajectory with value to assign
   * @return reference to the current trajectory with new values
   */
  CartesianTrajectory& operator=(const CartesianTrajectory& trajectory);

private:
  /**
   * @brief Assert that all states of a vector carry the same reference frame
   * @param states the states to check
   * @throw IncompatibleReferenceFramesException if a state has a different reference frame
   */
  void assert_same_reference_frame(const std::vector<CartesianState>& states) const;

  /**
   * @brief Assert that all states of a vector carry the same reference frame as the one provided
   * @param states the states to check
   * @param reference_frame the reference frame to check against
   * @throw IncompatibleReferenceFramesException if a state has a different reference frame
   */
  void assert_same_reference_frame(const std::vector<CartesianState>& states, const std::string& reference_frame) const;

  std::string reference_frame_;///< name of the reference frame
};

inline void swap(CartesianTrajectory& trajectory1, CartesianTrajectory& trajectory2) {
  swap(
      static_cast<TrajectoryBase<CartesianTrajectoryPoint>&>(trajectory1),
      static_cast<TrajectoryBase<CartesianTrajectoryPoint>&>(trajectory2));
  auto tmp = trajectory1.get_reference_frame();
  trajectory1.set_reference_frame(trajectory2.get_reference_frame());
  trajectory2.set_reference_frame(tmp);
}

}// namespace state_representation
