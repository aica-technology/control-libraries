#pragma once

#include "state_representation/space/joint/JointState.hpp"
#include "state_representation/trajectory/TrajectoryBase.hpp"

namespace state_representation {

/**
 * @class JointTrajectoryPoint
 * @brief A joint trajectory point representation
 */
struct JointTrajectoryPoint : public TrajectoryPoint {
  /**
   * @brief Empty constructor
   */
  JointTrajectoryPoint() = default;

  /**
   * @brief Constructor from joint state and duration
   * @param state the Joint state used to initialize the trajectory point
   * @param duration the intended duration for the trajectory point 
   */
  JointTrajectoryPoint(const JointState& state, const std::chrono::nanoseconds& duration)
      : TrajectoryPoint(state.get_name(), state.data(), duration) {}

  /**
   * @brief Convert the trajectory point to a joint state
   * @param joint_names the joint names of the trajectory point
   * @return the joint state representation of the trajectory point 
   */
  JointState to_joint_state(const std::vector<std::string>& joint_names) const {
    JointState state(name, joint_names);
    state.set_data(data);
    return state;
  }
};

/**
 * @class JointTrajectory
 * @brief Class to represent a trajectory of joint points and corresponding durations
 */
class JointTrajectory : public TrajectoryBase<JointTrajectoryPoint> {
public:
  /**
   * @brief Constructor with optional name
   */
  explicit JointTrajectory(const std::string& name = "");

  /**
   * @brief Constructor with name, initial point, and duration provided
   * @param name the name of the state
   * @param point the initial point
   * @param duration the initial duration
   */
  explicit JointTrajectory(const std::string& name, const JointState& point, const std::chrono::nanoseconds& duration);

  /**
   * @brief Constructor with name, initial points, and durations provided
   * @param name the name of the state
   * @param points vector of initial points
   * @param durations vector of initial durations
   */
  explicit JointTrajectory(
      const std::string& name, const std::vector<JointState>& points,
      const std::vector<std::chrono::nanoseconds>& durations);

  /**
   * @brief Copy constructor of a JointTrajectory
   * @param state the joint trajectory to copy from
   */
  JointTrajectory(const JointTrajectory& state);

  /**
   * @brief Get the joint names
   * @return vector of joint names associated with the trajectory
   */
  const std::vector<std::string>& get_joint_names() const;

  /**
   * @brief Set the joint names
   * @param joint_names vector of joint names associated with the trajectory
   */
  void set_joint_names(const std::vector<std::string>& joint_names);

  /**
   * @brief Get list of trajectory points
   * @return vector of the Joint states of the trajectory
   */
  const std::vector<JointState> get_points() const;

  /**
   * @brief Get the trajectory point at given index
   * @param index the index
   * @return the Joint state that corresponds to the index
   */
  const JointState get_point(unsigned int index) const;

  /**
   * @brief Add new point and corresponding duration to trajectory
   * @param point the new trajectory point
   * @param duration the duration for the new point
   * @throw EmptyStateException if point is empty
   * @throw IncompatibleStatesException if point has different joint names
   */
  void add_point(const JointState& point, const std::chrono::nanoseconds& duration);

  /**
   * @brief Add new points and corresponding durations to trajectory
   * @param points the new trajectory point
   * @param durations the duration for the new point
   * @throw IncompatibleSizeException if points and durations have different sizes
   * @throw EmptyStateException if point is empty
   * @throw IncompatibleStatesException if any of the points has different joint names
   */
  void add_points(const std::vector<JointState>& points, const std::vector<std::chrono::nanoseconds>& durations);

  /**
   * @brief Insert new point and corresponding duration to trajectory between two
   * already existing points
   * @param point the new trajectory point
   * @param duration the duration for the new point
   * @param index the desired position of the new point in the queue
   */
  void insert_point(const JointState& point, const std::chrono::nanoseconds& duration, unsigned int index);

  /**
   * @brief Set the trajectory point at given index
   * @param point the new point
   * @param duration the new duration
   * @param index the index
   * @throw std::out_of_range if index is out of range
   * @throw EmptyStateException if point is empty
   * @throw IncompatibleStatesException if point has different joint names to the current ones
   */
  void set_point(const JointState& point, const std::chrono::nanoseconds& duration, unsigned int index);

  /**
   * @brief Set the trajectory point at given index
   * @param points vector of new points
   * @param duration vector of new durations
   * @throw IncompatibleSizeException if points and durations have different sizes
   * @throw EmptyStateException if point is empty
   * @throw IncompatibleStatesException if any of the points has different joint names to the current ones
   */
  void set_points(const std::vector<JointState>& points, const std::vector<std::chrono::nanoseconds>& durations);

  /**
   * @brief Operator overload for returning a single trajectory point and
   * corresponding duration
   * @return the Joint state and duration pair that corresponds to the index
   */
  std::pair<JointState, const std::chrono::nanoseconds> operator[](unsigned int idx) const;

  /**
   * @brief Copy assignment operator that has to be defined to the custom assignment operator
   * @param trajectory the trajectory with value to assign
   * @return reference to the current trajectory with new values
   */
  JointTrajectory& operator=(const JointTrajectory& trajectory);

  /**
   * @brief Swap the values of trajectories
   * @param trajectory1 trajectory to be swapped with 2
   * @param trajectory2 trajectory to be swapped with 1
   */
  friend inline void swap(JointTrajectory& trajectory1, JointTrajectory& trajectory2) {
    swap(
        static_cast<TrajectoryBase<JointTrajectoryPoint>&>(trajectory1),
        static_cast<TrajectoryBase<JointTrajectoryPoint>&>(trajectory2));
    std::swap(trajectory1.joint_names_, trajectory2.joint_names_);
  }

private:
  /**
   * @brief Assert that all states of a vector carry the same joint names
   * @param states the states to check
   * @throw IncompatibleStatesException if a state has a different joint names
   */
  void assert_compatible_joint_names(const std::vector<JointState>& states) const;

  /**
   * @brief Assert that all states of a vector carry the same joint names as the one provided
   * @param states the states to check
   * @param reference_frame the joint names to check against
   * @throw IncompatibleStatesException if a state has a different joint names
   */
  void assert_compatible_joint_names(
      const std::vector<JointState>& states, const std::vector<std::string>& joint_names) const;

  std::vector<std::string> joint_names_;///< names of the joints
};
}// namespace state_representation
