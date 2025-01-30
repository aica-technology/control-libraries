#pragma once

#include "state_representation/space/joint/JointState.hpp"
#include "state_representation/trajectory/TrajectoryBase.hpp"

namespace state_representation {

/**
 * @class JointTrajectoryPoint
 * @brief Struct to represent a joint trajectory point
 */
struct JointTrajectoryPoint : public TrajectoryPoint {};

/**
 * @class JointTrajectory
 * @brief Class to represent a trajectory of joint points and corresponding durations
 */
class JointTrajectory : public TrajectoryBase<JointTrajectoryPoint> {
public:
  /**
   * @brief Constructor with name and reference frame provided
   * @param name the name of the state
   */
  explicit JointTrajectory(const std::string& name = "");

  /**
   * @brief Constructor with initial point, duration, name, and reference frame provided
   * @param name the name of the state
   * @param point the initial point
   * @param duration the initial duration
   */
  explicit JointTrajectory(const std::string& name, const JointState& point, const std::chrono::nanoseconds& duration);

  /**
   * @brief Constructor with initial points, durations, name, and reference frame provided
   * @param name the name of the state
   * @param points vector of initial points
   * @param durations vector of initial durations
   */
  explicit JointTrajectory(
      const std::string& name, const std::vector<JointState>& points,
      const std::vector<std::chrono::nanoseconds>& durations
  );

  /**
   * @brief Getter of the joint names
   * @return vector of joint names associated with the trajectory
   */
  const std::vector<std::string>& get_joint_names() const;

  /**
   * @brief Setter of the joint names
   * @param joint_names vector of joint names associated with the trajectory
   */
  void set_joint_names(const std::vector<std::string>& joint_names);

  /**
   * @brief Add new point and corresponding duration to trajectory
   * @param new_point the new trajectory point
   * @param duration the duration for the new point
   * @throw EmptyStateException if point is empty
   * @throw IncompatibleStatesException if point has different robot name
   */
  void add_point(const JointState& new_point, const std::chrono::nanoseconds& duration);

  /**
   * @brief Add new points and corresponding durations to trajectory
   * @param new_points the new trajectory point
   * @param durations the duration for the new point
   * @throw IncompatibleSizeException if points and durations have different sizes
   * @throw EmptyStateException if point is empty
   * @throw IncompatibleStatesException if point has different robot name
   */
  void add_points(const std::vector<JointState>& new_points, const std::vector<std::chrono::nanoseconds>& durations);

  /**
   * @brief Insert new point and corresponding duration to trajectory between two
   * already existing points
   * @param new_point the new trajectory point
   * @param duration the duration for the new point
   * @param index the desired position of the new point in the queue
   */
  void insert_point(const JointState& new_point, const std::chrono::nanoseconds& duration, unsigned int index);

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
   * @brief Set the trajectory point at given index
   * @param point the new point
   * @param duration the new duration
   * @param index the index
   * @throw std::out_of_range if index is out of range
   * @throw EmptyStateException if point is empty
   * @throw IncompatibleStatesException if point has different robot name
   */
  void set_point(const JointState& point, const std::chrono::nanoseconds& duration, unsigned int index);

  /**
   * @brief Set the trajectory point at given index
   * @param points vector of new points
   * @param duration vector of new durations
   * @throw IncompatibleSizeException if points and durations have different sizes
   * @throw EmptyStateException if point is empty
   * @throw IncompatibleStatesException if point has different robot name
   */
  void set_points(const std::vector<JointState>& points, const std::vector<std::chrono::nanoseconds>& durations);

  /**
   * @brief Operator overload for returning a single trajectory point and
   * corresponding duration
   * @return the Joint state and duration pair that corresponds to the index
   */
  std::pair<JointState, const std::chrono::nanoseconds> operator[](unsigned int idx) const;

private:
  std::vector<std::string> joint_names_;///< names of the joints
};
}// namespace state_representation
