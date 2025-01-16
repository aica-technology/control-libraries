#pragma once

#include "state_representation/space/joint/JointState.hpp"
#include "state_representation/trajectory/TrajectoryBase.hpp"

#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/IncompatibleStatesException.hpp"

namespace state_representation {

class JointTrajectory : public TrajectoryBase<Eigen::VectorXd> {
public:
  /**
   * @brief Constructor with name and reference frame provided
   * @brief name the name of the state
   */
  explicit JointTrajectory(const std::string& name = "");

  /**
   * @brief Constructor with name and reference frame provided
   * @param point the initial point
   * @param time the initial time
   * @param name the name of the state
   */
  template<typename DurationT>
  explicit JointTrajectory(
      const JointState& point, const std::chrono::duration<int64_t, DurationT>& time, const std::string& name = "");

  /**
   * @brief Constructor with name and reference frame provided
   * @param points vector of initial points
   * @param times vector of initial times
   * @param name the name of the state
   */
  template<typename DurationT>
  explicit JointTrajectory(
      const std::vector<JointState>& points, const std::vector<std::chrono::duration<int64_t, DurationT>>& times,
      const std::string& name = "");

  /**
   * @brief Getter of the names attribute
   */
  const std::vector<std::string>& get_joint_names() const;

  /**
   * @brief Add new point and corresponding time to trajectory
   * @return Success of the operation
   */
  template<typename DurationT>
  bool add_point(const JointState& new_point, const std::chrono::duration<int64_t, DurationT>& new_time);

  /**
   * @brief Insert new point and corresponding time to trajectory between two
   * already existing points
   * @return Success of the operation
   */
  template<typename DurationT>
  bool insert_point(const JointState& new_point, const std::chrono::duration<int64_t, DurationT>& new_time, int pos);

  /**
   * @brief Get attribute list of trajectory points
   */
  const std::deque<JointState> get_points() const;

  /**
   * @brief Get the trajectory point at given index
   * @param index the index
   */
  const JointState get_point(unsigned int index) const;

  /**
   * @brief Set the trajectory point at given index
   * @param point the new point
   * @param new_time the new time
   * @param index the index
   * @throw std::out_of_range if index is out of range
   * @throw EmptyStateException if point is empty
   * @throw IncompatibleReferenceFramesException if point has different reference frame
   */
  template<typename DurationT>
  void
  set_point(const JointState& point, const std::chrono::duration<int64_t, DurationT>& new_time, unsigned int index);

  /**
   * @brief Set the trajectory point at given index
   * @param points vector of new points
   * @param new_time vector of new times
   * @throw IncompatibleSizeException if points and new_times have different sizes
   * @throw EmptyStateException if point is empty
   * @throw IncompatibleReferenceFramesException if point has different reference frame
   */
  template<typename DurationT>
  void set_points(
      const std::vector<JointState>& points, const std::vector<std::chrono::duration<int64_t, DurationT>>& new_times);

  /**
   * @brief Operator overload for returning a single trajectory point and
   * corresponding time
   */
  std::pair<JointState, std::chrono::nanoseconds> operator[](unsigned int idx);

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

private:
  std::vector<std::string> joint_names_;///< names of the joints
  std::string robot_name_;              ///< name of the robot
};

template<typename DurationT>
JointTrajectory::JointTrajectory(
    const JointState& point, const std::chrono::duration<int64_t, DurationT>& time, const std::string& name)
    : TrajectoryBase<Eigen::VectorXd>(name) {
  this->set_type(StateType::JOINT_TRAJECTORY);
  this->reset();
  this->add_point(point, time);
}

template<typename DurationT>
JointTrajectory::JointTrajectory(
    const std::vector<JointState>& points, const std::vector<std::chrono::duration<int64_t, DurationT>>& times,
    const std::string& name)
    : TrajectoryBase<Eigen::VectorXd>(name) {
  this->set_type(StateType::JOINT_TRAJECTORY);
  this->reset();
  this->set_points(points, times);
}

template<typename DurationT>
inline bool
JointTrajectory::add_point(const JointState& new_point, const std::chrono::duration<int64_t, DurationT>& new_time) {
  if (new_point.is_empty()) {
    return false;
  }
  if (this->get_size() == 0) {
    this->joint_names_ = new_point.get_names();
    this->robot_name_ = new_point.get_name();
  } else if (this->joint_names_ != new_point.get_names()) {
    return false;
  }
  this->TrajectoryBase<Eigen::VectorXd>::add_point(new_point.data(), new_time);
  return true;
}

template<typename DurationT>
inline bool JointTrajectory::insert_point(
    const JointState& new_point, const std::chrono::duration<int64_t, DurationT>& new_time, int pos) {
  if (new_point.is_empty()) {
    return false;
  }
  if (this->get_size() == 0) {
    this->joint_names_ = new_point.get_names();
    this->robot_name_ = new_point.get_name();
  } else if (this->joint_names_ != new_point.get_names()) {
    return false;
  }
  this->TrajectoryBase<Eigen::VectorXd>::insert_point(new_point.data(), new_time, pos);
  return true;
}

template<typename DurationT>
inline void JointTrajectory::set_point(
    const JointState& point, const std::chrono::duration<int64_t, DurationT>& new_time, unsigned int index) {
  if (point.is_empty()) {
    throw exceptions::EmptyStateException("Point is empty");
  } else if (point.get_names() != this->joint_names_) {
    throw exceptions::IncompatibleStatesException(
        "Incompatible joint names between the new point and current trajectory");
  } else if (point.get_name() != this->robot_name_) {
    throw exceptions::IncompatibleStatesException(
        "Incompatible robot name between the new point and current trajectory");
  }
  try {
    this->TrajectoryBase<Eigen::VectorXd>::set_point(point.data(), new_time, index);
  } catch (...) {
    throw;
  }
}

template<typename DurationT>
inline void JointTrajectory::set_points(
    const std::vector<JointState>& points, const std::vector<std::chrono::duration<int64_t, DurationT>>& new_times) {
  if (points.empty()) {
    throw exceptions::EmptyStateException("Points vector is empty");
  } else if (points[0].is_empty()) {
    throw exceptions::EmptyStateException("Vector contains at least one point that is empty");
  }
  auto candidate_robot_name = points[0].get_name();
  auto candidate_joint_names = points[0].get_names();

  std::vector<Eigen::VectorXd> data;
  for (const auto& point : points) {
    if (point.is_empty()) {
      throw exceptions::EmptyStateException("Vector contains at least one point that is empty");
    } else if (point.get_names() != candidate_joint_names) {
      throw exceptions::IncompatibleStatesException(
          "Incompatible joint names between the new point and current trajectory");
    } else if (point.get_name() != candidate_robot_name) {
      throw exceptions::IncompatibleStatesException(
          "Incompatible robot name between the new point and current trajectory");
    }
    data.push_back(point.data());
  }
  try {
    this->TrajectoryBase<Eigen::VectorXd>::set_points(data, new_times);
    this->joint_names_ = candidate_joint_names;
    this->robot_name_ = candidate_robot_name;
  } catch (...) {
    throw;
  }
}
}// namespace state_representation
