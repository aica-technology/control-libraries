#pragma once

#include "state_representation/space/joint/JointState.hpp"
#include "state_representation/trajectories/TrajectoryBase.hpp"

namespace state_representation {
class JointTrajectory : public TrajectoryBase<Eigen::VectorXd> {
private:
  std::vector<std::string> joint_names_;///< names of the joints
  std::string robot_name_;              ///< name of the robot

public:
  /**
   * @brief Empty constructor
   */
  explicit JointTrajectory();

  /**
   * @brief Constructor with name and reference frame provided
   * @brief name the name of the state
   */
  explicit JointTrajectory(const std::string& name);

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
   * @brief Operator overload for returning a single trajectory point and
   * corresponding time
   */
  std::pair<JointState, std::chrono::nanoseconds> operator[](unsigned int idx);
};

template<typename DurationT>
inline bool
JointTrajectory::add_point(const JointState& new_point, const std::chrono::duration<int64_t, DurationT>& new_time) {
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
  if (this->get_size() == 0) {
    this->joint_names_ = new_point.get_names();
    this->robot_name_ = new_point.get_name();
  } else if (this->joint_names_ != new_point.get_names()) {
    return false;
  }
  this->TrajectoryBase<Eigen::VectorXd>::insert_point(new_point.data(), new_time, pos);
  return true;
}
}// namespace state_representation
