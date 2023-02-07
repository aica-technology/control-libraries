#pragma once

#include "state_representation/space/joint/JointState.hpp"
#include "state_representation/space/joint/JointVelocities.hpp"
#include "state_representation/space/joint/JointAccelerations.hpp"
#include "state_representation/space/joint/JointTorques.hpp"

namespace state_representation {

class JointVelocities;
class JointAccelerations;
class JointTorques;

/**
 * @class JointPositions
 * @brief Class to define positions of the joints
 */
class JointPositions : public JointState {
public:
  const Eigen::VectorXd& get_velocities() const = delete;
  double get_velocity(unsigned int joint_index) const = delete;
  double get_velocity(const std::string& joint_name) const = delete;
  void set_velocities(const Eigen::VectorXd& velocities) = delete;
  void set_velocities(const std::vector<double>& velocities) = delete;
  void set_velocity(double velocity, unsigned int joint_index) const = delete;
  void set_velocity(double velocity, const std::string& joint_name) const = delete;
  const Eigen::VectorXd& get_accelerations() const = delete;
  double get_acceleration(unsigned int joint_index) const = delete;
  double get_acceleration(const std::string& joint_name) const = delete;
  void set_accelerations(const Eigen::VectorXd& accelerations) = delete;
  void set_accelerations(const std::vector<double>& accelerations) = delete;
  void set_acceleration(double acceleration, unsigned int joint_index) const = delete;
  void set_acceleration(double acceleration, const std::string& joint_name) const = delete;
  const Eigen::VectorXd& get_torques() const = delete;
  double get_torque(unsigned int joint_index) const = delete;
  double get_torque(const std::string& joint_name) const = delete;
  void set_torques(const Eigen::VectorXd& torques) = delete;
  void set_torques(const std::vector<double>& torques) = delete;
  void set_torque(double torque, unsigned int joint_index) const = delete;
  void set_torque(double torque, const std::string& joint_name) const = delete;
  JointState& operator+=(const JointVelocities& velocities) = delete;
  JointState& operator+=(const JointAccelerations& accelerations) = delete;
  JointState& operator+=(const JointTorques& torques) = delete;
  JointState operator+(const JointVelocities& velocities) const = delete;
  JointState operator+(const JointAccelerations& accelerations) const = delete;
  JointState operator+(const JointTorques& torques) const = delete;
  JointState& operator-=(const JointVelocities& velocities) = delete;
  JointState& operator-=(const JointAccelerations& accelerations) = delete;
  JointState& operator-=(const JointTorques& torques) = delete;
  JointState operator-(const JointVelocities& velocities) const = delete;
  JointState operator-(const JointAccelerations& accelerations) const = delete;
  JointState operator-(const JointTorques& torques) const = delete;

  /**
   * @brief Empty constructor
   */
  explicit JointPositions();

  /**
   * @brief Constructor with name and number of joints provided
   * @brief name The name of the state
   * @brief nb_joints The number of joints for initialization
   */
  explicit JointPositions(const std::string& robot_name, unsigned int nb_joints = 0);

  /**
   * @brief Constructor with name and list of joint names provided
   * @brief name The name of the state
   * @brief joint_names List of joint names
   */
  explicit JointPositions(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Constructor with name and position values provided
   * @brief name The name of the state
   * @brief positions The vector of positions
   */
  explicit JointPositions(const std::string& robot_name, const Eigen::VectorXd& positions);

  /**
   * @brief Constructor with name, a list of joint names and position values provided
   * @brief name The name of the state
   * @brief joint_names List of joint names
   * @brief positions The vector of positions
   */
  explicit JointPositions(
      const std::string& robot_name, const std::vector<std::string>& joint_names, const Eigen::VectorXd& positions
  );

  /**
   * @brief Copy constructor
   */
  JointPositions(const JointPositions& positions);

  /**
   * @brief Copy constructor from a JointState
   */
  JointPositions(const JointState& state);

  /**
   * @brief Integration constructor from joint velocities by considering that it is equivalent to multiplying the
   * velocities by 1 second
   */
  JointPositions(const JointVelocities& velocities);

  /**
   * @brief Constructor for zero joint positions
   * @param robot_name The name of the associated robot
   * @param nb_joints The number of joints for initialization
   * @return Joint positions with zero values
   */
  static JointPositions Zero(const std::string& robot_name, unsigned int nb_joints);

  /**
   * @brief Constructor for zero joint positions
   * @param robot_name The name of the associated robot
   * @param joint_names List of joint names
   * @return Joint positions with zero values
   */
  static JointPositions Zero(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Constructor for random joint positions
   * @param robot_name The name of the associated robot
   * @param nb_joints The number of joints for initialization
   * @return Joint positions with random values
   */
  static JointPositions Random(const std::string& robot_name, unsigned int nb_joints);

  /**
   * @brief Constructor for the random joint positions
   * @param robot_name The name of the associated robot
   * @param joint_names List of joint names
   * @return Joint positions with random values
   */
  static JointPositions Random(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Copy assignment operator that has to be defined to the custom assignment operator
   * @param positions The state with value to assign
   * @return Reference to the current state with new values
   */
  JointPositions& operator=(const JointPositions& positions) = default;

  /**
   * @brief Returns the positions data as an Eigen vector
   * @return The positions data vector
   */
  Eigen::VectorXd data() const override;

  /**
   * @brief Set the positions data from an Eigen vector
   * @param data The positions data vector
   */
  virtual void set_data(const Eigen::VectorXd& data) override;

  /**
   * @brief Set the positions data from a std vector
   * @param data The positions data vector
   */
  virtual void set_data(const std::vector<double>& data) override;

  /**
   * @brief Clamp inplace the magnitude of the positions to the value in argument
   * @param max_absolute_value The maximum value of position for all the joints
   * @param noise_ratio If provided, this value will be used to apply a dead zone relative to the maximum absolute value
   * under which the position will be set to 0
   */
  void clamp(double max_absolute_value, double noise_ratio = 0.);

  /**
   * @brief Clamp inplace the magnitude of the positions to the values in argument
   * @param max_absolute_value_array The maximum value of position for each joint
   * @param noise_ratio_array Those values will be used to apply a dead zone relative to the maximum absolute value
   * under which the position will be set to 0 for each individual joint
   */
  void clamp(const Eigen::ArrayXd& max_absolute_value_array, const Eigen::ArrayXd& noise_ratio_array);

  /**
   * @brief Return the position clamped to the value in argument
   * @param max_absolute_value The maximum value of position for all the joints
   * @param noise_ratio If provided, this value will be used to apply a dead zone relative to the maximum absolute value
   * under which the position will be set to 0
   * @return The clamped joint positions
   */
  JointPositions clamped(double max_absolute_value, double noise_ratio = 0.) const;

  /**
   * @brief Return the position clamped to the values in argument
   * @param max_absolute_value_array The maximum value of position for each joint
   * @param noise_ratio_array Those values will be used to apply a dead zone relative to the maximum absolute value
   * under which the position will be set to 0 for each individual joint
   * @return The clamped joint positions
   */
  JointPositions clamped(const Eigen::ArrayXd& max_absolute_value_array, const Eigen::ArrayXd& noise_ratio_array) const;

  /**
   * @brief Return a copy of the joint positions
   */
  JointPositions copy() const;

  /**
   * @brief Scale inplace by a scalar
   * @copydetails JointState::operator*=(double)
   * @param lambda The scaling factor
   * @return The reference to the scaled joint positions
   */
  JointPositions& operator*=(double lambda);

  /**
   * @brief Scale joint positions by a scalar
   * @copydetails JointState::operator*=(double)
   * @param lambda The scaling factor
   * @return The scaled joint positions
   */
  JointPositions operator*(double lambda) const;

  /**
   * @brief Scale joint positions by a scalar
   * @copydetails JointState::operator*=(double)
   * @param lambda The scaling factor
   * @param positions The joint positions to be scaled
   * @return The scaled joint positions
   */
  friend JointPositions operator*(double lambda, const JointPositions& positions);

  /**
   * @brief Scale joint positions by a matrix
   * @param lambda The scaling matrix
   * @param accelerations The joint positions to be scaled
   * @return The scaled joint accelerations
   */
  friend JointPositions operator*(const Eigen::MatrixXd& lambda, const JointPositions& positions);

  /**
   * @brief Scale inplace by a scalar
   * @copydetails JointState::operator*=(double)
   * @param lambda The scaling factor
   * @return The reference to the scaled joint positions
   */
  JointPositions& operator/=(double lambda);

  /**
   * @brief Scale joint positions by a scalar
   * @copydetails JointState::operator*=(double)
   * @param lambda The scaling factor
   * @return The scaled joint positions
   */
  JointPositions operator/(double lambda) const;

  /**
   * @brief Differentiate joint positions pose over a time period
   * @param dt The time period used for derivation
   * @return The resulting joint velocities after derivation
   */
  JointVelocities operator/(const std::chrono::nanoseconds& dt) const;

  /**
   * @brief Add inplace other joint positions
   * @param positions Joint positions with same name and same joint names
   * @return The reference to the combined joint positions
   */
  JointPositions& operator+=(const JointPositions& positions);

  /**
   * @brief Add inplace other joint positions from a joint state
   * @param state A joint state with same name and same joint names
   * @return The reference to the combined joint positions
   */
  JointPositions& operator+=(const JointState& state);

  /**
   * @brief Add other joint positions
   * @param positions Joint positions with same name and same joint names
   * @return The combined joint positions
   */
  JointPositions operator+(const JointPositions& positions) const;

  /**
   * @brief Add another joint sate
   * @param state A joint state with same name and same joint names
   * @return The combined joint state
   */
  JointState operator+(const JointState& state) const;

  /**
   * @brief Negate joint positions
   * @return The negative value of the joint positions
   */
  JointPositions operator-() const;

  /**
   * @brief Compute inplace the difference with other joint positions
   * @param positions Joint positions with same name and same joint names
   * @return The reference to the difference in positions
   */
  JointPositions& operator-=(const JointPositions& positions);

  /**
   * @brief Compute inplace the difference with a joint state
   * @param state A joint state with same name and same joint names
   * @return The reference to the difference in positions
   */
  JointPositions& operator-=(const JointState& state);

  /**
   * @brief Compute the difference with other joint positions
   * @param positions Joint positions with same name and same joint names
   * @return The difference in positions
   */
  JointPositions operator-(const JointPositions& positions) const;

  /**
   * @brief Compute the difference with a joint state
   * @param state A joint state with same name and same joint names
   * @return The difference in all the state variables
   */
  JointState operator-(const JointState& state) const;

  /**
   * @brief Overload the ostream operator for printing
   * @param os The ostream to append the string representing the state
   * @param positions The state to print
   * @return The appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const JointPositions& positions);

private:
  using JointState::clamp_state_variable;
};
}// namespace state_representation
