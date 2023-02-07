#pragma once

#include "state_representation/space/joint/JointState.hpp"
#include "state_representation/space/joint/JointPositions.hpp"
#include "state_representation/space/joint/JointVelocities.hpp"
#include "state_representation/space/joint/JointTorques.hpp"

namespace state_representation {

class JointPositions;
class JointVelocities;
class JointTorques;

/**
 * @class JointAccelerations
 * @brief Class to define accelerations of the joints
 */
class JointAccelerations : public JointState {
public:
  const Eigen::VectorXd& get_positions() const = delete;
  double get_position(unsigned int joint_index) const = delete;
  double get_position(const std::string& joint_name) const = delete;
  void set_positions(const Eigen::VectorXd& positions) = delete;
  void set_positions(const std::vector<double>& positions) = delete;
  void set_position(double position, unsigned int joint_index) const = delete;
  void set_position(double position, const std::string& joint_name) const = delete;
  const Eigen::VectorXd& get_velocities() const = delete;
  double get_velocity(unsigned int joint_index) const = delete;
  double get_velocity(const std::string& joint_name) const = delete;
  void set_velocities(const Eigen::VectorXd& accelerations) = delete;
  void set_velocities(const std::vector<double>& accelerations) = delete;
  void set_velocity(double velocity, unsigned int joint_index) const = delete;
  void set_velocity(double velocity, const std::string& joint_name) const = delete;
  const Eigen::VectorXd& get_torques() const = delete;
  double get_torque(unsigned int joint_index) const = delete;
  double get_torque(const std::string& joint_name) const = delete;
  void set_torques(const Eigen::VectorXd& torques) = delete;
  void set_torques(const std::vector<double>& torques) = delete;
  void set_torque(double torque, unsigned int joint_index) const = delete;
  void set_torque(double torque, const std::string& joint_name) const = delete;
  JointState& operator+=(const JointPositions& positions) = delete;
  JointState& operator+=(const JointVelocities& velocities) = delete;
  JointState& operator+=(const JointTorques& torques) = delete;
  JointState operator+(const JointPositions& positions) const = delete;
  JointState operator+(const JointVelocities& velocities) const = delete;
  JointState operator+(const JointTorques& torques) const = delete;
  JointState& operator-=(const JointPositions& positions) = delete;
  JointState& operator-=(const JointVelocities& velocities) = delete;
  JointState& operator-=(const JointTorques& torques) = delete;
  JointState operator-(const JointPositions& positions) const = delete;
  JointState operator-(const JointVelocities& velocities) const = delete;
  JointState operator-(const JointTorques& torques) const = delete;

  /**
   * @brief Empty constructor
   */
  explicit JointAccelerations();

  /**
   * @brief Constructor with name and number of joints provided
   * @brief name The name of the state
   * @brief nb_joints The number of joints for initialization
   */
  explicit JointAccelerations(const std::string& robot_name, unsigned int nb_joints = 0);

  /**
   * @brief Constructor with name and list of joint names provided
   * @brief name The name of the state
   * @brief joint_names List of joint names
   */
  explicit JointAccelerations(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Constructor with name and acceleration values provided
   * @brief name The name of the state
   * @brief accelerations The vector of accelerations
   */
  explicit JointAccelerations(const std::string& robot_name, const Eigen::VectorXd& accelerations);

  /**
   * @brief Constructor with name, a list of joint names and accelerations values provided
   * @brief name The name of the state
   * @brief joint_names List of joint names
   * @brief accelerations The vector of accelerations
   */
  explicit JointAccelerations(
      const std::string& robot_name, const std::vector<std::string>& joint_names, const Eigen::VectorXd& accelerations
  );

  /**
   * @brief Copy constructor
   */
  JointAccelerations(const JointAccelerations& accelerations);

  /**
   * @brief Copy constructor from a joint state
   */
  JointAccelerations(const JointState& state);

  /**
   * @brief Differentiation constructor from joint velocities by considering that it is equivalent to dividing
   * the velocities by 1 second
   */
  JointAccelerations(const JointVelocities& velocities);

  /**
   * @brief Constructor for zero joint accelerations
   * @param robot_name The name of the associated robot
   * @param nb_joints The number of joints for initialization
   * @return Joint accelerations with zero values
   */
  static JointAccelerations Zero(const std::string& robot_name, unsigned int nb_joints);

  /**
   * @brief Constructor for zero joint accelerations
   * @param robot_name The name of the associated robot
   * @param joint_names List of joint names
   * @return Joint accelerations with zero values
   */
  static JointAccelerations Zero(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Constructor for random joint accelerations
   * @param robot_name The name of the associated robot
   * @param nb_joints The number of joints for initialization
   * @return Joint accelerations with random values
   */
  static JointAccelerations Random(const std::string& robot_name, unsigned int nb_joints);

  /**
   * @brief Constructor for random joint accelerations
   * @param robot_name The name of the associated robot
   * @param joint_names List of joint names
   * @return Joint accelerations with random values
   */
  static JointAccelerations Random(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Copy assignment operator that has to be defined to the custom assignment operator
   * @param accelerations The state with value to assign
   * @return Reference to the current state with new values
   */
  JointAccelerations& operator=(const JointAccelerations& accelerations) = default;

  /**
   * @brief Returns the accelerations data as an Eigen vector
   * @return The accelerations data vector
   */
  Eigen::VectorXd data() const override;

  /**
   * @brief Set the accelerations data from an Eigen vector
   * @param data The accelerations data vector
   */
  virtual void set_data(const Eigen::VectorXd& data) override;

  /**
   * @brief Set the accelerations data from a std vector
   * @param data The accelerations data vector
   */
  virtual void set_data(const std::vector<double>& data) override;

  /**
   * @brief Clamp inplace the magnitude of the acceleration to the values in argument
   * @param max_absolute_value The maximum magnitude of acceleration for all the joints
   * @param noise_ratio If provided, this value will be used to apply a dead zone under which
   * the acceleration will be set to 0
   */
  void clamp(double max_absolute_value, double noise_ratio = 0.);

  /**
   * @brief Clamp inplace the magnitude of the acceleration to the values in argument
   * @param max_absolute_value_array The maximum magnitude of acceleration for each joint
   * @param noise_ratio_array If provided, this value will be used to apply a dead zone under which
   * the acceleration will be set to 0
   */
  void clamp(const Eigen::ArrayXd& max_absolute_value_array, const Eigen::ArrayXd& noise_ratio_array);

  /**
   * @brief Return the acceleration clamped to the values in argument
   * @param max_absolute_value the maximum magnitude of acceleration for all the joints
   * @param noise_ratio If provided, this value will be used to apply a dead zone under which
   * the acceleration will be set to 0
   * @return The clamped joint accelerations
   */
  JointAccelerations clamped(double max_absolute_value, double noise_ratio = 0.) const;

  /**
   * @brief Return the acceleration clamped to the values in argument
   * @param max_absolute_value_array The maximum magnitude of acceleration for each joint
   * @param noise_ratio_array If provided, this value will be used to apply a dead zone under which
   * the acceleration will be set to 0
   * @return The clamped joint accelerations
   */
  JointAccelerations clamped(
      const Eigen::ArrayXd& max_absolute_value_array, const Eigen::ArrayXd& noise_ratio_array
  ) const;

  /**
   * @brief Return a copy of the joint accelerations
   */
  JointAccelerations copy() const;

  /**
   * @brief Scale inplace by a scalar
   * @copydetails JointState::operator*=(double)
   * @param lambda The scaling factor
   * @return The reference to the scaled joint accelerations
   */
  JointAccelerations& operator*=(double lambda);

  /**
   * @brief Scale joint accelerations by a scalar
   * @copydetails JointState::operator*=(double)
   * @param lambda The scaling factor
   * @return The scaled joint accelerations
   */
  JointAccelerations operator*(double lambda) const;

  /**
   * @brief Scale joint accelerations by a scalar
   * @copydetails JointState::operator*=(double)
   * @param lambda The scaling factor
   * @param accelerations The joint accelerations to be scaled
   * @return The scaled joint accelerations
   */
  friend JointAccelerations operator*(double lambda, const JointAccelerations& accelerations);

  /**
   * @brief Scale joint accelerations by a matrix
   * @param lambda The scaling matrix
   * @param accelerations The joint accelerations to be scaled
   * @return The scaled joint accelerations
   */
  friend JointAccelerations operator*(const Eigen::MatrixXd& lambda, const JointAccelerations& accelerations);

  /**
   * @brief Integrate joint accelerations over a time period
   * @param dt The time period used for integration
   * @return The resulting joint velocities after integration
   */
  JointVelocities operator*(const std::chrono::nanoseconds& dt) const;

  /**
   * @brief Integrate joint accelerations over a time period
   * @param dt The time period used for integration
   * @param accelerations The joint accelerations to be integrated
   * @return The resulting joint velocities after integration
   */
  friend JointVelocities operator*(const std::chrono::nanoseconds& dt, const JointAccelerations& accelerations);

  /**
   * @brief Scale inplace by a scalar
   * @copydetails JointState::operator*=(double)
   * @param lambda The scaling factor
   * @return The reference to the scaled joint accelerations
   */
  JointAccelerations& operator/=(double lambda);

  /**
   * @brief Scale joint accelerations by a scalar
   * @copydetails JointState::operator*=(double)
   * @param lambda The scaling factor
   * @return The scaled joint accelerations
   */
  JointAccelerations operator/(double lambda) const;

  /**
   * @brief Add inplace other joint accelerations
   * @param accelerations Joint accelerations with same name and same joint names
   * @return The reference to the combined joint accelerations
   */
  JointAccelerations& operator+=(const JointAccelerations& accelerations);

  /**
   * @brief Add inplace other joint accelerations from a joint state
   * @param state A joint state with same name and same joint names
   * @return The reference to the combined joint accelerations
   */
  JointAccelerations& operator+=(const JointState& state);

  /**
   * @brief Add other joint accelerations
   * @param accelerations Joint accelerations with same name and same joint names
   * @return The combined joint accelerations
   */
  JointAccelerations operator+(const JointAccelerations& accelerations) const;

  /**
   * @brief Add another joint sate
   * @param state A joint state with same name and same joint names
   * @return The combined joint state
   */
  JointState operator+(const JointState& state) const;

  /**
   * @brief Negate joint accelerations
   * @return The negative value of the joint accelerations
   */
  JointAccelerations operator-() const;

  /**
   * @brief Compute inplace the difference with other joint accelerations
   * @param accelerations Joint accelerations with same name and same joint names
   * @return The reference to the difference in accelerations
   */
  JointAccelerations& operator-=(const JointAccelerations& accelerations);

  /**
   * @brief Compute inplace the difference with a joint state
   * @param state A joint state with same name and same joint names
   * @return The reference to the difference in accelerations
   */
  JointAccelerations& operator-=(const JointState& state);

  /**
   * @brief Compute the difference with other joint accelerations
   * @param accelerations Joint accelerations with same name and same joint names
   * @return The difference in accelerations
   */
  JointAccelerations operator-(const JointAccelerations& accelerations) const;

  /**
   * @brief Compute the difference with a joint state
   * @param state A joint state with same name and same joint names
   * @return The difference in all the state variables
   */
  JointState operator-(const JointState& state) const;

  /**
   * @brief Overload the ostream operator for printing
   * @param os The ostream to append the string representing the state
   * @param state The state to print
   * @return The appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const JointAccelerations& accelerations);

private:
  using JointState::clamp_state_variable;
};
}// namespace state_representation
