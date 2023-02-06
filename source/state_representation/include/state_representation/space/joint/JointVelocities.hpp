#pragma once

#include "state_representation/space/joint/JointState.hpp"
#include "state_representation/space/joint/JointPositions.hpp"
#include "state_representation/space/joint/JointAccelerations.hpp"
#include "state_representation/space/joint/JointTorques.hpp"

namespace state_representation {

class JointPositions;
class JointAccelerations;
class JointTorques;

/**
 * @class JointVelocities
 * @brief Class to define velocities of the joints
 */
class JointVelocities : public JointState {
public:
  const Eigen::VectorXd& get_positions() const = delete;
  double get_position(unsigned int joint_index) const = delete;
  double get_position(const std::string& joint_name) const = delete;
  void set_positions(const Eigen::VectorXd& positions) = delete;
  void set_positions(const std::vector<double>& positions) = delete;
  void set_position(double position, unsigned int joint_index) const = delete;
  void set_position(double position, const std::string& joint_name) const = delete;
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
  JointState& operator+=(const JointPositions& positions) = delete;
  JointState& operator+=(const JointAccelerations& accelerations) = delete;
  JointState& operator+=(const JointTorques& torques) = delete;
  JointState operator+(const JointPositions& positions) const = delete;
  JointState operator+(const JointAccelerations& accelerations) const = delete;
  JointState operator+(const JointTorques& torques) const = delete;

  /**
   * @brief Empty constructor
   */
  explicit JointVelocities();

  /**
   * @brief Constructor with name and number of joints provided
   * @brief name The name of the state
   * @brief nb_joints The number of joints for initialization
   */
  explicit JointVelocities(const std::string& robot_name, unsigned int nb_joints = 0);

  /**
   * @brief Constructor with name and list of joint names provided
   * @brief name The name of the state
   * @brief joint_names List of joint names
   */
  explicit JointVelocities(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Constructor with name and velocity values provided
   * @brief name The name of the state
   * @brief velocities The vector of velocities
   */
  explicit JointVelocities(const std::string& robot_name, const Eigen::VectorXd& velocities);

  /**
   * @brief Constructor with name, a list of joint names and velocity values provided
   * @brief name The name of the state
   * @brief joint_names List of joint names
   * @brief velocities The vector of velocities
   */
  explicit JointVelocities(
      const std::string& robot_name, const std::vector<std::string>& joint_names, const Eigen::VectorXd& velocities
  );

  /**
   * @brief Copy constructor
   */
  JointVelocities(const JointVelocities& velocities);

  /**
   * @brief Copy constructor from a joint state
   */
  JointVelocities(const JointState& state);

  /**
   * @brief Integration constructor from joint accelerations by considering that it is equivalent to multiplying
   * the accelerations by 1 second
   */
  JointVelocities(const JointAccelerations& accelerations);

  /**
   * @brief Differentiation constructor from joint positions by considering that it is equivalent to dividing
   * the positions by 1 second
   */
  JointVelocities(const JointPositions& positions);

  /**
   * @brief Constructor for zero joint velocities
   * @param robot_name The name of the associated robot
   * @param nb_joints The number of joints for initialization
   * @return Joint velocities with zero values
   */
  static JointVelocities Zero(const std::string& robot_name, unsigned int nb_joints);

  /**
   * @brief Constructor for zero joint velocities
   * @param robot_name The name of the associated robot
   * @param joint_names List of joint names
   * @return Joint velocities with zero values
   */
  static JointVelocities Zero(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Constructor for random joint velocities
   * @param robot_name The name of the associated robot
   * @param nb_joints The number of joints for initialization
   * @return Joint velocities with random values
   */
  static JointVelocities Random(const std::string& robot_name, unsigned int nb_joints);

  /**
   * @brief Constructor for random joint velocities
   * @param robot_name The name of the associated robot
   * @param joint_names List of joint names
   * @return Joint velocities with random values
   */
  static JointVelocities Random(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Copy assignment operator that has to be defined to the custom assignment operator
   * @param velocities The state with value to assign
   * @return Reference to the current state with new values
   */
  JointVelocities& operator=(const JointVelocities& velocities) = default;

  /**
   * @brief Returns the velocities data as an Eigen vector
   * @return The velocities data vector
   */
  Eigen::VectorXd data() const override;

  /**
   * @brief Set the velocities data from an Eigen vector
   * @param data The velocities data vector
   */
  virtual void set_data(const Eigen::VectorXd& data) override;

  /**
   * @brief Set the velocities data from a std vector
   * @param data The velocities data vector
   */
  virtual void set_data(const std::vector<double>& data) override;

  /**
   * @brief Clamp inplace the magnitude of the velocity to the values in argument
   * @param max_absolute_value The maximum magnitude of velocity for all the joints
   * @param noise_ratio If provided, this value will be used to apply a dead zone under which
   * the velocity will be set to 0
   */
  void clamp(double max_absolute_value, double noise_ratio = 0.);

  /**
   * @brief Clamp inplace the magnitude of the velocity to the values in argument
   * @param max_absolute_value_array The maximum magnitude of velocity for each joint
   * @param noise_ratio_array If provided, this value will be used to apply a dead zone under which
   * the velocity will be set to 0
   */
  void clamp(const Eigen::ArrayXd& max_absolute_value_array, const Eigen::ArrayXd& noise_ratio_array);

  /**
   * @brief Return the velocity clamped to the values in argument
   * @param max_absolute_value The maximum magnitude of velocity for all the joints
   * @param noise_ratio If provided, this value will be used to apply a dead zone under which
   * the velocity will be set to 0
   * @return The clamped joint velocities
   */
  JointVelocities clamped(double max_absolute_value, double noise_ratio = 0.) const;

  /**
   * @brief Return the velocity clamped to the values in argument
   * @param max_absolute_value_array The maximum magnitude of velocity for each joint
   * @param noise_ratio_array If provided, this value will be used to apply a dead zone under which
   * the velocity will be set to 0
   * @return The clamped joint velocities
   */
  JointVelocities clamped(
      const Eigen::ArrayXd& max_absolute_value_array, const Eigen::ArrayXd& noise_ratio_array
  ) const;

  /**
   * @brief Return a copy of the joint velocities
   */
  JointVelocities copy() const;

  /**
   * @brief Scale inplace by a scalar
   * @copydetails JointState::operator*=(double)
   * @param lambda The scaling factor
   * @return The reference to the scaled joint velocities
   */
  JointVelocities& operator*=(double lambda);

  /**
   * @brief Scale joint velocities by a scalar
   * @copydetails JointState::operator*=(double)
   * @param lambda The scaling factor
   * @return The scaled joint velocities
   */
  JointVelocities operator*(double lambda) const;

  /**
   * @brief Scale joint velocities by a scalar
   * @copydetails JointState::operator*=(double)
   * @param lambda The scaling factor
   * @param velocities The joint velocities to be scaled
   * @return The scaled joint velocities
   */
  friend JointVelocities operator*(double lambda, const JointVelocities& velocities);

  /**
   * @brief Scale joint velocities by a matrix
   * @param lambda The scaling matrix
   * @param velocities The joint velocities to be scaled
   * @return The scaled joint velocities
   */
  friend JointVelocities operator*(const Eigen::MatrixXd& lambda, const JointVelocities& velocities);

  /**
   * @brief Integrate joint velocities over a time period
   * @param dt The time period used for integration
   * @return The resulting joint positions after integration
   */
  JointPositions operator*(const std::chrono::nanoseconds& dt) const;

  /**
   * @brief Integrate joint velocities over a time period
   * @param dt The time period used for integration
   * @param velocities The joint velocities to be integrated
   * @return The resulting joint positions after integration
   */
  friend JointPositions operator*(const std::chrono::nanoseconds& dt, const JointVelocities& velocities);

  /**
   * @brief Scale inplace by a scalar
   * @copydetails JointState::operator*=(double)
   * @param lambda The scaling factor
   * @return The reference to the scaled joint velocities
   */
  JointVelocities& operator/=(double lambda);

  /**
   * @brief Scale joint velocities by a scalar
   * @copydetails JointState::operator*=(double)
   * @param lambda The scaling factor
   * @return The scaled joint velocities
   */
  JointVelocities operator/(double lambda) const;

  /**
   * @brief Differentiate joint velocities pose over a time period
   * @param dt The time period used for derivation
   * @return The resulting joint accelerations after derivation
   */
  JointAccelerations operator/(const std::chrono::nanoseconds& dt) const;

  /**
   * @brief Add inplace other joint velocities
   * @param velocities Joint velocities with same name and same joint names
   * @return The reference to the combined joint velocities
   */
  JointVelocities& operator+=(const JointVelocities& velocities);

  /**
   * @brief Add inplace other joint velocities from a joint state
   * @param state A joint state with same name and same joint names
   * @return The reference to the combined joint velocities
   */
  JointVelocities& operator+=(const JointState& state);

  /**
   * @brief Add other joint velocities
   * @param accelerations Joint velocities with same name and same joint names
   * @return The combined joint velocities
   */
  JointVelocities operator+(const JointVelocities& velocities) const;

  /**
   * @brief Add another joint sate
   * @param state A joint state with same name and same joint names
   * @return The combined joint state
   */
  JointState operator+(const JointState& state) const;

  /**
   * @brief Overload the -= operator
   * @param velocities Joint velocities to subtract
   * @return The current joint velocities subtracted the joint velocities given in argument
   */
  JointVelocities& operator-=(const JointVelocities& velocities);

  /**
   * @brief Overload the - operator
   * @param velocities Joint velocities to subtract
   * @return The current joint velocities subtracted the joint velocities given in argument
   */
  JointVelocities operator-(const JointVelocities& velocities) const;

  /**
   * @brief Overload the ostream operator for printing
   * @param os The ostream to append the string representing the state
   * @param state The state to print
   * @return The appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const JointVelocities& velocities);

private:
  using JointState::clamp_state_variable;
};
}// namespace state_representation
