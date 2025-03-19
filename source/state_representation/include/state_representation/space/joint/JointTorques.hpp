#pragma once

#include "state_representation/space/joint/JointAccelerations.hpp"
#include "state_representation/space/joint/JointPositions.hpp"
#include "state_representation/space/joint/JointState.hpp"
#include "state_representation/space/joint/JointVelocities.hpp"

namespace state_representation {

class JointPositions;
class JointVelocities;
class JointAccelerations;

/**
 * @class JointTorques
 * @brief Class to define torques of the joints
 */
class JointTorques : public JointState {
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
  JointState& operator+=(const JointPositions& positions) = delete;
  JointState& operator+=(const JointVelocities& velocities) = delete;
  JointState& operator+=(const JointAccelerations& accelerations) = delete;
  JointState operator+(const JointPositions& positions) const = delete;
  JointState operator+(const JointVelocities& velocities) const = delete;
  JointState operator+(const JointAccelerations& accelerations) const = delete;
  JointState& operator-=(const JointPositions& positions) = delete;
  JointState& operator-=(const JointVelocities& velocities) = delete;
  JointState& operator-=(const JointAccelerations& accelerations) = delete;
  JointState operator-(const JointPositions& positions) const = delete;
  JointState operator-(const JointVelocities& velocities) const = delete;
  JointState operator-(const JointAccelerations& accelerations) const = delete;

  /**
   * @brief Empty constructor
   */
  explicit JointTorques();

  /**
   * @brief Constructor with name and number of joints provided
   * @brief name The name of the state
   * @brief nb_joints The number of joints for initialization
   */
  explicit JointTorques(const std::string& robot_name, unsigned int nb_joints = 0);

  /**
   * @brief Constructor with name and list of joint names provided
   * @brief name The name of the state
   * @brief joint_names List of joint names
   */
  explicit JointTorques(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Constructor with name and torque values provided
   * @brief name The name of the state
   * @brief torques The vector of torques
   */
  explicit JointTorques(const std::string& robot_name, const Eigen::VectorXd& torques);

  /**
   * @brief Constructor with name, a list of joint names and torque values provided
   * @brief name The name of the state
   * @brief joint_names List of joint names
   * @brief torques The vector of torques
   */
  explicit JointTorques(
      const std::string& robot_name, const std::vector<std::string>& joint_names, const Eigen::VectorXd& torques
  );

  /**
   * @brief Copy constructor
   */
  JointTorques(const JointTorques& torques);

  /**
   * @brief Copy constructor from a joint state
   */
  JointTorques(const JointState& state);

  /**
   * @brief Constructor for zero joint torques
   * @param robot_name The name of the associated robot
   * @param nb_joints The number of joints for initialization
   * @return Joint torques with zero values
   */
  static JointTorques Zero(const std::string& robot_name, unsigned int nb_joints);

  /**
   * @brief Constructor for zero joint torques
   * @param robot_name The name of the associated robot
   * @param joint_names List of joint names
   * @return Joint torques with zero values
   */
  static JointTorques Zero(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Constructor for random joint torques
   * @param robot_name The name of the associated robot
   * @param nb_joints The number of joints for initialization
   * @return Joint torques with random values
   */
  static JointTorques Random(const std::string& robot_name, unsigned int nb_joints);

  /**
   * @brief Constructor for random joint torques
   * @param robot_name The name of the associated robot
   * @param joint_names List of joint names
   * @return Joint torques with random values
   */
  static JointTorques Random(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Copy assignment operator that has to be defined to the custom assignment operator
   * @param torques The state with value to assign
   * @return Reference to the current state with new values
   */
  JointTorques& operator=(const JointTorques& torques) = default;

  /**
   * @brief Returns the torques data as an Eigen vector
   * @return The torque data vector
   */
  Eigen::VectorXd data() const override;

  /**
   * @brief Set the torques data from an Eigen vector
   * @param data The torques data vector
   */
  virtual void set_data(const Eigen::VectorXd& data) override;

  /**
   * @brief Set the torques data from a std vector
   * @param data The torques data vector
   */
  virtual void set_data(const std::vector<double>& data) override;

  /**
   * @brief Clamp inplace the magnitude of the torque to the values in argument
   * @param max_absolute_value the maximum magnitude of torque for all the joints
   * @param noise_ratio If provided, this value will be used to apply a dead zone under which
   * the torque will be set to 0
   */
  void clamp(double max_absolute_value, double noise_ratio = 0.);

  /**
   * @brief Clamp inplace the magnitude of the torque to the values in argument
   * @param max_absolute_value_array The maximum magnitude of torque for each joint
   * @param noise_ratio_array If provided, this value will be used to apply a dead zone under which
   * the torque will be set to 0
   */
  void clamp(const Eigen::ArrayXd& max_absolute_value_array, const Eigen::ArrayXd& noise_ratio_array);

  /**
   * @brief Return the torque clamped to the values in argument
   * @param max_absolute_value The maximum magnitude of torque for all the joints
   * @param noise_ratio If provided, this value will be used to apply a dead zone under which
   * the torque will be set to 0
   * @return The clamped joint torques
   */
  JointTorques clamped(double max_absolute_value, double noise_ratio = 0.) const;

  /**
   * @brief Return the torque clamped to the values in argument
   * @param max_absolute_value_array The maximum magnitude of torque for each joint
   * @param noise_ratio_array If provided, this value will be used to apply a dead zone under which
   * the torque will be set to 0
   * @return The clamped joint torques
   */
  JointTorques clamped(const Eigen::ArrayXd& max_absolute_value_array, const Eigen::ArrayXd& noise_ratio_array) const;

  /**
   * @brief Return a copy of the joint torques
   */
  JointTorques copy() const;

  /**
   * @brief Scale inplace by a scalar
   * @copydetails JointState::operator*=(double)
   * @param lambda The scaling factor
   * @return The reference to the scaled joint torques
   */
  JointTorques& operator*=(double lambda);

  /**
   * @brief Scale joint torques by a scalar
   * @copydetails JointState::operator*=(double)
   * @param lambda The scaling factor
   * @return The scaled joint torques
   */
  JointTorques operator*(double lambda) const;

  /**
   * @brief Scale joint torques by a scalar
   * @copydetails JointState::operator*=(double)
   * @param lambda The scaling factor
   * @param torques The joint torques to be scaled
   * @return The scaled joint torques
   */
  friend JointTorques operator*(double lambda, const JointTorques& torques);

  /**
   * @brief Scale joint torques by a matrix
   * @param lambda The scaling matrix
   * @param torques The joint torques to be scaled
   * @return The scaled joint torques
   */
  friend JointTorques operator*(const Eigen::MatrixXd& lambda, const JointTorques& torques);

  /**
   * @brief Scale inplace by a scalar
   * @copydetails JointState::operator*=(double)
   * @param lambda The scaling factor
   * @return The reference to the scaled joint torques
   */
  JointTorques& operator/=(double lambda);

  /**
   * @brief Scale joint torques by a scalar
   * @copydetails JointState::operator*=(double)
   * @param lambda The scaling factor
   * @return The scaled joint torques
   */
  JointTorques operator/(double lambda) const;

  /**
   * @brief Add inplace other joint torques
   * @param torques Joint torques with same name and same joint names
   * @return The reference to the combined joint torques
   */
  JointTorques& operator+=(const JointTorques& torques);

  /**
   * @brief Add inplace other joint torques from a joint state
   * @param state A joint state with same name and same joint names
   * @return The reference to the combined joint torques
   */
  JointTorques& operator+=(const JointState& state);

  /**
   * @brief Add other joint torques
   * @param torques Joint torques with same name and same joint names
   * @return The combined joint torques
   */
  JointTorques operator+(const JointTorques& torques) const;

  /**
   * @brief Add another joint sate
   * @param state A joint state with same name and same joint names
   * @return The combined joint state
   */
  JointState operator+(const JointState& state) const;

  /**
   * @brief Negate joint torques
   * @return The negative value of the joint torques
   */
  JointTorques operator-() const;

  /**
   * @brief Compute inplace the difference with other joint torques
   * @param torques Joint torques with same name and same joint names
   * @return The reference to the difference in torques
   */
  JointTorques& operator-=(const JointTorques& torques);

  /**
   * @brief Compute inplace the difference with a joint state
   * @param state A joint state with same name and same joint names
   * @return The reference to the difference in torques
   */
  JointTorques& operator-=(const JointState& state);

  /**
   * @brief Compute the difference with other joint torques
   * @param torques Joint torques with same name and same joint names
   * @return The difference in torques
   */
  JointTorques operator-(const JointTorques& torques) const;

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
  friend std::ostream& operator<<(std::ostream& os, const JointTorques& torques);

private:
  using JointState::clamp_state_variable;
};
}// namespace state_representation
