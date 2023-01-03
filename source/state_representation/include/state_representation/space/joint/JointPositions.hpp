#pragma once

#include "state_representation/space/joint/JointState.hpp"
#include "state_representation/space/joint/JointVelocities.hpp"

namespace state_representation {

class JointVelocities;

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
   * @brief Constructor for the zero joint positions
   * @param robot_name The name of the associated robot
   * @param nb_joints The number of joints for initialization
   * @return Joint positions with zero values
   */
  static JointPositions Zero(const std::string& robot_name, unsigned int nb_joints);

  /**
   * @brief Constructor for the zero joint positions
   * @param robot_name The name of the associated robot
   * @param joint_names List of joint names
   * @return Joint positions with zero values
   */
  static JointPositions Zero(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Constructor for the zero joint positions
   * @param robot_name The name of the associated robot
   * @param nb_joints The number of joints for initialization
   * @return Joint positions with random values
   */
  static JointPositions Random(const std::string& robot_name, unsigned int nb_joints);

  /**
   * @brief Constructor for the zero joint positions
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
   * @brief Overload the *= operator with a double gain
   * @param lambda The gain to multiply with
   * @return The joint positions multiplied by lambda
   */
  JointPositions& operator*=(double lambda);

  /**
   * @brief Overload the * operator with a double gain
   * @param lambda The gain to multiply with
   * @return The joint positions multiplied by lambda
   */
  JointPositions operator*(double lambda) const;

  /**
   * @brief Overload the * operator with a scalar
   * @param lambda The scalar to multiply with
   * @return The joint positions multiplied by lambda
   */
  friend JointPositions operator*(double lambda, const JointPositions& positions);

  /**
   * @brief Overload the *= operator with a matrix of gains
   * @param lambda The gain matrix to multiply with
   * @return The joint positions multiplied by lambda
   */
  JointPositions& operator*=(const Eigen::MatrixXd& lambda);

  /**
   * @brief Overload the * operator with a matrix of gains
   * @param lambda The gain matrix to multiply with
   * @return The joint positions multiplied by lambda
   */
  JointPositions operator*(const Eigen::MatrixXd& lambda) const;

  /**
   * @brief Overload the * operator with a matrix of gains
   * @param lambda The matrix to multiply with
   * @return The joint positions multiplied by lambda
   */
  friend JointPositions operator*(const Eigen::MatrixXd& lambda, const JointPositions& positions);

  /**
   * @brief Overload the *= operator with an array of gains
   * @param lambda The gain array to multiply with
   * @return The joint positions multiplied by lambda
   */
  JointPositions& operator*=(const Eigen::ArrayXd& lambda);

  /**
   * @brief Overload the *= operator with an array of gains
   * @param lambda The gain array to multiply with
   * @return The joint positions multiplied by lambda
   */
  JointPositions operator*(const Eigen::ArrayXd& lambda) const;

  /**
   * @brief Overload the * operator with an array of gains
   * @param lambda The array to multiply with
   * @return The joint positions multiplied by lambda
   */
  friend JointPositions operator*(const Eigen::ArrayXd& lambda, const JointPositions& positions);

  /**
   * @brief Overload the /= operator with a scalar
   * @param lambda The scalar to divide with
   * @return The joint positions divided by lambda
   */
  JointPositions& operator/=(double lambda);

  /**
   * @brief Overload the / operator with a scalar
   * @param lambda The scalar to divide with
   * @return The joint positions divided by lambda
   */
  JointPositions operator/(double lambda) const;

  /**
   * @brief Overload the / operator with a time period
   * @param dt The time period to divide with
   * @return The joint velocities corresponding to the velocities over the time period
   */
  JointVelocities operator/(const std::chrono::nanoseconds& dt) const;

  /**
   * @brief Overload the += operator
   * @param positions Joint positions to add
   * @return The current joint positions added the joint positions given in argument
   */
  JointPositions& operator+=(const JointPositions& positions);

  /**
   * @brief Overload the + operator
   * @param positions Joint positions to add
   * @return The current joint positions added the joint positions given in argument
   */
  JointPositions operator+(const JointPositions& positions) const;

  /**
   * @brief Overload the -= operator
   * @param positions Joint positions to subtract
   * @return The current joint positions subtracted the joint positions given in argument
   */
  JointPositions& operator-=(const JointPositions& positions);

  /**
   * @brief Overload the - operator
   * @param positions Joint positions to subtract
   * @return The current joint positions subtracted the joint positions given in argument
   */
  JointPositions operator-(const JointPositions& positions) const;

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
