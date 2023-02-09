#pragma once

#include "state_representation/State.hpp"
#include "state_representation/exceptions/IncompatibleSizeException.hpp"

namespace state_representation {

class JointState;

/**
 * @enum JointStateVariable
 * @brief Enum representing all the fields (positions, velocities, accelerations and torques)
 * of the JointState
 */
enum class JointStateVariable {
  POSITIONS, VELOCITIES, ACCELERATIONS, TORQUES, ALL
};

/**
 * @brief Compute the distance between two joint states
 * @param s1 The first joint state
 * @param s2 The second joint state
 * @param state_variable_type Name of the field from the JointStateVariable structure to apply
 * the distance on (default ALL for full distance across all dimensions)
 * @return The distance between the two states
 */
double dist(
    const JointState& s1, const JointState& s2, const JointStateVariable& state_variable_type = JointStateVariable::ALL
);

/**
 * @class JointState
 * @brief Class to define a state in joint space
 */
class JointState : public State {
public:
  /**
   * @brief Empty constructor for a joint state
   */
  JointState();

  /**
   * @brief Constructor with name and number of joints provided
   * @param robot_name The name of the associated robot
   * @param nb_joints The number of joints for initialization
   */
  explicit JointState(const std::string& robot_name, unsigned int nb_joints = 0);

  /**
   * @brief Constructor with name and list of joint names provided
   * @param robot_name The name of the associated robot
   * @param joint_names List of joint names
   */
  JointState(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Copy constructor of a joint state
   * @param state The joint state to copy from
   */
  JointState(const JointState& state);

  /**
   * @brief Constructor for a zero joint state
   * @param robot_name The name of the associated robot
   * @param nb_joints The number of joints for initialization
   * @return Joint state with zero values in all attributes
   */
  static JointState Zero(const std::string& robot_name, unsigned int nb_joints);

  /**
   * @brief Constructor for a zero joint state
   * @param robot_name The name of the associated robot
   * @param joint_names List of joint names
   * @return Joint state with zero values in all attributes
   */
  static JointState Zero(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Constructor for a random joint state
   * @param robot_name The name of the associated robot
   * @param nb_joints The number of joints for initialization
   * @return Joint state with random values in all attributes
   */
  static JointState Random(const std::string& robot_name, unsigned int nb_joints);

  /**
   * @brief Constructor for a random joint state
   * @param robot_name The name of the associated robot
   * @param joint_names List of joint names
   * @return Joint state with random values in all attributes
   */
  static JointState Random(const std::string& robot_name, const std::vector<std::string>& joint_names);

  /**
   * @brief Swap the values of the two joint states
   * @param state1 Joint state to be swapped with 2
   * @param state2 Joint state to be swapped with 1
   */
  friend void swap(JointState& state1, JointState& state2);

  /**
   * @brief Copy assignment operator that has to be defined to the custom assignment operator
   * @param state The state with value to assign
   * @return Reference to the current state with new values
   */
  JointState& operator=(const JointState& state);

  /**
   * @brief Getter of the size from the attributes
   */
  unsigned int get_size() const;

  /**
   * @brief Getter of the names attribute
   * @return The vector of strings containing the joint names
   */
  const std::vector<std::string>& get_names() const;

  /**
   * @brief Get joint index by the name of the joint, if it exists
   * @param joint_name The name of the desired joint
   * @raises JointNotFoundException if the desired joint doesn't exist
   * @return The index of the joint, if it exists
   */
  unsigned int get_joint_index(const std::string& joint_name) const;

  /**
   * @brief Getter of the positions attribute
   * @return The joint positions
   */
  const Eigen::VectorXd& get_positions() const;

  /**
   * @brief Get the position of a joint by its name, if it exists
   * @param joint_name The name of the joint
   * @raises JointNotFoundException if the desired joint doesn't exist
   * @return The position of the joint, if it exists
   */
  double get_position(const std::string& joint_name) const;

  /**
   * @brief Get the position of a joint by its index, if it exists
   * @param joint_index The index of the joint
   * @raises JointNotFoundException if the desired joint doesn't exist
   * @return The position of the joint, if it exists
   */
  double get_position(unsigned int joint_index) const;

  /**
   * @brief Getter of the velocities attribute
   * @return The joint velocities
   */
  const Eigen::VectorXd& get_velocities() const;

  /**
   * @brief Get the velocity of a joint by its name, if it exists
   * @param joint_name The name of the joint
   * @raises JointNotFoundException if the desired joint doesn't exist
   * @return The velocity of the joint, if it exists
   */
  double get_velocity(const std::string& joint_name) const;

  /**
   * @brief Get the velocity of a joint by its index, if it exists
   * @param joint_index The index of the joint
   * @raises JointNotFoundException if the desired joint doesn't exist
   * @return The velocity of the joint, if it exists
   */
  double get_velocity(unsigned int joint_index) const;

  /**
   * @brief Getter of the accelerations attribute
   * @return The joint accelerations
   */
  const Eigen::VectorXd& get_accelerations() const;

  /**
   * @brief Get the acceleration of a joint by its name, if it exists
   * @param joint_name The name of the joint
   * @raises JointNotFoundException if the desired joint doesn't exist
   * @return The acceleration of the joint, if it exists
   */
  double get_acceleration(const std::string& joint_name) const;

  /**
   * @brief Get the acceleration of a joint by its index, if it exists
   * @param joint_index The index of the joint
   * @raises JointNotFoundException if the desired joint doesn't exist
   * @return The acceleration of the joint, if it exists
   */
  double get_acceleration(unsigned int joint_index) const;

  /**
   * @brief Getter of the torques attribute
   * @return The joint torques
   */
  const Eigen::VectorXd& get_torques() const;

  /**
   * @brief Get the torque of a joint by its name, if it exists
   * @param joint_name The name of the joint
   * @raises JointNotFoundException if the desired joint doesn't exist
   * @return The torque of the joint, if it exists
   */
  double get_torque(const std::string& joint_name) const;

  /**
   * @brief Get the torque of a joint by its index, if it exists
   * @param joint_index The index of the joint
   * @raises JointNotFoundException if the desired joint doesn't exist
   * @return The torque of the joint, if it exists
   */
  double get_torque(unsigned int joint_index) const;

  /**
   * @brief Returns the data as the concatenation of all the state variables in a single vector
   * @return The concatenated data vector
   */
  virtual Eigen::VectorXd data() const;

  /**
   * @brief Returns the data vector as an Eigen array
   * @return The concatenated data array
   */
  Eigen::ArrayXd array() const;

  /**
   * @brief Setter of the names attribute from the number of joints
   * @param nb_joints The number of joints of the joint state
   */
  void set_names(unsigned int nb_joints);

  /**
   * @brief Setter of the names attribute
   * @param names The vector of strings containing the joint names
   */
  void set_names(const std::vector<std::string>& names);

  /**
   * @brief Setter of the positions attribute
   * @param positions The joint positions as Eigen vector
   */
  void set_positions(const Eigen::VectorXd& positions);

  /**
   * @brief Setter of the positions from std vector
   * @param positions The joint positions as std vector
   */
  void set_positions(const std::vector<double>& positions);

  /**
   * @brief Set the position of a joint by its name
   * @param position The position of the joint
   * @param joint_name The name of the joint
   * @raises JointNotFoundException if the desired joint doesn't exist
   */
  void set_position(double position, const std::string& joint_name);

  /**
   * @brief Set the position of a joint by its index
   * @param position The position of the joint
   * @param joint_index The index of the joint
   * @raises JointNotFoundException if the desired joint doesn't exist
   */
  void set_position(double position, unsigned int joint_index);

  /**
   * @brief Setter of the velocities attribute
   * @param velocities The joint velocities as Eigen vector
   */
  void set_velocities(const Eigen::VectorXd& velocities);

  /**
   * @brief Setter of the velocities from std vector
   * @param velocities The joint velocities as std vector
   */
  void set_velocities(const std::vector<double>& velocities);

  /**
   * @brief Set the velocity of a joint by its name
   * @param velocity The velocity of the joint
   * @param joint_name The name of the joint
   * @raises JointNotFoundException if the desired joint doesn't exist
   */
  void set_velocity(double velocity, const std::string& joint_name);

  /**
   * @brief Set the velocity of a joint by its index
   * @param velocity The velocity of the joint
   * @param joint_index The index of the joint
   * @raises JointNotFoundException if the desired joint doesn't exist
   */
  void set_velocity(double velocity, unsigned int joint_index);

  /**
   * @brief Setter of the accelerations attribute
   * @param accelerations The joint accelerations as Eigen vector
   */
  void set_accelerations(const Eigen::VectorXd& accelerations);

  /**
   * @brief Setter of the accelerations from std vector
   * @param accelerations The joint accelerations as std vector
   */
  void set_accelerations(const std::vector<double>& accelerations);

  /**
   * @brief Set the acceleration of a joint by its name
   * @param acceleration The acceleration of the joint
   * @param joint_name The name of the joint
   * @raises JointNotFoundException if the desired joint doesn't exist
   */
  void set_acceleration(double acceleration, const std::string& joint_name);

  /**
   * @brief Set the acceleration of a joint by its index
   * @param acceleration The acceleration of the joint
   * @param joint_index The index of the joint
   * @raises JointNotFoundException if the desired joint doesn't exist
   */
  void set_acceleration(double acceleration, unsigned int joint_index);

  /**
   * @brief Setter of the torques attribute
   * @param torques The joint torques as Eigen vector
   */
  void set_torques(const Eigen::VectorXd& torques);

  /**
   * @brief Setter of the torques attributes
   * @param torques The joint torques as std vector
   */
  void set_torques(const std::vector<double>& torques);

  /**
   * @brief Set the torque of a joint by its name
   * @param torque The torque of the joint
   * @param joint_name The name of the joint
   * @raises JointNotFoundException if the desired joint doesn't exist
   */
  void set_torque(double torque, const std::string& joint_name);

  /**
   * @brief Set the torque of a joint by its index
   * @param torque The torque of the joint
   * @param joint_index The index of the joint
   * @raises JointNotFoundException if the desired joint doesn't exist
   */
  void set_torque(double torque, unsigned int joint_index);

  /**
   * @brief Set the data of the state from all the state variables in a single Eigen vector
   * @param The concatenated data vector
   */
  virtual void set_data(const Eigen::VectorXd& data) override;

  /**
   * @brief Set the data of the state from all the state variables in a single std vector
   * @param The concatenated data vector
   */
  virtual void set_data(const std::vector<double>& data) override;

  /**
   * @brief Clamp inplace the magnitude of the a specific joint state variable
   * @param max_absolute_value The maximum absolute value of the state variable
   * @param state_variable_type Name of the variable from the JointStateVariable structure to clamp
   * @param noise_ratio If provided, this value will be used to apply a dead zone relative to the maximum absolute value
   * under which the state variable will be set to 0
   */
  void clamp_state_variable(
      double max_absolute_value, const JointStateVariable& state_variable_type, double noise_ratio = 0
  );

  /**
   * @brief Clamp inplace the magnitude of the a specific joint state variable
   * for each individual joint
   * @param max_absolute_value_array The maximum absolute value of the state variable for each joint individually
   * @param state_variable_type Name of the variable from the JointStateVariable structure to clamp
   * @param noise_ratio_array Those values will be used to apply a dead zone relative to the maximum absolute value
   * under which the state variable will be set to 0 for each individual joint
   */
  void clamp_state_variable(
      const Eigen::ArrayXd& max_absolute_value_array, const JointStateVariable& state_variable_type,
      const Eigen::ArrayXd& noise_ratio_array
  );

  /**
   * @brief Return a copy of the joint state
   */
  JointState copy() const;

  /**
   * @brief Compute the distance to another state as the sum of distances between each attribute
   * @param state The second state
   * @param state_variable_type Name of the variable from the JointStateVariable structure to apply
   * the distance on (default ALL for full distance across all dimensions)
   * @return dist The distance value as a double
   */
  double dist(const JointState& state, const JointStateVariable& state_variable_type = JointStateVariable::ALL) const;

  /**
   * @brief Compute the distance between two joint states
   * @param s1 The first joint state
   * @param s2 The second joint state
   * @param state_variable_type Name of the field from the JointStateVariable structure to apply
   * the distance on (default ALL for full distance across all dimensions)
   * @return The distance between the two states
   */
  friend double dist(const JointState& s1, const JointState& s2, const JointStateVariable& state_variable_type);

  /**
   * @copybrief State::reset
   */
  void reset() override;

  /**
   * @brief Check if the joint state is incompatible for operations with the state given as argument
   * @param state The state to check compatibility with
   */
  bool is_incompatible(const State& state) const override;

  /**
   * @brief Set the joint state to a zero value
   */
  void set_zero();

  /**
   * @brief Return the joint state as a std vector
   * @return The joint data as a std vector
   */
  std::vector<double> to_std_vector() const;

  /**
   * @brief Scale inplace by a scalar
   * @details All joints in all the state variables are scaled by the same factor.
   * @param lambda The scaling factor
   * @return The reference to the scaled joint state
   */
  JointState& operator*=(double lambda);

  /**
   * @brief Scale a joint state by a scalar
   * @copydetails JointState::operator*=(double)
   * @param lambda The scaling factor
   * @return The scaled joint state
   */
  JointState operator*(double lambda) const;

  /**
   * @brief Scale a joint state by a scalar
   * @copydetails JointState::operator*=(double)
   * @param lambda The scaling factor
   * @param state The joint state to be scaled
   * @return The scaled joint state
   */
  friend JointState operator*(double lambda, const JointState& state);

  /**
   * @brief Scale a joint state by a matrix
   * @param lambda The scaling matrix
   * @param state The joint state to be scaled
   * @return The scaled joint state
   */
  friend JointState operator*(const Eigen::MatrixXd& lambda, const JointState& state);

  /**
   * @brief Scale inplace by a scalar
   * @copydetails JointState::operator*=(double)
   * @param lambda The scaling factor
   * @return The reference to the scaled joint state
   */
  JointState& operator/=(double lambda);

  /**
   * @brief Scale a joint state by a scalar
   * @copydetails JointState::operator*=(double)
   * @param lambda The scaling factor
   * @return The scaled joint state
   */
  JointState operator/(double lambda) const;

  /**
   * @brief Add inplace another joint state
   * @param state A joint state with same name and same joint names
   * @return The reference to the combined joint state
   */
  JointState& operator+=(const JointState& state);

  /**
   * @brief Add another joint sate
   * @param state A joint state with same name and same joint names
   * @return The combined joint state
   */
  JointState operator+(const JointState& state) const;

  /**
   * @brief Negate a joint state
   * @return The negative value of the joint state
   */
  JointState operator-() const;

  /**
   * @brief Compute inplace the difference with another joint state
   * @param state A joint state with same name and same joint names
   * @return The reference to the difference in all the state variables
   */
  JointState& operator-=(const JointState& state);

  /**
   * @brief Compute the difference with another joint state
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
  friend std::ostream& operator<<(std::ostream& os, const JointState& state);

protected:
  /**
   * @brief Proxy function that scale the specified state variable by a matrix
   * @param lambda The scaling matrix
   * @param state_variable_type The state variable on which to apply the scaling
   */
  void multiply_state_variable(const Eigen::MatrixXd& lambda, const JointStateVariable& state_variable_type);

  /**
   * @brief Getter of the variable value corresponding to the input
   * @param state_variable_type The type of variable to get
   * @return The value of the variable as a vector
   */
  Eigen::VectorXd get_state_variable(const JointStateVariable& state_variable_type) const;

  /**
   * @brief Setter of the variable value corresponding to the input
   * @param new_value The new value of the variable as Eigen vector
   * @param state_variable_type The type of variable to set
   */
  void set_state_variable(const Eigen::VectorXd& new_value, const JointStateVariable& state_variable_type);

  /**
   * @brief Setter of the variable value corresponding to the input
   * @param new_value The new value of the variable as std vector
   * @param state_variable_type The type of variable to set
   */
  void set_state_variable(const std::vector<double>& new_value, const JointStateVariable& state_variable_type);

  /**
   * @brief Setter of the variable value corresponding to the input
   * @param new_value The new value of the variable as Eigen vector
   * @param joint_index The index at which to set the new value
   * @param state_variable_type The type of variable to set
   * @raises JointNotFoundException if the desired joint doesn't exist
   */
  void set_state_variable(double new_value, unsigned int joint_index, const JointStateVariable& state_variable_type);

  /**
   * @copydoc State::to_string
   */
  std::string to_string() const override;

private:
  /**
   * @brief Resize the data vectors to a new size
   * @param size The desired size
   */
  void resize(unsigned int size);

  std::vector<std::string> names_;///< names of the joints
  Eigen::VectorXd positions_;     ///< joints positions
  Eigen::VectorXd velocities_;    ///< joints velocities
  Eigen::VectorXd accelerations_; ///< joints accelerations
  Eigen::VectorXd torques_;       ///< joints torques
};

inline void swap(JointState& state1, JointState& state2) {
  swap(static_cast<State&>(state1), static_cast<State&>(state2));
  std::swap(state1.names_, state2.names_);
  std::swap(state1.positions_, state2.positions_);
  std::swap(state1.velocities_, state2.velocities_);
  std::swap(state1.accelerations_, state2.accelerations_);
  std::swap(state1.torques_, state2.torques_);
}
}// namespace state_representation
