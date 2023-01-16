#pragma once

#include "state_representation/space/SpatialState.hpp"
#include "state_representation/exceptions/IncompatibleSizeException.hpp"

namespace state_representation {

class CartesianState;

/**
 * @enum CartesianStateAttribute
 * @brief Enum representing the attributes (position, orientation, angular_velocity, ...)
 * of the CartesianState
 */
enum class CartesianStateVariable {
  POSITION,
  ORIENTATION,
  POSE,
  LINEAR_VELOCITY,
  ANGULAR_VELOCITY,
  TWIST,
  LINEAR_ACCELERATION,
  ANGULAR_ACCELERATION,
  ACCELERATION,
  FORCE,
  TORQUE,
  WRENCH,
  ALL
};

/**
 * @brief Compute the distance between two Cartesian states
 * @param s1 The first Cartesian state
 * @param s2 The second Cartesian state
 * @param state_variable_type Name of the state variable from the CartesianStateVariable enum to apply
 * the distance on. Default ALL for full distance across all dimensions
 * @return The distance between the two states
 */
double dist(
    const CartesianState& s1, const CartesianState& s2,
    const CartesianStateVariable& state_variable_type = CartesianStateVariable::ALL
);

/**
 * @class CartesianState
 * @brief Class to represent a state in Cartesian space
 */
class CartesianState : public SpatialState {
public:
  /**
   * @brief Empty constructor
   */
  explicit CartesianState();

  /**
   * @brief Constructor with name and reference frame provided
   */
  explicit CartesianState(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Copy constructor of a Cartesian state
   */
  CartesianState(const CartesianState& state) = default;

  /**
   * @brief Constructor for the identity Cartesian state (identity pose and 0 for the rest)
   */
  static CartesianState Identity(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Constructor for a random Cartsian state
   */
  static CartesianState Random(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Swap the values of two Cartesian states
   * @param state1 Cartesian state to be swapped with 2
   * @param state2 Cartesian state to be swapped with 1
   */
  friend void swap(CartesianState& state1, CartesianState& state2);

  /**
   * @brief Copy assignment operator that has to be defined to the custom assignment operator
   * @param state The state with value to assign
   * @return Reference to the current state with new values
   */
  CartesianState& operator=(const CartesianState& state);

  /**
   * @brief Getter of the position attribute
   */
  const Eigen::Vector3d& get_position() const;

  /**
   * @brief Getter of the orientation attribute
   */
  const Eigen::Quaterniond& get_orientation() const;

  /**
   * @brief Getter of the orientation attribute as Vector4d of coefficients
   * @details Quaternion coefficients are returned using the (w, x, y, z) convention
   */
  Eigen::Vector4d get_orientation_coefficients() const;

  /**
   * @brief Getter of the pose from position and orientation attributes
   * @return The pose as a 7d vector. Quaternion coefficients are
   * returned using the (w, x, y, z) convention
   */
  Eigen::Matrix<double, 7, 1> get_pose() const;

  /**
   * @brief Getter of a pose from position and orientation attributes
   * @return The pose as a 4x4 transformation matrix
   */
  Eigen::Matrix4d get_transformation_matrix() const;

  /**
   * @brief Getter of the linear velocity attribute 
   */
  const Eigen::Vector3d& get_linear_velocity() const;

  /**
   * @brief Getter of the angular velocity attribute 
   */
  const Eigen::Vector3d& get_angular_velocity() const;

  /**
   * @brief Getter of the 6d twist from linear and angular velocity attributes
   */
  Eigen::Matrix<double, 6, 1> get_twist() const;

  /**
   * @brief Getter of the linear acceleration attribute 
   */
  const Eigen::Vector3d& get_linear_acceleration() const;

  /**
   * @brief Getter of the angular acceleration attribute 
   */
  const Eigen::Vector3d& get_angular_acceleration() const;

  /**
   * @brief Getter of the 6d acceleration from linear and angular acceleration attributes
   */
  Eigen::Matrix<double, 6, 1> get_acceleration() const;

  /**
   * @brief Getter of the force attribute
   */
  const Eigen::Vector3d& get_force() const;

  /**
   * @brief Getter of the torque attribute
   */
  const Eigen::Vector3d& get_torque() const;

  /**
   * @brief Getter of the 6d wrench from force and torque attributes
   */
  Eigen::Matrix<double, 6, 1> get_wrench() const;

  /**
   * @brief Return the data as the concatenation of all the state variables in a single vector
   */
  virtual Eigen::VectorXd data() const;

  /**
   * @brief Return the data vector as an Eigen Array
   */
  Eigen::ArrayXd array() const;

  /**
   * @brief Return the state as a std vector
   */
  std::vector<double> to_std_vector() const;

  /**
   * @brief Setter of the position
   */
  void set_position(const Eigen::Vector3d& position);

  /**
   * @brief Setter of the position from a std vector
   */
  void set_position(const std::vector<double>& position);

  /**
   * @brief Setter of the position from three scalar coordinates
   */
  void set_position(const double& x, const double& y, const double& z);

  /**
   * @brief Setter of the orientation
   */
  void set_orientation(const Eigen::Quaterniond& orientation);

  /**
   * @brief Setter of the orientation from a 4d vector
   * @param orientation The orientation coefficients as a 4d vector. Quaternion coefficients
   * use the (w, x, y, z) convention
   */
  void set_orientation(const Eigen::Vector4d& orientation);

  /**
   * @brief Setter of the orientation from a std vector
   * @param orientation The orientation coefficients as a 4d std vector. Quaternion coefficients
   * use the (w, x, y, z) convention
   */
  void set_orientation(const std::vector<double>& orientation);

  /**
   * @brief Setter of the orientation from four scalar coefficients (w, x, y, z)
   */
  void set_orientation(const double& w, const double& x, const double& y, const double& z);

  /**
   * @brief Setter of the pose from both position and orientation
   */
  void set_pose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);

  /**
   * @brief Setter of the pose from both position and orientation as Eigen 7d vector
   * @param pose The pose as a 7d vector. Quaternion coefficients
   * use the (w, x, y, z) convention
   */
  void set_pose(const Eigen::Matrix<double, 7, 1>& pose);

  /**
   * @brief Setter of the pose from both position and orientation as std vector
   * @param pose The pose as a 7d vector. Quaternion coefficients
   * use the (w, x, y, z) convention
   */
  void set_pose(const std::vector<double>& pose);

  /**
   * @brief Setter of the linear velocity attribute
   */
  void set_linear_velocity(const Eigen::Vector3d& linear_velocity);

  /**
   * @brief Setter of the linear velocity from a std vector
   */
  void set_linear_velocity(const std::vector<double>& linear_velocity);

  /**
   * @brief Setter of the linear velocity from three scalar coordinates
   */
  void set_linear_velocity(const double& x, const double& y, const double& z);

  /**
   * @brief Setter of the angular velocity attribute
   */
  void set_angular_velocity(const Eigen::Vector3d& angular_velocity);

  /**
   * @brief Setter of the angular velocity from a std vector
   */
  void set_angular_velocity(const std::vector<double>& angular_velocity);

  /**
   * @brief Setter of the angular velocity from three scalar coordinates
   */
  void set_angular_velocity(const double& x, const double& y, const double& z);

  /**
   * @brief Setter of the linear and angular velocities from a 6d twist vector
   */
  void set_twist(const Eigen::Matrix<double, 6, 1>& twist);

  /**
   * @brief Setter of the linear and angular velocities from a std vector
   */
  void set_twist(const std::vector<double>& twist);

  /**
   * @brief Setter of the linear acceleration attribute
   */
  void set_linear_acceleration(const Eigen::Vector3d& linear_acceleration);

  /**
   * @brief Setter of the linear acceleration from a std vector
   */
  void set_linear_acceleration(const std::vector<double>& linear_acceleration);

  /**
   * @brief Setter of the linear acceleration from three scalar coordinates
   */
  void set_linear_acceleration(const double& x, const double& y, const double& z);

  /**
   * @brief Setter of the angular velocity attribute
   */
  void set_angular_acceleration(const Eigen::Vector3d& angular_acceleration);

  /**
   * @brief Setter of the angular acceleration from a std vector
   */
  void set_angular_acceleration(const std::vector<double>& angular_acceleration);

  /**
   * @brief Setter of the angular acceleration from three scalar coordinates
   */
  void set_angular_acceleration(const double& x, const double& y, const double& z);

  /**
   * @brief Setter of the linear and angular acceleration from a 6d acceleration vector
   */
  void set_acceleration(const Eigen::Matrix<double, 6, 1>& acceleration);

  /**
   * @brief Setter of the linear and angular acceleration from a std vector
   */
  void set_acceleration(const std::vector<double>& acceleration);

  /**
   * @brief Setter of the force attribute
   */
  void set_force(const Eigen::Vector3d& force);

  /**
   * @brief Setter of the force from a std vector
   */
  void set_force(const std::vector<double>& force);

  /**
   * @brief Setter of the force from three scalar coordinates
   */
  void set_force(const double& x, const double& y, const double& z);

  /**
   * @brief Setter of the torque attribute
   */
  void set_torque(const Eigen::Vector3d& torque);

  /**
   * @brief Setter of the torque from a std vector
   */
  void set_torque(const std::vector<double>& torque);

  /**
   * @brief Setter of the torque from three scalar coordinates
   */
  void set_torque(const double& x, const double& y, const double& z);

  /**
   * @brief Setter of the force and torque from a 6d wrench vector
   */
  void set_wrench(const Eigen::Matrix<double, 6, 1>& wrench);

  /**
   * @brief Setter of the force and torque from a std vector
   */
  void set_wrench(const std::vector<double>& wrench);

  /**
   * @brief Set the data of the state from all the state variables in a single Eigen vector
   */
  virtual void set_data(const Eigen::VectorXd& data) override;

  /**
   * @brief Set the data of the state from all the state variables in a single std vector
   */
  virtual void set_data(const std::vector<double>& data) override;

  /**
   * @brief Set the State to a zero value
   */
  void set_zero();

  /**
   * @brief Clamp inplace the norm of the a specific state variable
   * @param max_norm The maximum norm of the state variable
   * @param state_variable_type Name of the variable from the CartesianStateVariable structure to clamp
   * @param noise_ratio If provided, this value will be used to apply a dead zone under which
   * the norm of the state variable will be set to 0
   */
  void clamp_state_variable(double max_norm, const CartesianStateVariable& state_variable_type, double noise_ratio = 0);

  /**
   * @brief Return a copy of the Cartesian state
   */
  CartesianState copy() const;

  /**
   * @brief Compute the distance to another state as the sum of distances between each features
   * @param state The second state
   * @param state_variable_type The name of the variable from the CartesianStateVariable structure to apply
   * the distance on. Default ALL for full distance across all dimensions
   * @return dist The distance value as a double
   */
  double dist(
      const CartesianState& state, const CartesianStateVariable& state_variable_type = CartesianStateVariable::ALL
  ) const;

  /**
   * @brief Compute the distance between two Cartesian states
   * @param s1 The first Cartesian state
   * @param s2 The second Cartesian state
   * @param state_variable_type Type of the distance between position, orientation, linear_velocity, etc.
   * Default ALL for full distance across all dimensions
   * @return The distance between the two states
   */
  friend double dist(
      const CartesianState& s1, const CartesianState& s2, const CartesianStateVariable& state_variable_type
  );

  /**
   * @brief Initialize the CartesianState to a zero value
   */
  void initialize() override;

  /**
   * @brief Compute the inverse of the current Cartesian state
   * @return The inverse corresponding to b_S_f (assuming this is f_S_b)
   */
  CartesianState inverse() const;

  /**
   * @brief Normalize inplace the state at the state variable given in argument. Default is full state
   * @param state_variable_type The type of state variable to compute the norms on
   */
  void normalize(const CartesianStateVariable& state_variable_type = CartesianStateVariable::ALL);

  /**
   * @brief Compute the normalized state at the state variable given in argument. Default is full state
   * @param state_variable_type The type of state variable to compute the norms on
   * @return The normalized state
   */
  CartesianState normalized(const CartesianStateVariable& state_variable_type = CartesianStateVariable::ALL) const;

  /**
   * @brief Compute the norms of the state variable specified by the input type. Default is full state
   * @param state_variable_type The type of state variable to compute the norms on
   * @return The norms of the state variables as a vector
   */
  virtual std::vector<double>
  norms(const CartesianStateVariable& state_variable_type = CartesianStateVariable::ALL) const;

  /**
   * @brief Overload the *= operator with another state by deriving the equations of motions
   * @param state The state to compose with corresponding to b_S_c
   * @return The Cartesian state corresponding f_S_c = f_S_b * b_S_c (assuming this is f_S_b)
   */
  CartesianState& operator*=(const CartesianState& state);

  /**
   * @brief Overload the * operator with another state by deriving the equations of motions
   * @param state The state to compose with corresponding to b_S_c
   * @return The Cartesian state corresponding f_S_c = f_S_b * b_S_c (assuming this is f_S_b)
   */
  CartesianState operator*(const CartesianState& state) const;

  /**
   * @brief Overload the *= operator with a scalar
   * @param lambda The scalar to multiply with
   * @return The Cartesian state multiplied by lambda
   */
  CartesianState& operator*=(double lambda);

  /**
   * @brief Overload the * operator with a scalar
   * @param lambda The scalar to multiply with
   * @return The Cartesian state multiplied by lambda
   */
  CartesianState operator*(double lambda) const;

  /**
   * @brief Overload the * operator with a scalar
   * @param lambda The scalar to multiply with
   * @return The Cartesian state provided multiplied by lambda
   */
  friend CartesianState operator*(double lambda, const CartesianState& state);

  /**
   * @brief Overload the /= operator with a scalar
   * @param lambda The scalar to divide with
   * @return The Cartesian state divided by lambda
   */
  CartesianState& operator/=(double lambda);

  /**
   * @brief Overload the / operator with a scalar
   * @param lambda The scalar to divide with
   * @return The Cartesian state divided by lambda
   */
  CartesianState operator/(double lambda) const;

  /**
   * @brief Overload the += operator
   * @param state The Cartesian state to add
   * @return The current Cartesian state added with the Cartesian state given in argument
   */
  CartesianState& operator+=(const CartesianState& state);

  /**
   * @brief Overload the + operator
   * @param state The Cartesian state to add
   * @return The current Cartesian state added with the Cartesian state given in argument
   */
  CartesianState operator+(const CartesianState& state) const;

  /**
   * @brief Overload the -= operator
   * @param state The Cartesian state to subtract
   * @return The current Cartesian state minus the Cartesian state given in argument
   */
  CartesianState& operator-=(const CartesianState& state);

  /**
   * @brief Overload the - operator
   * @param state The Cartesian state to subtract
   * @return The current Cartesian state minus the Cartesian state given in argument
   */
  CartesianState operator-(const CartesianState& state) const;

  /**
   * @brief Overload the ostream operator for printing
   * @param os The ostream to append the string representing the state to
   * @param state The state to print
   * @return The appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const CartesianState& state);

protected:
  /**
   * @brief Getter of the variable value corresponding to the input
   * @param state_variable_type The type of variable to get
   */
  Eigen::VectorXd get_state_variable(const CartesianStateVariable& state_variable_type) const;

  /**
   * @brief Setter of the variable value corresponding to the input
   * @param new_value The new value of the variable
   * @param state_variable_type The type of variable to get
   */
  void set_state_variable(const Eigen::VectorXd& new_value, const CartesianStateVariable& state_variable_type);

private:
  Eigen::Vector3d position_;            ///< position of the point
  Eigen::Quaterniond orientation_;      ///< orientation of the point
  Eigen::Vector3d linear_velocity_;     ///< linear velocity of the point
  Eigen::Vector3d angular_velocity_;    ///< angular velocity of the point
  Eigen::Vector3d linear_acceleration_; ///< linear acceleration of the point
  Eigen::Vector3d angular_acceleration_;///< angular acceleration of the point
  Eigen::Vector3d force_;               ///< force applied at the point
  Eigen::Vector3d torque_;              ///< torque applied at the point

  /**
   * @brief Set new values in all the state variables
   */
  void set_all_state_variables(const Eigen::VectorXd& new_values);

  /**
   * @brief Set new values in the provided state variable
   * @param state_variable The state variable to fill
   * @param new_value The new value of the state variable
   */
  void set_state_variable(Eigen::Vector3d& state_variable, const Eigen::Vector3d& new_value);

  /**
   * @brief Set new values in the provided state variable
   * @param state_variable The state variable to fill
   * @param new_value The new value of the state variable
   */
  void set_state_variable(Eigen::Vector3d& state_variable, const std::vector<double>& new_value);

  /**
   * @brief Set new values in the provided state variables
   * @param linear_state_variable The linear part of the state variable to fill
   * @param angular_state_variable The angular part of the state variable to fill
   * @param new_value The new value of the state variable
   */
  void set_state_variable(
      Eigen::Vector3d& linear_state_variable, Eigen::Vector3d& angular_state_variable,
      const Eigen::Matrix<double, 6, 1>& new_value
  );
};

inline void swap(CartesianState& state1, CartesianState& state2) {
  swap(static_cast<SpatialState&>(state1), static_cast<SpatialState&>(state2));
  std::swap(state1.position_, state2.position_);
  std::swap(state1.orientation_, state2.orientation_);
  std::swap(state1.linear_velocity_, state2.linear_velocity_);
  std::swap(state1.angular_velocity_, state2.angular_velocity_);
  std::swap(state1.linear_acceleration_, state2.linear_acceleration_);
  std::swap(state1.angular_acceleration_, state2.angular_acceleration_);
  std::swap(state1.force_, state2.force_);
  std::swap(state1.torque_, state2.torque_);
}

inline Eigen::VectorXd CartesianState::get_state_variable(const CartesianStateVariable& state_variable_type) const {
  switch (state_variable_type) {
    case CartesianStateVariable::POSITION:
      return this->get_position();

    case CartesianStateVariable::ORIENTATION:
      return this->get_orientation_coefficients();

    case CartesianStateVariable::POSE:
      return this->get_pose();

    case CartesianStateVariable::LINEAR_VELOCITY:
      return this->get_linear_velocity();

    case CartesianStateVariable::ANGULAR_VELOCITY:
      return this->get_angular_velocity();

    case CartesianStateVariable::TWIST:
      return this->get_twist();

    case CartesianStateVariable::LINEAR_ACCELERATION:
      return this->get_linear_acceleration();

    case CartesianStateVariable::ANGULAR_ACCELERATION:
      return this->get_angular_acceleration();

    case CartesianStateVariable::ACCELERATION:
      return this->get_acceleration();

    case CartesianStateVariable::FORCE:
      return this->get_force();

    case CartesianStateVariable::TORQUE:
      return this->get_torque();

    case CartesianStateVariable::WRENCH:
      return this->get_wrench();

    case CartesianStateVariable::ALL:
      Eigen::VectorXd all_fields(25);
      all_fields << this->get_pose(), this->get_twist(), this->get_acceleration(), this->get_wrench();
      return all_fields;
  }
  // this never goes here but is compulsory to avoid a warning
  return Eigen::Vector3d::Zero();
}

inline void CartesianState::set_all_state_variables(const Eigen::VectorXd& new_values) {
  if (new_values.size() != 25) {
    throw exceptions::IncompatibleSizeException(
        "Input is of incorrect size: expected 25, given " + std::to_string(new_values.size()));
  }
  this->set_pose(new_values.segment(0, 7));
  this->set_twist(new_values.segment(7, 6));
  this->set_acceleration(new_values.segment(13, 6));
  this->set_wrench(new_values.segment(19, 6));
}

inline void CartesianState::set_state_variable(Eigen::Vector3d& state_variable, const Eigen::Vector3d& new_value) {
  this->set_empty(false);
  state_variable = new_value;
}

inline void CartesianState::set_state_variable(Eigen::Vector3d& state_variable, const std::vector<double>& new_value) {
  if (new_value.size() != 3) {
    throw exceptions::IncompatibleSizeException(
        "Input vector is of incorrect size: expected 3, given " + std::to_string(new_value.size()));
  }
  this->set_state_variable(state_variable, Eigen::Vector3d::Map(new_value.data(), new_value.size()));
}

inline void CartesianState::set_state_variable(
    Eigen::Vector3d& linear_state_variable, Eigen::Vector3d& angular_state_variable,
    const Eigen::Matrix<double, 6, 1>& new_value
) {
  this->set_state_variable(linear_state_variable, new_value.head(3));
  this->set_state_variable(angular_state_variable, new_value.tail(3));
}

inline void CartesianState::set_state_variable(
    const Eigen::VectorXd& new_value, const CartesianStateVariable& state_variable_type
) {
  switch (state_variable_type) {
    case CartesianStateVariable::POSITION:
      this->set_position(new_value);
      break;

    case CartesianStateVariable::ORIENTATION:
      this->set_orientation(new_value);
      break;

    case CartesianStateVariable::POSE:
      this->set_pose(new_value);
      break;

    case CartesianStateVariable::LINEAR_VELOCITY:
      this->set_linear_velocity(new_value);
      break;

    case CartesianStateVariable::ANGULAR_VELOCITY:
      this->set_angular_velocity(new_value);
      break;

    case CartesianStateVariable::TWIST:
      this->set_twist(new_value);
      break;

    case CartesianStateVariable::LINEAR_ACCELERATION:
      this->set_linear_acceleration(new_value);
      break;

    case CartesianStateVariable::ANGULAR_ACCELERATION:
      this->set_angular_acceleration(new_value);
      break;

    case CartesianStateVariable::ACCELERATION:
      this->set_acceleration(new_value);
      break;

    case CartesianStateVariable::FORCE:
      this->set_force(new_value);
      break;

    case CartesianStateVariable::TORQUE:
      this->set_torque(new_value);
      break;

    case CartesianStateVariable::WRENCH:
      this->set_wrench(new_value);
      break;

    case CartesianStateVariable::ALL:
      this->set_pose(new_value.segment(0, 7));
      this->set_twist(new_value.segment(7, 6));
      this->set_acceleration(new_value.segment(13, 6));
      this->set_wrench(new_value.segment(19, 6));
      break;
  }
}

}// namespace state_representation
