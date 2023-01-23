#pragma once

#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/space/cartesian/CartesianAcceleration.hpp"
#include "state_representation/space/cartesian/CartesianWrench.hpp"

namespace state_representation {

class CartesianPose;
class CartesianAcceleration;
class CartesianWrench;

/**
 * @class CartesianTwist
 * @brief Class to define twist in Cartesian space as 3D linear and angular velocity vectors
 */
class CartesianTwist : public CartesianState {
public:
  // delete inaccessible getter and setters
  const Eigen::Vector3d& get_position() const = delete;
  const Eigen::Quaterniond& get_orientation() const = delete;
  Eigen::Vector4d get_orientation_coefficients() const = delete;
  Eigen::Matrix<double, 7, 1> get_pose() const = delete;
  Eigen::Matrix4d get_transformation_matrix() const = delete;
  const Eigen::Vector3d& get_linear_acceleration() const = delete;
  const Eigen::Vector3d& get_angular_acceleration() const = delete;
  Eigen::Matrix<double, 6, 1> get_acceleration() const = delete;
  const Eigen::Vector3d& get_force() const = delete;
  const Eigen::Vector3d& get_torque() const = delete;
  Eigen::Matrix<double, 6, 1> get_wrench() const = delete;
  void set_position(const Eigen::Vector3d& position) = delete;
  void set_position(const std::vector<double>& position) = delete;
  void set_position(const double& x, const double& y, const double& z) = delete;
  void set_orientation(const Eigen::Quaterniond& orientation) = delete;
  void set_orientation(const Eigen::Vector4d& orientation) = delete;
  void set_orientation(const std::vector<double>& orientation) = delete;
  void set_orientation(const double& w, const double& x, const double& y, const double& z) = delete;
  void set_pose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) = delete;
  void set_pose(const Eigen::Matrix<double, 7, 1>& pose) = delete;
  void set_pose(const std::vector<double>& pose) = delete;
  void set_linear_acceleration(const Eigen::Vector3d& linear_acceleration) = delete;
  void set_linear_acceleration(const std::vector<double>& linear_acceleration) = delete;
  void set_linear_acceleration(const double& x, const double& y, const double& z) = delete;
  void set_angular_acceleration(const Eigen::Vector3d& angular_acceleration) = delete;
  void set_angular_acceleration(const std::vector<double>& angular_acceleration) = delete;
  void set_angular_acceleration(const double& x, const double& y, const double& z) = delete;
  void set_acceleration(const Eigen::Matrix<double, 6, 1>& acceleration) = delete;
  void set_acceleration(const std::vector<double>& acceleration) = delete;
  void set_force(const Eigen::Vector3d& force) = delete;
  void set_force(const std::vector<double>& force) = delete;
  void set_force(const double& x, const double& y, const double& z) = delete;
  void set_torque(const Eigen::Vector3d& torque) = delete;
  void set_torque(const std::vector<double>& torque) = delete;
  void set_torque(const double& x, const double& y, const double& z) = delete;
  void set_wrench(const Eigen::Matrix<double, 6, 1>& wrench) = delete;
  void set_wrench(const std::vector<double>& wrench) = delete;
  CartesianState operator*=(const CartesianState& state) = delete;
  CartesianState operator*(const CartesianState& state) = delete;
  friend CartesianState operator*=(const CartesianState& state, const CartesianTwist& twist) = delete;
  CartesianState& operator+=(const CartesianPose& pose) = delete;
  CartesianState& operator+=(const CartesianAcceleration& acceleration) = delete;
  CartesianState& operator+=(const CartesianWrench& wrench) = delete;
  CartesianState operator+(const CartesianPose& pose) const = delete;
  CartesianState operator+(const CartesianAcceleration& acceleration) const = delete;
  CartesianState operator+(const CartesianWrench& wrench) const = delete;

  /**
   * @brief Empty constructor
   */
  explicit CartesianTwist();

  /**
   * @brief Constructor with name and reference frame provided
   * @param name The name of the state
   * @param reference The name of the reference frame (default is "world")
   */
  explicit CartesianTwist(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Copy constructor
   */
  CartesianTwist(const CartesianTwist& twist);

  /**
   * @brief Copy constructor from a Cartesian state
   */
  CartesianTwist(const CartesianState& state);

  /**
   * @brief Copy constructor from a Cartesian pose by considering that
   * it is equivalent to dividing the pose by 1 second
   */
  CartesianTwist(const CartesianPose& pose);

  /**
   * @brief Copy constructor from a Cartesian acceleration by considering
   * that it is a twist over 1 second
   */
  CartesianTwist(const CartesianAcceleration& acceleration);

  /**
   * @brief Construct a Cartesian twist from a linear velocity given as a vector
   */
  explicit CartesianTwist(
      const std::string& name, const Eigen::Vector3d& linear_velocity, const std::string& reference = "world"
  );

  /**
   * @brief Construct a Cartesian twist from a linear velocity and angular velocity given as vectors.
   */
  explicit CartesianTwist(
      const std::string& name, const Eigen::Vector3d& linear_velocity, const Eigen::Vector3d& angular_velocity,
      const std::string& reference = "world"
  );

  /**
   * @brief Construct a Cartesian twist from a single 6d twist vector
   */
  explicit CartesianTwist(
      const std::string& name, const Eigen::Matrix<double, 6, 1>& twist, const std::string& reference = "world"
  );

  /**
   * @brief Constructor for the zero twist
   * @param name The name of the state
   * @param reference The name of the reference frame (default is "world")
   * @return The zero Cartesian twist
   */
  static CartesianTwist Zero(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Constructor for a random twist
   * @param name The name of the state
   * @param reference The name of the reference frame (default is "world")
   * @return The random Cartesian twist
   */
  static CartesianTwist Random(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Copy assignment operator that has to be defined to the custom assignment operator
   * @param twist The twist with value to assign
   * @return Reference to the current twist with new values
   */
  CartesianTwist& operator=(const CartesianTwist& twist) = default;

  /**
 * @brief Returns the twist data as an Eigen vector
 */
  Eigen::VectorXd data() const override;

  /**
   * @brief Set the twist data from an Eigen vector
   */
  void set_data(const Eigen::VectorXd& data) override;

  /**
   * @brief Set the twist data from a std vector
   */
  void set_data(const std::vector<double>& data) override;

  /**
 * @brief Clamp inplace the magnitude of the twist to the values in argument
 * @param max_linear The maximum magnitude of the linear velocity
 * @param max_angular The maximum magnitude of the angular velocity
 * @param linear_noise_ratio If provided, this value will be used to apply a deadzone under which
 * the linear velocity will be set to 0
 * @param angular_noise_ratio If provided, this value will be used to apply a deadzone under which
 * the angular velocity will be set to 0
 */
  void clamp(double max_linear, double max_angular, double linear_noise_ratio = 0, double angular_noise_ratio = 0);

  /**
   * @brief Return the clamped twist
   * @param max_linear The maximum magnitude of the linear velocity
   * @param max_angular The maximum magnitude of the angular velocity
   * @param noise_ratio If provided, this value will be used to apply a deadzone under which
   * the linear velocity will be set to 0
   * @param angular_noise_ratio If provided, this value will be used to apply a deadzone under which
   * the angular velocity will be set to 0
   * @return The clamped twist
   */
  CartesianTwist clamped(
      double max_linear, double max_angular, double noise_ratio = 0, double angular_noise_ratio = 0
  ) const;

  /**
   * @brief Return a copy of the Cartesian twist
   */
  CartesianTwist copy() const;

  /**
   * @brief Compute the inverse of the current Cartesian twist
   */
  CartesianTwist inverse() const;

  /**
   * @brief Compute the normalized twist at the state variable given in argument (default is full twist)
   * @param state_variable_type The type of state variable to compute the norms on
   * @return The normalized twist
   */
  CartesianTwist normalized(const CartesianStateVariable& state_variable_type = CartesianStateVariable::TWIST) const;

  /**
   * @brief Compute the norms of the state variable specified by the input type (default is full twist)
   * @param state_variable_type The type of state variable to compute the norms on
   * @return The norms of the state variables as a vector
   */
  std::vector<double>
  norms(const CartesianStateVariable& state_variable_type = CartesianStateVariable::TWIST) const override;

  /**
   * @brief Overload the * operator with a Cartesian state
   * @param state The state to multiply with
   * @return The Cartesian twist provided multiplied by the state
   */
  friend CartesianTwist operator*(const CartesianState& state, const CartesianTwist& twist);

  /**
   * @brief Overload the *= operator with a scalar
   * @param lambda The scalar to multiply with
   * @return The Cartesian twist multiplied by lambda
   */
  CartesianTwist& operator*=(double lambda);

  /**
   * @brief Overload the * operator with a scalar
   * @param lambda The scalar to multiply with
   * @return The Cartesian twist multiplied by lambda
   */
  CartesianTwist operator*(double lambda) const;

  /**
   * @brief Overload the * operator with a scalar
   * @param lambda The scalar to multiply with
   * @return The Cartesian twist provided multiplied by lambda
   */
  friend CartesianTwist operator*(double lambda, const CartesianTwist& twist);

  /**
   * @brief Overload the *= operator with a gain matrix
   * @param lambda The matrix to multiply with
   * @return The Cartesian twist multiplied by lambda
   */
  CartesianTwist& operator*=(const Eigen::Matrix<double, 6, 6>& lambda);

  /**
   * @brief Overload the * operator with a gain matrix
   * @param lambda The matrix to multiply with
   * @return The Cartesian twist provided multiplied by lambda
   */
  friend CartesianTwist operator*(const Eigen::Matrix<double, 6, 6>& lambda, const CartesianTwist& twist);

  /**
   * @brief Overload the * operator with a time period
   * @param dt The time period to multiply with
   * @return The Cartesian pose corresponding to the displacement over the time period
   */
  CartesianPose operator*(const std::chrono::nanoseconds& dt) const;

  /**
   * @brief Overload the * operator with a time period
   * @param dt The time period to multiply with
   * @return The Cartesian pose corresponding to the displacement over the time period
   */
  friend CartesianPose operator*(const std::chrono::nanoseconds& dt, const CartesianTwist& twist);

  /**
   * @brief Overload the /= operator with a scalar
   * @param lambda The scalar to divide with
   * @return The Cartesian twist divided by lambda
   */
  CartesianTwist& operator/=(double lambda);

  /**
   * @brief Overload the / operator with a scalar
   * @param lambda The scalar to divide with
   * @return The Cartesian twist divided by lambda
   */
  CartesianTwist operator/(double lambda) const;

  /**
   * @brief Overload the / operator with a time period
   * @param dt The time period to divide by
   * @return The corresponding Cartesian acceleration
   */
  CartesianAcceleration operator/(const std::chrono::nanoseconds& dt) const;

  /**
   * @brief Overload the += operator
   * @param twist The Cartesian twist to add to
   * @return The current Cartesian twist added the Cartesian twist given in argument
   */
  CartesianTwist& operator+=(const CartesianTwist& twist);

  /**
   * @brief Overload the += operator with a state
   * @param state The Cartesian state to add to
   * @return The current Cartesian twist added the Cartesian state given in argument
   */
  CartesianTwist& operator+=(const CartesianState& state);

  /**
   * @brief Overload the + operator with a twist
   * @param twist The Cartesian twist to add to
   * @return The current Cartesian twist added the Cartesian twist given in argument
   */
  CartesianTwist operator+(const CartesianTwist& twist) const;

  /**
   * @brief Overload the + operator with a state
   * @param state The Cartesian state to add to
   * @return the current Cartesian pose added the Cartesian state given in argument
   */
  CartesianState operator+(const CartesianState& state) const;

  /**
   * @brief Overload the -= operator
   * @param twist The Cartesian twist to subtract
   * @return The current Cartesian twist minus the Cartesian twist given in argument
   */
  CartesianTwist& operator-=(const CartesianTwist& twist);

  /**
   * @brief Overload the - operator with a twist
   * @param twist The Cartesian twist to subtract
   * @return The current Cartesian twist minus the Cartesian twist given in argument
   */
  CartesianTwist operator-(const CartesianTwist& twist) const;

  /**
   * @brief Overload the ostream operator for printing
   * @param os The ostream to append the string representing the Cartesian twist to
   * @param CartesianTwist The Cartesian twist to print
   * @return The appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const CartesianTwist& twist);

private:
  using CartesianState::clamp_state_variable;
};

}// namespace state_representation
