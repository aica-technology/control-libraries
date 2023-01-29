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
  CartesianState& operator*=(const CartesianState& state) = delete;
  CartesianState operator*(const CartesianState& state) const = delete;
  Eigen::Vector3d operator*(const Eigen::Vector3d& vector) const = delete;
  CartesianState& operator+=(const CartesianPose& pose) = delete;
  CartesianState& operator+=(const CartesianAcceleration& acceleration) = delete;
  CartesianState& operator+=(const CartesianWrench& wrench) = delete;
  CartesianState operator+(const CartesianPose& pose) const = delete;
  CartesianState operator+(const CartesianAcceleration& acceleration) const = delete;
  CartesianState operator+(const CartesianWrench& wrench) const = delete;
  CartesianState& operator-=(const CartesianPose& pose) = delete;
  CartesianState& operator-=(const CartesianAcceleration& acceleration) = delete;
  CartesianState& operator-=(const CartesianWrench& wrench) = delete;
  CartesianState operator-(const CartesianPose& pose) const = delete;
  CartesianState operator-(const CartesianAcceleration& acceleration) const = delete;
  CartesianState operator-(const CartesianWrench& wrench) const = delete;

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
   * @brief Scale inplace by a scalar
   * @copydetails CartesianState::operator*=(double)
   * @param lambda The scaling factor
   * @return The reference to the scaled Cartesian twist
   */
  CartesianTwist& operator*=(double lambda);

  /**
   * @brief Scale a Cartesian twist by a scalar
   * @copydetails CartesianState::operator*=(double)
   * @param lambda The scaling factor
   * @return The scaled Cartesian twist
   */
  CartesianTwist operator*(double lambda) const;

  /**
   * @brief Scale a Cartesian twist by a scalar
   * @copydetails CartesianState::operator*=(double)
   * @param lambda The scaling factor
   * @param twist The Cartesian twist to be scaled
   * @return The scaled Cartesian twist
   */
  friend CartesianTwist operator*(double lambda, const CartesianTwist& twist);

  /**
   * @brief Scale all dimensions inplace by a matrix
   * @param lambda The scaling factors in all the dimensions
   * @return The reference to the scaled Cartesian twist
   */
  CartesianTwist& operator*=(const Eigen::Matrix<double, 6, 6>& lambda);

  /**
   * @brief Scale a Cartesian twist in all dimensions by a matrix
   * @param lambda The scaling factors in all the dimensions
   * @param twist The Cartesian twist to be scaled
   * @return The scaled Cartesian twist
   */
  friend CartesianTwist operator*(const Eigen::Matrix<double, 6, 6>& lambda, const CartesianTwist& twist);

  /**
   * @brief Integrate a Cartesian twist over a time period
   * @param dt The time period used for integration
   * @return The resulting Cartesian pose after integration
   */
  CartesianPose operator*(const std::chrono::nanoseconds& dt) const;

  /**
   * @brief Integrate a Cartesian twist over a time period
   * @param dt The time period used for integration
   * @param twist The Cartesian twist to be integrated
   * @return The resulting Cartesian pose after integration
   */
  friend CartesianPose operator*(const std::chrono::nanoseconds& dt, const CartesianTwist& twist);

  /**
   * @brief Scale inplace by a scalar
   * @copydetails CartesianState::operator*=(double)
   * @param lambda The scaling factor
   * @return The reference to the scaled Cartesian twist
   */
  CartesianTwist& operator/=(double lambda);

  /**
   * @brief Scale a Cartesian twist by a scalar
   * @copydetails CartesianState::operator*=(double)
   * @param lambda The scaling factor
   * @return The scaled Cartesian twist
   */
  CartesianTwist operator/(double lambda) const;

  /**
   * @brief Derive a Cartesian twist over a time period
   * @param dt The time period used for derivation
   * @return The resulting Cartesian acceleration after derivation
   */
  CartesianAcceleration operator/(const std::chrono::nanoseconds& dt) const;

  /**
   * @brief Add inplace another Cartesian twist
   * @param twist A Cartesian twist in the same reference frame
   * @return The reference to the combined Cartesian twist
   */
  CartesianTwist& operator+=(const CartesianTwist& twist);

  /**
   * @brief Add inplace another twist from a Cartesian state
   * @param state A Cartesian state in the same reference frame
   * @return The reference to the combined Cartesian twist
   */
  CartesianTwist& operator+=(const CartesianState& state);

  /**
   * @brief Add another Cartesian twist
   * @param twist A Cartesian twist in the same reference frame
   * @return The combined Cartesian twist
   */
  CartesianTwist operator+(const CartesianTwist& twist) const;

  /**
   * @brief Add another Cartesian state
   * @param state A Cartesian state in the same reference frame
   * @return The combined Cartesian state
   */
  CartesianState operator+(const CartesianState& state) const;

  /**
   * @brief Negate a Cartesian twist
   * @return The negative value of the Cartesian twist
   */
  CartesianTwist operator-() const;

  /**
   * @brief Compute inplace the difference with another Cartesian twist
   * @param twist A Cartesian twist in the same reference frame
   * @return The reference to the difference in twist
   */
  CartesianTwist& operator-=(const CartesianTwist& twist);

  /**
   * @brief Compute inplace the difference with another Cartesian state
   * @param state A Cartesian state in the same reference frame
   * @return The reference to the difference in twist
   */
  CartesianTwist& operator-=(const CartesianState& state);

  /**
   * @brief Overload the - operator with a twist
   * @param twist The Cartesian twist to subtract
   * @return The current Cartesian twist minus the Cartesian twist given in argument
   */
  CartesianTwist operator-(const CartesianTwist& twist) const;

  /**
   * @brief Compute the difference with a Cartesian state
   * @param state A Cartesian state in the same reference frame
   * @return The difference in all the state variables
   */
  CartesianState operator-(const CartesianState& state) const;

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
