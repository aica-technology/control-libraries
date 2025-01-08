#pragma once

#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/space/cartesian/CartesianTwist.hpp"
#include "state_representation/space/cartesian/CartesianWrench.hpp"

namespace state_representation {

class CartesianPose;
class CartesianTwist;
class CartesianWrench;

/**
 * @class CartesianAcceleration
 * @brief Class to define acceleration in Cartesian space as 3D linear and angular acceleration vectors
 */
class CartesianAcceleration : public CartesianState {
public:
  // delete inaccessible getter and setters
  const Eigen::Vector3d& get_position() const = delete;
  const Eigen::Quaterniond& get_orientation() const = delete;
  Eigen::Vector4d get_orientation_coefficients() const = delete;
  Eigen::Matrix<double, 7, 1> get_pose() const = delete;
  Eigen::Matrix4d get_transformation_matrix() const = delete;
  const Eigen::Vector3d& get_linear_velocity() const = delete;
  const Eigen::Vector3d& get_angular_velocity() const = delete;
  Eigen::Matrix<double, 6, 1> get_twist() const = delete;
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
  void set_linear_velocity(const Eigen::Vector3d& linear_velocity) = delete;
  void set_linear_velocity(const std::vector<double>& linear_velocity) = delete;
  void set_linear_velocity(const double& x, const double& y, const double& z) = delete;
  void set_angular_velocity(const Eigen::Vector3d& angular_velocity) = delete;
  void set_angular_velocity(const std::vector<double>& angular_velocity) = delete;
  void set_angular_velocity(const double& x, const double& y, const double& z) = delete;
  void set_twist(const Eigen::Matrix<double, 6, 1>& twist) = delete;
  void set_twist(const std::vector<double>& twist) = delete;
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
  CartesianState& operator+=(const CartesianTwist& twist) = delete;
  CartesianState& operator+=(const CartesianWrench& wrench) = delete;
  CartesianState operator+(const CartesianPose& pose) const = delete;
  CartesianState operator+(const CartesianTwist& twist) const = delete;
  CartesianState operator+(const CartesianWrench& wrench) const = delete;
  CartesianState& operator-=(const CartesianPose& pose) = delete;
  CartesianState& operator-=(const CartesianTwist& twist) = delete;
  CartesianState& operator-=(const CartesianWrench& wrench) = delete;
  CartesianState operator-(const CartesianPose& pose) const = delete;
  CartesianState operator-(const CartesianTwist& twist) const = delete;
  CartesianState operator-(const CartesianWrench& wrench) const = delete;

  /**
   * @brief Empty constructor
   */
  explicit CartesianAcceleration();

  /**
   * @brief Constructor with name and reference frame provided
   * @param name The name of the state
   * @param reference The name of the reference frame (default is "world")
   */
  explicit CartesianAcceleration(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Copy constructor
   */
  CartesianAcceleration(const CartesianAcceleration& acceleration);

  /**
   * @brief Copy constructor from a Cartesian state
   */
  CartesianAcceleration(const CartesianState& state);

  /**
   * @brief Copy constructor from a Cartesian twist by considering
   * that it is equivalent to dividing the twist by 1 second
   */
  CartesianAcceleration(const CartesianTwist& pose);

  /**
   * @brief Construct a Cartesian acceleration from a linear acceleration given as a vector
   */
  explicit CartesianAcceleration(
      const std::string& name, const Eigen::Vector3d& linear_acceleration, const std::string& reference = "world"
  );

  /**
   * @brief Construct a Cartesian acceleration from a linear acceleration and angular acceleration given as vectors
   */
  explicit CartesianAcceleration(
      const std::string& name, const Eigen::Vector3d& linear_acceleration, const Eigen::Vector3d& angular_acceleration,
      const std::string& reference = "world"
  );

  /**
   * @brief Construct a Cartesian acceleration from a single 6d acceleration vector
   */
  explicit CartesianAcceleration(
      const std::string& name, const Eigen::Matrix<double, 6, 1>& acceleration, const std::string& reference = "world"
  );

  /**
   * @brief Constructor for the zero acceleration
   * @param name The name of the state
   * @param reference The name of the reference frame (default is "world")
   * @return The zero Cartesian acceleration
   */
  static CartesianAcceleration Zero(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Constructor for a random acceleration
   * @param name The name of the state
   * @param reference The name of the reference frame (default is "world")
   * @return The random Cartesian acceleration
   */
  static CartesianAcceleration Random(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Copy assignment operator that has to be defined to the custom assignment operator
   * @param acceleration The acceleration with value to assign
   * @return Reference to the current acceleration with new values
   */
  CartesianAcceleration& operator=(const CartesianAcceleration& acceleration) = default;

  /**
 * @brief Returns the acceleration data as an Eigen vector
 */
  Eigen::VectorXd data() const override;

  /**
   * @brief Set the acceleration data from an Eigen vector
   */
  void set_data(const Eigen::VectorXd& data) override;

  /**
   * @brief Set the acceleration data from a std vector
   */
  void set_data(const std::vector<double>& data) override;

  /**
   * @brief Clamp inplace the magnitude of the acceleration to the values in argument
   * @param max_linear The maximum magnitude of the linear acceleration
   * @param max_angular The maximum magnitude of the angular acceleration
   * @param linear_noise_ratio If provided, this value will be used to apply a deadzone under which
   * the linear acceleration will be set to 0
   * @param angular_noise_ratio If provided, this value will be used to apply a deadzone under which
   * the angular acceleration will be set to 0
   */
  void clamp(double max_linear, double max_angular, double linear_noise_ratio = 0, double angular_noise_ratio = 0);

  /**
   * @brief Return the clamped twist
   * @param max_linear The maximum magnitude of the linear acceleration
   * @param max_angular The maximum magnitude of the angular acceleration
   * @param noise_ratio If provided, this value will be used to apply a deadzone under which
   * the linear acceleration will be set to 0
   * @param angular_noise_ratio If provided, this value will be used to apply a deadzone under which
   * the angular acceleration will be set to 0
   * @return The clamped acceleration
   */
  CartesianAcceleration clamped(
      double max_linear, double max_angular, double noise_ratio = 0, double angular_noise_ratio = 0
  ) const;

  /**
   * @brief Return a copy of the Cartesian acceleration
   */
  CartesianAcceleration copy() const;

  /**
   * @brief Integrate the Cartesian acceleration over a time period
   * @param dt The time period used for integration in seconds
   * @return The resulting Cartesian twist after integration
   */
  CartesianTwist integrate(double dt) const;

  /**
   * @brief Integrate the Cartesian acceleration over a time period
   * @param dt The time period used for integration
   * @return The resulting Cartesian twist after integration
   */
  CartesianTwist integrate(const std::chrono::nanoseconds& dt) const;

  /**
   * @brief Compute the inverse of the current Cartesian acceleration
   */
  CartesianAcceleration inverse() const;

  /**
   * @brief Compute the normalized acceleration at the state variable given in argument (default is full acceleration)
   * @param state_variable_type The type of state variable to compute the norms on
   * @return The normalized acceleration
   */
  CartesianAcceleration
  normalized(const CartesianStateVariable& state_variable_type = CartesianStateVariable::ACCELERATION) const;

  /**
   * @brief Compute the norms of the state variable specified by the input type (default is full acceleration)
   * @param state_variable_type The type of state variable to compute the norms on
   * @return The norms of the state variables as a vector
   */
  std::vector<double>
  norms(const CartesianStateVariable& state_variable_type = CartesianStateVariable::ACCELERATION) const override;

  /**
   * @brief Scale inplace by a scalar
   * @copydetails CartesianState::operator*=(double)
   * @param lambda The scaling factor
   * @return The reference to the scaled Cartesian acceleration
   */
  CartesianAcceleration& operator*=(double lambda);

  /**
   * @brief Scale a Cartesian acceleration by a scalar
   * @copydetails CartesianState::operator*=(double)
   * @param lambda The scaling factor
   * @return The scaled Cartesian acceleration
   */
  CartesianAcceleration operator*(double lambda) const;

  /**
   * @brief Scale a Cartesian acceleration by a scalar
   * @copydetails CartesianState::operator*=(double)
   * @param lambda The scaling factor
   * @param acceleration The Cartesian acceleration to be scaled
   * @return The scaled Cartesian acceleration
   */
  friend CartesianAcceleration operator*(double lambda, const CartesianAcceleration& acceleration);

  /**
   * @brief Scale a Cartesian acceleration in all dimensions by a matrix
   * @param lambda The scaling factors in all the dimensions
   * @param acceleration The Cartesian acceleration to be scaled
   * @return The scaled Cartesian acceleration
   */
  friend CartesianAcceleration
  operator*(const Eigen::Matrix<double, 6, 6>& lambda, const CartesianAcceleration& acceleration);

  /**
   * @brief Integrate a Cartesian acceleration over a time period
   * @param dt The time period used for integration
   * @return The resulting Cartesian twist after integration
   */
  CartesianTwist operator*(const std::chrono::nanoseconds& dt) const;

  /**
   * @brief Integrate a Cartesian acceleration over a time period
   * @param dt The time period used for integration
   * @param acceleration The Cartesian acceleration to be integrated
   * @return The resulting Cartesian twist after integration
   */
  friend CartesianTwist operator*(const std::chrono::nanoseconds& dt, const CartesianAcceleration& acceleration);

  /**
   * @brief Scale inplace by a scalar
   * @copydetails CartesianState::operator*=(double)
   * @param lambda The scaling factor
   * @return The reference to the scaled Cartesian acceleration
   */
  CartesianAcceleration& operator/=(double lambda);

  /**
   * @brief Scale a Cartesian acceleration by a scalar
   * @copydetails CartesianState::operator*=(double)
   * @param lambda The scaling factor
   * @return The scaled Cartesian acceleration
   */
  CartesianAcceleration operator/(double lambda) const;

  /**
   * @brief Add inplace another Cartesian acceleration
   * @param acceleration A Cartesian acceleration in the same reference frame
   * @return The reference to the combined Cartesian acceleration
   */
  CartesianAcceleration& operator+=(const CartesianAcceleration& acceleration);

  /**
   * @brief Add inplace another acceleration from a Cartesian state
   * @param state A Cartesian state in the same reference frame
   * @return The reference to the combined Cartesian acceleration
   */
  CartesianAcceleration& operator+=(const CartesianState& state);

  /**
   * @brief Add another Cartesian acceleration
   * @param acceleration A Cartesian acceleration in the same reference frame
   * @return The combined Cartesian acceleration
   */
  CartesianAcceleration operator+(const CartesianAcceleration& acceleration) const;

  /**
   * @brief Add another Cartesian state
   * @param state A Cartesian state in the same reference frame
   * @return The combined Cartesian state
   */
  CartesianState operator+(const CartesianState& state) const;

  /**
   * @brief Negate a Cartesian acceleration
   * @return The negative value of the Cartesian acceleration
   */
  CartesianAcceleration operator-() const;

  /**
   * @brief Compute inplace the difference with another Cartesian acceleration
   * @param acceleration A Cartesian acceleration in the same reference frame
   * @return The reference to the difference in acceleration
   */
  CartesianAcceleration& operator-=(const CartesianAcceleration& acceleration);

  /**
   * @brief Compute inplace the difference with another Cartesian state
   * @param state A Cartesian state in the same reference frame
   * @return The reference to the difference in acceleration
   */
  CartesianAcceleration& operator-=(const CartesianState& state);

  /**
   * @brief Compute the difference with another Cartesian acceleration
   * @param acceleration A Cartesian acceleration in the same reference frame
   * @return The difference in acceleration
   */
  CartesianAcceleration operator-(const CartesianAcceleration& acceleration) const;

  /**
   * @brief Compute the difference with a Cartesian state
   * @param state A Cartesian state in the same reference frame
   * @return The difference in all the state variables
   */
  CartesianState operator-(const CartesianState& state) const;

  /**
   * @brief Overload the ostream operator for printing
   * @param os The ostream to append the string representing the Cartesian acceleration to
   * @param acceleration The Cartesian acceleration to print
   * @return The appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const CartesianAcceleration& acceleration);

private:
  using CartesianState::clamp_state_variable;
};

}// namespace state_representation
