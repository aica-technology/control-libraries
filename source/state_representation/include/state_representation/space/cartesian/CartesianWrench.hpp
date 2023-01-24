#pragma once

#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/space/cartesian/CartesianTwist.hpp"
#include "state_representation/space/cartesian/CartesianAcceleration.hpp"

namespace state_representation {

class CartesianPose;
class CartesianTwist;
class CartesianAcceleration;

/**
 * @class CartesianWrench
 * @brief Class to define wrench in Cartesian space as 3D force and torque vectors
 */
class CartesianWrench : public CartesianState {
public:
  // delete inaccessible getter and setters
  const Eigen::Vector3d& get_linear_velocity() const = delete;
  const Eigen::Vector3d& get_angular_velocity() const = delete;
  Eigen::Matrix<double, 6, 1> get_twist() const = delete;
  const Eigen::Vector3d& get_position() const = delete;
  const Eigen::Quaterniond& get_orientation() const = delete;
  Eigen::Vector4d get_orientation_coefficients() const = delete;
  Eigen::Matrix<double, 7, 1> get_pose() const = delete;
  Eigen::Matrix4d get_transformation_matrix() const = delete;
  const Eigen::Vector3d& get_linear_acceleration() const = delete;
  const Eigen::Vector3d& get_angular_acceleration() const = delete;
  Eigen::Matrix<double, 6, 1> get_acceleration() const = delete;
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
  void set_linear_acceleration(const Eigen::Vector3d& linear_acceleration) = delete;
  void set_linear_acceleration(const std::vector<double>& linear_acceleration) = delete;
  void set_linear_acceleration(const double& x, const double& y, const double& z) = delete;
  void set_angular_acceleration(const Eigen::Vector3d& angular_acceleration) = delete;
  void set_angular_acceleration(const std::vector<double>& angular_acceleration) = delete;
  void set_angular_acceleration(const double& x, const double& y, const double& z) = delete;
  void set_acceleration(const Eigen::Matrix<double, 6, 1>& acceleration) = delete;
  void set_acceleration(const std::vector<double>& acceleration) = delete;
  CartesianState& operator*=(const CartesianState& state) = delete;
  CartesianState operator*(const CartesianState& state) const = delete;
  Eigen::Vector3d operator*(const Eigen::Vector3d& vector) const = delete;
  CartesianState& operator+=(const CartesianPose& pose) = delete;
  CartesianState& operator+=(const CartesianTwist& twist) = delete;
  CartesianState& operator+=(const CartesianAcceleration& acceleration) = delete;
  CartesianState operator+(const CartesianPose& pose) const = delete;
  CartesianState operator+(const CartesianTwist& twist) const = delete;
  CartesianState operator+(const CartesianAcceleration& acceleration) const = delete;

  /**
   * @brief Empty constructor
   */
  explicit CartesianWrench();

  /**
    * @brief Constructor with name and reference frame provided
    * @param name The name of the state
    * @param reference The name of the reference frame (default is "world")
    */
  explicit CartesianWrench(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Copy constructor
   */
  CartesianWrench(const CartesianWrench& wrench);

  /**
   * @brief Copy constructor from a Cartesian state
   */
  CartesianWrench(const CartesianState& state);

  /**
   * @brief Construct a Cartesian wrench from a force given as a vector
   */
  explicit CartesianWrench(
      const std::string& name, const Eigen::Vector3d& force, const std::string& reference = "world"
  );

  /**
   * @brief Construct a Cartesian wrench from a force and torque given as vectors
   */
  explicit CartesianWrench(
      const std::string& name, const Eigen::Vector3d& force, const Eigen::Vector3d& torque,
      const std::string& reference = "world"
  );

  /**
   * @brief Construct a Cartesian wrench from a single 6d wrench vector
   */
  explicit CartesianWrench(
      const std::string& name, const Eigen::Matrix<double, 6, 1>& wrench, const std::string& reference = "world"
  );

  /**
   * @brief Constructor for the zero wrench
   * @param name The name of the state
   * @param reference The name of the reference frame (default is "world)
   * @return The zero Cartesian wrench
   */
  static CartesianWrench Zero(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Constructor for a random wrench
   * @param name The name of the state
   * @param reference The name of the reference frame (default is "world)
   * @return The random Cartesian wrench
   */
  static CartesianWrench Random(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Copy assignment operator that has to be defined to the custom assignment operator
   * @param wrench The wrench with value to assign
   * @return Reference to the current wrench with new values
   */
  CartesianWrench& operator=(const CartesianWrench& wrench) = default;

  /**
   * @brief Returns the wrench data as an Eigen vector
   */
  Eigen::VectorXd data() const override;

  /**
   * @brief Set the wrench data from an Eigen vector
   */
  void set_data(const Eigen::VectorXd& data) override;

  /**
   * @brief Set the wrench data from a std vector
   */
  void set_data(const std::vector<double>& data) override;

  /**
   * @brief Clamp inplace the magnitude of the wrench to the values in argument
   * @param max_force The maximum magnitude of the force
   * @param max_torque The maximum magnitude of the torque
   * @param force_noise_ratio If provided, this value will be used to apply a deadzone under which
   * the force will be set to 0
   * @param torque_noise_ratio If provided, this value will be used to apply a deadzone under which
   * the torque will be set to 0
   */
  void clamp(double max_force, double max_torque, double force_noise_ratio = 0, double torque_noise_ratio = 0);

  /**
   * @brief Return the clamped wrench
   * @param max_force The maximum magnitude of the force
   * @param max_torque The maximum magnitude of the torque
   * @param force_noise_ratio If provided, this value will be used to apply a deadzone under which
   * the force will be set to 0
   * @param torque_noise_ratio If provided, this value will be used to apply a deadzone under which
   * the torque will be set to 0
   * @return The clamped wrench
   */
  CartesianWrench clamped(
      double max_force, double max_torque, double force_noise_ratio = 0, double torque_noise_ratio = 0
  ) const;

  /**
   * @brief Return a copy of the Cartesian wrench
   */
  CartesianWrench copy() const;

  /**
 * @brief Compute the inverse of the current Cartesian wrench
 */
  CartesianWrench inverse() const;

  /**
   * @brief Compute the normalized wrench at the state variable given in argument (default is full wrench)
   * @param state_variable_type The type of state variable to compute the norms on
   * @return The normalized wrench
   */
  CartesianWrench normalized(const CartesianStateVariable& state_variable_type = CartesianStateVariable::WRENCH) const;

  /**
   * @brief Compute the norms of the state variable specified by the input type (default is full wrench)
   * @param state_variable_type The type of state variable to compute the norms on
   * @return The norms of the state variables as a vector
   */
  std::vector<double>
  norms(const CartesianStateVariable& state_variable_type = CartesianStateVariable::WRENCH) const override;

  /**
   * @brief Scale inplace by a scalar
   * @copydetails CartesianState::operator*=(double)
   * @param lambda The scaling factor
   * @return The reference to the scaled Cartesian wrench
   */
  CartesianWrench& operator*=(double lambda);

  /**
   * @brief Scale a Cartesian wrench by a scalar
   * @copydetails CartesianState::operator*=(double)
   * @param lambda The scaling factor
   * @return The scaled Cartesian wrench
   */
  CartesianWrench operator*(double lambda) const;

  /**
   * @brief Scale a Cartesian wrench by a scalar
   * @copydetails CartesianState::operator*=(double)
   * @param lambda The scaling factor
   * @param wrench The Cartesian wrench to be scaled
   * @return The scaled Cartesian wrench
   */
  friend CartesianWrench operator*(double lambda, const CartesianWrench& wrench);

  /**
   * @brief Scale all dimensions inplace by a matrix
   * @param lambda The scaling factors in all the dimensions
   * @return The reference to the scaled Cartesian wrench
   */
  CartesianWrench& operator*=(const Eigen::Matrix<double, 6, 6>& lambda);

  /**
   * @brief Scale a Cartesian wrench in all dimensions by a matrix
   * @param lambda The scaling factors in all the dimensions
   * @param wrench The Cartesian wrench to be scaled
   * @return The scaled Cartesian wrench
   */
  friend CartesianWrench
  operator*(const Eigen::Matrix<double, 6, 6>& lambda, const CartesianWrench& wrench);

  /**
   * @brief Overload the /= operator with a scalar
   * @param lambda The scalar to divide with
   * @return The Cartesian wrench divided by lambda
   */
  CartesianWrench& operator/=(double lambda);

  /**
   * @brief Overload the / operator with a scalar
   * @param lambda The scalar to divide with
   * @return The Cartesian wrench divided by lambda
   */
  CartesianWrench operator/(double lambda) const;

  /**
   * @brief Add inplace another Cartesian wrench
   * @param wrench A Cartesian wrench in the same reference frame
   * @return The reference to the combined Cartesian wrench
   */
  CartesianWrench& operator+=(const CartesianWrench& wrench);

  /**
   * @brief Add inplace another wrench from a Cartesian state
   * @param state A Cartesian state in the same reference frame
   * @return The reference to the combined Cartesian wrench
   */
  CartesianWrench& operator+=(const CartesianState& state);

  /**
   * @brief Add another Cartesian wrench
   * @param wrench A Cartesian wrench in the same reference frame
   * @return The combined Cartesian wrench
   */
  CartesianWrench operator+(const CartesianWrench& wrench) const;

  /**
   * @brief Add another Cartesian state
   * @param state A Cartesian state in the same reference frame
   * @return The combined Cartesian state
   */
  CartesianState operator+(const CartesianState& state) const;

  /**
   * @brief Overload the -= operator
   * @param wrench The Cartesian wrench to subtract
   * @return The current Cartesian wrench minus the Cartesian wrench given in argument
   */
  CartesianWrench& operator-=(const CartesianWrench& wrench);

  /**
   * @brief Overload the - operator
   * @param wrench The Cartesian wrench to subtract
   * @return The current Cartesian wrench minus the Cartesian wrench given in argument
   */
  CartesianWrench operator-(const CartesianWrench& wrench) const;

  /**
   * @brief Overload the ostream operator for printing
   * @param os The ostream to append the string representing the Cartesian wrench to
   * @param CartesianWrench The Cartesian wrench to print
   * @return The appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const CartesianWrench& wrench);

private:
  using CartesianState::clamp_state_variable;
};

}// namespace state_representation
