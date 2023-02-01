#pragma once

#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/space/cartesian/CartesianTwist.hpp"
#include "state_representation/space/cartesian/CartesianAcceleration.hpp"
#include "state_representation/space/cartesian/CartesianWrench.hpp"

namespace state_representation {

class CartesianTwist;
class CartesianAcceleration;
class CartesianWrench;

/**
 * @class CartesianPose
 * @brief Class to define Cartesian pose in Cartesian space as 3D position and quaternion based orientation
 */
class CartesianPose : public CartesianState {
public:
  // delete inaccessible getter and setters
  const Eigen::Vector3d& get_linear_velocity() const = delete;
  const Eigen::Vector3d& get_angular_velocity() const = delete;
  Eigen::Matrix<double, 6, 1> get_twist() const = delete;
  const Eigen::Vector3d& get_linear_acceleration() const = delete;
  const Eigen::Vector3d& get_angular_acceleration() const = delete;
  Eigen::Matrix<double, 6, 1> get_acceleration() const = delete;
  const Eigen::Vector3d& get_force() const = delete;
  const Eigen::Vector3d& get_torque() const = delete;
  Eigen::Matrix<double, 6, 1> get_wrench() const = delete;
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
  void set_force(const Eigen::Vector3d& force) = delete;
  void set_force(const std::vector<double>& force) = delete;
  void set_force(const double& x, const double& y, const double& z) = delete;
  void set_torque(const Eigen::Vector3d& torque) = delete;
  void set_torque(const std::vector<double>& torque) = delete;
  void set_torque(const double& x, const double& y, const double& z) = delete;
  void set_wrench(const Eigen::Matrix<double, 6, 1>& wrench) = delete;
  void set_wrench(const std::vector<double>& wrench) = delete;
  CartesianPose& operator*=(const CartesianTwist& twist) = delete;
  CartesianPose& operator*=(const CartesianAcceleration& acceleration) = delete;
  CartesianPose& operator*=(const CartesianWrench& wrench) = delete;
  CartesianState& operator+=(const CartesianTwist& twist) = delete;
  CartesianState& operator+=(const CartesianAcceleration& acceleration) = delete;
  CartesianState& operator+=(const CartesianWrench& wrench) = delete;
  CartesianState operator+(const CartesianTwist& twist) const = delete;
  CartesianState operator+(const CartesianAcceleration& acceleration) const = delete;
  CartesianState operator+(const CartesianWrench& wrench) const = delete;
  CartesianState& operator-=(const CartesianTwist& twist) = delete;
  CartesianState& operator-=(const CartesianAcceleration& acceleration) = delete;
  CartesianState& operator-=(const CartesianWrench& wrench) = delete;
  CartesianState operator-(const CartesianTwist& twist) const = delete;
  CartesianState operator-(const CartesianAcceleration& acceleration) const = delete;
  CartesianState operator-(const CartesianWrench& wrench) const = delete;


  /**
   * @brief Empty constructor
   */
  explicit CartesianPose();

  /**
   * @brief Constructor with name and reference frame provided
   * @param name The name of the state
   * @param reference The name of the reference frame (default is "world")
   */
  explicit CartesianPose(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Copy constructor
   */
  CartesianPose(const CartesianPose& pose);

  /**
   * @brief Copy constructor from a Cartesian state
   */
  CartesianPose(const CartesianState& state);

  /**
   * @brief Copy constructor from a Cartesian twist by considering that it is a displacement over 1 second
   */
  CartesianPose(const CartesianTwist& twist);

  /**
   * @brief Constructor of a Cartesian pose from a position given as a vector of coordinates
   * @param name The name of the state
   * @param position The position data given as Eigen vector
   * @param reference The name of the reference frame (default is "world")
   */
  explicit CartesianPose(
      const std::string& name, const Eigen::Vector3d& position, const std::string& reference = "world"
  );

  /**
   * @brief Constructor of a Cartesian pose from a position given as three scalar coordinates
   * @param name The name of the state
   * @param x The x coordinate of the position
   * @param y The y coordinate of the position
   * @param z The z coordinate of the position
   * @param reference The name of the reference frame (default is "world")
   */
  explicit CartesianPose(const std::string& name, double x, double y, double z, const std::string& reference = "world");

  /**
   * @brief Constructor of a Cartesian pose from a quaternion
   * @param name The name of the state
   * @param orientation The orientation given as Eigen quaternion
   * @param reference The name of the reference frame (default is "world")
   */
  explicit CartesianPose(
      const std::string& name, const Eigen::Quaterniond& orientation, const std::string& reference = "world"
  );

  /**
   * @brief Constructor of a Cartesian pose from a position given as a vector of coordinates and a quaternion
   * @param name The name of the state
   * @param position The position data given as Eigen vector
   * @param orientation The orientation given as Eigen quaternion
   * @param reference The name of the reference frame (default is "world")
   */
  explicit CartesianPose(
      const std::string& name, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
      const std::string& reference = "world"
  );

  /**
   * @brief Constructor for the identity pose
   * @param name The name of the state
   * @param reference The name of the reference frame (default is "world")
   * @return Cartesian identity pose
   */
  static CartesianPose Identity(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Constructor for a random pose
   * @param name The name of the state
   * @param reference The name of the reference frame (default is "world")
   * @return Cartesian random pose
   */
  static CartesianPose Random(const std::string& name, const std::string& reference = "world");

  /**
   * @brief Copy assignment operator that has to be defined to the custom assignment operator
   * @param pose The pose with value to assign
   * @return Reference to the current pose with new values
   */
  CartesianPose& operator=(const CartesianPose& pose) = default;

  /**
   * @brief Returns the pose data as an Eigen vector
   */
  Eigen::VectorXd data() const override;

  /**
   * @brief Set the pose data from an Eigen vector
   */
  void set_data(const Eigen::VectorXd& data) override;

  /**
   * @brief Set the pose data from a std vector
   */
  void set_data(const std::vector<double>& data) override;

  /**
   * @brief Return a copy of the Cartesian pose
   */
  CartesianPose copy() const;

  /**
   * @brief Compute the inverse of the current Cartesian pose
   * @return The inverse corresponding to b_S_f (assuming this is f_S_b)
   */
  CartesianPose inverse() const;

  /**
   * @brief Compute the normalized pose at the state variable given in argument (default is full pose)
   * @param state_variable_type The type of state variable to compute the norms on
   * @return The normalized pose
   * FIXME: state variable type doesnt make sense here
   */
  CartesianPose normalized(const CartesianStateVariable& state_variable_type = CartesianStateVariable::POSE) const;

  /**
   * @brief Compute the norms of the state variable specified by the input type (default is full pose)
   * @param state_variable_type The type of state variable to compute the norms on
   * @return The norms of the state variables as a vector
   */
  std::vector<double>
  norms(const CartesianStateVariable& state_variable_type = CartesianStateVariable::POSE) const override;

  /**
   * @brief Transform inplace a Cartesian state into the current reference frame
   * @details: For a pose A expressed in reference frame W multiplied with a state B expressed in reference frame A,
   * the result of the transformation is a pose B expressed in reference frame W.
   * @param state A Cartesian state expressed in the current pose frame
   * @return The transformed pose expressed in the original reference frame
   */
  CartesianPose& operator*=(const CartesianState& state);

  /**
   * @brief Transform inplace a Cartesian pose into the current reference frame
   * @details: For a pose A expressed in reference frame W multiplied with a pose B expressed in reference frame A,
   * the result of the transformation is a pose B expressed in reference frame W.
   * @param pose A Cartesian pose expressed in the current pose frame
   * @return The transformed pose expressed in the original reference frame
   */
  CartesianPose& operator*=(const CartesianPose& pose);

  /**
   * @brief Transform a Cartesian state into the pose reference frame
   * @details: For a pose A expressed in reference frame W multiplied with a state B expressed in reference frame A,
   * the result of the transformation is a state B expressed in reference frame W.
   * @param state A Cartesian state expressed in the pose frame
   * @return The transformed state expressed in the pose reference frame
   */
  CartesianState operator*(const CartesianState& state) const;

  /**
   * @brief Transform a Cartesian pose into the left operand pose reference frame
   * @details: For a pose A expressed in reference frame W multiplied with a pose B expressed in reference frame A,
   * the result of the transformation is a pose B expressed in reference frame W.
   * @param pose A Cartesian pose expressed in the left operand pose frame
   * @return The transformed pose expressed in the left operand pose reference frame
   */
  CartesianPose operator*(const CartesianPose& pose) const;

  /**
   * @brief Transform a Cartesian twist into the pose reference frame
   * @details: For a pose A expressed in reference frame W multiplied with a twist B expressed in reference frame A,
   * the result of the transformation is a twist B expressed in reference frame W.
   * @param twist A Cartesian twist expressed in the pose frame
   * @return The transformed twist expressed in the pose reference frame
   */
  CartesianTwist operator*(const CartesianTwist& twist) const;

  /**
   * @brief Transform a Cartesian acceleration into the pose reference frame
   * @details: For a pose A expressed in reference frame W multiplied with an acceleration B expressed in reference
   * frame A, the result of the transformation is an acceleration B expressed in reference frame W.
   * @param acceleration A Cartesian acceleration expressed in the pose frame
   * @return The transformed acceleration expressed in the pose reference frame
   */
  CartesianAcceleration operator*(const CartesianAcceleration& acceleration) const;

  /**
   * @brief Transform a Cartesian wrench into the pose reference frame
   * @details: For a pose A expressed in reference frame W multiplied with a wrench B expressed in reference frame A,
   * the result of the transformation is a wrench B expressed in reference frame W.
   * @param wrench A Cartesian wrench expressed in the pose frame
   * @return The transformed wrench expressed in the pose reference frame
   */
  CartesianWrench operator*(const CartesianWrench& wrench) const;

  /**
   * @brief Scale inplace by a scalar
   * @copydetails CartesianState::operator*=(double)
   * @param lambda The scaling factor
   * @return The reference to the scaled Cartesian pose
   */
  CartesianPose& operator*=(double lambda);

  /**
   * @brief Scale a Cartesian pose by a scalar
   * @copydetails CartesianState::operator*=(double)
   * @param lambda The scaling factor
   * @return The scaled Cartesian pose
   */
  CartesianPose operator*(double lambda) const;

  /**
   * @brief Scale a Cartesian pose by a scalar
   * @copydetails CartesianState::operator*=(double)
   * @param lambda The scaling factor
   * @param pose The Cartesian pose to be scaled
   * @return The scaled Cartesian pose
   */
  friend CartesianPose operator*(double lambda, const CartesianPose& pose);

  /**
   * @brief Scale inplace by a scalar
   * @copydetails CartesianState::operator*=(double)
   * @param lambda The scaling factor
   * @return The reference to the scaled Cartesian pose
   */
  CartesianPose& operator/=(double lambda);

  /**
   * @brief Scale a Cartesian pose by a scalar
   * @copydetails CartesianState::operator*=(double)
   * @param lambda The scaling factor
   * @return The scaled Cartesian acceleration
   */
  CartesianPose operator/(double lambda) const;

  /**
   * @brief Differentiate a Cartesian pose over a time period
   * @param dt The time period used for derivation
   * @return The resulting Cartesian twist after derivation
   */
  CartesianTwist operator/(const std::chrono::nanoseconds& dt) const;

  /**
   * @brief Add inplace another Cartesian pose
   * @param pose A Cartesian pose in the same reference frame
   * @return The reference to the combined Cartesian pose
   */
  CartesianPose& operator+=(const CartesianPose& pose);

  /**
   * @brief Add inplace another pose from a Cartesian state
   * @param state A Cartesian state in the same reference frame
   * @return The reference to the combined Cartesian pose
   */
  CartesianPose& operator+=(const CartesianState& state);

  /**
   * @brief Add another Cartesian pose
   * @param pose A Cartesian pose in the same reference frame
   * @return The combined Cartesian pose
   */
  CartesianPose operator+(const CartesianPose& pose) const;

  /**
   * @brief Add another Cartesian state
   * @param state A Cartesian state in the same reference frame
   * @return The combined Cartesian state
   */
  CartesianState operator+(const CartesianState& state) const;

  /**
   * @brief Negate a Cartesian pose
   * @return The negative value of the Cartesian pose
   */
  CartesianPose operator-() const;

  /**
   * @brief Compute inplace the difference with another Cartesian pose
   * @param pose A Cartesian pose in the same reference frame
   * @return The reference to the difference in pose
   */
  CartesianPose& operator-=(const CartesianPose& pose);

  /**
   * @brief Compute inplace the difference with another Cartesian state
   * @param state A Cartesian state in the same reference frame
   * @return The reference to the difference in pose
   */
  CartesianPose& operator-=(const CartesianState& state);

  /**
   * @brief Compute the difference with another Cartesian pose
   * @param pose A Cartesian pose in the same reference frame
   * @return The difference in pose
   */
  CartesianPose operator-(const CartesianPose& pose) const;

  /**
   * @brief Compute the difference with a Cartesian state
   * @param state A Cartesian state in the same reference frame
   * @return The difference in all the state variables
   */
  CartesianState operator-(const CartesianState& state) const;

  /**
   * @brief Overload the ostream operator for printing
   * @param os The ostream to append the string representing the Cartesian pose to
   * @param CartesianPose The Cartesian pose to print
   * @return The appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const CartesianPose& pose);

private:
  using CartesianState::clamp_state_variable;
};

}// namespace state_representation
