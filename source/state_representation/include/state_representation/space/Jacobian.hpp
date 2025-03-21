#pragma once

#include "state_representation/State.hpp"
#include "state_representation/exceptions/IncompatibleSizeException.hpp"
#include "state_representation/space/cartesian/CartesianTwist.hpp"
#include "state_representation/space/cartesian/CartesianWrench.hpp"
#include "state_representation/space/joint/JointTorques.hpp"
#include "state_representation/space/joint/JointVelocities.hpp"

namespace state_representation {

class CartesianTwist;
class CartesianWrench;
class JointVelocities;
class JointTorques;

/**
 * @class Jacobian
 * @brief Class to define a robot Jacobian matrix
 */
class Jacobian : public State {
public:
  /**
   * @brief Empty constructor for a Jacobian
   */
  Jacobian();

  /**
   * @brief Constructor with name, number of joints, frame and reference frame provided
   * @param robot_name The name of the associated robot
   * @param nb_joints The number of joints of the robot
   * @param frame The name of the frame at which the Jacobian is computed
   * @param reference_frame The name of the reference frame in which the Jacobian is expressed (default is "world")
   */
  Jacobian(
      const std::string& robot_name, unsigned int nb_joints, const std::string& frame,
      const std::string& reference_frame = "world"
  );

  /**
   * @brief Constructor with name, joint names, frame and reference frame provided
   * @param robot_name The name of the associated robot
   * @param joint_names The vector of joint names of the robot
   * @param frame The name of the frame at which the Jacobian is computed
   * @param reference_frame The name of the reference frame in which the Jacobian is expressed (default is "world")
   */
  Jacobian(
      const std::string& robot_name, const std::vector<std::string>& joint_names, const std::string& frame,
      const std::string& reference_frame = "world"
  );

  /**
   * @brief Constructor with name, frame, Jacobian matrix and reference frame provided
   * @param robot_name The name of the associated robot
   * @param frame The name of the frame at which the Jacobian is computed
   * @param data The values of the Jacobian matrix
   * @param reference_frame The name of the reference frame in which the Jacobian is expressed (default is "world")
   */
  Jacobian(
      const std::string& robot_name, const std::string& frame, const Eigen::MatrixXd& data,
      const std::string& reference_frame = "world"
  );

  /**
   * @brief Constructor with name, joint names, frame, Jacobian matrix and reference frame provided
   * @param robot_name The name of the associated robot
   * @param joint_names The vector of joint names of the robot
   * @param frame The name of the frame at which the Jacobian is computed
   * @param data The values of the Jacobian matrix
   * @param reference_frame The name of the reference frame in which the Jacobian is expressed (default is "world")
   */
  Jacobian(
      const std::string& robot_name, const std::vector<std::string>& joint_names, const std::string& frame,
      const Eigen::MatrixXd& data, const std::string& reference_frame = "world"
  );

  /**
   * @brief Copy constructor of a Jacobian
   */
  Jacobian(const Jacobian& jacobian);

  /**
   * @brief Constructor for a random Jacobian
   * @param robot_name The name of the associated robot
   * @param nb_joints The number of joints of the robot
   * @param frame The name of the frame at which the Jacobian is computed
   * @param reference_frame The name of the reference frame in which the Jacobian is expressed (default is "world")
   * @return Jacobian with random data values
   */
  static Jacobian Random(
      const std::string& robot_name, unsigned int nb_joints, const std::string& frame,
      const std::string& reference_frame = "world"
  );

  /**
   * @brief Constructor for a random Jacobian
   * @param robot_name The name of the associated robot
   * @param joint_names The vector of joint names of the robot
   * @param frame The name of the frame at which the Jacobian is computed
   * @param reference_frame The name of the reference frame in which the Jacobian is expressed (default is "world")
   * @return Jacobian with random data values
   */
  static Jacobian Random(
      const std::string& robot_name, const std::vector<std::string>& joint_names, const std::string& frame,
      const std::string& reference_frame = "world"
  );

  /**
   * @brief Swap the values of the two Jacobians
   * @param jacobian1 Jacobian to be swapped with 2
   * @param jacobian2 Jacobian to be swapped with 1
   */
  friend void swap(Jacobian& jacobian1, Jacobian& jacobian2);

  /**
   * @brief Copy assignment operator that has to be defined to the custom assignment operator
   * @param jacobian The Jacobian with value to assign
   * @return Reference to the current Jacobian with new values
   */
  Jacobian& operator=(const Jacobian& jacobian);

  /**
   * @brief Getter of the number of rows attribute
   */
  unsigned int rows() const;

  /**
   * @brief Accessor of the row data at given index
   * @param index The index of the desired row
   * @return The row vector at index
   */
  Eigen::VectorXd row(unsigned int index) const;

  /**
   * @brief Getter of the number of columns attribute
   */
  unsigned int cols() const;

  /**
   * @brief Accessor of the column data at given index
   * @param index The index of the desired column
   * @return The column vector at index
   */
  Eigen::VectorXd col(unsigned int index) const;

  /**
   * @brief Getter of the joint names attribute
   */
  const std::vector<std::string>& get_joint_names() const;

  /**
   * @brief Getter of the frame attribute
   */
  const std::string& get_frame() const;

  /**
   * @brief Getter of the reference frame attribute
   */
  const std::string& get_reference_frame() const;

  /**
   * @brief Getter of the data attribute
   */
  const Eigen::MatrixXd& data() const;

  /**
   * @brief Setter of the joint names attribute from the number of joints
   */
  void set_joint_names(unsigned int nb_joints);

  /**
   * @brief Setter of the joint names attribute from a vector of joint names
   */
  void set_joint_names(const std::vector<std::string>& joint_names);

  /**
   * @brief Setter of the frame
   */
  void set_frame(const std::string& frame);

  /**
   * @brief Setter of the reference frame
   */
  void set_reference_frame(const std::string& reference_frame);

  /**
   * @brief Setter of the data attribute
   */
  void set_data(const Eigen::MatrixXd& data) override;

  /**
   * @brief Set the Jacobian matrix to a zero value
   */
  void set_zero();

  /**
   * @brief Return a copy of the Jacobian
   */
  Jacobian copy() const;

  /**
   * @copybrief State::reset
   */
  void reset() override;

  /**
   * @brief Return the inverse of the Jacobian matrix
   * @details If the matrix is not invertible, an error is thrown advising to use the
   * pseudoinverse function instead
   * @return The inverse of the Jacobian
   */
  Eigen::MatrixXd inverse() const;

  /**
   * @brief Solve the system X = inv(J) * M to obtain X
   * @details This uses QR decomposition to solve for X, which is more efficient than calculating and multiplying
   * by the Jacobian inverse independently.
   * @param matrix The matrix to solve the system with
   * @return The result of the product inv(J) * M
   */
  Eigen::MatrixXd inverse(const Eigen::MatrixXd& matrix) const;

  /**
   * @brief Transform the given Cartesian twist to joint space
   * @details This uses QR decomposition to solve for the joint velocities, which is more efficient than calculating
   * and multiplying by the Jacobian inverse independently.
   * @param twist The Cartesian twist to be transformed to joint space
   * @return The resulting joint velocities from the product inv(J) * twist
   */
  JointVelocities inverse(const CartesianTwist& twist) const;

  /**
   * @brief Check if the Jacobian is incompatible for operations with the state given as argument
   * @param state The state to check compatibility with
   */
  bool is_incompatible(const State& state) const override;

  /**
   * @brief Return the pseudoinverse of the Jacobian matrix
   * @return The pseudoinverse of the Jacobian
   */
  Eigen::MatrixXd pseudoinverse() const;

  /**
   * @brief Multiply the given matrix by the pseudoinverse of the Jacobian matrix
   * @param matrix The matrix to be multiplied with the Jacobian
   * @return The result of the matrix multiplication
   */
  Eigen::MatrixXd pseudoinverse(const Eigen::MatrixXd& matrix) const;

  /**
   * @brief Transform a Cartesian twist to joint space by pre-multiplying the Jacobian pseudoinverse
   * @param twist The Cartesian twist to be transformed to joint space
   * @return The corresponding joint velocities
   */
  JointVelocities pseudoinverse(const CartesianTwist& twist) const;

  /**
    * @brief Return the transpose of the Jacobian matrix
    * @return The transposed Jacobian matrix
    */
  Eigen::MatrixXd transpose() const;

  /**
    * @brief Transform a Cartesian wrench to joint space by pre-multiplying the Jacobian transpose
    * @param wrench The Cartesian wrench to be transformed to joint space
    * @return The corresponding joint torques
    */
  JointTorques transpose(const CartesianWrench& wrench) const;

  /**
   * @brief Overload the * operator with an arbitrary matrix
   * @param matrix The matrix to multiply with
   * @return The Jacobian matrix multiplied by the matrix in parameter
   */
  Eigen::MatrixXd operator*(const Eigen::MatrixXd& matrix) const;

  /**
   * @brief Overload the * operator with a 6x6 matrix on the left side
   * @param matrix The matrix to multiply with
   * @param jacobian The Jacobian
   * @return The Jacobian transformed by the matrix
   */
  friend Jacobian operator*(const Eigen::Matrix<double, 6, 6>& matrix, const Jacobian& jacobian);

  /**
   * @brief Overload the * operator with a JointVelocities
   * @param dq The joint velocities to multiply with
   * @return The corresponding Cartesian twist of the Jacobian frame
   */
  CartesianTwist operator*(const JointVelocities& dq) const;

  /**
   * @brief Overload the * operator with a Cartesian pose on the left side
   * @details This operation is equivalent to a change of reference frame of the Jacobian
   * @param pose The Cartesian pose to multiply with
   * @param jacobian The Jacobian to be multiplied with the Cartesian pose
   * @return The Jacobian expressed in the new reference frame
   */
  friend Jacobian operator*(const CartesianPose& pose, const Jacobian& jacobian);

  /**
   * @brief Overload the () operator in a non const fashion to modify the value at given (row, col)
   * @param row The index of the row
   * @param column The index of the column
   * @return The reference to the value at the given row and column
   */
  double& operator()(unsigned int row, unsigned int col);

  /**
   * @brief Overload the () operator in const fashion to access the value at given (row, col)
   * @param row The index of the row
   * @param column The index of the column
   * @return The const reference to the value at the given row and column
   */
  const double& operator()(unsigned int row, unsigned int col) const;

  /**
   * @brief Overload the ostream operator for printing
   * @param os The ostream to append the string representing the Jacobian to
   * @param jacobian The Jacobian to print
   * @return The appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const Jacobian& jacobian);

private:
  std::vector<std::string> joint_names_;///< names of the joints
  std::string frame_;                   ///< name of the frame at which the Jacobian is computed
  std::string reference_frame_;         ///< name of the reference frame in which the Jacobian is expressed
  Eigen::MatrixXd data_;                ///< internal storage of the Jacobian matrix
};

inline void swap(Jacobian& jacobian1, Jacobian& jacobian2) {
  swap(static_cast<State&>(jacobian1), static_cast<State&>(jacobian2));
  std::swap(jacobian1.joint_names_, jacobian2.joint_names_);
  std::swap(jacobian1.frame_, jacobian2.frame_);
  std::swap(jacobian1.reference_frame_, jacobian2.reference_frame_);
  std::swap(jacobian1.data_, jacobian2.data_);
}
}// namespace state_representation
