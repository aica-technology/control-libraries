
#include <OsqpEigen/OsqpEigen.h>
#include <pinocchio/multibody/data.hpp>
#include <state_representation/space/joint/JointPositions.hpp>
#include <state_representation/space/cartesian/CartesianPose.hpp>

using namespace std::chrono_literals;

namespace robot_model {

/**
 * @brief parameters for the inverse velocity kinematics function
 * @param alpha gain associated to the time slack variable
 * @param proportional_gain gain to weight the cartesian coordinates in the gradient
 * @param linear_velocity_limit maximum linear velocity allowed in Cartesian space (m/s)
 * @param angular_velocity_limit maximum angular velocity allowed in Cartesian space (rad/s)
 * @param period of the control loop (ns)
 */
struct QPInverseVelocityParameters {
  double alpha = 0.1;
  double proportional_gain = 1.0;
  double linear_velocity_limit = 2.0;
  double angular_velocity_limit = 2.0;
  std::chrono::nanoseconds dt = 1000ns;
};

/**
 * @class QPSolver
 * @brief A class to solve the Quadratic Programming problem (QP) for the robot model
*/
class QPSolver {
private:
  // @format:off
  unsigned nb_joints_;                                    ///< number of joints in the robot model
  pinocchio::ModelTpl<double>::ConfigVectorType lower_position_limit_;   ///< lower position limits of the joints from the URDF
  pinocchio::ModelTpl<double>::ConfigVectorType upper_position_limit_;   ///< upper position limits of the joints from the URDF
  pinocchio::ModelTpl<double>::TangentVectorType velocity_limit_;        ///< velocity limits of the joints from the URDF
  OsqpEigen::Solver solver_;                         ///< osqp solver for the quadratic programming based inverse kinematics
  Eigen::SparseMatrix<double> hessian_;              ///< hessian matrix for the quadratic programming based inverse kinematics
  Eigen::VectorXd gradient_;                         ///< gradient vector for the quadratic programming based inverse kinematics
  Eigen::SparseMatrix<double> constraint_matrix_;    ///< constraint matrix for the quadratic programming based inverse kinematics
  Eigen::VectorXd lower_bound_constraints_;          ///< lower bound vector for the quadratic programming based inverse kinematics
  Eigen::VectorXd upper_bound_constraints_;          ///< upper bound vector for the quadratic programming based inverse kinematics

  /**
   * @brief Initialize the QP solver
   */
  bool init_solver();

public:
  /**
   * @brief Constructor of the QP solver
   * @param nb_joints: number of joints in the robot model
   * @param lower_position_limit: lower position limits of the joints
   * @param upper_position_limit: upper position limits of the joints
   * @param velocity_limit: velocity limits of the joints
   */
  explicit QPSolver(const unsigned nb_joints, 
                      pinocchio::ModelTpl<double>::ConfigVectorType lower_position_limit, 
                      pinocchio::ModelTpl<double>::ConfigVectorType upper_position_limit, 
                      pinocchio::ModelTpl<double>::TangentVectorType velocity_limit);

  /**
   * @brief Copy constructor of the QP solver
  */
  QPSolver(const QPSolver& other);
                      
  /**
   * @brief Solve the QP problem
   */
  Eigen::VectorXd solve();

  /**
   * @brief Set the matrices for the QP problem
   * @param coefficients: coefficients for the matrices
   * @param parameters: parameters for the inverse velocity kinematics function
   * @param joint_positions: joint positions of the robot model
   * @param full_displacement: full displacement of the robot model
   * @param delta_robot: delta robot
   * @param jacobian: jacobian matrix
   */
  void set_matrices(std::vector<Eigen::Triplet<double>> coefficients,
                  const QPInverseVelocityParameters& parameters,
                  const state_representation::JointPositions& joint_positions,
                  const state_representation::CartesianPose& full_displacement,
                  const Eigen::VectorXd& delta_robot,
                  const Eigen::MatrixXd& jacobian);

  /**
   * @brief Helper function to print the qp_problem (for debugging)
   */
  void print_qp_problem();
};
}