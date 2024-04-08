
#include <OsqpEigen/OsqpEigen.h>
#include "robot_model/Model.hpp"


namespace robot_model {

/**
 * @class QPSolver
 * @brief A class to solve the Quadratic Programming problem (QP) for the robot model
*/
class QPSolver {

private:
  // @format:off
  OsqpEigen::Solver solver_;                                                ///< osqp solver for the quadratic programming based inverse kinematics
  robot_model::Model& model_;                                               ///< robot model which is a wrapper around pinocchio dynamic computation library
  Eigen::SparseMatrix<double> hessian_;                                     ///< hessian matrix for the quadratic programming based inverse kinematics
  Eigen::VectorXd gradient_;                                                ///< gradient vector for the quadratic programming based inverse kinematics
  Eigen::SparseMatrix<double> constraint_matrix_;                           ///< constraint matrix for the quadratic programming based inverse kinematics
  Eigen::VectorXd lower_bound_constraints_;                                 ///< lower bound matrix for the quadratic programming based inverse kinematics
  Eigen::VectorXd upper_bound_constraints_;                                 ///< upper bound matrix for the quadratic programming based inverse kinematics
};

  /**
   * @brief Initialize the QP solver
   */
  void init_solver();

public:
  /**
   * @brief Constructor with a reference to the Model object
   * @param model Reference to a Model object
   */
  explicit QPSolver(robot_model::Model& model);

  /**
   * @brief Solve the QP problem
   */
  void solve();
}