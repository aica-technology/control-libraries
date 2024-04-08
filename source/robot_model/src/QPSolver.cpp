#include "robot_model/QPSolver.hpp"

namespace robot_model {

  QPSolver::QPSolver(robot_model::Model& model) : model_(model){
    // Initialize the QP solver
    // clear the solver
    this->solver_.data()->clearHessianMatrix();
    this->solver_.data()->clearLinearConstraintsMatrix();
    this->solver_.clearSolver();

    // get the number of joints
    unsigned int nb_joints = this->model_.get_number_of_joints();
    
    // initialize the matrices
    this->hessian_ = Eigen::SparseMatrix<double>(nb_joints + 1, nb_joints + 1);
    this->gradient_ = Eigen::VectorXd::Zero(nb_joints + 1);
    this->constraint_matrix_ = Eigen::SparseMatrix<double>(3 * nb_joints + 1 + 2, nb_joints + 1);
    this->lower_bound_constraints_ = Eigen::VectorXd::Zero(3 * nb_joints + 1 + 2);
    this->upper_bound_constraints_ = Eigen::VectorXd::Zero(3 * nb_joints + 1 + 2);

    // reserve the size of the matrices
    this->hessian_.reserve(nb_joints * nb_joints + 1);
    this->constraint_matrix_.reserve(5 * nb_joints + 2 * (nb_joints * nb_joints + nb_joints) + 4 * nb_joints + 3);

    // get the lower and upper position limits
    Eigen::VectorXd lower_position_limit = this->model_.robot_model_.lowerPositionLimit;
    Eigen::VectorXd upper_position_limit = this->model_.robot_model_.upperPositionLimit;

    // get the velocity limits
    Eigen::VectorXd velocity_limit = this->model_.robot_model_.velocityLimit;

    // configure the QP problem

  }

  void init_solver() {
    // Initialize the QP solver
  }

  void QPSolver::solve() {
        // Solve the QP
  }

}