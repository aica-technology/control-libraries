#include "robot_model/QPSolver.hpp"

namespace robot_model {

  QPSolver::QPSolver(const unsigned int nb_joints, 
                     pinocchio::ModelTpl<double>::ConfigVectorType lower_position_limit,
                     pinocchio::ModelTpl<double>::ConfigVectorType upper_position_limit,
                     pinocchio::ModelTpl<double>::TangentVectorType velocity_limit
                     ) :
    nb_joints_(nb_joints),
    lower_position_limit_(lower_position_limit),
    upper_position_limit_(upper_position_limit),
    velocity_limit_(velocity_limit)
    {
      // Initialize the QP solver
      this->init_solver();
  }

  QPSolver::QPSolver(const QPSolver& other) :
    nb_joints_(other.nb_joints_),
    lower_position_limit_(other.lower_position_limit_),
    upper_position_limit_(other.upper_position_limit_),
    velocity_limit_(other.velocity_limit_),
    hessian_(other.hessian_),
    gradient_(other.gradient_), 
    constraint_matrix_(other.constraint_matrix_),
    lower_bound_constraints_(other.lower_bound_constraints_),
    upper_bound_constraints_(other.upper_bound_constraints_) 
  {
    this->init_solver();
  }

  bool QPSolver::init_solver() {
    // Clear the solver
    this->solver_.data()->clearHessianMatrix();
    this->solver_.data()->clearLinearConstraintsMatrix();
    this->solver_.clearSolver();

    // Initialize the matrices
    this->hessian_ = Eigen::SparseMatrix<double>(this->nb_joints_ + 1, this->nb_joints_ + 1);
    this->gradient_ = Eigen::VectorXd::Zero(this->nb_joints_ + 1);
    this->constraint_matrix_ = Eigen::SparseMatrix<double>(3 * this->nb_joints_ + 1 + 2, this->nb_joints_ + 1);
    this->lower_bound_constraints_ = Eigen::VectorXd::Zero(3 * this->nb_joints_ + 1 + 2);
    this->upper_bound_constraints_ = Eigen::VectorXd::Zero(3 * this->nb_joints_ + 1 + 2);

    // reserve the size of the matrices
    this->hessian_.reserve(this->nb_joints_ * this->nb_joints_ + 1);
    this->constraint_matrix_.reserve(5 * this->nb_joints_ + 2 * (this->nb_joints_ * this->nb_joints_ + this->nb_joints_) + 4 * this->nb_joints_ + 3);

    // configure the QP problem
    this->solver_.settings()->setVerbosity(false);
    this->solver_.settings()->setWarmStart(true);

    // joint dependent constraints
    for (unsigned int n = 0; n < this->nb_joints_; ++n) {
      // joint limits
      this->constraint_matrix_.coeffRef(n, n) = 1.0;
      
      // joint velocity limits
      this->constraint_matrix_.coeffRef(n + this->nb_joints_, n) = 1.0;
      this->constraint_matrix_.coeffRef(n + this->nb_joints_, this->nb_joints_) = this->velocity_limit_(n);
      this->upper_bound_constraints_(n + this->nb_joints_) = std::numeric_limits<double>::infinity();
      this->constraint_matrix_.coeffRef(n + 2 * this->nb_joints_, n) = 1.0;
      this->constraint_matrix_.coeffRef(n + 2 * this->nb_joints_, this->nb_joints_) = -this->velocity_limit_(n);
      this->lower_bound_constraints_(n + 2 * this->nb_joints_) = -std::numeric_limits<double>::infinity();
    }

    // time constraint
    this->constraint_matrix_.coeffRef(3 * this->nb_joints_, this->nb_joints_) = 1.0;
    this->upper_bound_constraints_(3 * this->nb_joints_) = std::numeric_limits<double>::infinity();
    
    // cartesian velocity constraints
    this->upper_bound_constraints_(3 * this->nb_joints_ + 1) = std::numeric_limits<double>::infinity();
    this->upper_bound_constraints_(3 * this->nb_joints_ + 2) = std::numeric_limits<double>::infinity();

    // set the initial data of the QP solver_
    this->solver_.data()->setNumberOfVariables(static_cast<int>(this->nb_joints_) + 1);
    this->solver_.data()->setNumberOfConstraints(this->lower_bound_constraints_.size());

    if (!this->solver_.data()->setHessianMatrix(this->hessian_)) { return false; }
    if (!this->solver_.data()->setGradient(this->gradient_)) { return false; }
    if (!this->solver_.data()->setLinearConstraintsMatrix(this->constraint_matrix_)) { return false; }
    if (!this->solver_.data()->setLowerBound(this->lower_bound_constraints_)) { return false; }
    if (!this->solver_.data()->setUpperBound(this->upper_bound_constraints_)) { return false; }
    
    // instantiate the solver_
    return this->solver_.initSolver();
  }

  
  Eigen::VectorXd QPSolver::solve() {
    // update the constraints
    this->solver_.updateHessianMatrix(this->hessian_);
    this->solver_.updateGradient(this->gradient_);
    this->solver_.updateBounds(this->lower_bound_constraints_, this->upper_bound_constraints_);
    this->solver_.updateLinearConstraintsMatrix(this->constraint_matrix_);
    
    // solve the QP problem
    this->solver_.solveProblem();

    // return the solution
    return this->solver_.getSolution();
  }

  void QPSolver::set_matrices(std::vector<Eigen::Triplet<double>> coefficients,
                  const QPInverseVelocityParameters& parameters,
                  const state_representation::JointPositions& joint_positions,
                  const state_representation::CartesianPose& full_displacement,
                  const Eigen::VectorXd& delta_robot,
                  const Eigen::MatrixXd& jacobian
                  ){
    using namespace std::chrono;

    // update the hessian matrix
    this->hessian_.setFromTriplets(coefficients.begin(), coefficients.end());

    // update the gradient vector
    this->gradient_.head(this->nb_joints_) = -parameters.proportional_gain * delta_robot.transpose() * jacobian;

    // lower bound constraints
    this->lower_bound_constraints_(3 * this->nb_joints_) = duration_cast<duration<float>>(parameters.dt).count();

    for (unsigned int n = 0; n < this->nb_joints_; ++n) {
      this->lower_bound_constraints_(n) = this->lower_position_limit_(n) - joint_positions.data()(n);
      this->upper_bound_constraints_(n) = this->upper_position_limit_(n) - joint_positions.data()(n);
    }

    // update the constraint matrix
    this->constraint_matrix_.coeffRef(3 * this->nb_joints_ + 1, this->nb_joints_) = parameters.linear_velocity_limit;
    this->constraint_matrix_.coeffRef(3 * this->nb_joints_ + 2, this->nb_joints_) = parameters.angular_velocity_limit;
    this->lower_bound_constraints_(3 * this->nb_joints_ + 1) = full_displacement.get_position().norm();
    this->lower_bound_constraints_(3 * this->nb_joints_ + 2) = full_displacement.get_orientation().vec().norm();
  }

  void QPSolver::print_qp_problem() {
    std::cout << "hessian:" << std::endl;
    std::cout << this->hessian_ << std::endl;

    std::cout << "gradient:" << std::endl;
    std::cout << this->gradient_ << std::endl;

    for (unsigned int i = 0; i < this->constraint_matrix_.rows(); ++i) {
      std::cout << this->lower_bound_constraints_(i);
      std::cout << " < | ";
      for (unsigned int j = 0; j < this->constraint_matrix_.cols(); ++j) {
        std::cout << this->constraint_matrix_.coeffRef(i, j) << " | ";
      }
      std::cout << " < ";
      std::cout << this->upper_bound_constraints_(i) << std::endl;
    }
  }
}