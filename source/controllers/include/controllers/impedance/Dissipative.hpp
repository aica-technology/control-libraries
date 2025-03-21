#pragma once

#include "controllers/impedance/Impedance.hpp"
#include "state_representation/parameters/Parameter.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace controllers::impedance {

/**
 * @enum ComputationalSpaceType
 * @brief Selector of the space in which the controller should be computed
 * LINEAR and ANGULAR only compute the command in linear and angular space
 * respectively, TWIST applies it on the full twist vector, and
 * DECOUPLED_TWIST (default) computes the damping matrix for the linear
 * and angular part separately
 */
enum class ComputationalSpaceType { LINEAR, ANGULAR, DECOUPLED_TWIST, FULL };

/**
 * @class Dissipative
 * @brief Definition of a dissipative impedance controller (PassiveDS) in task space
 */
template<class S>
class Dissipative : public Impedance<S> {

public:
  /**
   * @brief Base constructor.
   * @param computational_space The computational space type
   * @param dimensions The number of dimensions
   */
  explicit Dissipative(const ComputationalSpaceType& computational_space, unsigned int dimensions = 6);

  /**
   * @brief Constructor from an initial parameter list.
   * @param parameters A parameter list containing the initial parameters
   * @param computational_space The computational space type
   * @param dimensions The number of dimensions
   */
  explicit Dissipative(
      const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters,
      const ComputationalSpaceType& computational_space, unsigned int dimensions = 6
  );

  /**
   * @brief Compute the force (task space) or torque (joint space) command based on the input state
   * of the system as the error between the desired state and the real state.
   * @param command_state the desired state to reach
   * @param feedback_state the real state of the system as read from feedback loop
   * @return the output command at the input state
   */
  S compute_command(const S& command_state, const S& feedback_state) override;

protected:
  /**
   * @brief Validate and set parameter for damping eigenvalues.
   * @param parameter A parameter interface pointer
   */
  void validate_and_set_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter) override;

  /**
   * @brief Orthornormalize the basis matrix given in input wrt the main engenvector
   * @param basis the basis matrix to orthonormalize
   * @param main_eigenvector the main eigenvector used to compute the basis.
   * Other eigenvectors are orthogonal and selected randomly
   * @return the orthonormalized basis
   */
  static Eigen::MatrixXd orthonormalize_basis(const Eigen::MatrixXd& basis, const Eigen::VectorXd& main_eigenvector);

  /**
   * @brief Compute the orthonormal basis based on the desired velocity input
   * @param desired_velocity the desired velocity used as main eigenvector used to compute the basis.
   * Other eigenvectors are orthogonal and selected randomly
   * @return the orthonormalized basis
   */
  Eigen::MatrixXd compute_orthonormal_basis(const S& desired_velocity);

  /**
   * @brief Compute the damping matrix as the orthonormal basis
   * to the direction of the desired velocity
   * @param desired_velocity the velocity from which the direction
   * of motion is extracted to compute the basis
   */
  void compute_damping(const S& desired_velocity);

  std::shared_ptr<state_representation::Parameter<Eigen::VectorXd>>
      damping_eigenvalues_;///< coefficient of eigenvalues used in the damping matrix computation

  const ComputationalSpaceType computational_space_;///< the space in which to compute the command vector

  Eigen::MatrixXd basis_;///< basis matrix used to compute the damping matrix
};

template<class S>
Dissipative<S>::Dissipative(const ComputationalSpaceType& computational_space, unsigned int dimensions)
    : Impedance<S>(dimensions),
      damping_eigenvalues_(state_representation::make_shared_parameter<Eigen::VectorXd>(
          "damping_eigenvalues", Eigen::ArrayXd::Ones(dimensions)
      )),
      computational_space_(computational_space),
      basis_(Eigen::MatrixXd::Random(dimensions, dimensions)) {
  this->parameters_.erase("stiffness");
  this->stiffness_->set_value(Eigen::MatrixXd::Zero(dimensions, dimensions));
  this->parameters_.erase("inertia");
  this->inertia_->set_value(Eigen::MatrixXd::Zero(dimensions, dimensions));

  this->damping_->set_value(Eigen::MatrixXd::Identity(dimensions, dimensions));
  this->parameters_.insert(std::make_pair("damping_eigenvalues", damping_eigenvalues_));
}

template<class S>
Dissipative<S>::Dissipative(
    const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters,
    const ComputationalSpaceType& computational_space, unsigned int dimensions
)
    : Dissipative<S>(computational_space, dimensions) {
  this->set_parameters(parameters);
}

template<class S>
void Dissipative<S>::validate_and_set_parameter(
    const std::shared_ptr<state_representation::ParameterInterface>& parameter
) {
  if (parameter->get_name() == "damping_eigenvalues") {
    this->damping_eigenvalues_->set_value(this->gain_matrix_from_parameter(parameter).diagonal());
  }
}

template<class S>
Eigen::MatrixXd
Dissipative<S>::orthonormalize_basis(const Eigen::MatrixXd& basis, const Eigen::VectorXd& main_eigenvector) {
  Eigen::MatrixXd orthonormal_basis = basis;
  uint dim = basis.rows();
  orthonormal_basis.col(0) = main_eigenvector.normalized();
  for (uint i = 1; i < dim; i++) {
    for (uint j = 0; j < i; j++) {
      orthonormal_basis.col(i) -= orthonormal_basis.col(j).dot(orthonormal_basis.col(i)) * orthonormal_basis.col(j);
    }
    orthonormal_basis.col(i).normalize();
  }
  return orthonormal_basis;
}

template<class S>
S Dissipative<S>::compute_command(const S& command_state, const S& feedback_state) {
  // compute the damping matrix out of the command_state twist
  this->compute_damping(command_state);
  // apply the impedance control law
  return this->Impedance<S>::compute_command(command_state, feedback_state);
}

template<class S>
void Dissipative<S>::compute_damping(const S& desired_velocity) {
  this->basis_ = this->compute_orthonormal_basis(desired_velocity);
  auto diagonal_eigenvalues = this->damping_eigenvalues_->get_value().asDiagonal();
  this->damping_->set_value(this->basis_ * diagonal_eigenvalues * this->basis_.transpose());
}
}// namespace controllers::impedance
