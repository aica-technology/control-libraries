#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "controllers/IController.hpp"
#include "state_representation/State.hpp"
#include "state_representation/parameters/Parameter.hpp"

namespace controllers::impedance {

/**
 * @class Impedance
 * @brief Definition of an impedance controller in either joint or task space
 * @tparam S the space of the controller (either CartesianState or JointState)
 */
template<class S>
class Impedance : public IController<S> {
public:
  /**
   * @brief Base constructor.
   * @details This initializes all gain matrices to the identity matrix of the corresponding dimensionality.
   * @param dimensions The number of dimensions associated with the controller
   */
  explicit Impedance(unsigned int dimensions = 6);

  /**
   * @brief Constructor from an initial parameter list
   * @param parameters A parameter list containing initial gain values
   * @param dimensions The number of dimensions associated with the controller
   */
  explicit Impedance(
      const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters,
      unsigned int dimensions = 6
  );

  /**
   * @copydoc IController::compute_command(const S&,const S&)
   */
  S compute_command(const S& command_state, const S& feedback_state) override;

protected:
  void clamp_force(Eigen::VectorXd& force);

  /**
   * @brief Validate and set parameters for damping, stiffness and inertia gain matrices.
   * @param parameter A parameter interface pointer
   */
  void validate_and_set_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter) override;

  /**
   * @brief Convert a parameterized gain to a gain matrix
   * @param parameter A parameter interface pointer of type double, double array or matrix
   * @return A square gain matrix
   */
  Eigen::MatrixXd gain_matrix_from_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter
  );

  std::shared_ptr<state_representation::Parameter<Eigen::MatrixXd>>
      stiffness_;///< stiffness matrix of the controller associated to position
  std::shared_ptr<state_representation::Parameter<Eigen::MatrixXd>>
      damping_;///< damping matrix of the controller associated to velocity
  std::shared_ptr<state_representation::Parameter<Eigen::MatrixXd>>
      inertia_;///< inertia matrix of the controller associated to acceleration
  std::shared_ptr<state_representation::Parameter<bool>>
      feed_forward_force_;///< flag to decide if force error should be passed on
  std::shared_ptr<state_representation::Parameter<Eigen::VectorXd>>
      force_limit_;///< vector of force limits for each degree of freedom

  const unsigned int dimensions_;///< dimensionality of the control space and associated gain matrices
};

template<class S>
Impedance<S>::Impedance(unsigned int dimensions)
    : stiffness_(state_representation::make_shared_parameter<Eigen::MatrixXd>(
          "stiffness", Eigen::MatrixXd::Identity(dimensions, dimensions)
      )),
      damping_(state_representation::make_shared_parameter<Eigen::MatrixXd>(
          "damping", Eigen::MatrixXd::Identity(dimensions, dimensions)
      )),
      inertia_(state_representation::make_shared_parameter<Eigen::MatrixXd>(
          "inertia", Eigen::MatrixXd::Identity(dimensions, dimensions)
      )),
      feed_forward_force_(state_representation::make_shared_parameter<bool>("feed_forward_force", false)),
      force_limit_(state_representation::make_shared_parameter<Eigen::VectorXd>("force_limit")),
      dimensions_(dimensions) {
  this->parameters_.insert(std::make_pair("stiffness", stiffness_));
  this->parameters_.insert(std::make_pair("damping", damping_));
  this->parameters_.insert(std::make_pair("inertia", inertia_));
  this->parameters_.insert(std::make_pair("feed_forward_force", feed_forward_force_));
  this->parameters_.insert(std::make_pair("force_limit", force_limit_));
}

template<class S>
Impedance<S>::Impedance(
    const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters, unsigned int dimensions
)
    : Impedance(dimensions) {
  this->set_parameters(parameters);
}

template<class S>
void Impedance<S>::clamp_force(Eigen::VectorXd& force) {
  if (*this->force_limit_) {
    force = force.cwiseMax(-this->force_limit_->get_value()).cwiseMin(this->force_limit_->get_value());
  }
}

template<class S>
void Impedance<S>::validate_and_set_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter
) {
  if (parameter->get_name() == "stiffness") {
    this->stiffness_->set_value(this->gain_matrix_from_parameter(parameter));
  } else if (parameter->get_name() == "damping") {
    this->damping_->set_value(this->gain_matrix_from_parameter(parameter));
  } else if (parameter->get_name() == "inertia") {
    this->inertia_->set_value(this->gain_matrix_from_parameter(parameter));
  } else if (parameter->get_name() == "feed_forward_force") {
    this->feed_forward_force_->set_value(parameter->get_parameter_value<bool>());
  } else if (parameter->get_name() == "force_limit") {
    if (parameter->get_parameter_type() == state_representation::ParameterType::MATRIX) {
      throw state_representation::exceptions::InvalidParameterException(
          "Parameter " + parameter->get_name() + " has incorrect type"
      );
    }
    auto limit_matrix = this->gain_matrix_from_parameter(parameter);
    this->force_limit_->set_value(limit_matrix.diagonal());
  } else {
    throw state_representation::exceptions::InvalidParameterException(
        "No parameter with name '" + parameter->get_name() + "' found"
    );
  }
}

template<class S>
Eigen::MatrixXd
Impedance<S>::gain_matrix_from_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter) {
  Eigen::MatrixXd matrix;
  if (parameter->get_parameter_type() == state_representation::ParameterType::DOUBLE) {
    auto gain = std::static_pointer_cast<state_representation::Parameter<double>>(parameter);
    matrix = gain->get_value() * Eigen::MatrixXd::Identity(this->dimensions_, this->dimensions_);
  } else if (parameter->get_parameter_type() == state_representation::ParameterType::DOUBLE_ARRAY) {
    auto gain = std::static_pointer_cast<state_representation::Parameter<std::vector<double>>>(parameter);
    if (gain->get_value().size() == 1) {
      matrix = gain->get_value().at(0) * Eigen::MatrixXd::Identity(this->dimensions_, this->dimensions_);
    } else if (gain->get_value().size() == this->dimensions_) {
      Eigen::VectorXd diagonal = Eigen::VectorXd::Map(gain->get_value().data(), this->dimensions_);
      matrix = diagonal.asDiagonal();
    } else {
      throw state_representation::exceptions::IncompatibleSizeException(
          "The provided diagonal coefficients do not match the dimensionality of the controller ("
          + std::to_string(this->dimensions_) + ")"
      );
    }
  } else if (parameter->get_parameter_type() == state_representation::ParameterType::VECTOR) {
    auto gain = std::static_pointer_cast<state_representation::Parameter<Eigen::VectorXd>>(parameter);
    if (gain->get_value().size() == 1) {
      matrix = gain->get_value()(0) * Eigen::MatrixXd::Identity(this->dimensions_, this->dimensions_);
    } else if (gain->get_value().size() == this->dimensions_) {
      matrix = gain->get_value().asDiagonal();
    } else {
      throw state_representation::exceptions::IncompatibleSizeException(
          "The provided diagonal coefficients do not match the dimensionality of the controller ("
          + std::to_string(this->dimensions_) + ")"
      );
    }
  } else if (parameter->get_parameter_type() == state_representation::ParameterType::MATRIX) {
    auto gain = std::static_pointer_cast<state_representation::Parameter<Eigen::MatrixXd>>(parameter);
    if (gain->get_value().rows() != this->dimensions_ || gain->get_value().cols() != this->dimensions_) {
      auto dim = std::to_string(this->dimensions_);
      throw state_representation::exceptions::IncompatibleSizeException(
          "The provided matrix does not have the expected size (" + dim + "x" + dim + ")"
      );
    }
    matrix = gain->get_value();
  } else {
    throw state_representation::exceptions::InvalidParameterException(
        "Parameter " + parameter->get_name() + " has incorrect type"
    );
  }
  if ((matrix.array() < 0).any()) {
    throw state_representation::exceptions::InvalidParameterException(
        "Parameter " + parameter->get_name() + " cannot have negative elements"
    );
  }
  return matrix;
}
}// namespace controllers::impedance
