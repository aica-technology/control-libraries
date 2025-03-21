#pragma once

#include "dynamical_systems/IDynamicalSystem.hpp"
#include "state_representation/parameters/Parameter.hpp"

namespace dynamical_systems {

/**
 * @class PointAttractor
 * @brief Represents a dynamical system to move towards an attractor.
 * @tparam S Underlying state type of the dynamical system
 */
template<class S>
class PointAttractor : public IDynamicalSystem<S> {
public:
  /**
   * @brief Empty constructor
   */
  PointAttractor();

  /**
   * @brief Constructor from an initial parameter list
   * @param parameters A parameter list containing initial attractor and gain values
   */
  explicit PointAttractor(const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters);

  /**
   * @copydoc IDynamicalSystem::set_base_frame
   */
  void set_base_frame(const S& base_frame) override;

  /**
   * @copydoc IDynamicalSystem::compute_dynamics
   */
  [[nodiscard]] S compute_dynamics(const S& state) const override;

  /**
   * @copydoc IDynamicalSystem::is_compatible
   */
  [[nodiscard]] bool is_compatible(const S& state) const override;

private:
  /**
   * @copydoc IDynamicalSystem::validate_and_set_parameter
   */
  void validate_and_set_parameter(const std::shared_ptr<state_representation::ParameterInterface>& parameter) override;

  /**
   * @brief Setter of the attractor as a new value
   * @param attractor the new attractor
   */
  void set_attractor(const S& attractor);

  /**
   * @brief Setter of the gain as a new value
   * @param attractor the new attractor
   */
  void set_gain(const std::shared_ptr<state_representation::ParameterInterface>& parameter, unsigned int expected_size);

  std::shared_ptr<state_representation::Parameter<S>> attractor_;///< attractor of the dynamical system in the space
  std::shared_ptr<state_representation::Parameter<Eigen::MatrixXd>> gain_;///< gain associate to the system
};

template<class S>
PointAttractor<S>::PointAttractor(const std::list<std::shared_ptr<state_representation::ParameterInterface>>& parameters
)
    : PointAttractor<S>() {
  this->set_parameters(parameters);
}
}// namespace dynamical_systems
