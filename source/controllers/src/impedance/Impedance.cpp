#include "controllers/impedance/Impedance.hpp"

#include "controllers/exceptions/NotImplementedException.hpp"
#include "state_representation/space/joint/JointState.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"

using namespace state_representation;

namespace controllers::impedance {

template<class S>
S Impedance<S>::compute_command(const S&, const S&) {
  throw exceptions::NotImplementedException("compute_command is not implemented for this state variable");
}

template<>
CartesianState Impedance<CartesianState>::compute_command(
    const CartesianState& command_state, const CartesianState& feedback_state
) {
  CartesianState state_error = command_state - feedback_state;
  // compute the wrench using the formula W = I * acc_desired + K * e_pose + D * e_twist
  CartesianState command(feedback_state.get_name(), feedback_state.get_reference_frame());
  // compute force
  Eigen::Vector3d position_control = this->stiffness_->get_value().topLeftCorner<3, 3>() * state_error.get_position()
      + this->damping_->get_value().topLeftCorner<3, 3>() * state_error.get_linear_velocity()
      + this->inertia_->get_value().topLeftCorner<3, 3>() * command_state.get_linear_acceleration();

  // compute torque (orientation requires special care)
  Eigen::Vector3d orientation_control =
      this->stiffness_->get_value().bottomRightCorner<3, 3>() * state_error.get_orientation().vec()
          + this->damping_->get_value().bottomRightCorner<3, 3>() * state_error.get_angular_velocity()
          + this->inertia_->get_value().bottomRightCorner<3, 3>() * command_state.get_angular_acceleration();

  Eigen::VectorXd wrench(6);
  wrench << position_control, orientation_control;
  // if the 'feed_forward_force' parameter is set to true, also add the wrench error to the command
  if (this->get_parameter_value<bool>("feed_forward_force")) {
    wrench += state_error.get_wrench();
  }
  clamp_force(wrench);

  command.set_wrench(wrench);
  return command;
}

template<>
JointState Impedance<JointState>::compute_command(
    const JointState& command_state, const JointState& feedback_state
) {
  JointState state_error = command_state - feedback_state;
  // compute the wrench using the formula T = I * acc_desired + K * e_pos + D * e_vel
  JointState command(feedback_state.get_name(), feedback_state.get_names());
  // compute torques
  Eigen::VectorXd torque_control = this->stiffness_->get_value() * state_error.get_positions()
      + this->damping_->get_value() * state_error.get_velocities()
      + this->inertia_->get_value() * command_state.get_accelerations();

  // if the 'feed_forward_force' parameter is set to true, also add the torque error to the command
  if (this->get_parameter_value<bool>("feed_forward_force")) {
    torque_control += state_error.get_torques();
  }
  clamp_force(torque_control);
  command.set_torques(torque_control);
  return command;
}

}// namespace controllers