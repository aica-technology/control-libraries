#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/IncompatibleReferenceFramesException.hpp"
#include "state_representation/exceptions/NotImplementedException.hpp"

namespace state_representation {

using namespace exceptions;

CartesianState::CartesianState() : SpatialState() {
  this->set_type(StateType::CARTESIAN_STATE);
  this->set_zero();
}

CartesianState::CartesianState(const std::string& name, const std::string& reference) : SpatialState(name, reference) {
  this->set_type(StateType::CARTESIAN_STATE);
  this->set_zero();
}

CartesianState::CartesianState(const CartesianState& state) :
    SpatialState(state),
    position_(state.position_),
    orientation_(state.orientation_),
    linear_velocity_(state.linear_velocity_),
    angular_velocity_(state.angular_velocity_),
    linear_acceleration_(state.linear_acceleration_),
    angular_acceleration_(state.angular_acceleration_),
    force_(state.force_),
    torque_(state.torque_) {
  this->set_type(StateType::CARTESIAN_STATE);
}

CartesianState CartesianState::Identity(const std::string& name, const std::string& reference) {
  CartesianState identity = CartesianState(name, reference);
  // as opposed to the constructor specify this state to be filled
  identity.set_empty(false);
  return identity;
}

CartesianState CartesianState::Random(const std::string& name, const std::string& reference) {
  CartesianState random = CartesianState(name, reference);
  // set all the state variables to random
  random.set_state_variable(Eigen::VectorXd::Random(25), CartesianStateVariable::ALL);
  return random;
}

CartesianState& CartesianState::operator=(const CartesianState& state) {
  CartesianState tmp(state);
  swap(*this, tmp);
  return *this;
}

Eigen::VectorXd CartesianState::get_state_variable(const CartesianStateVariable& state_variable_type) const {
  switch (state_variable_type) {
    case CartesianStateVariable::POSITION:
      return this->get_position();

    case CartesianStateVariable::ORIENTATION:
      return this->get_orientation_coefficients();

    case CartesianStateVariable::POSE:
      return this->get_pose();

    case CartesianStateVariable::LINEAR_VELOCITY:
      return this->get_linear_velocity();

    case CartesianStateVariable::ANGULAR_VELOCITY:
      return this->get_angular_velocity();

    case CartesianStateVariable::TWIST:
      return this->get_twist();

    case CartesianStateVariable::LINEAR_ACCELERATION:
      return this->get_linear_acceleration();

    case CartesianStateVariable::ANGULAR_ACCELERATION:
      return this->get_angular_acceleration();

    case CartesianStateVariable::ACCELERATION:
      return this->get_acceleration();

    case CartesianStateVariable::FORCE:
      return this->get_force();

    case CartesianStateVariable::TORQUE:
      return this->get_torque();

    case CartesianStateVariable::WRENCH:
      return this->get_wrench();

    case CartesianStateVariable::ALL:
      Eigen::VectorXd all_fields(25);
      all_fields << this->get_pose(), this->get_twist(), this->get_acceleration(), this->get_wrench();
      return all_fields;
  }
  // this never goes here but is compulsory to avoid a warning
  return Eigen::Vector3d::Zero();
}

const Eigen::Vector3d& CartesianState::get_position() const {
  return this->position_;
}

const Eigen::Quaterniond& CartesianState::get_orientation() const {
  return this->orientation_;
}

Eigen::Vector4d CartesianState::get_orientation_coefficients() const {
  return Eigen::Vector4d(
      this->get_orientation().w(), this->get_orientation().x(), this->get_orientation().y(),
      this->get_orientation().z());
}

Eigen::Matrix<double, 7, 1> CartesianState::get_pose() const {
  Eigen::Matrix<double, 7, 1> pose;
  pose << this->get_position(), this->get_orientation_coefficients();
  return pose;
}

Eigen::Matrix4d CartesianState::get_transformation_matrix() const {
  Eigen::Matrix4d pose;
  pose << this->orientation_.toRotationMatrix(), this->position_, 0., 0., 0., 1;
  return pose;
}

const Eigen::Vector3d& CartesianState::get_linear_velocity() const {
  return this->linear_velocity_;
}

const Eigen::Vector3d& CartesianState::get_angular_velocity() const {
  return this->angular_velocity_;
}

Eigen::Matrix<double, 6, 1> CartesianState::get_twist() const {
  Eigen::Matrix<double, 6, 1> twist;
  twist << this->get_linear_velocity(), this->get_angular_velocity();
  return twist;
}

const Eigen::Vector3d& CartesianState::get_linear_acceleration() const {
  return this->linear_acceleration_;
}

const Eigen::Vector3d& CartesianState::get_angular_acceleration() const {
  return this->angular_acceleration_;
}

Eigen::Matrix<double, 6, 1> CartesianState::get_acceleration() const {
  Eigen::Matrix<double, 6, 1> acceleration;
  acceleration << this->get_linear_acceleration(), this->get_angular_acceleration();
  return acceleration;
}

const Eigen::Vector3d& CartesianState::get_force() const {
  return this->force_;
}

const Eigen::Vector3d& CartesianState::get_torque() const {
  return this->torque_;
}

Eigen::Matrix<double, 6, 1> CartesianState::get_wrench() const {
  Eigen::Matrix<double, 6, 1> wrench;
  wrench << this->get_force(), this->get_torque();
  return wrench;
}

Eigen::VectorXd CartesianState::data() const {
  return this->get_state_variable(CartesianStateVariable::ALL);
}

Eigen::ArrayXd CartesianState::array() const {
  return this->data().array();
}

std::vector<double> CartesianState::to_std_vector() const {
  Eigen::VectorXd data = this->data();
  return std::vector<double>(data.data(), data.data() + data.size());
}

void CartesianState::set_all_state_variables(const Eigen::VectorXd& new_values) {
  if (new_values.size() != 25) {
    throw exceptions::IncompatibleSizeException(
        "Input is of incorrect size: expected 25, given " + std::to_string(new_values.size()));
  }
  this->set_pose(new_values.segment(0, 7));
  this->set_twist(new_values.segment(7, 6));
  this->set_acceleration(new_values.segment(13, 6));
  this->set_wrench(new_values.segment(19, 6));
}

void CartesianState::set_state_variable(Eigen::Vector3d& state_variable, const Eigen::Vector3d& new_value) {
  this->set_empty(false);
  state_variable = new_value;
}

void CartesianState::set_state_variable(Eigen::Vector3d& state_variable, const std::vector<double>& new_value) {
  if (new_value.size() != 3) {
    throw exceptions::IncompatibleSizeException(
        "Input vector is of incorrect size: expected 3, given " + std::to_string(new_value.size()));
  }
  this->set_state_variable(state_variable, Eigen::Vector3d::Map(new_value.data(), new_value.size()));
}

void CartesianState::set_state_variable(
    Eigen::Vector3d& linear_state_variable, Eigen::Vector3d& angular_state_variable,
    const Eigen::Matrix<double, 6, 1>& new_value
) {
  this->set_state_variable(linear_state_variable, new_value.head(3));
  this->set_state_variable(angular_state_variable, new_value.tail(3));
}

void CartesianState::set_state_variable(
    const Eigen::VectorXd& new_value, const CartesianStateVariable& state_variable_type
) {
  switch (state_variable_type) {
    case CartesianStateVariable::POSITION:
      this->set_position(new_value);
      break;

    case CartesianStateVariable::ORIENTATION:
      this->set_orientation(new_value);
      break;

    case CartesianStateVariable::POSE:
      this->set_pose(new_value);
      break;

    case CartesianStateVariable::LINEAR_VELOCITY:
      this->set_linear_velocity(new_value);
      break;

    case CartesianStateVariable::ANGULAR_VELOCITY:
      this->set_angular_velocity(new_value);
      break;

    case CartesianStateVariable::TWIST:
      this->set_twist(new_value);
      break;

    case CartesianStateVariable::LINEAR_ACCELERATION:
      this->set_linear_acceleration(new_value);
      break;

    case CartesianStateVariable::ANGULAR_ACCELERATION:
      this->set_angular_acceleration(new_value);
      break;

    case CartesianStateVariable::ACCELERATION:
      this->set_acceleration(new_value);
      break;

    case CartesianStateVariable::FORCE:
      this->set_force(new_value);
      break;

    case CartesianStateVariable::TORQUE:
      this->set_torque(new_value);
      break;

    case CartesianStateVariable::WRENCH:
      this->set_wrench(new_value);
      break;

    case CartesianStateVariable::ALL:
      this->set_pose(new_value.segment(0, 7));
      this->set_twist(new_value.segment(7, 6));
      this->set_acceleration(new_value.segment(13, 6));
      this->set_wrench(new_value.segment(19, 6));
      break;
  }
}

void CartesianState::set_position(const Eigen::Vector3d& position) {
  this->set_state_variable(this->position_, position);
}

void CartesianState::set_position(const std::vector<double>& position) {
  this->set_state_variable(this->position_, position);
}

void CartesianState::set_position(const double& x, const double& y, const double& z) {
  this->set_position(Eigen::Vector3d(x, y, z));
}

void CartesianState::set_orientation(const Eigen::Quaterniond& orientation) {
  this->set_empty(false);
  this->orientation_ = orientation.normalized();
}

void CartesianState::set_orientation(const Eigen::Vector4d& orientation) {
  this->set_orientation(Eigen::Quaterniond(orientation(0), orientation(1), orientation(2), orientation(3)));
}

void CartesianState::set_orientation(const std::vector<double>& orientation) {
  if (orientation.size() != 4) {
    throw exceptions::IncompatibleSizeException("The input vector is not of size 4 required for orientation");
  }
  this->set_orientation(Eigen::Vector4d::Map(orientation.data(), orientation.size()));
}

void CartesianState::set_orientation(const double& w, const double& x, const double& y, const double& z) {
  this->set_orientation(Eigen::Vector4d(w, x, y, z));
}

void CartesianState::set_pose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
  this->set_position(position);
  this->set_orientation(orientation);
}

void CartesianState::set_pose(const Eigen::Matrix<double, 7, 1>& pose) {
  this->set_position(pose.head(3));
  this->set_orientation(pose.tail(4));
}

void CartesianState::set_pose(const std::vector<double>& pose) {
  if (pose.size() != 7) {
    throw exceptions::IncompatibleSizeException("The input vector is not of size 7 required for pose");
  }
  this->set_position(std::vector<double>(pose.begin(), pose.begin() + 3));
  this->set_orientation(std::vector<double>(pose.begin() + 3, pose.end()));
}

void CartesianState::set_linear_velocity(const Eigen::Vector3d& linear_velocity) {
  this->set_state_variable(this->linear_velocity_, linear_velocity);
}

void CartesianState::set_linear_velocity(const std::vector<double>& linear_velocity) {
  this->set_state_variable(this->linear_velocity_, linear_velocity);
}

void CartesianState::set_linear_velocity(const double& x, const double& y, const double& z) {
  this->set_linear_velocity(Eigen::Vector3d(x, y, z));
}

void CartesianState::set_angular_velocity(const Eigen::Vector3d& angular_velocity) {
  this->set_state_variable(this->angular_velocity_, angular_velocity);
}

void CartesianState::set_angular_velocity(const std::vector<double>& angular_velocity) {
  this->set_state_variable(this->angular_velocity_, angular_velocity);
}

void CartesianState::set_angular_velocity(const double& x, const double& y, const double& z) {
  this->set_angular_velocity(Eigen::Vector3d(x, y, z));
}

void CartesianState::set_twist(const Eigen::Matrix<double, 6, 1>& twist) {
  this->set_state_variable(this->linear_velocity_, this->angular_velocity_, twist);
}

void CartesianState::set_twist(const std::vector<double>& twist) {
  if (twist.size() != 6) {
    throw exceptions::IncompatibleSizeException("The input vector is not of size 6 required for twist");
  }
  this->set_linear_velocity(std::vector<double>(twist.begin(), twist.begin() + 3));
  this->set_angular_velocity(std::vector<double>(twist.begin() + 3, twist.end()));
}

void CartesianState::set_linear_acceleration(const Eigen::Vector3d& linear_acceleration) {
  this->set_state_variable(this->linear_acceleration_, linear_acceleration);
}

void CartesianState::set_linear_acceleration(const std::vector<double>& linear_acceleration) {
  this->set_state_variable(this->linear_acceleration_, linear_acceleration);
}

void CartesianState::set_linear_acceleration(const double& x, const double& y, const double& z) {
  this->set_linear_acceleration(Eigen::Vector3d(x, y, z));
}

void CartesianState::set_angular_acceleration(const Eigen::Vector3d& angular_acceleration) {
  this->set_state_variable(this->angular_acceleration_, angular_acceleration);
}

void CartesianState::set_angular_acceleration(const std::vector<double>& angular_acceleration) {
  this->set_state_variable(this->angular_acceleration_, angular_acceleration);
}

void CartesianState::set_angular_acceleration(const double& x, const double& y, const double& z) {
  this->set_angular_acceleration(Eigen::Vector3d(x, y, z));
}

void CartesianState::set_acceleration(const Eigen::Matrix<double, 6, 1>& acceleration) {
  this->set_state_variable(this->linear_acceleration_, this->angular_acceleration_, acceleration);
}

void CartesianState::set_acceleration(const std::vector<double>& acceleration) {
  if (acceleration.size() != 6) {
    throw exceptions::IncompatibleSizeException("The input vector is not of size 6 required for acceleration");
  }
  this->set_linear_acceleration(std::vector<double>(acceleration.begin(), acceleration.begin() + 3));
  this->set_angular_acceleration(std::vector<double>(acceleration.begin() + 3, acceleration.end()));
}

void CartesianState::set_force(const Eigen::Vector3d& force) {
  this->set_state_variable(this->force_, force);
}

void CartesianState::set_force(const std::vector<double>& force) {
  this->set_state_variable(this->force_, force);
}

void CartesianState::set_force(const double& x, const double& y, const double& z) {
  this->set_force(Eigen::Vector3d(x, y, z));
}

void CartesianState::set_torque(const Eigen::Vector3d& torque) {
  this->set_state_variable(this->torque_, torque);
}

void CartesianState::set_torque(const std::vector<double>& torque) {
  this->set_state_variable(this->torque_, torque);
}

void CartesianState::set_torque(const double& x, const double& y, const double& z) {
  this->set_torque(Eigen::Vector3d(x, y, z));
}

void CartesianState::set_wrench(const Eigen::Matrix<double, 6, 1>& wrench) {
  this->set_state_variable(this->force_, this->torque_, wrench);
}

void CartesianState::set_wrench(const std::vector<double>& wrench) {
  if (wrench.size() != 6) {
    throw exceptions::IncompatibleSizeException("The input vector is not of size 6 required for wrench");
  }
  this->set_force(std::vector<double>(wrench.begin(), wrench.begin() + 3));
  this->set_torque(std::vector<double>(wrench.begin() + 3, wrench.end()));
}

void CartesianState::set_data(const Eigen::VectorXd& data) {
  this->set_all_state_variables(data);
}

void CartesianState::set_data(const std::vector<double>& data) {
  this->set_all_state_variables(Eigen::VectorXd::Map(data.data(), data.size()));
}

void CartesianState::set_zero() {
  this->position_.setZero();
  this->orientation_.setIdentity();
  this->linear_velocity_.setZero();
  this->angular_velocity_.setZero();
  this->linear_acceleration_.setZero();
  this->angular_acceleration_.setZero();
  this->force_.setZero();
  this->torque_.setZero();
}

void CartesianState::clamp_state_variable(
    double max_norm, const CartesianStateVariable& state_variable_type, double noise_ratio
) {
  if (state_variable_type == CartesianStateVariable::ORIENTATION || state_variable_type == CartesianStateVariable::POSE
      || state_variable_type == CartesianStateVariable::ALL) {
    throw NotImplementedException("clamp_state_variable is not implemented for this CartesianStateVariable");
  }
  Eigen::VectorXd state_variable_value = this->get_state_variable(state_variable_type);
  if (noise_ratio != 0 && state_variable_value.norm() < noise_ratio * max_norm) {
    // apply a dead zone
    state_variable_value.setZero();
  } else if (state_variable_value.norm() > max_norm) {
    // clamp the values to their maximum amplitude provided
    state_variable_value = max_norm * state_variable_value.normalized();
  }
  this->set_state_variable(state_variable_value, state_variable_type);
}

CartesianState CartesianState::copy() const {
  CartesianState result(*this);
  return result;
}

double CartesianState::dist(const CartesianState& state, const CartesianStateVariable& state_variable_type) const {
  // sanity check
  if (this->is_empty()) {
    throw EmptyStateException(this->get_name() + " state is empty");
  }
  if (state.is_empty()) {
    throw EmptyStateException(state.get_name() + " state is empty");
  }
  if (!(this->get_reference_frame() == state.get_reference_frame())) {
    throw IncompatibleReferenceFramesException("The two states do not have the same reference frame");
  }
  // calculation
  double result = 0;
  if (state_variable_type == CartesianStateVariable::POSITION || state_variable_type == CartesianStateVariable::POSE
      || state_variable_type == CartesianStateVariable::ALL) {
    result += (this->get_position() - state.get_position()).norm();
  }
  if (state_variable_type == CartesianStateVariable::ORIENTATION || state_variable_type == CartesianStateVariable::POSE
      || state_variable_type == CartesianStateVariable::ALL) {
    // https://math.stackexchange.com/questions/90081/quaternion-distance for orientation
    double inner_product = this->get_orientation().dot(state.get_orientation());
    double argument = 2 * inner_product * inner_product - 1;
    result += acos(std::min(1.0, std::max(-1.0, argument)));
  }
  if (state_variable_type == CartesianStateVariable::LINEAR_VELOCITY
      || state_variable_type == CartesianStateVariable::TWIST || state_variable_type == CartesianStateVariable::ALL) {
    result += (this->get_linear_velocity() - state.get_linear_velocity()).norm();
  }
  if (state_variable_type == CartesianStateVariable::ANGULAR_VELOCITY
      || state_variable_type == CartesianStateVariable::TWIST || state_variable_type == CartesianStateVariable::ALL) {
    result += (this->get_angular_velocity() - state.get_angular_velocity()).norm();
  }
  if (state_variable_type == CartesianStateVariable::LINEAR_ACCELERATION
      || state_variable_type == CartesianStateVariable::ACCELERATION
      || state_variable_type == CartesianStateVariable::ALL) {
    result += (this->get_linear_acceleration() - state.get_linear_acceleration()).norm();
  }
  if (state_variable_type == CartesianStateVariable::ANGULAR_ACCELERATION
      || state_variable_type == CartesianStateVariable::ACCELERATION
      || state_variable_type == CartesianStateVariable::ALL) {
    result += (this->get_angular_acceleration() - state.get_angular_acceleration()).norm();
  }
  if (state_variable_type == CartesianStateVariable::FORCE || state_variable_type == CartesianStateVariable::WRENCH
      || state_variable_type == CartesianStateVariable::ALL) {
    result += (this->get_force() - state.get_force()).norm();
  }
  if (state_variable_type == CartesianStateVariable::TORQUE || state_variable_type == CartesianStateVariable::WRENCH
      || state_variable_type == CartesianStateVariable::ALL) {
    result += (this->get_torque() - state.get_torque()).norm();
  }
  return result;
}

double dist(const CartesianState& s1, const CartesianState& s2, const CartesianStateVariable& state_variable_type) {
  return s1.dist(s2, state_variable_type);
}

void CartesianState::initialize() {
  this->State::initialize();
  this->set_zero();
}

CartesianState CartesianState::inverse() const {
  CartesianState inverse(*this);
  // invert name and reference frame
  std::string ref = inverse.get_reference_frame();
  inverse.set_reference_frame(inverse.get_name());
  inverse.set_name(ref);

  Eigen::Quaterniond inverse_orientation = this->get_orientation().conjugate();
  Eigen::Vector3d inverse_position = inverse_orientation * (-this->get_position());
  Eigen::Vector3d inverse_angular_velocity = inverse_orientation * (-this->get_angular_velocity());
  Eigen::Vector3d inverse_linear_velocity = inverse_orientation * (-this->get_linear_velocity());
  inverse_linear_velocity += inverse_angular_velocity.cross(inverse_position); // radially induced velocity

  Eigen::Vector3d inverse_angular_acceleration = inverse_orientation * (-this->get_angular_acceleration());
  Eigen::Vector3d inverse_linear_acceleration = inverse_orientation * (-this->get_linear_acceleration());
  inverse_linear_acceleration += inverse_angular_acceleration.cross(inverse_position); // Euler acceleration
  inverse_linear_acceleration += 2 * inverse_angular_velocity.cross(inverse_linear_velocity); // Coriolis acceleration
  inverse_linear_acceleration -=
      inverse_angular_velocity.cross(inverse_angular_velocity.cross(inverse_position)); // centrifugal acceleration

  // collect the results
  inverse.set_position(inverse_position);
  inverse.set_orientation(inverse_orientation);
  inverse.set_linear_velocity(inverse_linear_velocity);
  inverse.set_angular_velocity(inverse_angular_velocity);
  inverse.set_linear_acceleration(inverse_linear_acceleration);
  inverse.set_angular_acceleration(inverse_angular_acceleration);
  // TODO(#30): wrench inverse

  return inverse;
}

void CartesianState::normalize(const CartesianStateVariable& state_variable_type) {
  if (state_variable_type == CartesianStateVariable::POSITION || state_variable_type == CartesianStateVariable::POSE
      || state_variable_type == CartesianStateVariable::ALL) {
    this->position_.normalize();
  }
  // there shouldn't be a need to renormalize orientation as it is already normalized
  if (state_variable_type == CartesianStateVariable::ORIENTATION || state_variable_type == CartesianStateVariable::POSE
      || state_variable_type == CartesianStateVariable::ALL) {
    this->orientation_.normalize();
  }
  if (state_variable_type == CartesianStateVariable::LINEAR_VELOCITY
      || state_variable_type == CartesianStateVariable::TWIST || state_variable_type == CartesianStateVariable::ALL) {
    this->linear_velocity_.normalize();
  }
  if (state_variable_type == CartesianStateVariable::ANGULAR_VELOCITY
      || state_variable_type == CartesianStateVariable::TWIST || state_variable_type == CartesianStateVariable::ALL) {
    this->angular_velocity_.normalize();
  }
  if (state_variable_type == CartesianStateVariable::LINEAR_ACCELERATION
      || state_variable_type == CartesianStateVariable::ACCELERATION
      || state_variable_type == CartesianStateVariable::ALL) {
    this->linear_acceleration_.normalize();
  }
  if (state_variable_type == CartesianStateVariable::ANGULAR_ACCELERATION
      || state_variable_type == CartesianStateVariable::ACCELERATION
      || state_variable_type == CartesianStateVariable::ALL) {
    this->angular_acceleration_.normalize();
  }
  if (state_variable_type == CartesianStateVariable::FORCE || state_variable_type == CartesianStateVariable::WRENCH
      || state_variable_type == CartesianStateVariable::ALL) {
    this->force_.normalize();
  }
  if (state_variable_type == CartesianStateVariable::TORQUE || state_variable_type == CartesianStateVariable::WRENCH
      || state_variable_type == CartesianStateVariable::ALL) {
    this->torque_.normalize();
  }
}

CartesianState CartesianState::normalized(const CartesianStateVariable& state_variable_type) const {
  CartesianState result(*this);
  result.normalize(state_variable_type);
  return result;
}

std::vector<double> CartesianState::norms(const CartesianStateVariable& state_variable_type) const {
  // compute the norms for each independent state variable
  std::vector<double> norms;
  if (state_variable_type == CartesianStateVariable::POSITION || state_variable_type == CartesianStateVariable::POSE
      || state_variable_type == CartesianStateVariable::ALL) {
    norms.push_back(this->get_position().norm());
  }
  if (state_variable_type == CartesianStateVariable::ORIENTATION || state_variable_type == CartesianStateVariable::POSE
      || state_variable_type == CartesianStateVariable::ALL) {
    norms.push_back(this->get_orientation().norm());
  }
  if (state_variable_type == CartesianStateVariable::LINEAR_VELOCITY
      || state_variable_type == CartesianStateVariable::TWIST || state_variable_type == CartesianStateVariable::ALL) {
    norms.push_back(this->get_linear_velocity().norm());
  }
  if (state_variable_type == CartesianStateVariable::ANGULAR_VELOCITY
      || state_variable_type == CartesianStateVariable::TWIST || state_variable_type == CartesianStateVariable::ALL) {
    norms.push_back(this->get_angular_velocity().norm());
  }
  if (state_variable_type == CartesianStateVariable::LINEAR_ACCELERATION
      || state_variable_type == CartesianStateVariable::ACCELERATION
      || state_variable_type == CartesianStateVariable::ALL) {
    norms.push_back(this->get_linear_acceleration().norm());
  }
  if (state_variable_type == CartesianStateVariable::ANGULAR_ACCELERATION
      || state_variable_type == CartesianStateVariable::ACCELERATION
      || state_variable_type == CartesianStateVariable::ALL) {
    norms.push_back(this->get_angular_acceleration().norm());
  }
  if (state_variable_type == CartesianStateVariable::FORCE || state_variable_type == CartesianStateVariable::WRENCH
      || state_variable_type == CartesianStateVariable::ALL) {
    norms.push_back(this->get_force().norm());
  }
  if (state_variable_type == CartesianStateVariable::TORQUE || state_variable_type == CartesianStateVariable::WRENCH
      || state_variable_type == CartesianStateVariable::ALL) {
    norms.push_back(this->get_torque().norm());
  }
  return norms;
}

CartesianState& CartesianState::operator*=(const CartesianState& state) {
  // sanity check
  if (this->is_empty()) {
    throw EmptyStateException(this->get_name() + " state is empty");
  }
  if (state.is_empty()) {
    throw EmptyStateException(state.get_name() + " state is empty");
  }
  if (this->get_name() != state.get_reference_frame()) {
    throw IncompatibleReferenceFramesException("Expected " + this->get_name() + ", got " + state.get_reference_frame());
  }
  this->set_name(state.get_name());
  // intermediate variables for f_S_b
  Eigen::Vector3d f_P_b = this->get_position();
  Eigen::Quaterniond f_R_b = this->get_orientation();
  Eigen::Vector3d f_v_b = this->get_linear_velocity();
  Eigen::Vector3d f_omega_b = this->get_angular_velocity();
  Eigen::Vector3d f_a_b = this->get_linear_acceleration();
  Eigen::Vector3d f_alpha_b = this->get_angular_acceleration();
  // intermediate variables for b_S_c
  Eigen::Vector3d b_P_c = state.get_position();
  // specific operation on quaternion using Hamilton product, keeping the resulting quaternion on the same hemisphere
  Eigen::Quaterniond b_R_c = state.get_orientation();
  Eigen::Vector3d b_v_c = state.get_linear_velocity();
  Eigen::Vector3d b_omega_c = state.get_angular_velocity();
  Eigen::Vector3d b_a_c = state.get_linear_acceleration();
  Eigen::Vector3d b_alpha_c = state.get_angular_acceleration();
  // pose
  this->set_position(f_P_b + f_R_b * b_P_c);
  auto orientation = f_R_b * b_R_c;
  if (orientation.dot(this->get_orientation()) < 0) {
    orientation = Eigen::Quaterniond(-orientation.coeffs());
  }
  this->set_orientation(orientation);
  // twist
  this->set_linear_velocity(f_v_b + f_R_b * b_v_c + f_omega_b.cross(f_R_b * b_P_c));
  this->set_angular_velocity(f_omega_b + f_R_b * b_omega_c);
  // acceleration
  this->set_linear_acceleration(
      f_a_b + f_R_b * b_a_c + f_alpha_b.cross(f_R_b * b_P_c) + 2 * f_omega_b.cross(f_R_b * b_v_c)
          + f_omega_b.cross(f_omega_b.cross(f_R_b * b_P_c)));
  this->set_angular_acceleration(f_alpha_b + f_R_b * b_alpha_c + f_omega_b.cross(f_R_b * b_omega_c));
  // wrench
  //TODO
  return (*this);
}

CartesianState CartesianState::operator*(const CartesianState& state) const {
  CartesianState result(*this);
  result *= state;
  return result;
}

CartesianState& CartesianState::operator*=(double lambda) {
  // sanity check
  if (this->is_empty()) {
    throw EmptyStateException(this->get_name() + " state is empty");
  }
  // operation
  this->set_position(lambda * this->get_position());
  // calculate the scaled rotation as a displacement from identity
  Eigen::Quaterniond w = math_tools::log(this->get_orientation());
  // calculate the orientation corresponding to the scaled velocity
  this->set_orientation(math_tools::exp(w, lambda / 2.));
  // calculate the other vectors normally
  this->set_twist(lambda * this->get_twist());
  this->set_acceleration(lambda * this->get_acceleration());
  this->set_wrench(lambda * this->get_wrench());
  return (*this);
}

CartesianState CartesianState::operator*(double lambda) const {
  CartesianState result(*this);
  result *= lambda;
  return result;
}

CartesianState operator*(double lambda, const CartesianState& state) {
  return state * lambda;
}

Eigen::Vector3d CartesianState::operator*(const Eigen::Vector3d& vector) const {
  return this->get_orientation() * vector + this->get_position();
}

CartesianState& CartesianState::operator/=(double lambda) {
  if (std::abs(lambda) < std::numeric_limits<double>::min()) {
    throw std::runtime_error("Division by zero is not allowed");
  }
  lambda = 1.0 / lambda;
  return this->operator*=(lambda);
}

CartesianState CartesianState::operator/(double lambda) const {
  CartesianState result(*this);
  result /= lambda;
  return result;
}

CartesianState& CartesianState::operator+=(const CartesianState& state) {
  // sanity check
  if (this->is_empty()) {
    throw EmptyStateException(this->get_name() + " state is empty");
  }
  if (state.is_empty()) {
    throw EmptyStateException(state.get_name() + " state is empty");
  }
  if (!(this->get_reference_frame() == state.get_reference_frame())) {
    throw IncompatibleReferenceFramesException("The two states do not have the same reference frame");
  }
  // operation on pose
  this->set_position(this->get_position() + state.get_position());
  // specific operation on quaternion using Hamilton product, keeping the resulting quaternion on the same hemisphere
  auto orientation = this->get_orientation() * state.get_orientation();
  if (orientation.dot(this->get_orientation()) < 0) {
    orientation = Eigen::Quaterniond(-orientation.coeffs());
  }
  this->set_orientation(orientation);
  // operation on twist
  this->set_twist(this->get_twist() + state.get_twist());
  // operation on acceleration
  this->set_acceleration(this->get_acceleration() + state.get_acceleration());
  // operation on wrench
  this->set_wrench(this->get_wrench() + state.get_wrench());
  return (*this);
}

CartesianState CartesianState::operator+(const CartesianState& state) const {
  CartesianState result(*this);
  result += state;
  return result;
}

CartesianState CartesianState::operator-() const {
  // sanity check
  if (this->is_empty()) {
    throw EmptyStateException(this->get_name() + " state is empty");
  }
  // create a copy of the state
  CartesianState result(*this);
  // operation on pose
  result.set_position(-result.get_position());
  result.set_orientation(result.get_orientation().conjugate());
  // operation on twist
  result.set_twist(-result.get_twist());
  // operation on acceleration
  result.set_acceleration(-result.get_acceleration());
  // operation on wrench
  result.set_wrench(-result.get_wrench());
  return result;
}

CartesianState& CartesianState::operator-=(const CartesianState& state) {
  (*this) += -state;
  return (*this);
}

CartesianState CartesianState::operator-(const CartesianState& state) const {
  CartesianState result(*this);
  result -= state;
  return result;
}

std::ostream& operator<<(std::ostream& os, const CartesianState& state) {
  if (state.is_empty()) {
    os << "Empty CartesianState";
  } else {
    os << state.get_name() << " CartesianState expressed in " << state.get_reference_frame() << " frame" << std::endl;
    os << "position: (" << state.position_(0) << ", ";
    os << state.position_(1) << ", ";
    os << state.position_(2) << ")" << std::endl;
    os << "orientation: (" << state.orientation_.w() << ", ";
    os << state.orientation_.x() << ", ";
    os << state.orientation_.y() << ", ";
    os << state.orientation_.z() << ")";
    Eigen::AngleAxisd axis_angle(state.orientation_);
    os << " <=> theta: " << axis_angle.angle() << ", ";
    os << "axis: (" << axis_angle.axis()(0) << ", ";
    os << axis_angle.axis()(1) << ", ";
    os << axis_angle.axis()(2) << ")" << std::endl;
    os << "linear velocity: (" << state.linear_velocity_(0) << ", ";
    os << state.linear_velocity_(1) << ", ";
    os << state.linear_velocity_(2) << ")" << std::endl;
    os << "angular velocity: (" << state.angular_velocity_(0) << ", ";
    os << state.angular_velocity_(1) << ", ";
    os << state.angular_velocity_(2) << ")" << std::endl;
    os << "linear acceleration: (" << state.linear_acceleration_(0) << ", ";
    os << state.linear_acceleration_(1) << ", ";
    os << state.linear_acceleration_(2) << ")" << std::endl;
    os << "angular acceleration: (" << state.angular_acceleration_(0) << ", ";
    os << state.angular_acceleration_(1) << ", ";
    os << state.angular_acceleration_(2) << ")" << std::endl;
    os << "force: (" << state.force_(0) << ", ";
    os << state.force_(1) << ", ";
    os << state.force_(2) << ")" << std::endl;
    os << "torque: (" << state.torque_(0) << ", ";
    os << state.torque_(1) << ", ";
    os << state.torque_(2) << ")";
  }
  return os;
}

}// namespace state_representation
