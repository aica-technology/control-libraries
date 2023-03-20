#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/IncompatibleReferenceFramesException.hpp"
#include "state_representation/exceptions/NotImplementedException.hpp"

namespace state_representation {

using namespace exceptions;

static Eigen::Vector4d quat2vec(const Eigen::Quaterniond quat) {
  return {quat.w(), quat.x(), quat.y(), quat.z()};
}

static Eigen::Quaterniond vec2quat(const Eigen::Vector4d vec) {
  return Eigen::Quaterniond(vec(0), vec(1), vec(2), vec(3)).normalized();
}

static unsigned long get_state_variable_size(const CartesianStateVariable& state_variable_type) {
  switch (state_variable_type) {
    case CartesianStateVariable::POSITION:
    case CartesianStateVariable::LINEAR_VELOCITY:
    case CartesianStateVariable::ANGULAR_VELOCITY:
    case CartesianStateVariable::LINEAR_ACCELERATION:
    case CartesianStateVariable::ANGULAR_ACCELERATION:
    case CartesianStateVariable::FORCE:
    case CartesianStateVariable::TORQUE:
      return 3;
    case CartesianStateVariable::ORIENTATION:
      return 4;
    case CartesianStateVariable::TWIST:
    case CartesianStateVariable::ACCELERATION:
    case CartesianStateVariable::WRENCH:
      return 6;
    case CartesianStateVariable::POSE:
      return 7;
    case CartesianStateVariable::ALL:
      return 25;
    default:
      return 0;
  }
}

CartesianState::CartesianState() :
    SpatialState(),
    position_(Eigen::Vector3d::Zero()),
    orientation_(Eigen::Quaterniond::Identity()),
    linear_velocity_(Eigen::Vector3d::Zero()),
    angular_velocity_(Eigen::Vector3d::Zero()),
    linear_acceleration_(Eigen::Vector3d::Zero()),
    angular_acceleration_(Eigen::Vector3d::Zero()),
    force_(Eigen::Vector3d::Zero()),
    torque_(Eigen::Vector3d::Zero()) {
  this->set_type(StateType::CARTESIAN_STATE);
}

CartesianState::CartesianState(const std::string& name, const std::string& reference) : CartesianState() {
  this->set_name(name);
  this->set_reference_frame(reference);
}

CartesianState::CartesianState(const CartesianState& state) :
    CartesianState(state.get_name(), state.get_reference_frame()) {
  if (state) {
    this->set_state_variable(state.get_state_variable(CartesianStateVariable::ALL), CartesianStateVariable::ALL);
  }
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
  random.set_orientation(Eigen::Quaterniond::UnitRandom());
  return random;
}

CartesianState& CartesianState::operator=(const CartesianState& state) {
  CartesianState tmp(state);
  swap(*this, tmp);
  return *this;
}

Eigen::VectorXd CartesianState::get_state_variable(const CartesianStateVariable& state_variable_type) const {
  this->assert_not_empty();
  switch (state_variable_type) {
    case CartesianStateVariable::POSITION:
      return this->position_;
    case CartesianStateVariable::ORIENTATION:
      return quat2vec(this->orientation_);
    case CartesianStateVariable::POSE: {
      Eigen::VectorXd pose(7);
      pose << this->position_, quat2vec(this->orientation_);
      return pose;
    }
    case CartesianStateVariable::LINEAR_VELOCITY:
      return this->linear_velocity_;
    case CartesianStateVariable::ANGULAR_VELOCITY:
      return this->angular_velocity_;
    case CartesianStateVariable::TWIST: {
      Eigen::VectorXd twist(6);
      twist << this->linear_velocity_, this->angular_velocity_;
      return twist;
    }
    case CartesianStateVariable::LINEAR_ACCELERATION:
      return this->linear_acceleration_;
    case CartesianStateVariable::ANGULAR_ACCELERATION:
      return this->angular_acceleration_;
    case CartesianStateVariable::ACCELERATION: {
      Eigen::VectorXd acceleration(6);
      acceleration << this->linear_acceleration_, this->angular_acceleration_;
      return acceleration;
    }
    case CartesianStateVariable::FORCE:
      return this->force_;
    case CartesianStateVariable::TORQUE:
      return this->torque_;
    case CartesianStateVariable::WRENCH: {
      Eigen::VectorXd wrench(6);
      wrench << this->force_, this->torque_;
      return wrench;
    }
    case CartesianStateVariable::ALL: {
      Eigen::VectorXd all_fields(25);
      all_fields << this->position_, quat2vec(this->orientation_), this->linear_velocity_, this->angular_velocity_,
                    this->linear_acceleration_, this->angular_acceleration_, this->force_, this->torque_;
      return all_fields;
    }
    default:
      return {};
  }
}

const Eigen::Vector3d& CartesianState::get_position() const {
  this->assert_not_empty();
  return this->position_;
}

const Eigen::Quaterniond& CartesianState::get_orientation() const {
  this->assert_not_empty();
  return this->orientation_;
}

Eigen::Vector4d CartesianState::get_orientation_coefficients() const {
  return this->get_state_variable(CartesianStateVariable::ORIENTATION);
}

Eigen::Matrix<double, 7, 1> CartesianState::get_pose() const {
  return this->get_state_variable(CartesianStateVariable::POSE);
}

Eigen::Matrix4d CartesianState::get_transformation_matrix() const {
  this->assert_not_empty();
  Eigen::Matrix4d pose;
  pose << this->orientation_.toRotationMatrix(), this->position_, 0., 0., 0., 1;
  return pose;
}

const Eigen::Vector3d& CartesianState::get_linear_velocity() const {
  this->assert_not_empty();
  return this->linear_velocity_;
}

const Eigen::Vector3d& CartesianState::get_angular_velocity() const {
  this->assert_not_empty();
  return this->angular_velocity_;
}

Eigen::Matrix<double, 6, 1> CartesianState::get_twist() const {
  return this->get_state_variable(CartesianStateVariable::TWIST);
}

const Eigen::Vector3d& CartesianState::get_linear_acceleration() const {
  this->assert_not_empty();
  return this->linear_acceleration_;
}

const Eigen::Vector3d& CartesianState::get_angular_acceleration() const {
  this->assert_not_empty();
  return this->angular_acceleration_;
}

Eigen::Matrix<double, 6, 1> CartesianState::get_acceleration() const {
  return this->get_state_variable(CartesianStateVariable::ACCELERATION);
}

const Eigen::Vector3d& CartesianState::get_force() const {
  this->assert_not_empty();
  return this->force_;
}

const Eigen::Vector3d& CartesianState::get_torque() const {
  this->assert_not_empty();
  return this->torque_;
}

Eigen::Matrix<double, 6, 1> CartesianState::get_wrench() const {
  return this->get_state_variable(CartesianStateVariable::WRENCH);
}

Eigen::VectorXd CartesianState::data() const {
  return this->get_state_variable(CartesianStateVariable::ALL);
}

Eigen::ArrayXd CartesianState::array() const {
  return this->data().array();
}

std::vector<double> CartesianState::to_std_vector() const {
  Eigen::VectorXd data = this->data();
  return {data.data(), data.data() + data.size()};
}

void CartesianState::set_state_variable(
    const std::vector<double>& new_value, const CartesianStateVariable& state_variable_type
) {
  this->set_state_variable(Eigen::VectorXd::Map(new_value.data(), new_value.size()), state_variable_type);
}

void CartesianState::set_state_variable(
    const Eigen::VectorXd& new_value, const CartesianStateVariable& state_variable_type
) {
  auto expected_size = long(get_state_variable_size(state_variable_type));
  if (new_value.size() != expected_size) {
    throw exceptions::IncompatibleSizeException(
        "Input is of incorrect size, expected " + std::to_string(expected_size) + ", got "
            + std::to_string(new_value.size()));
  }
  switch (state_variable_type) {
    case CartesianStateVariable::POSITION:
      this->position_ = new_value;
      break;
    case CartesianStateVariable::ORIENTATION:
      this->orientation_ = vec2quat(new_value);
      break;
    case CartesianStateVariable::POSE:
      this->position_ = new_value.head(3);
      this->orientation_ = vec2quat(new_value.tail(4));
      break;
    case CartesianStateVariable::LINEAR_VELOCITY:
      this->linear_velocity_ = new_value;
      break;
    case CartesianStateVariable::ANGULAR_VELOCITY:
      this->angular_velocity_ = new_value;
      break;
    case CartesianStateVariable::TWIST:
      this->linear_velocity_ = new_value.head(3);
      this->angular_velocity_ = new_value.tail(3);
      break;
    case CartesianStateVariable::LINEAR_ACCELERATION:
      this->linear_acceleration_ = new_value;
      break;
    case CartesianStateVariable::ANGULAR_ACCELERATION:
      this->angular_acceleration_ = new_value;
      break;
    case CartesianStateVariable::ACCELERATION:
      this->linear_acceleration_ = new_value.head(3);
      this->angular_acceleration_ = new_value.tail(3);
      break;
    case CartesianStateVariable::FORCE:
      this->force_ = new_value;
      break;
    case CartesianStateVariable::TORQUE:
      this->torque_ = new_value;
      break;
    case CartesianStateVariable::WRENCH:
      this->force_ = new_value.head(3);
      this->torque_ = new_value.tail(3);
      break;
    case CartesianStateVariable::ALL:
      this->position_ = new_value.segment(0, 3);
      this->orientation_ = vec2quat(new_value.segment(3, 4));
      this->linear_velocity_ = new_value.segment(7, 3);
      this->angular_velocity_ = new_value.segment(10, 3);
      this->linear_acceleration_ = new_value.segment(13, 3);
      this->angular_acceleration_ = new_value.segment(16, 3);
      this->force_ = new_value.segment(19, 3);
      this->torque_ = new_value.segment(22, 3);
      break;
  }
  this->set_empty(false);
}

void CartesianState::set_position(const Eigen::Vector3d& position) {
  this->set_state_variable(position, CartesianStateVariable::POSITION);
}

void CartesianState::set_position(const std::vector<double>& position) {
  this->set_state_variable(position, CartesianStateVariable::POSITION);
}

void CartesianState::set_position(const double& x, const double& y, const double& z) {
  this->set_state_variable(Eigen::Vector3d(x, y, z), CartesianStateVariable::POSITION);
}

void CartesianState::set_orientation(const Eigen::Quaterniond& orientation) {
  // orientation is a special case, to avoid transforming between vector and quaternion, set it here directly
  // but also set filled as in set_state_variable
  this->orientation_ = orientation.normalized();
  this->set_empty(false);
}

void CartesianState::set_orientation(const Eigen::Vector4d& orientation) {
  this->set_state_variable(orientation, CartesianStateVariable::ORIENTATION);
}

void CartesianState::set_orientation(const std::vector<double>& orientation) {
  this->set_state_variable(orientation, CartesianStateVariable::ORIENTATION);
}

void CartesianState::set_orientation(const double& w, const double& x, const double& y, const double& z) {
  this->set_state_variable(Eigen::Vector4d(w, x, y, z), CartesianStateVariable::ORIENTATION);
}

void CartesianState::set_pose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
  this->orientation_ = orientation.normalized();
  this->set_state_variable(position, CartesianStateVariable::POSITION);
}

void CartesianState::set_pose(const Eigen::Matrix<double, 7, 1>& pose) {
  this->set_state_variable(pose, CartesianStateVariable::POSE);
}

void CartesianState::set_pose(const std::vector<double>& pose) {
  this->set_state_variable(pose, CartesianStateVariable::POSE);
}

void CartesianState::set_linear_velocity(const Eigen::Vector3d& linear_velocity) {
  this->set_state_variable(linear_velocity, CartesianStateVariable::LINEAR_VELOCITY);
}

void CartesianState::set_linear_velocity(const std::vector<double>& linear_velocity) {
  this->set_state_variable(linear_velocity, CartesianStateVariable::LINEAR_VELOCITY);
}

void CartesianState::set_linear_velocity(const double& x, const double& y, const double& z) {
  this->set_state_variable(Eigen::Vector3d(x, y, z), CartesianStateVariable::LINEAR_VELOCITY);
}

void CartesianState::set_angular_velocity(const Eigen::Vector3d& angular_velocity) {
  this->set_state_variable(angular_velocity, CartesianStateVariable::ANGULAR_VELOCITY);
}

void CartesianState::set_angular_velocity(const std::vector<double>& angular_velocity) {
  this->set_state_variable(angular_velocity, CartesianStateVariable::ANGULAR_VELOCITY);
}

void CartesianState::set_angular_velocity(const double& x, const double& y, const double& z) {
  this->set_state_variable(Eigen::Vector3d(x, y, z), CartesianStateVariable::ANGULAR_VELOCITY);
}

void CartesianState::set_twist(const Eigen::Matrix<double, 6, 1>& twist) {
  this->set_state_variable(twist, CartesianStateVariable::TWIST);
}

void CartesianState::set_twist(const std::vector<double>& twist) {
  this->set_state_variable(twist, CartesianStateVariable::TWIST);
}

void CartesianState::set_linear_acceleration(const Eigen::Vector3d& linear_acceleration) {
  this->set_state_variable(linear_acceleration, CartesianStateVariable::LINEAR_ACCELERATION);
}

void CartesianState::set_linear_acceleration(const std::vector<double>& linear_acceleration) {
  this->set_state_variable(linear_acceleration, CartesianStateVariable::LINEAR_ACCELERATION);
}

void CartesianState::set_linear_acceleration(const double& x, const double& y, const double& z) {
  this->set_state_variable(Eigen::Vector3d(x, y, z), CartesianStateVariable::LINEAR_ACCELERATION);
}

void CartesianState::set_angular_acceleration(const Eigen::Vector3d& angular_acceleration) {
  this->set_state_variable(angular_acceleration, CartesianStateVariable::ANGULAR_ACCELERATION);
}

void CartesianState::set_angular_acceleration(const std::vector<double>& angular_acceleration) {
  this->set_state_variable(angular_acceleration, CartesianStateVariable::ANGULAR_ACCELERATION);
}

void CartesianState::set_angular_acceleration(const double& x, const double& y, const double& z) {
  this->set_state_variable(Eigen::Vector3d(x, y, z), CartesianStateVariable::ANGULAR_ACCELERATION);
}

void CartesianState::set_acceleration(const Eigen::Matrix<double, 6, 1>& acceleration) {
  this->set_state_variable(acceleration, CartesianStateVariable::ACCELERATION);
}

void CartesianState::set_acceleration(const std::vector<double>& acceleration) {
  this->set_state_variable(acceleration, CartesianStateVariable::ACCELERATION);
}

void CartesianState::set_force(const Eigen::Vector3d& force) {
  this->set_state_variable(force, CartesianStateVariable::FORCE);
}

void CartesianState::set_force(const std::vector<double>& force) {
  this->set_state_variable(force, CartesianStateVariable::FORCE);
}

void CartesianState::set_force(const double& x, const double& y, const double& z) {
  this->set_state_variable(Eigen::Vector3d(x, y, z), CartesianStateVariable::FORCE);
}

void CartesianState::set_torque(const Eigen::Vector3d& torque) {
  this->set_state_variable(torque, CartesianStateVariable::TORQUE);
}

void CartesianState::set_torque(const std::vector<double>& torque) {
  this->set_state_variable(torque, CartesianStateVariable::TORQUE);
}

void CartesianState::set_torque(const double& x, const double& y, const double& z) {
  this->set_state_variable(Eigen::Vector3d(x, y, z), CartesianStateVariable::TORQUE);
}

void CartesianState::set_wrench(const Eigen::Matrix<double, 6, 1>& wrench) {
  this->set_state_variable(wrench, CartesianStateVariable::WRENCH);
}

void CartesianState::set_wrench(const std::vector<double>& wrench) {
  this->set_state_variable(wrench, CartesianStateVariable::WRENCH);
}

void CartesianState::set_data(const Eigen::VectorXd& data) {
  this->set_state_variable(data, CartesianStateVariable::ALL);
}

void CartesianState::set_data(const std::vector<double>& data) {
  this->set_state_variable(data, CartesianStateVariable::ALL);
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
  this->set_empty(false);
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
    result += this->get_orientation().angularDistance(state.get_orientation());
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

void CartesianState::reset() {
  this->set_zero();
  this->State::reset();
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

  // the inverse wrench is not supported by this operation
  inverse.set_wrench(Eigen::Vector<double, 6>::Zero());

  return inverse;
}

void CartesianState::normalize(const CartesianStateVariable& state_variable_type) {
  this->assert_not_empty();
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
  this->reset_timestamp();
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
  Eigen::Quaterniond b_R_c = state.get_orientation();
  Eigen::Vector3d b_v_c = state.get_linear_velocity();
  Eigen::Vector3d b_omega_c = state.get_angular_velocity();
  Eigen::Vector3d b_a_c = state.get_linear_acceleration();
  Eigen::Vector3d b_alpha_c = state.get_angular_acceleration();
  Eigen::Vector3d b_F_c = state.get_force();
  Eigen::Vector3d b_tau_c = state.get_torque();
  // pose
  this->set_position(f_P_b + f_R_b * b_P_c);
  auto orientation = f_R_b * b_R_c;

  // specific operation on quaternion using Hamilton product, keeping the resulting quaternion on the same hemisphere
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

  // keep only the wrench measured at the distal frame, aligned with the new reference frame
  this->set_force(f_R_b * b_F_c);
  this->set_torque(f_R_b * b_tau_c);

  return (*this);
}

CartesianState CartesianState::operator*(const CartesianState& state) const {
  CartesianState result(*this);
  result *= state;
  return result;
}

CartesianState& CartesianState::operator*=(double lambda) {
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

std::ostream& operator<<(std::ostream& os, const Eigen::Vector3d& field) {
  os << "(" << field(0) << ", " << field(1) << ", " << field(2) << ")";
  return os;
}

std::string CartesianState::to_string() const {
  std::stringstream s;
  s << this->SpatialState::to_string();
  if (this->is_empty()) {
    return s.str();
  }
  if (this->get_type() == StateType::CARTESIAN_POSE || this->get_type() == StateType::CARTESIAN_STATE) {
    s << std::endl << "position: " << this->get_position() << std::endl;
    s << "orientation: (" << this->get_orientation().w() << ", ";
    s << this->get_orientation().x() << ", ";
    s << this->get_orientation().y() << ", ";
    s << this->get_orientation().z() << ")";
    Eigen::AngleAxisd axis_angle(this->get_orientation());
    s << " <=> theta: " << axis_angle.angle() << ", ";
    s << "axis: " << axis_angle.axis();
  }
  if (this->get_type() == StateType::CARTESIAN_TWIST || this->get_type() == StateType::CARTESIAN_STATE) {
    s << std::endl << "linear velocity: " << this->get_linear_velocity() << std::endl;
    s << "angular velocity: " << this->get_angular_velocity();
  }
  if (this->get_type() == StateType::CARTESIAN_ACCELERATION || this->get_type() == StateType::CARTESIAN_STATE) {
    s << std::endl << "linear acceleration: " << this->get_linear_acceleration() << std::endl;
    s << "angular acceleration: " << this->get_angular_acceleration();
  }
  if (this->get_type() == StateType::CARTESIAN_WRENCH || this->get_type() == StateType::CARTESIAN_STATE) {
    s << std::endl << "force: " << this->get_force() << std::endl;
    s << "torque: " << this->get_torque();
  }
  return s.str();
}

std::ostream& operator<<(std::ostream& os, const CartesianState& state) {
  os << state.to_string();
  return os;
}
}// namespace state_representation
