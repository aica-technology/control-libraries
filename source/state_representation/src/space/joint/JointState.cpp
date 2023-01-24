#include "state_representation/space/joint/JointState.hpp"

#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/IncompatibleStatesException.hpp"
#include "state_representation/exceptions/InvalidCastException.hpp"
#include "state_representation/exceptions/JointNotFoundException.hpp"

namespace state_representation {

using namespace exceptions;

static void assert_index_in_range(unsigned int joint_index, unsigned int size) {
  if (joint_index > size) {
    throw JointNotFoundException(
        "Index '" + std::to_string(joint_index) + "' is out of range for joint state with size" + std::to_string(size));
  }
}

JointState::JointState() : State(StateType::JOINT_STATE) {}

JointState::JointState(const std::string& robot_name, unsigned int nb_joints) :
    State(StateType::JOINT_STATE, robot_name), names_(nb_joints) {
  this->set_names(nb_joints);
  this->initialize();
}

JointState::JointState(const std::string& robot_name, const std::vector<std::string>& joint_names) :
    State(StateType::JOINT_STATE, robot_name), names_(joint_names) {
  this->initialize();
}

JointState::JointState(const StateType& type, const std::string& robot_name, unsigned int nb_joints) :
    State(type, robot_name), names_(nb_joints) {
  this->set_names(nb_joints);
  // FIXME: this initializes all field, but subclasses might want to only initialize one field
  this->initialize();
}

JointState::JointState(
    const StateType& type, const std::string& robot_name, const std::vector<std::string>& joint_names
) : State(type, robot_name), names_(joint_names) {
  // FIXME: this initializes all field, but subclasses might want to only initialize one field
  this->initialize();
}

JointState JointState::Zero(const std::string& robot_name, unsigned int nb_joints) {
  JointState zero = JointState(robot_name, nb_joints);
  // as opposed to the constructor specify this state to be filled
  zero.set_empty(false);
  return zero;
}

JointState JointState::Zero(const std::string& robot_name, const std::vector<std::string>& joint_names) {
  JointState zero = JointState(robot_name, joint_names);
  // as opposed to the constructor specify this state to be filled
  zero.set_empty(false);
  return zero;
}

JointState JointState::Random(const std::string& robot_name, unsigned int nb_joints) {
  JointState random = JointState(robot_name, nb_joints);
  // set all the state variables to random
  random.set_state_variable(Eigen::VectorXd::Random(random.get_size() * 4), JointStateVariable::ALL);
  return random;
}

JointState JointState::Random(const std::string& robot_name, const std::vector<std::string>& joint_names) {
  JointState random = JointState(robot_name, joint_names);
  // set all the state variables to random
  random.set_state_variable(Eigen::VectorXd::Random(random.get_size() * 4), JointStateVariable::ALL);
  return random;
}

JointState& JointState::operator=(const JointState& state) {
  JointState tmp(state);
  swap(*this, tmp);
  return *this;
}

Eigen::VectorXd JointState::get_all_state_variables() const {
  Eigen::VectorXd all_fields(this->get_size() * 4);
  all_fields << this->get_positions(), this->get_velocities(), this->get_accelerations(), this->get_torques();
  return all_fields;
}

Eigen::VectorXd JointState::get_state_variable(const JointStateVariable& state_variable_type) const {
  switch (state_variable_type) {
    case JointStateVariable::POSITIONS:
      return this->get_positions();

    case JointStateVariable::VELOCITIES:
      return this->get_velocities();

    case JointStateVariable::ACCELERATIONS:
      return this->get_accelerations();

    case JointStateVariable::TORQUES:
      return this->get_torques();

    case JointStateVariable::ALL:
      return this->get_all_state_variables();
  }
  // this never goes here but is compulsory to avoid a warning
  return Eigen::Vector3d::Zero();
}

unsigned int JointState::get_size() const {
  return this->names_.size();
}

const std::vector<std::string>& JointState::get_names() const {
  return this->names_;
}

unsigned int JointState::get_joint_index(const std::string& joint_name) const {
  auto finder = std::find(this->names_.begin(), this->names_.end(), joint_name);
  if (finder == this->names_.end()) {
    throw JointNotFoundException("The joint with name '" + joint_name + "' could not be found in the joint state.");
  }
  return std::distance(this->names_.begin(), finder);
}

const Eigen::VectorXd& JointState::get_positions() const {
  return this->positions_;
}

double JointState::get_position(const std::string& joint_name) const {
  return this->positions_(this->get_joint_index(joint_name));
}

double JointState::get_position(unsigned int joint_index) const {
  assert_index_in_range(joint_index, this->get_size());
  return this->positions_(joint_index);
}

const Eigen::VectorXd& JointState::get_velocities() const {
  return this->velocities_;
}

double JointState::get_velocity(const std::string& joint_name) const {
  return this->velocities_(this->get_joint_index(joint_name));
}

double JointState::get_velocity(unsigned int joint_index) const {
  assert_index_in_range(joint_index, this->get_size());
  return this->velocities_(joint_index);
}

const Eigen::VectorXd& JointState::get_accelerations() const {
  return this->accelerations_;
}

double JointState::get_acceleration(const std::string& joint_name) const {
  return this->accelerations_(this->get_joint_index(joint_name));
}

double JointState::get_acceleration(unsigned int joint_index) const {
  assert_index_in_range(joint_index, this->get_size());
  return this->accelerations_(joint_index);
}

const Eigen::VectorXd& JointState::get_torques() const {
  return this->torques_;
}

double JointState::get_torque(const std::string& joint_name) const {
  return this->torques_(this->get_joint_index(joint_name));
}

double JointState::get_torque(unsigned int joint_index) const {
  assert_index_in_range(joint_index, this->get_size());
  return this->torques_(joint_index);
}

Eigen::VectorXd JointState::data() const {
  return this->get_all_state_variables();
}

Eigen::ArrayXd JointState::array() const {
  return this->data().array();
}

void JointState::set_names(unsigned int nb_joints) {
  if (this->get_size() != nb_joints) {
    throw state_representation::exceptions::IncompatibleSizeException(
        "Input number of joints is of incorrect size, expected " + std::to_string(this->get_size()) + " got "
            + std::to_string(nb_joints));
  }
  this->names_.resize(nb_joints);
  for (unsigned int i = 0; i < nb_joints; ++i) {
    this->names_[i] = "joint" + std::to_string(i);
  }
}

void JointState::set_state_variable(Eigen::VectorXd& state_variable, const Eigen::VectorXd& new_value) {
  if (new_value.size() != this->get_size()) {
    throw state_representation::exceptions::IncompatibleSizeException(
        "Input vector is of incorrect size: expected " + std::to_string(this->get_size()) + ", given "
            + std::to_string(new_value.size()));
  }
  this->set_empty(false);
  state_variable = new_value;
}

void JointState::set_state_variable(Eigen::VectorXd& state_variable, const std::vector<double>& new_value) {
  this->set_state_variable(state_variable, Eigen::VectorXd::Map(new_value.data(), new_value.size()));
}

void JointState::set_all_state_variables(const Eigen::VectorXd& new_values) {
  if (new_values.size() != 4 * this->get_size()) {
    throw state_representation::exceptions::IncompatibleSizeException(
        "Input is of incorrect size: expected " + std::to_string(this->get_size()) + ", given "
            + std::to_string(new_values.size()));
  }
  this->set_positions(new_values.segment(0, this->get_size()));
  this->set_velocities(new_values.segment(this->get_size(), this->get_size()));
  this->set_accelerations(new_values.segment(2 * this->get_size(), this->get_size()));
  this->set_torques(new_values.segment(3 * this->get_size(), this->get_size()));
}

void JointState::set_state_variable(
    const Eigen::VectorXd& new_value, const JointStateVariable& state_variable_type
) {
  switch (state_variable_type) {
    case JointStateVariable::POSITIONS:
      this->set_positions(new_value);
      break;

    case JointStateVariable::VELOCITIES:
      this->set_velocities(new_value);
      break;

    case JointStateVariable::ACCELERATIONS:
      this->set_accelerations(new_value);
      break;

    case JointStateVariable::TORQUES:
      this->set_torques(new_value);
      break;

    case JointStateVariable::ALL:
      this->set_all_state_variables(new_value);
      break;
  }
}

void JointState::set_names(const std::vector<std::string>& names) {
  if (this->get_size() != names.size()) {
    throw state_representation::exceptions::IncompatibleSizeException(
        "Input number of joints is of incorrect size, expected " + std::to_string(this->get_size()) + " got "
            + std::to_string(names.size()));
  }
  this->names_ = names;
}

void JointState::set_positions(const Eigen::VectorXd& positions) {
  this->set_state_variable(this->positions_, positions);
}

void JointState::set_positions(const std::vector<double>& positions) {
  this->set_state_variable(this->positions_, positions);
}

void JointState::set_position(double position, const std::string& joint_name) {
  this->set_empty(false);
  this->positions_(this->get_joint_index(joint_name)) = position;
}

void JointState::set_position(double position, unsigned int joint_index) {
  assert_index_in_range(joint_index, this->get_size());
  this->set_empty(false);
  this->positions_(joint_index) = position;
}

void JointState::set_velocities(const Eigen::VectorXd& velocities) {
  this->set_state_variable(this->velocities_, velocities);
}

void JointState::set_velocities(const std::vector<double>& velocities) {
  this->set_state_variable(this->velocities_, velocities);
}

void JointState::set_velocity(double velocity, const std::string& joint_name) {
  this->set_empty(false);
  this->velocities_(this->get_joint_index(joint_name)) = velocity;
}

void JointState::set_velocity(double velocity, unsigned int joint_index) {
  assert_index_in_range(joint_index, this->get_size());
  this->set_empty(false);
  this->velocities_(joint_index) = velocity;
}

void JointState::set_accelerations(const Eigen::VectorXd& accelerations) {
  this->set_state_variable(this->accelerations_, accelerations);
}

void JointState::set_accelerations(const std::vector<double>& accelerations) {
  this->set_state_variable(this->accelerations_, accelerations);
}

void JointState::set_acceleration(double acceleration, const std::string& joint_name) {
  this->set_empty(false);
  this->accelerations_(this->get_joint_index(joint_name)) = acceleration;
}

void JointState::set_acceleration(double acceleration, unsigned int joint_index) {
  assert_index_in_range(joint_index, this->get_size());
  this->set_empty(false);
  this->accelerations_(joint_index) = acceleration;
}

void JointState::set_torques(const Eigen::VectorXd& torques) {
  this->set_state_variable(this->torques_, torques);
}

void JointState::set_torques(const std::vector<double>& torques) {
  this->set_state_variable(this->torques_, torques);
}

void JointState::set_torque(double torque, const std::string& joint_name) {
  this->set_empty(false);
  this->torques_(this->get_joint_index(joint_name)) = torque;
}

void JointState::set_torque(double torque, unsigned int joint_index) {
  assert_index_in_range(joint_index, this->get_size());
  this->set_empty(false);
  this->torques_(joint_index) = torque;
}

void JointState::set_data(const Eigen::VectorXd& data) {
  this->set_all_state_variables(data);
}

void JointState::set_data(const std::vector<double>& data) {
  this->set_all_state_variables(Eigen::VectorXd::Map(data.data(), data.size()));
}

void JointState::clamp_state_variable(
    const Eigen::ArrayXd& max_absolute_value_array, const JointStateVariable& state_variable_type,
    const Eigen::ArrayXd& noise_ratio_array
) {
  Eigen::VectorXd state_variable = this->get_state_variable(state_variable_type);
  int expected_size = state_variable.size();
  if (max_absolute_value_array.size() != expected_size) {
    throw IncompatibleSizeException(
        "Array of max values is of incorrect size: expected " + std::to_string(expected_size) + ", given "
            + std::to_string(max_absolute_value_array.size()));
  }

  if (noise_ratio_array.size() != expected_size) {
    throw IncompatibleSizeException(
        "Array of max values is of incorrect size: expected " + std::to_string(expected_size) + ", given "
            + std::to_string(noise_ratio_array.size()));
  }
  for (int i = 0; i < expected_size; ++i) {
    if (noise_ratio_array(i) != 0.0 && abs(state_variable(i)) < noise_ratio_array(i) * max_absolute_value_array(i)) {
      // apply dead zone
      state_variable(i) = 0.0;
    } else if (abs(state_variable(i)) > max_absolute_value_array(i)) {
      // clamp to max value
      state_variable(i) *= max_absolute_value_array(i) / abs(state_variable(i));
    }
  }
  this->set_state_variable(state_variable, state_variable_type);
}

void JointState::clamp_state_variable(
    double max_absolute_value, const JointStateVariable& state_variable_type, double noise_ratio
) {
  Eigen::VectorXd state_variable = this->get_state_variable(state_variable_type);
  int expected_size = state_variable.size();
  this->clamp_state_variable(
      max_absolute_value * Eigen::ArrayXd::Ones(expected_size), state_variable_type,
      noise_ratio * Eigen::ArrayXd::Ones(expected_size));
}

JointState JointState::copy() const {
  JointState result(*this);
  return result;
}

double JointState::dist(const JointState& state, const JointStateVariable& state_variable_type) const {
  // sanity check
  if (this->is_empty()) { throw EmptyStateException(this->get_name() + " state is empty"); }
  if (state.is_empty()) { throw EmptyStateException(state.get_name() + " state is empty"); }
  if (this->is_incompatible(state)) {
    throw IncompatibleStatesException(
        "The two joint states are incompatible, check name, joint names and order or size"
    );
  }
  // calculation
  double result = 0;
  if (state_variable_type == JointStateVariable::POSITIONS || state_variable_type == JointStateVariable::ALL) {
    result += (this->get_positions() - state.get_positions()).norm();
  }
  if (state_variable_type == JointStateVariable::VELOCITIES || state_variable_type == JointStateVariable::ALL) {
    result += (this->get_velocities() - state.get_velocities()).norm();
  }
  if (state_variable_type == JointStateVariable::ACCELERATIONS || state_variable_type == JointStateVariable::ALL) {
    result += (this->get_accelerations() - state.get_accelerations()).norm();
  }
  if (state_variable_type == JointStateVariable::TORQUES || state_variable_type == JointStateVariable::ALL) {
    result += (this->get_torques() - state.get_torques()).norm();
  }
  return result;
}

double dist(const JointState& s1, const JointState& s2, const JointStateVariable& state_variable_type) {
  return s1.dist(s2, state_variable_type);
}

void JointState::initialize() {
  this->State::initialize();
  // resize
  unsigned int size = this->names_.size();
  this->positions_.resize(size);
  this->velocities_.resize(size);
  this->accelerations_.resize(size);
  this->torques_.resize(size);
  // set to zeros
  this->set_zero();
}

bool JointState::is_incompatible(const State& state) const {
  try {
    auto other = dynamic_cast<const JointState&>(state);
    if (this->names_.size() != other.names_.size()) {
      return true;
    }
    for (unsigned int i = 0; i < this->names_.size(); ++i) {
      if (this->names_[i] != other.names_[i]) {
        return true;
      }
    }
    return false;
  } catch (const std::bad_cast& ex) {
    throw exceptions::InvalidCastException(
        std::string("Could not cast the given object to a JointState: ") + ex.what());
  }
}

void JointState::set_zero() {
  this->positions_.setZero();
  this->velocities_.setZero();
  this->accelerations_.setZero();
  this->torques_.setZero();
}

std::vector<double> JointState::to_std_vector() const {
  Eigen::VectorXd data = this->data();
  return std::vector<double>(data.data(), data.data() + data.size());
}

void JointState::multiply_state_variable(const Eigen::ArrayXd& lambda, const JointStateVariable& state_variable_type) {
  Eigen::VectorXd state_variable = this->get_state_variable(state_variable_type);
  int expected_size = state_variable.size();
  if (lambda.size() != expected_size) {
    throw IncompatibleSizeException(
        "Gain matrix is of incorrect size: expected " + std::to_string(expected_size) + ", given "
            + std::to_string(lambda.size()));
  }
  this->set_state_variable((lambda * state_variable.array()).matrix(), state_variable_type);
}

void JointState::multiply_state_variable(const Eigen::MatrixXd& lambda, const JointStateVariable& state_variable_type) {
  Eigen::VectorXd state_variable = this->get_state_variable(state_variable_type);
  int expected_size = state_variable.size();
  if (lambda.rows() != expected_size || lambda.cols() != expected_size) {
    throw IncompatibleSizeException(
        "Gain matrix is of incorrect size: expected " + std::to_string(expected_size) + "x"
            + std::to_string(expected_size) + ", given " + std::to_string(lambda.rows()) + "x"
            + std::to_string(lambda.cols()));
  }
  this->set_state_variable(lambda * this->get_state_variable(state_variable_type), state_variable_type);
}

JointState& JointState::operator*=(double lambda) {
  if (this->is_empty()) {
    throw EmptyStateException(this->get_name() + " state is empty");
  }
  this->set_all_state_variables(lambda * this->get_all_state_variables());
  return (*this);
}

JointState JointState::operator*(double lambda) const {
  JointState result(*this);
  result *= lambda;
  return result;
}

JointState operator*(double lambda, const JointState& state) {
  if (state.is_empty()) { throw EmptyStateException(state.get_name() + " state is empty"); }
  JointState result(state);
  result *= lambda;
  return result;
}

JointState& JointState::operator*=(const Eigen::MatrixXd& lambda) {
  this->multiply_state_variable(lambda, JointStateVariable::ALL);
  return (*this);
}

JointState JointState::operator*(const Eigen::MatrixXd& lambda) const {
  JointState result(*this);
  result *= lambda;
  return result;
}

JointState operator*(const Eigen::MatrixXd& lambda, const JointState& state) {
  JointState result(state);
  result *= lambda;
  return result;
}

JointState& JointState::operator*=(const Eigen::ArrayXd& lambda) {
  this->multiply_state_variable(lambda, JointStateVariable::ALL);
  return (*this);
}

JointState JointState::operator*(const Eigen::ArrayXd& lambda) const {
  JointState result(*this);
  result *= lambda;
  return result;
}

JointState operator*(const Eigen::ArrayXd& lambda, const JointState& state) {
  JointState result(state);
  result *= lambda;
  return result;
}

JointState& JointState::operator/=(double lambda) {
  return JointState::operator*=(1 / lambda);
}

JointState JointState::operator/(double lambda) const {
  JointState result(*this);
  result /= lambda;
  return result;
}

JointState& JointState::operator+=(const JointState& state) {
  // sanity check
  if (this->is_empty()) {
    throw EmptyStateException(this->get_name() + " state is empty");
  }
  if (state.is_empty()) {
    throw EmptyStateException(state.get_name() + " state is empty");
  }
  if (this->is_incompatible(state)) {
    throw IncompatibleStatesException(
        "The two joint states are incompatible, check name, joint names and order or size"
    );
  }
  this->set_all_state_variables(this->get_all_state_variables() + state.get_all_state_variables());
  return (*this);
}

JointState JointState::operator+(const JointState& state) const {
  JointState result(*this);
  result += state;
  return result;
}

JointState& JointState::operator-=(const JointState& state) {
  // sanity check
  if (this->is_empty()) {
    throw EmptyStateException(this->get_name() + " state is empty");
  }
  if (state.is_empty()) {
    throw EmptyStateException(state.get_name() + " state is empty");
  }
  if (this->is_incompatible(state)) {
    throw IncompatibleStatesException(
        "The two joint states are incompatible, check name, joint names and order or size"
    );
  }
  this->set_all_state_variables(this->get_all_state_variables() - state.get_all_state_variables());
  return (*this);
}

JointState JointState::operator-(const JointState& state) const {
  JointState result(*this);
  result -= state;
  return result;
}

std::ostream& operator<<(std::ostream& os, const JointState& state) {
  if (state.is_empty()) {
    os << "Empty " << state.get_name() << " JointState";
  } else {
    os << state.get_name() << " JointState" << std::endl;
    os << "names: [";
    for (auto& n: state.names_) { os << n << ", "; }
    os << "]" << std::endl;
    os << "positions: [";
    for (unsigned int i = 0; i < state.positions_.size(); ++i) { os << state.positions_(i) << ", "; }
    os << "]" << std::endl;
    os << "velocities: [";
    for (unsigned int i = 0; i < state.velocities_.size(); ++i) { os << state.velocities_(i) << ", "; }
    os << "]" << std::endl;
    os << "accelerations: [";
    for (unsigned int i = 0; i < state.accelerations_.size(); ++i) { os << state.accelerations_(i) << ", "; }
    os << "]" << std::endl;
    os << "torques: [";
    for (unsigned int i = 0; i < state.torques_.size(); ++i) { os << state.torques_(i) << ", "; }
    os << "]";
  }
  return os;
}
}// namespace state_representation
