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

static unsigned int get_state_variable_size_factor(const JointStateVariable& state_variable_type) {
  switch (state_variable_type) {
    case JointStateVariable::POSITIONS:
    case JointStateVariable::VELOCITIES:
    case JointStateVariable::ACCELERATIONS:
    case JointStateVariable::TORQUES:
      return 1;
    case JointStateVariable::ALL:
      return 4;
    default:
      return 0;
  }
}

JointState::JointState() : State() {
  this->set_type(StateType::JOINT_STATE);
}

JointState::JointState(const std::string& robot_name, unsigned int nb_joints) :
    State(robot_name),
    names_(nb_joints),
    positions_(Eigen::VectorXd::Zero(nb_joints)),
    velocities_(Eigen::VectorXd::Zero(nb_joints)),
    accelerations_(Eigen::VectorXd::Zero(nb_joints)),
    torques_(Eigen::VectorXd::Zero(nb_joints)) {
  this->set_type(StateType::JOINT_STATE);
  this->set_names(nb_joints);
}

JointState::JointState(const std::string& robot_name, const std::vector<std::string>& joint_names) :
    JointState(robot_name, joint_names.size()) {
  this->set_names(joint_names);
}

JointState::JointState(const JointState& state) : JointState(state.get_name(), state.names_) {
  if (state) {
    this->set_state_variable(state.get_state_variable(JointStateVariable::ALL), JointStateVariable::ALL);
  }
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

Eigen::VectorXd JointState::get_state_variable(const JointStateVariable& state_variable_type) const {
  this->assert_not_empty();
  switch (state_variable_type) {
    case JointStateVariable::POSITIONS:
      return this->positions_;
    case JointStateVariable::VELOCITIES:
      return this->velocities_;
    case JointStateVariable::ACCELERATIONS:
      return this->accelerations_;
    case JointStateVariable::TORQUES:
      return this->torques_;
    case JointStateVariable::ALL: {
      Eigen::VectorXd all_fields(this->get_size() * 4);
      all_fields << this->positions_, this->velocities_, this->accelerations_, this->torques_;
      return all_fields;
    }
    default:
      return {};
  }
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
  this->assert_not_empty();
  return this->positions_;
}

double JointState::get_position(const std::string& joint_name) const {
  return this->positions_(this->get_joint_index(joint_name));
}

double JointState::get_position(unsigned int joint_index) const {
  this->assert_not_empty();
  assert_index_in_range(joint_index, this->get_size());
  return this->positions_(joint_index);
}

const Eigen::VectorXd& JointState::get_velocities() const {
  this->assert_not_empty();
  return this->velocities_;
}

double JointState::get_velocity(const std::string& joint_name) const {
  return this->velocities_(this->get_joint_index(joint_name));
}

double JointState::get_velocity(unsigned int joint_index) const {
  this->assert_not_empty();
  assert_index_in_range(joint_index, this->get_size());
  return this->velocities_(joint_index);
}

const Eigen::VectorXd& JointState::get_accelerations() const {
  this->assert_not_empty();
  return this->accelerations_;
}

double JointState::get_acceleration(const std::string& joint_name) const {
  return this->accelerations_(this->get_joint_index(joint_name));
}

double JointState::get_acceleration(unsigned int joint_index) const {
  this->assert_not_empty();
  assert_index_in_range(joint_index, this->get_size());
  return this->accelerations_(joint_index);
}

const Eigen::VectorXd& JointState::get_torques() const {
  this->assert_not_empty();
  return this->torques_;
}

double JointState::get_torque(const std::string& joint_name) const {
  return this->torques_(this->get_joint_index(joint_name));
}

double JointState::get_torque(unsigned int joint_index) const {
  this->assert_not_empty();
  assert_index_in_range(joint_index, this->get_size());
  return this->torques_(joint_index);
}

Eigen::VectorXd JointState::data() const {
  return this->get_state_variable(JointStateVariable::ALL);
}

Eigen::ArrayXd JointState::array() const {
  return this->data().array();
}

void JointState::set_state_variable(
    const std::vector<double>& new_value, const JointStateVariable& state_variable_type
) {
  this->set_state_variable(Eigen::VectorXd::Map(new_value.data(), new_value.size()), state_variable_type);
}

void JointState::set_state_variable(const Eigen::VectorXd& new_value, const JointStateVariable& state_variable_type) {
  auto expected_size = get_state_variable_size_factor(state_variable_type) * this->get_size();
  if (new_value.size() != expected_size) {
    throw exceptions::IncompatibleSizeException(
        "Input is of incorrect size, expected " + std::to_string(expected_size) + ", got "
            + std::to_string(new_value.size()));
  }
  switch (state_variable_type) {
    case JointStateVariable::POSITIONS:
      this->positions_ = new_value;
      break;
    case JointStateVariable::VELOCITIES:
      this->velocities_ = new_value;
      break;
    case JointStateVariable::ACCELERATIONS:
      this->accelerations_ = new_value;
      break;
    case JointStateVariable::TORQUES:
      this->torques_ = new_value;
      break;
    case JointStateVariable::ALL: {
      auto size = this->get_size();
      this->positions_ = new_value.head(size);
      this->velocities_ = new_value.segment(size, size);
      this->accelerations_ = new_value.segment(2 * size, size);
      this->torques_ = new_value.tail(size);
      break;
    }
  }
  this->set_empty(false);
  this->reset_timestamp();
}

void JointState::set_state_variable(
    double new_value, unsigned int joint_index, const JointStateVariable& state_variable_type
) {
  assert_index_in_range(joint_index, this->get_size());
  switch (state_variable_type) {
    case JointStateVariable::POSITIONS:
      this->positions_(joint_index) = new_value;
      break;
    case JointStateVariable::VELOCITIES:
      this->velocities_(joint_index) = new_value;
      break;
    case JointStateVariable::ACCELERATIONS:
      this->accelerations_(joint_index) = new_value;
      break;
    case JointStateVariable::TORQUES:
      this->torques_(joint_index) = new_value;
      break;
    case JointStateVariable::ALL:
      this->positions_(joint_index) = new_value;
      this->velocities_(joint_index) = new_value;
      this->accelerations_(joint_index) = new_value;
      this->torques_(joint_index) = new_value;
  }
  this->set_empty(false);
  this->reset_timestamp();
}

void JointState::set_names(unsigned int nb_joints) {
  if (this->get_size() != nb_joints) {
    throw state_representation::exceptions::IncompatibleSizeException(
        "Input number of joints is of incorrect size, expected " + std::to_string(this->get_size()) + " got "
            + std::to_string(nb_joints));
  }
  for (unsigned int i = 0; i < nb_joints; ++i) {
    this->names_[i] = "joint" + std::to_string(i);
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
  this->set_state_variable(positions, JointStateVariable::POSITIONS);
}

void JointState::set_positions(const std::vector<double>& positions) {
  this->set_state_variable(positions, JointStateVariable::POSITIONS);
}

void JointState::set_position(double position, const std::string& joint_name) {
  this->set_state_variable(position, this->get_joint_index(joint_name), JointStateVariable::POSITIONS);
}

void JointState::set_position(double position, unsigned int joint_index) {
  this->set_state_variable(position, joint_index, JointStateVariable::POSITIONS);
}

void JointState::set_velocities(const Eigen::VectorXd& velocities) {
  this->set_state_variable(velocities, JointStateVariable::VELOCITIES);
}

void JointState::set_velocities(const std::vector<double>& velocities) {
  this->set_state_variable(velocities, JointStateVariable::VELOCITIES);
}

void JointState::set_velocity(double velocity, const std::string& joint_name) {
  this->set_state_variable(velocity, this->get_joint_index(joint_name), JointStateVariable::VELOCITIES);
}

void JointState::set_velocity(double velocity, unsigned int joint_index) {
  this->set_state_variable(velocity, joint_index, JointStateVariable::VELOCITIES);
}

void JointState::set_accelerations(const Eigen::VectorXd& accelerations) {
  this->set_state_variable(accelerations, JointStateVariable::ACCELERATIONS);
}

void JointState::set_accelerations(const std::vector<double>& accelerations) {
  this->set_state_variable(accelerations, JointStateVariable::ACCELERATIONS);
}

void JointState::set_acceleration(double acceleration, const std::string& joint_name) {
  this->set_state_variable(acceleration, this->get_joint_index(joint_name), JointStateVariable::ACCELERATIONS);
}

void JointState::set_acceleration(double acceleration, unsigned int joint_index) {
  this->set_state_variable(acceleration, joint_index, JointStateVariable::ACCELERATIONS);
}

void JointState::set_torques(const Eigen::VectorXd& torques) {
  this->set_state_variable(torques, JointStateVariable::TORQUES);
}

void JointState::set_torques(const std::vector<double>& torques) {
  this->set_state_variable(torques, JointStateVariable::TORQUES);
}

void JointState::set_torque(double torque, const std::string& joint_name) {
  this->set_state_variable(torque, this->get_joint_index(joint_name), JointStateVariable::TORQUES);
}

void JointState::set_torque(double torque, unsigned int joint_index) {
  this->set_state_variable(torque, joint_index, JointStateVariable::TORQUES);
}

void JointState::set_data(const Eigen::VectorXd& data) {
  this->set_state_variable(data, JointStateVariable::ALL);
}

void JointState::set_data(const std::vector<double>& data) {
  this->set_state_variable(data, JointStateVariable::ALL);
}

void JointState::clamp_state_variable(
    const Eigen::ArrayXd& max_absolute_value_array, const JointStateVariable& state_variable_type,
    const Eigen::ArrayXd& noise_ratio_array
) {
  Eigen::VectorXd state_variable = this->get_state_variable(state_variable_type);
  long expected_size = state_variable.size();
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
  long expected_size = state_variable.size();
  this->clamp_state_variable(
      max_absolute_value * Eigen::ArrayXd::Ones(expected_size), state_variable_type,
      noise_ratio * Eigen::ArrayXd::Ones(expected_size));
}

JointState JointState::copy() const {
  JointState result(*this);
  return result;
}

double JointState::dist(const JointState& state, const JointStateVariable& state_variable_type) const {
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

void JointState::reset() {
  this->set_zero();
  this->State::reset();
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
  if (this->get_size() > 0) {
    this->positions_.setZero();
    this->velocities_.setZero();
    this->accelerations_.setZero();
    this->torques_.setZero();
    this->reset_timestamp();
    this->set_empty(false);
  }
}

std::vector<double> JointState::to_std_vector() const {
  Eigen::VectorXd data = this->data();
  return {data.data(), data.data() + data.size()};
}

void JointState::multiply_state_variable(const Eigen::MatrixXd& lambda, const JointStateVariable& state_variable_type) {
  Eigen::VectorXd state_variable = this->get_state_variable(state_variable_type);
  long expected_size = state_variable.size();
  if (lambda.rows() != expected_size || lambda.cols() != expected_size) {
    throw IncompatibleSizeException(
        "Gain matrix is of incorrect size: expected " + std::to_string(expected_size) + "x"
            + std::to_string(expected_size) + ", given " + std::to_string(lambda.rows()) + "x"
            + std::to_string(lambda.cols()));
  }
  this->set_state_variable(lambda * this->get_state_variable(state_variable_type), state_variable_type);
}

JointState& JointState::operator*=(double lambda) {
  this->set_state_variable(lambda * this->get_state_variable(JointStateVariable::ALL), JointStateVariable::ALL);
  return (*this);
}

JointState JointState::operator*(double lambda) const {
  JointState result(*this);
  result *= lambda;
  return result;
}

JointState operator*(double lambda, const JointState& state) {
  JointState result(state);
  result *= lambda;
  return result;
}

JointState operator*(const Eigen::MatrixXd& lambda, const JointState& state) {
  JointState result(state);
  result.multiply_state_variable(lambda, JointStateVariable::ALL);
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
  if (this->is_incompatible(state)) {
    throw IncompatibleStatesException(
        "The two joint states are incompatible, check name, joint names and order or size"
    );
  }
  this->set_state_variable(
      this->get_state_variable(JointStateVariable::ALL) + state.get_state_variable(JointStateVariable::ALL),
      JointStateVariable::ALL
  );
  return (*this);
}

JointState JointState::operator+(const JointState& state) const {
  JointState result(*this);
  result += state;
  return result;
}

JointState JointState::operator-() const {
  // create a copy of the state
  JointState result(*this);
  result.set_state_variable(-result.get_state_variable(JointStateVariable::ALL), JointStateVariable::ALL);
  return result;
}

JointState& JointState::operator-=(const JointState& state) {
  (*this) += -state;
  return (*this);
}

JointState JointState::operator-(const JointState& state) const {
  JointState result(*this);
  result -= state;
  return result;
}

std::string JointState::to_string() const {
  std::stringstream s;
  s << this->State::to_string();
  s << std::endl << "joint names: [";
  for (auto& n : this->get_names()) { s << n << ", "; }
  s << "]";
  if (this->is_empty()) {
    return s.str();
  }
  if (this->get_type() == StateType::JOINT_POSITIONS || this->get_type() == StateType::JOINT_STATE) {
    s << std::endl << "positions: [";
    for (auto& p : this->get_positions()) { s << p << ", "; }
    s << "]";
  }
  if (this->get_type() == StateType::JOINT_VELOCITIES || this->get_type() == StateType::JOINT_STATE) {
    s << std::endl << "velocities: [";
    for (auto& v : this->get_velocities()) { s << v << ", "; }
    s << "]";
  }
  if (this->get_type() == StateType::JOINT_ACCELERATIONS || this->get_type() == StateType::JOINT_STATE) {
    s << std::endl << "accelerations: [";
    for (auto& a : this->get_accelerations()) { s << a << ", "; }
    s << "]";
  }
  if (this->get_type() == StateType::JOINT_TORQUES || this->get_type() == StateType::JOINT_STATE) {
    s << std::endl << "torques: [";
    for (auto& t : this->get_torques()) { s << t << ", "; }
    s << "]";
  }
  return s.str();
}

std::ostream& operator<<(std::ostream& os, const JointState& state) {
  os << state.to_string();
  return os;
}
}// namespace state_representation
