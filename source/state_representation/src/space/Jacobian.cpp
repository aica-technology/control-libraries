#include "state_representation/space/Jacobian.hpp"

#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/IncompatibleStatesException.hpp"
#include "state_representation/exceptions/InvalidCastException.hpp"

namespace state_representation {

using namespace exceptions;

static void assert_matrix_size(const Eigen::MatrixXd& matrix, unsigned int expected_rows, unsigned int expected_cols) {
  if (expected_rows != matrix.rows() || expected_cols != matrix.cols()) {
    throw exceptions::IncompatibleSizeException(
        "Matrix is of incorrect size, expected " + std::to_string(expected_rows) + "x" + std::to_string(expected_cols)
            + " got " + std::to_string(matrix.rows()) + "x" + std::to_string(matrix.cols()));
  }
}

Jacobian::Jacobian() : State() {
  this->set_type(StateType::JACOBIAN);
}

Jacobian::Jacobian(
    const std::string& robot_name, unsigned int nb_joints, const std::string& frame, const std::string& reference_frame
) :
    State(robot_name),
    joint_names_(nb_joints),
    frame_(frame),
    reference_frame_(reference_frame),
    data_(Eigen::MatrixXd::Zero(6, nb_joints)) {
  this->set_type(StateType::JACOBIAN);
  this->set_joint_names(nb_joints);
}

Jacobian::Jacobian(
    const std::string& robot_name, const std::vector<std::string>& joint_names, const std::string& frame,
    const std::string& reference_frame
) : Jacobian(robot_name, joint_names.size(), frame, reference_frame) {
  this->set_joint_names(joint_names);
}

Jacobian::Jacobian(
    const std::string& robot_name, const std::string& frame, const Eigen::MatrixXd& data,
    const std::string& reference_frame
) : Jacobian(robot_name, data.cols(), frame, reference_frame) {
  assert_matrix_size(data, 6, data.cols());
  this->data_ = data;
  this->set_empty(false);
}

Jacobian::Jacobian(
    const std::string& robot_name, const std::vector<std::string>& joint_names, const std::string& frame,
    const Eigen::MatrixXd& data, const std::string& reference_frame
) : Jacobian(robot_name, frame, data, reference_frame) {
  this->set_joint_names(joint_names);
}

Jacobian::Jacobian(const Jacobian& jacobian) :
    Jacobian(jacobian.get_name(), jacobian.joint_names_, jacobian.frame_, jacobian.reference_frame_) {
  if (jacobian) {
    this->data_ = jacobian.data_;
    this->set_empty(false);
  }
}

Jacobian Jacobian::Random(
    const std::string& robot_name, unsigned int nb_joints, const std::string& frame, const std::string& reference_frame
) {
  return {robot_name, frame, Eigen::MatrixXd::Random(6, nb_joints), reference_frame};
}

Jacobian Jacobian::Random(
    const std::string& robot_name, const std::vector<std::string>& joint_names, const std::string& frame,
    const std::string& reference_frame
) {
  return {robot_name, joint_names, frame, Eigen::MatrixXd::Random(6, joint_names.size()), reference_frame};
}

Jacobian& Jacobian::operator=(const Jacobian& jacobian) {
  Jacobian tmp(jacobian);
  swap(*this, tmp);
  return *this;
}

unsigned int Jacobian::rows() const {
  return 6;
}

Eigen::VectorXd Jacobian::row(unsigned int index) const {
  return this->data().row(index);
}

unsigned int Jacobian::cols() const {
  return this->joint_names_.size();
}

Eigen::VectorXd Jacobian::col(unsigned int index) const {
  return this->data().col(index);
}

const std::vector<std::string>& Jacobian::get_joint_names() const {
  return this->joint_names_;
}

const std::string& Jacobian::get_frame() const {
  return this->frame_;
}

const std::string& Jacobian::get_reference_frame() const {
  return this->reference_frame_;
}

const Eigen::MatrixXd& Jacobian::data() const {
  this->assert_not_empty();
  return this->data_;
}

void Jacobian::set_joint_names(unsigned int nb_joints) {
  if (this->joint_names_.size() != nb_joints) {
    throw exceptions::IncompatibleSizeException(
        "Input number of joints is of incorrect size, expected " + std::to_string(this->joint_names_.size()) + " got "
            + std::to_string(nb_joints));
  }
  for (unsigned int i = 0; i < nb_joints; ++i) {
    this->joint_names_[i] = "joint" + std::to_string(i);
  }
}

void Jacobian::set_joint_names(const std::vector<std::string>& joint_names) {
  if (this->joint_names_.size() != joint_names.size()) {
    throw exceptions::IncompatibleSizeException(
        "Input vector of joint names is of incorrect size, expected " + std::to_string(this->joint_names_.size())
            + " got " + std::to_string(joint_names.size()));
  }
  this->joint_names_ = joint_names;
}

void Jacobian::set_reference_frame(const std::string& reference_frame) {
  this->reference_frame_ = reference_frame;
}

void Jacobian::set_data(const Eigen::MatrixXd& data) {
  assert_matrix_size(data, this->rows(), this->cols());
  this->set_empty(false);
  this->data_ = data;
}

void Jacobian::set_zero() {
  this->data_.resize(this->rows(), this->cols());
  this->data_.setZero();
}

Jacobian Jacobian::copy() const {
  Jacobian result(*this);
  return result;
}

void Jacobian::reset() {
  this->set_zero();
  this->State::reset();
}

Eigen::MatrixXd Jacobian::inverse() const {
  if (this->rows() != this->cols()) {
    throw exceptions::IncompatibleSizeException(
        "The Jacobian matrix is not invertible, use the pseudoinverse function instead");
  }
  return this->data().inverse();
}

Eigen::MatrixXd Jacobian::inverse(const Eigen::MatrixXd& matrix) const {
  assert_matrix_size(matrix, this->rows(), matrix.cols());
  return this->data().colPivHouseholderQr().solve(matrix);
}

JointVelocities Jacobian::inverse(const CartesianTwist& twist) const {
  if (this->is_incompatible(twist)) {
    throw IncompatibleStatesException("The Jacobian and the given Cartesian twist are incompatible");
  }
  return JointVelocities(this->get_name(), this->get_joint_names(), this->inverse(twist.data()));
}

bool Jacobian::is_incompatible(const State& state) const {
  try {
    switch (state.get_type()) {
      case StateType::JACOBIAN: {
        auto other = dynamic_cast<const Jacobian&>(state);
        if (this->cols() != other.joint_names_.size()) {
          return true;
        }
        if (this->reference_frame_ != other.reference_frame_) {
          return true;
        }
        for (unsigned int i = 0; i < this->cols(); ++i) {
          if (this->joint_names_[i] != other.joint_names_[i]) {
            return true;
          }
        }
        return false;
      }
      case StateType::JOINT_STATE:
      case StateType::JOINT_POSITIONS:
      case StateType::JOINT_VELOCITIES:
      case StateType::JOINT_ACCELERATIONS:
      case StateType::JOINT_TORQUES: {
        auto other = dynamic_cast<const JointState&>(state);
        if (this->cols() != other.get_names().size()) {
          return true;
        }
        for (unsigned int i = 0; i < this->cols(); ++i) {
          if (this->joint_names_[i] != other.get_names()[i]) {
            return true;
          }
        }
        return false;
      }
      case StateType::CARTESIAN_STATE:
      case StateType::CARTESIAN_POSE:
      case StateType::CARTESIAN_TWIST:
      case StateType::CARTESIAN_ACCELERATION:
      case StateType::CARTESIAN_WRENCH: {
        auto other = dynamic_cast<const CartesianState&>(state);
        if (this->reference_frame_ != other.get_reference_frame()) {
          return true;
        }
        return false;
      }
      default:
        return true;
    }
  } catch (const std::bad_cast& ex) {
    throw exceptions::InvalidCastException(std::string("Could not cast the given object: ") + ex.what());
  }
}

Eigen::MatrixXd Jacobian::pseudoinverse() const {
  return this->data().completeOrthogonalDecomposition().pseudoInverse();
}

Eigen::MatrixXd Jacobian::pseudoinverse(const Eigen::MatrixXd& matrix) const {
  assert_matrix_size(matrix, this->rows(), matrix.cols());
  return this->pseudoinverse() * matrix;
}

JointVelocities Jacobian::pseudoinverse(const CartesianTwist& twist) const {
  if (this->is_incompatible(twist)) {
    throw IncompatibleStatesException("The Jacobian and the given Cartesian twist are incompatible");
  }
  return JointVelocities(this->get_name(), this->get_joint_names(), this->pseudoinverse(twist.data()));
}

Eigen::MatrixXd Jacobian::transpose() const {
  return this->data().transpose();
}

JointTorques Jacobian::transpose(const CartesianWrench& wrench) const {
  if (this->is_incompatible(wrench)) {
    throw IncompatibleStatesException("The Jacobian and the given Cartesian wrench are incompatible");
  }
  return JointTorques(this->get_name(), this->get_joint_names(), this->transpose() * wrench.data());
}

Eigen::MatrixXd Jacobian::operator*(const Eigen::MatrixXd& matrix) const {
  assert_matrix_size(matrix, this->cols(), matrix.cols());
  return this->data() * matrix;
}

Jacobian operator*(const Eigen::Matrix<double, 6, 6>& matrix, const Jacobian& jacobian) {
  Jacobian result(jacobian);
  result.set_data(matrix * jacobian.data());
  return result;
}

CartesianTwist Jacobian::operator*(const JointVelocities& dq) const {
  if (this->is_incompatible(dq)) {
    throw IncompatibleStatesException("The Jacobian and the input JointVelocities are incompatible");
  }
  Eigen::Vector<double, 6> twist = (*this) * dq.data();
  return CartesianTwist(this->get_frame(), twist, this->get_reference_frame());
}

Jacobian operator*(const CartesianPose& pose, const Jacobian& jacobian) {
  if (pose.get_name() != jacobian.get_reference_frame()) {
    throw IncompatibleStatesException(
        "The CartesianPose and the Jacobian are incompatible, expected pose of '" + jacobian.get_reference_frame()
            + "', got '" + pose.get_name() + "'.");
  }
  Jacobian result(jacobian);
  // change the reference frame of all the columns
  for (unsigned int i = 0; i < jacobian.cols(); ++i) {
    // update position part
    result.data_.col(i).head(3) = pose.get_orientation() * jacobian.col(i).head(3);
    // update orientation part
    result.data_.col(i).tail(3) = pose.get_orientation() * jacobian.col(i).tail(3);
  }
  // change the reference frame
  result.reference_frame_ = pose.get_reference_frame();
  return result;
}

double& Jacobian::operator()(unsigned int row, unsigned int col) {
  this->assert_not_empty();
  if (row > this->rows()) {
    throw std::out_of_range("Given row is out of range: number of rows is " + std::to_string(this->rows()));
  }
  if (col > this->cols()) {
    throw std::out_of_range("Given column is out of range: number of columns is " + std::to_string(this->cols()));
  }
  return this->data_(row, col);
}

const double& Jacobian::operator()(unsigned int row, unsigned int col) const {
  this->assert_not_empty();
  if (row > this->rows()) {
    throw std::out_of_range("Given row is out of range: number of rows is " + std::to_string(this->rows()));
  }
  if (col > this->cols()) {
    throw std::out_of_range("Given column is out of range: number of columns is " + std::to_string(this->cols()));
  }
  return this->data_(row, col);
}

std::ostream& operator<<(std::ostream& os, const Jacobian& jacobian) {
  os << jacobian.to_string();
  os << " associated to '" << jacobian.get_frame();
  os << "', expressed in frame '" << jacobian.get_reference_frame() << "'" << std::endl;
  os << "joint names: [";
  for (auto& n : jacobian.get_joint_names()) { os << n << ", "; }
  os << "]";
  if (jacobian.is_empty()) {
    return os;
  }
  os << std::endl;
  os << "number of rows: " << jacobian.rows() << std::endl;
  os << "number of columns: " << jacobian.cols() << std::endl;
  for (unsigned int i = 0; i < jacobian.rows(); ++i) {
    os << "| " << jacobian(i, 0);
    for (unsigned int j = 1; j < jacobian.cols(); ++j) {
      os << ", " << jacobian(i, j);
    }
    os << " |";
    if (i != jacobian.rows() - 1) {
      os << std::endl;
    }
  }
  return os;
}
}// namespace state_representation
