#include "state_representation/geometry/Shape.hpp"

#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/IncompatibleReferenceFramesException.hpp"

namespace state_representation {

Shape::Shape() : State(), center_state_(CartesianState::Identity("")) {
  this->set_type(StateType::GEOMETRY_SHAPE);
}

Shape::Shape(const std::string& name, const std::string& reference_frame) :
    State(name), center_state_(CartesianState::Identity(name, reference_frame)) {
  this->set_type(StateType::GEOMETRY_SHAPE);
}

Shape::Shape(const Shape& shape) : Shape(shape.get_name()) {
  if (shape) {
    this->set_center_state(shape.get_center_state());
  }
}

Shape Shape::Unit(const std::string& name, const std::string& reference_frame) {
  Shape unit = Shape(name, reference_frame);
  unit.set_empty(false);
  return unit;
}

Shape& Shape::operator=(const Shape& state) {
  Shape tmp(state);
  swap(*this, tmp);
  return *this;
}

const CartesianState& Shape::get_center_state() const {
  this->assert_not_empty();
  return this->center_state_;
}

const CartesianPose& Shape::get_center_pose() const {
  this->assert_not_empty();
  return static_cast<const CartesianPose&>(this->center_state_);
}

const Eigen::Vector3d Shape::get_center_position() const {
  this->assert_not_empty();
  return this->center_state_.get_position();
}

const Eigen::Quaterniond Shape::get_center_orientation() const {
  this->assert_not_empty();
  return this->center_state_.get_orientation();
}

const CartesianTwist& Shape::get_center_twist() const {
  this->assert_not_empty();
  return static_cast<const CartesianTwist&>(this->center_state_);
}

void Shape::set_center_state(const CartesianState& state) {
  if (state.is_empty()) {
    throw exceptions::EmptyStateException(state.get_name() + " state is empty");
  }
  this->center_state_ = state;
  this->set_empty(false);
  this->reset_timestamp();
}

void Shape::set_center_pose(const CartesianPose& pose) {
  if (this->center_state_.get_reference_frame() != pose.get_reference_frame()) {
    throw exceptions::IncompatibleReferenceFramesException(
        "The shape state and the given pose are not expressed in the same reference frame");
  }
  this->center_state_.set_pose(pose.get_position(), pose.get_orientation());
  this->set_empty(false);
  this->reset_timestamp();
}

void Shape::set_center_position(const Eigen::Vector3d& position) {
  this->center_state_.set_position(position);
  this->set_empty(false);
  this->reset_timestamp();
}

void Shape::set_center_orientation(const Eigen::Quaterniond& orientation) {
  this->center_state_.set_orientation(orientation);
  this->set_empty(false);
  this->reset_timestamp();
}

std::string Shape::to_string() const {
  std::stringstream s;
  s << this->State::to_string();
  if (this->is_empty()) {
    return s.str();
  }
  s << std::endl << "state:" << std::endl;
  s << this->get_center_state();
  return s.str();
}

std::ostream& operator<<(std::ostream& os, const Shape& shape) {
  os << shape.to_string();
  return os;
}
}// namespace state_representation
