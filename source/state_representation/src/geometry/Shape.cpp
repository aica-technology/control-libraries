#include "state_representation/geometry/Shape.hpp"

namespace state_representation {

Shape::Shape() : State() {
  this->set_type(StateType::GEOMETRY_SHAPE);
}

Shape::Shape(const std::string& name, const std::string& reference_frame) :
    State(name), center_state_(CartesianPose::Identity(name, reference_frame)) {
  this->set_type(StateType::GEOMETRY_SHAPE);
}

Shape::Shape(const Shape& shape) : State(shape), center_state_(shape.center_state_) {
  this->set_type(StateType::GEOMETRY_SHAPE);
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
}