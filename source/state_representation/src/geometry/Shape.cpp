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

std::ostream& operator<<(std::ostream& os, const Shape& shape) {
  auto prefix = shape.is_empty() ? "Empty ": "";
  os << prefix << "Shape '" << shape.get_name() << "'";
  if (shape.is_empty()) {
    return os;
  }
  os << std::endl << "state:" << std::endl;
  os << shape.get_center_state();
  return os;
}
}