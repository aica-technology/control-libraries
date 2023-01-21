#include "state_representation/geometry/Shape.hpp"

namespace state_representation {

Shape::Shape() : State(StateType::GEOMETRY_SHAPE) {}

Shape::Shape(const std::string& name, const std::string& reference_frame) :
    State(StateType::GEOMETRY_SHAPE, name), center_state_(CartesianPose::Identity(name, reference_frame)) {
}

Shape::Shape(const StateType& type, const CartesianState& center_state) : State(type), center_state_(center_state) {}

Shape::Shape(const Shape& shape) :
    State(StateType::GEOMETRY_SHAPE, shape.get_name()), center_state_(shape.center_state_) {}

std::ostream& operator<<(std::ostream& os, const Shape& shape) {
  if (shape.is_empty()) {
    os << "Empty Shape";
  } else {
    os << "Shape " << shape.get_name() << " with state:" << std::endl;
    os << shape.get_center_state();
  }
  return os;
}
}