#include "state_representation/geometry/Shape.hpp"

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