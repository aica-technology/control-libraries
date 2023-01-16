#include "state_representation/space/SpatialState.hpp"

namespace state_representation {

SpatialState::SpatialState() : State(StateType::SPATIAL_STATE), reference_frame_("world") {}

SpatialState::SpatialState(const std::string& name, const std::string& reference_frame) :
    State(StateType::SPATIAL_STATE, name), reference_frame_(reference_frame) {}

SpatialState& SpatialState::operator=(const SpatialState& state) {
  SpatialState tmp(state);
  swap(*this, tmp);
  return *this;
}

const std::string& SpatialState::get_reference_frame() const {
  return this->reference_frame_;
}

void SpatialState::set_reference_frame(const std::string& reference_frame) {
  this->reference_frame_ = reference_frame;
}

bool SpatialState::is_incompatible(const State& state) const {
  // FIXME: not sure that the names should be compared here
  bool compatible = (this->get_name() == state.get_name())
      && (this->reference_frame_ == dynamic_cast<const SpatialState&>(state).reference_frame_);
  return compatible;
}

std::ostream& operator<<(std::ostream& os, const SpatialState& state) {
  auto prefix = state.is_empty() ? "Empty " : "";
  os << prefix << "SpatialState: " << state.get_name() << " expressed in " << state.get_reference_frame() << " frame";
  return os;
}

}// namespace state_representation
