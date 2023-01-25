#include "state_representation/space/SpatialState.hpp"

#include "state_representation/exceptions/InvalidCastException.hpp"

namespace state_representation {

SpatialState::SpatialState() : State(), reference_frame_("world") {
  this->set_type(StateType::SPATIAL_STATE);
}

SpatialState::SpatialState(const std::string& name, const std::string& reference_frame) :
    State(name), reference_frame_(reference_frame) {
  this->set_type(StateType::SPATIAL_STATE);
}

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
  try {
    auto other = dynamic_cast<const SpatialState&>(state);
    // the three conditions for compatibility are:
    // 1) this name matches other reference frame (this is parent transform of other)
    // 2) this reference frame matches other name (this is child transform of other)
    // 3) this reference frame matches other reference frame (this is sibling transform of other)
    return (this->get_name() != other.reference_frame_) && (this->reference_frame_ != other.get_name())
        && (this->get_reference_frame() != other.reference_frame_);
  } catch (const std::bad_cast& ex) {
    throw exceptions::InvalidCastException(
        std::string("Could not cast the given object to a SpatialState: ") + ex.what());
  }
}

std::ostream& operator<<(std::ostream& os, const SpatialState& state) {
  auto prefix = state.is_empty() ? "Empty " : "";
  os << prefix << "SpatialState: " << state.get_name() << " expressed in " << state.get_reference_frame() << " frame";
  return os;
}

}// namespace state_representation
