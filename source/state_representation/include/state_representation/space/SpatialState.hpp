#pragma once

#include "state_representation/State.hpp"

namespace state_representation {

class SpatialState : public State {
public:
  /**
   * @brief Empty constructor
   */
  SpatialState();

  /**
   * @brief Constructor with name and reference frame specification
   * @param name The name of the state
   * @param reference_frame The reference frame in which the state is expressed, by default world
   */
  explicit SpatialState(const std::string& name, const std::string& reference_frame = "world");

  /**
   * @brief Copy constructor from another spatial state
   */
  SpatialState(const SpatialState& state) = default;

  /**
   * @brief Swap the values of the two SpatialState
   * @param state1 Spatial state to be swapped with 2
   * @param state2 Spatial state to be swapped with 1
   */
  friend void swap(SpatialState& state1, SpatialState& state2);

  /**
   * @brief Copy assignment operator that has to be defined to the custom assignment operator
   * @param state The state with value to assign
   * @return Reference to the current state with new values
   */
  SpatialState& operator=(const SpatialState& state);

  /**
   * @brief Getter of the reference frame as const reference
   */
  const std::string& get_reference_frame() const;

  /**
   * @brief Setter of the reference frame
   */
  virtual void set_reference_frame(const std::string& reference_frame);

  /**
   * @brief Check if the spatial state is incompatible for operations with the state given as argument
   * @param state The state to check compatibility with
   */
  bool is_incompatible(const State& state) const override;

  /**
   * @brief Overload the ostream operator for printing
   * @param os The ostream to append the string representing the state to
   * @param state The spatial state to print
   * @return The appended ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const SpatialState& state);

protected:
  /**
   * @brief Constructor with type, name and reference frame specification (for derived classes)
   * @param type The type of the state
   * @param name The name of the state
   * @param reference_frame The reference frame in which the state is expressed
   */
  explicit SpatialState(
      const StateType& type, const std::string& name = "", const std::string& reference_frame = "world"
  );

private:
  std::string reference_frame_; ///< name of the reference frame
};

inline void swap(SpatialState& state1, SpatialState& state2) {
  swap(static_cast<State&>(state1), static_cast<State&>(state2));
  std::swap(state1.reference_frame_, state2.reference_frame_);
}

}// namespace state_representation
