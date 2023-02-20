#pragma once

#include <eigen3/Eigen/Core>
#include "state_representation/State.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/space/cartesian/CartesianTwist.hpp"
#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/IncompatibleStatesException.hpp"
#include "state_representation/exceptions/IncompatibleReferenceFramesException.hpp"

namespace state_representation {
/**
 * @class Shape
 */
class Shape : public State {
public:
  /**
   * @brief Empty constructor
   */
  Shape();

  /**
   * @brief Constructor with name and reference frame
   * @param name Name of the Shape and its state
   * @param reference_frame The reference frame in which the state is expressed (default is "world")
   */
  explicit Shape(const std::string& name, const std::string& reference_frame = "world");

  /**
   * @brief Copy constructor from another Shape
   * @param shape The Shape to copy from
   */
  Shape(const Shape& shape);

  /**
   * @brief Constructor for a Shape with identity state
   * @param name Name of the Shape and its state
   * @param reference_frame The reference frame in which the state is expressed (default is "world")
   */
  static Shape Unit(const std::string& name, const std::string& reference_frame = "world");

  /**
   * @brief Swap the values of the two Shapes
   * @param state1 Shape to be swapped with 2
   * @param state2 Shape to be swapped with 1
   */
  friend void swap(Shape& state1, Shape& state2);

  /**
   * @brief Copy assignment operator that has to be defined to the custom assignment operator
   * @param state The Shape with value to assign
   * @return Reference to the current Shape with new values
   */
  Shape& operator=(const Shape& state);

  /**
   * @brief Getter of the state
   * @return The state of the Shape
   */
  const CartesianState& get_center_state() const;

  /**
   * @brief Getter of the pose from the state
   * @return The pose of the Shape
   */
  const CartesianPose& get_center_pose() const;

  /**
   * @brief Getter of the position from the state
   * @return The position of the Shape
   */
  const Eigen::Vector3d get_center_position() const;

  /**
   * @brief Getter of the orientation from the state
   * @return The orientation of the Shape
   */
  const Eigen::Quaterniond get_center_orientation() const;

  /**
   * @brief Getter of the twist from the state
   * @return The twist of the Shape
   */
  const CartesianTwist& get_center_twist() const;

  /**
   * @brief Setter of the state
   * @param state The new state
   */
  void set_center_state(const CartesianState& state);

  /**
   * @brief Setter of the pose
   * @param pose The new pose
   */
  void set_center_pose(const CartesianPose& pose);

  /**
   * @brief Setter of the position
   * @param position The new position
   */
  void set_center_position(const Eigen::Vector3d& position);

  /**
   * @brief Setter of the pose
   * @param pose The new pose
   */
  void set_center_orientation(const Eigen::Quaterniond& orientation);

  /**
    * @brief Overload the ostream operator for printing
    * @param os The ostream to append the string representing the state
    * @param state The state to print
    * @return The appended ostream
     */
  friend std::ostream& operator<<(std::ostream& os, const Shape& shape);

protected:
  /**
   * @copydoc State::to_string
   */
  std::string to_string() const override;

private:
  CartesianState center_state_; ///< pose and potentially velocities and accelerations of the shape if moving
};

inline void swap(Shape& state1, Shape& state2) {
  swap(static_cast<State&>(state1), static_cast<State&>(state2));
  std::swap(state1.center_state_, state2.center_state_);
}

inline Shape& Shape::operator=(const Shape& state) {
  Shape tmp(state);
  swap(*this, tmp);
  return *this;
}

inline const CartesianState& Shape::get_center_state() const {
  return this->center_state_;
}

inline const CartesianPose& Shape::get_center_pose() const {
  return static_cast<const CartesianPose&>(this->center_state_);
}

inline const Eigen::Vector3d Shape::get_center_position() const {
  return this->center_state_.get_position();
}

inline const Eigen::Quaterniond Shape::get_center_orientation() const {
  return this->center_state_.get_orientation();
}

inline const CartesianTwist& Shape::get_center_twist() const {
  return static_cast<const CartesianTwist&>(this->center_state_);
}

inline void Shape::set_center_state(const CartesianState& state) {
  this->center_state_ = state;
  this->set_empty(false);
  this->reset_timestamp();
}

inline void Shape::set_center_pose(const CartesianPose& pose) {
  if (this->center_state_.get_reference_frame() != pose.get_reference_frame()) {
    throw exceptions::IncompatibleReferenceFramesException(
        "The shape state and the given pose are not expressed in the same reference frame");
  }
  this->center_state_.set_pose(pose.get_position(), pose.get_orientation());
  this->set_empty(false);
  this->reset_timestamp();
}

inline void Shape::set_center_position(const Eigen::Vector3d& position) {
  this->center_state_.set_position(position);
  this->set_empty(false);
  this->reset_timestamp();
}

inline void Shape::set_center_orientation(const Eigen::Quaterniond& orientation) {
  this->center_state_.set_orientation(orientation);
  this->set_empty(false);
  this->reset_timestamp();
}
}