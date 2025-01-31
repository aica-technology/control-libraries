#pragma once

#include <chrono>
#include <concepts>
#include <deque>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <stdexcept>

#include "state_representation/State.hpp"
#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/IncompatibleSizeException.hpp"

namespace state_representation {

/**
 * @class TrajectoryPoint
 * @brief Struct to represent the base characteristics of a trajectory point
 */
struct TrajectoryPoint {
  /**
   * @brief Empty constructor
   */
  TrajectoryPoint() = default;

  /**
   * @brief Constructor with name, data, and duration
   * @param name the trajectory point name
   * @param data the (flattened) trajectory data
   * @param duration the intended duration for the trajectory point 
   */
  TrajectoryPoint(const std::string& name, const Eigen::VectorXd& data, const std::chrono::nanoseconds& duration)
      : name(name), data(data), duration(duration) {}

  std::string name;
  Eigen::VectorXd data;
  std::chrono::nanoseconds duration;
};

/**
 * @class TrajectoryBase
 * @brief Base class that offers common functionalities for trajectory classes
 */
template<typename TrajectoryT>
class TrajectoryBase : public State {
public:
  /**
   * @brief Get the duration of the trajectory point at given index
   * @param index the index
   * @return the duration of the trajectory point
   */
  const std::chrono::nanoseconds& get_duration(unsigned int index) const;

  /**
   * @brief Get list of trajectory point durations
   * @return the list of trajectory point durations
   */
  const std::vector<std::chrono::nanoseconds> get_durations() const;

  /**
   * @brief Get the time from start of the trajectory point at given index
   * @param index the index
   * @return the time from start of the trajectory point
   */
  const std::chrono::nanoseconds get_time_from_start(unsigned int index) const;

  /**
   * @brief Get list of trajectory point times from start
   * @return the list of trajectory point times from start
   */
  const std::vector<std::chrono::nanoseconds> get_times_from_start() const;

  /**
   * @brief Get the total duration of the trajectory
   * @return the total duration of the trajectory
   */
  const std::chrono::nanoseconds get_trajectory_duration() const;

  /**
   * @brief Get number of points in trajectory
   * @return the number of points in trajectory
   */
  unsigned int get_size() const;

  /**
   * @brief Delete the last point from trajectory
   */
  void delete_point();

  /**
   * @brief Delete the last point from trajectory
   * @param index the index of the point to delete
   * @throw std::out_of_range if index is out of range
   */
  void delete_point(unsigned int index);

  /**
   * @brief Reset trajectory
   */
  virtual void reset();

protected:
  /**
   * @brief Empty constructor
   */
  explicit TrajectoryBase() = default;

  /**
   * @brief Constructor with name provided
   * @param name the name of the state
   */
  explicit TrajectoryBase(const std::string& name);

  /**
   * @brief Get list of trajectory points
   * @return the list of trajectory points
   */
  const std::vector<TrajectoryT> get_points() const;

  /**
   * @brief Get the trajectory point at given index
   * @param index the index
   * @return the trajectory point
   * @throw std::out_of_range if index is out of range
   */
  const TrajectoryT& get_point(unsigned int index) const;

  /**
   * @brief Add new point to trajectory
   * @param new_point the new point
   */
  void add_point(const TrajectoryT& new_point);

  /**
   * @brief Add new points to trajectory
   * @param new_point the new point
   * @throw IncompatibleSizeException if points vector is empty
   */
  void add_points(const std::vector<TrajectoryT>& new_points);

  /**
   * @brief Insert new trajectory point between two already existing points
   * @param new_point the new point
   * @param index the desired position of the new point in the queue
   * @throw std::out_of_range if index is out of range
   */
  void insert_point(const TrajectoryT& new_point, unsigned int index);

  /**
   * @brief Get the trajectory point at given index
   * @param index the index
   * @return the trajectory point
   */
  TrajectoryT& get_point(unsigned int index);

  /**
   * @brief Set the trajectory point at given index
   * @param point the new point
   * @param index the index
   * @throw std::out_of_range if index is out of range
   */
  void set_point(const TrajectoryT& point, unsigned int index);

  /**
   * @brief Set the trajectory points from a vector of points
   * @param points vector of new points
   * @throw IncompatibleSizeException if points vector is empty
   */
  void set_points(const std::vector<TrajectoryT>& points);

  /**
   * @brief Operator overload for returning a single trajectory point and
   * corresponding time
   * @param index the index
   * @return the trajectory point
   * @throw IncompatibleSizeException if points vector is empty or different size than current points
   * @throw std::out_of_range if index is out of range
   */
  const TrajectoryT& operator[](unsigned int index) const;

  /**
   * @brief Operator overload for returning a single trajectory point and
   * corresponding time
   * @param index the index
   * @return the trajectory point
   * @throw std::out_of_range if index is out of range
   */
  TrajectoryT& operator[](unsigned int index);

  /**
   * @brief Assert the index provided is in range of the current points list
   * @param index the index
   * @throw std::out_of_range if index is out of range
   */
  void assert_index_in_range(unsigned int index) const;

  /**
   * @brief Assert the points vector provided is not empty 
   * @param points the vector of points to check
   * @throw IncompatibleSizeException if points empty
   */
  template<typename T>
  void assert_points_empty(const std::vector<T>& points) const;

  /**
   * @brief Assert the points vector provided is of the same size as the current points 
   * @param points the vector of points to check
   * @throw IncompatibleSizeException if size points provided different to current points
   */
  template<typename T>
  void assert_points_size(const std::vector<T>& points) const;

  /**
   * @brief Assert the points vector and durations vectors are of equal size 
   * @param points the vector of points to check
   * @param durations the vector of durations to check
   * @throw IncompatibleSizeException if the two vector sizes are not equal
   */
  template<typename T>
  void assert_points_durations_sizes_equal(
      const std::vector<T>& points, const std::vector<std::chrono::nanoseconds>& durations
  ) const;

  /**
   * @brief Assert the that 2 vectors are element wise equal
   * @param lvec the vector of points to check
   * @param rvec the vector of durations to check
   * @throws std::runtime-derived exception if vectors differ
   */
  template<typename T, typename ExceptionType>
    requires std::derived_from<ExceptionType, std::runtime_error>
  void assert_vector_ewise_equal(
      const std::vector<T>& lvec, const std::vector<T>& rvec,
      const std::string& msg = "The vectors provided contain elements that differ!"
  ) const;

  /**
   * @brief Assert that vector of State type does not contain empty elements
   * @param states the vector of State-type elements to check 
   * @throws EmptyStateException if any of the elements is empty
   */
  template<typename StateT>
    requires std::derived_from<StateT, typename state_representation::State>
  void assert_contains_empty_state(const std::vector<StateT>& states) const;

  /**
   * @brief Assert that the trajectory is empty
   * @throws EmptyStateException if any of the elements is empty
   */
  void assert_trajectory_empty() const;

private:
  std::deque<TrajectoryT> points_;
};

template<typename TrajectoryT>
inline TrajectoryBase<TrajectoryT>::TrajectoryBase(const std::string& name) : State(name) {}

template<typename TrajectoryT>
inline void TrajectoryBase<TrajectoryT>::reset() {
  this->State::reset();
  this->points_.clear();
}

template<typename TrajectoryT>
inline void TrajectoryBase<TrajectoryT>::add_point(const TrajectoryT& new_point) {
  this->set_empty(false);
  this->points_.push_back(new_point);
}

template<typename TrajectoryT>
inline void TrajectoryBase<TrajectoryT>::add_points(const std::vector<TrajectoryT>& new_points) {
  if (new_points.empty()) {
    throw exceptions::IncompatibleSizeException("No points provided");
  }
  for (auto point : new_points) {
    points_.push_back(point);
  }
}

template<typename TrajectoryT>
inline void TrajectoryBase<TrajectoryT>::insert_point(const TrajectoryT& new_point, unsigned int index) {
  this->set_empty(false);
  this->assert_index_in_range(index);
  auto it_points = this->points_.begin();
  std::advance(it_points, index);
  this->points_.insert(it_points, new_point);
}

template<typename TrajectoryT>
inline void TrajectoryBase<TrajectoryT>::delete_point() {
  if (!this->points_.empty()) {
    this->points_.pop_back();
  }
  if (this->points_.empty()) {
    this->set_empty(true);
  }
}

template<typename TrajectoryT>
inline void TrajectoryBase<TrajectoryT>::delete_point(unsigned int index) {
  this->assert_index_in_range(index);
  this->points_.erase(this->points_.begin() + index);
  if (this->points_.empty()) {
    this->set_empty(true);
  }
}

template<typename TrajectoryT>
inline const std::vector<TrajectoryT> TrajectoryBase<TrajectoryT>::get_points() const {
  return std::vector<TrajectoryT>(this->points_.begin(), this->points_.end());
}

template<typename TrajectoryT>
inline const TrajectoryT& TrajectoryBase<TrajectoryT>::get_point(unsigned int index) const {
  this->assert_index_in_range(index);
  return this->points_[index];
}

template<typename TrajectoryT>
inline TrajectoryT& TrajectoryBase<TrajectoryT>::get_point(unsigned int index) {
  this->assert_index_in_range(index);
  return this->points_[index];
}

template<typename TrajectoryT>
inline void TrajectoryBase<TrajectoryT>::set_point(const TrajectoryT& point, unsigned int index) {
  this->assert_index_in_range(index);
  this->points_[index] = point;
}

template<typename TrajectoryT>
inline void TrajectoryBase<TrajectoryT>::set_points(const std::vector<TrajectoryT>& points) {
  this->assert_points_empty(points);
  this->assert_points_size(points);
  for (unsigned int i = 0; i < points.size(); ++i) {
    this->points_[i] = points[i];
  }
}

template<typename TrajectoryT>
inline const std::chrono::nanoseconds& TrajectoryBase<TrajectoryT>::get_duration(unsigned int index) const {
  this->assert_index_in_range(index);
  return this->points_[index].duration;
}

template<typename TrajectoryT>
inline const std::vector<std::chrono::nanoseconds> TrajectoryBase<TrajectoryT>::get_durations() const {
  std::vector<std::chrono::nanoseconds> durations;
  for (unsigned int i = 0; i < this->points_.size(); ++i) {
    durations.push_back(this->points_[i].duration);
  }
  return durations;
}

template<typename TrajectoryT>
inline const std::chrono::nanoseconds TrajectoryBase<TrajectoryT>::get_time_from_start(unsigned int index) const {
  this->assert_index_in_range(index);
  std::chrono::nanoseconds time_from_start = std::chrono::nanoseconds(0);
  for (unsigned int i = 0; i <= index; ++i) {
    time_from_start += this->points_[i].duration;
  }
  return time_from_start;
}

template<typename TrajectoryT>
inline const std::vector<std::chrono::nanoseconds> TrajectoryBase<TrajectoryT>::get_times_from_start() const {
  std::vector<std::chrono::nanoseconds> times_from_start;
  std::chrono::nanoseconds time_from_start = std::chrono::nanoseconds(0);
  for (unsigned int i = 0; i < this->points_.size(); ++i) {
    time_from_start += this->points_[i].duration;
    times_from_start.push_back(time_from_start);
  }
  return times_from_start;
}

template<typename TrajectoryT>
inline const std::chrono::nanoseconds TrajectoryBase<TrajectoryT>::get_trajectory_duration() const {
  return this->get_size() ? this->get_times_from_start().back() : std::chrono::nanoseconds(0);
}

template<typename TrajectoryT>
inline unsigned int TrajectoryBase<TrajectoryT>::get_size() const {
  return this->points_.size();
}

template<typename TrajectoryT>
inline const TrajectoryT& TrajectoryBase<TrajectoryT>::operator[](unsigned int index) const {
  this->assert_index_in_range(index);
  return this->points_[index];
}

template<typename TrajectoryT>
inline TrajectoryT& TrajectoryBase<TrajectoryT>::operator[](unsigned int index) {
  this->assert_index_in_range(index);
  return this->points_[index];
}

template<typename TrajectoryT>
inline void TrajectoryBase<TrajectoryT>::assert_index_in_range(unsigned int index) const {
  if (index >= this->points_.size()) {
    throw std::out_of_range("Index out of range");
  }
}

template<typename TrajectoryT>
template<typename T>
inline void TrajectoryBase<TrajectoryT>::assert_points_empty(const std::vector<T>& points) const {
  if (points.empty()) {
    throw exceptions::IncompatibleSizeException("Empty points vector provided!");
  }
}

template<typename TrajectoryT>
template<typename T>
inline void TrajectoryBase<TrajectoryT>::assert_points_size(const std::vector<T>& points) const {
  if (points.size() != this->points_.size()) {
    throw exceptions::IncompatibleSizeException("The size of the current vector and the new vector are not equal");
  }
}

template<typename TrajectoryT>
template<typename T>
inline void TrajectoryBase<TrajectoryT>::assert_points_durations_sizes_equal(
    const std::vector<T>& points, const std::vector<std::chrono::nanoseconds>& durations
) const {
  if (points.size() != durations.size()) {
    throw exceptions::IncompatibleSizeException("The size of the points and durations vectors are not equal");
  }
}

template<typename TrajectoryT>
template<typename T, typename ExceptionType>
  requires std::derived_from<ExceptionType, std::runtime_error>
inline void TrajectoryBase<TrajectoryT>::assert_vector_ewise_equal(
    const std::vector<T>& lvec, const std::vector<T>& rvec, const std::string& msg
) const {
  if (lvec != rvec) {
    throw ExceptionType(msg);
  }
}

template<typename TrajectoryT>
template<typename StateT>
  requires std::derived_from<StateT, typename state_representation::State>
inline void TrajectoryBase<TrajectoryT>::assert_contains_empty_state(const std::vector<StateT>& states) const {
  if (std::ranges::any_of(states, [&](const auto& state) { return state.is_empty(); })) {
    throw exceptions::EmptyStateException("Empty state variable provided");
  }
}

template<typename TrajectoryT>
inline void TrajectoryBase<TrajectoryT>::assert_trajectory_empty() const {
  if (this->is_empty()) {
    throw exceptions::EmptyStateException("Trajectory is empty");
  }
}
}// namespace state_representation
