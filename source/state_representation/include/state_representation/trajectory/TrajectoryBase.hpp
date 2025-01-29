#pragma once

#include <chrono>
#include <deque>
#include <eigen3/Eigen/src/Core/Matrix.h>

#include "state_representation/State.hpp"
#include "state_representation/exceptions/IncompatibleSizeException.hpp"

namespace state_representation {

/**
 * @class TrajectoryPoint
 * @brief Struct to represent the base characteristics of a trajectory point
 */
struct TrajectoryPoint {
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
   * @brief Get list of trajectory point durations
   * @return the list of trajectory point durations
   */
  const std::vector<std::chrono::nanoseconds> get_durations() const;

  /**
   * @brief Get list of trajectory point times from start
   * @return the list of trajectory point times from start
   */
  const std::vector<std::chrono::nanoseconds> get_times_from_start() const;

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
  explicit TrajectoryBase();

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
   * @param idx the index
   * @return the trajectory point
   * @throw std::out_of_range if index is out of range
   */
  const TrajectoryT& operator[](unsigned int idx) const;

  /**
   * @brief Operator overload for returning a single trajectory point and
   * corresponding time
   * @param idx the index
   * @return the trajectory point
   * @throw std::out_of_range if index is out of range
   */
  TrajectoryT& operator[](unsigned int idx);

private:
  std::deque<TrajectoryT> points_;
};

template<typename TrajectoryT>
inline TrajectoryBase<TrajectoryT>::TrajectoryBase() : State() {
  this->reset();
}

template<typename TrajectoryT>
inline TrajectoryBase<TrajectoryT>::TrajectoryBase(const std::string& name) : State(name) {
  this->reset();
}

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
  if (new_points.size() == 0) {
    return;
  }
  for (auto point : new_points) {
    points_.push_back(point);
  }
}

template<typename TrajectoryT>
inline void TrajectoryBase<TrajectoryT>::insert_point(const TrajectoryT& new_point, unsigned int index) {
  if (index > this->points_.size()) {
    throw std::out_of_range("Index out of range");
  }
  this->set_empty(false);
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
    this->set_empty(false);
  }
}

template<typename TrajectoryT>
inline void TrajectoryBase<TrajectoryT>::delete_point(unsigned int index) {
  if (index >= this->points_.size()) {
    throw std::out_of_range("Index out of range");
  }
  this->points_.erase(this->points_.begin() + index);
  if (this->points_.empty()) {
    this->set_empty(false);
  }
}

template<typename TrajectoryT>
inline const std::vector<TrajectoryT> TrajectoryBase<TrajectoryT>::get_points() const {
  return std::vector<TrajectoryT>(this->points_.begin(), this->points_.end());
}

template<typename TrajectoryT>
inline const TrajectoryT& TrajectoryBase<TrajectoryT>::get_point(unsigned int index) const {
  if (index >= this->points_.size()) {
    throw std::out_of_range("Index out of range");
  }
  return this->points_[index];
}

template<typename TrajectoryT>
inline TrajectoryT& TrajectoryBase<TrajectoryT>::get_point(unsigned int index) {
  if (index >= this->points_.size()) {
    throw std::out_of_range("Index out of range");
  }
  return this->points_[index];
}

template<typename TrajectoryT>
inline void TrajectoryBase<TrajectoryT>::set_point(const TrajectoryT& point, unsigned int index) {
  if (index >= this->points_.size()) {
    throw std::out_of_range("Index out of range");
  }
  this->points_[index] = point;
}

template<typename TrajectoryT>
inline void TrajectoryBase<TrajectoryT>::set_points(const std::vector<TrajectoryT>& points) {
  if (points.size() == 0) {
    throw exceptions::IncompatibleSizeException("No points provided");
  }
  if (points.size() != this->points_.size()) {
    throw exceptions::IncompatibleSizeException("The size of the current vector and the new vector are not equal");
  }
  for (unsigned int i = 0; i < points.size(); ++i) {
    this->points_[i] = points[i];
  }
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
inline unsigned int TrajectoryBase<TrajectoryT>::get_size() const {
  return this->points_.size();
}

template<typename TrajectoryT>
inline const TrajectoryT& TrajectoryBase<TrajectoryT>::operator[](unsigned int index) const {
  if (index >= this->points_.size()) {
    throw std::out_of_range("Index out of range");
  }
  return this->points_[index];
}

template<typename TrajectoryT>
inline TrajectoryT& TrajectoryBase<TrajectoryT>::operator[](unsigned int index) {
  if (index >= this->points_.size()) {
    throw std::out_of_range("Index out of range");
  }
  return this->points_[index];
}
}// namespace state_representation
