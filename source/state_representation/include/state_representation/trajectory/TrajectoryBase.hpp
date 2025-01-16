#pragma once

#include "state_representation/State.hpp"
#include "state_representation/StateType.hpp"

#include <chrono>
#include <deque>

namespace state_representation {
template<typename TrajectoryT>
class TrajectoryBase : public State {
public:
  /**
   * @brief Get attribute list of trajectory points
   */
  const std::deque<TrajectoryT>& get_points() const;

  /**
   * @brief Get the trajectory point at given index
   * @param index the index
   */
  const TrajectoryT& get_point(unsigned int index) const;

  /**
   * @brief Get attribute list of trajectory times
   */
  const std::deque<std::chrono::nanoseconds>& get_times() const;

  /**
   * @brief Get attribute number of point in trajectory
   */
  int get_size() const;

  /**
   * @brief Reset trajectory
   */
  virtual void reset();

  /**
   * @brief Delete last point and corresponding time from trajectory
   */
  virtual void delete_point();

  /**
   * @brief Clear trajectory
   */
  virtual void clear();

protected:
  /**
   * @brief Empty constructor
   */
  explicit TrajectoryBase();

  /**
   * @brief Constructor with name provided
   * @brief name the name of the state
   */
  explicit TrajectoryBase(const std::string& name);

  /**
   * @brief Add new point and corresponding time to trajectory
   */
  template<typename DurationT>
  void add_point(const TrajectoryT& new_point, const std::chrono::duration<int64_t, DurationT>& new_time);

  /**
   * @brief Insert new point and corresponding time to trajectory between two
   * already existing points
   */
  template<typename DurationT>
  void insert_point(const TrajectoryT& new_point, const std::chrono::duration<int64_t, DurationT>& new_time, int pos);

  /**
   * @brief Get the trajectory point at given index
   * @param index the index
   */
  TrajectoryT& get_point(unsigned int index);

  /**
   * @brief Set the trajectory point at given index
   * @param point the new point
   * @param new_time the new time
   * @param index the index
   * @return Success of the operation
   */
  template<typename DurationT>
  bool
  set_point(const TrajectoryT& point, const std::chrono::duration<int64_t, DurationT>& new_time, unsigned int index);

  /**
   * @brief Set the trajectory points from a vector of points
   * @param points vector of new points
   * @param new_time vector of new times
   * @return Success of the operation
   */
  template<typename DurationT>
  bool set_points(
      const std::vector<TrajectoryT>& points, const std::vector<std::chrono::duration<int64_t, DurationT>>& new_times);

  /**
   * @brief Operator overload for returning a single trajectory point and
   * corresponding time
   */
  const std::pair<TrajectoryT, std::chrono::nanoseconds> operator[](unsigned int idx) const;

  /**
   * @brief Operator overload for returning a single trajectory point and
   * corresponding time
   */
  std::pair<TrajectoryT, std::chrono::nanoseconds> operator[](unsigned int idx);

private:
  std::deque<TrajectoryT> points_;
  std::deque<std::chrono::nanoseconds> times_;
};

template<typename TrajectoryT>
inline TrajectoryBase<TrajectoryT>::TrajectoryBase() : State() {
  this->set_type(StateType::NONE);
  this->reset();
}

template<typename TrajectoryT>
inline TrajectoryBase<TrajectoryT>::TrajectoryBase(const std::string& name) : State(name) {
  this->set_type(StateType::NONE);
  this->reset();
}

template<typename TrajectoryT>
inline void TrajectoryBase<TrajectoryT>::reset() {
  this->State::reset();
  this->points_.clear();
  this->times_.clear();
}

template<typename TrajectoryT>
template<typename DurationT>
inline void TrajectoryBase<TrajectoryT>::add_point(
    const TrajectoryT& new_point, const std::chrono::duration<int64_t, DurationT>& new_time) {
  this->set_empty(false);
  this->points_.push_back(new_point);

  if (!this->times_.empty()) {
    auto const previous_time = this->times_.back();
    this->times_.push_back(previous_time + new_time);
  } else {
    this->times_.push_back(new_time);
  }
}

template<typename TrajectoryT>
template<typename DurationT>
inline void TrajectoryBase<TrajectoryT>::insert_point(
    const TrajectoryT& new_point, const std::chrono::duration<int64_t, DurationT>& new_time, int pos) {
  this->set_empty(false);

  auto it_points = this->points_.begin();
  auto it_times = this->times_.begin();
  std::advance(it_points, pos);
  std::advance(it_times, pos);

  this->points_.insert(it_points, new_point);

  auto previous_time = this->times_[pos - 1];
  this->times_.insert(it_times, previous_time + new_time);

  for (unsigned int i = pos + 1; i <= this->points_.size(); i++) {
    this->times_[i] += new_time;
  }
}

template<typename TrajectoryT>
inline void TrajectoryBase<TrajectoryT>::delete_point() {
  this->set_empty(false);
  if (!this->points_.empty()) {
    this->points_.pop_back();
  }
  if (!this->times_.empty()) {
    this->times_.pop_back();
  }
}

template<typename TrajectoryT>
inline void TrajectoryBase<TrajectoryT>::clear() {
  this->points_.clear();
  this->times_.clear();
}

template<typename TrajectoryT>
inline const std::deque<TrajectoryT>& TrajectoryBase<TrajectoryT>::get_points() const {
  return this->points_;
}

template<typename TrajectoryT>
inline const TrajectoryT& TrajectoryBase<TrajectoryT>::get_point(unsigned int index) const {
  return this->points_[index];
}

template<typename TrajectoryT>
inline TrajectoryT& TrajectoryBase<TrajectoryT>::get_point(unsigned int index) {
  return this->points_[index];
}

template<typename TrajectoryT>
template<typename DurationT>
inline bool TrajectoryBase<TrajectoryT>::set_point(
    const TrajectoryT& point, const std::chrono::duration<int64_t, DurationT>& new_time, unsigned int index) {
  if (index < this->points_.size()) {
    this->points_[index] = point;
    if (index == 0) {
      this->times_[index] = new_time;
    } else {
      this->times_[index] = this->times_[index - 1] + new_time;
    }
    for (unsigned int i = index + 1; i < this->points_.size(); ++i) {
      this->times_[i] += this->times_[index - 1];
    }
    return true;
  }
  return false;
}

template<typename TrajectoryT>
template<typename DurationT>
inline bool TrajectoryBase<TrajectoryT>::set_points(
    const std::vector<TrajectoryT>& points, const std::vector<std::chrono::duration<int64_t, DurationT>>& new_times) {
  if (points.size() != new_times.size()) {
    return false;
  }
  this->clear();
  for (unsigned int i = 0; i < points.size(); ++i) {
    this->add_point(points[i], new_times[i]);
  }
  return true;
}

template<typename TrajectoryT>
inline const std::deque<std::chrono::nanoseconds>& TrajectoryBase<TrajectoryT>::get_times() const {
  return this->times_;
}

template<typename TrajectoryT>
inline int TrajectoryBase<TrajectoryT>::get_size() const {
  return this->points_.size();
}

template<typename TrajectoryT>
inline const std::pair<TrajectoryT, std::chrono::nanoseconds>
TrajectoryBase<TrajectoryT>::operator[](unsigned int idx) const {
  return std::make_pair(this->points_[idx], this->times_[idx]);
}

template<typename TrajectoryT>
inline std::pair<TrajectoryT, std::chrono::nanoseconds> TrajectoryBase<TrajectoryT>::operator[](unsigned int idx) {
  this->set_empty(false);
  return std::make_pair(this->points_[idx], this->times_[idx]);
}
}// namespace state_representation
