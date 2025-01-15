#pragma once

#include "state_representation/State.hpp"
#include "state_representation/StateType.hpp"

#include <chrono>
#include <deque>

namespace state_representation {
template<typename TrajectoryT>
class TrajectoryBase : public State {
private:
  std::deque<TrajectoryT> points_;
  std::deque<std::chrono::nanoseconds> times_;

public:
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
   * @brief Reset trajectory
   */
  void reset();

  /**
   * @brief Delete last point and corresponding time from trajectory
   */
  void delete_point();

  /**
   * @brief Clear trajectory
   */
  void clear();

  /**
   * @brief Get attribute list of trajectory times
   */
  const std::deque<std::chrono::nanoseconds>& get_times() const;

  /**
   * @brief Get attribute number of point in trajectory
   */
  int get_size() const;

protected:
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
   * @brief Get attribute list of trajectory points
   */
  const std::deque<TrajectoryT>& get_points() const;

  /**
   * @brief Get the trajectory point at given index
   * @param index the index
   */
  const TrajectoryT& get_point(unsigned int index) const;

  /**
   * @brief Get the trajectory point at given index
   * @param index the index
   */
  TrajectoryT& get_point(unsigned int index);

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
};

template<typename TrajectoryT>
TrajectoryBase<TrajectoryT>::TrajectoryBase() : State() {
  this->set_type(StateType::NONE);
  this->reset();
}

template<typename TrajectoryT>
TrajectoryBase<TrajectoryT>::TrajectoryBase(const std::string& name) : State(name) {
  this->set_type(StateType::NONE);
  this->reset();
}

template<typename TrajectoryT>
void TrajectoryBase<TrajectoryT>::reset() {
  this->State::reset();
  this->points_.clear();
  this->times_.clear();
}

template<typename TrajectoryT>
template<typename DurationT>
void TrajectoryBase<TrajectoryT>::add_point(
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
void TrajectoryBase<TrajectoryT>::insert_point(
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
void TrajectoryBase<TrajectoryT>::delete_point() {
  this->set_empty(false);
  if (!this->points_.empty()) {
    this->points_.pop_back();
  }
  if (!this->times_.empty()) {
    this->times_.pop_back();
  }
}

template<typename TrajectoryT>
void TrajectoryBase<TrajectoryT>::clear() {
  this->points_.clear();
  this->times_.clear();
}

template<typename TrajectoryT>
inline const std::deque<TrajectoryT>& TrajectoryBase<TrajectoryT>::get_points() const {
  return this->points_;
}

template<typename TrajectoryT>
const TrajectoryT& TrajectoryBase<TrajectoryT>::get_point(unsigned int index) const {
  return this->points_[index];
}

template<typename TrajectoryT>
TrajectoryT& TrajectoryBase<TrajectoryT>::get_point(unsigned int index) {
  return this->points_[index];
}

template<typename TrajectoryT>
inline const std::deque<std::chrono::nanoseconds>& TrajectoryBase<TrajectoryT>::get_times() const {
  return this->times_;
}

template<typename TrajectoryT>
int TrajectoryBase<TrajectoryT>::get_size() const {
  return this->points_.size();
}

template<typename TrajectoryT>
const std::pair<TrajectoryT, std::chrono::nanoseconds> TrajectoryBase<TrajectoryT>::operator[](unsigned int idx) const {
  return std::make_pair(this->points_[idx], this->times_[idx]);
}

template<typename TrajectoryT>
std::pair<TrajectoryT, std::chrono::nanoseconds> TrajectoryBase<TrajectoryT>::operator[](unsigned int idx) {
  this->set_empty(false);
  return std::make_pair(this->points_[idx], this->times_[idx]);
}
}// namespace state_representation
