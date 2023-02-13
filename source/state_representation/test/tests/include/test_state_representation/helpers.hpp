#pragma once

#include <gtest/gtest.h>

#include "state_representation/State.hpp"
#include "state_representation/space/SpatialState.hpp"

using namespace state_representation;

template<typename StateT>
inline void assert_state_equal(const StateT& state1, const StateT& state2) {
  EXPECT_EQ(state1.get_type(), state2.get_type());
  EXPECT_EQ(state1.get_name(), state2.get_name());
  EXPECT_EQ(state1.is_empty(), state2.is_empty());
}

template<>
inline void assert_state_equal(const SpatialState& state1, const SpatialState& state2) {
  assert_state_equal<State>(state1, state2);
  EXPECT_EQ(state1.get_reference_frame(), state2.get_reference_frame());
}