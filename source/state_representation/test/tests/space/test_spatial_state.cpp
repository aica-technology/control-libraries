#include <gtest/gtest.h>

#include "state_representation/space/SpatialState.hpp"
#include "test_state_representation/helpers.hpp"

using namespace state_representation;

TEST(SpatialStateTest, Construction) {
  SpatialState state1;
  EXPECT_EQ(state1.get_type(), StateType::SPATIAL_STATE);
  EXPECT_EQ(state1.get_name(), "");
  EXPECT_EQ(state1.get_reference_frame(), "world");
  EXPECT_TRUE(state1.is_empty());
  EXPECT_FALSE(state1);

  auto state2 = SpatialState("test", "ref");
  EXPECT_EQ(state2.get_type(), StateType::SPATIAL_STATE);
  EXPECT_EQ(state2.get_name(), "test");
  EXPECT_EQ(state2.get_reference_frame(), "ref");
  EXPECT_TRUE(state2.is_empty());
  EXPECT_FALSE(state2);

  SpatialState copy(state1);
  assert_state_equal(state1, copy);

  copy = state2;
  assert_state_equal(state2, copy);
}

TEST(SpatialStateTest, Swap) {
  SpatialState state1("spatial");
  auto state1_copy = state1;
  assert_state_equal(state1, state1_copy);
  SpatialState state2("state", "ref");
  auto state2_copy = state2;
  assert_state_equal(state2, state2_copy);

  swap(state1, state2);
  assert_state_equal(state1_copy, state2);
  assert_state_equal(state2_copy, state1);
}

TEST(SpatialStateTest, Setters) {
  SpatialState state("state");
  state.set_reference_frame("ref");
  EXPECT_EQ(state.get_reference_frame(), "ref");
}

TEST(SpatialStateTest, Incompatibility) {
  SpatialState state1("spatial");
  SpatialState state2("state", "ref");
  EXPECT_TRUE(state1.is_incompatible(state2));
  state1.set_reference_frame("ref");
  EXPECT_FALSE(state1.is_incompatible(state2));
  state1.set_reference_frame("world");
  state1.set_name("ref");
  EXPECT_FALSE(state1.is_incompatible(state2));
}

TEST(SpatialStateTest, MakeShared) {
  auto state = SpatialState("state");
  auto state_ptr = make_shared_state(state);
  ASSERT_NE(state_ptr, nullptr);
  EXPECT_EQ(state_ptr->get_type(), state.get_type());
  EXPECT_EQ(state_ptr->get_name(), state.get_name());
  EXPECT_EQ(std::dynamic_pointer_cast<SpatialState>(state_ptr)->get_reference_frame(), state.get_reference_frame());
  EXPECT_TRUE(state_ptr->is_empty());
  EXPECT_FALSE(*state_ptr);
}