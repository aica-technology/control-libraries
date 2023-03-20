#include <gtest/gtest.h>

#include "state_representation/space/SpatialState.hpp"

using namespace state_representation;

TEST(SpatialStateTest, Constructors) {
  SpatialState state0;
  EXPECT_EQ(state0.get_type(), StateType::SPATIAL_STATE);
  EXPECT_EQ(state0.get_name(), "");
  EXPECT_EQ(state0.get_reference_frame(), "world");
  EXPECT_TRUE(state0.is_empty());
}

TEST(SpatialStateTest, Compatibility) {
  SpatialState state1("test", "robot");
  EXPECT_TRUE(state1.is_incompatible(SpatialState("state")));
  EXPECT_TRUE(state1.is_incompatible(SpatialState("test")));
  EXPECT_FALSE(state1.is_incompatible(SpatialState("state", "robot")));
  EXPECT_FALSE(state1.is_incompatible(SpatialState("state", "test")));
  EXPECT_FALSE(state1.is_incompatible(SpatialState("robot")));
}

TEST(SpatialStateTest, Swap) {
  SpatialState state1("cartesian");
  SpatialState state2("joint", "robot");
  swap(state1, state2);
  EXPECT_EQ(state1.get_name(), "joint");
  EXPECT_EQ(state1.get_reference_frame(), "robot");
  EXPECT_TRUE(state1.is_empty());

  EXPECT_EQ(state2.get_name(), "cartesian");
  EXPECT_EQ(state2.get_reference_frame(), "world");
  EXPECT_TRUE(state2.is_empty());
}