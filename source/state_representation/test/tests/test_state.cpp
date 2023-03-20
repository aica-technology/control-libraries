#include <chrono>
#include <gtest/gtest.h>
#include <thread>

#include "state_representation/State.hpp"

using namespace state_representation;

TEST(StateTest, Constructors) {
  State empty1;
  EXPECT_EQ(empty1.get_type(), StateType::STATE);
  EXPECT_EQ(empty1.get_name(), "");
  EXPECT_TRUE(empty1.is_empty());

  State empty2("test");
  EXPECT_EQ(empty2.get_type(), StateType::STATE);
  EXPECT_EQ(empty2.get_name(), "test");
  EXPECT_TRUE(empty2.is_empty());

  State state(empty2);
  EXPECT_EQ(state.get_type(), StateType::STATE);
  EXPECT_EQ(state.get_name(), "test");
  EXPECT_TRUE(state.is_empty());
}

TEST(StateTest, Compatibility) {
  State state1;
  state1.set_name("test");
  EXPECT_EQ(state1.get_name(), "test");

  State state2("test");
  EXPECT_FALSE(state1.is_incompatible(state2));

  state2.reset();
  EXPECT_TRUE(state2.is_empty());
}

TEST(StateTest, Timestamp) {
  State state("test");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_TRUE(state.is_deprecated(std::chrono::milliseconds(100)));
  EXPECT_TRUE(state.is_deprecated(0.1));
  state.reset_timestamp();
  EXPECT_FALSE(state.is_deprecated(std::chrono::milliseconds(100)));
  EXPECT_FALSE(state.is_deprecated(0.1));
  EXPECT_TRUE(state.get_age() < 0.1);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_TRUE(state.is_deprecated(std::chrono::milliseconds(100)));
  EXPECT_TRUE(state.is_deprecated(0.1));
  state.reset_timestamp();
  EXPECT_FALSE(state.is_deprecated(std::chrono::milliseconds(100)));
  EXPECT_FALSE(state.is_deprecated(0.1));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_TRUE(state.get_age() > 0.2);
}

TEST(StateTest, Swap) {
  State state1("test");
  State state2("state");
  swap(state1, state2);
  EXPECT_EQ(state1.get_type(), StateType::STATE);
  EXPECT_EQ(state1.get_name(), "state");
  EXPECT_TRUE(state1.is_empty());
  EXPECT_EQ(state2.get_type(), StateType::STATE);
  EXPECT_EQ(state2.get_name(), "test");
  EXPECT_TRUE(state2.is_empty());
}

TEST(StateTest, Truthiness) {
  State empty;
  EXPECT_TRUE(empty.is_empty());
  EXPECT_FALSE(empty);
}