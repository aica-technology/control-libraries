#include <chrono>
#include <gtest/gtest.h>
#include <thread>

#include "state_representation/State.hpp"
#include "state_representation/exceptions/NotImplementedException.hpp"
#include "test_state_representation/helpers.hpp"

using namespace state_representation;

TEST(StateTest, Construction) {
  State state1;
  EXPECT_EQ(state1.get_type(), StateType::STATE);
  EXPECT_EQ(state1.get_name(), "");
  EXPECT_TRUE(state1.is_empty());
  EXPECT_FALSE(state1);

  auto state2 = State("test");
  EXPECT_EQ(state2.get_type(), StateType::STATE);
  EXPECT_EQ(state2.get_name(), "test");
  EXPECT_TRUE(state2.is_empty());
  EXPECT_FALSE(state2);

  State copy(state1);
  assert_state_equal(state1, copy);

  copy = state2;
  assert_state_equal(state2, copy);
}

TEST(StateTest, Swap) {
  State state1("test");
  auto state1_copy = state1;
  assert_state_equal(state1, state1_copy);
  State state2("state");
  auto state2_copy = state2;
  assert_state_equal(state2, state2_copy);

  swap(state1, state2);
  assert_state_equal(state1_copy, state2);
  assert_state_equal(state2_copy, state1);
}

TEST(StateTest, Setters) {
  State state;
  state.set_name("state");
  EXPECT_EQ(state.get_name(), "state");

  EXPECT_THROW(state.set_data(Eigen::VectorXd()), exceptions::NotImplementedException);
  EXPECT_THROW(state.set_data(std::vector<double>()), exceptions::NotImplementedException);
  EXPECT_THROW(state.set_data(Eigen::MatrixXd()), exceptions::NotImplementedException);
}

TEST(StateTest, Incompatibility) {
  State state1;
  state1.set_name("state");
  auto state2 = state1;
  EXPECT_FALSE(state1.is_incompatible(state2));
}

TEST(StateTest, Timestamp) {
  State state;
  EXPECT_FALSE(state.is_deprecated(std::chrono::milliseconds(100)));
  EXPECT_FALSE(state.is_deprecated(0.1));
  EXPECT_TRUE(state.get_age() < 0.1);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_TRUE(state.is_deprecated(std::chrono::milliseconds(100)));
  EXPECT_TRUE(state.is_deprecated(0.1));
  EXPECT_TRUE(state.get_age() > 0.2);
  state.reset_timestamp();
  EXPECT_FALSE(state.is_deprecated(std::chrono::milliseconds(100)));
  EXPECT_FALSE(state.is_deprecated(0.1));
  EXPECT_TRUE(state.get_age() < 0.1);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_TRUE(state.is_deprecated(std::chrono::milliseconds(100)));
  EXPECT_TRUE(state.is_deprecated(0.1));
  EXPECT_TRUE(state.get_age() > 0.2);
  state.reset();
  // TODO reset timestamp on reset
//  EXPECT_FALSE(state.is_deprecated(std::chrono::milliseconds(100)));
//  EXPECT_FALSE(state.is_deprecated(0.1));
//  EXPECT_TRUE(state.get_age() < 0.1);
}

TEST(StateTest, MakeShared) {
  auto state = State("state");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  auto state_ptr = make_shared_state(state);
  ASSERT_NE(state_ptr, nullptr);
  EXPECT_EQ(state_ptr->get_type(), state.get_type());
  EXPECT_EQ(state_ptr->get_name(), state.get_name());
  EXPECT_TRUE(state_ptr->is_empty());
  EXPECT_FALSE(*state_ptr);
  EXPECT_NE(state_ptr->get_age(), state.get_age());
}