#include <chrono>
#include <gtest/gtest.h>
#include <string>

#include <state_representation/trajectory/CartesianTrajectory.hpp>
#include <state_representation/trajectory/JointTrajectory.hpp>

#include "clproto.hpp"
#include "include/test_encode_decode.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"

using namespace state_representation;

template<typename T>
static void test_trajectory_equal(const T& send_state, const T& recv_state) {
  EXPECT_EQ(send_state.get_type(), recv_state.get_type());
  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  if constexpr (std::is_same_v<T, CartesianTrajectory>) {
    EXPECT_STREQ(send_state.get_reference_frame().c_str(), recv_state.get_reference_frame().c_str());
  } else {
    EXPECT_EQ(send_state.get_joint_names(), recv_state.get_joint_names());
  }
  ASSERT_EQ(send_state.get_size(), recv_state.get_size());
  for (unsigned int i = 0; i < send_state.get_size(); ++i) {
    EXPECT_EQ(send_state.get_point(i).data(), recv_state.get_point(i).data());
    EXPECT_EQ(send_state.get_duration(i), recv_state.get_duration(i));
  }
}

template<typename T>
static void encode_decode_trajectory(T send_state, clproto::MessageType type) {
  clproto::test_encode_decode<T>(send_state, type, test_trajectory_equal<T>);
  send_state.reset();
  clproto::test_encode_decode<T>(send_state, type, test_trajectory_equal<T>);
}

TEST(CartesianProtoTest, EncodeDecodeTrajectory) {
  std::vector<std::string> jnames;
  for (unsigned int i = 0; i < 10; ++i) {
    jnames.push_back("joint_" + std::to_string(i));
  }

  CartesianTrajectory ct("foo", "reference");
  JointTrajectory jt("foo");
  jt.set_joint_names(jnames);
  for (unsigned int i = 0; i < 10; ++i) {
    ct.add_point(
        CartesianState::Random("foo" + std::to_string(i), "reference"), std::chrono::nanoseconds((i + 1) * 10));
    jt.add_point(JointState::Random("foo" + std::to_string(i), jnames), std::chrono::nanoseconds((i + 1) * 10));
  }

  encode_decode_trajectory(ct, clproto::CARTESIAN_TRAJECTORY_MESSAGE);
  encode_decode_trajectory(jt, clproto::JOINT_TRAJECTORY_MESSAGE);
}
