#include <gtest/gtest.h>

#include <state_representation/space/Jacobian.hpp>
#include <state_representation/space/joint/JointState.hpp>
#include <state_representation/space/joint/JointPositions.hpp>
#include <state_representation/space/joint/JointVelocities.hpp>
#include <state_representation/space/joint/JointAccelerations.hpp>
#include <state_representation/space/joint/JointTorques.hpp>

#include "clproto.h"
#include "test_clproto/encode_decode_helper.hpp"

using namespace state_representation;

template<typename T>
static void test_joint_state_equal(const T& send_state, const T& recv_state) {
  EXPECT_EQ(send_state.get_type(), recv_state.get_type());
  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  ASSERT_EQ(send_state.get_size(), recv_state.get_size());
  for (std::size_t ind = 0; ind < send_state.get_size(); ++ind) {
    EXPECT_STREQ(send_state.get_names().at(ind).c_str(), recv_state.get_names().at(ind).c_str());
  }
  if (send_state) {
    EXPECT_NEAR(send_state.dist(recv_state), 0, 1e-5);
  }
}

static void test_jacobian_equal(const Jacobian& send_state, const Jacobian& recv_state) {
  EXPECT_EQ(send_state.get_type(), recv_state.get_type());
  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  EXPECT_STREQ(send_state.get_frame().c_str(), recv_state.get_frame().c_str());
  EXPECT_STREQ(send_state.get_reference_frame().c_str(), recv_state.get_reference_frame().c_str());
  EXPECT_EQ(send_state.rows(), recv_state.rows());
  EXPECT_EQ(send_state.cols(), recv_state.cols());
  ASSERT_EQ(send_state.get_joint_names().size(), recv_state.get_joint_names().size());
  for (std::size_t ind = 0; ind < send_state.get_joint_names().size(); ++ind) {
    EXPECT_STREQ(send_state.get_joint_names().at(ind).c_str(), recv_state.get_joint_names().at(ind).c_str());
  }
  if (send_state) {
    EXPECT_NEAR(send_state.data().norm(), recv_state.data().norm(), 1e-5);
  }
}

template<typename T>
static void encode_decode_joint(T send_state, clproto::MessageType type) {
  clproto::test_encode_decode<T>(send_state, type, test_joint_state_equal<T>);
  send_state.reset();
  clproto::test_encode_decode<T>(send_state, type, test_joint_state_equal<T>);
}

TEST(JointProtoTest, EncodeDecodeRandomJoint) {
  encode_decode_joint(JointState::Random("robot", {"one", "two", "three"}), clproto::JOINT_STATE_MESSAGE);
  encode_decode_joint(JointPositions::Random("robot", {"one", "two", "three"}), clproto::JOINT_POSITIONS_MESSAGE);
  encode_decode_joint(JointVelocities::Random("robot", {"one", "two", "three"}), clproto::JOINT_VELOCITIES_MESSAGE);
  encode_decode_joint(JointAccelerations::Random("robot", {"one", "two", "three"}), clproto::JOINT_ACCELERATIONS_MESSAGE);
  encode_decode_joint(JointTorques::Random("robot", {"one", "two", "three"}), clproto::JOINT_TORQUES_MESSAGE);
}

TEST(JointProtoTest, EncodeDecodeJacobian) {
  auto send_state = Jacobian::Random("robot", {"one", "two", "three"}, "A", "B");
  clproto::test_encode_decode<Jacobian>(send_state, clproto::JACOBIAN_MESSAGE, test_jacobian_equal);
  send_state.reset();
  clproto::test_encode_decode<Jacobian>(send_state, clproto::JACOBIAN_MESSAGE, test_jacobian_equal);
}
