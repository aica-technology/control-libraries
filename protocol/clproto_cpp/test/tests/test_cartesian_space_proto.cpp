#include <gtest/gtest.h>

#include <state_representation/space/SpatialState.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianTwist.hpp>
#include <state_representation/space/cartesian/CartesianAcceleration.hpp>
#include <state_representation/space/cartesian/CartesianWrench.hpp>

#include "clproto.h"
#include "test_clproto/encode_decode_helper.hpp"

using namespace state_representation;

template<typename T>
static void test_cart_state_equal(const T& send_state, const T& recv_state) {
  EXPECT_EQ(send_state.get_type(), recv_state.get_type());
  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  EXPECT_STREQ(send_state.get_reference_frame().c_str(), recv_state.get_reference_frame().c_str());
  if (send_state) {
    EXPECT_NEAR(send_state.dist(recv_state), 0, 1e-5);
  }
}

template<typename T>
static void encode_decode_cartesian(T send_state, clproto::MessageType type) {
  clproto::test_encode_decode<T>(send_state, type, test_cart_state_equal<T>);
  send_state.reset();
  clproto::test_encode_decode<T>(send_state, type, test_cart_state_equal<T>);
}

TEST(CartesianProtoTest, EncodeDecodeSpatialState) {
  auto send_state = SpatialState("A", "B");
  clproto::test_encode_decode<SpatialState>(
      send_state, clproto::SPATIAL_STATE_MESSAGE, [](
          const SpatialState& send, const SpatialState& recv
      ) {
        EXPECT_EQ(send.get_type(), recv.get_type());
        EXPECT_STREQ(send.get_name().c_str(), recv.get_name().c_str());
        EXPECT_STREQ(send.get_reference_frame().c_str(), recv.get_reference_frame().c_str());
      }
  );
}

TEST(CartesianProtoTest, EncodeDecodeCartesian) {
  encode_decode_cartesian(CartesianState::Random("A", "B"), clproto::CARTESIAN_STATE_MESSAGE);
  encode_decode_cartesian(CartesianPose::Random("A", "B"), clproto::CARTESIAN_POSE_MESSAGE);
  encode_decode_cartesian(CartesianTwist::Random("A", "B"), clproto::CARTESIAN_TWIST_MESSAGE);
  encode_decode_cartesian(CartesianAcceleration::Random("A", "B"), clproto::CARTESIAN_ACCELERATION_MESSAGE);
  encode_decode_cartesian(CartesianWrench::Random("A", "B"), clproto::CARTESIAN_WRENCH_MESSAGE);
}
