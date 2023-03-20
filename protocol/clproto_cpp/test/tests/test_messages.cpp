#include <gtest/gtest.h>

#include <state_representation/State.hpp>
#include <state_representation/geometry/Ellipsoid.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/cartesian/CartesianPose.hpp>

#include "clproto.hpp"
#include "test_encode_decode.hpp"

using namespace state_representation;

TEST(MessageProtoTest, EncodeDecodeState) {
  auto send_state = State("A");
  clproto::test_encode_decode<State>(
      send_state, clproto::STATE_MESSAGE, [](
          const State& send, const State& recv
      ) {
        EXPECT_EQ(send.get_type(), recv.get_type());
        EXPECT_STREQ(send.get_name().c_str(), recv.get_name().c_str());
      }
  );
}

TEST(MessageProtoTest, EncodeDecodeInvalidState) {
  auto send_state = Ellipsoid("ellipsoid");
  auto send_state_ptr = make_shared_state(send_state);
  EXPECT_THROW(clproto::encode(send_state_ptr), std::invalid_argument);

  auto send_state_2 = State("A");
  std::string msg = clproto::encode(send_state_2);

  Ellipsoid recv_state;
  auto recv_state_ptr = make_shared_state(recv_state);
  EXPECT_FALSE(clproto::decode(msg, recv_state_ptr));
}

TEST(MessageProtoTest, DecodeInvalidString) {
  std::string dummy_msg = "hello world";

  EXPECT_FALSE(clproto::is_valid(dummy_msg));
  EXPECT_EQ(clproto::check_message_type(dummy_msg), clproto::UNKNOWN_MESSAGE);

  State obj;
  EXPECT_NO_THROW(clproto::decode(dummy_msg, obj));
  EXPECT_FALSE(clproto::decode(dummy_msg, obj));

  EXPECT_THROW(clproto::decode<State>(dummy_msg), clproto::DecodingException);
}

TEST(MessageProtoTest, DecodeParallelTypes) {
  auto state = CartesianState::Random("A", "B");
  auto pose = CartesianPose::Random("C", "D");
  auto encoded_state = clproto::encode(state);
  auto encoded_pose = clproto::encode(pose);

  EXPECT_THROW(clproto::decode<CartesianState>(encoded_pose), clproto::DecodingException);
  EXPECT_THROW(clproto::decode<CartesianPose>(encoded_state), clproto::DecodingException);
}

/* If an encode / decode template is invoked that is not implemented in clproto,
 * there will be a linker error "undefined reference" at compile time.
 * Of course, it's not really possible to test this at run-time.
PSEUDO_TEST(CartesianProtoTest, EncodeInvalidObject) {
  foo::Object invalid_object;

  EXPECT_LINKER_ERROR(clproto::encode(invalid_object));
}
*/