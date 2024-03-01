#include <gtest/gtest.h>

#include <state_representation/State.hpp>
#include <state_representation/AnalogIOState.hpp>
#include <state_representation/DigitalIOState.hpp>
#include <state_representation/geometry/Ellipsoid.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/cartesian/CartesianPose.hpp>

#include "clproto.hpp"
#include "test_encode_decode.hpp"

using namespace state_representation;

template<class IOT>
static void test_io_equal(const IOT& send_state, const IOT& recv_state) {
  EXPECT_EQ(send_state.get_type(), recv_state.get_type());
  EXPECT_STREQ(send_state.get_name().c_str(), recv_state.get_name().c_str());
  ASSERT_EQ(send_state.get_size(), recv_state.get_size());
  for (std::size_t ind = 0; ind < send_state.get_size(); ++ind) {
    EXPECT_STREQ(send_state.get_names().at(ind).c_str(), recv_state.get_names().at(ind).c_str());
  }
  if (send_state) {
    EXPECT_TRUE(send_state.data().isApprox(recv_state.data()));
  }
}

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

TEST(MessageProtoTest, EncodeDecodeRandomIO) {
  auto analog_state = AnalogIOState::Random("test", {"one", "two", "three"});
  clproto::test_encode_decode<AnalogIOState>(
      analog_state, clproto::ANALOG_IO_STATE_MESSAGE, test_io_equal<AnalogIOState>);
  analog_state.reset();
  clproto::test_encode_decode<AnalogIOState>(
      analog_state, clproto::ANALOG_IO_STATE_MESSAGE, test_io_equal<AnalogIOState>);

  auto digital_state = DigitalIOState::Random("test", {"one", "two", "three"});
  clproto::test_encode_decode<DigitalIOState>(
      digital_state, clproto::DIGITAL_IO_STATE_MESSAGE, test_io_equal<DigitalIOState>);
  digital_state.reset();
  clproto::test_encode_decode<DigitalIOState>(
      digital_state, clproto::DIGITAL_IO_STATE_MESSAGE, test_io_equal<DigitalIOState>);
}

/* If an encode / decode template is invoked that is not implemented in clproto,
 * there will be a linker error "undefined reference" at compile time.
 * Of course, it's not really possible to test this at run-time.
PSEUDO_TEST(CartesianProtoTest, EncodeInvalidObject) {
  foo::Object invalid_object;

  EXPECT_LINKER_ERROR(clproto::encode(invalid_object));
}
*/