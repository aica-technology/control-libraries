#pragma once

#include <gtest/gtest.h>
#include <state_representation/State.hpp>

#include "clproto.hpp"

namespace clproto {

template<typename T>
static void test_encode_decode(
    const T& send_state, clproto::MessageType type, std::function<void(const T&, const T&)> test_func,
    clproto::ParameterMessageType param_type = clproto::ParameterMessageType::UNKNOWN_PARAMETER) {
  std::string msg = clproto::encode(send_state);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_EQ(clproto::check_message_type(msg), type);
  EXPECT_EQ(clproto::check_parameter_message_type(msg), param_type);

  T recv_state;
  EXPECT_NO_THROW(clproto::decode<T>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state));
  EXPECT_EQ(send_state.is_empty(), recv_state.is_empty());

  test_func(send_state, recv_state);

  auto send_state_ptr = make_shared_state(send_state);
  msg = clproto::encode(send_state_ptr);
  EXPECT_TRUE(clproto::is_valid(msg));
  EXPECT_EQ(clproto::check_message_type(msg), type);
  EXPECT_EQ(clproto::check_parameter_message_type(msg), param_type);

  T recv_state_2;
  auto recv_state_ptr = make_shared_state(recv_state_2);
  EXPECT_NO_THROW(clproto::decode<std::shared_ptr<state_representation::State>>(msg));
  EXPECT_TRUE(clproto::decode(msg, recv_state_ptr));

  recv_state_2 = *std::dynamic_pointer_cast<T>(recv_state_ptr);
  test_func(send_state, recv_state_2);
}
}// namespace clproto
