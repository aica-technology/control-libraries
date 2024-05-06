#include <gtest/gtest.h>
#include <thread>

#include "communication_interfaces/sockets/ZMQPublisher.hpp"
#include "communication_interfaces/sockets/ZMQSubscriber.hpp"
#include "communication_interfaces/sockets/ZMQPublisherSubscriber.hpp"

using namespace communication_interfaces;
using namespace std::chrono_literals;

class TestZMQSockets : public ::testing::Test {
public:
  TestZMQSockets() {
    auto context = std::make_shared<zmq::context_t>(1);
    config_ = {context, "127.0.0.1", "4000"};
  }

  sockets::ZMQSocketConfiguration config_;
};

TEST_F(TestZMQSockets, SendReceive) {
  const std::string send_string = "Hello world!";
  std::string receive_string;

  sockets::ZMQPublisher publisher(this->config_);
  this->config_.bind = false;
  sockets::ZMQSubscriber subscriber(this->config_);

  publisher.open();
  subscriber.open();

  while (!subscriber.receive_bytes(receive_string)) {
    EXPECT_TRUE(publisher.send_bytes(send_string));
    usleep(10000);
  }
  EXPECT_STREQ(receive_string.c_str(), send_string.c_str());
  publisher.close();
  subscriber.close();
}

TEST_F(TestZMQSockets, SendReceiveCombined) {
  const std::string server_send_string = "Hello client!";
  const std::string client_send_string = "Hello server!";
  std::string server_receive_string, client_receive_string;

  sockets::ZMQCombinedSocketsConfiguration server_config = {config_.context, config_.ip_address, "5001", "5002"};
  sockets::ZMQPublisherSubscriber server(server_config);

  sockets::ZMQCombinedSocketsConfiguration
      client_config = {config_.context, config_.ip_address, "5002", "5001", false, false};
  sockets::ZMQPublisherSubscriber client(client_config);

  server.open();
  client.open();

  while (!client.receive_bytes(client_receive_string)) {
    EXPECT_TRUE(server.send_bytes(server_send_string));
    usleep(10000);
  }
  EXPECT_STREQ(client_receive_string.c_str(), server_send_string.c_str());

  while (!server.receive_bytes(server_receive_string)) {
    EXPECT_TRUE(client.send_bytes(client_send_string));
    usleep(10000);
  }
  EXPECT_STREQ(server_receive_string.c_str(), client_send_string.c_str());

  server.close();
  client.close();
}

TEST_F(TestZMQSockets, TestOpenClose) {
  std::string buffer;
  sockets::ZMQPublisher socket(this->config_);
  EXPECT_THROW(socket.send_bytes(buffer), exceptions::SocketConfigurationException);
  EXPECT_THROW(socket.receive_bytes(buffer), std::runtime_error);

  socket.open();
  EXPECT_TRUE(socket.send_bytes(std::string("test")));
  socket.close();
  EXPECT_THROW(socket.send_bytes(buffer), exceptions::SocketConfigurationException);
}
