#include <gtest/gtest.h>
#include <thread>

#include "communication_interfaces/sockets/TCPClient.hpp"
#include "communication_interfaces/sockets/TCPServer.hpp"

using namespace communication_interfaces;

class TestTCPSockets : public ::testing::Test {
public:
  TestTCPSockets() {
    server_ = std::make_shared<sockets::TCPServer>(sockets::TCPServerConfiguration{6000, 50, true});
    client_ = std::make_shared<sockets::TCPClient>(sockets::TCPClientConfiguration{"127.0.0.1", 6000, 50});
  }

  std::thread start_server() {
    return std::thread([this] { this->serve(); });
  }

  void serve() const {
    server_->open();
    std::string recv_message;
    EXPECT_TRUE(server_->receive_bytes(recv_message));
    EXPECT_STREQ(recv_message.c_str(), client_message_.c_str());
    EXPECT_TRUE(server_->send_bytes(server_message_));
  }

  std::thread start_client() {
    return std::thread([this] { this->client(); });
  }

  void client() const {
    client_->open();
    EXPECT_TRUE(client_->send_bytes(client_message_));
    std::string recv_message;
    EXPECT_TRUE(client_->receive_bytes(recv_message));
    EXPECT_STREQ(recv_message.c_str(), server_message_.c_str());
  }

  std::string client_message_ = "Hello server";
  std::string server_message_ = "Hello client";

  std::shared_ptr<sockets::TCPClient> client_;
  std::shared_ptr<sockets::TCPServer> server_;
};

TEST_F(TestTCPSockets, TestCommunication) {
  std::thread server = start_server();
  usleep(100000);
  std::thread client = start_client();

  client.join();
  server.join();

  std::string buffer;
  this->server_->close();
  EXPECT_THROW(this->server_->receive_bytes(buffer), exceptions::SocketConfigurationException);
  EXPECT_THROW(this->server_->send_bytes(buffer), exceptions::SocketConfigurationException);
  this->client_->close();
  EXPECT_THROW(this->client_->receive_bytes(buffer), exceptions::SocketConfigurationException);
  EXPECT_THROW(this->client_->send_bytes(buffer), exceptions::SocketConfigurationException);
}

TEST_F(TestTCPSockets, TestNotOpen) {
  std::string buffer;
  
  EXPECT_THROW(this->server_->receive_bytes(buffer), exceptions::SocketConfigurationException);
  EXPECT_THROW(this->server_->send_bytes(buffer), exceptions::SocketConfigurationException);
  
  EXPECT_THROW(this->client_->receive_bytes(buffer), exceptions::SocketConfigurationException);
  EXPECT_THROW(this->client_->send_bytes(buffer), exceptions::SocketConfigurationException);
}
