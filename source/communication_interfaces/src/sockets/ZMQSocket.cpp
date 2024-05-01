#include "communication_interfaces/sockets/ZMQSocket.hpp"

#include "communication_interfaces/exceptions/SocketConfigurationException.hpp"

namespace communication_interfaces::sockets {

ZMQSocket::ZMQSocket(ZMQSocketConfiguration configuration) : config_(std::move(configuration)) {}

ZMQSocket::~ZMQSocket() {
  ZMQSocket::on_close();
}

void ZMQSocket::open_socket() {
  try {
    auto address = "tcp://" + this->config_.ip_address + ":" + this->config_.port;
    if (this->config_.bind) {
      this->socket_->bind(address);
    } else {
      this->socket_->connect(address);
    }
  } catch (const std::exception& ex) {
    throw exceptions::SocketConfigurationException("Socket configuration failed: " + std::string(ex.what()));
  }
}

bool ZMQSocket::on_receive_bytes(std::string& buffer) {
  zmq::recv_flags recv_flag = this->config_.wait ? zmq::recv_flags::none : zmq::recv_flags::dontwait;
  zmq::message_t message;
  try {
    auto received = this->socket_->recv(message, recv_flag);
    if (received.has_value()) {
      buffer = std::string(static_cast<char*>(message.data()), message.size());
    }
    return received.has_value();
  } catch (const zmq::error_t&) {
    return false;
  }
}

bool ZMQSocket::on_send_bytes(const std::string& buffer) {
  zmq::send_flags send_flags = this->config_.wait ? zmq::send_flags::none : zmq::send_flags::dontwait;
  zmq::message_t msg(buffer.size());
  memcpy(msg.data(), buffer.data(), buffer.size());
  try {
    auto sent = this->socket_->send(msg, send_flags);
    if (!sent.has_value()) {
      return false;
    }
    return *sent == buffer.size();
  } catch (const zmq::error_t&) {
    return false;
  }
}

void ZMQSocket::on_close() {
  if (this->socket_ != nullptr && this->socket_->connected()) {
    this->socket_->close();
  }
}
} // namespace communication_interfaces::sockets
