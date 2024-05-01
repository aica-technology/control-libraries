#include "communication_interfaces/sockets/ISocket.hpp"

#include "communication_interfaces/exceptions/SocketConfigurationException.hpp"

namespace communication_interfaces::sockets {

void ISocket::open() {
  this->on_open();
  this->opened_ = true;
}

bool ISocket::receive_bytes(std::string& buffer) {
  if (!this->opened_) {
    throw exceptions::SocketConfigurationException("Failed to received bytes: socket has not been opened yet");
  }
  return this->on_receive_bytes(buffer);
}

bool ISocket::send_bytes(const std::string& buffer) {
  if (!this->opened_) {
    throw exceptions::SocketConfigurationException("Failed to send bytes: socket has not been opened yet");
  }
  return this->on_send_bytes(buffer);
}

void ISocket::close() {
  this->opened_ = false;
  this->on_close();
}
}// namespace communication_interfaces::sockets
