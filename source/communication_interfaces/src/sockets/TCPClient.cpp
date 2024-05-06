#include "communication_interfaces/sockets/TCPClient.hpp"

#include <cstring>

#include "communication_interfaces/exceptions/SocketConfigurationException.hpp"

namespace communication_interfaces::sockets {

TCPClient::TCPClient(TCPClientConfiguration configuration) : TCPSocket(configuration.buffer_size), config_(configuration) {}

void TCPClient::on_open() {
  try {
    bzero((char*) &this->server_address_, sizeof(this->server_address_));
    this->server_address_.sin_family = AF_INET;
    this->server_address_.sin_port = htons(this->config_.port);
    if (inet_pton(AF_INET, this->config_.ip_address.c_str(), &this->server_address_.sin_addr) <= 0) {
      throw std::invalid_argument("IP Address not supported");
    }
  } catch (const std::exception& ex) {
    throw exceptions::SocketConfigurationException("Socket configuration failed: " + std::string(ex.what()));
  }

  this->socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (this->socket_fd_ < 0) {
    throw exceptions::SocketConfigurationException("Opening socket failed");
  }

  if (::connect(this->socket_fd_, (sockaddr*) &this->server_address_, sizeof(this->server_address_)) != 0) {
    throw exceptions::SocketConfigurationException("Connecting client failed");
  }
}
} // namespace communication_interfaces::sockets
