#include "communication_interfaces/sockets/TCPServer.hpp"

#include <cstring>
#include <unistd.h>

#include "communication_interfaces/exceptions/SocketConfigurationException.hpp"

namespace communication_interfaces::sockets {

TCPServer::TCPServer(TCPServerConfiguration configuration) :
    TCPSocket(configuration.buffer_size), config_(configuration), server_fd_() {
}

TCPServer::~TCPServer() {
  TCPServer::on_close();
}

void TCPServer::on_open() {
  try {
    bzero((char*) &this->server_address_, sizeof(this->server_address_));
    this->server_address_.sin_family = AF_INET;
    this->server_address_.sin_addr.s_addr = htonl(INADDR_ANY);
    this->server_address_.sin_port = htons(this->config_.port);
  } catch (const std::exception& ex) {
    throw exceptions::SocketConfigurationException("Socket configuration failed: " + std::string(ex.what()));
  }

  // open stream oriented socket with internet address
  this->server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (this->server_fd_ < 0) {
    throw exceptions::SocketConfigurationException("Opening socket failed");
  }
  if (this->config_.enable_reuse) {
    const int opt_reuse = 1;
    if (setsockopt(this->server_fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt_reuse, sizeof(opt_reuse)) != 0) {
      throw exceptions::SocketConfigurationException("Setting socket option (enable reuse) failed");
    }
  }
  if (bind(this->server_fd_, (sockaddr*) &(this->server_address_), sizeof(this->server_address_)) != 0) {
    throw exceptions::SocketConfigurationException("Binding socket failed");
  }
  // listen for only 1 request at a time
  listen(this->server_fd_, 1);
  // receive a request from client using accept
  sockaddr_in new_socket_address{};
  socklen_t new_addr_len = sizeof(new_socket_address);
  // accept, create a new socket descriptor to handle the new connection with client
  this->socket_fd_ = accept(this->server_fd_, (sockaddr*) &new_socket_address, &new_addr_len);
  if (this->socket_fd_ < 0) {
    throw exceptions::SocketConfigurationException("Connecting server failed");
  }
}

void TCPServer::on_close() {
  ::close(this->server_fd_);
  TCPSocket::on_close();
}
} // namespace communication_interfaces::sockets
