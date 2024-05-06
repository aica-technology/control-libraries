#include "communication_interfaces/sockets/TCPSocket.hpp"

#include <unistd.h>
#include <vector>

#include "communication_interfaces/exceptions/SocketConfigurationException.hpp"

namespace communication_interfaces::sockets {

TCPSocket::TCPSocket(int buffer_size) : server_address_(), socket_fd_(), buffer_size_(buffer_size) {
  if (buffer_size <= 0) {
    throw exceptions::SocketConfigurationException("Configuration parameter 'buffer_size' has to be greater than 0.");
  }
}

TCPSocket::~TCPSocket() {
  TCPSocket::on_close();
}

bool TCPSocket::on_receive_bytes(std::string& buffer) {
  std::vector<char> local_buffer(this->buffer_size_);
  auto receive_length = recv(this->socket_fd_, local_buffer.data(), this->buffer_size_, 0);
  if (receive_length < 0) {
    return false;
  }
  buffer.assign(local_buffer.data(), local_buffer.size());
  return true;
}

bool TCPSocket::on_send_bytes(const std::string& buffer) {
  int send_length = send(this->socket_fd_, buffer.data(), buffer.size(), 0);
  return send_length == static_cast<int>(buffer.size());
}

void TCPSocket::on_close() {
  ::close(this->socket_fd_);
}
} // namespace communication_interfaces::sockets
