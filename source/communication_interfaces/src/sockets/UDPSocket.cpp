#include "communication_interfaces/sockets/UDPSocket.hpp"

#include <cmath>
#include <sys/socket.h>
#include <vector>
#include <unistd.h>

#include "communication_interfaces/exceptions/SocketConfigurationException.hpp"

namespace communication_interfaces::sockets {

UDPSocket::UDPSocket(UDPSocketConfiguration configuration) :
    server_address_(), config_(std::move(configuration)), server_fd_(), addr_len_() {
  if (this->config_.buffer_size <= 0) {
    throw exceptions::SocketConfigurationException("Configuration parameter 'buffer_size' has to be greater than 0.");
  }
}

UDPSocket::~UDPSocket() {
  UDPSocket::on_close();
}

void UDPSocket::open_socket(bool bind_socket) {
  try {
    this->addr_len_ = sizeof(this->server_address_);
    this->server_address_.sin_family = AF_INET;
    this->server_address_.sin_addr.s_addr = inet_addr(this->config_.ip_address.c_str());
    this->server_address_.sin_port = htons(this->config_.port);
  } catch (const std::exception& ex) {
    throw exceptions::SocketConfigurationException("Socket configuration failed: " + std::string(ex.what()));
  }

  this->server_fd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (this->server_fd_ < 0) {
    throw exceptions::SocketConfigurationException("Opening socket failed");
  }
  if (this->config_.enable_reuse) {
    const int opt_reuse = 1;
    if (setsockopt(this->server_fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt_reuse, sizeof(opt_reuse)) != 0) {
      throw exceptions::SocketConfigurationException("Setting socket option (enable reuse) failed");
    }
  }
  if (bind_socket) {
    if (bind(this->server_fd_, (sockaddr*) &(this->server_address_), sizeof(server_address_)) != 0) {
      throw exceptions::SocketConfigurationException("Binding socket failed.");
    }
  }

  if (this->config_.timeout_duration_sec > 0.0
      && this->config_.timeout_duration_sec < std::numeric_limits<double>::max()) {
    timeval timeout{};
    auto secs = std::floor(this->config_.timeout_duration_sec);
    timeout.tv_sec = static_cast<long>(secs);
    timeout.tv_usec = static_cast<long>((this->config_.timeout_duration_sec - secs) * 1e6);
    if (setsockopt(this->server_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) != 0) {
      throw exceptions::SocketConfigurationException("Setting socket timeout failed.");
    }
  }
}

bool UDPSocket::recvfrom(sockaddr_in& address, std::string& buffer) {
  std::vector<char> local_buffer(this->config_.buffer_size);
  auto receive_length = ::recvfrom(
      this->server_fd_, local_buffer.data(), this->config_.buffer_size, 0, (sockaddr*) &(address), &(this->addr_len_));
  if (receive_length < 0) {
    return false;
  }
  buffer.assign(local_buffer.data(), local_buffer.size());
  return true;
}

bool UDPSocket::sendto(const sockaddr_in& address, const std::string& buffer) const {
  int send_length = ::sendto(
      this->server_fd_, buffer.data(), buffer.size(), 0, (sockaddr*) &(address), this->addr_len_);
  return send_length == static_cast<int>(buffer.size());
}

void UDPSocket::on_close() {
  if (this->server_fd_ >= 0) {
    ::close(this->server_fd_);
    this->server_fd_ = -1;
  }
}
} // namespace communication_interfaces::sockets
