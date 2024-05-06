#pragma once

#include <arpa/inet.h>

#include "communication_interfaces/sockets/ISocket.hpp"

namespace communication_interfaces::sockets {

/**
 * @brief Abstract class to define a generic TCP socket
 * @details TCP is a connection-based communication protocol. Hence, TCP sockets need to implement an additional
 * interface method `connect()`.
 */
class TCPSocket : public ISocket {
public:
  /**
   * @brief Close the socket by calling TCPSocket::close()
   */
  ~TCPSocket() override;

protected:
  explicit TCPSocket(int buffer_size);

  /**
   * @copydoc ISocket::on_receive_bytes(std::string&)
   */
  bool on_receive_bytes(std::string& buffer) override;

  /**
   * @copydoc ISocket::on_receive_bytes(std::string&)
   */
  bool on_send_bytes(const std::string& buffer) override;

  /**
   * @copydoc ISocket::on_close(std::string&)
   */
  void on_close() override;

  sockaddr_in server_address_; ///< Address of the TCP server
  int socket_fd_; ///< File descriptor of the socket
  int buffer_size_; ///< Buffer size
};
} // namespace communication_interfaces::sockets
