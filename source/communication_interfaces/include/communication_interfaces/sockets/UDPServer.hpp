#pragma once

#include "communication_interfaces/sockets/UDPSocket.hpp"

namespace communication_interfaces::sockets {

/**
 * @class UDPServer
 * @brief Class to define a UDP server
 */
class UDPServer : public UDPSocket {
public:
  /**
   * @copydoc UDPSocket::UDPSocket(UDPSocketConfiguration)
   */
  UDPServer(UDPSocketConfiguration configuration);

private:
  /**
   * @copydoc ISocket::on_open()
   */
  void on_open() override;

  /**
   * @copydoc ISocket::on_receive_bytes(std::string&)
   */
  bool on_receive_bytes(std::string& buffer) override;

  /**
   * @copydoc ISocket::on_send_bytes(const std::string&)
   */
  bool on_send_bytes(const std::string& buffer) override;

  sockaddr_in client_address_;
};
} // namespace communication_interfaces::sockets
