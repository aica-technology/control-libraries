#pragma once

#include "communication_interfaces/sockets/UDPSocket.hpp"

namespace communication_interfaces::sockets {

/**
 * @class UDPClient
 * @brief Class to define a UDP client
 */
class UDPClient : public UDPSocket {
public:
  /**
   * @copydoc UDPSocket::UDPSocket(UDPSocketConfiguration)
   */
  UDPClient(UDPSocketConfiguration configuration);

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
};
} // namespace communication_interfaces::sockets
