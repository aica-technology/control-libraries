#pragma once

#include "communication_interfaces/sockets/TCPSocket.hpp"

namespace communication_interfaces::sockets {

/**
 * @struct TCPClientConfiguration
 * @brief Configuration parameters for a TCP client
 */
struct TCPClientConfiguration {
  std::string ip_address;
  int port;
  int buffer_size;
};

class TCPClient : public TCPSocket {
public:
  /**
   * @brief Constructor taking the configuration struct
   * @param The configuration struct
   */
  explicit TCPClient(TCPClientConfiguration configuration);

private:
  /**
   * @copydoc ISocket::on_open()
   * @details Connect the client socket to the server
   */
  void on_open() override;

  TCPClientConfiguration config_; ///< Socket configuration struct
};
} // namespace communication_interfaces::sockets
