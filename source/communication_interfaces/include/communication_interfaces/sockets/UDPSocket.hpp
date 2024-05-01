#pragma once

#include <arpa/inet.h>

#include "communication_interfaces/sockets/ISocket.hpp"

namespace communication_interfaces::sockets {

/**
 * @struct UDPSocketConfiguration
 * @brief Configuration parameters for a UDP sockets
 */
struct UDPSocketConfiguration {
  std::string ip_address;
  int port;
  int buffer_size;
  bool enable_reuse = false;
  double timeout_duration_sec = 0.0;
};

/**
 * @class UDPSocket
 * @brief Abstract class to define a generic UDP socket
 */
class UDPSocket : public ISocket {
public:
  /**
   * @brief Close the socket by calling UDPSocket::close()
   */
  ~UDPSocket() override;

protected:
  /**
   * @brief Constructor taking the configuration struct
   * @param The configuration struct
   */
  explicit UDPSocket(UDPSocketConfiguration configuration);

  /**
   * @brief Perform steps to open the socket on the desired IP/port, set reuse and timeout options and bind if desired.
   * @param bind_socket If true, bind the socket (for a UDP server), no binding otherwise (for a UDP client)
   */
  void open_socket(bool bind_socket);

  /**
   * @brief Receive bytes from the socket
   * @param address Reference to a sockaddr_in structure in which the sending address is to be stored
   * @param buffer The buffer to fill with the received bytes
   * @return True if bytes were received, false otherwise
   */
  [[nodiscard]] bool recvfrom(sockaddr_in& address, std::string& buffer);

  /**
   * @brief Send bytes to the socket
   * @param address Reference to a sockaddr_in structure containing the destination address
   * @param buffer The buffer with the bytes to send
   * @return True if bytes were sent, false otherwise
   */
  [[nodiscard]] bool sendto(const sockaddr_in& address, const std::string& buffer) const;

  /**
   * @copydoc ISocket::on_close()
   */
  void on_close() override;

  sockaddr_in server_address_; ///< Address of the UDP server

private:
  UDPSocketConfiguration config_; ///< Socket configuration struct
  int server_fd_; ///< File descriptor of the socket
  socklen_t addr_len_; ///< Length of the socket address
};
} // namespace communication_interfaces::sockets
