#pragma once

#include <zmq.hpp>

#include "communication_interfaces/sockets/ISocket.hpp"

namespace communication_interfaces::sockets {

/**
 * @struct ZMQSocketConfiguration
 * @brief Configuration parameters for a ZMQ socket
 */
struct ZMQSocketConfiguration {
  std::shared_ptr<zmq::context_t> context;
  std::string ip_address;
  std::string port;
  bool bind = true;
  bool wait = false;
};

/**
 * @class ZMQSocket
 * @brief Abstract class to define a generic ZMQ socket
 */
class ZMQSocket : public ISocket {
public:
  /**
   * @brief Close the socket by calling ZMQSocket::close()
   */
  ~ZMQSocket() override;

protected:
  /**
   * @brief Constructor taking the configuration struct
   * @param The configuration struct
   */
  ZMQSocket(ZMQSocketConfiguration configuration);

  /**
   * @brief Bind or connect the socket on the desired IP/port
   */
  void open_socket();

  ZMQSocketConfiguration config_; ///< Socket configuration struct
  std::shared_ptr<zmq::socket_t> socket_; ///< ZMQ socket

private:
  /**
   * @copydoc ISocket::on_receive_bytes(std::string&)
   */
  bool on_receive_bytes(std::string& buffer) override;

  /**
   * @copydoc ISocket::on_send_bytes(const std::string&)
   */
  bool on_send_bytes(const std::string& buffer) override;

  /**
   * @copydoc ISocket::on_close()
   */
  void on_close() override;
};
} // namespace communication_interfaces::sockets
