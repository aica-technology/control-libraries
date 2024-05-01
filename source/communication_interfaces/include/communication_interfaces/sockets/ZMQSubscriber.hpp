#pragma once

#include "communication_interfaces/sockets/ZMQSocket.hpp"

namespace communication_interfaces::sockets {

/**
 * @class ZMQSubscriber
 * @brief Class to define a ZMQ subscriber
 */
class ZMQSubscriber : public ZMQSocket {
public:
  /**
   * @copydoc ZMQSocket::ZMQSocket(ZMQSocketConfiguration)
   */
  explicit ZMQSubscriber(ZMQSocketConfiguration configuration);

  /**
   * @copydoc ISocket::on_open()
   */
  void on_open() override;

  /**
   * @brief This method throws a runtime error as sending is not available for a ZMQ publisher
   */
  bool on_send_bytes(const std::string& buffer) override;
};
} // namespace communication_interfaces::sockets
