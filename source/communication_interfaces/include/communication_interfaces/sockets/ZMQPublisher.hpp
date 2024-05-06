#pragma once

#include "communication_interfaces/sockets/ZMQSocket.hpp"

namespace communication_interfaces::sockets {

/**
 * @class ZMQPublisher
 * @brief Class to define a ZMQ publisher
 */
class ZMQPublisher : public ZMQSocket {
public:
  /**
   * @copydoc ZMQSocket::ZMQSocket(ZMQSocketConfiguration)
   */
  explicit ZMQPublisher(ZMQSocketConfiguration configuration);

private:
  /**
   * @copydoc ISocket::on_open()
   */
  void on_open() override;

  /**
   * @brief This method throws a runtime error as receiving is not available for a ZMQ publisher
   */
  bool on_receive_bytes(std::string& buffer) override;
};
} // namespace communication_interfaces::sockets
