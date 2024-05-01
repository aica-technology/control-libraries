#pragma once

#include "communication_interfaces/sockets/ISocket.hpp"
#include "communication_interfaces/sockets/ZMQPublisher.hpp"
#include "communication_interfaces/sockets/ZMQSubscriber.hpp"

namespace communication_interfaces::sockets {

/**
 * @struct ZMQCombinedSocketsConfiguration
 * @brief Configuration parameters for a the combination of a ZMQ Publisher and Pubscriber socket
 */
struct ZMQCombinedSocketsConfiguration {
  std::shared_ptr<zmq::context_t> context;
  std::string ip_address;
  std::string publisher_port;
  std::string subscriber_port;
  bool bind_publisher = true;
  bool bind_subscriber = true;
  bool wait = false;
};

/**
 * @brief A class that combines both a ZMQ Publisher and Subscriber socket into one single object
 */
class ZMQPublisherSubscriber : public ISocket {
public:
  /**
   * @brief Constructor taking the configuration struct
   * @param The configuration struct
   */
  explicit ZMQPublisherSubscriber(ZMQCombinedSocketsConfiguration configuration);

  /**
   * @brief Close the sockets by calling ZMQPublisherSubscriber::close()
   */
  ~ZMQPublisherSubscriber() override;

private:
  /**
   * @brief Open the internal ZMQ Publisher and Subscriber sockets for communication
   * @throws SocketConfigurationException if opening fails
   */
  void on_open() override;

  /**
   * @brief Receive bytes from the internal ZMQ Subscriber socket
   * @param buffer The buffer to fill with the received bytes
   * @return True if bytes were received, false otherwise
   */
  bool on_receive_bytes(std::string& buffer) override;

  /**
   * @brief Send bytes with the internal ZMQ Publisher socket
   * @param buffer The buffer with the bytes to send
   * @return True if bytes were sent, false otherwise
   */
  bool on_send_bytes(const std::string& buffer) override;

  /**
   * @copydoc ISocket::on_close()
   */
  void on_close() override;

  std::shared_ptr<ZMQPublisher> pub_; ///< ZMQ publisher
  std::shared_ptr<ZMQSubscriber> sub_; ///< ZMQ subscriber
};
} // namespace communication_interfaces::sockets
