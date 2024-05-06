#include "communication_interfaces/sockets/ZMQPublisherSubscriber.hpp"

namespace communication_interfaces::sockets {

ZMQPublisherSubscriber::ZMQPublisherSubscriber(ZMQCombinedSocketsConfiguration configuration) {
  this->pub_ = std::make_shared<ZMQPublisher>(
      ZMQSocketConfiguration(
          {
              configuration.context, configuration.ip_address, configuration.publisher_port,
              configuration.bind_publisher, configuration.wait
          }));
  this->sub_ = std::make_shared<ZMQSubscriber>(
      ZMQSocketConfiguration(
          {
              configuration.context, configuration.ip_address, configuration.subscriber_port,
              configuration.bind_subscriber, configuration.wait
          }));
}

ZMQPublisherSubscriber::~ZMQPublisherSubscriber() {
  ZMQPublisherSubscriber::close();
}

void ZMQPublisherSubscriber::on_open() {
  this->pub_->open();
  this->sub_->open();
}

bool ZMQPublisherSubscriber::on_receive_bytes(std::string& buffer) {
  return this->sub_->receive_bytes(buffer);
}

bool ZMQPublisherSubscriber::on_send_bytes(const std::string& buffer) {
  return this->pub_->send_bytes(buffer);
}

void ZMQPublisherSubscriber::on_close() {
  this->pub_->close();
  this->sub_->close();
}
} // namespace communication_interfaces::sockets
