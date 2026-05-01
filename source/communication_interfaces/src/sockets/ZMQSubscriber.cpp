#include "communication_interfaces/sockets/ZMQSubscriber.hpp"

#include "communication_interfaces/exceptions.hpp"

namespace communication_interfaces::sockets {

ZMQSubscriber::ZMQSubscriber(ZMQSocketConfiguration configuration) : ZMQSocket(std::move(configuration)) {}

void ZMQSubscriber::on_open() {
  this->socket_ = std::make_shared<zmq::socket_t>(*this->config_.context, ZMQ_SUB);
  this->socket_->set(zmq::sockopt::conflate, 1);
  this->socket_->set(zmq::sockopt::subscribe, "");
  this->open_socket();
}

bool ZMQSubscriber::on_send_bytes(const std::string&) {
  throw exceptions::SocketException("Send not available for socket of type ZMQSubscriber");
}
}// namespace communication_interfaces::sockets
