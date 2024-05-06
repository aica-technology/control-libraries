#include "communication_interfaces/sockets/UDPServer.hpp"

namespace communication_interfaces::sockets {

UDPServer::UDPServer(UDPSocketConfiguration configuration) : UDPSocket(std::move(configuration)) {}

void UDPServer::on_open() {
  this->open_socket(true);
}

bool UDPServer::on_receive_bytes(std::string& buffer) {
  return this->recvfrom(this->client_address_, buffer);
}

bool UDPServer::on_send_bytes(const std::string& buffer) {
  return this->sendto(this->client_address_, buffer);
}
} // namespace communication_interfaces::sockets
