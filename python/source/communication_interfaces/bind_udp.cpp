#include "communication_interfaces_bindings.hpp"

#include <communication_interfaces/sockets/UDPClient.hpp>
#include <communication_interfaces/sockets/UDPServer.hpp>

using namespace communication_interfaces::sockets;

void bind_udp(py::module_& m) {
  py::class_<UDPSocketConfiguration>(m, "UDPSocketConfiguration")
      .def(
          py::init<std::string, int, int, bool, double>(), "UDPSocketConfiguration struct", "ip_address"_a, "port"_a,
          "buffer_size"_a, "enable_reuse"_a = false, "timeout_duration_sec"_a = 0.0)
      .def_readwrite("ip_address", &UDPSocketConfiguration::ip_address)
      .def_readwrite("port", &UDPSocketConfiguration::port)
      .def_readwrite("buffer_size", &UDPSocketConfiguration::buffer_size)
      .def_readwrite("enable_reuse", &UDPSocketConfiguration::enable_reuse)
      .def_readwrite("timeout_duration_sec", &UDPSocketConfiguration::timeout_duration_sec);

  py::class_<UDPClient, std::shared_ptr<UDPClient>, ISocket>(m, "UDPClient")
      .def(py::init<UDPSocketConfiguration>(), "Constructor taking the configuration struct", "configuration"_a);

  py::class_<UDPServer, std::shared_ptr<UDPServer>, ISocket>(m, "UDPServer")
      .def(py::init<UDPSocketConfiguration>(), "Constructor taking the configuration struct", "configuration"_a);
}
