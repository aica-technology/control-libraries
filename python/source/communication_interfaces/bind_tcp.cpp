#include "communication_interfaces_bindings.hpp"

#include <communication_interfaces/sockets/TCPClient.hpp>
#include <communication_interfaces/sockets/TCPServer.hpp>

using namespace communication_interfaces::sockets;

void bind_tcp(py::module_& m) {
  py::class_<TCPClientConfiguration>(m, "TCPClientConfiguration")
      .def(
          py::init<std::string, int, int>(), "TCPClientConfiguration struct", "ip_address"_a, "port"_a, "buffer_size"_a)
      .def_readwrite("ip_address", &TCPClientConfiguration::ip_address)
      .def_readwrite("port", &TCPClientConfiguration::port)
      .def_readwrite("buffer_size", &TCPClientConfiguration::buffer_size);

  py::class_<TCPClient, std::shared_ptr<TCPClient>, ISocket>(m, "TCPClient")
      .def(py::init<TCPClientConfiguration>(), "Constructor taking the configuration struct", "configuration"_a);

  py::class_<TCPServerConfiguration>(m, "TCPServerConfiguration")
      .def(py::init<int, int, bool>(), "TCPServerConfiguration struct", "port"_a, "buffer_size"_a, "enable_reuse"_a)
      .def_readwrite("port", &TCPServerConfiguration::port)
      .def_readwrite("buffer_size", &TCPServerConfiguration::buffer_size)
      .def_readwrite("enable_reuse", &TCPServerConfiguration::enable_reuse);

  py::class_<TCPServer, std::shared_ptr<TCPServer>, ISocket>(m, "TCPServer")
      .def(py::init<TCPServerConfiguration>(), "Constructor taking the configuration struct", "configuration"_a);
}
