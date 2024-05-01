#include "communication_interfaces_bindings.hpp"

#include <communication_interfaces/exceptions/SocketConfigurationException.hpp>
#include <communication_interfaces/sockets/ISocket.hpp>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

using namespace communication_interfaces;

PYBIND11_MODULE(communication_interfaces, m) {
  m.doc() = "Python bindings for communication interfaces";

#ifdef MODULE_VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(MODULE_VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif

  auto m_sub_ex = m.def_submodule("exceptions", "Submodule for custom communication interfaces exceptions");
  py::register_exception<exceptions::SocketConfigurationException>(
      m_sub_ex, "SocketConfigurationError", PyExc_RuntimeError);

  auto m_sub_sock = m.def_submodule("sockets", "Submodule for communication interfaces sockets");

  py::class_<sockets::ISocket, std::shared_ptr<sockets::ISocket>>(m_sub_sock, "ISocket")
      .def("open", &sockets::ISocket::open, "Perform configuration steps to open the socket for communication")
      .def(
          "receive_bytes",
          [](sockets::ISocket& socket) -> py::object {
            std::string buffer;
            auto res = socket.receive_bytes(buffer);
            if (res) {
              return py::bytes(buffer);
            } else {
              return py::none();
            }
          },
          "Receive bytes from the socket")
      .def("send_bytes", &sockets::ISocket::send_bytes, "Receive bytes from the socket", "buffer"_a)
      .def("close", &sockets::ISocket::close, "Perform steps to disconnect and close the socket communication");

  bind_tcp(m_sub_sock);
  bind_udp(m_sub_sock);
  bind_zmq(m_sub_sock);
}
