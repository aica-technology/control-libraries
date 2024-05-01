#include "communication_interfaces_bindings.hpp"

#include <communication_interfaces/sockets/ZMQPublisherSubscriber.hpp>

using namespace communication_interfaces::sockets;

class PyZMQContext {
public:
  PyZMQContext(int io_threads = 1) { context = std::make_shared<zmq::context_t>(io_threads); }

  std::shared_ptr<zmq::context_t> context;
};

struct PyZMQSocketConfiguration {
  PyZMQContext context;
  std::string ip_address;
  std::string port;
  bool bind = true;
  bool wait = false;
};

struct PyZMQCombinedSocketsConfiguration {
  PyZMQContext context;
  std::string ip_address;
  std::string publisher_port;
  std::string subscriber_port;
  bool bind_publisher = true;
  bool bind_subscriber = true;
  bool wait = false;
};

class PyZMQPublisher : public ZMQPublisher, public std::enable_shared_from_this<PyZMQPublisher> {
public:
  PyZMQPublisher(PyZMQSocketConfiguration config)
      : ZMQPublisher({config.context.context, config.ip_address, config.port, config.bind, config.wait}) {}
};

class PyZMQSubscriber : public ZMQSubscriber, public std::enable_shared_from_this<PyZMQSubscriber> {
public:
  PyZMQSubscriber(PyZMQSocketConfiguration config)
      : ZMQSubscriber({config.context.context, config.ip_address, config.port, config.bind, config.wait}) {}
};

class PyZMQPublisherSubscriber : public ZMQPublisherSubscriber,
                                 public std::enable_shared_from_this<PyZMQPublisherSubscriber> {
public:
  PyZMQPublisherSubscriber(PyZMQCombinedSocketsConfiguration config)
      : ZMQPublisherSubscriber(
          {config.context.context, config.ip_address, config.publisher_port, config.subscriber_port,
           config.bind_publisher, config.bind_subscriber, config.wait}) {}
};

void bind_zmq(py::module_& m) {
  py::class_<PyZMQContext>(m, "ZMQContext").def(py::init<int>(), "Create a ZMQ context", "io_threads"_a = 1);

  py::class_<PyZMQSocketConfiguration>(m, "ZMQSocketConfiguration")
      .def(
          py::init<PyZMQContext, std::string, std::string, bool, bool>(), "ZMQSocketConfiguration struct", "context"_a,
          "ip_address"_a, "port"_a, "bind"_a = true, "wait"_a = false)
      .def_readwrite("ip_address", &PyZMQSocketConfiguration::ip_address)
      .def_readwrite("port", &PyZMQSocketConfiguration::port)
      .def_readwrite("bind", &PyZMQSocketConfiguration::bind)
      .def_readwrite("wait", &PyZMQSocketConfiguration::wait);

  py::class_<PyZMQCombinedSocketsConfiguration>(m, "ZMQCombinedSocketsConfiguration")
      .def(
          py::init<PyZMQContext, std::string, std::string, std::string, bool, bool, bool>(),
          "ZMQCombinedSocketsConfiguration struct", "context"_a, "ip_address"_a, "publisher_port"_a,
          "subscriber_port"_a, "bind_publisher"_a = true, "bind_subscriber"_a = true, "wait"_a = false)
      .def_readwrite("ip_address", &PyZMQCombinedSocketsConfiguration::ip_address)
      .def_readwrite("publisher_port", &PyZMQCombinedSocketsConfiguration::publisher_port)
      .def_readwrite("subscriber_port", &PyZMQCombinedSocketsConfiguration::subscriber_port)
      .def_readwrite("bind_publisher", &PyZMQCombinedSocketsConfiguration::bind_publisher)
      .def_readwrite("bind_subscriber", &PyZMQCombinedSocketsConfiguration::bind_subscriber)
      .def_readwrite("wait", &PyZMQCombinedSocketsConfiguration::wait);

  py::class_<PyZMQPublisher, std::shared_ptr<PyZMQPublisher>, ISocket>(m, "ZMQPublisher")
      .def(py::init<PyZMQSocketConfiguration>(), "Constructor taking the configuration struct", "configuration"_a);

  py::class_<PyZMQSubscriber, std::shared_ptr<PyZMQSubscriber>, ISocket>(m, "ZMQSubscriber")
      .def(py::init<PyZMQSocketConfiguration>(), "Constructor taking the configuration struct", "configuration"_a);

  py::class_<PyZMQPublisherSubscriber, std::shared_ptr<PyZMQPublisherSubscriber>, ISocket>(m, "ZMQPublisherSubscriber")
      .def(
          py::init<PyZMQCombinedSocketsConfiguration>(), "Constructor taking the configuration struct",
          "configuration"_a);
}
