#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace pybind11::literals;

void bind_tcp(py::module_& m);
void bind_udp(py::module_& m);
void bind_zmq(py::module_& m);
