#include "robot_model_bindings.hpp"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

PYBIND11_MODULE(robot_model, m) {
  m.doc() = "Python bindings for control libraries robot_model";

  #ifdef MODULE_VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(MODULE_VERSION_INFO);
  #else
  m.attr("__version__") = "dev";
  #endif

  auto m_sub = m.def_submodule("exceptions", "Submodule for custom robot model exceptions");
  bind_exceptions(m_sub);
  bind_model(m);
}