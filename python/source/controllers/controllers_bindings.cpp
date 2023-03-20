#include "controllers_bindings.hpp"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

PYBIND11_MODULE(controllers, m) {
  m.doc() = "Python bindings for control libraries controllers";

  #ifdef MODULE_VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(MODULE_VERSION_INFO);
  #else
  m.attr("__version__") = "dev";
  #endif

  py::module_::import("state_representation");

  auto m_sub = m.def_submodule("exceptions", "Submodule for custom controllers exceptions");
  bind_exceptions(m_sub);
  bind_controller_type(m);
  bind_computational_space(m);
  bind_cartesian_controllers(m);
  bind_joint_controllers(m);
}
