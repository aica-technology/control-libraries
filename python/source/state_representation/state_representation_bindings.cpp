#include "state_representation_bindings.hpp"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

PYBIND11_MODULE(state_representation, m) {
  m.doc() = "Python bindings for control libraries state_representation";

  #ifdef MODULE_VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(MODULE_VERSION_INFO);
  #else
  m.attr("__version__") = "dev";
  #endif

  auto m_sub = m.def_submodule("exceptions", "Submodule for custom state representation exceptions");
  bind_exceptions(m_sub);
  bind_state(m);
  bind_cartesian_space(m);
  bind_joint_space(m);
  bind_jacobian(m);
  bind_parameters(m);
  bind_geometry(m);
  bind_io_state(m);
}
