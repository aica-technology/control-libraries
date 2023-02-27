#include "dynamical_systems_bindings.hpp"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

PYBIND11_MODULE(dynamical_systems, m) {
  m.doc() = "Python bindings for control libraries dynamical_systems";

  #ifdef MODULE_VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(MODULE_VERSION_INFO);
  #else
  m.attr("__version__") = "dev";
  #endif

  py::module_::import("state_representation");

  auto m_sub = m.def_submodule("exceptions", "Submodule for custom dynamical systems exceptions");
  bind_exceptions(m_sub);
  bind_ds_type(m);
  bind_cartesian_ds(m);
  bind_joint_ds(m);
}
