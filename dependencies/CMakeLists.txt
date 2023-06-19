cmake_minimum_required(VERSION 3.18)
project(control_libraries_dependencies)

# needed to set options before the fact
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
# pinochio and osqp-eigen have conflicting target names
set_property(GLOBAL PROPERTY ALLOW_DUPLICATE_CUSTOM_TARGETS ON)

include(FetchContent)
FetchContent_Declare(
  osqp
  GIT_REPOSITORY https://github.com/oxfordcontrol/osqp
  GIT_TAG        v0.6.2
  OVERRIDE_FIND_PACKAGE
)

set(BUILD_DOCUMENTATION OFF)
set(BUILD_PYTHON_INTERFACE OFF)
FetchContent_Declare(
  pinocchio
  GIT_REPOSITORY https://github.com/stack-of-tasks/pinocchio
  GIT_TAG        v2.6.9
  OVERRIDE_FIND_PACKAGE
)

# FIXME: it would be nicer to have it all in CMakelists.txt but:
#  * `OVERRIDE_FIND_PACKAGE` requires CMake 3.24
#  * `pinochio` doesn't provide an include directory we can easily plug into `target_include_directories`
#  * `osqp::osqp` is somehow not working in OSQP-Eigen
FetchContent_MakeAvailable(osqp pinocchio)
