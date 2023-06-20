# FIXME: it would be nicer to have it all in the root CMakelists.txt but:
#  * `OVERRIDE_FIND_PACKAGE` requires CMake 3.24
#  * `osqp::osqp` is somehow not working in OSQP-Eigen when built together
cmake_minimum_required(VERSION 3.15)
project(control-libraries-osqp)

include(FetchContent)
FetchContent_Declare(
  osqp
  GIT_REPOSITORY https://github.com/oxfordcontrol/osqp
  GIT_TAG        v0.6.2
  OVERRIDE_FIND_PACKAGE
)

FetchContent_MakeAvailable(osqp)
