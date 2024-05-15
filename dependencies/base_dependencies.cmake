# FIXME: it would be nicer to have it all in the root CMakelists.txt but:
#  * `OVERRIDE_FIND_PACKAGE` requires CMake 3.24
#  * `osqp::osqp` is somehow not working in OSQP-Eigen when built together
cmake_minimum_required(VERSION 3.15)
project(control-libraries-base-deps)

include(FetchContent)
FetchContent_Declare(
  eigen3
  URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
)

FetchContent_Declare(
  osqp
  GIT_REPOSITORY https://github.com/oxfordcontrol/osqp
  GIT_TAG        v0.6.3
)

FetchContent_Declare(
  Octomap
  GIT_REPOSITORY https://github.com/OctoMap/octomap.git
  GIT_TAG        v1.10.0
  SOURCE_SUBDIR  octomap
)

FetchContent_MakeAvailable(eigen3 osqp octomap)
