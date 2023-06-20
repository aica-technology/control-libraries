# FIXME: it would be nicer to have it all in the root CMakelists.txt but:
#  * `OVERRIDE_FIND_PACKAGE` requires CMake 3.24
#  * some `protobuf` includes don't work when using `FetchContent`
cmake_minimum_required(VERSION 3.15)
project(control-libraries-dependencies)

include(FetchContent)
FetchContent_Declare(
  OsqpEigen
  GIT_REPOSITORY https://github.com/robotology/osqp-eigen.git
  GIT_TAG        v0.6.4
  OVERRIDE_FIND_PACKAGE
)

FetchContent_Declare(
  protobuf
  GIT_REPOSITORY https://github.com/protocolbuffers/protobuf.git
  GIT_TAG        v3.17.0
  SOURCE_SUBDIR  cmake
  OVERRIDE_FIND_PACKAGE
)

FetchContent_MakeAvailable(OsqpEigen protobuf)
