cmake_minimum_required(VERSION 3.24)
project(all_control_libraries)

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

# wget -c "https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_VERSION}/eigen-${EIGEN_VERSION}.tar.gz" -O - | tar -xz || exit 1
# cd "eigen-${EIGEN_VERSION}" && mkdir -p build && cd build && env CXXFLAGS=-DEIGEN_MPL2_ONLY cmake .. && make install || exit 1

add_subdirectory(source)
add_subdirectory(protocol/clproto_cpp)

if(BUILD_TESTING)
  # reexport the test target defined in the subdirectories
  enable_testing()
endif()
