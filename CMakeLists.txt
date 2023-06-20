cmake_minimum_required(VERSION 3.18)
project(all_control_libraries)

# wget -c "https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_VERSION}/eigen-${EIGEN_VERSION}.tar.gz" -O - | tar -xz || exit 1
# cd "eigen-${EIGEN_VERSION}" && mkdir -p build && cd build && env CXXFLAGS=-DEIGEN_MPL2_ONLY cmake .. && make install || exit 1

add_subdirectory(source)
add_subdirectory(protocol/clproto_cpp)

if(BUILD_TESTING)
  # reexport the test target defined in the subdirectories
  enable_testing()
endif()

# add licenses
install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/licenses"
  DESTINATION "share/doc/control-libraries/licenses"
)
