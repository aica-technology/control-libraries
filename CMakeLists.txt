cmake_minimum_required(VERSION 3.15)
project(all_control_libraries)

option(BUILD_PROTOCOL "Build and install the protocol" ON)

add_subdirectory(source)
if(BUILD_PROTOCOL)
  add_subdirectory(protocol/clproto_cpp)
endif()

if(BUILD_TESTING)
  # reexport the test target defined in the subdirectories
  enable_testing()
endif()

# add licenses
install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/licenses"
  DESTINATION "share/doc/control-libraries"
)
