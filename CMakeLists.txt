cmake_minimum_required(VERSION 3.18)
project(all_control_libraries)

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
