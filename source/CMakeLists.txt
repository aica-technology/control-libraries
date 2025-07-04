cmake_minimum_required(VERSION 3.15)

project(control_libraries VERSION 9.2.0)

# Build options
option(BUILD_TESTING "Build all tests." OFF)
option(BUILD_CONTROLLERS "Build and install controllers library" ON)
option(BUILD_DYNAMICAL_SYSTEMS "Build and install dynamical systems library" ON)
option(BUILD_ROBOT_MODEL "Build and install robot model library" ON)
option(BUILD_COMMUNICATION_INTERFACES "Build and install communication interfaces library" ON)
option(EXPERIMENTAL_FEATURES "Include experimental features" OFF)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

macro(add_pkgconfig_library)
  if(PKGCONFIG_LIBRARIES)
    set(PKGCONFIG_LIBRARIES "${PKGCONFIG_LIBRARIES}, ${ARGV0} >= ${ARGV1}")
  else()
    set(PKGCONFIG_LIBRARIES "${ARGV0} >= ${ARGV1}")
  endif()
endmacro()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(FindPkgConfig)

if(NOT ${PKG_CONFIG_FOUND})
  message(WARNING "Could not find pkg-config executable, skipping generation of pkg-config files.")
endif()

if(BUILD_TESTING)
  enable_testing()
  find_package(GTest REQUIRED)
  if (APPLE)
    add_definitions(-DGTEST_USE_OWN_TR1_TUPLE)
    add_definitions(-D__GLIBCXX__)
  endif (APPLE)
else()
  find_package(GTest QUIET)
endif()

if(EXPERIMENTAL_FEATURES)
  add_compile_definitions(EXPERIMENTAL_FEATURES)
endif()

set(INSTALL_SUPPORTED_COMPONENTS)

add_subdirectory(state_representation)
list(APPEND INSTALL_SUPPORTED_COMPONENTS state_representation)
add_pkgconfig_library(state_representation ${PROJECT_VERSION})

if(BUILD_DYNAMICAL_SYSTEMS)
  add_subdirectory(dynamical_systems)
  list(APPEND INSTALL_SUPPORTED_COMPONENTS dynamical_systems)
  add_pkgconfig_library(dynamical_systems ${PROJECT_VERSION})
endif()

if(BUILD_ROBOT_MODEL OR BUILD_CONTROLLERS)
  add_subdirectory(robot_model)
  list(APPEND INSTALL_SUPPORTED_COMPONENTS robot_model)
  add_pkgconfig_library(robot_model ${PROJECT_VERSION})
endif()

if(BUILD_CONTROLLERS)
  add_subdirectory(controllers)
  list(APPEND INSTALL_SUPPORTED_COMPONENTS controllers)
  add_pkgconfig_library(controllers ${PROJECT_VERSION})
endif()

if(BUILD_COMMUNICATION_INTERFACES)
  add_subdirectory(communication_interfaces)
  list(APPEND INSTALL_SUPPORTED_COMPONENTS communication_interfaces)
  add_pkgconfig_library(communication_interfaces ${PROJECT_VERSION})
endif()

# generate the version file for the config file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION "${PROJECT_VERSION}"
  COMPATIBILITY SameMajorVersion
)

# create config file
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/control_librariesConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# install config files
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

if(${PKG_CONFIG_FOUND})
  set(PKG_NAME ${CMAKE_PROJECT_NAME})
  set(PKG_DESC "control_libraries")
  set(PKG_EXTERNAL_DEPS "${PKGCONFIG_LIBRARIES}")
  set(pkg_conf_file "control_libraries.pc")
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/library_template.pc.in" "${CMAKE_CURRENT_BINARY_DIR}/${pkg_conf_file}" @ONLY)
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${pkg_conf_file}"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig/ COMPONENT pkgconfig)
endif()
