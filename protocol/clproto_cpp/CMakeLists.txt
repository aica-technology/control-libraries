cmake_minimum_required(VERSION 3.15)

project(clproto VERSION 7.4.10)

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

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(FindPkgConfig)

if(NOT ${PKG_CONFIG_FOUND})
  message(WARNING "Could not find pkg-config executable, skipping generation of pkg-config files.")
endif()

include(FindProtobuf)
find_package(Protobuf 3.17 REQUIRED)
if (NOT TARGET state_representation)
  # if we can't find the state_representation target, we are the top-level project and need to find it,
  # otherwise, we are a subproject and the state_representation target is already available
  find_package(control_libraries ${PROJECT_VERSION} REQUIRED COMPONENTS state_representation)
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

set(PROTOBUF_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../protobuf)
set(PROTOBUF_BINDINGS_DIR ${PROTOBUF_DIR}/bindings/cpp)

add_custom_target(generate_proto_bindings COMMAND make cpp_bindings
  WORKING_DIRECTORY ${PROTOBUF_DIR}
)

# if we are the top-level project, we assume the protobuf bindings are already built manually by the Makefile,
# otherwise, we built them ourselves in this project
if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  file(GLOB_RECURSE GENERATED_PROTO_BINDINGS "${PROTOBUF_BINDINGS_DIR}/*.pb.cc" "${PROTOBUF_BINDINGS_DIR}/*.pb.h")
else()
  # FIXME: unfortunately, protobuf_generate_* is a bit broken, so we do it by hand
  set(IMPORT_PATH ${PROTOBUF_DIR}/proto)
  file(GLOB_RECURSE PROTO_FILES "${IMPORT_PATH}/*.proto")
  set(PROTOBUF_BINDINGS_DIR ${CMAKE_CURRENT_BINARY_DIR}/bindings)

  set(GENERATED_PROTO_BINDINGS)
  set(PROTO_FILES_TEXT)
  foreach(PROTO_FILE ${PROTO_FILES})
    string(REPLACE "${IMPORT_PATH}/" "" PROTO_FILE_REL "${PROTO_FILE}")
    get_filename_component(PROTO_FILE_PATH ${PROTO_FILE_REL} DIRECTORY)
    get_filename_component(PROTO_FILE_NAME ${PROTO_FILE_REL} NAME_WE)
    list(APPEND GENERATED_PROTO_BINDINGS "${PROTOBUF_BINDINGS_DIR}/${PROTO_FILE_PATH}/${PROTO_FILE_NAME}.pb.cc")
    list(APPEND GENERATED_PROTO_BINDINGS "${PROTOBUF_BINDINGS_DIR}/${PROTO_FILE_PATH}/${PROTO_FILE_NAME}.pb.h")
    list(APPEND PROTO_FILES_TEXT ${PROTO_FILE_REL})
  endforeach()
  string(REPLACE ";" ", " PROTO_FILES_TEXT "${PROTO_FILES_TEXT}")

  make_directory(${PROTOBUF_BINDINGS_DIR})
  add_custom_command(
    OUTPUT ${GENERATED_PROTO_BINDINGS}
    COMMAND  protobuf::protoc
    ARGS --cpp_out ${PROTOBUF_BINDINGS_DIR} -I${IMPORT_PATH} ${PROTO_FILES}
    DEPENDS ${PROTOBUF_BINDINGS_DIR} ${PROTO_FILES} protobuf::protoc
    COMMENT "Running C++ protocol buffer compiler on ${PROTO_FILES_TEXT}"
    VERBATIM
  )
endif()

add_library(${PROJECT_NAME}_bindings STATIC ${GENERATED_PROTO_BINDINGS})
target_include_directories(${PROJECT_NAME}_bindings PUBLIC ${PROTOBUF_BINDINGS_DIR})
set_property(TARGET ${PROJECT_NAME}_bindings PROPERTY POSITION_INDEPENDENT_CODE ON)

add_library(${PROJECT_NAME} SHARED
  ${PROJECT_SOURCE_DIR}/src/clproto.cpp
  ${PROJECT_SOURCE_DIR}/src/decoders.cpp
  ${PROJECT_SOURCE_DIR}/src/encoders.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/include/clproto.hpp)
target_link_libraries(${PROJECT_NAME} PUBLIC protobuf::libprotobuf state_representation PRIVATE ${PROJECT_NAME}_bindings)

# install the target and create export-set
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(TARGETS ${PROJECT_NAME}
  PUBLIC_HEADER DESTINATION include
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# generate and install export file
install(EXPORT ${PROJECT_NAME}_targets
  FILE ${PROJECT_NAME}_targets.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

if (BUILD_TESTING)
  file(GLOB_RECURSE TEST_SOURCES test/ test_*.cpp)
  add_executable(test_${PROJECT_NAME} ${TEST_SOURCES})
  include_directories(test/tests/include)
  target_link_libraries(test_${PROJECT_NAME}
    protobuf
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    pthread
  )
  add_test(NAME test_${PROJECT_NAME} COMMAND test_${PROJECT_NAME})
endif ()

# generate the version file for the config file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION "${PROJECT_VERSION}"
  COMPATIBILITY SameMajorVersion
)

# create config file
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/clprotoConfig.cmake.in
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
  set(PKG_DESC "This library provides a simple interface for encoding and decoding control library type objects to and from a serialized binary string representation (the wire format).")
  set(PKG_EXTERNAL_DEPS "state_representation >= ${PROJECT_VERSION}")
  set(PKG_CFLAGS "-lprotobuf")
  set(pkg_conf_file "clproto.pc")
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${pkg_conf_file}.in" "${CMAKE_CURRENT_BINARY_DIR}/${pkg_conf_file}" @ONLY)
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${pkg_conf_file}"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig/ COMPONENT pkgconfig)
endif()
