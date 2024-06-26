set(LIBRARY_NAME communication_interfaces)

set(CPPZMQ_VERSION 4.7.1)
find_package(cppzmq ${CPPZMQ_VERSION} REQUIRED)

set(CORE_SOURCES
  src/sockets/ISocket.cpp
  src/sockets/UDPSocket.cpp
  src/sockets/UDPClient.cpp
  src/sockets/UDPServer.cpp
  src/sockets/ZMQSocket.cpp
  src/sockets/ZMQPublisher.cpp
  src/sockets/ZMQSubscriber.cpp
  src/sockets/ZMQPublisherSubscriber.cpp
  src/sockets/TCPSocket.cpp
  src/sockets/TCPClient.cpp
  src/sockets/TCPServer.cpp
)

add_library(${LIBRARY_NAME} SHARED ${CORE_SOURCES})
add_library(${PROJECT_NAME}::${LIBRARY_NAME} ALIAS ${LIBRARY_NAME})

# add include directories
target_include_directories(${LIBRARY_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(${LIBRARY_NAME}
  cppzmq
)

# install the target and create export-set
install(TARGETS ${LIBRARY_NAME}
  EXPORT ${LIBRARY_NAME}_targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# generate and install export file
install(EXPORT ${LIBRARY_NAME}_targets
  FILE ${PROJECT_NAME}_${LIBRARY_NAME}_targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

if (BUILD_TESTING)
  add_executable(test_${LIBRARY_NAME} test/test_communication_interfaces.cpp)
  file(GLOB_RECURSE MODULE_TEST_SOURCES test/tests test_*.cpp)
  target_sources(test_${LIBRARY_NAME} PRIVATE ${MODULE_TEST_SOURCES})
  target_link_libraries(test_${LIBRARY_NAME}
    ${LIBRARY_NAME}
    ${GTEST_LIBRARIES}
    pthread
  )
  add_test(NAME test_${LIBRARY_NAME} COMMAND test_${LIBRARY_NAME})
endif ()

if(${PKG_CONFIG_FOUND})
  set(PKG_NAME ${LIBRARY_NAME})
  set(PKG_DESC "Communication interfaces for socket communication.")
  set(pkg_conf_file "communication_interfaces.pc")
  set(PKG_CFLAGS "-lcppzmq")
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/../library_template.pc.in" "${CMAKE_CURRENT_BINARY_DIR}/${pkg_conf_file}" @ONLY)
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${pkg_conf_file}"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig/ COMPONENT pkgconfig)
endif()
