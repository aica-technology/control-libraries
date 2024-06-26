set(LIBRARY_NAME dynamical_systems)

set(CORE_SOURCES
  src/IDynamicalSystem.cpp
  src/DynamicalSystemFactory.cpp
  src/Circular.cpp
  src/PointAttractor.cpp
  src/Ring.cpp
)

add_library(${LIBRARY_NAME} SHARED ${CORE_SOURCES})
add_library(${PROJECT_NAME}::${LIBRARY_NAME} ALIAS ${LIBRARY_NAME})

# add include directories
target_include_directories(${LIBRARY_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(${LIBRARY_NAME} ${PROJECT_NAME}::state_representation)

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
  add_executable(test_${LIBRARY_NAME} test/test_dynamical_systems.cpp)
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
  set(PKG_DESC "This library provides a set of classes to represent dynamical systems: functions which map a state to a state derivative.")
  set(PKG_EXTERNAL_DEPS "state_representation >= ${PROJECT_VERSION}")
  set(pkg_conf_file "dynamical_systems.pc")
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/../library_template.pc.in" "${CMAKE_CURRENT_BINARY_DIR}/${pkg_conf_file}" @ONLY)
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${pkg_conf_file}"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig/ COMPONENT pkgconfig)
endif()
