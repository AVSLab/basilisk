# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# Distributed under the ISC license; see LICENSE.

add_executable(test_constrainedShortestPath test_constrainedShortestPath.cpp ../constrainedAttitudeManeuver.cpp)
target_link_libraries(test_constrainedShortestPath PRIVATE GTest::gtest_main architectureLib ArchitectureUtilities)

if(WIN32)
  add_custom_command(
    TARGET test_constrainedShortestPath
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:test_constrainedShortestPath>
            $<TARGET_FILE_DIR:test_constrainedShortestPath>
    COMMAND_EXPAND_LISTS)
endif()

if(CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "arm64" AND CMAKE_GENERATOR STREQUAL "Xcode")
  set(CMAKE_GTEST_DISCOVER_TESTS_DISCOVERY_MODE PRE_TEST)
endif()
gtest_discover_tests(test_constrainedShortestPath PROPERTIES TIMEOUT 60)
