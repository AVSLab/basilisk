# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# Distributed under the ISC license; see LICENSE.

add_executable(test_dynamicObject test_dynamicObject.cpp)
target_link_libraries(test_dynamicObject PRIVATE GTest::gtest_main dynamicsLib)

if(WIN32)
  add_custom_command(
    TARGET test_dynamicObject
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:test_dynamicObject>
            $<TARGET_FILE_DIR:test_dynamicObject>
    COMMAND_EXPAND_LISTS)
endif()

if(CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "arm64" AND CMAKE_GENERATOR STREQUAL "Xcode")
  gtest_discover_tests(test_dynamicObject DISCOVERY_MODE PRE_TEST)
else()
  gtest_discover_tests(test_dynamicObject)
endif()
