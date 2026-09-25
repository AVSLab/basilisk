# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# Distributed under the ISC license; see LICENSE.

# These implementations normally belong to separate Python extension targets.
add_executable(test_simThreadOwnership test_simThreadOwnership.cpp ../sim_model.cpp ../sys_process.cpp)
target_link_libraries(test_simThreadOwnership PRIVATE GTest::gtest_main architectureLib)

if(WIN32)
  add_custom_command(
    TARGET test_simThreadOwnership
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:test_simThreadOwnership>
            $<TARGET_FILE_DIR:test_simThreadOwnership>
    COMMAND_EXPAND_LISTS)
endif()

if(CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "arm64" AND CMAKE_GENERATOR STREQUAL "Xcode")
  set(CMAKE_GTEST_DISCOVER_TESTS_DISCOVERY_MODE PRE_TEST)
endif()
gtest_discover_tests(test_simThreadOwnership PROPERTIES TIMEOUT 15)
