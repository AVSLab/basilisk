# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# Distributed under the ISC license; see LICENSE.

# Compile the module implementations directly to test native input validation.
add_executable(test_lambertTransferDirection
               test_lambertTransferDirection.cpp
               test_lambertInputValidation.cpp
               ../lambertSolver.cpp
               ../../lambertPlanner/lambertPlanner.cpp)
target_link_libraries(test_lambertTransferDirection PRIVATE GTest::gtest_main architectureLib)

if(WIN32)
  add_custom_command(
    TARGET test_lambertTransferDirection
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:test_lambertTransferDirection>
            $<TARGET_FILE_DIR:test_lambertTransferDirection>
    COMMAND_EXPAND_LISTS)
endif()

if(CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "arm64" AND CMAKE_GENERATOR STREQUAL "Xcode")
  set(CMAKE_GTEST_DISCOVER_TESTS_DISCOVERY_MODE PRE_TEST)
endif()
gtest_discover_tests(test_lambertTransferDirection PROPERTIES TIMEOUT 15)
