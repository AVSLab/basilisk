# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# This file is distributed under the ISC License in LICENSE.

add_executable(test_effectorNames test_effectorNames.cpp)
target_link_libraries(test_effectorNames PRIVATE GTest::gtest_main dynamicsLib Eigen3::Eigen3)

# Keep the executable beside its package DLLs for both discovery and execution on Windows.
if(WIN32)
  set_target_properties(test_effectorNames PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Basilisk"
    RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_BINARY_DIR}/Basilisk"
    RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${CMAKE_BINARY_DIR}/Basilisk")
endif()
gtest_discover_tests(test_effectorNames DISCOVERY_MODE PRE_TEST)
