# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# Distributed under the ISC license; see LICENSE.

foreach(test_target IN ITEMS test_dynamicObject test_gravityEffectorEnergy)
  add_executable(${test_target} ${test_target}.cpp)
  target_link_libraries(${test_target} PRIVATE GTest::gtest_main dynamicsLib)

  if(WIN32)
    add_custom_command(
      TARGET ${test_target}
      POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              $<TARGET_RUNTIME_DLLS:${test_target}>
              $<TARGET_FILE_DIR:${test_target}>
      COMMAND_EXPAND_LISTS)
  endif()

  if(CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "arm64" AND CMAKE_GENERATOR STREQUAL "Xcode")
    gtest_discover_tests(${test_target} DISCOVERY_MODE PRE_TEST)
  else()
    gtest_discover_tests(${test_target})
  endif()
endforeach()
