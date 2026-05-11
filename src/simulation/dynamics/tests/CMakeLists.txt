add_executable(test_quaternionStateData test_quaternionStateData.cpp)
target_link_libraries(test_quaternionStateData GTest::gtest_main)
target_link_libraries(test_quaternionStateData dynamicsLib)

if(CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "arm64" AND CMAKE_GENERATOR STREQUAL "Xcode")
    set(CMAKE_GTEST_DISCOVER_TESTS_DISCOVERY_MODE PRE_TEST)
endif()

gtest_discover_tests(test_quaternionStateData)
