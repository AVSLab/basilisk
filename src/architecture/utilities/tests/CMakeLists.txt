add_executable(test_discretize test_discretize.cpp)
target_link_libraries(test_discretize GTest::gtest_main)
target_link_libraries(test_discretize ArchitectureUtilities)

if(CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "arm64" AND CMAKE_GENERATOR STREQUAL "Xcode")
    set(CMAKE_GTEST_DISCOVER_TESTS_DISCOVERY_MODE PRE_TEST)
endif()

gtest_discover_tests(test_discretize)
