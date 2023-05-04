add_executable(test_discretize test_discretize.cpp)
target_link_libraries(test_discretize GTest::gtest_main)
target_link_libraries(test_discretize ArchitectureUtilities)

add_executable(test_gaussMarkov test_gaussMarkov.cpp)
target_link_libraries(test_gaussMarkov GTest::gtest_main)
target_link_libraries(test_gaussMarkov ArchitectureUtilities)

add_executable(test_saturate test_saturate.cpp)
target_link_libraries(test_saturate GTest::gtest_main)
target_link_libraries(test_saturate ArchitectureUtilities)

if(CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "arm64" AND CMAKE_GENERATOR STREQUAL "Xcode")
    set(CMAKE_GTEST_DISCOVER_TESTS_DISCOVERY_MODE PRE_TEST)
endif()

gtest_discover_tests(test_discretize)
gtest_discover_tests(test_gaussMarkov)
gtest_discover_tests(test_saturate)
