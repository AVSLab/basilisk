add_executable(test_quaternionStateData test_quaternionStateData.cpp)
target_link_libraries(test_quaternionStateData PRIVATE GTest::gtest_main)
target_link_libraries(test_quaternionStateData PRIVATE dynamicsLib)

if(WIN32)
    add_custom_command(TARGET test_quaternionStateData POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:test_quaternionStateData>
            $<TARGET_FILE_DIR:test_quaternionStateData>
        COMMAND_EXPAND_LISTS)
endif()

if(CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "arm64" AND CMAKE_GENERATOR STREQUAL "Xcode")
    set(CMAKE_GTEST_DISCOVER_TESTS_DISCOVERY_MODE PRE_TEST)
endif()

gtest_discover_tests(test_quaternionStateData)
