# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

add_executable(test_rungeKuttaCache test_rungeKuttaCache.cpp)
target_link_libraries(test_rungeKuttaCache PRIVATE
    GTest::gtest_main dynamicsLib architectureLib ModuleIdGenerator)

# Check Eigen allocations in the header-defined integrator, including Release builds.
target_compile_definitions(test_rungeKuttaCache PRIVATE EIGEN_RUNTIME_NO_MALLOC)
if(MSVC)
  target_compile_options(test_rungeKuttaCache PRIVATE /UNDEBUG)
else()
  target_compile_options(test_rungeKuttaCache PRIVATE -UNDEBUG)
endif()

# Test discovery and execution must also find the Basilisk DLLs on Windows.
if(WIN32)
  add_custom_command(TARGET test_rungeKuttaCache POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:test_rungeKuttaCache>
            $<TARGET_FILE_DIR:test_rungeKuttaCache>
    COMMAND_EXPAND_LISTS)
endif()

# Defer discovery so Apple-silicon Xcode builds do not run the executable during the build.
gtest_discover_tests(test_rungeKuttaCache DISCOVERY_MODE PRE_TEST NO_PRETTY_VALUES)
