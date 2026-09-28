# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

add_executable(test_rungeKuttaCache
    test_rungeKuttaCache.cpp
    ../svIntegratorRKF45.cpp
    ../svIntegratorRKF78.cpp)
add_executable(test_adaptiveRungeKuttaAccuracy
    test_adaptiveRungeKuttaAccuracy.cpp
    ../svIntegratorRKF45.cpp
    ../svIntegratorRKF78.cpp)
add_executable(test_stochasticRouting
    test_stochasticRouting.cpp
    ../svStochasticIntegratorMayurama.cpp
    ../../_GeneralModuleFiles/stochasticRKIntegratorBase.cpp
    ../../_GeneralModuleFiles/stateVecStochasticIntegrator.cpp)

# Check Eigen allocations in the header-defined integrator, including Release builds.
target_compile_definitions(test_rungeKuttaCache PRIVATE EIGEN_RUNTIME_NO_MALLOC)

# Exact adaptive trial comparisons need consistent rounding: fused stage sums
# can change error estimates and later step sizes. Keep those strict checks in
# test_rungeKuttaCache; independently check analytic accuracy with contraction
# enabled in test_adaptiveRungeKuttaAccuracy. Production flags are unchanged.
if(MSVC)
  target_compile_options(test_rungeKuttaCache PRIVATE /UNDEBUG /fp:strict)
  target_compile_options(test_adaptiveRungeKuttaAccuracy PRIVATE /fp:fast)
else()
  target_compile_options(test_rungeKuttaCache PRIVATE -UNDEBUG -ffp-contract=off)
  target_compile_options(test_adaptiveRungeKuttaAccuracy PRIVATE -ffp-contract=fast)
endif()

foreach(test_target IN ITEMS test_rungeKuttaCache test_adaptiveRungeKuttaAccuracy test_stochasticRouting)
  target_link_libraries(${test_target} PRIVATE
      GTest::gtest_main dynamicsLib architectureLib ModuleIdGenerator)

  # Test discovery and execution must also find the Basilisk DLLs on Windows.
  if(WIN32)
    add_custom_command(TARGET ${test_target} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              $<TARGET_RUNTIME_DLLS:${test_target}>
              $<TARGET_FILE_DIR:${test_target}>
      COMMAND_EXPAND_LISTS)
  endif()

  # Defer discovery so Apple-silicon Xcode builds do not run the executable during the build.
  gtest_discover_tests(${test_target} DISCOVERY_MODE PRE_TEST NO_PRETTY_VALUES)
endforeach()
