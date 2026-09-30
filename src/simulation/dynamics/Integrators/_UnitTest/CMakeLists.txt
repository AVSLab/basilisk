set(flat_integrator_implementations
    ../svIntegratorEuler.cpp
    ../svIntegratorRK2.cpp
    ../svIntegratorRKF45.cpp
    ../svIntegratorRKF78.cpp
    ../svStochasticIntegratorDRI1.cpp
    ../svStochasticIntegratorEulerHeun.cpp
    ../svStochasticIntegratorMayurama.cpp
    ../svStochasticIntegratorRDI1WM.cpp
    ../svStochasticIntegratorRKMil.cpp
    ../svStochasticIntegratorRS.cpp
    ../svStochasticIntegratorSIESME.cpp
    ../svStochasticIntegratorSOSRA.cpp
    ../svStochasticIntegratorSOSRI.cpp
    ../svStochasticIntegratorSRA1.cpp
    ../svStochasticIntegratorSRIW1.cpp
    ../svStochasticIntegratorW2Ito.cpp
    ../svStochasticIntegratorW2Ito1.cpp
    ../svStochasticIntegratorW2Ito2.cpp)

add_library(flat_integrator_test_support STATIC
            ${flat_integrator_implementations})
target_link_libraries(flat_integrator_test_support
                      PUBLIC dynamicsLib architectureLib)
target_compile_definitions(flat_integrator_test_support
                           PUBLIC EIGEN_RUNTIME_NO_MALLOC)

set(flat_integrator_tests
    test_adaptiveRungeKuttaAccuracy
    test_flatRungeKutta
    test_flatStochastic
    test_flatStochasticBasic
    test_flatStochasticTableaux
    test_flatWeakItoFamilies)

foreach(test_target IN LISTS flat_integrator_tests)
  add_executable(${test_target} ${test_target}.cpp)
  target_link_libraries(${test_target}
                        PRIVATE flat_integrator_test_support GTest::gtest_main)
endforeach()

add_executable(test_stateBufferViews test_stateBufferViews.cpp)
target_link_libraries(test_stateBufferViews
                      PRIVATE dynamicsLib architectureLib GTest::gtest_main)
if(BUILD_MUJOCO)
  target_link_libraries(test_stateBufferViews PRIVATE mujocoDynamicsLib)
  target_compile_definitions(test_stateBufferViews PRIVATE BASILISK_TEST_MUJOCO_POLICIES)
endif()

add_executable(test_rungeKuttaAccuracy test_rungeKuttaAccuracy.cpp)
target_link_libraries(test_rungeKuttaAccuracy
                      PRIVATE dynamicsLib architectureLib GTest::gtest_main)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
  target_compile_options(test_rungeKuttaAccuracy PRIVATE -ffp-contract=fast)
endif()

if(APPLE)
  add_library(integrator_allocation_test_support SHARED allocationTracker.cpp)
else()
  add_library(integrator_allocation_test_support STATIC allocationTracker.cpp)
endif()
target_include_directories(integrator_allocation_test_support
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(integrator_allocation_test_support
                           PUBLIC EIGEN_RUNTIME_NO_MALLOC)

if(APPLE)
  add_library(integrator_allocation_probe SHARED allocationProbe.cpp)
else()
  add_library(integrator_allocation_probe STATIC allocationProbe.cpp)
endif()
target_include_directories(integrator_allocation_probe
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(test_integratorAllocations test_integratorAllocations.cpp)
target_link_libraries(test_integratorAllocations
                      PRIVATE flat_integrator_test_support
                              integrator_allocation_test_support
                              integrator_allocation_probe
                              GTest::gtest_main)

if(MSVC)
  target_compile_options(flat_integrator_test_support PRIVATE /UNDEBUG)
  target_compile_options(integrator_allocation_test_support PRIVATE /UNDEBUG)
  target_compile_options(test_integratorAllocations PRIVATE /UNDEBUG)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
  target_compile_options(flat_integrator_test_support PRIVATE -UNDEBUG)
  target_compile_options(integrator_allocation_test_support
                         PRIVATE -UNDEBUG
                                 -fno-builtin-malloc
                                 -fno-builtin-calloc
                                 -fno-builtin-realloc
                                 -fno-builtin-free)
  target_compile_options(test_integratorAllocations
                         PRIVATE -UNDEBUG
                                 -fno-builtin-malloc
                                 -fno-builtin-calloc
                                 -fno-builtin-realloc
                                 -fno-builtin-free)
  target_compile_options(integrator_allocation_probe
                         PRIVATE -fno-builtin-malloc
                                 -fno-builtin-calloc
                                 -fno-builtin-realloc
                                 -fno-builtin-free)
endif()

# Track C allocations on ELF platforms as well as C++ new/delete. The native
# positive controls verify that the selected interception backend is active.
if(UNIX AND NOT APPLE)
  include(CheckLinkerFlag)
  set(allocator_wrap_flags
      -Wl,--wrap=malloc -Wl,--wrap=calloc -Wl,--wrap=realloc
      -Wl,--wrap=free -Wl,--wrap=aligned_alloc -Wl,--wrap=posix_memalign
      -Wl,--wrap=memalign)
  string(JOIN " " allocator_wrap_probe ${allocator_wrap_flags})
  check_linker_flag(CXX "${allocator_wrap_probe}" allocator_wrap_supported)
  if(allocator_wrap_supported)
    target_compile_definitions(integrator_allocation_test_support
                               PRIVATE BASILISK_TEST_LINKER_WRAP_ALLOCATORS)
    target_link_options(integrator_allocation_test_support INTERFACE ${allocator_wrap_flags})
  endif()
endif()

# Reuse the calibrated allocator interception in the stochastic topology tests.
foreach(test_target test_flatStochastic test_flatStochasticBasic
                    test_flatStochasticTableaux test_flatWeakItoFamilies)
  target_link_libraries(${test_target} PRIVATE integrator_allocation_test_support)
endforeach()

foreach(test_target IN ITEMS ${flat_integrator_tests} test_stateBufferViews
                             test_rungeKuttaAccuracy test_integratorAllocations)
  if(WIN32)
    # Separate directories prevent parallel targets from copying over the same DLL.
    set_target_properties(${test_target} PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${test_target}/$<CONFIG>")
    # Refresh DLLs before discovery, including builds that do not relink the test.
    add_custom_target(${test_target}_runtime_dlls
      COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:${test_target}>"
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              "$<TARGET_RUNTIME_DLLS:${test_target}>" "$<TARGET_FILE_DIR:${test_target}>"
      DEPENDS "$<TARGET_RUNTIME_DLLS:${test_target}>"
      COMMAND_EXPAND_LISTS
      VERBATIM)
    # DLL file dependencies alone do not order library projects in Visual Studio.
    get_target_property(test_libraries ${test_target} LINK_LIBRARIES)
    foreach(test_library IN LISTS test_libraries)
      if(TARGET "${test_library}")
        add_dependencies(${test_target}_runtime_dlls ${test_library})
      endif()
    endforeach()
    add_dependencies(${test_target} ${test_target}_runtime_dlls)
  endif()
  gtest_discover_tests(${test_target})
endforeach()
