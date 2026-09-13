# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# This file is distributed under the ISC License in LICENSE.

add_executable(test_effectorNames test_effectorNames.cpp test_hingedEffectorNames.cpp
  test_simpleEffectorNames.cpp test_multibodyEffectorNames.cpp
  ../../HingedRigidBodies/hingedRigidBodyStateEffector.cpp
  ../../spacecraft/spacecraft.cpp
  ../../LinearSpringMassDamper/linearSpringMassDamper.cpp
  ../../sphericalPendulum/sphericalPendulum.cpp
  ../../FuelTank/fuelTank.cpp
  ../../Thrusters/thrusterStateEffector/thrusterStateEffector.cpp
  ../../meanRevertingNoiseStateEffector/meanRevertingNoiseStateEffector.cpp
  ../../igbmNoiseStateEffector/igbmNoiseStateEffector.cpp
  ../../prescribedMotion/prescribedMotionStateEffector.cpp
  ../../spinningBodies/spinningBodiesOneDOF/spinningBodyOneDOFStateEffector.cpp
  ../../spinningBodies/spinningBodiesTwoDOF/spinningBodyTwoDOFStateEffector.cpp
  ../../spinningBodies/spinningBodiesNDOF/spinningBodyNDOFStateEffector.cpp
  ../../linearTranslationalBodies/linearTranslationBodiesOneDOF/linearTranslationOneDOFStateEffector.cpp
  ../../linearTranslationalBodies/linearTranslationBodiesNDOF/linearTranslationNDOFStateEffector.cpp
  ../../dualHingedRigidBodies/dualHingedRigidBodyStateEffector.cpp
  ../../NHingedRigidBodies/nHingedRigidBodyStateEffector.cpp
  ../../constraintEffector/constraintDynamicEffector.cpp
)
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

# Keep the opt-in bridge outside the Basilisk package and build it only for tests.
set_property(SOURCE effectorNamingTestSupport.i PROPERTY CPLUSPLUS ON)
set_property(SOURCE effectorNamingTestSupport.i PROPERTY USE_TARGET_INCLUDE_DIRECTORIES TRUE)
swig_add_library(effectorNamingTestSupport LANGUAGE python TYPE MODULE
  SOURCES effectorNamingTestSupport.i
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  OUTFILE_DIR "${CMAKE_CURRENT_BINARY_DIR}")
bsk_suppress_strict_warnings_for_sources(
  "${CMAKE_CURRENT_BINARY_DIR}/effectorNamingTestSupportPYTHON_wrap.cxx")
target_link_libraries(effectorNamingTestSupport PRIVATE dynamicsLib ${PYTHON3_MODULE})
target_include_directories(effectorNamingTestSupport PRIVATE ${Python3_INCLUDE_DIRS})
set_target_properties(effectorNamingTestSupport PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
foreach(config DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
  set_target_properties(effectorNamingTestSupport PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY_${config} "${CMAKE_CURRENT_BINARY_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_${config} "${CMAKE_CURRENT_BINARY_DIR}")
endforeach()
add_test(NAME effectorNamingPython
  COMMAND "${Python3_EXECUTABLE}" -m pytest -q
  "${CMAKE_CURRENT_SOURCE_DIR}/test_hingedEffectorNaming.py"
  "${CMAKE_CURRENT_SOURCE_DIR}/test_simpleEffectorNaming.py"
  "${CMAKE_CURRENT_SOURCE_DIR}/test_multibodyEffectorNaming.py"
  "${CMAKE_CURRENT_SOURCE_DIR}/test_effectorNamingCompatibility.py")
cmake_path(CONVERT "${CMAKE_CURRENT_BINARY_DIR};${CMAKE_BINARY_DIR}"
  TO_NATIVE_PATH_LIST naming_test_pythonpath)
string(REPLACE ";" "\\;" naming_test_pythonpath "${naming_test_pythonpath}")
set_tests_properties(effectorNamingPython PROPERTIES
  ENVIRONMENT "PYTHONPATH=${naming_test_pythonpath};MPLBACKEND=Agg;BSK_REQUIRE_NAMING_TEST_SUPPORT=1")
