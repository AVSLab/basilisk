# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# This file is distributed under the ISC License in LICENSE.

add_executable(test_effectorNames test_effectorNames.cpp test_hingedEffectorNames.cpp
  test_simpleEffectorNames.cpp test_multibodyEffectorNames.cpp test_effectorNamingIntegration.cpp
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

add_test(NAME effectorNamingPython
  COMMAND "${Python3_EXECUTABLE}" -m pytest -q
  "${CMAKE_CURRENT_SOURCE_DIR}/test_hingedEffectorNaming.py"
  "${CMAKE_CURRENT_SOURCE_DIR}/test_simpleEffectorNaming.py"
  "${CMAKE_CURRENT_SOURCE_DIR}/test_actuatorEffectorNaming.py"
  "${CMAKE_CURRENT_SOURCE_DIR}/test_multibodyEffectorNaming.py"
  "${CMAKE_CURRENT_SOURCE_DIR}/test_effectorNamingIntegration.py"
  "${CMAKE_CURRENT_SOURCE_DIR}/test_effectorNamingDeprecation.py"
  "${CMAKE_CURRENT_SOURCE_DIR}/test_effectorNamingCompatibility.py")
set_tests_properties(effectorNamingPython PROPERTIES
  ENVIRONMENT "PYTHONPATH=${CMAKE_BINARY_DIR};MPLBACKEND=Agg")
