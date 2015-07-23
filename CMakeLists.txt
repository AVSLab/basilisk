cmake_minimum_required(VERSION 2.8)

project(basilisk)

add_definitions( -Wall -std=c++11 -gdwarf-3 "-I$ENV{SIMULATION_BASE}/SimCode -I$ENV{SIMULATION_BASE}/External/boost_1_58_0")
set_property(GLOBAL PROPERTY BUILT_LIB_LIST "SimMessaging;SimUtilities")
set(CMAKE_SWIG_OUTDIR "$ENV{SIMULATION_BASE}/modules")
set(library_dependencies 
   $ENV{SIMULATION_BASE}/External/cspice/lib/cspice.a)
set_property(GLOBAL PROPERTY ALG_LIB_LIST "AlgorithmMessaging")

add_subdirectory("SimCode/utilities")
add_subdirectory("SimCode/architecture/messaging")
add_subdirectory("SimCode/architecture/system_model")
add_subdirectory("SimCode/architecture/alg_contain")
add_subdirectory("SimCode/environment/spice")
add_subdirectory("SimCode/dynamics/SixDofEOM")
add_subdirectory("SimCode/dynamics/Thrusters")
add_subdirectory("SimCode/dynamics/DynOutput")
add_subdirectory("SimCode/sensors/sun_sensor")
add_subdirectory("SimCode/sensors/imu")
add_subdirectory("SimCode/sensors/star_tracker")
add_subdirectory("ADCSAlgorithms/messaging")
add_subdirectory("ADCSAlgorithms/CSSSensorData")

