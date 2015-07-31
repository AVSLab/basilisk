cmake_minimum_required(VERSION 2.8)

project(basilisk)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -gdwarf-3 -I${CMAKE_SOURCE_DIR}/SimCode -I${CMAKE_SOURCE_DIR}/External/boost_1_58_0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -gdwarf-3 -I${CMAKE_SOURCE_DIR}/SimCode -I${CMAKE_SOURCE_DIR}/External/boost_1_58_0")
set_property(GLOBAL PROPERTY BUILT_LIB_LIST "SimMessaging;SimUtilities")
set(CMAKE_SWIG_OUTDIR "${CMAKE_SOURCE_DIR}/modules")
# Create list of libraries depending on system
if(WIN32)
   set(library_dependencies 
      ${CMAKE_SOURCE_DIR}/External/cspice/lib/cspice.lib)
elseif(APPLE)
   set(library_dependencies 
      ${CMAKE_SOURCE_DIR}/External/cspice/lib/cspice_osx.a)
else()
   set(library_dependencies 
      ${CMAKE_SOURCE_DIR}/External/cspice/lib/cspice.a)
endif()
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
add_subdirectory("ADCSAlgorithms/CSSEst")

