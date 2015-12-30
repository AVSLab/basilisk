cmake_minimum_required(VERSION 2.8)

project("AVS basilisk")

set(CMAKE_MACOSX_RPATH 1)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -gdwarf-3")
if(NOT WIN32)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}  -Wall")
endif()
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")
if(NOT WIN32)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}  -Wall")
endif()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -gdwarf-3 -I${CMAKE_SOURCE_DIR}/SimCode -I${CMAKE_SOURCE_DIR}/External/boost_1_58_0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -gdwarf-3 -std=c++11 -I${CMAKE_SOURCE_DIR}/SimCode -I${CMAKE_SOURCE_DIR}/External/boost_1_58_0")
set_property(GLOBAL PROPERTY BUILT_LIB_LIST "SimUtilities;SimMessaging")
set(CMAKE_SWIG_OUTDIR ${CMAKE_SOURCE_DIR}/modules)
foreach( OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES} )
    string( TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG )
    set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_SOURCE_DIR}/modules )
    set( CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_SOURCE_DIR}/modules )
    set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_SOURCE_DIR}/modules )
endforeach( OUTPUTCONFIG CMAKE_CONFIGURATION_TYPES )
# Create list of libraries depending on system
# Test code change to be removed: Pat
if(WIN32)
   set(library_dependencies 
      ${CMAKE_SOURCE_DIR}/External/cspice/lib/cspice.lib)
elseif(APPLE)
   set(library_dependencies 
      ${CMAKE_SOURCE_DIR}/External/cspice/lib/cspice_osx.a)
   set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
   set(CMAKE_INSTALL_RPATH "@loader_path/")
else()
   set(library_dependencies
      ${CMAKE_SOURCE_DIR}/External/cspice/lib/cspice.a)
   set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
   set(CMAKE_INSTALL_RPATH "\$ORIGIN/")

endif()
set_property(GLOBAL PROPERTY ALG_LIB_LIST "AlgorithmMessaging")
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

add_subdirectory("SimCode/utilities")
add_subdirectory("SimCode/architecture/messaging")
add_subdirectory("SimCode")
add_subdirectory("ADCSAlgorithms/messaging")
add_subdirectory("ADCSAlgorithms")

#find_package(Doxygen)
#if(DOXYGEN_FOUND)
#   add_custom_target(ScriptRun ALL
#                   COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/DoxyData)
#endif(DOXYGEN_FOUND)
