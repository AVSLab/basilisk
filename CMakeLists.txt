cmake_minimum_required(VERSION 2.8)

project("AVS basilisk")

find_package(PythonLibs 2.7 REQUIRED)
find_package(PythonInterp 2.7 REQUIRED)

set_property(GLOBAL PROPERTY BUILT_LIB_LIST "SimUtilities;SimMessaging")
set(CMAKE_SWIG_OUTDIR ${CMAKE_SOURCE_DIR}/modules)

# Add general compiler flags
set(CMAKE_MACOSX_RPATH 1)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")

# Add platform specific compiler flags
if(MSVC)
    add_definitions(/D _CRT_SECURE_NO_WARNINGS)
    add_definitions(/D _WINSOCK_DEPRECATED_NO_WARNINGS)
    add_definitions(/D _WIN32_WINNT=0x0501) # Targets Windows xp
    add_definitions(/D BOOST_ALL_NO_LIB)
    add_definitions(/W3)
    add_definitions(/wd"4913")
    # Make sure we are using Multi-Threaded run time library
    foreach(flag 
        CMAKE_C_FLAGS
        CMAKE_C_FLAGS_DEBUG
        CMAKE_C_FLAGS_RELEASE
        CMAKE_CXX_FLAGS
        CMAKE_CXX_FLAGS_DEBUG
        CMAKE_CXX_FLAGS_RELEASE)
        string(REPLACE "/D_DEBUG" "" "${flag}" "${${flag}}")
        string(REPLACE "/MD" "/MT" "${flag}" "${${flag}}")
        string(REPLACE "/MDd" "/MT" "${flag}" "${${flag}}")
        string(REPLACE "/MTd" "/MT" "${flag}" "${${flag}}")
        set("${flag}" "${${flag}} /EHsc")
		message(${${flag}})
    endforeach()
elseif(APPLE)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -gdwarf-3")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -gdwarf-3 -std=c++11 -stdlib=libc++")
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -gdwarf-3")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}  -Wall")
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}  -Wall")
endif()

foreach( OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES} )
    string( TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG )
    set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_SOURCE_DIR}/modules )
    set( CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_SOURCE_DIR}/modules )
    set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_SOURCE_DIR}/modules )
endforeach( OUTPUTCONFIG CMAKE_CONFIGURATION_TYPES )

# Add include directories
include_directories(SimCode)
include_directories(External/boost_1_61_0)

# Create list of libraries depending on system
if(WIN32)
	message(${WIN32})
	set(library_dependencies 
      	${CMAKE_SOURCE_DIR}/External/cspice/lib/cspice.lib
		${CMAKE_SOURCE_DIR}/External/boost_1_61_0/lib/libboost_system-vc140-mt-s-1_61.lib
		${CMAKE_SOURCE_DIR}/External/boost_1_61_0/lib/libboost_serialization-vc140-mt-s-1_61.lib
		${CMAKE_SOURCE_DIR}/External/boost_1_61_0/lib/libboost_filesystem-vc140-mt-s-1_61.lib
		${CMAKE_SOURCE_DIR}/External/boost_1_61_0/lib/libboost_thread-vc140-mt-s-1_61.lib
	)
elseif(APPLE)
	set(library_dependencies 
		${CMAKE_SOURCE_DIR}/External/boost_1_61_0/lib/libboost_system.a
		${CMAKE_SOURCE_DIR}/External/cspice/lib/cspice_osx.a
		${CMAKE_SOURCE_DIR}/External/boost_1_61_0/lib/libboost_serialization.a
		${CMAKE_SOURCE_DIR}/External/boost_1_61_0/lib/libboost_filesystem.a
		${CMAKE_SOURCE_DIR}/External/boost_1_61_0/lib/libboost_thread.a
	)
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
add_subdirectory("Utilities/pyswice")

#find_package(Doxygen)
#if(DOXYGEN_FOUND)
#   add_custom_target(ScriptRun ALL
#                   COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/DoxyData)
#endif(DOXYGEN_FOUND)
