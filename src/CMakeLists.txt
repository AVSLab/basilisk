cmake_minimum_required(VERSION 2.8)
if(POLICY CMP0078)
  cmake_policy(SET CMP0078 NEW)
endif()
if(POLICY CMP0025)
  cmake_policy(SET CMP0025 NEW)
endif()
if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

if(POLICY CMP0086)
  cmake_policy(SET CMP0086 NEW)
endif()

if(POLICY CMP0046)
  cmake_policy(SET CMP0046 NEW)
endif()

project("basilisk")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING
      "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif(NOT CMAKE_BUILD_TYPE)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")




include(conan)
conan_add_remote(NAME conan-community URL https://api.bintray.com/conan/conan-community/conan)
conan_add_remote(NAME bincrafters URL https://api.bintray.com/conan/bincrafters/public-conan)

set(required_conan_libraries eigen/3.3.7@conan/stable)

# This appears to be temporarily nessesary until the new conan center libsodium is ready
list(APPEND required_conan_libraries libsodium/1.0.18@bincrafters/stable)


option(USE_PROTOBUFFERS "Include modules which use Google Protobuffers" ON)
option(USE_OPENCV "Include modules which use OpenCV" OFF)
option(USE_ZMQ "Include modules which use ZMQ" ON)
option(USE_PYTHON3 "Build For Python 3" ON)

set(requires_protobuff_modules "vizInterface/vizInterface.i")
set(requires_zmq_modules "vizInterface/vizInterface.i")


set(requires_opencv_modules "imageProcessing/houghCircles/houghCircles.i")
list(APPEND requires_opencv_modules "imageProcessing/limbFinding/limbFinding.i")
list(APPEND requires_opencv_modules "sensors/camera/camera.i")


if(USE_PROTOBUFFERS)
  list(APPEND required_conan_libraries protobuf/3.5.2@bincrafters/stable)
endif()
if(USE_OPENCV )
  list(APPEND required_conan_libraries opencv/4.1.1@conan/stable)
  # temp bug fix for bad pcre dependancies
  list(APPEND required_conan_libraries zlib/1.2.11@conan/stable)
  list(APPEND required_conan_libraries bzip2/1.0.8@conan/stable)
endif()

if(USE_ZMQ)
  list(APPEND required_conan_libraries cppzmq/4.3.0@bincrafters/stable)
endif()

conan_cmake_run(REQUIRES ${required_conan_libraries}
                OPTIONS opencv:contrib=True
	              BASIC_SETUP KEEP_RPATHS
                BUILD missing )

if(USE_PROTOBUFFERS)
   #! PROTO-BUFFERS
   find_package(protobuf CONFIG REQUIRED)
   message(STATUS "Using Protocol Buffers ${protobuf_VERSION}")
   set(CMAKE_INCLUDE_CURRENT_DIR TRUE)
endif()

if(USE_OPENCV)
   #! OPEN_CV
   #find_package(opencv CONFIG REQUIRED)
   message(STATUS "Using OpenCV ${opencv_VERSION}")
   set(CMAKE_INCLUDE_CURRENT_DIR TRUE)
   if(APPLE)
     link_libraries("-framework OpenCL")
   endif()
endif()

if(USE_ZMQ)
   #! USE_ZMQ
   find_package(cppzmq CONFIG REQUIRED)
   message(STATUS "Using ZMQ ${cppzmq_VERSION}")
   set(CMAKE_INCLUDE_CURRENT_DIR TRUE)
endif()


# HACK: Create Release and Debug directories for conan
# Conan hasn't fixed this issue yet, this removes warnings

foreach (library ${CONAN_LIB_DIRS})
   file(MAKE_DIRECTORY "${library}/Release")
   file(MAKE_DIRECTORY "${library}/Debug")
endforeach(library)


# Create a Swig module
# Avoids a deprecation error in CMAKE versions less than 3 by calling the correct API depending on version
# SWIG_ADD_MODULE
macro(MAKE_SWIG_MODULE name language)

	if("${CMAKE_VERSION}" VERSION_GREATER 3.8)
		SWIG_ADD_LIBRARY(${name}
	                   LANGUAGE ${language}
	                   TYPE MODULE
	                   SOURCES ${ARGN})
	else()
		SWIG_ADD_MODULE(${name} ${language} ${ARGN})
	endif()

endmacro()


#! make_python_package : create python package
#
# This function creates a python package directory and adds an __init__.py
# file or copies an existing custom __init__.py file. A list of package
# supporting files may be passed as the thried parameter. The function takes
# three arguments.
#
# \arg:FileBase the desired python package name
# \arg:FileDir the directory path relative to the cmake script calling the function
# \arg:moduleFilePath path to the Basilisk module's resulting python package directory
#
function(make_python_package FileBase FileDir moduleFilePath)
	#message("Called make python package with ${FileBase} ${FileDir} ${moduleFilePath}")
	make_python_package_dir(${FileBase} ${FileDir} ${moduleFilePath})
	# Create symlinks in the python package directory to any
	# Basilisk module supporting files.
	file(GLOB supportingFiles
		"${CMAKE_CURRENT_SOURCE_DIR}/${FileDir}/*.xml"
		"${CMAKE_CURRENT_SOURCE_DIR}/${FileDir}/*.py")
	list(REMOVE_ITEM supportingFiles ${CMAKE_CURRENT_SOURCE_DIR}/${FileDir}/__init__.py)

	create_symlinks(${moduleFilePath} ${supportingFiles})
endfunction(make_python_package)


#! create_symlinks : create python package directory
#
# This function creates symlinks to supporting files for python packages.
# The function takes one required argument and N optional arguments.
#
# \arg:destinationPath the desired location in which to create the symlinks
# \argn: a list of supporting file paths
#
function(create_symlinks destinationPath)
    # Do nothing if building in-source
    if (${CMAKE_CURRENT_BINARY_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
        return()
    endif()
	#message(destinationPath: ${destinationPath})
	#message(ARGN: ${ARGN})
    foreach (filePath ${ARGN})
        get_filename_component(fileName ${filePath} NAME)
# message(${folder})
#         # Create REAL folder
#         file(MAKE_DIRECTORY "${destinationPath}/${folder}")
# message("directory created: ${destinationPath}/${folder}")
        # Delete symlink if it exists
        # file(REMOVE "${CMAKE_CURRENT_BINARY_DIR}/${path_file}")

        # Get OS dependent path to use in `execute_process`
        file(TO_NATIVE_PATH "${destinationPath}/${fileName}" link)
        file(TO_NATIVE_PATH "${filePath}" target)
        file(REMOVE ${destinationPath}/${fileName})

        if (UNIX)
            set(command ln -s ${target} ${link})
        elseif(IS_DIRECTORY ${target})
            message("Creating Directory Symbolic Link")
            set(command cmd.exe /c mklink /D ${link} ${target})
        else()
            #message(target)
            set(command cmd.exe /c mklink ${link} ${target})
        endif()

        execute_process(COMMAND ${command}
                        RESULT_VARIABLE result
                        ERROR_VARIABLE output)

        if (NOT ${result} EQUAL 0)
            file( COPY ${target} DESTINATION ${destinationPath})
            message("Could not create symbolic link for: ${target} --> ${output}.  Copied instead")
        endif()

    endforeach(filePath)
endfunction(create_symlinks)


#! make_python_package_dir : create python package directory
#
# This function creates a python package directory and adds an __init__.py
# file or copies an existing custom __init__.py file. The function takes
# two arguments.
#
# \arg:FileBase the desired python package name
# \arg:FileDir the directory path relative to the cmake script calling the function
#
function(make_python_package_dir ModuleName FileDir OutputModule)
	#message(ModuleName: ${ModuleName})
	#message(FileDir: ${FileDir})
	#message(Making Directory ${OutputModule})
	set(destination ${OutputModule})
    file(MAKE_DIRECTORY ${OutputModule})
	if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${FileDir}/__init__.py")
		file(WRITE "__init__.py" "# This __init__.py file for the ${ModuleName} package is automatically generated by the build system
from .${ModuleName} import *")
		file(COPY "__init__.py" DESTINATION ${destination})
	else()
		configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${FileDir}/__init__.py ${destination}/__init__.py COPYONLY)
	endif()
	if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/__init__.py")
		file(REMOVE "__init__.py")
	endif()
endfunction(make_python_package_dir)


function(findAndCreateSwigModules SubModule)

	find_package(SWIG REQUIRED)
	include(${SWIG_USE_FILE})
  	if(USE_PYTHON3)
		find_package(PythonLibs 3 REQUIRED)
		set(CMAKE_SWIG_FLAGS "-py3")
	else()
		find_package(PythonLibs 2 REQUIRED)
  	endif()
	include_directories(${PYTHON_INCLUDE_PATH})

	set(CMAKE_SWIG_FLAGS "")

	file(GLOB sub_directories RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/*)

	string(FIND ${CMAKE_CURRENT_SOURCE_DIR} "/" DIR_NAME_START REVERSE)
	math(EXPR DIR_NAME_START "${DIR_NAME_START} + 1")
	string(SUBSTRING ${CMAKE_CURRENT_SOURCE_DIR} ${DIR_NAME_START} -1 DIR_NAME_STRING)

	project("${DIR_NAME_STRING}")

	file(GLOB_RECURSE swig_modules RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.i")
	get_property(LIBRARY_BUILD_LIST GLOBAL PROPERTY BUILT_LIB_LIST)
	get_property(ALG_LIST GLOBAL PROPERTY ALG_LIB_LIST)
	get_property(INTER_FILES GLOBAL PROPERTY SYS_INTER_FILES)

	file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/${SubModule}")
	file(WRITE "${CMAKE_BINARY_DIR}/Basilisk/${SubModule}__init__.py" "#empty init file")

	get_property(LIBRARY_BUILD_LIST GLOBAL PROPERTY BUILT_LIB_LIST)
	get_property(ALG_LIST GLOBAL PROPERTY ALG_LIB_LIST)

	foreach(module ${swig_modules})
	    get_filename_component(FileBase ${module} NAME_WE)
            get_filename_component(FileDir ${module} DIRECTORY)


 	    if((NOT USE_PROTOBUFFERS OR NOT USE_ZMQ) AND (module IN_LIST requires_protobuff_modules OR module IN_LIST requires_zmq_modules))
               continue()
 	    endif()
 	    if((NOT USE_OPENCV) AND (module IN_LIST requires_opencv_modules))
 	           continue()
 	    endif()

	    set(CMAKE_SWIG_OUTDIR "${CMAKE_BINARY_DIR}/Basilisk/${SubModule}${FileBase}")

	    file(GLOB impl_files
		    "${FileDir}/*.cpp"
		    "${FileDir}/*.c"
		    "${FileDir}/*.h"
		    "${FileDir}/*.rst"
		)
	    set(gen_files "")
	    set(swig_dep_files "")
	    file(GLOB gen_files
	        "${CMAKE_SOURCE_DIR}/simulation/_GeneralModuleFiles/*.cpp"
	        "${CMAKE_SOURCE_DIR}/simulation/_GeneralModuleFiles/*.h"
	        "${CMAKE_SOURCE_DIR}/simulation/_GeneralModuleFiles/*.c")
	    file(GLOB swig_dep_files
	        "${CMAKE_SOURCE_DIR}/simulation/_GeneralModuleFiles/*.i")
	    string(REPLACE "/" ";" DirList ${FileDir})
	    list(LENGTH DirList dirLength)
	    set(currentDir "")
	    set(endDir "")
	    foreach(localDir ${DirList})
	        set(currentDir "${currentDir}${localDir}/")
	        file(GLOB gen_files_loc "${currentDir}_GeneralModuleFiles/*.cpp"
	            "${currentDir}_GeneralModuleFiles/*.h"
	            "${currentDir}_GeneralModuleFiles/*.c")
	        file(GLOB swig_dep_loc "${currentDir}_GeneralModuleFiles/*.i")
	        set(gen_files "${gen_files};${gen_files_loc}")
	        set(swig_dep_files "${swig_dep_files};${swig_dep_loc}")
	    set(endDir ${localDir})
	    endforeach()
	    if("${endDir}" STREQUAL "_GeneralModuleFiles")
	        continue()
	    endif()

		# Make a python package
		set(moduleFilePath "${CMAKE_BINARY_DIR}/Basilisk/${SubModule}${FileBase}")
		make_python_package(${FileBase} ${FileDir} ${moduleFilePath})

	    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${FileDir})
	    set_source_files_properties(${module} PROPERTIES CPLUSPLUS ON)
		set_property(SOURCE ${module} PROPERTY SWIG_FLAGS "-I${CMAKE_SOURCE_DIR}/simulation/_GeneralModuleFiles" "-I${CMAKE_CURRENT_SOURCE_DIR}/${FileDir}/")
		# MESSAGE("${SWIG_FLAGS}")
	    #set_source_files_properties(${module} PROPERTIES SWIG_FLAGS "-I${CMAKE_CURRENT_SOURCE_DIR}/${FileDir}/")
		set_source_files_properties( ${swig_generated_file_fullname}
	               PROPERTIES COMPILE_FLAGS "-I${CMAKE_SOURCE_DIR}/simulation")
	    set(SWIG_MODULE_${FileBase}_EXTRA_DEPS ${impl_files} ${gen_files} ${swig_dep_files})
	    MAKE_SWIG_MODULE(${FileBase} python ${module} ${impl_files} ${gen_files})

	    SWIG_LINK_LIBRARIES(${FileBase} ${PYTHON_LIBRARIES})
	    SWIG_LINK_LIBRARIES(${FileBase} ${CONAN_LIBS})
            if (USE_PROTOBUFFERS)
	       SWIG_LINK_LIBRARIES(${FileBase} protocode)
            endif()
	    foreach(LibFile ${LIBRARY_BUILD_LIST})
	         SWIG_LINK_LIBRARIES(${FileBase} ${LibFile})
	    endforeach()

	    foreach(LibFile ${library_dependencies})
	         SWIG_LINK_LIBRARIES(${FileBase} ${LibFile})
	    endforeach()

	    set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
			FOLDER "${DIR_NAME_STRING}/${FileDir}")
		#set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
		#     CMAKE_SWIG_OUTDIR "${CMAKE_BINARY_DIR}/Basilisk/${SubModule}${FileBase}")
	    set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
	        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/${SubModule}${FileBase}")
	    set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
	        LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk/${SubModule}${FileBase}")
		# set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
	        # ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk/${SubModule}${FileBase}")
	    # set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
	        # RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk/${SubModule}${FileBase}")
		set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
			LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Basilisk/${SubModule}${FileBase}")
	    set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
		    COMPILE_FLAGS "-I${CMAKE_CURRENT_SOURCE_DIR}")
	endforeach()
endfunction(findAndCreateSwigModules)

# Start of main projection configuration
if(USE_PYTHON3)
	find_package(PythonInterp 3 REQUIRED)
	find_package(PythonLibs 3 REQUIRED)
else()
	find_package(PythonInterp 2.7 REQUIRED)
	find_package(PythonLibs 2.7 REQUIRED)
endif()

# set(CMAKE_SWIG_OUTDIR ${CMAKE_BINARY_DIR}/Basilisk)

############################################################################################
OPTION(USE_COVERAGE "GCOV code coverage analysis"      OFF)
############################################################################################

# Add general compiler flags
set(CMAKE_MACOSX_RPATH 1)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")

############################################################################################
IF (USE_COVERAGE AND (CMAKE_COMPILER_IS_GNUC OR CMAKE_COMPILER_IS_GNUCXX))
    MESSAGE(STATUS "USE_COVERAGE : ${USE_COVERAGE}")
    SET(GCOV_CFLAGS  "-fprofile-arcs -ftest-coverage -O0 -fno-default-inline -fno-inline")
    SET(GCOV_LDFLAGS "-fprofile-arcs -ftest-coverage -O0 -fno-default-inline -fno-inline")

    SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} ${GCOV_CFLAGS}")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCOV_CFLAGS}")
    SET(CMAKE_LD_FLAGS "${CMAKE_LD_FLAGS} ${GCOV_LDFLAGS}")

    MESSAGE(STATUS "CMAKE_C_FLAGS : ${CMAKE_C_FLAGS}")
    MESSAGE(STATUS "CMAKE_CXX_FLAGS : ${CMAKE_CXX_FLAGS}")
    MESSAGE(STATUS "CMAKE_LD_FLAGS : ${CMAKE_LD_FLAGS}")
ENDIF()
#############################################################################################

# Add platform specific compiler flags
if(MSVC)
    add_definitions(/MP)
    add_definitions(/D _CRT_SECURE_NO_WARNINGS)
    add_definitions(/D _WINSOCK_DEPRECATED_NO_WARNINGS)
    add_definitions(/D _WIN32_WINNT=0x0501) # Targets Windows xp
    add_definitions(/W3)
    add_definitions(/wd"4913")
    add_definitions(/wd"4251")
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
    endforeach()
elseif(APPLE)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -gdwarf-3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -gdwarf-3 -std=c++11 -stdlib=libc++")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -gdwarf-3")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}  -Wall")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}  -Wall")
else()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -gdwarf-3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -gdwarf-3 -std=c++11 -fPIC")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -gdwarf-3")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}  -Wall")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}  -Wall")
endif()

if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-register -std=c++11")
endif()


# foreach( OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES} )
    # string( TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG )
    # set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_BINARY_DIR}/Basilisk)
    # set( CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_BINARY_DIR}/Basilisk)
    # set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_BINARY_DIR}/Basilisk)
# endforeach( OUTPUTCONFIG CMAKE_CONFIGURATION_TYPES )

# Add include directories
include_directories(simulation)
include_directories(../libs)



if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(_arch_suffix 64)
else()
  set(_arch_suffix 32)
endif()




# Manually create list of libraries depending on system
if(WIN32)
	set(library_dependencies
      	${CMAKE_SOURCE_DIR}/../libs/cspice/lib/cspice_win${_arch_suffix}.lib
	)

        # fix for opencv not finding conan gflags for opencv sfm lib on windows
        # when the finding is fixed, the following line should be removed
        # https://github.com/conan-community/community/issues/210 
        list(REMOVE_ITEM CONAN_LIBS opencv_sfm411d multiview)

elseif(APPLE)
	set(library_dependencies
		${CMAKE_SOURCE_DIR}/../libs/cspice/lib/cspice_osx.a
		# ${CMAKE_SOURCE_DIR}/../libs/boost_1_61_0/lib64-osx/libboost_system.a
# 		${CMAKE_SOURCE_DIR}/../libs/boost_1_61_0/lib64-osx/libboost_serialization.a
# 		${CMAKE_SOURCE_DIR}/../libs/boost_1_61_0/lib64-osx/libboost_filesystem.a
#${CMAKE_SOURCE_DIR}/../libs/boost_1_61_0/lib64-osx/libboost_thread.a
	)
   	#set(CMAKE_INSTALL_RPATH "@rpath/../Basilisk")
	# use, i.e. don't skip the full RPATH for the build tree
	SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

	# when building, don't use the install RPATH already
	# (but later on when installing)
	SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

	# the RPATH to be used when installing
	SET(CMAKE_INSTALL_RPATH "${CMAKE_BINARY_DIR}/Basilisk")

	# don't add the automatically determined parts of the RPATH
	# which point to directories outside the build tree to the install RPATH
	SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
else()
	set(library_dependencies
		${CMAKE_SOURCE_DIR}/../libs/cspice/lib/cspice.a
	)
	SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
	set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
	#set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}../libs/")
	set(CMAKE_INSTALL_RPATH "\$ORIGIN/../../")
endif()

set_property(GLOBAL PROPERTY BUILT_LIB_LIST "SimUtilities;SimMessaging")
set_property(GLOBAL PROPERTY ALG_LIB_LIST "AlgorithmMessaging")
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# make the built package with an empty init file
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk")
file(WRITE "${CMAKE_BINARY_DIR}/Basilisk/__init__.py" "#empty init file written by the build")

#symlink into the package the current python module code
file(GLOB pythonModules
	"${CMAKE_SOURCE_DIR}/utilities/*.py"
	"${CMAKE_SOURCE_DIR}/utilities/**")

create_symlinks("${CMAKE_BINARY_DIR}/Basilisk/utilities" ${pythonModules})

#symlink into the package the test code
file(GLOB simTests "${CMAKE_SOURCE_DIR}/tests/*")
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/tests")
file(WRITE "${CMAKE_BINARY_DIR}/Basilisk/tests/__init__.py" "#empty init file written by the build")
create_symlinks("${CMAKE_BINARY_DIR}/Basilisk/tests" ${simTests})

#symlink into package the supportData files to keep
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/supportData")
file(GLOB dataFiles "${CMAKE_SOURCE_DIR}/../supportData/*")
create_symlinks("${CMAKE_BINARY_DIR}/Basilisk/supportData" ${dataFiles})

set(ARG setup.py develop --user)
execute_process(COMMAND ${PYTHON_EXECUTABLE} ${ARG}
								WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/../
								RESULT_VARIABLE rv
								OUTPUT_VARIABLE out)
message(STATUS "Adding Basilisk module to python with: ${PROGRAM} ${ARG} in ${CMAKE_SOURCE_DIR}/..
This resulted in the output:
${out}")

if(USE_PROTOBUFFERS)
  add_subdirectory("utilities/vizProtobuffer")
endif()
add_subdirectory("simulation")
add_subdirectory("fswAlgorithms")
add_subdirectory("topLevelModules") 	

#run python script to modify swig gen code
add_custom_target(OverwriteSwig ALL)
set(ARG utilities/overwriteSwig.py ${CMAKE_BINARY_DIR} ${SWIG_VERSION})
add_custom_command(
    TARGET OverwriteSwig
    COMMAND ${PYTHON_EXECUTABLE} ${ARG}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/../src/)
if(TARGET _thrusterDynamicEffector)
    add_dependencies(OverwriteSwig _thrusterDynamicEffector _vscmgStateEffector _reactionWheelStateEffector)
elseif(TARGET thrusterDynamicEffector)
    add_dependencies(OverwriteSwig thrusterDynamicEffector vscmgStateEffector reactionWheelStateEffector)
endif()
