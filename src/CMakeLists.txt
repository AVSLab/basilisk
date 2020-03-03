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

if(POLICY CMP0068)
  cmake_policy(SET CMP0068 NEW)
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

set(requires_protobuff_modules "vizInterface")
set(requires_zmq_modules "vizInterface")


set(requires_opencv_modules "houghCircles")
list(APPEND requires_opencv_modules "limbFinding")
list(APPEND requires_opencv_modules "camera")

set(requires_vulkan_modules "dynamics/SphericalHarmonicsSolvers/sphericalHarmonicsGPU/sphericalHarmonicsGPU.i"  "dynamics/SphericalHarmonicsSolvers/sphericalHarmonicsCPU/sphericalHarmonicsCPU.i")


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



#! create_symlinks : create symlinks to support files in source rather than keeping them in dist
# The function takes one required argument and N optional arguments.
# \argn: a list of supporting file paths
#
function(create_symlinks destinationPath)
    # Do nothing if building in-source
    if (${CMAKE_CURRENT_BINARY_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
        return()
    endif()

    foreach (filePath ${ARGN})
        get_filename_component(fileName ${filePath} NAME)

        # Get OS dependent path to use in `execute_process`
        file(TO_NATIVE_PATH "${destinationPath}/${fileName}" link)
        file(TO_NATIVE_PATH "${filePath}" target)

        # Delete the files from the dist directory, and instead just link them to the files in source
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

        # If the symlink is unsuccessful, then copy the file directly into dist
        if (NOT ${result} EQUAL 0)
            file( COPY ${target} DESTINATION ${destinationPath})
            message("Could not create symbolic link for: ${target} --> ${output}.  Copied instead")
        endif()

    endforeach(filePath)
endfunction(create_symlinks)





function(sub_dir_list result curdir)
  # Searches through all directories in source, and keeps those called GeneralModuleFiles
  FILE(GLOB_RECURSE children LIST_DIRECTORIES true RELATIVE ${curdir} ${curdir}/*/)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
        string(FIND ${curdir}/${child} "_GeneralModuleFiles" FOUND)
        string(FIND ${curdir}/${child} "__pycache__" FOUND_PY)
        string(FIND ${curdir}/${child} "_UnitTest" FOUND_UNIT)
        IF(${FOUND} GREATER -1 AND ${FOUND_PY} EQUAL -1 AND ${FOUND_UNIT} EQUAL -1)
            LIST(APPEND dirlist ${child})
        ENDIF()
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist} PARENT_SCOPE)
endfunction(sub_dir_list)


function(generate_package_libraries INIT_DIRECTORY)
    # Find all _GeneralModuleFiles and put them into library targets so they aren't rewrapped, built with every module.

    # Find all _GeneralModuleFiles directories
    sub_dir_list(LIB_DIRS ${INIT_DIRECTORY})
    foreach(LIB_DIR ${LIB_DIRS})

        # Determine the name of the parent directory of _GeneralModuleFiles (i.e. dynamics, environment, etc)
        get_filename_component(PARENT_DIR ${LIB_DIR} DIRECTORY) # Relative to the source directory (e.g. simulation/power)
        string(FIND ${PARENT_DIR} "/" DIR_NAME_IDX REVERSE)
        math(EXPR DIR_NAME_IDX "${DIR_NAME_IDX} + 1")# "simulation/"
        string(SUBSTRING ${PARENT_DIR} ${DIR_NAME_IDX} -1 PARENT_DIR_NAME) # Should be (e.g. power)

        # Set the library name (ex. dynamicsLib, environmentLib, etc)
        set(LIB_NAME "${PARENT_DIR_NAME}Lib")

        # Grab the library source files
        file(GLOB C_FILES
                "${LIB_DIR}/*.cpp" # SH Base Class
                "${LIB_DIR}/*.c" # SH Base Class
                "${LIB_DIR}/*.h"
            )

        # Grab the framework source files
        file(GLOB BSK_FWK_FILES
            "${CMAKE_SOURCE_DIR}/simulation/_GeneralModuleFiles/*.cpp" #Might not be needed
            "${CMAKE_SOURCE_DIR}/simulation/_GeneralModuleFiles/*.h"
            "${CMAKE_SOURCE_DIR}/simulation/_GeneralModuleFiles/*.c"
            )

        # Add Target
        add_library(${LIB_NAME} SHARED ${C_FILES} ${BSK_FWK_FILES})

        # Link all necessary libraries
        target_link_libraries(${LIB_NAME} SimUtilities)
        target_link_libraries(${LIB_NAME} SimMessaging)
        target_link_libraries(${LIB_NAME} AlgorithmMessaging)
        target_link_libraries(${LIB_NAME} ${PYTHON_LIBRARIES})
	    target_link_libraries(${LIB_NAME} ${CONAN_LIBS})

        # define build location, IDE generation specifications
        set_target_properties(${LIB_NAME} PROPERTIES FOLDER ${PARENT_DIR})
        set_target_properties(${LIB_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/Basilisk)
        set_target_properties(${LIB_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/Basilisk)
        set_target_properties(${LIB_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/Basilisk)
    endforeach()
endfunction(generate_package_libraries)

function(find_package_targets PKG_DIR ALL_TARGET_LIST)
	file(GLOB_RECURSE SWIG_TARGETS RELATIVE ${CMAKE_SOURCE_DIR} "${PKG_DIR}/*.i")
	set(${ALL_TARGET_LIST} ${SWIG_TARGETS} PARENT_SCOPE)
endfunction(find_package_targets)

macro(is_cpp_module C_FILES CPP_FILE_EXISTS)
    # If there are no files, use c++ mode.
    if(NOT C_FILES)
        set(${CPP_FILE_EXISTS} 1)
    endif()

    # If there are files and there is a .cpp file, use c++ mode
    foreach(C_FILE ${C_FILES})
        get_filename_component(FILE_EXT ${C_FILE} EXT)
        string(COMPARE EQUAL ${FILE_EXT} ".cpp" CPP_VAR)
        if(${CPP_VAR})
            set(${CPP_FILE_EXISTS} ${CPP_VAR})
            break()
        endif()
    endforeach()

    # If no C++ file found, check if there are any .c files or if the directory is header files only
    # If headers only, we use c++ mode (this is for the message directories)
    if(NOT ${CPP_VAR})
        set(HEADER_FILES_ONLY 1)
        foreach(C_FILE ${C_FILES})
            get_filename_component(FILE_EXT ${C_FILE} EXT)
            string(COMPARE EQUAL ${FILE_EXT} ".c" C_VAR)
            if(${C_VAR})
                set(${HEADER_FILES_ONLY} 0)
                break()
            endif()
        endforeach()
        set(${CPP_FILE_EXISTS} ${HEADER_FILES_ONLY})
    endif()
endmacro(is_cpp_module)

function(generate_package_targets TARGET_LIST LIB_DEP_LIST MODULE_DIR)

    find_package(SWIG REQUIRED)
    include(UseSwig REQUIRED)
    find_package(PythonLibs 3 REQUIRED)

    foreach(TARGET_FILE ${TARGET_LIST})
        get_filename_component(TARGET_NAME ${TARGET_FILE} NAME_WE)
        get_filename_component(PARENT_DIR ${TARGET_FILE} DIRECTORY)


        # TODO: Need to make custom CMakeLists.txt for each of these modules.
        if((${TARGET_NAME} IN_LIST requires_protobuff_modules) OR (${TARGET_NAME} IN_LIST requires_zmq_modules) OR (${TARGET_NAME} IN_LIST requires_opencv_modules))
               MESSAGE("SKIPPED: ${TARGET_NAME}")
               continue()
 	    endif()



        # Grab the target source files
        file(GLOB C_FILES
                "${PARENT_DIR}/*.c"
                "${PARENT_DIR}/*.cpp"
                "${PARENT_DIR}/*.h"
                "${PARENT_DIR}/*.rst"
            )

        set_property(SOURCE ${TARGET_FILE} PROPERTY USE_TARGET_INCLUDE_DIRECTORIES TRUE) # Allows for include paths in the .i files

        is_cpp_module("${C_FILES}" CPP_FILE_EXISTS) # Header only directories (message directories) will get set to c mode
        if(${CPP_FILE_EXISTS})
            set_property(SOURCE ${TARGET_FILE} PROPERTY CPLUSPLUS ON)
        endif()

        # Add Target
        swig_add_library(${TARGET_NAME}
                           LANGUAGE "python"
                           TYPE MODULE
                           SOURCES ${TARGET_FILE} ${C_FILES}
                           # TODO: Following commented lines will become default with future imports
                           #OUTFILE_DIR "${CMAKE_BINARY_DIR}/Basilisk/${PARENT_DIR}" #_wrap.c/.cxx file
                           #OUTPUT_DIR "${CMAKE_BINARY_DIR}/Basilisk/${PARENT_DIR}") # generated .py file
                           OUTFILE_DIR "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}" #_wrap.c/.cxx file
                           OUTPUT_DIR "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}") # generated .py file



        target_include_directories(${TARGET_NAME} PRIVATE ${PYTHON_INCLUDE_PATH}) # Exposes python.h to wrap.c(xx) file
        target_include_directories(${TARGET_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/${PARENT_DIR}) # Exposes module .h files to the PYTHON_wrap.c(xx) file (not located in src)


        #TODO: V2 change recommended -- messages should be in there own messaging folder at same level as sim/fsw/framework.
        target_include_directories(${TARGET_NAME} PRIVATE "${CMAKE_SOURCE_DIR}/simulation") # Exposes simMessages, simFswInterfaceMessages for easy includes
        target_include_directories(${TARGET_NAME} PRIVATE "${CMAKE_SOURCE_DIR}/fswAlgorithms") # Exposes fswMessages for easy includes
        target_include_directories(${TARGET_NAME} PRIVATE "${CMAKE_SOURCE_DIR}/simulation/_GeneralModuleFiles") # Exposes framework files for easy includes
        target_include_directories(${TARGET_NAME} PRIVATE "${CMAKE_SOURCE_DIR}/../libs") # Exposes third party library include folders


        # Link all necessary libraries
        foreach(LIB ${LIB_DEP_LIST})
            target_link_libraries(${TARGET_NAME} ${LIB}) 
        endforeach()

        # TODO: This is a better way to organize the dist olders, but it doesn't adhere to current python import structure
        # Would make (from Basilisk.simulation import SpacecraftPlus) --> (from Basilisk.simulation.dynamics.SpacecraftDynamics import SpacecraftPlus)
        #set_target_properties(${TARGET_NAME} PROPERTIES FOLDER ${PARENT_DIR}) # define folder in IDE
        #set_target_properties(${TARGET_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/Basilisk/${PARENT_DIR}) # Define the location for the .so file
        #set_target_properties(${TARGET_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/Basilisk/${PARENT_DIR})
        #set_target_properties(${TARGET_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/Basilisk/${PARENT_DIR})

        #
        set_target_properties(${TARGET_NAME} PROPERTIES FOLDER ${PARENT_DIR}) # define folder in IDE
        set_target_properties(${TARGET_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}) # Define the location for the .so file
        set_target_properties(${TARGET_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR})
        set_target_properties(${TARGET_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR})

        #Add an __init__.py for the resulting target .py file (the only case where there has been a custom python __init__.py is in radiation pressure)
        #file(WRITE "${CMAKE_BINARY_DIR}/Basilisk/${PARENT_DIR}/__init__.py" "# This __init__.py file for the ${TARGET_NAME} package is automatically generated by the build system
#from .${TARGET_NAME} import *")
    endforeach()
endfunction(generate_package_targets)













# Start of main projection configuration
if(USE_PYTHON3)
	find_package(PythonInterp 3 REQUIRED)
	find_package(PythonLibs 3 REQUIRED)
else()
	find_package(PythonInterp 2.7 REQUIRED)
	find_package(PythonLibs 2.7 REQUIRED)
endif()

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
	set(library_dependencies ${CMAKE_SOURCE_DIR}/../libs/cspice/lib/cspice_osx.a)

	SET(CMAKE_SKIP_BUILD_RPATH  FALSE)	# use, i.e. don't skip the full RPATH for the build tree
	SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE) # when building, don't use the install RPATH already (but later on when installing)
	SET(CMAKE_INSTALL_RPATH "${CMAKE_BINARY_DIR}/Basilisk")# the RPATH to be used when installing

	# don't add the automatically determined parts of the RPATH
	# which point to directories outside the build tree to the install RPATH
	SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
else()
	set(library_dependencies ${CMAKE_SOURCE_DIR}/../libs/cspice/lib/cspice.a)
	set(CMAKE_SKIP_BUILD_RPATH  FALSE)
	set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
	set(CMAKE_INSTALL_RPATH "\$ORIGIN/../../")
endif()

set_property(GLOBAL PROPERTY BUILT_LIB_LIST "SimUtilities;SimMessaging")
set_property(GLOBAL PROPERTY ALG_LIB_LIST "AlgorithmMessaging")
set_property(GLOBAL PROPERTY USE_FOLDERS ON)


# Targets that must be built first
add_subdirectory("simulation/utilities")
add_subdirectory("utilities") # has protobuffers included
add_subdirectory("simulation/architecture/messaging")
add_subdirectory("fswAlgorithms/messaging")
#add_subdirectory("topLevelModules")



# TODO: Currently the libraries generated works well except for the simulationLib which really is the architectureLib -- I'd like for these files to be moved.
# TODO: I'd call this generate_libraries(), because it really does find all of them.
# TODO: I'd like for all generated libraries to end up in a dist/Basilisk/lib folder rather than the /dist/Basilisk folder; however haven't found a way to use target_link_libraries() in a cross platform way to support this (needs full path and custom extensions --.dylib, .lib, .a, etc -- which gets messy in CMake).
generate_package_libraries("${CMAKE_SOURCE_DIR}") # This finds GeneralModuleFiles and generates a library of the parentDirectory name

set(ARCHITECTURE_LIBS simulationLib SimUtilities SimMessaging)

##############
# SIMULATION #
##############
# TODO: Move these commands into a CMakeList.txt in simulation (currently when I do this the targets arg in generate_package_targets gets interpreted differently
# TODO: Automate this: 1) Look for package directories (dynamics, environment),
#                      2) Find/Link any libraries associated with that package (dynamicsLib),
#                      3) Look for custom CMakeLists.txt to link additional libraries.
#

find_package_targets("${CMAKE_SOURCE_DIR}/simulation/dynamics" DYN_TARGETS) # Finds relative path of all .i files within the directory
find_package_targets("${CMAKE_SOURCE_DIR}/simulation/environment" ENV_TARGETS)
find_package_targets("${CMAKE_SOURCE_DIR}/simulation/onboardDataHandling" DATA_TARGETS)
find_package_targets("${CMAKE_SOURCE_DIR}/simulation/deviceInterface" DEVICE_INTERFACE_TARGETS)
find_package_targets("${CMAKE_SOURCE_DIR}/simulation/power" POWER_TARGETS)
find_package_targets("${CMAKE_SOURCE_DIR}/simulation/navigation" NAV_TARGETS)
find_package_targets("${CMAKE_SOURCE_DIR}/simulation/sensors" SENSORS_TARGETS)
find_package_targets("${CMAKE_SOURCE_DIR}/simulation/sim_synch" SIM_SYNCH_TARGETS)
find_package_targets("${CMAKE_SOURCE_DIR}/simulation/utilities" UTILITIES_TARGETS)
find_package_targets("${CMAKE_SOURCE_DIR}/simulation/utilitiesSelfCheck" UTILITIES_SELF_CHECK_TARGETS)
find_package_targets("${CMAKE_SOURCE_DIR}/simulation/vizInterface" VIZ_INTERFACE_TARGETS)





generate_package_targets("${DYN_TARGETS}" "dynamicsLib;${ARCHITECTURE_LIBS}" "simulation") # The quotations around DYN_TARGETS informs function that it is a list
generate_package_targets("${ENV_TARGETS}" "environmentLib;${ARCHITECTURE_LIBS};${library_dependencies}" "simulation") # TODO: include cspice for just spice_interface, not all packages
generate_package_targets("${DATA_TARGETS}" "onboardDataHandlingLib;${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${DEVICE_INTERFACE_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${POWER_TARGETS}" "powerLib;${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${NAV_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${SENSORS_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${SIM_SYNCH_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${UTILITIES_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${UTILITIES_SELF_CHECK_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${VIZ_INTERFACE_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")






##################
# FSW ALGORITHMS #
##################
find_package_targets("${CMAKE_SOURCE_DIR}/fswAlgorithms/attControl" ATT_CONTROL_TARGETS)
find_package_targets("${CMAKE_SOURCE_DIR}/fswAlgorithms/attDetermination" ATT_DET_TARGETS)
find_package_targets("${CMAKE_SOURCE_DIR}/fswAlgorithms/attGuidance" ATT_GUID_TARGETS)
find_package_targets("${CMAKE_SOURCE_DIR}/fswAlgorithms/dvGuidance" DV_GUID_TARGETS)
find_package_targets("${CMAKE_SOURCE_DIR}/fswAlgorithms/effectorInterfaces" EFF_INTERFACES_TARGETS)
find_package_targets("${CMAKE_SOURCE_DIR}/fswAlgorithms/formationFlying" FORM_FLYING_TARGETS)
find_package_targets("${CMAKE_SOURCE_DIR}/fswAlgorithms/fswUtilities" FSW_UTIL_TARGETS)
find_package_targets("${CMAKE_SOURCE_DIR}/fswAlgorithms/imageProcessing" IMAGE_PROC_TARGETS)
find_package_targets("${CMAKE_SOURCE_DIR}/fswAlgorithms/opticalNavigation" OPT_NAV_TARGETS)
find_package_targets("${CMAKE_SOURCE_DIR}/fswAlgorithms/rwConfigData" RW_CONFIG_TARGETS)
find_package_targets("${CMAKE_SOURCE_DIR}/fswAlgorithms/sensorInterfaces" SENSOR_INTERFACE_TARGETS)
find_package_targets("${CMAKE_SOURCE_DIR}/fswAlgorithms/transDetermination" TRANS_DET_TARGETS)
find_package_targets("${CMAKE_SOURCE_DIR}/fswAlgorithms/vehicleConfigData" VEH_CONFIG_TARGETS)


generate_package_targets("${ATT_CONTROL_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${ATT_DET_TARGETS}" "attDeterminationLib;${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${ATT_GUID_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${DV_GUID_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${EFF_INTERFACES_TARGETS}" "effectorInterfacesLib;${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${FORM_FLYING_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${FSW_UTIL_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${IMAGE_PROC_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${OPT_NAV_TARGETS}" "opticalNavigationLib;${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${RW_CONFIG_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${SENSOR_INTERFACE_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${TRANS_DET_TARGETS}" "transDeterminationLib;${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${VEH_CONFIG_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")


##################
# MESSAGES #
##################
find_package_targets("${CMAKE_SOURCE_DIR}/simulation/simMessages" SIM_MESSAGES_TARGETS)
find_package_targets("${CMAKE_SOURCE_DIR}/simulation/simFswInterfaceMessages" SIM_FSW_INTERFACE_TARGETS)

generate_package_targets("${SIM_MESSAGES_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${SIM_FSW_INTERFACE_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")

find_package_targets("${CMAKE_SOURCE_DIR}/fswAlgorithms/fswMessages" FSW_MESSAGES_TARGETS)
generate_package_targets("${FSW_MESSAGES_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")



##################
# ARCHITECTURE #
##################
find_package_targets("${CMAKE_SOURCE_DIR}/simulation/architecture" ARCHITECTURE_TARGETS)
generate_package_targets("${ARCHITECTURE_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")

find_package_targets("${CMAKE_SOURCE_DIR}/simulation/_GeneralModuleFiles" SIM_TARGETS)
generate_package_targets("${SIM_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")



#add_subdirectory("${CMAKE_SOURCE_DIR}/topLevelModules")
find_package_targets("${CMAKE_SOURCE_DIR}/topLevelModules/pyswice" PYSWICE_TARGETS) # Finds relative path of all .i files within the directory
generate_package_targets("${PYSWICE_TARGETS}" "${ARCHITECTURE_LIBS};${library_dependencies}" "topLevelModules")



##############################
#PYTHON PACKAGE CONFIGURATION#
##############################

# make the built package with an empty init file
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk")
file(WRITE "${CMAKE_BINARY_DIR}/Basilisk/__init__.py" "#empty init file written by the build")

#TODO: Iterate through all dist directories and add __init__.py's where they don't exist
file(GLOB_RECURSE DIST_DIRECTORIES LIST_DIRECTORIES true "${CMAKE_BINARY_DIR}/Basilisk/*" )
foreach(DIR ${DIST_DIRECTORIES})
    if(IS_DIRECTORY ${DIR})
        file(GLOB DIST_DIR_FILES RELATIVE ${DIR} "${DIR}/*.py")
        if(DIST_DIR_FILES)
            list(FIND DIST_DIR_FILES "__init__.py" INIT_FOUND)
            if(${INIT_FOUND} EQUAL -1)
                file(WRITE "${DIR}/__init__.py" "")
            endif()
        else()
            file(WRITE "${DIR}/__init__.py" "")
        endif()
    endif()
endforeach()




#TODO: Symlink the remainder of the .py and support files within src into the dist directory


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
