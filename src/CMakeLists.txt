cmake_minimum_required(VERSION 3.14)
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
list(APPEND CMAKE_MODULE_PATH "${CMAKE_BINARY_DIR}/conan")
list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})


list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_BINARY_DIR}/conan")


option(BUILD_VIZINTERFACE "Build VizInterface Module" ON)
option(BUILD_OPNAV "Build OpNav Modules" OFF)




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


function(add_message_headers)


	file(GLOB CMESSAGE_HEADER_FILES
	            "${CMAKE_SOURCE_DIR}/architecture/msgPayloadDefC/*.h"
	            "${EXTERNAL_MODULES_PATH}/msgPayloadDefC/*.h"
	        )
	file(GLOB CPPMESSAGE_HEADER_FILES
	            "${CMAKE_SOURCE_DIR}/architecture/msgPayloadDefCpp/*.h"
	            "${EXTERNAL_MODULES_PATH}/msgPayloadDefCpp/*.h"
	        )
	add_custom_target(msgPayloadDefC SOURCES ${CMESSAGE_HEADER_FILES})
	add_custom_target(msgPayloadDefCpp SOURCES ${CPPMESSAGE_HEADER_FILES})
	set_target_properties(msgPayloadDefC PROPERTIES FOLDER architecture)
	set_target_properties(msgPayloadDefCpp PROPERTIES FOLDER  architecture)
endfunction(add_message_headers)



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

# add header search directories
include_directories("${CMAKE_SOURCE_DIR}/../dist3/autoSource")
include_directories("${CMAKE_SOURCE_DIR}")


function(generate_package_libraries INIT_DIRECTORY AllLibs)
    # Find all _GeneralModuleFiles and put them into library targets so they aren't rewrapped, built with every module.

    # Find all _GeneralModuleFiles directories
    sub_dir_list(LIB_DIRS ${INIT_DIRECTORY})
    foreach(LIB_DIR ${LIB_DIRS})

        # Determine the name of the parent directory of _GeneralModuleFiles (i.e. dynamics, environment, etc)
        get_filename_component(PARENT_DIR ${LIB_DIR} DIRECTORY) # Relative to the source directory (e.g. simulation/power)
        string(FIND ${PARENT_DIR} "/" DIR_NAME_IDX REVERSE)
        math(EXPR DIR_NAME_IDX "${DIR_NAME_IDX} + 1")# "simulation/"
        string(SUBSTRING ${PARENT_DIR} ${DIR_NAME_IDX} -1 PARENT_DIR_NAME) # Should be "power"

        # Set the library name (ex. dynamicsLib, environmentLib, etc)
        set(LIB_NAME "${PARENT_DIR_NAME}Lib")
        #Added to build ExternalModulesLib 
        file(GLOB LIB_DIR RELATIVE ${CMAKE_SOURCE_DIR} "${INIT_DIRECTORY}/${LIB_DIR}")
        # Grab the library source files
        file(GLOB C_FILES
                "${LIB_DIR}/*.cpp"
                "${LIB_DIR}/*.c" 
                "${LIB_DIR}/*.h"
            )
        # Grab the framework source files
        file(GLOB BSK_FWK_FILES
            "${CMAKE_SOURCE_DIR}/architecture/_GeneralModuleFiles/*.cpp" #Might not be needed
            "${CMAKE_SOURCE_DIR}/architecture/_GeneralModuleFiles/*.h"
            "${CMAKE_SOURCE_DIR}/architecture/_GeneralModuleFiles/*.c"
            )

        # Add Target
        add_library(${LIB_NAME} SHARED ${C_FILES} ${BSK_FWK_FILES})

        #Add to list of library
        list(APPEND AllLibs ${LIB_NAME})

        # Link all necessary libraries
        target_link_libraries(${LIB_NAME} ArchitectureUtilities)
        target_link_libraries(${LIB_NAME} ModuleIdGenerator)
        target_link_libraries(${LIB_NAME} cMsgCInterface)
        target_link_libraries(${LIB_NAME} ${PYTHON_LIBRARIES})
        target_link_libraries(${LIB_NAME} Eigen3::Eigen3)

        # define build location, IDE generation specifications
        set_target_properties(${LIB_NAME} PROPERTIES FOLDER ${PARENT_DIR})
        set_target_properties(${LIB_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk")
        set_target_properties(${LIB_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk")
        set_target_properties(${LIB_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Basilisk")
        
        # Define the location for the executable part of the library (.dll) files (Windows Only) 
        # https://cmake.org/cmake/help/v3.17/manual/cmake-buildsystem.7.html#output-artifacts for details
        set_target_properties(${LIB_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk") 
        set_target_properties(${LIB_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk")
        set_target_properties(${LIB_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Basilisk")

        # Define the location of the import library file (.lib) needed for the .dll (Windows Only)
        set_target_properties(${LIB_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk") 
        set_target_properties(${LIB_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk")
        set_target_properties(${LIB_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Basilisk")
    endforeach()
    set(AllLibs ${AllLibs} PARENT_SCOPE) 
endfunction(generate_package_libraries)

function(find_package_targets PKG_DIR ALL_TARGET_LIST)
	file(GLOB_RECURSE SWIG_TARGETS RELATIVE ${CMAKE_SOURCE_DIR} "${PKG_DIR}/*.i")
	set(${ALL_TARGET_LIST} ${SWIG_TARGETS} PARENT_SCOPE)
endfunction(find_package_targets)

macro(is_cpp_module C_FILES RUN_CPP_MODE)
    set(${RUN_CPP_MODE} 0)

    # Check if there are C++ files
    set(CPP_FILE_EXISTS 0)
    foreach(C_FILE ${C_FILES})
        get_filename_component(FILE_EXT ${C_FILE} EXT)
        string(COMPARE EQUAL ${FILE_EXT} ".cpp" CPP_VAR)
        if(${CPP_VAR})
            set(CPP_FILE_EXISTS ${CPP_VAR})
            break()
        endif()
    endforeach()

    # If no C++ file found, check if there are any .c files 
    set(C_FILE_EXISTS 0)
    if(NOT ${CPP_FILE_EXISTS})
        foreach(C_FILE ${C_FILES})
            get_filename_component(FILE_EXT ${C_FILE} EXT)
            string(COMPARE EQUAL ${FILE_EXT} ".c" C_VAR_BOOL) # found .c files
            if(C_VAR_BOOL)
                set(C_FILE_EXISTS 1)
                break()
            endif()
        endforeach()
    endif()

    # If no C or C++ files exist -- the directory only contains header files. 
    # Check if this directory exists within FSWAlgorithms or Simulation
    set(HEADER_ONLY_IN_SIM 0)
    if(NOT ${CPP_FILE_EXISTS} AND NOT ${C_FILE_EXISTS})
        foreach(C_FILE ${C_FILES})
            get_filename_component(DIRECTORY_VAR ${C_FILE} DIRECTORY)
            string(FIND ${DIRECTORY_VAR} "fswAlgorithms" FSW_DIR_BOOL) # if the directory contains fswAlgorithms, value FSW_DIR_BOOL is 0
            if(FSW_DIR_BOOL EQUAL -1)
                set(HEADER_ONLY_IN_SIM 1)
                break()
            endif()
        endforeach()
    endif()

    # If there are 1) C++ files, 2) header only files in simulation directory, or 3) no C type files at all: Run swig in C++ mode
    if(${CPP_FILE_EXISTS} OR ${HEADER_ONLY_IN_SIM} OR NOT C_FILES)
        set(${RUN_CPP_MODE} 1)
    endif()
endmacro(is_cpp_module)

function(generate_package_targets TARGET_LIST LIB_DEP_LIST MODULE_DIR)

    find_package(SWIG REQUIRED)
    include(UseSWIG REQUIRED)
    find_package(PythonLibs 3 REQUIRED)
    
    if(TARGET_LIST)
        string(REPLACE "/" ";" DirList "${TARGET_LIST}")
        list(GET DirList 1 SUBMODULE)
    
        #It is redundant to compute simulation modules for each target.
        #This should be fixed when modules' directories are restructured.
        file(GLOB GEN_FILES
            "${CMAKE_SOURCE_DIR}/architecture/_GeneralModuleFiles/*.c"
            "${CMAKE_SOURCE_DIR}/architecture/_GeneralModuleFiles/*.cpp"
            "${CMAKE_SOURCE_DIR}/architecture/_GeneralModuleFiles/*.h"
            "${CMAKE_SOURCE_DIR}/architecture/_GeneralModuleFiles/*.i"
        )
    	if(${MODULE_DIR} STREQUAL "ExternalModules")
    		 file(GLOB GEN_FILES_LOCAL
                    "${EXTERNAL_MODULES_PATH}/${MODULE_DIR}/_GeneralModuleFiles/*.c"
                    "${EXTERNAL_MODULES_PATH}/${MODULE_DIR}/_GeneralModuleFiles/*.cpp"
                    "${EXTERNAL_MODULES_PATH}/${MODULE_DIR}/_GeneralModuleFiles/*.h"
                    "${EXTERNAL_MODULES_PATH}/${MODULE_DIR}/_GeneralModuleFiles/*.i"
                    )
    	
        else()
        	file(GLOB GEN_FILES_LOCAL
                    "${MODULE_DIR}/${SUBMODULE}/_GeneralModuleFiles/*.c"
                    "${MODULE_DIR}/${SUBMODULE}/_GeneralModuleFiles/*.cpp"
                    "${MODULE_DIR}/${SUBMODULE}/_GeneralModuleFiles/*.h"
                    "${MODULE_DIR}/${SUBMODULE}/_GeneralModuleFiles/*.i"
                    )
        endif()
    
    endif()
    foreach(TARGET_FILE ${TARGET_LIST})
        get_filename_component(TARGET_NAME ${TARGET_FILE} NAME_WE)
        get_filename_component(PARENT_DIR ${TARGET_FILE} DIRECTORY)
        
        # Collect any external dependencies
        set(CUSTOM_DEPENDENCIES)
        set(CUSTOM_DEPENDENCIES_HANDLED)
        if(EXISTS "${CMAKE_SOURCE_DIR}/${PARENT_DIR}/Custom.cmake")
          MESSAGE(STATUS "Including Custom Build Steps for: ${TARGET_FILE}")
          include("${PARENT_DIR}/Custom.cmake")
        endif()
        if(CUSTOM_DEPENDENCIES_HANDLED)
          continue()
        endif()


        # Grab the target source files
        file(GLOB C_FILES
                "${PARENT_DIR}/*.c"
                "${PARENT_DIR}/*.cpp"
                "${PARENT_DIR}/*.h"
                "${PARENT_DIR}/*.cmake"
                "${PARENT_DIR}/*.rst")
        file(GLOB SWIG_DEP
                "${PARENT_DIR}/*.c"
                "${PARENT_DIR}/*.cpp"
                "${PARENT_DIR}/*.h")

        set_property(SOURCE ${TARGET_FILE} PROPERTY USE_TARGET_INCLUDE_DIRECTORIES TRUE) # Allows for include paths in the .i files

        is_cpp_module("${C_FILES}" RUN_CPP_MODE) # Modules that don't contain C++ files or message directories within the fswAgorithms directory shouldn't run C++ mode
        if(${RUN_CPP_MODE})
            set_property(SOURCE ${TARGET_FILE} PROPERTY CPLUSPLUS ON)
        endif()

        # Add Target
        #To add dependency so that any change to source file will reswig the interface file
        #https://stackoverflow.com/questions/31007635/is-there-a-way-for-cmake-to-utilize-dependencies-generated-by-swig-mm
        #https://public.kitware.com/pipermail/cmake/2012-December/053045.html
        set(SWIG_MODULE_${TARGET_NAME}_EXTRA_DEPS ${SWIG_DEP} ${GEN_FILES} ${GEN_FILES_LOCAL})
        swig_add_library(${TARGET_NAME}
                           LANGUAGE "python"
                           TYPE MODULE
                           SOURCES ${TARGET_FILE} ${C_FILES}
                           # TODO: Following commented lines will become default with future imports
                           #OUTFILE_DIR "${CMAKE_BINARY_DIR}/Basilisk/${PARENT_DIR}" #_wrap.c/.cxx file
                           #OUTPUT_DIR "${CMAKE_BINARY_DIR}/Basilisk/${PARENT_DIR}") # generated .py file
                           OUTFILE_DIR "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}" #_wrap.c/.cxx file
                           OUTPUT_DIR "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}") # generated .py file


        # Add any custom user defined targets to the build aka external library
        foreach(CUSTOM_LIB ${CUSTOM_DEPENDENCIES})
          target_link_libraries(${TARGET_NAME} ${CUSTOM_LIB})
        endforeach()

        target_include_directories(${TARGET_NAME} PRIVATE ${PYTHON_INCLUDE_PATH}) # Exposes python.h to wrap.c(xx) file
        target_include_directories(${TARGET_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/${PARENT_DIR}) # Exposes module .h files to the PYTHON_wrap.c(xx) file (not located in src)

        target_include_directories(${TARGET_NAME} PRIVATE "${CMAKE_SOURCE_DIR}/architecture/_GeneralModuleFiles") # Exposes framework files for easy includes
        target_include_directories(${TARGET_NAME} PRIVATE "${CMAKE_SOURCE_DIR}/../libs") # Exposes third party library include folders

        # Link all necessary libraries
        foreach(LIB ${LIB_DEP_LIST})
            target_link_libraries(${TARGET_NAME} ${LIB}) 
        endforeach()

        if(${MODULE_DIR} STREQUAL "ExternalModules")
        	set_target_properties(${TARGET_NAME} PROPERTIES FOLDER ${MODULE_DIR})
        else()
        	set_target_properties(${TARGET_NAME} PROPERTIES FOLDER ${PARENT_DIR}) # define folder in IDE
        endif()
        # Define the location for the .so / .dylib file
        set_target_properties(${TARGET_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}") 
        set_target_properties(${TARGET_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}")
        set_target_properties(${TARGET_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}")

        # Define the location for the executable part of the library (.dll) files (Windows Only) 
        # https://cmake.org/cmake/help/v3.17/manual/cmake-buildsystem.7.html#output-artifacts for details
        set_target_properties(${TARGET_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}") 
        set_target_properties(${TARGET_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}")
        set_target_properties(${TARGET_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}")

        # Define the location of the import library file (.lib) needed for the .dll (Windows Only)
        set_target_properties(${TARGET_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}") 
        set_target_properties(${TARGET_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}")
        set_target_properties(${TARGET_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}")
    endforeach()
endfunction(generate_package_targets)






# Start of main projection configuration
find_package(PythonInterp 3 REQUIRED)
find_package(PythonLibs 3 REQUIRED)

#TODO: Remove the global link-libraries call
find_package(Eigen3 CONFIG REQUIRED)
# The following lines are necessary to prevent Xcode from issuing a warning about Eigen not being within the search path.
# Related to Eigen being a header only library.
if(DEFINED Eigen3_INCLUDE_DIRS_DEBUG)
    set(DIRECTORIES "$<$<CONFIG:Debug>:${Eigen3_INCLUDE_DIRS_DEBUG}>"
                                     "$<$<CONFIG:Release>:${Eigen3_INCLUDE_DIRS_DEBUG}>"
                                     "$<$<CONFIG:RelWithDebInfo>:${Eigen3_INCLUDE_DIRS_DEBUG}>"
                                     "$<$<CONFIG:MinSizeRel>:${Eigen3_INCLUDE_DIRS_DEBUG}>"
                                     )
else()
    set(DIRECTORIES "$<$<CONFIG:Debug>:${Eigen3_INCLUDE_DIRS_RELEASE}>"
                                     "$<$<CONFIG:Release>:${Eigen3_INCLUDE_DIRS_RELEASE}>"
                                     "$<$<CONFIG:RelWithDebInfo>:${Eigen3_INCLUDE_DIRS_RELEASE}>"
                                     "$<$<CONFIG:MinSizeRel>:${Eigen3_INCLUDE_DIRS_RELEASE}>")
endif()
set_property(TARGET Eigen3::Eigen3 PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${DIRECTORIES})
link_libraries(Eigen3::Eigen3)



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
    add_definitions(/wd"5208")
    add_definitions(/wd"4267")
    add_definitions(/bigobj)
    # Make sure we are using Multi-Threaded run time library
    foreach(flag
        CMAKE_C_FLAGS
        CMAKE_C_FLAGS_DEBUG
        CMAKE_C_FLAGS_RELEASE
        CMAKE_CXX_FLAGS
        CMAKE_CXX_FLAGS_DEBUG
        CMAKE_CXX_FLAGS_RELEASE)
        string(REPLACE "/D_DEBUG" "" "${flag}" "${${flag}}")
        string(REPLACE "/MD" "${CONAN_LINK_RUNTIME_MULTI}" "${flag}" "${${flag}}")
        string(REPLACE "/MDd" "${CONAN_LINK_RUNTIME_MULTI}" "${flag}" "${${flag}}")
        string(REPLACE "/MTd" "${CONAN_LINK_RUNTIME_MULTI}" "${flag}" "${${flag}}")
        set("${flag}" "${${flag}} /EHsc")
    endforeach()
elseif(APPLE)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -gdwarf-3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -gdwarf-3 -std=c++11 -stdlib=libc++")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -gdwarf-3")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}  -Wall")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}  -Wall")
    if(${CMAKE_GENERATOR} MATCHES "Xcode")
        set(CMAKE_XCODE_GENERATE_SCHEME YES)
    endif()
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

# TODO: Move the cspice library extension into a custom pyswice CMakeList.txt
# Manually create list of libraries depending on system
if(WIN32)
    set(CMAKE_MSVCIDE_RUN_PATH "${CMAKE_BINARY_DIR}/Basilisk")
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS YES)
	set(library_dependencies
      	${CMAKE_SOURCE_DIR}/../libs/cspice/lib/cspice_win${_arch_suffix}.lib
	)
        # fix for opencv not finding conan gflags for opencv sfm lib on windows
        # when the finding is fixed, the following line should be removed
        # https://github.com/conan-community/community/issues/210 
        list(REMOVE_ITEM CONAN_LIBS opencv_sfm411d multiview)

elseif(APPLE)
    if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm64")
	    set(library_dependencies ${CMAKE_SOURCE_DIR}/../libs/cspice/lib/cspice_osx_arm64.a)
	else()
		set(library_dependencies ${CMAKE_SOURCE_DIR}/../libs/cspice/lib/cspice_osx.a)
    endif()


	SET(CMAKE_SKIP_BUILD_RPATH  FALSE)	# use, i.e. don't skip the full RPATH for the build tree
	SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE) # when building, don't use the install RPATH already (but later on when installing)
	SET(CMAKE_INSTALL_RPATH "${CMAKE_BINARY_DIR}/Basilisk")# the RPATH to be used when installing

	# don't add the automatically determined parts of the RPATH
	# which point to directories outside the build tree to the install RPATH
	SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
else()
    if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "^(arm64|aarch64)$")
        set(library_dependencies ${CMAKE_SOURCE_DIR}/../libs/cspice/lib/cspice_arm64.a)
    else()
        set(library_dependencies ${CMAKE_SOURCE_DIR}/../libs/cspice/lib/cspice.a)
    endif()
    set(CMAKE_SKIP_BUILD_RPATH  FALSE)
	set(CMAKE_INSTALL_RPATH "\$ORIGIN/../../")
endif()

set_property(GLOBAL PROPERTY BUILT_LIB_LIST "ArchitectureUtilities")
set_property(GLOBAL PROPERTY USE_FOLDERS ON)


# Targets that must be built first
add_subdirectory("architecture/utilities")
add_subdirectory("utilities") # has protobuffers included
add_subdirectory("architecture/messaging/cMsgCInterface")
add_subdirectory("architecture/messaging/")



# TODO: I'd call this generate_libraries(), because it really does find all of them.
# TODO: I'd like for all generated libraries to end up in a dist/Basilisk/lib folder rather than the /dist/Basilisk folder; however haven't found a way to use target_link_libraries() in a cross platform way to support this (needs full path and custom extensions --.dylib, .lib, .a, etc -- which gets messy in CMake).
set(AllLibs "")
generate_package_libraries("${CMAKE_SOURCE_DIR}" "${AllLibs}") # This finds GeneralModuleFiles and generates a library of the parentDirectory name

set(ARCHITECTURE_LIBS architectureLib ArchitectureUtilities ModuleIdGenerator cMsgCInterface)

##############
# SIMULATION #
##############
# TODO: Move the following commands into a seperate CMakeList.txt s.t. this file is just configuration (problem: currently when I do this the targets argument in generate_package_targets gets interpreted differently)
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
find_package_targets("${CMAKE_SOURCE_DIR}/simulation/simSynch" SIM_SYNCH_TARGETS)
find_package_targets("${CMAKE_SOURCE_DIR}/simulation/vizard" VIZ_INTERFACE_TARGETS)
find_package_targets("${CMAKE_SOURCE_DIR}/simulation/thermal" THERMAL_TARGETS)

if(NOT "${EXTERNAL_MODULES_PATH}" STREQUAL "")
	string(REPLACE "\\" "/" EXTERNAL_MODULES_PATH "${EXTERNAL_MODULES_PATH}")
	include_directories("${EXTERNAL_MODULES_PATH}")
	find_package_targets("${EXTERNAL_MODULES_PATH}" EXTERNAL_MODULE_TARGETS)
	if(EXISTS "${EXTERNAL_MODULES_PATH}/ExternalModules/_GeneralModuleFiles")
		generate_package_libraries("${EXTERNAL_MODULES_PATH}" "${AllLibs}")
	endif()
	generate_package_targets("${EXTERNAL_MODULE_TARGETS}" "${AllLibs}" "ExternalModules")
endif(NOT "${EXTERNAL_MODULE_PATH}" STREQUAL "")

generate_package_targets("${DYN_TARGETS}" "dynamicsLib;${ARCHITECTURE_LIBS}" "simulation") # The quotations around DYN_TARGETS informs function that it is a list
generate_package_targets("${ENV_TARGETS}" "environmentLib;${ARCHITECTURE_LIBS};${library_dependencies}" "simulation") # TODO: include cspice for just spiceInterface, not all packages
generate_package_targets("${DATA_TARGETS}" "onboardDataHandlingLib;${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${DEVICE_INTERFACE_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${POWER_TARGETS}" "powerLib;${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${NAV_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${SENSORS_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${SIM_SYNCH_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${VIZ_INTERFACE_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${THERMAL_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")



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
find_package_targets("${CMAKE_SOURCE_DIR}/fswAlgorithms/smallBodyNavigation" SMALLBODY_NAV_TARGETS)
find_package_targets("${CMAKE_SOURCE_DIR}/fswAlgorithms/orbitControl" ORBIT_CONTROL_TARGETS)
find_package_targets("${CMAKE_SOURCE_DIR}/fswAlgorithms/transDetermination" TRANS_DET_TARGETS)
find_package_targets("${CMAKE_SOURCE_DIR}/fswAlgorithms/vehicleConfigData" VEH_CONFIG_TARGETS)


generate_package_targets("${ATT_CONTROL_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${ATT_DET_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${ATT_GUID_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${DV_GUID_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${EFF_INTERFACES_TARGETS}" "effectorInterfacesLib;${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${FORM_FLYING_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${FSW_UTIL_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${IMAGE_PROC_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${OPT_NAV_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${RW_CONFIG_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${SENSOR_INTERFACE_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${SMALLBODY_NAV_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${ORBIT_CONTROL_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${TRANS_DET_TARGETS}" "transDeterminationLib;${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${VEH_CONFIG_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")


####################
# TEMPLATE MODULES #
####################
find_package_targets("${CMAKE_SOURCE_DIR}/moduleTemplates" TEMPLATE_MODULES_TARGETS)

generate_package_targets("${TEMPLATE_MODULES_TARGETS}" "${ARCHITECTURE_LIBS};" "moduleTemplates")


##################
# MESSAGES #
##################
add_message_headers()


##################
# ARCHITECTURE #
##################
find_package_targets("${CMAKE_SOURCE_DIR}/architecture" ARCHITECTURE_TARGETS)
generate_package_targets("${ARCHITECTURE_TARGETS}" "${ARCHITECTURE_LIBS};" "architecture")


###########
# Pyswice #
###########
find_package_targets("${CMAKE_SOURCE_DIR}/topLevelModules/pyswice" PYSWICE_TARGETS) # Finds relative path of all .i files within the directory
generate_package_targets("${PYSWICE_TARGETS}" "${ARCHITECTURE_LIBS};${library_dependencies}" "topLevelModules")



##############################
#PYTHON PACKAGE CONFIGURATION#
##############################

# Must make the build directories first, so that cmake can insert empty init files before build (linux specific need)
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk")
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/architecture")
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/simulation")
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/fswAlgorithms")

if(WIN32 AND (PYTHON_VERSION GREATER 3.7))
	file(WRITE "${CMAKE_BINARY_DIR}/Basilisk/__init__.py" "#init file written by the build\n"
                                                           "import sys, os\n"
                                                           "from Basilisk import __path__\n"
                                                           "bskPath = __path__[0]\n"
                                                           "os.add_dll_directory(bskPath)\n")
else()
	file(WRITE "${CMAKE_BINARY_DIR}/Basilisk/__init__.py" "#empty init file written by the build")
endif()


#TODO: Iterate through all dist directories and add __init__.py's where they don't exist
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/topLevelModules")
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




#symlink utilities into the dist directory
# TODO: consider making the utilities directory it's own module with associated .i file and many `%pythoncode` directives.
file(GLOB pythonModules
	"${CMAKE_SOURCE_DIR}/utilities/*.py"
	"${CMAKE_SOURCE_DIR}/utilities/**")
create_symlinks("${CMAKE_BINARY_DIR}/Basilisk/utilities" ${pythonModules})

#symlink into package the supportData files to keep
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/supportData")
file(GLOB dataFiles "${CMAKE_SOURCE_DIR}/../supportData/*")
create_symlinks("${CMAKE_BINARY_DIR}/Basilisk/supportData" ${dataFiles})


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
