cmake_minimum_required(VERSION 3.26)
# cmake>=3.18 is required for FindPython3 to work. See https://github.com/pypa/manylinux/issues/613
# cmake>=3.26 is required for FindPython3 SABIModule.
# XXX: Some versions of CMake around 3.21~3.26 crash CMake on Windows. Reason and range is still TBD.



project("basilisk")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE
      "Debug"
      CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif(NOT CMAKE_BUILD_TYPE)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_BINARY_DIR}/conan")
list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})

list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_BINARY_DIR}/conan")

include(bskTargetExcludeBuildOptions)

# Find necessary packages
find_package(Python3 ${PYTHON_VERSION} EXACT REQUIRED COMPONENTS Interpreter Development.Module OPTIONAL_COMPONENTS Development.SABIModule)

# check for deprecated python 3.8
if(Python3_VERSION VERSION_LESS "3.9")
    message(WARNING "You are using Python ${Python3_VERSION}, but support for Python < 3.9 is deprecated and will be removed in April 2026.")
endif()

# Find SWIG early (no version requirement yet)
find_package(SWIG REQUIRED COMPONENTS python)

# Check both versions
if(Python3_VERSION VERSION_GREATER_EQUAL 3.12)
  if(SWIG_VERSION VERSION_LESS "4.2.0")
    message(FATAL_ERROR
      "Python ${Python3_VERSION} requires SWIG 4.2.0 or newer, but found SWIG ${SWIG_VERSION}."
    )
  endif()
endif()

# Now safely require SWIG in the appropriate version range
if(Python3_VERSION VERSION_GREATER_EQUAL 3.12)
  find_package(SWIG 4.2...<5 REQUIRED COMPONENTS python)
else()
  find_package(SWIG 4.0...<5 REQUIRED COMPONENTS python)
endif()
include(${SWIG_USE_FILE})

# Set the Python module library to be linked to.
set(PYTHON3_MODULE Python3::Module)
if(PY_LIMITED_API AND NOT PY_LIMITED_API STREQUAL "")
  message(STATUS "Requested Py_LIMITED_API=${PY_LIMITED_API}")
  message(STATUS "(Refer to https://docs.python.org/3/c-api/stable.html#limited-c-api for details)")

  if(${SWIG_VERSION} VERSION_LESS 4.2)
    message(FATAL_ERROR "Could not find a suitable version of SWIG (found ${SWIG_VERSION}, need SWIG>=4.2.0)")
  endif()

  if(NOT Python3_Development.SABIModule_FOUND)
    message(FATAL_ERROR "Could not find the required Python3 Stable ABI module!")
  endif()

  # Set the Python API to limited mode, to provide wider ABI
  # compatibility (basically, we can build a single Python wheel to
  # support all future Python versions).
  # See https://docs.python.org/3/c-api/stable.html
  # NOTE: Swig 4.2.0 is required, see https://github.com/swig/swig/pull/2727
  add_definitions("-DPy_LIMITED_API=${PY_LIMITED_API}")  # Support for current Python version

  # Force libraries to link to the Stable ABI module instead.
  # This especially fixes an issue with Windows, because
  set(PYTHON3_MODULE Python3::SABIModule)
endif()

# ! create_symlinks : create symlinks to support files in source rather than keeping them in dist The function takes one
# required argument and N optional arguments. \argn: a list of supporting file paths
#
function(create_symlinks destinationPath)
  # Do nothing if building in-source
  if(${CMAKE_CURRENT_BINARY_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
    return()
  endif()

  foreach(filePath ${ARGN})
    get_filename_component(fileName ${filePath} NAME)

    # Get OS dependent path to use in `execute_process`
    file(TO_NATIVE_PATH "${destinationPath}/${fileName}" link)
    file(TO_NATIVE_PATH "${filePath}" target)

    # Delete the files from the dist directory, and instead just link them to the files in source
    file(REMOVE ${destinationPath}/${fileName})

    if(UNIX)
      set(command ln -s ${target} ${link})
    elseif(IS_DIRECTORY ${target})
      message("Creating Directory Symbolic Link")
      set(command cmd.exe /c mklink /D ${link} ${target})
    else()
      # message(target)
      set(command cmd.exe /c mklink ${link} ${target})
    endif()

    execute_process(
      COMMAND ${command}
      RESULT_VARIABLE result
      ERROR_VARIABLE output)

    # If the symlink is unsuccessful, then copy the file directly into dist
    if(NOT ${result} EQUAL 0)
      file(COPY ${target} DESTINATION ${destinationPath})
      message("Could not create symbolic link for: ${target} --> ${output}.  Copied instead")
    endif()

  endforeach(filePath)
endfunction(create_symlinks)

function(add_message_headers)
  file(GLOB CMESSAGE_HEADER_FILES "${CMAKE_SOURCE_DIR}/architecture/msgPayloadDefC/*.h"
       "${EXTERNAL_MODULES_PATH}/msgPayloadDefC/*.h")
  file(GLOB CPPMESSAGE_HEADER_FILES "${CMAKE_SOURCE_DIR}/architecture/msgPayloadDefCpp/*.h"
       "${EXTERNAL_MODULES_PATH}/msgPayloadDefCpp/*.h")
  add_custom_target(msgPayloadDefC SOURCES ${CMESSAGE_HEADER_FILES})
  add_custom_target(msgPayloadDefCpp SOURCES ${CPPMESSAGE_HEADER_FILES})
  set_target_properties(msgPayloadDefC PROPERTIES FOLDER architecture)
  set_target_properties(msgPayloadDefCpp PROPERTIES FOLDER architecture)
endfunction(add_message_headers)

function(sub_dir_list result curdir)
  # Searches through all directories in source, and keeps those called GeneralModuleFiles
  file(
    GLOB_RECURSE children
    LIST_DIRECTORIES true
    RELATIVE ${curdir}
    ${curdir}/*/)
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
      string(FIND ${curdir}/${child} "_GeneralModuleFiles" FOUND)
      string(FIND ${curdir}/${child} "__pycache__" FOUND_PY)
      string(FIND ${curdir}/${child} "_UnitTest" FOUND_UNIT)
      if(${FOUND} GREATER -1
         AND ${FOUND_PY} EQUAL -1
         AND ${FOUND_UNIT} EQUAL -1)
        list(APPEND dirlist ${child})
      endif()
    endif()
  endforeach()
  set(${result}
      ${dirlist}
      PARENT_SCOPE)
endfunction(sub_dir_list)

# add header search directories
include_directories("${CMAKE_BINARY_DIR}/autoSource")
include_directories("${CMAKE_SOURCE_DIR}")

function(generate_package_libraries INIT_DIRECTORY AllLibs)
  # Find all _GeneralModuleFiles and put them into library targets so they aren't rewrapped, built with every module.

  # Find all _GeneralModuleFiles directories
  sub_dir_list(LIB_DIRS ${INIT_DIRECTORY})
  foreach(LIB_DIR ${LIB_DIRS})

    # Determine the name of the parent directory of _GeneralModuleFiles (i.e. dynamics, environment, etc)
    get_filename_component(PARENT_DIR ${LIB_DIR} DIRECTORY) # Relative to the source directory (e.g. simulation/power)
    string(FIND ${PARENT_DIR} "/" DIR_NAME_IDX REVERSE)
    math(EXPR DIR_NAME_IDX "${DIR_NAME_IDX} + 1") # "simulation/"
    string(SUBSTRING ${PARENT_DIR} ${DIR_NAME_IDX} -1 PARENT_DIR_NAME) # Should be "power"

    if(${PARENT_DIR_NAME} IN_LIST EXCLUDED_BSK_LIBRARIES)
      message("Skipped Package Library: ${PARENT_DIR_NAME}")
        continue()
    endif()

    # Set the library name (ex. dynamicsLib, environmentLib, etc)
    set(LIB_NAME "${PARENT_DIR_NAME}Lib")
    # Added to build ExternalModulesLib
    file(
      GLOB LIB_DIR
      RELATIVE ${CMAKE_SOURCE_DIR}
      "${INIT_DIRECTORY}/${LIB_DIR}")
    # Grab the library source files
    file(GLOB C_FILES "${LIB_DIR}/*.cpp" "${LIB_DIR}/*.c" "${LIB_DIR}/*.h")
    # Grab the framework source files
    file(GLOB BSK_FWK_FILES "${CMAKE_SOURCE_DIR}/architecture/_GeneralModuleFiles/*.cpp" # Might not be needed
         "${CMAKE_SOURCE_DIR}/architecture/_GeneralModuleFiles/*.h"
         "${CMAKE_SOURCE_DIR}/architecture/_GeneralModuleFiles/*.c")

    # Add Target
    add_library(${LIB_NAME} SHARED ${C_FILES} ${BSK_FWK_FILES})

    # Add to list of library
    list(APPEND AllLibs ${LIB_NAME})

    # If the library directory has a Custom.cmake, include that
    if(EXISTS "${CMAKE_SOURCE_DIR}/${LIB_DIR}/Custom.cmake")
      message(STATUS "Including custom Custom.cmake for package lib: ${LIB_DIR}")
      include("${CMAKE_SOURCE_DIR}/${LIB_DIR}/Custom.cmake")
    endif()

    # Link all necessary libraries
    target_link_libraries(${LIB_NAME} PRIVATE ModuleIdGenerator)
    target_link_libraries(${LIB_NAME} PRIVATE Eigen3::Eigen3)
    if(${PARENT_DIR_NAME} STREQUAL "mujocoDynamics")
      target_link_libraries(${LIB_NAME} PUBLIC dynamicsLib)
      # dynamicsLib already has ArchitectureUtilities and cMsgCInterface
      # publicly linked, so no need to add them (again) for mujocoDynamics
    else()
      target_link_libraries(${LIB_NAME} PUBLIC ArchitectureUtilities)
      target_link_libraries(${LIB_NAME} PUBLIC cMsgCInterface)
    endif()

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
  set(AllLibs
      ${AllLibs}
      PARENT_SCOPE)
endfunction(generate_package_libraries)

function(find_package_targets PKG_DIR ALL_TARGET_LIST)
  file(
    GLOB_RECURSE SWIG_TARGETS
    RELATIVE ${CMAKE_SOURCE_DIR}
    "${PKG_DIR}/*.i")
  set(${ALL_TARGET_LIST}
      ${SWIG_TARGETS}
      PARENT_SCOPE)
endfunction(find_package_targets)

function(generate_package_targets TARGET_LIST LIB_DEP_LIST MODULE_DIR)

  if(TARGET_LIST)
    string(REPLACE "/" ";" DirList "${TARGET_LIST}")
    list(GET DirList 1 SUBMODULE)

    # It is redundant to compute simulation modules for each target. This should be fixed when modules' directories are
    # restructured.
    file(
      GLOB
      GEN_FILES
      "${CMAKE_SOURCE_DIR}/architecture/_GeneralModuleFiles/*.c"
      "${CMAKE_SOURCE_DIR}/architecture/_GeneralModuleFiles/*.cpp"
      "${CMAKE_SOURCE_DIR}/architecture/_GeneralModuleFiles/*.h"
      "${CMAKE_SOURCE_DIR}/architecture/_GeneralModuleFiles/*.i")

    # mujocoDynamics is special in that it also gets to see the
    # dynamics/_GeneralModuleFiles headers
    if(${MODULE_DIR} STREQUAL "simulation/mujocoDynamics")
      file(
        GLOB
        GEN_FILES_DYNAMICS
        "${CMAKE_SOURCE_DIR}/simulation/dynamics/_GeneralModuleFiles/*.c"
        "${CMAKE_SOURCE_DIR}/simulation/dynamics/_GeneralModuleFiles/*.cpp"
        "${CMAKE_SOURCE_DIR}/simulation/dynamics/_GeneralModuleFiles/*.h"
        "${CMAKE_SOURCE_DIR}/simulation/dynamics/_GeneralModuleFiles/*.i"
      )
      list (APPEND GEN_FILES ${GEN_FILES_DYNAMICS})
    endif()

    if(${MODULE_DIR} STREQUAL "ExternalModules")
      file(
        GLOB
        GEN_FILES_LOCAL
        "${EXTERNAL_MODULES_PATH}/${MODULE_DIR}/_GeneralModuleFiles/*.c"
        "${EXTERNAL_MODULES_PATH}/${MODULE_DIR}/_GeneralModuleFiles/*.cpp"
        "${EXTERNAL_MODULES_PATH}/${MODULE_DIR}/_GeneralModuleFiles/*.h"
        "${EXTERNAL_MODULES_PATH}/${MODULE_DIR}/_GeneralModuleFiles/*.i")
    else()
      file(GLOB GEN_FILES_LOCAL "${MODULE_DIR}/${SUBMODULE}/_GeneralModuleFiles/*.c"
           "${MODULE_DIR}/${SUBMODULE}/_GeneralModuleFiles/*.cpp" "${MODULE_DIR}/${SUBMODULE}/_GeneralModuleFiles/*.h"
           "${MODULE_DIR}/${SUBMODULE}/_GeneralModuleFiles/*.i")
    endif()
  endif()

  foreach(TARGET_FILE ${TARGET_LIST})
    get_filename_component(TARGET_NAME ${TARGET_FILE} NAME_WE)
    get_filename_component(PARENT_DIR ${TARGET_FILE} DIRECTORY)

    if(${TARGET_NAME} IN_LIST EXCLUDED_BSK_TARGETS)
      message("Skipped Target: ${TARGET_NAME}")
        continue()
    endif()

    # Grab the target source files
    file(
      GLOB
      C_FILES
      "${PARENT_DIR}/*.c"
      "${PARENT_DIR}/*.cpp"
      "${PARENT_DIR}/*.h"
      "${PARENT_DIR}/*.cmake"
      "${PARENT_DIR}/*.rst")
    file(GLOB SWIG_DEP "${PARENT_DIR}/*.c" "${PARENT_DIR}/*.cpp" "${PARENT_DIR}/*.h")

    set_property(SOURCE ${TARGET_FILE} PROPERTY USE_TARGET_INCLUDE_DIRECTORIES TRUE) # Allows for include paths in the
                                                                                     # .i files
    set_property(SOURCE ${TARGET_FILE} PROPERTY CPLUSPLUS ON)

    # Add Target To add dependency so that any change to source file will reswig the interface file
    # https://stackoverflow.com/questions/31007635/is-there-a-way-for-cmake-to-utilize-dependencies-generated-by-swig-mm
    # https://public.kitware.com/pipermail/cmake/2012-December/053045.html
    set(SWIG_MODULE_${TARGET_NAME}_EXTRA_DEPS ${SWIG_DEP} ${GEN_FILES} ${GEN_FILES_LOCAL})
    swig_add_library(
      ${TARGET_NAME}
      LANGUAGE "python"
      TYPE MODULE
      SOURCES ${TARGET_FILE}
              ${C_FILES}
              # TODO: Following commented lines will become default with future imports OUTFILE_DIR
              # "${CMAKE_BINARY_DIR}/Basilisk/${PARENT_DIR}" #_wrap.c/.cxx file OUTPUT_DIR
              # "${CMAKE_BINARY_DIR}/Basilisk/${PARENT_DIR}") # generated .py file
              OUTFILE_DIR
              "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}" # _wrap.c/.cxx file
              OUTPUT_DIR
              "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}") # generated .py file

    # Collect any external dependencies
    if(EXISTS "${CMAKE_SOURCE_DIR}/${PARENT_DIR}/Custom.cmake")
      message(STATUS "Including custom Custom.cmake for: ${TARGET_FILE}")
      include("${PARENT_DIR}/Custom.cmake")
    endif()
    # Also look for a Custom.cmake on the _GeneralModuleFiles
    if(EXISTS "${MODULE_DIR}/${SUBMODULE}/_GeneralModuleFiles/Custom.cmake")
      message(STATUS "Including custom Custom.cmake for: ${TARGET_FILE}")
      include("${MODULE_DIR}/${SUBMODULE}/_GeneralModuleFiles/Custom.cmake")
    endif()

    target_include_directories(${TARGET_NAME} PRIVATE ${Python3_INCLUDE_DIRS}) # Exposes python.h to wrap.c(xx) file
    target_include_directories(
      ${TARGET_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/${PARENT_DIR}) # Exposes module .h files to the PYTHON_wrap.c(xx) file
                                                                # (not located in src)

    target_include_directories(${TARGET_NAME} PRIVATE "${CMAKE_SOURCE_DIR}/architecture/_GeneralModuleFiles"
    )# Exposes framework files for easy includes
    target_include_directories(${TARGET_NAME} PRIVATE "${CMAKE_SOURCE_DIR}/../libs") # Exposes third party library
                                                                                     # include folders

    # Link all necessary libraries
    foreach(LIB ${LIB_DEP_LIST})
      target_link_libraries(${TARGET_NAME} PRIVATE ${LIB})
    endforeach()

    target_link_libraries(${TARGET_NAME} PRIVATE ${PYTHON3_MODULE})
    if(PY_LIMITED_API AND NOT PY_LIMITED_API STREQUAL "")
      target_compile_definitions(${TARGET_NAME} PRIVATE "Py_LIMITED_API=${PY_LIMITED_API}")
    endif()

    if(${MODULE_DIR} STREQUAL "ExternalModules")
      set_target_properties(${TARGET_NAME} PROPERTIES FOLDER ${MODULE_DIR})
    else()
      set_target_properties(${TARGET_NAME} PROPERTIES FOLDER ${PARENT_DIR}) # define folder in IDE
    endif()
    # Define the location for the .so / .dylib file
    set_target_properties(${TARGET_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY
                                                    "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}")
    set_target_properties(${TARGET_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG
                                                    "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}")
    set_target_properties(${TARGET_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE
                                                    "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}")

    # Define the location for the executable part of the library (.dll) files (Windows Only)
    # https://cmake.org/cmake/help/v3.17/manual/cmake-buildsystem.7.html#output-artifacts for details
    set_target_properties(${TARGET_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY
                                                    "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}")
    set_target_properties(${TARGET_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG
                                                    "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}")
    set_target_properties(${TARGET_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE
                                                    "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}")

    # Define the location of the import library file (.lib) needed for the .dll (Windows Only)
    set_target_properties(${TARGET_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY
                                                    "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}")
    set_target_properties(${TARGET_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG
                                                    "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}")
    set_target_properties(${TARGET_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE
                                                    "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}")
  endforeach()
endfunction(generate_package_targets)

# Start of main projection configuration

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
include(GoogleTest)
enable_testing()


# Function to download and move files
function(download_and_move_file url destination)
    get_filename_component(filename "${destination}" NAME)
    set(temp_file "${CMAKE_BINARY_DIR}/data/${filename}")

    if(NOT EXISTS "${destination}")
        message(STATUS "File ${filename} not found, downloading from ${url}...")

        # Download the file and show a progress bar
        file(DOWNLOAD
            "${url}"
            "${temp_file}"
            SHOW_PROGRESS
            STATUS DOWNLOAD_RESULT_CODE
        )

        # Check for download errors
        if(NOT DOWNLOAD_RESULT_CODE EQUAL 0)
            message(FATAL_ERROR "Failed downloading ${filename}! Error code: ${DOWNLOAD_RESULT_CODE}.")
        endif()

        # Move the file into the appropriate directory
        file(RENAME "${temp_file}" "${destination}")
        message(STATUS "Downloaded and moved ${filename} to ${destination}.")
    else()
        message(STATUS "File ${filename} already exists.")
    endif()
endfunction()

# Define download URLs and target paths
set(SPICE_FILES
    "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/spk/planets/de430.bsp::${CMAKE_SOURCE_DIR}/../supportData/EphemerisData/de430.bsp"
    "https://naif.jpl.nasa.gov/pub/naif/HST/kernels/spk/hst_edited.bsp::${CMAKE_SOURCE_DIR}/../supportData/EphemerisData/hst_edited.bsp"
    "https://naif.jpl.nasa.gov/pub/naif/pds/data/nh-j_p_ss-spice-6-v1.0/nhsp_1000/data/spk/nh_pred_od077.bsp::${CMAKE_SOURCE_DIR}/../supportData/EphemerisData/nh_pred_od077.bsp"
)

# Process each large data file
message(STATUS "Checking large Basilisk data files")
foreach(spice_file IN LISTS SPICE_FILES)
    string(REPLACE "::" ";" spice_parts "${spice_file}")
    list(GET spice_parts 0 url)
    list(GET spice_parts 1 destination)
    download_and_move_file("${url}" "${destination}")
endforeach()



# TODO: Remove the global link-libraries call
find_package(Eigen3 CONFIG REQUIRED)
# The following lines are necessary to prevent Xcode from issuing a warning about Eigen not being within the search
# path. Related to Eigen being a header only library.
if(DEFINED Eigen3_INCLUDE_DIRS_DEBUG)
  set(DIRECTORIES
      "$<$<CONFIG:Debug>:${Eigen3_INCLUDE_DIRS_DEBUG}>" "$<$<CONFIG:Release>:${Eigen3_INCLUDE_DIRS_DEBUG}>"
      "$<$<CONFIG:RelWithDebInfo>:${Eigen3_INCLUDE_DIRS_DEBUG}>" "$<$<CONFIG:MinSizeRel>:${Eigen3_INCLUDE_DIRS_DEBUG}>")
else()
  set(DIRECTORIES
      "$<$<CONFIG:Debug>:${Eigen3_INCLUDE_DIRS_RELEASE}>" "$<$<CONFIG:Release>:${Eigen3_INCLUDE_DIRS_RELEASE}>"
      "$<$<CONFIG:RelWithDebInfo>:${Eigen3_INCLUDE_DIRS_RELEASE}>"
      "$<$<CONFIG:MinSizeRel>:${Eigen3_INCLUDE_DIRS_RELEASE}>")
endif()
set_property(TARGET Eigen3::Eigen3 PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${DIRECTORIES})
link_libraries(Eigen3::Eigen3)

# Add general compiler flags
set(CMAKE_MACOSX_RPATH 1)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")

# Test Coverage
option(USE_COVERAGE "GCOV code coverage analysis" OFF)

if(USE_COVERAGE AND (CMAKE_COMPILER_IS_GNUC OR CMAKE_COMPILER_IS_GNUCXX))
  message(STATUS "USE_COVERAGE : ${USE_COVERAGE}")
  set(GCOV_CFLAGS "-fprofile-arcs -ftest-coverage -O0 -fno-default-inline -fno-inline")
  set(GCOV_LDFLAGS "-fprofile-arcs -ftest-coverage -O0 -fno-default-inline -fno-inline")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${GCOV_CFLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCOV_CFLAGS}")
  set(CMAKE_LD_FLAGS "${CMAKE_LD_FLAGS} ${GCOV_LDFLAGS}")

  message(STATUS "CMAKE_C_FLAGS : ${CMAKE_C_FLAGS}")
  message(STATUS "CMAKE_CXX_FLAGS : ${CMAKE_CXX_FLAGS}")
  message(STATUS "CMAKE_LD_FLAGS : ${CMAKE_LD_FLAGS}")
endif()

# Add platform specific compiler flags
if(MSVC)
  set(CMAKE_CXX_STANDARD 17)
  add_definitions(/MP)
  add_definitions(/D _USE_MATH_DEFINES)
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
  foreach(flag CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG
               CMAKE_CXX_FLAGS_RELEASE)
    if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
      string(REPLACE "/D_DEBUG" "" "${flag}" "${${flag}}")
      string(REPLACE "/MDd" "${CONAN_LINK_RUNTIME_MULTI}" "${flag}" "${${flag}}")
      string(REPLACE "/MTd" "${CONAN_LINK_RUNTIME_MULTI}" "${flag}" "${${flag}}")
    else()
      string(REPLACE "/MD" "${CONAN_LINK_RUNTIME_MULTI}" "${flag}" "${${flag}}")
      string(REPLACE "/MT" "${CONAN_LINK_RUNTIME_MULTI}" "${flag}" "${${flag}}")
    endif()
    set("${flag}" "${${flag}} /EHsc")
  endforeach()
elseif(APPLE)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -gdwarf-3")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -gdwarf-3 -stdlib=libc++")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -gdwarf-3")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}  -Wall")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}  -Wall")
  if(${CMAKE_GENERATOR} MATCHES "Xcode")
    set(CMAKE_XCODE_GENERATE_SCHEME YES)
  endif()
else()
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -gdwarf-3")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -gdwarf-3 -fPIC")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -gdwarf-3")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}  -Wall")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}  -Wall")
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-register")
endif()

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(_arch_suffix 64)
else()
  set(_arch_suffix 32)
endif()

# TODO: Move the cspice library extension into a custom pyswice CMakeList.txt Manually create list of libraries
# depending on system
find_package(cspice REQUIRED CONFIG)
set(library_dependencies cspice::cspice)
if(WIN32)
  set(CMAKE_MSVCIDE_RUN_PATH "${CMAKE_BINARY_DIR}/Basilisk")
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS YES)
  # fix for opencv not finding conan gflags for opencv sfm lib on windows when the finding is fixed, the following line
  # should be removed https://github.com/conan-community/community/issues/210
  list(REMOVE_ITEM CONAN_LIBS opencv_sfm411d multiview)

elseif(APPLE)
  set(CMAKE_SKIP_BUILD_RPATH FALSE) # use, i.e. don't skip the full RPATH for the build tree
  set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE) # when building, don't use the install RPATH already (but later on when
                                           # installing)
  set(CMAKE_INSTALL_RPATH "${CMAKE_BINARY_DIR}/Basilisk"  # the RPATH to be used when installing
      # Also add relative paths so we can install from wheel/sdist.
      # TODO: Is there a proper way to set relative library paths without just
      # guessing how many parent ".." directories to use?
      "@loader_path"
      "@loader_path/.."
      "@loader_path/../.."
  )

  # don't add the automatically determined parts of the RPATH which point to directories outside the build tree to the
  # install RPATH
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
else()
  set(CMAKE_SKIP_BUILD_RPATH FALSE)
  set(CMAKE_INSTALL_RPATH "\$ORIGIN/../../")
endif()

set_property(GLOBAL PROPERTY BUILT_LIB_LIST "ArchitectureUtilities")
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Targets that must be built first
add_subdirectory("architecture/utilities")
add_subdirectory("utilities") # has protobuffers included
add_subdirectory("architecture/messaging/cMsgCInterface")
add_subdirectory("architecture/messaging/")

# TODO: I'd call this generate_libraries(), because it really does find all of them. TODO: I'd like for all generated
# libraries to end up in a dist/Basilisk/lib folder rather than the /dist/Basilisk folder; however haven't found a way
# to use target_link_libraries() in a cross platform way to support this (needs full path and custom extensions
# --.dylib, .lib, .a, etc -- which gets messy in CMake).
set(AllLibs "")
generate_package_libraries("${CMAKE_SOURCE_DIR}" "${AllLibs}") # This finds GeneralModuleFiles and generates a library
                                                               # of the parentDirectory name

set(ARCHITECTURE_LIBS architectureLib ArchitectureUtilities ModuleIdGenerator cMsgCInterface)

# SIMULATION
# TODO: Move the following commands into a seperate CMakeList.txt s.t. this file is just configuration (problem:
# currently when I do this the targets argument in generate_package_targets gets interpreted differently) TODO: Automate
# this: 1) Look for package directories (dynamics, environment), 2) Find/Link any libraries associated with that package
# (dynamicsLib), 3) Look for custom CMakeLists.txt to link additional libraries.

# Finds relative path of all .i files within the directory
find_package_targets("${CMAKE_SOURCE_DIR}/simulation/dynamics" DYN_TARGETS)
find_package_targets("${CMAKE_SOURCE_DIR}/simulation/mujocoDynamics" MULTI_BODY_DYN_TARGETS)
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
endif(NOT "${EXTERNAL_MODULES_PATH}" STREQUAL "")

# The quotations around DYN_TARGETS informs function that it is a list
generate_package_targets("${DYN_TARGETS}" "dynamicsLib;${ARCHITECTURE_LIBS}" "simulation")
# Only build mujocoDynamics targets if the mujocoDynamics library is available
if(NOT ("mujocoDynamics" IN_LIST EXCLUDED_BSK_LIBRARIES))
  generate_package_targets("${MULTI_BODY_DYN_TARGETS}" "mujocoDynamicsLib;dynamicsLib;${ARCHITECTURE_LIBS}" "simulation")
endif()
generate_package_targets("${ENV_TARGETS}" "environmentLib;${ARCHITECTURE_LIBS};${library_dependencies}" "simulation")
generate_package_targets("${DATA_TARGETS}" "onboardDataHandlingLib;${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${DEVICE_INTERFACE_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${POWER_TARGETS}" "powerLib;${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${NAV_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${SENSORS_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${SIM_SYNCH_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${VIZ_INTERFACE_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")
generate_package_targets("${THERMAL_TARGETS}" "${ARCHITECTURE_LIBS};" "simulation")

# FSW ALGORITHMS
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
find_package_targets("${CMAKE_SOURCE_DIR}/fswAlgorithms/stateEstimation" STATE_EST_TARGETS)
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
generate_package_targets("${STATE_EST_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${ORBIT_CONTROL_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${TRANS_DET_TARGETS}" "transDeterminationLib;${ARCHITECTURE_LIBS};" "fswAlgorithms")
generate_package_targets("${VEH_CONFIG_TARGETS}" "${ARCHITECTURE_LIBS};" "fswAlgorithms")

# TEMPLATE MODULES
find_package_targets("${CMAKE_SOURCE_DIR}/moduleTemplates" TEMPLATE_MODULES_TARGETS)

generate_package_targets("${TEMPLATE_MODULES_TARGETS}" "${ARCHITECTURE_LIBS};" "moduleTemplates")

# MESSAGES
add_message_headers()

# ARCHITECTURE
find_package_targets("${CMAKE_SOURCE_DIR}/architecture" ARCHITECTURE_TARGETS)
generate_package_targets("${ARCHITECTURE_TARGETS}" "${ARCHITECTURE_LIBS};" "architecture")

# Pyswice
find_package_targets("${CMAKE_SOURCE_DIR}/topLevelModules/pyswice" PYSWICE_TARGETS) # Finds relative path of all .i
                                                                                    # files within the directory
generate_package_targets("${PYSWICE_TARGETS}" "${ARCHITECTURE_LIBS};${library_dependencies}" "topLevelModules")

# PYTHON PACKAGE CONFIGURATION
# Must make the build directories first, so that cmake can insert empty init files before build (needed for packaging correctly)
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk")
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/architecture")
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/simulation")
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/fswAlgorithms")
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/moduleTemplates")
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/topLevelModules")
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/ExternalModules")

if(WIN32 AND (Python3_VERSION VERSION_GREATER 3.8))
  file(WRITE "${CMAKE_BINARY_DIR}/Basilisk/__init__.py"
       "#init file written by the build\n" "import sys, os\n" "from Basilisk import __path__\n"
       "bskPath = __path__[0]\n" "os.add_dll_directory(bskPath)\n"
       "from Basilisk.architecture import messaging\n")
else()
  file(WRITE "${CMAKE_BINARY_DIR}/Basilisk/__init__.py"
      "from Basilisk.architecture import messaging # ensure recorders() work without first importing messaging\n")
endif()

# TODO: Iterate through all dist directories and add __init__.py's where they don't exist
file(
  GLOB_RECURSE DIST_DIRECTORIES
  LIST_DIRECTORIES true
  "${CMAKE_BINARY_DIR}/Basilisk/*")

foreach(DIR ${DIST_DIRECTORIES})
  if(IS_DIRECTORY ${DIR})
    file(
      GLOB DIST_DIR_FILES
      RELATIVE ${DIR}
      "${DIR}/*.py")
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

# symlink utilities into the dist directory TODO: consider making the utilities directory it's own module with
# associated .i file and many `%pythoncode` directives.
file(GLOB pythonModules "${CMAKE_SOURCE_DIR}/utilities/*.py" "${CMAKE_SOURCE_DIR}/utilities/**")
create_symlinks("${CMAKE_BINARY_DIR}/Basilisk/utilities" ${pythonModules})

# symlink into package the supportData files to keep
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/supportData")
file(GLOB dataFiles "${CMAKE_SOURCE_DIR}/../supportData/*")
create_symlinks("${CMAKE_BINARY_DIR}/Basilisk/supportData" ${dataFiles})

# Tests targets
add_subdirectory("architecture/utilities/tests")
