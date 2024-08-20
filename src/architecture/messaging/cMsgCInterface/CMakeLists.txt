# Read the LICENSE file into a template variable
file(READ "${CMAKE_SOURCE_DIR}/../LICENSE" MSG_AUTOSOURCE_LICENSE)

# Set the output directory for all generated messages
set(CMSGCINTERFACE_DIR ${CMAKE_BINARY_DIR}/autoSource/cMsgCInterface)

function(GenCMessages TARGET searchDir)
    #--------------------------------------------------------------------------
    # Generates C++ definitions for each C message file named
    # "{searchDir}/msgPayloadDefC/*Payload.h". Generated source files are
    # appended to the TARGET.
    #
    # Implementation notes:
    # This function was ported from `messaging/architecture/GenCMessages.py`.
    # Originally, `GenCMessages.py` was called from the Conanfile prior to
    # configuring CMake, but this meant that CMake could not track which
    # C-messages changed, making incremental builds impossible.
    # Instead, this function tracks dependencies directly in CMake.
    #--------------------------------------------------------------------------

    # Find all C message payloads
    file(GLOB_RECURSE payloadFiles CONFIGURE_DEPENDS "${searchDir}/msgPayloadDefC/*Payload.h")
    foreach(payloadFile ${payloadFiles})

        # Compute the name of the payload structure and generated files
        get_filename_component(msgFileName "${payloadFile}" NAME_WLE)
        string(REGEX REPLACE "^(.+)Payload$" "\\1" structName "${msgFileName}")
        set(headerFile ${CMSGCINTERFACE_DIR}/${structName}_C.h)
        set(objectFile ${CMSGCINTERFACE_DIR}/${structName}_C.cpp)
        file(RELATIVE_PATH relativePayloadFile ${CMSGCINTERFACE_DIR} ${payloadFile})

        # Set the template variables
        set(MSG_AUTOSOURCE_TYPE   ${structName})
        set(MSG_AUTOSOURCE_HEADER ${relativePayloadFile})

        # Generate the corresponding .h header and .cpp object file
        configure_file(msg_C.h.in   ${headerFile} @ONLY)
        configure_file(msg_C.cpp.in ${objectFile} @ONLY)

        # Mark these files as GENERATED
        set(generatedFiles ${objectFile} ${headerFile})
        set_source_files_properties(${generatedFiles} PROPERTIES GENERATED ON)

        # Add these files as sources for the TARGET
        target_sources(${TARGET} PRIVATE ${generatedFiles})
    endforeach()
endfunction(GenCMessages)

SET(LIB_DIR "${CMAKE_SOURCE_DIR}/simulation/architecture/messaging/cMsgCInterface/")

# Determine the name of the parent directory of _GeneralModuleFiles (i.e. dynamics, environment, etc)
get_filename_component(PARENT_DIR ${LIB_DIR} DIRECTORY) # Relative to the source directory (e.g. simulation/power)
string(FIND ${PARENT_DIR} "/" DIR_NAME_IDX REVERSE)
math(EXPR DIR_NAME_IDX "${DIR_NAME_IDX} + 1")# "simulation/"
string(SUBSTRING ${PARENT_DIR} ${DIR_NAME_IDX} -1 PARENT_DIR_NAME) # Should be "power"

# Set the library name (ex. dynamicsLib, environmentLib, etc)
set(LIB_NAME "cMsgCInterface")

# Add Target
add_library(${LIB_NAME} STATIC ${BSK_FWK_FILES})
if(NOT WIN32)
	target_compile_options(${LIB_NAME} PUBLIC "-fPIC")
endif()

# Generate and add C message source files to the library
GenCMessages(${LIB_NAME} "../..")
GenCMessages(${LIB_NAME} "${EXTERNAL_MODULES_PATH}")

# Link all necessary libraries
#target_link_libraries(${LIB_NAME} ArchitectureUtilities)
target_link_libraries(${LIB_NAME} ${PYTHON3_MODULE})
#target_link_libraries(${LIB_NAME} Eigen3::Eigen3)

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
