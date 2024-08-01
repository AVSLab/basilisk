cmake_policy(SET CMP0078 NEW)

# Set the output directories for all generated message sources and library files
set(AUTOSOURCE_DIR  "${CMAKE_BINARY_DIR}/autoSource")
set(SWIG_SOURCE_DIR "${CMAKE_BINARY_DIR}/autoSource/SWIG") # SWIG-generated _wrap.c/.cxx source files
set(SWIG_OUTPUT_DIR "${CMAKE_BINARY_DIR}/messaging")  # SWIG-generated .py/.so library files

# Clean up the Basilisk output directory to delete any stale (previously built)
# messages. Note that stale messages are still available in the SWIG_OUTPUT_DIR.
set(BASILISK_MESSAGING_DIR "${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging")
file(REMOVE_RECURSE ${BASILISK_MESSAGING_DIR})

function(generate_messages searchDir generateCCode)
  file(
    GLOB_RECURSE message_files
    RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/"
    "../${searchDir}/*Payload.h" "${EXTERNAL_MODULES_PATH}/${searchDir}/*Payload.h")

  foreach(msgFile ${message_files})
    get_filename_component(TARGET_NAME ${msgFile} NAME_WE)
    get_filename_component(TARGET_DIR ${msgFile} DIRECTORY)
    set(COMP_OUT_NAME "${AUTOSOURCE_DIR}/${TARGET_NAME}.i")
    add_custom_command(
      OUTPUT ${COMP_OUT_NAME}
      COMMAND ${Python3_EXECUTABLE} generateSWIGModules.py ${COMP_OUT_NAME} ${msgFile} ${TARGET_NAME} ${searchDir}
              ${generateCCode}
      DEPENDS ${msgFile}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/msgAutoSource/)
    set_property(SOURCE ${COMP_OUT_NAME} PROPERTY CPLUSPLUS ON)

    if(NOT "${EXTERNAL_MODULES_PATH}" STREQUAL "")
      set_property(
        SOURCE ${COMP_OUT_NAME}
        PROPERTY SWIG_FLAGS "-I${CMAKE_CURRENT_SOURCE_DIR}/../" "-I${CMAKE_CURRENT_SOURCE_DIR}/../../"
                 "-I${EXTERNAL_MODULES_PATH}/" "-I${AUTOSOURCE_DIR}/" "-I${Python3_INCLUDE_DIRS}")
      include_directories("${EXTERNAL_MODULES_PATH}/")
    else()
      set_property(
        SOURCE ${COMP_OUT_NAME}
        PROPERTY SWIG_FLAGS "-I${CMAKE_CURRENT_SOURCE_DIR}/../" "-I${CMAKE_CURRENT_SOURCE_DIR}/../../"
                 "-I${AUTOSOURCE_DIR}/" "-I${Python3_INCLUDE_DIRS}")
    endif(NOT "${EXTERNAL_MODULES_PATH}" STREQUAL "")

    include_directories(${Python3_INCLUDE_DIRS})
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../")
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/")
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../../")

    # Build this message module using SWIG. Generates a <TARGET>.py and _<TARGET>.so file.
    swig_add_library(
      ${TARGET_NAME}
      LANGUAGE "python"
      TYPE MODULE
      SOURCES ${COMP_OUT_NAME} OUTFILE_DIR ${SWIG_SOURCE_DIR} OUTPUT_DIR ${SWIG_OUTPUT_DIR})

    # Ensure the Basilisk.architecture.message is built AFTER all the modules
    add_dependencies(messageModule ${TARGET_NAME})

    # Copy the generated module file into the Basilisk.architecture.message
    # module. This is done so that we can always clean any stale (previously
    # built) messages, and only copy messages we actually want now.
    # NOTE: messageModule is *always* built, so this copy *always* happens even
    # if the underlying module didn't need to get built again.
    add_custom_command(
      TARGET messageModule
      POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:${TARGET_NAME}> $<TARGET_PROPERTY:${TARGET_NAME},SWIG_SUPPORT_FILES>
        "${BASILISK_MESSAGING_DIR}"
    )

    set_target_properties(${TARGET_NAME} PROPERTIES FOLDER "architecture/messaging/derivedCode")  # TODO: Is this still used?
    set_target_properties(${SWIG_MODULE_${TARGET_NAME}_REAL_NAME}
                          PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${SWIG_OUTPUT_DIR}")
    set_target_properties(
      ${SWIG_MODULE_${TARGET_NAME}_REAL_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG
                                                         "${SWIG_OUTPUT_DIR}")
    set_target_properties(
      ${SWIG_MODULE_${TARGET_NAME}_REAL_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE
                                                         "${SWIG_OUTPUT_DIR}")
    target_link_libraries(${SWIG_MODULE_${TARGET_NAME}_REAL_NAME} PUBLIC architectureLib)
  endforeach()
endfunction(generate_messages)

# Copy the cMsgCInterfacePy wrapper module to support backwards-compatibility.
# TODO: Deprecate this!
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cMsgCInterfacePy/__init__.py ${CMAKE_BINARY_DIR}/Basilisk/architecture/cMsgCInterfacePy/__init__.py COPYONLY)

if(NOT "${EXTERNAL_MODULES_PATH}" STREQUAL "")
  add_custom_command(
    OUTPUT ${BASILISK_MESSAGING_DIR}/__init__.py
    COMMAND
      ${Python3_EXECUTABLE} generatePackageInit.py "${BASILISK_MESSAGING_DIR}"
      "../../msgPayloadDefC/" "../../msgPayloadDefCpp/" "${EXTERNAL_MODULES_PATH}/msgPayloadDefC/"
      "${EXTERNAL_MODULES_PATH}/msgPayloadDefCpp/"
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/msgAutoSource
    VERBATIM)
else()
  add_custom_command(
    OUTPUT ${BASILISK_MESSAGING_DIR}/__init__.py
    COMMAND ${Python3_EXECUTABLE} generatePackageInit.py "${BASILISK_MESSAGING_DIR}"
            "../../msgPayloadDefC/" "../../msgPayloadDefCpp/"
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/msgAutoSource
    VERBATIM)
endif(NOT "${EXTERNAL_MODULES_PATH}" STREQUAL "")

# Custom target representing the Basilisk.architecture.messaging module -- ALWAYS gets built!
add_custom_target(messageModule ALL DEPENDS ${BASILISK_MESSAGING_DIR}/__init__.py)

# Generate Python message modules
generate_messages(msgPayloadDefCpp "False")
generate_messages(msgPayloadDefC "True")
