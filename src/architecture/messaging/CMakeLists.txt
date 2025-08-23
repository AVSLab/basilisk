cmake_policy(SET CMP0078 NEW)

function(generate_messages searchDir generateCCode)
  file(
    GLOB_RECURSE message_files
    RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/"
    "../${searchDir}/*Payload.h" "${EXTERNAL_MODULES_PATH}/${searchDir}/*Payload.h")

  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/autoSource/xmlWrap)

  foreach(msgFile ${message_files})
    get_filename_component(TARGET_NAME ${msgFile} NAME_WE)

    set(SWIG_XML_OUTPUT "${CMAKE_BINARY_DIR}/autoSource/xmlWrap/${TARGET_NAME}.xml")
    add_custom_command(
      OUTPUT ${SWIG_XML_OUTPUT}
      COMMAND ${SWIG_EXECUTABLE} -c++ -xml -module dummy -o ${SWIG_XML_OUTPUT} ${msgFile}
      DEPENDS ${msgFile}
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/"
      COMMENT "Generating SWIG XML from ${msgFile}"
      VERBATIM
    )

    get_filename_component(TARGET_DIR ${msgFile} DIRECTORY)
    set(COMP_OUT_NAME "${CMAKE_BINARY_DIR}/autoSource/${TARGET_NAME}.i")
    add_custom_command(
      OUTPUT ${COMP_OUT_NAME}
      COMMAND ${Python3_EXECUTABLE} generateSWIGModules.py ${COMP_OUT_NAME} ${msgFile} ${TARGET_NAME} ${searchDir}
              ${generateCCode} ${SWIG_XML_OUTPUT} ${RECORDER_PROPERTY_ROLLBACK}
      DEPENDS ${msgFile} ${SWIG_XML_OUTPUT}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/msgAutoSource/)
    set_property(SOURCE ${COMP_OUT_NAME} PROPERTY CPLUSPLUS ON)

    if(NOT "${EXTERNAL_MODULES_PATH}" STREQUAL "")
      set_property(
        SOURCE ${COMP_OUT_NAME}
        PROPERTY SWIG_FLAGS "-I${CMAKE_CURRENT_SOURCE_DIR}/../" "-I${CMAKE_CURRENT_SOURCE_DIR}/../../"
                 "-I${EXTERNAL_MODULES_PATH}/" "-I${CMAKE_BINARY_DIR}/autoSource/" "-I${Python3_INCLUDE_DIRS}" "-I${NUMPY_INCLUDE_DIR}")
      include_directories("${EXTERNAL_MODULES_PATH}/")
    else()
      set_property(
        SOURCE ${COMP_OUT_NAME}
        PROPERTY SWIG_FLAGS "-I${CMAKE_CURRENT_SOURCE_DIR}/../" "-I${CMAKE_CURRENT_SOURCE_DIR}/../../"
                 "-I${CMAKE_BINARY_DIR}/autoSource/" "-I${Python3_INCLUDE_DIRS}" "-I${NUMPY_INCLUDE_DIR}")
    endif(NOT "${EXTERNAL_MODULES_PATH}" STREQUAL "")

    include_directories(${Python3_INCLUDE_DIRS})
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../")
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/")
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../../")
    include_directories(${NUMPY_INCLUDE_DIR})

    swig_add_library(
      ${TARGET_NAME}
      LANGUAGE "python"
      TYPE MODULE
      SOURCES ${COMP_OUT_NAME} OUTFILE_DIR "${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging/" # _wrap.c/.cxx file
              OUTPUT_DIR "${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging/")

    add_dependencies(${TARGET_NAME} swigtrick)
    set_target_properties(${TARGET_NAME} PROPERTIES FOLDER "architecture/messaging/derivedCode")
    set_target_properties(${SWIG_MODULE_${TARGET_NAME}_REAL_NAME}
                          PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging")
    set_target_properties(
      ${SWIG_MODULE_${TARGET_NAME}_REAL_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG
                                                         "${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging")
    set_target_properties(
      ${SWIG_MODULE_${TARGET_NAME}_REAL_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE
                                                         "${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging")
    target_link_libraries(${SWIG_MODULE_${TARGET_NAME}_REAL_NAME} PUBLIC architectureLib)
  endforeach()
endfunction(generate_messages)

# Copy the cMsgCInterfacePy wrapper module to support backwards-compatibility.
# TODO: Deprecate this!
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cMsgCInterfacePy/__init__.py ${CMAKE_BINARY_DIR}/Basilisk/architecture/cMsgCInterfacePy/__init__.py COPYONLY)

if(NOT "${EXTERNAL_MODULES_PATH}" STREQUAL "")
  add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging/__init__.py
    COMMAND
      ${Python3_EXECUTABLE} generatePackageInit.py "${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging/"
      "../../msgPayloadDefC/" "../../msgPayloadDefCpp/" "${EXTERNAL_MODULES_PATH}/msgPayloadDefC/"
      "${EXTERNAL_MODULES_PATH}/msgPayloadDefCpp/"
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/msgAutoSource
    VERBATIM)
else()
  add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging/__init__.py
    COMMAND ${Python3_EXECUTABLE} generatePackageInit.py "${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging/"
            "../../msgPayloadDefC/" "../../msgPayloadDefCpp/"
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/msgAutoSource
    VERBATIM)
endif(NOT "${EXTERNAL_MODULES_PATH}" STREQUAL "")

# Custom target for establishing dependency
add_custom_target(swigtrick DEPENDS ${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging/__init__.py)

# Dependency
generate_messages(msgPayloadDefCpp "False")
generate_messages(msgPayloadDefC "True")
