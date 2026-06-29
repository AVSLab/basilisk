cmake_policy(SET CMP0078 NEW)

function(generate_messages searchDir generateCCode outEqualityHeaders)
  file(
    GLOB_RECURSE message_files
    RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/"
    "../${searchDir}/*Payload.h" "${EXTERNAL_MODULES_PATH}/${searchDir}/*Payload.h")

  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/autoSource/cMsgMeta)

  # Determine C vs C++ language flag for libclang
  if("${searchDir}" STREQUAL "msgPayloadDefC")
    set(CLANG_LANG_FLAG "c")
  else()
    set(CLANG_LANG_FLAG "c++")
  endif()

  set(local_equality_headers "")

  foreach(msgFile ${message_files})
    get_filename_component(TARGET_NAME ${msgFile} NAME_WE)

    set(MSG_META_OUTPUT "${CMAKE_BINARY_DIR}/autoSource/cMsgMeta/${TARGET_NAME}.json")
    set(MSG_META_DEPFILE "${CMAKE_BINARY_DIR}/autoSource/cMsgMeta/${TARGET_NAME}.d")
    add_custom_command(
      OUTPUT ${MSG_META_OUTPUT}
      COMMAND ${Python3_EXECUTABLE}
              msgAutoSource/generatePayloadMetaJson.py
              ${msgFile}
              ${TARGET_NAME}
              ${MSG_META_OUTPUT}
              --depfile ${MSG_META_DEPFILE}
              --
              -x ${CLANG_LANG_FLAG}
              -I${CMAKE_CURRENT_SOURCE_DIR}/../
              -I${CMAKE_CURRENT_SOURCE_DIR}/../../
      DEPENDS ${msgFile} msgAutoSource/generatePayloadMetaJson.py
      DEPFILE ${MSG_META_DEPFILE}
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/"
      COMMENT "Generating metadata JSON from ${msgFile}"
      VERBATIM
    )

    set(EQUALITY_OUT_NAME "${CMAKE_BINARY_DIR}/autoSource/${TARGET_NAME}_equality.h")
    add_custom_command(
      OUTPUT ${EQUALITY_OUT_NAME}
      COMMAND ${Python3_EXECUTABLE}
              msgAutoSource/generatePayloadEqualityHeader.py
              ${EQUALITY_OUT_NAME} ${MSG_META_OUTPUT} ${TARGET_NAME} ${searchDir}
      DEPENDS ${MSG_META_OUTPUT} msgAutoSource/generatePayloadEqualityHeader.py
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/"
      COMMENT "Generating payload equality header for ${TARGET_NAME}"
      VERBATIM
    )
    list(APPEND local_equality_headers "${TARGET_NAME}_equality.h")

    get_filename_component(TARGET_DIR ${msgFile} DIRECTORY)
    set(COMP_OUT_NAME "${CMAKE_BINARY_DIR}/autoSource/${TARGET_NAME}.i")
    add_custom_command(
      OUTPUT ${COMP_OUT_NAME}
      COMMAND ${Python3_EXECUTABLE} generateSWIGModules.py ${COMP_OUT_NAME} ${msgFile} ${TARGET_NAME} ${searchDir}
              ${generateCCode} ${MSG_META_OUTPUT} ${RECORDER_PROPERTY_ROLLBACK}
      DEPENDS ${msgFile} ${MSG_META_OUTPUT} ${EQUALITY_OUT_NAME}
              msgAutoSource/generateSWIGModules.py msgAutoSource/msgInterfacePy.i.in
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
      SOURCES ${COMP_OUT_NAME} OUTFILE_DIR "${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging/"
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

  set(${outEqualityHeaders} ${local_equality_headers} PARENT_SCOPE)
endfunction(generate_messages)

# Copy the cMsgCInterfacePy wrapper module to support backwards-compatibility.
# TODO: Deprecate this!
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cMsgCInterfacePy/__init__.py ${CMAKE_BINARY_DIR}/Basilisk/architecture/cMsgCInterfacePy/__init__.py COPYONLY)

# Track payload headers so the generated Python message package is rebuilt when message definitions change.
file(GLOB_RECURSE package_init_payload_files CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/../msgPayloadDefC/*Payload.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/../msgPayloadDefCpp/*Payload.h")

if(NOT "${EXTERNAL_MODULES_PATH}" STREQUAL "")
  file(GLOB_RECURSE package_init_external_payload_files CONFIGURE_DEPENDS
    "${EXTERNAL_MODULES_PATH}/msgPayloadDefC/*Payload.h"
    "${EXTERNAL_MODULES_PATH}/msgPayloadDefCpp/*Payload.h")
  list(APPEND package_init_payload_files ${package_init_external_payload_files})

  add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging/__init__.py
    COMMAND
      ${Python3_EXECUTABLE} generatePackageInit.py "${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging/"
      "../../msgPayloadDefC/" "../../msgPayloadDefCpp/" "${EXTERNAL_MODULES_PATH}/msgPayloadDefC/"
      "${EXTERNAL_MODULES_PATH}/msgPayloadDefCpp/"
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/msgAutoSource/generatePackageInit.py ${package_init_payload_files}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/msgAutoSource
    VERBATIM)
else()
  add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging/__init__.py
    COMMAND ${Python3_EXECUTABLE} generatePackageInit.py "${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging/"
            "../../msgPayloadDefC/" "../../msgPayloadDefCpp/"
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/msgAutoSource/generatePackageInit.py ${package_init_payload_files}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/msgAutoSource
    VERBATIM)
endif(NOT "${EXTERNAL_MODULES_PATH}" STREQUAL "")

# Custom target for establishing dependency
add_custom_target(swigtrick DEPENDS ${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging/__init__.py)

# Generate per-payload equality headers and aggregate into a single umbrella.
generate_messages(msgPayloadDefCpp "False" cpp_equality_headers)
generate_messages(msgPayloadDefC "True" c_equality_headers)

set(UMBRELLA "${CMAKE_BINARY_DIR}/autoSource/payloadEquality_generated.h")
file(WRITE  ${UMBRELLA} "// AUTO-GENERATED — DO NOT EDIT\n#pragma once\n")
foreach(eqHeader ${cpp_equality_headers} ${c_equality_headers})
  file(APPEND ${UMBRELLA} "#if __has_include(\"${eqHeader}\")\n#  include \"${eqHeader}\"\n#endif\n")
endforeach()

# Verify the central SysModel copy restriction while keeping Recorder explicitly copyable.
add_executable(test_recorder_copy_contract _UnitTest/test_recorder_copy_contract.cpp)
target_sources(test_recorder_copy_contract PRIVATE ${CMAKE_SOURCE_DIR}/architecture/_GeneralModuleFiles/sys_model.cpp)
target_link_libraries(test_recorder_copy_contract GTest::gtest_main)
target_link_libraries(test_recorder_copy_contract ArchitectureUtilities)
target_link_libraries(test_recorder_copy_contract ModuleIdGenerator)

gtest_discover_tests(test_recorder_copy_contract)
