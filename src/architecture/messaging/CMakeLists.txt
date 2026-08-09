cmake_policy(SET CMP0078 NEW)

function(collect_message_payloads searchDir outputVariable)
  set(message_search_roots
      "${CMAKE_CURRENT_SOURCE_DIR}/../${searchDir}")
  if(NOT "${EXTERNAL_MODULES_PATH}" STREQUAL "")
    list(APPEND message_search_roots
         "${EXTERNAL_MODULES_PATH}/${searchDir}")
  endif()

  set(message_files)
  foreach(message_search_root IN LISTS message_search_roots)
    bsk_filter_source_inventory(
      root_message_files
      "${message_search_root}"
      BSK_HEADER_FILES)
    list(FILTER root_message_files INCLUDE REGEX "Payload\\.h$")
    list(APPEND message_files ${root_message_files})
  endforeach()
  list(REMOVE_DUPLICATES message_files)
  list(SORT message_files)
  set(${outputVariable} ${message_files} PARENT_SCOPE)
endfunction()

function(generate_payload_manifest outputFile)
  set(payload_files ${ARGN})
  list(REMOVE_DUPLICATES payload_files)
  list(SORT payload_files)

  set(payload_names)
  set(payload_sources)
  foreach(payload_file IN LISTS payload_files)
    get_filename_component(payload_name "${payload_file}" NAME_WE)
    list(FIND payload_names "${payload_name}" duplicate_index)
    if(NOT duplicate_index EQUAL -1)
      list(GET payload_sources ${duplicate_index} original_payload_file)
      message(
        FATAL_ERROR
          "Duplicate message payload name '${payload_name}' was discovered.\n"
          "  First header:  ${original_payload_file}\n"
          "  Second header: ${payload_file}\n"
          "Payload header names must be unique across built-in and external message trees.")
    endif()
    list(APPEND payload_names "${payload_name}")
    list(APPEND payload_sources "${payload_file}")
  endforeach()

  list(SORT payload_names)
  list(JOIN payload_names "\n" payload_manifest_content)
  if(payload_manifest_content)
    string(APPEND payload_manifest_content "\n")
  endif()
  file(GENERATE OUTPUT "${outputFile}" CONTENT "${payload_manifest_content}")
endfunction()

collect_message_payloads(msgPayloadDefC C_MESSAGE_PAYLOAD_FILES)
collect_message_payloads(msgPayloadDefCpp CPP_MESSAGE_PAYLOAD_FILES)
set(MESSAGE_PAYLOAD_FILES
    ${C_MESSAGE_PAYLOAD_FILES}
    ${CPP_MESSAGE_PAYLOAD_FILES})
list(REMOVE_DUPLICATES MESSAGE_PAYLOAD_FILES)
list(SORT MESSAGE_PAYLOAD_FILES)

file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/autoSource")
set(MESSAGE_PAYLOAD_MANIFEST
    "${CMAKE_BINARY_DIR}/autoSource/messagePayloads.txt")
# Keep this state file outside the custom command's declared outputs so a
# CMake clean cannot erase the prior inventory before regeneration compares it.
set(PREVIOUS_MESSAGE_PAYLOAD_MANIFEST
    "${CMAKE_BINARY_DIR}/autoSource/messagePayloads.previous.txt")
generate_payload_manifest(
  "${MESSAGE_PAYLOAD_MANIFEST}"
  ${MESSAGE_PAYLOAD_FILES})

function(generate_messages searchDir generateCCode messageFilesVariable)

  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/autoSource/cMsgMeta)

  # Determine C vs C++ language flag for libclang
  if("${searchDir}" STREQUAL "msgPayloadDefC")
    set(CLANG_LANG_FLAG "c")
  else()
    set(CLANG_LANG_FLAG "c++")
  endif()

  foreach(msgFile IN LISTS ${messageFilesVariable})
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
    get_filename_component(
      MESSAGE_HEADER_ABSOLUTE "${msgFile}" ABSOLUTE
      BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
    set(MESSAGE_IS_EXTERNAL FALSE)
    if(NOT "${EXTERNAL_MODULES_PATH}" STREQUAL "")
      cmake_path(
        IS_PREFIX EXTERNAL_MODULES_PATH "${MESSAGE_HEADER_ABSOLUTE}"
        NORMALIZE MESSAGE_IS_EXTERNAL)
    endif()
    if(MESSAGE_IS_EXTERNAL)
      file(RELATIVE_PATH MESSAGE_HEADER_INCLUDE
           "${EXTERNAL_MODULES_PATH}" "${MESSAGE_HEADER_ABSOLUTE}")
    else()
      file(RELATIVE_PATH MESSAGE_HEADER_INCLUDE
           "${CMAKE_SOURCE_DIR}" "${MESSAGE_HEADER_ABSOLUTE}")
    endif()
    add_custom_command(
      OUTPUT ${EQUALITY_OUT_NAME}
      COMMAND ${Python3_EXECUTABLE}
              msgAutoSource/generatePayloadEqualityHeader.py
              ${EQUALITY_OUT_NAME} ${MSG_META_OUTPUT} ${TARGET_NAME}
              ${MESSAGE_HEADER_INCLUDE}
      DEPENDS ${MSG_META_OUTPUT} msgAutoSource/generatePayloadEqualityHeader.py
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/"
      COMMENT "Generating payload equality header for ${TARGET_NAME}"
      VERBATIM
    )

    get_filename_component(TARGET_DIR ${msgFile} DIRECTORY)
    set(COMP_OUT_NAME "${CMAKE_BINARY_DIR}/autoSource/${TARGET_NAME}.i")
    add_custom_command(
      OUTPUT ${COMP_OUT_NAME}
      COMMAND ${Python3_EXECUTABLE} generateSWIGModules.py ${COMP_OUT_NAME} ${msgFile} ${TARGET_NAME} ${searchDir}
              ${generateCCode} ${MSG_META_OUTPUT} ${RECORDER_PROPERTY_ROLLBACK}
      DEPENDS ${msgFile} ${MSG_META_OUTPUT} ${EQUALITY_OUT_NAME}
              msgAutoSource/generateSWIGModules.py
              msgAutoSource/msgInterfacePy.i.in
              msgAutoSource/cMsgCInterfacePy.i.in
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

    bsk_suppress_strict_warnings_for_sources(
      "${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging/${TARGET_NAME}PYTHON_wrap.cxx")

    set_property(TARGET ${TARGET_NAME} APPEND PROPERTY SWIG_DEPENDS
                 "${CMAKE_CURRENT_SOURCE_DIR}/newMessaging.ih"
                 "${CMAKE_CURRENT_SOURCE_DIR}/messaging.h"
                 "${CMAKE_CURRENT_SOURCE_DIR}/messagingBase.h"
                 "${CMAKE_CURRENT_SOURCE_DIR}/messagingBase.i"
                 "${CMAKE_CURRENT_SOURCE_DIR}/messagingSupport.i"
                 "${CMAKE_CURRENT_SOURCE_DIR}/messagingSupportTypemaps.swg")
    add_dependencies(${TARGET_NAME} messagingBase messagingSupport swigtrick)
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

# issue #1433: Python-side keep-alive helpers for Msg_C subscribers. Copy at
# build time so this helper does not make the messaging package directory exist
# during configure, before generatePackageInit.py writes messaging/__init__.py.
set(MSG_KEEP_ALIVE_PY "${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging/_msgKeepAlive.py")
add_custom_command(
  OUTPUT ${MSG_KEEP_ALIVE_PY}
  COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          "${CMAKE_CURRENT_SOURCE_DIR}/_msgKeepAlive.py"
          "${MSG_KEEP_ALIVE_PY}"
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/_msgKeepAlive.py
  VERBATIM)
add_custom_target(msgKeepAlivePy DEPENDS ${MSG_KEEP_ALIVE_PY})

# Generate package imports from the exact payload inventory used for SWIG
# targets.  This avoids a second directory scan with different file matching.
add_custom_command(
  OUTPUT ${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging/__init__.py
  COMMAND
    ${Python3_EXECUTABLE} generatePackageInit.py
    "${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging/"
    "${MESSAGE_PAYLOAD_MANIFEST}"
    "${PREVIOUS_MESSAGE_PAYLOAD_MANIFEST}"
  DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/msgAutoSource/generatePackageInit.py
    ${MESSAGE_PAYLOAD_MANIFEST}
    ${MESSAGE_PAYLOAD_FILES}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/msgAutoSource
  VERBATIM)

# Custom target for establishing dependency
add_custom_target(swigtrick DEPENDS ${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging/__init__.py)
add_dependencies(swigtrick msgKeepAlivePy)

# Build common payload-wrapper types and helpers once, inside the messaging
# package, instead of emitting them into every generated payload extension.
set(MESSAGING_SUPPORT_INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/messagingSupport.i")
set_property(SOURCE ${MESSAGING_SUPPORT_INTERFACE} PROPERTY CPLUSPLUS ON)
set_property(SOURCE ${MESSAGING_SUPPORT_INTERFACE} PROPERTY USE_TARGET_INCLUDE_DIRECTORIES TRUE)
set_property(
  SOURCE ${MESSAGING_SUPPORT_INTERFACE}
  PROPERTY SWIG_FLAGS
           "-I${CMAKE_CURRENT_SOURCE_DIR}/../"
           "-I${CMAKE_CURRENT_SOURCE_DIR}/../../"
           "-I${CMAKE_BINARY_DIR}/autoSource/"
           "-I${Python3_INCLUDE_DIRS}"
           "-I${NUMPY_INCLUDE_DIR}")

swig_add_library(
  messagingSupport
  LANGUAGE "python"
  TYPE MODULE
  SOURCES ${MESSAGING_SUPPORT_INTERFACE}
  OUTFILE_DIR "${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging/"
  OUTPUT_DIR "${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging/")
set_property(
  TARGET messagingSupport
  APPEND
  PROPERTY SWIG_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/messagingSupportTypemaps.swg")

bsk_suppress_strict_warnings_for_sources(
  "${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging/messagingSupportPYTHON_wrap.cxx")
target_include_directories(messagingSupport PRIVATE ${Python3_INCLUDE_DIRS})
target_include_directories(messagingSupport PRIVATE ${NUMPY_INCLUDE_DIR})
target_include_directories(messagingSupport PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../..)
target_link_libraries(messagingSupport PUBLIC architectureLib)
set_target_properties(messagingSupport PROPERTIES FOLDER "architecture/messaging")
set_target_properties(
  ${SWIG_MODULE_messagingSupport_REAL_NAME}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging"
             LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging"
             LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Basilisk/architecture/messaging")

# The target is created above with messaging-package output paths; do not let
# the root architecture target discovery create it a second time.
list(APPEND EXCLUDED_BSK_TARGETS messagingSupport)
set(EXCLUDED_BSK_TARGETS
    ${EXCLUDED_BSK_TARGETS}
    PARENT_SCOPE)

# Generate per-payload equality headers. Each "<Payload>_equality.h" is included
# only by that payload's own SWIG module.
generate_messages(msgPayloadDefCpp "False" CPP_MESSAGE_PAYLOAD_FILES)
generate_messages(msgPayloadDefC "True" C_MESSAGE_PAYLOAD_FILES)

set(recorder_copy_contract_test_source
    "${CMAKE_CURRENT_SOURCE_DIR}/_UnitTest/test_recorder_copy_contract.cpp")
if(BUILD_TESTING AND EXISTS "${recorder_copy_contract_test_source}")
  # Verify the central SysModel copy restriction while keeping Recorder explicitly copyable.
  add_executable(test_recorder_copy_contract "${recorder_copy_contract_test_source}")
  target_sources(test_recorder_copy_contract PRIVATE ${CMAKE_SOURCE_DIR}/architecture/_GeneralModuleFiles/sys_model.cpp)
  target_link_libraries(test_recorder_copy_contract GTest::gtest_main)
  target_link_libraries(test_recorder_copy_contract ArchitectureUtilities)
  target_link_libraries(test_recorder_copy_contract ModuleIdGenerator)

  # Windows test discovery runs the executable from this folder, so copy the required DLL beside it first.
  if(WIN32)
    add_custom_command(
      TARGET test_recorder_copy_contract
      POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              $<TARGET_FILE:ModuleIdGenerator>
              $<TARGET_FILE_DIR:test_recorder_copy_contract>)
  endif()

  # Apple-silicon Xcode builds cannot reliably run the test during the build, so defer discovery to CTest.
  if(CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "arm64" AND CMAKE_GENERATOR STREQUAL "Xcode")
    gtest_discover_tests(test_recorder_copy_contract DISCOVERY_MODE PRE_TEST)
  else()
    gtest_discover_tests(test_recorder_copy_contract)
  endif()
endif()
