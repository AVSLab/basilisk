# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

cmake_minimum_required(VERSION 3.26)

if(NOT DEFINED BSK_SOURCE_DIR OR NOT DEFINED TEST_ROOT)
  message(FATAL_ERROR "BSK_SOURCE_DIR and TEST_ROOT are required.")
endif()

include("${BSK_SOURCE_DIR}/cmake/bskCollectWrapperCustomFiles.cmake")

set(FIXTURE_SOURCE_DIR "${TEST_ROOT}/source")
set(FIXTURE_EXTERNAL_DIR "${TEST_ROOT}/external")

set(IN_TREE_MODULE_DIR "${FIXTURE_SOURCE_DIR}/simulation/package/module")
set(IN_TREE_LOCAL_CUSTOM "${IN_TREE_MODULE_DIR}/Custom.cmake")
set(IN_TREE_SHARED_CUSTOM "${FIXTURE_SOURCE_DIR}/simulation/package/_GeneralModuleFiles/Custom.cmake")
file(MAKE_DIRECTORY "${IN_TREE_MODULE_DIR}")
file(MAKE_DIRECTORY "${FIXTURE_SOURCE_DIR}/simulation/package/_GeneralModuleFiles")
file(WRITE "${IN_TREE_LOCAL_CUSTOM}" "")
file(WRITE "${IN_TREE_SHARED_CUSTOM}" "")

bsk_collect_wrapper_custom_files(
  IN_TREE_CUSTOM_FILES
  "simulation/package/module"
  "simulation"
  "${FIXTURE_SOURCE_DIR}"
  "${FIXTURE_EXTERNAL_DIR}")
set(EXPECTED_IN_TREE_CUSTOM_FILES "${IN_TREE_LOCAL_CUSTOM};${IN_TREE_SHARED_CUSTOM}")
if(NOT "${IN_TREE_CUSTOM_FILES}" STREQUAL "${EXPECTED_IN_TREE_CUSTOM_FILES}")
  message(FATAL_ERROR
    "Unexpected in-tree Custom.cmake files: ${IN_TREE_CUSTOM_FILES}")
endif()

set(EXTERNAL_MODULE_DIR "${FIXTURE_EXTERNAL_DIR}/ExternalModules/package/module")
set(EXTERNAL_LOCAL_CUSTOM "${EXTERNAL_MODULE_DIR}/Custom.cmake")
set(EXTERNAL_SHARED_CUSTOM "${FIXTURE_EXTERNAL_DIR}/ExternalModules/_GeneralModuleFiles/Custom.cmake")
file(MAKE_DIRECTORY "${EXTERNAL_MODULE_DIR}")
file(MAKE_DIRECTORY "${FIXTURE_EXTERNAL_DIR}/ExternalModules/_GeneralModuleFiles")
file(WRITE "${EXTERNAL_LOCAL_CUSTOM}" "")
file(WRITE "${EXTERNAL_SHARED_CUSTOM}" "")
file(RELATIVE_PATH EXTERNAL_PARENT_DIR "${FIXTURE_SOURCE_DIR}" "${EXTERNAL_MODULE_DIR}")

bsk_collect_wrapper_custom_files(
  EXTERNAL_CUSTOM_FILES
  "${EXTERNAL_PARENT_DIR}"
  "ExternalModules"
  "${FIXTURE_SOURCE_DIR}"
  "${FIXTURE_EXTERNAL_DIR}")
set(EXPECTED_EXTERNAL_CUSTOM_FILES "${EXTERNAL_LOCAL_CUSTOM};${EXTERNAL_SHARED_CUSTOM}")
if(NOT "${EXTERNAL_CUSTOM_FILES}" STREQUAL "${EXPECTED_EXTERNAL_CUSTOM_FILES}")
  message(FATAL_ERROR
    "Unexpected external Custom.cmake files: ${EXTERNAL_CUSTOM_FILES}")
endif()

set(CONFIGURE_PROJECT_DIR "${TEST_ROOT}/configure-dependency-source")
set(CONFIGURE_BUILD_DIR "${TEST_ROOT}/configure-dependency-build")
set(OPTIONAL_CONFIG_DIR "${CONFIGURE_PROJECT_DIR}/package/_GeneralModuleFiles")
set(OPTIONAL_CONFIG_FILE "${OPTIONAL_CONFIG_DIR}/Custom.cmake")
set(OPTIONAL_MANIFEST_FILE "${OPTIONAL_CONFIG_DIR}/PackageSources.cmake")
set(DISCOVERED_CONFIG_FILE "${CONFIGURE_BUILD_DIR}/discovered-config.txt")
set(DISCOVERED_MANIFEST_FILE "${CONFIGURE_BUILD_DIR}/discovered-manifest.txt")
file(MAKE_DIRECTORY "${OPTIONAL_CONFIG_DIR}")

set(CONFIGURE_PROJECT_CONTENT [=[
cmake_minimum_required(VERSION 3.26)
project(bsk_configure_dependency_test NONE)
include("@BSK_SOURCE_DIR@/cmake/bskCollectWrapperCustomFiles.cmake")
bsk_collect_custom_cmake_files(
  DISCOVERED_CONFIG
  "${CMAKE_SOURCE_DIR}")
bsk_collect_optional_cmake_file(
  DISCOVERED_MANIFEST
  "${CMAKE_SOURCE_DIR}/package/_GeneralModuleFiles/PackageSources.cmake")
file(WRITE "${CMAKE_BINARY_DIR}/discovered-config.txt" "${DISCOVERED_CONFIG}")
file(WRITE "${CMAKE_BINARY_DIR}/discovered-manifest.txt" "${DISCOVERED_MANIFEST}")
]=])
string(REPLACE
  "@BSK_SOURCE_DIR@"
  "${BSK_SOURCE_DIR}"
  CONFIGURE_PROJECT_CONTENT
  "${CONFIGURE_PROJECT_CONTENT}")
file(WRITE "${CONFIGURE_PROJECT_DIR}/CMakeLists.txt" "${CONFIGURE_PROJECT_CONTENT}")

execute_process(
  COMMAND "${CMAKE_COMMAND}" -S "${CONFIGURE_PROJECT_DIR}" -B "${CONFIGURE_BUILD_DIR}"
  RESULT_VARIABLE INITIAL_CONFIGURE_RESULT
  OUTPUT_VARIABLE INITIAL_CONFIGURE_OUTPUT
  ERROR_VARIABLE INITIAL_CONFIGURE_ERROR)
if(NOT INITIAL_CONFIGURE_RESULT EQUAL 0)
  message(FATAL_ERROR
    "Configure-dependency fixture failed to configure:\n"
    "${INITIAL_CONFIGURE_OUTPUT}\n${INITIAL_CONFIGURE_ERROR}")
endif()
file(READ "${DISCOVERED_CONFIG_FILE}" INITIAL_DISCOVERED_CONFIG)
if(INITIAL_DISCOVERED_CONFIG)
  message(FATAL_ERROR
    "Absent optional CMake file was unexpectedly discovered: ${INITIAL_DISCOVERED_CONFIG}")
endif()
file(READ "${DISCOVERED_MANIFEST_FILE}" INITIAL_DISCOVERED_MANIFEST)
if(INITIAL_DISCOVERED_MANIFEST)
  message(FATAL_ERROR
    "Absent ownership manifest was unexpectedly discovered: ${INITIAL_DISCOVERED_MANIFEST}")
endif()

file(WRITE "${OPTIONAL_MANIFEST_FILE}" "# Ownership-manifest fixture.\n")
execute_process(
  COMMAND "${CMAKE_COMMAND}" --build "${CONFIGURE_BUILD_DIR}"
  RESULT_VARIABLE MANIFEST_RECONFIGURE_RESULT
  OUTPUT_VARIABLE MANIFEST_RECONFIGURE_OUTPUT
  ERROR_VARIABLE MANIFEST_RECONFIGURE_ERROR)
if(NOT MANIFEST_RECONFIGURE_RESULT EQUAL 0)
  message(FATAL_ERROR
    "Adding an ownership manifest did not rebuild the fixture:\n"
    "${MANIFEST_RECONFIGURE_OUTPUT}\n${MANIFEST_RECONFIGURE_ERROR}")
endif()
file(READ "${DISCOVERED_CONFIG_FILE}" MANIFEST_DISCOVERED_CONFIG)
if(MANIFEST_DISCOVERED_CONFIG)
  message(FATAL_ERROR
    "Absent optional CMake file was unexpectedly discovered: ${MANIFEST_DISCOVERED_CONFIG}")
endif()
file(READ "${DISCOVERED_MANIFEST_FILE}" UPDATED_DISCOVERED_MANIFEST)
if(NOT "${UPDATED_DISCOVERED_MANIFEST}" STREQUAL "${OPTIONAL_MANIFEST_FILE}")
  message(FATAL_ERROR
    "New ownership manifest did not trigger reconfiguration: ${UPDATED_DISCOVERED_MANIFEST}")
endif()

file(WRITE "${OPTIONAL_CONFIG_FILE}" "# Configure-dependency fixture.\n")
execute_process(
  COMMAND "${CMAKE_COMMAND}" --build "${CONFIGURE_BUILD_DIR}"
  RESULT_VARIABLE CONFIG_RECONFIGURE_RESULT
  OUTPUT_VARIABLE CONFIG_RECONFIGURE_OUTPUT
  ERROR_VARIABLE CONFIG_RECONFIGURE_ERROR)
if(NOT CONFIG_RECONFIGURE_RESULT EQUAL 0)
  message(FATAL_ERROR
    "Adding an optional CMake file did not rebuild the fixture:\n"
    "${CONFIG_RECONFIGURE_OUTPUT}\n${CONFIG_RECONFIGURE_ERROR}")
endif()
file(READ "${DISCOVERED_CONFIG_FILE}" UPDATED_DISCOVERED_CONFIG)
if(NOT "${UPDATED_DISCOVERED_CONFIG}" STREQUAL "${OPTIONAL_CONFIG_FILE}")
  message(FATAL_ERROR
    "New optional CMake file did not trigger reconfiguration: ${UPDATED_DISCOVERED_CONFIG}")
endif()

message(STATUS "Package Custom.cmake discovery passed.")
message(STATUS "Optional CMake file and ownership-manifest creation triggered reconfiguration.")
