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

include_guard(GLOBAL)

function(bsk_collect_optional_cmake_file OUTPUT_VARIABLE FILE_PATH)
  cmake_path(
    ABSOLUTE_PATH
    FILE_PATH
    BASE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    NORMALIZE
    OUTPUT_VARIABLE ABSOLUTE_FILE_PATH)

  # CONFIGURE_DEPENDS is available only while configuring a project, not in
  # `cmake -P` script mode. The latter is used by the focused helper test.
  if(CMAKE_SCRIPT_MODE_FILE)
    file(GLOB OPTIONAL_CMAKE_FILE "${ABSOLUTE_FILE_PATH}")
  else()
    file(GLOB OPTIONAL_CMAKE_FILE CONFIGURE_DEPENDS "${ABSOLUTE_FILE_PATH}")
  endif()
  set("${OUTPUT_VARIABLE}" "${OPTIONAL_CMAKE_FILE}" PARENT_SCOPE)
endfunction(bsk_collect_optional_cmake_file)

function(bsk_collect_custom_cmake_files OUTPUT_VARIABLE ROOT_DIRECTORY)
  cmake_path(
    ABSOLUTE_PATH
    ROOT_DIRECTORY
    BASE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    NORMALIZE
    OUTPUT_VARIABLE ABSOLUTE_ROOT_DIRECTORY)

  # One recursive glob tracks all module-local and shared hooks without adding
  # a configure-time glob for every wrapper and ancestor directory.
  if(CMAKE_SCRIPT_MODE_FILE)
    file(GLOB_RECURSE CUSTOM_CMAKE_FILES "${ABSOLUTE_ROOT_DIRECTORY}/Custom.cmake")
  else()
    file(GLOB_RECURSE CUSTOM_CMAKE_FILES CONFIGURE_DEPENDS
         "${ABSOLUTE_ROOT_DIRECTORY}/Custom.cmake")
  endif()
  set("${OUTPUT_VARIABLE}" "${CUSTOM_CMAKE_FILES}" PARENT_SCOPE)
endfunction(bsk_collect_custom_cmake_files)

function(
  bsk_collect_wrapper_custom_files
  OUTPUT_VARIABLE
  PARENT_DIR
  MODULE_DIR
  SOURCE_DIR
  EXTERNAL_PROJECT_DIR)
  cmake_path(
    ABSOLUTE_PATH
    PARENT_DIR
    BASE_DIRECTORY "${SOURCE_DIR}"
    NORMALIZE
    OUTPUT_VARIABLE WRAPPER_SOURCE_DIR)

  if("${MODULE_DIR}" STREQUAL "ExternalModules")
    set(MODULE_ROOT "${EXTERNAL_PROJECT_DIR}/ExternalModules")
  else()
    set(MODULE_ROOT "${SOURCE_DIR}/${MODULE_DIR}")
  endif()
  cmake_path(NORMAL_PATH MODULE_ROOT OUTPUT_VARIABLE MODULE_ROOT)

  set(CUSTOM_FILES)
  set(MODULE_CUSTOM_FILE "${WRAPPER_SOURCE_DIR}/Custom.cmake")
  if(EXISTS "${MODULE_CUSTOM_FILE}")
    list(APPEND CUSTOM_FILES "${MODULE_CUSTOM_FILE}")
  endif()

  cmake_path(IS_PREFIX MODULE_ROOT "${WRAPPER_SOURCE_DIR}" NORMALIZE WRAPPER_IS_IN_MODULE_ROOT)
  if(WRAPPER_IS_IN_MODULE_ROOT)
    set(SEARCH_DIR "${WRAPPER_SOURCE_DIR}")
    while(TRUE)
      set(SHARED_CUSTOM_FILE "${SEARCH_DIR}/_GeneralModuleFiles/Custom.cmake")
      if(EXISTS "${SHARED_CUSTOM_FILE}")
        list(APPEND CUSTOM_FILES "${SHARED_CUSTOM_FILE}")
      endif()
      if("${SEARCH_DIR}" STREQUAL "${MODULE_ROOT}")
        break()
      endif()

      get_filename_component(PARENT_SEARCH_DIR "${SEARCH_DIR}" DIRECTORY)
      if("${PARENT_SEARCH_DIR}" STREQUAL "${SEARCH_DIR}")
        break()
      endif()
      cmake_path(IS_PREFIX MODULE_ROOT "${PARENT_SEARCH_DIR}" NORMALIZE PARENT_IS_IN_MODULE_ROOT)
      if(NOT PARENT_IS_IN_MODULE_ROOT)
        break()
      endif()
      set(SEARCH_DIR "${PARENT_SEARCH_DIR}")
    endwhile()
  endif()

  list(REMOVE_DUPLICATES CUSTOM_FILES)
  set("${OUTPUT_VARIABLE}" "${CUSTOM_FILES}" PARENT_SCOPE)
endfunction(bsk_collect_wrapper_custom_files)
