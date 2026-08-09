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

function(
  bsk_collect_wrapper_custom_files
  OUTPUT_VARIABLE
  PARENT_DIR
  MODULE_DIR
  SOURCE_DIR
  EXTERNAL_PROJECT_DIR
  CUSTOM_FILE_INVENTORY)
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
  if(MODULE_CUSTOM_FILE IN_LIST CUSTOM_FILE_INVENTORY)
    list(APPEND CUSTOM_FILES "${MODULE_CUSTOM_FILE}")
  endif()

  cmake_path(IS_PREFIX MODULE_ROOT "${WRAPPER_SOURCE_DIR}" NORMALIZE WRAPPER_IS_IN_MODULE_ROOT)
  if(WRAPPER_IS_IN_MODULE_ROOT)
    set(SEARCH_DIR "${WRAPPER_SOURCE_DIR}")
    while(TRUE)
      set(SHARED_CUSTOM_FILE "${SEARCH_DIR}/_GeneralModuleFiles/Custom.cmake")
      if(SHARED_CUSTOM_FILE IN_LIST CUSTOM_FILE_INVENTORY)
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
