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

# A macro keeps the adjusted compiler flags in the calling directory scope.
macro(bsk_remove_msvc_default_warning_levels)
  # Remove toolchain warning levels before assigning one level to each source.
  foreach(LANGUAGE C CXX)
    foreach(CONFIGURATION "" _DEBUG _RELEASE _RELWITHDEBINFO _MINSIZEREL)
      set(FLAG_VARIABLE "CMAKE_${LANGUAGE}_FLAGS${CONFIGURATION}")
      string(REGEX REPLACE "(^| )[/-]W[0-4]( |$)" " " ${FLAG_VARIABLE} "${${FLAG_VARIABLE}}")
    endforeach()
  endforeach()
endmacro()

# Choose warning levels per source so generated files never receive both /W4 and /w.
function(bsk_set_msvc_warning_levels SOURCE_DIRECTORY)
  get_property(DIRECTORY_TARGETS DIRECTORY "${SOURCE_DIRECTORY}" PROPERTY BUILDSYSTEM_TARGETS)
  foreach(TARGET_NAME IN LISTS DIRECTORY_TARGETS)
    get_target_property(TARGET_SOURCES "${TARGET_NAME}" SOURCES)
    foreach(SOURCE_FILE IN LISTS TARGET_SOURCES)
      if(NOT SOURCE_FILE MATCHES "\\.(c|cc|cpp|cxx|C)$")
        continue()
      endif()
      cmake_path(ABSOLUTE_PATH SOURCE_FILE BASE_DIRECTORY "${SOURCE_DIRECTORY}")
      get_source_file_property(SOURCE_OPTIONS "${SOURCE_FILE}" DIRECTORY "${SOURCE_DIRECTORY}" COMPILE_OPTIONS)
      if(NOT "/w" IN_LIST SOURCE_OPTIONS AND NOT "/W4" IN_LIST SOURCE_OPTIONS)
        set_property(SOURCE "${SOURCE_FILE}" DIRECTORY "${SOURCE_DIRECTORY}"
                     APPEND PROPERTY COMPILE_OPTIONS /W4)
      endif()
    endforeach()
  endforeach()
  get_property(SUBDIRECTORIES DIRECTORY "${SOURCE_DIRECTORY}" PROPERTY SUBDIRECTORIES)
  foreach(SUBDIRECTORY IN LISTS SUBDIRECTORIES)
    bsk_set_msvc_warning_levels("${SUBDIRECTORY}")
  endforeach()
endfunction()
