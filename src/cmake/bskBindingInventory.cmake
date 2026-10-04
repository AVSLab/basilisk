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

# Call after all binding targets are registered. Cleanup uses this complete
# inventory instead of inspecting shims that another group may be replacing.
function(bsk_write_binding_inventory OUTPUT)
  get_property(FSW_NAMES GLOBAL PROPERTY BSK_FSW_COMBINED_MODULES)
  set(NAMES)
  foreach(NAME IN LISTS FSW_NAMES)
    list(APPEND NAMES "fswAlgorithms.${NAME}")
  endforeach()
  foreach(GROUP simulationCoreNative mujocoNative opNavNative)
    get_property(GROUP_NAMES GLOBAL PROPERTY BSK_${GROUP}_MODULES)
    list(APPEND NAMES ${GROUP_NAMES})
  endforeach()
  list(REMOVE_DUPLICATES NAMES)
  list(SORT NAMES)
  string(REPLACE ";" "\n" CONTENT "${NAMES}")
  set(INVENTORY "${CMAKE_BINARY_DIR}/autoSource/combinedBindingModules.txt")
  file(CONFIGURE OUTPUT "${INVENTORY}" CONTENT "${CONTENT}\n" @ONLY)
  set(${OUTPUT} "${INVENTORY}" PARENT_SCOPE)
endfunction()
