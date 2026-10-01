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

option(BSK_COMBINE_SIMULATION_BINDINGS "Experimentally combine core simulation bindings" OFF)
option(BSK_COMBINE_MUJOCO_BINDINGS "Experimentally combine MuJoCo bindings" OFF)
option(BSK_COMBINE_OPNAV_BINDINGS "Experimentally combine OpenCV bindings" OFF)

function(bsk_select_simulation_binding_group OUTPUT MODULE_NAME PARENT_DIR MODULE_DIR)
  set(GROUP "")
  if(MODULE_DIR STREQUAL "simulation" OR MODULE_DIR STREQUAL "fswAlgorithms")
    bsk_collect_wrapper_custom_files(CUSTOM_FILES "${PARENT_DIR}" "${MODULE_DIR}"
      "${CMAKE_SOURCE_DIR}" "${EXTERNAL_MODULES_PATH}" "${BSK_CUSTOM_CMAKE_FILES}")
    set(USES_OPENCV FALSE)
    foreach(CUSTOM_FILE IN LISTS CUSTOM_FILES)
      file(READ "${CUSTOM_FILE}" CUSTOM_CONTENT)
      if(CUSTOM_CONTENT MATCHES "include[ \t]*\\([ \t]*usingOpenCV[ \t]*\\)")
        set(USES_OPENCV TRUE)
      endif()
    endforeach()
    if(USES_OPENCV)
      if(BSK_COMBINE_OPNAV_BINDINGS)
        set(GROUP opNavNative)
      endif()
    elseif(MODULE_DIR STREQUAL "simulation")
      if(PARENT_DIR MATCHES "(^|/)mujocoDynamics(/|$)")
        if(BSK_COMBINE_MUJOCO_BINDINGS)
          set(GROUP mujocoNative)
        endif()
      elseif(BSK_COMBINE_SIMULATION_BINDINGS)
        # These two custom files add only existing core Basilisk libraries.
        # Keep Vizard and unrecognized custom dependencies outside the core group.
        if(NOT CUSTOM_FILES OR MODULE_NAME MATCHES "^(downlinkHandling|spacecraftChargingDynamics)$")
          set(GROUP simulationCoreNative)
        endif()
      endif()
    endif()
  endif()
  set(${OUTPUT} "${GROUP}" PARENT_SCOPE)
endfunction()

function(bsk_add_binding_group GROUP PACKAGE LOADER)
  get_property(OBJECT_TARGETS GLOBAL PROPERTY BSK_${GROUP}_OBJECT_TARGETS)
  get_property(MODULE_NAMES GLOBAL PROPERTY BSK_${GROUP}_MODULES)
  list(SORT MODULE_NAMES)
  string(REPLACE ";" "\n" CONTENT "${MODULE_NAMES}")
  set(MANIFEST "${CMAKE_BINARY_DIR}/autoSource/${GROUP}Bindings.txt")
  file(CONFIGURE OUTPUT "${MANIFEST}" CONTENT "${CONTENT}\n" @ONLY)
  get_property(PROXIES GLOBAL PROPERTY BSK_${GROUP}_PROXIES)
  list(SORT PROXIES)
  string(REPLACE ";" "\n" PROXY_CONTENT "${PROXIES}")
  file(CONFIGURE OUTPUT "${CMAKE_BINARY_DIR}/autoSource/${GROUP}Proxies.txt"
    CONTENT "${PROXY_CONTENT}\n" @ONLY)
  set(PACKAGE_ROOT "${CMAKE_BINARY_DIR}/Basilisk")
  set(OUTPUT_DIR "${PACKAGE_ROOT}/${PACKAGE}")
  set(NATIVE_NAME "_${GROUP}")
  if(PACKAGE)
    set(NATIVE_NAME "${PACKAGE}.${NATIVE_NAME}")
  endif()
  set(STAMP "${CMAKE_BINARY_DIR}/autoSource/${GROUP}Layout.stamp")
  set(OUTPUTS "${STAMP}")
  foreach(MODULE_NAME IN LISTS MODULE_NAMES)
    string(REGEX REPLACE "^([^.]+)\\.(.+)$" "\\1/_\\2.py" SHIM "${MODULE_NAME}")
    list(APPEND OUTPUTS "${PACKAGE_ROOT}/${SHIM}")
  endforeach()
  if(OBJECT_TARGETS)
    # A source on the aggregate is required for Xcode to create its link phase.
    set(ANCHOR "${CMAKE_BINARY_DIR}/autoSource/${GROUP}.cpp")
    file(CONFIGURE OUTPUT "${ANCHOR}" CONTENT "// Generated empty translation unit for the Xcode link phase.\n" @ONLY)
    add_library(${GROUP} MODULE "${ANCHOR}")
    target_link_libraries(${GROUP} PRIVATE ${OBJECT_TARGETS})
    set_target_properties(${GROUP} PROPERTIES PREFIX "_" NO_SONAME ON LINKER_LANGUAGE CXX FOLDER "simulation")
    foreach(KIND LIBRARY RUNTIME ARCHIVE)
      set_target_properties(${GROUP} PROPERTIES ${KIND}_OUTPUT_DIRECTORY "${OUTPUT_DIR}")
      foreach(CONFIG DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
        set_target_properties(${GROUP} PROPERTIES ${KIND}_OUTPUT_DIRECTORY_${CONFIG} "${OUTPUT_DIR}")
      endforeach()
    endforeach()
    if(WIN32)
      set_target_properties(${GROUP} PROPERTIES SUFFIX ".pyd")
    endif()
    string(REPLACE "." "/" LOADER_PATH "${LOADER}")
    list(APPEND OUTPUTS "${PACKAGE_ROOT}/${LOADER_PATH}.py")
    add_dependencies(${GROUP} ${GROUP}Layout)
  endif()
  add_custom_command(OUTPUT ${OUTPUTS}
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/cmake/generateGroupedBindings.py"
      "${MANIFEST}" "${PACKAGE_ROOT}" "${CMAKE_SOURCE_DIR}/fswAlgorithms/_load_fsw.py" "${NATIVE_NAME}" "${LOADER}"
    COMMAND "${CMAKE_COMMAND}" -E touch "${STAMP}"
    DEPENDS "${MANIFEST}" "${CMAKE_SOURCE_DIR}/cmake/generateGroupedBindings.py"
      "${CMAKE_SOURCE_DIR}/fswAlgorithms/_load_fsw.py"
    COMMENT "Updating ${GROUP} binding layout" VERBATIM)
  add_custom_target(${GROUP}Layout DEPENDS ${OUTPUTS})
  # Clean old binaries/shims before either separate or combined targets build.
  get_property(BINDING_TARGETS GLOBAL PROPERTY BSK_SIMULATION_BINDING_TARGETS)
  foreach(BINDING_TARGET IN LISTS BINDING_TARGETS)
    add_dependencies(${BINDING_TARGET} ${GROUP}Layout)
  endforeach()
endfunction()

function(bsk_finalize_simulation_bindings)
  bsk_add_binding_group(simulationCoreNative "simulation" "simulation._load_simulation")
  bsk_add_binding_group(mujocoNative "simulation" "simulation._load_mujoco")
  bsk_add_binding_group(opNavNative "" "_load_opnav")
endfunction()
