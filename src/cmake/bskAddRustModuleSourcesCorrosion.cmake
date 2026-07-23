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

# Keep the Corrosion trial isolated from the existing Cargo integration until
# its behavior has been exercised on every supported platform. FetchContent is
# evaluated only when BSK_RUST_USE_CORROSION is enabled.
function(_bsk_load_corrosion)
  if(COMMAND corrosion_import_crate)
    return()
  endif()

  include(FetchContent)
  FetchContent_Declare(
    Corrosion
    GIT_REPOSITORY https://github.com/corrosion-rs/corrosion.git
    # Corrosion v0.6.1. Pin the immutable commit rather than the movable v0.6
    # tag so a clean Basilisk configure cannot acquire different build logic.
    GIT_TAG 1499b14e4906a2890f5cee1547c8848db261753d
  )
  FetchContent_MakeAvailable(Corrosion)
endfunction()

# Build one Rust Basilisk module through Corrosion. This deliberately mirrors
# only the inputs needed by the in-tree rustModuleTemplate trial; the legacy
# bsk_add_rust_module_sources() function remains the default build path.
function(bsk_add_rust_module_sources_corrosion)
  set(_one
      TARGET
      PACKAGE_NAME
      HEADER
      MANIFEST
      INTERFACE
      INCLUDE_DIR
      OUT_LINK_TARGET_VAR
      OUT_HEADER_VAR
      OUT_INTERFACE_VAR
      OUT_BUILD_TARGET_VAR)
  set(_multi CARGO_FEATURES CARGO_ENV)
  cmake_parse_arguments(RUST "" "${_one}" "${_multi}" ${ARGN})

  if(NOT RUST_TARGET)
    message(FATAL_ERROR
      "bsk_add_rust_module_sources_corrosion: TARGET is required")
  endif()
  if(NOT RUST_MANIFEST)
    message(FATAL_ERROR
      "bsk_add_rust_module_sources_corrosion: MANIFEST is required")
  endif()

  get_filename_component(_manifest "${RUST_MANIFEST}" ABSOLUTE)
  if(NOT EXISTS "${_manifest}")
    message(FATAL_ERROR
      "bsk_add_rust_module_sources_corrosion: manifest not found: ${_manifest}")
  endif()

  if(NOT RUST_PACKAGE_NAME)
    set(RUST_PACKAGE_NAME "${RUST_TARGET}")
  endif()
  if(NOT RUST_INCLUDE_DIR)
    set(RUST_INCLUDE_DIR "${CMAKE_SOURCE_DIR}")
  endif()
  if(NOT RUST_HEADER)
    set(RUST_HEADER
        "${CMAKE_CURRENT_BINARY_DIR}/rust_headers/${RUST_TARGET}.h")
  endif()
  if(NOT RUST_INTERFACE)
    set(RUST_INTERFACE
        "${CMAKE_CURRENT_BINARY_DIR}/${RUST_TARGET}_rust_wrap.i")
  endif()

  _bsk_load_corrosion()
  corrosion_import_crate(
    MANIFEST_PATH "${_manifest}"
    CRATES "${RUST_PACKAGE_NAME}"
    CRATE_TYPES staticlib
    LOCKED
    IMPORTED_CRATES _rust_imported_targets
  )

  list(LENGTH _rust_imported_targets _rust_imported_target_count)
  if(NOT _rust_imported_target_count EQUAL 1)
    message(FATAL_ERROR
      "Corrosion imported ${_rust_imported_target_count} targets for "
      "${RUST_PACKAGE_NAME}: expected exactly one static library target")
  endif()
  list(GET _rust_imported_targets 0 _rust_link_target)

  corrosion_set_env_vars(
    "${_rust_link_target}"
    "BSK_INCLUDE_DIR=${RUST_INCLUDE_DIR}"
    "BSK_HEADER_PATH=${RUST_HEADER}"
    "BSK_INTERFACE_PATH=${RUST_INTERFACE}"
    ${RUST_CARGO_ENV}
  )
  if(RUST_CARGO_FEATURES)
    corrosion_set_features(
      "${_rust_link_target}"
      FEATURES ${RUST_CARGO_FEATURES}
    )
  endif()

  # Corrosion owns the Cargo invocation and static-library byproduct, while
  # this module's build.rs owns the generated header and SWIG interface.
  # Give CMake an explicit producer rule for those two files so Make/Ninja do
  # not reject the missing generated interface before Cargo has run.
  set(_cargo_build_target "cargo-build_${_rust_link_target}")
  set(_bindings_target "_rust_bindings_${RUST_TARGET}")
  add_custom_command(
    OUTPUT "${RUST_HEADER}" "${RUST_INTERFACE}"
    COMMAND "${CMAKE_COMMAND}" -E compare_files
            "${RUST_HEADER}" "${RUST_HEADER}"
    COMMAND "${CMAKE_COMMAND}" -E compare_files
            "${RUST_INTERFACE}" "${RUST_INTERFACE}"
    DEPENDS "${_cargo_build_target}"
    COMMENT "Verifying generated Rust bindings for '${RUST_TARGET}'"
    VERBATIM
  )
  add_custom_target(
    "${_bindings_target}"
    DEPENDS "${RUST_HEADER}" "${RUST_INTERFACE}"
  )

  if(RUST_OUT_LINK_TARGET_VAR)
    set("${RUST_OUT_LINK_TARGET_VAR}" "${_rust_link_target}" PARENT_SCOPE)
  endif()
  if(RUST_OUT_HEADER_VAR)
    set("${RUST_OUT_HEADER_VAR}" "${RUST_HEADER}" PARENT_SCOPE)
  endif()
  if(RUST_OUT_INTERFACE_VAR)
    set("${RUST_OUT_INTERFACE_VAR}" "${RUST_INTERFACE}" PARENT_SCOPE)
  endif()
  if(RUST_OUT_BUILD_TARGET_VAR)
    set("${RUST_OUT_BUILD_TARGET_VAR}" "${_bindings_target}" PARENT_SCOPE)
  endif()
endfunction()
