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

set(BSK_CORROSION_VERSION "0.6.1")
set(BSK_CORROSION_GIT_TAG "1499b14e4906a2890f5cee1547c8848db261753d")

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
    GIT_TAG ${BSK_CORROSION_GIT_TAG}
  )
  FetchContent_MakeAvailable(Corrosion)
endfunction()

# Read the Cargo package that owns a manifest without parsing Cargo.toml.
# Corrosion also consumes Cargo metadata internally; this small query supplies
# the package allowlist needed to import only the requested workspace member.
function(_bsk_rust_package_name_from_metadata MANIFEST OUT_PACKAGE_NAME)
  get_target_property(_cargo_executable Rust::Cargo IMPORTED_LOCATION)
  if(NOT _cargo_executable)
    message(FATAL_ERROR
      "Corrosion did not provide the Rust::Cargo executable target")
  endif()

  file(REAL_PATH "${MANIFEST}" _requested_manifest)
  execute_process(
    COMMAND
      "${_cargo_executable}" metadata
      --locked
      --no-deps
      --format-version 1
      --manifest-path "${_requested_manifest}"
    RESULT_VARIABLE _metadata_result
    OUTPUT_VARIABLE _metadata_json
    ERROR_VARIABLE _metadata_error
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_STRIP_TRAILING_WHITESPACE
  )
  if(NOT _metadata_result EQUAL 0)
    message(FATAL_ERROR
      "Could not read Cargo metadata for ${_requested_manifest}:\n"
      "${_metadata_error}")
  endif()

  string(JSON _package_count
         ERROR_VARIABLE _metadata_json_error
         LENGTH "${_metadata_json}" packages)
  if(_metadata_json_error)
    message(FATAL_ERROR
      "Cargo returned invalid metadata for ${_requested_manifest}:\n"
      "${_metadata_json_error}")
  endif()
  if(_package_count EQUAL 0)
    message(FATAL_ERROR
      "Cargo metadata contains no packages for ${_requested_manifest}")
  endif()

  set(_matching_packages 0)
  math(EXPR _last_package_index "${_package_count} - 1")
  foreach(_package_index RANGE 0 ${_last_package_index})
    string(JSON _candidate_manifest
           GET "${_metadata_json}" packages ${_package_index} manifest_path)
    file(REAL_PATH "${_candidate_manifest}" _candidate_manifest)
    if("${_candidate_manifest}" STREQUAL "${_requested_manifest}")
      math(EXPR _matching_packages "${_matching_packages} + 1")
      string(JSON _package_name
             GET "${_metadata_json}" packages ${_package_index} name)
    endif()
  endforeach()

  if(NOT _matching_packages EQUAL 1)
    message(FATAL_ERROR
      "Cargo metadata matched ${_matching_packages} packages to "
      "${_requested_manifest}; expected exactly one")
  endif()
  set("${OUT_PACKAGE_NAME}" "${_package_name}" PARENT_SCOPE)
endfunction()

# Build one Rust Basilisk module through Corrosion. Cargo metadata determines
# the package and library target names; Corrosion determines the profile and
# platform artifact path. The legacy bsk_add_rust_module_sources() function
# remains available as the comparison and rollback path.
function(bsk_add_rust_module_sources_corrosion)
  set(_one
      TARGET
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
  set_property(DIRECTORY APPEND PROPERTY
               CMAKE_CONFIGURE_DEPENDS "${_manifest}")

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
  set(_bindings_trigger
      "${CMAKE_CURRENT_BINARY_DIR}/rust_bindings/${RUST_TARGET}.trigger")

  _bsk_load_corrosion()
  _bsk_rust_package_name_from_metadata("${_manifest}" _rust_package_name)
  corrosion_import_crate(
    MANIFEST_PATH "${_manifest}"
    CRATES "${_rust_package_name}"
    CRATE_TYPES staticlib
    LOCKED
    IMPORTED_CRATES _rust_imported_targets
  )

  list(LENGTH _rust_imported_targets _rust_imported_target_count)
  if(NOT _rust_imported_target_count EQUAL 1)
    message(FATAL_ERROR
      "Corrosion imported ${_rust_imported_target_count} targets for "
      "${_rust_package_name}: expected exactly one static library target")
  endif()
  list(GET _rust_imported_targets 0 _rust_target)

  # corrosion_import_crate() returns the public interface target. For a
  # staticlib-only import that target forwards to Corrosion's concrete
  # <crate>-static target. Linking the interface target can place the actual
  # Rust archive after later direct dependencies such as cMsgCInterface,
  # leaving Rust's C-message references unresolved with one-pass GNU linkers.
  # Link the concrete imported archive so CMake preserves the requested
  # Rust-then-C-message archive order.
  set(_rust_link_target "${_rust_target}-static")
  if(NOT TARGET "${_rust_link_target}")
    message(FATAL_ERROR
      "Corrosion did not provide the expected static-library target "
      "${_rust_link_target}")
  endif()

  # CMake's Makefile generators can leave GNU Make jobserver descriptors in
  # MAKEFLAGS even though those descriptors are closed before Corrosion starts
  # Cargo. Do not pass that stale jobserver state into Cargo; Cargo will select
  # its own worker count instead of warning and falling back implicitly.
  set(_cargo_makeflags_env)
  if(CMAKE_GENERATOR MATCHES "Makefiles")
    set(_cargo_makeflags_env "MAKEFLAGS=")
  endif()

  corrosion_set_env_vars(
    "${_rust_target}"
    ${_cargo_makeflags_env}
    "BSK_INCLUDE_DIR=${RUST_INCLUDE_DIR}"
    "BSK_HEADER_PATH=${RUST_HEADER}"
    "BSK_INTERFACE_PATH=${RUST_INTERFACE}"
    "BSK_BINDINGS_TRIGGER_PATH=${_bindings_trigger}"
    ${RUST_CARGO_ENV}
  )
  if(RUST_CARGO_FEATURES)
    corrosion_set_features(
      "${_rust_target}"
      FEATURES ${RUST_CARGO_FEATURES}
    )
  endif()

  # Corrosion intentionally exposes a pre-build hook for generators that must
  # run before Cargo. Use it to change a build.rs-watched trigger only when a
  # generated binding has disappeared. This restores manually deleted outputs
  # without forcing code generation during an ordinary incremental build.
  set(_bindings_prebuild_target "_rust_bindings_prebuild_${RUST_TARGET}")
  add_custom_target(
    "${_bindings_prebuild_target}"
    COMMAND
      "${CMAKE_COMMAND}"
      "-DBSK_RUST_BINDINGS_TRIGGER=${_bindings_trigger}"
      "-DBSK_RUST_HEADER=${RUST_HEADER}"
      "-DBSK_RUST_INTERFACE=${RUST_INTERFACE}"
      -P "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/bskEnsureRustBindings.cmake"
    BYPRODUCTS "${_bindings_trigger}"
    VERBATIM
  )
  add_dependencies(
    "cargo-prebuild_${_rust_target}"
    "${_bindings_prebuild_target}"
  )

  # Corrosion owns the Cargo invocation and static-library byproduct, while
  # this module's build.rs owns the generated header and SWIG interface.
  # Give CMake an explicit producer rule for those two files so Make/Ninja do
  # not reject the missing generated interface before Cargo has run.
  set(_cargo_build_target "cargo-build_${_rust_target}")
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
