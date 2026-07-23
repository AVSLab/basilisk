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

# ---------------------------------------------------------------------------
# bsk_add_rust_module_sources
#
# Build a Rust crate implementing a Basilisk module (see bsk_rust_module.h)
# into a static library, and locate/generate the C header + SWIG .i file
# that go with it. This is the language-support half of Rust module
# building: "given a Cargo.toml, produce a .a + .h + .i". It has no opinion
# on what happens to those three outputs — packaging them into an
# independently distributable wheel (Python/Eigen discovery, output-dir
# wiring, etc.) is a separate concern; see bsk-sdk's bsk_add_rust_module,
# which calls this macro and then bsk_add_swig_module.
#
# architecture/rust/bsk_build's build.rs does the file codegen (parsing the crate's
# `bsk_build::module` config struct via `syn`, emitting the header and .i);
# this macro only drives `cargo build` and tells it where to put things.
#
# Required arguments
# ------------------
#   TARGET      CMake target name (also the module/symbol name bsk-build
#               generates against: New_<TARGET>, SelfInit_<TARGET>, etc.).
#   MANIFEST    Path to the crate's Cargo.toml.
#
# Optional arguments (header/interface overrides)
# -------------------------------------------------
#   HEADER      Path to the module's .h file. When omitted (recommended),
#               bsk-build generates it under CMAKE_CURRENT_BINARY_DIR.
#   INTERFACE   Hand-written .i file. When omitted (recommended), bsk-build
#               generates it under CMAKE_CURRENT_BINARY_DIR.
#
# Optional arguments
# ------------------
#   INCLUDE_DIR    Directory containing the Basilisk headers a module's own
#                  build.rs may need (e.g. for bindgen against a custom
#                  message type). Forwarded to cargo as BSK_INCLUDE_DIR.
#                  Defaults to CMAKE_SOURCE_DIR (this works unmodified when
#                  called from Basilisk's own src/CMakeLists.txt, where that
#                  already is the include root the generated headers'
#                  #include paths resolve against). Callers building outside
#                  this source tree (e.g. bsk-sdk, against vendored headers)
#                  must pass this explicitly.
#   CRATE_NAME     Override the crate lib name when it differs from TARGET.
#   CARGO_PROFILE  "release" (default) or "dev".
#   CARGO_FEATURES Cargo feature flags to enable (list).
#   CARGO_ENV      Extra environment variable assignments forwarded to cargo
#                  (e.g. "RUSTFLAGS=-C opt-level=3").
#
# Cargo lock policy
# -----------------
# The crate must have a Cargo.lock, either beside its own manifest or at its
# Cargo workspace root. The build always passes --locked so dependency drift
# is reported instead of modifying a lockfile during a CMake build.
#
# Outputs (set in the caller's scope; pass the variable *name* you want each
# written to)
# ---------------------------------------------------------------------------
#   OUT_LIB_VAR         Absolute path to the built static library.
#   OUT_HEADER_VAR      Absolute path to the module's C header.
#   OUT_INTERFACE_VAR   Absolute path to the module's SWIG .i file.
#   OUT_BUILD_TARGET_VAR  Name of the custom target that runs `cargo build`;
#                         depend on this before consuming the paths above.
#
# Example
# -------
#   bsk_add_rust_module_sources(
#     TARGET      myModule
#     MANIFEST    "${CMAKE_CURRENT_SOURCE_DIR}/myModule/Cargo.toml"
#     OUT_LIB_VAR           _lib
#     OUT_HEADER_VAR        _header
#     OUT_INTERFACE_VAR     _interface
#     OUT_BUILD_TARGET_VAR  _build_target
#   )
#   swig_add_library(myModule LANGUAGE python SOURCES "${_interface}")
#   target_link_libraries(myModule PRIVATE "${_lib}")
#   add_dependencies(myModule "${_build_target}")
# ---------------------------------------------------------------------------
function(bsk_add_rust_module_sources)
  set(_one  TARGET HEADER MANIFEST INTERFACE CRATE_NAME CARGO_PROFILE
            INCLUDE_DIR OUT_LIB_VAR OUT_HEADER_VAR OUT_INTERFACE_VAR OUT_BUILD_TARGET_VAR)
  set(_multi CARGO_FEATURES CARGO_ENV)
  cmake_parse_arguments(RUST "" "${_one}" "${_multi}" ${ARGN})

  # ------------------------------------------------------------------
  # Validate required arguments
  # ------------------------------------------------------------------
  if(NOT RUST_TARGET)
    message(FATAL_ERROR "bsk_add_rust_module_sources: TARGET is required")
  endif()
  if(NOT RUST_MANIFEST)
    message(FATAL_ERROR "bsk_add_rust_module_sources: MANIFEST (path to Cargo.toml) is required")
  endif()

  # ------------------------------------------------------------------
  # Locate cargo
  # ------------------------------------------------------------------
  find_program(CARGO_EXECUTABLE cargo)
  if(NOT CARGO_EXECUTABLE)
    message(FATAL_ERROR
      "bsk_add_rust_module_sources: 'cargo' not found.\n"
      "Install the Rust toolchain from https://rustup.rs/ and make sure "
      "'cargo' is on PATH.")
  endif()

  if(NOT RUST_INCLUDE_DIR)
    set(RUST_INCLUDE_DIR "${CMAKE_SOURCE_DIR}")
  endif()

  # ------------------------------------------------------------------
  # Resolve Cargo.toml and derive the staticlib output path
  # ------------------------------------------------------------------
  get_filename_component(_manifest "${RUST_MANIFEST}" ABSOLUTE)
  get_filename_component(_crate_dir "${_manifest}" DIRECTORY)

  execute_process(
    COMMAND "${CARGO_EXECUTABLE}" locate-project --workspace
            --manifest-path "${_manifest}" --message-format plain
    RESULT_VARIABLE _cargo_locate_result
    OUTPUT_VARIABLE _cargo_root_manifest
    ERROR_VARIABLE _cargo_locate_error
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_STRIP_TRAILING_WHITESPACE
  )
  if(NOT _cargo_locate_result EQUAL 0)
    message(FATAL_ERROR
      "bsk_add_rust_module_sources: could not locate the Cargo root for ${_manifest}.\n"
      "${_cargo_locate_error}")
  endif()
  get_filename_component(_cargo_root_dir "${_cargo_root_manifest}" DIRECTORY)
  set(_cargo_lock "${_cargo_root_dir}/Cargo.lock")
  if(NOT EXISTS "${_cargo_lock}")
    message(FATAL_ERROR
      "bsk_add_rust_module_sources: required Cargo lockfile not found: ${_cargo_lock}\n"
      "Generate and commit it with:\n"
      "  cargo generate-lockfile --manifest-path ${_cargo_root_manifest}")
  endif()

  set(_generates_header FALSE)
  if(NOT RUST_INTERFACE AND NOT RUST_HEADER)
    # bsk-build writes this Cargo byproduct outside the source tree. Keeping
    # its path predictable lets SWIG refer to it before the first cargo build.
    set(_generates_header TRUE)
    set(RUST_HEADER "${CMAKE_CURRENT_BINARY_DIR}/rust_headers/${RUST_TARGET}.h")
    message(STATUS
      "bsk_add_rust_module_sources: HEADER not provided; "
      "generating bsk-build header at: ${RUST_HEADER}")
  endif()
  set(_extra_byproducts "")
  if(_generates_header)
    list(APPEND _extra_byproducts "${RUST_HEADER}")
    set(_header_env "BSK_HEADER_PATH=${RUST_HEADER}")
  else()
    set(_header_env "")
  endif()

  # bsk-build (the crate's build.rs) writes the SWIG .i file itself -- see
  # BSK_INTERFACE_PATH in its docs -- so this only has to point SWIG at a
  # predictable path, the same way it already does for HEADER above. There
  # is no CMake-side parsing of the crate's Rust source at all: cargo build
  # produces the .i file as a build byproduct, exactly like the .h file.
  set(_interface_env "")
  if(NOT RUST_INTERFACE)
    set(_gen_i "${CMAKE_CURRENT_BINARY_DIR}/${RUST_TARGET}_rust_wrap.i")
    set(_interface_env "BSK_INTERFACE_PATH=${_gen_i}")
    list(APPEND _extra_byproducts "${_gen_i}")
  endif()
  set(_header_byproducts BYPRODUCTS ${_extra_byproducts})

  if(NOT RUST_CARGO_PROFILE)
    set(RUST_CARGO_PROFILE "release")
  endif()

  # Cargo places the output in target/debug for profile "dev".
  if(RUST_CARGO_PROFILE STREQUAL "dev")
    set(_profile_dir "debug")
    set(_profile_flag "")
  else()
    set(_profile_dir "${RUST_CARGO_PROFILE}")
    set(_profile_flag "--${RUST_CARGO_PROFILE}")
  endif()

  # Derive the lib name: use CRATE_NAME if given, otherwise read the [lib] name
  # or [package] name from Cargo.toml (Cargo converts hyphens to underscores in
  # artifact names, matching the staticlib filename).
  if(RUST_CRATE_NAME)
    set(_lib_name "${RUST_CRATE_NAME}")
  else()
    # Try [lib] name first, fall back to [package] name.
    file(READ "${_manifest}" _cargo_toml_text)
    set(_lib_name "")
    if(_cargo_toml_text MATCHES "\\[lib\\][^\n]*\n[^\[]*name[ \t]*=[ \t]*\"([^\"]+)\"")
      set(_lib_name "${CMAKE_MATCH_1}")
    endif()
    if(NOT _lib_name AND _cargo_toml_text MATCHES "\\[package\\][^\n]*\n[^\[]*name[ \t]*=[ \t]*\"([^\"]+)\"")
      set(_lib_name "${CMAKE_MATCH_1}")
    endif()
    if(NOT _lib_name)
      message(FATAL_ERROR
        "bsk_add_rust_module_sources: could not read crate name from ${_manifest}.\n"
        "Set CRATE_NAME explicitly.")
    endif()
    # Cargo replaces hyphens with underscores in staticlib filenames.
    string(REPLACE "-" "_" _lib_name "${_lib_name}")
  endif()

  set(_rust_target_dir "${CMAKE_CURRENT_BINARY_DIR}/rust_target")
  set(_rust_lib "${_rust_target_dir}/${_profile_dir}/lib${_lib_name}.a")

  # ------------------------------------------------------------------
  # Feature flags
  # ------------------------------------------------------------------
  set(_feature_args "")
  if(RUST_CARGO_FEATURES)
    list(JOIN RUST_CARGO_FEATURES "," _feat_str)
    set(_feature_args "--features" "${_feat_str}")
  endif()

  # ------------------------------------------------------------------
  # Custom command: build the Rust crate → staticlib
  #
  # Cargo handles its own incremental compilation, but CMake must first know
  # when to invoke Cargo. List the module crate, workspace metadata, and
  # in-tree Basilisk Rust support crates as explicit dependencies; source
  # changes then cause Cargo to produce a newer .a, which makes the downstream
  # link step re-run automatically.
  # ------------------------------------------------------------------
  set(_dep_files "${_manifest}" "${_cargo_root_manifest}" "${_cargo_lock}")
  # Also depend on all Rust source files in the module crate so CMake re-runs
  # Cargo for either the standard src/ layout or a Basilisk-style root source.
  file(GLOB_RECURSE _rust_sources CONFIGURE_DEPENDS "${_crate_dir}/*.rs")
  list(FILTER _rust_sources EXCLUDE REGEX "/target/")
  list(APPEND _dep_files ${_rust_sources})
  # Basilisk's in-tree workspace keeps bsk-build, bsk-macros, bsk-messages,
  # and bsk-utilities here. Without these dependencies, changing the shared
  # support layer does not rerun this output-producing custom command, so
  # CMake can continue linking a stale Rust static library. Out-of-tree
  # extension workspaces normally fetch these crates and have no such
  # directory, in which case this block is intentionally a no-op.
  set(_bsk_rust_support_dir "${_cargo_root_dir}/architecture/rust")
  if(IS_DIRECTORY "${_bsk_rust_support_dir}")
    file(GLOB_RECURSE _bsk_rust_support_sources CONFIGURE_DEPENDS
         "${_bsk_rust_support_dir}/*.rs"
         "${_bsk_rust_support_dir}/Cargo.toml")
    list(FILTER _bsk_rust_support_sources EXCLUDE REGEX "/target/")
    list(APPEND _dep_files ${_bsk_rust_support_sources})
  endif()
  # build.rs changes should also trigger a rebuild.
  if(EXISTS "${_crate_dir}/build.rs")
    list(APPEND _dep_files "${_crate_dir}/build.rs")
  endif()
  list(REMOVE_DUPLICATES _dep_files)

  add_custom_command(
    OUTPUT  "${_rust_lib}"
    ${_header_byproducts}
    COMMAND ${CMAKE_COMMAND} -E env
            "CARGO_TARGET_DIR=${_rust_target_dir}"
            "BSK_INCLUDE_DIR=${RUST_INCLUDE_DIR}"
            ${_header_env}
            ${_interface_env}
            ${RUST_CARGO_ENV}
            "${CARGO_EXECUTABLE}" build --locked ${_profile_flag} ${_feature_args}
            --manifest-path "${_manifest}"
    DEPENDS ${_dep_files}
    WORKING_DIRECTORY "${_crate_dir}"
    COMMENT "Cargo: building Rust crate for BSK module '${RUST_TARGET}'"
    VERBATIM
  )

  set(_rust_target_name "_rust_build_${RUST_TARGET}")
  add_custom_target("${_rust_target_name}" DEPENDS "${_rust_lib}")

  if(NOT RUST_INTERFACE)
    set(RUST_INTERFACE "${_gen_i}")
  endif()

  # ------------------------------------------------------------------
  # Report outputs to the caller
  # ------------------------------------------------------------------
  if(RUST_OUT_LIB_VAR)
    set("${RUST_OUT_LIB_VAR}" "${_rust_lib}" PARENT_SCOPE)
  endif()
  if(RUST_OUT_HEADER_VAR)
    set("${RUST_OUT_HEADER_VAR}" "${RUST_HEADER}" PARENT_SCOPE)
  endif()
  if(RUST_OUT_INTERFACE_VAR)
    set("${RUST_OUT_INTERFACE_VAR}" "${RUST_INTERFACE}" PARENT_SCOPE)
  endif()
  if(RUST_OUT_BUILD_TARGET_VAR)
    set("${RUST_OUT_BUILD_TARGET_VAR}" "${_rust_target_name}" PARENT_SCOPE)
  endif()
endfunction()
