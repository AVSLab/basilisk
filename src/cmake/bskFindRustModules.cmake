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
include(bskAddRustModuleSources)

function(_bsk_find_cargo OUT_VAR)
  find_program(_BSK_CARGO_EXECUTABLE NAMES cargo)
  if(NOT _BSK_CARGO_EXECUTABLE)
    message(FATAL_ERROR
      "BUILD_RUST_MODULES is ON but no 'cargo' executable was found on PATH. "
      "Install a supported stable Rust toolchain from https://rustup.rs/ and "
      "make sure 'cargo' is on PATH, or configure with BUILD_RUST_MODULES=OFF.")
  endif()
  set("${OUT_VAR}" "${_BSK_CARGO_EXECUTABLE}" PARENT_SCOPE)
endfunction()

# Ask Cargo for the typed workspace/package model once per discovery manifest
# and configure. This avoids duplicating Cargo.toml syntax, workspace
# inheritance, or metadata semantics in CMake, while allowing an integrated
# external-module project to keep its own Cargo workspace and lockfile.
function(_bsk_rust_workspace_metadata WORKSPACE_MANIFEST OUT_VAR)
  get_filename_component(
    _workspace_manifest "${WORKSPACE_MANIFEST}" ABSOLUTE)
  if(NOT EXISTS "${_workspace_manifest}")
    message(FATAL_ERROR
      "Rust module discovery requires the Cargo manifest: "
      "${_workspace_manifest}")
  endif()
  file(REAL_PATH "${_workspace_manifest}" _workspace_manifest)
  string(MD5 _metadata_cache_key "${_workspace_manifest}")
  set(_metadata_property
      "BSK_RUST_WORKSPACE_METADATA_${_metadata_cache_key}")

  get_property(
    _metadata_cached
    GLOBAL PROPERTY "${_metadata_property}"
    SET)
  if(_metadata_cached)
    get_property(
      _metadata_json
      GLOBAL PROPERTY "${_metadata_property}")
    set("${OUT_VAR}" "${_metadata_json}" PARENT_SCOPE)
    return()
  endif()

  _bsk_find_cargo(_cargo_executable)
  get_filename_component(_workspace_directory "${_workspace_manifest}" DIRECTORY)
  set(_workspace_lockfile "${_workspace_directory}/Cargo.lock")
  set_property(
    DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${_workspace_manifest}"
    "${_workspace_lockfile}")

  execute_process(
    COMMAND
      "${_cargo_executable}" metadata
      --locked
      --no-deps
      --format-version 1
      --manifest-path "${_workspace_manifest}"
    RESULT_VARIABLE _metadata_result
    OUTPUT_VARIABLE _metadata_json
    ERROR_VARIABLE _metadata_error
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_STRIP_TRAILING_WHITESPACE)
  if(NOT _metadata_result EQUAL 0)
    message(FATAL_ERROR
      "Cargo metadata failed during Rust module discovery:\n"
      "${_metadata_error}")
  endif()

  string(
    JSON _package_count
    ERROR_VARIABLE _metadata_json_error
    LENGTH "${_metadata_json}" packages)
  if(_metadata_json_error)
    message(FATAL_ERROR
      "Cargo returned invalid workspace metadata during Rust module discovery:\n"
      "${_metadata_json_error}")
  endif()

  # Reconfigure when an existing workspace member changes its marker. A new
  # member must also be registered in the root workspace manifest, which is
  # already watched above.
  if(_package_count GREATER 0)
    math(EXPR _last_package_index "${_package_count} - 1")
    set(_workspace_package_manifests "")
    foreach(_package_index RANGE 0 ${_last_package_index})
      string(
        JSON _package_manifest
        GET "${_metadata_json}" packages ${_package_index} manifest_path)
      list(APPEND _workspace_package_manifests "${_package_manifest}")
    endforeach()
    set_property(
      DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
      ${_workspace_package_manifests})
  endif()

  set_property(
    GLOBAL PROPERTY "${_metadata_property}" "${_metadata_json}")
  set("${OUT_VAR}" "${_metadata_json}" PARENT_SCOPE)
endfunction()

# rust-bindgen uses libclang at Cargo build time. On Linux and Windows,
# Basilisk's Python libclang package provides that native library without
# requiring another user-installed dependency. macOS instead lets clang-sys
# select Xcode's libclang so the compiler and SDK headers remain compatible.
function(_bsk_find_python_libclang OUT_VAR)
  execute_process(
    COMMAND
      "${Python3_EXECUTABLE}" -c
      "from pathlib import Path; import clang; print((Path(clang.__file__).resolve().parent / 'native').as_posix())"
    RESULT_VARIABLE _libclang_result
    OUTPUT_VARIABLE _libclang_dir
    ERROR_VARIABLE _libclang_error
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_STRIP_TRAILING_WHITESPACE
  )
  if(NOT _libclang_result EQUAL 0 OR NOT IS_DIRECTORY "${_libclang_dir}")
    message(FATAL_ERROR
      "Rust message binding generation could not locate the libclang Python "
      "package used by Basilisk.\n${_libclang_error}")
  endif()
  set("${OUT_VAR}" "${_libclang_dir}" PARENT_SCOPE)
endfunction()

# ---------------------------------------------------------------------------
# find_rust_package_targets / generate_rust_package_targets
#
# Counterpart of find_package_targets()/generate_package_targets() (see
# src/CMakeLists.txt) for modules implemented in Rust. Same discovery
# convention -- one target per module directory -- but keyed on a crate's
# Cargo metadata and an explicit `[package.metadata.basilisk] module = true`
# marker instead of a hand-written .i file. The marker distinguishes BSK
# modules from support crates such as architecture/rust/bsk_build. A Rust
# module's .i file is a `cargo build` *byproduct* (bsk-build generates it from
# the crate's source), not something committed to disk. find_package_targets()
# runs at CMake *configure* time, before any build step has run, so it can never
# see a not-yet-generated .i file -- hence the separate discovery function.
#
# generate_rust_package_targets() is a no-op unless BUILD_RUST_MODULES is ON
# (see bskTargetExcludeBuildOptions.cmake) -- Rust module support is
# experimental and this keeps a Cargo/Rust toolchain fully optional for
# everyone who isn't building one.
# ---------------------------------------------------------------------------

function(find_rust_package_targets PKG_DIR ALL_TARGET_LIST)
  if(NOT BUILD_RUST_MODULES)
    set("${ALL_TARGET_LIST}" "" PARENT_SCOPE)
    return()
  endif()

  file(REAL_PATH "${PKG_DIR}" _package_root)
  set(_discovery_manifests ${ARGN})
  if(NOT _discovery_manifests)
    set(_discovery_manifests "${CMAKE_SOURCE_DIR}/Cargo.toml")
  endif()

  set(RUST_TARGETS "")
  foreach(_discovery_manifest IN LISTS _discovery_manifests)
    _bsk_rust_workspace_metadata("${_discovery_manifest}" _metadata_json)
    string(JSON _package_count LENGTH "${_metadata_json}" packages)

    if(_package_count EQUAL 0)
      continue()
    endif()

    math(EXPR _last_package_index "${_package_count} - 1")
    foreach(_package_index RANGE 0 ${_last_package_index})
      string(
        JSON _manifest_path
        GET "${_metadata_json}" packages ${_package_index} manifest_path)
      file(REAL_PATH "${_manifest_path}" _manifest_path)
      cmake_path(
        IS_PREFIX _package_root "${_manifest_path}"
        NORMALIZE _manifest_is_in_package)
      if(NOT _manifest_is_in_package)
        continue()
      endif()

      string(
        JSON _module_metadata_type
        ERROR_VARIABLE _module_metadata_error
        TYPE "${_metadata_json}"
        packages ${_package_index} metadata basilisk module)
      if(_module_metadata_error)
        continue()
      endif()
      if(NOT _module_metadata_type STREQUAL "BOOLEAN")
        message(FATAL_ERROR
          "Rust package marker must be a Boolean: "
          "[package.metadata.basilisk] module = true in ${_manifest_path}")
      endif()
      string(
        JSON _is_basilisk_module
        GET "${_metadata_json}"
        packages ${_package_index} metadata basilisk module)
      if(_is_basilisk_module)
        file(
          RELATIVE_PATH _relative_manifest
          "${CMAKE_SOURCE_DIR}" "${_manifest_path}")
        list(APPEND RUST_TARGETS "${_relative_manifest}")
      endif()
    endforeach()
  endforeach()
  list(REMOVE_DUPLICATES RUST_TARGETS)
  list(SORT RUST_TARGETS)

  set("${ALL_TARGET_LIST}"
      ${RUST_TARGETS}
      PARENT_SCOPE)
endfunction(find_rust_package_targets)

# Discover Rust modules supplied through EXTERNAL_MODULES_PATH. Prefer one
# Cargo workspace at the external project root, but also accept independent
# packages directly under ExternalModules/<module>. Cargo remains the source
# of truth for package metadata in both layouts.
function(find_external_rust_package_targets EXTERNAL_ROOT ALL_TARGET_LIST)
  if(NOT BUILD_RUST_MODULES)
    set("${ALL_TARGET_LIST}" "" PARENT_SCOPE)
    return()
  endif()

  file(GLOB _external_rust_manifests CONFIGURE_DEPENDS
       "${EXTERNAL_ROOT}/Cargo.toml"
       "${EXTERNAL_ROOT}/ExternalModules/*/Cargo.toml")
  if(NOT _external_rust_manifests)
    set("${ALL_TARGET_LIST}" "" PARENT_SCOPE)
    return()
  endif()

  find_rust_package_targets(
    "${EXTERNAL_ROOT}/ExternalModules"
    _external_rust_targets
    ${_external_rust_manifests})
  set("${ALL_TARGET_LIST}" ${_external_rust_targets} PARENT_SCOPE)
endfunction(find_external_rust_package_targets)

function(generate_rust_package_targets TARGET_LIST LIB_DEP_LIST MODULE_DIR)
  if(NOT TARGET_LIST)
    return()
  endif()

  if(NOT BUILD_RUST_MODULES)
    return()
  endif()

  _bsk_find_cargo(_cargo_executable)

  set(_rust_cargo_env
      "BSK_CMSG_DIR=${CMAKE_BINARY_DIR}/autoSource/cMsgCInterface"
      "BSK_SRC_ROOT=${CMAKE_SOURCE_DIR}")
  if(UNIX AND NOT APPLE AND CMAKE_C_IMPLICIT_INCLUDE_DIRECTORIES)
    # The Python libclang wheels used in isolated manylinux builds provide
    # libclang itself but not a discoverable compiler resource-header tree.
    # Give rust-bindgen the same standard C include paths CMake detected for
    # the active compiler so headers such as stddef.h remain available.
    cmake_path(
      CONVERT "${CMAKE_C_IMPLICIT_INCLUDE_DIRECTORIES}"
      TO_NATIVE_PATH_LIST _rust_c_system_include_dirs)
    list(APPEND
      _rust_cargo_env
      "BSK_C_SYSTEM_INCLUDE_DIRS=${_rust_c_system_include_dirs}")
  endif()
  if(NOT APPLE)
    _bsk_find_python_libclang(_rust_libclang_dir)
    list(APPEND _rust_cargo_env "LIBCLANG_PATH=${_rust_libclang_dir}")
  endif()
  foreach(TARGET_FILE IN LISTS TARGET_LIST)
    get_filename_component(PARENT_DIR "${TARGET_FILE}" DIRECTORY)
    get_filename_component(TARGET_NAME "${PARENT_DIR}" NAME)

    if("${TARGET_NAME}" IN_LIST EXCLUDED_BSK_TARGETS)
      message("Skipped Target: ${TARGET_NAME}")
      continue()
    endif()

    if(IS_ABSOLUTE "${TARGET_FILE}")
      set(_rust_manifest "${TARGET_FILE}")
    else()
      set(_rust_manifest "${CMAKE_SOURCE_DIR}/${TARGET_FILE}")
    endif()
    bsk_add_rust_module_sources(
      TARGET      ${TARGET_NAME}
      MANIFEST    "${_rust_manifest}"
      OUT_LINK_TARGET_VAR  _rust_link_target
      OUT_HEADER_VAR       _rust_header
      OUT_INTERFACE_VAR    _rust_interface
      OUT_BUILD_TARGET_VAR _rust_build_target
      CARGO_ENV             ${_rust_cargo_env}
    )

    # Corrosion names its imported CMake library after the Cargo [lib]
    # target. Use an internal name for the SWIG target to avoid colliding
    # with that Rust target; OUTPUT_NAME below preserves the installed
    # Python extension and import name.
    set(_swig_target "_bsk_python_${TARGET_NAME}")
    set_source_files_properties("${_rust_interface}" PROPERTIES GENERATED TRUE)

    set_property(SOURCE ${_rust_interface} PROPERTY USE_TARGET_INCLUDE_DIRECTORIES TRUE)
    set_property(SOURCE ${_rust_interface} PROPERTY CPLUSPLUS ON)

    set(_out_dir "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}")
    swig_add_library(
      ${_swig_target}
      LANGUAGE "python"
      TYPE MODULE
      SOURCES ${_rust_interface}
      OUTFILE_DIR "${_out_dir}"
      OUTPUT_DIR  "${_out_dir}")
    # UseSWIG names the generated wrapper after the interface file rather than
    # the internal CMake target.
    get_filename_component(_swig_interface_stem "${_rust_interface}" NAME_WE)
    bsk_suppress_strict_warnings_for_sources(
      "${_out_dir}/${_swig_interface_stem}PYTHON_wrap.cxx")
    set_target_properties(${_swig_target} PROPERTIES OUTPUT_NAME ${TARGET_NAME})
    _bsk_add_rust_windows_exports("${_swig_target}" "${TARGET_NAME}")

    # UseSWIG does not discover files included by a generated interface.
    # Re-run SWIG when either the generated Rust header or the shared wrapper
    # template changes.
    set_property(TARGET ${_swig_target} APPEND PROPERTY SWIG_DEPENDS
      "${_rust_header}"
      "${CMAKE_SOURCE_DIR}/architecture/_GeneralModuleFiles/swig_c_wrap.i")

    # The SWIG command opens files generated by Cargo and must not race
    # build.rs on a cold build. The library-target dependency provides the
    # required ordering for Ninja and multi-config generators. UseSWIG gives
    # Makefile generators a separate target that owns the SWIG command, so
    # order that helper explicitly when CMake provides it as well.
    add_dependencies(${_swig_target} ${_rust_build_target})
    if(TARGET ${_swig_target}_swig_compilation)
      add_dependencies(${_swig_target}_swig_compilation ${_rust_build_target})
    endif()

    target_include_directories(${_swig_target} PRIVATE ${Python3_INCLUDE_DIRS})
    target_include_directories(${_swig_target} PRIVATE
      "${CMAKE_SOURCE_DIR}/architecture/_GeneralModuleFiles")

    target_link_libraries(${_swig_target} PRIVATE "${_rust_link_target}")
    foreach(LIB ${LIB_DEP_LIST})
      target_link_libraries(${_swig_target} PRIVATE ${LIB})
    endforeach()
    target_link_libraries(${_swig_target} PRIVATE ${PYTHON3_MODULE})

    # Unix Python modules may leave symbols for the interpreter to resolve
    # while loading the extension. Basilisk C-message functions, however,
    # must have been resolved from cMsgCInterface during the link. Detect
    # archive-order regressions before packaging a wheel.
    if(UNIX)
      add_custom_command(
        TARGET ${_swig_target}
        POST_BUILD
        COMMAND
          "${CMAKE_COMMAND}"
          "-DBSK_RUST_MODULE_FILE=$<TARGET_FILE:${_swig_target}>"
          "-DBSK_NM_EXECUTABLE=${CMAKE_NM}"
          -P "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/bskCheckRustModuleSymbols.cmake"
        COMMENT "Checking Rust module C-message linkage for '${TARGET_NAME}'"
        VERBATIM
      )
    endif()

    if(PY_LIMITED_API AND NOT PY_LIMITED_API STREQUAL "")
      target_compile_definitions(${_swig_target} PRIVATE "Py_LIMITED_API=${PY_LIMITED_API}")
    endif()

    if("${MODULE_DIR}" STREQUAL "ExternalModules")
      set_target_properties(${_swig_target} PROPERTIES FOLDER ${MODULE_DIR})
    else()
      set_target_properties(${_swig_target} PROPERTIES FOLDER ${PARENT_DIR})
    endif()
    foreach(_prop LIBRARY_OUTPUT_DIRECTORY RUNTIME_OUTPUT_DIRECTORY ARCHIVE_OUTPUT_DIRECTORY)
      set_target_properties(${_swig_target} PROPERTIES
        ${_prop}            "${_out_dir}"
        ${_prop}_DEBUG      "${_out_dir}"
        ${_prop}_RELEASE    "${_out_dir}")
    endforeach()
  endforeach()
endfunction(generate_rust_package_targets)
