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
include(bskAddRustModuleSourcesCorrosion)

# Windows does not automatically export functions pulled into a DLL from a
# static Rust library. Add the Rust/C boundary explicitly so low-level ABI
# consumers see the same symbols as they do on Linux and macOS. The generated
# module-definition file remains in the build tree; Rust module authors do not
# need to provide or maintain one.
function(_bsk_add_rust_windows_exports SWIG_TARGET MODULE_NAME)
  if(NOT WIN32)
    return()
  endif()

  set(_rust_exports
      "Create_${MODULE_NAME}"
      "Config_${MODULE_NAME}"
      "Destroy_${MODULE_NAME}"
      "SelfInit_${MODULE_NAME}"
      "Reset_${MODULE_NAME}"
      "Update_${MODULE_NAME}"
      "BskRustError_kind"
      "BskRustError_message"
      "Destroy_BskRustError")
  list(JOIN _rust_exports "\n    " _rust_export_lines)

  set(_rust_export_file
      "${CMAKE_CURRENT_BINARY_DIR}/rust_exports/${MODULE_NAME}.def")
  file(GENERATE
       OUTPUT "${_rust_export_file}"
       CONTENT "EXPORTS\n    ${_rust_export_lines}\n")
  set_source_files_properties("${_rust_export_file}" PROPERTIES GENERATED TRUE)
  target_sources("${SWIG_TARGET}" PRIVATE "${_rust_export_file}")
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
# Cargo.toml and an explicit `[package.metadata.basilisk] module = true`
# marker instead of a hand-written .i file. The marker distinguishes BSK
# modules from support crates such as architecture/rust/bsk_build. A Rust
# module's .i file is a `cargo build` *byproduct* (bsk-build generates it from
# the crate's source), not something committed to disk. find_package_targets()'s
# glob runs at CMake *configure* time, before any build step has run, so it
# can never see a not-yet-generated .i file -- hence the separate discovery
# function here instead of teaching the existing one to also match Cargo.toml.
#
# generate_rust_package_targets() is a no-op unless BUILD_RUST_MODULES is ON
# (see bskTargetExcludeBuildOptions.cmake) -- Rust module support is
# experimental and this keeps a Cargo/Rust toolchain fully optional for
# everyone who isn't building one.
# ---------------------------------------------------------------------------

function(find_rust_package_targets PKG_DIR ALL_TARGET_LIST)
  file(
    GLOB_RECURSE RUST_MANIFESTS
    CONFIGURE_DEPENDS
    RELATIVE ${CMAKE_SOURCE_DIR}
    "${PKG_DIR}/Cargo.toml")

  set(RUST_TARGETS "")
  foreach(RUST_MANIFEST IN LISTS RUST_MANIFESTS)
    if(RUST_MANIFEST MATCHES "(^|/)target(/|$)")
      continue()
    endif()
    file(STRINGS "${CMAKE_SOURCE_DIR}/${RUST_MANIFEST}" CARGO_MANIFEST_LINES)
    set(IN_BASILISK_METADATA FALSE)
    foreach(CARGO_LINE IN LISTS CARGO_MANIFEST_LINES)
      string(STRIP "${CARGO_LINE}" CARGO_LINE)
      if(CARGO_LINE MATCHES "^\\[package\\.metadata\\.basilisk\\][ \\t]*(#.*)?$")
        set(IN_BASILISK_METADATA TRUE)
      elseif(CARGO_LINE MATCHES "^\\[.*\\]")
        set(IN_BASILISK_METADATA FALSE)
      elseif(IN_BASILISK_METADATA AND
             CARGO_LINE MATCHES "^module[ \\t]*=[ \\t]*true([ \\t]*(#.*)?)?$")
        list(APPEND RUST_TARGETS "${RUST_MANIFEST}")
        break()
      endif()
    endforeach()
  endforeach()

  set(${ALL_TARGET_LIST}
      ${RUST_TARGETS}
      PARENT_SCOPE)
endfunction(find_rust_package_targets)

function(generate_rust_package_targets TARGET_LIST LIB_DEP_LIST MODULE_DIR)
  if(NOT TARGET_LIST)
    return()
  endif()

  if(NOT BUILD_RUST_MODULES)
    return()
  endif()

  find_program(CARGO_EXECUTABLE cargo)
  if(NOT CARGO_EXECUTABLE)
    message(FATAL_ERROR
      "BUILD_RUST_MODULES is ON but no 'cargo' executable was found on PATH; "
      "the explicitly requested Rust module(s) under ${MODULE_DIR} cannot be built: "
      "${TARGET_LIST}. Install Rust 1.85 or newer from https://rustup.rs/ and "
      "make sure 'cargo' is on PATH, or configure with BUILD_RUST_MODULES=OFF.")
  endif()

  set(_rust_cargo_env
      "BSK_CMSG_DIR=${CMAKE_BINARY_DIR}/autoSource/cMsgCInterface"
      "BSK_SRC_ROOT=${CMAKE_SOURCE_DIR}")
  if(NOT APPLE)
    _bsk_find_python_libclang(_rust_libclang_dir)
    list(APPEND _rust_cargo_env "LIBCLANG_PATH=${_rust_libclang_dir}")
  endif()
  file(
    GLOB _rust_message_headers
    CONFIGURE_DEPENDS
    "${CMAKE_BINARY_DIR}/autoSource/cMsgCInterface/*_C.h")

  foreach(TARGET_FILE ${TARGET_LIST})
    get_filename_component(PARENT_DIR ${TARGET_FILE} DIRECTORY)
    get_filename_component(TARGET_NAME ${PARENT_DIR} NAME)

    if(${TARGET_NAME} IN_LIST EXCLUDED_BSK_TARGETS)
      message("Skipped Target: ${TARGET_NAME}")
      continue()
    endif()

    set(_rust_manifest "${CMAKE_SOURCE_DIR}/${TARGET_FILE}")
    set(_swig_target "${TARGET_NAME}")
    if(BSK_RUST_USE_CORROSION)
      bsk_add_rust_module_sources_corrosion(
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
    else()
      bsk_add_rust_module_sources(
        TARGET      ${TARGET_NAME}
        MANIFEST    "${_rust_manifest}"
        OUT_LIB_VAR          _rust_link_target
        OUT_HEADER_VAR       _rust_header
        OUT_INTERFACE_VAR    _rust_interface
        OUT_BUILD_TARGET_VAR _rust_build_target
        CARGO_ENV             ${_rust_cargo_env}
        CARGO_DEPENDS         ${_rust_message_headers}
      )
    endif()

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
    if(BSK_RUST_USE_CORROSION)
      set_target_properties(${_swig_target} PROPERTIES OUTPUT_NAME ${TARGET_NAME})
    endif()
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
    if(PY_LIMITED_API AND NOT PY_LIMITED_API STREQUAL "")
      target_compile_definitions(${_swig_target} PRIVATE "Py_LIMITED_API=${PY_LIMITED_API}")
    endif()

    set_target_properties(${_swig_target} PROPERTIES FOLDER ${PARENT_DIR})
    foreach(_prop LIBRARY_OUTPUT_DIRECTORY RUNTIME_OUTPUT_DIRECTORY ARCHIVE_OUTPUT_DIRECTORY)
      set_target_properties(${_swig_target} PROPERTIES
        ${_prop}            "${_out_dir}"
        ${_prop}_DEBUG      "${_out_dir}"
        ${_prop}_RELEASE    "${_out_dir}")
    endforeach()
  endforeach()
endfunction(generate_rust_package_targets)
