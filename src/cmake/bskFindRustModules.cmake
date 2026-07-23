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

  foreach(TARGET_FILE ${TARGET_LIST})
    get_filename_component(PARENT_DIR ${TARGET_FILE} DIRECTORY)
    get_filename_component(TARGET_NAME ${PARENT_DIR} NAME)

    if(${TARGET_NAME} IN_LIST EXCLUDED_BSK_TARGETS)
      message("Skipped Target: ${TARGET_NAME}")
      continue()
    endif()

    bsk_add_rust_module_sources(
      TARGET      ${TARGET_NAME}
      MANIFEST    "${CMAKE_SOURCE_DIR}/${TARGET_FILE}"
      OUT_LIB_VAR          _rust_lib
      OUT_HEADER_VAR       _rust_header
      OUT_INTERFACE_VAR    _rust_interface
      OUT_BUILD_TARGET_VAR _rust_build_target
    )

    set_property(SOURCE ${_rust_interface} PROPERTY USE_TARGET_INCLUDE_DIRECTORIES TRUE)
    set_property(SOURCE ${_rust_interface} PROPERTY CPLUSPLUS ON)

    set(_out_dir "${CMAKE_BINARY_DIR}/Basilisk/${MODULE_DIR}")
    swig_add_library(
      ${TARGET_NAME}
      LANGUAGE "python"
      TYPE MODULE
      SOURCES ${_rust_interface}
      OUTFILE_DIR "${_out_dir}"
      OUTPUT_DIR  "${_out_dir}")

    # UseSWIG does not discover files included by a generated interface.
    # Re-run SWIG when the shared Rust wrapper template changes.
    set_property(TARGET ${TARGET_NAME} APPEND PROPERTY SWIG_DEPENDS
      "${CMAKE_SOURCE_DIR}/architecture/_GeneralModuleFiles/swig_c_wrap.i")

    # The SWIG compile step (which opens the header the .i %include-s) runs
    # before link and can race the Cargo build that generates that header on
    # a cold build; UseSWIG names this intermediate target
    # ${TARGET_NAME}_swig_compilation.
    add_dependencies(${TARGET_NAME}_swig_compilation ${_rust_build_target})

    target_include_directories(${TARGET_NAME} PRIVATE ${Python3_INCLUDE_DIRS})
    target_include_directories(${TARGET_NAME} PRIVATE
      "${CMAKE_SOURCE_DIR}/architecture/_GeneralModuleFiles")

    target_link_libraries(${TARGET_NAME} PRIVATE "${_rust_lib}")
    foreach(LIB ${LIB_DEP_LIST})
      target_link_libraries(${TARGET_NAME} PRIVATE ${LIB})
    endforeach()
    target_link_libraries(${TARGET_NAME} PRIVATE ${PYTHON3_MODULE})
    if(PY_LIMITED_API AND NOT PY_LIMITED_API STREQUAL "")
      target_compile_definitions(${TARGET_NAME} PRIVATE "Py_LIMITED_API=${PY_LIMITED_API}")
    endif()

    set_target_properties(${TARGET_NAME} PROPERTIES FOLDER ${PARENT_DIR})
    foreach(_prop LIBRARY_OUTPUT_DIRECTORY RUNTIME_OUTPUT_DIRECTORY ARCHIVE_OUTPUT_DIRECTORY)
      set_target_properties(${TARGET_NAME} PROPERTIES
        ${_prop}            "${_out_dir}"
        ${_prop}_DEBUG      "${_out_dir}"
        ${_prop}_RELEASE    "${_out_dir}")
    endforeach()
  endforeach()
endfunction(generate_rust_package_targets)
