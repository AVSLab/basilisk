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

set(_BSK_ABI_DESCRIPTOR_HEADER
    "${CMAKE_CURRENT_LIST_DIR}/../architecture/utilities/bskAbiDescriptor.h")

function(_bsk_python_string output_variable value)
  set(_escaped_value "${value}")
  string(REPLACE "\\" "\\\\" _escaped_value "${_escaped_value}")
  string(REPLACE "\"" "\\\"" _escaped_value "${_escaped_value}")
  string(REPLACE "\n" "\\n" _escaped_value "${_escaped_value}")
  string(REPLACE "\r" "\\r" _escaped_value "${_escaped_value}")
  set(${output_variable} "\"${_escaped_value}\"" PARENT_SCOPE)
endfunction()

function(_bsk_python_list output_variable)
  set(_list_literal "[")
  foreach(_value IN LISTS ARGN)
    _bsk_python_string(_quoted_value "${_value}")
    if(NOT _list_literal STREQUAL "[")
      string(APPEND _list_literal ", ")
    endif()
    string(APPEND _list_literal "${_quoted_value}")
  endforeach()
  string(APPEND _list_literal "]")
  set(${output_variable} "${_list_literal}" PARENT_SCOPE)
endfunction()

function(_bsk_command_name output_variable)
  set(_command ${ARGN})
  set(_command_name "")
  if(_command)
    list(GET _command 0 _executable)
    get_filename_component(_command_name "${_executable}" NAME)
  endif()
  set(${output_variable} "${_command_name}" PARENT_SCOPE)
endfunction()

function(_bsk_program_version output_variable executable program_name)
  set(_version "")
  if(executable)
    execute_process(
      COMMAND "${executable}" --version
      RESULT_VARIABLE _version_result
      OUTPUT_VARIABLE _version_output
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET
    )
    if(_version_result EQUAL 0
       AND _version_output MATCHES "^${program_name}[ \t]+([^ \t\r\n]+)")
      set(_version "${CMAKE_MATCH_1}")
    endif()
  endif()
  set(${output_variable} "${_version}" PARENT_SCOPE)
endfunction()

function(_bsk_rust_host_target output_variable rustc_executable)
  set(_host_target "")
  if(rustc_executable)
    execute_process(
      COMMAND "${rustc_executable}" --version --verbose
      RESULT_VARIABLE _version_result
      OUTPUT_VARIABLE _version_output
      ERROR_QUIET
    )
    if(_version_result EQUAL 0
       AND _version_output MATCHES "(^|\n)host:[ \t]+([^\r\n]+)")
      set(_host_target "${CMAKE_MATCH_2}")
    endif()
  endif()
  set(${output_variable} "${_host_target}" PARENT_SCOPE)
endfunction()

function(_bsk_git_metadata revision_variable dirty_variable)
  set(_source_revision "")
  set(_source_dirty None)
  find_package(Git QUIET)
  if(GIT_EXECUTABLE)
    execute_process(
      COMMAND "${GIT_EXECUTABLE}" rev-parse HEAD
      WORKING_DIRECTORY "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../.."
      RESULT_VARIABLE _revision_result
      OUTPUT_VARIABLE _source_revision
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET
    )
    if(_revision_result EQUAL 0)
      execute_process(
        COMMAND "${GIT_EXECUTABLE}" status --porcelain --untracked-files=normal
        WORKING_DIRECTORY "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../.."
        RESULT_VARIABLE _status_result
        OUTPUT_VARIABLE _source_status
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
      )
      if(_status_result EQUAL 0)
        if(_source_status)
          set(_source_dirty True)
        else()
          set(_source_dirty False)
        endif()
      endif()
    else()
      set(_source_revision "")
    endif()
  endif()
  set(${revision_variable} "${_source_revision}" PARENT_SCOPE)
  set(${dirty_variable} "${_source_dirty}" PARENT_SCOPE)
endfunction()

function(_bsk_swig_runtime_version output_variable)
  set(_runtime_version "")
  if(EXISTS "${SWIG_DIR}/swigrun.swg")
    file(STRINGS "${SWIG_DIR}/swigrun.swg" _swig_runtime_lines REGEX "SWIG_RUNTIME_VERSION")
    foreach(_line IN LISTS _swig_runtime_lines)
      if(_line MATCHES "#define[ \t]+SWIG_RUNTIME_VERSION[ \t]+\"([0-9]+)\"")
        set(_runtime_version "${CMAKE_MATCH_1}")
        break()
      endif()
    endforeach()
  endif()
  set(${output_variable} "${_runtime_version}" PARENT_SCOPE)
endfunction()

function(_bsk_read_integer_define output_variable define_name)
  file(STRINGS
       "${_BSK_ABI_DESCRIPTOR_HEADER}"
       _matching_defines
       REGEX "^#define[ \t]+${define_name}[ \t]+[0-9]+")
  list(LENGTH _matching_defines _define_count)
  if(NOT _define_count EQUAL 1)
    message(FATAL_ERROR "Expected exactly one ${define_name} in ${_BSK_ABI_DESCRIPTOR_HEADER}")
  endif()
  list(GET _matching_defines 0 _define)
  string(REGEX REPLACE ".*[ \t]+([0-9]+)$" "\\1" _value "${_define}")
  set(${output_variable} "${_value}" PARENT_SCOPE)
endfunction()

function(_bsk_wire_build_info_targets directory)
  get_property(_targets DIRECTORY "${directory}" PROPERTY BUILDSYSTEM_TARGETS)
  foreach(_target IN LISTS _targets)
    if(_target STREQUAL "bskBuildInfoProbe" OR _target STREQUAL "bskExtensionAbiSettings")
      continue()
    endif()
    get_target_property(_imported "${_target}" IMPORTED)
    get_target_property(_target_type "${_target}" TYPE)
    if(NOT _imported
       AND _target_type MATCHES "^(MODULE_LIBRARY|SHARED_LIBRARY|STATIC_LIBRARY|OBJECT_LIBRARY)$")
      set_property(TARGET "${_target}" APPEND PROPERTY LINK_LIBRARIES bskExtensionAbiSettings)
    endif()
    # Keep this dependency on loadable modules. Adding a configuration-sensitive
    # utility dependency to static libraries makes multi-config generators attach
    # every configuration to shared generated sources such as C message wrappers.
    if(NOT _imported AND _target_type STREQUAL "MODULE_LIBRARY")
      add_dependencies("${_target}" bskGenerateBuildInfo)
    endif()
  endforeach()

  get_property(_subdirectories DIRECTORY "${directory}" PROPERTY SUBDIRECTORIES)
  foreach(_subdirectory IN LISTS _subdirectories)
    string(FIND "${_subdirectory}" "${CMAKE_SOURCE_DIR}/" _source_prefix)
    if(_source_prefix EQUAL 0)
      _bsk_wire_build_info_targets("${_subdirectory}")
    endif()
  endforeach()
endfunction()

function(bsk_generate_build_info package_directory)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_BSK_ABI_DESCRIPTOR_HEADER}")
  _bsk_command_name(_c_compiler_executable "${CMAKE_C_COMPILER}")
  _bsk_command_name(_cxx_compiler_executable "${CMAKE_CXX_COMPILER}")
  _bsk_command_name(_c_compiler_launcher ${CMAKE_C_COMPILER_LAUNCHER})
  _bsk_command_name(_cxx_compiler_launcher ${CMAKE_CXX_COMPILER_LAUNCHER})

  if(CMAKE_CONFIGURATION_TYPES)
    set(_multi_config True)
    set(_build_configuration "")
    _bsk_python_list(BSK_INFO_AVAILABLE_CONFIGURATIONS ${CMAKE_CONFIGURATION_TYPES})
  else()
    set(_multi_config False)
    set(_build_configuration "${CMAKE_BUILD_TYPE}")
    _bsk_python_list(BSK_INFO_AVAILABLE_CONFIGURATIONS)
  endif()

  _bsk_python_string(BSK_INFO_SYSTEM "${CMAKE_SYSTEM_NAME}")
  _bsk_python_string(BSK_INFO_PROCESSOR "${CMAKE_SYSTEM_PROCESSOR}")
  _bsk_python_string(BSK_INFO_OSX_ARCHITECTURES "${CMAKE_OSX_ARCHITECTURES}")
  _bsk_python_string(BSK_INFO_OSX_DEPLOYMENT_TARGET "${CMAKE_OSX_DEPLOYMENT_TARGET}")

  _bsk_python_string(BSK_INFO_CONFIGURATION "${_build_configuration}")
  _bsk_python_string(BSK_INFO_GENERATOR "${CMAKE_GENERATOR}")
  _bsk_python_string(BSK_INFO_GENERATOR_PLATFORM "${CMAKE_GENERATOR_PLATFORM}")
  _bsk_python_string(BSK_INFO_GENERATOR_TOOLSET "${CMAKE_GENERATOR_TOOLSET}")
  _bsk_python_string(BSK_INFO_PY_LIMITED_API "${PY_LIMITED_API}")

  _bsk_python_string(BSK_INFO_C_COMPILER_ID "${CMAKE_C_COMPILER_ID}")
  _bsk_python_string(BSK_INFO_C_COMPILER_VERSION "${CMAKE_C_COMPILER_VERSION}")
  _bsk_python_string(BSK_INFO_C_COMPILER_EXECUTABLE "${_c_compiler_executable}")
  _bsk_python_string(BSK_INFO_C_COMPILER_LAUNCHER "${_c_compiler_launcher}")
  _bsk_python_string(BSK_INFO_C_FRONTEND_VARIANT "${CMAKE_C_COMPILER_FRONTEND_VARIANT}")
  _bsk_python_string(BSK_INFO_C_SIMULATE_ID "${CMAKE_C_SIMULATE_ID}")
  _bsk_python_string(BSK_INFO_C_SIMULATE_VERSION "${CMAKE_C_SIMULATE_VERSION}")
  _bsk_python_string(BSK_INFO_C_COMPILER_TARGET "${CMAKE_C_COMPILER_TARGET}")
  _bsk_python_string(BSK_INFO_C_STANDARD "${CMAKE_C_STANDARD}")
  _bsk_python_string(BSK_INFO_C_EXTENSIONS "${CMAKE_C_EXTENSIONS}")

  _bsk_python_string(BSK_INFO_CXX_COMPILER_ID "${CMAKE_CXX_COMPILER_ID}")
  _bsk_python_string(BSK_INFO_CXX_COMPILER_VERSION "${CMAKE_CXX_COMPILER_VERSION}")
  _bsk_python_string(BSK_INFO_CXX_COMPILER_EXECUTABLE "${_cxx_compiler_executable}")
  _bsk_python_string(BSK_INFO_CXX_COMPILER_LAUNCHER "${_cxx_compiler_launcher}")
  _bsk_python_string(BSK_INFO_CXX_FRONTEND_VARIANT "${CMAKE_CXX_COMPILER_FRONTEND_VARIANT}")
  _bsk_python_string(BSK_INFO_CXX_SIMULATE_ID "${CMAKE_CXX_SIMULATE_ID}")
  _bsk_python_string(BSK_INFO_CXX_SIMULATE_VERSION "${CMAKE_CXX_SIMULATE_VERSION}")
  _bsk_python_string(BSK_INFO_CXX_COMPILER_TARGET "${CMAKE_CXX_COMPILER_TARGET}")
  _bsk_python_string(BSK_INFO_CXX_STANDARD "${CMAKE_CXX_STANDARD}")
  _bsk_python_string(BSK_INFO_CXX_EXTENSIONS "${CMAKE_CXX_EXTENSIONS}")

  _bsk_python_string(BSK_INFO_CONAN_BUILD_TYPE "${BSK_CONAN_BUILD_TYPE}")
  _bsk_python_string(BSK_INFO_CONAN_CXX_STANDARD "${BSK_CONAN_CXX_STANDARD}")
  _bsk_python_string(BSK_INFO_CONAN_CXX_STANDARD_LIBRARY "${BSK_CONAN_CXX_STANDARD_LIBRARY}")
  _bsk_python_string(BSK_INFO_CONAN_COMPILER_RUNTIME "${BSK_CONAN_COMPILER_RUNTIME}")
  _bsk_python_string(BSK_INFO_CONAN_COMPILER_RUNTIME_TYPE "${BSK_CONAN_COMPILER_RUNTIME_TYPE}")

  _bsk_python_string(BSK_INFO_CMAKE_VERSION "${CMAKE_VERSION}")
  _bsk_python_string(BSK_INFO_CONAN_VERSION "${BSK_CONAN_VERSION}")
  _bsk_python_string(BSK_INFO_SWIG_VERSION "${SWIG_VERSION}")
  _bsk_python_string(BSK_INFO_PYTHON_VERSION "${Python3_VERSION}")

  set(_rust_modules False)
  set(_rustc_id "")
  set(_rustc_executable "")
  set(_rustc_version "")
  set(_rust_target "")
  set(_cargo_executable "")
  set(_cargo_version "")
  set(_corrosion_version "")
  if(BUILD_RUST_MODULES)
    set(_rust_modules True)
    set(_rustc_id "rustc")
    set(_rustc_executable "${Rust_COMPILER_CACHED}")
    set(_rustc_version "${Rust_VERSION}")
    set(_rust_target "${Rust_CARGO_TARGET_CACHED}")
    set(_cargo_executable "${Rust_CARGO_CACHED}")
    set(_cargo_version "${Rust_CARGO_VERSION}")
    if(Corrosion_VERSION)
      set(_corrosion_version "${Corrosion_VERSION}")
    else()
      set(_corrosion_version "${BSK_CORROSION_VERSION}")
    endif()

    if(NOT _rustc_executable)
      find_program(_rustc_executable NAMES rustc NO_CACHE)
    endif()
    if(NOT _cargo_executable)
      set(_cargo_executable "${CARGO_EXECUTABLE}")
    endif()
    if(NOT _cargo_executable)
      find_program(_cargo_executable NAMES cargo NO_CACHE)
    endif()
    if(NOT _rustc_version)
      _bsk_program_version(_rustc_version "${_rustc_executable}" rustc)
    endif()
    if(NOT _cargo_version)
      _bsk_program_version(_cargo_version "${_cargo_executable}" cargo)
    endif()
    if(NOT _rust_target)
      _bsk_rust_host_target(_rust_target "${_rustc_executable}")
    endif()
  endif()

  _bsk_command_name(_rustc_command "${_rustc_executable}")
  _bsk_python_string(BSK_INFO_RUSTC_ID "${_rustc_id}")
  _bsk_python_string(BSK_INFO_RUSTC_VERSION "${_rustc_version}")
  _bsk_python_string(BSK_INFO_RUSTC_EXECUTABLE "${_rustc_command}")
  _bsk_python_string(BSK_INFO_RUSTC_TARGET "${_rust_target}")
  _bsk_python_string(BSK_INFO_CARGO_VERSION "${_cargo_version}")
  _bsk_python_string(BSK_INFO_CORROSION_VERSION "${_corrosion_version}")

  if(BSK_VERSION)
    set(_basilisk_version "${BSK_VERSION}")
  else()
    file(STRINGS "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../../docs/source/bskVersion.txt" _basilisk_version LIMIT_COUNT 1)
  endif()
  _bsk_git_metadata(_source_revision _source_dirty)
  _bsk_read_integer_define(_extension_abi_version BSK_EXTENSION_ABI_VERSION)
  _bsk_python_string(BSK_INFO_BASILISK_VERSION "${_basilisk_version}")
  _bsk_python_string(BSK_INFO_SOURCE_REVISION "${_source_revision}")
  set(BSK_INFO_SOURCE_DIRTY "${_source_dirty}")
  set(BSK_INFO_EXTENSION_ABI_VERSION "${_extension_abi_version}")

  configure_file(
    "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/bskBuildInfoData.py.in"
    "${package_directory}/_buildInfoData.py"
    @ONLY
  )
  configure_file(
    "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/bskBuildInfo.py"
    "${package_directory}/_buildInfo.py"
    COPYONLY
  )

  _bsk_swig_runtime_version(_swig_runtime_version)
  # Keep ABI-affecting target properties here so Basilisk modules and the SDK
  # descriptor probe consume the same build requirements.
  add_library(bskExtensionAbiSettings INTERFACE)
  target_compile_features(bskExtensionAbiSettings INTERFACE cxx_std_17)
  target_include_directories(
    bskExtensionAbiSettings
    INTERFACE
      "${CMAKE_SOURCE_DIR}"
      "${Python3_INCLUDE_DIRS}"
      "${Eigen3_INCLUDE_DIRS}"
  )
  target_link_libraries(bskExtensionAbiSettings INTERFACE Eigen3::Eigen3)
  if(PY_LIMITED_API AND NOT PY_LIMITED_API STREQUAL "")
    target_compile_definitions(bskExtensionAbiSettings INTERFACE "Py_LIMITED_API=${PY_LIMITED_API}")
  endif()

  add_executable(
    bskBuildInfoProbe
    "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/bskBuildInfoProbe.c"
    "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/bskBuildInfoProbe.cpp"
    "${_BSK_ABI_DESCRIPTOR_HEADER}"
  )
  target_link_libraries(bskBuildInfoProbe PRIVATE bskExtensionAbiSettings ${PYTHON3_MODULE})
  target_compile_definitions(
    bskBuildInfoProbe
    PRIVATE
      "BSK_BUILD_CONFIGURATION=\"$<CONFIG>\""
      "BSK_PYTHON_SOABI=\"${Python3_SOABI}\""
      "BSK_SWIG_RUNTIME_VERSION=\"${_swig_runtime_version}\""
  )
  set_target_properties(bskBuildInfoProbe PROPERTIES FOLDER "Build Support")

  set(_abi_data_file "${package_directory}/_buildAbiData.py")
  # A custom target intentionally runs for every selected configuration. A
  # shared output timestamp cannot tell Visual Studio Debug and Release apart.
  add_custom_target(
    bskGenerateBuildInfo ALL
    COMMAND bskBuildInfoProbe "${_abi_data_file}"
    DEPENDS bskBuildInfoProbe
    BYPRODUCTS "${_abi_data_file}"
    COMMENT "Recording compiled Basilisk ABI metadata"
    VERBATIM
  )
  set_target_properties(bskGenerateBuildInfo PROPERTIES FOLDER "Build Support")

  _bsk_wire_build_info_targets("${CMAKE_SOURCE_DIR}")

  if(CMAKE_CONFIGURATION_TYPES)
    set(_configuration_description "multi-config")
    if(BSK_CONAN_BUILD_TYPE)
      string(APPEND _configuration_description " (Conan profile: ${BSK_CONAN_BUILD_TYPE})")
    endif()
  else()
    set(_configuration_description "${CMAKE_BUILD_TYPE}")
  endif()

  message(STATUS "Basilisk build toolchain:")
  message(STATUS "  C compiler: ${CMAKE_C_COMPILER_ID} ${CMAKE_C_COMPILER_VERSION} (${CMAKE_C_COMPILER})")
  message(STATUS "  C++ compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION} (${CMAKE_CXX_COMPILER})")
  message(STATUS "  Generator: ${CMAKE_GENERATOR}")
  message(STATUS "  Configuration: ${_configuration_description}")
endfunction()
