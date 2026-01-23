include_guard(GLOBAL)
include(CMakeParseArguments)

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/bsk_add_swig_module.cmake")
  include("${CMAKE_CURRENT_LIST_DIR}/bsk_add_swig_module.cmake")
endif()

function(_bsk_resolve_msg_autosource_dir out_var)
  if(DEFINED BSK_SDK_MSG_AUTOSOURCE_DIR)
    if(EXISTS "${BSK_SDK_MSG_AUTOSOURCE_DIR}/generateSWIGModules.py")
      set(${out_var} "${BSK_SDK_MSG_AUTOSOURCE_DIR}" PARENT_SCOPE)
      return()
    endif()
  endif()

  if(DEFINED bsk-sdk_DIR AND EXISTS "${bsk-sdk_DIR}")
    set(_cmake_dir "${bsk-sdk_DIR}")
  else()
    set(_cmake_dir "${CMAKE_CURRENT_FUNCTION_LIST_DIR}")
  endif()

  get_filename_component(_pkg_root "${_cmake_dir}/../../.." REALPATH)

  set(_cand_installed "${_pkg_root}/tools/msgAutoSource")
  set(_cand_source "${_cmake_dir}/../tools/msgAutoSource")
  set(_candidates
    "${_cand_installed}"
    "${_cand_source}"
  )

  foreach(_cand IN LISTS _candidates)
    if(EXISTS "${_cand}/generateSWIGModules.py")
      set(${out_var} "${_cand}" PARENT_SCOPE)
      return()
    endif()
  endforeach()

  string(JOIN "\n  " _cand_list ${_candidates})
  message(FATAL_ERROR
    "bsk-sdk message generator not found.\n\n"
    "Looked for generateSWIGModules.py under:\n"
    "  ${_cand_list}\n\n"
    "Resolver context:\n"
    "  bsk-sdk_DIR = ${bsk-sdk_DIR}\n"
    "  CMAKE_CURRENT_FUNCTION_LIST_DIR = ${CMAKE_CURRENT_FUNCTION_LIST_DIR}\n"
    "  (note: caller CMAKE_CURRENT_LIST_DIR is irrelevant here)\n\n"
    "If using an installed wheel, ensure it installs:\n"
    "  bsk_sdk/tools/msgAutoSource/generateSWIGModules.py\n"
    "If building from source, ensure this exists:\n"
    "  sdk/tools/msgAutoSource/generateSWIGModules.py\n"
    "Or set BSK_SDK_MSG_AUTOSOURCE_DIR explicitly."
  )
endfunction()

# Main
function(bsk_generate_messages)
  set(options GENERATE_C_INTERFACE)
  set(oneValueArgs OUTPUT_DIR OUT_VAR)
  set(multiValueArgs MSG_HEADERS INCLUDE_DIRS SWIG_INCLUDE_DIRS TARGET_LINK_LIBS)
  cmake_parse_arguments(BSK "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(NOT BSK_MSG_HEADERS)
    message(FATAL_ERROR "bsk_generate_messages requires MSG_HEADERS")
  endif()

  if(NOT BSK_OUTPUT_DIR)
    set(BSK_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}")
  endif()

  find_package(SWIG REQUIRED COMPONENTS python)
  include(${SWIG_USE_FILE})
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module NumPy)
  find_package(Eigen3 CONFIG REQUIRED)

  if(NOT BSK_TARGET_LINK_LIBS)
    _bsk_resolve_basilisk_libs(_bsk_libs)
    set(BSK_TARGET_LINK_LIBS "${_bsk_libs}")
  endif()


  _bsk_collect_swig_flags(_swig_flags)

  _bsk_resolve_msg_autosource_dir(_msg_autosrc)
  set(_gen_swig "${_msg_autosrc}/generateSWIGModules.py")

  set(_auto_root "${CMAKE_CURRENT_BINARY_DIR}/autoSource")
  set(_xml_dir "${_auto_root}/xmlWrap")
  file(MAKE_DIRECTORY "${_xml_dir}")

  set(_generated_targets "")
  set(_gen_c "False")
  if(BSK_GENERATE_C_INTERFACE)
    set(_gen_c "True")
  endif()

  foreach(_hdr IN LISTS BSK_MSG_HEADERS)
    get_filename_component(_hdr_abs "${_hdr}" ABSOLUTE)
    get_filename_component(_payload_name "${_hdr_abs}" NAME_WE)
    get_filename_component(_hdr_dir "${_hdr_abs}" DIRECTORY)

    set(_xml_out "${_xml_dir}/${_payload_name}.xml")
    add_custom_command(
      OUTPUT "${_xml_out}"
      COMMAND ${SWIG_EXECUTABLE} -c++ -xml -module dummy -o "${_xml_out}" "${_hdr_abs}"
      DEPENDS "${_hdr_abs}"
      COMMENT "Generating SWIG XML for ${_payload_name}"
      VERBATIM
    )

    set(_i_out "${_auto_root}/${_payload_name}.i")
    add_custom_command(
      OUTPUT "${_i_out}"
      COMMAND ${Python3_EXECUTABLE}
              "${_gen_swig}"
              "${_i_out}" "${_hdr_abs}" "${_payload_name}" "${_hdr_dir}"
              "${_gen_c}"
              "${_xml_out}" 0
      DEPENDS "${_hdr_abs}" "${_xml_out}"
      WORKING_DIRECTORY "${_msg_autosrc}"
      COMMENT "Generating SWIG interface for ${_payload_name}"
      VERBATIM
    )

    set_property(SOURCE "${_i_out}" PROPERTY CPLUSPLUS ON)
    set_property(SOURCE "${_i_out}" PROPERTY USE_TARGET_INCLUDE_DIRECTORIES TRUE)
    set_property(SOURCE "${_i_out}" PROPERTY SWIG_FLAGS ${_swig_flags})  # LIST, not string

    swig_add_library(
      ${_payload_name}
      LANGUAGE python
      TYPE MODULE
      SOURCES "${_i_out}"
      OUTPUT_DIR "${BSK_OUTPUT_DIR}"
      OUTFILE_DIR "${_auto_root}"
    )

    target_include_directories(${_payload_name} PRIVATE
      ${BSK_INCLUDE_DIRS}
      ${_hdr_dir}
      "${BSK_SDK_INCLUDE_DIR}"
      "${BSK_SDK_INCLUDE_DIR}/Basilisk"
      "${BSK_SDK_INCLUDE_DIR}/Basilisk/architecture"
      "${BSK_SDK_INCLUDE_DIR}/Basilisk/architecture/_GeneralModuleFiles"
      $<$<BOOL:${BSK_SDK_COMPAT_INCLUDE_DIR}>:${BSK_SDK_COMPAT_INCLUDE_DIR}>
      ${Python3_INCLUDE_DIRS}
      ${Python3_NumPy_INCLUDE_DIRS}
    )

    target_link_libraries(${_payload_name} PRIVATE
      Python3::Module
      Eigen3::Eigen
      ${BSK_TARGET_LINK_LIBS}
    )

    set_target_properties(${_payload_name} PROPERTIES
      POSITION_INDEPENDENT_CODE ON
      LIBRARY_OUTPUT_DIRECTORY "${BSK_OUTPUT_DIR}"
      RUNTIME_OUTPUT_DIRECTORY "${BSK_OUTPUT_DIR}"
    )

    list(APPEND _generated_targets ${_payload_name})
  endforeach()

  file(MAKE_DIRECTORY "${BSK_OUTPUT_DIR}")
  set(_init_file "${BSK_OUTPUT_DIR}/__init__.py")
  file(WRITE "${_init_file}" "")
  foreach(_hdr IN LISTS BSK_MSG_HEADERS)
    get_filename_component(_payload_name "${_hdr}" NAME_WE)
    file(APPEND "${_init_file}" "from .${_payload_name} import *\n")
  endforeach()

  if(BSK_OUT_VAR)
    set(${BSK_OUT_VAR} "${_generated_targets}" PARENT_SCOPE)
  endif()
endfunction()
