cmake_minimum_required(VERSION 3.18)
project(bsk_plugin_exponential_atmosphere LANGUAGES CXX)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module NumPy)

# Locate bsk-sdk's CMake config dir from the active Python env
execute_process(
  COMMAND "${Python3_EXECUTABLE}" -c "import bsk_sdk; print(bsk_sdk.cmake_config_dir(), end='')"
  OUTPUT_VARIABLE bsk_sdk_dir
  RESULT_VARIABLE rc
)
if(NOT rc EQUAL 0 OR bsk_sdk_dir STREQUAL "")
  message(FATAL_ERROR "bsk-sdk not found (is it installed in this Python environment?)")
endif()

set(bsk-sdk_DIR "${bsk_sdk_dir}")
find_package(bsk-sdk CONFIG REQUIRED)

# Where scikit-build-core wants wheel outputs
set(PLUGIN_PKG_DIR "${SKBUILD_PLATLIB_DIR}/custom_atm")

# Plugin sources
file(GLOB PLUGIN_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cxx"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

# Build the SWIG module
bsk_add_swig_module(
  TARGET customExponentialAtmosphere
  INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/swig/customExponentialAtmosphere.i"
  SOURCES ${PLUGIN_SOURCES}
  INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src"
  OUTPUT_DIR "${PLUGIN_PKG_DIR}"
)

target_include_directories(customExponentialAtmosphere PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
  "${CMAKE_CURRENT_SOURCE_DIR}/messages"
)

# Generate message bindings into custom_atm/messaging
bsk_generate_messages(
  OUTPUT_DIR "${PLUGIN_PKG_DIR}/messaging"
  MSG_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/messages/CustomAtmStatusMsgPayload.h"
)

# Link against SDK-provided Basilisk minimal libs (if your plugin needs them)
target_link_libraries(customExponentialAtmosphere PRIVATE bsk::runtime_min bsk::arch_min)
