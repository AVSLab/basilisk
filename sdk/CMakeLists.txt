cmake_minimum_required(VERSION 3.18)

# We compile both C and C++ sources (linearAlgebra.c etc.)
project(bsk_sdk LANGUAGES C CXX)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# Python wheel layout
set(BSK_SDK_INCLUDE_SRC_DIR       "${CMAKE_CURRENT_SOURCE_DIR}/src/bsk_sdk/include")
set(BSK_ARCH_MIN_SRC_DIR          "${CMAKE_CURRENT_SOURCE_DIR}/src/bsk_sdk/arch_min")
set(BSK_RUNTIME_MIN_SRC_DIR       "${CMAKE_CURRENT_SOURCE_DIR}/src/bsk_sdk/runtime_min")
set(BSK_SDK_COMPAT_INCLUDE_DIR    "${CMAKE_CURRENT_SOURCE_DIR}/src/bsk_sdk/include_compat")
set(BSK_SDK_SWIG_SRC_DIR          "${CMAKE_CURRENT_SOURCE_DIR}/src/bsk_sdk/swig")
set(BSK_SDK_TOOLS_SRC_DIR         "${CMAKE_CURRENT_SOURCE_DIR}/tools")

find_package(Eigen3 CONFIG REQUIRED)

# Header-only SDK target
add_library(bsk_sdk_headers INTERFACE)
add_library(bsk::sdk_headers ALIAS bsk_sdk_headers)

target_compile_features(bsk_sdk_headers INTERFACE cxx_std_17)

target_include_directories(bsk_sdk_headers INTERFACE
  $<BUILD_INTERFACE:${BSK_SDK_INCLUDE_SRC_DIR}>
  $<BUILD_INTERFACE:${BSK_SDK_COMPAT_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/compat>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/Basilisk>
)

# Minimal runtime subset which is a build ALL curated .c/.cpp
file(GLOB BSK_RUNTIME_MIN_SOURCES
  CONFIGURE_DEPENDS
  "${BSK_RUNTIME_MIN_SRC_DIR}/*.c"
  "${BSK_RUNTIME_MIN_SRC_DIR}/*.cpp"
)

add_library(bsk_runtime_min STATIC ${BSK_RUNTIME_MIN_SOURCES})
add_library(bsk::runtime_min ALIAS bsk_runtime_min)

target_compile_features(bsk_runtime_min PUBLIC cxx_std_17)
target_link_libraries(bsk_runtime_min PUBLIC bsk::sdk_headers Eigen3::Eigen)

set_target_properties(bsk_runtime_min PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(bsk_runtime_min PUBLIC
  $<BUILD_INTERFACE:${BSK_SDK_INCLUDE_SRC_DIR}/Basilisk>
  $<BUILD_INTERFACE:${BSK_SDK_COMPAT_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/Basilisk>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/compat>
)

# Minimal Basilisk "arch" subset
file(GLOB BSK_ARCH_MIN_SOURCES
  CONFIGURE_DEPENDS
  "${BSK_ARCH_MIN_SRC_DIR}/*.c"
  "${BSK_ARCH_MIN_SRC_DIR}/*.cpp"
)

add_library(bsk_arch_min STATIC ${BSK_ARCH_MIN_SOURCES})
add_library(bsk::arch_min ALIAS bsk_arch_min)

target_compile_features(bsk_arch_min PUBLIC cxx_std_17)
target_link_libraries(bsk_arch_min PUBLIC bsk::sdk_headers)

set_target_properties(bsk_arch_min PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(bsk_arch_min PUBLIC
  $<BUILD_INTERFACE:${BSK_SDK_INCLUDE_SRC_DIR}/Basilisk>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/Basilisk>
)

target_include_directories(bsk_arch_min PRIVATE
  ${BSK_SDK_INCLUDE_SRC_DIR}/Basilisk/architecture/utilities
  ${BSK_SDK_INCLUDE_SRC_DIR}/Basilisk/architecture/utilities/moduleIdGenerator
)


add_library(bsk_plugin INTERFACE)
add_library(bsk::bsk_plugin ALIAS bsk_plugin)

target_link_libraries(bsk_plugin INTERFACE
  bsk::sdk_headers
  bsk::arch_min
  bsk::runtime_min
)

install(
  DIRECTORY "${BSK_SDK_INCLUDE_SRC_DIR}/"
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

install(
  DIRECTORY "${BSK_SDK_COMPAT_INCLUDE_DIR}/"
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/compat"
)

install(
  DIRECTORY "${BSK_RUNTIME_MIN_SRC_DIR}/"
  DESTINATION "runtime_min"
  FILES_MATCHING
    PATTERN "*.c"
    PATTERN "*.cpp"
    PATTERN "*.h"
)

install(
  DIRECTORY "${BSK_SDK_SWIG_SRC_DIR}/"
  DESTINATION "swig"
  COMPONENT python
)

install(
  DIRECTORY "${BSK_SDK_TOOLS_SRC_DIR}/"
  DESTINATION "tools"
  PATTERN "__pycache__" EXCLUDE
)

install(
  TARGETS bsk_arch_min bsk_runtime_min bsk_sdk_headers bsk_plugin
  EXPORT bsk-sdkTargets
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
)

install(
  EXPORT bsk-sdkTargets
  NAMESPACE bsk::
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/bsk-sdk"
)

install(
  FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/bsk_sdk/__init__.py"
  DESTINATION "."
)

configure_package_config_file(
  cmake/bsk-sdkConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/bsk-sdkConfig.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/bsk-sdk"
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/bsk-sdkConfigVersion.cmake"
  VERSION 1.0.0
  COMPATIBILITY SameMajorVersion
)

install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/bsk-sdkConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/bsk-sdkConfigVersion.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/bsk-sdk"
)

install(
  FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/bsk_add_swig_module.cmake"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/bsk_generate_messages.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/bsk-sdk"
)
