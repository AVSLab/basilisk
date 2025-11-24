cmake_minimum_required(VERSION 3.18)
project(bsk_plugin_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)

execute_process(
  COMMAND "${Python3_EXECUTABLE}" -c "import bsk_sdk; print('\\n'.join(bsk_sdk.include_dirs()), end='')"
  OUTPUT_VARIABLE BSK_SDK_INCLUDE_OUTPUT
  RESULT_VARIABLE BSK_SDK_RESULT
)
if(NOT BSK_SDK_RESULT EQUAL 0)
  message(FATAL_ERROR "Failed to locate bsk-sdk include directories. Is the package installed?")
endif()
string(REPLACE "\r" "" BSK_SDK_INCLUDE_OUTPUT "${BSK_SDK_INCLUDE_OUTPUT}")
string(STRIP "${BSK_SDK_INCLUDE_OUTPUT}" BSK_SDK_INCLUDE_OUTPUT)
string(REPLACE "\n" ";" BSK_SDK_INCLUDE_DIRS "${BSK_SDK_INCLUDE_OUTPUT}")
list(GET BSK_SDK_INCLUDE_DIRS 1 BSK_SDK_BASILISK_INCLUDE)

Python3_add_library(_custom_cpp MODULE ExternalModules/CustomCppModule/custom_cpp_module.cpp WITH_SOABI)
target_compile_features(_custom_cpp PRIVATE cxx_std_17)
target_include_directories(
  _custom_cpp
  PRIVATE
    ${BSK_SDK_INCLUDE_DIRS}
)
target_link_libraries(_custom_cpp PRIVATE Python3::Module)

set(BSK_ARCHITECTURE_SOURCES
  "${BSK_SDK_BASILISK_INCLUDE}/architecture/_GeneralModuleFiles/sys_model.cpp"
  "${BSK_SDK_BASILISK_INCLUDE}/architecture/utilities/bskLogging.cpp"
  "${BSK_SDK_BASILISK_INCLUDE}/architecture/utilities/moduleIdGenerator/moduleIdGenerator.cpp"
)

target_sources(_custom_cpp PRIVATE ${BSK_ARCHITECTURE_SOURCES})

install(
  TARGETS _custom_cpp
  DESTINATION Basilisk/ExternalModules
  COMPONENT extensions
)

install(
  DIRECTORY src/python/
  DESTINATION .
  COMPONENT python
)

install(
  DIRECTORY msgPayloadDefC msgPayloadDefCpp
  DESTINATION Basilisk
  COMPONENT python
)

if(APPLE)
  # 1) Don't hide symbols for this target (avoid losing PyInit export)
  set_target_properties(_custom_cpp PROPERTIES
    C_VISIBILITY_PRESET default
    CXX_VISIBILITY_PRESET default
    VISIBILITY_INLINES_HIDDEN OFF
  )

endif()
