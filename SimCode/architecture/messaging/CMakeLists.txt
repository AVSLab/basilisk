cmake_minimum_required(VERSION 2.8)

file(GLOB basilisk_src
    "*.h"
    "*.cpp"
    "*.c"
)

add_library(SimMessaging SHARED ${basilisk_src})
target_compile_options(SimMessaging PUBLIC "-fPIC")
target_include_directories(SimMessaging PUBLIC "${CMAKE_SOURCE_DIR}/SimCode")
set_target_properties(SimMessaging PROPERTIES
LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/modules")
set_target_properties(SimMessaging PROPERTIES
LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/modules")
set_target_properties(SimMessaging PROPERTIES
LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/modules")
