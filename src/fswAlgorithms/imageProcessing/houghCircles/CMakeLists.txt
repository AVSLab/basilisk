cmake_minimum_required(VERSION 2.8)

file(GLOB basilisk_src
    "*.h"
    "*.c"
    "*.cpp"
    "*.i"
)

add_library(HoughCircles STATIC ${basilisk_src})

STRING(LENGTH ${CMAKE_SOURCE_DIR} DIR_NAME_START)
MATH(EXPR DIR_NAME_START "${DIR_NAME_START} + 1")
STRING(SUBSTRING ${CMAKE_CURRENT_SOURCE_DIR} ${DIR_NAME_START} -1 DIR_NAME_STRING)
set_target_properties(HoughCircles PROPERTIES
FOLDER "${DIR_NAME_STRING}")

if(NOT WIN32)
	target_compile_options(HoughCircles PUBLIC "-fPIC")
endif()
target_include_directories(HoughCircles PUBLIC "${CMAKE_SOURCE_DIR}")
target_include_directories(HoughCircles PUBLIC "${CMAKE_SOURCE_DIR}/fswAlgorithms")
set_target_properties(HoughCircles PROPERTIES
ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk")
set_target_properties(HoughCircles PROPERTIES
ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk")
set_target_properties(HoughCircles PROPERTIES
ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Basilisk")