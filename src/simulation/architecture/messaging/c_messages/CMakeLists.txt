cmake_minimum_required(VERSION 3.14)

file(GLOB basilisk_src
    "*.h"
    "*.cpp"
    "*.c"
)

get_property(LIBRARY_BUILD_LIST GLOBAL PROPERTY BUILT_LIB_LIST)
add_library(CMessages STATIC ${basilisk_src})
STRING(LENGTH ${CMAKE_SOURCE_DIR} DIR_NAME_START)
MATH(EXPR DIR_NAME_START "${DIR_NAME_START} + 1")
STRING(SUBSTRING ${CMAKE_CURRENT_SOURCE_DIR} ${DIR_NAME_START} -1 DIR_NAME_STRING)
set_target_properties(CMessages PROPERTIES FOLDER "${DIR_NAME_STRING}")

target_link_libraries(CMessages ${LIBRARY_BUILD_LIST})
if(NOT WIN32)
	target_compile_options(CMessages PUBLIC "-fPIC")
endif()
target_include_directories(CMessages PUBLIC "${CMAKE_SOURCE_DIR}/simulation")
target_include_directories(CMessages PUBLIC ${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/fswAlgorithms)
set_target_properties(CMessages PROPERTIES
ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk")
set_target_properties(CMessages PROPERTIES
ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk")
set_target_properties(CMessages PROPERTIES
ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Basilisk")
