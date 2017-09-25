cmake_minimum_required(VERSION 2.8)

file(GLOB basilisk_src
    "*.h"
    "*.cpp"
    "*.c"
)
get_property(LIBRARY_BUILD_LIST GLOBAL PROPERTY BUILT_LIB_LIST)
add_library(AlgorithmMessaging STATIC ${basilisk_src})

STRING(LENGTH ${CMAKE_SOURCE_DIR} DIR_NAME_START)
MATH(EXPR DIR_NAME_START "${DIR_NAME_START} + 1")
STRING(SUBSTRING ${CMAKE_CURRENT_SOURCE_DIR} ${DIR_NAME_START} -1 DIR_NAME_STRING)
set_target_properties(AlgorithmMessaging PROPERTIES
FOLDER "${DIR_NAME_STRING}")

target_link_libraries(AlgorithmMessaging ${LIBRARY_BUILD_LIST})
target_compile_options(AlgorithmMessaging PUBLIC "-fPIC")
target_include_directories(AlgorithmMessaging PUBLIC ${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/FswAlgorithms)
set_target_properties(AlgorithmMessaging PROPERTIES
ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/package/modules")
set_target_properties(AlgorithmMessaging PROPERTIES
ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/package/modules")
set_target_properties(AlgorithmMessaging PROPERTIES
ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/package/modules")

