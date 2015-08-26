cmake_minimum_required(VERSION 2.8)

file(GLOB basilisk_src
    "*.h"
    "*.cpp"
    "*.c"
)
get_property(LIBRARY_BUILD_LIST GLOBAL PROPERTY BUILT_LIB_LIST)
add_library(AlgorithmMessaging STATIC ${basilisk_src})
target_link_libraries(AlgorithmMessaging ${LIBRARY_BUILD_LIST})
target_compile_options(AlgorithmMessaging PUBLIC "-fPIC")
target_include_directories(AlgorithmMessaging PUBLIC ${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/ADCSAlgorithms)
set_target_properties(AlgorithmMessaging PROPERTIES
LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/modules")
set_target_properties(AlgorithmMessaging PROPERTIES
LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/modules")
set_target_properties(AlgorithmMessaging PROPERTIES
LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/modules")

