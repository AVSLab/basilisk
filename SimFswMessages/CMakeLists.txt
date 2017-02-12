
project("${DIR_NAME_STRING}")

file(GLOB inter_files
            "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
            "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
            "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
     )


set_property(GLOBAL PROPERTY SYS_INTER_FILES "${inter_files}")

add_library(SimFswMessages_lib INTERFACE)
add_dependencies(SimFswMessages_lib ${inter_files})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_custom_target(SimFswMessages SOURCES ${inter_files})
