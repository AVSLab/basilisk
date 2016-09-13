cmake_minimum_required(VERSION 2.8)

file(GLOB basilisk_src
    "*.h"
    "*.cpp"
    "*.c"
)


add_library(SimMessaging SHARED ${basilisk_src})
STRING(LENGTH ${CMAKE_SOURCE_DIR} DIR_NAME_START)
MATH(EXPR DIR_NAME_START "${DIR_NAME_START} + 1")
STRING(SUBSTRING ${CMAKE_CURRENT_SOURCE_DIR} ${DIR_NAME_START} -1 DIR_NAME_STRING)
set_target_properties(SimMessaging PROPERTIES
    FOLDER "${DIR_NAME_STRING}")

target_compile_options(SimMessaging PUBLIC "-fPIC")
target_include_directories(SimMessaging PUBLIC "${CMAKE_SOURCE_DIR}/SimCode")
set_target_properties(SimMessaging PROPERTIES
LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/modules")
set_target_properties(SimMessaging PROPERTIES
LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/modules")
set_target_properties(SimMessaging PROPERTIES
LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/modules")

#Have to copy this file everywhere if we're on Windows.  Ugly kludge, but... Windows!
if(WIN32)
    file(GLOB moduleContentsList  ${CMAKE_SOURCE_DIR}/modules/*)
    foreach(direc ${moduleContentsList})
        if(IS_DIRECTORY ${direc})
            message(${direc})
            add_custom_command(TARGET SimMessaging POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:SimMessaging> "${direc}/")
        endif()
    endforeach()
endif()
