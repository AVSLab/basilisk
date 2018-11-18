cmake_minimum_required(VERSION 2.8)
if(POLICY CMP0068)
  cmake_policy(SET CMP0068 NEW)
endif()

file(GLOB basilisk_src
    "*.h"
    "*.cpp"
    "*.c"
)


add_library(SimMessaging SHARED ${basilisk_src})
STRING(LENGTH ${CMAKE_SOURCE_DIR} DIR_NAME_START)
MATH(EXPR DIR_NAME_START "${DIR_NAME_START} + 1")
STRING(SUBSTRING ${CMAKE_CURRENT_SOURCE_DIR} ${DIR_NAME_START} -1 DIR_NAME_STRING)
set_target_properties(SimMessaging PROPERTIES FOLDER "${DIR_NAME_STRING}")

if(NOT WIN32)
	target_compile_options(SimMessaging PUBLIC "-fPIC")
endif()
target_include_directories(SimMessaging PUBLIC "${CMAKE_SOURCE_DIR}/simulation")
#target_include_directories(SimMessaging PUBLIC "${CMAKE_SOURCE_DIR}/fswAlgorithms")
set_target_properties(SimMessaging PROPERTIES
LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk")
set_target_properties(SimMessaging PROPERTIES
LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk")
set_target_properties(SimMessaging PROPERTIES
LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Basilisk")

#Have to copy this file everywhere if we're on Windows.  Ugly kludge, but... Windows!
if(WIN32)
    file(GLOB moduleContentsList "${CMAKE_BINARY_DIR}/Basilisk/simulation/*"
	"${CMAKE_BINARY_DIR}/Basilisk/fswAlgorithms/*")	
    foreach(direc ${moduleContentsList})
        if(IS_DIRECTORY ${direc})
            add_custom_command(TARGET SimMessaging POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:SimMessaging> "${direc}/")
        endif()
    endforeach()
endif()
