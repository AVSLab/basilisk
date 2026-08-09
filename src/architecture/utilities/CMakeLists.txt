
bsk_get_directory_source_files(basilisk_src "${CMAKE_CURRENT_SOURCE_DIR}")
list(FILTER basilisk_src INCLUDE REGEX "\\.(c|cpp|h|hpp|i)$")
bsk_get_directory_source_files(
  moduleId "${CMAKE_CURRENT_SOURCE_DIR}/moduleIdGenerator")
list(FILTER moduleId INCLUDE REGEX "\\.(c|cpp|h|hpp|i)$")

foreach(source_file IN LISTS basilisk_src)
  if(source_file MATCHES "\\.(c|cpp)$")
    bsk_claim_production_source("${source_file}" "library ArchitectureUtilities")
  endif()
endforeach()
foreach(source_file IN LISTS moduleId)
  if(source_file MATCHES "\\.(c|cpp)$")
    bsk_claim_production_source("${source_file}" "library ModuleIdGenerator")
  endif()
endforeach()

add_library(ArchitectureUtilities STATIC ${basilisk_src})
# Find and link cfitsio
find_package(cfitsio REQUIRED)
target_link_libraries(ArchitectureUtilities PUBLIC cfitsio::cfitsio)

add_library(ModuleIdGenerator SHARED ${moduleId})
string(LENGTH ${CMAKE_SOURCE_DIR} DIR_NAME_START)
math(EXPR DIR_NAME_START "${DIR_NAME_START} + 1")
string(SUBSTRING ${CMAKE_CURRENT_SOURCE_DIR} ${DIR_NAME_START} -1 DIR_NAME_STRING)
set_target_properties(ArchitectureUtilities PROPERTIES FOLDER "${DIR_NAME_STRING}")
set_target_properties(ModuleIdGenerator PROPERTIES FOLDER "${DIR_NAME_STRING}")

if(NOT WIN32)
  target_compile_options(ArchitectureUtilities PUBLIC "-fPIC")
  target_compile_options(ModuleIdGenerator PUBLIC "-fPIC")
endif()

set_target_properties(ArchitectureUtilities PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk")
set_target_properties(ArchitectureUtilities PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk")
set_target_properties(ArchitectureUtilities PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Basilisk")

set_target_properties(ModuleIdGenerator PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk")
set_target_properties(ModuleIdGenerator PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Basilisk")
set_target_properties(ModuleIdGenerator PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Basilisk")

if(WIN32)
  add_custom_command(
    TARGET ModuleIdGenerator
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:ModuleIdGenerator> "${CMAKE_BINARY_DIR}/Basilisk/")
endif()
