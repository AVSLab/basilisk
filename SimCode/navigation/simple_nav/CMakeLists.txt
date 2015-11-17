cmake_minimum_required(VERSION 2.8)


find_package(SWIG REQUIRED)
include(${SWIG_USE_FILE})

find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_PATH})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(CMAKE_SWIG_FLAGS "")


file(GLOB file_src
    "*.i"
)

set(TOTAL_FILE_STRING "")
file(GLOB impl_files
    "*.cpp"
    "*.c"
    "*.h"
)

get_property(LIBRARY_BUILD_LIST GLOBAL PROPERTY BUILT_LIB_LIST)

foreach(file ${file_src})
   get_filename_component(FileBase ${file} NAME_WE)
   set_source_files_properties(${file} PROPERTIES CPLUSPLUS ON)
   set_source_files_properties(${file} PROPERTIES SWIG_FLAGS "-includeall")
   set_source_files_properties(${file} PROPERTIES SWIG_FLAGS "-I${CMAKE_SOURCE_DIR}/SimCode/utilities")
   set_source_files_properties( ${swig_generated_file_fullname}
               PROPERTIES COMPILE_FLAGS "-I${CMAKE_SOURCE_DIR}/SimCode")
   set(SWIG_MODULE_${FileBase}_EXTRA_DEPS ${impl_files})
   SWIG_ADD_MODULE(${FileBase} python ${file} ${impl_files})
   SWIG_LINK_LIBRARIES(${FileBase} ${PYTHON_LIBRARIES})
   foreach(LibFile ${LIBRARY_BUILD_LIST})
         SWIG_LINK_LIBRARIES(${FileBase} ${LibFile})
   endforeach()
   foreach(LibFile ${library_dependencies})
      SWIG_LINK_LIBRARIES(${FileBase} ${LibFile})
   endforeach()
   set_property(GLOBAL APPEND PROPERTY TOTAL_TARGET_LIST "${FileBase}.py;_${FileBase}.so;")
   set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
      LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/modules")
   set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
      LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/modules")
   set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
      LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/modules")

endforeach()
