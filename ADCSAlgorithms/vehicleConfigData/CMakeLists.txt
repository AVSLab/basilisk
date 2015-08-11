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
)

get_property(LIBRARY_BUILD_LIST GLOBAL PROPERTY BUILT_LIB_LIST)
get_property(ALG_LIST GLOBAL PROPERTY ALG_LIB_LIST)

foreach(file ${file_src})
   get_filename_component(FileBase ${file} NAME_WE)
   set_source_files_properties(${file} PROPERTIES CPLUSPLUS OFF)
   set_source_files_properties(${file} PROPERTIES SWIG_FLAGS "-includeall")
   set_source_files_properties(${file} PROPERTIES SWIG_FLAGS "-I${CMAKE_SOURCE_DIR}/SimCode/utilities")
   SWIG_ADD_MODULE(${FileBase} python ${file} ${impl_files})
   SWIG_LINK_LIBRARIES(${FileBase} ${PYTHON_LIBRARIES})
   #foreach(LibFile ${LIBRARY_BUILD_LIST})
   #      SWIG_LINK_LIBRARIES(${FileBase} ${LibFile})
   #endforeach()
   foreach(LibFile ${ALG_LIST})
      SWIG_LINK_LIBRARIES(${FileBase} ${LibFile})
   endforeach()
   set_property(GLOBAL APPEND PROPERTY TOTAL_TARGET_LIST "${FileBase}.py;_${FileBase}.so;")
   set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
         LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/modules")
   set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
         COMPILE_FLAGS "-I${CMAKE_SOURCE_DIR}/ADCSAlgorithms")
   #install(FILES _${FileBase}.so DESTINATION /home/spiggott/modules)
endforeach()
