cmake_minimum_required(VERSION 2.8)


find_package(SWIG REQUIRED)
include(${SWIG_USE_FILE})
find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_PATH})

set(CMAKE_SWIG_FLAGS "")

FILE(GLOB sub_directories RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/*)

STRING(FIND ${CMAKE_CURRENT_SOURCE_DIR} "/" DIR_NAME_START REVERSE)
MATH(EXPR DIR_NAME_START "${DIR_NAME_START} + 1")
STRING(SUBSTRING ${CMAKE_CURRENT_SOURCE_DIR} ${DIR_NAME_START} -1 DIR_NAME_STRING)

PROJECT("${DIR_NAME_STRING}")

FOREACH(subdir ${sub_directories})

    file(GLOB file_src
        "${subdir}/*.i"
    )
    
    set(TOTAL_FILE_STRING "")
    file(GLOB impl_files
        "${subdir}/*.cpp"
        "${subdir}/*.c"
        "${subdir}/*.h"
		"_GeneralModuleFiles/*.cpp"
		"_GeneralModuleFiles/*.c"
		"_GeneralModuleFiles/*.h"
    )
    
    get_property(LIBRARY_BUILD_LIST GLOBAL PROPERTY BUILT_LIB_LIST)
    get_property(ALG_LIST GLOBAL PROPERTY ALG_LIB_LIST)
    
    foreach(file ${file_src})
       get_filename_component(FileBase ${file} NAME_WE)
       include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${subdir})
       set_source_files_properties(${file} PROPERTIES CPLUSPLUS OFF)
       set_source_files_properties(${file} PROPERTIES SWIG_FLAGS "-includeall")
       set_source_files_properties(${file} PROPERTIES SWIG_FLAGS "-I${CMAKE_SOURCE_DIR}/SimCode/utilities")
       set_source_files_properties(${file} PROPERTIES SWIG_FLAGS "-I${CMAKE_CURRENT_SOURCE_DIR}/${subdir}/")
       set(SWIG_MODULE_${FileBase}_EXTRA_DEPS ${impl_files})
       SWIG_ADD_MODULE(${FileBase} python ${file} ${impl_files})
       SWIG_LINK_LIBRARIES(${FileBase} ${PYTHON_LIBRARIES})
       foreach(LibFile ${ALG_LIST})
          SWIG_LINK_LIBRARIES(${FileBase} ${LibFile})
       endforeach()
       set_property(GLOBAL APPEND PROPERTY TOTAL_TARGET_LIST "${FileBase}.py;_${FileBase}.so;")
       set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES 
           FOLDER "ADCSAlgorithms/${DIR_NAME_STRING}")
       set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
             LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/modules")
       set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
          LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/modules")
       set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
          LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/modules")
       set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
             COMPILE_FLAGS "-I${CMAKE_SOURCE_DIR}/ADCSAlgorithms")
    endforeach()
endforeach()
