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

FILE(GLOB_RECURSE swig_modules RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.i")
get_property(LIBRARY_BUILD_LIST GLOBAL PROPERTY BUILT_LIB_LIST)
get_property(ALG_LIST GLOBAL PROPERTY ALG_LIB_LIST)

FOREACH(module ${swig_modules})
    get_filename_component(FileBase ${module} NAME_WE)
    get_filename_component(FileDir ${module} DIRECTORY)
    file(GLOB impl_files
	    "${FileDir}/*.cpp"
	    "${FileDir}/*.c"
	    "${FileDir}/*.h"
	    "${FileDir}/../_GeneralModuleFiles/*.cpp"
	    "${FileDir}/../_GeneralModuleFiles/*.h"
	    "${FileDir}/../_GeneralModuleFiles/*.c")
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${FileDir})
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${FileDir})
    set_source_files_properties(${module} PROPERTIES CPLUSPLUS ON)
    #set_source_files_properties(${module} PROPERTIES SWIG_FLAGS "-includeall")
    #set_source_files_properties(${module} PROPERTIES SWIG_FLAGS "-I${CMAKE_SOURCE_DIR}/SimCode/utilities")
	set_property(SOURCE ${module} PROPERTY SWIG_FLAGS "-I${CMAKE_SOURCE_DIR}/SimCode/utilities" "-I${CMAKE_CURRENT_SOURCE_DIR}/${FileDir}/")
	#MESSAGE("${SWIG_FLAGS}")
    #set_source_files_properties(${module} PROPERTIES SWIG_FLAGS "-I${CMAKE_CURRENT_SOURCE_DIR}/${FileDir}/")
	set_source_files_properties( ${swig_generated_file_fullname}
               PROPERTIES COMPILE_FLAGS "-I${CMAKE_SOURCE_DIR}/SimCode")
    set(SWIG_MODULE_${FileBase}_EXTRA_DEPS ${impl_files})
    SWIG_ADD_MODULE(${FileBase} python ${module} ${impl_files})
    SWIG_LINK_LIBRARIES(${FileBase} ${PYTHON_LIBRARIES})
    foreach(LibFile ${LIBRARY_BUILD_LIST})
         SWIG_LINK_LIBRARIES(${FileBase} ${LibFile})
    endforeach()
    foreach(LibFile ${library_dependencies})
         SWIG_LINK_LIBRARIES(${FileBase} ${LibFile})
    endforeach()
    set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES 
	FOLDER "${DIR_NAME_STRING}/${FileDir}")
    set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/modules")
    set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/modules")
    set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/modules")

    set_target_properties(${SWIG_MODULE_${FileBase}_REAL_NAME} PROPERTIES
	    COMPILE_FLAGS "-I${CMAKE_CURRENT_SOURCE_DIR}")
ENDFOREACH()

